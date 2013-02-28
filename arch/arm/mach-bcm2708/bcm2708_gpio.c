/*
 *  linux/arch/arm/mach-bcm2708/bcm2708_gpio.c
 *
 *  Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/platform.h>
#include <mach/gpio.h>

#define BCM_GPIO_DRIVER_NAME "bcm2708_gpio"
#define DRIVER_NAME BCM_GPIO_DRIVER_NAME
#define BCM_GPIO_USE_IRQ 1

#define GPIOFSEL(x)  (0x00+(x)*4)
#define GPIOSET(x)   (0x1c+(x)*4)
#define GPIOCLR(x)   (0x28+(x)*4)
#define GPIOLEV(x)   (0x34+(x)*4)
#define GPIOEDS(x)   (0x40+(x)*4)
#define GPIOREN(x)   (0x4c+(x)*4)
#define GPIOFEN(x)   (0x58+(x)*4)
#define GPIOHEN(x)   (0x64+(x)*4)
#define GPIOLEN(x)   (0x70+(x)*4)
#define GPIOAREN(x)  (0x7c+(x)*4)
#define GPIOAFEN(x)  (0x88+(x)*4)
#define GPIOUD(x)    (0x94+(x)*4)
#define GPIOUDCLK(x) (0x98+(x)*4)

enum { GPIO_FSEL_INPUT, GPIO_FSEL_OUTPUT,
	GPIO_FSEL_ALT5, GPIO_FSEL_ALT_4,
	GPIO_FSEL_ALT0, GPIO_FSEL_ALT1,
	GPIO_FSEL_ALT2, GPIO_FSEL_ALT3,
};

	/* Each of the two spinlocks protects a different set of hardware
	 * regiters and data structurs. This decouples the code of the IRQ from
	 * the GPIO code. This also makes the case of a GPIO routine call from
	 * the IRQ code simpler.
	 */
static DEFINE_SPINLOCK(lock);	/* GPIO registers */

struct bcm2708_gpio {
	struct list_head list;
	void __iomem *base;
	struct gpio_chip gc;
	unsigned long rising;
	unsigned long falling;
};


/*  JWJ -- types and values for GPIO timing  ------------------------------  */

static struct kset *gpioks;

struct gpio_time {
	struct kobject kobj;
	unsigned int time[2]; /* newer sample in 0, older in 1 */
	int pin; /* helpful in gpio_release() */
};
#define to_gpio_time(x) container_of(x, struct gpio_time, kobj)

struct gpio_attribute {
	struct attribute attr;
	ssize_t (*show)(struct gpio_time *foo, struct gpio_attribute *attr, char *buf);
	ssize_t (*store)(struct gpio_time *foo, struct gpio_attribute *attr,
		const char *buf, size_t count);
};
#define to_gpio_attr(x) container_of(x, struct gpio_attribute, attr)

static struct gpio_time *gptimes[BCM_NR_GPIOS];

struct period_attribute {
	struct attribute attr;
	ssize_t (*show)(char *buf);
	ssize_t (*store)(const char *buf, size_t count);
};
#define to_period_attr(x) container_of(x, struct period_attribute, attr)

/* timer value register */
#define TVAL        __io_address(ARMCTRL_TIMER0_1_BASE + 0x20)

/*  JWJ -- end of GPIO timing stuffs  -------------------------------------  */


static int bcm2708_set_function(struct gpio_chip *gc, unsigned offset,
				int function)
{
	struct bcm2708_gpio *gpio = container_of(gc, struct bcm2708_gpio, gc);
	unsigned long flags;
	unsigned gpiodir;
	unsigned gpio_bank = offset / 10;
	unsigned gpio_field_offset = (offset - 10 * gpio_bank) * 3;

//printk(KERN_ERR DRIVER_NAME ": bcm2708_gpio_set_function %p (%d,%d)\n", gc, offset, function);
	if (offset >= BCM_NR_GPIOS)
		return -EINVAL;

	spin_lock_irqsave(&lock, flags);

	gpiodir = readl(gpio->base + GPIOFSEL(gpio_bank));
	gpiodir &= ~(7 << gpio_field_offset);
	gpiodir |= function << gpio_field_offset;
	writel(gpiodir, gpio->base + GPIOFSEL(gpio_bank));
	spin_unlock_irqrestore(&lock, flags);
	gpiodir = readl(gpio->base + GPIOFSEL(gpio_bank));

	return 0;
}

static int bcm2708_gpio_dir_in(struct gpio_chip *gc, unsigned offset)
{
	return bcm2708_set_function(gc, offset, GPIO_FSEL_INPUT);
}

static void bcm2708_gpio_set(struct gpio_chip *gc, unsigned offset, int value);
static int bcm2708_gpio_dir_out(struct gpio_chip *gc, unsigned offset,
				int value)
{
	int ret;
	ret = bcm2708_set_function(gc, offset, GPIO_FSEL_OUTPUT);
	if (ret >= 0)
		bcm2708_gpio_set(gc, offset, value);
	return ret;
}

static int bcm2708_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct bcm2708_gpio *gpio = container_of(gc, struct bcm2708_gpio, gc);
	unsigned gpio_bank = offset / 32;
	unsigned gpio_field_offset = (offset - 32 * gpio_bank);
	unsigned lev;

	if (offset >= BCM_NR_GPIOS)
		return 0;
	lev = readl(gpio->base + GPIOLEV(gpio_bank));
//printk(KERN_ERR DRIVER_NAME ": bcm2708_gpio_get %p (%d)=%d\n", gc, offset, 0x1 & (lev>>gpio_field_offset));
	return 0x1 & (lev >> gpio_field_offset);
}

static void bcm2708_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct gpio_time *gpt;
	struct bcm2708_gpio *gpio = container_of(gc, struct bcm2708_gpio, gc);
	unsigned gpio_bank = offset / 32;
	unsigned gpio_field_offset = (offset - 32 * gpio_bank);
	unsigned int t;
//printk(KERN_ERR DRIVER_NAME ": bcm2708_gpio_set %p (%d=%d)\n", gc, offset, value);
	if (offset >= BCM_NR_GPIOS)
		return;
	/* JWJ -- get time of change before output */
	t = readl(TVAL);
	if (value)
		writel(1 << gpio_field_offset, gpio->base + GPIOSET(gpio_bank));
	else
		writel(1 << gpio_field_offset, gpio->base + GPIOCLR(gpio_bank));
	/* JWJ -- record time of change; may run after an interrupt caused by change */
	gpt = gptimes[offset];
	gpt->time[1] = gpt->time[0];
	gpt->time[0] = t;
	/* kobject_uevent(&gpt->kobj, KOBJ_CHANGE); */
}

/*************************************************************************************************************************
 * bcm2708 GPIO IRQ
 */

#if BCM_GPIO_USE_IRQ

#define IRQ_TO_GPIO(x)	irq_to_gpio(x)

static int bcm2708_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	return gpio_to_irq(gpio);
}

static int bcm2708_gpio_irq_set_type(struct irq_data *d, unsigned type)
{
	unsigned irq = d->irq;
	struct bcm2708_gpio *gpio = irq_get_chip_data(irq);

	if (type & ~(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	if (type & IRQ_TYPE_EDGE_RISING) {
		gpio->rising |= (1 << IRQ_TO_GPIO(irq));
	} else {
		gpio->rising &= ~(1 << IRQ_TO_GPIO(irq));
	}

	if (type & IRQ_TYPE_EDGE_FALLING) {
		gpio->falling |= (1 << IRQ_TO_GPIO(irq));
	} else {
		gpio->falling &= ~(1 << IRQ_TO_GPIO(irq));
	}
	return 0;
}

static void bcm2708_gpio_irq_mask(struct irq_data *d)
{
	unsigned irq = d->irq;
	struct bcm2708_gpio *gpio = irq_get_chip_data(irq);
	unsigned gn = IRQ_TO_GPIO(irq);
	unsigned gb = gn / 32;
	unsigned long rising = readl(gpio->base + GPIOREN(gb));
	unsigned long falling = readl(gpio->base + GPIOFEN(gb));

	writel(rising & ~(1 << gn), gpio->base + GPIOREN(gb));
	writel(falling & ~(1 << gn), gpio->base + GPIOFEN(gb));
}

static void bcm2708_gpio_irq_unmask(struct irq_data *d)
{
	unsigned irq = d->irq;
	struct bcm2708_gpio *gpio = irq_get_chip_data(irq);
	unsigned gn = IRQ_TO_GPIO(irq);
	unsigned gb = gn / 32;
	unsigned long rising = readl(gpio->base + GPIOREN(gb));
	unsigned long falling = readl(gpio->base + GPIOFEN(gb));

	gn = gn % 32;

	writel(1 << gn, gpio->base + GPIOEDS(gb));

	if (gpio->rising & (1 << gn)) {
		writel(rising | (1 << gn), gpio->base + GPIOREN(gb));
	} else {
		writel(rising & ~(1 << gn), gpio->base + GPIOREN(gb));
	}

	if (gpio->falling & (1 << gn)) {
		writel(falling | (1 << gn), gpio->base + GPIOFEN(gb));
	} else {
		writel(falling & ~(1 << gn), gpio->base + GPIOFEN(gb));
	}
}

static struct irq_chip bcm2708_irqchip = {
	.name = "GPIO",
	.irq_enable = bcm2708_gpio_irq_unmask,
	.irq_disable = bcm2708_gpio_irq_mask,
	.irq_unmask = bcm2708_gpio_irq_unmask,
	.irq_mask = bcm2708_gpio_irq_mask,
	.irq_set_type = bcm2708_gpio_irq_set_type,
};

/* sending uevents for changes doesn't seem to be required
static unsigned long gpio_uevent_flags[2];

static void send_uevent(struct work_struct *ws) {
	struct gpio_time *gpt;
	unsigned long flags;
	int bank, i;
	for (bank = 0; bank <= 1; bank++) {
		flags = gpio_uevent_flags[bank];
		gpio_uevent_flags[bank] = 0;
		for_each_set_bit(i, &flags, 32) {
			gpt = gptimes[i + bank * 32];
			kobject_uevent(&gpt->kobj, KOBJ_CHANGE);
		}
	}
}

DECLARE_WORK(send_uevent_work, send_uevent);
*/

static irqreturn_t bcm2708_gpio_interrupt(int irq, void *dev_id)
{
	unsigned long edsr;
	unsigned bank;
	int i;
	unsigned gpio;
	/* JWJ -- obtain time of change */
	unsigned t = readl(TVAL);
	struct gpio_time *gpt;
	for (bank = 0; bank <= 1; bank++) {
		edsr = readl(__io_address(GPIO_BASE) + GPIOEDS(bank));
		/* gpio_uevent_flags[bank] |= edsr; */
		for_each_set_bit(i, &edsr, 32) {
			gpio = i + bank * 32;
			/* JWJ -- record time of change */
			gpt = gptimes[gpio];
			gpt->time[1] = gpt->time[0];
			gpt->time[0] = t;
			generic_handle_irq(gpio_to_irq(gpio));
		}
		writel(0xffffffff, __io_address(GPIO_BASE) + GPIOEDS(bank));
	}
	/* schedule_work(&send_uevent_work); */
	return IRQ_HANDLED;
}

static struct irqaction bcm2708_gpio_irq = {
	.name = "BCM2708 GPIO catchall handler",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,  /* | IRQF_SHARED, */
	.handler = bcm2708_gpio_interrupt,
};

static void bcm2708_gpio_irq_init(struct bcm2708_gpio *ucb)
{
	unsigned irq;

	ucb->gc.to_irq = bcm2708_gpio_to_irq;

	for (irq = GPIO_IRQ_START; irq < (GPIO_IRQ_START + GPIO_IRQS); irq++) {
		irq_set_chip_data(irq, ucb);
		irq_set_chip(irq, &bcm2708_irqchip);
		set_irq_flags(irq, IRQF_VALID);
	}
	setup_irq(IRQ_GPIO3, &bcm2708_gpio_irq);
}

#else

static void bcm2708_gpio_irq_init(struct bcm2708_gpio *ucb)
{
}

#endif /* #if BCM_GPIO_USE_IRQ ***************************************************************************************************************** */

static struct kobject *periodobj;  /*, *rctimeobj; */

/*
 * The default show function that must be passed to sysfs.  This will be
 * called by sysfs for whenever a show function is called by the user on a
 * sysfs file associated with the kobjects we have registered.  We need to
 * transpose back from a "default" kobject to our custom struct foo_obj and
 * then call the show function for that specific object.
 */
static ssize_t gpio_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct gpio_attribute *attribute;
	struct gpio_time *foo;

	attribute = to_gpio_attr(attr);
	foo = to_gpio_time(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(foo, attribute, buf);
}

/*
 * Just like the default show function above, but this one is for when the
 * sysfs "store" is requested (when a value is written to a file.)
 */
static ssize_t gpio_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct gpio_attribute *attribute;
	struct gpio_time *foo;

	attribute = to_gpio_attr(attr);
	foo = to_gpio_time(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(foo, attribute, buf, len);
}

/* Our custom sysfs_ops that we will associate with our ktype later on */
static const struct sysfs_ops gpio_sysfs_ops = {
	.show = gpio_attr_show,
	.store = gpio_attr_store,
};

/* for timer period */

/* timer control register */
#define TCTRL               __io_address(ARMCTRL_TIMER0_1_BASE + 8)
/* frequency divisor, 1 to 256 inclusive */
#define TDIV(ctrl)          (((ctrl >> 16) & 0xFF) + 1)
/* timer enable bit */
#define TENB(ctrl)          (ctrl & 0x200)
/* set timer frequency divisor and enable timer */
#define TSETDIV(ctrl, scl)  ((ctrl & 0x1FF) | ((scl & 0xFF) << 16) | 0x200)

static ssize_t period_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct period_attribute *attribute = to_period_attr(attr);
	if (!attribute->show)
		return -EIO;
	return attribute->show(buf);
}

static ssize_t period_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct period_attribute *attribute = to_period_attr(attr);
	if (!attribute->store)
		return -EIO;
	return attribute->store(buf, len);
}

static const struct sysfs_ops period_sysfs_ops = {
	.show = period_attr_show,
	.store = period_attr_store,
};

static ssize_t period_show(char *buf) {
	int p;
	p = readl(TCTRL);
	if (!TENB(p)) {
		return sprintf(buf, "0\n");
	}
	return sprintf(buf, "%d\n", TDIV(p) * 4);
}

static ssize_t period_store(const char *buf, size_t count) {
	int p, c;
	sscanf(buf, "%d", &p);
	if ((p > 3) && (p < 1025)) {
		p = (p >> 2) - 1;
		c = readl(TCTRL);
		c = TSETDIV(c, p);
		writel(c, TCTRL);
	}
	return count;
}

static struct period_attribute tper_attribute =
	__ATTR(period, 0666, period_show, period_store);

static struct attribute *period_default_attrs[] = {
	&tper_attribute.attr, NULL
};

/* for GPIO times */

static ssize_t gpio_time_show(struct gpio_time *gpt, struct gpio_attribute *attr,
		      char *buf)
{
	return sprintf(buf, "%u %u\n", gpt->time[0], gpt->time[1]);
}

static struct gpio_attribute gp_attribute =
	__ATTR_RO(gpio_time);

static struct attribute *gpio_default_attrs[] = {
	&gp_attribute.attr, NULL
};

static void gpio_release(struct kobject *kobj) {
	struct gpio_time *gpt = to_gpio_time(kobj);
	gptimes[gpt->pin] = NULL;
	kfree(gpt);
}

static void period_release(struct kobject *kobj) {
	kfree(kobj);
}

/* ktypes */

static struct kobj_type gpio_ktype = {
	.sysfs_ops = &gpio_sysfs_ops,
	.release = gpio_release,
	.default_attrs = gpio_default_attrs,
};

static struct kobj_type period_ktype = {
	.sysfs_ops = &period_sysfs_ops,
	.release = period_release,
	.default_attrs = period_default_attrs,
};
/*
static struct kobj_type rctime_ktype = {
	.sysfs_ops = &foo_sysfs_ops,
	.release = foo_release,
	.default_attrs = foo_default_attrs,
};
*/

static void destroy_kobjs(void) {
	int n;
	for (n = BCM_NR_GPIOS; n; n--) {
		if (gptimes[n-1]) {
			kobject_put(&(gptimes[n-1]->kobj));
		}
	}
	/* timer period */
	kobject_put(periodobj);
	/* RC timing */
}

static int bcm2708_gpio_probe(struct platform_device *dev)
{
	struct bcm2708_gpio *ucb;
	struct resource *res;
	int err = 0, n;

	printk(KERN_INFO DRIVER_NAME ": bcm2708_gpio_probe %p\n", dev);

	ucb = kzalloc(sizeof(*ucb), GFP_KERNEL);
	if (NULL == ucb) {
		printk(KERN_ERR DRIVER_NAME ": failed to allocate "
		       "mailbox memory\n");
		err = -ENOMEM;
		goto err;
	}

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);

	platform_set_drvdata(dev, ucb);
	ucb->base = __io_address(GPIO_BASE);

	ucb->gc.label = "bcm2708_gpio";
	ucb->gc.base = 0;
	ucb->gc.ngpio = BCM_NR_GPIOS;
	ucb->gc.owner = THIS_MODULE;

	ucb->gc.direction_input = bcm2708_gpio_dir_in;
	ucb->gc.direction_output = bcm2708_gpio_dir_out;
	ucb->gc.get = bcm2708_gpio_get;
	ucb->gc.set = bcm2708_gpio_set;
	ucb->gc.can_sleep = 0;

	bcm2708_gpio_irq_init(ucb);

	err = gpiochip_add(&ucb->gc);
	if (err) {
		printk(KERN_ERR DRIVER_NAME ": failed on gpiochip_add()\n");
		goto err;
	}
	
	/* JWJ -- GPIO timing sysfs interface */
	
	gpioks = kset_create_and_add("gpio_time", NULL, kernel_kobj);
	// seems to cause kernel warning:
	//   kobject_uevent_env: attempted to send uevent without kset!
	if (!gpioks) {
		printk(KERN_ERR "GPT: kset_create_and_add() failed\n");
		err = -ENOMEM;
		goto err;
	}
	/* GPIOs */
	for (n = BCM_NR_GPIOS; n; n--) {
		struct gpio_time *gpt;
		gpt = kzalloc(sizeof(*gpt), GFP_KERNEL);
		if (!gpt) {
			printk(KERN_ERR "GPT: kzalloc() failed on GPIO %d\n", n);
			goto gperr;
		}
		gptimes[(gpt->pin = n-1)] = gpt;
		gpt->kobj.kset = gpioks;
		if (kobject_init_and_add(&gpt->kobj, &gpio_ktype, NULL, "time%02d", n)) {
			printk(KERN_ERR "GPT: kobject_init_and_add() failed on GPIO %d\n", n);
			goto gperr;
		}
		kobject_uevent(&gpt->kobj, KOBJ_ADD);
	}
	/* timer period */
	if (!(periodobj = kzalloc(sizeof(*periodobj), GFP_KERNEL))) {
		printk(KERN_ERR "GPT: kzalloc() failed on timer");
		goto gperr;
	}
	periodobj->kset = gpioks;
	if (kobject_init_and_add(periodobj, &period_ktype, NULL, "%s", "period")) {
		printk(KERN_ERR "GPT: kobject_init_and_add() failed on timer\n");
		goto gperr;
	}
	kobject_uevent(periodobj, KOBJ_ADD);
	/* RC timing -- root level
	   not yet implemented
	if (!(rctimeobj = kzalloc(sizeof(gpio_time), GFP_KERNEL))) {
		goto gperr;
	}
	periodobj->kobj.kest = gpioks;
	if (kobject_init_and_add(&rctimeobj, &gpio_ktype, NULL, "time%02n", n)) {
		goto gperr;
	}
	// RC timing -- params
	*/
	printk(KERN_INFO "GPIO init done\n");
	return 0;
	
gperr:
	destroy_kobjs();
	err = -ENOMEM;
err:
	return err;

}

static int bcm2708_gpio_remove(struct platform_device *dev)
{
	int err = 0;
	struct bcm2708_gpio *ucb = platform_get_drvdata(dev);

	printk(KERN_ERR DRIVER_NAME ": bcm2708_gpio_remove %p\n", dev);
	
	destroy_kobjs();
	kset_unregister(gpioks);

	err = gpiochip_remove(&ucb->gc);

	platform_set_drvdata(dev, NULL);
	kfree(ucb);

	return err;
}

static struct platform_driver bcm2708_gpio_driver = {
	.probe = bcm2708_gpio_probe,
	.remove = bcm2708_gpio_remove,
	.driver = {
		   .name = "bcm2708_gpio",
		   /* .owner = THIS_MODULE, */
	},
};

static int __init bcm2708_gpio_init(void)
{
	return platform_driver_register(&bcm2708_gpio_driver);
}

static void __exit bcm2708_gpio_exit(void)
{
	platform_driver_unregister(&bcm2708_gpio_driver);
}

module_init(bcm2708_gpio_init);
module_exit(bcm2708_gpio_exit);

MODULE_DESCRIPTION("Broadcom BCM2708 GPIO driver");
MODULE_LICENSE("GPL");
/* MODULE_ALIAS("platform:bcm2708_gpio"); */
