// SPDX-License-Identifier: GPL-2.0-only
/*
 * A hardware timer implementation for AM335x usable by user-space applications.
 *
 * Copyright (C) 2020 Berrux Kana <Leuze electronic>
 * Written by Berrux Kana < bkana@leuze.de >
 *
 * Copyright (C) 2020 Martin Kaul <private>
 * Reformat & enhanced by Martin Kaul <martin@familie-kaul.de>
 */

// #define ENABLE_DEBUGGING 1
#if defined(ENABLE_DEBUGGING)
#	define USE_NON_OPTIMIZED_FUNCTION __attribute__((optimize("-O0")))
#	define USE_INLINED_FUNCTION
#else
#	define USE_NON_OPTIMIZED_FUNCTION
#	define USE_INLINED_FUNCTION inline
#endif

//#define USE_DEBUG_PORT 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/cdev.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <clocksource/timer-ti-dm.h>
#include <linux/fs.h>

#include "omap_hwmod.h"

//==================================================================================================

#define IOCTL_INIT_TIMER 	_IO('U', 6)
#define IOCTL_START_TIMER 	_IO('U', 7)
#define IOCTL_STOP_TIMER 	_IO('U', 8)

//==================================================================================================

// IRQ treshold/prio control
extern void omap_intc_set_irq_priority(u8 irq_number, u8 priority);
extern void omap_intc_enable_low_prio_irqs(void);
extern void omap_intc_disable_low_prio_irqs(void);

#if defined(USE_DEBUG_PORT)
// debug port
extern void am335x_set_debug_port(u8 port_number, bool status);
#endif

//==================================================================================================
struct FipTimerData {
	uint32_t timer_rate;
	int32_t timer_irq;
	dev_t dev_timer;
	struct class *dev_class_timer;
	struct cdev timer_cdev;
	struct omap_dm_timer clksrc;
	unsigned long lew_rate;
	int timer_hwirq_number;

	bool enabled;
	unsigned int reload_time_us;
};

//--------------------------------------------------------------------------------------------------
struct FipTimerData fip_timer_data = {
	.timer_rate = 0,
	.timer_irq = -1,
	.dev_timer = 0,
	.dev_class_timer = NULL,
	.lew_rate = 0,
	.timer_hwirq_number = 0,
	.enabled = false,
	.reload_time_us = 1300U,
};

//==================================================================================================
static irqreturn_t
omap2_timer_interrupt(int irq, void *dev_id) USE_NON_OPTIMIZED_FUNCTION;

static int timer_open(struct inode *inode,
		      struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static int timer_release(struct inode *inode,
			 struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static long timer_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg) USE_NON_OPTIMIZED_FUNCTION;
static void

omap2_timer4_clocksource_init(int timer4_id, const char *fck_source,
			      const char *property) USE_NON_OPTIMIZED_FUNCTION;
static int omap_dm_timer4_init(struct omap_dm_timer *timer,
			       const char *fck_source, const char *property,
			       int posted) USE_NON_OPTIMIZED_FUNCTION;

//==================================================================================================
static struct clock_event_device clockevent_timer = {
	.features		= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.rating			= 300,
	.min_delta_ns   = 1000,
	.max_delta_ns 	= 2000000,
};

//--------------------------------------------------------------------------------------------------
static const struct of_device_id irq_control_timer_module_timer_of_match[] = {
    { .compatible   = "ti,am335x-timer" },
    {},
};

//--------------------------------------------------------------------------------------------------
static struct irqaction omap2_timer_irq = {
	.name		= "irq_control_timer",
	.flags		= __IRQF_TIMER | IRQF_NO_SUSPEND | IRQF_ONESHOT,
	.handler	= omap2_timer_interrupt,
};

//==================================================================================================
static struct file_operations fops =
{
		.owner	        = THIS_MODULE,
		.open	        = timer_open,
		.unlocked_ioctl = timer_ioctl,
		.release 		= timer_release,
};

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
void enable_module_clock(void)
{
	// enable timer module
	// TODO debug: force module clock to stay enabled - no clue why this is not happening automatically...
	void __iomem * cm_reg = NULL;
	u32 reg_val = 0;
	cm_reg = ioremap(0x44E00088, 4);
	reg_val = readl_relaxed( cm_reg );
	reg_val |= 0x2;
	writel_relaxed( reg_val, cm_reg );
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
void irq_control_timer_init(void)
{
	omap2_timer4_clocksource_init(3, "timer_sys_ck", "ti,timer-alwon");
	fip_timer_data.lew_rate = ((unsigned long)(1000000000UL/fip_timer_data.clksrc.rate)); // 41 for fip_timer_data.clksrc.rate==24MHz

	enable_module_clock();
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
void irq_control_timer_enable(void)
{
	enable_module_clock();

	fip_timer_data.enabled = true;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
void irq_control_timer_disable(void)
{
	fip_timer_data.enabled = false;
	omap_intc_enable_low_prio_irqs();
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
void irq_control_timer_start(unsigned int time_us, unsigned int reload_time_us)
{
	unsigned long cycles;
	

	if( !fip_timer_data.enabled )
	{
		return;
	}

	cycles = (unsigned int)((time_us * 1000U) / fip_timer_data.lew_rate);
	__omap_dm_timer_load_start(&fip_timer_data.clksrc, OMAP_TIMER_CTRL_ST,
				   (unsigned int)(0xffffffffU - cycles),
				   OMAP_TIMER_NONPOSTED);

	fip_timer_data.reload_time_us = reload_time_us;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
void irq_control_timer_stop(void)
{
	__omap_dm_timer_stop(&fip_timer_data.clksrc, OMAP_TIMER_POSTED, fip_timer_data.clksrc.rate);
}

/*******************************************************************************/ /*!
 * @brief  Open function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int timer_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Release function that will be called when the device driver is released.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int timer_release(struct inode *inode, struct file *file)
{
	fip_timer_data.enabled = false;
	omap_intc_enable_low_prio_irqs();
	
    /* Release the IRQ handler */
    // fip_enable_foreign_irq();
    //remove_irq(fip_timer_data.timer_irq, &omap2_timer_irq);

    /* Release the timer */
    // ret = omap_dm_timer_free(&fip_timer_data.clksrc);

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Ioctl function that will be called when the user space application
 *         performs an ioctl call on the device.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static long timer_ioctl(struct file *file, unsigned int cmd, unsigned long arg) // arg in µs
{
	unsigned long cycles;

	switch (cmd) {
		case IOCTL_INIT_TIMER:
			fip_timer_data.enabled = true;
			break;

		case IOCTL_START_TIMER:
		    cycles = (unsigned int)((arg*1000U)/fip_timer_data.lew_rate); // 12195 for 500us
			__omap_dm_timer_load_start(&fip_timer_data.clksrc,
				   OMAP_TIMER_CTRL_ST, (unsigned int)(0xffffffffU - cycles),
				   OMAP_TIMER_NONPOSTED);
			// fip_enable_foreign_irq();

#if defined(USE_DEBUG_PORT)
			am335x_set_debug_port(16, true);
#endif
 			break;

		case IOCTL_STOP_TIMER:
			break;

		default:
			break;
	}

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Function that initializes the clock-source of timer4.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void omap2_timer4_clocksource_init(int timer4_id, const char *fck_source, const char *property)
{
	int res;

	fip_timer_data.clksrc.id = timer4_id;
	fip_timer_data.clksrc.errata = OMAP_TIMER_ERRATA_I103_I767;
	
	res = omap_dm_timer4_init(&fip_timer_data.clksrc, fck_source, property, OMAP_TIMER_NONPOSTED);		   

	omap2_timer_irq.dev_id = &fip_timer_data.clksrc;
	setup_irq(fip_timer_data.clksrc.irq, &omap2_timer_irq);

	__omap_dm_timer_int_enable(&fip_timer_data.clksrc, OMAP_TIMER_INT_OVERFLOW);

	clockevent_timer.cpumask = cpu_possible_mask;
	clockevent_timer.irq = omap_dm_timer_get_irq(&fip_timer_data.clksrc);
	clockevents_config_and_register(&clockevent_timer, fip_timer_data.clksrc.rate,
					1000, /* Timer internal resynch latency */
					500000);
}

/*******************************************************************************/ /*!
 * @brief  Gget a timer using device-tree.
 *
 *         Helper function to get a timer during early boot using device-tree for 
 *         use as kernel system timer. Optionally, the property argument can be used 
 *         to select a timer with a specific property. Once a timer is found then mark
 *         the timer node in device-tree as disabled, to prevent the kernel from
 *         registering this timer as a platform device and so no one else can use it.
 * 
 * @param match - device-tree match structure for matching a device type
 * @param property - optional timer property to match
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static struct device_node * omap_get_timer_dt(const struct of_device_id *match,
						     const char *property)
{
	struct device_node *np;

	for_each_matching_node(np, match) {
		if (!of_device_is_available(np)) {
			continue;
		}

		if (property && !of_get_property(np, property, NULL)) {
			continue;
		}

		if (!property && (of_get_property(np, "ti,timer-alwon", NULL) ||
				  of_get_property(np, "ti,timer-pwm", NULL))) {
			continue;
		}

		return np;
	}

	return NULL;
}

/*******************************************************************************/ /*!
 * @brief  Function that initializes the timer4.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int omap_dm_timer4_init(struct omap_dm_timer *timer,
					 const char *fck_source,
					 const char *property,
					 int posted)
{
	const char *oh_name = NULL;
	struct device_node *np;
	struct omap_hwmod *oh;
	struct clk *src;

	np = omap_get_timer_dt(irq_control_timer_module_timer_of_match, property);
	if (!np) {
		return -ENODEV;
	}

	of_property_read_string_index(np, "ti,hwmods", 0, &oh_name);
	if (!oh_name) {
		of_property_read_string_index(np->parent, "ti,hwmods", 0,
					      &oh_name);
		if (!oh_name) {
			return -ENODEV;
		}
	}

	timer->irq = irq_of_parse_and_map(np, 0);
	fip_timer_data.timer_irq = timer->irq;
	if (!timer->irq) {
		return -ENXIO;
	}

	timer->io_base = of_iomap(np, 0);

	timer->fclk = of_clk_get_by_name(np, "fck");

	of_node_put(np);

	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
		return -ENODEV;
	}

	if (!timer->io_base) {
		return -ENXIO;
	}

	/* After the dmtimer is using hwmod these clocks won't be needed */
	if (IS_ERR_OR_NULL(timer->fclk)) {
		timer->fclk = clk_get(NULL, omap_hwmod_get_main_clk(oh));
	}
	if (IS_ERR(timer->fclk)) {
		return PTR_ERR(timer->fclk);
	}

	src = clk_get(NULL, fck_source);
	if (IS_ERR(src)) {
		return PTR_ERR(src);
	}

	WARN(clk_set_parent(timer->fclk, src) < 0,
	     "Cannot set timer parent clock, no PLL clock driver?");

	clk_put(src);

	omap_hwmod_enable(oh);
	__omap_dm_timer_init_regs(timer);

	if (posted) {
		__omap_dm_timer_enable_posted(timer);
	}

	/* Check that the intended posted configuration matches the actual */
	if (posted != timer->posted) {
		return -EINVAL;
	}

	timer->rate = clk_get_rate(timer->fclk);
	fip_timer_data.timer_rate = timer->rate;
	timer->reserved = 1;

	/* set priority of timer interrupt */
	of_property_read_s32(np, "interrupts", &fip_timer_data.timer_hwirq_number);
	omap_intc_set_irq_priority(fip_timer_data.timer_hwirq_number, 0x04);

	enable_module_clock();

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Interrupt function that is called when the timer has expired.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static irqreturn_t omap2_timer_interrupt(int irq, void *dev_id)
{
	unsigned long cycles;

	__omap_dm_timer_write_status(&fip_timer_data.clksrc, OMAP_TIMER_INT_OVERFLOW);

	if( !fip_timer_data.enabled )
	{
		// timer was already disabled
		return IRQ_NONE;
	}	

	cycles = (unsigned int)((fip_timer_data.reload_time_us * 1000U) / fip_timer_data.lew_rate);
	__omap_dm_timer_load_start(&fip_timer_data.clksrc, OMAP_TIMER_CTRL_ST,
				   (unsigned int)(0xffffffffU - cycles),
				   OMAP_TIMER_NONPOSTED);

	omap_intc_disable_low_prio_irqs();

	return IRQ_HANDLED;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the device driver.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init am335x_irq_control_timer_module_init(void)
{		
    /* Allocating Major number */
	if((alloc_chrdev_region(&fip_timer_data.dev_timer, 0, 1, "irq_control_timer_dev")) < 0){
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n", MAJOR(fip_timer_data.dev_timer), MINOR(fip_timer_data.dev_timer));

	/* creating cdev structure */
	cdev_init(&fip_timer_data.timer_cdev, &fops);

	/* Adding character device to the system */
	if((cdev_add(&fip_timer_data.timer_cdev, fip_timer_data.dev_timer, 1)) < 0){
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	/* Creating struct class */
	if((fip_timer_data.dev_class_timer = class_create(THIS_MODULE, "irq_control_timer_class")) == NULL){
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/* Creating device */
	if((device_create(fip_timer_data.dev_class_timer, NULL, fip_timer_data.dev_timer, NULL, "irq_control_timer")) == NULL){
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

	// omap2_timer4_clocksource_init(3, "timer_sys_ck", "ti,timer-alwon");
	// fip_timer_data.lew_rate = ((unsigned long)(1000000000UL/fip_timer_data.clksrc.rate)); // 41 for fip_timer_data.clksrc.rate==24MHz

    return 0;

r_device:
	class_destroy(fip_timer_data.dev_class_timer);

r_class:
	unregister_chrdev_region(fip_timer_data.dev_timer, 1);

	return -1;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module exit function for the device driver.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void __exit am335x_irq_control_timer_module_exit(void)
{
	/*Enable irq*/
	// fip_enable_foreign_irq();
    /* stop the timer */
	__omap_dm_timer_stop(&fip_timer_data.clksrc, OMAP_TIMER_POSTED, fip_timer_data.clksrc.rate);
    /* Release the IRQ handler */
    remove_irq(fip_timer_data.timer_irq, &omap2_timer_irq);

    /* Release the timer */
    //ret = omap_dm_timer_free(&fip_timer_data.clksrc);

	printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

//==================================================================================================
module_init(am335x_irq_control_timer_module_init);
module_exit(am335x_irq_control_timer_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Berrux Kana");
MODULE_AUTHOR("Martin Kaul < martin@familie-kaul.de >");
MODULE_DESCRIPTION("AM335x IRQ control timer");
