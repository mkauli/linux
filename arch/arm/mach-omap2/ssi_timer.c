// SPDX-License-Identifier: GPL-2.0-only
/*
 * A hardware timer implementation for AM335x usable by ssi.
 *
 * Copyright (C) 2020 Berrux Kana <Leuze electronic>
 * Written by Berrux Kana < bkana@leuze.de >
 *
 */

//#define ENABLE_DEBUGGING 1
#if defined(ENABLE_DEBUGGING)
#define USE_NON_OPTIMIZED_FUNCTION __attribute__((optimize("-O0")))
#define USE_INLINED_FUNCTION
#else
#define USE_NON_OPTIMIZED_FUNCTION
#define USE_INLINED_FUNCTION inline
#endif

// #define USE_DEBUG_PORT 1
// #define USE_SIGNAL 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <clocksource/timer-ti-dm.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

#include "omap_hwmod.h"

#if defined USE_SIGNAL
#include <linux/sched/signal.h>
#define SIGNAL_SSI 60
#endif

//==================================================================================================

#define IOCTL_FIP_SET_VARIABLES _IO('U', 0)
#define IOCTL_FIP_ENABLE_FOREIGN_IRQ _IO('U', 1)
#define IOCTL_FIP_DISABLE_FOREIGN_IRQ _IO('U', 2)
#define IOCTL_SSI_TIMER_ENABLE _IO('U', 3)
#define IOCTL_SSI_TIMER_DISABLE _IO('U', 4)
#define IOCTL_IRQ_TIMER_ENABLE _IO('U', 5)
#define IOCTL_IRQ_TIMER_DISABLE _IO('U', 6)

//==================================================================================================

// IRQ treshold/prio control
extern void omap_intc_set_irq_priority(u8 irq_number, u8 priority);
extern void omap_intc_enable_low_prio_irqs(void);
extern void omap_intc_disable_low_prio_irqs(void);

extern void irq_control_timer_enable(void);
extern void irq_control_timer_disable(void);

#if defined(USE_DEBUG_PORT)
// debug port
extern void am335x_set_debug_port(u8 port_number, bool status);
#endif

//==================================================================================================
struct SsiDeviceInfo {
	dev_t device;
	struct cdev driver_cdev;
	struct class *dev_class;
};

//--------------------------------------------------------------------------------------------------
struct SSiIoctlInfo {
	unsigned long pid;
	unsigned long ssi_output_delay_us;
	void *instance_ptr;
};

//--------------------------------------------------------------------------------------------------
struct SsiTimerData {
	uint32_t timer_rate;
	int32_t timer_irq;
	struct omap_dm_timer clksrc;
	unsigned long lew_rate;
	int timer_hwirq_number;
};

//--------------------------------------------------------------------------------------------------
#if defined USE_SIGNAL
struct SSiUserSpaceApplicationInfo {
	unsigned long pid;
	void *instance_ptr;
	struct kernel_siginfo signal_info;
	struct task_struct *app_task;
};
#endif

//--------------------------------------------------------------------------------------------------
struct ssi_timer_device {
	wait_queue_head_t wait_queue;
	int wait_queue_flag;

	bool enabled;
	unsigned int ssi_output_delay_us;

	struct SsiDeviceInfo ssi_device_info;
	struct SsiTimerData ssi_timer_data;
	
#if defined USE_SIGNAL
	struct SSiUserSpaceApplicationInfo ssi_us_app_info;
#endif
};

// device data
// TODO should not be static but some private data of the module/driver instance
static struct ssi_timer_device *device;

//==================================================================================================
static irqreturn_t
omap2_timer_interrupt(int irq, void *dev_id) USE_NON_OPTIMIZED_FUNCTION;

static int ssi_open(struct inode *inode,
		    struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static int ssi_release(struct inode *inode,
		       struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static long ssi_ioctl(struct file *file, unsigned int cmd,
		      unsigned long arg) USE_NON_OPTIMIZED_FUNCTION;
static ssize_t ssi_read(struct file *file, char __user *buf,
				size_t count, loff_t *offset);

int ssi_timer_init(void) USE_NON_OPTIMIZED_FUNCTION;
int ssi_timer_start(void) USE_NON_OPTIMIZED_FUNCTION;

static void
ssi_timer_clocksource_init(int timer5_id, const char *fck_source,
			   const char *property) USE_NON_OPTIMIZED_FUNCTION;
static int omap_dm_ssi_timer_init(struct omap_dm_timer *timer,
				  const char *fck_source, const char *property,
				  int posted) USE_NON_OPTIMIZED_FUNCTION;

//==================================================================================================
static struct clock_event_device clockevent_timer = {
	//TODO check those values. is periodic fine/needed?
	.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC, 
	.rating = 300,
	.min_delta_ns = 1000,
	.max_delta_ns = 2000000,
};

//--------------------------------------------------------------------------------------------------
static const struct of_device_id ssi_timer_of_match[] = {
	{ .compatible = "ti,am335x-ssi-timer" },
};

//--------------------------------------------------------------------------------------------------
static struct irqaction omap2_timer_irq = {
	.name = "SSI_timer",
	.flags = __IRQF_TIMER | IRQF_NO_SUSPEND | IRQF_ONESHOT,
	.handler = omap2_timer_interrupt,
};

//==================================================================================================
static struct file_operations ssiops = {
	.owner = THIS_MODULE,
	.open = ssi_open,
	.unlocked_ioctl = ssi_ioctl,
	.release = ssi_release,
	.read = ssi_read
};

/*******************************************************************************/ /*!
 * @brief  Open function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int ssi_open(struct inode *inode, struct file *file)
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
static int ssi_release(struct inode *inode, struct file *file)
{
#if defined USE_SIGNAL
	device->ssi_us_app_info.app_task = NULL;
	device->ssi_us_app_info.pid = 0;
	device->ssi_us_app_info.instance_ptr = NULL;
#endif

	device->enabled = false;
	omap_intc_enable_low_prio_irqs();

	irq_control_timer_disable();

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
static void init_userspace_data(struct SSiIoctlInfo *args)
{
#if defined USE_SIGNAL
	memset(&device->ssi_us_app_info.signal_info, 0,
	       sizeof(struct kernel_siginfo));
	device->ssi_us_app_info.signal_info.si_signo = SIGNAL_SSI;
	device->ssi_us_app_info.signal_info.si_code = SI_KERNEL;

	device->ssi_us_app_info.pid = args->pid;
	device->ssi_us_app_info.instance_ptr = args->instance_ptr;
	device->ssi_us_app_info.app_task =
		pid_task(find_pid_ns(device->ssi_us_app_info.pid, &init_pid_ns),
			 PIDTYPE_PID);
	device->ssi_us_app_info.signal_info.si_int = (int)args->instance_ptr;
#endif

	device->ssi_output_delay_us = args->ssi_output_delay_us;
}


/*******************************************************************************/ /*!
 * @brief  Ioctl function that will be called when the user space application
 *         performs an ioctl call on the device.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static long ssi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct SSiIoctlInfo args;

	switch (cmd) {
	case IOCTL_FIP_SET_VARIABLES:
		if (copy_from_user(&args, (struct SSiIoctlInfo *)arg,
				   sizeof(struct SSiIoctlInfo))) {
			printk(KERN_INFO
			       "ssi_ioctl: cannot copy user arguments");
			return -EACCES;
		}
		init_userspace_data(&args);
		break;

	case IOCTL_FIP_ENABLE_FOREIGN_IRQ:		
		omap_intc_enable_low_prio_irqs();
		break;

	case IOCTL_FIP_DISABLE_FOREIGN_IRQ:
		omap_intc_disable_low_prio_irqs();
		break;

	case IOCTL_SSI_TIMER_ENABLE:
		device->enabled = true;
		break;

	case IOCTL_SSI_TIMER_DISABLE:
		device->enabled = false;
		// enable IRQs to avoid deadlocks
		// TODO also do this in device-close, etc... ?
		omap_intc_enable_low_prio_irqs();
		break;

	case IOCTL_IRQ_TIMER_ENABLE:
		irq_control_timer_enable();
		break;

	case IOCTL_IRQ_TIMER_DISABLE:
		irq_control_timer_disable();
		omap_intc_enable_low_prio_irqs();
		break;

	default:
		break;
	}

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
static ssize_t ssi_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	char dummy;

    if(file->f_flags & O_NONBLOCK){
		return -EAGAIN;
	}

	wait_event_interruptible(device->wait_queue, device->wait_queue_flag != 0);
	device->wait_queue_flag = 0;

	// send dummy byte to userspace
	if(count == 1)
	{
		copy_to_user(buf, &dummy, 1);
		return 1;
	}	

    return 0;
}

//==================================================================================================
/*******************************************************************************/ /*!
 * @brief  ssi_timer_init function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
int ssi_timer_init(void)
{
	ssi_timer_clocksource_init(4, "timer_sys_ck", "ti,timer-alwon");
	device->ssi_timer_data.lew_rate = ((
		unsigned long)(1000000000UL /
			       device->ssi_timer_data.clksrc
				       .rate)); // 41 for ssi_timer_data.clksrc.rate==24MHz
	/* Allocating Major number */
	if ((alloc_chrdev_region(&device->ssi_device_info.device, 0, 1,
				 "ssiTimer_Dev")) < 0) {
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n",
	       MAJOR(device->ssi_device_info.device), MINOR(device->ssi_device_info.device));

	/* creating cdev structure */
	cdev_init(&device->ssi_device_info.driver_cdev, &ssiops);

	/* Adding character device to the system */
	if ((cdev_add(&device->ssi_device_info.driver_cdev, device->ssi_device_info.device,
		      1)) < 0) {
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	// /* Creating struct class */
	if ((device->ssi_device_info.dev_class =
		     class_create(THIS_MODULE, "ssi_class")) == NULL) {
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/* Creating device */
	if ((device_create(device->ssi_device_info.dev_class, NULL,
			   device->ssi_device_info.device, NULL, "ssi_timer")) ==
	    NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

	return 0;

r_device:
	class_destroy(device->ssi_device_info.dev_class);
r_class:
	unregister_chrdev_region(device->ssi_device_info.device, 1);
	return -1;
}
EXPORT_SYMBOL_GPL(ssi_timer_init);

/*******************************************************************************/ /*!
 * @brief  ssi_timer_start function that will be start the timer.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
int ssi_timer_start(void)
{
	unsigned long cycles;

	if( !device->enabled )
	{
		return -1;
	}

	cycles = (unsigned int)((device->ssi_output_delay_us * 1000U) /
				device->ssi_timer_data.lew_rate); // 12195 for 500us
	__omap_dm_timer_load_start(&device->ssi_timer_data.clksrc, OMAP_TIMER_CTRL_ST,
				   (unsigned int)(0xffffffffU - cycles),
				   OMAP_TIMER_NONPOSTED);
	return 0;
}
EXPORT_SYMBOL_GPL(ssi_timer_start);

/*******************************************************************************/ /*!
 * @brief  Function that initializes the clock-source of timer5.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void ssi_timer_clocksource_init(int timer5_id, const char *fck_source,
				       const char *property)
{
	int res;

	device->ssi_timer_data.clksrc.id = timer5_id;
	device->ssi_timer_data.clksrc.errata = OMAP_TIMER_ERRATA_I103_I767;

	res = omap_dm_ssi_timer_init(&device->ssi_timer_data.clksrc, fck_source,
				     property, OMAP_TIMER_NONPOSTED);

	omap2_timer_irq.dev_id = &device->ssi_timer_data.clksrc;
	setup_irq(device->ssi_timer_data.clksrc.irq, &omap2_timer_irq);

	__omap_dm_timer_int_enable(&device->ssi_timer_data.clksrc,
				   OMAP_TIMER_INT_OVERFLOW);

	clockevent_timer.cpumask = cpu_possible_mask;
	clockevent_timer.irq = omap_dm_timer_get_irq(&device->ssi_timer_data.clksrc);
	clockevents_config_and_register(
		&clockevent_timer, device->ssi_timer_data.clksrc.rate,
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
static struct device_node *omap_get_timer_dt(const struct of_device_id *match,
					     const char *property)
{
	struct device_node *np;

	for_each_matching_node (np, match) {
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
static int omap_dm_ssi_timer_init(struct omap_dm_timer *timer,
				  const char *fck_source, const char *property,
				  int posted)
{
	const char *oh_name = NULL;
	struct device_node *np;
	struct omap_hwmod *oh;
	struct clk *src;

	np = omap_get_timer_dt(ssi_timer_of_match, property);
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
	device->ssi_timer_data.timer_irq = timer->irq;
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
	device->ssi_timer_data.timer_rate = timer->rate;
	timer->reserved = 1;

	/* set priority of timer interrupt */
	of_property_read_s32(np, "interrupts",
			     &device->ssi_timer_data.timer_hwirq_number);
	omap_intc_set_irq_priority(device->ssi_timer_data.timer_hwirq_number,
				   0x04);

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
	__omap_dm_timer_write_status(&device->ssi_timer_data.clksrc,
				     OMAP_TIMER_INT_OVERFLOW);

	if( !device->enabled )
	{
		// timer was already disabled
		return IRQ_NONE;
	}	

#if defined(USE_DEBUG_PORT)
	am335x_set_debug_port(8, false);
#endif

	device->wait_queue_flag = 1 ;
	wake_up_interruptible(&(device->wait_queue));

#if defined USE_SIGNAL
	if (device->ssi_us_app_info.app_task != NULL) {
		if (send_sig_info(SIGNAL_SSI, &device->ssi_us_app_info.signal_info,
				  device->ssi_us_app_info.app_task) < 0) {
			printk(KERN_INFO
			       "SSI, omap2_timer_interrupt : cannot send signal\n");
		}
	}
#endif

#if defined(USE_DEBUG_PORT)
	am335x_set_debug_port(8, true);
#endif

	return IRQ_HANDLED;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
static void init_dev_data(struct ssi_timer_device *dev)
{
	init_waitqueue_head(&(dev->wait_queue));

	dev->enabled = false;
	
	dev->ssi_output_delay_us = 400; //us

	dev->ssi_timer_data.timer_irq = -1;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the device driver.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init ssi_timer_module_init(void)
{
	// init global device data
	device = NULL;

	device = kzalloc(sizeof(struct ssi_timer_device), GFP_KERNEL);
	if (device == NULL) {
		return -ENOMEM;
	}

	init_dev_data(device);

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module exit function for the device driver.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void __exit ssi_timer_module_exit(void)
{
	/* stop the timer */
	__omap_dm_timer_stop(&device->ssi_timer_data.clksrc, OMAP_TIMER_POSTED,
			     device->ssi_timer_data.clksrc.rate);
	/* Release the IRQ handler */
	remove_irq(device->ssi_timer_data.timer_irq, &omap2_timer_irq);

	/* Release the timer */
	//ret = omap_dm_timer_free(&ssi_timer_data.clksrc);
	device_destroy(device->ssi_device_info.dev_class, device->ssi_device_info.device);
	class_destroy(device->ssi_device_info.dev_class);
	cdev_del(&device->ssi_device_info.driver_cdev);
	unregister_chrdev_region(device->ssi_device_info.device, 1);

	printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

//==================================================================================================
module_init(ssi_timer_module_init);
module_exit(ssi_timer_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Berrux Kana < bkana@leuze.de >");
MODULE_DESCRIPTION("SSITimer");