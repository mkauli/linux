// SPDX-License-Identifier: GPL-2.0-only
/*
 * A fast GPIO Interrupt driver to send the Signal to the userspace
 *
 * Copyright (C) 2020 Berrux Kana <Leuze electronic>
 * Written by Berrux Kana < bkana@leuze.de >
 *
 * Copyright (C) 2020 Martin Kaul <private>
 * Reformat & enhanced by Martin Kaul <martin@familie-kaul.de>
 */
// #define ENABLE_DEBUGGING 1
#if defined(ENABLE_DEBUGGING)
#	define USE_NON_OPTIMIZED_FUNCTION __attribute__((optimize("-Og")))
#	define USE_INLINED_FUNCTION
#else
#	define USE_NON_OPTIMIZED_FUNCTION
#	define USE_INLINED_FUNCTION inline
#endif

#define MONITOR_TIME_DIFFERENCE 1
// #define USE_DEBUG_PORT 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/sched/signal.h>

#define IOCTL_FIP_SET_VARIABLES _IO('U', 0)
#define IOCTL_FIP_ENABLE_FOREIGN_IRQ _IO('U', 1)
#define IOCTL_FIP_DISABLE_FOREIGN_IRQ _IO('U', 2)

#define SIGNAL_FIP 44

#define START_ADDR 0x48200288
#define INTC_INTC_ILR0_BASE_REG 0x48200100

#define INCT_CONTROL_BASE 0x48200000
#define INTC_THRESHOLD 0x0068

extern struct class * class_find( const char * name );

//==================================================================================================
struct FipGpioData {
	unsigned int gpio_id;
	unsigned int irq_number;
	void __iomem *intc_ilr0_reg_mem;
	struct gpio_desc *gpio;
	struct irq_data *irq;
	struct irq_desc *irqd;
	struct irq_desc *parent_irqd;
	int gpio_bank_hwirq_number;
};

//--------------------------------------------------------------------------------------------------
struct FipDeviceInfo {
	dev_t device;
	struct cdev driver_cdev;
	struct class *dev_class;
};

//--------------------------------------------------------------------------------------------------
struct FipUserSpaceApplicationInfo {
	unsigned long pid;
	void *instance_ptr;
	struct kernel_siginfo signal_info;
	struct task_struct *app_task;
};

//--------------------------------------------------------------------------------------------------
struct FipIoctlInfo {
	unsigned long pid;
	void *instance_ptr;
};

//--------------------------------------------------------------------------------------------------
struct FipInterruptData {
	void __iomem *intc_threshold_reg;
	bool interrupt_enabled;
};

//--------------------------------------------------------------------------------------------------
#if defined(MONITOR_TIME_DIFFERENCE)
struct FipSystemTimerData {
	void __iomem * system_timer_reg;
	unsigned int init_counter;
	unsigned int max_time_value;
	unsigned int min_time_value;
	unsigned int last_time_value;
};
#endif

//--------------------------------------------------------------------------------------------------
#if defined(USE_DEBUG_PORT)
struct FipDebugPort {
	void __iomem *clear_reg_mem;
	void __iomem *set_reg_mem;
};
#endif

//==================================================================================================
static struct FipGpioData fip_gpio_data = {
	.gpio_id = 44,
	.irq_number = 0,
	.intc_ilr0_reg_mem = NULL,
	.gpio = NULL,
	.irq = NULL,
	.irqd = NULL,
	.parent_irqd = NULL,
	.gpio_bank_hwirq_number = 0,
};

static struct FipDeviceInfo fip_device_info = {
	.device = 0,
	.dev_class = NULL,
};

static struct FipUserSpaceApplicationInfo fip_us_app_info = {
	.pid = 0,
	.instance_ptr = NULL,
	.app_task = NULL,
};

static struct FipInterruptData fip_irq_data = {
	.intc_threshold_reg = NULL,
	.interrupt_enabled = true,
};

#if defined(MONITOR_TIME_DIFFERENCE)
static struct FipSystemTimerData fip_system_timer_data = {
	.system_timer_reg = NULL,
	.init_counter = 20,
	.max_time_value = 0,
	.min_time_value = 0xFFFFFFFF,
	.last_time_value = 0,
};
#endif

#if defined(USE_DEBUG_PORT)
struct FipDebugPort fip_debug_port = {
	.clear_reg_mem = NULL,
	.set_reg_mem = NULL,
};
#endif

// ToDo: check if this is neccessary
extern ktime_t tick_period;

//==================================================================================================
static irq_handler_t fip_irq_handler(unsigned int irq, void *dev_id,
				     struct pt_regs *regs) USE_NON_OPTIMIZED_FUNCTION;
static int fip_open(struct inode *inode, struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static int fip_release(struct inode *inode, struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static long fip_ioctl(struct file *file, unsigned int cmd, unsigned long arg) USE_NON_OPTIMIZED_FUNCTION;

static void fip_enable_foreign_irq(void) USE_NON_OPTIMIZED_FUNCTION;
static void fip_disable_foreign_irq(void) USE_NON_OPTIMIZED_FUNCTION;

#if defined(USE_DEBUG_PORT)
static void fip_set_debug_port(bool status) USE_NON_OPTIMIZED_FUNCTION;
#endif

//==================================================================================================
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = fip_open,
	.unlocked_ioctl = fip_ioctl,
	.release = fip_release,
};

//==================================================================================================

/*******************************************************************************/ /*!
 * @brief  Open function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int fip_open(struct inode *inode, struct file *file)
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
static int fip_release(struct inode *inode, struct file *file)
{
	fip_us_app_info.app_task = NULL;
	fip_us_app_info.pid = 0;
	fip_us_app_info.instance_ptr = NULL;
	tick_period = 1000000; //reset period to 1 ms

#if defined(MONITOR_TIME_DIFFERENCE)
	fip_system_timer_data.init_counter = 20;
#endif

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
static long fip_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct FipIoctlInfo args;

	switch (cmd) {
	case IOCTL_FIP_SET_VARIABLES:
		memset(&fip_us_app_info.signal_info, 0,
		       sizeof(struct kernel_siginfo));
		fip_us_app_info.signal_info.si_signo = SIGNAL_FIP;
		fip_us_app_info.signal_info.si_code = SI_KERNEL;

		if (copy_from_user(&args, (struct FipIoctlInfo *)arg,
				   sizeof(struct FipIoctlInfo))) {
			printk(KERN_INFO
			       "fip_ioctl: cannot copy user arguments");
			return -EACCES;
		}
		fip_us_app_info.pid = args.pid;
		fip_us_app_info.instance_ptr = args.instance_ptr;
		fip_us_app_info.app_task =
			pid_task(find_pid_ns(fip_us_app_info.pid, &init_pid_ns),
				 PIDTYPE_PID);

		fip_us_app_info.signal_info.si_int = (int)args.instance_ptr;
		break;

	case IOCTL_FIP_ENABLE_FOREIGN_IRQ:
		fip_enable_foreign_irq();
		fip_irq_data.interrupt_enabled = true;
		break;

	case IOCTL_FIP_DISABLE_FOREIGN_IRQ:
		fip_disable_foreign_irq();
		fip_irq_data.interrupt_enabled = false;
		break;

	default:
		break;
	}

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Interrupt handler function that will be called when the interrup
 *         occure.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static irq_handler_t fip_irq_handler(unsigned int irq, void *dev_id,
				     struct pt_regs *regs)
{
#if defined(MONITOR_TIME_DIFFERENCE)
	unsigned int time_value = readl_relaxed(fip_system_timer_data.system_timer_reg);
	unsigned int time_difference;
#endif

	writel_relaxed(1U << 4, fip_gpio_data.intc_ilr0_reg_mem);

	if (fip_us_app_info.app_task != NULL) {

#if defined(MONITOR_TIME_DIFFERENCE)
		if(!fip_irq_data.interrupt_enabled) {
			if(fip_system_timer_data.init_counter > 0) {
				fip_system_timer_data.last_time_value = time_value;
				fip_system_timer_data.init_counter--;
				fip_system_timer_data.max_time_value = 0;
				fip_system_timer_data.min_time_value = 0xFFFFFFFF;
			}
			else {
				time_difference = (time_value - fip_system_timer_data.last_time_value);
				if(time_difference > fip_system_timer_data.max_time_value) {
					fip_system_timer_data.max_time_value = time_difference;
				}
				if(time_difference < fip_system_timer_data.min_time_value) {
					fip_system_timer_data.min_time_value = time_difference;
				}
				fip_system_timer_data.last_time_value = time_value;
			}

			if(fip_system_timer_data.max_time_value > (9600000+8400)) {
				static int counter = 0;
				counter++;
			}
		}
#endif

#if defined(USE_DEBUG_PORT)
		fip_set_debug_port(true);
#endif

		tick_period = 100000000; //set period to 100 ms
		if (send_sig_info(SIGNAL_FIP, &fip_us_app_info.signal_info,
				  fip_us_app_info.app_task) < 0) {
			printk(KERN_INFO
			       "fip_irq_handler: cannot send signal\n");
		}

#if defined(USE_DEBUG_PORT)
		fip_set_debug_port(false);
#endif

		return (irq_handler_t)IRQ_HANDLED;
	}

	return (irq_handler_t)IRQ_NONE;
}

/*******************************************************************************/ /*!
 * @brief  Enables interrupt handling of other interrupts than the FIP irqs.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void fip_enable_foreign_irq(void)
{
	writel_relaxed(0xFF, fip_irq_data.intc_threshold_reg);
}

/*******************************************************************************/ /*!
 * @brief  Disables interrupt handling of other interrupts than the FIP irqs.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void fip_disable_foreign_irq(void)
{
	writel_relaxed(0x05, fip_irq_data.intc_threshold_reg);
}

#if defined(USE_DEBUG_PORT)
/*******************************************************************************/ /*!
 * @brief  Disables interrupt handling of other interrupts than the FIP irqs.
 *
 * @param status: true = set the port ; false = clear the port
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void fip_set_debug_port(bool status)
{
	if(status) {
		writel_relaxed(1U << 16, fip_debug_port.set_reg_mem);
	}
	else {
		writel_relaxed(1U << 16, fip_debug_port.clear_reg_mem);
	}
}
#endif

/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the module fast_input_port.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init fast_input_port_init(void)
{
	int gpio_bank_ilr0_base_reg;

	/* Setup the memory accesses of the AMS335x interrupt intc registers */
	fip_irq_data.intc_threshold_reg =
		ioremap(INCT_CONTROL_BASE + INTC_THRESHOLD, 4);

	/* Setup the requested GPIO-ID */
	gpio_request(fip_gpio_data.gpio_id, "sysfs");
	gpio_direction_input(fip_gpio_data.gpio_id);
	gpio_export(fip_gpio_data.gpio_id, false);

	fip_gpio_data.irq_number = gpio_to_irq(fip_gpio_data.gpio_id);
	fip_gpio_data.gpio = gpio_to_desc(fip_gpio_data.gpio_id);
	fip_gpio_data.irq = irq_get_irq_data(fip_gpio_data.irq_number);
	fip_gpio_data.irqd = irq_to_desc(fip_gpio_data.irq_number);
	fip_gpio_data.parent_irqd = irq_to_desc(fip_gpio_data.irqd->parent_irq);
	fip_gpio_data.gpio_bank_hwirq_number =
		fip_gpio_data.parent_irqd->irq_data.hwirq;
	gpio_bank_ilr0_base_reg = INTC_INTC_ILR0_BASE_REG +
				  (fip_gpio_data.gpio_bank_hwirq_number * 4);

	/* Allocating Major number */
	if ((alloc_chrdev_region(&fip_device_info.device, 0, 1,
				 "fastinputport_Dev")) < 0) {
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n",
	       MAJOR(fip_device_info.device), MINOR(fip_device_info.device));

	/* creating cdev structure */
	cdev_init(&fip_device_info.driver_cdev, &fops);

	/* Adding character device to the system */
	if ((cdev_add(&fip_device_info.driver_cdev, fip_device_info.device,
		      1)) < 0) {
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	// /* Creating struct class */
	if ((fip_device_info.dev_class =
		     class_create(THIS_MODULE, "fip_class")) == NULL) {
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/* Creating device */
	if ((device_create(fip_device_info.dev_class, NULL,
			   fip_device_info.device, NULL, "fip_input")) ==
	    NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

	fip_gpio_data.intc_ilr0_reg_mem = ioremap(gpio_bank_ilr0_base_reg, 4);
	if ((request_threaded_irq(fip_gpio_data.irq_number, NULL,
				  (irq_handler_t)fip_irq_handler,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
					  IRQF_NO_SUSPEND | IRQF_NO_THREAD |
					  IRQF_NOBALANCING,
				  "fip_input", NULL))) {
		printk(KERN_INFO "cannot register IRQ");
		goto irq;
	}

#if defined(MONITOR_TIME_DIFFERENCE)
	/* system timer settings. */
	fip_system_timer_data.system_timer_reg = ioremap(0x44E31028, 4);
#endif

#if defined(USE_DEBUG_PORT)
	/* debug port settings */
	fip_debug_port.clear_reg_mem = ioremap(0x481AE190, 4);
	fip_debug_port.set_reg_mem = ioremap(0x481AE194, 4);
#endif

	return 0;

irq:
	free_irq(fip_gpio_data.irq_number, NULL);
r_device:
	class_destroy(fip_device_info.dev_class);
r_class:
	unregister_chrdev_region(fip_device_info.device, 1);
	return -1;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module exit function for the module fast_input_port.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void __exit fast_input_port_exit(void)
{
	free_irq(fip_gpio_data.irq_number, NULL);
	gpio_unexport(fip_gpio_data.gpio_id);
	gpio_free(fip_gpio_data.gpio_id);
	device_destroy(fip_device_info.dev_class, fip_device_info.device);
	class_destroy(fip_device_info.dev_class);
	cdev_del(&fip_device_info.driver_cdev);
	unregister_chrdev_region(fip_device_info.device, 1);
	printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

//==================================================================================================
module_init(fast_input_port_init);
module_exit(fast_input_port_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Berrux Kana < bkana@leuze.de >");
MODULE_AUTHOR("Martin Kaul < martin@familie-kaul.de >");
MODULE_DESCRIPTION(
	"A fast GPIO Interrupt driver to send the Signal to the userspace");
MODULE_VERSION("0.1");
