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

// #define MONITOR_TIME_DIFFERENCE 1
// #define USE_DEBUG_PORT 1
// #define USE_SIGNAL 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#if defined USE_SIGNAL
#include <linux/sched/signal.h>
#endif

//==================================================================================================

#define IOCTL_FIP_SET_VARIABLES _IO('U', 0)
#define IOCTL_FIP_ENABLE_FOREIGN_IRQ _IO('U', 1)
#define IOCTL_FIP_DISABLE_FOREIGN_IRQ _IO('U', 2)

//==================================================================================================

// ssi timer
extern int ssi_timer_init(void);
extern int ssi_timer_start(void);

// IRQ treshold/prio control
extern void omap_intc_set_irq_priority(u8 irq_number, u8 priority);
extern void omap_intc_enable_low_prio_irqs(void);
extern void omap_intc_disable_low_prio_irqs(void);

// IRQ control timer
extern void irq_control_timer_init(void);
extern void irq_control_timer_start(unsigned int time_us, unsigned int reload_time_us);
extern void irq_control_timer_stop(void);

#if defined(USE_DEBUG_PORT)
// debug port
extern void am335x_set_debug_port(u8 port_number, bool status); // use port_number 8
#endif

// ToDo: check if this is neccessary
extern ktime_t tick_period;

//==================================================================================================
struct FipGpioData {
	unsigned int gpio_id;
	unsigned int irq_number;
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
struct FipIoctlInfo {
	unsigned long pid;
	unsigned long irq_disable_time_1_us;
	unsigned long irq_disable_time_2_us;
	void *instance_ptr;
};

//--------------------------------------------------------------------------------------------------
struct FipInterruptData {
	unsigned long irq_disable_time_1_us;
	unsigned long irq_disable_time_2_us;
};

//--------------------------------------------------------------------------------------------------
#if defined USE_SIGNAL
struct FipUserSpaceApplicationInfo {
	unsigned long pid;
	void *instance_ptr;
	struct kernel_siginfo signal_info;
	struct task_struct *app_task;
};
#endif

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
struct fip_device {
	wait_queue_head_t wait_queue;
	int wait_queue_flag;

	struct FipGpioData fip_gpio_data;
	struct FipDeviceInfo fip_device_info;
	struct FipInterruptData fip_irq_data;

#if defined USE_SIGNAL
	struct FipUserSpaceApplicationInfo fip_us_app_info;
#endif

#if defined(MONITOR_TIME_DIFFERENCE)
	struct FipSystemTimerData fip_system_timer_data;
#endif
};	

// device data
// TODO should not be static but some private data of the module/driver instance
static struct fip_device *device;

//==================================================================================================
static irq_handler_t fip_irq_handler(unsigned int irq, void *dev_id,
				     struct pt_regs *regs) USE_NON_OPTIMIZED_FUNCTION;

static int fip_open(struct inode *inode,
		    struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static int fip_release(struct inode *inode,
		       struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static long fip_ioctl(struct file *file, unsigned int cmd,
		      unsigned long arg) USE_NON_OPTIMIZED_FUNCTION;
static ssize_t fip_read(struct file *file, char __user *buf, size_t count,
			loff_t *offset);

//==================================================================================================
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = fip_open,
	.unlocked_ioctl = fip_ioctl,
	.release = fip_release,
	.read = fip_read
};

/*******************************************************************************/ /*!
 * @brief  Open function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int fip_open(struct inode *inode, struct file *file)
{
	device->wait_queue_flag = 0;

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
#if defined USE_SIGNAL
	device->fip_us_app_info.app_task = NULL;
	device->fip_us_app_info.pid = 0;
	device->fip_us_app_info.instance_ptr = NULL;
#endif
	
	// TODO required?
	tick_period = 1000000; //reset period to 1 ms

#if defined(MONITOR_TIME_DIFFERENCE)
	device->fip_system_timer_data.init_counter = 100;
#endif

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
static void init_userspace_data(struct FipIoctlInfo *args)
{
#if defined USE_SIGNAL
	memset(&device->fip_us_app_info.signal_info, 0,
	       sizeof(struct kernel_siginfo));
	device->fip_us_app_info.signal_info.si_signo =
		CONFIG_GPIO_FAST_INPUT_PORT_SIGNO;
	device->fip_us_app_info.signal_info.si_code = SI_KERNEL;

	device->fip_us_app_info.pid = args->pid;
	device->fip_us_app_info.instance_ptr = args->instance_ptr;
	device->fip_us_app_info.app_task =
		pid_task(find_pid_ns(device->fip_us_app_info.pid, &init_pid_ns),
			 PIDTYPE_PID);
	device->fip_us_app_info.signal_info.si_int = (int)args->instance_ptr;
#endif

	device->fip_irq_data.irq_disable_time_1_us = args->irq_disable_time_1_us;
	device->fip_irq_data.irq_disable_time_2_us = args->irq_disable_time_2_us;
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
		if (copy_from_user(&args, (struct FipIoctlInfo *)arg,
				   sizeof(struct FipIoctlInfo))) {
			printk(KERN_INFO
			       "fip_ioctl: cannot copy user arguments");
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
static ssize_t fip_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	char dummy;

    if(file->f_flags & O_NONBLOCK){
		return -EAGAIN;
	}

	wait_event_interruptible(device->wait_queue, device->wait_queue_flag != 0);
	device->wait_queue_flag = 0;

	// return a dummy byte to userspace
	if(count == 1)
	{
		copy_to_user(buf, &dummy, 1);
		return 1;
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
	// disable low-prio IRQs after 300us (approx. 100us before SSI timer expires ),
	// then disable low-prio IRQs again after another 1300us (right before next sync input edge)
	// actual time values are set from userspace via IOCTL
	irq_control_timer_start( device->fip_irq_data.irq_disable_time_1_us, device->fip_irq_data.irq_disable_time_2_us );


	// try to start SSI timer, actual start will only take place if it is enabled
	ssi_timer_start();
	
	// TODO this is probably not required on every invocation
	omap_intc_set_irq_priority(device->fip_gpio_data.gpio_bank_hwirq_number, 0x04);

#if defined(MONITOR_TIME_DIFFERENCE)
	unsigned int time_value = readl_relaxed(fip_system_timer_data.system_timer_reg);
	unsigned int time_difference;

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

#if defined USE_SIGNAL
	if (fip_us_app_info.app_task != NULL)
	{
		if (send_sig_info(CONFIG_GPIO_FAST_INPUT_PORT_SIGNO,
				  &fip_us_app_info.signal_info,
				  fip_us_app_info.app_task) < 0) {
			printk(KERN_INFO
			       "fip_irq_handler: cannot send signal\n");
		}
	}
#endif

	device->wait_queue_flag = 1 ;
	wake_up_interruptible(&device->wait_queue);

	return (irq_handler_t)IRQ_HANDLED;
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
static void init_dev_data(struct fip_device *dev)
{
	init_waitqueue_head(&(dev->wait_queue));

	dev->fip_gpio_data.gpio_id = CONFIG_GPIO_FAST_INPUT_PORT_ID;

	dev->fip_irq_data.irq_disable_time_1_us = 300;
	dev->fip_irq_data.irq_disable_time_2_us = 1300;

	#if defined(MONITOR_TIME_DIFFERENCE)
	dev->fip_system_timer_data.init_counter = 100;
	dev->fip_system_timer_data.min_time_value = 0xFFFFFFFF;
	#endif
}

/*******************************************************************************/ /*!
 * @brief  
 *
 * @return 
 * @exception
 * @globals
 ***********************************************************************************/
static void init_gpio(struct fip_device *dev)
{
	/* Setup the requested GPIO-ID */
	gpio_request(dev->fip_gpio_data.gpio_id, "sysfs");
	gpio_direction_input(dev->fip_gpio_data.gpio_id);
	gpio_export(dev->fip_gpio_data.gpio_id, false);

	dev->fip_gpio_data.irq_number = gpio_to_irq(dev->fip_gpio_data.gpio_id);
	dev->fip_gpio_data.gpio = gpio_to_desc(dev->fip_gpio_data.gpio_id);
	dev->fip_gpio_data.irq = irq_get_irq_data(dev->fip_gpio_data.irq_number);
	dev->fip_gpio_data.irqd = irq_to_desc(dev->fip_gpio_data.irq_number);
	dev->fip_gpio_data.parent_irqd = irq_to_desc(dev->fip_gpio_data.irqd->parent_irq);
	dev->fip_gpio_data.gpio_bank_hwirq_number = dev->fip_gpio_data.parent_irqd->irq_data.hwirq;
}

static void init_regs(struct fip_device *dev)
{
#if defined(MONITOR_TIME_DIFFERENCE)
	dev->fip_system_timer_data.system_timer_reg = ioremap(0x44E31028, 4);
#endif
}


/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the module fast_input_port.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init fast_input_port_init(void)
{
	// init global device data
	device = NULL;

	device = kzalloc(sizeof(struct fip_device), GFP_KERNEL);
	if (device == NULL) {
		return -ENOMEM;
	}

	init_dev_data(device);

	init_gpio(device);

	init_regs(device);

	/* allocate major number */
	if ((alloc_chrdev_region(&device->fip_device_info.device, 0, 1,
				 "fastinputport_Dev")) < 0) {
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n",
	       MAJOR(device->fip_device_info.device), MINOR(device->fip_device_info.device));

	/* create cdev structure */
	cdev_init(&device->fip_device_info.driver_cdev, &fops);

	/* Add character device to the system */
	if ((cdev_add(&device->fip_device_info.driver_cdev, device->fip_device_info.device,
		      1)) < 0) {
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	/* create struct class */
	if ((device->fip_device_info.dev_class =
		     class_create(THIS_MODULE, "fip_class")) == NULL) {
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}


	/* create device */
	//			   fip_device_info.device, NULL, "fip_input-%d", CONFIG_GPIO_FAST_INPUT_PORT_ID)) ==
	if ((device_create(device->fip_device_info.dev_class, NULL,
			   device->fip_device_info.device, NULL, "fip_input")) ==
	    NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

	if ((request_irq(device->fip_gpio_data.irq_number, (irq_handler_t)fip_irq_handler,
#if defined USE_SIGNAL
			// sending signal will not work in hard-irq
			IRQF_TRIGGER_FALLING | IRQF_NOBALANCING | IRQF_NO_SUSPEND,
#else
			IRQF_TRIGGER_FALLING | IRQF_NOBALANCING | IRQF_NO_SUSPEND | IRQF_NO_THREAD,
#endif
			 "fip_input", NULL))) {
		printk(KERN_INFO "cannot register IRQ");
		goto irq;
	}

	//init ssi timer
	ssi_timer_init();

	// init IRQ control timer
	irq_control_timer_init();

	return 0;

irq:
	free_irq(device->fip_gpio_data.irq_number, NULL);
r_device:
	class_destroy(device->fip_device_info.dev_class);
r_class:
	unregister_chrdev_region(device->fip_device_info.device, 1);	
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
	free_irq(device->fip_gpio_data.irq_number, NULL);
	gpio_unexport(device->fip_gpio_data.gpio_id);
	gpio_free(device->fip_gpio_data.gpio_id);
	device_destroy(device->fip_device_info.dev_class, device->fip_device_info.device);
	class_destroy(device->fip_device_info.dev_class);
	cdev_del(&device->fip_device_info.driver_cdev);
	unregister_chrdev_region(device->fip_device_info.device, 1);
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

