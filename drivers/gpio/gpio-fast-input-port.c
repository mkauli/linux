// SPDX-License-Identifier: GPL-2.0-only
/*
 * A fast GPIO Interrupt driver to send the Signal to the userspace
 *
 * Copyright (C) 2020 Martin Kaul <private>
 * Written by Martin Kaul <martin@familie-kaul.de>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/sched/signal.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>

#define IOCTL_SET_VARIABLES _IO('U', 0)
#define IOCTL_ENABLE_FOREIGN_IRQ _IO('U', 1)
#define IOCTL_DISABLE_FOREIGN_IRQ _IO('U', 2)

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
};

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
};

// ToDo: check if this is neccessary
extern ktime_t tick_period;

//==================================================================================================
static irq_handler_t fip_irq_handler(unsigned int irq, void *dev_id,
				     struct pt_regs *regs);
static int fip_open(struct inode *inode, struct file *file);
static int fip_release(struct inode *inode, struct file *file);
static long fip_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static void fip_enable_foreign_irq(void);
static void fip_disable_foreign_irq(void);

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
	case IOCTL_SET_VARIABLES:
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

	case IOCTL_ENABLE_FOREIGN_IRQ:
		fip_enable_foreign_irq();
		break;

	case IOCTL_DISABLE_FOREIGN_IRQ:
		fip_disable_foreign_irq();
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
	if (fip_us_app_info.app_task != NULL) {
		tick_period = 100000000; //set period to 100 ms
		writel_relaxed(1U << 4, fip_gpio_data.intc_ilr0_reg_mem);
		if (send_sig_info(SIGNAL_FIP, &fip_us_app_info.signal_info,
				  fip_us_app_info.app_task) < 0) {
			printk(KERN_INFO
			       "fip_irq_handler: cannot send signal\n");
		}

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
MODULE_AUTHOR("Martin Kaul < martin@familie-kaul.de >");
MODULE_DESCRIPTION(
	"A fast GPIO Interrupt driver to send the Signal to the userspace");
MODULE_VERSION("0.1");

#ifdef notdefined

#define SIGLEUZE 44
#define REG_CURRENT_TASK _IOW('a', 'a', int32_t *)
#define IOCTL_SET_VARIABLES 0
#define IOCTL_ENABLE_SYSTIMER 1
#define START_ADDR 0x48200288
/* GPIO Port */
static unsigned int gpioSync = 47;
static unsigned int input_port_irq_number;

//?? PATCH bkana@leuze.com : jitter optimization
extern unsigned long lew_local_irq_save(void);
extern void lew_local_irq_restore(unsigned long flags);

/* Signaling to Application */
static struct task_struct *task = NULL;

int32_t value = 0;
dev_t dev = 0;
static struct class *dev_class;
static struct cdev leuze_cdev;
pid_t pid;
void __iomem *mem;
extern ktime_t tick_period;

static int leuze_open(struct inode *inode, struct file *file);
static int leuze_release(struct inode *inode, struct file *file);
static long leuze_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static irq_handler_t leuze_irq_handler(unsigned int irq, void *dev_id,
				       struct pt_regs *regs);

typedef struct _ioctl_arg {
	unsigned long pid;
} ioctl_arg_t;

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = leuze_open,
	.unlocked_ioctl = leuze_ioctl,
	.release = leuze_release,
};

struct siginfo info;
struct task_struct *t = NULL;

static int leuze_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "Device File Opened...\n");
	return 0;
}
static int leuze_release(struct inode *inode, struct file *file)
{
	struct task_struct *ref_task = get_current();
	printk(KERN_INFO "Device File Closed...\n");

	/* delete task */
	if (ref_task) {
		task = NULL;
	}
	return 0;
}
static long leuze_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	ioctl_arg_t args;

	switch (cmd) {
	case IOCTL_SET_VARIABLES:
		memset(&info, 0, sizeof(struct siginfo));
		info.si_signo = SIGLEUZE;
		info.si_code = SI_QUEUE;
		info.si_int = 1234;
		if (copy_from_user(&args, (ioctl_arg_t *)arg,
				   sizeof(ioctl_arg_t)))
			return -EACCES;
		pid = args.pid;
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
		// find the task with that pid
		lew_local_irq_save();

		break;
	case IOCTL_ENABLE_SYSTIMER:
		lew_local_irq_restore(0xff);
		break;
	default:
		break;
	}

	return 0;
}

static int __init leuze_init(void)
{
	gpio_request(gpioSync, "sysfs"); /* Set up the gpioSync */
	gpio_direction_input(gpioSync);
	gpio_export(gpioSync, false);

	input_port_irq_number = gpio_to_irq(gpioSync);

	/* Allocating Major number */
	if ((alloc_chrdev_region(&dev, 0, 1, "leuze_Dev")) < 0) {
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n", MAJOR(dev), MINOR(dev));

	/* creating cdev structure */
	cdev_init(&leuze_cdev, &fops);

	/* Adding character device to the system */
	if ((cdev_add(&leuze_cdev, dev, 1)) < 0) {
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	/* Creating struct class */
	if ((dev_class = class_create(THIS_MODULE, "leuze_class")) == NULL) {
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/* Creating device */
	if ((device_create(dev_class, NULL, dev, NULL, "dsp_sync_input")) ==
	    NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

	mem = ioremap(START_ADDR, 4);
	if ((request_threaded_irq(input_port_irq_number, NULL,
				  (irq_handler_t)leuze_irq_handler,
				  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				  "dsp_sync_input", NULL))) {
		printk(KERN_INFO "cannot register IRQ");
		goto irq;
	}
	return 0;

irq:
	free_irq(input_port_irq_number, NULL);
r_device:
	class_destroy(dev_class);
r_class:
	unregister_chrdev_region(dev, 1);
	return -1;
}

static void __exit leuze_exit(void)
{
	free_irq(input_port_irq_number, NULL);
	gpio_unexport(gpioSync);
	gpio_free(gpioSync);
	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&leuze_cdev);
	unregister_chrdev_region(dev, 1);
	tick_period = 1000000; //reset period to 1 ms
	printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

static irq_handler_t leuze_irq_handler(unsigned int irq, void *dev_id,
				       struct pt_regs *regs)
{
	lew_local_irq_save();
	//rcu_read_lock();
	if (t != NULL) {
		tick_period = 100000000; //set period to 100 ms
		iowrite32(1U << 4, mem);
		send_sig_info(SIGLEUZE, &info, t);
		//rcu_read_unlock();
		//return (irq_handler_t) IRQ_HANDLED;
		return (irq_handler_t)IRQ_WAKE_THREAD;
	} else {
		//rcu_read_unlock();
		return (irq_handler_t)IRQ_NONE;
	}
}

#endif // notdefined
