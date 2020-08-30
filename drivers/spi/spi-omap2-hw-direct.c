// SPDX-License-Identifier: GPL-2.0-only
/*
 * A spi driver that uses direct the OMAP2-MCSPI hardware without use of the queued spi driver class.
 *
 * Copyright (C) 2020 Martin Kaul <private>
 * Written by Martin Kaul <martin@familie-kaul.de>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

//==================================================================================================
struct Omap2HwDirectDeviceInfo {
	dev_t device;
	struct cdev driver_cdev;
	struct class *dev_class;
};

//==================================================================================================
static struct Omap2HwDirectDeviceInfo omap2_hw_direct_device_info = {
	.device = 0,
	.dev_class = NULL,
};

//==================================================================================================
static int omap2_hw_direct_open(struct inode *inode, struct file *file);
static int omap2_hw_direct_release(struct inode *inode, struct file *file);
static long omap2_hw_direct_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

//==================================================================================================
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = omap2_hw_direct_open,
	.unlocked_ioctl = omap2_hw_direct_ioctl,
	.release = omap2_hw_direct_release,
};

//==================================================================================================

/*******************************************************************************/ /*!
 * @brief  Open function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int omap2_hw_direct_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "omap2_hw_direct_open\n");
	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Release function that will be called when the device driver is released.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int omap2_hw_direct_release(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "omap2_hw_direct_release\n");
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
static long omap2_hw_direct_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return 0;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the module fast_input_port.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init omap2_hw_direct_init(void)
{
	/* Allocating Major number */
	if ((alloc_chrdev_region(&omap2_hw_direct_device_info.device, 0, 1,
				 "spiomap2hw_Dev")) < 0) {
		printk(KERN_INFO "Cannot allocate major number");
		return -1;
	}
	printk(KERN_INFO "Major = %d Minor = %d \n",
	       MAJOR(omap2_hw_direct_device_info.device), MINOR(omap2_hw_direct_device_info.device));

	/* creating cdev structure */
	cdev_init(&omap2_hw_direct_device_info.driver_cdev, &fops);

	/* Adding character device to the system */
	if ((cdev_add(&omap2_hw_direct_device_info.driver_cdev, omap2_hw_direct_device_info.device,
		      1)) < 0) {
		printk(KERN_INFO "Cannot add the device to the system");
		goto r_class;
	}

	// /* Creating struct class */
	if ((omap2_hw_direct_device_info.dev_class =
		     class_create(THIS_MODULE, "spiomap2hw_class")) == NULL) {
		printk(KERN_INFO "Cannot create the struct class\n");
		goto r_class;
	}
	/* Creating device */
	if ((device_create(omap2_hw_direct_device_info.dev_class, NULL,
			   omap2_hw_direct_device_info.device, NULL, "spiomap2hw_input")) ==
	    NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

    return 0;

r_device:
	class_destroy(omap2_hw_direct_device_info.dev_class);
r_class:
	unregister_chrdev_region(omap2_hw_direct_device_info.device, 1);
	return -1;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module exit function for the module fast_input_port.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void __exit omap2_hw_direct_exit(void)
{
	device_destroy(omap2_hw_direct_device_info.dev_class, omap2_hw_direct_device_info.device);
	class_destroy(omap2_hw_direct_device_info.dev_class);
	cdev_del(&omap2_hw_direct_device_info.driver_cdev);
	unregister_chrdev_region(omap2_hw_direct_device_info.device, 1);
}

//==================================================================================================
module_init(omap2_hw_direct_init);
module_exit(omap2_hw_direct_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Kaul < martin@familie-kaul.de >");
MODULE_DESCRIPTION("A spi driver that uses direct the OMAP2-MCSPI hardware without use of the queued spi driver class.");
MODULE_VERSION("0.1");
