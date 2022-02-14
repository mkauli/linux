// SPDX-License-Identifier: GPL-2.0-only
/*
 * A simple debug port accessor module
 *
 * Copyright (C) 2022 Dominik Spindler <Leuze electronic>
 * Written by Dominik Spindler < dspindle@leuze.com >
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>

//==================================================================================================
struct debug_port_device {
	void __iomem *clear_reg_mem;
	void __iomem *set_reg_mem;
};	

// device data
// TODO should not be static but some private data of the module/driver instance
static struct debug_port_device *device;

/*******************************************************************************/ /*!
 * @brief  Sets the status of the debug port.
 *
 * @param status: true = set the port ; false = clear the port
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void am335x_set_debug_port(u8 port_number, bool status)
{
	if (status) {
		writel_relaxed(1U << port_number, device->set_reg_mem);
	} else {
		writel_relaxed(1U << port_number, device->clear_reg_mem);
	}
}

/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the module am335x_debug_port.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init am335x_debug_port_init(void)
{
	// init global device data
	device = NULL;

	device = kzalloc(sizeof(struct debug_port_device), GFP_KERNEL);
	if (device == NULL) {
		return -ENOMEM;
	}

    // init debug port registers
    device->clear_reg_mem = ioremap(0x481AE190, 4);
	device->set_reg_mem = ioremap(0x481AE194, 4);

	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Kernel module exit function for the module am335x_debug_port.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void __exit am335x_debug_port_exit(void)
{
    // TODO delete allocated device structure
}

//==================================================================================================
module_init(am335x_debug_port_init);
module_exit(am335x_debug_port_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dominik Spindler < dspindle@leuze.com >");
MODULE_DESCRIPTION("A simple debug port accessor module");
MODULE_VERSION("0.1");
