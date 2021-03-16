// SPDX-License-Identifier: GPL-2.0-only
/*
 * A hardware timer implementation for AM335x usable by ssi.
 *
 * Copyright (C) 2020 Berrux Kana <Leuze electronic>
 * Written by Berrux Kana < bkana@leuze.de >
 *
 */
#define ENABLE_DEBUGGING 1
#if defined(ENABLE_DEBUGGING)
#	define USE_NON_OPTIMIZED_FUNCTION __attribute__((optimize("-O0")))
#	define USE_INLINED_FUNCTION
#else
#	define USE_NON_OPTIMIZED_FUNCTION
#	define USE_INLINED_FUNCTION inline
#endif

#define USE_DEBUG_PORT 1

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

#include "omap_hwmod.h"

#define LEW_MICROSECOND    500UL

#define INTC_INTC_ILR0_BASE_REG 0x48200100

//==================================================================================================
struct SsiTimerData {
	uint32_t timer_rate;
	int32_t timer_irq;
	struct omap_dm_timer clksrc;
	unsigned long lew_rate;
    void __iomem *intc_ilr0_reg_mem;
	int timer_hwirq_number;
};

//--------------------------------------------------------------------------------------------------
#if defined(USE_DEBUG_PORT)
struct SsiDebugPort {
	void __iomem *clear_reg_mem;
	void __iomem *set_reg_mem;
};
#endif

//==================================================================================================
static struct SsiTimerData ssi_timer_data = {
	.timer_rate = 0,
	.timer_irq = -1,
	.lew_rate = 0,
	.timer_hwirq_number = 0,
};

#if defined(USE_DEBUG_PORT)
static struct SsiDebugPort ssi_debug_port = {
	.clear_reg_mem = NULL,
	.set_reg_mem = NULL,
};
#endif

//==================================================================================================
int ssi_timer_init(void) USE_NON_OPTIMIZED_FUNCTION;
int ssi_timer_start(unsigned long arg) USE_NON_OPTIMIZED_FUNCTION;
static void ssi_timer_clocksource_init(int timer5_id, const char *fck_source, const char *property) USE_NON_OPTIMIZED_FUNCTION;
static int omap_dm_ssi_timer_init(struct omap_dm_timer *timer, const char *fck_source, const char *property, int posted) USE_NON_OPTIMIZED_FUNCTION;
static irqreturn_t omap2_timer_interrupt(int irq, void *dev_id) USE_NON_OPTIMIZED_FUNCTION;

#if defined(USE_DEBUG_PORT)
static void timer_set_debug_port(bool status) USE_NON_OPTIMIZED_FUNCTION;
#endif


//==================================================================================================
static struct clock_event_device clockevent_timer = {
	.features		= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.rating			= 300,
	.min_delta_ns   = 1000,
	.max_delta_ns 	= 500000,
};

//--------------------------------------------------------------------------------------------------
static const struct of_device_id test_module_timer_of_match[] = {
    { .compatible   = "ti,am335x-ssi-timer" },
    {},
};

//--------------------------------------------------------------------------------------------------
static struct irqaction omap2_timer_irq = {
	.name		= "SSI_timer",
	.flags		= IRQF_TIMER | IRQF_ONESHOT,
	.handler	= omap2_timer_interrupt,
};

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
	ssi_timer_data.lew_rate = ((unsigned long)(1000000000UL/ssi_timer_data.clksrc.rate)); // 41 for ssi_timer_data.clksrc.rate==24MHz

	return 0;
}
EXPORT_SYMBOL_GPL(ssi_timer_init);

/*******************************************************************************/ /*!
 * @brief  ssi_timer_start function that will be start the timer.
 *
 * @arg arg : timer value
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
int ssi_timer_start(unsigned long arg)
{
    unsigned long cycles;

    cycles = (unsigned int)((arg*1000U)/ssi_timer_data.lew_rate); // 12195 for 500us
	__omap_dm_timer_load_start(&ssi_timer_data.clksrc,
                OMAP_TIMER_CTRL_ST, (unsigned int)(0xffffffffU - cycles),
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
static void ssi_timer_clocksource_init(int timer5_id, const char *fck_source, const char *property)
{
	int res;

	ssi_timer_data.clksrc.id = timer5_id;
	ssi_timer_data.clksrc.errata = OMAP_TIMER_ERRATA_I103_I767;
	
	res = omap_dm_ssi_timer_init(&ssi_timer_data.clksrc, fck_source, property, OMAP_TIMER_NONPOSTED);		   

	omap2_timer_irq.dev_id = &ssi_timer_data.clksrc;
	setup_irq(ssi_timer_data.clksrc.irq, &omap2_timer_irq);

	__omap_dm_timer_int_enable(&ssi_timer_data.clksrc, OMAP_TIMER_INT_OVERFLOW);

	clockevent_timer.cpumask = cpu_possible_mask;
	clockevent_timer.irq = omap_dm_timer_get_irq(&ssi_timer_data.clksrc);
	clockevents_config_and_register(&clockevent_timer, ssi_timer_data.clksrc.rate,
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
static int omap_dm_ssi_timer_init(struct omap_dm_timer *timer,
					 const char *fck_source,
					 const char *property,
					 int posted)
{
	const char *oh_name = NULL;
	struct device_node *np;
	struct omap_hwmod *oh;
	struct clk *src;
	int timer_ilr0_base_reg;

	np = omap_get_timer_dt(test_module_timer_of_match, property);
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
	ssi_timer_data.timer_irq = timer->irq;
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
	ssi_timer_data.timer_rate = timer->rate;
	timer->reserved = 1;

	/* set threshold of timer interrupt */
	of_property_read_s32(np, "interrupts", &ssi_timer_data.timer_hwirq_number);
	timer_ilr0_base_reg = INTC_INTC_ILR0_BASE_REG +
				  (ssi_timer_data.timer_hwirq_number * 4);
    ssi_timer_data.intc_ilr0_reg_mem = ioremap(timer_ilr0_base_reg, 4);
	writel_relaxed(1U << 4, ssi_timer_data.intc_ilr0_reg_mem);


	return 0;
}

/*******************************************************************************/ /*!
 * @brief  Interrupt function that is called when the timer has expired.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/

volatile int counter; 

static irqreturn_t omap2_timer_interrupt(int irq, void *dev_id)
{
	__omap_dm_timer_write_status(&ssi_timer_data.clksrc, OMAP_TIMER_INT_OVERFLOW);

#if defined(USE_DEBUG_PORT)
    timer_set_debug_port(true);
    for ( counter = 0; counter < 2000; counter++)
    {
        /* code */
    }
	timer_set_debug_port(false);
#endif

	return IRQ_HANDLED;
}

#if defined(USE_DEBUG_PORT)
/*******************************************************************************/ /*!
 * @brief  Sets the status of the debug port.
 *
 * @param status: true = set the port ; false = clear the port
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void timer_set_debug_port(bool status)
{
	if(status) {
		writel_relaxed(1U << 15, ssi_debug_port.set_reg_mem);
	}
	else {
		writel_relaxed(1U << 15, ssi_debug_port.clear_reg_mem);
	}
}
#endif

/*******************************************************************************/ /*!
 * @brief  Kernel module initialization function for the device driver.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int __init ssi_timer_module_init(void)
{		
#if defined(USE_DEBUG_PORT)
	/* debug port settings */
	ssi_debug_port.clear_reg_mem = ioremap(0x481AE190, 4);
	ssi_debug_port.set_reg_mem = ioremap(0x481AE194, 4);
#endif
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
	__omap_dm_timer_stop(&ssi_timer_data.clksrc, OMAP_TIMER_POSTED, ssi_timer_data.clksrc.rate);
    /* Release the IRQ handler */
    remove_irq(ssi_timer_data.timer_irq, &omap2_timer_irq);

    /* Release the timer */
    //ret = omap_dm_timer_free(&ssi_timer_data.clksrc);

	printk(KERN_INFO "Device Driver Remove...Done!!!\n");
}

module_init(ssi_timer_module_init);
module_exit(ssi_timer_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Berrux Kana");
MODULE_DESCRIPTION("SSITimer");