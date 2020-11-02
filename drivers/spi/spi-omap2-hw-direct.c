// SPDX-License-Identifier: GPL-2.0-only
/*
 * A spi driver that uses direct the OMAP2-MCSPI hardware without use of the queued spi driver class.
 *
 * Copyright (C) 2020 Martin Kaul <private>
 * Written by Martin Kaul <martin@familie-kaul.de>
 */

//#define ENABLE_DEBUGGING 1
#if defined(ENABLE_DEBUGGING)
#	define USE_NON_OPTIMIZED_FUNCTION __attribute__((optimize("-Og")))
#	define USE_INLINED_FUNCTION
#else
#	define USE_NON_OPTIMIZED_FUNCTION
#	define USE_INLINED_FUNCTION inline
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/io.h>

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#define OMAP2_HWDIRECT_SPI0_BASE_REGISTER 0x48030000
#define OMAP2_HWDIRECT_PRCM_BASE_REGISTER 0x44E00000
#define OMAP2_HWDIRECT_PINMUX_BASE_REGISTER 0x44E10000

#define HW_WR_FIELD32(regAddr, REG_FIELD, fieldVal)                            \
	(omap2_hw_direct_wr_field32_raw(regAddr, (uint32_t)REG_FIELD##_MASK,   \
					(uint32_t)REG_FIELD##_SHIFT,           \
					(uint32_t)fieldVal))

#define HW_SET_FIELD(regVal, REG_FIELD, fieldVal)                              \
	((regVal) = ((regVal) & (uint32_t)(~(uint32_t)REG_FIELD##_MASK)) |     \
		    ((((uint32_t)fieldVal) << (uint32_t)REG_FIELD##_SHIFT) &   \
		     (uint32_t)REG_FIELD##_MASK))

/*******************************************************************************/ /*!
 * @brief  Macros for the AM335x SPI ioctl handling
 *
 * HINT: this is a cloned declaration from spidev.h
 ***********************************************************************************/
#define SPI_MSGSIZE(N)                                                         \
	((((N) * (sizeof(struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) ?  \
		 ((N) * (sizeof(struct spi_ioc_transfer))) :                   \
		 0)
#define SPI_IOC_MESSAGE(N) _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])

#define SPI_IOC_MAGIC 'k'

#define SPI_IOC_RD_MODE _IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_WR_MODE _IOW(SPI_IOC_MAGIC, 1, __u8)

/* Read / Write SPI bit justification */
#define SPI_IOC_RD_LSB_FIRST _IOR(SPI_IOC_MAGIC, 2, __u8)
#define SPI_IOC_WR_LSB_FIRST _IOW(SPI_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
#define SPI_IOC_RD_BITS_PER_WORD _IOR(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_WR_BITS_PER_WORD _IOW(SPI_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
#define SPI_IOC_RD_MAX_SPEED_HZ _IOR(SPI_IOC_MAGIC, 4, __u32)
#define SPI_IOC_WR_MAX_SPEED_HZ _IOW(SPI_IOC_MAGIC, 4, __u32)

/* Read / Write of the SPI mode field */
#define SPI_IOC_RD_MODE32 _IOR(SPI_IOC_MAGIC, 5, __u32)
#define SPI_IOC_WR_MODE32 _IOW(SPI_IOC_MAGIC, 5, __u32)

// #define IOCTL_SPI_TRANSFER_DATA _IOW(SPI_IOC_MAGIC, 6, __u8)

struct spi_ioc_transfer {
	__u64 tx_buf;
	__u64 rx_buf;

	__u32 len;
	__u32 speed_hz;

	__u16 delay_usecs;
	__u8 bits_per_word;
	__u8 cs_change;
	__u8 tx_nbits;
	__u8 rx_nbits;
	__u8 word_delay_usecs;
	__u8 pad;
};

/*******************************************************************************/ /*!
 * @brief  Macros for the AM335x SPI mode handling
 ***********************************************************************************/
#define SPI_CPHA 0x01
#define SPI_CPOL 0x02

#define SPI_MODE_0 (0 | 0)
#define SPI_MODE_1 (0 | SPI_CPHA)
#define SPI_MODE_2 (SPI_CPOL | 0)
#define SPI_MODE_3 (SPI_CPOL | SPI_CPHA)

#define SPI_CS_HIGH 0x04
#define SPI_LSB_FIRST 0x08
#define SPI_3WIRE 0x10
#define SPI_LOOP 0x20
#define SPI_NO_CS 0x40
#define SPI_READY 0x80
#define SPI_TX_DUAL 0x100
#define SPI_TX_QUAD 0x200
#define SPI_RX_DUAL 0x400
#define SPI_RX_QUAD 0x800

/*******************************************************************************/ /*!
 * @brief  Macros for the AM335x SPI handling
 ***********************************************************************************/

#define OMAP2_HWDIRECT_SPIGCR1 0x104
#define OMAP2_HWDIRECT_CHCONF0 0x12c
#define OMAP2_HWDIRECT_CHSTAT0 0x130
#define OMAP2_HWDIRECT_CHCTRL0 0x134
#define OMAP2_HWDIRECT_TX0 0x138
#define OMAP2_HWDIRECT_RX0 0x13c

#define ENABLE (0x01000000U) // bit 24

#ifndef TRUE
#define TRUE (1U)
#endif

#ifndef FALSE
#define FALSE (0U)
#endif

#ifndef S_PASS
#define S_PASS (0)
#endif

#ifndef E_FAIL
#define E_FAIL (-((int32_t)1))
#endif

#define MCSPI_CHCONF(m) (0x12cU + ((m)*0x14U))
#define MCSPI_CHCTRL(m) (0x134U + ((m)*0x14U))
#define MCSPI_CHSTAT(m) (0x130U + ((m)*0x14U))
#define MCSPI_SYSCONFIG (0x110U)
#define MCSPI_SYSSTS (0x114U)
#define MCSPI_MODULCTRL (0x128U)
#define MCSPI_WAKEUPEN (0x120U)

#define MCSPI_SYSSTS_RESETDONE_SHIFT (0U)
#define MCSPI_SYSSTS_RESETDONE_MASK (0x00000001U)
#define MCSPI_SYSSTS_RESETDONE_INPROGRESS (0U)
#define MCSPI_SYSSTS_RESETDONE_COMPLETED (1U)

#define MCSPI_SYSCONFIG_SOFTRESET_SHIFT (1U)
#define MCSPI_SYSCONFIG_SOFTRESET_MASK (0x00000002U)
#define MCSPI_SYSCONFIG_SOFTRESET_OFF (0U)
#define MCSPI_SYSCONFIG_SOFTRESET_ON (1U)

#define MCSPI_CHCONF_CLKG_SHIFT (29U)
#define MCSPI_CHCONF_CLKG_MASK (0x20000000U)
#define MCSPI_CHCONF_CLKG_ONECYCLE (1U)
#define MCSPI_CHCONF_CLKG_POWERTWO (0U)

#define MCSPI_CHCTRL_EXTCLK_SHIFT (8U)
#define MCSPI_CHCTRL_EXTCLK_MASK (0x0000ff00U)
#define MCSPI_CHCTRL_EXTCLK_EXTZERO (0U)
#define MCSPI_CHCTRL_EXTCLK_EXTONE (1U)
#define MCSPI_CHCTRL_EXTCLK_EXT4080 (255U)

#define MCSPI_CHCONF_CLKD_SHIFT (2U)
#define MCSPI_CHCONF_CLKD_MASK (0x0000003cU)
#define MCSPI_CHCONF_CLKD_DIVBY4K (12U)
#define MCSPI_CHCONF_CLKD_DIVBY8 (3U)
#define MCSPI_CHCONF_CLKD_DIVBY8K (13U)
#define MCSPI_CHCONF_CLKD_DIVBY2K (11U)
#define MCSPI_CHCONF_CLKD_DIVBY1K (10U)
#define MCSPI_CHCONF_CLKD_DIVBY512 (9U)
#define MCSPI_CHCONF_CLKD_DIVBY32K (15U)
#define MCSPI_CHCONF_CLKD_DIVBY4 (2U)
#define MCSPI_CHCONF_CLKD_DIVBY1 (0U)
#define MCSPI_CHCONF_CLKD_DIVBY64 (6U)
#define MCSPI_CHCONF_CLKD_DIVBY256 (8U)
#define MCSPI_CHCONF_CLKD_DIVBY128 (7U)
#define MCSPI_CHCONF_CLKD_DIVBY32 (5U)
#define MCSPI_CHCONF_CLKD_DIVBY16K (14U)
#define MCSPI_CHCONF_CLKD_DIVBY16 (4U)
#define MCSPI_CHCONF_CLKD_DIVBY2 (1U)

#define MCSPI_CHCONF_PHA_SHIFT (0U)
#define MCSPI_CHCONF_PHA_MASK (0x00000001U)
#define MCSPI_CHCONF_PHA_ODD (0U)
#define MCSPI_CHCONF_PHA_EVEN (1U)

#define MCSPI_CHCONF_POL_SHIFT (1U)
#define MCSPI_CHCONF_POL_MASK (0x00000002U)
#define MCSPI_CHCONF_POL_ACTIVELOW (1U)
#define MCSPI_CHCONF_POL_ACTIVEHIGH (0U)

#define MCSPI_MODULCTRL_MS_SHIFT (2U)
#define MCSPI_MODULCTRL_MS_MASK (0x00000004U)
#define MCSPI_MODULCTRL_MS_MASTER (0U)
#define MCSPI_MODULCTRL_MS_SLAVE (1U)

#define MCSPI_MODULCTRL_SINGLE_SHIFT (0U)
#define MCSPI_MODULCTRL_SINGLE_MASK (0x00000001U)
#define MCSPI_MODULCTRL_SINGLE_MODMULTI (0U)
#define MCSPI_MODULCTRL_SINGLE_MODSINGLE (1U)

#define MCSPI_CHCONF_SPIENSLV_SHIFT (21U)
#define MCSPI_CHCONF_SPIENSLV_MASK (0x00600000U)
#define MCSPI_CHCONF_SPIENSLV_SPIEN2 (2U)
#define MCSPI_CHCONF_SPIENSLV_SPIEN3 (3U)
#define MCSPI_CHCONF_SPIENSLV_SPIEN0 (0U)
#define MCSPI_CHCONF_SPIENSLV_SPIEN1 (1U)

#define MCSPI_CHCONF_IS_SHIFT (18U)
#define MCSPI_CHCONF_IS_MASK (0x00040000U)
#define MCSPI_CHCONF_IS_LINE0 (0U)
#define MCSPI_CHCONF_IS_LINE1 (1U)

#define MCSPI_CHCONF_DPE1_SHIFT (17U)
#define MCSPI_CHCONF_DPE1_MASK (0x00020000U)
#define MCSPI_CHCONF_DPE1_ENABLED (0U)
#define MCSPI_CHCONF_DPE1_DISABLED (1U)

#define MCSPI_CHCONF_DPE0_SHIFT (16U)
#define MCSPI_CHCONF_DPE0_MASK (0x00010000U)
#define MCSPI_CHCONF_DPE0_DISABLED (1U)
#define MCSPI_CHCONF_DPE0_ENABLED (0U)

#define MCSPI_CHCONF_TRM_SHIFT (12U)
#define MCSPI_CHCONF_TRM_MASK (0x00003000U)
#define MCSPI_CHCONF_TRM_TRANSONLY (2U)
#define MCSPI_CHCONF_TRM_RSVD (3U)
#define MCSPI_CHCONF_TRM_TRANSRECEI (0U)
#define MCSPI_CHCONF_TRM_RECEIVONLY (1U)

#define MCSPI_CHCONF_SBE_SHIFT (23U)
#define MCSPI_CHCONF_SBE_MASK (0x00800000U)
#define MCSPI_CHCONF_SBE_DISABLED (0U)
#define MCSPI_CHCONF_SBE_ENABLED (1U)

#define MCSPI_CHCTRL_EN_SHIFT (0U)
#define MCSPI_CHCTRL_EN_MASK (0x00000001U)
#define MCSPI_CHCTRL_EN_ACT (1U)
#define MCSPI_CHCTRL_EN_NACT (0U)

#define MCSPI_CHCONF_EPOL_SHIFT (6U)
#define MCSPI_CHCONF_EPOL_MASK (0x00000040U)
#define MCSPI_CHCONF_EPOL_ACTIVELOW (1U)
#define MCSPI_CHCONF_EPOL_ACTIVEHIGH (0U)

#define MCSPI_CHCONF_WL_SHIFT (7U)
#define MCSPI_CHCONF_WL_MASK (0x00000f80U)

#define MCSPI_CHCONF_FORCE_SHIFT (20U)
#define MCSPI_CHCONF_FORCE_MASK (0x00100000U)
#define MCSPI_CHCONF_FORCE_DEASSERT (0U)
#define MCSPI_CHCONF_FORCE_ASSERT (1U)

#define MCSPI_WAKEUPEN_WKEN_SHIFT (0U)
#define MCSPI_WAKEUPEN_WKEN_MASK (0x00000001U)
#define MCSPI_WAKEUPEN_WKEN_NOWAKEUP (0U)
#define MCSPI_WAKEUPEN_WKEN_WAKEUP (1U)

#define MCSPI_CHSTAT_TXS_SHIFT (1U)
#define MCSPI_CHSTAT_TXS_MASK (0x00000002U)
#define MCSPI_CHSTAT_TXS_EMPTY (1U)
#define MCSPI_CHSTAT_TXS_FULL (0U)

#define MCSPI_CHSTAT_RXS_SHIFT (0U)
#define MCSPI_CHSTAT_RXS_MASK (0x00000001U)
#define MCSPI_CHSTAT_RXS_EMPTY (0U)
#define MCSPI_CHSTAT_RXS_FULL (1U)

#define MCSPI_CLKDIV_MASK (0xFU)

#define MCSPI_IN_CLK (48000000u)
#define MCSPI_OUT_FREQ (1000000u)
#define MCSPI_MAX_SPEED (24000000u)
#define MCSPI_CH_NUM (0u)

#define MCSPI_TX(m) (0x138U + ((m)*0x14U))

/*******************************************************************************/ /*!
 * @brief  Macros for the AM335x clock module
 ***********************************************************************************/
#define PRCM_MODULEMODE_ENABLE (2U)
#define PRCM_MODULEMODE_MASK (3U)
#define PRCM_IDLE_ST_MASK (0x00030000U)
#define PRCM_IDLE_ST_SHIFT (16U)

#define PRCM_MODULE_IDLEST_FUNC (0U)
#define PRCM_MODULE_IDLEST_TRANS (1U)
#define PRCM_MODULE_IDLEST_IDLE (2U)
#define PRCM_MODULE_IDLEST_DISABLE (3U)

#define CM_PER_L3_CLKSTCTRL (0xc)
#define CM_PER_SPI0_CLKCTRL (0x4c)

#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK (0x00000010u)

/*******************************************************************************/ /*!
 * @brief  Declarations for the AM335x pin-mux
 ***********************************************************************************/
struct pinmuxPerCfg {
	uint16_t pinOffset;
	/**< Register offset for configuring the pin */
	uint16_t optParam;
	/**< Optional param to hold the peripheral specific data */
	uint32_t pinSettings;
	/**< Value to be configured,
		  - Active mode configurations like Mux mode, pull config, Rx enable &
		    slew rate
		  - Sleep mode configurations like Deep sleep enable, o/p value &
	        pull config
		  - Wake up enable/disable
		Refer TRM section "7.2.1 Pad Control Registers" for more details.
	*/
};

/*******************************************************************************/ /*!
 * @brief  Declarations for the AM335x spi
 ***********************************************************************************/
enum mcspiClkMode {
	MCSPI_CLK_MODE_0 = (MCSPI_CHCONF_POL_ACTIVEHIGH | MCSPI_CHCONF_PHA_ODD),
	/**< MCSPI clock mode 0, POL = 0, PHA = 0. */
	MCSPI_CLK_MODE_1 =
		(MCSPI_CHCONF_POL_ACTIVEHIGH | MCSPI_CHCONF_PHA_MASK),
	/**< MCSPI clock mode 1, POL = 0, PHA = 1. */
	MCSPI_CLK_MODE_2 = (MCSPI_CHCONF_POL_MASK | MCSPI_CHCONF_PHA_ODD),
	/**< MCSPI clock mode 2, POL = 1, PHA = 0. */
	MCSPI_CLK_MODE_3 = (MCSPI_CHCONF_POL_MASK | MCSPI_CHCONF_PHA_MASK)
	/**< MCSPI clock mode 3, POL = 1, PHA = 1.*/
};

//--------------------------------------------------------------------------------------------------
enum mcspiMode {
	MCSPI_MODE_MASTER = MCSPI_MODULCTRL_MS_MASTER,
	/**< MCSPI Master mode of operation. */
	MCSPI_MODE_SLAVE = MCSPI_MODULCTRL_MS_SLAVE
	/**< MCSPI Slave mode of operation. */
};

//--------------------------------------------------------------------------------------------------
enum mcspiTransferMode {
	MCSPI_TRANSFER_MODE_TX_RX = MCSPI_CHCONF_TRM_TRANSRECEI,
	/**< Enable Tx and Rx mode. */
	MCSPI_TRANSFER_MODE_RX_ONLY = MCSPI_CHCONF_TRM_RECEIVONLY,
	/**< Enable Rx only mode. */
	MCSPI_TRANSFER_MODE_TX_ONLY = MCSPI_CHCONF_TRM_TRANSONLY
	/**< Enable Tx only mode. */
};

//--------------------------------------------------------------------------------------------------
enum mcspiDataLineCommMode {
	MCSPI_DATA_LINE_COMM_MODE_0 =
		(MCSPI_CHCONF_IS_LINE0 | MCSPI_CHCONF_DPE1_ENABLED |
		 MCSPI_CHCONF_DPE0_ENABLED),
	/**< Pin mode 0 */
	MCSPI_DATA_LINE_COMM_MODE_1 =
		(MCSPI_CHCONF_IS_LINE0 | MCSPI_CHCONF_DPE1_ENABLED |
		 MCSPI_CHCONF_DPE0_MASK),
	/**< Pin mode 1 */
	MCSPI_DATA_LINE_COMM_MODE_2 =
		(MCSPI_CHCONF_IS_LINE0 | MCSPI_CHCONF_DPE1_MASK |
		 MCSPI_CHCONF_DPE0_ENABLED),
	/**< Pin mode 2 */
	MCSPI_DATA_LINE_COMM_MODE_3 =
		(MCSPI_CHCONF_IS_LINE0 | MCSPI_CHCONF_DPE1_MASK |
		 MCSPI_CHCONF_DPE0_MASK),
	/**< Pin mode 3 */
	MCSPI_DATA_LINE_COMM_MODE_4 =
		(MCSPI_CHCONF_IS_MASK | MCSPI_CHCONF_DPE1_ENABLED |
		 MCSPI_CHCONF_DPE0_ENABLED),
	/**< Pin mode 4 */
	MCSPI_DATA_LINE_COMM_MODE_5 =
		(MCSPI_CHCONF_IS_MASK | MCSPI_CHCONF_DPE1_ENABLED |
		 MCSPI_CHCONF_DPE0_MASK),
	/**< Pin mode 5 */
	MCSPI_DATA_LINE_COMM_MODE_6 =
		(MCSPI_CHCONF_IS_MASK | MCSPI_CHCONF_DPE1_MASK |
		 MCSPI_CHCONF_DPE0_ENABLED),
	/**< Pin mode 6 */
	MCSPI_DATA_LINE_COMM_MODE_7 =
		(MCSPI_CHCONF_IS_MASK | MCSPI_CHCONF_DPE1_MASK |
		 MCSPI_CHCONF_DPE0_MASK)
	/**< Pin mode 7 */
};

//--------------------------------------------------------------------------------------------------
enum mcspiChNum {
	MCSPI_CH_NUM_MIN,
	/**< Minimum value of the enum. */
	MCSPI_CH_NUM_0 = MCSPI_CH_NUM_MIN,
	/**< MCSPI channel 0. */
	MCSPI_CH_NUM_1,
	/**< MCSPI channel 1. */
	MCSPI_CH_NUM_2,
	/**< MCSPI channel 2. */
	MCSPI_CH_NUM_3,
	/**< MCSPI channel 3.*/
	MCSPI_CH_NUM_MAX = MCSPI_CH_NUM_3
	/**< Maximum value of the enum. */
};

//--------------------------------------------------------------------------------------------------
enum pinOffsets {
	PIN_SPI0_SCLK = 0x0950U,
	PIN_SPI0_D0 = 0x0954U,
	PIN_SPI0_D1 = 0x0958U,
	PIN_SPI0_CS0 = 0x095cU,
	PIN_SPI0_CS1 = 0x0960U,
};

//--------------------------------------------------------------------------------------------------
#define PINMUX_INVALID_PIN (-1)

#define PIN_MODE(mode) (mode)
#define PIN_PULL_UD_EN (0x1U << 3U)
#define PIN_PULL_TYPE_SEL (0x1U << 4U)
#define PIN_RX_ACTIVE (0x1U << 5U)
#define PIN_SLEW_SLOW (0x1U << 6U)

//--------------------------------------------------------------------------------------------------
static struct pinmuxPerCfg gMcspi0PinCfg[] = {
	{ /* MySPI1 -> spi0_sclk -> A18 */
	  PIN_SPI0_SCLK, 0,
	  (PIN_MODE(0) |
	   ((PIN_PULL_TYPE_SEL | PIN_RX_ACTIVE) & (~PIN_PULL_UD_EN))) },
	{ /* MySPI1 -> spi0_d0 -> B18 */
	  PIN_SPI0_D0, 0,
	  (PIN_MODE(0) |
	   ((PIN_PULL_TYPE_SEL | PIN_RX_ACTIVE) & (~PIN_PULL_UD_EN))) },
	{ /* MySPI1 -> spi0_d1 -> B17 */
	  PIN_SPI0_D1, 0,
	  (PIN_MODE(0) |
	   ((PIN_PULL_TYPE_SEL) & (~PIN_PULL_UD_EN & ~PIN_RX_ACTIVE))) },
	{ /* MySPI1 -> spi0_cs0 -> A16 */
	  PIN_SPI0_CS0, 0,
	  (PIN_MODE(0) |
	   ((PIN_PULL_TYPE_SEL | PIN_RX_ACTIVE) & (~PIN_PULL_UD_EN))) },
	{ PINMUX_INVALID_PIN }
};

//==================================================================================================
struct Omap2HwDirectDeviceInfo {
	dev_t device;
	struct cdev driver_cdev;
	struct class *dev_class;
};

//--------------------------------------------------------------------------------------------------
struct Omap2HwDirectSpiData {
	void __iomem *base;
	uint32_t max_speed;
	uint8_t lsb_first;
	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
};

//--------------------------------------------------------------------------------------------------
struct Omap2HwDirectPrcmData {
	void __iomem *base;
};

//--------------------------------------------------------------------------------------------------
struct Omap2HwDirectPinMuxData {
	void __iomem *base;
};

//==================================================================================================
static struct Omap2HwDirectDeviceInfo omap2_hw_direct_device_info = {
	.device = 0,
	.dev_class = NULL,
};

static struct Omap2HwDirectSpiData omap2_hw_direct_spi_data = {
	.base = NULL,
	.max_speed = MCSPI_MAX_SPEED,
	.lsb_first = 0,
	.tx_buffer = NULL,
	.rx_buffer = NULL,
};

static struct Omap2HwDirectPrcmData omap2_hw_direct_prcm_data = {
	.base = NULL,
};

static struct Omap2HwDirectPinMuxData omap2_hw_direct_pinmux_data = {
	.base = NULL,
};

//==================================================================================================
static int omap2_hw_direct_open(struct inode *inode, struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static int omap2_hw_direct_release(struct inode *inode, struct file *file) USE_NON_OPTIMIZED_FUNCTION;
static long omap2_hw_direct_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg) USE_NON_OPTIMIZED_FUNCTION;
static void prcm_enable_module(void __iomem *base_addr, uint32_t clk_ctrl_reg,
			       uint32_t clk_st_ctrl_reg, uint32_t clk_act_mask) USE_NON_OPTIMIZED_FUNCTION;
static void prcm_disable_module(void __iomem *base_addr, uint32_t clk_ctrl_reg,
				uint32_t clk_st_ctrl_reg,
				uint32_t clk_act_mask) USE_NON_OPTIMIZED_FUNCTION;
static void pinmux_initialize(void *param) USE_NON_OPTIMIZED_FUNCTION;
static void spi_reset(void) USE_NON_OPTIMIZED_FUNCTION;
void spi_clk_config(uint32_t chNum, uint32_t spiInClk, uint32_t spiOutClk,
		    uint32_t clkMode) USE_NON_OPTIMIZED_FUNCTION;
static int32_t spi_mode_config(uint32_t chNum, uint32_t spiMode,
			       uint32_t chMode, uint32_t trMode,
			       uint32_t pinMode) USE_NON_OPTIMIZED_FUNCTION;
static int32_t spi_word_length(uint32_t chNum, uint32_t wordLength) USE_NON_OPTIMIZED_FUNCTION;
static void spi_start_bit_enable(uint32_t chNum, uint32_t enableStartBit) USE_NON_OPTIMIZED_FUNCTION;
static void spi_ch_enable(uint32_t chNum, uint32_t enableCh) USE_NON_OPTIMIZED_FUNCTION;
static void spi_wakeup_enable(uint32_t enableWu) USE_NON_OPTIMIZED_FUNCTION;
static int spi_get_mode(uint32_t chNum) USE_NON_OPTIMIZED_FUNCTION;
static void spi_set_mode(uint32_t chNum, int mode) USE_NON_OPTIMIZED_FUNCTION;
static int spi_get_bits_per_word(uint32_t chNum) USE_NON_OPTIMIZED_FUNCTION;
static void spi_set_bits_per_word(uint32_t chNum, int wordLength) USE_NON_OPTIMIZED_FUNCTION;
static void spi_set_speed(uint32_t chNum, uint32_t spiInClk,
			  uint32_t spiOutClk) USE_NON_OPTIMIZED_FUNCTION;
static void spi_set_cs(uint32_t chNum, bool enable);
static struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
		       unsigned *n_ioc) USE_NON_OPTIMIZED_FUNCTION;
static int spidev_message(uint32_t chNum, struct spi_ioc_transfer *u_xfers,
			  unsigned n_xfers) USE_NON_OPTIMIZED_FUNCTION;

//==================================================================================================
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = omap2_hw_direct_open,
	.unlocked_ioctl = omap2_hw_direct_ioctl,
	.release = omap2_hw_direct_release,
};

//==================================================================================================

static USE_INLINED_FUNCTION uint32_t omap2_hw_direct_rd_reg32(void __iomem *reg_addr)
{
	uint32_t val = ioread32(reg_addr);
	// printk(KERN_INFO "AAA rd: base=%x v=%x \n", (int)reg_addr, val);
	return val;
}

//--------------------------------------------------------------------------------------------------
static USE_INLINED_FUNCTION void omap2_hw_direct_wr_reg32(void __iomem *reg_addr,
					    uint32_t value)
{
	// printk(KERN_INFO "AAA wd: base=%x v=%x \n", (int)reg_addr, value);
	iowrite32(value, reg_addr);
}

//--------------------------------------------------------------------------------------------------
static USE_INLINED_FUNCTION uint16_t omap2_hw_direct_rd_reg16(void __iomem *reg_addr)
{
	return ioread16(reg_addr);
}

//--------------------------------------------------------------------------------------------------
static USE_INLINED_FUNCTION void omap2_hw_direct_wr_reg16(void __iomem *reg_addr,
					    uint16_t value)
{
	iowrite16(value, reg_addr);
}

//--------------------------------------------------------------------------------------------------
static USE_INLINED_FUNCTION uint8_t omap2_hw_direct_rd_reg8(void __iomem *reg_addr)
{
	return ioread8(reg_addr);
}

//--------------------------------------------------------------------------------------------------
static USE_INLINED_FUNCTION void omap2_hw_direct_wr_reg8(void __iomem *reg_addr,
					   uint8_t value)
{
	iowrite8(value, reg_addr);
}

//--------------------------------------------------------------------------------------------------
static USE_INLINED_FUNCTION void omap2_hw_direct_wr_field32_raw(void __iomem *reg_addr,
						  uint32_t mask, uint32_t shift,
						  uint32_t value)
{
	uint32_t reg_value = ioread32(reg_addr);
	// printk(KERN_INFO "AAA fr: base=%x m=%x s=%d v=%x\n", (int)reg_addr,
	//        mask, shift, value);
	reg_value &= (~mask);
	reg_value |= (value << shift) & mask;
	// printk(KERN_INFO "AAA fw: base=%x m=%x s=%d v=%x\n", (int)reg_addr,
	//        mask, shift, value);
	iowrite32(reg_value, reg_addr);

	return;
}

/*******************************************************************************/ /*!
 * @brief  Open function that will be called when the device driver is opened.
 *
 * @return 0: success
 * @exception
 * @globals
 ***********************************************************************************/
static int omap2_hw_direct_open(struct inode *inode, struct file *file)
{
	prcm_enable_module(omap2_hw_direct_prcm_data.base, CM_PER_SPI0_CLKCTRL,
			   CM_PER_L3_CLKSTCTRL,
			   CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK);
	pinmux_initialize(NULL);
	spi_reset();
	spi_wakeup_enable(TRUE);
	spi_clk_config(MCSPI_CH_NUM, MCSPI_IN_CLK, MCSPI_OUT_FREQ,
		       MCSPI_CLK_MODE_2);
	spi_mode_config(MCSPI_CH_NUM_0, MCSPI_MODE_MASTER,
			MCSPI_MODULCTRL_SINGLE_MODSINGLE,
			MCSPI_CHCONF_TRM_TRANSRECEI,
			MCSPI_DATA_LINE_COMM_MODE_1);
	spi_word_length(MCSPI_CH_NUM_0, 8);

	omap2_hw_direct_rd_reg32(omap2_hw_direct_spi_data.base +
				 MCSPI_CHCONF(0));

	spi_set_cs(MCSPI_CH_NUM_0, FALSE); 
 	spi_ch_enable(MCSPI_CH_NUM_0, FALSE);

	// printk(KERN_INFO "omap2_hw_direct_open\n");

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
	spi_wakeup_enable(FALSE);
	prcm_disable_module(omap2_hw_direct_prcm_data.base, CM_PER_SPI0_CLKCTRL,
			    CM_PER_L3_CLKSTCTRL,
			    CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK);

	// printk(KERN_INFO "omap2_hw_direct_release\n");
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
static long omap2_hw_direct_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	uint32_t tmp;
	int retval = 0;
	unsigned n_ioc;
	struct spi_ioc_transfer *ioc;

	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC) {
		return -ENOTTY;
	}

	switch (cmd) {
	case SPI_IOC_RD_LSB_FIRST:
		retval = put_user(omap2_hw_direct_spi_data.lsb_first,
				  (__u8 __user *)arg);
		/* not supported by hardware */
		break;
	case SPI_IOC_WR_LSB_FIRST:
		/* not supported by hardware*/
		retval = get_user(omap2_hw_direct_spi_data.lsb_first,
				  (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = put_user(spi_get_bits_per_word(MCSPI_CH_NUM),
				  (__u8 __user *)arg);
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			spi_set_bits_per_word(MCSPI_CH_NUM, tmp);
		}
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = put_user(omap2_hw_direct_spi_data.max_speed,
				  (__u32 __user *)arg);
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = get_user(omap2_hw_direct_spi_data.max_speed,
				  (__u32 __user *)arg);
		break;
	case SPI_IOC_RD_MODE:
		retval = put_user(spi_get_mode(MCSPI_CH_NUM),
				  (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MODE32:
		retval = put_user(spi_get_mode(MCSPI_CH_NUM),
				  (__u32 __user *)arg);
		break;
	case SPI_IOC_WR_MODE:
	case SPI_IOC_WR_MODE32:
		if (cmd == SPI_IOC_WR_MODE) {
			retval = get_user(tmp, (u8 __user *)arg);
		} else {
			retval = get_user(tmp, (u32 __user *)arg);
		}
		if (retval == 0) {
			spi_set_mode(MCSPI_CH_NUM, tmp);
		}
		break;
	default:
		// transfer messages
		ioc = spidev_get_ioc_message(
			cmd, (struct spi_ioc_transfer __user *)arg, &n_ioc);
		if (IS_ERR(ioc)) {
			retval = PTR_ERR(ioc);
			break;
		}
		if (!ioc)
			break; /* n_ioc is also 0 */

		/* translate to spi_message, execute */
		retval = spidev_message(MCSPI_CH_NUM, ioc, n_ioc);
		kfree(ioc);
		break;
	}
	return retval;
}

/*******************************************************************************/ /*!
 * @brief  Enables the clock of the given hw module.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void prcm_enable_module(void __iomem *base_addr, uint32_t clk_ctrl_reg,
			       uint32_t clk_st_ctrl_reg, uint32_t clk_act_mask)
{
	/* Enable the module */
	omap2_hw_direct_wr_reg32(base_addr + clk_ctrl_reg,
				 PRCM_MODULEMODE_ENABLE);
	/* Check for module enable status */
	while (PRCM_MODULEMODE_ENABLE !=
	       (omap2_hw_direct_rd_reg32(base_addr + clk_ctrl_reg) &
		PRCM_MODULEMODE_MASK))
		;
	/* Check clock activity - ungated */
	while (clk_act_mask !=
	       (omap2_hw_direct_rd_reg32(base_addr + clk_st_ctrl_reg) &
		clk_act_mask))
		;
	/* Check idle status value - should be in functional state */
	while ((PRCM_MODULE_IDLEST_FUNC << PRCM_IDLE_ST_SHIFT) !=
	       (omap2_hw_direct_rd_reg32(base_addr + clk_ctrl_reg) &
		PRCM_IDLE_ST_MASK))
		;
}

/*******************************************************************************/ /*!
 * @brief  Disables the clock of the given hw module.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
static void prcm_disable_module(void __iomem *base_addr, uint32_t clk_ctrl_reg,
				uint32_t clk_st_ctrl_reg, uint32_t clk_act_mask)
{
	uint32_t value;
	/* Diable the module */
	value = omap2_hw_direct_rd_reg32(base_addr + clk_ctrl_reg);
	value &= ~PRCM_MODULEMODE_MASK; /* clear enable bits */
	omap2_hw_direct_wr_reg32(base_addr + clk_ctrl_reg, value);
	/* Check for module enable status */
	while (PRCM_MODULEMODE_ENABLE ==
	       (omap2_hw_direct_rd_reg32(base_addr + clk_ctrl_reg) &
		PRCM_MODULEMODE_MASK))
		;
	/* Check clock activity - ungated */
	while (clk_act_mask !=
	       (omap2_hw_direct_rd_reg32(base_addr + clk_st_ctrl_reg) &
		clk_act_mask))
		;
	/* Check idle status value - should be in functional state */
	while ((PRCM_MODULE_IDLEST_DISABLE << PRCM_IDLE_ST_SHIFT) !=
	       (omap2_hw_direct_rd_reg32(base_addr + clk_ctrl_reg) &
		PRCM_IDLE_ST_MASK))
		;
}

/*******************************************************************************/ /*!
 * @brief  Initializes the SPI pinmux.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void pinmux_initialize(void *param)
{
	struct pinmuxPerCfg *pInstanceData = &gMcspi0PinCfg[0];
	int index;
	void __iomem *ctrlModBase = omap2_hw_direct_pinmux_data.base;

	for (index = 0;
	     ((uint16_t)PINMUX_INVALID_PIN != pInstanceData[index].pinOffset);
	     index++) {
		if (NULL != param) {
			if (pInstanceData[index].optParam ==
			    *(uint16_t *)param) {
				omap2_hw_direct_wr_reg32(
					(ctrlModBase +
					 pInstanceData[index].pinOffset),
					pInstanceData[index].pinSettings);
				break;
			}
		} else {
			omap2_hw_direct_wr_reg32(
				(ctrlModBase + pInstanceData[index].pinOffset),
				pInstanceData[index].pinSettings);
		}
	}
}

/*******************************************************************************/ /*!
 * @brief  Resets the SPI device
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_reset(void)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;

	/* Perform soft reset on the MCSPI controller. */
	HW_WR_FIELD32((baseAddr + MCSPI_SYSCONFIG), MCSPI_SYSCONFIG_SOFTRESET,
		      MCSPI_SYSCONFIG_SOFTRESET_ON);

	/* Stay in the loop until reset is done. */
	while (!(MCSPI_SYSSTS_RESETDONE_MASK &
		 omap2_hw_direct_rd_reg32(baseAddr + MCSPI_SYSSTS)))
		;
}

/*******************************************************************************/ /*!
 * @brief  Configures the SPI clock settings.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_clk_config(uint32_t chNum, uint32_t spiInClk, uint32_t spiOutClk,
		    uint32_t clkMode)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t clkRatio = 0U;
	uint32_t polarity = 0U;
	uint32_t regVal = 0U;
	uint32_t extClk = 0U;
	uint32_t clkDiv = 0U;
	uint32_t phase = 0U;

	/* Calculate the value of clkRatio. */
	clkRatio = (spiInClk / spiOutClk);

	/* If clkRatio is not a power of 2, set granularity of 1 clock cycle */
	if (0U != (clkRatio & (clkRatio - 1U))) {
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_CLKG, MCSPI_CHCONF_CLKG_ONECYCLE);

		/* Calculate the ratios clkD and extClk based on fClk */
		extClk = (clkRatio - 1U) >> 4U;
		clkDiv = (clkRatio - 1U) & MCSPI_CLKDIV_MASK;

		HW_WR_FIELD32((baseAddr + MCSPI_CHCTRL(chNum)),
			      MCSPI_CHCTRL_EXTCLK, extClk);
	} else {
		/* Clock granularity of power of 2. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_CLKG, MCSPI_CHCONF_CLKG_POWERTWO);

		while (1U != clkRatio) {
			clkRatio /= 2U;
			clkDiv++;
		}
	}

	/* Configure the clkD field of MCSPI_CHCONF register. */
	HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_CLKD,
		      clkDiv);

	/* Configure the Phase and Polarity values. */
	phase = ((clkMode & MCSPI_CHCONF_PHA_MASK) >> MCSPI_CHCONF_PHA_SHIFT);
	polarity =
		((clkMode & MCSPI_CHCONF_POL_MASK) >> MCSPI_CHCONF_POL_SHIFT);
	regVal = omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHCONF(chNum));
	HW_SET_FIELD(regVal, MCSPI_CHCONF_PHA, phase);
	HW_SET_FIELD(regVal, MCSPI_CHCONF_POL, polarity);
	omap2_hw_direct_wr_reg32((baseAddr + MCSPI_CHCONF(chNum)), regVal);
}

/*******************************************************************************/ /*!
 * @brief  Configures the SPI mode settings.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
int32_t spi_mode_config(uint32_t chNum, uint32_t spiMode, uint32_t chMode,
			uint32_t trMode, uint32_t pinMode)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t retVal = E_FAIL;
	uint32_t inputSelect = 0U;
	uint32_t dataLine1 = 0U;
	uint32_t dataLine0 = 0U;
	uint32_t regVal;

	if (MCSPI_MODE_MASTER == spiMode) {
		/* Configure MCSPI controller in Master mode. */
		HW_WR_FIELD32((baseAddr + MCSPI_MODULCTRL), MCSPI_MODULCTRL_MS,
			      MCSPI_MODULCTRL_MS_MASTER);

		/* Configure the MCSPI controller single/multichannel mode. */
		HW_WR_FIELD32((baseAddr + MCSPI_MODULCTRL),
			      MCSPI_MODULCTRL_SINGLE, chMode);
	} else if (MCSPI_MODE_SLAVE == spiMode) {
		/* Configure MCSPI controller in Slave mode. */
		HW_WR_FIELD32((baseAddr + MCSPI_MODULCTRL), MCSPI_MODULCTRL_MS,
			      MCSPI_MODULCTRL_MS_SLAVE);

		/* Only channel 0 is supported in Slave mode. */
		chNum = 0U;

		/* Configure the SPIENSLV field. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_SPIENSLV, chNum);
	} else {
		/* Perform nothing. Invalid parameter. */
		printk(KERN_INFO "ERROR\n");
	}

	/* Configure the MCSPI controller in Tx/Rx modes. */
	HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_TRM,
		      trMode);

	if (((MCSPI_TRANSFER_MODE_TX_RX == trMode) &&
	     (MCSPI_DATA_LINE_COMM_MODE_3 == pinMode)) ||
	    ((MCSPI_TRANSFER_MODE_TX_ONLY == trMode) &&
	     (MCSPI_DATA_LINE_COMM_MODE_3 == pinMode)) ||
	    ((MCSPI_TRANSFER_MODE_TX_RX == trMode) &&
	     (MCSPI_DATA_LINE_COMM_MODE_7 == pinMode)) ||
	    ((MCSPI_TRANSFER_MODE_TX_ONLY == trMode) &&
	     (MCSPI_DATA_LINE_COMM_MODE_7 == pinMode))) {
		retVal = E_FAIL;
		printk(KERN_INFO "ERROR\n");
	} else {
		/* Extract the input select value. */
		inputSelect = ((pinMode & MCSPI_CHCONF_IS_MASK) >>
			       MCSPI_CHCONF_IS_SHIFT);
		/* Extract the transmission enable value for data line 1. */
		dataLine1 = ((pinMode & MCSPI_CHCONF_DPE1_MASK) >>
			     MCSPI_CHCONF_DPE1_SHIFT);
		/* Extract the transmission enable value for data line 0. */
		dataLine0 = ((pinMode & MCSPI_CHCONF_DPE0_MASK) >>
			     MCSPI_CHCONF_DPE0_SHIFT);

		/* Configure the IS, DPE0 and DPE1 field of MCSPI_CHCONF register. */
		regVal = omap2_hw_direct_rd_reg32(baseAddr +
						  MCSPI_CHCONF(chNum));
		HW_SET_FIELD(regVal, MCSPI_CHCONF_IS, inputSelect);
		HW_SET_FIELD(regVal, MCSPI_CHCONF_DPE1, dataLine1);
		HW_SET_FIELD(regVal, MCSPI_CHCONF_DPE0, dataLine0);
		omap2_hw_direct_wr_reg32((baseAddr + MCSPI_CHCONF(chNum)),
					 regVal);

		retVal = S_PASS;
	}

	/* Configure the SPIEN polarity */
	HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_EPOL,
		      MCSPI_CHCONF_EPOL_ACTIVELOW);

	/* Configure the Manual SPIEN assertion */
	HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_FORCE,
		      MCSPI_CHCONF_FORCE_ASSERT);

	return retVal;
}

/*******************************************************************************/ /*!
 * @brief  Configures the SPI word length
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
int32_t spi_word_length(uint32_t chNum, uint32_t wordLength)
{
	uint32_t retVal = E_FAIL;
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;

	if ((wordLength >= 4) && (wordLength <= 32)) {
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_WL,
			      wordLength - 1);
		retVal = S_PASS;
	}

	return retVal;
}

/*******************************************************************************/ /*!
 * @brief  Enable/disable transmit start bit.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_start_bit_enable(uint32_t chNum, uint32_t enableStartBit)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	if (TRUE == enableStartBit) {
		/* Enable the start bit. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_SBE, MCSPI_CHCONF_SBE_ENABLED);
	} else {
		/* Disable the start bit. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_SBE, MCSPI_CHCONF_SBE_DISABLED);
	}
}

/*******************************************************************************/ /*!
 * @brief  SPI channel enable/disable.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_ch_enable(uint32_t chNum, uint32_t enableCh)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	if (TRUE == enableCh) {
		/* Enable the MCSPI channel. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCTRL(chNum)), MCSPI_CHCTRL_EN,
			      MCSPI_CHCTRL_EN_ACT);
	} else {
		/* Disable the MCSPI channel. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCTRL(chNum)), MCSPI_CHCTRL_EN,
			      MCSPI_CHCTRL_EN_NACT);
	}
	/* Flash post-writes */
	omap2_hw_direct_rd_reg32((baseAddr + MCSPI_CHCTRL(chNum)));
}

/*******************************************************************************/ /*!
 * @brief  SPI wakeup enable/disable.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_wakeup_enable(uint32_t enableWu)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	if (TRUE == enableWu) {
		omap2_hw_direct_wr_reg32(baseAddr + MCSPI_WAKEUPEN, 1);
	} else {
		omap2_hw_direct_wr_reg32(baseAddr + MCSPI_WAKEUPEN, 0);
	}
}

/*******************************************************************************/ /*!
 * @brief  Get SPI pol/phase mode.
 *
 * @return the current SPI mode
 * @exception
 * @globals
 ***********************************************************************************/
int spi_get_mode(uint32_t chNum)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t value =
		omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHCONF(chNum));
	return value & (MCSPI_CHCONF_PHA_MASK | MCSPI_CHCONF_POL_MASK);
}

/*******************************************************************************/ /*!
 * @brief  Set SPI pol/phase mode.
 *
 * @param mode: The new mode.
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_set_mode(uint32_t chNum, int mode)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t mode_value =
		mode & (MCSPI_CHCONF_PHA_MASK | MCSPI_CHCONF_POL_MASK);
	uint32_t reg_value =
		omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHCONF(chNum));

	reg_value &= ~(MCSPI_CHCONF_PHA_MASK | MCSPI_CHCONF_POL_MASK);
	reg_value |= mode_value;

	omap2_hw_direct_wr_reg32(baseAddr + MCSPI_CHCONF(chNum), reg_value);
}

/*******************************************************************************/ /*!
 * @brief  Get SPI bits per word.
 *
 * @return the current bits per word.
 * @exception
 * @globals
 ***********************************************************************************/
int spi_get_bits_per_word(uint32_t chNum)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t reg_value =
		omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHCONF(chNum));

	reg_value &= MCSPI_CHCONF_WL_MASK;
	reg_value >>= MCSPI_CHCONF_WL_SHIFT;

	return reg_value + 1;
}

/*******************************************************************************/ /*!
 * @brief  Set SPI bits per word.
 *
 * @param wordLength: The new bits per word.
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_set_bits_per_word(uint32_t chNum, int wordLength)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;

	if ((wordLength >= 4) && (wordLength <= 32)) {
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_WL,
			      wordLength - 1);
	}
}

/*******************************************************************************/ /*!
 * @brief  Set SPI speed.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_set_speed(uint32_t chNum, uint32_t spiInClk, uint32_t spiOutClk)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t clkRatio = 0U;
	uint32_t extClk = 0U;
	uint32_t clkDiv = 0U;

	/* Calculate the value of clkRatio. */
	clkRatio = (spiInClk / spiOutClk);

	/* If clkRatio is not a power of 2, set granularity of 1 clock cycle */
	if (0U != (clkRatio & (clkRatio - 1U))) {
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_CLKG, MCSPI_CHCONF_CLKG_ONECYCLE);

		/* Calculate the ratios clkD and extClk based on fClk */
		extClk = (clkRatio - 1U) >> 4U;
		clkDiv = (clkRatio - 1U) & MCSPI_CLKDIV_MASK;

		HW_WR_FIELD32((baseAddr + MCSPI_CHCTRL(chNum)),
			      MCSPI_CHCTRL_EXTCLK, extClk);
	} else {
		/* Clock granularity of power of 2. */
		HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)),
			      MCSPI_CHCONF_CLKG, MCSPI_CHCONF_CLKG_POWERTWO);

		while (1U != clkRatio) {
			clkRatio /= 2U;
			clkDiv++;
		}
	}

	/* Configure the clkD field of MCSPI_CHCONF register. */
	HW_WR_FIELD32((baseAddr + MCSPI_CHCONF(chNum)), MCSPI_CHCONF_CLKD,
		      clkDiv);
}

/*******************************************************************************/ /*!
 * @brief  Set SPI chip-select.
 *
 * @return
 * @exception
 * @globals
 ***********************************************************************************/
void spi_set_cs(uint32_t chNum, bool enable)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	uint32_t reg_value =
		omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHCONF(chNum));

	if (!enable) {
		reg_value &= ~MCSPI_CHCONF_FORCE_MASK;
	} else {
		reg_value |= MCSPI_CHCONF_FORCE_MASK;
	}

	omap2_hw_direct_wr_reg32(baseAddr + MCSPI_CHCONF(chNum), reg_value);
}

/*******************************************************************************/ /*!
 * @brief  Extract the transfer data from the IOCTL.
 *
 * @return the transfer data.
 * @exception
 * @globals
 ***********************************************************************************/
struct spi_ioc_transfer *
spidev_get_ioc_message(unsigned int cmd, struct spi_ioc_transfer __user *u_ioc,
		       unsigned *n_ioc)
{
	u32 tmp;

	/* Check type, command number and direction */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC ||
	    _IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0)) ||
	    _IOC_DIR(cmd) != _IOC_WRITE)
		return ERR_PTR(-ENOTTY);

	tmp = _IOC_SIZE(cmd);
	if ((tmp % sizeof(struct spi_ioc_transfer)) != 0)
		return ERR_PTR(-EINVAL);
	*n_ioc = tmp / sizeof(struct spi_ioc_transfer);
	if (*n_ioc == 0)
		return NULL;

	/* copy into scratch area */
	return memdup_user(u_ioc, tmp);
}

static USE_INLINED_FUNCTION void spi_wait_for_tx_free(uint32_t chNum) USE_NON_OPTIMIZED_FUNCTION;
static USE_INLINED_FUNCTION void spi_wait_for_rx_full(uint32_t chNum) USE_NON_OPTIMIZED_FUNCTION;

USE_INLINED_FUNCTION void spi_wait_for_tx_free(uint32_t chNum)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	while ((omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHSTAT(chNum)) &
		MCSPI_CHSTAT_TXS_MASK) == 0)
		;
}

USE_INLINED_FUNCTION void spi_wait_for_rx_full(uint32_t chNum)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	while ((omap2_hw_direct_rd_reg32(baseAddr + MCSPI_CHSTAT(chNum)) &
		MCSPI_CHSTAT_RXS_MASK) == 0)
		;
}

/*******************************************************************************/ /*!
 * @brief  Transfers the given message to SPI.
 *
 * @return the transfer data.
 * @exception
 * @globals
 ***********************************************************************************/
int spidev_message(uint32_t chNum, struct spi_ioc_transfer *u_xfers,
		   unsigned n_xfers)
{
	void __iomem *baseAddr = omap2_hw_direct_spi_data.base;
	struct spi_ioc_transfer *u_tmp;
	int status = -EFAULT;
	unsigned n, total;
	void *tx_buf, *rx_buf;
	uint16_t c;
	uint32_t tmp_len;
	bool keep_cs = false;

	spi_set_cs(chNum, TRUE); // ??ToDo  disables output??
 	spi_ch_enable(chNum, TRUE);

	total = 0;
	tx_buf = omap2_hw_direct_spi_data.tx_buffer;
	rx_buf = omap2_hw_direct_spi_data.rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->len > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

 		spi_set_speed(chNum, MCSPI_IN_CLK, u_tmp->speed_hz);

		/* Copy user tx data  */
		if (u_tmp->tx_buf) {
			if (copy_from_user(
				    tx_buf,
				    (const void __user *)(uintptr_t)u_tmp->tx_buf,
				    u_tmp->len)) {
				goto done;
			}
		}

		spi_set_bits_per_word(chNum, u_tmp->bits_per_word);

		if(u_tmp->bits_per_word <= 8) {
			uint8_t *tx_buf_u8 = tx_buf;
			uint8_t *rx_buf_u8 = rx_buf;
			uint8_t rx_value;
			tmp_len = u_tmp->len;
			while (tmp_len > 0) {
				spi_wait_for_tx_free(chNum);
				if (u_tmp->tx_buf) {
					c = *tx_buf_u8++;
				} else {
					c = 0; // dummy data
				}
				omap2_hw_direct_wr_reg32(baseAddr + OMAP2_HWDIRECT_TX0,
							c);

				spi_wait_for_rx_full(chNum);
				rx_value = omap2_hw_direct_rd_reg32(baseAddr + OMAP2_HWDIRECT_RX0);
				*rx_buf_u8++ = rx_value;

				tmp_len--;
				total++;
			}
		}
		else if(u_tmp->bits_per_word <= 16) {
			uint16_t *tx_buf_u16 = tx_buf;
			uint16_t *rx_buf_u16 = rx_buf;
			uint16_t rx_value;
			tmp_len = u_tmp->len / 2;
			while (tmp_len > 0) {
				spi_wait_for_tx_free(chNum);
				if (u_tmp->tx_buf) {
					c = *tx_buf_u16++;
				} else {
					c = 0; // dummy data
				}
				omap2_hw_direct_wr_reg32(baseAddr + OMAP2_HWDIRECT_TX0,
							c);

				spi_wait_for_rx_full(chNum);
				rx_value = omap2_hw_direct_rd_reg32(baseAddr + OMAP2_HWDIRECT_RX0);
				*rx_buf_u16++ = rx_value;

				tmp_len--;
				total++;
			}
			total *= 2;
		}

		/* Copy received data to user space */
		if (u_tmp->rx_buf) {
			if (copy_to_user((void __user *)
					(uintptr_t) u_tmp->rx_buf, omap2_hw_direct_spi_data.rx_buffer,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}

		if (u_tmp->cs_change == 1) {
			keep_cs = true;
		}
	}

	if(!keep_cs) {
		spi_set_cs(chNum, FALSE);
	}

	status = total;

done:
 	spi_ch_enable(chNum, FALSE);
	return status;
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
	       MAJOR(omap2_hw_direct_device_info.device),
	       MINOR(omap2_hw_direct_device_info.device));

	/* creating cdev structure */
	cdev_init(&omap2_hw_direct_device_info.driver_cdev, &fops);

	/* Adding character device to the system */
	if ((cdev_add(&omap2_hw_direct_device_info.driver_cdev,
		      omap2_hw_direct_device_info.device, 1)) < 0) {
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
			   omap2_hw_direct_device_info.device, NULL,
			   "spiomap2hw_input")) == NULL) {
		printk(KERN_INFO "Cannot create the Device 1\n");
		goto r_device;
	}

	omap2_hw_direct_spi_data.base =
		ioremap(OMAP2_HWDIRECT_SPI0_BASE_REGISTER, 0x1000);
	omap2_hw_direct_prcm_data.base =
		ioremap(OMAP2_HWDIRECT_PRCM_BASE_REGISTER, 0x4000);
	omap2_hw_direct_pinmux_data.base =
		ioremap(OMAP2_HWDIRECT_PINMUX_BASE_REGISTER, 0x2000);

	omap2_hw_direct_spi_data.tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
	omap2_hw_direct_spi_data.rx_buffer = kmalloc(bufsiz, GFP_KERNEL);

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
	device_destroy(omap2_hw_direct_device_info.dev_class,
		       omap2_hw_direct_device_info.device);
	class_destroy(omap2_hw_direct_device_info.dev_class);
	cdev_del(&omap2_hw_direct_device_info.driver_cdev);
	unregister_chrdev_region(omap2_hw_direct_device_info.device, 1);

	kfree(omap2_hw_direct_spi_data.tx_buffer);
	kfree(omap2_hw_direct_spi_data.rx_buffer);
}

//==================================================================================================
module_init(omap2_hw_direct_init);
module_exit(omap2_hw_direct_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Kaul < martin@familie-kaul.de >");
MODULE_DESCRIPTION(
	"A spi driver that uses direct the OMAP2-MCSPI hardware without use of the queued spi driver class.");
MODULE_VERSION("0.1");
