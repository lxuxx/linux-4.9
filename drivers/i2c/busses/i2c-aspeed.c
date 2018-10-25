/*
 * i2c-ast.c - I2C driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
/* I2C Register */

#define  I2C_DMA_BASE_REG				0x24
#define  I2C_DMA_LEN_REG				0x28

/* 0x28 : I2C_DMA_LEN_REG	*/
#define AST_G6_I2CD_GET_DMA_LEN(x)			((x >> 16) & 0x1fff)

/***************************************************************************/
//AST2600 reg
#define AST_I2CC_FUN_CTRL			0x00 	/* 0x00 : I2CC Master/Slave Function Control Register  */
#define AST_I2CC_SLAVE_ADDR_RX_EN 		BIT(19)
#define AST_I2CC_MASTER_RETRY 			BIT(18)
#define AST_I2CC_BUS_AUTO_RELEASE		BIT(17)	
#define AST_I2CC_M_SDA_LOCK_EN			BIT(16)
#define AST_I2CC_MULTI_MASTER_DIS		BIT(15)
#define AST_I2CC_M_SCL_DRIVE_EN			BIT(14)
#define AST_I2CC_MSB_STS				BIT(9)
#define AST_I2CC_SDA_DRIVE_1T_EN		BIT(8)
#define AST_I2CC_M_SDA_DRIVE_1T_EN		BIT(7)
#define AST_I2CC_M_HIGH_SPEED_EN		BIT(6)
/* reserver 5 : 2 */
#define AST_I2CC_SLAVE_EN				BIT(1)
#define AST_I2CC_MASTER_EN				BIT(0)

#define AST_I2CC_AC_TIMING			0x04	/* 0x04 : I2CC Master/Slave Clock and AC Timing Control Register #1 */
#define AST_I2CC_tTIMEOUT(x)			((x & 0x1f) << 24)	// 0~7
#define AST_I2CC_tCKHIGHMin(x)			((x & 0xf) << 20)	// 0~f 
#define AST_I2CC_tCKHIGH(x)				((x & 0xf) << 16)	// 0~7 
#define AST_I2CC_tCKLOW(x)				((x & 0xf) << 12)	// 0~7 
#define AST_I2CC_tHDDAT(x)				((x & 0x3) << 10)	// 0~3 
#define AST_I2CC_toutBaseCLK(x)			((x & 0x3) << 8)	//0~3
#define AST_I2CC_tBaseCLK(x)			(x & 0xf)	// 0~0xf

#define AST_I2CC_STS_AND_BUFF		0x08	/* 0x08 : I2CC Master/Skave Transmit/Receive Byte Buffer Register */

#define AST_I2CC_TX_DIR_MASK			(0x7 << 29)
#define AST_I2CC_SDA_OE					BIT(28)
#define AST_I2CC_SDA_O					BIT(27)
#define AST_I2CC_SCL_OE					BIT(26)
#define AST_I2CC_SCL_O					BIT(25)

// Tx State Machine
#define AST_I2CM_IDLE	 				0x0
#define AST_I2CM_MACTIVE				0x8
#define AST_I2CM_MSTART					0x9
#define AST_I2CM_MSTARTR				0xa
#define AST_I2CM_MSTOP					0xb
#define AST_I2CM_MTXD					0xc  
#define AST_I2CM_MRXACK					0xd
#define AST_I2CM_MRXD 					0xe
#define AST_I2CM_MTXACK 				0xf
#define AST_I2CM_SWAIT					0x1
#define AST_I2CM_SRXD 					0x4
#define AST_I2CM_STXACK 				0x5
#define AST_I2CM_STXD					0x6
#define AST_I2CM_SRXACK 				0x7
#define AST_I2CM_RECOVER 				0x3


#define AST_I2CC_SCL_LINE_STS			BIT(18)
#define AST_I2CC_SDA_LINE_STS			BIT(17)
#define AST_I2CC_BUS_BUSY_STS			BIT(16)

#define AST_I2CC_GET_RX_BUFF(x)		((x & 0xff00) >> 8)

#define AST_I2CC_BUFF_CTRL		0x0C	/* 0x0C : I2CC Master/Slave Pool Buffer Control Register  */
#define AST_I2CC_RX_BUF_ADDR_GET(x)			((x >> 24) & 0x3f)
#define AST_I2CC_RX_BUF_END_ADDR_SET(x)		(x << 16)
#define AST_I2CC_TX_DATA_BUF_END_SET(x)		((x & 0xff) << 8)
#define AST_I2CC_TX_DATA_BUF_GET(x)			((x >>8) & 0x1f)
#define AST_I2CC_BUF_BASE_ADDR_SET(x)		(x & 0x3f)

#define AST_I2CM_IER			0x10	/* 0x10 : I2CM Master Interrupt Control Register */
#define AST_I2CM_ISR			0x14	/* 0x14 : I2CM Master Interrupt Status Register   : WC */

#define AST_I2CM_PKT_TIMEOUT			BIT(18)
#define AST_I2CM_PKT_ERROR				BIT(17)
#define AST_I2CM_PKT_DONE				BIT(16)

#define AST_I2CM_BUS_RECOVER_FAIL		BIT(15)
#define AST_I2CM_SDA_DL_TO				BIT(14)
#define AST_I2CM_BUS_RECOVER			BIT(13)
#define AST_I2CM_SMBUS_ALT				BIT(12)

#define AST_I2CM_SCL_LOW_TO				BIT(6)
#define AST_I2CM_ABNORMAL_COND			BIT(5)
#define AST_I2CM_NORMAL_STOP			BIT(4)
#define AST_I2CM_ARBIT_LOSS				BIT(3)
#define AST_I2CM_RX_DONE				BIT(2)
#define AST_I2CM_TX_NAK					BIT(1)
#define AST_I2CM_TX_ACK					BIT(0)

#define AST_I2CM_CMD_STS		0x18	/* 0x18 : I2CM Master Command/Status Register   */
#define AST_I2CM_PKT_ADDR(x)			((x & 0x7f) << 24)	
#define AST_I2CM_PKT_EN					BIT(16)
#define AST_I2CM_SDA_OE_OUT_DIR			BIT(15)
#define AST_I2CM_SDA_O_OUT_DIR			BIT(14)
#define AST_I2CM_SCL_OE_OUT_DIR			BIT(13)
#define AST_I2CM_SCL_O_OUT_DIR			BIT(12)
#define AST_I2CM_RECOVER_CMD_EN			BIT(11)

#define AST_I2CM_RX_DMA_EN				BIT(9)
#define AST_I2CM_TX_DMA_EN				BIT(8)

/* Command Bit */
#define AST_I2CM_RX_BUFF_EN				BIT(7)
#define AST_I2CM_TX_BUFF_EN				BIT(6)
#define AST_I2CM_STOP_CMD				BIT(5)
#define AST_I2CM_RX_CMD_LAST			BIT(4)
#define AST_I2CM_RX_CMD					BIT(3)

#define AST_I2CM_TX_CMD					BIT(1)
#define AST_I2CM_START_CMD				BIT(0)

#define AST_I2CM_DMA_LEN		0x1C	/* 0x1C : I2CM Master DMA Transfer Length Register   */
#define AST_I2CM_RX_DMA_LEN(x)			((x) << 12)	/* 1 ~ 4096 */
#define AST_I2CM_GET_RX_DMA_LEN(x)		(((x) & 0x1FFF) >> 16)	
#define AST_I2CM_TX_DMA_LEN(x)			(x)			/* 1 ~ 4096 */

#define AST_I2CS_IER			0x20	/* 0x20 : I2CS Slave Interrupt Control Register   */
#define AST_I2CS_ISR			0x24	/* 0x24 : I2CS Slave Interrupt Status Register   */

#define AST_I2CS_ADDR3_NAK			BIT(22)
#define AST_I2CS_ADDR2_NAK			BIT(21)
#define AST_I2CS_ADDR1_NAK			BIT(20)

#define AST_I2CS_PKT_CMD_FAIL		BIT(17)
#define AST_I2CS_PKT_CMD_DONE		BIT(16)
#define AST_I2CS_INACTIVE_TO		BIT(15)
//
#define AST_I2CS_SLAVE_MATCH		BIT(7)
//
#define AST_I2CS_ABNOR_STOP			BIT(5)
#define AST_I2CS_STOP				BIT(4)
#define AST_I2CS_MATCH_NAK			BIT(3)
#define AST_I2CS_RX_DONE			BIT(2)
#define AST_I2CS_TX_NAK				BIT(1)
#define AST_I2CS_TX_ACK				BIT(0)

#define AST_I2CS_CMD_STS		0x28	/* 0x28 : I2CS Slave CMD/Status Register   */
#define AST_I2CS_ACTIVE_ALL				(0x3 << 17)
#define AST_I2CS_PKT_MODE_EN			BIT(16)
#define AST_I2CS_AUTO_NAK_NA_EN			BIT(15)
#define AST_I2CS_AUTO_NAK_EN			BIT(14)

#define AST_I2CS_ALT_EN					BIT(10)
#define AST_I2CS_RX_DMA_EN				BIT(9)
#define AST_I2CS_TX_DMA_EN				BIT(8)
#define AST_I2CS_RX_BUFF_EN				BIT(7)
#define AST_I2CS_TX_BUFF_EN				BIT(6)
#define AST_I2CS_RX_CMD_LAST			BIT(4)

#define AST_I2CS_TX_CMD					BIT(2)

#define AST_I2CS_DMA_LEN		0x2C

#define AST_I2CM_TX_DMA			0x30 	/* I2CM Master DMA Tx Buffer Register   */
#define AST_I2CM_RX_DMA			0x34	/* I2CM Master DMA Rx Buffer Register   */
#define AST_I2CS_TX_DMA			0x38 	/* I2CS Slave DMA Tx Buffer Register   */
#define AST_I2CS_RX_DMA			0x3C	/* I2CS Slave DMA Rx Buffer Register   */


#define AST_I2CS_ADDR_CTRL		0x40

#define AST_I2CM_DMA_LEN_STS	0x48
#define AST_I2CS_DMA_LEN_STS	0x4C

/* 0x2C : I2CS SLAVE DMA Transfer Length Register   */
#define AST_I2CS_RX_DMA_LEN(x)			((x) << 12)	/* 1 ~ 4096 */
#define AST_I2CS_TX_DMA_LEN(x)			(x)			/* 1 ~ 4096 */


/* 0x40 : Slave Device Address Register */
#define AST_I2CS_ADDR3_ENABLE			BIT(23)
#define AST_I2CS_ADDR3(x)				(x << 16)

#define AST_I2CS_ADDR2_ENABLE			BIT(15)
#define AST_I2CS_ADDR2(x)				(x << 8)
#define AST_I2CS_ADDR_RANGE_MODE		BIT(7)
#define AST_I2CS_ADDR1(x)				(x)

/***************************************************************************/
//#define AST_I2C_S_DEBUG 1
#ifdef AST_I2C_S_DEBUG
#define I2C_S_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2C_S_DBUG(fmt, args...)
#endif

//#define AST_I2C_M_DEBUG 1

#ifdef AST_I2C_M_DEBUG
#define I2C_M_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2C_M_DBUG(fmt, args...)
#endif

/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */
/***************************************************************************/
#define I2C_S_BUF_SIZE 		256
#define I2C_S_RX_BUF_NUM 	20

#define AST_LOCKUP_DETECTED 	BIT(15)
#define AST_I2C_LOW_TIMEOUT 	0x07
/***************************************************************************/
#define BYTE_MODE		0
#define BUFF_MODE		1
#define DMA_MODE	2


/***************************************************************************/
typedef enum i2c_slave_event_e {
	I2C_SLAVE_EVENT_START_READ,
	I2C_SLAVE_EVENT_READ,
	I2C_SLAVE_EVENT_START_WRITE,
	I2C_SLAVE_EVENT_WRITE,
	I2C_SLAVE_EVENT_NACK,
	I2C_SLAVE_EVENT_STOP
} i2c_slave_event_t;

struct ast_i2c_timing_table {
	u32 divisor;
	u32 timing;
};

static struct ast_i2c_timing_table ast_g6_i2c_timing_table[] = {
	/* Divisor : Base Clock : tCK High : tCK Low  */
	/* Divisor :	  [3:0]    :   [19:16]:   [15:12] */
	{6,	0x00000300 | (0x0) | (0x2 << 16) | (0x2 << 12) },
	{7,	0x00000300 | (0x0) | (0x3 << 16) | (0x2 << 12) },
	{8,	0x00000300 | (0x0) | (0x3 << 16) | (0x3 << 12) },
	{9,	0x00000300 | (0x0) | (0x4 << 16) | (0x3 << 12) },
	{10, 	0x00000300 | (0x0) | (0x4 << 16) | (0x4 << 12) },
	{11, 	0x00000300 | (0x0) | (0x5 << 16) | (0x4 << 12) },
	{12, 	0x00000300 | (0x0) | (0x5 << 16) | (0x5 << 12) },
	{13, 	0x00000300 | (0x0) | (0x6 << 16) | (0x5 << 12) },
	{14, 	0x00000300 | (0x0) | (0x6 << 16) | (0x6 << 12) },
	{15, 	0x00000300 | (0x0) | (0x7 << 16) | (0x6 << 12) },
	{16, 	0x00000300 | (0x0) | (0x7 << 16) | (0x7 << 12) },
	{17, 	0x00000300 | (0x0) | (0x8 << 16) | (0x7 << 12) },
	{18, 	0x00000300 | (0x0) | (0x8 << 16) | (0x8 << 12) },
	{19, 	0x00000300 | (0x0) | (0x9 << 16) | (0x8 << 12) },
	{20, 	0x00000300 | (0x0) | (0x9 << 16) | (0x9 << 12) },
	{21, 	0x00000300 | (0x0) | (0xa << 16) | (0x9 << 12) },
	{22, 	0x00000300 | (0x0) | (0xa << 16) | (0xa << 12) },
	{23, 	0x00000300 | (0x0) | (0xb << 16) | (0xa << 12) },
	{24, 	0x00000300 | (0x0) | (0xb << 16) | (0xb << 12) },
	{25, 	0x00000300 | (0x0) | (0xc << 16) | (0xb << 12) },
	{26, 	0x00000300 | (0x0) | (0xc << 16) | (0xc << 12) },
	{27, 	0x00000300 | (0x0) | (0xd << 16) | (0xc << 12) },
	{28, 	0x00000300 | (0x0) | (0xd << 16) | (0xd << 12) },
	{29, 	0x00000300 | (0x0) | (0xe << 16) | (0xd << 12) },
	{30, 	0x00000300 | (0x0) | (0xe << 16) | (0xe << 12) },
	{31, 	0x00000300 | (0x0) | (0xf << 16) | (0xe << 12) },
	{32, 	0x00000300 | (0x0) | (0xf << 16) | (0xf << 12) },

	{34, 	0x00000300 | (0x1) | (0x8 << 16) | (0x7 << 12) },
	{36, 	0x00000300 | (0x1) | (0x8 << 16) | (0x8 << 12) },
	{38, 	0x00000300 | (0x1) | (0x9 << 16) | (0x8 << 12) },
	{40, 	0x00000300 | (0x1) | (0x9 << 16) | (0x9 << 12) },
	{42, 	0x00000300 | (0x1) | (0xa << 16) | (0x9 << 12) },
	{44, 	0x00000300 | (0x1) | (0xa << 16) | (0xa << 12) },
	{46, 	0x00000300 | (0x1) | (0xb << 16) | (0xa << 12) },
	{48, 	0x00000300 | (0x1) | (0xb << 16) | (0xb << 12) },
	{50, 	0x00000300 | (0x1) | (0xc << 16) | (0xb << 12) },
	{52, 	0x00000300 | (0x1) | (0xc << 16) | (0xc << 12) },
	{54, 	0x00000300 | (0x1) | (0xd << 16) | (0xc << 12) },
	{56, 	0x00000300 | (0x1) | (0xd << 16) | (0xd << 12) },
	{58, 	0x00000300 | (0x1) | (0xe << 16) | (0xd << 12) },
	{60, 	0x00000300 | (0x1) | (0xe << 16) | (0xe << 12) },
	{62, 	0x00000300 | (0x1) | (0xf << 16) | (0xe << 12) },
	{64, 	0x00000300 | (0x1) | (0xf << 16) | (0xf << 12) },

	{68, 	0x00000300 | (0x2) | (0x8 << 16) | (0x7 << 12) },
	{72, 	0x00000300 | (0x2) | (0x8 << 16) | (0x8 << 12) },
	{76, 	0x00000300 | (0x2) | (0x9 << 16) | (0x8 << 12) },
	{80, 	0x00000300 | (0x2) | (0x9 << 16) | (0x9 << 12) },
	{84, 	0x00000300 | (0x2) | (0xa << 16) | (0x9 << 12) },
	{88, 	0x00000300 | (0x2) | (0xa << 16) | (0xa << 12) },
	{92, 	0x00000300 | (0x2) | (0xb << 16) | (0xa << 12) },
	{96, 	0x00000300 | (0x2) | (0xb << 16) | (0xb << 12) },
	{100, 	0x00000300 | (0x2) | (0xc << 16) | (0xb << 12) },
	{104, 	0x00000300 | (0x2) | (0xc << 16) | (0xc << 12) },
	{108, 	0x00000300 | (0x2) | (0xd << 16) | (0xc << 12) },
	{112, 	0x00000300 | (0x2) | (0xd << 16) | (0xd << 12) },
	{116, 	0x00000300 | (0x2) | (0xe << 16) | (0xd << 12) },
	{120, 	0x00000300 | (0x2) | (0xe << 16) | (0xe << 12) },
	{124, 	0x00000300 | (0x2) | (0xf << 16) | (0xe << 12) },
	{128, 	0x00000300 | (0x2) | (0xf << 16) | (0xf << 12) },

	{136, 	0x00000300 | (0x3) | (0x8 << 16) | (0x7 << 12) },
	{144, 	0x00000300 | (0x3) | (0x8 << 16) | (0x8 << 12) },
	{152, 	0x00000300 | (0x3) | (0x9 << 16) | (0x8 << 12) },
	{160, 	0x00000300 | (0x3) | (0x9 << 16) | (0x9 << 12) },
	{168, 	0x00000300 | (0x3) | (0xa << 16) | (0x9 << 12) },
	{176, 	0x00000300 | (0x3) | (0xa << 16) | (0xa << 12) },
	{184, 	0x00000300 | (0x3) | (0xb << 16) | (0xa << 12) },
	{192, 	0x00000300 | (0x3) | (0xb << 16) | (0xb << 12) },
	{200, 	0x00000300 | (0x3) | (0xc << 16) | (0xb << 12) },
	{208, 	0x00000300 | (0x3) | (0xc << 16) | (0xc << 12) },
	{216, 	0x00000300 | (0x3) | (0xd << 16) | (0xc << 12) },
	{224, 	0x00000300 | (0x3) | (0xd << 16) | (0xd << 12) },
	{232, 	0x00000300 | (0x3) | (0xe << 16) | (0xd << 12) },
	{240, 	0x00000300 | (0x3) | (0xe << 16) | (0xe << 12) },
	{248, 	0x00000300 | (0x3) | (0xf << 16) | (0xe << 12) },
	{256, 	0x00000300 | (0x3) | (0xf << 16) | (0xf << 12) },

	{272, 	0x00000300 | (0x4) | (0x8 << 16) | (0x7 << 12) },
	{288, 	0x00000300 | (0x4) | (0x8 << 16) | (0x8 << 12) },
	{304, 	0x00000300 | (0x4) | (0x9 << 16) | (0x8 << 12) },
	{320, 	0x00000300 | (0x4) | (0x9 << 16) | (0x9 << 12) },
	{336, 	0x00000300 | (0x4) | (0xa << 16) | (0x9 << 12) },
	{352, 	0x00000300 | (0x4) | (0xa << 16) | (0xa << 12) },
	{368, 	0x00000300 | (0x4) | (0xb << 16) | (0xa << 12) },
	{384, 	0x00000300 | (0x4) | (0xb << 16) | (0xb << 12) },
	{400, 	0x00000300 | (0x4) | (0xc << 16) | (0xb << 12) },
	{416, 	0x00000300 | (0x4) | (0xc << 16) | (0xc << 12) },
	{432, 	0x00000300 | (0x4) | (0xd << 16) | (0xc << 12) },
	{448, 	0x00000300 | (0x4) | (0xd << 16) | (0xd << 12) },
	{464, 	0x00000300 | (0x4) | (0xe << 16) | (0xd << 12) },
	{480, 	0x00000300 | (0x4) | (0xe << 16) | (0xe << 12) },
	{496, 	0x00000300 | (0x4) | (0xf << 16) | (0xe << 12) },
	{512, 	0x00000300 | (0x4) | (0xf << 16) | (0xf << 12) },

	{544, 	0x00000300 | (0x5) | (0x8 << 16) | (0x7 << 12) },
	{576, 	0x00000300 | (0x5) | (0x8 << 16) | (0x8 << 12) },
	{608, 	0x00000300 | (0x5) | (0x9 << 16) | (0x8 << 12) },
	{640, 	0x00000300 | (0x5) | (0x9 << 16) | (0x9 << 12) },
	{672, 	0x00000300 | (0x5) | (0xa << 16) | (0x9 << 12) },
	{704, 	0x00000300 | (0x5) | (0xa << 16) | (0xa << 12) },
	{736, 	0x00000300 | (0x5) | (0xb << 16) | (0xa << 12) },
	{768, 	0x00000300 | (0x5) | (0xb << 16) | (0xb << 12) },
	{800, 	0x00000300 | (0x5) | (0xc << 16) | (0xb << 12) },
	{832, 	0x00000300 | (0x5) | (0xc << 16) | (0xc << 12) },
	{864, 	0x00000300 | (0x5) | (0xd << 16) | (0xc << 12) },
	{896, 	0x00000300 | (0x5) | (0xd << 16) | (0xd << 12) },
	{928, 	0x00000300 | (0x5) | (0xe << 16) | (0xd << 12) },
	{960, 	0x00000300 | (0x5) | (0xe << 16) | (0xe << 12) },
	{992, 	0x00000300 | (0x5) | (0xf << 16) | (0xe << 12) },
	{1024, 	0x00000300 | (0x5) | (0xf << 16) | (0xf << 12) },

	{1088, 	0x00000300 | (0x6) | (0x8 << 16) | (0x7 << 12) },
	{1152, 	0x00000300 | (0x6) | (0x8 << 16) | (0x8 << 12) },
	{1216, 	0x00000300 | (0x6) | (0x9 << 16) | (0x8 << 12) },
	{1280, 	0x00000300 | (0x6) | (0x9 << 16) | (0x9 << 12) },
	{1344, 	0x00000300 | (0x6) | (0xa << 16) | (0x9 << 12) },
	{1408, 	0x00000300 | (0x6) | (0xa << 16) | (0xa << 12) },
	{1472, 	0x00000300 | (0x6) | (0xb << 16) | (0xa << 12) },
	{1536, 	0x00000300 | (0x6) | (0xb << 16) | (0xb << 12) },
	{1600, 	0x00000300 | (0x6) | (0xc << 16) | (0xb << 12) },
	{1664, 	0x00000300 | (0x6) | (0xc << 16) | (0xc << 12) },
	{1728, 	0x00000300 | (0x6) | (0xd << 16) | (0xc << 12) },
	{1792, 	0x00000300 | (0x6) | (0xd << 16) | (0xd << 12) },
	{1856, 	0x00000300 | (0x6) | (0xe << 16) | (0xd << 12) },
	{1920, 	0x00000300 | (0x6) | (0xe << 16) | (0xe << 12) },
	{1984, 	0x00000300 | (0x6) | (0xf << 16) | (0xe << 12) },
	{2048, 	0x00000300 | (0x6) | (0xf << 16) | (0xf << 12) },

	{2176, 	0x00000300 | (0x7) | (0x8 << 16) | (0x7 << 12) },
	{2304, 	0x00000300 | (0x7) | (0x8 << 16) | (0x8 << 12) },
	{2432, 	0x00000300 | (0x7) | (0x9 << 16) | (0x8 << 12) },
	{2560, 	0x00000300 | (0x7) | (0x9 << 16) | (0x9 << 12) },
	{2688, 	0x00000300 | (0x7) | (0xa << 16) | (0x9 << 12) },
	{2816, 	0x00000300 | (0x7) | (0xa << 16) | (0xa << 12) },
	{2944, 	0x00000300 | (0x7) | (0xb << 16) | (0xa << 12) },
	{3072, 	0x00000300 | (0x7) | (0xb << 16) | (0xb << 12) },
};

struct aspeed_i2c_bus_config   {
	u8	aspeed_version;
	u32	timing_table_size;
	struct ast_i2c_timing_table *timing_table;
	u16	dma_size;
};

struct buf_page {
	u8 page_no;
	u16 page_size;
	u32 page_addr;
};

//1. array, 2, flag 3 fifo buffer, 4 tx length
struct i2c_slave_tx_fifo {
	int flag;
	u8 buff[32];
	int len;
};

struct aspeed_i2c_bus {
//	struct ast_i2c_driver_data *ast_i2c_data;
	struct device			*dev;
	void __iomem			*reg_base;		/* virtual */
	int 				irq;			//I2C IRQ number
	struct clk 			*clk;
	u32				apb_clk;
	u32				bus_frequency;
	struct aspeed_i2c_bus_config	*bus_config;
//buff mode info
	struct buf_page 		buff_pool;
	u32				bus_clk;
	u16 				dma_size;
	u8				master_dma; 		//0,byte mode 1,Buffer pool mode 256 , or 2048 , 2: DMA mode
	u8				slave_dma;		//0,byte mode 1,Buffer pool mode 256 , or 2048 , 2: DMA mode
	u32				state;			//I2C xfer mode state matchine
	u32				bus_recover;
	struct i2c_adapter		adap;
//master dma or buff mode needed
	unsigned char			*dma_buf;
	dma_addr_t			dma_addr;
//master
	u8				master_operation;
	int				xfer_last;		//cur xfer is last msgs for stop msgs
	struct i2c_msg 			*master_msgs;		//cur xfer msgs
	int				master_xfer_len;	//cur xfer len
	int				master_xfer_cnt;	//total xfer count
	u32				master_xfer_mode;	//cur xfer mode ... 0 : no_op , master: 1 byte , 2 : buffer , 3: dma , slave : xxxx
	struct completion		cmd_complete;
	int				cmd_err;
	u8 				blk_r_flag; 		//for smbus block read
	void	(*do_master_xfer)(struct aspeed_i2c_bus *i2c_bus);
	void	(*do_master_xfer_done)(struct aspeed_i2c_bus *i2c_bus);
//Slave structure
	u8				slave_operation;
	u8				slave_event;
	struct i2c_msg			*slave_msgs; 		//cur slave xfer msgs
	int 				slave_xfer_len;
	int 				slave_xfer_cnt;
	u32				slave_xfer_mode;	//cur xfer mode ... 0 : no_op , master: 1 byte , 2 : buffer , 3: dma , slave : xxxx
	void	(*do_slave_xfer)(struct aspeed_i2c_bus *i2c_bus);
	u8				slave_en;
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	u8				slave_rx_full;
	u8				slave_rx_idx;
	u8				slave_ioctl_idx;
	struct i2c_msg			slave_rx_msg[I2C_S_RX_BUF_NUM];
	struct i2c_msg			slave_tx_msg;
#if 1
		//fifo size 00~ff
		struct i2c_slave_tx_fifo	slave_tx[256];
#endif
	
#endif
};

static inline void
aspeed_i2c_write(struct aspeed_i2c_bus *i2c_bus, u32 val, u32 reg)
{
//	dev_dbg(i2c_bus->dev, "aspeed_i2c_write : val: %x , reg : %x \n",val,reg);
	writel(val, i2c_bus->reg_base + reg);
}

static inline u32
aspeed_i2c_read(struct aspeed_i2c_bus *i2c_bus, u32 reg)
{
#if 0
	u32 val = readl(i2c_bus->reg_base + reg);
	printk("R : reg %x , val: %x \n", reg, val);
	return val;
#else
	return readl(i2c_bus->reg_base + reg);
#endif
}

static u32 aspeed_select_i2c_clock(struct aspeed_i2c_bus *i2c_bus)
{
	int i;
	u32 data;

	for (i = 0; i < i2c_bus->bus_config->timing_table_size; i++) {
		if ((i2c_bus->apb_clk / i2c_bus->bus_config->timing_table[i].divisor) <
		    i2c_bus->bus_frequency) {
			break;
		}
	}
	data = i2c_bus->bus_config->timing_table[i].timing;
	//printk("divisor [%d], timing [%x] \n", i2c_bus->bus_config->timing_table[i].divisor, i2c_bus->bus_config->timing_table[i].timing);

	return data;
}


#ifdef CONFIG_AST_I2C_SLAVE_MODE
/* AST I2C Slave mode  */
static void ast_slave_issue_alert(struct aspeed_i2c_bus *i2c_bus, u8 enable)
{
	//only support dev0~3
	if (i2c_bus->adap.nr > 3)
		return;
	else {
		if (enable)
			aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CS_CMD_STS) | AST_I2CS_ALT_EN,
				      AST_I2CS_CMD_STS);
		else
			aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CS_CMD_STS) & ~AST_I2CS_ALT_EN,
				      AST_I2CS_CMD_STS);
	}
}

static void ast_slave_mode_enable(struct aspeed_i2c_bus *i2c_bus,
				  struct i2c_msg *msgs)
{
	if (msgs->buf[0] == 1) {
		i2c_bus->slave_en = 1;
		aspeed_i2c_write(i2c_bus, msgs->addr, AST_I2CS_ADDR_CTRL);
		aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
						    AST_I2CC_FUN_CTRL) | AST_I2CC_SLAVE_EN, AST_I2CC_FUN_CTRL);
	} else {
		i2c_bus->slave_en = 0;
		aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
						    AST_I2CC_FUN_CTRL) & ~AST_I2CC_SLAVE_EN, AST_I2CC_FUN_CTRL);
	}
}

#endif

static u8
aspeed_i2c_bus_error_recover(struct aspeed_i2c_bus *i2c_bus)
{
	u32 sts;
	int r;
	u32 i = 0;
	dev_dbg(i2c_bus->dev, "aspeed_i2c_bus_error_recover \n");

	aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
					    AST_I2CC_FUN_CTRL) & ~(AST_I2CC_MASTER_EN | AST_I2CC_SLAVE_EN),
		      AST_I2CC_FUN_CTRL);
	if (i2c_bus->slave_en)
		aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
						    AST_I2CC_FUN_CTRL) | (AST_I2CC_MASTER_EN | AST_I2CC_SLAVE_EN), AST_I2CC_FUN_CTRL);
	else
		aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
						    AST_I2CC_FUN_CTRL) | AST_I2CC_MASTER_EN, AST_I2CC_FUN_CTRL);
	//Check 0x14's SDA and SCL status
	sts = aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF);

	if ((sts & AST_I2CC_SDA_LINE_STS) && (sts & AST_I2CC_SCL_LINE_STS)) {
		//Means bus is idle.
		dev_dbg(i2c_bus->dev, "I2C bus (%d) is idle. I2C slave doesn't exist?! [%x]\n",
			i2c_bus->adap.nr, sts);
		return -1;
	}

	dev_dbg(i2c_bus->dev, "ERROR!! I2C(%d) bus hanged, try to recovery it!\n",
		i2c_bus->adap.nr);


	if ((sts & AST_I2CC_SDA_LINE_STS) && !(sts & AST_I2CC_SCL_LINE_STS)) {
		//if SDA == 1 and SCL == 0, it means the master is locking the bus.
		//Send a stop command to unlock the bus.
		dev_dbg(i2c_bus->dev, "I2C's master is locking the bus, try to stop it.\n");
//
		init_completion(&i2c_bus->cmd_complete);
		i2c_bus->cmd_err = 0;

		aspeed_i2c_write(i2c_bus, AST_I2CM_STOP_CMD, AST_I2CM_CMD_STS);

		r = wait_for_completion_timeout(&i2c_bus->cmd_complete,
						i2c_bus->adap.timeout * HZ);

		if (i2c_bus->cmd_err) {
			dev_dbg(i2c_bus->dev, "recovery error \n");
			return -1;
		}

		if (r == 0) {
			dev_dbg(i2c_bus->dev, "recovery timed out\n");
			return -1;
		} else {
			dev_dbg(i2c_bus->dev, "Recovery successfully\n");
			return 0;
		}


	} else if (!(sts & AST_I2CC_SDA_LINE_STS)) {
		//else if SDA == 0, the device is dead. We need to reset the bus
		//And do the recovery command.
		dev_dbg(i2c_bus->dev, "I2C's slave is dead, try to recover it\n");
		//Let's retry 10 times
		for (i = 0; i < 10; i++) {
//			aspeed_i2c_bus_init(i2c_bus);
			//Do the recovery command BIT11
			init_completion(&i2c_bus->cmd_complete);
			aspeed_i2c_write(i2c_bus, AST_I2CM_RECOVER_CMD_EN, AST_I2CM_CMD_STS);
			i2c_bus->bus_recover = 1;
			r = wait_for_completion_timeout(&i2c_bus->cmd_complete,
							i2c_bus->adap.timeout * HZ);
			if (i2c_bus->cmd_err != 0) {
				dev_dbg(i2c_bus->dev, "ERROR!! Failed to do recovery command(0x%08x)\n",
					i2c_bus->cmd_err);
				return -1;
			}
			//Check 0x14's SDA and SCL status
			sts = aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF);
			if (sts & AST_I2CC_SDA_LINE_STS) //Recover OK
				break;
		}
		if (i == 10) {
			dev_dbg(i2c_bus->dev, "ERROR!! recover failed\n");
			return -1;
		}
	} else {
		dev_dbg(i2c_bus->dev, "Don't know how to handle this case?!\n");
		return -1;
	}
	dev_dbg(i2c_bus->dev, "Recovery successfully\n");
	return 0;
}

static int aspeed_i2c_wait_bus_not_busy(struct aspeed_i2c_bus *i2c_bus)
{
	int timeout = 10;

	while (aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF) & AST_I2CC_BUS_BUSY_STS) {
		if (timeout <= 0) {
			dev_dbg(i2c_bus->dev, "%d-bus busy %x \n", i2c_bus->adap.nr,
				aspeed_i2c_read(i2c_bus, AST_I2CM_CMD_STS));
			aspeed_i2c_bus_error_recover(i2c_bus);
			return -EAGAIN;
		}
		timeout--;
		mdelay(10);
	}

	return 0;
}

static void aspeed_i2c_master_do_dma_xfer(struct aspeed_i2c_bus *i2c_bus)
{
	u32 cmd = 0;
	int i;

	i2c_bus->master_xfer_mode = DMA_MODE;
	i2c_bus->slave_xfer_mode = DMA_MODE;
	dev_dbg(i2c_bus->dev, "aspeed_i2c_master_do_dma_xfer cnt %d, xf len %d \n", 
				i2c_bus->master_xfer_cnt, i2c_bus->master_msgs->len);
	if (i2c_bus->master_xfer_cnt == -1) {
		//send start
		dev_dbg(i2c_bus->dev, " %sing %d byte%s %s 0x%02x\n",
			i2c_bus->master_msgs->flags & I2C_M_RD ? "read" : "write",
			i2c_bus->master_msgs->len, i2c_bus->master_msgs->len > 1 ? "s" : "",
			i2c_bus->master_msgs->flags & I2C_M_RD ? "from" : "to",
			i2c_bus->master_msgs->addr);

		if (i2c_bus->master_msgs->flags & I2C_M_RD) {
			//workaround .. HW can;t send start read addr with buff mode
			cmd = AST_I2CM_START_CMD | AST_I2CM_TX_CMD;
			aspeed_i2c_write(i2c_bus, (i2c_bus->master_msgs->addr << 1) | 0x1,
				      AST_I2CC_STS_AND_BUFF);
//				tx_buf[0] = (i2c_bus->master_msgs->addr <<1); //+1
			i2c_bus->master_xfer_len = 1;
			aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
				      AST_I2CM_TX_ACK | AST_I2CM_TX_NAK, AST_I2CM_IER);
		} else {
			//tx
			cmd = AST_I2CM_START_CMD | AST_I2CM_TX_CMD | AST_I2CM_TX_DMA_EN;

			i2c_bus->dma_buf[0] = (i2c_bus->master_msgs->addr << 1);	//+1
			//next data write
			if ((i2c_bus->master_msgs->len + 1) > i2c_bus->bus_config->dma_size)
				i2c_bus->master_xfer_len = i2c_bus->bus_config->dma_size;
			else
				i2c_bus->master_xfer_len = i2c_bus->master_msgs->len + 1;

			for (i = 1; i < i2c_bus->master_xfer_len; i++)
				i2c_bus->dma_buf[i] = i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i];

			if (i2c_bus->xfer_last == 1) {
				dev_dbg(i2c_bus->dev, "last stop \n");
				cmd |= AST_I2CM_STOP_CMD;
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
					      ~AST_I2CM_TX_ACK &
					      ~AST_I2CM_TX_NAK, AST_I2CM_IER);
				dev_dbg(i2c_bus->dev, "intr en %x \n", aspeed_i2c_read(i2c_bus,
						AST_I2CM_IER));
			} else {
				aspeed_i2c_write(i2c_bus, (aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
							AST_I2CM_TX_ACK) & ~AST_I2CM_TX_NAK, AST_I2CM_IER);
			}
			aspeed_i2c_write(i2c_bus, i2c_bus->dma_addr, I2C_DMA_BASE_REG);
			aspeed_i2c_write(i2c_bus, (i2c_bus->master_xfer_len), I2C_DMA_LEN_REG);
		}
		aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
		dev_dbg(i2c_bus->dev, "txfer size %d , cmd = %x \n", i2c_bus->master_xfer_len,
			cmd);

	} else if (i2c_bus->master_xfer_cnt < i2c_bus->master_msgs->len) {
		//Next send
		if (i2c_bus->master_msgs->flags & I2C_M_RD) {
			//Rx data
			cmd = AST_I2CM_RX_CMD | AST_I2CM_RX_DMA_EN;

			if ((i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt) > i2c_bus->bus_config->dma_size) {
				i2c_bus->master_xfer_len = i2c_bus->bus_config->dma_size;
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
					      AST_I2CM_RX_DONE, AST_I2CM_IER);

			} else {
				i2c_bus->master_xfer_len = i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt;
				if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) &&
				    (i2c_bus->blk_r_flag == 0)) {
					dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN \n");
					aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
						      AST_I2CM_RX_DONE, AST_I2CM_IER);
				} else {
					if (i2c_bus->xfer_last == 1) {
						dev_dbg(i2c_bus->dev, "last stop \n");
						cmd |= AST_I2CM_STOP_CMD;
						aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
							      ~AST_I2CM_RX_DONE, AST_I2CM_IER);
						dev_dbg(i2c_bus->dev, "intr en %x \n", aspeed_i2c_read(i2c_bus,
								AST_I2CM_IER));
					} else {
						aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
							      AST_I2CM_RX_DONE, AST_I2CM_IER);
					}
					//TODO check....
					cmd |= AST_I2CM_RX_CMD_LAST;
				}

			}
			aspeed_i2c_write(i2c_bus, i2c_bus->dma_addr, I2C_DMA_BASE_REG);
			aspeed_i2c_write(i2c_bus, (i2c_bus->master_xfer_len), I2C_DMA_LEN_REG);
			aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
			dev_dbg(i2c_bus->dev, "rxfer size %d , cmd = %x \n", i2c_bus->master_xfer_len,
				cmd);
		} else {
			//Tx data
			//next data write
			cmd = AST_I2CM_TX_CMD | AST_I2CM_TX_DMA_EN;
			if ((i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt) > i2c_bus->bus_config->dma_size) {
				i2c_bus->master_xfer_len = i2c_bus->bus_config->dma_size;
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
					      AST_I2CM_TX_ACK, AST_I2CM_IER);

			} else {
				i2c_bus->master_xfer_len = i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt;
				if (i2c_bus->xfer_last == 1) {
					dev_dbg(i2c_bus->dev, "last stop \n");
					cmd |= AST_I2CM_STOP_CMD;
					aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
						      ~AST_I2CM_TX_ACK, AST_I2CM_IER);
					dev_dbg(i2c_bus->dev, "intr en %x \n", aspeed_i2c_read(i2c_bus,
							AST_I2CM_IER));
				} else {
					aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
						      AST_I2CM_TX_ACK, AST_I2CM_IER);
				}
			}

			for (i = 0; i < i2c_bus->master_xfer_len; i++)
				i2c_bus->dma_buf[i] = i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i];

			aspeed_i2c_write(i2c_bus, i2c_bus->dma_addr, I2C_DMA_BASE_REG);
			aspeed_i2c_write(i2c_bus, (i2c_bus->master_xfer_len), I2C_DMA_LEN_REG);
			aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
			dev_dbg(i2c_bus->dev, "txfer size %d , cmd = %x \n", i2c_bus->master_xfer_len,
				cmd);

		}
	} else {
		//should send next msg
		if (i2c_bus->master_xfer_cnt != i2c_bus->master_msgs->len)
			printk("complete rx ... ERROR \n");

		dev_dbg(i2c_bus->dev, "aspeed_i2c_do_byte_xfer complete \n");
		i2c_bus->cmd_err = 0;
		complete(&i2c_bus->cmd_complete);
	}

}

static void aspeed_i2c_do_pool_xfer(struct aspeed_i2c_bus *i2c_bus)
{
	u32 cmd = 0;
	int i;
	u32 *tx_buf;

	dev_dbg(i2c_bus->dev, "aspeed_i2c_do_pool_xfer \n");

	tx_buf = (u32 *) i2c_bus->buff_pool.page_addr;

	if (i2c_bus->slave_operation) {
		i2c_bus->slave_xfer_mode = BUFF_MODE;
		if (i2c_bus->slave_msgs->flags & I2C_M_RD) {
			dev_dbg(i2c_bus->dev, "(<--) slave tx buf \n");

			if (i2c_bus->slave_msgs->len > i2c_bus->buff_pool.page_size)
				i2c_bus->slave_xfer_len = i2c_bus->buff_pool.page_size;
			else
				i2c_bus->slave_xfer_len = i2c_bus->slave_msgs->len;

			for (i = 0; i < i2c_bus->slave_xfer_len; i++) {
				if (i % 4 == 0)
					tx_buf[i / 4] = 0;
				tx_buf[i / 4] |= (i2c_bus->slave_msgs->buf[i2c_bus->slave_xfer_cnt + i] << ((
							  i % 4) * 8)) ;
				dev_dbg(i2c_bus->dev, "[%x] ", tx_buf[i / 4]);
			}
			//ast2400 sw always fix from page_add_point 0
			aspeed_i2c_write(i2c_bus, AST_I2CC_TX_DATA_BUF_END_SET((i2c_bus->slave_xfer_len -
					1)), AST_I2CC_BUFF_CTRL);

			aspeed_i2c_write(i2c_bus, AST_I2CS_TX_BUFF_EN | AST_I2CS_TX_CMD,
				      AST_I2CS_CMD_STS);
		} else {
			//prepare for new rx
			dev_dbg(i2c_bus->dev, "prepare rx buff pool %d \n",
				i2c_bus->buff_pool.page_size);
			//ast2400 sw always fix from page_add_point 0
			aspeed_i2c_write(i2c_bus, AST_I2CC_RX_BUF_END_ADDR_SET((
						i2c_bus->buff_pool.page_size - 1)), AST_I2CC_BUFF_CTRL);

			aspeed_i2c_write(i2c_bus, AST_I2CS_RX_BUFF_EN, AST_I2CS_CMD_STS);

		}
	} else {
		i2c_bus->master_xfer_mode = BUFF_MODE;
		dev_dbg(i2c_bus->dev, "M cnt %d, xf len %d \n", i2c_bus->master_xfer_cnt,
			i2c_bus->master_msgs->len);
		if (i2c_bus->master_xfer_cnt == -1) {
			//send start
			dev_dbg(i2c_bus->dev, " %sing %d byte%s %s 0x%02x\n",
				i2c_bus->master_msgs->flags & I2C_M_RD ? "read" : "write",
				i2c_bus->master_msgs->len, i2c_bus->master_msgs->len > 1 ? "s" : "",
				i2c_bus->master_msgs->flags & I2C_M_RD ? "from" : "to",
				i2c_bus->master_msgs->addr);

			if (i2c_bus->master_msgs->flags & I2C_M_RD) {
//workaround .. HW can;t send start read addr with buff mode
				cmd = AST_I2CM_START_CMD | AST_I2CM_TX_CMD;
				aspeed_i2c_write(i2c_bus, (i2c_bus->master_msgs->addr << 1) | 0x1,
					      AST_I2CC_STS_AND_BUFF);

//				tx_buf[0] = (i2c_bus->master_msgs->addr <<1); //+1
				i2c_bus->master_xfer_len = 1;
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
					      AST_I2CM_TX_ACK | AST_I2CM_TX_NAK, AST_I2CM_IER);
			} else {
				cmd = AST_I2CM_START_CMD | AST_I2CM_TX_CMD | AST_I2CM_TX_BUFF_EN;
				tx_buf[0] = (i2c_bus->master_msgs->addr << 1);	//+1
				//next data write
				if ((i2c_bus->master_msgs->len + 1) > i2c_bus->buff_pool.page_size)
					i2c_bus->master_xfer_len = i2c_bus->buff_pool.page_size;
				else
					i2c_bus->master_xfer_len = i2c_bus->master_msgs->len + 1;

				for (i = 1; i < i2c_bus->master_xfer_len; i++) {
					if (i % 4 == 0)
						tx_buf[i / 4] = 0;
					tx_buf[i / 4] |= (i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i] << ((
								  i % 4) * 8)) ;
				}

				if ((i2c_bus->xfer_last == 1) &&
				    (i2c_bus->master_xfer_len == i2c_bus->master_msgs->len + 1)) {
					dev_dbg(i2c_bus->dev, "last stop \n");
					cmd |= AST_I2CM_STOP_CMD;
					aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
						      ~AST_I2CM_TX_ACK &
						      ~AST_I2CM_TX_NAK, AST_I2CM_IER);
					dev_dbg(i2c_bus->dev, "intr en %x \n", aspeed_i2c_read(i2c_bus,
							AST_I2CM_IER));
				} else {
					aspeed_i2c_write(i2c_bus, (aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
								AST_I2CM_TX_ACK) & ~AST_I2CM_TX_NAK, AST_I2CM_IER);
				}
				//ast2400 sw always fix from page_add_point 0
				aspeed_i2c_write(i2c_bus, AST_I2CC_TX_DATA_BUF_END_SET((i2c_bus->master_xfer_len -
						1)), AST_I2CC_BUFF_CTRL);
			}
			aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
			dev_dbg(i2c_bus->dev, "txfer size %d , cmd = %x \n", i2c_bus->master_xfer_len,
				cmd);

		} else if (i2c_bus->master_xfer_cnt < i2c_bus->master_msgs->len) {
			//Next send
			if (i2c_bus->master_msgs->flags & I2C_M_RD) {
				//Rx data
				cmd = AST_I2CM_RX_CMD | AST_I2CM_RX_BUFF_EN;

				if ((i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt) >
				    i2c_bus->buff_pool.page_size) {
					i2c_bus->master_xfer_len = i2c_bus->buff_pool.page_size;
					aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
						      AST_I2CM_RX_DONE, AST_I2CM_IER);
				} else {
					i2c_bus->master_xfer_len = i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt;
					if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) &&
					    (i2c_bus->blk_r_flag == 0)) {
						dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN \n");
						aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
							      AST_I2CM_RX_DONE, AST_I2CM_IER);
					} else {
						if (i2c_bus->xfer_last == 1) {
							dev_dbg(i2c_bus->dev, "last stop \n");
							cmd |= AST_I2CM_STOP_CMD;
							aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
								      ~AST_I2CM_RX_DONE, AST_I2CM_IER);
							dev_dbg(i2c_bus->dev, "intr en %x \n", aspeed_i2c_read(i2c_bus,
									AST_I2CM_IER));
						} else {
							aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
								      AST_I2CM_RX_DONE, AST_I2CM_IER);
						}
						cmd |= AST_I2CM_RX_CMD_LAST;
					}
				}
				//ast2400 sw always fix from page_add_point 0
				aspeed_i2c_write(i2c_bus, AST_I2CC_RX_BUF_END_ADDR_SET((i2c_bus->master_xfer_len -
						1)), AST_I2CC_BUFF_CTRL);
				aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
				dev_dbg(i2c_bus->dev, "rxfer size %d , cmd = %x \n", i2c_bus->master_xfer_len,
					cmd);
			} else {
				//Tx data
				//next data write
				cmd = AST_I2CM_TX_CMD | AST_I2CM_TX_BUFF_EN;
				if ((i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt) >
				    i2c_bus->buff_pool.page_size) {
					i2c_bus->master_xfer_len = i2c_bus->buff_pool.page_size;
					aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
						      AST_I2CM_TX_ACK, AST_I2CM_IER);

				} else {
					i2c_bus->master_xfer_len = i2c_bus->master_msgs->len - i2c_bus->master_xfer_cnt;
					if (i2c_bus->xfer_last == 1) {
						dev_dbg(i2c_bus->dev, "last stop \n");
						cmd |= AST_I2CM_STOP_CMD;
						aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
							      ~AST_I2CM_TX_ACK, AST_I2CM_IER);
						dev_dbg(i2c_bus->dev, "intr en %x \n", aspeed_i2c_read(i2c_bus,
								AST_I2CM_IER));
					} else {
						aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
							      AST_I2CM_TX_ACK, AST_I2CM_IER);
					}
				}

				for (i = 0; i < i2c_bus->master_xfer_len; i++) {
					if (i % 4 == 0)
						tx_buf[i / 4] = 0;
					tx_buf[i / 4] |= (i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i] << ((
								  i % 4) * 8)) ;
				}
//				printk("count %x \n",aspeed_i2c_read(i2c_bus,AST_I2CM_CMD_STS));
				//ast2400 sw always fix from page_add_point 0
				aspeed_i2c_write(i2c_bus, AST_I2CC_TX_DATA_BUF_END_SET((i2c_bus->master_xfer_len -
						1)), AST_I2CC_BUFF_CTRL);

				aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
				dev_dbg(i2c_bus->dev, "txfer size %d , cmd = %x \n", i2c_bus->master_xfer_len,
					cmd);
			}
		} else {
			//should send next msg
			if (i2c_bus->master_xfer_cnt != i2c_bus->master_msgs->len)
				printk("complete rx ... ERROR \n");

			dev_dbg(i2c_bus->dev, "aspeed_i2c_do_byte_xfer complete \n");
			i2c_bus->cmd_err = 0;
			complete(&i2c_bus->cmd_complete);
		}

	}
}

#ifdef CONFIG_AST_I2C_SLAVE_MODE
//for memory buffer initial
static void ast_i2c_slave_buff_init(struct aspeed_i2c_bus *i2c_bus)
{
	int i;
	//Tx buf  1
	i2c_bus->slave_tx_msg.len = I2C_S_BUF_SIZE;
	i2c_bus->slave_tx_msg.buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	//Rx buf 4
	for (i = 0; i < I2C_S_RX_BUF_NUM; i++) {
		i2c_bus->slave_rx_msg[i].addr = 0;	//mean ~BUFF_ONGOING
		i2c_bus->slave_rx_msg[i].flags = 0;	//mean empty buffer
		i2c_bus->slave_rx_msg[i].len = I2C_S_BUF_SIZE;
		i2c_bus->slave_rx_msg[i].buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	}
	//assign cur msg is #0
	i2c_bus->slave_msgs = &i2c_bus->slave_rx_msg[0];
	i2c_bus->slave_rx_full = 0;
	i2c_bus->slave_rx_idx = 0;
	i2c_bus->slave_ioctl_idx = 0;

}

static void ast_i2c_slave_rdwr_xfer(struct aspeed_i2c_bus *i2c_bus)
{
	switch (i2c_bus->slave_event) {
	case I2C_SLAVE_EVENT_START_WRITE:
		dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_START_WRITE\n");
		if (i2c_bus->slave_msgs->addr) {
			//use repeat start, use over write buffer ..
			dev_err(i2c_bus->dev, "slave ERROR S[%d], sts[%x] 0x00[%x]~~ TODO ~~~~!!\n",
				i2c_bus->slave_operation, (aspeed_i2c_read(i2c_bus, AST_I2CS_CMD_STS) >> 19) & 0xf,
				aspeed_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL));
			dev_err(i2c_bus->dev, "slave msg flag [%x] len [%d]\n",
				i2c_bus->slave_msgs->flags, i2c_bus->slave_msgs->len);
			dev_err(i2c_bus->dev, "slave ioctl idx [%d] rx_idx [%d] rx_full [%d]\n",
				i2c_bus->slave_ioctl_idx, i2c_bus->slave_rx_idx, i2c_bus->slave_rx_full);
		} else {
			i2c_bus->slave_msgs->addr = BUFF_ONGOING;
		}
		if (i2c_bus->slave_rx_msg[(i2c_bus->slave_rx_idx + 1) % I2C_S_RX_BUF_NUM].flags
		    == BUFF_FULL) {
			i2c_bus->slave_rx_full = 1;
			dev_err(i2c_bus->dev, "buffer full\n com[%x] \n", aspeed_i2c_read(i2c_bus,
					AST_I2CS_CMD_STS));
#if 1
			//if ((i2c_bus->slave_rx_idx + 1) % I2C_S_RX_BUF_NUM == i2c_bus->slave_ioctl_idx) {
			dev_err(i2c_bus->dev, "disable slave to stop recv\n");
			aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
							    AST_I2CC_FUN_CTRL) & ~(AST_I2CC_SLAVE_EN), AST_I2CC_FUN_CTRL);
			//}
#endif
		}	//will enable when slave rx stop
		break;
	case I2C_SLAVE_EVENT_START_READ:
		printk("I2C_SLAVE_EVENT_START_READ ERROR .. not imple \n");
		i2c_bus->slave_msgs = &i2c_bus->slave_tx_msg;
		break;
	case I2C_SLAVE_EVENT_WRITE:
		//check buffer full ?
		if (i2c_bus->slave_msgs->len > I2C_S_BUF_SIZE)
			dev_dbg(i2c_bus->dev, "ERROR : buffer is out ~~!! \n");
		break;
	case I2C_SLAVE_EVENT_READ:
		printk("I2C_SLAVE_EVENT_READ ERROR .. not imple \n");
		dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_READ ERROR ... \n");
		i2c_bus->slave_msgs = &i2c_bus->slave_tx_msg;
		break;
	case I2C_SLAVE_EVENT_NACK:
		printk("I2C_SLAVE_EVENT_NACK ERROR .. not imple \n");
		dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_NACK ERROR ... \n");
		i2c_bus->slave_msgs = &i2c_bus->slave_tx_msg;
		break;
	case I2C_SLAVE_EVENT_STOP:
		dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_STOP buf idx %d, rx len : %d\n",
			i2c_bus->slave_rx_idx, i2c_bus->slave_msgs->len);
		i2c_bus->slave_msgs->addr = 0;
		i2c_bus->slave_msgs->flags = BUFF_FULL;
		i2c_bus->slave_rx_idx++;
		i2c_bus->slave_rx_idx %= I2C_S_RX_BUF_NUM;
		i2c_bus->slave_msgs = &i2c_bus->slave_rx_msg[i2c_bus->slave_rx_idx];
		//i2c_bus->slave_msgs->len = 0;
		if (i2c_bus->slave_rx_full) {
			printk("slave full check !!! \n");
		} else {
			aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus,
							    AST_I2CC_FUN_CTRL) | (AST_I2CC_SLAVE_EN),
				      AST_I2CC_FUN_CTRL);	//will enable when slave rx stop
		}
		i2c_bus->slave_operation = 0;
		break;
	}

}

static int ast_i2c_slave_ioctl_xfer(struct i2c_adapter *adap,
				    struct i2c_msg *msgs)
{
	struct aspeed_i2c_bus *i2c_bus = adap->algo_data;
	struct i2c_msg *slave_rx_msg = &i2c_bus->slave_rx_msg[i2c_bus->slave_ioctl_idx];
	int ret = 0;
//	int i;
	switch (msgs->flags) {
	case 0:
		if ((slave_rx_msg->addr == 0) && (slave_rx_msg->flags == BUFF_FULL)) {
			dev_dbg(i2c_bus->dev, "I2C_SLAVE_EVENT_STOP buf idx %d : len %d \n",
				i2c_bus->slave_ioctl_idx, slave_rx_msg->len);
#if 0
			printk("slave rx buff \n");
			for (i = 0; i < slave_rx_msg->len; i++)
				printk("%x ", slave_rx_msg->buf[i]);
			printk("\n");
#endif
			if (slave_rx_msg->len != 0)
				memcpy(msgs->buf, slave_rx_msg->buf, slave_rx_msg->len);
			msgs->len = slave_rx_msg->len;
			slave_rx_msg->len = 0;
			slave_rx_msg->flags = 0;
			i2c_bus->slave_ioctl_idx++;
			i2c_bus->slave_ioctl_idx %= I2C_S_RX_BUF_NUM;
			if (i2c_bus->slave_rx_full) {
				dev_err(i2c_bus->dev, "slave re-enable \n");
				i2c_bus->slave_rx_full = 0;
				if (aspeed_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL) & AST_I2CC_SLAVE_EN) {
					printk("buffer handle error !! ~~~~\n");
				} else {
					if ((i2c_bus->slave_en) && (!i2c_bus->master_operation)) {
						printk("\n %d-slave enable watch out TODO\n", i2c_bus->adap.nr);
						aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL) | AST_I2CC_SLAVE_EN,
							      AST_I2CC_FUN_CTRL);
					} else {
						printk("\n TODO ~~~~~~~~~~~~~~");
					}
				}
			}
		} else {
//				printk("%d-No Buff\n", i2c_bus->adap.nr);
			msgs->len = 0;
			ret = -1;
		}
		break;
	case I2C_M_RD:	//slave write
		dev_dbg(i2c_bus->dev, "slave write \n");
		memcpy(msgs->buf, i2c_bus->slave_tx_msg.buf, I2C_S_BUF_SIZE);
		break;
	case I2C_S_EN:
		if ((msgs->addr < 0x1) || (msgs->addr > 0xff)) {
			ret = -1;
			printk("addrsss not correct !! \n");
			ret = -1;
		} else {
			if (msgs->len != 1) printk("ERROR \n");
			ast_slave_mode_enable(i2c_bus, msgs);
		}
		break;
	case I2C_S_ALT:
		printk("slave issue alt\n");
		if (msgs->len != 1) printk("ERROR \n");
		if (msgs->buf[0] == 1)
			ast_slave_issue_alert(i2c_bus, 1);
		else
			ast_slave_issue_alert(i2c_bus, 0);
		break;
	default:
		printk("slave xfer error \n");
		break;

	}
	return ret;
}

static void ast_i2c_slave_xfer_done(struct aspeed_i2c_bus *i2c_bus)
{
	u32 xfer_len = 0;
	int i;
	u8 *rx_buf;

	dev_dbg(i2c_bus->dev, "ast_i2c_slave_xfer_done [%d]\n",
		i2c_bus->slave_xfer_mode);

	if (i2c_bus->slave_msgs->flags & I2C_M_RD) {
		dev_dbg(i2c_bus->dev, "slave tx \n");
		//tx done , only check tx count ...
		if (i2c_bus->slave_xfer_mode == BYTE_MODE) {
			xfer_len = 1;
		} else if (i2c_bus->slave_xfer_mode == BUFF_MODE) {
			xfer_len = AST_I2CC_TX_DATA_BUF_GET(aspeed_i2c_read(i2c_bus, AST_I2CC_BUFF_CTRL));
			xfer_len++;
			dev_dbg(i2c_bus->dev, "S tx buff done len %d \n", xfer_len);
		} else if (i2c_bus->slave_xfer_mode == INC_DMA_MODE) {
			//DMA mode
			xfer_len = aspeed_i2c_read(i2c_bus, I2C_DMA_LEN_REG);
			xfer_len = i2c_bus->slave_xfer_len - xfer_len;
			dev_dbg(i2c_bus->dev, "S tx tx dma done len %d \n", xfer_len);
		} else if (i2c_bus->slave_xfer_mode == G6_DMA_MODE) {
			xfer_len = AST_G6_I2CD_GET_DMA_LEN(aspeed_i2c_read(i2c_bus, I2C_DMA_LEN_REG));
			dev_dbg(i2c_bus->dev, "S tx tx dma done len %d \n", xfer_len);
		} else {
			printk("ERROR type !! \n");
		}

	} else {
		//rx done
//		dev_dbg(i2c_bus->dev, "rx \n");
		if (i2c_bus->slave_xfer_mode == BYTE_MODE) {
			i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len] = 
					AST_I2CC_GET_RX_BUFF(aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF));
			i2c_bus->slave_msgs->len++;
		} else if (i2c_bus->slave_xfer_mode == BUFF_MODE) {
			xfer_len = AST_I2CC_RX_BUF_ADDR_GET(aspeed_i2c_read(i2c_bus, AST_I2CC_BUFF_CTRL));
			if (i2c_bus->bus_config->aspeed_version == 4) {
				if (xfer_len == 0)
					xfer_len = i2c_bus->buff_pool.page_size;
			}
			rx_buf = (u8 *)i2c_bus->buff_pool.page_addr;
			if ((i2c_bus->slave_msgs->len + xfer_len) > I2C_S_BUF_SIZE) {
				printk("out of buff size ~~~~\n");
			}
			for (i = 0; i < xfer_len ; i++) {
				i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len + i] = rx_buf[i];
				dev_dbg(i2c_bus->dev, "%d, [%x] \n", i2c_bus->slave_xfer_cnt + i,
					i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len + i]);
			}
			i2c_bus->slave_msgs->len += xfer_len;
			//Not with stop so prepare to next rx
			if (!(aspeed_i2c_read(i2c_bus, AST_I2CS_ISR) & AST_I2CS_STOP)) {
				//ast2400 sw always fix from page_add_point 0
				aspeed_i2c_write(i2c_bus, AST_I2CC_RX_BUF_END_ADDR_SET((i2c_bus->buff_pool.page_size - 1)),
					      AST_I2CC_BUFF_CTRL);
				aspeed_i2c_write(i2c_bus, AST_I2CS_RX_BUFF_EN, AST_I2CS_CMD_STS);
			}
		} else if (i2c_bus->slave_xfer_mode == INC_DMA_MODE) {
			//RX DMA DONE
			xfer_len = aspeed_i2c_read(i2c_bus, I2C_DMA_LEN_REG);
			if (xfer_len == 0)
				xfer_len = i2c_bus->bus_config->dma_size;
			else
				xfer_len = i2c_bus->bus_config->dma_size - xfer_len;

			dev_dbg(i2c_bus->dev, " S rx dma done len %d \n", xfer_len);

			dev_dbg(i2c_bus->dev, "0, [%02x] \n", i2c_bus->slave_msgs->buf[0]);
			for (i = 0; i < xfer_len; i++) {
				i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len + i] = i2c_bus->dma_buf[i];
				dev_dbg(i2c_bus->dev, "%d, [%02x] \n", i2c_bus->slave_msgs->len + i,
					i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len + i]);
			}
			i2c_bus->slave_msgs->len += xfer_len;
		} else if (i2c_bus->slave_xfer_mode == G6_DMA_MODE) {
			//RX DMA DONE
			xfer_len = AST_G6_I2CD_GET_DMA_LEN(aspeed_i2c_read(i2c_bus, I2C_DMA_LEN_REG));
			dev_dbg(i2c_bus->dev, " S rx dma done len %d \n", xfer_len);
		
			dev_dbg(i2c_bus->dev, "0, [%02x] \n", i2c_bus->slave_msgs->buf[0]);
			for (i = 0; i < xfer_len; i++) {
				i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len + i] = i2c_bus->dma_buf[i];
				dev_dbg(i2c_bus->dev, "%d, [%02x] \n", i2c_bus->slave_msgs->len + i,
					i2c_bus->slave_msgs->buf[i2c_bus->slave_msgs->len + i]);
			}
			i2c_bus->slave_msgs->len += xfer_len;

		} else {
			printk("ERROR !! XFER Type \n");
		}

	}

}

static void ast_i2c_slave_addr_match(struct aspeed_i2c_bus *i2c_bus)
{
	u8 match;

	i2c_bus->slave_operation = 1;
	i2c_bus->slave_xfer_cnt = 0;
	match = AST_I2CC_GET_RX_BUFF(aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF));
	i2c_bus->slave_msgs->buf[0] = match;
	i2c_bus->slave_msgs->len = 1;
	dev_dbg(i2c_bus->dev, "S Start Addr match [%x] \n", match);

	if (match & 1) {
		i2c_bus->slave_event = I2C_SLAVE_EVENT_START_READ;
	} else {
		i2c_bus->slave_event = I2C_SLAVE_EVENT_START_WRITE;
	}

	ast_i2c_slave_rdwr_xfer(i2c_bus);

	i2c_bus->do_slave_xfer(i2c_bus);

}

int aspeed_i2c_slave_handler(struct aspeed_i2c_bus *i2c_bus)
{
	u32 sts = aspeed_i2c_read(i2c_bus, AST_I2CS_ISR);

	if ((sts & (AST_I2CS_RX_DONE | AST_I2CS_SLAVE_MATCH)) ==
	    (AST_I2CS_RX_DONE | AST_I2CS_SLAVE_MATCH)) {
		if (sts & AST_I2CS_STOP) {
			I2C_S_DBUG("%d-P|S|W\n", i2c_bus->adap.nr);
			i2c_bus->slave_event = I2C_SLAVE_EVENT_STOP;
			ast_i2c_slave_rdwr_xfer(i2c_bus);
			aspeed_i2c_write(i2c_bus, AST_I2CS_STOP, AST_I2CS_ISR);
			sts &= ~AST_I2CS_STOP;
			if (!(aspeed_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL) & AST_I2CC_SLAVE_EN)) {
				printk("full - disable \n");
				return IRQ_HANDLED;
			}
		}
		ast_i2c_slave_addr_match(i2c_bus);
		dev_dbg(i2c_bus->dev,
			"S clear isr: AST_I2CS_RX_DONE | AST_I2CS_SLAVE_MATCH = %x\n",
			sts);
		aspeed_i2c_write(i2c_bus, AST_I2CS_RX_DONE |
			      AST_I2CS_SLAVE_MATCH, AST_I2CS_ISR);
		sts &= ~(AST_I2CS_RX_DONE | AST_I2CS_SLAVE_MATCH);
	}

	if (!sts)
		return IRQ_HANDLED;

	switch (sts) {
	case AST_I2CS_TX_ACK:
		i2c_bus->slave_event = I2C_SLAVE_EVENT_READ;
		ast_i2c_slave_xfer_done(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CS_TX_ACK = %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CS_TX_ACK, AST_I2CS_ISR);
		break;
	case AST_I2CS_TX_ACK | AST_I2CS_STOP:
		printk("ast_i2c: S  TX_ACK | NORMAL_STOP;  TODO ~~~");
		break;
	case AST_I2CS_TX_NAK:
		printk("ast_i2c: S  TX_NAK ~~\n");
		i2c_bus->slave_event = I2C_SLAVE_EVENT_NACK;
		ast_i2c_slave_rdwr_xfer(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CS_TX_NAK = %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CS_TX_NAK, AST_I2CS_ISR);
		break;
	case AST_I2CS_TX_NAK | AST_I2CS_STOP:
		printk("SLAVE TODO .... \n");
		break;

	//Issue : Workaround for I2C slave mode
	case AST_I2CS_TX_NAK | AST_I2CS_SLAVE_MATCH:
		i2c_bus->slave_event = I2C_SLAVE_EVENT_NACK;
		printk("isr AST_I2CS_TX_NAK | AST_I2CS_SLAVE_MATCH TODO ~~~~~~\n");
		ast_i2c_slave_xfer_done(i2c_bus);
		ast_i2c_slave_addr_match(i2c_bus);
		aspeed_i2c_write(i2c_bus, AST_I2CS_TX_NAK | AST_I2CS_SLAVE_MATCH
			      , AST_I2CS_ISR);
		break;
	case AST_I2CS_RX_DONE:
		i2c_bus->slave_event = I2C_SLAVE_EVENT_WRITE;
		ast_i2c_slave_xfer_done(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CS_RX_DONE = %x\n", sts);
		ast_i2c_slave_rdwr_xfer(i2c_bus);
		aspeed_i2c_write(i2c_bus, AST_I2CS_RX_DONE, AST_I2CS_ISR);
		break;

	case AST_I2CS_STOP:
		i2c_bus->slave_event = I2C_SLAVE_EVENT_STOP;
		ast_i2c_slave_rdwr_xfer(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CS_STOP = %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CS_STOP, AST_I2CS_ISR);
		dev_dbg(i2c_bus->dev, "state [%x] \n", i2c_bus->state);
		break;
	case (AST_I2CS_RX_DONE | AST_I2CS_STOP):
		i2c_bus->slave_event = I2C_SLAVE_EVENT_WRITE;
		ast_i2c_slave_xfer_done(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CS_RX_DONE = %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CS_RX_DONE, AST_I2CS_ISR);
		i2c_bus->slave_event = I2C_SLAVE_EVENT_STOP;
		ast_i2c_slave_rdwr_xfer(i2c_bus);
		dev_dbg(i2c_bus->dev, "S clear isr: AST_I2CS_STOP = %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CS_STOP, AST_I2CS_ISR);
		break;
	default:
		printk("%d-Slave No one care : isr :%x, state %x\n", i2c_bus->adap.nr, sts,
		       i2c_bus->state);
		break;
	}

}

static void aspeed_i2c_slave_do_dma_xfer(struct aspeed_i2c_bus *i2c_bus)
{
	int i;

	i2c_bus->master_xfer_mode = INC_DMA_MODE;
	i2c_bus->slave_xfer_mode = INC_DMA_MODE;
	dev_dbg(i2c_bus->dev, "aspeed_i2c_do_dma_xfer \n");

	dev_dbg(i2c_bus->dev, "S cnt %d, xf len %d \n", i2c_bus->slave_xfer_cnt,
		i2c_bus->slave_msgs->len);
	if (i2c_bus->slave_msgs->flags & I2C_M_RD) {
		//DMA tx mode
		if (i2c_bus->slave_msgs->len > i2c_bus->bus_config->dma_size)
			i2c_bus->slave_xfer_len = i2c_bus->bus_config->dma_size;
		else
			i2c_bus->slave_xfer_len = i2c_bus->slave_msgs->len;

		dev_dbg(i2c_bus->dev, "(<--) slave tx DMA len %d \n", i2c_bus->slave_xfer_len);
		for (i = 0; i < i2c_bus->slave_xfer_len; i++)
			i2c_bus->dma_buf[i] = i2c_bus->slave_msgs->buf[i2c_bus->slave_xfer_cnt + i];

		aspeed_i2c_write(i2c_bus, i2c_bus->dma_addr, I2C_DMA_BASE_REG);
		aspeed_i2c_write(i2c_bus, i2c_bus->slave_xfer_len, I2C_DMA_LEN_REG);
		aspeed_i2c_write(i2c_bus, AST_I2CS_TX_DMA_EN | AST_I2CS_TX_CMD, AST_I2CS_CMD_STS);
	} else {
		//DMA prepare rx
		dev_dbg(i2c_bus->dev, "(-->) slave prepare rx DMA len %d \n", i2c_bus->bus_config->dma_size);
		aspeed_i2c_write(i2c_bus, i2c_bus->dma_addr, I2C_DMA_BASE_REG);
		aspeed_i2c_write(i2c_bus, i2c_bus->bus_config->dma_size, I2C_DMA_LEN_REG);
		aspeed_i2c_write(i2c_bus, AST_I2CS_RX_DMA_EN, AST_I2CS_CMD_STS);
	}

}

static void aspeed_i2c_slave_do_byte_xfer(struct aspeed_i2c_bus *i2c_bus)
{
	u8 *xfer_buf;
	u32 cmd = 0;

	dev_dbg(i2c_bus->dev, "aspeed_i2c_do_byte_xfer \n");
	i2c_bus->slave_xfer_mode = BYTE_MODE;
	i2c_bus->slave_xfer_len = 1;

	dev_dbg(i2c_bus->dev, "S cnt %d, xf len %d \n", i2c_bus->slave_xfer_cnt,
		i2c_bus->slave_msgs->len);
	if (i2c_bus->slave_msgs->flags & I2C_M_RD) {
		//READ <-- TX
		dev_dbg(i2c_bus->dev, "(<--) slave(tx) buf %d [%x]\n", i2c_bus->slave_xfer_cnt,
			i2c_bus->slave_msgs->buf[i2c_bus->slave_xfer_cnt]);
		aspeed_i2c_write(i2c_bus, i2c_bus->slave_msgs->buf[i2c_bus->slave_xfer_cnt],
			      AST_I2CC_STS_AND_BUFF);
		aspeed_i2c_write(i2c_bus, AST_I2CS_TX_CMD, AST_I2CS_CMD_STS);
	} else {
		// Write -->Rx
		//no need to handle in byte mode
		dev_dbg(i2c_bus->dev, "(-->) slave(rx) BYTE do nothing\n");
	}

}

#endif
static void aspeed_i2c_master_do_byte_xfer(struct aspeed_i2c_bus *i2c_bus)
{
	u32 cmd = 0;

	dev_dbg(i2c_bus->dev, "aspeed_i2c_do_byte_xfer \n");

	i2c_bus->master_xfer_mode = BYTE_MODE;
	i2c_bus->master_xfer_len = 1;

	if (i2c_bus->master_msgs->flags & I2C_M_NOSTART) {
		dev_dbg(i2c_bus->dev, "I2C_M_NOSTART xfer \n");
	} else {
		dev_dbg(i2c_bus->dev, " %sing %d byte%s %s 0x%02x\n",
			i2c_bus->master_msgs->flags & I2C_M_RD ? "read" : "write",
			i2c_bus->master_msgs->len, i2c_bus->master_msgs->len > 1 ? "s" : "",
			i2c_bus->master_msgs->flags & I2C_M_RD ? "from" : "to",
			i2c_bus->master_msgs->addr);

		cmd = AST_I2CM_PKT_EN | AST_I2CM_PKT_ADDR(i2c_bus->master_msgs->addr) | AST_I2CM_START_CMD;
		if (i2c_bus->master_msgs->flags & I2C_M_RD) {
			cmd |= AST_I2CM_RX_CMD;
			if ((i2c_bus->master_msgs->len == 1) && (i2c_bus->xfer_last)) {
				dev_dbg(i2c_bus->dev, "last stop \n");
				cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
			}
		} else {
			if (i2c_bus->master_msgs->len) {
				cmd |= AST_I2CM_TX_CMD;
				aspeed_i2c_write(i2c_bus, i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt], AST_I2CC_STS_AND_BUFF);
				dev_dbg(i2c_bus->dev, "M: tx %d:[%x] \n", i2c_bus->master_xfer_cnt, i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt]);
			}
			if ((i2c_bus->master_msgs->len == 1) && (i2c_bus->xfer_last)) {
				dev_dbg(i2c_bus->dev, "last stop \n");
				cmd |= AST_I2CM_STOP_CMD;
			}
		}
		dev_dbg(i2c_bus->dev, "cmd %x \n", cmd);
		aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
	}
///////////////// TODO ~~~
#if 0
	} else if (i2c_bus->master_xfer_cnt < i2c_bus->master_msgs->len) {
		xfer_buf = i2c_bus->master_msgs->buf;
		if (i2c_bus->master_msgs->flags & I2C_M_RD) {
			//Rx data
			cmd = AST_I2CM_RX_CMD;
			if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) &&
			    (i2c_bus->master_xfer_cnt == 0)) {
				dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN \n");
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
					      AST_I2CM_RX_DONE, AST_I2CM_IER);

			} else if (i2c_bus->xfer_last &&
				   (i2c_bus->master_xfer_cnt + 1 == i2c_bus->master_msgs->len)) {
				cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_STOP_CMD;
				//				disable rx_dwn isr
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
					      ~AST_I2CM_RX_DONE, AST_I2CM_IER);
			} else {
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
					      AST_I2CM_RX_DONE, AST_I2CM_IER);
			}

			dev_dbg(i2c_bus->dev, "(<--) rx byte, cmd = %x \n", cmd);

			aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);


		} else {
			//Tx data
			dev_dbg(i2c_bus->dev, "(-->) xfer byte data index[%02x]:%02x  \n",
				i2c_bus->master_xfer_cnt, *(xfer_buf + i2c_bus->master_xfer_cnt));
			aspeed_i2c_write(i2c_bus, *(xfer_buf + i2c_bus->master_xfer_cnt),
				      AST_I2CC_STS_AND_BUFF);
			if (i2c_bus->xfer_last &&
			    (i2c_bus->master_xfer_cnt + 1 == i2c_bus->master_msgs->len)) {
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
					      ~AST_I2CM_TX_ACK, AST_I2CM_IER);
				aspeed_i2c_write(i2c_bus, AST_I2CM_TX_CMD | AST_I2CM_STOP_CMD, AST_I2CM_CMD_STS);
			} else {
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) |
					      AST_I2CM_TX_ACK, AST_I2CM_IER);
				aspeed_i2c_write(i2c_bus, AST_I2CM_TX_CMD, AST_I2CM_CMD_STS);
			}
		}

	} else {
		//should send next msg
		if (i2c_bus->master_xfer_cnt != i2c_bus->master_msgs->len)
			printk("CNT ERROR \n");

		dev_dbg(i2c_bus->dev, "aspeed_i2c_do_byte_xfer complete \n");
		i2c_bus->cmd_err = 0;
		complete(&i2c_bus->cmd_complete);

	}
#endif	
}

static void aspeed_i2c_master_xfer_done(struct aspeed_i2c_bus *i2c_bus)
{
	u32 xfer_len = 0;
	int i;
	u32 cmd = AST_I2CM_PKT_EN;
	u8 *pool_buf;

	dev_dbg(i2c_bus->dev, "aspeed_i2c_master_xfer_done mode[%d] %s\n",
		i2c_bus->master_xfer_mode,
		i2c_bus->master_msgs->flags & I2C_M_RD ? "read" : "write");

	if (i2c_bus->master_msgs->flags & I2C_M_RD) {
		if (i2c_bus->master_xfer_mode == BYTE_MODE) {
			if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_bus->master_xfer_cnt == 0)) {
				i2c_bus->master_msgs->len += AST_I2CC_GET_RX_BUFF(aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF));
				dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN %d \n", i2c_bus->master_msgs->len - 1);
			}
			i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt] = AST_I2CC_GET_RX_BUFF(aspeed_i2c_read(i2c_bus, AST_I2CC_STS_AND_BUFF));
			dev_dbg(i2c_bus->dev, "M: rx %d:[%x] \n", i2c_bus->master_xfer_cnt, i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt]);			
			i2c_bus->master_xfer_cnt++;
			if (i2c_bus->master_msgs->len == i2c_bus->master_xfer_cnt) {
				i2c_bus->cmd_err = 0;
				dev_dbg(i2c_bus->dev, "M: rx complete \n");
				complete(&i2c_bus->cmd_complete);
			} else {
				if(i2c_bus->xfer_last)
					cmd = AST_I2CM_STOP_CMD;
				cmd |= AST_I2CM_RX_CMD_LAST | AST_I2CM_RX_CMD;
				aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
			}
		} else if (i2c_bus->master_xfer_mode == BUFF_MODE) {
			//TODO ~~
			pool_buf = (u8 *)i2c_bus->buff_pool.page_addr;
			xfer_len = AST_I2CC_RX_BUF_ADDR_GET(aspeed_i2c_read(i2c_bus, AST_I2CC_BUFF_CTRL));
			if (i2c_bus->bus_config->aspeed_version == 4) {
				if (xfer_len == 0)
					xfer_len = i2c_bus->buff_pool.page_size;
			}
			for (i = 0; i < xfer_len; i++) {
				i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i] = pool_buf[i];
				dev_dbg(i2c_bus->dev, "rx %d buff[%x]\n", i2c_bus->master_xfer_cnt + i,
					i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i]);
			}

			if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) &&
			    (i2c_bus->blk_r_flag == 0)) {
				i2c_bus->master_msgs->len += pool_buf[0];
				i2c_bus->blk_r_flag = 1;
				dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN %d \n", i2c_bus->master_msgs->len - 1);
			}
		} else if (i2c_bus->master_xfer_mode == DMA_MODE) {
			//TODO ~~
			xfer_len = AST_G6_I2CD_GET_DMA_LEN(aspeed_i2c_read(i2c_bus, I2C_DMA_LEN_REG));
			for (i = 0; i < xfer_len; i++) {
				i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt + i] = i2c_bus->dma_buf[i];
				dev_dbg(i2c_bus->dev, "buf[%x] \n", i2c_bus->dma_buf[i]);
				dev_dbg(i2c_bus->dev, "buf[%x] \n", i2c_bus->dma_buf[i + 1]);
			}
			if ((i2c_bus->master_msgs->flags & I2C_M_RECV_LEN) &&
				(i2c_bus->blk_r_flag == 0)) {
				i2c_bus->master_msgs->len += i2c_bus->dma_buf[0];
				i2c_bus->blk_r_flag = 1;
				dev_dbg(i2c_bus->dev, "I2C_M_RECV_LEN %d \n", i2c_bus->master_msgs->len - 1);
			}
		} else {
			printk("ERROR xfer type \n");
		}

	} else {
		if (i2c_bus->master_xfer_mode == BYTE_MODE) {
			i2c_bus->master_xfer_cnt++;
			if(i2c_bus->master_xfer_cnt == i2c_bus->master_msgs->len) {
				i2c_bus->cmd_err = 0;
				dev_dbg(i2c_bus->dev, "M: tx complete \n");
				complete(&i2c_bus->cmd_complete);
			} else {
				dev_dbg(i2c_bus->dev, "M: tx %d:[%x] \n", i2c_bus->master_xfer_cnt, i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt]);
				aspeed_i2c_write(i2c_bus, i2c_bus->master_msgs->buf[i2c_bus->master_xfer_cnt], AST_I2CC_STS_AND_BUFF);
				if(i2c_bus->xfer_last)
					cmd = AST_I2CM_STOP_CMD;
				cmd |= AST_I2CM_PKT_EN | AST_I2CM_TX_CMD;
				aspeed_i2c_write(i2c_bus, cmd, AST_I2CM_CMD_STS);
			}
		} else if (i2c_bus->master_xfer_mode == BUFF_MODE) {
			xfer_len = AST_I2CC_TX_DATA_BUF_GET(aspeed_i2c_read(i2c_bus, AST_I2CC_BUFF_CTRL));
			xfer_len++;
			dev_dbg(i2c_bus->dev, "tx buff done len %d \n", xfer_len);
		} else if (i2c_bus->master_xfer_mode == DMA_MODE) {
			xfer_len = AST_G6_I2CD_GET_DMA_LEN(aspeed_i2c_read(i2c_bus, I2C_DMA_LEN_REG));
			dev_dbg(i2c_bus->dev, "tx dma done len %d \n", xfer_len);
		} else {
			printk("ERROR xfer type \n");
		}
	}

}

int aspeed_i2c_master_handler(struct aspeed_i2c_bus *i2c_bus)
{
	u32 sts = aspeed_i2c_read(i2c_bus, AST_I2CM_ISR);
	dev_dbg(i2c_bus->dev, "M sts %x\n", sts);

	if (AST_I2CM_BUS_RECOVER_FAIL & sts) {
		printk("AST_I2CM_BUS_RECOVER_FAIL \n");
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_BUS_RECOVER_FAIL= %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CM_BUS_RECOVER_FAIL, AST_I2CM_ISR);
		if (i2c_bus->bus_recover) {
			complete(&i2c_bus->cmd_complete);
			i2c_bus->cmd_err = AST_I2CM_BUS_RECOVER_FAIL;
			i2c_bus->bus_recover = 0;
		} else {
			printk("Error !! Bus revover\n");
		}
		
		return 1;
	}
	
	if (AST_I2CM_BUS_RECOVER & sts) {
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_BUS_RECOVER= %x\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CM_BUS_RECOVER, AST_I2CM_ISR);
		i2c_bus->cmd_err = 0;
		if (i2c_bus->bus_recover) {
			complete(&i2c_bus->cmd_complete);
			i2c_bus->bus_recover = 0;
		} else {
			printk("Error !! Bus revover\n");
		}
		return 1;
	}
	
	if (AST_I2CM_SMBUS_ALT & sts) {
		printk("AST_I2CM_SMBUS_ALT \n");
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_SMBUS_ALT= %x\n", sts);
		//Disable ALT INT
		aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_IER) &
			      ~AST_I2CM_SMBUS_ALT,
			      AST_I2CM_IER);
		aspeed_i2c_write(i2c_bus, AST_I2CM_SMBUS_ALT, AST_I2CM_ISR);
		printk("TODO aspeed_master_alert_recv bus id %d, Disable Alt, Please Imple \n",
			   i2c_bus->adap.nr);
		return 1;
	}

	if ((AST_I2CM_PKT_ERROR & sts) && !(AST_I2CM_PKT_DONE & sts)) {
		printk("DO AST_I2CM_PKT_ERROR \n");
		dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_PKT_ERROR = %x ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n", sts);
		aspeed_i2c_write(i2c_bus, AST_I2CM_PKT_ERROR, AST_I2CM_ISR);
		
		if (AST_I2CM_PKT_TIMEOUT & sts) {
			i2c_bus->cmd_err = AST_I2CM_PKT_TIMEOUT;
			aspeed_i2c_write(i2c_bus, AST_I2CM_PKT_TIMEOUT, AST_I2CM_ISR);
			complete(&i2c_bus->cmd_complete);
			return 1;
		}
		
		if (AST_I2CM_SDA_DL_TO & sts) {
			i2c_bus->cmd_err = AST_I2CM_SDA_DL_TO;
			aspeed_i2c_write(i2c_bus, AST_I2CM_SDA_DL_TO, AST_I2CM_ISR);
			complete(&i2c_bus->cmd_complete);
			return 1;
		}
		
		if (AST_I2CM_SCL_LOW_TO & sts) {
			i2c_bus->cmd_err = AST_I2CM_SCL_LOW_TO;
			aspeed_i2c_write(i2c_bus, AST_I2CM_SCL_LOW_TO, AST_I2CM_ISR);
			complete(&i2c_bus->cmd_complete);
			return 1;
		}

		if (AST_I2CM_ABNORMAL_COND & sts) {
			i2c_bus->cmd_err = AST_I2CM_ABNORMAL_COND;
			aspeed_i2c_write(i2c_bus, AST_I2CM_ABNORMAL_COND, AST_I2CM_ISR);
			complete(&i2c_bus->cmd_complete);
			return 1;
		}

		if (AST_I2CM_ARBIT_LOSS & sts) {
			i2c_bus->cmd_err = AST_I2CM_ARBIT_LOSS;
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_ARBIT_LOSS = %x\n", sts);
			aspeed_i2c_write(i2c_bus, AST_I2CM_ARBIT_LOSS, AST_I2CM_ISR);
			complete(&i2c_bus->cmd_complete);
			return 1;
		}

		if (AST_I2CM_TX_NAK & sts) {
			dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_TX_NAK = %x\n", sts);
			aspeed_i2c_write(i2c_bus, AST_I2CM_TX_NAK, AST_I2CM_ISR);
			if (i2c_bus->master_msgs->flags == I2C_M_IGNORE_NAK) {
				dev_dbg(i2c_bus->dev, "I2C_M_IGNORE_NAK next send\n");
				i2c_bus->cmd_err = 0;
			} else {
				dev_dbg(i2c_bus->dev, "NAK error\n");
				i2c_bus->cmd_err = AST_I2CM_TX_NAK;
			}
			complete(&i2c_bus->cmd_complete);
		}
		switch (sts) {
		}
		
	}

	if (AST_I2CM_PKT_DONE & sts) {
		sts &= ~(AST_I2CM_PKT_DONE | AST_I2CM_PKT_ERROR);
		switch (sts) {
			case 0:
				printk("workaround for write 0 byte \n");
				aspeed_i2c_write(i2c_bus,  AST_I2CM_PKT_DONE, AST_I2CM_ISR);
				dev_dbg(i2c_bus->dev, "AST_I2CM_PKT_DONE \n");
				i2c_bus->cmd_err = 0;
				complete(&i2c_bus->cmd_complete);		
				break;
			case AST_I2CM_TX_ACK:
				dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_TX_ACK = %x\n", sts);
				aspeed_i2c_write(i2c_bus, AST_I2CM_TX_ACK | AST_I2CM_PKT_DONE, AST_I2CM_ISR);
				aspeed_i2c_master_xfer_done(i2c_bus);
				break;
				
			case AST_I2CM_TX_NAK | AST_I2CM_NORMAL_STOP:
				aspeed_i2c_write(i2c_bus, AST_I2CM_TX_NAK | AST_I2CM_NORMAL_STOP |
							AST_I2CM_PKT_DONE | AST_I2CM_PKT_ERROR, AST_I2CM_ISR);
				dev_dbg(i2c_bus->dev, "M TX NAK | NORMAL STOP \n");
				i2c_bus->cmd_err = AST_I2CM_TX_NAK | AST_I2CM_NORMAL_STOP;
				complete(&i2c_bus->cmd_complete);
				break;
				
			case AST_I2CM_TX_ACK | AST_I2CM_NORMAL_STOP:
				dev_dbg(i2c_bus->dev,
					"M clear isr: AST_I2CM_TX_ACK | AST_I2CM_NORMAL_STOP= %x\n",
					sts);
				aspeed_i2c_write(i2c_bus, AST_I2CM_TX_ACK | AST_I2CM_NORMAL_STOP |
							AST_I2CM_PKT_DONE, AST_I2CM_ISR);
				complete(&i2c_bus->cmd_complete);
				break;

			case AST_I2CM_RX_DONE:
				dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_RX_DONE = %x\n", sts);
				aspeed_i2c_write(i2c_bus, AST_I2CM_RX_DONE, AST_I2CM_ISR);
				complete(&i2c_bus->cmd_complete);
				break;
			case AST_I2CM_RX_DONE | AST_I2CM_NORMAL_STOP:
				dev_dbg(i2c_bus->dev,
					"M clear isr: AST_I2CM_RX_DONE | AST_I2CM_NORMAL_STOP = %x\n", sts);
				aspeed_i2c_write(i2c_bus, AST_I2CM_PKT_DONE | AST_I2CM_RX_DONE | 
							AST_I2CM_NORMAL_STOP, AST_I2CM_ISR);
				aspeed_i2c_master_xfer_done(i2c_bus);
				break;
#if 0				


			case AST_I2CM_NORMAL_STOP:
				dev_dbg(i2c_bus->dev, "M clear isr: AST_I2CM_NORMAL_STOP = %x\n", sts);
				aspeed_i2c_write(i2c_bus, AST_I2CM_NORMAL_STOP, AST_I2CM_ISR);
				i2c_bus->cmd_err = 0;
				break;
#endif				
			default:
				printk("TODO care -- > sts %x \n", aspeed_i2c_read(i2c_bus, AST_I2CM_ISR));
				aspeed_i2c_write(i2c_bus, aspeed_i2c_read(i2c_bus, AST_I2CM_ISR), AST_I2CM_ISR);
				break;
			
		}
		
		return 1;
	}
	return 0;
}

static irqreturn_t aspeed_i2c_handler(int irq, void *dev_id)
{
	struct aspeed_i2c_bus *i2c_bus = dev_id;

#ifdef CONFIG_AST_I2C_SLAVE_MODE	
	if(aspeed_i2c_slave_handler(i2c_bus)) {
		dev_dbg(i2c_bus->dev, "slave handle \n");
		return IRQ_HANDLED;
	}
#endif
	return aspeed_i2c_master_handler(i2c_bus) ? IRQ_HANDLED : IRQ_NONE;
}

static int aspeed_i2c_do_msgs_xfer(struct aspeed_i2c_bus *i2c_bus,
				struct i2c_msg *msgs, int num)
{
	int i;
	int ret = 0;
	long timeout = 0;

	dev_dbg(i2c_bus->dev, "aspeed_i2c_do_msgs_xfer\n");

	for (i = 0; i < num; i++) {
		init_completion(&i2c_bus->cmd_complete);
		i2c_bus->cmd_err = 0;
		
		i2c_bus->blk_r_flag = 0;
		i2c_bus->master_msgs = &msgs[i];
		if (num == i + 1)
			i2c_bus->xfer_last = 1;
		else
			i2c_bus->xfer_last = 0;

		i2c_bus->master_xfer_cnt = 0;

		i2c_bus->do_master_xfer(i2c_bus);

		timeout = wait_for_completion_timeout(&i2c_bus->cmd_complete,
						      i2c_bus->adap.timeout * HZ);

		if (timeout <= 0) {
			u32 reg_val = aspeed_i2c_read(i2c_bus, AST_I2CC_FUN_CTRL);
			if (timeout == 0) {
				dev_err(i2c_bus->dev, "controller timed out\n");
				ret = -ETIMEDOUT;
			} else if (timeout == -ERESTARTSYS) {
				dev_err(i2c_bus->dev, "controller ERESTARTSYS\n");
				ret = -ERESTARTSYS;
			}
			aspeed_i2c_write(i2c_bus, reg_val & ~AST_I2CC_MASTER_EN, AST_I2CC_FUN_CTRL);
			aspeed_i2c_write(i2c_bus, reg_val | AST_I2CC_MASTER_EN, AST_I2CC_FUN_CTRL);
			goto out;
		}

		if (i2c_bus->cmd_err != 0) {
			if (i2c_bus->cmd_err == (AST_I2CM_TX_NAK |
						 AST_I2CM_NORMAL_STOP)) {
				dev_dbg(i2c_bus->dev, "normal nak go out \n");
				ret = -ETIMEDOUT;
				goto out;
			} else if (i2c_bus->cmd_err == AST_I2CM_ABNORMAL_COND) {
				dev_dbg(i2c_bus->dev, "abnormal go out \n");
				ret = -ETIMEDOUT;
				goto out;
			} else {
				dev_dbg(i2c_bus->dev, "send stop \n");
				ret = -EAGAIN;
				goto stop;
			}
		}
		ret++;
	}

	if (i2c_bus->cmd_err == 0)
		goto out;
stop:
	printk("TODO ~~");
	init_completion(&i2c_bus->cmd_complete);
	aspeed_i2c_write(i2c_bus, AST_I2CM_STOP_CMD, AST_I2CM_CMD_STS);
	timeout = wait_for_completion_timeout(&i2c_bus->cmd_complete,
					      i2c_bus->adap.timeout * HZ);
	if (timeout == 0) {
		dev_dbg(i2c_bus->dev, "send stop timed out\n");
		i2c_bus->state = (aspeed_i2c_read(i2c_bus, AST_I2CM_CMD_STS) >> 19) & 0xf;
		dev_dbg(i2c_bus->dev, "sts [%x], isr sts [%x] \n", i2c_bus->state,
			aspeed_i2c_read(i2c_bus, AST_I2CM_ISR));
		ret = -ETIMEDOUT;
	}

out:
	return ret;

}

static int aspeed_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct aspeed_i2c_bus *i2c_bus = adap->algo_data;
	int ret, i;

	/*
	 * Wait for the bus to become free.
	 */
	ret = aspeed_i2c_wait_bus_not_busy(i2c_bus);
	if (ret) {
		dev_dbg(i2c_bus->dev, "i2c_ast: timeout waiting for bus free\n");
		goto out;
	}

	for (i = adap->retries; i >= 0; i--) {
		ret = aspeed_i2c_do_msgs_xfer(i2c_bus, msgs, num);
		if (ret != -EAGAIN)
			goto out;
		dev_dbg(i2c_bus->dev, "Retrying transmission [%d]\n", i);
		udelay(100);
	}

	ret = -EREMOTEIO;
out:
	dev_dbg(i2c_bus->dev, "bus%d-m: end \n", i2c_bus->adap.nr);

	return ret;
}

static void aspeed_i2c_bus_init(struct aspeed_i2c_bus *i2c_bus)
{
	//I2CG Reset
	aspeed_i2c_write(i2c_bus, 0, AST_I2CC_FUN_CTRL);

	aspeed_i2c_write(i2c_bus, AST_I2CC_BUS_AUTO_RELEASE | AST_I2CC_MASTER_EN,
		      AST_I2CC_FUN_CTRL);

	/* Set AC Timing */
	aspeed_i2c_write(i2c_bus, aspeed_select_i2c_clock(i2c_bus), AST_I2CC_AC_TIMING);
	printk("set AC timeing %x \n", aspeed_i2c_read(i2c_bus, AST_I2CC_AC_TIMING));

	//Clear Interrupt
	aspeed_i2c_write(i2c_bus, 0xfffffff, AST_I2CM_ISR);


	/* Set interrupt generation of I2C master controller */
	aspeed_i2c_write(i2c_bus,
		      AST_I2CM_SDA_DL_TO |
		      AST_I2CM_BUS_RECOVER |
		      AST_I2CM_SMBUS_ALT |
		      AST_I2CM_SCL_LOW_TO |
		      AST_I2CM_ABNORMAL_COND |
		      AST_I2CM_NORMAL_STOP |
		      AST_I2CM_ARBIT_LOSS |
		      AST_I2CM_RX_DONE |
		      AST_I2CM_TX_NAK |
		      AST_I2CM_TX_ACK,
		      AST_I2CM_IER);

	aspeed_i2c_write(i2c_bus, 0xfffffff, AST_I2CS_ISR);

	/* Set interrupt generation of I2C master controller */
	aspeed_i2c_write(i2c_bus,
		      AST_I2CS_INACTIVE_TO |
		      AST_I2CS_ABNOR_STOP |
		      AST_I2CS_STOP |
		      AST_I2CS_MATCH_NAK |
		      AST_I2CS_RX_DONE |
		      AST_I2CS_TX_NAK |
		      AST_I2CS_TX_ACK,
		      AST_I2CS_IER);	
}

static u32 ast_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_aspeed_algorithm = {
	.master_xfer	= aspeed_i2c_xfer,
#ifdef CONFIG_AST_I2C_SLAVE_MODE
	.slave_xfer		= ast_i2c_slave_ioctl_xfer,
#endif
	.functionality	= ast_i2c_functionality,
};

static struct aspeed_i2c_bus_config aspeed_i2c_config = {
	.timing_table_size = sizeof(ast_g6_i2c_timing_table),
	.timing_table = ast_g6_i2c_timing_table,
};

static const struct of_device_id aspeed_i2c_bus_of_table[] = {
	{	.compatible = "aspeed,aspeed-i2c", 	.data = &aspeed_i2c_config, 	},
	{ },
};

MODULE_DEVICE_TABLE(of, aspeed_i2c_bus_of_table);

static int aspeed_i2c_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct aspeed_i2c_bus *i2c_bus;
	struct resource *res;
	int bus_nr;
	int ret = 0;

	dev_dbg(&pdev->dev, "aspeed_i2c_probe \n");

	i2c_bus = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_i2c_bus), GFP_KERNEL);
	if (!i2c_bus) {
		return -ENOMEM;
	}

	i2c_bus->dev = &pdev->dev;
	init_completion(&i2c_bus->cmd_complete);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENODEV;
		goto free_mem;
	}

	i2c_bus->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!i2c_bus->reg_base) {
		ret = -EIO;
		goto release_mem;
	}

	i2c_bus->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (i2c_bus->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -i2c_bus->irq;
		goto free_irq;
	}

	match = of_match_node(aspeed_i2c_bus_of_table, pdev->dev.of_node);
	if (!match) {
		ret = -ENOENT;
		goto free_irq;
	}
	i2c_bus->bus_config = (struct aspeed_i2c_bus_config *)match->data;

	/*defaule is byte mode */
	i2c_bus->master_dma = BYTE_MODE;
	i2c_bus->slave_dma = BYTE_MODE;
	i2c_bus->do_master_xfer = aspeed_i2c_master_do_byte_xfer;
//	i2c_bus->do_slave_xfer = aspeed_i2c_slave_do_byte_xfer;

	//TODO
#if 0
	if (of_device_is_compatible(pdev->dev.of_node, "aspeed,ast-buff-i2c")) {
		i2c_bus->master_dma = BUFF_MODE;
		i2c_bus->slave_dma = BUFF_MODE;
		i2c_bus->do_master_xfer = aspeed_i2c_do_pool_xfer;
		i2c_bus->do_slave_xfer = aspeed_i2c_do_pool_xfer;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
			ret = -ENOENT;
			goto free_mem;
		}
		i2c_bus->buff_pool.page_addr = (u32) devm_ioremap_resource(&pdev->dev, res);
		if (!i2c_bus->buff_pool.page_addr) {
			dev_err(&pdev->dev, "no buff pool information \n");
			ret = -EIO;
			goto release_mem;
		}
		i2c_bus->buff_pool.page_size = resource_size(res);
		dev_dbg(&pdev->dev, "ast_i2c buff mode enable, page addr %x, page size %x \n",
			i2c_bus->buff_pool.page_addr, i2c_bus->buff_pool.page_size);
	}

	if (of_device_is_compatible(pdev->dev.of_node, "aspeed,ast-dma-i2c")) {
		i2c_bus->master_dma = INC_DMA_MODE;
		i2c_bus->slave_dma = INC_DMA_MODE;
		i2c_bus->do_master_xfer = aspeed_i2c_master_do_dma_xfer;
		i2c_bus->do_slave_xfer = aspeed_i2c_master_do_dma_xfer;
		i2c_bus->dma_buf = dma_alloc_coherent(NULL, i2c_bus->bus_config->dma_size,
						      &i2c_bus->dma_addr, GFP_KERNEL);
		if (!i2c_bus->dma_buf) {
			dev_err(&pdev->dev, "unable to allocate tx Buffer memory\n");
			ret = -ENOMEM;
			goto free_mem;
		}
		if (i2c_bus->dma_addr % 4 != 0) {
			dev_err(&pdev->dev, "not 4 byte boundary \n");
			ret = -ENOMEM;
			goto free_mem;
		}
		dev_dbg(&pdev->dev,
			"dma enable dma_buf = [0x%x] dma_addr = [0x%x], please check 4byte boundary \n",
			(u32)i2c_bus->dma_buf, i2c_bus->dma_addr);
		memset(i2c_bus->dma_buf, 0, i2c_bus->bus_config->dma_size);
	}
#endif
	dev_dbg(&pdev->dev, "master mode  [%d] slave mode [%d]\n", i2c_bus->master_dma,
		i2c_bus->slave_dma);

	platform_set_drvdata(pdev, i2c_bus);

	i2c_bus->clk = devm_clk_get(i2c_bus->dev, NULL);
	if (IS_ERR(i2c_bus->clk)) {
		dev_err(i2c_bus->dev, "no clock defined\n");
		return -ENODEV;
	}
	i2c_bus->apb_clk = clk_get_rate(i2c_bus->clk);
	dev_dbg(i2c_bus->dev, "i2c_bus->apb_clk %d \n", i2c_bus->apb_clk);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &i2c_bus->bus_frequency);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		i2c_bus->bus_frequency = 100000;
	}

	/* Initialize the I2C adapter */
	i2c_set_adapdata(&i2c_bus->adap, i2c_bus);
	i2c_bus->adap.owner = THIS_MODULE;
	i2c_bus->adap.class = I2C_CLASS_DEPRECATED;
	i2c_bus->adap.algo = &i2c_aspeed_algorithm;
//		i2c_bus->adap.quirks = &;
	i2c_bus->adap.dev.parent = i2c_bus->dev;
	i2c_bus->adap.timeout = 3;
	i2c_bus->adap.dev.of_node = pdev->dev.of_node;
	i2c_bus->adap.algo_data = i2c_bus;
	i2c_bus->adap.retries = 0;

	/* Try to set the I2C adapter number from dt */
	bus_nr = of_alias_get_id(pdev->dev.of_node, "i2c");

	snprintf(i2c_bus->adap.name, sizeof(i2c_bus->adap.name), "aspeed_i2c.%u",
		 bus_nr);

	i2c_bus->slave_operation = 0;
	i2c_bus->blk_r_flag = 0;

	aspeed_i2c_bus_init(i2c_bus);

#ifdef CONFIG_AST_I2C_SLAVE_MODE
	ast_i2c_slave_buff_init(i2c_bus);
#endif

	ret = devm_request_irq(&pdev->dev, i2c_bus->irq, aspeed_i2c_handler,
			       0, dev_name(&pdev->dev), i2c_bus);
	if (ret) {
		printk(KERN_INFO "I2C: Failed request irq %d\n", i2c_bus->irq);
		goto unmap;
	}

	ret = i2c_add_adapter(&i2c_bus->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto free_irq;
	}

	printk(KERN_INFO "I2C: %s [%d]: AST I2C mode [%d] adapter [%d khz] \n",
	       pdev->dev.of_node->name, i2c_bus->adap.nr, i2c_bus->master_dma,
	       i2c_bus->bus_frequency / 1000);

	return 0;

unmap:
	free_irq(i2c_bus->irq, i2c_bus);
free_irq:
	iounmap(i2c_bus->reg_base);
release_mem:
	release_mem_region(res->start, resource_size(res));
free_mem:
	kfree(i2c_bus);

	return ret;
}

static int aspeed_i2c_remove(struct platform_device *pdev)
{
	struct aspeed_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	struct resource *res;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c_bus->adap);

	free_irq(i2c_bus->irq, i2c_bus);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(i2c_bus->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c_bus);

	return 0;
}

#ifdef CONFIG_PM
static int aspeed_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TODO
//	struct aspeed_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	return 0;
}

static int aspeed_i2c_resume(struct platform_device *pdev)
{
	//TODO
//	struct aspeed_i2c_bus *i2c_bus = platform_get_drvdata(pdev);
	//Should reset i2c ???
	return 0;
}
#endif

static struct platform_driver aspeed_i2c_bus_driver = {
	.probe		= aspeed_i2c_probe,
	.remove		= aspeed_i2c_remove,
#ifdef CONFIG_PM
	.suspend	= aspeed_i2c_suspend,
	.resume		= aspeed_i2c_resume,
#endif
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_i2c_bus_of_table,
	},
};

module_platform_driver(aspeed_i2c_bus_driver);


MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED I2C Bus Driver");
MODULE_LICENSE("GPL");
