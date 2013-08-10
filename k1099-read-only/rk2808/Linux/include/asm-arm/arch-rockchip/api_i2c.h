/*
 *  *  *  linux/include/asm-arm/arch-rockchip/api.i2c.h
 *   *
 * Copyright (C) ROCKCHIP
 * Author	: WQQ
 * Date 	:2009-04-21
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * Common hardware definitions
 */

#ifndef __ROCKCHIP_DEVICE_I2C_H
#define __ROCKCHIP_DEVICE_I2C_H
#include "typedef.h"


//zhaojun add below

#define START_BIT				 (1)
#define NO_START_BIT			 (0)

//I2C_IER
#define I2C_ARBITR_LOSE_ENABLE	 (1<<7)  //Arbitration lose interrupt

//I2C_ISR
#define I2C_ARBITR_LOSE_STATUS	 (1<<7)  //Arbitration lose STATUS
#define I2C_RECE_INT_MACKP		 (1<<1) //Master ACK period interrupt status bit
#define I2C_RECE_INT_MACK		 (1)	 //Master receives ACK interrupt status bit


//I2C_LSR
#define I2C_LSR_RECE_NACK		 (1<<1)

//I2C_LCMR
#define I2C_LCMR_RESUME 		 (1<<2)
#define I2C_LCMR_STOP			 (1<<1)
#define I2C_LCMR_START			 (1)

//I2C_CONR
#define I2C_CON_ACK 			 (0)
#define I2C_CON_NACK			 (1<<4)
#define I2C_MASTER_TRAN_MODE	 (1<<3)
#define I2C_MASTER_RECE_MODE	 (0)
#define I2C_MASTER_PORT_ENABLE	 (1<<2)
#define I2C_SLAVE_RECE_MODE 	 (0)
#define I2C_SLAVE_TRAN_MODE 	 (1<<1)
#define I2C_SLAVE_PORT_ENABLE	 (1)


//I2C_OPR
#define SLAVE_7BIT_ADDRESS_MODE  (0)
#define SLAVE_10BIT_ADDRESS_MODE  (1<<8)
#define RESET_I2C_STATUS		 (1<<7)
#define I2C_CORE_ENABLE 		 (1<<6)
#define I2C_CORE_DISABLE		 (0)


#define I2C_READ_BIT			 (1)
#define I2C_WRITE_BIT			 (0)


//I2C Registers
typedef volatile struct tagIIC_STRUCT
{
	uint32 I2C_MTXR;
	uint32 I2C_MRXR;
	uint32 I2C_STXR;
	uint32 I2C_SRXR;
	uint32 I2C_SADDR;
	uint32 I2C_IER;
	uint32 I2C_ISR;
	uint32 I2C_LCMR;
	uint32 I2C_LSR;
	uint32 I2C_CONR;
	uint32 I2C_OPR;
}I2C_REG,*pI2C_REG;



typedef enum I2C_slaveaddr
{
	tvp5145 = 0x5C,
	WM8987codec = 0x34,
	FM5767 = 0xC0,
	FM5800 = 0x20,
	RTCM41 = 0xD0,
	QN8006_I2C_DEVICE_ADDRESS = 0x56,
	InterCodec = 0x4E
}eI2C_slaveaddr_t;

typedef enum I2C_mode
{
	NORMALMODE = 0,
	DIRECTMODE,
	RegisterMode,
	TS8205MODE,
	NONEMode,
	PCA955XMode,
	NORMALNOSTOPMODE
}eI2C_mode_t;

typedef enum I2C_ch
{
	I2C_CH0,
	I2C_CH1
}eI2C_ch_t;

typedef enum I2C_Address_Reg
{
	I2C_7BIT_ADDRESS_8BIT_REG=0,
	I2C_10BIT_ADDRESS_16BIT_REG,
	I2C_10BIT_ADDRESS_8BIT_REG,
	I2C_7BIT_ADDRESS_16BIT_REG,
	I2C_ADDRESS_BIT_REG_ERR
}eI2C_Address_Reg_t;
/*
#undef	EXT
#ifdef	IN_DRIVER_API_I2C
#define EXT
#else
#define EXT 	extern
#endif

EXT uint16 g_i2cSpeed0;
EXT uint16 g_i2cSpeed1;
EXT uint16 g_i2cSlaveAddr0;
EXT uint16 g_i2cSlaveAddr1;
EXT uint16 g_i2cAddressBit0;
EXT uint16 g_i2cAddressBit1;
EXT uint16 g_i2cMode0;
EXT uint16 g_i2cMode1;
*/

//extern void __init rk28_add_device_i2c(struct i2c_board_info *devices, int nr_devices); 

#include <linux/i2c.h>


#endif
