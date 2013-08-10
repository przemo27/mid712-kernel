/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)   2004 BY ROCK-CHIP FUZHOU
				--  ALL RIGHTS RESERVED  --

File Name:  nand_config.h
Author:     RK28XX Driver Develop Group
Created:    25th OCT 2008
Modified:
Revision:   1.00
********************************************************************************
********************************************************************************/
#ifndef     _NAND_CONFIG_H
#define     _NAND_CONFIG_H
#define     DRIVERS_NAND
#define     LINUX

#include    <linux/kernel.h>
#include    <linux/string.h>
#include    <linux/sched.h>
#include    <linux/delay.h>
#include 	<asm/arch-rockchip/hardware.h>
#include    <asm/arch-rockchip/typedef.h>
#include    "epphal.h"
#include    "flash.h"
#include    "ftl.h"

#ifdef CONFIG_MTD_NAND_RK28XX_DEBUG
#undef RKNAND_DEBUG
#define RKNAND_DEBUG(format, arg...) \
		printk(KERN_NOTICE format, ## arg);
#else
#undef RKNAND_DEBUG
#define RKNAND_DEBUG(n, arg...)
#endif

extern void rkNand_cond_resched(void);

#define COND_RESCHED() rkNand_cond_resched()//cond_resched()

#endif

