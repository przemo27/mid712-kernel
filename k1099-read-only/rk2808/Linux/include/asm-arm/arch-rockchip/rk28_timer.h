/* include/asm-arm/arch-ROCK28/timer.h
**
** Copyright (C) 2009 ROCKCHIP, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#ifndef __ASM_ARCH_TIMER_H
#define __ASM_ARCH_TIMER_H

enum {
	TIMER_TIME_LOW          = 0x00, // get low bits of current time and update TIMER_TIME_HIGH
	TIMER_TIME_HIGH         = 0x04, // get high bits of time at last TIMER_TIME_LOW read
	TIMER_ALARM_LOW         = 0x08, // set low bits of alarm and activate it
	TIMER_ALARM_HIGH        = 0x0c, // set high bits of next alarm
	TIMER_CLEAR_INTERRUPT   = 0x10,
	TIMER_CLEAR_ALARM       = 0x14
};

/********************************************************************
 * **                       2806 timer                                 *
 * ********************************************************************/

enum
{
    TIMER1 = 0,
    TIMER2,
    TIMER3,
    TIMER_MAX
};
/*
 * rk28 timer register offset from RK28_TIMER
 */

enum
{
     Timer1LoadCount 	= 0x00 ,     	// Load Count Register
     Timer1CurrentValue = 0x04 ,  	// Current Value Register
     Timer1ControlReg	= 0x08 ,    	// Control Register
     Timer1EOI          = 0x0c ,  	// End-of-Interrupt Register
     Timer1IntStatus	= 0x10 ,     	// Interrupt Status Register
     Timer2LoadCount	= 0x14 ,
     Timer2CurrentValue = 0x18 ,
     Timer2ControlReg	= 0x1c ,
     Timer2EOI		= 0x20 ,
     Timer2IntStatus	= 0x24 ,
     Timer3LoadCount	= 0x28 ,
     Timer3CurrentValue	= 0x2c ,
     Timer3ControlReg	= 0x30 ,
     Timer3EOI		= 0x34 ,
     Timer3IntStatus	= 0x38 ,
     TimersIntStatus	= 0xa0 ,     	// Interrupt Status Register
     TimersEOI		= 0xa4 ,        // End-of-Interrupt Register
     TimersRawIntStatus = 0xa8 ,  		// Raw Interrupt Status Register
};

/*
 *macro to get timer register offset from APB_BASEADD_PA 
 */


#define     Timer1_LoadCount     = (RK28_TIMER + 0x00)        // Load Count Register
#define     Timer1_CurrentValue  = (RK28_TIMER + 0x04)        // Current Value Register
#define     Timer1_ControlReg    = (RK28_TIMER + 0x08)        // Control Register
#define	   	RK28_TIMER_TEN	(1<<0)			/*enable timer*/
#define		RK28_TIMER_TTM	(1<<1)			/*user-defined count mode*/
#define		RK28_TIMER_TIM	(1<<2)			/*interrupt mask*/
#define     Timer1_EOI           = (RK28_TIMER + 0x0c)       // End-of-Interrupt Register
#define     Timer1_IntStatus     = (RK28_TIMER + 0x10)        // Interrupt Status Register
#define     Timer2_LoadCount     = (RK28_TIMER + 0x14)
#define     Timer2_CurrentValue  = (RK28_TIMER + 0x18)
#define     Timer2_ControlReg    = (RK28_TIMER + 0x1c)
#define     Timer2_EOI           = (RK28_TIMER + 0x20)
#define     Timer2_IntStatus     = (RK28_TIMER + 0x24)
#define     Timer3_LoadCount     = (RK28_TIMER + 0x28)
#define     Timer3_CurrentValue  = (RK28_TIMER + 0x2c) 
#define     Timer3_ControlReg    = (RK28_TIMER + 0x30) 
#define     Timer3_EOI           = (RK28_TIMER + 0x34)
#define     Timer3_IntStatus     = (RK28_TIMER + 0x38) 
#define     Timers_IntStatus     = (RK28_TIMER + 0xa0)        // Interrupt Status Register
#define     Timers_EOI           = (RK28_TIMER + 0xa4)        // End-of-Interrupt Register
#define     Timers_RawIntStatus  = (RK28_TIMER + 0xa8)        // Raw Interrupt Status Register


#endif
