/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	api_intc.h
Desc 	:	定义INTC的寄存器结构体\寄存器位的宏定义
Author 	:  	yangkai
Date 	:	2008-11-05
Notes 	:   
$Log: api_intc.h,v $
********************************************************************/
#ifndef _API_INTC_H
#define _API_INTC_H
#include <asm/arch/typedef.h>

/********************************************************************
**                            宏定义                                *
********************************************************************/

/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef enum _IRQ_NUM
{
    IRQ_DWDMA = 0,          // 0  -- low
    IRQ_UHI,                // 1  -- USB Host Interface
    IRQ_NANDC,              // 2
    IRQ_LCDC,               // 3
    IRQ_SDMMC0,             // 4
    IRQ_VIP,                // 5
    IRQ_GPIO0,              // 6
    IRQ_GPIO1,              // 7
    IRQ_OTG,                // 8  -- USB OTG
    IRQ_ABTARMD,            // 9  -- Arbiter in ARMD BUS
    IRQ_ABTEXP,             // 10 -- Arbiter in EXP BUS
    IRQ_I2C0,               // 11
    IRQ_I2C1,               // 12
    IRQ_I2S,                // 13
    IRQ_SPIM,               // 14 -- SPI Master
    IRQ_SPIS,               // 15 -- SPI Slave
    IRQ_TIMER1,             // 16
    IRQ_TIMER2,             // 17
    IRQ_TIMER3,             // 18
    IRQ_UART0,              // 19
    IRQ_UART1,              // 20
    IRQ_WDT,                // 21
    IRQ_PWM0,               // 22
    IRQ_PWM1,               // 23
    IRQ_PWM2,               // 24
    IRQ_PWM3,               // 25
    IRQ_ADC,                // 26
    IRQ_RTC,                // 27
    IRQ_PIUSEM0,            // 28 -- PIU Semphore 0
    IRQ_PIUSEM1,            // 29
    IRQ_PIUSEM3,            // 30
    IRQ_PIUCMD,             // 31 -- PIU command/reply
    IRQ_XDMA,               // 32
    IRQ_SDMMC1,             // 33
    IRQ_DSPSEI,             // 34 -- DSP slave interface error interrupt
    IRQ_DSPSWI,             // 35 -- DSP interrupt by software set
    IRQ_SCU,                // 36
    IRQ_SWI,                // 37 -- Software Interrupt
    IRQ_DSPMEI,             // 38 -- DSP master interface error interrupt
    IRQ_DSPSAEI,            // 39 -- DSP system access error interrupt
    IRQ_MAXNUM             // 40  -- interrupt 
}eIRQ_NUM;

typedef enum FIQ_NUM
{
    FIQ_GPIO0 = 0,
    FIQ_SWI,
    FIQ_MAXNUM
}eFIQ_NUM;


/********************************************************************
**                          变量定义                                *
********************************************************************/

/********************************************************************
**                      对外函数接口声明                            *
********************************************************************/
extern uint32 INTCInit(void);                          //中断控制器初始化
// 注册中断服务程序，关联物理中断与中断服务
extern uint32 IRQRegISR(eIRQ_NUM irqNum, pFunc routine);
extern uint32 IRQEnable(eIRQ_NUM irqNum);              //IRQ使能
extern uint32 IRQDisable(eIRQ_NUM irqNum);             //IRQ禁止

extern uint32 FIQRegISR(eFIQ_NUM fiqNum, pFunc Routine);
extern uint32 FIQEnable(eFIQ_NUM fiqNum);              //FIQ 使能
extern uint32 FIQDisable(eFIQ_NUM fiqNum);             //FIQ禁止
extern void   FIQGenFIQ1(void);                        //打开FIQ1中断
extern void   FIQClrFIQ1(void);                        //清除FIQ1中断
 void rockchip_mask_irq(unsigned int irq);
  void rockchip_unmask_irq(unsigned int irq);



/********************************************************************
**                          表格定义                                *
********************************************************************/

#endif
