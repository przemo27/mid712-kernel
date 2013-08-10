/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	intc.c
Desc 	:	定义INTC的寄存器结构体\寄存器位的宏定义
Author 	:  	yangkai
Date 	:	2008-11-05
Notes 	:   
$Log: intc.h,v $
********************************************************************/
#include <asm/arch/hardware.h>

#ifndef _INTC_H
#define _INTC_H

/********************************************************************
**                            宏定义                                *
********************************************************************/
#define PRI_MAX 16  //最多含有16个优先级，取值为0~15
/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef volatile struct tagINTC_REG
{   /*offset 0x00~0x30*/
    unsigned int IRQ_INTEN_L;      //IRQ interrupt source enable register (low)
    unsigned int IRQ_INTEN_H;      //IRQ interrupt source enable register (high)
    unsigned int IRQ_INTMASK_L;    //IRQ interrupt source mask register (low).
    unsigned int IRQ_INTMASK_H;    //IRQ interrupt source mask register (high).
    unsigned int IRQ_INTFORCE_L;   //IRQ interrupt force register
    unsigned int IRQ_INTFORCE_H;   //
    unsigned int IRQ_RAWSTATUS_L;  //IRQ raw status register
    unsigned int IRQ_RAWSTATUS_H;  //
    unsigned int IRQ_STATUS_L;     //IRQ status register
    unsigned int IRQ_STATUS_H;     //
    unsigned int IRQ_MASKSTATUS_L; //IRQ interrupt mask status register
    unsigned int IRQ_MASKSTATUS_H; //
    unsigned int IRQ_FINALSTATUS_L;//IRQ interrupt final status
    unsigned int IRQ_FINALSTATUS_H; 
    unsigned int reserved0[(0xC0-0x38)/4];
    
    /*offset 0xc0~0xd8*/
    unsigned int FIQ_INTEN;        //Fast interrupt enable register
    unsigned int FIQ_INTMASK;      //Fast interrupt mask register
    unsigned int FIQ_INTFORCE;     //Fast interrupt force register
    unsigned int FIQ_RAWSTATUS;    //Fast interrupt source raw status register
    unsigned int FIQ_STATUS;       //Fast interrupt status register
    unsigned int FIQ_FINALSTATUS;  //Fast interrupt final status register
    unsigned int IRQ_PLEVEL;       //IRQ System Priority Level Register
    unsigned int reserved1[(0xe8-0xdc)/4];
    
    /*offset 0xe8~0xe8+39*4*/
    unsigned int IRQ_PN_OFFSET[40];//Interrupt N priority level register(s),
                             // where N is from 0 to 15
    unsigned int reserved2[(0x3f0-0x188)/4];
    
    /*offset 0x3f0~0x3fc*/
    unsigned int ICTL_COMP_PARAMS_2;   //Component Parameter Register 2
    unsigned int ICTL_COMP_PARAMS_1;   //Component Parameter Register 1
    unsigned int AHB_ICTL_COMP_VERSION;//Version register
    unsigned int ICTL_COMP_TYPE;       //Component Type Register
}INTC_REG, *pINTC_REG;


/********************************************************************
**                          变量定义                                *
********************************************************************/
#undef EXT
#ifdef IN_INTC
    #define EXT
#else    
    #define EXT extern
#endif    
    
EXT     pFunc g_irqVectorTable[40];  // IRQ controller interrupt handle vector table
EXT     pFunc g_fiqVectorTable[2];

#define g_intcReg 	((pINTC_REG)(INTC_BASE_ADDR_VA))

#endif
