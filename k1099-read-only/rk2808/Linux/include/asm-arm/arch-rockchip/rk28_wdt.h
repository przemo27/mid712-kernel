/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
 * File 	:	wdt.h
 * Desc 	:	
 * Author 	:  	wqq
 * Date 	:	2009-06-20
 * Notes 	:   
 * ********************************************************************/
#ifndef _WDT_H
#define _WDT_H

typedef volatile struct tagWDT_REG
{
    uint32 WDT_CR;
    uint32 WDT_TORR;
    uint32 WDT_CCVR;
    uint32 WDT_CRR;
    uint32 WDT_STAT;
    uint32 WDT_EOI;
}WDT_REG,*pWDT_REG;

#define g_wdtReg ((pWDT_REG)WDT_BASE_ADDR)

typedef enum _WDT_PULSELEN
{
    WDT_PLEN_2PCLK,
    WDT_PLEN_4PCLK,
    WDT_PLEN_8PCLK,
    WDT_PLEN_16PCLK,
    WDT_PLEN_32PCLK,
    WDT_PLEN_64PCLK,
    WDT_PLEN_128PCLK,
    WDT_PLEN_256PCLK,
    WDT_PLEN_MAX
}eWDT_PULSELEN;

typedef enum _WDT_RESMODE
{
    WDT_MODE_RST,
    WDT_MODE_INT,
    WDT_MODE_MAX
}eWDT_RESMODE;


typedef enum _WDT_TIMEOUT
{
    WDT_TO_0000FFFF,
    WDT_TO_0001FFFF,
    WDT_TO_0003FFFF,
    WDT_TO_0007FFFF,
    WDT_TO_000FFFFF,
    WDT_TO_001FFFFF,
    WDT_TO_003FFFFF,
    WDT_TO_007FFFFF,
    WDT_TO_00FFFFFF,
    WDT_TO_01FFFFFF,
    WDT_TO_03FFFFFF,
    WDT_TO_07FFFFFF,
    WDT_TO_0FFFFFFF,
    WDT_TO_1FFFFFFF,
    WDT_TO_3FFFFFFF,
    WDT_TO_7FFFFFFF,
    WDT_TO_MAX
}eWDT_TIMEOUT;
///********************************************************************
//**                          ....                                *
//********************************************************************/
//extern void WDTEnable(void);
//extern void WDTDisable(void);
//extern void WDTIsr(void);
//extern uint32 WDTIntStatus(void);
//
///********************************************************************
//**                          ....                                *
//********************************************************************/
//
//#endif
#endif
