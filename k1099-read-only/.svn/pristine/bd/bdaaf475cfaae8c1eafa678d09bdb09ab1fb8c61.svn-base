/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	api_scu.h
Desc 	:	
Author 	:  	yangkai
Date 	:	2008-12-16
Notes 	:
$Log: api_scu.h,v $
********************************************************************/
#ifndef _API_SCU_H
#define _API_SCU_H
#include <asm/arch/typedef.h>
#include <asm/arch/hw_define.h>

/********************************************************************
**                            宏定义                                *
********************************************************************/

/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef enum _SCU_RST
{
    SCU_RST_USBOTG = 0,
    SCU_RST_LCDC,
    SCU_RST_VIP,
    SCU_RST_NANDC,
    SCU_RST_DSP,
    SCU_RST_DSPPER,
    SCU_RST_I2S,
    SCU_RST_LSADC,
    SCU_RST_DEBLK,
    SCU_RST_SDMMC0,
    SCU_RST_DEMOD,
    SCU_RST_USBC,
    SCU_RST_USBPHY,
    SCU_RST_AGC,
    SCU_RST_DOWNMIXER,
    SCU_RST_IQIMBALANCE,
    SCU_RST_FRAMEDET,
    SCU_RST_FFT,
    SCU_RST_VITERBI,
    SCU_RST_BITDITL,
    SCU_RST_RS,
    SCU_RST_PREFFT,
    SCU_RST_DEMODGEN,
    SCU_RST_ARM,
    SCU_RST_SDMMC1,
    SCU_RST_DSPA2A,
    SCU_RST_SHMEM0,
    SCU_RST_SHMEM1,
    SCU_RST_SDRAM,
    SCU_RST_MAX
}eSCU_RST;

typedef enum _CLK_GATE
{
    /*SCU CLK GATE 0 CON*/
    CLK_GATE_ARM = 0,
    CLK_GATE_DSP,
    CLK_GATE_DMA,
    CLK_GATE_SRAMARM,
    CLK_GATE_SRAMDSP,
    CLK_GATE_HIF,
    CLK_GATE_OTGBUS,
    CLK_GATE_OTGPHY,
    CLK_GATE_NANDC,
    CLK_GATE_INTC,
    CLK_GATE_DEBLK,
    CLK_GATE_LCDC,
    CLK_GATE_VIP,
    CLK_GATE_I2S,
    CLK_GATE_SDMMC0,
    CLK_GATE_EBROM,
    CLK_GATE_GPIO0,
    CLK_GATE_GPIO1,
    CLK_GATE_UART0,
    CLK_GATE_UART1,
    CLK_GATE_I2C0,
    CLK_GATE_I2C1,
    CLK_GATE_SPI0,
    CLK_GATE_SPI1,
    CLK_GATE_PWM,
    CLK_GATE_TIMER,
    CLK_GATE_WDT,
    CLK_GATE_RTC,
    CLK_GATE_LSADC,
    CLK_GATE_SHMEM0,
    CLK_GATE_SHMEM1,
    CLK_GATE_SDMMC1,
    
    /*SCU CLK GATE 1 CON*/
    CLK_GATE_HSADC = 32,
    CLK_GATE_DEMODFIFO,
    CLK_GATE_DEMODBUS,
    CLK_GATE_DEMODOTHER,
    CLK_GATE_AGC,
    CLK_GATE_DOWNMIXER,
    CLK_GATE_PREFFT,
    CLK_GATE_IQIMBALANCE,
    CLK_GATE_FRAMEDET,
    CLK_GATE_FFTMEM,
    CLK_GATE_BITDITL,
    CLK_GATE_VITERBIMEM,
    CLK_GATE_PREFFTMEM,
    CLK_GATE_VITERBI,
    CLK_GATE_RS,
    CLK_GATE_EXTMEM,
    CLK_GATE_SDRMEM,
    CLK_GATE_MSDRMEM,
    CLK_GATE_DEMOD,
    CLK_GATE_LCDCh,
    
    CLK_GATE_MAX
}eCLK_GATE;
typedef enum _SCU_PD
{
    SCU_PD1_DSP,
    SCU_PD2_CPU,
    SCU_PD3_DEMOD,
    SCU_PD4_SHMEM,
    //other power domain are always power on
    SCU_PD_MAX
}eSCU_PD;

/********************************************************************
**                          变量定义                                *
********************************************************************/

/********************************************************************
**                      对外函数接口声明                            *
********************************************************************/
#if(BOARDTYPE == RK2800_FPGA)
extern void SetArmPll(int ndiv, int bypass);
extern void DisableTestBLK(void);
//extern void SCUInit(uint32 nMHz);
#else
extern void SCUInit(void);
#endif

extern int32 SCUSelSDClk(uint32 sdmmcId, uint32 div);
extern int32 SCUSelVIPClk(uint32 nMHz);
extern int32 SCUSelLSADCClk(uint32 div);

extern void SCUEnableClk(eCLK_GATE clkId);
extern void SCUDisableClk(eCLK_GATE clkId);

extern void SCURstModule(eSCU_RST moduleId);
extern void SCUUnrstModule(eSCU_RST moduleId);

extern void SCUPwrOnDomain(eSCU_PD domainId);
extern void SCUPwrOffDomain(eSCU_PD domainId);

/********************************************************************
**                          表格定义                                *
********************************************************************/

#endif
