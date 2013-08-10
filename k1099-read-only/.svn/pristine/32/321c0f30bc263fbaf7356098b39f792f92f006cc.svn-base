/******************************************************************/
/*   Copyright (C) 2008 ROCK-CHIPS FUZHOU . All Rights Reserved.  */
/*******************************************************************
File    :  uart.h
Desc    :  定义UART的寄存器结构体\寄存器位的宏定义\接口函数

Author  : lhh
Date    : 2008-10-29
Modified:
Revision:           1.00
$Log:   uart.h,v $
*********************************************************************/

#ifndef _DRIVER_UART_H_
#define _DRIVER_UART_H_
#include <asm/arch/hardware.h>

#define     SCU_BASE_ADDR           0x18018000
#define     UART0_BASE_ADDR         0x18000000
#define     UART1_BASE_ADDR         0x18002000
#define     REG_FILE_BASE_ADDR      0x18019000

#ifndef __ASSEMBLY__
typedef volatile struct tagGRF_REG
{
    unsigned int  CPU_APB_REG0;
    unsigned int  CPU_APB_REG1;
    unsigned int  CPU_APB_REG2;
    unsigned int  CPU_APB_REG3;
    unsigned int  CPU_APB_REG4;
    unsigned int  CPU_APB_REG5;
    unsigned int  CPU_APB_REG6;
    unsigned int  CPU_APB_REG7;
    unsigned int  IOMUX_A_CON;
    unsigned int  IOMUX_B_CON;
    unsigned int  GPIO0_AB_PU_CON;
    unsigned int  GPIO0_CD_PU_CON;
    unsigned int  GPIO1_AB_PU_CON;
    unsigned int  GPIO1_CD_PU_CON;
    unsigned int  OTGPHY_CON0;
    unsigned int  OTGPHY_CON1;
}GRF_REG, *pGRF_REG,*pAPB_REG;

typedef volatile struct tagSCU_REG
{
    unsigned int SCU_APLL_CON;//[3];//0:arm 1:dsp 2:codec
    unsigned int SCU_DPLL_CON;
    unsigned int SCU_CPLL_CON;
    unsigned int SCU_MODE_CON;
    unsigned int SCU_PMU_CON;
    unsigned int SCU_CLKSEL0_CON;
    unsigned int SCU_CLKSEL1_CON;
    unsigned int SCU_CLKGATE0_CON;
    unsigned int SCU_CLKGATE1_CON;
    unsigned int SCU_CLKGATE2_CON;
    unsigned int SCU_SOFTRST_CON;
    unsigned int SCU_CHIPCFG_CON;
    unsigned int SCU_CPUPD;
}SCU_REG,*pSCU_REG;

typedef volatile struct tagUART_STRUCT
{
    unsigned int UART_RBR;
    unsigned int UART_DLH;
    unsigned int UART_IIR;
    unsigned int UART_LCR;
    unsigned int UART_MCR;
    unsigned int UART_LSR;
    unsigned int UART_MSR;
    unsigned int UART_SCR;
    unsigned int RESERVED1[(0x30-0x20)/4];
    unsigned int UART_SRBR[(0x70-0x30)/4];
    unsigned int UART_FAR;
    unsigned int UART_TFR;
    unsigned int UART_RFW;
    unsigned int UART_USR;
    unsigned int UART_TFL;
    unsigned int UART_RFL;
    unsigned int UART_SRR;
    unsigned int UART_SRTS;
    unsigned int UART_SBCR;
    unsigned int UART_SDMAM;
    unsigned int UART_SFE;
    unsigned int UART_SRT;
    unsigned int UART_STET;
    unsigned int UART_HTX;
    unsigned int UART_DMASA;
    unsigned int RESERVED2[(0xf4-0xac)/4];
    unsigned int UART_CPR;
    unsigned int UART_UCV;
    unsigned int UART_CTR;
} UART_REG, *pUART_REG;

#define UART_IER UART_DLH
#define UART_DLL UART_RBR
#define UART_THR UART_RBR

#endif /* asm */

#endif
