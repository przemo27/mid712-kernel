/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	scu.h
Desc 	:	
Author 	:  	yangkai
Date 	:	2008-12-16
Notes 	:
$Log    :   scu.h,v $
********************************************************************/
#ifndef _SCU_H
#define _SCU_H
#include "api_scu.h"
#include <asm/arch/clock.h>


/********************************************************************
**                            宏定义                                *
********************************************************************/
/*SCU PLL CON*/
#define PLL_TEST        (0x01u<<25)
#define PLL_SAT         (0x01u<<24)
#define PLL_FAST        (0x01u<<23)
#define PLL_PD          (0x01u<<22)
#define PLL_CLKR(i)     (((i)&0x3f)<<16)
#define PLL_CLKF(i)     (((i)&0x0fff)<<4)
#define PLL_CLKOD(i)    (((i)&0x07)<<1)
#define PLL_BYPASS      (0X01)

/*SCU MODE CON*/
#define SCU_INT_CLR         (0x01u<<8)
#define SCU_WAKEUP_POS      (0x00u<<7)
#define SCU_WAKEUP_NEG      (0x01u<<7)
#define SCU_ALARM_WAKEUP_DIS (0x01u<<6)
#define SCU_EXT_WAKEUP_DIS   (0x01u<<5)
#define SCU_STOPMODE_EN     (0x01u<<4)

#define SCU_CPUMODE_MASK    (0x11u<<2)
#define SCU_CPUMODE_SLOW    (0x00u<<2)
#define SCU_CPUMODE_NORMAL  (0x01u<<2)
#define SCU_CPUMODE_DSLOW   (0x02u<<2)

#define SCU_DSPMODE_MASK    (0x11u<<0)
#define SCU_DSPMODE_SLOW    (0x00u<<0)
#define SCU_DSPMODE_NORMAL  (0x01u<<0)
#define SCU_DSPMODE_DSLOW   (0x02u<<0)

/*SCU PMU MODE*/
#define PMU_SHMEM_PWR_STAT  (0x01u<<8)
#define PMU_DEMOD_PWR_STAT  (0x01u<<7)
#define PMU_CPU_PWR_STAT    (0x01u<<6)
#define PMU_DSP_PWR_STAT    (0x01u<<5)

#define PMU_EXT_SWITCH_PWR  (0x01u<<4)

#define PMU_SHMEM_PD        (0x01u<<3)
#define PMU_DEMOD_PD        (0x01u<<2)
#define PMU_CPU_PD          (0x01u<<1)
#define PMU_DSP_PD          (0x01u<<0)

/*SCU SOFTWARE RESET CON*/
#define CLK_RST_SDRAM       (1<<28)
#define CLK_RST_SHMEM1      (1<<27)
#define CLK_RST_SHMEM0      (1<<26)
#define CLK_RST_DSPA2A      (1<<25)
#define CLK_RST_SDMMC1      (1<<24)
#define CLK_RST_ARM         (1<<23)
#define CLK_RST_DEMODGEN    (1<<22)
#define CLK_RST_PREFFT      (1<<21)
#define CLK_RST_RS          (1<<20)
#define CLK_RST_BITDITL     (1<<19)
#define CLK_RST_VITERBI     (1<<18)
#define CLK_RST_FFT         (1<<17)
#define CLK_RST_FRAMEDET    (1<<16)
#define CLK_RST_IQIMBALANCE (1<<15)
#define CLK_RST_DOWNMIXER   (1<<14)
#define CLK_RST_AGC         (1<<13)
#define CLK_RST_USBPHY      (1<<12)
#define CLK_RST_USBC        (1<<11)
#define CLK_RST_DEMOD       (1<<10)
#define CLK_RST_SDMMC0      (1<<9)
#define CLK_RST_DEBLK       (1<<8)
#define CLK_RST_LSADC       (1<<7)
#define CLK_RST_I2S         (1<<6)
#define CLK_RST_DSPPER      (1<<5)
#define CLK_RST_DSP         (1<<4)
#define CLK_RST_NANDC       (1<<3)
#define CLK_RST_VIP         (1<<2)
#define CLK_RST_LCDC        (1<<1)
#define CLK_RST_USBOTG      (1<<0)

/*SCU CLK SEL0 CON*/
#define CLK_SDMMC1_SHFT     25
#define CLK_SDMMC1_MASK     (0x07u<<25)
#define CLK_SDMMC1_DIV(i)   (((i-1)&0x07u)<<25)

#define CLK_SENSOR_SHFT     23
#define CLK_SENSOR_MASK     (0x03u<<23)
#define CLK_SENSOR_24M      (0x00u<<23)
#define CLK_SENSOR_27M      (0x01u<<23)
#define CLK_SENSOR_48M      (0x02u<<23)

#define CLK_48M_SHFT     20
#define CLK_48M_MASK        (0x07u<<20)
#define CLK_48M_DIV(i)      (((i-1)&0x07u)<<20)


#define CLK_USBPHY_SHFT     18
#define CLK_USBPHY_MASK     (0x03u<<18)
#define CLK_USBPHY_24M      (0x00u<<18)
#define CLK_USBPHY_12M      (0x01u<<18)
#define CLK_USBPHY_48M      (0x01u<<18)

#define CLK_LCDC_ARMPLL     (0x00u<<16)//
#define CLK_LCDC_DSPPLL     (0x01u<<16)//
#define CLK_LCDC_CODPLL     (0x02u<<16)//

#define CLK_LCDC_SHFT     8
#define CLK_LCDC_MASK       (0x0ffu<<8)
#define CLK_LCDC_DIV(i)     (((i-1)&0xffu)<<8)

#define CLK_LCDC_DIVOUT     (0x00<<7)//
#define CLK_LCDC_27M        (0X01<<7)//

#define CLK_SDMMC0_SHFT     4
#define CLK_SDMMC0_MASK     (0x07u<<4)
#define CLK_SDMMC0_DIV(i)   (((i-1)&0x07u)<<4)

#define CLK_PCLK_SHFT     2
#define CLK_PCLK_MASK       (0x03u<<2)
#define CLK_HCLK_PCLK_11    (0x00u<<2)
#define CLK_HCLK_PCLK_21    (0x01u<<2)
#define CLK_HCLK_PCLK_41    (0x02u<<2)

#define CLK_HCLK_SHFT     0
#define CLK_HCLK_MASK       (0x03u<<0)
#define CLK_ARM_HCLK_11     (0x00u<<0)
#define CLK_ARM_HCLK_21     (0x01u<<0)
#define CLK_ARM_HCLK_31     (0x02u<<0)
#define CLK_ARM_HCLK_41     (0x03u<<0)

/*SCU CLK SEL1 CON*/
#define CLK_SHMEM1_SHFT     30
#define CLK_SHMEM1_MASK     (0x01u<<30)
#define CLK_SHMEM1_DEMODCLK (0x00u<<30)
#define CLK_SHMEM1_ARMCLK   (0x01u<<30)

#define CLK_SHMEM0_SHFT     29
#define CLK_SHMEM0_MASK     (0x01u<<29)
#define CLK_SHMEM0_DEMODCLK (0x00u<<29)
#define CLK_SHMEM0_ARMCLK   (0x01u<<29)

#define CLK_HSADCO_SHFT     28
#define CLK_HSADCO_MASK      (0x01u<<28)
#define CLK_HSADCO_NORMAL    (0x00u<<28)
#define CLK_HSADCO_INVERT    (0x01u<<28)

#define CLK_GPS_SHFT     27
#define CLK_GPS_MASK        (0x01u<<27)
#define CLK_GPS_DEMODCLK    (0x00u<<27)
#define CLK_GPS_TUNER_INPUT (0x01u<<27)

#define CLK_DEMOD_INTCLK    (0x00u<<26)//
#define CLK_DEMOD_EXTCLK    (0x01u<<26)//

#define CLK_DEMOD_ARMPLL    (0x00u<<24)//
#define CLK_DEMOD_DSPPLL    (0x01u<<24)//
#define CLK_DEMOD_CODPLL    (0x02u<<24)//

#define CLK_DEMOD_SHFT     16
#define CLK_DEMOD_MASK      (0x0ffu<<16)
#define CLK_DEMOD_DIV(i)    (((i-1)&0x0ffu)<<16)

#define CLK_LSADC_SHFT     8
#define CLK_LSADC_MASK      (0x0ffu<<8)
#define CLK_LSADC_DIV(i)    (((i-1)&0x0ffu)<<8)

#define CLK_CODEC_SHFT     3
#define CLK_CODEC_MASK      (0x1fu<<3)
#define CLK_CODEC_DIV(i)    (((i-1)&0x1fu)<<3)

#define CLK_CODEC_CPLLCLK   (0x00u<<2)//
#define CLK_CODEC_12M       (0x01u<<2)//

#define CLK_CPLL_SLOW       (0x00u<<0)//
#define CLK_CPLL_NORMAL     (0x01u<<0)//
#define CLK_CPLL_DSLOW      (0x02u<<0)//

#define CLK_ENABLE			0
#define CLK_DISABLE			1

/************************************************************
** chip clock define                                        *
************************************************************/
#define DEF_ARMAHB_DIV  CLK_ARM_HCLK_21
#define DEF_DSPAHB_DIV  
#define DEF_ARMPLL_VCO 300

#define FREQ_ARM_MAX    300   
#define FREQ_ARM_MIN    24
#define FREQ_ARM_IDLE   24
#define FREQ_HCLK_MAX   FREQ_ARM_MAX/2
#define FREQ_HCLK_MIN   FREQ_ARM_MIN/2
#define FREQ_PCLK_MAX   FREQ_HCLK_MAX/2
#define FREQ_PCLK_MIN   FREQ_HCLK_MIN/2

#define FREQ_DSP_MAX    360
#define FREQ_DSP_MIN    24
#define FREQ_DSPAHB_MAX FREQ_DSP_MAX/2
#define FREQ_DSPAHB_MIN    12

/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef volatile struct tagSCU_REG
{
    uint32 SCU_APLL_CON;//[3];//0:arm 1:dsp 2:codec
    uint32 SCU_DPLL_CON;
    uint32 SCU_CPLL_CON;
    uint32 SCU_MODE_CON;
    uint32 SCU_PMU_CON;
    uint32 SCU_CLKSEL0_CON;
    uint32 SCU_CLKSEL1_CON;
    uint32 SCU_CLKGATE0_CON;
    uint32 SCU_CLKGATE1_CON;
    uint32 SCU_CLKGATE2_CON;
    uint32 SCU_SOFTRST_CON;
    uint32 SCU_CHIPCFG_CON;
    uint32 SCU_CPUPD;
}SCU_REG,*pSCU_REG;

typedef volatile struct tagCPMU_REG
{
    uint32 reserved1;
    uint32 reserved2;
    uint32 CXCLK_DIV;
    uint32 XHCLK_DIV;
    uint32 XPCLK_DIV;
    uint32 CXPMOD;
    uint32 XHPMOD;
    uint32 reserved3;
    uint32 XAPBMOD;
}CPMU_REG,*pCPMU_REG;

typedef struct tagSCU_CLK_INFO
{
    uint32 armFreq;     //ARM PLL FREQ
    uint32 dspFreq;     //DSP PLL FREQ
    uint32 AuxFreq;   //AUX PLL FREQ
    uint32 ahbDiv;
    uint32 apbDiv;
    uint32 armFreqLast;
}SCU_CLK_INFO,*pSCU_CLK_INFO;

typedef enum _CLK_GATE2
{
    CLK_GATE_ARMIBUS = 0,
    CLK_GATE_ARMDBUS,
    CLK_GATE_DSPBUS,
    CLK_GATE_EXPBUS,
    CLK_GATE_APBBUS,
    CLK_GATE_EFUSE,
    CLK_GATE_DTCM1,
    CLK_GATE_DTCM0,
    CLK_GATE_ITCM
}eCLK_GATE2;
typedef enum _SCU_PLL_ID
{
    PLL_ARM = 0,
    PLL_DSP = 1,
    PLL_CODEC = 2,
    PLL_MAX
}eSCU_PLL_ID;

#define SCU_GET_REG_VAL(reg_addr,reg_val,MASK)		\
	do{												\
		unsigned int mask_temp = MASK;										\
		if(MASK == 0){								\
			reg_val = *reg_addr;					\
			break;									\
		}											\
		reg_val = (*reg_addr)&MASK;			\
		while((mask_temp&0x01) == 0){			\
			mask_temp = mask_temp >> 1;			\
			reg_val = reg_val >> 1;				\
		}										\
	}while(0)		



/********************************************************************
**                          变量定义                                *
********************************************************************/
#undef EXT
#ifdef IN_SCU
    #define EXT
#else    
    #define EXT extern
#endif    
    
EXT SCU_CLK_INFO g_chipClk;

#define g_scuReg   ((pSCU_REG)(SCU_BASE_ADDR_VA)) //(IO_PA2VA_AHB(SCU_BASE_ADDR)))//((pSCU_REG)(IO_PA2VA_AHB(SCU_BASE_ADDR)))

#define g_cpmuReg ((pCPMU_REG)(DSP_BASE_ADDR+DSP_PMU_BASE_ADDR))
EXT uint64 g_APPList;
EXT uint32 g_moduleClkList[3];

//EXT struct clk;
/********************************************************************
**                          函数声明                                *
********************************************************************/
extern void SCUSetAUXPll(uint32 nMHz);
extern void SCUSetDSPPll(uint32 nMHz);
extern void SCUSetARMPll(uint32 div);
extern unsigned int rk28_scu_get_busdiv(int *hdiv,int *pdiv);
extern unsigned int rk28_get_pll(eSCU_PLL_ID pll_id);
extern int rk28_scu_set_pll(struct clk * clk_va );

#endif
