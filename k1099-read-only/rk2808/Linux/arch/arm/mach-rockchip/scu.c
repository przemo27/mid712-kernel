/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	scu.c
Desc 	:
Author 	:  	yangkai
Date 	:	2008-12-16
Notes 	:
$Log: scu.c,v $
Revision 1.7  2009/03/13 01:44:43  hcy
卡检测和卡电源管理改成GPIO方式

Revision 1.6  2009/03/12 03:44:05  yk
SCU,PLL Update

Revision 1.5  2009/03/12 02:13:49  yk
修改SCU程序bug，在2800SDK板上可运行

Revision 1.4  2009/03/11 03:33:38  hcy
驱动调整

Revision 1.3  2009/03/07 07:30:18  yk
(yk)更新SCU模块各频率设置，完成所有函数及代码，更新初始化设置，
更新遥控器代码，删除FPGA_BOARD宏。
(hcy)SDRAM驱动改成28的


********************************************************************/

#define IN_SCU
#if 0
#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_pll.h>
#include <asm/arch/api_scu.h>
#include <asm/arch/scu.h>
#include <asm/arch/drivers_delay.h>
#include <asm/arch/api_pmu.h>
#include <asm/arch/iomux.h>
#endif

#if 1

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/hardware.h>

#endif
#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_pll.h>
#include <asm/arch/api_scu.h>
#include <asm/arch/scu.h>
#include <asm/arch/drivers_delay.h>
#include <asm/arch/api_pmu.h>
#include <asm/arch/iomux.h>
#include <asm/arch/clock.h>
#include <asm/arch/rk28_macro.h>

#define PLL_KF_MASK				0xfff0
#define PLL_OD_MASK				0x0E
#define PLL_KR_MASK				0x3F0000
#define SCU_CLK_HDIV_MASK		0x03
#define SCU_CLK_PDIV_MASK		0x0C

extern void SCUSetPll_temp(eSCU_PLL_ID pllId, uint32 nMHz);
/*----------------------------------------------------------------------
Name	: SCUSelSDClk
Desc	: 设置hclk到SDMMC模块时钟分频值
Params  : sdmmcId:sdmmc控制端口号 0~1
          div: 分频值取1~8
Return  :
Notes   : 分频值必须在1~8范围内
----------------------------------------------------------------------*/
int32 SCUSelSDClk(uint32 sdmmcId, uint32 div)
{
    uint32 value;
#if(BOARDTYPE ==RK2800_FPGA)
    return(0);
#endif

    if((div == 0)||(sdmmcId > 1))
    {
        return (-1);
    }
    value = g_scuReg->SCU_CLKSEL0_CON;
    value &= ~(CLK_SDMMC0_MASK<<(sdmmcId*21));//CLK_SDMMC1_MASK == CLK_SDMMC0_MASK<<21
    value |= (CLK_SDMMC0_DIV(div)<<(sdmmcId*21));
    g_scuReg->SCU_CLKSEL0_CON = value;
    return(0);
}

/*----------------------------------------------------------------------
Name	: SCUSelVIPClk
Desc	: 设置VIP模块时钟
Params  : 
Return  : 0:设置成功;1:设置失败
Notes   : 由于系统无法提供准确的48MHz时钟，当要求48MHz时钟，系统将提供
        50MHz的时钟给VIP，如果无法提供50MHz时钟，返回失败
----------------------------------------------------------------------*/
int32 SCUSelVIPClk(uint32 nMHz)
{
    uint32 freq;
#if(BOARDTYPE ==RK2800_FPGA)
        return(0);
#endif
    if(24 == nMHz)
    {
        g_scuReg->SCU_CLKSEL0_CON &= ~CLK_SENSOR_MASK;//CLK_SENSOR_24M
        g_scuReg->SCU_CLKSEL0_CON |= CLK_SENSOR_24M;
    }
    else if(48 == nMHz)//
    {
        freq = PLLGetArmFreq();
        if((freq % 50) == 0)
        {
            g_scuReg->SCU_CLKSEL0_CON &= ~CLK_48M_MASK;
            g_scuReg->SCU_CLKSEL0_CON |= CLK_48M_DIV(freq/50);
            g_scuReg->SCU_CLKSEL0_CON &= ~CLK_SENSOR_MASK;
            g_scuReg->SCU_CLKSEL0_CON |= CLK_SENSOR_48M;
        }
        else
        {
            return(-1);
        }
    }
    return(0);
}

/*----------------------------------------------------------------------
Name	: SCUSelLSADCClk
Desc	: 设置pclk到LSADC模块时钟分频值
Params  : div: 分频值取1~128
Return  :
Notes   : 分频值必须在1~128范围内
----------------------------------------------------------------------*/
int32 SCUSelLSADCClk(uint32 div)
{
#if(BOARDTYPE ==RK2800_FPGA)
        return(0);
#endif
    if(div == 0)
    {
        return (-1);
    }
    g_scuReg->SCU_CLKSEL1_CON &= ~CLK_LSADC_MASK;
    g_scuReg->SCU_CLKSEL1_CON |= (CLK_LSADC_DIV(div));
    return(0);
}

/*----------------------------------------------------------------------
Name	: SCUClrInt
Desc	: 清SCU中断
Params  :
Return  :
Notes   :
----------------------------------------------------------------------*/
void SCUClrInt(void)
{
    g_scuReg->SCU_MODE_CON |= SCU_INT_CLR;
}

/*----------------------------------------------------------------------
Name	: SCUStopMode
Desc	: CPU STOP模式
Params  :
Return  :
Notes   : 该模式CPU停止运行，处于等待中断状态
----------------------------------------------------------------------*/
void SCUStopMode(void)
{
}

/*-------------------------------------------------------------------
Name	: SCURstModule/SCUUnrstModule
Desc	: 复位IP
Params  : moduleId:IP模块ID
Return  :
Notes   : 这两个函数需要成对使用
-------------------------------------------------------------------*/
void SCURstModule(eSCU_RST moduleId)
{
#if(BOARDTYPE ==RK2800_FPGA)
        return;
#endif
    g_scuReg->SCU_SOFTRST_CON |= (0x01<<moduleId);
}

/*-------------------------------------------------------------------
Name	: SCURstModule/SCUUnrstModule
Desc	: 复位IP
Params  : moduleId:IP模块ID
Return  :
Notes   : 这两个函数需要成对使用
-------------------------------------------------------------------*/
void SCUUnrstModule(eSCU_RST moduleId)
{
#if(BOARDTYPE ==RK2800_FPGA)
        return;
#endif
    g_scuReg->SCU_SOFTRST_CON &= ~(0x01<<moduleId);
}

/*-------------------------------------------------------------------
Name	: SCUEnableClk
Desc	: 开启IP时钟
Params  :
Return  :
Notes   : 开启IP时钟并更新IP时钟状态
-------------------------------------------------------------------*/
void SCUEnableClk(eCLK_GATE clkId)
{
    volatile uint32 *pClkgateAddr;
#if(BOARDTYPE ==RK2800_FPGA)
        return;
#endif
    pClkgateAddr = &(g_scuReg->SCU_CLKGATE0_CON);
    if(clkId>>5)
    {
        pClkgateAddr +=1;
    }
    *pClkgateAddr &= ~(0x01<<(clkId&0x1f));
    g_moduleClkList[(clkId>>5)] = *pClkgateAddr;
}

/*-------------------------------------------------------------------
Name	: SCUDisableClk
Desc	: 关闭IP时钟
Params  :
Return  :
Notes   : 关闭IP时钟并更新IP时钟状态
-------------------------------------------------------------------*/
void SCUDisableClk(eCLK_GATE clkId)
{
    volatile uint32 *pClkgateAddr;
#if(BOARDTYPE ==RK2800_FPGA)
        return;
#endif
    pClkgateAddr = &(g_scuReg->SCU_CLKGATE0_CON);
    if(clkId>>5)
    {
        pClkgateAddr +=1;
    }
    *pClkgateAddr |= (0x01<<(clkId&0x1f));
    g_moduleClkList[(clkId>>5)] = *pClkgateAddr;
}

/*-------------------------------------------------------------------
Name	: SCUPwrOnDomain
Desc	: 打开电源域电源
Params  :
Return  :
Notes   : Power Domain 1 -- DSP System
          Power Domain 2 -- CPU System
          Power Domain 3 -- Demodulator
          Power Domain 4 -- Share memory
-------------------------------------------------------------------*/
void SCUPwrOnDomain(eSCU_PD domainId)
{
#if(BOARDTYPE ==RK2800_FPGA)
        return;
#endif
    g_scuReg->SCU_PMU_CON &= ~(0x20<<domainId);
}

/*-------------------------------------------------------------------
Name	: SCUPwrOffDomain
Desc	: 关闭电源域电源
Params  :
Return  :
Notes   : Power Domain 1 -- DSP System
          Power Domain 2 -- CPU System
          Power Domain 3 -- Demodulator
          Power Domain 4 -- Share memory
-------------------------------------------------------------------*/
void SCUPwrOffDomain(eSCU_PD domainId)
{
#if(BOARDTYPE == RK2800_FPGA)
        return;
#endif
    g_scuReg->SCU_PMU_CON |= (0x20<<domainId);
}

/*----------------------------------------------------------------------
Name	: SCUDefaultSet
Desc	: SCU默认设置
Params  :
Return  :
Notes   :
----------------------------------------------------------------------*/
void SCUDefaultSet(void)
{
    g_scuReg->SCU_MODE_CON = SCU_INT_CLR|SCU_WAKEUP_POS|SCU_ALARM_WAKEUP_DIS\
                             |SCU_EXT_WAKEUP_DIS|SCU_CPUMODE_NORMAL|SCU_DSPMODE_NORMAL;
    g_scuReg->SCU_PMU_CON = PMU_SHMEM_PD | PMU_DEMOD_PD | PMU_DSP_PD;
    g_APPList = 0;
    //clock gate初始设置，0x00为打开，0x01为关闭
    g_moduleClkList[0] = ((0x01u<<CLK_GATE_SDMMC1)
                         |(0x01u<<CLK_GATE_SHMEM1)  | (0x01u<<CLK_GATE_SHMEM0)
                         |(0x01u<<CLK_GATE_LSADC)   | (0X00u<<CLK_GATE_RTC)//
                         |(0x01u<<CLK_GATE_WDT)     | (0x00u<<CLK_GATE_PWM)
                         |(0x01u<<CLK_GATE_SPI1)    | (0x01u<<CLK_GATE_SPI0)
                         |(0x01u<<CLK_GATE_I2C1)    | (0x01u<<CLK_GATE_I2C0)
                         |(0x01u<<CLK_GATE_UART1)   
                         |(0x00u<<CLK_GATE_UART0)//串口调试输出
                         |(0x00u<<CLK_GATE_GPIO1)   | (0x00u<<CLK_GATE_GPIO0)
                         |(0x00u<<CLK_GATE_SDMMC0)  | (0x01u<<CLK_GATE_I2S)
                         |(0x01u<<CLK_GATE_VIP)     | (0x01u<<CLK_GATE_DEBLK)
                         |(0x01u<<CLK_GATE_HIF)     | (0x01u<<CLK_GATE_SRAMDSP)
                         |(0x01u<<CLK_GATE_SRAMARM) | (0x01u<<CLK_GATE_DMA)
                         |(0x01u<<CLK_GATE_DSP));
    
    g_moduleClkList[1] = ((0x01u<<(CLK_GATE_HSADC&0x1f))  | (0x01u<<(CLK_GATE_DEMODFIFO&0x1f))
                         |(0x01u<<(CLK_GATE_DEMODBUS&0x1f))| (0x01u<<(CLK_GATE_DEMODOTHER&0x1f))
                         |(0x01u<<(CLK_GATE_AGC&0x1f))     | (0x01u<<(CLK_GATE_DOWNMIXER&0x1f))
                         |(0x01u<<(CLK_GATE_PREFFT&0x1f))  | (0x01u<<(CLK_GATE_IQIMBALANCE&0x1f))
                         |(0x01u<<(CLK_GATE_FRAMEDET&0x1f))| (0x01u<<(CLK_GATE_FFTMEM&0x1f))
                         |(0x01u<<(CLK_GATE_BITDITL&0x1f)) | (0x01u<<(CLK_GATE_VITERBIMEM&0x1f))
                         |(0x01u<<(CLK_GATE_PREFFTMEM&0x1f))|(0x01u<<(CLK_GATE_VITERBI&0x1f))
                         |(0x01u<<(CLK_GATE_RS&0x1f))       | (0x00u<<(CLK_GATE_EXTMEM&0x1f))
                         |(0x00u<<(CLK_GATE_MSDRMEM&0x1f))  | (0x01u<<(CLK_GATE_DEMOD&0x1f)))
                         |(0x00u<<(CLK_GATE_LCDCh&0x1f));

    g_moduleClkList[2] = ((0x01u<<(CLK_GATE_DSPBUS&0x1f)) | (0x01u<<(CLK_GATE_EFUSE&0x1f)));
    
    g_scuReg->SCU_CLKGATE0_CON = g_moduleClkList[0];
    g_scuReg->SCU_CLKGATE1_CON = g_moduleClkList[1];
    g_scuReg->SCU_CLKGATE2_CON = g_moduleClkList[2];
    
    g_scuReg->SCU_CLKSEL0_CON |= (CLK_SDMMC1_DIV(4)       //sdmmc1 divider div (4)
                                | CLK_SENSOR_24M        //sensor clock select 24MHz
                                | CLK_48M_DIV(4)        //48MHz divider div (4)
                                | CLK_USBPHY_24M        //USB PHY clock select 24MHz
                                | CLK_LCDC_CODPLL       //lcdc clock divide from codecpll
                                | CLK_LCDC_DIV(8)       //lcdc divider div (8)
                                | CLK_LCDC_DIVOUT       //lcdc clock from divider out
                                | CLK_SDMMC0_DIV(4));     //sdmmc0 divder div (4)
                                //| CLK_ARM_HCLK_21       //arm clk:hclk = 2:1
                                //| CLK_HCLK_PCLK_21;     //hclk:pclk = 2:1
    g_scuReg->SCU_CLKSEL1_CON = CLK_SHMEM1_DEMODCLK     //shmem1 clock from demod_clock
                                | CLK_SHMEM0_DEMODCLK   //shmem0 clock from demod_clock
                                | CLK_HSADCO_NORMAL     //hsadc clock output demod_clock/2
                                | CLK_GPS_DEMODCLK      //hsadc clock not from gps tuner input
                                | CLK_DEMOD_INTCLK      //demod_clk from internal divider out
                                | CLK_DEMOD_DSPPLL      //demod_clk divide from dsppll 
                                | CLK_DEMOD_DIV(4)      //demod_clk divider div (4)
                                | CLK_LSADC_DIV(128)      //lsadc_clk divider div (128)
                                | CLK_CODEC_DIV(2)      //codec_clk divider div (2)
                                | CLK_CODEC_12M         //codec_Clk from 12MHz osc input
                                | CLK_CPLL_SLOW;        //codecpll work slow mode
    
}

#if(BOARDTYPE == RK2800_FPGA)
//只能设置25MHz跟50MHz两种模式
//其他设置值按照25M处理
void SCUInit(uint32 nMHz)
{
    if(nMHz == 50)
    {
        SetArmPll(2, 1);//50M clk
    //    DisableTestBLK();
        g_chipClk.armFreq =50;
        g_chipClk.ahbDiv= HCLK_DIV1;
        g_chipClk.apbDiv = PCLK_DIV1;
        g_chipClk.dspFreq = 0;
        g_chipClk.AuxFreq = 0;
    }
    else//if(nMHz == 25)
    {
        SetArmPll(4, 1);//25M clk
    //    DisableTestBLK();
        g_chipClk.armFreq = 25;
        g_chipClk.ahbDiv= HCLK_DIV1;
        g_chipClk.apbDiv = PCLK_DIV1;
        g_chipClk.dspFreq = 0;
        g_chipClk.AuxFreq = 0;
    }
}
#else
/*----------------------------------------------------------------------
Name	: SCUInit
Desc	: SCU初始化
Params  :
Return  :
Notes   : 初始化本模块使用的全局变量并进行默认设置
----------------------------------------------------------------------*/
void SCUInit(void)
{
    //g_scuReg = (pSCU_REG)SCU_BASE_ADDR;
    SCUDefaultSet();
    g_chipClk.armFreq = 0;
    g_chipClk.AuxFreq = 0;
    g_chipClk.dspFreq = 0;
    g_chipClk.ahbDiv = HCLK_DIV2;//BOOTROM将该参数设置成2:1
    g_APPList |= ((uint64)0x01<< PMU_IDLE);
    PMUStartAPP(PMU_INIT);
}

#endif

unsigned int rk28_get_pll(eSCU_PLL_ID pll_id)
{
	unsigned int freq , pll_nr,pll_nf,pll_od;
    unsigned int pll_lock = (0x40u << pll_id);
    volatile unsigned int *pll_reg;
	pSCU_REG scu_reg = (pSCU_REG)SCU_BASE_ADDR_VA;

    switch (pll_id)
    {
        case PLL_ARM:
            pll_reg = &(scu_reg->SCU_APLL_CON);
            break;
        case PLL_DSP:
            pll_reg = &(scu_reg->SCU_DPLL_CON);
            break;
        case PLL_CODEC:
            pll_reg = &(scu_reg->SCU_CPLL_CON);
            break;
        default:
            break;
    }
	SCU_GET_REG_VAL(pll_reg,pll_nr,PLL_KR_MASK);
   	SCU_GET_REG_VAL(pll_reg,pll_nf,PLL_KF_MASK);
	SCU_GET_REG_VAL(pll_reg,pll_od,PLL_OD_MASK);

	pll_nr += 1;
	pll_nf += 1;
	pll_od += 1;

	freq = 24 * pll_nf / pll_nr / pll_od;
	freq = freq * 1000000;
#if ROCK_DEBUG	
	printk("pll_reg_nr = %d \n",pll_nr);
	
	printk("pll_reg_nf = %d \n",pll_nf);
	
	printk("pll_reg_od = %d \n",pll_od);

	printk("current frequence == %d \n",freq);
#endif	
	return freq;

}
unsigned int rk28_scu_get_busdiv(int *hdiv,int *pdiv)
{
	pSCU_REG scu_reg = (pSCU_REG)SCU_BASE_ADDR_VA;
	*hdiv = (scu_reg->SCU_CLKSEL0_CON);
	*pdiv = (scu_reg->SCU_CLKSEL0_CON);
	//SCU_GET_REG_VAL( &(scu_reg->SCU_CLKSEL0_CON) , *hdiv, CLK_HCLK_MASK);
	//SCU_GET_REG_VAL(&(scu_reg->SCU_CLKSEL0_CON), *pdiv, CLK_PCLK_MASK);
	switch (*hdiv&CLK_HCLK_MASK) {
		case CLK_ARM_HCLK_11:
			*hdiv = 1;
			break;
	
		case CLK_ARM_HCLK_21:
			*hdiv = 2;
			break;
	
		case CLK_ARM_HCLK_31:
			*hdiv = 3;
			break;
	
		case CLK_ARM_HCLK_41:
			*hdiv = 4;
			break;
	}

	switch (*pdiv&CLK_PCLK_MASK) {
		case CLK_HCLK_PCLK_11:
			*pdiv = 1;
			break;

		case CLK_HCLK_PCLK_21:
			*pdiv = 2;
			break;

		case CLK_HCLK_PCLK_41:
			*pdiv = 4;
			break;
	}
}
int rk28_scu_set_pll(struct clk *c)
{
	eSCU_PLL_ID pll_id = 0;
	unsigned int freq ;
	volatile unsigned int *pll_reg;
	if(strcmp(c->name,"arm_clk") == 0)
		pll_id = PLL_ARM;
	else if(strcmp(c->name,"dsp_clk") == 0)
		pll_id = PLL_DSP;
	else if(strcmp(c->name,"codec_clk") == 0)
		pll_id = PLL_CODEC;
    	SCUSetPll_temp(pll_id,c->rate/1000000);
    	freq = rk28_get_pll(PLL_ARM);
    	c->rate = freq;
#if ROCK_DEBUG	
    	printk("---------------new arm pll = %d --------------\n",freq);
#endif
	return 0;
}
