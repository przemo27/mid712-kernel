/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	pll.c
Desc 	:
Author 	:  	yangkai
Date 	:	2008-12-16
Notes 	:
$Log: pll.c,v $
Revision 1.5  2009/03/12 03:11:21  yk
更新PLL,AHB,APB初始化设置

Revision 1.4  2009/03/12 02:13:49  yk
修改SCU程序bug，在2800SDK板上可运行

Revision 1.3  2009/03/11 03:33:38  hcy
驱动调整

Revision 1.2  2009/03/07 07:30:18  yk
(yk)更新SCU模块各频率设置，完成所有函数及代码，更新初始化设置，
更新遥控器代码，删除FPGA_BOARD宏。
(hcy)SDRAM驱动改成28的


********************************************************************/
#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_pll.h>
#include <asm/arch/api_scu.h>
#include <asm/arch/scu.h>
#include <asm/arch/drivers_delay.h>
#include <asm/arch/api_pmu.h>
#include <asm/arch/iomux.h>

#include <asm/io.h>
#include <asm/arch/rk28_macro.h>
#include <linux/kernel.h>
#define APBREG0_APLL_LOCK_STAT              (1<<7)
#define g_grfReg		((pGRF_REG)(REG_FILE_BASE_ADDR_VA))


/*-------------------------------------------------------------------
Name	: PLLGetArmFreq
Desc	: 获取ARM频率
Params  :
Return  : ARM频率,  MHz 为单位
Notes   :
-------------------------------------------------------------------*/
uint32 PLLGetArmFreq(void)
{
    return((uint32)g_chipClk.armFreq);
}

/*-------------------------------------------------------------------
Name	: PLLGetDspFreq
Desc	: 获取DSP频率
Params  :
Return  : DSP频率,  MHz 为单位
Notes   :
-------------------------------------------------------------------*/
uint32 PLLGetDspFreq(void)
{
    return ((uint32)g_chipClk.dspFreq);
}

/*-------------------------------------------------------------------
Name	: SCUGetCodecFreq
Desc	: 获取aux PLL频率
Params  :
Return  : aux PLL频率,  MHz 为单位
Notes   :
-------------------------------------------------------------------*/
uint32 PLLGetAuxFreq(void)
{
    return ((uint32)g_chipClk.AuxFreq);
}

/*-------------------------------------------------------------------
Name	: PLLGetAHBFreq
Desc	: 获取AHB频率
Params  :
Return  : AHB频率,  kHz 为单位
Notes   :
-------------------------------------------------------------------*/
uint32 PLLGetAHBFreq(void)
{
    return ((g_chipClk.armFreq*1000)>>g_chipClk.ahbDiv);
}

/*-------------------------------------------------------------------
Name	: PLLGetAPBFreq
Desc	: 获取APB频率
Params  :
Return  : APB频率,  KHz 为单位
Notes   :
-------------------------------------------------------------------*/
uint32 PLLGetAPBFreq(void)
{
    return (PLLGetAHBFreq()>>g_chipClk.apbDiv);
}

/*----------------------------------------------------------------------
Name	: SCUPwrDwnPLL
Desc	: 关闭一个PLL
Params  : pllId:PLL编号
Return  :
Notes   :
----------------------------------------------------------------------*/
void SCUPwrDwnPLL(eSCU_PLL_ID pllId)
{
    switch (pllId)
    {
        case PLL_ARM:
            g_scuReg->SCU_APLL_CON |= PLL_PD;
            break;
        case PLL_DSP:
            g_scuReg->SCU_DPLL_CON |= PLL_PD;
            break;
        case PLL_CODEC:
            g_scuReg->SCU_CPLL_CON |= PLL_PD;
            break;
        default:
            break;
    }
}

/*----------------------------------------------------------------------
Name	: SCUSetPll
Desc	: 设置PLL
Params  : pllId:PLL编号
          freq:要设置的频率
Return  :
Notes   : PLL分频方式见下图，需要设置三个分频因子，并需要满足以下条件:
          1. Fref/NR value range requirement : 97.7KHz - 800MHz
          2. Fref/NR * NF value range requirement: 160MHz - 800MHz
           ,------------------------.
           |   ___     ___     ___  |  ___
           '->|/NF|-->| P |-->|VCO|-->|/OD|--->|\
              `==='   | F |   `---'   `---'    | \
           ,->|/NR|-->| D |                    |  \
           |  `---'   `---'                    |  |-->CLKOUT
           |                                   |  /
          Fref(24M)--------------------------->| /
                                               |/
----------------------------------------------------------------------*/
void SCUSetPll(eSCU_PLL_ID pllId, uint32 nMHz)
{
    uint32 count;
    uint32 pllLock = (0x40u << pllId);
	volatile uint32 *pllReg = NULL;

    switch (pllId)
    {
        case PLL_ARM:
            pllReg = &(g_scuReg->SCU_APLL_CON);
            break;
        case PLL_DSP:
            pllReg = &(g_scuReg->SCU_DPLL_CON);
            break;
        case PLL_CODEC:
            pllReg = &(g_scuReg->SCU_CPLL_CON);
            break;
        default:
            break;
    }

    if (*pllReg & PLL_PD)
    {
        *pllReg &= ~PLL_PD;
    }

    if (nMHz < 40 )			    //  25M ~ 39	vco= 200 ~ 320
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(3-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(8-1));
    }
    else if (nMHz < 80)			//  40 ~ 79,  vco = 160 ~ 320
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(4-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(6-1));
    }
    else if (nMHz < 160)		// 80 ~ 159,  vco = 160 ~ 320
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(12-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(2-1));
    }
    else                        // 160 ~ 300 , VCO = 160 ~ 300
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(24-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(1-1));
    }

    //等待PLL进入稳定状态，约0.3ms.
    DRVDelayCyc(5000);
    count = 2000;
    while (count --)
    {
        if (g_grfReg->CPU_APB_REG0 & pllLock)
        {
            break;
        }
        DRVDelayCyc(100);
    }
}
void SCUSetPll_temp(eSCU_PLL_ID pllId, uint32 nMHz)
{
    uint32 count,reg_temp;
    uint32 pllLock = (0x40u << pllId);
    volatile uint32 *pllReg,*div_reg ;
	pSCU_REG scu_reg = (pSCU_REG)SCU_BASE_ADDR_VA;
    switch (pllId)
    {
        case PLL_ARM:
            pllReg = &(scu_reg->SCU_APLL_CON);
            break;
        case PLL_DSP:
            pllReg = &(scu_reg->SCU_DPLL_CON);
            break;
        case PLL_CODEC:
            pllReg = &(scu_reg->SCU_CPLL_CON);
            break;
        default:
            break;
    }
	div_reg = &((scu_reg)->SCU_CLKSEL0_CON);
    if (*pllReg & PLL_PD)
    {
        *pllReg &= ~PLL_PD;
    }

    if (nMHz < 40 )			    //  25M ~ 39	vco= 200 ~ 320
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(3-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(8-1));
    }
    else if (nMHz < 80)			//  40 ~ 79,  vco = 160 ~ 320
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(4-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(6-1));
    }
    else if (nMHz < 160)		// 80 ~ 159,  vco = 160 ~ 320
    {
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(12-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(2-1));
    }
    else                        // 160 ~ 300 , VCO = 160 ~ 300
    {
    	reg_temp = *div_reg & (~0xf);
    	*div_reg = reg_temp | 0x06;  		// 3:1 and 2:1

#if ROCK_DEBUG
    	printk("\n\n-------------div_reg addr = %x",div_reg);
#endif
        *pllReg = PLL_SAT|PLL_FAST|(PLL_CLKR(24-1))|(PLL_CLKF(nMHz-1))|(PLL_CLKOD(1-1));
    }

    //等待PLL进入稳定状态，约0.3ms.
    DRVDelayCyc(5000);
    count = 2000;
    while (count --)
    {
        if (g_grfReg->CPU_APB_REG0 & pllLock)
        {
            break;
        }
        DRVDelayCyc(100);
    }
}

/*----------------------------------------------------------------------
Name	: SCUSetARMPll
Desc	: 设置ARM PLL频率
Params  : freq:ARM PLL频率
Return  :
Notes   : ARM pll VCO输出固定为300M，仅切换OD，另外ARM:HCLK比例固定为2:1
----------------------------------------------------------------------*/
void SCUSetARMPll(uint32 div)
{
    uint32 count;
    if (g_chipClk.armFreq)
    {
        if (div)
        {
            if (div > 6)//ahb<25MHz
            {
                g_scuReg->SCU_CLKSEL0_CON &= ~CLK_PCLK_MASK;    //HCLK:PCLK = 1:1
                g_chipClk.apbDiv = PCLK_DIV1;
            }
            else
            {
                g_scuReg->SCU_CLKSEL0_CON |= (CLK_HCLK_PCLK_21);//HCLK:PCLK = 2:1
                g_chipClk.apbDiv = PCLK_DIV2;
            }

            g_scuReg->SCU_APLL_CON = PLL_SAT|PLL_FAST|(PLL_CLKR(2-1))|(PLL_CLKF(25-1))|(PLL_CLKOD(div-1));

            //大概 0.3MS后处于稳定状态.
            DRVDelayCyc(5000);

            count = 2000;
            while (count --)
            {
                if (g_grfReg->CPU_APB_REG0 & APBREG0_APLL_LOCK_STAT)
                {
                    break;
                }
                DRVDelayCyc(100);
            }
            g_scuReg->SCU_MODE_CON |= SCU_CPUMODE_NORMAL;
        }
        else //FREQ_ARM_MIN power off arm pll
        {
            g_scuReg->SCU_MODE_CON &= ~SCU_CPUMODE_MASK;//SLOW MODE
            g_scuReg->SCU_CLKSEL0_CON &= ~CLK_PCLK_MASK;//ARM:HCLK = 1:1
            g_chipClk.apbDiv = PCLK_DIV1;

            SCUPwrDwnPLL(PLL_ARM);
        }
    }
    else//initial hclk div
    {
        g_scuReg->SCU_MODE_CON &= ~SCU_CPUMODE_MASK;//SLOW MODE
        g_scuReg->SCU_APLL_CON = PLL_SAT|PLL_FAST|(PLL_CLKR(2-1))|(PLL_CLKF(25-1))|(PLL_CLKOD(div-1));

        //大概 0.3MS后处于稳定状态.
        DRVDelayCyc(5000);

        count = 2000;
        while (count --)
        {
            if (g_grfReg->CPU_APB_REG0 & APBREG0_APLL_LOCK_STAT)
            {
                break;
            }
            DRVDelayCyc(100);
        }
        g_scuReg->SCU_CLKSEL0_CON &= ~CLK_HCLK_MASK;//ARM:HCLK = 2:1
        if (div > 6)//ahb<25MHz
        {
            g_scuReg->SCU_CLKSEL0_CON &= ~CLK_PCLK_MASK;    //HCLK:PCLK = 1:1
            g_chipClk.apbDiv = PCLK_DIV1;
        }
        else
        {
            g_scuReg->SCU_CLKSEL0_CON |= (CLK_HCLK_PCLK_21);//HCLK:PCLK = 2:1
            g_chipClk.apbDiv = PCLK_DIV2;
        }
        g_scuReg->SCU_CLKSEL0_CON |= CLK_ARM_HCLK_21;
        g_scuReg->SCU_MODE_CON |= SCU_CPUMODE_NORMAL;
        g_chipClk.ahbDiv = HCLK_DIV2;
    }
}

/*----------------------------------------------------------------------
Name	: SCUSetDSPPll
Desc	: 设置DSP PLL频率
Params  : nMHz:DSP频率
Return  :
Notes   : 频率应在DSP主频有效范围内
----------------------------------------------------------------------*/
void SCUSetDSPPll(uint32 nMHz)
{
    g_scuReg->SCU_MODE_CON &= ~SCU_DSPMODE_MASK;//enter slow mode
    if ((nMHz == 24)||(nMHz == 0))
    {
        SCUPwrDwnPLL(PLL_DSP);
    }
    else
    {
        SCUSetPll(PLL_DSP, nMHz);
        g_scuReg->SCU_MODE_CON |= SCU_DSPMODE_NORMAL;//normal mode
    }
    g_chipClk.dspFreq = nMHz;
}

/*----------------------------------------------------------------------
Name	: SCUSetAUXPll
Desc	: 设置CODEC PLL输出频率
Params  : nMHz:pll频率
Return  :
Notes   : 如果频率为24M或者0则关闭CODEC PLL
----------------------------------------------------------------------*/
void SCUSetAUXPll(uint32 nMHz)
{
    g_scuReg->SCU_CLKSEL1_CON &= ~0x11u;//CLK_CPLL_SLOW;enter slow mode
    if ((nMHz == 24)||(nMHz == 0))
    {
        SCUPwrDwnPLL(PLL_CODEC);
    }
    else
    {
        SCUSetPll(PLL_CODEC, nMHz);
        g_scuReg->SCU_CLKSEL1_CON |= CLK_CPLL_NORMAL;//normal mode
    }
    g_chipClk.AuxFreq = nMHz;
}

