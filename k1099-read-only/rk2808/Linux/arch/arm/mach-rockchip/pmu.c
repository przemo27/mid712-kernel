/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File	:	pmu.c
Desc	:
Author	:	yangkai
Date	:	2008-12-16
Notes	:
$Log: pmu.c,v $
Revision 1.4  2009/03/11 03:33:38  hcy
驱动调整

Revision 1.3  2009/03/09 06:25:35  hcy
(hcy)文件系统修改了卡的mount实现，相应的卡驱动进行修改：文件系统卡mount从以前主动发起IO_CTL_GET_MEDIUM_STATUS命令改为由卡调用FS_MountDevice来实现mount，因此在插卡开机时需要跟着修改。
(yk)更新clock管理

Revision 1.2  2009/03/07 07:30:18  yk
(yk)更新SCU模块各频率设置，完成所有函数及代码，更新初始化设置，
更新遥控器代码，删除FPGA_BOARD宏。
(hcy)SDRAM驱动改成28的


********************************************************************/
#define IN_PMU
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


/*----------------------------------------------------------------------
Name	: PMUSetARMFreq
Desc	: 设置ARM主频、HCLK、PCLK分频
Params	: freq: ARM 频率
		  hclkDiv: HCLK 分频因子
		  pclkDiv: PCLK 分频因子
Return	:
Notes	: ARM, HCLK, PCLK的频率应在有效范围内
----------------------------------------------------------------------*/
void PMUSetARMFreq(uint32 nMHz)
{
}

/*----------------------------------------------------------------------
Name	: PMUSetDSPFreq
Desc	: 设置DSP频率
Params	: nMHz:DSP频率
Return	:
Notes	: 频率应在DSP主频有效范围内
----------------------------------------------------------------------*/
void PMUSetDSPFreq(uint32 nMHz)
{
	if (g_chipClk.dspFreq == nMHz)
	{
		return;
	}
	if (nMHz)//频率不为0，设置PLL
	{
		if (nMHz < FREQ_DSP_MIN)
		{
			nMHz = FREQ_DSP_MIN;
		}
		if (nMHz > FREQ_DSP_MAX)
		{
			nMHz = FREQ_DSP_MAX;
		}
		
		if (!PLLGetDspFreq())//原dsp不运行
		{
			SCUPwrOnDomain(SCU_PD1_DSP);
			SCUDisableClk(CLK_GATE_DSP);
			SCUDisableClk(CLK_GATE_DSPBUS);
			SCURstModule(SCU_RST_DSP);
			SCURstModule(SCU_RST_DSPPER);
			SCURstModule(SCU_RST_DSPA2A);
			
			SCUSetDSPPll(nMHz);
			
			SCUEnableClk(CLK_GATE_DSPBUS);
			SCUEnableClk(CLK_GATE_DSP);
			SCUUnrstModule(SCU_RST_DSPPER);
            SCUUnrstModule(SCU_RST_DSPA2A);
            //设置DSP的PMU
            g_cpmuReg->CXCLK_DIV = 0;   //clki:ceva = 1:1
            g_cpmuReg->XHCLK_DIV = 1;   //ceva:hclk = 2:1
            g_cpmuReg->XPCLK_DIV = 1;   //hclk:pclk = 2:1
          
        }
		else//DSP已经在运行
		{
			SCUSetDSPPll(nMHz);
		}
	}
	else//DSP频率为0，关闭DSP子系统及DSP PLL
	{
		SCURstModule(SCU_RST_DSP);
		SCURstModule(SCU_RST_DSPPER);
		SCURstModule(SCU_RST_DSPA2A);
		SCUDisableClk(CLK_GATE_DSP);
		SCUDisableClk(CLK_GATE_DSPBUS);
		
		SCUSetDSPPll(nMHz);
		SCUPwrOffDomain(SCU_PD1_DSP);
	}
}

/*----------------------------------------------------------------------
Name	: PMUSetCodecFreq
Desc	: 设置CODEC频率
Params	: freq:codec频率
Return	:
Notes	: codec使用12M clk，直接使用外部时钟，不使用CODEC PLL
----------------------------------------------------------------------*/
int32 PMUSetCodecFreq(uint32 nKHz)
{
#if(BOARDTYPE ==RK2800_FPGA)
		return(0);
#endif
	if (nKHz == 12000)//12M Hz
	{
        g_scuReg->SCU_CLKSEL1_CON |= CLK_CODEC_12M;//select external 12M clk
		return(0);
	}
	else
	{
		return( -1);
	}
}

/*----------------------------------------------------------------------
Name	: PMUSetLCDCFreq
Desc	: 设置LCDC频率
Params	: freq: LCDC频率
Return	:
Notes	: LCDC默认为CODEC PLL
----------------------------------------------------------------------*/
void PMUSetLCDCFreq(uint32 nMHz)
{
#if(BOARDTYPE ==RK2800_FPGA)
		return;
#endif
	if (g_chipClk.AuxFreq == nMHz)
	{
		return;
	}
    g_scuReg->SCU_CLKSEL0_CON &= ~(0x7ff<<7);
    g_scuReg->SCU_CLKSEL0_CON |= ((CLK_LCDC_CODPLL)
                                 |CLK_LCDC_DIV(1)
                                 |CLK_LCDC_DIVOUT);
    SCUSetAUXPll(nMHz);
	if(nMHz == 0)
	{
		SCUDisableClk(CLK_GATE_LCDC);
		SCUDisableClk(CLK_GATE_LCDCh);
	}
	else if((nMHz > 0) && (g_chipClk.AuxFreq == 0))
	{
		SCUEnableClk(CLK_GATE_LCDC);
		SCUEnableClk(CLK_GATE_LCDCh);
	}
}

/*----------------------------------------------------------------------
Name	: PMUSetFreq
Desc	: 设置各个PLL的输出频率以及总线频率
Params	: appList:应用列表
Return	: 
Notes	: appList中的每一位代表一个应用
----------------------------------------------------------------------*/
int32 PMUSetFreq(uint64 appList)
{
	uint32 armFreq = 0;
	uint32 dspFreq = 0;
	uint32 i;

	if(!appList)
	{
		appList = PMU_IDLE;
	}
	for(i = 0; i < PMU_APP_MAX; i++)
	{
		if(appList & ((uint64)0x01<<i))
		{
			armFreq += g_scuAPPTabel[i].armFreq;
			dspFreq += g_scuAPPTabel[i].dspFreq;
		}
	}
	if(armFreq > FREQ_ARM_IDLE)
	{
		armFreq -= FREQ_ARM_IDLE;
	}
	
	PMUSetARMFreq(armFreq);
	PMUSetDSPFreq(dspFreq);
	return(0);
}

/*----------------------------------------------------------------------
Name	: PMUStartAPP
Desc	: 启动一个应用
Params	: appId:应用编号
Return	:
Notes	: 
----------------------------------------------------------------------*/
int32 PMUStartAPP(ePMU_APP appId)
{
#if(BOARDTYPE ==RK2800_FPGA)
		return(0);
#endif
	g_scuAPPTabel[appId].counter += 1;
	if(g_scuAPPTabel[appId].counter > 1)
	{
		return(0);
	}
	else
	{

		g_APPList |= ((uint64)0x01<< appId);
		PMUSetFreq(g_APPList);

		return(0);
	}
	
}

/*----------------------------------------------------------------------
Name	: PMUStopAPP
Desc	: 停止一个应用
Params	: appId:应用编号
Return	:
Notes	: 
----------------------------------------------------------------------*/
int32 PMUStopAPP(ePMU_APP appId)
{
#if(BOARDTYPE ==RK2800_FPGA)
		return(0);
#endif
	if(appId >= PMU_APP_MAX)
	{
		return (-1);
	}
	if(g_scuAPPTabel[appId].counter > 1)
	{
		g_scuAPPTabel[appId].counter -= 1;
		return(0);
	}
	else if(g_scuAPPTabel[appId].counter == 0)
	{
		return(-1);
	}
	g_scuAPPTabel[appId].counter -= 1;
	if(g_scuAPPTabel[appId].counter == 0)
	{

		g_APPList &= ~((uint64)0x01<< appId);
		PMUSetFreq(g_APPList);

	}
	return(0);
}


