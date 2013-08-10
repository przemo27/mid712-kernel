/****************************************************************
//	  CopyRight(C) 2008 by Rock-Chip Fuzhou
//		All Rights Reserved
//文件名:hw_sdram.c
//描述:sdram driver implement
//作者:hcy
//创建日期:2008-11-08
//更改记录:
$Log: hw_sdram.c,v $
Revision 1.3  2009/03/19 13:38:39  hxy
hcy去掉SDRAM保守时序,保守时序不对,导致MP3播放时界面抖动

Revision 1.2  2009/03/19 12:21:18  hxy
hcy增加SDRAM保守时序供测试

Revision 1.1.1.1  2009/03/16 01:34:06  zjd
20090316 邓训金提供初始SDK版本

Revision 1.2  2009/03/07 07:30:18  yk
(yk)更新SCU模块各频率设置，完成所有函数及代码，更新初始化设置，
更新遥控器代码，删除FPGA_BOARD宏。
(hcy)SDRAM驱动改成28的

//当前版本:1.00
****************************************************************/
#define  SDRAM_DRIVER
#include <asm/arch/hw_sdram.h>
#include <asm/arch/api_pll.h>
#include <asm/arch/api_scu.h>
#include <asm/arch/drivers_delay.h>
#include <asm/arch/rk28_scu.h>


#define pSDR_Reg       ((pSDRAMC_REG_T)SDRAM_REG_BASE)
#define pSDR_SUB_Reg  ((volatile uint32*)SDRAM_SUB_REG_BASE)


void SDRAM_UpdateAhbFreq(uint32 newKHz);

/****************************************************************/
//函数名:SDRAM_Init
//描述:SDRAM初始化
//参数说明:
//返回值:
//相关全局变量:
//注意:
/****************************************************************/
void SDRAM_Init(void)
{
	volatile uint32 value = 0;
	//uint32			ahbKHz = PLLGetAHBFreq();
        uint32			ahbKHz = rockchip_clk_get_ahb( );
	uint32			n = 0;

#if (SDRAM_TYPE == MOBILE_SDRAM)
    value = pSDR_Reg->MSDR_SCONR;
    value &= ~(0x7FF8);
    value |= (SDRAM_DATA_WIDTH | SDRAM_COLUMN | SDRAM_ROW | SDRAM_BANK);
    pSDR_Reg->MSDR_SCONR = value;
    pSDR_Reg->MSDR_EXN_MODE_REG = (DS_FULL | TCSR_70 | PASR_1_BANK);
    value = *pSDR_SUB_Reg;
    value |= MSDR_1_8V_ENABLE;
    *pSDR_SUB_Reg = value;
    //SCUDisableClk(CLK_GATE_SDRMEM);
    rockchip_scu_disableclk( SCU_IPID_MSDRMEM);
#else
    value = pSDR_Reg->MSDR_SCONR;
    value &= ~(0x1FFC);
    value |= (SDRAM_COLUMN | SDRAM_ROW | SDRAM_BANK);
    pSDR_Reg->MSDR_SCONR = value;
    //SCUDisableClk(CLK_GATE_MSDRMEM);
    rockchip_scu_disableclk( SCU_IPID_SDRMEM);
#endif

    SDRAM_UpdateAhbFreq(ahbKHz);
    // 200us
    n = 200*ahbKHz/1000;
    //precharge命令会自动发给所有的bank
    pSDR_Reg->MSDR_STMG1R = ((7 << AR_COUNT_SHIFT) | n);
    value = pSDR_Reg->MSDR_SCTLR;
#if (SDRAM_TYPE == MOBILE_SDRAM)
    value |= (SDR_INIT | UPDATE_MRS | UPDATE_EMRS);
    pSDR_Reg->MSDR_SCTLR  = value;
    while((value = pSDR_Reg->MSDR_SCTLR) & (SDR_INIT | UPDATE_MRS | UPDATE_EMRS));
#else
    value |= (SDR_INIT | UPDATE_MRS);
    pSDR_Reg->MSDR_SCTLR  = value;
    while((value = pSDR_Reg->MSDR_SCTLR) & (SDR_INIT | UPDATE_MRS));
#endif
}

/****************************************************************/
//函数名:SDRAM_UpdateAhbFreq
//描述:根据当前AHB时钟频率，调整SDRAM控制器参数
//参数说明:newKHz   输入参数   AHB频率，单位KHz
//返回值:
//相关全局变量:
//注意:这个函数只修改MSDR_STMG0R，MSDR_SREFR，MSDR_SCTLR的值
/****************************************************************/
void SDRAM_UpdateAhbFreq(uint32 newKHz)
{
    uint32 value =0;
    uint32 tmp = 0;

#if (SDRAM_SIZE == SDRAM_2x32x4)
    // 64Mb and 128Mb SDRAM's auto refresh cycle 15.6us or a burst of 4096 auto refresh cycles once in 64ms
    pSDR_Reg->MSDR_SREFR = (((125*newKHz)/1000) >> 3) & 0xFFFF;  // 125/8 = 15.625us
#else
    // 256Mb and 512Mb SDRAM's auto refresh cycle 7.8us or a burst of 8192 auto refresh cycles once in 64ms
	pSDR_Reg->MSDR_SREFR = (((62*newKHz)/1000) >> 3) & 0xFFFF;  // 62/8 = 7.75us
#endif   

#if 1
    value = CL_3;
    //t_rc =  77ns   65       60    60     66   67.5  72.5
    tmp = (77*newKHz/1000000) + ((((77*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= (tmp & T_RC_MASK) << T_RC_SHIFT;
    //t_xsr = 120ns  67       67    75     80    112.5
    tmp = (120*newKHz/1000000) + ((((120*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= ((((tmp >>  4) & T_XSR_MSB_MASK) << T_XSR_MSB_SHIFT) | ((tmp & T_XSR_LSB_MASK) << T_XSR_LSB_SHIFT));
    //t_rcar = 80ns  60       66    66     72
    tmp = (80*newKHz/1000000) + ((((80*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= (tmp & T_RCAR_MASK) << T_RCAR_SHIFT;
    //t_wr =  15ns   tRDL = 2 CLK
    tmp = (15*newKHz/1000000) + ((((15*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= (tmp & T_WR_MASK) << T_WR_SHIFT;
    //t_rp =  27ns   20       18    15     20   19    22.5
    tmp = (27*newKHz/1000000) + ((((27*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= (tmp & T_RP_MASK) << T_RP_SHIFT;
    //t_rcd = 27ns   20       18    15     20   19    22.5
    tmp = (27*newKHz/1000000) + ((((27*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= (tmp & T_RCD_MASK) << T_RCD_SHIFT;
    //t_ras = 50ns   45       42    37     44   45    50
    tmp = (50*newKHz/1000000) + ((((50*newKHz)%1000000) > 0) ? 1:0);
    tmp = (tmp > 0) ? (tmp - 1) : 0;
    value |= (tmp & T_RAS_MASK) << T_RAS_SHIFT;
#else
    value = ((11<<22) | (1<<27) | (1<<18) | (11<<14) | (2<<12) | (7<<9) | (7 << 6) | (7 << 2) | CL_3);
#endif
    pSDR_Reg->MSDR_STMG0R = value;

#if 0
    if(newKHz >= 90000)  //大于90M，开启read pipe
    {
        value = *pSDR_SUB_Reg;
        value |= READ_PIPE_ENABLE;
        *pSDR_SUB_Reg = value;
        value = pSDR_Reg->MSDR_SCTLR;
        value &= ~(0x7 << READ_PIPE_SHIFT);
        value |= (0x1 << READ_PIPE_SHIFT);
        pSDR_Reg->MSDR_SCTLR = value;
    }
    else
    {
        value = *pSDR_SUB_Reg;
        value &= ~(READ_PIPE_ENABLE);
        *pSDR_SUB_Reg = value;
    }
#endif
}

/****************************************************************/
//函数名:SDRAM_EnterSelfRefresh
//描述:SDRAM进入自刷新模式
//参数说明:
//返回值:
//相关全局变量:
//注意:(1)系统完全idle后才能进入自刷新模式，进入自刷新后不能再访问SDRAM
//	   (2)要进入自刷新模式，必须保证运行时这个函数所调用到的所有代码不在SDRAM上
/****************************************************************/
void SDRAM_EnterSelfRefresh(void)
{
#if (BOARDTYPE != RK2800_FPGA)
    volatile uint32 value =0;
    
    value = pSDR_Reg->MSDR_SCTLR;
    value |= ENTER_SELF_REFRESH;
    pSDR_Reg->MSDR_SCTLR = value;

    while(!((value = pSDR_Reg->MSDR_SCTLR) & SR_MODE));  //确定已经进入self-refresh
    
#if (SDRAM_TYPE == MOBILE_SDRAM)
    //SCUDisableClk(CLK_GATE_MSDRMEM);
    rockchip_scu_disableclk( SCU_IPID_MSDRMEM);
#else
    //SCUDisableClk(CLK_GATE_SDRMEM);
    rockchip_scu_disableclk( SCU_IPID_SDRMEM);
#endif
#endif
}

/****************************************************************/
//函数名:SDRAM_ExitSelfRefresh
//描述:SDRAM退出自刷新模式
//参数说明:
//返回值:
//相关全局变量:
//注意:(1)SDRAM在自刷新模式后不能被访问，必须先退出自刷新模式
//	   (2)必须保证运行时这个函数的代码不在SDRAM上
/****************************************************************/
void SDRAM_ExitSelfRefresh(void)
{
#if (BOARDTYPE != RK2800_FPGA)
    volatile uint32 value =0;

#if (SDRAM_TYPE == MOBILE_SDRAM)
//    SCUEnableClk(CLK_GATE_MSDRMEM);
        rockchip_scu_enableclk( SCU_IPID_SDRMEM);
#else
    //SCUEnableClk(CLK_GATE_SDRMEM);
        rockchip_scu_enableclk( SCU_IPID_SDRMEM);
#endif
    value = pSDR_Reg->MSDR_SCTLR;
    value &= ~(ENTER_SELF_REFRESH);
    pSDR_Reg->MSDR_SCTLR = value;

    while((value = pSDR_Reg->MSDR_SCTLR) & SR_MODE);  //确定退出进入self-refresh
    DRVDelayCyc(100); //延时一下比较安全，保证退出后稳定
#endif
}


