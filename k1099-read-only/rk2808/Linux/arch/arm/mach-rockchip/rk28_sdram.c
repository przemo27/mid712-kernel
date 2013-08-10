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
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_scu.h>

#define SDRAM_2x32x4     (1)   // 32M
#define SDRAM_4x32x4     (2)   // 64M
#define SDRAM_8x32x4     (3)   // 128M
#define SDRAM_16x32x4    (4)   // 256M

#define SDRAM_SIZE       SDRAM_8x32x4

/* CPU_APB_REG4 */
#define MSDR_1_8V_ENABLE  (0x1 << 24)
#define READ_PIPE_ENABLE  (0x1 << 22)
#define EXIT_SELF_REFRESH (0x1 << 21)

/* SDRAM Config Register */
#define DATA_WIDTH_16     (0x0 << 13)
#define DATA_WIDTH_32     (0x1 << 13)
#define DATA_WIDTH_64     (0x2 << 13)
#define DATA_WIDTH_128    (0x3 << 13)

#define COLUMN_8          (7 << 9)
#define COLUMN_9          (8 << 9)
#define COLUMN_10         (9 << 9)
#define COLUMN_11         (10 << 9)
#define COLUMN_12         (11 << 9)
#define COLUMN_13         (12 << 9)
#define COLUMN_14         (13 << 9)
#define COLUMN_15         (14 << 9)

#define ROW_11            (10 << 5)
#define ROW_12            (11 << 5)
#define ROW_13            (12 << 5)
#define ROW_14            (13 << 5)
#define ROW_15            (14 << 5)
#define ROW_16            (15 << 5)

#define BANK_2            (0 << 3)
#define BANK_4            (1 << 3)
#define BANK_8            (2 << 3)
#define BANK_16           (3 << 3)

#if (SDRAM_SIZE == SDRAM_2x32x4)
#define SDRAM_DATA_WIDTH  DATA_WIDTH_32
#define SDRAM_COLUMN      COLUMN_9
#define SDRAM_ROW         ROW_12
#define SDRAM_BANK        BANK_4
#elif (SDRAM_SIZE == SDRAM_4x32x4)
#define SDRAM_DATA_WIDTH  DATA_WIDTH_32
#define SDRAM_COLUMN      COLUMN_9
#define SDRAM_ROW         ROW_13
#define SDRAM_BANK        BANK_4
#elif (SDRAM_SIZE == SDRAM_8x32x4)
#define SDRAM_DATA_WIDTH  DATA_WIDTH_32
#define SDRAM_COLUMN      COLUMN_10
#define SDRAM_ROW         ROW_13
#define SDRAM_BANK        BANK_4
#elif (SDRAM_SIZE == SDRAM_16x32x4)  //hynix有这么大的
#define SDRAM_DATA_WIDTH  DATA_WIDTH_32
#define SDRAM_COLUMN      COLUMN_10
#define SDRAM_ROW         ROW_14
#define SDRAM_BANK        BANK_4
#else
#error SDRAM_SIZE defined error!
#endif

/* SDRAM Timing Register0 */
#define T_RC_SHIFT        (22)
#define T_RC_MAX         (0xF)
#define T_XSR_MSB_SHIFT   (27)
#define T_XSR_MSB_MASK    (0x1F)
#define T_XSR_LSB_SHIFT   (18)
#define T_XSR_LSB_MASK    (0xF)
#define T_RCAR_SHIFT      (14)
#define T_RCAR_MAX       (0xF)
#define T_WR_SHIFT        (12)
#define T_WR_MAX         (0x3)
#define T_RP_SHIFT        (9)
#define T_RP_MAX         (0x7)
#define T_RCD_SHIFT       (6)
#define T_RCD_MAX        (0x7)
#define T_RAS_SHIFT       (2)
#define T_RAS_MASK        (0xF)


#define CL_1              (0)
#define CL_2              (1)
#define CL_3              (2)
#define CL_4              (3)

/* SDRAM Timing Register1 */
#define AR_COUNT_SHIFT    (16)

/* SDRAM Control Regitster */
#define MSD_DEEP_POWERDOWN (1 << 20)
#define UPDATE_EMRS    (1 << 18)
#define OPEN_BANK_COUNT_SHIFT  (12)
#define SR_MODE            (1 << 11)
#define UPDATE_MRS         (1 << 9)
#define READ_PIPE_SHIFT    (6)
#define REFRESH_ALL_ROW_A  (1 << 5)
#define REFRESH_ALL_ROW_B  (1 << 4)
#define DELAY_PRECHARGE    (1 << 3)
#define SDR_POWERDOWN      (1 << 2)
#define ENTER_SELF_REFRESH (1 << 1)
#define SDR_INIT           (1 << 0)

/* Extended Mode Register */
#define DS_FULL            (0 << 5)
#define DS_1_2             (1 << 5)
#define DS_1_4             (2 << 5)
#define DS_1_8             (3 << 5)

#define TCSR_70            (0 << 3)
#define TCSR_45            (1 << 3)
#define TCSR_15            (2 << 3)
#define TCSR_85            (3 << 3)

#define PASR_4_BANK        (0)
#define PASR_2_BANK        (1)
#define PASR_1_BANK        (2)
#define PASR_1_2_BANK      (5)
#define PASR_1_4_BANK      (6)

/* SDRAM Controller register struct */
typedef volatile struct TagSDRAMC_REG
{
    volatile uint32 MSDR_SCONR;         //SDRAM configuration register
    volatile uint32 MSDR_STMG0R;        //SDRAM timing register0
    volatile uint32 MSDR_STMG1R;        //SDRAM timing register1
    volatile uint32 MSDR_SCTLR;         //SDRAM control register
    volatile uint32 MSDR_SREFR;         //SDRAM refresh register
    volatile uint32 MSDR_SCSLR0_LOW;    //Chip select register0(lower 32bits)
    volatile uint32 MSDR_SCSLR1_LOW;    //Chip select register1(lower 32bits)
    volatile uint32 MSDR_SCSLR2_LOW;    //Chip select register2(lower 32bits)
    uint32 reserved0[(0x54-0x1c)/4 - 1];
    volatile uint32 MSDR_SMSKR0;        //Mask register 0
    volatile uint32 MSDR_SMSKR1;        //Mask register 1
    volatile uint32 MSDR_SMSKR2;        //Mask register 2
    uint32 reserved1[(0x84-0x5c)/4 - 1];
    volatile uint32 MSDR_CSREMAP0_LOW;  //Remap register for chip select0(lower 32 bits)
    uint32 reserved2[(0x94-0x84)/4 - 1];
    volatile uint32 MSDR_SMTMGR_SET0;   //Static memory timing register Set0
    volatile uint32 MSDR_SMTMGR_SET1;   //Static memory timing register Set1
    volatile uint32 MSDR_SMTMGR_SET2;   //Static memory timing register Set2
    volatile uint32 MSDR_FLASH_TRPDR;   //FLASH memory tRPD timing register
    volatile uint32 MSDR_SMCTLR;        //Static memory control register
    uint32 reserved4;
    volatile uint32 MSDR_EXN_MODE_REG;  //Extended Mode Register
}SDRAMC_REG_T,*pSDRAMC_REG_T;


#define pSDR_Reg       ((pSDRAMC_REG_T)SDRAMC_BASE_ADDR_VA)

/****************************************************************/
//函数名:SDRAM_UpdateAhbFreq
//描述:根据当前AHB时钟频率，调整SDRAM控制器参数
//参数说明:newKHz   输入参数   AHB频率，单位KHz
//返回值:
//相关全局变量:
//注意:这个函数只修改MSDR_STMG0R，MSDR_SREFR，MSDR_SCTLR的值
/****************************************************************/
static void __sdram_change_refresh( int khz )
{
#if (SDRAM_SIZE == SDRAM_2x32x4)
        pSDR_Reg->MSDR_SREFR = (((125*khz)/1000) >> 3) & 0xFFFF;  // 125/8 = 15.625us
#else
        pSDR_Reg->MSDR_SREFR = (((62*khz)/1000) >> 3) & 0xFFFF;  // 62/8 = 7.75us
#endif   

}
static void __sdram_change_timing( int newKHz )
{
        unsigned int value , tmp ;
        value = pSDR_Reg->MSDR_STMG0R;
        value &= 0xFC3C003F;
        //t_rc =  15ns
        tmp = (15*newKHz/1000000) + ((((15*newKHz)%1000000) > 0) ? 1:0);
        tmp = (tmp > 0) ? (tmp - 1) : 0;
        tmp = (tmp > T_RC_MAX) ? T_RC_MAX : tmp;
        value |= tmp << T_RC_SHIFT;
        //t_rcar = 80ns
        tmp = (80*newKHz/1000000) + ((((80*newKHz)%1000000) > 0) ? 1:0);
        tmp = (tmp > 0) ? (tmp - 1) : 0;
        tmp = (tmp > T_RCAR_MAX) ? T_RCAR_MAX : tmp;
        value |= tmp << T_RCAR_SHIFT;
        //t_wr 固定为2个clk
        value |= 1 << T_WR_SHIFT;
        //t_rp =  20ns
        tmp = (20*newKHz/1000000) + ((((20*newKHz)%1000000) > 0) ? 1:0);
        tmp = (tmp > 0) ? (tmp - 1) : 0;
        tmp = (tmp > T_RP_MAX) ? T_RP_MAX : tmp;
        value |= tmp << T_RP_SHIFT;
        //t_rcd = 20ns
        tmp = (20*newKHz/1000000) + ((((20*newKHz)%1000000) > 0) ? 1:0);
        tmp = (tmp > 0) ? (tmp - 1) : 0;
        tmp = (tmp > T_RCD_MAX) ? T_RCD_MAX : tmp;
        value |= tmp << T_RCD_SHIFT;
        pSDR_Reg->MSDR_STMG0R = value;

}

int __sdram_change_ahbclk( struct rockchip_scu_unit * p  , int stage )
{

       // int newKHz = p->tmp_clk*1000;   /* tmp clk unit = MHZ */
       	/* 20100319,HSL@RK,new version scu,tmp clk unit=KHZ.
       	 * XXX:direct use p->tmp_clk is dangerous!!!
       	*/
       	 int newKHz = p->tmp_clk;	
        //SCU_BUG("chang sdram clk to %d,cur=%d,stage=%d" , p->tmp_clk , p->cur_clk , stage );

        /*
         *  频率升高情况下，refresh需要在改频后进行，timing需要在改频前进行
         *  频率减低的情况下，相反.
         */
         if( p->tmp_clk < p->cur_clk )  {
                if( stage == 0 ) {
                        __sdram_change_refresh( newKHz );
                        return SCU_SETCLK_PARENT;
                } else
                        __sdram_change_timing( newKHz );
                
        } else {
                /*
                 * XXX: BUG: at scu , when clk grow,ip will always change first.
                 */
                 if( stage == 0 ) {
                        __sdram_change_timing( newKHz );
                        return SCU_SETCLK_PARENT;
                }  else
                        __sdram_change_refresh( newKHz );
        }
        return SCU_SETCLK_OK;
}

#if 0
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
#endif

