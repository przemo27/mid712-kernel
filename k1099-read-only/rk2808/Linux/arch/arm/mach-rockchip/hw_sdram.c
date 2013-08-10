/****************************************************************
//	  CopyRight(C) 2008 by Rock-Chip Fuzhou
//		All Rights Reserved
//�ļ���:hw_sdram.c
//����:sdram driver implement
//����:hcy
//��������:2008-11-08
//���ļ�¼:
$Log: hw_sdram.c,v $
Revision 1.3  2009/03/19 13:38:39  hxy
hcyȥ��SDRAM����ʱ��,����ʱ�򲻶�,����MP3����ʱ���涶��

Revision 1.2  2009/03/19 12:21:18  hxy
hcy����SDRAM����ʱ�򹩲���

Revision 1.1.1.1  2009/03/16 01:34:06  zjd
20090316 ��ѵ���ṩ��ʼSDK�汾

Revision 1.2  2009/03/07 07:30:18  yk
(yk)����SCUģ���Ƶ�����ã�������к��������룬���³�ʼ�����ã�
����ң�������룬ɾ��FPGA_BOARD�ꡣ
(hcy)SDRAM�����ĳ�28��

//��ǰ�汾:1.00
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
//������:SDRAM_Init
//����:SDRAM��ʼ��
//����˵��:
//����ֵ:
//���ȫ�ֱ���:
//ע��:
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
    //precharge������Զ��������е�bank
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
//������:SDRAM_UpdateAhbFreq
//����:���ݵ�ǰAHBʱ��Ƶ�ʣ�����SDRAM����������
//����˵��:newKHz   �������   AHBƵ�ʣ���λKHz
//����ֵ:
//���ȫ�ֱ���:
//ע��:�������ֻ�޸�MSDR_STMG0R��MSDR_SREFR��MSDR_SCTLR��ֵ
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
    if(newKHz >= 90000)  //����90M������read pipe
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
//������:SDRAM_EnterSelfRefresh
//����:SDRAM������ˢ��ģʽ
//����˵��:
//����ֵ:
//���ȫ�ֱ���:
//ע��:(1)ϵͳ��ȫidle����ܽ�����ˢ��ģʽ��������ˢ�º����ٷ���SDRAM
//	   (2)Ҫ������ˢ��ģʽ�����뱣֤����ʱ������������õ������д��벻��SDRAM��
/****************************************************************/
void SDRAM_EnterSelfRefresh(void)
{
#if (BOARDTYPE != RK2800_FPGA)
    volatile uint32 value =0;
    
    value = pSDR_Reg->MSDR_SCTLR;
    value |= ENTER_SELF_REFRESH;
    pSDR_Reg->MSDR_SCTLR = value;

    while(!((value = pSDR_Reg->MSDR_SCTLR) & SR_MODE));  //ȷ���Ѿ�����self-refresh
    
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
//������:SDRAM_ExitSelfRefresh
//����:SDRAM�˳���ˢ��ģʽ
//����˵��:
//����ֵ:
//���ȫ�ֱ���:
//ע��:(1)SDRAM����ˢ��ģʽ���ܱ����ʣ��������˳���ˢ��ģʽ
//	   (2)���뱣֤����ʱ��������Ĵ��벻��SDRAM��
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

    while((value = pSDR_Reg->MSDR_SCTLR) & SR_MODE);  //ȷ���˳�����self-refresh
    DRVDelayCyc(100); //��ʱһ�±Ƚϰ�ȫ����֤�˳����ȶ�
#endif
}


