/*******************************************************************/
/*    Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.             */
/*******************************************************************
File        :      adc.c
Desc        :     ADC 的驱动相关程序
Author      :     lhh
Date        :      2008-11-25
Notes       :
Revision 0.00
$Log: adc.c,v $
Revision 1.1.1.1  2009/03/16 01:34:06  zjd
20090316 邓训金提供初始SDK版本

Revision 1.4  2009/03/07 03:15:59  lhh
对系统组的建议对驱动的更改

Revision 1.3  2009/03/06 07:30:30  hxy
根据RK2800/RK2806原理图进行AD按键修改

Revision 1.2  2009/03/05 12:37:15  hxy
添加CVS版本自动注释

********************************************************************/
#define  IN_API_DRIVER_ADC
#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/iomux.h>
#include <asm/arch/adc.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/pll.h>
#include <asm/delay.h>

#include <linux/kernel.h>
#include <asm/io.h>
/*----------------------------------------------------------------------
Name      : ADCReadData(void)
Desc      :读ADC的值
Params    : 无
Return    : 还回ADC的值
----------------------------------------------------------------------*/
int32 ADCReadData(void)
{
    pADC_REG pheadAdc;
    int32  adcReturn;
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
    if ((pheadAdc->ADC_STAS&1) == ADC_STOP)
    {
        adcReturn = (pheadAdc->ADC_DATA & 0x3ff);
    }
    else
    {
        adcReturn = -1;
    }
    return (adcReturn);
}
/*----------------------------------------------------------------------
Name      : ADCStart(uint8 ch)
Desc      :启动ADC的转换  ch (0---3)
Params    : 无
Return    :启动成功还回为0，否则为 -1
----------------------------------------------------------------------*/
int32 ADCStart(uint8 ch)
{
    pADC_REG pheadAdc;
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
    //rockchip_scu_reset_unit( 7 );
    
    if(ch>3)
    {
        return (-1);
    }
 
    pheadAdc->ADC_CTRL = (pheadAdc->ADC_CTRL & (~0x1f)) | ADC_POWER_ON | ADC_START |ch;
    return (0);
}

/*----------------------------------------------------------------------
Name      : ADCIntEnabled()
Desc      :打开ADC中断使能
Params    : 无
Return    :
----------------------------------------------------------------------*/
void ADCIntEnabled(void)
{
    pADC_REG pheadAdc;
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
    pheadAdc->ADC_CTRL = (pheadAdc->ADC_CTRL & 0xdf) | ADC_ENABLED_INT;
}

/*----------------------------------------------------------------------
Name      : ADCIntDisabled(void)
Desc      :关闭ADC中断使能
Params    : 无
Return    :
----------------------------------------------------------------------*/
void ADCIntDisabled(void)
{
    pADC_REG pheadAdc;
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
    pheadAdc->ADC_CTRL = (pheadAdc->ADC_CTRL & 0xdf) | ADC_DISABLED_INT;
}

/*----------------------------------------------------------------------
Name      : ADCIntHander(void)
Desc      :ADC中断服务程序
Params    : 无
Return    :
----------------------------------------------------------------------*/
void ADCIntHander(void)
{
    int32 adcData;
    ADCIntDisabled();
    adcData = ADCReadData();
}

/*----------------------------------------------------------------------
Name      :ADCInit(void)
Desc      :ADC初值化
Params    : 无
Return    :初始化成功还回为0，否则为 -1
----------------------------------------------------------------------*/
int32 ADCInit(void)
{
    pADC_REG pheadAdc;
    
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;

    rockchip_scu_register( SCU_IPID_LSADC , SCU_MODE_FREQ ,  1 , NULL );   /* max 1M CLK*/
    
    pheadAdc->ADC_CTRL = 0 ; //ADC_POWER_ON;
    if(ADCStart(Adc_channel0) !=0)
    {
        return (-1);
    }

    return (0);
}

/*----------------------------------------------------------------------
Name      : ADCDeinit(void)
Desc      :ADC反初值化
Params    : 无
Return    :
----------------------------------------------------------------------*/
void ADCDeinit(void)
{
    //pSCU_REG pheadScu;

    //pheadScu = (pSCU_REG) SCU_BASE_ADDR;
   // ADCPowerOff();
   // pheadScu->SCU_CLKGATE0_CON = pheadScu->SCU_CLKGATE0_CON & (1 << 28); 
    pADC_REG pheadAdc;
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;

    pheadAdc->ADC_CTRL = ADC_POWER_OFF;

        rockchip_scu_disableclk( SCU_IPID_LSADC );
}

/*----------------------------------------------------------------------
Name      : RockAdcScanning(void)
Desc      :系统adc 扫描，对四个通道的adc定时扫描
Params    : 无
Return    :
----------------------------------------------------------------------*/
void RockAdcScanning(void)
{

    int32 adcTemp;

    
    //udelay(20);
    
    adcTemp = ADCReadData();
    if (adcTemp != -1)
    {
        g_adcValue[g_adcch] = (uint16)adcTemp;
    }
    else
    {
        printk ("\nRockAdcScanning: not stop\n");
        return;
    }
    g_adcch++;
    if (g_adcch>=Adc_channel_max)
        g_adcch = Adc_channel0;

   
   ADCStart(g_adcch);
    
}


/*  20100309,HSL@RK, for some project need read adc value  
  *  when wake from deep sleep . 
  *  CALL AT rk28_pm.c::rk28_pm_enter.
  */
#if 0
void ADCRestart( int ch )
{
#if 0
        int32 adcTemp = ADCReadData();
        if (adcTemp != -1)
        {
                debug_print("\nRockAdcScanning: not stop\n");
                return;
        }
#else
        pADC_REG pheadAdc;
        pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
        pheadAdc->ADC_CTRL &=  ~0x8; /* stop first */
#endif
	g_adcch = ch;
	if (g_adcch > Adc_channel2)
        	g_adcch = Adc_channel0;
        ADCStart(g_adcch);
}

int ADCGetKey( void )
{
	int 	tm = 250;
        pADC_REG pheadAdc;
        pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
        while( (pheadAdc->ADC_STAS&1)  && tm){
		tm --;
		udelay( 20 );
        }
	if(!tm )
		debug_print("\nRockAdcScanning: time out\n");
        g_adcValue[g_adcch] = (pheadAdc->ADC_DATA & 0x3ff);
        return g_adcValue[g_adcch];
}
#endif
/*BATTERY ADC*/
u16 get_rock_adc0(void)
{
	return g_adcValue[0];
}


int get_rock_adc1(void)
{
	return g_adcValue[1];

}


int get_rock_adc2(void)
{
	return g_adcValue[2];

}

/*BATTERY  VREF_ADC*/
u16 get_rock_adc3(void)	
{
	return g_adcValue[3];
}

//zhaojun add for see the register
void see_ioaddr(unsigned int io_addr)
{

	unsigned int reg_addr=io_addr&0xFFFFFFFC;

//	unsigned int pa=IO_VA2PA(reg_addr);

	printk("\n----------------------------------------------------------------------");
	printk("\n Below is the value of the addr,VA=%x,",reg_addr);
	printk("\n----------------------------------------------------------------------");
	
	printk("\n    (0x%x)",__raw_readl(reg_addr));
	printk("	  (0x%x)",__raw_readl(reg_addr+4));
	printk("	  (0x%x)",__raw_readl(reg_addr+8));
	printk("	  (0x%x)",__raw_readl(reg_addr+0xc));

	printk("\n    (0x%x)",__raw_readl(reg_addr+0x10));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x14));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x18));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x1c));

	printk("\n    (0x%x)",__raw_readl(reg_addr+0x20));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x24));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x28));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x2c));
	
	printk("\n    (0x%x)",__raw_readl(reg_addr+0x30));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x34));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x38));
	printk("	  (0x%x)",__raw_readl(reg_addr+0x3c));

	printk("\n----------------------------------------------------------------------");

}

void printADCValue(void)
{

	printk ("\nADC CH0=%d",g_adcValue[0]);
	printk ("  ADC CH1=%d",g_adcValue[1]);
	printk ("  ADC CH2=%d",g_adcValue[2]);
	printk ("  ADC CH3=%d",g_adcValue[3]);
	
//	printk("\n	  (0x%x)",__raw_readl(ADC_BASE_ADDR_VA));
//	printk("\n	  (0x%x)",__raw_readl(ADC_BASE_ADDR_VA+0x04));
//	printk("\n	  (0x%x)",__raw_readl(ADC_BASE_ADDR_VA+0x08));
//	see_ioaddr(SCU_BASE_ADDR_VA);
	
	


}

