/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)	2004 BY ROCK-CHIP FUZHOU
				--	ALL RIGHTS RESERVED  --

File Name:  drivers_delay.c
Author:     XUESHAN LIN
Created:    7th JAN 2009
Modified:
Revision:   1.00
$Log: drivers_delay.c,v $
Revision 1.2  2009/03/05 12:37:16  hxy
添加CVS版本自动注释

********************************************************************************
********************************************************************************/
#define     IN_DELAY
#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
//#define     DELAY_ARM_FREQ      50

extern 	uint32 PLLGetArmFreq(void);

#define ASM_LOOP_INSTRUCTION_NUM     4
#define ASM_LOOP_PER_US    (PLLGetArmFreq()/ASM_LOOP_INSTRUCTION_NUM)

/***************************************************************************
函数描述:延时
入口参数:cycle数
出口参数:
调用函数:
***************************************************************************/
void DRVDelayCyc(uint32 count)
{
    count/=4;           //因为每条while循环需要4个CYC, 假设MEM是0等待
    while (count--)
        ;
}

/***************************************************************************
函数描述:延时
入口参数:us数
出口参数:
调用函数:
***************************************************************************/
void DRVDelayUs(uint32 count)
{
    unsigned int tmp;

    tmp = count * ASM_LOOP_PER_US;

    while (--tmp) {}
//    while (count--)
//        DRVDelayCyc(DELAY_ARM_FREQ);
}

/***************************************************************************
函数描述:延时
入口参数:ms数
出口参数:
调用函数:
***************************************************************************/
void DRVDelayMs(uint32 count)
{
    while (count--)
        DRVDelayUs(1000);
}

/***************************************************************************
函数描述:延时
入口参数:s数
出口参数:
调用函数:
***************************************************************************/
void DRVDelayS(uint32 count)
{
    while (count--)
        DRVDelayMs(1000);
}
void DelayMs_nops(uint32 count)
{
    DRVDelayMs(count);
}
void USDELAY(uint32 count)
{
    DRVDelayUs(count);
}
