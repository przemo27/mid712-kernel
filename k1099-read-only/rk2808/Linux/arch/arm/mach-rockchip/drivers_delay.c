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
���CVS�汾�Զ�ע��

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
��������:��ʱ
��ڲ���:cycle��
���ڲ���:
���ú���:
***************************************************************************/
void DRVDelayCyc(uint32 count)
{
    count/=4;           //��Ϊÿ��whileѭ����Ҫ4��CYC, ����MEM��0�ȴ�
    while (count--)
        ;
}

/***************************************************************************
��������:��ʱ
��ڲ���:us��
���ڲ���:
���ú���:
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
��������:��ʱ
��ڲ���:ms��
���ڲ���:
���ú���:
***************************************************************************/
void DRVDelayMs(uint32 count)
{
    while (count--)
        DRVDelayUs(1000);
}

/***************************************************************************
��������:��ʱ
��ڲ���:s��
���ڲ���:
���ú���:
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
