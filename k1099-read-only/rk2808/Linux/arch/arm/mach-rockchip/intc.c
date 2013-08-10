/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File	:	intc.c
Desc	:	ARM�жϿ�������������
Author	:	yangkai
Date	:	2008-11-05
Notes	:
$Log: intc.c,v $
Revision 1.3  2009/03/09 06:25:35  hcy
(hcy)�ļ�ϵͳ�޸��˿���mountʵ�֣���Ӧ�Ŀ����������޸ģ��ļ�ϵͳ��mount����ǰ��������IO_CTL_GET_MEDIUM_STATUS�����Ϊ�ɿ�����FS_MountDevice��ʵ��mount������ڲ忨����ʱ��Ҫ�����޸ġ�
(yk)����clock����

Revision 1.2  2009/03/05 12:37:14  hxy
���CVS�汾�Զ�ע��

********************************************************************/
#define IN_INTC

#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/intc.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/iomux.h>


#include <asm/io.h>

#define g_grfReg		((pGRF_REG)(REG_FILE_BASE_ADDR_VA))
//for linux ,please modify
#ifdef	ROCKCHIP_DEBUG
#define Assert (msg...)  printk(KERN_DEBUG," INTC_DEBUG: ",msg)
#else
#define Assert(msg...)
#endif


//static pINTC_REG g_intcReg = (pINTC_REG)(INTC_BASE_ADDR_VA);

//static pGRF_REG g_grfReg	= (pGRF_REG)(REG_FILE_BASE_ADDR_VA);


/*----------------------------------------------------------------------
Name	: IRQRegISR
Desc	: ע���жϴ����̣߳����жϴ����߳����жϺŹ�������
Params	: irqNum:�жϺ�
		  Routine:�жϷ������
Return	:
Notes	:
----------------------------------------------------------------------*/
uint32 IRQRegISR(eIRQ_NUM irqNum, pFunc Routine)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    Assert(Routine, "---ERROR---no IRQ routine, IRQ num:", irqNum);

    if(irqNum >= IRQ_MAXNUM)
    {
        return(-1);
    }
    g_irqVectorTable[irqNum] = Routine;

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQSetSysPrio
Desc	: ����IRQ ϵͳ�ж����ȼ�
Params	: sysPrio:ϵͳ�ж����ȼ�����Χ:0~15
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32  IRQSetSysPrio(uint32 sysPrio)
{
    Assert((sysPrio < PRI_MAX), "---ERROR---System Priority level error, sysPrio:", sysPrio);
    if (sysPrio >= PRI_MAX)
    {
        return(-1);
    }

    g_intcReg->IRQ_PLEVEL = sysPrio;

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQSetForce
Desc	: �����IRQ
Params	: irqNum:IRQ�жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQSetForce(eIRQ_NUM irqNum)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);

    if (irqNum < 32)
    {
        g_intcReg->IRQ_INTFORCE_L |= (0x01u << (irqNum));
    }
    else
    {
        g_intcReg->IRQ_INTFORCE_H |= (0x01u << (irqNum  & 0x1f));
    }

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQClrForce
Desc	: ��IRQ FORCE
Params	: irqNum:IRQ�жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQClrForce(eIRQ_NUM irqNum)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    if (irqNum >= IRQ_MAXNUM)
    {
        return(-1);
    }

    if (irqNum < 32)
    {
        g_intcReg->IRQ_INTFORCE_L &= ~(0x01u << (irqNum));
    }
    else
    {
        g_intcReg->IRQ_INTFORCE_H &= ~(0x01u << (irqNum  & 0x1f));
    }

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQGetStatus
Desc	: ��ѯIRQ״̬
Params	:
Return	: 64λIRQ�ж�״̬�Ĵ���ֵ
Notes	: ע�ⷵ��ֵ�ǼĴ���״ֵ̬�������жϺ�, ��n bit����1��ʾ�жϺ�Ϊ
		  N��0~39bit��Ч
----------------------------------------------------------------------*/
uint64 IRQGetStatus(void)
{
    uint64 regStatus = (((uint64)(g_intcReg->IRQ_FINALSTATUS_H)<<32)
                        |(uint64)(g_intcReg->IRQ_FINALSTATUS_L));

    return(regStatus);
}

/*----------------------------------------------------------------------
Name	: IRQGetForceStatus
Desc	: ��ѯ�������IRQ״̬
Params	:
Return	: 64λFORCE IRQ���üĴ���ֵ
Notes	: ע�ⷵ��ֵ�ǼĴ���״ֵ̬�������жϺ�
----------------------------------------------------------------------*/
uint64 IRQGetForceStatus(void)
{
    uint64 regStatus = (((uint64)(g_intcReg->IRQ_INTFORCE_H)<<32)
                        |(uint64)(g_intcReg->IRQ_INTFORCE_L));

    return(regStatus);
}

/*----------------------------------------------------------------------
Name	: IRQSetPrio
Desc	: ����IRQ���ȼ�
Params	: irqNum:IRQ�жϺ�
		  irqPrio:IRQ�ж����ȼ���ȡֵ0~15
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQSetPrio(eIRQ_NUM irqNum,uint32 irqPrio)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    Assert((irqPrio < PRI_MAX), "---ERROR---IRQ priority error:", irqPrio);
    if ((irqNum >= IRQ_MAXNUM)||(irqPrio >= PRI_MAX))
    {
        return(-1);
    }

    g_intcReg->IRQ_PN_OFFSET[irqNum] = irqPrio & 0x0f;

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQMask
Desc	: MASK IRQ interrupt
Params	: irqNum:ҪMASK�жϵ��жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQMask(eIRQ_NUM irqNum)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    if (irqNum >= IRQ_MAXNUM)
    {
        return(-1);
    }

    if (irqNum >= 32)
    {
        g_intcReg->IRQ_INTMASK_H |= (0x01u << (irqNum  & 0x1f));
    }
    else
    {
        g_intcReg->IRQ_INTMASK_L |= (0x01u << irqNum);
    }

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQUnmask
Desc	: UNMASK IRQ interrupt
Params	: irqNum:ҪUNMASK���жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQUnmask(eIRQ_NUM irqNum)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    if (irqNum >= IRQ_MAXNUM)
    {
        return(-1);
    }

    if (irqNum >= 32)
    {
        g_intcReg->IRQ_INTMASK_H &= ~(0x01u << (irqNum  & 0x1f));
    }
    else
    {
        g_intcReg->IRQ_INTMASK_L &= ~(0x01u << irqNum);
    }

	return(0);
}

/*----------------------------------------------------------------------
Name	: IRQEnable
Desc	: IRQ ʹ��
Params	: irqNum:Ҫʹ���жϵ��жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQEnable(eIRQ_NUM irqNum)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    if (irqNum >= IRQ_MAXNUM)
    {
        return(-1);
    }

    if (irqNum >= 32)
    {
        g_intcReg->IRQ_INTEN_H |= (0x01u << (irqNum  & 0x1f));
    }
    else
    {
        g_intcReg->IRQ_INTEN_L |= (0x01u << irqNum);
    }

    IRQUnmask(irqNum);

    return(0);
}

/*----------------------------------------------------------------------
Name	: IRQDisable
Desc	: IRQ ��ֹ
Params	: irqNum:Ҫ��ֹ�жϵ��жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 IRQDisable(eIRQ_NUM irqNum)
{
    Assert((irqNum < IRQ_MAXNUM), "---ERROR---IRQ Number error:", irqNum);
    if (irqNum >= IRQ_MAXNUM)
    {
        return(-1);
    }

    if (irqNum >= 32)
    {
        g_intcReg->IRQ_INTEN_H &= ~(0x01u << (irqNum  & 0x1f));
    }
    else
    {
        g_intcReg->IRQ_INTEN_L &= ~(0x01u << irqNum);
    }

    return(0);
}
/*----------------------------------------------------------------------
Name	: ARMEnableIRQ
Desc	: ARM CPSR��ʹ��IRQ
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void ARMEnableIRQ(void )
{
//    uint32 tmp;

/*
	__asm__ (
        "mrs r0, cpsr"
        "bic r0, r0, #0x80"
        "msr cpsr, r0"
	);
*/

	}

/*----------------------------------------------------------------------
Name	: ARMDisableIRQ
Desc	: ARM CPSR�н�ֹIRQ
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void ARMDisableIRQ(void)
{
//	  uint32 tmp;
/*
	__asm__ (
		"mrs tmp, cpsr"
		"orr tmp, tmp, #0x80"
		"msr cpsr_cxsf, tmp"
	);
*/

	}

/*----------------------------------------------------------------------
Name	: FIQGenInt
Desc	: ����FIQ1�ж�
Params	:
Return	:
Notes	: FIQ1��software interrupt��ͨ������CPU_APB_REG5 bit 1����
----------------------------------------------------------------------*/
void FIQGenFIQ1(void)
{
    /*����CPU_APB_REG5 bit 1*/
    g_grfReg->CPU_APB_REG5 |= 0x02;
}

/*----------------------------------------------------------------------
Name	: FIQGenInt
Desc	: ���FIQ1�ж�
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void FIQClrFIQ1(void)
{
    /*���CPU_APB_REG5 bit 1*/
    g_grfReg->CPU_APB_REG5 &= ~0x02;
}

/*----------------------------------------------------------------------
Name	: FIQSetForce
Desc	: �����FIQ
Params	: fiqNum:FIQ�жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 FIQSetForce(eFIQ_NUM fiqNum)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_intcReg->FIQ_INTFORCE |= (0x01u << (fiqNum));

    return(0);
}

/*----------------------------------------------------------------------
Name	: FIQClrForce
Desc	: ��FIQ FORCE
Params	: fiqNum:FIQ�жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 FIQClrForce(eFIQ_NUM fiqNum)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_intcReg->FIQ_INTFORCE &= ~(0x01u << (fiqNum));

    return(0);
}

/*----------------------------------------------------------------------
Name	: FIQGetStatus
Desc	: ��ѯFIQ״̬
Params	:
Return	: FIQ״̬�Ĵ���ֵ
Notes	: ע�ⷵ��ֵ�ǼĴ���״ֵ̬�������жϺ�
----------------------------------------------------------------------*/
uint32 FIQGetStatus(void)
{
    return (g_intcReg->FIQ_FINALSTATUS);
}

/*----------------------------------------------------------------------
Name	: FIQGetForceStatus
Desc	: ��ѯ�������IRQ״̬
Params	:
Return	: FIQ FORCE״̬�Ĵ���ֵ
Notes	: ע�ⷵ��ֵ�ǼĴ���״ֵ̬�������жϺ�
----------------------------------------------------------------------*/
uint32 FIQGetForceStatus(void)
{
    return (g_intcReg->FIQ_INTFORCE);
}


/*----------------------------------------------------------------------
Name	: FIQMask
Desc	: MASK FIQ interrupt
Params	: fiqNum:ҪMASK�жϵ��жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 FIQMask(eFIQ_NUM fiqNum)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_intcReg->FIQ_INTMASK |= (0x01u << fiqNum);

    return(0);
}

/*----------------------------------------------------------------------
Name	: FIQUnmask
Desc	: UNMASK FIQ interrupt
Params	: fiqNum:ҪUNMASK���жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 FIQUnmask(eFIQ_NUM fiqNum)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_intcReg->FIQ_INTMASK &= ~(0x01u << fiqNum);

    return(0);
}

/*----------------------------------------------------------------------
Name	: FIQEnable
Desc	: fiq ʹ��
Params	: fiqNum:Ҫʹ���жϵ��жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 FIQEnable(eFIQ_NUM fiqNum)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_intcReg->FIQ_INTEN |= (0x01u << fiqNum);
    FIQUnmask(fiqNum);

    return(0);
}

/*----------------------------------------------------------------------
Name	: FIQDisable
Desc	: FIQ ��ֹ
Params	: fiqNum:Ҫ��ֹ�жϵ��жϺ�
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 FIQDisable(eFIQ_NUM fiqNum)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_intcReg->FIQ_INTEN &= ~(0x01u << fiqNum);

    return(0);
}
/*----------------------------------------------------------------------
Name	: ARMEnableFIQ
Desc	: ARM CPSR��ʹ��FIQ
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void ARMEnableFIQ(void)
{
//	  uint32 tmp;
/*
	__asm__ (
		"mrs tmp, cpsr"
		"bic tmp, tmp, #0x40"
		"msr cpsr_c, tmp"
	);

*/	
}

/*----------------------------------------------------------------------
Name	: ARMDisableFIQ
Desc	: ARM CPSR�н�ֹFIQ
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void ARMDisableFIQ(void)
{
 //   uint32 tmp;
/*
	__asm__ (
		"mrs tmp, cpsr"
		"orr tmp, tmp, #0x40"
		"msr cpsr_c, tmp"
	);
*/
	}
/*----------------------------------------------------------------------
Name	:  FIQRegISR
Desc	: ע���жϴ����̣߳����жϴ����߳����жϺŹ�������
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
uint32 FIQRegISR(eFIQ_NUM fiqNum, pFunc Routine)
{
    Assert((fiqNum < FIQ_MAXNUM), "---ERROR---IRQ Number error:", fiqNum);
    Assert(Routine, "---ERROR---no IRQ routine, IRQ num:", fiqNum);
    if (fiqNum >= FIQ_MAXNUM)
    {
        return(-1);
    }

    g_fiqVectorTable[fiqNum] = Routine;

    return(0);
}

/*----------------------------------------------------------------------
Name	: FIQHandler
Desc	: IRQ�жϴ�����
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void FIQHandler(void)
{
    uint32 fiqStat;
    uint32 fiqNum;
    pFunc  Routine;
    fiqStat = FIQGetStatus();
    fiqNum = (fiqStat == 0x01)?0:1;
    if (FIQGetForceStatus() == fiqStat)
    {
        FIQClrForce(fiqNum);
    }


    /*�����жϷ������*/
    Routine = g_fiqVectorTable[fiqNum];
    if (Routine)
    {
        Routine();
	}

}

/*----------------------------------------------------------------------
Name	: INTCInit
Desc	: �жϼĴ�����ʼ������ARM��IRQ FIQ����λ����ֹ�����ж�
Params	:
Return	: 0:�ɹ�,1:��������ʧ��
Notes	:
----------------------------------------------------------------------*/
uint32 INTCInit(void)
{

//    SCUEnableClk(CLK_GATE_INTC);
    g_intcReg->IRQ_INTEN_L = 0;       //disable all irq interrupt
    g_intcReg->IRQ_INTEN_H = 0;
    g_intcReg->IRQ_INTMASK_L = -1;    //mask all irq interrupt
    g_intcReg->IRQ_INTMASK_H = -1;
    g_intcReg->IRQ_INTFORCE_L = 0;
    g_intcReg->IRQ_INTFORCE_H = 0;
    g_intcReg->FIQ_INTEN = 0;         //disable fiq interrupt
    g_intcReg->FIQ_INTMASK = 0x03;    //mask fiq interrupt
    g_intcReg->IRQ_PLEVEL = 0;        //set the irq system priority level,default:0

//	  ARMEnableFIQ();
	ARMDisableFIQ();
	ARMEnableIRQ();

	return(0);
}


