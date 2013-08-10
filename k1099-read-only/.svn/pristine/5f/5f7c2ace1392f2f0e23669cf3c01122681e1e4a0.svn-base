/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File	:	intc.c
Desc	:	ARM中断控制器驱动程序
Author	:	yangkai
Date	:	2008-11-05
Notes	:
$Log: intc.c,v $
Revision 1.3  2009/03/09 06:25:35  hcy
(hcy)文件系统修改了卡的mount实现，相应的卡驱动进行修改：文件系统卡mount从以前主动发起IO_CTL_GET_MEDIUM_STATUS命令改为由卡调用FS_MountDevice来实现mount，因此在插卡开机时需要跟着修改。
(yk)更新clock管理

Revision 1.2  2009/03/05 12:37:14  hxy
添加CVS版本自动注释

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
Desc	: 注册中断处理线程，将中断处理线程与中断号关联起来
Params	: irqNum:中断号
		  Routine:中断服务程序
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
Desc	: 设置IRQ 系统中断优先级
Params	: sysPrio:系统中断优先级，范围:0~15
Return	: 0:成功,1:参数错误，失败
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
Desc	: 软件置IRQ
Params	: irqNum:IRQ中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: 清IRQ FORCE
Params	: irqNum:IRQ中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: 查询IRQ状态
Params	:
Return	: 64位IRQ中断状态寄存器值
Notes	: 注意返回值是寄存器状态值，不是中断号, 第n bit被置1表示中断号为
		  N，0~39bit有效
----------------------------------------------------------------------*/
uint64 IRQGetStatus(void)
{
    uint64 regStatus = (((uint64)(g_intcReg->IRQ_FINALSTATUS_H)<<32)
                        |(uint64)(g_intcReg->IRQ_FINALSTATUS_L));

    return(regStatus);
}

/*----------------------------------------------------------------------
Name	: IRQGetForceStatus
Desc	: 查询软件设置IRQ状态
Params	:
Return	: 64位FORCE IRQ设置寄存器值
Notes	: 注意返回值是寄存器状态值，不是中断号
----------------------------------------------------------------------*/
uint64 IRQGetForceStatus(void)
{
    uint64 regStatus = (((uint64)(g_intcReg->IRQ_INTFORCE_H)<<32)
                        |(uint64)(g_intcReg->IRQ_INTFORCE_L));

    return(regStatus);
}

/*----------------------------------------------------------------------
Name	: IRQSetPrio
Desc	: 设置IRQ优先级
Params	: irqNum:IRQ中断号
		  irqPrio:IRQ中断优先级，取值0~15
Return	: 0:成功,1:参数错误，失败
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
Params	: irqNum:要MASK中断的中断号
Return	: 0:成功,1:参数错误，失败
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
Params	: irqNum:要UNMASK的中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: IRQ 使能
Params	: irqNum:要使能中断的中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: IRQ 禁止
Params	: irqNum:要禁止中断的中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: ARM CPSR中使能IRQ
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
Desc	: ARM CPSR中禁止IRQ
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
Desc	: 产生FIQ1中断
Params	:
Return	:
Notes	: FIQ1是software interrupt，通过设置CPU_APB_REG5 bit 1产生
----------------------------------------------------------------------*/
void FIQGenFIQ1(void)
{
    /*设置CPU_APB_REG5 bit 1*/
    g_grfReg->CPU_APB_REG5 |= 0x02;
}

/*----------------------------------------------------------------------
Name	: FIQGenInt
Desc	: 清除FIQ1中断
Params	:
Return	:
Notes	:
----------------------------------------------------------------------*/
void FIQClrFIQ1(void)
{
    /*清除CPU_APB_REG5 bit 1*/
    g_grfReg->CPU_APB_REG5 &= ~0x02;
}

/*----------------------------------------------------------------------
Name	: FIQSetForce
Desc	: 软件置FIQ
Params	: fiqNum:FIQ中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: 清FIQ FORCE
Params	: fiqNum:FIQ中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: 查询FIQ状态
Params	:
Return	: FIQ状态寄存器值
Notes	: 注意返回值是寄存器状态值，不是中断号
----------------------------------------------------------------------*/
uint32 FIQGetStatus(void)
{
    return (g_intcReg->FIQ_FINALSTATUS);
}

/*----------------------------------------------------------------------
Name	: FIQGetForceStatus
Desc	: 查询软件设置IRQ状态
Params	:
Return	: FIQ FORCE状态寄存器值
Notes	: 注意返回值是寄存器状态值，不是中断号
----------------------------------------------------------------------*/
uint32 FIQGetForceStatus(void)
{
    return (g_intcReg->FIQ_INTFORCE);
}


/*----------------------------------------------------------------------
Name	: FIQMask
Desc	: MASK FIQ interrupt
Params	: fiqNum:要MASK中断的中断号
Return	: 0:成功,1:参数错误，失败
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
Params	: fiqNum:要UNMASK的中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: fiq 使能
Params	: fiqNum:要使能中断的中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: FIQ 禁止
Params	: fiqNum:要禁止中断的中断号
Return	: 0:成功,1:参数错误，失败
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
Desc	: ARM CPSR中使能FIQ
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
Desc	: ARM CPSR中禁止FIQ
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
Desc	: 注册中断处理线程，将中断处理线程与中断号关联起来
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
Desc	: IRQ中断处理函数
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


    /*进入中断服务程序*/
    Routine = g_fiqVectorTable[fiqNum];
    if (Routine)
    {
        Routine();
	}

}

/*----------------------------------------------------------------------
Name	: INTCInit
Desc	: 中断寄存器初始化，打开ARM的IRQ FIQ屏蔽位，禁止所有中断
Params	:
Return	: 0:成功,1:参数错误，失败
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


