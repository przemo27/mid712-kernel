/*******************************************************************/
/*	  Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.			  */
/*******************************************************************
File		:	   gpio.c
Desc		:	  GPIO 的驱动相关程序
Author		:	  
Date		:	   2008-11-11
Notes		:
$Log: gpio.c,v $
Revision 0.00
********************************************************************/
#define IN_API_DRIVER_GPIO

#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>

#include <asm/io.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>

#include <asm/arch/rk28_irqs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>

#include <asm/mach/irq.h>

static DEFINE_SPINLOCK(gpio_lock);
static DECLARE_BITMAP(gpio_in_use, ROCK_NR_GPIOS);

static pGPIO_REG pheadGpio = (pGPIO_REG)(GPIO0_BASE_ADDR_VA);


/*----------------------------------------------------------------------
Name	  : GPIOSetPinDirection(eGPIOPinNum_t GPIOPinNum, eGPIOPinDirection_t direction)
Desc	  : 设置端口方向，PA、PE默认为Debounce方式
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
							direction:输入方向
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOSetPinDirection(eGPIOPinNum_t GPIOPinNum, eGPIOPinDirection_t direction)
{
	uint8 gpioPortNum;
	uint8 gpioPinNum;  
	if(GPIOPinNum > 31)
	{
		pheadGpio = (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	gpioPortNum = GPIOPinNum/8;
	gpioPinNum	= GPIOPinNum%8;
	switch ( gpioPortNum )
	{
		case GPIOPortA:
            pheadGpio->GPIO_SWPORTA_DDR = (pheadGpio->GPIO_SWPORTA_DDR & (~(1ul<<gpioPinNum)))| (direction << gpioPinNum);
            pheadGpio->GPIO_DEBOUNCE = pheadGpio->GPIO_DEBOUNCE | (1ul << GPIOPinNum);
			break;
		case GPIOPortB:
            pheadGpio->GPIO_SWPORTB_DDR = (pheadGpio->GPIO_SWPORTB_DDR & (~(1ul<<gpioPinNum)))| (direction << gpioPinNum);
			break;
		case GPIOPortC:
            pheadGpio->GPIO_SWPORTC_DDR = (pheadGpio->GPIO_SWPORTC_DDR & (~(1ul<<gpioPinNum)))| (direction << gpioPinNum);
		 break;
		case GPIOPortD:
            pheadGpio->GPIO_SWPORTD_DDR = (pheadGpio->GPIO_SWPORTD_DDR & (~(1ul<<gpioPinNum)))| (direction << gpioPinNum);
			break;
		default:
			return(-1);
	}
	return(0);
}


/*----------------------------------------------------------------------
Name	  : GPIOGetPinDirection(eGPIOPinNum_t GPIOPinNum)
Desc	  : 读取端口方向
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :1 -- 输出方向
		   0 -- 输入方向
		   失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOGetPinDirection(eGPIOPinNum_t GPIOPinNum)
{
	uint8 gpioPortNum;
	uint8 gpioPinNum;
	uint32 temp;
	
	if(GPIOPinNum > 31)
	{
		pheadGpio = (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	gpioPortNum = GPIOPinNum/8;
	gpioPinNum	= GPIOPinNum%8;
	switch ( gpioPortNum )
	{
		case GPIOPortA:
            temp = pheadGpio->GPIO_SWPORTA_DDR;
			break;
		case GPIOPortB:
            temp = pheadGpio->GPIO_SWPORTB_DDR;
			break;
		case GPIOPortC:
            temp = pheadGpio->GPIO_SWPORTC_DDR;
			break;
		case GPIOPortD:
            temp = pheadGpio->GPIO_SWPORTD_DDR;
			break;
		default:
			return(-1);
	}

	return ((temp >> gpioPinNum) & 0x01);
}

/*----------------------------------------------------------------------
Name	  : GPIOSetPinLevel(eGPIOPinNum_t GPIOPinNum,eGPIOPinLevel_t level)
Desc	  : 设置端口电平
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
							level:电平	 0:low 1:high
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOSetPinLevel(eGPIOPinNum_t GPIOPinNum,eGPIOPinLevel_t level)
{
	uint8 gpioPortNum;
	uint8 gpioPinNum;
	
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio= (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	gpioPortNum = GPIOPinNum/8;
	gpioPinNum	= GPIOPinNum%8;
	switch ( gpioPortNum )
    {
        case GPIOPortA:
            pheadGpio->GPIO_SWPORTA_DDR = (pheadGpio->GPIO_SWPORTA_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
            pheadGpio->GPIO_SWPORTA_DR = (pheadGpio->GPIO_SWPORTA_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
            break;
        case GPIOPortB:
            pheadGpio->GPIO_SWPORTB_DDR = (pheadGpio->GPIO_SWPORTB_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
            pheadGpio->GPIO_SWPORTB_DR = (pheadGpio->GPIO_SWPORTB_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
            break;
        case GPIOPortC:
            pheadGpio->GPIO_SWPORTC_DDR = (pheadGpio->GPIO_SWPORTC_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
            pheadGpio->GPIO_SWPORTC_DR = (pheadGpio->GPIO_SWPORTC_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
            break;
        case GPIOPortD:
            pheadGpio->GPIO_SWPORTD_DDR = (pheadGpio->GPIO_SWPORTD_DDR & (~(1ul<<gpioPinNum)))| (1ul << gpioPinNum);
            pheadGpio->GPIO_SWPORTD_DR = (pheadGpio->GPIO_SWPORTD_DR & (~(1ul<<gpioPinNum)))| (level << gpioPinNum);
            break;
        default:
			return(-1);
	}
	return (0);
}

/*----------------------------------------------------------------------
Name	  : GPIOGetPinLevel(eGPIOPinNum_t GPIOPinNum)
Desc	  : 读取端口的电平
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :1 -- 高电平
		   0 -- 低电平
		   失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOGetPinLevel(eGPIOPinNum_t GPIOPinNum)
{
	uint8 gpioPortNum;
	uint8 gpioPinNum;
	uint32 temp;
	
	if(GPIOPinNum > 31)
	{
		pheadGpio = (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	gpioPortNum = GPIOPinNum/8;
	gpioPinNum	= GPIOPinNum%8;
	switch ( gpioPortNum )
	{
        case GPIOPortA:
            temp = pheadGpio->GPIO_EXT_PORTA;
            break;
        case GPIOPortB:
            temp = pheadGpio->GPIO_EXT_PORTB;
            break;
        case GPIOPortC:
            temp = pheadGpio->GPIO_EXT_PORTC;
            break;
        case GPIOPortD:
            temp = pheadGpio->GPIO_EXT_PORTD;
            break;
        default:
			return(-1);
	}

	return ((temp >> gpioPinNum) & 0x01);
}

/*----------------------------------------------------------------------
Name	  : GPIODisableIntr(uint8 GPIOPinNum)
Desc	  : 设置端口不能中断，只有PA、PE才可以中断
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIODisableIntr(eGPIOPinNum_t GPIOPinNum)
{
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	
	if (GPIOPinNum > 7 )
	{
		return (-1);
	}
    pheadGpio->GPIO_INTEN = pheadGpio->GPIO_INTEN & (~(1ul << GPIOPinNum));
	return (0);
}

/*----------------------------------------------------------------------
Name	  : GPIOClearIntr(eGPIOPinNum_t GPIOPinNum)
Desc	  : 清除中断口标志，只有在边沿中断时有效，电平中断无效
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOClearIntr(eGPIOPinNum_t GPIOPinNum)
{	 
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	
	if (GPIOPinNum > 7)
	{
		return (-1);
	}
    pheadGpio->GPIO_PORTS_EOI = pheadGpio->GPIO_PORTS_EOI | (1ul << GPIOPinNum);
	return (0);
}

/*----------------------------------------------------------------------
Name	  : GPIOInmarkIntr(eGPIOPinNum_t GPIOPinNum)
Desc	  : 先屏蔽其中的某个中断，在电平中断中必须要这样做，否则会接着中断
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOInmarkIntr(eGPIOPinNum_t GPIOPinNum)
{
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio= (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	
	if (GPIOPinNum > 7)
	{
		return (-1);
	}
    pheadGpio->GPIO_INTMASK = pheadGpio->GPIO_INTMASK | (1ul << GPIOPinNum);
	return (0);
}

/*----------------------------------------------------------------------
Name	  : GPIOClrearInmarkIntr(eGPIOPinNum_t GPIOPinNum)
Desc	  : 先屏蔽其中的某个中断，在电平中断中必须要这样做，否则会接着中断
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOClrearInmarkIntr(eGPIOPinNum_t GPIOPinNum)
{
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	
	if (GPIOPinNum > 7)
	{
		return (-1);
	}
    pheadGpio->GPIO_INTMASK = pheadGpio->GPIO_INTMASK & (~(1ul << GPIOPinNum));
	return (0);
}

/*----------------------------------------------------------------------
Name	  : GPIOEnableIntr(eGPIOPinNum_t GPIOPinNum)
Desc	  : 设置端口的中断脚，只有PA、PE口才可以中断
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位 						  
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOEnableIntr(eGPIOPinNum_t GPIOPinNum)
{
	if(GPIOPinNum > 31)
	{
		pheadGpio= (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio= (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	
	if (GPIOPinNum > 7 )
	{
		return (-1);
	}
    pheadGpio->GPIO_INTEN = pheadGpio->GPIO_INTEN | (1ul << GPIOPinNum);
	return (0);
}

/*----------------------------------------------------------------------
Name	  : GPIOPullUpDown(eGPIOPinNum_t GPIOPinNum, eGPIOPullType_t GPIOPullUpDown)
Desc	  : 对其中的脚位进行端口上拉或下拉初始化
Params	  : 此处对入口参数逐一说明
							GPIOPinNum:输入脚位
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 GPIOPullUpDown(eGPIOPinNum_t GPIOPinNum, eGPIOPullType_t GPIOPullUpDown)
{
	uint8 gpioPortNum;
	uint8 gpioPinNum;
	uint8 temp=0;
	
	#if (BOARDTYPE == RK2800_FPGA)
	pAPB_REG  pAPBRegStart = (pAPB_REG)RTC_BASE_ADDR_VA;
	#else
	pAPB_REG  pAPBRegStart = (pAPB_REG)REG_FILE_BASE_ADDR_VA;
	#endif
	if (GPIOPinNum > 63)
	{
		return (-1);
	}
	gpioPortNum = GPIOPinNum/8;
	gpioPinNum	= GPIOPinNum%8;
	if(gpioPortNum%2 == 1)
	{
		temp = 16;
	}
    if((gpioPortNum == GPIOPortA) || (gpioPortNum == GPIOPortB))
    {
        pAPBRegStart->GPIO0_AB_PU_CON = (pAPBRegStart->GPIO0_AB_PU_CON & (~((uint32)0x03<<(2*gpioPinNum+temp)))) | ((uint32)GPIOPullUpDown<<(2*gpioPinNum+temp));
    }
    if((gpioPortNum == GPIOPortC) || (gpioPortNum == GPIOPortD))
    {
        pAPBRegStart->GPIO0_CD_PU_CON = (pAPBRegStart->GPIO0_CD_PU_CON & (~((uint32)0x03<<(2*gpioPinNum+temp)))) | ((uint32)GPIOPullUpDown<<(2*gpioPinNum+temp));
    }
    gpioPortNum = gpioPortNum - 4;
    if((gpioPortNum == GPIOPortA) || (gpioPortNum == GPIOPortB))
    {
        pAPBRegStart->GPIO1_AB_PU_CON = (pAPBRegStart->GPIO1_AB_PU_CON & (~((uint32)0x03<<(2*gpioPinNum+temp)))) | ((uint32)GPIOPullUpDown<<(2*gpioPinNum+temp));
    }
    if((gpioPortNum == GPIOPortC) || (gpioPortNum == GPIOPortD))
    {
        pAPBRegStart->GPIO1_CD_PU_CON = (pAPBRegStart->GPIO1_CD_PU_CON & (~((uint32)0x03<<(2*gpioPinNum+temp)))) | ((uint32)GPIOPullUpDown<<(2*gpioPinNum+temp));
    }
	return (0);
}

/*----------------------------------------------------------------------
Name	  : void GPIO0IntHander(int irq, void *dev_id)
Desc	  : GPIO0中断服务程序
Params	  : 无
Return	  :无
----------------------------------------------------------------------*/
static void GPIO0IntHander(int irq, void *dev_id)
{
	uint32 gpioIntStatus0=0;
	uint32 gpioTemp0;
	pGPIO_REG phwHeadGpio0 = (pGPIO_REG)GPIO0_BASE_ADDR_VA;

    gpioTemp0 = phwHeadGpio0->GPIO_INT_STATUS;
	while (gpioTemp0>0)
	{
		gpioTemp0 = gpioTemp0>>1;
		gpioIntStatus0++;
	}
	if (gpioIntStatus0 == 0 )
	{
		return ;
	}
	gpioIntStatus0--;
	if (gpioIntStatus0 < 8)
	{
		/*进入外部中断0服务程序*/
		if (g_gpioVectorTable0[gpioIntStatus0].gpio_vector)
		{
			g_gpioVectorTable0[gpioIntStatus0].gpio_vector(irq, g_gpioVectorTable0[gpioIntStatus0].gpio_devid);
		}
        if (phwHeadGpio0->GPIO_INTTYPE_LEVEL)
		{
			GPIOClearIntr(gpioIntStatus0);
		}
		else
		{
			GPIOClrearInmarkIntr(gpioIntStatus0);
		}
	}
}

/*----------------------------------------------------------------------
Name	  : void GPIO1IntHander()
Desc	  : GPIO1中断服务程序
Params	  : 无
Return	  :无
----------------------------------------------------------------------*/
static void GPIO1IntHander(int irq, void *dev_id)
{
	uint32 gpioIntStatus1=0;
	uint32 gpioTemp1;
	pGPIO_REG phwHeadGpio1 = (pGPIO_REG)GPIO1_BASE_ADDR_VA;
	
  gpioTemp1 = phwHeadGpio1->GPIO_INT_STATUS;
	while (gpioTemp1 > 0)
	{
		gpioTemp1 = gpioTemp1>>1;
		gpioIntStatus1++;
	}
	if (gpioIntStatus1 == 0 )
	{
		return ;
	}
	gpioIntStatus1--;
	if (gpioIntStatus1 < 8)
	{
		/*进入外部中断1服务程序*/
		if (g_gpioVectorTable1[gpioIntStatus1].gpio_vector)
		{
			g_gpioVectorTable1[gpioIntStatus1].gpio_vector(irq,g_gpioVectorTable1[gpioIntStatus1].gpio_devid);
		}
        if (phwHeadGpio1->GPIO_INTTYPE_LEVEL)
		{
			GPIOClearIntr(gpioIntStatus1+32);
		}
		else
		{
			GPIOClrearInmarkIntr(gpioIntStatus1+32);
		}
	}
}

/* 20100309,HSL@RK,function for 
 *  check which gpio wake up the system.
 *  for fast irq handle. CALL AT rk28_pm.c::rk28_pm_enter.
 *  if mult gpio irq,the highest is picked.
 */
#if 0
/*
  *  gpio: 0 for GPIO0 , 1 for GPIO1.
  *  return pin: 0--7 , < 0 : on int.
*/
int gpio_get_int_pin( int gpio )
{	
                int pin = -1;
                uint32  int_status;
                pGPIO_REG phwHeadGpio1;
	if( gpio != 0 )
	{
		phwHeadGpio1 = (pGPIO_REG)GPIO1_BASE_ADDR_VA;
	}
	else
	{
		phwHeadGpio1 = (pGPIO_REG)GPIO0_BASE_ADDR_VA;	
	}
	int_status = phwHeadGpio1->GPIO_INT_STATUS&0xff;
	while ( int_status)
	{
		int_status >>= 1;
		pin++;
	}
	return pin;
}
#endif

/*----------------------------------------------------------------------
Name	: GPIOSetIntrType(eGPIOPinNum_t GPIOPinNum, eGPIOIntType_t IntType)
Desc	: 设置中断类型
Params	: GPIOPinNum:管脚
		  IntType:中断触发类型
Return	:成功还回0，失败还回-1?
Notes	:
----------------------------------------------------------------------*/
int32 GPIOSetIntrType(eGPIOPinNum_t GPIOPinNum, eGPIOIntType_t IntType)
{
	if(GPIOPinNum > 31)
	{
		pheadGpio = (pGPIO_REG)GPIO1_BASE_ADDR_VA;
		GPIOPinNum = GPIOPinNum -32;
	}
	else
	{
		pheadGpio = (pGPIO_REG)GPIO0_BASE_ADDR_VA; 
	}
	
	if (GPIOPinNum > 7)
	{
		return (-1);
	}
	switch ( IntType )
	{
        case GPIOLevelLow:
            pheadGpio->GPIO_INT_POLARITY = pheadGpio->GPIO_INT_POLARITY & (~(1ul << GPIOPinNum));
            pheadGpio->GPIO_INTTYPE_LEVEL = pheadGpio->GPIO_INTTYPE_LEVEL & (~(1ul << GPIOPinNum));
            break;
        case GPIOLevelHigh:
            pheadGpio->GPIO_INTTYPE_LEVEL = pheadGpio->GPIO_INTTYPE_LEVEL & (~(1ul << GPIOPinNum));
            pheadGpio->GPIO_INT_POLARITY = pheadGpio->GPIO_INT_POLARITY | (1ul << GPIOPinNum);
            break;
        case GPIOEdgelFalling:
            pheadGpio->GPIO_INTTYPE_LEVEL = pheadGpio->GPIO_INTTYPE_LEVEL | (1ul << GPIOPinNum);
            pheadGpio->GPIO_INT_POLARITY = pheadGpio->GPIO_INT_POLARITY & (~(1ul << GPIOPinNum));
            break;
        case GPIOEdgelRising:
            pheadGpio->GPIO_INTTYPE_LEVEL = pheadGpio->GPIO_INTTYPE_LEVEL | (1ul << GPIOPinNum);
            pheadGpio->GPIO_INT_POLARITY = pheadGpio->GPIO_INT_POLARITY | (1ul << GPIOPinNum);
            break;
		default:
			return(-1);
	}
	 return(0);
}
/*----------------------------------------------------------------------
Name	: GPIOIRQRegISR(eGPIOPinNum_t GPIOPinNum, int Routine, eGPIOIntType_t IntType)
Desc	: 注册中断处理线程，将中断处理线程与中断号关联起来
Params	: irqNum:中断的脚位
		  Routine:中断服务程序
		  IntType:中断触发类型
Return	:成功还回0，失败还回-1?
Notes	:
----------------------------------------------------------------------*/
int32 GPIOIRQRegISR(eGPIOPinNum_t GPIOPinNum, pFunc Routine, eGPIOIntType_t IntType,void *dev_id)
{
   
	GPIOSetIntrType(GPIOPinNum,IntType);
	if(GPIOPinNum <= GPIOPortA_Pin7) 
	{
		if(g_gpioVectorTable0[GPIOPinNum].gpio_vector) 
			return -1;
		g_gpioVectorTable0[GPIOPinNum].gpio_vector = Routine;
		g_gpioVectorTable0[GPIOPinNum].gpio_devid= dev_id;
		return(0);
	}
	else if((GPIOPinNum >= GPIOPortE_Pin0) && (GPIOPinNum <= GPIOPortE_Pin7))
	{
	
		if(	g_gpioVectorTable1[GPIOPinNum-GPIOPortE_Pin0].gpio_vector) 
			return -1;

		g_gpioVectorTable1[GPIOPinNum-GPIOPortE_Pin0].gpio_vector = Routine;
		g_gpioVectorTable1[GPIOPinNum-GPIOPortE_Pin0].gpio_devid= dev_id;
		return(0);
	}

	return(-1);
}






//irq－GPIO固定映射
int rock28_irq_to_gpio(int irq)
{

	
	if (IRQ_GPIO1==irq)
		return GPIOPortE_Pin7;

	else
		return -1;

}


int gpio_request(unsigned gpio, const char *tag)
{
	if (gpio >= ROCK_NR_GPIOS)
		return -EINVAL;

	if (test_and_set_bit(gpio, gpio_in_use))
		return -EBUSY;
	
	//printk("\n---------------------gpio_request gpio=%d\n",gpio);

	return 0;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	if (gpio >= ROCK_NR_GPIOS)
		return;
	//printk("\n---------------------gpio_free gpio=%d\n",gpio);

	clear_bit(gpio, gpio_in_use);
}


EXPORT_SYMBOL(gpio_free);

/* create a non-inlined version */
static struct gpio_controller *__iomem gpio2controller(unsigned gpio)
{
//	return __gpio_to_controller(gpio);
}

/*
 * Assuming the pin is muxed as a gpio output, set its output value.
 */
void __gpio_set(unsigned gpio, int value)
{
	//struct gpio_controller *__iomem g = gpio2controller(gpio);
//	printk("\n---------------------__gpio_set gpio=%d\n",gpio);

	 GPIOSetPinLevel((eGPIOPinNum_t)gpio,(value ?GPIO_HIGH:GPIO_LOW));

//	__raw_writel(__gpio_mask(gpio), value ? &g->set_data : &g->clr_data);
}
EXPORT_SYMBOL(__gpio_set);


/*
 * Read the pin's value (works even if it's set up as output);
 * returns zero/nonzero.
 *
 * Note that changes are synched to the GPIO clock, so reading values back
 * right after you've set them may give old values.
 */
int __gpio_get(unsigned gpio)
{
	//struct gpio_controller *__iomem g = gpio2controller(gpio);
	//printk("\n---------------------__gpio_get gpio=%d\n",gpio);

	return GPIOGetPinLevel((eGPIOPinNum_t)gpio);

//	return !!(__gpio_mask(gpio) & __raw_readl(&g->in_data));
}
EXPORT_SYMBOL(__gpio_get);


/*--------------------------------------------------------------------------*/

/*
 * board setup code *MUST* set PINMUX0 and PINMUX1 as
 * needed, and enable the GPIO clock.
 */

int gpio_direction_input(unsigned gpio)
{

//	printk("\n---------------------gpio_direction_input gpio=%d\n",gpio);

	spin_lock(&gpio_lock);

	
	 GPIOSetPinDirection((eGPIOPinNum_t) gpio,  GPIO_IN);
	//mask = __gpio_mask(gpio);
	//temp = __raw_readl(&g->dir);
	//temp |= mask;
	//__raw_writel(temp, &g->dir);
	spin_unlock(&gpio_lock);
	
	return 0;
}
EXPORT_SYMBOL(gpio_direction_input);

int gpio_direction_output(unsigned gpio, int value)
{
	//printk("\n---------------------gpio_direction_output gpio=%d\n",gpio);

	spin_lock(&gpio_lock);

	GPIOSetPinDirection((eGPIOPinNum_t) gpio,  GPIO_OUT);

	/*
	mask = __gpio_mask(gpio);
	temp = __raw_readl(&g->dir);
	temp &= ~mask;
	__raw_writel(mask, value ? &g->set_data : &g->clr_data);
	__raw_writel(temp, &g->dir);
	*/
	spin_unlock(&gpio_lock);
	return 0;
}
EXPORT_SYMBOL(gpio_direction_output);

/*
 * We expect irqs will normally be set up as input pins, but they can also be
 * used as output pins ... which is convenient for testing.
 *
 * NOTE:  GPIO0..GPIO7 also have direct INTC hookups, which work in addition
 * to their GPIOBNK0 irq (but with a bit less overhead).  But we don't have
 * a good way to hook those up ...
 *
 * All those INTC hookups (GPIO0..GPIO7 plus five IRQ banks) can also
 * serve as EDMA event triggers.
 */

void gpio_irq_disable(unsigned gpio)
{

	GPIODisableIntr((eGPIOPinNum_t)gpio);
	

}

void gpio_irq_enable(unsigned gpio)
{

	GPIOEnableIntr((eGPIOPinNum_t)gpio);

}

static int gpio_irq_type(unsigned gpio, unsigned trigger)
{

	eGPIOIntType_t IntType;

	if (trigger & (IRQ_TYPE_EDGE_FALLING ))
		IntType=GPIOEdgelFalling;
	else if(trigger & (IRQ_TYPE_EDGE_RISING))
		IntType=GPIOEdgelRising;
	else if(trigger & (IRQ_TYPE_LEVEL_HIGH))
		IntType=GPIOLevelHigh;
	else if(trigger & (IRQ_TYPE_LEVEL_LOW))
		IntType=GPIOLevelLow;
	else
		return -EINVAL;

	
	if(gpio>0)
		return GPIOSetIntrType((eGPIOPinNum_t)gpio, IntType);

	return 0;
}



static void gpio0_irq_handler(int irq, void *dev_id)
{
  
  //printk("\n---------------------gpio0_irq_handler irq=%d,id=%d\n",irq,*((int*)dev_id));
  	unsigned long flags;
 	local_irq_save(flags);				/*save irq status and disable irq*/
	GPIO0IntHander(irq, dev_id);
	local_irq_restore(flags);			/*revert irq status*/
}


static void gpio1_irq_handler(int irq, void *dev_id)
{
 //printk("\n---------------------gpio1_irq_handler irq=%d,id=%d\n",irq,*((int*)dev_id));
 	unsigned long flags;
 	local_irq_save(flags);				/*save irq status and disable irq*/
	GPIO1IntHander(irq, dev_id);
	local_irq_restore(flags);			/*revert irq status*/
}


  


int32 request_gpio_irq(unsigned int gpio, pFunc Routine, eGPIOIntType_t IntType,void *dev_id)
{
  int ret;
  
  gpio_direction_input(gpio);
  //GPIOPullUpDown(gpio,GPIOPullUp);
	ret = GPIOIRQRegISR( (eGPIOPinNum_t) gpio,  Routine,  IntType,dev_id);
  gpio_irq_enable(gpio);
	return  ret;

}


int32 free_gpio_irq(unsigned int gpio)
{

	eGPIOPinNum_t GPIOPinNum=(eGPIOPinNum_t)gpio;


	gpio_irq_disable(GPIOPinNum);
	if(GPIOPinNum <= GPIOPortA_Pin7) 
	{
		g_gpioVectorTable0[GPIOPinNum].gpio_vector = NULL;
		return(0);
	}
	else if((GPIOPinNum >= GPIOPortE_Pin0) && (GPIOPinNum <= GPIOPortE_Pin7))
	{
		g_gpioVectorTable1[GPIOPinNum-GPIOPortE_Pin0].gpio_vector = NULL;
		return(0);
	}

	return	0;

}

static void rock28_GPIO_irq_mask_ack(unsigned int irq)
{

}

static void rock28_GPIO_irq_mask(unsigned int irq)
{

}

static void rock28_GPIO_irq_unmask(unsigned int irq)
{

}

static struct irq_chip rock28_GPIO_irq_chip = {
	.name	= "rockgpio",
	.ack		= rock28_GPIO_irq_mask_ack,
	.mask		= rock28_GPIO_irq_mask,
	.unmask 	= rock28_GPIO_irq_unmask,
};

static int __init rock28_gpio_irq_setup(void)
{

	  unsigned int gpio;

	 /// printk("\n---------------------rock28_gpio_irq_setup\n");

		
		set_irq_chip(IRQ_GPIO0, &rock28_GPIO_irq_chip);
		set_irq_handler(IRQ_GPIO0, gpio0_irq_handler);
		set_irq_flags(IRQ_GPIO0, IRQF_VALID);

		set_irq_chip(IRQ_GPIO1, &rock28_GPIO_irq_chip);
		set_irq_handler(IRQ_GPIO1, gpio1_irq_handler);
		set_irq_flags(IRQ_GPIO1, IRQF_VALID);

		//request_irq(IRQ_GPIO0, gpio0_irq_handler, IRQF_SHARED, 0, 0);
		//request_irq(IRQ_GPIO1, gpio1_irq_handler, IRQF_SHARED, 0, 0);

		for (gpio=0;gpio<=GPIOPortA_Pin7;gpio++)
			{
				GPIODisableIntr((eGPIOPinNum_t)gpio);
			}
		for (gpio=GPIOPortE_Pin0;gpio<=GPIOPortE_Pin7;gpio++)
			{
				GPIODisableIntr((eGPIOPinNum_t)gpio);
			}
		
		IRQEnable(IRQ_GPIO0);
		IRQEnable(IRQ_GPIO1);

	return 0;
}

arch_initcall(rock28_gpio_irq_setup);

