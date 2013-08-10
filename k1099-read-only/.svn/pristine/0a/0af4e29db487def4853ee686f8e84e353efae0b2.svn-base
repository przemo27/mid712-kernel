/******************************************************************/
/*   Copyright (C) 2008 ROCK-CHIPS FUZHOU . All Rights Reserved.  */
/*******************************************************************
File    :  gpio.h
Desc    :  定义gpio的寄存器结构体\寄存器位的宏定义\接口函数

Author  : 
Date    : 2008-11-20
Modified:
Revision:           1.00
$Log: gpio.h,v $
*********************************************************************/
#ifndef _DRIVER_GPIO_H_
#define _DRIVER_GPIO_H_
#include "typedef.h"
/*********************************************************************
 ENUMERATIONS AND STRUCTURES
*********************************************************************/
typedef enum eGPIOPinLevel
{
	GPIO_LOW=0,
	GPIO_HIGH
}eGPIOPinLevel_t;

typedef enum eGPIOPinDirection
{
	GPIO_IN=0,
	GPIO_OUT
}eGPIOPinDirection_t;

// Constants for GPIO ports
typedef enum eGPIOPORT
{
	GPIOPortA=0,
	GPIOPortB,
	GPIOPortC,
	GPIOPortD,
	GPIOPortE,
	GPIOPortF,
	GPIOPortG,
	GPIOPortH,
	GPIOPORTLast
}eGPIOPORT_t;

typedef enum GPIOPullType {
	GPIONormal,
	GPIOPullUp,
	GPIOPullDown,
	GPIONOInit
}eGPIOPullType_t;

typedef enum GPIOIntType {
	GPIOLevelLow=0,
	GPIOLevelHigh,	 
	GPIOEdgelFalling,
	GPIOEdgelRising
}eGPIOIntType_t;

typedef enum eGPIOPinNum
{
	GPIOPortA_Pin0=0,
	GPIOPortA_Pin1,
	GPIOPortA_Pin2,
	GPIOPortA_Pin3,
	GPIOPortA_Pin4,
	GPIOPortA_Pin5,
	GPIOPortA_Pin6,
	GPIOPortA_Pin7,
	GPIOPortB_Pin0,
	GPIOPortB_Pin1,
	GPIOPortB_Pin2,
	GPIOPortB_Pin3,
	GPIOPortB_Pin4,
	GPIOPortB_Pin5,
	GPIOPortB_Pin6,
	GPIOPortB_Pin7,
	GPIOPortC_Pin0,
	GPIOPortC_Pin1,
	GPIOPortC_Pin2,
	GPIOPortC_Pin3,
	GPIOPortC_Pin4,
	GPIOPortC_Pin5,
	GPIOPortC_Pin6,
	GPIOPortC_Pin7,
	GPIOPortD_Pin0,
	GPIOPortD_Pin1,
	GPIOPortD_Pin2,
	GPIOPortD_Pin3,
	GPIOPortD_Pin4,
	GPIOPortD_Pin5,
	GPIOPortD_Pin6,
	GPIOPortD_Pin7,
	GPIOPortE_Pin0,
	GPIOPortE_Pin1,
	GPIOPortE_Pin2,
	GPIOPortE_Pin3,
	GPIOPortE_Pin4,
	GPIOPortE_Pin5,
	GPIOPortE_Pin6,
	GPIOPortE_Pin7,
	GPIOPortF_Pin0,
	GPIOPortF_Pin1,
	GPIOPortF_Pin2,
	GPIOPortF_Pin3,
	GPIOPortF_Pin4,
	GPIOPortF_Pin5,
	GPIOPortF_Pin6,
	GPIOPortF_Pin7,
	GPIOPortG_Pin0,
	GPIOPortG_Pin1,
	GPIOPortG_Pin2,
	GPIOPortG_Pin3,
	GPIOPortG_Pin4,
	GPIOPortG_Pin5,
	GPIOPortG_Pin6,
	GPIOPortG_Pin7,
	GPIOPortH_Pin0,
	GPIOPortH_Pin1,
	GPIOPortH_Pin2,
	GPIOPortH_Pin3,
	GPIOPortH_Pin4,
	GPIOPortH_Pin5,
	GPIOPortH_Pin6,
	GPIOPortH_Pin7,
	GPIOPinNumLast		  // for init config cycle num
}eGPIOPinNum_t;

extern int32 GPIOSetPinDirection(eGPIOPinNum_t GPIOPinNum, eGPIOPinDirection_t direction);
extern int32 GPIOGetPinDirection(eGPIOPinNum_t GPIOPinNum);
extern int32 GPIOSetPinLevel(eGPIOPinNum_t GPIOPinNum, eGPIOPinLevel_t level);
extern int32 GPIOGetPinLevel(eGPIOPinNum_t GPIOPinNum);
extern int32 GPIOEnableIntr(eGPIOPinNum_t GPIOPinNum);
extern int32 GPIODisableIntr(eGPIOPinNum_t GPIOPinNum);
extern int32 GPIOPullUpDown(eGPIOPinNum_t GPIOPinNum, eGPIOPullType_t GPIOPullUpDown);
extern int32 GPIOSetIntrType(eGPIOPinNum_t GPIOPinNum, eGPIOIntType_t IntType);
extern int32 GPIOIRQRegISR(eGPIOPinNum_t GPIOPinNum, pFunc Routine, eGPIOIntType_t IntType,void *dev_id);
extern int32 GPIOClrearInmarkIntr(eGPIOPinNum_t GPIOPinNum);
extern int32 GPIOInmarkIntr(eGPIOPinNum_t GPIOPinNum);

extern int gpio_direction_input(unsigned gpio);
extern int gpio_direction_output(unsigned gpio, int value);

extern int32 GPIOClearIntr(eGPIOPinNum_t GPIOPinNum);

#undef	EXT
#ifdef	IN_API_DRIVER_GPIO
#define EXT
#else
#define EXT 	extern
#endif

typedef struct
{
	pFuncIntr gpio_vector;
	void *gpio_devid;
}GPIO_PDATA;


#ifdef	IN_API_DRIVER_GPIO
GPIO_PDATA g_gpioVectorTable0[8]={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}}; 
GPIO_PDATA g_gpioVectorTable1[8]={{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};	
#else
EXT GPIO_PDATA g_gpioVectorTable0[8]; 
EXT GPIO_PDATA g_gpioVectorTable1[8];
#endif


/*********************************************************************
 ENUMERATIONS AND STRUCTURES
*********************************************************************/
//GPIO Registers
typedef volatile struct tagGPIO_STRUCT
{
    uint32 GPIO_SWPORTA_DR;
    uint32 GPIO_SWPORTA_DDR;
    uint32 RESERVED1;
    uint32 GPIO_SWPORTB_DR;
    uint32 GPIO_SWPORTB_DDR;
    uint32 RESERVED2;
    uint32 GPIO_SWPORTC_DR;
    uint32 GPIO_SWPORTC_DDR;
    uint32 RESERVED3;
    uint32 GPIO_SWPORTD_DR;
    uint32 GPIO_SWPORTD_DDR;
    uint32 RESERVED4;
    uint32 GPIO_INTEN;
    uint32 GPIO_INTMASK;
    uint32 GPIO_INTTYPE_LEVEL;
    uint32 GPIO_INT_POLARITY;
    uint32 GPIO_INT_STATUS;
    uint32 GPIO_INT_RAWSTATUS;
    uint32 GPIO_DEBOUNCE;
    uint32 GPIO_PORTS_EOI;
    uint32 GPIO_EXT_PORTA;
    uint32 GPIO_EXT_PORTB;
    uint32 GPIO_EXT_PORTC;
    uint32 GPIO_EXT_PORTD;
    uint32 GPIO_LS_SYNC;
}GPIO_REG,*pGPIO_REG;

#ifdef CONFIG_RK28_I2C_GPIO_EXPANDERS
typedef enum ePCA955XPinNum
{
	PCA955X_Pin0=0,
	PCA955X_Pin1,
	PCA955X_Pin2,
	PCA955X_Pin3,
	PCA955X_Pin4,
	PCA955X_Pin5,
	PCA955X_Pin6,
	PCA955X_Pin7,
	PCA955X_Pin10,
	PCA955X_Pin11,
	PCA955X_Pin12,
	PCA955X_Pin13,
	PCA955X_Pin14,
	PCA955X_Pin15,
	PCA955X_Pin16,
	PCA955X_Pin17
}ePCA955XPinNum_t;

extern int pca955x_gpio_direction_input(unsigned off);
extern int pca955x_gpio_direction_output( int off,int val);
extern int pca955x_gpio_get_value(unsigned off);
extern int pca955x_gpio_set_value( unsigned off, int val);
/*#else
#define pca955x_gpio_direction_input(a) do{}while(0)
#define pca955x_gpio_direction_output(a,b) do{}while(0)
#define pca955x_gpio_get_value(a)  do{}while(0)
#define pca955x_gpio_set_value(a,b) do{}while(0)*/
#endif

#ifndef ARCH_NR_GPIOS
#define ARCH_NR_GPIOS		64
#endif
#define ROCK_NR_GPIOS		64

extern int32 request_gpio_irq(unsigned int gpio, pFunc Routine, eGPIOIntType_t IntType,void *dev_id);
extern int32 free_gpio_irq(unsigned int gpio);
extern void gpio_irq_disable(unsigned gpio);
extern void gpio_irq_enable(unsigned gpio);
extern void __gpio_set(unsigned gpio, int value);//nzy add

#endif
