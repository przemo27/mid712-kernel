/***************************************************************
 * /include/asm/arch-rockchip/api_touchscreen.h
 * Copyright	: ROCKCHIP Inc
 * Author 	: WQQ
 * Date		: 2009-04-27
 * **************************************************************/
#ifndef  __RK28_DEVICE_TOUCHSCREEN
#define	 __RK28_DEVICE_TOUCHSCREEN

#define PT2046_START_BIT	(0x80)
#define PT2046_SCALE_X		(0x50)
#define PT2046_SCALE_X_NOIRQ		(0x53)

#define PT2046_SCALE_Y		(0x10)
#define PT2046_SCALE_Y_NOIRQ		(0x13)

#define PT2046_AD_MODE_BIT	(0x80)
#define PT2046_INPUT_MODE_BIT	(0x04)
#define PT2046_POWER_BIT	(0x30)

#define PT2046_INT_SET_INPUT()      GPIOSetPinDirection(GPIOPortE_Pin7, GPIO_IN)
#define PT2046_INT_GET()            GPIOGetPinLevel(GPIOPortE_Pin7)
#define PT2046_INT_CLR()	     GPIOPullUpDown(GPIOPortE_Pin7, GPIOPullUp)

#define PT2046_CS_SET_OUPUT()	    GPIOSetPinDirection(GPIOPortB_Pin4, GPIO_OUT)	
#define PT2046_CS_GET()		    GPIOGetPinLevel(GPIOPortB_Pin4)
#define PT2046_CS_SET()		    GPIOSetPinLevel(GPIOPortB_Pin4,GPIO_LOW)
#define PT2046_CS_CLR()		    GPIOSetPinLevel(GPIOPortB_Pin4,GPIO_HIGH)	

#define PT2046_SDO_SET_OUPUT()      GPIOSetPinDirection(GPIOPortB_Pin6, GPIO_OUT)
#define PT2046_SDO_SET()            GPIOSetPinLevel(GPIOPortB_Pin6, GPIO_HIGH)
#define PT2046_SDO_CLR()            GPIOSetPinLevel(GPIOPortB_Pin6, GPIO_LOW)

#define PT2046_SDI_SET_INPUT()      GPIOSetPinDirection(GPIOPortB_Pin7, GPIO_IN)
#define PT2046_SDI_GET()            GPIOGetPinLevel(GPIOPortB_Pin7)

#define PT2046_CLK_SET_OUPUT()      GPIOSetPinDirection(GPIOPortB_Pin5, GPIO_OUT)
#define PT2046_CLK_SET()            GPIOSetPinLevel(GPIOPortB_Pin5, GPIO_HIGH)
#define PT2046_CLK_CLR()            GPIOSetPinLevel(GPIOPortB_Pin5, GPIO_LOW)




#endif 
