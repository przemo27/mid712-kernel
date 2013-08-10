/*
 * linux/drivers/input/keyboard/rk28_ir.c
 *
 * Driver for the rk28 IR keyboard controller.
 *
 * Created: 2010-1-5
 * Author:	DQZ <dqz@rockchip.com>
 *
 * This driver program support to IR key which use for rk28 chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/hardware.h>
/*
 * Keypad Controller registers
 */
 //#define RK28_PRINT
#include <asm/arch/rk28_debug.h>
#include "Rk28_ir.h"


/* Debug */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
extern int pm_sleep_status ;
static int irq_record = 0;

#define KEY_PHYS_NAME	"rk28_AD_button/input0"
//ADC Registers
typedef  struct tagADC_keyst
{
	unsigned int adc_value;
	unsigned int adc_keycode;
}ADC_keyst,*pADC_keyst;




//key code tab
static unsigned char initkey_code[ ] = 
{
	AD1KEY1, AD1KEY2,AD1KEY3,AD1KEY4,AD1KEY5,AD1KEY6,
	AD2KEY1, AD2KEY2,AD2KEY3,AD2KEY4,AD2KEY5,ENDCALL,KEY_WAKEUP
};


struct rk28_AD_button_platform_data {
	int x;
	
};

struct rk28_AD_button {
	struct rk28_AD_button_platform_data *pdata;
	struct timer_list timer;

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;
	/* matrix key code map */
	unsigned char keycodes[13];
	/* state row bits of each column scan */
	uint32_t direct_key_state;
	unsigned int direct_key_mask;
	int rotary_rel_code[2];
	int rotary_up_key[2];
	int rotary_down_key[2];
};

 struct rk28_AD_button *prockIR_button;






static int rk28_IR_button_open(struct input_dev *dev)
{
//	struct rk28_AD_button *AD_button = input_get_drvdata(dev);


	return 0;
}

static void rk28_IR_button_close(struct input_dev *dev)
{
//	struct rk28_AD_button *AD_button = input_get_drvdata(dev);

}


#define res_size(res)	((res)->end - (res)->start + 1)

/*----------------------------------------------------------------------
Name : RMCTimeOut
Desc : 遥控器按键弹起超时检测
Params  :
Return  :
Notes   : 连击状态下启动超时计数，如果在超时周期内没有收到新的连击信号
          表示按键已弹起，发送按键弹起消息并回到IDLE状态
----------------------------------------------------------------------*/
void RMCTimeOut(unsigned long data)
{
    uint32 keyParam;
    del_timer(&prockIR_button->timer);
    g_rmcInfo.timerId = NULL;
	if(g_rmcInfo.press == 1)
	{
	//	printk("key time out!!!\n");
		input_report_key(prockIR_button->input_dev,g_rmcInfo.scanCode,0);
		input_sync(prockIR_button->input_dev);//key release
	}
	g_rmcInfo.press = 0;
    g_rmcInfo.state = RMC_IDLE;
}

/*----------------------------------------------------------------------
Name : RMCDecode
Desc : 遥控器解码
Params  :
Return  :
Notes   : 将接收的遥控数据转换为系统可识别的按键编码
----------------------------------------------------------------------*/
void RMCDecode(void)
{
    uint32 i;
    uint16 keyData = (g_rmcInfo.data >> 8) & 0x0ff;
	DBG("ReMOT DOWN=%d\n",keyData);
	
    pRMCKEY_TRANSFER keyTrans = g_rmcKeyBoard[g_rmcInfo.keybdNum].keyTrans;
    for (i = 0; i < g_rmcKeyBoard[g_rmcInfo.keybdNum].count; i ++)
    {
        if (keyData == keyTrans[i].keycode)
        {
            g_rmcInfo.scanCode = keyTrans[i].scanCode;
		//	printk("the scanCode is %x!!!\n",g_rmcInfo.scanCode);
            return;
        }
    }
	g_rmcInfo.scanCode = 0xFFEE;//erro code;
	return;
}

/*----------------------------------------------------------------------
Name : RMCPreLoad
Desc : 处理遥控器引导码阶段
Params  :
Return  :
Notes   : 如果接收到为引导码则进入接收用户码状态，否则进入IDLE状态
----------------------------------------------------------------------*/
void RMCPreLoad(uint32 period)
{
    if ((TIME_PRE_MIN < period) && (period < TIME_PRE_MAX))
    {
        g_rmcInfo.data = 0;
        g_rmcInfo.count = 0;
        g_rmcInfo.keybdNum = 0x0ff;
        g_rmcInfo.state = RMC_USERCODE;
    }
    else
    {
    	g_rmcInfo.data = 0;
        g_rmcInfo.count = 0;
        g_rmcInfo.keybdNum = 0x0ff;
        g_rmcInfo.state = RMC_IDLE;
    }
}

/*----------------------------------------------------------------------
Name : RMCUserCode
Desc : 处理遥控器接收用户码阶段
Params  :
Return  :
Notes   : 如果接收到为可识别用户码则进入接入接收数据状态；
          否则进入IDLE状态
----------------------------------------------------------------------*/
void RMCUserCode(uint32 period)
{
    uint32 i;
    g_rmcInfo.data <<= 1;
    g_rmcInfo.count ++;
    if ((TIME_BIT1_MIN < period) && (period < TIME_BIT1_MAX))
    {
        g_rmcInfo.data |= 0x01;
    }
	//DBG("ReMOT usercode=%d\n",g_rmcInfo.data);

    if (g_rmcInfo.count == 0x10)//16 bit user code
    {
//    	printk("the usercode is %d!!!\n",g_rmcInfo.data);	
        for (i = 0; i < ARRSIZE(g_rmcKeyBoard); i++)
        {
            if (g_rmcInfo.data == g_rmcKeyBoard[i].usercode)
            {
                g_rmcInfo.keybdNum = i;
                break;
            }
        }
        if (g_rmcInfo.keybdNum == 0x0ff)//user code error
        {
            g_rmcInfo.state = RMC_IDLE;
        }
        else
        {
    //    printk("the keybdNum is %d!!!\n",g_rmcInfo.keybdNum);	
            g_rmcInfo.state = RMC_GETDATA;
            g_rmcInfo.count = 0;
        }
       // g_rmcInfo.state = RMC_GETDATA;
       // g_rmcInfo.count = 0;
    }
}

/*----------------------------------------------------------------------
Name : RMCGetData
Desc : 处理遥控器接收数据阶段
Params  :
Return  :
Notes   : 如果接收到正确数据则进入接入连击状态；
          否则进入IDLE状态
----------------------------------------------------------------------*/
void RMCGetData(uint32 period)
{
    uint32 keyParam;
    g_rmcInfo.count ++;
    g_rmcInfo.data <<= 1;
    if ((TIME_BIT1_MIN < period) && (period < TIME_BIT1_MAX))
    {
        g_rmcInfo.data |= 0x01;
    }
    if (g_rmcInfo.count == 0x10)//16 bit data
    {
   // printk("the datacode is %d!!!\n",g_rmcInfo.data);
        if ((g_rmcInfo.data&0x0ff) == ((~g_rmcInfo.data >> 8)&0x0ff))
        {
        //	printk("the datacode is %d!!!\n",g_rmcInfo.data);
            RMCDecode();
			if(g_rmcInfo.scanCode == 0xFFEE)
			{
				g_rmcInfo.state = RMC_IDLE;
				return;
			}
			//printk("the scandata is %d!!!\n",g_rmcInfo.data);
            g_rmcInfo.press = 1;
          //  keyParam = (1 << 16) | g_rmcInfo.scanCode;
			input_report_key(prockIR_button->input_dev,g_rmcInfo.scanCode,1);
			input_sync(prockIR_button->input_dev);//key press
            g_rmcInfo.state = RMC_SEQUENCE;
        }
        else
        {
            g_rmcInfo.state = RMC_IDLE;
        }
    }
	return;
}

/*----------------------------------------------------------------------
Name : RMCSeq
Desc : 处理遥控器连击阶段
Params  :
Return  :
Notes   : 连击阶段波形如下，由两部分组成
          _________      ___
         | 9ms     |2.25|.56|                                        |
                   `----`   `----------------------------------------`
         |<---------------------------108ms------------------------->|
----------------------------------------------------------------------*/
void RMCSeq(uint32 period)
{
    uint32 keyParam;
    if ((TIME_RPT_MIN < period) && (period < TIME_RPT_MAX))
    {

    }
    else if ((TIME_SEQ_MIN < period) && (period < TIME_SEQ_MAX))
    {
		del_timer(&prockIR_button->timer);
		g_rmcInfo.press = 1;
  		input_report_key(prockIR_button->input_dev,g_rmcInfo.scanCode,1);
		input_sync(prockIR_button->input_dev);//key press
        //g_rmcInfo.timerId = RockStartTimerEx(50, RMCTimeOut, NULL, INVALID_PHANDLE);
	   	setup_timer(&prockIR_button->timer, RMCTimeOut, NULL);
		prockIR_button->timer.expires  = jiffies + 50;
		add_timer(&prockIR_button->timer);
    }
}

/*----------------------------------------------------------------------
Name : RMCHandler
Desc : 遥控器中断处理函数
Params  :
Return  :
Notes   : 遥控器编码流程:
          引导码->16bit用户码->8bit数据码->8bit数据反码->连击码
----------------------------------------------------------------------*/
static irqreturn_t RMCHandler(int irq, void *dev_id)
{
    uint32 timeofday;
    uint32 period;
	struct timeval ts;

	do_gettimeofday(&ts);
	//period = (uint32)__get_nsec_offset();

	timeofday = ts.tv_sec * 1000000 + ts.tv_usec;
    period = timeofday - g_rmcInfo.lastTime;
	
//	++g_rmcInfo.state;
//	if(g_rmcInfo.state > 30)
//	printk("the period is %d!!!\n",period);	
//	printk("the status is %d!!!\n",g_rmcInfo.state);
  if( pm_sleep_status == 1)
  		irq_record++;

  if( pm_sleep_status == 1 && irq_record > 30)
    {
    	input_report_key(prockIR_button->input_dev,AD1KEY5,1);
    	input_sync(prockIR_button->input_dev);
    	input_report_key(prockIR_button->input_dev,AD1KEY5,0);
    	input_sync(prockIR_button->input_dev);
		irq_record = 0;
	//printk("\n%s^^level 2^^Wake Up ^^^^^!!\n",__FUNCTION__);

	 return IRQ_HANDLED;	
    }	
    switch (g_rmcInfo.state)
    {
        case RMC_IDLE:
			if(g_rmcInfo.timerId == TRUE)
			{
				del_timer(&prockIR_button->timer);
				g_rmcInfo.timerId = NULL;
			}
			setup_timer(&prockIR_button->timer, RMCTimeOut, NULL);
     		prockIR_button->timer.expires  = jiffies + 24;
			add_timer(&prockIR_button->timer);
            g_rmcInfo.timerId = TRUE;
            g_rmcInfo.state = RMC_PRELOAD;
            break;
        case RMC_PRELOAD:
            RMCPreLoad(period);
            break;
        case RMC_USERCODE:
            RMCUserCode(period);
            break;
        case RMC_GETDATA:
            RMCGetData(period);
            break;
        case RMC_SEQUENCE:
            RMCSeq(period);
            break;
        default:
            break;
    }
      
    g_rmcInfo.lastTime = timeofday;
 
	 return IRQ_HANDLED;
}

static int __devinit rk28_IR_button_probe(struct platform_device *pdev)
{
	struct rk28_AD_button *IR_button;
	struct input_dev *input_dev;
	int  error,i;
	
	//printk("request rk28_IR key IRQ\n");
	IR_button = kzalloc(sizeof(struct rk28_AD_button), GFP_KERNEL);
	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev || !IR_button) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed1;
	}
	
    g_rmcInfo.state = 0;
    g_rmcInfo.timerId = 0;
    g_rmcInfo.press = 0;

	memcpy(IR_button->keycodes, initkey_code, sizeof(IR_button->keycodes));
	input_dev->name = pdev->name;
	input_dev->open = rk28_IR_button_open;
	input_dev->close = rk28_IR_button_close;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycode = IR_button->keycodes;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(initkey_code);
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	IR_button->input_dev = input_dev;
	input_set_drvdata(input_dev, IR_button);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;

	platform_set_drvdata(pdev, IR_button);

	prockIR_button=IR_button;

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed2;
	}

	//setup_timer(&IR_button->timer, rk28_adkeyscan_timer, (unsigned long)IR_button);
	//IR_button->timer.expires  = jiffies + 3;
	//add_timer(&IR_button->timer);

	GPIOSetPinDirection(GPIOPortE_Pin1,GPIO_IN);
//	GPIOPullUpDown(GPIOPortE_Pin1,GPIOPullUp);
	GPIOSetPinLevel(GPIOPortE_Pin1,GPIO_HIGH);
    error = request_gpio_irq(GPIOPortE_Pin1,RMCHandler,GPIOEdgelRising,input_dev);
	//printk("request IR key IRQ\n");
	if(error)
	{
		printk("unable to request recover IR key IRQ\n");
		goto failed2;
	}	 
    
	return 0;

failed2:
	input_unregister_device(IR_button->input_dev);
	platform_set_drvdata(pdev, NULL);	
failed1:
	input_free_device(input_dev);
	kfree(IR_button);
	return error;
}

static int __devexit rk28_IR_button_remove(struct platform_device *pdev)
{
	struct rk28_AD_button *IR_button = platform_get_drvdata(pdev);

	input_unregister_device(IR_button->input_dev);
	input_free_device(IR_button->input_dev);
	kfree(IR_button);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver rk28_IR_button_driver = {
	.probe		= rk28_IR_button_probe,
	.remove 	= __devexit_p(rk28_IR_button_remove),
	.driver 	= {
		.name	= "rk28_IR_button",
		.owner	= THIS_MODULE,
	},

};

 int __init rk28_IR_button_init(void)
{
	return platform_driver_register(&rk28_IR_button_driver);
}

static void __exit rk28_IR_button_exit(void)
{
	platform_driver_unregister(&rk28_IR_button_driver);
}

fs_initcall(rk28_IR_button_init);
module_exit(rk28_IR_button_exit);

MODULE_DESCRIPTION("rk28 ir button Controller Driver");
MODULE_LICENSE("GPL");
