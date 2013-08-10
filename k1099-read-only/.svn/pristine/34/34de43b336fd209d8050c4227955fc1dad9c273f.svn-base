/*
 *  driver/input/keyboard/rk28_atkeyboard.c
 *
 *  Copyright (C) 2009 rockchip lhh
 *  2009-11-12
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
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
#include <asm/arch/iomux.h>
#include <asm/arch/hardware.h>

MODULE_DESCRIPTION("ASoC rk2808 ps2 key board driver");
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_LICENSE("GPL");

/*
 * Debug
 */
#if 0
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif

#define IOMUX_PIN_NAME       GPIOF5_APWM3_DPWM3_NAME
#define IOMUX_PIN_NAME_DATA  GPIOE_SPI1_SEL_NAME
#define IOMUX_PIN_DIR        IOMUXB_GPIO1_B5
#define IOMUX_PIN_DIR_DATA   IOMUXA_GPIO1_A1237
#define PS2_CLK_PIN          GPIOPortE_Pin0
#define PS2_DATA_PIN         GPIOPortE_Pin2

static unsigned  char g_bit_cnt = 0;
static unsigned  int g_rece_code = 0;
struct timer_list at_key_timer;
static unsigned  char g_key_up_flag = 0; ///1£ºdown 0:up 2:wil up
static unsigned  int g_scancode=0;
/*
 0x47: KP_7     71
 0x48: KP_8     72
 0x49: KP_9     73
 0x62: KP_/     98
 0x4b: KP_4     75
 0x4c: KP_5     76
 0x4d: KP_6     77
 0x37: KP_*     55
 0x4f: KP_1     79
 0x50: KP_2     80
 0x51: KP_3     81
 0x4a: KP_-     74
 0x52: KP_0     82
 0x53: KP_.     83
 0x4e: KP_+     78

 0x67: Up       103
 0x6c: Down     108
 0x69: Left     105
 0x6a: Right    106
 */
static unsigned char rk28_atkeycode[160] = {
	 0,24/*118*/, 22, 30, 38, 37, 46, 54, 61, 62, 70, 69, 78, 85,102, 13,
     21, 29, 36, 45, 44, 53, 60, 67, 68, 77, 84, 91, 90, 20, 28, 27,
     35, 43, 52, 51, 59, 66, 75, 76, 82, 14, 18, 93, 26, 34, 33, 42,
     50, 49, 58, 65, 73, 74, 89,124, 17, 41, 88,  131/*5*/,  6,  4, 12,  3,
     0/*11*/,2,0,81/*1*/,9,119,126,108,32/*117*/,125,123,48 /*107*/,115,56/*116*/,121,105,
     72/*114*/,122,112,113,127, 96, 97,120,  7, 15, 23, 31, 39, 47, 55, 63,
     71, 79, 86, 94,  8, 16, 118/*24*/, 117/*32*/, 40, 107/*48*/, 116/*56*/, 64, 114/*72*/, 80, 87,111,
     19, 25, 10/*57*/, 1/*81*/, 83, 92, 95, 98, 99,100,101,103,104,106,109,110,
     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,0,0,0,0,0,0,0,11,0
};	
static struct input_dev *rk28_at_key_dev;

static inline irqreturn_t rk28_atkey_interrupt(s32 irq, void *dev_id)
{
    unsigned  int g_scancode_bak=0;
    
   // del_timer(&at_key_timer);
   // at_key_timer.expires  = jiffies + msecs_to_jiffies(1000);
    //add_timer(&at_key_timer);
    mod_timer(&at_key_timer,jiffies + msecs_to_jiffies(1000));
    g_rece_code &= 0xfbff; 
    g_rece_code |= (GPIOGetPinLevel(PS2_DATA_PIN)&0x01)<<10;
    if((g_bit_cnt == 0)&&(g_rece_code != 0)){       
    	goto erro_exit;   		
    }
    g_bit_cnt++;	
    if(g_bit_cnt > 0x0a){ ///11 bit code get end
    	g_bit_cnt = 0;
    	if((g_rece_code&0x400) != 0x400){
    		g_rece_code = 0;
    	    goto erro_exit;
    	}
        g_rece_code = (g_rece_code>>1) & 0xff;
        if(g_rece_code == 0xf0){
        	g_key_up_flag = 2;
        	g_rece_code = 0;
        	return IRQ_HANDLED;
        }
        if(g_rece_code == 0xe0){
        	g_rece_code = 0;
        	return IRQ_HANDLED;
        }
        g_scancode_bak = g_scancode;	       
        for(g_scancode=0; g_scancode<160; g_scancode++){
        	if(g_rece_code == rk28_atkeycode[g_scancode])break;
        }	
        if(g_scancode == 160){
        	g_scancode = g_scancode_bak;
        }
        if(g_key_up_flag == 2){
            input_report_key(rk28_at_key_dev, g_scancode, 0); 
            input_sync(rk28_at_key_dev); 
            g_key_up_flag = 0;
            DBG("%s [%d]---g_scancode = %d g_rece_code = %d at key up\n",__FUNCTION__,__LINE__,g_scancode,g_rece_code);
        }else if (g_key_up_flag == 0){
        	input_report_key(rk28_at_key_dev, g_scancode, 1); 
            input_sync(rk28_at_key_dev);
            g_key_up_flag = 1;
            DBG("%s [%d]---g_scancode = %d g_rece_code = %d at key down\n",__FUNCTION__,__LINE__,g_scancode,g_rece_code);
        }else if ((g_key_up_flag == 1)&&(g_scancode != g_scancode_bak)){    
        	input_report_key(rk28_at_key_dev, g_scancode, 1); 
            input_sync(rk28_at_key_dev);
            DBG("%s [%d]---g_scancode = %d g_rece_code = %d at key down\n",__FUNCTION__,__LINE__,g_scancode,g_rece_code);
        }    
        g_rece_code = 0; 
        return IRQ_HANDLED;	
    }	
    g_rece_code = g_rece_code >> 1;           
    return IRQ_HANDLED;
    
erro_exit:   	 
    g_rece_code = 0;
    g_bit_cnt = 0;
    if((g_key_up_flag != 0)&&(g_scancode != 0)){
    	input_report_key(rk28_at_key_dev, g_scancode, 0); 
        input_sync(rk28_at_key_dev); 
        DBG("%s [%d]---g_scancode = %d at key up\n",__FUNCTION__,__LINE__,g_scancode);
    }
    g_scancode = 0;	
    g_key_up_flag = 0;
    return IRQ_HANDLED;
}

static void rk28_atkey_timer_out(unsigned long data)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
	g_rece_code = 0;
    g_bit_cnt = 0;
    if((g_key_up_flag != 0)&&(g_scancode != 0)){
    	input_report_key(rk28_at_key_dev, g_scancode, 0); 
        input_sync(rk28_at_key_dev); 
        DBG("%s [%d]---g_scancode = %d at key up time out\n",__FUNCTION__,__LINE__,g_scancode);
    }
    g_scancode = 0;	
    g_key_up_flag = 0;
}

static int __init rk28_atboard_init(void)
{
	int i, error;
	
    DBG("%s [%d]\n",__FUNCTION__,__LINE__);
    
    rockchip_mux_api_set(IOMUX_PIN_NAME,IOMUX_PIN_DIR);
    rockchip_mux_api_set(IOMUX_PIN_NAME_DATA,IOMUX_PIN_DIR_DATA);
    GPIOPullUpDown(PS2_DATA_PIN,GPIOPullUp);
    gpio_direction_input(PS2_DATA_PIN);
    
	rk28_at_key_dev = input_allocate_device();
	if (!rk28_at_key_dev)
		return -ENOMEM;

	rk28_at_key_dev->name = "Rk28 At Keyboard";
	rk28_at_key_dev->id.bustype = BUS_HOST;
	rk28_at_key_dev->id.vendor = 0x0001;
	rk28_at_key_dev->id.product = 0x0001;
	rk28_at_key_dev->id.version = 0x0001;
	rk28_at_key_dev->evbit[0] = BIT_MASK(EV_KEY);  /// | BIT_MASK(EV_REP);
	rk28_at_key_dev->keycode = rk28_atkeycode;
	rk28_at_key_dev->keycodesize = sizeof(unsigned char);
	rk28_at_key_dev->keycodemax = ARRAY_SIZE(rk28_atkeycode);

	for (i = 1; i < 160; i++) {
		set_bit(i, rk28_at_key_dev->keybit);
	}

	/* error check */
	error = input_register_device(rk28_at_key_dev);
	if (error) {
		input_free_device(rk28_at_key_dev);
		return error;
	}
    GPIOPullUpDown(PS2_CLK_PIN,GPIOPullUp);
    error = request_gpio_irq(PS2_CLK_PIN,rk28_atkey_interrupt,GPIOEdgelFalling,NULL);
	if(error)
	{
		printk("unable to request at key board IRQ\n");
		goto failed;
	}

    setup_timer(&at_key_timer,rk28_atkey_timer_out,0);
    at_key_timer.expires  = jiffies + msecs_to_jiffies(1000);
    add_timer(&at_key_timer);
    
	return 0;
failed:
	return error;	
}

static void __exit rk28_atboard_exit(void)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
	input_unregister_device(rk28_at_key_dev);
}

module_init(rk28_atboard_init);
module_exit(rk28_atboard_exit);



