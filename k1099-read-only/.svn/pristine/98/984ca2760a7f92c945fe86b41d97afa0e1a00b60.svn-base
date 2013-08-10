/*
 *  driver/input/keyboard/rk28_mouse_touchpad.c
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
#include <linux/slab.h>

#include <linux/input.h>
#include <linux/serio.h>
#include <linux/libps2.h>
#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/hardware.h>
#include <linux/delay.h>

#include "psmouse.h"
#include "touchkit_ps2.h"

MODULE_DESCRIPTION("ASoC rk2808 mouse touch pad driver");
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_LICENSE("GPL");


#define MOUSE_IOMUX_PIN_NAME GPIOE_U1IR_I2C1_NAME
#define MOUSE_IOMUX_PIN_DIR  IOMUXA_GPIO1_A67
#define MOUSE_PS2_CLK_PIN    GPIOPortE_Pin6
#define MOUSE_PS2_DATA_PIN   GPIOPortE_Pin7
/*
 * Debug
 */
#if 0
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif

static struct input_dev *rk28_mouse_dev;

static unsigned  char g_mouse_bit_cnt = 0;
static unsigned  int g_rece_mouse_code = 0;
struct timer_list mouse_timer;
//struct timer_list mouse_timer_up;
static unsigned  int g_mouse_scancode=0;
static unsigned  char g_sent_code = 0xf3;  ///0xf4;
static unsigned  char g_host_sent_flag = 0;
static unsigned  int g_rece_code_byte[3] = {0,0,0};
static unsigned  int g_rece_code_byte_cnt = 0;
static unsigned  int g_sent_byte = 0;

static void rk28_ps2_sent_start(void)
{
	g_host_sent_flag = 1;
	gpio_irq_disable(MOUSE_PS2_CLK_PIN);
	gpio_direction_output(MOUSE_PS2_CLK_PIN,GPIO_LOW);
	GPIOSetPinLevel(MOUSE_PS2_CLK_PIN,GPIO_LOW);
	udelay(100);
	gpio_direction_output(MOUSE_PS2_DATA_PIN,GPIO_LOW);
	GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_LOW);
	GPIOSetPinLevel(MOUSE_PS2_CLK_PIN,GPIO_HIGH);
	gpio_direction_input(MOUSE_PS2_CLK_PIN);
	gpio_irq_enable(MOUSE_PS2_CLK_PIN);
}

static inline irqreturn_t rk28_mouse_interrupt(s32 irq, void *dev_id)
{   
    int i,j;
   // del_timer(&mouse_timer);
   // mouse_timer.expires  = jiffies + msecs_to_jiffies(200);
    //add_timer(&mouse_timer);
    mod_timer(&mouse_timer,jiffies + msecs_to_jiffies(200));
    if(g_host_sent_flag == 1){
    	g_mouse_bit_cnt++;
    	udelay(10);
    	if(g_mouse_bit_cnt < 9){    		
    		if(g_sent_code & 0x1)
    			GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_HIGH);
    		else
    			GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_LOW);
    		g_sent_code = g_sent_code>>1;	
    		return IRQ_HANDLED;
    	}else if(g_mouse_bit_cnt == 9){
    		if(g_sent_byte != 0) // 0xf4 
    			GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_LOW);
    		else
    			GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_HIGH);	
    		return IRQ_HANDLED;
    	}else if(g_mouse_bit_cnt == 10){
    		GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_HIGH);	
    		return IRQ_HANDLED;
        }else{
        	//GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_LOW);
        	//udelay(100);
        	GPIOSetPinLevel(MOUSE_PS2_DATA_PIN,GPIO_HIGH);
        	g_mouse_bit_cnt = 0;
        	g_rece_mouse_code = 0;
        	g_host_sent_flag = 0;
        	g_sent_byte++;
        	gpio_direction_input(MOUSE_PS2_DATA_PIN);
        	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
        	if(g_sent_byte==1)
        		g_sent_code = 0x64;
        	else
        		g_sent_code = 0xf4;  ///14;
    	    return IRQ_HANDLED;
        }
    }
    g_rece_mouse_code &= 0xfbff; 
    g_rece_mouse_code |= (GPIOGetPinLevel(MOUSE_PS2_DATA_PIN)&0x01)<<10;
    if((g_mouse_bit_cnt == 0)&&(g_rece_mouse_code != 0)){       
    	goto mouse_erro_exit;   		
    }
    g_mouse_bit_cnt++;	    
    if(g_mouse_bit_cnt > 0x0a){ ///11 bit code get end
    	g_mouse_bit_cnt = 0;
    	if((g_rece_mouse_code&0x400) != 0x400){
    	    goto mouse_erro_exit;
    	}
        g_rece_mouse_code = (g_rece_mouse_code>>1) & 0xff;
       /* if((g_rece_mouse_code == 0x00)&&(g_mouse_scancode == 0xaa)){
        	rk28_ps2_sent_start();
        	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
        	g_rece_mouse_code = 0; 
            return IRQ_HANDLED;
        }*/
        if(g_sent_byte>2)g_sent_byte = 0;
        if((g_sent_byte != 0)&&(g_rece_mouse_code == 0xfa)){ ///ack 0xfa
        	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
        	rk28_ps2_sent_start();
        	return IRQ_HANDLED;
        }
        g_mouse_scancode = g_rece_mouse_code;             
        g_rece_code_byte[g_rece_code_byte_cnt] = g_mouse_scancode;
        if((g_rece_code_byte[0]&0xc) != 0x8)
        	goto mouse_erro_exit;  ///  mouse_erro_exit; 
        g_rece_code_byte_cnt++;
        g_rece_mouse_code = 0; 
        if(g_rece_code_byte_cnt > 2){
            g_rece_code_byte_cnt = 0;
            i = (g_rece_code_byte[1] ? (int) g_rece_code_byte[1] - (int) ((g_rece_code_byte[0] << 4) & 0x100) : 0);
            j = (g_rece_code_byte[2] ? (int) ((g_rece_code_byte[0] << 3) & 0x100) - (int) g_rece_code_byte[2] : 0);	
            if((i>150)||(i<-150)||(j>150)||(j<-150))
            	goto mouse_erro_exit;	
            input_report_key(rk28_mouse_dev, BTN_LEFT,g_rece_code_byte[0]&0x01);
            //input_report_key(rk28_mouse_dev, BTN_RIGHT,(g_rece_code_byte[0]>>1)&0x01);
            input_report_rel(rk28_mouse_dev, REL_X, i);
	        input_report_rel(rk28_mouse_dev, REL_Y, j);
	        input_sync(rk28_mouse_dev);
	        g_rece_code_byte_cnt = 0;
    		g_rece_code_byte[0] = 0;
    		g_rece_code_byte[1] = 0;
    		g_rece_code_byte[2] = 0;
	      //  printk("%s [%d] x= %d y=%d\n",__FUNCTION__,__LINE__,i,j);
            DBG("%s [%d]---g_rece_code_byte[0]=%x g_rece_code_byte[1]=%x g_rece_code_byte[2]=%x\n",__FUNCTION__,__LINE__,g_rece_code_byte[0],g_rece_code_byte[1],g_rece_code_byte[2]);           
        }
        return IRQ_HANDLED;	
    }	
    g_rece_mouse_code = g_rece_mouse_code >> 1;           
    return IRQ_HANDLED;
    
mouse_erro_exit:     
   /// printk("%s [%d]---g_rece_code_byte[0] = %d g_rece_code_byte[1] = %d g_rece_code_byte[2] = %d check err\n",__FUNCTION__,__LINE__,g_rece_code_byte[0],g_rece_code_byte[1],g_rece_code_byte[2]);	
///mouse_erro_exit:   	 
    g_mouse_scancode = 0;
    g_mouse_bit_cnt = 0;
    g_rece_mouse_code = 0;
    g_rece_code_byte_cnt = 0;
    g_rece_code_byte[0] = 0;
    g_rece_code_byte[1] = 0;
    g_rece_code_byte[2] = 0;    
    return IRQ_HANDLED;
}
#if 0
static void rk28_mouse_timer_up(unsigned long data)
{
	g_sent_flag = 0;
}
#endif
static void rk28_mouse_timer_out(unsigned long data)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
	g_rece_mouse_code = 0;
    g_mouse_bit_cnt = 0;
    g_mouse_scancode = 0;	
    g_rece_code_byte_cnt = 0;   
    g_rece_code_byte[0] = 0;
    g_rece_code_byte[1] = 0;
    g_rece_code_byte[2] = 0;
}

static int __init rk28_mouse_init(void)
{
	int error;
	
    DBG("%s [%d]\n",__FUNCTION__,__LINE__);
    
    rockchip_mux_api_set(MOUSE_IOMUX_PIN_NAME,MOUSE_IOMUX_PIN_DIR);
    GPIOPullUpDown(MOUSE_PS2_DATA_PIN,GPIOPullUp);
    gpio_direction_input(MOUSE_PS2_DATA_PIN);
    
	rk28_mouse_dev = input_allocate_device();
	if (!rk28_mouse_dev)
		return -ENOMEM;

	rk28_mouse_dev->name = "Rk28 Mouse";
	rk28_mouse_dev->id.bustype = BUS_HOST;
	rk28_mouse_dev->id.vendor = 0x0001;
	rk28_mouse_dev->id.product = 0x0001;
	rk28_mouse_dev->id.version = 0x0100;
	rk28_mouse_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
	rk28_mouse_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT); 
	rk28_mouse_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);

	/* error check */
	error = input_register_device(rk28_mouse_dev);
	if (error) {
		input_free_device(rk28_mouse_dev);
		return error;
	}
	
    setup_timer(&mouse_timer,rk28_mouse_timer_out,0);       
    mouse_timer.expires  = jiffies + msecs_to_jiffies(1000);
    add_timer(&mouse_timer);
  //  setup_timer(&mouse_timer_up,rk28_mouse_timer_up,0);       
    rk28_ps2_sent_start();
    GPIOPullUpDown(MOUSE_PS2_CLK_PIN,GPIOPullUp);	
    error = request_gpio_irq(MOUSE_PS2_CLK_PIN,rk28_mouse_interrupt,GPIOEdgelFalling,NULL);
	if(error)
	{
		printk("unable to request mouse IRQ\n");
		goto failed;
	}
	mdelay(500);
	return 0;
failed:
	return error;	
}



static void __exit rk28_mouse_exit(void)
{
	DBG("%s [%d]\n",__FUNCTION__,__LINE__);
	input_unregister_device(rk28_mouse_dev);
}

module_init(rk28_mouse_init);
module_exit(rk28_mouse_exit);
