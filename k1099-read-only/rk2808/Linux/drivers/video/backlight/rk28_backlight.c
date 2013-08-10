/* arch/arm/mach-rockchip/rk28_backlight.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <asm/io.h>
#include <asm/arch/typedef.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rk28_backlight.h>

//#define RK28_PRINT 
#include <asm/arch/rk28_debug.h>

/*
 * Debug
 */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif


#define write_pwm_reg(id, addr, val)        __raw_writel(val, addr+(PWM_BASE_ADDR_VA+id*0x10)) 
#define read_pwm_reg(id, addr)              __raw_readl(addr+(PWM_BASE_ADDR_VA+id*0x10))    
#define mask_pwm_reg(id, addr, msk, val)    write_dma_reg(id, addr, (val)|((~(msk))&read_dma_reg(id, addr)))
#if(defined(CONFIG_BOARD_TD05D6)||defined(CONFIG_BOARD_TD10D6))
#undef PWM_APB_PRE_DIV
#define PWM_APB_PRE_DIV      1000
#endif

static struct backlight_device *rk28_bl = NULL;
static int suspend_flag = 0;
#define BACKLIGHT_SEE_MINVALUE	52

static s32 rk28_bl_update_status(struct backlight_device *bl)
{
    u32 divh,div_total;
    struct rk28bl_info *rk28_bl_info = bl->dev.parent->platform_data;
    u32 id = rk28_bl_info->pwm_id;
    u32 ref = rk28_bl_info->bl_ref;

    if (suspend_flag)
        return 0;
    
    div_total = read_pwm_reg(id, PWM_REG_LRC);
    if (ref) {
	 DBG(">>>%s-->%d   bl->props.brightness == %d\n",__FUNCTION__,__LINE__,bl->props.brightness);
        divh = div_total*(bl->props.brightness)/BL_STEP;
    } else {
     	 DBG(">>>%s-->%d   bl->props.brightness == %d\n",__FUNCTION__,__LINE__,bl->props.brightness);
	 if(bl->props.brightness < BACKLIGHT_SEE_MINVALUE)	/*avoid can't view screen when close backlight*/
	 	bl->props.brightness = BACKLIGHT_SEE_MINVALUE;
        divh = div_total*(BL_STEP-bl->props.brightness)/BL_STEP;
    }
    write_pwm_reg(id, PWM_REG_HRC, divh);
    rk28printk("%s::========================================\n",__func__);
    return 0;
}

static s32 rk28_bl_get_brightness(struct backlight_device *bl)
{
    u32 divh,div_total;
    struct rk28bl_info *rk28_bl_info = bl->dev.parent->platform_data;
    u32 id = rk28_bl_info->pwm_id;
    u32 ref = rk28_bl_info->bl_ref;
    
    div_total = read_pwm_reg(id, PWM_REG_LRC);
    divh = read_pwm_reg(id, PWM_REG_HRC);

	rk28printk("%s::========================================\n",__func__);

    if (ref) {
        return BL_STEP*divh/div_total;
    } else {
        return BL_STEP-(BL_STEP*divh/div_total);
    }
}

static struct backlight_ops rk28_bl_ops = {
	.update_status = rk28_bl_update_status,
	.get_brightness = rk28_bl_get_brightness,
};


static int rk28_bl_change_clk(ip_id ip , int input_clk)
{
    struct rk28bl_info *rk28_bl_info;
    u32 id;
    u32 divl, divh, tmp;
    u32 div_total;

	if (!rk28_bl) {
        rk28printk(KERN_CRIT "%s: backlight device does not exist \n",
               __func__); 
		return -ENODEV;		
    }
    
    rk28_bl_info = rk28_bl->dev.parent->platform_data;
    id = rk28_bl_info->pwm_id;
    
    divl = read_pwm_reg(id, PWM_REG_LRC);
    divh = read_pwm_reg(id, PWM_REG_HRC);

    tmp = input_clk/PWM_APB_PRE_DIV;
    tmp >>= (1 + (PWM_DIV >> 9));
    
    div_total = (tmp) ? tmp : 1;
    tmp = div_total*divh/divl;
    
    write_pwm_reg(id, PWM_REG_LRC, div_total);
    write_pwm_reg(id, PWM_REG_HRC, tmp);    
    write_pwm_reg(id, PWM_REG_CNTR, 0);    
    
    return 0;
}   
static void rk28_delaybacklight_timer(unsigned long data)
{
	struct rk28bl_info *rk28_bl_info = (struct rk28bl_info *)data;
	u32 id, brightness;
    	u32 div_total, divh;
	id = rk28_bl_info->pwm_id;
    brightness = rk28_bl->props.brightness;
    div_total = read_pwm_reg(id, PWM_REG_LRC);
    if (rk28_bl_info->bl_ref) {
        divh = div_total*(brightness)/BL_STEP;
    } else {
        divh = div_total*(BL_STEP-brightness)/BL_STEP;
    }
    write_pwm_reg(id, PWM_REG_HRC, divh);
    suspend_flag = 0;
    rk28printk("%s: ======================== \n",__func__); 
}

#ifdef CONFIG_ANDROID_POWER
void rk28_bl_suspend(android_early_suspend_t *h)
{
    struct rk28bl_info *rk28_bl_info;
    u32 id;
    u32 div_total, divh;
    
    rk28_bl_info = rk28_bl->dev.parent->platform_data;

    id = rk28_bl_info->pwm_id;

    div_total = read_pwm_reg(id, PWM_REG_LRC);
    
    if(rk28_bl_info->bl_ref) {
        divh = 0;
    } else {
        divh = div_total;
    }

    write_pwm_reg(id, PWM_REG_HRC, divh);

    suspend_flag = 1;
    
    rk28printk("%s: ========================= \n",__func__); 
}


void rk28_bl_resume(android_early_suspend_t *h)
{
    struct rk28bl_info *rk28_bl_info;
   // u32 id, brightness;
    //u32 div_total, divh;
    rk28printk(">>>>>> %s : %s\n", __FILE__, __FUNCTION__);
    rk28_bl_info = rk28_bl->dev.parent->platform_data;
	
	rk28_bl_info->timer.expires  = jiffies + 30;
	add_timer(&rk28_bl_info->timer);
	#if 0
    id = rk28_bl_info->pwm_id;
    brightness = rk28_bl->props.brightness;
    
    div_total = read_pwm_reg(id, PWM_REG_LRC);
    if (rk28_bl_info->bl_ref) {
        divh = div_total*(brightness)/BL_STEP;
    } else {
        divh = div_total*(BL_STEP-brightness)/BL_STEP;
    }
    //mdelay(100);
    write_pwm_reg(id, PWM_REG_HRC, divh);

    suspend_flag = 0;

    rk28printk("%s: ======================== \n",__func__); 
	#endif 
}

static android_early_suspend_t bl_early_suspend;
#endif

static char *pwm_iomux[] = {
     GPIOF2_APWM0_SEL_NAME,
     GPIOF3_APWM1_MMC0DETN_NAME,
     GPIOF4_APWM2_MMC0WPT_NAME,
     GPIOF5_APWM3_DPWM3_NAME,
};

static int rk28_backlight_probe(struct platform_device *pdev)
{		
    s32 ret;
    struct rk28bl_info *rk28_bl_info = pdev->dev.platform_data;
    u32 pin =  (rk28_bl_info->pw_pin >> 8) & 0xff;
    u32 lev =  rk28_bl_info->pw_pin & 0xf;
    u32 id  =  rk28_bl_info->pwm_id;
    u32 divh, div_total;

	if (rk28_bl) {
        rk28printk(KERN_CRIT "%s: backlight device register has existed \n",
               __func__); 
		return -EEXIST;		
    }
    
	rk28_bl = backlight_device_register("rk28_bl", &pdev->dev, NULL, &rk28_bl_ops);
	if (!rk28_bl) {
        rk28printk(KERN_CRIT "%s: backlight device register error\n",
               __func__); 
		return -ENODEV;		
	}

    div_total = rockchip_clk_get_apb()/PWM_APB_PRE_DIV;
    div_total >>= (1 + (PWM_DIV >> 9));
    div_total = (div_total) ? div_total : 1;
    
#if defined(CONFIG_BOARD_BM999)||defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_IPADV5)||defined(CONFIG_BOARD_IPADY1006)\
	||defined(CONFIG_BOARD_RK5900)||defined(CONFIG_BOARD_BM730) ||defined(CONFIG_BOARD_E700)||defined(CONFIG_BOARD_NX7005)\
	||defined(CONFIG_BOARD_TD05D6)||defined(CONFIG_BOARD_TD10D6)||defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_IPAD100)\
	||defined(CONFIG_BOARD_NM701)||defined(CONFIG_BOARD_ZTX)
//	divh = div_total / 2;
//	divh = 0;
    if(rk28_bl_info->bl_ref) {
        divh = div_total/3;
    } else {
        divh = div_total*2/3;
    }
#else    
    if(rk28_bl_info->bl_ref) {
        divh = 0;
    } else {
        divh = div_total / 2;
    }
#endif
    /*init timer to dispose workqueue */
    setup_timer(&rk28_bl_info->timer, rk28_delaybacklight_timer, (unsigned long)rk28_bl_info);
    
    write_pwm_reg(id, PWM_REG_CTRL, PWM_DIV|PWM_RESET);
    write_pwm_reg(id, PWM_REG_LRC, div_total);
    write_pwm_reg(id, PWM_REG_HRC, divh);
    write_pwm_reg(id, PWM_REG_CNTR, 0x0);
    write_pwm_reg(id, PWM_REG_CTRL, PWM_DIV|PWM_ENABLE|PWM_TIME_EN);

    ret = rockchip_scu_apbunit_register(SCU_IPID_PWM, "pwm0", rk28_bl_change_clk);
    if (ret < 0) {
        rk28printk(KERN_CRIT "%s: scu apbunit register error\n",
               __func__); 
        backlight_device_unregister(rk28_bl);
        return ret;
    }
   
	rk28_bl->props.power = FB_BLANK_UNBLANK;
	rk28_bl->props.fb_blank = FB_BLANK_UNBLANK;
	rk28_bl->props.max_brightness = BL_STEP;
	rk28_bl->props.brightness = rk28_bl_get_brightness(rk28_bl);

#ifdef CONFIG_ANDROID_POWER
    bl_early_suspend.suspend = rk28_bl_suspend;
    bl_early_suspend.resume = rk28_bl_resume;
    bl_early_suspend.level = ~0x0;
    android_register_early_suspend(&bl_early_suspend);
#endif

    rockchip_mux_api_set(pwm_iomux[id], 1);
    
    gpio_direction_output(pin, 0);
    GPIOSetPinLevel(pin, lev);

/*enable power pin*///kevin add
#if(defined(CONFIG_BOARD_TD05D6)||defined(CONFIG_BOARD_TD10D6))
	GPIOSetPinDirection(GPIOPortF_Pin5,GPIO_OUT);
	
	GPIOPullUpDown(GPIOPortF_Pin5,GPIOPullUp);
	
	GPIOSetPinLevel(GPIOPortF_Pin5,GPIO_HIGH);
#elif(defined(CONFIG_BOARD_ZTX))
	GPIOSetPinDirection(GPIOPortE_Pin1,GPIO_OUT);
	
	GPIOPullUpDown(GPIOPortE_Pin1,GPIOPullUp);
	
	GPIOSetPinLevel(GPIOPortE_Pin1,GPIO_HIGH);
#else
	GPIOSetPinDirection(GPIOPortF_Pin1,GPIO_OUT);
	
	GPIOPullUpDown(GPIOPortF_Pin1,GPIOPullUp);
	
	GPIOSetPinLevel(GPIOPortF_Pin1,GPIO_HIGH);
#endif
////////////////////////////////////////////////////////////////////   
	rk28printk("%s::========================================\n",__func__);

	rk28_bl_suspend(NULL);
/*	suspend_flag = 0;*/
	rk28_bl_resume(NULL);
	return 0;
}

static int rk28_backlight_remove(struct platform_device *pdev)
{		
	if (rk28_bl) {
		backlight_device_unregister(rk28_bl);
#ifdef CONFIG_ANDROID_POWER
        android_unregister_early_suspend(&bl_early_suspend);
#endif        
        return 0;
    } else {
        rk28printk(KERN_CRIT "%s: no backlight device has registered\n",
               __func__); 
        return -ENODEV;      
    }
}
static void rk28_backlight_shutdown(struct platform_device *pdev)
{

   u32 divh,div_total;
    struct rk28bl_info *rk28_bl_info = pdev->dev.platform_data;
    u32 id = rk28_bl_info->pwm_id;
    u32 pin=rk28_bl_info->pw_pin;
    u32  brightness;
    u8 lev;
	brightness = rk28_bl->props.brightness; 
	brightness/=2;
	div_total = read_pwm_reg(id, PWM_REG_LRC);   
	if (rk28_bl_info->bl_ref) {
			divh = div_total*(brightness)/BL_STEP;
	} else {
			divh = div_total*(BL_STEP-brightness)/BL_STEP;
	}
	write_pwm_reg(id, PWM_REG_HRC, divh);
	//printk("divh=%d\n",divh);
	 mdelay(100);
	brightness/=2;
	if (rk28_bl_info->bl_ref) {
	divh = div_total*(brightness)/BL_STEP;
	} else {
	divh = div_total*(BL_STEP-brightness)/BL_STEP;
	}
	//printk("------------rk28_backlight_shutdown  mdelay----------------------------\n");
	write_pwm_reg(id, PWM_REG_HRC, divh); 
    mdelay(100);
	/*set  PF1=1 PF2=1 for close backlight*/	
#ifdef CONFIG_MACH_LANMO_W7
	rockchip_mux_api_set(GPIOF2_APWM0_SEL_NAME, IOMUXB_GPIO1_B2);
	GPIOSetPinDirection(GPIOPortF_Pin2,GPIO_OUT);
	GPIOSetPinLevel(GPIOPortF_Pin2,GPIO_HIGH);
	mdelay(100);
#else
      if(rk28_bl_info->bl_ref) {
        lev = GPIO_LOW;
    } else {
        lev = GPIO_HIGH;
    }
    GPIOSetPinLevel(pin, lev);
#endif
  
}

static struct platform_driver rk28_backlight_driver = {
	.probe	= rk28_backlight_probe,
	.remove = rk28_backlight_remove,
	.driver	= {
		.name	= "rk28_backlight",
		.owner	= THIS_MODULE,
	},
	.shutdown=rk28_backlight_shutdown,
};


static int __init rk28_backlight_init(void)
{
	rk28printk("%s::========================================\n",__func__);
	platform_driver_register(&rk28_backlight_driver);
	return 0;
}
rootfs_initcall(rk28_backlight_init);

//late_initcall(rk28_backlight_init);
//module_init(rk28_backlight_init);
