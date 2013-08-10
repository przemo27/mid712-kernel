/*
 * linux/drivers/input/keyboard/rk28_ad_button.c
 *
 * Driver for the rk28 matrix keyboard controller.
 *
 * Created: 2009-5-4
 * Author:	LZJ <lzj@rockchip.com>
 *
 * This driver program support to AD key which use for rk28 chip
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
#include <asm/arch/adc.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_backlight.h>

//#define WAKEUP_KEY_PORT         GPIOPortA_Pin0  /* 2808SDK BOARD */
#define WAKEUP_KEY_PORT         GPIOPortE_Pin2  /* RUIGUAN 2808 */

/* Debug */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
//ROCKCHIP AD KEY CODE ,for demo board
//      key		--->	EV	
#if 1
#define AD1KEY1		103//DPAD_UP	----------UP
#define AD1KEY2 		106//DPAD_RIGHT-------FFD
#define AD1KEY3 		105//DPAD_LEFT--------FFW	
#define AD1KEY4 		108//DPAD_DOWN------DOWN		
#define AD1KEY5		62   //ENDCALL           WAKE_DROPPED
#define AD1KEY6		28    //ENTER             
#define AD1KEY7		102    //ENTER   

#define AD2KEY1		105   //VOLUME_DOWN  	 	59	//MENU			//115   //VOLUME_UP
#define AD2KEY2 		106   //VOLUME_UP      		//114   //VOLUME_DOWN
#define AD2KEY3 		59	//MENU	
#define AD2KEY4 		28   //HOME				//62   //ENDCALL  		
#define AD2KEY5 		158	//BACK----ESC   		115   //VOLUME_UP	//158	//BACK------------ESC
#define AD2KEY6 		62	//POWER
#define AD2KEY7 		116	//POWER
#else 
#define AD1KEY1 		103//DPAD_UP	----------UP
#define AD1KEY2 		106//DPAD_RIGHT-------FFD
#define AD1KEY3 		105//DPAD_LEFT--------FFW	
#define AD1KEY4 		108//DPAD_DOWN------DOWN		
#define AD1KEY5 		28    //ENTER
#define AD1KEY6 		28    //ENTER

#define AD2KEY1 		103//DPAD_UP	----------UP
#define AD2KEY2 		106//DPAD_RIGHT-------FFD
#define AD2KEY3 		108//DPAD_DOWN------DOWN
#define AD2KEY4 		59	//MENU		
#define AD2KEY5 		158 //BACK------------ESC
#define AD2KEY6 		116 //POWER

#endif
#define Valuedrift		70
#define EmptyADValue			950  //1000
#define InvalidADValue	10
#define ADKEYNum		12

/*power event button*/
#define  POWER				116
#define  ENDCALL				62
#define  ONESEC_TIMES		100
#define  SEC_NUM			1
#define  SLEEP_TIME			2	/*per 40ms*/
static unsigned  int	pwrscantimes = 0;
static unsigned  int valuecount = 0;
static unsigned  int g_code = 0;
extern u8 test_for_backligh;
extern int pm_sleep_status ;
volatile int g_enable_sleep=1;
//extern  int rk28_bl_update_status_for_Test(int data);
extern int wifi_turn_on_card(void);
extern int wifi_turn_off_card(void);
extern int wifi_power_up_wifi(void);
extern int wifi_power_down_wifi(void);
int g_wake_press = 0;
struct timer_list g_wakeup_key_timer;
volatile int g_wakeup_key_enable = 1;
u8 Wifi_turn_on_flag=0;
u8 Wifi_turn_off_flag=0;
extern u8 needfirstonwifi;

#define KEY_PHYS_NAME	"rk28_AD_button/input0"
//ADC Registers
typedef  struct tagADC_keyst
{
	unsigned int adc_value;
	unsigned int adc_keycode;
}ADC_keyst,*pADC_keyst;

//	adc	 ---> key	
static  ADC_keyst ad1valuetab[] = {
	{ 95, AD1KEY1},
	{247,AD1KEY2},
	{403,AD1KEY3},
	{561,AD1KEY4},
	{723,AD1KEY5},
//	{899,AD1KEY6},

	{EmptyADValue,0}
};
static  ADC_keyst ad2valuetab[] = {
	{220,  AD2KEY1},
	{120, AD2KEY2},
	{330, AD1KEY7},
	{440, AD2KEY3},
	{560, AD2KEY4},
	{640, AD2KEY5},
	{730, AD1KEY2},

	{50, AD1KEY3},
	{EmptyADValue,0}

};



//key code tab
static unsigned char initkey_code[ ] = 
{
	AD1KEY1, AD1KEY2,AD1KEY3,AD1KEY4,AD1KEY5,AD1KEY6,
	AD2KEY1, AD2KEY2,AD2KEY3,AD2KEY4,AD2KEY5,AD2KEY6,ENDCALL	,KEY_WAKEUP
};

int IoCode[4][2]={
		GPIOPortB_Pin4,AD1KEY2,	// 106
		GPIOPortB_Pin5,AD1KEY1,	// 106
		GPIOPortB_Pin6,AD1KEY3,	// 105
		GPIOPortB_Pin7,AD1KEY4,	// 108
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

 struct rk28_AD_button *prockAD_button;


extern 	void  RockAdcScanning(void);
extern 	void  printADCValue(void);
extern 	int32 ADCInit(void);
extern 	int get_rock_adc1(void);
extern	int get_rock_adc2(void);

unsigned int find_rock_adkeycode(unsigned int advalue,pADC_keyst ptab)
{	
	while(ptab->adc_value!=EmptyADValue)
		{
			if((advalue>ptab->adc_value-Valuedrift)&&(advalue<ptab->adc_value+Valuedrift))
				return ptab->adc_keycode;
			
			ptab++;
		}
	
	return 0;
}
#define write_pwm_reg(id, addr, val)        __raw_writel(val, addr+(PWM_BASE_ADDR_VA+id*0x10)) 
#define read_pwm_reg(id, addr)              __raw_readl(addr+(PWM_BASE_ADDR_VA+id*0x10)) 
#define BACKLIGHT_SEE_MINVALUE	52
#define mask_pwm_reg(id, addr, msk, val)    write_dma_reg(id, addr, (val)|((~(msk))&read_dma_reg(id, addr)))
static s32  rk28_bl_update_status_for_Test(int data)
{
    u32 divh,div_total;
   

    if (test_for_backligh)
       return 0;
   
    div_total = read_pwm_reg(0, PWM_REG_LRC);
  
	 if(data < BACKLIGHT_SEE_MINVALUE)	/*avoid can't view screen when close backlight*/
	 	data = BACKLIGHT_SEE_MINVALUE;
        divh = div_total*(BL_STEP-data)/BL_STEP;
    
    write_pwm_reg(0, PWM_REG_HRC, divh);
    //rk28printk("%s::========================================\n",__func__);
    return 0;
}
static int rk28_AD_button_open(struct input_dev *dev)
{
//	struct rk28_AD_button *AD_button = input_get_drvdata(dev);


	return 0;
}

static void rk28_AD_button_close(struct input_dev *dev)
{
//	struct rk28_AD_button *AD_button = input_get_drvdata(dev);

}
#define res_size(res)	((res)->end - (res)->start + 1)

int keydata=58;
static  int ADSampleTimes = 0;
static  int IOkeyback = 0;
UINT32 RemoteKey=0;
int backligh_just_vallue=0;
int backligh_back_vallue=0;
#define LED_LIGHT_TIME	100

static UINT32 LED_Delay_Counter=LED_LIGHT_TIME;
static void rk28_adkeyscan_timer(unsigned long data)
{
	unsigned int ADKEY1,ADKEY2,code = 0,i;
	/*Enable  AD controller to sample */

	prockAD_button->timer.expires  = jiffies+msecs_to_jiffies(20);
	add_timer(&prockAD_button->timer);
	//	printk("\n LED_Delay_Counter=%d  \n",LED_Delay_Counter);
	


	RockAdcScanning();
	if (ADSampleTimes < 4)
	{
		ADSampleTimes ++;		
		goto scan_io_key;  	/* scan gpio button event*/
	}
	ADSampleTimes = 0;	
	/*Get button value*/
	ADKEY1=get_rock_adc1();
	//ADKEY2=get_rock_adc2();
	
	//if(ADKEY1<EmptyADValue)
		//printk("\n ADC1 value=%d  \n",ADKEY1);
		
	//printk("\n ADC2 value=%d  \n",ADKEY2);

	/*
	if(backligh_back_vallue!=ADKEY2){
	backligh_back_vallue=ADKEY2;
	backligh_just_vallue=ADKEY2/3;
	if(backligh_just_vallue>255)
		backligh_just_vallue=255;
	//printk("\n backligh_just_vallue value=%d  \n",backligh_just_vallue);
	//rk28_bl_update_status_for_Test(255-backligh_just_vallue);
	}
	*/
#if 1	//ffhh
	if((ADKEY1>EmptyADValue)&&(ADKEY1<=InvalidADValue))  
		goto scan_io_key1;
#endif	
	valuecount++;
 	if(valuecount < 2)
		goto scan_code;	
	if(ADKEY1<50)
	code=AD1KEY3;
	else
	code=find_rock_adkeycode(ADKEY1,ad2valuetab);
	valuecount = 2;	
	goto scan_code;  ///scan_code;	
scan_io_key1:
	valuecount = 0;	

scan_code:	
	if((g_code == 0) && (code == 0)){
	    goto scan_io_key; 
	}
	///DBG("\n key button PE2 == %d  \n",GPIOGetPinLevel(GPIOPortE_Pin2)); 
	if(code != 0){
		if(valuecount<2)
			goto scan_io_key; 
		if(g_code == 0){
			g_code = code;
			DBG("\n %s::%d rock adc1 key scan ,find press down a key=%d  \n",__func__,__LINE__,g_code);
			input_report_key(prockAD_button->input_dev,g_code,1);
	        	input_sync(prockAD_button->input_dev);
                     pca955x_gpio_direction_output(PCA955X_Pin12,GPIO_HIGH);	 
                     mdelay(500);
                     pca955x_gpio_direction_output(PCA955X_Pin12,GPIO_LOW);	
	        	goto scan_io_key; 
		}else{
			if(g_code != code){
				DBG("\n %s::%d rock adc1 key scan ,find press up a key=%d  \n",__func__,__LINE__,g_code);
				input_report_key(prockAD_button->input_dev,g_code,0);
	            		input_sync(prockAD_button->input_dev);
	            		DBG("\n %s::%d rock adc1 key scan ,find press down a key=%d  \n",__func__,__LINE__,code);
	            		input_report_key(prockAD_button->input_dev,code,1);
	            		input_sync(prockAD_button->input_dev);
				
	            		g_code = code;
				
	           		goto scan_io_key; 
			}
		}
    
    	}
	if((g_code != 0)&&(code == 0)&&(ADSampleTimes == 0)){
		DBG("\n %s::%d rock adc1 key scan ,find press up a key=%d  \n",__func__,__LINE__,g_code);
	    	input_report_key(prockAD_button->input_dev,g_code,0);
	    	input_sync(prockAD_button->input_dev);
		
		valuecount = 0;
		g_code = 0;
		goto scan_io_key;
	}
scan_io_key :
 //  rockchip_mux_api_set(GPIOF5_APWM3_DPWM3_NAME, IOMUXB_GPIO1_B5);
   //GPIOSetPinDirection(GPIOPortF_Pin5,GPIO_IN);
   #if 0
   if(needfirstonwifi){
   if(GPIOGetPinLevel(GPIOPortF_Pin5)){
   	if(!Wifi_turn_on_flag)
   		{
   		 

	        if(firstonwifi!=1){
   		  wifi_turn_on_card();
		 wifi_power_up_wifi();
   		 printk("GPIOPortF_Pin5  HIGH  \n");
		  Wifi_turn_on_flag=1;
		  Wifi_turn_off_flag=0;
	       }
   		}
   	}
   else {
   	      if(!Wifi_turn_off_flag)
   		{
   		  if(firstonwifi!=2){
		    
   		  wifi_turn_off_card();
		 wifi_power_up_wifi();
   		  printk("GPIOPortF_Pin5  LOW  \n");
		  Wifi_turn_on_flag=0;
		  Wifi_turn_off_flag=1;
   		  	}
   		}
			
   	}
   	}
   #endif
for(i=0;i<4;i++){	
	GPIOSetPinDirection(IoCode[i][0],GPIO_IN);
	
#if 1	
	if(GPIOGetPinLevel(IoCode[i][0])){
			if((IOkeyback!=IoCode[i][1])&&(IOkeyback)){
			input_report_key(prockAD_button->input_dev,IOkeyback,0);
			input_sync(prockAD_button->input_dev);
			DBG("Key Pb%d up code=%d!!!\n",i,IOkeyback);
			}
			IOkeyback=IoCode[i][1];
			input_report_key(prockAD_button->input_dev,IoCode[i][1],1);
			input_sync(prockAD_button->input_dev);
			DBG("Key Pb%d down code=%d!!!\n",i,IoCode[i][1]);
			pca955x_gpio_direction_output(PCA955X_Pin12,GPIO_HIGH);	 
                     mdelay(500);
                     pca955x_gpio_direction_output(PCA955X_Pin12,GPIO_LOW);	
			break;
	}else if(IOkeyback==IoCode[i][1]){
			IOkeyback=0;
			input_report_key(prockAD_button->input_dev,IoCode[i][1],0);
			input_sync(prockAD_button->input_dev);
			DBG("Key Pb%d up code=%d!!!\n",i,IoCode[i][1]);
			
			break;
	}
#endif	
}


    if(g_wake_press)
    {
        pwrscantimes = 0;
        g_wake_press = !GPIOGetPinLevel(WAKEUP_KEY_PORT);
        if(g_wake_press) return;
    }

//	if(!GPIOGetPinLevel(GPIOPortF_Pin0))
	if(!GPIOGetPinLevel(WAKEUP_KEY_PORT))
	{
		pwrscantimes += 1;
		if(pwrscantimes == (SEC_NUM * ONESEC_TIMES))
		{
			input_report_key(prockAD_button->input_dev,ENDCALL,1);
			input_sync(prockAD_button->input_dev);
			DBG("the kernel come to power down!!!\n");
			LED_Delay_Counter=LED_LIGHT_TIME;
		}
		if(pwrscantimes ==( (SEC_NUM + 1)* ONESEC_TIMES))
		{
			pwrscantimes = 0;
			input_report_key(prockAD_button->input_dev,ENDCALL,0);
			input_sync(prockAD_button->input_dev);
			DBG("the kernel come to power up!!!\n");
			
		}
		return ;
	}
	
	if( pwrscantimes > SLEEP_TIME)
	{
		pwrscantimes = 0;
        DBG("========== %s: g_enable_sleep=%d\n", __FUNCTION__, g_enable_sleep);
		if(g_enable_sleep)
		{
			input_report_key(prockAD_button->input_dev,AD1KEY5,1);
			input_sync(prockAD_button->input_dev);
			input_report_key(prockAD_button->input_dev,AD1KEY5,0);
			input_sync(prockAD_button->input_dev);
                     rockchip_mux_api_set(GPIOE_I2C0_SEL_NAME, IOMUXA_I2C0);
                     
                     rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, IOMUXA_I2C1);
                    
		}
		 DBG("\n%s^^^^Wake Up ^^^^^!!\n",__FUNCTION__);
	}
}

/*
 * 20100309,HSL@RK,send a key to light the screen.
 * (like when insert use to wake up system from deep sleep)
 * the key value should matain unchange!!
 */
void rk28_send_wakeup_key( void ) 
{
        input_report_key(prockAD_button->input_dev,62,1);
        input_sync(prockAD_button->input_dev);
        input_report_key(prockAD_button->input_dev,62,0);
        input_sync(prockAD_button->input_dev);
}

static void rk28_enable_wakeup_key(void *dev_id)
{
    g_wakeup_key_enable = 1;
    del_timer(&g_wakeup_key_timer);
}

//a@nzy
static irqreturn_t rk28_AD_irq_handler(s32 irq, void *dev_id)
{
	rk28printk("\n =================================rk28_AD_irq_handler===================================\n");

    rk28printk("========= pm_sleep_status=%d   PA0=%d\n", pm_sleep_status, GPIOGetPinLevel(WAKEUP_KEY_PORT));
    
   if( pm_sleep_status == 1)
    {
        g_wake_press = !GPIOGetPinLevel(WAKEUP_KEY_PORT);
        rk28printk("========== in sleep: g_wakeup_key_enable=%d  g_wake_press=%d\n", g_wakeup_key_enable, g_wake_press);
        if(g_wakeup_key_enable)
        {
        	input_report_key(prockAD_button->input_dev,AD1KEY5,1);
        	input_sync(prockAD_button->input_dev);
        	input_report_key(prockAD_button->input_dev,AD1KEY5,0);
        	input_sync(prockAD_button->input_dev);
        	
        	printk("\n%s^^level 2^^Wake Up ^^^^^!!\n",__FUNCTION__);

            g_wakeup_key_enable = 0;

            // cmy@20100326: 只有在5S后，或者系统被唤醒后(pm_sleep_status == 1)，才能再处理唤醒IRQ
        	setup_timer(&g_wakeup_key_timer, rk28_enable_wakeup_key, (unsigned long)0);
        	g_wakeup_key_timer.expires  = jiffies + (5*HZ);
        	add_timer(&g_wakeup_key_timer);
        }
    }
    return IRQ_HANDLED;
}

static int __devinit rk28_AD_button_probe(struct platform_device *pdev)
{
	struct rk28_AD_button *AD_button;
	struct input_dev *input_dev;
	int  error,i;
	AD_button = kzalloc(sizeof(struct rk28_AD_button), GFP_KERNEL);
	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev || !AD_button) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed1;
	}

    //a@nzy
	int ret = request_irq(IRQ_NR_ADC, rk28_AD_irq_handler, 0, "ADC", NULL);
	if (ret < 0) {
		printk(KERN_CRIT "Can't register IRQ for ADC\n");
		return ret;
	}
	
	memcpy(AD_button->keycodes, initkey_code, sizeof(AD_button->keycodes));
	input_dev->name = pdev->name;
	input_dev->open = rk28_AD_button_open;
	input_dev->close = rk28_AD_button_close;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycode = AD_button->keycodes;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(initkey_code);
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	AD_button->input_dev = input_dev;
	input_set_drvdata(input_dev, AD_button);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;

	platform_set_drvdata(pdev, AD_button);

	ADCInit();
	prockAD_button=AD_button;

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed2;
	}

	setup_timer(&AD_button->timer, rk28_adkeyscan_timer, (unsigned long)AD_button);
	//mod_timer(&AD_button->timer, 100);
	AD_button->timer.expires  = jiffies + 3;
	add_timer(&AD_button->timer);
 #if defined(CONFIG_BOARD_NX7005)   
       GPIOSetPinDirection(GPIOPortB_Pin4,GPIO_IN);
       GPIOSetPinDirection(GPIOPortB_Pin5,GPIO_IN);
       GPIOSetPinDirection(GPIOPortB_Pin6,GPIO_IN);
       GPIOSetPinDirection(GPIOPortB_Pin7,GPIO_IN);
 #endif
#ifdef CONFIG_MACH_RK2808SDK
        error = request_gpio_irq(WAKEUP_KEY_PORT,rk28_AD_irq_handler,GPIOEdgelFalling,NULL);
	if(error)
	{
		printk("unable to request recover key IRQ\n");
		goto failed2;
	}	
#endif        

#ifdef CONFIG_MACH_PWS700AA
    error = request_gpio_irq(GPIOPortE_Pin3,rk28_AD_irq_handler,GPIOEdgelRising,NULL);
    GPIOPullUpDown(GPIOPortE_Pin3,GPIOPullDown);    
	if(error)
	{
		printk("unable to request recover key IRQ\n");
		goto failed2;
	}	
#endif        

	return 0;

failed2:
	input_unregister_device(AD_button->input_dev);
	platform_set_drvdata(pdev, NULL);	
failed1:
	input_free_device(input_dev);
	kfree(AD_button);
	return error;
}

static int __devexit rk28_AD_button_remove(struct platform_device *pdev)
{
	struct rk28_AD_button *AD_button = platform_get_drvdata(pdev);

	input_unregister_device(AD_button->input_dev);
	input_free_device(AD_button->input_dev);
	kfree(AD_button);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver rk28_AD_button_driver = {
	.probe		= rk28_AD_button_probe,
	.remove 	= __devexit_p(rk28_AD_button_remove),
	.driver 	= {
		.name	= "rk28_AD_button",
		.owner	= THIS_MODULE,
	},
};

 int __init rk28_AD_button_init(void)
{
	return platform_driver_register(&rk28_AD_button_driver);
}
static void __exit rk28_AD_button_exit(void)
{
	platform_driver_unregister(&rk28_AD_button_driver);
}



//fs_initcall(rk28_AD_button_init);
module_init(rk28_AD_button_init);
//device_initcall(rk28_AD_button_init);
module_exit(rk28_AD_button_exit);

MODULE_DESCRIPTION("rk28 AD button Controller Driver");
MODULE_LICENSE("GPL");
