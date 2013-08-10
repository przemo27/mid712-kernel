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


/* Debug */
#if 1
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


	int IoCode[4][2]={
		GPIOPortB_Pin4,AD1KEY2,	// 108
		GPIOPortB_Pin5,AD1KEY1,	// 105
		GPIOPortB_Pin6,AD1KEY3,	// 106
		GPIOPortB_Pin7,AD1KEY4,	// 103
	};

#define AD2KEY1		    114   //VOLUME_DOWN  59	//MENU			//115   //VOLUME_UP
#define AD2KEY2 		115   //VOLUME_UP  102   //HOME			//114   //VOLUME_DOWN
#define AD2KEY3 		59	  //MENU   158	//BACK----ESC	//59	//MENU	
#define AD2KEY4 		102   //HOME   114   //VOLUME_DOWN	//62   //ENDCALL  		
#define AD2KEY5 		158	  //BACK----ESC 115   //VOLUME_UP	//158	//BACK------------ESC
#define AD2KEY6 		28    //ENTER  116	//POWER
/*
#define KEY_VOLUMEDOWN	114
#define KEY_VOLUMEUP		115
#define KEY_HOME			102
#define KEY_UP				103
#define KEY_LEFT				105
#define KEY_RIGHT			106
#define KEY_DOWN			108
#define KEY_KPENTER			96
#define KEY_POWER			116
#define KEY_ESC				158
#define KEY_BACK			1
#define KEY_MENU			59
#define KEY_Confirm			28
*/
#define KEY_VOLUMEDOWN	114
#define KEY_VOLUMEUP		115
#define KEY_HOME			102
#define KEY_UP				103
#define KEY_LEFT				105
#define KEY_RIGHT			106
#define KEY_DOWN			108
#define KEY_KPENTER			96
#define KEY_POWER			116
#define KEY_ESC				158
#define KEY_BACK			1
#define KEY_MENU			59
#define KEY_Confirm			28

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
#define Valuedrift		50
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
static unsigned  int g_wake =0;

extern int rk28_pm_status ;


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
	{899,AD1KEY6},

	{EmptyADValue,0}
};
/*  ipad
94   250
*/
/*
static  ADC_keyst ad2valuetab[] = {
	{94, KEY_HOME},

	{250, KEY_MENU},

	{EmptyADValue,0}

};
*/
/*  y1006
	547   430    318     205

					93
					
					250
					
					power
	
*/
static  ADC_keyst ad2valuetab[] = {
	{547, KEY_LEFT},
	{430, KEY_RIGHT},
	{318, KEY_HOME},		// home
	{205, KEY_MENU},		//MENU

	{93, KEY_UP},
	{250, KEY_DOWN},

	{EmptyADValue,0}

};



//key code tab
static unsigned char initkey_code[ ] = 
{
	AD1KEY1, AD1KEY2,AD1KEY3,AD1KEY4,AD1KEY5,AD1KEY6,
	AD2KEY1, AD2KEY2,AD2KEY3,AD2KEY4,AD2KEY5,AD2KEY6,ENDCALL	,KEY_WAKEUP
};


struct rk28_AD_button_platform_data {
	int x;
	
};

struct rk28_AD_button {
	struct rk28_AD_button_platform_data *pdata;
	struct timer_list timer;
	struct timer_list timer1;

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
		//	if((advalue>ptab->adc_value-Valuedrift)&&(advalue<ptab->adc_value+Valuedrift))
			if(abs(advalue-ptab->adc_value)<Valuedrift)
				return ptab->adc_keycode;
			ptab++;
		}
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
UINT32 RemoteKeybak=0;
static void rk28_adkeyscan_timer(unsigned long data)
{
	unsigned int ADKEY1,code = 0,i;
	/*Enable  AD controller to sample */
	prockAD_button->timer.expires  = jiffies+msecs_to_jiffies(10);
	add_timer(&prockAD_button->timer);
	RockAdcScanning();
	if (ADSampleTimes < 4)
	{
		ADSampleTimes ++;		
		goto scan_io_key;  	/* scan gpio button event*/
	}
	ADSampleTimes = 0;	
	/*Get button value*/
	ADKEY1=get_rock_adc1();
	if(ADKEY1<EmptyADValue)
		printk("\n ADC1 value=%d  \n",ADKEY1);
#if 1	//ffhh
	if((ADKEY1>EmptyADValue)&&(ADKEY1<=InvalidADValue))  
		goto scan_io_key1;
#endif	
	valuecount++;
 	if(valuecount < 2)
		goto scan_code;	
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
//	if(!GPIOGetPinLevel(GPIOPortF_Pin0))
	if(!GPIOGetPinLevel(GPIOPortE_Pin2))
	{
		pwrscantimes += 1;
		if(pwrscantimes == (SEC_NUM * ONESEC_TIMES))
		{
			input_report_key(prockAD_button->input_dev,ENDCALL,1);
			input_sync(prockAD_button->input_dev);
			DBG("the kernel come to power down!!!\n");
		
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
		if(rk28_pm_status == 0)
		{
			if(g_wake == 1)	/*already wake up*/
			{
				g_wake = 0;
				return;
			}
			input_report_key(prockAD_button->input_dev,AD1KEY5,1);
			input_sync(prockAD_button->input_dev);
			input_report_key(prockAD_button->input_dev,AD1KEY5,0);
			input_sync(prockAD_button->input_dev);
			
		}
		 rk28printk("\n%s^^^^Wake Up ^^^^^!!\n",__FUNCTION__);
	}
}
void rk28_send_wakeup_key( void ) 
{
        input_report_key(prockAD_button->input_dev,KEY_WAKEUP,1);
        input_sync(prockAD_button->input_dev);
        input_report_key(prockAD_button->input_dev,KEY_WAKEUP,0);
        input_sync(prockAD_button->input_dev);
}

//a@nzy
static irqreturn_t rk28_AD_irq_handler(s32 irq, void *dev_id)
{
	rk28printk("\n =================================rk28_AD_irq_handler===================================\n");

   if( rk28_pm_status == 1)
    {
    	input_report_key(prockAD_button->input_dev,AD1KEY5,1);
    	input_sync(prockAD_button->input_dev);
    	input_report_key(prockAD_button->input_dev,AD1KEY5,0);
    	input_sync(prockAD_button->input_dev);
	g_wake =1;
    }
	rk28printk("\n%s^^^^Wake Up ^^^^^!!\n",__FUNCTION__);
    return IRQ_HANDLED;
}
#if 1
// TODO:  remote key code
#define RC_DEBUG
typedef enum
{
//	REMOTE_Idle = 0,
	REMOTE_PrePlus = 0,
	REMOTE_Code,
//	REMOTE_EndPlus,
	REMOTE_Max
} REMOTE_STATE;
typedef struct _REMOTECTRL
{
	UINT32 state;
	UINT32 code;
//	UINT16 repeat;
	UINT16 scanCode;
	UINT32 dataCnt;
	UINT32 bPress;
	HTIMER TimerID;
	UINT32 Error;
#ifdef RC_DEBUG
	UINT32 IntCnt;
	INT32 time[200];
	UINT32 PinLevel;
#endif
} REMOTECTRL, * PREMOTECTRL;
typedef struct _RCKEY_TRANSLATION
{
    UINT8 keycode;        
    UINT16 scanCode;      
	 
} RCKEY_TRANSLATION;
typedef struct      
{
	 UINT16 usercode;
    UINT16 count;
    RCKEY_TRANSLATION* keyTrans;
}  RCKEY_BOARD;


#define TIME_BIT0_MIN		75		/*Bit0  1.125ms*/  
#define TIME_BIT0_MAX		145

#define TIME_BIT1_MIN		185		/*Bit1  2.25ms*/  
#define TIME_BIT1_MAX		265

#define TIME_PRE_MIN		1300//	490		/*PrePlus 5.05ms 4.5+0.56*/ 
#define TIME_PRE_MAX		1400//	550

#define TIME_RPT_MIN 		270		/*Repeat  2.25 +0.56*/ 
#define TIME_RPT_MAX 		310
#define NUMBER_OF_BIT		32
#define IS_PREPLUS(time)   (time > TIME_PRE_MIN && time < TIME_PRE_MAX)
#define IS_REPEAT(time)		(time > TIME_RPT_MIN && time < TIME_RPT_MAX)

#define IS_BIT_0(time)		(time > TIME_BIT0_MIN && time < TIME_BIT0_MAX)
#define IS_BIT_1(time)     (time > TIME_BIT1_MIN && time < TIME_BIT1_MAX)

#define TIME_REPEAT_H_MIN	1070
#define TIME_REPEAT_H_MAX	1200

#define TIME_REPEAT_L_MIN	9580
#define TIME_REPEAT_L_MAX	9780

#define IS_REPEAT_H(time)		(time > TIME_REPEAT_H_MIN  && time < TIME_REPEAT_H_MAX)
#define IS_REPEAT_L(time)		(time  >  TIME_REPEAT_L_MIN && time < TIME_REPEAT_L_MAX)

static REMOTECTRL g_RemoteCtrl = {0};
static UINT16 usercode = 0;
static UINT16 Tkeycode = 0;
#if 0
#define REMOTE_USERCODE   0x1fe                         /*本机的用户码*/
static RCKEY_TRANSLATION gRCKeyTran_01FE[]  = {
	{183,	AD1KEY5	},	//POWER
	{135,AD2KEY3	},		//PLAY
	{127,AD2KEY5	},		//ESC
	{63,AD2KEY4	},		//HOME
	{95,AD1KEY1	},		//UP
	{39,AD1KEY4	},		//DN
	{31,AD1KEY3	},		//FFW
	{111,AD1KEY2	},		//FFD
	{239,AD1KEY6	},		//ENTER
};
#else
#define REMOTE_USERCODE   255                        /*本机的用户码*/
/*
	255		223
		95
	207		199
	143	191	167
	63	127	159
	55	247	119
	39	231	103
*/
static RCKEY_TRANSLATION gRCKeyTran_01FE[]  = {
	{255,	KEY_POWER},		//POWER
	{223,	KEY_MUTE},		//POWER
	{95,		KEY_UP},		//PLAY
	{207,	KEY_LEFT},		//PLAY
	{199,	KEY_RIGHT},		//ESC
	{143,	KEY_ENTER},		//HOME
	{191,	KEY_DOWN	},		//UP
	{167,	KEY_ESC	},		//DN
	{63,		KEY_VOLUMEDOWN	},		//FFW
	{127,	KEY_VOLUMEUP	},		//FFD
	{159,	KEY_PAUSE	},		//ENTER
	{55,		KEY_PAGEUP	},		//ENTER
	{247,	KEY_PAGEDOWN	},		//ENTER
	{119,	KEY_MENU},		//ENTER
	{39,		KEY_KPLEFTPAREN	},		//ENTER
	{231,	KEY_KPRIGHTPAREN	},		//ENTER
	{103,	KEY_HOME},		//ENTER
};

#endif

#include "../../../include/linux/time.h"

struct time10usval {
	time_t		tv_sec;		/* seconds */
	suseconds_t	tv_10usec;	/* microseconds */
};
int GetRemoteKeyCode(int keynum){
	int i;
	for(i=0;i<ARRSIZE(gRCKeyTran_01FE);i++){
		if(gRCKeyTran_01FE[i].keycode==keynum)
			return gRCKeyTran_01FE[i].scanCode;
	}
}
void do10us_gettimeofday(struct time10usval *tv)
{
	struct timespec now;

	getnstimeofday(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_10usec = now.tv_nsec/10000;
}
static 	spinlock_t	lockRemote;

void RemoteCtrlHandle(int irq,void *dev_id)
{
#if 1
	static UINT32 tick1 = 0;
	static UINT32 RepeatFlag = 0;
	static UINT32 pulseCnt = 0;
	UINT32 tick2 = 0;
	UINT32 time = 0;
	int keycode=0;
	int i;
	unsigned long flags;
static struct time10usval nowtimevalprev;

//此处计时容易溢出，需要注意	
struct time10usval nowtimeval;
	spin_lock_irqsave(&lockRemote,flags);
	
/*	if (tick1 == 0)
	{
		tick1 = 1;

		do10us_gettimeofday(&nowtimevalprev);

		return;
	}
	else*/
	{
		do10us_gettimeofday(&nowtimeval);
		if(unlikely(nowtimeval.tv_sec<nowtimevalprev.tv_sec))
			time=nowtimeval.tv_10usec+100000-nowtimevalprev.tv_10usec;
		else
			time=nowtimeval.tv_10usec-nowtimevalprev.tv_10usec;
		nowtimevalprev.tv_sec=nowtimeval.tv_sec;
		nowtimevalprev.tv_10usec=nowtimeval.tv_10usec;
	}
//			printk("ucode=%ld, keycode=%ld \n",nowtimevalprev.tv_sec,nowtimevalprev.tv_10usec);
	if(IS_PREPLUS(time)){
		g_RemoteCtrl.IntCnt =0;
		RemoteKey=0;
		pulseCnt++;
		RepeatFlag=0;
	}
#if 1//def RC_DEBUG
	if (g_RemoteCtrl.IntCnt <= NUMBER_OF_BIT)	//0:start byte  33 byte	NUMBER_OF_BIT
	{
		g_RemoteCtrl.time[g_RemoteCtrl.IntCnt++] = time;
	}else{
		if(RepeatFlag){	//处理连续按键
			if(IS_REPEAT_H(time)&&(RepeatFlag==1)){
				RepeatFlag=2;
			}else if(IS_REPEAT_L(time)&&(RepeatFlag==2)){
				keycode=GetRemoteKeyCode(RemoteKey&0xff);
				input_report_key(prockAD_button->input_dev,keycode,1);
				input_sync(prockAD_button->input_dev);
				input_report_key(prockAD_button->input_dev,keycode,0);
				input_sync(prockAD_button->input_dev);
				RepeatFlag=1;
			}
			return;
		}
				RemoteKey=0;
		for(i=0;i<=NUMBER_OF_BIT;i++){		//NUMBER_OF_BIT 
			if(IS_BIT_0(g_RemoteCtrl.time[i]) ){
				RemoteKey<<=1;
			}else if(IS_BIT_1(g_RemoteCtrl.time[i]) ) {
				RemoteKey<<=1;
				RemoteKey|=1;
			}
		}		
		RemoteKeybak=1;
			if(REMOTE_USERCODE!=(RemoteKey>>16)) return;
			keycode=GetRemoteKeyCode(RemoteKey&0xff);
			input_report_key(prockAD_button->input_dev,keycode,1);
			input_sync(prockAD_button->input_dev);
			input_report_key(prockAD_button->input_dev,keycode,0);
			input_sync(prockAD_button->input_dev);
			RepeatFlag=1;
	}
#elif 0
	if(IS_PREPLUS(time)){
		g_RemoteCtrl.IntCnt =0;
		RemoteKey=0;
		pulseCnt++;
		return;
	}
	if (g_RemoteCtrl.IntCnt < NUMBER_OF_BIT)	//0:start byte  33 byte	NUMBER_OF_BIT
	{
		g_RemoteCtrl.time[g_RemoteCtrl.IntCnt++] = time;
	}else if(g_RemoteCtrl.IntCnt == NUMBER_OF_BIT){
		//	g_RemoteCtrl.IntCnt=0;
				RemoteKey=0;
		for(i=0;i<NUMBER_OF_BIT;i++){		//NUMBER_OF_BIT 
			if(IS_BIT_0(g_RemoteCtrl.time[i]) ){
				RemoteKey<<=1;
			}else if(IS_BIT_1(g_RemoteCtrl.time[i]) ) {
				RemoteKey<<=1;
				RemoteKey|=1;
			}
			printk("time=%d,num=%d  IS_PREPLUS=%d\n",g_RemoteCtrl.time[i],i,IS_PREPLUS(g_RemoteCtrl.time[i]));
		}		
			if(REMOTE_USERCODE!=(RemoteKey>>16)) return;
			keycode=GetRemoteKeyCode(RemoteKey&0xff);
			input_report_key(prockAD_button->input_dev,keycode,1);
			input_sync(prockAD_button->input_dev);
			input_report_key(prockAD_button->input_dev,keycode,0);
			input_sync(prockAD_button->input_dev);
			printk("keycode=%d\n",keycode);
//			printk("ucode=%d, keycode=%d keynum=%d\n",RemoteKey>>16,RemoteKey&0xff, pulseCnt);
	}

#else

	int errflag=0;
	if(g_RemoteCtrl.IntCnt<=32){
		g_RemoteCtrl.time[g_RemoteCtrl.IntCnt++] = time;
	}else{
		g_RemoteCtrl.IntCnt=0;
		for(i=0;i<=32;i++){		//NUMBER_OF_BIT 
			if(IS_BIT_0(g_RemoteCtrl.time[i]) ){
				RemoteKey<<=1;
			}else if(IS_BIT_1(g_RemoteCtrl.time[i]) ) {
				RemoteKey<<=1;
				RemoteKey|=1;
			}else{
				if(i>0){
					RemoteKey=0xffffff;
		//		printk("erro, time=%d \n",g_RemoteCtrl.time[i]);
				errflag=1;
			//		break;

				}
			}
			printk("num=%d  time=%d,IS_PREPLUS=%d\n",i ,g_RemoteCtrl.time[i],IS_PREPLUS(g_RemoteCtrl.time[i]));
		}		
		if(errflag)
			printk("errocode\n");
		else
			printk("ucode=%d, keycode=%d \n",RemoteKey>>16,RemoteKey&0xff);
	}

#endif
	spin_unlock_irqrestore(&lockRemote, flags);
	

#endif
}

#endif
//int wifi_turn_on_card(void);
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
// wifi power pin
//	GPIOSetPinDirection(GPIOPortF_Pin5,GPIO_OUT);
//	GPIOPullUpDown(GPIOPortF_Pin5,GPIOPullUp);
//	GPIOSetPinLevel(GPIOPortF_Pin5,GPIO_HIGH);
	
//	wifi_turn_on_card();//ffhh added
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
#if 1

	GPIOSetPinDirection(GPIOPortE_Pin1,GPIO_IN);
	GPIOSetPinLevel(GPIOPortE_Pin1,GPIO_HIGH);

//	setup_timer(&AD_button->timer1, RCTimerCallback, (unsigned long)AD_button);
//	AD_button->timer.expires  = jiffies + 23;
//	add_timer(&AD_button->timer1);
	error = request_gpio_irq(GPIOPortE_Pin1,RemoteCtrlHandle,GPIOEdgelFalling,NULL);//
	if(error)
	{
		printk("unable to request remote key IRQ\n");
		goto fail3;
	}else{
		printk("GET remote  IRQ\n");
	}
	

#endif
	setup_timer(&AD_button->timer, rk28_adkeyscan_timer, (unsigned long)AD_button);
	//mod_timer(&AD_button->timer, 100);
	AD_button->timer.expires  = jiffies + 3;
	add_timer(&AD_button->timer);
#if 1	//ffhh
        error = request_gpio_irq(GPIOPortE_Pin2,rk28_AD_irq_handler,GPIOEdgelFalling,NULL);
	if(error)
	{
		printk("unable to request recover key IRQ\n");
		goto failed2;
	}	
#endif        

	return 0;
fail3:

	free_irq(7,NULL);

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

//a@nzy
static int rk28_AD_button_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
    pADC_REG pheadAdc;
	unsigned long flags;    
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;
	
	local_irq_save(flags);
    pheadAdc->ADC_CTRL |= ADC_ENABLED_INT;
	local_irq_restore(flags);
#endif
	printk("IN AD suspend !!\n");
	return 0;
}

//a@nzy
static int rk28_AD_button_resume(struct platform_device *pdev)
{
#if 0
    pADC_REG pheadAdc;
	unsigned long flags;    
    pheadAdc = (pADC_REG) ADC_BASE_ADDR_VA;

	local_irq_save(flags);
    pheadAdc->ADC_CTRL |= ADC_START;
	local_irq_restore(flags);
#endif
	printk("IN AD resume !!\n");

    return 0;
}

static struct platform_driver rk28_AD_button_driver = {
	.probe		= rk28_AD_button_probe,
	.remove 	= __devexit_p(rk28_AD_button_remove),
	.driver 	= {
		.name	= "rk28_AD_button",
		.owner	= THIS_MODULE,
	},
 //   .suspend    = rk28_AD_button_suspend,
  //  .resume     = rk28_AD_button_resume,
};

 int __init rk28_AD_button_init(void)
{
	return platform_driver_register(&rk28_AD_button_driver);
}

static void __exit rk28_AD_button_exit(void)
{
	platform_driver_unregister(&rk28_AD_button_driver);
}

fs_initcall(rk28_AD_button_init);
module_exit(rk28_AD_button_exit);

MODULE_DESCRIPTION("rk28 AD button Controller Driver");
MODULE_LICENSE("GPL");
