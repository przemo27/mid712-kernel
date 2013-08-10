/*
 * linux/drivers/input/keyboard/rock28_keypad.c
 *
 * Driver for the rock28 matrix keyboard controller.
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

#include <asm/arch/hardware.h>
/*
 * Keypad Controller registers
 */
#include <asm/arch/rk28_debug.h>

//ROCKCHIP AD KEY CODE ,for demo board
//      key		--->	EV	
#define AD1KEY1		KEY_BACKSPACE
#define AD1KEY2 		KEY_BACKSPACE		
#define AD1KEY3 		KEY_3		
#define AD1KEY4 		KEY_4		
#define AD1KEY5			KEY_5

#define AD2KEY1			KEY_ESC
#define AD2KEY2 		KEY_7
#define AD2KEY3 		KEY_8
#define AD2KEY4 		KEY_9
#define AD2KEY5 		KEY_0
#define AD2KEY6 		KEY_W

#define Valuedrift		50
#define ADEmpty			1000
#define ADKEYNum		11

#define KEY_PHYS_NAME	"rockchip-keypad/input0"
//ADC Registers
typedef  struct tagADC_keyst
{
	unsigned int adc_value;
	unsigned int adc_keycode;
}ADC_keyst,*pADC_keyst;

//	adc	 ---> key	
static  ADC_keyst ad1valuetab[] = {
	{95, AD1KEY1},
	{247,AD1KEY2},
	{403,AD1KEY3},
	{561,AD1KEY4},
	{723,AD1KEY5},
	
	{ADEmpty,0}
};
static  ADC_keyst ad2valuetab[] = {
	{95,  AD2KEY1},
	{249, AD2KEY2},
	{406, AD2KEY3},
	{561, AD2KEY4},
	{726, AD2KEY5},
	{899, AD2KEY6},
	
	{ADEmpty,0}

};



//key code tab
static unsigned char initkey_code[10] = {
	AD1KEY1, AD1KEY2,AD1KEY3,AD1KEY4,AD1KEY5,
	AD2KEY1,AD2KEY2,AD2KEY3,AD2KEY4,AD2KEY5,AD2KEY6	
};

/*
static unsigned char initkey_code[10] = {
	58,59,60,61,62,63,64,65,66,67
};*/




struct rock28_keypad_platform_data {
	int x;
	
};

struct rock28_keypad {
	struct rock28_keypad_platform_data *pdata;
	struct timer_list timer;

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;

	/* matrix key code map */
	unsigned char keycodes[10];

	/* state row bits of each column scan */
//	uint32_t matrix_key_state[MAX_MATRIX_KEY_COLS];
	uint32_t direct_key_state;
	unsigned int direct_key_mask;
	int rotary_rel_code[2];
	int rotary_up_key[2];
	int rotary_down_key[2];
};

 struct rock28_keypad *prockkeypad;


extern 	RockAdcScanning();
extern 	printADCValue();
extern int32 ADCInit(void);
extern 	int get_rock_adc1(void);
extern	int get_rock_adc2(void);




unsigned int find_rock_adkeycode(unsigned int advalue,pADC_keyst ptab)
{

	
	while(ptab->adc_value!=ADEmpty)
		{
			if((advalue>ptab->adc_value-Valuedrift)&&(advalue<ptab->adc_value+Valuedrift))
				return ptab->adc_keycode;
			ptab++;
		}
	
	return 0;

}


//---------------------------------------------------------------
static void report_rotary_event(struct rock28_keypad *keypad, int r, int delta)
{
	struct input_dev *dev = keypad->input_dev;

/*
	if (delta == 0)
		return;

	if (keypad->rotary_up_key[r] && keypad->rotary_down_key[r]) {
		int keycode = (delta > 0) ? keypad->rotary_up_key[r] :
						keypad->rotary_down_key[r];

		// simulate a press-n-release 
		input_report_key(dev, keycode, 1);
		input_sync(dev);
		input_report_key(dev, keycode, 0);
		input_sync(dev);
	} else {
		input_report_rel(dev, keypad->rotary_rel_code[r], delta);
		input_sync(dev);
	}

*/	
}

static void rock28_keypad_scan_rotary(struct rock28_keypad *keypad)
{
/*
	struct rock28_keypad_platform_data *pdata = keypad->pdata;
	uint32_t kprec;

	// read and reset to default count value 
	kprec = keypad_readl(KPREC);
	keypad_writel(KPREC, DEFAULT_KPREC);

	if (pdata->enable_rotary0)
		report_rotary_event(keypad, 0, rotary_delta(kprec));

	if (pdata->enable_rotary1)
		report_rotary_event(keypad, 1, rotary_delta(kprec >> 16));
*/
	
}

static void rock28_keypad_scan_direct(struct rock28_keypad *keypad)
{
/*
	struct rock28_keypad_platform_data *pdata = keypad->pdata;
	unsigned int new_state;
	uint32_t kpdk, bits_changed;
	int i;

	kpdk = keypad_readl(KPDK);

	if (pdata->enable_rotary0 || pdata->enable_rotary1)
		rock28_keypad_scan_rotary(keypad);

	if (pdata->direct_key_map == NULL)
		return;

	new_state = KPDK_DK(kpdk) & keypad->direct_key_mask;
	bits_changed = keypad->direct_key_state ^ new_state;

	if (bits_changed == 0)
		return;

	for (i = 0; i < pdata->direct_key_num; i++) {
		if (bits_changed & (1 << i))
			input_report_key(keypad->input_dev,
					pdata->direct_key_map[i],
					(new_state & (1 << i)));
	}
	input_sync(keypad->input_dev);
	keypad->direct_key_state = new_state;
*/
	
}

static irqreturn_t rock28_keypad_irq_handler(int irq, void *dev_id)
{
	struct rock28_keypad *keypad = dev_id;
//	unsigned long kpc = keypad_readl(KPC);
/*
	if (kpc & KPC_DI)
		rock28_keypad_scan_direct(keypad);

	if (kpc & KPC_MI)
		rock28_keypad_scan_matrix(keypad);
*/
	return IRQ_HANDLED;
}

static void rock28_keypad_config(struct rock28_keypad *keypad)
{
	struct rock28_keypad_platform_data *pdata = keypad->pdata;
	unsigned int mask = 0, direct_key_num = 0;
	unsigned long kpc = 0;
/*
	// enable matrix keys with automatic scan /
	if (pdata->matrix_key_rows && pdata->matrix_key_cols) {
		kpc |= KPC_ASACT | KPC_MIE | KPC_ME | KPC_MS_ALL;
		kpc |= KPC_MKRN(pdata->matrix_key_rows) |
			   KPC_MKCN(pdata->matrix_key_cols);
	}

	// enable rotary key, debounce interval same as direct keys /
	if (pdata->enable_rotary0) {
		mask |= 0x03;
		direct_key_num = 2;
		kpc |= KPC_REE0;
	}

	if (pdata->enable_rotary1) {
		mask |= 0x0c;
		direct_key_num = 4;
		kpc |= KPC_REE1;
	}

	if (pdata->direct_key_num > direct_key_num)
		direct_key_num = pdata->direct_key_num;

	keypad->direct_key_mask = ((2 << direct_key_num) - 1) & ~mask;

	// enable direct key 
	if (direct_key_num)
		kpc |= KPC_DE | KPC_DIE | KPC_DKN(direct_key_num);

	keypad_writel(KPC, kpc | KPC_RE_ZERO_DEB);
	keypad_writel(KPREC, DEFAULT_KPREC);
	keypad_writel(KPKDI, pdata->debounce_interval);
*/	
}

static int rock28_keypad_open(struct input_dev *dev)
{
	struct rock28_keypad *keypad = input_get_drvdata(dev);


	return 0;
}

static void rock28_keypad_close(struct input_dev *dev)
{
	struct rock28_keypad *keypad = input_get_drvdata(dev);

}

#ifdef CONFIG_PM
static int rock28_keypad_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rock28_keypad *keypad = platform_get_drvdata(pdev);

	clk_disable(keypad->clk);
	return 0;
}

static int rock28_keypad_resume(struct platform_device *pdev)
{
	struct rock28_keypad *keypad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = keypad->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		/* Enable unit clock */
		clk_enable(keypad->clk);
		rock28_keypad_config(keypad);
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#else
#define rock28_keypad_suspend	NULL
#define rock28_keypad_resume	NULL
#endif

#define res_size(res)	((res)->end - (res)->start + 1)

int keydata=58;
unsigned int ADSampleTimes = 0;

static void rock28_adkeyscan_timer(unsigned long data)
{

	unsigned int ADKEY1,ADKEY2,code;

//	setup_timer(&keypad->timer, rock28_kp_timer, (unsigned long)keypad);
	//mod_timer(&keypad->timer, 100);
	prockkeypad->timer.expires  = jiffies+2;
	add_timer(&prockkeypad->timer);
	RockAdcScanning();
	
//	printADCValue();	

//	printk("*****ADKEY1==%d,ADKEY2 ==%d\n",ADKEY1,ADKEY2);

	//input_report_key(prockkeypad->input_dev,keydata++,0x1);
	//input_sync(prockkeypad->input_dev);
	//return ;

	if((ADKEY1>ADEmpty)&&(ADKEY2>ADEmpty))
		return;
	if (ADSampleTimes < 4)
	{
		ADSampleTimes ++;
		return;
	}
	ADKEY1=get_rock_adc1();
	ADKEY2=get_rock_adc2();
	ADSampleTimes = 0;
	code=find_rock_adkeycode(ADKEY1,ad1valuetab);
	if(code)
	{
		rk28printk("\n rock adc1 key scan ,find a key=%d  \n",code);
		input_report_key(prockkeypad->input_dev,code,1);
		input_sync(prockkeypad->input_dev);
		input_report_key(prockkeypad->input_dev,code,0);
		input_sync(prockkeypad->input_dev);
		return;
	}

	code=find_rock_adkeycode(ADKEY2,ad2valuetab);
	if(code)
	{
		rk28printk("\n rock adc2 key scan ,find a key=%d  \n",code);
		input_report_key(prockkeypad->input_dev,code,1);
		input_sync(prockkeypad->input_dev);
		input_report_key(prockkeypad->input_dev,code,0);
		input_sync(prockkeypad->input_dev);
		return;
	}
	
	rk28printk("\n rock adc key scan ,there is no key,ad1=%d,ad2=%d",ADKEY1,ADKEY1);


			


}


static int __devinit rock28_keypad_probe(struct platform_device *pdev)
{
	struct rock28_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq, error,i;

	//printk("\n___________________rock28_keypad_probe___________________\n");
	keypad = kzalloc(sizeof(struct rock28_keypad), GFP_KERNEL);
	if (keypad == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}
	
	memcpy(keypad->keycodes, initkey_code, sizeof(keypad->keycodes));


	/* Create and register the input driver. */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		error = -ENOMEM;
		goto failed_put_clk;
	}

	input_dev->name = pdev->name;
	//input_dev->id.bustype = BUS_HOST;
	input_dev->open = rock28_keypad_open;
	input_dev->close = rock28_keypad_close;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_dev->keycode = keypad->keycodes;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(initkey_code);
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	keypad->input_dev = input_dev;
	input_set_drvdata(input_dev, keypad);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;

//	rock28_keypad_build_keycode(keypad);
	platform_set_drvdata(pdev, keypad);

	ADCInit();
	prockkeypad=keypad;

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq;
	}

	setup_timer(&keypad->timer, rock28_adkeyscan_timer, (unsigned long)keypad);
	//mod_timer(&keypad->timer, 100);
	keypad->timer.expires  = jiffies+30;
	add_timer(&keypad->timer);

	return 0;

failed_free_irq:
	free_irq(irq, pdev);
	platform_set_drvdata(pdev, NULL);
failed_free_dev:
	input_free_device(input_dev);
failed_put_clk:
//	clk_put(keypad->clk);
failed_free_io:
	iounmap(keypad->mmio_base);
failed_free_mem:
	release_mem_region(res->start, res_size(res));
failed_free:
	kfree(keypad);
	return error;
}

static int __devexit rock28_keypad_remove(struct platform_device *pdev)
{
	struct rock28_keypad *keypad = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(platform_get_irq(pdev, 0), pdev);

//	clk_disable(keypad->clk);
//	clk_put(keypad->clk);

	input_unregister_device(keypad->input_dev);
	input_free_device(keypad->input_dev);

	iounmap(keypad->mmio_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(keypad);
	return 0;
}

static struct platform_driver rock28_keypad_driver = {
	.probe		= rock28_keypad_probe,
	.remove 	= __devexit_p(rock28_keypad_remove),
	.suspend	= rock28_keypad_suspend,
	.resume 	= rock28_keypad_resume,
	.driver 	= {
		.name	= "rock28-keypad",
		.owner	= THIS_MODULE,
	},
};

 int __init rock28_keypad_init(void)
{
	int tmp;
	


	tmp=platform_driver_register(&rock28_keypad_driver);
//	printk("\n___________________rock28_keypad_init___________________ result=%d  \n",tmp);

	return tmp;
}

static void __exit rock28_keypad_exit(void)
{
	platform_driver_unregister(&rock28_keypad_driver);
}

module_init(rock28_keypad_init);
module_exit(rock28_keypad_exit);

MODULE_DESCRIPTION("rockchip Keypad Controller Driver");
MODULE_LICENSE("GPL");
