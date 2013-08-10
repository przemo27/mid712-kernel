/****************************************************************************************
 * driver/input/touchscreen/rk28_i2c_tsgtt8205s.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: 	wqq
 *Date		: 2009-07-15
 *This driver use for rk28 chip extern touchscreen. Use i2c IF ,the chip is GTT8205s
 ********************************************************************************************/
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/hw_define.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include <asm/arch/api_i2c.h>

/******************************************
		DEBUG
*** ***************************************/
//#define RK28_PRINT
#include <asm/arch/rk28_debug.h>

/*calibration parameter*/
#define GTT8205_TOUCH_AD_LEFT			 0
#define GTT8205_TOUCH_AD_RIGHT		 2047
#define GTT8205_TOUCH_AD_TOP			 2047
#define GTT8205_TOUCH_AD_BOTTOM 	 	 0
#define LCD_MAX_LENGTH					 800
#define LCD_MAX_WIDTH					 480
#define AD_TO_X(adx)	(LCD_MAX_LENGTH * (adx-GTT8205_TOUCH_AD_LEFT )/( GTT8205_TOUCH_AD_RIGHT-GTT8205_TOUCH_AD_LEFT ))
#define AD_TO_Y(ady)	(LCD_MAX_WIDTH * (ady - GTT8205_TOUCH_AD_BOTTOM)/(GTT8205_TOUCH_AD_TOP-GTT8205_TOUCH_AD_BOTTOM ))

/*touchscreen config parameter*/
#define SINGLETOUCH_MODE 		1
#define MULTITOUCH_MODE		0
#define GTT8205_IRQ             7
#define GTT8205_NAME    "GTT8205 touchscreen"

#define TS_POLL_DELAY   		(100*1000*1000)  /* ns delay before the first sample */
#define TS_POLL_PERIOD  		(120*1000*1000)  /* ns delay between samples */
#define GTT8250_I2C_ADDR   	0x15
#define INT_TEST_TIMES 		40
/*tochscreen private data*/
static u8 i2c_buf[10];

static int rk28_ts_gtt8205_probe(struct i2c_adapter *bus, int address, int kind);
struct single_touch_event{
	uint16 x;
	uint16 y;
};
struct multi_touch_event {
	uint16 x1;
	uint16 x2;
	uint16 y1;
	uint16 y2;
};
struct GTT8205_TS_EVENT{	
	struct i2c_client *client;
	struct input_dev *input;
	spinlock_t	lock;
	char	phys[32];
	int 		irq;
#if SINGLETOUCH_MODE
	struct single_touch_event singletouch_point;
#endif
#if MULTITOUCH_MODE
	struct multi_touch_event multitouch_point; 
#endif
	bool		pendown;
	int		counttimes;
	bool 	sleepstatus;
	struct timer_list timer;
	struct timer_list	timer1;	
	struct hrtimer	timer2;  	/*monitor interrupt pin*/
	struct work_struct   downwork;	/*report down event*/
	struct delayed_work work;	/*report second event*/
	
};
struct GTT8205_TS_EVENT *g_ts_dev;

/*read the gtt8205 register ,used i2c bus*/
static int gtt8205_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
	client->mode = TS8205MODE;
	client->Channel = I2C_CH1;
	client->speed = 80;
	buf[0] = reg;
	//printk("%s the slave i2c device mode == %d\n",__FUNCTION__,client->adapter->mode);
	ret = i2c_transfer(client->adapter, msgs, 1);
	
	if (ret > 0)
		ret = 0;	
	return ret;
}


/* set the gtt8205 registe,used i2c bus*/
static int gtt8205_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], unsigned short len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};
	client->mode = TS8205MODE;
	client->Channel = I2C_CH1;
	client->speed = 80;
	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}

static enum hrtimer_restart gtt8205_dotimer2(struct hrtimer *handle)
{

	struct GTT8205_TS_EVENT *ts_dev = container_of(handle, struct GTT8205_TS_EVENT, timer2);
	int32 PE3status = 0;
	u8  buf[12] ;		/*single touch*/
	int sr;
	rk28printk("************.....%s.....PE3status=%d  counttimes=%d pendown == %d\n",__FUNCTION__,PE3status,ts_dev->counttimes,ts_dev->pendown);
	
	if (ts_dev->sleepstatus == 1)
	   return HRTIMER_RESTART;
	
	PE3status =  GPIOGetPinLevel(GPIOPortE_Pin3);
	if(PE3status)
	{
		ts_dev->counttimes = 0;
		goto next;
	}
	ts_dev->counttimes++;		/*test interrupt status --max time 50ms*/
	if(ts_dev->counttimes > INT_TEST_TIMES)
	{
		ts_dev->counttimes = 0;
		/*reset interrupt status*/
		rockchip_mux_api_set(GPIOE_SPI1_SEL_NAME, IOMUXA_GPIO1_A1237);
		GPIOSetPinDirection(GPIOPortE_Pin3,GPIO_IN);
		//GPIOSetPinLevel(GPIOPortE_Pin3,GPIO_HIGH);
		gpio_irq_enable(GPIOPortE_Pin3);	
		//GPIOClrearInmarkIntr(GPIOPortE_Pin3);
		/*reset touchscreen*/
		rockchip_mux_api_set(GPIOF0_UART1_CPWM0_NAME, IOMUXA_GPIO1_B0);
		GPIOSetPinDirection(GPIOPortF_Pin0,GPIO_OUT);
		GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_HIGH);
		mdelay(50);
		GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_LOW);
		/*set touchscreen single touch mode*/
		buf[0] = 0x10;
		sr=gtt8205_set_regs(g_ts_dev->client, 0x07, buf,1);
		if(sr<0)	
			printk("\n--%s--I2C set gtt8205 singletouch mode error !!!\n",__FUNCTION__);
		printk("\n******%s  reset touchscreen as interrup error \n",__FUNCTION__);
	}

next:
	handle->expires = ktime_add(handle->expires, ktime_set(0,TS_POLL_DELAY));
	return HRTIMER_RESTART;
}

static void gtt8205_dotimer1(unsigned long data)
{

	struct GTT8205_TS_EVENT *ts_dev = (struct GTT8205_TS_EVENT *)data;
	struct input_dev *input;   
	
	if (ts_dev->sleepstatus == 1)
	    return;
	    
	input = ts_dev->input;       
	input_report_abs(input,ABS_X,ts_dev->singletouch_point.x);
	input_report_abs(input,ABS_Y,ts_dev->singletouch_point.y);	
	input_report_key(input,BTN_TOUCH,1);
	 input_sync(input);
	del_timer(&ts_dev->timer1);
	ts_dev->timer1.expires  = jiffies + 5;
	add_timer(&ts_dev->timer1);
	rk28printk("gtt8205_dotimer1  down  %dx=%dy=%d\n",jiffies,ts_dev->singletouch_point.x,ts_dev->singletouch_point.y);    
	return;
}
static void rk28_tsscan_timer(unsigned long data)
{
	struct GTT8205_TS_EVENT *ts_dev = (struct GTT8205_TS_EVENT *)data;
	struct input_dev *input;
	int sr;
	u8  buf[12];
	
	if (ts_dev->sleepstatus == 1)
	    return;
	    
	input = ts_dev->input;			
	rk28printk("The touchscreen next event down!!\n");
#if SINGLETOUCH_MODE
	sr=gtt8205_read_regs(ts_dev->client, 0, buf,6);
	if(sr<0)		
	{
		printk("\n---%s -I2C read gtt8205 err\n",__FUNCTION__);
		goto fail;
	}
	ts_dev->counttimes = 0;
	ts_dev->singletouch_point.x = (((((unsigned short)buf[1] )<< 7) )| buf[2] )& 0x7ff;
	ts_dev->singletouch_point.y = (((((unsigned short)buf[3] )<< 7) )| buf[4] )& 0x7ff;     	
	rk28printk("gtt8502 read position x==%d y=  %d\n",ts_dev->singletouch_point.x,ts_dev->singletouch_point.y);      
	ts_dev->singletouch_point.y =	2048 - ts_dev->singletouch_point.y;

#endif

#if MULTITOUCH_MODE
	sr=gtt8205_read_regs(ts_dev->client, 0, buf,12);
	if(sr<0)		
	{
			printk("\n---%s -I2C read gtt8205 err\n",__FUNCTION__);
			goto fail;
	}
	ts_dev->counttimes = 0;
	ts_dev->multitouch_point.x1 = (((((unsigned short)buf[4] )<< 7) )| buf[5] )& 0x7ff;
	ts_dev->multitouch_point.x2 = (((((unsigned short)buf[6] )<< 7) )| buf[7] )& 0x7ff;
	ts_dev->multitouch_point.y1 = (((((unsigned short)buf[8] )<< 7) )| buf[9] )& 0x7ff;
	ts_dev->multitouch_point.y2 = (((((unsigned short)buf[10] )<< 7) )| buf[11] )& 0x7ff;
	ts_dev->multitouch_point.y1 = 2048 - ts_dev->multitouch_point.y1;
	ts_dev->multitouch_point.y2 = 2048 - ts_dev->multitouch_point.y2;
      
#endif
#if SINGLETOUCH_MODE
	if(buf[0] == 0x80)
	{
              del_timer(&ts_dev->timer1);
		input_report_abs(input,ABS_X,ts_dev->singletouch_point.x);
		input_report_abs(input,ABS_Y,ts_dev->singletouch_point.y);	
		input_report_key(input,BTN_TOUCH,0);
                input_sync(input);
                rk28printk("rk28_tsscan_timer up %dx=%dy=%d\n",jiffies,ts_dev->singletouch_point.x,ts_dev->singletouch_point.y);			
		gpio_irq_enable(GPIOPortE_Pin3);
		return;
	}
	if(buf[0] == 0x81)	
	{
		ts_dev->pendown = 1;
              		
		input_report_abs(input,ABS_X,ts_dev->singletouch_point.x);
		input_report_abs(input,ABS_Y,ts_dev->singletouch_point.y);	
		input_report_key(input,BTN_TOUCH,1);	
		input_sync(input);
                rk28printk("rk28_tsscan_timer down %dx=%dy=%d\n",jiffies,ts_dev->singletouch_point.x,ts_dev->singletouch_point.y);
                del_timer(&ts_dev->timer1);
	        ts_dev->timer1.expires  = jiffies + 5;
	        add_timer(&ts_dev->timer1);               		
		gpio_irq_enable(GPIOPortE_Pin3);
		return;
	}	
#endif
#if MULTITOUCH_MODE
	if(ts_dev->pendown) 
	{
		input_report_abs(input,ABS_HAT0X,ts_dev->multitouch_point.x1);
		input_report_abs(input,ABS_HAT1X,ts_dev->multitouch_point.x2);
		input_report_abs(input,ABS_HAT0Y,ts_dev->multitouch_point.y1);
		input_report_abs(input,ABS_HAT1Y,ts_dev->multitouch_point.y2);
		input_sync(input);
	}
#endif
fail:
	gpio_irq_enable(GPIOPortE_Pin3);

	return;
}

static irqreturn_t gtt8205_ts_isr(int irq, void *dev_id)
{
	struct GTT8205_TS_EVENT *ts_dev ;
	ts_dev = dev_id;
	rk28printk("*******>the interrupt status==%d\n",GPIOGetPinLevel(GPIOPortE_Pin3));
	rk28printk("------->the reset status is ==%d\n",GPIOGetPinLevel(GPIOPortF_Pin0));
	if (ts_dev->sleepstatus == 1)
		return IRQ_HANDLED;
	gpio_irq_disable(GPIOPortE_Pin3);
	del_timer(&ts_dev->timer1);

	del_timer(&ts_dev->timer);
	ts_dev->timer.expires  = jiffies + 1;
	add_timer(&ts_dev->timer);

	return IRQ_HANDLED;
}
static int rk28_ts_gtt8205_detach_client(struct i2c_client *client)
{
	rk28printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
	return 0;
}

static void rk28_ts_gtt8205_shutdown(struct i2c_client *client)
{
	rk28printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
}


static unsigned short gtt8205_normal_i2c[] = {GTT8250_I2C_ADDR>>1,I2C_CLIENT_END};
static unsigned short gtt8205_ignore = I2C_CLIENT_END;

static struct i2c_client_address_data gtt8205_addr_data={
	.normal_i2c = gtt8205_normal_i2c,
	.probe = &gtt8205_ignore,
	.ignore =& gtt8205_ignore,
};
static int rk28_ts_gtt8205_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap,&gtt8205_addr_data,rk28_ts_gtt8205_probe);
}

static struct i2c_driver gtt8205_driver  = {
	.driver = {
		.name = "gtt8205s",
		.owner = THIS_MODULE,
	},
	.id = GTT8250_I2C_ADDR,
	.attach_adapter = &rk28_ts_gtt8205_attach_adapter,
	.detach_client 	=  &rk28_ts_gtt8205_detach_client,
	.shutdown     	=  &rk28_ts_gtt8205_shutdown,
};
static struct  i2c_client gtt8205_client = {
		.driver = &gtt8205_driver,
		.name	= "gtt8205s",
	};

#ifdef CONFIG_ANDROID_POWER
static void rk28_ts_suspend(android_early_suspend_t *h)
{
	u8 buf[12];
	int ret;
	/*set chip sleep and power save*/
	buf[0] = 0x00;
	g_ts_dev->sleepstatus = 1;
	
	gpio_irq_disable(GPIOPortE_Pin3);
	del_timer(&g_ts_dev->timer1);
	del_timer(&g_ts_dev->timer);
	//msleep(10);
	
	ret=gtt8205_set_regs(g_ts_dev->client, 0x07, buf,1);
	if(ret<0)	
	{
		g_ts_dev->sleepstatus = 0;
		printk("\n----%s##I2C set gtt8205 err sleepstatus = %d",__FUNCTION__,g_ts_dev->sleepstatus);
		return;
	}
	//GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_HIGH);//disable gpio intterrupt during sleep already
	printk("rk28 GTT8205 touchscreen enter suspend! sleepstatus = %d\n ",g_ts_dev->sleepstatus);
	rk28printk("------>the reset status is ==%d\n",GPIOGetPinLevel(GPIOPortF_Pin0));
}



static void rk28_ts_resume(android_early_suspend_t *h)

{
	u8  buf[12] ;		/*single touch*/
	int sr;
	if(g_ts_dev->sleepstatus == 0)
		g_ts_dev->sleepstatus = 1;
	GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_HIGH);
	mdelay(10);
	GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_LOW);
	mdelay(4);
	gpio_irq_enable(GPIOPortE_Pin3);
	//GPIOClrearInmarkIntr(GPIOPortE_Pin3);
	/*set touchscreen single touch mode*/
	buf[0] = 0x10;
	sr=gtt8205_set_regs(g_ts_dev->client, 0x07, buf,1);
	if(sr<0)	
		printk("\n--%s--I2C set gtt8205 singletouch mode error !!!\n",__FUNCTION__);
	printk("rk28 GTT8205 touchscreen enter resume!sleepstatus = %d\n ",g_ts_dev->sleepstatus);
	g_ts_dev->sleepstatus = 0;
}


static android_early_suspend_t ts_early_suspend;

#endif



static int rk28_ts_gtt8205_probe(struct i2c_adapter *bus, int address, int kind)

{


	struct GTT8205_TS_EVENT *ts_dev;

	struct input_dev *input;

	unsigned int err = 0;
	int sr;
	u8  buf[12] ;		/*single touch*/
	/*set PE3  IOMODE  use to screen intterrupt signal*/

	rockchip_mux_api_set(GPIOE_SPI1_SEL_NAME, IOMUXA_GPIO1_A1237);

	GPIOSetPinDirection(GPIOPortE_Pin3,GPIO_IN);

	GPIOSetPinLevel(GPIOPortE_Pin3,GPIO_HIGH);


	/*enable touchscreen PB2 pin*/

	//GPIOSetPinLevel(GPIOPortF_Pin0,GPIO_LOW);


	rk28printk("------>the intterrup status is ==%d\n",GPIOGetPinLevel(GPIOPortE_Pin3));

	rk28printk("------>the reset status is ==%d\n",GPIOGetPinLevel(GPIOPortF_Pin0));

	ts_dev=kzalloc(sizeof(struct GTT8205_TS_EVENT), GFP_KERNEL);

	if(!ts_dev)

	{

		rk28printk("failed to allocate memory!!\n");

		goto nomem;

	}

	gtt8205_client.adapter = bus;

	gtt8205_client.addr= GTT8250_I2C_ADDR>>1;//address;

	gtt8205_client.mode = TS8205MODE;

	gtt8205_client.Channel = I2C_CH1;
	gtt8205_client.speed = 80;

	gtt8205_client.addressBit=I2C_7BIT_ADDRESS_8BIT_REG;

	ts_dev->client=&gtt8205_client; 

	err = i2c_attach_client(&gtt8205_client);

	if (err < 0)

		 return err;	

	input = input_allocate_device();

	if(!input)

	{

		rk28printk("rk28 gtt8205_TS allocate input device failed!!!\n"); 

		goto fail1;

	}	

	ts_dev->input = input;

	/*init timer to dispose workqueue */
	setup_timer(&ts_dev->timer, rk28_tsscan_timer, (unsigned long)ts_dev);
	
//	hrtimer_init(&ts_dev->timer1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//	ts_dev->timer1.function = gtt8205_dotimer1;
	setup_timer(&ts_dev->timer1, gtt8205_dotimer1, (unsigned long)ts_dev);

	hrtimer_init(&ts_dev->timer2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_dev->timer2.function = gtt8205_dotimer2;
	ts_dev->irq = 7;
	ts_dev->pendown = 0;
	ts_dev->counttimes = 0;
	ts_dev->sleepstatus = 0;
	ts_dev->input->phys="gtt8205_TS/input0";
	input->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY);

	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input,ABS_X,0,0x7ff,0,0);

	input_set_abs_params(input,ABS_Y,0,0x7ff,0,0);

	input->name = "gtt8205s";	

	err = input_register_device(input);
	if(err)
		goto fail2;
#ifdef CONFIG_ANDROID_POWER

   	 ts_early_suspend.suspend = rk28_ts_suspend;

    	ts_early_suspend.resume = rk28_ts_resume;

    	android_register_early_suspend(&ts_early_suspend);
#endif
	g_ts_dev = ts_dev;
#if SINGLETOUCH_MODE
	/*set touchscreen single touch mode*/
	buf[0] = 0x10;
	sr=gtt8205_set_regs(ts_dev->client, 0x07, buf,1);
	if(sr<0)	
		printk("\n----I2C set gtt8205 singletouch mode error !!!\n");
#endif

#if MULTITOUCH_MODE
	/*set touchscreen single touch mode*/
	buf[0] = 0x11;
	sr=gtt8205_set_regs(ts_dev->client, 0x07, buf,1);
	if(sr<0)	
		printk("\n----I2C set gtt8205 multitouch mode error!!!\n");
#endif
//	gtt8205_read_values(ts_dev);
	err = request_gpio_irq(GPIOPortE_Pin3,gtt8205_ts_isr,GPIOLevelLow,ts_dev);
	if(err)
	{
		printk("unable to request touchscreen IRQ\n");
		goto fail3;
	}	
	hrtimer_start(&ts_dev->timer2,ktime_set(10,TS_POLL_DELAY),HRTIMER_MODE_REL);	/*monitor up event*/
	return 0;

fail3:

	free_irq(7,NULL);

fail2:

	input_unregister_device(input);

	input = NULL;

	//hrtimer_cancel(&ts_dev->timer1);
fail1:

	input_free_device(input);

nomem:

		kfree(ts_dev);


	return err;

}


static int __init rk28_ts_gtt8205_init(void)

{

	return i2c_add_driver(&gtt8205_driver);

}


module_init(rk28_ts_gtt8205_init);

//late_initcall(rk28_ts_gtt8205_init);


static void __exit rk28_ts_gtt8205_exit(void)

{

	i2c_del_driver(&gtt8205_driver);

}

module_exit(rk28_ts_gtt8205_exit);


MODULE_DESCRIPTION ("GTT touchscreen driver");

MODULE_AUTHOR("WQQ<wqq@rockchip.com.cn>");

MODULE_LICENSE("GPL");



