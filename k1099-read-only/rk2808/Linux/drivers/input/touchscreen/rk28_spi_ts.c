/****************************************************************************************
 * driver/input/touchscreen/rk28_tocuscreen.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: WQQ 
 *Date		: 2009-04-25
 *This driver use for rk28 chip extern touchscreen. Use gpio to simulate  clock for touchscreen.
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
/***********************************************
 *	DEBUG
 * ********************************************/
//#include <asm/arch/rk28_macro.h>
//#define RK28_PRINT	
#include <asm/arch/rk28_debug.h>


#define MODNAME	"xpt2046_ts_input"

#define PT2046_TOUCH_AD_LEFT		3855
#define PT2046_TOUCH_AD_RIGHT		260
#define PT2046_TOUCH_AD_TOP		300
#define PT2046_TOUCH_AD_BOTTOM	3755
#define LCD_MAX_LENGTH				800
#define LCD_MAX_WIDTH				480

#define XPT2046_IRQ 		7
#define XPT2046_NAME	"xpt2046 touchscreen"

#define TS_POLL_DELAY	(15*1000*1000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD	(35*1000*1000)	/* ns delay between samples */

#define MAX_12BIT	((1<<12)-1)

/*xpt2046 parameter*/
#define PT2046_START_BIT		(1<<7)
#define PT2046_A2A1A0_x		(5<<4)
#define PT2046_A2A1A0_y		(1<<4)
#define PT2046_A2A1A0_z1 		(3<<4)
#define PT2046_A2A1A0_z2 		(4<<4)
#define PT2046_8_BIT			(1<<3)
#define PT2046_12_BIT			(0<<3)
#define PT2046_DFR				(0<<2)
#define PT2046_PD10_PDOWN		(0<<0)
#define PT2046_PD10_ADC_ON	(1<<0)
#define PT2046_PD10_REF_ON		(2<<0)
#define PT2046_PD10_ALL_ON		(3<<0)

#define READ_X		(PT2046_START_BIT | PT2046_A2A1A0_x |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Y		(PT2046_START_BIT | PT2046_A2A1A0_y |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Z1         (PT2046_START_BIT | PT2046_A2A1A0_z1 |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Z2 		(PT2046_START_BIT | PT2046_A2A1A0_z2 |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define PWRDOWN	(PT2046_START_BIT | PT2046_A2A1A0_y |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)

#define AD_TO_Y(ady)	(LCD_MAX_LENGTH * (PT2046_TOUCH_AD_LEFT - ady) / (PT2046_TOUCH_AD_LEFT - PT2046_TOUCH_AD_RIGHT))
#define AD_TO_X(adx)	(LCD_MAX_WIDTH * (adx - PT2046_TOUCH_AD_TOP) / ( PT2046_TOUCH_AD_BOTTOM  - PT2046_TOUCH_AD_TOP ))

#define MAX_SAMPLE_TIMES	20

struct XPT2046_TS_EVENT{
	struct input_dev *input;
	struct spi_device *spi;
	struct work_struct  x_work;
	struct work_struct  y_work;
	char 	phys[32];
	int 		irq;
	spinlock_t	lock;
	uint16	x;
	uint16	y;
	uint16	z1;
	uint16	z2;
	uint16     touch_x;
	uint16 	touch_y;
	bool		pendown;
	bool 	 status;
	struct hrtimer  timer;
};
typedef struct {
	unsigned x,y;
	bool flag;
}ts_event_t;

/*----------------------------------------------------------------------------*/
uint16 PT2046_read_op(struct XPT2046_TS_EVENT *ts_dev,u8 operation)
{	
	u8 tx_buf[1];
	u8 rx_buf[2];
	u16 val = 0;
	int ret;
	tx_buf[0] = operation;
	ret = spi_write_then_read(ts_dev->spi,tx_buf,1,rx_buf,2);
	if(ret)
		printk("spi_read_op failded!!\n");
	else
		val = rx_buf[0] ;		/*correct sample date(clear high bit)*/
		val = (val <<8) + rx_buf[1];
		val = (val&(~(1<<15))) >> 3;	
		//rk28printk("pt2046_read_op->rx_buf[0]= %x\n",rx_buf[0]);
		//rk28printk("pt2046_read_op->rx_buf[1]= %x\n",rx_buf[1]);
		//rk28printk("pt2046_read_op->val= %x\n",val);
	return val;
} 

 static int clear_invaild_sample_data(ts_event_t *tmpDataArr,int sample_times)
 {
 
	 int i;
	 if(tmpDataArr == NULL)
	 {
		 printk("%s---error!\n",__FUNCTION__);
		 return 1;
	 }
	 for(i=0;i<sample_times;i++)
	 {	 
		 if((tmpDataArr[i].x == 0)||(tmpDataArr[i].y == 4095))	 /*clear invaild sample data*/
		 {
				 tmpDataArr[i].flag = 1;
				 continue;
		 }
	 }
	 for(i=0;i<sample_times;i++)
			 rk28printk("*****tmpData.x=%d**tmpData.y=%d** tmpData.flag =%d\n",tmpDataArr[i].x,tmpDataArr[i].y,tmpDataArr[i].flag);

	 return 0;
 }
 static ts_event_t xpt2046_clear_dithering(ts_event_t *tmpDataArr,int sample_times)
 {
 
	 int i;
	 int numValidData;				 /*record the number of  valid AD sample value*/
	 ts_event_t tmpData;
	 unsigned int totalxData,totalyData;
	 numValidData = totalxData = totalyData = 0;
	 if(tmpDataArr == NULL)
	 {
		 printk("%s---error!\n",__FUNCTION__);
		 return tmpData;
	 }
	 clear_invaild_sample_data( tmpDataArr,sample_times);
	 for(i=0; i<sample_times; i++)
	 {
		 if(tmpDataArr[i].flag ==0)
		 {
			 numValidData++;
			 totalxData += tmpDataArr[i].x;
			 totalyData += tmpDataArr[i].y;
		 }
	 }
	 if(numValidData < 9)	 /*no touch event*/
	 {	 
		 tmpData.x = 0;
		 tmpData.y = 4095;
		 return tmpData;
	 }
	 tmpData.x = totalxData/numValidData;
	 tmpData.y = totalyData/numValidData;
	 return tmpData;
 }
 
  static int xpt2046_read_values(struct XPT2046_TS_EVENT *ts_dev,int sample_times)
  {
	 ts_event_t *tmpDataArr;
	 ts_event_t   tmpData;
	 int i,err;
	 tmpDataArr = kzalloc(sizeof(ts_event_t) * MAX_SAMPLE_TIMES , GFP_KERNEL);
	 if(!tmpDataArr)
	 {
		 err = -ENOMEM;
		 goto fail1;
	 }
	 for(i=0;i<sample_times;i++) /*sample 10 times*/
	 {
		 udelay(30);
		 tmpDataArr[i].x= PT2046_read_op(ts_dev,READ_X);
		 udelay(30);
		 tmpDataArr[i].y= PT2046_read_op(ts_dev,READ_Y);
		 if((tmpDataArr[i].x < PT2046_TOUCH_AD_TOP)||(tmpDataArr[i].x > PT2046_TOUCH_AD_BOTTOM))
		 {
		 		tmpDataArr[i].flag = 1;
				continue;
		 }
		 if((tmpDataArr[i].y < PT2046_TOUCH_AD_RIGHT)||(tmpDataArr[i].y > PT2046_TOUCH_AD_LEFT))
		 {
		 		tmpDataArr[i].flag = 1;
				continue;
		 }
		 tmpDataArr[i].flag = 0;
 //  ts_dev->z1 = PT2046_read_op(ts_dev,READ_Z1);
 //  ts_dev->z2 = PT2046_read_op(ts_dev,READ_Z2);
	 /*power down*/
 //  PT2046_read_op(ts_dev,PWRDOWN);
 
	 }
	 tmpData = xpt2046_clear_dithering(tmpDataArr,sample_times);
	 ts_dev->x = tmpData.x;
	 ts_dev->y = tmpData.y;
	 rk28printk("after clear dithering tmpData.x = %d  tmpData.y = %d\n",tmpData.x,tmpData.y);
 fail1:
		 kfree(tmpDataArr);
	 return err;
  }

static void  xpt2046_send_values(struct XPT2046_TS_EVENT *ts_dev)
{
	struct XPT2046_TS_EVENT *ts = ts_dev;
	struct input_dev *xpt2046_ts_dev;
	u16 x,y,z1;
	x = ts->x;
	y = ts->y;
	z1 = ts->z1;

	rk28printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);

	if((x == 0)&&(y == 4095))	/*ignored pressure*/
	{
		ts_dev->pendown = 1;
		rk28printk("ignored pressure\n");
		hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_PERIOD),HRTIMER_MODE_REL);
		return ;
	}
	else		/*valid event*/
	{
		xpt2046_ts_dev = ts->input;
		if(!ts->pendown)
		{
			rk28printk("The touchscreen down!!\n");
			input_report_key(xpt2046_ts_dev,BTN_TOUCH,1);
			ts->pendown = 1;
		}
		rk28printk("************>%s.(adsample_value)...x=%d...y=%d\n",__FUNCTION__,x,y);
/***************************************************************************************
*	change AD sample value to Touchscreen point value
********************************************************************************************/
		ts->touch_x =   AD_TO_Y(y);
		ts->touch_y =   AD_TO_X(x);
		
		rk28printk("************>%s.(touch_point)...x=%d...y=%d\n",__FUNCTION__,ts->touch_x,ts->touch_y);
		
		input_report_abs(xpt2046_ts_dev,ABS_X,ts->touch_x);
		input_report_abs(xpt2046_ts_dev,ABS_Y,ts->touch_y);
		//input_report_key(xpt2046_ts_dev,BTN_TOUCH,1);
		input_sync(xpt2046_ts_dev);
	}
	hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_PERIOD),HRTIMER_MODE_REL);
}

 static enum hrtimer_restart xpt2046_dostimer(struct hrtimer *handle)
{

	struct XPT2046_TS_EVENT *ts_dev = container_of(handle, struct XPT2046_TS_EVENT, timer);
	struct input_dev *xpt2046_ts_dev;
	int PE7status;
	rk28printk("************>%s.....%s.....\n",__FILE__,__FUNCTION__);
	spin_lock_irq(&ts_dev->lock);
	PE7status =  GPIOGetPinLevel(GPIOPortE_Pin3);
	rk28printk("************>%s....PE7status=%d\n",__FUNCTION__,PE7status);
	if(unlikely(ts_dev->pendown && PE7status))
	{
		xpt2046_ts_dev = ts_dev->input;
		rk28printk("The touchscreen up!!\n");
		input_report_key(xpt2046_ts_dev,BTN_TOUCH,0);
		input_sync(xpt2046_ts_dev);
		ts_dev->pendown = 0;
		///__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x4c) | 0x80,(GPIO1_BASE_ADDR_VA + 0x4c)); /*clear penirq*/
		//enable_irq(ts_dev->irq);
		gpio_irq_enable(GPIOPortE_Pin3);
	}
	else{
		/*still down ,continue with the measurement*/
		rk28printk("pen still down!!\n");
		xpt2046_read_values(ts_dev,MAX_SAMPLE_TIMES);
		xpt2046_send_values(ts_dev);
	}
	spin_unlock_irq(&ts_dev->lock);
	return HRTIMER_NORESTART;



}


static irqreturn_t xpt2046_ts_interrupt(int irq,void *handle)
{
	struct XPT2046_TS_EVENT *ts_dev = handle;
	unsigned long flags;
	rk28printk("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);
	spin_lock_irqsave(&ts_dev->lock,flags);
	gpio_irq_disable(GPIOPortE_Pin3);
	///disable_irq(ts_dev->irq);	
	hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	///__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x4c) | 0x80,(GPIO1_BASE_ADDR_VA + 0x4c)); /*clear penirq*/
	
	spin_unlock_irqrestore(&ts_dev->lock, flags);

	return IRQ_HANDLED;

}


static int  __devinit xpt2046_ts_proble(struct spi_device *spi)
{

	struct XPT2046_TS_EVENT  *ts_dev;
	struct input_dev *xpt2046_ts_dev;
	unsigned int err = 0;

	rk28printk("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);


	ts_dev=kzalloc(sizeof(struct XPT2046_TS_EVENT), GFP_KERNEL);
	if(!ts_dev)
	{
		rk28printk("failed to allocate memory!!\n");
		goto nomem;
	}

	ts_dev->spi = spi;	/*ts_dev to spi reference*/

	
	xpt2046_ts_dev = input_allocate_device();
	if(!xpt2046_ts_dev)
	{
		rk28printk("rk28 xpt2046_ts allocate input device failed!!!\n");	
		goto fail1;
	}
	ts_dev->input = xpt2046_ts_dev;
/*init  timer to dispose workqueue */
	hrtimer_init(&ts_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_dev->timer.function = xpt2046_dostimer;

	ts_dev->x = 0;
	ts_dev->y = 0;
	ts_dev->touch_x = 0;
	ts_dev->touch_y = 0;
	ts_dev->status = 0;
	ts_dev->pendown = 0;
	ts_dev->irq =XPT2046_IRQ;
	snprintf(ts_dev->phys,sizeof(ts_dev->phys),"%s/input0",spi->dev.bus_id);
	
	xpt2046_ts_dev->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY);
	xpt2046_ts_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(xpt2046_ts_dev,ABS_X,0,800,0,0);
	input_set_abs_params(xpt2046_ts_dev,ABS_Y,0,480,0,0);
	xpt2046_ts_dev->name = XPT2046_NAME;
	
	xpt2046_ts_dev->phys = ts_dev->phys;

	
	dev_set_drvdata(&spi->dev, ts_dev);
	xpt2046_ts_dev->dev.parent = &spi->dev;

	xpt2046_read_values(ts_dev,MAX_SAMPLE_TIMES);

	rk28printk("************>%s....x=%d...y=%d...z1=%d...z2=%d\n",__FUNCTION__,ts_dev->x,ts_dev->y,ts_dev->z1,ts_dev->z2);
	
	///__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x30)|0x80,(GPIO1_BASE_ADDR_VA + 0x30));	
	//err = request_irq(ts_dev->irq,xpt2046_ts_interrupt,IRQF_TRIGGER_FALLING,spi->dev.driver->name,ts_dev);
GPIOPullUpDown(GPIOPortE_Pin3,GPIOPullUp);
	err = request_gpio_irq(GPIOPortE_Pin3,xpt2046_ts_interrupt,GPIOEdgelFalling,ts_dev);		
	if(err<0)
	{
		rk28printk("xpt2046 request irq failed !!\n");
		err = -EBUSY;
		goto fail1;
	}
	err = input_register_device(xpt2046_ts_dev);
	if(err)
		goto fail2;
	return err;

fail2:	
	free_irq(XPT2046_IRQ,NULL);

fail1:
	input_free_device(xpt2046_ts_dev);
	hrtimer_cancel(&ts_dev->timer);
nomem:
		kfree(ts_dev);

	return err;
}
static  int __devexit xpt2046_ts_remove(struct spi_device *pdev)
{
	struct XPT2046_TS_EVENT *ts_dev =dev_get_drvdata(&pdev->dev);
	rk28printk("*****************************xpt2046_ts_remove******************\n");
	free_irq(ts_dev->irq,ts_dev);
	hrtimer_cancel(&ts_dev->timer);
	input_free_device(ts_dev->input);
	kfree(ts_dev);
	return 0;
}


static struct spi_driver xpt2046_ts_driver = {
	.driver = {
		.name = "xpt2046_ts",
		.bus	  = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = xpt2046_ts_proble,
	.remove = __devexit_p(xpt2046_ts_remove),

};

static int __init xpt2046_ts_init(void)
{
//	rk28printk("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);
	return spi_register_driver(&xpt2046_ts_driver);
	
}

static void __exit xpt2046_ts_exit(void)
{
	rk28printk("************>%s.....%s.....%d\n",__FILE__,__FUNCTION__,__LINE__);
	return spi_unregister_driver(&xpt2046_ts_driver);
}
module_init(xpt2046_ts_init);
module_exit(xpt2046_ts_exit);
MODULE_AUTHOR("WQQ,wqq@rockchip.com");
MODULE_DESCRIPTION("rockchip rk28chip extern touchscreen");
MODULE_LICENSE("GPL");

