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

//#define RK28_PRINT
#include <asm/arch/rk28_debug.h>

#include "touchp.h"

#define SAMPLE_TIMES	    3
#define SAMPLE_DIFF_THRES   20
#define SAMPLE_DIFF_ABBC_THRES  15
#define RESISTER_X_PANEL    750
#define RESISTER_Y_PANEL    350
#define PRESSURE_THRES      10
#define SAMPLE_INTERVAL     10  //us

#define TOUCHPANEL_PRODUCTION   0
#define TOUCHPANEL_PRODUCTION1 0
#define TOUCHPANEL_ITO2050		  1
#define MODNAME	"xpt2046_ts_input"
#define PT2046_PENIRQ GPIOPortE_Pin3//This Pin is SDK Board GPIOPortE_Pin3 

#define PT2046_TOUCH_AD_LEFT	50
#define PT2046_TOUCH_AD_RIGHT	4090
#define PT2046_TOUCH_AD_TOP	    50
#define PT2046_TOUCH_AD_BOTTOM	4090
#if(defined(CONFIG_BOARD_IPAD8))
#define LCD_MAX_LENGTH          800
#define LCD_MAX_WIDTH           600

int screen_x[] = { 50,  750,   50,  750,  400};
int screen_y[] = { 40,   40,  550,  550,  300};
int uncali_x[] = {329, 3750,  331, 3757, 2046};
int uncali_y[] = {593,  532, 3675, 3655, 2121};
#else
#define LCD_MAX_LENGTH          800
#define LCD_MAX_WIDTH           480

int screen_x[] = { 50,  750,   50,  750,  400};
int screen_y[] = { 40,   40,  440,  440,  240};
int uncali_x[] = {329, 3750,  331, 3757, 2046};
int uncali_y[] = {593,  532, 3675, 3655, 2121};
#endif
#define XPT2046_IRQ 		7
#define XPT2046_NAME	"xpt2046 touchscreen"

#define TS_POLL_DELAY	(15*1000*1000)//(13*1000*1000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD	(35*1000*1000)	/* ns delay between samples */

#define MAX_12BIT	((1<<12)-1)

/*xpt2046 parameter*/
#define PT2046_START_BIT		(1<<7)
#define PT2046_A2A1A0_x		    (5<<4)
#define PT2046_A2A1A0_y         (1<<4)
#define PT2046_A2A1A0_z1 		(3<<4)
#define PT2046_A2A1A0_z2 		(4<<4)
#define PT2046_8_BIT			(1<<3)
#define PT2046_12_BIT			(0<<3)
#define PT2046_DFR				(0<<2)
#define PT2046_PD10_PDOWN		(0<<0)
#define PT2046_PD10_ADC_ON	    (1<<0)
#define PT2046_PD10_REF_ON		(2<<0)
#define PT2046_PD10_ALL_ON		(3<<0)

#define READ_X		(PT2046_START_BIT | PT2046_A2A1A0_y |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN) 
#define READ_Y		(PT2046_START_BIT | PT2046_A2A1A0_x |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Z1     (PT2046_START_BIT | PT2046_A2A1A0_z1 |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define READ_Z2     (PT2046_START_BIT | PT2046_A2A1A0_z2 |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)
#define PWRDOWN	(PT2046_START_BIT | PT2046_A2A1A0_y |PT2046_12_BIT | PT2046_DFR |PT2046_PD10_PDOWN)

#define MAX_SAMPLE_TIMES	6

typedef enum
{
    CenterPoint = 0,    // 中心校准点
    TopLeftPoint,       // 左上校准点
    TopRightPoint,      // 右上校准点
    BottomLeftPoint,    // 左下校准点
    BottomRightPoint    // 右下校准点
}eTOUCH_CALIBRATEPOINT;

typedef struct
{
    INT32    Ax;
    INT32    Ay;
    INT32    Bx;
    INT32    By;
    INT32    Cx;
    INT32    Cy;
    INT32    Dx;
    INT32    Dy;
} CALI_MATRIX ;

typedef struct
{
    INT16 x;
    INT16 y;
}POINT;

#if 0
static POINT gTouchCalibrateADPoint[] =
{
    { 115,  319},     // 左上校准点
    {3982,  247},     // 右上校准点
    {  88, 3946},     // 左下校准点
    {3990, 3946},      // 右下校准点
    {2027, 2103}     // 中心校准点
};

static POINT gTouchCalibrateLogicPoint[] =
{
    {50, 40},
    {750, 40},
    {50, 440},
    {750, 440},
    {400, 240}
};
#endif

struct XPT2046_TS_EVENT
{
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
	struct timer_list scan_timer;
};

struct XPT2046_TS_EVENT *pts_dev = NULL;

typedef struct {
	unsigned x,y;
	bool flag;
}ts_event_t;

/*----------------------------------------------------------------------------*/
//static POINT gTouchADPoint;
static POINT gADPoint;

// This code is touch check
static ssize_t touch_mode_show(struct device_driver *_drv,char *_buf)
{
    int count;
    
	count = sprintf(_buf,"TouchCheck:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	                uncali_x[0], uncali_y[0],
	                uncali_x[1], uncali_y[1],
	                uncali_x[2], uncali_y[2],
	                uncali_x[3], uncali_y[3],
	                uncali_x[4], uncali_y[4]);
#if 0
	count = sprintf(_buf,"TouchCheck:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	                gTouchCalibrateADPoint[0].x, gTouchCalibrateADPoint[0].y,
	                gTouchCalibrateADPoint[1].x, gTouchCalibrateADPoint[1].y,
	                gTouchCalibrateADPoint[2].x, gTouchCalibrateADPoint[2].y,
	                gTouchCalibrateADPoint[3].x, gTouchCalibrateADPoint[3].y,
	                gTouchCalibrateADPoint[4].x, gTouchCalibrateADPoint[4].y);
#endif
	printk("buf: %s", _buf);
	
	return count;
}

static ssize_t touch_mode_store(struct device_driver *_drv, const char *_buf, size_t _count)
{
    int i, j = 0;
    char temp[5];

    printk("Read data from Android: %s\n", _buf);
    
    for (i = 0; i < 5; i++)
    {
        strncpy(temp, _buf + 5 * (j++), 4);
        uncali_x[i] = simple_strtol(temp, NULL, 10);
        strncpy(temp, _buf + 5 * (j++), 4);
        uncali_y[i] = simple_strtol(temp, NULL, 10);
        printk("SN=%d uncali_x=%d uncali_y=%d\n", 
                i, uncali_x[i], uncali_y[i]);
    }
    
#if 0
  strncpy(temp,_buf,4);
  gTouchCalibrateADPoint[0].x = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 5,4);
  gTouchCalibrateADPoint[0].y = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 10,4);
  gTouchCalibrateADPoint[1].x = simple_strtol(temp,NULL,10); 
  strncpy(temp,_buf + 15,4);
  gTouchCalibrateADPoint[1].y = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 20,4);
  gTouchCalibrateADPoint[2].x = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 25,4);
  gTouchCalibrateADPoint[2].y = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 30,4);
  gTouchCalibrateADPoint[3].x = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 35,4);
  gTouchCalibrateADPoint[3].y = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 40,4);
  gTouchCalibrateADPoint[4].x = simple_strtol(temp,NULL,10);
  strncpy(temp,_buf + 45,4);
  gTouchCalibrateADPoint[4].y = simple_strtol(temp,NULL,10);
#endif

  return _count; 
}

//This code is Touch adc simple value
static ssize_t touch_adc_show(struct device_driver *_drv,char *_buf)
{
    printk("ADC show: x=%d y=%d\n", gADPoint.x, gADPoint.y);
    
	return sprintf(_buf, "%d,%d\n", gADPoint.x, gADPoint.y);
}

static ssize_t touch_cali_status(struct device_driver *_drv, char *_buf)
{
    int ret;
    
    ret = TouchPanelSetCalibration(4, screen_x, screen_y, uncali_x, uncali_y);
    if (ret == 1)
        ret = sprintf(_buf, "successful\n");
    else
        ret = sprintf(_buf, "fail\n");
    
    printk("Calibration status: _buf=<%s", _buf);
    
	return ret;
}

u16 xpt2046_spi_read(struct XPT2046_TS_EVENT *ts_dev, u8 operation)
{
	u8 tx_buf[1], rx_buf[2];
	u16 val = 0;
	int ret;
	
	tx_buf[0] = operation;
	ret = spi_write_then_read(ts_dev->spi, tx_buf, 1, NULL, 0);
	if (ret)
	{
		printk("spi_write_op failded!!\n");
		return 0;
	}
    mdelay(2);
    
    ret = spi_write_then_read(ts_dev->spi, NULL, 0, rx_buf, 2);
	if (ret)
	{
		printk("spi_read_op failded!!\n");
		return 0;
	}
	
    val = rx_buf[0] ;   /*correct sample date(clear high bit)*/
    val = (val <<8) + rx_buf[1];
    val = (val&(~(1<<15))) >> 3;

	return val;
}

u16 xpt2046_read_op(struct XPT2046_TS_EVENT *ts_dev, u8 operation)
{
	u8 tx_buf[1], rx_buf[2];
	u16 val = 0;
	int ret;
	
	tx_buf[0] = operation;
	ret = spi_write_then_read(ts_dev->spi, tx_buf, 1, rx_buf, 2);
	if (ret)
	{
		printk("spi_read_op failded!!\n");
		return 0;
	}

    val = rx_buf[0] ;   /*correct sample date(clear high bit)*/
    val = (val <<8) + rx_buf[1];
    val = (val&(~(1<<15))) >> 3;

	return val;
}

#define TP_STATE_IDLE         0
#define TP_STATE_PRE_DOWN     1
#define TP_STATE_DOWN         2
#define TP_STATE_PRE_IDLE     3

unsigned char tp_state = TP_STATE_IDLE;

#if 1
#define DO_ABS(x, y)  ((x) > (y) ? (x) - (y) : (y) - (x))

static u32 rk28_get_pressure(struct XPT2046_TS_EVENT *ts_dev)
{
    u16 posx, posy, pres_z1, pres_z2;
    u32 presa = 0, presb = 0;
    
    //udelay(SAMPLE_INTERVAL);
    posx = xpt2046_read_op(ts_dev, READ_X);
    //udelay(SAMPLE_INTERVAL);
    posy = xpt2046_read_op(ts_dev, READ_Y);
    //udelay(SAMPLE_INTERVAL);
    pres_z1 = xpt2046_read_op(ts_dev, READ_Z1);
    //udelay(SAMPLE_INTERVAL);
    pres_z2 = xpt2046_read_op(ts_dev, READ_Z2);
    
    rk28printk("Pressure: posx=%u posy=%u z1=%u z2=%u ", 
                posx, posy, pres_z1, pres_z2);

    if (pres_z1 > 0)
    {
        presa = (RESISTER_X_PANEL * posx) * (pres_z2 / pres_z1 - 1) / 4096;
        presb = ((RESISTER_X_PANEL * posx) * (4096 / pres_z1 - 1) -
                 RESISTER_Y_PANEL * (4096 - posy)) / 4096;
        //thres = (pres_z2 / pres_z1 - 1) * posx;
    }
    
    rk28printk("presa=%u presb=%d\n", presa, presb);
    
    return presa;
}
#endif

static int tp_average_filter(u16 *cor, int *ret)
{
    int diffab, diffbc, diffac;
    u32 group_a, group_b, group_c;
    
    if ((cor == NULL) || (ret == NULL))
        return -1;
        
#if (SAMPLE_TIMES == 3)
    group_a = cor[0];
    group_b = cor[1];
    group_c = cor[2];
#elif (SAMPLE_TIMES == 6)
    group_a = (cor[0] + cor[1]) / 2;
    group_b = (cor[2] + cor[3]) / 2;
    group_c = (cor[4] + cor[5]) / 2;
#else /* 9 */
    group_a = (cor[0] + cor[1] + cor[2]) / 3;
    group_b = (cor[3] + cor[4] + cor[5]) / 3;
    group_c = (cor[6] + cor[7] + cor[8]) / 3;
#endif
    
    diffab = abs(group_a - group_b);
    diffac = abs(group_a - group_c);
    diffbc = abs(group_b - group_c);

    rk28printk("DDDD: diffab = %d diffac=%d diffbc=%d\n",
                diffab, diffac, diffbc);
                
    if ((diffab > SAMPLE_DIFF_THRES) ||
        (diffbc > SAMPLE_DIFF_THRES))
        return -2;

    if ((diffab <= diffac) && (diffab <= diffbc))
    {
        *ret = (group_a + group_b) / 2;
    }
    else if ((diffac <= diffab) && (diffac <= diffbc))
    {
        *ret = (group_a + group_c) / 2;
    }
    else
    {
        *ret = (group_b + group_c) / 2;
    }
                
    return 0;
}

static void rk28_tpscan_timer(unsigned long data)
{
    int i = 0, retx, rety;
    u16 cor_x[SAMPLE_TIMES], cor_y[SAMPLE_TIMES];
    int total_x = 0, total_y = 0;
    int xd, yd;
    //u32 pressure;
    //static u32 prev_pressure = 0;
    struct XPT2046_TS_EVENT *ts_dev = (struct XPT2046_TS_EVENT *)data;

    cor_x[i] = xpt2046_read_op(ts_dev, READ_X);
    cor_y[i] = xpt2046_read_op(ts_dev, READ_Y);
    
    for (i = 0; i < SAMPLE_TIMES; i++)
    {
        //udelay(SAMPLE_INTERVAL);
        cor_x[i] = xpt2046_read_op(ts_dev, READ_X);
        //udelay(SAMPLE_INTERVAL);
        cor_y[i] = xpt2046_read_op(ts_dev, READ_Y);
    }

#ifdef RK28_PRINT
    for (i = 0; i < SAMPLE_TIMES; i++)
    {
        printk("RAW samples: id=%2d x=%5d y=%5d\n", i, cor_x[i], cor_y[i]);
    }
    printk("\n");
#endif

    //pressure = rk28_get_pressure(ts_dev);

    for (i = 0; i < SAMPLE_TIMES; i++)
    {
        if (cor_x[i] == 4095 || cor_x[i] == 0)
        {
            if (tp_state == TP_STATE_DOWN)
                tp_state = TP_STATE_PRE_IDLE;
            goto out;
        }
        if (cor_y[i] == 4095 || cor_y[i] == 0)
        {
            if (tp_state == TP_STATE_DOWN)
                tp_state = TP_STATE_PRE_IDLE;
            goto out;
        }
    }

    retx = tp_average_filter(cor_x, &total_x);
    rety = tp_average_filter(cor_y, &total_y);
    if (((retx < 0) && (rety < 0)) ||
        (retx == -2) || (rety == -2))
    {
        if (tp_state == TP_STATE_PRE_DOWN)
            tp_state = TP_STATE_IDLE;
        goto out;
    }

    TouchPanelCalibrateAPoint(total_x, total_y, &xd, &yd);
    
    xd = xd / 4;
    yd = yd / 4;
    
    rk28printk("total_x = %d total_y=%d >> x=%d y=%d\n",
        total_x, total_y, xd, yd);
    
    if (tp_state == TP_STATE_IDLE)
    {
        tp_state = TP_STATE_PRE_DOWN;
        rk28printk("TP state changed from IDLE to PRE_DOWN.\n");
    }
    else if (tp_state == TP_STATE_PRE_DOWN)
    {
        rk28printk("TP state changed from PRE_DOWN to DOWN.\n");
        tp_state = TP_STATE_DOWN;
        input_report_key(ts_dev->input, BTN_TOUCH, 1);
        //input_sync(ts_dev->input);
    }
    
    if (tp_state == TP_STATE_DOWN)
    {
        gADPoint.x = total_x;
        gADPoint.y = total_y;
        
        rk28printk("state=%d send coordinate: x=%d y=%d\n", tp_state, xd, yd);
        //input_report_key(ts_dev->input, BTN_TOUCH, 1);
        input_report_abs(ts_dev->input, ABS_X, xd);
        input_report_abs(ts_dev->input, ABS_Y, yd);
        input_sync(ts_dev->input);

#if 0
        total_x = cor_x[2];
        total_y = cor_y[2];
        TouchPanelCalibrateAPoint(total_x, total_y, &xd, &yd);
    
        xd = xd / 4;
        yd = yd / 4;
    
        rk28printk("state=%d send coordinate: x=%d y=%d\n", tp_state, xd, yd);
        //input_report_key(ts_dev->input, BTN_TOUCH, 1);
        input_report_abs(ts_dev->input, ABS_X, xd);
        input_report_abs(ts_dev->input, ABS_Y, yd);
        input_sync(ts_dev->input);
#endif
    }

out:

    if ((GPIOGetPinLevel(PT2046_PENIRQ) == 1) &&
        ((tp_state == TP_STATE_PRE_IDLE) ||
         (tp_state == TP_STATE_DOWN)))
    {
        rk28printk("TP state changed from PRE_IDLE to IDLE.\n");
        tp_state = TP_STATE_IDLE;
        input_report_key(ts_dev->input, BTN_TOUCH, 0);
        input_sync(ts_dev->input);
    }
    else if (GPIOGetPinLevel(PT2046_PENIRQ) == 0)
    {
        mod_timer(&ts_dev->scan_timer, jiffies + msecs_to_jiffies(15));
        return;
    }
    
    gpio_irq_enable(PT2046_PENIRQ);
}

static irqreturn_t xpt2046_ts_interrupt(int irq,void *handle)
{
    //unsigned long flags;
	struct XPT2046_TS_EVENT *ts_dev = (struct XPT2046_TS_EVENT *)handle;
	
	//spin_lock_irqsave(&ts_dev->lock,flags);
	gpio_irq_disable(PT2046_PENIRQ);
	
	//hrtimer_start(&ts_dev->timer,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	
	//spin_unlock_irqrestore(&ts_dev->lock, flags);
	
    mod_timer(&ts_dev->scan_timer, jiffies + msecs_to_jiffies(15));
    
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
	//hrtimer_init(&ts_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	//ts_dev->timer.function = xpt2046_dostimer;

    setup_timer(&ts_dev->scan_timer, rk28_tpscan_timer, (unsigned long)ts_dev);
    
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
	//input_set_abs_params(xpt2046_ts_dev,ABS_X,400,3680,0,0);
	//input_set_abs_params(xpt2046_ts_dev,ABS_Y,550,3350,0,0);
	
    input_set_abs_params(xpt2046_ts_dev,ABS_X,0,LCD_MAX_LENGTH - 1,0,0);
    input_set_abs_params(xpt2046_ts_dev,ABS_Y,0,LCD_MAX_WIDTH - 1,0,0);
	xpt2046_ts_dev->name = XPT2046_NAME;
	
	xpt2046_ts_dev->phys = ts_dev->phys;

	
	dev_set_drvdata(&spi->dev, ts_dev);
	xpt2046_ts_dev->dev.parent = &spi->dev;    
    
	//xpt2046_read_values(ts_dev,MAX_SAMPLE_TIMES);

	//rk28printk("************>%s....x=%d...y=%d...z1=%d...z2=%d\n",__FUNCTION__,ts_dev->x,ts_dev->y,ts_dev->z1,ts_dev->z2);
	
	///__raw_writel(__raw_readl(GPIO1_BASE_ADDR_VA + 0x30)|0x80,(GPIO1_BASE_ADDR_VA + 0x30));	
	//err = request_irq(ts_dev->irq,xpt2046_ts_interrupt,IRQF_TRIGGER_FALLING,spi->dev.driver->name,ts_dev);
    GPIOPullUpDown(PT2046_PENIRQ, GPIOPullUp);
	//err = request_gpio_irq(PT2046_PENIRQ,xpt2046_ts_interrupt,GPIOEdgelFalling,ts_dev);	
	err = request_gpio_irq(PT2046_PENIRQ, (pFunc)xpt2046_ts_interrupt, GPIOLevelLow, ts_dev);		
	if(err<0)
	{
		rk28printk("xpt2046 request irq failed !!\n");
		err = -EBUSY;
		goto fail1;
	}
	err = input_register_device(xpt2046_ts_dev);
	if(err)
		goto fail2;
    
    //Enable XPT2046 interrupt.
    xpt2046_read_op(ts_dev, READ_X);
    
    pts_dev = ts_dev;
    
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
	struct XPT2046_TS_EVENT *ts_dev = dev_get_drvdata(&pdev->dev);
	
	rk28printk("XPT2046 driver removed!\n");
	
	free_irq(ts_dev->irq,ts_dev);
	hrtimer_cancel(&ts_dev->timer);
	input_free_device(ts_dev->input);
	kfree(ts_dev);
	
	return 0;
}

#ifdef CONFIG_PM
int xpt2046_suspend(struct spi_device *spi, pm_message_t state)
{
    printk("XPT2046 driver suspend!!\n");
    
    gpio_irq_disable(PT2046_PENIRQ);
    
    return 0;
}

int xpt2046_resume(struct spi_device *spi)
{
    printk("XPT2046 driver resume!!\n");
    
    gpio_irq_enable(PT2046_PENIRQ);
    
    return 0;
}
#endif 

static struct spi_driver xpt2046_ts_driver = 
{
	.driver = {
		.name = "xpt2046_ts",
		.bus	  = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = xpt2046_ts_proble,
	.remove = __devexit_p(xpt2046_ts_remove),
#ifdef CONFIG_PM
	.suspend = xpt2046_suspend,
	.resume = xpt2046_resume
#endif
};

static DRIVER_ATTR(touchcheck, 0666, touch_mode_show, touch_mode_store);
static DRIVER_ATTR(touchadc, 0666, touch_adc_show, NULL);
static DRIVER_ATTR(calistatus, 0666, touch_cali_status, NULL);

static int __init xpt2046_ts_init(void)
{
	int ret = spi_register_driver(&xpt2046_ts_driver);

    rk28printk("Touch panel drive XPT2046 driver init...\n");
    
    if (ret == 0)
    {
	    gADPoint.x = 0;
	    gADPoint.y = 0;
	    ret = driver_create_file(&xpt2046_ts_driver.driver, &driver_attr_touchcheck);
	    ret += driver_create_file(&xpt2046_ts_driver.driver, &driver_attr_touchadc);
	    ret += driver_create_file(&xpt2046_ts_driver.driver, &driver_attr_calistatus);
	}
	
	return ret;
}

static void __exit xpt2046_ts_exit(void)
{
    rk28printk("Touch panel drive XPT2046 driver exit...\n");
    
    driver_remove_file(&xpt2046_ts_driver.driver, &driver_attr_touchcheck);
    driver_remove_file(&xpt2046_ts_driver.driver, &driver_attr_touchadc);
    driver_remove_file(&xpt2046_ts_driver.driver, &driver_attr_calistatus);
    
	return spi_unregister_driver(&xpt2046_ts_driver);
}

#if 0
/**************************************************************************
* 函数描述:  得到Touchpanel的校准系数矩阵
* 入口参数:  pLogicPoint -- 校准的逻辑坐标点
*            pADPoint -- 校准的AD采样点
* 出口参数:  matrixPtr -- 校准的系数矩阵
* 返回值:    无
***************************************************************************/
int Touch_GetCalibrationMatrix(POINT * pLogicPoint, POINT * pADPoint, CALI_MATRIX * matrixPtr)
{
    // 将百分比放大
    matrixPtr->Ax = (abs(pLogicPoint[TopLeftPoint].x - pLogicPoint[CenterPoint].x) << 8) / abs(pADPoint[TopLeftPoint].x - pADPoint[CenterPoint].x);
    matrixPtr->Ay = (abs(pLogicPoint[TopLeftPoint].y - pLogicPoint[CenterPoint].y) << 8) / abs(pADPoint[TopLeftPoint].y - pADPoint[CenterPoint].y);
    matrixPtr->Bx = (abs(pLogicPoint[TopRightPoint].x - pLogicPoint[CenterPoint].x) << 8) / abs(pADPoint[TopRightPoint].x - pADPoint[CenterPoint].x);
    matrixPtr->By = (abs(pLogicPoint[TopRightPoint].y - pLogicPoint[CenterPoint].y) << 8) / abs(pADPoint[TopRightPoint].y - pADPoint[CenterPoint].y);
    matrixPtr->Cx = (abs(pLogicPoint[BottomLeftPoint].x - pLogicPoint[CenterPoint].x) << 8) / abs(pADPoint[BottomLeftPoint].x - pADPoint[CenterPoint].x);
    matrixPtr->Cy = (abs(pLogicPoint[BottomLeftPoint].y - pLogicPoint[CenterPoint].y) << 8) / abs(pADPoint[BottomLeftPoint].y - pADPoint[CenterPoint].y);
    matrixPtr->Dx = (abs(pLogicPoint[BottomRightPoint].x - pLogicPoint[CenterPoint].x) << 8) / abs(pADPoint[BottomRightPoint].x - pADPoint[CenterPoint].x);
    matrixPtr->Dy = (abs(pLogicPoint[BottomRightPoint].y - pLogicPoint[CenterPoint].y) <<8) / abs(pADPoint[BottomRightPoint].y - pADPoint[CenterPoint].y);

    return 0 ;

}

/**************************************************************************
* 函数描述:  得到Touchpanel坐标点
* 入口参数:  adPoint
* 出口参数:  pPixelPoint
* 返回值:    无
***************************************************************************/
void Touch_GetPixelPoint(POINT adPoint, POINT* pPixelPoint)
{
    CALI_MATRIX  matrixPtr;
    INT16 tempx, tempy;

    tempx = adPoint.x;
    tempy = adPoint.y;

    Touch_GetCalibrationMatrix(gTouchCalibrateLogicPoint, gTouchCalibrateADPoint, &matrixPtr);

    if (tempx >= gTouchCalibrateADPoint[CenterPoint].x)
    {
        if (tempy >= gTouchCalibrateADPoint[CenterPoint].y)   // 右下区域(D区)
        {
            pPixelPoint->x = (((UINT32)(tempx - gTouchCalibrateADPoint[CenterPoint].x) * matrixPtr.Dx) >> 8) + (LCD_MAX_LENGTH >> 1);
            pPixelPoint->y = (((UINT32)(tempy - gTouchCalibrateADPoint[CenterPoint].y) * matrixPtr.Dy) >> 8) + (LCD_MAX_WIDTH >> 1);
        }
        else  // 右上区域(B区)
        {
            pPixelPoint->x = (((UINT32)(tempx - gTouchCalibrateADPoint[CenterPoint].x) * matrixPtr.Bx) >> 8) + (LCD_MAX_LENGTH >> 1);
            pPixelPoint->y = (((UINT32)(tempy - gTouchCalibrateADPoint[TopRightPoint].y) * matrixPtr.By) >> 8);
        }
    }
    else
    {
        if (tempy <= gTouchCalibrateADPoint[CenterPoint].y)  // 左上区域(A区)
        {
            pPixelPoint->x = (((UINT32)(tempx - gTouchCalibrateADPoint[TopLeftPoint].x) * matrixPtr.Ax) >> 8);
            pPixelPoint->y = (((UINT32)(tempy - gTouchCalibrateADPoint[TopLeftPoint].y) * matrixPtr.Ay) >> 8);
        }
        else       // 左下区域(C区)
        {
            pPixelPoint->x = (((UINT32)(tempx - gTouchCalibrateADPoint[TopLeftPoint].x) * matrixPtr.Cx) >> 8);
            pPixelPoint->y = (((UINT32)(tempy -  gTouchCalibrateADPoint[CenterPoint].y) * matrixPtr.Cy) >> 8) + (LCD_MAX_WIDTH >> 1);
        }
    }
    if (pPixelPoint->x < 0) pPixelPoint->x = 0;
    if (pPixelPoint->y < 0) pPixelPoint->y = 0;
    if (pPixelPoint->x > LCD_MAX_LENGTH) pPixelPoint->x = LCD_MAX_LENGTH;
    if (pPixelPoint->y > LCD_MAX_WIDTH) pPixelPoint->y = LCD_MAX_WIDTH;

    //Touch_TransCoordinate(pPixelPoint);
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
	 if(numValidData < 3)	 /*no touch event*/
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
		// udelay(6);
		 udelay(9);
		 tmpDataArr[i].x= xpt2046_read_op(ts_dev,READ_X);
		 //udelay(6);
		 udelay(9);
		 tmpDataArr[i].y= xpt2046_read_op(ts_dev,READ_Y);

#if 0		
		 if((tmpDataArr[i].x < PT2046_TOUCH_AD_LEFT )||(tmpDataArr[i].x > PT2046_TOUCH_AD_RIGHT ))
		 {
		 		tmpDataArr[i].flag = 1;
		 		rk28printk("x ^^^^^^^^error tmpData.x = %d  tmpData.y = %d\n",tmpDataArr[i].x,tmpDataArr[i].y);
				continue;
		 }
		 if((tmpDataArr[i].y < PT2046_TOUCH_AD_TOP)||(tmpDataArr[i].y > PT2046_TOUCH_AD_BOTTOM ))
		 {
		 		tmpDataArr[i].flag = 1;
		 		 rk28printk("y ^^^^^^^^ error tmpData.x = %d  tmpData.y = %d\n",tmpDataArr[i].x,tmpDataArr[i].y);
				continue;
		 }
#endif
		 tmpDataArr[i].flag = 0;
	 }
    for (i = 0; i < sample_times; i++)
    {
        rk28printk("Samples=%2d: x=%d y=%d\n", i, tmpDataArr[i].x, tmpDataArr[i].y);
    }
	 //tmpData = xpt2046_clear_dithering(tmpDataArr,sample_times);
	 //ts_dev->x = tmpData.x;
	 //ts_dev->y = tmpData.y;
	 //printk("after clear dithering tmpData.x = %d  tmpData.y = %d\n",tmpData.x,tmpData.y);
 fail1:
    kfree(tmpDataArr);

	 return err;
}

static void  xpt2046_send_values(struct XPT2046_TS_EVENT *ts_dev)
{
	struct XPT2046_TS_EVENT *ts = ts_dev;
	struct input_dev *xpt2046_ts_dev;
	POINT  pixelpoint;
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
		//ts->touch_x = AD_TO_X(x);
		//ts->touch_y = AD_TO_Y(y);
		 gTouchADPoint.x = x;
	         gTouchADPoint.y = y;
		 gADPoint.x = x;
		 gADPoint.y = y;	
//		printk("************(adc_touch_point)...x=%d...y=%d\n",gTouchADPoint.x,gTouchADPoint.y);
		
	      Touch_GetPixelPoint(gTouchADPoint, &pixelpoint);
		ts->touch_x = pixelpoint.x;
		ts->touch_y = pixelpoint.y;  
//		printk("************>%s.(touch_point)...x=%d...y=%d\n",__FUNCTION__,ts->touch_x,ts->touch_y);
		
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
	PE7status =  GPIOGetPinLevel(PT2046_PENIRQ);
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
		gpio_irq_enable(PT2046_PENIRQ);
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

#endif

module_init(xpt2046_ts_init);
module_exit(xpt2046_ts_exit);
MODULE_AUTHOR("WQQ,wqq@rockchip.com");
MODULE_DESCRIPTION("rockchip rk28chip extern touchscreen");
MODULE_LICENSE("GPL");
