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
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
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
#include <asm/arch/api_i2c.h>
#include "touchp.h"

#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

static int ak4183_debug = 0;

#define D(fmt, arg...) \
	do { \
		if (ak4183_debug >= 2) printk(KERN_DEBUG "D/%s[%d]-%s: " fmt, __FILE__, __LINE__, __FUNCTION__, ##arg); \
	} while(0)


#define I(fmt, arg...) \
	do { \
		if (ak4183_debug >= 1) printk(KERN_DEBUG "I/%s[%d]-%s: " fmt, __FILE__, __LINE__, __FUNCTION__, ##arg); \
	} while(0)
		
#define E(fmt, arg...) \
	do { \
		if (ak4183_debug >= 0) printk(KERN_ERR "E/%s[%d]-%s: " fmt, __FILE__, __LINE__, __FUNCTION__, ##arg); \
	} while(0)

#define AK4183_I2C_ADDR 0x90

#define LCD_MAX_LENGTH 1024
#define LCD_MAX_WIDTH  600

#define SAMPLE_TIMES        3
#define TP_SAMPLE_DIFF_THRES   160
#define TP_POLL_DELAY          8
#define TP_POLL_PERIOD         4

#define TP_STATE_IDLE         0
#define TP_STATE_DOWN         1

typedef struct
{
    INT16 x;
    INT16 y;
}POINT;

static POINT gADPoint;

struct AK4183_TS_EVENT{	

	struct input_dev *input;
	struct i2c_client *client;
	int 	irq;
	struct delayed_work work;	
};
struct AK4183_TS_EVENT *g_ts_dev = NULL;

int screen_x[5] 	= {50, 974,  50, 974, 512};
int screen_y[5] 	= {50,  50, 550, 550, 300};
int uncali_x[5] 	= {0};
int uncali_y[5] 	= {0};

unsigned char tp_state = TP_STATE_IDLE;

static ssize_t ak4183_set_debug_levlel(struct device_driver *_drv, const char *_buf, size_t _count)
{
	printk("ak4183_set_debug_levlel = %s", _buf);

	switch(_buf[0]){
	case '0':
		ak4183_debug = 0;
		break;
	case '1':
		ak4183_debug = 1;
		break;
	case '2':
		ak4183_debug = 2;
		break;
	default:
		break;
	}

	return _count;
}

static unsigned short ak4183_normal_i2c[] = {AK4183_I2C_ADDR>>1, I2C_CLIENT_END};
static unsigned short ak4183_i2c_ignore = I2C_CLIENT_END;

static struct i2c_client_address_data ak4183_i2c_addr_data={
	.normal_i2c = ak4183_normal_i2c,
	.probe = &ak4183_i2c_ignore,
	.ignore =&ak4183_i2c_ignore,
};

static int ak4183_i2c_attach_adapter(struct i2c_adapter *adap);
static int ak4183_i2c_detach_client(struct i2c_client *client);
static void ak4183_i2c_shutdown(struct i2c_client *client);

static struct i2c_driver ak4183_i2c_driver  = {
	.driver = {
		.name = "ak4183",
		.owner = THIS_MODULE,
	},
	.id = AK4183_I2C_ADDR,
	.attach_adapter = ak4183_i2c_attach_adapter,
	.detach_client 	= ak4183_i2c_detach_client,
	.shutdown     	= ak4183_i2c_shutdown,
};

static struct  i2c_client ak4183_i2c_client = {
		.driver = &ak4183_i2c_driver,
		.name	= "ak4183",
};

static int ak4183_i2c_probe(struct i2c_adapter *bus, int address, int kind)
{
	int ret;

	D("enter!\n");
	
	ak4183_i2c_client.adapter = bus;
	ak4183_i2c_client.addr= AK4183_I2C_ADDR >> 1;
	ak4183_i2c_client.mode = NORMALMODE;
	ak4183_i2c_client.Channel = I2C_CH1;
	ak4183_i2c_client.addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	ak4183_i2c_client.speed = 60;

	ret = i2c_attach_client(&ak4183_i2c_client);

	if (ret < 0){
		E("i2c_attach_client err!\n");
	}

	return ret;
}


static int ak4183_i2c_attach_adapter(struct i2c_adapter *adap)
{
	D("enter!\n");
	
	return i2c_probe(adap, &ak4183_i2c_addr_data, ak4183_i2c_probe);
}

static int ak4183_i2c_detach_client(struct i2c_client *client)
{
	int ret;
	
	D("enter!\n");
	
	ret = i2c_detach_client(&ak4183_i2c_client);
	if (ret < 0){
		E("i2c_detach_client!\n");
	}
	
	return ret;
}

static void ak4183_i2c_shutdown(struct i2c_client *client)
{
	D("enter!\n");
}

static int ak4183_i2c_transfer(unsigned short flags, unsigned char *buf, int len)
{
	int ret;
	struct i2c_msg msgs = {
		.addr = AK4183_I2C_ADDR >> 1,
		.flags = flags,
		.len = len,
		.buf = buf,
	};

	ret = i2c_transfer(ak4183_i2c_client.adapter, &msgs, 1);
	if (ret < 0){
		E("i2c_transfer err, ret = %d\n", ret);
	}

	return ret;
}

static int ak4183_read_regs(unsigned char reg_add, unsigned char *buf, int len)
{
	int ret;

	if(len < 1){
		E("len = %d\n", len);
		return -1;
	}

	buf[0] = reg_add;
	ret = ak4183_i2c_transfer(1, buf, len);
	if(ret < 0){
		E("ak4183_read err!\n");
	}

	return ret;
}

/*
static int ak4183_write_regs(unsigned char reg_add, unsigned char *buf, int len)
{
	int ret;
	unsigned char *write_buf= NULL;

	write_buf = kmalloc(len + 1, GFP_KERNEL);
	if(write_buf == NULL){
		E("malloc write buf err!\n");
		return -ENOMEM; 
	}

	write_buf[0] = reg_add;
	if(buf != NULL && len > 0){
		memcpy(write_buf + 1, buf, len);
	}

	ret = ak4183_i2c_transfer(0, write_buf, len + 1);
	if(ret < 0){
		E("ak4183_i2c_transfer err!\n");
	}

	kfree(write_buf);
	write_buf = NULL;

	return ret;
}
*/

static int ak4183_read_x(int *xpos)
{
	unsigned char buf[8];
	int ret;
	
	/* read xpos, 0xC0 for 12bit, 0xC2 for 10bit */
	ret = ak4183_read_regs(0xC0, buf, 2);
	if(ret < 0){
		E("read xpos err!\n");
		return ret;
	}
	
	*xpos = ((buf[0] << 8) | buf[1]) >> 4; 
	return 0;
}

static int ak4183_read_y(int *ypos)
{
	unsigned char buf[8];
	int ret;

	/* read ypos, 0xD0 for 12bit, 0xD2 for 10bit */
	ret = ak4183_read_regs(0xD0, buf, 2);
	if(ret < 0){
		E("read ypos err!\n");
		return ret;
	}
	
	*ypos = ((buf[0] << 8) | buf[1]) >> 4;
	return 0;
}

#ifdef CONFIG_ANDROID_POWER
static void ak4183_suspend(android_early_suspend_t *h)
{
	D("enter!\n");

	gpio_irq_disable(GPIOPortE_Pin3);
}

static void ak4183_resume(android_early_suspend_t *h)
{
	D("enter!\n");

	/* disable ak4183 irq */
	gpio_irq_enable(GPIOPortE_Pin3);
}

static android_early_suspend_t ts_early_suspend;
#endif 

static int tp_average_filter(int *cor, int *filtered_val)
{
    int diffab, diffbc, diffac;
    int group_a, group_b, group_c;
    
    if ((cor == NULL) || (filtered_val == NULL))
        return -1;
        
    group_a = cor[0];
    group_b = cor[1];
    group_c = cor[2];

    
    diffab = abs(group_a - group_b);
    diffac = abs(group_a - group_c);
    diffbc = abs(group_b - group_c);
	
	I("ab=%d, ac=%d, bc=%d\n", diffab, diffac, diffbc);

    if ((diffab > TP_SAMPLE_DIFF_THRES) ||
        (diffbc > TP_SAMPLE_DIFF_THRES))
        return -2;

    if ((diffab <= diffac) && (diffab <= diffbc))
    {
        *filtered_val = (group_a + group_b) / 2;
    }
    else if ((diffac <= diffab) && (diffac <= diffbc))
    {
        *filtered_val = (group_a + group_c) / 2;
    }
    else
    {
        *filtered_val = (group_b + group_c) / 2;
    }
                
    return 0;
}

static irqreturn_t ak4183_isr(int irq,void *dev_id)
{
	struct AK4183_TS_EVENT *ts_dev = (struct AK4183_TS_EVENT *)dev_id;

	D("enter!\n");

	gpio_irq_disable(GPIOPortE_Pin3);

	schedule_delayed_work(&ts_dev->work, TP_POLL_DELAY);
	
	return IRQ_HANDLED;
}

void ak4183_delay_work(struct work_struct *work)
{
	struct AK4183_TS_EVENT *ts_dev = g_ts_dev;
	int ak4183_irq_pin_level = 0;
	int ak4183_input_report_flag = 0;
	int raw_x[SAMPLE_TIMES] = {0}; /* x raw adc data */
	int raw_y[SAMPLE_TIMES] = {0}; /* y raw adc data */
    int filtered_x = 0;
	int filtered_y = 0;
    int xpos;
	int ypos;
	int ret = 0;	
	int i;
	
	D("enter!\n");

	ak4183_irq_pin_level = GPIOGetPinLevel(GPIOPortE_Pin3);

	/* touch */
	if(ak4183_irq_pin_level == 0){
		/* get raw data of x and y */
		for(i = 0; i < SAMPLE_TIMES; i++){
			ret |= ak4183_read_x(&raw_x[i]);
			ret |= ak4183_read_y(&raw_y[i]);
			if(ret != 0){
				E("an4183_read xpos or ypos err!\n");
				goto fake_touch;
			}
		}

		/* check raw data whether valid */
	    for (i = 0; i < SAMPLE_TIMES; i++)
	    {
	        if (raw_x[i] == 4095 || raw_x[i] == 0 ||
			    raw_y[i] == 4095 || raw_y[i] == 0){
			    I("raw data invalid\n");
	            goto fake_touch;
	        }
	    }

		/* filter raw data */
	    ret = tp_average_filter(raw_x, &filtered_x);
		if(ret < 0){
			I("x fake touch\n");
			goto fake_touch;
		}

		ret = tp_average_filter(raw_y, &filtered_y);
	    if (ret < 0){
			I("y fake touch\n");
	        goto fake_touch;
	    }

	    TouchPanelCalibrateAPoint(filtered_x, filtered_y, &xpos, &ypos);
		xpos = xpos / 4;
	    ypos = ypos / 4;

		if(tp_state == TP_STATE_IDLE){
			input_report_key(ts_dev->input, BTN_TOUCH, 1);
			ak4183_input_report_flag = 1;
			tp_state = TP_STATE_DOWN;
		}
		
		if(tp_state == TP_STATE_DOWN){
			I("xpos=%d, ypos=%d\n", xpos, ypos);
			I("gADPoint.x=%d, gADPoint.y=%d\n", gADPoint.x, gADPoint.y);
			gADPoint.x = filtered_x;
			gADPoint.y = filtered_y;
			
			input_report_abs(ts_dev->input, ABS_X, xpos);
			input_report_abs(ts_dev->input, ABS_Y, ypos);
			ak4183_input_report_flag = 1;
		}
	}

fake_touch:
	if(ak4183_irq_pin_level == 1 && tp_state == TP_STATE_DOWN){
		input_report_key(ts_dev->input, BTN_TOUCH, 0);
		ak4183_input_report_flag = 1;
		tp_state = TP_STATE_IDLE;
	}

	if(ak4183_input_report_flag != 0){
		input_sync(ts_dev->input);
	}

	/* still on touch */
	if(ak4183_irq_pin_level == 0){
		schedule_delayed_work(&ts_dev->work, TP_POLL_PERIOD);
	}
	/* untouch, open ts irq */
	else{
		gpio_irq_enable(GPIOPortE_Pin3);
	}
}

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

	return _count; 
}

//This code is Touch adc simple value
static ssize_t touch_adc_show(struct device_driver *_drv,char *_buf)
{
	D("gADPoint.x=%d, gADPoint.y=%d\n", gADPoint.x, gADPoint.y);
    
	return sprintf(_buf, "%d,%d\n", gADPoint.x, gADPoint.y);
}

static ssize_t touch_cali_status(struct device_driver *_drv, char *_buf)
{
    int ret;

	D("enter!\n");
    ret = TouchPanelSetCalibration(4, screen_x, screen_y, uncali_x, uncali_y);
    if (ret == 1)
        ret = sprintf(_buf, "successful\n");
    else
        ret = sprintf(_buf, "fail\n");
    
    printk("Calibration status: _buf=<%s", _buf);
    
	return ret;
}

static DRIVER_ATTR(touchcheck,0666,touch_mode_show,touch_mode_store);
static DRIVER_ATTR(touchadc,0666,touch_adc_show,NULL);
static DRIVER_ATTR(calistatus, 0666, touch_cali_status, NULL);
static DRIVER_ATTR(debug_ak4183, 0666, NULL, ak4183_set_debug_levlel);

static int __init rk28_ts_ak4183_init(void)
{
	int ret;
	struct AK4183_TS_EVENT *ts_dev = NULL;

	GPIOSetPinDirection(GPIOPortE_Pin3,GPIO_IN);
	GPIOSetPinLevel(GPIOPortE_Pin3,GPIO_HIGH);

	ts_dev = kzalloc(sizeof(struct AK4183_TS_EVENT), GFP_KERNEL);
	if(ts_dev == NULL)
	{
		E("ts_dev kzalloc failed!\n");
		ret = -ENOMEM;
		goto err1;
	}

	ts_dev->input = input_allocate_device();
	if(ts_dev->input == NULL)
	{
		E("input_allocate_device err!\n"); 
		ret = -ENOMEM;
		goto err2;
	}	

	ts_dev->irq = 7;
	
	ts_dev->input->name = "ak4183";	
	ts_dev->input->phys = "ak4183_TS/input0";
	ts_dev->input->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY);
	ts_dev->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(ts_dev->input, ABS_X, 0, LCD_MAX_LENGTH - 1, 0, 0);
	input_set_abs_params(ts_dev->input, ABS_Y, 0, LCD_MAX_WIDTH - 1, 0, 0);
	
	ret = input_register_device(ts_dev->input);
	if(ret < 0){
		goto err3;

	}
#ifdef CONFIG_ANDROID_POWER
	ts_early_suspend.suspend = ak4183_suspend;
    ts_early_suspend.resume = ak4183_resume;
    android_register_early_suspend(&ts_early_suspend);
#endif

    rockchip_mux_api_set(GPIOB_SPI0_MMC0_NAME, 0);
	rockchip_mux_api_set(GPIOB4_SPI0CS0_MMC0D4_NAME, IOMUXA_GPIO0_B4);
	GPIOSetPinDirection(GPIOPortB_Pin4,GPIO_IN);
	GPIOSetPinLevel(GPIOPortB_Pin4,GPIO_HIGH);
	GPIOSetPinDirection(GPIOPortB_Pin5,GPIO_IN);
	GPIOSetPinLevel(GPIOPortB_Pin5,GPIO_HIGH);
	GPIOSetPinDirection(GPIOPortB_Pin6,GPIO_IN);
	GPIOSetPinLevel(GPIOPortB_Pin6,GPIO_HIGH);
	GPIOSetPinDirection(GPIOPortB_Pin7,GPIO_IN);
	GPIOSetPinLevel(GPIOPortB_Pin7,GPIO_HIGH);

	INIT_DELAYED_WORK(&ts_dev->work, ak4183_delay_work);

	ret = request_gpio_irq(GPIOPortE_Pin3, (void *)ak4183_isr, GPIOEdgelFalling, ts_dev);
	if(ret < 0){
		E("ak4183 request irq err, ret = %d\n", ret);
		goto err4;
	}	

	ret =  i2c_add_driver(&ak4183_i2c_driver);
	if(ret < 0){
		E("i2c_add_driver err, ret = %d\n", ret);
		goto err5;
	}
	
	g_ts_dev = ts_dev;
	
	ret = driver_create_file(&ak4183_i2c_driver.driver, &driver_attr_touchcheck);
	ret += driver_create_file(&ak4183_i2c_driver.driver, &driver_attr_touchadc);
	ret += driver_create_file(&ak4183_i2c_driver.driver, &driver_attr_calistatus);
	ret += driver_create_file(&ak4183_i2c_driver.driver, &driver_attr_debug_ak4183);

	I("ak4183 init success!\n");

	return 0;

err5:
	free_irq(7, NULL);
err4:
	input_unregister_device(ts_dev->input);
err3:
	input_free_device(ts_dev->input);
err2:
	kfree(ts_dev);
err1:
	return ret;
}

static void __exit rk28_ts_ak4183_exit(void)
{
	D("enter!\n");

    driver_remove_file(&ak4183_i2c_driver.driver, &driver_attr_touchcheck);
    driver_remove_file(&ak4183_i2c_driver.driver, &driver_attr_touchadc);
    driver_remove_file(&ak4183_i2c_driver.driver, &driver_attr_calistatus);
	driver_remove_file(&ak4183_i2c_driver.driver, &driver_attr_debug_ak4183);

	i2c_del_driver(&ak4183_i2c_driver);
	free_irq(7, NULL);
	input_unregister_device(g_ts_dev->input);
	input_free_device(g_ts_dev->input);
	kfree(g_ts_dev);
	g_ts_dev = NULL;
}

module_init(rk28_ts_ak4183_init);
module_exit(rk28_ts_ak4183_exit);

MODULE_DESCRIPTION ("AK4183 touchscreen driver");
MODULE_LICENSE("GPL");



