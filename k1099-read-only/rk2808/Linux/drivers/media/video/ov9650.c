/*
 * Driver for MT9M001 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>


#if 1
static const char ov9650_init_data[][2] =
{
    {0x12, 0x80},
    {0x11, 0x00},//0x82:为20貞；//0x02：为10貞；
    {0x6b, 0x0a},
    {0x6a, 0x64},
    {0x3b, 0x09}, // night mode
    {0x13, 0xe0},
    {0x01, 0x80},
    {0x02, 0x80},
    {0x00, 0x00},
    {0x10, 0x00},
    {0x13, 0xe5},
    {0x39, 0x50},
    {0x38, 0x92},
    {0x37, 0x00},
    {0x35, 0x81},
    {0x0e, 0x20},
    {0x1e, 0x24},//34：第一个水平线有异常 -> ;//04: <- ;//14: ;//旋转度
    {0xA8, 0x80},
    {0x12, 0x10},
    {0x04, 0x00},
    {0x0c, 0x04},
    {0x0d, 0x80},
    {0x18, 0xc7},
    {0x17, 0x27},
    {0x32, 0xbd},
    {0x03, 0x36},
    {0x1a, 0x1e},
    {0x19, 0x00},
    {0x3f, 0xa6},
    {0x14, 0x1e},//降低增益，减小躁声
    {0x15, 0x00},//0x02
    {0x41, 0x02},
    {0x42, 0x08},
    {0x1b, 0x00},
    {0x16, 0x06},
    {0x33, 0xe2},
    {0x34, 0xbf},
    {0x96, 0x04},
    {0x3a, 0x00},
    {0x8e, 0x00},
    {0x3c, 0x77},
    {0x8B, 0x06},
    {0x94, 0x88},
    {0x95, 0x88},
    {0x40, 0xc1},
    {0x29, 0x3f},
    {0x0f, 0x42},
    {0x3d, 0x92},
    {0x69, 0x40},
    {0x5C, 0xb9},
    {0x5D, 0x96},
    {0x5E, 0x10},
    {0x59, 0xc0},
    {0x5A, 0xaf},
    {0x5B, 0x55},
    {0x43, 0xf0},
    {0x44, 0x10},
    {0x45, 0x68},
    {0x46, 0x96},
    {0x47, 0x60},
    {0x48, 0x80},
    {0x5F, 0xe0},
    {0x60, 0x8c},
    {0x61, 0x20},
    {0xa5, 0xd9},
    {0xa4, 0x74},
    {0x8d, 0x02},
    {0x13, 0xe7},
    {0x4f, 0x46},
    {0x50, 0x49},
    {0x51, 0x04},
    {0x52, 0x16},
    {0x53, 0x2e},
    {0x54, 0x43},
    {0x55, 0x40},
    {0x56, 0x40},
    {0x57, 0x40},
    {0x58, 0x0d},
    {0x8C, 0x23},
    {0x3E, 0x02},
    {0xa9, 0xb8},
    {0xaa, 0x92},
    {0xab, 0x0a},
    {0x8f, 0xdf},
    {0x90, 0x00},
    {0x91, 0x00},
    {0x9f, 0x00},
    {0xa0, 0x00},
    {0x3A, 0x0D},
    {0x24, 0x70},
    {0x25, 0x64},
    {0x26, 0xc3},
    {0x0f, 0x4a},
    {0x27, 0x20},
    {0x28, 0x20},
    {0x2c, 0x20},
    {0x2a, 0x10},
    {0x2b, 0x40},
    {0x6c, 0x40},
    {0x6d, 0x30},
    {0x6e, 0x4b},
    {0x6f, 0x60},
    {0x70, 0x70},
    {0x71, 0x70},
    {0x72, 0x70},
    {0x73, 0x70},
    {0x74, 0x60},
    {0x75, 0x60},
    {0x76, 0x50},
    {0x77, 0x48},
    {0x78, 0x3a},
    {0x79, 0x2e},
    {0x7a, 0x28},
    {0x7b, 0x22},
    {0x7c, 0x04},
    {0x7d, 0x07},
    {0x7e, 0x10},
    {0x7f, 0x28},
    {0x80, 0x36},
    {0x81, 0x44},
    {0x82, 0x52},
    {0x83, 0x60},
    {0x84, 0x6c},
    {0x85, 0x78},
    {0x86, 0x8c},
    {0x87, 0x9e},
    {0x88, 0xbb},
    {0x89, 0xd2},
    {0x8a, 0xe6}

};
#else
static const char ov9650_init_data[][2] =
{
//	VGA 10fps		 
	{0x12,0x80},
	{0x11,0x80/*0x82*/},
	{0x6b,0x0a},
	{0x6a,0x32},
	{0x3b,0x09},
	{0x13,0xe0},
	{0x01,0x80},
	{0x02,0x80},
	{0x00,0x00},
	{0x10,0x00},
	{0x13,0xe5},
	{0x39,0x50/*0x43*/}, //;50 for 30fps
	{0x38,0x92/*0x12*/}, //;92 for 30fps
	{0x37,0x00},
	{0x35,0x81/*0x91*/}, //;81 for 30fps
	{0x0e,0x20},
	{0x1e,0x04},
	{0xA8,0x80},
	{0x12,0x40},
	{0x04,0x00},
	{0x0c,0x04},
	{0x0d,0x80},
	{0x18,0xc6},
	{0x17,0x26},
	{0x32,0xad},
	{0x03,0x00},
	{0x1a,0x3d},
	{0x19,0x01},
	{0x3f,0xa6},
	{0x14,0x2e},
	{0x15,0x02},
	{0x41,0x02},
	{0x42,0x08},
	{0x1b,0x00},
	{0x16,0x06},
	{0x33,0xe2}, //;c0 for internal regulator
	{0x34,0xbf},
	{0x96,0x04},
	{0x3a,0x00},
	{0x8e,0x00},
	{0x3c,0x77},
	{0x8B,0x06},
	{0x94,0x88},
	{0x95,0x88},
	{0x40,0xc1},
	{0x29,0x3f}, //;2f for internal regulator
	{0x0f,0x42},
	{0x3d,0x92},
	{0x69,0x40},
	{0x5C,0xb9},
	{0x5D,0x96},
	{0x5E,0x10},
	{0x59,0xc0},
	{0x5A,0xaf},
	{0x5B,0x55},
	{0x43,0xf0},
	{0x44,0x10},
	{0x45,0x68},
	{0x46,0x96},
	{0x47,0x60},
	{0x48,0x80},
	{0x5F,0xe0},
	{0x60,0x8c}, //;0c for advanced AWB (related to lens)
	{0x61,0x20},
	{0xa5,0xd9},
	{0xa4,0x74},
	{0x8d,0x02},
	{0x13,0xe7},
	{0x4f,0x3a},
	{0x50,0x3d},
	{0x51,0x03},
	{0x52,0x12},
	{0x53,0x26},
	{0x54,0x38},
	{0x55,0x40},
	{0x56,0x40},
	{0x57,0x40},
	{0x58,0x0d},
	{0x8C,0x23},
	{0x3E,0x02},
	{0xa9,0xb8},
	{0xaa,0x92},
	{0xab,0x0a},
	{0x8f,0xdf},
	{0x90,0x00},
	{0x91,0x00},
	{0x9f,0x00},
	{0xa0,0x00},
	{0x3A,0x0D},
	{0x24,0x70},
	{0x25,0x64},
	{0x26,0xc3},
	{0x2a,0x00}, //;10 for 50Hz
	{0x2b,0x00}, //;40 for 50Hz
	{0x6c,0x40},
	{0x6d,0x30},
	{0x6e,0x4b},
	{0x6f,0x60},
	{0x70,0x70},
	{0x71,0x70},
	{0x72,0x70},
	{0x73,0x70},
	{0x74,0x60},
	{0x75,0x60},
	{0x76,0x50},
	{0x77,0x48},
	{0x78,0x3a},
	{0x79,0x2e},
	{0x7a,0x28},
	{0x7b,0x22},
	{0x7c,0x04},
	{0x7d,0x07},
	{0x7e,0x10},
	{0x7f,0x28},
	{0x80,0x36},
	{0x81,0x44},
	{0x82,0x52},
	{0x83,0x60},
	{0x84,0x6c},
	{0x85,0x78},
	{0x86,0x8c},
	{0x87,0x9e},
	{0x88,0xbb},
	{0x89,0xd2},
	{0x8a,0xe6},
	/*
	//---QVGA
	{0x03, 0x36},	
	{0x04, 0x00},
	{0x0c, 0x04},
	{0x0d, 0x80},
	{0x12, 0x10},
	{0x17, 0x26},
	{0x18, 0xc6},
	{0x19, 0x00},
	{0x1A, 0x1e},
	{0x32, 0xa4},// Flicker Setting (Default : 60Hz)
	{0x2a, 0x00},	// 50Hz -> 10
	{0x2b, 0x00},	// 50Hz -> 40
	*/   		
};
#endif

/*1280*1024*/
static const char ov9650_sxga[][2]=
{   {0x04, 0x00},
    {0xa8, 0x80},
    {0x0c, 0x00},
    {0x0d, 0x00},
    {0x11, 0x80},
    {0x6b, 0x0a},
    {0x6a, 0x41},
    {0x12, 0x00},
    {0x18, 0xbd},
    {0x17, 0x1d},
    {0x32, 0xbd},
    {0x03, 0x12},
    {0x1a, 0x81},
    {0x19, 0x01},
    {0x39, 0x43},
    {0x38, 0x12},
    {0x35, 0x91},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x2a, 0x10},
    {0x2b, 0x34}

};

/*640*480*/
static const char ov9650_vga[][2]= 
{
    {0xa8, 0x80},
    {0x0c, 0x04},
    {0x0d, 0x80},
    {0x11, 0x00},
    {0x6b, 0x0a},
    {0x6a, 0x3e},
    {0x12, 0x40},
    {0x18, 0xc7},
    {0x17, 0x27},
    {0x32, 0xbd},
    {0x03, 0x00},
    {0x1a, 0x3d},
    {0x19, 0x01},
    {0x39, 0x50},
    {0x38, 0x92},
    {0x35, 0x81},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x2a, 0x10},
    {0x2b, 0x40}

};

/*352*288*/
static const char ov9650_qcif[][2]= 
{
    {0x0c ,0x04},
    {0x0d ,0x80},
    {0x11 ,0x80},
    {0x12 ,0x20},
    {0x13 ,0xe5},
    {0x18 ,0xc7},
    {0x17 ,0x27},
    {0x03 ,0x00},
    {0x1a ,0x3d},
    {0x19 ,0x01},
    {0x39 ,0x50},
    {0x38 ,0x92},
    {0x35 ,0x81},
};

/*320*240*/
static const char ov9650_qvga[][2]=
{
    {0x12, 0x10},
    {0xa8, 0x80},
    {0x04, 0x00},
    {0x0c, 0x04},
    {0x0d, 0x80},
    {0x18, 0xc7},
    {0x17, 0x27},
    {0x32, 0xbd},
    {0x03, 0x36},
    {0x1a, 0x1e},
    {0x19, 0x00},
    {0x11, 0x00},
    {0x6b, 0x0a},
    {0x92, 0x00},
    {0x93, 0x00},
    {0x2a, 0x10},
    {0x2b, 0x40},
    {0x6a, 0x3e},
    {0x3b, 0x09}
};



static const struct soc_camera_data_format ov9650_colour_formats = {
	/* Order important: first natively supported,
	 * second supported with a GPIO extender */
		.name		= "ov9650 uvyv",
		.depth		= 8,
		.fourcc		= V4L2_PIX_FMT_UYVY,
};


struct ov9650 {
	struct i2c_client *client;
	struct soc_camera_device icd;
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;
};

#define OV9650_IIC_ADDR 	    0x60  

static unsigned short normal_i2c[] = {OV9650_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_ov9650 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static int ov9650_probe(struct i2c_adapter *adapter, int addr, int kind);
static int ov9650_video_probe(struct soc_camera_device *);
static void ov9650_video_remove(struct soc_camera_device *);
static int ov9650_get_control(struct soc_camera_device *, struct v4l2_control *);
static int ov9650_set_control(struct soc_camera_device *, struct v4l2_control *);


static int ov9650_rx_data(struct i2c_client *this_client, char *rxData, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 1,
		 .len = length,
		 .buf = rxData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msgs, 1) < 0) {
		printk(KERN_ERR "ov9650 ov9650_rx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int ov9650_tx_data(struct i2c_client *this_client, char *txData, int length)
{

	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "ov9650 ov9650_tx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int ov9650_init(struct soc_camera_device *icd)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);
    int i;

    for (i = 0; i < sizeof(ov9650_init_data) / 2; i++) {
        if (0 > ov9650_tx_data(ov9650->client, &ov9650_init_data[i][0], 2))
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
    }

    char chekid[2];
    chekid[0] = 0x0a;
    ov9650_rx_data(ov9650->client, &chekid[0], 1);
    printk("\n%s..%s..%d    ******** nzy *********0x%x\n",__FUNCTION__,__FILE__,__LINE__, chekid[0]);

	return 0;
}

static int ov9650_release(struct soc_camera_device *icd)
{
	return 0;
}

static int ov9650_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int ov9650_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}
#if (defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_E700))
#define SENSOR_RESET   	GPIOPortB_Pin7		// SENSOR RESET
#define SENSOR_POWERDN   GPIOPortH_Pin6		// SENSOR POWER
#elif(defined(CONFIG_BOARD_IPADV5))
#define SENSOR_RESET   	GPIOPortB_Pin1		// SENSOR RESET
#define SENSOR_POWERDN   GPIOPortH_Pin6		// SENSOR POWER
#elif defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_NM701)
#define SENSOR_RESET   	GPIOPortF_Pin4		// SENSOR RESET
#define SENSOR_POWERDN   GPIOPortH_Pin6		// SENSOR POWER
#endif
static int bus_switch_request(struct ov9650 *ov9650,
			      struct soc_camera_link *icl)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "ov9650");
    if (ret < 0) {
        dev_err(&ov9650->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&ov9650->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	ov9650->switch_gpio = gpio;
    */

#if(defined(CONFIG_BOARD_ZTX))
#else
    rockchip_mux_api_set(GPIOH6_IQ_SEL_NAME, IOMUXB_GPIO1_D6);
    gpio_direction_output(GPIOPortH_Pin6, GPIO_OUT);
    __gpio_set(GPIOPortH_Pin6, GPIO_LOW);
#if (defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_IPADV5)||defined(CONFIG_BOARD_E700)||defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_NM701))
	GPIOSetPinDirection(SENSOR_RESET,GPIO_OUT);
	GPIOSetPinLevel(SENSOR_RESET,GPIO_HIGH);	//
	mdelay(500);
	GPIOSetPinLevel(SENSOR_RESET,GPIO_LOW);	//
#endif    
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
#endif
    
	return 0;
}

static void bus_switch_release(struct ov9650 *ov9650)
{

}

static int ov9650_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);
	int ret;

	return 0;
}

static unsigned long ov9650_query_bus_param(struct soc_camera_device *icd)
{
	/* 0v9650 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_SENSOR_UYVY;
}

static int ov9650_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);
	int ret;
    int i;
    char *ov9650_win;
    unsigned int ov9650_win_data_len;

    if ((rect->width == icd->width) && (rect->height == icd->height)) { 
        printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);
        return 0;
    }

#if 1    
    if ((rect->width <= 320) && (rect->height <= 240)) {
        ov9650_win = ov9650_qvga;
        ov9650_win_data_len = sizeof(ov9650_qvga);
    } else if ((rect->width <= 352) && (rect->height <= 288)) {
//        ov9650_win = ov9650_qcif;
//        ov9650_win_data_len = sizeof(ov9650_qcif);
        ov9650_win = ov9650_vga;
        ov9650_win_data_len = sizeof(ov9650_vga);
    } else if ((rect->width <= 640) && (rect->height <= 480)){
#else
    if ((rect->width <= 640) && (rect->height <= 480)){
#endif    
        ov9650_win = ov9650_vga;
        ov9650_win_data_len = sizeof(ov9650_vga);
    } else {
        ov9650_win = ov9650_sxga;
        ov9650_win_data_len = sizeof(ov9650_sxga);
    }
    
    for (i = 0; i < ov9650_win_data_len / 2; i++) {
        ret = ov9650_tx_data(ov9650->client, ov9650_win + 2*i, 2);
        if (0 > ret) {
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
            return ret;
        }
    }
    
    //printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);

	return 0;
}

static int ov9650_try_fmt_cap(struct soc_camera_device *icd,
			       struct v4l2_format *f)
{
	if (f->fmt.pix.height < 32 + icd->y_skip_top)
		f->fmt.pix.height = 32 + icd->y_skip_top;
	if (f->fmt.pix.height > 1024 + icd->y_skip_top)
		f->fmt.pix.height = 1024 + icd->y_skip_top;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 1280)
		f->fmt.pix.width = 1280;
	f->fmt.pix.width &= ~0x01; /* has to be even, unsure why was ~3 */

	return 0;
}

static int ov9650_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != ov9650->client->addr)
		return -ENODEV;

	id->ident	= ov9650->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov9650_get_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov9650->client->addr)
		return -ENODEV;

    char reg = reg->reg;
    int ret = ov9650_rx_data(ov9650->client, &reg, 1);
    if (!ret)
        reg->val = reg;

	return ret;
}

static int ov9650_set_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != ov9650->client->addr)
		return -ENODEV;

    char reg[2];
    reg[0] = reg->reg;
    reg[1] = reg->val;
    int ret = ov9650_tx_data(ov9650->client, reg, 2);

	return ret;
}
#endif

static const struct v4l2_queryctrl ov9650_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain",
		.minimum	= 0,
		.maximum	= 127,
		.step		= 1,
		.default_value	= 64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 1,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 255,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}
};

static int ov9650_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

static struct soc_camera_ops ov9650_ops = {
	.owner			= THIS_MODULE,
	.probe			= ov9650_video_probe,
	.remove			= ov9650_video_remove,
	.init			= ov9650_init,
	.release		= ov9650_release,
	.start_capture		= ov9650_start_capture,
	.stop_capture		= ov9650_stop_capture,
	.set_fmt_cap		= ov9650_set_fmt_cap,
	.try_fmt_cap		= ov9650_try_fmt_cap,
	.set_bus_param		= ov9650_set_bus_param,
	.query_bus_param	= ov9650_query_bus_param,
	.controls		= ov9650_controls,
	.num_controls		= ARRAY_SIZE(ov9650_controls),
	.get_control		= ov9650_get_control,
	.set_control		= ov9650_set_control,
	.get_chip_id		= ov9650_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= ov9650_get_register,
	.set_register		= ov9650_set_register,
#endif
};

static int ov9650_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);
	const struct v4l2_queryctrl *qctrl;
	int data;

	qctrl = soc_camera_find_qctrl(&ov9650_ops, ctrl->id);

	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov9650_video_probe(struct soc_camera_device *icd)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);
	s32 data;
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

    ov9650->model = V4L2_IDENT_OV9650;
    icd->formats = &ov9650_colour_formats;
    icd->num_formats = 1;

	/* Now that we know the model, we can start video */
	ret = soc_camera_video_start(icd);
	if (ret)
		goto eisis;

	return 0;

eisis:
ei2c:
	return ret;
}

static void ov9650_video_remove(struct soc_camera_device *icd)
{
	struct ov9650 *ov9650 = container_of(icd, struct ov9650, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", ov9650->client->addr,
		ov9650->icd.dev.parent, ov9650->icd.vdev);
	soc_camera_video_stop(&ov9650->icd);
}

static int ov9650_remove(struct i2c_client *client)
{
	struct ov9650 *ov9650 = i2c_get_clientdata(client);

	soc_camera_device_unregister(&ov9650->icd);
	kfree(ov9650);

	return 0;
}

static int ov9650_detach_client(struct i2c_client *client)
{
	ov9650_remove(client);

	return i2c_detach_client(client);
}

static int ov9650_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_ov9650, ov9650_probe);
}

static struct i2c_driver ov9650_driver = {
	.driver = {
		.name = "ov9650",
	    },
	.id 	= OV9650_IIC_ADDR,
	.attach_adapter = &ov9650_attach_adapter,
	.detach_client  = &ov9650_detach_client,
};

static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
    
static int ov9650_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct ov9650 *ov9650;
	struct soc_camera_device *icd;
	struct soc_camera_link *icl;
	struct i2c_client *client = NULL;
	int ret, i;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		ret = -EIO;
        goto exit_check_functionality_failed;
	}

	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!client) {
		ret = -ENOMEM;
        goto exit_alloc_client_failed;
	}

	strlcpy(client->name, "ov9650", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &ov9650_driver;
	client->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 40;//100	
	ret = i2c_attach_client(client);
	if (ret) {
        goto exit_i2c_attach_client_failed;
	}
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	
	ov9650 = kzalloc(sizeof(struct ov9650), GFP_KERNEL);
	if (!ov9650) {
		ret = -ENOMEM;
        goto exit_alloc_data_failed;
	}

	ov9650->client = client;
	i2c_set_clientdata(client, ov9650);
    //icl = &iclink;//client->dev.platform_data;
    
	/* Second stage probe - when a capture adapter is there */
	icd = &ov9650->icd;
	icd->ops	= &ov9650_ops;
	icd->control	= &client->dev;
	icd->x_min	= 0;
	icd->y_min	= 0;
	icd->x_current	= 0;
	icd->y_current	= 0;
	icd->width_min	= 48;
	icd->width_max	= 1280;
	icd->height_min	= 32;
	icd->height_max	= 1024;
	icd->y_skip_top	= 1;
	icd->iface	= 33;//icl->bus_id;
	/* Default datawidth - this is the only width this camera (normally)
	 * supports. It is only with extra logic that it can support
	 * other widths. Therefore it seems to be a sensible default. */
	ov9650->datawidth = 8;
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	ov9650->autoexposure = 1;

	ret = bus_switch_request(ov9650, icl);
	if (ret)
		goto exit_bus_switch_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;
                
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    bus_switch_release(ov9650);
exit_bus_switch_request_failed:
	kfree(ov9650);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init ov9650_mod_init(void)
{
	return i2c_add_driver(&ov9650_driver);
}

static void __exit ov9650_mod_exit(void)
{
	i2c_del_driver(&ov9650_driver);
}

module_init(ov9650_mod_init);
module_exit(ov9650_mod_exit);

MODULE_DESCRIPTION("Micron OV9650 Camera driver");
MODULE_AUTHOR("nzy <kernel@rock-chips>");
MODULE_LICENSE("GPL");
