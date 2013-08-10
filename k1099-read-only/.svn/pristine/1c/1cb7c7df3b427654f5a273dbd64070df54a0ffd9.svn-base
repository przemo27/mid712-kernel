/*
 * Driver for MT9M112 CMOS Image Sensor from Micron
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

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>

/*init data*/
static const u16 mt9m112_init_data[][2] =
{
    0xf0, 0x0000,
    0x35, 0x0022,
    //0x20, 0x0103,  // mirror
    0x0c, 0x0000,
    0x21, 0x8400,

    0x05,0x022e,
   // 0x06,0x0300,

    0xf0, 0x0001,
    0x06,0x708e,
    0x05,0x000e,

    0xf0, 0x0002,
    0xCB, 0x0001,		  // PROGRAM_ADVANCE
    0xCC, 0x0004,		  // PROGRAM_SELECT
    0xD2, 0x0000,		  // DEFAULT_CONFIG
    0x5B, 0x0001,

    0xf0, 0x0001,

    //[Lens Correction 05/16/08 11:19:07]
    0x80, 0x000A , //LENS_CORRECT_CONTROL
    0x81, 0xED0F , //LENS_ADJ_VERT_RED_0
    0x82, 0xF8F1 , //LENS_ADJ_VERT_RED_1_2
    0x83, 0x00FD , //LENS_ADJ_VERT_RED_3_4
    0x84, 0xEF0D , //LENS_ADJ_VERT_GREEN_0
    0x85, 0xF9F3 , //LENS_ADJ_VERT_GREEN_1_2
    0x86, 0x00FD , //LENS_ADJ_VERT_GREEN_3_4
    0x87, 0xF30C , //LENS_ADJ_VERT_BLUE_0
    0x88, 0xF9F3 , //LENS_ADJ_VERT_BLUE_1_2
    0x89, 0x00FD , //LENS_ADJ_VERT_BLUE_3_4
    0x8A, 0xE616 , //LENS_ADJ_HORIZ_RED_0
    0x8B, 0xF5F0 , //LENS_ADJ_HORIZ_RED_1_2
    0x8C, 0xFDF9 , //LENS_ADJ_HORIZ_RED_3_4
    0x8D, 0x0002 , //LENS_ADJ_HORIZ_RED_5
    0x8E, 0xE714 , //LENS_ADJ_HORIZ_GREEN_0
    0x8F, 0xF5F1 , //LENS_ADJ_HORIZ_GREEN_1_2
    0x90, 0xFEFA , //LENS_ADJ_HORIZ_GREEN_3_4
    0x91, 0x0001 , //LENS_ADJ_HORIZ_GREEN_5
    0x92, 0xE913 , //LENS_ADJ_HORIZ_BLUE_0
    0x93, 0xF7F2 , //LENS_ADJ_HORIZ_BLUE_1_2
    0x94, 0xFEFB , //LENS_ADJ_HORIZ_BLUE_3_4
    0x95, 0x0002 , //LENS_ADJ_HORIZ_BLUE_5
    0xB6, 0x0904 , //LENS_ADJ_VERT_RED_5_6
    0xB7, 0x1E0C , //LENS_ADJ_VERT_RED_7_8
    0xB8, 0x0904 , //LENS_ADJ_VERT_GREEN_5_6
    0xB9, 0x1B0A , //LENS_ADJ_VERT_GREEN_7_8
    0xBA, 0x0704 , //LENS_ADJ_VERT_BLUE_5_6
    0xBB, 0x1A0A , //LENS_ADJ_VERT_BLUE_7_8
    0xBC, 0x0906 , //LENS_ADJ_HORIZ_RED_6_7
    0xBD, 0x1410 , //LENS_ADJ_HORIZ_RED_8_9
    0xBE, 0x0026 , //LENS_ADJ_HORIZ_RED_10
    0xBF, 0x0806 , //LENS_ADJ_HORIZ_GREEN_6_7
    0xC0, 0x110E , //LENS_ADJ_HORIZ_GREEN_8_9
    0xC1, 0x0024 , //LENS_ADJ_HORIZ_GREEN_10
    0xC2, 0x0A05 , //LENS_ADJ_HORIZ_BLUE_6_7
    0xC3, 0x100F , //LENS_ADJ_HORIZ_BLUE_8_9
    0xC4, 0x0027 , //LENS_ADJ_HORIZ_BLUE_10
    0x06, 0x640e, //LENS_CORRECTION

    0x53,0x0602,
    0x54,0x4416,
    0x55,0xb68a,
    0x56,0xe0cf,
    0x57,0xf7e0,
    0x58,0xff00,

    //输出窗口大小
    0xA7, 640,
    0xaa, 512,
     
    //背光初始化
    0xf0,	0x0002,
    0x37,   0x0080,//0x0300,
    0x2f,   0x9100,
    0x2e,   0x0c44,//0x0c30, //
    //0x39, 0x03D2,  // AE Line size Context A d:6A6
    //0x3A, 0x03D2,  // AE Line size Context B d:6A6
    //0x3B, 0x055B,  // AE shutter delay limit Context A
    //0x3C, 0x055B,  // AE shutter delay limit Context B
    0x57, 0x2CA,  // Context A Flicker full frame time (60Hz)
    0x58, 0x2CA,		//0x01E9,  // Context A Flicker full frame time (50Hz)
    0x59, 0x2CA,  // Context B Flicker full frame time (60Hz)
    0x5A, 0x2CA,  //0x01E9,  // Context B Flicker full frame time (50Hz)
    0x5C, 0x0E0A,  // 60Hz Flicker Search Range
    0x5D, 0x120E,  // 50Hz Flicker Search Range
    0x64, 0x5E1C,  // Flicker parameter

    0x5B,   0x0001,
}; 

//分辨率

static const u16 mt9m112_qvga[][2] = //320*240
{
    0xf0,	0x0001,
    0xA1, 320,
    0xa4, 256,
};

static const u16 mt9m112_cif[][2] = //352*288
{
    0xf0,	0x0001,
    0xA1, 352,
    0xa4, 288,
    
    0xf0, 0x0000,
    0x35, 0x0022,
    //0x20, 0x0103,  // mirror
    0x0c, 0x0000,
    0x21, 0x8400,

    0x05,0x022e,
   // 0x06,0x0300,

    0xf0, 0x0001,
    0x06,0x708e,
    0x05,0x000e,

    0xf0, 0x0002,
    0xCB, 0x0001,		  // PROGRAM_ADVANCE
    0xCC, 0x0004,		  // PROGRAM_SELECT
    0xD2, 0x0000,		  // DEFAULT_CONFIG
    0x5B, 0x0001,

    0xf0, 0x0001,

    //[Lens Correction 05/16/08 11:19:07]
    0x80, 0x000A , //LENS_CORRECT_CONTROL
    0x81, 0xED0F , //LENS_ADJ_VERT_RED_0
    0x82, 0xF8F1 , //LENS_ADJ_VERT_RED_1_2
    0x83, 0x00FD , //LENS_ADJ_VERT_RED_3_4
    0x84, 0xEF0D , //LENS_ADJ_VERT_GREEN_0
    0x85, 0xF9F3 , //LENS_ADJ_VERT_GREEN_1_2
    0x86, 0x00FD , //LENS_ADJ_VERT_GREEN_3_4
    0x87, 0xF30C , //LENS_ADJ_VERT_BLUE_0
    0x88, 0xF9F3 , //LENS_ADJ_VERT_BLUE_1_2
    0x89, 0x00FD , //LENS_ADJ_VERT_BLUE_3_4
    0x8A, 0xE616 , //LENS_ADJ_HORIZ_RED_0
    0x8B, 0xF5F0 , //LENS_ADJ_HORIZ_RED_1_2
    0x8C, 0xFDF9 , //LENS_ADJ_HORIZ_RED_3_4
    0x8D, 0x0002 , //LENS_ADJ_HORIZ_RED_5
    0x8E, 0xE714 , //LENS_ADJ_HORIZ_GREEN_0
    0x8F, 0xF5F1 , //LENS_ADJ_HORIZ_GREEN_1_2
    0x90, 0xFEFA , //LENS_ADJ_HORIZ_GREEN_3_4
    0x91, 0x0001 , //LENS_ADJ_HORIZ_GREEN_5
    0x92, 0xE913 , //LENS_ADJ_HORIZ_BLUE_0
    0x93, 0xF7F2 , //LENS_ADJ_HORIZ_BLUE_1_2
    0x94, 0xFEFB , //LENS_ADJ_HORIZ_BLUE_3_4
    0x95, 0x0002 , //LENS_ADJ_HORIZ_BLUE_5
    0xB6, 0x0904 , //LENS_ADJ_VERT_RED_5_6
    0xB7, 0x1E0C , //LENS_ADJ_VERT_RED_7_8
    0xB8, 0x0904 , //LENS_ADJ_VERT_GREEN_5_6
    0xB9, 0x1B0A , //LENS_ADJ_VERT_GREEN_7_8
    0xBA, 0x0704 , //LENS_ADJ_VERT_BLUE_5_6
    0xBB, 0x1A0A , //LENS_ADJ_VERT_BLUE_7_8
    0xBC, 0x0906 , //LENS_ADJ_HORIZ_RED_6_7
    0xBD, 0x1410 , //LENS_ADJ_HORIZ_RED_8_9
    0xBE, 0x0026 , //LENS_ADJ_HORIZ_RED_10
    0xBF, 0x0806 , //LENS_ADJ_HORIZ_GREEN_6_7
    0xC0, 0x110E , //LENS_ADJ_HORIZ_GREEN_8_9
    0xC1, 0x0024 , //LENS_ADJ_HORIZ_GREEN_10
    0xC2, 0x0A05 , //LENS_ADJ_HORIZ_BLUE_6_7
    0xC3, 0x100F , //LENS_ADJ_HORIZ_BLUE_8_9
    0xC4, 0x0027 , //LENS_ADJ_HORIZ_BLUE_10
    0x06, 0x640e, //LENS_CORRECTION

    0x53,0x0602,
    0x54,0x4416,
    0x55,0xb68a,
    0x56,0xe0cf,
    0x57,0xf7e0,
    0x58,0xff00,

    //输出窗口大小
    0xA7, 352,
    0xaa, 288,
     
    //背光初始化
    0xf0,	0x0002,
    0x37,   0x0080,//0x0300,
    0x2f,   0x9100,
    0x2e,   0x0c44,//0x0c30, //
    //0x39, 0x03D2,  // AE Line size Context A d:6A6
    //0x3A, 0x03D2,  // AE Line size Context B d:6A6
    //0x3B, 0x055B,  // AE shutter delay limit Context A
    //0x3C, 0x055B,  // AE shutter delay limit Context B
    0x57, 0x2CA,  // Context A Flicker full frame time (60Hz)
    0x58, 0x2CA,		//0x01E9,  // Context A Flicker full frame time (50Hz)
    0x59, 0x2CA,  // Context B Flicker full frame time (60Hz)
    0x5A, 0x2CA,  //0x01E9,  // Context B Flicker full frame time (50Hz)
    0x5C, 0x0E0A,  // 60Hz Flicker Search Range
    0x5D, 0x120E,  // 50Hz Flicker Search Range
    0x64, 0x5E1C,  // Flicker parameter

    0x5B,   0x0001,

};

static const u16 mt9m112_vga[][2]= //640*512
{
    0xf0,	0x0001,
    0xA1, 640,
    0xa4, 512,
};

static const u16 mt9m112_4cif[][2] = //704*576
{
    0xf0,	0x0001,
    0xA1, 704,
    0xa4, 576,
};

static const u16 mt9m112_sxga[][2]= //1280*1024
{
    0xf0,	0x0001,
    0xA1, 1280,
    0xa4, 1024,
    //0xae, 0x0A08,
    
    0xf0,	0x0002,
    0xCB, 0x0001,          // PROGRAM_ADVANCE
    0xCC, 0x0004,          // PROGRAM_SELECT
    0xD2, 0x007f,          // DEFAULT_CONFIG};
};

// context change
static const u16 mt9m112_preview[][2] = //Preveiw
{
    0xf0, 0x0002,
    0xCB, 0x0001,		  // PROGRAM_ADVANCE
    0xCC, 0x0004,		  // PROGRAM_SELECT
    0xD2, 0x0000,		  // DEFAULT_CONFIG

};

static const u16 mt9m112_capture[][2] = //Capture
{

    0xf0,	0x0002,
    0xCB, 0x0001,          // PROGRAM_ADVANCE
    0xCC, 0x0004,          // PROGRAM_SELECT
    0xD2, 0x007f,          // DEFAULT_CONFIG

};

static const struct soc_camera_data_format mt9m112_colour_formats = {
	/* Order important: first natively supported,
	 * second supported with a GPIO extender */
		.name		= "mt9m112 uvyv",
		.depth		= 8,
		.fourcc		= V4L2_PIX_FMT_UYVY,
};


struct mt9m112 {
	struct i2c_client *client;
	struct soc_camera_device icd;
	int model;	/* V4L2_IDENT_MT9M001* codes from v4l2-chip-ident.h */
	int switch_gpio;
	unsigned char autoexposure;
	unsigned char datawidth;
};

#define MT9M112_IIC_ADDR 	    0xBA  

static unsigned short normal_i2c[] = {MT9M112_IIC_ADDR >> 1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data_mt9m112 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};

static int mt9m112_probe(struct i2c_adapter *adapter, int addr, int kind);
static int mt9m112_video_probe(struct soc_camera_device *);
static void mt9m112_video_remove(struct soc_camera_device *);
static int mt9m112_get_control(struct soc_camera_device *, struct v4l2_control *);
static int mt9m112_set_control(struct soc_camera_device *, struct v4l2_control *);


static int mt9m112_rx_data(struct i2c_client *this_client, char *rxData, int length)
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
		printk(KERN_ERR "mt9m112 mt9m112_rx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int mt9m112_tx_data(struct i2c_client *this_client, char *txData, int length)
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
		printk(KERN_ERR "mt9m112 mt9m112_tx_data: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int mt9m112_init(struct soc_camera_device *icd)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);
    int i;
	u16 addr = 0, value = 0;
    u8  data8[3];
    char checkid[2];
	
    checkid[0] = 0x00;
	//checkid[1] = 0x00;
	//mt9m112_tx_data(mt9m112->client, &checkid[0], 3);
    mt9m112_rx_data(mt9m112->client, &checkid[0], 2);
	//mt9m112_rx_data(mt9m112->client, &checkid[0], 2);

    printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, checkid[0],checkid[1]);
    
    for (i = 0; i < sizeof(mt9m112_init_data) / 4; i++) {
		
    	addr  = mt9m112_init_data[i][0];
    	value = mt9m112_init_data[i][1];

    	data8[0]= (u8)addr ;
    	data8[1]= (u8)((value&0xff00)>>8); 	//get the high reg data
    	data8[2]= (u8)(value&0xff); 		//get the low reg data
    	
		if(i<10)
			//printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, data8[0],data8[1],data8[2]);
		
        if (0 > mt9m112_tx_data(mt9m112->client, data8, 3))
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
		
		if(i<10){
    		checkid[0] = data8[0];
			//checkid[1] = 0x00;
	    	mt9m112_rx_data(mt9m112->client, &checkid[0], 2);
    		//printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, data8[0], checkid[0],checkid[1]);
    	}
		
    }

    for (i = 0; i < sizeof(mt9m112_preview) / 4; i++) {
		
    	addr  = mt9m112_preview[i][0];
    	value = mt9m112_preview[i][1];

    	data8[0]= (u8)addr ;
    	data8[1]= (u8)((value&0xff00)>>8); 	//get the high reg data
    	data8[2]= (u8)(value&0xff); 		//get the low reg data
    	
		if(i<10)
			printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, data8[0],data8[1],data8[2]);
		
        //if (0 > mt9m112_tx_data(mt9m112->client, data8, 3))
            //printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
		
		if(i<10 ){
    		checkid[0] = data8[0];
			//checkid[1] = 0x00;
	    	mt9m112_rx_data(mt9m112->client, &checkid[0], 2);
    		printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, data8[0], checkid[0],checkid[1]);
		}
		
    }

	return 0;
}

static int mt9m112_release(struct soc_camera_device *icd)
{
	return 0;
}

static int mt9m112_start_capture(struct soc_camera_device *icd)
{
	return 0;
}

static int mt9m112_stop_capture(struct soc_camera_device *icd)
{
	return 0;
}

#if (defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_IPADV5)||defined(CONFIG_BOARD_E700)||defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_NM701))
#define SENSOR_RESET   	GPIOPortB_Pin7		// SENSOR RESET
#define SENSOR_POWERDN   GPIOPortH_Pin6		// SENSOR POWER

#endif
static int bus_switch_request(struct mt9m112 *mt9m112,
			      struct soc_camera_link *icl)
{
    /*
	unsigned int gpio = icl->gpio;

    int ret = gpio_request(gpio, "mt9m112");
    if (ret < 0) {
        dev_err(&mt9m112->client->dev, "Cannot get GPIO %u\n",
            gpio);
        return ret;
    }

    ret = gpio_direction_output(gpio, 0);
    if (ret < 0) {
        dev_err(&mt9m112->client->dev,
            "Cannot set GPIO %u to output\n", gpio);
        gpio_free(gpio);
        return ret;
    }
	mt9m112->switch_gpio = gpio;
    */

    rockchip_mux_api_set(GPIOH6_IQ_SEL_NAME, IOMUXB_GPIO1_D6);
    gpio_direction_output(GPIOPortH_Pin6, GPIO_OUT);
    __gpio_set(GPIOPortH_Pin6, GPIO_LOW);
    
#if( defined(CONFIG_BOARD_IPAD)||defined(CONFIG_BOARD_IPADV5) ||defined(CONFIG_BOARD_E700)||defined(CONFIG_BOARD_IPAD8)||defined(CONFIG_BOARD_NM701))
int i=100;
		GPIOSetPinDirection(SENSOR_RESET,GPIO_OUT);
		GPIOSetPinLevel(SENSOR_RESET,GPIO_HIGH);	//  height reset
//		mdelay(500);
		while(i--);
		GPIOSetPinLevel(SENSOR_RESET,GPIO_LOW);	//
#endif    
    
	return 0;
}

static void bus_switch_release(struct mt9m112 *mt9m112)
{

}

static int mt9m112_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);
	int ret;

	return 0;
}

static unsigned long mt9m112_query_bus_param(struct soc_camera_device *icd)
{
	/* 0v9650 has all capture_format parameters fixed */
	return SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_HIGH|
		SOCAM_SENSOR_UYVY;
}

static int mt9m112_set_fmt_cap(struct soc_camera_device *icd,
		__u32 pixfmt, struct v4l2_rect *rect)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);
	int ret;
    int i;
    u16 *mt9m112_win;
    unsigned int mt9m112_win_data_len;
	u16 addr = 0, value = 0;
    u8  data8[3], checkid[2];

    if ((rect->width == icd->width) && (rect->height == icd->height)) { 
        printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__,rect->width,rect->height);
        return 0;
    }

    if ((rect->width <= 320) && (rect->height <= 256)) {
        mt9m112_win = mt9m112_qvga;
        mt9m112_win_data_len = sizeof(mt9m112_qvga);
    } else if ((rect->width <= 352) && (rect->height <= 288)) {
        mt9m112_win = mt9m112_cif;
        mt9m112_win_data_len = sizeof(mt9m112_cif);
    } else if ((rect->width <= 640) && (rect->height <= 512)){
        mt9m112_win = mt9m112_vga;
        mt9m112_win_data_len = sizeof(mt9m112_vga);
    } else {
        mt9m112_win = mt9m112_sxga;
        mt9m112_win_data_len = sizeof(mt9m112_sxga);
    }
    
    for (i = 0; i < mt9m112_win_data_len / 4; i++) {
		
		addr  = *(mt9m112_win + 2*i);
    	value = *(mt9m112_win + 2*i+1);
printk("value = %d",value);
    	data8[0]= (u8)addr ;
    	data8[1]= (u8)((value&0xff00)>>8); 	//get the high reg data
    	data8[2]= (u8)(value&0xff); 		//get the low reg data
    	
        ret = mt9m112_tx_data(mt9m112->client, data8, 3);
		if(i<10)
			printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, data8[0],data8[1],data8[2]);
        if (0 > ret) {
            printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__, i);
            return ret;
		}
		if(i<10){
    	checkid[0] = data8[0];
		//checkid[1] = 0x00;
	    mt9m112_rx_data(mt9m112->client, &checkid[0], 2);
    	printk("\n%s..%s..%d    ******** nzy *********0x%x--0x%x--0x%x\n",__FUNCTION__,__FILE__,__LINE__, data8[0], checkid[0],checkid[1]);}
		
    }
    
    //printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);

	return 0;
}

static int mt9m112_try_fmt_cap(struct soc_camera_device *icd,
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

static int mt9m112_get_chip_id(struct soc_camera_device *icd,
			       struct v4l2_chip_ident *id)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);

	if (id->match_type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match_chip != mt9m112->client->addr)
		return -ENODEV;

	id->ident	= mt9m112->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9m112_get_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != mt9m112->client->addr)
		return -ENODEV;

    char reg = reg->reg;
    int ret = mt9m112_rx_data(mt9m112->client, &reg, 1);
    if (!ret)
        reg->val = reg;

	return ret;
}

static int mt9m112_set_register(struct soc_camera_device *icd,
				struct v4l2_register *reg)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);

	if (reg->match_type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match_chip != mt9m112->client->addr)
		return -ENODEV;

    char reg[2];
    reg[0] = reg->reg;
    reg[1] = reg->val;
    int ret = mt9m112_tx_data(mt9m112->client, reg, 2);

	return ret;
}
#endif

static const struct v4l2_queryctrl mt9m112_controls[] = {
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

static int mt9m112_get_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		break;
	}
	return 0;
}

static struct soc_camera_ops mt9m112_ops = {
	.owner			= THIS_MODULE,
	.probe			= mt9m112_video_probe,
	.remove			= mt9m112_video_remove,
	.init			= mt9m112_init,
	.release		= mt9m112_release,
	.start_capture		= mt9m112_start_capture,
	.stop_capture		= mt9m112_stop_capture,
	.set_fmt_cap		= mt9m112_set_fmt_cap,
	.try_fmt_cap		= mt9m112_try_fmt_cap,
	.set_bus_param		= mt9m112_set_bus_param,
	.query_bus_param	= mt9m112_query_bus_param,
	.controls		= mt9m112_controls,
	.num_controls		= ARRAY_SIZE(mt9m112_controls),
	.get_control		= mt9m112_get_control,
	.set_control		= mt9m112_set_control,
	.get_chip_id		= mt9m112_get_chip_id,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register		= mt9m112_get_register,
	.set_register		= mt9m112_set_register,
#endif
};

static int mt9m112_set_control(struct soc_camera_device *icd, struct v4l2_control *ctrl)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);
	const struct v4l2_queryctrl *qctrl;
	int data;

	qctrl = soc_camera_find_qctrl(&mt9m112_ops, ctrl->id);

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
static int mt9m112_video_probe(struct soc_camera_device *icd)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);
	s32 data;
	int ret;

	/* We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant. */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

    mt9m112->model = V4L2_IDENT_MT9M112;
    icd->formats = &mt9m112_colour_formats;
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

static void mt9m112_video_remove(struct soc_camera_device *icd)
{
	struct mt9m112 *mt9m112 = container_of(icd, struct mt9m112, icd);

	dev_dbg(&icd->dev, "Video %x removed: %p, %p\n", mt9m112->client->addr,
		mt9m112->icd.dev.parent, mt9m112->icd.vdev);
	soc_camera_video_stop(&mt9m112->icd);
}

static int mt9m112_remove(struct i2c_client *client)
{
	struct mt9m112 *mt9m112 = i2c_get_clientdata(client);

	soc_camera_device_unregister(&mt9m112->icd);
	kfree(mt9m112);

	return 0;
}

static int mt9m112_detach_client(struct i2c_client *client)
{
	mt9m112_remove(client);

	return i2c_detach_client(client);
}

static int mt9m112_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data_mt9m112, mt9m112_probe);
}

static struct i2c_driver mt9m112_driver = {
	.driver = {
		.name = "mt9m112",
	    },
	.id 	= MT9M112_IIC_ADDR,
	.attach_adapter = &mt9m112_attach_adapter,
	.detach_client  = &mt9m112_detach_client,
};

static struct soc_camera_link iclink = {//nzy add
    .bus_id = 33, /* Must match with the camera ID above */
    .gpio   = 1,
};
    
static int mt9m112_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct mt9m112 *mt9m112;
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

	strlcpy(client->name, "mt9m112", I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adapter;
	client->driver = &mt9m112_driver;
	client->addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 100;	
	ret = i2c_attach_client(client);
	if (ret) {
        goto exit_i2c_attach_client_failed;
	}
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	
	mt9m112 = kzalloc(sizeof(struct mt9m112), GFP_KERNEL);
	if (!mt9m112) {
		ret = -ENOMEM;
        goto exit_alloc_data_failed;
	}

	mt9m112->client = client;
	i2c_set_clientdata(client, mt9m112);
    //icl = &iclink;//client->dev.platform_data;
    
	/* Second stage probe - when a capture adapter is there */
	icd = &mt9m112->icd;
	icd->ops	= &mt9m112_ops;
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
	mt9m112->datawidth = 8;
	/* Simulated autoexposure. If enabled, we calculate shutter width
	 * ourselves in the driver based on vertical blanking and frame width */
	mt9m112->autoexposure = 1;

	ret = bus_switch_request(mt9m112, icl);
	if (ret)
		goto exit_bus_switch_request_failed;
		
	ret = soc_camera_device_register(icd);
	if (ret)
        goto exit_soc_camera_device_register_failed;
                
    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);
	return 0;

exit_soc_camera_device_register_failed:
    bus_switch_release(mt9m112);
exit_bus_switch_request_failed:
	kfree(mt9m112);
exit_alloc_data_failed:
   i2c_detach_client(client);
exit_i2c_attach_client_failed:
	kfree(client);
exit_alloc_client_failed:
exit_check_functionality_failed:
	return ret;
}

static int __init mt9m112_mod_init(void)
{
	return i2c_add_driver(&mt9m112_driver);
}

static void __exit mt9m112_mod_exit(void)
{
	i2c_del_driver(&mt9m112_driver);
}

module_init(mt9m112_mod_init);
module_exit(mt9m112_mod_exit);

MODULE_DESCRIPTION("Micron mt9m112 Camera driver");
MODULE_AUTHOR("tb <kernel@rock-chips>");
MODULE_LICENSE("GPL");
