/*
 * rk1000.c 
 *
 * Driver for rockchip rk1000 control
 *  Copyright (C) 2009 lhh
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/arch/gpio.h>

//#define DEBUG

#ifdef DEBUG
#define D(fmt, arg...) printk("<7>%s:%d: " fmt, __FILE__, __LINE__, ##arg)
#else
#define D(fmt, arg...)
#endif
#define E(fmt, arg...) printk("<3>!!!%s:%d: " fmt, __FILE__, __LINE__, ##arg)

#define  ADC_CON             0
#define  CODEC_CON           1
#define  I2C_CON             2
#define  TVE_CON             3
#define  SRESET              4
#define  ADC_STAT            7
#define  RK1000_RESET_PIN    GPIOPortF_Pin4


#define DRV_NAME "RK1000_CONTROL"
static const unsigned short normal_i2c[] = {
	0x80 >> 1,			/* rk1000 control address */
	I2C_CLIENT_END
};

I2C_CLIENT_INSMOD;			/* defines addr_data */

struct i2c_client *rk1000_control_i2c_client = NULL;

static int rk1000_control_probe(struct i2c_adapter *adapter, int addr, int kind);

int rk1000_control_set_reg(struct i2c_client *client, u8 reg, u8 const buf[], u8 len)
{
    int ret;
	u8 i2c_buf[8];
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};

	D("reg = 0x%.2X,value = 0x%.2X\n", reg, buf[0]);
	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}

int rk1000_control_write_block(u8 addr, u8 *buf, u8 len)
{
	int i;
	int ret = 0;
	
	if(rk1000_control_i2c_client == NULL){
		E("rk1000_tv_i2c_client not init!\n");
	}

	for(i=0; i<len; i++){
		ret = rk1000_control_set_reg(rk1000_control_i2c_client, addr+i, buf+i, 1);
		if(ret != 0){
			E("rk1000_control_set_reg err, addr=0x%.x, val=0x%.x", addr+i, buf[i]);
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(rk1000_control_write_block);

static int rk1000_control_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, rk1000_control_probe);
}

static int rk1000_control_detach_client(struct i2c_client *client)
{
	return i2c_detach_client(client);
}

static struct i2c_driver rk1000_control_driver = {
	.driver 	= {
		.name	= DRV_NAME,
	},
	.id 	= I2C_DRIVERID_RK1000_CONTROL,
	.attach_adapter = rk1000_control_attach_adapter,
	.detach_client	= rk1000_control_detach_client,
};

static int rk1000_control_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int rc = 0;
	u8 buff;
	struct i2c_client *client = NULL;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		rc = -ENODEV;
		goto failout;
	}
	
	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == NULL) {
		rc = -ENOMEM;
		goto failout;
	}

	client->addr = addr;
	client->adapter = adapter;
	client->driver = &rk1000_control_driver;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH0;
	client->speed = 150;
	client->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	strlcpy(client->name, DRV_NAME, I2C_NAME_SIZE);

	rc = i2c_attach_client(client);
	if (rc < 0)
		goto failout;

	rk1000_control_i2c_client = client;
    //reset
    mdelay(10);  //用于等待前面的I2S时钟稳定
   // GPIOSetPinDirection(RK1000_RESET_PIN,GPIO_OUT);
	//GPIOSetPinLevel(RK1000_RESET_PIN,GPIO_HIGH);  
    //mdelay(10);
    //GPIOSetPinLevel(RK1000_RESET_PIN, GPIO_HIGH);
    //mdelay(10);
    buff = 0x00;
    rk1000_control_set_reg(client, TVE_CON, &buff, 1);
    
    //set I2C glitch filter
    buff = 0x22;
    rk1000_control_set_reg(client,I2C_CON,&buff,1);
    //disable no use block
    #if  !(CONFIG_SND_SOC_RK1000)
    buff = 0x0D;
    rk1000_control_set_reg(client,CODEC_CON,&buff,1);
    #endif
	
	#ifdef CONFIG_ANX7150 //add for hmdi, zyy 2010.2.25
    buff = 0x04;	//CODEC ADC LRCK output disable
    rk1000_control_set_reg(client,CODEC_CON,&buff,1);
	#endif

	
    //目前RK1000的HS-ADC暂时还没用，所以把HS-ADC给关闭掉，以节省功耗，以后要使用得开启
    buff = 0x88;
    rk1000_control_set_reg(client,ADC_CON,&buff,1);
	return 0;

failout:
	kfree(client);
	return rc;
}

static int __init rk1000_init(void)
{    
    int tmp;
    tmp=i2c_add_driver(&rk1000_control_driver);
    D("tmp = 0x%x\n", tmp);
    return 0;
}

static void __exit rk1000_exit(void)
{
    i2c_del_driver(&rk1000_control_driver);
}

module_init(rk1000_init);
module_exit(rk1000_exit);
/* Module information */
MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("ROCKCHIP IIS ASoC Interface");
MODULE_LICENSE("GPL");

