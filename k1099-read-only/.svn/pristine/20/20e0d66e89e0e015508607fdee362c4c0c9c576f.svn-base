/*******************************************************************/
/*	  Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.			  */
/*******************************************************************
File		:	 I2C-ROCKSOFT.c
Desc		:	 the driver of hym8563
Author		:	 some one
Date		:	 2009-5-20
Notes		:
$Log: i2c.c,v $
Revision 0.00
********************************************************************/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>



//#define DBG_I2CSOFTMODE  
//zhaojun add for debug
#ifdef DBG_I2CSOFTMODE
	#define dgbI2C  printk
#else
	#define dgbI2C  if(0)
#endif


struct rock_i2c_data {
	struct resource		*ioarea;
	void __iomem		*reg;
	struct i2c_adapter	 adap;
	struct i2c_algo_bit_data bit;
};





#define I2C_SCL_PIN 	(GPIOPortE_Pin5)
#define I2C_SDA_PIN  	(GPIOPortE_Pin4)


#define CMD_SET_SDA	(1<<2)
#define CMD_SET_SCL	(1<<3)

#define STATE_SDA	(1<<0)
#define STATE_SCL	(1<<1)

/* i2c bit-bus functions */

static void rock_i2c_setsda(void *pw, int state)
{
	struct rock_i2c_data *pd = pw;


	dgbI2C("\n %s  ----i2c_setsda! state=%d",__FILE__,state);

//	gpio_direction_output(I2C_SDA_PIN,0);
	GPIOSetPinDirection(I2C_SDA_PIN,  GPIO_OUT);

	
	
	GPIOSetPinLevel((eGPIOPinNum_t)I2C_SDA_PIN,(state ?GPIO_HIGH:GPIO_LOW));
	
	//__gpio_set(,state);

	//writeb(CMD_SET_SDA | (state ? STATE_SDA : 0), pd->reg);
}

static void rock_i2c_setscl(void *pw, int state)
{
	struct rock_i2c_data *pd = pw;

	dgbI2C("\n %s  ----i2c_setscl! state=%d",__FILE__,state);

	//gpio_direction_output(I2C_SCL_PIN,0);
	GPIOSetPinDirection(I2C_SCL_PIN,  GPIO_OUT);

	
	GPIOSetPinLevel((eGPIOPinNum_t)I2C_SCL_PIN,(state ?GPIO_HIGH:GPIO_LOW));
	
	//__gpio_set(I2C_SCL_PIN,state);
	//writeb(CMD_SET_SCL | (state ? STATE_SCL : 0), pd->reg);
}

static int rock_i2c_getsda(void *pw)
{
	struct rock_i2c_data *pd = pw;

	dgbI2C("\n %s  ---- i2c_getsda! ",__FILE__);


	GPIOSetPinDirection(I2C_SDA_PIN,  GPIO_IN);

//	gpio_direction_input(I2C_SDA_PIN);

	
	return GPIOGetPinLevel((eGPIOPinNum_t)I2C_SDA_PIN);
	
	//return __gpio_get(I2C_SDA_PIN);
	
	//return readb(pd->reg) & STATE_SDA ? 1 : 0;
}

static int rock_i2c_getscl(void *pw)
{
	struct rock_i2c_data *pd = pw;

	dgbI2C("\n %s  ----i2c_getscl! ",__FILE__);

	//gpio_direction_input(I2C_SCL_PIN);
	GPIOSetPinDirection(I2C_SCL_PIN,  GPIO_IN);

	return GPIOGetPinLevel((eGPIOPinNum_t)I2C_SCL_PIN);
	
	//return __gpio_get(I2C_SCL_PIN);

	
	//return readb(pd->reg) & STATE_SCL ? 1 : 0;
}

/* device registration */
//extern 	void IOMUXSetI2C0_gpio( void);
static int rock_i2c_probe(struct platform_device *dev)
{
	struct rock_i2c_data *pd;
	struct resource *res;
	int size;
	int ret;

	dgbI2C("\n %s  -------- rock_i2c_probe! ",__FILE__);

	//rockchip_mux_api_set(GPIOE_I2C0_SEL_NAME, IOMUXA_GPIO1_A45);
	//IOMUXSetI2C0_gpio();


	pd = kzalloc(sizeof(struct rock_i2c_data), GFP_KERNEL);
	if (pd == NULL) {
		dev_err(&dev->dev, "cannot allocate private data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(dev, pd);

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&dev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err;
	}

	size = (res->end-res->start)+1;

	pd->ioarea = request_mem_region(res->start, size, dev->name);
	if (pd->ioarea == NULL) {
		dev_err(&dev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err;
	}

	pd->reg = ioremap(res->start, size);
	if (pd->reg == NULL) {
		dev_err(&dev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_res;
	}

	/* setup the private data */

	pd->adap.owner = THIS_MODULE;
	pd->adap.algo_data = &pd->bit;
	pd->adap.dev.parent = &dev->dev;

	strlcpy(pd->adap.name, "rock I2C", sizeof(pd->adap.name));

	pd->bit.data = pd;
	pd->bit.setsda = rock_i2c_setsda;
	pd->bit.setscl = rock_i2c_setscl;
	pd->bit.getsda = rock_i2c_getsda;
	pd->bit.getscl = rock_i2c_getscl;
	pd->bit.timeout = HZ/100;
	pd->bit.udelay = 500*1000;//20;

	ret = i2c_bit_add_bus(&pd->adap);
	if (ret)
		goto err_all;

	return 0;

 err_all:
	iounmap(pd->reg);

 err_res:
	release_resource(pd->ioarea);
	kfree(pd->ioarea);

 err:
	kfree(pd);
	return ret;
}

static int rock_i2c_remove(struct platform_device *dev)
{
	struct rock_i2c_data *pd = platform_get_drvdata(dev);

	i2c_del_adapter(&pd->adap);

	iounmap(pd->reg);
	release_resource(pd->ioarea);
	kfree(pd->ioarea);
	kfree(pd);

	//rockchip_mux_api_set(GPIOE_I2C0_SEL_NAME, IOMUXA_GPIO1_A45);

	return 0;
}


/* device driver */

static struct platform_driver rock_i2c_driver = {
	.driver		= {
		.name		= "rock-i2csoft",
		.owner		= THIS_MODULE,
	},
	.probe		= rock_i2c_probe,
	.remove		= rock_i2c_remove,
};

static int __init i2c_adap_rock_init(void)
{
	return platform_driver_register(&rock_i2c_driver);
}

static void __exit i2c_adap_rock_exit(void)
{
	platform_driver_unregister(&rock_i2c_driver);
}

module_init(i2c_adap_rock_init);
module_exit(i2c_adap_rock_exit);

MODULE_DESCRIPTION("rock Generic I2C Bus driver");
MODULE_AUTHOR("Ben Dooks <ben@rock.co.uk>");
MODULE_LICENSE("GPL");
