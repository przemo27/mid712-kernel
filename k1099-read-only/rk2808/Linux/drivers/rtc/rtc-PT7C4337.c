/****************************************************************************************
 * driver/rtc/rtc-PT7C4337.c
 *Copyright 	:ROCKCHIP  Inc
 *Author		:	lhh
 *Date		: 2009-08-10
 *This driver use for rk28 chip extern rtc. Use i2c1 ,the chip is PT7C4337
 ********************************************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <asm/arch/rk28_macro.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif


#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif



#define DRV_NAME "rtc_PT7C4337"

#define   PT7C4337_ADDR		0xD0
#define   RTC_SEC		0x00
#define   RTC_MIN		0x01
#define   RTC_HOUR		0x02
#define   RTC_WEEK		0x03
#define   RTC_DAY		0x04
#define   RTC_MON		0x05
#define   RTC_YEAR		0x06
#define   RTC_A1_SEC	0x07
#define   RTC_A1_MIN	0x08
#define   RTC_A1_HOUR	0x09
#define   RTC_A1_DAY	0x0A
#define   RTC_A2_MIN	0x0B
#define   RTC_A2_HOUR	0x0C
#define   RTC_A2_DAY	0x0D
#define   RTC_CTR		0x0E
#define   RTC_STA		0x0F
#define   CENTURY	  0x80
#define   E_TIME 	  0x80
#define   INTCN 	  0x04
#define   A2IE		  0x02
#define   A1IE		  0x01
#define   A2F		  0x02
#define   A1F		  0x01
#define   OSF		  0x80

static const unsigned short normal_i2c[] = {
	PT7C4337_ADDR >> 1,			/* HYM8563 address */
	I2C_CLIENT_END
};

I2C_CLIENT_INSMOD;			/* defines addr_data */

struct i2c_client *g_client;

static int PT7C4337_probe(struct i2c_adapter *adapter, int addr, int kind);

static int PT7C4337_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{

	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 200;
	buf[0] = reg;
	ret = i2c_transfer(client->adapter, msgs, 1);
	DBG("\n*************PT7C4337_i2c_read_regs:reg=%d,value=%d\n",reg,buf[0]);
	if (ret > 0)
		ret = 0;
	return ret;
}
static int PT7C4337_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len)
{

	int ret;
	u8 i2c_buf[8];
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 200;
	DBG("\n*************PT7C4337_i2c_set_regs:reg=%d,value=%d\n",reg,buf[0]);
	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	return ret;
}
static int PT7C4337_i2c_read_time(struct i2c_client *client, struct rtc_time *tm)
{
	u8 regs[7] = { 0, };
	int ret;
	ret = PT7C4337_i2c_read_regs(client, RTC_SEC, regs, 7);
	tm->tm_year = 70 + (regs[6] & 0x0f) + (regs[6]>>4) * 10;
   	tm->tm_mon = (regs[5] & 0x0f) + ((regs[5]>>4) & 0x01) * 10;
   	tm->tm_mday = (regs[4] & 0x0f) + (regs[4]>>4) * 10;
   	tm->tm_wday = regs[3];
   	tm->tm_hour = (regs[2] & 0x0f) + ((regs[2]>>4) & 0x03) * 10;
   	tm->tm_min = (regs[1] & 0x0f) + (regs[1]>>4) * 10;
   	tm->tm_sec= (regs[0] & 0x0f) + (regs[0]>>4) * 10;
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	DBG("\n-----------rtc_sec=%d",tm->tm_sec);
	DBG("\n-----------rtc_min=%d",tm->tm_min);
	DBG("\n-----------rtc_hour=%d",tm->tm_hour);
	DBG("\n-----------rtc_wday=%d",tm->tm_wday);
	DBG("\n-----------rtc_mday=%d",tm->tm_mday);
	DBG("\n-----------rtc_mon=%d",tm->tm_mon);
	DBG("\n-----------rtc_year=%d",tm->tm_year);
	DBG("\n-----------rtc_yday=%d",tm->tm_yday);
	DBG("\n-----------rtc_isdst=%d\n",tm->tm_isdst);
	return ret;
}

static int PT7C4337_i2c_set_time(struct i2c_client *client, struct rtc_time  *tm)	
{
	u8 regs[7] = { 0, };
	int ret = 0;

	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	DBG("\n-----------rtc_sec=%d",tm->tm_sec);
	DBG("\n-----------rtc_min=%d",tm->tm_min);
	DBG("\n-----------rtc_hour=%d\n",tm->tm_hour);
	
	DBG("\n-----------rtc_wday=%d\n",tm->tm_wday);
	
	DBG("\n-----------rtc_mday=%d",tm->tm_mday);
	DBG("\n-----------rtc_mon=%d",tm->tm_mon);
	DBG("\n-----------rtc_year=%d\n",tm->tm_year);
	
	DBG("\n-----------rtc_yday=%d",tm->tm_yday);
	DBG("\n-----------rtc_isdst=%d\n",tm->tm_isdst);
	 tm->tm_year -= 70;
	 regs[6] = ((((tm->tm_year) / 10) %10) << 4) + ((tm->tm_year) % 10);
	 regs[5] = (((tm->tm_mon) / 10) << 4) + ((tm->tm_mon) % 10);
	 regs[4] = (((tm->tm_mday) / 10) << 4) + ((tm->tm_mday) % 10);
	 regs[3] = tm->tm_wday;
	 regs[2] = (((tm->tm_hour) / 10) << 4) + ((tm->tm_hour) % 10);
	 regs[1] = (((tm->tm_min) / 10) << 4) + ((tm->tm_min) % 10);
	 regs[0] = (((tm->tm_sec) / 10) << 4) + ((tm->tm_sec) % 10);


	ret = PT7C4337_i2c_set_regs(client, RTC_SEC, regs, 7);	
	/*enable oscillator and time count chain*/
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
	regs[0]  = regs[0] & (~ E_TIME);
	 PT7C4337_i2c_set_regs(client, RTC_CTR, regs, 1);
	 /*enable oscillator flag*/
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_STA, regs, 1);
	regs[0]  = regs[0] & (~ OSF);
	 PT7C4337_i2c_set_regs(client, RTC_STA, regs, 1);
	 
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
	DBG("\n----------- RTC_CTR=0x%x\n",regs[0]);
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_STA, regs, 1);
	DBG("\n----------- RTC_STA=0x%x\n",regs[0]);

	return ret;
}



/*the init of the PT7C4337 at first time */
static int PT7C4337_init_device(struct i2c_client *client)	
{
	u8 regs[2];
	struct rtc_time tm_read_default = {
	.tm_year = 107,
	.tm_mon = 11,
	.tm_mday = 20,
	.tm_hour = 12, 
	.tm_min = 0,
	.tm_sec = 0,
	.tm_wday = 2,
	};	
	struct rtc_time tm_read;
/*clear alarm */
	regs[0]=OSF & (~A1F);
	PT7C4337_i2c_set_regs(client, RTC_STA, regs, 1);	
/*read time from PT7C4337*/
 	PT7C4337_i2c_read_time(client, &tm_read); 	
	if(tm_read.tm_year > 138 || tm_read.tm_mon > 12 || tm_read.tm_mday > 31  || tm_read.tm_hour > 23  || tm_read.tm_min > 59)	
		PT7C4337_i2c_set_time(client,&tm_read_default);
/*set frqency  00=1k ; 01=4.096k;  10=8.192;  11=32.768 */	
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client,RTC_CTR,regs,1);
	regs[0] = (regs[0] & 0xe7)|( 0x11 << 3 );
	PT7C4337_i2c_set_regs(client, RTC_CTR, regs, 1);
/*Enable interrupt related bits*/
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
	regs[0] = (regs[0] & (~INTCN)) | INTCN ;
	PT7C4337_i2c_set_regs(client, RTC_CTR, regs, 1);
	/*enable oscillator and time count chain*/
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
	regs[0]  = regs[0] & (~ E_TIME);
	 PT7C4337_i2c_set_regs(client, RTC_CTR, regs, 1);
	 /*enable oscillator flag*/
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_STA, regs, 1);
	regs[0]  = regs[0] & (~ OSF);
	 PT7C4337_i2c_set_regs(client, RTC_STA, regs, 1);
	 
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_CTR, regs, 1);
	DBG("\n----------- RTC_CTR=0x%x\n",regs[0]);
	regs[0] = 0;
	PT7C4337_i2c_read_regs(client, RTC_STA, regs, 1);
	DBG("\n----------- RTC_STA=0x%x\n",regs[0]);
	return 0;
}


static int PT7C4337_i2c_validate_client(struct i2c_client *client)
{
	return 0;
}

static int PT7C4337_i2c_read_alarm(struct i2c_client *client, struct rtc_wkalrm *tm)
{
#if 0
	int sr;
	u8 regs[4] = { 0, };

	sr = PT7C4337_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	if (sr < 0) {
		dev_err(&client->dev, "%s: reading RTC section failed\n",
			__func__);
		return sr;
	}
	if(regs[0x00] & 0x80)
	{
		tm->time.tm_min = BCD2BIN(regs[0x00] & 0x7F);
	}
	else
	{
		tm->time.tm_min = 0;
	}
	if(regs[0x01] & 0x80)
	{
		tm->time.tm_hour = BCD2BIN(regs[0x01] & 0x7F);
	}
	else
	{
		tm->time.tm_hour = 0;
	}
	if(regs[0x02] & 0x80)
	{
		tm->time.tm_mday = BCD2BIN(regs[0x02] & 0x3F);
	}
	else
	{
		tm->time.tm_mday = 0;
	}
	if(regs[0x03] & 0x80)
	{
		tm->time.tm_wday = BCD2BIN(regs[0x03] & 0x3F);
	}
	else
	{
		tm->time.tm_wday = 0;
	}
	tm->time.tm_mon = 0; 
	tm->time.tm_year = 0;	
	tm->time.tm_yday = 0;
	tm->time.tm_isdst = 0;
#endif
	return 0;

}

static int PT7C4337_i2c_set_alarm(struct i2c_client *client, struct rtc_wkalrm const *tm)
{	
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);

#if 0
	u8 regs[4] = { 0, };
	u8 mon_day ;
	
		
	regs[0x00] = BIN2BCD(tm->time.tm_min) | 0x80;
	regs[0x01] = BIN2BCD(tm->time.tm_hour) | 0x80;

	if(tm->time.tm_mday)		//if the tm_mday is not NULL
	{
		if(tm->time.tm_year)	//if the tm_year is not NULL
			mon_day = rtc_month_days((tm->time.tm_mon), tm->time.tm_year + 1900);
		else
			mon_day = rtc_month_days((tm->time.tm_mon), 2009);
		
		if((tm->time.tm_mday) > mon_day)		//if the input month is bigger than the right month day, set the biggest num of that month 
			regs[0x02] = BIN2BCD(mon_day) | 0x80;
		else
			regs[0x02] = BIN2BCD(tm->time.tm_mday) | 0x80;
	}
	else
	{
		regs[0x02] &= ~0x80;
	}
	if(tm->time.tm_wday)
	{
		regs[0x02] = BIN2BCD(tm->time.tm_wday) | 0x80;
	}
	else
	{
		regs[0x02] &= ~0x80;
	}

	PT7C4337_i2c_set_regs(client, RTC_A_MIN, regs, 4);
#endif
	return 0;
}

static int PT7C4337_i2c_open_alarm(struct i2c_client *client)
{
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);

#if 0
	u8 data;
	PT7C4337_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data |= AIE;
	PT7C4337_i2c_set_regs(client, RTC_CTL2, &data, 1);
#endif
	return 0;
}

static int PT7C4337_i2c_close_alarm(struct i2c_client *client)
{
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);

#if 0
	u8 data;
	PT7C4337_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data &= ~AIE;
	PT7C4337_i2c_set_regs(client, RTC_CTL2, &data, 1);
#endif
	return 0;
}


static int PT7C4337_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int tmp;
	tmp=PT7C4337_i2c_read_time(to_i2c_client(dev), tm);
	return tmp;
}

static int PT7C4337_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return PT7C4337_i2c_set_time(to_i2c_client(dev), tm);
}

static int PT7C4337_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	return PT7C4337_i2c_read_alarm(to_i2c_client(dev), tm);
}

static int PT7C4337_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	return PT7C4337_i2c_set_alarm(to_i2c_client(dev), tm);
}

#if defined(CONFIG_RTC_INTF_DEV) || defined(CONFIG_RTC_INTF_DEV_MODULE)
static int PT7C4337_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	switch (cmd) {
	case RTC_AIE_OFF:
		if(PT7C4337_i2c_close_alarm(client) < 0)
			goto err;
		break;
	case RTC_AIE_ON:
		if(PT7C4337_i2c_open_alarm(client))
			goto err;
		break;
	default:
		return -ENOIOCTLCMD;
	}	
	return 0;
err:
	return -EIO;
}
#else
#define PT7C4337_rtc_ioctl NULL
#endif

#if defined(CONFIG_RTC_INTF_PROC) || defined(CONFIG_RTC_INTF_PROC_MODULE)
static int PT7C4337_rtc_proc(struct device *dev, struct seq_file *seq)
{
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	return 0;
}
#else
#define PT7C4337_rtc_proc NULL
#endif




static int PT7C4337_attach_adapter(struct i2c_adapter *adapter)
{

	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);

//	adapter->deviceaddr=(PT7C4337_ADDR);
//	adapter->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
//	adapter->mode=0;

	return i2c_probe(adapter, &addr_data, PT7C4337_probe);
}

static int PT7C4337_detach_client(struct i2c_client *client)
{
	struct rtc_device *const rtc = i2c_get_clientdata(client);
	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
	if (rtc)
		rtc_device_unregister(rtc);

	return i2c_detach_client(client);
}

static struct i2c_driver PT7C4337_driver = {
	.driver 	= {
		.name	= DRV_NAME,
	},
	.id 	= I2C_DRIVERID_PT7C4337,
	.attach_adapter = PT7C4337_attach_adapter,
	.detach_client	= PT7C4337_detach_client,
};

static const struct rtc_class_ops PT7C4337_rtc_ops = {
	.read_time	= PT7C4337_rtc_read_time,
	.set_time		= PT7C4337_rtc_set_time,
	.read_alarm 	= PT7C4337_rtc_read_alarm,
	.set_alarm	= PT7C4337_rtc_set_alarm,
	.ioctl 		= PT7C4337_rtc_ioctl,
	.proc		= PT7C4337_rtc_proc
};

static int PT7C4337_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int rc = 0;
	struct i2c_client *client = NULL;
	struct rtc_device *rtc = NULL;
	
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
	client->driver = &PT7C4337_driver;
	client->mode = NORMALMODE;
	client->Channel = I2C_CH1;
	client->speed = 150;
	client->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	strlcpy(client->name, DRV_NAME, I2C_NAME_SIZE);
	if (kind < 0) {
		rc = PT7C4337_i2c_validate_client(client);
		if (rc < 0)
			goto failout;
	}
	rc = i2c_attach_client(client);
	if (rc < 0)
		goto failout;
/*rtc :first member is dev*/
	rtc = rtc_device_register(PT7C4337_driver.driver.name, &client->dev, &PT7C4337_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rc = PTR_ERR(rtc);
		goto failout_detach;

		DBG("-----------rtc device register fail!");
	}
	DBG("-----------rtc device register sucess !\n");
	i2c_set_clientdata(client, rtc);
	PT7C4337_init_device(client);
	g_client = client;
	return 0;

failout_detach:
	i2c_detach_client(client);
failout:
	kfree(client);
	return rc;
}

static int __init PT7C4337_init(void)
{
	int tmp;
	
	tmp=i2c_add_driver(&PT7C4337_driver);

	DBG("#####----->%s---->%d\n",__FUNCTION__,__LINE__);
return 0;
}

static void __exit PT7C4337_exit(void)
{
	i2c_del_driver(&PT7C4337_driver);
}

MODULE_DESCRIPTION("Maxim MAX6900 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(PT7C4337_init);
module_exit(PT7C4337_exit);

