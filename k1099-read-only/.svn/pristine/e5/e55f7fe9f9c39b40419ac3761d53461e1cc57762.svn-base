


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <asm/arch/rk28_macro.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>


#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif



#define DRV_NAME "rtc_HYM8563"


#define   HYM_ADDR		0xA2
#define   RTC_SPEED 		200
		
#define   RTC_CTL1		0x00
#define   RTC_CTL2		0x01
#define   RTC_SEC		0x02
#define   RTC_MIN		0x03
#define   RTC_HOUR		0x04
#define   RTC_DAY		0x05
#define   RTC_WEEK		0x06
#define   RTC_MON		0x07
#define   RTC_YEAR		0x08
#define   RTC_A_MIN 		0x09
#define   RTC_A_HOUR	0x0A
#define   RTC_A_DAY 		0x0B
#define   RTC_A_WEEK	0x0C
#define   RTC_CLKOUT	0x0D
#define   RTC_T_CTL 	0x0E
#define   RTC_T_COUNT	0x0F
#define   CENTURY	0x80
#define   TI		0x10
#define   AF		0x08
#define   TF		0x04
#define   AIE		0x02
#define   TIE		0x01
#define   FE		0x80
#define   TE		0x80
#define   FD1		0x02
#define   FD0		0x01
#define   TD1		0x02
#define   TD0		0x01
#define   VL		0x80

#define HYM8563_REG_LEN 	0x10
#define HYM8563_RTC_SECTION_LEN	0x07
static const unsigned short normal_i2c[] = {
	HYM_ADDR >> 1,			/* HYM8563 address */
	I2C_CLIENT_END
};

I2C_CLIENT_INSMOD;			/* defines addr_data */


struct i2c_client *rtc_client;


static int hym8563_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len);
static int hym8563_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len);
static int hym8563_probe(struct i2c_adapter *adapter, int addr, int kind);
static int hym8563_i2c_read_alarm(struct i2c_client *client, struct rtc_wkalrm *tm);
static int hym8563_i2c_read_time(struct i2c_client *client, struct rtc_time *tm);


static ssize_t rtc_attr_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 regs[4] = { 0, };
	struct rtc_time tm = {
	.tm_wday = 4,
	.tm_year = 109,
	.tm_mon = 9,
	.tm_mday = 1,
	.tm_hour = 12, 
	.tm_min = 10,
	.tm_sec = 58
	};
	hym8563_i2c_read_time(client, &tm);
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	printk("\n---hym8563_i2c_set_alarm-->r_min=%d r_hour=%d r_mday=%d r_wday=%d!\n",BCD2BIN(regs[0]& 0x7f),BCD2BIN(regs[1]& 0x7f),BCD2BIN(regs[2]& 0x7f),BCD2BIN(regs[3]& 0x7f));
	return sprintf(buf, "HYM8563_%s\n", __FUNCTION__);
}

static DEVICE_ATTR(rtc_attr, 0444, rtc_attr_show, NULL);

static struct attribute *hym8563_attributes[] =
{
	&dev_attr_rtc_attr.attr,
	NULL
};
static const struct attribute_group hym8563_group = {
	.attrs = hym8563_attributes,
};




/*the init of the hym8563 at first time */
static int hym8563_init_device(struct i2c_client *client)	
{
	u8 regs[2];
	int sr;

	regs[0]=0;
	hym8563_i2c_set_regs(client, RTC_CTL1, regs, 1);		
	
	
	//disable clkout
	regs[0] = 0x80;
	hym8563_i2c_set_regs(client, RTC_CLKOUT, regs, 1);
	/*enable alarm interrupt*/
	hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	DBG("%s--->%d  RTC_CTL2 = %x\n",__FUNCTION__,__LINE__,regs[0]);
	regs[0] = 0x0;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	DBG("%s--->%d  RTC_CTL2 = %x\n",__FUNCTION__,__LINE__,regs[0]);

	//clear /int
	
	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if(sr<0)
	{
		DBG("----hym8563_init err");
	}
	
	if(regs[0] & (AF|TF))
	{
		regs[0] &= ~(AF|TF);
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	}
	

	return 0;
}



/*check a rtc time  in range*/
bool check_time_inrange(struct rtc_time *time)
{
	bool res=0;

	if (time->tm_sec>60)
		return res;
	if (time->tm_min>60)
		return res;
	if (time->tm_hour>24)
		return res;
	if (time->tm_mon>12)
		return res;
	if (time->tm_year>255)
		return res;
	if (time->tm_mday>31)
		return res;

	return 1;

}
/********************************************************************
********************************************************************/



/*
	get the weekday
*/
static u8 rk28_get_weekday(struct rtc_time  *tm)	//after  1582 - 10 - 15 
{
	int year, y, c, m, d, weekday;
	year = tm->tm_year;
	if(tm->tm_mon < 3)
	{
		m = tm->tm_mon + 12;
		year --;
	}
	else
	{
		m = tm->tm_mon;
	}
	y = year % 100;
	c = year / 100;
	d = tm->tm_mday;
	
	weekday = y + y/4 + c/4 - 2*c + 26*(m + 1)/10 + d -1;
	
	weekday = weekday % 7;	
	
	if(weekday < 0)
		weekday += 7;	
	
	return weekday;
}



static int hym8563_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
	buf[0] = reg;
	BUG_ON(len == 0);
	BUG_ON(reg > RTC_T_COUNT);
	BUG_ON(reg + len > RTC_T_COUNT + 1);

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}

static int hym8563_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len)
{
	int ret;
	u8 i2c_buf[HYM8563_REG_LEN + 1];
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};

	DBG("\nhym8563_i2c_set_regs:reg=%d,value=%d",reg,buf[0]);
	BUG_ON(len == 0);
	BUG_ON(reg > RTC_T_COUNT);
	BUG_ON(reg + len > RTC_T_COUNT + 1);

	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}

static int hym8563_i2c_validate_client(struct i2c_client *client)
{
	return 0;
}

static int hym8563_i2c_read_time(struct i2c_client *client, struct rtc_time *tm)
{
	u8 i,regs[HYM8563_RTC_SECTION_LEN] = { 0, };
    
    for(i=0;i<HYM8563_RTC_SECTION_LEN;i++){
		hym8563_i2c_read_regs(client, RTC_SEC+i, &regs[i], 1);
	}
	//hym8563_i2c_read_regs(client, RTC_SEC, regs, HYM8563_RTC_SECTION_LEN);
	
	tm->tm_sec = BCD2BIN(regs[0x00] & 0x7F);
	tm->tm_min = BCD2BIN(regs[0x01] & 0x7F);
	tm->tm_hour = BCD2BIN(regs[0x02] & 0x3F);
	tm->tm_mday = BCD2BIN(regs[0x03] & 0x3F);
	tm->tm_wday = BCD2BIN(regs[0x04] & 0x07);	
	
	tm->tm_mon = BCD2BIN(regs[0x05] & 0x1F) ; 
	tm->tm_mon -= 1;			//inorder to cooperate the systerm time
	
	tm->tm_year = BCD2BIN(regs[0x06] & 0xFF);
	if(regs[5] & 0x80)
    	{
        	tm->tm_year += 1900;
    	}
    	else
    	{
        	tm->tm_year += 2000;
    	}	
		
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	
	tm->tm_year -= 1900;			//inorder to cooperate the systerm time
		
	if(tm->tm_year < 0)
		tm->tm_year = 0;
	
	tm->tm_isdst = 0;

	printk("---read time ------  year:%d, mon:%d, day:%d, weekday:%d hour:%d min:%d sec:%d\n", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_wday,tm->tm_hour,tm->tm_min,tm->tm_sec);
	
	return 0;
}


/*
	the tm->tm_year is : (the real year) - 1900 
	the tm->tm_mon is : (the real mon) - 1	
*/
static int hym8563_i2c_set_time(struct i2c_client *client, struct rtc_time  *tm)	
{
	u8 regs[HYM8563_RTC_SECTION_LEN] = { 0, };
	u8 mon_day,i;
	//u8 temp_for;
	int ret=0;
	mon_day = rtc_month_days((tm->tm_mon), tm->tm_year + 1900);
	
	if(tm->tm_sec >= 60 || tm->tm_sec < 0 )		//set  sec
		regs[0x00] = BIN2BCD(0x00);
	else
		regs[0x00] = BIN2BCD(tm->tm_sec);
	
	if(tm->tm_min >= 60 || tm->tm_min < 0 )		//set  min	
		regs[0x01] = BIN2BCD(0x00);
	else
		regs[0x01] = BIN2BCD(tm->tm_min);

	if(tm->tm_hour >= 24 || tm->tm_hour < 0 )		//set  hour
		regs[0x02] = BIN2BCD(0x00);
	else
		regs[0x02] = BIN2BCD(tm->tm_hour);
	
	if((tm->tm_mday) > mon_day)				//if the input month day is bigger than the biggest day of this month, set the biggest day 
		regs[0x03] = BIN2BCD(mon_day);
	else if((tm->tm_mday) > 0)
		regs[0x03] = BIN2BCD(tm->tm_mday);
	else if((tm->tm_mday) <= 0)
		regs[0x03] = BIN2BCD(0x01);

	if( tm->tm_year >= 200)		// year >= 2100
	{
		regs[0x06] = BIN2BCD(99);	//year = 2099
	}
	else if(tm->tm_year >= 100)			// 2000 <= year < 2100
	{
		regs[0x06] = BIN2BCD(tm->tm_year - 100);
	}
	else if(tm->tm_year >= 0)				// 1900 <= year < 2000
	{
		regs[0x06] = BIN2BCD(tm->tm_year);	
		regs[0x05] |= 0x80;	
	}
	else									// year < 1900
	{
		regs[0x06] = BIN2BCD(0);	//year = 1900	
		regs[0x05] |= 0x80;	
	}
	
	regs[0x04] = BIN2BCD(tm->tm_wday);		//set  the  weekday

	regs[0x05] = (regs[0x05] & 0x80)| (BIN2BCD(tm->tm_mon + 1) & 0x7F);		//set  the  month
	
	for(i=0;i<HYM8563_RTC_SECTION_LEN;i++){
		ret = hym8563_i2c_set_regs(client, RTC_SEC+i, &regs[i], 1);
		if (ret < 0)
		    return 0;
	}
	//hym8563_i2c_set_regs(client, RTC_SEC, regs, HYM8563_RTC_SECTION_LEN);		//write the time date depend on the i2c

	DBG("\n--set time-- %d-%d-%d   %d:%d:%d weekday:%d\n",tm->tm_year+1900,tm->tm_mon+1,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec,BCD2BIN(regs[0x04]));
	return 0;
}


static int hym8563_i2c_read_alarm(struct i2c_client *client, struct rtc_wkalrm *tm)
{
	u8 regs[4] = { 0, };

	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	DBG(">>%s->%d---tm->enabled=%d  tm->pending=%d\n",__FUNCTION__,__LINE__,tm->enabled,tm->pending);
	DBG("\n---hym8563_i2c_set_alarm-->rmin=%d rhour=%d rmday=%d rwday=%d\n",BCD2BIN(regs[0]& 0x7f),BCD2BIN(regs[1]& 0x7f),BCD2BIN(regs[2]& 0x7f),BCD2BIN(regs[3]& 0x7f));
	regs[0] = 0x0;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	DBG(">>%s->%d---data==%x \n",__FUNCTION__,__LINE__,regs[0]);
	return 0;
}

static int hym8563_i2c_set_alarm(struct i2c_client *client, struct rtc_wkalrm const *tm)
{	
	u8 regs[4] = { 0, };
	u8 mon_day;
	regs[0] = 0x0;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	mon_day = rtc_month_days((tm->time.tm_mon), tm->time.tm_year + 1900);
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	DBG(">>%s->%d---tm->enabled=%d  tm->pending=%d\n",__FUNCTION__,__LINE__,tm->enabled,tm->pending);
	DBG("\n---hym8563_i2c_set_alarm-->rmin=%d rhour=%d rmday=%d rwday=%d!\n",BCD2BIN(regs[0])& 0x7f,BCD2BIN(regs[1])& 0x7f,BCD2BIN(regs[2])& 0x7f,BCD2BIN(regs[3])& 0x7f);
	DBG("\n-----------alarm_rtc_sec=%d",tm->time.tm_sec);
	DBG("\n-----------alarm_rtc_min=%d",tm->time.tm_min);
	DBG("\n-----------alarm_rtc_hour=%d",tm->time.tm_hour);
	DBG("\n-----------alarm_rtc_mday=%d",tm->time.tm_mday);
	DBG("\n-----------alarm_rtc_mon=%d",tm->time.tm_mon);
	DBG("\n-----------alarm_rtc_year=%d",tm->time.tm_year);
	DBG("\n-----------alarm_rtc_wday=%d",tm->time.tm_wday);
	DBG("\n-----------alarm_rtc_yday=%d",tm->time.tm_yday);
	
	if(tm->time.tm_min >= 60 || tm->time.tm_min < 0 )		//set  min	
		regs[0x00] = BIN2BCD(0x00) & 0x7f;
	else
		regs[0x00] = BIN2BCD(tm->time.tm_min) & 0x7f;

	if(tm->time.tm_hour >= 24 || tm->time.tm_hour < 0 )		//set  hour
		regs[0x01] = BIN2BCD(0x00) & 0x7f;
	else
		regs[0x01] = BIN2BCD(tm->time.tm_hour) & 0x7f;

	regs[0x03] = BIN2BCD (tm->time.tm_wday) & 0x7f;	
	
	if((tm->time.tm_mday) > mon_day)				//if the input month day is bigger than the biggest day of this month, set the biggest day 
		regs[0x02] = BIN2BCD(mon_day) & 0x7f;
	else if((tm->time.tm_mday) > 0)
		regs[0x02] = BIN2BCD(tm->time.tm_mday) & 0x7f;
	else if((tm->time.tm_mday) <= 0)
		regs[0x02] = BIN2BCD(0x01) & 0x7f;

	hym8563_i2c_set_regs(client, RTC_A_MIN, regs, 4);
	
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	DBG("\n---hym8563_i2c_set_alarm-->rmin=%d rhour=%d rmday=%d rwday=%d!\n",BCD2BIN(regs[0]& 0x7f),BCD2BIN(regs[1]& 0x7f),BCD2BIN(regs[2]& 0x7f),BCD2BIN(regs[3]& 0x7f));
	hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	DBG(">>%s->%d---tm->enabled=%d  tm->pending=%d  data= %x\n",__FUNCTION__,__LINE__,tm->enabled,tm->pending,regs[0]);
	if(tm->enabled == 1)
		regs[0] |= AIE;
	else
		regs[0] &= 0x0;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	DBG(">>%s->%d---data==%x \n",__FUNCTION__,__LINE__,regs[0]);
	return 0;
}

static int hym8563_i2c_open_alarm(struct i2c_client *client)
{

	u8 data;
	printk("%s-->%d\n",__FUNCTION__,__LINE__);
	hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data = 0;
	hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	return 0;
}

static int hym8563_i2c_close_alarm(struct i2c_client *client)
{
	u8 data;
	hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data &= ~AIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	return 0;
}


static int hym8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int tmp;

	tmp=hym8563_i2c_read_time(to_i2c_client(dev), tm);
	
	DBG("\n-----------hym8563_rtc_read_time !");
	DBG("\n-----------rtc_sec=%d",tm->tm_sec);
	DBG("\n-----------rtc_min=%d",tm->tm_min);
	DBG("\n-----------rtc_hour=%d",tm->tm_hour);
	DBG("\n-----------rtc_mday=%d",tm->tm_mday);
	DBG("\n-----------rtc_mon=%d",tm->tm_mon);
	DBG("\n-----------rtc_year=%d",tm->tm_year);

	return tmp;
}

static int hym8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	DBG("\n-----------hym8563_rtc_set_time !");

	return hym8563_i2c_set_time(to_i2c_client(dev), tm);
}

static int hym8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	DBG("**************8>>%s->%d---tm->enabled=%d  tm->pending=%d\n",__FUNCTION__,__LINE__,tm->enabled,tm->pending);
	return hym8563_i2c_read_alarm(to_i2c_client(dev), tm);
}

static int hym8563_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	DBG("**************>>%s->%d---tm->enabled=%d  tm->pending=%d\n",__FUNCTION__,__LINE__,tm->enabled,tm->pending);
	return hym8563_i2c_set_alarm(to_i2c_client(dev), tm);
}

#if defined(CONFIG_RTC_INTF_DEV) || defined(CONFIG_RTC_INTF_DEV_MODULE)
static int hym8563_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	switch (cmd) {
	case RTC_AIE_OFF:
		if(hym8563_i2c_close_alarm(client) < 0)
			goto err;
		break;
	case RTC_AIE_ON:
		if(hym8563_i2c_open_alarm(client))
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
#define hym8563_rtc_ioctl NULL
#endif

#if defined(CONFIG_RTC_INTF_PROC) || defined(CONFIG_RTC_INTF_PROC_MODULE)
static int hym8563_rtc_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}
#else
#define hym8563_rtc_proc NULL
#endif

#if 0

/*two level wakeup event function*/
static irqreturn_t rk28_clean_rtcirq_handler(s32 irq, void *dev_id)
{
	u8 regs[4] = { 0, };
	regs[0] = 0x0;
	hym8563_i2c_set_regs(rtc_client, RTC_CTL2, regs, 1);
	printk(">>>Enter %s\n",__FUNCTION__);
    return IRQ_HANDLED;
}

#endif
static int hym8563_attach_adapter(struct i2c_adapter *adapter)
{

	DBG("\n-----------hym8563_attach_adapter !");

//	adapter->deviceaddr=(HYM_ADDR);
//	adapter->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
//	adapter->mode=0;

	return i2c_probe(adapter, &addr_data, hym8563_probe);
}

static int hym8563_detach_client(struct i2c_client *client)
{
	struct rtc_device *const rtc = i2c_get_clientdata(client);
	
	if (rtc)
		rtc_device_unregister(rtc);
	sysfs_remove_group(&client->dev.kobj, &hym8563_group);
	return i2c_detach_client(client);
}

static struct i2c_driver hym8563_driver = {
	.driver 	= {
		.name	= DRV_NAME,
	},
	.id 	= I2C_DRIVERID_MAX6900,
	.attach_adapter = hym8563_attach_adapter,
	.detach_client	= hym8563_detach_client,
};

static const struct rtc_class_ops hym8563_rtc_ops = {
	.read_time	= hym8563_rtc_read_time,
	.set_time		= hym8563_rtc_set_time,
	.read_alarm 	= hym8563_rtc_read_alarm,
	.set_alarm	= hym8563_rtc_set_alarm,
	.ioctl 		= hym8563_rtc_ioctl,
	.proc		= hym8563_rtc_proc
};

static int hym8563_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	struct rtc_time tm_read, tm = {
	.tm_wday = 4,
	.tm_year = 109,
	.tm_mon = 9,
	.tm_mday = 1,
	.tm_hour = 12, 
	.tm_min = 10,
	.tm_sec = 58
	};	
	
	int rc = 0;
	struct i2c_client *client = NULL;
	struct rtc_device *rtc = NULL;
	int error;
	DBG("-----------rtc  hym8563_probe !");
	rockchip_mux_api_set(GPIOE_SPI1_SEL_NAME, IOMUXA_GPIO1_A1237);/*RTC interrupt pin*/
	GPIOSetPinDirection(GPIOPortE_Pin2,GPIO_IN);
	GPIOPullUpDown(GPIOPortE_Pin2,GPIOPullUp);
	//GPIOSetPinLevel(GPIOPortF_Pin7,GPIO_HIGH);
	DBG("RTC INT status == %d!!\n",GPIOGetPinLevel(GPIOPortE_Pin2));

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		rc = -ENODEV;
		goto failout;
	}
	
	client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == NULL) {
		rc = -ENOMEM;
		goto failout;
	}
	client->mode = NORMALMODE;
#ifdef CONFIG_MACH_PWS700AA
	client->Channel = I2C_CH0;
#else
	client->Channel = I2C_CH1;
#endif	
	client->addr = addr;
	client->adapter = adapter;
	client->speed = 100;
	client->driver = &hym8563_driver;
	rtc_client = client;
	strlcpy(client->name, DRV_NAME, I2C_NAME_SIZE);

	if (kind < 0) {
		rc = hym8563_i2c_validate_client(client);
		if (rc < 0)
			goto failout;
	}

	rc = i2c_attach_client(client);
	if (rc < 0)
		goto failout;

	//dev_info(&client->dev,"chip found, driver version  0.1 \n");	
	error = sysfs_create_group(&client->dev.kobj,&hym8563_group);
	if(error)
	{
		printk("rtc hym8563 probe create sysfs group error!\n");
		goto failout_detach;
	}
	//rtc :first member is dev;
	rtc = rtc_device_register(hym8563_driver.driver.name, &client->dev, &hym8563_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rc = PTR_ERR(rtc);
		goto failout_detach;

		DBG("-----------rtc device register fail!");
		
	}
	DBG("-----------rtc device register sucess !");
	//   client->dev.driver_data = rtc 
	i2c_set_clientdata(client, rtc);



	hym8563_init_device(client);
		
	hym8563_i2c_read_time(client, &tm_read);	//read time from hym8563


	DBG("\n-----------read tm_sec=%d",tm_read.tm_sec);
	DBG("\n----------- tm_min=%d",tm_read.tm_min);
	DBG("\n----------- tm_hour=%d",tm_read.tm_hour);
	DBG("\n----------- tm_mday=%d",tm_read.tm_mday);
	DBG("\n----------- tm_mon=%d",tm_read.tm_mon);
	DBG("\n----------- tm_year=%d",tm_read.tm_year);

#if 1
    if(((tm_read.tm_year < 70) | (tm_read.tm_year > 137 )) | (tm_read.tm_mon == -1))	//if the hym8563 haven't initialized
	{
		hym8563_i2c_set_time(client, &tm);	//initialize the hym8563 
	}	
#endif
	return 0;

failout_detach:
	i2c_detach_client(client);
failout:
	kfree(client);
	return rc;
}

static int __init hym8563_init(void)
{
	int tmp;
	
	tmp=i2c_add_driver(&hym8563_driver);

	DBG ("--------i2c add driver hym8563 result=%d",tmp);
return 0;
}

static void __exit hym8563_exit(void)
{
	i2c_del_driver(&hym8563_driver);
}

MODULE_DESCRIPTION("HYM8563 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(hym8563_init);
module_exit(hym8563_exit);

