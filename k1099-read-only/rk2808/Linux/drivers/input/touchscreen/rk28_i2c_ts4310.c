/****************************************************************************************
 * driver/input/touchscreen/rk28_i2c_ts4310.c
 *Copyright 	:ROCKCHIP  Inc
 *Author	: zhaojun 
 *Date		: 2009-06-25
 *This driver use for rk28 chip extern touchscreen. Use i2c IF ,the chip is ar4310
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

#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
//#include <linux/workqueue.h>
//#include <linux/leds.h>
#include <asm/arch/api_i2c.h>


/* macro difine*/
#define RA4310_IIC_ADDR 	0x38  /*(0x1a<<1)*/
#define RA4310_SENSE_IO 	(GPIOPortE_Pin3)
#define TOUCH_NOTHING		(-1)

#define SCREEN_X			800		/*logic lcd size*/
#define SCREEN_Y			480
#define MAX_TOUCH_X			0XFB 
#define MAX_TOUCH_Y 		0X8B

#define  CALI_ALL_OK		0
#define  CALI_X_ERR    		1
#define  CALI_Y_ERR    		2
#define  CALI_CENT_ERR	 	3
#define  CALI_X_DELTA		20
#define  CALI_Y_DELTA		20
#define 	CALI_X_PIXEL			8
#define 	CALI_Y_PIXEL			6

#define CALI_MIN_X	68
#define CALI_MIN_Y	100
#define CALI_MAX_X	950
#define CALI_MAX_Y	950
#define TOUCH_GLOBALS
#define CALI_MID_X	((CALI_MIN_X+CALI_MAX_X)/2)
#define CALI_MID_Y	((CALI_MIN_Y+CALI_MAX_Y)/2)

#define TS4310_POLL_DELAY	(40*1000000)	/* ns delay before the first sample */
#define TS4310_POLL_PERIOD	(20*1000000)	/* ns delay between samples */


#define ra4310_NAME "rockchip Application touchPanel"
#define ra4310_VERSION	"1.3.1"
#define ra4310_drverN		"ra4310name"

/* How often we poll keys - msecs */
#define POLL_INTERVAL_DEFAULT	1000


#ifdef	TOUCH_GLOBALS	//模块名称.
#define  TOUCH_EXT		/* 固定公共部分*/
#else
#define  TOUCH_EXT	extern
#endif

#define DBG_ROCK4310
//zhaojun add for debug
#ifdef DBG_ROCK4310
	#define DBG4310 printk
#else
	#define DBG4310 if(0)
#endif

typedef struct
{
	u8  px;	  //LOGIC POINT
	u8  py;

} PHY_XY,PPHY_XY;

typedef struct
{
	uint16	lx;   //LOGIC POINT
	uint16	ly;

} LOGIC_XY,PLOGIC_XY;

typedef enum
{
	TopLPos_A = 0,	// 左上校准点
	TopRPos_B,	 // 右上校准点
	BotLPos_C,	 // 左下校准点
	BotRPos_D,	 // 右下校准点
	CentPos_E	// 中心校准点
}TOUCH_CALIPOINT;//, 


typedef struct
{
	INT32  Kx;	  //LOGIC POINT
	INT32  Ky;

} CALI_COEF;

/*
typedef struct
{
	UHPOINT  TouchAdPoint[5];	 // AD值用来精确判断注册表中的校准数值是否在合理范围
	CALI_COEF TouchCaliCoef;	 // AD值与像素坐标逻辑值之间的关联系数
}TOUCH_GLOBAL;


typedef struct TouchActiveArea
{
	UINT16 TouchActiveArea[8];
	UINT16	TotalTouchActiveArea;
}TOUCHACTIVEAREA;
typedef struct _TOUCH_INFO
{
	INT32U			TouchState;
	UHPOINT 		TouchPoint;
	UHPOINT 		AdPoint;
} TOUCH_INFO , *PTOUCH_INFO;
*/

static u8 i2c_buf[10];


//event
struct RA4310_TS_EVENT{
	struct input_dev *input;
//	struct i2c_driver *i2c;
	struct i2c_client *client;
	struct work_struct	x_work;
	struct work_struct	y_work;
	char	phys[32];
	int 		irq;

	int 	ADCcount;
	PHY_XY 	pos_phy;
	LOGIC_XY pos_logic;
	
	//uint16	x;
	//uint16	y;
	uint16	z1;
	uint16	z2;
	//uint16	   touch_x;
	//uint16	touch_y;
	bool		pendown;
	bool	 status;
	struct hrtimer	timer;
};




//funktion
static int ra4310_probe(struct i2c_adapter *, int, int);



//--------------------------------------------------------------------

/* for now, we only support one address */
static unsigned short normal_i2c[] = {RA4310_IIC_ADDR>>1 , I2C_CLIENT_END};
static unsigned short ignore = I2C_CLIENT_END;
static struct i2c_client_address_data addr_data4310 = {
	.normal_i2c	= normal_i2c,
	.probe		= &ignore,
	.ignore		= &ignore,
};


/*
	 __________ 	   __________	   __________								  
	|		   |	  | 		 |	  | 		 |							 
	|	  a    |  b   | 	c	 |	d | 	e	 |			-------
	|__________|	  |__________|	  |__________|			

		  1
	 __________ 	   __________						 
	|		   |	  | 		 |									 
	|	  2    |	  | 		 |			 ---------------------------			  
	|__________|	  |__________|	

		  3

	 __________ 	   __________						 
	|		   |	  | 		 |									 
	|	  4    |	  | 		 |			  --------------------------					
	|__________|	  |__________|	
   把屏幕分为a和c为中间线的条状区域
	  认为该区域内X线型以 a 和c方框的中间点求出该线型系数

   分别以a和c的X坐标求出x相关系数
	 gTouchCaliCoef.Kx = 100 * (71-35 ) /(161-64);
   分别以4和2的X坐标求出x相关系数
	 gTouchCaliCoef.Ky = 100 * (69-34 ) /(154-61);
   条状区域内的点以b为基准点求出他们的转换值	 
   pPixelPoint->x = 100 * (adPoint.x - TouchADPoint_x) / gTouchCaliCoef.Kx + TouchLogicPoint_x;
   y方向同理
	
*/

typedef struct Tag_Point
{
  int ad_value;
  int Logic_value;
  
}TransPoint;

int XAreaBounder[]={35,71,107,143,179,215,251};
int YAreaBounder[]={34,69,107,139};
TransPoint XBasePoint[]=
{
	{28,112},
	{57,210},
	{91,307},
	{127,404},
	{163,494},
	{198,592},
	{244,698},

};
TransPoint YBasePoint[]=
{
	{27,107},
	{59,199},
	{94,291},
	{127,383},
};

CALI_COEF gTouchCaliCoef;



int FindXAreaId(UHPOINT adpoint)
{
  int i;
  UHPOINT TemAdPoint;
  TemAdPoint=adpoint;
  if(adpoint.x>251)
	 return 0;
  for(i=0;i<ARRSIZE(XAreaBounder);i++)
  {
   if(adpoint.x<=XAreaBounder[i])
	  return  i;
  }
  
}
int FindYAreaId(UHPOINT adpoint)
{
  int i;
  UHPOINT TemAdPoint;
  TemAdPoint=adpoint;
  if(adpoint.y>139)
	 return  0;
  for(i=0;i<ARRSIZE(YAreaBounder);i++)
  {
   if(adpoint.y<=YAreaBounder[i])
	  return  i;
  }
  
}
UINT16 CheckPointInRect(UHPOINT *point, UHRECT* Rect)
{

	if ((point->x >= Rect->x) && (point->x <= Rect->x + Rect->w) && (point->y >= Rect->y) && (point->y <= Rect->y + Rect->h))
		return TRUE;
	else
		return FALSE;

}







// read the ra4310 registe,used i2c bus
static int ra4310_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 1, len, buf },
	};
	buf[0] = reg;
	//BUG_ON(len == 0);
	//BUG_ON(reg > RTC_T_COUNT);
	//BUG_ON(reg + len > RTC_T_COUNT + 1);

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}


// set the ra4310 registe,used i2c bus
static int ra4310_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len)
{
	int ret;
	struct i2c_msg msgs[1] = {
		{ client->addr, 0, len + 1, i2c_buf }
	};

	//BUG_ON(len == 0);
	//BUG_ON(reg > RTC_T_COUNT);
	//BUG_ON(reg + len > RTC_T_COUNT + 1);

	i2c_buf[0] = reg;
	memcpy(&i2c_buf[1], &buf[0], len);
	
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret > 0)
		ret = 0;
	
	return ret;
}




//change the position from PHY to logic value
void tsposition_phy2logic(PHY_XY*p, LOGIC_XY*l)
{
	UHPOINT phy_point;
	int xid,yid;   
	int TouchLogicPoint_x,TouchLogicPoint_y;
	int TouchADPoint_x,TouchADPoint_y;

	//防止数组溢出
	if(p->px>MAX_TOUCH_X)p->px=MAX_TOUCH_X;
	if(p->py>MAX_TOUCH_Y)p->py=MAX_TOUCH_Y;

	

	if (p->px == 0 && p->py == 0)
	{
		l->lx	= 0;
		l->ly = 0;
		return;
		
	}
	
	if((p->px>(MAX_TOUCH_X-3))||(p->py>(MAX_TOUCH_Y-3)))
	{
				
			//线性转换
			l->lx=((uint32)p->px)*(SCREEN_X)/MAX_TOUCH_X;
			l->ly=((uint32)p->py)*(SCREEN_Y)/MAX_TOUCH_Y;;
	}
	else	//校准
	{
		phy_point.x=p->px;
		phy_point.y=p->py;
	
		xid= FindXAreaId(phy_point);
		yid= FindYAreaId(phy_point);
	   
		TouchADPoint_x=XBasePoint[xid].ad_value;
		TouchLogicPoint_x=XBasePoint[xid].Logic_value;
		
		TouchADPoint_y=YBasePoint[yid-1].ad_value;
		TouchLogicPoint_y=YBasePoint[yid-1].Logic_value;
		
		l->lx = 100 * (phy_point.x - TouchADPoint_x) / gTouchCaliCoef.Kx + TouchLogicPoint_x;
		l->ly = 100 * (phy_point.y - TouchADPoint_y) / gTouchCaliCoef.Ky + TouchLogicPoint_y;
	}






	
}



//get position and state in touch screen ra4310
int get_logic_position_ra4310(struct RA4310_TS_EVENT *ts_dev)
{
	int i,sr,touchstate;
	uint32 sumx,sumy;
	u8 buf[2];

	touchstate =  GPIOGetPinLevel(RA4310_SENSE_IO);
	if(touchstate>0)			//低电平－按下 ，高电平－未按
		{
			return TOUCH_NOTHING;	
		}
	

	sumx=sumy=0;
	for(i=0;i<5;i++)  //连续采用5次取平均值
		{
			sr=ra4310_read_regs(ts_dev->client, 0, buf,2);
		  	if(sr<0)
		  	{
			  DBG4310("\n----I2C read ra4310 err");
			  return -2;
		  	}

			sumx=sumx+buf[0];
			sumy=sumy+buf[1];
		}
	ts_dev->pos_phy.px=(sumx/5)&0xff;
	ts_dev->pos_phy.py=(sumy/5)&0xff;		

	if((ts_dev->pos_phy.px>MAX_TOUCH_X)&&(ts_dev->pos_phy.py>MAX_TOUCH_Y))
		return -3;		//no msg
	
	tsposition_phy2logic(&ts_dev->pos_phy,&ts_dev->pos_logic);

	return 0;
	

}



//the polling handle of touch screen
 static enum hrtimer_restart ra4310_timerhandle(struct hrtimer *handle)
{
	struct RA4310_TS_EVENT *ts_dev = container_of(handle, struct RA4310_TS_EVENT, timer);
	struct input_dev *ra4310_ts_dev;
	int touchstat=0;

		ra4310_ts_dev=ts_dev->input;


		touchstat=get_logic_position_ra4310(ts_dev);
		if((TOUCH_NOTHING==touchstat)&&(ts_dev->pendown))	//has be released
			{
			
			printk("\n The touchscreen release!!");
			ts_dev->pendown = 0;
			input_report_key(ra4310_ts_dev,BTN_TOUCH,0);
			input_sync(ra4310_ts_dev);
			return HRTIMER_RESTART;
			}

		 if(touchstat<0)
		 	{
		 		return HRTIMER_RESTART;			//no msg
		 	};


			if(!ts_dev->pendown)
			{
				printk("\nThe touchscreen down!!,X=%d, Y=%d",ts_dev->pos_logic.lx,ts_dev->pos_logic.ly);
				input_report_key(ra4310_ts_dev,BTN_TOUCH,1);
				ts_dev->pendown = 1;
			}

			input_report_abs(ra4310_ts_dev,ABS_X,ts_dev->pos_logic.lx);
			input_report_abs(ra4310_ts_dev,ABS_Y,ts_dev->pos_logic.ly);
			input_sync(ra4310_ts_dev);
			printk("\n get a touch position,x=0x%x,y=0x%x,   X=%d, Y=%d",
			ts_dev->pos_phy.px,ts_dev->pos_phy.py,ts_dev->pos_logic.lx,ts_dev->pos_logic.ly);

		 
		 return HRTIMER_RESTART;
	 
	 
	 
}




//detach ra4310 from i2c bus
static int ra4310_detach_client(struct i2c_client *client)
{
/*	struct ra4310 *ap = i2c_get_clientdata(client);

	if (device_chip[ra4310_DEV_LED] != CHIP_NONE)
		led_classdev_unregister(&ap->mail_led);

	input_unregister_polled_device(ap->ipdev);
	i2c_detach_client(&ap->client);
	input_free_polled_device(ap->ipdev);
*/
	return 0;
}

/* Function is invoked for every i2c adapter. */
static int ra4310_attach_adapter(struct i2c_adapter *adap)
{
	
	//adap->deviceaddr=(RA4310_IIC_ADDR);
	//adap->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	//adap->mode=0;

	return i2c_probe(adap, &addr_data4310, ra4310_probe);
}

static void ra4310_shutdown(struct i2c_client *client)
{
	ra4310_detach_client(client);
}

static struct i2c_driver ra4310_driver = {
	.driver = {
		.name = ra4310_drverN,
			
	},
	
	.id 	= RA4310_IIC_ADDR,
	.attach_adapter = &ra4310_attach_adapter,
	.detach_client  = &ra4310_detach_client,
	.shutdown	= &ra4310_shutdown,
};

static struct  i2c_client ra4310_client = {
		.driver = &ra4310_driver,
		.name   = ra4310_drverN,
	};


/* NB: Only one panel on the i2c. */
static int ra4310_probe(struct i2c_adapter *bus, int address, int kind)
{
	//struct input_polled_dev *ipdev;
	//u8 cmd = device_chip[ra4310_DEV_APPBTN] == CHIP_OZ992C ? 0 : 8;
	struct RA4310_TS_EVENT	*ts_dev;
	struct input_dev *ra4310_ts_dev;
	unsigned int err = 0;
	u8 regs[4];
	int sr,i;

	DBG4310("---------------probe ra4310 adapter %p addr %x kind %d\n",	bus, address, kind);

	ts_dev=kzalloc(sizeof(struct RA4310_TS_EVENT), GFP_KERNEL);
	if(!ts_dev)
	{
		DBG4310("failed to allocate memory!!\n");
		goto nomem;
	}


	gTouchCaliCoef.Kx = (INT32)100 * (71-35 ) /(161-64);
	gTouchCaliCoef.Ky = (INT32)100 * (69-34 ) /(154-61);
		



	ra4310_client.adapter = bus;
	ra4310_client.addr= RA4310_IIC_ADDR>>1;//address;
	ts_dev->client=&ra4310_client;
	//ap->client.adapter->deviceaddr=(RA4310_IIC_ADDR);
	//ap->client.adapter->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
	//ap->client.adapter->mode=0;

	
	//i2c_set_clientdata(&ts_dev->client, &ra4310_client);

/*	err = i2c_attach_client(&ts_dev->client);
	if (err)
		{
		DBG4310("\n----ra4310 err: attach i2c fail!");
		goto outi2c;
		}
*/




	
	ra4310_ts_dev = input_allocate_device();
	if(!ra4310_ts_dev)
	{
		DBG4310("rk28 ra4310_ts allocate input device failed!!!\n");	
		goto fail1;
	}
	ts_dev->input = ra4310_ts_dev;

	
/*init	timer to dispose workqueue */
	hrtimer_init(&ts_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts_dev->timer.function = ra4310_timerhandle;

	ts_dev->status = 0;
	ts_dev->pendown = 0;
	//ts_dev->irq =XPT2046_IRQ;
	ts_dev->input->phys="rock4310-touch/input0";
	//snprintf(ts_dev->phys,sizeof(ts_dev->phys),"%s/input0",spi->dev.bus_id);
	
	ra4310_ts_dev->evbit[0] = BIT_MASK(EV_ABS)|BIT_MASK(EV_KEY);
	ra4310_ts_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(ra4310_ts_dev,ABS_X,5,800,0,0);
	input_set_abs_params(ra4310_ts_dev,ABS_Y,10,480,0,0);
	ra4310_ts_dev->name = ra4310_NAME;
	

	
	err = input_register_device(ra4310_ts_dev);
	if(err)
		goto fail2;

	GPIODisableIntr(RA4310_SENSE_IO);
	GPIOSetPinDirection(RA4310_SENSE_IO,GPIO_IN);


	hrtimer_start(&ts_dev->timer,ktime_set(0,TS4310_POLL_DELAY),HRTIMER_MODE_REL);	

	
	return err;

fail2:	
//	free_irq(XPT2046_IRQ,NULL);

fail1:
	input_free_device(ra4310_ts_dev);
	hrtimer_cancel(&ts_dev->timer);
	
outi2c:	
		i2c_detach_client(&ts_dev->client);
nomem:
		kfree(ts_dev);

	return err;



}



static int __init ra4310_init(void)
{

	int tmp;
	return i2c_add_driver(&ra4310_driver);
	
	

}

static void __exit ra4310_cleanup(void)
{
	i2c_del_driver(&ra4310_driver);
	
}


module_init(ra4310_init);
module_exit(ra4310_cleanup);

MODULE_AUTHOR("Stephen Hemminger <shemminger@linux-foundation.org>");
MODULE_DESCRIPTION(ra4310_NAME " driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(ra4310_VERSION);

MODULE_ALIAS("dmi:*:svnFUJITSU:pnLifeBook*:pvr*:rvnFUJITSU:*");
MODULE_ALIAS("dmi:*:svnFUJITSU:pnLifebook*:pvr*:rvnFUJITSU:*");
