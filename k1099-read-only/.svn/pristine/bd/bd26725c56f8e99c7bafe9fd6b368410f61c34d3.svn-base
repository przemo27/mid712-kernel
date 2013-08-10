/*******************************************************************/
/*	  Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.			  */
/*******************************************************************
File		:	 I2C-ROCKSOFT.c
Desc		:	 the driver of i2c
Author		:	 some body(YONG,CONG &ZHAOJUN)
Date		:	 2009-5-20
Notes		:
$Log: i2c.c,v $
Revision 0.00

   fix some bug of i2c controller
   zhaojun	2009 7-20
********************************************************************/


#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/typedef.h>
#include <asm/arch/iomux.h>
#include <asm/arch/rk28_macro.h>
#include <asm/arch/rk28_debug.h>

#define IN_DRIVER_API_I2C
#include <asm/arch/rk28_i2c.h>
#include <asm/arch/api_i2c.h>
#include <asm/arch/rk28_scu.h>

#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
#define RK28_I2C0_SPEED 		    80 	 /* KHZ */
#define RK28_I2C1_SPEED                   80      /* KHZ */
#define rk28_i2c_read(reg)		__raw_readl(I2C0_BASE_ADDR_VA	 + (reg))
#define rk28_i2c_write(reg, val)	__raw_writel((val), I2C0_BASE_ADDR_VA	 + (reg))
#define rk28_i2c1_read(reg)		__raw_readl(I2C1_BASE_ADDR_VA	 + (reg))
#define rk28_i2c1_write(reg, val)	__raw_writel((val), I2C1_BASE_ADDR_VA	 + (reg))

#ifdef CONFIG_MACH_PWS700AA
#define I2C1_DEV_OPEN 0
#else
#define I2C1_DEV_OPEN 1
#endif
extern 	int rockchip_clk_get_apb( void );
long I2CSendData(unsigned char data, unsigned char startBit);
long I2CReadData(unsigned char * data);
static void __devinit rk28_i2c_hwinit(void);
static void __devinit rk28_i2c1_hwinit(void);

static struct mutex i2c_rw_mutex;
//static DEFINE_SPINLOCK(rk28i2c_lock);

static uint16 g_i2cSpeed0=RK28_I2C0_SPEED;
static uint16 g_i2cSpeed1=RK28_I2C1_SPEED;


/*----------------------------------------------------------------------
Name	  : I2CStop(eI2C_ch_t i2cNumb)
Desc	  : I2C的停止
Params	  : I2C控制器通道号，0或1
Return	  :无
----------------------------------------------------------------------*/
void _I2CStop(eI2C_ch_t i2cNumb)
{
	int32 waiteDelay;
	pI2C_REG  pI2cRegDev;
	DBG("%s[%d]\n",__FUNCTION__,__LINE__);
	if(i2cNumb == 0)
	{
		waiteDelay = 20000 / g_i2cSpeed0;
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;
	}
	else
	{
		waiteDelay = 20000 / g_i2cSpeed1;
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
	}
	pI2cRegDev->I2C_LCMR = pI2cRegDev->I2C_LCMR | I2C_LCMR_RESUME | I2C_LCMR_STOP;

	while (((pI2cRegDev->I2C_LCMR & I2C_LCMR_STOP) != 0)&& (waiteDelay > 0))
	{
		udelay(100*10);
		waiteDelay--;
	}
	pI2cRegDev->I2C_IER = 0;
	pI2cRegDev->I2C_ISR = 0;

}

/*----------------------------------------------------------------------
Name	  : I2CSetSpeed(eI2C_ch_t i2cNumb, uint16 speed, uint32 APBnKHz)
Desc	  : 设置I2C的速度，
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			speed:输入要设置的频率 (以K为单位) 最高速度为400KHz
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CSetSpeed(eI2C_ch_t i2cNumb, uint16 speed, uint32 APBnKHz)
{
	uint32 exp;
	uint32 rem;
	pI2C_REG  pI2cRegDev;
	

	if(i2cNumb == 0)
	{
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;
		g_i2cSpeed0 = speed;
	}
	else
	{
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
		g_i2cSpeed1 = speed;
	}
	/*
	SCL Divisor = (I2CCDVR[5:3] + 1) x 2^((I2CCDVR[2:0] + 1)
	SCL = PCLK/ 5*SCLK Divisor
	rem = I2CCDVR[5:3]
	exp = I2CCDVR[2:0]	 ////PLLGetAPBFreq()
	*/
	rem = APBnKHz/(5*speed);
	for (exp=0; exp<8;exp++)
	{
		rem = rem >> 1;
		if (rem < 8)
		{
			break;
		}
	}
	if (rem > 7)
	{
		return (-1);
	}

	pI2cRegDev->I2C_OPR = (pI2cRegDev->I2C_OPR & (~0x3f)) | (rem<<3) | exp;

	return (0);
}

/*----------------------------------------------------------------------
Name	  : I2CInit(uint8 i2cNumb, uint16 speed)
Desc	  : 设置I2C的初始化
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			speed：I2C的速度 输入要设置的频率 (以K为单位) 最高速度为400KHz
Return	  :成功还回0，失败还回-1.	////还回1为I2C已打开过。
----------------------------------------------------------------------*/
int32 __devinit _I2CInit(eI2C_ch_t i2cNumb, uint16 speed,uint16 I2CSlaveAddr,eI2C_Address_Reg_t addressBit, eI2C_mode_t mode)
{
   // pSCU_REG pheadScu;
	pI2C_REG  pI2cRegDev;
	uint32 APBnKHz;
	

	
	if (i2cNumb > 1)
	{
		return (-1);
	}
	if (i2cNumb == 1)
	{
		pI2cRegDev = (pI2C_REG)I2C1_BASE_ADDR_VA;
		//g_i2cSlaveAddr1 = I2CSlaveAddr;
		//g_i2cAddressBit1 = addressBit;
		//g_i2cMode1 = mode;
	}
	else
	{
		pI2cRegDev = (pI2C_REG)I2C0_BASE_ADDR_VA;
		//g_i2cSlaveAddr0 = I2CSlaveAddr;
		//g_i2cAddressBit0 = addressBit;
		//g_i2cMode0 = mode;
	}

	pI2cRegDev->I2C_OPR = pI2cRegDev->I2C_OPR | RESET_I2C_STATUS;//reset I2C
	udelay(10*10);
	rk28_i2c_hwinit();
	rk28_i2c1_hwinit();
	pI2cRegDev->I2C_OPR = pI2cRegDev->I2C_OPR & (~(0x01ul<<7));
	APBnKHz=rockchip_clk_get_apb()/1000;
	//APBnKHz = 75*1000;//PLLGetAPBFreq();
	if (_I2CSetSpeed(i2cNumb, speed,APBnKHz) != 0) //set I2C speed
	{
		return (-1);
	}

	pI2cRegDev->I2C_IER = 0;//暂不用中断处理，以后可以再加
	pI2cRegDev->I2C_LCMR = 0;
	pI2cRegDev->I2C_OPR = pI2cRegDev->I2C_OPR | I2C_CORE_ENABLE; //enable I2C controller.

	return (0);
}

/*----------------------------------------------------------------------
Name	  : I2CDeInit(eI2C_ch_t i2cNumb)
Desc	  : 设置I2C的反初始化
Params	  : 
			i2cNumb：I2C设备，为 0或1
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CDeInit(eI2C_ch_t i2cNumb)
{
	//pSCU_REG pheadScu;
	pI2C_REG  pI2cRegDev;
	
	if (i2cNumb == 1)
	{
		pI2cRegDev = (pI2C_REG)I2C1_BASE_ADDR_VA;
	}
	else
	{
		pI2cRegDev = (pI2C_REG)I2C0_BASE_ADDR_VA;
	}
	pI2cRegDev->I2C_OPR = (pI2cRegDev->I2C_OPR & (~(0x1ul<<6))) | I2C_CORE_DISABLE; //disable I2C controller.

	if (i2cNumb > 1)
	{
		return (-1);
	}
	return (0);
}

/*----------------------------------------------------------------------
Name	  : I2CUpdateAllApbFreq(uint32 APBnKHz)
Desc	  : 设置APB时一起调整I2C的频率
Params	  : 
Return	  :无
----------------------------------------------------------------------*/

//typedef int (*change_clk )( ip_id ip , int input_clk );
#if 0
int _I2CUpdateAllApbFreq(ip_id ip,int APBHz)
{
	APBHz = APBHz/1000; /* hz to khz */
	if(g_i2cSpeed0 > 0 && ip == SCU_IPID_I2C0 )
	{
		_I2CSetSpeed(I2C_CH0,g_i2cSpeed0,APBHz);
	}
	if(g_i2cSpeed1 > 0 && ip == SCU_IPID_I2C0 )
	{
		_I2CSetSpeed(I2C_CH1,g_i2cSpeed1,APBHz);
	}
}
#else
int _I2CUpdateAllApbFreq(ip_id ip,int APBHz)
{
	APBHz = APBHz/1000; /* hz to khz */
			printk("%s:set I2C%d to %d KHZ\n" , __func__, ip-SCU_IPID_I2C0,APBHz);
	if(g_i2cSpeed0 > 0 && ip == SCU_IPID_I2C0 )
	{
		_I2CSetSpeed(I2C_CH0,g_i2cSpeed0,APBHz);
	}
	if(g_i2cSpeed1 > 0 && ip == SCU_IPID_I2C1 )
	{
		_I2CSetSpeed(I2C_CH1,g_i2cSpeed1,APBHz);
	}
		return 0;
}
#endif
/*----------------------------------------------------------------------
Name	  : I2CSendDataMode(eI2C_ch_t i2cNumb, uint8 data, uint8 startBit)
Desc	  : 发送单个数据
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			data：输入要发送的数据
			startBit: 1：为有起始位，0为没有
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CSendDataMode(eI2C_ch_t i2cNumb, uint8 data, uint8 startBit)
{
	pI2C_REG  pI2cRegDev;
	int32 waiteSendDelay;
	int32 i2cIntStatus;
	int32 ackTempStatus;

	//return I2CSendData( data,  startBit);
	
	if(i2cNumb == 0)
	{
		waiteSendDelay = 20000/g_i2cSpeed0;
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;
	}
	else
	{
		waiteSendDelay = 20000 / g_i2cSpeed1;
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
	}

	pI2cRegDev->I2C_MTXR = data;
	pI2cRegDev->I2C_CONR = (pI2cRegDev->I2C_CONR & (~(1ul<<4))) | I2C_CON_ACK;	///enable ack
	if (startBit == 1)	///有起始位
	{
		pI2cRegDev->I2C_LCMR = /*pI2cRegDev->I2C_LCMR | */I2C_LCMR_START | I2C_LCMR_RESUME;
	}
	else
	{
		pI2cRegDev->I2C_LCMR = /*pI2cRegDev->I2C_LCMR | */I2C_LCMR_RESUME;
	}

    udelay(10);
	while (waiteSendDelay > 0) ///(/*((g_pI2cReg->I2C_LSR & I2C_LSR_RECE_NACK) != 0)&& */(waiteSendDelay > 0))
	{
		i2cIntStatus = pI2cRegDev->I2C_ISR;
		ackTempStatus = pI2cRegDev->I2C_LSR;
		if ((i2cIntStatus & I2C_ARBITR_LOSE_STATUS) != 0)
		{
			pI2cRegDev->I2C_ISR = 0;
			//_I2CStop(i2cNumb);
			printk("%s[%d]---error in _I2CSendData:I2C_ARBITR_LOSE_STATUS\n",__FUNCTION__,__LINE__);
			return (-100);
		}
		if ((i2cIntStatus & I2C_RECE_INT_MACK) != 0)
		{
			break;
		}
		udelay(100*2);
		waiteSendDelay--;
	}
	pI2cRegDev->I2C_ISR = pI2cRegDev->I2C_ISR & (~1ul);
	if (waiteSendDelay > 0)
	{
		return (0);
	}

	printk("%s[%d]--error in _I2CSendData:timer out\n",__FUNCTION__,__LINE__);
	return (-102);
}

/*----------------------------------------------------------------------
Name	  : I2CSendData(eI2C_ch_t i2cNumb, uint8 data, uint8 startBit)
Desc	  : 发送单个数据
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			data：输入要发送的数据
			startBit: 1：为有起始位，0为没有
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CSendData(eI2C_ch_t i2cNumb, uint8 data, uint8 startBit)
{
	pI2C_REG  pI2cRegDev;
	int32 waiteSendDelay;
	int32 i2cIntStatus;
	int32 ackTempStatus = 0;

	//return I2CSendData( data,  startBit);
	
	if(i2cNumb == 0)
	{
		waiteSendDelay = 20000/g_i2cSpeed0;
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;
	}
	else
	{
		waiteSendDelay = 20000 / g_i2cSpeed1;
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
	}

	pI2cRegDev->I2C_MTXR = data;
	pI2cRegDev->I2C_CONR = (pI2cRegDev->I2C_CONR & (~(1ul<<4))) | I2C_CON_ACK;	///enable ack
	if (startBit == 1)	///有起始位
	{
		pI2cRegDev->I2C_LCMR = /*pI2cRegDev->I2C_LCMR | */I2C_LCMR_START | I2C_LCMR_RESUME;
	}
	else
	{
		pI2cRegDev->I2C_LCMR = /*pI2cRegDev->I2C_LCMR | */I2C_LCMR_RESUME;
	}

    udelay(10);
	while (waiteSendDelay > 0) ///(/*((g_pI2cReg->I2C_LSR & I2C_LSR_RECE_NACK) != 0)&& */(waiteSendDelay > 0))
	{
		i2cIntStatus = pI2cRegDev->I2C_ISR;
		ackTempStatus = pI2cRegDev->I2C_LSR;
		if ((i2cIntStatus & I2C_ARBITR_LOSE_STATUS) != 0)
		{
			pI2cRegDev->I2C_ISR = 0;
			//_I2CStop(i2cNumb);
			printk("%s[%d]---error in _I2CSendData:I2C_ARBITR_LOSE_STATUS\n",__FUNCTION__,__LINE__);
			return (-100);
		}
		if ((i2cIntStatus & I2C_RECE_INT_MACK) != 0)
		{
			break;
		}
		udelay(100*2);
		waiteSendDelay--;
	}
#if 0
	if((ackTempStatus & 0x02) == 0x02)
	{
		//printk("\n--%s--error in _I2CSendData: ACK ERROR\n",__FUNCTION__);
		return (-101);
	}
#endif
	pI2cRegDev->I2C_ISR = pI2cRegDev->I2C_ISR & (~1ul);
	if (waiteSendDelay > 0)
	{
		return (0);
	}

	printk("%s[%d]--error in _I2CSendData:timer out\n",__FUNCTION__,__LINE__);
	return (-102);
}

/*----------------------------------------------------------------------
Name	  : I2CReadData(eI2C_ch_t i2cNumb, uint8 *data)
Desc	  : 读单个数据
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			data：输入要接收数据的地址
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CReadData(eI2C_ch_t i2cNumb, uint8 *data)
{
	int32 waiteReadDelay;
	int32 i2cIntStatus;
	pI2C_REG  pI2cRegDev;

	//return I2CReadData( data);
	
	if(i2cNumb == 0)
	{
		waiteReadDelay = 20000 / g_i2cSpeed0;
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;
	}
	else
	{
		waiteReadDelay = 20000 / g_i2cSpeed1;
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
	}

	pI2cRegDev->I2C_LCMR = pI2cRegDev->I2C_LCMR | I2C_LCMR_RESUME;
	while (waiteReadDelay > 0)
	{
		i2cIntStatus = pI2cRegDev->I2C_ISR;
		if ((i2cIntStatus & I2C_ARBITR_LOSE_STATUS) != 0)
		{
			pI2cRegDev->I2C_ISR = 0;
			//_I2CStop(i2cNumb);
			printk("%s[%d]--error in _I2CReadData:I2C_ARBITR_LOSE_STATUS\n",__FUNCTION__,__LINE__);
			return (-1);
		}
		if ((i2cIntStatus & I2C_RECE_INT_MACKP) != 0)
		{
			break;
		}
		udelay(200);
		waiteReadDelay--;
	}

	*data = (uint8)pI2cRegDev->I2C_MRXR;
	pI2cRegDev->I2C_ISR = pI2cRegDev->I2C_ISR & (~(1ul<<1));
	if (waiteReadDelay > 0)
	{
		return (0);
	}
	printk("%s[%d]--error in _I2CReadData: time out\n",__FUNCTION__,__LINE__);
	return (-1);
}

/*----------------------------------------------------------------------
Name	  : I2CRead(eI2C_ch_t i2cNumb, uint16 I2CSlaveAddr, uint16 regAddr, void *pdataBuff, uint16 size, eI2C_Address_Reg_t addressBit, eI2C_mode_t mode)
Desc	  : 读多个数据，开始时并发送地址。
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			I2CSlaveAddr：设备的地址
			regAddr：I2C设备内寄存器地址
			pData：接收数据的指针
			size：接收数据的大小
			addressBit: addressBit: 7位或10位设备地址和8位或16位寄存器地址。 例：I2C_10BIT_ADDRESS_8BIT_REG  表示设备地址是7位的，寄存器地址是8位的
			mode：读的模式 NormalMode/DirectMode
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CRead(struct i2c_client *client,uint16 SlaveAddr,eI2C_ch_t i2cNumb,uint16 regAddr, void *pdataBuff, uint16 size)
{
	int32 ret=0;
	uint8 *pdata;
	pI2C_REG  pI2cRegDev;
	uint16 I2CSlaveAddr;
	uint16 speed;
	eI2C_Address_Reg_t addressBit;
	eI2C_mode_t mode;
	uint32 APBnKHz;
	I2CSlaveAddr = SlaveAddr<<1;//adap->deviceaddr; //g_i2cSlaveAddr0;
	if(client)
	{
			addressBit	 = client->addressBit;	//g_i2cAddressBit0;
			mode = client->mode;
			i2cNumb=(eI2C_ch_t)client->Channel;
			speed = client->speed;
	}
	else
	{
			addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
			mode=NORMALMODE;
			i2cNumb=I2C_CH0;
	}	
	
#if 1
	/*set i2c host speed */
	speed = client->speed;
	if(!speed)
	{
		if(i2cNumb == I2C_CH0)
			speed = RK28_I2C0_SPEED;
		else
			speed = RK28_I2C1_SPEED;
	}
		APBnKHz=rockchip_clk_get_apb()/1000;
		if (_I2CSetSpeed(i2cNumb, speed,APBnKHz) != 0) //set I2C speed
		{
			printk("%s[%d] set  i2c speed error\n",__FUNCTION__,__LINE__);
			return (-1);
		}
#endif 
	if(i2cNumb == 0)
	{
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;

	}
	else
	{
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
	}	
	DBG("%s[%d] the slave i2c addr=0x%x  Channel =%d mode=%d\n",__FUNCTION__,__LINE__,I2CSlaveAddr,i2cNumb,mode);
//	printk(" the slave i2c addr=%x mode = %d\n",I2CSlaveAddr,mode);
	
	pdata = (uint8 *)pdataBuff;
	pI2cRegDev->I2C_LCMR = 0;
	pI2cRegDev->I2C_ISR = 0;
	pI2cRegDev->I2C_IER = 0;  ///g_pI2cReg->I2C_IER | I2C_ARBITR_LOSE_ENABLE;
	pI2cRegDev->I2C_CONR = pI2cRegDev->I2C_CONR | I2C_MASTER_TRAN_MODE | I2C_MASTER_PORT_ENABLE;
	//	I2CStart();
	if (mode == NORMALMODE)
	{
		if ((addressBit  == I2C_10BIT_ADDRESS_16BIT_REG) || (addressBit  == I2C_10BIT_ADDRESS_8BIT_REG)) ///10bit device address
		{
			if (_I2CSendData(i2cNumb, ((I2CSlaveAddr >> 7) & 0x06) | 0xf0 | I2C_WRITE_BIT, START_BIT) != 0)
			{
				ret = -1;
				printk("%s[%d]--NORMALMODE-send slave 10 bit address start_bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
			if (_I2CSendData(i2cNumb, (I2CSlaveAddr & 0xff) | I2C_WRITE_BIT, NO_START_BIT) != 0)
			{
				ret = -2;
				printk("%s[%d]--NORMALMODE-send slave 10 bit address no_start_bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}			
		}
		else  ///7bit device address
		{
			if ((ret=_I2CSendData(i2cNumb, I2CSlaveAddr | I2C_WRITE_BIT, START_BIT)) != 0)
			{
				//ret = -3;
				printk("%s[%d]--NORMALMODE-send slave 7 bit address start_bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
		}
	
		if ((addressBit == I2C_10BIT_ADDRESS_8BIT_REG) || (addressBit == I2C_7BIT_ADDRESS_8BIT_REG))///8 bit address
		{
			if (_I2CSendData(i2cNumb, regAddr, NO_START_BIT) != 0)
			{
				ret = -4;
				printk("%s[%d]--NORMALMODE-send 8 bit  register address !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
		}
		else  ///16 bit address
		{
			if (_I2CSendData(i2cNumb, regAddr>>8, NO_START_BIT) != 0)
			{
				ret = -5;
				printk("%s[%d]--NORMALMODE-send 16 bit  register address low 8 bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
			if (_I2CSendData(i2cNumb, regAddr & 0xff, NO_START_BIT) != 0)
			{
				ret = -6;
				printk("%s[%d]--NORMALMODE-send 16 bit  register address hight 8bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
		}
		_I2CStop(i2cNumb);
		if ((addressBit  == I2C_10BIT_ADDRESS_16BIT_REG) || (addressBit  == I2C_10BIT_ADDRESS_8BIT_REG)) ///10bit device address
		{
			if (_I2CSendData(i2cNumb, ((I2CSlaveAddr >> 7) & 0x06) | 0xf0 | I2C_READ_BIT, START_BIT) != 0)
			{
				ret = -7;
				goto ENDI2CREAD;
			}
		}
		else  ///7bit device address
		{
			if (_I2CSendData(i2cNumb, I2CSlaveAddr | I2C_READ_BIT, START_BIT) != 0)
			{
				ret = -8;
				goto ENDI2CREAD;
			}
		}
	}
	else if (mode == NORMALNOSTOPMODE)
	{
		if ((addressBit  == I2C_10BIT_ADDRESS_16BIT_REG) || (addressBit  == I2C_10BIT_ADDRESS_8BIT_REG)) ///10bit device address
		{
			if (_I2CSendData(i2cNumb, ((I2CSlaveAddr >> 7) & 0x06) | 0xf0 | I2C_WRITE_BIT, START_BIT) != 0)
			{
				ret = -1;
				printk("%s[%d]--NORMALMODE-send slave 10 bit address start_bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
			if (_I2CSendData(i2cNumb, (I2CSlaveAddr & 0xff) | I2C_WRITE_BIT, NO_START_BIT) != 0)
			{
				ret = -2;
				printk("%s[%d]--NORMALMODE-send slave 10 bit address no_start_bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}			
		}
		else  ///7bit device address
		{
			if ((ret=_I2CSendData(i2cNumb, I2CSlaveAddr | I2C_WRITE_BIT, START_BIT)) != 0)
			{
				//ret = -3;
				printk("%s[%d]--NORMALMODE-send slave 7 bit address start_bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
		}
	
		if ((addressBit == I2C_10BIT_ADDRESS_8BIT_REG) || (addressBit == I2C_7BIT_ADDRESS_8BIT_REG))///8 bit address
		{
			if (_I2CSendData(i2cNumb, regAddr, NO_START_BIT) != 0)
			{
				ret = -4;
				printk("%s[%d]--NORMALMODE-send 8 bit  register address !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
		}
		else  ///16 bit address
		{
			if (_I2CSendData(i2cNumb, regAddr>>8, NO_START_BIT) != 0)
			{
				ret = -5;
				printk("%s[%d]--NORMALMODE-send 16 bit  register address low 8 bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
			if (_I2CSendData(i2cNumb, regAddr & 0xff, NO_START_BIT) != 0)
			{
				ret = -6;
				printk("%s[%d]--NORMALMODE-send 16 bit  register address hight 8bit !\n",__FUNCTION__,__LINE__);
				goto ENDI2CREAD;
			}
		}

		if ((addressBit  == I2C_10BIT_ADDRESS_16BIT_REG) || (addressBit  == I2C_10BIT_ADDRESS_8BIT_REG)) ///10bit device address
		{
			if (_I2CSendData(i2cNumb, ((I2CSlaveAddr >> 7) & 0x06) | 0xf0 | I2C_READ_BIT, START_BIT) != 0)
			{
				ret = -7;
				goto ENDI2CREAD;
			}
		}
		else  ///7bit device address
		{
			if (_I2CSendData(i2cNumb, I2CSlaveAddr | I2C_READ_BIT, START_BIT) != 0)
			{
				ret = -8;
				goto ENDI2CREAD;
			}
		}
	}
	else if ((mode == DIRECTMODE)||(mode == TS8205MODE)) // FM5767
	{
		if (_I2CSendData(i2cNumb,I2CSlaveAddr | I2C_READ_BIT, START_BIT) != 0)
		{
			ret = -9;
			printk("%s[%d]--DIRECTMODE/TS8205MODE-send slave address !\n",__FUNCTION__,__LINE__);
			goto ENDI2CREAD;
		}
	}
	else if(mode == RegisterMode)
	{ 
		if(_I2CSendData(i2cNumb,I2CSlaveAddr | I2C_READ_BIT,START_BIT) != 0)
		{
			ret = -10;
			printk("%s[%d]--RegisterMode-send slave address start_bit !\n",__FUNCTION__,__LINE__);
			goto ENDI2CREAD;
		}

		if(_I2CSendData(i2cNumb,regAddr,NO_START_BIT) != 0)
		{
			ret = -11;
			printk("%s[%d]--RegisterMode-send slave address no_start_bit!\n",__FUNCTION__,__LINE__);
			goto ENDI2CREAD;
		}
	}
	else if(mode == PCA955XMode)
	{ 
		if(_I2CSendDataMode(i2cNumb,I2CSlaveAddr | I2C_READ_BIT,START_BIT) != 0)
		{
			ret = -10;
			DBG("%s[%d]--RegisterMode-send slave address start_bit !\n",__FUNCTION__,__LINE__);
			goto ENDI2CREAD;
		}

		if(_I2CSendDataMode(i2cNumb,regAddr,NO_START_BIT) != 0)
		{
			ret = -11;
			DBG("%s[%d]--RegisterMode-send slave address no_start_bit!\n",__FUNCTION__,__LINE__);
			goto ENDI2CREAD;
		}
	}
	pI2cRegDev->I2C_CONR = (pI2cRegDev->I2C_CONR &(~(1ul<<3))) | I2C_MASTER_RECE_MODE | I2C_MASTER_PORT_ENABLE;

	do
	{
		if (_I2CReadData(i2cNumb, pdata) != 0)
		{
			ret = -12;
			printk("%s[%d]--read data error!\n",__FUNCTION__,__LINE__);
			goto ENDI2CREAD;
		}
		if (size == 1)
		{
			pI2cRegDev->I2C_CONR = pI2cRegDev->I2C_CONR | I2C_CON_NACK;
		}
		else
		{
			pI2cRegDev->I2C_CONR = (pI2cRegDev->I2C_CONR &(~(1ul<<4))) | I2C_CON_ACK;
		}

		pdata++;
		size--;
	}
	while (size);

ENDI2CREAD:

	_I2CStop(i2cNumb);

	return (ret);
}

/*----------------------------------------------------------------------
Name	  : I2CWrite(eI2C_ch_t i2cNumb, uint16 I2CSlaveAddr, uint16 regAddr, void *pdataBuff, uint16 size, eI2C_Address_Reg_t addressBit, eI2C_mode_t mode)
Desc	  : 写多个数据，开始时并发送地址。
Params	  : 
			i2cNumb：I2C设备，为 I2C_CH0或I2C_CH1
			I2CSlaveAddr：设备的地址
			regAddr：I2C设备内寄存器地址
			pData：发送数据的指针
			size：发送数据的大小
			addressBit: 7位或10位设备地址和8位或16位寄存器地址。 例：I2C_7BIT_ADDRESS_8BIT_REG	表示设备地址是7位的，寄存器地址是8位的
			mode：写的模式 NormalMode/DirectMode
Return	  :成功还回0，失败还回-1。
----------------------------------------------------------------------*/
int32 _I2CWrite(struct i2c_client *client,uint16 SlaveAddr,eI2C_ch_t i2cNumb,uint16 regAddr, void *pdataBuff, uint16 size)
{
	int32 ret=0;
	uint8 *pdata;
	pI2C_REG  pI2cRegDev;
	uint16 I2CSlaveAddr;
	eI2C_Address_Reg_t addressBit;
	eI2C_mode_t mode;
	uint32 APBnKHz;
	uint16 speed;
	I2CSlaveAddr = SlaveAddr<<1;//adap->deviceaddr; //g_i2cSlaveAddr0;

	if(client)
	{
		addressBit	 = client->addressBit;	//g_i2cAddressBit0;
		mode = client->mode;
		i2cNumb=(eI2C_ch_t)client->Channel;
		speed = client->speed;	
	}
	else
	{
		addressBit=I2C_7BIT_ADDRESS_8BIT_REG;
		mode=NORMALMODE;
		i2cNumb=0;
	}
#if 1
	/*set i2c host speed */
	speed = client->speed;
	if(!speed)
	{
		if(i2cNumb == I2C_CH0)
			speed = RK28_I2C0_SPEED;
		else
			speed = RK28_I2C1_SPEED;
	}
	APBnKHz=rockchip_clk_get_apb()/1000;
	if (_I2CSetSpeed(i2cNumb, speed,APBnKHz) != 0) //set I2C speed
	{
		printk("%s[%d] set  i2c speed error\n",__FUNCTION__,__LINE__);
		return (-1);
	}
#endif 
	if(i2cNumb == 0)
	{
		pI2cRegDev = (pI2C_REG) I2C0_BASE_ADDR_VA;
	
	}
	else
	{
		pI2cRegDev = (pI2C_REG) I2C1_BASE_ADDR_VA;
	}
	DBG("%s[%d] the slave i2c addr=0x%x  Channel =%d mode=%d size=%d\n",__FUNCTION__,__LINE__,I2CSlaveAddr,i2cNumb,mode,size);
	pdata = (uint8 *) pdataBuff;
	pI2cRegDev->I2C_LCMR = 0;
	pI2cRegDev->I2C_ISR = 0;
	pI2cRegDev->I2C_IER = 0;  ///g_pI2cReg->I2C_IER | I2C_ARBITR_LOSE_ENABLE;
	pI2cRegDev->I2C_CONR = (pI2cRegDev->I2C_CONR & (~(1ul<<4))) | I2C_MASTER_TRAN_MODE | I2C_MASTER_PORT_ENABLE;
	// I2CStart();
	if ((addressBit  == I2C_10BIT_ADDRESS_16BIT_REG) || (addressBit  == I2C_10BIT_ADDRESS_8BIT_REG)) ///10bit device address
	{
		if (_I2CSendData(i2cNumb, ((I2CSlaveAddr >> 7) & 0x06) | 0xf0 | I2C_WRITE_BIT, START_BIT) != 0)
		{
			ret = -1;
			printk("%s[%d]---send slave 10 bit address start_bit !\n",__FUNCTION__,__LINE__);
			goto ENDI2CWRITE;
		}
		if (_I2CSendData(i2cNumb, (I2CSlaveAddr & 0xff) | I2C_WRITE_BIT, NO_START_BIT) != 0)
		{
			ret = -2;
			printk("%s[%d]---send slave 10 bit address no_start_bit !\n",__FUNCTION__,__LINE__);
			goto ENDI2CWRITE;
		}
	}
	else ///7bit device address
	{
		if (_I2CSendData(i2cNumb, I2CSlaveAddr | I2C_WRITE_BIT, START_BIT) != 0)
		{
			ret = -3;
			printk("%s[%d]---send slave 7 bit address start_bit !\n",__FUNCTION__,__LINE__);
			goto ENDI2CWRITE;
		}
	}

	if (mode == NORMALMODE || mode == NORMALNOSTOPMODE)
	{
		if ((addressBit == I2C_10BIT_ADDRESS_8BIT_REG) || (addressBit == I2C_7BIT_ADDRESS_8BIT_REG))///8 bit address
		{
			if (_I2CSendData(i2cNumb, regAddr, NO_START_BIT) != 0)
			{
				ret = -4;
				printk("%s[%d]--NORMALMODE-send 8bit register address no_start_bit!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
		}
		else  ///16 bit address
		{
			if (_I2CSendData(i2cNumb, regAddr>>8, NO_START_BIT) != 0)
			{
				ret = -5;
				printk("%s[%d]--NORMALMODE-send low 8 bit register address no_start_bit!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
			if (_I2CSendData(i2cNumb, regAddr & 0xff, NO_START_BIT) != 0)
			{
				ret = -6;
				printk("%s[%d]--NORMALMODE-send high 8 bit register address no_start_bit!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
		}
	}
	if (mode == TS8205MODE)
	{
			if (_I2CSendData(i2cNumb, regAddr, NO_START_BIT) != 0)
			{
				ret = -4;
				printk("%s[%d]--TS8205MODE--send 8bit register address no_start_bit!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
	}
	if (mode == PCA955XMode)
	{
			if (_I2CSendDataMode(i2cNumb, regAddr, NO_START_BIT) != 0)
			{
				ret = -4;
				DBG("%s[%d]--TS8205MODE--send 8bit register address no_start_bit!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
	}

	do
	{
		if(mode == TS8205MODE)
		{
			if (_I2CSendDataMode(i2cNumb, *pdata, NO_START_BIT) != 0)
			{
				ret = -7;
				printk("%s[%d]--TS8205MODE-write data error!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
			
		}
		else if(mode == PCA955XMode)
		{
			if (_I2CSendDataMode(i2cNumb, *pdata, NO_START_BIT) != 0)
			{
				ret = -7;
				DBG("%s[%d] %d--write data error!\n",__FUNCTION__,__LINE__,*pdata);
				goto ENDI2CWRITE;
			}
			
		}
		else
		{
			if (_I2CSendData(i2cNumb, *pdata, NO_START_BIT) != 0)
			{
				ret = -7;
				printk("%s[%d]--write data error!\n",__FUNCTION__,__LINE__);
				goto ENDI2CWRITE;
			}
		}
		pdata++;
		size--;
	}
	while (size);

ENDI2CWRITE:

	pI2cRegDev->I2C_CONR = pI2cRegDev->I2C_CONR | I2C_CON_NACK; //// Set CONR NACK	  释放SDA总线
	_I2CStop(i2cNumb);

	return (ret);
}


void __devinit _i2c0_hwinit(void)
{
    rockchip_scu_apbunit_register( SCU_IPID_I2C0 , "i2c0" , (_I2CUpdateAllApbFreq) ); /* for apb not div unit register */
    #if I2C1_DEV_OPEN
    rockchip_scu_apbunit_register( SCU_IPID_I2C1 , "i2c1" , (_I2CUpdateAllApbFreq) ); /* for apb not div unit register */
    #endif   
	_I2CInit(I2C_CH0, RK28_I2C0_SPEED,0xff,I2C_7BIT_ADDRESS_8BIT_REG, NORMALMODE);
	#if I2C1_DEV_OPEN
	_I2CInit(I2C_CH1, RK28_I2C1_SPEED,0xff,I2C_7BIT_ADDRESS_8BIT_REG, NORMALMODE);
	#endif 
 	
}

/*----------------------------------------------------------------------
Name	:	_i2c_getClient_byaddr
Desc	:	由地址获取到设备client
Params	:	adap:总线adapter
			addr:设备地址
Return	:	设备client
Notes	:	注意事项
----------------------------------------------------------------------*/
struct i2c_client* _i2c_getClient_byaddr(struct i2c_adapter *adap,uint16 addr)
{
	struct list_head  *item, *_n;
	struct i2c_client *client;

	list_for_each_safe(item, _n, &adap->clients) 
		{
		client = list_entry(item, struct i2c_client, list);

		if(client->addr==addr)
			return client;
		}

		return NULL;
   

}



/*----------------------------------------------------------------------
Name	:	i2c_check_clientdate
Desc	:	检查校正设备client数据
			设备client
Notes	:	注意事项
----------------------------------------------------------------------*/
void i2c_check_clientdate(struct i2c_client *client)
{

	if(client->addressBit>I2C_7BIT_ADDRESS_16BIT_REG)
		client->addressBit=I2C_7BIT_ADDRESS_8BIT_REG;

	if(client->mode>RegisterMode)
		client->mode=NORMALMODE;
	
	if(client->Channel>1)
		client->Channel=0;

}




/*----------------------------------------------------------------------
Name	:	rock28_i2c_transfer
Desc	:	I2C标准传输函数
Params	:	adap:总线adapter
Return	:	
Notes	:	注意事项
----------------------------------------------------------------------*/
static int rock28_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{	
	int ret;
	struct i2c_client *client;

	
	client=_i2c_getClient_byaddr(adap,pmsg->addr);
	
	//DBG("\n**********rock28_i2c_transfer*****");
	//DBG(" device addr=0x%x",pmsg->addr);
	//DBG(" regist addr=0x%x",pmsg->buf[0]);
	//DBG(" size =0x%x,flag=%d\n",pmsg->len,(pmsg->flags));

	if(pmsg->len==0)
		return 0;
	//rk28_i2c_hwinit();
	if(pmsg->flags)
	{
		//spin_lock(&rk28i2c_lock);
		//mutex_lock(&i2c_rw_mutex);
                if(client->addressBit == I2C_7BIT_ADDRESS_16BIT_REG){
		   ret=_I2CRead(client,pmsg->addr,I2C_CH0,((pmsg->buf[0]<<8) | (pmsg->buf[1])), pmsg->buf, 1); 
                }
                else{ 
		   ret=_I2CRead(client,pmsg->addr,I2C_CH0,pmsg->buf[0], pmsg->buf, pmsg->len);
                }
		//spin_unlock(&rk28i2c_lock);
		//mutex_unlock(&i2c_rw_mutex);
		if(ret<0)
		{
			printk("%s[%d] ERROR :read date in rock28 i2c,ret=%d,date=%x\n",__FUNCTION__,__LINE__,ret,pmsg->buf[0]);
			return ret;
		}
		DBG("%s[%d] transfer get date =0x%x,0x%x\n",__FUNCTION__,__LINE__,pmsg->buf[0],pmsg->buf[1]);		
	}
	else
	{
	
		//spin_lock(&rk28i2c_lock);
		//mutex_lock(&i2c_rw_mutex);
                if(client->addressBit == I2C_7BIT_ADDRESS_16BIT_REG){
		   ret=_I2CWrite(client,pmsg->addr, I2C_CH0,((pmsg->buf[0]<<8) | (pmsg->buf[1])),&(pmsg->buf[2]), pmsg->len-2);
                }
                else{
		   ret=_I2CWrite(client,pmsg->addr, I2C_CH0,pmsg->buf[0] ,&(pmsg->buf[1]), pmsg->len-1);
                }
		//spin_unlock(&rk28i2c_lock);
		//mutex_unlock(&i2c_rw_mutex);
		if(ret<0)
		{
			printk("%s[%d] ERROR :write date in rock28 i2c,ret=%d,date=%x\n",__FUNCTION__,__LINE__,ret,pmsg->buf[0]);
			return ret;
		}
		DBG("%s[%d] transfer write date =0x%x,0x%x\n",__FUNCTION__,__LINE__,pmsg->buf[0],pmsg->buf[1]);	
	}


	//_I2CDeInit(I2C_CH0);




	return pmsg->len;

}


//add by zhaojun before
//////////////////////////////////////////
















static void __devinit rk28_i2c_hwinit(void)
{
	unsigned long r1,r2,r4;//cdiv, ckdiv;
	//DBG("\nrk28_i2c_hwinit    ---------------------------##Yong##\n");
	//sba=__raw_readl(SCU_BASE_ADDR_VA+0x1c);
	
	//__raw_writel(~(1<<20 )&sba,SCU_BASE_ADDR_VA+0x1c);
	//p243,i2c0 clock enable when high,disable clock
	
	
	rk28_i2c_write(RK28_I2C_OPR, RESET_I2C_STATUS);	/* Reset peripheral */
	r2=rk28_i2c_read(RK28_I2C_OPR);
	udelay(10);
	rk28_i2c_write(RK28_I2C_OPR, r2&(0x01ul<<7));
	r4=rk28_i2c_read(RK28_I2C_OPR);
	rk28_i2c_write(RK28_I2C_OPR,  r4 |(2<<3) | 4);
	//r4=rk28_i2c_read(RK28_I2C_OPR);
	
	rk28_i2c_write(RK28_I2C_IER, 0x00000000);	/* Disable all interrupts */
	//r1=rk28_i2c_read(RK28_I2C_IER);
	rk28_i2c_write(RK28_I2C_LCMR, 0x00000000);
	
	r1=rk28_i2c_read(RK28_I2C_OPR);
	rk28_i2c_write(RK28_I2C_OPR, r1|I2C_CORE_ENABLE);//enable I2C controller	

	//I2CSetSpeed();		//set I2C speed
}


static void __devinit rk28_i2c1_hwinit(void)
{
	#if I2C1_DEV_OPEN
	unsigned long r1,r2,r4;//cdiv, ckdiv;
	//DBG("\nrk28_i2c_hwinit	 ---------------------------##Yong##\n");

	
	//sba=__raw_readl(SCU_BASE_ADDR_VA+0x1c);	
	//__raw_writel(~(1<<21 )&sba,SCU_BASE_ADDR_VA+0x1c);
	
	rk28_i2c1_write(RK28_I2C_OPR, RESET_I2C_STATUS); /* Reset peripheral */
	r2=rk28_i2c1_read(RK28_I2C_OPR);
	udelay(10);
	rk28_i2c1_write(RK28_I2C_OPR, r2&(0x01ul<<7));
	r4=rk28_i2c1_read(RK28_I2C_OPR);
	rk28_i2c1_write(RK28_I2C_OPR,  r4 |(2<<3) | 4);
	//r4=rk28_i2c_read(RK28_I2C_OPR);
	
	rk28_i2c1_write(RK28_I2C_IER, 0x00000000);	/* Disable all interrupts */
	//r1=rk28_i2c_read(RK28_I2C_IER);
	rk28_i2c1_write(RK28_I2C_LCMR, 0x00000000);
	
	r1=rk28_i2c1_read(RK28_I2C_OPR);
	rk28_i2c1_write(RK28_I2C_OPR, r1|I2C_CORE_ENABLE);//enable I2C controller	
    #endif
	//I2CSetSpeed();		//set I2C speed
}



/*
 * Return list of supported functionality.
 */
static u32 rk28_func(struct i2c_adapter *adapter)
{
	//DBG("---->%s..%s..:%i  rk28_func \n",__FILE__,__FUNCTION__,__LINE__);
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm rk28_algorithm = {
	.master_xfer	= rock28_i2c_transfer,   //rock28_i2c_transfer,		//rk28_xfer,
	.functionality	= rk28_func,
};

/*
 * Main initialization routine.
 */
static int __devinit rk28_i2c_probe(struct platform_device *pdev)
{
	struct i2c_adapter *adapter;
	struct resource *res;
	int rc;
	mutex_init(&i2c_rw_mutex);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res){
     		printk("platform_get_resource failed\n");
		return -ENXIO;
       }
	
  	if (!request_mem_region(res->start, res->end - res->start + 1, "rk28_i2c"))
		return -EBUSY;
	adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);

	if (adapter == NULL) {
		dev_err(&pdev->dev, "can't allocate inteface!\n");
		rc = -ENOMEM;
		goto fail2;
	}
	sprintf(adapter->name, "rk28_i2c");
	adapter->algo = &rk28_algorithm;
	adapter->class = I2C_CLASS_HWMON;
	adapter->dev.parent = &pdev->dev; 
	
	platform_set_drvdata(pdev, adapter);
		

	  _i2c0_hwinit();   
	  
	rc = i2c_add_numbered_adapter(adapter);
	if (rc) {
		dev_err(&pdev->dev, "Adapter %s registration failed\n",
				adapter->name);
		goto fail3;
	}

	return 0;

fail3:
	platform_set_drvdata(pdev, NULL);
	kfree(adapter);

fail2:
	DBG("---->%s..%s..:%i\n",__FILE__,__FUNCTION__,__LINE__);
	return rc; 
}

static int __devexit rk28_i2c_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	struct resource *res;
	int rc;

	rc = i2c_del_adapter(adapter);
	platform_set_drvdata(pdev, NULL);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);
 	DBG("---->%s..%s..:%i\n",__FILE__,__FUNCTION__,__LINE__);
	return rc;
}

#ifdef CONFIG_PM

/* NOTE: could save a few mA by keeping clock off outside of at91_xfer... */

static int rk28_i2c_suspend(struct platform_device *pdev, pm_message_t mesg)
{

	return 0;
}

static int rk28_i2c_resume(struct platform_device *pdev)
{
	return 0;
}

#else
static int rk28_i2c_suspend(struct platform_device *pdev, pm_message_t mesg)	{ }

static int rk28_i2c_resume(struct platform_device *pdev)	 { }

#endif

/* work with "modprobe rk28_i2c" from hotplugging or coldplugging */
MODULE_ALIAS("rk28_i2c");

static struct platform_driver rk28_i2c_driver = {
	.probe		= rk28_i2c_probe,
	.remove		= __devexit_p(rk28_i2c_remove),
	.suspend		= rk28_i2c_suspend,
	.resume		= rk28_i2c_resume,
	.driver		= {
	.name		= "rk28_i2c",
	.owner		= THIS_MODULE,
	},
};

static int __init rk28_i2c_init(void)
{
	DBG("\n-- 1 ----##Cong##->%s..%s..:%i--> begin run platform_driver_register() : \n", __FILE__,__FUNCTION__,__LINE__);

	//iomux
	rockchip_mux_api_set(GPIOE_I2C0_SEL_NAME, IOMUXA_I2C0);
	#if I2C1_DEV_OPEN
	rockchip_mux_api_set(GPIOE_U1IR_I2C1_NAME, IOMUXA_I2C1);
	#endif
	return platform_driver_register(&rk28_i2c_driver);
}

static void __exit rk28_i2c_exit(void)
{
	platform_driver_unregister(&rk28_i2c_driver);
}

module_init(rk28_i2c_init);
module_exit(rk28_i2c_exit);




MODULE_AUTHOR("Rock ");
MODULE_DESCRIPTION("I2C (TWI) driver for rk28");
MODULE_LICENSE("GPL");
