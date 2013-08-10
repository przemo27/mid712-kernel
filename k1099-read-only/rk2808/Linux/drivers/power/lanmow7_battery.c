/* arch/arm/mach-rockchip/rk28_battery.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/mach/time.h>
#include <asm/arch/gpio.h>




//#include <linux/wakelock.h>
//#include <asm/gpio.h>

/* Debug */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

/*use adc sample battery capacity*/
#define BATT_1V2_MODIFY	100
#define BATT_1V2_VALUE		1338*BATT_1V2_MODIFY/100
#define BATT_FULL_VALUE	4200
#define BATT_STEP_FULL_VALUE  4100
#define BATT_EMPTY_VALUE	3500
#define PERCENT				100
#define BATT_LEVEL_FULL		100
#define BATT_LEVEL_EMPTY	0
#define BATT_PRESENT_TRUE	 1
#define BATT_PRESENT_FALSE  0
#define BATT_NOMAL_VOL_VALUE	4000
#define BATT_VOLTAGE_MAX	4200
#define BATT_VOLTAGE_MIN	3500
#define AD_SAMPLE_TIMES	6

#define AD_NO_BATT_VALE       200
#define AD_NO_DC_VALE         200

#define TS_POLL_DELAY		100*1000*1000
#define SEC_NUM				2  ///8
#define PER_SEC_NUM			20  ///10
#define MINS_NUM			15
#define HALF_HOUR_NUM		35*MINS_NUM

static int bat_vol_cnt = 0;  
static int bat_vol_up_cnt = 0; 
static int bat_vol_no_power_cnt = 0;  
static int bat_status =  POWER_SUPPLY_STATUS_UNKNOWN;
static int bat_health = POWER_SUPPLY_HEALTH_GOOD;
static int bat_capacity = BATT_LEVEL_EMPTY;
static int bat_present = BATT_PRESENT_TRUE;
static int bat_voltage =  BATT_NOMAL_VOL_VALUE;
static int ad_sample_current_time = 0;
unsigned int sample_times = 0;			/*count times (report batter status)*/
unsigned int bat_charge_full_times = 0;
static int  rectify_vol = 120;
//static int g_charge_full_cnt = 0;

static int batt_step_table[56]={
    3400,3420,3470,3500,3540,3545,3555,3560,3580,3600,
	3615,3630,3640,3650,3660,3670,3680,3690,3700,3710,
	3720,3730,3740,3750,3760,3770,3780,3790,3800,3810,
	3815,3830,3845,3860,3875,3890,3900,3910,3920,3930,
	3940,3950,3960,3970,3985,4000,4005,4010,4015,4020,
	4030,4040,4050,4060,4070,4200	
};
static int batt_no_current_step_table[56]={
	3410,3440,3600,3650,3680,3690,3700,3715,3725,3740,
	3745,3755,3765,3770,3778,3784,3790,3800,3810,3820,
	3835,3845,3855,3870,3880,3890,3898,3905,3913,3920,
	3930,3940,3950,3960,3970,3980,3988,3995,4002,4010,
	4018,4025,4035,4042,4051,4060,4065,4070,4075,4080,
	4085,4090,4095,4100,4105,4200
};
static int batt_disp_table[56]={
     0, 1, 5, 8,10,12,14,15,18,20,
	23,26,28,30,33,37,40,43,47,50,
	52,54,57,60,62,64,66,68,69,70,
	72,74,76,78,79,80,81,82,83,84,
	85,86,87,88,89,90,91,92,93,94,
	95,96,97,98,99,100	
};
static u16 g_adcref = 0;
static u16 g_adcbat = 0;	//adccharge;
extern	u16 get_rock_adc0(void);	/*battery capacity*/
extern	u16 get_rock_adc1(void);	/*battery charge status*/
extern	u16 get_rock_adc3(void);	/*battery vref*/
extern       int get_msc_connect_flag( void );



struct rk28_battery_data {
	spinlock_t lock;

	struct power_supply 	battery;
	struct power_supply	usb;
	struct power_supply	ac;
	struct hrtimer  timer;	
};


#define APP_BATT_PDEV_NAME		"rockchip_battery"
#define DRV_VER 				"1.0"
typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;


static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);
static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val);

static enum power_supply_property rockchip_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property rockchip_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};


static struct power_supply rockchip_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = rockchip_battery_properties,
		.num_properties = ARRAY_SIZE(rockchip_battery_properties),
		.get_property = rockchip_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_usb_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_AC,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_ac_get_property,
	},
};

static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	
	//todo 
	charger =  CHARGER_USB;
//	DBG("--------%s-->%s-->%d\n",__FILE__,__FUNCTION__,__LINE__);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_AC)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
//			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
			val->intval = GPIOGetPinLevel(GPIOPortB_Pin1) ;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;

}
static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	charger_type_t charger;
	
	//todo 
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_AC)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;

}
 static void rk28_power_off(void)
{
	GPIOSetPinLevel(GPIOPortF_Pin1,GPIO_LOW);	/*power down*/
}
static int get_battery_charge_status( void )
{
	return (GPIOGetPinLevel(GPIOPortB_Pin1) && !get_msc_connect_flag() );
}
static int rockchip_get_battery_status(void)
{
	//u16 adcref,adcbat;	//adccharge;
	int  current_vol,i;
	
	ad_sample_current_time++;
    g_adcbat = (g_adcbat + get_rock_adc0())/2;
   	g_adcref = (g_adcref + get_rock_adc3())/2;
	if(ad_sample_current_time < AD_SAMPLE_TIMES) return 1;
	ad_sample_current_time = 0;		
	if(g_adcbat < AD_NO_BATT_VALE)	/*haven't battery*/ 
	{
		bat_present = BATT_PRESENT_FALSE;	
		goto nobattery;
	}
	bat_present = BATT_PRESENT_TRUE;	/*have battery*/
	/*get charge status*/
	 if(get_battery_charge_status())	/*the battery charge*/
		bat_status =POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status =POWER_SUPPLY_STATUS_NOT_CHARGING ;	/*no charge*/

	/*get present voltage*/
	current_vol = ((u32)g_adcbat * 2UL * BATT_1V2_VALUE)/((u32)g_adcref);		/*current voltage*/
	bat_voltage = current_vol;
	/*get battery health status*/
	if(GPIOGetPinLevel(GPIOPortB_Pin1))	
	{
		current_vol-=rectify_vol;
		bat_voltage = current_vol;
		if(batt_no_current_step_table[0]>=current_vol)
		{
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =  1;	/* assure power on*/
			return 1;
		}
		
	}
	else{
		if(batt_step_table[0]>=current_vol)
		{
			bat_vol_no_power_cnt++;
			if(bat_vol_no_power_cnt<80){
			    bat_capacity = 1;
			    return 1;
			}
			bat_vol_no_power_cnt = 0;
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =	0;   ///9;
			return 1;
		}
		else if(batt_step_table[55]<=current_vol)
		{
			bat_health = POWER_SUPPLY_HEALTH_GOOD;
			bat_vol_no_power_cnt = 0;
			bat_capacity =  BATT_LEVEL_FULL;
			return 1;
		}
	}
	bat_vol_no_power_cnt = 0;
	if(GPIOGetPinLevel(GPIOPortB_Pin1)){
		for(i=0; i<56; i++){		
		    if((batt_no_current_step_table[i]<=current_vol)&&(batt_no_current_step_table[i+1]>current_vol))break;		
	    }
	}else{
	    for(i=0; i<56; i++){		
		    if((batt_step_table[i]<=current_vol)&&(batt_step_table[i+1]>current_vol))break;		
	    }
	}
	bat_capacity = batt_disp_table[i];
	bat_health = POWER_SUPPLY_HEALTH_GOOD;
	return 1;
nobattery:
	printk("Current no battery exist!!\n");
	if(GPIOGetPinLevel(GPIOPortB_Pin1) && !get_msc_connect_flag() )	/*the battery charge*/
		bat_status =POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status =POWER_SUPPLY_STATUS_NOT_CHARGING ;	/*no charge*/
	bat_health = POWER_SUPPLY_HEALTH_GOOD;
	/*haven't battery  but have usb supply*/
	if((GPIOGetPinLevel(GPIOPortB_Pin1))&&(bat_capacity  < 15))	
		bat_capacity = 16;
	return 0;

}

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	//DBG("--------%s-->%s-->property_psp%d\n",__FILE__,__FUNCTION__,psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* get power supply */
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* Todo return battery level */	
		val->intval = bat_capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val ->intval = bat_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = BATT_VOLTAGE_MAX;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = BATT_VOLTAGE_MIN;
		break;
	default:		
		return -EINVAL;
	}
	
	return 0;
}


 static enum hrtimer_restart rk28_battery_dostimer(struct hrtimer *handle)
{

	struct rk28_battery_data *data = container_of(handle, struct rk28_battery_data, timer);
	int old_bat_status = bat_status;
	int old_bat_capacity = bat_capacity;
	unsigned long flags;
	spin_lock_irqsave(&data->lock, flags);
	rockchip_get_battery_status();		/*have battery*/
	/*if have usb supply power*/		
	if((bat_present == BATT_PRESENT_TRUE)&&(old_bat_status != bat_status))
	{
		/*set charge status*/
		if(GPIOGetPinLevel(GPIOPortB_Pin1) && !get_msc_connect_flag())	/*the battery charge*/
	 		GPIOSetPinLevel(GPIOPortB_Pin0,GPIO_HIGH);
		else 
			GPIOSetPinLevel(GPIOPortB_Pin0,GPIO_LOW);	/*no charge*/
		goto next;
	}
	/*fine set battery capacity*/
	if((GPIOGetPinLevel(GPIOPortB_Pin1))&&(bat_capacity < old_bat_capacity)){	
		if((old_bat_capacity-bat_capacity)<10){
		    bat_capacity = old_bat_capacity;
		    bat_vol_up_cnt = 0;
		}else{
		    bat_vol_up_cnt++;
			if(bat_vol_up_cnt > 20)
			    bat_vol_up_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
	    }
	}	
	if((GPIOGetPinLevel(GPIOPortB_Pin1)==0)&&(bat_capacity > old_bat_capacity)){		
		if((bat_capacity-old_bat_capacity)<10){
			bat_capacity = old_bat_capacity;
			bat_vol_cnt = 0;
		}else{
			bat_vol_cnt++;
			if(bat_vol_cnt > 80)
			    bat_vol_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
		}	
	}
	#if 0
	bat_status =  POWER_SUPPLY_STATUS_NOT_CHARGING;
	bat_health = POWER_SUPPLY_HEALTH_GOOD;
	bat_capacity = 20;
	bat_present = BATT_PRESENT_TRUE;
	#endif
	sample_times ++;						/*count times (report batter status)*/
	if((bat_present == BATT_PRESENT_TRUE)&&(sample_times > SEC_NUM * PER_SEC_NUM) )
	{
		sample_times = 0;
		bat_charge_full_times++;	/*As want full battery must charge more than half hour after 4.0v*/
next:
		if((bat_voltage<=BATT_STEP_FULL_VALUE))
			bat_charge_full_times=0;
		if((GPIOGetPinLevel(GPIOPortB_Pin1))&&(bat_voltage>=BATT_STEP_FULL_VALUE)&&(bat_charge_full_times >= HALF_HOUR_NUM))
		{	
			DBG("Enter charge battery full jurge\n");
			bat_charge_full_times = HALF_HOUR_NUM;
			bat_status = POWER_SUPPLY_STATUS_FULL;
			bat_capacity = 100;
		}
		DBG("\n****>battery adcbat = %d adcref=%d\n",g_adcbat,g_adcref);
		DBG("---->battery present = %d\n",bat_present);
		DBG("---->battery status  = %d\n",bat_status);
		DBG("---->pb1 status = %d  usbchargestatus== %d\n",GPIOGetPinLevel(GPIOPortB_Pin1),usbchargestatus);
		DBG("---->battery current voltage = %d\n",bat_voltage);
		DBG("---->battery capacity = %d\n",bat_capacity);
		DBG("---->bat_charge_full_times==%d",bat_charge_full_times);
		power_supply_changed(&data->battery);
	}
	spin_unlock_irqrestore(&data->lock, flags);
	handle->expires = ktime_add(handle->expires, ktime_set(0,TS_POLL_DELAY));
	return HRTIMER_RESTART;

 }
static int rockchip_battery_probe(struct platform_device *pdev)
{
	int  rc,i;
	struct rk28_battery_data  *data;
	
	DBG("RockChip battery driver %s\n",DRV_VER);
	/* init power supplier framework */
	data=kzalloc(sizeof(*data), GFP_KERNEL);
	spin_lock_init(&data->lock);
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rk28_battery_dostimer;
	data ->battery = rockchip_power_supplies[0];
	data ->usb	  = rockchip_power_supplies[1];
	data ->ac        = rockchip_power_supplies[2];
	DBG("test %s-->%s-->%s\n",data->battery.name,data->usb.name,data->ac.name);
	rc = power_supply_register(&pdev->dev, &data ->battery);
	if (rc)
	{
		printk(KERN_ERR "Failed to register battery power supply (%d)\n", rc);
		goto err_battery_fail;
	}

	rc = power_supply_register(&pdev->dev, &data ->usb);
	if (rc)
	{
		printk(KERN_ERR "Failed to register usb power supply (%d)\n", rc);
		goto err_usb_fail;
	}
    bat_vol_no_power_cnt = 81;
	/*get originally battery stauts*/
	for(i=0;i<AD_SAMPLE_TIMES;i++)
	{
		rockchip_get_battery_status( );	
		mdelay(15);
	}
	/*low battery low need power down*/
	DBG("---->battery adcbat = %d adcref=%d\n",g_adcbat,g_adcref);
	DBG("---->battery present = %d\n",bat_present);
	DBG("---->battery status  = %d\n",bat_status);
	DBG("---->pb1 status = %d  usbchargestatus== %d\n",GPIOGetPinLevel(GPIOPortB_Pin1),!get_msc_connect_flag());
	DBG("---->battery current voltage = %d\n",bat_voltage);
	DBG("---->battery capacity = %d\n",bat_capacity);
	if((bat_capacity <14 )&&(GPIOGetPinLevel(GPIOPortB_Pin1)))
		bat_capacity = 16;
	if((bat_capacity == 0 )&&(GPIOGetPinLevel(GPIOPortB_Pin1) == 0))
		 rk28_power_off();
	hrtimer_start(&data->timer,ktime_set(10,TS_POLL_DELAY),HRTIMER_MODE_REL);
	
	return 0;
err_battery_fail:
	power_supply_unregister(&data->battery);
	
err_usb_fail:
	power_supply_unregister(&data->usb);
#if 0
err_ac_fail:
	power_supply_unregister(&data->ac);
#endif 
	return rc;
		
}

static struct platform_driver rockchip_battery_driver = {
	.probe	= rockchip_battery_probe,
	.driver	= {
		.name	= APP_BATT_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};


static int __init rockchip_battery_init(void)
{
	printk("%s::========================================\n",__func__);
	platform_driver_register(&rockchip_battery_driver);
	return 0;
}
module_init(rockchip_battery_init);
MODULE_DESCRIPTION("Rockchip Battery Driver");
MODULE_LICENSE("GPL");
