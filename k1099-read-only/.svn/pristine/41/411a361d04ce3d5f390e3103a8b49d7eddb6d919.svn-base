/*
 * wifi_power.c
 *
 * Power control for WIFI module.
 *
 * There are Power supply and Power Up/Down controls for WIFI typically.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include "wifi_power.h"

#if (WIFI_GPIO_POWER_CONTROL == 1)

#define BOARD_ANDROID_RK2800SDK		0
#ifdef CONFIG_MACH_RK2808SDK
//#define BOARD_ANDROID_RK2808SDK		1
#else
//#define BOARD_ANDROID_RK2808SDK		0
#endif
#define BOARD_RK2800_DEMO					0
#define BOARD_RUIGUANG_T28				0
#ifdef CONFIG_MACH_PWS700AA 
#define BOARD_RK2808_PWS700AA			1
#else
#define BOARD_RK2808_PWS700AA			0
#endif
#if(defined(CONFIG_BOARD_RK5900)\
	||defined(CONFIG_BOARD_BM999) \
	||defined(CONFIG_BOARD_BM730)\
	||defined(CONFIG_BOARD_IPAD)\
	||defined(CONFIG_BOARD_IPADV5)\
	||defined(CONFIG_BOARD_E700)\
	||defined(CONFIG_BOARD_IPADY1006)\
	||defined(CONFIG_BOARD_NX7005)\
	||defined(CONFIG_BOARD_TD05D6)\
	||defined(CONFIG_BOARD_TD10D6)\
	||defined(CONFIG_BOARD_IPAD8)\
	||defined(CONFIG_BOARD_IPAD100)\
	||defined(CONFIG_BOARD_NM701)\
	||defined(CONFIG_BOARD_ZTX)\
	)
//#undef BOARD_ANDROID_RK2808SDK
#define BOARD_ANDROID_RK2808SDK		0
#define BOARD_AIGO5900				1
#else
#define BOARD_ANDROID_RK2808SDK		1
#define BOARD_AIGO5900				0
#endif


struct wifi_power power_gpio = 
{
#if ((BOARD_ANDROID_RK2800SDK == 1) || (BOARD_RK2800_DEMO == 1))

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOH6_IQ_SEL_NAME,
	IOMUXB_GPIO1_D6, GPIOPortH_Pin6, GPIO_HIGH
	
#elif (BOARD_RUIGUANG_T28 == 1)

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOF5_APWM3_DPWM3_NAME,
	IOMUXB_GPIO1_B5, GPIOPortF_Pin5, GPIO_HIGH
	
#elif (BOARD_RK2808_PWS700AA == 1)

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOE_SPI1_SEL_NAME,
	IOMUXA_GPIO1_A1237, GPIOPortF_Pin7, GPIO_HIGH
	
#elif (BOARD_AIGO5900 == 1)//V5版本改成常上电，故此设置实际没用到，io打开放到key里
#if(defined(CONFIG_BOARD_NX7005)||defined(CONFIG_BOARD_TD05D6)||defined(CONFIG_BOARD_TD10D6))
#if(defined(CONFIG_RK28_I2C_GPIO_EXPANDERS))
	0, 0, 0, 0, PCA955X_Pin0, GPIO_HIGH
#endif
#else
	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOF5_APWM3_DPWM3_NAME,
	IOMUXB_GPIO1_B5, GPIOPortF_Pin5, GPIO_HIGH
#endif
#else
	0, 0, 0, 0, 0, 0
#endif
};

struct wifi_power power_save_gpio = 
{
#if (BOARD_ANDROID_RK2800SDK == 1)

	0, 0, 0, 0, 0, 0

#elif (BOARD_RK2800_DEMO == 1)

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOE_U1IR_I2C1_NAME,
	IOMUXA_GPIO1_A67, GPIOPortE_Pin6, GPIO_HIGH

#elif ((BOARD_RUIGUANG_T28 == 1) || (BOARD_ANDROID_RK2808SDK == 1))

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOE_SPI1_SEL_NAME,
	IOMUXA_GPIO1_A1237, GPIOPortE_Pin1, GPIO_HIGH
#elif (BOARD_RK2808_PWS700AA == 1)

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOB1_SMCS1_MMC0PCA_NAME,
	IOMUXA_GPIO0_B1, GPIOPortB_Pin1, GPIO_HIGH
#elif (BOARD_AIGO5900 == 1)
#if(defined(CONFIG_BOARD_NX7005)||defined(CONFIG_BOARD_TD05D6)||defined(CONFIG_BOARD_TD10D6))
#if(defined(CONFIG_RK28_I2C_GPIO_EXPANDERS))
	0, 0, 0, 0, PCA955X_Pin15, GPIO_HIGH
#endif
#else
//	POWER_NOT_USE_GPIO, POWER_GPIO_IOMUX, GPIOE_SPI1_SEL_NAME,
//	IOMUXA_GPIO1_A1237, GPIOPortE_Pin1, GPIO_HIGH

	POWER_USE_GPIO, POWER_GPIO_IOMUX, GPIOF0_UART1_CPWM0_NAME,
	IOMUXA_GPIO1_B0, GPIOPortF_Pin0, GPIO_HIGH
#endif
#else
	0, 0, 0, 0, 0, 0
#endif
};

int wifi_gpio_operate(struct wifi_power *gpio, int flag)
{
	int value, sensitive;

#if(defined(CONFIG_BOARD_NX7005)||defined(CONFIG_BOARD_TD05D6)||defined(CONFIG_BOARD_TD10D6))
#if(defined(CONFIG_RK28_I2C_GPIO_EXPANDERS))
	if (flag == GPIO_SWITCH_ON)
		sensitive = gpio->sensi_level;
	else
		sensitive = 1 - gpio->sensi_level;
		
	pca955x_gpio_direction_output(gpio->gpio_id,sensitive);
#endif
#else
	
	if (gpio->use_gpio == POWER_NOT_USE_GPIO)
		return 0;
	
	if (gpio->gpio_iomux == POWER_GPIO_IOMUX)
	{
		rockchip_mux_api_set(gpio->iomux_name, gpio->iomux_value);
	}
	
	GPIOSetPinDirection(gpio->gpio_id, GPIO_OUT);
	
	if (flag == GPIO_SWITCH_ON)
		sensitive = gpio->sensi_level;
	else
		sensitive = 1 - gpio->sensi_level;
		
	GPIOSetPinLevel(gpio->gpio_id, sensitive);
#endif
#if 0 /* Useless */	
	value = GPIOGetPinLevel(gpio->gpio_id);
	if (value != sensitive)
	{
		return -1;
	}
#endif

	return 0;
}


int wifi_turn_on_card(void)
{
	//printk("Turning on SDIO card.\n");
	
	if (wifi_gpio_operate(&power_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] successfully for power supply.\n");
		return -1;
	}
	
	return 0;
}

int wifi_turn_off_card(void)
{
	//printk("Turning off SDIO card.\n");
	
	if (wifi_gpio_operate(&power_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [OFF] successfully for power supply.\n");
		return -1;
	}
	
	return 0;
}

int wifi_power_up_wifi(void)
{
	//printk("Power up SDIO card.\n");

	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] successfully for power up.\n");
		return -1;
	}
	mdelay(5);
	
	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [ON] successfully for power up.\n");
		return -1;
	}
	msleep(150);

	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_ON) != 0)
	{
		printk("Couldn't set GPIO [ON] successfully for power up.\n");
		return -1;
	}
	msleep(50);

	return 0;
}

int wifi_power_down_wifi(void)
{
	if (wifi_gpio_operate(&power_save_gpio, GPIO_SWITCH_OFF) != 0)
	{
		printk("Couldn't set GPIO [OFF] successfully for power down.\n");
		return -1;
	}
	
	return 0;
}

#endif /* WIFI_GPIO_POWER_CONTROL */

