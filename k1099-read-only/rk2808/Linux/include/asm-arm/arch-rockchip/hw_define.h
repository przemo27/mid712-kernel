/*********************************************************************************
*     Copyright (C),2004-2005,  Fuzhou Rockchip Co.,Ltd.
*         All Rights Reserved
*          V1.00
* FileName :  Hw_define.h
* Author :  lzy
* Description:   包含硬件系统的相关定义和宏定义
* History  :
*   <author>  <time>    <version>    <desc>
*    lzy     07/8/23    1.0    ORG
$Log: hw_define.h,v $
*********************************************************************************/
#ifndef _HW_DEFINE_H
#define _HW_DEFINE_H

/************************        板子类型定义            ****************************/
#define RK2800_FPGA      (1)
#define RK2800_SDKDEMO   (2)
#define RK2806_SDKDEMO   (3)

#define BOARDTYPE        RK2800_SDKDEMO//RK2800_FPGA

/************************        SDRAM define            ****************************/
#define SDRAM_2x32x4     (1)   // 32M
#define SDRAM_4x32x4     (2)   // 64M
#define SDRAM_8x32x4     (3)   // 128M
#define SDRAM_16x32x4    (4)   // 256M

#define SDRAM            (1)
#define MOBILE_SDRAM     (2)

#define SDRAM_TYPE       SDRAM
#define SDRAM_SIZE       SDRAM_4x32x4

/************************        Codec define            ****************************/
#define WM8750_CODEC     (1)
#define WM8987_CODEC     (2)
#define WM8988_CODEC     (3)

#define CODEC_TYPE       WM8987_CODEC
#define MAX_VOLUME       (32)

/************************        RTC define              ****************************/
#define INTERNAL_RTC     (0)
#define PT7C4337         (1)
#define HYM8563          (2)

#define RTC_TYPE         HYM8563

/***********************         FM define               ****************************/
#define FM5800_TYPE      (0)
#define FM5767_TYPE      (1)
#define FM5807_TYPE      (2)
#define FM4702_TYPE      (3)
#define FM8006_TYPE      (4)

#define FM_TYPE          FM8006_TYPE

/***********************         Keyboard define         ****************************/
#define NAVIKEY          (0)
#define MATRIXKEY        (1)
#define GAMEPAD          (2)
#define ADKEY_ALL        (3)
#define RM970PLUSKEY     (4)

//有可能同时使用AD按键和IO键
#if ( BOARDTYPE == RK2800_SDKDEMO || BOARDTYPE == RK2806_SDKDEMO )
#define ADKEY_SUPPORT    1 // 1:支持 AD按键, 0:不支持AD按键.
#define KEY_TYPE         ADKEY_ALL
#else
#define KEY_TYPE         NAVIKEY
#endif

#ifdef DRIVER_ONLY
#define KEY_GLOBALS
#endif

/***********************         ADC define              ****************************/
#define BATTERY_ADC      Adc_channel0
#define KEYBOARD_ADC     Adc_channel1
#define VREF_ADC         Adc_channel3
#define VCC_VALUE        (3300)    //定义LDO 的电压，必须根据硬件更改

/***********************         LCD define              ****************************/
#undef  MCU_PANEL
#define RGB_PANEL                  //屏幕类型使能选择，RGB_PANEL表示当前系统使用RGB屏，MCU_PANEL表示当前系统使用MCU屏

/********************************
//目前已有的TVOUT类型
//MCU:  MCU_ILI9320
//RGB:  RGB_LTE700      RGB_TD043MGEA1
//NULL: NOPANEL
********************************/
#if ( BOARDTYPE == RK2800_SDKDEMO || BOARDTYPE == RK2806_SDKDEMO )
#define PANEL_LCD        RGB_TD043MGEA1      //配置主屏
#else
#define PANEL_LCD        RGB_TD043MGEA1      //配置主屏
#endif

/***********************         TVOUT define            ****************************/
#define TVOUT_VIDEO_W    (720)
#define TVOUT_VIDEO_H    (576)
#define TVOUT_UI_W       (720)
#define TVOUT_UI_H       (576)

#define TVOUT_PANEL       //TVOUT使能开关，有定义表示有TVOUT，没定义表示没有TVOUT
/********************************
//目前已有的TVOUT类型
//TVOUT:TV_ADV7391      TV_CH7026
//NULL: NOPANEL  
********************************/
#if ( BOARDTYPE == RK2800_SDKDEMO || BOARDTYPE == RK2806_SDKDEMO )
#define PANEL_TVOUT      TV_CH7026           //配置TVOUT芯片
#else
#define PANEL_TVOUT      TV_CH7026           //配置TVOUT芯片
#endif

/***********************         backlight define       *****************************/
#define BALCKIGHT_PIN_INACTIVE_LEVEL   GPIO_HIGH

/***********************         Sensor define          *****************************/
#define SENSOR_OV_9650              (1)
#define SENSOR_MICRON_MT9M112       (2)

#define SENSOR_TYPE                 SENSOR_OV_9650

/***********************         GPIO define            *****************************/
#if (BOARDTYPE == RK2800_SDKDEMO || BOARDTYPE == RK2806_SDKDEMO || BOARDTYPE ==RK2800_FPGA)
#define POWER_CONTROL_PIN   GPIOPortF_Pin1
#define KEY_PLAYON_PIN      GPIOPortF_Pin0
#define CODEC_EAR_CTRLPIN   GPIOPortC_Pin2   // 耳机公共端控制管脚
#define CHARGE_OK_PIN       GPIOPortB_Pin0
#define CMMBPWR_EN_PIN      GPIOPortG_Pin5
#define DIB_RESET_CA_PIN    GPIOPortG_Pin4
#define VIP_RESET_PIN       GPIOPortG_Pin3
#define VIP_PWRDN_PIN       GPIOPortG_Pin2
#define SPEAKER_CTRLPIN     GPIOPortG_Pin7
#define TOUCHPAD_INT_PIN    GPIOPortE_Pin7
#define RMC_GPIO            GPIOPortE_Pin6   //Remote(IR)
#define LCDC_DISP_ON_PIN    GPIOPortB_Pin2
#define LCDPANEL_CS_PIN     GPIOPortB_Pin3
#define CMMB_INT_PIN        GPIOPortE_Pin0
#endif

//游戏手柄使用的GPIO
#if (KEY_TYPE == GAMEPAD)
#define GAMEPAD_SHIFT    GPIOPortC_Pin2
#define GAMEPAD_DATA2    GPIOPortC_Pin3
#define GAMEPAD_DATA1    GPIOPortC_Pin4
#define GAMEPAD_CLK      GPIOPortC_Pin6
#endif

//矩阵按键使用的GPIO
#if ( BOARDTYPE == RK2800_SDKDEMO || BOARDTYPE == RK2806_SDKDEMO )
#define MatrixKey_ROW_1
#define MatrixKey_ROW_2
#define MatrixKey_ROW_3
#define MatrixKey_COL_1
#define MatrixKey_COL_2
#define MatrixKey_COL_3
#define MatrixKey_COL_PORT
#define MatrixKey_COL_SHIFT
#define MatrixKey_COL_MASK
#endif

/***********************         DV define            *****************************/
//打开该编译确保是在支持sensor的板子上运行(RK2700/RK2708等)，注意BOARDTYPE 定义的同步!!!
#define ROCK_CAMERA		0
	
#if ROCK_CAMERA
    #include "CamConfig.h"
#endif

#endif
