/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File	:	api_pmu.h
Desc	:
Author	:	yangkai
Date	:	2008-12-16
Notes	:
$Log: api_pmu.h,v $
********************************************************************/
#ifndef _API_PMU_H
#define _API_PMU_H
#include "scu.h"


/********************************************************************
**							结构定义								*
********************************************************************/
typedef enum _PMU_APP
{
	PMU_DUMMY = 0,
	PMU_IDLE ,
	PMU_INIT,
	PMU_MEDIALIBUPDATE,
	PMU_MP3,
	PMU_MP3H,
	PMU_WMA,
	PMU_MP2,
	PMU_WAV,
	PMU_APE,
	PMU_FLAC,
	PMU_RA,
	PMU_AAC,
	PMU_OGG,
	PMU_EQ,
	PMU_RECORDADPCM,
	PMU_RECORDMP3,
	PMU_EMAP,
	PMU_VIDEOLOWLL,
	PMU_VIDEOLOWL,
	PMU_VIDEOLOW,
	PMU_VIDEOMEDLOW,
	PMU_VIDEOMED,
	PMU_VIDEOMEDHIGH,
	PMU_VIDEOHIGH,
	PMU_VIDEOTVOUT,
	//PMU_PICTURE,
	PMU_BMP,
	PMU_JPEG,
	PMU_GIF,
	PMU_STOPWATCH,
	PMU_CAMLOW, 	//camera
	PMU_CAMHIGH,
	PMU_GAME,
	PMU_USB,
	PMU_BLON,
	PMU_LCD_UPDATE,

	PMU_APP_MAX
}ePMU_APP;

typedef struct tagPMU_APP_TABLE
{
	uint8  scuAppId;
	uint8  counter;
	uint16 armFreq;
	uint16 dspFreq;
}PMU_APP_TABLE,*pPMU_APP_TABLE;

typedef enum _PMU_PCLK_DIV
{
	PCLK_DIV1 = 0,//CLK_HCLK_PCLK_11>>2,	   // AHB clk : APB clk =  1:1
	PCLK_DIV2 = 1,//CLK_HCLK_PCLK_21>>2,	   // AHB clk : APB clk =  2:1
	PCLK_DIV4 = 2,//CLK_HCLK_PCLK_41>>2,	   // AHB clk : APB clk =  4:1
	PCLK_DIV_MAX
}ePMU_PCLK_DIV;

typedef enum _PMU_HCLK_DIV
{
	HCLK_DIV1 = 0,//CLK_ARM_HCLK_11,		   // arm clk : hclk  =  1:1
	HCLK_DIV2 = 1,//CLK_ARM_HCLK_21,		   // arm clk : hclk  =  2:1
	HCLK_DIV3 = 2,//CLK_ARM_HCLK_31,		   // arm clk : hclk  =  3:1
	HCLK_DIV4 = 3,//CLK_ARM_HCLK_41,		   // arm clk : hclk  =  4:1
	HCLK_DIV_MAX
}ePMU_HCLK_DIV;

/********************************************************************
**						对外函数接口声明							*
********************************************************************/

extern int32 PMUStartAPP(ePMU_APP appId);
extern int32 PMUStopAPP(ePMU_APP appId);

extern int32 PMUSetCodecFreq(uint32 freq);
extern void PMUSetLCDCFreq(uint32 freq);

/********************************************************************
**							表格定义								*
********************************************************************/
#ifdef IN_PMU
PMU_APP_TABLE g_scuAPPTabel[PMU_APP_MAX] =
{
	{PMU_DUMMY, 				0,	0,				0},
	{PMU_IDLE,					0,	FREQ_ARM_IDLE,	0},
	{PMU_INIT,					0,	FREQ_ARM_MAX,			0},
	{PMU_MEDIALIBUPDATE,		0,	192,			0},
	{PMU_MP3,					0,	60, 			0},
	{PMU_MP3H,					0,	72, 			0},
	{PMU_MP2,					0,	92, 			0},
	{PMU_WMA,					0,	120,			0},
	{PMU_WAV,					0,	24, 			0},
	{PMU_APE,					0,	FREQ_ARM_MAX,	0},
	{PMU_FLAC,					0,	64, 			0},
	{PMU_RA,					0,	60, 			0},
	{PMU_AAC,					0,	200,			0},
	{PMU_OGG,					0,	150,			0},
	{PMU_EQ,					0,	70, 			0},
	{PMU_RECORDADPCM,			0,	110,			0},
	{PMU_RECORDMP3, 			0,	48, 			0},
	{PMU_EMAP,					0,	200,			0},
	{PMU_VIDEOLOWLL,			0,	FREQ_ARM_MAX,	FREQ_DSP_MAX},	 
	{PMU_VIDEOLOWL, 			0,	FREQ_ARM_MAX,	FREQ_DSP_MAX},
	{PMU_VIDEOLOW,				0,	FREQ_ARM_MAX,	FREQ_DSP_MAX}, 
	{PMU_VIDEOMEDLOW,			0,	FREQ_ARM_MAX,	FREQ_DSP_MAX},
	{PMU_VIDEOMED,				0,	FREQ_ARM_MAX,	FREQ_DSP_MAX}, 
	{PMU_VIDEOMEDHIGH,			0,	FREQ_ARM_MAX,	FREQ_DSP_MAX},
	{PMU_VIDEOHIGH, 			0,	FREQ_ARM_MAX,	FREQ_DSP_MAX},
	{PMU_VIDEOTVOUT,			0,	FREQ_ARM_MAX,	FREQ_DSP_MAX},
	{PMU_BMP,					0,	FREQ_ARM_MAX,	0},
	{PMU_JPEG,					0,	FREQ_ARM_MAX,	0},
	{PMU_GIF,					0,	FREQ_ARM_MAX,	0},
	{PMU_STOPWATCH, 			0,	40, 			0},
	{PMU_CAMLOW,				0,	140,			120}, //camera
	{PMU_CAMHIGH,				0,	200,			176},
	{PMU_GAME,					0,	60, 			0},
	{PMU_USB,					0,	240,			0},
#ifdef RGB_PANEL
	{PMU_BLON,					0,	FREQ_ARM_MAX,	0},  //根据屏大小再做调节
#else
	{PMU_BLON,					0,	60, 			0},
#endif
	{PMU_LCD_UPDATE,			0,	192,			0},
};

#endif
#endif

