/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File	:	api_pll.h
Desc	:	
Author	:	yangkai
Date	:	2008-12-16
Notes	:
$Log: api_pll.h,v $
********************************************************************/
#ifndef _API_PLL_H
#define _API_PLL_H

/********************************************************************
**						对外函数接口声明							*
********************************************************************/

extern uint32 PLLGetArmFreq(void);
extern uint32 PLLGetDspFreq(void);
extern uint32 PLLGetAuxFreq(void);
extern uint32 PLLGetAHBFreq(void);
extern uint32 PLLGetAPBFreq(void);

#endif

