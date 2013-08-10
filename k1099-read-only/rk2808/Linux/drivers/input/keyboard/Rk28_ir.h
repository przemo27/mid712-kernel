/******************************************************************/
/* Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.     */
/*******************************************************************
File  : remotectl.h
Desc  :   遥控器接收驱动程序
Author  :   dqz
Date  : 2010-1-04
Notes  :

********************************************************************/
#ifndef _REMOTECTL_H
#define _REMOTECTL_H
#define IN_REMOTE
#define RAMOS_REMOTE
/********************************************************************
**                            宏定义                                *
********************************************************************/
#define TIME_BIT0_MIN  625  /*Bit0  1.125ms*/
#define TIME_BIT0_MAX  1625

#define TIME_BIT1_MIN  1650  /*Bit1  2.25ms*/
#define TIME_BIT1_MAX  2650

#define TIME_PRE_MIN   4500
#define TIME_PRE_MAX   5500  /*PreLoad 4.5+0.56 = 5.06ms*/

#define TIME_RPT_MIN   101000
#define TIME_RPT_MAX   103000  /*Repeat  105-2.81=102.19ms*/

#define TIME_SEQ_MIN   2650
#define TIME_SEQ_MAX   3000  /*sequence  2.25+0.56=2.81ms*/


#define AD1KEY1		103//DPAD_UP	----------UP
#define AD1KEY2 		106//DPAD_RIGHT-------FFD
#define AD1KEY3 		105//DPAD_LEFT--------FFW	
#define AD1KEY4 		108//DPAD_DOWN------DOWN		
#define AD1KEY5		62   //ENDCALL           WAKE_DROPPED
#define AD1KEY6		28    //ENTER             

#define AD2KEY1		114   //VOLUME_DOWN  	 	59	//MENU			//115   //VOLUME_UP
#define AD2KEY2 		115   //VOLUME_UP      		//114   //VOLUME_DOWN
#define AD2KEY3 		59	//MENU	
#define AD2KEY4 		102   //HOME				//62   //ENDCALL  		
#define AD2KEY5 		158	//158	//BACK----ESC   		115   //VOLUME_UP	//158	//BACK------------ESC
//#define AD2KEY6 		116	//POWER

#define Valuedrift		70
#define EmptyADValue	1000
#define InvalidADValue	10
#define ADKEYNum		12

/*power event button*/
#define  POWER				116
#define  ENDCALL				62
#define  ONESEC_TIMES		100
#define  SEC_NUM			1
#define  SLEEP_TIME			2	/*per 40ms*/

/********************************************************************
**                          结构定义                                *
********************************************************************/
typedef enum _RMC_STATE
{
    RMC_IDLE,
    RMC_PRELOAD,
    RMC_USERCODE,
    RMC_GETDATA,
    RMC_SEQUENCE
}eRMC_STATE;

typedef struct _REMOTE_CTL
{
    uint8 state;
    uint8 count;
    uint8 keybdNum;
    uint8 press;
    uint16 scanCode;
    uint16 data;
    uint16 timerId;
    uint32 lastTime;
}REMOTE_CTL, *pREMOTE_CTL;

typedef struct _RMCKEY_TRANSFER
{
    uint16 keycode;
    uint16 scanCode;
} RMCKEY_TRANSFER, *pRMCKEY_TRANSFER;

typedef struct _RMCKEY_BOARD
{
    uint16 usercode;
    uint16 count;
    pRMCKEY_TRANSFER keyTrans;
} RMCKEY_BOARD, *pRMCKEY_BOARD;

/********************************************************************
**                          变量定义                                *
********************************************************************/
#ifdef IN_REMOTE
static RMCKEY_TRANSFER g_rmcKeyTrans_8057[]  =   /*  根据按键使用情况排列 */
{
    {0x48,   10},       //MUTE
    {0x40,   0xFF},     //MODE
    {0x28,   11},       //POWER
    {0x58,   12},       //AV   rotate
    {0x68,   5},        //UP
    {0xB8,   0},        //ESC
    {0xA0,   2},        //LEFT
    {0xF0,   3},        //ENTER
    {0x98,   1},        //RIGHT
    {0x38,  0xFF},      //OSD
    {0xB0,  4},         //DOWN
    {0x18,  13},        //ZOOM
    {0x78,  14},        //VOL+
    {0xD8,  19},        //FFD   快退
    {0xF8,  16},        //PREV
    {0x60,  15},        //VOL-
    {0x30,  18},        //FFW   快进
    {0xC8,  17}         //NEXT
};


static RMCKEY_TRANSFER g_rmcKeyTrans_7070[]  =
{
    {0xA8,   10},       //MUTE
    {0x98,   0xFF},     //MODE
    {0x28,   11},       //POWER
    {0x58,   12},       //AV  rotate
    {0x80,   5},        //UP
    {0x40,   0},        //ESC
    {0xC0,   2},        //LEFT
    {0x20,   3},        //ENTER
    {0xA0,   1},        //RIGHT
    {0x00,  0xFF},      //OSD
    {0xE0,  4},         //DOWN
    {0x90,  13},        //ZOOM
    {0x48,  14},        //VOL+
    {0x10,  18},        //FFW
    {0x88,  16},        //PREV
    {0xC8,  15},        //VOL-
    {0x60,  19},        //FFD
    {0x08,  17}         //NEXT
};

static RMCKEY_TRANSFER g_rmcKeyTrans_0AF5[]  =
{
    {0x80,    20},      //Module 1
    {0x40,    21},      //Module 2
    {0xc0,    22},      //Module 3
    {0x20,    23},      //Module 4
    {0xa0,    24},      //Module 5
    {0x60,    25},      //Module 6
    {0xe0,    26},      //Module 7
    {0x10,    27},      //Module 8

    {0xA8,   10},       //MUTE
    {0x98,   0xFF},     //MODE
    {0xE8,   11},       //POWER
    {0x58,   12},       //AV  rotate
    {0x80,   5},        //UP
    {0x40,   0},        //ESC
    {0xC0,   2},        //LEFT
    {0x20,   3},        //ENTER
    {0xA0,   1},        //RIGHT
    {0x00,  0xFF},      //OSD
    {0xE0,  4},         //DOWN
    {0x90,  13},        //ZOOM
    {0x48,  14},        //VOL+
    {0x10,  18},        //FFW
    {0x88,  16},        //PREV
    {0xC8,  15},        //VOL-
    {0x60,  19},        //FFD
    {0x08,  17}         //NEXT
};

#ifndef RAMOS_REMOTE
static RMCKEY_TRANSFER g_rmcKeyTrans_00FF[]  =
{
    {0x90,    20},      //Module 1
    {0xB8,    21},      //Module 2
    {0xF8,    22},      //Module 3
    {0xB0,    23},      //Module 4
    {0x98,    24},      //Module 5
    {0xD8,    25},      //Module 6
    {0x88,    26},      //Module 7
    {0xA8,    27},      //Module 8

    {0x82,   10},       //MUTE
    {0x98,   0xFF},     //MODE
    {0x28,   11},       //POWER
    {0x9A,   12},       //AV  rotate
    {0x12,   5},        //UP
    {0x52,   0},        //ESC
    {0x60,   2},        //LEFT
    {0xA0,   3},        //ENTER
    {0xC0,   1},        //RIGHT
    {0x00,  0xFF},      //OSD
    {0x02,  4},         //DOWN
    {0x10,  13},        //ZOOM
    {0xAA,  14},        //VOL+
    {0x10,  18},        //FFW
    {0x88,  16},        //PREV
    {0x0A,  15},        //VOL-
    {0x60,  19},        //FFD
    {0x08,  17}         //NEXT
};
#elif (defined(CONFIG_BOARD_BM999))
static RMCKEY_TRANSFER g_rmcKeyTrans_00FF[]  =
{
	{0,		AD1KEY5},
	{32,		KEY_MUTE},
	{160,	KEY_UP},
	{48,		KEY_LEFT},
	{56,		KEY_RIGHT},
	{64,		KEY_DOWN},
	{200,	KEY_VOLUMEDOWN},
	{8,		KEY_VOLUMEUP},
	{136,	KEY_HOME},
	{216,	KEY_BACK},
	{24,		AD2KEY3},
	{152,	KEY_ENTER},
};
#elif(defined(CONFIG_BOARD_RK5900)) //英唐5900  CONFIG_BOARD_RK5900  
//#define REMOTE_USERCODE   255                        /*本机的用户码*/
static RMCKEY_TRANSFER g_rmcKeyTrans_00FF[]  =
{
	{216,	ENDCALL},//KEY_POWER
	{232,	KEY_BACK },// //KEY_ESC
	{152,	KEY_UP},
	{40,	KEY_LEFT},
	{168,	KEY_ENTER},
	{104,	KEY_RIGHT},
	{136,	KEY_DOWN	},
	{240,	KEY_VOLUMEDOWN	},
	{32,	KEY_VOLUMEUP	},
};
#elif defined (CONFIG_BOARD_NX7005)
static RMCKEY_TRANSFER g_rmcKeyTrans_00FF[]  =
{
	{216,	ENDCALL},//KEY_POWER
	{232,	KEY_BACK },// //KEY_ESC
	{152,	KEY_UP},
	{40,	KEY_LEFT},
	{168,	KEY_ENTER},
	{104,	KEY_RIGHT},
	{136,	KEY_DOWN	},
	{240,	KEY_VOLUMEDOWN	},
	{32,	KEY_VOLUMEUP	},
};
#else
/* onda IR_key value */
static RMCKEY_TRANSFER g_rmcKeyTrans_00FF[]  =
{
    {0x88, AD1KEY3},  //LEFT//PREV
    {0xC8,  AD1KEY2},    //RIGHT//NEXT
    {0x48, AD1KEY6},  //OK/PLAY

    {0x50, AD1KEY1},  //UP
    {0x68, AD1KEY4},   //DOWN

    {0x58, AD2KEY3},  //MENU
    {0x98, AD2KEY5},  //ESC

    {0xB0, AD1KEY5},  //EndCall

    {0xF0, 333},  //Silence
    {0xA8, 444},  //Scale
    {0xE8, 555},  //TV	
    {0xD8, 555},  //Caption	
    {0xB8, AD2KEY1},  //tmp-defined VOL+
    {0xF8, AD2KEY2},  //tmp-defined VOL-
};
#endif

static RMCKEY_TRANSFER g_rmcKeyTrans_807F[]  =
{
    {0xE0,   10},       //MUTE
    {0x98,   0xFF},     //MODE
    {0x40,   11},       //POWER
    {0xA8,   12},       //AV  rotate
    {0x50,   5},        //UP
    {0x20,   0},        //ESC
    {0x88,   2},        //LEFT
    {0x48,   3},        //ENTER
    {0xC8,   1},        //RIGHT
    {0x00,  0xFF},      //OSD
    {0x68,  4},         //DOWN
    {0xE8,  13},        //ZOOM
    {0x28,  14},        //VOL+
    {0x10,  18},        //FFW
    {0x88,  16},        //PREV
    {0x18,  15},        //VOL-
    {0x60,  19},        //FFD
    {0x08,  17}         //NEXT
};

static RMCKEY_TRANSFER g_rmcKeyTrans_01FE[]  =
{
#ifndef RAMOS_REMOTE
    {0x10, AD1KEY1},  //菜单键 Menu Key
    {0x80, AD1KEY1},  //退出键 ESC Key
    {0xE0, AD1KEY1},  //Pre
    {0x90, AD1KEY1},  //Next
    {0xA0, AD1KEY1},  //VOLDN
    {0xD8, AD1KEY1},  //VOLUP
    {0x78, AD1KEY1},  //Play 键
    {0xC0, 1},  //<- 键
    {0x48, 9}  //Power 键
#else
    //{0xFF,   10},     //MUTE
    {0x78,   6},  //MODE PLAY
    {0x48,   11},  //POWER 直接关机键
    {0xA8,   12},   //AV rotate
    {0xA0,   5},  //VOLINC
    {0x80,   0},  //ESCMain//直接返回主界面
    {0xE0,   2},  //LEFT
    {0x10,   3},  //ENTER
    {0x90,   1},  //RIGHT
    {0x00,  0xFF},  //OSD
    {0xD8,  4},  // VOLDEC
    {0xE8,  13},  //ZOOM
    {0x28,  5},   //VOL+
// {0x10,  18},  //FFW
    {0x88,  16},  //PREV
    {0x18,  4},  //VOL-
    {0x60,  19},  //FFD
    {0x08,  17},  //NEXT
    {0xc0,    1}        // <----一级一级返回
#endif
};
#ifndef RAMOS_REMOTE
static RMCKEY_TRANSFER g_rmcKeyTrans_0202[]  =
{
    {0x02,   AD1KEY1}, //MENU
    {0x82,   AD1KEY1}, //RETURN
    {0x08,   AD1KEY1},  //PREV
    {0x88,   AD1KEY1},  //NEXT
    {0xB0,   AD1KEY1},  //PLAY
    {0x70,  AD1KEY1},   //DOWN
    {0xD0,  AD1KEY1}  //UP
};
#else
static RMCKEY_TRANSFER g_rmcKeyTrans_0202[]  =
{
    {0x02,   AD1KEY1},    //MENU
    {0x82,   AD1KEY1},  //RETURN
    {0x08,   AD1KEY1},     //PREV
    {0x88,   AD1KEY1},     //NEXT
    {0xB0,   AD1KEY1},  //PLAY
    {0x70,   AD1KEY1},      //DOWN
    {0xD0,   AD1KEY1}     //UP
};
#endif
static RMCKEY_BOARD g_rmcKeyBoard[] =
{
    {0x0202, ARRSIZE(g_rmcKeyTrans_0202), &g_rmcKeyTrans_0202[0]},
    {0x8057, ARRSIZE(g_rmcKeyTrans_8057), &g_rmcKeyTrans_8057[0]},
    {0x00FF, ARRSIZE(g_rmcKeyTrans_00FF), &g_rmcKeyTrans_00FF[0]},
    {0x7070, ARRSIZE(g_rmcKeyTrans_7070), &g_rmcKeyTrans_7070[0]},
    {0x0AF5, ARRSIZE(g_rmcKeyTrans_0AF5), &g_rmcKeyTrans_0AF5[0]},
    {0x807F, ARRSIZE(g_rmcKeyTrans_807F), &g_rmcKeyTrans_807F[0]},
    {0x01FE, ARRSIZE(g_rmcKeyTrans_01FE), &g_rmcKeyTrans_01FE[0]}
};

REMOTE_CTL g_rmcInfo;
eRMC_STATE g_rmcState;
#endif
/********************************************************************
**                      对外函数接口声明                            *
********************************************************************/

/********************************************************************
**                          表格定义                                *
********************************************************************/

#endif
