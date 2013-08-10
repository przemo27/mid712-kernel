/* Copyright (C) 2006 ROCK-CHIPS FUZHOU . All Rights Reserved. */
/*
File  : system\include\typedef.h
Desc  : 定义 与处理器无关的基本数据类型 。

Author : huangsl
Date  : 2006-04-03
Notes  : 要求所有的 .c ,.h 文件都必须第一个包含这个文件。


$Log: typedef.h,v $
Revision 1.5  2008/01/11 02:34:28  Huangshilin
增加更新 二级 LOADER的功能.打开 特效功能.

Revision 1.4  2007/11/05 09:53:48  Huangshilin
增加GUI接口,增加应用资源互斥控制.

Revision 1.3  2007/10/30 10:25:52  Hanjiang
no message

Revision 1.2  2007/10/08 02:44:49  Lingzhaojun
添加版本自动注释脚本

* huangsl     2006/04/03 建立此文件，用于定义 基本数据类型。
*
* huangsl     2006/04/23 增加 小写习惯的类型定义。
*
* huangsl     2007/06/07 修改为 ARM 处理器定义。
*
*/

#ifndef  _SYS_TYPE_DEFINE_H_
#define  _SYS_TYPE_DEFINE_H_
/*********************************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
    /*-----------------------------------------------------------*/

    /* 基本类型 */
#ifndef _WINNT_
    typedef  char   BOOLEAN; //unsigned
#endif

    typedef unsigned char   INT8U;
    typedef signed  char   INT8S;
    typedef unsigned short  INT16U;
    typedef signed  short  INT16S;
    typedef int       INT32S;
    typedef unsigned int   INT32U;

    typedef unsigned long   L32U;
    typedef signed  long  L32S;

    typedef unsigned char   BYTE;
    typedef unsigned long ULONG;

#ifndef _WINNT_
    typedef unsigned short  WCHAR;  //两位长度,为了统一
#endif

#ifdef __arm
    typedef signed long long   INT64S;   //The low word of a long long is at the low address in little-endian mode
    typedef unsigned long long  INT64U;
#elif defined( _MSC_VER )
    typedef unsigned __int64       INT64S;   //The low word of a long long is at the low address in little-endian mode
    typedef __int64       INT64U;
#endif
    /**********************************************************************/
    /* 小写习惯 */
    typedef unsigned long  int32u;
    typedef unsigned short int16u;
    typedef unsigned char  int8u;

    typedef long  int32s;
    typedef short int16s;
    typedef char  int8s;

    typedef unsigned char byte;

    typedef unsigned long long uint64;
    typedef unsigned long  uint32;
    typedef unsigned short uint16;
    typedef unsigned char  uint8;

    typedef long  int32;
    typedef short int16;
    typedef signed char  int8;
 //   typedef unsigned char bool;
    /********************************************************************/
    typedef INT32U      RK_GUID;
    typedef void*       PROCHANDLE;    //PROC HANDLE
    typedef void*       THANDLE;    //THREAD HANDLE
    typedef void*       HTIMER;

    /********************************************************************
     TYPE DEFINE
    ********************************************************************/
    typedef volatile unsigned int       REG32;
    typedef volatile unsigned short     REG16;
    typedef volatile unsigned char      REG8;

#if !defined( _WINDEF_ ) //_MSC_VER
    typedef BOOLEAN BOOL;
#endif

    typedef volatile unsigned int  data_t;
    typedef volatile unsigned int* addr_t;
    
    typedef 	void (*pFuncIntr)(int,void *);	
//    typedef unsigned int size_t;
    typedef     void (*pFunc)(void);            //定义函数指针, 用于调用绝对地址

    /*-----------------------------------------------------------------------------*/
    /* 扩展类型 */
#if !defined( _WINNT_ )
#define BYTE     INT8S
#define UBYTE     INT8U
#define WORD     INT16S
#define UWORD     INT16U
#define LONG     INT32S
#define ULONG     INT32U
#define DWORD     INT32U
#endif

#define UCHAR           INT8U
#define SSHORT          INT16S
#define USHORT          INT16U
#define INT             INT32S
#define UINT            INT32U

    /*----------------------------------------------------------------------------------*/
    /* 以下定义为了方便不同的习惯 */
#ifndef UINT8
#define UINT8 INT8U
#endif

#ifndef INT8
#define INT8 INT8S
#endif

#ifndef UINT16
#define UINT16 INT16U
#endif

#ifndef INT16
#define INT16 INT16S
#endif

#ifndef UINT32
#define UINT32 INT32U
#endif

#ifndef INT32
#define INT32 INT32S
#endif

#if !defined( VOID )
#define VOID void
#endif

    /*--------------------------------------------------------------------------------*/
#ifndef NULL
#define NULL    ((void*)(0))
#endif

#ifndef  FALSE
#define  FALSE    (0)
#endif

#ifndef  TRUE
#define  TRUE     (1)
#endif

    /*--------------------------------------------------------------------------------*/
#ifndef SIZEOF
#define SIZEOF(x)       sizeof(x)
#endif

#ifndef ARRSIZE /* 取得数组的元素个数 */
#define ARRSIZE( array ) ( sizeof(array)/sizeof(array[0]) )
#endif

#if !defined( UNUSED )
#define UNUSED(x)   ( x = x )
#endif

    /* for function param */
#ifndef IN_OUT
#define IN_OUT
#define IN  //input param 
#define OUT //output param 
#endif

//******************************************************************************
#ifndef ALIGENSIZE
#define ALIGENSIZE( x , n )             ( ( (x)+(n)-1)  & (~((n)-1))  )
#define ALIGENSIZE4( x  )              ALIGENSIZE( x , 4 )
#endif

    /********************************************************************************/
    /* from dmr */
#ifndef ISODD
#define ISODD(x) ((x)&1)
#endif

//#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32-(n))))
//#define ROTATE_RIGHT(x, n) (((x) >> (n)) | ((x) << (32-(n))))
//#define WORDSWAP(d) ( ( (d) >> 16 ) + ( (d) << 16 ) )


    /* Would be good to have a test for platform.  If the platform isn't Windows
     we should define __cdecl to be nothing.  We can't just use _MSC_VER though
     as there are other compilers for the Windows platforms. */

#define ALL_N_BITS_ON(n) (~((~0)<<(n)))     // ((1<<(n))-1)
    /*
    ** This macro will calculate the maximum value of an unsigned data type
    ** where the type is passed in as x.
    ** For example MAX_UNSIGNED_TYPE( DRM_BYTE ) will evaluate to 0xFF
    **      MAX_UNSIGNED_TYPE( DRM_DWORD ) will evaluate to 0xFFFFFFFF
    */
#define MAX_UNSIGNED_TYPE(x) (ALL_N_BITS_ON(SIZEOF(x)*BITS_PER_BYTE))

#define BITMASK(nbits)   ((1<<nbits)-1)


//改为 24 位--实际占用 32位
#define     IMAGE_BMP_BITCOUNT                          32 /*32//16*/

typedef INT16U              ALPHA_RGBDATA;
#if IMAGE_BMP_BITCOUNT == 16
    typedef INT16U          LCD_RGBDATA;
#else
    typedef INT32U          LCD_RGBDATA;
#endif

    typedef struct RECTtag
    {
        INT16S x;
        INT16S y;
        INT16U w;
        INT16U h;
    }     UHRECT;


    typedef struct _UHPOINT
    {
        INT16S          x;
        INT16S          y;
    }  UHPOINT , *PUHPOINT;

    typedef struct _TOUCHMSG
    {
        INT32U          TouchState;
        UHPOINT         TouchPoint;
    } TOUCHMSG , *PTOUCHMSG;


    typedef void (*VOIDFUNCPTR)(void);  /* ptr to function returning void */
    /****************************************************************/

#define FILED_OFFSET( type , a )         ((INT32U)&(((type*)0)->a))    //取得变量的 偏移地址.

//INT64U  BIT
#define UI64_BIT              ((INT64U)0x01ul)
    /****************************************************************/

    /*--------------------------------------*/
#ifdef __cplusplus
}
#endif
/******************************************************************************************************/
#endif /* not _SYS_TYPE_DEFINE_H_ */
/*--------------------------------end of file ------------------------------------------*/

