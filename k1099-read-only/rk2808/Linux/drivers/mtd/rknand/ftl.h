/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)   2004 BY ROCK-CHIP FUZHOU
				--  ALL RIGHTS RESERVED  --

File Name:  ftl.h
Author:     XUESHAN LIN
Created:    1st Dec 2008
Modified:
Revision:   1.00
********************************************************************************
********************************************************************************/
#ifndef _FTL_H
#define _FTL_H


//1可配置参数
//#define     IN_LOADER                       //定义编译loader时宏开关, 没定义是系统驱动
//#ifndef IN_LOADER
//#define     SYS_PROTECT                     //定义系统区写保护使能
//#endif
#define   MALLOC_DISABLE
#define     FTL_VERSION             0x0200  //FTL版本号, 0x100表示ver1.00, 需要低格时修改主版本号, 否则只需修改次版本号
#define     MAX_REMAP_TBL           (32768*2)//最大的映射表, 字节单位, 必须>=1024
#define     MAX_BAD_BLK_NUM         256     //最大的的坏块交替块数
#define     MAX_CACHE_BLK_NUM       16      //用来作CACHE的块数
#define     MAX_EXCH_BLK_NUM        8       //交换块数
#define     MAX_FREE_BLK_NUM        1024    //用来作扩展块的数量最大限制
#define     MIN_FREE_BLK_NUM        64      //用来作扩展块的数量最小限制
#define     MAX_RSVD_BLK_NUM        4       //保留块数(包含2个坏块表记录块)

#define     DISK_NAND_CODE          0
#define     DISK_NAND_DATA          1
#define     DISK_NAND_USER          2
#define     DISK_NAND_TOTAL         0xff

#define     MAX_CACHE               16      //至少要6条
#define     MAX_CACHE_CONTINUE_LEN  3

#define     FLASH_PROT_MAGIC        0x444e414e  //NAND

/*******************************************************************
宏常数定义
*******************************************************************/
#define     FTL_OK                  0
#define     FTL_ERROR               -1

#define     SIGN_BAD_BLK            0xf000
#define     SIGN_CACHE_BLK          0xf100
#define     SIGN_DATA_BLK           0xf200
#define     SIGN_RCV_BLK            0xf300


//1结构定义
typedef struct tagEXCH_BLK_INFO
{
    uint32	Start;
    uint32	End;
    uint32	SrcAddr;		//源BLOCK地址, 用于关闭或恢复
    uint16  ver;            //版本号, 表示最新数据
    uint8	Count;
    uint8	Valid;
}EXCH_BLK_INFO, *pEXCH_BLK_INFO;

typedef struct tagFTL_CACHE
{
    uint32 LBA;
    uint32 Mask;
    uint8 count;
    uint8 Valid;
    uint32 *Buf;
}FTL_CACHE, *pFTL_CACHE;

typedef struct tagREMAP_INFO
{
    uint8 num[64];
    uint32 write[2];
    uint16 max;
    uint16 blkAddr;
    uint16 pageAddr;
    uint16 tbl[MAX_REMAP_TBL/2];
}REMAP_INFO, *pREMAP_INFO;

typedef struct tagBAD_BLK_INFO
{
    uint16 max;
    uint16 cnt;
    uint16 offset;
    uint16 blkAddr[4];
    uint16 tbl[MAX_BAD_BLK_NUM];
}BAD_BLK_INFO, *pBAD_BLK_INFO;

typedef struct tagCACHE_BLK_INFO
{
    uint32 curPageAddr;
    uint32 curBlkAddr;
    uint16 ver;
    uint16 tbl[MAX_CACHE_BLK_NUM];
}CACHE_BLK_INFO, *pCACHE_BLK_INFO;

typedef struct tagCIR_QUEUE
{
    uint16 max;
	uint16 front;
	uint16 rear;
	uint16 count;
	uint16 arr[MAX_FREE_BLK_NUM];
}CIR_QUEUE, *pCIR_QUEUE;


typedef struct tagFTL_INFO
{
    uint8 valid;    //FTL有效
    uint16 ftlVer;  //FTL版本号
    
    uint8 secPerPage;   //FLASH的page size
    uint16 secPerBlk;   //FLASH的block size
    uint16 startPhyBlk; //FLASH作为数据区的起始块
    uint32 maxPhyBlk;   //FLASH作为数据区的最大块
    uint16 totalLogicBlk;

    REMAP_INFO remapInfo;           //块映射表, 随容量大小而变, e.g. 32GB/(1MB/blk)=64KB
    BAD_BLK_INFO badBlkInfo;        //坏块信息
    CACHE_BLK_INFO cacheBlkInfo;    //CACHE块信息
    CIR_QUEUE freeBlkInfo;          //空块表, 当磁盘空间满时至少保证有64块可以滚动
    EXCH_BLK_INFO exchBlk[MAX_EXCH_BLK_NUM];//交换块

    uint8 cacheSortList[MAX_CACHE+1];
    pFTL_CACHE curCache;
    FTL_CACHE cache[MAX_CACHE];
}FTL_INFO, *pFTL_INFO;

//1全局变量
#undef	EXT
#ifdef	IN_FTL
#define	EXT
#else
#define	EXT		extern
#endif
EXT     FTL_INFO gFtlInfo;
EXT     uint32 gCacheBuf[MAX_CACHE][PAGE_SIZE];
EXT    uint32  CurEraseBlock;

//1函数原型声明
//FTL.c
extern	int32   FTLInit(void);
extern	uint16  GetRemapBlk(uint16 LBA, uint16 PBA);
extern	uint32  GetRemap(uint32 LBA, uint8 mod);
extern  uint32  GetUnremap(uint32 PBA);

extern	uint32 	FlashProgError(uint32 secAddr, void *DataBuf, uint16 nSec);

extern	void    FtlCacheDelayHook(void);
extern	uint32  FtlGetCapacity(uint8 LUN);
extern	int32   FtlRead(uint8 LUN, uint32 Index, uint32 nSec, void *buf);
extern	uint32  FtlWritePage(uint32 Index, void *pData);
extern	int32   FtlWrite(uint8 LUN, uint32 Index, uint32 nSec, void *buf);
extern void FtlFlashSysProtSetOn(void);
extern void FtlFlashSysProtSetOff(void);
extern void FtlLowFormatSysDisk(void);

//1表格定义
#ifdef	IN_FTL
#else
#endif
#endif

