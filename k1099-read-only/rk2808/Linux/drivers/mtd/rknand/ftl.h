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


//1�����ò���
//#define     IN_LOADER                       //�������loaderʱ�꿪��, û������ϵͳ����
//#ifndef IN_LOADER
//#define     SYS_PROTECT                     //����ϵͳ��д����ʹ��
//#endif
#define   MALLOC_DISABLE
#define     FTL_VERSION             0x0200  //FTL�汾��, 0x100��ʾver1.00, ��Ҫ�͸�ʱ�޸����汾��, ����ֻ���޸Ĵΰ汾��
#define     MAX_REMAP_TBL           (32768*2)//����ӳ���, �ֽڵ�λ, ����>=1024
#define     MAX_BAD_BLK_NUM         256     //���ĵĻ��齻�����
#define     MAX_CACHE_BLK_NUM       16      //������CACHE�Ŀ���
#define     MAX_EXCH_BLK_NUM        8       //��������
#define     MAX_FREE_BLK_NUM        1024    //��������չ��������������
#define     MIN_FREE_BLK_NUM        64      //��������չ���������С����
#define     MAX_RSVD_BLK_NUM        4       //��������(����2��������¼��)

#define     DISK_NAND_CODE          0
#define     DISK_NAND_DATA          1
#define     DISK_NAND_USER          2
#define     DISK_NAND_TOTAL         0xff

#define     MAX_CACHE               16      //����Ҫ6��
#define     MAX_CACHE_CONTINUE_LEN  3

#define     FLASH_PROT_MAGIC        0x444e414e  //NAND

/*******************************************************************
�곣������
*******************************************************************/
#define     FTL_OK                  0
#define     FTL_ERROR               -1

#define     SIGN_BAD_BLK            0xf000
#define     SIGN_CACHE_BLK          0xf100
#define     SIGN_DATA_BLK           0xf200
#define     SIGN_RCV_BLK            0xf300


//1�ṹ����
typedef struct tagEXCH_BLK_INFO
{
    uint32	Start;
    uint32	End;
    uint32	SrcAddr;		//ԴBLOCK��ַ, ���ڹرջ�ָ�
    uint16  ver;            //�汾��, ��ʾ��������
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
    uint8 valid;    //FTL��Ч
    uint16 ftlVer;  //FTL�汾��
    
    uint8 secPerPage;   //FLASH��page size
    uint16 secPerBlk;   //FLASH��block size
    uint16 startPhyBlk; //FLASH��Ϊ����������ʼ��
    uint32 maxPhyBlk;   //FLASH��Ϊ������������
    uint16 totalLogicBlk;

    REMAP_INFO remapInfo;           //��ӳ���, ��������С����, e.g. 32GB/(1MB/blk)=64KB
    BAD_BLK_INFO badBlkInfo;        //������Ϣ
    CACHE_BLK_INFO cacheBlkInfo;    //CACHE����Ϣ
    CIR_QUEUE freeBlkInfo;          //�տ��, �����̿ռ���ʱ���ٱ�֤��64����Թ���
    EXCH_BLK_INFO exchBlk[MAX_EXCH_BLK_NUM];//������

    uint8 cacheSortList[MAX_CACHE+1];
    pFTL_CACHE curCache;
    FTL_CACHE cache[MAX_CACHE];
}FTL_INFO, *pFTL_INFO;

//1ȫ�ֱ���
#undef	EXT
#ifdef	IN_FTL
#define	EXT
#else
#define	EXT		extern
#endif
EXT     FTL_INFO gFtlInfo;
EXT     uint32 gCacheBuf[MAX_CACHE][PAGE_SIZE];
EXT    uint32  CurEraseBlock;

//1����ԭ������
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

//1�����
#ifdef	IN_FTL
#else
#endif
#endif

