/********************************************************************************
*********************************************************************************
			COPYRIGHT (c)   2004 BY ROCK-CHIP FUZHOU
				--  ALL RIGHTS RESERVED  --

File Name:  flash.h
Author:     XUESHAN LIN
Created:    1st Dec 2008
Modified:
Revision:   1.00
********************************************************************************
********************************************************************************/
#ifndef _FLASH_H
#define _FLASH_H

//1可配置参数
#define     DUAL_PLANE      //定义是否使能DUAL PLANE
//#define     INTERLEAVE      //定义是否使能Interleave

#define 	MAX_FLASH_NUM			4			            /*最大支持的FLASH数*/
#define     MAX_REFLESH_LOG     10                      //定义需要更新的地址记录数
#define     DATA_LEN            (8192*2/4)              //数据块单位word
#define     SPARE_LEN           (256*2/4)               //校验数据长度
#define     PAGE_SIZE           (DATA_LEN+SPARE_LEN)    //每个数据单位的长度

#define     CHIP_SIGN           0x38324B52              //RK28

/*******************************************************************
宏常数定义
*******************************************************************/


/*******************************************************************
厂商ID编码
*******************************************************************/
#define     SAMSUNG             0x00		//三星SAMSUNG
#define     TOSHIBA             0x01		//东芝TOSHIBA
#define     HYNIX               0x02		//海力士HYNIX
#define     INFINEON            0x03		//英飞凌INFINEON
#define     MICRON              0x04		//美光MICRON
#define     RENESAS             0x05		//瑞萨RENESAS
#define     ST                  0x06		//意法半导体ST
#define     INTEL               0x07		//英特尔intel

/*******************************************************************
器件ID编码
*******************************************************************/
#define     Small64M            0x00
#define     Small128M           0x01
#define     Large128M           0x02
#define     Large256M           0x03
#define     Large512M           0x04
#define     Large1G             0x05
#define     Large2G             0x06


/*******************************************************************
FLASH通过操作命令集(三星)
*******************************************************************/
#define     RESET_CMD               0xff
#define     READ_ID_CMD             0x90
#define     READ_UNIQUE_ID_CMD      0xed
#define     READ_STATUS_CMD         0x70
#define     READ_STATUS_CMD_TOSHIBA 0x71
#define     READ_STATUS_CMD_MICRON  0x78
#define     CHIP1_STATUS_CMD        0xf1
#define     CHIP2_STATUS_CMD        0xf2
#define     PAGE_PROG_CMD           0x8010
#define     PLANE2_PAGE_PROG_CMD    0x8111
#define     INTERLEAVE_PROG_CMD     0x8080
#define     BLOCK_ERASE_CMD         0x60d0

/*******************************************************************
LARGE PAGE FLASH操作命令集(三星)
*******************************************************************/
#define     READ_CMD                0x0030
#define     DUALPLANE_READ_CMD      0x6030
#define     READ_COPY_CMD           0x0035
#define     CACHE_PROG_CMD          0x8015
#define     COPY_PROG_CMD           0x8510
#define     RAND_DATAIN_CMD         0x85
#define     RAND_DATAOUT_CMD        0x05e0
#define     PLANE1_DATAOUT_CMD      0x06e0
/*******************************************************************
SMALL PAGE FLASH操作命令集(三星)
*******************************************************************/
#define     READ0_CMD               0x00
#define     READ1_CMD               0x01
#define     READ_SPARE_CMD          0x50
#define     SMALL_COPY_PROG_CMD     0x8A10

//BCHCTL寄存器
#define     BCH_WR                  0x0002
#define     BCH_RST                 0x0001
//FLCTL寄存器
#define     FL_RDY                  0x1000
#define     FL_COR_EN               0x0800
#define     FL_INT_EN               0x0400
#define     FL_XFER_EN              0x0200
#define     FL_INTCLR_EN            0x0100
#define     FL_START                0x0004
#define     FL_WR                   0x0002
#define     FL_RST                  0x0001

#define     FLASH_PROTECT_ON()      //write_XDATA32(FMCTL, ReadNandReg(FMCTL) & (~0x10))
#define     FLASH_PROTECT_OFF()     //write_XDATA32(FMCTL, ReadNandReg(FMCTL) | 0x10)

/*******************************************************************
FLASH读写接口
*******************************************************************/

//1结构定义
//chip interface reg
typedef volatile struct tagCHIP_IF
{
    uint32 data;
    uint32 addr;
    uint32 cmd;
    uint32 RESERVED[0x7d];
}CHIP_IF, *pCHIP_IF;

//NANDC Registers
typedef volatile struct tagNANDC
{
    uint32 FMCTL;
    uint32 FMWAIT;
    uint32 FLCTL;
    uint32 BCHCTL;
    uint32 RESERVED1[(0xd0-0x10)/4];
    uint32 BCHST;
    uint32 RESERVED2[(0x200-0xd4)/4];
    CHIP_IF chip[4];
    uint32 buf[0x800/4];
    uint32 spare[0x80/4];
} NANDC, *pNANDC;

typedef struct tagFLASH_SPEC
{
    uint8 	EccBits;			//ECC比特数
    uint8   CacheProg;          //是否支持cache program
    uint8   MulPlane;           //是否支持MultiPlane
    uint8   Interleave;         //是否支持Interleave
    uint8   Large;              //是否LargeBlock
    uint8   Five;               //是否有3个行地址
    uint8   MLC;                //是否MLC
    uint8   Vonder;             //厂商
    uint8   AccessTime;         //cycle time
    uint8   SecPerPage;         //FLASH DualPlane每PAGE扇区数
    uint8   SecPerPageRaw;		//FLASH原始每PAGE扇区数
    uint32	SecPerBlock;
    uint32	SecPerBlockRaw;
    uint32	PagePerBlock;
    uint32	Die2PageAddr;
    uint32  TotalPhySec;
    uint32	TotPhySec[MAX_FLASH_NUM];
} FLASH_SPEC, *pFLASH_SPEC;

typedef struct _FLASH_CMD
{
    uint8   Valid;      //指示是否有效
    uint16  Cmd;        //FLASH命令码
    uint32  ReadSec;    //物理扇区地址
    uint32  ErrSec;     //出错物理扇区地址
    uint8*  Buf;        //缓冲区
    uint16  Len;        //扇区数
} FLASH_CMD, *pFLASH_CMD;

/*ID BLOCK SECTOR 0 INFO*/
#ifndef LINUX
typedef __packed struct tagIDSEC0
#else
typedef struct tagIDSEC0
#endif
{
    uint32  magic;              //0x0ff0aa55, MASKROM限定不能更改
    uint8   reserved[8];
    uint16  bootCodeOffset1;    //中间件扇区偏移, MASKROM限定不能更改
    uint16  bootCodeOffset2;    //第二份中间件偏移
    uint8   reserved1[508-16];
    uint16  bootCodeSize;       //以SEC表示的中间件大小, MASKROM限定不能更改
    uint16  crc16;              //整个ID SEC的前512－2字节数据的CRC16
#ifndef LINUX
} IDSEC0, *pIDSEC0;
#else
}__attribute__((packed)) IDSEC0, *pIDSEC0;
#endif

/*ID BLOCK SECTOR 1 INFO*/
#ifndef LINUX
typedef __packed struct tagIDSEC1
#else
typedef struct tagIDSEC1
#endif
{
    uint16  sysAreaBlockRaw;        //系统保留块, 以原始Block为单位
    uint16  sysProgDiskCapacity;    //系统固件盘容量, 以MB为单位
    uint16  sysDataDiskCapacity;    //系统参数盘容量, 以MB为单位
    uint16  reserved0[2];           //保留部分填0
    uint16  chipSignL;              // 28
    uint16  chipSignH;              // RK
    uint16  MachineUIDL;            //机型UID,升级时匹配固件用
    uint16  MachineUIDH;
    uint16  year;                   //中间件生成日期年
    uint16  data;                   //中间件生成日期月日
    uint16  mainVer;                //主版本号
    uint8   reserved1[489-24];      //保留部分填0
    uint8   accessTime;             //读写cycle时间, ns
    uint16  blockSize;              //以SEC表示的BLOCK SIZE
    uint8   pageSize;               //以SEC表示的PAGE SIZE
    uint8   eccBits;                //需要的ECC BIT数, eg. 8/14
    uint16  bStartEIB;
    uint16  bEndEIB;
    uint16  bStartRB;
    uint16  bEndRB;
    uint16  idBlk[5];                //ID块位置
#ifndef LINUX
}IDSEC1, *pIDSEC1;
#else
}__attribute__((packed)) IDSEC1, *pIDSEC1;
#endif

#ifndef LINUX
typedef __packed struct _FLASH_INFO
#else
typedef struct _FLASH_INFO
#endif
{
    uint32  FlashSize;          //(Sector为单位)  4Byte
    uint16  BlockSize;          //(Sector为单位)  2Byte
    uint8   PageSize;           //(Sector为单位)  1Byte
    uint8   ECCBits;            //(bits为单位)    1Byte
    uint8   AccessTime;
    uint8   ManufacturerName;   // 1Byte
    uint8   FlashMask;          //每一bit代表那个片选是否有FLASH
#ifndef LINUX
} FLASH_INFO, *pFLASH_INFO;
#else
}__attribute__((packed)) FLASH_INFO, *pFLASH_INFO;
#endif

//1全局变量
#undef	EXT
#ifdef	IN_FLASH
#define	EXT
#else
#define	EXT		extern
#endif
EXT		FLASH_SPEC FlashSpec;
EXT		FLASH_CMD FlashPendCmd;
EXT		pNANDC  NandReg;       //NANDC寄存器
EXT		uint32 	RefleshSec[MAX_REFLESH_LOG];
EXT		uint32 	SysProgDiskCapacity;			//系统程序盘容量
EXT		uint32 	SysDataDiskCapacity;			//系统数据盘容量
EXT		uint16 	SysAreaBlockRaw;
EXT		uint8   FlashReadStatusCmd;
EXT		uint32  FlashSysProtMagic;
EXT		uint32 	FlashSysProtAddr;

//1函数原型声明
//flash.c
extern	void    FlashTimingCfg(uint32 AHBnKHz);
extern  void    FlashSysProtSet(uint32);
extern	uint32  FlashInit(void);
extern	uint32  FlashReadEnhanced(uint32 sec, void *pData, void *pSpare, uint8 nSec, uint32 nextSec);
extern	uint32  FlashProgPage(uint32 sec, void *pData, void *pSpare, uint8 nSec);
extern	uint32  FlashProgEnhanced(uint32 sec, void* buf, void* spare);
extern	uint32  FlashBlockErase(uint32 sec);
extern	uint32  FlashCopyProg(uint32 srcSec, uint32 destSec, uint16 nSec, void *pSpare);
extern uint32 FlashWaitBusy(uint32 sec, uint32 *secReplace);

//1表格定义
#ifdef	IN_FLASH
/*******************************
厂商ID表
********************************/
uint8 ManufactureIDTbl[]=
{
    0xec,					//三星SAMSUNG
    0x98,					//东芝TOSHIBA
    0xad,					//海力士HYNIX
    0xc1,					//英飞凌INFINEON
    0x2c,					//美光MICRON
    0x07,					//瑞萨RENESAS
    0x20,					//意法半导体ST
    0x89					//英特尔intel
};

/*******************************
器件ID表
********************************/
uint8 DeviceCode[]=
{
    0x76,					//small 8bit-64MB
    0x79,					//small 8bit-128MB
    0xF1,					//large 8bit-128M
    0xDA,					//large 8bit-256M
    0xDC,					//large 8bit-512M
    0xD3,					//large 8bit-1G
    0xD5,					//large 8bit-2G
    0xD7,					//large 8bit-4G
    0xD9,					//large 8bit-8G
    0x48,					//large 8bit-2G
    0x68,                   //large 8bit-4G
    0x88,                   //large 8bit-8G
};

/*******************************
器件信息表
********************************/
uint32 DeviceInfo[]=		//phisical sectors
{
    0x20000, 					// DI_64M,	small page
    0x40000, 					// DI_128M,	small page
    0x40000, 					// DI_128M,	large page
    0x80000,  					// DI_256M,	large page
    0x100000,					// DI_512M,	large page
    0x200000,					// DI_1G,   large page
    0x400000,					// DI_2G,   large page
    0x800000,					// DI_4G,   large page
    0x1000000,					// DI_8G,   large page
    0x400000,					// DI_2G,   large page
    0x800000,					// DI_4G,   large page
    0x1000000					// DI_8G,   large page
};

#if 0
const char* VonderDesc[]=
{
    "Samsung",
    "Toshiba",
    "Hynix",
    "Infineon",
    "Micron",
    "Renesas",
    "ST",
    "Intel"
};
#endif
#else
extern	uint8 	ManufactureIDTbl[];
extern	uint8 	DeviceCode[];
extern	uint32	DeviceInfo[];
#endif
#endif

