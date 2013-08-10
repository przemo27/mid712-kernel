/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved.  	  */
/*******************************************************************
File 	:	lcdc.h
Desc 	:	定义LCDC的寄存器结构体\寄存器位的宏定义
Author 	:  	dukunming
Date 	:	2008-12-29
Notes 	:
********************************************************************/
#ifndef _REG_LCDC_H
#define _REG_LCDC_H

/********************************************************************
**                            宏定义                                *
********************************************************************/
/* 输往屏的数据格式 */
#define OUT_P888            0
#define OUT_P666            1
#define OUT_P565            2
#define OUT_S888x           4
#define OUT_CCIR656         6
#define OUT_S888            8
#define OUT_S888DUMY        12

/* Low Bits Mask */
#define m_WORDLO            (0xffff<<0)
#define m_WORDHI            (0xffff<<16)
#define v_WORDLO(x)         (((x)&0xffff)<<0)
#define v_WORDHI(x)         (((x)&0xffff)<<16)

#define m_BIT11LO           (0x7ff<<0)
#define m_BIT11HI           (0x7ff<<16)
#define v_BIT11LO(x)        (((x)&0x7ff)<<0)
#define v_BIT11HI(x)        (((x)&0x7ff)<<16)


/* SYS_CONFIG */
#define m_W0_MASTER         (1<<0)
#define m_W1_MASTER         (1<<1)
#define m_W0_INFMT          (7<<2)
#define m_W1_A0FMT          (1<<5)
#define m_W1_A1FMT          (1<<6)
#define m_W1_A2FMT          (1<<7)
#define m_W1_A3FMT          (1<<8)
#define m_W0_ENC            (1<<9)
#define m_W1_A0ENC          (1<<11)
#define m_W1_A1ENC          (1<<12)
#define m_W1_A2ENC          (1<<13)
#define m_W1_A3ENC          (1<<14)
#define m_AHB_JUMP          (1<<15)
#define m_AHB_FIELD         (1<<16)
#define m_AHB_INCR          (31<<17)
#define m_AHB_WRITE         (1<<22)
#define m_AHB_SIZE          (7<<23)
#define m_AHB_BURST         (7<<26)
#define m_MCU_RS            (1<<29)
#define m_MCU_BYPASS        (1<<30)
#define m_MCU_SELECT        (1<<31)
#define v_W0_MASTER(x)      (((x)&1)<<0)
#define v_W1_MASTER(x)      (((x)&1)<<1)
#define v_W0_INFMT(x)       (((x)&7)<<2)
#define v_W1_A0FMT(x)       (((x)&1)<<5)
#define v_W1_A1FMT(x)       (((x)&1)<<6)
#define v_W1_A2FMT(x)       (((x)&1)<<7)
#define v_W1_A3FMT(x)       (((x)&1)<<8)
#define v_W0_ENC(x)         (((x)&1)<<9)
#define v_W1_A0ENC(x)       (((x)&1)<<11)
#define v_W1_A1ENC(x)       (((x)&1)<<12)
#define v_W1_A2ENC(x)       (((x)&1)<<13)
#define v_W1_A3ENC(x)       (((x)&1)<<14)
#define v_AHB_JUMP(x)       (((x)&1)<<15)
#define v_AHB_FIELD(x)      (((x)&1)<<16)
#define v_AHB_INCR(x)       (((x)&31)<<17)
#define v_AHB_WRITE(x)      (((x)&1)<<22)
#define v_AHB_SIZE(x)       (((x)&7)<<23)
#define v_AHB_BURST(x)      (((x)&7)<<26)
#define v_MCU_RS(x)         (((x)&1)<<29)
#define v_MCU_BYPASS(x)     (((x)&1)<<30)
#define v_MCU_SELECT(x)     (((x)&1)<<31)

/* WIN0_VIR */
#define m_W0_VIRWIDTH       (0xffff<<0)
#define m_H_DEFLICK         (3<<19)
#define m_V_DEFLICK         (3<<21)
#define m_TRISTATE_OUT      (1<<23)
#define v_W0_VIRWIDTH(x)    (((x)&0xffff)<<0)
#define v_H_DEFLICK(x)      (((x)&3)<<19)
#define v_V_DEFLICK(x)      (((x)&3)<<21)
#define v_TRISTATE_OUT(x)   (((x)&1)<<23)


// INT_LUT
#define m_W1_A3EMPTY        (1<<0)
#define m_W1_A2EMPTY        (1<<1)
#define m_W1_A1EMPTY        (1<<2)
#define m_W1_A0EMPTY        (1<<3)
#define m_W0_CBREMPTY       (1<<4)
#define m_W0_YRGBMPTY       (1<<5)
#define m_HOR_START         (1<<6)
#define m_FRM_START         (1<<7)
#define m_AHB_ERR           (1<<8)
#define m_AHB_ERRMASK       (1<<14)
#define m_AHB_ERRCLEAR      (1<<15)
#define m_W1A0_ETYMASK      (1<<16)
#define m_W1A1_ETYMASK      (1<<17)
#define m_W1A2_ETYMASK      (1<<18)
#define m_W1A3_ETYMASK      (1<<19)
#define m_W0CBR_ETYMASK     (1<<20)
#define m_W0YRGB_MTYMASK    (1<<21)
#define m_HOR_STARTMASK     (1<<22)
#define m_FRM_STARTMASK     (1<<23)
#define m_W1A0_ETYCLEAR     (1<<24)
#define m_W1A1_ETYCLEAR     (1<<25)
#define m_W1A2_ETYCLEAR     (1<<26)
#define m_W1A3_ETYCLEAR     (1<<27)
#define m_W0CBR_ETYCLEAR    (1<<28)
#define m_W0YRGB_MTYCLEAR   (1<<29)
#define m_HOR_STARTCLEAR    (1<<30)
#define m_FRM_STARTCLEAR    (1<<31)
#define v_W1_A3EMPTY(x)        (((x)&1)<<0)
#define v_W1_A2EMPTY(x)        (((x)&1)<<1)
#define v_W1_A1EMPTY(x)        (((x)&1)<<2)
#define v_W1_A0EMPTY(x)        (((x)&1)<<3)
#define v_W0_CBREMPTY(x)       (((x)&1)<<4)
#define v_W0_YRGBMPTY(x)       (((x)&1)<<5)
#define v_HOR_START(x)         (((x)&1)<<6)
#define v_FRM_START(x)         (((x)&1)<<7)
#define v_AHB_ERR(x)           (((x)&1)<<8)
#define v_AHB_ERRMASK(x)       (((x)&1)<<14)
#define v_AHB_ERRCLEAR(x)      (((x)&1)<<15)
#define v_W1A0_ETYMASK(x)      (((x)&1)<<16)
#define v_W1A1_ETYMASK(x)      (((x)&1)<<17)
#define v_W1A2_ETYMASK(x)      (((x)&1)<<18)
#define v_W1A3_ETYMASK(x)      (((x)&1)<<19)
#define v_W0CBR_ETYMASK(x)     (((x)&1)<<20)
#define v_W0YRGB_MTYMASK(x)    (((x)&1)<<21)
#define v_HOR_STARTMASK(x)     (((x)&1)<<22)
#define v_FRM_STARTMASK(x)     (((x)&1)<<23)
#define v_W1A0_ETYCLEAR(x)     (((x)&1)<<24)
#define v_W1A1_ETYCLEAR(x)     (((x)&1)<<25)
#define v_W1A2_ETYCLEAR(x)     (((x)&1)<<26)
#define v_W1A3_ETYCLEAR(x)     (((x)&1)<<27)
#define v_W0CBR_ETYCLEAR(x)    (((x)&1)<<28)
#define v_W0YRGB_MTYCLEAR(x)   (((x)&1)<<29)
#define v_HOR_STARTCLEAR(x)    (((x)&1)<<30)
#define v_FRM_STARTCLEAR(x)    (((x)&1)<<31)


/* DSP_CTRL_REG0 */
#define m_BLACK_OUT         (1<<0)
#define m_INTER_LACE        (1<<1)
#define m_RB_SWAP           (1<<2)
#define m_RG_SWAP           (1<<3)
#define m_DIS_FMT0          (7<<4)
#define m_TOPWIN0           (1<<7)
#define m_H_PINPOL          (1<<8)
#define m_V_PINPOL          (1<<9)
#define m_DEN_PINPOL        (1<<10)
#define m_DCLK_PINPOL       (1<<11)
#define m_W1A0_TRSPREG      (1<<12)
#define m_W1A1_TRSPREG      (1<<13)
#define m_W1A2_TRSPREG      (1<<14)
#define m_W1A3_TRSPREG      (1<<15)
#define m_W1A0_TRSPVAL      (15<<16)
#define m_W1A1_TRSPVAL      (15<<20)
#define m_W1A2_TRSPVAL      (15<<24)
#define m_W1A3_TRSPVAL      (15<<28)
#define v_BLACK_OUT(x)      (((x)&1)<<0)
#define v_INTER_LACE(x)     (((x)&1)<<1)
#define v_RB_SWAP(x)        (((x)&1)<<2)
#define v_RG_SWAP(x)        (((x)&1)<<3)
#define v_DIS_FMT0(x)       (((x)&7)<<4)
#define v_TOPWIN0(x)        (((x)&1)<<7)
#define v_H_PINPOL(x)       (((x)&1)<<8)
#define v_V_PINPOL(x)       (((x)&1)<<9)
#define v_DEN_PINPOL(x)     (((x)&1)<<10)
#define v_DCLK_PINPOL(x)    (((x)&1)<<11)
#define v_W1A0_TRSPREG(x)   (((x)&1)<<12)
#define v_W1A1_TRSPREG(x)   (((x)&1)<<13)
#define v_W1A2_TRSPREG(x)   (((x)&1)<<14)
#define v_W1A3_TRSPREG(x)   (((x)&1)<<15)
#define v_W1A0_TRSPVAL(x)   (((x)&15)<<16)
#define v_W1A1_TRSPVAL(x)   (((x)&15)<<20)
#define v_W1A2_TRSPVAL(x)   (((x)&15)<<24)
#define v_W1A3_TRSPVAL(x)   (((x)&15)<<28)


/* DSP_CTRL_REG1 */
#define m_BLACK_RGB             (0xffffff<<0)
#define m_W1A0_TRSPRAM          (1<<24)         //only used in RGB888
#define m_W1A1_TRSPRAM          (1<<25)
#define m_W1A2_TRSPRAM          (1<<26)
#define m_W1A3_TRSPRAM          (1<<27)
#define m_SWAP_DELTA            (1<<28)
#define m_SCALE_DROPLINE        (1<<29)
#define m_SWAP_DUMY             (1<<30)
#define m_BLANK_OUT             (1<<31)
#define v_BLACK_RGB(x)          (((x)&0xffffff)<<0)
#define v_W1A0_TRSPRAM(x)       (((x)&1)<<24)
#define v_W1A1_TRSPRAM(x)       (((x)&1)<<25)
#define v_W1A2_TRSPRAM(x)       (((x)&1)<<26)
#define v_W1A3_TRSPRAM(x)       (((x)&1)<<27)
#define v_SWAP_DELTA(x)         (((x)&1)<<28)
#define v_SCALE_DROPLINE(x)     (((x)&1)<<29)
#define v_SWAP_DUMY(x)          (((x)&1)<<30)
#define v_BLANK_OUT(x)          (((x)&1)<<31)


// WIN1_WATERMARK (swap on ahb)
#define m_WATER_MARK            (0xff<<0)
#define m_W1A3_RBSWAP           (1<<8)          // used in RGB565
#define m_W1A2_RBSWAP           (1<<9)
#define m_W1A1_RBSWAP           (1<<10)
#define m_W1A0_RBSWAP           (1<<11)
#define m_W0_RBSWAP             (1<<12)
#define m_W0YRGB_MID8SWAP       (1<<13)
#define m_W0YRGB_LOOPSWAP       (1<<14)
#define m_W0CBR_LOOPSWAP        (1<<15)
#define m_W0YRGB_16SWAP         (1<<16)
#define m_W0YRGB_8SWAP          (1<<17)
#define m_W0CBR_16SWAP          (1<<18)
#define m_W0CBR_8SWAP           (1<<19)
#define m_W1A0_16SWAP           (1<<20)
#define m_W1A0_8SWAP            (1<<21)
#define m_W1A1_16SWAP           (1<<22)
#define m_W1A1_8SWAP            (1<<23)
#define m_W1A2_16SWAP           (1<<24)
#define m_W1A2_8SWAP            (1<<25)
#define m_W1A3_16SWAP           (1<<26)
#define m_W1A3_8SWAP            (1<<27)
#define m_W1A0_LOOPSWAP         (1<<28)
#define m_W1A1_LOOPSWAP         (1<<29)
#define m_W1A2_LOOPSWAP         (1<<30)
#define m_W1A3_LOOPSWAP         (1<<31)
#define v_WATER_MARK(x)         (((x)&0xff)<<0)
#define v_W1A3_RBSWAP(x)        (((x)&1)<<8)
#define v_W1A2_RBSWAP(x)        (((x)&1)<<9)
#define v_W1A1_RBSWAP(x)        (((x)&1)<<10)
#define v_W1A0_RBSWAP(x)        (((x)&1)<<11)
#define v_W0_RBSWAP(x)          (((x)&1)<<12)
#define v_W0YRGB_MID8SWAP(x)    (((x)&1)<<13)
#define v_W0YRGB_LOOPSWAP(x)    (((x)&1)<<14)
#define v_W0CBR_LOOPSWAP(x)     (((x)&1)<<15)
#define v_W0YRGB_16SWAP(x)      (((x)&1)<<16)
#define v_W0YRGB_8SWAP(x)       (((x)&1)<<17)
#define v_W0CBR_16SWAP(x)       (((x)&1)<<18)
#define v_W0CBR_8SWAP(x)        (((x)&1)<<19)
#define v_W1A0_16SWAP(x)        (((x)&1)<<20)
#define v_W1A0_8SWAP(x)         (((x)&1)<<21)
#define v_W1A1_16SWAP(x)        (((x)&1)<<22)
#define v_W1A1_8SWAP(x)         (((x)&1)<<23)
#define v_W1A2_16SWAP(x)        (((x)&1)<<24)
#define v_W1A2_8SWAP(x)         (((x)&1)<<25)
#define v_W1A3_16SWAP(x)        (((x)&1)<<26)
#define v_W1A3_8SWAP(x)         (((x)&1)<<27)
#define v_W1A0_LOOPSWAP(x)      (((x)&1)<<28)
#define v_W1A1_LOOPSWAP(x)      (((x)&1)<<29)
#define v_W1A2_LOOPSWAP(x)      (((x)&1)<<30)
#define v_W1A3_LOOPSWAP(x)      (((x)&1)<<31)


/* MCU_TIMING_CTRL */
#define m_MCU_TOTAL             (31<<0)
#define m_MCU_CSSTART           (31<<5)
#define m_MCU_CSEND             (31<<10)
#define m_MCU_RWSTART           (31<<15)
#define m_MCU_RWEND             (31<<20)
#define m_BG_SWAP               (1<<25)
#define m_DIS_FMT1              (1<<26)
#define m_JPG_COLORSPACE        (1<<27)
#define m_YUV2RGB_BYPASS        (1<<28)
#define m_MCU_HOLDED            (1<<29)
#define m_MCU_HOLDMODE          (1<<30)
#define m_MCU_HOLDSIGNAL        (1<<31)
#define v_MCU_TOTAL(x)          (((x)&31)<<0)
#define v_MCU_CSSTART(x)        (((x)&31)<<5)
#define v_MCU_CSEND(x)          (((x)&31)<<10)
#define v_MCU_RWSTART(x)        (((x)&31)<<15)
#define v_MCU_RWEND(x)          (((x)&31)<<20)
#define v_BG_SWAP(x)            (((x)&1)<<25)
#define v_DIS_FMT1(x)           (((x)&1)<<26)
#define v_JPG_COLORSPACE(x)     (((x)&1)<<27)
#define v_YUV2RGB_BYPASS(x)     (((x)&1)<<28)
#define v_MCU_HOLDED(x)         (((x)&1)<<29)
#define v_MCU_HOLDMODE(x)       (((x)&1)<<30)
#define v_MCU_HOLDSIGNAL(x)     (((x)&1)<<31)


/* RAM_CEN_CTRL */
#define m_RAM1BUF0_CEN          (1<<0)
#define m_RAM1BUF1_CEN          (1<<1)
#define m_DEFLICK_EN            (1<<2)
#define m_W1A0_KEYCOLOR         (1<<4)
#define m_W1A1_KEYCOLOR         (1<<5)
#define m_W1A2_KEYCOLOR         (1<<6)
#define m_W1A3_KEYCOLOR         (1<<7)
#define m_KEY_COLOR             (0xffffff<<8)
#define v_RAM1BUF0_CEN(x)       (((x)&1)<<0)
#define v_RAM1BUF1_CEN(x)       (((x)&1)<<1)
#define v_DEFLICK_EN(x)         (((x)&1)<<2)
#define v_MCUBYPASS_REMAP(x)    (((x)&1)<<3)
#define v_KEY_COLOR(x)          (((x)&0xffffff)<<8)


/********************************************************************
**                          结构定义                                *
********************************************************************/
/* LCDC的寄存器结构 */
typedef volatile struct tagLCDC_REG
{
    /* offset 0x00~0xa4 */
    unsigned int SYS_CONFIG;              //SYSTEM configure register
    unsigned int WIN0_VIR;                //WIN0 VIRTUAL DISPLAY Width
    unsigned int WIN0_YRGB_MST;           //Win0 YRGB memory start address
    unsigned int WIN0_CBR_MST;            //Win0 Cbr memory start address
    unsigned int INT_LUT;                 //Interrupt lookup table
    unsigned int WIN0_ACT_INFO;           //Win0 active window widht and height
    unsigned int WIN1_VIR0;               //Win1 AREA0 and  AREA1 Virtual display width
    unsigned int WIN1_VIR1;               //Win1 AREA2 and  AREA3 Virtual display width
    unsigned int WIN1_AREA0_MST;          //Win1 AREA0 memory start address
    unsigned int WIN1_AREA1_MST;          //Win1 AREA1 memory start address
    unsigned int WIN1_AREA2_MST;          //Win1 AREA2 memory start address
    unsigned int WIN1_AREA3_MST;          //Win1 AREA3 memory start address
    unsigned int DSP_HTOTAL ;             //Dsiplay pannel horizontal total
    unsigned int DSP_HS_END;              //Display Pannel hsync end point
    unsigned int DSP_HACT_ST;             //Display Pannle horizontal enable start point
    unsigned int DSP_HACT_END;            //Display Pannel horizontal enable end point
    unsigned int DSP_VTOTAL;              //Dsplay Pannel verital total
    unsigned int DSP_VS_END;              //Display Pannel vertical sync end point, start point is always from0
    unsigned int DSP_VACT_ST;             //Display Pannel veritcal enable start line
    unsigned int DSP_VACT_END;            //Display Pannel vertical enable end line
    unsigned int DSP_VS_ST_F1;            //Display Pannle filed1 start line, only in Interlace Mode
    unsigned int DSP_VS_END_F1;           //Display Pannel field1 end line, only in Interlce Mode
    unsigned int DSP_VACT_ST_F1;          //Display Pannel field 1 vertical enable start line, only in Interlce Mode
    unsigned int DSP_VACT_END_F1;         //Dsiplay Pannel filed 1 vertical enable end line, Only in Interlace Mode
    unsigned int DSP_WIN0_ST;             //Display win0 horizontal and veritical start point on the pannel
    unsigned int DSP_WIN0_INFO;           //Display win0 width and height on the pannel
    unsigned int SD_FACTOR;               //Horizontal and vertical scale down factor setting
    unsigned int SP_FACTOR;               //Horizontal and vertical scale up factor setting
    unsigned int DSP_WIN1_AREA0_ST;       //Win1 AREA0 horizontal and vertical start point on Pannel
    unsigned int DSP_WIN1_AREA1_ST;       //Win1 AREA1 horizontal and vertical start point on Pannel
    unsigned int DSP_WIN1_AREA2_ST;       //Win1 AREA2 horizontal and vertical start point on Pannel
    unsigned int DSP_WIN1_AREA3_ST;       //Win1 AREA3 horizontal and vertical start point on Pannel
    unsigned int DSP_WIN1_AREA0_INFO;     //Win1 AREA0 horizontal width and vertical height on Pannel
    unsigned int DSP_WIN1_AREA1_INFO;     //Win1 AREA1 horizontal width and vertical height on Pannel
    unsigned int DSP_WIN1_AREA2_INFO;     //Win1 AREA2 horizontal width and vertical height on Pannel
    unsigned int DSP_WIN1_AREA3_INFO;     //Win1 AREA3 horizontal width and vertical height on Pannel
    unsigned int REG_CFG_DONE;            //REGISTER CONFIG FINISH REGISTER
    unsigned int DSP_CTRL_REG0;           //Display control register0
    unsigned int DSP_CTRL_REG1;           //Display Control register1
    unsigned int WIN1_WATERMARK;          //Win1 display watermark
    unsigned int MCU_TIMING_CTRL;         //MCU display timing control register
    unsigned int RAM_CEN_CTRL;            //RAM CEN Control Register

    unsigned int reserved0[(0x100-0xa8)/4];

    /* offset 0x0100~0x0700 */
    unsigned int WIN0_YRGB_WPORT;                 //WIN0 YRGB DATA Write only PORT
    unsigned int reserved1[(0x200-0x104)/4];

    unsigned int WIN0_CBR_WPORT;                  //WIIN0 Cbr Data Write only port
    unsigned int reserved2[(0x300-0x204)/4];

    unsigned int WIN1_AREA0_WPORT;                //WIN1 AREA0 Data Write Only Port
    unsigned int reserved3[(0x400-0x304)/4];

    unsigned int WIN1_AREA1_WPORT;                //WIN1 AREA1 Data Write Only Port
    unsigned int reserved4[(0x500-0x404)/4];

    unsigned int WIN1_AREA2_WPORT;                //WIN1 AREA2 Data Write Only Port
    unsigned int reserved5[(0x600-0x504)/4];

    unsigned int WIN1_AREA3_WPORT;                //WIN1 AREA3 Data Write Only Port
    unsigned int reserved6[(0x700-0x604)/4];

    unsigned int MCU_BYPASS_WPORT;                //MCU BYPASS MODE, Data Write Only Port

} LCDC_REG, *pLCDC_REG;





#endif






