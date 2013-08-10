/*
 * rockchip_dsp.h  --  Dsp for rockchip
 *
 * Driver for rockchip dsp
 *  Copyright (C) 2009 lhh lhh@rock-chips.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#ifndef __PLAT_ROCKCHIP_DSP_DSP_H
#define __PLAT_ROCKCHIP_DSP_DSP_H


#define PIU_CMD0_OFFSET         (0x30)
#define PIU_REPLY0_OFFSET       (0x3c)
#define PIU_STATUS_OFFSET       (0x4c)
#define PIU_IMASK_OFFSET        (0x48)
#define PIU_STATUS_R0WRS        3

#define DSP_IOCTL_DOWNLOAD_FIRMWARE     (0x00810000)

/*more format*/
#define DSP_IOCTL_DECODE_INPUT_ONE_BUFF (0x00800001)
#define DSP_IOCTL_DECODE_EVENT          (0x00800002)
#define DSP_IOCTL_DECODE_EVENT_SYSN     (0x00800003)
#define DSP_IOCTL_ALLOC_BUFFER          (0x00800004)
#define DSP_IOCTL_RECE_OUT_BUFF         (0x00800005)
#define DSP_IOCTL_FREE_OUT_BUFF         (0x00800006)
#define DSP_IOCTL_STATE_SET             (0x00800007)
#define DSP_IOCTL_JPG_GET_DATA			(0x00800008)
#define DSP_IOCTL_JPG_GET_REBACK        (0x00800009)

#define DSP_IOCTL_DECODE_EXIT          (0x00800010)
#define DSP_IOCTL_SEND_DECODE_EXIT     (0x00800011)
#define DSP_IOCTL_END_ONE_DECODE_EXIT  (0x00800012)

#define DSP_IOCTL_DECODE_TEST          (0x00800015)
#define DSP_IOCTL_GET_SIZE		       (0x00800016)

#define DSP_IOCTL_SET_FREQ		       (0x00800017)

#define DSP_IOCTL_SET_BUS               (0x00800020)


/*WORK ID EVENT*/
#define EXIT_DSP_OFF_ID                 0x00000000
#define DOWNLOAD_FIRMWARE_ID            0x00000001
#define ALLOC_BUFF_ID                   0x00000002
#define DECODEING_ID                    0x00000003
#define EXIT_WAITING_ID                 0x00000004

#define CAM_WIDTH   640
#define CAM_HIGHT   480

typedef enum XMVCodecReply 
{
	MSG_CODEC_ERROR,
	MSG_PREPARE_BITSTREAM,
	MSG_UPDATE_PARAMETERS,
	MSG_SHARE_POINTER_REQ
} XMVCodecReply;

typedef enum XMVCodecCMD
{
    XM_CODEC_CMD_PARAM,
    XM_CODEC_CMD_CLOSE
}XMVCodecCMD;

typedef enum XMVCodecState
{
    XM_CODEC_STATE_UNLOADED,
    XM_CODEC_STATE_LOADED,
    XM_CODEC_STATE_PAUSED,
    XM_CODEC_STATE_DECODING,
    XM_CODEC_STATE_ERROR,
    XM_CODEC_STATE_CLOSED,
    XM_CODEC_STATE_PAUSING,
    XM_CODEC_STATE_PIC_SIZE_TOO_BIG
}XMVCodecState;

typedef enum XMVCodecDecCtl
{
	XM_CODEC_DEC_IDLE,
    XM_CODEC_DEC_CONTINUE,
    XM_CODEC_DEC_SUSPEND,
    XM_CODEC_DEC_STOP    
}XMVCodecDecCtl;

typedef enum XMVOutDataFormat
{
	XMVVOutData_ARGB8888,
	XMVVOutData_ABGR8888,
	XMVVOutData_RGB888,
	XMVVOutData_RGB565,
	XMVVOutData_YUV444,
	XMVVOutData_YUV422,
	XMVVOutData_YUV420,
	XMVVOutData_YUV411
}XMVOutDataFormat;

#define XMV_MAX_BUFFER_NUM    20

typedef struct XMVCodecBuffer
{
    uint32_t nb_blk;
    uint32_t blk_size;
    uint32_t index;
    uint8_t *blk[XMV_MAX_BUFFER_NUM];
} XMVCodecBuffer;

typedef struct XMVCodecParam
{
    uint8_t *buf;
    
    uint32_t buf_size;

    uint32_t width;

    uint32_t height;

    uint8_t *loader_buff;

    uint32_t dsp_boot_flag;

	uint8_t *tab_buff;

	uint32_t Out_YUVType;

	uint32_t Display_Width;

	uint32_t DisPlay_Height;

	uint32_t Scale_En;
	
    XMVCodecBuffer *blk_buf;
    uint32_t Alpha;    
    uint32_t reserved[3];
} XMVCodecParam;

typedef struct XMVBlockTrans 
{
    /* start address of bitstream buffer. */
    uint8_t *buf;

    /* number filled, only modified by host. */
    uint32_t nb_filled;

    /* number consumed, only modified by dsp. */
    uint32_t nb_consumed;

    /* total block number of bitstream buffer. */
    uint32_t nb_blocks;

    /* indicate end of stream. */
    uint32_t block_eof;
} XMVBlockTrans;

typedef struct XMVCodecMsg 
{
    /* current codec state */
    XMVCodecState codec_state;

    /* hurry up level msg */
    uint32_t hurry_up;
    
    /* flush all input & output buffers in dsp */
    uint32_t flush;

	uint32_t SkipTime_rang;
	uint32_t NextKeyF_time;
	uint32_t CurAudio_time;
	uint32_t CurDec_time;
    uint32_t reserved[4];
} XMVCodecMsg;

typedef struct
{
    //-------------ARM2DSP--------------
    uint32_t      imgWidth;               
    uint32_t      imgHeight;              
    uint32_t      qp;                     
    uint32_t      max_keyfrm_interval;   
	uint32_t      yBufAddr;               
    uint32_t      uvBufAddr;              
    uint32_t      mp4BufAddr;             
    uint32_t      mp4BufSize;             
    uint32_t      RefBufAddr;             
    uint32_t      RefBufSize;
    
    //-------------DSP2ARM--------------
    uint32_t      frmLength;               
    uint32_t      frmType;                

    uint32_t      signal;                
    uint32_t      exitShakeHands;
}DVCodeParam;

typedef struct XMVCodecShare 
{
    /* codec initial parameters */
    XMVCodecParam param;

    /* bitstream transfer. */
    XMVBlockTrans bits;

    /* control messages */
    XMVCodecMsg msg;
    
	/* block buffers */
    XMVCodecBuffer blk_buf;

    DVCodeParam dvparam; 

    uint32_t reserved[8];
} XMVCodecShare;

typedef struct 
{
	uint32_t width;
	uint32_t height;
	uint32_t buf_size;
	uint32_t data_format;
}JPEG_INFO_TO_KERNEL;

extern void rockchip_add_device_dsp(void);
#endif /* __PLAT_ROCKCHIP_DSP_DSP_H *//* insmod /system/lib/modules/rk28dsp.ko */




