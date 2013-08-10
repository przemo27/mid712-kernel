/******************************************************************************
 * xmedia_iv_common.h
 * 
 * Copyright (c) 2009 Fuzhou Rockchip Co.,Ltd.
 * 
 * DESCRIPTION: - 
 *      The xmedia_iv_common header file contains the common definitions used by 
 *      image/video decoders.
 *
 * modification history
 * --------------------
 * Keith Lin, Feb 17, 2009,  Initial version
 * --------------------
 ******************************************************************************/
 
#ifndef XMEDIA_IV_COMMON_H
#define XMEDIA_IV_COMMON_H

#define XMEDIA_TICKS_PER_SECOND             1000000

#define XMEDIA_BITSTREAM_START_CODE         (0x42564b52) /* RKVB, rockchip video bitstream */

/** 
 * Specifies the picture type. 
 */
typedef enum {
    XM_PICTURE_TYPE_I   = 0x01,
    XM_PICTURE_TYPE_P   = 0x02,
    XM_PICTURE_TYPE_B   = 0x04,
    XM_PICTURE_TYPE_SI  = 0x08,
    XM_PICTURE_TYPE_SP  = 0x10,
    XM_PICTURE_TYPE_EI  = 0x11,
    XM_PICTURE_TYPE_EP  = 0x12,
    XM_PICTURE_TYPE_S   = 0x14,
    XM_PICTURE_TYPE_MAX = 0x7FFFFFFF
} VideoPictureType;

/** 
 * Enumeration defining possible uncompressed image/video formats, 
 * from OpenMAX IL 1.1. 
 *
 * ENUMS:
 *  Unused                 : Placeholder value when format is N/A
 *  Monochrome             : black and white
 *  8bitRGB332             : Red 7:5, Green 4:2, Blue 1:0
 *  12bitRGB444            : Red 11:8, Green 7:4, Blue 3:0
 *  16bitARGB4444          : Alpha 15:12, Red 11:8, Green 7:4, Blue 3:0
 *  16bitARGB1555          : Alpha 15, Red 14:10, Green 9:5, Blue 4:0
 *  16bitRGB565            : Red 15:11, Green 10:5, Blue 4:0
 *  16bitBGR565            : Blue 15:11, Green 10:5, Red 4:0
 *  18bitRGB666            : Red 17:12, Green 11:6, Blue 5:0
 *  18bitARGB1665          : Alpha 17, Red 16:11, Green 10:5, Blue 4:0
 *  19bitARGB1666          : Alpha 18, Red 17:12, Green 11:6, Blue 5:0
 *  24bitRGB888            : Red 24:16, Green 15:8, Blue 7:0
 *  24bitBGR888            : Blue 24:16, Green 15:8, Red 7:0
 *  24bitARGB1887          : Alpha 23, Red 22:15, Green 14:7, Blue 6:0
 *  25bitARGB1888          : Alpha 24, Red 23:16, Green 15:8, Blue 7:0
 *  32bitBGRA8888          : Blue 31:24, Green 23:16, Red 15:8, Alpha 7:0
 *  32bitARGB8888          : Alpha 31:24, Red 23:16, Green 15:8, Blue 7:0
 *  YUV411Planar           : U,Y are subsampled by a factor of 4 horizontally
 *  YUV411PackedPlanar     : packed per payload in planar slices
 *  YUV420Planar           : Three arrays Y,U,V.
 *  YUV420PackedPlanar     : packed per payload in planar slices
 *  YUV420SemiPlanar       : Two arrays, one is all Y, the other is U and V
 *  YUV422Planar           : Three arrays Y,U,V.
 *  YUV422PackedPlanar     : packed per payload in planar slices
 *  YUV422SemiPlanar       : Two arrays, one is all Y, the other is U and V
 *  YCbYCr                 : Organized as 16bit YUYV (i.e. YCbYCr)
 *  YCrYCb                 : Organized as 16bit YVYU (i.e. YCrYCb)
 *  CbYCrY                 : Organized as 16bit UYVY (i.e. CbYCrY)
 *  CrYCbY                 : Organized as 16bit VYUY (i.e. CrYCbY)
 *  YUV444Interleaved      : Each pixel contains equal parts YUV
 *  RawBayer8bit           : SMIA camera output format
 *  RawBayer10bit          : SMIA camera output format
 *  RawBayer8bitcompressed : SMIA camera output format
 */
typedef enum ColorFormatType {
    COLOR_FormatUnused,
    COLOR_FormatMonochrome,
    COLOR_Format8bitRGB332,
    COLOR_Format12bitRGB444,
    COLOR_Format16bitARGB4444,
    COLOR_Format16bitARGB1555,
    COLOR_Format16bitRGB565,
    COLOR_Format16bitBGR565,
    COLOR_Format18bitRGB666,
    COLOR_Format18bitARGB1665,
    COLOR_Format19bitARGB1666, 
    COLOR_Format24bitRGB888,
    COLOR_Format24bitBGR888,
    COLOR_Format24bitARGB1887,
    COLOR_Format25bitARGB1888,
    COLOR_Format32bitBGRA8888,
    COLOR_Format32bitARGB8888,
    COLOR_FormatYUV411Planar,
    COLOR_FormatYUV411PackedPlanar,
    COLOR_FormatYUV420Planar,
    COLOR_FormatYUV420PackedPlanar,
    COLOR_FormatYUV420SemiPlanar,
    COLOR_FormatYUV422Planar,
    COLOR_FormatYUV422PackedPlanar,
    COLOR_FormatYUV422SemiPlanar,
    COLOR_FormatYCbYCr,
    COLOR_FormatYCrYCb,
    COLOR_FormatCbYCrY,
    COLOR_FormatCrYCbY,
    COLOR_FormatYUV444Interleaved,
    COLOR_FormatRawBayer8bit,
    COLOR_FormatRawBayer10bit,
    COLOR_FormatRawBayer8bitcompressed,
    COLOR_FormatL2, 
    COLOR_FormatL4, 
    COLOR_FormatL8, 
    COLOR_FormatL16, 
    COLOR_FormatL24, 
    COLOR_FormatL32,
    COLOR_FormatYUV420PackedSemiPlanar,
    COLOR_FormatYUV422PackedSemiPlanar,
    COLOR_Format18BitBGR666,
    COLOR_Format24BitARGB6666,
    COLOR_Format24BitABGR6666,
    COLOR_FormatMax = 0x7FFFFFFF
} ColorFormatType;

/**
 * Specifies the unit type.
 */
typedef enum
{
    XM_UINT_TYPE_BITSTREAM  = 0x00,
    XM_UINT_TYPE_EXTRA      = 0x01,
    XM_UINT_TYPE_MAX        = 0x7F
} VideoUnitType;

/** 
 * Structure representing some time or duration in microseconds. The quantity is 64 
 * bit to accommodate a large dynamic range. 
 */
typedef struct TimeStamp
{
    uint32_t low_part;    /** low bits of the signed 64 bit tick value */
    uint32_t high_part;   /** high bits of the signed 64 bit tick value */
} TimeStamp;

/**
 * This is a structure for configuring bitstream unit information.
 */
typedef struct BitsHeader
{
    /* bitstream unit start code */
    uint32_t start_code;

    /* size of this unit */
    uint32_t size;

    /* presentation timestamp (time when frame should be shown to user). */
    TimeStamp time;

    /* unit type */
    uint32_t type;

    /* picture number in bitstream order */
    uint32_t pic_num;

    /* reserved */
    uint32_t reserved[2];
} BitsHeader;

/** 
 * the information of video/image frame for RK28.
 */
typedef struct XMFrame {
    /**
     * Pointer to the picture.
     */
    uint8_t *data[4];
    
    /**
     * width of the frame.
     */
	uint16_t width;
	
    /**
     * height of the frame.
     */
	uint16_t height;
	
    /**
     * picture type of the frame.
     */
    VideoPictureType pict_type;
    
    /**
     * color type of the picture.
     */
    ColorFormatType color_type;

    /**
     * presentation timestamp (time when frame should be shown to user).
     */
    TimeStamp pts;

    /**
     * picture number in bitstream order.
     */
    uint32_t coded_picture_number;
    
    /**
     * picture number in display order
     */
    uint32_t display_picture_number;

    /**
     * is this picture used as reference
     */
    //uint16_t reference;
    
    /**
     * is this picture displayed
     * This value is set by host cpu, and cleared by decoder.
     */
    uint32_t *displayed_ptr;

    /**
     * error information.
     */
    uint32_t error_info;

    /**
     * The content of the picture is interlaced.
     */
    uint16_t interlaced_frame;
    
    /**
     * If the content is interlaced, is top field displayed first.
     */
    uint16_t top_field_first;

} XMFrame;

#endif
