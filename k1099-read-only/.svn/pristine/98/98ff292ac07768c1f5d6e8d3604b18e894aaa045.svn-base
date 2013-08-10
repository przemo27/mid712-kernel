/*
 * V4L2 Driver for RK28 camera host
 *
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>


#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-contig.h>
#include <media/soc_camera.h>

#include <linux/videodev2.h>

#include <asm/dma.h>
#include <asm/arch/hardware.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>


#define RK28_VIP_AHBR_CTRL                0x00
#define RK28_VIP_INT_MASK                 0x04
#define RK28_VIP_INT_STS                  0x08
#define RK28_VIP_STS                      0x0c
#define RK28_VIP_CTRL                     0x10
#define RK28_VIP_CAPTURE_F1SA_Y           0x14
#define RK28_VIP_CAPTURE_F1SA_UV          0x18
#define RK28_VIP_CAPTURE_F1SA_Cr          0x1c
#define RK28_VIP_CAPTURE_F2SA_Y           0x20
#define RK28_VIP_CAPTURE_F2SA_UV          0x24
#define RK28_VIP_CAPTURE_F2SA_Cr          0x28
#define RK28_VIP_FB_SR                    0x2c
#define RK28_VIP_FS                       0x30
#define RK28_VIP_VIPRESERVED              0x34
#define RK28_VIP_CROP                     0x38
#define RK28_VIP_CRM                      0x3c
#define RK28_VIP_RESET                    0x40
#define RK28_VIP_L_SFT                    0x44

#define RK28_CPU_API_REG                  (REG_FILE_BASE_ADDR_VA+0x14)


//ctrl-------------------
#define  DISABLE_CAPTURE              0x00000000
#define  ENABLE_CAPTURE               0x00000001

#define  VSY_HIGH_ACTIVE              0x00000080
#define  VSY_LOW_ACTIVE               0x00000000

#define  HSY_HIGH_ACTIVE              0x00000000
#define  HSY_LOW_ACTIVE               0x00000002

#define  CCIR656                      0x00000000
#define  SENSOR                       0x00000004

#define  SENSOR_UYVY                  0x00000000
#define  SENSOR_YUYV                  0x00000008

#define  CON_OR_PIN                   0x00000000
#define  ONEFRAME                     0x00000020

#define  VIPREGYUV420                 0x00000000
#define  VIPREGYUV422                 0x00000040

#define  FIELD0_START                 0x00000000
#define  FIELD1_START                 0x00000080

#define  CONTINUOUS                   0x00000000
#define  PING_PONG                    0x00000100

#define  POSITIVE_EDGE                0x00000000
#define  NEGATIVE_EDGE                0x00000200

#define  VIPREGNTSC                   0x00000000
#define  VIPREGPAL                    0x00000400
//--------------------------


#define RK28_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5)
#define RK28_CAM_DRV_NAME "rk28-camera"


static DEFINE_MUTEX(camera_lock);



#define write_vip_reg(addr, val)        __raw_writel(val, addr+VIP_BASE_ADDR_VA) 
#define read_vip_reg(addr)              __raw_readl(addr+VIP_BASE_ADDR_VA)    
#define mask_vip_reg(addr, msk, val)    write_vip_reg(addr, (val)|((~(msk))&read_vip_reg(addr)))

#define set_vip_vsp(val)    __raw_writel(((val) | __raw_readl(RK28_CPU_API_REG)), RK28_CPU_API_REG)


#define RK28_SENSOR_24MHZ      0
#define RK28_SENSOR_48MHZ      1

//#define RK28_CAMERA_PCP		1
//#define RK28_CAMERA_HSP		2
//#define RK28_CAMERA_VSP		4
//#define RK28_CAMERA_SEN		8



extern void videobuf_dma_contig_free(struct videobuf_queue *q, struct videobuf_buffer *buf);
extern dma_addr_t videobuf_to_dma_contig(struct videobuf_buffer *buf);
extern void videobuf_queue_dma_contig_init(struct videobuf_queue *q,
				    struct videobuf_queue_ops *ops,
				    struct device *dev,
				    spinlock_t *irqlock,
				    enum v4l2_buf_type type,
				    enum v4l2_field field,
				    unsigned int msize,
				    void *priv);
/*
 * Structures
 */
struct rk28_camera_platform_data {
	int (*init)(struct device *);
	int (*power)(struct device *, int);
	int (*reset)(struct device *, int);

	unsigned long flags;
	unsigned long mclk_10khz;
};

/* buffer for one video frame */
struct rk28_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;
	const struct soc_camera_data_format        *fmt;
	int			inwork;
};

struct rk28_camera_dev {
	struct device		*dev;
	/* RK2827x is only supposed to handle one camera on its Quick Capture
	 * interface. If anyone ever builds hardware to enable more than
	 * one camera, they will have to modify this driver too */
	struct soc_camera_device *icd;
	unsigned int		clk;

	unsigned int		irq;
	void __iomem		*base;

	struct rk28_camera_platform_data *pdata;
	struct resource		*res;

	struct list_head	capture;

	spinlock_t		lock;

	struct videobuf_buffer	*active;

};

static const char *rk28_cam_driver_description = "RK28_Camera";

static unsigned int vid_limit = 16;	/* Video memory limit, in Mb */

/*
 *  Videobuf operations
 */
static int rk28_videobuf_setup(struct videobuf_queue *vq, unsigned int *count,
			      unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	int bytes_per_pixel = (icd->current_fmt->depth + 7) >> 3;

	dev_dbg(&icd->dev, "count=%d, size=%d\n", *count, *size);

	/* planar capture requires Y, U and V buffers to be page aligned */
    *size = PAGE_ALIGN(2 * icd->width * icd->height * bytes_per_pixel); /* Y pages UV pages, yuv422*/

	if (0 == *count)
		*count = 32;
	while (*size * *count > vid_limit * 1920 * 1920)
		(*count)--;
		
    //printk("\n%s..%s..%d    ******** nzy *********%d %d %d\n",__FUNCTION__,__FILE__,__LINE__, icd->width, icd->height, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct rk28_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		&buf->vb, buf->vb.baddr, buf->vb.bsize);

	if (in_interrupt())
		BUG();

	videobuf_dma_contig_free(vq, &buf->vb);
	dev_dbg(&icd->dev, "%s freed\n", __func__);
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
		
}

static int rk28_videobuf_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb, enum v4l2_field field)
{
    struct soc_camera_device *icd = vq->priv_data;
    struct rk28_buffer *buf;
    int ret;
    
    buf = container_of(vb, struct rk28_buffer, vb);
    
    dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
        vb, vb->baddr, vb->bsize);
    
    /* Added list head initialization on alloc */
    WARN_ON(!list_empty(&vb->queue));
    
    /* This can be useful if you want to see if we actually fill
     * the buffer with something */
    //memset((void *)vb->baddr, 0xaa, vb->bsize);
    
    BUG_ON(NULL == icd->current_fmt);
    
    if (buf->fmt    != icd->current_fmt ||
        vb->width   != icd->width ||
        vb->height  != icd->height ||
        vb->field   != field) {
        buf->fmt    = icd->current_fmt;
        vb->width   = icd->width;
        vb->height  = icd->height;
        vb->field   = field;
        vb->state   = VIDEOBUF_NEEDS_INIT;
    }
    
    vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3) * 2;
    if (0 != vb->baddr && vb->bsize < vb->size) {
        ret = -EINVAL;
        goto out;
    }
    //printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__, vb->bsize, vb->size);
    
    if (vb->state == VIDEOBUF_NEEDS_INIT) {
        ret = videobuf_iolock(vq, vb, NULL);
        if (ret)
            goto fail;
        vb->state = VIDEOBUF_PREPARED;
    }
    
    return 0;
fail:
    free_buffer(vq, buf);
out:
    return ret;
}

static inline void rk28_videobuf_capture(struct videobuf_buffer *vb)
{
      unsigned int size;
      char *y, *uv;
    if (vb) {        
        size = vb->width * vb->height; /* Y pages UV pages, yuv422*/
        //y = (char *)videobuf_to_cpu_contig(vb);
        //uv = (char *)videobuf_to_cpu_contig(vb)+ size;
        //memset(y, 0, size);
        //memset(uv, 0, size/2);
        write_vip_reg(RK28_VIP_CAPTURE_F1SA_Y, videobuf_to_dma_contig(vb));
        write_vip_reg(RK28_VIP_CAPTURE_F1SA_UV, videobuf_to_dma_contig(vb) + size);
        write_vip_reg(RK28_VIP_CAPTURE_F2SA_Y, videobuf_to_dma_contig(vb));
        write_vip_reg(RK28_VIP_CAPTURE_F2SA_UV, videobuf_to_dma_contig(vb) + size);
        //printk("\n%s..%s..%d    ******** nzy *********0x%08x 0x%08x\n",__FUNCTION__,__FILE__,__LINE__,videobuf_to_dma_contig(vb), size);
        write_vip_reg(RK28_VIP_FB_SR,  0x00000002);//frame1 has been ready to receive data,frame 2 is not used        
        //mask_vip_reg(RK28_VIP_CTRL, ENABLE_CAPTURE, ENABLE_CAPTURE);
    }
}

static void rk28_videobuf_queue(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	unsigned long flags;
	unsigned int size;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		vb, vb->baddr, vb->bsize);

	vb->state = VIDEOBUF_ACTIVE;
	spin_lock_irqsave(&pcdev->lock, flags);
	list_add_tail(&vb->queue, &pcdev->capture);

	if (!pcdev->active) {
		pcdev->active = vb;
        rk28_videobuf_capture(vb);
        //printk("\n%s..%s..%d    ******** nzy *********%d %d\n",__FUNCTION__,__FILE__,__LINE__, vb->width, vb->height);
	}


	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static irqreturn_t rk28_camera_irq(int irq, void *data)
{
	struct rk28_camera_dev *pcdev = data;
	struct videobuf_buffer *vb;
	unsigned long flags;
	unsigned int size;

    unsigned int int_sts = read_vip_reg(RK28_VIP_INT_STS);//clear vip interrupte single

	spin_lock_irqsave(&pcdev->lock, flags);

	vb = pcdev->active;
	list_del_init(&vb->queue);

	if (!list_empty(&pcdev->capture))
		pcdev->active = list_entry(pcdev->capture.next,
					   struct videobuf_buffer, queue);
	else
		pcdev->active = NULL;

    rk28_videobuf_capture(pcdev->active);
    //printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);

	vb->state = VIDEOBUF_DONE;
	do_gettimeofday(&vb->ts);
	vb->field_count++;
	wake_up(&vb->done);
	spin_unlock_irqrestore(&pcdev->lock, flags);
    //printk("\n%s..%s..%d    ******** nzy *********%d\n",__FUNCTION__,__FILE__,__LINE__,pcdev->active);

	return IRQ_HANDLED;
}


static void rk28_videobuf_release(struct videobuf_queue *vq,
				 struct videobuf_buffer *vb)
{
	struct rk28_buffer *buf = container_of(vb, struct rk28_buffer, vb);
#ifdef DEBUG
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_dbg(&icd->dev, "%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		dev_dbg(&icd->dev, "%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		dev_dbg(&icd->dev, "%s (prepared)\n", __func__);
		break;
	default:
		dev_dbg(&icd->dev, "%s (unknown)\n", __func__);
		break;
	}
#endif

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops rk28_videobuf_ops = {
	.buf_setup      = rk28_videobuf_setup,
	.buf_prepare    = rk28_videobuf_prepare,
	.buf_queue      = rk28_videobuf_queue,
	.buf_release    = rk28_videobuf_release,
};

static void rk28_camera_init_videobuf(struct videobuf_queue *q,
			      struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;

	/* We must pass NULL as dev pointer, then all pci_* dma operations
	 * transform to normal dma_* ones. */
	videobuf_queue_dma_contig_init(q,
				       &rk28_videobuf_ops,
				       &ici->dev, &pcdev->lock,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE,
				       V4L2_FIELD_NONE,
				       sizeof(struct rk28_buffer),
				       icd);
}

static void rk28_camera_activate(struct rk28_camera_dev *pcdev)
{
    /*
	struct rk28_camera_platform_data *pdata = pcdev->pdata;

	dev_dbg(pcdev->dev, "Registered platform device at %p data %p\n",
		pcdev, pdata);

	if (pdata && pdata->init) {
		dev_dbg(pcdev->dev, "%s: Init gpios\n", __func__);
		pdata->init(pcdev->dev);
	}

	if (pdata && pdata->power) {
		dev_dbg(pcdev->dev, "%s: Power on camera\n", __func__);
		pdata->power(pcdev->dev, 1);
	}

	if (pdata && pdata->reset) {
		dev_dbg(pcdev->dev, "%s: Releasing camera reset\n",
			__func__);
		pdata->reset(pcdev->dev, 1);
	}
	*/

    rockchip_mux_api_set(GPIOF6_VIPCLK_SEL_NAME, IOMUXB_VIP_CLKOUT);
    write_vip_reg(RK28_VIP_AHBR_CTRL, 0x05);//only 8 or 4 will be the actual length.
    write_vip_reg(RK28_VIP_INT_MASK, 0x01);//capture complete interrupt enable
    write_vip_reg(RK28_VIP_CRM,  0x00000000);//Y/CB/CR color modification
    write_vip_reg(RK28_VIP_FB_SR,  0x00000003);//frame1 has been ready to receive data,frame 2 is not used
	
}

static void rk28_camera_deactivate(struct rk28_camera_dev *pcdev)
{
    /*
	struct rk28_camera_platform_data *pdata = pcdev->pdata;

	if (pdata && pdata->reset) {
		dev_dbg(pcdev->dev, "%s: Asserting camera reset\n",
			__func__);
		pdata->reset(pcdev->dev, 0);
	}

	if (pdata && pdata->power) {
		dev_dbg(pcdev->dev, "%s: Power off camera\n", __func__);
		pdata->power(pcdev->dev, 0);
	}
	*/
}

/* The following two functions absolutely depend on the fact, that
 * there can be only one camera on RK28 quick capture interface */
static int rk28_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	int ret;

	mutex_lock(&camera_lock);

	if (pcdev->icd) {
		ret = -EBUSY;
		goto ebusy;
	}

	dev_info(&icd->dev, "RK28 Camera driver attached to camera %d\n",
		 icd->devnum);

	rk28_camera_activate(pcdev);
	ret = icd->ops->init(icd);

	if (!ret)
		pcdev->icd = icd;

ebusy:
	mutex_unlock(&camera_lock);

	return ret;
}

static void rk28_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	dev_info(&icd->dev, "RK28 Camera driver detached from camera %d\n",
		 icd->devnum);

	icd->ops->release(icd);

	rk28_camera_deactivate(pcdev);

	pcdev->icd = NULL;
}

static int rk28_camera_set_bus_param(struct soc_camera_device *icd, __u32 pixfmt)
{
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
	unsigned long bus_flags, camera_flags, common_flags;
	unsigned int vip_ctrl_val = 0;
	int ret;
	
	bus_flags = SOCAM_MASTER |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_PCLK_SAMPLE_FALLING|
		SOCAM_SENSOR_UYVY |
		SOCAM_SENSOR_YUYV |		
		SOCAM_DATAWIDTH_8;

	camera_flags = icd->ops->query_bus_param(icd);

	common_flags = soc_camera_bus_param_compatible(camera_flags, bus_flags);
	if (!common_flags)
		return -EINVAL;

	/* Make choises, based on platform preferences */
#if 0	
    if (pcdev->platform_flags & RK28_CAMERA_HSP)
        common_flags &= ~SOCAM_HSYNC_ACTIVE_HIGH;
    else
        common_flags &= ~SOCAM_HSYNC_ACTIVE_LOW;


    if (pcdev->platform_flags & RK28_CAMERA_VSP)
        common_flags &= ~SOCAM_VSYNC_ACTIVE_HIGH;
    else
        common_flags &= ~SOCAM_VSYNC_ACTIVE_LOW;
    
    if (pcdev->platform_flags & RK28_CAMERA_PCP)
        common_flags &= ~SOCAM_PCLK_SAMPLE_RISING;
    else
        common_flags &= ~SOCAM_PCLK_SAMPLE_FALLING;

    if (pcdev->platform_flags & RK28_CAMERA_SEN)
        common_flags &= ~SOCAM_SENSOR_UYVY;
    else
        common_flags &= ~SOCAM_SENSOR_YUYV;
#endif

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

    icd->buswidth = 8;

    switch (pixfmt) {
    case V4L2_PIX_FMT_YUV422P:
        vip_ctrl_val |= VIPREGYUV422;
        break;
    case V4L2_PIX_FMT_YUV420:
        vip_ctrl_val |= VIPREGYUV420;
        break;
    default:
        return -EINVAL;
    }

	if (common_flags & SOCAM_PCLK_SAMPLE_FALLING)
        vip_ctrl_val |= NEGATIVE_EDGE;
	if (common_flags & SOCAM_HSYNC_ACTIVE_LOW)
        vip_ctrl_val |= HSY_LOW_ACTIVE;
	if (common_flags & SOCAM_VSYNC_ACTIVE_HIGH)
        set_vip_vsp(VSY_HIGH_ACTIVE);
	if (common_flags & SOCAM_SENSOR_YUYV)
        vip_ctrl_val |= SENSOR_YUYV;

    vip_ctrl_val |= SENSOR | ONEFRAME | ENABLE_CAPTURE;//need to be modified,nzy add
    
    write_vip_reg(RK28_VIP_CTRL, vip_ctrl_val);

	return 0;
}

static int rk28_camera_try_bus_param(struct soc_camera_device *icd, __u32 pixfmt)
{
	unsigned long bus_flags, camera_flags;

	bus_flags = SOCAM_MASTER |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_PCLK_SAMPLE_FALLING|
		SOCAM_SENSOR_UYVY |
		SOCAM_SENSOR_YUYV |		
		SOCAM_DATAWIDTH_8;

	camera_flags = icd->ops->query_bus_param(icd);

	return soc_camera_bus_param_compatible(camera_flags, bus_flags) ? 0 : -EINVAL;
}

static int rk28_camera_set_fmt_cap(struct soc_camera_device *icd,
				  __u32 pixfmt, struct v4l2_rect *rect)
{
    //mask_vip_reg(RK28_VIP_CTRL, ENABLE_CAPTURE, DISABLE_CAPTURE);

#if 1
    unsigned int sensorSize = ((rect->width + rect->left) << 16) + rect->height;
    unsigned int imageSize  = (rect->left << 16) + rect->top;
#else
    unsigned int sensorSize;
    unsigned int imageSize;
    unsigned int crop_x;
    unsigned int crop_y;

    if ((rect->width < 640) && (rect->height < 480)){
        crop_x = (640 - rect->width) >> 1;
        crop_y = (480 - rect->height) >> 1;
        sensorSize = ((rect->width + crop_x) << 16) + rect->height + crop_y;
        imageSize  = (crop_x << 16) + crop_y;
    } else {
        sensorSize = (rect->width << 16) + rect->height;
        imageSize  = (rect->left << 16) + rect->top;
    }
#endif    
    printk("\n%s..%s..%d    ******** nzy *********left = %d, top = %d, width = %d, height = %d\n",__FUNCTION__,__FILE__,__LINE__, rect->left, rect->top, rect->width, rect->height);

    write_vip_reg(RK28_VIP_CROP, imageSize);
    write_vip_reg(RK28_VIP_FS, sensorSize);
    
	return icd->ops->set_fmt_cap(icd, pixfmt, rect);
}

static int rk28_camera_try_fmt_cap(struct soc_camera_device *icd,
				  struct v4l2_format *f)
{
	/* limit to rk28 hardware capabilities */
	if (f->fmt.pix.height < 32)
		f->fmt.pix.height = 32;
	if (f->fmt.pix.height > 2048)
		f->fmt.pix.height = 2048;
	if (f->fmt.pix.width < 48)
		f->fmt.pix.width = 48;
	if (f->fmt.pix.width > 2048)
		f->fmt.pix.width = 2048;
	f->fmt.pix.width &= ~0x01;

	/* limit to sensor capabilities */
	return icd->ops->try_fmt_cap(icd, f);
}

static int rk28_camera_reqbufs(struct soc_camera_file *icf,
			      struct v4l2_requestbuffers *p)
{
	int i;

	/* This is for locking debugging only. I removed spinlocks and now I
	 * check whether .prepare is ever called on a linked buffer, or whether
	 * a dma IRQ can occur for an in-work or unlinked buffer. Until now
	 * it hadn't triggered */
	for (i = 0; i < p->count; i++) {
		struct rk28_buffer *buf = container_of(icf->vb_vidq.bufs[i],
						      struct rk28_buffer, vb);
		buf->inwork = 0;
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static unsigned int rk28_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct rk28_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next, struct rk28_buffer,
			 vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN|POLLRDNORM;

	return 0;
}

static int rk28_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the firendly caller:-> */
	strlcpy(cap->card, rk28_cam_driver_description, sizeof(cap->card));
	cap->version = RK28_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int rk28_camera_suspend(struct soc_camera_device *icd, pm_message_t state)
{
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
    int ret = 0;
    
	if ((pcdev->icd) && (pcdev->icd->ops->suspend))
		ret = pcdev->icd->ops->suspend(pcdev->icd, state);
		
	return ret;
}

static int rk28_camera_resume(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici =
		to_soc_camera_host(icd->dev.parent);
	struct rk28_camera_dev *pcdev = ici->priv;
    int ret = 0;

	if ((pcdev->icd) && (pcdev->icd->ops->resume))
		ret = pcdev->icd->ops->resume(pcdev->icd);

	return ret;
}

static struct soc_camera_host_ops rk28_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= rk28_camera_add_device,
	.remove		= rk28_camera_remove_device,
	.suspend	= rk28_camera_suspend,
	.resume		= rk28_camera_resume,
	.set_fmt_cap	= rk28_camera_set_fmt_cap,
	.try_fmt_cap	= rk28_camera_try_fmt_cap,
	.init_videobuf	= rk28_camera_init_videobuf,
	.reqbufs	= rk28_camera_reqbufs,
	.poll		= rk28_camera_poll,
	.querycap	= rk28_camera_querycap,
	.try_bus_param	= rk28_camera_try_bus_param,
	.set_bus_param	= rk28_camera_set_bus_param,
};

static const __u32 fourcc[] = {V4L2_PIX_FMT_YUV422P, V4L2_PIX_FMT_YUV420};

/* Should be allocated dynamically too, but we have only one. */
static struct soc_camera_host rk28_soc_camera_host = {
	.drv_name		= RK28_CAM_DRV_NAME,
	.ops			= &rk28_soc_camera_host_ops,
	.fourcc         = fourcc,
	.num_formats    = sizeof(fourcc)/sizeof(__u32),
};

static int rk28_camera_probe(struct platform_device *pdev)
{
	struct rk28_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0) {
		err = -ENODEV;
		goto exit;
	}
	
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

    /*config output clk*/
	pcdev->clk = RK28_SENSOR_24MHZ;

	dev_set_drvdata(&pdev->dev, pcdev);
	pcdev->res = res;

	pcdev->pdata = pdev->dev.platform_data;
	//pcdev->platform_flags = pcdev->pdata->flags;

	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);
	
	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res->start, res->end - res->start + 1,
				RK28_CAM_DRV_NAME)) {
		err = -EBUSY;
		goto exit_kfree;
	}
	
	base = ioremap(res->start, res->end - res->start + 1);
	if (!base) {
		err = -ENOMEM;
		goto exit_release;
	}
	
	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->dev = &pdev->dev;

	/* config buffer address */
	/* request irq */
	err = request_irq(pcdev->irq, rk28_camera_irq, 0, RK28_CAM_DRV_NAME,
			  pcdev);
	if (err) {
		dev_err(pcdev->dev, "Camera interrupt register failed \n");
		goto exit_iounmap;
	}

	rk28_soc_camera_host.priv	= pcdev;
	rk28_soc_camera_host.dev.parent	= &pdev->dev;
	rk28_soc_camera_host.nr		= pdev->id;
	err = soc_camera_host_register(&rk28_soc_camera_host);
	if (err)
		goto exit_free_irq;

    printk("\n%s..%s..%d    ******** nzy *********\n",__FUNCTION__,__FILE__,__LINE__);

	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_iounmap:
	iounmap(base);
exit_release:
	release_mem_region(res->start, res->end - res->start + 1);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __devexit rk28_camera_remove(struct platform_device *pdev)
{
	struct rk28_camera_dev *pcdev = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(pcdev->irq, pcdev);

	soc_camera_host_unregister(&rk28_soc_camera_host);

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(pcdev);

	dev_info(&pdev->dev, "RK28 Camera driver unloaded\n");

	return 0;
}

static struct platform_driver rk28_camera_driver = {
	.driver 	= {
		.name	= RK28_CAM_DRV_NAME,
	},
	.probe		= rk28_camera_probe,
	.remove		= rk28_camera_remove,
};


static int __devinit rk28_camera_init(void)
{
	return platform_driver_register(&rk28_camera_driver);
}

static void __exit rk28_camera_exit(void)
{
	platform_driver_unregister(&rk28_camera_driver);
}

module_init(rk28_camera_init);
module_exit(rk28_camera_exit);

MODULE_DESCRIPTION("RK28             Camera Host driver");
MODULE_AUTHOR("nzy <kernel@rock-chips>");
MODULE_LICENSE("GPL");
