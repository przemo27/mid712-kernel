/*
 *  linux/drivers/video/sidsafb.h
 *
 *  Driver for SIDSA LCD Controller IP
 *
 *  Copyright (C) 2004 Atmel Norway AS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __SIDSAFB_H__
#define __SIDSAFB_H__

struct sidsafb_info {
	spinlock_t		lock;
	struct fb_info *	info;
	char *			mmio;
	unsigned long		irq_base;
	wait_queue_head_t	vsync_wait;
	unsigned int		guard_time;
	struct platform_device	*pdev;
	u32			pseudo_palette[16];
};


static inline void lcdc_writel(u32 value, u32 reg)
{
#ifdef IO_DEBUG
	pr_debug("lcdc writel %p <- %08lx\n",
		 reg,
		 (unsigned long)value);
#endif
	writel(value, reg);
}

static inline u32 lcdc_readl(u32 reg)
{
	u32 value = readl(reg);
#ifdef IO_DEBUG
	pr_debug("lcdc readl %p -> %08lx\n",
		 reg,
		 (unsigned long)value);
#endif
	return value;
}

#if defined(IO_DEBUG)
#define lcdc_debug_readl(reg) lcdc_readl(reg)
#else
#define lcdc_debug_readl(reg)
#endif


/* More or less configurable parameters */
#define SIDSAFB_FIFO_SIZE		512

#ifdef CONFIG_STN_MONO_AT91
#define SIDSAFB_DMA_BURST_LEN		8
#elif CONFIG_TFT_AT91
#define SIDSAFB_DMA_BURST_LEN		16
#endif

#define SIDSAFB_CRST_VAL                0xc8   // 0xda

#endif /* __SIDSAFB_H__ */
