/* include/asm-arm/arch-ROCK28/uncompress.h
**
** Copyright (C) 2009 ROCKCHIP, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#define ROCK28_TTY_PUT_CHAR  (*((volatile unsigned int *)0xff002000))

/*
 * This does not append a newline
 */

static inline void putc(unsigned int c)
{
	ROCK28_TTY_PUT_CHAR = (volatile unsigned int)(c);
}
static inline void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif
