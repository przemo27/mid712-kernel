/* include/asm-arm/arch-ROCK28/system.h
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

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <asm/proc-fns.h>
#include <asm/arch/hardware.h>
#include <asm/arch/rk28_debug.h>

extern volatile int rk28_pm_status ;
static inline void arch_idle(void)
{        
        if( rk28_pm_status )
                return;
	cpu_do_idle();
}

extern void (*rk28_arch_reset)( int mode );

static inline void arch_reset(char mode)
{
	/*call watch-dog reset system*/
	if(rk28_arch_reset)
		(rk28_arch_reset)( mode );
	
}

#endif
