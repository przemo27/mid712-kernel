/* arch/arm/mach-s5pv210/include/mach/cpu-freq-v210.h
 *
 *  Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *
 * S5PV210/S5PC110 CPU frequency scaling support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_CPU_FREQ_H
#define __ASM_ARCH_CPU_FREQ_H

#include <linux/cpufreq.h>

#define USE_FREQ_TABLE
#define KHZ_T		1000
#define MPU_CLK		"armclk"

enum S5PV210_CPUFREQ_MODE {
    MAXIMUM_TABLE = 0,
    NORMAL_TABLE,
    RESTRICT_TABLE,
};

/*
 * APLL M,P,S value for target frequency
 **/
#define APLL_VAL_1664	((1<<31)|(417<<16)|(3<<8)|(0))
#define APLL_VAL_1332	((1<<31)|(444<<16)|(4<<8)|(0))
#define APLL_VAL_1200	((1<<31)|(150<<16)|(3<<8)|(1))
#define APLL_VAL_1000	((1<<31)|(125<<16)|(3<<8)|(1))
#define APLL_VAL_800	((1<<31)|(100<<16)|(3<<8)|(1))

enum perf_level {
	L0 = 0,
	L1,
	L2,
	L3,
	L4,
};

struct s5pv210_domain_freq {
	unsigned long	apll_out;
	unsigned long	armclk;
	unsigned long	hclk_msys;
	unsigned long	pclk_msys;
	unsigned long	hclk_dsys;
	unsigned long	pclk_dsys;
	unsigned long	hclk_psys;
	unsigned long	pclk_psys;
};

struct s5pv210_cpufreq_freqs {
	struct cpufreq_freqs	freqs;
	struct s5pv210_domain_freq	old;
	struct s5pv210_domain_freq	new;
};

struct s5pv210_dvs_conf {
	const unsigned long	lvl;		/* DVFS level : L0,L1,L2,L3.*/
	unsigned long		arm_volt;	/* uV */
	unsigned long		int_volt;	/* uV */
};

extern int s5pv210_set_cpufreq_level(unsigned int flag);

#define SLEEP_FREQ      (800 * 1000) /* Use 800MHz when entering sleep */

/* additional symantics for "relation" in cpufreq with pm */
#define DISABLE_FURTHER_CPUFREQ         0x10
#define ENABLE_FURTHER_CPUFREQ          0x20
#define MASK_FURTHER_CPUFREQ            0x30
/* With 0x00(NOCHANGE), it depends on the previous "further" status */

#endif /* __ASM_ARCH_CPU_FREQ_H */
