#ifndef _CLOCK_H
#define _CLOCK_H

#include <asm/arch/scu.h>
#include <linux/list.h>


struct clk
{
	struct list_head    	list;
	struct module      	*owner;
	struct clk           	*parent;
	const char           	*name;
	int		      		id;
	int		      		usage;
	unsigned long        rate;
	eCLK_GATE 		scu_clk_gate;
	int	    			(*add_usage)(struct clk *c);
	int	    			(*sub_usage)(struct clk *c);
	
	int		     		(*enable)(struct clk *, int enable);
	int		    		(*set_rate)(struct clk *c, unsigned long rate);
	unsigned long	    	(*get_rate)(struct clk *c);
	unsigned long	    	(*round_rate)(struct clk *c, unsigned long rate);
	int		    		(*set_parent)(struct clk *c, struct clk *parent);
	unsigned int 		clk_div_from_parent;
};

/* other clocks which may be registered by board support */
#if 0
struct clk rk28_arm_clk;
struct clk rk28_dsp_clk;
struct clk rk28_codec_clk;
struct clk rk28_ahb_clk;
struct clk rk28_apb_clk;
#endif
#if 0
extern struct clk clk_usb_bus;

/* core clock support */

extern struct clk clk_f;
extern struct clk clk_h;
extern struct clk clk_p;
extern struct clk clk_mpll;
extern struct clk clk_upll;
extern struct clk clk_xtal;
#endif
/* exports for arch/arm/mach-s3c2410
 *
 * Please DO NOT use these outside of arch/arm/mach-s3c2410
*/

extern struct mutex clocks_mutex;
extern void rk28_init_clocks(unsigned int main_clock);
extern unsigned int get_clk_rate(const char *name);
extern int rk28_clkcon_enable(struct clk *clk, int enable);

extern int rk28_register_clock(struct clk *clk);
extern int rk28_register_clocks(struct clk **clk, int nr_clks);

extern int rk28_setup_clocks(unsigned long xtal,
				unsigned long fclk,
				unsigned long hclk);

#endif
