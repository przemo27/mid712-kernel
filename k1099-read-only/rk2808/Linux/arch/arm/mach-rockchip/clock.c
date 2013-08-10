
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/hardware.h>

#include <asm/arch/hw_define.h>
#include <asm/arch/typedef.h>
#include <asm/arch/hardware.h>
#include <asm/arch/api_pll.h>
#include <asm/arch/api_scu.h>
#include <asm/arch/scu.h>
#include <asm/arch/drivers_delay.h>
#include <asm/arch/api_pmu.h>
#include <asm/arch/iomux.h>
#include <asm/arch/scu.h>
#include <asm/arch/clock.h>
#include <asm/arch/rk28_macro.h>
#include <asm/arch/rk28_debug.h>
extern struct mutex clocks_mutex;
struct list_head clocks;

int rk28_clkusage_add(struct clk *clk);
int rk28_clkusage_sub(struct clk *clk);

static struct clk rk28_arm_clk = {
		.id = 0,
		.name = "arm_clk",
		.parent = NULL,
		.rate = 0,
		.usage = 0,
		.add_usage = rk28_clkusage_add,
		.sub_usage = rk28_clkusage_sub,
	};

static struct clk rk28_dsp_clk = {
		.id = 1,
		.name = "dsp_clk",
		.parent = NULL,
		.rate = 0,
		.usage = 0,
		.add_usage = rk28_clkusage_add,
		.sub_usage = rk28_clkusage_sub,
};

static struct clk rk28_codec_clk = {
		.id = 2,
		.name = "codec_clk",
		.parent = NULL,
		.rate = 0,
		.usage = 0,
		.add_usage = rk28_clkusage_add,
		.sub_usage = rk28_clkusage_sub,
};

static struct clk rk28_ahb_clk = {
		.id = 3,
		.name = "ahb_clk",
		.parent = &rk28_arm_clk,
		.rate = 0,
		.usage = 0,
		.add_usage = rk28_clkusage_add,
		.sub_usage = rk28_clkusage_sub,
};

static struct clk rk28_apb_clk = {
		.id = 4,
		.name = "apb_clk",
		.parent = &rk28_ahb_clk,
		.rate = 0,
		.usage = 0,
		.add_usage = rk28_clkusage_add,
		.sub_usage = rk28_clkusage_sub,
};
#if 1
static struct clk rk28_i2c_clk = {
		.id = 5,
		.name = "i2c_clk",
		.parent = &rk28_apb_clk,
		.rate = 0,
		.usage = 0,
		.add_usage = rk28_clkusage_add,
		.sub_usage = rk28_clkusage_sub,
};
#endif

int rk28_clkusage_add(struct clk *clk)
{
	clk->usage++;
	return 0;
}

int rk28_clkusage_sub(struct clk *clk)
{
	clk->usage--;
	/*							//AUTO remove
	if(clk->usage == 0 && clk->remove == auto)
		rk28_clk_remove(clk);
	*/
	return 0;
}

static int clk_null_enable(struct clk *clk, int enable)
{
	return 0;
}

int rk28_clkcon_enable(struct clk *clk, int enable)		
{
	return 0;
}
int clk_enable(struct clk *clk)
{
	struct clk *clk_ops;
	list_for_each_entry(clk_ops, &clocks, list){
		if(unlikely((strcmp(clk_ops->name,clk->name) == 0))){
			clk->enable(clk,CLK_ENABLE);
		}
	}
	return -ENODEV;
}
unsigned int get_clk_rate(const char *name)
{
	struct clk *clk_ops;
	list_for_each_entry(clk_ops, &clocks, list){
		if(unlikely((strcmp(clk_ops->name,name) == 0)))
			return clk_ops->rate;
	}
	return 0;
}
struct clk *clk_get(struct device *dev, const char *name)
{
	struct clk *clk_ops;
	list_for_each_entry(clk_ops, &clocks, list){
		if(unlikely((strcmp(clk_ops->name,name) == 0)))
			return clk_ops;
	}
	return NULL;
}
int updata_clk(struct clk *c)
{
	struct clk *clk_ops;
	#if 1
	list_for_each_entry(clk_ops, &clocks, list){

		if(unlikely((clk_ops->parent != NULL)&&(strcmp(clk_ops->parent->name,c->name) == 0))){
			if(clk_ops->clk_div_from_parent != 0){
				clk_ops->rate = (clk_ops->parent->rate / clk_ops->clk_div_from_parent);
#if  ROCK_DEBUG
				printk("\n  %s div is %d\n",clk_ops->name,clk_ops->clk_div_from_parent);
#endif
			}else{
#if  ROCK_DEBUG
				printk("\n  %s div is %d\n",clk_ops->name);
#endif
			}
#if ROCK_DEBUG
			printk("\n updata %s  rate = %d\n",clk_ops->name,clk_ops->rate);

#endif
			updata_clk(clk_ops);
			return 0;
		}
	}
	return 0;
	#endif
}

int rk28_set_pllrate(struct clk *c, unsigned long rate)
{
	int ret,hdiv,pdiv;
	struct clk* clk_ops;
	list_for_each_entry(clk_ops, &clocks, list){
		if(unlikely((clk_ops->id == c->id)&&(strcmp(clk_ops->name,c->name) == 0))){
			c = clk_ops;
			c->rate = rate;
		}
	}
	
	ret = rk28_scu_set_pll(c);
	
	if(ret){
		return -1;	
	}else{
		rk28_scu_get_busdiv(&hdiv,&pdiv);
		rk28_ahb_clk.clk_div_from_parent = hdiv;
		rk28_apb_clk.clk_div_from_parent = pdiv;
		updata_clk(c);
	}
#if ROCK_DEBUG
	printk("\n clock set ret ok %d\n",c->rate);
#endif
	return 0;
}

unsigned long	rk28_get_rate(struct clk *c)
{
	return 0;
}

int rk28_register_clock(struct clk *clk)
{
#if ROCK_DEBUG
	printk("rk28_register_clock \n");
#endif
	clk->owner = THIS_MODULE;

	if (clk->enable == NULL)
		clk->enable = clk_null_enable;

	/* add to the list of available clocks */
	//mutex_lock(&clocks_mutex);
	list_add(&clk->list, &clocks);
	//mutex_unlock(&clocks_mutex);

	return 0;
}

int rk28_register_clocks(struct clk **clk, int nr_clks)
{
	return 0;
}
#if 0
int rk28_clk_remove(struct clk *clk)
{
	struct clk	*clk_ops;
	int find_pflag = 0;
	list_for_each_entry(clk_ops, &clocks, list) {
		if(unlikely((clk_ops->id == clk->id)&&(strcmp(clk_ops->name,clk->name) == 0)))
			return 0;
	}
	if(clk->parent != NULL){
		list_for_each_entry(clk_ops, &clocks, list) {
			if(unlikely((clk_ops->id == clk->parent->id)&&(strcmp(clk_ops->name,clk->parent->name) == 0))){
				find_pflag = 1;
				printk("find a parent \n");
				break;
			}else{
				printk("don 't find a parent \n");
			}
		}
		if(find_pflag == 1)
			clk->parent->add_usage(clk->parent);
		else
			rk28_clk_add(clk->parent);
		
		rk28_register_clock(clk);
	}else{
		rk28_register_clock(clk);
	}
	return 0;
}
#endif
int rk28_clk_add(struct clk *clk)
{
	struct clk	*clk_ops;
	int find_pflag = 0;
	list_for_each_entry(clk_ops, &clocks, list) {
		if(unlikely((clk_ops->id == clk->id)&&(strcmp(clk_ops->name,clk->name) == 0)))
			return 0;
	}
	if(clk->parent != NULL){
		list_for_each_entry(clk_ops, &clocks, list) {
			if(unlikely((clk_ops->id == clk->parent->id)&&(strcmp(clk_ops->name,clk->parent->name) == 0))){
				find_pflag = 1;
#if ROCK_DEBUG
				printk("find a parent \n");
#endif
				break;
			}else{
#if ROCK_DEBUG
				printk("don 't find a parent \n");
#endif
			}
		}
		if(find_pflag == 1)
			clk->parent->add_usage(clk->parent);
		else
			rk28_clk_add(clk->parent);
		
		rk28_register_clock(clk);
	}else{
		rk28_register_clock(clk);
	}
	return 0;
}
int rk28_setup_clocks(unsigned long arm_clk,
				unsigned long dsp_clk,
				unsigned long codec_clk)
{	
	int hdiv = 1 ,pdiv = 2;
	struct clk	*clk_ops;
#if ROCK_DEBUG
	printk("rk28_setup_clocks \n");
#endif	
	INIT_LIST_HEAD(&clocks);
	/* initialise the main system clocks */
#if ROCK_DEBUG
	printk("rk28_register_clock start \n");
#endif
	rk28_arm_clk.rate = arm_clk;
	rk28_dsp_clk.rate = dsp_clk;
	rk28_codec_clk.rate = codec_clk;
	
	rk28_register_clock(&rk28_arm_clk);
	rk28_register_clock(&rk28_dsp_clk);
	rk28_register_clock(&rk28_codec_clk);
#if ROCK_DEBUG	
	printk("rk28_arm_clk = %d \n",arm_clk);
	printk("rk28_dsp_clk = %d \n",dsp_clk);
	printk("rk28_codec_clk = %d \n",codec_clk);
#endif
	rk28_scu_get_busdiv(&hdiv,&pdiv);
#if ROCK_DEBUG
	printk("hdiv = %d \n",hdiv);
	printk("pdiv = %d \n",pdiv);
#endif	
	rk28_ahb_clk.rate = (rk28_ahb_clk.parent)->rate / hdiv;
	rk28_ahb_clk.clk_div_from_parent = hdiv;
	rk28_apb_clk.rate = (rk28_apb_clk.parent)->rate / pdiv;
	rk28_apb_clk.clk_div_from_parent = pdiv;

	rk28_clk_add(&rk28_ahb_clk);
	rk28_clk_add(&rk28_apb_clk);
	
	rk28_clk_add(&rk28_i2c_clk);
	list_for_each_entry(clk_ops, &clocks, list) {
#if ROCK_DEBUG	
			printk(clk_ops->name);
			printk("\n");
#endif
	}
	list_for_each_entry(clk_ops, &clocks, list) {
#if ROCK_DEBUG
			printk("%s usage = %d ",clk_ops->name,clk_ops->usage);
			printk("\n");
#endif
	}

	rk28_set_pllrate(&rk28_arm_clk,rk28_arm_clk.rate);
	

	return 0;
}

void __init rk28_init_clocks(unsigned int main_clock)
{
	unsigned long arm_fclk,dsp_fclk,codec_fclk;
	
	
	if (main_clock == 0)
		main_clock = 450*1000*1000;

	arm_fclk = rk28_get_pll(PLL_ARM);
	dsp_fclk = rk28_get_pll(PLL_DSP);
	codec_fclk = rk28_get_pll(PLL_CODEC);
	
	if(arm_fclk != main_clock)
		arm_fclk = main_clock;

	rk28_setup_clocks(arm_fclk, dsp_fclk, codec_fclk);

	return ;
}

