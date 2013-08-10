#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/i2c.h>
#include <linux/console.h>
#include <linux/fb.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/typedef.h>
#include <asm/arch/gpio.h>
#include <asm/arch/api_intc.h>
#include <asm/arch/hw_define.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/iomux.h>
#include <asm/arch/api_i2c.h>
#include <asm/arch/rk28_scu.h>
#include <asm/arch/anx7150.h>
#include <asm/arch/rk28_scu.h>
#include <asm-arm/uaccess.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <sound/soc.h>
#include <linux/android_power.h>

#include "anx7150_sys.h"

#define DEVICE_NAME "anx7150"

#undef NULL
#define NULL ((void *)0)

#define DEBUG

#ifdef DEBUG
#define D(fmt, arg...) printk("<7>%s:%d: " fmt, __FILE__, __LINE__, ##arg)
#else
#define D(fmt, arg...)
#endif
#define E(fmt, arg...) printk("<3>!!!%s:%d: " fmt, __FILE__, __LINE__, ##arg)

/*******************ANX7150 I2C ADDR*************************/
#define ANX7150_I2C_ADDR0 0x72
#define ANX7150_I2C_ADDR1 0x7A

/*******************ANX7150_INIT_STATUS**********************/
#define UNINITIALIZED   0
#define INITIALIZED     1

/*******************ANX7150 ENABLE***************************/
#define DISABLE 0
#define ENABLE  1

typedef void (*timer_handler_t)(unsigned long);

struct anx7150_dev_s{
	struct miscdevice misc_dev;
	int open_cnt;
	int rate;               /* anx7150 scan rate(ms)*/ 
	int anx7150_init_status;/* 0-uninitialized 1-initialized */
	int anx7150_enable;
	int anx7150_resolution; /**/
	int connect_status;     /* 0-UnPlug 1-Plug      */
	int rk28_output_status;
	int timer_en_cnt;
	struct timer_list *timer;
	struct fasync_struct *async_queue;
};

struct anx7150_dev_s anx7150_dev;

static unsigned short anx7150_normal_i2c[] = {
	ANX7150_I2C_ADDR0 >> 1, 
	ANX7150_I2C_ADDR1 >> 1, 
	I2C_CLIENT_END
};

static unsigned short anx7150_i2c_ignore[] = {I2C_CLIENT_END, I2C_CLIENT_END};

static struct i2c_client_address_data anx7150_i2c_addr_data = {
	.normal_i2c = anx7150_normal_i2c,
	.probe  = anx7150_i2c_ignore,
	.ignore = anx7150_i2c_ignore,
};

/* set flag in rk28_fb.c */
extern void rk28fb_put_output_status(int status);
extern void rk28fb_put_output_resloution(int resloution);

/* win0fb ioctl in rk28_fb.c */
extern int win0fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);


static int anx7150_i2c_attach_adapter(struct i2c_adapter *adap);
static int anx7150_i2c_detach_client(struct i2c_client *client);
static void anx7150_i2c_shutdown(struct i2c_client *client);

static struct i2c_driver anx7150_i2c_driver  = {
	.driver = {
		.name  = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.id = ANX7150_I2C_ADDR0,
	.attach_adapter = &anx7150_i2c_attach_adapter,
	.detach_client 	= &anx7150_i2c_detach_client,
	.shutdown     	= &anx7150_i2c_shutdown,
};

static struct  i2c_client anx7150_i2c_client = {
	.driver = &anx7150_i2c_driver,
	.name	= "anx7150_i2c_addr0",
};

static struct  i2c_client anx7150_i2c_client1 = {
	.driver = &anx7150_i2c_driver,
	.name	= "anx7150_i2c_addr1",
};

static int anx7150_i2c_probe(struct i2c_adapter *bus, int address, int kind)
{
	int ret;
	
	anx7150_i2c_client.adapter    = bus;
	anx7150_i2c_client.addr       = ANX7150_I2C_ADDR0 >> 1; //address;
	anx7150_i2c_client.mode       = NORMALNOSTOPMODE;
	anx7150_i2c_client.Channel    = I2C_CH0;
	anx7150_i2c_client.addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	anx7150_i2c_client.speed      = 200;
	
	ret = i2c_attach_client(&anx7150_i2c_client);
	if(ret < 0){
		E("i2c_attach_client err!\n");
		return ret;
	}

	anx7150_i2c_client1.adapter    = bus;
	anx7150_i2c_client1.addr       = ANX7150_I2C_ADDR1 >> 1; //address;
	anx7150_i2c_client1.mode       = NORMALNOSTOPMODE;
	anx7150_i2c_client1.Channel    = I2C_CH0;
	anx7150_i2c_client1.addressBit = I2C_7BIT_ADDRESS_8BIT_REG;
	anx7150_i2c_client1.speed      = 200;
	
	ret = i2c_attach_client(&anx7150_i2c_client1);
	if(ret < 0){
		E("i2c_attach_client err!\n");
		return ret;
	}
		
	return 0;
}

static int anx7150_i2c_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &anx7150_i2c_addr_data, anx7150_i2c_probe);
}

static int anx7150_i2c_detach_client(struct i2c_client *client)
{
	int ret;
	
	ret = i2c_detach_client(&anx7150_i2c_client);
	if(ret < 0){
		E("i2c_detach_client err!\n");
		return ret;
	}
	
	ret = i2c_detach_client(&anx7150_i2c_client1);
	if(ret < 0){
		E("i2c_detach_client err!\n");
		return ret;
	}
	
	return 0;
}

static void anx7150_i2c_shutdown(struct i2c_client *client)
{
	D("enter anx7150_i2c_shutdown.\n");
}

int anx7150_send_msg(unsigned char addr, unsigned char *buf, unsigned short len, unsigned short rw_flag)
{
	int ret;
	struct i2c_msg msgs = {
		.addr = addr,
		.flags = rw_flag,
		.len = len,
		.buf = buf,
	};

	ret = i2c_transfer(anx7150_i2c_client.adapter, &msgs, 1);
	if(ret < 0 && ret != -EAGAIN){
		E("anx7150_send_msg err, ret=%d\n", ret);
		if(rw_flag == 1){
			E("read  addr=0x%.2x\n", buf[0]);
		}else{
			E("write addr=0x%.2x, val=0x%.2x\n", buf[0], buf[1]);
			return -1;
		}
	}
	
	return 0;
}

unsigned char ANX7150_i2c_read_p0_reg(unsigned char reg_addr, unsigned char *val)
{
	unsigned char c;
	int ret;
	c = reg_addr;

	ret = anx7150_send_msg(ANX7150_I2C_ADDR0 >> 1, &c, 1, 1);
	if(ret < 0){
		E("anx7150_send_msg err!\n");
		return 1;
	}

	*val = c;
	return 0;
}

unsigned char ANX7150_i2c_read_p1_reg(unsigned char reg_addr, unsigned char *val)
{
	unsigned char c;
	int ret;
	c = reg_addr;

	ret = anx7150_send_msg(ANX7150_I2C_ADDR1 >> 1, &c, 1, 1);
	if(ret < 0){
		E("anx7150_send_msg err!\n");
		return 1;
	}

	*val = c;
	return 0;
}

unsigned char ANX7150_i2c_write_p0_reg(unsigned char reg_addr, unsigned char val)
{
	unsigned char buf[2] = {reg_addr, val};

	return anx7150_send_msg(ANX7150_I2C_ADDR0 >> 1, buf, 2, 0);
}

unsigned char ANX7150_i2c_write_p1_reg(unsigned char reg_addr, unsigned char val)
{
	unsigned char buf[2] = {reg_addr, val};

	return anx7150_send_msg(ANX7150_I2C_ADDR1 >> 1, buf, 2, 0);
}


/************************ rk28 hdmi切换时音视频操作函数 *********************************/
/* sound/soc/codecs/rk1000_codec.c */
#if(defined(CONFIG_SND_ROCKCHIP_SOC_RK1000))
extern struct snd_soc_codec_dai rk1000_codec_dai;
extern int rk1000_codec_mute(struct snd_soc_codec_dai *dai, int mute);
extern int rk1000_codec_set_clk(int mclk, int rate);
/* sound/soc/codecs/wm8988.c */
#elif(defined(CONFIG_SND_SOC_WM8988))
extern struct snd_soc_codec_dai wm8988_dai;
extern int wm8988_mute(struct snd_soc_codec_dai *dai, int mute);
extern int wm8988_codec_set_clk(int mclk, int rate);
#endif
/* drivers/video/fbmem.c */
extern int fb_set_var(struct fb_info *info, struct fb_var_screeninfo *var);

/* drivers/video/rk28_fb.c */
extern int win0fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
extern int get_lcd_width(void);
extern int get_lcd_height(void);

/* drivers/video/rk28_backlight.c */
extern void rk28_bl_suspend(android_early_suspend_t *h);
extern void rk28_bl_resume(android_early_suspend_t *h);

int rk28_hdmi_win0fb_set_par(int x, int y, int w, int h)
{
	int ret;
	struct fb_info *info = registered_fb[1]; // fb1
	struct fb_var_screeninfo var;

	var = info->var;
	var.nonstd = ((y << 20) & 0xfff00000) + 
	             ((x << 8 )& 0xfff00) + 3;	    //win0 ypos & xpos & format (ypos<<20 + xpos<<8 + format)    // 2
	var.grayscale = ((h << 20) & 0xfff00000) +
	                ((w << 8) & 0xfff00) + 0;   //win0 xsize & ysize;

	acquire_console_sem();
	info->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();

	if(ret < 0){
		E("fb_set_var err!\n");
	}

	return ret;
}

static int hdmi_arm_clk = 600;

void rk28_hdmi_enter(int resolution)
{
	struct fb_info *info = registered_fb[1];

	//关闭LCD背光
	rk28_bl_suspend((void *)0);

	//设置arm频率 594M
    hdmi_arm_clk = rockchip_clk_get_arm()/1000000;
    rockchip_clk_set_arm(594);

	//设置fb
	switch(resolution){
	case HDMI_1280x720:
		win0fb_ioctl(info, 0x5001, 7);
		rk28_hdmi_win0fb_set_par(0, 0, 1280, 720);
		break;
	case HDMI_720x576:
		win0fb_ioctl(info, 0x5001, 6);
		rk28_hdmi_win0fb_set_par(0, 0, 720, 576);
		break;
	default:
		E("unsupport resolution, set default 1280x720!\n");
		win0fb_ioctl(info, 0x5001, 7);
		rk28_hdmi_win0fb_set_par(0, 0, 1280, 720);
		break;
	}

#if(defined(CONFIG_SND_ROCKCHIP_SOC_RK1000))
	rk1000_codec_mute(&rk1000_codec_dai, 1);
	rk1000_codec_set_clk(12288000, 48000);
#elif(defined(CONFIG_SND_SOC_WM8988))
	wm8988_mute(&wm8988_dai, 1);
	wm8988_codec_set_clk(12288000, 48000);
#endif
}

void rk28_hdmi_exit(void)
{
	struct fb_info *info = registered_fb[1];

#if(defined(CONFIG_SND_ROCKCHIP_SOC_RK1000))
	rk1000_codec_set_clk(12000000, 48000);
	rk1000_codec_mute(&rk1000_codec_dai, 0);
#elif(defined(CONFIG_SND_SOC_WM8988))
	wm8988_codec_set_clk(12000000, 48000);
	wm8988_mute(&wm8988_dai, 0);
#endif

	//切回LCD显示
	win0fb_ioctl(info, 0x5001, 0);
	rk28_hdmi_win0fb_set_par(0, 0, get_lcd_width(), get_lcd_height());

	//打开LCD背光
	rk28_bl_resume((void *)0);

	//恢复arm频率
    rockchip_clk_set_arm(hdmi_arm_clk);
}

/**********************************************************************/



void anx7150_timer_handler(unsigned long arg)
{
	struct anx7150_dev_s *dev = (struct anx7150_dev_s *)arg;

	if(dev->timer_en_cnt > 0){
		HDMI_System_Task();
		dev->timer_en_cnt--;
	}

	if(dev->anx7150_enable == ENABLE){
		dev->timer_en_cnt = 1;
	}

	mod_timer(dev->timer, jiffies + (dev->rate * HZ) / 1000);
}

int anx7150_timer_init(struct anx7150_dev_s *dev, timer_handler_t handler)
{
	if (dev->timer != NULL){
		return 0;
	}

	dev->timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (dev->timer == NULL){
		E("malloc timer err!\n");
		return -ENOMEM;
	}

	setup_timer(dev->timer, handler, (unsigned long)dev);
	add_timer(dev->timer);
	mod_timer(dev->timer, jiffies + (dev->rate * HZ) / 1000);

	return 0;
}

void anx7150_timer_uninit(struct anx7150_dev_s *dev)
{
	if (dev->timer != NULL){
		del_timer(dev->timer);
		kfree(dev->timer);
		dev->timer = NULL;
	}
}

void anx7150_callback(unsigned char status)
{
	struct anx7150_dev_s *dev = &anx7150_dev;

	if (status){
		D("hdmi connect!\n");
	}else{
		D("hdmi disconnect!\n");
	}
	
	dev->connect_status = status;
	kill_fasync(&dev->async_queue, SIGIO, POLL_MSG);
}

void anx7150_output_enable(struct anx7150_dev_s *dev)
{
	if (dev->rk28_output_status == HDMI){
		D("hdmi output already enabled!!\n");
		return;
	}

	if (dev->anx7150_enable == DISABLE){
		E("anx7150 not enable!\n");
		return;
	}
	
	D("hdmi output enable!\n");
	dev->rk28_output_status = HDMI;	
	rk28_hdmi_enter(dev->anx7150_resolution);
	ANX7150_TMDS_Enable();
}

void anx7150_output_disable(struct anx7150_dev_s *dev)
{	
	if (dev->rk28_output_status == LCD){
		D("hdmi output already disabled!!\n");
		return;
	}
	
	D("hdmi output disable\n");
	dev->rk28_output_status = LCD;
	ANX7150_TMDS_Disable();
	rk28_hdmi_exit();
}

int anx7150_get_output_status(void){
	return anx7150_dev.rk28_output_status;
}

int anx7150_sys_init(struct anx7150_dev_s *dev)
{
	int ret;
	
	if (dev->anx7150_init_status == INITIALIZED){
		D("anx7150 already initialized!!\n");
		return 0;
	}
	
	ret = HDMI_System_Init();
	if(ret < 0){
		E("HDMI_System_Init err!\n");
		return -ENODEV;
	}
	
	ANX7150_TMDS_Disable();
	HDMI_Set_Auto_Detect(1);
	HDMI_Set_CallBack(anx7150_callback);

	dev->anx7150_init_status = INITIALIZED;

	return 0;
}

void anx7150_sys_uninit(struct anx7150_dev_s *dev)
{
	if (dev->anx7150_init_status == UNINITIALIZED){
		D("anx7150 already uninitialized!!\n");
		return;
	}

	dev->anx7150_init_status = UNINITIALIZED;
	HDMI_Set_CallBack(NULL);
}

int anx7150_enable(struct anx7150_dev_s *dev)
{
	int ret;

	printk("%s:%d\n",__FUNCTION__,__LINE__);
	if (dev->anx7150_enable == ENABLE){
		D("anx7150 already enabled!!\n");
		return 0;
	}

	ret = anx7150_sys_init(dev);
	if(ret < 0){
		E("anx7150_sys_init err!\n");
		return ret;
	}

	dev->anx7150_enable = ENABLE;	
	dev->rate = 200;
	dev->timer_en_cnt = 1;
	
	return 0;
}

int anx7150_disable(struct anx7150_dev_s *dev)
{
	printk("%s:%d\n",__FUNCTION__,__LINE__);
	if (dev->anx7150_enable == DISABLE){
		D("anx7150 already disabled!!\n");
		return 0;
	}
	
	anx7150_output_disable(dev);
	anx7150_sys_uninit(dev);
	
	dev->anx7150_enable = DISABLE;	
	dev->rate = 1000;
	dev->timer_en_cnt = 10;
	return 0;
}


int anx7150_set_output_resolution(struct anx7150_dev_s *dev, int resolution)
{
	switch(resolution){
	case HDMI_1280x720:
		break;
	case HDMI_720x576:
		break;
	default:
		E("Invalid resolution, set default 1280x720\n");
		resolution = HDMI_1280x720;
	}
	
	HDMI_Set_Video_Format(resolution);
	dev->anx7150_resolution = resolution;
	return 0;
}

int anx7150_get_output_resolution(void){
	return anx7150_dev.anx7150_resolution;
}


int anx7150_get_connect_status(struct anx7150_dev_s *dev)
{
	return dev->connect_status;
}

int anx7150_open(struct inode *inode, struct file *filp)
{
	struct anx7150_dev_s *dev = &anx7150_dev;

	filp->private_data = dev;
	return 0;

	if(dev->open_cnt != 0){
		E("anx7150 has been opened!\n");
		return -EBUSY;
	}else{
		dev->open_cnt = 1;	
		filp->private_data = dev;
		return 0;
	}	
}

ssize_t anx7150_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	char c = 0;
	int ret;
	
	ret = copy_from_user(&c, buff, 1);
	if(ret < 0){
		E("copy_from_user err!\n");
		return ret;
	}

	if(c == '1'){
		E("hdmi on\n");
		anx7150_enable(&anx7150_dev);
		anx7150_output_enable(&anx7150_dev);
	}
	else if(c == '0'){
		E("hdmi off\n");
		anx7150_enable(&anx7150_dev);
		anx7150_output_disable(&anx7150_dev);
	}

	return 1;
}

static int anx7150_fasync(int fd, struct file *filp, int mode)
{
	struct anx7150_dev_s *dev = filp->private_data;

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

int anx7150_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct anx7150_dev_s *dev = filp->private_data;
	int ret = 0;

	switch (cmd){
	case ANX7150_ENABLE:
		anx7150_enable(dev);
		break;
	case ANX7150_DISABLE:
		anx7150_disable(dev);
		break;
	case HDMI_OUTPUT_ENABLE:
		anx7150_output_enable(dev);
		break;
		
	case HDMI_OUTPUT_DISABLE:
		anx7150_output_disable(dev);
		break;
		
	case HDMI_OUTPUT_RESOLUTION:
		anx7150_set_output_resolution(dev, (int)arg);
		break;
		
	case HDMI_CONNECT_STATUS:
		ret = anx7150_get_connect_status(dev);
		ret = put_user(ret, (int *)arg);
		if(ret < 0){
			E("put_user err!\n");
		}
		break;
		
	default:
		E("unknown ioctl cmd!\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

int anx7150_release(struct inode *inode, struct file *filp)
{
	struct anx7150_dev_s *dev = filp->private_data;
	
	if (dev->open_cnt != 0){
		anx7150_fasync(-1, filp, 0);
		if (dev->anx7150_enable == ENABLE ||
			dev->rk28_output_status == HDMI){
			anx7150_disable(dev);
		}
		
		dev->open_cnt = 0;
	}

	return 0;
}

static struct file_operations anx7150_fops = {
	.owner   = THIS_MODULE,
	.open    = anx7150_open,
	.write   = anx7150_write,
	.ioctl   = anx7150_ioctl,
	.fasync  = anx7150_fasync,
	.release = anx7150_release,
};

static int __init anx7150_init(void)
{
	int ret;

	/* step 0 init anx7150_dev */
	anx7150_dev.misc_dev.minor = MISC_DYNAMIC_MINOR;
    anx7150_dev.misc_dev.name = DEVICE_NAME;
    anx7150_dev.misc_dev.fops = &anx7150_fops;
    
	anx7150_dev.open_cnt = 0;
	anx7150_dev.rate = 1000;
	anx7150_dev.anx7150_init_status = UNINITIALIZED;
	anx7150_dev.anx7150_enable = DISABLE;
	anx7150_dev.anx7150_resolution = HDMI_1280x720;
	anx7150_dev.connect_status = UnPlug;
	anx7150_dev.rk28_output_status = LCD;
	anx7150_dev.timer_en_cnt = 0;
	anx7150_dev.timer = NULL;
	anx7150_dev.async_queue = NULL;
	
	D("anx7150_init!!\n");
	
	/* step 1 i2c_add_driver */
	ret = i2c_add_driver(&anx7150_i2c_driver);
	if (ret < 0){
		E("i2c_add_driver err!\n");
		goto err1;
	}

	/* step 2 add misc device */
	ret = misc_register(&anx7150_dev.misc_dev);
	if (ret < 0){
		E("misc_register err!\n");
		goto err2;
	}
	
	/* step 3 init timer */
	ret = anx7150_timer_init(&anx7150_dev, anx7150_timer_handler);
	if (ret < 0){
		E("anx7150_timer_init err!\n");
		goto err3;
	}

	return 0;

err3:
	misc_deregister(&anx7150_dev.misc_dev);
err2:
	i2c_del_driver(&anx7150_i2c_driver);
err1:
	return ret;
}

static void __exit anx7150_exit(void)
{
	D("anx7150_exit!!\n");

	anx7150_timer_uninit(&anx7150_dev);
	misc_deregister(&anx7150_dev.misc_dev);
	i2c_del_driver(&anx7150_i2c_driver);
}

module_init(anx7150_init);
module_exit(anx7150_exit);

MODULE_DESCRIPTION ("ANX7150 driver");
MODULE_AUTHOR("zyy<zyy@rock-chips.com>");
MODULE_LICENSE("GPL");

