typedef enum _SCREEN_TYPE {
    SCREEN_NULL = 0,
    SCREEN_RGB,
	SCREEN_MCU,
    SCREEN_TVOUT,
    SCREEN_HDMI,
} SCREEN_TYPE;

/* Sceen description */
struct rk28fb_screen {
    /* screen type & out face */
    u16 type;
    u16 face;

	/* Screen size */
	u16 x_res;
	u16 y_res;

    /* Timing */
	u16 pixclock;
	u16 left_margin;
	u16 right_margin;
	u16 hsync_len;
	u16 upper_margin;
	u16 lower_margin;
	u16 vsync_len;

	/* Pin polarity */
	u8 pin_hsync;
	u8 pin_vsync;
	u8 pin_den;
	u8 pin_dclk;
	u8 pin_dispon;

	/* Swap rule */
    u8 swap_rb;
    u8 swap_rg;
    u8 swap_gb;
    u8 swap_delta;
    u8 swap_dumy;

    /* Operation function*/
    int (*init)(void);
    int (*standby)(u8 enable);

};

extern void set_lcd_info(struct rk28fb_screen *screen);
extern void set_tv_info(struct rk28fb_screen *screen);
extern void set_hdmi_info(struct rk28fb_screen *screen);


