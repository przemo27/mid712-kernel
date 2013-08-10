//  ANALOGIX Company
//  ANX7150 Demo Firmware
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/arch/anx7150.h>
#include "anx7150_sys.h"
#include <asm/arch/gpio.h>

#undef NULL
#define NULL ((void *)0)

//#define DEBUG

#ifdef DEBUG
//#define D(fmt, arg...) printk("<7>%s:%d: " fmt, __FILE__, __LINE__, ##arg)
#define D(fmt, arg...) printk("<7>```%s:%d: " fmt, __FUNCTION__, __LINE__, ##arg)
#define D1(fmt, arg...) printk(fmt, ##arg)
#else
#define D(fmt, arg...)
#define D1(fmt, arg...)
#endif
#define E(fmt, arg...) printk("<3>!!!%s:%d: " fmt, __FILE__, __LINE__, ##arg)

/*******************ANX7150 I2C ADDR*************************/
#define ANX7150_I2C_ADDR0 0x72
#define ANX7150_I2C_ADDR1 0x7A

void DRVDelayMs(int n)
{
//	mdelay(n);
}

//#ifdef ITU656
struct ANX7150_video_timingtype ANX7150_video_timingtype_table =
{
    //640x480p-60hz
    {0x20/*H_RES_LOW*/, 0x03/*H_RES_HIGH*/,0x80 /*ACT_PIX_LOW*/,0x02 /*ACT_PIX_HIGH*/,
        0x60/*HSYNC_WIDTH_LOW*/,0x00 /*HSYNC_WIDTH_HIGH*/,0x30 /*H_BP_LOW*/,0x00 /*H_BP_HIGH*/,
        0xe0/*ACT_LINE_LOW*/, 0x01/*ACT_LINE_HIGH*/,0x02 /*VSYNC_WIDTH*/, 0x21/*V_BP_LINE*/,
        0x0a/*V_FP_LINE*/,0x10 /*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
        ANX7150_Progressive, ANX7150_Neg_Hsync_pol, ANX7150_Neg_Vsync_pol},
    //720x480p-60hz
    {0x5a/*H_RES_LOW*/,0x03 /*H_RES_HIGH*/,0xd0/*ACT_PIX_LOW*/, 0x02/*ACT_PIX_HIGH*/,
     0x3e/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0x3c/*H_BP_LOW*/, 0x00/*H_BP_HIGH*/,
     0xe0/*ACT_LINE_LOW*/, 0x01/*ACT_LINE_HIGH*/, 0x06/*VSYNC_WIDTH*/, 0x1e/*V_BP_LINE*/,
     0x09/*V_FP_LINE*/, 0x10/*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
     ANX7150_Progressive, ANX7150_Neg_Hsync_pol, ANX7150_Neg_Vsync_pol},
    //720p-60hz
    {0x72/*H_RES_LOW*/, 0x06/*H_RES_HIGH*/, 0x00/*ACT_PIX_LOW*/, 0x05/*ACT_PIX_HIGH*/,
     0x28/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0xdc/*H_BP_LOW*/, 0x00/*H_BP_HIGH*/,
     0xd0/*ACT_LINE_LOW*/, 0x02/*ACT_LINE_HIGH*/, 0x05/*VSYNC_WIDTH*/, 0x14/*V_BP_LINE*/,
     0x05/*V_FP_LINE*/, 0x6e/*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
     ANX7150_Progressive, ANX7150_Pos_Hsync_pol, ANX7150_Pos_Vsync_pol},
    //1080i-60hz
    {0x98/*H_RES_LOW*/, 0x08/*H_RES_HIGH*/, 0x80/*ACT_PIX_LOW*/, 0x07/*ACT_PIX_HIGH*/,
     0x2c/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0x94/*H_BP_LOW*/, 0x00/*H_BP_HIGH*/,
     0x38/*ACT_LINE_LOW*/, 0x04/*ACT_LINE_HIGH*/, 0x05/*VSYNC_WIDTH*/, 0x0f/*V_BP_LINE*/,
     0x02/*V_FP_LINE*/, 0x58/*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
     ANX7150_Interlace, ANX7150_Pos_Hsync_pol, ANX7150_Pos_Vsync_pol},
    //720x480i-60hz
    {0x5a/*H_RES_LOW*/,0x03 /*H_RES_HIGH*/,0xd0/*ACT_PIX_LOW*/, 0x02/*ACT_PIX_HIGH*/,
     0x3e/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0x39/*H_BP_LOW*/, 0x00/*H_BP_HIGH*/,
     0xe0/*ACT_LINE_LOW*/, 0x01/*ACT_LINE_HIGH*/, 0x03/*VSYNC_WIDTH*/, 0x0f/*V_BP_LINE*/,
     0x04/*V_FP_LINE*/, 0x13/*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
     ANX7150_Interlace, ANX7150_Neg_Hsync_pol, ANX7150_Neg_Vsync_pol},											//update
    //576p-50hz
    {0x60/*H_RES_LOW*/,0x03 /*H_RES_HIGH*/,0xd0 /*ACT_PIX_LOW*/, 0x02/*ACT_PIX_HIGH*/,
     0x40/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0x44/*H_BP_LOW*/,0x00 /*H_BP_HIGH*/,
     0x40/*ACT_LINE_LOW*/, 0x02/*ACT_LINE_HIGH*/, 0x05/*VSYNC_WIDTH*/, 0x27/*V_BP_LINE*/,
     0x05/*V_FP_LINE*/, 0x0c/*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
     ANX7150_Progressive, ANX7150_Neg_Hsync_pol, ANX7150_Neg_Vsync_pol},
    //720p-50hz
    {0xbc/*H_RES_LOW*/, 0x07/*H_RES_HIGH*/, 0x00/*ACT_PIX_LOW*/, 0x05/*ACT_PIX_HIGH*/,
     0x28/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0xdc/*H_BP_LOW*/, 0x00/*H_BP_HIGH*/,
     0xd0/*ACT_LINE_LOW*/, 0x02/*ACT_LINE_HIGH*/, 0x05/*VSYNC_WIDTH*/, 0x14/*V_BP_LINE*/,
     0x05/*V_FP_LINE*/, 0xb8/*H_FP_LOW*/, 0x01/*H_FP_HIGH*/,
     ANX7150_Progressive, ANX7150_Pos_Hsync_pol, ANX7150_Pos_Vsync_pol},
    //1080i-50hz
    {0x50/*H_RES_LOW*/, 0x0a/*H_RES_HIGH*/, 0x80/*ACT_PIX_LOW*/, 0x07/*ACT_PIX_HIGH*/,
     0x2c/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0x94/*H_BP_LOW*/, 0x00/*H_BP_HIGH*/,
     0x38/*ACT_LINE_LOW*/, 0x04/*ACT_LINE_HIGH*/, 0x05/*VSYNC_WIDTH*/, 0x0f/*V_BP_LINE*/,
     0x02/*V_FP_LINE*/, 0x10/*H_FP_LOW*/, 0x02/*H_FP_HIGH*/,
     ANX7150_Interlace, ANX7150_Pos_Hsync_pol, ANX7150_Pos_Vsync_pol},
    //576i-50hz
    {0x60/*H_RES_LOW*/,0x03 /*H_RES_HIGH*/,0xd0 /*ACT_PIX_LOW*/, 0x02/*ACT_PIX_HIGH*/,
     0x3f/*HSYNC_WIDTH_LOW*/, 0x00/*HSYNC_WIDTH_HIGH*/, 0x45/*H_BP_LOW*/,0x00 /*H_BP_HIGH*/,
     0x40/*ACT_LINE_LOW*/,0x02 /*ACT_LINE_HIGH*/, 0x03/*VSYNC_WIDTH*/, 0x13/*V_BP_LINE*/,
     0x02/*V_FP_LINE*/, 0x0c/*H_FP_LOW*/, 0x00/*H_FP_HIGH*/,
     ANX7150_Interlace, ANX7150_Neg_Hsync_pol, ANX7150_Neg_Vsync_pol},
};
//#endif

pCallBack ANX7150_callback = NULL;

uint8 g_video_format = 0x00;
uint8 g_audio_format = 0x00;


uint8 timer_slot = 0;
uint8 ANX7150_EDID_Buf[256];
uint8 ANX7150_avi_data[19];//, ANX7150_avi_checksum;
uint8 ANX7150_system_state;
uint8 spdif_error_cnt = 0x00;
uint8 misc_reset_needed;
uint8 ANX7150_stdaddr,ANX7150_stdreg,ANX7150_ext_block_num;
uint8 ANX7150_svd_length,ANX7150_sau_length;
uint8 ANX7150_edid_dtd[18];
WORD ANX7150_edid_length;
ANX7150_edid_result_4_system ANX7150_edid_result;

uint8 ANX7150_ddc_fifo_full;
uint8 ANX7150_ddc_progress;
uint8 ANX7150_hdcp_auth_en;
//uint8 ANX7150_bksv_ready; //replace by srm_checked xy 01.09
uint8 ANX7150_HDCP_enable;
uint8 ANX7150_ksv_srm_pass;
uint8 ANX7150_hdcp_bcaps;
uint8 ANX7150_hdcp_bstatus[2];
uint8 ANX7150_srm_checked;
uint8 ANX7150_hdcp_auth_pass;
uint8 ANX7150_avmute_enable;
uint8 ANX7150_send_blue_screen;
uint8 ANX7150_hdcp_encryption;
uint8 ANX7150_hdcp_init_done;
uint8 ANX7150_hdcp_wait_100ms_needed;
uint8 ANX7150_auth_fully_pass;
uint8 ANX7150_parse_edid_done;//060714 XY
//uint8 testen;
//uint8 ANX7150_avi_data[19], ANX7150_avi_checksum;
uint8 ANX7150_hdcp_auth_fail_counter ;

uint8 ANX7150_video_format_config;
uint8 ANX7150_emb_sync_mode,ANX7150_de_gen_en,ANX7150_demux_yc_en,ANX7150_ddr_bus_mode;
uint8 ANX7150_ddr_edge,ANX7150_ycmux_uint8_sel;
uint8 ANX7150_system_config_done;
uint8 ANX7150_RGBorYCbCr; //modified by zy 060814
uint8 ANX7150_in_pix_rpt,ANX7150_tx_pix_rpt;
uint8 ANX7150_in_pix_rpt_bkp,ANX7150_tx_pix_rpt_bkp;
uint8 ANX7150_video_timing_id;
uint8 ANX7150_pix_rpt_set_by_sys;
uint8 ANX7150_video_timing_parameter[18];
uint8 switch_value_sw_backup,switch_value_pc_backup;
uint8 switch_value,bist_switch_value_pc;
uint8 ANX7150_new_csc,ANX7150_new_vid_id,ANX7150_new_HW_interface;
uint8 ANX7150_INT_Done;

audio_config_struct s_ANX7150_audio_config;
config_packets s_ANX7150_packet_config;

uint8 FREQ_MCLK;         //0X72:0X50 uint82:0
//000b:Fm = 128*Fs
//001b:Fm = 256*Fs
//010b:Fm = 384*Fs
//011b:Fm = 512*Fs
uint8 ANX7150_audio_clock_edge;
uint8 HDMI_Mode_Auto_Manual,HDMI_Lowpower_Mode;

void ANX7150_Task()
{
    ANX7150_Interrupt_Process();
    ANX7150_Timer_Process ();

}

void ANX7150_Timer_Slot1()
{
    uint8 c;

    if (ANX7150_system_state == ANX7150_INITIAL)
    {
        return;
    }

    if (ANX7150_system_state == ANX7150_WAIT_HOTPLUG)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_INTR_STATE_REG, &c);
        if ((c & 0x01)&&(HDMI_Get_Auto_Detect()))
        {
            ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL3_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL3_REG, c | 0x01);//power up all, 090630
            if (ANX7150_callback)
                ANX7150_callback(1);    //set hotplug flag
            ANX7150_Hotplug_Change_Interrupt();
            D("ANX7150 start normal work in auto mode.\n");
        }
        else if ((c & 0x01)&&((HDMI_Get_Auto_Detect()==0x00)&&(HDMI_Get_Lowpower()==0x00)))
        {
            ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL3_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL3_REG, c | 0x01);//power up all, 090630
            ANX7150_Hotplug_Change_Interrupt();
            D("ANX7150 start normal work in manual mode.\n");

        }
        else
        {
            //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL3_REG, &c);
            //ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL3_REG, c & 0xfe);
            D("ANX7150 hold in standby mode.\n");
        }
    }


    if (ANX7150_system_state == ANX7150_READ_PARSE_EDID)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c &(~ANX7150_HDCP_CTRL0_HW_AUTHEN)));
        ANX7150_hdcp_auth_en = 0;
#if EDID_Parse_Enable

        ANX7150_RST_DDCChannel();

        ANX7150_Parse_EDID();
#else
        ANX7150_edid_result.is_HDMI=1;
#endif

        ANX7150_parse_edid_done = 1;

        ANX7150_Set_System_State(ANX7150_WAIT_RX_SENSE);//060819
    }

    if (ANX7150_system_state == ANX7150_WAIT_RX_SENSE)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_STATE_REG, &c);
        if (c & ANX7150_SYS_STATE_RSV_DET)
        {
            //D("Receiver sense active.\n");
            ANX7150_Set_System_State(ANX7150_CONFIG_VIDEO);//060819
        }
        else
        {
            ANX7150_Set_System_State(ANX7150_CONFIG_VIDEO);//060819 //jack wen
        }
    }
}

void ANX7150_Timer_Slot2(void)
{
    if (ANX7150_system_state == ANX7150_CONFIG_VIDEO)
    {
        ANX7150_Config_Video();
    }

    if (ANX7150_system_state == ANX7150_CONFIG_AUDIO)
    {
        ANX7150_Config_Audio();
    }

    if (ANX7150_system_state == ANX7150_CONFIG_PACKETS)
    {
        ANX7150_Config_Packet();
    }
}

void ANX7150_Timer_Slot3(void)
{

    if (ANX7150_system_state == ANX7150_HDCP_AUTHENTICATION)
    {
        ANX7150_HDCP_Process();
    }

    if (ANX7150_system_state == ANX7150_PLAY_BACK)
    {
        ANX7150_PLAYBACK_Process();
    }
}

void ANX7150_Timer_Slot4()
{
    ;
}

void ANX7150_Variable_Initial()
{
    uint8 i;
    ANX7150_Set_System_State(ANX7150_INITIAL);
    ANX7150_hdcp_auth_en = 0;
    ANX7150_ksv_srm_pass =0;
    ANX7150_srm_checked = 0;
    ANX7150_hdcp_auth_pass = 0;
    ANX7150_avmute_enable = 1;
    ANX7150_hdcp_auth_fail_counter =0;
    ANX7150_hdcp_encryption = 0;
    ANX7150_send_blue_screen = 0;
    ANX7150_hdcp_init_done = 0;
    ANX7150_hdcp_wait_100ms_needed = 1;
    ANX7150_auth_fully_pass = 0;
    timer_slot = 0;
    //********************for video config**************
    ANX7150_video_timing_id = 0;
    ANX7150_in_pix_rpt = 0;
    ANX7150_tx_pix_rpt = 0;
    ANX7150_new_csc = 0;
    ANX7150_new_vid_id = 0;
    ANX7150_new_HW_interface = 0;
    //********************end of video config*********

    //********************for edid parse***********
    ANX7150_edid_result.is_HDMI = 0;
    ANX7150_edid_result.ycbcr422_supported = 0;
    ANX7150_edid_result.ycbcr444_supported = 0;
    ANX7150_edid_result.supported_720p_60Hz = 0;
    ANX7150_edid_result.supported_720p_50Hz = 0;
    ANX7150_edid_result.supported_576p_50Hz = 0;
    ANX7150_edid_result.supported_576i_50Hz = 0;
    ANX7150_edid_result.supported_1080i_60Hz = 0;
    ANX7150_edid_result.supported_1080i_50Hz = 0;
    ANX7150_edid_result.supported_640x480p_60Hz = 0;
    ANX7150_edid_result.supported_720x480p_60Hz = 0;
    ANX7150_edid_result.supported_720x480i_60Hz = 0;
    ANX7150_edid_result.edid_errcode = 0;
    ANX7150_edid_result.SpeakerFormat = 0;
    for (i = 0; i < 8; i ++)
    {
        ANX7150_edid_result.AudioChannel[i] = 0;
        ANX7150_edid_result.AudioFormat[i] = 0;
        ANX7150_edid_result.AudioFs[i] = 0;
        ANX7150_edid_result.AudioLength[i] = 0;
    }
    //********************end of edid**************

    s_ANX7150_packet_config.packets_need_config = 0x03;   //new avi infoframe
    s_ANX7150_packet_config.avi_info.type = 0x82;
    s_ANX7150_packet_config.avi_info.version = 0x02;
    s_ANX7150_packet_config.avi_info.length = 0x0d;
    s_ANX7150_packet_config.avi_info.pb_uint8[1] = 0x21;//YCbCr422
    s_ANX7150_packet_config.avi_info.pb_uint8[2] = 0x08;
    s_ANX7150_packet_config.avi_info.pb_uint8[3] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[4] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[5] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[6] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[7] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[8] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[9] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[10] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[11] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[12] = 0x00;
    s_ANX7150_packet_config.avi_info.pb_uint8[13] = 0x00;

    // audio infoframe
    s_ANX7150_packet_config.audio_info.type = 0x84;
    s_ANX7150_packet_config.audio_info.version = 0x01;
    s_ANX7150_packet_config.audio_info.length = 0x0a;
    s_ANX7150_packet_config.audio_info.pb_uint8[1] = 0x00;  //zy 061123 for ATC
    s_ANX7150_packet_config.audio_info.pb_uint8[2] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[3] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[4] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[5] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[6] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[7] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[8] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[9] = 0x00;
    s_ANX7150_packet_config.audio_info.pb_uint8[10] = 0x00;

    ANX7150_INT_Done = 0;
}

void ANX7150_HW_Interface_Variable_Initial()
{
    uint8 c;

    ANX7150_video_format_config = 0x00;
    ANX7150_RGBorYCbCr = 0x00;
    ANX7150_ddr_edge = ANX7150_IDCK_EDGE_DDR;

    c = 0;
    c = (ANX7150_I2S_CH0_ENABLE << 2) | (ANX7150_I2S_CH1_ENABLE << 3) |
        (ANX7150_I2S_CH2_ENABLE << 4) | (ANX7150_I2S_CH3_ENABLE << 5);
    s_ANX7150_audio_config.audio_type = ANX7150_AUD_HW_INTERFACE;     // input I2S
    s_ANX7150_audio_config.down_sample = 0x00;
    s_ANX7150_audio_config.i2s_config.audio_channel = c;//0x04;
    s_ANX7150_audio_config.i2s_config.Channel_status1 =0x00;
    s_ANX7150_audio_config.i2s_config.Channel_status1 = 0x00;
    s_ANX7150_audio_config.i2s_config.Channel_status2 = 0x00;
    s_ANX7150_audio_config.i2s_config.Channel_status3 = 0x00;
    s_ANX7150_audio_config.i2s_config.Channel_status4 = 0x00;//0x02;//48k
    s_ANX7150_audio_config.i2s_config.Channel_status5 = ANX7150_I2S_WORD_LENGTH;//0x0b;
    s_ANX7150_audio_config.audio_layout = 0x00;

    c = (ANX7150_I2S_SHIFT_CTRL << 3) | (ANX7150_I2S_DIR_CTRL << 2)  |
        (ANX7150_I2S_WS_POL << 1) | ANX7150_I2S_JUST_CTRL;
    s_ANX7150_audio_config.i2s_config.i2s_format = c;//0x00;

    FREQ_MCLK = ANX7150_MCLK_Fs_RELATION;//set the relation of MCLK and WS
    ANX7150_audio_clock_edge = ANX7150_AUD_CLK_EDGE;


}

void ANX7150_Hotplug_Change_Interrupt(void)
{
    uint8 c,c1;
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_STATE_REG, &c1);
    if ((HDMI_Get_Lowpower())&&(HDMI_Get_Auto_Detect()==0x00))
    {
        D("ANX7150 run into standby in manual mode \n");
        ANX7150_Variable_Initial();   //simon
        ANX7150_HW_Interface_Variable_Initial();  //simon
        ANX7150_Hardware_Initial();   //simon
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, 0x00);   //simon

    }
    else if (c1 & ANX7150_SYS_STATE_HP)
    {
    	D("ANX7150 HotPlug detected.\n");

        //disable audio & video & hdcp & TMDS and init    begin
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));

        ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c & (~ANX7150_VID_CTRL_IN_EN));

		//zyy 2010.01.12
        //ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);
        //ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, c & (~ANX7150_TMDS_CLKCH_MUTE));

        ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_HW_AUTHEN)));

        ANX7150_Variable_Initial();
        //disable video & audio & hdcp & TMDS and init    end

        
        //Power on chip and select DVI mode
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c | 0x05);  //  cwz change 0x01 -> 0x05
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c & 0xfd);
        //D("ANX7150 is set to DVI mode\n");
        ANX7150_RST_DDCChannel();
        //Initial Interrupt
        // disable video/audio CLK,Format change and before config video. 060713 xy
        ANX7150_i2c_write_p0_reg(ANX7150_INTR1_MASK_REG, 0x04);//3
        ANX7150_i2c_write_p0_reg(ANX7150_INTR2_MASK_REG, 0x00);
        ANX7150_i2c_write_p0_reg(ANX7150_INTR3_MASK_REG, 0x00);

        ANX7150_i2c_read_p0_reg(ANX7150_INTR1_STATUS_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_INTR1_STATUS_REG, c);

        ANX7150_i2c_read_p0_reg(ANX7150_INTR2_STATUS_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_INTR2_STATUS_REG, c);

        ANX7150_i2c_read_p0_reg(ANX7150_INTR3_STATUS_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_INTR3_STATUS_REG, c);

        ANX7150_i2c_write_p0_reg(ANX7150_INTR_CTRL_REG, 0x00);
		ANX7150_Set_System_State(ANX7150_READ_PARSE_EDID);//060819

    }
    else
    {
        D("ANX7150 detect unplug \n");
        if (HDMI_Get_Auto_Detect())
        {
            if (ANX7150_callback)
                ANX7150_callback(0);    //Shut down HDMI output
        }
        //wen HDCP CTS
        ANX7150_Variable_Initial();   //simon
        ANX7150_HW_Interface_Variable_Initial();  //simon
        ANX7150_Hardware_Initial();   //simon
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, 0x00);   //simon
        //wen HDCP CTS
        ANX7150_hdcp_wait_100ms_needed = 1;
        ANX7150_auth_fully_pass = 0;
    }
    // clear ANX7150_parse_edid_done & ANX7150_system_config_done
    ANX7150_parse_edid_done = 0;
    ANX7150_system_config_done = 0;
    ANX7150_srm_checked = 0;
}

void ANX7150_Video_Clock_Change_Interrupt(void)
{
    uint8 c;
    if ((ANX7150_system_state != ANX7150_INITIAL) && (ANX7150_system_state != ANX7150_WAIT_HOTPLUG)
            &&  (ANX7150_system_state != ANX7150_READ_PARSE_EDID)
            &&  (ANX7150_system_state != ANX7150_WAIT_RX_SENSE))
    {
        ANX7150_Set_AVMute(); //wen
        //stop HDCP and reset DDC
        ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_HW_AUTHEN)));
        ANX7150_RST_DDCChannel();
        D("after video clock change int \n");
        ANX7150_Set_System_State(ANX7150_CONFIG_VIDEO);
    }
    //when clock change, clear this reg to avoid error in package config
    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, 0x00);	//wen
    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, 0x00);
    //xy 11.06 when clock change, need system config again
    ANX7150_system_config_done = 0;
}

void ANX7150_Video_Format_Change_Interrupt(void)
{
    uint8 c;
    if ((ANX7150_system_state != ANX7150_INITIAL)
            && (ANX7150_system_state != ANX7150_WAIT_HOTPLUG)
            &&  (ANX7150_system_state != ANX7150_READ_PARSE_EDID)
            && (ANX7150_system_state != ANX7150_WAIT_RX_SENSE))
    {
        D("after video format change int \n");
        ANX7150_Set_AVMute();//wen
        //stop HDCP and reset DDC
        ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_HW_AUTHEN)));
        ANX7150_RST_DDCChannel();
        ANX7150_Set_System_State(ANX7150_CONFIG_VIDEO);
    }
    //when format change, clear this reg to avoid error in package config
    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, 0x00);	//wen
    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, 0x00);
    //xy 11.06 when format change, need system config again
    ANX7150_system_config_done = 0;
}

void ANX7150_Audio_CLK_Change_Interrupt(void)
{

    uint8 c;
    if ((ANX7150_system_state != ANX7150_INITIAL)
            && (ANX7150_system_state != ANX7150_WAIT_HOTPLUG)
            && (ANX7150_system_state != ANX7150_READ_PARSE_EDID)
            && (ANX7150_system_state != ANX7150_WAIT_RX_SENSE)
            && (ANX7150_system_state != ANX7150_CONFIG_VIDEO))
    {
        D("ANX7150: audio clock changed interrupt,disable audio.\n");
        // disable audio
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG,&c);
        c &= ~ANX7150_HDMI_AUDCTRL1_IN_EN;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG,c);
        ANX7150_Set_System_State(ANX7150_CONFIG_AUDIO);
    }

    //xy 11.06 when format change, need system config again
    ANX7150_system_config_done = 0;
}

void ANX7150_Set_AVMute(void)
{
    uint8 c;

    ANX7150_i2c_write_p1_reg(ANX7150_GNRL_CTRL_PKT_REG, 0x01);//wen
    ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, (c | 0x0c));
    ANX7150_avmute_enable = 1;
//    D("@@@@@@@@@@@@@@@@@@@@ANX7150_Set_AVMute\n");

}

void ANX7150_Clear_AVMute(void)
{
    uint8 c;
    ANX7150_i2c_write_p1_reg(ANX7150_GNRL_CTRL_PKT_REG, 0x02);
    ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, (c | 0x0c));
    ANX7150_avmute_enable = 0;
//    D("@@@@@@@@@@@@@@@@@@@@ANX7150_Clear_AVMute\n");

}

void ANX7150_TMDS_Disable(void)
{
	unsigned char c;
	D("Disable TMDS clock!\n");

	/* audio stream disable */
	ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));

	/* video stream disable */
	ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c & (~ANX7150_VID_CTRL_IN_EN));

	/* TMDS disable */
	ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, c & (~ANX7150_TMDS_CLKCH_MUTE));
}

void ANX7150_TMDS_Enable(void)
{
	unsigned char c;
	
	D("Enable TMDS clock output!\n");

	/* audio stream enable */
	ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c | ANX7150_HDMI_AUDCTRL1_IN_EN);

	/* video stream enable */
	ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c |ANX7150_VID_CTRL_IN_EN);

	/* TMDS enable */
    ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, c | ANX7150_TMDS_CLKCH_MUTE);
}

void ANX7150_Auth_Done_Interrupt()
{
    uint8 c;
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_STATUS_REG, &c);
    if (c & ANX7150_HDCP_STATUS_AUTH_PASS)
    {
        D("ANX7150_Authentication pass in Auth_Done\n");
        ANX7150_Blue_Screen_Disable();
        ANX7150_hdcp_auth_pass = 1;
        ANX7150_hdcp_auth_fail_counter = 0;
    }
    else
    {
        D("ANX7150_Authentication failed\n");
        ANX7150_hdcp_wait_100ms_needed = 1;
        ANX7150_auth_fully_pass = 0;
        ANX7150_hdcp_auth_pass = 0;
        ANX7150_hdcp_auth_fail_counter ++;
        if (ANX7150_hdcp_auth_fail_counter >= ANX7150_HDCP_FAIL_THRESHOLD)
        {
            ANX7150_hdcp_auth_fail_counter = 0;
            //ANX7150_bksv_ready = 0;
            // TODO: Reset link;
            ANX7150_Blue_Screen_Enable();
            ANX7150_HDCP_Encryption_Disable();
            //disable audio
            ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));
        }
    }
}

void ANX7150_Auth_Change_Interrupt()
{
    uint8 c;
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_STATUS_REG, &c);
    if (c & ANX7150_HDCP_STATUS_AUTH_PASS)
    {
        ANX7150_hdcp_auth_pass = 1;
        D("ANX7150_Authentication pass in Auth_Change\n");
    }
    else
    {
        ANX7150_Set_AVMute(); //wen
        D("ANX7150_Authentication failed_by_Auth_change\n");
        ANX7150_hdcp_auth_pass = 0;
        ANX7150_hdcp_wait_100ms_needed = 1;
        ANX7150_auth_fully_pass = 0;
        ANX7150_hdcp_init_done=0;   //wen HDCP CTS
        ANX7150_hdcp_auth_en=0;   //wen HDCP CTS
        ANX7150_HDCP_Encryption_Disable();
        if (ANX7150_system_state == ANX7150_PLAY_BACK)
        {
            ANX7150_auth_fully_pass = 0;
            ANX7150_Set_System_State(ANX7150_HDCP_AUTHENTICATION);//ANX7150_CONFIG_VIDEO);	//wen updated for Changhong TV;jack wen
            //disable audio
            ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));
            ANX7150_Clean_HDCP();							//wen updated for Changhong TV
        }
    }
}


void ANX7150_AFIFO_Overrun_Interrupt(void)
{
    uint8 c;

    if (ANX7150_system_state != ANX7150_INITIAL
            && ANX7150_system_state != ANX7150_WAIT_HOTPLUG
            && ANX7150_system_state != ANX7150_READ_PARSE_EDID
            && ANX7150_system_state != ANX7150_WAIT_RX_SENSE
            && ANX7150_system_state != ANX7150_CONFIG_VIDEO)
    {
        D("ANX7150: AFIFO overrun interrupt,disable audio.\n");
        // disable audio
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG,&c);
        c &= ~ANX7150_HDMI_AUDCTRL1_IN_EN;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG,c);
        ANX7150_Set_System_State(ANX7150_CONFIG_AUDIO);
    }
}

void ANX7150_PllLock_Interrupt()
{
    uint8 c;

    ANX7150_i2c_read_p0_reg(ANX7150_CHIP_STATUS_REG,&c);
    if((c&0x01) == 0)
	{
	    if ((ANX7150_system_state != ANX7150_INITIAL)
	            && (ANX7150_system_state != ANX7150_WAIT_HOTPLUG)
	            && (ANX7150_system_state != ANX7150_READ_PARSE_EDID)
	            && (ANX7150_system_state != ANX7150_WAIT_RX_SENSE))
	    {
	        ANX7150_Set_AVMute();//wen
	        D("ANX7150: PLL unlock interrupt,disable audio.\n");
	        // disable audio & video
	        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG,&c);
	        c &= ~ANX7150_HDMI_AUDCTRL1_IN_EN;
	        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG,c);

	        ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG,&c);
        c &= ~ANX7150_VID_CTRL_IN_EN;
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG,c);
        ANX7150_Set_System_State(ANX7150_CONFIG_VIDEO);
	    }
	    //when pll change, clear this reg to avoid error in package config
	    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, 0x00);	//wen
	    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, 0x00);

	    ANX7150_system_config_done = 0;//jack wen
	}
}

void ANX7150_SPDIF_Error_Interrupt(uint8 int1, uint8 int3)
{
	D("int1 = 0x%.2x, int3 = 0x%.2x\n", int1, int3);
	
    if ((ANX7150_system_state == ANX7150_CONFIG_AUDIO
            || ANX7150_system_state == ANX7150_CONFIG_PACKETS
            || ANX7150_system_state == ANX7150_HDCP_AUTHENTICATION
            || ANX7150_system_state == ANX7150_PLAY_BACK )
            && (int3 & 0x81))
    {
        D("SPDIF BI Phase or Unstable error.\n");
        spdif_error_cnt += 0x03;
    }
    if ((ANX7150_system_state == ANX7150_CONFIG_AUDIO
            || ANX7150_system_state == ANX7150_CONFIG_PACKETS
            || ANX7150_system_state == ANX7150_HDCP_AUTHENTICATION
            || ANX7150_system_state == ANX7150_PLAY_BACK )
            && (int1 & ANX7150_INTR1_STATUS_SPDIF_ERR))
    {
        D("SPDIF Parity error.\n");
        spdif_error_cnt += 0x01;
    }
    // adjust spdif phase
    if (spdif_error_cnt >= spdif_error_th)
    {
        uint8 freq_mclk,c1,c;
        spdif_error_cnt = 0x00;
        D("adjust mclk phase!\n");
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
        ANX7150_i2c_read_p0_reg(ANX7150_I2S_CTRL_REG, &c1);

        freq_mclk = c & 0x07;
        switch (freq_mclk)
        {
            case ANX7150_mclk_128_Fs:   //invert 0x50[3]
                D("adjust mclk phase when 128*Fs!\n");
                if ( c & 0x08 )    c &= 0xf7;
                else   c |= 0x08;
                ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, c);
                break;

            case ANX7150_mclk_256_Fs:
            case ANX7150_mclk_384_Fs:
                D("adjust mclk phase when 256*Fs or 384*Fs!\n");
                if ( c1 & 0x60 )   c1 &= 0x9f;
                else     c1 |= 0x20;
                ANX7150_i2c_write_p0_reg(ANX7150_I2S_CTRL_REG, c1);
                break;

            case ANX7150_mclk_512_Fs:
                D("adjust mclk phase when 512*Fs!\n");
                if ( c1 & 0x60 )   c1 &= 0x9f;
                else    c1 |= 0x40;
                ANX7150_i2c_write_p0_reg(ANX7150_I2S_CTRL_REG, c1);
                break;
            default:
                break;

        }
    }
}

void ANX7150_Rx_Sense_Interrupt(void)
{
    uint8 c;

    ANX7150_i2c_read_p0_reg(ANX7150_SYS_STATE_REG,&c);
    D("ANX7150_Rx_Sense_Interrupt, ANX7150_SYS_STATE_REG = %.2x\n", (unsigned int)c); //wen

    if ( c & ANX7150_SYS_STATE_RSV_DET)
    {
        //xy 11.06 Power on chip
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c | 0x01);
        s_ANX7150_packet_config.packets_need_config = 0x03;   //new avi infoframe	wen
    }
    else
    {
        // Rx is not active
        if ((ANX7150_system_state != ANX7150_INITIAL)
                && (ANX7150_system_state != ANX7150_WAIT_HOTPLUG)
                && (ANX7150_system_state != ANX7150_READ_PARSE_EDID))
        {
            ANX7150_Set_System_State(ANX7150_WAIT_RX_SENSE);
            //stop HDCP and reset DDC when lost Rx sense
            ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_HW_AUTHEN)));
            ANX7150_RST_DDCChannel();
            ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c & 0xfd);
            // mute TMDS link
            //ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);
            //ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, c & (~ANX7150_TMDS_CLKCH_MUTE));
        }
        //Power down chip
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c & 0xfe);
    }
    //xy 11.06 when format change, need system config again
    ANX7150_system_config_done = 0;//wen HDCP CTS
}

void ANX7150_HDCPLINK_CHK_Interrupt(void)
{
    uint8 c,c1, ri0,ri1;
    //ANX7150_InitDDC_Read(0x74, segmentpointer, offset, 0x01, 0x00);
    //Write slave device address
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_ADDR_REG, 0x74);
    // Write segment address
    //ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_SEGADDR_REG, segmentpointer);
    //Write offset
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_OFFADDR_REG, 0x08);
    //Write number for access
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACCNUM0_REG, 0x02);
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACCNUM1_REG, 0x00);
    //Clear FIFO
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x05);
    //EDDC sequential Read
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x01);

    DRVDelayMs(1);

    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &ri0);
    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &ri1);


    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_RI1_REG, &c);
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_RI2_REG, &c1);
    //D("DDC read Ri0= %.2x\n", (unsigned int)ri0);
    //D("DDC read Ri1= %.2x\n", (unsigned int)ri1);
    //D("###ANX7150 Ri0= %.2x\n", (unsigned int)c);
    //D("###ANX7150 Ri1= %.2x\n", (unsigned int)c1);
    if ((ri0 != c) || (ri1 != c1))
    {
        D("Ri check error!!!!!!!\n");

        ANX7150_hdcp_auth_fail_counter ++;
        if (ANX7150_hdcp_auth_fail_counter >= ANX7150_HDCP_FAIL_THRESHOLD)
        {
            ANX7150_Set_AVMute(); //wen
            D("ANX7150_Authentication failed_by_Ri check error!\n");
            ANX7150_hdcp_auth_pass = 0;
            ANX7150_hdcp_wait_100ms_needed = 1;
            ANX7150_auth_fully_pass = 0;
            ANX7150_hdcp_init_done=0;   //wen HDCP CTS
            ANX7150_hdcp_auth_en=0;   //wen HDCP CTS
            ANX7150_HDCP_Encryption_Disable();
            ANX7150_hdcp_auth_fail_counter = 0;
            if (ANX7150_system_state == ANX7150_PLAY_BACK)
            {
                ANX7150_auth_fully_pass = 0;
                ANX7150_Set_System_State(ANX7150_HDCP_AUTHENTICATION);//ANX7150_CONFIG_VIDEO);	//wen updated for Changhong TV;jack wen
                //disable audio
                ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
                ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));
                ANX7150_Clean_HDCP();							//wen updated for Changhong TV
            }
        }
    }

}


void ANX7150_Blue_Screen_Format_Config(void)
{
    // TODO:Add ITU 601 format.(Now only ITU 709 format added)
    switch (ANX7150_RGBorYCbCr)
    {
        case ANX7150_RGB: //select RGB mode
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN0_REG, 0x10);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN1_REG, 0xeb);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN2_REG, 0x10);
            break;
        case ANX7150_YCbCr422: //select YCbCr4:2:2 mode
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN0_REG, 0x00);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN1_REG, 0xad);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN2_REG, 0x2a);
            break;
        case ANX7150_YCbCr444: //select YCbCr4:4:4 mode
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN0_REG, 0x1a);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN1_REG, 0xad);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN2_REG, 0x2a);
            break;
        default:
            break;
    }
}

void ANX7150_Blue_Screen_Enable(void)
{
    uint8 c;
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL1_REG,  c | ANX7150_HDCP_CTRL1_BLUE_SCREEN_EN);
    ANX7150_send_blue_screen = 1;
}

void ANX7150_Blue_Screen_Disable(void)
{
    uint8 c;
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL1_REG,  (c & 0xfb));
    ANX7150_send_blue_screen = 0;
}

void ANX7150_HDCP_Encryption_Enable(void)
{
    uint8 c;
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c | ANX7150_HDCP_CTRL0_ENC_EN));
    ANX7150_hdcp_encryption = 1;
}

void ANX7150_HDCP_Encryption_Disable(void)
{
    uint8 c;
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & 0xfb));
    ANX7150_hdcp_encryption = 0;
}


//******************************Video Config***************************************
void ANX7150_Config_Video()
{
    uint8 c,TX_is_HDMI;
    uint8 cspace_y2r, y2r_sel, up_sample,range_y2r;

    cspace_y2r = 0;
    y2r_sel = 0;
    up_sample = 0;
    range_y2r = 0;


    //ANX7150_RGBorYCbCr = 0x00;						//RGB
    //ANX7150_RGBorYCbCr = ANX7150_INPUT_COLORSPACE;						//update
    ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, 0x00);			//update

    ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);					//jack wen
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c & (~ANX7150_VID_CTRL_uint8CTRL_EN));

    if (!ANX7150_system_config_done)
    {
        //D("System has not finished config!\n");
        return;
    }

    ANX7150_i2c_read_p0_reg(ANX7150_SYS_STATE_REG, &c);
    if (!(c & 0x02))
    {
        //D("No clock detected !\n");
        //ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, 0x02);
        return;
    }

    ANX7150_Clean_HDCP();

    //color space issue
    switch (ANX7150_video_timing_id)
    {
        case ANX7150_V1280x720p_50Hz:
        case ANX7150_V1280x720p_60Hz:
        case ANX7150_V1920x1080i_60Hz:
        case ANX7150_V1920x1080i_50Hz:
        case ANX7150_V1920x1080p_60Hz:
        case ANX7150_V1920x1080p_50Hz:
            y2r_sel = ANX7150_CSC_BT709;
            break;
        default:
            y2r_sel = ANX7150_CSC_BT601;
            break;
    }
    //rang[0~255]/[16~235] select
    if (ANX7150_video_timing_id == ANX7150_V640x480p_60Hz)
        range_y2r = 1;//rang[0~255]
    else
        range_y2r = 0;//rang[16~235]
    if ((ANX7150_RGBorYCbCr == ANX7150_YCbCr422) && (!ANX7150_edid_result.ycbcr422_supported))
    {
        up_sample = 1;
        if (ANX7150_edid_result.ycbcr444_supported)
            cspace_y2r = 0;
        else
            cspace_y2r = 1;
    }
    if ((ANX7150_RGBorYCbCr == ANX7150_YCbCr444) && (!ANX7150_edid_result.ycbcr444_supported))
    {
        cspace_y2r = 1;
    }
    //Config the embeded blue screen format according to output video format.
    ANX7150_Blue_Screen_Format_Config();

    ANX7150_Parse_Video_Format();

    if (ANX7150_de_gen_en)
    {
        //D("ANX7150_de_gen_en!\n");
        ANX7150_DE_Generator();
    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c & (~ANX7150_VID_CAPCTRL0_DEGEN_EN));
    }
    if (ANX7150_emb_sync_mode)
    {
        //D("ANX7150_Embed_Sync_Decode!\n");
        ANX7150_Embed_Sync_Decode();
        if (ANX7150_ddr_bus_mode) //jack wen; for DDR embeded sync
        {
            ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL4_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL4_REG, c | 0x04);
        }
        else
        {
            ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL4_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL4_REG, c & 0xfb);
        }
    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c & (~ANX7150_VID_CAPCTRL0_EMSYNC_EN));
    }
    if (ANX7150_demux_yc_en)
    {
        D("ANX7150_demux_yc_en!\n");
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c | ANX7150_VID_CAPCTRL0_DEMUX_EN);
        if (ANX7150_ycmux_uint8_sel)
        {
            ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c | ANX7150_VID_CTRL_YCuint8_SEL);
            //jack wen, uint8 mapping for yc mux, D13-8,1-0 -->D11-4
            D("ANX7150_demux_yc_en!####D11-4\n");

            ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c | ANX7150_VID_CTRL_uint8CTRL_EN);

            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL11,  0x0d);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL10,  0x0c);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL9,  0x0b);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL8,  0x0a);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL7,  0x09);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL6,  0x08);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL5,  0x01);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL4,  0x00);

            //
        }
        else
        {
            ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c & (~ANX7150_VID_CTRL_YCuint8_SEL));
            //jack wen, uint8 mapping for yc mux, D13-8,1-0 -->D15-8,
            D("ANX7150_demux_yc_en!####D15-8\n");

            ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c | ANX7150_VID_CTRL_uint8CTRL_EN);

            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL15, 0x0d);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL14, 0x0c);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL13,  0x0b);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL12, 0x0a);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL11, 0x09);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL10, 0x08);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL9, 0x01);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL8, 0x00);
            //

        }
    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c & (~ANX7150_VID_CAPCTRL0_DEMUX_EN));

        //jack wen

        //

    }
    if (ANX7150_ddr_bus_mode)
    {
        //D("ANX7150_ddr_bus_mode!\n");
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c | ANX7150_VID_CAPCTRL0_DV_BUSMODE);

        //jack wen
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL4_REG, &c); //jack wen, qfn48
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL4_REG, (c & 0xfc) | 0x02);
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c); //uint8 map for
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c |ANX7150_VID_CTRL_YCuint8_SEL);
        //jack wen

        if (ANX7150_ddr_edge)
        {
            ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c | ANX7150_VID_CAPCTRL0_DDR_EDGE);
        }
        else
        {
            ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c & (~ANX7150_VID_CAPCTRL0_DDR_EDGE));
        }

        //jack wen for DDR+seperate maping
        if (ANX7150_video_format_config == 0x07)//jack wen, DDR yc422, 601,
        {
            D("ANX7150_DDR_601_Maping!\n");

            ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c | ANX7150_VID_CTRL_uint8CTRL_EN);

            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL23,  0x0B);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL22,  0x0A);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL21,  0x09);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL20,  0x08);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL19,  0x07);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL18,  0x06);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL17,  0x05);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL16, 0x04);

            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL15,  0x17);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL14,  0x16);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL13,  0x15);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL12,  0x14);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL11,  0x13);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL10,  0x12);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL9,  0x11);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL8,  0x10);

            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL7,  0x03);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL6,  0x02);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL5,  0x01);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL4,  0x00);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL3,  0x0F);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL2,  0x0E);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL1, 0x0D);
            ANX7150_i2c_write_p0_reg(VID_uint8_CTRL0,  0x0C);

        }
        else if (ANX7150_video_format_config == 0x08)//jack wen, DDR yc422, 656,
        {
            D("ANX7150_DDR_656_Maping!\n");

            ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c & (~ANX7150_VID_CTRL_uint8CTRL_EN));

        }

    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c & (~ANX7150_VID_CAPCTRL0_DV_BUSMODE));
        ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c & (~ANX7150_VID_CAPCTRL0_DDR_EDGE));
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL4_REG, &c); //jack wen, qfn48
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL4_REG, c & 0xfc);
    }

    if (cspace_y2r)
    {
        //D("Color space Y2R enabled********\n");
        ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c | ANX7150_VID_MODE_CSPACE_Y2R);
        if (y2r_sel)
        {
            //D("Y2R_SEL!\n");
            ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c | ANX7150_VID_MODE_Y2R_SEL);
        }
        else
        {
            ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c & (~ANX7150_VID_MODE_Y2R_SEL));
        }
    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c & (~ANX7150_VID_MODE_CSPACE_Y2R));
    }

    if (up_sample)
    {
        //D("UP_SAMPLE!\n");
        ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c | ANX7150_VID_MODE_UPSAMPLE);
    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c & (~ANX7150_VID_MODE_UPSAMPLE));
    }

    if (range_y2r)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c | ANX7150_VID_MODE_RANGE_Y2R);
    }
    else
    {
        ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, c & (~ANX7150_VID_MODE_RANGE_Y2R));
    }

    if (!ANX7150_pix_rpt_set_by_sys)
    {
        if ((ANX7150_video_timing_id == ANX7150_V720x480i_60Hz_16x9)
                || (ANX7150_video_timing_id == ANX7150_V720x576i_50Hz_16x9)
                || (ANX7150_video_timing_id == ANX7150_V720x480i_60Hz_4x3)
                || (ANX7150_video_timing_id == ANX7150_V720x576i_50Hz_4x3))
            ANX7150_tx_pix_rpt = 1;
        else
            ANX7150_tx_pix_rpt = 0;
    }
    //set input pixel repeat times
    ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_MODE_REG, ((c & 0xfc) |ANX7150_in_pix_rpt));
    //set link pixel repeat times
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, ((c & 0xfc) |ANX7150_tx_pix_rpt));

    if ((ANX7150_in_pix_rpt != ANX7150_in_pix_rpt_bkp)
            ||(ANX7150_tx_pix_rpt != ANX7150_tx_pix_rpt_bkp) )
    {
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, 0x02);
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, 0x00);
        D("MISC_Reset!\n");
        ANX7150_in_pix_rpt_bkp = ANX7150_in_pix_rpt;
        ANX7150_tx_pix_rpt_bkp = ANX7150_tx_pix_rpt;
    }
    //enable video input
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CTRL_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CTRL_REG, c | ANX7150_VID_CTRL_IN_EN);

    //D("Video configure OK!\n");

    ANX7150_i2c_read_p0_reg(ANX7150_VID_STATUS_REG, &c);
    if (!(c & ANX7150_VID_STATUS_VID_STABLE))
    {
        D("Video not stable!\n");
        return;
    }
    if (cspace_y2r)
        ANX7150_RGBorYCbCr = ANX7150_RGB;
    //Enable video CLK,Format change after config video.
    // ANX7150_i2c_read_p0_reg(ANX7150_INTR1_MASK_REG, &c);
    // ANX7150_i2c_write_p0_reg(ANX7150_INTR1_MASK_REG, c |0x01);//3
    ANX7150_i2c_read_p0_reg(ANX7150_INTR2_MASK_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_INTR2_MASK_REG, c | 0x48);
    ANX7150_i2c_read_p0_reg(ANX7150_INTR3_MASK_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_INTR3_MASK_REG, c | 0x40);


    if (ANX7150_edid_result.is_HDMI)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c | 0x02);
        D("ANX7150 is set to HDMI mode\n");
    }

    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
    TX_is_HDMI = c & 0x02;

    if (TX_is_HDMI == 0x02)
    {
        ANX7150_Set_AVMute();//wen
        ANX7150_Set_System_State(ANX7150_CONFIG_AUDIO);
    }
    else
    {
        //To-Do: Config to DVI mode.
        ANX7150_Set_System_State(ANX7150_HDCP_AUTHENTICATION);
    }
    //reset TMDS link to align 4 channels  xy 061120
    D("reset TMDS link to align 4 channels\n");
    ANX7150_i2c_read_p0_reg(ANX7150_SRST_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SRST_REG, (c | ANX7150_TX_RST));
    ANX7150_i2c_write_p0_reg(ANX7150_SRST_REG, (c & (~ANX7150_TX_RST)));
    //Enable TMDS clock output // just enable uint87, and let the other uint8s along to avoid overwriting.
    D("Enable TMDS clock output\n");
    //ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);
    //ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, (c | ANX7150_TMDS_CLKCH_MUTE));

#ifdef TV_HDMI_HDCP
    DRVDelayMs(400);  //400ms only for HDCP CTS
#else
    ANX7150_Clear_AVMute();
    DRVDelayMs(10);
#endif

    //ANX7150_i2c_read_p0_reg(ANX7150_VID_MODE_REG, &c);  //zy 061110
}

void ANX7150_Parse_Video_Format(void)
{
    switch (ANX7150_video_format_config)
    {
        case ANX7150_RGB_YCrCb444_SepSync:
            ANX7150_emb_sync_mode = 0;
            ANX7150_demux_yc_en = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            //D("RGB_YCrCb444_SepSync mode!\n");
            break;
        case ANX7150_YCrCb422_SepSync:
            ANX7150_emb_sync_mode = 0;
            ANX7150_demux_yc_en = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            //D("YCrCb422_SepSync mode!\n");
            break;
        case ANX7150_YCrCb422_EmbSync:
            //D("YCrCb422_EmbSync mode!\n");
            ANX7150_demux_yc_en = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_emb_sync_mode = 1;
            ANX7150_Get_Video_Timing();
            break;
        case ANX7150_YCMux422_SepSync_Mode1:
            //D("YCMux422_SepSync_Mode1 mode!\n");
            ANX7150_emb_sync_mode = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_ycmux_uint8_sel = 0;
            ANX7150_demux_yc_en = 1;
            break;
        case ANX7150_YCMux422_SepSync_Mode2:
            //D("YCMux422_SepSync_Mode2 mode!\n");
            ANX7150_emb_sync_mode = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_ycmux_uint8_sel = 1;
            ANX7150_demux_yc_en = 1;
            break;
        case ANX7150_YCMux422_EmbSync_Mode1:
            //D("YCMux422_EmbSync_Mode1 mode!\n");
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_emb_sync_mode = 1;
            ANX7150_ycmux_uint8_sel = 0;
            ANX7150_demux_yc_en = 1;
            ANX7150_Get_Video_Timing();
            break;
        case ANX7150_YCMux422_EmbSync_Mode2:
            //D("YCMux422_EmbSync_Mode2 mode!\n");
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_emb_sync_mode = 1;
            ANX7150_ycmux_uint8_sel = 1;
            ANX7150_demux_yc_en = 1;
            ANX7150_Get_Video_Timing();
            break;
        case ANX7150_RGB_YCrCb444_DDR_SepSync:
            //D("RGB_YCrCb444_DDR_SepSync mode!\n");
            ANX7150_emb_sync_mode = 0;
            ANX7150_demux_yc_en = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_ddr_bus_mode = 1;
            break;
        case ANX7150_RGB_YCrCb444_DDR_EmbSync:
            //D("RGB_YCrCb444_DDR_EmbSync mode!\n");
            ANX7150_demux_yc_en = 0;
            ANX7150_de_gen_en = 0;
            ANX7150_emb_sync_mode = 1;
            ANX7150_ddr_bus_mode = 1;
            ANX7150_Get_Video_Timing();
            break;
        case ANX7150_RGB_YCrCb444_SepSync_No_DE:
            //D("RGB_YCrCb444_SepSync_No_DE mode!\n");
            ANX7150_emb_sync_mode = 0;
            ANX7150_demux_yc_en = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 1;
            ANX7150_Get_Video_Timing();
            break;
        case ANX7150_YCrCb422_SepSync_No_DE:
            //D("YCrCb422_SepSync_No_DE mode!\n");
            ANX7150_emb_sync_mode = 0;
            ANX7150_demux_yc_en = 0;
            ANX7150_ddr_bus_mode = 0;
            ANX7150_de_gen_en = 1;
            ANX7150_Get_Video_Timing();
            break;
        default:
            break;
    }
}

void ANX7150_Get_Video_Timing()
{
    uint8 i;
//#ifdef ITU656
    for (i = 0; i < 18; i++)
    {
        switch (ANX7150_video_timing_id)
        {
            case ANX7150_V640x480p_60Hz:
                //D("640x480p_60Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_640x480p_60Hz[i];
                break;
            case ANX7150_V720x480p_60Hz_4x3:
            case ANX7150_V720x480p_60Hz_16x9:
                //D("720x480p_60Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_720x480p_60Hz[i];
                break;
            case ANX7150_V1280x720p_60Hz:
                //D("1280x720p_60Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_1280x720p_60Hz[i];
                break;
            case ANX7150_V1920x1080i_60Hz:
                //D("1920x1080i_60Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_1920x1080i_60Hz[i];
                break;
            case ANX7150_V720x480i_60Hz_4x3:
            case ANX7150_V720x480i_60Hz_16x9:
                //D("720x480i_60Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_720x480i_60Hz[i];
                break;
            case ANX7150_V720x576p_50Hz_4x3:
            case ANX7150_V720x576p_50Hz_16x9:
                //D("720x576p_50Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_720x576p_50Hz[i];
                break;
            case ANX7150_V1280x720p_50Hz:
                //D("1280x720p_50Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_1280x720p_50Hz[i];
                break;
            case ANX7150_V1920x1080i_50Hz:
                //D("1920x1080i_50Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_1920x1080i_50Hz[i];
                break;
            case ANX7150_V720x576i_50Hz_4x3:
            case ANX7150_V720x576i_50Hz_16x9:
                //D("720x576i_50Hz!\n");
                ANX7150_video_timing_parameter[i] = ANX7150_video_timingtype_table.ANX7150_720x576i_50Hz[i];
                break;

            default:
                break;
        }
        //D("Video_Timing_Parameter[%.2x]=%.2x\n", (WORD)i, (WORD) ANX7150_video_timing_parameter[i]);
    }
    /*#else
        for(i = 0; i < 18; i++)
        {
            switch(ANX7150_video_timing_id)
            {
                case ANX7150_V640x480p_60Hz:
                    //D("640x480p_60Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V720x480p_60Hz_4x3:
                case ANX7150_V720x480p_60Hz_16x9:
                    //D("720x480p_60Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 18 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V1280x720p_60Hz:
                    //D("1280x720p_60Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 36 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V1920x1080i_60Hz:
                    //D("1920x1080i_60Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 54 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V720x480i_60Hz_4x3:
                case ANX7150_V720x480i_60Hz_16x9:
                    //D("720x480i_60Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 72 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V720x576p_50Hz_4x3:
                case ANX7150_V720x576p_50Hz_16x9:
                    //D("720x576p_50Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 90 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V1280x720p_50Hz:
                    //D("1280x720p_50Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 108 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V1920x1080i_50Hz:
                    //D("1920x1080i_50Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 126 + i);
                    DRVDelayMs(3);
                    break;
                case ANX7150_V720x576i_50Hz_4x3:
                case ANX7150_V720x576i_50Hz_16x9:
                    //D("720x576i_50Hz!\n");
                    ANX7150_video_timing_parameter[i] = Load_from_EEPROM(0, 144 + i);
                    DRVDelayMs(3);
                    break;

                default:
                    break;
            }
            //D("Video_Timing_Parameter[%.2x]=%.2x\n", (WORD)i, (WORD) ANX7150_video_timing_parameter[i]);
        }
    #endif*/
}

void ANX7150_DE_Generator()
{
    uint8 c,video_type,hsync_pol,vsync_pol,v_fp,v_bp,vsync_width;
    uint8 hsync_width_low,hsync_width_high,v_active_low,v_active_high;
    uint8 h_active_low,h_active_high,h_res_low,h_res_high,h_bp_low,h_bp_high;
    WORD hsync_width,h_active,h_res,h_bp;

    video_type = ANX7150_video_timing_parameter[15];
    hsync_pol = ANX7150_video_timing_parameter[16];
    vsync_pol = ANX7150_video_timing_parameter[17];
    v_fp = ANX7150_video_timing_parameter[12];
    v_bp = ANX7150_video_timing_parameter[11];
    vsync_width = ANX7150_video_timing_parameter[10];
    hsync_width = ANX7150_video_timing_parameter[5];
    hsync_width = (hsync_width << 8) + ANX7150_video_timing_parameter[4];
    v_active_high = ANX7150_video_timing_parameter[9];
    v_active_low = ANX7150_video_timing_parameter[8];
    h_active = ANX7150_video_timing_parameter[3];
    h_active = (h_active << 8) + ANX7150_video_timing_parameter[2];
    h_res = ANX7150_video_timing_parameter[1];
    h_res = (h_res << 8) + ANX7150_video_timing_parameter[0];
    h_bp = ANX7150_video_timing_parameter[7];
    h_bp = (h_bp << 8) + ANX7150_video_timing_parameter[6];
    if (ANX7150_demux_yc_en)
    {
        hsync_width = 2* hsync_width;
        h_active = 2 * h_active;
        h_res = 2 * h_res;
        h_bp = 2 * h_bp;
    }
    hsync_width_low = hsync_width & 0xff;
    hsync_width_high = (hsync_width >> 8) & 0xff;
    h_active_low = h_active & 0xff;
    h_active_high = (h_active >> 8) & 0xff;
    h_res_low = h_res & 0xff;
    h_res_high = (h_res >> 8) & 0xff;
    h_bp_low = h_bp & 0xff;
    h_bp_high = (h_bp >> 8) & 0xff;

    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL1_REG, (c & 0xf7) | video_type);//set video type
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL1_REG, (c & 0xdf) | hsync_pol);//set HSYNC polarity
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL1_REG, (c & 0xbf) | vsync_pol);//set VSYNC polarity
    ANX7150_i2c_write_p0_reg(ANX7150_ACT_LINEL_REG, v_active_low);
    ANX7150_i2c_write_p0_reg(ANX7150_ACT_LINEH_REG, v_active_high);
    ANX7150_i2c_write_p0_reg(ANX7150_VSYNC_WID_REG, vsync_width);
    ANX7150_i2c_write_p0_reg(ANX7150_VSYNC_TAIL2VIDLINE_REG, v_bp);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_PIXL_REG, h_active_low);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_PIXH_REG, h_active_high);
    ANX7150_i2c_write_p0_reg(ANX7150_H_RESL_REG, h_res_low);
    ANX7150_i2c_write_p0_reg(ANX7150_H_RESH_REG, h_res_high);
    ANX7150_i2c_write_p0_reg(ANX7150_HSYNC_ACT_WIDTHL_REG, hsync_width_low);
    ANX7150_i2c_write_p0_reg(ANX7150_HSYNC_ACT_WIDTHH_REG, hsync_width_high);
    ANX7150_i2c_write_p0_reg(ANX7150_H_BACKPORCHL_REG, h_bp_low);
    ANX7150_i2c_write_p0_reg(ANX7150_H_BACKPORCHH_REG, h_bp_high);
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c | ANX7150_VID_CAPCTRL0_DEGEN_EN);
}

void ANX7150_Embed_Sync_Decode()
{
    uint8 c,video_type,hsync_pol,vsync_pol,v_fp,vsync_width;
    uint8 h_fp_low,h_fp_high,hsync_width_low,hsync_width_high;
    WORD h_fp,hsync_width;

    video_type = ANX7150_video_timing_parameter[15];
    hsync_pol = ANX7150_video_timing_parameter[16];
    vsync_pol = ANX7150_video_timing_parameter[17];
    v_fp = ANX7150_video_timing_parameter[12];
    vsync_width = ANX7150_video_timing_parameter[10];
    h_fp = ANX7150_video_timing_parameter[14];
    h_fp = (h_fp << 8) + ANX7150_video_timing_parameter[13];
    hsync_width = ANX7150_video_timing_parameter[5];
    hsync_width = (hsync_width << 8) + ANX7150_video_timing_parameter[4];
    if (ANX7150_demux_yc_en)
    {
        h_fp = 2 * h_fp;
        hsync_width = 2* hsync_width;
    }
    h_fp_low = h_fp & 0xff;
    h_fp_high = (h_fp >> 8) & 0xff;
    hsync_width_low = hsync_width & 0xff;
    hsync_width_high = (hsync_width >> 8) & 0xff;

    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL1_REG, (c & 0xf7) | video_type);//set video type
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL1_REG, (c & 0xdf) | hsync_pol);//set HSYNC polarity
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL1_REG, (c & 0xbf) | vsync_pol);//set VSYNC polarity
    ANX7150_i2c_read_p0_reg(ANX7150_VID_CAPCTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_VID_CAPCTRL0_REG, c | ANX7150_VID_CAPCTRL0_EMSYNC_EN);
    ANX7150_i2c_write_p0_reg(ANX7150_ACT_LINE2VSYNC_REG, v_fp);
    ANX7150_i2c_write_p0_reg(ANX7150_VSYNC_WID_REG, vsync_width);
    ANX7150_i2c_write_p0_reg(ANX7150_H_FRONTPORCHL_REG, h_fp_low);
    ANX7150_i2c_write_p0_reg(ANX7150_H_FRONTPORCHH_REG, h_fp_high);
    ANX7150_i2c_write_p0_reg(ANX7150_HSYNC_ACT_WIDTHL_REG, hsync_width_low);
    ANX7150_i2c_write_p0_reg(ANX7150_HSYNC_ACT_WIDTHH_REG, hsync_width_high);
}

void ANX7150_Show_Video_Parameter()
{
    // int h_res,h_act,v_res,v_act,h_fp,hsync_width,h_bp;
    uint8 c, c1;

    D1("\n\n**********************************ANX7150 Info**********************************\n");

    D("   ANX7150 mode = Normal mode\n");
    if ((ANX7150_demux_yc_en == 1) && (ANX7150_emb_sync_mode == 0))
        D("   Input video format = YC_MUX\n");
    if ((ANX7150_demux_yc_en == 0) && (ANX7150_emb_sync_mode == 1))
        D("   Input video format = 656\n");
    if ((ANX7150_demux_yc_en == 1) && (ANX7150_emb_sync_mode == 1))
        D("   Input video format = YC_MUX + 656\n");
    if ((ANX7150_demux_yc_en == 0) && (ANX7150_emb_sync_mode == 0))
        D("   Input video format = Seperate Sync\n");
    if (ANX7150_de_gen_en)
        D("   DE generator = Enable\n");
    else
        D("   DE generator = Disable\n");
    if ((ANX7150_ddr_bus_mode == 1)&& (ANX7150_emb_sync_mode == 0))
        D("   Input video format = DDR mode\n");
    else if ((ANX7150_ddr_bus_mode == 1)&& (ANX7150_emb_sync_mode == 1))
        D("   Input video format = DDR mode + 656\n");
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c1);
    c1 = (c1 & 0x02);
    if (c1)
    {
        D("   Output video mode = HDMI\n");
        ANX7150_i2c_read_p1_reg(0x04, &c);
        c = (c & 0x60) >> 5;
        switch (c)
        {
            case ANX7150_RGB:
                D("   Output video color format = RGB\n");
                break;
            case ANX7150_YCbCr422:
                D("   Output video color format = YCbCr422\n");
                break;
            case ANX7150_YCbCr444:
                D("   Output video color format = YCbCr444\n");
                break;
            default:
                break;
        }
    }
    else
    {
        D("   Output video mode = DVI\n");
        D("   Output video color format = RGB\n");
    }

    /*for(i = 0x10; i < 0x25; i ++)
    {
        ANX7150_i2c_read_p0_reg(i, &c );
        D("0x%.2x = 0x%.2x\n",(unsigned int)i,(unsigned int)c);
    }*/
    /*   ANX7150_i2c_read_p0_reg(ANX7150_VID_STATUS_REG, &c);
       if((c & ANX7150_VID_STATUS_TYPE) == 0x04)
           D("Video Type = Interlace");
       else
           D("Video Type = Progressive");
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HRESH_REG, &c);
       h_res = c;
       h_res = h_res << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HRESL_REG, &c);
       h_res = h_res + c;
       D("H_resolution = %u\n",h_res);
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_PIXH_REG, &c);
       h_act = c;
       h_act = h_act << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_PIXL_REG, &c);
       h_act = h_act + c;
       D("H_active = %u\n",h_act);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_VRESH_REG, &c);
       v_res = c;
       v_res = v_res << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_VRESL_REG, &c);
       v_res = v_res + c;
       D("V_resolution = %u\n",v_res);
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_ACTVIDLINEH_REG, &c);
       v_act = c;
       v_act = v_act << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_ACTVIDLINEL_REG, &c);
       v_act = v_act + c;
       D("V_active = %u\n",v_act);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HFORNTPORCHH_REG, &c);
       h_fp = c;
       h_fp = h_fp << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HFORNTPORCHL_REG, &c);
       h_fp = h_fp + c;
       D("H_FP = %u\n",h_fp);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HBACKPORCHH_REG, &c);
       h_bp = c;
       h_bp = h_bp << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HBACKPORCHL_REG, &c);
       h_bp = h_bp + c;
       D("H_BP = %u\n",h_bp);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HSYNCWIDH_REG, &c);
       hsync_width = c;
       hsync_width = hsync_width << 8;
       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_HSYNCWIDL_REG, &c);
       hsync_width = hsync_width + c;
       D("Hsync_width = %u\n",hsync_width);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_ACTLINE2VSYNC_REG, &c);
       D("Vsync_FP = %bu\n",c);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_VSYNCTAIL2VIDLINE_REG, &c);
       D("Vsync_BP = %bu\n",c);

       ANX7150_i2c_read_p0_reg(ANX7150_VIDF_VSYNCWIDLINE_REG, &c);
       D("Vsync_width = %bu\n",c);*/
    D("\n");
    {
        D("   Normal mode output video format: ");
        switch (ANX7150_video_timing_id)
        {
            case ANX7150_V720x480p_60Hz_4x3:
            case ANX7150_V720x480p_60Hz_16x9:
                D1("720x480p@60, ");
                if (ANX7150_edid_result.supported_720x480p_60Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V1280x720p_60Hz:
                D1("1280x720p@60, ");
                if (ANX7150_edid_result.supported_720p_60Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V1920x1080i_60Hz:
                D1("1920x1080i@60, ");
                if (ANX7150_edid_result.supported_1080i_60Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V1920x1080p_60Hz:
                D1("1920x1080p@60, ");
                if (ANX7150_edid_result.supported_1080p_60Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V1920x1080p_50Hz:
                D1("1920x1080p@50, ");
                if (ANX7150_edid_result.supported_1080p_50Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V1280x720p_50Hz:
                D1("1280x720p@50, ");
                if (ANX7150_edid_result.supported_720p_50Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V1920x1080i_50Hz:
                D1("1920x1080i@50, ");
                if (ANX7150_edid_result.supported_1080i_50Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V720x576p_50Hz_4x3:
            case ANX7150_V720x576p_50Hz_16x9:
                D1("720x576p@50, ");
                if (ANX7150_edid_result.supported_576p_50Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V720x576i_50Hz_4x3:
            case ANX7150_V720x576i_50Hz_16x9:
                D1("720x576i@50, ");
                if (ANX7150_edid_result.supported_576i_50Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            case ANX7150_V720x480i_60Hz_4x3:
            case ANX7150_V720x480i_60Hz_16x9:
                D1("720x480i@60, ");
                if (ANX7150_edid_result.supported_720x480i_60Hz)
                    D1("and sink supports this format.\n");
                else
                    D1("but sink does not support this format.\n");
                break;
            default:
                D1("unknown(video ID is: %.2x).\n",(WORD)ANX7150_video_timing_id);
                break;
        }
    }
    if (c1)//HDMI output
    {
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
        c = c & 0x03;
        D("   MCLK Frequence = ");

        switch (c)
        {
            case 0x00:
                D1("128 * Fs.\n");
                break;
            case 0x01:
                D1("256 * Fs.\n");
                break;
            case 0x02:
                D1("384 * Fs.\n");
                break;
            case 0x03:
                D1("512 * Fs.\n");
                break;
            default :
                D1("Wrong MCLK output.\n");
                break;
        }

        if ( ANX7150_AUD_HW_INTERFACE == 0x01)
        {
            D("   Input Audio Interface = I2S.\n");
            ANX7150_i2c_read_p0_reg(ANX7150_I2SCH_STATUS4_REG, &c);
        }
        else if (ANX7150_AUD_HW_INTERFACE == 0x02)
        {
            D("   Input Audio Interface = SPDIF.\n");
            ANX7150_i2c_read_p0_reg(ANX7150_SPDIFCH_STATUS_REG, &c);
            c=c>>4;
        }
        ANX7150_i2c_read_p0_reg(ANX7150_I2SCH_STATUS4_REG, &c);
        D("   Audio Fs = ");
        c &= 0x0f;
        switch (c)
        {
            case 0x00:
                D1("44.1 KHz.\n");
                break;
            case 0x02:
                D1("48 KHz.\n");
                break;
            case 0x03:
                D1("32 KHz.\n");
                break;
            case 0x08:
                D1("88.2 KHz.\n");
                break;
            case 0x0a:
                D1("96 KHz.\n");
                break;
            case 0x0c:
                D1("176.4 KHz.\n");
                break;
            case 0x0e:
                D1("192 KHz.\n");
                break;
            default :
                D1("Wrong Fs output.\n");
                break;
        }

        if	(ANX7150_HDCP_enable == 1)
            D("   ANX7150_HDCP_Enable.\n");
        else
            D("   ANX7150_HDCP_Disable.\n");

    }
    D1("********************************************************************************\n\n");
}

//********************************************end video config*******************************

uint8 ANX7150_Config_Audio()
{
    uint8 exe_result = 0x00;
    uint8 c = 0x00;
    uint8 audio_layout = 0x00;
    uint8 fs = 0x00;
    WORD ACR_N = 0x0000;


    //set audio clock edge
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG,  (c & 0xf7) | ANX7150_audio_clock_edge);
    //cts get select from SCK
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG,  (c & 0xef));//cts get select from mclk
    D("audio_type = 0x%.2x\n",(WORD)s_ANX7150_audio_config.audio_type);
    if (s_ANX7150_audio_config.audio_type & ANX7150_i2s_input)
    {
    	D("Config I2s.\n");
        exe_result |= ANX7150_Config_I2s();
    }
    else
    {
        //disable I2S audio input
        D("Disable I2S audio input.\n");
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        c &= 0xc3;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);
    }

    if (s_ANX7150_audio_config.audio_type & ANX7150_spdif_input)
    {
        exe_result |= ANX7150_Config_Spdif();
    }
    else
    {
        //disable SPDIF audio input
        D("Disable SPDIF audio input.\n");
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        c &= ~ANX7150_HDMI_AUDCTRL1_SPDIFIN_EN;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);

    }

    if (s_ANX7150_audio_config.audio_type & ANX7150_super_audio_input)
    {
        exe_result |= ANX7150_Config_Super_Audio();
    }
    else
    {
        //disable super audio output
        //D("ANX7150: disable super audio output.\n");
        ANX7150_i2c_write_p0_reg(ANX7150_ONEuint8_AUD_CTRL_REG, 0x00);
    }

    if ((s_ANX7150_audio_config.audio_type & 0x07) == 0x00)
    {
        ;//D("ANX7150 input no audio type.\n");
    }

    //audio layout
    if (s_ANX7150_audio_config.audio_type & ANX7150_i2s_input)
    {
        //ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        audio_layout = s_ANX7150_audio_config.audio_layout;

        //HDMI_RX_ReadI2C_RX0(0x15, &c);
#if 0
        if ((c & 0x08) ==0x08 )   //uint8[5:3]
        {
            audio_layout = 0x80;
        }
        else
        {
            audio_layout = 0x00;
        }
#endif
    }
    if (s_ANX7150_audio_config.audio_type & ANX7150_super_audio_input)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_ONEuint8_AUD_CTRL_REG, &c);
        if ( c & 0xfc)      //uint8[5:3]
        {
            audio_layout = 0x80;
        }
    }

    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
    c &= ~0x80;
    c |= audio_layout;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, c);

    if (  (s_ANX7150_audio_config.audio_type & 0x07) == exe_result )
    {
        //Initial N value

        ANX7150_i2c_read_p0_reg(ANX7150_I2SCH_STATUS4_REG, &c);
        fs = c & 0x0f;
        // set default value to N
        ACR_N = ANX7150_N_48k;
        switch (fs)
        {
            case(0x00)://44.1k
                ACR_N = ANX7150_N_44k;
                break;
            case(0x02)://48k
                ACR_N = ANX7150_N_48k;
                break;
            case(0x03)://32k
                ACR_N = ANX7150_N_32k;
                break;
            case(0x08)://88k
                ACR_N = ANX7150_N_88k;
                break;
            case(0x0a)://96k
                ACR_N = ANX7150_N_96k;
                break;
            case(0x0c)://176k
                ACR_N = ANX7150_N_176k;
                break;
            case(0x0e)://192k
                ACR_N = ANX7150_N_192k;
                break;
            default:
                //D("note wrong fs.\n");
                break;
        }
        // write N(ACR) to corresponding regs
        c = ACR_N;
        ANX7150_i2c_write_p1_reg(ANX7150_ACR_N1_SW_REG, c);
        c = ACR_N>>8;
        ANX7150_i2c_write_p1_reg(ANX7150_ACR_N2_SW_REG, c);

        ANX7150_i2c_write_p1_reg(ANX7150_ACR_N3_SW_REG, 0x00);

        // set the relation of MCLK and Fs  xy 070117
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, (c & 0xf8) | FREQ_MCLK);
        D("Audio MCLK input mode is: %.2x\n",(WORD)FREQ_MCLK);

        //Enable control of ACR
        ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
        ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, (c | ANX7150_INFO_PKTCTRL1_ACR_EN));

        //audio enable:
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        c |= ANX7150_HDMI_AUDCTRL1_IN_EN;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);

        ANX7150_Set_System_State(ANX7150_CONFIG_PACKETS);
    }

    return exe_result;

}

uint8 ANX7150_Config_I2s()
{
    uint8 exe_result = 0x00;
    uint8 c = 0x00;
    uint8 c1 = 0x00;

    D("ANX7150: config i2s audio.\n");

    //select SCK as source
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
    c &=  ~ANX7150_HDMI_AUDCTRL1_CLK_SEL;
    D("select SCK as source, c = 0x%.2x\n",(WORD)c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);


    //config i2s channel
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
    c1 = s_ANX7150_audio_config.i2s_config.audio_channel;    // need uint8[5:2]
    c1 &= 0x3c;
    c &= ~0x3c;
    c |= c1;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);
    D("config i2s channel, c = 0x%.2x\n",(WORD)c);

    //config i2s format
    //ANX7150_i2c_read_p0_reg(ANX7150_I2S_CTRL_REG, &c);
    c = s_ANX7150_audio_config.i2s_config.i2s_format;
    ANX7150_i2c_write_p0_reg(ANX7150_I2S_CTRL_REG, c);
    D("config i2s format, c = 0x%.2x\n",(WORD)c);

    //map i2s fifo

    //TODO: config I2S channel map register according to system


    //ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_CTRL_REG, c);

    //swap right/left channel
    /*ANX7150_i2c_read_p0_reg(ANX7150_I2SCH_SWCTRL_REG, &c);
    c1 = 0x00;
    c1 &= 0xf0;
    c &= ~0xf0;
    c |= c1;
    ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_SWCTRL_REG, c);
    D("map i2s ffio, c = 0x%.2x\n",(WORD)c);*/

    //down sample
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
    c1 = s_ANX7150_audio_config.down_sample;
    c1 &= 0x60;
    c &= ~0x60;
    c |= c1;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, c);
    D("down sample, c = 0x%.2x\n",(WORD)c);

    //config i2s channel status(5 regs)
    c = s_ANX7150_audio_config.i2s_config.Channel_status1;
    ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS1_REG, c);
    c = s_ANX7150_audio_config.i2s_config.Channel_status2;
    ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS2_REG, c);
    c = s_ANX7150_audio_config.i2s_config.Channel_status3;
    ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS3_REG, c);
    c = s_ANX7150_audio_config.i2s_config.Channel_status4;
    ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS4_REG, c);
    D("@@@@@@@@config i2s channel status4, c = 0x%.2x\n",(unsigned int)c);//jack wen

    c = s_ANX7150_audio_config.i2s_config.Channel_status5;
    ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS5_REG, c);
    D("config i2s channel status, c = 0x%.2x\n",(WORD)c);

    exe_result = ANX7150_i2s_input;
    //D("return = 0x%.2x\n",(WORD)exe_result);

    // open corresponding interrupt
    //ANX7150_i2c_read_p0_reg(ANX7150_INTR1_MASK_REG, &c);
    //ANX7150_i2c_write_p0_reg(ANX7150_INTR1_MASK_REG, (c | 0x22) );
    //ANX7150_i2c_read_p0_reg(ANX7150_INTR3_MASK_REG, &c);
    //ANX7150_i2c_write_p0_reg(ANX7150_INTR3_MASK_REG, (c | 0x20) );


    return exe_result;
}



uint8 ANX7150_Config_Spdif()
{
    uint8 exe_result = 0x00;
    uint8 c = 0x00;
    uint8 c1 = 0x00;
 //   uint8 c2 = 0x00;
 //   uint8 freq_mclk = 0x00;

    D("ANX7150: config SPDIF audio.\n");


    //Select MCLK
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
    c |= ANX7150_HDMI_AUDCTRL1_CLK_SEL;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);

    //D("ANX7150: enable SPDIF audio.\n");
    //Enable SPDIF
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
    c |= ANX7150_HDMI_AUDCTRL1_SPDIFIN_EN;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);

    //adjust MCLK phase in interrupt routine

    // adjust FS_FREQ   //FS_FREQ
    c1 = s_ANX7150_audio_config.i2s_config.Channel_status4 & 0x0f;
    ANX7150_i2c_read_p0_reg(ANX7150_SPDIFCH_STATUS_REG, &c);
    c &= ANX7150_SPDIFCH_STATUS_FS_FREG;
    c = c >> 4;

    if ( c != c1)
    {
        //D("adjust FS_FREQ by system!\n");
        ANX7150_i2c_read_p0_reg(ANX7150_I2SCH_STATUS4_REG, &c);
        c &= 0xf0;
        c |= c1;
        ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS4_REG, c);

        //enable using FS_FREQ from 0x59
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        c |= 0x02;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);
    }

    // down sample
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, &c);
    c1 = s_ANX7150_audio_config.down_sample;
    c1 &= 0x60;
    c &= ~0x60;
    c |= c1;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL0_REG, c);

    if (s_ANX7150_audio_config.down_sample)     //zy 060816
    {
        // adjust FS_FREQ by system because down sample
        //D("adjust FS_FREQ by system because down sample!\n");

        c1 = s_ANX7150_audio_config.i2s_config.Channel_status4 & 0x0f;
        ANX7150_i2c_read_p0_reg(ANX7150_I2SCH_STATUS4_REG, &c);
        c &= 0xf0;
        c |= c1;
        ANX7150_i2c_write_p0_reg(ANX7150_I2SCH_STATUS4_REG, c);
    }


    // spdif is stable
    D("config SPDIF audio done");
    exe_result = ANX7150_spdif_input;

    // open corresponding interrupt
    ANX7150_i2c_read_p0_reg(ANX7150_INTR1_MASK_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_INTR1_MASK_REG, (c | 0x32) );
    //ANX7150_i2c_read_p0_reg(ANX7150_INTR3_MASK_REG, &c);
    //ANX7150_i2c_write_p0_reg(ANX7150_INTR3_MASK_REG, (c | 0xa1) );
    return exe_result;
}

uint8 ANX7150_Config_Super_Audio()
{
    uint8 exe_result = 0x00;
    uint8 c = 0x00;


    //D("ANX7150: config one uint8 audio.\n");

    // select sck as source
    ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
    c &=  ~ANX7150_HDMI_AUDCTRL1_CLK_SEL;
    ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);

    // Enable stream  0x60
    c = s_ANX7150_audio_config.super_audio_config.one_uint8_ctrl;
    ANX7150_i2c_write_p0_reg(ANX7150_ONEuint8_AUD_CTRL_REG, c);


    // Map stream 0x61
    // TODO: config super audio  map register according to system

    exe_result = ANX7150_super_audio_input;
    return exe_result;

}

//*************** Config Packet ****************************
uint8 ANX7150_Config_Packet()
{
    uint8 exe_result = 0x00;     // There is no use in current solution
    uint8 info_packet_sel;
    uint8 c;


    info_packet_sel = s_ANX7150_packet_config.packets_need_config;
    D("info_packet_sel = 0x%.2x\n",(WORD) info_packet_sel);
    // New packet?
    if ( info_packet_sel != 0x00)
    {
        // avi infoframe
        if ( info_packet_sel & ANX7150_avi_sel )
        {
            c = s_ANX7150_packet_config.avi_info.pb_uint8[1];  //color space
            c &= 0x9f;
            c |= (ANX7150_RGBorYCbCr << 5);
            s_ANX7150_packet_config.avi_info.pb_uint8[1] = c | 0x10;

		    switch(ANX7150_video_timing_id)
	     	{
		     	case ANX7150_V720x576p_50Hz_4x3:
					s_ANX7150_packet_config.avi_info.pb_uint8[2] = 0x58;
				break;
				case ANX7150_V1280x720p_50Hz:
				default:
					s_ANX7150_packet_config.avi_info.pb_uint8[2] = 0xa8;
				break;

	     	}

            c = s_ANX7150_packet_config.avi_info.pb_uint8[4];// vid ID
            c = c & 0x80;
            s_ANX7150_packet_config.avi_info.pb_uint8[4] = c | ANX7150_video_timing_id;
            c = s_ANX7150_packet_config.avi_info.pb_uint8[5]; //repeat times
            c = c & 0xf0;
            c |= (ANX7150_tx_pix_rpt & 0x0f);
            s_ANX7150_packet_config.avi_info.pb_uint8[5] = c;
            D("config avi infoframe packet.\n");
            // Disable repeater
            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);

            c &= ~ANX7150_INFO_PKTCTRL1_AVI_RPT;
            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, c);

            // Enable?wait:go
            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
            if (c & ANX7150_INFO_PKTCTRL1_AVI_EN)
            {
                //D("wait disable, config avi infoframe packet.\n");
                return exe_result; //jack wen
            }

            // load packet data to regs
            ANX7150_Load_Infoframe( ANX7150_avi_infoframe,
                                    &(s_ANX7150_packet_config.avi_info));
            // Enable and repeater
            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
            c |= 0x30;
            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, c);

            // complete avi packet
            D("config avi infoframe packet done.\n");
            s_ANX7150_packet_config.packets_need_config &= ~ANX7150_avi_sel;

        }

        // audio infoframe
        if ( info_packet_sel & ANX7150_audio_sel )
        {
            D("config audio infoframe packet.\n");

            // Disable repeater
            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
            c &= ~ANX7150_INFO_PKTCTRL2_AIF_RPT;
            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);

            // Enable?wait:go
            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
            if (c & ANX7150_INFO_PKTCTRL2_AIF_EN)
            {
                //D("wait disable, config audio infoframe packet.\n");
                //return exe_result;//jack wen
            }
            // config packet

            // load packet data to regs
            ANX7150_Load_Infoframe( ANX7150_audio_infoframe,
                                    &(s_ANX7150_packet_config.audio_info));
            // Enable and repeater
            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
            c |= 0x03;
            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);

            // complete avi packet

            D("config audio infoframe packet done.\n");
            s_ANX7150_packet_config.packets_need_config &= ~ANX7150_audio_sel;

        }

        // config other 4 packets
        /*

                if( info_packet_sel & 0xfc )
                {
                    D("other packets.\n");

                    //find the current type need config
                    if(info_packet_sel & ANX7150_spd_sel)    type_sel = ANX7150_spd_sel;
                    else if(info_packet_sel & ANX7150_mpeg_sel)    type_sel = ANX7150_mpeg_sel;
                    else if(info_packet_sel & ANX7150_acp_sel)    type_sel = ANX7150_acp_sel;
                    else if(info_packet_sel & ANX7150_isrc1_sel)    type_sel = ANX7150_isrc1_sel;
                    else if(info_packet_sel & ANX7150_isrc2_sel)    type_sel = ANX7150_isrc2_sel;
                    else  type_sel = ANX7150_vendor_sel;


                    // Disable repeater
                    ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                    c &= ~ANX7150_INFO_PKTCTRL2_AIF_RPT;
                    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);

                    switch(type_sel)
                    {
                        case ANX7150_spd_sel:
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
                            c &= ~ANX7150_INFO_PKTCTRL1_SPD_RPT;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, c);

                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
                            if(c & ANX7150_INFO_PKTCTRL1_SPD_EN)
                            {
                                D("wait disable, config spd infoframe packet.\n");
                                return exe_result;
                            }
                            break;

                        case ANX7150_mpeg_sel:
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c &= ~ANX7150_INFO_PKTCTRL2_MPEG_RPT;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);

                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            if(c & ANX7150_INFO_PKTCTRL2_MPEG_EN)
                            {
                                D("wait disable, config mpeg infoframe packet.\n");
                                return exe_result;
                            }
                            break;

                        case ANX7150_acp_sel:
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c &= ~ANX7150_INFO_PKTCTRL2_UD0_RPT;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);

                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            if(c & ANX7150_INFO_PKTCTRL2_UD0_EN)
                            {
                                D("wait disable, config mpeg infoframe packet.\n");
                                return exe_result;
                            }
                            break;

                        case ANX7150_isrc1_sel:
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c &= ~ANX7150_INFO_PKTCTRL2_UD0_RPT;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            if(c & ANX7150_INFO_PKTCTRL2_UD0_EN)
                            {
                                D("wait disable, config isrc1 packet.\n");
                                return exe_result;
                            }
                            break;

                        case ANX7150_isrc2_sel:
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c &= ~ANX7150_INFO_PKTCTRL2_UD1_RPT;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            if(c & ANX7150_INFO_PKTCTRL2_UD1_EN)
                            {
                                D("wait disable, config isrc2 packet.\n");
                                return exe_result;
                            }
                            break;

                        case ANX7150_vendor_sel:
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c &= ~ANX7150_INFO_PKTCTRL2_UD1_RPT;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            if(c & ANX7150_INFO_PKTCTRL2_UD1_EN)
                            {
                                D("wait disable, config vendor packet.\n");
                                return exe_result;
                            }
                            break;

                        default : break;
                    }


                    // config packet
                    // TODO: config packet in top level

                    // load packet data to regs
                    switch(type_sel)
                    {
                        case ANX7150_spd_sel:
                            ANX7150_Load_Infoframe( ANX7150_spd_infoframe,
                                                    &(s_ANX7150_packet_config.spd_info));
                            D("config spd done.\n");
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL1_REG, &c);
                            c |= 0xc0;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL1_REG, c);
                            break;

                        case ANX7150_mpeg_sel:
                            ANX7150_Load_Infoframe( ANX7150_mpeg_infoframe,
                                                    &(s_ANX7150_packet_config.mpeg_info));
                            D("config mpeg done.\n");
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c |= 0x0c;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            break;

                        case ANX7150_acp_sel:
                            ANX7150_Load_Packet( ANX7150_acp_packet,
                                                    &(s_ANX7150_packet_config.acp_pkt));
                            D("config acp done.\n");
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c |= 0x30;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            break;

                        case ANX7150_isrc1_sel:
                            ANX7150_Load_Packet( ANX7150_isrc1_packet,
                                                    &(s_ANX7150_packet_config.acp_pkt));
                            D("config isrc1 done.\n");
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c |= 0x30;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            break;

                        case ANX7150_isrc2_sel:
                            ANX7150_Load_Packet( ANX7150_isrc2_packet,
                                                    &(s_ANX7150_packet_config.acp_pkt));
                            D("config isrc2 done.\n");
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c |= 0xc0;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            break;

                        case ANX7150_vendor_sel:
                            ANX7150_Load_Infoframe( ANX7150_vendor_infoframe,
                                                    &(s_ANX7150_packet_config.vendor_info));
                            D("config vendor done.\n");
                            ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                            c |= 0xc0;
                            ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);
                            break;

                        default : break;
                    }

                    // Enable and repeater
                    ANX7150_i2c_read_p1_reg(ANX7150_INFO_PKTCTRL2_REG, &c);
                    c |= 0x03;
                    ANX7150_i2c_write_p1_reg(ANX7150_INFO_PKTCTRL2_REG, c);

                    // complete config packet
                    D("config other packets done.\n");
                    s_ANX7150_packet_config.packets_need_config &= ~type_sel;

                }
                */
    }


    if ( s_ANX7150_packet_config.packets_need_config  == 0x00)
    {
        D("config packets done\n");
        ANX7150_Set_System_State(ANX7150_HDCP_AUTHENTICATION);
    }


    return exe_result;
}



uint8 ANX7150_Load_Infoframe(packet_type member,
                             infoframe_struct *p)
{
    uint8 exe_result = 0x00;
    uint8 address[8] = {0x00,0x20,0x40,0x60,0x80,0x80,0xa0,0xa0};
    uint8 i;
    uint8 c;

    p->pb_uint8[0] = ANX7150_Checksum(p);

    // write infoframe to according regs
    ANX7150_i2c_write_p1_reg(address[member], p->type);
    ANX7150_i2c_write_p1_reg(address[member]+1, p->version);
    ANX7150_i2c_write_p1_reg(address[member]+2, p->length);

    for (i=0; i <= p->length; i++)
    {
        ANX7150_i2c_write_p1_reg(address[member]+3+i, p->pb_uint8[i]);
        ANX7150_i2c_read_p1_reg(address[member]+3+i, &c);
    }
    return exe_result;
}

uint8 ANX7150_Checksum(infoframe_struct *p)
{
    uint8 checksum = 0x00;
    uint8 i;

    checksum = p->type + p->length + p->version;
    for (i=1; i <= p->length; i++)
    {
        checksum += p->pb_uint8[i];
    }
    checksum = ~checksum;
    checksum += 0x01;

    return checksum;
}

/*
uint8 ANX7150_Load_Packet(packet_type member,
                            infoframe_struct *p)
{
    uint8 exe_result = 0x00;
    uint8 address[8] = {0x00,0x20,0x40,0x60,0x80,0x80,0xa0,0xa0};
    uint8 i;

    D("address  = 0x%.2x\n",(WORD) address[member]);

    // write packet to according regs
    ANX7150_i2c_write_p1_reg(address[member], p->type);

    ANX7150_i2c_write_p1_reg(address[member]+1, p->version);

    ANX7150_i2c_write_p1_reg(address[member]+2, p->length);

    for(i=0; i < 28; i++)
    {
      ANX7150_i2c_write_p1_reg(address[member]+3+i, p->pb_uint8[i]);
    }
    return exe_result;
}
*/
//***************  end of Config Packet ****************************

//******************** HDCP process ********************************

void ANX7150_HDCP_Process(void)
{
    uint8 c,i;
	//uint8 c1;
    uint8 Bksv_valid=0;//wen HDCP CTS

    if (ANX7150_HDCP_enable)
    { //HDCP_EN =1 means to do HDCP authentication,SWITCH4 = 0 means not to do HDCP authentication.

        //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        //ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c | 0x04);//power on HDCP, 090630

        //ANX7150_i2c_read_p0_reg(ANX7150_INTR2_MASK_REG, &c);
        //ANX7150_i2c_write_p0_reg(ANX7150_INTR2_MASK_REG, c |0x03);
        DRVDelayMs(10);//let unencrypted video play a while, required by HDCP CTS. SY//wen HDCP CTS
        ANX7150_Set_AVMute();//before auth, set_avmute//wen
        DRVDelayMs(10);//wen HDCP CTS

        if ( !ANX7150_hdcp_init_done )
        {
            ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c); //72:07.2 hdcp on//wen HDCP CTS
            ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, (c | ANX7150_SYS_CTRL1_HDCPMODE));//wen HDCP CTS
            if (ANX7150_edid_result.is_HDMI)
                ANX7150_Hardware_HDCP_Auth_Init();
            else
            {   //DVI, disable 1.1 feature and enable HDCP two special point check
                ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL1_REG, &c);
                ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL1_REG,
                                         ((c & (~ANX7150_HDCP_CTRL1_HDCP11_EN)) | ANX7150_LINK_CHK_12_EN));
            }

            //wen HDCP CTS
            if (!ANX7150_BKSV_SRM())
            {
                ANX7150_Blue_Screen_Enable();
                ANX7150_Clear_AVMute();
                Bksv_valid=0;
                return;
            }
            else //SY.
            {
                Bksv_valid=1;
                ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
                ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c | 0x03));
            }

            ANX7150_hdcp_init_done = 1;
//wen HDCP CTS
        }


//wen HDCP CTS
        if ((Bksv_valid) && (!ANX7150_hdcp_auth_en))
        {
            D("enable hw hdcp\n");
            ANX7150_RST_DDCChannel();
            ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c | ANX7150_HDCP_CTRL0_HW_AUTHEN));
            ANX7150_hdcp_auth_en = 1;
        }

        if ((Bksv_valid) && (ANX7150_hdcp_wait_100ms_needed))
        {
            ANX7150_hdcp_wait_100ms_needed = 0;
            //disable audio
            ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));
            D("++++++++ANX7150_hdcp_wait_100ms_needed+++++++++\n");
            DRVDelayMs(150);    //  100 -> 150
            return;
        }
//wen HDCP CTS

        if (ANX7150_hdcp_auth_pass) 			//wen HDCP CTS
        {
            //Clear the SRM_Check_Pass uint8, then when reauthentication occurs, firmware can catch it.
            ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, c & 0xfc);

            //Enable HDCP Hardware encryption
            if (!ANX7150_hdcp_encryption)
            {
                ANX7150_HDCP_Encryption_Enable();
            }
            if (ANX7150_send_blue_screen)
            {
                ANX7150_Blue_Screen_Disable();
            }
            if (ANX7150_avmute_enable)
            {
                ANX7150_Clear_AVMute();
            }

            i = 0;
		    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_STATUS_REG, &c);
			while((c&0x04)==0x00)//wait for encryption.
			{
                DRVDelayMs(2);
				ANX7150_i2c_read_p0_reg(ANX7150_HDCP_STATUS_REG, &c);
                i++;
                if (i > 10)
                    break;
			}

            //enable audio SY.
            ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
            c |= ANX7150_HDMI_AUDCTRL1_IN_EN;
            ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);
            D("@@@@@  HDCP Auth PASSED!   @@@@@\n");

            if (ANX7150_hdcp_bcaps & 0x40) //repeater
            {
                D("Find a repeater!\n");
                //actually it is KSVList check. we can't do SRM check due to the lack of SRM file. SY.
                if (!ANX7150_srm_checked)
                {
                    if (!ANX7150_IS_KSVList_VLD())
                    {
                        D("ksvlist not good. disable encryption");
                        ANX7150_HDCP_Encryption_Disable();
                        ANX7150_Blue_Screen_Enable();
                        ANX7150_Clear_AVMute();
                        ANX7150_ksv_srm_pass = 0;
                        ANX7150_Clean_HDCP();//SY.
                        //remove below will pass 1b-05/1b-06
                        //ANX7150_Set_System_State(ANX7150_WAIT_HOTPLUG);//SY.
                        return;
                    }
                    ANX7150_srm_checked=1;
                    ANX7150_ksv_srm_pass = 1;
                }
            }
            else
            {
                D("Find a receiver.\n");
            }
        }
        else 							//wen HDCP CTS
        {
            D("#####   HDCP Auth FAILED!   #####\n");
            //also need to disable HW AUTHEN
			ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
			ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c &(~ANX7150_HDCP_CTRL0_HW_AUTHEN)));
            ANX7150_hdcp_auth_en = 0;
			//ANX7150_hdcp_init_done = 0;
			//ANX7150_hdcp_wait_100ms_needed = 1; //wen, update 080703

            if (ANX7150_hdcp_encryption)
            {
                ANX7150_HDCP_Encryption_Disable();
            }
            if (!ANX7150_send_blue_screen)
            {
                ANX7150_Blue_Screen_Enable();
            }
            if (ANX7150_avmute_enable)
            {
                ANX7150_Clear_AVMute();
            }
            //disable audio
            ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
            ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c & (~ANX7150_HDMI_AUDCTRL1_IN_EN));

            return;
        }

    }
    else				//wen HDCP CTS
    {
        D("hdcp pin is off.\n");
        if (ANX7150_send_blue_screen)
        {
            ANX7150_Blue_Screen_Disable();
        }
        if (ANX7150_avmute_enable)
        {
            ANX7150_Clear_AVMute();
        }
        //enable audio SY.
        ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c);
        c |= ANX7150_HDMI_AUDCTRL1_IN_EN;
        ANX7150_i2c_write_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, c);
    }

//wen HDCP CTS
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c); //72:07.1 hdmi or dvi mode
    c = c & 0x02;
    if (c == 0x02)
    {
        D("end of ANX7150_HDCP_Process(): in HDMI mode.\n");
    }
    else
    {
        D("!end of ANX7150_HDCP_Process(): in DVI mode.\n");
        //To-Do: Config to DVI mode.
    }
    ANX7150_Set_System_State(ANX7150_PLAY_BACK);
    ANX7150_Show_Video_Parameter();
}
//******************** end of HDCP process ********************************


//************************Play back process   **************************
void ANX7150_PLAYBACK_Process(void)
{
    if ((s_ANX7150_packet_config.packets_need_config != 0x00) && (ANX7150_edid_result.is_HDMI == 1))
    {
        ANX7150_Set_System_State(ANX7150_CONFIG_PACKETS);
    }
}
//******************** end of Play back process ********************************

void ANX7150_RST_DDCChannel(void)
{
    uint8 c;
    //Reset the DDC channel
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL2_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, (c | ANX7150_SYS_CTRL2_DDC_RST));
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, (c & (~ANX7150_SYS_CTRL2_DDC_RST)));
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x00); //abort current operation
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x06);//reset I2C command
//Clear FIFO
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x05);
}


uint8 ANX7150_BKSV_SRM(void)
{

#if 1
    uint8 bksv[5],i,bksv_one,c1;
    ANX7150_InitDDC_Read(0x74, 0x00, 0x00, 0x05, 0x00);
    DRVDelayMs(15);
    for (i = 0; i < 5; i ++)
    {
        ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &bksv[i]);
    }

    bksv_one = 0;
    for (i = 0; i < 8; i++)
    {
        c1 = 0x01 << i;
        if (bksv[0] & c1)
            bksv_one ++;
        if (bksv[1] & c1)
            bksv_one ++;
        if (bksv[2] & c1)
            bksv_one ++;
        if (bksv[3] & c1)
            bksv_one ++;
        if (bksv[4] & c1)
            bksv_one ++;
    }
    //wen HDCP CTS
    if (bksv_one != 20)
    {
        D("BKSV check fail\n");
        return 0;
    }
    else
    {
        D("BKSV check OK\n");
        return 1;
    }
#endif

#if 0					//wen HDCP CTS
    /*address by gerard.zhu*/
    uint8 i,j,bksv_ones_count,bksv_data[Bksv_Data_Nums] = {0};
    ANX7150_DDC_Addr bksv_ddc_addr;
    WORD bksv_length;
    ANX7150_DDC_Type ddc_type;

    i = 0;
    j = 0;
    bksv_ones_count = 0;
    bksv_ddc_addr.dev_addr = HDCP_Dev_Addr;
    bksv_ddc_addr.sgmt_addr = 0;
    bksv_ddc_addr.offset_addr = HDCP_Bksv_Offset;
    bksv_length = Bksv_Data_Nums;
    ddc_type = DDC_Hdcp;

    if (!ANX7150_DDC_Read(bksv_ddc_addr, bksv_data, bksv_length, ddc_type))
    {
        /*Judge validity for Bksv*/
        while (i < Bksv_Data_Nums)
        {
            while (j < 8)
            {
                if (((bksv_data[i] >> j) & 0x01) == 1)
                {
                    bksv_ones_count++;
                }
                j++;
            }
            i++;
            j = 0;
        }
        if (bksv_ones_count != 20)
        {
            rk28printk ("!!!!BKSV 1s ¡Ù20\n");					//update  rk28printk ("!!!!BKSV 1s ¡Ù20\n");
            return 0;
        }
    }
    /*end*/

    D("bksv is ready.\n");
    // TODO: Compare the bskv[] value to the revocation list to decide if this value is a illegal BKSV. This is system depended.
    //If illegal, return 0; legal, return 1. Now just return 1
    return 1;
#endif
}

uint8 ANX7150_IS_KSVList_VLD(void)
{
//wen HDCP CTS
#if 1
    D("ANX7150_IS_KSVList_VLD() is called.\n");
    ANX7150_InitDDC_Read(0x74, 0x00, 0x41, 0x02, 0x00); //Bstatus, two uint8s
    DRVDelayMs(5);
    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &ANX7150_hdcp_bstatus[0]);
    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &ANX7150_hdcp_bstatus[1]);
    if ((ANX7150_hdcp_bstatus[0] & 0x80) | (ANX7150_hdcp_bstatus[1] & 0x08))
    {
        D("Max dev/cascade exceeded: ANX7150_hdcp_bstatus[0]: 0x%x,ANX7150_hdcp_bstatus[1]:0x%x\n", (WORD)ANX7150_hdcp_bstatus[0],(WORD)ANX7150_hdcp_bstatus[1]);
        return 0;//HDCP topology error. More than 127 RX are attached or more than seven levels of repeater are cascaded.
    }
    return 1;
#endif
//wen HDCP CTS


}

void ANX7150_Hardware_HDCP_Auth_Init(void)
{
    uint8 c;

//    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c); //72:07.2 hdcp on
//    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, (c | ANX7150_SYS_CTRL1_HDCPMODE));
	// disable hw hdcp
//    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
//    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_HW_AUTHEN)));

    //ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, 0x03); //h/w auth off, jh simplay/hdcp
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, 0x00); //bit 0/1 off, as from start, we don't know if Bksv/srm/KSVList valid or not. SY.

    // DDC reset
    ANX7150_RST_DDCChannel();

    ANX7150_InitDDC_Read(0x74, 0x00, 0x40, 0x01, 0x00);
    DRVDelayMs(5);
    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &ANX7150_hdcp_bcaps);
    D("ANX7150_Hardware_HDCP_Auth_Init(): ANX7150_hdcp_bcaps = 0x%.2x\n",    (WORD)ANX7150_hdcp_bcaps);

    if (ANX7150_hdcp_bcaps & 0x02)
    {   //enable 1.1 feature
    	 D("ANX7150_Hardware_HDCP_Auth_Init(): bcaps supports 1.1\n");
        ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL1_REG, (c |ANX7150_HDCP_CTRL1_HDCP11_EN));
    }
    else
    {   //disable 1.1 feature and enable HDCP two special point check
    	D("bcaps don't support 1.1\n");
    	ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL1_REG, &c);
        ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL1_REG,
                                                     ((c & (~ANX7150_HDCP_CTRL1_HDCP11_EN)) | ANX7150_LINK_CHK_12_EN));
    }
    //handle repeater bit. SY.
    if (ANX7150_hdcp_bcaps & 0x40)
    {
	         //repeater
	D("ANX7150_Hardware_HDCP_Auth_Init(): bcaps shows Sink is a repeater\n");
	ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c | ANX7150_HDCP_CTRL0_RX_REP));
    }
    else
    {
			 //receiver
	D("ANX7150_Hardware_HDCP_Auth_Init(): bcaps shows Sink is a receiver\n");
	ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
	ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_RX_REP)));
    }
    ANX7150_RST_DDCChannel();
    ANX7150_hdcp_auth_en = 0;
}


void ANX7150_Clean_HDCP(void)
{
    uint8 c;
    //mute TMDS link
    //ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);//jack wen
    //ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, c & (~ANX7150_TMDS_CLKCH_MUTE));

    //Disable hardware HDCP
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, (c & (~ANX7150_HDCP_CTRL0_HW_AUTHEN)));

    //Reset HDCP logic
    ANX7150_i2c_read_p0_reg(ANX7150_SRST_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SRST_REG, (c | ANX7150_SRST_HDCP_RST) );
    ANX7150_i2c_write_p0_reg(ANX7150_SRST_REG, (c & (~ANX7150_SRST_HDCP_RST)) );

    //Set ReAuth
    ANX7150_i2c_read_p0_reg(ANX7150_HDCP_CTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, c |ANX7150_HDCP_CTRL0_RE_AUTH);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_CTRL0_REG, c & (~ANX7150_HDCP_CTRL0_RE_AUTH));
    ANX7150_hdcp_auth_en = 0;
    //ANX7150_bksv_ready = 0;
    ANX7150_hdcp_auth_pass = 0;
    ANX7150_hdcp_auth_fail_counter =0 ;
    ANX7150_hdcp_encryption = 0;
    ANX7150_send_blue_screen = 0;
    ANX7150_hdcp_init_done = 0;
    ANX7150_hdcp_wait_100ms_needed = 1;
    ANX7150_auth_fully_pass = 0;
    ANX7150_srm_checked = 0;
    ANX7150_RST_DDCChannel();
}


void ANX7150_Set_System_State(uint8 ss)
{
    ANX7150_system_state = ss;
    D("ANX7150 State: ");
    switch (ss)
    {
        case ANX7150_INITIAL:
            D1("ANX7150_INITIAL\n");
            break;
        case ANX7150_WAIT_HOTPLUG:
            D1("ANX7150_WAIT_HOTPLUG\n");
            break;
        case ANX7150_READ_PARSE_EDID:
            D1("ANX7150_READ_PARSE_EDID\n");
            break;
        case ANX7150_WAIT_RX_SENSE:
            D1("ANX7150_WAIT_RX_SENSE\n");
            break;
        case ANX7150_CONFIG_VIDEO:
            D1("ANX7150_CONFIG_VIDEO\n");
            break;
        case ANX7150_CONFIG_AUDIO:
            D1("ANX7150_CONFIG_AUDIO\n");
            break;
        case ANX7150_CONFIG_PACKETS:
            D1("ANX7150_CONFIG_PACKETS\n");
            break;
        case ANX7150_HDCP_AUTHENTICATION:
            D1("ANX7150_HDCP_AUTHENTICATION\n");
            break;
            ////////////////////////////////////////////////
            // System ANX7150_RESET_LINK is kept for RX clock recovery error case, not used in normal case.
        case ANX7150_RESET_LINK:
            D1("ANX7150_RESET_LINK\n");
            break;
            ////////////////////////////////////////////////
        case ANX7150_PLAY_BACK:
            D1("ANX7150_PLAY_BACK\n");
            break;
		default:
			D1("ANX7150 unknown state\n");
			break;
    }
}

void ANX7150_Hardware_Initial(void)
{
    uint8 c = 0;
	
    //clear HDCP_HPD_RST
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL2_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, c | 0x01);;
	
    DRVDelayMs(10);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, c &~0x01);
    
    //Power on I2C
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL3_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL3_REG, (c | ANX7150_SYS_CTRL3_I2C_PWON));

    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL2_REG, 0x00);
    ANX7150_i2c_write_p0_reg(ANX7150_SRST_REG, 0x00);
    //clear HDCP_HPD_RST
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c & 0xbf);
    //Power on Audio capture and Video capture module clock
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_PD_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_PD_REG, (c | 0x06));


    //Enable auto set clock range for video PLL
    ANX7150_i2c_read_p0_reg(ANX7150_CHIP_CTRL_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_CHIP_CTRL_REG, (c & 0x00));    //  cwz change 0xfe to 0x00

    //Set registers value of Blue Screen when HDCP authentication failed--RGB mode,green field
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN0_REG, 0x10);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN1_REG, 0xeb);
    ANX7150_i2c_write_p0_reg(ANX7150_HDCP_BLUESCREEN2_REG, 0x10);

    //ANX7150_i2c_read_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, &c);
    //ANX7150_i2c_write_p0_reg(ANX7150_TMDS_CLKCH_CONFIG_REG, (c | 0x80));

    ANX7150_i2c_read_p0_reg(ANX7150_PLL_CTRL0_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_PLL_CTRL0_REG, 0x00); //wen change to 0xa8 // cwz change 0xa8 to 0x00


    ANX7150_i2c_read_p0_reg(ANX7150_CHIP_DEBUG1_CTRL_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_CHIP_DEBUG1_CTRL_REG, (c | 0x08));

    ANX7150_i2c_read_p0_reg(ANX7150_PLL_TX_AMP, &c);//jack wen
    ANX7150_i2c_write_p0_reg(ANX7150_PLL_TX_AMP, (c | 0x01));//TMDS swing


    ANX7150_i2c_write_p0_reg(ANX7150_PLL_CTRL1_REG,0x00);//Added for PLL unlock issue in high temperature - Feiw
    //if (ANX7150_AUD_HW_INTERFACE == 0x02) //jack wen, spdif

    ANX7150_i2c_read_p0_reg(ANX7150_I2S_CTRL_REG, &c);//jack wen, for spdif input from SD0.
    ANX7150_i2c_write_p0_reg(ANX7150_I2S_CTRL_REG, (c&  0xef));//


    ANX7150_i2c_write_p0_reg(0xE1, 0xC7);

    //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, 0);//c & 0xfb);//power down HDCP, 090630
    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL3_REG, &c);
    ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL3_REG, c & 0xfe);//power down all, 090630

    ANX7150_Set_System_State(ANX7150_WAIT_HOTPLUG);
}

void ANX7150_API_Initial()
{
    ANX7150_Variable_Initial();
    ANX7150_HW_Interface_Variable_Initial();
    ANX7150_Hardware_Initial();
}

/*void ANX7150_Interrupt_Information(uint8 c, uint8 n)
{
    uint8 TX_is_HDMI,c1;

    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c1);
    TX_is_HDMI = c1 & 0x02;

    switch (n)
    {
        case 1:
            if (c & ANX7150_INTR1_STATUS_CLK_CHG)
                ;//D("ANX7150_Int: Video input clock change detected.\n");//jack wen
            if ((c & ANX7150_INTR1_STATUS_CTS_OVRWR) && (TX_is_HDMI == 0x02))
                ;//D("ANX7150_Int: Audio CTS is overwrite before sending by ACR packer.\n");
            if (c & ANX7150_INTR1_STATUS_HP_CHG)
                D("ANX7150_Int: Hotplug change detected.\n");
            if (c & ANX7150_INTR1_STATUS_SW_INT)
                D("ANX7150_Int: Software induced interrupt.\n");
            if ((c & ANX7150_INTR1_STATUS_SPDIF_ERR)&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: S/PDIF parity errors.\n");
            if ((c & ANX7150_INTR1_STATUS_AFIFO_OVER)&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: Audio FIFO is overrun.\n");
            if ((c & ANX7150_INTR1_STATUS_AFIFO_UNDER)&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: Audio FIFO is underrun.\n");
            if ((c & ANX7150_INTR1_STATUS_CTS_CHG)&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: Audio CTS changed.\n");
            break;
        case 2:
            if (c & ANX7150_INTR2_STATUS_AUTH_DONE)
                D("ANX7150_Int: HDCP authentication ended.\n");
            if (c & ANX7150_INTR2_STATUS_AUTH_CHG)
                D("ANX7150_Int: Hardware HDCP authentication state changed.\n");
            if (c & ANX7150_INTR2_STATUS_SHA_DONE)
                D("ANX7150_Int: Hardware HDCP computing V ended.\n");
            if (c & ANX7150_INTR2_STATUS_PLLLOCK_CHG)
                D("ANX7150_Int: PLL clock state changed.\n");
            if (c & ANX7150_INTR2_STATUS_BKSV_RDY)
                D("ANX7150_Int: BKSV ready for check.\n");
            if (c & ANX7150_INTR2_STATUS_HDCPENHC_CHK)
                D("ANX7150_Int: Enhanced link verification is need.\n");
            if (c & ANX7150_INTR2_STATUS_HDCPLINK_CHK)
                D("ANX7150_Int: Link integrity check is need.\n");
            if (c & ANX7150_INTR2_STATUS_ENCEN_CHG)
                //D("ANX7150_Int: ENC_EN changed detected.\n");
            break;
        case 3:
            if ((c & ANX7150_INTR3_STATUS_SPDIF_UNSTBL)&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: Not find expected preamble for SPDIF input.\n");
            if (c & ANX7150_INTR3_STATUS_RXSEN_CHG)
                D("ANX7150_Int: Receiver active sense changed.\n");
            if (c & ANX7150_INTR3_STATUS_VSYNC_DET)
                D("ANX7150_Int: VSYNC active edge detected.\n");
            if (c & ANX7150_INTR3_STATUS_DDC_NOACK)
                D("ANX7150_Int: DDC master not detected any ACK.\n");
            if (c & ANX7150_INTR3_STATUS_DDCACC_ERR)
                D("ANX7150_Int: DDC channel access error.\n");
            if ((c & ANX7150_INTR3_STATUS_AUDCLK_CHG)&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: Audio input clock changed.\n");
            if (c & ANX7150_INTR3_STATUS_VIDF_CHG)
                D("ANX7150_Int: Video input format changed.\n");
            if ((c & ANX7150_INTR3_STATUS_SPDIFBI_ERR )&& (TX_is_HDMI == 0x02))
                D("ANX7150_Int: SPDIF bi-phase error.\n");
            break;
    }
}
*/

void ANX7150_Interrupt_Process()
{
    uint8 c,c1;
	//uint8 c2;
    uint8 int_s1, int_s2, int_s3;

    if ((ANX7150_system_state == ANX7150_INITIAL) || (ANX7150_system_state == ANX7150_WAIT_HOTPLUG))
        return;

    ANX7150_i2c_read_p0_reg(ANX7150_INTR1_STATUS_REG, &int_s1);
    ANX7150_i2c_write_p0_reg(ANX7150_INTR1_STATUS_REG, int_s1);

    ANX7150_i2c_read_p0_reg(ANX7150_INTR2_STATUS_REG, &int_s2);
    ANX7150_i2c_write_p0_reg(ANX7150_INTR2_STATUS_REG, int_s2);

    ANX7150_i2c_read_p0_reg(ANX7150_INTR3_STATUS_REG, &int_s3);
    ANX7150_i2c_write_p0_reg(ANX7150_INTR3_STATUS_REG, int_s3);

    //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
    //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL3_REG, &c1);
    //ANX7150_i2c_read_p0_reg(ANX7150_INTR_STATE_REG, &c2);


    if ((int_s1 & ANX7150_INTR1_STATUS_HP_CHG)||(HDMI_Get_Lowpower()))//Normal and standby both
       {
            D("int_s1 =0x%.2x, HDMI_Get_Lowpower=0x%.2x\n", int_s1,HDMI_Get_Lowpower());
           ANX7150_Hotplug_Change_Interrupt();
       }


    if (int_s1 & ANX7150_INTR1_STATUS_CLK_CHG)
        //ANX7150_Video_Clock_Change_Interrupt();//jack wen
        
	if (int_s3 & ANX7150_INTR3_STATUS_VIDF_CHG)
            ANX7150_Video_Format_Change_Interrupt();

    if (int_s2 & ANX7150_INTR2_STATUS_AUTH_DONE)
        ANX7150_Auth_Done_Interrupt();

    if (int_s2 & ANX7150_INTR2_STATUS_AUTH_CHG)
        ANX7150_Auth_Change_Interrupt();

    ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
    c = c & 0x02;          // HDMI mode
    if (c == 0x02)
    {
        if (int_s3 & ANX7150_INTR3_STATUS_AUDCLK_CHG)
            ANX7150_Audio_CLK_Change_Interrupt();
        if (int_s1 & ANX7150_INTR1_STATUS_AFIFO_OVER)
            ANX7150_AFIFO_Overrun_Interrupt();

        // SPDIF error
        if ((int_s3 & 0x81) || (int_s1 & ANX7150_INTR1_STATUS_SPDIF_ERR))
        {
            ANX7150_i2c_read_p0_reg(ANX7150_HDMI_AUDCTRL1_REG, &c1);
            if ( c1 & ANX7150_HDMI_AUDCTRL1_SPDIFIN_EN)
            {
                ANX7150_SPDIF_Error_Interrupt(int_s1,int_s3);
            }
        }
        else
        {
            if (spdif_error_cnt > 0 && ANX7150_system_state == ANX7150_PLAY_BACK)    spdif_error_cnt --;
            if (spdif_error_cnt > 0 && ANX7150_system_state < ANX7150_CONFIG_AUDIO)    spdif_error_cnt = 0x00;
        }
    }

    if (int_s2 & ANX7150_INTR2_STATUS_PLLLOCK_CHG)
        ANX7150_PllLock_Interrupt();

    if (int_s3 & ANX7150_INTR3_STATUS_RXSEN_CHG)
    {
        ANX7150_Rx_Sense_Interrupt(); //060819
    }

    //if(int_s2 & ANX7150_INTR2_STATUS_HDCPLINK_CHK)
    //  ANX7150_HDCPLINK_CHK_Interrupt();



    //}
}


uint8 ANX7150_Parse_EDID(void)
{
    uint8 i;
    ANX7150_GetEDIDLength();

    D("EDIDLength is %.u\n",  ANX7150_edid_length);

    ANX7150_Read_EDID();
    /*
        if(!(ANX7150_Parse_EDIDHeader()))
        {
            D("BAD EDID Header, Stop parsing \n");
            ANX7150_edid_result.edid_errcode = ANX7150_EDID_BadHeader;
            return ANX7150_edid_result.edid_errcode;
        }

        if(!(ANX7150_Parse_EDIDVersion()))
        {
            D("EDID does not support 861B, Stop parsing\n");
            ANX7150_edid_result.edid_errcode = ANX7150_EDID_861B_not_supported;
            return ANX7150_edid_result.edid_errcode;
        }

        if(ANX7150_EDID_Checksum(0) == 0)
        {
            D("EDID Block one check sum error, Stop parsing\n");
            ANX7150_edid_result.edid_errcode = ANX7150_EDID_CheckSum_ERR;
            return ANX7150_edid_result.edid_errcode;
        }*/

    //ANX7150_Parse_BasicDis();
    ANX7150_Parse_DTDinBlockONE();
    /*
        if(ANX7150_EDID_Buf[0x7e] == 0)
        {
            D("No EDID extension blocks.\n");
            ANX7150_edid_result.edid_errcode = ANX7150_EDID_No_ExtBlock;
            return ANX7150_edid_result.edid_errcode;
        }*/
    ANX7150_Parse_NativeFormat();
    ANX7150_Parse_ExtBlock();

    if (ANX7150_edid_result.edid_errcode == 0x05)
        return ANX7150_edid_result.edid_errcode;

    if (ANX7150_edid_result.edid_errcode == 0x03)
        return ANX7150_edid_result.edid_errcode;

    D("EDID parsing finished!\n");

    {
        D("ANX7150_edid_result.edid_errcode = 0x%.2x\n",(WORD)ANX7150_edid_result.edid_errcode);
        D("ANX7150_edid_result.is_HDMI = 0x%.2x\n",(WORD)ANX7150_edid_result.is_HDMI);
        D("ANX7150_edid_result.ycbcr422_supported = 0x%.2x\n",(WORD)ANX7150_edid_result.ycbcr422_supported);
        D("ANX7150_edid_result.ycbcr444_supported = 0x%.2x\n",(WORD)ANX7150_edid_result.ycbcr444_supported);
        D("ANX7150_edid_result.supported_1080i_60Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_1080i_60Hz);
        D("ANX7150_edid_result.supported_1080i_50Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_1080i_50Hz);
        D("ANX7150_edid_result.supported_720p_60Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_720p_60Hz);
        D("ANX7150_edid_result.supported_720p_50Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_720p_50Hz);
        D("ANX7150_edid_result.supported_640x480p_60Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_640x480p_60Hz);
        D("ANX7150_edid_result.supported_720x480p_60Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_720x480p_60Hz);
        D("ANX7150_edid_result.supported_720x480i_60Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_720x480i_60Hz);
        D("ANX7150_edid_result.supported_576p_50Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_576p_50Hz);
        D("ANX7150_edid_result.supported_576i_50Hz = 0x%.2x\n",(WORD)ANX7150_edid_result.supported_576i_50Hz);
        if (!ANX7150_edid_result.edid_errcode)
        {
            for (i = 0; i < ANX7150_sau_length/3; i++)
            {
                D("ANX7150_edid_result.AudioChannel = 0x%.2x\n",(WORD)ANX7150_edid_result.AudioChannel[i]);
                D("ANX7150_edid_result.AudioFormat = 0x%.2x\n",(WORD)ANX7150_edid_result.AudioFormat[i]);
                D("ANX7150_edid_result.AudioFs = 0x%.2x\n",(WORD)ANX7150_edid_result.AudioFs[i]);
                D("ANX7150_edid_result.AudioLength = 0x%.2x\n",(WORD)ANX7150_edid_result.AudioLength[i]);
            }
            D("ANX7150_edid_result.SpeakerFormat = 0x%.2x\n",(WORD)ANX7150_edid_result.SpeakerFormat);
        }
    }

	return 0;
}

void ANX7150_GetEDIDLength()
{
    uint8 edid_data_length,c,i;

    ANX7150_RST_DDCChannel();

    ANX7150_InitDDC_Read(0xa0, 0x00, 0x7e, 0x01, 0x00);
    DRVDelayMs(3);//FeiW - Analogix
    for(i=0;i<10;i++)
   	{
		ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFOCNT_REG, &c);
		if(c!=0)
			return;
	}
	DRVDelayMs(10);

    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &edid_data_length);

    ANX7150_edid_length = edid_data_length * 128 + 128;

}
void ANX7150_Read_EDID(void)
{
    uint8 edid_segment,segmentpointer,k;

    ANX7150_RST_DDCChannel();

    edid_segment = ANX7150_edid_length / 256;
    if (edid_segment==0)																			//update
        segmentpointer =0;
    else
        segmentpointer = edid_segment - 1;
    //segmentpointer = edid_segment - 1;

    for (k = 0; k <= segmentpointer; k ++)
    {
        ANX7150_InitDDC_Read(0xa0, k, 0x00, 0x80, 0x00);
        ANX7150_DDC_Mass_Read(128, k);
        ANX7150_InitDDC_Read(0xa0, k, 0x80, 0x80, 0x00);
        ANX7150_DDC_Mass_Read(128, k + 1);
    }

    if ((ANX7150_edid_length - 256 * edid_segment) == 0)
        D("Finish reading EDID\n");
    else
    {
        D("Read one more block(128 uint8s).........\n");
        ANX7150_InitDDC_Read(0xa0, segmentpointer + 1, 0x00, 0x80, 0x00);
        ANX7150_DDC_Mass_Read(128, segmentpointer + 1);
        D("Finish reading EDID\n");
    }
}


void ANX7150_DDC_Mass_Read(WORD length, uint8 segment)
{
    WORD i, j;
    uint8 c, c1,ddc_empty_cnt;

    i = length;
    while (i > 0)
    {
        //check DDC FIFO statue
        ANX7150_i2c_read_p0_reg(ANX7150_DDC_CHSTATUS_REG, &c);
        if (c & ANX7150_DDC_CHSTATUS_DDC_OCCUPY)
        {
            //D("ANX7150 DDC channel is accessed by an external device, break!.\n");
            break;
        }
        if (c & ANX7150_DDC_CHSTATUS_FIFO_FULL)
            ANX7150_ddc_fifo_full = 1;
        else
            ANX7150_ddc_fifo_full = 0;
        if (c & ANX7150_DDC_CHSTATUS_INPRO)
            ANX7150_ddc_progress = 1;
        else
            ANX7150_ddc_progress = 0;
        if (ANX7150_ddc_fifo_full)
        {
            //D("DDC FIFO is full during edid reading");
            ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFOCNT_REG, &c);
            //D("FIFO counter is %.2x\n", (WORD) c);
            for (j=0; j<c; j++)
            {
                ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &c1);
                if (segment == 0)
                {
                    ANX7150_EDID_Buf[length - i + j] = c1;
                    //D("EDID[0x%.2x]=0x%.2x    ", (WORD)(length - i + j), (WORD) c1);
                }
                else if (segment == 1)
                {
                    ANX7150_EDID_Buf[length - i + j + 0x80] = c1;
                    //D("EDID[0x%.2x]=0x%.2x    ", (WORD)(length - i + j + 0x80), (WORD) c1);
                }

                ANX7150_ddc_fifo_full = 0;
            }
            i = i - c;
            //D("\n");
        }
        else if (!ANX7150_ddc_progress)
        {
            //D("ANX7150 DDC FIFO access finished.\n");
            ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFOCNT_REG, &c);
            //D("FIFO counter is %.2x\n", (WORD) c);
            if (!c)
            {
                i =0;
                break;
            }
            for (j=0; j<c; j++)
            {
                ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &c1);
                if (segment == 0)
                {
                    ANX7150_EDID_Buf[length - i + j] = c1;
                    //D("EDID[0x%.2x]=0x%.2x    ", (WORD)(length - i + j), (WORD) c1);
                }
                else if (segment == 1)
                {
                    ANX7150_EDID_Buf[length - i + j + 0x80] = c1;
                    //D("EDID[0x%.2x]=0x%.2x    ", (WORD)(length - i + j + 0x80), (WORD) c1);
                }
            }
            i = i - c;
            //D("\ni=%d\n", i);
        }
        else
        {
            ddc_empty_cnt = 0x00;
            for (c1=0; c1<0x0a; c1++)
            {
                ANX7150_i2c_read_p0_reg(ANX7150_DDC_CHSTATUS_REG, &c);
                //D("DDC FIFO access is progressing.\n");
                //D("DDC Channel status is 0x%.2x\n",(WORD)c);
                if (c & ANX7150_DDC_CHSTATUS_FIFO_EMPT)
                    ddc_empty_cnt++;
                DRVDelayMs(5);
                //D("ddc_empty_cnt =  0x%.2x\n",(WORD)ddc_empty_cnt);
            }
            if (ddc_empty_cnt >= 0x0a)
                i = 0;
        }
    }
}


uint8 ANX7150_Parse_EDIDHeader(void)
{
    uint8 i,temp;
    temp = 0;
    // the EDID header should begin with 0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00
    if ((ANX7150_Read_EDID_uint8(0, 0) == 0x00) && (ANX7150_Read_EDID_uint8(0, 7) == 0x00))
    {
        for (i = 1; i < 7; i++)
        {
            if (ANX7150_Read_EDID_uint8(0, i) != 0xff)
            {
                temp = 0x01;
                break;
            }
        }
    }
    else
    {
        temp = 0x01;
    }
    if (temp == 0x01)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
uint8 ANX7150_Parse_EDIDVersion(void)
{

    if (!((ANX7150_Read_EDID_uint8(0, 0x12) == 1) && (ANX7150_Read_EDID_uint8(0, 0x13) >= 3) ))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
uint8 ANX7150_Parse_ExtBlock()
{
    uint8 i,c;

    for (i = 0; i < ANX7150_Read_EDID_uint8(0, 0x7e); i++)   //read in blocks
    {
        c = ANX7150_Read_EDID_uint8(i/2, 0x80);
        if ( c == 0x02)
        {
            ANX7150_ext_block_num = i + 1;
            ANX7150_Parse_DTDinExtBlock();
            ANX7150_Parse_STD();
            if (!(ANX7150_EDID_Checksum(ANX7150_ext_block_num)))
            {
                ANX7150_edid_result.edid_errcode = ANX7150_EDID_CheckSum_ERR;
                return ANX7150_edid_result.edid_errcode;
            }
        }
        else
        {
            ANX7150_edid_result.edid_errcode = ANX7150_EDID_ExtBlock_NotFor_861B;
            return ANX7150_edid_result.edid_errcode;
        }
    }

	return 0;
}
void ANX7150_Parse_DTDinBlockONE()
{
    uint8 i;
    for (i = 0; i < 18; i++)
    {
        ANX7150_edid_dtd[i] = ANX7150_Read_EDID_uint8(0, (i + 0x36));
    }
    //D("Parse the first DTD in Block one:\n");
    ANX7150_Parse_DTD();

    if ((ANX7150_Read_EDID_uint8(0, 0x48) == 0)
            && (ANX7150_Read_EDID_uint8(0, 0x49) == 0)
            && (ANX7150_Read_EDID_uint8(0, 0x4a) == 0))
    {
        ;//D("the second DTD in Block one is not used to descript video timing.\n");
    }
    else
    {
        for (i = 0; i < 18; i++)
        {
            ANX7150_edid_dtd[i] = ANX7150_Read_EDID_uint8(0, (i + 0x48));
        }
        ANX7150_Parse_DTD();
    }

    if ((ANX7150_Read_EDID_uint8(0,0x5a) == 0)
            && (ANX7150_Read_EDID_uint8(0,0x5b) == 0)
            && (ANX7150_Read_EDID_uint8(0,0x5c) == 0))
    {
        ;//D("the third DTD in Block one is not used to descript video timing.\n");
    }
    else
    {
        for (i = 0; i < 18; i++)
        {
            ANX7150_edid_dtd[i] = ANX7150_Read_EDID_uint8(0, (i + 0x5a));
        }
        ANX7150_Parse_DTD();
    }

    if ((ANX7150_Read_EDID_uint8(0,0x6c) == 0)
            && (ANX7150_Read_EDID_uint8(0,0x6d) == 0)
            && (ANX7150_Read_EDID_uint8(0,0x6e) == 0))
    {
        ;//D("the fourth DTD in Block one is not used to descript video timing.\n");
    }
    else
    {
        for (i = 0; i < 18; i++)
        {
            ANX7150_edid_dtd[i] = ANX7150_Read_EDID_uint8(0,(i + 0x6c));
        }
        ANX7150_Parse_DTD();
    }
}

void ANX7150_Parse_DTDinExtBlock()
{
    uint8 i,DTDbeginAddr;
    DTDbeginAddr = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2, 0x82)
                   + 0x80;
    while (DTDbeginAddr < (0x6c + 0x80))
    {
        if ((ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,DTDbeginAddr) == 0)
                && (ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(DTDbeginAddr + 1)) == 0)
                && (ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(DTDbeginAddr + 2)) == 0))
        {
            ;//D("this DTD in Extension Block is not used to descript video timing.\n");
        }
        else
        {
            for (i = 0; i < 18; i++)
            {
                ANX7150_edid_dtd[i] = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(i + DTDbeginAddr));
            }
            //D("Parse the DTD in Extension Block :\n");
            ANX7150_Parse_DTD();
        }
        DTDbeginAddr = DTDbeginAddr + 18;
    }
}

void ANX7150_Parse_DTD()
{
    WORD temp;
    unsigned long temp1,temp2;
    WORD Hresolution,Vresolution,Hblanking,Vblanking;
    WORD PixelCLK,Vtotal,H_image_size,V_image_size;
    uint8 Hz;
    //float Ratio;

    temp = ANX7150_edid_dtd[1];
    temp = temp << 8;
    PixelCLK = temp + ANX7150_edid_dtd[0];
    //	D("Pixel clock is 10000 * %u\n",  temp);

    temp = ANX7150_edid_dtd[4];
    temp = (temp << 4) & 0x0f00;
    Hresolution = temp + ANX7150_edid_dtd[2];
    //D("Horizontal Active is  %u\n",  Hresolution);

    temp = ANX7150_edid_dtd[4];
    temp = (temp << 8) & 0x0f00;
    Hblanking = temp + ANX7150_edid_dtd[3];
    //D("Horizontal Blanking is  %u\n",  temp);

    temp = ANX7150_edid_dtd[7];
    temp = (temp << 4) & 0x0f00;
    Vresolution = temp + ANX7150_edid_dtd[5];
    //D("Vertical Active is  %u\n",  Vresolution);

    temp = ANX7150_edid_dtd[7];
    temp = (temp << 8) & 0x0f00;
    Vblanking = temp + ANX7150_edid_dtd[6];
    //D("Vertical Blanking is  %u\n",  temp);

    temp = ANX7150_edid_dtd[11];
    temp = (temp << 2) & 0x0300;
    temp = temp + ANX7150_edid_dtd[8];
    //D("Horizontal Sync Offset is  %u\n",  temp);

    temp = ANX7150_edid_dtd[11];
    temp = (temp << 4) & 0x0300;
    temp = temp + ANX7150_edid_dtd[9];
    //D("Horizontal Sync Pulse is  %u\n",  temp);

    temp = ANX7150_edid_dtd[11];
    temp = (temp << 2) & 0x0030;
    temp = temp + (ANX7150_edid_dtd[10] >> 4);
    //D("Vertical Sync Offset is  %u\n",  temp);

    temp = ANX7150_edid_dtd[11];
    temp = (temp << 4) & 0x0030;
    temp = temp + (ANX7150_edid_dtd[8] & 0x0f);
    //D("Vertical Sync Pulse is  %u\n",  temp);

    temp = ANX7150_edid_dtd[14];
    temp = (temp << 4) & 0x0f00;
    H_image_size = temp + ANX7150_edid_dtd[12];
    //D("Horizontal Image size is  %u\n",  temp);

    temp = ANX7150_edid_dtd[14];
    temp = (temp << 8) & 0x0f00;
    V_image_size = temp + ANX7150_edid_dtd[13];
    //D("Vertical Image size is  %u\n",  temp);

    //D("Horizontal Border is  %bu\n",  ANX7150_edid_dtd[15]);

    //D("Vertical Border is  %bu\n",  ANX7150_edid_dtd[16]);

    temp1 = Hresolution + Hblanking;
    Vtotal = Vresolution + Vblanking;
    temp1 = temp1 * Vtotal;
    temp2 = PixelCLK;
    temp2 = temp2 * 10000;
    if (temp1 == 0)																												//update
        Hz=0;
    else
        Hz = temp2 / temp1;
    //Hz = temp2 / temp1;
    if ((Hz == 59) || (Hz == 60))
    {
        Hz = 60;
        //D("_______________Vertical Active is  %u\n",  Vresolution);
        if (Vresolution == 540)
            ANX7150_edid_result.supported_1080i_60Hz = 1;
        if (Vresolution == 1080)
            ANX7150_edid_result.supported_1080p_60Hz = 1;
        if (Vresolution == 720)
            ANX7150_edid_result.supported_720p_60Hz = 1;
        if ((Hresolution == 640) && (Vresolution == 480))
            ANX7150_edid_result.supported_640x480p_60Hz = 1;
        if ((Hresolution == 720) && (Vresolution == 480))
            ANX7150_edid_result.supported_720x480p_60Hz = 1;
        if ((Hresolution == 720) && (Vresolution == 240))
            ANX7150_edid_result.supported_720x480i_60Hz = 1;
    }
    if (Hz == 50)
    {
        //D("+++++++++++++++Vertical Active is  %u\n",  Vresolution);
        if (Vresolution == 540)
            ANX7150_edid_result.supported_1080i_50Hz = 1;
        if (Vresolution == 1080)
            ANX7150_edid_result.supported_1080p_50Hz = 1;
        if (Vresolution == 720)
            ANX7150_edid_result.supported_720p_50Hz = 1;
        if (Vresolution == 576)
            ANX7150_edid_result.supported_576p_50Hz = 1;
        if (Vresolution == 288)
            ANX7150_edid_result.supported_576i_50Hz = 1;
    }
    //D("Fresh rate :% bu Hz\n", Hz);
    //Ratio = H_image_size;
    //Ratio = Ratio / V_image_size;
    //D("Picture ratio : %f \n", Ratio);
}
/*void ANX7150_Parse_BasicDis()
{
    uint8 temp;
    temp = ANX7150_Read_EDID_uint8(0,0x18) & 0x18;
    	if(temp == 0x00)
    	//D("EDID Display type: mon/gray display.\n");
    else if(temp == 0x08)
    	//D("EDID Display type: RGB color display.\n");
    else if(temp == 0x10)
    	//D("EDID Display type: non-RGB color display.\n");
    else
    	//D("EDID Display type: Undefined.\n");
    temp = ANX7150_Read_EDID_uint8(0,0x18) & 0x02;
    if(temp == 0x00)
    	//D("EDID Preferred_timing: not supported.\n");
    else
    	//D("EDID Preferred_timing: supported.\n");
}
*/
void ANX7150_Parse_NativeFormat()
{
    uint8 temp;
    temp = ANX7150_Read_EDID_uint8(0,0x83) & 0xf0;
    /*if(temp & 0x80)
     	;//D("DTV supports underscan.\n");
     if(temp & 0x40)
     	;//D("DTV supports BasicAudio.\n");*/
    if (temp & 0x20)
    {
        //D("DTV supports YCbCr 4:4:4.\n");
        ANX7150_edid_result.ycbcr444_supported= 1;
    }
    if (temp & 0x10)
    {
        //D("DTV supports YCbCr 4:2:2.\n");
        ANX7150_edid_result.ycbcr422_supported= 1;
    }
}

void ANX7150_Parse_STD()
{
    uint8 DTDbeginAddr;
    ANX7150_stdaddr = 0x84;
    DTDbeginAddr = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,0x82) + 0x80;
    // D("Video DTDbeginAddr Register :%.2x\n", (WORD) DTDbeginAddr);
    while (ANX7150_stdaddr < DTDbeginAddr)
    {
        ANX7150_stdreg = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,ANX7150_stdaddr);
        switch (ANX7150_stdreg & 0xe0)
        {
            case 0x20:
                ANX7150_Parse_AudioSTD();
                ANX7150_sau_length = ANX7150_stdreg & 0x1f;
                break;
            case 0x40:
                ANX7150_Parse_VideoSTD();
                ANX7150_svd_length = ANX7150_stdreg & 0x1f;
                break;
            case 0x80:
                ANX7150_Parse_SpeakerSTD();
                break;
            case 0x60:
                ANX7150_Parse_VendorSTD();
                break;
            default:
                break;
        }
        ANX7150_stdaddr = ANX7150_stdaddr + (ANX7150_stdreg & 0x1f) + 0x01;
    }
}

void ANX7150_Parse_AudioSTD()
{
    uint8 i,AudioFormat,STDReg_tmp,STDAddr_tmp;
    STDReg_tmp = ANX7150_stdreg & 0x1f;
    STDAddr_tmp = ANX7150_stdaddr + 1;
    i = 0;
    while (i < STDReg_tmp)
    {
        AudioFormat = (ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,STDAddr_tmp ) & 0xF8) >> 3;
        ANX7150_edid_result.AudioChannel[i/3] = (ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,STDAddr_tmp) & 0x07) + 1;
        ANX7150_edid_result.AudioFormat[i/3] = AudioFormat;
        ANX7150_edid_result.AudioFs[i/3] = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(STDAddr_tmp + 1)) & 0x7f;

        if (AudioFormat == 1)
            ANX7150_edid_result.AudioLength[i/3] = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(STDAddr_tmp + 2)) & 0x07;
        else
            ANX7150_edid_result.AudioLength[i/3] = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(STDAddr_tmp + 2)) << 3;

        i = i + 3;
        STDAddr_tmp = STDAddr_tmp + 3;
    }
}

void ANX7150_Parse_VideoSTD()
{
    uint8 i,STDReg_tmp,STDAddr_tmp;
    uint8 SVD_ID[34];
    STDReg_tmp = ANX7150_stdreg & 0x1f;
    STDAddr_tmp = ANX7150_stdaddr + 1;
    i = 0;
    while (i < STDReg_tmp)
    {
        SVD_ID[i] = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,STDAddr_tmp) & 0x7F;
        //D("ANX7150_edid_result.SVD_ID[%.2x]=0x%.2x\n",(WORD)i,(WORD)ANX7150_edid_result.SVD_ID[i]);
        //if(ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,STDAddr_tmp) & 0x80)
        //    D(" Native mode");
        if (SVD_ID[i] == 1)
            ANX7150_edid_result.supported_640x480p_60Hz = 1;
        else if (SVD_ID[i] == 4)
            ANX7150_edid_result.supported_720p_60Hz = 1;
        else if (SVD_ID[i] == 19)
            ANX7150_edid_result.supported_720p_50Hz = 1;
        else if (SVD_ID[i] == 16)
            ANX7150_edid_result.supported_1080p_60Hz = 1;
        else if (SVD_ID[i] == 31)
            ANX7150_edid_result.supported_1080p_50Hz = 1;
        else if (SVD_ID[i] == 5)
            ANX7150_edid_result.supported_1080i_60Hz = 1;
        else if (SVD_ID[i] == 20)
            ANX7150_edid_result.supported_1080i_50Hz = 1;
        else if ((SVD_ID[i] == 2) ||(SVD_ID[i] == 3))
            ANX7150_edid_result.supported_720x480p_60Hz = 1;
        else if ((SVD_ID[i] == 6) ||(SVD_ID[i] == 7))
            ANX7150_edid_result.supported_720x480i_60Hz = 1;
        else if ((SVD_ID[i] == 17) ||(SVD_ID[i] == 18))
            ANX7150_edid_result.supported_576p_50Hz = 1;
        else if ((SVD_ID[i] == 21) ||(SVD_ID[i] == 22))
            ANX7150_edid_result.supported_576i_50Hz = 1;

        i = i + 1;
        STDAddr_tmp = STDAddr_tmp + 1;
    }
}

void ANX7150_Parse_SpeakerSTD()
{
    ANX7150_edid_result.SpeakerFormat = ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(ANX7150_stdaddr + 1)) ;
}

void ANX7150_Parse_VendorSTD()
{
    //uint8 c;
    if ((ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(ANX7150_stdaddr + 1)) == 0x03)
            && (ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(ANX7150_stdaddr + 2)) == 0x0c)
            && (ANX7150_Read_EDID_uint8(ANX7150_ext_block_num/2,(ANX7150_stdaddr + 3)) == 0x00))
    {
        ANX7150_edid_result.is_HDMI = 1;
        //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        //ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c |ANX7150_SYS_CTRL1_HDMI);
    }
    else
    {
        ANX7150_edid_result.is_HDMI = 0;
        //ANX7150_i2c_read_p0_reg(ANX7150_SYS_CTRL1_REG, &c);
        //ANX7150_i2c_write_p0_reg(ANX7150_SYS_CTRL1_REG, c & (~ANX7150_SYS_CTRL1_HDMI));
    }
}

uint8 ANX7150_EDID_Checksum(uint8 block_number)
{
    uint8 i, real_checksum;
    uint8 edid_block_checksum;

    edid_block_checksum = 0;
    for (i = 0; i < 127; i ++)
    {
        if ((block_number / 2) * 2 == block_number)
            edid_block_checksum = edid_block_checksum + ANX7150_Read_EDID_uint8(block_number/2, i);
        else
            edid_block_checksum = edid_block_checksum + ANX7150_Read_EDID_uint8(block_number/2, i + 0x80);
    }
    edid_block_checksum = (~edid_block_checksum) + 1;
    // D("edid_block_checksum = 0x%.2x\n",(WORD)edid_block_checksum);
    if ((block_number / 2) * 2 == block_number)
        real_checksum = ANX7150_Read_EDID_uint8(block_number/2, 0x7f);
    else
        real_checksum = ANX7150_Read_EDID_uint8(block_number/2, 0xff);
    if (real_checksum == edid_block_checksum)
        return 1;
    else
        return 0;
}


void ANX7150_InitDDC_Read(uint8 devaddr, uint8 segmentpointer,
                          uint8 offset, uint8  access_num_Low,uint8 access_num_high)
{
    //Write slave device address
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_ADDR_REG, devaddr);
    // Write segment address
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_SEGADDR_REG, segmentpointer);
    //Write offset
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_OFFADDR_REG, offset);
    //Write number for access
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACCNUM0_REG, access_num_Low);
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACCNUM1_REG, access_num_high);
    //Clear FIFO
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x05);
    //EDDC sequential Read
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, 0x04);
}

uint8 ANX7150_Read_EDID_uint8(uint8 segmentpointer,uint8 offset)
{
    /*uint8 c;
    ANX7150_InitDDC_Read(0xa0, segmentpointer, offset, 0x01, 0x00);
     ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFOCNT_REG, &c);
     while(c==0)
    	ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, &c);
    return c;*/

    return ANX7150_EDID_Buf[offset];
}



void ANX7150_Timer_Process ()
{
	//D("timer_slot = 0x%.2x\n", timer_slot);
    if (timer_slot == 0)
    {
        ANX7150_Timer_Slot1();
    }
    else if (timer_slot == 1)
    {
        ANX7150_Timer_Slot2();
    }
    else if (timer_slot == 2)
    {
        ANX7150_Timer_Slot3();
    }

	timer_slot++;

    if (timer_slot == 3)
        timer_slot = 0;
}

int ANX7150_Chip_Located(void)
{
    uint8 i;
    uint8 c, d1, d2;

    for (i=0; i<10; i++)
    {
        c = ANX7150_i2c_read_p0_reg(ANX7150_DEV_IDL_REG, &d1);
        if (c) continue;

        c = ANX7150_i2c_read_p0_reg(ANX7150_DEV_IDH_REG, &d2);
        if (c) continue;

        if (d1 == 0x50 && d2 == 0x71)
        {
            D("ANX7150 detected!\n");
            return 0;
        }
    }
	
    D("device not detected");
    return -1;
}




/*added by gerard.zhu*/
/*DDC operate start*/


/*Function name  :ANX7150_DDC_Parameter_Validity()*/
/*Function  :Judge the validity for input parameter*/
/*Parameter  :   Addr,length*/
/*Return  :Judge result is DDC_Data_Addr_Err,DDC_Length_Err,DDC_NO_Err*/
uint8 ANX7150_DDC_Parameter_Validity(uint8 *Addr,WORD length)
{
    if (Addr == NULL)
    {
        return DDC_Data_Addr_Err;
    }
    else if (length > DDC_Max_Length)
    {
        return DDC_Length_Err;
    }
    else
    {
        return DDC_NO_Err;
    }

}

/*Function name :ANX7150_DDC_Set_Address()*/
/*Function  :Set address for DDC device*/
/*Parameter :   ddc_address,ddc_type*/
/*Return  :None*/
void ANX7150_DDC_Set_Address(ANX7150_DDC_Addr ddc_address, ANX7150_DDC_Type ddc_type)
{
    /*set DDC channel slave device address*/
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_ADDR_REG, ddc_address.dev_addr);
    /*set DDC channel slave segment address,when ddc type is edid*/
    if (ddc_type == DDC_Edid)
        ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_SEGADDR_REG, ddc_address.sgmt_addr);
    /*set DDC channel slave offset address*/
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_SLV_OFFADDR_REG, ddc_address.offset_addr);
}

/*Function name :ANX7150_DDC_Set_Number()*/
/*Function  :Set number for DDC data access*/
/*Parameter :   length*/
/*Return  :None*/
void ANX7150_DDC_Set_Number(WORD length)
{
    uint8 length_low,length_high;

    D("!!!!DDC_data_number :%x\n",length);
    length_high = (uint8)((length >> 8) & 0xff);
    length_low = (uint8)(length & 0xff);
    /*set number of uint8s to DDC channel*/
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACCNUM0_REG, length_low);
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACCNUM1_REG, length_high);
}

/*Function name :ANX7150_DDC_Command()*/
/*Function  :Send command to DDC*/
/*Parameter :   DDC_command*/
/*Return  :None*/
void ANX7150_DDC_Command(uint8 DDC_command)
{
    /*set DDC command*/
    ANX7150_i2c_write_p0_reg(ANX7150_DDC_ACC_CMD_REG, DDC_command);

}

/*Function name :ANX7150_DDC_Check_Status*/
/*Function  :Check DDC status or report information of error*/
/*Parameter :   DDC_status_need_type,status_uint8*/
/*Return  :always is 0 if DDC_status_need_status is "report",or return DDC status uint8 value
if DDC_status_need_type is "Judge"*/
uint8 ANX7150_DDC_Check_Status(ANX7150_DDC_Status_Check_Type DDC_status_need_type,
                               uint8 status_uint8)
{
    uint8 DDC_status,i,j;
    char *status[8] =
    {
        "!!!!DDC____An Error is Occurred!\n",
        "!!!!DDC____channel is accessed by an external device!\n",
        "!!!!DDC____Fifo is Full!\n",
        "!!!!DDC____Fifo is Empty!\n",
        "!!!!DDC____No Acknowledge detection!\n",
        "!!!!DDC____Fifo is being read!\n",
        "!!!!DDC____Fifo is being written!\n",
    };

    ANX7150_i2c_read_p0_reg(ANX7150_DDC_CHSTATUS_REG, &DDC_status);

    if (DDC_status_need_type == report)
    {
        for (i= 0,j=7; i < 8; i++,j--)
        {
            if (DDC_status & (0x01 << i))
            {
                D("%s",status[j]);
            }
        }
        return 0;
    }
    else
    {
        return ((DDC_status >> status_uint8) & 0x01 );
    }
}

/*Function name :ANX7150_DDC_Count_Compare()*/
/*Function :Check status for access count*/
/*Parameter :length*/
/*Return  :0 if check success,1 if check fail*/
uint8 ANX7150_DDC_Count_Compare(uint8 length)
{
    uint8 Fifo_Count;
    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFOCNT_REG, &Fifo_Count);
    return ((Fifo_Count & 0x01f) == length ? 0 : 1);
}

/*Function name:    ANX7150_DDC_Read()*/
/*Function :    read data from DDC channel*/
/*Parameter :   ddc_address,DDC_data,length,DDC_type*/
/*Return  :Read reslut,ex.DDC_NO_Err,DDC_Status_Err*/
uint8 ANX7150_DDC_Read (ANX7150_DDC_Addr ddc_address, const uint8 *DDC_data,
                        WORD length, ANX7150_DDC_Type DDC_type)
{
    uint8 DDC_Err,fifo_read_data_compare;
    int data_cnt;
    int fifo_data_cnt;
    uint8 *Fifo_Read_P;

    DDC_Err = DDC_NO_Err;
    fifo_read_data_compare = 0;
    fifo_data_cnt = length;
    Fifo_Read_P = (uint8 *)DDC_data;

    /*Judge validity for read address and length*/
    /*if (DDC_Err = ANX7150_DDC_Parameter_Validity(Fifo_Read_P, length))
        return (DDC_Err);
    */ 																																		//update
    if (DDC_Err != ANX7150_DDC_Parameter_Validity(Fifo_Read_P, length))		//update
        return (ANX7150_DDC_Parameter_Validity(Fifo_Read_P, length));

    /*set DDC address*/
    ANX7150_DDC_Set_Address(ddc_address, DDC_type);
    /*set number for DDC read*/
    ANX7150_DDC_Set_Number(length);
    /*send "clear DDC fifo" command*/
    ANX7150_DDC_Command((uint8)Clear_DDC_Fifo);

    /*check DDC channel status*/
    if (!ANX7150_DDC_Check_Status(Judge, DDC_Error_uint8) &&
            !ANX7150_DDC_Check_Status(Judge, DDC_Occup_uint8) &&
            !ANX7150_DDC_Check_Status(Judge, DDC_No_Ack_uint8))
    {
        /*send "sequential uint8 read"command if check success*/
        ANX7150_DDC_Command((uint8)Sequential_uint8_Read);
        /*delay*/
        DRVDelayMs(DDC_Read_Delay);
    }
    else
    {
        ANX7150_DDC_Check_Status(report, 0);
        return DDC_Status_Err;
    }

    /*read DDC fifo data*/
    do
    {
        /*read data from fifo if length <= DDC fifo depth*/
        if (fifo_data_cnt <= DDC_Fifo_Depth)
        {

            fifo_read_data_compare = fifo_data_cnt;

            /*check uint8 DDC_Progress of DDC status,fifo count*/
            if (!ANX7150_DDC_Check_Status(Judge, DDC_Progress_uint8) &&
                    !ANX7150_DDC_Count_Compare(fifo_read_data_compare))
            {
                data_cnt = fifo_data_cnt;
                while (data_cnt--)
                {
                    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, Fifo_Read_P++);
                }
            }
            else
            {
                ANX7150_DDC_Check_Status(report, null);
                return DDC_Status_Err;
            }
        }
        /*read data from fifo if length >DDC fifo depth*/
        else
        {
            /*check uint8 DDC_Progress of DDC status,Fifo_full*/
            if (ANX7150_DDC_Check_Status(Judge, DDC_Progress_uint8) &&
                    ANX7150_DDC_Check_Status(Judge, DDC_Fifo_Full_uint8))
            {
                data_cnt = DDC_Fifo_Depth;
                while (data_cnt--)
                {
                    ANX7150_i2c_read_p0_reg(ANX7150_DDC_FIFO_ACC_REG, Fifo_Read_P++);
                }
            }
            else
            {
                ANX7150_DDC_Check_Status(report, null);
                return DDC_Status_Err;
            }

        }
        fifo_data_cnt -= DDC_Fifo_Depth;
    }
    while (fifo_data_cnt > 0);

    return DDC_Err;
}

/*Function name:    ANX7150_DDC_Write()*/
/*Function :    write data to DDC channel*/
/*Parameter :   ddc_address,DDC_data,length,DDC_type*/
/*Return  :write reslut,ex.DDC_NO_Err,DDC_Status_Err*/
/*					//wen
uint8 ANX7150_DDC_Write (ANX7150_DDC_Addr ddc_address, const uint8 *DDC_data,
    WORD length, ANX7150_DDC_Type DDC_type)
{
    uint8 DDC_Err,fifo_write_data_compare;
    int data_cnt;
    int fifo_data_cnt;
    uint8 *Fifo_Write_P;

    DDC_Err = DDC_NO_Err;
    fifo_write_data_compare = 0;
    fifo_data_cnt = length;
    Fifo_Write_P = DDC_data;

    //Judge validity for write address and length
    if (DDC_Err = ANX7150_DDC_Parameter_Validity(Fifo_Write_P, length))
        return (DDC_Err);

    //set DDC address
    ANX7150_DDC_Set_Address(ddc_address, DDC_type);
    //set number for DDC write
    ANX7150_DDC_Set_Number(length);
    //send "clear DDC fifo" command
    ANX7150_DDC_Command((uint8)Clear_DDC_Fifo);

    //check DDC channel status//
    if (!ANX7150_DDC_Check_Status(Judge, DDC_Error_uint8) &&
        !ANX7150_DDC_Check_Status(Judge, DDC_Occup_uint8) &&
        !ANX7150_DDC_Check_Status(Judge, DDC_No_Ack_uint8)){

            do {
                //write data to fifo if data length <= DDC Fifo Depth/
                if (fifo_data_cnt <= DDC_Fifo_Depth){

                    fifo_write_data_compare = fifo_data_cnt;

                    while (fifo_data_cnt--){
                        //write data to DDC fifo
                        ANX7150_i2c_write_p0_reg(ANX7150_DDC_FIFO_ACC_REG, *(Fifo_Write_P++));
                    }
                    //send "sequential uint8 write"command after finish writing data to fifo
                    ANX7150_DDC_Command((uint8)Sequential_uint8_Write);
                    //delay
                    DRVDelayMs(DDC_Write_Delay);
                    //check DDC status when data length <= DDC Fifo Depth
                    if (ANX7150_DDC_Check_Status(Judge, DDC_Progress_uint8) ||
                        ANX7150_DDC_Count_Compare(fifo_write_data_compare)){
                        ANX7150_DDC_Check_Status(report, null);
                        return DDC_Status_Err;
                    }
                }
                //write data to fifo if data length > DDC Fifo Depth
                else{
                    data_cnt = DDC_Fifo_Depth;
                    while(data_cnt--){
                        ANX7150_i2c_write_p0_reg(ANX7150_DDC_FIFO_ACC_REG, *(Fifo_Write_P++));
                    }
                    //send "sequential uint8 write"command after finish writing data to fifo
                    ANX7150_DDC_Command((uint8)Sequential_uint8_Write);
                    //delay
                    DRVDelayMs(DDC_Write_Delay);
                    //check DDC status when data length > DDC Fifo Depth
                    if (!ANX7150_DDC_Check_Status(Judge, DDC_Progress_uint8) ||
                        !ANX7150_DDC_Check_Status(Judge, DDC_Fifo_Full_uint8)){
                        ANX7150_DDC_Check_Status(report, null);
                        return DDC_Status_Err;
                    }

                    fifo_data_cnt -= DDC_Fifo_Depth;
                }

            }while(fifo_data_cnt > 0);
    }
    else{
        ANX7150_DDC_Check_Status(report, null);
        return DDC_Status_Err;
    }

    return DDC_Err;
}
*/

void ANX7150_API_System_Config()
{

    ANX7150_API_Video_Config(g_video_format,input_pixel_clk_1x_repeatition);
    ANX7150_API_Packets_Config(ANX7150_avi_sel | ANX7150_audio_sel);
    if (s_ANX7150_packet_config.packets_need_config & ANX7150_avi_sel)
        ANX7150_API_AVI_Config(	0x00,source_ratio,null,null,null,null,null,null,null,null,null,null,null);
    if (s_ANX7150_packet_config.packets_need_config & ANX7150_audio_sel)
        ANX7150_API_AUD_INFO_Config(null,null,null,null,null,null,null,null,null,null);
    ANX7150_API_AUD_CHStatus_Config(null,null,null,null,null,null,null,null,null,g_audio_format);
}

void ANX7150_API_Video_Config(uint8 video_id,uint8 input_pixel_rpt_time)
{
    ANX7150_video_timing_id = video_id;
    ANX7150_in_pix_rpt = input_pixel_rpt_time;
}

void ANX7150_API_Packets_Config(uint8 pkt_sel)
{
    s_ANX7150_packet_config.packets_need_config = pkt_sel;
}

void ANX7150_API_AUD_CHStatus_Config(uint8 MODE,uint8 PCM_MODE,uint8 SW_CPRGT,uint8 NON_PCM,
                                     uint8 PROF_APP,uint8 CAT_CODE,uint8 CH_NUM,uint8 SOURCE_NUM,uint8 CLK_ACCUR,uint8 Fs)
{
    //MODE: 0x00 = PCM Audio
    //PCM_MODE: 0x00 = 2 audio channels without pre-emphasis;
    //0x01 = 2 audio channels with 50/15 usec pre-emphasis;
    //SW_CPRGT: 0x00 = copyright is asserted;
    // 0x01 = copyright is not asserted;
    //NON_PCM: 0x00 = Represents linear PCM
    //0x01 = For other purposes
    //PROF_APP: 0x00 = consumer applications;
    // 0x01 = professional applications;

    //CAT_CODE: Category code
    //CH_NUM: 0x00 = Do not take into account
    // 0x01 = left channel for stereo channel format
    // 0x02 = right channel for stereo channel format
    //SOURCE_NUM: source number
    // 0x00 = Do not take into account
    // 0x01 = 1; 0x02 = 2; 0x03 = 3
    //CLK_ACCUR: 0x00 = level II
    // 0x01 = level I
    // 0x02 = level III
    // else reserved;

    s_ANX7150_audio_config.i2s_config.Channel_status1 = (MODE << 7) | (PCM_MODE << 5) |
            (SW_CPRGT << 2) | (NON_PCM << 1) | PROF_APP;
    s_ANX7150_audio_config.i2s_config.Channel_status2 = CAT_CODE;
    s_ANX7150_audio_config.i2s_config.Channel_status3 = (CH_NUM << 7) | SOURCE_NUM;
    s_ANX7150_audio_config.i2s_config.Channel_status4 = (CLK_ACCUR << 5) | Fs;
    D("g_audio_format = 0x%02x \n", g_audio_format);
}

void ANX7150_API_AVI_Config(uint8 pb1,uint8 pb2,uint8 pb3,uint8 pb4,uint8 pb5,
                            uint8 pb6,uint8 pb7,uint8 pb8,uint8 pb9,uint8 pb10,uint8 pb11,uint8 pb12,uint8 pb13)
{
    s_ANX7150_packet_config.avi_info.pb_uint8[1] = pb1;
    s_ANX7150_packet_config.avi_info.pb_uint8[2] = pb2;
    s_ANX7150_packet_config.avi_info.pb_uint8[3] = pb3;
    s_ANX7150_packet_config.avi_info.pb_uint8[4] = pb4;
    s_ANX7150_packet_config.avi_info.pb_uint8[5] = pb5;
    s_ANX7150_packet_config.avi_info.pb_uint8[6] = pb6;
    s_ANX7150_packet_config.avi_info.pb_uint8[7] = pb7;
    s_ANX7150_packet_config.avi_info.pb_uint8[8] = pb8;
    s_ANX7150_packet_config.avi_info.pb_uint8[9] = pb9;
    s_ANX7150_packet_config.avi_info.pb_uint8[10] = pb10;
    s_ANX7150_packet_config.avi_info.pb_uint8[11] = pb11;
    s_ANX7150_packet_config.avi_info.pb_uint8[12] = pb12;
    s_ANX7150_packet_config.avi_info.pb_uint8[13] = pb13;
}

void ANX7150_API_AUD_INFO_Config(uint8 pb1,uint8 pb2,uint8 pb3,uint8 pb4,uint8 pb5,
                                 uint8 pb6,uint8 pb7,uint8 pb8,uint8 pb9,uint8 pb10)
{
    s_ANX7150_packet_config.audio_info.pb_uint8[1] = pb1;
    s_ANX7150_packet_config.audio_info.pb_uint8[2] = pb2;
    s_ANX7150_packet_config.audio_info.pb_uint8[3] = pb3;
    s_ANX7150_packet_config.audio_info.pb_uint8[4] = pb4;
    s_ANX7150_packet_config.audio_info.pb_uint8[5] = pb5;
    s_ANX7150_packet_config.audio_info.pb_uint8[6] = pb6;
    s_ANX7150_packet_config.audio_info.pb_uint8[7] = pb7;
    s_ANX7150_packet_config.audio_info.pb_uint8[8] = pb8;
    s_ANX7150_packet_config.audio_info.pb_uint8[9] = pb9;
    s_ANX7150_packet_config.audio_info.pb_uint8[10] = pb10;
}

int ANX7150_API_DetectDevice(void)
{
    return ANX7150_Chip_Located();
}


void ANX7150_API_HDCP_ONorOFF(uint8 HDCP_ONorOFF)
{
    ANX7150_HDCP_enable = HDCP_ONorOFF;// 1: on;  0:off
}


void ANX7150_API_Set_AVMute()
{
    ANX7150_Set_AVMute();//wen
}

void ANX7150_API_Clean_HDCP()
{
    ANX7150_Clean_HDCP();
}
void HDMI_Set_CallBack(pCallBack callbcak)//Set CPU prepare for HDMI output
{
    ANX7150_callback = callbcak;
}
void HDMI_Set_Auto_Detect(uint8 mode) //Set HDMI work mode for ANX7150
{
    HDMI_Mode_Auto_Manual=mode;
}
uint8 HDMI_Get_Auto_Detect(void) //ANX7150 call this to check work mode
{
    return HDMI_Mode_Auto_Manual;
}

void  HDMI_Set_Lowpower(uint8 lowpower) //CPU set the lowpower mode
{
    HDMI_Lowpower_Mode = lowpower;
}
uint8  HDMI_Get_Lowpower( void) //ANX7150 call this to check lowpower
{
    return HDMI_Lowpower_Mode;
}
void  HDMI_Set_Video_Format(uint8 video_format) //CPU set the lowpower mode
{
    switch (video_format)
    {
        case 0:
            g_video_format = ANX7150_V1280x720p_50Hz;
            break;
        default:
            g_video_format = ANX7150_V720x576p_50Hz_4x3;
            break;
    }
    ANX7150_system_config_done = 0;
}
void  HDMI_Set_Audio_Fs( uint8 audio_fs) //ANX7150 call this to check lowpower
{
    g_audio_format = audio_fs;
    ANX7150_system_config_done = 0;
}

int  HDMI_System_Init(void) //CPU set the lowpower mode
{
	int ret;
#if defined(CONFIG_BOARD_NX7005)||defined(CONFIG_BOARD_TD05D6) 
	pca955x_gpio_direction_output(PCA955X_Pin10,GPIO_LOW);	
	mdelay(500);
	pca955x_gpio_direction_output(PCA955X_Pin10,GPIO_HIGH);
#endif
	
    ret = ANX7150_API_DetectDevice();
	if(ret < 0){
		E("ANX7150_API_DetectDevice err!\n");
		return -1;
	}
	
	HDMI_Set_Audio_Fs(2);
    ANX7150_API_Initial();
    ANX7150_API_HDCP_ONorOFF(0);    //  cwz 1 : open for HCDP test, 0: close

	return 0;
}

void HDMI_System_Task(void)
{	
    if ( ANX7150_parse_edid_done == 1 &&
		 ANX7150_system_config_done == 0){
        //system should config all the parameters here
        D("******** system config here.\n");
        ANX7150_API_System_Config();
        ANX7150_system_config_done = 1;
    }
    ANX7150_Task();
}

