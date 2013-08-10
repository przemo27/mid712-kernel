#ifndef _ANX7150_H
#define _ANX7150_H
#include <linux/ioctl.h>

/*******************HDMI ioctl CMD************************/
#define HDMI_MAGIC 'H'

#define ANX7150_ENABLE              _IOW(HDMI_MAGIC, 1, int)
#define ANX7150_DISABLE             _IOW(HDMI_MAGIC, 2, int)
#define HDMI_OUTPUT_ENABLE          _IOW(HDMI_MAGIC, 3, int)
#define HDMI_OUTPUT_DISABLE         _IOW(HDMI_MAGIC, 4, int)
#define HDMI_OUTPUT_RESOLUTION      _IOW(HDMI_MAGIC, 5, int)
#define HDMI_CONNECT_STATUS         _IOR(HDMI_MAGIC, 6, int)

/*******************HDMI CONNECT STATUS********************/
#define UnPlug  0
#define Plug    1

/*******************HDMI OUTPUT RESOLUTION*****************/
#define HDMI_1280x720    0
#define HDMI_720x576     1

/*******************RK28 OUTPUT STATUS*********************/
#define LCD     0
#define HDMI    1

int anx7150_get_output_status(void);
int anx7150_get_output_resolution(void);

#endif
