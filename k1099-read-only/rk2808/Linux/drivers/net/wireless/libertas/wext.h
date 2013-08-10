/**
  * This file contains definition for IOCTL call.
  */
#ifndef	_LBS_WEXT_H_
#define	_LBS_WEXT_H_

extern struct iw_handler_def lbs_handler_def;
extern struct iw_handler_def mesh_handler_def;
extern int lbs_do_ioctl(struct net_device *dev, struct ifreq *req, int cmd);

#endif
