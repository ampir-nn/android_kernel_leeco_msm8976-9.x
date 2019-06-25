/*
 *  alsprox_common.h - definitions for all ALS&Prox sensors
 *
 *  Copyright (C) 2010 Yulong Tech. Co., Ltd.
 *  Jay.HF <huangfujie@yulong.com>
 *  Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ALSPROX_COMMON_H__
#define __ALSPROX_COMMON_H__

#include <linux/ioctl.h>

#define ALSPROX_DEVICE_NAME          "yl_alsprox_sensor"
#define ALSPROX_DRIVER_NAME          "yl_alsprox_sensor"
#define ALSPROX_INPUT_NAME           "yl_alsprox_input"

#define ALSPROX_IOCTL_MAGIC          (0xCF)
#define ALSPROX_IOCTL_ALS_ON         _IOW(ALSPROX_IOCTL_MAGIC, 1, unsigned long)
#define ALSPROX_IOCTL_ALS_OFF        _IOW(ALSPROX_IOCTL_MAGIC, 2, unsigned long)
#define ALSPROX_IOCTL_PROX_ON        _IOW(ALSPROX_IOCTL_MAGIC, 3, unsigned long)
#define ALSPROX_IOCTL_PROX_OFF       _IOW(ALSPROX_IOCTL_MAGIC, 4, unsigned long)
#define ALSPROX_IOCTL_PROX_OFFSET    _IOW(ALSPROX_IOCTL_MAGIC, 5, unsigned long)
#define ALSPROX_IOCTL_PROX_CALIBRATE _IOW(ALSPROX_IOCTL_MAGIC, 6, unsigned long)
#define ALSPROX_IOCTL_PHONE_STATE    _IOW(ALSPROX_IOCTL_MAGIC, 7, unsigned long)

#define ALSPROX_IOCTL_ALS_ON32         _IOW(ALSPROX_IOCTL_MAGIC, 1, unsigned int)
#define ALSPROX_IOCTL_ALS_OFF32        _IOW(ALSPROX_IOCTL_MAGIC, 2, unsigned int)
#define ALSPROX_IOCTL_PROX_ON32        _IOW(ALSPROX_IOCTL_MAGIC, 3, unsigned int)
#define ALSPROX_IOCTL_PROX_OFF32       _IOW(ALSPROX_IOCTL_MAGIC, 4, unsigned int)
#define ALSPROX_IOCTL_PROX_OFFSET32    _IOW(ALSPROX_IOCTL_MAGIC, 5, unsigned int)
#define ALSPROX_IOCTL_PROX_CALIBRATE32 _IOW(ALSPROX_IOCTL_MAGIC, 6, unsigned int)
#define ALSPROX_IOCTL_PHONE_STATE32    _IOW(ALSPROX_IOCTL_MAGIC, 7, unsigned int)


struct prox_offset {
	unsigned short key;
	unsigned short x; /* hi  */
	unsigned short y; /* lo  */
	unsigned short z; /* avg */
	unsigned short offset;
};
struct prox_status {
	unsigned prox_cal; /*0: no calibrate; 1:success; 2: fail*/
	unsigned prox_update; /*0: no need calibrate; 1: need calibrate*/
};

#endif
