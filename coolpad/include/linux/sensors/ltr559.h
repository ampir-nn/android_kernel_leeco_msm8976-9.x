/***********************************************************************
 **
 **  Copyright (C), 2013-2015, Yulong Tech. Co., Ltd.
 **  FileName:		ltr559.h
 **  Description:	Linux device driver for ltr559 ambient light and proximity sensors
 **  Author:		longjiang
 **  Version:		1.00
 **  Date:             	2013-09-25
 **
 ***********************************************************************/

#ifndef __LTR559_H__
#define __LTR559_H__

#include <linux/sensors/alsprox_common.h>

#define LTR559_DRIVER_NAME           	"ltr559"
#define LTR559_DEVICE_NAME            	ALSPROX_DEVICE_NAME
#define LTR559_INPUT_NAME               ALSPROX_INPUT_NAME

/*ioctl numbers*/
#define LTR559_IOCTL_ALS_ON           	ALSPROX_IOCTL_ALS_ON
#define LTR559_IOCTL_ALS_OFF           	ALSPROX_IOCTL_ALS_OFF
#define LTR559_IOCTL_PROX_ON          	ALSPROX_IOCTL_PROX_ON
#define LTR559_IOCTL_PROX_OFF          	ALSPROX_IOCTL_PROX_OFF
#define LTR559_IOCTL_PROX_OFFSET      	ALSPROX_IOCTL_PROX_OFFSET
#define LTR559_IOCTL_PROX_CALIBRATE    	ALSPROX_IOCTL_PROX_CALIBRATE

#define LTR559_IOCTL_ALS_ON32           	ALSPROX_IOCTL_ALS_ON32
#define LTR559_IOCTL_ALS_OFF32           	ALSPROX_IOCTL_ALS_OFF32
#define LTR559_IOCTL_PROX_ON32          	ALSPROX_IOCTL_PROX_ON32
#define LTR559_IOCTL_PROX_OFF32          	ALSPROX_IOCTL_PROX_OFF32
#define LTR559_IOCTL_PROX_OFFSET32      	ALSPROX_IOCTL_PROX_OFFSET32
#define LTR559_IOCTL_PROX_CALIBRATE32    	ALSPROX_IOCTL_PROX_CALIBRATE32

/* LTR-559 Registers */
#define LTR559_ALS_CONTR	0x80
#define LTR559_PS_CONTR		0x81
#define LTR559_PS_LED		0x82
#define LTR559_PS_N_PULSES	0x83
#define LTR559_PS_MEAS_RATE	0x84
#define LTR559_ALS_MEAS_RATE	0x85
#define LTR559_MANUFACTURER_ID	0x87

#define LTR559_INTERRUPT	0x8F
#define LTR559_PS_THRES_UP_0	0x90
#define LTR559_PS_THRES_UP_1	0x91
#define LTR559_PS_THRES_LOW_0	0x92
#define LTR559_PS_THRES_LOW_1	0x93

#define LTR559_ALS_THRES_UP_0	0x97
#define LTR559_ALS_THRES_UP_1	0x98
#define LTR559_ALS_THRES_LOW_0	0x99
#define LTR559_ALS_THRES_LOW_1	0x9A

#define LTR559_INTERRUPT_PERSIST 0x9E

/* 559's Read Only Registers */
#define LTR559_ALS_DATA_CH1_0	0x88
#define LTR559_ALS_DATA_CH1_1	0x89
#define LTR559_ALS_DATA_CH0_0	0x8A
#define LTR559_ALS_DATA_CH0_1	0x8B
#define LTR559_ALS_PS_STATUS	0x8C
#define LTR559_PS_DATA_0	0x8D
#define LTR559_PS_DATA_1	0x8E


/* Basic Operating Modes */
#define LTR559_MODE_ALS_ON_Range1	0x01   ///1 lux to 64k lux
#define LTR559_MODE_ALS_ON_Range2	0x05   ///0.5 lux to 32k lux
#define LTR559_MODE_ALS_ON_Range4	0x09   ///0.25 lux to 16k lux
#define LTR559_MODE_ALS_ON_Range8	0x0d   ///0.125 lux to 8k lux
#define LTR559_MODE_ALS_ON_Range48	0x19   ///0.02 lux to 1.3k lux
#define LTR559_MODE_ALS_ON_Range96	0x1d   ///0.01 lux to 600 lux
#define LTR559_MODE_ALS_StdBy		0x00

#define LTR559_MODE_PS_ON_Gain16		0x03
#define LTR559_MODE_PS_ON_Gain32		0x0b
#define LTR559_MODE_PS_ON_Gain64	0x0F
#define LTR559_MODE_PS_StdBy		0x00

#define LTR559_PS_RANGE1 	1
#define LTR559_PS_RANGE2	2
#define LTR559_PS_RANGE4 	4
#define LTR559_PS_RANGE8	8

#define LTR559_ALS_RANGE1_320	1
#define LTR559_ALS_RANGE2_64K 	2

/* Power On response time in ms */
#define LTR559_PON_DELAY	600
#define LTR559_WAKEUP_DELAY	10

/*the units is ms*/
#define LTR559_LIGHT_MEA_INTERVAL              200
#define LTR559_PROXIMITY_MEA_INTERVAL     200

#define LTR559_LIGHT_DEBOUNCE 10

#define LTR559_LIGHT_LUX_MIN    0
#define LTR559_LIGHT_LUX_MAX    65535

struct ltr559_platform_data {
	unsigned short prox_threshold_hi;
	unsigned short prox_threshold_lo;
	unsigned short prox_factory_threshold_hi;
	unsigned short prox_factory_threshold_lo;
	unsigned short interrupt_persist;
	unsigned short ps_led;
	unsigned short ps_n_pulses;
	unsigned short ps_meas_rate;
	unsigned short als_meas_rate;
	unsigned short ltr559_proximity_low_offset0;
	unsigned short ltr559_proximity_high_offset0;
	unsigned short ltr559_proximity_low_offset1;
	unsigned short ltr559_proximity_high_offset1;
	unsigned short ltr559_proximity_low_offset2;
	unsigned short ltr559_proximity_high_offset2;
	unsigned short ltr559_struct_noise_threshold;
	/* Interrupt */
	int gpio_int;
	unsigned int irq_gpio_flags;
	unsigned int prox_version;
};

#endif
