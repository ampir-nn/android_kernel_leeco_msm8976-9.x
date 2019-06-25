/* drivers/input/touchscreen/gt9xx.h
 *
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Linux Foundation chooses to take subject only to the GPLv2 license
 * terms, and distributes only under these terms.
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef _GOODIX_GT9XX_H_
#define _GOODIX_GT9XX_H_

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>

#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>

#define GOODIX_MAX_CFG_GROUP	6
struct goodix_ts_platform_data {
	int irq_gpio;
	u32 irq_gpio_flag;
	int reset_gpio;
	u32 reset_gpio_flag;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	bool no_force_update;
	bool i2c_pull_up;
	size_t config_data_len[GOODIX_MAX_CFG_GROUP];
	u8 *config_data[GOODIX_MAX_CFG_GROUP];
	u8 pwr_ctrl_en;
	u8 sleep_pwr_en;
};
struct goodix_ts_data {
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev  *input_dev;
	struct goodix_ts_platform_data *pdata;
	struct hrtimer timer;
	struct workqueue_struct *goodix_wq;
	struct work_struct	work;
	s32 irq_is_disabled;
	s32 use_irq;
	u16 abs_x_max;
	u16 abs_y_max;
	u8  max_touch_num;
	u8  int_trigger_type;
	u8  green_wake_mode;
	u8  chip_type;
	u8 *config_data;
	u8  enter_update;
	u8  gtp_is_suspend;
	u8  gtp_rawdiff_mode;
	u8  gtp_cfg_len;
	u8  fixed_cfg;
	u8  esd_running;
	u8  fw_error;
	struct regulator *avdd;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

extern u16 show_len;
extern u16 total_len;

/***************************PART1:ON/OFF define*******************************/
#define GTP_CUSTOM_CFG        1
#define GTP_CHANGE_X2Y        0
#define GTP_DRIVER_SEND_CFG   1
#define GTP_HAVE_TOUCH_KEY    1

#define GTP_POWER_CTRL_SLEEP  0
#define GTP_POWER_CTRL_EN     1     /*add by yulong 2013.11.14*/

#define GTP_AUTO_UPDATE       0     /* auto updated by .bin file as default */
#define GTP_HEADER_FW_UPDATE  1     /* auto updated by head_fw_array in gt9xx_firmware.h, function together with GTP_AUTO_UPDATE*/

#define GTP_CREATE_WR_NODE    1
#define GTP_ESD_PROTECT       1

#define GTP_ICS_SLOT_REPORT   0

#define GTP_WITH_PEN          0
#define GTP_SLIDE_WAKEUP      0
#define GTP_DBL_CLK_WAKEUP    0     /* double-cli ck wakeup, function together with GTP_SLIDE_WAKEUP*/

#define GTP_DEBUG_ON          1
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

/*************************** PART2:TODO define **********************************/
/* STEP_1(REQUIRED): Define Configuration Information Group(s)
 * Sensor_ID Map:
 * sensor_opt1 sensor_opt2 Sensor_ID
    GND         GND         0
    VDDIO       GND         1
    NC          GND         2
    GND         NC/300K     3
    VDDIO       NC/300K     4
    NC          NC/300K     5
*/
/* TODO: define your own default or for Sensor_ID == 0 config here.
* The predefined one is just a sample config, which is not suitable for your tp in most cases.
*/
#define CTP_CFG_GROUP1 {\
	0x4D, 0xE0, 0x01, 0x56, 0x03, 0x05, 0x05, 0x01, 0x01, 0x0F,\
	0x1E, 0x0F, 0x50, 0x3C, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8A, 0x2A, 0x0B,\
	0x49, 0x46, 0x2E, 0x0B, 0x00, 0x00, 0x00, 0x01, 0x03, 0x1D,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x1E, 0x50, 0x94, 0xC5, 0x02, 0x05, 0x00, 0x00, 0x04,\
	0xF4, 0x21, 0x00, 0xCD, 0x28, 0x00, 0xAC, 0x31, 0x00, 0x93,\
	0x3B, 0x00, 0x7D, 0x48, 0x00, 0x7D, 0x10, 0x30, 0x50, 0x00,\
	0xF0, 0x50, 0x3C, 0xFF, 0xFF, 0x27, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x16, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08,\
	0x06, 0x04, 0x02, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0x00, 0x1D, 0x1E, 0x1F, 0x20, 0x22, 0x24, 0x25, 0x26,\
	0x28, 0x29, 0x00, 0x01, 0x02, 0x04, 0x06, 0x07, 0x08, 0x09,\
	0x0A, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,\
	0xFF, 0xFF, 0xFF, 0xFF, 0xF2, 0x01\
}

/* TODO: define your config for Sensor_ID == 1 here, if needed*/
#define CTP_CFG_GROUP2 {\
    }
/* TODO: define your config for Sensor_ID == 2 here, if needed*/
#define CTP_CFG_GROUP3 {\
    }

/* TODO: define your config for Sensor_ID == 3 here, if needed*/
#define CTP_CFG_GROUP4 {\
    }

/* TODO: define your config for Sensor_ID == 4 here, if needed*/
#define CTP_CFG_GROUP5 {\
    }

/* TODO: define your config for Sensor_ID == 5 here, if needed*/
#define CTP_CFG_GROUP6 {\
    }
/*add start  yulong 2013.11.14*/
struct touch_panel_info {
	unsigned char  tp_id;     /* touch panel factory id */
	unsigned char *tp_name;   /* touch panel factory name */
};
extern bool gtp_debug_on;  	  /*use with GTP_DEBUG_ON for GTP_DEBGU()*/
/*add end yulong 2013.11.14*/
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

/* STEP_3(optional): Specify your special config info if needed*/
#if GTP_CUSTOM_CFG
	#if defined(CONFIG_BOARD_CP3600L)
		#define GTP_MAX_HEIGHT   960
		#define GTP_MAX_WIDTH    540
		#define GTP_INT_TRIGGER  1
	#elif defined(CONFIG_BOARD_CP8716)
		#define GTP_MAX_HEIGHT   1280
		#define GTP_MAX_WIDTH    720
		#define GTP_INT_TRIGGER  1
	#else
		#define GTP_MAX_HEIGHT   854
		#define GTP_MAX_WIDTH    480
		#define GTP_INT_TRIGGER  1/*IC: 0: Rising 1: Falling*/
	#endif
#else
	#define GTP_MAX_HEIGHT   4096
	#define GTP_MAX_WIDTH    4096
	#define GTP_INT_TRIGGER  1
#endif
#define GTP_MAX_TOUCH         5
#define GTP_ESD_CHECK_CIRCLE  2000      /* jiffy: ms*/

/* STEP_4(optional): If keys are available and reported as keys, config your key info here*/
#if GTP_HAVE_TOUCH_KEY
#if defined (CONFIG_BOARD_CP5560S)
#define GTP_KEY_TAB  {KEY_MENU, KEY_HOME, KEY_BACK}
#elif defined(CONFIG_BOARD_CP3600L)
#define GTP_KEY_TAB  {KEY_BACK, KEY_HOME, KEY_MENU}
#elif defined(CONFIG_BOARD_CP8716)
#define GTP_KEY_TAB  {KEY_MENU, KEY_HOME, KEY_BACK}
#else
#define GTP_KEY_TAB  {KEY_BACK, KEY_HOME, KEY_MENU}
#endif
#endif
/***************************PART3:OTHER define*********************************/
#define GTP_DRIVER_VERSION		"V2.1<2013/11/04>"
#define GTP_I2C_NAME			"Goodix-TS"
#define GTP_POLL_TIME			10     /* jiffy: ms*/
#define GTP_ADDR_LENGTH			2
#define GTP_CONFIG_MIN_LENGTH		186
#define GTP_CONFIG_MAX_LENGTH		240
#define FAIL				0
#define SUCCESS				1
#define SWITCH_OFF			0
#define SWITCH_ON			1

/* Registers define */
#define GTP_READ_COOR_ADDR		0x814E
#define GTP_REG_SLEEP			0x8040
#define GTP_REG_SENSOR_ID		0x814A
#define GTP_REG_CONFIG_DATA		0x8047
#define GTP_REG_VERSION			0x8140

#define RESOLUTION_LOC			3
#define TRIGGER_LOC			8

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

/*****************************End of Part III********************************/

/* Log define */
#define GTP_INFO(fmt,arg...)           printk(KERN_NOTICE "<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk(KERN_NOTICE "<<-GTP-ERROR->> "fmt"\n",##arg)

#define GTP_DEBUG(fmt, arg...)	       do {\
					if (GTP_DEBUG_ON && gtp_debug_on) {\
					printk(KERN_NOTICE "<<-GTP-DEBUG->> [%d]"fmt"\n",\
					__LINE__, ##arg); } \
					} while (0)

#define GTP_DEBUG_ARRAY(array, num)   do {\
					s32 i; \
					u8 *a = array; \
					if (GTP_DEBUG_ARRAY_ON) {\
						printk("<<-GTP-DEBUG-ARRAY->>\n");\
						for (i = 0; i < (num); i++) { \
						printk("%02x   ", (a)[i]);\
						if ((i + 1) % 10 == 0) { \
							printk("\n");\
						} \
					} \
					printk("\n");\
					} \
					} while (0)

#define GTP_DEBUG_FUNC()	do {\
	if (GTP_DEBUG_FUNC_ON)\
		printk(KERN_NOTICE "<<-GTP-FUNC->> Func:%s@Line:%d\n",\
					__func__, __LINE__);\
	} while (0)

#define GTP_SWAP(x, y)		do {\
					typeof(x) z = x;\
					x = y;\
					y = z;\
				} while (0)
#endif /* _GOODIX_GT9XX_H_ */
