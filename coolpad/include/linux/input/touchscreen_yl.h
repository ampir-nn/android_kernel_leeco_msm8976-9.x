/**
 *
 * Copyright (c) 2012-2017  YULONG Company
 *
 * PROPRIETARY RIGHTS of YULONG Company are involved in the
 * subject matter of this material.  All manufacturing, reproduction, use,
 * and sales rights pertaining to this subject matter are governed by the
 * license agreement.  The recipient of this software implicitly accepts
 * the terms of the license.
 *
 */

#ifndef _TOUCHSCREEN_YL_H
#define _TOUCHSCREEN_YL_H

enum touch_id_type {
	ID_MAIN			= 0,
	ID_SUB			= 1,
	ID_INVALID		= 2,
};

enum touch_mode_type {
	MODE_INVALID		= 0,
	MODE_NORMAL		= 1,
	MODE_HANDWRITE		= 2,
	MODE_GLOVE		= 3,
	MODE_GLOVE_FORCE	= 4,
	MODE_NORMAL_WINDOW	= 5,
	MODE_GLOVE_WINDOW	= 6,
	MODE_MAX		= 7,
};

enum touch_orientation_type {
	OREITATION_INVALID	= 0,
	OREITATION_0		= 1,
	OREITATION_90		= 2,
	OREITATION_180		= 3,
	OREITATION_270		= 4,
};

struct touchscreen_funcs {
	enum touch_id_type touch_id;
	/* 1: capacitive screen; 2: resistive screen */
	int touch_type;
	int (*active)(void);
	int (*firmware_need_update)(void);
	int (*firmware_do_update)(void);
	int (*need_calibrate)(void);
	int (*calibrate)(void);
	int (*get_firmware_version)(char *);
	int (*reset_touchscreen)(void);
	/* touch input mode: "handwrite" or "normal" */
	enum touch_mode_type (*get_mode)(void);
	int (*set_mode)(enum touch_mode_type);
	enum touch_orientation_type (*get_orientation)(void);
	int (*set_orientation)(enum touch_orientation_type);
	int (*read_regs)(char *);
	int (*write_regs)(const char *);
	int (*debug)(int);
	int (*get_rawdata)(char *buf);
	int (*get_vendor)(char *);
	int (*get_wakeup_gesture)(char *);
	int (*gesture_ctrl)(const char *);
	int (*ftsscaptest)(char *);
};

struct virtual_keys_button {
	int x;
	int y;
	int width;
	int height;
	unsigned int code;
	unsigned int key_status;
};

struct tw_platform_data {
	int  (*init)(void);
	void (*release)(void);
	int  (*power) (int on);
	int  (*reset) (int ms);
	int  (*suspend)(void);
	int  (*resume)(void);
	unsigned char (*get_id_pin)(void);
	struct virtual_keys_button *buttons;
	int nbuttons;
	unsigned long  irqflag;
	unsigned int gpio_irq;
	/* unsigned int gpio_reset; */
	int screen_x;
	int screen_y;
	int key_debounce;
#if CONFIG_OF
	unsigned int gpio_reset;
	unsigned int pwr_en;
	unsigned int sleep_pwr_en;
//	const char *ts_vcc_i2c;
//	const char *ts_vdd;
	struct regulator  *ts_vcc_i2c;
	struct regulator  *ts_vdd;
	unsigned int reset_delay;
	bool i2c_pull_up;
	bool resume_in_workqueue;
#endif
};

extern int touchscreen_set_ops(struct touchscreen_funcs *ops);

#endif /* _TOUCHSCREEN_YL_H */
