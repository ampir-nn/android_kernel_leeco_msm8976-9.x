/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_FTS_H__
#define __LINUX_FTS_H__
 /*******************************************************************************
*
* File Name: focaltech.c
*
* Author: mshl
*
* Created: 2014-09
*
* Modify by mshl on 2015-04-30
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include <linux/input/touchscreen_yl_TBD.h>

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_DRIVER_INFO  "Qualcomm_Ver 1.1 2015-04-30"
#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06
#define FT6X36_ID       	0x36

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55

#define FTS_MAX_POINTS          10

#define FTS_WORKQUEUE_NAME	"fts_wq"

#define FTS_DEBUG_DIR_NAME	"fts_debug"

#define FTS_INFO_MAX_LEN		512
#define FTS_FW_NAME_MAX_LEN	50

#define FTS_REG_ID		0xA3
#define FTS_REG_FW_VER		0xA6
#define FTS_REG_FW_VENDOR_ID	0xA8
#define FTS_REG_POINT_RATE					0x88

#define FTS_FACTORYMODE_VALUE	0x40
#define FTS_WORKMODE_VALUE	0x00


#define FTS_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FTS_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= %s\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FTS_DRIVER_INFO, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)


#define FTS_DBG_EN 1
#if FTS_DBG_EN
#define FTS_DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define FTS_DBG(fmt, args...) 				do{}while(0)
#endif

//yulong add
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;


enum support_gesture_e {
    TW_SUPPORT_NONE_SLIDE_WAKEUP = 0x0,
    TW_SUPPORT_UP_SLIDE_WAKEUP = 0x1,
    TW_SUPPORT_DOWN_SLIDE_WAKEUP = 0x2,
    TW_SUPPORT_LEFT_SLIDE_WAKEUP = 0x4,
    TW_SUPPORT_RIGHT_SLIDE_WAKEUP = 0x8,
    TW_SUPPORT_E_SLIDE_WAKEUP = 0x10,
    TW_SUPPORT_O_SLIDE_WAKEUP = 0x20,
    TW_SUPPORT_W_SLIDE_WAKEUP = 0x40,
    TW_SUPPORT_C_SLIDE_WAKEUP = 0x80,
    TW_SUPPORT_M_SLIDE_WAKEUP = 0x100,
    TW_SUPPORT_DOUBLE_CLICK_WAKEUP = 0x200,
    TW_SUPPORT_L_SLIDE_WAKEUP = 0x400,
    TW_SUPPORT_S_SLIDE_WAKEUP = 0x800,
    TW_SUPPORT_V_SLIDE_WAKEUP = 0x1000,

    TW_SUPPORT_GESTURE_IN_ALL = (TW_SUPPORT_UP_SLIDE_WAKEUP |
                                 TW_SUPPORT_DOWN_SLIDE_WAKEUP |
                                 TW_SUPPORT_LEFT_SLIDE_WAKEUP |
                                 TW_SUPPORT_RIGHT_SLIDE_WAKEUP |
                                 TW_SUPPORT_E_SLIDE_WAKEUP |
                                 TW_SUPPORT_O_SLIDE_WAKEUP |
                                 TW_SUPPORT_W_SLIDE_WAKEUP |
                                 TW_SUPPORT_C_SLIDE_WAKEUP |
                                 TW_SUPPORT_M_SLIDE_WAKEUP |
                                 TW_SUPPORT_DOUBLE_CLICK_WAKEUP |
                                 TW_SUPPORT_L_SLIDE_WAKEUP |
                                 TW_SUPPORT_S_SLIDE_WAKEUP |
                                 TW_SUPPORT_V_SLIDE_WAKEUP)
};



/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

struct fts_Upgrade_Info
{
    u8 CHIP_ID;
    u8 TPD_MAX_POINTS;
    u8 AUTO_CLB;
    u16 delay_aa;						/*delay of write FT_UPGRADE_AA */
    u16 delay_55;						/*delay of write FT_UPGRADE_55 */
    u8 upgrade_id_1;					/*upgrade id 1 */
    u8 upgrade_id_2;					/*upgrade id 2 */
    u16 delay_readid;					/*delay of read id */
    u16 delay_erase_flash; 				/*delay of earse flash*/
};

struct fts_ts_platform_data {
	struct fts_Upgrade_Info info;
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	int (*power_init) (bool);
	int (*power_on) (bool);
};

struct ts_event {
	u16 au16_x[FTS_MAX_POINTS];	/*x coordinate */
	u16 au16_y[FTS_MAX_POINTS];	/*y coordinate */
	u8 au8_touch_event[FTS_MAX_POINTS];	/*touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[FTS_MAX_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
	u8 point_num;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct fts_ts_platform_data *pdata;
	struct work_struct 	touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FTS_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
	int touchs;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
};

/*******************************************************************************
* Static variables
*******************************************************************************/


/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//Function Switchs: define to open,  comment to close
#define FTS_UPGRADE_EN 1
#define FTS_GESTRUE_EN 1
#define FTS_APK_DEBUG
#define FTS_SYSFS_DEBUG
extern struct fts_Upgrade_Info fts_updateinfo_curr;
extern struct i2c_client *fts_i2c_client;
extern struct fts_ts_data *fts_wq_data;
extern struct input_dev *fts_input_dev;

static DEFINE_MUTEX(i2c_rw_access);

//Getstre functions
extern int fts_Gesture_init(struct input_dev *input_dev);
extern int fts_read_Gestruedata(void);
extern int fts_gesture_ctrl(const char*  gesture_buf);
extern int fts_get_wakeup_gesture(char*  gesture);

//upgrade functions
extern void fts_update_fw_vendor_id(struct fts_ts_data *data);
extern void fts_update_fw_ver(struct fts_ts_data *data);
extern void fts_get_upgrade_array(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);
extern int fts_fw_upgrade(struct device *dev, bool force);
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);

//Apk and functions
extern int fts_create_apk_debug_channel(struct i2c_client * client);
extern void fts_release_apk_debug_channel(void);

//ADB functions
extern int fts_create_sysfs(struct i2c_client *client);
extern int fts_remove_sysfs(struct i2c_client *client);

//Base functions
extern int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int fts_read_reg(struct i2c_client *client, u8 addr, u8 *val);
extern int fts_write_reg(struct i2c_client *client, u8 addr, const u8 val);

/*******************************************************************************
* Static function prototypes
*******************************************************************************/



#endif
