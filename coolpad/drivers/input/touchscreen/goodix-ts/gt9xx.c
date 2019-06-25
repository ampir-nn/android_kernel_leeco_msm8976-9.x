/* drivers/input/touchscreen/gt9xx.c
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
 * Version: 2.2
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2014/01/14
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 *      V2.2:
 *          1. gt9xx_config for debug
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *                  By Meta, 2014/01/14
 */

#include <linux/irq.h>
#include <linux/input/touchscreen_yl.h>
#include <linux/input/gt9xx.h>
#include <linux/pinctrl/consumer.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include <linux/bootreason.h>

const char * TW_IC_PREFIX_NAME=NULL;
unsigned char GTP_DEBUG_ON;
//#define YL_TW_DEBUG
static unsigned int tp_index;
s16 gtp_fw_version = 0;
u8 gtp_cfg_version = 0;
u8 tp_supported = 0;
//#if GTP_COB
u8 cfg_version_bak = 0;
//#endif

/*********** dtsi config ********************/
struct goodix_config_info {
	unsigned char tp_id;
	const char *vendor_name;
	unsigned char *config;
	unsigned char *config_glove;
	unsigned char *config_window;
	unsigned int config_length;
	unsigned int config_glove_length;
	unsigned int config_window_length;
};
struct goodix_config_info *yl_cfg=NULL;
static unsigned int config_array_size = 0;
/*********** dtsi config ********************/

static const char *goodix_ts_name = "goodix";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL;
static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
    = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

#ifdef GTP_CHARGER_STATUS_CHECK
#include <linux/power_supply.h>
static int charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
struct power_supply  *batt_psy;
struct workqueue_struct *goodix_rd_chg_wq;
struct delayed_work goodix_rd_chg_work;
#define Charging	1
#define Discharging	0

int Goodix_get_charger_status(void)
{
	union power_supply_propval ret = {0,};

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return Discharging;
	batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &ret);
	if (ret.intval == POWER_SUPPLY_STATUS_CHARGING)
		return Charging;
	else
		return Discharging;
}
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
//void gtp_reset_guitar(struct i2c_client *client, s32 ms);

#if defined (CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *handler);
static void goodix_ts_late_resume(struct early_suspend *handler);
#elif defined(CONFIG_FB)
static void fb_notify_resume_work(struct work_struct *work);
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data);
#endif
static int goodix_ts_disable(struct input_dev *in_dev);
static int goodix_ts_enable(struct input_dev *in_dev);
static int goodix_ts_suspend(struct device *dev);
static int goodix_ts_resume(struct device *dev);

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
#endif

#ifdef YL_TW_DEBUG
char *key_name[3] ={
	"Menu",
	"Home",
	"Back",
};
#endif

#pragma pack(1)
typedef struct
{
	u8  hw_info[4];          //hardware info//
	u8  pid[8];              //product id   //
	u16 vid;                 //version id   //
}st_fw_head;
#pragma pack()

extern u8 gup_check_update_file(struct i2c_client *client, st_fw_head* fw_head, u8* path);
extern u8 gup_get_ic_fw_msg(struct i2c_client *client);

//extern int get_device_info(char* buf); // yinchao add tp_info to factory mode @20140901

#if GTP_MODE_SWITCH
static int current_mode_flag  = MODE_NORMAL;
#endif

#if GTP_WINDOW_MODE
bool gtp_window_mode_flag = false;
struct cover_window_info {
	unsigned int win_x_min;
	unsigned int win_x_max;
	unsigned int win_y_min;
	unsigned int win_y_max;
};
struct cover_window_info gt9xx_cover_window;
#endif

#if GTP_SLIDE_WAKEUP
typedef enum
{
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);

enum support_gesture_e {
	TW_SUPPORT_NONE_SLIDE_WAKEUP   = 0x0,
	TW_SUPPORT_UP_SLIDE_WAKEUP     = 0x1,
	TW_SUPPORT_DOWN_SLIDE_WAKEUP   = 0x2,
	TW_SUPPORT_LEFT_SLIDE_WAKEUP   = 0x4,
	TW_SUPPORT_RIGHT_SLIDE_WAKEUP  = 0x8,
	TW_SUPPORT_E_SLIDE_WAKEUP      = 0x10,
	TW_SUPPORT_O_SLIDE_WAKEUP      = 0x20,
	TW_SUPPORT_W_SLIDE_WAKEUP      = 0x40,
	TW_SUPPORT_C_SLIDE_WAKEUP      = 0x80,
	TW_SUPPORT_M_SLIDE_WAKEUP      = 0x100,
	TW_SUPPORT_DOUBLE_CLICK_WAKEUP = 0x200,

	TW_SUPPORT_GESTURE_IN_ALL = (
			TW_SUPPORT_UP_SLIDE_WAKEUP    |
			TW_SUPPORT_DOWN_SLIDE_WAKEUP  |
			TW_SUPPORT_LEFT_SLIDE_WAKEUP  |
			TW_SUPPORT_RIGHT_SLIDE_WAKEUP |
			TW_SUPPORT_E_SLIDE_WAKEUP |
			TW_SUPPORT_O_SLIDE_WAKEUP |
			TW_SUPPORT_W_SLIDE_WAKEUP |
			TW_SUPPORT_C_SLIDE_WAKEUP |
			TW_SUPPORT_M_SLIDE_WAKEUP |
			TW_SUPPORT_DOUBLE_CLICK_WAKEUP)
};

u32 support_gesture = TW_SUPPORT_NONE_SLIDE_WAKEUP;
char wakeup_slide[32];
struct slide_wakeup
{
	unsigned char code;
	unsigned int mask;
	char *evp[2];
	char *gesture;
};
struct slide_wakeup tp_slide_wakeup[]={
	{0xAA,TW_SUPPORT_RIGHT_SLIDE_WAKEUP,{"GESTURE=RIGHT",NULL}, "right"},
	{0xBB,TW_SUPPORT_LEFT_SLIDE_WAKEUP,{"GESTURE=LEFT",NULL}, "left"},
	{0xBA,TW_SUPPORT_UP_SLIDE_WAKEUP,{"GESTURE=UP",NULL}, "up"},
	{0xAB,TW_SUPPORT_DOWN_SLIDE_WAKEUP,{"GESTURE=DOWN",NULL}, "down"},
	{0xCC,TW_SUPPORT_DOUBLE_CLICK_WAKEUP,{"GESTURE=DOUBLE_CLICK",NULL}, "double_click"},
	{0x63,TW_SUPPORT_C_SLIDE_WAKEUP,{"GESTURE=C",NULL}, "c"},
	{0x65,TW_SUPPORT_E_SLIDE_WAKEUP,{"GESTURE=E",NULL}, "e"},
	{0x6D,TW_SUPPORT_M_SLIDE_WAKEUP,{"GESTURE=M",NULL}, "m"},
	{0x6F,TW_SUPPORT_O_SLIDE_WAKEUP,{"GESTURE=O",NULL}, "o"},
	{0x77,TW_SUPPORT_W_SLIDE_WAKEUP,{"GESTURE=W",NULL}, "w"},
};
#endif

/*******************************************************
Function:
Read data from the i2c slave device.

Input:
client:	i2c device.
buf[0]:operate address.
buf[1]~buf[len]:read data buffer.
len:operate length.

Output:
numbers of i2c_msgs to transfer
 *********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret=-1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)break;
		retries++;
#if GTP_SLIDE_WAKEUP
		if (DOZE_ENABLED == doze_status)
		{
			mdelay(10);  //wait for i2c resume
		}
#endif
	}
	if(retries >= 5)
	{
#if GTP_SLIDE_WAKEUP
		// reset chip would quit doze mode
		if (DOZE_ENABLED == doze_status)
		{
			return ret;
		}
#endif
		GTP_DEBUG("I2C retry timeout, reset chip.");
		//gtp_reset_guitar(client, 10);
	}
	return ret;
}

/*******************************************************
Function:
write data to the i2c slave device.

Input:
client:	i2c device.
buf[0]:operate address.
buf[1]~buf[len]:write data buffer.
len:operate length.

Output:
numbers of i2c_msgs to transfer.
 *********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
	struct i2c_msg msg;
	s32 ret=-1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)break;
		retries++;
	}
	if(retries >= 5)
	{
#if GTP_SLIDE_WAKEUP
		if (DOZE_ENABLED == doze_status)
		{
			return ret;
		}
#endif
		GTP_DEBUG("I2C retry timeout, reset chip.");
		//gtp_reset_guitar(client, 10);
	}
	return ret;
}

#if GTP_SLIDE_WAKEUP
void gtp_enable_irq_wake(struct goodix_ts_data *ts, u8 enable)
{
	static u8 irq_wake_status = 0;

	if (irq_wake_status != enable) {
		if (enable) {
			enable_irq_wake(ts->client->irq);
		} else {
			disable_irq_wake(ts->client->irq);
		}
		irq_wake_status = enable;
	}
}
#endif
/*******************************************************
Function:
i2c read twice, compare the results
Input:
client:  i2c device
addr:    operate address
rxbuf:   read data to store, if compare successful
len:     bytes to read
Output:
FAIL:    read failed
SUCCESS: read successful
 *********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;

	while (retry++ < 3)
	{
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len+2))
		{
			break;
		}
	}
	if (retry < 3)
	{
		memcpy(rxbuf, confirm_buf+2, len);
		return SUCCESS;
	}
	else
	{
		GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
		return FAIL;
	}
}

/*******************************************************
Function:
Send config Function.

Input:
client:	i2c client.

Output:
Executive outcomes.0--success,non-0--fail.
 *******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 0;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;

	for (retry = 0; retry < 5; retry++)
	{
		ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0)
		{
			break;
		}
	}
	GTP_DEBUG_FUNC();
#endif

	return ret;
}

/*******************************************************
Function:
Enable IRQ Function.

Input:
ts:	i2c client private struct.

Output:
None.
 *******************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable)
	{
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
Disable IRQ Function.

Input:
ts:	i2c client private struct.

Output:
None.
 *******************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable)
	{
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
Touch down report function.

Input:
ts:private data.
id:tracking id.
x:input x.
y:input y.
w:input weight.

Output:
None.
 *******************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

#if GTP_WINDOW_MODE
	if(gtp_window_mode_flag &&
			!( (x > gt9xx_cover_window.win_x_min)
				&& (x < gt9xx_cover_window.win_x_max)
				&& (y > gt9xx_cover_window.win_y_min)
				&& (y < gt9xx_cover_window.win_y_max)));
#if 0
	if(atomic_read(&gt9xx_cover_window.windows_switch) && !(x > gt9xx_cover_window.win_x_min
				&& x < gt9xx_cover_window.win_x_max
				&& y > gt9xx_cover_window.win_y_min
				&& y < gt9xx_cover_window.win_y_max));
#endif
	else
#endif
	{
#if GTP_ICS_SLOT_REPORT
		input_mt_slot(ts->input_dev, id);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
		input_report_key(ts->input_dev, BTN_TOUCH, 1);//for fastmmi
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
		//input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 200);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
		input_mt_sync(ts->input_dev);
#endif
		GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
	}
}

/*******************************************************
Function:
Touch up report function.

Input:
ts:private data.

Output:
None.
 *******************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	GTP_DEBUG("Touch id[%2d] release!", id);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);//for fastmmi
	input_mt_sync(ts->input_dev);
#endif
}

/*******************************************************
Function:
Goodix touchscreen work function.

Input:
work:	work_struct of goodix_wq.

Output:
None.
 *******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8  touch_num = 0;
	u8  finger = 0;
	static u16 pre_touch = 0;
	static u8 pre_key = 0;
	u8  key_value = 0;
	u8* coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i  = 0;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
#ifdef YL_TW_DEBUG
	static u8 pre_touch_status = 0;
	u8 cur_touch_status = 0;
#endif

#if GTP_SLIDE_WAKEUP
	char **envp;
	u8 doze_buf[3] = {0x81, 0x4B};
	struct device *touchscreen_dev;
#endif

	GTP_DEBUG_FUNC();

	ts = container_of(work, struct goodix_ts_data, work);
	if (ts->enter_update)
	{
		return;
	}
#if GTP_SLIDE_WAKEUP
	if (DOZE_ENABLED == doze_status)
	{
		ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		GTP_INFO("0x814B = 0x%02x", doze_buf[2]);
		if (ret > 0)
		{
			for(i = 0; i < ARRAY_SIZE(tp_slide_wakeup); i++)
			{
				if( (doze_buf[2] == tp_slide_wakeup[i].code) && (support_gesture&tp_slide_wakeup[i].mask))
				{
					GTP_INFO("Slide(0x%x) To Light up the screen!", doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					sprintf(wakeup_slide,tp_slide_wakeup[i].gesture);
					envp = tp_slide_wakeup[i].evp;
					touchscreen_dev = touchscreen_get_dev();
					//input_report_key(ts->input_dev, KEY_POWER, 1);
					//input_sync(ts->input_dev);
					//input_report_key(ts->input_dev, KEY_POWER, 0);
					//input_sync(ts->input_dev);
					kobject_uevent_env(&touchscreen_dev->kobj, KOBJ_CHANGE, envp);
					GTP_INFO("send < %s slide wakeup > kobject uevent!", wakeup_slide);
					break;
				}
			}
		}

		// clear 0x814B
		doze_buf[2] = 0x00;
		gtp_i2c_write(i2c_connect_client, doze_buf, 3);
		gtp_enter_doze(ts);

		if (ts->use_irq)
		{
			gtp_irq_enable(ts);
		}
		return;
	}
#endif

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0)
	{
		GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
		goto exit_work_func;
	}

	finger = point_data[GTP_ADDR_LENGTH];

	if (finger == 0x00)
	{
		if (ts->use_irq)
		{
			gtp_irq_enable(ts);
		}
		return;
	}

	if((finger & 0x80) == 0)
	{
		goto exit_work_func;
	}

	touch_num = finger & 0x0f;
	if (touch_num > GTP_MAX_TOUCH)
	{
		goto exit_work_func;
	}

	if (touch_num > 1)
	{
		u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

		ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
		memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
	}

#if GTP_HAVE_TOUCH_KEY
	key_value = point_data[3 + 8 * touch_num];

	if(key_value || pre_key)
	{
		for (i = 0; i < GTP_MAX_KEY_NUM; i++)
		{
			input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));
			GTP_DEBUG("input_report_key:%d, touch_key_arr:%d\n", i, touch_key_array[i]);
#ifdef YL_TW_DEBUG
			if (key_value & (0x01<<i))
				GTP_INFO("%s KEY press.", key_name[i]);
#endif
		}
		touch_num = 0;
		pre_touch = 0;
	}
#endif
	pre_key = key_value;
	GTP_DEBUG("pre_key:%02x, finger:%02x.", pre_key, finger);

	GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT
	if (pre_touch || touch_num)
	{
		s32 pos = 0;
		u16 touch_index = 0;

		coor_data = &point_data[3];
		if(touch_num)
		{
			id = coor_data[pos] & 0x0F;
			touch_index |= (0x01<<id);
		}

		for (i = 0; i < GTP_MAX_TOUCH; i++)
		{
			if (touch_index & (0x01<<i))
			{
				input_x  = coor_data[pos + 1] | coor_data[pos + 2] << 8;
				input_y  = coor_data[pos + 3] | coor_data[pos + 4] << 8;
				input_w  = coor_data[pos + 5] | coor_data[pos + 6] << 8;

				gtp_touch_down(ts, id, input_x, input_y, input_w);
				pre_touch |= 0x01 << i;

				pos += 8;
				id = coor_data[pos] & 0x0F;
				touch_index |= (0x01<<id);
			}
			else// if (pre_touch & (0x01 << i))
			{
				gtp_touch_up(ts, i);
				pre_touch &= ~(0x01 << i);
			}
		}
	}

#else
	input_report_key(ts->input_dev, BTN_TOUCH, !!touch_num);
	if (touch_num )
	{
		for (i = 0; i < touch_num; i++)
		{
			coor_data = &point_data[i * 8 + 3];

			id = coor_data[0] & 0x0F;
			input_x  = coor_data[1] | coor_data[2] << 8;
			input_y  = coor_data[3] | coor_data[4] << 8;
			input_w  = coor_data[5] | coor_data[6] << 8;

			gtp_touch_down(ts, id, input_x, input_y, input_w);
#ifdef YL_TW_DEBUG
			cur_touch_status |= (0x01<<id);
			if (pre_touch < touch_num) {
				if (pre_touch_status ^ (0x01<<id)) {
					GTP_INFO("Point[%d] Touch Down! x=%d, y=%d.", id, input_x, input_y);
				}
			}
#endif
		}
#ifdef YL_TW_DEBUG
		if(pre_touch > touch_num) {
			for (i = 0; i < GTP_MAX_TOUCH; i++) {
				if ((pre_touch_status^cur_touch_status)&(0x01<<i)) {
					GTP_INFO("Point[%d] Touch Up!", i);
				}
			}
		}
#endif
	}
	else if (pre_touch)
	{
#ifdef YL_TW_DEBUG
		GTP_INFO("All Touch Release!");
#else
		GTP_DEBUG("Touch Release!");
#endif
		gtp_touch_up(ts, 0);
	}
#ifdef YL_TW_DEBUG
	pre_touch_status = cur_touch_status;
#endif
	pre_touch = touch_num;
#endif

	input_sync(ts->input_dev);

exit_work_func:
	if(!ts->gtp_rawdiff_mode)
	{
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
		{
			GTP_INFO("I2C write end_cmd  error!");
		}
	}

	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
}

/*******************************************************
Function:
Timer interrupt service routine.

Input:
timer:	timer struct pointer.

Output:
Timer work mode. HRTIMER_NORESTART---not restart mode
 *******************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
External interrupt service routine.

Input:
irq:	interrupt number.
dev_id: private data pointer.

Output:
irq execute status.
 *******************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();

	gtp_irq_disable(ts);
	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}
/*******************************************************
Function:
Reset chip Function.

Input:
ms:reset time.

Output:
None.
 *******************************************************/
#if 1
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	struct goodix_ts_data * ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();

	if(ts->pdata->reset)
		ts->pdata->reset(ms);

}
#endif
#if GTP_SLIDE_WAKEUP
/*******************************************************
Function:
Enter doze mode for sliding wakeup.
Input:
ts: goodix tp private data
Output:
1: succeed, otherwise failed
 *******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

	GTP_DEBUG_FUNC();

#if GTP_DBL_CLK_WAKEUP
	i2c_control_buf[2] = 0x09;
#endif

	gtp_irq_disable(ts);
	sprintf(wakeup_slide,"none");
	GTP_DEBUG("entering doze mode...");
	while(retry++ < 5)
	{
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0)
		{
			GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0)
		{
			doze_status = DOZE_ENABLED;
			GTP_INFO("GTP has been working in doze mode!");
			gtp_enable_irq_wake(ts, 1);
			gtp_irq_enable(ts);
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send doze cmd failed.");
	gtp_irq_enable(ts);
	return ret;
}
#endif
/*******************************************************
Function:
Eter sleep function.

Input:
ts:private data.

Output:
Executive outcomes.0--success,non-0--fail.
 *******************************************************/

static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
	s8 ret = 0;

	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_POWER_CTRL_SLEEP
	gtp_i2c_write(ts->client, i2c_control_buf, 3);

	if(ts->pdata->power)
		ts->pdata->power(0);
#else
	s8 retry = 0;

	if (1 == yl_get_ftm()) {
		pr_info("ftm mode, turn off vdd and vdd_io.\n");
		gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ts->pdata->power)
			ts->pdata->power(0);
		return ret;
	}

	//irq output 0
	gpio_direction_output(ts->pdata->gpio_irq, 0);
	msleep(5);

	while(retry++ < 5)
	{
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0)
		{
			GTP_DEBUG("GTP enter sleep!");
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send sleep cmd failed.");
#endif

	return ret;
}
/*******************************************************
Function:
Wakeup from sleep mode Function.

Input:
ts:	private data.

Output:
Executive outcomes.0--success,non-0--fail.
 *******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
	if(ts->pdata->power)
		ts->pdata->power(1);
#endif

	if (1 == yl_get_ftm()) {
		pr_info("ftm mode, turn on vdd and vdd_io.\n");
		if (ts->pdata->power)
			ts->pdata->power(1);
	}

	while(retry++ < 5)
	{
#if GTP_SLIDE_WAKEUP
	  doze_status = DOZE_DISABLED;
	  if(support_gesture & TW_SUPPORT_GESTURE_IN_ALL)
	     gtp_irq_disable(ts);
#endif

		gtp_reset_guitar(ts->client, 20);

#if GTP_SLIDE_WAKEUP
    if(support_gesture & TW_SUPPORT_GESTURE_IN_ALL)
	     gtp_irq_enable(ts);
#endif


		ret = gtp_send_cfg(ts->client);
		if (ret < 0)
		{
			GTP_INFO("Wakeup sleep send config failed!");
			continue;
		}
		GTP_INFO("GTP wakeup sleep");
		return 1;
	}

	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}
/*******************************************************
Function:
GTP initialize function.

Input:
ts:	i2c client private struct.

Output:
Executive outcomes.0---succeed.
 *******************************************************/
s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	s32 i;
	u8 check_sum = 0;
	u8 opr_buf[16];
	u8 sensor_id = 0;
	u8 retry = 0;

	ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
	if (SUCCESS == ret)
	{
		if (opr_buf[0] != 0xBE)
		{
			ts->fw_error = 1;
			printk("Firmware error, no config sent!");
			return -1;
		}
	}

	// module id
	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
	if (SUCCESS == ret)
	{
		while((sensor_id == 0xff)&&(retry++ < 3))
		{
			msleep(100);
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
		GTP_ERROR("GTP sensor_ID read failed time %d.",retry);
		}

		if (sensor_id >= 0x06)
		{
			GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
			return -1;
		}
	}
	else
	{
		GTP_ERROR("Failed to get sensor_id, No config sent!");
		return -1;
	}

	tp_supported = 0;
	for(i=0; i<config_array_size; i++)
	{
		if(sensor_id == yl_cfg[i].tp_id)
		{
			tp_index = i;
			tp_supported = 1;
			ts->gtp_cfg_len = yl_cfg[tp_index].config_length;
			break;
		}
	}
	if(tp_supported != 1)
	{
		GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
		return -1;
	}

	if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
	{
		GTP_ERROR("Sensor_ID(%d) matches with NULL or INVALID CONFIG GROUP! \
				NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id);
		return -1;
	}

	//config
	ts->gtp_cfg_len = yl_cfg[tp_index].config_length;
	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], yl_cfg[tp_index].config, ts->gtp_cfg_len);

//#if GTP_COB
	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
	if (ret == SUCCESS)
	{
		GTP_INFO("IC cfg ver:0x%02X, driver code cfg ver:0x%02X\n",
				opr_buf[0], config[GTP_ADDR_LENGTH]);
		if ((opr_buf[0] < 90) && (opr_buf[0] > config[GTP_ADDR_LENGTH])) {
			cfg_version_bak = config[GTP_ADDR_LENGTH];
			config[GTP_ADDR_LENGTH] = 0x00;
		}
	}
//#endif

#if GTP_CUSTOM_CFG
	//max size
	config[RESOLUTION_LOC]     = ts->pdata->screen_x&0xff;
	config[RESOLUTION_LOC + 1] = (ts->pdata->screen_x>>8)&0xff;
	config[RESOLUTION_LOC + 2] = ts->pdata->screen_y&0xff;
	config[RESOLUTION_LOC + 3] = (ts->pdata->screen_y>>8)&0xff;

#endif  //endif GTP_CUSTOM_CFG

	//trigger type
	if (ts->pdata->irqflag == IRQF_TRIGGER_RISING)  //RISING
	{
		config[TRIGGER_LOC] &= 0xfc;
	}
	else if (ts->pdata->irqflag == IRQF_TRIGGER_FALLING)  //FALLING
	{
		config[TRIGGER_LOC] &= 0xfc;
		config[TRIGGER_LOC] |= 0x01;
	}

	//chechsum
	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
	{
		check_sum += config[i];
	}
	config[ts->gtp_cfg_len] = (~check_sum) + 1;

	ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
	ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
	ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	if ((!ts->abs_x_max)||(!ts->abs_y_max))
	{
		GTP_ERROR("GTP resolution & max_touch_num invalid, use default value!");
		ts->abs_x_max = ts->pdata->screen_x;
		ts->abs_y_max = ts->pdata->screen_y;
	}

	ret = gtp_send_cfg(ts->client);
	if (ret < 0)
	{
		GTP_ERROR("Send config error.");
	}

	msleep(10);

//#if GTP_COB
	if(cfg_version_bak)
	{
		config[GTP_ADDR_LENGTH] = cfg_version_bak;
		cfg_version_bak = 0;
		GTP_INFO("IC cfg ver > code cfg ver, back to code cfg ver.\n");

		//chechsum
		check_sum = 0;
		for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
		{
			check_sum += config[i];
		}
		config[ts->gtp_cfg_len] = (~check_sum) + 1;
		/*send two times cfg params, IC need longer time to update.*/
		msleep(500);
		ret = gtp_send_cfg(ts->client);
		msleep(10);
	}
//#endif

	return 0;
}
/*******************************************************
Function:
Read goodix touchscreen version function.

Input:
client:	i2c client struct.
version:address to store version info

Output:
Executive outcomes.0---succeed.
 *******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
	s32 ret = -1;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0)
	{
		printk("GTP read version failed");
		return ret;
	}

	if (version)
	{
		*version = (buf[7] << 8) | buf[6];
	}

	if (buf[5] == 0x00)
	{
		printk("[GTP]IC Version: %c%c%c_%02x%02x\n", buf[2], buf[3], buf[4], buf[7], buf[6]);
	}
	else
	{
		printk("[GTP]IC Version: %c%c%c%c_%02x%02x\n", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	}

	return ret;
}

s32 gtp_read_fw_cfg_version(void)
{
	s32 ret = -1;
	u8 cfg[3] = {0};

	ret = gtp_read_version(i2c_connect_client, &gtp_fw_version);
	if (ret < 0)
	{
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	cfg[0] = GTP_REG_CONFIG_DATA >> 8;
	cfg[1] = GTP_REG_CONFIG_DATA & 0xff;

	ret = gtp_i2c_read(i2c_connect_client, cfg, 3);
	if( ret < 0 )
	{
		GTP_ERROR("Read config version failed.");
		return ret;
	}
	//	if(gtp_cfg_version < cfg[2])
	gtp_cfg_version = cfg[2];

	return 0;
}

/*******************************************************
Function:
I2c test Function.

Input:
client:i2c client.

Output:
Executive outcomes.0--success,non-0--fail.
 *******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while(retry++ < 5)
	{
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0)
		{
			return ret;
		}
		GTP_ERROR("GTP i2c test failed time %d.",retry);
		msleep(10);
	}

	return ret;
}
/*******************************************************
Function:
Request irq Function.

Input:
ts:private data.

Output:
Executive outcomes.0--success,non-0--fail.
 *******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;

	GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

	ret  = request_irq(ts->client->irq,
			goodix_ts_irq_handler,
			ts->pdata->irqflag,
			ts->client->name,
			ts);
	if (ret)
	{
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -1;
	}
	else
	{
		ts->use_irq = 1;
		gtp_irq_disable(ts);
		return 0;
	}
}
/*******************************************************
Function:
Request input device Function.

Input:
ts:private data.

Output:
Executive outcomes.0--success,non-0--fail.
 *******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL)
	{
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

#if GTP_ICS_SLOT_REPORT
	input_mt_init_slots(ts->input_dev, 10);     // in case of "out of memory"
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++)
	{
		input_set_capability(ts->input_dev,EV_KEY,touch_key_array[index]);
	}
#endif

#if GTP_SLIDE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif
#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

	//input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->screen_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata->screen_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, GTP_MAX_TOUCH, 0, 0);

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	sprintf(phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_FB)

	INIT_WORK(&ts->fb_notify_work, fb_notify_resume_work);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		GTP_ERROR("Unable to register fb_notifier: %d\n",ret);
#endif
	input_set_drvdata(ts->input_dev, ts);

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		GTP_ERROR("Register %s input device failed", ts->input_dev->name);
		return -ENODEV;
	}

	return 0;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

 ***********************************************************************************************/

/*******************************************************
 ********  1--active  0--not active***********
 *******************************************************/
int goodix_active(void)
{
	GTP_DEBUG("enter %s",__FUNCTION__);
	return 1;
}
/*******************************************************
 ***********check firmware if need update***********
 *******************************************************/
int goodix_firmware_need_update(void)
{
	st_fw_head fw_head;
	int ret = -1;

	GTP_INFO(" check if need firmware update...");
	GTP_DEBUG("enter %s",__FUNCTION__);

	ret = gup_get_ic_fw_msg(i2c_connect_client);
	if(FAIL == ret)
	{
		GTP_ERROR("[update_proc]get ic message fail.");
		return FAIL;
	}
	ret = gup_check_update_file(i2c_connect_client, &fw_head, 0);
	return ret;
}
/*******************************************************
 *********************do firmware update ***************
 *******************************************************/
int goodix_firmware_do_update(void)
{
	int ret = -1;

	GTP_DEBUG("enter %s",__FUNCTION__);
	ret = goodix_firmware_need_update();
	if(ret)
	{
		struct goodix_ts_data *ts = NULL;
		GTP_DEBUG_FUNC();
		ts = i2c_get_clientdata(i2c_connect_client);

		if(0 == gup_init_update_proc(ts))
			return -1;
		else
		{
			gtp_read_fw_cfg_version();
			return 0;
		}
	}

	return ret;
}
/*******************************************************
 *******************check if need calibrate***********
 *******************************************************/
int goodix_need_calibrate(void)
{
	GTP_DEBUG("enter %s",__FUNCTION__);
	return 0;
}
/*******************************************************
 ******************system write "calibrate"************
 *******************************************************/
int goodix_calibrate(void)
{
	GTP_DEBUG("enter %s",__FUNCTION__);
	return 0;
}
/*******************************************************
 ******************get firmware version **************
 *******************************************************/
int goodix_get_firmware_version(char * version )
{
	gtp_read_fw_cfg_version();

	if(tp_supported)
		return sprintf(version, "%s:%s:0x%4x:0x%2x", yl_cfg[tp_index].vendor_name,
				TW_IC_PREFIX_NAME, gtp_fw_version, gtp_cfg_version);
	else
		return sprintf(version, "%s:%s:0x%4x:0x%2x", "----", TW_IC_PREFIX_NAME,
				gtp_fw_version, gtp_cfg_version);
}
/*******************************************************
 ******************system write "reset"***************
 *******************************************************/
int goodix_reset_touchscreen(void)
{
	GTP_DEBUG("enter %s",__FUNCTION__);
	return 1;
}
/*******************************************************
 ******************charger status check***************
 *******************************************************/
u8 gt9158_has_resumed = 0;
#ifdef GTP_CHARGER_STATUS_CHECK
void yl_chg_status_changed(struct work_struct *work)
{
	struct goodix_ts_data *ts = NULL;
	unsigned int chg_status = 0;
	int count = 0;
	u8 write_val;
	s32 ret = -1;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 0x0a};

	ts = i2c_get_clientdata(i2c_connect_client);
	chg_status = Goodix_get_charger_status();
	GTP_DEBUG("tw get charger status is: %d\n", chg_status);
	if (chg_status != charge_status) {
		charge_status = chg_status;
		if (chg_status == 0)
			write_val = 0x07;
		else
			write_val = 0x06;
		i2c_control_buf[2] = write_val;

		for (count = 0; count < 5; count++) {
			if (i2c_connect_client != NULL)
				ret = gtp_i2c_write(i2c_connect_client,
					i2c_control_buf, 3);
			if (ret > 0) {
				GTP_INFO("write 0x8040 value = %d.\n",
					write_val);
				break;
			} else {
				GTP_ERROR("write 0x8040 error.\n");
			}
		}
	}
	queue_delayed_work(goodix_rd_chg_wq, &goodix_rd_chg_work,
		ts->clk_tick_cnt);
}
EXPORT_SYMBOL(yl_chg_status_changed);
#endif

/*******************************************************
 ************ get work mode &set work mode*************
 *******************************************************/
#if GTP_WINDOW_MODE
DEFINE_MUTEX(glove_mutex);

void mode_window(bool on)
{
	mutex_lock(&glove_mutex);
	if(on)
		gtp_window_mode_flag = true;
	else
		gtp_window_mode_flag = false;
	mutex_unlock(&glove_mutex);

	return;
}
void glove_windows_switch(int in_hall)
{
	mutex_lock(&glove_mutex);
	if(in_hall)
	{
		mode_window(true);
	} else {
		mode_window(false);
	}
	mutex_lock(&glove_mutex);

	return;
}
EXPORT_SYMBOL_GPL(glove_windows_switch);
#endif

enum touch_mode_type goodix_get_mode(void)
{
	GTP_DEBUG("enter %s",__FUNCTION__);
#if GTP_MODE_SWITCH
	GTP_INFO("current tw mode flag is %d", current_mode_flag);
	return current_mode_flag;
#endif

	return 1;
}
int goodix_set_mode(enum touch_mode_type work_mode)
{
#if GTP_MODE_SWITCH
	u8 retry = 0;
	int ret = -1;
	struct goodix_ts_data *ts;


	GTP_DEBUG("enter %s",__FUNCTION__);
	if (i2c_connect_client == NULL)
	{
		return -1;
	}
	ts = i2c_get_clientdata(i2c_connect_client);
	current_mode_flag = work_mode;
#if GTP_WINDOW_MODE
	//	if(MODE_WINDOW == current_mode_flag)
	//	{
	//	    mode_window(true);
	//		return 0;
	//	} else {
	//	    mode_window(false);
	//		return 0;
	//	}
#endif
	if (ts->use_irq)
	{
		gtp_irq_disable(ts);
	}
	else
	{
		hrtimer_cancel(&ts->timer);
	}

	if(MODE_NORMAL == current_mode_flag)
	{
		ts->gtp_cfg_len = yl_cfg[tp_index].config_length;
		memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
		memcpy(&config[GTP_ADDR_LENGTH], yl_cfg[tp_index].config, ts->gtp_cfg_len);
		GTP_INFO("send normal mode config.");
	}
	else if(MODE_GLOVE == current_mode_flag)
	{
		ts->gtp_cfg_len = yl_cfg[tp_index].config_glove_length;
		memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
		memcpy(&config[GTP_ADDR_LENGTH], yl_cfg[tp_index].config_glove, ts->gtp_cfg_len);
		GTP_INFO("send glove mode config.");
	}

	while(retry++ < 5)
	{
		gtp_reset_guitar(ts->client, 20);
		ret = gtp_send_cfg(ts->client);
		if (ret >= 0)
		{
			GTP_INFO("send config successed!");
			break;
		}
	}

	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
	else
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	printk("[GTP]: current_mode_flag = %d\n", current_mode_flag);
#endif

	return 1;
}
/*******************************************************
 ****************get "oreitation:X" ************
 *******************************************************/
/*enum  touch_orientation_type goodix_get_oreitation(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}
int goodix_set_oreitation(enum touch_orientation_type oreitate)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}
*/
/*******************************************************
 **********read buf[256]: ef:ab [???????]?šºo??š¬  ********
 *******************************************************/
int goodix_read_regs(char * buf)
{
	int ret = -1;
	u8 cfg[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = { 0 };

	GTP_DEBUG_FUNC();

	cfg[0] = GTP_REG_CONFIG_DATA >> 8;
	cfg[1] = GTP_REG_CONFIG_DATA & 0xff;
	ret = gtp_i2c_read(i2c_connect_client, cfg, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
	if( ret < 0 )
	{
		GTP_ERROR("Read config version failed.\n");
		return ret;
	}

	for(ret=0; ret<GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH; ret++)
	{
		printk("reg%d = 0x%x\n", ret, cfg[ret]);
	}

	return 1;
}
/*******************************************************
 **********write buf[256]: ef:ab [???????]?šºo??š¬ ********
 *******************************************************/
int goodix_write_regs(const char * buf)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}
/*******************************************************
 ***************tw debug on or off *************
 *******************************************************/
int goodix_debug(int val)
{
	GTP_INFO("enter %s\n",__FUNCTION__);
	if(val)
		GTP_DEBUG_ON=1;
	else
		GTP_DEBUG_ON=0;
	return 1;
}
/*******************************************************
 ********************tw vendor**********************
 *******************************************************/
int goodix_vendor(char*  vendor)
{

	return sprintf(vendor, "goodix:%s", TW_IC_PREFIX_NAME);

}
/*******************************************************
 *******get slide wakeup current setting gesture*******
 *******************************************************/
int goodix_get_wakeup_gesture(char*  gesture)
{
#if GTP_SLIDE_WAKEUP
	int i;
	char wakeup_gesture[64]={0};

	if(support_gesture == 0)
		return sprintf(gesture, "%s", "none");

	for(i=0; i<ARRAY_SIZE(tp_slide_wakeup); i++)
	{
		if(support_gesture & tp_slide_wakeup[i].mask)
		{
			if(wakeup_gesture[0] != 0)
				sprintf(wakeup_gesture + strlen(wakeup_gesture), "%s", ",");
			sprintf(wakeup_gesture + strlen(wakeup_gesture) , "%s", tp_slide_wakeup[i].gesture);
		}
	}

	return sprintf(gesture, "%s", wakeup_gesture);
#endif

	return 1;
}
/*******************************************************
 *************** system set wakeup gesture*************
 *******************************************************/
int goodix_gesture_ctrl(const char*  gesture_buf)
{
#if GTP_SLIDE_WAKEUP
	char *gesture;
	int buf_len;
	char *gesture_p;
	char *temp_p;
	char tmp_buf[32]={0};
	int i;

#if defined(CONFIG_BOARD_CPY76_I01) || defined(CONFIG_BOARD_CPK1_NW_I00) || defined(CONFIG_BOARD_CPY76_U00)
	/* note: not use vars for 8716 */
#else
	s8 ret = -1;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
#endif

	buf_len = strlen(gesture_buf);

	GTP_DEBUG("%s buf_len = %d.", __func__, buf_len);
	GTP_DEBUG("%s gesture_buf:%s.", __func__, gesture_buf);

	gesture_p = kzalloc(buf_len + 1, GFP_KERNEL);
	if(gesture_p == NULL) {
		GTP_ERROR("%s: alloc mem error.", __func__);
		return -1;
	}
	temp_p = gesture_p;
	strlcpy(gesture_p, gesture_buf, buf_len + 1);

	while(gesture_p) {
		gesture = strsep(&gesture_p, ",");
		GTP_DEBUG("%s gesture:%s.", __func__, gesture);

		for(i=0; i<ARRAY_SIZE(tp_slide_wakeup); i++)
		{
			memset(tmp_buf, 0, 32);
			strcpy(tmp_buf, tp_slide_wakeup[i].gesture);
			if(!strncmp(gesture, tmp_buf, strlen(tmp_buf))){
				if(!strncmp(gesture+strlen(tmp_buf), "=true", 5)) {
					GTP_DEBUG("%s: enable up slide wakeup func.", tmp_buf);
					support_gesture |= tp_slide_wakeup[i].mask;
					break;
				} else if(!strncmp(gesture+strlen(tmp_buf), "=false", 6)) {
					GTP_DEBUG("%s: disable up slide wakeup func.", tmp_buf);
					support_gesture &= ~tp_slide_wakeup[i].mask;
					break;
				}
			}
		}
	}

#if defined(CONFIG_BOARD_CPY76_I01) || defined(CONFIG_BOARD_CPK1_NW_I00) || defined(CONFIG_BOARD_CPY76_U00)
	/* note: use this code may result in TP is invalid!         */
	/*    1) in gesture(doze) mode;                             */
	/*    2) goodix_ts_enable() can set gt9158_has_resumed = 1; */
	/*    3) but gesture_ctrl() and goodix_ts_enable() have not */
	/*       in same thread, so there may be conflict;          */
#else
	GTP_DEBUG("%s: gt9158_has_resumed=%d\n", __func__, gt9158_has_resumed);
	if (gt9158_has_resumed == 0) {
		gtp_reset_guitar(ts->client, 20);
		if(support_gesture & TW_SUPPORT_GESTURE_IN_ALL)
		{
			ret = gtp_enter_doze(ts);
			if (ret < 0) {
				printk("%s: gtp enter doze failed, ret=%d\n", __func__, ret);
			} else {
				printk("%s: gtp enter doze success\n", __func__);
			}
		} else {
			if (ts->use_irq)
			{
				gtp_irq_disable(ts);
			} else {
				hrtimer_cancel(&ts->timer);
			}

			ret = gtp_enter_sleep(ts);
			if (ret < 0) {
				printk("%s: gtp enter sleep failed, ret=%d\n", __func__, ret);
			} else {
				printk("%s: gtp enter sleep success\n", __func__);
			}
			gtp_enable_irq_wake(ts, 0);

			if(ts->pdata->suspend)
				ts->pdata->suspend();
		}
	}
#endif

	kfree(temp_p);
#endif

	return 1;
}
int goodix_ftsscaptest(char *result)
{
	GTP_DEBUG("enter %s, cmd: %s", __func__, result);
	return 1;
}

struct touchscreen_funcs goodix_ops=
{
	.touch_id				= 0,
	.touch_type			        = 1,
	.active					= goodix_active,
	.firmware_need_update			= goodix_firmware_need_update,
	.firmware_do_update			= goodix_firmware_do_update,
	.need_calibrate				= goodix_need_calibrate,
	.calibrate				= goodix_calibrate,
	.get_firmware_version			= goodix_get_firmware_version,
	.reset_touchscreen			= goodix_reset_touchscreen,
	.get_mode				= goodix_get_mode,
	.set_mode				= goodix_set_mode,
//	.get_oreitation				= goodix_get_oreitation,
//	.set_oreitation				= goodix_set_oreitation,
	.read_regs				= goodix_read_regs,
	.write_regs				= goodix_write_regs,
	.debug					= goodix_debug,
	.get_vendor				= goodix_vendor,
	.get_wakeup_gesture			= goodix_get_wakeup_gesture,
	.gesture_ctrl				= goodix_gesture_ctrl,
	.ftsscaptest				= goodix_ftsscaptest,

};

#ifdef CONFIG_OF
static struct regulator *ts_vdd;
static struct regulator *ts_vdd_io;

static unsigned int gtp_gpio_reset;
static unsigned int gtp_gpio_irq;
static unsigned int gtp_pwr_en=0;

const char *gtp_vdd=NULL;
const char *gtp_vdd_io=NULL;

static int goodix_pinctrl_init(struct goodix_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(ts->client->dev));
	if (IS_ERR_OR_NULL(ts->ts_pinctrl)) {
		dev_dbg(&ts->client->dev,
				"Target does not use pinctrl\n");
		retval = PTR_ERR(ts->ts_pinctrl);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_active
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->gpio_state_active)) {
		dev_dbg(&ts->client->dev,
				"Can not get ts default pinstate\n");
		retval = PTR_ERR(ts->gpio_state_active);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_suspend
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->gpio_state_suspend)) {
		dev_dbg(&ts->client->dev,
				"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ts->gpio_state_suspend);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int goodix_pinctrl_select(struct goodix_ts_data *ts,
		bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ts->gpio_state_active
		: ts->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ts->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ts->client->dev,
					"can not set %s pins\n",
					on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		dev_err(&ts->client->dev,
				"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}

static int goodix_power_init(bool on)
{
	int rc;

	if(!gtp_pwr_en)
		return 0;

	if(gtp_vdd==NULL || gtp_vdd_io==NULL)
		return 0;

	if (!on)
		goto gtp_pwr_deinit;

	ts_vdd = regulator_get(&i2c_connect_client->dev, gtp_vdd);
	if (IS_ERR(ts_vdd)) {
		rc = PTR_ERR(ts_vdd);
		pr_err("Touchscreen vdd Regulator %s get failed rc=%d\n", gtp_vdd, rc);
		return rc;
	}

	if (regulator_count_voltages(ts_vdd) > 0) {
		rc = regulator_set_voltage(ts_vdd, 2600000,3300000);
		if (rc) {
			pr_err("Touchscreen Regulator vdd set failed, rc=%d\n", rc);
			goto gtp_reg_vdd_put;
		}
	}

	ts_vdd_io = regulator_get(&i2c_connect_client->dev, gtp_vdd_io);
	if (IS_ERR(ts_vdd_io)) {
		rc = PTR_ERR(ts_vdd_io);
		pr_err("Touchscreen vdd_io Regulator %s get failed, rc=%d\n", gtp_vdd_io, rc);
		goto gtp_reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(ts_vdd_io) > 0) {
		rc = regulator_set_voltage(ts_vdd_io, 1800000,1800000);
		if (rc) {
			pr_err("Touchscreen Regulator vdd_io set voltage failed, rc=%d\n", rc);
			goto gtp_reg_vcc_i2c_put;
		}
	}

	return 0;

gtp_reg_vcc_i2c_put:
	regulator_put(ts_vdd_io);
gtp_reg_vdd_set_vtg:
	if (regulator_count_voltages(ts_vdd) > 0)
		regulator_set_voltage(ts_vdd, 0, 3300000);
gtp_reg_vdd_put:
	regulator_put(ts_vdd);
	return rc;

gtp_pwr_deinit:
	if (regulator_count_voltages(ts_vdd) > 0)
		regulator_set_voltage(ts_vdd, 0, 3300000);

	regulator_put(ts_vdd);

	if (regulator_count_voltages(ts_vdd_io) > 0)
		regulator_set_voltage(ts_vdd_io, 0, 1800000);

	regulator_put(ts_vdd_io);

	return 0;
}

static int goodix_ts_power(int on)
{
	int rc;

	if(!gtp_pwr_en)
		return 0;

	if (!on)
		goto goodix_power_off;

	rc = regulator_enable(ts_vdd);
	if (rc) {
		pr_err("Touchscreen Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(ts_vdd_io);
	if (rc) {
		pr_err("Touchscreen Regulator vdd_i2c enable failed rc=%d\n", rc);
		regulator_disable(ts_vdd);
	}

	return rc;

goodix_power_off:
	rc = regulator_disable(ts_vdd);
	if (rc) {
		pr_err("Touchscreen Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(ts_vdd_io);
	if (rc) {
		pr_err("Touchscreen Regulator vdd_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(ts_vdd);
	}

	return rc;
}

static int goodix_ts_gpio_init(bool on)
{
	int ret = 0;

	if(!on)
		goto gtp_gpio_deinit;

	/* touchscreen reset gpio request and init */
	if (gpio_is_valid(gtp_gpio_reset)) {
		ret = gpio_request(gtp_gpio_reset, "goodix-ts-reset");
		if (ret){
			pr_err("TOUCH:%s:Failed to request GPIO %d\n",__func__, gtp_gpio_reset);
			return ret;
		}
		gpio_direction_output(gtp_gpio_reset, 0);
	} else {
		pr_err("TOUCH:%s:irq gpio not provided\n",__func__);
		return -1;
	}

	if (gpio_is_valid(gtp_gpio_irq)) {
		ret = gpio_request(gtp_gpio_irq, "goodix-ts-irq");
		if (ret){
			pr_err("TOUCH:%s: Failed to request GPIO %d\n",__func__, gtp_gpio_irq);
			gpio_free(gtp_gpio_reset);
			return ret;
		}
		gpio_direction_input(gtp_gpio_irq);
	} else {
		pr_err("TOUCH:%s: reset gpio not provided\n",__func__);
		gpio_free(gtp_gpio_reset);
		return -1;
	}

	return 0;


gtp_gpio_deinit:
	gpio_free(gtp_gpio_reset);
	gpio_free(gtp_gpio_irq);

	return 0;
}

static int goodix_ts_hw_init(void)
{
	int ret = 0;

	ret = goodix_ts_gpio_init(1);
	if(ret)
	{
		pr_err("Touchscreen GPIO init failed!\n");
		return ret;
	}

	ret = goodix_power_init(1);
	if(ret)
	{
		pr_err("Touchscreen Power init failed!\n");
		goodix_ts_gpio_init(0);
		return ret;
	}

	pr_debug("TOUCH: requst and init resource complete\n");

	return 0;
}

static void goodix_ts_release(void)
{
	int ret = 0;

	ret = goodix_ts_gpio_init(0);
	if(ret)
	{
		pr_err("Touchscreen GPIO release failed!\n");
	}

	ret = goodix_power_init(0);
	if(ret)
	{
		pr_err("Touchscreen Power release failed!\n");
	}

}

static int goodix_ts_reset(int ms)
{
	gpio_direction_output(gtp_gpio_irq, 0);
	gpio_direction_output(gtp_gpio_reset, 0);
	msleep(ms);

	//step2:select I2C slave addr,INT:0--0x5d;1--0x14.
	gpio_direction_output(gtp_gpio_irq, 0);
	mdelay(2);

	//end select I2C slave addr
	gpio_direction_output(gtp_gpio_reset, 1);
	mdelay(8);                          //must > 3ms

	gpio_direction_input(gtp_gpio_reset);
	gpio_direction_output(gtp_gpio_irq, 0);
	msleep(55);
	gpio_direction_input(gtp_gpio_irq);

#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(i2c_connect_client);
#endif

	return 0;
}


static int goodix_ts_sleep(void)
{
	return 0;
}


static int goodix_ts_wakeup(void)
{
	return 0;
}

static int goodix_parse_config(struct device *dev, struct device_node *np,
		struct goodix_config_info *info)
{
	struct property *prop;
	unsigned char *temp_cfg;
	unsigned char *temp_glove_cfg;

	prop = of_find_property(np, "goodix,config_normal", &info->config_length);
	if(!prop){
		GTP_ERROR("Looking up goodix,config property failed.\n");
		return -ENODEV;
	}else if(!info->config_length){
		GTP_ERROR("Invalid length of configuration data\n");
		return -EINVAL;
	}
	temp_cfg = devm_kzalloc(dev,
			info->config_length*sizeof(unsigned int),GFP_KERNEL);
	if(!temp_cfg){
		GTP_ERROR("Unable to allocate memory to store cfg\n");
		return -ENOMEM;
	}
	memcpy(temp_cfg, prop->value, info->config_length);
	info->config = temp_cfg;

	prop = of_find_property(np, "goodix,config_glove", &info->config_glove_length);
	if(!prop){
		GTP_ERROR("Looking up goodix,config glove property failed.\n");
		return -ENODEV;
	}else if(!info->config_glove_length){
		GTP_ERROR("Invalid length of configuration data\n");
		return -EINVAL;
	}
	temp_glove_cfg = devm_kzalloc(dev,
			info->config_glove_length*sizeof(unsigned int),GFP_KERNEL);
	if(!temp_glove_cfg){
		GTP_ERROR("Unable to allocate memory to store cfg\n");
		return -ENOMEM;
	}
	memcpy(temp_glove_cfg, prop->value, info->config_glove_length);
	info->config_glove = temp_glove_cfg;

	return 0;
}

static int goodix_ts_parse_dt(struct device *dev, struct tw_platform_data *pdata)
{
	struct goodix_config_info *info;
	struct device_node *temp,*np = dev->of_node;
	int ret;
	enum of_gpio_flags flags;
	unsigned int res[4];

	/* reset, irq gpio info */
	ret = of_get_named_gpio_flags(np, "goodix,reset-gpio", 0, &flags);
	if (ret < 0){
		dev_err(dev, "Looking up %s property in node %s failed",
				"goodix,reset-gpio", np->full_name);
		return -ENODEV;
	}
	pdata->gpio_reset = ret;
	GTP_DEBUG("reset-gpio = %d", pdata->gpio_reset);

	ret = of_get_named_gpio_flags(np, "goodix,irq-gpio", 0, &flags);
	if (ret < 0) {
		dev_err(dev, "Looking up %s property in node %s failed",
				"goodix,irq-gpio", np->full_name);
		return -ENODEV;
	}
	pdata->gpio_irq = ret;
	GTP_DEBUG("irq-gpio = %d", pdata->gpio_irq);

	ret = of_property_read_u32(np, "goodix,irq_flags", (unsigned int *)&pdata->irqflag);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
				"goodix,irq_flags", np->full_name);
		return -ENODEV;
	}
	GTP_DEBUG("irq-flag = %d", (int)pdata->irqflag);

	ret = of_property_read_u32_array(np, "goodix,pixel", res, 2);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
				"goodix,pixel", np->full_name);
		return -ENODEV;
	}
	pdata->screen_x = res[0];
	pdata->screen_y = res[1];
	GTP_DEBUG("screen_x=%d, screen_y=%d", pdata->screen_x, pdata->screen_y);

	ret = of_property_read_u32_array(np, "goodix,pwr_ctrl", res, 1);
	if (ret) {
		GTP_ERROR("Looking up %s property in node %s failed",
				"goodix,pwr_crtl", np->full_name);
		gtp_pwr_en = 0;
	}
	else
	{
		gtp_pwr_en = res[0];
	}
	GTP_DEBUG("gtp_pwr_en=%d", gtp_pwr_en);

	ret = of_property_read_string(np, "goodix,vdd", &gtp_vdd);
	if (ret)
		GTP_ERROR("Looking up %s property in node %s failed",
				"goodix,vdd", np->full_name);

	ret = of_property_read_string(np, "goodix,vddio", &gtp_vdd_io);
	if (ret)
		GTP_ERROR("Looking up %s property in node %s failed",
				"goodix,vddio", np->full_name);

	ret = of_property_read_string(np, "goodix,ic_type", &TW_IC_PREFIX_NAME);
	if (ret)
	{
		TW_IC_PREFIX_NAME = "GT9XX";
		GTP_ERROR("Looking up %s property in node %s failed",
				"goodix,ic_type", np->full_name);
	}

/************* dtsi config parse start *********************/
	config_array_size = 0;
	temp = NULL;
	while((temp = of_get_next_child(np, temp)))
	{
		config_array_size++;
	}
	if(!config_array_size)
		return 0;

	info = devm_kzalloc(dev,config_array_size*
			sizeof(struct goodix_config_info),GFP_KERNEL);
	if(!info){
		GTP_ERROR("Unable to allocate memory\n");
		return -ENOMEM;
	}

	yl_cfg = info;
	for_each_child_of_node(np,temp){
		ret = of_property_read_u32(temp, "goodix,vendor-id", (unsigned int *)&info->tp_id);
		if (ret) {
			dev_err(dev, "Looking up %s property in node %s failed",
					"goodix, vendor-id", np->full_name);
			return -ENODEV;
		}
		ret = of_property_read_string(temp, "goodix,vendor-name", &info->vendor_name);
		if (ret)
			GTP_ERROR("Looking up vendor_name property failed");

		ret = goodix_parse_config(dev,temp,info);
		if(ret){
			GTP_ERROR("Unable to parse config data\n");
			return ret;
		}
		info++;
	}
/************* dtsi config parse end *********************/

	pdata->resume_in_workqueue = of_property_read_bool(np,
		"goodix,resume-in-workqueue");
	GTP_INFO("resume_in_workqueue = %d\n", pdata->resume_in_workqueue);

	pdata->init = goodix_ts_hw_init;
	pdata->release = goodix_ts_release;
	pdata->reset = goodix_ts_reset;
	pdata->power = goodix_ts_power;
	pdata->suspend = goodix_ts_sleep;
	pdata->resume = goodix_ts_wakeup;

	gtp_gpio_reset = pdata->gpio_reset;
	gtp_gpio_irq = pdata->gpio_irq;

	return 0;
}
#endif
/****************************back_off_cfg_ver************/
static ssize_t goodix_back_off_cfg_ver_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned char cfg_version = 0;
	unsigned char check_sum = 0;
	unsigned char save_cfg_version = 0;
	unsigned char save_check_sum = 0;

	u8 retry = 0;
	s32 ret = -1;
	s32 i;
	struct goodix_ts_data *goodix_ts;

	goodix_ts = i2c_get_clientdata(i2c_connect_client);
	if(buf == NULL) {
		GTP_ERROR("buf is NULL!");
		return -ENOMEM;
	}

	cfg_version = (unsigned char)simple_strtoul(buf, NULL, 10);
	GTP_DEBUG("cfg version = %d.",cfg_version);

	if(cfg_version < 'A' || cfg_version > 'Z' ) {
		cfg_version = 0x00;
	}

	save_cfg_version = config[GTP_ADDR_LENGTH];
	save_check_sum = config[goodix_ts->gtp_cfg_len];

	config[GTP_ADDR_LENGTH] = cfg_version;
	for (i = GTP_ADDR_LENGTH; i < goodix_ts->gtp_cfg_len; i++)
	{
		check_sum += config[i];
	}
	config[goodix_ts->gtp_cfg_len] = (~check_sum) + 1;

	if (goodix_ts->use_irq) {
		gtp_irq_disable(goodix_ts);
	}
	while(retry++ < 3)
	{
		gtp_reset_guitar(goodix_ts->client, 20);
		ret = gtp_send_cfg(goodix_ts->client);
		if (ret > 0)
		{
			GTP_INFO("send new config success.");
			break;
		}
	}

	if (goodix_ts->use_irq) {
		gtp_irq_enable(goodix_ts);
	}

	config[GTP_ADDR_LENGTH] = save_cfg_version;
	config[goodix_ts->gtp_cfg_len] = save_check_sum;
	return count;
}

static DEVICE_ATTR(back_off_cfg_ver,S_IRUGO|S_IWUSR, NULL, goodix_back_off_cfg_ver_store);

static struct attribute *goodix_attributes[] = {
	&dev_attr_back_off_cfg_ver.attr,
	NULL
};

static struct attribute_group goodix_attribute_group = {
	.attrs = goodix_attributes
};
/*******************************************************
Function:
Goodix touchscreen probe function.

Input:
client:	i2c device struct.
id:device id.

Output:
Executive outcomes. 0---succeed.
 *******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	struct tw_platform_data *pdata=NULL;
	char tp_info[30] = {0};
	// struct pinctrl_state* set_state;

	GTP_DEBUG_FUNC();

#if GTP_WINDOW_MODE
	//atomic_set(&gt9xx_cover_window.windows_switch, 0);

	/*** default window size ***/
	gt9xx_cover_window.win_x_min = 0;
	gt9xx_cover_window.win_y_min = 0;
	gt9xx_cover_window.win_x_max = 540;
	gt9xx_cover_window.win_y_max = 300;
#endif
	//do NOT remove these output log
	GTP_INFO("GTP Driver Version:%s",GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver build@%s,%s", __TIME__,__DATE__);
	GTP_INFO("GTP I2C Address:0x%02x", client->addr);

	i2c_connect_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct tw_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = goodix_ts_parse_dt(&client->dev, pdata);
		if (ret)
		{
			dev_err(&client->dev, "Failed to parse dt\n");
			devm_kfree(&client->dev, pdata);
			return ret;
		}
	} else
#endif
		pdata = client->dev.platform_data;

	if(!pdata)
	{
		GTP_ERROR("dev platform data is null.");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL)
	{
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	ts->gtp_rawdiff_mode = 0;
	ts->pdata = pdata;
	ts->abs_x_max = ts->pdata->screen_x;
	ts->abs_y_max = ts->pdata->screen_y;
	ts->client = client;
	ts->client->irq = gpio_to_irq(ts->pdata->gpio_irq);
	ts->fw_error = 0;
	if(!ts->pdata->init || ts->pdata->init() < 0)
	{
		GTP_ERROR("GTP request IO port failed.");
		goto exit_init_failed;
	}

	if(ts->pdata->power)
	{
		ret = ts->pdata->power(1);
		if(ret < 0)
		{
			GTP_ERROR("GTP power on failed.");
			goto exit_power_failed;
		}
	}

	if(ts->pdata->reset)
	{
		ret = ts->pdata->reset(20);
		if(ret < 0)
		{
			GTP_ERROR("GTP reset failed.");
			goto exit_reset_failed;
		}
	}

	/* Get pinctrl if target uses pinctrl */
#if 0
	ts->ts_pinctrl = devm_pinctrl_get(&(client->dev));
	if (IS_ERR(ts->ts_pinctrl)) {
		if(PTR_ERR(ts->ts_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		pr_debug("Target does not use pinctrl\n");
		ts->ts_pinctrl = NULL;
	}
	pr_info("after pinctrl_get \n");
	if(ts->ts_pinctrl){
		set_state =
			pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
		if(IS_ERR(set_state)){
			pr_info("cannot get ts pinctrl active state\n");
			ret = pinctrl_select_state(ts->ts_pinctrl, set_state);
			if(ret){
				pr_info("cannot set ts pinctrl active state\n");
			}
		}
	}
#endif
	ret = goodix_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		ret = goodix_pinctrl_select(ts, true);
		if (ret < 0)
			goto exit_init_panel_failed;
	}

	spin_lock_init(&ts->irq_lock);
	INIT_WORK(&ts->work, goodix_ts_work_func);

#if GTP_ESD_PROTECT
	ts->clk_tick_cnt = GTP_ESD_CHECK_CIRCLE * HZ;      // HZ: clock ticks in 1 second generated by system
	GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
	spin_lock_init(&ts->esd_lock);
	// ts->esd_lock = SPIN_LOCK_UNLOCKED;
#endif
	i2c_set_clientdata(client, ts);

	ret = gtp_i2c_test(client);
	if (ret < 0)
	{
		GTP_ERROR("I2C communication ERROR!");
		goto exit_i2c_test_failed;
	}

#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	gup_init_update_proc(ts);
#endif

	ret = gtp_init_panel(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP init panel failed .");
		ts->abs_x_max = ts->pdata->screen_x;
		ts->abs_y_max = ts->pdata->screen_y;
		ts->int_trigger_type = ts->pdata->irqflag;
	}

	// beg yinchao add tp_info to factory mode @20140901
	sprintf(tp_info, "TW: %s %s Version 0x%x\n", TW_IC_PREFIX_NAME,
			yl_cfg[tp_index].vendor_name, yl_cfg[tp_index].config[0]);
	//get_device_info(tp_info);
	// end

	ret = gtp_request_input_dev(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP request input dev failed");
		goto exit_init_panel_failed;
	}

	GTP_DEBUG("GTP ts->client->irq = %d",ts->client->irq);
	ret = gtp_request_irq(ts);
	if (ret < 0)
	{
		GTP_INFO("GTP works in polling mode.");
	}
	else
	{
		GTP_INFO("GTP works in interrupt mode.");
	}

	ret = gtp_read_fw_cfg_version();
	if (ret < 0)
	{
		GTP_ERROR("Read version failed.");
	}

	gtp_irq_enable(ts);

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

#if GTP_SLIDE_WAKEUP
	sprintf(wakeup_slide,"none");
#endif

	touchscreen_set_ops(&goodix_ops);
	gt9158_has_resumed = 1;
#ifdef GTP_CHARGER_STATUS_CHECK
	ts->clk_tick_cnt = GTP_CHARGER_STATUS_CHECK_CIRCLE * HZ;
	queue_delayed_work(goodix_rd_chg_wq, &goodix_rd_chg_work,
		ts->charger_check_clk_tick_cnt);
#endif

	ret= sysfs_create_group(&client->dev.kobj, &goodix_attribute_group);
	if (0 != ret)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, ret);
		sysfs_remove_group(&client->dev.kobj, &goodix_attribute_group);
	}
	else
	{
		GTP_INFO("%s sysfs_create_group() success",__func__);
	}
	GTP_INFO("probe end...");
	return 0;

exit_init_panel_failed:
exit_i2c_test_failed:
exit_reset_failed:
exit_power_failed:
	if(ts->pdata->release)
		ts->pdata->release();
exit_init_failed:
	if (ts->ts_pinctrl) {
		ret = goodix_pinctrl_select(ts, false);
		if (ret < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
	kfree(ts);
	return ret;
}
/*******************************************************
Function:
Goodix touchscreen driver release function.

Input:
client:	i2c device struct.

Output:
Executive outcomes. 0---succeed.
 *******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#elif defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif

#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

#if GTP_CHARGER_STATUS_CHECK
	destroy_workqueue(goodix_rd_chg_wq);
#endif


	if (ts)
	{
		if (ts->use_irq)
		{
			free_irq(client->irq, ts);
		}
		else
		{
			hrtimer_cancel(&ts->timer);
		}
	}

	GTP_INFO("GTP driver is removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	if(ts->pdata->release)
		ts->pdata->release();
	kfree(ts);

	return 0;
}

/*******************************************************
Function:
Early suspend function.

Input:
h:early_suspend struct.

Output:
None.
 *******************************************************/
static int goodix_ts_disable(struct input_dev *in_dev)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = i2c_get_clientdata(i2c_connect_client);
	GTP_DEBUG_FUNC();
    if(ts->gtp_is_suspend || ts->enter_update)
		return 0;
#ifdef YL_TW_DEBUG
	GTP_INFO("==goodix_ts_early_suspend==");
#endif
	ts->gtp_is_suspend = 1;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_SLIDE_WAKEUP
	sprintf(wakeup_slide,"none");

	if(support_gesture & TW_SUPPORT_GESTURE_IN_ALL)
		ret = gtp_enter_doze(ts);
	else
#endif
	{
#if GTP_CHARGER_STATUS_CHECK
	GTP_INFO("Charge check cancelled");
	cancel_delayed_work_sync(&goodix_rd_chg_work);
	flush_workqueue(goodix_rd_chg_wq);
#endif
#if GTP_SLIDE_WAKEUP
		sprintf(wakeup_slide,"none");
#endif
		if (ts->use_irq)
		{
			gtp_irq_disable(ts);
		}
		else
		{
			hrtimer_cancel(&ts->timer);
		}
		ret = gtp_enter_sleep(ts);
	}

	if (ret < 0)
	{
		GTP_ERROR("GTP early suspend failed.");
		return ret;
	}
	gt9158_has_resumed = 0;
	return ret;
}
/*******************************************************
Function:
Late resume function.

Input:
h:early_suspend struct.

Output:
None.
 *******************************************************/
static int goodix_ts_enable(struct input_dev *in_dev)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	gt9158_has_resumed = 1;

	ts = i2c_get_clientdata(i2c_connect_client);

	GTP_DEBUG_FUNC();
    if(!ts->gtp_is_suspend || ts->enter_update)
		return 0;
#if GTP_SLIDE_WAKEUP
	doze_status = DOZE_DISABLED;
	gtp_enable_irq_wake(ts, 0);
#endif
	ret = gtp_wakeup_sleep(ts);
	if (ret < 0)
	{
		GTP_ERROR("GTP later resume failed.");
	}
	if (ts->use_irq)
	{
		gtp_irq_enable(ts);
	}
	else
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif

	return ret;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *handler)
{
	struct goodix_ts_data *ts = container_of(handler, struct goodix_ts_data, early_suspend);
	goodix_ts_disable(ts->input_dev);
}
static void goodix_ts_late_resume(struct early_suspend *handler)
{
	struct goodix_ts_data *ts = container_of(handler, struct goodix_ts_data, early_suspend);
	goodix_ts_enable(ts->input_dev);
}
#elif defined(CONFIG_FB)
static int goodix_ts_suspend(struct device *dev)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	GTP_DEBUG_FUNC();
	return goodix_ts_disable(ts->input_dev);
}
static int goodix_ts_resume(struct device *dev)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	GTP_DEBUG_FUNC();
	return goodix_ts_enable(ts->input_dev);
}
#endif

#if defined(CONFIG_FB)

static void fb_notify_resume_work(struct work_struct *work)
{
	struct goodix_ts_data *ts_data =
		container_of(work, struct goodix_ts_data, fb_notify_work);
	goodix_ts_resume(&ts_data->client->dev);

}


static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct goodix_ts_data *ts_data =
		container_of(self, struct goodix_ts_data, fb_notif);
	GTP_DEBUG_FUNC();

	if (evdata && evdata->data && ts_data && ts_data->client) {
		blank = evdata->data;

		if (ts_data->pdata->resume_in_workqueue) {
			if (event == FB_EARLY_EVENT_BLANK && *blank == FB_BLANK_UNBLANK)
				schedule_work(&ts_data->fb_notify_work);
			else if (event == FB_EVENT_BLANK && *blank == FB_BLANK_POWERDOWN) {
				flush_work(&ts_data->fb_notify_work);
				goodix_ts_suspend(&ts_data->client->dev);
			}
		} else {
			if (event == FB_EVENT_BLANK) {
				if (*blank == FB_BLANK_UNBLANK)
					goodix_ts_resume(&ts_data->client->dev);
				else if (*blank == FB_BLANK_POWERDOWN)
					goodix_ts_suspend(&ts_data->client->dev);
			}
		}

	}

	return 0;
}
#endif

static const struct dev_pm_ops goodix_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend	= mxt_suspend,
	.resume	    = mxt_resume,
#endif
};

#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret=-1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = client->addr;
	msgs[0].len   = GTP_ADDR_LENGTH;
	msgs[0].buf   = &buf[0];
	//msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len - GTP_ADDR_LENGTH;
	msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
	//msgs[1].scl_rate = 300 * 1000;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	if ((retries >= 5))
	{
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	}
	return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;
	//msg.scl_rate = 300 * 1000;    // for Rockchip, etc

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)break;
		retries++;
	}
	if((retries >= 5))
	{
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
	}
	return ret;
}
/*******************************************************
Function:
switch on & off esd delayed work
Input:
client:  i2c device
on:      SWITCH_ON / SWITCH_OFF
Output:
void
 *********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);
	spin_lock(&ts->esd_lock);

	if (SWITCH_ON == on)     // switch on esd
	{
		if (!ts->esd_running)
		{
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd started");
			queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
		}
		else
		{
			spin_unlock(&ts->esd_lock);
		}
	}
	else    // switch off esd
	{
		if (ts->esd_running)
		{
			ts->esd_running = 0;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd cancelled");
			cancel_delayed_work_sync(&gtp_esd_check_work);
		}
		else
		{
			spin_unlock(&ts->esd_lock);
		}
	}
}

/*******************************************************
Function:
Initialize external watchdog for esd protect
Input:
client:  i2c device.
Output:
result of i2c write operation.
1: succeed, otherwise: failed
 *********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
	u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
	GTP_DEBUG("[Esd]Init external watchdog");
	return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
Esd protect function.
External watchdog added by meta, 2013/03/07
Input:
work: delayed work
Output:
None.
 *******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	s32 i;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	u8 esd_buf[5] = {0x80, 0x40};

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);


	if (ts->gtp_is_suspend || ts->enter_update)
	{
		GTP_INFO("Esd suspended or IC update firmware!");
		return;
	}


	for (i = 0; i < 3; i++)
	{
		ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

		GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
		if ((ret < 0))
		{
			// IIC communication problem
			continue;
		}
		else
		{
			if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
			{
				// IC works abnormally..
				u8 chk_buf[4] = {0x80, 0x40};

				gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

				GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);

				if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
				{
					i = 3;
					break;
				}
				else
				{
					continue;
				}
			}
			else
			{
				// IC works normally, Write 0x8040 0xAA, feed the dog
				esd_buf[2] = 0xAA;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
				break;
			}
		}
	}
	if (i >= 3)
	{
		GTP_ERROR("IC working abnormally! Process reset guitar.");
		esd_buf[0] = 0x42;
		esd_buf[1] = 0x26;
		esd_buf[2] = 0x01;
		esd_buf[3] = 0x01;
		esd_buf[4] = 0x01;
		gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
		msleep(50);
		gtp_reset_guitar(ts->client, 50);
		msleep(50);
		gtp_send_cfg(ts->client);
	}

	if(!ts->gtp_is_suspend)
	{
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
	}
	else
	{
		GTP_INFO("Esd suspended!");
	}
	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};

static struct of_device_id goodix_match_table[] = {
	{ .compatible = "Goodix,Goodix-TS",},
	{ },
};

static struct i2c_driver goodix_ts_driver = {
	.probe      = goodix_ts_probe,
	.remove     = goodix_ts_remove,
	.id_table   = goodix_ts_id,
	.driver = {
		.name     = GTP_I2C_NAME,
		.owner    = THIS_MODULE,
		.of_match_table = goodix_match_table,
#if defined(CONFIG_PM)
		.pm   = &goodix_pm_ops,
#endif
	},
};

/*******************************************************
Function:
Driver Install function.
Input:
None.
Output:
Executive Outcomes. 0---succeed.
 ********************************************************/
static int __init goodix_ts_init(void)
{
	s32 ret;

	GTP_DEBUG_FUNC();
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq)
	{
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif

#if GTP_CHARGER_STATUS_CHECK
	INIT_DELAYED_WORK(&goodix_rd_chg_work, yl_chg_status_changed);
	goodix_rd_chg_wq = create_workqueue("gtp_charger_status_check");
#endif

	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}
/*******************************************************
Function:
Driver uninstall function.
Input:
None.
Output:
Executive Outcomes. 0---succeed.
 ********************************************************/
static void __exit goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq)
	{
		destroy_workqueue(goodix_wq);
	}
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
