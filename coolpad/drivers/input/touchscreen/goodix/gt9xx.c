/* drivers/input/touchscreen/gt9xx.c
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
 * Version: 2.1
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/04/25
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F.
 *                  By Andrew, 2012/10/15
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
 *          2. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 */

#include <linux/irq.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "gt9xx.h"
#include <linux/input/touchscreen_yl.h>

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif


#define GOODIX_DEV_NAME	"goodix"
#define CFG_MAX_TOUCH_POINTS	5
#define GOODIX_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
#define GTP_I2C_ADDRESS_HIGH	0x14
#define GTP_I2C_ADDRESS_LOW	0x5D

#define GOODIX_VTG_MIN_UV	2600000
#define GOODIX_VTG_MAX_UV	3300000
#define GOODIX_I2C_VTG_MIN_UV	1800000
#define GOODIX_I2C_VTG_MAX_UV	1800000
#define GOODIX_VDD_LOAD_MIN_UA	0
#define GOODIX_VDD_LOAD_MAX_UA	10000
#define GOODIX_VIO_LOAD_MIN_UA	0
#define GOODIX_VIO_LOAD_MAX_UA	10000

#define RESET_DELAY_T3_US	200	/* T3: > 100us */
#define RESET_DELAY_T4		20	/* T4: > 5ms */

#define	PHY_BUF_SIZE		32
#define PROP_NAME_SIZE		24



#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#if GTP_DEBUG_ON
static const int  key_codes[] = {
	KEY_BACK, KEY_HOME, KEY_MENU, KEY_SEARCH
};
static const char *const key_names[] = {
	"Key_Back", "Key_Home", "Key_Menu", "Key_Search"
};
#endif

#endif

void gtp_reset_guitar(struct goodix_ts_data *ts, int ms);
void gtp_int_sync(struct goodix_ts_data *ts, int ms);
static int gtp_i2c_test(struct i2c_client *client);
static int goodix_ts_pinctrl_select(struct  goodix_ts_data *ts, bool on);
static int goodix_ts_gpio_configure(struct goodix_ts_data *ts, bool on);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif
#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static void gtp_esd_check_func(struct work_struct *);
static int gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, int);
#endif
struct i2c_client  *i2c_connect_client;

#if GTP_SLIDE_WAKEUP
enum doze_status {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
};
static enum doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif
bool init_done;
static u8 chip_gt9xxs;  /* true if ic is gt9xxs, like gt915s */
u8 grp_cfg_version;
/*add by yulong 2013.11.14*/
bool gtp_debug_on;
struct touch_panel_info yl_cfg[] = {
	{0x00, "Yeji"},
	{0x01, "NC"},
	{0x02, "NC"},
	{0x03, "Shenyue"},
	{0x04, "NC"},
	{0x05, "Junda"},
};
/*add end*/
/*******************************************************
Function:
	Read data from the i2c slave device.
Input:
	client:     i2c device.
	buf[0~1]:   read start address.
	buf[2~len-1]:   read data buffer.
	len:    GTP_ADDR_LENGTH + read bytes count
Output:
	numbers of i2c_msgs to transfer:
		2: succeed, otherwise: failed
*********************************************************/
int gtp_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	struct i2c_msg msgs[2];
	int ret = -EIO;
	int retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if (retries >= 5) {
#if GTP_SLIDE_WAKEUP
		/* reset chip would quit doze mode */
		if (DOZE_ENABLED == doze_status)
			return ret;
#endif
		GTP_DEBUG("I2C communication timeout, resetting chip...");
		if (init_done)
			gtp_reset_guitar(ts, 10);
		else
			dev_warn(&client->dev,
			         "<GTP> gtp_reset_guitar exit init_done=%d:\n",
			         init_done);
	}
	return ret;
}

/*******************************************************
Function:
	Write data to the i2c slave device.
Input:
	client:     i2c device.
	buf[0~1]:   write start address.
	buf[2~len-1]:   data buffer
	len:    GTP_ADDR_LENGTH + write bytes count
Output:
	numbers of i2c_msgs to transfer:
	1: succeed, otherwise: failed
*********************************************************/
int gtp_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
	struct i2c_msg msg;
	int ret = -EIO;
	int retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5)) {
#if GTP_SLIDE_WAKEUP
		if (DOZE_ENABLED == doze_status)
			return ret;
#endif
		GTP_DEBUG("I2C communication timeout, resetting chip...");
		if (init_done)
			gtp_reset_guitar(ts, 10);
		else
			dev_warn(&client->dev,
			         "<GTP> gtp_reset_guitar exit init_done=%d:\n",
			         init_done);
	}
	return ret;
}
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
int gtp_i2c_read_dbl_check(struct i2c_client *client,
                           u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;

	while (retry++ < 3) {
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len + 2))
			break;
	}
	if (retry < 3) {
		memcpy(rxbuf, confirm_buf + 2, len);
		return SUCCESS;
	} else {
		dev_err(&client->dev,
		        "i2c read 0x%04X, %d bytes, double check failed!",
		        addr, len);
		return FAIL;
	}
}

/*******************************************************
Function:
	Send config data.
Input:
	client: i2c device.
Output:
	result of i2c write operation.
	> 0: succeed, otherwise: failed
*********************************************************/
int gtp_send_cfg(struct goodix_ts_data *ts)
{
	int ret = 2;
#if GTP_DRIVER_SEND_CFG
	int retry = 0;

	if (ts->fixed_cfg) {
		GTP_INFO("Ic fixed config, no config sent!");
		ret = 2;
	} else {
		GTP_INFO("driver send config");
		for (retry = 0; retry < 5; retry++) {
			ret = gtp_i2c_write(ts->client,
			                    ts->config_data,
			                    GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
			if (ret > 0)
				break;
		}
	}
#endif

	return ret;
}

/*******************************************************
Function:
	Disable irq function
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disabled) {
		ts->irq_is_disabled = true;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Enable irq function
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disabled) {
		enable_irq(ts->client->irq);
		ts->irq_is_disabled = false;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Report touch point event
Input:
	ts: goodix i2c_client private data
	id: trackId
	x:  input x coordinate
	y:  input y coordinate
	w:  input pressure
Output:
	None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data *ts, int id, int x, int y,
                           int w)
{
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);
#endif

	GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
	Report touch release event
Input:
	ts: goodix i2c_client private data
Output:
	None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data *ts, int id)
{
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	GTP_DEBUG("Touch id[%2d] release!", id);
#else
	/*
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 0);
	*/
	input_mt_sync(ts->input_dev);
#endif
}



/*******************************************************
Function:
	Goodix touchscreen work function
Input:
	work: work struct of goodix_workqueue
Output:
	None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8 end_cmd[3] = { GTP_READ_COOR_ADDR >> 8,
	                  GTP_READ_COOR_ADDR & 0xFF, 0
	                };
	u8 point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {
		GTP_READ_COOR_ADDR >> 8,
		GTP_READ_COOR_ADDR & 0xFF
	};
	u8 touch_num = 0;
	u8 finger = 0;
	static u16 pre_touch;
	static u8 pre_key;
#if GTP_WITH_PEN
	static u8 pre_pen;
#endif
	u8 key_value = 0;
	u8 *coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i = 0;
	int ret = -1;
	struct goodix_ts_data *ts = NULL;

#if GTP_SLIDE_WAKEUP
	u8 doze_buf[3] = {0x81, 0x4B};
#endif

	GTP_DEBUG_FUNC();

	ts = container_of(work, struct goodix_ts_data, work);

	if (ts->enter_update)
		return;


#if GTP_SLIDE_WAKEUP
	if (DOZE_ENABLED == doze_status) {
		ret = gtp_i2c_read(ts->client, doze_buf, 3);
		GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
		if (ret > 0) {
			if (doze_buf[2] == 0xAA) {
				GTP_INFO("Slide(0xAA) To Light up the screen!");
				doze_status = DOZE_WAKEUP;
				input_report_key(
				    ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(
				    ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(ts->client, doze_buf, 3);
			} else if (doze_buf[2] == 0xBB) {
				GTP_INFO("Slide(0xBB) To Light up the screen!");
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				/* clear 0x814B*/
				doze_buf[2] = 0x00;
				gtp_i2c_write(ts->client, doze_buf, 3);
			} else if (0xC0 == (doze_buf[2] & 0xC0)) {
				GTP_INFO("double click to light up the screen!");
				doze_status = DOZE_WAKEUP;
				input_report_key(ts->input_dev, KEY_POWER, 1);
				input_sync(ts->input_dev);
				input_report_key(ts->input_dev, KEY_POWER, 0);
				input_sync(ts->input_dev);
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(ts->client, doze_buf, 3);
			} else {
				gtp_enter_doze(ts);
			}
		}
		if (ts->use_irq)
			gtp_irq_enable(ts);

		return;
	}
#endif

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		dev_err(&ts->client->dev,
		        "I2C transfer error. errno:%d\n ", ret);
		goto exit_work_func;
	}

	finger = point_data[GTP_ADDR_LENGTH];
	if ((finger & 0x80) == 0)
		goto exit_work_func;

	touch_num = finger & 0x0f;
	if (touch_num > GTP_MAX_TOUCH)
		goto exit_work_func;

	if (touch_num > 1) {
		u8 buf[8 * GTP_MAX_TOUCH] = { (GTP_READ_COOR_ADDR + 10) >> 8,
		                              (GTP_READ_COOR_ADDR + 10) & 0xff
		                            };

		ret = gtp_i2c_read(ts->client, buf,
		                   2 + 8 * (touch_num - 1));
		memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
	}

#if GTP_HAVE_TOUCH_KEY
	key_value = point_data[3 + 8 * touch_num];

	if (key_value || pre_key) {
		for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
#if GTP_DEBUG_ON
			for (ret = 0; ret < 4; ++ret) {
				if (key_codes[ret] == touch_key_array[i]) {
					GTP_DEBUG("Key: %s %s",
					          key_names[ret],
					          (key_value & (0x01 << i))
					          ? "Down" : "Up");
					break;
				}
			}
#endif
			input_report_key(ts->input_dev,
			                 touch_key_array[i], key_value & (0x01<<i));
		}
		touch_num = 0;
		pre_touch = 0;
	}
#endif
	pre_key = key_value;

	GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT

#if GTP_WITH_PEN
	if (pre_pen && (touch_num == 0)) {
		GTP_DEBUG("Pen touch UP(Slot)!");
		input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
		input_mt_slot(ts->input_dev, 0);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
		pre_pen = 0;
	}
#endif
	if (pre_touch || touch_num) {
		s32 pos = 0;
		u16 touch_index = 0;
		u8  report_num = 0;

		coor_data = &point_data[3];
		if (touch_num) {
			id = coor_data[pos] & 0x0F;
#if GTP_WITH_PEN
			id = coor_data[pos];
			if (id == 128) {
				GTP_DEBUG("Pen touch DOWN(Slot)!");
				input_x  = coor_data[pos + 1]
				           | (coor_data[pos + 2] << 8);
				input_y  = coor_data[pos + 3]
				           | (coor_data[pos + 4] << 8);
				input_w  = coor_data[pos + 5]
				           | (coor_data[pos + 6] << 8);
#if GTP_CHANGE_X2Y
				GTP_SWAP(x, y);
#endif
				input_report_key(ts->input_dev,
				                 BTN_TOOL_PEN, 1);
				input_mt_slot(ts->input_dev, 0);
				input_report_abs(ts->input_dev,
				                 ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev,
				                 ABS_MT_POSITION_X, input_x);
				input_report_abs(ts->input_dev,
				                 ABS_MT_POSITION_Y, input_y);
				input_report_abs(ts->input_dev,
				                 ABS_MT_TOUCH_MAJOR, input_w);
				GTP_DEBUG("Pen/Stylus: (%d, %d)[%d]",
				          input_x, input_y, input_w);
				pre_pen = 1;
				pre_touch = 0;
				touch_num = 0;
			}
#endif

			touch_index |= (0x01<<id);
		}

		GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",
		          id, touch_index, pre_touch);
		for (i = 0; i < GTP_MAX_TOUCH; i++) {
#if GTP_WITH_PEN
			if (pre_pen == 1)
				break;
#endif
			if (touch_index & (0x01<<i)) {
				input_x = coor_data[pos + 1] |
				          coor_data[pos + 2] << 8;
				input_y = coor_data[pos + 3] |
				          coor_data[pos + 4] << 8;
				input_w = coor_data[pos + 5] |
				          coor_data[pos + 6] << 8;

				gtp_touch_down(ts, id,
				               input_x, input_y, input_w);
				pre_touch |= 0x01 << i;
				report_num++;
				if(report_num < touch_num) {
					pos += 8;
					id = coor_data[pos] & 0x0F;
					touch_index |= (0x01<<id);
				}
			} else {
				gtp_touch_up(ts, i);
				pre_touch &= ~(0x01 << i);
			}
		}
	}
#else
	if (touch_num) {
		for (i = 0; i < touch_num; i++)	{
			coor_data = &point_data[i * 8 + 3];

			id = coor_data[0];      /*  & 0x0F */
			input_x  = coor_data[1] | (coor_data[2] << 8);
			input_y  = coor_data[3] | (coor_data[4] << 8);
			input_w  = coor_data[5] | (coor_data[6] << 8);

#if GTP_WITH_PEN
			if (id == 128) {
				GTP_DEBUG("Pen touch DOWN!");
				input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
				input_report_key(ts->input_dev, BTN_TOOL_PEN, 1);
				pre_pen = 1;
				id = 0;
			}
#endif

			gtp_touch_down(ts, id, input_x, input_y, input_w);
		}
	} else if (pre_touch) {

#if GTP_WITH_PEN
		if (pre_pen == 1) {
			GTP_DEBUG("Pen touch UP!");
			input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
			pre_pen = 0;
		}
#endif

		GTP_DEBUG("Touch Release!");
		gtp_touch_up(ts, 0);
	}

	pre_touch = touch_num;
#endif

	input_sync(ts->input_dev);

exit_work_func:
	if (!ts->gtp_rawdiff_mode) {
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0)
			dev_warn(&ts->client->dev, "I2C write end_cmd error!\n");

	}
	if (ts->use_irq)
		gtp_irq_enable(ts);

	return;
}

/*******************************************************
Function:
	Timer interrupt service routine for polling mode.
Input:
	timer: timer struct pointer
Output:
	Timer work mode.
	HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data
	*ts = container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(ts->goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000),
	              HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
	External interrupt service routine for interrupt mode.
Input:
	irq:  interrupt number.
	dev_id: private data pointer
Output:
	Handle Result.
	IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();

	gtp_irq_disable(ts);

	queue_work(ts->goodix_wq, &ts->work);

	return IRQ_HANDLED;
}
/*******************************************************
Function:
	Synchronization.
Input:
	ms: synchronization time in millisecond.
Output:
	None.
*******************************************************/
void gtp_int_sync(struct goodix_ts_data *ts, int ms)
{
	gpio_direction_output(ts->pdata->irq_gpio, 0);
	msleep(ms);
	gpio_direction_input(ts->pdata->irq_gpio);
}

/*******************************************************
Function:
	Reset chip.
Input:
	ms: reset time in millisecond, must >10ms
Output:
	None.
*******************************************************/
void gtp_reset_guitar(struct goodix_ts_data *ts, int ms)
{
	GTP_DEBUG_FUNC();

	/* This reset sequence will selcet I2C slave address */
	gpio_direction_output(ts->pdata->reset_gpio, 0);
	msleep(ms);						/* T2: > 10ms */
	/* HIGH: 0x28&0x29, LOW: 0xBA&0xBB*/
	if (ts->client->addr == GTP_I2C_ADDRESS_HIGH)
		gpio_direction_output(ts->pdata->irq_gpio, 1);
	else
		gpio_direction_output(ts->pdata->irq_gpio, 0);

	usleep(RESET_DELAY_T3_US);				/* T3: > 100us */
	gpio_direction_output(ts->pdata->reset_gpio, 1);
	msleep(RESET_DELAY_T4);					/* T4: > 5ms */

	gpio_direction_input(ts->pdata->reset_gpio);		/*end select I2C slave addr*/

	gtp_int_sync(ts, 50);

#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(ts->client);
#endif
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
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
	u8 i2c_control_buf[3] = {
		(u8)(GTP_REG_SLEEP >> 8),
		(u8)GTP_REG_SLEEP, 8
	};

	GTP_DEBUG_FUNC();

#if GTP_DBL_CLK_WAKEUP
	i2c_control_buf[2] = 0x09;
#endif
	gtp_irq_disable(ts);

	GTP_DEBUG("entering doze mode...");
	while (retry++ < 5) {
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0) {
			GTP_DEBUG(
			    "failed to set doze flag into 0x8046, %d",
			    retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			doze_status = DOZE_ENABLED;
			GTP_INFO("GTP has been working in doze mode!");
			gtp_irq_enable(ts);
			return ret;
		}
		msleep(10);
	}
	dev_err(&ts->client->dev, "GTP send doze cmd failed.\n");
	gtp_irq_enable(ts);
	return ret;
}
#else
/*******************************************************
Function:
	Enter sleep mode.
Input:
	ts: private data.
Output:
	Executive outcomes.
	1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data  *ts)
{
	s8 ret = -1;
	s8 retval = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {
		(u8)(GTP_REG_SLEEP >> 8),
		(u8)GTP_REG_SLEEP, 5
	};

	GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
#if GTP_POWER_CTRL_EN
	goodix_power_off(ts);
	GTP_INFO("GTP power down!");
	return 0;
#endif
	return ret;
#else
	ret = gpio_direction_output(ts->pdata->irq_gpio, 0);
	usleep(5000);
	while (retry++ < 5) {
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			GTP_INFO("GTP enter sleep!\n");
			if (ts->ts_pinctrl) {
				retval = goodix_ts_pinctrl_select(ts, false);
				if (retval < 0)
					pr_err("Cannot get idle pinctrl state\n");
			}
			retval = goodix_ts_gpio_configure(ts, false);
			if (retval < 0)
				pr_err("failed to put gpios in suspend state\n");
			return ret;
		}
		msleep(10);
	}
	dev_err(&ts->client->dev, "GTP send sleep cmd failed.\n");
#endif
	return ret;
}
#endif

/*******************************************************
Function:
	Wakeup from sleep.
Input:
	ts: private data.
Output:
	Executive outcomes.
	>0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
#if GTP_POWER_CTRL_EN
	goodix_power_on(ts);
#endif
	gtp_reset_guitar(ts, 20);

	ret = gtp_send_cfg(ts);
	if (ret > 0) {
		GTP_INFO("Wakeup sleep send config success.");
		return 1;
	}

#else
	while (retry++ < 10) {
#if GTP_SLIDE_WAKEUP
		if (DOZE_WAKEUP != doze_status) {	/* wakeup not by slide */
			doze_status = DOZE_DISABLED;
			gtp_irq_disable(ts);
			gtp_reset_guitar(ts, 10);
			gtp_irq_enable(ts);
		} else {             /* wakeup by slide */
			doze_status = DOZE_DISABLED;
#if GTP_ESD_PROTECT
			gtp_init_ext_watchdog(ts->client);
#endif
		}
#else
		if (chip_gt9xxs == 1) {
			gtp_reset_guitar(ts, 10);
		} else {
			if (ts->ts_pinctrl) {
				ret = goodix_ts_pinctrl_select(ts, true);
				if (ret < 0)
					pr_err("Cannot get idle pinctrl state\n");
			}
			ret = goodix_ts_gpio_configure(ts, true);
			if (ret < 0)
				pr_err("failed to put gpios in suspend state\n");

			ret = gpio_direction_output(ts->pdata->irq_gpio, 1);
			usleep(5000);
		}
#endif
		ret = gtp_i2c_test(ts->client);
		if (ret > 0) {
			GTP_INFO("GTP wakeup sleep.\n");
#if (!GTP_SLIDE_WAKEUP)
			if (chip_gt9xxs == 0) {
				gtp_int_sync(ts, 25);
				msleep(20);
#if GTP_ESD_PROTECT
				gtp_init_ext_watchdog(ts->client);
#endif
			}
#endif
			return ret;
		}
		gtp_reset_guitar(ts, 20);
	}
#endif

	dev_err(&ts->client->dev, "GTP wakeup sleep failed.\n");
	return ret;
}
#endif /* !CONFIG_HAS_EARLYSUSPEND && !CONFIG_FB*/

/*******************************************************
Function:
	Initialize gtp.
Input:
	ts: goodix private data
Output:
	Executive outcomes.
	> =0: succeed, otherwise: failed
*******************************************************/
static int gtp_init_panel(struct goodix_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	unsigned char *config_data = NULL;
	int ret = -EIO;

#if GTP_DRIVER_SEND_CFG
	int i;
	u8 check_sum = 0;
	u8 opr_buf[16];
	u8 sensor_id = 0;

	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++)
		GTP_DEBUG("Config Groups(%d) Lengths: %d",
		i, ts->pdata->config_data_len[i]);
	ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
	if (SUCCESS == ret) {
		if (opr_buf[0] != 0xBE) {
			ts->fw_error = 1;
			dev_err(&client->dev,
			        "Firmware error, no config sent!");
			return -EINVAL;
		}
	}
	for (i = 1; i < GOODIX_MAX_CFG_GROUP; i++) {
		if (ts->pdata->config_data_len[i])
			break;
	}
	if (i == GOODIX_MAX_CFG_GROUP) {
		sensor_id = 0;
	} else {
		ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID,
		                             &sensor_id, 1);
		if (SUCCESS == ret) {
			if (sensor_id >= GOODIX_MAX_CFG_GROUP) {
				dev_err(&client->dev,
				        "Invalid sensor_id(0x%02X), No Config Sent!",
				        sensor_id);
				return -EINVAL;
			}
		} else {
			dev_err(&client->dev,
			        "Failed to get sensor_id, No config sent!");
			return -EINVAL;
		}
	}
	GTP_DEBUG("Sensor_ID: %d", sensor_id);

	if (ts->pdata->config_data_len[sensor_id] < GTP_CONFIG_MIN_LENGTH ||
		!ts->pdata->config_data[sensor_id]) {
		dev_err(&client->dev,
		        "Sensor_ID(%d) matches with NULL\
				or INVALID CONFIG GROUP! NO Config Sent!\
				You need to check you header file CFG_GROUP section!\n",\
		        sensor_id);
		return -EINVAL;
	}
	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA,
	                             &opr_buf[0], 1);

	if (ret == SUCCESS) {
		GTP_DEBUG("Config Version: 0x%02X; IC Config Version: 0x%02X",
			ts->pdata->config_data[sensor_id][GTP_ADDR_LENGTH],
			opr_buf[0]);
		if (opr_buf[0] < 90) {
			/* backup group config version */
			grp_cfg_version =
			ts->pdata->config_data[sensor_id][GTP_ADDR_LENGTH];
			/*ts->pdata->config_data[sensor_id][GTP_ADDR_LENGTH]
					=0x00;*/
			ts->fixed_cfg = 0;
		} else {
			/* treated as fixed config, not send config */
			dev_warn(&client->dev,
			         "Ic fixed config with config version(%d, 0x%02X)",
			         opr_buf[0], opr_buf[0]);
			ts->fixed_cfg = 1;
		}
	} else {
		dev_err(&client->dev,
		        "Failed to get ic config version!No config sent!");
		return -EINVAL;
	}

		config_data = ts->pdata->config_data[sensor_id];
		ts->config_data = ts->pdata->config_data[sensor_id];
		ts->gtp_cfg_len = ts->pdata->config_data_len[sensor_id];

#if GTP_CUSTOM_CFG
	if (ts->pdata->x_max |ts->pdata->y_max) {
		config_data[RESOLUTION_LOC]     = ts->pdata->x_max & 0xff;
		config_data[RESOLUTION_LOC + 1] = (ts->pdata->x_max >> 8) & 0xff;
		config_data[RESOLUTION_LOC + 2] = ts->pdata->y_max & 0xff;
		config_data[RESOLUTION_LOC + 3] = (ts->pdata->y_max >> 8) & 0xff;
	} else {
		config_data[RESOLUTION_LOC] =
		    (unsigned char)(GTP_MAX_WIDTH && 0xFF);
		config_data[RESOLUTION_LOC + 1] =
		    (unsigned char)(GTP_MAX_WIDTH >> 8);
		config_data[RESOLUTION_LOC + 2] =
		    (unsigned char)(GTP_MAX_HEIGHT && 0xFF);
		config_data[RESOLUTION_LOC + 3] =
		    (unsigned char)(GTP_MAX_HEIGHT >> 8);
	}
	if (ts->pdata->irq_gpio_flag >0 && ts->pdata->irq_gpio_flag == IRQF_TRIGGER_RISING)
		config_data[TRIGGER_LOC] &= 0xfc;
	else if(ts->pdata->irq_gpio_flag >0 && ts->pdata->irq_gpio_flag == IRQF_TRIGGER_FALLING) {
		config_data[TRIGGER_LOC] &= 0xfc;
		config_data[TRIGGER_LOC] |= 0x01;
	} else if (GTP_INT_TRIGGER == 0)
		config_data[TRIGGER_LOC] &= 0xfc;
	else if (GTP_INT_TRIGGER == 1) {
		config_data[TRIGGER_LOC] &= 0xfc;
		config_data[TRIGGER_LOC] |= 0x01;
	}
	ts->int_trigger_type = config_data[TRIGGER_LOC] & 0x03;	 /*GT9xx IC-> 0:RISING 1:FALLING */
#endif  /* !GTP_CUSTOM_CFG */

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
		check_sum += config_data[i];

	config_data[ts->gtp_cfg_len] = (~check_sum) + 1;
#else /* DRIVER NOT SEND CONFIG */
	ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gtp_i2c_read(ts->client, config_data,
	                   ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0) {
		dev_err(&client->dev,
		        "Read Config Failed, Using DEFAULT Resolution & INT Trigger!\n");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER; /*default */
	}
	ts->int_trigger_type = config_data[TRIGGER_LOC] & 0x03;
#endif /* !DRIVER NOT SEND CONFIG */

	GTP_DEBUG_FUNC();
	if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
		ts->abs_x_max = (config_data[RESOLUTION_LOC + 1] << 8)
		                + config_data[RESOLUTION_LOC];
		ts->abs_y_max = (config_data[RESOLUTION_LOC + 3] << 8)
		                + config_data[RESOLUTION_LOC + 2];
		ts->int_trigger_type = (config_data[TRIGGER_LOC]) & 0x03;
	}
	ret = gtp_send_cfg(ts);
	if (ret < 0)
		dev_err(&client->dev, "%s: Send config error.\n", __func__);

	GTP_DEBUG("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
	          ts->abs_x_max, ts->abs_y_max,
	          ts->int_trigger_type);

	msleep(10);
	return ret;
}

/*******************************************************
Function:
	Read chip version.
Input:
	client:  i2c device
	version: buffer to keep ic firmware version
Output:
	read operation return.
	2: succeed, otherwise: failed
*******************************************************/
int gtp_read_version(struct i2c_client *client, u16 *version)
{
	int ret = -EIO;
	u8 buf[8] = { GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff };

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed.\n");
		return ret;
	}

	if (version)
		*version = (buf[7] << 8) | buf[6];

	if (buf[5] == 0x00) {
		GTP_INFO("IC Version: %c%c%c_%02x%02x\n", buf[2],
		         buf[3], buf[4], buf[7], buf[6]);
	} else {
		if (buf[5] == 'S' || buf[5] == 's')
			chip_gt9xxs = 1;
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x\n", buf[2],
		         buf[3], buf[4], buf[5], buf[7], buf[6]);
	}
	return ret;
}

/*******************************************************
Function:
	I2c test Function.
Input:
	client:i2c client.
Output:
	Executive outcomes.
	2: succeed, otherwise failed.
*******************************************************/
static int gtp_i2c_test(struct i2c_client *client)
{
	u8 buf[3] = { GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };
	int retry = 5;
	int ret = -EIO;

	GTP_DEBUG_FUNC();

	while (retry--) {
		ret = gtp_i2c_read(client, buf, 3);
		if (ret > 0)
			return ret;
		dev_err(&client->dev, "GTP i2c test failed time %d.\n", retry);
		msleep(20);
	}
	return ret;
}

/*******************************************************
Function:
	Request gpio(INT & RST) ports.
Input:
	ts: private data.
Output:
	Executive outcomes.
	= 0: succeed, != 0: failed
*******************************************************/
static int gtp_request_io_port(struct goodix_ts_data *ts)
{
	struct i2c_client *client = ts->client;
	struct goodix_ts_platform_data *pdata = ts->pdata;
	int ret;
	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request(pdata->irq_gpio, "goodix_ts_irq_gpio");
		if (ret) {
			dev_err(&client->dev, "irq gpio request failed\n");
			return ret;
		}
		ret = gpio_direction_input(pdata->irq_gpio);
		if (ret) {
			dev_err(&client->dev,
			        "set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	} else {
		dev_err(&client->dev, "irq gpio is invalid!\n");
		ret = -EINVAL;
		goto free_irq_gpio;
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "goodix_ts__reset_gpio");
		if (ret) {
			dev_err(&client->dev, "reset gpio request failed\n");
			goto free_reset_gpio;
		}

		ret = gpio_direction_output(pdata->reset_gpio, 0);
		if (ret) {
			dev_err(&client->dev,
			        "set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
	} else {
		dev_err(&client->dev, "reset gpio is invalid!\n");
		ret = -EINVAL;
		goto free_reset_gpio;
	}
	gpio_direction_input(pdata->reset_gpio);
	gtp_reset_guitar(ts, 20);
	return ret;
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

	return ret;
}

/*******************************************************
Function:
	Request interrupt.
Input:
	ts: private data.
Output:
	Executive outcomes.
	0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	int ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG("INT trigger type:%x, irq=%d", ts->int_trigger_type,
	          ts->client->irq);

	ret = request_irq(ts->client->irq, goodix_ts_irq_handler,
	                  irq_table[ts->int_trigger_type],
	                  ts->client->name, ts);
	if (ret) {
		dev_err(&ts->client->dev, "Request IRQ failed!ERRNO:%d.\n",
		        ret);
		gpio_direction_input(ts->pdata->irq_gpio);
		gpio_free(ts->pdata->irq_gpio);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC,
		             HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0),
		              HRTIMER_MODE_REL);
		ts->use_irq = false;
		return -1;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = true;
		return 0;
	}
}

/*******************************************************
Function:
	Request input device Function.
Input:
	ts:private data.
Output:
	Executive outcomes.
	0: succeed, otherwise: failed.
*******************************************************/
static int gtp_request_input_dev(struct goodix_ts_data *ts)
{
	int ret;
	char phys[PHY_BUF_SIZE];
#if GTP_HAVE_TOUCH_KEY
	int index = 0;
#endif

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		dev_err(&ts->client->dev,
		        "Failed to allocate input device.\n");
		return -ENOMEM;
	}

#if GTP_ICS_SLOT_REPORT
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	input_mt_init_slots(ts->input_dev, 10);/* in case of "out of memory" */
#else
	/*	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);*/
#endif

#if GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++) {
		input_set_capability(ts->input_dev,
		                     EV_KEY, touch_key_array[index]);
	}
#endif

#if GTP_SLIDE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_WITH_PEN
	/* pen support */
	__set_bit(BTN_TOOL_PEN, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	__set_bit(INPUT_PROP_POINTER, ts->input_dev->propbit);
#endif

#if GTP_CHANGE_X2Y
	GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
	                     0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
	                     0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
	                     0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
	                     0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
	                     0, 5, 0, 0);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	snprintf(phys, PHY_BUF_SIZE, "input/ts");
	ts->input_dev->name = GOODIX_DEV_NAME;
	ts->input_dev->phys = phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	input_set_drvdata(ts->input_dev, ts);
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&ts->client->dev,
		        "Register %s input device failed.\n",
		        ts->input_dev->name);
		goto exit_free_inputdev;
	}

	return 0;

exit_free_inputdev:
	input_free_device(ts->input_dev);
	ts->input_dev = NULL;
	return ret;
}
#if GTP_POWER_CTRL_EN
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
	       regulator_set_optimum_mode(reg, load_uA) : 0;
}
#endif
/**
 * goodix_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_on(struct goodix_ts_data *ts)
{
	int ret = 0;
#if GTP_POWER_CTRL_EN
	if (!IS_ERR(ts->avdd)) {
		ret = reg_set_optimum_mode_check(ts->avdd,
		                                 GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
			        "Regulator avdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_avdd;
		}
		ret = regulator_enable(ts->avdd);
		if (ret) {
			dev_err(&ts->client->dev,
			        "Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		}
	}
/*
	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
		                            GOODIX_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->client->dev,
			        "Regulator set_vtg failed vdd ret=%d\n", ret);
			goto err_set_vtg_vdd;
		}
		ret = reg_set_optimum_mode_check(ts->vdd,
		                                 GOODIX_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
			        "Regulator vdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_vdd;
		}
		ret = regulator_enable(ts->vdd);
		if (ret) {
			dev_err(&ts->client->dev,
			        "Regulator vdd enable failed ret=%d\n", ret);
			goto err_enable_vdd;
		}
	}
*/
	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
		                            GOODIX_I2C_VTG_MAX_UV);
		if (ret) {
			dev_err(&ts->client->dev,
			        "Regulator set_vtg failed vcc_i2c ret=%d\n",
			        ret);
			goto err_set_vtg_vcc_i2c;
		}
		ret = reg_set_optimum_mode_check(ts->vcc_i2c,
		                                 GOODIX_VIO_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&ts->client->dev,
			        "Regulator vcc_i2c set_opt failed rc=%d\n",
			        ret);
			goto err_set_opt_vcc_i2c;
		}
		ret = regulator_enable(ts->vcc_i2c);
		if (ret) {
			dev_err(&ts->client->dev,
			        "Regulator vcc_i2c enable failed ret=%d\n",
			        ret);
			regulator_disable(ts->vdd);
			goto err_enable_vcc_i2c;
		}
	}
	return 0;

err_enable_vcc_i2c:
err_set_opt_vcc_i2c:
	if (!IS_ERR(ts->vcc_i2c))
		regulator_set_voltage(ts->vcc_i2c, 0, GOODIX_I2C_VTG_MAX_UV);
err_set_vtg_vcc_i2c:
	if (!IS_ERR(ts->avdd))
		regulator_disable(ts->avdd);
err_enable_avdd:
err_set_opt_avdd:
#endif
	return ret;
}

/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_off(struct goodix_ts_data *ts)
{
	int ret = 0;
#if GTP_POWER_CTRL_EN
	if (!IS_ERR(ts->vcc_i2c)) {
		ret = regulator_set_voltage(ts->vcc_i2c, 0,
		                            GOODIX_I2C_VTG_MAX_UV);
		if (ret < 0)
			dev_err(&ts->client->dev,
			        "Regulator vcc_i2c set_vtg failed ret=%d\n",
			        ret);
		ret = regulator_disable(ts->vcc_i2c);
		if (ret)
			dev_err(&ts->client->dev,
			        "Regulator vcc_i2c disable failed ret=%d\n",
			        ret);
	}

/*	if (!IS_ERR(ts->vdd)) {
		ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
		if (ret < 0)
			dev_err(&ts->client->dev,
			        "Regulator vdd set_vtg failed ret=%d\n", ret);
		ret = regulator_disable(ts->vdd);
		if (ret)
			dev_err(&ts->client->dev,
			        "Regulator vdd disable failed ret=%d\n", ret);
	}
*/
	if (!IS_ERR(ts->avdd)) {
		ret = regulator_disable(ts->avdd);
		if (ret)
			dev_err(&ts->client->dev,
			        "Regulator avdd disable failed ret=%d\n", ret);
	}
#endif
	return ret;
}

/**
 * goodix_power_init - Initialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_init(struct goodix_ts_data *ts)
{
	int ret = 0;
#if GTP_POWER_CTRL_EN

	ts->avdd = regulator_get(&ts->client->dev, "avdd");
	if (IS_ERR(ts->avdd)) {
		ret = PTR_ERR(ts->avdd);
		dev_info(&ts->client->dev,
		         "Regulator get failed avdd ret=%d\n", ret);
	}
/*
	ts->vdd = regulator_get(&ts->client->dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_info(&ts->client->dev,
		         "Regulator get failed vdd ret=%d\n", ret);
	}
*/
	ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc-i2c");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		dev_info(&ts->client->dev,
		         "Regulator get failed vcc_i2c ret=%d\n", ret);
	}
#endif
	return ret;
}

/**
 * goodix_power_deinit - Deinitialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_deinit(struct goodix_ts_data *ts)
{
#if GTP_POWER_CTRL_EN

	regulator_put(ts->vcc_i2c);
	regulator_put(ts->avdd);
#endif
	return 0;
}

/***********************for yulong factory test**********************/
#pragma pack(1)
typedef struct {
	u8  hw_info[4];		/* hardware info */
	u8  pid[8];		/* product id   */
	u16 vid;		/* version id   */
} st_fw_head;
#pragma pack()

extern u8 gup_get_ic_fw_msg(struct i2c_client *client);
extern u8 gup_check_update_file(struct i2c_client *client, st_fw_head *fw_head,
                                u8 *path);

/*****************************************************
fuction:
	goodix_active - return goodix working status
input:
	none
output:
	Returns 1--active  0--not active
*****************************************************/
int goodix_active(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}
/****************************************************
fuction:
	check firmware if need update
input:
	none
output:

*****************************************************/
int goodix_firmware_need_update(void)
{
	st_fw_head fw_head;
	int ret = -1;

	GTP_INFO(" check if need firmware update...\n");
	GTP_DEBUG("enter %s\n",__FUNCTION__);

	ret = gup_get_ic_fw_msg(i2c_connect_client);
	if(FAIL == ret) {
		GTP_ERROR("[update_proc]get ic message fail.");
		return FAIL;
	}
	ret = gup_check_update_file(i2c_connect_client, &fw_head, 0);
	return ret;
}

/****************************************************
fuction:
	do firmware update
input:
	none
output:

*****************************************************/
int goodix_firmware_do_update(void)
{
	int ret = -1;

	GTP_DEBUG("enter %s\n",__FUNCTION__);
	ret = goodix_firmware_need_update();
	if(ret) {
		struct goodix_ts_data *ts = NULL;
		GTP_DEBUG_FUNC();
		ts = i2c_get_clientdata(i2c_connect_client);
#if GTP_AUTO_UPDATE
		if(0 == gup_init_update_proc(ts))
			return -1;
		else
			return 0;
#endif
		ret = -1;
	}

	return ret;
}

/****************************************************
fuction:
	check if need calibrate
input:
	none
output:

*****************************************************/
int goodix_need_calibrate(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

/****************************************************
fuction:
	system write "calibrate"
input:
	none
output:

****************************************************/

int goodix_calibrate(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 0;
}

/****************************************************
fuction:
	get firmware version
input:
	version:store firmware version string pointer
output:

****************************************************/
int goodix_get_firmware_version(char * version )
{
	int ret = -1;
	u8 cfg[3] = { 0 };
	u8 sensor_id[3] = { 0 };

	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};
	ret = gtp_i2c_read(i2c_connect_client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	cfg[0] = GTP_REG_CONFIG_DATA >> 8;
	cfg[1] = GTP_REG_CONFIG_DATA & 0xff;
	ret = gtp_i2c_read(i2c_connect_client, cfg, 3);
	if( ret < 0 ) {
		GTP_ERROR("Read config version failed.\n");
		return ret;
	}

	sensor_id[0] = GTP_REG_SENSOR_ID >> 8;
	sensor_id[1] = GTP_REG_SENSOR_ID & 0xff;
	ret = gtp_i2c_read(i2c_connect_client, sensor_id, 3);
	if( ret < 0 ) {
		GTP_ERROR("Read sensor_id failed.\n");
		return ret;
	}

	GTP_INFO("sensor_id:%d\n",sensor_id[2]);
	if (buf[5] == 0x00)
		return snprintf(version, 64, "%s:GT%c%c%c:0x%4x:0x%2x",
		                yl_cfg[sensor_id[2]].tp_name, buf[2], buf[3],
		                buf[4], (buf[7] << 8)|buf[6], cfg[2]);
	else
		return snprintf(version, 64, "%s:GT%c%c%c%c:0x%4x:0x%2x",
		                yl_cfg[sensor_id[2]].tp_name, buf[2], buf[3],
		                buf[4], buf[5], (buf[7] << 8)|buf[6], cfg[2]);
}

/****************************************************
fuction:
	system write "reset"
input:
	none
output:

****************************************************/
int goodix_reset_touchscreen(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

enum touch_mode_type goodix_get_mode(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

int goodix_set_mode(enum touch_mode_type work_mode)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

enum touch_orientation_type goodix_get_orientation(void)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

int goodix_set_orientation(enum touch_orientation_type oreitate)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

int goodix_read_regs(char * buf)
{
	int ret = -1;
	u8 cfg[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = { 0 };

	GTP_DEBUG_FUNC();

	cfg[0] = GTP_REG_CONFIG_DATA >> 8;
	cfg[1] = GTP_REG_CONFIG_DATA & 0xff;
	ret = gtp_i2c_read(i2c_connect_client, cfg, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
	if( ret < 0 ) {
		GTP_ERROR("Read config version failed.\n");
		return ret;
	}
	GTP_INFO("TW panel config data:\n");
	for(ret=2; ret<GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH; ret += 10) {
		printk(KERN_NOTICE
			"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				cfg[ret],cfg[ret + 1],cfg[ret + 2],
				cfg[ret + 3],cfg[ret + 4],cfg[ret + 5],
				cfg[ret + 6],cfg[ret + 7],cfg[ret + 8],
				cfg[ret + 9]);
	}
	return 1;
}

int goodix_write_regs(const char * buf)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

/****************************************************
fuction:
	set goodix debug on or off
input:
	on:1  off:0
output:
	return 1
****************************************************/
int goodix_debug(int val)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	if(val)
		gtp_debug_on = 1;
	else
		gtp_debug_on = 0;
	return 1;
}
int goodix_get_vendor(char *vendor)
{
	int ret = -1;
	u8 sensor_id[3] = {0};
	u8 buf[6] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};
	GTP_DEBUG_FUNC();
	ret = gtp_i2c_read(i2c_connect_client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}
	sensor_id[0] = GTP_REG_SENSOR_ID >> 8;
	sensor_id[1] = GTP_REG_SENSOR_ID & 0xff;
	ret = gtp_i2c_read(i2c_connect_client, sensor_id, 3);
	if(ret < 0) {
		GTP_ERROR("Read sensor_id failed.\n");
		return ret;
	}
	return snprintf(vendor, 64, "goodix:GT%c%c%c%c:%s",
		buf[2], buf[3], buf[4], buf[5] == 0x0 ? '0' : buf[5],
		yl_cfg[sensor_id[2]].tp_name);
}

struct touchscreen_funcs goodix_ops= {
	.touch_id					= 0,
	.touch_type					= 1,
	.active						= goodix_active,
	.firmware_need_update				= goodix_firmware_need_update,
	.firmware_do_update				= goodix_firmware_do_update,
	.need_calibrate					= goodix_need_calibrate,
	.calibrate					= goodix_calibrate,
	.get_firmware_version				= goodix_get_firmware_version,
	.reset_touchscreen				= goodix_reset_touchscreen,
	.get_mode					= goodix_get_mode,
	.set_mode					= goodix_set_mode,
	.get_orientation				= goodix_get_orientation,
	.set_orientation				= goodix_set_orientation,
	.read_regs					= goodix_read_regs,
	.write_regs					= goodix_write_regs,
	.debug						= goodix_debug,
	.get_vendor					= goodix_get_vendor,
};

#if CONFIG_OF
static int goodix_ts_get_dt_coords(struct device *dev, char *name,
                                   struct goodix_ts_platform_data *pdata)
{
	struct property *prop;
	struct device_node *np = dev->of_node;
	int rc;
	u32 coords[GOODIX_COORDS_ARR_SIZE];

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, coords,
	                                GOODIX_COORDS_ARR_SIZE);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "goodix,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "goodix,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int goodix_parse_dt(struct device *dev,
                           struct goodix_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];
	char prop_name[PROP_NAME_SIZE];
	int i, read_cfg_num;

	rc = goodix_ts_get_dt_coords(dev, "goodix,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = goodix_ts_get_dt_coords(dev, "goodix,display-coords", pdata);
	if (rc)
		return rc;
	GTP_DEBUG("display:%d %d %d %d \n", pdata->x_min, pdata->y_min,
	          pdata->x_max, pdata->y_max);
	GTP_DEBUG("panel:%d %d %d %d \n", pdata->panel_minx,
	          pdata->panel_miny,
	          pdata->panel_maxx,
	          pdata->panel_maxy);

	pdata->i2c_pull_up = of_property_read_bool(np,
	                     "goodix,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
	                         "goodix,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpio",
	                    0, &pdata->reset_gpio_flag);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "interrupt-gpio",
	                  0, &pdata->irq_gpio_flag);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	GTP_DEBUG("irq:%d flag:%d reset:%d flag:%d \n",
	          pdata->irq_gpio, pdata->irq_gpio_flag,
	          pdata->reset_gpio, pdata->reset_gpio_flag);

#ifndef CONFIG_BOARD_CP8716
	rc = of_property_read_u32(np, "goodix,product-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;
#endif

	prop = of_find_property(np, "goodix,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
		                                "goodix,button-map", button_map,
		                                num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	read_cfg_num = 0;
	for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++) {
		snprintf(prop_name, sizeof(prop_name), "goodix,cfg-data%d", i);
		prop = of_find_property(np, prop_name,
			&pdata->config_data_len[i]);
	if (!prop || !prop->value) {
			pdata->config_data_len[i] = 0;
			pdata->config_data[i] = NULL;
			continue;
		}
		pdata->config_data[i] = devm_kzalloc(dev,
				GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH,
				GFP_KERNEL);
		if (!pdata->config_data[i]) {
			dev_err(dev,
				"Not enough memory for panel config data %d\n",
				i);
			return -ENOMEM;
		}

		pdata->config_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
		pdata->config_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
		memcpy(&pdata->config_data[i][GTP_ADDR_LENGTH],
				prop->value, pdata->config_data_len[i]);
		read_cfg_num++;
	}
	if (read_cfg_num)
		GTP_INFO("%d cfg data read from dts.\n", read_cfg_num);
	else
		dev_err(dev, "Unable to get cfg data from dts.\n");

	return 0;
}
#endif

static int goodix_ts_pinctrl_init(struct goodix_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(ts->client->dev));
	if (IS_ERR_OR_NULL(ts->ts_pinctrl)) {
		GTP_INFO("Target does not use pinctrl\n");
		retval = PTR_ERR(ts->ts_pinctrl);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_active
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->gpio_state_active)) {
		GTP_INFO("Can not get ts default pinstate\n");
		retval = PTR_ERR(ts->gpio_state_active);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_suspend
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->gpio_state_suspend)) {
		GTP_INFO("Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ts->gpio_state_suspend);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int goodix_ts_pinctrl_select(struct  goodix_ts_data *ts, bool on)
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

static int goodix_ts_gpio_configure(struct goodix_ts_data *ts, bool on)
{
	int retval = 0;

	if (on) {
		if (gpio_is_valid(ts->pdata->irq_gpio)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(ts->pdata->irq_gpio,
					"goodix_ts_irq_gpio");
			if (retval) {
				dev_err(&ts->client->dev,
					"unable to request gpio [%d]\n",
					ts->pdata->irq_gpio);
				goto err_irq_gpio_req;
			}
		} else {
			dev_err(&ts->client->dev,
				"irq gpio not provided\n");
			goto err_irq_gpio_req;
		}

		if (gpio_is_valid(ts->pdata->reset_gpio)) {
			/* configure touchscreen reset out gpio */
			retval = gpio_request(ts->pdata->reset_gpio,
					"goodix_ts_reset_gpio");
			if (retval) {
				dev_err(&ts->client->dev,
					"unable to request gpio [%d]\n",
					ts->pdata->reset_gpio);
				goto err_irq_gpio_dir;
			}

			retval = gpio_direction_output(ts->pdata->reset_gpio, 1);
			if (retval) {
				dev_err(&ts->client->dev,
					"set direction gpio[%d] fail\n",
					ts->pdata->reset_gpio);
				goto err_reset_gpio_dir;
			}
		} else {
			dev_err(&ts->client->dev,
				"reset gpio not provided\n");
			goto err_irq_gpio_dir;
		}
		return 0;
	} else {
		if (gpio_is_valid(ts->pdata->irq_gpio)) {
			gpio_free(ts->pdata->irq_gpio);
		}
		if (gpio_is_valid(ts->pdata->reset_gpio)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */
			retval = gpio_direction_input(ts->pdata->reset_gpio);
			if (retval) {
				dev_err(&ts->client->dev,
				"unable to set direction for gpio "
				"[%d]\n", ts->pdata->reset_gpio);
			}
			gpio_free(ts->pdata->reset_gpio);
		}
		return 0;
	}
err_reset_gpio_dir:
	if (gpio_is_valid(ts->pdata->reset_gpio))
		gpio_free(ts->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(ts->pdata->irq_gpio))
		gpio_free(ts->pdata->irq_gpio);
err_irq_gpio_req:
	return retval;
}
/*******************************************************
Function:
	I2c probe.
Input:
	client: i2c device struct.
	id: device id.
Output:
	Executive outcomes.
	0: succeed.
*******************************************************/

static int goodix_ts_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
	struct goodix_ts_platform_data *pdata;
	struct goodix_ts_data *ts;
	u16 version_info;
	int ret;

	GTP_INFO("GTP I2C Address: 0x%02x\n", client->addr);
#if CONFIG_OF
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
		                     sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,
			        "GTP Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		ret = goodix_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else
#endif
		pdata = client->dev.platform_data;

	if (pdata == NULL) {
		dev_err(&client->dev, "GTP invalid pdata\n");
		return -EINVAL;
	}


	i2c_connect_client = client;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "GTP I2C not supported\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "GTP not enough memory for ts\n");
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	ts->client = client;
	ts->pdata = pdata;
	/* For 2.6.39 & later use spin_lock_init(&ts->irq_lock)
	 * For 2.6.39 & before, use ts->irq_lock = SPIN_LOCK_UNLOCKED
	 */
	spin_lock_init(&ts->irq_lock);
	i2c_set_clientdata(client, ts);
	ts->gtp_rawdiff_mode = 0;

	ret = goodix_power_init(ts);
	if (ret) {
		dev_err(&client->dev, "GTP power init failed\n");
		goto exit_free_client_data;
	}

	ret = goodix_power_on(ts);
	if (ret) {
		dev_err(&client->dev, "GTP power on failed\n");
		goto exit_deinit_power;
	}

	ret = goodix_ts_pinctrl_init(ts);
	if (ret)
		dev_err(&client->dev, "GTP init pinctrl failed\n");

	ret = goodix_ts_pinctrl_select(ts, true);
	if (ret)
		dev_err(&client->dev, "GTP select pinctrl failed\n");

	ret = gtp_request_io_port(ts);
	if (ret) {
		dev_err(&client->dev, "GTP request IO port failed.\n");
		goto exit_power_off;
	}

	ret = gtp_i2c_test(client);
	if (ret != 2) {
		dev_err(&client->dev, "I2C communication ERROR!\n");
		goto exit_free_io_port;
	}

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0) {
		dev_err(&client->dev,
		        "GTP Create firmware update thread error.\n");
		goto exit_free_io_port;
	}
#endif

	ret = gtp_init_panel(ts);
	if (ret < 0) {
		dev_err(&client->dev, "GTP init panel failed.\n");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}

	ret = gtp_request_input_dev(ts);
	if (ret) {
		dev_err(&client->dev, "GTP request input dev failed.\n");
		goto exit_free_inputdev;
	}

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		dev_err(&ts->client->dev,
		        "Unable to register fb_notifier: %d\n",
		        ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	ts->goodix_wq = create_singlethread_workqueue("goodix_wq");
	INIT_WORK(&ts->work, goodix_ts_work_func);

	ret = gtp_request_irq(ts);
	if (ret < 0)
		dev_info(&client->dev, "GTP works in polling mode.\n");
	else
		dev_info(&client->dev, "GTP works in interrupt mode.\n");

	ret = gtp_read_version(client, &version_info);
	if (ret != 2) {
		dev_err(&client->dev, "Read version failed.\n");
		goto exit_free_irq;
	}
	if (ts->use_irq)
		gtp_irq_enable(ts);

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

	touchscreen_set_ops(&goodix_ops);

#if GTP_ESD_PROTECT
	gtp_esd_switch(client, SWITCH_ON);
#endif
	init_done = true;
	return 0;
exit_free_irq:
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev,
		        "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	cancel_work_sync(&ts->work);
	flush_workqueue(ts->goodix_wq);
	destroy_workqueue(ts->goodix_wq);

	input_unregister_device(ts->input_dev);
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
exit_free_inputdev:
	kfree(ts->config_data);
exit_free_io_port:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
exit_power_off:
	goodix_power_off(ts);
exit_deinit_power:
	goodix_power_deinit(ts);
exit_free_client_data:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Function:
	Goodix touchscreen driver release function.
Input:
	client: i2c device struct.
Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev,
		        "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	cancel_delayed_work_sync(&gtp_esd_check_work);
	flush_workqueue(gtp_esd_check_workqueue);
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	if (ts) {
		if (ts->use_irq)
			free_irq(client->irq, ts);
		else
			hrtimer_cancel(&ts->timer);

		cancel_work_sync(&ts->work);
		flush_workqueue(ts->goodix_wq);
		destroy_workqueue(ts->goodix_wq);

		input_unregister_device(ts->input_dev);
		if (ts->input_dev) {
			input_free_device(ts->input_dev);
			ts->input_dev = NULL;
		}
		kfree(ts->config_data);

		if (gpio_is_valid(ts->pdata->reset_gpio))
			gpio_free(ts->pdata->reset_gpio);
		if (gpio_is_valid(ts->pdata->irq_gpio))
			gpio_free(ts->pdata->irq_gpio);

		goodix_power_off(ts);
		goodix_power_deinit(ts);
		i2c_set_clientdata(client, NULL);
		kfree(ts);
	}

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_suspend(struct goodix_ts_data *ts)
{
	int ret = -1;
	int i;

	GTP_DEBUG_FUNC();

#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 1;
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_SLIDE_WAKEUP
	ret = gtp_enter_doze(ts);
#else
	if (ts->use_irq)
		gtp_irq_disable(ts);
	else
		hrtimer_cancel(&ts->timer);

	for (i = 0; i < GTP_MAX_TOUCH; i++)
		gtp_touch_up(ts, i);

	input_sync(ts->input_dev);

	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0)
		dev_err(&ts->client->dev, "GTP early suspend failed.\n");
	/* to avoid waking up while not sleeping,
	 * delay 48 + 10ms to ensure reliability
	 */
	msleep(58);
}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_resume(struct goodix_ts_data *ts)
{
	int ret = -1;

	GTP_DEBUG_FUNC();

	ret = gtp_wakeup_sleep(ts);

#if GTP_SLIDE_WAKEUP
	doze_status = DOZE_DISABLED;
#endif

	if (ret < 0)
		dev_err(&ts->client->dev, "GTP resume failed.\n");

	if (ts->use_irq)
		gtp_irq_enable(ts);
	else
		hrtimer_start(&ts->timer,
		              ktime_set(1, 0), HRTIMER_MODE_REL);

#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 0;
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct goodix_ts_data *ts =
	    container_of(self, struct goodix_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			goodix_ts_resume(ts);
		else if (*blank == FB_BLANK_POWERDOWN)
			goodix_ts_suspend(ts);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Function:
	Early suspend function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(ts);
	return;
}

/*******************************************************
Function:
	Late resume function.
Input:
	h: early_suspend struct.
Output:
	None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;

	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(ts);
	return;
}
#endif
#endif /* !CONFIG_HAS_EARLYSUSPEND && !CONFIG_FB*/

#if GTP_ESD_PROTECT
/*******************************************************
Function:
	switch on & off esd delayed work
Input:
	client:  i2c device
	on:	SWITCH_ON / SWITCH_OFF
Output:
	void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, int on)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);
	if (SWITCH_ON == on) {
		/* switch on esd  */
		if (!ts->esd_running) {
			ts->esd_running = 1;
			GTP_INFO("Esd started\n");
			queue_delayed_work(gtp_esd_check_workqueue,
				&gtp_esd_check_work,
				msecs_to_jiffies(GTP_ESD_CHECK_CIRCLE));
		}
	} else {
		/* switch off esd */
		if (ts->esd_running) {
			ts->esd_running = 0;
			GTP_INFO("Esd cancelled\n");
			cancel_delayed_work_sync(&gtp_esd_check_work);
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
static int gtp_init_ext_watchdog(struct i2c_client *client)
{
	/* in case of recursively reset by calling gtp_i2c_write*/
	struct i2c_msg msg;
	u8 opr_buffer[4] = {0x80, 0x40, 0xAA, 0xAA};
	int ret;
	int retries = 0;

	GTP_DEBUG("Init external watchdog...");
	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr  = client->addr;
	msg.len   = sizeof(opr_buffer);
	msg.buf   = opr_buffer;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			return 1;
		retries++;
	}
	if (retries >= 5)
		dev_err(&client->dev, "init external watchdog failed!");
	return 0;
}

/*******************************************************
Function:
	Esd protect function.
	Added external watchdog by meta, 2013/03/07
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
	u8 test[4] = {0x80, 0x40};

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);

	if (ts->gtp_is_suspend) {
		GTP_INFO("Esd terminated!\n");
		ts->esd_running = 0;
		return;
	}

	if (ts->enter_update)
		return;

	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_read(ts->client, test, 4);

		GTP_DEBUG("0x8040 = 0x%02X, 0x8041 = 0x%02X", test[2], test[3]);
		if ((ret < 0)) {
			/* IC works abnormally..*/
			continue;
		} else {
			if ((test[2] == 0xAA) || (test[3] != 0xAA)) {
				/* IC works abnormally..*/
				i = 3;
				break;
			} else {
				/* IC works normally, Write 0x8040 0xAA*/
				test[2] = 0xAA;
				gtp_i2c_write(ts->client, test, 3);
				break;
			}
		}
	}
	if (i >= 3) {
		dev_err(&ts->client->dev,
		        "IC Working ABNORMALLY, Resetting Guitar...\n");
		gtp_reset_guitar(ts, 50);
	}

	if (!ts->gtp_is_suspend)
		queue_delayed_work(gtp_esd_check_workqueue,
			&gtp_esd_check_work,
			msecs_to_jiffies(GTP_ESD_CHECK_CIRCLE));
	else {
		GTP_INFO("Esd terminated!\n");
		ts->esd_running = 0;
	}

	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{ GTP_I2C_NAME, 0 },
	{ }
};

static struct of_device_id goodix_match_table[] = {
	{ .compatible = "Goodix,Goodix-TS", },
	{ },
};

static struct i2c_driver goodix_ts_driver = {
	.probe      = goodix_ts_probe,
	.remove     = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend    = goodix_ts_early_suspend,
	.resume     = goodix_ts_late_resume,
#endif
	.id_table   = goodix_ts_id,
	.driver = {
		.name     = GTP_I2C_NAME,
		.owner    = THIS_MODULE,
		.of_match_table = goodix_match_table,
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
	int ret;
	GTP_INFO("GTP driver installing...");
	GTP_DEBUG_FUNC();
#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
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
	i2c_del_driver(&goodix_ts_driver);
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
