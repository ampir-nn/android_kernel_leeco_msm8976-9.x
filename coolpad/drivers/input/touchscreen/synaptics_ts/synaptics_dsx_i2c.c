/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx.h>
#include "synaptics_dsx_i2c.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/input/touchscreen_yl.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>

#define  MAX_BUTTONS  4
/*unsigned int IC_TYPE = 0;*/

#define SYNA_PWR_EN             1  		/* the tw power enable switch */
#define SYNA_SLEEP_PWR_EN       0	    /* the tw sleep power enable switch */

#define SYNA_COB		1
#define SYNA_COF		2

static struct i2c_client * i2c_connect_client = NULL;
extern int syna_tw_reflash_init(struct synaptics_rmi4_data * rmi4_data);


/*********************** GLOVE *************************/
/*#define CONFIG_TW_GLOVE_SWITCH*/

#ifdef CONFIG_TW_GLOVE_SWITCH
u8 glove_switch = 0;
u8 windows_switch = 0;

/*struct synaptics_rmi4_data *rmi4_data;*/
struct synaptics_rmi4_data *glove_phys = NULL;
struct synaptics_rmi4_data *windows_phys = NULL;

void glove_mode_switch(void);
void windows_mode_switch(void);
void glove_windows_switch(int in_hall);

DEFINE_MUTEX(glove_mutex);
#endif
/*********************** GLOVE *************************/
#define DRIVER_NAME "synaptics_rmi4_i2c"
#define INPUT_PHYS_NAME "synaptics_dsx_i2c/input0"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

#define NO_0D_WHILE_2D
/*
#define REPORT_2D_Z
*/
#define REPORT_2D_W

#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 1000 /* ms */
#define POLLING_PERIOD 1 /* ms */
#define SYN_I2C_RETRY_TIMES 10
#define MAX_ABS_MT_TOUCH_MAJOR 15

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)

enum device_status {
	STATUS_NO_ERROR = 0x00,
	STATUS_RESET_OCCURRED = 0x01,
	STATUS_INVALID_CONFIG = 0x02,
	STATUS_DEVICE_FAILURE = 0x03,
	STATUS_CONFIG_CRC_FAILURE = 0x04,
	STATUS_FIRMWARE_CRC_FAILURE = 0x05,
	STATUS_CRC_IN_PROGRESS = 0x06
};

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
                                   unsigned short addr, unsigned char *data,
                                   unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
                                    unsigned short addr, unsigned char *data,
                                    unsigned short length);

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data);

static void synaptics_rmi4_swap_axis(struct synaptics_rmi4_data *rmi4_data);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data);

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_flipx_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_flipx_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_flipy_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_flipy_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_swap_axes_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

struct synaptics_rmi4_f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_query {
	union {
		struct {
			unsigned char max_button_count:3;
			unsigned char reserved:5;
			unsigned char has_general_control:1;
			unsigned char has_interrupt_enable:1;
			unsigned char has_multibutton_select:1;
			unsigned char has_tx_rx_map:1;
			unsigned char has_perbutton_threshold:1;
			unsigned char has_release_threshold:1;
			unsigned char has_strongestbtn_hysteresis:1;
			unsigned char has_filter_strength:1;
		} __packed;
		unsigned char data[2];
	};
};

struct synaptics_rmi4_f1a_control_0 {
	union {
		struct {
			unsigned char multibutton_report:2;
			unsigned char filter_mode:2;
			unsigned char reserved:4;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f1a_control_3_4 {
	unsigned char transmitterbutton;
	unsigned char receiverbutton;
};

struct synaptics_rmi4_f1a_control {
	struct synaptics_rmi4_f1a_control_0 general_control;
	unsigned char *button_int_enable;
	unsigned char *multi_button;
	struct synaptics_rmi4_f1a_control_3_4 *electrode_map;
	unsigned char *button_threshold;
	unsigned char button_release_threshold;
	unsigned char strongest_button_hysteresis;
	unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
	int button_bitmask_size;
	unsigned char button_count;
	unsigned char valid_button_count;
	unsigned char *button_data_buffer;
	unsigned char *button_map;
	struct synaptics_rmi4_f1a_query button_query;
	struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_f54_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;
			/* query 1 */
			unsigned char num_of_tx_electrodes;
			/* query 2 */
			unsigned char f54_query2_b0__1:2;
			unsigned char has_baseline:1;
			unsigned char has_image8:1;
			unsigned char f54_query2_b4__5:2;
			unsigned char has_image16:1;
			unsigned char f54_query2_b7:1;
			/* queries 3.0 and 3.1 */
			unsigned short clock_rate;
			/* query 4 */
			unsigned char touch_controller_family;
			/* query 5 */
			unsigned char has_pixel_touch_threshold_adjustment:1;
			unsigned char f54_query5_b1__7:7;
			/* query 6 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_interference_metric:1;
			unsigned char has_sense_frequency_control:1;
			unsigned char has_firmware_noise_mitigation:1;
			unsigned char has_ctrl11:1;
			unsigned char has_two_byte_report_rate:1;
			unsigned char has_one_byte_report_rate:1;
			unsigned char has_relaxation_control:1;
		} __packed;
		unsigned char data[8];
	};
};

struct synaptics_rmi4_exp_fn {
	enum exp_fn fn_type;
	bool inserted;
	int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
	void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
	                  unsigned char intr_mask);
	struct list_head link;
};

static struct device_attribute attrs[] = {
	__ATTR(reset, S_IWUSR,
	synaptics_rmi4_show_error,
	synaptics_rmi4_f01_reset_store),
	__ATTR(productinfo, S_IRUGO,
	synaptics_rmi4_f01_productinfo_show,
	synaptics_rmi4_store_error),
	__ATTR(buildid, S_IRUGO,
	synaptics_rmi4_f01_buildid_show,
	synaptics_rmi4_store_error),
	__ATTR(flashprog, S_IRUGO,
	synaptics_rmi4_f01_flashprog_show,
	synaptics_rmi4_store_error),
	__ATTR(0dbutton, (S_IRUGO | S_IWUSR),
	synaptics_rmi4_0dbutton_show,
	synaptics_rmi4_0dbutton_store),
	__ATTR(flipx, (S_IRUGO | S_IWUSR),
	synaptics_rmi4_flipx_show,
	synaptics_rmi4_flipx_store),
	__ATTR(flipy, (S_IRUGO | S_IWUSR),
	synaptics_rmi4_flipy_show,
	synaptics_rmi4_flipy_store),
	__ATTR(swapaxes, (S_IWUSR),
	synaptics_rmi4_show_error,
	synaptics_rmi4_swap_axes_store),
};

struct synaptics_rmi4_exp_fn_data {
	bool initialized;
	bool queue_work;
	struct mutex mutex;
	struct list_head list;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int reset;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	retval = synaptics_rmi4_reset_device(rmi4_data);
	if (retval < 0) {
		dev_err(dev,
		        "%s: Failed to issue reset command, error = %d\n",
		        __func__, retval);
		return retval;
	}

	return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
	                (rmi4_data->rmi4_mod_info.product_info[0]),
	                (rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	unsigned int build_id;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	build_id = (unsigned int)rmi->build_id[0] +
	           (unsigned int)rmi->build_id[1] * 0x100 +
	           (unsigned int)rmi->build_id[2] * 0x10000;

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                build_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int retval;
	struct synaptics_rmi4_f01_device_status device_status;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 rmi4_data->f01_data_base_addr,
	                                 device_status.data,
	                                 sizeof(device_status.data));
	if (retval < 0) {
		dev_err(dev,
		        "%s: Failed to read device status, error = %d\n",
		        __func__, retval);
		return retval;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	unsigned char ii;
	unsigned char intr_enable;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;

	if (rmi4_data->button_0d_enabled == input)
		return count;

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
				ii = fhandler->intr_reg_num;

				retval = synaptics_rmi4_i2c_read(rmi4_data,
				                                 rmi4_data->f01_ctrl_base_addr +
				                                 1 + ii,
				                                 &intr_enable,
				                                 sizeof(intr_enable));
				if (retval < 0)
					return retval;

				if (input == 1)
					intr_enable |= fhandler->intr_mask;
				else
					intr_enable &= ~fhandler->intr_mask;

				retval = synaptics_rmi4_i2c_write(rmi4_data,
				                                  rmi4_data->f01_ctrl_base_addr +
				                                  1 + ii,
				                                  &intr_enable,
				                                  sizeof(intr_enable));
				if (retval < 0)
					return retval;
			}
		}
	}

	rmi4_data->button_0d_enabled = input;

	return count;
}

static ssize_t synaptics_rmi4_flipx_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                rmi4_data->flip_x);
}

static ssize_t synaptics_rmi4_flipx_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->flip_x = input > 0 ? 1 : 0;

	return count;
}

static ssize_t synaptics_rmi4_flipy_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                rmi4_data->flip_y);
}

static ssize_t synaptics_rmi4_flipy_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	rmi4_data->flip_y = input > 0 ? 1 : 0;

	return count;
}

static ssize_t synaptics_rmi4_swap_axes_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	synaptics_rmi4_swap_axis(rmi4_data);

	return count;
}

static void synaptics_rmi4_swap_axis(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	struct i2c_client *client = rmi4_data->i2c_client;
	struct synaptics_rmi4_fn *sensor_tuning = NULL;

	rmi = &(rmi4_data->rmi4_mod_info);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			SYNA_DEBUG("function number 0x%x\n", fhandler->fn_number);
			if (fhandler->fn_number == SYNAPTICS_RMI4_F55)
				sensor_tuning = fhandler;
			if (sensor_tuning == NULL &&
			    fhandler->fn_number == SYNAPTICS_RMI4_F54)
				sensor_tuning = fhandler;
		}
	}

	if (sensor_tuning != NULL) {
		int retval = 0;
		unsigned char val;
		unsigned short swap_ctrl_addr;
		unsigned short offset;

		if (sensor_tuning->fn_number == SYNAPTICS_RMI4_F55)
			swap_ctrl_addr = sensor_tuning->full_addr.ctrl_base + 0;
		else {
			struct synaptics_rmi4_f54_query f54_query;
			retval = synaptics_rmi4_i2c_read(rmi4_data,
			                                 sensor_tuning->full_addr.query_base,
			                                 (unsigned char *)f54_query.data,
			                                 sizeof(f54_query.data));
			if (retval < 0)
				dev_err(&client->dev,
				        "%s: Failed to read swap control registers\n",
				        __func__);

			/* general ctrl 0 */
			offset = 1;
			/* ctrl 1/4/5/6/8.0/8.1/9*/
			if (f54_query.touch_controller_family == 0x00 ||
			    f54_query.touch_controller_family == 0x01)
				offset += 7;
			/* ctrl 2/2.1 */
			offset += 2;
			/* ctrl 3 */
			if (f54_query.has_pixel_touch_threshold_adjustment)
				offset++;
			/* ctrl 7*/
			if (f54_query.touch_controller_family == 0x01)
				offset += 1;
			/* ctrl 10 */
			if (f54_query.has_interference_metric)
				offset++;
			/* ctrl 11/11.0 */
			if (f54_query.has_ctrl11)
				offset +=2;
			/* ctrl 12/13 */
			if (f54_query.has_relaxation_control)
				offset +=2;
			if (!f54_query.has_sensor_assignment)
				dev_err(&client->dev,
				        "%s: Sensor assignment properties not exist\n",
				        __func__);
			swap_ctrl_addr = sensor_tuning->full_addr.ctrl_base + offset;
		}
		retval = synaptics_rmi4_i2c_read(rmi4_data,
		                                 swap_ctrl_addr,
		                                 (unsigned char *)&val,
		                                 sizeof(val));
		if (retval < 0)
			dev_err(&client->dev,
			        "%s: Failed to read swap control registers\n",
			        __func__);

		val = (val & 0xFE) | (!val & 0x01);

		dev_info(&client->dev,
		         "swap value :0x%x, rmi address 0x%02X\n",
		         val, swap_ctrl_addr);

		retval = synaptics_rmi4_i2c_write(rmi4_data,
		                                  swap_ctrl_addr,
		                                  (unsigned char *)&val,
		                                  sizeof(val));
		if (retval < 0)
			dev_err(&client->dev,
			        "%s: Failed to write swap control registers\n",
			        __func__);
	} else
		dev_err(&client->dev,
		        "%s: Firmware not support swap function\n",
		        __func__);
}

/**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
                                   unsigned int address)
{
	int retval = 0;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = rmi4_data->i2c_client;

	page = ((address >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(&i2c->dev,
					"%s:I2C set page(%d) retry %d\n",
					__func__, page, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else
		return PAGE_SELECT_LEN;

#ifdef CONFIG_TW_GLOVE_SWITCH
	/*	glove_windows_switch(0);*/
#endif  /*CONFIG_TW_GLOVE_SWITCH*/

	return (retval == PAGE_SELECT_LEN) ? retval : -EIO;
}

/**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
                                   unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
			"%s:I2C read reg(0x%x) retry %d\n",
			__func__, addr, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: I2C read over retry limit\n",
		        __func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

/**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
                                    unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = rmi4_data->i2c_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN)
		goto exit;

	buf[0] = addr & MASK_8BIT;

	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&rmi4_data->i2c_client->dev,
			"%s:I2C write reg(0x%x):%d retry 0x%x\n",
			__func__, addr, data[0], retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: I2C write over retry limit\n",
		        __func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

	return retval;
}

/**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char touch_count = 0; /* number of touch points */
	unsigned char reg_index;
	unsigned char finger;
	unsigned char fingers_supported;
	unsigned char num_of_finger_status_regs;
	unsigned char finger_shift;
	unsigned char finger_status;
	unsigned char data_reg_blk_size;
	unsigned char finger_status_reg[3];
	unsigned char data[F11_STD_DATA_LEN];
	unsigned short data_addr;
	unsigned short data_offset;
	int x;
	int y;
	int wx;
	int wy;
	int z;

	/*
	 * The number of finger status registers is determined by the
	 * maximum number of fingers supported - 2 bits per finger. So
	 * the number of finger status registers to read is:
	 * register_count = ceil(max_num_of_fingers / 4)
	 */
	fingers_supported = fhandler->num_of_data_points;
	num_of_finger_status_regs = (fingers_supported + 3) / 4;
	data_addr = fhandler->full_addr.data_base;
	data_reg_blk_size = fhandler->size_of_data_register_block;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 data_addr,
	                                 finger_status_reg,
	                                 num_of_finger_status_regs);
	if (retval < 0)
		return 0;

	for (finger = 0; finger < fingers_supported; finger++) {
		reg_index = finger / 4;
		finger_shift = (finger % 4) * 2;
		finger_status = (finger_status_reg[reg_index] >> finger_shift)
		                & MASK_2BIT;

		/*
		 * Each 2-bit finger status field represents the following:
		 * 00 = finger not present
		 * 01 = finger present and data accurate
		 * 10 = finger present but data may be inaccurate
		 * 11 = reserved
		 */
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(rmi4_data->input_dev, finger);
		input_mt_report_slot_state(rmi4_data->input_dev,
		                           MT_TOOL_FINGER, finger_status != 0);
#endif

		if (finger_status) {
			data_offset = data_addr +
			              num_of_finger_status_regs +
			              (finger * data_reg_blk_size);
			retval = synaptics_rmi4_i2c_read(rmi4_data,
			                                 data_offset,
			                                 data,
			                                 data_reg_blk_size);
			if (retval < 0)
				return 0;

			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
			wx = (data[3] & MASK_4BIT);
			wy = (data[3] >> 4) & MASK_4BIT;
			z = data[4];

			if (rmi4_data->flip_x)
				x = rmi4_data->sensor_max_x - x;
			if (rmi4_data->flip_y)
				y = rmi4_data->sensor_max_y - y;

			SYNA_DEBUG("Finger %d: "
			           "status = 0x%02x "
			           "x = %04d "
			           "y = %04d "
			           "wx = %04d "
			           "wy = %04d.",
			           finger,
			           finger_status,
			           x, y, wx, wy);

			input_report_key(rmi4_data->input_dev,
			                 BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,
			                 BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,
			                 ABS_MT_POSITION_X, x);
			input_report_abs(rmi4_data->input_dev,
			                 ABS_MT_POSITION_Y, y);
			input_report_abs(rmi4_data->input_dev,
			                 ABS_MT_PRESSURE, z);

#ifdef REPORT_2D_W
			input_report_abs(rmi4_data->input_dev,
			                 ABS_MT_TOUCH_MAJOR, max(wx, wy));
			input_report_abs(rmi4_data->input_dev,
			                 ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(rmi4_data->input_dev);
#endif
			touch_count++;
		}
	}

	if (touch_count == 0) {
		input_report_key(rmi4_data->input_dev, BTN_TOUCH, 0);
		input_report_key(rmi4_data->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(rmi4_data->input_dev);
#endif
	}

	input_sync(rmi4_data->input_dev);

	return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
                                      struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	unsigned char button;
	unsigned char index;
	unsigned char shift;
	unsigned char status;
	unsigned char *data;
	unsigned short data_addr = fhandler->full_addr.data_base;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	static unsigned char do_once = 1;
	static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
	static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
	static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

	if (do_once) {
		memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
		memset(before_2d_status, 0, sizeof(before_2d_status));
		memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
		do_once = 0;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 data_addr,
	                                 f1a->button_data_buffer,
	                                 f1a->button_bitmask_size);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to read button data registers\n",
		        __func__);
		return;
	}

	data = f1a->button_data_buffer;

	for (button = 0; button < f1a->valid_button_count; button++) {
		index = button / 8;
		shift = button % 8;
		status = ((data[index] >> shift) & MASK_1BIT);

		if (current_status[button] == status)
			continue;
		else
			current_status[button] = status;

		SYNA_DEBUG("%s: Button %d (code %d) ->%d.",__func__, button,f1a->button_map[button],status);
#ifdef NO_0D_WHILE_2D
		if (rmi4_data->fingers_on_2d == false) {
			if (status == 1) {
				before_2d_status[button] = 1;
			} else {
				if (while_2d_status[button] == 1) {
					while_2d_status[button] = 0;
					continue;
				} else {
					before_2d_status[button] = 0;
				}
			}
			input_report_key(rmi4_data->input_dev,
			                 f1a->button_map[button],
			                 status);
		} else {
			if (before_2d_status[button] == 1) {
				before_2d_status[button] = 0;
				input_report_key(rmi4_data->input_dev,
				                 f1a->button_map[button],
				                 status);
			} else {
				if (status == 1)
					while_2d_status[button] = 1;
				else
					while_2d_status[button] = 0;
			}
		}
#else
		input_report_key(rmi4_data->input_dev,
		                 f1a->button_map[button],
		                 status);
#endif
	}

	input_sync(rmi4_data->input_dev);

	return;
}

/**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
                                        struct synaptics_rmi4_fn *fhandler,
                                        unsigned char *touch_count)
{
	unsigned char touch_count_2d;

	dev_dbg(&rmi4_data->i2c_client->dev,"%s: Function %02x reporting\n",__func__, fhandler->fn_number);

	switch (fhandler->fn_number) {
	case SYNAPTICS_RMI4_F11:
		touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,
		                 fhandler);

		*touch_count += touch_count_2d;

		if (touch_count_2d)
			rmi4_data->fingers_on_2d = true;
		else
			rmi4_data->fingers_on_2d = false;
		break;

	case SYNAPTICS_RMI4_F1A:
		synaptics_rmi4_f1a_report(rmi4_data, fhandler);
		break;

	default:
		break;
	}

	return;
}

/**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static int synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char touch_count = 0;
	unsigned char intr[MAX_INTR_REGISTERS];
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_exp_fn *exp_fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

	/*
	 * Get interrupt status information from F01 Data1 register to
	 * determine the source(s) that are flagging the interrupt.
	 */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 rmi4_data->f01_data_base_addr + 1,
	                                 intr,
	                                 rmi4_data->num_of_intr_regs);
	if (retval < 0)
		return retval;

	/*
	 * Traverse the function handler list and service the source(s)
	 * of the interrupt accordingly.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->num_of_data_sources) {
				if (fhandler->intr_mask &
				    intr[fhandler->intr_reg_num]) {
					synaptics_rmi4_report_touch(rmi4_data,
					                            fhandler, &touch_count);
				}
			}
		}
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->inserted &&
			    (exp_fhandler->func_attn != NULL))
				exp_fhandler->func_attn(rmi4_data, intr[0]);
		}
	}
	mutex_unlock(&exp_data.mutex);

	return touch_count;
}

/**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
	struct synaptics_rmi4_data *rmi4_data = data;
	synaptics_rmi4_sensor_report(rmi4_data);
	return IRQ_HANDLED;
}

/**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_irq_acquire() and power management
 * functions in this driver
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
                                     bool enable)
{
	int retval = 0;
	unsigned char intr_status;

	if (enable) {
		if(rmi4_data->irq_enabled)
			return retval;
		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
		                                 rmi4_data->f01_data_base_addr + 1,
		                                 &intr_status,
		                                 rmi4_data->num_of_intr_regs);
		if (retval < 0)
			return retval;

		enable_irq(rmi4_data->irq);

		rmi4_data->irq_enabled = true;
	} else {
		if(rmi4_data->irq_enabled) {
			disable_irq(rmi4_data->irq);
			rmi4_data->irq_enabled = false;
		}
	}
	return retval;
}

/**
 * synaptics_rmi4_irq_acquire()
 *
 * Called by synaptics_rmi4_probe()  in this driver and also exported
 * to other expansion Function modules such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_acquire(struct synaptics_rmi4_data *rmi4_data,
					bool enable)
{
	int retval = 0;
	const struct synaptics_rmi4_platform_data *pdata = rmi4_data->board;
	unsigned char intr_status;

	if (enable) {
		/* Clear interrupts first */
		retval = synaptics_rmi4_i2c_read(rmi4_data,
		                                 rmi4_data->f01_data_base_addr + 1,
		                                 &intr_status,
		                                 rmi4_data->num_of_intr_regs);
		if (retval < 0)
			return retval;

		retval = request_threaded_irq(rmi4_data->irq, NULL,
		                              synaptics_rmi4_irq, pdata->irq_flags,
		                              DRIVER_NAME, rmi4_data);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
			        "%s: Failed to request_threaded_irq\n",
			        __func__);
			return retval;
		}

		rmi4_data->irq_enabled = true;/*ang*/
	} else
		free_irq(rmi4_data->irq, rmi4_data);
	return retval;
}

/**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
                                   struct synaptics_rmi4_fn *fhandler,
                                   struct synaptics_rmi4_fn_desc *fd,
                                   unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned char intr_offset;
	unsigned char abs_data_size;
	unsigned char abs_data_blk_size;
	unsigned char query[F11_STD_QUERY_LEN];
	unsigned char control[F11_STD_CTRL_LEN];

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 fhandler->full_addr.query_base,
	                                 query,
	                                 sizeof(query));
	if (retval < 0)
		return retval;

	/* Maximum number of fingers supported */
	if ((query[1] & MASK_3BIT) <= 4)
		fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
	else if ((query[1] & MASK_3BIT) == 5)
		fhandler->num_of_data_points = 10;

	rmi4_data->num_of_fingers = fhandler->num_of_data_points;
	SYNA_DEBUG("Maximum number of fingers supported is %d.", rmi4_data->num_of_fingers);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 fhandler->full_addr.ctrl_base,
	                                 control,
	                                 sizeof(control));
	if (retval < 0)
		return retval;

	/* Maximum x and y */
	rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
	                          ((control[7] & MASK_4BIT) << 8);
	rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
	                          ((control[9] & MASK_4BIT) << 8);
	dev_err(&rmi4_data->i2c_client->dev, "%s: Function %02x maxx=%d maxy=%d.",
	           __func__, fhandler->fn_number,
	           rmi4_data->sensor_max_x,
	           rmi4_data->sensor_max_y);

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
	     ii < ((fd->intr_src_count & MASK_3BIT) +
	           intr_offset);
	     ii++)
		fhandler->intr_mask |= 1 << ii;

	abs_data_size = query[5] & MASK_2BIT;
	abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
	fhandler->size_of_data_register_block = abs_data_blk_size;

	return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
                                        struct synaptics_rmi4_fn *fhandler)
{
	int retval;
	struct synaptics_rmi4_f1a_handle *f1a;

	f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
	if (!f1a) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to alloc mem for function handle\n",
		        __func__);
		return -ENOMEM;
	}

	fhandler->data = (void *)f1a;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 fhandler->full_addr.query_base,
	                                 f1a->button_query.data,
	                                 sizeof(f1a->button_query.data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to read query registers\n",
		        __func__);
		return retval;
	}

	f1a->button_count = f1a->button_query.max_button_count + 1;
	f1a->button_bitmask_size = (f1a->button_count + 7) / 8;

	f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
	                                  sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
	if (!f1a->button_data_buffer) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to alloc mem for data buffer\n",
		        __func__);
		return -ENOMEM;
	}

	f1a->button_map = kcalloc(f1a->button_count,
	                          sizeof(*(f1a->button_map)), GFP_KERNEL);
	if (!f1a->button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to alloc mem for button map\n",
		        __func__);
		return -ENOMEM;
	}

	return 0;
}

static int synaptics_rmi4_cap_button_map(
    struct synaptics_rmi4_data *rmi4_data,
    struct synaptics_rmi4_fn *fhandler)
{
	unsigned char ii;
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
	const struct synaptics_rmi4_platform_data *pdata = rmi4_data->board;

	if (!pdata->capacitance_button_map) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: cap_button_map is" \
		        "NULL in board file\n",
		        __func__);
		return -ENODEV;
	} else if (!pdata->capacitance_button_map->map) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Button map is missing in board file\n",
		        __func__);
		return -ENODEV;
	} else {
		if (pdata->capacitance_button_map->nbuttons !=
		    f1a->button_count) {
			f1a->valid_button_count = min(f1a->button_count,
						pdata->capacitance_button_map->nbuttons);
		} else {
			f1a->valid_button_count = f1a->button_count;
		}

		for (ii = 0; ii < f1a->valid_button_count; ii++)
			f1a->button_map[ii] =
			    pdata->capacitance_button_map->map[ii];
	}

	return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
	struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

	if (f1a) {
		kfree(f1a->button_data_buffer);
		kfree(f1a->button_map);
		kfree(f1a);
		fhandler->data = NULL;
	}

	return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
                                   struct synaptics_rmi4_fn *fhandler,
                                   struct synaptics_rmi4_fn_desc *fd,
                                   unsigned int intr_count)
{
	int retval;
	unsigned char ii;
	unsigned short intr_offset;

	fhandler->fn_number = fd->fn_number;
	fhandler->num_of_data_sources = fd->intr_src_count;

	fhandler->intr_reg_num = (intr_count + 7) / 8;
	if (fhandler->intr_reg_num != 0)
		fhandler->intr_reg_num -= 1;

	/* Set an enable bit for each data source */
	intr_offset = intr_count % 8;
	fhandler->intr_mask = 0;
	for (ii = intr_offset;
	     ii < ((fd->intr_src_count & MASK_3BIT) +
	           intr_offset);
	     ii++)
		fhandler->intr_mask |= 1 << ii;

	retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	retval = synaptics_rmi4_cap_button_map(rmi4_data, fhandler);
	if (retval < 0)
		goto error_exit;

	rmi4_data->button_0d_enabled = 1;

	return 0;

error_exit:
	synaptics_rmi4_f1a_kfree(fhandler);

	return retval;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
                                   struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
	*fhandler = kzalloc(sizeof(**fhandler), GFP_KERNEL);
	if (!(*fhandler))
		return -ENOMEM;

	(*fhandler)->full_addr.data_base =
	    (rmi_fd->data_base_addr |
	     (page_number << 8));
	(*fhandler)->full_addr.ctrl_base =
	    (rmi_fd->ctrl_base_addr |
	     (page_number << 8));
	(*fhandler)->full_addr.cmd_base =
	    (rmi_fd->cmd_base_addr |
	     (page_number << 8));
	(*fhandler)->full_addr.query_base =
	    (rmi_fd->query_base_addr |
	     (page_number << 8));
	(*fhandler)->fn_number = rmi_fd->fn_number;
	printk(KERN_NOTICE "%s:fun_num:0x%x,data_base_addr:0x%x,ctrl_base_addr:0x%x,cmd_base_addr:0x%x,query_base_addr:0x%x\n",
				__func__,rmi_fd->fn_number,
				(*fhandler)->full_addr.data_base,
				(*fhandler)->full_addr.ctrl_base,
				(*fhandler)->full_addr.cmd_base,
				(*fhandler)->full_addr.query_base);
	return 0;
}


/**
 * synaptics_rmi4_query_device_info()
 *
 * Called by synaptics_rmi4_query_device().
 *
 */
static int synaptics_rmi4_query_device_info(
    struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char f01_query[F01_STD_QUERY_LEN];
	struct synaptics_rmi4_device_info *rmi = &(rmi4_data->rmi4_mod_info);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 rmi4_data->f01_query_base_addr,
	                                 f01_query,
	                                 sizeof(f01_query));
	if (retval < 0)
		return retval;

	/* RMI Version 4.0 currently supported */
	rmi->version_major = 4;
	rmi->version_minor = 0;

	rmi->manufacturer_id = f01_query[0];
	rmi->product_props = f01_query[1];
	rmi->product_info[0] = f01_query[2] & MASK_7BIT;
	rmi->product_info[1] = f01_query[3] & MASK_7BIT;
	rmi->date_code[0] = f01_query[4] & MASK_5BIT;
	rmi->date_code[1] = f01_query[5] & MASK_4BIT;
	rmi->date_code[2] = f01_query[6] & MASK_5BIT;
	rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) |
	                 (f01_query[8] & MASK_7BIT);
	rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) |
	                     (f01_query[10] & MASK_7BIT);
	memcpy(rmi->product_id_string, &f01_query[11], 10);

	if (rmi->manufacturer_id != 1) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Non-Synaptics device found, manufacturer ID = %d\n",
		        __func__, rmi->manufacturer_id);
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET,
	                                 rmi->build_id,
	                                 sizeof(rmi->build_id));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to read firmware build id (code %d)\n",
		        __func__, retval);
		return retval;
	}
	return 0;
}

/**
 * synaptics_rmi4_crc_in_progress()
 *
 * Check if crc in progress ever occured
 *
 */
static bool synaptics_rmi4_crc_in_progress(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_f01_device_status *status)
{
	int retval;
	int times = 0;
	bool rescan = false;

	while (1) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
		                                 rmi4_data->f01_data_base_addr,
		                                 status->data,
		                                 sizeof(status->data));
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
			        "%s: read status register failed\n",
			        __func__);
			return false;
		}
		if (status->status_code ==
		    STATUS_CRC_IN_PROGRESS) {
			dev_info(&rmi4_data->i2c_client->dev,
			         "%s: CRC is in progress...\n",
			         __func__);
			rescan = true;
			msleep(20);
		} else {
			break;
		}
		if (times++ > 500)
			return false;
	}
	return rescan;
}

/**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char page_number;
	unsigned char intr_count;
	unsigned char data_sources;
	unsigned short pdt_entry_addr;
	unsigned short intr_addr;
	struct synaptics_rmi4_f01_device_status status;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;

	rmi = &(rmi4_data->rmi4_mod_info);

rescan:
	INIT_LIST_HEAD(&rmi->support_fn_list);
	intr_count = 0;
	data_sources = 0;

	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
		     pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
			                                 pdt_entry_addr,
			                                 (unsigned char *)&rmi_fd,
			                                 sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			fhandler = NULL;

			if (rmi_fd.fn_number == 0) {
				dev_dbg(&rmi4_data->i2c_client->dev,
				        "%s: Reached end of PDT\n",
				        __func__);
				break;
			}

			dev_err(&rmi4_data->i2c_client->dev,
			        "%s: F%02x found (page %d)\n",
			        __func__, rmi_fd.fn_number,
			        page_number);

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_query_base_addr =
				    rmi_fd.query_base_addr;
				rmi4_data->f01_ctrl_base_addr =
				    rmi_fd.ctrl_base_addr;
				rmi4_data->f01_data_base_addr =
				    rmi_fd.data_base_addr;
				rmi4_data->f01_cmd_base_addr =
				    rmi_fd.cmd_base_addr;

				if (synaptics_rmi4_crc_in_progress(rmi4_data, &status))
					goto rescan;
				printk(KERN_ERR "%s:fun_num:0x%x,data_base_addr:0x%x,ctrl_base_addr:0x%x,cmd_base_addr:0x%x,query_base_addr:0x%x\n",
				                                __func__,rmi_fd.fn_number,
								rmi4_data->f01_data_base_addr,
								rmi4_data->f01_ctrl_base_addr,
								rmi4_data->f01_cmd_base_addr,
								rmi4_data->f01_query_base_addr);
				retval =
				    synaptics_rmi4_query_device_info(rmi4_data);
				if (retval < 0)
					return retval;

				if (status.flash_prog == 1) {
					pr_notice("%s: In flash prog mode, status = 0x%02x\n",
					          __func__,
					          status.status_code);
					goto flash_prog_mode;
				}
				break;

			case SYNAPTICS_RMI4_F11:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
				                                 &rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
					        "%s: Failed to alloc for F%d\n",
					        __func__,
					        rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f11_init(rmi4_data,
				                                 fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;

			case SYNAPTICS_RMI4_F1A:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
				                                 &rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
					        "%s: Failed to alloc for F%d\n",
					        __func__,
					        rmi_fd.fn_number);
					return retval;
				}

				retval = synaptics_rmi4_f1a_init(rmi4_data,
				                                 fhandler, &rmi_fd, intr_count);
				if (retval < 0)
					return retval;
				break;

			case SYNAPTICS_RMI4_F54:
			case SYNAPTICS_RMI4_F55:
				if (rmi_fd.intr_src_count == 0)
					break;

				retval = synaptics_rmi4_alloc_fh(&fhandler,
				                                 &rmi_fd, page_number);
				if (retval < 0) {
					dev_err(&rmi4_data->i2c_client->dev,
					        "%s: Failed to alloc for F%d\n",
					        __func__,
					        rmi_fd.fn_number);
					return retval;
				}
				break;

			default:
				break;
			}

			/* Accumulate the interrupt count */
			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

			if (fhandler && rmi_fd.intr_src_count) {
				list_add_tail(&fhandler->link,
				              &rmi->support_fn_list);
			}
		}
	}

flash_prog_mode:
	rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
	dev_err(&rmi4_data->i2c_client->dev,
	        "%s: Number of interrupt registers = %d\n",
	        __func__, rmi4_data->num_of_intr_regs);

	memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

	/*
	 * Map out the interrupt bit masks for the interrupt sources
	 * from the registered function handlers.
	 */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link)
		data_sources += fhandler->num_of_data_sources;
	}
	if (data_sources) {
		if (!list_empty(&rmi->support_fn_list)) {
			list_for_each_entry(fhandler,
			                    &rmi->support_fn_list, link) {
				if (fhandler->num_of_data_sources) {
					rmi4_data->intr_mask[fhandler->intr_reg_num] |=
					    fhandler->intr_mask;
				}
			}
		}
	}

	/* Enable the interrupt sources */
	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (rmi4_data->intr_mask[ii] != 0x00) {
			intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			dev_err(&rmi4_data->i2c_client->dev,
				"%s: Interrupt enable mask 0x%x = 0x%02x\n",
				__func__, intr_addr, rmi4_data->intr_mask[ii]);
			retval = synaptics_rmi4_i2c_write(rmi4_data,
			                                  intr_addr,
			                                  &(rmi4_data->intr_mask[ii]),
			                                  sizeof(rmi4_data->intr_mask[ii]));
			if (retval < 0)
				return retval;
		}
	}

	return 0;
}


static int synaptics_rmi4_reset_command(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	int page_number;
	unsigned char command = 0x01;
	unsigned short pdt_entry_addr;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_f01_device_status status;
	bool done = false;

rescan:
	/* Scan the page description tables of the pages to service */
	for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
		for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END;
		     pdt_entry_addr -= PDT_ENTRY_SIZE) {
			pdt_entry_addr |= (page_number << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
			                                 pdt_entry_addr,
			                                 (unsigned char *)&rmi_fd,
			                                 sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			if (rmi_fd.fn_number == 0)
				break;

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				rmi4_data->f01_cmd_base_addr =
				    rmi_fd.cmd_base_addr;
				rmi4_data->f01_data_base_addr =
				    rmi_fd.data_base_addr;
				if (synaptics_rmi4_crc_in_progress(rmi4_data, &status))
					goto rescan;
				done = true;
				break;
			}
		}
		if (done) {
			dev_info(&rmi4_data->i2c_client->dev,
			         "%s: Find F01 in page description table 0x%x\n",
			         __func__, rmi4_data->f01_cmd_base_addr);
			break;
		}
	}


	if (!done) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Cannot find F01 in page description table\n",
		        __func__);
		return -EINVAL;;
	}

	retval = synaptics_rmi4_i2c_write(rmi4_data,
	                                  rmi4_data->f01_cmd_base_addr,
	                                  &command,
	                                  sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to issue reset command, error = %d\n",
		        __func__, retval);
		return retval;
	} else
		dev_err(&rmi4_data->i2c_client->dev, "%s: send reset cmd 0x%x to 0x%x OK\n",
			__func__, command, rmi4_data->f01_cmd_base_addr);

	/*msleep(rmi4_data->reset_delay);*/
	msleep(100);
	return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_device_info *rmi;
	struct synaptics_rmi4_exp_fn *exp_fhandler;

	rmi = &(rmi4_data->rmi4_mod_info);

	retval = synaptics_rmi4_reset_command(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to send command reset\n",
		        __func__);
		return retval;
	}

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				synaptics_rmi4_f1a_kfree(fhandler);
			else
				kfree(fhandler->data);
			kfree(fhandler);
		}
	}

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
		        "%s: Failed to query device\n",
		        __func__);
		return retval;
	}

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry(exp_fhandler, &exp_data.list, link) {
			if (exp_fhandler->fn_type == RMI_F54) {
				exp_fhandler->func_remove(rmi4_data);
				exp_fhandler->func_init(rmi4_data);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);
	return 0;
}

/**
 * synaptics_rmi4_detection_work()
 *
 * Called by the kernel at the scheduled time.
 *
 * This function is a self-rearming work thread that checks for the
 * insertion and removal of other expansion Function modules such as
 * rmi_dev and calls their initialization and removal callback functions
 * accordingly.
 */
static void synaptics_rmi4_detection_work(struct work_struct *work)
{
	struct synaptics_rmi4_exp_fn *exp_fhandler, *next_list_entry;

	mutex_lock(&exp_data.mutex);
	if (!list_empty(&exp_data.list)) {
		list_for_each_entry_safe(exp_fhandler,
		                         next_list_entry,
		                         &exp_data.list,
		                         link) {
			if ((exp_fhandler->func_init != NULL) &&
			    (exp_fhandler->inserted == false)) {
				exp_fhandler->func_init(exp_data.rmi4_data);
				exp_fhandler->inserted = true;
			} else if ((exp_fhandler->func_init == NULL) &&
			           (exp_fhandler->inserted == true)) {
				exp_fhandler->func_remove(exp_data.rmi4_data);
				list_del(&exp_fhandler->link);
				kfree(exp_fhandler);
			}
		}
	}
	mutex_unlock(&exp_data.mutex);

	return;
}

/**
 * synaptics_rmi4_new_function()
 *
 * Called by other expansion Function modules in their module init and
 * module exit functions.
 *
 * This function is used by other expansion Function modules such as
 * rmi_dev to register themselves with the driver by providing their
 * initialization and removal callback function pointers so that they
 * can be inserted or removed dynamically at module init and exit times,
 * respectively.
 */
void synaptics_rmi4_new_function(enum exp_fn fn_type, bool insert,
                                 int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
                                 void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
                                 void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
                                         unsigned char intr_mask))
{
	struct synaptics_rmi4_exp_fn *exp_fhandler;

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = 1;
	}

	mutex_lock(&exp_data.mutex);
	if (insert) {
		exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
		if (!exp_fhandler) {
			pr_err("%s: Failed to alloc mem for expansion function\n",
			       __func__);
			goto exit;
		}
		exp_fhandler->fn_type = fn_type;
		exp_fhandler->func_init = func_init;
		exp_fhandler->func_attn = func_attn;
		exp_fhandler->func_remove = func_remove;
		exp_fhandler->inserted = false;
		list_add_tail(&exp_fhandler->link, &exp_data.list);
	} else {
		if (!list_empty(&exp_data.list)) {
			list_for_each_entry(exp_fhandler, &exp_data.list, link) {
				if (exp_fhandler->func_init == func_init) {
					exp_fhandler->inserted = false;
					exp_fhandler->func_init = NULL;
					exp_fhandler->func_attn = NULL;
					goto exit;
				}
			}
		}
	}
exit:
	mutex_unlock(&exp_data.mutex);

	if (exp_data.queue_work) {
		queue_delayed_work(exp_data.workqueue,
		                   &exp_data.work,
		                   msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
	}
	return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int synaptics_rmi4_power_on(struct synaptics_rmi4_data *rmi4_data,
					bool on) {
#if SYNA_PWR_EN
	int retval;

	if (on == false)
		goto power_off;

	retval = reg_set_optimum_mode_check(rmi4_data->board->ts_vdd,
	15000);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
			"Regulator vdd set_opt failed rc=%d\n",
			retval);
		return retval;
	}

	retval = regulator_enable(rmi4_data->board->ts_vdd);
	if (retval) {
		dev_err(&rmi4_data->i2c_client->dev,
			"Regulator vdd enable failed rc=%d\n",
			retval);
		goto error_reg_en_vdd;
	}

	if (rmi4_data->board->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(
			rmi4_data->board->ts_vcc_i2c, 10000);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n",
				retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(rmi4_data->board->ts_vcc_i2c);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n",
				retval);
			goto error_reg_en_vcc_i2c;
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	if (rmi4_data->board->i2c_pull_up)
		reg_set_optimum_mode_check(rmi4_data->board->ts_vcc_i2c, 0);
error_reg_opt_i2c:
	regulator_disable(rmi4_data->board->ts_vdd);
error_reg_en_vdd:
	reg_set_optimum_mode_check(rmi4_data->board->ts_vdd, 0);
	return retval;

power_off:
	reg_set_optimum_mode_check(rmi4_data->board->ts_vdd, 0);
	retval = regulator_disable(rmi4_data->board->ts_vdd);
	if (retval)
		pr_err("Ts regulator vdd disable failed rc=%d\n", retval);
	if (rmi4_data->board->i2c_pull_up) {
		reg_set_optimum_mode_check(rmi4_data->board->ts_vcc_i2c, 0);
		retval = regulator_disable(rmi4_data->board->ts_vcc_i2c);
		if (retval)
			pr_err("Ts regulator vcc_i2c disable failed rc=%d\n", retval);
	}
	return retval;
#endif
	return 0;
}

static int synaptics_rmi4_pinctrl_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	rmi4_data->ts_pinctrl = devm_pinctrl_get(&(rmi4_data->i2c_client->dev));
	if (IS_ERR_OR_NULL(rmi4_data->ts_pinctrl)) {
		dev_dbg(&rmi4_data->i2c_client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(rmi4_data->ts_pinctrl);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	rmi4_data->gpio_state_active
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(rmi4_data->gpio_state_active)) {
		dev_dbg(&rmi4_data->i2c_client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(rmi4_data->gpio_state_active);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	rmi4_data->gpio_state_suspend
		= pinctrl_lookup_state(rmi4_data->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(rmi4_data->gpio_state_suspend)) {
		dev_dbg(&rmi4_data->i2c_client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(rmi4_data->gpio_state_suspend);
		rmi4_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int synpatics_rmi4_pinctrl_select(
		struct synaptics_rmi4_data *rmi4_data, bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? rmi4_data->gpio_state_active
		: rmi4_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(rmi4_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&rmi4_data->i2c_client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		dev_err(&rmi4_data->i2c_client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}

static int synaptics_rmi4_gpio_configure(
		struct synaptics_rmi4_data *rmi4_data, bool on)
{
	int retval = 0;

	if (on) {
		if (gpio_is_valid(rmi4_data->board->irq_gpio)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(rmi4_data->board->irq_gpio,
				"touch_irq_gpio");
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"unable to request gpio [%d]\n",
					rmi4_data->board->irq_gpio);
				goto err_irq_gpio_req;
			}
			retval = gpio_direction_input(rmi4_data->board->irq_gpio);
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"unable to set direction for gpio " \
					"[%d]\n", rmi4_data->board->irq_gpio);
				goto err_irq_gpio_dir;
			}
		} else {
			dev_err(&rmi4_data->i2c_client->dev,
				"irq gpio not provided\n");
			goto err_irq_gpio_req;
		}

		if (gpio_is_valid(rmi4_data->board->reset_gpio)) {
			/* configure touchscreen reset out gpio */
			retval = gpio_request(rmi4_data->board->reset_gpio,
					"touch_reset_gpio");
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"unable to request gpio [%d]\n",
					rmi4_data->board->reset_gpio);
				goto err_irq_gpio_dir;
			}

			retval = gpio_direction_output(rmi4_data->board->reset_gpio, 1);
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"unable to set direction for gpio[%d]\n",
					rmi4_data->board->reset_gpio);
				goto err_reset_gpio_dir;
			}
/*
			gpio_set_value(rmi4_data->board->reset_gpio, 1);
			msleep(rmi4_data->board->reset_delay);
*/
		} else {
			dev_err(&rmi4_data->i2c_client->dev,
				"reset gpio not provided\n");
			goto err_irq_gpio_dir;
		}
		return 0;
	} else {
			if (gpio_is_valid(rmi4_data->board->irq_gpio))
				gpio_free(rmi4_data->board->irq_gpio);
			if (gpio_is_valid(rmi4_data->board->reset_gpio)) {
				/*
				 * This is intended to save leakage current
				 * only. Even if the call(gpio_direction_input)
				 * fails, only leakage current will be more but
				 * functionality will not be affected.
				 */
				retval = gpio_direction_input(rmi4_data->board->reset_gpio);
				if (retval) {
					dev_err(&rmi4_data->i2c_client->dev,
					"unable to set direction for gpio "
					"[%d]\n", rmi4_data->board->irq_gpio);
				}
				gpio_free(rmi4_data->board->reset_gpio);
			}

		return 0;
	}

err_reset_gpio_dir:
	if (gpio_is_valid(rmi4_data->board->reset_gpio))
		gpio_free(rmi4_data->board->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(rmi4_data->board->irq_gpio))
		gpio_free(rmi4_data->board->irq_gpio);
err_irq_gpio_req:
	return retval;
}

static int synaptics_rmi4_regulator_configure(struct synaptics_rmi4_data
						*rmi4_data, bool on)
{
#if SYNA_PWR_EN
	int retval;

	if (on == false)
		goto hw_shutdown;

	rmi4_data->board->ts_vdd = regulator_get(&rmi4_data->i2c_client->dev,
					"vdd");
	if (IS_ERR(rmi4_data->board->ts_vdd)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to get vdd regulator\n",
				__func__);
		return PTR_ERR(rmi4_data->board->ts_vdd);
	}

	if (regulator_count_voltages(rmi4_data->board->ts_vdd) > 0) {
		retval = regulator_set_voltage(rmi4_data->board->ts_vdd,
			2600000, 3300000);
		if (retval) {
			dev_err(&rmi4_data->i2c_client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd;
		}
	}

	if (rmi4_data->board->i2c_pull_up) {
		rmi4_data->board->ts_vcc_i2c = regulator_get(&rmi4_data->i2c_client->dev,
						"vcc_i2c");
		if (IS_ERR(rmi4_data->board->ts_vcc_i2c)) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to get i2c regulator\n",
					__func__);
			retval = PTR_ERR(rmi4_data->board->ts_vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(rmi4_data->board->ts_vcc_i2c) > 0) {
			retval = regulator_set_voltage(rmi4_data->board->ts_vcc_i2c,
				1800000, 1800000);
			if (retval) {
				dev_err(&rmi4_data->i2c_client->dev,
					"reg set i2c vtg failed retval =%d\n",
					retval);
			goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (rmi4_data->board->i2c_pull_up)
		regulator_put(rmi4_data->board->ts_vcc_i2c);
err_get_vtg_i2c:
	if (regulator_count_voltages(rmi4_data->board->ts_vdd) > 0)
		regulator_set_voltage(rmi4_data->board->ts_vdd, 0,
		1800000);
err_set_vtg_vdd:
	regulator_put(rmi4_data->board->ts_vdd);
	return retval;

hw_shutdown:
	if (regulator_count_voltages(rmi4_data->board->ts_vdd) > 0)
		regulator_set_voltage(rmi4_data->board->ts_vdd, 0,
			3300000);

	regulator_put(rmi4_data->board->ts_vdd);

	if (rmi4_data->board->i2c_pull_up) {
		if (regulator_count_voltages(rmi4_data->board->ts_vcc_i2c) > 0)
			regulator_set_voltage(rmi4_data->board->ts_vcc_i2c, 0,
					1800000);
		regulator_put(rmi4_data->board->ts_vcc_i2c);
	}
#endif

	return 0;
};


static int synaptics_hw_reset(struct synaptics_rmi4_data
				*rmi4_data)
{
	udelay(500);	/*reset after DVDDH and VBUS power on */
	gpio_direction_output(rmi4_data->board->reset_gpio, 1);
	msleep(5);

	gpio_direction_output(rmi4_data->board->reset_gpio, 0);
	msleep(rmi4_data->board->reset_delay);

	gpio_direction_output(rmi4_data->board->reset_gpio, 1);
	msleep(33);

	return 0;
}

/************************* GLOVE ****************************/

#ifdef CONFIG_TW_GLOVE_SWITCH

unsigned char g_mode_s_on	= 0x00;
unsigned char g_mode_s_off  = 0x02;
unsigned char w_mode_s_off	= 0x00;
unsigned char w_mode_s_on	= 0x01;
unsigned char g_mode_in_w	= 0x03;

extern int resume_ver(void);
static ssize_t resume_version(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	resume_ver();
	ret = sprintf(buf, "%s: resume_version OK!\n",__func__);
	SYNA_DEBUG("TOUCH_RMI:YLLOG:%s(%d): resume version OK!\n", __func__, __LINE__);
	return ret;
}

void glove_mode_switch(void)
{
	int error;

	if(glove_switch == 0) {
		if( glove_phys != NULL ) {
			/*error=rmi_write(glove_phys->rmi_dev, 0x0400,0x02);*/
			error = synaptics_rmi4_i2c_write(glove_phys, 0x0400, &g_mode_s_off, sizeof(g_mode_s_off));
			SYNA_DEBUG("TOUCH_RMI:YLLOG:%s: glove mode off. ret = %d\n", __func__, error);
		}
	} else {
		if( glove_phys != NULL ) {
			/*error=rmi_write(glove_phys, 0x0400,0x00);*/
			error = synaptics_rmi4_i2c_write(glove_phys, 0x0400, &g_mode_s_on, sizeof(g_mode_s_on));
			SYNA_DEBUG("TOUCH_RMI:YLLOG:%s: glove mode on. ret = %d\n", __func__, error);
		}
	}
}

void windows_mode_switch(void)
{
	int error;

	if(windows_switch != 0) {			/* windows_switch = 1, enter windows mode*/
		if( windows_phys != NULL ) {
			/*error = rmi_write(windows_phys, 0x041B, 0x01);*/
			error = synaptics_rmi4_i2c_write(windows_phys, 0x041B, &w_mode_s_on, sizeof(w_mode_s_on));
			if (error < 0) {
				SYNA_DEBUG("TOUCH_RMI:YLLOG:windows rmi_i2c_write 0x0400 failed.%d.\n", error);
				/*return;*/
			}

			/*error = rmi_write(windows_phys, 0x0202,0x00);	disable menu & back keys*/
			/*if (error < 0) {*/
			/*SYNA_DEBUG("TOUCH_RMI:YLLOG:windows rmi_i2c_write 0x0202 failed.%d.\n", error);*/
			/*return;*/
			/*}*/

			if( glove_phys != NULL ) {
				/*error=rmi_write(glove_phys, 0x0400, 0x03);*/
				error = synaptics_rmi4_i2c_write(glove_phys, 0x0400, &g_mode_in_w, sizeof(g_mode_in_w));
				SYNA_DEBUG("TOUCH_RMI:YLLOG:%s: glove mode on in windowns mode. ret = %d\n", __func__, error);
			}
			SYNA_DEBUG("TOUCH_RMI:YLLOG:enter windows mode.\n");
		}
	} else {							/*windows_switch = 0, exit windows mode*/
		if( windows_phys != NULL) {
			/*error = rmi_write(windows_phys, 0x041B,0x00);*/
			error = synaptics_rmi4_i2c_write(glove_phys, 0x041B, &w_mode_s_off, sizeof(w_mode_s_off));
			if (error < 0) {
				SYNA_DEBUG("TOUCH_RMI:YLLOG:windows rmi_i2c_write 0x0400 failed.%d.\n", error);
				/*return;*/
			}

			/*error = rmi_write(windows_phys->rmi_dev, 0x0202,0x03); disable menu & back keys*/
			/*if (error < 0) {*/
			/*SYNA_DEBUG("TOUCH_RMI:YLLOG:windows rmi_i2c_write 0x0202 failed.%d.\n", error);*/
			/*return;*/
			/*}*/

			if( glove_phys != NULL ) {
				/*error=rmi_write(glove_phys, 0x0400,(unsigned char *)0x02);*/
				error = synaptics_rmi4_i2c_write(glove_phys, 0x0400, &g_mode_s_off, sizeof(g_mode_s_off));
				SYNA_DEBUG("TOUCH_RMI:YLLOG:%s: glove mode off in windows mode. ret = %d\n", __func__, error);
			}
			SYNA_DEBUG("TOUCH_RMI:YLLOG:exit windows mode.\n");
		}
	}
}

void glove_windows_switch(int in_hall)
{
	mutex_lock(&glove_mutex);

	if (0 == windows_switch)
		glove_mode_switch();
	else {
		if (in_hall) {
			windows_switch = 0;
			windows_mode_switch();
			glove_mode_switch();
			mutex_unlock(&glove_mutex);
			return;
		}
		windows_mode_switch();
	}

	mutex_unlock(&glove_mutex);
}
EXPORT_SYMBOL_GPL(glove_windows_switch);

static ssize_t glove_show(struct device *dev,
                          struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	unsigned char data = 0;
	int error;
	if(glove_phys != NULL) {
		/*		error=rmi_i2c_read(glove_phys, 0x0400, &data);*/
		error = synaptics_rmi4_i2c_read(glove_phys, 0x0400, &data, sizeof(data));
		SYNA_DEBUG("TOUCH_RMI:YLLOG:%s(%d): glove read 0x0400=%d, ret=%d, glove_mode = %d.\n",
		       __func__, __LINE__, data, error, glove_switch);
	}

	ret = sprintf(buf, "%d\n", glove_switch);
	return ret;
}

static ssize_t glove_store(struct device *dev,
                           struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&glove_mutex);
	glove_switch = val;
	glove_mode_switch();
	mutex_unlock(&glove_mutex);

	return count;
}

static ssize_t windows_show(struct device *dev,
                            struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int error = 0;
	unsigned char data = 0;
	if(windows_phys != NULL) {
		/*error = rmi_i2c_read( windows_phys, 0x0056, &data);*/
		/*error = rmi_i2c_read( windows_phys, 0x041B, &data);*/
		error = synaptics_rmi4_i2c_read(glove_phys, 0x041B, &data, sizeof(data));
		SYNA_DEBUG("TOUCH_RMI:YLLOG:%s(%d): windows read 0x041B = %d, ret = %d, window_mode = %d.\n",
		       __func__, __LINE__, data, error, windows_switch);
	}

	ret = sprintf(buf, "%d\n", windows_switch);
	return ret;
}

static ssize_t windows_store(struct device *dev,
                             struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&glove_mutex);
	windows_switch = val;
	windows_mode_switch();
	mutex_unlock(&glove_mutex);

	return count;
}

static ssize_t reset_store(struct device *dev,
                           struct device_attribute *attr,const char *buf, size_t count)
{
	/*	struct tw_platform_data *ts_pdata = pdata->ts_platform_data;*/

	const struct synaptics_rmi4_platform_data *ts_pdata = glove_phys->board;
	/*	struct synaptics_rmi4_platform_data *ts_pdata;*/
	unsigned long val;
	int error = 0;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	if (val > 0) {
		if (ts_pdata->reset) {
			error = ts_pdata->reset(10);
			if (error)
				SYNA_DEBUG("TOUCH:YLLOG:reset test :failed to reset.\n");
		}

		glove_windows_switch(0);
	}

	SYNA_DEBUG("TOUCH:YLLOG:reset test OK.\n");
	return count;
}

static DEVICE_ATTR(resume, S_IRUGO, resume_version, NULL);
static DEVICE_ATTR(glove_switch, S_IRUGO|S_IWUSR, glove_show, glove_store);
static DEVICE_ATTR(windows_switch, S_IRUGO|S_IWUSR, windows_show, windows_store);
static DEVICE_ATTR(reset_ic, S_IRUGO|S_IWUSR, NULL, reset_store);

static struct attribute *glove_attribute[] = {
	&dev_attr_resume.attr,
	&dev_attr_glove_switch.attr,
	&dev_attr_windows_switch.attr,
	&dev_attr_reset_ic.attr,
	NULL
};

static struct attribute_group glove_attribute_group = {
	.attrs = glove_attribute
};
#endif
/*************************** GLOVE **************************/

static unsigned char f1a_button_codes[] = {KEY_BACK,KEY_HOME,KEY_MENU};
static struct synaptics_rmi4_capacitance_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(f1a_button_codes),
	.map = f1a_button_codes,
};

/***************************************************
 *
 *
 *  synaptics_parse_dt function for S7020A
 *
 *
 ****************************************************/
static int synaptics_parse_dt(struct device *dev, struct synaptics_rmi4_platform_data *pdata)
{
	int rc;
	const char *hw_type_str;
	const char *ic_type_str;
	struct device_node *np = dev->of_node;
	struct property *prop;
	int num_buttons;
	int button_map[MAX_BUTTONS];
	unsigned char i;
	pdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, &pdata->reset_gpio_flags);
	pdata->irq_gpio = of_get_named_gpio_flags(np,
				"synaptics,irq-gpio", 0, &pdata->irq_gpio_flags);
	SYNA_DEBUG("pdata->gpio_reset = %d, pdata->gpio_irq = %d\n",
				pdata->reset_gpio, pdata->irq_gpio);
	pdata->i2c_pull_up = of_property_read_bool(np,
				"synaptics,i2c-pull-up");
	rc = of_property_read_u32(np, "synaptics,reset-delay", &pdata->reset_delay);
	if (rc) {
		pdata->reset_delay = 10;
		dev_err(dev, "Unable to read reset delay\n");
	}

	rc = of_property_read_u32(np, "synaptics,irq_flags",
				(unsigned int *)&pdata->irq_flags);
	if (rc) {
		dev_err(dev, "Unable to read %s property in node %s\n",
				"synaptics,irq_flags", np->full_name);
		return -ENODEV;
	}
	SYNA_DEBUG("irq_flags = %ld\n", (unsigned long)pdata->irq_flags);

	/*
	   rc = of_property_read_u32(np,"synaptics,screen_x",&pdata->screen_x);
	   if(rc){
	   dev_err(dev,"Unable to read %s property in node %s\n","synaptics,screen_x",np->full_name);
	   return -ENODEV;
	   }
	   rc = of_property_read_u32(np,"synaptics,screen_y",&pdata->screen_y);
	   if(rc){
	   dev_err(dev,"Unable to read %s property in node %s\n","synaptics,screen_y",np->full_name);
	   return -ENODEV;
	   }
	   printk("screen x,y = %d x %d\n",pdata->screen_x,pdata->screen_y);
	*/

	rc = of_property_read_string(np, "synaptics,hw_type", &hw_type_str);
	if (rc) {
		dev_err(dev, "Unable to get hw_type in dt\n");
		return rc;
	}
	if (strcmp(hw_type_str, "SYNA_COB") == 0) {
		SYNA_DEBUG("(dts define)hw_type is %s.\n", hw_type_str);
		pdata->hw_type = SYNA_COB;
	} else if (strcmp(hw_type_str, "SYNA_COF") == 0) {
		SYNA_DEBUG("(dts define)hw_type is %s.\n", hw_type_str);
		pdata->hw_type = SYNA_COF;
	} else
		SYNA_DEBUG("unknown hw_type %s.\n", hw_type_str);

	rc = of_property_read_string(np, "synaptics,ic_type", &ic_type_str);
	if (rc) {
		dev_err(dev, "Unable to get ic_type in dt\n");
		return rc;
	}
	if (strcmp(ic_type_str, "S2202") == 0) {
		SYNA_DEBUG("(dts define)ic_type is %s.\n", ic_type_str);
		pdata->ic_type = 2202;
	} else if (strcmp(ic_type_str, "S3202") == 0) {
		SYNA_DEBUG("(dts define)ic_type is %s.\n", ic_type_str);
		pdata->ic_type = 3202;
	} else
		SYNA_DEBUG("nonsupport ic_type %s.\n", ic_type_str);

	prop = of_find_property(np, "synaptics,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(num_buttons);
		SYNA_DEBUG("num_buttons:%d\n", num_buttons);

		if (num_buttons > MAX_BUTTONS)
			return -ENODEV;

		rc = of_property_read_u32_array(np, "synaptics,button-map",
			button_map, num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
		for (i = 0; i < num_buttons; i++)
			cap_button_map.map[i] = button_map[i];
			cap_button_map.nbuttons = num_buttons;
		SYNA_DEBUG("use button map in dts\n");
	}

	pdata->capacitance_button_map = &cap_button_map;

	return 0;
}

/**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */
static int synaptics_rmi4_probe(struct i2c_client *client,
        const struct i2c_device_id *dev_id)
{
	int retval;
	unsigned char ii;
	unsigned char attr_count;
	struct synaptics_rmi4_f1a_handle *f1a;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data;
	struct synaptics_rmi4_device_info *rmi;
	struct synaptics_rmi4_platform_data *platform_data;

	printk("SYNA Driver Version V0101\n");

	i2c_connect_client = client;
	if (!i2c_check_functionality(client->adapter,
	                             I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
		        "%s: SMBus byte data not supported\n",
		        __func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_rmi4_platform_data), GFP_KERNEL);
		if(!platform_data) {
			dev_err(&client->dev,"Failed to allocate memory\n");
			return -ENOMEM;
		}
		retval = synaptics_parse_dt(&client->dev, platform_data);
		if(retval)
			return retval;
	} else
		platform_data = client->dev.platform_data;

	if (!platform_data) {
		dev_err(&client->dev,
		        "%s: No platform data found\n",
		        __func__);
		return -EINVAL;
	}

	rmi4_data = kzalloc(sizeof(*rmi4_data) * 2, GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev,
		        "%s: Failed to alloc mem for rmi4_data\n",
		        __func__);
		return -ENOMEM;
	}

	rmi = &(rmi4_data->rmi4_mod_info);

	rmi4_data->input_dev = input_allocate_device();
	if (rmi4_data->input_dev == NULL) {
		dev_err(&client->dev,
		        "%s: Failed to allocate input device\n",
		        __func__);
		retval = -ENOMEM;
		goto err_input_device;
	}

	rmi4_data->i2c_client = client;
	rmi4_data->current_page = MASK_8BIT;
	rmi4_data->board = platform_data;
	rmi4_data->touch_stopped = false;
	rmi4_data->sensor_sleep = false;
	rmi4_data->irq_enabled = false;

	rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
	rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
	rmi4_data->irq_enable = synaptics_rmi4_irq_acquire;
	rmi4_data->reset_device = synaptics_rmi4_reset_device;

	rmi4_data->flip_x = rmi4_data->board->x_flip;
	rmi4_data->flip_y = rmi4_data->board->y_flip;
	rmi4_data->swap_axes = false;

	rmi4_data->reset_delay = rmi4_data->board->reset_delay ?
				rmi4_data->board->reset_delay : 90;

	retval = synaptics_rmi4_regulator_configure(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to configure regulators\n");
		goto err_reg_configure;
	}

	retval = synaptics_rmi4_power_on(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to power on\n");
		goto err_power_device;
	}

	retval = synaptics_rmi4_pinctrl_init(rmi4_data);
	if (!retval && rmi4_data->ts_pinctrl) {
		retval = synpatics_rmi4_pinctrl_select(rmi4_data, true);
		if (retval < 0)
			goto err_gpio_config;
	}

	retval = synaptics_rmi4_gpio_configure(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to configure gpios\n");
		goto pinctrl_sleep;
	}

	synaptics_hw_reset(rmi4_data);

	init_waitqueue_head(&rmi4_data->wait);
	mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));

	retval = synaptics_rmi4_query_device(rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev,
		        "%s: Failed to query device\n",
		        __func__);
		goto err_query_device;
	}

	if (!exp_data.initialized) {
		mutex_init(&exp_data.mutex);
		INIT_LIST_HEAD(&exp_data.list);
		exp_data.initialized = 1;
	}

	exp_data.workqueue =
			create_singlethread_workqueue("dsx_exp_workqueue");
	INIT_DELAYED_WORK(&exp_data.work,
			synaptics_rmi4_detection_work);
	exp_data.rmi4_data = rmi4_data;
	exp_data.queue_work = true;
	queue_delayed_work(exp_data.workqueue,
			&exp_data.work,
			msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));

	retval = syna_tw_reflash_init(rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev, "syna_tw_reflash_init,rmi tw reflash init failed.\n");
	} else if (retval == 0) {
		printk(KERN_NOTICE "Need not update fw.\n");
	} else if (retval > 0) {
		printk(KERN_NOTICE "Need to reupdate the rmi4_data struct.\n");
		retval = synaptics_rmi4_reset_device(rmi4_data);
		if (retval < 0) {
			dev_err(&client->dev,
				"%s: Failed to reset device\n",
				__func__);
		goto err_reflash_device;
		}
	}

	if (rmi4_data->swap_axes)
		synaptics_rmi4_swap_axis(rmi4_data);

	i2c_set_clientdata(client, rmi4_data);

	rmi4_data->input_dev->name = DRIVER_NAME;
	rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
	rmi4_data->input_dev->id.bustype = BUS_I2C;
	rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
	rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
	rmi4_data->input_dev->dev.parent = &client->dev;
	input_set_drvdata(rmi4_data->input_dev, rmi4_data);

	set_bit(EV_SYN, rmi4_data->input_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_dev->evbit);
	set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);

#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

	input_set_abs_params(rmi4_data->input_dev, ABS_X,
	                     0, rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev, ABS_Y,
	                     0, rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(rmi4_data->input_dev, ABS_PRESSURE,
	                     0, 255, 0, 0);

	input_set_abs_params(rmi4_data->input_dev,
	                     ABS_MT_POSITION_X, 0,
	                     rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
	                     ABS_MT_POSITION_Y, 0,
	                     rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
	                     ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(rmi4_data->input_dev,
	                     ABS_MT_TRACKING_ID,0, 10, 0, 0);
#ifdef REPORT_2D_W
	input_set_abs_params(rmi4_data->input_dev,
	                     ABS_MT_TOUCH_MAJOR, 0,
	                     MAX_ABS_MT_TOUCH_MAJOR, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
#ifdef KERNEL_ABOVE_3_7
	/* input_mt_init_slots now has a "flags" parameter */
	input_mt_init_slots(rmi4_data->input_dev,
	                    rmi4_data->num_of_fingers, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(rmi4_data->input_dev,
	                    rmi4_data->num_of_fingers);
#endif
#endif

	f1a = NULL;
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				f1a = fhandler->data;
		}
	}

	if (f1a) {
		for (ii = 0; ii < f1a->valid_button_count; ii++) {
			set_bit(f1a->button_map[ii],
			        rmi4_data->input_dev->keybit);
			input_set_capability(rmi4_data->input_dev,
			                     EV_KEY, f1a->button_map[ii]);
		}
	}

	retval = input_register_device(rmi4_data->input_dev);
	if (retval) {
		dev_err(&client->dev,
		        "%s: Failed to register input device\n",
		        __func__);
		goto err_register_input;
	}

#if defined(CONFIG_FB)
	rmi4_data->fb_notif.notifier_call = fb_notifier_callback;
	retval = fb_register_client(&rmi4_data->fb_notif);
	if(retval)
		dev_err(&client->dev, "Unable to register fb_botifer: %d\n",retval);
#endif

	rmi4_data->irq = gpio_to_irq(platform_data->irq_gpio);

	retval = synaptics_rmi4_irq_acquire(rmi4_data, true);
	if (retval < 0) {
		dev_err(&client->dev,
		        "%s: Failed to acquire irq\n",
		        __func__);
		goto err_enable_irq;
	}

#ifdef CONFIG_TW_GLOVE_SWITCH
	glove_phys = rmi4_data;
	windows_phys = rmi4_data;
	retval = sysfs_create_group(&client->dev.kobj, &glove_attribute_group);
	if (retval) {
		SYNA_DEBUG("%s: S3202 sysfs_create_group glove failed.\n", __func__);
	}
#endif

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
		                           &attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&client->dev,
			        "%s: Failed to create sysfs attributes\n",
			        __func__);
			goto err_sysfs;
		}
	}

	return retval;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
		                  &attrs[attr_count].attr);
	}
	retval = synaptics_rmi4_irq_acquire(rmi4_data, false);
	if (retval < 0) {
		dev_err(&client->dev,
			"%s: Failed to free irq\n", __func__);
	}

err_enable_irq:
	input_unregister_device(rmi4_data->input_dev);

err_register_input:
err_reflash_device:
	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				synaptics_rmi4_f1a_kfree(fhandler);
			else
				kfree(fhandler->data);
			kfree(fhandler);
		}
	}
err_query_device:
		retval = synaptics_rmi4_gpio_configure(rmi4_data, false);
		if (retval < 0)
			pr_err("Failed to deconfigure gpios\n");
pinctrl_sleep:
		if (rmi4_data->ts_pinctrl) {
			retval = synpatics_rmi4_pinctrl_select(rmi4_data, false);
			if (retval < 0)
				pr_err("Cannot set idle pinctrl state\n");
		}
err_gpio_config:
		synaptics_rmi4_power_on(rmi4_data, false);
err_power_device:
		synaptics_rmi4_regulator_configure(rmi4_data, false);

err_reg_configure:
	input_free_device(rmi4_data->input_dev);
	rmi4_data->input_dev = NULL;
err_input_device:
	kfree(rmi4_data);

	return retval;
}

/**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int synaptics_rmi4_remove(struct i2c_client *client)
{
	int retval;
	unsigned char attr_count;
	struct synaptics_rmi4_fn *fhandler;
	struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
	struct synaptics_rmi4_device_info *rmi;
	/*const struct synaptics_rmi4_platform_data *platform_data =	rmi4_data->board;*/

	rmi = &(rmi4_data->rmi4_mod_info);

	cancel_delayed_work_sync(&exp_data.work);
	flush_workqueue(exp_data.workqueue);
	destroy_workqueue(exp_data.workqueue);

	rmi4_data->touch_stopped = true;
	wake_up(&rmi4_data->wait);

	synaptics_rmi4_irq_acquire(rmi4_data, false);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
		                  &attrs[attr_count].attr);
	}

	input_unregister_device(rmi4_data->input_dev);
	/*
	   if (platform_data->power_down_enable) {
	   regulator_disable(rmi4_data->regulator);
	   regulator_put(rmi4_data->regulator);
	   }
	   */
	if (!list_empty(&rmi->support_fn_list)) {
		list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
			if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
				synaptics_rmi4_f1a_kfree(fhandler);
			else
				kfree(fhandler->data);
			kfree(fhandler);
		}
	}
	input_free_device(rmi4_data->input_dev);

	if (gpio_is_valid(rmi4_data->board->reset_gpio))
		gpio_free(rmi4_data->board->reset_gpio);
	if (gpio_is_valid(rmi4_data->board->irq_gpio))
		gpio_free(rmi4_data->board->irq_gpio);

	if (rmi4_data->ts_pinctrl) {
		retval = synpatics_rmi4_pinctrl_select(rmi4_data, false);
		if (retval < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}

	synaptics_rmi4_power_on(rmi4_data, false);
	synaptics_rmi4_regulator_configure(rmi4_data, false);

	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_PM
/**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 rmi4_data->f01_ctrl_base_addr,
	                                 &device_ctrl,
	                                 sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
		        "%s: Failed to enter sleep mode\n",
		        __func__);
		rmi4_data->sensor_sleep = false;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
	                                  rmi4_data->f01_ctrl_base_addr,
	                                  &device_ctrl,
	                                  sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
		        "%s: Failed to enter sleep mode\n",
		        __func__);
		rmi4_data->sensor_sleep = false;
		return;
	} else {
		rmi4_data->sensor_sleep = true;
	}

	return;
}

/**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char device_ctrl;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
	                                 rmi4_data->f01_ctrl_base_addr,
	                                 &device_ctrl,
	                                 sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
		        "%s: Failed to wake from sleep mode\n",
		        __func__);
		rmi4_data->sensor_sleep = true;
		return;
	}

	device_ctrl = (device_ctrl & ~MASK_3BIT);
	device_ctrl = (device_ctrl | NO_SLEEP_OFF | NORMAL_OPERATION);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
	                                  rmi4_data->f01_ctrl_base_addr,
	                                  &device_ctrl,
	                                  sizeof(device_ctrl));
	if (retval < 0) {
		dev_err(&(rmi4_data->input_dev->dev),
		        "%s: Failed to wake from sleep mode\n",
		        __func__);
		rmi4_data->sensor_sleep = true;
		return;
	} else {
		rmi4_data->sensor_sleep = false;
	}

	return;
}

/**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (!rmi4_data->sensor_sleep) {
		rmi4_data->touch_stopped = true;
		wake_up(&rmi4_data->wait);
		synaptics_rmi4_irq_enable(rmi4_data, false);
		synaptics_rmi4_sensor_sleep(rmi4_data);
	}

#if SYNA_SLEEP_PWR_EN
		synaptics_rmi4_power_on(rmi4_data, 0);
#endif

	/*
	   if (platform_data->power_down_enable)
	   regulator_disable(rmi4_data->regulator);
	   */
	if (rmi4_data->ts_pinctrl) {
		retval = synpatics_rmi4_pinctrl_select(rmi4_data, false);
		if (retval < 0)
			dev_err(dev, "Cannot get idle pinctrl state\n");
	}

	retval = synaptics_rmi4_gpio_configure(rmi4_data, false);
	if (retval < 0)
		dev_err(dev, "failed to put gpios in suspend state\n");
	printk(KERN_NOTICE "TW had suspended.\n");

	return 0;
}

/**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (rmi4_data->ts_pinctrl) {
		retval = synpatics_rmi4_pinctrl_select(rmi4_data, true);
		if (retval < 0)
			dev_err(dev, "Cannot get idle pinctrl state\n");
	}
	retval = synaptics_rmi4_gpio_configure(rmi4_data, true);
	if (retval < 0)
		dev_err(dev, "failed to put gpios in suspend state\n");

#if SYNA_SLEEP_PWR_EN
		synaptics_rmi4_power_on(rmi4_data, 1);
		synaptics_hw_reset(rmi4_data);
#endif

	/*
	   if (platform_data->power_down_enable)
	   regulator_enable(rmi4_data->regulator);
	 */
	synaptics_rmi4_sensor_wake(rmi4_data);
	rmi4_data->touch_stopped = false;
	synaptics_rmi4_irq_enable(rmi4_data, true);

	printk(KERN_NOTICE "TW had resumed.\n");
	return 0;
}

/* add for suspend and resume*/
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct synaptics_rmi4_data *synaptics_dev_data =
	    container_of(self, struct synaptics_rmi4_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && synaptics_dev_data &&
	    synaptics_dev_data->i2c_client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			synaptics_rmi4_resume(&synaptics_dev_data->i2c_client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			synaptics_rmi4_suspend(&synaptics_dev_data->i2c_client->dev);
	}

	return 0;
}
#endif
/*add for suspend and resume*/

#if (!defined(CONFIG_FB))
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
	.suspend = synaptics_rmi4_suspend,
	.resume  = synaptics_rmi4_resume,
};
#else
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
};
#endif

#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct of_device_id synaptics_match_table[]= {
	{ .compatible = "synaptics,rmi4"},
};

static struct i2c_driver synaptics_rmi4_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_match_table,
#ifdef CONFIG_PM
		.pm = &synaptics_rmi4_dev_pm_ops,
#endif
	},
	.probe = synaptics_rmi4_probe,
	.remove = synaptics_rmi4_remove,
	.id_table = synaptics_rmi4_id_table,
};

/**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
	SYNA_DEBUG("***synaptics_rmi4 driver installing***");
	return i2c_add_driver(&synaptics_rmi4_driver);
}

/**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_driver);
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL v2");
