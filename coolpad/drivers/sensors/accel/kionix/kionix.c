/***************************************************************************/
/* Copyright (c) 2000-2010  YULONG Company                                 */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the                */
/* subject matter of this material.  All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the  */
/* license agreement.  The recipient of this software implicitly accepts   */
/* the terms of the license.                                               */
/***************************************************************************/

/*******************************************************************
*****	Copyright(C), 2013-2020, Yulong Company.
*****	FileName:	kionix.c
*****   Description:    Linux device driver for accelerometer sensor
*****
*****   History:
*****   <author>	<time>	    <version >	    <desc>
*****   longjiang     2013.02.17     1.00           Create
*****   longjiang     2013.12.21     2.00           Modify
*****
********************************************************************/

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/ioctl.h>
//#include <linux/proc_fs.h>
#if 0//def    CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

//longjiang add start 20130620
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#endif
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sensors/kionix.h>
#include <linux/sensors/sensparams.h>
#include <linux/sensors.h>

#define CAL_FUZZ 386

#define CAL_CONVERT 1024
//add end

/* Debug Message Flags */
#define KIONIX_KMSG_ERR	1	/* Print kernel debug message for error */

#if KIONIX_KMSG_ERR
#define KMSGERR(format, ...)	\
		dev_err(format, ## __VA_ARGS__)
#else
#define KMSGERR(format, ...)
#endif

#ifdef CONFIG_YL_DEBUG
#define KMSGINF(format, ...)	\
		dev_info(format, ## __VA_ARGS__)
#else
#define KMSGINF(format, ...)
#endif


/******************************************************************************
 * Accelerometer WHO_AM_I return value
 *****************************************************************************/
#define KIONIX_ACCEL_WHO_AM_I_KXTE9 		0x00
#define KIONIX_ACCEL_WHO_AM_I_KXTF9 		0x01
#define KIONIX_ACCEL_WHO_AM_I_KXTI9_1001 	0x04
#define KIONIX_ACCEL_WHO_AM_I_KXTIK_1004 	0x05
#define KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005 	0x07
#define KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007 	0x08
#define KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008 	0x0A
#define KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009 	0x09
#define KIONIX_ACCEL_WHO_AM_I_KXCJK_1013 	0x11

/******************************************************************************
 * Accelerometer Grouping
 *****************************************************************************/
#define KIONIX_ACCEL_GRP1	1	/* KXTE9 */
#define KIONIX_ACCEL_GRP2	2	/* KXTF9/I9-1001/J9-1005 */
#define KIONIX_ACCEL_GRP3	3	/* KXTIK-1004 */
#define KIONIX_ACCEL_GRP4	4	/* KXTJ9-1007/KXCJ9-1008 */
#define KIONIX_ACCEL_GRP5	5	/* KXTJ2-1009 */
#define KIONIX_ACCEL_GRP6	6	/* KXCJK-1013 */

/******************************************************************************
 * Registers for Accelerometer Group 1 & 2 & 3
 *****************************************************************************/
#define ACCEL_WHO_AM_I		0x0F

/*****************************************************************************/
/* Registers for Accelerometer Group 1 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_GRP1_XOUT			0x12
/* Control Registers */
#define ACCEL_GRP1_CTRL_REG1	0x1B
/* CTRL_REG1 */
#define ACCEL_GRP1_PC1_OFF		0x7F
#define ACCEL_GRP1_PC1_ON		(1 << 7)
#define ACCEL_GRP1_ODR40		(3 << 3)
#define ACCEL_GRP1_ODR10		(2 << 3)
#define ACCEL_GRP1_ODR3			(1 << 3)
#define ACCEL_GRP1_ODR1			(0 << 3)
#define ACCEL_GRP1_ODR_MASK		(3 << 3)

/*****************************************************************************/
/* Registers for Accelerometer Group 2 & 3 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_GRP2_XOUT_L		0x06
/* Control Registers */
#define ACCEL_GRP2_INT_REL		0x1A
#define ACCEL_GRP2_CTRL_REG1	0x1B
#define ACCEL_GRP2_INT_CTRL1	0x1E
#define ACCEL_GRP2_DATA_CTRL	0x21
/* CTRL_REG1 */
#define ACCEL_GRP2_PC1_OFF		0x7F
#define ACCEL_GRP2_PC1_ON		(1 << 7)
#define ACCEL_GRP2_DRDYE		(1 << 5)
#define ACCEL_GRP2_G_8G			(2 << 3)
#define ACCEL_GRP2_G_4G			(1 << 3)
#define ACCEL_GRP2_G_2G			(0 << 3)
#define ACCEL_GRP2_G_MASK		(3 << 3)
#define ACCEL_GRP2_RES_8BIT		(0 << 6)
#define ACCEL_GRP2_RES_12BIT	(1 << 6)
#define ACCEL_GRP2_RES_MASK		(1 << 6)
/* INT_CTRL1 */
#define ACCEL_GRP2_IEA			(1 << 4)
#define ACCEL_GRP2_IEN			(1 << 5)
/* DATA_CTRL_REG */
#define ACCEL_GRP2_ODR12_5		0x00
#define ACCEL_GRP2_ODR25		0x01
#define ACCEL_GRP2_ODR50		0x02
#define ACCEL_GRP2_ODR100		0x03
#define ACCEL_GRP2_ODR200		0x04
#define ACCEL_GRP2_ODR400		0x05
#define ACCEL_GRP2_ODR800		0x06
/*****************************************************************************/

/*****************************************************************************/
/* Registers for Accelerometer Group 4 & 5 & 6 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_GRP4_XOUT_L		0x06
/* Control Registers */
#define ACCEL_GRP4_INT_REL		0x1A
#define ACCEL_GRP4_CTRL_REG1	0x1B
#define ACCEL_GRP4_INT_CTRL1	0x1E
#define ACCEL_GRP4_DATA_CTRL	0x21
/* CTRL_REG1 */
#define ACCEL_GRP4_PC1_OFF		0x7F
#define ACCEL_GRP4_PC1_ON		(1 << 7)
#define ACCEL_GRP4_DRDYE		(1 << 5)
#define ACCEL_GRP4_G_8G			(2 << 3)
#define ACCEL_GRP4_G_4G			(1 << 3)
#define ACCEL_GRP4_G_2G			(0 << 3)
#define ACCEL_GRP4_G_MASK		(3 << 3)
#define ACCEL_GRP4_RES_8BIT		(0 << 6)
#define ACCEL_GRP4_RES_12BIT	(1 << 6)
#define ACCEL_GRP4_RES_MASK		(1 << 6)
/* INT_CTRL1 */
#define ACCEL_GRP4_IEA			(1 << 4)
#define ACCEL_GRP4_IEN			(1 << 5)
/* DATA_CTRL_REG */
#define ACCEL_GRP4_ODR0_781		0x08
#define ACCEL_GRP4_ODR1_563		0x09
#define ACCEL_GRP4_ODR3_125		0x0A
#define ACCEL_GRP4_ODR6_25		0x0B
#define ACCEL_GRP4_ODR12_5		0x00
#define ACCEL_GRP4_ODR25		0x01
#define ACCEL_GRP4_ODR50		0x02
#define ACCEL_GRP4_ODR100		0x03
#define ACCEL_GRP4_ODR200		0x04
#define ACCEL_GRP4_ODR400		0x05
#define ACCEL_GRP4_ODR800		0x06
#define ACCEL_GRP4_ODR1600		0x07
/*****************************************************************************/

/* Input Event Constants */
#define ACCEL_G_MAX			8096
#define ACCEL_FUZZ			0
#define ACCEL_FLAT			0
/* I2C Retry Constants */
#define KIONIX_I2C_RETRY_COUNT		10 	/* Number of times to retry i2c */
#define KIONIX_I2C_RETRY_TIMEOUT	1	/* Timeout between retry (miliseconds) */

/* Earlysuspend Contants */
#define KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT	5000	/* Timeout (miliseconds) */

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate (ODR).
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_grp1_odr_table[] = {
	{ 100,	ACCEL_GRP1_ODR40 },
	{ 334,	ACCEL_GRP1_ODR10 },
	{ 1000,	ACCEL_GRP1_ODR3  },
	{ 0,	ACCEL_GRP1_ODR1  },
};

static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_grp2_odr_table[] = {
	{ 3,	ACCEL_GRP2_ODR800 },
	{ 5,	ACCEL_GRP2_ODR400 },
	{ 10,	ACCEL_GRP2_ODR200 },
	{ 20,	ACCEL_GRP2_ODR100 },
	{ 40,	ACCEL_GRP2_ODR50  },
	{ 80,	ACCEL_GRP2_ODR25  },
	{ 0,	ACCEL_GRP2_ODR12_5},
};

static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_grp4_odr_table[] = {
	{ 2,	ACCEL_GRP4_ODR1600 },
	{ 3,	ACCEL_GRP4_ODR800 },
	{ 5,	ACCEL_GRP4_ODR400 },
	{ 10,	ACCEL_GRP4_ODR200 },
	{ 20,	ACCEL_GRP4_ODR100 },
	{ 40,	ACCEL_GRP4_ODR50  },
	{ 80,	ACCEL_GRP4_ODR25  },
	{ 160,	ACCEL_GRP4_ODR12_5},
	{ 320,	ACCEL_GRP4_ODR6_25},
	{ 640,	ACCEL_GRP4_ODR3_125},
	{ 1280,	ACCEL_GRP4_ODR1_563},
	{ 0,	ACCEL_GRP4_ODR0_781},
};

enum {
	accel_grp1_ctrl_reg1 = 0,
	accel_grp1_regs_count,
};

enum {
	accel_grp2_ctrl_reg1 = 0,
	accel_grp2_data_ctrl,
	accel_grp2_int_ctrl,
	accel_grp2_regs_count,
};

enum {
	accel_grp4_ctrl_reg1 = 0,
	accel_grp4_data_ctrl,
	accel_grp4_int_ctrl,
	accel_grp4_regs_count,
};

struct kionix_accel_driver {
	struct i2c_client *client;
	struct kionix_accel_platform_data accel_pdata;
	struct input_dev *input_dev;
	struct delayed_work accel_work;
	struct workqueue_struct *accel_workqueue;
	wait_queue_head_t wqh_suspend;

	int accel_data[3];
	int accel_cali[3];
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	bool negate_x;
	bool negate_y;
	bool negate_z;
	u8 shift;

	unsigned int poll_interval;
	unsigned int poll_delay;
	unsigned int accel_group;
	u8 *accel_registers;

	atomic_t accel_suspended;
	atomic_t accel_suspend_continue;
	atomic_t accel_enabled;
	atomic_t accel_input_event;
	atomic_t accel_enable_resume;
	struct mutex mutex_earlysuspend;
	struct mutex mutex_resume;
	rwlock_t rwlock_accel_data;

	bool accel_drdy;

	/* Function callback */
	int (*kionix_accel_report_accel_data)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_update_odr)(struct kionix_accel_driver *acceld, unsigned int poll_interval);
	int (*kionix_accel_power_on_init)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_operate)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_standby)(struct kionix_accel_driver *acceld);

  #if 0//def    CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
  #endif /* CONFIG_HAS_EARLYSUSPEND */

  //longjiang add start 20131017
  #ifdef CONFIG_SENSOR_POWER
	struct regulator *vcc_i2c;
	struct regulator *vdd_ana;
  #endif
  //add end
};
#define I2C_RETRY_TIMES 3
#define SENSORS_I2C_RETRY_ERR

#ifdef SENSORS_I2C_RETRY_ERR
static int i2c_retry_err;
#endif

#ifdef CONFIG_SENSORS
static struct sensors_classdev sensors_accelerometer_cdev = {
	.name = "accelerometer",
	.vendor = "kionix",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "100000",
	.resolution = "1.0",
	.sensor_power = "0.1",
	.min_delay = 20000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

static int kionix_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
	int err = 0;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	do {
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err < 0) {
			pr_debug("%s: i2c_read failed err = %d, tries = %d\n",
				__func__, err, tries);
			msleep(5);
		}
	} while ((err < 0) && (++tries < I2C_RETRY_TIMES));

#ifdef SENSORS_I2C_RETRY_ERR
	if (err < 0) {
		i2c_retry_err = 1;
		pr_err("%s: i2c_retry %d failed i2c_retry_err = %d\n",
			__func__, tries, i2c_retry_err);
	}
#endif

	return err;
}

static int kionix_i2c_write(struct i2c_client *client, u8 addr, u8 value)
{
	int err = 0;
	int tries = 0;

	do {
		err = i2c_smbus_write_byte_data(client, addr, value);
		if (err < 0) {
			pr_debug("%s: i2c_write failed err = %d, tries = %d\n",
				__func__, err, tries);
			msleep(5);
		}
	} while ((err < 0) && (++tries < I2C_RETRY_TIMES));

#ifdef SENSORS_I2C_RETRY_ERR
	if (err < 0) {
		i2c_retry_err = 1;
		pr_debug("%s: i2c_retry %d failed i2c_retry_err = %d\n",
			__func__, tries, i2c_retry_err);
	}
#endif

	return err;
}

/*longjiang add start 20130217*/
static long gsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
struct kionix_accel_driver *kionix_drv = NULL;
char kionix_accel_reg_addr = 0;

static int gsensor_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int gsensor_close(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations gsensor_fops = {
	.owner	= THIS_MODULE,
	.open 	= gsensor_open,
	.release = gsensor_close,
	#ifdef CONFIG_COMPAT
	.compat_ioctl   = gsensor_ioctl,
	#endif
	.unlocked_ioctl	= gsensor_ioctl

};

static struct miscdevice gsensor_misc = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= KIONIX_DEVICE_NAME,//longjiang add 20130620
	.fops 	= &gsensor_fops
};
/*longjiang add end 20130217*/

static int kionix_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
	char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
	char **token2 = token;
	unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
	int i = 0, start = 0, end = (int)count;

	strcpy(buf2, buf);

	/* We need to breakup the string into separate chunks in order for kstrtoint
	 * or strict_strtol to parse them without returning an error. Stop when the end of
	 * the string is reached or when enough value is read from the string */
	while((start < end) && (i < token_nr)) {
		/* We found a negative sign */
		if(*(buf2 + start) == '-') {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				/* If there is a pending negative sign, we adjust the variables to account for it */
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
				/* Reset */
				num_ptr = num_nr = 0;
			}
			/* This indicates that there is a pending negative sign in the string */
			num_neg = 1;
		}
		/* We found a numeric */
		else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
			/* If the previous char(s) are not numeric, set num_ptr to current char */
			if(num_nr < 1)
				num_ptr = start;
			num_nr++;
		}
		/* We found an unwanted character */
		else {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
			}
			/* Reset all the variables to start afresh */
			num_ptr = num_nr = num_neg = 0;
		}
		start++;
	}

	kfree(buf2);

	return (i == token_nr) ? token_nr : -1;
}

static int kionix_accel_grp1_power_on_init(struct kionix_accel_driver *acceld)
{
	int err;

	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP1_CTRL_REG1, acceld->accel_registers[accel_grp1_ctrl_reg1] | ACCEL_GRP1_PC1_ON);
		if (err < 0)
			return err;
	}
	else {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP1_CTRL_REG1, acceld->accel_registers[accel_grp1_ctrl_reg1]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp1_operate(struct kionix_accel_driver *acceld)
{
	int err;

	err = kionix_i2c_write(acceld->client, ACCEL_GRP1_CTRL_REG1,
			acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP1_PC1_ON);
	if (err < 0)
		return err;

	queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return 0;
}

static int kionix_accel_grp1_standby(struct kionix_accel_driver *acceld)
{
	int err;

	cancel_delayed_work_sync(&acceld->accel_work);

	err = kionix_i2c_write(acceld->client, ACCEL_GRP1_CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

static int kionix_accel_grp1_report_accel_data(struct kionix_accel_driver *acceld)
{
	u8 accel_data[3];
	s16 x, y, z;
	int err=0;
	struct input_dev *input_dev = acceld->input_dev;
	int loop = KIONIX_I2C_RETRY_COUNT;

	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP1_XOUT, accel_data, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(((s16)(accel_data[acceld->axis_map_x] >> 2)) - 32)) << 6;
				y = ((s16) le16_to_cpu(((s16)(accel_data[acceld->axis_map_y] >> 2)) - 32)) << 6;
				z = ((s16) le16_to_cpu(((s16)(accel_data[acceld->axis_map_z] >> 2)) - 32)) << 6;

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

				if(atomic_read(&acceld->accel_input_event) > 0) {
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}
        return err;
}

static int kionix_accel_grp1_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_grp1_odr_table); i++) {
		odr = kionix_accel_grp1_odr_table[i].mask;
		if (poll_interval < kionix_accel_grp1_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update CTRL_REG1 register if the ODR is not changed */
	if((acceld->accel_registers[accel_grp1_ctrl_reg1] & ACCEL_GRP1_ODR_MASK) == odr)
		return 0;
	else {
		acceld->accel_registers[accel_grp1_ctrl_reg1] &= ~ACCEL_GRP1_ODR_MASK;
		acceld->accel_registers[accel_grp1_ctrl_reg1] |= odr;
	}

	/* Do not need to update CTRL_REG1 register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld->client, ACCEL_GRP1_CTRL_REG1,
				acceld->accel_registers[accel_grp1_ctrl_reg1] | ACCEL_GRP1_PC1_ON);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp2_power_on_init(struct kionix_accel_driver *acceld)
{
	int err;

	/* ensure that PC1 is cleared before updating control registers */
	err = kionix_i2c_write(acceld->client,
					ACCEL_GRP2_CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = kionix_i2c_write(acceld->client,
					ACCEL_GRP2_DATA_CTRL, acceld->accel_registers[accel_grp2_data_ctrl]);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (acceld->client->irq) {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP2_INT_CTRL1, acceld->accel_registers[accel_grp2_int_ctrl]);
		if (err < 0)
			return err;
	}

	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP2_CTRL_REG1, acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP2_PC1_ON);
		if (err < 0)
			return err;
	}
	else {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP2_CTRL_REG1, acceld->accel_registers[accel_grp2_ctrl_reg1]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp2_operate(struct kionix_accel_driver *acceld)
{
	int err;

	err = kionix_i2c_write(acceld->client, ACCEL_GRP2_CTRL_REG1,
			acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP2_PC1_ON);
	if (err < 0)
		return err;

	if(acceld->accel_drdy == 0)
		queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return 0;
}

static int kionix_accel_grp2_standby(struct kionix_accel_driver *acceld)
{
	int err;

	if(acceld->accel_drdy == 0)
		cancel_delayed_work_sync(&acceld->accel_work);

	err = kionix_i2c_write(acceld->client, ACCEL_GRP2_CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

static int kionix_accel_grp2_report_accel_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8	accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int err=0;
	struct input_dev *input_dev = acceld->input_dev;
	int loop;
	/* Only read the output registers if enabled */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			loop = KIONIX_I2C_RETRY_COUNT;
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP2_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
				y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
				z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

				if(atomic_read(&acceld->accel_input_event) > 0) {
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}

	/* Clear the interrupt if using drdy */
	if(acceld->accel_drdy == 1) {
		loop = KIONIX_I2C_RETRY_COUNT;
		while(loop) {
			err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP2_INT_REL);
			if(err < 0){
				loop--;
				mdelay(KIONIX_I2C_RETRY_TIMEOUT);
			}
			else
				loop = 0;
		}
		if (err < 0)
			KMSGERR(&acceld->client->dev, "%s: clear interrupt error = %d\n", __func__, err);
	}
        return err;
}

static void kionix_accel_grp2_update_g_range(struct kionix_accel_driver *acceld)
{
	acceld->accel_registers[accel_grp2_ctrl_reg1] &= ~ACCEL_GRP2_G_MASK;

	switch (acceld->accel_pdata.accel_g_range) {
		case KIONIX_ACCEL_G_8G:
		case KIONIX_ACCEL_G_6G:
			acceld->shift = 2;
			acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_G_8G;
			break;
		case KIONIX_ACCEL_G_4G:
			acceld->shift = 3;
			acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_G_4G;
			break;
		case KIONIX_ACCEL_G_2G:
		default:
			acceld->shift = 4;
			acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_G_2G;
			break;
	}

	return;
}

static int kionix_accel_grp2_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_grp2_odr_table); i++) {
		odr = kionix_accel_grp2_odr_table[i].mask;
		if (poll_interval < kionix_accel_grp2_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
	if(acceld->accel_registers[accel_grp2_data_ctrl] == odr)
		return 0;
	else
		acceld->accel_registers[accel_grp2_data_ctrl] = odr;

	/* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld->client, ACCEL_GRP2_CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = kionix_i2c_write(acceld->client, ACCEL_GRP2_DATA_CTRL,
				acceld->accel_registers[accel_grp2_data_ctrl]);
		if (err < 0)
			return err;

		err = kionix_i2c_write(acceld->client, ACCEL_GRP2_CTRL_REG1,
			acceld->accel_registers[accel_grp2_ctrl_reg1] |
			ACCEL_GRP2_PC1_ON);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp4_power_on_init(struct kionix_accel_driver *acceld)
{
	int err;

	/* ensure that PC1 is cleared before updating control registers */
	err = kionix_i2c_write(acceld->client,
					ACCEL_GRP4_CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = kionix_i2c_write(acceld->client,
					ACCEL_GRP4_DATA_CTRL, acceld->accel_registers[accel_grp4_data_ctrl]);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (acceld->client->irq) {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP4_INT_CTRL1, acceld->accel_registers[accel_grp4_int_ctrl]);
		if (err < 0)
			return err;
	}

	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP4_CTRL_REG1, acceld->accel_registers[accel_grp4_ctrl_reg1] | ACCEL_GRP4_PC1_ON);
		if (err < 0)
			return err;
	}
	else {
		err = kionix_i2c_write(acceld->client,
						ACCEL_GRP4_CTRL_REG1, acceld->accel_registers[accel_grp4_ctrl_reg1]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp4_operate(struct kionix_accel_driver *acceld)
{
	int err;
	err = kionix_i2c_write(acceld->client, ACCEL_GRP4_CTRL_REG1,
			acceld->accel_registers[accel_grp4_ctrl_reg1] | ACCEL_GRP4_PC1_ON);
	if (err < 0)
		return err;
	if(acceld->accel_drdy == 0)
		queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return 0;
}

static int kionix_accel_grp4_standby(struct kionix_accel_driver *acceld)
{
	int err;

	if(acceld->accel_drdy == 0)
		cancel_delayed_work_sync(&acceld->accel_work);

	err = kionix_i2c_write(acceld->client, ACCEL_GRP4_CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

static int kionix_accel_grp4_report_accel_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8	accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int err=0;
	struct input_dev *input_dev = acceld->input_dev;
	int loop;

	/* Only read the output registers if enabled */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			loop = KIONIX_I2C_RETRY_COUNT;
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP4_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
				y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
				z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

				acceld->accel_data[acceld->axis_map_x] = ((acceld->negate_x ? -x : x) - acceld->accel_cali[acceld->axis_map_x])/4;
				acceld->accel_data[acceld->axis_map_y] = ((acceld->negate_y ? -y : y) - acceld->accel_cali[acceld->axis_map_y])/4;
				acceld->accel_data[acceld->axis_map_z] = ((acceld->negate_z ? -z : z) - acceld->accel_cali[acceld->axis_map_z])/4;

                                //chenyunzhe modify due to hal using 1024 as base,but 256 on the version 4.3,20140416
                              //  acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) - acceld->accel_cali[acceld->axis_map_x];
                              //  acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) - acceld->accel_cali[acceld->axis_map_y];
                              //  acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) - acceld->accel_cali[acceld->axis_map_z];
			  #if 0
				KMSGINF(&acceld->client->dev, "%s: x = %d,y = %d,z = %d\n", __func__,acceld->accel_data[acceld->axis_map_x],
				acceld->accel_data[acceld->axis_map_y], acceld->accel_data[acceld->axis_map_z]);
			  #endif
				//if(atomic_read(&acceld->accel_input_event) > 0) {//chenyunzhe remove,20140416
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				//}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}

	/* Clear the interrupt if using drdy */
	if(acceld->accel_drdy == 1) {
		loop = KIONIX_I2C_RETRY_COUNT;
		while(loop) {
			err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_INT_REL);
			if(err < 0){
				loop--;
				mdelay(KIONIX_I2C_RETRY_TIMEOUT);
			}
			else
				loop = 0;
		}
		if (err < 0)
			KMSGERR(&acceld->client->dev, "%s: clear interrupt error = %d\n", __func__, err);
	}
        return err;
}

static void kionix_accel_grp4_update_g_range(struct kionix_accel_driver *acceld)
{
	acceld->accel_registers[accel_grp4_ctrl_reg1] &= ~ACCEL_GRP4_G_MASK;

	switch (acceld->accel_pdata.accel_g_range) {
		case KIONIX_ACCEL_G_8G:
		case KIONIX_ACCEL_G_6G:
			acceld->shift = 2;
			acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_G_8G;
			break;
		case KIONIX_ACCEL_G_4G:
			acceld->shift = 3;
			acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_G_4G;
			break;
		case KIONIX_ACCEL_G_2G:
		default:
			acceld->shift = 4;
			acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_G_2G;
			break;
	}

	return;
}

static int kionix_accel_grp4_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_grp4_odr_table); i++) {
		odr = kionix_accel_grp4_odr_table[i].mask;
		if (poll_interval < kionix_accel_grp4_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
	if(acceld->accel_registers[accel_grp4_data_ctrl] == odr)
		return 0;
	else
		acceld->accel_registers[accel_grp4_data_ctrl] = odr;

	/* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = kionix_i2c_write(acceld->client, ACCEL_GRP4_CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = kionix_i2c_write(acceld->client, ACCEL_GRP4_DATA_CTRL,
			acceld->accel_registers[accel_grp4_data_ctrl]);
		if (err < 0)
			return err;

		err = kionix_i2c_write(acceld->client, ACCEL_GRP4_CTRL_REG1,
			acceld->accel_registers[accel_grp4_ctrl_reg1] |
				ACCEL_GRP4_PC1_ON);
		if (err < 0)
			return err;
		//#############
		err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_DATA_CTRL);
		if (err < 0)
			return err;
		switch(err) {
			case ACCEL_GRP4_ODR0_781:
				dev_info(&acceld->client->dev, "ODR = 0.781 Hz\n");
				break;
			case ACCEL_GRP4_ODR1_563:
				dev_info(&acceld->client->dev, "ODR = 1.563 Hz\n");
				break;
			case ACCEL_GRP4_ODR3_125:
				dev_info(&acceld->client->dev, "ODR = 3.125 Hz\n");
				break;
			case ACCEL_GRP4_ODR6_25:
				dev_info(&acceld->client->dev, "ODR = 6.25 Hz\n");
				break;
			case ACCEL_GRP4_ODR12_5:
				dev_info(&acceld->client->dev, "ODR = 12.5 Hz\n");
				break;
			case ACCEL_GRP4_ODR25:
				dev_info(&acceld->client->dev, "ODR = 25 Hz\n");
				break;
			case ACCEL_GRP4_ODR50:
				dev_info(&acceld->client->dev, "ODR = 50 Hz\n");
				break;
			case ACCEL_GRP4_ODR100:
				dev_info(&acceld->client->dev, "ODR = 100 Hz\n");
				break;
			case ACCEL_GRP4_ODR200:
				dev_info(&acceld->client->dev, "ODR = 200 Hz\n");
				break;
			case ACCEL_GRP4_ODR400:
				dev_info(&acceld->client->dev, "ODR = 400 Hz\n");
				break;
			case ACCEL_GRP4_ODR800:
				dev_info(&acceld->client->dev, "ODR = 800 Hz\n");
				break;
			case ACCEL_GRP4_ODR1600:
				dev_info(&acceld->client->dev, "ODR = 1600 Hz\n");
				break;
			default:
				dev_info(&acceld->client->dev, "Unknown ODR\n");
				break;
		}
		//#############
	}

	return 0;
}

static int kionix_accel_power_on(struct kionix_accel_driver *acceld)
{
	if (acceld->accel_pdata.power_on)
		return acceld->accel_pdata.power_on();

	return 0;
}

static void kionix_accel_power_off(struct kionix_accel_driver *acceld)
{
	if (acceld->accel_pdata.power_off)
		acceld->accel_pdata.power_off();
}

static irqreturn_t kionix_accel_isr(int irq, void *dev)
{
	struct kionix_accel_driver *acceld = dev;

	queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return IRQ_HANDLED;
}

static void kionix_accel_work(struct work_struct *work)
{
	struct kionix_accel_driver *acceld = container_of((struct delayed_work *)work,	struct kionix_accel_driver, accel_work);
        int err=0;
        err=acceld->kionix_accel_report_accel_data(acceld);
	if( err<0 )
        {
           printk(KERN_ERR "%s: report_accel_data failed, err=%d\n", __func__, err);
           return;
        }
        if(acceld->accel_drdy == 0)
		queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, acceld->poll_delay);

}

static void kionix_accel_update_direction(struct kionix_accel_driver *acceld)
{
    struct kionix_accel_platform_data *pdata = NULL;

    if(NULL == acceld)
      return ;

    pdata =  &acceld->accel_pdata;

    write_lock(&acceld->rwlock_accel_data);
        acceld->axis_map_x = pdata->map_x;
        acceld->axis_map_y = pdata->map_y;
        acceld->axis_map_z = pdata->map_z;

        acceld->negate_x = pdata->neg_x;
        acceld->negate_y = pdata->neg_y;
        acceld->negate_z = pdata->neg_z;
        write_unlock(&acceld->rwlock_accel_data);
}

static int kionix_accel_enable(struct kionix_accel_driver *acceld)
{
	int err = 0;
	long remaining;

        if (atomic_read(&acceld->accel_enabled) == 1)
        {
           return err;
        }
	mutex_lock(&acceld->mutex_earlysuspend);

	atomic_set(&acceld->accel_suspend_continue, 0);

	/* Make sure that the sensor had successfully resumed before enabling it */
	if(atomic_read(&acceld->accel_suspended) == 1) {
		KMSGINF(&acceld->client->dev, "%s: waiting for resume\n", __func__);
		remaining = wait_event_interruptible_timeout(acceld->wqh_suspend, \
				atomic_read(&acceld->accel_suspended) == 0, \
				msecs_to_jiffies(KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT));

		if(atomic_read(&acceld->accel_suspended) == 1) {
			KMSGERR(&acceld->client->dev, "%s: timeout waiting for resume\n", __func__);
			err = -ETIME;
			goto exit;
		}
	}

	err = acceld->kionix_accel_operate(acceld);

	if (err < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: kionix_accel_operate returned err = %d\n", __func__, err);
		goto exit;
	}

	atomic_set(&acceld->accel_enabled,1);

exit:
	mutex_unlock(&acceld->mutex_earlysuspend);

	return err;
}

static int kionix_accel_disable(struct kionix_accel_driver *acceld)
{
	int err = 0;

        if (atomic_read(&acceld->accel_enabled) == 0)
        {
           return err;
        }

	mutex_lock(&acceld->mutex_resume);

	atomic_set(&acceld->accel_suspend_continue, 1);

	//if(atomic_read(&acceld->accel_enabled) > 0){
	//	if(atomic_dec_and_test(&acceld->accel_enabled)) {
			if(atomic_read(&acceld->accel_enable_resume) > 0)
				atomic_set(&acceld->accel_enable_resume, 0);
			err = acceld->kionix_accel_standby(acceld);
			if (err < 0) {
				KMSGERR(&acceld->client->dev, \
						"%s: kionix_accel_standby returned err = %d\n", __func__, err);
				goto exit;
			}
                        atomic_set(&acceld->accel_enabled,0);
			wake_up_interruptible(&acceld->wqh_suspend);
	//	}
	//}

exit:
	mutex_unlock(&acceld->mutex_resume);

	return err;
}

static int kionix_accel_input_open(struct input_dev *input)
{
	//struct kionix_accel_driver *acceld = input_get_drvdata(input);

	//atomic_inc(&acceld->accel_input_event); //chenyunzhe removed,201440416

	return 0;
}

static void kionix_accel_input_close(struct input_dev *dev)
{
	//struct kionix_accel_driver *acceld = input_get_drvdata(dev);

	//atomic_dec(&acceld->accel_input_event); //chenyunzhe removed,201440416
}

static void /*__devinit*/ kionix_accel_init_input_device(struct kionix_accel_driver *acceld,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
	input_set_abs_params(input_dev, ABS_Y, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
	input_set_abs_params(input_dev, ABS_Z, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);

	input_dev->name = KIONIX_INPUT_NAME;//longjiang add 20130620
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &acceld->client->dev;
}

static int /*__devinit*/ kionix_accel_setup_input_device(struct kionix_accel_driver *acceld)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		KMSGERR(&acceld->client->dev, "input_allocate_device failed\n");
		return -ENOMEM;
	}

	acceld->input_dev = input_dev;

	input_dev->open = kionix_accel_input_open;
	input_dev->close = kionix_accel_input_close;
	input_set_drvdata(input_dev, acceld);

	kionix_accel_init_input_device(acceld, input_dev);

	err = input_register_device(acceld->input_dev);
	if (err) {
		KMSGERR(&acceld->client->dev, \
				"%s: input_register_device returned err = %d\n", __func__, err);
		input_free_device(acceld->input_dev);
		return err;
	}

	return 0;
}

/*longjiang add start 20130520*/
static int kionix_read_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8	accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int err;
	struct input_dev *input_dev = acceld->input_dev;
	int loop;

	loop = KIONIX_I2C_RETRY_COUNT;
	while(loop) {
		mutex_lock(&input_dev->mutex);
		err = kionix_i2c_read(acceld->client, ACCEL_GRP4_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
		mutex_unlock(&input_dev->mutex);
		if(err < 0){
			loop--;
			mdelay(KIONIX_I2C_RETRY_TIMEOUT);
		}
		else
			loop = 0;
	}
	if (err < 0) {
		KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
	}
	else {
		write_lock(&acceld->rwlock_accel_data);

		x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
		y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
		z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

		acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x);
		acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y);
		acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z);

		write_unlock(&acceld->rwlock_accel_data);
	}

	return err;
}
static  short xavg = 0, yavg = 0, zavg = 0;

static int kionix_accel_Calibration(int count)
{
	int ret = 0;
	int i = 0;
	struct kionix_accel_driver *acceld = kionix_drv;

        xavg=yavg=zavg=0;

	memset(&kionix_drv->accel_cali, 0, sizeof(kionix_drv->accel_cali));

	for (i = 0; i < count; i++) {
		mdelay(65);
		ret = kionix_read_data(acceld);
 		if (ret < 0) {
			KMSGERR(&acceld->client->dev, "%s: read data failed\n", __func__);
			return ret;
		}
		KMSGINF(&acceld->client->dev, "%s: x = %d, y = %d, z= %d\n", __func__, acceld->accel_data[acceld->axis_map_x],
			acceld->accel_data[acceld->axis_map_y], acceld->accel_data[acceld->axis_map_z]);
		xavg += acceld->accel_data[acceld->axis_map_x];
		yavg += acceld->accel_data[acceld->axis_map_y];
		zavg += acceld->accel_data[acceld->axis_map_z];
	}
	xavg = xavg/count;
	yavg = yavg/count;
	zavg = zavg/count;
	printk(KERN_INFO "%s: xavg = %d, yavg = %d, zavg = %d\n", __func__, xavg, yavg, zavg);

        if(((xavg > -CAL_FUZZ)&&(xavg < CAL_FUZZ))&&((yavg > -CAL_FUZZ)&&(yavg < CAL_FUZZ))&&
                ((zavg > (CAL_CONVERT - CAL_FUZZ))&&(zavg < (CAL_CONVERT + CAL_FUZZ))))
        {
                kionix_drv->accel_cali[acceld->axis_map_x] = xavg;
                kionix_drv->accel_cali[acceld->axis_map_y] = yavg;
                kionix_drv->accel_cali[acceld->axis_map_z] = zavg - CAL_CONVERT;
                KMSGINF(&acceld->client->dev, "%s(9.8): success!\n", __func__);
                ret = 0;
        } else if (((xavg > -CAL_FUZZ)&&(xavg < CAL_FUZZ))&&((yavg > -CAL_FUZZ)&&(yavg < CAL_FUZZ))&&
                ((zavg > (-CAL_CONVERT - CAL_FUZZ))&&(zavg < (-CAL_CONVERT + CAL_FUZZ))))
        {
            kionix_drv->accel_cali[acceld->axis_map_x] = xavg;
                kionix_drv->accel_cali[acceld->axis_map_y] = yavg;
                kionix_drv->accel_cali[acceld->axis_map_z] = zavg + CAL_CONVERT;
                KMSGINF(&acceld->client->dev, "%s(-9.8): success!\n", __func__);
                ret = 0;
        } else {
                KMSGINF(&acceld->client->dev, "%s: failed!\n", __func__);
                ret = -1;
        }

	return ret;
}

static long gsensor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	short flag;

	struct kionix_accel_driver *acceld = kionix_drv;
	struct input_dev *input_dev = acceld->input_dev;
	//longjiang add start 20130620
	struct acc_offset acc_cal_data;
	struct acc_offset *acc_cal_ptr = &acc_cal_data;
	//add end

	KMSGINF(&acceld->client->dev, "%s: cmd = %u\n", __func__, cmd);

	switch(cmd)
	{
	  case KIONIX_IOCTL_APP_SET_AFLAG:
		if(copy_from_user(&flag, (short*)arg, sizeof(flag))){
			KMSGERR(&acceld->client->dev, "%s: copy_from_user AFLAG failed!\n", __func__);
			return -EFAULT;
		}
		printk(KERN_INFO "%s: %s: flag = %s\n", KIONIX_ACCEL_NAME, __func__, flag ? "Open" : "Close");
		mutex_lock(&input_dev->mutex);
		if(flag == 1)
		{
			ret = kionix_accel_enable(acceld);
			if(ret < 0){
				KMSGERR(&acceld->client->dev, "%s: can't enable kionix\n", __func__);
				mutex_unlock(&input_dev->mutex);
				return ret;
			}
		}
		else if(flag == 0)
		{
			ret = kionix_accel_disable(acceld);
			if (ret < 0) {
				KMSGERR(&acceld->client->dev, "%s: can't disable kionix\n", __func__);
				mutex_unlock(&input_dev->mutex);
				return ret;
			}
		}
		mutex_unlock(&input_dev->mutex);

		break;

	  case KIONIX_IOCTL_APP_SET_DELAY:
		if(copy_from_user(&flag, (short*)arg, sizeof(flag))){
			KMSGERR(&acceld->client->dev, "%s: copy_from_user DELAY failed!\n", __func__);
			return -EFAULT;
		}

	 	KMSGINF(&acceld->client->dev, "%s: set delay = %u\n", __func__, flag);
		acceld->poll_interval = flag;
		acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);
		ret = acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);
		if(ret < 0)
			KMSGERR(&acceld->client->dev, "%s: can't update odr\n", __func__);

		break;

	  //longjiang add platform code 20130520
	  case KIONIX_IOCTL_APP_CALIBRATE:
		KMSGINF(&acceld->client->dev, "%s: Calibrate start\n",  __func__);

		ret = kionix_accel_Calibration(10);
		if(ret < 0)
			KMSGERR(&acceld->client->dev, "%s: set default offset!\n", __func__);

		((struct acc_offset *)acc_cal_ptr)->x = kionix_drv->accel_cali[acceld->axis_map_x];
		((struct acc_offset *)acc_cal_ptr)->y = kionix_drv->accel_cali[acceld->axis_map_y];
		((struct acc_offset *)acc_cal_ptr)->z = kionix_drv->accel_cali[acceld->axis_map_z];
		((struct acc_offset *)acc_cal_ptr)->key = ret ? 2 : 1;

		printk(KERN_INFO "%s write: x = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
		printk(KERN_INFO "%s write: y = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
		printk(KERN_INFO "%s write: z = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->z);
		printk(KERN_INFO "%s write: key = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->key);

		sensparams_write_to_flash(SENSPARAMS_TYPE_ACC, (unsigned char*)acc_cal_ptr, sizeof(acc_cal_data));

		if(ret<0)
		{
		    /* added by liwenpeng */
		    acc_cal_ptr->x=xavg;
		    acc_cal_ptr->y=yavg;
		    acc_cal_ptr->z=zavg>0?(zavg-1024):(zavg+1024);
		}

		if(copy_to_user((struct acc_offset *)arg, acc_cal_ptr, sizeof(acc_cal_data)))
		{
			KMSGERR(&acceld->client->dev,  "%s:  Calibrate copy_to_user failed!\n", __func__);
		}

		break;

	  case KIONIX_IOCTL_APP_OFFSET:
		sensparams_read_from_flash(SENSPARAMS_TYPE_ACC, (unsigned char*)acc_cal_ptr, sizeof(acc_cal_data));
  		kionix_drv->accel_cali[acceld->axis_map_x] = ((struct acc_offset *)acc_cal_ptr)->x;
		kionix_drv->accel_cali[acceld->axis_map_y] = ((struct acc_offset *)acc_cal_ptr)->y;
		kionix_drv->accel_cali[acceld->axis_map_z] = ((struct acc_offset *)acc_cal_ptr)->z;


		if(copy_to_user((short *)arg, &acc_cal_ptr->key, sizeof(short)))//added by wudongxing for factory pattern.2013.11.07
		{
			KMSGERR(&acceld->client->dev,  "%s:  get offset copy_to_user failed!\n", __func__);
		}

		printk(KERN_INFO "%s read: xoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
		printk(KERN_INFO "%s read: yoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
		printk(KERN_INFO "%s read: zoffset = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->z);
		printk(KERN_INFO "%s read: CalStatus = %d\n",  __func__,((struct acc_offset *)acc_cal_ptr)->key);

		break;
	  //add end

	  default:
		return -ENOTTY;
	}

	return 0;
}
//add end

/* Returns the enable state of device */
static ssize_t kionix_accel_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&acceld->accel_enabled) > 0 ? 1 : 0);
}

/* Allow users to enable/disable the device */
static ssize_t kionix_accel_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int enable_count = 1;
	unsigned long enable;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, enable_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: No enable data being read. " \
				"No enable data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&enable);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &enable);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
		#endif

		if(enable)
			err = kionix_accel_enable(acceld);
		else
			err = kionix_accel_disable(acceld);
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t kionix_accel_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", acceld->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kionix_accel_set_delay(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int delay_count = 1;
	unsigned long interval;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, delay_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: No delay data being read. " \
				"No delay data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&interval);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &interval);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
		#endif

		if(acceld->accel_drdy == 1)
			disable_irq(client->irq);

		/*
		 * Set current interval to the greater of the minimum interval or
		 * the requested interval
		 */
		acceld->poll_interval = max((unsigned int)interval, acceld->accel_pdata.min_interval);
		acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);

		err = acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);

		if(acceld->accel_drdy == 1)
			enable_irq(client->irq);
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* Returns the direction of device */
static ssize_t kionix_accel_get_direct(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", acceld->accel_pdata.accel_direction);
}

/* Allow users to change the direction the device */
static ssize_t kionix_accel_set_direct(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int direct_count = 1;
	unsigned long direction;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, direct_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: No direction data being read. " \
				"No direction data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&direction);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &direction);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
		#endif

		if(direction < 1 || direction > 8)
			KMSGERR(&acceld->client->dev, "%s: invalid direction = %d\n", __func__, (unsigned int) direction);

		else {
			acceld->accel_pdata.accel_direction = (u8) direction;
			kionix_accel_update_direction(acceld);
		}
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* Returns the data output of device */
static ssize_t kionix_accel_get_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int x, y, z;

	read_lock(&acceld->rwlock_accel_data);

	x = acceld->accel_data[acceld->axis_map_x];
	y = acceld->accel_data[acceld->axis_map_y];
	z = acceld->accel_data[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);

	return sprintf(buf, "%d %d %d\n", x, y, z);
}

/* Returns the calibration value of the device */
static ssize_t kionix_accel_get_cali(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int calibration[3];

	read_lock(&acceld->rwlock_accel_data);

	calibration[0] = acceld->accel_cali[acceld->axis_map_x];
	calibration[1] = acceld->accel_cali[acceld->axis_map_y];
	calibration[2] = acceld->accel_cali[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);

	return sprintf(buf, "%d %d %d\n", calibration[0], calibration[1], calibration[2]);
}

/* Allow users to change the calibration value of the device */
static ssize_t kionix_accel_set_cali(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	const int cali_count = 3; /* How many calibration that we expect to get from the string */
	char **buf2;
	long calibration[cali_count];
	int err = 0, i = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	buf2 = (char **)kzalloc(cali_count * sizeof(char *), GFP_KERNEL);

	if(kionix_strtok(buf, count, buf2, cali_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: Not enough calibration data being read. " \
				"No calibration data will be updated.\n", __func__);
	}
	else {
		/* Convert string to integers  */
		for(i = 0 ; i < cali_count ; i++) {
			#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
			err = kstrtoint((const char *)*(buf2+i), 10, (int *)&calibration[i]);
			if(err < 0) {
				KMSGERR(&acceld->client->dev, \
						"%s: kstrtoint returned err = %d." \
						"No calibration data will be updated.\n", __func__ , err);
				goto exit;
			}
			#else
			err = strict_strtol((const char *)*(buf2+i), 10, &calibration[i]);
			if(err < 0) {
				KMSGERR(&acceld->client->dev, \
						"%s: strict_strtol returned err = %d." \
						"No calibration data will be updated.\n", __func__ , err);
				goto exit;
			}
			#endif
		}

		write_lock(&acceld->rwlock_accel_data);

		acceld->accel_cali[acceld->axis_map_x] = (int)calibration[0];
		acceld->accel_cali[acceld->axis_map_y] = (int)calibration[1];
		acceld->accel_cali[acceld->axis_map_z] = (int)calibration[2];

		write_unlock(&acceld->rwlock_accel_data);
	}

exit:
	for(i = 0 ; i < cali_count ; i++)
		kfree(*(buf2+i));

	kfree(buf2);

	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/*longjiang add attribute start 20130217*/
static ssize_t kionix_accel_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	char data = 0;
	int ret;

	data = i2c_smbus_read_byte_data(client, kionix_accel_reg_addr);
	ret = sprintf(buf, "reg0x%02x = %02x\n", kionix_accel_reg_addr, data);

	return ret;
}

static ssize_t kionix_accel_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);

	unsigned long val;
	int ret;
	if(strict_strtoul(buf, 16, &val))
		return -EINVAL;

	ret = kionix_i2c_write(client, kionix_accel_reg_addr, val);
	if(!ret)
		printk(KERN_ERR "kionix write reg addr error!\n");

	return size;
}

static ssize_t kionix_accel_addr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val;

	if(strict_strtoul(buf, 16, &val))
		return -EINVAL;
	kionix_accel_reg_addr = val;

	return size;
}

static DEVICE_ATTR(reg_value, S_IRUGO|S_IWUSR, kionix_accel_reg_show, kionix_accel_reg_store);
static DEVICE_ATTR(reg_addr, S_IWUSR, NULL, kionix_accel_addr_store);
/*longjiang add attribute end 20130217*/

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, kionix_accel_get_enable, kionix_accel_set_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kionix_accel_get_delay, kionix_accel_set_delay);
static DEVICE_ATTR(direct, S_IRUGO|S_IWUSR, kionix_accel_get_direct, kionix_accel_set_direct);
static DEVICE_ATTR(data, S_IRUGO, kionix_accel_get_data, NULL);
static DEVICE_ATTR(cali, S_IRUGO|S_IWUSR, kionix_accel_get_cali, kionix_accel_set_cali);

static struct attribute *kionix_accel_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_direct.attr,
	&dev_attr_data.attr,
	&dev_attr_cali.attr,
	&dev_attr_reg_value.attr,//longjiang add 20130217
	&dev_attr_reg_addr.attr,//longjiang add 20130217
	NULL
};

static struct attribute_group kionix_accel_attribute_group = {
	.attrs = kionix_accel_attributes
};

static int /*__devinit*/ kionix_verify(struct kionix_accel_driver *acceld)
{
	int retval = i2c_smbus_read_byte_data(acceld->client, ACCEL_WHO_AM_I);

        printk("%s(): chip_id=%d\n",__func__, retval);
#if KIONIX_KMSG_ERR
	switch (retval) {
		case KIONIX_ACCEL_WHO_AM_I_KXTE9:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTE9.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTF9:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTF9.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTI9_1001:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTI9-1001.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTIK_1004:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTIK-1004.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTJ9-1005.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTJ9-1007.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXCJ9-1008.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTJ2-1009.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXCJK_1013:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXCJK-1013.\n");
			break;
		default:
			break;
	}
#endif

	return retval;
}

#if 0//def    CONFIG_HAS_EARLYSUSPEND
void kionix_accel_earlysuspend_suspend(struct early_suspend *h)
{
	struct kionix_accel_driver *acceld = container_of(h, struct kionix_accel_driver, early_suspend);
	long remaining;

	mutex_lock(&acceld->mutex_earlysuspend);

	/* Only continue to suspend if enable did not intervene */
	if(atomic_read(&acceld->accel_suspend_continue) > 0) {
		/* Make sure that the sensor had successfully disabled before suspending it */
		if(atomic_read(&acceld->accel_enabled) > 0) {
			KMSGINF(&acceld->client->dev, "%s: waiting for disable\n", __func__);
			remaining = wait_event_interruptible_timeout(acceld->wqh_suspend, \
					atomic_read(&acceld->accel_enabled) < 1, \
					msecs_to_jiffies(KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT));

			if(atomic_read(&acceld->accel_enabled) > 0) {
				KMSGERR(&acceld->client->dev, "%s: timeout waiting for disable\n", __func__);
			}
		}

		kionix_accel_power_off(acceld);

		atomic_set(&acceld->accel_suspended, 1);
	}

	mutex_unlock(&acceld->mutex_earlysuspend);

	return;
}

void kionix_accel_earlysuspend_resume(struct early_suspend *h)
{
	struct kionix_accel_driver *acceld = container_of(h, struct kionix_accel_driver, early_suspend);
	int err;

	mutex_lock(&acceld->mutex_resume);

	if(atomic_read(&acceld->accel_suspended) == 1) {
		err = kionix_accel_power_on(acceld);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on returned err = %d\n", __func__, err);
			goto exit;
		}

		/* Only needs to reinitialized the registers if Vdd is pulled low during suspend */
		if(err > 0) {
			err = acceld->kionix_accel_power_on_init(acceld);
			if (err) {
				KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on_init returned err = %d\n", __func__, err);
				goto exit;
			}
		}

		atomic_set(&acceld->accel_suspended, 0);
	}

	wake_up_interruptible(&acceld->wqh_suspend);

exit:
	mutex_unlock(&acceld->mutex_resume);

	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

//longjiang add start 20131017
#ifdef CONFIG_SENSOR_POWER
static int sensors_power_init(struct kionix_accel_driver *acceld, bool on)
{
   	int rc;

    	if (!on)
        		goto pwr_deinit;

    	acceld->vdd_ana = regulator_get(&acceld->client->dev, "vdd_ana");
    	if (IS_ERR(acceld->vdd_ana)) {
        		rc = PTR_ERR(acceld->vdd_ana);
        		dev_err(&acceld->client->dev,
            		"Regulator get failed vdd_ana rc=%d\n", rc);
        		return rc;
    	}

    	if (regulator_count_voltages(acceld->vdd_ana) > 0) {
        		rc = regulator_set_voltage(acceld->vdd_ana, 1620000,3300000);
        		if (rc) {
            		dev_err(&acceld->client->dev,
                			"Regulator set_vtg failed vdd_ana rc=%d\n", rc);
            		goto reg_vdd_ana_put;
        		}
    	}

    	acceld->vcc_i2c = regulator_get(&acceld->client->dev, "vcc_i2c");
    	if (IS_ERR(acceld->vcc_i2c)) {
        		rc = PTR_ERR(acceld->vcc_i2c);
        		dev_err(&acceld->client->dev,
            		"Regulator get failed vcc_i2c rc=%d\n", rc);
        		goto reg_vdd_ana_set_vtg;
    	}

    	if (regulator_count_voltages(acceld->vcc_i2c) > 0) {
        		rc = regulator_set_voltage(acceld->vcc_i2c, 1800000,1800000);
        		if (rc) {
            		dev_err(&acceld->client->dev,
            			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
            		goto reg_vcc_i2c_put;
        		}
    	}

    	return 0;

reg_vcc_i2c_put:
   	regulator_put(acceld->vcc_i2c);

reg_vdd_ana_set_vtg:
    	if (regulator_count_voltages(acceld->vdd_ana) > 0)
        		regulator_set_voltage(acceld->vdd_ana, 0, 3300000);
reg_vdd_ana_put:
    	regulator_put(acceld->vdd_ana);
    	return rc;

pwr_deinit:
	if (regulator_count_voltages(acceld->vdd_ana) > 0)
		regulator_set_voltage(acceld->vdd_ana, 0, 3300000);
	regulator_put(acceld->vdd_ana);

	if (regulator_count_voltages(acceld->vcc_i2c) > 0)
        		regulator_set_voltage(acceld->vcc_i2c, 0, 1800000);
	regulator_put(acceld->vcc_i2c);

    	return 0;
}

static int sensors_power_on(struct kionix_accel_driver *acceld, bool on)
{
    	int rc,ret;

    	if (!on)
        		goto power_off;

    	rc = regulator_enable(acceld->vdd_ana);
    	if (rc) {
        		dev_err(&acceld->client->dev,
            		"Regulator vdd_ana enable failed rc=%d\n", rc);
        		return rc;
    	}
    	rc = regulator_enable(acceld->vcc_i2c);
    	if (rc) {
        		dev_err(&acceld->client->dev,
            		"Regulator vcc_i2c enable failed rc=%d\n", rc);
        		regulator_disable(acceld->vdd_ana);
    	}

    	return rc;

power_off:
    	rc = regulator_disable(acceld->vdd_ana);
    	if (rc) {
        		dev_err(&acceld->client->dev,
            		"Regulator vdd_ana disable failed rc=%d\n", rc);
        		return rc;
    	}
    	rc = regulator_disable(acceld->vcc_i2c);
    	if (rc) {
        		dev_err(&acceld->client->dev,
            		"Regulator vcc_i2c disable failed rc=%d\n", rc);
       		ret = regulator_enable(acceld->vdd_ana);
			if (ret) {
			dev_err(&acceld->client->dev,
                 	   "Regulator vdd_ana enable failed ret=%d\n", ret);
			return ret;
			}
    	}

    	return rc;
}
#endif

#ifdef CONFIG_OF
static int kionix_accel_parse_dt(struct device *dev, struct kionix_accel_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	rc = of_property_read_u32(np, "kionix,poll_interval", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read poll_interval\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->poll_interval= temp_val;

	rc = of_property_read_u32(np, "kionix,min_interval", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read min_interval\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->min_interval= temp_val;

	rc = of_property_read_u32(np, "kionix,accel_irq_use_drdy", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_irq_use_drdy\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->accel_irq_use_drdy= temp_val;

	rc = of_property_read_u32(np, "kionix,accel_res", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_res\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->accel_res= temp_val;

        rc = of_property_read_u32(np, "kionix,axis_map_x", &temp_val);
        if (rc && (rc != -EINVAL)){
                dev_err(dev, "Unable to read axis_map_x\n");
                return rc;
        } else if (rc != -EINVAL)
                pdata->map_x= temp_val;

        rc = of_property_read_u32(np, "kionix,axis_map_y", &temp_val);
        if (rc && (rc != -EINVAL)){
                dev_err(dev, "Unable to read axis_map_y\n");
                return rc;
        } else if (rc != -EINVAL)
                pdata->map_y= temp_val;

        rc = of_property_read_u32(np, "kionix,axis_map_z", &temp_val);
        if (rc && (rc != -EINVAL)){
                dev_err(dev, "Unable to read axis_map_z\n");
                return rc;
        } else if (rc != -EINVAL)
                pdata->map_z= temp_val;

        rc = of_property_read_u32(np, "kionix,negative_x", &temp_val);
        if (rc && (rc != -EINVAL)){
                dev_err(dev, "Unable to read negative_x\n");
                return rc;
        } else if (rc != -EINVAL)
                pdata->neg_x= temp_val;

        rc = of_property_read_u32(np, "kionix,negative_y", &temp_val);
        if (rc && (rc != -EINVAL)){
                dev_err(dev, "Unable to read negative_y\n");
                return rc;
        } else if (rc != -EINVAL)
                pdata->neg_y= temp_val;

        rc = of_property_read_u32(np, "kionix,negative_z", &temp_val);
        if (rc && (rc != -EINVAL)){
                dev_err(dev, "Unable to read negative_z\n");
                return rc;
        } else if (rc != -EINVAL)
                pdata->neg_z= temp_val;

        rc = of_property_read_u32(np, "kionix,accel_g_range", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_g_range\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->accel_g_range= (u8)temp_val;

	KMSGINF(dev, "poll_interval = %d, min_interval = %d\n", pdata->poll_interval, pdata->min_interval);
	KMSGINF(dev, "accel_g_range = %u, use_drdy = %d\n", pdata->accel_g_range, pdata->accel_irq_use_drdy);

	return 0;
}
#endif
//add end

static int /*__devinit*/ kionix_accel_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct kionix_accel_driver *acceld;
	int err;
//	struct proc_dir_entry *proc_dir, *proc_entry;
  //longjiang add start 20130620
  #ifdef CONFIG_OF
	struct kionix_accel_platform_data *accel_pdata;
  #else
	const struct kionix_accel_platform_data *accel_pdata = client->dev.platform_data;
  #endif
  //add end

	printk(KERN_INFO "%s: %s start.\n", KIONIX_ACCEL_NAME, __func__);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		KMSGERR(&client->dev, "client is not i2c capable. Abort.\n");
		return -ENXIO;
	}

  //longjiang add start 20130620
  #ifdef CONFIG_OF
	if(client->dev.of_node){
		accel_pdata = devm_kzalloc(&client->dev,
			sizeof(struct kionix_accel_platform_data), GFP_KERNEL);
		if (!accel_pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		err = kionix_accel_parse_dt(&client->dev, accel_pdata);
		if (err)
			return err;
	}else{
		accel_pdata = client->dev.platform_data;
		if (!accel_pdata)
			return -EINVAL;
	}
    	#else
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!accel_pdata) {
		KMSGERR(&client->dev, "platform data is NULL. Abort.\n");
		return -EINVAL;
	}
  #endif
  //add end

	acceld = kzalloc(sizeof(*acceld), GFP_KERNEL);
	if (acceld == NULL) {
		KMSGERR(&client->dev, \
			"failed to allocate memory for module data. Abort.\n");
		return -ENOMEM;
	}

	acceld->client = client;
	acceld->accel_pdata = *accel_pdata;

	i2c_set_clientdata(client, acceld);
	kionix_drv = acceld;

  //longjiang add start 20131017
  #ifdef CONFIG_SENSOR_POWER
	sensors_power_init(kionix_drv, true);
	sensors_power_on(kionix_drv, true);
	msleep(10);
  #endif
  //add end

	err = kionix_accel_power_on(acceld);
	if (err < 0)
		goto err_free_mem;

	if (accel_pdata->init) {
		err = accel_pdata->init();
		if (err < 0)
			goto err_accel_pdata_power_off;
	}

	err = kionix_verify(acceld);
	if (err < 0) {
		KMSGERR(&acceld->client->dev, "%s: kionix_verify returned err = %d. Abort.\n", __func__, err);
		goto err_accel_pdata_exit;
	}

	/* Setup group specific configuration and function callback */
	switch (err) {
		case KIONIX_ACCEL_WHO_AM_I_KXTE9:
			acceld->accel_group = KIONIX_ACCEL_GRP1;
			acceld->accel_registers = kzalloc(sizeof(u8)*accel_grp1_regs_count, GFP_KERNEL);
			if (acceld->accel_registers == NULL) {
				KMSGERR(&client->dev, \
					"failed to allocate memory for accel_registers. Abort.\n");
				goto err_accel_pdata_exit;
			}
			acceld->accel_drdy = 0;
			acceld->kionix_accel_report_accel_data	= kionix_accel_grp1_report_accel_data;
			acceld->kionix_accel_update_odr			= kionix_accel_grp1_update_odr;
			acceld->kionix_accel_power_on_init		= kionix_accel_grp1_power_on_init;
			acceld->kionix_accel_operate			= kionix_accel_grp1_operate;
			acceld->kionix_accel_standby			= kionix_accel_grp1_standby;
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTF9:
		case KIONIX_ACCEL_WHO_AM_I_KXTI9_1001:
		case KIONIX_ACCEL_WHO_AM_I_KXTIK_1004:
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005:
			if(err == KIONIX_ACCEL_WHO_AM_I_KXTIK_1004)
				acceld->accel_group = KIONIX_ACCEL_GRP3;
			else
				acceld->accel_group = KIONIX_ACCEL_GRP2;
			acceld->accel_registers = kzalloc(sizeof(u8)*accel_grp2_regs_count, GFP_KERNEL);
			if (acceld->accel_registers == NULL) {
				KMSGERR(&client->dev, \
					"failed to allocate memory for accel_registers. Abort.\n");
				goto err_accel_pdata_exit;
			}
			switch(acceld->accel_pdata.accel_res) {
				case KIONIX_ACCEL_RES_6BIT:
				case KIONIX_ACCEL_RES_8BIT:
					acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_RES_8BIT;
					break;
				case KIONIX_ACCEL_RES_12BIT:
				default:
					acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_RES_12BIT;
					break;
			}
			if(acceld->accel_pdata.accel_irq_use_drdy && client->irq) {
				acceld->accel_registers[accel_grp2_int_ctrl] |= ACCEL_GRP2_IEN | ACCEL_GRP2_IEA;
				acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_DRDYE;
				acceld->accel_drdy = 1;
			}
			else
				acceld->accel_drdy = 0;
			kionix_accel_grp2_update_g_range(acceld);
			acceld->kionix_accel_report_accel_data	= kionix_accel_grp2_report_accel_data;
			acceld->kionix_accel_update_odr			= kionix_accel_grp2_update_odr;
			acceld->kionix_accel_power_on_init		= kionix_accel_grp2_power_on_init;
			acceld->kionix_accel_operate			= kionix_accel_grp2_operate;
			acceld->kionix_accel_standby			= kionix_accel_grp2_standby;
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007:
		case KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008:
		case KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009:
		case KIONIX_ACCEL_WHO_AM_I_KXCJK_1013:
			if(err == KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009)
				acceld->accel_group = KIONIX_ACCEL_GRP5;
			else if(err == KIONIX_ACCEL_WHO_AM_I_KXCJK_1013)
				acceld->accel_group = KIONIX_ACCEL_GRP6;
			else
				acceld->accel_group = KIONIX_ACCEL_GRP4;
			acceld->accel_registers = kzalloc(sizeof(u8)*accel_grp4_regs_count, GFP_KERNEL);
			if (acceld->accel_registers == NULL) {
				KMSGERR(&client->dev, \
					"failed to allocate memory for accel_registers. Abort.\n");
				goto err_accel_pdata_exit;
			}
			switch(acceld->accel_pdata.accel_res) {
				case KIONIX_ACCEL_RES_6BIT:
				case KIONIX_ACCEL_RES_8BIT:
					acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_RES_8BIT;
					break;
				case KIONIX_ACCEL_RES_12BIT:
				default:
					acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_RES_12BIT;
					break;
			}
			if(acceld->accel_pdata.accel_irq_use_drdy && client->irq) {
				acceld->accel_registers[accel_grp4_int_ctrl] |= ACCEL_GRP4_IEN | ACCEL_GRP4_IEA;
				acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_DRDYE;
				acceld->accel_drdy = 1;
			}
			else
				acceld->accel_drdy = 0;
			kionix_accel_grp4_update_g_range(acceld);
			acceld->kionix_accel_report_accel_data	= kionix_accel_grp4_report_accel_data;
			acceld->kionix_accel_update_odr			= kionix_accel_grp4_update_odr;
			acceld->kionix_accel_power_on_init		= kionix_accel_grp4_power_on_init;
			acceld->kionix_accel_operate			= kionix_accel_grp4_operate;
			acceld->kionix_accel_standby			= kionix_accel_grp4_standby;
			break;
		default:
			KMSGERR(&acceld->client->dev, \
					"%s: unsupported device, who am i = %d. Abort.\n", __func__, err);
			goto err_accel_pdata_exit;
	}

	err = kionix_accel_setup_input_device(acceld);
	if (err)
		goto err_free_accel_registers;

	atomic_set(&acceld->accel_suspended, 0);
	atomic_set(&acceld->accel_suspend_continue, 1);
	atomic_set(&acceld->accel_enabled, 0);
	atomic_set(&acceld->accel_input_event, 0);
	atomic_set(&acceld->accel_enable_resume, 0);

	mutex_init(&acceld->mutex_earlysuspend);
	mutex_init(&acceld->mutex_resume);
	rwlock_init(&acceld->rwlock_accel_data);

	acceld->poll_interval = acceld->accel_pdata.poll_interval;
	acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);
	acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);
	kionix_accel_update_direction(acceld);

	/*proc_dir = proc_mkdir("sensors", NULL);
	if (proc_dir == NULL)
		KMSGERR(&client->dev, "failed to create /proc/sensors\n");
	else {
		proc_entry = create_proc_entry( "accelinfo", 0644, proc_dir);
		if (proc_entry == NULL)
			KMSGERR(&client->dev, "failed to create /proc/cpu/accelinfo\n");
	}*/

	acceld->accel_workqueue = create_workqueue("Kionix Accel Workqueue");
	INIT_DELAYED_WORK(&acceld->accel_work, kionix_accel_work);
	init_waitqueue_head(&acceld->wqh_suspend);

	if (acceld->accel_drdy) {
		err = request_threaded_irq(client->irq, NULL, kionix_accel_isr, \
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT, \
					   KIONIX_ACCEL_IRQ, acceld);
		if (err) {
			KMSGERR(&acceld->client->dev, "%s: request_threaded_irq returned err = %d\n", __func__, err);
			KMSGERR(&acceld->client->dev, "%s: running in software polling mode instead\n", __func__);
			acceld->accel_drdy = 0;
		}
		KMSGINF(&acceld->client->dev, "running in hardware interrupt mode\n");
	} else {
		KMSGINF(&acceld->client->dev, "running in software polling mode\n");
	}

	err = acceld->kionix_accel_power_on_init(acceld);
	if (err) {
		KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on_init returned err = %d. Abort.\n", __func__, err);
		goto err_free_irq;
	}
	#ifdef CONFIG_SENSORS
	err = sensors_classdev_register(&client->dev,
				&sensors_accelerometer_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto err_free_accel_registers;
	}
	#endif
	err = sysfs_create_group(&client->dev.kobj, &kionix_accel_attribute_group);
	if (err) {
		KMSGERR(&acceld->client->dev, "%s: sysfs_create_group returned err = %d. Abort.\n", __func__, err);
		goto err_free_irq;
	}

	//longjiang add start 20130620
	err = misc_register(&gsensor_misc);
	if(err < 0){
		printk(KERN_ERR "%s: can not register sensor misc device\n", __func__);
		err = -EINVAL;
		goto err_misc_register;
	}
	//add end

#if 0//def    CONFIG_HAS_EARLYSUSPEND
	/* The higher the level, the earlier it resume, and the later it suspend */
	acceld->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 50;
	acceld->early_suspend.suspend = kionix_accel_earlysuspend_suspend;
	acceld->early_suspend.resume = kionix_accel_earlysuspend_resume;
	register_early_suspend(&acceld->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	printk(KERN_INFO "%s: %s success.\n", KIONIX_ACCEL_NAME, __func__);//longjiang add 20130217

	return 0;

//longjiang add start 20130925
err_misc_register:
	sysfs_remove_group(&client->dev.kobj, &kionix_accel_attribute_group);
//add end
err_free_irq:
	if (acceld->accel_drdy)
		free_irq(client->irq, acceld);
	destroy_workqueue(acceld->accel_workqueue);
	input_unregister_device(acceld->input_dev);
err_free_accel_registers:
	kfree(acceld->accel_registers);
err_accel_pdata_exit:
	if (accel_pdata->exit)
		accel_pdata->exit();
err_accel_pdata_power_off:
	kionix_accel_power_off(acceld);
err_free_mem:
  //longjiang add start 20131017
  #ifdef CONFIG_SENSOR_POWER
  	sensors_power_on(kionix_drv, false);
  	sensors_power_init(kionix_drv, false);
  #endif
  //add end
	kfree(acceld);
	return err;
}

static int /*__devinit*/ kionix_accel_remove(struct i2c_client *client)
{
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

  #if 0//def    CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&acceld->early_suspend);
  #endif /* CONFIG_HAS_EARLYSUSPEND */
	sysfs_remove_group(&client->dev.kobj, &kionix_accel_attribute_group);
	if (acceld->accel_drdy)
		free_irq(client->irq, acceld);
	destroy_workqueue(acceld->accel_workqueue);
	input_unregister_device(acceld->input_dev);
	kfree(acceld->accel_registers);
	if (acceld->accel_pdata.exit)
		acceld->accel_pdata.exit();
	kionix_accel_power_off(acceld);
  //longjiang add start 20131017
  #ifdef CONFIG_SENSOR_POWER
  	sensors_power_on(kionix_drv, false);
	sensors_power_init(kionix_drv, false);
  #endif
  //add end
	kfree(acceld);

	return 0;
}

//longjiang add start 20130620
static int accel_status=0;
static int kionix_accel_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	//KMSGINF(&acceld->client->dev, "%s: enter\n", __func__);

		pr_err("%s: pre_enable = %d\n", __func__,
			atomic_read(&acceld->accel_enabled));
		#ifdef SENSORS_I2C_RETRY_ERR
		if (i2c_retry_err == 0) {
		#endif
			accel_status = atomic_read(&acceld->accel_enabled);
			if (accel_status > 0) {
				ret = kionix_accel_disable(acceld);
				if (ret < 0)
					KMSGERR(&acceld->client->dev,
					"%s: can't disable kionix\n", __func__);
			}
		if (acceld->accel_pdata.suspend)
			acceld->accel_pdata.suspend();
		atomic_set(&acceld->accel_suspended, 1);
		#ifdef SENSORS_I2C_RETRY_ERR
		}
		#endif
	return ret;
}
static int kionix_accel_resume(struct i2c_client *client)
{
	int ret = 0;
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	//KMSGINF(&acceld->client->dev, "%s: enter\n", __func__);
        printk(KERN_INFO "%s: pre_enable = %d\n", __func__, accel_status);
	#ifdef SENSORS_I2C_RETRY_ERR
	if (i2c_retry_err == 0) {
	#endif
		if (acceld->accel_pdata.resume)
			acceld->accel_pdata.resume();
			atomic_set(&acceld->accel_suspended, 0);
			if (accel_status > 0) {
				ret = kionix_accel_enable(acceld);
				accel_status = 0;
				if (ret < 0)
					KMSGERR(&acceld->client->dev,
					"%s: can't enable kionix\n", __func__);
			}
	#ifdef SENSORS_I2C_RETRY_ERR
		}
	#endif
	return ret;
}
//add end

static const struct i2c_device_id kionix_accel_id[] = {
	{ KIONIX_ACCEL_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kionix_accel_id);

//longjiang add start 20130620
#ifdef CONFIG_OF
static const struct of_device_id kionix_of_match[] = {
	{ .compatible = "kionix", },
	{ }
};
MODULE_DEVICE_TABLE(of, kionix_of_match);
#else
#define kionix_of_match NULL
#endif
//add end

static struct i2c_driver kionix_accel_driver = {
	.driver = {
		.name	= KIONIX_ACCEL_NAME,
		.owner	= THIS_MODULE,
	  //longjiang add start 20130620
 	  #ifdef CONFIG_OF
		.of_match_table = kionix_of_match,
	  #endif
	  //add end
	},
	.probe		= kionix_accel_probe,
	.remove		= kionix_accel_remove,//__devexit_p(kionix_accel_remove),
	//longjiang add start 20130620
	.suspend	= kionix_accel_suspend,
	.resume		= kionix_accel_resume,
	//ad end
	.id_table	= kionix_accel_id,
};

static int __init kionix_accel_init(void)
{
	return i2c_add_driver(&kionix_accel_driver);
}
module_init(kionix_accel_init);

static void __exit kionix_accel_exit(void)
{
	i2c_del_driver(&kionix_accel_driver);
}
module_exit(kionix_accel_exit);

MODULE_DESCRIPTION("Kionix accelerometer driver");
MODULE_AUTHOR("Kuching Tan <kuchingtan@kionix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.3.0");
