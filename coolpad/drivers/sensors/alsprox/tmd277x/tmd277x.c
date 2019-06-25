/***********************************************************************
 **
 **  Copyright (C), 2013-2015, Yulong Tech. Co., Ltd.
 **  FileName:		tmd277x.c
 **  Description:	Linux device driver for tmd277x ambient light and proximity sensors
 **  Author:		longjiang
 **  Version:		1.00
 **  Date:			2013-09-13
 **
 ***********************************************************************/

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/hardirq.h>
#include <linux/workqueue.h>

#include <linux/sensors/tmd277x.h>
#include <linux/sensors/sensparams.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_YL_SENSORS_DEBUG
	#define YL_DEBUG(fmt, args...) pr_info(fmt, ##args)
#else
	#define YL_DEBUG(fmt, args...)
#endif

#define TAOS_PROX_AVG_MAX         600
#define TAOS_PROX_AVG_MORE        100
#define TAOS_PROX_AVG_LESS        70
#define TAOS_PROX_CROSSTALK_MAX   500
#define TAOS_PROX_THRESHOLD_HI    330
#define TAOS_PROX_THRESHOLD_LO    300

#define TAOS_SUSPEND_TIMEOUT 5000 /* Timeout (miliseconds) */
#define TAOS_SUSPEND_SLEEP_EN  0 /* the suspend goto sleep mode enable switch              */
#define PROXIMITY_GPIO_INT 113 /*65*/

struct taos_dev {
	struct i2c_client *client;
	struct input_dev  *input_dev;
	struct work_struct work;
	wait_queue_head_t  wqh_suspend;
	struct wake_lock   wake_lock;
	struct wake_lock   wake_lock_timeout;
	unsigned int       als_ps_int;
	struct taos_platform_data *pdata;
	unsigned short     sat_als;
	unsigned short     sat_prox;
	int                prox_on;
	int                prox_enabled;
	int                als_enabled;
	int                suspend_flag;
	int                polling;
	unsigned int       phone_state;
	int                data_ready;
#if TAOS_SUSPEND_SLEEP_EN
	u8                 working; /* reg_cntrl;*/
#endif
	u8                 chip_id; /* add by Jay.HF, 2013-03-07 */
	u8                 prox_on_ok; /* add by Jay.HF, 2013-04-16 */

/* longjiang add start 20131017 */
	struct regulator *vcc_i2c;
	struct regulator *vdd_ana;

	struct pinctrl *taos_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};
static struct taos_dev *tmd277x_dev = NULL;

u8 taos_triton_reg_init[16] = {
	0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
	0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00
};
u8 taos_triton_gain_table[] = {1, 8, 16, 120};

struct time_scale_factor {
	u16 numerator;
	u16 denominator;
	u16 saturation;
};
struct time_scale_factor TritonTime = {1, 0, 0};
struct time_scale_factor *lux_timep = &TritonTime;

struct lux_data {
	u16 ratio;
	u16 clear;
	u16 ir;
};
struct lux_data TritonFN_lux_data[] = {
	{ 9830,  8320,  15360 },
	{ 12452, 10554, 22797 },
	{ 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
	{ 0,     0,     0     }
};
struct lux_data *lux_tablep = TritonFN_lux_data;

struct taos_prox_info prox_cal_info[20];

static int taos_pinctrl_init(struct taos_dev *taos_data)
{
	int retval;

    /* Get pinctrl if target uses pinctrl */
	taos_data->taos_pinctrl = devm_pinctrl_get(&(taos_data->client->dev));
	if (IS_ERR_OR_NULL(taos_data->taos_pinctrl)) {
		dev_dbg(&taos_data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(taos_data->taos_pinctrl);
		taos_data->taos_pinctrl = NULL;
		return retval;
	}

	taos_data->gpio_state_active
		= pinctrl_lookup_state(taos_data->taos_pinctrl,
			"pmx_tmd277x_active");
	if (IS_ERR_OR_NULL(taos_data->gpio_state_active)) {
		dev_dbg(&taos_data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(taos_data->gpio_state_active);
		taos_data->taos_pinctrl = NULL;
		return retval;
	}

	taos_data->gpio_state_suspend
		= pinctrl_lookup_state(taos_data->taos_pinctrl,
			"pmx_tmd277x_suspend");
	if (IS_ERR_OR_NULL(taos_data->gpio_state_suspend)) {
		dev_err(&taos_data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(taos_data->gpio_state_suspend);
		taos_data->taos_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int taos_pinctrl_select(struct taos_dev *taos_data,
								bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? taos_data->gpio_state_active
		: taos_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(taos_data->taos_pinctrl, pins_state);
		if (ret) {
			dev_err(&taos_data->client->dev,
			"can not set %s pins\n",
			on ? "pmx_tmd277x_active" : "pmx_tmd277x_suspend");
			return ret;
		}
	} else {
		dev_err(&taos_data->client->dev,
			"not a valid '%s' pinstate\n",
			on ? "pmx_tmd277x_active" : "pmx_tmd277x_suspend");
	}

	return 0;
}

/* longjiang add start 20130913 */
#ifdef CONFIG_OF
static int taos_parse_dt(struct device *dev, struct taos_platform_data *pdata)
{
	int rc;
	u32 temp_val;
	struct device_node *np = dev->of_node;

	rc = of_property_read_u32(np, "taos,calibrate_target", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read calibrate_target\n");
		return rc;
	} else
		pdata->calibrate_target = temp_val;

	rc = of_property_read_u32(np, "taos,als_time", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read als_time\n");
		return rc;
	} else
		pdata->als_time = temp_val;

	rc = of_property_read_u32(np, "taos,scale_factor", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read scale_factor\n");
		return rc;
	} else
		pdata->scale_factor = temp_val;

	rc = of_property_read_u32(np, "taos,gain_trim", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read gain_trim\n");
		return rc;
	} else
		pdata->gain_trim = temp_val;

	rc = of_property_read_u32(np, "taos,filter_history", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read filter_history\n");
		return rc;
	} else
		pdata->filter_history = temp_val;

	rc = of_property_read_u32(np, "taos,filter_count", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read filter_count\n");
		return rc;
	} else
		pdata->filter_count = temp_val;

	rc = of_property_read_u32(np, "taos,gain", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read gain\n");
		return rc;
	} else
		pdata->gain = temp_val;

	rc = of_property_read_u32(np, "taos,prox_threshold_hi", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_threshold_hi\n");
		return rc;
	} else
		pdata->prox_threshold_hi = temp_val;

	rc = of_property_read_u32(np, "taos,prox_threshold_lo", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_threshold_lo\n");
		return rc;
	} else
		pdata->prox_threshold_lo = temp_val;

	rc = of_property_read_u32(np, "taos,als_threshold_hi", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read als_threshold_hi\n");
		return rc;
	} else
		pdata->als_threshold_hi = temp_val;

	rc = of_property_read_u32(np, "taos,als_threshold_lo", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read als_threshold_lo\n");
		return rc;
	} else
		pdata->als_threshold_lo = temp_val;

	rc = of_property_read_u32(np, "taos,prox_int_time", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_int_time\n");
		return rc;
	} else
		pdata->prox_int_time = temp_val;

	rc = of_property_read_u32(np, "taos,prox_adc_time", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_adc_time\n");
		return rc;
	} else
		pdata->prox_adc_time = temp_val;

	rc = of_property_read_u32(np, "taos,prox_wait_time", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_wait_time\n");
		return rc;
	} else
		pdata->prox_wait_time = temp_val;

	rc = of_property_read_u32(np, "taos,prox_intr_filter", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_intr_filter\n");
		return rc;
	} else
		pdata->prox_intr_filter = temp_val;

	rc = of_property_read_u32(np, "taos,prox_config", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_config\n");
		return rc;
	} else
		pdata->prox_config = temp_val;

	rc = of_property_read_u32(np, "taos,prox_pulse_cnt", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_pulse_cnt\n");
		return rc;
	} else
		pdata->prox_pulse_cnt = temp_val;

	rc = of_property_read_u32(np, "taos,prox_gain", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_gain\n");
		return rc;
	} else
		pdata->prox_gain = temp_val;

	rc = of_property_read_u32(np, "taos,prox_offset", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_offset\n");
		return rc;
	} else
		pdata->prox_offset = temp_val;

	pdata->gpio_int = of_get_named_gpio_flags(np, "taos,gpio_int",
		0, &pdata->irq_gpio_flags);

	YL_DEBUG("%s: %d, %d, %d\n", __func__, pdata->calibrate_target, pdata->als_time, pdata->scale_factor);
	YL_DEBUG("%s: %d, %d, %d\n", __func__, pdata->gain_trim, pdata->filter_history, pdata->filter_count);
	YL_DEBUG("%s: %d, %d, %d\n", __func__, pdata->gain, pdata->prox_threshold_hi, pdata->prox_threshold_lo);
	YL_DEBUG("%s: %d, %d, %0x\n", __func__, pdata->als_threshold_hi, pdata->als_threshold_lo, pdata->prox_int_time);
	YL_DEBUG("%s: %0x, %0x, %0x\n", __func__, pdata->prox_adc_time, pdata->prox_wait_time, pdata->prox_intr_filter);
	YL_DEBUG("%s: %0x, %0x, %0x\n", __func__, pdata->prox_config, pdata->prox_pulse_cnt, pdata->prox_gain);
	YL_DEBUG("%s: %0x, %d, %d\n", __func__, pdata->prox_offset, pdata->gpio_int, pdata->irq_gpio_flags);

	return rc;
}
#endif
/* add end */

/* longjiang add start 20131017 */
static int sensors_power_on(struct taos_dev *data, bool on)
{
	int rc, rc1;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd_ana);
	if (rc) {
		dev_err(&data->client->dev,
		 "Regulator vdd_ana enable failed rc=%d\n", rc);
		return rc;
	}
	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd_ana);
	}
	return rc;

power_off:
	rc = regulator_disable(data->vdd_ana);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vdd_ana disable failed rc=%d\n", rc);
		return rc;
	}
	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
		"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc1 = regulator_enable(data->vdd_ana);
	}
	return rc;
}

static int sensors_power_init(struct taos_dev *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd_ana = regulator_get(&data->client->dev, "vdd_ana");
	if (IS_ERR(data->vdd_ana)) {
		rc = PTR_ERR(data->vdd_ana);
		dev_err(&data->client->dev,
		"Regulator get failed vdd_ana rc=%d\n", rc);
		return rc;
	}

    if (regulator_count_voltages(data->vdd_ana) > 0) {
        rc = regulator_set_voltage(data->vdd_ana, 2850000,2850000);
        if (rc) {
		dev_err(&data->client->dev,
			"Regulator set_vtg failed vdd_ana rc=%d\n", rc);
		goto reg_vdd_ana_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_ana_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, 1800000,1800000);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);

reg_vdd_ana_set_vtg:
	if (regulator_count_voltages(data->vdd_ana) > 0)
		regulator_set_voltage(data->vdd_ana, 0, 2850000);
reg_vdd_ana_put:
	regulator_put(data->vdd_ana);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd_ana) > 0)
		regulator_set_voltage(data->vdd_ana, 0, 2850000);

	regulator_put(data->vdd_ana);
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, 1800000);

	regulator_put(data->vcc_i2c);
	return 0;
}
/* add end */

int taos_power_on(void)
{
	int err = 0;

	if (NULL == tmd277x_dev || NULL == tmd277x_dev->pdata) {
	return 0;
	}

	if (tmd277x_dev->pdata->power_on) {
	err = tmd277x_dev->pdata->power_on();
	if (err < 0) {
	printk(KERN_ERR "%s: power_on failed: %d\n", __func__, err);
	return err;
	}
	if (tmd277x_dev->pdata->gpio_int >= 0)
		enable_irq(tmd277x_dev->als_ps_int);
	}

	return 0;
}

int taos_power_off(void)
{
	int err = 0;

	if (NULL == tmd277x_dev || NULL == tmd277x_dev->pdata) {
	return 0;
	}

	if (tmd277x_dev->pdata->power_off) {
	if (tmd277x_dev->pdata->gpio_int >= 0)
	disable_irq_nosync(tmd277x_dev->als_ps_int);
	err = tmd277x_dev->pdata->power_off();
	if (err < 0) {
		printk(KERN_ERR "%s: power_off failed: %d\n", __func__, err);
		return err;
	}
	}

	return 0;
}

static int taos_get_lux(void)
{
#if TAOS_ALGO_OPTIMIZE
	u8 dev_gain = 0;
	u16 Tint = 0;
#endif
	u8 chdata[4];
	u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
	u32 lux = 0, ratio = 0;
	int ret = 0, tmp = 0, i = 0;
	struct lux_data *p;

	for (i=0;i<4;i++) {
	if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))))) < 0) {
	printk(KERN_ERR"%s: i2c_write to ch0/1/lo/hi regs failed \n", __func__);
	return ret;
	}

	chdata[i] = i2c_smbus_read_byte(tmd277x_dev->client);
	}

	tmp = (tmd277x_dev->pdata->als_time+25)/50;
	TritonTime.denominator = tmp;

	tmp = 300*tmd277x_dev->pdata->als_time; /* tmp = 300*atime  400 */
	if (tmp > 65535)
	tmp = 65535;
	TritonTime.saturation = tmp;
	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir = chdata[3];
	raw_ir <<= 8;
	raw_ir |= chdata[2];

	if (raw_ir > raw_clear) {
	raw_lux = raw_ir;
	raw_ir = raw_clear;
	raw_clear = raw_lux;
	}

#if TAOS_ALGO_OPTIMIZE
	raw_clear *= (tmd277x_dev->pdata->scale_factor);
	raw_ir *= (tmd277x_dev->pdata->scale_factor);
	dev_gain = taos_triton_gain_table[tmd277x_dev->pdata->gain & 0x3];
#endif

	if (raw_clear >= lux_timep->saturation) {
	return TAOS_MAX_LUX;
	}

	if (raw_ir >= lux_timep->saturation) {
	return TAOS_MAX_LUX;
	}

	if (raw_clear == 0)
	return 0;

#if TAOS_ALGO_OPTIMIZE
	if (dev_gain == 0 || dev_gain > 127) {
	printk(KERN_ERR"%s: dev_gain = 0 or > 127 in taos_get_lux()\n", __func__);
	return -1;
	}
#endif

	if (lux_timep->denominator == 0) {
	printk(KERN_ERR"%s: lux_timep->denominator = 0 in taos_get_lux()\n", __func__);
	return -1;
	}

	ratio = (raw_ir << 15) / raw_clear;
	for (p = lux_tablep; p->ratio && p->ratio < ratio; p++);
	if (!p->ratio) {
		return 0;
	}

#if TAOS_ALGO_OPTIMIZE
	Tint = tmd277x_dev->pdata->als_time;
	raw_clear = ((raw_clear*400 + (dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
	raw_ir = ((raw_ir*400 +(dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
	lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
	lux = (lux + 32000)/64000;
#else
	lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)) + 8000)/16000;
#endif

	printk(KERN_INFO "%s: lux(%d)\n", __func__, lux);

	if (lux > TAOS_MAX_LUX) {
	lux = TAOS_MAX_LUX;
	return lux;
	}

	return lux;
}

static int taos_als_threshold_set(void)
{
	u8 chdata[2];
	u16 ch0, thresh_h, thresh_l;
	char buf[4];
	int i, count, ret = 0;
	for (i = 0; i < 2; i++) {
	chdata[i] = (i2c_smbus_read_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))));
	}

	ch0 = chdata[0] + ((u16)chdata[1]<<8);

#if 1
	thresh_h = (12 * ch0) / 10;
	if (thresh_h >= 65535)
	thresh_h = 65535;
	thresh_l = (8 * ch0) / 10;

#else
	if (ch0 < 25) {
		thresh_h = 93;
		thresh_l = 0;
	} else {
	thresh_h = (16 * ch0) / 10;
	if (thresh_h >= 65535)
		thresh_h = 65535;
	if (thresh_h < 93)
	thresh_h = 93;
	thresh_l = (6*ch0)/10;
	}
#endif

	buf[0] = thresh_l & 0x0ff;
	buf[1] = thresh_l >> 8;
	buf[2] = thresh_h & 0x0ff;
	buf[3] = thresh_h >> 8;

	for (count=0; count<4; count++) {
	ret = i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_MINTHRESHLO) + count, buf[count]);
	if (ret < 0) {
		printk(KERN_ERR"%s: ALS threshold set fail\n", __func__);
	return ret;
	}
	}

	return ret;
}

static int taos_als_default_threshold_set(void)
{
	char buf[4];
	int count, ret = 0;

	YL_DEBUG("%s: start\n", __func__);

	buf[0] = 0xfe;
	buf[1] = 0x0;
	buf[2] = 0xff;
	buf[3] = 0x0;

	for (count = 0; count < 4; count ++) {
	ret = i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_MINTHRESHLO) + count, buf[count]);
	if (ret < 0) {
		printk(KERN_ERR"%s: ALS set fail\n", __func__);
		return (ret);
	}
	}

    return ret;
}

static int taos_prox_threshold_set(void)
{
	u8 chdata[4];
	u16 proxdata = 0;
	u16 cleardata = 0;
	char pro_buf[4];
	int i, ret = 0;
	int distance = 0;

	chdata[0] = i2c_smbus_read_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_CHAN0LO));
	chdata[1] = i2c_smbus_read_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_CHAN0HI));
	chdata[2] = i2c_smbus_read_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_LO));
	chdata[3] = i2c_smbus_read_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_HI));
	cleardata = chdata[1];
	cleardata <<= 8;
	cleardata |= chdata[0];
	proxdata = chdata[3];
	proxdata <<= 8;
	proxdata |= chdata[2];

	if (proxdata < tmd277x_dev->pdata->prox_threshold_lo) {
		pro_buf[0] = 0x00; /* set the prox_threshold_lo to zero, */
		pro_buf[1] = 0x00; /* next time the proxdata must be bigger than the prox_threshold_lo,
                              this is avoid two anear appear. add by Jay.HF 2012-09-15 */
		pro_buf[2] = tmd277x_dev->pdata->prox_threshold_hi & 0x0ff;
		pro_buf[3] = tmd277x_dev->pdata->prox_threshold_hi >> 8;
		distance = 5; /* faraway */
	}
	else if(proxdata > tmd277x_dev->pdata->prox_threshold_hi) {
	if (cleardata > ((tmd277x_dev->sat_als * 80) / 100)) {
		printk(KERN_ERR"%s: ERROR, cleardata = [%6d], ((tmd277x_dev->sat_als * 80) / 100) = [%6d] \n",
			__func__, cleardata, ((tmd277x_dev->sat_als*80)/100));
		return -ENODATA;
	}

		pro_buf[0] = tmd277x_dev->pdata->prox_threshold_lo & 0x0ff;
		pro_buf[1] = tmd277x_dev->pdata->prox_threshold_lo >> 8;
		pro_buf[2] = 0xff; /* set the prox_threshold_hi to 0xFFFF, */
        pro_buf[3] = 0xff; /* next time the proxdata must be less than the prox_threshold_hi,
                              this is avoid two apart appear. add by Jay.HF 2012-09-15 */
        distance = 3; /* nearby */
    }

	if (distance) {
		input_event(tmd277x_dev->input_dev, EV_MSC, MSC_SCAN, distance);/*longjiang modify for fastmmi 20131104*/
		input_sync(tmd277x_dev->input_dev);
	}
    for (i = 0; i < 4; i++) {
        if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x08) + i, pro_buf[i]))) < 0) {
		printk(KERN_ERR"%s: i2c fail \n", __func__);
		return ret;
		}
	}

	tmd277x_dev->prox_on = 0;
	return ret;
}

static int taos_als_init_regs(void)
{
	int  ret = 0;
	u8 itime = 0, reg_val = 0, reg_cntrl = 0;

	if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|TAOS_TRITON_CMD_ALS_INTCLR)))) < 0) {
		printk(KERN_ERR"%s: i2c fail[1] \n", __func__);
	return (ret);
	}

	itime = (unsigned char)(((tmd277x_dev->pdata->als_time*32)/87) - 1);
	itime = (~itime);
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))) < 0) {
		printk(KERN_ERR"%s: i2c fail[2] \n", __func__);
		return (ret);
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT), tmd277x_dev->pdata->prox_intr_filter))) < 0) {
		printk(KERN_ERR"%s: i2c fail[3] \n", __func__);
		return (ret);
	}

	if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN)))) < 0) {
		printk(KERN_ERR"%s: i2c fail[4] \n", __func__);
		return ret;
	}

	reg_val = i2c_smbus_read_byte(tmd277x_dev->client);
	reg_val = reg_val & 0xFC;
	reg_val = reg_val | (tmd277x_dev->pdata->gain & 0x03);
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))) < 0) {
		printk(KERN_ERR"%s: i2c fail[5] \n", __func__);
		return ret;
	}

	reg_cntrl = (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ALS_INT_ENBL);
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		printk(KERN_ERR"%s: i2c fail[6] \n", __func__);
		return ret;
    }

	return ret;
}

static int taos_prox_init_regs(void)
{
	int ret = 0;
	unsigned char reg_cntrl = 0;

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x01), tmd277x_dev->pdata->prox_int_time))) < 0) {
		printk(KERN_ERR"%s: i2c fail[4] \n", __func__);
		return ret;
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x02), tmd277x_dev->pdata->prox_adc_time))) < 0) {
		printk(KERN_ERR"%s: i2c fail[5] \n", __func__);
		return ret;
    }

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x03), tmd277x_dev->pdata->prox_wait_time))) < 0) {
		printk(KERN_ERR"%s: i2c fail[6] \n", __func__);
		return ret;
    }

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0C), tmd277x_dev->pdata->prox_intr_filter))) < 0) {
		printk(KERN_ERR"%s: i2c fail[7] \n", __func__);
		return (ret);
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0D), tmd277x_dev->pdata->prox_config))) < 0) {
		printk(KERN_ERR"%s: i2c fail[8] \n", __func__);
		return (ret);
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0E), tmd277x_dev->pdata->prox_pulse_cnt))) < 0) {
		printk(KERN_ERR"%s: i2c fail[9] \n", __func__);
		return (ret);
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0F), tmd277x_dev->pdata->prox_gain))) < 0) {
	printk(KERN_ERR"%s: i2c fail[10] \n", __func__);
	return (ret);
	}

	if ((TMD2772_CHIP_ID == tmd277x_dev->chip_id) &&
		(ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x1E), tmd277x_dev->pdata->prox_offset))) < 0) {
		printk(KERN_ERR "%s: i2c failed 7 \n", __func__);
		return (ret);
	}

	reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_PROX_INT_ENBL |
		TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_WAIT_TMR_ENBL;
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		printk(KERN_ERR"%s: i2c fail[11] \n", __func__);
		return (ret);
	}
	return ret;
}


static int taos_calc_crosstalk(int count)
{
	int i = 0;
	int prox_sum = 0, prox_mean = 0;
	u8 chdata[2];
	msleep(10);
	for (i = 0; i < count; i++) {
	chdata[0] = i2c_smbus_read_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_LO));
	chdata[1] = i2c_smbus_read_byte_data(tmd277x_dev->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_HI));
	prox_cal_info[i].prox_data = chdata[1];
	prox_cal_info[i].prox_data <<= 8;
	prox_cal_info[i].prox_data |= chdata[0];
	prox_sum += prox_cal_info[i].prox_data;
	msleep(5);
	}

	prox_mean = prox_sum/count;
	YL_DEBUG("%s: prox_sum=%d, prox_mean=%d, count=%d\n", __func__, prox_sum, prox_mean, count);

	return prox_mean;
}

static int taos_cal_init_prox(void) {
	int ret = 0;
	u8 reg_cntrl = 0;
	if ((0 == tmd277x_dev->als_enabled) && (0 == tmd277x_dev->prox_enabled)) {
		if (taos_power_on()) {
			printk(KERN_ERR"%s: set power on fail \n", __func__);
		return -1;
		}
	}
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x01), tmd277x_dev->pdata->prox_int_time))) < 0) {
        printk(KERN_ERR "%s: i2c failed 1 \n", __func__);
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x02), tmd277x_dev->pdata->prox_adc_time))) < 0) {
        printk(KERN_ERR "%s: i2c failed 2 \n", __func__);
		return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x03), tmd277x_dev->pdata->prox_wait_time))) < 0) {
		printk(KERN_ERR "%s: i2c failed 3 \n", __func__);
		return ret;
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0D), tmd277x_dev->pdata->prox_config))) < 0) {
		printk(KERN_ERR "%s: i2c failed 4 \n", __func__);
		return ret;
	}

	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0E), tmd277x_dev->pdata->prox_pulse_cnt))) < 0) {
		printk(KERN_ERR "%s: i2c failed 5 \n", __func__);
		return ret;
	}

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x0F), tmd277x_dev->pdata->prox_gain))) < 0) {
		printk(KERN_ERR "%s: i2c failed 6 \n", __func__);
		return ret;
	}

    if ((TMD2772_CHIP_ID == tmd277x_dev->chip_id) &&
        (ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x1E), tmd277x_dev->pdata->prox_offset))) < 0) {
		printk(KERN_ERR "%s: i2c failed 7 \n", __func__);
		return ret;
    }

	reg_cntrl = (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		printk(KERN_ERR "%s: i2c failed 8 \n", __func__);
		return (ret);
	}

	msleep(50);

	return ret;
}

static int taos_cal_set_prox_threshold(int prox_avg)
{
	if ( prox_avg < TAOS_PROX_AVG_MAX) {
	tmd277x_dev->pdata->prox_threshold_hi = prox_avg + TAOS_PROX_AVG_MORE;
	tmd277x_dev->pdata->prox_threshold_lo = prox_avg + TAOS_PROX_AVG_LESS;
	printk(KERN_INFO "%s: thres_hi = [%d], thres_lo = [%d], prox_avg = [%d] \n", __func__,
	tmd277x_dev->pdata->prox_threshold_hi, tmd277x_dev->pdata->prox_threshold_lo, prox_avg);
	} else {
	tmd277x_dev->pdata->prox_threshold_hi = 0x3FF;
	tmd277x_dev->pdata->prox_threshold_lo = 0x00;
	printk(KERN_ERR "%s: Calibration fail, prox_avg = [%d] \n", __func__, prox_avg);
	return -1;
	}
	return 0;
}

static int taos_cal_recover_regs(void)
{
	int ret = 0;
	u8 count = 0;

	if (tmd277x_dev->prox_enabled) {
	tmd277x_dev->prox_on = 1;

	/* init regs */
	ret = taos_prox_init_regs();
	if (ret) {
		printk(KERN_ERR"%s: prox_init_regs fail \n", __func__);
		return ret;
	}

	/* set PROX threshold */
	ret = taos_prox_threshold_set();
	if (ret) {
		printk(KERN_ERR"%s: prox_threshold_set fail \n", __func__);
		return ret;
	}

	tmd277x_dev->prox_enabled = 1;
	}
	else if (tmd277x_dev->als_enabled) {
	taos_als_init_regs();
	tmd277x_dev->als_enabled = 1;
	}
	else {
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
			printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
			return ret;
        }

	tmd277x_dev->prox_enabled = 0;
	cancel_work_sync(&tmd277x_dev->work);
	tmd277x_dev->als_enabled = 0;
	if (tmd277x_dev->polling && (count < 50)) {
		YL_DEBUG("%s: %d ms to power down \n", __func__, count);
		count++;
		msleep(1);
	}
	if (taos_power_off()) {
		printk(KERN_ERR"%s: set power fail \n", __func__);
		ret = -1;
	}
	}
    return ret;
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id) {
	wake_lock(&tmd277x_dev->wake_lock); /* add by Jay.HF 2012-12-31 */
	YL_DEBUG("%s: irq handler\n", __func__);
	tmd277x_dev->data_ready = 1;
	schedule_work(&tmd277x_dev->work);
	return IRQ_HANDLED;
}

static int taos_als_get_data(void)
{
    u8 reg_val;
	int ret = 0;
	int lux_val = 0;
	if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
		printk(KERN_ERR"%s: i2c fail[1] \n", __func__);
		return (ret);
	}

	reg_val = i2c_smbus_read_byte(tmd277x_dev->client);
	if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) {
		printk(KERN_ERR"%s: ALS NOT enabled or power on !\n", __func__);
		return -1;
	}

	if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
		printk(KERN_ERR"%s: i2c fail[2] \n", __func__);
		return (ret);
    }

    reg_val = i2c_smbus_read_byte(tmd277x_dev->client);
    if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID) {
        printk(KERN_ERR"%s: ALS had NOT been integrated!\n", __func__);
        return -1;
    }

    if ((lux_val = taos_get_lux()) < 0) {
        printk(KERN_ERR"%s: returned error [%d] \n", __func__, lux_val);
        return -1;
    }

    input_report_abs(tmd277x_dev->input_dev, ABS_MISC, lux_val);
    input_sync(tmd277x_dev->input_dev);

    return ret;
}

static int taos_get_data(void)
{
	int status;
	int ret = 0;
	if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | 0x13)))) < 0) {
	printk(KERN_ERR"%s: get_data fail \n", __func__);
	return (ret);
	}

	status = i2c_smbus_read_byte(tmd277x_dev->client);

	if (tmd277x_dev->prox_enabled && (status & 0x20)) {
	ret = taos_prox_threshold_set();
	if (ret) {
		printk(KERN_ERR"%s: prox_threshold_set fail\n", __func__);
		return ret;
	}
	} else if (tmd277x_dev->als_enabled && (status & 0x10)) {
		ret = taos_als_threshold_set();
		if (ret) {
		printk(KERN_ERR"%s: als_threshold fail\n", __func__);
		return ret;
		}

	ret = taos_als_get_data();
	if (ret) {
	printk(KERN_ERR"%s: als_get_data fail\n", __func__);
	return ret;
	}
	}

	return ret;
}

static int taos_clear_interrupts(void)
{
	int ret = 0;

	ret = i2c_smbus_write_byte(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07));
	if (ret < 0) {
	printk(KERN_ERR"%s: taos_clear_interrupts fail\n", __func__);
	return (ret);
	}
	tmd277x_dev->data_ready = 0;

	return ret;
}

static void taos_work_func(struct work_struct * work)
{
	long remaining;


	remaining = wait_event_interruptible_timeout(tmd277x_dev->wqh_suspend, \
		tmd277x_dev->suspend_flag == 0, \
		msecs_to_jiffies(TAOS_SUSPEND_TIMEOUT));
	if (tmd277x_dev->suspend_flag == 1) {
	printk(KERN_ERR"%s: timeout waiting for resume\n", __func__);
	}
	if (!tmd277x_dev->suspend_flag && (tmd277x_dev->prox_enabled || tmd277x_dev->als_enabled) \
		&& (!tmd277x_dev->polling)) {
	tmd277x_dev->polling = 1;
	taos_get_data();
	tmd277x_dev->polling = 0;
	wake_lock_timeout(&tmd277x_dev->wake_lock_timeout, 2*HZ);
	}
	taos_clear_interrupts();
	wake_unlock(&tmd277x_dev->wake_lock);
}

static int taos_calc_offset(int value) {
	int ret = 0;
	int prox_avg = value;
	unsigned char offset = 0x1f;
	while ((prox_avg >= TAOS_PROX_CROSSTALK_MAX) && (offset <= 0x7f)) {
	YL_DEBUG("%s: offset = %d\n", __func__, offset);
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG|0x1E), offset))) < 0) {
	printk(KERN_ERR ":%s: i2c failed\n", __func__);
	return (ret);
	}
	msleep(5);
	prox_avg = taos_calc_crosstalk(10);
	if (prox_avg < TAOS_PROX_CROSSTALK_MAX) {
		break;
	}
	offset += 0x10;
	}
	if (offset > 0x7f) offset = 0x7f;
	tmd277x_dev->pdata->prox_offset = offset;
	return prox_avg;
}

static long taos_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int prox_avg = 0;
	u8 count = 0;
	u8 reg_tmp = 0;
	struct prox_offset taos_cal_data;
	struct prox_offset *taos_cal_ptr = &taos_cal_data;
	u8 prox_param[4];

	switch (cmd) {
	case TAOS_IOCTL_ALS_ON:
		YL_DEBUG("%s: ALS_ON Entry\n", __func__);
		tmd277x_dev->als_enabled = 1;
		if (0 == tmd277x_dev->prox_enabled) {
			YL_DEBUG("%s: ALS_ON taos_als_init_regs() \n", __func__);
			if (taos_power_on()) {
  				printk(KERN_ERR"%s: set power on fail \n", __func__);
				return -1;
			}
			ret = taos_als_init_regs();
			if (ret) {
				printk(KERN_ERR"%s: als_init_regs fail \n", __func__);
				return ret;
			}
			msleep(5);

			ret = taos_als_default_threshold_set();
			if (ret) {
				printk(KERN_ERR"%s: als_threshold_set fail \n", __func__);
			}
		}
		else if (tmd277x_dev->prox_enabled == 1) {
			reg_tmp = i2c_smbus_read_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | 0x01));
			// tmd277x_dev->prox_on = 1; /* resolv twice continuously move away, add by Jay.HF 2012-11-09 9:33 */

			if (reg_tmp != tmd277x_dev->pdata->prox_int_time) {
			printk(KERN_ERR"%s: ctl_reg1[0x%x] might be error, reset prox!\n", __func__, reg_tmp);

			/* init regs */
				ret = taos_prox_init_regs();
				if (ret) {
					printk(KERN_ERR"%s: prox_init_regs fail \n", __func__);
					return ret;
				}
				msleep(5);
				/* set prox threshold */
				ret = taos_prox_threshold_set();
				if (ret) {
			printk(KERN_ERR"%s: prox_threshold_set fail \n", __func__);
		return ret;
		}
		}
	}
	taos_clear_interrupts(); /* Clear ALS or PROX Interrupt Flag, By Jay.HF 2012-10-17 */
	YL_DEBUG("%s: ALS_ON Exit\n", __func__);
	break;

	case TAOS_IOCTL_ALS_OFF:
		YL_DEBUG("%s: ALS_OFF Entry\n", __func__);
		tmd277x_dev->als_enabled = 0;

		if (0 == tmd277x_dev->prox_enabled) {
			if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client,
				(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
				printk(KERN_ERR"%s: clear ALS regs\n", __func__);
				return ret;
			}
		cancel_work_sync(&tmd277x_dev->work);
		if (tmd277x_dev->polling && (count < 50)) {
		YL_DEBUG("%s: %d ms\n", __func__, count);
			count++;
			msleep(1);
		}
		if(taos_power_off()) {
			printk(KERN_ERR"%s: set_power fail \n", __func__);
			ret = -1;
		}
			tmd277x_dev->data_ready = 0;
		}
		YL_DEBUG("%s: ALS_OFF Exit\n", __func__);
		break;

	case TAOS_IOCTL_PROX_ON:
		YL_DEBUG("%s: PROX_ON Entry\n", __func__);
		tmd277x_dev->prox_enabled = 1;
 		tmd277x_dev->prox_on = 1;
		if (0 == tmd277x_dev->als_enabled) { /* als not open */
 			if (taos_power_on()) {
				printk(KERN_ERR"%s: set power on fail \n", __func__);
				return -1;
			}
		}
		if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client,
			(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
			printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
			return ret;
		}

		ret = taos_prox_init_regs();
		if (ret) {
			printk(KERN_ERR"%s: prox_init_regs failed\n", __func__);
			return ret;
		}
		ret = taos_calc_crosstalk(4);
		if (ret < 0) {
			printk(KERN_ERR"%s: taos_calc_crosstalk failed\n", __func__);
			return ret;
		}
		/*report the correct status for proximity sensor when power on*/
		ret = taos_prox_threshold_set();
		if (ret) {
			printk(KERN_ERR"%s: prox_threshold_set fail \n", __func__);
			return ret;
		}
		taos_clear_interrupts(); /* Clear ALS or PROX Interrupt Flag, By Jay.HF 2012-09-24 */
		if (tmd277x_dev->prox_on_ok == 0) {
			ret = irq_set_irq_wake(tmd277x_dev->als_ps_int, 1);
			if (ret != 0) {
			printk(KERN_ERR"%s: set irq wake source failed! \n", __func__);
			return(ret);
			}
			tmd277x_dev->prox_on_ok = 1;
		}
		YL_DEBUG("%s: PROX_ON Exit\n", __func__);
		break;
	case TAOS_IOCTL_PROX_OFF:
		YL_DEBUG("%s: PROX_OFF Entry\n", __func__);
		tmd277x_dev->prox_enabled = 0;
		tmd277x_dev->prox_on = 0;
		if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
			printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
			return ret;
		}
		if (tmd277x_dev->prox_on_ok == 1) {
			ret = irq_set_irq_wake(tmd277x_dev->als_ps_int, 0);
			if (ret != 0) {
				printk(KERN_ERR"%s: clear irq wake source failed ! \n", __func__);
				return ret;
			}
			tmd277x_dev->prox_on_ok = 0;
		}
		if (tmd277x_dev->als_enabled == 1) {
		taos_als_init_regs();
			msleep(5);
			ret = taos_als_default_threshold_set();
		if (ret) {
				printk(KERN_ERR"%s: als_threshold_set fail \n", __func__);
			}

		tmd277x_dev->als_enabled = 1;
		} else {
			cancel_work_sync(&tmd277x_dev->work);
			tmd277x_dev->als_enabled = 0;

			if (tmd277x_dev->polling && (count < 50)) {
			YL_DEBUG("%s: %d ms to power down \n", __func__, count);
			count++;
			msleep(1);
		}
		if (taos_power_off()) {
			printk(KERN_ERR"%s: set power fail \n", __func__);
			ret = -1;
		}
			tmd277x_dev->data_ready = 0;
		}
		YL_DEBUG("%s: PROX_OFF Exit\n", __func__);
		break;

	case TAOS_IOCTL_PROX_CALIBRATE:
		YL_DEBUG("%s: PROX_CALIBRATE\n", __func__);
		if ((struct prox_offset *)arg != NULL) {
		wake_lock(&tmd277x_dev->wake_lock);
		if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
			tmd277x_dev->pdata->prox_offset = 0x00; /* clear the offset register */
		}
		ret = taos_cal_init_prox();
		if (ret) {
		printk(KERN_ERR"%s: regs for caling failed \n", __func__);
		taos_cal_recover_regs();
		wake_unlock(&tmd277x_dev->wake_lock);
		return ret;
		}

		prox_avg = taos_calc_crosstalk(20);
		if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
		if (prox_avg >= TAOS_PROX_CROSSTALK_MAX) {
			prox_avg = taos_calc_offset(prox_avg);
			if (prox_avg < 0) {
				taos_cal_recover_regs();
				wake_unlock(&tmd277x_dev->wake_lock);
				return prox_avg;
			}
			}
		}
		taos_cal_set_prox_threshold(prox_avg);
		memset(prox_param, 0, sizeof(prox_param));
		prox_param[0] = TAOS_FLASH_MAGIC;
		if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
			prox_param[1] = tmd277x_dev->pdata->prox_offset;
			prox_param[2] = tmd277x_dev->pdata->prox_threshold_hi & 0x00ff;
			prox_param[3] = (tmd277x_dev->pdata->prox_threshold_hi & 0xff00) >> 8;
		}else {
			prox_param[1] = tmd277x_dev->pdata->prox_threshold_hi & 0x00ff;
			prox_param[2] = (tmd277x_dev->pdata->prox_threshold_hi & 0xff00) >> 8;
		}
		sensparams_write_to_flash(SENSPARAMS_TYPE_PROX, prox_param, 4);

		YL_DEBUG("%s: prox_hig = [%5d]\n", __func__,
			tmd277x_dev->pdata->prox_threshold_hi);
		YL_DEBUG("%s: prox_low = [%5d]\n", __func__,
			tmd277x_dev->pdata->prox_threshold_lo);
		YL_DEBUG("%s: prox_avg = [%5d]\n", __func__, prox_avg);
		YL_DEBUG("%s: prox_offset = [%5d]\n", __func__,
			tmd277x_dev->pdata->prox_offset);

		((struct prox_offset *)taos_cal_ptr)->x = (unsigned short)(tmd277x_dev->pdata->prox_threshold_hi);
		((struct prox_offset *)taos_cal_ptr)->y = (unsigned short)(tmd277x_dev->pdata->prox_threshold_lo);
		((struct prox_offset *)taos_cal_ptr)->z = (unsigned short)(prox_avg);
		((struct prox_offset *)taos_cal_ptr)->key = prox_avg < 600 ? 1 : 2;
		((struct prox_offset *)taos_cal_ptr)->offset
			= (unsigned short)(tmd277x_dev->pdata->prox_offset);

		if (copy_to_user((struct prox_offset *)arg, taos_cal_ptr, sizeof(taos_cal_data))) {
			printk(KERN_ERR"%s: data trans error,use default offset !\n",
				__func__);
		}

			taos_cal_recover_regs();
			wake_unlock(&tmd277x_dev->wake_lock);
		} else {
			printk(KERN_ERR"%s: (%d) null pointer !\n", __func__, __LINE__);
			return -1;
		}
		break;

	case TAOS_IOCTL_PROX_OFFSET:
		YL_DEBUG("%s: PROX_OFFSET\n", __func__);
		sensparams_read_from_flash(SENSPARAMS_TYPE_PROX, prox_param, 4);
	if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
		tmd277x_dev->pdata->prox_offset = prox_param[1];
		tmd277x_dev->pdata->prox_threshold_hi = (prox_param[3] << 8) | prox_param[2];
		tmd277x_dev->pdata->prox_threshold_lo = tmd277x_dev->pdata->prox_threshold_hi - \
		(TAOS_PROX_AVG_MORE - TAOS_PROX_AVG_LESS);
	} else {
		tmd277x_dev->pdata->prox_threshold_hi = (prox_param[2] << 8) | prox_param[1];
		tmd277x_dev->pdata->prox_threshold_lo = tmd277x_dev->pdata->prox_threshold_hi - \
		(TAOS_PROX_AVG_MORE - TAOS_PROX_AVG_LESS);
	}
	if ((tmd277x_dev->pdata->prox_threshold_lo == 0) ||
		(tmd277x_dev->pdata->prox_threshold_hi == 0)) {
		tmd277x_dev->pdata->prox_threshold_lo = TAOS_PROX_THRESHOLD_LO;
		tmd277x_dev->pdata->prox_threshold_hi = TAOS_PROX_THRESHOLD_HI;
	}

	printk(KERN_INFO "%s: prox_hig = [%5d] \n", __func__, tmd277x_dev->pdata->prox_threshold_hi);
	printk(KERN_INFO "%s: prox_low = [%5d] \n", __func__, tmd277x_dev->pdata->prox_threshold_lo);
	break;

	default:
	printk(KERN_ERR"%s: DEFAULT!\n", __func__);
	ret = -1;
	break;
	}

	return ret;
}

static int taos_open(struct inode *inode, struct file *file)
{
	YL_DEBUG("%s: start\n", __func__);
	return 0;
}

static int taos_release(struct inode *inode, struct file *file)
{
	YL_DEBUG("%s: start\n", __func__);
	return 0;
}

static struct file_operations tmd277x_fops = {
	.owner          = THIS_MODULE,
	.open           = taos_open,
	.release        = taos_release,
	.unlocked_ioctl = taos_ioctl,
};

static struct miscdevice tmd277x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = TAOS_DEVICE_NAME,
	.fops  = &tmd277x_fops,
};

static int taos_suspend(struct i2c_client *client, pm_message_t mesg)
{
	 int ret = 0;
#if TAOS_SUSPEND_SLEEP_EN
	u8 reg_val = 0;
	u8 reg_cntrl = 0;
#endif

	YL_DEBUG("%s: start\n", __func__);
	if (tmd277x_dev->pdata->suspend) {
	tmd277x_dev->pdata->suspend();
	}
	taos_clear_interrupts(); /* Clear ALS or PROX Interrupt Flag, By Jay.HF 2012-12-27 */
#if TAOS_SUSPEND_SLEEP_EN
	if (!(tmd277x_dev->prox_enabled)) {
	printk(KERN_INFO"%s: tmd277x power off begain\n", __func__);
        /* Jay.HF add for low-power-mode 20121108 */
	if ((ret = (i2c_smbus_write_byte(client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
		printk(KERN_ERR"%s: i2c_smbus_write_byte failed 1\n", __func__);
		return 0;
	}
	reg_val = i2c_smbus_read_byte(client);
	if (reg_val & TAOS_TRITON_CNTL_PWRON) {
	reg_cntrl = reg_val & (~TAOS_TRITON_CNTL_PWRON);
	if ((ret = (i2c_smbus_write_byte_data(client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		printk(KERN_ERR"%s: i2c_smbus_write_byte_data failed 2\n", __func__);
		return 0;
	}
	tmd277x_dev->working = 1;
	cancel_work_sync(&tmd277x_dev->work);
	printk(KERN_INFO"%s: tmd277x power off success\n", __func__);
	}
	}
#endif
	tmd277x_dev->suspend_flag = 1;
	return ret;
}

static int taos_resume(struct i2c_client *client)
{
	int ret = 0;
#if TAOS_SUSPEND_SLEEP_EN
	u8 reg_val = 0;
	u8 reg_cntrl = 0;
#endif

	tmd277x_dev->suspend_flag = 0;
	YL_DEBUG("%s: start\n", __func__);

#if TAOS_SUSPEND_SLEEP_EN
	if (!(tmd277x_dev->prox_enabled)) {
	/* Jay.HF add for low-power-mode 20121108 */
	if ((ret = (i2c_smbus_write_byte(client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
		printk(KERN_ERR"%s: i2c_smbus_write_byte failed 1\n", __func__);
		return 0;
	}
	reg_val = i2c_smbus_read_byte(client);
	if (tmd277x_dev->working == 1) {
		reg_cntrl = reg_val | TAOS_TRITON_CNTL_PWRON;
	if ((ret = (i2c_smbus_write_byte_data(client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		printk(KERN_ERR"%s: i2c_smbus_write_byte failed 2\n", __func__);
		return 0;
	}
	tmd277x_dev->working = 0;
	printk(KERN_INFO"%s: tmd277x power on success\n", __func__);
	}
	}
#endif
	if ((tmd277x_dev->prox_enabled || tmd277x_dev->als_enabled) && tmd277x_dev->data_ready) {
	YL_DEBUG("%s: prox_enabled(%d), als_enabled(%d), data_ready(%d)\n",
	__func__, tmd277x_dev->prox_enabled, tmd277x_dev->als_enabled, tmd277x_dev->data_ready);

	wake_lock_timeout(&tmd277x_dev->wake_lock_timeout, 2*HZ);
	taos_get_data();
	}
	taos_clear_interrupts(); /* add by Jay.HF 2012-09-24 */
	if (tmd277x_dev->pdata->resume) {
	tmd277x_dev->pdata->resume();
	}
	wake_up_interruptible(&tmd277x_dev->wqh_suspend);
	return ret;
}

static int taos_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int chip_id = 0;
	struct taos_platform_data *pdata = NULL;
	int err;

	printk(KERN_INFO "%s: start\n", __func__);

	if (!client) {
		printk(KERN_ERR "%s: i2c client null pointer! \n", __func__);
		ret = -EINVAL;
		goto out;
	}

    /* check i2c bus */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA)) {
	printk(KERN_ERR "%s: i2c check failed \n", __func__);
	ret = -EIO;
	goto out;
	}


/* longjiang add start 20130913 */
  #ifdef CONFIG_OF
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct taos_platform_data), GFP_KERNEL);
		if (NULL == pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = taos_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Get pdata failed from Device Tree\n");
			return ret;
		}
	} else {
		pdata = client->dev.platform_data;
		if (NULL == pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	}
  #else
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL\n");
		return -ENOMEM;
	} else {
	pdata = client->dev.platform_data;
		if (NULL == pdata) {
			dev_err(&client->dev, "pdata is NULL\n");
			return -ENOMEM;
		}
	}
  #endif
	/* sunbingtong 2014.5.29 this gpio request later */
	/* gpio_request(PROXIMITY_GPIO_INT, "proximity-tmd277x"); */

	tmd277x_dev = kzalloc(sizeof(struct taos_dev), GFP_KERNEL);
	if (NULL == tmd277x_dev) {
		printk(KERN_ERR "%s: allocate tmd277x dev fail \n", __func__);
		ret = -ENOMEM;
		goto out;
	}
	memset(tmd277x_dev, 0, sizeof(struct taos_dev));
	tmd277x_dev->client  = client;
	tmd277x_dev->pdata   = pdata;

  /* longjiang add start 20131017*/
    sensors_power_init(tmd277x_dev, true);
    sensors_power_on(tmd277x_dev, true);
    msleep(500);

	chip_id = i2c_smbus_read_byte_data(client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CHIPID));
	printk(KERN_INFO "%s: chip_id = %0x\n", __func__, chip_id);
	if ((chip_id != TMD2771_CHIP_ID) && (chip_id != TMD2772_CHIP_ID)) {
		printk(KERN_ERR"%s: error chip_id = 0x%x\n", __func__, chip_id);
		ret = -EIO;
		goto out_free_dev;
	}
	tmd277x_dev->chip_id = chip_id;

	if (tmd277x_dev->pdata->init) {
		ret = tmd277x_dev->pdata->init();
		if (ret < 0) {
			pr_err("%s: init failed: %d\n", __func__, ret);
			goto out_free_dev;
		}
	}

	err = taos_pinctrl_init(tmd277x_dev);
	if (!err && tmd277x_dev->taos_pinctrl) {
		err = taos_pinctrl_select(tmd277x_dev, true);
		if (err < 0)
			goto out_power_off;
	}

	if (tmd277x_dev->pdata->gpio_int >= 0) {
		/* sunbingtong 2014.5.29 move gpio request to this place*/
		gpio_request(tmd277x_dev->pdata->gpio_int, "proximity-tmd277x");
		tmd277x_dev->als_ps_int =
			 gpio_to_irq(tmd277x_dev->pdata->gpio_int);
	}

	tmd277x_dev->input_dev = input_allocate_device();
	if (tmd277x_dev->input_dev == NULL) {
		printk(KERN_ERR"%s: allocate input device fail !\n", __func__);
		ret = -1;
		goto out_power_off;
	}
	tmd277x_dev->input_dev->name = TAOS_INPUT_NAME;
	tmd277x_dev->input_dev->id.bustype = BUS_I2C;
	set_bit(EV_MSC, tmd277x_dev->input_dev->evbit);
	input_set_capability(tmd277x_dev->input_dev, EV_MSC, MSC_SCAN);/*longjiang modify for fastmmi 20131104*/
	input_set_capability(tmd277x_dev->input_dev, EV_ABS, ABS_MISC);
    /* light Lux data */
	input_set_abs_params(tmd277x_dev->input_dev, ABS_MISC, 0, 100000, 0, 0);
    /* proximity data */
//	input_set_abs_params(tmd277x_dev->input_dev, ABS_DISTANCE, 0, 1000, 0, 0);/*longjiang delete for fastmmi 20131104*/
	ret = input_register_device(tmd277x_dev->input_dev);
	if (ret != 0) {
		printk(KERN_ERR"%s: input_register_device failed ! \n", __func__);
		goto out_free_input;
	}

	wake_lock_init(&tmd277x_dev->wake_lock, WAKE_LOCK_SUSPEND, "taos_wake_lock");
	wake_lock_init(&tmd277x_dev->wake_lock_timeout, WAKE_LOCK_SUSPEND, "taos_wake_lock_timeout");
	INIT_WORK(&(tmd277x_dev->work), taos_work_func);
	init_waitqueue_head(&tmd277x_dev->wqh_suspend);
	tmd277x_dev->prox_on      = 0;
	tmd277x_dev->prox_enabled = 0;
	tmd277x_dev->prox_on_ok = 0;
	tmd277x_dev->als_enabled  = 0;
	tmd277x_dev->suspend_flag = 0;
	tmd277x_dev->polling      = 0;
	tmd277x_dev->sat_als      = (256-tmd277x_dev->pdata->prox_int_time)<<10;
	tmd277x_dev->sat_prox     = (256-tmd277x_dev->pdata->prox_adc_time)<<10;
	if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, \
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
	printk(KERN_ERR"%s: i2c_write failed in power down\n", __func__);
	goto out_unregister_input;
	}

	YL_DEBUG("%s: request irq [%d] \n", __func__, tmd277x_dev->als_ps_int);

	ret = request_irq(tmd277x_dev->als_ps_int, taos_irq_handler, IRQF_TRIGGER_FALLING, "taos_irq", NULL);
	if (ret != 0) {
		printk(KERN_ERR"%s: request_irq fail, releasing irq... \n", __func__);
		goto out_unregister_input;
    }

	ret = misc_register(&tmd277x_device);
	if (ret) {
		printk(KERN_ERR"%s: tmd277x_device register failed \n", __FUNCTION__);
		goto out_free_irq;
	}
	ret = taos_power_off();
	if (0 != ret) {
	printk(KERN_ERR"%s: taos_power_off() fail \n", __func__);
	goto out_deregister_misc;
	}

	printk(KERN_INFO "%s: probe success\n", __func__);

	return 0;

out_deregister_misc:
	misc_deregister(&tmd277x_device);
out_free_irq:
	free_irq(tmd277x_dev->als_ps_int, NULL);
out_unregister_input:
	input_unregister_device(tmd277x_dev->input_dev);
out_free_input:
	input_free_device(tmd277x_dev->input_dev);
out_power_off:
	if (tmd277x_dev->pdata->power_off) {
	tmd277x_dev->pdata->power_off();
	}
out_free_dev:
	devm_kfree(&client->dev, pdata);
	kfree(tmd277x_dev);
out:
	return ret;
}

static int taos_remove(struct i2c_client *client)
{
	int ret = 0;
	misc_deregister(&tmd277x_device);
	disable_irq_nosync(tmd277x_dev->als_ps_int);
	free_irq(tmd277x_dev->als_ps_int, NULL);
	input_unregister_device(tmd277x_dev->input_dev);
	input_free_device(tmd277x_dev->input_dev);
	if (tmd277x_dev->pdata->power_off) {
		tmd277x_dev->pdata->power_off();
	}
	if (tmd277x_dev->pdata->gpio_int >= 0) {
		disable_irq_nosync(tmd277x_dev->als_ps_int);
		free_irq(tmd277x_dev->als_ps_int, NULL);
	}
	if (tmd277x_dev->pdata->exit) {
	tmd277x_dev->pdata->exit();
	}
	kfree(tmd277x_dev->pdata);
	kfree(tmd277x_dev);
	return ret;
}

static const struct i2c_device_id taos_id[] = {
	{ TAOS_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, taos_id);

#ifdef CONFIG_OF
static struct of_device_id taos_match_table[] = {
	{ .compatible = "tmd277x",},
	{ },
};
#else
#define taos_match_table NULL
#endif

static struct i2c_driver tmd277x_driver = {
	.driver = {
	.name  = TAOS_DRIVER_NAME,
	.owner = THIS_MODULE,
      /* longjiang add start 20131017 */
	#ifdef CONFIG_OF
	.of_match_table = taos_match_table,
	#endif
	},
	/* longjiang add start 20130913 */
	.suspend   = taos_suspend,
	.resume    = taos_resume,
	.probe     = taos_probe,
	.remove    = taos_remove,
	.id_table  = taos_id,
};

static int __init taos_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&tmd277x_driver);
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c_add_driver() failed\n", __func__);
		return ret;
	}

	return ret;
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&tmd277x_driver);
}

MODULE_AUTHOR("longjiang@yulong.com");
MODULE_DESCRIPTION("TAOS Ambient Light and Proximity Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

module_init(taos_init);
module_exit(taos_exit);

