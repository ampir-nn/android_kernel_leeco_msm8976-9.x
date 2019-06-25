/*
 * Copyright (c) 2013 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "yas.h"


#define YAS_RANGE_2G                                                         (0)
#define YAS_RANGE_4G                                                         (1)
#define YAS_RANGE_8G                                                         (2)
#define YAS_RANGE_16G                                                        (3)
#define YAS_RANGE                                                   YAS_RANGE_2G
#if YAS_RANGE == YAS_RANGE_2G
#define YAS_RESOLUTION                                                    (1000)
#elif YAS_RANGE == YAS_RANGE_4G
#define YAS_RESOLUTION                                                     (500)
#elif YAS_RANGE == YAS_RANGE_8G
#define YAS_RESOLUTION                                                     (250)
#elif YAS_RANGE == YAS_RANGE_16G
#define YAS_RESOLUTION                                                      (83)
#else
#define YAS_RESOLUTION                                                    (1000)
#endif
#define YAS_GRAVITY_EARTH                                              (9806550)
/* AN3308 3 Startup sequence.  The boot procedure is completed after
 * approximately 5 milliseconds */
#define YAS_BOOT_TIME                                                     (5000)
#define YAS_DEFAULT_POSITION                                                 (0)
#define YAS_WHO_AM_I                                                      (0x0f)
#define YAS_WHO_AM_I_VAL                                                  (0x33)
#define YAS_CTRL_REG1                                                     (0x20)
#define YAS_CTRL_REG4                                                     (0x23)
#define YAS_CTRL_XYZ_ENABLE                                               (0x07)
#define YAS_CTRL_HR_ENABLE                                                (0x08)
#define YAS_CTRL_BDU_ENABLE                                               (0x80)
#define YAS_CTRL_RANGE_2G                                                 (0x00)
#define YAS_CTRL_RANGE_4G                                                 (0x10)
#define YAS_CTRL_RANGE_8G                                                 (0x20)
#define YAS_CTRL_RANGE_16G                                                (0x30)
#if YAS_RANGE == YAS_RANGE_2G
#define YAS_CTRL_RANGE                                         YAS_CTRL_RANGE_2G
#elif YAS_RANGE == YAS_RANGE_4G
#define YAS_CTRL_RANGE                                         YAS_CTRL_RANGE_4G
#elif YAS_RANGE == YAS_RANGE_8G
#define YAS_CTRL_RANGE                                         YAS_CTRL_RANGE_8G
#elif YAS_RANGE == YAS_RANGE_16G
#define YAS_CTRL_RANGE                                        YAS_CTRL_RANGE_16G
#else
#define YAS_CTRL_RANGE                                         YAS_CTRL_RANGE_2G
#endif
#define YAS_ODR_MASK                                                      (0xf0)
#define YAS_ODR_1250HZ                                                    (0x90)
#define YAS_ODR_400HZ                                                     (0x70)
#define YAS_ODR_200HZ                                                     (0x60)
#define YAS_ODR_100HZ                                                     (0x50)
#define YAS_ODR_50HZ                                                      (0x40)
#define YAS_ODR_25HZ                                                      (0x30)
#define YAS_ODR_10HZ                                                      (0x20)
#define YAS_ODR_1HZ                                                       (0x10)
#define YAS_OUT_X_L                                                       (0x28)

struct yas_odr {
	int delay;
	uint8_t odr;
};

struct yas_module {
	int initialized;
	int enable;
	int delay;
	int position;
	uint8_t odr;
	int turn_on_time;
	struct yas_driver_callback cbk;
};

static const struct yas_odr yas_odr_tbl[] = {
	{1,    YAS_ODR_1250HZ},
	{3,    YAS_ODR_400HZ},
	{5,    YAS_ODR_200HZ},
	{10,   YAS_ODR_100HZ},
/*
	{20,   YAS_ODR_50HZ},
	{40,   YAS_ODR_25HZ},
	{100,  YAS_ODR_10HZ},
	{1000, YAS_ODR_1HZ},
*/
};

static const int8_t yas_position_map[][3][3] = {
	{ {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} },/* top/upper-left */
	{ { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} },/* top/upper-right */
	{ { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} },/* top/lower-right */
	{ { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} },/* top/lower-left */
	{ { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} },/* bottom/upper-left */
	{ { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} },/* bottom/upper-right */
	{ {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} },/* bottom/lower-right */
	{ { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} },/* bottom/lower-right */
};

static struct yas_module module;

static int yas_read_reg(uint8_t adr, uint8_t *val);
static int yas_write_reg(uint8_t adr, uint8_t val);
static void yas_set_odr(int delay);
static int yas_power_up(void);
static int yas_power_down(void);
static int yas_init(void);
static int yas_term(void);
static int yas_get_delay(void);
static int yas_set_delay(int delay);
static int yas_get_enable(void);
static int yas_set_enable(int enable);
static int yas_get_position(void);
static int yas_set_position(int position);
static int yas_measure(struct yas_data *raw, int num);
static int yas_ext(int32_t cmd, void *result);

static int
yas_read_reg(uint8_t adr, uint8_t *val)
{
	return module.cbk.device_read(YAS_TYPE_ACC, adr, val, 1);
}

static int
yas_write_reg(uint8_t adr, uint8_t val)
{
	return module.cbk.device_write(YAS_TYPE_ACC, adr, &val, 1);
}

static void
yas_set_odr(int delay)
{
	int i;
	for (i = 1; i < NELEMS(yas_odr_tbl) &&
		     delay >= yas_odr_tbl[i].delay; i++)
		;
	module.odr = yas_odr_tbl[i-1].odr;
	/* AN3308 2.4 Switch mode timing.
	 *	Power-down -> Normal : 7 / ODR (ms).
	 * Worst timing is unknown, so we doubled here: 14 / ODR (ms)
	 */
	module.turn_on_time = yas_odr_tbl[i-1].delay * 14000;
}

static int
yas_power_up(void)
{
	if (yas_write_reg(YAS_CTRL_REG1, module.odr
			  | YAS_CTRL_XYZ_ENABLE) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_write_reg(YAS_CTRL_REG4, YAS_CTRL_RANGE
			  | YAS_CTRL_HR_ENABLE | YAS_CTRL_BDU_ENABLE) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	module.cbk.usleep(module.turn_on_time);
	return YAS_NO_ERROR;
}

static int
yas_power_down(void)
{
	if (yas_write_reg(YAS_CTRL_REG1, 0) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_write_reg(YAS_CTRL_REG4, 0) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	return YAS_NO_ERROR;
}

static int
yas_init(void)
{
	uint8_t id;
	if (module.initialized)
		return YAS_ERROR_INITIALIZE;
	module.cbk.usleep(YAS_BOOT_TIME);
	if (module.cbk.device_open(YAS_TYPE_ACC) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (yas_read_reg(YAS_WHO_AM_I, &id) < 0) {
		module.cbk.device_close(YAS_TYPE_ACC);
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (id != YAS_WHO_AM_I_VAL) {
		module.cbk.device_close(YAS_TYPE_ACC);
		return YAS_ERROR_CHIP_ID;
	}
	module.enable = 0;
	module.delay = YAS_DEFAULT_SENSOR_DELAY;
	module.position = YAS_DEFAULT_POSITION;
	yas_set_odr(module.delay);
	yas_power_down();
	module.cbk.device_close(YAS_TYPE_ACC);
	module.initialized = 1;
	return YAS_NO_ERROR;
}

static int
yas_term(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	yas_set_enable(0);
	module.initialized = 0;
	return YAS_NO_ERROR;
}

static int
yas_get_delay(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.delay;
}

static int
yas_set_delay(int delay)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (delay < 0)
		return YAS_ERROR_ARG;
	module.delay = delay;
	yas_set_odr(delay);
	if (!module.enable)
		return YAS_NO_ERROR;
	if (yas_write_reg(YAS_CTRL_REG1, module.odr
			  | YAS_CTRL_XYZ_ENABLE) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	module.cbk.usleep(module.turn_on_time);
	return YAS_NO_ERROR;
}

static int
yas_get_enable(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.enable;
}

static int
yas_set_enable(int enable)
{
	int rt;

	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (enable != 0)
		enable = 1;
	if (module.enable == enable)
		return YAS_NO_ERROR;
	if (enable) {
		module.cbk.usleep(YAS_BOOT_TIME);
		if (module.cbk.device_open(YAS_TYPE_ACC))
			return YAS_ERROR_DEVICE_COMMUNICATION;
		rt = yas_power_up();
		if (rt < 0) {
			module.cbk.device_close(YAS_TYPE_ACC);
			return rt;
		}
	} else {
		yas_power_down();
		module.cbk.device_close(YAS_TYPE_ACC);
	}
	module.enable = enable;
	return YAS_NO_ERROR;
}

static int
yas_get_position(void)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return module.position;
}

static int
yas_set_position(int position)
{
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (position < 0 || position > 7)
		return YAS_ERROR_ARG;
	module.position = position;
	return YAS_NO_ERROR;
}

static int
yas_measure(struct yas_data *raw, int num)
{
	uint8_t buf[6];
	int16_t dat[3];
	int i, j;

	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	if (raw == NULL || num < 0)
		return YAS_ERROR_ARG;
	if (num == 0 || module.enable == 0)
		return 0;
	if (module.cbk.device_read(YAS_TYPE_ACC
				   , YAS_OUT_X_L | 0x80
				   , buf
				   , 6) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	for (i = 0; i < 3; i++)
		dat[i] = (int16_t)(((int16_t)((buf[i*2+1] << 8))
				    | (buf[i*2] & 0xf0)) >> 4);
	for (i = 0; i < 3; i++) {
		raw->xyz.v[i] = 0;
		for (j = 0; j < 3; j++)
			raw->xyz.v[i] += dat[j] *
				yas_position_map[module.position][i][j];
		raw->xyz.v[i] *= (YAS_GRAVITY_EARTH / YAS_RESOLUTION);
	}
	raw->type = YAS_TYPE_ACC;
	if (module.cbk.current_time == NULL)
		raw->timestamp = 0;
	else
		raw->timestamp = module.cbk.current_time();
	raw->accuracy = 0;
	return 1;
}

static int
yas_ext(int32_t cmd, void *result)
{
	(void)cmd;
	(void)result;
	if (!module.initialized)
		return YAS_ERROR_INITIALIZE;
	return YAS_NO_ERROR;
}

int
yas_acc_driver_init(struct yas_acc_driver *f)
{
	if (f == NULL
	    || f->callback.device_open == NULL
	    || f->callback.device_close == NULL
	    || f->callback.device_write == NULL
	    || f->callback.device_read == NULL
	    || f->callback.usleep == NULL)
		return YAS_ERROR_ARG;
	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->measure = yas_measure;
	f->ext = yas_ext;
	module.cbk = f->callback;
	yas_term();
	return YAS_NO_ERROR;
}
