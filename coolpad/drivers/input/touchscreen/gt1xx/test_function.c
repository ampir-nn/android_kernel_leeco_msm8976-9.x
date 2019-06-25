/* drivers/input/touchscreen/test_function.c
 *
 * 2010 - 2014 Shenzhen Huiding Technology Co.,Ltd.
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
 * Version: 1.0
 * Release Date: 2014/11/14
 *
 */

#ifdef __cplusplus
//extern "C" {
#endif

#include <linux/string.h>

#ifdef __cplusplus
#include "tool.h"
#endif

#include "test_function.h"

#if !GTP_TEST_PARAMS_FROM_INI
#include "test_params.h"
#endif
/*----------------------------------- test param variable define-----------------------------------*/
/* sample data frame numbers */
int samping_set_number = 16;

/* raw data max threshold */
int *max_limit_value;
int max_limit_value_tmp = 2285;

/* raw data min threshold */
int *min_limit_value;
int min_limit_value_tmp = 978;

/* ratio threshold between adjacent(up down right left) data */
long *accord_limit;
long accord_limit_tmp = 200;
long *accord_line_limit;

/* maximum deviation ratio,|(data - average)|/average */
long offset_limit = 150;

/* uniformity = minimum / maximum */
long uniformity_limit = 0;

/* further judgement for not meet the entire screen offset limit of the channel,
   when the following conditions are satisfied that the legitimate:
   1 beyond the entire screen offset limit but not beyond the limits ,
   2 to meet the condition of 1 point number no more than
   3 3 of these two non adjacent */
long special_limit = 458;

/* maximum jitter the entire screen data value */
int permit_jitter_limit = 30;

/* key raw data max threshold */
int *max_key_limit_value;
int max_key_limit_value_tmp = 0;

/* key raw data min threshold */
int *min_key_limit_value;
int min_key_limit_value_tmp = 0;

int ini_module_type = 1;

unsigned char *ini_version1;

unsigned char *ini_version2;

unsigned short gt900_short_threshold = 10;
unsigned short gt900_short_dbl_threshold = 500;
unsigned short gt900_drv_drv_resistor_threshold = 500;;
unsigned short gt900_drv_sen_resistor_threshold = 500;
unsigned short gt900_sen_sen_resistor_threshold = 500;
unsigned short gt900_resistor_warn_threshold = 1000;
unsigned short gt900_drv_gnd_resistor_threshold = 400;
unsigned short gt900_sen_gnd_resistor_threshold = 400;
unsigned char gt900_short_test_times = 1;

long tri_pattern_r_ratio = 115;
int sys_avdd = 28;

/*----------------------------------- test input variable define-----------------------------------*/
unsigned short *current_data_temp;

unsigned short *channel_max_value;

unsigned short *channel_min_value;

/* average value of channel */
int *channel_average;

/* average current value of each channel of the square */
int *channel_square_average;

/* the test group number */
int current_data_index;

unsigned char *need_check;

unsigned char *channel_key_need_check;

/* max sample data number */
int samping_num = 64;

u8 *global_large_buf;

unsigned char *driver_status;
unsigned char *upload_short_data;

/*----------------------------------- test output variable define-----------------------------------*/
int test_error_code;

/* channel status,0 is normal,otherwise is anormaly */
unsigned short *channel_status;

long *channel_max_accord;

long *channel_max_line_accord;

/* maximum number exceeds the set value */
unsigned char *beyond_max_limit_num;

/*  minimum number exceeds the set value */
unsigned char *beyond_min_limit_num;

/* deviation ratio exceeds the set value. */
unsigned char *beyond_accord_limit_num;

/* full screen data maximum deviation ratio exceeds the set value */
unsigned char *beyond_offset_limit_num;

/* maximum frequency jitter full screen data exceeds a set value */
unsigned char *beyond_jitter_limit_num;

/* the number of data consistency over the setting value */
unsigned char beyond_uniformity_limit_num;

/*----------------------------------- other define-----------------------------------*/
char save_result_dir[250] = "/sdcard/rawdata/";
char ini_find_dir1[250] = "/sdcard/";
char ini_find_dir2[250] = "/data/data/com.goodix.rawdata/files/";
char ini_format[250] = "";
char save_data_path[250];

u8 *original_cfg;
u8 *module_cfg;

s32 _node_in_key_chn(u16 node, u8 * config, u16 keyoffest)
{
	int ret = -1;
	u8 i, tmp, chn;

	if (node < sys.sc_sensor_num * sys.sc_driver_num) {
		return -1;
	}

	chn = node - sys.sc_sensor_num * sys.sc_driver_num;
	for (i = 0; i < 4; i++) {
		tmp = config[keyoffest + i];
		if ((tmp != 0) && (tmp % 8 != 0)) {
			return 1;
		}

		if (tmp == (chn + 1) * 8) {
			ret = 1;
		}
	}

	return ret;
}

static void _get_channel_min_value(void)
{
	int i;

	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (current_data_temp[i] < channel_min_value[i]) {
			channel_min_value[i] = current_data_temp[i];
		}
	}
}

static void _get_channel_max_value(void)
{
	int i;

	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (current_data_temp[i] > channel_max_value[i]) {
			channel_max_value[i] = current_data_temp[i];
		}
	}
}

static void _get_channel_average_value(void)
{
	int i;
	if (current_data_index == 0) {
		memset(channel_average, 0, sizeof(channel_average) / sizeof(channel_average[0]));
		memset(channel_average, 0, sizeof(channel_square_average) / sizeof(channel_square_average[0]));
	}

	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		channel_average[i] += current_data_temp[i] / samping_set_number;
		channel_square_average[i] += (current_data_temp[i] * current_data_temp[i]) / samping_set_number;
	}
}

static unsigned short _get_average_value(unsigned short *data)
{
	int i;
	int temp = 0;
	int not_check_num = 0;
	unsigned short average_temp = 0;

	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (need_check[i] == _NEED_NOT_CHECK) {
			not_check_num++;
			continue;
		}
		temp += data[i];
	}

	DEBUG("NOT CHECK NUM:%d\ntmp:%d\n", not_check_num, temp);
	if (not_check_num < sys.sc_sensor_num * sys.sc_driver_num) {
		average_temp = (unsigned short)(temp / (sys.sc_sensor_num * sys.sc_driver_num - not_check_num));
	}
	return average_temp;
}

#ifdef SIX_SIGMA_JITTER
static float _get_six_sigma_value(void)
{
	int i;
	float square_sigma = 0;
	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		square_sigma += channel_square_average[i] - (channel_average[i] * channel_average[i]);
	}
	square_sigma /= samping_set_number;
	return sqrt(square_sigma);
}
#endif /*
        */
static unsigned char _check_channel_min_value(void)
{
	int i;
	unsigned char test_result = 1;

	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (need_check[i] == _NEED_NOT_CHECK) {
			continue;
		}

		if (current_data_temp[i] < min_limit_value[i]) {
			channel_status[i] |= _BEYOND_MIN_LIMIT;
			test_error_code |= _BEYOND_MIN_LIMIT;
			beyond_min_limit_num[i]++;
			test_result = 0;
			//DEBUG("current[%d]%d,limit[%d]%d",i,current_data_temp[i],i,min_limit_value[i]);
		}
	}

	return test_result;
}

static unsigned char _check_channel_max_value(void)
{
	int i;
	unsigned char test_result = 1;
	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (need_check[i] == _NEED_NOT_CHECK) {
			continue;
		}

		if (current_data_temp[i] > max_limit_value[i]) {
			channel_status[i] |= _BEYOND_MAX_LIMIT;
			test_error_code |= _BEYOND_MAX_LIMIT;
			beyond_max_limit_num[i]++;
			test_result = 0;
		}
	}

	return test_result;
}

static unsigned char _check_key_min_value(void)
{
	int i, j;
	unsigned char test_result = 1;
	if (sys.key_number == 0) {
		return test_result;
	}

	for (i = sys.sc_sensor_num * sys.sc_driver_num, j = 0; i < sys.sc_sensor_num * sys.sc_driver_num + sys.key_number; i++) {
		if (channel_key_need_check[i - sys.sc_sensor_num * sys.sc_driver_num] == _NEED_NOT_CHECK) {
			continue;
		}

		if (_node_in_key_chn(i, module_cfg, sys.key_offest) < 0) {
			continue;
		}

		if (current_data_temp[i] < min_key_limit_value[j++]) {
			channel_status[i] |= _KEY_BEYOND_MIN_LIMIT;
			test_error_code |= _KEY_BEYOND_MIN_LIMIT;
			beyond_min_limit_num[i]++;
			test_result = 0;
		}
	}

	return test_result;
}

static unsigned char _check_key_max_value(void)
{
	int i, j;
	unsigned char test_result = 1;

	if (sys.key_number == 0) {
		return test_result;
	}

	for (i = sys.sc_sensor_num * sys.sc_driver_num, j = 0; i < sys.sc_sensor_num * sys.sc_driver_num + sys.key_number; i++) {
		if (channel_key_need_check[i - sys.sc_sensor_num * sys.sc_driver_num] == _NEED_NOT_CHECK) {
			continue;
		}

		if (_node_in_key_chn(i, module_cfg, sys.key_offest) < 0) {
			continue;
		}

		if (current_data_temp[i] > max_key_limit_value[j++]) {
			channel_status[i] |= _KEY_BEYOND_MAX_LIMIT;
			test_error_code |= _KEY_BEYOND_MAX_LIMIT;
			beyond_max_limit_num[i]++;
			test_result = 0;
		}
	}

	return test_result;
}

static unsigned char _check_area_accord(void)
{
	int i, j, index;
	long temp;
	long accord_temp;
	unsigned char test_result = 1;

	for (i = 0; i < sys.sc_sensor_num; i++) {
		for (j = 0; j < sys.sc_driver_num; j++) {
			index = i + j * sys.sc_sensor_num;

			accord_temp = 0;
			temp = 0;
			if (need_check[index] == _NEED_NOT_CHECK) {
				continue;
			}

			if (current_data_temp[index] == 0) {
				current_data_temp[index] = 1;
				continue;
			}

			if (j == 0) {
				if (need_check[i + (j + 1) * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					accord_temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + (j + 1) * sys.sc_sensor_num] - current_data_temp[index]))) /
					    current_data_temp[index];
				}
			} else if (j == sys.sc_driver_num - 1) {
				if (need_check[i + (j - 1) * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					accord_temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + (j - 1) * sys.sc_sensor_num] - current_data_temp[index]))) /
					    current_data_temp[index];
				}
			} else {
				if (need_check[i + (j + 1) * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					accord_temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + (j + 1) * sys.sc_sensor_num] - current_data_temp[index]))) /
					    current_data_temp[index];
				}
				if (need_check[i + (j - 1) * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + (j - 1) * sys.sc_sensor_num] - current_data_temp[index]))) /
					    current_data_temp[index];
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
			}

			if (i == 0) {
				if (need_check[i + 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + 1 + j * sys.sc_sensor_num] - current_data_temp[index]))) / current_data_temp[index];
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
			} else if (i == sys.sc_sensor_num - 1) {
				if (need_check[i - 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i - 1 + j * sys.sc_sensor_num] - current_data_temp[index]))) / current_data_temp[index];
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
			} else {
				if (need_check[i + 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + 1 + j * sys.sc_sensor_num] - current_data_temp[index]))) / current_data_temp[index];
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
				if (need_check[i - 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) {
					temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i - 1 + j * sys.sc_sensor_num] - current_data_temp[index]))) / current_data_temp[index];
				}
				if (temp > accord_temp) {
					accord_temp = temp;
				}
			}

			channel_max_accord[index] = accord_temp;

			if (accord_temp > accord_limit[index]) {
				channel_status[index] |= _BEYOND_ACCORD_LIMIT;
				test_error_code |= _BEYOND_ACCORD_LIMIT;
				test_result = 0;
				beyond_accord_limit_num[index]++;
			}
		}
	}

	return test_result;
}

static unsigned char _check_area_accord_1143(void)
{
	int i, j, index, sign = 1;
	long accord_temp;
	unsigned char test_result = 1;

	DEBUG("%s", __func__);
	for (i = 0; i < sys.sc_sensor_num; i++) {
		for (j = 0; j < sys.sc_driver_num; j++) {
			index = i + j * sys.sc_sensor_num;

			accord_temp = 0;
//                      DEBUG("need_check[%d]%d.",index,need_check[index]);
			if (need_check[index] == _NEED_NOT_CHECK) {
				continue;
			}

			if (current_data_temp[index] == 0) {
//                              current_data_temp[index] = 1;
				continue;
			}

			if (j % 2 == 0) {
				sign = 1;	//right
			} else {
				sign = -1;	//left
			}

			if (i == 0) {
				if ((need_check[i + 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) && (need_check[i + (j + sign) * sys.sc_sensor_num] != _NEED_NOT_CHECK)
				    && (need_check[i + 1 + (j + sign) * sys.sc_sensor_num] != _NEED_NOT_CHECK)) {
					accord_temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + 1 + j * sys.sc_sensor_num] - current_data_temp[index])) +
					     FLOAT_AMPLIFIER *
					     abs((s16) (current_data_temp[i + 1 + (j + sign) * sys.sc_sensor_num] - current_data_temp[i + (j + sign) * sys.sc_sensor_num]))) /
					    current_data_temp[index];

//                                      DEBUG("buf[%d]=%d,buf[%d]=%d,buf[%d]=%d,buf[%d]=%d",i + 1 + j * sys.sc_sensor_num,current_data_temp[i + 1 + j * sys.sc_sensor_num],index,current_data_temp[index],
//                                                      i + 1 + (j + sign) * sys.sc_sensor_num,current_data_temp[i + 1 + (j + sign) * sys.sc_sensor_num],i + (j + sign) * sys.sc_sensor_num,current_data_temp[i + (j + sign) * sys.sc_sensor_num]);
				}
			} else if (i == sys.sc_sensor_num - 1) {
				if ((need_check[i - 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) && (need_check[i + (j + sign) * sys.sc_sensor_num] != _NEED_NOT_CHECK)
				    && (need_check[i - 1 + (j + sign) * sys.sc_sensor_num] != _NEED_NOT_CHECK)) {
					accord_temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i - 1 + j * sys.sc_sensor_num] - current_data_temp[index])) +
					     FLOAT_AMPLIFIER *
					     abs((s16) (current_data_temp[i - 1 + (j + sign) * sys.sc_sensor_num] - current_data_temp[i + (j + sign) * sys.sc_sensor_num]))) /
					    current_data_temp[index];
				}
			} else {
				if ((need_check[i + 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK) && (need_check[i - 1 + j * sys.sc_sensor_num] != _NEED_NOT_CHECK)) {
					accord_temp =
					    (FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i + 1 + j * sys.sc_sensor_num] - current_data_temp[index])) +
					     FLOAT_AMPLIFIER * abs((s16) (current_data_temp[i - 1 + j * sys.sc_sensor_num] - current_data_temp[index]))) / current_data_temp[index];
				}
			}

			channel_max_line_accord[index] += accord_temp;

			channel_max_accord[index] = accord_temp;

		}
	}

	return test_result;
}

static unsigned char _check_full_screen_offest(unsigned char special_check)
{
	int average_temp = 0;
	int i, j;
	long offset_temp;
	int special_num = 0;
	int special_channel[_SPECIAL_LIMIT_CHANNEL_NUM];
	unsigned char test_result = 1;

	/* calculate the average value of total screen */
	average_temp = _get_average_value(current_data_temp);
	DEBUG("average:%d\n", average_temp);

	/* caculate the offset between the channel value and the average value */
	for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (need_check[i] == _NEED_NOT_CHECK) {
			continue;
		}
		/* get the max ratio of the total screen,(current_chn_value - screen_average_value)/screen_average_value */
		offset_temp = abs((current_data_temp[i] - average_temp) * FLOAT_AMPLIFIER) / average_temp;

//              /* if area accord test is pass,and then do not do the screen accord test */
//              if ((channel_status[i] & _BEYOND_ACCORD_LIMIT) != _BEYOND_ACCORD_LIMIT) {
//                      continue;
//              }
		/* current channel accord validity detection */
		if (offset_temp > offset_limit) {
			if ((special_check == _SPECIAL_CHECK) && (special_num < _SPECIAL_LIMIT_CHANNEL_NUM) && (offset_temp <= special_limit)) {
				if ((current_data_index == 0 || special_num == 0)) {
					special_channel[special_num] = i;
					special_num++;
				} else {
					for (j = 0; j < special_num; j++) {
						if (special_channel[j] == i) {
							break;
						}
					}
					if (j == special_num) {
						special_channel[special_num] = i;
						special_num++;
					}
				}
			} else {
				channel_status[i] |= _BEYOND_OFFSET_LIMIT;
				test_error_code |= _BEYOND_OFFSET_LIMIT;
				beyond_offset_limit_num[i]++;
				test_result = 0;
			}
		}		/* end of if (offset_temp > offset_limit) */
	}			/* end of for (i = 0; i < sys.sensor_num*sys.driver_num; i++) */
	if (special_check && test_result == 1) {
		for (i = special_num - 1; i > 0; i--) {
			for (j = i - 1; j >= 0; j--) {
				if ((special_channel[i] - special_channel[j] == 1)
				    || (special_channel[i] - special_channel[j] == sys.sc_driver_num)) {
					channel_status[special_channel[j]] |= _BEYOND_OFFSET_LIMIT;
					test_error_code |= _BEYOND_OFFSET_LIMIT;
					beyond_offset_limit_num[special_channel[j]]++;
					test_result = 0;
				}
			}
		}
	}			/* end of if (special_check && test_result == TRUE) */
	return test_result;
}

static unsigned char _check_full_screen_jitter(void)
{
	int j;
	unsigned short max_jitter = 0;
	unsigned char test_result = 1;
	unsigned short *shake_value;
#ifdef SIX_SIGMA_JITTER
	double six_sigma = 0;
#endif
	shake_value = (u16 *) (&global_large_buf[0]);

	for (j = 0; j < sys.sc_sensor_num * sys.sc_driver_num; j++) {
		if (need_check[j] == _NEED_NOT_CHECK) {
			continue;
		}
		shake_value[j] = channel_max_value[j] - channel_min_value[j];

		if (shake_value[j] > max_jitter) {
			max_jitter = shake_value[j];
		}
	}

#ifdef SIX_SIGMA_JITTER
	six_sigma = 6 * _get_six_sigma_value();
	/* if 6sigama>jitter_limit or max_jitter>jitter_limit+10，jitter is not legal */
	if ((six_sigma > permit_jitter_limit) || (max_jitter > permit_jitter_limit + 10))
#endif
	{
		for (j = 0; j < sys.sc_sensor_num * sys.sc_driver_num; j++) {
			if (shake_value[j] >= permit_jitter_limit) {
				channel_status[j] |= _BEYOND_JITTER_LIMIT;
				test_error_code |= _BEYOND_JITTER_LIMIT;
				test_result = 0;
				DEBUG("point %d beyond jitter limit", j);
			}
		}
	}
	return test_result;
}

static unsigned char _check_uniformity(void)
{
	u16 i = 0;
	u16 min_val = 0, max_val = 0;
	long uniformity = 0;
	unsigned char test_result = 1;

	min_val = current_data_temp[0];
	max_val = current_data_temp[0];
	for (i = 1; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
		if (need_check[i] == _NEED_NOT_CHECK) {
			continue;
		}
		if (current_data_temp[i] > max_val) {
			max_val = current_data_temp[i];
		}
		if (current_data_temp[i] < min_val) {
			min_val = current_data_temp[i];
		}
	}

	if (0 == max_val) {
		uniformity = 0;
	} else {
		uniformity = (min_val * FLOAT_AMPLIFIER) / max_val;
	}
	DEBUG("min_val: %d, max_val: %d, tp uniformity(x1000): %lx", min_val, max_val, uniformity);
	if (uniformity < uniformity_limit) {
		beyond_uniformity_limit_num++;
		//channel_status[i] |= _BEYOND_UNIFORMITY_LIMIT;
		test_error_code |= _BEYOND_UNIFORMITY_LIMIT;
		test_result = 0;
	}
	return test_result;
}

static s32 _check_modele_type(int type)
{
	int ic_id = read_sensorid();

	if (ic_id != read_sensorid()) {
		ic_id = read_sensorid();
		if (ic_id != read_sensorid()) {
			WARNING("Read many ID inconsistent");
			return -1;
		}
	}

	if ((ic_id | ini_module_type) < 0) {
		return ic_id < ini_module_type ? ic_id : ini_module_type;
	}

	if (ic_id != ini_module_type) {
		test_error_code |= _MODULE_TYPE_ERR;
	}

	return 0;
}

static s32 _check_device_version(int type)
{
	s32 ret = 0;

	if (type & _VER_EQU_CHECK) {
		DEBUG("ini version:%s\n", ini_version1);
		ret = check_version(ini_version1);
		if (ret != 0) {
			test_error_code |= _VERSION_ERR;
		}
	} else if (type & _VER_GREATER_CHECK) {
		DEBUG("ini version:%s\n", ini_version1);
		ret = check_version(ini_version1);
		if (ret == 1 || ret < 0) {
			test_error_code |= _VERSION_ERR;
		}
	} else if (type & _VER_BETWEEN_CHECK) {
		signed int ret1 = 0;
		signed int ret2 = 0;

		DEBUG("ini version1:%s\n", ini_version1);
		DEBUG("ini version2:%s\n", ini_version2);
		ret1 = check_version(ini_version1);
		ret2 = check_version(ini_version2);
		if (ret1 == 1 || ret2 == 2 || ret1 < 0 || ret2 < 0) {
			test_error_code |= _VERSION_ERR;
		}
	}

	return 0;
}

static unsigned char _rawdata_test_result_analysis(int check_types)
{
	int i;
	int accord_temp = 0;
	int temp;
	int error_code_temp = test_error_code;
	int err = 0;
	int test_end = 0;

	// screen max value check
	error_code_temp &= ~_BEYOND_MAX_LIMIT;
	if (((check_types & _MAX_CHECK) != 0) && ((test_error_code & _BEYOND_MAX_LIMIT) != 0)) {
		for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
			if ((channel_status[i] & _BEYOND_MAX_LIMIT) == _BEYOND_MAX_LIMIT) {
				if (beyond_max_limit_num[i] >= (samping_set_number * 9 / 10)) {
					error_code_temp |= _BEYOND_MAX_LIMIT;
					test_end |= _BEYOND_MAX_LIMIT;
				} else if (beyond_max_limit_num[i] > samping_set_number / 10) {
					error_code_temp |= _BEYOND_MAX_LIMIT;
					err |= _BEYOND_MAX_LIMIT;
				} else {
					channel_status[i] &= ~_BEYOND_MAX_LIMIT;
				}
			}
		}
	}
	/* touch key max value check */
	error_code_temp &= ~_KEY_BEYOND_MAX_LIMIT;
	if (((check_types & _KEY_MAX_CHECK) != 0) && ((test_error_code & _KEY_BEYOND_MAX_LIMIT) != 0)) {
		for (i = sys.sc_sensor_num * sys.sc_driver_num; i < sys.sc_sensor_num * sys.sc_driver_num + sys.key_number; i++) {
			if ((channel_status[i] & _KEY_BEYOND_MAX_LIMIT) == _KEY_BEYOND_MAX_LIMIT) {
				if (beyond_max_limit_num[i] >= (samping_set_number * 9 / 10)) {
					error_code_temp |= _KEY_BEYOND_MAX_LIMIT;
					test_end |= _KEY_BEYOND_MAX_LIMIT;
				} else if (beyond_max_limit_num[i] > samping_set_number / 10) {
					error_code_temp |= _KEY_BEYOND_MAX_LIMIT;
					err |= _KEY_BEYOND_MAX_LIMIT;
				} else {
					channel_status[i] &= ~_KEY_BEYOND_MAX_LIMIT;
				}
			}
		}
	}
	/* screen min value check */
	error_code_temp &= ~_BEYOND_MIN_LIMIT;
	if (((check_types & _MIN_CHECK) != 0) && ((test_error_code & _BEYOND_MIN_LIMIT) != 0)) {
		for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
			if ((channel_status[i] & _BEYOND_MIN_LIMIT) == _BEYOND_MIN_LIMIT) {
				if (beyond_min_limit_num[i] >= (samping_set_number * 9 / 10)) {
					error_code_temp |= _BEYOND_MIN_LIMIT;
					test_end |= _BEYOND_MIN_LIMIT;
				} else if (beyond_min_limit_num[i] > samping_set_number / 10) {
					error_code_temp |= _BEYOND_MIN_LIMIT;
					err |= _BEYOND_MIN_LIMIT;
				} else {
					channel_status[i] &= ~_BEYOND_MIN_LIMIT;
				}
			}
		}
	}
	/* touch key min value check */
	error_code_temp &= ~_KEY_BEYOND_MIN_LIMIT;
	if (((check_types & _KEY_MIN_CHECK) != 0) && ((test_error_code & _KEY_BEYOND_MIN_LIMIT) != 0)) {
		for (i = sys.sc_sensor_num * sys.sc_driver_num; i < sys.sc_sensor_num * sys.sc_driver_num + sys.key_number; i++) {
			if ((channel_status[i] & _KEY_BEYOND_MIN_LIMIT) == _KEY_BEYOND_MIN_LIMIT) {
				if (beyond_min_limit_num[i] >= (samping_set_number * 9 / 10)) {
					error_code_temp |= _KEY_BEYOND_MIN_LIMIT;
					test_end |= _KEY_BEYOND_MIN_LIMIT;
				} else if (beyond_min_limit_num[i] > samping_set_number / 10) {
					error_code_temp |= _KEY_BEYOND_MIN_LIMIT;
					err |= _KEY_BEYOND_MIN_LIMIT;
				} else {
					channel_status[i] &= ~_KEY_BEYOND_MIN_LIMIT;
				}
			}
		}
	}
	/* screen uniformity check */
	error_code_temp &= ~_BEYOND_UNIFORMITY_LIMIT;
	if (((check_types & _UNIFORMITY_CHECK) != 0) && ((test_error_code & _BEYOND_UNIFORMITY_LIMIT) != 0)) {
		DEBUG("beyond_uniformity_limit_num:%d", beyond_uniformity_limit_num);
		if (beyond_uniformity_limit_num >= (samping_set_number * 9 / 10)) {
			error_code_temp |= _BEYOND_UNIFORMITY_LIMIT;
			test_end |= _BEYOND_UNIFORMITY_LIMIT;
		} else if (beyond_uniformity_limit_num > samping_set_number / 10) {
			error_code_temp |= _BEYOND_UNIFORMITY_LIMIT;
			err |= _BEYOND_UNIFORMITY_LIMIT;
			DEBUG("beyond_uniformity_limit_num:%d", beyond_uniformity_limit_num);
		}
	}
	/* adjacent data accord check */
	error_code_temp &= ~_BEYOND_ACCORD_LIMIT;
	if (((check_types & _ACCORD_CHECK) != 0) && ((test_error_code & _BEYOND_ACCORD_LIMIT) != 0)) {
		DEBUG("analysis accord ");
		for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
			if ((channel_status[i] & _BEYOND_ACCORD_LIMIT) == _BEYOND_ACCORD_LIMIT) {
				if (beyond_accord_limit_num[i] >= (samping_set_number * 9 / 10)) {
					error_code_temp |= _BEYOND_ACCORD_LIMIT;
					test_end |= _BEYOND_ACCORD_LIMIT;
					accord_temp++;
				} else if (beyond_accord_limit_num[i] > samping_set_number / 10) {
					error_code_temp |= _BEYOND_ACCORD_LIMIT;
					err |= _BEYOND_ACCORD_LIMIT;
					accord_temp++;
				} else {
					channel_status[i] &= ~_BEYOND_ACCORD_LIMIT;
				}
			}
		}
	}
	/* screen max accord check */
	error_code_temp &= ~_BEYOND_OFFSET_LIMIT;
	if (((check_types & _OFFSET_CHECK) != 0) && ((test_error_code & _BEYOND_OFFSET_LIMIT) != 0)) {
		for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
			if ((channel_status[i] & _BEYOND_OFFSET_LIMIT) == _BEYOND_OFFSET_LIMIT) {
				if (beyond_offset_limit_num[i] >= (samping_set_number * 9 / 10)) {
					error_code_temp |= _BEYOND_OFFSET_LIMIT;
					test_end |= _BEYOND_OFFSET_LIMIT;
				} else if (beyond_offset_limit_num[i] > samping_set_number / 10) {
					error_code_temp |= _BEYOND_OFFSET_LIMIT;
					err |= _BEYOND_OFFSET_LIMIT;
				} else {
					channel_status[i] &= ~_BEYOND_OFFSET_LIMIT;
				}
			}
		}
	}

	if (1) {		/* (sys.AccordOrOffsetNG == FALSE) */
		if (((check_types & _ACCORD_CHECK) != 0) && ((check_types & _OFFSET_CHECK) != 0)) {
			for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
				if (((channel_status[i] & _BEYOND_OFFSET_LIMIT) != _BEYOND_OFFSET_LIMIT)
				    && ((channel_status[i] & _BEYOND_ACCORD_LIMIT) == _BEYOND_ACCORD_LIMIT)) {
					channel_status[i] &= ~_BEYOND_ACCORD_LIMIT;
					accord_temp--;
				}
			}
			if (accord_temp == 0) {
				error_code_temp &= ~_BEYOND_ACCORD_LIMIT;
				test_end &= ~_BEYOND_ACCORD_LIMIT;
				err &= ~_BEYOND_ACCORD_LIMIT;
			}
		}
	}

	error_code_temp |= (test_error_code & _BEYOND_JITTER_LIMIT);

	test_error_code = error_code_temp;
	DEBUG("test_end:0x%0x err:0x%0x", test_end, err);
	if (test_end != _CHANNEL_PASS) {
		return 1;
	}

	if (err != _CHANNEL_PASS) {
		if ((check_types & _FAST_TEST_MODE) != 0) {
			if (samping_set_number < samping_num) {
				temp = samping_set_number;
				samping_set_number += (samping_num / 4);
				for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
					channel_average[i] = channel_average[i] * temp / samping_set_number;
					channel_square_average[i] += channel_square_average[i] * temp / samping_set_number;
				}
				return 0;
			}
		}
	}

	return 1;
}

static unsigned char _accord_test_result_analysis(int check_types)
{
	int i, error, error_tmp;
	long accord_tmp;

	if (sys.chip_type != _GT1143 && sys.chip_type != _GT1133) {
		return 1;
	}
	//1143
	test_error_code &= ~_BEYOND_ACCORD_LIMIT;
	error = error_tmp = _CHANNEL_PASS;
	if ((check_types & _ACCORD_CHECK) != 0) {
		for (i = 0; i < sys.sc_sensor_num * sys.sc_driver_num; i++) {
			accord_tmp = channel_max_line_accord[i] / current_data_index;
//                      DEBUG("accord_line_limit[%d]=%ld",i,accord_line_limit[i]);
			if (accord_tmp > accord_line_limit[i])	//NG
			{
				error |= _BEYOND_ACCORD_LIMIT;
			} else if (accord_tmp > accord_limit[i] && accord_tmp <= accord_line_limit[i])	//
			{
				error_tmp |= _BETWEEN_ACCORD_AND_LINE;
			}
		}
	}

	if (error) {
		test_error_code |= error;
	} else {
		test_error_code |= error_tmp;
	}
	DEBUG("1143 accord test_error_code 0x%x", test_error_code);
	return 1;
}

#if GTP_SAVE_TEST_DATA
static s32 _save_testing_data(char *save_test_data_dir, int test_types)
{
	FILE *fp = NULL;
	s32 ret;
	s32 tmp = 0;
	u8 *data = NULL;
	s32 i = 0, j;
	s32 bytes = 0;
	int max, min;
	int average;

	DEBUG("_save_testing_data");
	data = (u8 *) malloc(MAX_BUFFER_SIZE);
	if (NULL == data) {
		WARNING("memory error!");
		return MEMORY_ERR;
	}

	fp = fopen((const char *)save_test_data_dir, "a+");
	if (NULL == fp) {
		WARNING("open %s failed!", save_test_data_dir);
		free(data);
		return FILE_OPEN_CREATE_ERR;
	}

	if (current_data_index == 0) {
		bytes = (s32) sprintf((char *)data, "Config:\n");
		for (i = 0; i < sys.config_length; i++) {
			bytes += (s32) sprintf((char *)&data[bytes], "0x%02X,", module_cfg[i]);
		}
		bytes += (s32) sprintf((char *)&data[bytes], "\n\n");
		ret = fwrite(data, bytes, 1, fp);
		bytes = 0;
		if (ret < 0) {
			WARNING("write to file fail.");
			goto exit_save_testing_data;
		}

		if ((test_types & _MAX_CHECK) != 0) {
			bytes = (s32) sprintf((char *)data, "Channel maximum:\n");
			for (i = 0; i < sys.sc_sensor_num; i++) {
				for (j = 0; j < sys.sc_driver_num; j++) {
					bytes += (s32) sprintf((char *)&data[bytes], "%d,", max_limit_value[i + j * sys.sc_sensor_num]);
				}
				bytes += (s32) sprintf((char *)&data[bytes], "\n");
				ret = fwrite(data, bytes, 1, fp);
				bytes = 0;
				if (ret < 0) {
					WARNING("write to file fail.");
					goto exit_save_testing_data;
				}
			}

			j = 0;
			bytes = 0;
			for (i = 0; i < sys.key_number; i++) {
				if (_node_in_key_chn(i + sys.sc_sensor_num * sys.sc_driver_num, module_cfg, sys.key_offest) < 0) {
					continue;
				}
				bytes += (s32) sprintf((char *)&data[bytes], "%d,", max_key_limit_value[j++]);
				/* DEBUG("max[%d]%d",i,max_key_limit_value[i]); */
			}
			bytes += (s32) sprintf((char *)&data[bytes], "\n");
			ret = fwrite(data, bytes, 1, fp);
			bytes = 0;
			if (ret < 0) {
				WARNING("write to file fail.");
				goto exit_save_testing_data;
			}
		}

		if ((test_types & _MIN_CHECK) != 0) {
			bytes = (s32) sprintf((char *)data, "Channel minimum:\n");
			for (i = 0; i < sys.sc_sensor_num; i++) {
				for (j = 0; j < sys.sc_driver_num; j++) {
					bytes += (s32) sprintf((char *)&data[bytes], "%d,", min_limit_value[i + j * sys.sc_sensor_num]);
				}
				bytes += (s32) sprintf((char *)&data[bytes], "\n");
				ret = fwrite(data, bytes, 1, fp);
				bytes = 0;
				if (ret < 0) {
					WARNING("write to file fail.");
					goto exit_save_testing_data;
				}
			}

			j = 0;
			bytes = 0;
			for (i = 0; i < sys.key_number; i++) {
				if (_node_in_key_chn(i + sys.sc_sensor_num * sys.sc_driver_num, module_cfg, sys.key_offest) < 0) {
					continue;
				}
				bytes += (s32) sprintf((char *)&data[bytes], "%d,", min_key_limit_value[j++]);
			}
			bytes += (s32) sprintf((char *)&data[bytes], "\n");
			ret = fwrite(data, bytes, 1, fp);
			if (ret < 0) {
				WARNING("write to file fail.");
				goto exit_save_testing_data;
			}
		}

		if ((test_types & _ACCORD_CHECK) != 0) {
			bytes = (s32) sprintf((char *)data, "Channel average:(%d)\n", FLOAT_AMPLIFIER);
			for (i = 0; i < sys.sc_sensor_num; i++) {
				for (j = 0; j < sys.sc_driver_num; j++) {
					bytes += (s32) sprintf((char *)&data[bytes], "%ld,", accord_limit[i + j * sys.sc_sensor_num]);
				}
				bytes += (s32) sprintf((char *)&data[bytes], "\n");
				ret = fwrite(data, bytes, 1, fp);
				bytes = 0;
				if (ret < 0) {
					WARNING("write to file fail.");
					goto exit_save_testing_data;
				}
			}

			bytes = (s32) sprintf((char *)data, "\n");
			ret = fwrite(data, bytes, 1, fp);
			if (ret < 0) {
				WARNING("write to file fail.");
				goto exit_save_testing_data;
			}

			if (sys.chip_type == _GT1143 || sys.chip_type == _GT1133) {
				bytes = (s32) sprintf((char *)data, "Channel line accord:(%d)\n", FLOAT_AMPLIFIER);
				for (i = 0; i < sys.sc_sensor_num; i++) {
					for (j = 0; j < sys.sc_driver_num; j++) {
						bytes += (s32) sprintf((char *)&data[bytes], "%ld,", accord_line_limit[i + j * sys.sc_sensor_num]);
					}
					bytes += (s32) sprintf((char *)&data[bytes], "\n");
					ret = fwrite(data, bytes, 1, fp);
					bytes = 0;
					if (ret < 0) {
						WARNING("write to file fail.");
						goto exit_save_testing_data;
					}
				}

				bytes = (s32) sprintf((char *)data, "\n");
				ret = fwrite(data, bytes, 1, fp);
				if (ret < 0) {
					WARNING("write to file fail.");
					goto exit_save_testing_data;
				}
			}
		}

		bytes = (s32) sprintf((char *)data, " Rawdata\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			WARNING("write to file fail.");
			goto exit_save_testing_data;
		}
	}

	bytes = (s32) sprintf((char *)data, "No.%d\n", current_data_index);
	ret = fwrite(data, bytes, 1, fp);
	if (ret < 0) {
		WARNING("write to file fail.");
		goto exit_save_testing_data;
	}

	max = min = current_data_temp[0];
	average = 0;
	for (i = 0; i < sys.sc_sensor_num; i++) {
		bytes = 0;
		for (j = 0; j < sys.sc_driver_num; j++) {
			tmp = current_data_temp[i + j * sys.sc_sensor_num];
			bytes += (s32) sprintf((char *)&data[bytes], "%d,", tmp);
			if (tmp > max) {
				max = tmp;
			}
			if (tmp < min) {
				min = tmp;
			}
			average += tmp;
		}
		bytes += (s32) sprintf((char *)&data[bytes], "\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			WARNING("write to file fail.");
			goto exit_save_testing_data;
		}
	}
	average = average / (sys.sc_sensor_num * sys.sc_driver_num);

	if (sys.key_number > 0) {
		bytes = (s32) sprintf((char *)data, "Key Rawdata:\n");
		for (i = 0; i < sys.key_number; i++) {
			if (_node_in_key_chn(i + sys.sc_sensor_num * sys.sc_driver_num, module_cfg, sys.key_offest) < 0) {
				continue;
			}
			bytes += (s32) sprintf((char *)&data[bytes], "%d,", current_data_temp[i + sys.sc_sensor_num * sys.sc_driver_num]);
		}
		bytes += (s32) sprintf((char *)&data[bytes], "\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			WARNING("write to file fail.");
			goto exit_save_testing_data;
		}
	}

	bytes = (s32) sprintf((char *)data, "  Maximum:%d  Minimum:%d  Average:%d\n\n", max, min, average);
	ret = fwrite(data, bytes, 1, fp);
	if (ret < 0) {
		WARNING("write to file fail.");
		goto exit_save_testing_data;
	}

	if ((test_types & _ACCORD_CHECK) != 0) {
		bytes = (s32) sprintf((char *)data, "Channel_Accord :(%d)\n", FLOAT_AMPLIFIER);
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			WARNING("write to file fail.");
			goto exit_save_testing_data;
		}
		for (i = 0; i < sys.sc_sensor_num; i++) {
			bytes = 0;
			for (j = 0; j < sys.sc_driver_num; j++) {
				bytes += (s32) sprintf((char *)&data[bytes], "%ld,", channel_max_accord[i + j * sys.sc_sensor_num]);
			}
			bytes += (s32) sprintf((char *)&data[bytes], "\n");
			ret = fwrite(data, bytes, 1, fp);
			if (ret < 0) {
				WARNING("write to file fail.");
				goto exit_save_testing_data;
			}
		}
		bytes = (s32) sprintf((char *)data, "\n");
		ret = fwrite(data, bytes, 1, fp);
		if (ret < 0) {
			WARNING("write to file fail.");
			goto exit_save_testing_data;
		}
	}
exit_save_testing_data:
	/* DEBUG("step4"); */
	free(data);
	/* DEBUG("step3"); */
	fclose(fp);
	/* DEBUG("step2"); */
	return ret;
}

static s32 _save_test_result_data(char *save_test_data_dir, int test_types, u8 * shortresult)
{
	FILE *fp = NULL;
	s32 ret, index;
	u8 *data = NULL;
	s32 bytes = 0;
	data = (u8 *) malloc(MAX_BUFFER_SIZE);
	if (NULL == data) {
		WARNING("memory error!");
		return MEMORY_ERR;
	}

	fp = fopen((const char *)save_test_data_dir, "a+");
	if (NULL == fp) {
		WARNING("open %s failed!", save_test_data_dir);
		free(data);
		return FILE_OPEN_CREATE_ERR;
	}

	bytes = (s32) sprintf((char *)data, "Test Result:");
	if (test_error_code == _CHANNEL_PASS) {
		bytes += (s32) sprintf((char *)&data[bytes], "Pass\n\n");
	} else {
		bytes += (s32) sprintf((char *)&data[bytes], "Fail\n\n");
	}
	bytes += (s32) sprintf((char *)&data[bytes], "Test items:\n");
	if ((test_types & _MAX_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Max Rawdata:  ");
		if (test_error_code & _BEYOND_MAX_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _MIN_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Min Rawdata:  ");
		if (test_error_code & _BEYOND_MIN_LIMIT) {
			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");
		} else {
			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");
		}
	}

	if ((test_types & _ACCORD_CHECK) != 0) {
		bytes += (s32) sprintf((char *)&data[bytes], "Area Accord:  ");

		if (test_error_code & _BETWEEN_ACCORD_AND_LINE) {
			bytes += (s32) sprintf((char *)&data[bytes], "Fuzzy !\n");
		} else {
			if (test_error_code & _BEYOND_ACCORD_LIMIT) {

				bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

			} else {

				bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

			}
		}
	}

	if ((test_types & _OFFSET_CHECK) != 0) {

		bytes += (s32) sprintf((char *)&data[bytes], "Max Offest:  ");

		if (test_error_code & _BEYOND_OFFSET_LIMIT) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	if ((test_types & _JITTER_CHECK) != 0) {

		bytes += (s32) sprintf((char *)&data[bytes], "Max Jitier:  ");

		if (test_error_code & _BEYOND_JITTER_LIMIT) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	if (test_types & _UNIFORMITY_CHECK) {

		bytes += (s32) sprintf((char *)&data[bytes], "Uniformity:  ");

		if (test_error_code & _BEYOND_UNIFORMITY_LIMIT) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	if ((test_types & _KEY_MAX_CHECK) != 0) {

		bytes += (s32) sprintf((char *)&data[bytes], "Key Max Rawdata:  ");

		if (test_error_code & _KEY_BEYOND_MAX_LIMIT) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	if ((test_types & _KEY_MIN_CHECK) != 0) {

		bytes += (s32) sprintf((char *)&data[bytes], "Key Min Rawdata:  ");

		if (test_error_code & _KEY_BEYOND_MIN_LIMIT) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	if (test_types & (_VER_EQU_CHECK | _VER_GREATER_CHECK | _VER_BETWEEN_CHECK)) {

		bytes += (s32) sprintf((char *)&data[bytes], "Device Version:  ");

		if (test_error_code & _VERSION_ERR) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	if (test_types & _MODULE_TYPE_CHECK) {

		bytes += (s32) sprintf((char *)&data[bytes], "Module Type:  ");

		if (test_error_code & _MODULE_TYPE_ERR) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n");

		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
	}

	ret = fwrite(data, bytes, 1, fp);

	if (ret < 0) {

		WARNING("write to file fail.");

		free(data);

		fclose(fp);

		return ret;

	}

	if ((test_types & _MODULE_SHORT_CHECK) != 0) {

		bytes = (s32) sprintf((char *)data, "Module short test:  ");

		if (test_error_code & _GT_SHORT) {

			bytes += (s32) sprintf((char *)&data[bytes], "NG !\n\n\nError items:\nShort:\n");

			if (shortresult[0] > _GT9_UPLOAD_SHORT_TOTAL) {

				WARNING("short total over limit, data error!");

				shortresult[0] = 0;

			}

			for (index = 0; index < shortresult[0]; index++) {

				/* DEBUG("bytes=%d shortresult[0]=%d",bytes,shortresult[0]); */
				if (shortresult[1 + index * 4] & 0x80) {

					bytes += (s32) sprintf((char *)&data[bytes], "Drv%d - ", shortresult[1 + index * 4] & 0x7F);

				}

				else {

					if (shortresult[1 + index * 4] == (sys.max_driver_num + 1)) {

						bytes += (s32) sprintf((char *)&data[bytes], "GND\\VDD%d - ", shortresult[1 + index * 4] & 0x7F);

					}

					else {

						bytes += (s32) sprintf((char *)&data[bytes], "Sen%d - ", shortresult[1 + index * 4] & 0x7F);

					}
				}
				if (shortresult[2 + index * 4] & 0x80) {

					bytes += (s32) sprintf((char *)&data[bytes], "Drv%d 之间短路", shortresult[2 + index * 4] & 0x7F);

				}

				else {

					if (shortresult[2 + index * 4] == (sys.max_driver_num + 1)) {

						bytes += (s32) sprintf((char *)&data[bytes], "GND\\VDD 之间短路");

					}

					else {

						bytes += (s32) sprintf((char *)&data[bytes], "Sen%d 之间短路", shortresult[2 + index * 4] & 0x7F);

					}
				}
				bytes += (s32) sprintf((char *)&data[bytes], "(R=%d Kohm)\n", (((shortresult[3 + index * 4] << 8) + shortresult[4 + index * 4]) & 0xffff) / 10);

				DEBUG("%d&%d:", shortresult[1 + index * 4], shortresult[2 + index * 4]);

				DEBUG("%dK", (((shortresult[3 + index * 4] << 8) + shortresult[4 + index * 4]) & 0xffff) / 10);

			}
		}

		else {

			bytes += (s32) sprintf((char *)&data[bytes], "pass\n");

		}
		ret = fwrite(data, bytes, 1, fp);

		if (ret < 0) {

			WARNING("write to file fail.");

			free(data);

			fclose(fp);

			return ret;

		}

	}

	free(data);

	fclose(fp);

	return 1;

}

#endif

static void _unzip_nc(unsigned char *s_buf, unsigned char *key_nc_buf, unsigned short s_length, unsigned short key_length)
{

	unsigned short i, point;

	unsigned char m, data;

	int b_size = sys.max_driver_num * sys.max_sensor_num;

	u8 *tmp;

	tmp = (u8 *) (&global_large_buf[s_length + key_length]);

	memset(need_check, 0, b_size);

	memset(tmp, 0, b_size);

	point = 0;

	for (i = 0; i < s_length; i++) {

		data = *s_buf;

		for (m = 0; m < 8; m++) {

			if (point >= b_size) {

				goto KEY_NC_UNZIP;

			}

			tmp[point] &= 0xfe;

			if ((data & 0x80) == 0x80) {

				tmp[point] |= 0x01;

			}

			data <<= 1;

			point++;
		}

		s_buf++;

	}

	/* memcpy(need_check,tmp,sys.sensor_num*sys.sc_driver_num); */
KEY_NC_UNZIP:

	for (i = 0, point = 0; i < sys.sc_driver_num; i++) {

		for (m = 0; m < sys.sc_sensor_num; m++) {

			need_check[point++] = tmp[i + m * sys.sc_driver_num];
//                      DEBUG("need_check[%d]%d",point-1,need_check[point-1]);
		}

	}

	DEBUG("Load key nc\n");

	memset(channel_key_need_check, 0, MAX_KEY_RAWDATA);

	point = 0;

	for (i = 0; i < key_length; i++) {

		data = *key_nc_buf;

		for (m = 0; m < 8; m++) {

			if (point >= MAX_KEY_RAWDATA) {

				return;

			}

			channel_key_need_check[point] &= 0xfe;

			if ((data & 0x80) == 0x80) {

				channel_key_need_check[point] |= 0x01;

			}

			data <<= 1;

			point++;

		}

		key_nc_buf++;

	}

}

#if GTP_TEST_PARAMS_FROM_INI
static s32 _init_special_node(char *inipath, const char *key, int *max_limit, int *min_limit, long *accord_limit)
{

	FILE *fp = NULL;

	char *buf = NULL;

	char *tmp = NULL;

	size_t bytes = 0;

	s32 i = 0, space_count = 0;

	u16 tmpNode = 0, max, min;

	long accord;

	int b_size = sys.max_sensor_num * sys.max_driver_num;

	if (NULL == inipath) {
		return PARAMETERS_ILLEGL;
	}

	buf = (char *)malloc(b_size * 4 * 6);

	if (NULL == buf) {

		return MEMORY_ERR;

	}

	fp = fopen((const char *)inipath, "r");

	if (fp == NULL) {

		free(buf);

		buf = NULL;

		DEBUG("open %s fail!", inipath);

		return INI_FILE_OPEN_ERR;

	}
	//while(!feof(fp))
	while (1) {

		i = 0;

		space_count = 0;

		do {

			bytes = fread(&buf[i], 1, 1, fp);

			if (i >= b_size * 4 * 6 || bytes < 0) {

				fclose(fp);

				free(buf);

				return INI_FILE_READ_ERR;

			}
			/* DEBUG("%c", buf[i]); */

			if (buf[i] == ' ') {

				continue;

			}

		} while (buf[i] != '\r' && buf[i++] != '\n');

		//buf[i] = '\0';

		getrid_space(buf, i);

		strtok((char *)buf, "=");

		if (0 == strcmp((const char *)buf, key)) {

			i = 0;

			DEBUG("Begin get node data.");

			do {

				tmp = (char *)strtok((char *)NULL, ",");

				if (tmp == NULL) {

					break;

				}

				tmpNode = atoi((char const *)tmp);

				/* DEBUG("tmpNode:%d", tmpNode); */

				tmp = (char *)strtok((char *)NULL, ",");

				if (tmp == NULL) {

					fclose(fp);

					free(buf);

					return INI_FILE_ILLEGAL;

				}

				max = atoi((char const *)tmp);

				/* DEBUG("max:%d", max); */

				tmp = (char *)strtok((char *)NULL, ",");

				if (tmp == NULL) {

					fclose(fp);

					free(buf);

					return INI_FILE_ILLEGAL;

				}

				min = atoi((char const *)tmp);

				/* DEBUG("min:%d", min); */

				tmp = (char *)strtok((char *)NULL, ",");

				if (tmp == NULL) {

					fclose(fp);

					free(buf);

					return INI_FILE_ILLEGAL;

				}

				accord = (atof((char const *)tmp));	//*FLOAT_AMPLIFIER

//                              DEBUG("accord:%d", (int)accord);

				if (tmpNode < sys.sc_driver_num * sys.sc_sensor_num) {

					tmpNode = tmpNode / sys.sc_driver_num + (tmpNode % sys.sc_driver_num) * sys.sc_sensor_num;

					if (max_limit != NULL) {
						max_limit[tmpNode] = max;
					}

					if (min_limit != NULL) {
						min_limit[tmpNode] = min;
					}

					if (accord_limit != NULL) {
						accord_limit[tmpNode] = accord;
					}
//                                      max_limit_value[tmpNode] = max;
//
//                                      min_limit_value[tmpNode] = min;
//
//                                      accord_limit[tmpNode] = accord;

				}

				else {

					tmpNode -= sys.sc_driver_num * sys.sc_sensor_num;

					max_key_limit_value[tmpNode] = max;

					min_key_limit_value[tmpNode] = min;
				}

			} while (++i < b_size);

			/* DEBUG("get node data end."); */
			fclose(fp);

			free(buf);

			return 1;

		}

	}

	fclose(fp);

	free(buf);

	return INI_FILE_ILLEGAL;

}
#else
static s32 _init_special_node_array(void)
{
	s32 i = 0;

	u16 tmpNode = 0;
	const u16 special_node_tmp[] = SEPCIALTESTNODE;
	const u16 special_line_node_tmp[] = SEPCIALLINECHKNODE;

	for (i = 0; i < sizeof(special_node_tmp) / sizeof(special_node_tmp[0]); i += 4) {
		tmpNode = special_node_tmp[i];
		if (tmpNode < sys.sc_driver_num * sys.sc_sensor_num) {
			tmpNode = tmpNode / sys.sc_driver_num + (tmpNode % sys.sc_driver_num) * sys.sc_sensor_num;

			max_limit_value[tmpNode] = special_node_tmp[i + 1];

			min_limit_value[tmpNode] = special_node_tmp[i + 2];

			accord_limit[tmpNode] = special_node_tmp[i + 3];

		} else {
			tmpNode -= sys.sc_driver_num * sys.sc_sensor_num;

			max_key_limit_value[tmpNode] = special_node_tmp[i + 1];

			min_key_limit_value[tmpNode] = special_node_tmp[i + 2];
		}
	}

	for (i = 0; i < sizeof(special_line_node_tmp) / sizeof(special_line_node_tmp[0]); i += 4) {
		tmpNode = special_line_node_tmp[i];
		if (tmpNode < sys.sc_driver_num * sys.sc_sensor_num) {
			tmpNode = tmpNode / sys.sc_driver_num + (tmpNode % sys.sc_driver_num) * sys.sc_sensor_num;

			accord_line_limit[tmpNode] = special_line_node_tmp[i + 3];

		}
	}

	return i / 4;
}
#endif
static s32 _check_rawdata_proc(int check_types, u16 * data, int len, char *save_path)
{

	if (data == NULL || len < sys.driver_num * sys.sensor_num) {

		return PARAMETERS_ILLEGL;

	}

	memcpy(current_data_temp, data, sys.driver_num * sys.sensor_num * 2);

	_get_channel_max_value();

	_get_channel_min_value();

	_get_channel_average_value();

	if ((check_types & _MAX_CHECK) != 0) {

		_check_channel_max_value();

		DEBUG("After max check\n");

		DEBUG_DATA(channel_status, sys.driver_num * sys.sensor_num);

	}

	if ((check_types & _MIN_CHECK) != 0) {

		_check_channel_min_value();

		DEBUG("After min check\n");

		DEBUG_DATA(channel_status, sys.driver_num * sys.sensor_num);

	}

	if ((check_types & _KEY_MAX_CHECK) != 0) {

		_check_key_max_value();

		DEBUG("After key max check\n");

		DEBUG_DATA(channel_status, sys.driver_num * sys.sensor_num);

	}

	if ((check_types & _KEY_MIN_CHECK) != 0) {

		_check_key_min_value();

		DEBUG("After key min check\n");

		DEBUG_DATA(channel_status, sys.driver_num * sys.sensor_num);

	}

	if ((check_types & _ACCORD_CHECK) != 0) {

		if (sys.chip_type == _GT1143 || sys.chip_type == _GT1133) {
			_check_area_accord_1143();
		} else {
			_check_area_accord();
		}

		DEBUG("After area accord check\n");

		DEBUG_DATA(channel_status, sys.driver_num * sys.sensor_num);

	}

	if ((check_types & _OFFSET_CHECK) != 0) {

		_check_full_screen_offest(check_types & _SPECIAL_CHECK);

		DEBUG("After offset check:%ld", offset_limit);

		DEBUG_DATA(channel_status, sys.driver_num * sys.sensor_num);

	}

	if ((check_types & _UNIFORMITY_CHECK) != 0) {

		_check_uniformity();

		DEBUG("After uniformity check:%ld\n", uniformity_limit);

	}
#if GTP_SAVE_TEST_DATA
//  if ((check_types & _TEST_RESULT_SAVE) != 0)
	{
		_save_testing_data(save_path, check_types);
	}

#endif /*
        */

	DEBUG("The %d group rawdata test end!\n", current_data_index);

	current_data_index++;

	if (current_data_index < samping_set_number) {

		return 0;

	}

	if ((check_types & _JITTER_CHECK) != 0) {

		_check_full_screen_jitter();

		DEBUG("After FullScreenJitterCheck\n");

	}

	if (_rawdata_test_result_analysis(check_types) == 0) {

		DEBUG("After TestResultAnalyse\n");

		return 0;

	}

	_accord_test_result_analysis(check_types);

	DEBUG("rawdata test end!\n");

	return 1;
}

static s32 _check_other_options(int type)
{

	int ret = 0;

	if (type & (_VER_EQU_CHECK | _VER_GREATER_CHECK | _VER_BETWEEN_CHECK)) {

		ret = _check_device_version(type);

		if (ret < 0) {

			return ret;

		}

	}

	if (type & _MODULE_TYPE_CHECK) {

		ret = _check_modele_type(type);

		if (ret < 0) {

			return ret;

		}

		DEBUG("module test");

	}

	return ret;

}

static int _hold_ss51_dsp(void)
{

	int ret = -1;

	int retry = 0;

	unsigned char rd_buf[3];

	/* reset cpu */
	reset_guitar();

	enter_update_mode();

	while (retry++ < 200) {

		/* Hold ss51 & dsp */
		rd_buf[0] = 0x0C;

		ret = i2c_write_data(_rRW_MISCTL__SWRST_B0_, rd_buf, 1);

		if (ret <= 0) {

			DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);

			continue;

		}

		usleep(2 * 1000);

		if (retry < 100) {
			continue;
		}
		/* Confirm hold */
		ret = i2c_read_data(_rRW_MISCTL__SWRST_B0_, rd_buf, 1);

		if (ret <= 0) {

			DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);

			continue;

		}

		if (0x0C == rd_buf[0]) {

			DEBUG("Hold ss51 & dsp confirm SUCCESS");

			break;

		}

		DEBUG("Hold ss51 & dsp confirm 0x4180 failed,value:%d", rd_buf[0]);

	}

	if (retry >= 200) {

		WARNING("Enter update Hold ss51 failed.");

		return -1;

	}

	return 1;

}

/**
 * fun: take several tests of average value
 */
static int _ave_resist_analysis(u8 * s, u8 * d)
{
	u32 g_size, offest, ave;
	u8 i, j, k, cnt, s_chn1, s_chn2, upload_cnt, index;
	u16 *tmp;

	if (s == NULL || d == NULL) {
		return PARAMETERS_ILLEGL;
	}

	g_size = _GT9_UPLOAD_SHORT_TOTAL * 4 + 1;
	tmp = (u16 *) malloc(gt900_short_test_times * g_size * sizeof(u16));
	if (tmp == NULL) {
		return MEMORY_ERR;
	}
//      DEBUG("%s",__func__);
	DEBUG_ARRAY(s, gt900_short_test_times * g_size);
	cnt = s[0];
	for (i = 0; i < gt900_short_test_times; i++) {
		if (s[i * g_size] > cnt) {
			cnt = s[i * g_size];
		}
	}

	for (i = 0, upload_cnt = 0; i < cnt; i++) {
		s_chn1 = s[i * 4 + 1];
		s_chn2 = s[i * 4 + 2];
		memset(tmp, 0, gt900_short_test_times * g_size * 2);

		for (j = 0, index = 1; j < gt900_short_test_times; j++) {
			offest = j * g_size;
			for (k = 0; k < s[offest]; k++) {
				if ((s_chn1 == s[offest + k * 4 + 1] && s_chn2 == s[offest + k * 4 + 2]) || (s_chn2 == s[offest + k * 4 + 1] && s_chn1 == s[offest + k * 4 + 2])) {
					tmp[0]++;
					tmp[index++] = (s[offest + k * 4 + 3] << 8) + s[offest + k * 4 + 4];
//                                      DEBUG("tmp[%d]%d,offest %d",index -1,tmp[index -1],offest);
					break;
				}
			}
		}

		/*frequency */
		DEBUG("tmp[0]%d", tmp[0]);
		if (tmp[0] * FLOAT_AMPLIFIER >= gt900_short_test_times * FLOAT_AMPLIFIER * 8 / 10) {
			for (k = 0, ave = 0; k < tmp[0]; k++) {
				ave += tmp[k + 1];
//                              DEBUG("tmp[%d]%d",k+1,tmp[k+1]);
			}

			ave = ave / tmp[0];
//                      DEBUG("ave%d,cnt%d",ave,tmp[0]);
			d[upload_cnt * 4 + 1] = s[i * 4 + 1];
			d[upload_cnt * 4 + 2] = s[i * 4 + 2];
			d[upload_cnt * 4 + 3] = (u8) (ave >> 8);
			d[upload_cnt * 4 + 4] = (u8) (ave);
			upload_cnt++;
			test_error_code |= _GT_SHORT;
		}
	}

	d[0] = upload_cnt;

	free(tmp);
	return upload_cnt;
}

static s32 _check_short_circuit(int test_types, u8 * short_result)
{

	u8 *data, *upload_data_ave, *tmp, short_test_cnt = 0;

	s32 i, ret, g_size;

	DEBUG("short test start.");
	g_size = (_GT9_UPLOAD_SHORT_TOTAL * 4 + 1);
	data = (u8 *) malloc((gt900_short_test_times + 1) * g_size + 10);
	if (data == NULL) {
		return MEMORY_ERR;
	}

	memset(data, 0, (gt900_short_test_times + 1) * g_size + 10);
	tmp = (u8 *) & data[5];
	upload_data_ave = (u8 *) & data[5 + g_size];
gt900_short_again:
	/* select addr & hold ss51_dsp */
	ret = _hold_ss51_dsp();

	if (ret <= 0) {

		DEBUG("hold ss51 & dsp failed.");

		ret = ENTER_UPDATE_MODE_ERR;

		goto gt900_test_exit;

	}

	/******preparation of downloading the DSP code**********/
	enter_update_mode_noreset();

	DEBUG("Loading..\n");

	if (load_dsp_code_check() < 0) {

		ret = SHORT_TEST_ERROR;

		goto gt900_test_exit;

	}

	dsp_fw_startup(short_test_cnt);

	usleep(30 * 1000);

	for (i = 0; i < 100; i++) {

		i2c_read_data(_rRW_MISCTL__SHORT_BOOT_FLAG, data, 1);

		if (data[0] == 0xaa) {

			break;

		}

		DEBUG("buf[0]:0x%x", data[0]);

		usleep(10 * 1000);

	}

	if (i >= 20) {

		WARNING("Didn't get 0xaa at 0x%X\n", _rRW_MISCTL__SHORT_BOOT_FLAG);

		ret = SHORT_TEST_ERROR;

		goto gt900_test_exit;

	}

	write_test_params(module_cfg);

	/* check whether the test is completed */
	i = 0;

	while (1) {

		i2c_read_data(0x8800, data, 1);

		if (data[0] == 0x88) {

			break;

		}

		usleep(50 * 1000);

		i++;

		if (i > 150) {

			WARNING("Didn't get 0x88 at 0x8800\n");

			ret = SHORT_TEST_ERROR;

			goto gt900_test_exit;

		}

	}

	i = ((sys.max_driver_num + sys.max_sensor_num) * 2 + 20) / 4;
	i += (sys.max_driver_num + sys.max_sensor_num) * 2 / 4;

	i += (_GT9_UPLOAD_SHORT_TOTAL + 1) * 4;

	if (i > 1343) {

		ret = -1;
		goto gt900_test_exit;

	}

	/*
	 * DEBUG("AVDD:%d",sys_avdd);
	 DEBUG("gt900_short_threshold:%d",gt900_short_threshold);
	 DEBUG("gt900_resistor_warn_threshold:%d",gt900_resistor_warn_threshold);
	 DEBUG("gt900_drv_drv_resistor_threshold:%d",gt900_drv_drv_resistor_threshold);
	 DEBUG("gt900_drv_sen_resistor_threshold:%d",gt900_drv_sen_resistor_threshold);
	 DEBUG("gt900_sen_sen_resistor_threshold:%d",gt900_sen_sen_resistor_threshold);
	 DEBUG("gt900_drv_gnd_resistor_threshold:%d",gt900_drv_gnd_resistor_threshold);
	 DEBUG("gt900_sen_gnd_resistor_threshold:%d",gt900_sen_gnd_resistor_threshold);
	 */
	memset(tmp, 0, g_size);
	ret = short_test_analysis(tmp, _GT9_UPLOAD_SHORT_TOTAL);
	if (ret < 0) {
		goto gt900_test_exit;
	}

	memcpy(&upload_data_ave[short_test_cnt * g_size], tmp, tmp[0] * 4 + 1);
	short_test_cnt++;
	if (short_test_cnt < gt900_short_test_times) {
		DEBUG("short test again %d", short_test_cnt);
		goto gt900_short_again;
	} else {
		ret = _ave_resist_analysis(upload_data_ave, short_result);
		if (ret > 0) {

			test_to_show(&short_result[1], short_result[0]);

			ret = short_result[0];
		}
	}

gt900_test_exit:

	short_test_end();
	free(data);
	DEBUG("short upload_cnt:%d", short_result[0]);

	DEBUG("short test end,ret 0x%x", ret);

	return ret;
}

static s32 _alloc_test_memory(int b_size)
{

	int offest = 0;

	if (global_large_buf != NULL) {
		return 1;
	}

	global_large_buf = (u8 *) malloc(48 * b_size + 4 * 1024);

	if (global_large_buf == NULL) {

		return MEMORY_ERR;

	}

	memset(global_large_buf, 0, 48 * b_size + 4 * 1024);

	offest = 2 * 1024;

	need_check = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	channel_key_need_check = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	channel_status = (u16 *) (&global_large_buf[offest]);

	offest += b_size * sizeof(short);

	channel_max_value = (u16 *) (&global_large_buf[offest]);

	offest += b_size * sizeof(short);

	channel_min_value = (u16 *) (&global_large_buf[offest]);

	offest += b_size * sizeof(short);

	channel_max_accord = (long *)(&global_large_buf[offest]);

	offest += b_size * sizeof(long);

	channel_max_line_accord = (long *)(&global_large_buf[offest]);

	offest += b_size * sizeof(long);

	channel_average = (int *)(&global_large_buf[offest]);

	offest += b_size * sizeof(int);

	channel_square_average = (int *)(&global_large_buf[offest]);

	offest += b_size * sizeof(int);

	driver_status = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	beyond_max_limit_num = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	beyond_min_limit_num = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	beyond_accord_limit_num = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	beyond_offset_limit_num = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	beyond_jitter_limit_num = &global_large_buf[offest];

	offest += b_size * sizeof(char);

	max_limit_value = (s32 *) (&global_large_buf[offest]);

	offest += b_size * sizeof(int);

	min_limit_value = (s32 *) (&global_large_buf[offest]);

	offest += b_size * sizeof(int);

	accord_limit = (long *)(&global_large_buf[offest]);

	offest += b_size * sizeof(long);

	accord_line_limit = (long *)(&global_large_buf[offest]);

	offest += b_size * sizeof(long);

	ini_version1 = &global_large_buf[offest];

	offest += 50;

	ini_version2 = &global_large_buf[offest];

	offest += 50;

	original_cfg = &global_large_buf[offest];

	offest += 350;

	module_cfg = &global_large_buf[offest];

	offest += 350;

	max_key_limit_value = (s32 *) (&global_large_buf[offest]);

	offest += MAX_KEY_RAWDATA * sizeof(int);

	min_key_limit_value = (s32 *) (&global_large_buf[offest]);

	offest += MAX_KEY_RAWDATA * sizeof(int);

	current_data_temp = (u16 *) (&global_large_buf[offest]);

	offest += b_size * sizeof(short);

	DEBUG("offest:%d b_size:%d", offest, b_size);
	return 1;

}

static void _exit_test(void)
{

	disable_hopping(original_cfg, sys.config_length, 0);

	if (global_large_buf != NULL) {

		free(global_large_buf);

		global_large_buf = NULL;

	}

}

#if GTP_TEST_PARAMS_FROM_INI
static s32 _get_test_parameters(char *inipath)
{

	int test_types;

	if (inipath == NULL) {

		WARNING("ini file path is null.");

		return PARAMETERS_ILLEGL;

	}

	test_types = ini_read_hex(inipath, (const char *)"test_types");

	if (test_types < 0) {

		WARNING("get the test types 0x%x is error.", test_types);

		return test_types;

	}

	DEBUG("test type:%x\n", test_types);

	if (test_types & _MAX_CHECK) {

		max_limit_value_tmp = ini_read_int(inipath, (const char *)"max_limit_value");

		DEBUG("max_limit_value:%d\n", max_limit_value_tmp);

	}

	if (test_types & _MIN_CHECK) {

		min_limit_value_tmp = ini_read_int(inipath, (const char *)"min_limit_value");

		DEBUG("min_limit_value:%d\n", min_limit_value_tmp);

	}

	if (test_types & _ACCORD_CHECK) {

		accord_limit_tmp = ini_read_float(inipath, (const char *)"accord_limit");

		DEBUG("accord_limit:%ld", accord_limit_tmp);

	}

	if (test_types & _OFFSET_CHECK) {

		offset_limit = ini_read_float(inipath, (const char *)"offset_limit");

		DEBUG("offset_limit:%ld", offset_limit);

	}

	if (test_types & _JITTER_CHECK) {

		permit_jitter_limit = ini_read_int(inipath, (const char *)"permit_jitter_limit");

		DEBUG("permit_jitter_limit:%d\n", permit_jitter_limit);

	}

	if (test_types & _SPECIAL_CHECK) {

		special_limit = ini_read_float(inipath, (const char *)"special_limit");

		DEBUG("special_limit:%ld", special_limit);

	}

	if (test_types & _KEY_MAX_CHECK) {

		max_key_limit_value_tmp = ini_read_int(inipath, (const char *)"max_key_limit_value");

		DEBUG("max_key_limit_value:%d\n", max_key_limit_value_tmp);

	}

	if (test_types & _KEY_MIN_CHECK) {

		min_key_limit_value_tmp = ini_read_int(inipath, (const char *)"min_key_limit_value");

		DEBUG("min_key_limit_value:%d\n", min_key_limit_value_tmp);

	}

	if (test_types & _MODULE_TYPE_CHECK) {

		ini_module_type = ini_read_int(inipath, (const char *)"module_type");

		DEBUG("Sensor ID:%d\n", ini_module_type);

	}

	if (test_types & _VER_EQU_CHECK) {

		ini_read(inipath, (const char *)"version_equ", (char *)ini_version1);

		DEBUG("version_equ:%s", ini_version1);

	}

	else if (test_types & _VER_GREATER_CHECK) {

		ini_read(inipath, (const char *)"version_greater", (char *)ini_version1);

		DEBUG("version_greater:%s", ini_version1);

	}

	else if (test_types & _VER_BETWEEN_CHECK) {

		ini_read(inipath, (const char *)"version_between1", (char *)ini_version1);

		DEBUG("version_between1:%s", ini_version1);

		ini_read(inipath, (const char *)"version_between2", (char *)ini_version2);

		DEBUG("version_between2:%s", ini_version2);

	}

	if (test_types & _MODULE_SHORT_CHECK) {

		long lret;

		int ret = ini_read_int(inipath, (const char *)"gt900_short_threshold");

		if (ret > 0) {

			gt900_short_threshold = ret;

			DEBUG("gt900_short_threshold:%d", gt900_short_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_resistor_threshold");

		if (ret > 0) {

			gt900_drv_drv_resistor_threshold = ret;
			gt900_drv_sen_resistor_threshold = ret;
			gt900_sen_sen_resistor_threshold = ret;

			DEBUG("gt900_resistor_threshold:%d", ret);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_gnd_resistor_threshold");

		if (ret > 0) {

			gt900_drv_gnd_resistor_threshold = ret;
			gt900_sen_gnd_resistor_threshold = ret;

			DEBUG("gt900_gnd_resistor_threshold:%d", ret);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_drv_drv_resistor_threshold");

		if (ret > 0) {

			gt900_drv_drv_resistor_threshold = ret;

			DEBUG("gt900_drv_drv_resistor_threshold:%d", gt900_drv_drv_resistor_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_drv_sen_resistor_threshold");

		if (ret > 0) {

			gt900_drv_sen_resistor_threshold = ret;

			DEBUG("gt900_drv_sen_resistor_threshold:%d", gt900_drv_sen_resistor_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_sen_sen_resistor_threshold");

		if (ret > 0) {

			gt900_sen_sen_resistor_threshold = ret;

			DEBUG("gt900_sen_sen_resistor_threshold:%d", gt900_sen_sen_resistor_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_resistor_warn_threshold");

		if (ret > 0) {

			gt900_resistor_warn_threshold = ret;

			DEBUG("gt900_resistor_warn_threshold:%d", gt900_resistor_warn_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_drv_gnd_resistor_threshold");

		if (ret > 0) {

			gt900_drv_gnd_resistor_threshold = ret;

			DEBUG("gt900_drv_gnd_resistor_threshold:%d", gt900_drv_gnd_resistor_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_sen_gnd_resistor_threshold");

		if (ret > 0) {

			gt900_sen_gnd_resistor_threshold = ret;

			DEBUG("gt900_sen_gnd_resistor_threshold:%d", gt900_sen_gnd_resistor_threshold);

		}

		ret = ini_read_int(inipath, (const char *)"gt900_short_test_times");

		if (ret > 0) {

			gt900_short_test_times = ret;

			DEBUG("gt900_short_test_times:%d", gt900_short_test_times);

		}

		lret = ini_read_float(inipath, (const char *)"tri_pattern_r_ratio");

		if (ret > 0) {

			tri_pattern_r_ratio = lret;

			DEBUG("tri_pattern_r_ratio:%ld", tri_pattern_r_ratio);

		}

		ret = ini_read_float(inipath, (const char *)"AVDD");

		if (ret > 0) {

			sys_avdd = ret * 10 / FLOAT_AMPLIFIER;

			DEBUG("sys_avdd:%d", sys_avdd);

		}

	}

	uniformity_limit = ini_read_float(inipath, (const char *)"rawdata_uniformity");

	if (uniformity_limit > 0) {

		test_types |= _UNIFORMITY_CHECK;

		DEBUG("uniformity_limit:%ld", uniformity_limit);

	}

	DEBUG("_get_test_parameters success!");

	return test_types;

}
#else
static s32 _get_test_parameters_array(void)
{

	int test_types;

	test_types = TEST_TYPES;

	if (test_types < 0) {

		WARNING("get the test types 0x%x is error.", test_types);

		return test_types;

	}

	DEBUG("test type:%x\n", test_types);

	if (test_types & _MAX_CHECK) {

		max_limit_value_tmp = MAX_LIMIT_VALUE;

		DEBUG("max_limit_value:%d\n", max_limit_value_tmp);

	}

	if (test_types & _MIN_CHECK) {

		min_limit_value_tmp = MIN_LIMIT_VALUE;

		DEBUG("min_limit_value:%d\n", min_limit_value_tmp);

	}

	if (test_types & _ACCORD_CHECK) {

		accord_limit_tmp = ACCORD_LIMIT;

		DEBUG("accord_limit:%ld", accord_limit_tmp);

	}

	if (test_types & _OFFSET_CHECK) {

		offset_limit = OFFSET_LIMIT;

		DEBUG("offset_limit:%ld", offset_limit);

	}

	if (test_types & _JITTER_CHECK) {

		permit_jitter_limit = PERMIT_JITTER_LIMIT;

		DEBUG("permit_jitter_limit:%d\n", permit_jitter_limit);

	}

	if (test_types & _SPECIAL_CHECK) {

		special_limit = SPECIAL_LIMIT;

		DEBUG("special_limit:%ld", special_limit);

	}

	if (test_types & _KEY_MAX_CHECK) {

		max_key_limit_value_tmp = MAX_KEY_LIMIT_VALUE;

		DEBUG("max_key_limit_value:%d\n", max_key_limit_value_tmp);

	}

	if (test_types & _KEY_MIN_CHECK) {

		min_key_limit_value_tmp = MIN_KEY_LIMIT_VALUE;

		DEBUG("min_key_limit_value:%d\n", min_key_limit_value_tmp);

	}

	if (test_types & _MODULE_TYPE_CHECK) {

		ini_module_type = MODULE_TYPE;

		DEBUG("Sensor ID:%d\n", ini_module_type);

	}

	if (test_types & _VER_EQU_CHECK) {
		memcpy((char *)ini_version1, (const char *)VERSION_EQU, sizeof(VERSION_EQU));

		DEBUG("version_equ:%s", ini_version1);

	}

	else if (test_types & _VER_GREATER_CHECK) {

		memcpy(ini_version1, VERSION_GREATER, sizeof(VERSION_GREATER));
		DEBUG("version_greater:%s", ini_version1);

	}

	else if (test_types & _VER_BETWEEN_CHECK) {

		memcpy(ini_version1, VERSION_BETWEEN1, sizeof(VERSION_BETWEEN1));
		DEBUG("version_between1:%s", ini_version1);

		memcpy(ini_version2, VERSION_BETWEEN2, sizeof(VERSION_BETWEEN2));
		DEBUG("version_between2:%s", ini_version2);

	}

	if (test_types & _MODULE_SHORT_CHECK) {

		gt900_short_threshold = GT900_SHORT_THRESHOLD;

		DEBUG("gt900_short_threshold:%d", gt900_short_threshold);

		gt900_drv_drv_resistor_threshold = GT900_DRV_DRV_RESISTOR_THRESHOLD;

		DEBUG("gt900_drv_drv_resistor_threshold:%d", gt900_drv_drv_resistor_threshold);

		gt900_drv_sen_resistor_threshold = GT900_DRV_SEN_RESISTOR_THRESHOLD;

		DEBUG("gt900_drv_sen_resistor_threshold:%d", gt900_drv_sen_resistor_threshold);

		gt900_sen_sen_resistor_threshold = GT900_SEN_SEN_RESISTOR_THRESHOLD;

		DEBUG("gt900_sen_sen_resistor_threshold:%d", gt900_sen_sen_resistor_threshold);

		gt900_resistor_warn_threshold = GT900_RESISTOR_WARN_THRESHOLD;

		DEBUG("gt900_resistor_warn_threshold:%d", gt900_resistor_warn_threshold);

		gt900_drv_gnd_resistor_threshold = GT900_DRV_GND_RESISTOR_THRESHOLD;

		DEBUG("gt900_drv_gnd_resistor_threshold:%d", gt900_drv_gnd_resistor_threshold);

		gt900_sen_gnd_resistor_threshold = GT900_SEN_GND_RESISTOR_THRESHOLD;

		DEBUG("gt900_sen_gnd_resistor_threshold:%d", gt900_sen_gnd_resistor_threshold);

		gt900_short_test_times = GT900_SHORT_TEST_TIMES;

		DEBUG("gt900_short_test_times:%d", gt900_short_test_times);

		tri_pattern_r_ratio = TRI_PATTERN_R_RATIO;

		DEBUG("tri_pattern_r_ratio:%ld", tri_pattern_r_ratio);

		sys_avdd = AVDD * 10 / FLOAT_AMPLIFIER;

		DEBUG("sys_avdd:%d", sys_avdd);

	}

	uniformity_limit = RAWDATA_UNIFORMITY;

	if (uniformity_limit > 0) {

		test_types |= _UNIFORMITY_CHECK;

		DEBUG("uniformity_limit:%ld", uniformity_limit);

	}

	DEBUG("_get_test_parameters success!");

	return test_types;

}
#endif

static s32 _init_test_paramters(char *inipath)
{
	int b_size = sys.max_driver_num * sys.max_sensor_num + 10;

	int i;

	int check_types;

	int ret = -1;

	u8 *s_nc_buf;

	u8 *key_nc_buf;

#if !GTP_TEST_PARAMS_FROM_INI
	const u8 nc_tmp[] = NC;
	const u8 key_nc_tmp[] = KEY_NC;
	const u8 module_cfg_tmp[] = MODULE_CFG;
#endif

	DEBUG("%s", __func__);

	if (_alloc_test_memory(b_size) < 0) {

		return MEMORY_ERR;

	}

	DEBUG("begin _get_test_parameters");
#if GTP_TEST_PARAMS_FROM_INI
	check_types = _get_test_parameters(inipath);
//      check_types = 0x601003;
#else
	check_types = _get_test_parameters_array();
#endif
	if (check_types < 0) {
		return check_types;

	}

	current_data_index = 0;

	test_error_code = _CHANNEL_PASS;

	if ((check_types & _FAST_TEST_MODE) != 0) {

		samping_set_number = (samping_num / 4) + (samping_num % 4);

	}

	else {

		samping_set_number = samping_num;

	}

	beyond_uniformity_limit_num = 0;

	for (i = 0; i < b_size; i++) {

		channel_status[i] = _CHANNEL_PASS;

		channel_max_value[i] = 0;

		channel_min_value[i] = 0xFFFF;

		channel_max_accord[i] = 0;

		channel_max_line_accord[i] = 0;

		channel_average[i] = 0;

		channel_square_average[i] = 0;

		beyond_max_limit_num[i] = 0;

		beyond_min_limit_num[i] = 0;

		beyond_accord_limit_num[i] = 0;

		beyond_offset_limit_num[i] = 0;

		max_limit_value[i] = max_limit_value_tmp;

		min_limit_value[i] = min_limit_value_tmp;

		accord_limit[i] = accord_limit_tmp;

	}

	for (i = 0; i < MAX_KEY_RAWDATA; i++) {

		max_key_limit_value[i] = max_key_limit_value_tmp;

		min_key_limit_value[i] = min_key_limit_value_tmp;

	}

	read_config(original_cfg, sys.config_length);

#if GTP_TEST_PARAMS_FROM_INI
	ret = ini_read_text(inipath, (const char *)"module_cfg", module_cfg);
#else
	memcpy(module_cfg, module_cfg_tmp, sizeof(module_cfg_tmp));
	ret = sizeof(module_cfg_tmp);
#endif

	DEBUG("module_cfg len %d ,config len %d", ret, sys.config_length);

	if (ret < 0 || ret != sys.config_length) {

		DEBUG("switch the original cfg.");

		memcpy(module_cfg, original_cfg, sys.config_length);

		disable_hopping(module_cfg, sys.config_length, 1);

	} else {

		DEBUG("switch the module cfg.");
		module_cfg[0] = original_cfg[0];
//              for(int i = 0; i < sys.config_length; i++)
//                      {
//                      DEBUG("module[%d]0x%X,original[%d]0x%X",i,module_cfg[i],i,original_cfg[i]);
//                      }
		if (disable_hopping(module_cfg, sys.config_length, 0) < 0) {
			memcpy(module_cfg, original_cfg, sys.config_length);
		}
	}
	s_nc_buf = (unsigned char *)(&global_large_buf[0]);
	key_nc_buf = (unsigned char *)(&global_large_buf[2 * (b_size / 8) + 10]);

	memset(s_nc_buf, 0, 2 * (b_size / 8) + 10);

	memset(key_nc_buf, 0, sys.max_sensor_num / 8 + 10);

	DEBUG("here\n");

#if GTP_TEST_PARAMS_FROM_INI
	ini_read_text(inipath, (const char *)"NC", s_nc_buf);
	ret = ini_read_text(inipath, (const char *)"KEY_NC", key_nc_buf);
#else
	memcpy(s_nc_buf, nc_tmp, sizeof(nc_tmp));
	memcpy(key_nc_buf, key_nc_tmp, sizeof(key_nc_tmp));
	ret = sizeof(key_nc_tmp);
#endif

	DEBUG("WITH KEY?:0x%x\n", ret);

	if (ret <= 0) {
		_unzip_nc(s_nc_buf, key_nc_buf, b_size / 8 + 8, 0);
	} else {
		_unzip_nc(s_nc_buf, key_nc_buf, b_size / 8 + 8, sys.max_sensor_num);
	}

//      DEBUG("need check array:\n");

//      DEBUG_ARRAY(need_check, sys.sc_driver_num * sys.sc_sensor_num);

//      DEBUG("key need check array:\n");

	/* DEBUG_ARRAY(channel_key_need_check, b_size); */

//      DEBUG("key_nc_buf:\n");

	/* DEBUG_ARRAY(key_nc_buf, strlen((const char*)key_nc_buf)); */
#if GTP_TEST_PARAMS_FROM_INI
	ret = _init_special_node(inipath, "SepcialTestNode", max_limit_value, min_limit_value, accord_limit);
	if (ret < 0) {

		WARNING("set special node fail, ret 0x%x", ret);

	}

	for (i = 0; i < b_size; i++) {
		accord_line_limit[i] = accord_limit[i];
	}

	ret = _init_special_node(inipath, "SepcialLineChkNode", NULL, NULL, accord_line_limit);
	if (ret < 0) {

		WARNING("set line special node fail, ret 0x%x", ret);

	}
#else
	_init_special_node_array();
#endif

	return check_types;

}

static int _check_config(void)
{
	u8 *config;
	int ret = -1;

	if (module_cfg == NULL) {
		return -1;
	}

	config = (u8 *) malloc(sys.config_length);
	if (config == NULL) {
		return MEMORY_ERR;
	}

	memset(config, 0, sys.config_length);

	read_config(config, sys.config_length);
	if (memcmp(&module_cfg[1], &config[1], sys.config_length - 2) == 0) {
		ret = 1;
	}
//      else
//      {
//              for(i = 0; i < sys.config_length; i++)
//              {
//                      DEBUG("module[%d]0x%02X, config[%d]0x%02X",i,module_cfg[i],i,config[i]);
//              }
//      }

	free(config);
	return ret;
}

s32 open_short_test(unsigned char *short_result_data)
{
	int ret = -1, test_types;

	char times = 0, timeouts = 0, check_cfg = 0;

	u16 *rawdata;

	u8 *largebuf, *short_result;

	char *ini_path, *save_path;

	largebuf = (unsigned char *)malloc(600 + sys.max_sensor_num * sys.max_driver_num * 2);

	if (largebuf == NULL) {
		return MEMORY_ERR;
	}
TEST_START:

	memset(largebuf, 0, 600 + sys.max_sensor_num * sys.max_driver_num * 2);

	ini_path = (char *)(&largebuf[0]);

	short_result = (u8 *) (&largebuf[250]);

	save_path = (char *)(&largebuf[350]);

	rawdata = (u16 *) (&largebuf[600]);

	DEBUG("sen*drv:%d*%d", sys.sensor_num, sys.driver_num);

#if GTP_TEST_PARAMS_FROM_INI
	if (auto_find_ini(ini_find_dir1, ini_format, ini_path) < 0) {

		if (auto_find_ini(ini_find_dir2, ini_format, ini_path) < 0) {

			WARNING("Not find the ini file.");

			free(largebuf);

			return INI_FILE_ILLEGAL;

		}
	}

	DEBUG("find ini path:%s len %zd", ini_path, strlen(ini_path));
#endif

	test_types = _init_test_paramters(ini_path);

	if (test_types < 0) {

		WARNING("get test params failed.");

		free(largebuf);

		return test_types;

	}

	FORMAT_PATH(save_path, save_result_dir, "test_data");

	DEBUG("save path is %s", save_path);

	memset(save_data_path, 0, sizeof(save_data_path));
	memcpy(save_data_path, save_path, strlen(save_path));

	memset(short_result, 0, 100);

	if (test_types & _MODULE_SHORT_CHECK || ((sys.chip_type == _GT1143 || sys.chip_type == _GT1133) && test_types & _ACCORD_CHECK)) {
		if (disable_irq_esd() < 0) {

			WARNING("disable irq and esd fail.");

			goto TEST_END;

		}

		usleep(20 * 1000);

		if (sys.chip_type == _GT1143 || sys.chip_type == _GT1133) {
			gt900_drv_gnd_resistor_threshold = gt900_sen_gnd_resistor_threshold;
			gt900_drv_drv_resistor_threshold = gt900_sen_sen_resistor_threshold;
			gt900_drv_sen_resistor_threshold = gt900_sen_sen_resistor_threshold;

		}

		ret = _check_short_circuit(test_types, short_result);
		if (sys.chip_type != _GT1143 && sys.chip_type != _GT1133) {
			reset_guitar();

			usleep(2 * 1000);

			disable_hopping(module_cfg, sys.config_length, 0);
		}

		if (ret < 0) {

			WARNING("Short Test Fail.");

			goto TEST_END;

		}

		DEBUG("cnt %d", short_result[0]);

		if (short_result_data != NULL) {

			memcpy(short_result_data, short_result, short_result[0] * 4 + 1);
		}
		if ((sys.chip_type == _GT1143 || sys.chip_type == _GT1133) && test_error_code & _GT_SHORT) {
			goto TEST_COMPLETE;
		}
	}

	ret = _check_other_options(test_types);

	if (ret < 0) {

		WARNING("DeviceVersion or ModuleType test failed.");

		goto TEST_END;

	}

	times = 0;
	timeouts = 0;
	if (test_types & (_MAX_CHECK | _MIN_CHECK | _ACCORD_CHECK | _OFFSET_CHECK | _JITTER_CHECK | _KEY_MAX_CHECK | _KEY_MIN_CHECK | _UNIFORMITY_CHECK)) {

		while (times < 64) {

			ret = read_raw_data(rawdata, sys.sensor_num * sys.driver_num);

			if (ret < 0) {

				DEBUG("read rawdata timeout %d.", timeouts);

				if (++timeouts > 5) {

					WARNING("read rawdata timeout.");

					break;

				}

				be_normal();

				continue;

			}

			ret = _check_rawdata_proc(test_types, rawdata, sys.sensor_num * sys.driver_num, save_path);

			if (ret < 0) {

				WARNING("raw data test proc error.");

				break;

			}

			else if (ret == 1) {

				DEBUG("rawdata check finish.");

				break;

			}

			times++;

		}

		be_normal();

		if (ret < 0) {

			WARNING("rawdata check fail.");

			goto TEST_END;

		}

		if (check_cfg < 1) {
			check_cfg++;
			if (_check_config() < 0) {
				WARNING("The configuration is accidental changes.");
				disable_hopping(original_cfg, sys.config_length, 0);
				goto TEST_START;
			}
		}
	}

TEST_COMPLETE:

	ret = test_error_code;

#if GTP_SAVE_TEST_DATA
//  if ((check_types & _TEST_RESULT_SAVE) != 0)
	{

		_save_test_result_data(save_path, test_types, short_result);

	}

#endif

TEST_END:

	if (sys.chip_type == _GT1143 || sys.chip_type == _GT1133) {
		reset_guitar();

		usleep(2 * 1000);
	}

	enable_irq_esd();

	_exit_test();

	DEBUG("test result 0x%X", ret);

	free(largebuf);

	return ret;

}

#ifdef __cplusplus
//}
#endif
