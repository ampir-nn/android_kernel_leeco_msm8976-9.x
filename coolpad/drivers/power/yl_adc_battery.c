/* Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 *yulong ADC convert SOC driver
 *
 */

#define pr_fmt(fmt) "yl_adc_batt: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/alarmtimer.h>
#include <linux/wakelock.h>
#include <linux/qpnp/power-on.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/platform_device.h>
#include "yl_pm8916_vbus.h"

#ifdef CONFIG_USE_QC_BMS_BATT_FORMAT
#include <linux/of_batterydata.h>
#endif


extern int pon_batt_volt;
extern int yl_get_lcd_status;
extern int yl_get_bk_level;

struct yl_adc_batt_chip {
	struct device         *dev;

	bool	     shutdown_soc_invalid;
	bool      vbus_present;
	/* battery status tracking */
	bool      batt_present;
	int        batt_temp;
	int        batt_volt;
	int        batt_crude_volt;
	int        batt_crude_capa;
	int        pon_volt;
	int        pon_volt_delta;
	int 	    shutdown_ocv;
	int 	    shutdown_soc;
	int 	    warm_reset;

	int        charge_stat;
	int        to_sys_capa;
	int        last_to_sys_capa;
	int        report_soc;

	int        lcd_open_delta_mv;
	int        lcd_close_delta_mv;
	int        bk_level_offset_step;

	int        voltage_max_uv;
	int        capa_design_mAH;
	int        soc_high_scaled;
	int        soc_low_scaled;

	const char     *adc_batt_psy_name;
	const char     *charge_psy_name;

	struct qpnp_vadc_chip           *vadc_dev;
#ifdef CONFIG_USE_QC_BMS_BATT_FORMAT
	struct bms_battery_data		*batt_data;
#endif

	struct delayed_work monitor_work;
	struct workqueue_struct *yl_adc_batt_wq;

	struct mutex			calc_soc_lock;

	struct power_supply     adc_batt_psy;
	struct power_supply     *charge_psy;
};

struct yl_adc_batt_capa_data {
	int capa;
	int volt;
};

enum charge_stat {
	CHARGE_STAT_READY = 0,
	CHARGE_STAT_CHARGING,
	CHARGE_STAT_DONE,
	CHARGE_STAT_FAULT,
};

static struct yl_adc_batt_chip *this_chip;

#define PON_VOLT_DELTA_DEFAULT 50
#define SOC_UNINITIALIZED         -99

#define LOWER      (3)
#define HIGER      (98)
#define CALIBRATE_CAPA(x)      ((100)*(x - LOWER))/(HIGER-LOWER)
//#define CALIBRATE_CAPA(x)      ((100+HIGER)*((x)-LOWER)/(100-LOWER))    ;;LOWER = 2 HIGER =2
static int soc_scaled(struct yl_adc_batt_chip *chip, int soc)
{
	pr_debug("soc_high_scaled = %d soc_low_scaled = %d \n", chip->soc_high_scaled, chip->soc_low_scaled);
	return (100*(soc - chip->soc_low_scaled))/(chip->soc_high_scaled - chip->soc_low_scaled);
}

static int read_batt_temp(struct yl_adc_batt_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("unable to read batt temp rc = %d \n", rc);
		return 0;
	}

	pr_debug("read batt temp= %lld \n", results.physical);
	return (int)results.physical;

}

#define BATT_MVOLT_DEFAULT      3800  /* 3800MV */
static int read_batt_mvol(struct yl_adc_batt_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("unable to read vbat rc = %d battery voltage use default value 3800mV\n", rc);
		//return 0;
		return BATT_MVOLT_DEFAULT;
	}

	pr_debug("read vbat= %lld \n", results.physical);
	return (int)results.physical/1000;

}

#ifdef CONFIG_USE_QC_BMS_BATT_FORMAT
static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);

	return soc;
}

static int lookup_soc_ocv_temp(struct yl_adc_batt_chip *chip, int ocv_uv, int batt_temp)
{
	int soc_ocv = 0, soc_cutoff = 0, soc_final = 0;
	//int fcc, acc, soc_uuc = 0, soc_acc = 0, iavg_ma = 0;

	soc_ocv = interpolate_pc(chip->batt_data->pc_temp_ocv_lut,
					batt_temp, ocv_uv / 1000);
	soc_cutoff = interpolate_pc(chip->batt_data->pc_temp_ocv_lut,
				batt_temp, chip->batt_data->cutoff_uv / 1000);
	/* disabled  soc_cutoff effct */
	soc_cutoff = 0;

	soc_final = (100 * (soc_ocv - soc_cutoff)) / (100 - soc_cutoff);

#if 0
	if (chip->batt_data->ibat_acc_lut) {
		/* Apply  ACC logic only if we discharging */
		if (!is_battery_charging(chip) && chip->current_now > 0) {

			iavg_ma = calculate_uuc_iavg(chip);

			fcc = interpolate_fcc(chip->batt_data->fcc_temp_lut,
								batt_temp);
			acc = interpolate_acc(chip->batt_data->ibat_acc_lut,
							batt_temp, iavg_ma);
			if (acc <= 0) {
				if (chip->last_acc)
					acc = chip->last_acc;
				else
					acc = fcc;
			}
			soc_uuc = ((fcc - acc) * 100) / acc;

			soc_uuc = adjust_uuc(chip, soc_uuc);

			soc_acc = soc_final - soc_uuc;

			pr_debug("fcc=%d acc=%d soc_final=%d soc_uuc=%d soc_acc=%d current_now=%d iavg_ma=%d\n",
				fcc, acc, soc_final, soc_uuc,
				soc_acc, chip->current_now / 1000, iavg_ma);

			soc_final = soc_acc;
			//chip->last_acc = acc;
		} else {
			/* charging - reset all the counters */
			chip->last_acc = 0;
			chip->iavg_num_samples = 0;
			chip->iavg_index = 0;
			chip->iavg_ma = 0;
			chip->prev_current_now = 0;
			chip->prev_soc_uuc = -EINVAL;
		}
	}
#endif

	soc_final = bound_soc(soc_final);

	pr_debug("soc_final=%d soc_ocv=%d soc_cutoff=%d ocv_uv=%u batt_temp=%d\n",
			soc_final, soc_ocv, soc_cutoff, ocv_uv, batt_temp);

	return soc_final;
}
#else

static struct yl_adc_batt_capa_data batt_capa_data[] = {
	{100, 4319},
	{95,  4257},
	{90,  4201},
	{85,  4146},
	{80,  4094},
	{75,  4050},
	{70,  3983},
	{65,  3942},
	{60,  3897},
	{55,  3864},
	{50,  3837},
	{45,  3815},
	{40,  3797},
	{35,  3779},
	{30,  3762},
	{25,  3746},
	{20,  3728},
	{15,  3705},
	{10,  3686},
	{9,   3684},
	{8,   3680},
	{7,   3671},
	{6,   3649},
	{5,   3612},
	{4,   3566},
	{3,   3508},
	{2,   3430},
	{1,   3313},
	{0,   3000},
};

static int lookup_soc_ocv(int volt)
{
	int crude_capa = 88;
	int loop_max_size = 0;
	int loop = 0;
	loop_max_size = sizeof(batt_capa_data)/sizeof(struct yl_adc_batt_capa_data) - 1;

	for (loop = 0; loop < loop_max_size; loop ++) {
		if (volt >= batt_capa_data[loop].volt)
			break;
	}


	if (loop == loop_max_size || loop == 0)
		crude_capa = batt_capa_data[loop].capa;
	else {
		pr_debug("loop caps = %d, loop-1 capa = %d,loop-1.volt = %d \n", batt_capa_data[loop].capa,
				batt_capa_data[loop-1].capa, batt_capa_data[loop-1].volt);
		crude_capa = batt_capa_data[loop - 1].capa - ((batt_capa_data[loop - 1].capa - batt_capa_data[loop].capa) *
				(batt_capa_data[loop - 1].volt - volt)/(batt_capa_data[loop - 1].volt - batt_capa_data[loop].volt));
	}

	pr_debug( "volt = %d, crude_capa = %d ===\n", volt, crude_capa);

	return crude_capa;
}
#endif

static int yl_adc_batt_volt_temp_to_capa(struct yl_adc_batt_chip *chip, int volt, int temp)
{
	int capa_ocv = 0;

	#ifdef CONFIG_USE_QC_BMS_BATT_FORMAT
	capa_ocv = lookup_soc_ocv_temp(chip, volt*1000, temp);
	#else
	capa_ocv = lookup_soc_ocv(volt);
	#endif

	pr_debug( "capa_ocv = %d\n", capa_ocv);

	return capa_ocv;
}

static long long get_wall_time(void)
{
	struct timespec     wall_time;
	getnstimeofday(&wall_time);
	return wall_time.tv_sec;
}

static int yl_adc_batt_get_batt_ave_mvolt(struct yl_adc_batt_chip *chip)
{
	int loop_num = 0;
	int mvolt = 0;
	int mvolt_min = 0;
	int mvolt_max = 0;
	int mvolt_sum = 0;

	//mvolt = get_yl_pm8916_batt_mvol();
	mvolt = read_batt_mvol(chip);

	mvolt_min = mvolt;
	mvolt_max = mvolt;
	mvolt_sum += mvolt;
	for (loop_num = 0; loop_num < 5; loop_num++) {
		mvolt = read_batt_mvol(chip);
		mvolt_sum += mvolt;
		if (mvolt_max < mvolt)
			mvolt_max = mvolt;
		if (mvolt_min > mvolt)
			mvolt_min = mvolt;
	}
	mvolt = (mvolt_sum - mvolt_min - mvolt_max) / 4;
	chip->batt_crude_volt = mvolt;
	pr_debug("batt_volt_mv = %d, yl_get_bk_level = %d, yl_get_lcd_status = %d\n", chip->batt_crude_volt, yl_get_bk_level, yl_get_lcd_status);
	pr_debug("battery average voltage : %d mvolt sum  = %d , mvolt max = %d , mvolt min = %d \n", mvolt, mvolt_sum, mvolt_max, mvolt_min);
	return mvolt;
}

static int yl_adc_batt_force_en_charging(struct yl_adc_batt_chip *chip, bool en)
{
	union power_supply_propval ret = {0,};
	int rc = 0;

	if (chip->charge_psy == NULL) {
		chip->charge_psy = power_supply_get_by_name(chip->charge_psy_name);
		if (!chip->charge_psy) {
			dev_err(chip->dev, "battery supply not found.\n");
		}
	} else {
		ret.intval = en;
		rc = chip->charge_psy->set_property(chip->charge_psy,
			POWER_SUPPLY_PROP_YL_CRTL_CHG_INTERFACE	, &ret);
		if (rc)
			dev_err(chip->dev, "Unable to set charge enable rc=%d\n", rc);
	}

	return rc;
}


#define LCD_OPEN_BATT_MVOLT_DELTA      50
#define LCD_CLOSE_BATT_MVOLT_DELTA      30
static int yl_adc_batt_evaluate_batt_mvolt(struct yl_adc_batt_chip *chip)
{
	int evaluate_mvolt = 0;
	int mvolt = 0;
	int batt_mvolt_offset = 0;

	if (!chip->vbus_present) {
		mvolt = yl_adc_batt_get_batt_ave_mvolt(chip);
	} else {
		yl_adc_batt_force_en_charging(chip, false);
		//disable charging
		msleep(10);
		mvolt = yl_adc_batt_get_batt_ave_mvolt(chip);
		yl_adc_batt_force_en_charging(chip, true);
		//enable charging
	}
	
	if (yl_get_lcd_status) {
	/* LCD display is open */
		//batt_mvolt_offset = LCD_OPEN_BATT_MVOLT_DELTA + BK_LEVEL_MVOLT_OFFSET(yl_get_bk_level);
		batt_mvolt_offset = chip->lcd_open_delta_mv + (yl_get_bk_level / chip->bk_level_offset_step);
	} else {
	/* LCD display is close */
		//batt_mvolt_offset = LCD_CLOSE_BATT_MVOLT_DELTA;
		batt_mvolt_offset = chip->lcd_close_delta_mv;
	}

	evaluate_mvolt = mvolt + batt_mvolt_offset;
	pr_debug("evaluate_mvolt = %d, batt_mvolt_offset = %d, \n", evaluate_mvolt, batt_mvolt_offset);
	return evaluate_mvolt;
}

#define BATT_CUTOFF_MVOLT       2600
#define SHUTDOWN_V_PON_SOC_STEP       10
static int initialized_to_sys_capa(struct yl_adc_batt_chip *chip)
{
	int read_ocv;
	int read_soc;
	int calculate_soc;
	int rc;

	rc = yl_read_shutdown_ocv_soc(&read_ocv, &read_soc);
	if(rc < 0)
		chip->shutdown_soc_invalid = true;

	chip->shutdown_soc = read_soc;
	calculate_soc = chip->to_sys_capa;

	if(chip->warm_reset){
		if(!chip->shutdown_soc_invalid){
			/* pon soc > shutdown soc + 10 and usb is present (is charging) */
			if (((calculate_soc - read_soc) > SHUTDOWN_V_PON_SOC_STEP) && (true == chip->vbus_present)) {
				chip->to_sys_capa = read_soc + 1;
				/* shutdown soc > pon soc +10 and usb is absent (is discharging) */
			} else if (((read_soc - calculate_soc) > SHUTDOWN_V_PON_SOC_STEP) && (false == chip->vbus_present)) {
				chip->to_sys_capa = read_soc - 1;
			} else {
				chip->to_sys_capa = read_soc;
			}
			chip->batt_volt = read_ocv;
			pr_err("warm_reset: using shutdown SOC\n");
		} else {
			pr_err("using PON SOC\n");
		}
	} else {
		if(!chip->shutdown_soc_invalid &&
				(abs(read_soc - calculate_soc) < SHUTDOWN_V_PON_SOC_STEP)){
			chip->to_sys_capa = read_soc;
			chip->batt_volt = read_ocv;
			pr_err("using shutdown SOC\n");
		} else {
			pr_err("else . using PON SOC\n");
		}
	}

	pr_info("shutdown_soc_invalid = %d  read_soc = %d calculate_soc = %d, chip->to_sys_capa = %d\n",
			chip->shutdown_soc_invalid, read_soc, calculate_soc, chip->to_sys_capa);


	return chip->to_sys_capa;
}

#define SOC_TOO_DIFF        20
#define EXPIRE_TIME         120 /* 120 secs */
#define SOC_LOW_EXPIRE_TIME         60 /* 60 secs */
#define BATT_CRUDE_SOC_LOW      20
#define BATT_MVOLT_TO_LOW        3100 /* 3100MV */
#define BATT_MVOLT_DEFAULT       3800 /* 3800MV */
static int calculated_to_sys_capa(struct yl_adc_batt_chip *chip)
{
	static int first_time = 0;
	static long long last_update_tm_sec = 0;
	static int charging_need_to_decrease = 0;
	long long the_past_time = 0;
	long long now_tm_sec = 0;
	long long expire_time_sec = SOC_LOW_EXPIRE_TIME;
	int time_to_soc = 0;
	int soc_temp = 0;

	mutex_lock(&chip->calc_soc_lock);
	pm_stay_awake(chip->dev);

	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	if ((BATT_CRUDE_SOC_LOW < chip->batt_crude_capa) && (chip->vbus_present == false))
		expire_time_sec = EXPIRE_TIME;

	chip->batt_temp = read_batt_temp(chip);
	chip->batt_volt  = yl_adc_batt_evaluate_batt_mvolt(chip);
	if (chip->batt_volt < 0) {
		pr_err("failt fo read battery voltage : %d \n", chip->batt_volt);
		return 50;
	}

	now_tm_sec = get_wall_time();
	if (first_time == 0) {
		first_time = 1;
		last_update_tm_sec = now_tm_sec + expire_time_sec;
		chip->batt_crude_capa = initialized_to_sys_capa(chip);
		if (SOC_UNINITIALIZED == chip->to_sys_capa) {
			pr_err("battery capacity not yet initialized!\n");
			//chip->to_sys_capa = yl_adc_batt_volt_to_capa(chip->batt_volt);
			chip->to_sys_capa = yl_adc_batt_volt_temp_to_capa(chip, chip->batt_volt, chip->batt_temp);
			chip->batt_crude_capa = chip->to_sys_capa;
		}
	} else {
		if (3 > first_time) {
			first_time++;
			last_update_tm_sec = now_tm_sec + expire_time_sec;
		}
		//chip->batt_crude_capa = yl_adc_batt_volt_to_capa(chip->batt_volt);
		chip->batt_crude_capa = yl_adc_batt_volt_temp_to_capa(chip, chip->batt_volt, chip->batt_temp);
	}

	//pr_info("before to_sys_capa = %d, batt_crude_capa = %d\n", chip->to_sys_capa, chip->batt_crude_capa);
	if (now_tm_sec > last_update_tm_sec ) {
		the_past_time = now_tm_sec -last_update_tm_sec + expire_time_sec;
		last_update_tm_sec = now_tm_sec + expire_time_sec;


		if (chip->vbus_present == false){
		/* charger unplugged */
			time_to_soc = chip->to_sys_capa - ((int)the_past_time/(int)expire_time_sec);
			soc_temp = max(time_to_soc, chip->batt_crude_capa);

			chip->charge_stat = CHARGE_STAT_READY;

			if (chip->to_sys_capa > soc_temp) {
				chip->to_sys_capa = soc_temp;
				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;
			}

		} else {
		/* charger plugged */
			time_to_soc = chip->to_sys_capa + ((int)the_past_time/(int)expire_time_sec);
			soc_temp = min(time_to_soc, chip->batt_crude_capa);

			if (chip->to_sys_capa > soc_temp) {
				++charging_need_to_decrease;
				if (charging_need_to_decrease > 3)
					chip->to_sys_capa--;

				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;

			} else if(chip->to_sys_capa < soc_temp){
				charging_need_to_decrease = 0;
				chip->to_sys_capa = soc_temp;
				if ((chip->batt_crude_capa - chip->to_sys_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa++;
			}

			if (CHARGE_STAT_DONE == chip->charge_stat && (100 > chip->to_sys_capa))
				chip->to_sys_capa++;

		}
	} else if (now_tm_sec < last_update_tm_sec - expire_time_sec) {
		last_update_tm_sec = now_tm_sec + expire_time_sec;
	}

	if (BATT_MVOLT_TO_LOW > chip->batt_crude_volt) {
		chip->batt_crude_volt = yl_adc_batt_get_batt_ave_mvolt(chip);
		if (BATT_MVOLT_TO_LOW > chip->batt_crude_volt)
			chip->to_sys_capa = 0;
	}

	if (chip->last_to_sys_capa != chip->to_sys_capa) {
		yl_backup_ocv_soc(chip->batt_volt, chip->to_sys_capa);
		chip->last_to_sys_capa = chip->to_sys_capa;
		pr_debug("backup_soc chip->last_to_sys_capa = %d, chip->to_sys_capa = %d\n", chip->last_to_sys_capa, chip->to_sys_capa);
	}

	pm_relax(chip->dev);
	mutex_unlock(&chip->calc_soc_lock);

	pr_debug("now_tm_sec = %lld, last_update_tm_sec = %lld, the_past_time = %lld\n", now_tm_sec, last_update_tm_sec, the_past_time);
	pr_debug("after last_to_sys_capa = %d, to_sys_capa = %d, batt_crude_capa = %d, the_past_time = %lld\n",
			chip->last_to_sys_capa,chip->to_sys_capa, chip->batt_crude_capa, the_past_time);

	return chip->to_sys_capa;

}


/* power supply segment */
static enum power_supply_property yl_adc_batt_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};


static int yl_adc_batt_get_prop_batt_present(struct yl_adc_batt_chip *chip)
{
	chip->batt_present = is_battery_present();

	return chip->batt_present;
}

static int yl_adc_batt_get_prop_charge_full_design(struct yl_adc_batt_chip *chip)
{
	union power_supply_propval ret = {0, };

	ret.intval = chip->capa_design_mAH;
	return ret.intval;

}

static int yl_adc_batt_get_prop_voltage_max_design(struct yl_adc_batt_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = chip->voltage_max_uv;

	return ret.intval;
}

static int yl_adc_batt_get_prop_voltage_now(struct yl_adc_batt_chip *chip)
{
	union power_supply_propval ret = {0, };

	ret.intval = 1000 * read_batt_mvol(chip);
	pr_debug("yl==========.batr_volt = %d \n", ret.intval);

	return ret.intval;
}


static int yl_adc_batt_get_prop_batt_temp(struct yl_adc_batt_chip *chip)
{
	union power_supply_propval ret = {0, };

	//ret.intval = get_yl_pm8916_batt_temp();
	ret.intval =  read_batt_temp(chip);

	pr_debug("report_capa ret.batr_temp = %d \n", ret.intval);
	return ret.intval;
}


static int yl_adc_batt_get_prop_batt_capa(struct yl_adc_batt_chip *chip)
{
	int batt_capa;


	if (SOC_UNINITIALIZED == chip->to_sys_capa) {
		pr_err("battery capacity not yet initialized!\n");
		return 88;
	}

	//batt_capa = CALIBRATE_CAPA(chip->to_sys_capa);
	batt_capa = soc_scaled(chip, chip->to_sys_capa);
	if ( batt_capa > 100 )
		batt_capa = 100;
	if ( batt_capa < 0 )
		batt_capa = 0;

	chip->report_soc = batt_capa;

	pr_debug("report_capa ret.batt_capa = %d \n", batt_capa);
	return batt_capa;
}

static int yl_adc_batt_get_property(struct power_supply *psy,
										enum power_supply_property prop,
										union power_supply_propval *val)
{
	struct yl_adc_batt_chip *chip = container_of(psy, struct yl_adc_batt_chip, adc_batt_psy);

	switch (prop) {
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = yl_adc_batt_get_prop_batt_present(chip);
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = yl_adc_batt_get_prop_batt_capa(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = yl_adc_batt_get_prop_charge_full_design(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = yl_adc_batt_get_prop_voltage_max_design(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = yl_adc_batt_get_prop_voltage_now(chip);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = yl_adc_batt_get_prop_batt_temp(chip);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int yl_adc_batt_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int yl_adc_batt_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct yl_adc_batt_chip *chip = container_of(psy,
				struct yl_adc_batt_chip, adc_batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		chip->charge_stat = val->intval;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


#define UPDATE_HEART_PERIOD_NORMAL_MS      61000
static void yl_adc_batt_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct yl_adc_batt_chip *chip = container_of(dwork,
				struct yl_adc_batt_chip, monitor_work);

	int update_period = UPDATE_HEART_PERIOD_NORMAL_MS;

	calculated_to_sys_capa(chip);

	queue_delayed_work(chip->yl_adc_batt_wq,
		&chip->monitor_work, msecs_to_jiffies(update_period));

	pr_info("last_to_sys_capa = %d, to_sys_capa = %d, batt_crude_capa = %d, batt_volt = %d, batt_temp = %d , reprot_soc = %d, chg_stat=%d\n",
			chip->last_to_sys_capa,chip->to_sys_capa, chip->batt_crude_capa, chip->batt_volt, chip->batt_temp, chip->report_soc, chip->charge_stat);

	return;
}

#ifdef CONFIG_USE_QC_BMS_BATT_FORMAT
static int64_t read_battery_id(struct yl_adc_batt_chip *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}

	return result.physical;
}

static int set_battery_data(struct yl_adc_batt_chip *chip)
{
	int64_t battery_id;
	int rc = 0;
	struct bms_battery_data *batt_data;
	struct device_node *node;

	battery_id = read_battery_id(chip);
	if (battery_id < 0) {
		pr_err("cannot read battery id err = %lld\n", battery_id);
		return battery_id;
	}

	node = of_find_node_by_name( chip->dev->of_node,
					"qcom,battery-data");
	if (!node) {
			pr_err("No available batterydata\n");
			return -EINVAL;
	}

	batt_data = devm_kzalloc(chip->dev,
			sizeof(struct bms_battery_data), GFP_KERNEL);
	if (!batt_data) {
		pr_err("Could not alloc battery data\n");
		return -EINVAL;
	}

	batt_data->fcc_temp_lut = devm_kzalloc(chip->dev,
		sizeof(struct single_row_lut), GFP_KERNEL);
	batt_data->pc_temp_ocv_lut = devm_kzalloc(chip->dev,
			sizeof(struct pc_temp_ocv_lut), GFP_KERNEL);
	batt_data->rbatt_sf_lut = devm_kzalloc(chip->dev,
				sizeof(struct sf_lut), GFP_KERNEL);
	batt_data->ibat_acc_lut = devm_kzalloc(chip->dev,
				sizeof(struct ibat_temp_acc_lut), GFP_KERNEL);

	batt_data->max_voltage_uv = -1;
	batt_data->cutoff_uv = -1;
	batt_data->iterm_ua = -1;

	/*
	 * if the alloced luts are 0s, of_batterydata_read_data ignores
	 * them.
	 */
	rc = of_batterydata_read_data(node, batt_data, battery_id);
	if (rc || !batt_data->pc_temp_ocv_lut
		|| !batt_data->fcc_temp_lut
		|| !batt_data->rbatt_sf_lut) {
		pr_err("battery data load failed\n");
		devm_kfree(chip->dev, batt_data->fcc_temp_lut);
		devm_kfree(chip->dev, batt_data->pc_temp_ocv_lut);
		devm_kfree(chip->dev, batt_data->rbatt_sf_lut);
		devm_kfree(chip->dev, batt_data->ibat_acc_lut);
		devm_kfree(chip->dev, batt_data);
		return rc;
	}

	if (batt_data->pc_temp_ocv_lut == NULL) {
		pr_err("temp ocv lut table has not been loaded\n");
		devm_kfree(chip->dev, batt_data->fcc_temp_lut);
		devm_kfree(chip->dev, batt_data->pc_temp_ocv_lut);
		devm_kfree(chip->dev, batt_data->rbatt_sf_lut);
		devm_kfree(chip->dev, batt_data->ibat_acc_lut);
		devm_kfree(chip->dev, batt_data);

		return -EINVAL;
	}

	/* check if ibat_acc_lut is valid */
	if (!batt_data->ibat_acc_lut->rows) {
		pr_info("ibat_acc_lut not present\n");
		devm_kfree(chip->dev, batt_data->ibat_acc_lut);
		batt_data->ibat_acc_lut = NULL;
	}

#if 0
	/* Override battery properties if specified in the battery profile */
	if (batt_data->max_voltage_uv >= 0)
		chip->dt.cfg_max_voltage_uv = batt_data->max_voltage_uv;
	if (batt_data->cutoff_uv >= 0)
		chip->dt.cfg_v_cutoff_uv = batt_data->cutoff_uv;
#endif

	chip->batt_data = batt_data;

	pr_info("battery type : %s\n", batt_data->battery_type);

	return 0;
}
#endif


static int yl_adc_batt_parse_dt(struct yl_adc_batt_chip *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing \n");
		return -EINVAL;
	}


	rc = of_property_read_string(node, "yl,adc_batt_psy_name", &chip->adc_batt_psy_name);
	if (rc) {
		dev_err(chip->dev, "Failed to read adc_batt_psy_name\n");
		chip->adc_batt_psy_name = "yl_adc_battery_d";
		rc = 0;
	}

	rc = of_property_read_string(node, "yl,charge_psy_name", &chip->charge_psy_name);
	if (rc){
		dev_err(chip->dev, "Failed to read charge_psy_name\n");
		chip->charge_psy_name = "battery";
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,pon_volt_delta", &chip->pon_volt_delta);
	if (rc < 0){
		chip->pon_volt_delta = PON_VOLT_DELTA_DEFAULT;
		dev_err(chip->dev,"fail ===== chip->pon_volt_delta = %d \n", chip->pon_volt_delta);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,voltage_max_uv", &chip->voltage_max_uv);
	if (rc < 0){
		chip->voltage_max_uv = 4350000;
		dev_err(chip->dev,"fail ===== chip->voltage_max_uv = %d \n", chip->voltage_max_uv);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,capa_design_mAH", &chip->capa_design_mAH);
	if (rc < 0){
		chip->capa_design_mAH = 2500;
		dev_err(chip->dev,"fail ===== chip->capa_design_mAH = %d \n", chip->capa_design_mAH);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,lcd_open_delta_mv", &chip->lcd_open_delta_mv);
	if (rc < 0){
		chip->lcd_open_delta_mv = LCD_OPEN_BATT_MVOLT_DELTA;
		dev_err(chip->dev,"fail ===== chip->lcd_open_delta_mv = %d \n", chip->lcd_open_delta_mv);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,lcd_close_delta_mv", &chip->lcd_close_delta_mv);
	if (rc < 0){
		chip->lcd_close_delta_mv = LCD_CLOSE_BATT_MVOLT_DELTA;
		dev_err(chip->dev,"fail ===== chip->lcd_close_delta_mv = %d \n", chip->lcd_close_delta_mv);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,bk_level_offset_step", &chip->bk_level_offset_step);
	if (rc < 0){
		chip->bk_level_offset_step = 5;
		dev_err(chip->dev,"fail ===== chip->bk_level_offset_step = %d \n", chip->bk_level_offset_step);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,soc_high_scaled", &chip->soc_high_scaled);
	if (rc < 0){
		chip->soc_high_scaled = 99;
		dev_err(chip->dev,"fail ===== chip->soc_high_scaled = %d \n", chip->soc_high_scaled);
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,soc_low_scaled", &chip->soc_low_scaled);
	if (rc < 0){
		chip->soc_low_scaled = 0;
		dev_err(chip->dev,"fail ===== chip->soc_low_scaled = %d \n", chip->soc_low_scaled);
		rc = 0;
	}
	pr_debug("device tree info. ok \n");

	return rc;
}

static int yl_adc_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct yl_adc_batt_chip *chip;

	//dev_err(&pdev->dev, "enter ==== yl_adc_batt_probe\n");
	chip = devm_kzalloc(&pdev->dev, sizeof(struct yl_adc_batt_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	rc = yl_adc_batt_parse_dt(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to parse DT nodes\n");
		goto unregister_batt_psy;
	}

	device_init_wakeup(chip->dev, 1);


	chip->shutdown_soc_invalid = false;
	rc = qpnp_pon_is_warm_reset();
	if (rc < 0) {
		pr_err("Error reading warm reset status rc=%d\n", rc);
		goto unregister_batt_psy;
	}
	chip->warm_reset = !!rc;

	//dev_info(&pdev->dev, "enter chip->pon_volt = %d =chip->to_sys_capa = %d==\n", chip->pon_volt, chip->to_sys_capa);

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "yl_adc");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc_dev prop missing rc = %d \n", rc);
		goto unregister_batt_psy;
	}

#ifdef CONFIG_USE_QC_BMS_BATT_FORMAT
	/* read battery-id and select the battery profile */
	rc = set_battery_data(chip);
	if (rc) {
		pr_err("Unable to read battery data %d\n", rc);
		goto unregister_batt_psy;
	}
#endif

	chip->batt_temp = read_batt_temp(chip);
	chip->pon_volt = pon_batt_volt;
	if (BATT_CUTOFF_MVOLT > chip->pon_volt)
		chip->to_sys_capa = SOC_UNINITIALIZED;
	else {
		//chip->to_sys_capa = yl_adc_batt_volt_to_capa(chip->pon_volt + chip->pon_volt_delta);
		chip->to_sys_capa = yl_adc_batt_volt_temp_to_capa(chip, chip->pon_volt + chip->pon_volt_delta, chip->batt_temp);
	}

	/* register battery power supply */
	chip->adc_batt_psy.name		= chip->adc_batt_psy_name;
	chip->adc_batt_psy.type		= POWER_SUPPLY_TYPE_YL_BATTERY;
	chip->adc_batt_psy.get_property	= yl_adc_batt_get_property;
	chip->adc_batt_psy.property_is_writeable = yl_adc_batt_is_writeable;
	chip->adc_batt_psy.set_property	= yl_adc_batt_set_property;
	chip->adc_batt_psy.properties	= yl_adc_batt_properties;
	chip->adc_batt_psy.num_properties  = ARRAY_SIZE(yl_adc_batt_properties);

	rc = power_supply_register(chip->dev, &chip->adc_batt_psy);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to register adc_batt_psy rc = %d\n", rc);
		goto unregister_batt_psy;
	}

	mutex_init(&chip->calc_soc_lock);

	chip->yl_adc_batt_wq = create_singlethread_workqueue("yl_adc_batt");
	if (!chip->yl_adc_batt_wq ) {
		dev_err(&pdev->dev, "create workqueue faild.\n");
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&chip->monitor_work, yl_adc_batt_work);

	this_chip = chip;

	queue_delayed_work(chip->yl_adc_batt_wq,
		&chip->monitor_work, msecs_to_jiffies(5000));

	//dev_err(&pdev->dev, "success ==== yl_adc_batt_probe\n");

	return 0;

unregister_batt_psy:

	return rc;
}

static int yl_adc_batt_remove(struct platform_device *pdev)
{
	int rc = 0;
	struct yl_adc_batt_chip *chip = platform_get_drvdata(pdev);

	mutex_destroy(&chip->calc_soc_lock);
	destroy_workqueue(chip->yl_adc_batt_wq);

	return rc;
}

static int yl_adc_batt_suspend(struct device *dev)
{
	int rc = 0;
	//struct yl_adc_batt_chip *chip = this_chip;

	return rc;
}


static int yl_adc_batt_resume(struct device *dev)
{
	int rc = 0;
	struct yl_adc_batt_chip *chip = this_chip;
	dev_dbg(chip->dev, " yl_adc_batt_resume\n");

	calculated_to_sys_capa(chip);

	return rc;
}

static const struct dev_pm_ops yl_adc_batt_pm_ops = {
	.resume = yl_adc_batt_resume,
	.suspend = yl_adc_batt_suspend,
};

static struct of_device_id yl_adc_batt_match_table[] = {
	{.compatible = "yl,yl_adc_battery",},
	{},
};



static struct platform_driver yl_adc_batt_driver = {
	.driver = {
		.name = "yl_adc_battery",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(yl_adc_batt_match_table),
		.pm = &yl_adc_batt_pm_ops,
	},
	.probe = yl_adc_batt_probe,
	.remove = yl_adc_batt_remove,
};

module_platform_driver(yl_adc_batt_driver);


MODULE_DESCRIPTION("yulong adc  battery driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("LASER");

