/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR        Note
 *    1.0		  2010-01-05			WenFS    only support mulititouch	Wenfs 2010-10-01
 *    2.0          2011-09-05                   Duxx      Add touch key, and project setting update, auto CLB command
 *    3.0		  2011-09-09			Luowj
 *

 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include "ft5x06_ts_fw.h" /* modidy firmware you will use*/
#include <linux/input/touchscreen_yl.h>
#include <linux/input/ft5x0x_ts.h>
#include <linux/pm_runtime.h> //add by anxufeng for runtime suspend

#ifdef FT5336_DOWNLOAD
#include "ft5336_download_lib.h"
static struct i2c_client *g_i2c_client = NULL;
#endif

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define TW_TAG  "TouchScreen:"
#define TW_DEBUG //调试时打开，传CC时关闭
#ifdef TW_DEBUG
#define TW_DBG(fmt, ...) \
			if(debug_flag)printk(KERN_ERR TW_TAG pr_fmt(fmt),##__VA_ARGS__)
#else
#define TW_DBG(fmt, ...) do{ \
		 }while(0)
#endif

#define BUF_LENGTH 128
#define resume_mode     0
#define suspend_mode    1

static struct i2c_client *this_client;
static int touch_flag;
static int debug_flag;
static int sleep_flag = suspend_mode;
static int SCREEN_MAX_X;
static int SCREEN_MAX_Y;


#if CFG_SUPPORT_TOUCH_KEY
static int key_flags = 0;
static int DEBOUNCE;
static bool inDebounce;
#endif
/* yulong end */

int result = 0;
unsigned int file_version;
static int Read_boot_tp_ID(void);

void TW_irq_disable(struct ft5x0x_ts_data *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disabled) {
		ts->irq_is_disabled = true;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void TW_irq_enable(struct ft5x0x_ts_data *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disabled) {
		enable_irq(ts->client->irq);
		ts->irq_is_disabled = false;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

struct virtual_keys_button virtual_keys_button[] = {
			[0] = {
			.x = 321,
			.y = 854,
			.width = 160,
			.height = 50,
			.code = KEY_MENU,
			},
			[1] = {
			.x = 161,
			.y = 854,
			.width = 160,
			.height = 50,
			.code = KEY_HOME,
			},
			[2] = {
			.x = 0,
			.y = 854,
			.width = 160,
			.height = 50,
			.code = KEY_BACK,
			},
};

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata

Input	:	*rxdata
                     *length

Output	:	ret

function	:

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	  int ret;

	  struct i2c_msg msgs[] = {
		 {
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		 },
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		    pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	  int ret;

	  struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

//msleep(1);
    ret = i2c_transfer(this_client->adapter, msg, 1);
    if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	  return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0)
    {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }

    return 0;
}


/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
    int ret;
    u8 buf[2];
    struct i2c_msg msgs[2];
    //
    buf[0] = addr;    //register address

    msgs[0].addr = this_client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = buf;
    msgs[1].addr = this_client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = buf;

    ret = i2c_transfer(this_client->adapter, msgs, 2);
    if (ret < 0)
        pr_err("msg %s i2c read error: %d\n", __func__, ret);
    *pdata = buf[0];
    return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/

static unsigned char ft5x0x_read_fw_ver(void)
{
    unsigned char ver;
    ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
    return(ver);
}

#if	FOCALTECH_COB
static int ft5x0x_read_tp_type(void)
{
	unsigned char type, pin_status;
	int i;

	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	if(data->pdata->get_id_pin)
		pin_status = data->pdata->get_id_pin();
	else
	{
	    	return (-1);
	}

	for(i=0; i < ARRAY_SIZE(PIN_ID); i++)
	{
		if(pin_status == PIN_ID[i].pin)
		{
			type = PIN_ID[i].id;
			printk("ft5x0x_read_tp_type cob: 0x%x",type);
	    		return (type);
		}
	}

	return (-1);
}
#ifndef FT5336_DOWNLOAD
static int ft5x0x_read_tp_type_(void)
{

    unsigned char type;
    int ret;
    type=0;
    ret=ft5x0x_read_reg(0xA8, &type);
    if(ret < 0) return -1;
    printk("ft5x0x_read_tp_type_ cob: 0x%x\n",type);
    return (type);
}
#endif

#else
static int ft5x0x_read_tp_type(void)
{
	unsigned char type;
	int ret;

	type = 0;
	ret = ft5x0x_read_reg(0xA8, &type);
	if (ret < 0)
		return ret;
	else {
		if (type == 0xa8)
			type = Read_boot_tp_ID();
		return type;
	}
}
#endif

static void ft5x0x_reset(int ms)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);//tanliyu add
	if(data->pdata->reset)
		data->pdata->reset(ms);
}

#if 1 //upgrade related

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

typedef struct _FTS_CTP_PROJECT_SETTING_T
{
    unsigned char uc_i2C_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID
}FTS_CTP_PROJECT_SETTING_T;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}
#endif
#if 1
/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)





{
    int ret;

    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk(KERN_ERR "[FTS]i2c_read_interface error\n");

        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;

[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;



[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk(KERN_ERR "[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}
#endif
/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{

    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}
/***add for download***/
#ifdef FT5336_DOWNLOAD
int ft5x0x_download_i2c_Read(unsigned char *writebuf,
		    int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_download_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = 0x38,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(g_i2c_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&g_i2c_client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

#endif


/***add for download****/

#ifdef CONFIG_OF

/**********************************************************************/
#define FOCALTECH_TS_GPIO_RESET	        16
#define FOCALTECH_TS_GPIO_IRQ		17
#define FOCALTECH_TS_GPIO_ID1   38
#define FOCALTECH_TS_GPIO_ID2   49
#define FTP_PWR_EN      1  		/* the tw power enable switch */
#define FTP_SLEEP_PWR_EN 0		/* the tw sleep power enable switch */
#if FTP_PWR_EN
struct regulator *ftp_ts_vdd;
struct regulator *ftp_ts_vcc_i2c;
#define FTP_VDD		"vdd"
#define FTP_VCC_IO	"vcc_i2c"
#endif
/**********************************************************************/
/*static int focaltech_power_init(bool on)
{
#if FTP_PWR_EN
	int rc;
	if (!on)
		goto pwr_deinit;
	ftp_ts_vdd = regulator_get(&this_client->dev, FTP_VDD);
	if (IS_ERR(ftp_ts_vdd)) {
		rc = PTR_ERR(ftp_ts_vdd);
		pr_err("Touchscreen Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(ftp_ts_vdd) > 0) {
		rc = regulator_set_voltage(ftp_ts_vdd, 2600000,3300000);
		if (rc) {
			pr_err("Touchscreen Regulator set failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}
	ftp_ts_vcc_i2c = regulator_get(&this_client->dev, FTP_VCC_IO);
	if (IS_ERR(ftp_ts_vcc_i2c)) {
		rc = PTR_ERR(ftp_ts_vcc_i2c);
		pr_err("Touchscreen Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}
	if (regulator_count_voltages(ftp_ts_vcc_i2c) > 0) {
		rc = regulator_set_voltage(ftp_ts_vcc_i2c, 1800000,1800000);
		if (rc) {
			pr_err("Touchscreen Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	return 0;
reg_vcc_i2c_put:
	regulator_put(ftp_ts_vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(ftp_ts_vdd) > 0)
		regulator_set_voltage(ftp_ts_vdd, 0, 3300000);
reg_vdd_put:
	regulator_put(ftp_ts_vdd);
	return rc;
pwr_deinit:
	if (regulator_count_voltages(ftp_ts_vdd) > 0)
		regulator_set_voltage(ftp_ts_vdd, 0, 3300000);
	regulator_put(ftp_ts_vdd);
	if (regulator_count_voltages(ftp_ts_vcc_i2c) > 0)
		regulator_set_voltage(ftp_ts_vcc_i2c, 0, 1800000);
	regulator_put(ftp_ts_vcc_i2c);
#endif
	return 0;
}
static int focaltech_ts_power(int on)
{
#if FTP_PWR_EN
	int rc;
	if (!on)
		goto power_off;
	rc = regulator_enable(ftp_ts_vdd);
	if (rc) {
		pr_err("Touchscreen Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}
	rc = regulator_enable(ftp_ts_vcc_i2c);
	if (rc) {
		pr_err("Touchscreen Regulator vdd_i2c enable failed rc=%d\n", rc);
		regulator_disable(ftp_ts_vdd);
	}
	return rc;
power_off:
	rc = regulator_disable(ftp_ts_vdd);
	if (rc) {
		pr_err("Touchscreen Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}
	rc = regulator_disable(ftp_ts_vcc_i2c);
	if (rc) {
		pr_err("Touchscreen Regulator vdd_i2c disable failed rc=%d\n", rc);
		//regulator_enable(ftp_ts_vdd);
	}
	return rc;
#endif
	return 0;
}

static int focaltech_ts_gpio_init(bool on)
{
	int ret = 0;
	if(!on)
		goto ftp_gpio_deinit;
	ret = gpio_request(FOCALTECH_TS_GPIO_RESET, "focaltech-ts-reset");
	if (ret){
		pr_err("TOUCH:YLLOG:%s: Failed to request GPIO %d\n",__func__, FOCALTECH_TS_GPIO_RESET);
		return ret;
	}
	ret = gpio_direction_output(FOCALTECH_TS_GPIO_RESET, 0);
		ret = gpio_request(FOCALTECH_TS_GPIO_IRQ, "focaltech-ts-irq");
		if (ret){
			pr_err("TOUCH:%s: Failed to request GPIO %d\n",__func__, FOCALTECH_TS_GPIO_IRQ);
			gpio_free(FOCALTECH_TS_GPIO_RESET);
			return ret;
		}
		gpio_direction_input(FOCALTECH_TS_GPIO_IRQ);
		ret = gpio_request(FOCALTECH_TS_GPIO_ID1, "focaltech-ts-id1");
		if (ret){
			pr_err("TOUCH:%s: Failed to request GPIO %d\n",__func__, FOCALTECH_TS_GPIO_ID1);
			gpio_free(FOCALTECH_TS_GPIO_RESET);
			gpio_free(FOCALTECH_TS_GPIO_IRQ);
			return ret;
		}
		gpio_direction_input(FOCALTECH_TS_GPIO_ID1);
		ret = gpio_request(FOCALTECH_TS_GPIO_ID2, "focaltech-ts-id1");
		if (ret){
			pr_err("TOUCH:%s: Failed to request GPIO %d\n",__func__, FOCALTECH_TS_GPIO_ID2);
			gpio_free(FOCALTECH_TS_GPIO_RESET);
			gpio_free(FOCALTECH_TS_GPIO_IRQ);
			gpio_free(FOCALTECH_TS_GPIO_ID1);
			return ret;
		}
		gpio_direction_input(FOCALTECH_TS_GPIO_ID2);
	return 0;
ftp_gpio_deinit:
	gpio_free(FOCALTECH_TS_GPIO_RESET);
	gpio_free(FOCALTECH_TS_GPIO_IRQ);
	gpio_free(FOCALTECH_TS_GPIO_ID1);
	gpio_free(FOCALTECH_TS_GPIO_ID2);
	return 0;
}
*/
static unsigned char focaltech_ts_get_id_pin(void)
{
	unsigned char  pin_status=0x00;
	unsigned char  pin_status1=0x00;
	unsigned char  pin_status2=0x00;
	pin_status1=gpio_get_value(FOCALTECH_TS_GPIO_ID1);
	pin_status2=gpio_get_value(FOCALTECH_TS_GPIO_ID2);
	pin_status=(pin_status1)|(pin_status2<<1);
	printk("focaltech_ts_get_id_pin pin_status1=0x%x,pin_status2=0x%x,pin_status=0x%x\n",pin_status1,pin_status2,pin_status);
	return pin_status;
}

static int focaltech_ts_pinctrl_init(struct  ft5x0x_ts_data *ft5x0x_ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x0x_ts->ts_pinctrl = devm_pinctrl_get(&(ft5x0x_ts->client->dev));
	if (IS_ERR_OR_NULL(ft5x0x_ts->ts_pinctrl)) {
		dev_dbg(&ft5x0x_ts->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(ft5x0x_ts->ts_pinctrl);
		ft5x0x_ts->ts_pinctrl = NULL;
		return retval;
	}

	ft5x0x_ts->gpio_state_active
		= pinctrl_lookup_state(ft5x0x_ts->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ft5x0x_ts->gpio_state_active)) {
		dev_dbg(&ft5x0x_ts->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(ft5x0x_ts->gpio_state_active);
		ft5x0x_ts->ts_pinctrl = NULL;
		return retval;
	}

	ft5x0x_ts->gpio_state_suspend
		= pinctrl_lookup_state(ft5x0x_ts->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ft5x0x_ts->gpio_state_suspend)) {
		dev_dbg(&ft5x0x_ts->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ft5x0x_ts->gpio_state_suspend);
		ft5x0x_ts->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}
static int focaltech_ts_pinctrl_select(struct  ft5x0x_ts_data *ft5x0x_ts, bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ft5x0x_ts->gpio_state_active
		: ft5x0x_ts->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ft5x0x_ts->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ft5x0x_ts->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		dev_err(&ft5x0x_ts->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}
static int focaltech_ts_gpio_configure(struct  ft5x0x_ts_data *ft5x0x_ts, bool on)
{
	int retval = 0;

	if (on) {
		if (gpio_is_valid(ft5x0x_ts->pdata->gpio_irq)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(ft5x0x_ts->pdata->gpio_irq,"touch_irq_gpio");
			if (retval) {
				dev_err(&ft5x0x_ts->client->dev,
					"unable to request gpio [%d]\n",
					ft5x0x_ts->pdata->gpio_irq);
				goto err_irq_gpio_req;
			}
			retval = gpio_direction_input(ft5x0x_ts->pdata->gpio_irq);
			if (retval) {
				dev_err(&ft5x0x_ts->client->dev,
					"unable to set direction for gpio " \
					"[%d]\n",ft5x0x_ts->pdata->gpio_irq);
				goto err_irq_gpio_dir;
			}
		} else {
			dev_err(&ft5x0x_ts->client->dev,
				"irq gpio not provided\n");
			goto err_irq_gpio_req;
		}

		if (gpio_is_valid(ft5x0x_ts->pdata->gpio_reset)) {
			/* configure touchscreen reset out gpio */
			retval = gpio_request(ft5x0x_ts->pdata->gpio_reset,
					"touch_reset_gpio");
			if (retval) {
				dev_err(&ft5x0x_ts->client->dev,
					"unable to request gpio [%d]\n",
					ft5x0x_ts->pdata->gpio_reset);
				goto err_irq_gpio_dir;
			}

			retval = gpio_direction_output(ft5x0x_ts->pdata->gpio_reset, 1);
			if (retval) {
				dev_err(&ft5x0x_ts->client->dev,
					"unable to set direction for gpio[%d]\n",
					ft5x0x_ts->pdata->gpio_reset);
				goto err_reset_gpio_dir;
			}
		} else {
			dev_err(&ft5x0x_ts->client->dev,
				"reset gpio not provided\n");
			goto err_irq_gpio_dir;
		}
		return 0;
	} else {
			if (gpio_is_valid(ft5x0x_ts->pdata->gpio_irq))
				gpio_free(ft5x0x_ts->pdata->gpio_irq);
			if (gpio_is_valid(ft5x0x_ts->pdata->gpio_reset)) {
				/*
				 * This is intended to save leakage current
				 * only. Even if the call(gpio_direction_input)
				 * fails, only leakage current will be more but
				 * functionality will not be affected.
				 */
				retval = gpio_direction_input(ft5x0x_ts->pdata->gpio_reset);
				if (retval) {
					dev_err(&ft5x0x_ts->client->dev,
					"unable to set direction for gpio "
					"[%d]\n", ft5x0x_ts->pdata->gpio_reset);
				}
				gpio_free(ft5x0x_ts->pdata->gpio_reset);
			}

		return 0;
	}

err_reset_gpio_dir:
	if (gpio_is_valid(ft5x0x_ts->pdata->gpio_reset))
		gpio_free(ft5x0x_ts->pdata->gpio_reset);
err_irq_gpio_dir:
	if (gpio_is_valid(ft5x0x_ts->pdata->gpio_irq))
		gpio_free(ft5x0x_ts->pdata->gpio_irq);
err_irq_gpio_req:
	return retval;
}


static int focaltech_ts_regulator_configure(struct ft5x0x_ts_data
						*ft5x0x_ts, bool on)
{
#if FTP_PWR_EN
	int retval;

	if (on == false)
		goto hw_shutdown;
	ft5x0x_ts->pdata->ts_vdd = regulator_get(&ft5x0x_ts->client->dev,"vdd");
	if (IS_ERR(ft5x0x_ts->pdata->ts_vdd)) {
		dev_err(&ft5x0x_ts->client->dev,
				"%s: Failed to get vdd regulator\n",
				__func__);
		return PTR_ERR(ft5x0x_ts->pdata->ts_vdd);
	}

	if (regulator_count_voltages(ft5x0x_ts->pdata->ts_vdd) > 0) {
		retval = regulator_set_voltage(ft5x0x_ts->pdata->ts_vdd,
			2600000, 3300000);
		if (retval) {
			dev_err(&ft5x0x_ts->client->dev,
				"regulator set_vtg failed retval =%d\n",
				retval);
			goto err_set_vtg_vdd;
		}
	}

	if (ft5x0x_ts->pdata->i2c_pull_up) {
		ft5x0x_ts->pdata->ts_vcc_i2c = regulator_get(&ft5x0x_ts->client->dev,"vcc_i2c");
		if (IS_ERR(ft5x0x_ts->pdata->ts_vcc_i2c)) {
			printk("vcc_i2c regulator get fail!\n");
			dev_err(&ft5x0x_ts->client->dev,
					"%s: Failed to get i2c regulator\n",
					__func__);
			retval = PTR_ERR(ft5x0x_ts->pdata->ts_vcc_i2c);
			goto err_get_vtg_i2c;
		}
		if (regulator_count_voltages(ft5x0x_ts->pdata->ts_vcc_i2c) > 0) {
			retval = regulator_set_voltage(ft5x0x_ts->pdata->ts_vcc_i2c,1800000, 1800000);
			//	printk("vcc_i2c regulator get fail!\n");
			if (retval) {
				printk("vcc_i2c set voltage fail, retval=%d!\n",retval);
			//	dev_err(&ft5x0x_ts->client->dev,"reg set i2c vtg failed retval =%d\n",	retval);
			goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (ft5x0x_ts->pdata->i2c_pull_up)
		regulator_put(ft5x0x_ts->pdata->ts_vcc_i2c);
err_get_vtg_i2c:
	if (regulator_count_voltages(ft5x0x_ts->pdata->ts_vdd) > 0)
		regulator_set_voltage(ft5x0x_ts->pdata->ts_vdd, 0,
		1800000);
err_set_vtg_vdd:
	regulator_put(ft5x0x_ts->pdata->ts_vdd);
	return retval;

hw_shutdown:
	if (regulator_count_voltages(ft5x0x_ts->pdata->ts_vdd) > 0)
		regulator_set_voltage(ft5x0x_ts->pdata->ts_vdd, 0,
			3300000);

	regulator_put(ft5x0x_ts->pdata->ts_vdd);

	if (ft5x0x_ts->pdata->i2c_pull_up) {
		if (regulator_count_voltages(ft5x0x_ts->pdata->ts_vcc_i2c) > 0)
			regulator_set_voltage(ft5x0x_ts->pdata->ts_vcc_i2c, 0,
					1800000);
		regulator_put(ft5x0x_ts->pdata->ts_vcc_i2c);
	}
#endif

	return 0;
}
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int focaltech_ts_power(	int on) {
	struct  ft5x0x_ts_data *ft5x0x_ts=i2c_get_clientdata(this_client);
#if FTP_PWR_EN
	int retval;

	if (!on)
		goto power_off;

	retval = reg_set_optimum_mode_check(ft5x0x_ts->pdata->ts_vdd,
	15000);
	if (retval < 0) {
		printk("mode check vdd error, retval=%d\n",retval);
		return retval;
	}

	retval = regulator_enable(ft5x0x_ts->pdata->ts_vdd);
	if (retval) {
		printk("power on vdd error, retval=%d\n",retval);
		goto error_reg_en_vdd;
	}

	if (ft5x0x_ts->pdata->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(
			ft5x0x_ts->pdata->ts_vcc_i2c, 10000);
		if (retval < 0) {
			printk("mode check vcc_i2c error, retval=%d\n",retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(ft5x0x_ts->pdata->ts_vcc_i2c);
		if (retval) {
			printk("power on vcc_i2c error, retval=%d\n",retval);
			goto error_reg_en_vcc_i2c;
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	if (ft5x0x_ts->pdata->i2c_pull_up)
		reg_set_optimum_mode_check(ft5x0x_ts->pdata->ts_vcc_i2c, 0);
error_reg_opt_i2c:
	regulator_disable(ft5x0x_ts->pdata->ts_vdd);
error_reg_en_vdd:
	reg_set_optimum_mode_check(ft5x0x_ts->pdata->ts_vdd, 0);
	return retval;

power_off:
	reg_set_optimum_mode_check(ft5x0x_ts->pdata->ts_vdd, 0);
	retval = regulator_disable(ft5x0x_ts->pdata->ts_vdd);
	if (retval)
		pr_err("Ts regulator vdd disable failed rc=%d\n", retval);
	if (ft5x0x_ts->pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(ft5x0x_ts->pdata->ts_vcc_i2c, 0);
		retval = regulator_disable(ft5x0x_ts->pdata->ts_vcc_i2c);
		if (retval)
			pr_err("Ts regulator vcc_i2c disable failed rc=%d\n", retval);
	}
	return retval;
#endif
	return 0;
}

static int focaltech_ts_init_resource(void)
{
	int ret = 0;
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

	ret = focaltech_ts_pinctrl_init(ft5x0x_ts);
	if (!ret && ft5x0x_ts->ts_pinctrl) {
		ret = focaltech_ts_pinctrl_select(ft5x0x_ts, true);
		if(ret)	{
			pr_err("Touchscreen GPIO config failed!\n");
			return ret;
		}
	}

	ret = focaltech_ts_gpio_configure(ft5x0x_ts, true);
	if(ret)	{
			pr_err("Touchscreen GPIO active failed!\n");
			return ret;
	}
	ret = focaltech_ts_regulator_configure(ft5x0x_ts, true);
	if(ret)
	{
		ret = focaltech_ts_gpio_configure(ft5x0x_ts, false);
		if (ret < 0)
			pr_err("Failed to deconfigure gpios\n");
		if (ft5x0x_ts->ts_pinctrl) {
			ret = focaltech_ts_pinctrl_select(ft5x0x_ts, false);
			if (ret < 0)
				pr_err("Cannot set idle pinctrl state\n");
		}
		return ret;
	}
	pr_debug("TOUCH:YLLOG:requst and init resource complete\n");
	return 0;
}
static void focaltech_ts_release_resource(void)
{
	int ret;
	struct  ft5x0x_ts_data *ft5x0x_ts=i2c_get_clientdata(this_client);

		ret = focaltech_ts_gpio_configure(ft5x0x_ts, false);
		if (ret < 0)
			pr_err("Failed to deconfigure gpios\n");
		if (ft5x0x_ts->ts_pinctrl) {
			ret = focaltech_ts_pinctrl_select(ft5x0x_ts, false);
			if (ret < 0)
				pr_err("Cannot set idle pinctrl state\n");
		}
	focaltech_ts_regulator_configure(ft5x0x_ts, false);
}
static int focaltech_ts_reset(int ms)
{
	struct  ft5x0x_ts_data *ft5x0x_ts=i2c_get_clientdata(this_client);
	gpio_direction_output(ft5x0x_ts->pdata->gpio_reset, 0);
	mdelay(5);
	gpio_direction_output(ft5x0x_ts->pdata->gpio_reset, 1);
	mdelay(ms);
	return 0;
}

static int focaltech_ts_suspend(void){
	int ret;
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

#if FTP_SLEEP_PWR_EN
	focaltech_ts_power(0);
#endif
	if (ft5x0x_ts->ts_pinctrl) {
		ret = focaltech_ts_pinctrl_select(ft5x0x_ts, false);
		if (ret < 0)
			pr_err( "Cannot get idle pinctrl state\n");
	}

	ret = focaltech_ts_gpio_configure(ft5x0x_ts, false);
	if (ret < 0)
		pr_err("failed to put gpios in suspend state\n");
	printk(KERN_NOTICE "TW had suspended.\n");
	return 0;
}
static int focaltech_ts_resume(void){
	int retval;
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

	if (ft5x0x_ts->ts_pinctrl) {
		retval = focaltech_ts_pinctrl_select(ft5x0x_ts, true);
		if (retval < 0)
			pr_err( "Cannot get idle pinctrl state\n");
	}
	retval = focaltech_ts_gpio_configure(ft5x0x_ts, true);
	if (retval < 0)
		pr_err("failed to put gpios in suspend state\n");

#if FTP_SLEEP_PWR_EN
	focaltech_ts_power(1);
#endif
	focaltech_ts_reset(ft5x0x_ts->pdata->reset_delay);
	pr_info("TW resume.\n");
	return 0;
}

static int goodix_ts_parse_dt(struct device *dev, struct tw_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret;
	enum of_gpio_flags flags;

	/* reset, irq gpio info */
	pdata->gpio_reset = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &flags);
	TW_DBG("reset-gpio = %d\n", pdata->gpio_reset);

	pdata->gpio_irq = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &flags);
	TW_DBG("irq-gpio = %d\n", pdata->gpio_irq);

	ret = of_property_read_u32(np, "focaltech,irq_flags", (unsigned int *)&pdata->irqflag);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"focaltech,irq_flags", np->full_name);
		return -ENODEV;
	}
	TW_DBG("irq-flag = %d\n", (int)pdata->irqflag);
	pdata->irqflag = 2;

	ret = of_property_read_u32(np, "focaltech,screen_x", &pdata->screen_x);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"focaltech,screen_x", np->full_name);
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "focaltech,screen_y", &pdata->screen_y);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"focaltech,screen_y", np->full_name);
		return -ENODEV;
	}
	TW_DBG("screen x,y = %d, %d\n", pdata->screen_x, pdata->screen_y);

	ret = of_property_read_u32(np, "focaltech,pwr_en", &pdata->pwr_en);
	ret = of_property_read_u32(np, "focaltech,sleep_pwr_en", &pdata->sleep_pwr_en);
//	ret = of_property_read_string(np, "focaltech,vcc_i2c-supply", &pdata->ts_vcc_i2c);
//	ret = of_property_read_string(np, "focaltech,vdd-supply", &pdata->ts_vdd);
	pdata->i2c_pull_up = of_property_read_bool(np,
				"focaltech,i2c-pull-up");
	TW_DBG("pwr_en=%d, sleep_pwr_en=%d\n", pdata->pwr_en,
				pdata->sleep_pwr_en);
	ret = of_property_read_u32(np, "focaltech,reset-delay", &pdata->reset_delay);
	if (ret) {
		pdata->reset_delay = 300;
		dev_err(dev, "Unable to read reset delay\n");
	}

	if (pdata->buttons == NULL) {
		pdata->buttons = virtual_keys_button;
		pdata->nbuttons = ARRAY_SIZE(virtual_keys_button);
	}
	pdata->init = focaltech_ts_init_resource;//goodix_ts_hw_init;
	pdata->release = focaltech_ts_release_resource;//goodix_ts_release;
	pdata->reset = focaltech_ts_reset;//goodix_ts_reset;
	pdata->power = focaltech_ts_power;//goodix_ts_power;
	pdata->suspend = focaltech_ts_suspend;//goodix_ts_sleep;
	pdata->resume = focaltech_ts_resume;//goodix_ts_wakeup;
	pdata->get_id_pin=focaltech_ts_get_id_pin;

	return 0;
}
#endif

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
	int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c error.\n", __func__);
	}
	return ret;
}

static int Read_boot_tp_ID(void)
{
	u8 reg_val[2] = {0};
	u8 i = 0;
	u8 try_count = 0;
	u8 packet_buf[6];
	u8 auc_i2c_write_buf[10];
	u8 i_ret;
	u8 ret = -1;

retryupdate:
	ft5x0x_write_reg(0xfc, 0xaa);
	msleep(DELAY_AA);
	ft5x0x_write_reg(0xfc, 0x55);
	msleep(DELAY_55);

	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do {
		i++;
		i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
		msleep(20);
	} while (i_ret <= 0 && i < 5);
	msleep(DELAY_ID);

	cmd_write(0x90, 0x00, 0x00, 0x00, 4);
	byte_read(reg_val, 2);
	if (reg_val[0] == 0x79 && reg_val[1] == CTPM_ID2) {
		TW_DBG("CTPM,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
	} else {
		TW_DBG("Check ctpm id failed, need try again!\n");
		while (try_count < 20) {
			try_count++;
			goto retryupdate;
		}
		return ret;
	}

	packet_buf[0] = 0x3;
	packet_buf[1] = 0x0;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	ft5x06_i2c_read(this_client, packet_buf, 4, packet_buf, 6);
	TW_DBG("uc_panel_factory_id = 0x%x\n", packet_buf[4]);

	cmd_write(0x07, 0x00, 0x00, 0x00, 1);
	msleep(300);
	return  packet_buf[4];
}

#define    FTS_PACKET_LENGTH        128

int fts_check_tp_id(unsigned char tp_id)
{
	int i = 0;

	for(i=0; i< fw_array_size; i++)
	{
		if (tp_id == tp_info[i].tp_id)
			 return i;
	}

	return 0;//-1;//maleijie
}

void fts_get_current_tp_info(int num)
{
	current_tp_supported = true;
	current_tp_name  = tp_info[num].tp_name;
	current_fw_to_tp = tp_info[num].firmware;
	current_fw_size  = tp_info[num].firmware_size;
}
//add for download mode
void fts_get_main_current_tp_info(int num)
{

	current_main_fw_to_tp = yl_main_fw[num].firmware;
	current_main_fw_size  = yl_main_fw[num].firmware_size;
}
E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;
    FTS_BYTE is_5336_new_bootloader = 0;
    FTS_DWRD try_count = 0;
    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

retryupdate:
    /*********Step 1:Reset  CTPM *****/
/* yulong modify */
/* author: tanliyu*/
/* time: 2012.10.20 */

/*write 0xaa to register 0xfc*/
/* soft reset */
#if 1
    ft5x0x_write_reg(0xfc,0xaa);
    msleep(DELAY_AA);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");

    msleep(DELAY_55);
#endif

#if 0
/* hardware reset */
    ft5x0x_reset(35);
/* yulong end */
#endif

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        msleep(5);
    }while(i_ret <= 0 && i < 5 );

    msleep(DELAY_ID);

    /*********Step 3:check READ-ID***********************/
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == CTPM_ID2)
    {
        printk(KERN_INFO "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
	printk(KERN_ERR "[FTS] Check ctpm id failed, need try again. CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
       while (try_count < 20)
       {
	try_count ++;
         goto retryupdate;
       }

	ft5x0x_reset(300);
	return ERR_READID;
      //i_is_new_protocol = 1;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
/**litao3 modify ft5x06 serises chip's bootloader id must be 0*/
    if (reg_val[0] > 4)
	is_5336_new_bootloader = 0;
    printk(KERN_INFO "[FTS] bootloader version = 0x%x\n", reg_val[0]);

     /*********Step 4:erase app and panel paramenter area ********************/
    cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
    msleep(DELAY_EARSE);
    printk(KERN_DEBUG "[FTS] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk(KERN_DEBUG "[FTS] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk(KERN_INFO "[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
	if (is_5336_new_bootloader && DEVICE_IC_TYPE == IC_FT5X06)
		temp = 0x7bfa + i;
	else
		temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        delay_qt_ms(20);
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(300);  //make sure CTP startup normally

    return ERR_OK;
}


int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    ft5x0x_write_reg(0, 0x40);
    delay_qt_ms(100);   //make sure already enter factory mode
    ft5x0x_write_reg(2, 0x4);  //write command to start calibration
    delay_qt_ms(300);
    if (DEVICE_IC_TYPE == IC_FT5X06) {
	for(i=0;i<100;i++)
	{
		ft5x0x_read_reg(0x02, &uc_temp);
		if (0x02 == uc_temp || 0xFF == uc_temp)
			{
				/*if 0x02, then auto clb ok, else 0xff, auto clb failure*/
			    break;
			}
			delay_qt_ms(20);
		}
    }
    else {
		for(i=0;i<100;i++)
		{
			ft5x0x_read_reg(0, &uc_temp);
			if (0x0 == ((uc_temp&0x70)>>4))  /*return to normal mode, calibration finish*/
			{
			    break;
			}
			delay_qt_ms(20);
		}
    }
    printk("[FTS] calibration OK.\n");

    msleep(300);
    ft5x0x_write_reg(0, 0x40);  //goto factory mode
    delay_qt_ms(200);   //make sure already enter factory mode
    ft5x0x_write_reg(2, 0x5);  //store CLB result
    delay_qt_ms(300);
    ft5x0x_write_reg(0, 0x0); //return to normal mode
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
	int ret = 0;
	//=========FW upgrade========================*/
	ret = fts_ctpm_fw_upgrade(current_fw_to_tp, current_fw_size);
	if (ret != 0)
		printk("[FTS] upgrade %s failed ret = %d.\n", current_tp_name, ret);
	else
	{
		printk("[FTS] upgrade %s successfully.\n", current_tp_name);
		fts_ctpm_auto_clb();  //start auto CLB
	}
	return ret;
}

#if	FOCALTECH_COB
int fts_cob_ctpm_fw_upgrade_with_i_file(void)
{
	int ret = 0;
	//=========FW upgrade========================*/
	ret = fts_ctpm_fw_upgrade(current_fw_to_tp, current_fw_size);
	if (ret != 0)
		printk("[FTS] upgrade %s failed ret = %d.\n", current_tp_name, ret);
	else
	{
		printk("[FTS] upgrade %s successfully.\n", current_tp_name);
	}
	return ret;
}

/***********************************************************************************************
function	:	read tp calibrate state.
			0,  calibrated
			1,  none calibrated.
***********************************************************************************************/
static unsigned int ft5x0x_read_tp_calibrated(void)
{
	FTS_BYTE reg_val[2] = {0};
	FTS_BYTE  auc_i2c_write_buf[10];
	FTS_DWRD i = 0;
	int	i_ret = 0;
	int	ret=0;
	int try_count=0;

	//struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);

	/*********Step 1:Reset  CTPM *****/
READID:

#if 1
    ft5x0x_write_reg(0xfc,0xaa);
    msleep(DELAY_AA);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");

    msleep(DELAY_55);
#else
    /* hardware reset */
    ft5x0x_reset(35);
#endif

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        msleep(5);
    }while(i_ret <= 0 && i < 5 );

    msleep(DELAY_ID);

    /*********Step 3:check READ-ID***********************/
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == CTPM_ID2)
    {
        printk(KERN_INFO "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
	printk(KERN_ERR "[FTS] Check ctpm id failed, need try again. CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
       while (try_count < 20)
       {
	try_count ++;
         goto READID;
       }

	ft5x0x_reset(300);
	return ERR_READID;
      //i_is_new_protocol = 1;
    }

	/*********Step 4:read clb area start addr:0x7C00***********************/
	cmd_write(0x03,0x00,0x7c,0x00,4);
	printk("cmd_write\n");
	mdelay(5);
	i_ret = byte_read(reg_val,1);
	if (i_ret == 0)
	{
		printk( "check clb failed!\r\n");
		return 1;
	}

	printk("the value reg_val[0]=0x%x \n",reg_val[0]);
	if(0xFF == reg_val[0])
	{
		printk("None calibration.\n");
		ret = 1;
	}
	else
	{
		printk("Calibrated. \n");
		ret = 0;
	}
	printk("Step 4:read clb area start addr:0x7C00\n");

	/**********Step 5:reset FW to return work mode**************/
	cmd_write(0x07,0x00,0x00,0x00,1);

	msleep(300);

	printk("[FTS]: fts_ctpm_check_calibration over.\n");

	return ret;
}

#endif

/* force update firmware */
int fts_ctpm_fw_upgrade_with_tp_id(unsigned char tp_id)
{
	   int ret = 0;
	   int num = fts_check_tp_id(tp_id);
	   if (num < 0)
	   	   return -1;
	   fts_get_current_tp_info(num);
	   ret = fts_ctpm_fw_upgrade_with_i_file();
	   if (ret == 0)
	      current_tp_id = tp_id; /*use to check, the id is right*/

          return ret;

}

unsigned char fts_ctpm_get_i_file_ver(unsigned char tp_id)
{
	unsigned char uc_if_ver;
	unsigned int ui_sz;
	int num = fts_check_tp_id(tp_id);
	if (num < 0)
		return -1;
	fts_get_current_tp_info(num);
	ui_sz = current_fw_size;
	if (ui_sz > 2)
		uc_if_ver = current_fw_to_tp[ui_sz - 2];
	else
		uc_if_ver = 0xff;

	return uc_if_ver;
}
#define    FTS_SETTING_BUF_LEN        128

#if  CFG_SUPPORT_TOUCH_KEY
 /* yulong add */
 /* reason : key debounce */
 /* author : tanliyu */
 /* time : 2012-10-04 */
static void ft_keys_timer(unsigned long _ft_data)
{
	   	inDebounce = false;
	   	TW_DBG("TOUCH:YLLOG:timer inDebounce false\n");
}


/******************************/
int fts_ctpm_chk_upg_probe(void)
{
	unsigned char uc_tp_fm_ver = 0;
	unsigned char IC_tp_id = 0;
	unsigned char uc_if_ver = 0;
	unsigned int ui_sz = 0;
	if (!current_tp_id) {
		TW_DBG("Get tp vid error!\n");
		return 1;
	} else
		TW_DBG("tp_vendor_id = %x.\n", current_tp_id);
	IC_tp_id = ft5x0x_read_tp_type();
	if (IC_tp_id == 0xa8) {
		TW_DBG("Get tp vid error!\n");
		return 1;
	} else
		TW_DBG("IC_tp_id = %x.\n", IC_tp_id);
	if (IC_tp_id != current_tp_id) {
		TW_DBG("current_tp_id and IC_tp_id is not coherent!\n");
		return 1;
	} else {
		uc_tp_fm_ver = ft5x0x_read_fw_ver();
		ui_sz = current_fw_size;
		if (ui_sz > 2)
			uc_if_ver = current_fw_to_tp[ui_sz - 2];
		TW_DBG("IC_fm_ver = 0x%02x,I_file_fw_ver= 0x%02x\n",
			uc_tp_fm_ver, uc_if_ver);
		if (uc_tp_fm_ver == 0xa6 || uc_tp_fm_ver < 0x10 ||
			uc_tp_fm_ver < uc_if_ver || uc_tp_fm_ver == 0x17)
			return 1;
		else
			return 0;
	}
}

int fts_ctpm_auto_upg(void)
{
	unsigned char uc_host_fm_ver;
	int           i_ret = 0;
	unsigned char i;

	/*try to update firmware if fails*/
	for(i=0;i<1;i++)
	{
		// get tp_id
//		vid_TP = ft5x06_read_vendor_id_TP();
	//	mdelay(100);
		i_ret = fts_ctpm_fw_upgrade_with_i_file();
		if (i_ret == 0)
		{
			uc_host_fm_ver = ft5x0x_read_fw_ver();
			printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
			return 0;
		}
		mdelay(5);
	}
	printk("[FTS] upgrade failed ret=%d.\n", i_ret);
	return (-1);
}
/***********************************/
static void key_release(int index)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);//tanliyu add
	int i = index;

	key_flags = 0;
	data->pdata->buttons[i].key_status = KEY_RELEASE;
	input_report_key(data->input_dev, data->pdata->buttons[i].code, 0);
	input_sync(data->input_dev);
//     printk("maleijie:key_release %d\n",data->pdata->buttons[i].code);//changed by taokai
	/* yulong add */
	/* reason : key debounce */
	/* author : tanliyu */
	/* time : 2012-10-04 */
	if (DEBOUNCE)
		mod_timer(&data->timer, jiffies + msecs_to_jiffies(DEBOUNCE));
	/* yulong end */

}

static void key_press(int index)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);//tanliyu add
	int i = index;
// printk("maleijie:key_press\n");//changed by taokai
	/* yulong modify */
	/* reason : key debounce */
	/* author : tanliyu */
	/* time : 2012-10-04 */
	if (DEBOUNCE && inDebounce)
	{
		TW_DBG("TOUCH:YLLOG:indebouce should return %d\n", i);
	}else{
		TW_DBG("TOUCH:YLLOG:indebouce changes to true%d\n", i);
		key_flags = i + 1;
	// 	printk("maleijie:YLLOG: data->pdata->buttons %d,%d\n", i,data->pdata->buttons[i].code);//changed by taokai
		data->pdata->buttons[i].key_status = KEY_PRESS;
		input_report_key(data->input_dev, data->pdata->buttons[i].code, 1);
		input_sync(data->input_dev);
		if(DEBOUNCE)
			inDebounce = true;//tanliyu add
	}

}


/* yulong end */
#endif


/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#if  CFG_SUPPORT_TOUCH_KEY
	if (key_flags != 0)
		key_release(key_flags-1);
#endif
	if (touch_flag) {
		touch_flag = 0;
	input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
	TW_DBG("Point[%d] Touch Up. x=%d, y=%d.\n",
		event->au8_finger_id[0], event->au16_x[0], event->au16_y[0]);
	}
}

static int ft5x0x_read_data(void)
{
    struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
    struct ts_event *event = &data->event;
    u8 buf[CFG_POINT_READ_BUF] = {0};
    int ret = -1;
    int i;



//	printk("ft5x0x_read_data\n");//maleijie //changed by taokai

    ret = ft5x0x_i2c_rxdata(buf, CFG_POINT_READ_BUF);
    if (ret < 0)
    {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		    return ret;
	  }

    memset(event, 0, sizeof(struct ts_event));

    event->touch_point = buf[2]>>4;
    //event->touch_point = buf[2]&&0x07;
//    printk("ft5x0x_read_data2 touch_point: %d\n",event->touch_point);//maleijie//changed bu taokai
    if (event->touch_point > CFG_MAX_TOUCH_POINTS)
    {
        event->touch_point = CFG_MAX_TOUCH_POINTS;
    }

    for (i = 0; i < event->touch_point; i++)
    {
        event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
        event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
        event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
        event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
        event->pressure[i]     = buf[7 + 6*i];
        event->touch_size[i]   = buf[8 + 6*i];
        if (event->au16_x[i] > TP_MAX_X || event->au16_y[i] > TP_MAX_Y)
		printk(KERN_ERR "TOUCH:YLLOG:Error pointer x = %d y = %d event = %d\n", event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);


//				event->au16_y[i], event->au8_touch_event[i]);  //changed by taokai
    }
    /*One point and active up, release it*/
    if (event->touch_point == 1)
    {
       if (event->au8_touch_event[0]== 1){
		   if (event->au16_y[0] < SCREEN_MAX_Y){
			     touch_flag = 1;
		   }
           ft5x0x_ts_release();
           return 1;
       }

    }
    //event->pressure = 200;
//	printk("ft5x0x_read_data end..\n");//maleijie    //chenged by taokai
    return 0;
}

/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/


#if CFG_SUPPORT_TOUCH_KEY
int ft5x0x_touch_key_process(struct input_dev *dev, u16 x, u16 y, int touch_event)
{
	int i;
	int nbuttons;
	int nx0, ny0, nx1, ny1;
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);//tanliyu add
	nbuttons = data->pdata->nbuttons;
//  printk("maleijie ft5x0x_touch_key_process :%d,%d\n",x,y);//maleijie  //changed by taokai
	for (i = 0; i < nbuttons; i++){

       nx0 = data->pdata->buttons[i].x;
       ny0 = data->pdata->buttons[i].y;
       nx1 = data->pdata->buttons[i].x + data->pdata->buttons[i].width;
       ny1 = data->pdata->buttons[i].y + data->pdata->buttons[i].height;
//	   printk("maleijie ft5x0x_touch_key_process nbuttons :%d,%d,%d,%d\n",nx0,ny0,nx1,ny1);//maleijie
       if ( x >=nx0 && x <=nx1 && y >=ny0 && y<= ny1){
//	   	printk("maleijie ft5x0x_touch_key_process touch_event :%d,%d,%d\n",x,y,touch_event);//maleijie
	         if (data->pdata->buttons[i].key_status)
		  {
	              if (touch_event == 2)		//this key is contact
	                	break;
	              else if(touch_event == 1)	//this key is up
		  	{
		  		key_release(i);
				break;
	              	}
		      else if (touch_event == 0) //this key is down
			{
				key_release(i);
				key_press(i);
				break;
			}
	         }
		  else
		  {
		  	if(touch_event == 0)  //this key is down
	            	{
				key_press(i);
				break;
			}
	        }
       }
   }

   return 0;
}

#endif

void report_point(struct input_dev *dev, u16 x, u16 y, u16 t, u16 p, u8 id)
{
	input_report_abs(dev, ABS_MT_POSITION_X, x);
	input_report_abs(dev, ABS_MT_POSITION_Y, y);
	input_report_abs(dev, ABS_MT_TOUCH_MAJOR, t);
	input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 1);
	input_report_abs(dev, ABS_MT_PRESSURE, p);
	input_report_abs(dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(dev);
//	printk/*maleijie TW_DBG*/( "maleijie pointer%d x=%d y=%d\n", id, x, y);   //changed by taokai
}


static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	int i;
	int release[CFG_MAX_TOUCH_POINTS] = {0};
	int rel = 1;

	#if  CFG_SUPPORT_TOUCH_KEY
	if ((key_flags != 0) && (event->au16_y[0] < SCREEN_MAX_Y))
		key_release(key_flags-1);
	#endif

	for (i  = 0; i < event->touch_point; i++) {
		if (event->au16_x[i] < SCREEN_MAX_X) {
			if (event->au16_y[i] < (SCREEN_MAX_Y-3)) {
				if (event->au8_touch_event[i] == 0
					|| event->au8_touch_event[i] == 2) {
					touch_flag = 1;
					TW_DBG("P[%d]Down.x=%d,y=%d,i=%d.\n",
						event->au8_finger_id[i],
						event->au16_x[i],
						event->au16_y[i], i);
					if (event->pressure[i] == 0)
						event->pressure[i] = 10;
					report_point(data->input_dev,
						event->au16_x[i],
						event->au16_y[i],
						event->touch_size[i],
						event->pressure[i],
						event->au8_finger_id[i]);
				} else
					release[i] = 1;
			} else if (event->au16_y[i] == SCREEN_MAX_Y)
				release[i] = 1;
			else {
				if (event->au8_touch_event[i] == 0
					|| event->au8_touch_event[i] == 2) {
					touch_flag = 1;
					TW_DBG("P[%d]Down.x=%d,y=%d,i=%d.\n",
						event->au8_finger_id[i],
						event->au16_x[i],
						event->au16_y[i], i);
				}
				if (i != 0)
					release[i] = 1;
				else {
					#if  CFG_SUPPORT_TOUCH_KEY
					ft5x0x_touch_key_process(
						data->input_dev,
						event->au16_x[i],
						event->au16_y[i],
						event->au8_touch_event[i]);
					if (key_flags == 0 &&
						event->au8_touch_event[i] != 0)
						release[i] = 1;
					else
						return;
					#else
						release[i] = 1;
					#endif
				}
			}
		}
	}

	for (i = 0; i < event->touch_point; i++)
		rel = rel & release[i];
	if (rel) {
		ft5x0x_ts_release();
		return;
	} else
		input_sync(data->input_dev);
}

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	struct  ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

	ret = ft5x0x_read_data();
	if (ret == 0)
		ft5x0x_report_value();
	TW_irq_enable(ft5x0x_ts);
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	TW_irq_disable(ft5x0x_ts);
	queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	return IRQ_HANDLED;
}
#define CONFIG_TOUCHSCREEN_FOCALTECH_READ_CHARGE_STATUS
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_READ_CHARGE_STATUS
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#define TW_CHARGING_STATUS_CHECK_CIRCLE 2000

static int charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
struct workqueue_struct *ft5x16_rd_chg_wq;
struct delayed_work ft5x16_rd_chg_work;
int ft5x16_init_complete = 0;
struct power_supply  *batt_psy;

int ft5x16_read_charge_status(void)
{
	union power_supply_propval ret = {0,};

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &ret);
	return ret.intval;
}

void ft5x16_read_charge_changed(void)
{
	if(ft5x16_init_complete)
		queue_delayed_work(ft5x16_rd_chg_wq,
			&ft5x16_rd_chg_work,
			msecs_to_jiffies(TW_CHARGING_STATUS_CHECK_CIRCLE));
}

static void ft5x16_read_chg_work_func(struct work_struct *work)
{
	int new_charge_status = 0;
	u8 write_val;
	u8 read_val;
	int count = 0;
	TW_DBG("charge_status = %d.\n", charge_status);
	new_charge_status = ft5x16_read_charge_status();
	if(new_charge_status != charge_status) {
		TW_DBG("%s: charge status has changed, new_charge_status = %d.\n",
			__func__, new_charge_status); //TW_DBG
		if(new_charge_status == POWER_SUPPLY_STATUS_CHARGING)
			write_val = 0x01;
		else
			write_val = 0x00;

		for(count =0; count <5; count++) {
			ft5x0x_write_reg(0x8b, write_val);
			mdelay(15);
			ft5x0x_read_reg(0x8b, &read_val);
			if(write_val == read_val) {
				TW_DBG("%s: read 0x8b register value = %d.\n", __func__, read_val); //TW_DBG
				break;
			} else {
				printk(KERN_ERR "%s : write 0x8b err, count = %d.\n", __func__, count);
				mdelay(5);
			}
		}
		charge_status = new_charge_status;
	}
	queue_delayed_work(ft5x16_rd_chg_wq,
		&ft5x16_rd_chg_work,
		msecs_to_jiffies(TW_CHARGING_STATUS_CHECK_CIRCLE));
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)||defined(CONFIG_FB)
static void ft5x0x_ts_suspend(struct device *dev)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

	TW_irq_disable(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	flush_workqueue(ft5x0x_ts->ts_workqueue);

#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_READ_CHARGE_STATUS
	charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	ft5x16_init_complete = 0;
	cancel_delayed_work_sync(&ft5x16_rd_chg_work);
	flush_workqueue(ft5x16_rd_chg_wq);
#endif

#if CFG_SUPPORT_TOUCH_KEY
	if (DEBOUNCE) {
		del_timer_sync(&ft5x0x_ts->timer);
		inDebounce = false;
	}
#endif

	ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	ft5x0x_ts_release();
	if (ft5x0x_ts->pdata->suspend)
		ft5x0x_ts->pdata->suspend();
}

static void ft5x0x_ts_resume(struct device *dev)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);

	if (ft5x0x_ts->pdata->resume)
		ft5x0x_ts->pdata->resume();
	TW_irq_enable(ft5x0x_ts);

#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_READ_CHARGE_STATUS
	ft5x16_init_complete = 1;
	ft5x16_read_charge_changed();
#endif
}
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x0x_ts_data *ft5x0x_data =
		container_of(self, struct ft5x0x_ts_data, fb_notif);
	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x0x_data && ft5x0x_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (sleep_flag == resume_mode) {
				TW_DBG("Resume, don't enter again!\n");
				return 0;
			} else
				sleep_flag = resume_mode;
			ft5x0x_ts_resume(&ft5x0x_data->client->dev);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			if (sleep_flag == suspend_mode) {
				TW_DBG("Suspend, don't enter again!\n");
				return 0;
			} else
				sleep_flag = suspend_mode;
			ft5x0x_ts_suspend(&ft5x0x_data->client->dev);
		}
	}
	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x0x_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *data = container_of(handler,
						   struct ft5x0x_ts_data,
						   early_suspend);

	ft5x0x_ts_suspend(&data->client->dev);
}
static void ft5x0x_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *data = container_of(handler,
						   struct ft5x0x_ts_data,
						   early_suspend);
	ft5x0x_ts_resume(&data->client->dev);
}
#endif
//add by taokai

#ifdef CONFIG_PM_RUNTIME
static int ft5x0x_ts_runtime_suspend(struct device *dev)
{
    //printk("+++++enter tp suspend+++++axf\n");
    struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);
    struct input_dev *input_dev = ft5x0x_ts->input_dev;
    struct irq_desc *desc = irq_to_desc(ft5x0x_ts->irq);
	printk("+++++enter tp suspend+++++axf\n");
/* sysfs */
    mutex_lock(&input_dev->mutex);
    disable_irq_nosync(ft5x0x_ts->irq);
    cancel_work_sync(&ft5x0x_ts->pen_event_work);
    flush_workqueue(ft5x0x_ts->ts_workqueue);

#if CFG_SUPPORT_TOUCH_KEY
	if(DEBOUNCE)
	{
		del_timer_sync(&ft5x0x_ts->timer);
	    inDebounce = false;
	}
#endif

    if(unlikely ( 0 == desc->depth ) )
    {
        mdelay(3);
        printk(KERN_ERR "%s:%d,desc->depth=%d\n",__func__,__LINE__,desc->depth);
        disable_irq_nosync(ft5x0x_ts->irq);
    }
    ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
    ft5x0x_ts_release();
    if(ft5x0x_ts->pdata->suspend)
	ft5x0x_ts->pdata->suspend();
    mutex_unlock(&input_dev->mutex);
	return 0;
}
static int ft5x0x_ts_runtime_resume(struct device *dev)
{
    //printk("+++++enter tp resume+++++axf\n");
    struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(this_client);
    struct input_dev *input_dev = ft5x0x_ts->input_dev;
    struct irq_desc *desc = irq_to_desc(ft5x0x_ts->irq);
	printk("+++++enter tp resume+++++axf\n");
    mutex_lock(&input_dev->mutex);
    if(ft5x0x_ts->pdata->resume)
	ft5x0x_ts->pdata->resume();
    if( unlikely( desc->depth > 1 ) )
    {
        enable_irq(ft5x0x_ts->irq);
        mdelay(10);
        if( unlikely( desc->depth ) )
        enable_irq(ft5x0x_ts->irq);
        printk(KERN_ERR "%s:%d,desc->depth=%d\n",__func__,__LINE__,desc->depth);
    }
    else
        enable_irq(ft5x0x_ts->irq);
    mutex_unlock(&input_dev->mutex);
	return 0;
}
static const struct dev_pm_ops ft5x0x_ts_pmops={
	SET_RUNTIME_PM_OPS(ft5x0x_ts_runtime_suspend, ft5x0x_ts_runtime_resume, NULL)
};
#endif				/* CONFIG_PM_RUNTIME */

/***********************************************************************************************
Name	:fts_ctpm_repair_firemware

Input	:


Output	:

function	:repair_firemware

***********************************************************************************************/
#if  0//maleijie
static int fts_ctpm_repair_firemware(void)
{
    unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID

    unsigned char buf[FTS_SETTING_BUF_LEN];

    FTS_BYTE reg_val[2] = {0};
    FTS_BYTE  auc_i2c_write_buf[10];
    //FTS_BYTE  packet_buf[FTS_SETTING_BUF_LEN + 6];
    FTS_DWRD i = 0;
    int i_ret;
    int ret;
    int repair_result = 0;
    unsigned char uc_host_fm_ver;

    uc_i2c_addr = 0x70;
    uc_io_voltage = 0x0;
    uc_panel_factory_id = 0x5a;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(0xfc,0xaa);
    delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");

    delay_qt_ms(30);

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    /*
    check id is different.
    chip:ft5506 0x79:0x6
    chip:ft5306 0x79:0x3
    */
    if (reg_val[0] == 0x79 && reg_val[1] == CTPM_ID2)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        //return ERR_READID;
        return 0;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("bootloader version = 0x%x\n", reg_val[0]);


    /* --------- read current project setting  ---------- */
    //set read start address
    buf[0] = 0x3;
    buf[1] = 0x0;
    buf[2] = 0x78;
    buf[3] = 0x0;
    byte_write(buf, 4);
    byte_read(buf, FTS_SETTING_BUF_LEN);

    printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
        buf[0],  buf[2], buf[4]);
    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
    {
        if (i % 16 == 0)     printk("\n");
        printk("0x%x, ", buf[i]);

    }
    printk("\n");
    /********* reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(300);
     ret = fts_ctpm_fw_upgrade_with_tp_id(buf[4]);
     if (ret == 0)
        {
                msleep(300);
                uc_host_fm_ver = fts_ctpm_get_i_file_ver(buf[4]);
                pr_info("%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
                repair_result = 1;
        }
    else
        {
                repair_result = 0;
                pr_err("%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, ret);
         }
     return repair_result;

}
#endif

/*sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name
*note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft5x0x_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int ft5x0x_ReadFirmware(char *firmware_name,
			       unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}



/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client,
				       char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);

	if (fwsize <= 0) {
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	if (fwsize < 8 || fwsize > 32 * 1024) {
		dev_dbg(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}

	/*=========FW upgrade========================*/
	pbt_buf = kmalloc(fwsize + 1, GFP_ATOMIC);

	if (ft5x0x_ReadFirmware(firmware_name, pbt_buf)) {
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(pbt_buf);
		return -EIO;
	}

	if(current_tp_id != pbt_buf[fwsize-1])
	{
		printk(KERN_ERR"current_tp_id=0x%x, bin_file_id=0x%x\n", current_tp_id, pbt_buf[fwsize-1]);
		kfree(pbt_buf);
		return -EIO;
	}

	if ((pbt_buf[fwsize - 8] ^ pbt_buf[fwsize - 6]) == 0xFF
		&& (pbt_buf[fwsize - 7] ^ pbt_buf[fwsize - 5]) == 0xFF
		&& (pbt_buf[fwsize - 3] ^ pbt_buf[fwsize - 4]) == 0xFF) {
		/*call the upgrade function */
		i_ret = fts_ctpm_fw_upgrade(pbt_buf, fwsize);
		if (i_ret != 0)
			dev_dbg(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
						__func__);
		else {
			fts_ctpm_auto_clb();	/*start auto CLB*/
		 }
		kfree(pbt_buf);
	} else {
		dev_dbg(&client->dev, "%s:FW format error\n", __func__);
		kfree(pbt_buf);
		return -EIO;
	}

	return i_ret;
}



/*upgrade from app.bin*/
static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = this_client;

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	disable_irq(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);

	return count;
}

/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO | S_IWUSR, NULL,
			ft5x0x_fwupgradeapp_store);

/*add your attr in here*/
static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftsfwupgradeapp.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};


/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/

/*******************************************************
   ********  1--active  0--not active***********
*******************************************************/
int ft5x06_active(void)
{
	//printk("enter %s\n",__FUNCTION__);
	return 1;
}
/*******************************************************
***********check firmware if need update***********
*******************************************************/
int ft5x06_firmware_need_update(void)
{
	unsigned char uc_fw_ver;
	unsigned char uc_if_ver;
	unsigned int ui_sz;
#if 	FOCALTECH_COB
	unsigned char uc_clb;
#endif

	TW_DBG("enter %s\n",__FUNCTION__);

	if (!current_tp_supported)
		return 0;

	uc_fw_ver = ft5x0x_read_fw_ver();
	TW_DBG("[FT]:new  firmware version = %x",uc_fw_ver);
	ui_sz = current_fw_size;
	if (ui_sz > 2)
		uc_if_ver = current_fw_to_tp[ui_sz - 2];
	else
		uc_if_ver = 0xff;

	TW_DBG("[FT]:host firmware version = %x",uc_if_ver);

#if 	FOCALTECH_COB
	uc_clb = ft5x0x_read_tp_calibrated();
//	      if(uc_clb)
//		return 1;      //changed by taokai


#endif

	if( uc_fw_ver < uc_if_ver || debug_flag)
		return 1;
	else
		return 0;
 }

/**********add for download*********/

#ifdef FT5336_DOWNLOAD

int ft5336_Enter_Debug(void)
{
	/*ft5x0x_reset_tp(0);
	msleep(4);
	ft5x0x_reset_tp(1);*/
    ft5x0x_reset(0);
	return ft5336_Lib_Enter_Download_Mode();
}
//if return 0, main flash is ok, else download.
int ft5336_IsDownloadMain(void)
{
	//add condition to check main flash
	return -1;
}
int ft5336_DownloadMain(void)
{
//	unsigned short fwlen = 0;
	if (ft5336_Enter_Debug() < 0) {
		printk("[fts]-----enter debug mode failed\n");
		return -1;
	}
//	fwlen = sizeof(CTPM_MAIN_FW);
	printk("[fts]-ft5336_DownloadMain---fwlen=%d\n", current_main_fw_size);
//	return 1;
	//return ft6x06_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
	return ft5336_Lib_DownloadMain(current_main_fw_to_tp, current_main_fw_size);
}


/********add for download********/

#endif


/***********add for download*********/
/*******************************************************
*********************do firmware update ***************
*******************************************************/
int ft5x06_firmware_do_update(void)
{
	int ret = 0;
	TW_DBG("enter %s\n",__FUNCTION__);
	if (!current_tp_supported)
		return -1;
#ifdef FT5336_DOWNLOAD
	{
		 if(ft5336_IsDownloadMain() < 0) {
	 	#if 1
				printk("[fts]--------FTS-----enter----download main\n");
				if(ft5336_DownloadMain()<0)
				{
					printk("[fts]---------FTS---------Download main failed\n");
					ret=1;
				}
				else
					ret=0;
		#endif
			 }
		else
				printk("[fts]--------FTS---------no download main\n");

	}
#else
	ret = fts_ctpm_fw_upgrade_with_i_file();
#endif
	if (ret != 0)
		return -1;
	else
		return 0;
}

/*******************************************************
*******************check if need calibrate***********
*******************************************************/

int ft5x06_need_calibrate(void)
{
	TW_DBG("enter %s\n",__FUNCTION__);
	return 1;
}

/*******************************************************
 ******************system write "calibrate"************
*******************************************************/

int ft5x06_calibrate(void)
{
	TW_DBG("enter %s\n",__FUNCTION__);
	mdelay(2000);

	if(!fts_ctpm_auto_clb())
		return 0;
	else
		return -1;
}

/*******************************************************
 ******************get firmware version **************
*******************************************************/
int ft5x06_get_firmware_version(char * version )
{
	unsigned int uc_fw_version;
	TW_DBG("enter %s\n",__FUNCTION__);


	if (!current_tp_supported)
		return sprintf(version, "%s%x:%s","invalid touch panel id:",current_tp_id, "NA");


	uc_fw_version = ft5x0x_read_fw_ver();
	return snprintf(version, BUF_LENGTH, "%s:%s:0x%x:0x%x", current_tp_name,
		FOCALTECH_SOC, uc_fw_version, uc_fw_version);
}

/*******************************************************
  ******************system write "reset"***************
*******************************************************/
int ft5x06_reset_touchscreen(void)
{
	TW_DBG(KERN_ERR "enter %s\n",__FUNCTION__);

	disable_irq_nosync(this_client->irq);

	/*write 0xaa to register 0xfc*/
	ft5x0x_write_reg(0xfc,0xaa);
	delay_qt_ms(50);
	/*write 0x55 to register 0xfc*/
	ft5x0x_write_reg(0xfc,0x55);
	delay_qt_ms(30);
	printk(KERN_ERR "[FTS]: Reset CTPM ok!\n");

	enable_irq(this_client->irq);
	return 1;
}

/*******************************************************
  ******************"handwrite" "normal" *************
*******************************************************/
enum touch_mode_type ft5x06_get_mode(void)
{
	printk("enter %s\n",__FUNCTION__);
	return 1;
}

int ft5x06_set_mode(enum touch_mode_type work_mode)
{
	printk("enter %s\n",__FUNCTION__);
	return 1;
}

/*******************************************************
  ****************get "oreitation:X" ************
*******************************************************/
/* litao3 modify for compile
enum touch_oreintation_type ft5x06_get_oreitation(void)
{
	printk("enter %s\n",__FUNCTION__);
	return 1;
}


int ft5x06_set_oreitation(enum touch_oreintation_type oreitate)
{
	printk("enter %s\n",__FUNCTION__);
	return 1;
}
litao3 end */

/*******************************************************
  **********read buf[256]: ef:ab [???????]?êo??ì  ********
*******************************************************/
int ft5x06_read_regs(char * buf)
{
	int ret;
	u8 reg, val;
	int i = 0;

	printk("enter %s\n",__FUNCTION__);

	for(i=0; i<0xB0; i++)
	{
		reg = i;
		ret = ft5x0x_read_reg(reg, &val);
		printk("CP_Touchscreen: reg0x%x = 0x%x.\n", reg, val);
	}

	return 1;
}

/*******************************************************
  **********write buf[256]: ef:ab [???????]?êo??ì ********
*******************************************************/
int ft5x06_write_regs(const char * buf)
{
	printk("enter %s\n",__FUNCTION__);
	return 1;
}

/*******************************************************
  ***************tw debug on or off *************
*******************************************************/
int ft5x06_debug(int val)
{
	printk("enter %s debug=%d\n",__FUNCTION__,val);
	debug_flag = val;
	return 1;
}
//add by taokai
int focaltech_vendor(char* vendor)
{
	return sprintf(vendor,"%s","focaltech");
}
struct touchscreen_funcs focaltech_ops=
{
	.touch_id					= 0,
	.touch_type			  = 1,
	.active						= ft5x06_active,
	.firmware_need_update		= ft5x06_firmware_need_update,
	.firmware_do_update			= ft5x06_firmware_do_update,
	.need_calibrate				= ft5x06_need_calibrate,
	.calibrate					= ft5x06_calibrate,
	.get_firmware_version		= ft5x06_get_firmware_version,
	.reset_touchscreen			= ft5x06_reset_touchscreen,
/*litao3 modify for compile
	.get_mode					=  ft5x06_get_mode,
	.set_mode					=  ft5x06_set_mode,
	.get_oreitation				= ft5x06_get_oreitation,
	.set_oreitation				= ft5x06_set_oreitation,
modify end */
	.read_regs					= ft5x06_read_regs,
	.write_regs					= ft5x06_write_regs,
	.debug						= ft5x06_debug,
//add by taokai
  .get_vendor     =focaltech_vendor,
};
/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct tw_platform_data *pdata;
	int ret = 0;
	int err_time = 2;
	int i = 0;
	int err = 0;
	TW_DBG("ft5x0x_ts_probe start.\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto exit_check_functionality_failed;
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
		}else

			dev_err(&client->dev, "Success to parse dt\n");
	} else
		pdata = client->dev.platform_data;
#endif
    if(!pdata)
    {
        printk("dev platform data is null.");
        return -ENODEV;
    }

    this_client = client;
#ifdef FT5336_DOWNLOAD
	g_i2c_client= client;
#endif
    ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
    if (!ft5x0x_ts)
    {
        ret = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    input_dev = input_allocate_device();
    if (!input_dev)
    {
        ret = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }
    ft5x0x_ts->input_dev = input_dev;
    ft5x0x_ts->pdata = pdata;//client->dev.platform_data;	  //maleijie
    ft5x0x_ts->irq = gpio_to_irq(ft5x0x_ts->pdata->gpio_irq);
    ft5x0x_ts->client = client;

	SCREEN_MAX_X = ft5x0x_ts->pdata->screen_x;
	SCREEN_MAX_Y = ft5x0x_ts->pdata->screen_y;

	i2c_set_clientdata(client, ft5x0x_ts);

       if (ft5x0x_ts->pdata->init)
       {
            ret = ft5x0x_ts->pdata->init();
		if (ret)
		{
			pr_err("TOUCH:YLLOG:failed to init resource in probe\n");
			goto exit_init;
		}
        }
	if (ft5x0x_ts->pdata->power)
	{
		ret = ft5x0x_ts->pdata->power(1);
		if (ret)
		{
			pr_err("TOUCH:YLLOG:failed to power up in probe\n");
			goto exit_power;
		}
	}
	if (ft5x0x_ts->pdata->reset)
	{
		TW_DBG("reset_delay=%d!\n", ft5x0x_ts->pdata->reset_delay);
		ret = ft5x0x_ts->pdata->reset(ft5x0x_ts->pdata->reset_delay);
		if (ret)
		{
			pr_err("TOUCH:YLLOG:failed to reset in probe\n");
			goto exit_reset;
		}
	}
	spin_lock_init(&ft5x0x_ts->irq_lock);
read_id:
	ret = ft5x0x_read_tp_type();
	current_tp_id = ret;
	current_tp_version = ft5x0x_read_fw_ver();
	TW_DBG("tp_id = 0x%x tp_version = 0x%x!\n",
		current_tp_id, current_tp_version);
	if (ret >= 0)
		ret = fts_check_tp_id(current_tp_id);
	if (ret < 0) {
		pr_err("Don't support update firmware\n");
		current_tp_supported = false;
		fts_get_main_current_tp_info(0);
		fts_get_current_tp_info(0);
		if (err_time-- > 0)
			goto read_id;
		goto exit_reset;
	} else {
		fts_get_current_tp_info(ret);
		fts_get_main_current_tp_info(ret);
	}
#if FOCALTECH_COB
#ifndef FT5336_DOWNLOAD
	if (current_tp_version <= 0x0a || current_tp_version == 0xa6)
		fts_cob_ctpm_fw_upgrade_with_i_file();
	else if (ft5x0x_read_tp_type_() != current_tp_id)
		fts_cob_ctpm_fw_upgrade_with_i_file();
#endif
#endif
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_TRACKING_ID, 0, 5, 0, 0);
	input_set_abs_params(input_dev,
		ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#if CFG_SUPPORT_TOUCH_KEY
	set_bit(EV_KEY, input_dev->evbit);
	for (i = 0; i < ft5x0x_ts->pdata->nbuttons; i++) {
		input_set_capability(input_dev, EV_KEY,
			ft5x0x_ts->pdata->buttons[i].code);
		ft5x0x_ts->pdata->buttons[i].key_status = KEY_RELEASE;
	}
	DEBOUNCE = ft5x0x_ts->pdata->key_debounce;
	if (DEBOUNCE) {
		inDebounce = false;
		setup_timer(&ft5x0x_ts->timer, ft_keys_timer,
			(unsigned long)ft5x0x_ts);
	}
#endif
	input_dev->name = FT5X0X_NAME;
	input_dev->dev.parent = &client->dev;
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "register input dev fail.\n");
		goto exit_input_register_device_failed;
	}
	ft5x0x_ts->ts_workqueue =
		create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		ret = -ESRCH;
		dev_err(&client->dev, "create workqueue failed\n");
		goto exit_create_singlethread;
	}
#if defined(CONFIG_FB)
	ft5x0x_ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ft5x0x_ts->fb_notif);
	if (ret)
		dev_err(&client->dev, "Register fb_noti fail: %d\n", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif
#ifdef FT5336_DOWNLOAD
	FTS_I2c_Read_Function fun_i2c_read = ft5x0x_download_i2c_Read;
	FTS_I2c_Write_Function fun_i2c_write = ft5x0x_download_i2c_Write;
	Init_I2C_Read_Func(fun_i2c_read);
	Init_I2C_Write_Func(fun_i2c_write);
#endif
	err = fts_ctpm_chk_upg_probe();
/*	if (err == 1) {
		TW_DBG("down load update firmware.\n");
		ft5x06_firmware_do_update();
	}
*/	mutex_init(&ft5x0x_ts->device_mode_mutex);
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	ret = request_irq(ft5x0x_ts->irq, ft5x0x_ts_interrupt,
		ft5x0x_ts->pdata->irqflag, client->dev.driver->name, ft5x0x_ts);
	if (ret < 0) {
		dev_err(&client->dev, "request irq failed\n");
		goto exit_irq_request_failed;
	} else
		TW_irq_disable(ft5x0x_ts);
	touch_flag = 0;
	touchscreen_set_ops(&focaltech_ops);
	ret = sysfs_create_group(&client->dev.kobj,
		&ft5x0x_attribute_group);
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_READ_CHARGE_STATUS
	ft5x16_rd_chg_wq = create_singlethread_workqueue("ft5x16_timer_wq");
	if (!ft5x16_rd_chg_wq) {
		ret = -ESRCH;
		dev_err(&client->dev, "create workqueue failed\n");
		goto exit_irq_request_failed;
	}
	INIT_DELAYED_WORK(&ft5x16_rd_chg_work, ft5x16_read_chg_work_func);
	ft5x16_init_complete = 1;
	queue_delayed_work(ft5x16_rd_chg_wq,
		&ft5x16_rd_chg_work,
		msecs_to_jiffies(TW_CHARGING_STATUS_CHECK_CIRCLE));
#endif
	sleep_flag = resume_mode;
	TW_irq_enable(ft5x0x_ts);
	TW_DBG("ft5x0x_ts_probe success!\n");
	return 0;


exit_irq_request_failed:
    destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
    input_unregister_device(input_dev);
exit_input_register_device_failed:
exit_reset:
exit_power:
    ft5x0x_ts->pdata->release();
exit_init:
    input_free_device(input_dev);
exit_input_dev_alloc_failed:
    i2c_set_clientdata(client, NULL);
    kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
return ret;
}
/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	int ret;

	ft5x0x_ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
#ifdef CONFIG_TOUCHSCREEN_FOCALTECH_READ_CHARGE_STATUS
	cancel_delayed_work_sync(&ft5x16_rd_chg_work);
	destroy_workqueue(ft5x16_rd_chg_wq);
#endif
	mutex_destroy(&ft5x0x_ts->device_mode_mutex);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);

	if (gpio_is_valid(ft5x0x_ts->pdata->gpio_reset))
		gpio_free(ft5x0x_ts->pdata->gpio_reset);
	if (gpio_is_valid(ft5x0x_ts->pdata->gpio_irq))
		gpio_free(ft5x0x_ts->pdata->gpio_irq);

	if (ft5x0x_ts->ts_pinctrl) {
		ret = focaltech_ts_pinctrl_select(ft5x0x_ts, false);
		if (ret < 0)
			dev_err(&client->dev,"Cannot get idle pinctrl state\n");
	}
	focaltech_ts_power(0);
	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}
static void ft5x0x_ts_shutdown(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;

	ft5x0x_ts = i2c_get_clientdata(client);
	gpio_direction_output(ft5x0x_ts->pdata->gpio_reset, 0);
}
#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static const struct i2c_device_id ft5x0x_ts_id[] = {
    { FT5X0X_NAME, 0 },{ }
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);
static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.id_table	= ft5x0x_ts_id,
	.shutdown	= ft5x0x_ts_shutdown,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ft5x06_match_table,
		.pm	= &ft5x0x_ts_pmops,
	},
};

/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static int __init ft5x0x_ts_init(void)
{
    int ret;
    ret = i2c_add_driver(&ft5x0x_ts_driver);
    return ret;

}

/***********************************************************************************************
Name	:

Input	:

Output	:

function	:

***********************************************************************************************/
static void __exit ft5x0x_ts_exit(void)
{
    i2c_del_driver(&ft5x0x_ts_driver);

}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
