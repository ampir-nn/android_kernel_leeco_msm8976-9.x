/************************************************************
**
**  Copyright (C), 2013-2015, Yulong Tech. Co., Ltd.
**  FileName:leds-ktd.c
**  Description:Linux device driver for ktd2xxx rgb led driver
**  Author:
**  Version:1.00
**  Date:2013-09-13
**
**************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/leds.h>
#include <linux/of.h>

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES			5
#define KTD_LEDS_DEV_NAME        "ktd_leds"
#define DEBUG_TRICOLOR_LED		1


enum ktd_led_color {
	LED_COLOR_RED,
	LED_COLOR_GREEN,
	LED_COLOR_BLUE,
	LED_COLOR_DEFAULT,
};

struct ktd_leds_data {
	struct i2c_client *client;
	struct mutex lock;
	unsigned int ldo_ctl_flag;
	unsigned int led_flag[4];
	int led_data[3];
	struct led_classdev leds[3];
	struct regulator *rgb_vdd;
	/*add for save rgb leds statu
	reg4	0b 00  00  00  00
		       G   R   B
	Register Reg4 sets the mode of each LED channel to either always ON/OFF or PWM1/PWM2
			00   always off
			01   always on
			10   pwm1
			11   pwm2	*/
	unsigned char rgb_status;
	unsigned int imax_level;

};

struct ktd_leds_data *ktd_data;

static int ktd_leds_i2c_write_reg(u8 regnum, u8 value)
{
	int32_t ret = -1;
	uint8_t tries = 128;
	int k;
	/*
	 * i2c_smbus_write_byte_data - SMBus "write byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 * @value: Byte being written
	 *
	 * This executes the SMBus "write byte" protocol,
	 * returning negative errno
	 * else zero on success.
	 */

	ret = i2c_smbus_write_byte_data(ktd_data->client, regnum, value);

	if (ret < 0) {
		pr_err("%s: write regnum=%d fail(addr=%d)\n", __func__,
			regnum, ktd_data->client->addr);

		for (k = 0; k < tries; k++) {
			if ((k == 0x04) || (k == 0x05) || (k == 0x06)
				|| (k == 0x07) || (k == 0x0c) || (k == 0x0e)
				|| (k == 0x18) || (k == 0x1e) || (k == 0x23)
				|| (k == 0x39) || (k == 0x45) || (k == 0x68)) {
				continue;
			}

			ktd_data->client->addr = k;
			ret = i2c_smbus_write_byte_data(ktd_data->client, regnum, value);
			if (ret < 0) {
				pr_debug("%s: write fail addr=k=%d\n", __func__, k);
				usleep(100);
			} else {
				pr_err("%s: write success addr=k=%d\n", __func__, k);
				break;
			}
		}

		ret = i2c_smbus_write_byte_data(ktd_data->client, 0x00, 0x07);
		pr_err("%s: reset addr=%d, ret=%d\n", __func__, ktd_data->client->addr, ret);
		usleep(2000);

		ktd_data->client->addr = 0x30;
		ret = i2c_smbus_write_byte_data(ktd_data->client, regnum, value);
		if (ret < 0) {
			pr_err("%s: write fail addr=%d\n", __func__, ktd_data->client->addr);
			ktd_data->client->addr = k;
		} else {
			pr_err("%s: write success addr=%d\n", __func__, ktd_data->client->addr);
		}
	}

	return ret;

}


static void ktd_leds_dev_init(struct ktd_leds_data *data)
{
/*
	int ret;
	int ledgain;
	printk(KERN_INFO "ktd_leds:%s ", __func__);
	ktd_leds_i2c_write_reg(ktd_GCR, 0x01);

	ktd_leds_i2c_write_reg(ktd_LEDE, 0x00);

	ktd_leds_i2c_write_reg(ktd_LED0_CTRL, 0x01);
	ktd_leds_i2c_write_reg(ktd_LED1_CTRL, 0x01);
	ktd_leds_i2c_write_reg(ktd_LED2_CTRL, 0x02);
	ktd_leds_i2c_write_reg(ktd_PWM0, 0xff);
	ktd_leds_i2c_write_reg(ktd_PWM1, 0xff);
	ktd_leds_i2c_write_reg(ktd_PWM2, 0xff);

	ktd_leds_i2c_write_reg(ktd_LED0_T0, Rise_time << 4 | Hold_time);
	ktd_leds_i2c_write_reg(ktd_LED0_T1, Fall_time << 4 | Off_time);
	ktd_leds_i2c_write_reg(ktd_LED0_T2, Delay_time << 4 | Period_Num);

	ktd_leds_i2c_write_reg(ktd_LED1_T0, Rise_time << 4 | Hold_time);
	ktd_leds_i2c_write_reg(ktd_LED1_T1, Fall_time << 4 | Off_time);
	ktd_leds_i2c_write_reg(ktd_LED1_T2, Delay_time << 4 | Period_Num);

	ktd_leds_i2c_write_reg(ktd_LED2_T0, Rise_time << 4 | Hold_time);
	ktd_leds_i2c_write_reg(ktd_LED2_T1, Fall_time << 4 | Off_time);
	ktd_leds_i2c_write_reg(ktd_LED2_T2, Delay_time << 4 | Period_Num);
*/
	ktd_data->rgb_status =0;
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);
	msleep(20);
}

static void blue_led_blink(void)
{
		/*
	 * RED flash time period: 2.5s, rise/fall 1s, sleep 0.5s
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	ktd_data->rgb_status &= 0b11111100;
	ktd_leds_i2c_write_reg(0x04, 0x00);// initialization LED off
	ktd_leds_i2c_write_reg(0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	ktd_leds_i2c_write_reg(0x06, ktd_data->imax_level);//set current
	ktd_leds_i2c_write_reg(0x05, 0xaa);//rase time
	ktd_leds_i2c_write_reg(0x01, 0x12);//dry flash period
	ktd_leds_i2c_write_reg(0x02, 0x00);//reset internal counter
	ktd_leds_i2c_write_reg(0x04, 0x02);//allocate led1 to timer1
	ktd_leds_i2c_write_reg(0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

static void blue_led_off(void)
{
	ktd_data->rgb_status &= 0b11111100;
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);//Device OFF-Either SCL goes low or SDA stops toggling
}

static void red_led_blink(void)
{
	/*
	 * Green flash time period: 2.5s, rise/fall 1s, sleep 0.5s
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	ktd_data->rgb_status &= 0b11110011;
	ktd_leds_i2c_write_reg(0x04, 0x00);// initialization LED off
	ktd_leds_i2c_write_reg(0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	ktd_leds_i2c_write_reg(0x07, ktd_data->imax_level);//set current
	ktd_leds_i2c_write_reg(0x05, 0xaa);//rase time
	ktd_leds_i2c_write_reg(0x01, 0x12);//dry flash period
	ktd_leds_i2c_write_reg(0x02, 0x00);//reset internal counter
	ktd_leds_i2c_write_reg(0x04, 0x08);//allocate led1 to timer1
	ktd_leds_i2c_write_reg(0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

static void red_led_off(void)
{
	ktd_data->rgb_status &= 0b11110011;
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);//Device OFF-Either SCL goes low or SDA stops toggling
}

static void green_led_blink(void)
{
	/*
	 * blue flash time period: 2.5s, rise/fall 1s, sleep 0.5s
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	ktd_data->rgb_status &= 0b11001111;
	ktd_leds_i2c_write_reg(0x04, 0x00);// initialization LED off
	ktd_leds_i2c_write_reg(0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	ktd_leds_i2c_write_reg(0x08, ktd_data->imax_level);//set current
	ktd_leds_i2c_write_reg(0x05, 0xaa);//rase time
	ktd_leds_i2c_write_reg(0x01, 0x12);//dry flash period
	ktd_leds_i2c_write_reg(0x02, 0x00);//reset internal counter
	ktd_leds_i2c_write_reg(0x04, 0x20);//allocate led1 to timer1
	ktd_leds_i2c_write_reg(0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

static void green_led_off(void)
{
	ktd_data->rgb_status &= 0b11001111;
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);//Device OFF-Either SCL goes low or SDA stops toggling
}

static void blue_led_on(void)
{
	ktd_data->rgb_status &= 0b11111100;
	ktd_data->rgb_status |= 0b00000001;
	ktd_leds_i2c_write_reg(0x04, 0x00);//initialization LED off
	ktd_leds_i2c_write_reg(0x06, ktd_data->imax_level);//set current
	ktd_leds_i2c_write_reg(0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);//turn on led
}
static void red_led_on(void)
{
	ktd_data->rgb_status &= 0b11110011;
	ktd_data->rgb_status |= 0b00000100;
	ktd_leds_i2c_write_reg(0x04, 0x00);//initialization LED off
	ktd_leds_i2c_write_reg(0x07, ktd_data->imax_level);//set current
	ktd_leds_i2c_write_reg(0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);//turn on led
}
static void green_led_on(void)
{
	ktd_data->rgb_status &= 0b11001111;
	ktd_data->rgb_status |= 0b00010000;
	ktd_leds_i2c_write_reg(0x04, 0x00);//initialization LED off
	ktd_leds_i2c_write_reg(0x08, ktd_data->imax_level);//set current
	ktd_leds_i2c_write_reg(0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	ktd_leds_i2c_write_reg(0x04, ktd_data->rgb_status);//turn on led
}

static ssize_t led_blink_solid_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long enable = 0;
	enum ktd_led_color color = LED_COLOR_DEFAULT;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_leds_data *ktd_led_data = NULL;

	if (!strcmp(led_cdev->name, "red"))
		color = LED_COLOR_RED;
	else if (!strcmp(led_cdev->name, "green"))
		color = LED_COLOR_GREEN;
	else if (!strcmp(led_cdev->name, "blue"))
		color = LED_COLOR_BLUE;
	ktd_led_data = container_of(led_cdev, struct ktd_leds_data, leds[color]);
	if (!ktd_led_data)
		printk(KERN_ERR "%s ktd_led_data is NULL ", __func__);
	/*sscanf(buf, "%d", &blink);*/
	enable = simple_strtoul(buf, NULL, 10);
#if DEBUG_TRICOLOR_LED
	printk("tricolor %s is %ld\n", led_cdev->name, enable);
#endif
	mutex_lock(&ktd_led_data->lock);

	if (enable) {
		switch (color) {
		case LED_COLOR_RED:
			ktd_data->led_flag[0] = 1;
			red_led_blink();
			break;
		case LED_COLOR_GREEN:
			ktd_data->led_flag[1] = 1;
			green_led_blink();
			break;
		case LED_COLOR_BLUE:
			ktd_data->led_flag[2] =1;
			blue_led_blink();
			break;
		default:
			break;
		}
	} else {
		switch (color) {
		case LED_COLOR_RED:
			ktd_data->led_flag[0] = 0;
			red_led_off();
			break;
		case LED_COLOR_GREEN:
			ktd_data->led_flag[1] = 0;
			green_led_off();
			break;
		case LED_COLOR_BLUE:
			ktd_data->led_flag[2] = 0;
			blue_led_off();
			break;
		default:
			break;
		}
	}
	ktd_led_data->led_data[color] = enable;
	mutex_unlock(&ktd_led_data->lock);
	return size;
}
static ssize_t led_blink_solid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	enum ktd_led_color color = LED_COLOR_DEFAULT;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd_leds_data *ktd_led_data = NULL;

	if (!strcmp(led_cdev->name, "red"))
		color = LED_COLOR_RED;
	else if (!strcmp(led_cdev->name, "green"))
		color = LED_COLOR_GREEN;
	else if (!strcmp(led_cdev->name, "blue"))
		color = LED_COLOR_BLUE;
	ktd_led_data = container_of(led_cdev, struct ktd_leds_data, leds[color]);
	if (!ktd_led_data)
		printk(KERN_ERR "%s tricolor_led is NULL ", __func__);
	ret = sprintf(buf, "%u\n", ktd_led_data->led_data[color]);
	return ret;
}

static void led_brightness_set_tricolor(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct ktd_leds_data *ktd_led_data = NULL;
	enum ktd_led_color color = LED_COLOR_DEFAULT;

	printk(KERN_DEBUG "%s ******start*******\n", __func__);
	printk(KERN_DEBUG "brightness value is %d\n ", brightness);
	if (!strcmp(led_cdev->name, "red"))
		color = LED_COLOR_RED;
	else if (!strcmp(led_cdev->name, "green"))
		color = LED_COLOR_GREEN;
	else if (!strcmp(led_cdev->name, "blue"))
		color = LED_COLOR_BLUE;
	ktd_led_data = container_of(led_cdev, struct ktd_leds_data, leds[color]);
	if (!ktd_led_data)
		printk(KERN_ERR "%s tricolor_led is NULL ", __func__);

	mutex_lock(&ktd_led_data->lock);
	if (brightness) {
		switch (color) {
		case LED_COLOR_RED:
			printk(KERN_DEBUG "*****LED_COLOR_RED  OPEN *****\n ");
			ktd_data->led_flag[0] = 1;
			red_led_on();
			break;
		case LED_COLOR_GREEN:
			printk(KERN_DEBUG "*****LED_COLOR_GREEN OPEN*****\n ");
			ktd_data->led_flag[1] = 1;
			green_led_on();
			break;
		case LED_COLOR_BLUE:
			printk(KERN_DEBUG "*****LED_COLOR_BLUE OPEN*****\n ");
			ktd_data->led_flag[2] = 1;
			blue_led_on();
			break;
		default:
			break;
		}
	} else {
		switch (color) {
		case LED_COLOR_RED:
			printk(KERN_DEBUG "*****LED_COLOR_RED  CLOSED*****\n ");
			ktd_data->led_flag[0] = 0;
			red_led_off();
			break;
		case LED_COLOR_GREEN:
			printk(KERN_DEBUG "*****LED_COLOR_GREEN  CLOSED*****\n ");
			ktd_data->led_flag[1] = 0;
			green_led_off();
			break;
		case LED_COLOR_BLUE:
			printk(KERN_DEBUG "*****LED_COLOR_BLUE  CLOSED*****\n ");
			ktd_data->led_flag[2] = 0;
			blue_led_off();
			break;
		default:
			break;
		}
	}
	printk(KERN_DEBUG "%s ******end*******\n", __func__);
	mutex_unlock(&ktd_led_data->lock);
}

static DEVICE_ATTR(blink, 0644,
	led_blink_solid_show, led_blink_solid_store);

static int ktd_led_resume(struct i2c_client *client)
{
/*
	int flag;
	flag = ktd_data->ldo_ctl_flag;
	ktd_led_power_on(ktd_data);
	ktd_leds_dev_init(ktd_data);
*/
	return 0;
}


static int ktd_led_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;

	if (ktd_data->led_flag[0] || ktd_data->led_flag[1] ||
		ktd_data->led_flag[2] || ktd_data->led_flag[3])
		return 0;
	else {
		ret = ktd_leds_i2c_write_reg(0x00, 0x08);
		if (ret) {
			printk(KERN_ERR "ktd_led_suspend:ktd_led suspend error\n");
			return 0;
		}
		return ret;
	}
}

/* extern int boot_charger_flags; */

static int ktd_leds_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	int i, j;
	struct device_node *node = client->dev.of_node;

	printk(KERN_DEBUG "[ktd_leds]...func:%s...probe start....\n", __func__);
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		printk(KERN_ERR "%s: ktd_LEDS functionality check failed.\n",
			__func__);
		ret = -EIO;
		goto out;
	}

	printk(KERN_DEBUG "ktd_leds i2c check success\n");
	ktd_data = kzalloc(sizeof(struct ktd_leds_data), GFP_KERNEL);
	if (ktd_data == NULL) {
		printk(KERN_ERR "%s: ktd_LEDS kzalloc failed.\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	mutex_init(&ktd_data->lock);
	ktd_data->ldo_ctl_flag = 0;
	ktd_data->client = client;
	ktd_leds_dev_init(ktd_data);
	ktd_data->rgb_status = 0;
	/* ret = sysfs_create_group(&client->dev.kobj,&ktd_attribute_group); */
	ktd_data->leds[0].name = "red";
	ktd_data->leds[0].brightness_set = led_brightness_set_tricolor;
	ktd_data->leds[1].name = "green";
	ktd_data->leds[1].brightness_set = led_brightness_set_tricolor;
	ktd_data->leds[2].name = "blue";
	ktd_data->leds[2].brightness_set = led_brightness_set_tricolor;
	i2c_set_clientdata(client, ktd_data);

	for (i = 0; i < 3; i++) {
		ret = led_classdev_register(&client->dev, &ktd_data->leds[i]);
		if (ret < 0) {
			printk(KERN_ERR "ktd_leds: led_classdev_register failed\n");
			goto err_led_classdev_register_failed;
		}
	}

	for (i = 0; i < 3; i++) {
		ret = device_create_file(ktd_data->leds[i].dev, &dev_attr_blink);
		if (ret < 0) {
			printk(KERN_ERR "tricolor_led: device_create_file failed\n");
			goto err_out_attr_blink;
		}
	}

	ktd_data->rgb_vdd = regulator_get(&ktd_data->client->dev, "rgb_led");
	if (IS_ERR(ktd_data->rgb_vdd)) {
		printk(KERN_INFO "ldo15 is not configured as rgb power supply\n");
	} else {
		ret = regulator_set_voltage(ktd_data->rgb_vdd, 2850000, 2850000);
		ret = regulator_enable(ktd_data->rgb_vdd);
		printk(KERN_INFO "rgb led is powered by l15\n");
	}
	printk(KERN_DEBUG "[ktd_leds]...func:%s...probe end....\n", __func__);

	ret = of_property_read_u32(node, "ktd-leds,imax-level",
                               &ktd_data->imax_level);
	if (ret) {
		pr_err("ret =%d read imax failed, use default setting\n", ret);
		ktd_data->imax_level = 0x77; /* 15mA */
	}
	pr_info ("imax_level is %d\n",((ktd_data->imax_level+1) >> 3));

	return 0;

err_out_attr_blink:
	for (j = 0; j < i; j++)
		device_remove_file(ktd_data->leds[j].dev, &dev_attr_blink);

err_led_classdev_register_failed:
	for (j = 0; j < i; j++)
		led_classdev_unregister(&ktd_data->leds[j]);

out:
	kfree(ktd_data);

	return 0;
}

static int /*__devexit */ktd_leds_remove(struct i2c_client *client)
{
	return 0;
}

static void ktd_shutdown(struct i2c_client *client)
{
	ktd_leds_i2c_write_reg(0x04, 0);
}

static const struct i2c_device_id ktd_leds_id[] = {
	{KTD_LEDS_DEV_NAME, 0}, { }, };


MODULE_DEVICE_TABLE(i2c, ktd_leds_id);

#ifdef CONFIG_OF
static const struct of_device_id ktd_of_match[] = {
	{.compatible = "ktd-leds", },
	{ },
};
MODULE_DEVICE_TABLE(of, ktd_of_match);
#else
#define ktd_of_match NULL
#endif
static struct i2c_driver ktd_leds_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = KTD_LEDS_DEV_NAME,
		#ifdef CONFIG_OF
			.of_match_table = ktd_of_match,
		#endif
	},
	.probe = ktd_leds_probe,
	.remove = /*__devexit_p*/(ktd_leds_remove),
	.suspend = ktd_led_suspend,
	.resume = ktd_led_resume,
	.id_table = ktd_leds_id,
	.shutdown = ktd_shutdown,

};

static int __init ktd_leds_init(void)
{
	return i2c_add_driver(&ktd_leds_driver);

}
static void __exit ktd_leds_exit(void)
{
	i2c_del_driver(&ktd_leds_driver);
	return;
}

late_initcall(ktd_leds_init);
module_exit(ktd_leds_exit);

MODULE_DESCRIPTION("KTD three color leds sysfs driver");
MODULE_AUTHOR("yulong");
MODULE_LICENSE("GPL");
