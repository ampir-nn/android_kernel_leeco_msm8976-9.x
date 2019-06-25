
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>

#define LED_GPIO_FLASH_DRIVER_NAME	"rohm_flash,sgm3780"
#define LED_TRIGGER_DEFAULT		"none"

struct led_sgm3780_flash_data {
	int flash_en;
	int torch_en;
	int brightness;
	struct led_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
};


static struct of_device_id led_sgm3780_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

static void led_sgm3780_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	int rc = 0;
	struct led_sgm3780_flash_data *flash_led =
	    container_of(led_cdev, struct led_sgm3780_flash_data, cdev);

	int brightness = value;
	int flash_en = 0, torch_en = 0;

	if (brightness > LED_HALF) {
		flash_en = 1;
		torch_en = 0;
	} else if (brightness > LED_OFF) {
		flash_en = 0;
		torch_en = 1;
	} else {
		flash_en = 0;
		torch_en = 0;
	}

	pr_err("CHYL E %s brightness=%d flash_en=%d torch_en=%d\n",
		__func__, brightness, flash_en, torch_en);

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_en);
		goto err;
	}
	rc = gpio_direction_output(flash_led->torch_en, torch_en);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->torch_en);
		goto err;
	}
	flash_led->brightness = brightness;
err:
	return;
}

static enum led_brightness led_sgm3780_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_sgm3780_flash_data *flash_led =
	    container_of(led_cdev, struct led_sgm3780_flash_data, cdev);
	return flash_led->brightness;
}

int led_sgm3780_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	const char *temp_str;
	struct led_sgm3780_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;
	flash_led = devm_kzalloc(&pdev->dev,
		sizeof(struct led_sgm3780_flash_data), GFP_KERNEL);
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger", &temp_str);
	if (!rc)
		flash_led->cdev.default_trigger = temp_str;

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"sgm3780_flash_default");
	if (IS_ERR(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);

	flash_led->flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);
	if (flash_led->flash_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-en", node->full_name, flash_led->flash_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_en, rc);

			goto error;
		}

		rc = gpio_direction_output(flash_led->flash_en, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->flash_en);
		}
	}

	flash_led->torch_en = of_get_named_gpio(node, "qcom,torch-en", 0);
	if (flash_led->torch_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"torch-en", node->full_name, flash_led->torch_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->torch_en, "TORCH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->torch_en, rc);
			goto error;
		}

		rc = gpio_direction_output(flash_led->torch_en, 0);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
				flash_led->torch_en);
		}
	}

	rc = of_property_read_string(node, "linux,name", &flash_led->cdev.name);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to read linux name. rc = %d\n",
			__func__, rc);
		goto error;
	}

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_FULL;
	flash_led->cdev.brightness_set = led_sgm3780_brightness_set;
	flash_led->cdev.brightness_get = led_sgm3780_brightness_get;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	pr_err("%s:probe successfully!\n", __func__);
	return 0;

error:
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int led_sgm3780_flash_remove(struct platform_device *pdev)
{
	struct led_sgm3780_flash_data *flash_led =
	    (struct led_sgm3780_flash_data *)platform_get_drvdata(pdev);

	led_classdev_unregister(&flash_led->cdev);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_sgm3780_flash_driver = {
	.probe = led_sgm3780_flash_probe,
	.remove = led_sgm3780_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_sgm3780_flash_of_match,
		   }
};

static int __init led_sgm3780_flash_init(void)
{
	return platform_driver_register(&led_sgm3780_flash_driver);
}

static void __exit led_sgm3780_flash_exit(void)
{
	return platform_driver_unregister(&led_sgm3780_flash_driver);
}

late_initcall(led_sgm3780_flash_init);
module_exit(led_sgm3780_flash_exit);

MODULE_DESCRIPTION("sgm3780 LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds_sgm3780_flash");
