/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include "uei1700.h"

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/notifier.h>

#define DEVICE_NAME				"irremote_uei1700"
#define SYSFS_REMOTE_PATH   "remote"

static struct ir_remocon_data	*ir_data;

struct class *uei1700_class;

static int power_supply_gpio = 1003;

static int uei1700_remote_open(struct inode *inode, struct file *file)
{
	pr_info("uei1700---open OK---\n");
	return 0;
}

static int uei1700_remote_close(struct inode *inode, struct file *file)
{
	pr_info("uei1700---close---\n");
	return 0;
}


static const struct file_operations uei1700_remote_ops = {
	.owner			= THIS_MODULE,
	.open			= uei1700_remote_open,
	.release		= uei1700_remote_close,
};

static ssize_t power_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;

	sscanf(buf, "%d", &data);
	pr_info("%s:data = %d\n", __func__, data);
	if (data <= 0)
		gpio_set_value(power_supply_gpio, 0);
	else
		gpio_set_value(power_supply_gpio, 1);

	return count;
}

static ssize_t power_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char buf_ir_test = 0;

	buf_ir_test = gpio_get_value(power_supply_gpio);
	len += sprintf(buf + len, "%d", buf_ir_test);
	return  len;
}

static ssize_t vendor_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *ir_remote_vendor = NULL;

	ir_remote_vendor = "UEI:UEI-iTwo";
	len += sprintf(buf + len, "%s\n", ir_remote_vendor);
	return  len;
}
static struct kobj_attribute uei1700_attrs[] = {
	__ATTR(enable, (S_IRUGO | S_IWUSR | S_IWGRP),
					power_show,
					power_store),
	__ATTR(vendor, (S_IRUGO | S_IWUSR | S_IWGRP),
					vendor_show,
					NULL),
};

static struct miscdevice uei1700_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &uei1700_remote_ops,
};

static int uei1700_gpio_init(void)
{
	int ret;

	ret = gpio_request(ir_data->uei1700_power_supply,
				"uei1700_power_supply");
	if (ret) {
		pr_err("request uei1700_power_supply(%d) gpio failed, rc=%d\n",
					ir_data->uei1700_power_supply, ret);
	} else{
		ret = gpio_direction_output(ir_data->uei1700_power_supply, 1);
		if (ret) {
			pr_err("%s: Set direction for uei1700_power_supply failed, ret=%d\n",
				__func__, ret);
	    }

	}
	return 0;
}
static int uei1700_dev_init(void)
{
	int rc;
	int i, j;

	ir_data = kzalloc(sizeof(struct ir_remocon_data), GFP_KERNEL);
	if (NULL == ir_data) {
		pr_info("Failed to data allocate %s\n", __func__);
		rc = -ENOMEM;
		goto err_free_mem;
	}
	ir_data->uei1700_obj = kobject_create_and_add(SYSFS_REMOTE_PATH, NULL);
	if (!ir_data->uei1700_obj) {
		pr_info("Failed to create uei1700_obj\n");
		rc = -ENOMEM;
		goto err_free_mem;
	}
	for (i = 0; i < 2; i++) {
		rc = sysfs_create_file(ir_data->uei1700_obj,
					&uei1700_attrs[i].attr);
		if (rc) {
			pr_info("Failed to create uei1700 device file!\n");
			rc = -ENOMEM;
			goto err_create_sysfs;
		}
	}
	if ((rc = misc_register(&uei1700_misc_dev))) {
		pr_info("uei1700: misc_register register failed\n");
		rc = -ENOMEM;
		goto err_misc_register;
	}
	return 0;
err_misc_register:
		misc_deregister(&uei1700_misc_dev);
err_create_sysfs:
		for (j = 0; j <= i; j++)
			sysfs_remove_file(ir_data->uei1700_obj,
					&uei1700_attrs[j].attr);
err_free_mem:
		kfree(ir_data);

	return rc;
}

static int uei1700_fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ir_remocon_data *ir_remote_data =
		container_of(self, struct ir_remocon_data, fb_notif);

	if (evdata && evdata->data && ir_remote_data) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
				gpio_set_value(power_supply_gpio, 1);
			else if (*blank == FB_BLANK_POWERDOWN)
				gpio_set_value(power_supply_gpio, 0);
		}
	}

	return 0;
}

static int  uei1700_misc_dev_probe(struct platform_device *pdev)
{

	int ret, rc;

	if (pdev->dev.of_node) {
		ir_data = devm_kzalloc(&pdev->dev,
		sizeof(struct ir_remocon_data), GFP_KERNEL);
			if (!ir_data) {
				dev_err(&pdev->dev,
					"uei1700 Failed to allocate memory for pdata\n");
				return -ENOMEM;
			}
	}
	ir_data->uei1700_power_supply = of_get_named_gpio(pdev->dev.of_node,
						"uei1700,power-gpio", 0);
	power_supply_gpio = ir_data->uei1700_power_supply;
	if (!gpio_is_valid(ir_data->uei1700_power_supply)) {
		pr_err("%s: uei1700 power gpio not specified\n", __func__);
	} else{

	ret = uei1700_gpio_init();
		if (ret) {
			pr_err("uei1700_gpio_init failed! ret = %d\n", ret);
			return ret;
		}
	}

	ret = uei1700_dev_init();
	if (ret) {
		pr_err("uei1700_dev_init failed! ret = %d\n", ret);
		return ret;
	}

	ir_data->fb_notif.notifier_call = uei1700_fb_notifier_callback;
	rc = fb_register_client(&ir_data->fb_notif);
	if (rc)
		pr_err("failed to register fb_notifier!\n");

	return 0;
}

static struct of_device_id uei1700_misc_dev_match_table[] = {
	{.compatible = "uei,uei1700"},
	{}
};
static struct platform_driver uei1700_misc_dev_driver = {
	.probe = uei1700_misc_dev_probe,
	.driver = {
		.name = "uei,uei1700",
		.of_match_table = uei1700_misc_dev_match_table,
	},
};

static int __init uei1700_remote_dev_init(void)
{
	pr_info("%s:E\n", __func__);
	return platform_driver_register(&uei1700_misc_dev_driver);
}

static void __exit uei1700_remote_dev_exit(void)
{
	platform_driver_unregister(&uei1700_misc_dev_driver);
}

module_init(uei1700_remote_dev_init);
module_exit(uei1700_remote_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("UEI1700 Inc.");
MODULE_DESCRIPTION("uei1700 consumerir  Driver");

