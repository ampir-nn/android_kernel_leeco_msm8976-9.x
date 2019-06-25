/**
 *
 * Copyright (c) 2012-2017  YULONG Company
 *
 * PROPRIETARY RIGHTS of YULONG Company are involved in the
 * subject matter of this material.  All manufacturing, reproduction, use,
 * and sales rights pertaining to this subject matter are governed by the
 * license agreement.  The recipient of this software implicitly accepts
 * the terms of the license.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/input/touchscreen_yl.h>
#include <linux/power_supply.h>

#define TSFS_INFO(fmt, arg...) \
	pr_info("CP_Touchscreen %s: "fmt"\n", __func__, ##arg)
#define TSFS_WARN(fmt, arg...) \
	pr_warn("CP_Touchscreen %s "fmt"\n", __func__, ##arg)
#define TSFS_ERR(fmt, arg...) \
	pr_err("CP_Touchscreen %s "fmt"\n", __func__, ##arg)
#define TSFS_DBG(fmt, arg...) \
	pr_debug("CP_Touchscreen %s: "fmt"\n", __func__, ##arg)

/****************************************************
*   charger status detect
*****************************************************/
void (*touch_charger_notify)(void);
static int charger_status = 0;

/****************************************************
 *		0 charger offline
 *		1 charger online
 *****************************************************/
unsigned int touch_get_charger_status(void)
{
	if (charger_status)
		return 1;
	else
		return 0;
}
void touch_register_charger_notify(void (*fn)(void))
{
	touch_charger_notify = fn;
}
void touch_unregister_charger_notify(void)
{
	touch_charger_notify = NULL;
}
void touch_charger_status_changed(struct power_supply *psy)
{
	struct power_supply *psy1;
	static unsigned int pre_chg_status = 2;

	pr_debug("%s: enter touch_charger_status_changed(), psy->type = %d.\n",
			__func__, psy->type);

	if (psy->type != POWER_SUPPLY_TYPE_USB &&
			psy->type != POWER_SUPPLY_TYPE_UNKNOWN &&
			psy->type != POWER_SUPPLY_TYPE_MAINS &&
			psy->type != POWER_SUPPLY_TYPE_USB_DCP &&
			psy->type != POWER_SUPPLY_TYPE_USB_CDP &&
			psy->type != POWER_SUPPLY_TYPE_USB_ACA)
		return;
	psy1 = power_supply_get_by_name("battery");
	if (psy1 == NULL)
		return;
	charger_status = power_supply_am_i_supplied(psy1);
	pr_debug("%s:charger_status=%d pre_status=%d\n", __func__, charger_status, pre_chg_status);
	if (pre_chg_status == charger_status)
		return;
	pre_chg_status = charger_status;

	if (touch_charger_notify)
		touch_charger_notify();
}
/**********************************************************************
*end
**********************************************************************/
struct touchscreen_funcs *touchscreen_ops[2];

#define TOUCH_IN_ACTIVE(num) (touchscreen_ops[num] && \
	touchscreen_ops[num]->active && \
	touchscreen_ops[num]->active())

static DEFINE_MUTEX(touchscreen_mutex);

int touchscreen_set_ops(struct touchscreen_funcs *ops)
{
	if (!ops || ops->touch_id > 1) {
		TSFS_ERR("ops error!");
		return -EBUSY;
	}
	mutex_lock(&touchscreen_mutex);
	if (touchscreen_ops[ops->touch_id]) {
		TSFS_WARN("ops has been used!");
		mutex_unlock(&touchscreen_mutex);
		return -EBUSY;
	}
	touchscreen_ops[ops->touch_id] = ops;
	mutex_unlock(&touchscreen_mutex);
	TSFS_DBG("ops add success!");
	return 0;
}

static ssize_t ft5x0x_rawdata_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->get_rawdata)
			ret = touchscreen_ops[0]->get_rawdata(buf);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->get_rawdata)
			ret = touchscreen_ops[1]->get_rawdata(buf);
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ft5x0x_rawdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return -EPERM;
}


/* touch_type: 1-capacitive screen, 2-reisitive screen */
static ssize_t touchscreen_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0))
		ret = touchscreen_ops[0]->touch_type;
	else if (TOUCH_IN_ACTIVE(1))
		ret = touchscreen_ops[1]->touch_type;
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t touchscreen_active_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int ret1 = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (touchscreen_ops[0] && touchscreen_ops[0]->active)
		ret = touchscreen_ops[0]->active();

	if (touchscreen_ops[1] && touchscreen_ops[1]->active)
		ret1 = touchscreen_ops[1]->active();
	mutex_unlock(&touchscreen_mutex);

	TSFS_DBG("touch 1 return %d, touch 2 return %d", ret, ret1);
	return snprintf(buf, PAGE_SIZE, "%d,%d\n", ret, ret1);
}

static ssize_t touchscreen_firmware_update_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->firmware_need_update)
			ret = touchscreen_ops[0]->firmware_need_update();
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->firmware_need_update)
			ret = touchscreen_ops[1]->firmware_need_update();
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t touchscreen_firmware_update_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	if (strncmp(buf, "update", count-1)) {
		TSFS_ERR("string is %s, count=%zd not update!", buf, count);
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->firmware_need_update &&
				touchscreen_ops[0]->firmware_need_update() &&
				touchscreen_ops[0]->firmware_do_update) {
			ret = touchscreen_ops[0]->firmware_do_update();
		}
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->firmware_need_update &&
				touchscreen_ops[1]->firmware_need_update() &&
				touchscreen_ops[1]->firmware_do_update) {
			ret = touchscreen_ops[1]->firmware_do_update();
		}
	}
	mutex_unlock(&touchscreen_mutex);
	return count;
}

static ssize_t touchscreen_calibrate_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->need_calibrate)
			ret = touchscreen_ops[0]->need_calibrate();
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->need_calibrate)
			ret = touchscreen_ops[1]->need_calibrate();
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t touchscreen_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	if (strncmp(buf, "calibrate", count-1)) {
		TSFS_ERR("string is %s, not calibrate!", buf);
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->calibrate)
			ret = touchscreen_ops[0]->calibrate();
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->calibrate)
			ret = touchscreen_ops[1]->calibrate();
	}
	mutex_unlock(&touchscreen_mutex);
	return count;
}

static ssize_t touchscreen_firmware_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char version[32] = {0};

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->get_firmware_version)
			touchscreen_ops[0]->get_firmware_version(version);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->get_firmware_version)
			touchscreen_ops[1]->get_firmware_version(version);
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%s\n", version);
}

static ssize_t touchscreen_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	if (strncmp(buf, "reset", count-1)) {
		TSFS_ERR("string is %s, not reset!", buf);
		return -EINVAL;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->reset_touchscreen)
			ret = touchscreen_ops[0]->reset_touchscreen();
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->reset_touchscreen)
			ret = touchscreen_ops[1]->reset_touchscreen();
	}
	mutex_unlock(&touchscreen_mutex);

	return count;
}

static ssize_t touchscreen_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->get_mode)
			ret = touchscreen_ops[0]->get_mode();
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->get_mode)
			ret = touchscreen_ops[1]->get_mode();
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t touchscreen_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int mode = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	if (strncmp(buf, "normal", count-1) == 0)
		mode = MODE_NORMAL;
	else if (strncmp(buf, "handwrite", count-1) == 0)
		mode = MODE_HANDWRITE;
	else if (strncmp(buf, "glove", count - 1) == 0)
		mode = MODE_GLOVE;
	else if (strncmp(buf, "glove_force", count - 1) == 0)
		mode = MODE_GLOVE_FORCE;
	else if (strncmp(buf, "normal_window", count - 1) == 0)
		mode = MODE_NORMAL_WINDOW;
	else if (strncmp(buf, "glove_window", count - 1) == 0)
		mode = MODE_GLOVE_WINDOW;
	else {
		TSFS_ERR("Doesn't support %s mode!", buf);
		return -EINVAL;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->set_mode)
			ret = touchscreen_ops[0]->set_mode(mode);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->set_mode)
			ret = touchscreen_ops[1]->set_mode(mode);
	}
	mutex_unlock(&touchscreen_mutex);

	return count;
}

static ssize_t touchscreen_orientation_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->get_orientation)
			ret = touchscreen_ops[0]->get_orientation();
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->get_orientation)
			ret = touchscreen_ops[1]->get_orientation();
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t touchscreen_orientation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = 0;
	int oreitation = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	if (strncmp(buf, "oreitation", count-2)) {
		TSFS_ERR("string is %s, not oreitation", buf);
		return -EINVAL;
	}

	oreitation = buf[count-2]-'0';
	TSFS_DBG("oreitation=%d", oreitation);
	if (oreitation < 0 || oreitation > 3) {
		TSFS_ERR("oreitation[%d] is invalid", oreitation);
		return -EINVAL;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->set_orientation)
			ret = touchscreen_ops[0]->set_orientation(oreitation);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->set_orientation)
			ret = touchscreen_ops[1]->set_orientation(oreitation);
	}
	mutex_unlock(&touchscreen_mutex);

	return count;
}

static ssize_t touchscreen_regs_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->read_regs)
			ret = touchscreen_ops[0]->read_regs(buf);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->read_regs)
			ret = touchscreen_ops[1]->read_regs(buf);
	}
	mutex_unlock(&touchscreen_mutex);

	return ret;
}

static ssize_t touchscreen_regs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->write_regs)
			ret = touchscreen_ops[0]->write_regs(buf);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->write_regs)
			ret = touchscreen_ops[1]->write_regs(buf);
	}
	mutex_unlock(&touchscreen_mutex);
	return count;
}

static ssize_t touchscreen_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int on = 0;

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	if (strncmp(buf, "on", count-1) == 0)
		on = 1;

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->debug)
			ret = touchscreen_ops[0]->debug(on);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->debug)
			ret = touchscreen_ops[1]->debug(on);
	}
	mutex_unlock(&touchscreen_mutex);
	return count;
}


static ssize_t touchscreen_vendor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char vendor[64] = {0};

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->get_vendor)
			touchscreen_ops[0]->get_vendor(vendor);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->get_vendor)
			touchscreen_ops[1]->get_vendor(vendor);
	}
	mutex_unlock(&touchscreen_mutex);

	return snprintf(buf, PAGE_SIZE, "%s\n", vendor);
}
static ssize_t  touchscreen_gesture_wakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char gesture[64] = {0};

	if (buf == NULL) {
		TSFS_ERR("buf is NULL!\n");
		return -ENOMEM;
	}
	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->get_wakeup_gesture)
			touchscreen_ops[0]->get_wakeup_gesture(gesture);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->get_wakeup_gesture)
			touchscreen_ops[1]->get_wakeup_gesture(gesture);
	}
	mutex_unlock(&touchscreen_mutex);
	return snprintf(buf, PAGE_SIZE, "%s\n", gesture);
}
static ssize_t  touchscreen_gesture_ctrl_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;

	if (buf == NULL) {
		TSFS_ERR("buf is NULL!\n");
		return -ENOMEM;
	}
	mutex_lock(&touchscreen_mutex);
	if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->gesture_ctrl)
			ret = touchscreen_ops[0]->gesture_ctrl(buf);
	} else if (TOUCH_IN_ACTIVE(1)) {
		if (touchscreen_ops[1]->gesture_ctrl)
			ret = touchscreen_ops[1]->gesture_ctrl(buf);
	}
	mutex_unlock(&touchscreen_mutex);
	return count;
}
static DEVICE_ATTR(type, 0444, touchscreen_type_show, NULL);
static DEVICE_ATTR(active, 0444, touchscreen_active_show, NULL);
static DEVICE_ATTR(firmware_update, 0664,
	touchscreen_firmware_update_show, touchscreen_firmware_update_store);
static DEVICE_ATTR(calibrate, 0664,
	touchscreen_calibrate_show, touchscreen_calibrate_store);
static DEVICE_ATTR(firmware_version, 0444,
	touchscreen_firmware_version_show, NULL);
static DEVICE_ATTR(reset, 0224, NULL, touchscreen_reset_store);
static DEVICE_ATTR(mode, 0664, touchscreen_mode_show, touchscreen_mode_store);
static DEVICE_ATTR(oreitation, 0664,
	touchscreen_orientation_show, touchscreen_orientation_store);
static DEVICE_ATTR(regs, 0664, touchscreen_regs_show, touchscreen_regs_store);
static DEVICE_ATTR(debug, 0224, NULL, touchscreen_debug_store);
static DEVICE_ATTR(get_rawdata, 0664,
	ft5x0x_rawdata_show, ft5x0x_rawdata_store);
static DEVICE_ATTR(vendor, 0444, touchscreen_vendor_show, NULL);
static DEVICE_ATTR(gesture_wakeup, 0444,
	touchscreen_gesture_wakeup_show, NULL);
static DEVICE_ATTR(gesture_ctrl, 0220, NULL, touchscreen_gesture_ctrl_store);

static ssize_t touchscreen_ftsscaptest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char result[64] = {0};

	if (!buf) {
		TSFS_ERR("buf is NULL!");
		return -ENOMEM;
	}

        mutex_lock(&touchscreen_mutex);
        if (TOUCH_IN_ACTIVE(0)) {
		if (touchscreen_ops[0]->debug)
			touchscreen_ops[0]->ftsscaptest(result);
        } else if (TOUCH_IN_ACTIVE(1)) {
                if (touchscreen_ops[1]->debug)
                        touchscreen_ops[1]->ftsscaptest(result);
        }
        mutex_unlock(&touchscreen_mutex);
	return snprintf(buf, PAGE_SIZE, "%s\n", result);
}
static DEVICE_ATTR(ftsscaptest, 0664, touchscreen_ftsscaptest_show, NULL);

static const struct attribute *touchscreen_attrs[] = {
	&dev_attr_type.attr,
	&dev_attr_active.attr,
	&dev_attr_firmware_update.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_reset.attr,
	&dev_attr_mode.attr,
	&dev_attr_oreitation.attr,
	&dev_attr_regs.attr,
	&dev_attr_debug.attr,
	&dev_attr_get_rawdata.attr,
	&dev_attr_vendor.attr,
	&dev_attr_gesture_wakeup.attr,
	&dev_attr_gesture_ctrl.attr,
	&dev_attr_ftsscaptest.attr,
	NULL,
};

static const struct attribute_group touchscreen_attr_group = {
	.attrs = (struct attribute **) touchscreen_attrs,
};

static ssize_t export_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t len)
{
	return 1;
}

static ssize_t unexport_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t len)
{
	return 1;
}

static struct class_attribute uart_class_attrs[] = {
	__ATTR(export, 0200, NULL, export_store),
	__ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

static struct class touchscreen_class = {
	.name =		"touchscreen",
	.owner =	THIS_MODULE,
	.class_attrs =	uart_class_attrs,
};

static struct device *touchscreen_dev;
struct device *touchscreen_get_dev(void)
{
	return touchscreen_dev;
}
EXPORT_SYMBOL(touchscreen_get_dev);

static int touchscreen_export(void)
{
	int status = 0;
	struct device *dev = NULL;

	dev = device_create(&touchscreen_class,
		NULL, MKDEV(0, 0), NULL, "touchscreen_dev");
	if (dev) {
		status = sysfs_create_group(&dev->kobj,
				&touchscreen_attr_group);
		touchscreen_dev = dev;
	} else {
		pr_err("%s: touchscreen_dev device_create fail\n", __func__);
		status = -ENODEV;
	}

	return status;
}

static int __init touchscreen_sysfs_init(void)
{
	int status = 0;

	touchscreen_ops[0] = NULL;
	touchscreen_ops[1] = NULL;
	status = class_register(&touchscreen_class);
	if (status < 0) {
		pr_err("%s: touchscreen_class class_register fail\n", __func__);
		return status;
	}
	status = touchscreen_export();

	return status;
}

arch_initcall(touchscreen_sysfs_init);

