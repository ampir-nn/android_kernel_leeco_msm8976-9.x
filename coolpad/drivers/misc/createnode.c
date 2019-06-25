/* 宇龙计算机通信科技（深圳）有限公司  版权所有 2000-2011 */
/**/
/* PROPRIETARY RIGHTS of YULONG Company are involved in the*/
/* subject matter of this material.  All manufacturing, reproduction, use,  */
/* and sales rights pertaining to this subject matter are governed by the   */
/* license agreement.  The recipient of this software implicitly accepts    */
/* the terms of the license.                                                */
/* 本软件文档资料是宇龙公司的资产,任何人士阅读和使用本资料必须获得     */
/* 相应的书面授权,承担保密责任和接受相应的法律约束.                         */
/**/
/****************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2012, Yulong Tech. Co., Ltd.
**  FileName:          createnode.c
**  Author:            zhuhui
**  Version :          1.00
**  Date:              2014-3-1

**  History:
**  <author>      <time>      <version >      <desc>
**   zhuhui         2014-3-1     1.00         create
**
**************************************************************************/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/createnode.h>

int smartphone_calling_enable;

static struct class_attribute volumenode_class_attrs[];

int get_smartphone_calling_enable(void)
{
	return smartphone_calling_enable;
}

static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

static ssize_t phone_call_enable_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t len)
{
	if (attr == volumenode_class_attrs) {
		smartphone_calling_enable = atoi(buf);
		printk(KERN_ERR "%s,%d\n", __func__, smartphone_calling_enable);
	}

	return 0;
}

static struct class_attribute volumenode_class_attrs[] = {
	__ATTR(phone_call_enable, 0200, NULL, phone_call_enable_store),
	__ATTR_NULL,
};

static struct class volumenode_class = {
	.name =        "volumenode",
	.owner =       THIS_MODULE,
	.class_attrs = volumenode_class_attrs,
};

static int __init volumenode_sysfs_init(void)
{
	int status = 0;

	status = class_register(&volumenode_class);
	if (status < 0) {
		printk(KERN_ERR "%s: volumenode_class class register fail\n",
			__func__);
		return status;
	}

	smartphone_calling_enable = 0;

	return status;
}

arch_initcall(volumenode_sysfs_init);

MODULE_AUTHOR("zhuhui <zhuhui@yulong.com>");
MODULE_DESCRIPTION("Create device node to user space");
MODULE_LICENSE("GPL");
