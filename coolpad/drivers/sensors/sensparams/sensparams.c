/*
 *  sensparams.c - definitions for storage all sensors parameters
 *
 *  Copyright (C) 2010 Yulong Tech. Co., Ltd.
 *  Jay.HF <huangfujie@yulong.com>
 *  Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sensors/sensparams.h>
#include <linux/yl_params.h>

ssize_t sensparams_write_to_flash(int type, unsigned char *in, int len)
{
	ssize_t ret = 0;
	struct ProductlineInfo *ptr = NULL;
	unsigned char param[512] = "PRODUCTLINE";
	int size = sizeof(param);

	if (in == NULL || len <= 0 || len > sizeof(struct SensorCalInfo)) {
		pr_err("%s: error, in = %zd, len = %d\n", __func__,
			(ssize_t)in, len);
		return -ENOMEM;
	}

	ret = yl_params_kernel_read(param, size);
	if (ret < 0) {
		pr_err("%s: read %d type sensor param failed\n",
			__func__, type);
		return -EIO;
	}

	ptr = (struct ProductlineInfo *)param;
	switch (type) {
	case SENSPARAMS_TYPE_ACC:
		memcpy(&ptr->AccInfo, in, len);
		break;

	case SENSPARAMS_TYPE_PROX:
		memcpy(&ptr->LightProxInfo, in, len);
		break;

	case SENSPARAMS_TYPE_PRESS:
		memcpy(&ptr->PressInfo, in, len);
		break;

	case SENSPARAMS_TYPE_RESV1:
		memcpy(&ptr->SensorReserved1, in, len);
		break;

	case SENSPARAMS_TYPE_RESV2:
		memcpy(&ptr->SensorReserved2, in, len);
		break;

	case SENSPARAMS_TYPE_RESV3:
		memcpy(&ptr->SensorReserved3, in, len);
		break;

	default:
		pr_err("%s: %d type sensor is not supported\n",
			__func__, type);
		return -EINVAL;
	}

	return yl_params_kernel_write(param, size);
}
EXPORT_SYMBOL_GPL(sensparams_write_to_flash);

int sensparams_read_from_flash(int type, unsigned char *out, int len)
{
	ssize_t ret = 0;
	struct ProductlineInfo *ptr = NULL;
	unsigned char param[512] = "PRODUCTLINE";
	int size = sizeof(param);

	if (out == NULL || len <= 0 || len > sizeof(struct SensorCalInfo)) {
		pr_err("%s: error, out = %zd, len = %d\n",
			__func__, (ssize_t)out, len);
		return -ENOMEM;
	}

	ret = yl_params_kernel_read(param, size);
	if (ret != size) {
		pr_err("%s: read %d type sensor param failed\n",
			__func__, type);
		return -EIO;
	}

	ptr = (struct ProductlineInfo *)param;
	switch (type) {
	case SENSPARAMS_TYPE_ACC:
		memcpy(out, &ptr->AccInfo, len);
		break;

	case SENSPARAMS_TYPE_PROX:
		memcpy(out, &ptr->LightProxInfo, len);
		break;

	case SENSPARAMS_TYPE_PRESS:
		memcpy(out, &ptr->PressInfo, len);
		break;

	case SENSPARAMS_TYPE_RESV1:
		memcpy(out, &ptr->SensorReserved1, len);
		break;

	case SENSPARAMS_TYPE_RESV2:
		memcpy(out, &ptr->SensorReserved2, len);
		break;

	case SENSPARAMS_TYPE_RESV3:
		memcpy(out, &ptr->SensorReserved3, len);
		break;

	default:
		pr_err("%s: %d type sensor is not supported\n",
			__func__, type);
		return -EINVAL;
	}
	return len;
}
EXPORT_SYMBOL_GPL(sensparams_read_from_flash);
