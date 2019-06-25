/*
 * Copyright (c) 2012-2017  YULONG Company
 *
 * PROPRIETARY RIGHTS of YULONG Company are involved in the
 * subject matter of this material.  All manufacturing, reproduction, use,
 * and sales rights pertaining to this subject matter are governed by the
 * license agreement.  The recipient of this software implicitly accepts
 * the terms of the license.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>

static int ftm;
static char hw_version_str[3];
static int hw_version;
static int boot_board = 0;

static int __init ftm_setup(char *line)
{
	ftm = 1;
	return 1;
}
__setup("androidboot.mode=ffbm-02", ftm_setup);

int yl_get_ftm(void)
{
	return ftm;
}
EXPORT_SYMBOL(yl_get_ftm);

static int __init hw_version_setup(char *line)
{
	char *str = hw_version_str;

	strlcpy(hw_version_str, line, sizeof(hw_version_str));
	if (!strncmp(str, "P0", 2))
		hw_version = 0x00;
	else if (!strncmp(str, "P1", 2))
		hw_version = 0x01;
	else if (!strncmp(str, "P2", 2))
		hw_version = 0x02;
	else if (!strncmp(str, "P3", 2))
		hw_version = 0x03;
	else if (!strncmp(str, "P4", 2))
		hw_version = 0x04;
	else if (!strncmp(str, "P5", 2))
		hw_version = 0x05;
	else if (!strncmp(str, "P6", 2))
		hw_version = 0x06;
	else if (!strncmp(str, "T0", 2))
		hw_version = 0x10;
	else if (!strncmp(str, "T1", 2))
		hw_version = 0x11;
	else if (!strncmp(str, "T2", 2))
		hw_version = 0x12;
	else
		hw_version = -1;
	return 1;
}
__setup("hardware_version=", hw_version_setup);

int yl_get_hardware_version(void)
{
	return hw_version;
}
EXPORT_SYMBOL(yl_get_hardware_version);

static int __init boot_boardhwid_setup(char *line)
{
        if(NULL == line) {
                pr_err("YULONG:jk:%s boardhwid is null", __func__);
                return 1;
        }
        pr_err("YULONG:jk:%s boot board is %s\n", __func__, line);
        if(0 == strcmp(line, "0x09")) {
                boot_board = 0x09;
        } else if(0 == strcmp(line, "0x0b")) {
                boot_board = 0x0b;
        } else if(0 == strcmp(line, "0x06")) {
                boot_board = 0x06;
        } else if(0 == strcmp(line, "0x04")) {
                boot_board = 0x04;
        } else if(0 == strcmp(line, "0x02")) {
                boot_board = 0x02;
        } else {
                boot_board =0xfe;;
        }
        return 1;
}

__setup("androidboot.boardhwid=", boot_boardhwid_setup);

int yl_get_boardhwid(void)
{
        return boot_board;
}

EXPORT_SYMBOL(yl_get_boardhwid);
