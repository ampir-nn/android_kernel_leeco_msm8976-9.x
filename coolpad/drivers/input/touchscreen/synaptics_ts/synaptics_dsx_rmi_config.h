#ifndef _SYNA_CONFIG_H
#define _SYNA_CONFIG_H

/*auto update firmware in bootloader,close it when release*/
#define POWER_ON_REPAIR 0

struct touch_panel_info
{
    unsigned char  tp_id;       /*touch panel factory id*/
    unsigned char *tp_name;    	/*touch panel factory name */
    unsigned char *firmware;    /*firmware for this factory's touch panel*/
    unsigned int   firmware_size;
};

#ifdef CONFIG_BOARD_CP8875U
#include "Coolpad_8875U_Ofilm.h"
#include "Coolpad_8875U_Eachopto.h"
struct touch_panel_info syna_tw_fw_8875u[] = {
	{0xa0, "ofilm", SYNA_FW_OFILM_8875u, sizeof(SYNA_FW_OFILM_8875u)},
	{0xa1, "eachopto", SYNA_FW_Eachopto_8875u, sizeof(SYNA_FW_Eachopto_8875u)},
};
#endif

#if defined(CONFIG_BOARD_CP3601U) || defined(CONFIG_BOARD_CP3605U) \
|| defined(CONFIG_BOARD_CP3320A) || defined(CONFIG_BOARD_YY8909)
#include "Coolpad_8865U_Eachopto.h"
#include "Coolpad_8865U_Toptouch.h"
struct touch_panel_info syna_tw_fw_8865u[] = {
	{0xa1, "eachopto", SYNA_FW_Eachopto_8865u, sizeof(SYNA_FW_Eachopto_8865u)},
	{0xaf, "Toptouch", SYNA_FW_Toptouch_8865u, sizeof(SYNA_FW_Toptouch_8865u)},
};
#endif

#ifdef CONFIG_BOARD_CP3600L
#include "Coolpad_3600L_Eachopto.h"
#include "Coolpad_3600L_Toptouch.h"
struct touch_panel_info syna_tw_fw_3600l[] = {
	{0xa1, "eachopto", SYNA_FW_Eachopto_3600l, sizeof(SYNA_FW_Eachopto_3600l)},
	{0xaf, "Toptouch", SYNA_FW_Toptouch_3600l, sizeof(SYNA_FW_Toptouch_3600l)},
};
#endif


#ifdef CONFIG_BOARD_CP8865U
#include "Coolpad_8865U_Eachopto.h"
#include "Coolpad_8865U_Toptouch.h"
struct touch_panel_info syna_tw_fw_8865u[] = {
	{0xa1, "eachopto", SYNA_FW_Eachopto_8865u, sizeof(SYNA_FW_Eachopto_8865u)},
	{0xaf, "Toptouch", SYNA_FW_Toptouch_8865u, sizeof(SYNA_FW_Toptouch_8865u)},
};
#endif

#ifdef CONFIG_BOARD_CP8875S
#include "Coolpad_8875S_Ofilm.h"
#include "Coolpad_8875S_Eachopto.h"
struct touch_panel_info syna_tw_fw_8875s[] = {
	{0xa0, "ofilm", SYNA_FW_OFILM_8875s, sizeof(SYNA_FW_OFILM_8875s)},
	{0xa1, "eachopto", SYNA_FW_Eachopto_8875s, sizeof(SYNA_FW_Eachopto_8875s)},
};
#endif


#if defined(CONFIG_BOARD_CP8890U) || defined(CONFIG_BOARD_CP8890E)
#include "Coolpad_8890U_Ofilm.h"
#include "Coolpad_8890U_Junda.h"
struct touch_panel_info syna_tw_fw_8890u[] = {
	{0xa0, "ofilm", SYNA_FW_OFILM_8890u, sizeof(SYNA_FW_OFILM_8890u)},
	{0xab, "junda", SYNA_FW_JUNDA_8890u, sizeof(SYNA_FW_JUNDA_8890u)},
};
#endif

struct pin_to_id
{
    unsigned char  pin;		/*gpio state*/
    unsigned char  id;		/*touch panel factory id*/
};

static struct pin_to_id PIN_ID[] = {
	{0x00, 0xaf},/*toptouch	S2202*/
	{0x05, 0xa0},/*ofilm	s3202*/
	{0x07, 0xa1},/*eachopto	S3202*/
	{0x0c, 0xa1},/*eachopto	S2202*/
};

#endif

