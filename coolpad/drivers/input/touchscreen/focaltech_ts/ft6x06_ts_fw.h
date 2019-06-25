#ifndef __LINUX_FT6X06_TS_FW_H__
#define __LINUX_FT6X06_TS_FW_H__

//#define IC_FT5X06             /*x=2,3,4*/
//#define IC_FT5X16             /*ft5506 ft5606*/
#define IC_FT6X36       4	//add by anxufeng
#define DEVICE_IC_TYPE      IC_FT6X36
//#define IC_FT5X16             /*5x16*/
//#define IC_FT6208
#define FOCALTECH_SOC  "ft6336"	/*chipset version */
#define TP_MAX_Y	900	//maleijie 854
#define TP_MAX_X	480

#define CONFIG_FIRMWARE_UPDATE 1  /*just for debug,open it for MP*/

struct touch_panel_info {
	unsigned char tp_id;	/*touch panel factory id */
	unsigned char *tp_name;	/*touch panel factory name */
	unsigned char *firmware;	/*firmware for this factory's touch panel */
	unsigned int firmware_size;
};


static unsigned char CTPM_FW_OFILM[] = {
#if defined(CONFIG_BOARD_CP3605U)
#include "Coolpad_3605U_Ofilm.h"
#elif defined(CONFIG_BOARD_CP3320A)
#include "Coolpad_3320A_Ofilm.h"
#endif
};

static unsigned char CTPM_FW_SHENYUE[] = {
#if defined(CONFIG_BOARD_CP3605U)
#include "Coolpad_3605U_Shenyue.h"
#elif defined(CONFIG_BOARD_CP3320A)
#include "Coolpad_3320A_Shenyue.h"
#elif defined(CONFIG_BOARD_CP3622A)
#include "Coolpad_3622A_Shenyue_0xA0.h"
#endif
};

static unsigned char CTPM_FW_Yeji[] = {
#if defined(CONFIG_BOARD_CP3622A)
#include "Coolpad_3622A_Yeji_0x80.h"
#endif
};

struct touch_panel_info yl_fw[] = {
	{0xA0, "Shenyue", CTPM_FW_SHENYUE, sizeof(CTPM_FW_SHENYUE)},
	{0x51, "Ofilm", CTPM_FW_OFILM, sizeof(CTPM_FW_OFILM)},
	{0x80, "Yeji", CTPM_FW_Yeji, sizeof(CTPM_FW_Yeji)},
};

/* this blow don't modify except yl_fw */
const int fw_array_size = ARRAY_SIZE(yl_fw);
struct touch_panel_info *tp_info = yl_fw;
unsigned char *current_fw_to_tp = NULL;
unsigned int current_fw_size = 0;
unsigned char *current_tp_name = NULL;
unsigned char current_tp_id;
unsigned int current_tp_version;
bool current_tp_supported = false;
#endif
