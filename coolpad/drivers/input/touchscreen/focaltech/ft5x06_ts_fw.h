#ifndef __LINUX_FT5X0X_TS_FW_H__
#define __LINUX_FT5X0X_TS_FW_H__

/*芯片型号，在移植时根据具体IC的型号来修改,只能定义其中一款*/
//#define IC_FT5X06             /*x=2,3,4*/
//#define IC_FT5X16             /*ft5506 ft5606*/
#define IC_FT5X06       4	//add by anxufeng
#define DEVICE_IC_TYPE      IC_FT5X06
//#define IC_FT5X16             /*5x16*/
//#define IC_FT6208

/*芯片型号，在移植时根据具体IC的型号来修改*/
#define FOCALTECH_SOC  "ft5406"	/*chipset version */

/*COB/COF 开关，如果使用COB 方案，为1，否则为0 */
#define FOCALTECH_COB	0 // fix by sdy for cp3602u proj

//X,Y轴最大值
#define TP_MAX_Y	900	//maleijie 854
#define TP_MAX_X	480

struct touch_panel_info {

	unsigned char tp_id;	/*touch panel factory id */

	unsigned char *tp_name;	/*touch panel factory name */

	unsigned char *firmware;	/*firmware for this factory's touch panel */

	unsigned int firmware_size;

};

/****************down模式，升级bootloader用*************/
static unsigned char CTPM_MAIN_FW_OFILM[] = {
};

static unsigned char CTPM_MAIN_FW_EACH[] = {
};

//static unsigned char CTPM_MAIN_FW_JUNDA[]=
//{
//   #include "coolpad_5892_0x85_v26_20131212_all.i"
//};

/*模组固件，将项目使用的模组固件定义在这里，并将模组ID、模组厂名修改在下表中*/
//static unsigned char CTPM_FW_LC[]=
//{
    //  #include "coolpad_9070_0x87_V11_20121223_app.i"
//};

//static unsigned char CTPM_FW_LB[]=
//{
    //   #include "coolpad_9070_0x55_V12_20130106_app.i"
//};

static unsigned char CTPM_FW_OFILM[] = {
#include "Coolpad_3602U_Ofilm.h"
};

static unsigned char CTPM_FW_EACH[] = {
#include "Coolpad_3602U_Eachopto.h"
};


struct touch_panel_info yl_fw[] = {
	{0x51, "Ofilm", CTPM_FW_OFILM, sizeof(CTPM_FW_OFILM)}
	,
	{0x80, "Each", CTPM_FW_EACH, sizeof(CTPM_FW_EACH)}
	,
//  {0x85, "JunDa", CTPM_FW_JUNDA, sizeof(CTPM_FW_JUNDA)},
	// {0x85, "JunDa", CTPM_FW_JUNDA, sizeof(CTPM_FW_JUNDA)},
//  {0xa6, "boyi", CTPM_FW_BOYI, sizeof(CTPM_FW_BOYI)},
};

struct touch_panel_info yl_main_fw[] = {
	{0x51, "ofilm", CTPM_MAIN_FW_OFILM, sizeof(CTPM_MAIN_FW_OFILM)}
	,
	{0x80, "each", CTPM_MAIN_FW_EACH, sizeof(CTPM_MAIN_FW_EACH)}
	,
	//   {0x85,"junda",CTPM_MAIN_FW_JUNDA,sizeof(CTPM_MAIN_FW_JUNDA)},
};

#if FOCALTECH_COB
struct pin_to_id {

	unsigned char pin;	/*gpio state */

	unsigned char id;	/*touch panel factory id */

};

/*COB方案，ID 识别的GPIO 状态对应到具体的模组ID */
static const struct pin_to_id PIN_ID[] = {
	{0x00, 0x51},
	{0x01, 0x80},
	{0x02, 0x85},
	{0x03, 0x85},
};

#endif

/*以下部分不要修改*/
/* this blow don't modify except yl_fw */
const int fw_array_size = ARRAY_SIZE(yl_fw);

struct touch_panel_info *tp_info = yl_fw;

unsigned char *current_main_fw_to_tp = NULL;	//add for download mode
unsigned int current_main_fw_size = 0;	//add for download mode

unsigned char *current_fw_to_tp = NULL;

unsigned int current_fw_size = 0;

unsigned char *current_tp_name = NULL;

unsigned char current_tp_id;

unsigned int current_tp_version;

bool current_tp_supported = false;

#endif
