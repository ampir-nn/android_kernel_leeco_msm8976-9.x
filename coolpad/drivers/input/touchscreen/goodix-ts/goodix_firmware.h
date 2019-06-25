// make sense only when GTP_HEADER_FW_UPDATE & GTP_AUTO_UPDATE are enabled
// define your own firmware array here
#ifndef LINUX_GT9XX_FIRMWARE_H
#define LINUX_GT9XX_FIRMWARE_H

#if defined(CONFIG_BOARD_CPK1_NW_I00)
const unsigned char gtp_default_FW[] = {
	#include "GT915_1091"
};
#elif defined(CONFIG_BOARD_CPK1_NW_I00)
const unsigned char gtp_default_FW[] = {
	#include "GT9157_1070"
};
#elif defined(CONFIG_BOARD_CP8675_I02)
const unsigned char gtp_default_FW[] = {
#ifdef CONFIG_COVER_WINDOW_CFG
#include "GT970_1039_DACA"
#else
#include "GT970_1030_5A1F"
#endif
};
#elif defined(CONFIG_BOARD_CPC1) || defined(CONFIG_BOARD_C1)\
	|| defined(CONFIG_BOARD_C2) || defined(CONFIG_BOARD_C107)
const unsigned char gtp_default_FW[] = {
	#include "GT915L_3003_9067"
};
#else
const unsigned char gtp_default_FW[] = {
	#include "GT9157_1070"
};
#endif
#endif
