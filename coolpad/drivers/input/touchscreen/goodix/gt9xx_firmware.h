// make sense only when GTP_HEADER_FW_UPDATE & GTP_AUTO_UPDATE are enabled
// define your own firmware array here
const unsigned char gtp_default_FW[] = {
#if defined(CONFIG_BOARD_CP5560S)
	#include "cp5560s_GT968.fw"
#elif defined(CONFIG_BOARD_CP3600L)
	#include "cp3600l_GT913.fw"
#elif defined(CONFIG_BOARD_CP8716)
	#include "cp8716.fw"
#else
	#include "cp7576u_GT913.fw"
#endif
};
