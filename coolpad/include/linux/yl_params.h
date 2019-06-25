/*
 * YULONG PARAMETER PARTITION
 * VERSION 1.11
 */

#ifndef _YL_PARAMS_H_
#define _YL_PARAMS_H_

enum yl_params_index {
	YL_DEVICE = 0,
	YL_CONFIGURATION,
	YL_PRODUCTLINE,
	YL_DYNAMIC,
	YL_GUARD,
	YL_CMDLINE,
	YL_TOUCHSCREEN0,
	YL_TOUCHSCREEN1,
	YL_TOUCHSCREEN2,
	YL_TOUCHSCREEN3,
	YL_RESERVE0,
	YL_RESERVE1,
	YL_PROJECT0,
	YL_PROJECT1,
	YL_PROJECT2,
	YL_PROJECT3,
	YL_FETCH_PASSWD,
	YL_FCT_DIAG,
	YL_RCP,
	YL_RETURNZERO,
	YL_VIRTOS_PASSWD,
	YL_VIRTOS_MISC,
	YL_PARAMS_COUNT,
};

enum yl_params_index_v0 {
	YL_OBSOLETE_DEVICE = 0,
	YL_OBSOLETE_WLAN,
	YL_OBSOLETE_ACCTW_CAL,
	YL_OBSOLETE_DOWNFLAG,
	YL_OBSOLETE_GUARD,
	YL_OBSOLETE_odm_lckc,
	YL_OBSOLETE_odm_lckl,
	YL_OBSOLETE_ALM_TIME,
	YL_OBSOLETE_mms_imsi,
	YL_OBSOLETE_CAM_CAL,
	YL_OBSOLETE_AUTO_REG,
	YL_OBSOLETE_VOL_TAB,
	YL_OBSOLETE_PASSWRD,
	YL_OBSOLETE_MMS_VER,
	YL_OBSOLETE_ALM_POFF,
	YL_OBSOLETE_RESERVED,
};

struct DeviceInfo_v0 {
	char	SyncByte[10];
	char	DeviceName[32];
	char	BSP[32];
	char	ESN[32];
	char	SN[32];
	char	CommunicationModel1[32];
	char	CommunicationModel2[32];
	char	BlueModel[32];
	char	ImageSensorModel[32];
	char	WiFi[32];
	char	HardwareVersionMajor;
	char	HardwareVersionAux;
	char	Year;
	char	Month;
	char	Day;
	char	GPS[32];
	char	IMEI1[32];
	char	IMEI2[32];
	char	Module1CalStatus;
	char	Module2CalStatus;
	char	Module1RFTestStatus;
	char	Module2RFTestStatus;
	char	PCBAInformation[32];
	char	AccInfo[12];
	char	DMtag;
	char	sensor_cal_flag;
	char	RPtag;
	char	SupperPassword[8];
	char	pad[54];
};

#define TAG_LENGTH	16
#define ONE_BLOCK_SIZE	512

struct DeviceInfo {
	char	SyncByte[16];
	char	ParamVer[2];
	char	Date[6];
	char	CommunicationModel1[16];
	char	CommunicationModel2[16];
	char	ImageSensorModel[16];
	char	SafeboxKey[128];
	char	pad1[128];
	char	Sim1Capacity[2];
	char	Sim2Capacity[2];
	char	Sim3Capacity[2];
	char	NET_CARRIER;
	char	SimSlots;
	char 	SalesChannel;
	char 	AllmodeOperator;
	char	SafeboxPassword[16];
	char	pad[158];
};

struct MainDevInfo {
	char	name[8];
	char	vendor[16];
	char	model[16];
};

struct ConfigurationInfo {
	char	SyncByte[16];
	char	ProductName[16];
	char	u8HardwareVersionMajor[6];
	char	u8HardwareVersionAux[6];
	char	u8HardwareRF_NV[6];
	struct	MainDevInfo DevInfo[11];
	char	pad[22];
};

struct SensorCalInfo {
	char SensorCalStatus;
	char value[7];
} __attribute__ ((packed));

struct ProductlineInfo {
	char	SyncByte[16];
	char	SN[16];
	char	IMEI1[32];
	char	IMEI2[32];
	char	ModuleCalStatus1;
	char	ModuleCalStatus2;
	char	ModuleRFTestStatus1;
	char	ModuleRFTestStatus2;
	char	ModuleCouplingTestStatus1;
	char	ModuleCouplingTestStatus2;
	char	DMtag;
	char	CameraCal;
	char	RPtag;
	char	BatteryTest;
	char	ModuleSoftVersion1[48];
	char	ModuleSoftVersion2[48];
	char	ModuleAudioVersion1[48];
	char	ModuleAudioVersion2[48];
	char	FuseBurnStatus;
	char	MiscStatus[5];
	struct	SensorCalInfo	LightProxInfo;
	struct	SensorCalInfo	AccInfo;
	struct	SensorCalInfo	PressInfo;
	struct	SensorCalInfo	SensorReserved1;
	struct	SensorCalInfo	SensorReserved2;
	struct	SensorCalInfo	SensorReserved3;
	char	DSDS_IMEI[32];
	char	WIFI_MAC[6];
	char	BT_MAC[6];
	char	AF_Code[12];
	char	Optical_axis[12];
	char    SN2[24];
	char	KEY[32];
	char	pad[36];
} __attribute__ ((packed));

struct DynamicInfo {
	char	SyncByte[16];
	char	BSP[32];
	char	Password[16];
	char	SuperPassword[16];
	char	DownloadFlag[16];
	char	DownloadTool[32];
	char	SoftwareVersion[48];
	char	DIYimageFlag[16];
	char	SecurityFlag;
	char	CTSFlag;
	char	DRMFlag;
	char	USBChargeFlag;
	char	LTEState;
	char	MultiBootloader;
	char	GMSDownload;
	char	CPBDownload;
	char	TouchIC[1];
	char	ActivedBySIM[1];
	char	MiscFlags[55];
	char	Virgin[16];
	char	NfcUnlockKey[32];
	char	VirtOsPassword[16];
	char	Flag;
	char	IsReportedFlag;
	char	pad[189];
};

struct GuardInfo {
	char	SyncByte[16];
};

struct CommandlineInfo {
	char	SyncByte[16];
	unsigned short crc;
	unsigned short len;
	char	data[492];
};

struct Touch0Info {
	char	SyncByte[16];
};

struct Touch1Info {
	char	SyncByte[16];
};

struct Touch2Info {
	char	SyncByte[16];
};

struct Touch3Info {
	char	SyncByte[16];
};

struct ParamInfo {
	char	SyncByte[16];
	unsigned short crc;
	unsigned int   len;
	char	param[490];
};

struct Reserve0Info {
	char	SyncByte[16];
	char	LockCode[8];
	char	Reserved[7];
	char	LockLevel;
	char	RecordVersion[32];
	char	YL_IMSI[12 * 16];
	char	Tele_IMSI[12 * 16];
	char	pad[64];
};

struct Reserve1Info {
	char	SyncByte[16];
};

struct Project0Info {
	char	SyncByte[16];
	char	PowerOffFlag;
	char	ProxCall[16];
	char	SecondMic;
	char	pad[478];
};

/*
 * sometimes we need to catch ramdump logs in release or TA version,
 * so the dload mode flag is writed to this block in engineer mode,
 * and read this flag in restart.c
 */
struct Project1Info {
	char	SyncByte[16];
	/* dload_flag: 49 go to dload mode, other is ignored. */
	char	dload_flag;
	char	pad[495];
};

struct Project2Info {
	char	SyncByte[16];
};

struct Project3Info {
	char	SyncByte[16];
};

struct Fetch_passwd {
	char	SyncByte[16];
};

struct Fct_DiagInfo {
	char	SyncByte[16];
	char	DUT_Flag;
	char	Reserved;
	char	PT_Flag[256];
	char	pad[238];
};

struct RCP_info {
	char	SyncByte[16];
	char	RFlag[4];
	char	RTime[4];
	char	AuthCode[260];
	char	EncryptoCode[116];
	char	CoolyunID[8];
	char	CoolyunPassword[32];
	char	LogStatus;
	char	pad[71];
};

struct ReturnZero_info {
	char	SyncByte[16];
	char	AlarmTime[4];
	char	AlarmAssigned;
	char	USBChargerType;
	char	BootNoVib;
	char	res;
	char	CommRunMode[4];
	char	RestartCount[4];
	char	pad[480];
};

struct VirtosPwd_info {
	char	SyncByte[16];
	char	SedKgjPassword[16];
	char	pad[480];
};

struct VirtOS_info {
	char	SyncByte[16];
	char	SafeBox[128];
	char	VirtOSPassword[16];
	char	pad[352];
};

struct yl_params_blocks {
	unsigned yl_params_block_ptr[64];
	unsigned yl_params_block_size[64];
};

/*DEVICE*/
#define DEVICE_PARAM_VER				16
#define DEVICE_DATA					18
#define DEVICE_COMMUNICATION_MODE_1			24
#define DEVICE_COMMUNICATION_MODE_2			40
#define DEVICE_IMAGE_SENSOR_MODEL			56
#define DEVICE_SAFE_BOX_KEY				72

/*CONFIGURATION*/
#define CONFIGURATION_PRODUCT_NAME			16
#define CONFIGURATION_U8_HARDWARE_VERSION_MAJOR_T_P	32
#define CONFIGURATION_U8_HARDWARE_VERSION_MAJOR_NUM	33
#define CONFIGURATION_U8_HARDWARE_VERSION_AUX_T_P	38
#define CONFIGURATION_U8_HARDWARE_VERSION_AUX_NUM	39
#define CONFIGURATION_U8_HARDWARE_VERSION_AUX_CHAR	41
#define CONFIGURATION_U8_HADRWARE_RF_NV			44

#define CONFIGURATION_MAIN_DEV_INFO_STRUCT_BASE		50
#define CONFIGURATION_MAIN_DEV_INFO_STRUCT_LEN		40
#define CONFIGURATION_MAIN_DEV_INFO_NAME		0
#define CONFIGURATION_MAIN_DEV_INFO_VENDOR		8
#define CONFIGURATION_MAIN_DEV_INFO_MODEL		24

/*PRODUCTLINE*/
#define PRODUCTLINE_SN					16
#define PRODUCTLINE_IMEI_1				32
#define PRODUCTLINE_IMEI_2				64
#define PRODUCTLINE_MODULE_CAL_STATUS_1			96
#define PRODUCTLINE_MODULE_CAL_STATUS_2			97
#define PRODUCTLINE_MODULE_RF_TEST_STATUS_1		98
#define PRODUCTLINE_MODULE_RF_TEST_STATUS_2		99
#define PRODUCTLINE_MODULE_COUPLING_TEST_STATUS_1	100
#define PRODUCTLINE_MODULE_COUPLING_TEST_STATUS_2	101
#define PRODUCTLINE_DM_TAG				102
#define PRODUCTLINE_CAMERA_CAL				103
#define PRODUCTLINE_RP_TAG				104
#define PRODUCTLINE_BATTERY_TEST			105
#define PRODUCTLINE_MODULE_SOFT_VERSION_1		106
#define PRODUCTLINE_MODULE_SOFT_VERSION_2		154
#define PRODUCTLINE_MODULE_AUDIO_VERSION_1		202
#define PRODUCTLINE_MODULE_AUDIO_VERSION_2		250
#define PRODUCTLINE_FUSE_BURN_STATUS			298
#define PRODUCTLINE_MISC_STATUS				299
#define PRODUCTLINE_SENSOR_CAL_INFO_LIGHT_PROX_INFO	304
#define PRODUCTLINE_SENSOR_CAL_INFO_ACC_INFO		312
#define PRODUCTLINE_SENSOR_CAL_INFO_PRESS_INFO		320
#define PRODUCTLINE_SENSOR_CAL_INFO_SENSOR_RESERVED_1	328
#define PRODUCTLINE_SENSOR_CAL_INFO_SENSOR_RESERVED_2	336
#define PRODUCTLINE_SENSOR_CAL_INFO_SENSOR_RESERVED_3	344
#define PRODUCTLINE_DSDS_IMEI				352

/*DYNAMIC*/
#define DYNAMIC_BSP					16
#define DYNAMIC_PASSWORD				48
#define DYNAMIC_SUPER_PASSWORD				64
#define DYNAMIC_DOWNLOAD_FLAG				80
#define DYNAMIC_DOWNLOAD_TOOL				96
#define DYNAMIC_SOFTWARE_VERSION			128
#define DYNAMIC_DIY_IMAGE_FLAG				176
#define DYNAMIC_SECURITY_FLAG				192
#define DYNAMIC_CTS_FLAG				193
#define DYNAMIC_DRM_FLAG				194
#define DYNAMIC_USB_CHARGE_FLAG				195
#define DYNAMIC_LTE_STATE				196
#define DYNAMIC_MISC_FLAGS				197
#define DYNAMIC_VIRGIN					256
#define DYNAMIC_NFC_UNLOCK_SCREEN_KEY			272


/*GUARD*/


/*CMDLINE*/
#define CMDLINE_CRC					16
#define CMDLINE_LEN					18


/*TOUCHSCREEN0*/


/*TOUCHSCREEN1*/


/*TOUCHSCREEN2*/


/*TOUCHSCREEN3*/

/*RESERVE0*/
#define RESERVE0_LOCK_CODE				16
#define RESERVE0_RESERVED				24
#define RESERVE0_LOCK_LEVEL				31
#define RESERVE0_RECORD_VERSION				32
#define RESERVE0_YL_IMSI				64
#define RESERVE0_TELE_IMSI				256

/*RESERVE1*/


/*PROJECT0*/
#define PROJECT0_POWER_OFF_FLAG				16
#define PROJECT0_PROX_CALL				17
#define PROJECT0_SECOND_MIC				33

/*PROJECT1*/


/*PROJECT2*/


/*PROJECT3*/


/*FETCH_PASSWD*/


/*FCT_DIAG*/
#define FCT_DIAG_DUT_FLAG				16
#define FCT_DIAG_RESERVED				17
#define FCT_DIAG_PT_FLAG				18

/*RCP*/
#define RCP_RFLAG					16
#define RCP_RTIME					20

/*RETURNZERO*/
#define RETURNZERO_AlARM_TIME				16
#define RETURNZERO_ALARM_ASSIGNED			20
#define RETURNZERO_USB_CHARGER_TYPE			21

#ifdef __KERNEL__
#include <linux/mmc/card.h>

extern int yl_params_init(struct mmc_card *card);
extern ssize_t yl_params_kernel_read(char *buf, ssize_t count);
extern ssize_t yl_params_kernel_write(char *buf, ssize_t count);
extern void notify_ylparams(struct hd_struct *part);
#endif /*__KERNEL__*/

#endif
