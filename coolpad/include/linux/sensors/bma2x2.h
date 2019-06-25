#ifndef __BMA250E_H__
#define __BMA250E_H__

#define GSENSOR_DEVICE_NAME			"yl_acc_sensor"
#define GSENSOR_INPUT_NAME			"yl_acc_input"

#define GSENSORIO	0x1D
#define GSENSOR_IOCTL_APP_SET_AFLAG	_IOW(GSENSORIO, 1, short)
#define GSENSOR_IOCTL_APP_SET_DELAY	_IOW(GSENSORIO, 2, short)
#define GSENSOR_IOCTL_APP_CALIBRATE	_IOR(GSENSORIO, 3, short)
#define GSENSOR_IOCTL_APP_OFFSET	_IOR(GSENSORIO, 4, short)	//modifed by wudongxing for factory pattern,2013.11.7

struct bosch_accel_platform_data {
	/* Although the accelerometer can perform at high ODR,
	 * there is a need to keep the maximum ODR to a lower
	 * value due to power consumption or other concern.
	 * Use this variable to set the minimum allowable
	 * interval for data to be reported from the
	 * accelerometer. Unit is measured in milli-
	 * seconds. Recommended value is 5ms. */
	unsigned int min_interval;
	/* Use this variable to set the default interval for
	 * data to be reported from the accelerometer. This
	 * value will be used during driver setup process,
	 * but can be changed by the system during runtime via
	 * sysfs control. Recommended value is 200ms.*/
	unsigned int poll_interval;

	/* This variable controls the corresponding direction
	 */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negative_x;
	u8 negative_y;
	u8 negative_z;

	/* Use this variable to control the G range of
	 * the accelerometer output. Use the macro definition
	 * to select the desired G range.*/

	u8 acc_range;
};

struct bma2x2acc {
	s16 x, y, z;
};

struct acc_offset {
	signed short key;	/* calibrate status */
	signed short x;		/* x offset */
	signed short y;		/* y offset */
	signed short z;		/* z offset */
};

#endif
