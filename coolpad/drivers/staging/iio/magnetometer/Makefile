#
# Makefile for industrial I/O Magnetometer sensors
#

obj-$(CONFIG_SENSORS_YAS_MAGNETOMETER)	+= yas_mag_drv-yas537.o yas_mag_drv-yas532.o yas_mag_kernel.o

CFLAGS_yas_mag_kernel.o += -Icoolpad/drivers/staging/iio

obj-$(CONFIG_SENSORS_BMA_ACCELEROMETER) += yas_acc_drv-bma250e.o yas_acc_kernel.o
obj-$(CONFIG_SENSORS_LIS3DH_ACCELEROMETER) += yas_acc_drv-lis3dh.o yas_acc_kernel.o
obj-$(CONFIG_SENSORS_KXTJ2_ACCELEROMETER) += yas_acc_drv-kxtj2.o yas_acc_kernel.o
CFLAGS_yas_acc_kernel.o += -Icoolpad/drivers/staging/iio
