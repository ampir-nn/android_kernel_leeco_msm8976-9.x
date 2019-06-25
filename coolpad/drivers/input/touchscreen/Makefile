#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.

# zouxin add 120117  ###huangxu modify for s3202
ifeq ($(CONFIG_TOUCHSCREEN_FOCALTECH_5446),y)
obj-$(CONFIG_TOUCHSCREEN_HAL)		+= touchscreen_yl_TBD.o
else
obj-$(CONFIG_TOUCHSCREEN_HAL)		+= touchscreen_yl.o
endif

# zouxin add end 120117
obj-$(CONFIG_TOUCHSCREEN_GOODIX_TS)	+= goodix-ts/
obj-$(CONFIG_TOUCHSCREEN_GOODIX)	+= goodix/
obj-$(CONFIG_TOUCHSCREEN_GT1XX)		+= gt1xx/
# litao3 add 121228 for FT5406
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH)	+= focaltech/
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_TS)	+= focaltech_ts/
# litao3 add end
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_TS)	+= synaptics_ts/

obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_5446)	+= ft5446/

obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_5346)	+= ft5346/

obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX)  += synaptics_dsx/

obj-$(CONFIG_TOUCHSCREEN_SHTSC)         += shtsc/
