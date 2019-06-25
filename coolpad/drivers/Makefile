#
# Makefile for the Linux kernel device drivers.
#
# 15 Sep 2000, Christoph Hellwig <hch@infradead.org>
# Rewritten to use lists instead of if-statements.
#

# Following the sequences in kernel/drivers/Makefile
obj-$(CONFIG_REGULATOR)		+= regulator/
obj-y				+= misc/
obj-$(CONFIG_INPUT)		+= input/
obj-$(CONFIG_POWER_SUPPLY)	+= power/
obj-y				+= mmc/
obj-y				+= leds/
obj-$(CONFIG_STAGING)		+= staging/
obj-$(CONFIG_YL_SENSORS)	+= sensors/
obj-$(CONFIG_USB)               += usb/

# Without this, built-in.o won't be created when it's empty, and the
# final vmlinux link will fail.
obj-n := dummy
