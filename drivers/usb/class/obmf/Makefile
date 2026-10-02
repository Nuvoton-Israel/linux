# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the USB OBMF-ICP class driver
#

obj-$(CONFIG_USB_OBMF)		+= obmf.o
obmf-y					:= obmf-core.o obmf-transport.o obmf-discovery.o obmf-oem.o obmf-mmio-misc.o obmf-io-misc.o
obmf-$(CONFIG_USB_OBMF_I3C)		+= obmf-i3c.o
obmf-$(CONFIG_USB_OBMF_I2C)		+= obmf-i2c.o
obmf-$(CONFIG_USB_OBMF_GPIO)		+= obmf-gpio.o
obmf-$(CONFIG_USB_OBMF_SPI)		+= obmf-spi.o
obmf-$(CONFIG_USB_OBMF_MEM_DEV)	+= obmf-mem-dev.o
obmf-$(CONFIG_USB_OBMF_SERIAL)		+= obmf-serial.o
obmf-$(CONFIG_USB_OBMF_IPMI)		+= obmf-ipmi.o
