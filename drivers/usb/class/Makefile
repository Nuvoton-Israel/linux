# SPDX-License-Identifier: GPL-2.0
#
# Makefile for USB Class drivers
# (one step up from the misc category)
#

obj-$(CONFIG_USB_ACM)		+= cdc-acm.o
obj-$(CONFIG_USB_PRINTER)	+= usblp.o
obj-$(CONFIG_USB_WDM)		+= cdc-wdm.o
obj-$(CONFIG_USB_TMC)		+= usbtmc.o
obj-$(CONFIG_USB_OBMF)		+= obmf.o
obmf-y					:= obmf-core.o obmf-transport.o obmf-discovery.o obmf-oem.o obmf-mmio-misc.o
obmf-$(CONFIG_USB_OBMF_I2C)		+= obmf-i2c.o
obmf-$(CONFIG_USB_OBMF_GPIO)		+= obmf-gpio.o
obmf-$(CONFIG_USB_OBMF_SPI)		+= obmf-spi.o
obmf-$(CONFIG_USB_OBMF_SERIAL)		+= obmf-serial.o
obmf-$(CONFIG_USB_OBMF_IPMI)		+= obmf-ipmi.o
