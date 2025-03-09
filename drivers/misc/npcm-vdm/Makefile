#
# Makefile for Nuvoton Miscellaneous optional drivers
#

ifeq ($(CONFIG_ARM64),y)
npcmx50_vdm-y := vdm_module.o vdm_common.o CircularBuffer.o
else
npcmx50_vdm-y := vdm_module.o vdm_common.o CircularBuffer.o ndelay-loop.o
endif
obj-$(CONFIG_NPCM_VDM)	         += npcmx50_vdm.o
