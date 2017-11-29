# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(src)			# needed for trace events

obj-$(CONFIG_ABL_BOOTLOADER_CONTROL)	+= abl/
obj-$(CONFIG_SBL_BOOTLOADER_CONTROL)	+= sbl/
obj-$(CONFIG_VSBL_BOOTLOADER_CONTROL)	+= vsbl/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_DEBUG_KINFO)	+= debug_kinfo.o
obj-$(CONFIG_ANDROID_FWDATA)		+= fwdata.o
