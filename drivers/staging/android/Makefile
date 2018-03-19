ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/
obj-$(CONFIG_ABL_BOOTLOADER_CONTROL)	+= abl/
obj-$(CONFIG_SBL_BOOTLOADER_CONTROL)	+= sbl/
obj-$(CONFIG_VSBL_BOOTLOADER_CONTROL)	+= vsbl/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
