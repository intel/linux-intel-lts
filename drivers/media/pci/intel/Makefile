# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the IPU3 cio2 and ImGU drivers
#

obj-y	+= ipu3/
# force check the compile warning to make sure zero warnings
# note we may have build issue when gcc upgraded.
ccflags-y := -Wno-error=uninitialized
subdir-ccflags-y += $(call cc-disable-warning, unused-parameter)
subdir-ccflags-y += $(call cc-disable-warning, implicit-fallthrough)
subdir-ccflags-y += $(call cc-disable-warning, missing-field-initializers)
subdir-ccflags-y += $(call cc-disable-warning, int-conversion)
subdir-ccflags-y += $(call cc-disable-warning, enum-conversion)
subdir-ccflags-y += $(call cc-disable-warning, uninitialized)
subdir-ccflags-$(CONFIG_VIDEO_INTEL_IPU_WERROR) += -Werror

obj-$(CONFIG_VIDEO_INTEL_IPU4)  += ipu4/
obj-$(CONFIG_VIDEO_INTEL_IPU4P)	+= ipu4/
