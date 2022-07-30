# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2010 - 2022 Intel Corporation.

ifneq ($(EXTERNAL_BUILD), 1)
srcpath := $(srctree)
endif

# force check the compile warning to make sure zero warnings
# note we may have build issue when gcc upgraded.
ccflags-y := -Wall -Wextra
ccflags-y += $(call cc-disable-warning, unused-parameter)
ccflags-y += $(call cc-disable-warning, implicit-fallthrough)
ccflags-y += $(call cc-disable-warning, missing-field-initializers)
ccflags-$(CONFIG_VIDEO_INTEL_IPU_WERROR) += -Werror

ccflags-y += -I$(srcpath)/$(src)/../../../../include/
ccflags-y += -I$(srcpath)/$(src)/../../pci/intel/

obj-$(CONFIG_INTEL_IPU6_TGLRVP_PDATA) += ipu6-tglrvp-pdata.o
obj-$(CONFIG_INTEL_IPU6_ADLRVP_PDATA) += ipu6-adlrvp-pdata.o
obj-$(CONFIG_INTEL_IPU6_ACPI)         += ipu6-acpi.o
