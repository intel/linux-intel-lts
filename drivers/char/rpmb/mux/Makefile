# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2018-2019, Intel Corporation.

obj-$(CONFIG_RPMB_MUX) += rpmb_mux.o
rpmb_mux-objs := mux.o

obj-$(CONFIG_RPMB_MUX_KEY) += mux_hkdf.o
obj-$(CONFIG_RPMB_MUX_KEY) += key_abl.o
obj-$(CONFIG_RPMB_MUX_KEY) += key_sbl.o
obj-$(CONFIG_RPMB_MUX_KEY) += key.o

ccflags-y += -D__CHECK_ENDIAN__
