# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2018-2019, Intel Corporation.

obj-$(CONFIG_RPMB_MUX) += rpmb_mux.o
rpmb_mux-objs := mux.o

obj-$(CONFIG_RPMB_KEY) += mux_hkdf.o

ccflags-y += -D__CHECK_ENDIAN__
