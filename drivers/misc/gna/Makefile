# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile - Intel GMM & Neural Network Accelerator (Intel GNA) Linux driver
# Copyright (c) 2018-2021, Intel Corporation.
#

ccflags-y += -Werror

gna-y := device.o \
         hw.o \
         ioctl.o \
         mem.o \
         pci.o \
         request.o \
         score.o

obj-m += gna.o
