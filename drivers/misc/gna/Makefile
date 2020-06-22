# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile - Intel GMM & Neural Network Accelerator (Intel GNA) Linux driver
# Copyright (c) 2018-2019, Intel Corporation.
#

ccflags-y += -Werror

gna-y := gna_drv.o \
         gna_hw.o \
         gna_ioctl.o \
         gna_irq.o \
         gna_mem.o \
         gna_pci.o \
         gna_request.o \
         gna_score.o \

obj-m += gna.o

ifeq ($(KERNELRELEASE),)
ccflags-y += -I $(PWD)
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD)
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
endif
