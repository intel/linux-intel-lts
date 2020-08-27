/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/* Copyright (C) 2019 Intel Corporation */
#ifndef VPUSMM_H
#define VPUSMM_H

#include <linux/ioctl.h>

#define VPUSMM_IOC_MAGIC 'V'

struct vpusmm_args_alloc {
	int fd;
	int force_physical_contiguous;
	int cachable;
	unsigned long size;
};

struct vpusmm_args_import {
	int fd;
	int vpu_access;
	unsigned long vpu_addr;
	unsigned long size;
};

#define VPUSMM_IOCTL_ALLOC \
	_IOWR(VPUSMM_IOC_MAGIC, 0, struct vpusmm_args_alloc)

#define VPUSMM_IOCTL_IMPORT \
	_IOWR(VPUSMM_IOC_MAGIC, 1, struct vpusmm_args_import)

#define VPUSMM_IOCTL_UNIMPORT \
	_IOWR(VPUSMM_IOC_MAGIC, 2, int)

#define VPUSMM_IOCTL_PTR2VPU \
	_IOWR(VPUSMM_IOC_MAGIC, 3, unsigned long)

#define VPUSMM_IOCTL_PTR2PHYS \
	_IOWR(VPUSMM_IOC_MAGIC, 4, unsigned long)

#define VPUSMM_IOCTL_END \
	_IO(VPUSMM_IOC_MAGIC, 5)

#endif
