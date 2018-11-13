/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef __IPU4_VIRTIO_COMMON_PSYS_H__
#define __IPU4_VIRTIO_COMMON_PSYS_H__

struct ipu_psys_manifest_virt {
	uint32_t index;
	uint32_t size;
	//since the manifest memory is allocated by user space
	//and the struct ia_cipr_buffer_t is not expose to
	//driver. We assume the size is less than 1 page and
	//allocate the max.
	uint8_t manifest[PAGE_SIZE];
};

#endif
