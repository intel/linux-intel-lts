/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef __IPU4_VIRTIO_COMMON_PSYS_H__
#define __IPU4_VIRTIO_COMMON_PSYS_H__

struct ipu_psys_manifest_wrap {
	u64 psys_manifest;
	//since the manifest memory is allocated by user space
	//and the struct ia_cipr_buffer_t is not expose to
	//driver. We assume the size is less than 1 page and
	//allocate the max.
	int8_t manifest[PAGE_SIZE];
};

struct ipu_psys_usrptr_map {
	bool vma_is_io;
	u64 page_table_ref;
	size_t npages;
};

struct ipu_psys_buffer_wrap {
	u64 psys_buf;
	struct ipu_psys_usrptr_map map;
};

#endif
