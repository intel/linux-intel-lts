// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/init_task.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/poll.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
#include <linux/sched.h>
#else
#include <uapi/linux/sched/types.h>
#endif
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
#include <linux/dma-attrs.h>
#else
#include <linux/dma-mapping.h>
#endif

#include <uapi/linux/ipu-psys.h>

#include "ipu.h"
#include "ipu-bus.h"
#include "ipu-platform.h"
#include "ipu-buttress.h"
#include "ipu-cpd.h"
#include "ipu-fw-psys.h"
#include "ipu-platform-regs.h"
#include "ipu-fw-isys.h"
#include "ipu-fw-com.h"

#include <linux/vhm/acrn_vhm_mm.h>
#include "virtio/intel-ipu4-virtio-common.h"
#include "virtio/intel-ipu4-virtio-common-psys.h"
#include "virtio/intel-ipu4-virtio-be.h"
#include "ipu-psys-virt.h"

int psys_get_manifest(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info);
int psys_map_buf(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info);
int psys_unmap_buf(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info);
int psys_qcmd(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info);
int psys_dqevent(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info);
int psys_get_buf(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info);

int psys_get_manifest(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info)
{
	struct ipu_device *isp = psys->adev->isp;
	struct ipu_cpd_client_pkg_hdr *client_pkg;
	u32 entries;
	void *host_fw_data;
	dma_addr_t dma_fw_data;
	u32 client_pkg_offset;

	struct ipu_psys_manifest_virt *manifest;
	manifest = (struct ipu_psys_manifest_virt *)map_guest_phys(
										req_info->domid,
										req_info->request->payload,
										PAGE_SIZE
										);
	if (manifest == NULL) {
		pr_err("%s: failed to get payload", __func__);
		return -EFAULT;
	}

	host_fw_data = (void *)isp->cpd_fw->data;
	dma_fw_data = sg_dma_address(psys->fw_sgt.sgl);

	entries = ipu_cpd_pkg_dir_get_num_entries(psys->pkg_dir);
	if (!manifest || manifest->index > entries - 1) {
		dev_err(&psys->adev->dev, "invalid argument\n");
		return -EINVAL;
	}

	if (!ipu_cpd_pkg_dir_get_size(psys->pkg_dir, manifest->index) ||
		ipu_cpd_pkg_dir_get_type(psys->pkg_dir, manifest->index) <
		IPU_CPD_PKG_DIR_CLIENT_PG_TYPE) {
		dev_dbg(&psys->adev->dev, "invalid pkg dir entry\n");
		return -ENOENT;
	}

	client_pkg_offset = ipu_cpd_pkg_dir_get_address(psys->pkg_dir,
							manifest->index);
	client_pkg_offset -= dma_fw_data;

	client_pkg = host_fw_data + client_pkg_offset;
	manifest->size = client_pkg->pg_manifest_size;

	if (manifest->size > PAGE_SIZE) {
		pr_err("%s: manifest size is more than 1 page %d",
										__func__,
										manifest->size);
		return -EFAULT;
	}

	memcpy(&manifest->manifest,
			(uint8_t *) client_pkg + client_pkg->pg_manifest_offs,
			manifest->size);

	return 0;
}

int psys_map_buf(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

int psys_unmap_buf(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

int psys_qcmd(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

int psys_dqevent(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

int psys_get_buf(struct ipu_psys *psys,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

struct psys_fops_virt psys_vfops = {
	.get_manifest = psys_get_manifest,
	.map_buf = psys_map_buf,
	.unmap_buf = psys_unmap_buf,
	.qcmd = psys_qcmd,
	.dqevent = psys_dqevent,
	.get_buf = psys_get_buf,
};
