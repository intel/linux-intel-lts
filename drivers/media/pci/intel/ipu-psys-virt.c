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
#include "ipu-psys.h"

#include <linux/vhm/acrn_vhm_mm.h>
#include "virtio/intel-ipu4-virtio-common.h"
#include "virtio/intel-ipu4-virtio-common-psys.h"
#include "virtio/intel-ipu4-virtio-be.h"
#include "ipu-psys-virt.h"

extern struct dma_buf_ops ipu_dma_buf_ops;

#define POLL_WAIT 500 //500ms

int virt_ipu_psys_get_manifest(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys *psys = fh->psys;
	struct ipu_device *isp = psys->adev->isp;
	struct ipu_cpd_client_pkg_hdr *client_pkg;
	u32 entries;
	void *host_fw_data;
	dma_addr_t dma_fw_data;
	u32 client_pkg_offset;
	struct ipu_psys_manifest_wrap *manifest_wrap;
	struct ipu_psys_manifest *manifest;
	void *manifest_data;

	manifest_wrap = (struct ipu_psys_manifest_wrap *)map_guest_phys(
										req_info->domid,
										req_info->request->payload,
										PAGE_SIZE
										);
	if (manifest_wrap == NULL) {
		pr_err("%s: failed to get payload", __func__);
		return -EFAULT;
	}

	manifest = (struct ipu_psys_manifest *)map_guest_phys(
										req_info->domid,
										manifest_wrap->psys_manifest,
										PAGE_SIZE
										);
	if (manifest == NULL) {
		pr_err("%s: failed to get ipu_psys_manifest", __func__);
		return -EFAULT;
	}

	manifest_data = (void *)map_guest_phys(
							req_info->domid,
							manifest_wrap->manifest_data,
							PAGE_SIZE
							);
	if (manifest_data == NULL) {
		pr_err("%s: failed to get manifest_data", __func__);
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

	memcpy(manifest_data,
		(uint8_t *) client_pkg + client_pkg->pg_manifest_offs,
		manifest->size);

	return 0;
}

int virt_ipu_psys_map_buf(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

int virt_ipu_psys_unmap_buf(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info)
{
	int fd;

	fd = req_info->request->op[0];

	return ipu_psys_unmapbuf(fd, fh);
}

int virt_ipu_psys_qcmd(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info)
{
	return -1;
}

int virt_ipu_psys_dqevent(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info,
			unsigned int f_flags)
{
	struct ipu_psys_event *event;

	event = (struct ipu_psys_event *)map_guest_phys(
									req_info->domid,
									req_info->request->payload,
									PAGE_SIZE
									);
	if (event == NULL) {
		pr_err("%s: failed to get payload", __func__);
		return -EFAULT;
	}

	return ipu_ioctl_dqevent(event, fh, f_flags);
}

int virt_ipu_psys_poll(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info)
{
	struct ipu_psys *psys = fh->psys;
	long time_remain = -1;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	dev_dbg(&psys->adev->dev, "ipu psys poll\n");

	add_wait_queue(&fh->wait, &wait);
	while (1) {
		if (ipu_get_completed_kcmd(fh) ||
			time_remain == 0)
			break;
		time_remain =
			wait_woken(&wait, TASK_INTERRUPTIBLE, POLL_WAIT);
	}
	remove_wait_queue(&fh->wait, &wait);

	if (time_remain)
		req_info->request->func_ret = POLLIN;
	else
		req_info->request->func_ret = 0;

	dev_dbg(&psys->adev->dev, "ipu psys poll res %u\n",
						req_info->request->func_ret);

	return 0;
}

int __map_buf(struct ipu_psys_fh *fh,
		struct ipu_psys_buffer_wrap *buf_wrap,
		struct ipu_psys_kbuffer *kbuf,
		int domid, int fd)
{
	struct ipu_psys *psys = fh->psys;
	struct dma_buf *dbuf;
	int ret = -1, i;
	struct ipu_dma_buf_attach *ipu_attach;
	struct page **data_pages = NULL;
	u64 *page_table = NULL;
	void *pageaddr;

	mutex_lock(&fh->mutex);
	kbuf->dbuf = dma_buf_get(fd);
	if (IS_ERR(kbuf->dbuf)) {
		goto error_get;
	}

	if (kbuf->len == 0)
		kbuf->len = kbuf->dbuf->size;

	kbuf->fd = fd;

	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, &psys->adev->dev);
	if (IS_ERR(kbuf->db_attach)) {
		ret = PTR_ERR(kbuf->db_attach);
		goto error_put;
	}

	data_pages = kcalloc(buf_wrap->map.npages, sizeof(struct page *), GFP_KERNEL);
	if (data_pages == NULL) {
		pr_err("%s: Failed alloc data page set", __func__);
		goto error_put;
	}

	pr_debug("%s: Total number of pages:%lu",
		__func__, buf_wrap->map.npages);

	page_table = (u64 *)map_guest_phys(domid,
		buf_wrap->map.page_table_ref, PAGE_SIZE);

	if (page_table == NULL) {
		pr_err("%s: Failed to map page table", __func__);
		kfree(data_pages);
		goto error_detach;
	} else {
		 pr_debug("%s: first page %lld",
				__func__, page_table[0]);
		for (i = 0; i < buf_wrap->map.npages; i++) {
			pageaddr = map_guest_phys(domid,
					page_table[i], PAGE_SIZE);
			if (pageaddr == NULL) {
				pr_err("%s: Cannot map pages from UOS", __func__);
				break;
			}
			data_pages[i] = virt_to_page(pageaddr);
		}
	}

	ipu_attach = kbuf->db_attach->priv;
	ipu_attach->npages = buf_wrap->map.npages;
	ipu_attach->pages = data_pages;
	ipu_attach->vma_is_io = buf_wrap->map.vma_is_io;

	kbuf->sgt = dma_buf_map_attachment(kbuf->db_attach, DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(kbuf->sgt)) {
		ret = -EINVAL;
		kbuf->sgt = NULL;
		dev_dbg(&psys->adev->dev, "map attachment failed\n");
		goto error_detach;
	}

	kbuf->dma_addr = sg_dma_address(kbuf->sgt->sgl);

	kbuf->kaddr = dma_buf_vmap(kbuf->dbuf);
	if (!kbuf->kaddr) {
		ret = -EINVAL;
		goto error_unmap;
	}

	kbuf->valid = true;

	mutex_unlock(&fh->mutex);

	return 0;

error_unmap:
	dma_buf_unmap_attachment(kbuf->db_attach, kbuf->sgt, DMA_BIDIRECTIONAL);
error_detach:
	dma_buf_detach(kbuf->dbuf, kbuf->db_attach);
	kbuf->db_attach = NULL;
error_put:
	dbuf = kbuf->dbuf;
	dma_buf_put(dbuf);
error_get:
	mutex_unlock(&fh->mutex);

	return ret;
}

int virt_ipu_psys_get_buf(struct ipu_psys_fh *fh,
			struct ipu4_virtio_req_info *req_info)
{
	struct dma_buf *dbuf;
	int ret;
	struct ipu_psys_buffer_wrap *buf_wrap;
	struct ipu_psys_buffer *buf;
	struct ipu_psys_kbuffer *kbuf;
	struct ipu_psys *psys = fh->psys;

	buf_wrap = (struct ipu_psys_buffer_wrap *)map_guest_phys(
										req_info->domid,
										req_info->request->payload,
										PAGE_SIZE
										);
	if (buf_wrap == NULL) {
		pr_err("%s: failed to get payload", __func__);
		return -EFAULT;
	}

	buf = (struct ipu_psys_buffer *)map_guest_phys(
										req_info->domid,
										buf_wrap->psys_buf,
										PAGE_SIZE
										);
	if (buf == NULL) {
		pr_err("%s: failed to get ipu_psys_buffer", __func__);
		return -EFAULT;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
#endif

	if (!buf->base.userptr) {
		dev_err(&psys->adev->dev, "Buffer allocation not supported\n");
		return -EINVAL;
	}

	kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	kbuf->len = buf->len;
	kbuf->userptr = buf->base.userptr;
	kbuf->flags = buf->flags;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	exp_info.ops = &ipu_dma_buf_ops;
	exp_info.size = kbuf->len;
	exp_info.flags = O_RDWR;
	exp_info.priv = kbuf;

	dbuf = dma_buf_export(&exp_info);
#else
	dbuf = dma_buf_export(kbuf, &ipu_dma_buf_ops, kbuf->len, 0);
#endif
	if (IS_ERR(dbuf)) {
		kfree(kbuf);
		return PTR_ERR(dbuf);
	}

	ret = dma_buf_fd(dbuf, 0);
	if (ret < 0) {
		kfree(kbuf);
		return ret;
	}

	dev_dbg(&psys->adev->dev, "IOC_GETBUF: userptr %p", buf->base.userptr);

	kbuf->fd = ret;
	buf->base.fd = ret;
	kbuf->flags = buf->flags &= ~IPU_BUFFER_FLAG_USERPTR;
	kbuf->flags = buf->flags |= IPU_BUFFER_FLAG_DMA_HANDLE;

	ret = __map_buf(fh, buf_wrap, kbuf, req_info->domid, kbuf->fd);
	if (ret < 0) {
		kfree(kbuf);
		return ret;
	}

	mutex_lock(&fh->mutex);
	list_add_tail(&kbuf->list, &fh->bufmap);
	mutex_unlock(&fh->mutex);

	dev_dbg(&psys->adev->dev, "to %d\n", buf->base.fd);

	return 0;
}

struct psys_fops_virt psys_vfops = {
	.get_manifest = virt_ipu_psys_get_manifest,
	.map_buf = virt_ipu_psys_map_buf,
	.unmap_buf = virt_ipu_psys_unmap_buf,
	.qcmd = virt_ipu_psys_qcmd,
	.dqevent = virt_ipu_psys_dqevent,
	.get_buf = virt_ipu_psys_get_buf,
	.poll = virt_ipu_psys_poll,
};
