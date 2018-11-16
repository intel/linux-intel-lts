// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/highmem.h>
#include <linux/init_task.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/poll.h>
#include <linux/hashtable.h>
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

#include "intel-ipu4-para-virt-psys.h"
#include "intel-ipu4-virtio-common.h"
#include "intel-ipu4-virtio-common-psys.h"
#include "intel-ipu4-virtio-fe-request-queue.h"
#include "intel-ipu4-virtio-fe-payload.h"

#define FD_MAX_SIZE	8
#define IPU_PSYS_NUM_DEVICES		4
#define IPU_PSYS_NAME	"intel-ipu4-psys"

DECLARE_HASHTABLE(FD_BUF_HASH, FD_MAX_SIZE);

#ifdef CONFIG_COMPAT
extern long virt_psys_compat_ioctl32(struct file *file, unsigned int cmd,
			     unsigned long arg);
#endif

static dev_t virt_psys_dev_t;
static struct virt_ipu_psys *g_psys;
static struct class *virt_psys_class;

static DECLARE_BITMAP(virt_psys_devices, IPU_PSYS_NUM_DEVICES);
static DEFINE_MUTEX(psys_mutex);

int ipu_get_manifest(struct ipu_psys_manifest *m,
				 struct virt_ipu_psys_fh *fh)
{
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	struct ipu_psys_manifest_wrap *manifest_wrap;
	int rval = 0;
	void *manifest_data;

	pr_debug("%s: processing start", __func__);

	manifest_wrap = kzalloc(sizeof(struct ipu_psys_manifest_wrap),
								GFP_KERNEL);

	manifest_wrap->psys_manifest = virt_to_phys(m);

	//since the manifest memory is allocated by user space
	//and the struct ia_cipr_buffer_t is not expose to
	//driver. We assume the size is less than 1 page and
	//allocate the max.
	manifest_data = kzalloc(PAGE_SIZE, GFP_KERNEL);
	manifest_wrap->manifest_data = virt_to_phys(manifest_data);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	req->payload = virt_to_phys(manifest_wrap);

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_GET_MANIFEST, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);
	if (rval) {
		pr_err("%s: Failed to get manifest", __func__);
		goto error_exit;
	}

	if (m->manifest != NULL && copy_to_user(m->manifest,
			manifest_data,
			m->size)) {
		pr_err("%s: Failed copy_to_user", __func__);
		rval = -EFAULT;
		goto error_exit;
	}

error_exit:

	kfree(manifest_data);
	kfree(manifest_wrap);

	ipu4_virtio_fe_req_queue_put(req);

	pr_debug("%s: processing ended %d", __func__, rval);

	return rval;
}

int ipu_query_caps(struct ipu_psys_capability *caps,
				 struct virt_ipu_psys_fh *fh)
{
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	int rval = 0;

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	req->payload = virt_to_phys(caps);

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_QUERYCAP, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);
	if (rval) {
		pr_err("%s: Failed to query capability", __func__);
		ipu4_virtio_fe_req_queue_put(req);
		return rval;
	}

	ipu4_virtio_fe_req_queue_put(req);

	pr_debug("%s: processing ended %d", __func__, rval);

	return rval;
}

int ipu_psys_kcmd_new(struct ipu_psys_command *cmd,
				struct virt_ipu_psys_fh *fh)
{
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	struct ipu_psys_command_wrap *cmd_wrap = NULL;
	struct ipu_psys_buffer *psys_buffers = NULL;
	void *pg_manifest = NULL;

	int rval = 0;

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	cmd_wrap = kzalloc(sizeof(struct ipu_psys_command_wrap),
								GFP_KERNEL);

	/* Allocate for pg_manifest */
	pg_manifest = kzalloc(cmd->pg_manifest_size, GFP_KERNEL);

	/* Copy data from user */
	if (copy_from_user(pg_manifest,
				cmd->pg_manifest,
				cmd->pg_manifest_size)) {
		pr_err("%s, Failed copy_from_user", __func__);
		rval = -EFAULT;
		goto error_exit;
	}


	/* Map pg_manifest to physical address */
	cmd_wrap->psys_manifest = virt_to_phys(pg_manifest);

	/* Map ipu_psys_command to physical address */
	cmd_wrap->psys_command = virt_to_phys(cmd);

	psys_buffers = kcalloc(cmd->bufcount,
								sizeof(struct ipu_psys_buffer),
								GFP_KERNEL);

	if (copy_from_user(psys_buffers, 
						cmd->buffers,
						cmd->bufcount * sizeof(struct ipu_psys_buffer))) {
		pr_err("%s, Failed copy_from_user", __func__);
		rval = -EFAULT;
		goto error_exit;
	}

	/* Map ipu_psys_buffer to physical address */
	cmd_wrap->psys_buffer = virt_to_phys(psys_buffers);

	req->payload = virt_to_phys(cmd_wrap);

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_QCMD, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);

	if (rval) {
		pr_err("%s: Failed to queue command", __func__);
		goto error_exit;
	}

error_exit:
	if (pg_manifest) kfree(pg_manifest);
	if (cmd_wrap) kfree(cmd_wrap);
	if (psys_buffers) kfree(psys_buffers);

	ipu4_virtio_fe_req_queue_put(req);

	return rval;
}

int psys_get_userpages(struct ipu_psys_buffer *buf,
				struct ipu_psys_usrptr_map *map)
{
	struct vm_area_struct *vma;
	unsigned long start, end;
	int npages, array_size;
	struct page **pages;
	u64 *page_table;
	int nr = 0, i;
	int ret = -ENOMEM;

	start = (unsigned long)buf->base.userptr;
	end = PAGE_ALIGN(start + buf->len);
	npages = (end - (start & PAGE_MASK)) >> PAGE_SHIFT;
	array_size = npages * sizeof(struct page *);

	page_table = kcalloc(npages, sizeof(*page_table), GFP_KERNEL);
	if (!page_table) {
		pr_err("%s: Shared Page table for mediation failed", __func__);
		return -ENOMEM;
	}

	if (array_size <= PAGE_SIZE)
		pages = kzalloc(array_size, GFP_KERNEL);
	else
		pages = vzalloc(array_size);
	if (!pages) {
		pr_err("%s: failed to get userpages:%d", __func__, -ENOMEM);
		ret = -ENOMEM;
		goto exit_page_table;
	}

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, start);
	if (!vma) {
		ret = -EFAULT;
		goto exit_up_read;
	}

	if (vma->vm_end < start + buf->len) {
		pr_err("%s: vma at %lu is too small for %llu bytes",
			__func__, start, buf->len);
		ret = -EFAULT;
		goto exit_up_read;
	}

	/*
	 * For buffers from Gralloc, VM_PFNMAP is expected,
	 * but VM_IO is set. Possibly bug in Gralloc.
	 */
	map->vma_is_io = vma->vm_flags & (VM_IO | VM_PFNMAP);

	if (map->vma_is_io) {
		unsigned long io_start = start;

		for (nr = 0; nr < npages; nr++, io_start += PAGE_SIZE) {
			unsigned long pfn;

			ret = follow_pfn(vma, io_start, &pfn);
			if (ret)
				goto exit_up_read;
			pages[nr] = pfn_to_page(pfn);
		}
	} else {
		nr = get_user_pages(
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
				   current, current->mm,
#endif
				   start & PAGE_MASK, npages,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
				   1, 0,
#else
				   FOLL_WRITE,
#endif
				   pages, NULL);
		if (nr < npages)
			goto exit_pages;
	}

	for (i = 0; i < npages; i++)
		page_table[i] = page_to_phys(pages[i]);

	map->page_table_ref = virt_to_phys(page_table);
	map->len = buf->len;
	map->userptr = buf->base.userptr;

	up_read(&current->mm->mmap_sem);

	map->npages = npages;

	if (array_size <= PAGE_SIZE)
		kfree(pages);
	else
		vfree(pages);

	return 0;

exit_pages:
	if (!map->vma_is_io)
		while (nr > 0)
			put_page(pages[--nr]);

	if (array_size <= PAGE_SIZE)
		kfree(pages);
	else
		vfree(pages);
exit_up_read:
	up_read(&current->mm->mmap_sem);
exit_page_table:
	kfree(page_table);

	return ret;
}

static void psys_put_userpages(struct ipu_psys_usrptr_map *map)
{
	unsigned long start, end;
	int npages, i;
	u64 *page_table;
	struct page *pages;
	struct mm_struct* mm;

	start = (unsigned long)map->userptr;
	end = PAGE_ALIGN(start + map->len);
	npages = (end - (start & PAGE_MASK)) >> PAGE_SHIFT;

	mm = current->active_mm;
	if (!mm){
		pr_err("%s: Failed to get active mm_struct ptr from current process",
			__func__);
		return;
	}

	down_read(&mm->mmap_sem);

	page_table = phys_to_virt(map->page_table_ref);
	for (i = 0; i < npages; i++) {
		pages = phys_to_page(page_table[i]);
		set_page_dirty_lock(pages);
		put_page(pages);
	}

	kfree(page_table);

	up_read(&mm->mmap_sem);
}

static struct ipu_psys_buffer_wrap *ipu_psys_buf_lookup(
											int fd)
{
	struct ipu_psys_buffer_wrap *psys_buf_wrap;

	hash_for_each_possible(FD_BUF_HASH, psys_buf_wrap, node, fd) {
		if (psys_buf_wrap)
			return psys_buf_wrap;
	}

	return NULL;
}

int ipu_psys_getbuf(struct ipu_psys_buffer *buf,
				struct virt_ipu_psys_fh *fh)
{
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	struct ipu_psys_buffer_wrap *attach;
	int rval = 0;

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	attach = kzalloc(sizeof(struct ipu_psys_buffer_wrap),
								GFP_KERNEL);

	attach->psys_buf = virt_to_phys(buf);

	if (psys_get_userpages(buf, &attach->map)) {
		goto error_exit;
	}

	req->payload = virt_to_phys(attach);

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_GETBUF, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);
	if (rval) {
		pr_err("%s: Failed to get buf", __func__);
		psys_put_userpages(&attach->map);
		goto error_exit;
	}

	mutex_lock(&fh->mutex);
	if(!ipu_psys_buf_lookup(buf->base.fd)) {
		hash_add(FD_BUF_HASH, &attach->node, buf->base.fd);
	}
	mutex_unlock(&fh->mutex);

	goto exit;

error_exit:

	kfree(attach);

exit:
	ipu4_virtio_fe_req_queue_put(req);

	pr_debug("%s: processing ended %d", __func__, rval);

	return rval;
}

int ipu_psys_unmapbuf(int fd, struct virt_ipu_psys_fh *fh)
{
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	struct ipu_psys_buffer_wrap *psys_buf_wrap;
	int rval = 0, op[1];

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	op[0] = fd;

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_UNMAPBUF, &op[0]);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);
	if (rval) {
		pr_err("%s: Failed to unmapbuf", __func__);
		goto error_exit;
	}

	mutex_lock(&fh->mutex);
	psys_buf_wrap = ipu_psys_buf_lookup(fd);
	if (psys_buf_wrap) {
		psys_put_userpages(&psys_buf_wrap->map);
		hash_del(&psys_buf_wrap->node);
		kfree(psys_buf_wrap);
	}
	mutex_unlock(&fh->mutex);

error_exit:

	ipu4_virtio_fe_req_queue_put(req);

	pr_debug("%s: processing ended %d", __func__, rval);

	return rval;
}

unsigned int virt_psys_poll(struct file *file,
						  struct poll_table_struct *wait)
{
	struct virt_ipu_psys_fh *fh = file->private_data;
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	int rval = 0;

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_POLL, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);
	if (rval) {
		pr_err("%s: Failed psys polling", __func__);
		ipu4_virtio_fe_req_queue_put(req);
		return rval;
	}

	rval = req->func_ret;

	ipu4_virtio_fe_req_queue_put(req);

	pr_debug("%s: processing ended %d", __func__, rval);

	return rval;
}

long ipu_ioctl_dqevent(struct ipu_psys_event *event,
			      struct virt_ipu_psys_fh *fh, unsigned int f_flags)
{
	struct virt_ipu_psys *psys = fh->psys;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	int rval = 0;

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req)
		return -ENOMEM;

	req->payload = virt_to_phys(event);

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_DQEVENT, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
									IPU_VIRTIO_QUEUE_1);
	if (rval) {
		pr_err("%s: Failed to dqevent", __func__);
		goto error_exit;
	}

error_exit:

	ipu4_virtio_fe_req_queue_put(req);

	pr_debug("%s: processing ended %d", __func__, rval);

	return rval;
}

static long virt_psys_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	union kargs {
		struct ipu_psys_buffer buf;
		struct ipu_psys_command cmd;
		struct ipu_psys_event ev;
		struct ipu_psys_capability caps;
		struct ipu_psys_manifest m;
	};
	int err = 0;
	union kargs *data = NULL;

	struct virt_ipu_psys_fh *fh = file->private_data;
	if(fh == NULL)
		return -EFAULT;
	void __user *up = (void __user *)arg;
	bool copy = (cmd != IPU_IOC_MAPBUF && cmd != IPU_IOC_UNMAPBUF);

	if (copy) {
		if (_IOC_SIZE(cmd) > sizeof(union kargs)) {
			pr_err("%s: the incoming object size it too large! %d %d",
				__func__, _IOC_SIZE(cmd), cmd);
			return -ENOTTY;
		}

		data = (union kargs *) kzalloc(sizeof(union kargs), GFP_KERNEL);
		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(data, up, _IOC_SIZE(cmd));
			if (err) {
				pr_err("%s: failed to copy from user space! %d",
					__func__, cmd);
				kfree(data);
				return -EFAULT;
			}
		}
	}
	switch (cmd) {
	case IPU_IOC_MAPBUF:
		pr_debug("%s: IPU_IOC_MAPBUF", __func__);
		// mapbuf combined with getbuf
		break;
	case IPU_IOC_UNMAPBUF:
		pr_debug("%s: IPU_IOC_UNMAPBUF", __func__);
		err = ipu_psys_unmapbuf(arg, fh);
		break;
	case IPU_IOC_QUERYCAP:
		pr_debug("%s: IPU_IOC_QUERYCAP", __func__);
		err = ipu_query_caps(&data->caps, fh);
		break;
	case IPU_IOC_GETBUF:
		pr_debug("%s: IPU_IOC_GETBUF", __func__);
		err = ipu_psys_getbuf(&data->buf, fh);
		break;
	case IPU_IOC_PUTBUF:
		pr_debug("%s: IPU_IOC_PUTBUF", __func__);
		//err = ipu_psys_putbuf(&karg.buf, fh);
		break;
	case IPU_IOC_QCMD:
		pr_debug("%s: IPU_IOC_QCMD", __func__);
		err = ipu_psys_kcmd_new(&data->cmd, fh);
		break;
	case IPU_IOC_DQEVENT:
		pr_debug("%s: IPU_IOC_DQEVENT", __func__);
		err = ipu_ioctl_dqevent(&data->ev, fh, file->f_flags);
		break;
	case IPU_IOC_GET_MANIFEST:
		pr_debug("%s: IPU_IOC_GET_MANIFEST", __func__);
		err = ipu_get_manifest(&data->m, fh);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if (!err && copy && _IOC_DIR(cmd) & _IOC_READ) {
		err = copy_to_user(up, data, _IOC_SIZE(cmd));
		kfree(data);
	}

	pr_debug("%s: return status %d", __func__, err);

	if (err)
		return err;

	return 0;
}

static int virt_psys_open(struct inode *inode, struct file *file)
{
	struct virt_ipu_psys *psys = inode_to_ipu_psys(inode);
	struct virt_ipu_psys_fh *fh;
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	int rval = 0;
	unsigned int op[1];

	pr_debug("virt psys open\n");

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh)
	  return -ENOMEM;
	mutex_init(&fh->bs_mutex);
	INIT_LIST_HEAD(&fh->bufmap);
	hash_init(FD_BUF_HASH);

	fh->psys = psys;
	file->private_data = fh;

	req = ipu4_virtio_fe_req_queue_get();
	if (!req) {
	   dev_err(&psys->dev, "Virtio Req buffer failed\n");
	   return -ENOMEM;
	}

	op[0] = file->f_flags;

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_OPEN, &op[0]);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
					  IPU_VIRTIO_QUEUE_1);
	if (rval) {
	   dev_err(&psys->dev, "Failed to PSYS open virtual device\n");
	   ipu4_virtio_fe_req_queue_put(req);
	   return rval;
	}
	ipu4_virtio_fe_req_queue_put(req);

	return rval;
}

static int virt_psys_release(struct inode *inode, struct file *file)
{
	struct virt_ipu_psys *psys = inode_to_ipu_psys(inode);
	struct ipu4_virtio_req *req;
	struct ipu4_virtio_ctx *fe_ctx = psys->ctx;
	struct ipu_psys_buffer_wrap *psys_buf_wrap;
	struct virt_ipu_psys_fh *fh = file->private_data;
	int rval = 0, bkt;

	pr_debug("%s: processing start", __func__);

	req = ipu4_virtio_fe_req_queue_get();
	if (!req) {
		dev_err(&psys->dev, "Virtio Req buffer failed\n");
		return -ENOMEM;
	}

	intel_ipu4_virtio_create_req(req, IPU4_CMD_PSYS_CLOSE, NULL);

	rval = fe_ctx->bknd_ops->send_req(fe_ctx->domid, req, true,
					  IPU_VIRTIO_QUEUE_1);
	if (rval) {
	   dev_err(&psys->dev, "Failed to PSYS close virtual device\n");
	   ipu4_virtio_fe_req_queue_put(req);
	   return rval;
	}
	ipu4_virtio_fe_req_queue_put(req);

	mutex_lock(&fh->mutex);
	/* clean up buffers */
	if(!hash_empty(FD_BUF_HASH)) {
		hash_for_each(FD_BUF_HASH, bkt, psys_buf_wrap, node) {
			psys_put_userpages(&psys_buf_wrap->map);
			hash_del(&psys_buf_wrap->node);
			kfree(psys_buf_wrap);
		}
	}
	mutex_unlock(&fh->mutex);

	kfree(file->private_data);

	return rval;
}

static const struct file_operations virt_psys_fops = {
	.open = virt_psys_open,
	.release = virt_psys_release,
	.unlocked_ioctl = virt_psys_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = virt_psys_compat_ioctl32,
#endif
	.poll = virt_psys_poll,
	.owner = THIS_MODULE,
};

static void virt_psys_dev_release(struct device *dev)
{
}
void virt_psys_exit(void)
{
	class_unregister(virt_psys_class);
	unregister_chrdev_region(virt_psys_dev_t, IPU_PSYS_NUM_DEVICES);
	if (g_psys)
		kfree(g_psys);


	device_unregister(&g_psys->dev);

	clear_bit(MINOR(g_psys->cdev.dev), virt_psys_devices);

	cdev_del(&g_psys->cdev);

	mutex_destroy(&g_psys->mutex);

	pr_notice("Virtual psys device unregistered\n");

}

int virt_psys_init(struct ipu4_virtio_ctx *fe_ctx)
{
	unsigned int minor;
	int rval = -E2BIG;

	if (!fe_ctx)
		return -ENOMEM;

	rval = alloc_chrdev_region(&virt_psys_dev_t, 0,
			IPU_PSYS_NUM_DEVICES, IPU_PSYS_NAME);
	if (rval) {
		pr_err("can't alloc psys chrdev region (%d)\n", rval);
		return rval;
	}
	mutex_lock(&psys_mutex);

	virt_psys_class = class_create(THIS_MODULE, IPU_PSYS_NAME);
	if (IS_ERR(virt_psys_class)) {
		unregister_chrdev_region(virt_psys_dev_t, IPU_PSYS_NUM_DEVICES);
		pr_err("Failed to register device class %s\n",	IPU_PSYS_NAME);
		return PTR_ERR(virt_psys_class);
	}

	minor = find_next_zero_bit(virt_psys_devices, IPU_PSYS_NUM_DEVICES, 0);
	if (minor == IPU_PSYS_NUM_DEVICES) {
		pr_err("too many devices\n");
		goto out_unlock;
	}

	g_psys = kzalloc(sizeof(*g_psys), GFP_KERNEL);
	if (!g_psys) {
		rval = -ENOMEM;
		goto out_unlock;
	}

	cdev_init(&g_psys->cdev, &virt_psys_fops);
	g_psys->cdev.owner = virt_psys_fops.owner;

	rval = cdev_add(&g_psys->cdev, MKDEV(MAJOR(virt_psys_dev_t), minor), 1);
	if (rval) {
		pr_err("cdev_add failed (%d)\n", rval);
		goto out_unlock;
	}

	set_bit(minor, virt_psys_devices);

	mutex_init(&g_psys->mutex);

	g_psys->dev.class = virt_psys_class;
	g_psys->dev.devt = MKDEV(MAJOR(virt_psys_dev_t), minor);
	g_psys->dev.release = virt_psys_dev_release;
	dev_set_name(&g_psys->dev, "ipu-psys%d", minor);
	rval = device_register(&g_psys->dev);
	if (rval < 0) {
		dev_err(&g_psys->dev, "psys device_register failed\n");
		goto out_mutex_destroy;
	}

	g_psys->ctx = fe_ctx;

	pr_info("psys probe minor: %d\n", minor);

	goto out_unlock;

out_mutex_destroy:
	mutex_destroy(&g_psys->mutex);
	cdev_del(&g_psys->cdev);
out_unlock:
	mutex_unlock (&psys_mutex);
	return rval;
}

