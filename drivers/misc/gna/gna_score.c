// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "gna_drv.h"
#include "gna_request.h"
#include "gna_score.h"

int gna_validate_score_config(struct gna_compute_cfg *compute_cfg,
	struct gna_file_private *file_priv)
{
	struct gna_private *gna_priv;
	size_t buffers_size;

	gna_priv = file_priv->gna_priv;

	if (compute_cfg->gna_mode > GNA_MODE_XNN) {
		dev_err(&gna_priv->dev, "gna mode invalid\n");
		return -EINVAL;
	}

	if (compute_cfg->layer_count > gna_priv->info.max_layer_count) {
		dev_err(&gna_priv->dev, "max layer count exceeded\n");
		return -EINVAL;
	}

	if (compute_cfg->buffer_count == 0) {
		dev_err(&gna_priv->dev, "no buffers\n");
		return -EINVAL;
	}

	buffers_size = sizeof(struct gna_buffer) * compute_cfg->buffer_count;
	if (!access_ok(compute_cfg->buffers_ptr, buffers_size)) {
		dev_err(&gna_priv->dev, "invalid buffers pointer\n");
		return -EINVAL;
	}

	return 0;
}

static int gna_buffer_fill_patches(
		struct gna_buffer *buffer,
		struct gna_private *gna_priv)
{
	struct gna_memory_patch *patches;
	u64 patches_size;
	u64 patch_count;

	patch_count = buffer->patch_count;
	if (!patch_count)
		return 0;

	patches_size = sizeof(struct gna_memory_patch) * patch_count;
	patches = kvmalloc_array(patch_count, patches_size, GFP_KERNEL);
	if (!patches)
		return -ENOMEM;

	if (copy_from_user(patches,
			   u64_to_user_ptr(buffer->patches_ptr),
			   patches_size)) {
		kvfree(patches);
		dev_err(&gna_priv->dev,
			"copy %llu patches from user failed\n", patch_count);
		return -EFAULT;
	}

	buffer->patches_ptr = (uintptr_t)patches;

	return 0;
}

static int gna_request_fill_buffers(
	struct gna_request *score_request,
	struct gna_compute_cfg *compute_cfg)
{
	struct gna_buffer *buffer_list;
	struct gna_memory_object *mo;
	struct gna_private *gna_priv;
	struct gna_buffer *buffer;
	u64 buffers_total_size;
	void __user *ptr;
	u64 buffer_count;
	u64 memory_id;
	u64 i, j;
	int ret;
	bool skip;

	gna_priv = score_request->gna_priv;

	buffers_total_size = 0;
	ret = 0;

	/* get memory buffer list */
	buffer_count = compute_cfg->buffer_count;
	buffer_list = kvmalloc_array(
			buffer_count, sizeof(struct gna_buffer), GFP_KERNEL);
	if (!buffer_list)
		return -ENOMEM;

	ptr = u64_to_user_ptr(compute_cfg->buffers_ptr);
	if (copy_from_user(buffer_list, ptr,
				sizeof(*buffer_list) * buffer_count)) {
		dev_err(&gna_priv->dev,
			"copying %llu buffers failed\n", buffer_count);
		ret = -EFAULT;
		goto err_free_buffers;
	}

	buffer = buffer_list;
	for (i = 0; i < buffer_count; i++, buffer++) {
		memory_id = buffer->memory_id;

		skip = false;
		for (j = 0; j < i; j++) {
			if (buffer_list[j].memory_id == memory_id) {
				skip = true;
				break;
			}
		}
		if (skip) {
			dev_warn_once(&gna_priv->dev,
				"multiple memory id in score config\n");
			continue;
		}

		buffers_total_size +=
			gna_buffer_get_size(buffer->offset, buffer->size);
		if (buffers_total_size > gna_priv->info.max_hw_mem) {
			dev_err(&gna_priv->dev, "buffers total size too big");
			ret =  -EINVAL;
			goto err_free_patches;
		}

		ret = gna_buffer_fill_patches(buffer, gna_priv);
		if (ret)
			goto err_free_patches;

		mutex_lock(&gna_priv->memidr_lock);
		mo = idr_find(&gna_priv->memory_idr, memory_id);
		if (!mo) {
			mutex_unlock(&gna_priv->memidr_lock);
			dev_err(&gna_priv->dev,
				"memory object %llu not found\n", memory_id);
			ret = -EINVAL;
			i++;
			goto err_free_patches;
		}
		mutex_unlock(&gna_priv->memidr_lock);
	}

	score_request->buffer_list = buffer_list;
	score_request->buffer_count = buffer_count;

	return 0;

err_free_patches:
	while (i--)
		if (buffer_list[i].patch_count)
			kvfree((void *)buffer_list[i].patches_ptr);

err_free_buffers:
	kvfree(buffer_list);
	return ret;
}

int gna_request_enqueue(struct gna_compute_cfg *compute_cfg,
	struct gna_file_private *file_priv, u64 *request_id)
{
	struct gna_request *score_request;
	struct gna_private *gna_priv;
	int ret;

	if (!file_priv)
		return -EINVAL;

	gna_priv = file_priv->gna_priv;

	score_request = gna_request_create(file_priv, compute_cfg);
	if (!score_request)
		return -ENOMEM;

	ret = gna_request_fill_buffers(score_request, compute_cfg);
	if (ret) {
		kref_put(&score_request->refcount, gna_request_release);
		return ret;
	}

	kref_get(&score_request->refcount);
	mutex_lock(&gna_priv->reqlist_lock);
	list_add_tail(&score_request->node, &gna_priv->request_list);
	mutex_unlock(&gna_priv->reqlist_lock);

	queue_work(gna_priv->request_wq, &score_request->work);
	kref_put(&score_request->refcount, gna_request_release);

	*request_id = score_request->request_id;

	return 0;
}

static struct gna_file_private *gna_find_file(struct gna_request *score_request)
{
	struct file *fd;
	struct list_head *list;
	struct gna_file_private *temp_file;
	struct gna_file_private *file;
	struct gna_private *gna_priv;

	fd = score_request->fd;
	gna_priv = score_request->gna_priv;
	list = &gna_priv->file_list;

	mutex_lock(&gna_priv->filelist_lock);
	if (!list_empty(list)) {
		list_for_each_entry_safe(file, temp_file, list, flist) {
			if (file->fd == fd) {
				mutex_unlock(&gna_priv->filelist_lock);
				return file;
			}
		}
	}
	mutex_unlock(&gna_priv->filelist_lock);
	return NULL;
}

static int gna_parse_hw_status(struct gna_private *gna_priv, u32 hw_status)
{
	int status;

	if (hw_status & GNA_ERROR) {
		dev_dbg(&gna_priv->dev,
			"GNA completed with errors: %#x\n", hw_status);
		status = -EIO;
	} else if (hw_status & GNA_STS_SCORE_COMPLETED) {
		status = 0;
		dev_dbg(&gna_priv->dev,
			"GNA completed successfully: %#x\n", hw_status);
	} else {
		dev_err(&gna_priv->dev,
			"GNA not completed, status: %#x\n", hw_status);
		status = -ETIME;
	}

#if defined(CONFIG_INTEL_GNA_DEBUG)
	if (hw_status & GNA_STS_PARAM_OOR)
		dev_dbg(&gna_priv->dev, "GNA error: Param Out Range Error\n");

	if (hw_status & GNA_STS_VA_OOR)
		dev_dbg(&gna_priv->dev, "GNA error: VA Out of Range Error\n");

	if (hw_status & GNA_STS_PCI_MMU_ERR)
		dev_dbg(&gna_priv->dev, "GNA error: PCI MMU Error\n");

	if (hw_status & GNA_STS_PCI_DMA_ERR)
		dev_dbg(&gna_priv->dev, "GNA error: PCI MMU Error\n");

	if (hw_status & GNA_STS_PCI_UNEXCOMPL_ERR)
		dev_dbg(&gna_priv->dev,
				"GNA error: PCI Unexpected Completion Error\n");

	if (hw_status & GNA_STS_SATURATE)
		dev_dbg(&gna_priv->dev, "GNA error: Saturation Reached !\n");
#endif

	return status;
}

void gna_request_tasklet(unsigned long data)
{
	struct gna_request *score_request;
	struct gna_private *gna_priv;
	unsigned long irq_flags;
	void __iomem *addr;
	u32 stall_cycles;
	u32 total_cycles;
	u32 hw_status;
	int isr_left;
	int ret;

	score_request = (struct gna_request *) data;
	gna_priv = score_request->gna_priv;
	dev_dbg(&gna_priv->dev, "%s: enter\n", __func__);

	del_timer(&gna_priv->isr_timer);

	spin_lock_bh(&score_request->perf_lock);
	score_request->drv_perf.hw_completed = ktime_get_ns();
	spin_unlock_bh(&score_request->perf_lock);

	/* get hw status written to device context by interrupt handler */
	spin_lock_irqsave(&gna_priv->hw_lock, irq_flags);
	hw_status = gna_priv->hw_status;
	spin_unlock_irqrestore(&gna_priv->hw_lock, irq_flags);

	/* save hw status in request context */
	spin_lock_irqsave(&score_request->hw_lock, irq_flags);
	score_request->hw_status = hw_status;
	spin_unlock_irqrestore(&score_request->hw_lock, irq_flags);

	spin_lock_bh(&score_request->status_lock);
	score_request->status = gna_parse_hw_status(gna_priv, hw_status);
	spin_unlock_bh(&score_request->status_lock);

	addr = gna_priv->bar0.mem_addr;
	if (hw_status & GNA_STS_STATISTICS_VALID) {
		dev_dbg(&gna_priv->dev,
			"GNA statistics calculated successfully\n");
		total_cycles = gna_reg_read(addr, GNAPTC);
		stall_cycles = gna_reg_read(addr, GNAPSC);

		spin_lock_bh(&score_request->perf_lock);
		score_request->hw_perf.total = total_cycles;
		score_request->hw_perf.stall = stall_cycles;
		dev_dbg(&gna_priv->dev,
			"GNAPTC %llu\n", score_request->hw_perf.total);
		dev_dbg(&gna_priv->dev,
			"GNAPSC %llu\n", score_request->hw_perf.stall);
		spin_unlock_bh(&score_request->perf_lock);
	} else
		dev_warn_once(&gna_priv->dev, "GNA statistics missing\n");

#if defined(CONFIG_INTEL_GNA_DEBUG)
	gna_debug_isi(gna_priv, addr);
#endif
	gna_abort_hw(gna_priv, addr);
	ret = pm_runtime_put(&gna_priv->pdev->dev);
	if (ret < 0)
		dev_warn_once(&gna_priv->dev,
			"pm_runtime_put() failed: %d\n", ret);

	spin_lock_bh(&gna_priv->busy_lock);
	gna_priv->busy = false;
	spin_unlock_bh(&gna_priv->busy_lock);
	wake_up(&gna_priv->busy_waitq);

	/* reschedule itself if another interrupt came in the meantime */
	/* unlikely: device queue synchronizes starting GNA device */
	isr_left = atomic_dec_return(&gna_priv->isr_count);
	if (isr_left) {
		dev_dbg(&gna_priv->dev, "scheduling another tasklet\n");
		tasklet_schedule(&gna_priv->request_tasklet);
	}

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);
}

void gna_isr_timeout(struct timer_list *timer)
{
	struct gna_private *gna_priv;
	unsigned long irq_flags;
	void __iomem *addr;
	u32 hw_status;

	gna_priv = from_timer(gna_priv, timer, isr_timer);
	dev_dbg(&gna_priv->dev, "%s enter\n", __func__);

	addr = gna_priv->bar0.mem_addr;
	hw_status = gna_reg_read(addr, GNASTS);
	spin_lock_irqsave(&gna_priv->hw_lock, irq_flags);
	gna_priv->hw_status = hw_status;
	spin_unlock_irqrestore(&gna_priv->hw_lock, irq_flags);

	atomic_inc(&gna_priv->isr_count);
	tasklet_schedule(&gna_priv->request_tasklet);

	dev_dbg(&gna_priv->dev, "%s exit\n", __func__);
}

static int gna_do_patch_memory(struct gna_private *gna_priv,
		struct gna_memory_object *mo, struct gna_memory_patch *patch)
{
	unsigned long addr;
	u8 __user *dest;
	size_t copied;
	size_t size;
	u64 value;

	value = patch->value;
	size = patch->size;
	dest = (u8 *)mo->vaddr + patch->offset;
	dev_dbg(&gna_priv->dev, "patch offset: %llu, size: %lu, value: %llu\n",
			patch->offset, size, value);

	switch (size) {
	case 0:
		return -EFAULT;
	case sizeof(u8):
		*((u8 *)dest) = (u8)value;
		break;
	case sizeof(u16):
		*((u16 *)dest) = (u16)value;
		break;
	case sizeof(u32):
		*((u32 *)dest) = (u32)value;
		break;
	case sizeof(u64):
		*((u64 *)dest) = (u64)value;
		break;
	default:
		addr = (unsigned long)patch->user_ptr;
		copied = access_process_vm(mo->task, addr,
				dest, patch->size, GFP_KERNEL);
		if (copied < patch->size)
			return -EFAULT;
	}

	return 0;
}

static int gna_mem_patch_memory(
	struct gna_file_private *file_priv,
	struct gna_buffer *buffer)
{
	struct gna_private *gna_priv;
	struct gna_memory_patch *patch;
	struct gna_memory_object *mo;
	void *vaddr;
	int ret;
	u32 i;

	ret = 0;

	gna_priv = file_priv->gna_priv;

	dev_dbg(&gna_priv->dev, "memory_id: %llu, patch_count, %llu\n",
			buffer->memory_id, buffer->patch_count);

	/* get kernel space memory pointer */
	mutex_lock(&gna_priv->memidr_lock);
	mo = idr_find(&gna_priv->memory_idr, buffer->memory_id);
	mutex_unlock(&gna_priv->memidr_lock);
	if (!mo)
		return -EINVAL;

	mutex_lock(&mo->page_lock);
	ret = mo->ops->get_pages(mo, buffer->offset, buffer->size);
	mutex_unlock(&mo->page_lock);
	if (ret)
		return ret;

	if (buffer->patch_count) {
		vaddr = vm_map_ram(mo->pages, mo->num_pinned, 0, PAGE_KERNEL);
		if (!vaddr)
			return -ENOMEM;

		mo->vaddr = vaddr;
		patch = (struct gna_memory_patch *)buffer->patches_ptr;
		for (i = 0; i < buffer->patch_count; i++, patch++) {
			ret = gna_do_patch_memory(gna_priv, mo, patch);
			if (ret)
				break;
		}

		kvfree((void *)buffer->patches_ptr);
		buffer->patches_ptr = 0;
		vm_unmap_ram(vaddr, mo->num_pages);
		mo->vaddr = NULL;

		if (ret)
			return ret;
	}

	gna_mmu_add(gna_priv, mo);

	return ret;
}

int gna_score_wait(struct gna_request *score_request, unsigned int timeout)
{
	struct timeval time_val;

	time_val.tv_sec = timeout / 1000;
	time_val.tv_usec = (timeout % 1000) * 1000;
	return wait_event_interruptible_timeout(score_request->waitq,
		score_request->state == DONE, timeval_to_jiffies(&time_val));
}

static struct gna_buffer *gna_find_buffer(
	struct gna_buffer *buffer_list, u32 buffer_count,
	u32 mmu_offset, u32 *memory_offset)
{
	struct gna_buffer *buffer;
	u32 offset;
	u32 page_offset;
	u32 memory_size;
	u32 i;

	offset = 0;
	for (i = 0; i < buffer_count; i++) {
		buffer = buffer_list + i;
		page_offset = buffer->offset & ~PAGE_MASK;
		memory_size = ROUND_UP(page_offset + buffer->size, PAGE_SIZE);
		if (mmu_offset < offset + memory_size) {
			*memory_offset = offset;
			return buffer;
		}
		offset += memory_size;
	}

	return NULL;
}

static int gna_copy_gmm_config(
	struct gna_file_private *file_priv,
	struct gna_buffer *buffer_list, u32 buffer_count, u32 mmu_offset)
{
	struct gna_hw_descriptor *hwdesc;
	struct gna_private *gna_priv;
	struct gna_memory_object *mo;
	struct gna_mmu_object *mmu;
	struct gna_buffer *buffer;
	u8 *gmm_desc;
	void *vaddr;
	u32 memory_offset;
	u32 skip_offset;

	gna_priv = file_priv->gna_priv;
	mmu = &gna_priv->mmu;
	hwdesc = mmu->hwdesc;

	buffer = gna_find_buffer(buffer_list, buffer_count,
			mmu_offset, &memory_offset);
	if (!buffer) {
		dev_dbg(&gna_priv->dev, "buffer not found\n");
		return -EINVAL;
	}

	mutex_lock(&gna_priv->memidr_lock);
	mo = idr_find(&gna_priv->memory_idr, buffer->memory_id);
	mutex_unlock(&gna_priv->memidr_lock);
	if (!mo) {
		dev_dbg(&gna_priv->dev, "memory object not found\n");
		return -EFAULT;
	}

	vaddr = vm_map_ram(mo->pages, mo->num_pinned, 0, PAGE_KERNEL);
	if (!vaddr) {
		dev_dbg(&gna_priv->dev, "mappping failed\n");
		return -EFAULT;
	}

	skip_offset = ROUND_DOWN(buffer->offset, PAGE_SIZE);
	gmm_desc = (u8 *)vaddr + skip_offset + (mmu_offset - memory_offset);
	memcpy(&hwdesc->xnn_config, gmm_desc, GMM_CFG_SIZE);
	vm_unmap_ram(vaddr, mo->num_pages);

	return 0;
}

int gna_priv_score(struct gna_request *score_request)
{
	struct gna_xnn_descriptor *xnn_config;
	struct gna_file_private *file_priv;
	struct gna_compute_cfg *compute_cfg;
	struct gna_private *gna_priv;
	struct gna_memory_object *mo;
	struct gna_mmu_object *mmu;
	struct gna_buffer *buffer;
	void __iomem *addr;
	u64 buffer_count;
	u32 desc_base;
	u64 i;
	int ret;
	bool mo_valid = true;

	ret = 0;

	gna_priv = score_request->gna_priv;

	file_priv = gna_find_file(score_request);
	if (!file_priv)
		goto err_no_gna_file;

	mmu = &gna_priv->mmu;
	xnn_config = &mmu->hwdesc->xnn_config;
	compute_cfg = &score_request->compute_cfg;

	dev_dbg(&gna_priv->dev, "updating descriptors in user memory\n");

	buffer = score_request->buffer_list;
	buffer_count = score_request->buffer_count;
	dev_dbg(&gna_priv->dev, "buffer count: %llu\n", buffer_count);
	for (i = 0; i < buffer_count; i++, buffer++) {
		dev_dbg(&gna_priv->dev, "patch count: %llu\n",
						buffer->patch_count);
		ret = gna_mem_patch_memory(file_priv, buffer);
		if (ret)
			goto err_put_pages;
	}

	switch (compute_cfg->gna_mode) {
	case GNA_MODE_XNN:
		dev_dbg(&gna_priv->dev,
			"xNN mode, labase: %d, lacount: %d\n",
			compute_cfg->layer_base, compute_cfg->layer_count);
		xnn_config->labase = compute_cfg->layer_base;
		xnn_config->lacount = compute_cfg->layer_count;
		break;
	case GNA_MODE_GMM:
		dev_dbg(&gna_priv->dev, "GMM mode, offset: %d\n",
				compute_cfg->layer_base);
		ret = gna_copy_gmm_config(file_priv,
				score_request->buffer_list,
				buffer_count, compute_cfg->layer_base);
		if (ret)
			goto err_put_pages_decr;
		break;
	default:
		goto err_put_pages_decr;
	}

	addr = gna_priv->bar0.mem_addr;
	desc_base = (u32)(mmu->hwdesc_dma >> PAGE_SHIFT);
	gna_reg_write(addr, GNADESBASE, desc_base);

	gna_priv->request_tasklet.data = (unsigned long)score_request;
	gna_start_scoring(gna_priv, addr, compute_cfg);

	return 0;

err_put_pages_decr:
	i--;
	buffer--;
err_put_pages:
	do {
		mutex_lock(&gna_priv->memidr_lock);
		mo = idr_find(&gna_priv->memory_idr, buffer->memory_id);
		mutex_unlock(&gna_priv->memidr_lock);
		if (mo) {
			mutex_lock(&mo->page_lock);
			mo->ops->put_pages(mo);
			mutex_unlock(&mo->page_lock);
		} else {
			mo_valid = false;
			dev_warn(&gna_priv->dev, "memory object not found %llu\n",
				 buffer->memory_id);
		}
		buffer--;
	} while (i--);

err_no_gna_file:
	if (mo_valid) {
		i = score_request->buffer_count;
		while (i--)
			kvfree((void *)score_request->buffer_list[i].patches_ptr);
		kvfree(score_request->buffer_list);
	}
	score_request->buffer_list = NULL;
	score_request->buffer_count = 0;

	spin_lock_bh(&gna_priv->busy_lock);
	gna_priv->busy = false;
	spin_unlock_bh(&gna_priv->busy_lock);
	gna_request_set_done(score_request, ret);

	wake_up(&gna_priv->busy_waitq);
	wake_up_interruptible_all(&score_request->waitq);

	return ret;
}
