// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */

#include <linux/anon_inodes.h>

#include "gt/intel_gt.h"
#include "gt/intel_gt_mcr.h"
#include "gt/intel_gt_regs.h"
#include "gem/i915_gem_region.h"
#include "gem/i915_gem_lmem.h"
#include "i915_drv.h"
#include "i915_trace.h"
#include "i915_perf_stall_cntr.h"
#include "gt/intel_engine_user.h"

#define DEFAULT_POLL_FREQUENCY_HZ 100
#define DEFAULT_POLL_PERIOD_NS (NSEC_PER_SEC / DEFAULT_POLL_FREQUENCY_HZ)

/**
 * struct eu_stall_open_properties - properties given to open a stream
 *
 * @eu_stall_buf_sz: EU stall counters' data buffer size
 * @eu_stall_sample_rate: EU stall counters sampling rate
 * @poll_period: The period in nanoseconds at which the CPU will check for
 *		 EU stall data in the buffer.
 */
struct eu_stall_open_properties {
	struct intel_gt *tile_gt;
	u8 eu_stall_sample_rate;
	u32 event_report_count;
	u32 eu_stall_buf_sz;
	u64 poll_period;
};

/**
 * num_data_rows - Return the number of EU stall data rows of 64B each
 * 		   for a given data size.
 *
 * @data_size: EU stall data size
 */
static inline u32
num_data_rows(u32 data_size)
{
	return (data_size >> 6);
}

void i915_perf_stall_cntr_init(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	u8 i;

	for_each_gt(gt, i915, i)
		mutex_init(&gt->eu_stall_cntr.lock);
}

/*
 * set_mcr_multicast - set the multicast bit in the MCR packet control
 * 		       selector register.
 *
 * This shouldn't be necessary as the multicast bit is supposed to be set by
 * default and cleared only during a unicast write. But it has been observed
 * that sometimes the multicast bit is not set leading to unexpected and
 * erroneous results.
 */
static void set_mcr_multicast(struct intel_uncore *uncore)
{
	enum forcewake_domains fw_domain;
	u32 mcr;

	fw_domain = intel_uncore_forcewake_for_reg(uncore, GEN8_MCR_SELECTOR,
						   FW_REG_READ | FW_REG_WRITE);
	spin_lock_irq(&uncore->lock);
	intel_uncore_forcewake_get__locked(uncore, fw_domain);
	mcr = intel_uncore_read_fw(uncore, GEN8_MCR_SELECTOR);
	if (!(mcr & GEN11_MCR_MULTICAST)) {
		drm_dbg(&uncore->i915->drm,
			"Setting multicast bit in the MCR packet ctrl selector register.\n");
		intel_uncore_write_fw(uncore, GEN8_MCR_SELECTOR,
				      (mcr | GEN11_MCR_MULTICAST));
	}
	intel_uncore_forcewake_put__locked(uncore, fw_domain);
	spin_unlock_irq(&uncore->lock);
}

/**
 * read_eu_stall_properties - validate + copy userspace
 *			      stream open properties
 * @perf: i915 perf instance
 * @uprops: The array of u64 key value pairs given by userspace
 * @n_props: The number of key value pairs expected in @uprops
 * @props: The stream configuration built up while validating properties
 */
static int read_eu_stall_properties(struct drm_i915_private *i915,
				    u64 __user *uprops,
				    u32 n_props,
				    struct eu_stall_open_properties *props)
{
	struct intel_engine_cs *engine;
	u64 __user *uprop = uprops;
	int ret;
	u32 i;

	struct i915_engine_class_instance ci = {
		.engine_class = I915_ENGINE_CLASS_INVALID,
		.engine_instance = I915_ENGINE_CLASS_INVALID_NONE,
	};

	memset(props, 0, sizeof(struct eu_stall_open_properties));

	/* Set default values */
	props->eu_stall_buf_sz = SZ_128K;
	props->eu_stall_sample_rate = 4;
	props->poll_period = DEFAULT_POLL_PERIOD_NS;
	props->event_report_count = 1;

	if (!n_props || n_props >= PRELIM_DRM_I915_EU_STALL_PROP_MAX) {
		DRM_DEBUG("Invalid i915 perf EU stall properties\n");
		return -EINVAL;
	}

	for (i = 0; i < n_props; i++) {
		u64 id, value;

		ret = get_user(id, uprop);
		if (ret)
			return ret;

		ret = get_user(value, uprop + 1);
		if (ret)
			return ret;

		if (id == 0 || id >= PRELIM_DRM_I915_EU_STALL_PROP_MAX) {
			DRM_DEBUG("Unknown i915 perf property ID\n");
			return -EINVAL;
		}

		switch ((enum prelim_drm_i915_eu_stall_property_id)id) {
		case PRELIM_DRM_I915_EU_STALL_PROP_BUF_SZ:
			if (value != SZ_128K &&
			    value != SZ_256K &&
			    value != SZ_512K) {
				DRM_DEBUG("Invalid EU stall buf size %llu\n", value);
				return -EINVAL;
			}
			props->eu_stall_buf_sz = value;
			break;
		case PRELIM_DRM_I915_EU_STALL_PROP_SAMPLE_RATE:
			if (value == 0 || value > 7) {
				DRM_DEBUG("Invalid EU stall sample rate %llu\n", value);
				return -EINVAL;
			}
			props->eu_stall_sample_rate = value;
			break;
		case PRELIM_DRM_I915_EU_STALL_PROP_POLL_PERIOD:
			if (value < 100000 /* 100us */) {
				DRM_DEBUG("Stall data poll period %lluns less than 100us\n", value);
				return -EINVAL;
			}
			props->poll_period = value;
			break;
		case PRELIM_DRM_I915_EU_STALL_PROP_EVENT_REPORT_COUNT:
			if (value == 0) {
				DRM_DEBUG("Invalid EU stall poll event report count %llu\n", value);
				return -EINVAL;
			}
			props->event_report_count = (u32)value;
			break;
		case PRELIM_DRM_I915_EU_STALL_PROP_ENGINE_CLASS:
			ci.engine_class = (u16)value;
			break;
		case PRELIM_DRM_I915_EU_STALL_PROP_ENGINE_INSTANCE:
			ci.engine_instance = (u16)value;
			break;
		case PRELIM_DRM_I915_EU_STALL_PROP_MAX:
			MISSING_CASE(id);
			return -EINVAL;
		}

		uprop += 2;
	}

	/*
	 * If the user didn't pass engine class and instance,
	 * use tile 0 as the default tile to sample EU stalls.
	 */
	if (ci.engine_class == (u16)I915_ENGINE_CLASS_INVALID &&
	    ci.engine_instance == (u16)I915_ENGINE_CLASS_INVALID_NONE) {
		props->tile_gt = i915->gt[0];
		return 0;
	}

	engine = intel_engine_lookup_user(i915, ci.engine_class, ci.engine_instance);
	if (!engine) {
		DRM_DEBUG("Invalid engine class and instance %u:%u\n",
			  ci.engine_class,
			  ci.engine_instance);
		return -EINVAL;
	}

	props->tile_gt = engine->gt;
	return 0;
}

/**
 * buf_data_size - Calculate the number of bytes in a circular buffer
 *		   of size buf_size given the read and write pointers
 *		   into the buffer.
 *
 * @read_ptr: Read pointer. Uses an additional overflow bit
 * @write_ptr: Write pointer. Uses an additional overflow bit
 *
 * Returns: number of bytes of data in the buffer
 */
static u32
buf_data_size(size_t buf_size, u32 read_ptr, u32 write_ptr)
{
	u32 read_offset, write_offset, size = 0;

	read_offset = read_ptr & (buf_size - 1);
	write_offset = write_ptr & (buf_size - 1);

	if (write_offset > read_offset)
		size = write_offset - read_offset;
	else
		size = buf_size - read_offset + write_offset;

	return size;
}

/**
 * eu_stall_cntr_buf_check - check for data in the EU stall counter buffer
 *
 * @stream: i915 EU stall counters stream instance
 *
 * Returns: true if the EU stall buffer contains minimum stall data as
 *	    specified by the event report count, else false.
 */
static bool
eu_stall_cntr_buf_check(struct i915_eu_stall_cntr_stream *stream)
{
	u32 read_ptr_reg, read_ptr, write_ptr_reg, write_ptr, total_data = 0;
	u32 buf_size = stream->per_dss_buf_size;
	struct intel_gt *gt = stream->tile_gt;
	struct per_dss_buf *dss_buf;
	bool min_data_present;
	int dss, group, instance;

	min_data_present = false;
	for_each_ss_steering(dss, gt, group, instance) {
		dss_buf = &stream->dss_buf[dss];
		mutex_lock(&dss_buf->lock);
		read_ptr = dss_buf->read;
		write_ptr_reg = intel_gt_mcr_read(gt, XEHPC_EUSTALL_REPORT,
						  group, instance);
		write_ptr = write_ptr_reg & XEHPC_EUSTALL_REPORT_WRITE_PTR_MASK;
		write_ptr <<= (6 - XEHPC_EUSTALL_REPORT_WRITE_PTR_SHIFT);
		write_ptr &= ((buf_size << 1) - 1);
		/*
		 * If there has been an engine reset by GuC, and GuC doesn't restore
		 * the read and write pointer registers, the pointers will reset to 0.
		 * If so, update the cached read pointer.
		 */
		if (unlikely((write_ptr < read_ptr) &&
			     ((read_ptr & buf_size) == (write_ptr & buf_size)))) {
			read_ptr_reg = intel_gt_mcr_read(gt, XEHPC_EUSTALL_REPORT1,
							 group, instance);
			read_ptr = read_ptr_reg & XEHPC_EUSTALL_REPORT1_READ_PTR_MASK;
			read_ptr <<= (6 - XEHPC_EUSTALL_REPORT1_READ_PTR_SHIFT);
			read_ptr &= ((buf_size << 1) - 1);
			dss_buf->read = read_ptr;
		}
		if ((write_ptr != read_ptr) && !min_data_present) {
			total_data += buf_data_size(buf_size, read_ptr, write_ptr);
			/*
			 * Check if there are at least minimum number of stall data
			 * rows for poll() to indicate that the data is present.
			 * Each stall data row is 64B (cacheline size).
			 */
			if (num_data_rows(total_data) >= stream->event_report_count)
				min_data_present = true;
		}
		if (write_ptr_reg & XEHPC_EUSTALL_REPORT_OVERFLOW_DROP)
			dss_buf->line_drop = true;
		dss_buf->write = write_ptr;
		mutex_unlock(&dss_buf->lock);
	}
	return min_data_present;
}

static void
clear_dropped_eviction_line_bit(struct intel_gt *gt, u8 s, u8 ss)
{
	u32 write_ptr_reg;

	/* Clear the overflow bit by setting it to 1 */
	write_ptr_reg = _MASKED_BIT_ENABLE(XEHPC_EUSTALL_REPORT_OVERFLOW_DROP);
	intel_gt_mcr_unicast_write(gt, XEHPC_EUSTALL_REPORT, write_ptr_reg, s, ss);
	trace_i915_reg_rw(true, _MMIO(XEHPC_EUSTALL_REPORT.reg),
			  write_ptr_reg, sizeof(write_ptr_reg), true);
}

static int
__i915_eu_stall_buf_read(struct i915_eu_stall_cntr_stream *stream,
			 char __user *buf, size_t count,
			 size_t *total_size, struct intel_gt *gt,
			 u8 s, u8 ss)
{
	size_t size, tmp_size, buf_size = stream->per_dss_buf_size;
	u16 flags = 0, subslice = (s * GEN_DSS_PER_CSLICE) + ss;
	struct prelim_drm_i915_stall_cntr_info info;
	u8 *dss_start_vaddr, *read_vaddr, *tmp_addr;
	u32 read_ptr_reg, read_ptr, write_ptr;
	u32 read_offset, write_offset;
	struct per_dss_buf *dss_buf;
	bool line_drop = false;
	int ret = 0;

	/* Hardware increments the read and write pointers such that they can
	 * overflow into one additional bit. For example, a 256KB size buffer
	 * offset pointer needs 18 bits. But HW uses 19 bits for the read and
	 * write pointers. This technique avoids wasting a slot in the buffer.
	 * Read and write offsets are calculated from the pointers in order to
	 * check if the write pointer has wrapped around the array.
	 */
	dss_buf = &stream->dss_buf[subslice];
	mutex_lock(&dss_buf->lock);
	dss_start_vaddr = dss_buf->vaddr;
	read_ptr = dss_buf->read;
	write_ptr = dss_buf->write;
	line_drop = dss_buf->line_drop;
	read_offset = read_ptr & (buf_size - 1);
	write_offset = write_ptr & (buf_size - 1);
	/*
	 * If there has been an engine reset by GuC, and GuC doesn't restore
	 * the read and write pointer registers, the pointers will reset to 0.
	 * If so, update the cached read pointer.
	 */
	if (unlikely((write_ptr < read_ptr) &&
		     ((read_ptr & buf_size) == (write_ptr & buf_size)))) {
		read_ptr_reg = intel_gt_mcr_read(gt, XEHPC_EUSTALL_REPORT1,
						s, ss);
		read_ptr = read_ptr_reg & XEHPC_EUSTALL_REPORT1_READ_PTR_MASK;
		read_ptr <<= (6 - XEHPC_EUSTALL_REPORT1_READ_PTR_SHIFT);
		read_ptr &= ((buf_size << 1) - 1);
		read_offset = read_ptr & (buf_size - 1);
		dss_buf->read = read_ptr;
	}

	trace_i915_eu_stall_cntr_read(s, ss, read_ptr, write_ptr,
				      read_offset, write_offset, *total_size);
	if (write_ptr == read_ptr) {
		mutex_unlock(&dss_buf->lock);
		return 0;
	}

	/* If write pointer offset is less than the read pointer offset,
	 * it means, write pointer has wrapped around the array.
	 */
	if (write_offset > read_offset)
		size = write_offset - read_offset;
	else
		size = buf_size - read_offset + write_offset;

	/* Read only the data that the user space buffer can accommodate */
	if ((*total_size + size) > count)
		size = count - *total_size;

	if (size == 0) {
		mutex_unlock(&dss_buf->lock);
		return 0;
	}

	if (line_drop)
		flags = PRELIM_I915_EUSTALL_FLAG_OVERFLOW_DROP;

	/* Driver doesn't expose the number of C-slices to user space.
	 * A PVC configuration of 8 c-slices x 8 sub-slices will be
	 * exposed to the user space as 1 slice x 64 sub-slices.
	 */
	info.subslice = subslice;
	info.flags = flags;

	read_vaddr = dss_start_vaddr + read_offset;

	if (write_offset > read_offset) {
		/* Each entry is cacheline sized, out of which only first 16B
		 * have data. All upper bits are zeros. Copy info struct into
		 * bytes 49 through 52.
		 */
		for (tmp_addr = read_vaddr; tmp_addr < read_vaddr + size;
						tmp_addr += CACHELINE_BYTES)
			memcpy((tmp_addr + 48), &info, sizeof(info));
		if (copy_to_user((buf + *total_size), read_vaddr, size)) {
			mutex_unlock(&dss_buf->lock);
			return -EFAULT;
		}
		*total_size += size;
		read_ptr += size;
	} else {
		tmp_size = buf_size - read_offset;
		if (tmp_size > size)
			tmp_size = size;
		for (tmp_addr = read_vaddr; tmp_addr < read_vaddr + tmp_size;
						tmp_addr += CACHELINE_BYTES)
			memcpy((tmp_addr + 48), &info, sizeof(info));
		if (copy_to_user((buf + *total_size), read_vaddr, tmp_size)) {
			mutex_unlock(&dss_buf->lock);
			return -EFAULT;
		}
		*total_size += tmp_size;
		read_ptr += tmp_size;

		if (size > tmp_size) {
			size = size - tmp_size;
			for (tmp_addr = dss_start_vaddr;
					tmp_addr < dss_start_vaddr + size;
					tmp_addr += CACHELINE_BYTES)
				memcpy((tmp_addr + 48), &info, sizeof(info));
			if (copy_to_user((buf + *total_size), dss_start_vaddr, size)) {
				mutex_unlock(&dss_buf->lock);
				return -EFAULT;
			}
			*total_size += size;
			read_ptr += size;
		}
		/* Read pointer can overflow into one additional bit */
		read_ptr &= ((buf_size << 1) - 1);
	}

	read_ptr_reg = ((read_ptr >> 6) << XEHPC_EUSTALL_REPORT1_READ_PTR_SHIFT);
	read_ptr_reg &= XEHPC_EUSTALL_REPORT1_READ_PTR_MASK;
	read_ptr_reg |= (XEHPC_EUSTALL_REPORT1_READ_PTR_MASK <<
			 XEHPC_EUSTALL_REPORT1_MASK_SHIFT);
	intel_gt_mcr_unicast_write(gt, XEHPC_EUSTALL_REPORT1, read_ptr_reg, s, ss);
	trace_i915_reg_rw(true, _MMIO(XEHPC_EUSTALL_REPORT1.reg),
			  read_ptr_reg, sizeof(read_ptr_reg), true);
	if (dss_buf->line_drop) {
		clear_dropped_eviction_line_bit(gt, s, ss);
		dss_buf->line_drop = false;
	}
	dss_buf->read = read_ptr;
	mutex_unlock(&dss_buf->lock);
	trace_i915_eu_stall_cntr_read(s, ss, read_ptr, write_ptr,
				      read_offset, write_offset, *total_size);
	return ret;
}

/**
 * i915_eu_stall_buf_read_locked - copy EU stall counters data from the
 *				   per dss buffers to the userspace buffer
 * @stream: A stream opened for EU stall count metrics
 * @buf: destination buffer given by userspace
 * @count: the number of bytes userspace wants to read
 * @ppos: (inout) file seek position (unused)
 *
 * Returns: Number of bytes copied or a negative error code
 * If we've successfully copied any data then reporting that takes
 * precedence over any internal error status, so the data isn't lost.
 */
static ssize_t
i915_eu_stall_buf_read_locked(struct i915_eu_stall_cntr_stream *stream,
			      struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct intel_gt *gt = stream->tile_gt;
	size_t total_size = 0;
	int ret = 0, dss, group, instance;

	if (count == 0)
		return -EINVAL;

	for_each_ss_steering(dss, gt, group, instance) {
		ret = __i915_eu_stall_buf_read(stream, buf, count, &total_size,
					       gt, group, instance);
		if (ret || count == total_size)
			goto exit;
	}
exit:
	if (total_size)
		return total_size;
	else if (ret)
		return ret;
	else
		return -EAGAIN;
}

static void
free_eu_stall_cntr_buf(struct i915_eu_stall_cntr_stream *stream)
{
	if (stream->vma) {
		i915_vma_unpin_and_release(&stream->vma,
					   I915_VMA_RELEASE_MAP);
		stream->vaddr = NULL;
		stream->vma = NULL;
	}
	destroy_workqueue(stream->buf_check_wq);
}

static int alloc_eu_stall_cntr_buf(struct i915_eu_stall_cntr_stream *stream,
				   u32 per_dss_buf_size)
{
	struct intel_gt *gt = stream->tile_gt;
	struct drm_i915_gem_object *bo;
	struct sseu_dev_info *sseu;
	struct i915_vma *vma;
	u8 *vaddr;
	u32 size;
	int ret;

	/*
	 * Enabled subslices can be discontiguous. Find the last subslice
	 * and calculate total buffer size based on that.
	 * intel_sseu_highest_xehp_dss returns zero based position.
	 * Therefore the result is incremented.
	 */
	sseu = &gt->info.sseu;
	size = per_dss_buf_size *
			(intel_sseu_highest_xehp_dss(sseu->subslice_mask) + 1);

	bo = intel_gt_object_create_lmem(gt, size, 0);
	if (IS_ERR(bo))
		bo = i915_gem_object_create_shmem(gt->i915, size);
	if (IS_ERR(bo)) {
		ret = PTR_ERR(bo);
		DRM_ERROR("Failed to allocate EU stall counter buf : %d\n", ret);
		goto err;
	}

	vma = i915_gem_object_ggtt_pin(bo, gt->ggtt, NULL, 0, SZ_64, 0);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		DRM_ERROR("EU stall i915 GEM object GGTT pin error: %d\n", ret);
		goto err_obj_put;
	}
	vaddr = i915_gem_object_pin_map_unlocked(bo, I915_MAP_WC);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		DRM_ERROR("EU stall gem object pin map error: %d\n", ret);
		goto err_vma_unpin;
	}
	stream->vma = vma;
	stream->vaddr = vaddr;
	return 0;

err_vma_unpin:
	i915_vma_unpin(vma);
err_obj_put:
	i915_gem_object_put(bo);
err:
	free_eu_stall_cntr_buf(stream);
	return ret;
}

static u32
gen_eustall_base(struct i915_eu_stall_cntr_stream *stream, bool enable)
{
	u32 val = i915_ggtt_offset(stream->vma);
	u32 sz;

	drm_WARN_ON(&stream->tile_gt->i915->drm, !IS_ALIGNED(val, 64));

	switch (stream->per_dss_buf_size) {
	case SZ_128K:
		sz = 0;
		break;
	case SZ_256K:
		sz = 1;
		break;
	case SZ_512K:
		sz = 2;
		break;
	default:
		MISSING_CASE(stream->per_dss_buf_size);
		sz = 2;
	}

	val |= REG_FIELD_PREP(XEHPC_EUSTALL_BASE_DSS_BUF_SZ, sz);
	if (enable)
		val |= XEHPC_EUSTALL_BASE_ENABLE_SAMPLING;

	return val;
}

static void
i915_eu_stall_stream_enable(struct i915_eu_stall_cntr_stream *stream)
{
	struct intel_gt *gt = stream->tile_gt;
	intel_wakeref_t wakeref;

	with_intel_runtime_pm(gt->uncore->rpm, wakeref) {
		/* Wa_22012878696:pvc */
		if (IS_PONTEVECCHIO(gt->i915))
			intel_uncore_forcewake_get(gt->uncore, FORCEWAKE_RENDER);

		/*
		 * Wa_22016596838:pvc
		 * GPU may hang if EU DOP gating is enabled during stall sampling.
		 * Disable EU DOP gating.
		 */
		if (IS_PONTEVECCHIO(gt->i915))
			intel_gt_mcr_multicast_write(gt, GEN8_ROW_CHICKEN2,
						     _MASKED_BIT_ENABLE(GEN12_DISABLE_DOP_GATING));

		set_mcr_multicast(gt->uncore);
		intel_gt_mcr_multicast_write(gt, XEHPC_EUSTALL_BASE,
					     gen_eustall_base(stream, true));
	}
}

static void
i915_eu_stall_stream_disable(struct i915_eu_stall_cntr_stream *stream)
{
	struct intel_gt *gt = stream->tile_gt;
	intel_wakeref_t wakeref;
	u32 reg_value;
	int dss, group, instance;

	with_intel_runtime_pm(gt->uncore->rpm, wakeref) {
		/*
		 * Before disabling EU stall sampling, check if any of the
		 * XEHPC_EUSTALL_REPORT registers have the drop bit set. If set,
		 * clear the bit. If the user space application reads all the
		 * stall data, the drop bit would be cleared during the read.
		 * But if there is any unread data and the drop bit is set for
		 * any subslice, the drop bit would continue to be set even
		 * after disabling EU stall sampling and may cause erroneous
		 * stall data in the subsequent stall data sampling run.
		 */
		for_each_ss_steering(dss, gt, group, instance) {
			reg_value = intel_gt_mcr_read(gt, XEHPC_EUSTALL_REPORT,
						      group, instance);
			if (reg_value & XEHPC_EUSTALL_REPORT_OVERFLOW_DROP)
				clear_dropped_eviction_line_bit(gt, group, instance);
		}
		set_mcr_multicast(gt->uncore);
		intel_gt_mcr_multicast_write(gt, XEHPC_EUSTALL_BASE,
					     gen_eustall_base(stream, false));

		/* Wa_22016596838:pvc */
		if (IS_PONTEVECCHIO(gt->i915))
			intel_gt_mcr_multicast_write(gt, GEN8_ROW_CHICKEN2,
						     _MASKED_BIT_DISABLE(GEN12_DISABLE_DOP_GATING));

		/* Wa_22012878696:pvc */
		if (IS_PONTEVECCHIO(gt->i915))
			intel_uncore_forcewake_put(gt->uncore, FORCEWAKE_RENDER);
	}
}

static void eu_stall_buf_check_work_fn(struct work_struct *work)
{
	struct i915_eu_stall_cntr_stream *stream =
		container_of(work, typeof(*stream), buf_check_work);

	if (eu_stall_cntr_buf_check(stream)) {
		stream->pollin = true;
		wake_up(&stream->poll_wq);
	}
}

static enum
hrtimer_restart eu_stall_poll_check_timer_cb(struct hrtimer *hrtimer)
{
	struct i915_eu_stall_cntr_stream *stream =
		container_of(hrtimer, typeof(*stream), poll_check_timer);

	queue_work(stream->buf_check_wq, &stream->buf_check_work);
	hrtimer_forward_now(hrtimer, ns_to_ktime(stream->poll_period));

	return HRTIMER_RESTART;
}

static int i915_eu_stall_stream_init(struct i915_eu_stall_cntr_stream *stream,
				     struct drm_i915_perf_open_param *param,
				     struct eu_stall_open_properties *props)
{
	u32 write_ptr_reg, write_ptr, read_ptr_reg;
	u32 vaddr_offset;
	struct intel_gt *gt = stream->tile_gt;
	struct per_dss_buf *dss_buf;
	intel_wakeref_t wakeref;
	int ret, dss, group, instance;

	init_waitqueue_head(&stream->poll_wq);
	INIT_WORK(&stream->buf_check_work, eu_stall_buf_check_work_fn);
	stream->buf_check_wq = alloc_ordered_workqueue("i915_eustall_cntr", 0);
	if (!stream->buf_check_wq)
		return -ENOMEM;
	hrtimer_init(&stream->poll_check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stream->poll_check_timer.function = eu_stall_poll_check_timer_cb;
	stream->event_report_count = props->event_report_count;
	stream->per_dss_buf_size = props->eu_stall_buf_sz;
	stream->poll_period = props->poll_period;

	ret = alloc_eu_stall_cntr_buf(stream, props->eu_stall_buf_sz);
	if (ret)
		return ret;

	with_intel_runtime_pm(gt->uncore->rpm, wakeref) {
		set_mcr_multicast(gt->uncore);
		intel_gt_mcr_multicast_write(gt, XEHPC_EUSTALL_BASE,
					     gen_eustall_base(stream, false));
		/* GGTT addresses can never be > 32 bits */
		intel_gt_mcr_multicast_write(gt, XEHPC_EUSTALL_BASE_UPPER, 0);
		intel_gt_mcr_multicast_write(gt, XEHPC_EUSTALL_CTRL,
					     _MASKED_FIELD(EUSTALL_MOCS | EUSTALL_SAMPLE_RATE,
							   REG_FIELD_PREP(EUSTALL_MOCS, gt->mocs.uc_index << 1) |
							   REG_FIELD_PREP(EUSTALL_SAMPLE_RATE, props->eu_stall_sample_rate)));

		for_each_ss_steering(dss, gt, group, instance) {
			write_ptr_reg = intel_gt_mcr_read(gt, XEHPC_EUSTALL_REPORT,
							  group, instance);
			write_ptr = write_ptr_reg & XEHPC_EUSTALL_REPORT_WRITE_PTR_MASK;
			write_ptr <<= (6 - XEHPC_EUSTALL_REPORT_WRITE_PTR_SHIFT);
			write_ptr &= ((stream->per_dss_buf_size << 1) - 1);
			read_ptr_reg = write_ptr >> (6 - XEHPC_EUSTALL_REPORT1_READ_PTR_SHIFT);
			read_ptr_reg &= XEHPC_EUSTALL_REPORT1_READ_PTR_MASK;
			read_ptr_reg |= (XEHPC_EUSTALL_REPORT1_READ_PTR_MASK <<
					 XEHPC_EUSTALL_REPORT1_MASK_SHIFT);
			intel_gt_mcr_unicast_write(gt, XEHPC_EUSTALL_REPORT1,
						   read_ptr_reg,
						   group, instance);
			dss_buf = &stream->dss_buf[dss];
			vaddr_offset = dss * props->eu_stall_buf_sz;
			dss_buf->vaddr = stream->vaddr + vaddr_offset;
			dss_buf->write = write_ptr;
			dss_buf->read = write_ptr;
			dss_buf->line_drop = false;
			mutex_init(&dss_buf->lock);
		}
	}
	return 0;
}

/**
 * i915_eu_stall_buf_read - handles read FOP for i915 EU stall cntr stream FDs
 * @file: An i915 EU stall cntr stream file
 * @buf: destination buffer given by userspace
 * @count: the number of bytes userspace wants to read
 * @ppos: (inout) file seek position (unused)
 *
 * Returns: The number of bytes copied or a negative error code on failure.
 */
static ssize_t i915_eu_stall_buf_read(struct file *file,
				      char __user *buf,
				      size_t count,
				      loff_t *ppos)
{
	struct i915_eu_stall_cntr_stream *stream = file->private_data;
	struct intel_gt *gt = stream->tile_gt;
	ssize_t ret;

	if (!stream->enabled)
		return -EIO;

	if (!(file->f_flags & O_NONBLOCK)) {
		do {
			if (!stream->pollin) {
				ret = wait_event_interruptible(stream->poll_wq, stream->pollin);
				if (ret)
					return -EINTR;
			}

			mutex_lock(&gt->eu_stall_cntr.lock);
			ret = i915_eu_stall_buf_read_locked(stream, file, buf, count, ppos);
			mutex_unlock(&gt->eu_stall_cntr.lock);
		} while (ret == -EAGAIN);
	} else {
		mutex_lock(&gt->eu_stall_cntr.lock);
		ret = i915_eu_stall_buf_read_locked(stream, file, buf, count, ppos);
		mutex_unlock(&gt->eu_stall_cntr.lock);
	}

	stream->pollin = false;

	return ret;
}

static __poll_t
i915_eu_stall_buf_poll_locked(struct i915_eu_stall_cntr_stream *stream,
			      struct file *file, poll_table *wait)
{
	__poll_t events = 0;

	poll_wait(file, &stream->poll_wq, wait);

	if (stream->pollin)
		events |= EPOLLIN;

	return events;
}

static __poll_t
i915_eu_stall_buf_poll(struct file *file, poll_table *wait)
{
	struct i915_eu_stall_cntr_stream *stream = file->private_data;
	struct intel_gt *gt = stream->tile_gt;
	__poll_t ret;

	mutex_lock(&gt->eu_stall_cntr.lock);
	ret = i915_eu_stall_buf_poll_locked(stream, file, wait);
	mutex_unlock(&gt->eu_stall_cntr.lock);

	return ret;
}

static void
i915_eu_stall_cntr_enable_locked(struct i915_eu_stall_cntr_stream *stream)
{
	if (stream->enabled)
		return;

	stream->enabled = true;

	i915_eu_stall_stream_enable(stream);
	hrtimer_start(&stream->poll_check_timer,
		      ns_to_ktime(stream->poll_period),
		      HRTIMER_MODE_REL);
}

static void
i915_eu_stall_cntr_disable_locked(struct i915_eu_stall_cntr_stream *stream)
{
	if (!stream->enabled)
		return;

	stream->enabled = false;

	hrtimer_cancel(&stream->poll_check_timer);
	flush_workqueue(stream->buf_check_wq);
	i915_eu_stall_stream_disable(stream);
}

static long
i915_eu_stall_cntr_ioctl_locked(struct i915_eu_stall_cntr_stream *stream,
				unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case I915_PERF_IOCTL_ENABLE:
		i915_eu_stall_cntr_enable_locked(stream);
		return 0;
	case I915_PERF_IOCTL_DISABLE:
		i915_eu_stall_cntr_disable_locked(stream);
		return 0;
	}

	return -EINVAL;
}

/**
 * i915_eu_stall_cntr_ioctl - support ioctl() usage with i915 EU stall counter
 *								stream FDs
 * @file: An i915 EU stall cntr stream file
 * @cmd: the ioctl request
 * @arg: the ioctl data
 *
 * Implementation deferred to i915_eu_stall_cntr_ioctl_locked().
 *
 * Returns: zero on success or a negative error code. Returns -EINVAL for
 * an unknown ioctl request.
 */
static long i915_eu_stall_cntr_ioctl(struct file *file,
				     unsigned int cmd,
				     unsigned long arg)
{
	struct i915_eu_stall_cntr_stream *stream = file->private_data;
	struct intel_gt *gt = stream->tile_gt;
	long ret;

	mutex_lock(&gt->eu_stall_cntr.lock);
	ret = i915_eu_stall_cntr_ioctl_locked(stream, cmd, arg);
	mutex_unlock(&gt->eu_stall_cntr.lock);

	return ret;
}

static void
i915_eu_stall_destroy_locked(struct i915_eu_stall_cntr_stream *stream)
{
	i915_eu_stall_cntr_disable_locked(stream);
	free_eu_stall_cntr_buf(stream);
}

/**
 * i915_eu_stall_release - handles userspace close() of a EU stall cntr
 *			   stream file.
 * @inode: anonymous inode associated with file
 * @file: An i915 EU stall stream file
 *
 * Cleans up any resources associated with an open EU stall cntr stream file.
 */
static int i915_eu_stall_release(struct inode *inode, struct file *file)
{
	struct i915_eu_stall_cntr_stream *stream = file->private_data;
	struct intel_gt *gt = stream->tile_gt;

	mutex_lock(&gt->eu_stall_cntr.lock);
	i915_eu_stall_destroy_locked(stream);
	kfree(stream);
	gt->eu_stall_cntr.stream = NULL;
	mutex_unlock(&gt->eu_stall_cntr.lock);

	/* Release the reference the EU stall stream kept on the driver */
	drm_dev_put(&gt->i915->drm);

	return 0;
}

static const struct file_operations fops_eu_stall = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.release	= i915_eu_stall_release,
	.poll		= i915_eu_stall_buf_poll,
	.read		= i915_eu_stall_buf_read,
	.unlocked_ioctl = i915_eu_stall_cntr_ioctl,
	.compat_ioctl   = i915_eu_stall_cntr_ioctl,
};

/**
 * i915_open_eu_stall_cntr_locked - Open a EU stall counter stream FD.
 * @perf: i915 device instance
 * @param: The open parameters passed to 'DRM_I915_PERF_OPEN`
 * @props: individually validated u64 property value pairs
 * @file: drm file
 *
 * Note: at this point the @props have only been validated in isolation and
 * it's still necessary to validate that the combination of properties makes
 * sense.
 *
 * Returns: zero on success or a negative error code.
 */
static int
i915_open_eu_stall_cntr_locked(struct drm_i915_private *i915,
			       struct drm_i915_perf_open_param *param,
			       struct eu_stall_open_properties *props,
			       struct drm_file *file)
{
	struct i915_eu_stall_cntr_stream *stream;
	struct intel_gt *gt = props->tile_gt;
	unsigned long f_flags = 0;
	u32 tile_buf_size;
	int stream_fd;
	int ret;

	/* EU stall counters are currently supported only on PVC */
	if (!HAS_EU_STALL_SAMPLING(i915))
		return -EPERM;

	if (i915_perf_stream_paranoid && !perfmon_capable()) {
		DRM_DEBUG("Insufficient privileges for EU stall monitoring\n");
		return -EACCES;
	}

	/* Only one session can be active at any time */
	if (gt->eu_stall_cntr.stream) {
		DRM_DEBUG("EU stall cntr session already active\n");
		return -EBUSY;
	}

	tile_buf_size = props->eu_stall_buf_sz *
			intel_sseu_subslice_total(&gt->info.sseu);
	if (props->event_report_count > num_data_rows(tile_buf_size)) {
		DRM_DEBUG("Invalid EU stall data poll event report count %u\n",
			  props->event_report_count);
		DRM_DEBUG("Maximum event report count for the given buffer size is %u\n",
			  num_data_rows(tile_buf_size));
		return -EINVAL;
	}

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	gt->eu_stall_cntr.stream = stream;
	stream->tile_gt = gt;

	ret = i915_eu_stall_stream_init(stream, param, props);
	if (ret) {
		DRM_DEBUG("EU stall stream init failed : %d\n", ret);
		goto err_alloc;
	}

	if (param->flags & I915_PERF_FLAG_FD_CLOEXEC)
		f_flags |= O_CLOEXEC;
	if (param->flags & I915_PERF_FLAG_FD_NONBLOCK)
		f_flags |= O_NONBLOCK;

	stream_fd = anon_inode_getfd("[i915_eu_stall]", &fops_eu_stall,
				     stream, f_flags);
	if (stream_fd < 0) {
		ret = stream_fd;
		DRM_DEBUG("EU stall inode get fd failed : %d\n", ret);
		goto err_open;
	}

	if (!(param->flags & I915_PERF_FLAG_DISABLED))
		i915_eu_stall_cntr_enable_locked(stream);

	/* Take a reference on the driver that will be kept with stream_fd
	 * until its release.
	 */
	drm_dev_get(&gt->i915->drm);

	return stream_fd;

err_open:
	free_eu_stall_cntr_buf(stream);
err_alloc:
	gt->eu_stall_cntr.stream = NULL;
	kfree(stream);
	return ret;
}

int
i915_open_eu_stall_cntr(struct drm_i915_private *i915,
			struct drm_i915_perf_open_param *param,
			struct drm_file *file)
{
	struct eu_stall_open_properties props;
	int ret;

	ret = read_eu_stall_properties(i915,
				       u64_to_user_ptr(param->properties_ptr),
				       param->num_properties,
				       &props);
	if (ret)
		return ret;

	mutex_lock(&props.tile_gt->eu_stall_cntr.lock);
	ret = i915_open_eu_stall_cntr_locked(i915, param, &props, file);
	mutex_unlock(&props.tile_gt->eu_stall_cntr.lock);
	return ret;
}
