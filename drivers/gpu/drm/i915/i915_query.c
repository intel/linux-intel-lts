/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2018 Intel Corporation
 */

#include <linux/nospec.h>
#include <linux/bits.h>

#include <drm/intel_iaf_platform.h>

#include "gt/intel_engine_pm.h"
#include "gt/intel_engine_regs.h"
#include "gt/intel_engine_user.h"
#include "i915_drv.h"
#include "i915_perf.h"
#include "i915_query.h"
#include "gt/intel_engine_user.h"
#include <uapi/drm/i915_drm.h>
#include "gt/intel_engine_user.h"
#include "gt/intel_gt.h"

static int copy_query_item(void *query_hdr, size_t query_sz,
			   u32 total_length,
			   struct drm_i915_query_item *query_item)
{
	if (query_item->length == 0)
		return total_length;

	if (query_item->length < total_length)
		return -EINVAL;

	if (copy_from_user(query_hdr, u64_to_user_ptr(query_item->data_ptr),
			   query_sz))
		return -EFAULT;

	return 0;
}

static int fill_topology_info(const struct sseu_dev_info *sseu,
			      struct drm_i915_query_item *query_item,
			      intel_sseu_ss_mask_t subslice_mask)
{
	struct drm_i915_query_topology_info topo;
	u32 slice_length, subslice_length, eu_length, total_length;
	int ss_stride = GEN_SSEU_STRIDE(sseu->max_subslices);
	int eu_stride = GEN_SSEU_STRIDE(sseu->max_eus_per_subslice);
	int ret;

	BUILD_BUG_ON(sizeof(u8) != sizeof(sseu->slice_mask));

	if (sseu->max_slices == 0)
		return -ENODEV;

	slice_length = sizeof(sseu->slice_mask);
	subslice_length = sseu->max_slices * ss_stride;
	eu_length = sseu->max_slices * sseu->max_subslices * eu_stride;
	total_length = sizeof(topo) + slice_length + subslice_length +
		       eu_length;

	ret = copy_query_item(&topo, sizeof(topo), total_length, query_item);

	if (ret != 0)
		return ret;

	memset(&topo, 0, sizeof(topo));
	topo.max_slices = sseu->max_slices;
	topo.max_subslices = sseu->max_subslices;
	topo.max_eus_per_subslice = sseu->max_eus_per_subslice;

	topo.subslice_offset = slice_length;
	topo.subslice_stride = ss_stride;
	topo.eu_offset = slice_length + subslice_length;
	topo.eu_stride = eu_stride;

	if (copy_to_user(u64_to_user_ptr(query_item->data_ptr),
			 &topo, sizeof(topo)))
		return -EFAULT;

	if (copy_to_user(u64_to_user_ptr(query_item->data_ptr + sizeof(topo)),
			 &sseu->slice_mask, slice_length))
		return -EFAULT;

	if (intel_sseu_copy_ssmask_to_user(u64_to_user_ptr(query_item->data_ptr +
							   sizeof(topo) + slice_length),
					   sseu))
		return -EFAULT;

	if (intel_sseu_copy_eumask_to_user(u64_to_user_ptr(query_item->data_ptr +
							   sizeof(topo) +
							   slice_length + subslice_length),
					   sseu))
		return -EFAULT;

	return total_length;
}

static int query_topology_info(struct drm_i915_private *dev_priv,
			       struct drm_i915_query_item *query_item)
{
	const struct sseu_dev_info *sseu = &to_gt(dev_priv)->info.sseu;

	if (query_item->flags != 0)
		return -EINVAL;

	return fill_topology_info(sseu, query_item, sseu->subslice_mask);
}

static int query_geometry_subslices(struct drm_i915_private *i915,
				    struct drm_i915_query_item *query_item)
{
	const struct sseu_dev_info *sseu;
	struct intel_engine_cs *engine;
	struct i915_engine_class_instance classinstance;

	if (GRAPHICS_VER_FULL(i915) < IP_VER(12, 50))
		return -ENODEV;

	classinstance = *((struct i915_engine_class_instance *)&query_item->flags);

	engine = intel_engine_lookup_user(i915, (u8)classinstance.engine_class,
					  (u8)classinstance.engine_instance);

	if (!engine)
		return -EINVAL;

	if (engine->class != RENDER_CLASS)
		return -EINVAL;

	sseu = &engine->gt->info.sseu;

	return fill_topology_info(sseu, query_item, sseu->geometry_subslice_mask);
}

static int
query_distance_info(struct drm_i915_private *i915,
		    struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_distance_info __user *query_ptr =
				u64_to_user_ptr(query_item->data_ptr);
	const unsigned int pcie_hop_distance = 10000;
	const unsigned int tile_hop_distance = 1000;
	struct prelim_drm_i915_query_distance_info query;
	enum intel_memory_type mem_type;
	struct intel_memory_region *region;
	struct intel_engine_cs *engine;
	s32 distance;
	int ret;

	ret = copy_query_item(&query, sizeof(query), sizeof(query), query_item);
	if (ret != 0)
		return ret;

	if (query.rsvd[0] || query.rsvd[1] || query.rsvd[2])
		return -EINVAL;

	region = intel_memory_region_lookup(i915,
					    query.region.memory_class,
					    query.region.memory_instance);
	if (!region)
		return -EINVAL;

	engine = intel_engine_lookup_user(i915,
					  query.engine.engine_class,
					  query.engine.engine_instance);
	if (!engine)
		return -EINVAL;

	mem_type = query.region.memory_class;

	if (!HAS_LMEM(i915)) {
		distance = 0;
	} else if (mem_type == INTEL_MEMORY_SYSTEM) {
		distance = pcie_hop_distance;
		if (engine->gt->info.id > 0)
			distance += tile_hop_distance;
		if (engine->gt->info.id > 2)
			distance += tile_hop_distance;
	} else if (mem_type == INTEL_MEMORY_STOLEN) {
		/* FIXME determine the appropriate value */
		distance = 0;
	} else {
		/*
		 * As per bspec tiles are laid out and interconnected as:
		 *
		 *      0 - 1
		 *      |   |
		 *      2 - 3
		 *
		 *  Therefore distance between opposite corner tiles is two
		 *  hops, while the rest are one hop apart.
		 */
		int a = min_t(int,
			      region->gt->info.id,
			      engine->gt->info.id);
		int b = max_t(int,
			      region->gt->info.id,
			      engine->gt->info.id);
		int d;

		if ((a == 0 && b == 3) ||
		    (a == 1 && b == 2))
			d = 2;
		else if (a == b)
			d = 0;
		else
			d = 1;

		distance = d * tile_hop_distance;
	}

	if (put_user(distance, &query_ptr->distance))
		return -EFAULT;

	return sizeof(query);
}

typedef u64 (*__ktime_func_t)(void);
static __ktime_func_t __clock_id_to_func(clockid_t clk_id)
{
	/*
	 * Use logic same as the perf subsystem to allow user to select the
	 * reference clock id to be used for timestamps.
	 */
	switch (clk_id) {
	case CLOCK_MONOTONIC:
		return &ktime_get_ns;
	case CLOCK_MONOTONIC_RAW:
		return &ktime_get_raw_ns;
	case CLOCK_REALTIME:
		return &ktime_get_real_ns;
	case CLOCK_BOOTTIME:
		return &ktime_get_boottime_ns;
	case CLOCK_TAI:
		return &ktime_get_clocktai_ns;
	default:
		return NULL;
	}
}

static inline int
__read_timestamps(struct intel_uncore *uncore,
		  i915_reg_t lower_reg,
		  i915_reg_t upper_reg,
		  u64 *cs_ts,
		  u64 *cpu_ts,
		  __ktime_func_t cpu_clock)
{
	u32 upper, lower, old_upper, loop = 0;

	upper = intel_uncore_read_fw(uncore, upper_reg);
	do {
		*cpu_ts = cpu_clock();
		lower = intel_uncore_read_fw(uncore, lower_reg);
		old_upper = upper;
		upper = intel_uncore_read_fw(uncore, upper_reg);
	} while (upper != old_upper && loop++ < 2);

	*cs_ts = (u64)upper << 32 | lower;

	return 0;
}

static int
__query_cs_cycles(struct intel_engine_cs *engine,
		  u64 *cs_ts, u64 *cpu_ts,
		  __ktime_func_t cpu_clock)
{
	struct intel_uncore *uncore = engine->uncore;
	enum forcewake_domains fw_domains;
	u32 base = engine->mmio_base;
	intel_wakeref_t wakeref;
	int ret;

	fw_domains = intel_uncore_forcewake_for_reg(uncore,
						    RING_TIMESTAMP(base),
						    FW_REG_READ);

	with_intel_runtime_pm(uncore->rpm, wakeref) {
		spin_lock_irq(&uncore->lock);
		intel_uncore_forcewake_get__locked(uncore, fw_domains);

		ret = __read_timestamps(uncore,
					RING_TIMESTAMP(base),
					RING_TIMESTAMP_UDW(base),
					cs_ts,
					cpu_ts,
					cpu_clock);

		intel_uncore_forcewake_put__locked(uncore, fw_domains);
		spin_unlock_irq(&uncore->lock);
	}

	return ret;
}

static int
query_cs_cycles(struct drm_i915_private *i915,
		struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_cs_cycles __user *query_ptr;
	struct prelim_drm_i915_query_cs_cycles query;
	struct intel_engine_cs *engine;
	__ktime_func_t cpu_clock;
	int ret;

	query_ptr = u64_to_user_ptr(query_item->data_ptr);
	ret = copy_query_item(&query, sizeof(query), sizeof(query), query_item);
	if (ret != 0)
		return ret;

	if (query.flags)
		return -EINVAL;

	if (query.rsvd)
		return -EINVAL;

	cpu_clock = __clock_id_to_func(query.clockid);
	if (!cpu_clock)
		return -EINVAL;

	engine = intel_engine_lookup_user(i915,
					  query.engine.engine_class,
					  query.engine.engine_instance);
	if (!engine)
		return -EINVAL;

	query.cs_frequency = engine->gt->clock_frequency;
	ret = __query_cs_cycles(engine,
				&query.cs_cycles,
				&query.cpu_timestamp,
				cpu_clock);
	if (ret)
		return ret;

	if (put_user(query.cs_frequency, &query_ptr->cs_frequency))
		return -EFAULT;

	if (put_user(query.cpu_timestamp, &query_ptr->cpu_timestamp))
		return -EFAULT;

	if (put_user(query.cs_cycles, &query_ptr->cs_cycles))
		return -EFAULT;

	return sizeof(query);
}

static int query_fabric_connectivity(struct drm_i915_private *i915,
				     struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_fabric_info __user *info_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct prelim_drm_i915_query_fabric_info info;
	struct query_info *qi;
	u32 latency = 0;
	int ret;
	int cnt;
	int i;

	ret = copy_query_item(&info, sizeof(info), sizeof(info), query_item);
	if (ret)
		return ret;

	info.bandwidth = 0;
	info.latency = 0;

	/* "Local" access will be on chip, not fabric (bandwidth = 0) */
	if (info.fabric_id == i915->intel_iaf.fabric_id)
		goto done;

	qi = i915->intel_iaf.ops->connectivity_query(i915->intel_iaf.handle,
						     info.fabric_id);
	if (IS_ERR(qi))
		goto done;
	/*
	 * Examine the query information for connectivity.
	 * Minimum bandwidth value is the bandwidth, 0 == no connectivity
	 * Latency is averaged.
	 */
	cnt = qi->src_cnt * qi->dst_cnt;
	if (!cnt) {
		kfree(qi);
		return -ENXIO;
	}

	info.bandwidth = 0xffff;
	for (i = 0; i < cnt; i++) {
		info.bandwidth = min(qi->sd2sd[i].bandwidth, info.bandwidth);
		GEM_WARN_ON(add_overflows_t(u32, latency,
					    qi->sd2sd[i].latency));
		latency += qi->sd2sd[i].latency;
	}

	info.latency = latency / cnt;

	/* we are responsible for freeing qi */
	kfree(qi);

done:
	if (copy_to_user(info_ptr, &info, sizeof(info)))
		return -EFAULT;

	return 0;
}

static int
query_engine_info(struct drm_i915_private *i915,
		  struct drm_i915_query_item *query_item)
{
	struct drm_i915_query_engine_info __user *query_ptr =
				u64_to_user_ptr(query_item->data_ptr);
	struct drm_i915_engine_info __user *info_ptr;
	struct drm_i915_query_engine_info query;
	struct drm_i915_engine_info info = { };
	unsigned int num_uabi_engines = 0;
	struct intel_engine_cs *engine;
	int len, ret;

	if (query_item->flags)
		return -EINVAL;

	for_each_uabi_engine(engine, i915)
		num_uabi_engines++;

	len = struct_size(query_ptr, engines, num_uabi_engines);

	ret = copy_query_item(&query, sizeof(query), len, query_item);
	if (ret != 0)
		return ret;

	if (query.num_engines || query.rsvd[0] || query.rsvd[1] ||
	    query.rsvd[2])
		return -EINVAL;

	info_ptr = &query_ptr->engines[0];

	for_each_uabi_engine(engine, i915) {
		info.engine.engine_class = engine->uabi_class;
		info.engine.engine_instance = engine->uabi_instance;
		info.flags = I915_ENGINE_INFO_HAS_LOGICAL_INSTANCE;
		info.capabilities = engine->uabi_capabilities;
		info.logical_instance = ilog2(engine->logical_mask);

		if (copy_to_user(info_ptr, &info, sizeof(info)))
			return -EFAULT;

		query.num_engines++;
		info_ptr++;
	}

	if (copy_to_user(query_ptr, &query, sizeof(query)))
		return -EFAULT;

	return len;
}

static int
prelim_query_engine_info(struct drm_i915_private *i915,
			 struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_engine_info __user *query_ptr =
					u64_to_user_ptr(query_item->data_ptr);
	struct prelim_drm_i915_engine_info __user *info_ptr;
	struct prelim_drm_i915_query_engine_info query;
	struct prelim_drm_i915_engine_info info = { };
	unsigned int num_uabi_engines = 0;
	struct intel_engine_cs *engine;
	int len, ret;

	if (query_item->flags)
		return -EINVAL;

	for_each_uabi_engine(engine, i915)
		num_uabi_engines++;

	len = struct_size(query_ptr, engines, num_uabi_engines);

	ret = copy_query_item(&query, sizeof(query), len, query_item);
	if (ret != 0)
		return ret;

	if (query.num_engines || query.rsvd[0] || query.rsvd[1] ||
	    query.rsvd[2])
		return -EINVAL;

	info_ptr = &query_ptr->engines[0];

	for_each_uabi_engine(engine, i915) {
		info.engine.engine_class = engine->uabi_class;
		info.engine.engine_instance = engine->uabi_instance;
		info.flags = PRELIM_I915_ENGINE_INFO_HAS_LOGICAL_INSTANCE |
			PRELIM_I915_ENGINE_INFO_HAS_OA_UNIT_ID |
			PRELIM_I915_ENGINE_INFO_HAS_KNOWN_CAPABILITIES;
		info.capabilities = engine->uabi_capabilities;
		info.logical_instance = ilog2(engine->logical_mask);
		info.oa_unit_id = engine->oa_group && engine->oa_group->num_engines ?
				  engine->oa_group->oa_unit_id : U32_MAX;

		switch (engine->uabi_class) {
		case I915_ENGINE_CLASS_COPY:
			info.known_capabilities =
				PRELIM_I915_COPY_CLASS_CAP_BLOCK_COPY |
				PRELIM_I915_COPY_CLASS_CAP_SATURATE_PCIE |
				PRELIM_I915_COPY_CLASS_CAP_SATURATE_LINK |
				PRELIM_I915_COPY_CLASS_CAP_SATURATE_LMEM;
			break;
		case I915_ENGINE_CLASS_VIDEO:
			info.known_capabilities =
				I915_VIDEO_CLASS_CAPABILITY_HEVC |
				I915_VIDEO_AND_ENHANCE_CLASS_CAPABILITY_SFC |
				PRELIM_I915_VIDEO_CLASS_CAPABILITY_VDENC;
			break;
		case I915_ENGINE_CLASS_VIDEO_ENHANCE:
			info.known_capabilities =
				I915_VIDEO_AND_ENHANCE_CLASS_CAPABILITY_SFC;
			break;
		}

		GEM_WARN_ON(info.capabilities & ~info.known_capabilities);

		if (__copy_to_user(info_ptr, &info, sizeof(info)))
			return -EFAULT;

		query.num_engines++;
		info_ptr++;
	}

	if (copy_to_user(query_ptr, &query, sizeof(query)))
		return -EFAULT;

	return len;
}

static int can_copy_perf_config_registers_or_number(u32 user_n_regs,
						    u64 user_regs_ptr,
						    u32 kernel_n_regs)
{
	/*
	 * We'll just put the number of registers, and won't copy the
	 * register.
	 */
	if (user_n_regs == 0)
		return 0;

	if (user_n_regs < kernel_n_regs)
		return -EINVAL;

	return 0;
}

static int copy_perf_config_registers_or_number(const struct i915_oa_reg *kernel_regs,
						u32 kernel_n_regs,
						u64 user_regs_ptr,
						u32 *user_n_regs)
{
	u32 __user *p = u64_to_user_ptr(user_regs_ptr);
	u32 r;

	if (*user_n_regs == 0) {
		*user_n_regs = kernel_n_regs;
		return 0;
	}

	*user_n_regs = kernel_n_regs;

	if (!user_write_access_begin(p, 2 * sizeof(u32) * kernel_n_regs))
		return -EFAULT;

	for (r = 0; r < kernel_n_regs; r++, p += 2) {
		unsafe_put_user(i915_mmio_reg_offset(kernel_regs[r].addr),
				p, Efault);
		unsafe_put_user(kernel_regs[r].value, p + 1, Efault);
	}
	user_write_access_end();
	return 0;
Efault:
	user_write_access_end();
	return -EFAULT;
}

static int query_perf_config_data(struct drm_i915_private *i915,
				  struct drm_i915_query_item *query_item,
				  bool use_uuid)
{
	struct drm_i915_query_perf_config __user *user_query_config_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct drm_i915_perf_oa_config __user *user_config_ptr =
		u64_to_user_ptr(query_item->data_ptr +
				sizeof(struct drm_i915_query_perf_config));
	struct drm_i915_perf_oa_config user_config;
	struct i915_perf *perf = &i915->perf;
	struct i915_oa_config *oa_config;
	char uuid[UUID_STRING_LEN + 1];
	u64 config_id;
	u32 flags, total_size;
	int ret;

	if (!perf->i915)
		return -ENODEV;

	total_size =
		sizeof(struct drm_i915_query_perf_config) +
		sizeof(struct drm_i915_perf_oa_config);

	if (query_item->length == 0)
		return total_size;

	if (query_item->length < total_size) {
		DRM_DEBUG("Invalid query config data item size=%u expected=%u\n",
			  query_item->length, total_size);
		return -EINVAL;
	}

	if (get_user(flags, &user_query_config_ptr->flags))
		return -EFAULT;

	if (flags != 0)
		return -EINVAL;

	if (use_uuid) {
		struct i915_oa_config *tmp;
		int id;

		BUILD_BUG_ON(sizeof(user_query_config_ptr->uuid) >= sizeof(uuid));

		memset(&uuid, 0, sizeof(uuid));
		if (copy_from_user(uuid, user_query_config_ptr->uuid,
				     sizeof(user_query_config_ptr->uuid)))
			return -EFAULT;

		oa_config = NULL;
		rcu_read_lock();
		idr_for_each_entry(&perf->metrics_idr, tmp, id) {
			if (!strcmp(tmp->uuid, uuid)) {
				oa_config = i915_oa_config_get(tmp);
				break;
			}
		}
		rcu_read_unlock();
	} else {
		if (get_user(config_id, &user_query_config_ptr->config))
			return -EFAULT;

		oa_config = i915_perf_get_oa_config(perf, config_id);
	}
	if (!oa_config)
		return -ENOENT;

	if (copy_from_user(&user_config, user_config_ptr, sizeof(user_config))) {
		ret = -EFAULT;
		goto out;
	}

	ret = can_copy_perf_config_registers_or_number(user_config.n_boolean_regs,
						       user_config.boolean_regs_ptr,
						       oa_config->b_counter_regs_len);
	if (ret)
		goto out;

	ret = can_copy_perf_config_registers_or_number(user_config.n_flex_regs,
						       user_config.flex_regs_ptr,
						       oa_config->flex_regs_len);
	if (ret)
		goto out;

	ret = can_copy_perf_config_registers_or_number(user_config.n_mux_regs,
						       user_config.mux_regs_ptr,
						       oa_config->mux_regs_len);
	if (ret)
		goto out;

	ret = copy_perf_config_registers_or_number(oa_config->b_counter_regs,
						   oa_config->b_counter_regs_len,
						   user_config.boolean_regs_ptr,
						   &user_config.n_boolean_regs);
	if (ret)
		goto out;

	ret = copy_perf_config_registers_or_number(oa_config->flex_regs,
						   oa_config->flex_regs_len,
						   user_config.flex_regs_ptr,
						   &user_config.n_flex_regs);
	if (ret)
		goto out;

	ret = copy_perf_config_registers_or_number(oa_config->mux_regs,
						   oa_config->mux_regs_len,
						   user_config.mux_regs_ptr,
						   &user_config.n_mux_regs);
	if (ret)
		goto out;

	memcpy(user_config.uuid, oa_config->uuid, sizeof(user_config.uuid));

	if (copy_to_user(user_config_ptr, &user_config, sizeof(user_config))) {
		ret = -EFAULT;
		goto out;
	}

	ret = total_size;

out:
	i915_oa_config_put(oa_config);
	return ret;
}

static size_t sizeof_perf_config_list(size_t count)
{
	return sizeof(struct drm_i915_query_perf_config) + sizeof(u64) * count;
}

static size_t sizeof_perf_metrics(struct i915_perf *perf)
{
	struct i915_oa_config *tmp;
	size_t i;
	int id;

	i = 1;
	rcu_read_lock();
	idr_for_each_entry(&perf->metrics_idr, tmp, id)
		i++;
	rcu_read_unlock();

	return sizeof_perf_config_list(i);
}

static int query_perf_config_list(struct drm_i915_private *i915,
				  struct drm_i915_query_item *query_item)
{
	struct drm_i915_query_perf_config __user *user_query_config_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct i915_perf *perf = &i915->perf;
	u64 *oa_config_ids = NULL;
	int alloc, n_configs;
	u32 flags;
	int ret;

	if (!perf->i915)
		return -ENODEV;

	if (query_item->length == 0)
		return sizeof_perf_metrics(perf);

	if (get_user(flags, &user_query_config_ptr->flags))
		return -EFAULT;

	if (flags != 0)
		return -EINVAL;

	n_configs = 1;
	do {
		struct i915_oa_config *tmp;
		u64 *ids;
		int id;

		ids = krealloc(oa_config_ids,
			       n_configs * sizeof(*oa_config_ids),
			       GFP_KERNEL);
		if (!ids)
			return -ENOMEM;

		alloc = fetch_and_zero(&n_configs);

		ids[n_configs++] = 1ull; /* reserved for test_config */
		rcu_read_lock();
		idr_for_each_entry(&perf->metrics_idr, tmp, id) {
			if (n_configs < alloc)
				ids[n_configs] = id;
			n_configs++;
		}
		rcu_read_unlock();

		oa_config_ids = ids;
	} while (n_configs > alloc);

	if (query_item->length < sizeof_perf_config_list(n_configs)) {
		DRM_DEBUG("Invalid query config list item size=%u expected=%zu\n",
			  query_item->length,
			  sizeof_perf_config_list(n_configs));
		kfree(oa_config_ids);
		return -EINVAL;
	}

	if (put_user(n_configs, &user_query_config_ptr->config)) {
		kfree(oa_config_ids);
		return -EFAULT;
	}

	ret = copy_to_user(user_query_config_ptr + 1,
			   oa_config_ids,
			   n_configs * sizeof(*oa_config_ids));
	kfree(oa_config_ids);
	if (ret)
		return -EFAULT;

	return sizeof_perf_config_list(n_configs);
}

static int query_perf_config(struct drm_i915_private *i915,
			     struct drm_i915_query_item *query_item)
{
	switch (query_item->flags) {
	case DRM_I915_QUERY_PERF_CONFIG_LIST:
		return query_perf_config_list(i915, query_item);
	case DRM_I915_QUERY_PERF_CONFIG_DATA_FOR_UUID:
		return query_perf_config_data(i915, query_item, true);
	case DRM_I915_QUERY_PERF_CONFIG_DATA_FOR_ID:
		return query_perf_config_data(i915, query_item, false);
	default:
		return -EINVAL;
	}
}

static int query_memregion_info(struct drm_i915_private *i915,
				struct drm_i915_query_item *query_item)
{
	struct drm_i915_query_memory_regions __user *query_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct drm_i915_memory_region_info __user *info_ptr =
		&query_ptr->regions[0];
	struct drm_i915_memory_region_info info = { };
	struct drm_i915_query_memory_regions query;
	struct intel_memory_region *mr;
	u32 total_length;
	int ret, id, i;

	if (query_item->flags != 0)
		return -EINVAL;

	total_length = sizeof(query);
	for_each_memory_region(mr, i915, id) {
		if (mr->private)
			continue;

		total_length += sizeof(info);
	}

	ret = copy_query_item(&query, sizeof(query), total_length, query_item);
	if (ret != 0)
		return ret;

	if (query.num_regions)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(query.rsvd); i++) {
		if (query.rsvd[i])
			return -EINVAL;
	}

	for_each_memory_region(mr, i915, id) {
		if (mr->private)
			continue;

		info.region.memory_class = mr->type;
		info.region.memory_instance = mr->instance;
		info.probed_size = mr->total;
		info.unallocated_size = atomic64_read(&mr->avail);

		if (__copy_to_user(info_ptr, &info, sizeof(info)))
			return -EFAULT;

		query.num_regions++;
		info_ptr++;
	}

	if (__copy_to_user(query_ptr, &query, sizeof(query)))
		return -EFAULT;

	return total_length;
}

static int query_hwconfig_blob(struct drm_i915_private *i915,
			       struct drm_i915_query_item *query_item)
{
	struct intel_gt *gt = to_gt(i915);
	struct intel_hwconfig *hwconfig = &gt->info.hwconfig;

	if (!hwconfig->size || !hwconfig->ptr)
		return -ENODEV;

	if (query_item->length == 0)
		return hwconfig->size;

	if (query_item->length < hwconfig->size)
		return -EINVAL;

	if (copy_to_user(u64_to_user_ptr(query_item->data_ptr),
			 hwconfig->ptr, hwconfig->size))
		return -EFAULT;

	return hwconfig->size;
}

static int prelim_query_memregion_info(struct drm_i915_private *dev_priv,
				       struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_memory_regions __user *query_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct prelim_drm_i915_memory_region_info __user *info_ptr =
		&query_ptr->regions[0];
	struct prelim_drm_i915_memory_region_info info = { };
	struct prelim_drm_i915_query_memory_regions query;
	u32 total_length;
	int ret, i;

	if (query_item->flags != 0)
		return -EINVAL;

	total_length = sizeof(query);
	for (i = 0; i < ARRAY_SIZE(dev_priv->mm.regions); ++i) {
		struct intel_memory_region *region = dev_priv->mm.regions[i];

		if (!region)
			continue;

		if (region->private)
			continue;

		total_length += sizeof(info);
	}

	ret = copy_query_item(&query, sizeof(query), total_length, query_item);
	if (ret != 0)
		return ret;

	if (query.num_regions)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(query.rsvd); ++i) {
		if (query.rsvd[i])
			return  -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(dev_priv->mm.regions); ++i) {
		struct intel_memory_region *region = dev_priv->mm.regions[i];

		if (!region)
			continue;

		if (region->private)
			continue;

		info.region.memory_class = region->type;
		info.region.memory_instance = region->instance;
		info.probed_size = region->total;
		info.unallocated_size = atomic64_read(&region->avail);

		if (__copy_to_user(info_ptr, &info, sizeof(info)))
			return -EFAULT;

		query.num_regions++;
		info_ptr++;
	}

	if (__copy_to_user(query_ptr, &query, sizeof(query)))
		return -EFAULT;

	return total_length;
}

static int prelim_query_lmem_memregion_info(struct drm_i915_private *i915,
					    struct drm_i915_query_item
					    *query_item)
{
	struct prelim_drm_i915_query_lmem_memory_regions __user *query_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct prelim_drm_i915_lmem_memory_region_info __user *info_ptr =
		&query_ptr->regions[0];
	struct prelim_drm_i915_lmem_memory_region_info info = { };
	struct prelim_drm_i915_query_lmem_memory_regions query;
	u32 total_length;
	int ret, i;

	if (query_item->flags != 0)
		return -EINVAL;

	total_length = sizeof(query);
	for (i = 0; i < ARRAY_SIZE(i915->mm.regions); ++i) {
		struct intel_memory_region *region = i915->mm.regions[i];

		if (!region)
			continue;

		if (region->private)
			continue;

		if (region->type != INTEL_MEMORY_LOCAL)
			continue;

		total_length += sizeof(info);
	}

	ret = copy_query_item(&query, sizeof(query), total_length, query_item);
	if (ret != 0)
		return ret;

	if (query.num_lmem_regions)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(query.rsvd); ++i) {
		if (query.rsvd[i])
			return  -EINVAL;
	}

	i915_gem_drain_freed_objects(i915);

	for (i = 0; i < ARRAY_SIZE(i915->mm.regions); ++i) {
		struct intel_memory_region *region = i915->mm.regions[i];

		if (!region)
			continue;

		if (region->private)
			continue;

		if (region->type != INTEL_MEMORY_LOCAL)
			continue;

		info.region.memory_class = region->type;
		info.region.memory_instance = region->instance;
		if (capable(CAP_SYS_ADMIN)) {
			info.unallocated_usr_lmem_size =
				READ_ONCE(region->acct_limit[INTEL_MEMORY_OVERCOMMIT_LMEM]);
			info.unallocated_usr_shared_size =
				READ_ONCE(region->acct_limit[INTEL_MEMORY_OVERCOMMIT_SHARED]);
		}

		if (__copy_to_user(info_ptr, &info, sizeof(info)))
			return -EFAULT;

		query.num_lmem_regions++;
		info_ptr++;
	}

	if (__copy_to_user(query_ptr, &query, sizeof(query)))
		return -EFAULT;

	return total_length;
}

static int query_compute_subslices(struct drm_i915_private *i915,
				struct drm_i915_query_item *query_item)
{
	const struct sseu_dev_info *sseu;
	struct intel_engine_cs *engine;
	u8 engine_class, engine_instance;

	if (GRAPHICS_VER_FULL(i915) < IP_VER(12, 50))
		return -ENODEV;

	engine_class = query_item->flags & 0xFF;
	engine_instance = (query_item->flags >> 8) & 0xFF;

	engine = intel_engine_lookup_user(i915, engine_class, engine_instance);

	if (!engine)
		return -EINVAL;

	sseu = &engine->gt->info.sseu;

	return fill_topology_info(sseu, query_item, sseu->compute_subslice_mask);
}

static int prelim_query_hw_ip_version(struct drm_i915_private *i915,
			       struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_hw_ip_version ipver;
	struct intel_engine_cs *engine;
	int ret;

	ret = copy_query_item(&ipver, sizeof(ipver), sizeof(ipver), query_item);
	if (ret != 0)
		return ret;

	/*
	 * Flags (both inside the query item and inside the ip version
	 * structure) are reserved for future expansion; we don't accept any
	 * yet.
	 */
	if (ipver.flags != 0 || query_item->flags != 0)
		return -EINVAL;

	engine = intel_engine_lookup_user(i915,
					  ipver.engine.engine_class,
					  ipver.engine.engine_instance);
	if (!engine)
		return -EINVAL;

	switch (engine->uabi_class) {
		default:
			MISSING_CASE(engine->class);
			fallthrough;
		case I915_ENGINE_CLASS_RENDER:
		case I915_ENGINE_CLASS_COMPUTE:
		case I915_ENGINE_CLASS_COPY:
			ipver.arch = RUNTIME_INFO(i915)->graphics.ver;
			ipver.release = RUNTIME_INFO(i915)->graphics.rel;
			ipver.stepping = RUNTIME_INFO(i915)->graphics.step;
			break;
		case I915_ENGINE_CLASS_VIDEO:
		case I915_ENGINE_CLASS_VIDEO_ENHANCE:
			ipver.arch = RUNTIME_INFO(i915)->media.ver;
			ipver.release = RUNTIME_INFO(i915)->media.rel;
			ipver.stepping = RUNTIME_INFO(i915)->media.step;
			break;
	}

	if (copy_to_user(u64_to_user_ptr(query_item->data_ptr), &ipver,
			 sizeof(ipver)))
		return -EFAULT;

	return sizeof(ipver);
}

static int
prelim_query_l3bank_count(struct drm_i915_private *i915,
			  struct drm_i915_query_item *query_item)
{
	struct prelim_drm_i915_query_memory_regions __user *query_ptr =
		u64_to_user_ptr(query_item->data_ptr);
	struct intel_engine_cs *engine;
	u8 engine_class, engine_instance;
	int count;

	engine_class = query_item->flags & 0xFF;
	engine_instance = (query_item->flags >> 8) & 0xFF;

	engine = intel_engine_lookup_user(i915, engine_class, engine_instance);

	if (!engine)
		return -EINVAL;

	count = intel_count_l3_banks(i915, engine);

	if (count < 0)
		return count;

	if (copy_to_user(query_ptr, &count, sizeof(count)))
		return -EFAULT;

	return sizeof(count);
}

typedef int (* const i915_query_funcs_table)(struct drm_i915_private *dev_priv,
						    struct drm_i915_query_item *query_item);

static i915_query_funcs_table i915_query_funcs[] = {
	query_topology_info,
	query_engine_info,
	query_perf_config,
	query_memregion_info,
	query_hwconfig_blob,
	query_geometry_subslices,
};

static i915_query_funcs_table i915_query_funcs_prelim[] = {
#define MAKE_TABLE_IDX(id)		[PRELIM_DRM_I915_QUERY_MASK(PRELIM_DRM_I915_QUERY_##id) - 1]

	MAKE_TABLE_IDX(MEMORY_REGIONS) = prelim_query_memregion_info,
	MAKE_TABLE_IDX(DISTANCE_INFO) = query_distance_info,
	MAKE_TABLE_IDX(HWCONFIG_TABLE) = query_hwconfig_blob,
	MAKE_TABLE_IDX(GEOMETRY_SUBSLICES) = query_geometry_subslices,
	MAKE_TABLE_IDX(COMPUTE_SUBSLICES) = query_compute_subslices,
	MAKE_TABLE_IDX(CS_CYCLES) = query_cs_cycles,
	MAKE_TABLE_IDX(FABRIC_INFO) = query_fabric_connectivity,
	MAKE_TABLE_IDX(HW_IP_VERSION) = prelim_query_hw_ip_version,
	MAKE_TABLE_IDX(ENGINE_INFO) = prelim_query_engine_info,
	MAKE_TABLE_IDX(L3BANK_COUNT) = prelim_query_l3bank_count,
	MAKE_TABLE_IDX(LMEM_MEMORY_REGIONS) = prelim_query_lmem_memregion_info,

#undef MAKE_TABLE_IDX
};

int i915_query_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct drm_i915_query *args = data;
	struct drm_i915_query_item __user *user_item_ptr =
		u64_to_user_ptr(args->items_ptr);
	u32 i;

	if (args->flags != 0)
		return -EINVAL;

	for (i = 0; i < args->num_items; i++, user_item_ptr++) {
		struct drm_i915_query_item item;
		i915_query_funcs_table *table;
		unsigned long func_idx;
		size_t table_size;
		int ret;

		if (copy_from_user(&item, user_item_ptr, sizeof(item)))
			return -EFAULT;

		if (item.query_id == 0)
			return -EINVAL;

		if (overflows_type(item.query_id - 1, unsigned long))
			return -EINVAL;

		if (item.query_id & PRELIM_DRM_I915_QUERY) {
			table = i915_query_funcs_prelim;
			table_size = ARRAY_SIZE(i915_query_funcs_prelim);
			func_idx = PRELIM_DRM_I915_QUERY_MASK(item.query_id) - 1;
		} else {
			table = i915_query_funcs;
			table_size = ARRAY_SIZE(i915_query_funcs);
			func_idx = item.query_id - 1;
		}

		ret = -EINVAL;
		if (func_idx < table_size) {
			func_idx = array_index_nospec(func_idx, table_size);
			if (table[func_idx])
				ret = table[func_idx](dev_priv, &item);
		}

		/* Only write the length back to userspace if they differ. */
		if (ret != item.length && put_user(ret, &user_item_ptr->length))
			return -EFAULT;
	}

	return 0;
}
