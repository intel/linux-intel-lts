/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2008-2018 Intel Corporation
 */

#ifndef _I915_GPU_ERROR_H_
#define _I915_GPU_ERROR_H_

#include <linux/atomic.h>
#include <linux/kref.h>
#include <linux/ktime.h>
#include <linux/sched.h>

#include <drm/drm_mm.h>

#include "gt/intel_engine.h"
#include "gt/intel_gt_types.h"
#include "gt/uc/intel_uc_fw.h"

#include "intel_device_info.h"

#include "i915_gem.h"
#include "i915_gem_gtt.h"
#include "i915_params.h"
#include "i915_scheduler.h"

struct drm_i915_private;
struct i915_page_compress;
struct intel_engine_capture_vma;
struct intel_overlay_error_state;

struct i915_compressed_pages {
	int num_pages;
	int page_count;
	int unused;
	u32 *pages[0];
};

struct i915_uuid_resource {
	/* This is for internal tracking.
	 * During error handler the whole structure is passed
	 * to error dump which holds data structure for a while.
	 * Use i915_uuid_get() and i915_uuid_put() for this.
	 */
	struct kref ref;
	/* Helps keep track if someting depends on this uuid */
	atomic_t bind_count;

	/* String formatted like "%08x-%04x-%04x-%04x-%012x" */
	char uuid[36];
	/* Predefined or previously registered UUID indicating class */
	u32 uuid_class;
	/* Copy of the CPU memory payload associated with UUID */
	void *ptr;
	/* Length of the payload in bytes */
	u64 size;
	u32 handle;
};

struct i915_uuid_resource_coredump {
	struct i915_uuid_resource_coredump *next;
	/* UUID of the resource */
	char uuid[36];
	/* UUID of the class this is instance of. 0 means base */
	char class[36];

	bool string_class;
	union {
		char *str;
		struct i915_compressed_pages *cpages;
	};
};

struct i915_vma_metadata_coredump {
	char uuid[36]; /* String formatted like "%08x-%04x-%04x-%04x-%012x" */
	struct i915_vma_metadata_coredump *next;
};

struct i915_vma_coredump {
	struct i915_vma_coredump *next;

	char name[20];

	u64 gtt_offset;
	u64 gtt_size;
	u32 gtt_page_sizes;

	struct i915_compressed_pages *cpages;
	struct i915_vma_metadata_coredump *metadata;
};

struct i915_request_coredump {
	unsigned long flags;
	pid_t pid;
	u32 context;
	u32 seqno;
	u32 head;
	u32 tail;
	struct i915_sched_attr sched_attr;
};

struct __guc_capture_parsed_output;

struct intel_engine_coredump {
	const struct intel_engine_cs *engine;

	bool hung;
	bool simulated;
	int reset_count;

	/* position of active request inside the ring */
	u32 rq_head, rq_post, rq_tail, vm_poison;

	/* Register state */
	u32 ccid;
	u32 start;
	u32 tail;
	u32 head;
	u32 ctl;
	u32 mode;
	u32 hws;
	u32 ipeir;
	u32 ipehr;
	u32 esr;
	u32 bbstate;
	u32 instpm;
	u32 instps;
	u64 bbaddr;
	u64 acthd;
	u32 fault_reg;
	u32 ctxt_sr_ctl;
	u64 faddr;
	u32 rc_psmi; /* sleep state */
	u32 nopid;
	u32 excc;
	u32 cmd_cctl;
	u32 cscmdop;
	u32 ctx_sr_ctl;
	u32 dma_faddr_hi;
	u32 dma_faddr_lo;
	struct intel_instdone instdone;

	/* GuC matched capture-lists info */
	struct intel_guc_state_capture *guc_capture;
	struct __guc_capture_parsed_output *guc_capture_node;

	struct i915_gem_context_coredump {
		char comm[TASK_COMM_LEN];

		u64 total_runtime;
		u64 avg_runtime;

		pid_t pid;
		int active;
		int guilty;
		struct i915_sched_attr sched_attr;
		u32 hwsp_seqno;
		bool sip_installed;

		struct i915_uuid_resource_coredump *uuid_dump;
	} context;

	struct i915_vma_coredump *vma;

	struct i915_request_coredump execlist[EXECLIST_MAX_PORTS];
	unsigned int num_ports;

	struct {
		u32 gfx_mode;
		union {
			u64 pdp[4];
			u32 pp_dir_base;
		};
	} vm_info;

	struct intel_engine_coredump *next;
};

struct intel_ctb_coredump {
	u32 raw_head, head;
	u32 raw_tail, tail;
	u32 raw_status;
	u32 desc_offset;
	u32 cmds_offset;
	u32 size;
};

struct intel_eu_attentions {
#define ES_MAX_EUS 1024
#define ES_MAX_THREADS 8

	u8 att[ES_MAX_EUS * ES_MAX_THREADS / BITS_PER_BYTE];
	unsigned int size;
	ktime_t ts;
};

static inline unsigned int
intel_eu_attentions_count(const struct intel_eu_attentions *a)
{
	return bitmap_weight((void *)a->att, a->size * BITS_PER_BYTE);
}

struct intel_gt_coredump {
	const struct intel_gt *_gt;
	bool awake;
	bool simulated;

	struct intel_gt_info info;
	u32 engines_reset_count;

	/* Generic register state */
	u32 eir;
	u32 pgtbl_er;
	u32 ier;
	u32 gtier[6], ngtier;
	u32 forcewake;
	u32 error; /* gen6+ */
	u32 err_int; /* gen7 */
	u32 fault_data0; /* gen8, gen9 */
	u32 fault_data1; /* gen8, gen9 */
	u32 done_reg;
	u32 gac_eco;
	u32 gam_ecochk;
	u32 gab_ctl;
	u32 gfx_mode;
	u32 gtt_cache;
	u32 aux_err; /* gen12 */
	u32 gam_done; /* gen12 */
	u32 clock_frequency;
	u32 clock_period_ns;
	u32 eu_global_sip;

	/* Display related */
	u32 derrmr;
	u32 sfc_done[I915_MAX_SFC]; /* gen12 */

	u32 nfence;
	u64 fence[I915_MAX_NUM_FENCES];

	struct intel_engine_coredump *engine;

	struct intel_uc_coredump {
		struct intel_uc_fw guc_fw;
		struct intel_uc_fw huc_fw;
		struct guc_info {
			struct intel_ctb_coredump ctb[2];
			struct i915_vma_coredump *vma_ctb;
			struct i915_vma_coredump *vma_log;
			u32 timestamp;
			u16 last_fence;
			bool is_guc_capture;
		} guc;
	} *uc;

	struct {
		u32 td_ctl; /* can be in power ctx on newer gens */

		struct intel_eu_attentions before;
		struct intel_eu_attentions after;
		struct intel_eu_attentions resolved;
	} attentions;

	struct intel_gt_coredump *next;
};

struct i915_gpu_coredump {
	struct kref ref;
	ktime_t time;
	ktime_t boottime;
	ktime_t uptime;
	unsigned long capture;

	struct drm_i915_private *i915;

	struct intel_gt_coredump *gt;

	char error_msg[128];
	bool simulated;
	bool wakelock;
	bool suspended;
	int iommu;
	u32 reset_count;
	u32 suspend_count;

	struct {
		u64 addr;
		int type;
		int level;
		int access;
	} fault;

	struct intel_device_info device_info;
	struct intel_runtime_info runtime_info;
	struct intel_driver_caps driver_caps;
	struct i915_params params;

	struct intel_overlay_error_state *overlay;

	struct scatterlist *sgl, *fit;

	void *private;
};

struct i915_gpu_error {
	/* For reset and error_state handling. */
	spinlock_t lock;
	/* Protected by the above dev->gpu_error.lock. */
	struct i915_gpu_coredump *first_error;

	atomic_t pending_fb_pin;

	/** Number of times the device has been reset (global) */
	atomic_t reset_count;
};

struct drm_i915_error_state_buf {
	struct drm_i915_private *i915;
	struct scatterlist *sgl, *cur, *end;

	char *buf;
	size_t bytes;
	size_t size;
	loff_t iter;

	int err;
};

static inline u32 i915_reset_count(struct i915_gpu_error *error)
{
	return atomic_read(&error->reset_count);
}

static inline int i915_reset_engine_count(const struct intel_engine_cs *engine)
{
	/* the present guc interface doesn't support per engine reset counts */
	if (intel_engine_uses_guc(engine))
		return -1;

	return atomic_read(&engine->reset.count);
}

#define CORE_DUMP_FLAG_NONE           0x0
#define CORE_DUMP_FLAG_IS_GUC_CAPTURE BIT(0)

#if IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR) && IS_ENABLED(CONFIG_DRM_I915_DEBUG_GEM)
void intel_klog_error_capture(struct intel_gt *gt,
			      intel_engine_mask_t engine_mask);
#else
static inline void intel_klog_error_capture(struct intel_gt *gt,
					    intel_engine_mask_t engine_mask)
{
}
#endif

#if IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR)

__printf(2, 3)
void i915_error_printf(struct drm_i915_error_state_buf *e, const char *f, ...);
void intel_gpu_error_print_vma(struct drm_i915_error_state_buf *m,
			       const struct intel_engine_cs *engine,
			       const struct i915_vma_coredump *vma);
struct i915_vma_coredump *
intel_gpu_error_find_batch(const struct intel_engine_coredump *ee);

struct i915_gpu_coredump *i915_gpu_coredump(struct intel_gt *gt,
					    intel_engine_mask_t engine_mask, u32 dump_flags);
void i915_capture_error_state(struct intel_gt *gt,
			      intel_engine_mask_t engine_mask, u32 dump_flags);

struct i915_gpu_coredump *
i915_gpu_coredump_alloc(struct drm_i915_private *i915, gfp_t gfp);

struct intel_gt_coredump *
intel_gt_coredump_alloc(struct intel_gt *gt, gfp_t gfp, u32 dump_flags);

struct intel_engine_coredump *
intel_engine_coredump_alloc(struct intel_engine_cs *engine, gfp_t gfp, u32 dump_flags);

struct i915_gpu_coredump *
i915_gpu_coredump_create_for_engine(struct intel_engine_cs *engine, gfp_t gfp);

struct intel_engine_capture_vma *
intel_engine_coredump_add_request(struct intel_engine_coredump *ee,
				  struct i915_request *rq,
				  gfp_t gfp,
				  struct i915_page_compress *compress);

void intel_engine_coredump_add_vma(struct intel_engine_coredump *ee,
				   struct intel_engine_capture_vma *capture,
				   struct i915_page_compress *compress);

struct i915_page_compress *
i915_vma_capture_prepare(struct intel_gt_coredump *gt);

void i915_vma_capture_finish(struct intel_gt_coredump *gt,
			     struct i915_page_compress *compress);

void i915_error_state_store(struct i915_gpu_coredump *error);

static inline struct i915_gpu_coredump *
i915_gpu_coredump_get(struct i915_gpu_coredump *gpu)
{
	kref_get(&gpu->ref);
	return gpu;
}

ssize_t
i915_gpu_coredump_copy_to_buffer(struct i915_gpu_coredump *error,
				 char *buf, loff_t offset, size_t count);

void __i915_gpu_coredump_free(struct kref *kref);
static inline void i915_gpu_coredump_put(struct i915_gpu_coredump *gpu)
{
	if (gpu)
		kref_put(&gpu->ref, __i915_gpu_coredump_free);
}

struct i915_gpu_coredump *i915_first_error_state(struct drm_i915_private *i915);
void i915_reset_error_state(struct drm_i915_private *i915);
void i915_disable_error_state(struct drm_i915_private *i915, int err);

int i915_uuid_register_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file);
int i915_uuid_unregister_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file);
void i915_uuid_init(struct i915_drm_client *client);
void i915_uuid_cleanup(struct i915_drm_client *client);

static inline void i915_uuid_get(struct i915_uuid_resource *uuid_res)
{
	kref_get(&uuid_res->ref);
}

void __i915_uuid_free(struct kref *ref);
static inline void i915_uuid_put(struct i915_uuid_resource *uuid_res)
{
	kref_put(&uuid_res->ref, __i915_uuid_free);
}

void intel_eu_attentions_read(struct intel_gt *gt,
			      struct intel_eu_attentions *a,
			      const unsigned int settle_time_ms);

#else

__printf(2, 3)
static inline void
i915_error_printf(struct drm_i915_error_state_buf *e, const char *f, ...)
{
}

static inline void
i915_capture_error_state(struct intel_gt *gt, intel_engine_mask_t engine_mask, u32 dump_flags)
{
}

static inline struct i915_gpu_coredump *
i915_gpu_coredump_alloc(struct drm_i915_private *i915, gfp_t gfp)
{
	return NULL;
}

static inline struct i915_gpu_coredump *
i915_gpu_coredump_create_for_engine(struct intel_engine_cs *engine, gfp_t gfp)
{
	return NULL;
}

static inline struct intel_gt_coredump *
intel_gt_coredump_alloc(struct intel_gt *gt, gfp_t gfp, u32 dump_flags)
{
	return NULL;
}

static inline struct intel_engine_coredump *
intel_engine_coredump_alloc(struct intel_engine_cs *engine, gfp_t gfp, u32 dump_flags)
{
	return NULL;
}

static inline struct intel_engine_capture_vma *
intel_engine_coredump_add_request(struct intel_engine_coredump *ee,
				  struct i915_request *rq,
				  gfp_t gfp,
				  struct i915_page_compress *compress)
{
	return NULL;
}

static inline void
intel_engine_coredump_add_vma(struct intel_engine_coredump *ee,
			      struct intel_engine_capture_vma *capture,
			      struct i915_page_compress *compress)
{
}

static inline struct i915_page_compress *
i915_vma_capture_prepare(struct intel_gt_coredump *gt)
{
	return NULL;
}

static inline void
i915_vma_capture_finish(struct intel_gt_coredump *gt,
			struct i915_page_compress *compress)
{
}

static inline void
i915_error_state_store(struct i915_gpu_coredump *error)
{
}

static inline void i915_gpu_coredump_put(struct i915_gpu_coredump *gpu)
{
}

static inline struct i915_gpu_coredump *
i915_first_error_state(struct drm_i915_private *i915)
{
	return ERR_PTR(-ENODEV);
}

static inline void i915_reset_error_state(struct drm_i915_private *i915)
{
}

static inline void i915_disable_error_state(struct drm_i915_private *i915,
					    int err)
{
}

static inline int
i915_uuid_register_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file)
{
	return -EOPNOTSUPP;
}

static inline int
i915_uuid_unregister_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file)
{
	return -EOPNOTSUPP;
}


static inline void i915_uuid_init(struct i915_drm_client *client)
{
}

static inline void i915_uuid_cleanup(struct i915_drm_client *client)
{
}

static inline void i915_uuid_get(struct i915_uuid_resource *uuid_res)
{
}

static inline void i915_uuid_put(struct i915_uuid_resource *uuid_res)
{
}

static inline void
intel_eu_attentions_read(struct intel_gt *gt,
			 struct intel_eu_attentions *a,
			 const unsigned int settle_time_ms)
{
}

#endif /* IS_ENABLED(CONFIG_DRM_I915_CAPTURE_ERROR) */

#endif /* _I915_GPU_ERROR_H_ */
