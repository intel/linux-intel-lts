/* i915_drv.h -- Private header for the I915 driver -*- linux-c -*-
 */
/*
 *
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _I915_DRV_H_
#define _I915_DRV_H_

#include <uapi/drm/i915_drm.h>

#include <linux/i2c.h>
#include <linux/intel-iommu.h>
#include <linux/pm_qos.h>
#include <linux/xarray.h>

#include <drm/drm_connector.h>
#include <drm/i915_cp_fw_hdcp_interface.h>
#include <drm/ttm/ttm_device.h>

#include "display/intel_cdclk.h"
#include "display/intel_display.h"
#include "display/intel_display_power.h"
#include "display/intel_dmc.h"
#include "display/intel_dpll_mgr.h"
#include "display/intel_dsb.h"
#include "display/intel_fbc.h"
#include "display/intel_frontbuffer.h"
#include "display/intel_global_state.h"
#include "display/intel_gmbus.h"
#include "display/intel_opregion.h"

#include "gem/i915_gem_context_types.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_shrinker.h"
#include "gem/i915_gem_stolen.h"

#include "gt/intel_engine.h"
#include "gt/intel_gt_types.h"
#include "gt/intel_region_lmem.h"
#include "gt/intel_timeline.h"
#include "gt/intel_workarounds.h"
#include "gt/uc/intel_uc.h"

#include "spi/intel_spi.h"

#include "i915_drm_client.h"
#include "i915_gem.h"
#include "i915_gem_gtt.h"
#include "i915_gpu_error.h"
#include "i915_params.h"
#include "i915_perf_types.h"
#include "i915_request.h"
#include "i915_scheduler.h"
#include "i915_sriov.h"
#include "i915_sriov_types.h"
#include "i915_utils.h"
#include "i915_virtualization.h"
#include "i915_vma.h"
#include "intel_device_info.h"
#include "intel_memory_region.h"
#include "intel_pch.h"
#include "intel_pm_types.h"
#include "intel_runtime_pm.h"
#include "intel_step.h"
#include "intel_uncore.h"
#include "i915_addr_trans_svc.h"

struct dpll;
struct drm_i915_clock_gating_funcs;
struct drm_i915_gem_object;
struct drm_i915_private;
struct i915_ggtt;
struct i915_sched_engine;
struct intel_atomic_state;
struct intel_audio_funcs;
struct intel_cdclk_config;
struct intel_cdclk_funcs;
struct intel_cdclk_state;
struct intel_cdclk_vals;
struct intel_color_funcs;
struct intel_connector;
struct intel_crtc;
struct intel_dp;
struct intel_dpll_funcs;
struct intel_encoder;
struct intel_fbdev;
struct intel_fdi_funcs;
struct intel_gmbus;
struct intel_hotplug_funcs;
struct intel_initial_plane_config;
struct intel_limit;
struct intel_overlay;
struct intel_overlay_error_state;
struct vlv_s0ix_state;

#define I915_GFP_ALLOW_FAIL (GFP_KERNEL | __GFP_RETRY_MAYFAIL | __GFP_NOWARN)

enum i915_driver_errors {
	I915_DRIVER_ERROR_OBJECT_MIGRATION = 0,
	I915_DRIVER_ERROR_COUNT,
};

/* Threshold == 5 for long IRQs, 50 for short */
#define HPD_STORM_DEFAULT_THRESHOLD 50

struct i915_hotplug {
	struct delayed_work hotplug_work;

	const u32 *hpd, *pch_hpd;

	struct {
		unsigned long last_jiffies;
		int count;
		enum {
			HPD_ENABLED = 0,
			HPD_DISABLED = 1,
			HPD_MARK_DISABLED = 2
		} state;
	} stats[HPD_NUM_PINS];
	u32 event_bits;
	u32 retry_bits;
	struct delayed_work reenable_work;

	u32 long_port_mask;
	u32 short_port_mask;
	struct work_struct dig_port_work;

	struct work_struct poll_init_work;
	bool poll_enabled;

	unsigned int hpd_storm_threshold;
	/* Whether or not to count short HPD IRQs in HPD storms */
	u8 hpd_short_storm_enabled;

	/*
	 * if we get a HPD irq from DP and a HPD irq from non-DP
	 * the non-DP HPD could block the workqueue on a mode config
	 * mutex getting, that userspace may have taken. However
	 * userspace is waiting on the DP workqueue to run which is
	 * blocked behind the non-DP one.
	 */
	struct workqueue_struct *dp_wq;

	/*
	 * Flag to track if long HPDs need not to be processed
	 *
	 * Some panels generate long HPDs while keep connected to the port.
	 * This can cause issues with CI tests results. In CI systems we
	 * don't expect to disconnect the panels and could ignore the long
	 * HPDs generated from the faulty panels. This flag can be used as
	 * cue to ignore the long HPDs and can be set / unset using debugfs.
	 */
	bool ignore_long_hpd;
};

#define I915_GEM_GPU_DOMAINS \
	(I915_GEM_DOMAIN_RENDER | \
	 I915_GEM_DOMAIN_SAMPLER | \
	 I915_GEM_DOMAIN_COMMAND | \
	 I915_GEM_DOMAIN_INSTRUCTION | \
	 I915_GEM_DOMAIN_VERTEX)

struct drm_i915_file_private {
	struct drm_i915_private *dev_priv;

	union {
		struct drm_file *file;
		struct rcu_head rcu;
	};

	struct xarray context_xa;
	struct xarray vm_xa;

	unsigned int bsd_engine;

/*
 * Every context ban increments per client ban score. Also
 * hangs in short succession increments ban score. If ban threshold
 * is reached, client is considered banned and submitting more work
 * will fail. This is a stop gap measure to limit the badly behaving
 * clients access to gpu. Note that unbannable contexts never increment
 * the client ban score.
 */
#define I915_CLIENT_SCORE_HANG_FAST	1
#define   I915_CLIENT_FAST_HANG_JIFFIES (60 * HZ)
#define I915_CLIENT_SCORE_CONTEXT_BAN   3
#define I915_CLIENT_SCORE_BANNED	9
	/** ban_score: Accumulated score of all ctx bans and fast hangs. */
	atomic_t ban_score;
	unsigned long hang_timestamp;

	struct i915_drm_client *client;

	struct clos_reservation {
		u32 clos_mask;         // Mask of CLOS sets reserved by client
		u8 l3_rsvd_ways;      // Number of L3 Ways reserved by client, across all CLOS
	} clos_resv;
};

struct sdvo_device_mapping {
	u8 initialized;
	u8 dvo_port;
	u8 slave_addr;
	u8 dvo_wiring;
	u8 i2c_pin;
	u8 ddc_pin;
};

/* functions used for watermark calcs for display. */
struct drm_i915_wm_disp_funcs {
	/* update_wm is for legacy wm management */
	void (*update_wm)(struct drm_i915_private *dev_priv);
	int (*compute_pipe_wm)(struct intel_atomic_state *state,
			       struct intel_crtc *crtc);
	int (*compute_intermediate_wm)(struct intel_atomic_state *state,
				       struct intel_crtc *crtc);
	void (*initial_watermarks)(struct intel_atomic_state *state,
				   struct intel_crtc *crtc);
	void (*atomic_update_watermarks)(struct intel_atomic_state *state,
					 struct intel_crtc *crtc);
	void (*optimize_watermarks)(struct intel_atomic_state *state,
				    struct intel_crtc *crtc);
	int (*compute_global_watermarks)(struct intel_atomic_state *state);
};

struct drm_i915_display_funcs {
	/* Returns the active state of the crtc, and if the crtc is active,
	 * fills out the pipe-config with the hw state. */
	bool (*get_pipe_config)(struct intel_crtc *,
				struct intel_crtc_state *);
	void (*get_initial_plane_config)(struct intel_crtc *,
					 struct intel_initial_plane_config *);
	void (*crtc_enable)(struct intel_atomic_state *state,
			    struct intel_crtc *crtc);
	void (*crtc_disable)(struct intel_atomic_state *state,
			     struct intel_crtc *crtc);
	void (*commit_modeset_enables)(struct intel_atomic_state *state);
};


#define I915_COLOR_UNEVICTABLE (-1) /* a non-vma sharing the address space */

#define GEM_QUIRK_PIN_SWIZZLED_PAGES	BIT(0)

#define QUIRK_LVDS_SSC_DISABLE (1<<1)
#define QUIRK_INVERT_BRIGHTNESS (1<<2)
#define QUIRK_BACKLIGHT_PRESENT (1<<3)
#define QUIRK_INCREASE_T12_DELAY (1<<6)
#define QUIRK_INCREASE_DDI_DISABLED_TIME (1<<7)
#define QUIRK_NO_PPS_BACKLIGHT_POWER_HOOK (1<<8)

struct i915_suspend_saved_registers {
	u32 saveDSPARB;
	u32 saveSWF0[16];
	u32 saveSWF1[16];
	u32 saveSWF3[3];
	u16 saveGCDGMBUS;
};

#define MAX_L3_SLICES 2
struct intel_l3_parity {
	u32 *remap_info[MAX_L3_SLICES];
	struct work_struct error_work;
	int which_slice;
};

struct i915_gem_mm {
	/*
	 * Shortcut for the stolen region. This points to either
	 * INTEL_REGION_STOLEN_SMEM for integrated platforms, or
	 * INTEL_REGION_STOLEN_LMEM for discrete, or NULL if the device doesn't
	 * support stolen.
	 */
	struct intel_memory_region *stolen_region;
	/** Memory allocator for GTT stolen memory */
	struct drm_mm stolen;
	/** Protects the usage of the GTT stolen memory allocator. This is
	 * always the inner lock when overlapping with struct_mutex. */
	struct mutex stolen_lock;

	/* Protects bound_list/unbound_list and #drm_i915_gem_object.mm.link */
	spinlock_t obj_lock;

	/**
	 * List of objects which are purgeable.
	 */
	struct list_head purge_list;

	/**
	 * List of objects which have allocated pages and are shrinkable.
	 */
	struct list_head shrink_list;

	/**
	 * List of objects which are pending destruction.
	 */
	struct llist_head free_list;
	struct work_struct free_work;
	/**
	 * Count of objects pending destructions. Used to skip needlessly
	 * waiting on an RCU barrier if no objects are waiting to be freed.
	 */
	atomic_t free_count;

	/**
	 * tmpfs instance used for shmem backed objects
	 */
	struct vfsmount *gemfs;

	struct intel_memory_region *regions[INTEL_REGION_UNKNOWN];

	struct notifier_block oom_notifier;
	struct notifier_block vmap_notifier;
	struct shrinker shrinker;

	/* shrinker accounting, also useful for userland debugging */
	u64 shrink_memory;
	u32 shrink_count;

	/* lmemovercommit limit in %, set through sysfs */
	u8 user_acct_limit[2];

	/* background task for returning bound system pages */
	struct {
		struct task_struct *tsk; /* our kswapd equivalent */

		/*
		 * Track the number of pages do_shrink_slab() has asked us
		 * to reclaim, and we have failed to find. This count of
		 * outstanding reclaims is passed to the swapper thread,
		 * which then blocks as it tries to achieve that goal.
		 * It is likely that the target overshoots due to the
		 * latency between our thread and kswapd making new requests.
		 */
		atomic_long_t target;
	} swapper;

	struct workqueue_struct *wq;
	struct i915_sched_engine *sched;
};

#define I915_IDLE_ENGINES_TIMEOUT (500) /* in ms */

unsigned long i915_fence_context_timeout(const struct drm_i915_private *i915,
					 u64 context);

static inline unsigned long
i915_fence_timeout(const struct drm_i915_private *i915)
{
	return i915_fence_context_timeout(i915, U64_MAX);
}

/* Amount of SAGV/QGV points, BSpec precisely defines this */
#define I915_NUM_QGV_POINTS 8

#define HAS_HW_SAGV_WM(i915) (DISPLAY_VER(i915) >= 13 && !IS_DGFX(i915))

/* Amount of PSF GV points, BSpec precisely defines this */
#define I915_NUM_PSF_GV_POINTS 3

struct intel_vbt_data {
	/* bdb version */
	u16 version;

	/* Feature bits */
	unsigned int int_tv_support:1;
	unsigned int int_crt_support:1;
	unsigned int lvds_use_ssc:1;
	unsigned int int_lvds_support:1;
	unsigned int display_clock_mode:1;
	unsigned int fdi_rx_polarity_inverted:1;
	int lvds_ssc_freq;
	enum drm_panel_orientation orientation;

	bool override_afc_startup;
	u8 override_afc_startup_val;

	int crt_ddc_pin;

	struct list_head display_devices;
	struct list_head bdb_blocks;

	struct intel_bios_encoder_data *ports[I915_MAX_PORTS]; /* Non-NULL if port present. */
	struct sdvo_device_mapping sdvo_mappings[2];
};

struct i915_frontbuffer_tracking {
	spinlock_t lock;

	/*
	 * Tracking bits for delayed frontbuffer flushing du to gpu activity or
	 * scheduled flips.
	 */
	unsigned busy_bits;
	unsigned flip_bits;
};

struct i915_virtual_gpu {
	struct mutex lock; /* serialises sending of g2v_notify command pkts */
	bool active;
	u32 caps;
};

struct i915_selftest_stash {
	atomic_t counter;
	struct ida mock_region_instances;
};

/* intel_audio.c private */
struct intel_audio_private {
	/* Display internal audio functions */
	const struct intel_audio_funcs *funcs;

	/* hda/i915 audio component */
	struct i915_audio_component *component;
	bool component_registered;
	/* mutex for audio/video sync */
	struct mutex mutex;
	int power_refcount;
	u32 freq_cntrl;

	/* Used to save the pipe-to-encoder mapping for audio */
	struct intel_encoder *encoder_map[I915_MAX_PIPES];

	/* necessary resource sharing with HDMI LPE audio driver. */
	struct {
		struct platform_device *platdev;
		int irq;
	} lpe;
};

struct drm_i915_private {
	struct drm_device drm;

	/* FIXME: Device release actions should all be moved to drmm_ */
	bool do_release;

	/* i915 device parameters */
	struct i915_params params;

	/* i915 virtualization mode, use IOV_MODE() to access */
	enum i915_iov_mode __mode;
#define IOV_MODE(i915) ({				\
	BUILD_BUG_ON(!I915_IOV_MODE_NONE);		\
	GEM_BUG_ON(!(i915)->__mode);			\
	(i915)->__mode;					\
})

	const struct intel_device_info __info; /* Use INTEL_INFO() to access. */
	struct intel_runtime_info __runtime; /* Use RUNTIME_INFO() to access. */
	struct intel_driver_caps caps;

	/**
	 * Data Stolen Memory - aka "i915 stolen memory" gives us the start and
	 * end of stolen which we can optionally use to create GEM objects
	 * backed by stolen memory. Note that stolen_usable_size tells us
	 * exactly how much of this we are actually allowed to use, given that
	 * some portion of it is in fact reserved for use by hardware functions.
	 */
	struct resource dsm;
	/**
	 * Reseved portion of Data Stolen Memory
	 */
	struct resource dsm_reserved;

	/*
	 * Stolen memory is segmented in hardware with different portions
	 * offlimits to certain functions.
	 *
	 * The drm_mm is initialised to the total accessible range, as found
	 * from the PCI config. On Broadwell+, this is further restricted to
	 * avoid the first page! The upper end of stolen memory is reserved for
	 * hardware functions and similarly removed from the accessible range.
	 */
	resource_size_t stolen_usable_size;	/* Total size minus reserved ranges */

	unsigned int remote_tiles;
	struct intel_uncore uncore;
	struct intel_uncore_mmio_debug mmio_debug;

	struct i915_sriov sriov;
	struct i915_virtual_gpu vgpu;

	struct intel_gvt *gvt;

	struct intel_dmc dmc;

	struct intel_gmbus *gmbus[GMBUS_NUM_PINS];

	/** gmbus_mutex protects against concurrent usage of the single hw gmbus
	 * controller on different i2c buses. */
	struct mutex gmbus_mutex;

	/* svm_init_mutex  protects concurrent svm devmem initialization for lmem regions */
	struct mutex svm_init_mutex;

	/**
	 * Base address of where the gmbus and gpio blocks are located (either
	 * on PCH or on SoC for platforms without PCH).
	 */
	u32 gpio_mmio_base;

	/* MMIO base address for MIPI regs */
	u32 mipi_mmio_base;

	u32 pps_mmio_base;

	wait_queue_head_t gmbus_wait_queue;

	struct pci_dev *bridge_dev;

	struct rb_root uabi_engines;

	struct resource mch_res;

	/* protects the irq masks */
	spinlock_t irq_lock;

	bool display_irqs_enabled;

	/* Sideband mailbox protection */
	struct mutex sb_lock;
	struct pm_qos_request sb_qos;

	/** Cached value of IMR to avoid reads in updating the bitfield */
	union {
		u32 irq_mask;
		u32 de_irq_mask[I915_MAX_PIPES];
	};
	u32 pipestat_irq_mask[I915_MAX_PIPES];

	struct i915_hotplug hotplug;
	struct intel_fbc *fbc[I915_MAX_FBCS];
	struct intel_opregion opregion;
	struct intel_vbt_data vbt;

	bool preserve_bios_swizzle;

	/* overlay */
	struct intel_overlay *overlay;

	/* backlight registers and fields in struct intel_panel */
	struct mutex backlight_lock;

	/* protects panel power sequencer state */
	struct mutex pps_mutex;

	unsigned int fsb_freq, mem_freq, is_ddr3;
	unsigned int skl_preferred_vco_freq;
	unsigned int max_cdclk_freq;

	unsigned int max_dotclk_freq;
	unsigned int hpll_freq;
	unsigned int fdi_pll_freq;
	unsigned int czclk_freq;

	struct {
		/* The current hardware cdclk configuration */
		struct intel_cdclk_config hw;

		/* cdclk, divider, and ratio table from bspec */
		const struct intel_cdclk_vals *table;

		struct intel_global_obj obj;
	} cdclk;

	struct {
		/* The current hardware dbuf configuration */
		u8 enabled_slices;

		struct intel_global_obj obj;
	} dbuf;

	struct {
		wait_queue_head_t waitqueue;
		struct mutex lock;
		struct intel_global_obj obj;
	} pmdemand;

	/**
	 * wq - Driver workqueue for GEM.
	 *
	 * NOTE: Work items scheduled here are not allowed to grab any modeset
	 * locks, for otherwise the flushing done in the pageflip code will
	 * result in deadlocks.
	 */
	struct workqueue_struct *wq;
	struct i915_sched_engine *sched;

	/* ordered wq for modesets */
	struct workqueue_struct *modeset_wq;
	/* unbound hipri wq for page flips/plane updates */
	struct workqueue_struct *flip_wq;

	/* pm private clock gating functions */
	const struct drm_i915_clock_gating_funcs *clock_gating_funcs;

	/* pm display functions */
	const struct drm_i915_wm_disp_funcs *wm_disp;

	/* irq display functions */
	const struct intel_hotplug_funcs *hotplug_funcs;

	/* fdi display functions */
	const struct intel_fdi_funcs *fdi_funcs;

	/* display pll funcs */
	const struct intel_dpll_funcs *dpll_funcs;

	/* Display functions */
	const struct drm_i915_display_funcs *display;

	/* Display internal color functions */
	const struct intel_color_funcs *color_funcs;

	/* Display CDCLK functions */
	const struct intel_cdclk_funcs *cdclk_funcs;

	/* PCH chipset type */
	enum intel_pch pch_type;
	unsigned short pch_id;

	unsigned long gem_quirks;
	unsigned long quirks;

	struct drm_atomic_state *modeset_restore_state;
	struct drm_modeset_acquire_ctx reset_ctx;

	struct i915_gem_mm mm;

	/* Kernel Modesetting */

	/**
	 * dpll and cdclk state is protected by connection_mutex
	 * dpll.lock serializes intel_{prepare,enable,disable}_shared_dpll.
	 * Must be global rather than per dpll, because on some platforms plls
	 * share registers.
	 */
	struct {
		struct mutex lock;

		int num_shared_dpll;
		struct intel_shared_dpll shared_dplls[I915_NUM_PLLS];
		const struct intel_dpll_mgr *mgr;

		struct {
			int nssc;
			int ssc;
		} ref_clks;
	} dpll;

	struct list_head global_obj_list;

	struct i915_frontbuffer_tracking fb_tracking;

	struct intel_atomic_helper {
		struct llist_head free_list;
		struct work_struct free_work;
	} atomic_helper;

	bool mchbar_need_disable;

	struct intel_l3_parity l3_parity;

	/*
	 * HTI (aka HDPORT) state read during initial hw readout.  Most
	 * platforms don't have HTI, so this will just stay 0.  Those that do
	 * will use this later to figure out which PLLs and PHYs are unavailable
	 * for driver usage.
	 */
	u32 hti_state;

	/*
	 * edram size in MB.
	 * Cannot be determined by PCIID. You must always read a register.
	 */
	u32 edram_size_mb;

	struct i915_power_domains power_domains;

	struct i915_gpu_error gpu_error;

	/* list of fbdev register on this device */
	struct intel_fbdev *fbdev;
	struct work_struct fbdev_suspend_work;

	struct drm_property *broadcast_rgb_property;
	struct drm_property *force_audio_property;

	u32 fdi_rx_config;

	/* Shadow for DISPLAY_PHY_CONTROL which can't be safely read */
	u32 chv_phy_control;
	/*
	 * Shadows for CHV DPLL_MD regs to keep the state
	 * checker somewhat working in the presence hardware
	 * crappiness (can't read out DPLL_MD for pipes B & C).
	 */
	u32 chv_dpll_md[I915_MAX_PIPES];
	u32 bxt_phy_grc;

	u32 suspend_count;
	struct i915_suspend_saved_registers regfile;
	struct vlv_s0ix_state *vlv_s0ix_state;

	enum {
		I915_SAGV_UNKNOWN = 0,
		I915_SAGV_DISABLED,
		I915_SAGV_ENABLED,
		I915_SAGV_NOT_CONTROLLED
	} sagv_status;

	u32 sagv_block_time_us;

	struct {
		/*
		 * Raw watermark latency values:
		 * in 0.1us units for WM0,
		 * in 0.5us units for WM1+.
		 */
		/* primary */
		u16 pri_latency[5];
		/* sprite */
		u16 spr_latency[5];
		/* cursor */
		u16 cur_latency[5];
		/*
		 * Raw watermark memory latency values
		 * for SKL for all 8 levels
		 * in 1us units.
		 */
		u16 skl_latency[8];

		/* current hardware state */
		union {
			struct ilk_wm_values hw;
			struct vlv_wm_values vlv;
			struct g4x_wm_values g4x;
		};

		u8 max_level;

		/*
		 * Should be held around atomic WM register writing; also
		 * protects * intel_crtc->wm.active and
		 * crtc_state->wm.need_postvbl_update.
		 */
		struct mutex wm_mutex;
	} wm;

	struct dram_info {
		bool wm_lv_0_adjust_needed;
		u8 num_channels;
		bool symmetric_memory;
		enum intel_dram_type {
			INTEL_DRAM_UNKNOWN,
			INTEL_DRAM_DDR3,
			INTEL_DRAM_DDR4,
			INTEL_DRAM_LPDDR3,
			INTEL_DRAM_LPDDR4,
			INTEL_DRAM_DDR5,
			INTEL_DRAM_LPDDR5,
		} type;
		u8 num_qgv_points;
		u8 num_psf_gv_points;
	} dram_info;

	struct intel_bw_info {
		/* for each QGV point */
		unsigned int deratedbw[I915_NUM_QGV_POINTS];
		/* for each PSF GV point */
		unsigned int psf_bw[I915_NUM_PSF_GV_POINTS];
		u8 num_qgv_points;
		u8 num_psf_gv_points;
		u8 num_planes;
	} max_bw[6];

	struct intel_global_obj bw_obj;

	struct intel_runtime_pm runtime_pm;

	struct i915_perf perf;

	struct intel_spi spi;

	struct i915_hwmon *hwmon;

	/* Abstract the submission mechanism (legacy ringbuffer or execlists) away */
	struct intel_gt gt0;

	/*
	 * i915->gt[0] == &i915->gt0
	 */
	struct intel_gt *gt[I915_MAX_GT];

	/* Quick lookup of media GT (current platforms only have one) */
	struct intel_gt *media_gt;

	struct kobject *sysfs_gt;

	struct {
		struct i915_gem_contexts {
			spinlock_t lock; /* locks list */
			struct list_head list;
		} contexts;

		/*
		 * We replace the local file with a global mappings as the
		 * backing storage for the mmap is on the device and not
		 * on the struct file, and we do not want to prolong the
		 * lifetime of the local fd. To minimise the number of
		 * anonymous inodes we create, we use a global singleton to
		 * share the global mapping.
		 */
		struct file *mmap_singleton;
	} gem;

	struct {
		/* Xarry to hold address space entries */
		struct xarray xa;
		/* Support cyclic allocation of asid */
		u32 next_id;
#define I915_MAX_ASID BIT(20)
	} asid_resv;

	u8 pch_ssc_use;

	/* For i915gm/i945gm vblank irq workaround */
	u8 vblank_enabled;

	union {
		/* perform PHY state sanity checks? */
		bool chv_phy_assert[2];

		/*
		 * DG2: Mask of PHYs that were not calibrated by the firmware
		 * and should not be used.
		 */
		u8 snps_phy_failed_calibration;
	};

	bool ipc_enabled;

	struct intel_audio_private audio;

	bool quiesce_gpu;

	bool invalidate_lmem_mmaps;

	atomic_t active_fault_handlers;

	struct i915_pmu pmu;

	struct i915_drm_clients clients;

#if IS_ENABLED(CONFIG_DRM_I915_DEBUGGER)
	struct {
		spinlock_t lock; /* lock for list */
		struct list_head list;
		u64 session_count;

		struct mutex eu_flush_lock;

		/* Allow to enable the debugger */
		bool allow_eu_debug;
		/* lock for enable_eu_debug */
		struct mutex enable_eu_debug_lock;
		/* debugger state */
		bool enable_eu_debug;
	} debuggers;
#endif

	struct i915_hdcp_fw_master *hdcp_master;
	bool hdcp_comp_added;

	/* Mutex to protect the above hdcp component related values. */
	struct mutex hdcp_comp_mutex;

	/* The TTM device structure. */
	struct ttm_device bdev;

	bool bind_ctxt_ready;

	atomic_t level4_wa_disabled;

	I915_SELFTEST_DECLARE(struct i915_selftest_stash selftest;)

	/**
	 * struct intel_iaf - fabric information (if available)
	 * @ops: shared interface operations
	 * @handle: fabric device handle
	 * @pd: platform data needed for auxiliary bus
	 * @dpa: base device physical address
	 * @irq_base: base IRQ for multi tile devices
	 * @index: internal index for i915 devices
	 * @fabric_id: fabric id generated by the fabric device
	 * @socket_id: socket from certain platforms
	 * @power_enabled: IAF power state
	 * @present: Reflect PUNIT presence information
	 * @power_mutex: Lock to protect power state changes
	 *
	 * For those gpus with fabric connectivity
	 */
	struct {
		const struct iaf_ops *ops;
		void *handle;
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
		struct iaf_pdata *pd;
#endif
		u64 dpa;
		int irq_base;
		int index;
		u32 fabric_id;
		u8 socket_id;
		bool power_enabled;
		bool present;
		/* protect handle and power_enabled during power changes */
		struct mutex power_mutex;
	} intel_iaf;

	/*
	 * NOTE: This is the dri1/ums dungeon, don't add stuff here. Your patch
	 * will be rejected. Instead look for a better place.
	 */

	struct wait_queue_head user_fence_wq;

	unsigned long errors[I915_DRIVER_ERROR_COUNT];

#define NUM_CLOS 4
	struct cache_reservation {
		u32 free_clos_mask;    // Mask of CLOS sets that have not been reserved
		struct mutex clos_mutex;
		u8 ways[NUM_CLOS];
	} cache_resv;

	bool device_faulted;
	struct pci_saved_state *pci_state;

	/* Address translation service support */
	struct i915_ats_priv *ats_priv;
	unsigned long flags;
	int pasid_counter;
#define INTEL_FLAG_ATS_ENABLED		0
};

static inline struct drm_i915_private *to_i915(const struct drm_device *dev)
{
	return container_of(dev, struct drm_i915_private, drm);
}

static inline struct drm_i915_private *kdev_to_i915(struct device *kdev)
{
	return dev_get_drvdata(kdev);
}

static inline struct drm_i915_private *pdev_to_i915(struct pci_dev *pdev)
{
	return pci_get_drvdata(pdev);
}

#define PRELIM_DRM_IOCTL_DEF_DRV(ioctl, _func, _flags)			\
	[DRM_IOCTL_NR(PRELIM_DRM_IOCTL_##ioctl) - DRM_COMMAND_BASE] = {	\
		.cmd = PRELIM_DRM_IOCTL_##ioctl,			\
		.func = _func,						\
		.flags = _flags,					\
		.name = #ioctl						\
	}

/* Simple iterator over all initialised engines */
#define for_each_engine(engine__, dev_priv__, id__) \
	for ((id__) = 0; \
	     (id__) < I915_NUM_ENGINES; \
	     (id__)++) \
		for_each_if ((engine__) = (dev_priv__)->engine[(id__)])

/* Iterator over subset of engines selected by mask */
#define for_each_engine_masked(engine__, gt__, mask__, tmp__) \
	for ((tmp__) = (mask__) & (gt__)->info.engine_mask; \
	     (tmp__) ? \
	     ((engine__) = (gt__)->engine[__mask_next_bit(tmp__)]), 1 : \
	     0;)

#define rb_to_uabi_engine(rb) \
	rb_entry_safe(rb, struct intel_engine_cs, uabi_node)

#define for_each_uabi_engine(engine__, i915__) \
	for ((engine__) = rb_to_uabi_engine(rb_first(&(i915__)->uabi_engines));\
	     (engine__); \
	     (engine__) = rb_to_uabi_engine(rb_next(&(engine__)->uabi_node)))

#define for_each_uabi_class_engine(engine__, class__, i915__) \
	for ((engine__) = intel_engine_lookup_user((i915__), (class__), 0); \
	     (engine__) && (engine__)->uabi_class == (class__); \
	     (engine__) = rb_to_uabi_engine(rb_next(&(engine__)->uabi_node)))

#define I915_GTT_OFFSET_NONE ((u32)-1)

#define INTEL_INFO(dev_priv)	(&(dev_priv)->__info)
#define RUNTIME_INFO(dev_priv)	(&(dev_priv)->__runtime)
#define DRIVER_CAPS(dev_priv)	(&(dev_priv)->caps)

#define INTEL_DEVID(dev_priv)	(RUNTIME_INFO(dev_priv)->device_id)
#define INTEL_PPGTT_MSB(dev_priv)	(INTEL_INFO(dev_priv)->ppgtt_msb)

#define IP_VER(ver, rel)		((ver) << 8 | (rel))

#define GRAPHICS_VER(i915)		(RUNTIME_INFO(i915)->graphics.ver)
#define GRAPHICS_VER_FULL(i915)		IP_VER(RUNTIME_INFO(i915)->graphics.ver, \
					       RUNTIME_INFO(i915)->graphics.rel)
#define IS_GRAPHICS_VER(i915, from, until) \
	(GRAPHICS_VER(i915) >= (from) && GRAPHICS_VER(i915) <= (until))

#define MEDIA_VER(i915)			(RUNTIME_INFO(i915)->media.ver)
#define MEDIA_VER_FULL(i915)		IP_VER(RUNTIME_INFO(i915)->media.ver, \
					       RUNTIME_INFO(i915)->media.rel)
#define IS_MEDIA_VER(i915, from, until) \
	(MEDIA_VER(i915) >= (from) && MEDIA_VER(i915) <= (until))

#define DISPLAY_VER(i915)		(RUNTIME_INFO(i915)->display.ver)
#define DISPLAY_VER_FULL(i915)		IP_VER(RUNTIME_INFO(i915)->display.ver, \
					       RUNTIME_INFO(i915)->display.rel)
#define IS_DISPLAY_VER(i915, from, until) \
	(DISPLAY_VER(i915) >= (from) && DISPLAY_VER(i915) <= (until))

#define INTEL_REVID(dev_priv)	(to_pci_dev((dev_priv)->drm.dev)->revision)

#define HAS_DSB(dev_priv)	(INTEL_INFO(dev_priv)->display.has_dsb)

#define INTEL_DISPLAY_STEP(__i915) (RUNTIME_INFO(__i915)->step.display_step)
#define INTEL_GRAPHICS_STEP(__i915) (RUNTIME_INFO(__i915)->step.graphics_step)
#define INTEL_MEDIA_STEP(__i915) (RUNTIME_INFO(__i915)->step.media_step)
#define INTEL_BASEDIE_STEP(__i915) (RUNTIME_INFO(__i915)->step.basedie_step)

#define IS_DISPLAY_STEP(__i915, since, until) \
	(drm_WARN_ON(&(__i915)->drm, INTEL_DISPLAY_STEP(__i915) == STEP_NONE), \
	 INTEL_DISPLAY_STEP(__i915) >= (since) && INTEL_DISPLAY_STEP(__i915) < (until))

#define IS_GRAPHICS_STEP(__i915, since, until) \
	(drm_WARN_ON(&(__i915)->drm, INTEL_GRAPHICS_STEP(__i915) == STEP_NONE), \
	 INTEL_GRAPHICS_STEP(__i915) >= (since) && INTEL_GRAPHICS_STEP(__i915) < (until))

#define IS_MEDIA_STEP(__i915, since, until) \
	(drm_WARN_ON(&(__i915)->drm, INTEL_MEDIA_STEP(__i915) == STEP_NONE), \
	 INTEL_MEDIA_STEP(__i915) >= (since) && INTEL_MEDIA_STEP(__i915) < (until))

#define IS_BASEDIE_STEP(__i915, since, until) \
	(drm_WARN_ON(&(__i915)->drm, INTEL_BASEDIE_STEP(__i915) == STEP_NONE), \
	 INTEL_BASEDIE_STEP(__i915) >= (since) && INTEL_BASEDIE_STEP(__i915) < (until))

static __always_inline unsigned int
__platform_mask_index(const struct intel_runtime_info *info,
		      enum intel_platform p)
{
	const unsigned int pbits =
		BITS_PER_TYPE(info->platform_mask[0]) - INTEL_SUBPLATFORM_BITS;

	/* Expand the platform_mask array if this fails. */
	BUILD_BUG_ON(INTEL_MAX_PLATFORMS >
		     pbits * ARRAY_SIZE(info->platform_mask));

	return p / pbits;
}

static __always_inline unsigned int
__platform_mask_bit(const struct intel_runtime_info *info,
		    enum intel_platform p)
{
	const unsigned int pbits =
		BITS_PER_TYPE(info->platform_mask[0]) - INTEL_SUBPLATFORM_BITS;

	return p % pbits + INTEL_SUBPLATFORM_BITS;
}

static inline u32
intel_subplatform(const struct intel_runtime_info *info, enum intel_platform p)
{
	const unsigned int pi = __platform_mask_index(info, p);

	return info->platform_mask[pi] & INTEL_SUBPLATFORM_MASK;
}

static __always_inline bool
IS_PLATFORM(const struct drm_i915_private *i915, enum intel_platform p)
{
	const struct intel_runtime_info *info = RUNTIME_INFO(i915);
	const unsigned int pi = __platform_mask_index(info, p);
	const unsigned int pb = __platform_mask_bit(info, p);

	BUILD_BUG_ON(!__builtin_constant_p(p));

	return info->platform_mask[pi] & BIT(pb);
}

static __always_inline bool
IS_SUBPLATFORM(const struct drm_i915_private *i915,
	       enum intel_platform p, unsigned int s)
{
	const struct intel_runtime_info *info = RUNTIME_INFO(i915);
	const unsigned int pi = __platform_mask_index(info, p);
	const unsigned int pb = __platform_mask_bit(info, p);
	const unsigned int msb = BITS_PER_TYPE(info->platform_mask[0]) - 1;
	const u32 mask = info->platform_mask[pi];

	BUILD_BUG_ON(!__builtin_constant_p(p));
	BUILD_BUG_ON(!__builtin_constant_p(s));
	BUILD_BUG_ON((s) >= INTEL_SUBPLATFORM_BITS);

	/* Shift and test on the MSB position so sign flag can be used. */
	return ((mask << (msb - pb)) & (mask << (msb - s))) & BIT(msb);
}

#define IS_MOBILE(dev_priv)	(INTEL_INFO(dev_priv)->is_mobile)
#define IS_DGFX(dev_priv)   (INTEL_INFO(dev_priv)->is_dgfx)

#define IS_I830(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I830)
#define IS_I845G(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I845G)
#define IS_I85X(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I85X)
#define IS_I865G(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I865G)
#define IS_I915G(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I915G)
#define IS_I915GM(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I915GM)
#define IS_I945G(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I945G)
#define IS_I945GM(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I945GM)
#define IS_I965G(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I965G)
#define IS_I965GM(dev_priv)	IS_PLATFORM(dev_priv, INTEL_I965GM)
#define IS_G45(dev_priv)	IS_PLATFORM(dev_priv, INTEL_G45)
#define IS_GM45(dev_priv)	IS_PLATFORM(dev_priv, INTEL_GM45)
#define IS_G4X(dev_priv)	(IS_G45(dev_priv) || IS_GM45(dev_priv))
#define IS_PINEVIEW(dev_priv)	IS_PLATFORM(dev_priv, INTEL_PINEVIEW)
#define IS_G33(dev_priv)	IS_PLATFORM(dev_priv, INTEL_G33)
#define IS_IRONLAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_IRONLAKE)
#define IS_IRONLAKE_M(dev_priv) \
	(IS_PLATFORM(dev_priv, INTEL_IRONLAKE) && IS_MOBILE(dev_priv))
#define IS_SANDYBRIDGE(dev_priv) IS_PLATFORM(dev_priv, INTEL_SANDYBRIDGE)
#define IS_IVYBRIDGE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_IVYBRIDGE)
#define IS_IVB_GT1(dev_priv)	(IS_IVYBRIDGE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 1)
#define IS_VALLEYVIEW(dev_priv)	IS_PLATFORM(dev_priv, INTEL_VALLEYVIEW)
#define IS_CHERRYVIEW(dev_priv)	IS_PLATFORM(dev_priv, INTEL_CHERRYVIEW)
#define IS_HASWELL(dev_priv)	IS_PLATFORM(dev_priv, INTEL_HASWELL)
#define IS_BROADWELL(dev_priv)	IS_PLATFORM(dev_priv, INTEL_BROADWELL)
#define IS_SKYLAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_SKYLAKE)
#define IS_BROXTON(dev_priv)	IS_PLATFORM(dev_priv, INTEL_BROXTON)
#define IS_KABYLAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_KABYLAKE)
#define IS_GEMINILAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_GEMINILAKE)
#define IS_COFFEELAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_COFFEELAKE)
#define IS_COMETLAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_COMETLAKE)
#define IS_ICELAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_ICELAKE)
#define IS_JSL_EHL(dev_priv)	(IS_PLATFORM(dev_priv, INTEL_JASPERLAKE) || \
				IS_PLATFORM(dev_priv, INTEL_ELKHARTLAKE))
#define IS_TIGERLAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_TIGERLAKE)
#define IS_ROCKETLAKE(dev_priv)	IS_PLATFORM(dev_priv, INTEL_ROCKETLAKE)
#define IS_DG1(dev_priv)        IS_PLATFORM(dev_priv, INTEL_DG1)
#define IS_ALDERLAKE_S(dev_priv) IS_PLATFORM(dev_priv, INTEL_ALDERLAKE_S)
#define IS_ALDERLAKE_P(dev_priv) IS_PLATFORM(dev_priv, INTEL_ALDERLAKE_P)
#define IS_XEHPSDV(dev_priv) IS_PLATFORM(dev_priv, INTEL_XEHPSDV)
#define IS_DG2(dev_priv)	IS_PLATFORM(dev_priv, INTEL_DG2)
#define IS_PONTEVECCHIO(dev_priv) IS_PLATFORM(dev_priv, INTEL_PONTEVECCHIO)
#define IS_METEORLAKE(dev_priv) IS_PLATFORM(dev_priv, INTEL_METEORLAKE)

#define IS_METEORLAKE_M(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_METEORLAKE, INTEL_SUBPLATFORM_M)
#define IS_METEORLAKE_P(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_METEORLAKE, INTEL_SUBPLATFORM_P)
#define IS_DG2_G10(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_DG2, INTEL_SUBPLATFORM_G10)
#define IS_DG2_G11(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_DG2, INTEL_SUBPLATFORM_G11)
#define IS_DG2_G12(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_DG2, INTEL_SUBPLATFORM_G12)
#define IS_ADLS_RPLS(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_ALDERLAKE_S, INTEL_SUBPLATFORM_RPL)
#define IS_ADLP_N(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_ALDERLAKE_P, INTEL_SUBPLATFORM_N)
#define IS_ADLP_RPLP(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_ALDERLAKE_P, INTEL_SUBPLATFORM_RPL)
#define IS_HSW_EARLY_SDV(dev_priv) (IS_HASWELL(dev_priv) && \
				    (INTEL_DEVID(dev_priv) & 0xFF00) == 0x0C00)
#define IS_BDW_ULT(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_BROADWELL, INTEL_SUBPLATFORM_ULT)
#define IS_BDW_ULX(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_BROADWELL, INTEL_SUBPLATFORM_ULX)
#define IS_BDW_GT3(dev_priv)	(IS_BROADWELL(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 3)
#define IS_HSW_ULT(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_HASWELL, INTEL_SUBPLATFORM_ULT)
#define IS_HSW_GT3(dev_priv)	(IS_HASWELL(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 3)
#define IS_HSW_GT1(dev_priv)	(IS_HASWELL(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 1)
/* ULX machines are also considered ULT. */
#define IS_HSW_ULX(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_HASWELL, INTEL_SUBPLATFORM_ULX)
#define IS_SKL_ULT(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_SKYLAKE, INTEL_SUBPLATFORM_ULT)
#define IS_SKL_ULX(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_SKYLAKE, INTEL_SUBPLATFORM_ULX)
#define IS_KBL_ULT(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_KABYLAKE, INTEL_SUBPLATFORM_ULT)
#define IS_KBL_ULX(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_KABYLAKE, INTEL_SUBPLATFORM_ULX)
#define IS_SKL_GT2(dev_priv)	(IS_SKYLAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 2)
#define IS_SKL_GT3(dev_priv)	(IS_SKYLAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 3)
#define IS_SKL_GT4(dev_priv)	(IS_SKYLAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 4)
#define IS_KBL_GT2(dev_priv)	(IS_KABYLAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 2)
#define IS_KBL_GT3(dev_priv)	(IS_KABYLAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 3)
#define IS_CFL_ULT(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_COFFEELAKE, INTEL_SUBPLATFORM_ULT)
#define IS_CFL_ULX(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_COFFEELAKE, INTEL_SUBPLATFORM_ULX)
#define IS_CFL_GT2(dev_priv)	(IS_COFFEELAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 2)
#define IS_CFL_GT3(dev_priv)	(IS_COFFEELAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 3)

#define IS_CML_ULT(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_COMETLAKE, INTEL_SUBPLATFORM_ULT)
#define IS_CML_ULX(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_COMETLAKE, INTEL_SUBPLATFORM_ULX)
#define IS_CML_GT2(dev_priv)	(IS_COMETLAKE(dev_priv) && \
				 INTEL_INFO(dev_priv)->gt == 2)

#define IS_ICL_WITH_PORT_F(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_ICELAKE, INTEL_SUBPLATFORM_PORTF)

#define IS_TGL_UY(dev_priv) \
	IS_SUBPLATFORM(dev_priv, INTEL_TIGERLAKE, INTEL_SUBPLATFORM_UY)

#define IS_SKL_GRAPHICS_STEP(p, since, until) (IS_SKYLAKE(p) && IS_GRAPHICS_STEP(p, since, until))

#define IS_KBL_GRAPHICS_STEP(dev_priv, since, until) \
	(IS_KABYLAKE(dev_priv) && IS_GRAPHICS_STEP(dev_priv, since, until))
#define IS_KBL_DISPLAY_STEP(dev_priv, since, until) \
	(IS_KABYLAKE(dev_priv) && IS_DISPLAY_STEP(dev_priv, since, until))

#define IS_JSL_EHL_GRAPHICS_STEP(p, since, until) \
	(IS_JSL_EHL(p) && IS_GRAPHICS_STEP(p, since, until))
#define IS_JSL_EHL_DISPLAY_STEP(p, since, until) \
	(IS_JSL_EHL(p) && IS_DISPLAY_STEP(p, since, until))

#define IS_TGL_DISPLAY_STEP(__i915, since, until) \
	(IS_TIGERLAKE(__i915) && \
	 IS_DISPLAY_STEP(__i915, since, until))

#define IS_RKL_DISPLAY_STEP(p, since, until) \
	(IS_ROCKETLAKE(p) && IS_DISPLAY_STEP(p, since, until))

#define IS_ADLS_DISPLAY_STEP(__i915, since, until) \
	(IS_ALDERLAKE_S(__i915) && \
	 IS_DISPLAY_STEP(__i915, since, until))

#define IS_ADLS_GRAPHICS_STEP(__i915, since, until) \
	(IS_ALDERLAKE_S(__i915) && \
	 IS_GRAPHICS_STEP(__i915, since, until))

#define IS_ADLP_DISPLAY_STEP(__i915, since, until) \
	(IS_ALDERLAKE_P(__i915) && \
	 IS_DISPLAY_STEP(__i915, since, until))

#define IS_ADLP_GRAPHICS_STEP(__i915, since, until) \
	(IS_ALDERLAKE_P(__i915) && \
	 IS_GRAPHICS_STEP(__i915, since, until))

#define IS_XEHPSDV_GRAPHICS_STEP(__i915, since, until) \
	(IS_XEHPSDV(__i915) && IS_GRAPHICS_STEP(__i915, since, until))

#define IS_MTL_GRAPHICS_STEP(__i915, variant, since, until) \
	(IS_SUBPLATFORM(__i915, INTEL_METEORLAKE, INTEL_SUBPLATFORM_##variant) && \
	 IS_GRAPHICS_STEP(__i915, since, until))

#define IS_MTL_DISPLAY_STEP(__i915, since, until) \
	(IS_METEORLAKE(__i915) && \
	 IS_DISPLAY_STEP(__i915, since, until))

#define IS_MTL_MEDIA_STEP(__i915, since, until) \
	(IS_METEORLAKE(__i915) && \
	 IS_MEDIA_STEP(__i915, since, until))

/*
 * DG2 hardware steppings are a bit unusual.  The hardware design was forked to
 * create three variants (G10, G11, and G12) which each have distinct
 * workaround sets.  The G11 and G12 forks of the DG2 design reset the GT
 * stepping back to "A0" for their first iterations, even though they're more
 * similar to a G10 B0 stepping and G10 C0 stepping respectively in terms of
 * functionality and workarounds.  However the display stepping does not reset
 * in the same manner --- a specific stepping like "B0" has a consistent
 * meaning regardless of whether it belongs to a G10, G11, or G12 DG2.
 *
 * TLDR:  All GT workarounds and stepping-specific logic must be applied in
 * relation to a specific subplatform (G10/G11/G12), whereas display workarounds
 * and stepping-specific logic will be applied with a general DG2-wide stepping
 * number.
 */
#define IS_DG2_GRAPHICS_STEP(__i915, variant, since, until) \
	(IS_SUBPLATFORM(__i915, INTEL_DG2, INTEL_SUBPLATFORM_##variant) && \
	 IS_GRAPHICS_STEP(__i915, since, until))

#define IS_DG2_DISPLAY_STEP(__i915, since, until) \
	(IS_DG2(__i915) && \
	 IS_DISPLAY_STEP(__i915, since, until))

#define IS_PVC_BD_STEP(__i915, since, until) \
	(IS_PONTEVECCHIO(__i915) && \
	 IS_BASEDIE_STEP(__i915, since, until))

#define IS_PVC_CT_STEP(__i915, since, until) \
	(IS_PONTEVECCHIO(__i915) && \
	 IS_GRAPHICS_STEP(__i915, since, until))

#define IS_LP(dev_priv)		(INTEL_INFO(dev_priv)->is_lp)
#define IS_GEN9_LP(dev_priv)	(GRAPHICS_VER(dev_priv) == 9 && IS_LP(dev_priv))
#define IS_GEN9_BC(dev_priv)	(GRAPHICS_VER(dev_priv) == 9 && !IS_LP(dev_priv))

#define __HAS_ENGINE(engine_mask, id) ((engine_mask) & BIT(id))
#define HAS_ENGINE(gt, id) __HAS_ENGINE((gt)->info.engine_mask, id)

#define ENGINE_INSTANCES_MASK(gt, first, count) ({		\
	unsigned int first__ = (first);					\
	unsigned int count__ = (count);					\
	((gt)->info.engine_mask &						\
	 GENMASK(first__ + count__ - 1, first__)) >> first__;		\
})
#define RCS_MASK(gt) \
	ENGINE_INSTANCES_MASK(gt, RCS0, I915_MAX_RCS)
#define BCS_MASK(gt) \
	ENGINE_INSTANCES_MASK(gt, BCS0, I915_MAX_BCS)
#define VDBOX_MASK(gt) \
	ENGINE_INSTANCES_MASK(gt, VCS0, I915_MAX_VCS)
#define VEBOX_MASK(gt) \
	ENGINE_INSTANCES_MASK(gt, VECS0, I915_MAX_VECS)
#define CCS_MASK(gt) \
	ENGINE_INSTANCES_MASK(gt, CCS0, I915_MAX_CCS)

#define HAS_MEDIA_RATIO_MODE(dev_priv) (INTEL_INFO(dev_priv)->has_media_ratio_mode)

#define HAS_LINK_COPY_ENGINES(dev_priv) (INTEL_INFO(dev_priv)->has_link_copy_engines)

/*
 * The Gen7 cmdparser copies the scanned buffer to the ggtt for execution
 * All later gens can run the final buffer from the ppgtt
 */
#define CMDPARSER_USES_GGTT(dev_priv) (GRAPHICS_VER(dev_priv) == 7)

#define HAS_LLC(dev_priv)	(INTEL_INFO(dev_priv)->has_llc)
#define HAS_4TILE(dev_priv)	(INTEL_INFO(dev_priv)->has_4tile)
#define HAS_SNOOP(dev_priv)	(INTEL_INFO(dev_priv)->has_snoop)
#define HAS_EDRAM(dev_priv)	((dev_priv)->edram_size_mb)
#define HAS_SECURE_BATCHES(dev_priv) (GRAPHICS_VER(dev_priv) < 6)
#define HAS_WT(dev_priv)	HAS_EDRAM(dev_priv)

#define HWS_NEEDS_PHYSICAL(dev_priv)	(INTEL_INFO(dev_priv)->hws_needs_physical)

#define HAS_LOGICAL_RING_CONTEXTS(dev_priv) \
		(INTEL_INFO(dev_priv)->has_logical_ring_contexts)
#define HAS_LOGICAL_RING_ELSQ(dev_priv) \
		(INTEL_INFO(dev_priv)->has_logical_ring_elsq)

#define HAS_EXECLISTS(dev_priv) HAS_LOGICAL_RING_CONTEXTS(dev_priv)

#define INTEL_PPGTT(dev_priv) (INTEL_INFO(dev_priv)->ppgtt_type)
#define HAS_PPGTT(dev_priv) \
	(INTEL_PPGTT(dev_priv) != INTEL_PPGTT_NONE)
#define HAS_FULL_PPGTT(dev_priv) \
	(INTEL_PPGTT(dev_priv) >= INTEL_PPGTT_FULL)
#define HAS_RECOVERABLE_PAGE_FAULT(dev_priv) \
	(INTEL_INFO(dev_priv)->has_recoverable_page_fault)

#define HAS_PAGE_SIZES(dev_priv, sizes) ({ \
	GEM_BUG_ON((sizes) == 0); \
	((sizes) & ~INTEL_INFO(dev_priv)->page_sizes) == 0; \
})

#define HAS_OVERLAY(dev_priv)		 (INTEL_INFO(dev_priv)->display.has_overlay)
#define OVERLAY_NEEDS_PHYSICAL(dev_priv) \
		(INTEL_INFO(dev_priv)->display.overlay_needs_physical)

/* Early gen2 have a totally busted CS tlb and require pinned batches. */
#define HAS_BROKEN_CS_TLB(dev_priv)	(IS_I830(dev_priv) || IS_I845G(dev_priv))

#define NEEDS_RC6_CTX_CORRUPTION_WA(dev_priv)	\
	(IS_BROADWELL(dev_priv) || GRAPHICS_VER(dev_priv) == 9)

/* WaRsDisableCoarsePowerGating:skl,cnl */
#define NEEDS_WaRsDisableCoarsePowerGating(dev_priv)			\
	(IS_SKL_GT3(dev_priv) || IS_SKL_GT4(dev_priv))

#define HAS_GMBUS_IRQ(dev_priv) (DISPLAY_VER(dev_priv) >= 4)
#define HAS_GMBUS_BURST_READ(dev_priv) (DISPLAY_VER(dev_priv) >= 11 || \
					IS_GEMINILAKE(dev_priv) || \
					IS_KABYLAKE(dev_priv))

/* With the 945 and later, Y tiling got adjusted so that it was 32 128-byte
 * rows, which changed the alignment requirements and fence programming.
 */
#define HAS_128_BYTE_Y_TILING(dev_priv) (GRAPHICS_VER(dev_priv) != 2 && \
					 !(IS_I915G(dev_priv) || IS_I915GM(dev_priv)))
#define SUPPORTS_TV(dev_priv)		(INTEL_INFO(dev_priv)->display.supports_tv)
#define I915_HAS_HOTPLUG(dev_priv)	(INTEL_INFO(dev_priv)->display.has_hotplug)

#define HAS_FW_BLC(dev_priv)	(DISPLAY_VER(dev_priv) > 2)
#define HAS_FBC(dev_priv)	(INTEL_INFO(dev_priv)->display.fbc_mask != 0)
#define HAS_CUR_FBC(dev_priv)	(!HAS_GMCH(dev_priv) && DISPLAY_VER(dev_priv) >= 7)

#define HAS_IPS(dev_priv)	(IS_HSW_ULT(dev_priv) || IS_BROADWELL(dev_priv))

#define HAS_DP_MST(dev_priv)	(INTEL_INFO(dev_priv)->display.has_dp_mst)
#define HAS_DP20(dev_priv)	(IS_DG2(dev_priv) || DISPLAY_VER(dev_priv) >= 14)

#define HAS_DOUBLE_BUFFERED_M_N(dev_priv)	(DISPLAY_VER(dev_priv) >= 9 || IS_BROADWELL(dev_priv))

#define HAS_CDCLK_CRAWL(dev_priv)	 (INTEL_INFO(dev_priv)->display.has_cdclk_crawl)
#define HAS_DDI(dev_priv)		 (INTEL_INFO(dev_priv)->display.has_ddi)
#define HAS_FPGA_DBG_UNCLAIMED(dev_priv) (INTEL_INFO(dev_priv)->display.has_fpga_dbg)
#define HAS_PSR(dev_priv)		 (INTEL_INFO(dev_priv)->display.has_psr)
#define HAS_PSR_HW_TRACKING(dev_priv) \
	(INTEL_INFO(dev_priv)->display.has_psr_hw_tracking)
#define HAS_PSR2_SEL_FETCH(dev_priv)	 (DISPLAY_VER(dev_priv) >= 12)
#define HAS_TRANSCODER(dev_priv, trans)	 ((INTEL_INFO(dev_priv)->display.cpu_transcoder_mask & BIT(trans)) != 0)

#define HAS_RC6(dev_priv)		 (INTEL_INFO(dev_priv)->has_rc6)
#define HAS_RC6p(dev_priv)		 (INTEL_INFO(dev_priv)->has_rc6p)
#define HAS_RC6pp(dev_priv)		 (false) /* HW was never validated */

#define HAS_RPS(dev_priv)	(INTEL_INFO(dev_priv)->has_rps)

#define HAS_DMC(dev_priv)	(INTEL_INFO(dev_priv)->display.has_dmc)

#define HAS_HECI_PXP(dev_priv) \
	(INTEL_INFO(dev_priv)->has_heci_pxp)

#define HAS_HECI_GSCFI(dev_priv) \
	(INTEL_INFO(dev_priv)->has_heci_gscfi)

#define HAS_HECI_GSC(dev_priv) (HAS_HECI_PXP(dev_priv) || HAS_HECI_GSCFI(dev_priv))

#define HAS_MSO(i915)		(DISPLAY_VER(i915) >= 12)

#define HAS_RUNTIME_PM(dev_priv) (INTEL_INFO(dev_priv)->has_runtime_pm)
#define HAS_64BIT_RELOC(dev_priv) (INTEL_INFO(dev_priv)->has_64bit_reloc)

#define HAS_SEMAPHORE_XEHPSDV(dev_priv) \
	(INTEL_INFO(dev_priv)->has_semaphore_xehpsdv)
#define HAS_OA_BPC_REPORTING(dev_priv) \
	(INTEL_INFO(dev_priv)->has_oa_bpc_reporting)
#define HAS_OA_SLICE_CONTRIB_LIMITS(dev_priv) \
	(INTEL_INFO(dev_priv)->has_oa_slice_contrib_limits)

/*
 * Set this flag, when platform requires 64K GTT page sizes or larger for
 * device local memory access. Also this flag implies that we require or
 * at least support the compact PT layout for the ppGTT when using the 64K
 * GTT pages.
 */
#define HAS_64K_PAGES(dev_priv) (INTEL_INFO(dev_priv)->has_64k_pages)
#define HAS_OA_BUF_128M(dev_priv) (INTEL_INFO(dev_priv)->has_oa_buf_128m)
#define HAS_OA_MMIO_TRIGGER(dev_priv) \
	(INTEL_INFO(dev_priv)->has_oa_mmio_trigger)
#define HAS_OAM(dev_priv) \
	(INTEL_INFO(dev_priv)->has_oam)
#define OAM_USES_VDBOX0_CHANNEL(dev_priv) \
	(INTEL_INFO(dev_priv)->oam_uses_vdbox0_channel)
#define HAS_OAC(dev_priv) (INTEL_INFO(dev_priv)->has_oac)

#define HAS_FULL_PS64(dev_priv) (INTEL_INFO(dev_priv)->has_full_ps64)

#define HAS_GMD_ID(i915)	INTEL_INFO(i915)->has_gmd_id

#define HAS_IPC(dev_priv)		 (INTEL_INFO(dev_priv)->display.has_ipc)

#define HAS_REGION(i915, i) (INTEL_INFO(i915)->memory_regions & (i))
#define HAS_LMEM(i915) HAS_REGION(i915, REGION_LMEM)
#define HAS_LMEM_MAX_BW(dev_priv) (INTEL_INFO(dev_priv)->has_lmem_max_bandwidth)
#define HAS_REMOTE_TILES(dev_priv)   (INTEL_INFO(dev_priv)->has_remote_tiles)
#define HAS_IAF(dev_priv)   (INTEL_INFO(dev_priv)->has_iaf)

#define HAS_EXTRA_GT_LIST(dev_priv)   (INTEL_INFO(dev_priv)->extra_gt_list)

/*
 * Platform has the dedicated compression control state for each lmem surfaces
 * stored in lmem to support the 3D and media compression formats.
 */
#define HAS_FLAT_CCS(dev_priv)   (INTEL_INFO(dev_priv)->has_flat_ccs)

#define HAS_UM_QUEUES(dev_priv) (INTEL_INFO(dev_priv)->has_um_queues)

#define HAS_NULL_PAGE(dev_priv) (INTEL_INFO(dev_priv)->has_null_page)

#define HAS_GT_UC(dev_priv)	(INTEL_INFO(dev_priv)->has_gt_uc)

#define HAS_SRIOV(dev_priv)	(INTEL_INFO(dev_priv)->has_sriov)

#define HAS_SELECTIVE_TLB_INVALIDATION(dev_priv) \
	(INTEL_INFO(dev_priv)->has_selective_tlb_invalidation)

#define HAS_POOLED_EU(dev_priv)	(INTEL_INFO(dev_priv)->has_pooled_eu)

#define HAS_GLOBAL_MOCS_REGISTERS(dev_priv)	(INTEL_INFO(dev_priv)->has_global_mocs)

#define HAS_PXP(dev_priv)  ((IS_ENABLED(CONFIG_DRM_I915_PXP) && \
			    INTEL_INFO(dev_priv)->has_pxp) && \
			    VDBOX_MASK(to_gt(dev_priv)))

#define HAS_MEMORY_IRQ_STATUS(dev_priv) \
	(INTEL_INFO(dev_priv)->has_iov_memirq && IS_SRIOV_VF(dev_priv))

#define HAS_GMCH(dev_priv) (INTEL_INFO(dev_priv)->display.has_gmch)

#define HAS_LSPCON(dev_priv) (IS_DISPLAY_VER(dev_priv, 9, 10))

#define HAS_L3_CCS_READ(i915) (INTEL_INFO(i915)->has_l3_ccs_read)

/* DPF == dynamic parity feature */
#define HAS_L3_DPF(dev_priv) (INTEL_INFO(dev_priv)->has_l3_dpf)
#define NUM_L3_SLICES(dev_priv) (IS_HSW_GT3(dev_priv) ? \
				 2 : HAS_L3_DPF(dev_priv))

#define GT_FREQUENCY_MULTIPLIER 50
#define GEN9_FREQ_SCALER 3

#define INTEL_NUM_PIPES(dev_priv) (hweight8(INTEL_INFO(dev_priv)->display.pipe_mask))

#define HAS_DISPLAY(dev_priv) (INTEL_INFO(dev_priv)->display.pipe_mask != 0)

#define HAS_VRR(i915)	(DISPLAY_VER(i915) >= 11)

#define HAS_ASYNC_FLIPS(i915)		(DISPLAY_VER(i915) >= 5)

/* Only valid when HAS_DISPLAY() is true */
#define INTEL_DISPLAY_ENABLED(dev_priv) \
	(drm_WARN_ON(&(dev_priv)->drm, !HAS_DISPLAY(dev_priv)),		\
	 !(dev_priv)->params.disable_display &&				\
	 !intel_opregion_headless_sku(dev_priv))

#define HAS_GUC_DEPRIVILEGE(dev_priv) \
	(INTEL_INFO(dev_priv)->has_guc_deprivilege)

#define HAS_D12_PLANE_MINIMIZATION(dev_priv) (IS_ROCKETLAKE(dev_priv) || \
					      IS_ALDERLAKE_S(dev_priv))

#define HAS_MBUS_JOINING(i915) (IS_ALDERLAKE_P(i915) || DISPLAY_VER(i915) >= 14)

#define HAS_3D_PIPELINE(i915)	(INTEL_INFO(i915)->has_3d_pipeline)

#define HAS_ONE_EU_PER_FUSE_BIT(i915)	(INTEL_INFO(i915)->has_one_eu_per_fuse_bit)

#define HAS_BAR2_SMEM_STOLEN(i915) (!HAS_LMEM(i915) && \
				    GRAPHICS_VER_FULL(i915) >= IP_VER(12, 70))
#define HAS_MEM_SPARING_SUPPORT(dev_priv) \
	(INTEL_INFO(dev_priv)->has_mem_sparing)

#define HAS_ASID_TLB_INVALIDATION(i915) \
	(INTEL_INFO(i915)->has_asid_tlb_invalidation)

#define HAS_SLIM_VDBOX(i915) (INTEL_INFO(i915)->has_slim_vdbox)

#define HAS_CACHE_CLOS(i915) (INTEL_INFO(i915)->has_cache_clos)

#define HAS_GUC_PROGRAMMABLE_MOCS(i915) (INTEL_INFO(i915)->has_guc_programmable_mocs)

#define HAS_EU_STALL_SAMPLING(i915) (INTEL_INFO(i915)->has_eu_stall_sampling)

#define HAS_GT_ERROR_VECTORS(i915) (INTEL_INFO(i915)->has_gt_error_vectors)

#define HAS_MEM_FENCE_SUPPORT(i915) ((i915)->params.enable_mem_fence && IS_PONTEVECCHIO((i915)))

#define HAS_LMTT_LVL2(i915) (INTEL_INFO(i915)->has_lmtt_lvl2)

static inline bool i915_has_svm(struct drm_i915_private *dev_priv)
{
#ifdef CONFIG_DRM_I915_SVM
	return HAS_RECOVERABLE_PAGE_FAULT(dev_priv);
#else
	return false;
#endif
}

static inline struct intel_gt *to_root_gt(struct drm_i915_private *i915)
{
	return &i915->gt0;
}

static inline struct intel_gt *to_gt(struct drm_i915_private *i915)
{
	if (HAS_REMOTE_TILES(i915) && IOV_MODE(i915) == I915_IOV_MODE_SRIOV_VF)
		return i915->gt[__ffs(to_root_gt(i915)->iov.vf.config.tile_mask)];

	return &i915->gt0;
}

static inline bool
intel_ggtt_needs_same_mem_type_within_cl_wa(struct drm_i915_private *dev_priv)
{
	/*
	 * Wa_14011411649:xehpsdv,dg2_g10
	 * We can't mix LMEM and SMEM allocation within the same GGTT
	 * cacheline (i.e. 16 PTEs, 64k address block).
	 */
	return IS_DG2_GRAPHICS_STEP(dev_priv, G10, STEP_A0, STEP_B0) ||
		IS_XEHPSDV(dev_priv);
}

/* i915_gem.c */
void i915_gem_init_early(struct drm_i915_private *dev_priv);
void i915_gem_cleanup_early(struct drm_i915_private *dev_priv);

struct intel_memory_region *i915_gem_shmem_setup(struct intel_gt *gt,
						 u16 type, u16 instance);

static inline void i915_gem_drain_freed_objects(struct drm_i915_private *i915)
{
	/*
	 * A single pass should suffice to release all the freed objects (along
	 * most call paths) , but be a little more paranoid in that freeing
	 * the objects does take a little amount of time, during which the rcu
	 * callbacks could have added new objects into the freed list, and
	 * armed the work again.
	 */
	while (atomic_read(&i915->mm.free_count)) {
		flush_work(&i915->mm.free_work);
		rcu_barrier();
	}
}

static inline void i915_gem_drain_workqueue(struct drm_i915_private *i915)
{
	/*
	 * Similar to objects above (see i915_gem_drain_freed-objects), in
	 * general we have workers that are armed by RCU and then rearm
	 * themselves in their callbacks. To be paranoid, we need to
	 * drain the workqueue a second time after waiting for the RCU
	 * grace period so that we catch work queued via RCU from the first
	 * pass. As neither drain_workqueue() nor flush_workqueue() report
	 * a result, we make an assumption that we only don't require more
	 * than 3 passes to catch all _recursive_ RCU delayed work.
	 *
	 */
	int pass = 3;
	do {
		flush_workqueue(i915->wq);
		rcu_barrier();
		i915_gem_drain_freed_objects(i915);
	} while (--pass);
	drain_workqueue(i915->wq);
}

struct i915_vma * __must_check
i915_gem_object_ggtt_pin_ww(struct drm_i915_gem_object *obj,
			    struct i915_gem_ww_ctx *ww,
			    struct i915_ggtt *ggtt,
			    const struct i915_ggtt_view *view,
			    u64 size, u64 alignment, u64 flags);

static inline struct i915_vma * __must_check
i915_gem_object_ggtt_pin(struct drm_i915_gem_object *obj,
			 struct i915_ggtt *ggtt,
			 const struct i915_ggtt_view *view,
			 u64 size, u64 alignment, u64 flags)
{
	return i915_gem_object_ggtt_pin_ww(obj, NULL, ggtt, view,
					   size, alignment, flags);
}

int i915_gem_object_unbind(struct drm_i915_gem_object *obj,
			   struct i915_gem_ww_ctx *ww,
			   unsigned long flags);
#define I915_GEM_OBJECT_UNBIND_ACTIVE BIT(0)
#define I915_GEM_OBJECT_UNBIND_BARRIER BIT(1)
#define I915_GEM_OBJECT_UNBIND_TEST BIT(2)
#define I915_GEM_OBJECT_UNBIND_KEEP_RESIDENT BIT(3)

void i915_gem_runtime_suspend(struct drm_i915_private *dev_priv);

struct drm_i915_gem_object *
i915_gem_object_create_user(struct drm_i915_private *i915, u64 size,
			    struct intel_memory_region **placements,
			    unsigned int n_placements);

int __must_check i915_gem_set_global_seqno(struct drm_device *dev, u32 seqno);

int __must_check i915_gem_init(struct drm_i915_private *dev_priv);
void i915_gem_driver_register(struct drm_i915_private *i915);
void i915_gem_driver_unregister(struct drm_i915_private *i915);
void i915_gem_driver_remove(struct drm_i915_private *dev_priv);
void i915_gem_driver_release(struct drm_i915_private *dev_priv);
int i915_gem_idle_engines(struct drm_i915_private *i915);
int i915_gem_resume_engines(struct drm_i915_private *i915);

int i915_gem_open(struct drm_i915_private *i915, struct drm_file *file);

static inline struct i915_gem_context *
__i915_gem_context_lookup_rcu(struct drm_i915_file_private *file_priv, u32 id)
{
	return xa_load(&file_priv->context_xa, id);
}

static inline struct i915_gem_context *
i915_gem_context_lookup(struct drm_i915_file_private *file_priv, u32 id)
{
	struct i915_gem_context *ctx;

	rcu_read_lock();
	ctx = __i915_gem_context_lookup_rcu(file_priv, id);
	if (ctx && !kref_get_unless_zero(&ctx->ref))
		ctx = NULL;
	rcu_read_unlock();

	return ctx;
}

static inline struct i915_address_space *
__i915_address_space_lookup_rcu(struct drm_i915_file_private *file_priv,
				u32 id)
{
	return xa_load(&file_priv->vm_xa, id);
}

static inline struct i915_address_space *
i915_address_space_lookup(struct drm_i915_file_private *file_priv,
			  u32 id)
{
	struct i915_address_space *vm;

	rcu_read_lock();
	vm = __i915_address_space_lookup_rcu(file_priv, id);
	if (vm)
		vm = i915_vm_tryget(vm);
	rcu_read_unlock();

	return vm;
}

/* intel_device_info.c */
static inline struct intel_device_info *
mkwrite_device_info(struct drm_i915_private *dev_priv)
{
	return (struct intel_device_info *)INTEL_INFO(dev_priv);
}

static inline enum i915_map_type
i915_coherent_map_type(struct drm_i915_private *i915,
		       struct drm_i915_gem_object *obj, bool always_coherent)
{
	/*
	 * Wa_22016122933: FIXME: always return I915_MAP_WC for MTL
	 * Issue is expected to be tracked as Wa_22016122933, but not
	 * finalized by hardware team yet. So marked as fixme for now.
	 */
	if (i915_gem_object_is_lmem(obj) || IS_METEORLAKE(i915))
		return I915_MAP_WC;
	if (HAS_LLC(i915) || always_coherent)
		return I915_MAP_WB;
	else
		return I915_MAP_WC;
}

static inline void i915_write_barrier(struct drm_i915_private *i915)
{
	/*
	 * We need a MMIO write as barrier to lmem. A following lmem write
	 * will stall until all previous writes are pushed to lmem.
	 *
	 * We don't need to use a local uncore. A mmio write to a root
	 * tile register also ensures remote tile lmem writes are
	 * pushed.
	 */
	raw_reg_write(i915->uncore.regs, SOFTWARE_FLAGS_SPR33, 0);
}

void i915_silent_driver_error(struct drm_i915_private *i915,
			      const enum i915_driver_errors error);

__printf(3, 4)
void i915_log_driver_error(struct drm_i915_private *i915,
			   const enum i915_driver_errors error,
			   const char *fmt, ...);

/* Below are various work arounds available for the pcie deadlock issue:
 * Each WA can be enabled by setting corresponding bit in module parameter.
 * Note that I915_WA_IDLE_GPU_BEFORE_UPDATE & I915_WA_USE_FLAT_PPGTT_UPDATE cannot
 * co-exist, so the only legal values for module parameter smem_access_control are
 * 0, 1, 2, 3 and 5. But 3 and 5 are recommended values to avoid pcie deadlock.
 *
 * When I915_WA_FORCE_SMEM_OBJECT is enabled, certain objects like(lrc, hwsp, ringbuffer)
 * are created in smem instead of creating in lmem to avoid access from host while GPU
 * access smem to  * avoid the possibility of pcie deadlock.
 *
 * When I915_WA_IDLE_GPU_BEFORE_UPDATE is enabled, engines are stalled completely before
 * writing to page tables which are located in lmem, to avoid the possibility of deadlock
 * due to simultaneous traffic from host to lmem and traffice from GPU to access smem.
 *
 * When I915_WA_USE_FLAT_PPGTT_UPDATE, i915 makes use of blitter engine to update the
 * page tables. Also note this mechanism of updating page tables cannot be used during
 * driver load since resources required are not yet initialized when driver loading is
 * started, bind_ctxt_ready is used to indicate when i915 ready to use blitter commands
 * to update page tables.
 */
#define I915_WA_FORCE_SMEM_OBJECT	0
#define I915_WA_IDLE_GPU_BEFORE_UPDATE	1
#define I915_WA_USE_FLAT_PPGTT_UPDATE	2

/*
 * Default WA is Level-4 which set with during driver load if the
 * module parameter value is DEFAULT value (256).
 */
#define I915_SMEM_ACCESS_CONTROL_DEFAULT	256

static inline bool
i915_is_mem_wa_enabled(struct drm_i915_private *i915, int val)
{
	return i915->params.smem_access_control & BIT(val);
}
static inline bool i915_allows_overcommit(const struct drm_i915_private *i915)
{
	/* If either acct_limit[] is non-zero, both are non-zero */
	return !i915->mm.user_acct_limit[INTEL_MEMORY_OVERCOMMIT_LMEM];
}

static inline void
i915_pci_error_set_fault(struct drm_i915_private *i915)
{
	WRITE_ONCE(i915->device_faulted, true);
}

static inline void
i915_pci_error_clear_fault(struct drm_i915_private *i915)
{
	WRITE_ONCE(i915->device_faulted, false);
}

static inline bool
i915_is_pci_faulted(const struct drm_i915_private *i915)
{
	return READ_ONCE(i915->device_faulted);
}

#endif
