/*
 *  skl.h - HD Audio skylake defintions.
 *
 *  Copyright (C) 2015 Intel Corp
 *  Author: Jeeja KP <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#ifndef __SOUND_SOC_SKL_H
#define __SOUND_SOC_SKL_H

#include <sound/hda_register.h>
#include <sound/hdaudio_ext.h>
#include "skl-nhlt.h"
#include "skl-sst-ipc.h"

#define SKL_SUSPEND_DELAY 2000

#define AZX_PCIREG_PGCTL		0x44
#define AZX_PGCTL_LSRMD_MASK		(1 << 4)
#define AZX_PCIREG_CGCTL		0x48
#define AZX_CGCTL_MISCBDCGE_MASK	(1 << 6)
/* D0I3C Register fields */
#define AZX_REG_VS_D0I3C_CIP      0x1 /* Command in progress */
#define AZX_REG_VS_D0I3C_I3       0x4 /* D0i3 enable */

#define SKL_MAX_MODULE_RESOURCES 8
#define SKL_MAX_MODULE_FORMATS 8
#define SKL_MAX_IN_QUEUE 8
#define SKL_MAX_OUT_QUEUE 8
#define SKL_MAX_LL_SRC_CFG  8
#define SKL_MAX_DMA_CFG    24

struct skl_dsp_resource {
	u32 max_mcps;
	u32 max_mem;
	u32 mcps;
	u32 mem;
};

struct skl_debug;
struct snd_soc_dapm_widget;

struct skl_module_fmt {
	u32 channels;
	u32 s_freq;
	u32 bit_depth;
	u32 valid_bit_depth;
	u32 ch_cfg;
	u32 interleaving_style;
	u32 sample_type;
	u32 ch_map;
};

struct skl_module_pin_fmt {
	u8 pin_id;
	struct skl_module_fmt pin_fmt;
};

struct skl_module_intf {
	u8 fmt_idx;
	u8 nr_input_fmt;
	u8 nr_output_fmt;
	struct skl_module_pin_fmt input[SKL_MAX_IN_QUEUE];
	struct	skl_module_pin_fmt output[SKL_MAX_OUT_QUEUE];
};

struct skl_module_pin_resources {
	u8 pin_index;
	u32 buf_size;
};

struct skl_module_res {
	u8 res_idx;
	u32 is_pages;
	u32 cps;
	u32 ibs;
	u32 obs;
	u32 dma_buffer_size;
	u32 cpc;
	u32 mod_flags;
	u32 obls;
	u8 nr_input_pins;
	u8 nr_output_pins;
	struct skl_module_pin_resources input[SKL_MAX_IN_QUEUE];
	struct skl_module_pin_resources output[SKL_MAX_OUT_QUEUE];
};

struct skl_module {
	u16 major_version;
	u16 minor_version;
	u16 hotfix_version;
	u16 build_version;
	uuid_le uuid;
	u8 loadable;
	u8 input_pin_type;
	u8 output_pin_type;
	u8 auto_start;
	u8 max_input_pins;
	u8 max_output_pins;
	u8 max_instance_count;
	char library_name[SKL_LIB_NAME_LENGTH];
	u8 nr_resources;
	u8 nr_interfaces;
	struct skl_module_res resources[SKL_MAX_MODULE_RESOURCES];
	struct skl_module_intf formats[SKL_MAX_MODULE_FORMATS];
};

struct skl_dma_config {
	u32 min_size;
	u32 max_size;
} __packed;

struct skl_mem_status {
	u32 type;
	u32 size;
	u32 mem_reclaim;
} __packed;

struct skl_dsp_freq {
	u32 type;
	u32 size;
	u32 freq;
} __packed;

struct skl_dma_buff_cfg {
	u32 type;
	u32 size;
	struct skl_dma_config dma_cfg[SKL_MAX_DMA_CFG];
} __packed;

struct skl_sch_config {
	u32 type;
	u32 length;
	u32 sys_tick_mul;
	u32 sys_tick_div;
	u32 ll_src;
	u32 num_cfg;
	u32 node_info[SKL_MAX_LL_SRC_CFG];
} __packed;

struct skl_fw_cfg_info {
	struct skl_mem_status mem_sts;
	struct skl_dsp_freq slw_frq;
	struct skl_dsp_freq fst_frq;
	struct skl_dma_buff_cfg dmacfg;
	struct skl_sch_config sch_cfg;
} __packed;

struct skl {
	struct hdac_ext_bus ebus;
	struct pci_dev *pci;

	unsigned int init_done:1; /* delayed init status */
	struct platform_device *dmic_dev;
	struct platform_device *i2s_dev;
	struct snd_soc_platform *platform;

	struct nhlt_acpi_table *nhlt; /* nhlt ptr */
	struct skl_sst *skl_sst; /* sst skl ctx */

	struct skl_dsp_resource resource;
	struct list_head ppl_list;
	struct list_head bind_list;

	const char *fw_name;
	char tplg_name[64];
	unsigned short pci_id;
	const struct firmware *tplg;

	int supend_active;

	struct work_struct probe_work;
	struct skl_debug *debugfs;
	bool nhlt_override;
	bool mod_set_get_status;
	struct skl_fw_cfg_info cfg;
	u8 nr_modules;
	u8 conf_version;
	struct skl_module *modules;
};

#define skl_to_ebus(s)	(&(s)->ebus)
#define ebus_to_skl(sbus) \
	container_of(sbus, struct skl, sbus)

/* to pass dai dma data */
struct skl_dma_params {
	u32 format;
	u8 stream_tag;
};

/* to pass dmic data */
struct skl_machine_pdata {
	u32 dmic_num;
};

struct skl_dsp_ops {
	int id;
	unsigned int num_cores;
	struct skl_dsp_loader_ops (*loader_ops)(void);
	int (*init)(struct device *dev, void __iomem *mmio_base, int irq,
			const char *fw_name,
			struct skl_dsp_loader_ops loader_ops,
			struct skl_sst **skl_sst, void *ptr);
	int (*init_fw)(struct device *dev, struct skl_sst *ctx);
	void (*cleanup)(struct device *dev, struct skl_sst *ctx);
};

int skl_platform_unregister(struct device *dev);
int skl_platform_register(struct device *dev);

struct nhlt_acpi_table *skl_nhlt_init(struct device *dev);
void skl_nhlt_free(struct nhlt_acpi_table *addr);
struct nhlt_specific_cfg *skl_get_ep_blob(struct skl *skl, u32 instance,
					u8 link_type, u8 s_fmt, u8 no_ch,
					u32 s_rate, u8 dirn, u8 dev_type);

int skl_get_dmic_geo(struct skl *skl);
int skl_nhlt_update_topology_bin(struct skl *skl);
int skl_init_dsp(struct skl *skl);
int skl_free_dsp(struct skl *skl);
int skl_suspend_late_dsp(struct skl *skl);
int skl_suspend_dsp(struct skl *skl);
int skl_resume_dsp(struct skl *skl);
void skl_cleanup_resources(struct skl *skl);
const struct skl_dsp_ops *skl_get_dsp_ops(int pci_id);
void skl_update_d0i3c(struct device *dev, bool enable);
int skl_nhlt_create_sysfs(struct skl *skl);
void skl_nhlt_remove_sysfs(struct skl *skl);

struct skl_module_cfg;

#ifdef CONFIG_DEBUG_FS

struct skl_debug *skl_debugfs_init(struct skl *skl);
void skl_debugfs_exit(struct skl_debug *d);
struct nhlt_specific_cfg
*skl_nhlt_get_debugfs_blob(struct skl_debug *d, u8 link_type, u32 instance,
			u8 stream);
void skl_debug_init_module(struct skl_debug *d,
			struct snd_soc_dapm_widget *w,
			struct skl_module_cfg *mconfig);
#else

struct skl_debug {
}

struct skl_debug *skl_debugfs_init(struct skl *skl)
{
	return NULL;
}

void skl_debugfs_exit(struct skl_debug *d)
{
}

struct nhlt_specific_cfg
*skl_nhlt_get_debugfs_blob(struct skl_debug *d, u8 link_type, u32 instance, u8 stream)
{
	return NULL;
}

void skl_debug_init_module(struct skl_debug *d,
			struct snd_soc_dapm_widget *w,
			struct skl_module_cfg *mconfig)
{
}

#endif

#endif /* __SOUND_SOC_SKL_H */
