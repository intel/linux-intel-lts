/* SPDX-License-Identifier: GPL-2.0
 *
 * skl-virtio.h -- SKL Virtio driver header
 *
 * Copyright (C) 2018 Intel Corporation.
 *
 * definitions/declarations for skl virtio FE/BE driver
 */

#ifndef __SOUND_SOC_SKL_VIRTIO_H
#define __SOUND_SOC_SKL_VIRTIO_H

#if IS_ENABLED(CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_FE)
struct skl_sst;
struct skl_dsp_loader_ops;

extern int vfe_sst_dsp_init(struct device *dev, void __iomem *mmio_base,
		int irq, const char *fw_name, struct skl_dsp_loader_ops dsp_ops,
		struct skl_sst **dsp, void *ptr);
extern int vfe_sst_init_fw(struct device *dev, struct skl_sst *ctx);
extern void vfe_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx);

#endif

void kctl_notify_machine_ready(struct snd_soc_card *card);

#endif //__SOUND_SOC_SKL_VIRTIO_H

