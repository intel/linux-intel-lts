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

#if IS_ENABLED(CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_BE)

extern int snd_soc_skl_virtio_miscdev_register(struct skl *sdev);
extern void skl_notify_stream_update(struct hdac_bus *bus,
		struct snd_pcm_substream *substr);
#else

#define snd_soc_skl_virtio_miscdev_register(...) {}
#define skl_notify_stream_update(...) {}

#endif

#if IS_ENABLED(CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_FE)

extern int vfe_sst_dsp_init(struct device *dev, void __iomem *mmio_base,
		int irq, const char *fw_name, struct skl_dsp_loader_ops dsp_ops,
		struct skl_sst **dsp, void *ptr);
extern int vfe_sst_init_fw(struct device *dev, struct skl_sst *ctx);
extern void vfe_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx);

#endif

#endif //__SOUND_SOC_SKL_VIRTIO_H
