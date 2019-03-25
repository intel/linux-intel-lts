# SPDX-License-Identifier: GPL-2.0
snd-soc-skl-virtio-sst-objs := skl-virtio-sst.o
snd-soc-skl-virtio-card-objs := skl-virtio-card.o
snd-soc-skl-virtio-fe-objs := skl-virtio-kctl.o skl-virtio-fe.o

obj-$(CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_FE) := snd-soc-skl-virtio-sst.o \
snd-soc-skl-virtio-fe.o snd-soc-skl-virtio-card.o

snd-soc-skl-virtio-be-objs := skl-virt-audio.o skl-virtio-kctl.o skl-virtio-be.o
obj-$(CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_BE) += snd-soc-skl-virtio-be.o
