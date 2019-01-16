snd-soc-skl-virtio-sst-objs := virtio/skl-virtio-sst.o
snd-soc-skl-virtio-card-objs := virtio/skl-virtio-card.o
snd-soc-skl-virtio-fe-objs := virtio/skl-virtio-fe.o

obj-$(CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_FE) := snd-soc-skl-virtio-sst.o \
snd-soc-skl-virtio-fe.o snd-soc-skl-virtio-card.o

ifdef CONFIG_SND_SOC_INTEL_SKYLAKE_VIRTIO_BE
snd-soc-skl-objs += virtio/skl-virtio-be.o virtio/skl-virtio-miscdev.o
endif
