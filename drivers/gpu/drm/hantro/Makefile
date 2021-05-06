# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Hantro DRM media codec driver. This driver provides
# support for Keem Bay Hantro VPU IP.
hantro-objs := hantro_drv.o hantro_fs.o hantro_drm.o hantro_dmabuf.o hantro_enc.o hantro_dec.o hantro_fence.o hantro_cache.o hantro_dec400.o hantro_devicemgr.o hantro_cooling.o hantro_metadata.o
hantro-$(CONFIG_DRM_HANTRO_TRACEPOINTS) += trace.o
obj-$(CONFIG_DRM_HANTRO_UNIFY) += hantro.o
CFLAGS_trace.o := -I$(src)
