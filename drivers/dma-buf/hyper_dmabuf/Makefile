obj-y := hyper_dmabuf_drv.o \
	 hyper_dmabuf_ioctl.o \
	 hyper_dmabuf_list.o \
	 hyper_dmabuf_sgl_proc.o \
	 hyper_dmabuf_ops.o \
	 hyper_dmabuf_msg.o \
	 hyper_dmabuf_id.o \
	 hyper_dmabuf_remote_sync.o \
	 hyper_dmabuf_query.o \

ifeq ($(CONFIG_HYPER_DMABUF_EVENT_GEN), y)
	obj-y += hyper_dmabuf_event.o
endif

ifeq ($(CONFIG_HYPER_DMABUF_XEN), y)
	obj-y += xen/hyper_dmabuf_xen_comm.o \
		 xen/hyper_dmabuf_xen_comm_list.o \
		 xen/hyper_dmabuf_xen_shm.o \
		 xen/hyper_dmabuf_xen_drv.o
else ifeq ($(CONFIG_HYPER_DMABUF_ACRN), y)
	ifeq ($(CONFIG_HYPER_DMABUF_VIRTIO_BE), y)
		obj-y += virtio/hyper_dmabuf_virtio_be_drv.o \
			 virtio/hyper_dmabuf_virtio_fe_list.o
	else
		obj-y += virtio/hyper_dmabuf_virtio_fe_drv.o
	endif
		obj-y += virtio/hyper_dmabuf_virtio_common.o \
			 virtio/hyper_dmabuf_virtio_shm.o \
			 virtio/hyper_dmabuf_virtio_comm_ring.o
endif
