/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Hantro driver trace header.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#if defined(CONFIG_DRM_HANTRO_TRACEPOINTS)
#if !defined(__HANTRO_DRIVER_TRACE) || defined(TRACE_HEADER_MULTI_READ)
#define __HANTRO_DRIVER_TRACE
#include <linux/tracepoint.h>
#undef TRACE_SYSTEM
#define TRACE_SYSTEM hantro
#define START_TIME u64 start = sched_clock()
#define MAX_MSG_LEN 120

TRACE_EVENT(drm_file_open,
	    TP_PROTO(void *drmdev, void *file),
	    TP_ARGS(drmdev, file),
	    TP_STRUCT__entry(__field(void *, drmdev)
			     __field(void *, file)),
	    TP_fast_assign(__entry->drmdev = drmdev;
			   __entry->file = file;),
	    TP_printk("drmdev = %p, file = %p", __entry->drmdev,
		      __entry->file));

TRACE_EVENT(drm_file_close,
	    TP_PROTO(void *drmdev, void *file),
	    TP_ARGS(drmdev, file),
	    TP_STRUCT__entry(__field(void *, drmdev)
			     __field(void *, file)),
	    TP_fast_assign(__entry->drmdev = drmdev;
			   __entry->file = file;),
	    TP_printk("drmdev = %p, file = %p", __entry->drmdev,
		      __entry->file));

TRACE_EVENT(client_add,
	    TP_PROTO(void *file_attr, int deviceidx, int clientid, int codec,
		     int profile, unsigned long width, unsigned long height),
	    TP_ARGS(file_attr, deviceidx, clientid, codec, profile, width,
		    height),
	    TP_STRUCT__entry(__field(void *, file_attr)
			     __field(int, deviceidx)
			     __field(int, clientid)
			     __field(int, codec)
			     __field(int, profile)
			     __field(unsigned long, width)
			     __field(unsigned long, height)),
	    TP_fast_assign(__entry->file_attr = file_attr;
			   __entry->deviceidx = deviceidx;
			   __entry->clientid = clientid;
			   __entry->codec = codec;
			   __entry->profile = profile;
			   __entry->width = width;
			   __entry->height = height;),
	    TP_printk("file_attr %p, deviceidx = %d, clientid = %x, type = %d, profile = %d, width = %ld, height = %ld",
		      __entry->file_attr, __entry->deviceidx,
		      __entry->clientid, __entry->codec, __entry->profile,
		      __entry->width, __entry->height));

TRACE_EVENT(client_remove,
	    TP_PROTO(void *file_attr, int clientid),
	    TP_ARGS(file_attr, clientid),
	    TP_STRUCT__entry(__field(void *, file_attr)
			     __field(int, clientid)),
	    TP_fast_assign(__entry->file_attr = file_attr;
			   __entry->clientid = clientid;),
	    TP_printk("file_attr %p, clientid = %x", __entry->file_attr,
		      __entry->clientid));

TRACE_EVENT(hantro_cma_alloc,
	    TP_PROTO(int deviceidx, int region, void *paddr, int handle,
		     int size),
	    TP_ARGS(deviceidx, region, paddr, handle, size),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(int, region)
			     __field(void *, paddr)
			     __field(int, handle)
			     __field(int, size)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->region = region;
			   __entry->paddr = paddr;
			   __entry->handle = handle;
			   __entry->size = size;),
	    TP_printk("deviceidx = %d, region = %d, paddr = %p, handle = %-3d, size = %d",
		      __entry->deviceidx, __entry->region, __entry->paddr,
		      __entry->handle, __entry->size));

TRACE_EVENT(hantro_cma_free,
	    TP_PROTO(int deviceidx, void *paddr, int handle),
	    TP_ARGS(deviceidx, paddr, handle),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(void *, paddr)
			     __field(int, handle)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->paddr = paddr;
			   __entry->handle = handle;),
	    TP_printk("deviceidx = %d, paddr = %p, handle = %-3d,",
		      __entry->deviceidx,
		      __entry->paddr,
		      __entry->handle));

TRACE_EVENT(dec_reserve,
	    TP_PROTO(int deviceidx, int core, int waittime),
	    TP_ARGS(deviceidx, core, waittime),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(int, core)
			     __field(int, waittime)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->core = core;
			   __entry->waittime = waittime;),
	    TP_printk("deviceidx = %d, core = %d, waittime = %d us",
		      __entry->deviceidx, __entry->core, __entry->waittime));

TRACE_EVENT(dec_release,
	    TP_PROTO(int deviceidx, int core),
	    TP_ARGS(deviceidx, core),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(int, core)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->core = core;),
	    TP_printk("deviceidx = %d, core = %d", __entry->deviceidx,
		      __entry->core));

TRACE_EVENT(enc_reserve,
	    TP_PROTO(int deviceidx, int core, int waittime),
	    TP_ARGS(deviceidx, core, waittime),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(int, core)
			     __field(int, waittime)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->core = core;
			   __entry->waittime = waittime;),
	    TP_printk("deviceidx = %d, core = %d, waittime = %d us",
		      __entry->deviceidx, __entry->core, __entry->waittime));

TRACE_EVENT(enc_release,
	    TP_PROTO(int deviceidx, int core),
	    TP_ARGS(deviceidx, core),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(int, core)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->core = core;),
	    TP_printk("deviceidx = %d, core = %d", __entry->deviceidx,
		      __entry->core));

TRACE_EVENT(cache_reserve,
	    TP_PROTO(int deviceidx, int waittime),
	    TP_ARGS(deviceidx, waittime),
	    TP_STRUCT__entry(__field(int, deviceidx)
			     __field(int, waittime)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;
			   __entry->waittime = waittime;),
	    TP_printk("deviceidx = %d, waittime = %d us", __entry->deviceidx,
		      __entry->waittime));

TRACE_EVENT(cache_release,
	    TP_PROTO(int deviceidx),
	    TP_ARGS(deviceidx),
	    TP_STRUCT__entry(__field(int, deviceidx)),
	    TP_fast_assign(__entry->deviceidx = deviceidx;),
	    TP_printk("deviceidx = %d", __entry->deviceidx));

TRACE_EVENT(fence_acquirebuf,
	    TP_PROTO(void *obj, int handle, int fence_handle, int waittime,
		     int ret),
	    TP_ARGS(obj, handle, fence_handle, waittime, ret),
	    TP_STRUCT__entry(__field(void *, obj)
			     __field(int, handle)
			     __field(int, fence_handle)
			     __field(int, waittime)
			     __field(int, ret)),
	    TP_fast_assign(__entry->obj = obj;
			   __entry->handle = handle;
			   __entry->fence_handle = fence_handle;
			   __entry->waittime = waittime;
			   __entry->ret = ret;),
	    TP_printk("obj = %p, handle = %-3d, fence = %-3d, waittime = %-4d us, ret = %d",
		      __entry->obj, __entry->handle, __entry->fence_handle,
		      __entry->waittime, __entry->ret));

TRACE_EVENT(fence_releasebuf,
	    TP_PROTO(int fence_handle, int ret),
	    TP_ARGS(fence_handle, ret),
	    TP_STRUCT__entry(__field(int, fence_handle)
			     __field(int, ret)),
	    TP_fast_assign(__entry->fence_handle = fence_handle;
			   __entry->ret = ret;),
	    TP_printk("fence = %-3d, ret = %d", __entry->fence_handle,
		      __entry->ret));

TRACE_EVENT(prime_handle_to_fd,
	    TP_PROTO(void *obj, int handle, int fd, int ret),
	    TP_ARGS(obj, handle, fd, ret),
	    TP_STRUCT__entry(__field(void *, obj)
			     __field(int, handle)
			     __field(int, fd)
			     __field(int, ret)),
	    TP_fast_assign(__entry->obj = obj;
			   __entry->handle = handle;
			   __entry->fd = fd;
			   __entry->ret = ret;),
	    TP_printk("obj = %p, handle = %-3d, fd = %-3d, ret = %d",
		      __entry->obj, __entry->handle, __entry->fd,
		      __entry->ret));

TRACE_EVENT(prime_fd_to_handle,
	    TP_PROTO(int fd, int handle, int ret),
	    TP_ARGS(fd, handle, ret),
	    TP_STRUCT__entry(__field(int, fd)
			     __field(int, handle)
			     __field(int, ret)),
	    TP_fast_assign(__entry->fd = fd;
			   __entry->handle = handle;
			   __entry->ret = ret;),
	    TP_printk("fd = %-3d, handle = %-3d, ret = %d", __entry->fd,
		      __entry->handle, __entry->ret));

TRACE_EVENT(prime_dmabuf_export,
	    TP_PROTO(void *paddr, int handle, void *dmabuf),
	    TP_ARGS(paddr, handle, dmabuf),
	    TP_STRUCT__entry(__field(void *, paddr)
			     __field(int, handle)
			     __field(void *, dmabuf)),
	    TP_fast_assign(__entry->paddr = paddr;
			   __entry->handle = handle;
			   __entry->dmabuf = dmabuf;),
	    TP_printk("paddr = %p, handle = %-3d, dmabuf = %p", __entry->paddr,
		      __entry->handle, __entry->dmabuf));

TRACE_EVENT(prime_dmabuf_import,
	    TP_PROTO(void *dmabuf, void *gem_obj),
	    TP_ARGS(dmabuf, gem_obj),
	    TP_STRUCT__entry(__field(void *, dmabuf)
			     __field(void *, gem_obj)),
	    TP_fast_assign(__entry->dmabuf = dmabuf;
			   __entry->gem_obj = gem_obj;),
	    TP_printk("dmabuf = %p, gemobj = %p", __entry->dmabuf,
		      __entry->gem_obj));

TRACE_EVENT(prime_dmabuf_put,
	    TP_PROTO(void *paddr, void *dmabuf, int fd),
	    TP_ARGS(paddr, dmabuf, fd),
	    TP_STRUCT__entry(__field(void *, paddr)
			     __field(void *, dmabuf)
			     __field(int, fd)),
	    TP_fast_assign(__entry->paddr = paddr;
			   __entry->dmabuf = dmabuf;
			   __entry->fd = fd;),
	    TP_printk("paddr = %p, dmabuf = %p", __entry->paddr,
		      __entry->dmabuf));

TRACE_EVENT(prime_drm_dmabuf_release,
	    TP_PROTO(void *dmabuf),
	    TP_ARGS(dmabuf),
	    TP_STRUCT__entry(__field(void *, dmabuf)),
	    TP_fast_assign(__entry->dmabuf = dmabuf;),
	    TP_printk("dmabuf = %p", __entry->dmabuf));

TRACE_EVENT(gem_handle_create,
	    TP_PROTO(int handle),
	    TP_ARGS(handle),
	    TP_STRUCT__entry(__field(int, handle)),
	    TP_fast_assign(__entry->handle = handle;),
	    TP_printk("handle = %-3d,", __entry->handle));

TRACE_EVENT(gem_handle_delete,
	    TP_PROTO(int handle),
	    TP_ARGS(handle),
	    TP_STRUCT__entry(__field(int, handle)),
	    TP_fast_assign(__entry->handle = handle;),
	    TP_printk("handle = %-3d,", __entry->handle));

TRACE_EVENT(hantro_msg,
	    TP_PROTO(struct va_format *vaf),
	    TP_ARGS(vaf),
	    TP_STRUCT__entry(__dynamic_array(char, msg, MAX_MSG_LEN)),
	    TP_fast_assign(WARN_ON_ONCE(vsnprintf(__get_dynamic_array(msg),
						  MAX_MSG_LEN, vaf->fmt,
						  *vaf->va) >= MAX_MSG_LEN);),
	    TP_printk("%s", __get_str(msg)));

TRACE_EVENT(hantro_err,
	    TP_PROTO(struct va_format *vaf),
	    TP_ARGS(vaf),
	    TP_STRUCT__entry(__dynamic_array(char, msg, MAX_MSG_LEN)),
	    TP_fast_assign(WARN_ON_ONCE(vsnprintf(__get_dynamic_array(msg),
						  MAX_MSG_LEN, vaf->fmt,
						  *vaf->va) >= MAX_MSG_LEN);),
	    TP_printk("%s", __get_str(msg)));

#endif /* !__HANTRO_DRIVER_TRACE || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>

void __trace_hantro_msg(const char *fmt, ...);
void __trace_hantro_err(const char *fmt, ...);
#else

#define START_TIME
#define trace_hantro_msg(vaf)
#define trace_hantro_err(vaf)
#define trace_drm_file_open(drmdev, file)
#define trace_drm_file_close(drmdev, file)
#define trace_client_add(file_attr, deviceidx, clientid, codec, profile,       \
			 width, height)
#define trace_client_remove(file_attr, clientid)
#define trace_hantro_cma_alloc(deviceidx, region, paddr, handle, size)
#define trace_hantro_cma_free(deviceidx, paddr, handle)
#define trace_dec_reserve(deviceidx, core, waittime)
#define trace_dec_release(deviceidx, core)
#define trace_enc_reserve(deviceidx, core, waittime)
#define trace_enc_release(deviceidx, core)
#define trace_cache_reserve(deviceidx, waittime)
#define trace_cache_release(deviceidx)
#define trace_fence_acquirebuf(obj, handle, fence_handle, waittime, ret)
#define trace_fence_releasebuf(fence_handle, ret)
#define trace_prime_handle_to_fd(obj, handle, fd, ret)
#define trace_prime_fd_to_handle(fd, handle, ret)
#define trace_gem_handle_create(handle)
#define trace_gem_handle_delete(handle)
#define trace_prime_dmabuf_import(dev, file)
#define trace_prime_dmabuf_export(obj, dev, file)
#define trace_prime_dmabuf_put(paddr, dmabuf, fd)
#define trace_prime_drm_dmabuf_release(dmabuf)
#define __trace_hantro_msg(fmt, args...)
#define __trace_hantro_err(fmt, args...)
#endif
