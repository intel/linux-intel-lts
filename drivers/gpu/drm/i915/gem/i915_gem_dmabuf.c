/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright 2012 Red Hat Inc
 */

#include <linux/dma-buf.h>
#include <linux/highmem.h>
#include <linux/list.h>
#include <linux/pci-p2pdma.h>
#include <linux/scatterlist.h>
#include <drm/intel_iaf_platform.h>

#include "gem/i915_gem_dmabuf.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_requests.h"

#include "i915_drv.h"
#include "i915_gem_lmem.h"
#include "i915_gem_mman.h"
#include "i915_gem_object.h"
#include "i915_scatterlist.h"
#include "i915_trace.h"
#include "intel_iaf.h"

I915_SELFTEST_DECLARE(static bool force_different_devices;)

static const struct drm_i915_gem_object_ops i915_gem_object_dmabuf_ops;
static void i915_gem_unmap_dma_buf(struct dma_buf_attachment *attach,
				   struct sg_table *sgt,
				   enum dma_data_direction dir);
static int update_fabric(struct dma_buf *dma_buf,
			 struct drm_i915_gem_object *obj);

static struct drm_i915_gem_object *dma_buf_to_obj(struct dma_buf *buf)
{
	return to_intel_bo(buf->priv);
}

static void unmap_sg(struct device *dev,
		     struct scatterlist *sg,
		     enum dma_data_direction dir,
		     unsigned long attrs)
{
	while (sg) {
		dma_unmap_resource(dev,
				   sg_dma_address(sg), sg_dma_len(sg),
				   dir, attrs);
		sg = __sg_next(sg);
	}
}

/*
 * Objects may not have any pages.  Pinning is the usual method to allocate
 * the backing store, however dma-buf would like to avoid pinning.  Since
 * the caller holds the dma-resv, go straight to _get_pages.
 */
static int check_get_pages(struct drm_i915_gem_object *obj)
{
	int ret = 0;

	if (i915_gem_object_has_segments(obj)) {
		struct drm_i915_gem_object *sobj;

		for_each_object_segment(sobj, obj) {
			if (!i915_gem_object_has_pages(sobj))
				ret = ____i915_gem_object_get_pages(sobj);
			if (ret)
				break;
		}
	} else {
		if (!i915_gem_object_has_pages(obj)) {
			ret = ____i915_gem_object_get_pages(obj);

			if (!ret && obj->pair)
				if (!i915_gem_object_has_pages(obj->pair))
					ret = ____i915_gem_object_get_pages(obj->pair);
		}
	}

	return ret;
}

static struct sg_table *
i915_gem_copy_map_dma_buf(struct dma_buf_attachment *attach,
			  enum dma_data_direction map_dir)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(attach->dmabuf);
	struct scatterlist *dst, *end = NULL;
	struct sg_table *sgt;
	unsigned int nents;
	int ret;

	ret = check_get_pages(obj);
	if (ret)
		goto err;

	/*
	 * Make a copy of the object's sgt, so that we can make an independent
	 * mapping.
	 * NOTE: For LMEM objects the dma entries contain the device specific
	 * address information.  This will get overwritten by dma-buf-map
	 */
	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		ret = -ENOMEM;
		goto err;
	}

	if (i915_gem_object_has_segments(obj)) {
		struct drm_i915_gem_object *sobj;

		nents = 0;
		for_each_object_segment(sobj, obj)
			nents += sobj->mm.pages->orig_nents;

		obj = i915_gem_object_first_segment(obj);
	} else {
		nents = obj->mm.pages->orig_nents;
		if (obj->pair)
			nents += obj->pair->mm.pages->orig_nents;
	}

	if (sg_alloc_table(sgt, nents, I915_GFP_ALLOW_FAIL)) {
		ret = -ENOMEM;
		goto err_free;
	}

	sgt->nents = 0;
	dst = sgt->sgl;
	do {
		struct intel_memory_region *mem = obj->mm.region.mem;
		struct scatterlist *src;
		u64 dma_offset;

		dma_offset = 0;
		if (i915_gem_object_is_lmem(obj))
			dma_offset = mem->io_start - mem->region.start;

		ret = i915_gem_object_migrate_sync(obj);
		if (ret)
			goto err_unmap;

		for (src = obj->mm.pages->sgl; src; src = __sg_next(src)) {
			dma_addr_t addr, len;

			GEM_BUG_ON(!dst);
			sg_set_page(dst, sg_page(src), src->length, 0);

			if (map_dir == DMA_NONE) {
				addr = sg_dma_address(src);
				len = sg_dma_len(src);
			} else if (dma_offset) {
				len = sg_dma_len(src);
				addr = dma_map_resource(attach->dev,
							sg_dma_address(src) + dma_offset,
							len,
							map_dir,
							DMA_ATTR_SKIP_CPU_SYNC);
			} else {
				len = src->length;
				addr = dma_map_page_attrs(attach->dev,
							  sg_page(src), 0, len,
							  map_dir,
							  DMA_ATTR_SKIP_CPU_SYNC);
			}
			if (dma_mapping_error(obj->base.dev->dev, addr)) {
				ret = -ENOMEM;
				goto err_unmap;
			}

			sg_dma_address(dst) = addr;
			sg_dma_len(dst) = len;

			end = dst;
			dst = __sg_next(dst);
			sgt->nents++;
		}

		/* advance to next segment object, or to the object's pair */
		if (i915_gem_object_is_segment(obj))
			obj = rb_entry_safe(rb_next(&obj->segment_node),
					    typeof(*obj), segment_node);
		else
			obj = obj->pair;
	} while (obj);
	sg_mark_end(end);

	return sgt;

err_unmap:
	if (end) {
		sg_mark_end(end);
		unmap_sg(attach->dev,
			 sgt->sgl, map_dir,
			 DMA_ATTR_SKIP_CPU_SYNC);
	}
	sg_free_table(sgt);
err_free:
	kfree(sgt);
err:
	return ERR_PTR(ret);
}

static void i915_gem_unmap_dma_buf(struct dma_buf_attachment *attach,
				   struct sg_table *sgt,
				   enum dma_data_direction dir)
{
	unmap_sg(attach->dev, sgt->sgl, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static int i915_gem_dmabuf_vmap(struct dma_buf *dma_buf,
				struct iosys_map *map)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(dma_buf);
	enum i915_map_type type;
	void *vaddr;

	type = i915_coherent_map_type(to_i915(obj->base.dev), obj, true);
	vaddr = i915_gem_object_pin_map_unlocked(obj, type);
	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);

	iosys_map_set_vaddr(map, vaddr);

	return 0;
}

static void i915_gem_dmabuf_vunmap(struct dma_buf *dma_buf,
				   struct iosys_map *map)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(dma_buf);

	i915_gem_object_flush_map(obj);
	i915_gem_object_unpin_map(obj);
}

static int i915_gem_dmabuf_mmap(struct dma_buf *dma_buf,
				struct vm_area_struct *vma)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(dma_buf);
	struct i915_mmap_offset *mmo;
	int err;

	if (obj->base.size < vma->vm_end - vma->vm_start)
		return -EINVAL;

	i915_gem_object_get(obj);
	mmo = i915_gem_mmap_offset_attach(obj, I915_MMAP_TYPE_WC, NULL);
	if (IS_ERR(mmo)) {
		err = PTR_ERR(mmo);
		goto out;
	}

	err = i915_gem_update_vma_info(obj, mmo, vma);
	if (err)
		goto out;

	return 0;

out:
	i915_gem_object_put(obj);
	return err;
}

static int i915_gem_begin_cpu_access(struct dma_buf *dma_buf, enum dma_data_direction direction)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(dma_buf);
	bool write = (direction == DMA_BIDIRECTIONAL || direction == DMA_TO_DEVICE);
	struct i915_gem_ww_ctx ww;
	int err;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	err = i915_gem_object_lock(obj, &ww);
	if (!err)
		err = i915_gem_object_pin_pages_sync(obj);
	if (!err) {
		if (i915_gem_object_is_lmem(obj))
			err = i915_gem_object_set_to_wc_domain(obj, write);
		else
			err = i915_gem_object_set_to_cpu_domain(obj, write);
		i915_gem_object_unpin_pages(obj);
	}
	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}
	i915_gem_ww_ctx_fini(&ww);
	return err;
}

static int i915_gem_end_cpu_access(struct dma_buf *dma_buf, enum dma_data_direction direction)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(dma_buf);
	struct i915_gem_ww_ctx ww;
	int err;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	err = i915_gem_object_lock(obj, &ww);
	if (!err)
		err = i915_gem_object_pin_pages(obj);
	if (!err) {
		err = i915_gem_object_set_to_gtt_domain(obj, false);
		i915_gem_object_unpin_pages(obj);
	}
	if (err == -EDEADLK) {
		err = i915_gem_ww_ctx_backoff(&ww);
		if (!err)
			goto retry;
	}
	i915_gem_ww_ctx_fini(&ww);
	return err;
}

static const struct pci_device_id supported_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x09a2), 256 },
	{ 0 }
};

/*
 * It is possible that the necessary bridge ID has not made it to the p2pdma
 * list.  Double check.
 * NOTE: this is modeled after pci_p2pdma_distance_many function.  Only the bridge
 * verification path is "followed".
 */
static int local_distance_check(struct pci_dev *pci_dev)
{
	const struct pci_device_id *match = NULL;
	struct pci_host_bridge *host;
	struct pci_dev *root;

	host = pci_find_host_bridge(pci_dev->bus);
	if (!host)
		goto exit;

	root = pci_get_slot(host->bus, 0);
	if (!root)
		goto exit;

	match = pci_match_id(supported_ids, root);

exit:
	return match ? match->driver_data : -1;
}

#define I915_P2PDMA_OVERRIDE BIT(0)
#define I915_FABRIC_ONLY BIT(1)

static bool fabric_only(struct drm_i915_private *i915)
{
	return i915->params.prelim_override_p2p_dist & I915_FABRIC_ONLY;
}

static bool p2pdma_override(struct drm_i915_private *i915)
{
	return i915->params.prelim_override_p2p_dist & I915_P2PDMA_OVERRIDE;
}

static int i915_p2p_distance(struct drm_i915_private *i915, struct device *dev)
{
	int distance = 255; /* Override uses an arbitrary > 0 value */

	if (!p2pdma_override(i915))
		distance = pci_p2pdma_distance(to_pci_dev(i915->drm.dev), dev,
					       false);

	/* double check the PCI bridge info (WA for missing ID) */
	if (distance == -1)
		distance = local_distance_check(to_pci_dev(i915->drm.dev));

	return distance;
}

static int object_to_attachment_p2p_distance(struct drm_i915_gem_object *obj,
					     struct dma_buf_attachment *attach)
{
	return i915_p2p_distance(to_i915(obj->base.dev), attach->dev);
}

static int i915_gem_dmabuf_pin(struct dma_buf_attachment *attach)
{
	struct drm_i915_gem_object *sobj, *obj = dma_buf_to_obj(attach->dmabuf);
	int err = 0;

	if (i915_gem_object_has_segments(obj)) {
		for_each_object_segment(sobj, obj) {
			struct rb_node *node;

			err = i915_gem_object_pin_pages(sobj);
			if (!err)
				continue;

			for (node = rb_prev(&sobj->segment_node);
			     node; node = rb_prev(node)) {
				sobj = rb_entry(node, typeof(*sobj),
						segment_node);
				i915_gem_object_unpin_pages(sobj);
			}
			break;
		}
	} else {
		err = i915_gem_object_pin_pages(obj);
		if (!err && obj->pair) {
			err = i915_gem_object_pin_pages(obj->pair);
			if (err)
				i915_gem_object_unpin_pages(obj);
		}
	}

	return err;
}

static void i915_gem_dmabuf_unpin(struct dma_buf_attachment *attach)
{
	struct drm_i915_gem_object *sobj, *obj = dma_buf_to_obj(attach->dmabuf);

	if (i915_gem_object_has_segments(obj)) {
		for_each_object_segment(sobj, obj)
			i915_gem_object_unpin_pages(sobj);
	} else {
		if (obj->pair)
			i915_gem_object_unpin_pages(obj->pair);
		i915_gem_object_unpin_pages(obj);
	}
}

/*
 * Order of communication path is
 *    fabric
 *    p2p
 *    migrate
 */
static int i915_gem_dmabuf_attach(struct dma_buf *dmabuf,
				  struct dma_buf_attachment *attach)
{
	struct drm_i915_gem_object *sobj, *obj = dma_buf_to_obj(dmabuf);
	struct i915_gem_ww_ctx ww;
	int p2p_distance;
	int fabric;
	int err;

	fabric = update_fabric(dmabuf, attach->importer_priv);
	p2p_distance = object_to_attachment_p2p_distance(obj, attach);

	if (fabric < 0)
		return -EOPNOTSUPP;

	if (!fabric && p2p_distance < 0 &&
	    !i915_gem_object_can_migrate(obj, INTEL_REGION_SMEM))
		return -EOPNOTSUPP;

	trace_i915_dma_buf_attach(obj, fabric, p2p_distance);

	pvc_wa_disallow_rc6(to_i915(obj->base.dev));
	for_i915_gem_ww(&ww, err, true) {
		err = i915_gem_object_lock(obj, &ww);
		if (err)
			continue;
		if (obj->pair) {
			err = i915_gem_object_lock(obj->pair, &ww);
			if (err) {
				i915_gem_object_unlock(obj);
				continue;
			}
		}

		if (!fabric && p2p_distance < 0) {
			if (i915_gem_object_has_segments(obj)) {
				for_each_object_segment(sobj, obj) {
					err = i915_gem_object_lock(sobj, &ww);
					if (err)
						break;

					/*
					 * can_migrate() above tested that placement
					 * list allows migration.
					 * Here we perform the missed step from that
					 * function and test that pin_count for this
					 * segment is zero.
					 */
					if (!i915_gem_object_evictable(sobj)) {
						err = -EOPNOTSUPP;
						break;
					}

					err = i915_gem_object_migrate(sobj,
								      INTEL_REGION_SMEM,
								      false);
					if (err)
						break;
				}
			} else {
				GEM_BUG_ON(obj->pair);
				err = i915_gem_object_migrate(obj, INTEL_REGION_SMEM, false);
			}
			if (err)
				continue;
		}
	}

	return err;
}

static void i915_gem_dmabuf_detach(struct dma_buf *dmabuf,
				   struct dma_buf_attachment *attach)
{
	struct drm_i915_gem_object *obj = dma_buf_to_obj(dmabuf);
	struct drm_i915_private *i915 = to_i915(obj->base.dev);

	pvc_wa_allow_rc6(i915);
}

static const struct dma_buf_ops i915_dmabuf_ops =  {
	.attach = i915_gem_dmabuf_attach,
	.detach = i915_gem_dmabuf_detach,
	.map_dma_buf = i915_gem_copy_map_dma_buf,
	.unmap_dma_buf = i915_gem_unmap_dma_buf,
	.release = drm_gem_dmabuf_release,
	.mmap = i915_gem_dmabuf_mmap,
	.vmap = i915_gem_dmabuf_vmap,
	.vunmap = i915_gem_dmabuf_vunmap,
	.begin_cpu_access = i915_gem_begin_cpu_access,
	.end_cpu_access = i915_gem_end_cpu_access,
	.pin = i915_gem_dmabuf_pin,
	.unpin = i915_gem_dmabuf_unpin,
};

struct dma_buf *i915_gem_prime_export(struct drm_gem_object *gem_obj, int flags)
{
	struct drm_i915_gem_object *obj = to_intel_bo(gem_obj);
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct i915_gem_ww_ctx ww;
	struct dma_buf *dmabuf = NULL;
	int err;

	if (obj->vm) {
		drm_dbg(obj->base.dev,
			"Exporting VM private objects is not allowed\n");
		return ERR_PTR(-EINVAL);
	}

	/*
	 * Hold the object lock so the EPERM check below is not circumvented
	 * by concurrent prime_export and vm_madvise usage.
	 */
	for_i915_gem_ww(&ww, err, true) {
		err = i915_gem_object_lock(obj, &ww);
		if (err)
			continue;

		/*
		 * We don't allow exporting BOs if LMEM + SMEM placement and
		 * the ATOMIC_SYSTEM hint is set. The complexity of supporting
		 * migration with multi-device atomics has not been solved.
		 */
		if (i915_gem_object_allows_atomic_system(obj) &&
		    obj->memory_mask & REGION_LMEM &&
		    obj->memory_mask & REGION_SMEM) {
			err = -EPERM;
			continue;
		}

		exp_info.ops = &i915_dmabuf_ops;
		exp_info.size = gem_obj->size;
		if (obj->pair)
			exp_info.size += obj->pair->base.size;
		exp_info.flags = flags;
		exp_info.priv = gem_obj;
		exp_info.resv = obj->base.resv;

		if (obj->ops->dmabuf_export) {
			err = obj->ops->dmabuf_export(obj);
			if (err)
				continue;
		}

		dmabuf = drm_gem_dmabuf_export(gem_obj->dev, &exp_info);
		/*
		 * gem_obj->dma_buf is set in the caller upon return, but we
		 * set here with the object lock held, so that the check for
		 * gem_object_is_exported() in __i915_gem_object_set_hint()
		 * correctly sees if BO is exported or not.
		 */
		if (!IS_ERR(dmabuf))
			gem_obj->dma_buf = dmabuf;
	}

	if (err)
		dmabuf =  ERR_PTR(err);
	/* not possible, placed here so static checker doesn't complain */
	GEM_BUG_ON(!dmabuf);

	return dmabuf;
}

/*
 * update_fabric - check for fabric connectivity if available
 * @obj: object to check fabric connectivity
 *
 * If the imported object is a i915 dma-buf, and LMEM based, query to see if
 * there is a fabric, and if the fabric is connected set the fabric bit.
 *
 * 0 no connectivity, use P2P if available
 * 1 fabric is available
 * -1 fabric only is requested, and there is no fabric
 *
 */
static int update_fabric(struct dma_buf *dma_buf,
			 struct drm_i915_gem_object *obj)
{
	struct drm_i915_gem_object *import;
	struct drm_i915_private *src;
	struct drm_i915_private *dst;
	struct query_info *qi;
	int connected;
	int i;
	int n;

	/* Verify that both sides are i915s */
	if (dma_buf->ops != &i915_dmabuf_ops ||
	    !obj || obj->ops != &i915_gem_object_dmabuf_ops)
		return 0;

	import = dma_buf_to_obj(dma_buf);
	if (!i915_gem_object_is_lmem(import))
		return 0;

	src = to_i915(obj->base.dev);
	dst = to_i915(import->base.dev);

	qi = src->intel_iaf.ops->connectivity_query(src->intel_iaf.handle,
						    dst->intel_iaf.fabric_id);
	if (IS_ERR(qi))
		return fabric_only(src) ? -1 : 0;

	/*
	 * Examine the query information.  A zero bandwidth link indicates we
	 * are NOT connected.
	 */
	connected = 1;
	for (i = 0, n = qi->src_cnt * qi->dst_cnt; i < n && connected; i++)
		if (!qi->sd2sd[i].bandwidth)
			connected = 0;

	/* we are responsible for freeing qi */
	kfree(qi);

	if (connected) {
		if (intel_iaf_mapping_get(src))
			return 0;
		if (intel_iaf_mapping_get(dst)) {
			intel_iaf_mapping_put(src);
			return 0;
		}
		i915_gem_object_set_fabric(obj);
	}

	/* Object can use fabric or P2P, check for fabric only request */
	if (!connected && fabric_only(src))
		return -1;

	return connected;
}

/**
 * map_fabric_connectivity - check for fabric and create a mappable sgt if
 * available
 * @obj: object to check fabric connectivity
 *
 * Returns sgt or -errno on error, -EIO indicates no fabric connectivity.
 */
static struct sg_table *map_fabric_connectivity(struct drm_i915_gem_object *obj)
{
	struct dma_buf_attachment *attach = obj->base.import_attach;
	struct drm_i915_gem_object *import;

	if (!i915_gem_object_has_fabric(obj))
		return ERR_PTR(-EIO);

	import = dma_buf_to_obj(attach->dmabuf);

	/* Make sure the object didn't migrate */
	if (!i915_gem_object_is_lmem(import)) {
		i915_gem_object_clear_fabric(obj);
		return ERR_PTR(-EIO);
	}

	return i915_gem_copy_map_dma_buf(attach, DMA_NONE);
}

/**
 * i915_gem_object_get_pages_dmabuf - get SG Table of pages from dmabuf
 * @obj: object on import side of dmabuf
 *
 * obj is created int _prime_import().  Determine where the pages need to
 * come from, and go get them.
 *
 */
static int i915_gem_object_get_pages_dmabuf(struct drm_i915_gem_object *obj)
{
	struct sg_table *sgt;
	unsigned int sg_page_sizes;

	assert_object_held(obj);

	/* See if there is a fabric, and set things up. */
	sgt = map_fabric_connectivity(obj);

	if (IS_ERR(sgt) && PTR_ERR(sgt) == -EIO)
		sgt = dma_buf_map_attachment(obj->base.import_attach,
					     DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	sg_page_sizes = i915_sg_dma_sizes(sgt->sgl);

	__i915_gem_object_set_pages(obj, sgt, sg_page_sizes);

	return 0;
}

static int i915_gem_object_put_pages_dmabuf(struct drm_i915_gem_object *obj,
					     struct sg_table *pages)
{
	if (i915_gem_object_has_fabric(obj)) {
		struct drm_i915_gem_object *export;

		export = dma_buf_to_obj(obj->base.import_attach->dmabuf);
		intel_iaf_mapping_put(to_i915(export->base.dev));
		intel_iaf_mapping_put(to_i915(obj->base.dev));

		i915_gem_object_clear_fabric(obj);
		sg_free_table(pages);
		kfree(pages);
		return 0;
	}

	dma_buf_unmap_attachment(obj->base.import_attach, pages,
				 DMA_BIDIRECTIONAL);
	return 0;
}

static const struct drm_i915_gem_object_ops i915_gem_object_dmabuf_ops = {
	.name = "i915_gem_object_dmabuf",
	.get_pages = i915_gem_object_get_pages_dmabuf,
	.put_pages = i915_gem_object_put_pages_dmabuf,
};

/*
 * The exporter has moved or evicted its pages associatd with this dma-buf.
 * As the importer, release any mappings that have been done on this imported
 * buffer.
 *
 * NOTE: the exporter is calling this function while holding the dma-resv
 *
 */
static void i915_dmabuf_move_notify(struct dma_buf_attachment *attach)
{
	struct drm_i915_gem_object *obj = attach->importer_priv;

	if (!obj)
		return;

	GEM_WARN_ON(i915_gem_object_has_segments(obj));
	GEM_WARN_ON(obj->pair);

	i915_gem_object_wait(obj, 0, MAX_SCHEDULE_TIMEOUT);

	if (i915_gem_object_unbind(obj, NULL, I915_GEM_OBJECT_UNBIND_ACTIVE |
				   I915_GEM_OBJECT_UNBIND_BARRIER) == 0)
		__i915_gem_object_put_pages(obj);

	GEM_WARN_ON(i915_gem_object_has_pages(obj));
}

static const struct dma_buf_attach_ops i915_dmabuf_attach_ops = {
	.allow_peer2peer = true,
	.move_notify = i915_dmabuf_move_notify,
};

struct drm_gem_object *i915_gem_prime_import(struct drm_device *dev,
					     struct dma_buf *dma_buf)
{
	static struct lock_class_key lock_class;
	struct dma_buf_attachment *attach;
	struct drm_i915_gem_object *obj;

	/* is this one of own objects? */
	if (dma_buf->ops == &i915_dmabuf_ops) {
		obj = dma_buf_to_obj(dma_buf);
		/* is it from our device? */
		if (obj->base.dev == dev &&
		    !I915_SELFTEST_ONLY(force_different_devices)) {
			/*
			 * Importing dmabuf exported from out own gem increases
			 * refcount on gem itself instead of f_count of dmabuf.
			 */
			return &i915_gem_object_get(obj)->base;
		}
	}

	if (i915_gem_object_size_2big(dma_buf->size))
		return ERR_PTR(-E2BIG);

	obj = i915_gem_object_alloc();
	if (!obj)
		return ERR_PTR(-ENOMEM);

	drm_gem_private_object_init(dev, &obj->base, dma_buf->size);
	i915_gem_object_init(obj, &i915_gem_object_dmabuf_ops, &lock_class,
			     I915_BO_ALLOC_USER);
	obj->base.resv = dma_buf->resv;

	/*
	 * We use GTT as shorthand for a coherent domain, one that is
	 * neither in the GPU cache nor in the CPU cache, where all
	 * writes are immediately visible in memory. (That's not strictly
	 * true, but it's close! There are internal buffers such as the
	 * write-combined buffer or a delay through the chipset for GTT
	 * writes that do require us to treat GTT as a separate cache domain.)
	 */
	obj->read_domains = I915_GEM_DOMAIN_GTT;
	obj->write_domain = 0;

	/* and attach the object */
	attach = dma_buf_dynamic_attach(dma_buf, dev->dev,
					&i915_dmabuf_attach_ops, obj);
	if (IS_ERR(attach)) {
		i915_gem_object_put(obj);
		return ERR_CAST(attach);
	}

	get_dma_buf(dma_buf);
	obj->base.import_attach = attach;

	return &obj->base;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/mock_dmabuf.c"
#include "selftests/i915_gem_dmabuf.c"
#endif
