// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 - 2022 Intel Corporation
 */

#include <linux/irq.h>
#include <linux/firmware.h>
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
#include <linux/auxiliary_bus.h>
#else
#include <linux/mfd/core.h>
#endif
#include <linux/mutex.h>
#include <linux/xarray.h>

#include <drm/intel_iaf_platform.h>

#include "gt/intel_gt.h"
#include "gt/intel_gt_mcr.h"
#include "gt/intel_gt_regs.h"

#include "i915_drv.h"
#include "intel_iaf.h"
#include "intel_pcode.h"

/* Xarray of fabric devices */
static DEFINE_XARRAY_ALLOC(intel_fdevs);

static struct query_info *default_query(void *handle, u32 fabric_id)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static int default_handle_event(void *handle, enum iaf_parent_event event)
{
	return -EOPNOTSUPP;
}

static const struct iaf_ops default_ops = {
	.connectivity_query = default_query,
	.parent_event = default_handle_event,
};

static int register_dev(void *parent, void *handle, u32 fabric_id,
			const struct iaf_ops *ops)
{
	struct drm_i915_private *i915 = parent;

	WARN(i915->intel_iaf.ops != &default_ops, "IAF: already registered");

	mutex_lock(&i915->intel_iaf.power_mutex);
	if (!i915->intel_iaf.power_enabled) {
		mutex_unlock(&i915->intel_iaf.power_mutex);
		return -EAGAIN;
	}

	i915->intel_iaf.handle = handle;
	i915->intel_iaf.fabric_id = fabric_id;
	i915->intel_iaf.ops = ops;
	mutex_unlock(&i915->intel_iaf.power_mutex);

	drm_info(&i915->drm, "IAF: registered fabric: 0x%x\n", fabric_id);

	return 0;
}

static void unregister_dev(void *parent, const void *handle)
{
	struct drm_i915_private *i915 = parent;

	WARN(i915->intel_iaf.handle != handle, "IAF: invalid handle");

	drm_info(&i915->drm, "IAF: unregistered fabric: 0x%x\n",
		 i915->intel_iaf.fabric_id);

	mutex_lock(&i915->intel_iaf.power_mutex);
	i915->intel_iaf.handle = NULL;
	i915->intel_iaf.ops = &default_ops;
	mutex_unlock(&i915->intel_iaf.power_mutex);
}

static int dev_event(void *parent, const void *handle, const enum iaf_dev_event event,
		     void *event_data)
{
	struct drm_i915_private *i915 = parent;

	WARN(i915->intel_iaf.handle != handle, "IAF: invalid handle");

	if (event == IAF_DEV_REMOVE)
		i915_gem_flush_free_objects(i915);

	return 0;
}

/**
 * init_pd - Allocate and initialize platform specific data
 * @i915: Valid i915 instance
 *
 * Return:
 * * pd - initialized iaf_pdata,
 * * NULL - Allocation failure
 */
static struct iaf_pdata *init_pd(struct drm_i915_private *i915)
{
	struct iaf_pdata *pd;
	u32 reg;

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return NULL;

	pd->version = IAF_VERSION;
	pd->parent = i915;
	pd->product = IAF_PONTEVECCHIO;
	pd->index = i915->intel_iaf.index & 0xFFFF;
	pd->sd_cnt = i915->remote_tiles + 1;
	pd->socket_id = i915->intel_iaf.socket_id;
	pd->slot = PCI_SLOT(to_pci_dev(i915->drm.dev)->devfn);

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	pd->resources = NULL;
	pd->num_resources = 0;
#endif
	pd->register_dev = register_dev;
	pd->unregister_dev = unregister_dev;
	pd->dev_event = dev_event;

	/*
	 * Calculate the actual DPA offset and size (in GB) for the device.
	 * Each tile will have the same amount of memory, so we only need to
	 * read the first one.
	 */
	reg = intel_gt_mcr_read_any(to_gt(i915), XEHP_TILE0_ADDR_RANGE) &
		XEHP_TILE_LMEM_RANGE_MASK;

	// FIXME: On some systems, TILE0 is < 8Gb. PVC needs 8GB, so fake it.
	if (reg >> XEHP_TILE_LMEM_RANGE_SHIFT < 8) {
		drm_err(&i915->drm, "XEHP_TILE0_ADDR_RANGE: %x\n", reg);
		reg = 8 << XEHP_TILE_LMEM_RANGE_SHIFT;
	}
	pd->dpa.pkg_offset = (u32)i915->intel_iaf.index * MAX_DPA_SIZE;
	pd->dpa.pkg_size = (reg >> XEHP_TILE_LMEM_RANGE_SHIFT) * pd->sd_cnt;

	return pd;
}

/**
 * init_resource - Create the resource array, and apply the appropriate data
 * @i915: valid i915 instance
 * @res_cnt: pointer to return number of allocated resources
 *
 * First resource [0] is for the IRQ(s).  Each device gets 1 IRQ. Subsequent
 * resources describe the IO memory space for the device(s).
 *
 * Make sure to set the gt->iaf_irq value.
 *
 * Return:
 * * res - Initialized resource array
 * * NULL - Allocaction failure
 */
static struct resource *init_resource(struct drm_i915_private *i915,
				      u32 *res_cnt)
{
	struct intel_gt *gt;
	struct resource *res_base, *res;
	u32 cnt = (i915->remote_tiles + 1) * 2;
	unsigned int i;

	/* Each sd gets one resource for IRQ and one for MEM */
	res_base = kcalloc(cnt, sizeof(*res_base), GFP_KERNEL);
	if (!res_base)
		return NULL;

	res = res_base;
	for_each_gt(gt, i915, i) {
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
		res->start = i915->intel_iaf.irq_base + i;
		res->end = i915->intel_iaf.irq_base + i;
#else
		res->start = i;
		res->end = i;
#endif
		res->flags = IORESOURCE_IRQ;
		res++;

		res->start = gt->phys_addr + CD_BASE_OFFSET;
		res->end = res->start + CD_BAR_SIZE - 1;
		res->flags = IORESOURCE_MEM;
		drm_info(&i915->drm, "IAF: mem_resource = %pR\n", res);
		res++;

		gt->iaf_irq = i915->intel_iaf.irq_base + i;
	}

	*res_cnt = cnt;
	return res_base;
}

/**
 * iaf_irq_mask - Null callback.  Masking/unmasking happens in the parent
 * driver
 * @d: Valid irq_data information
 */
static void iaf_irq_mask(struct irq_data *d)
{
}

static void iaf_irq_unmask(struct irq_data *d)
{
}

static struct irq_chip iaf_irq_chip = {
	.name = "iaf_irq_chip",
	.irq_mask = iaf_irq_mask,
	.irq_unmask = iaf_irq_unmask,
};

/**
 * init_irq_desc - Allocate IRQ descriptors to use for the iaf
 * @i915: Valid i915 instance
 *
 * Allocate the required IRQ descriptor(s) and initialize the
 * appropriate state.
 *
 * Return:
 * * 0 - Success
 * * errno - Error that occurred
 */
static int init_irq_desc(struct drm_i915_private *i915)
{
	unsigned int num_subdevs = i915->remote_tiles + 1;
	int err;
	int irq;
	int irq_base;

	irq_base = irq_alloc_descs(-1, 0, num_subdevs, 0);
	if (irq_base < 0) {
		err = irq_base;
		goto cleanup;
	}

	err = 0;
	for (irq = irq_base; !err && irq < irq_base + num_subdevs; irq++) {
		irq_set_chip_and_handler_name(irq, &iaf_irq_chip,
					      handle_simple_irq,
					      "iaf_irq_handler");
		err = irq_set_chip_data(irq, i915);
	}

	if (err) {
		irq_free_descs(irq_base, num_subdevs);
		goto cleanup;
	}

	drm_info(&i915->drm, "IAF: IRQ base: %d  cnt: %d\n", irq_base,
		 num_subdevs);

	i915->intel_iaf.irq_base = irq_base;

	return 0;

cleanup:
	i915->intel_iaf.index = err;
	drm_err(&i915->drm, "IAF: Failed to allocate IRQ data: %d\n", err);
	return err;
}

/**
 * intel_iaf_init_early - Set the iaf info to the defaults
 * @i915: valid i915 instance
 *
 * index is set to ENODEV to show that, by default, there is no device.
 * If any of the initialization steps fail, it will be set to the appropriate
 * errno value.
 */
void intel_iaf_init_early(struct drm_i915_private *i915)
{
	i915->intel_iaf.ops = &default_ops;
	i915->intel_iaf.index = -ENODEV;
	mutex_init(&i915->intel_iaf.power_mutex);
}

static bool iaf_power_enabled(struct drm_i915_private *i915)
{
	u32 mbox = REG_FIELD_PREP(GEN6_PCODE_MB_COMMAND,
				  PCODE_MBOX_CD) |
		REG_FIELD_PREP(GEN6_PCODE_MB_PARAM1, PCODE_MBOX_CD_STATUS) |
		REG_FIELD_PREP(GEN6_PCODE_MB_PARAM2, 0);
	u32 val = 0;
	int err = snb_pcode_read(&i915->uncore, mbox, &val, NULL);

	if (err)
		return false;

	return val == PCODE_MBOX_CD_STATUS_DATA_ONLINE;
}

/**
 * intel_iaf_init_mmio - check if iaf is available via MMIO
 * @i915: valid i915 instance
 *
 * Read the relevant regs to check for IAF availability and get the socket id
 */
void intel_iaf_init_mmio(struct drm_i915_private *i915)
{
	u32 iaf_info;

	if (!HAS_IAF(i915) || !i915->params.enable_iaf || IS_SRIOV_VF(i915))
		return;

	iaf_info = intel_uncore_read(&i915->uncore, PUNIT_MMIO_CR_POC_STRAPS);

	if (!REG_FIELD_GET(CD_ALIVE, iaf_info))
		return;

	i915->intel_iaf.power_enabled = true;
	if (IS_PVC_BD_STEP(i915, STEP_B0, STEP_FOREVER)) {
		if (!iaf_power_enabled(i915)) {
			i915->intel_iaf.power_enabled = false;
			drm_warn(&i915->drm, "IAF: power disabled\n");
		}
	}

	drm_info(&i915->drm, "IAF available\n");
	i915->intel_iaf.socket_id = REG_FIELD_GET(SOCKET_ID_MASK, iaf_info);
	i915->intel_iaf.present = true;
}

/**
 * intel_iaf_init - Allocate device index and complete initial HW setup
 * @i915: valid device instance
 *
 * NOTE: index is zero inited.  If the IAF is not present, or an error occurs
 * during setup, this must be 0 for the range registers.
 *
 */
void intel_iaf_init(struct drm_i915_private *i915)
{
	struct intel_gt *gt;
	static u32 last_id;
	unsigned int i;
	u32 index = 0;
	u32 range;
	u32 base;
	int err;

	if (!HAS_IAF(i915) || IS_SRIOV_VF(i915))
		return;

	if (i915->intel_iaf.present) {
		err = init_irq_desc(i915);
		if (err)
			goto set_range;

		/*
		 * Try the socket id first.  Systems with this feature, will
		 * get a deterministic value.  If not, try with the cyclic.
		 */
		err = xa_insert(&intel_fdevs, i915->intel_iaf.socket_id, i915,
				GFP_KERNEL);
		if (!err)
			index = i915->intel_iaf.socket_id;

		/* socket_id is not available */
		if (err == -EBUSY) {
			/*
			 * NOTE: index is only updated on success i.e. >= 0
			 * < 0 err, 0 ok, > 0 wrapped
			 */
			err = xa_alloc_cyclic(&intel_fdevs, &index, i915,
					      XA_LIMIT(0, MAX_DEVICE_COUNT - 1),
					      &last_id, GFP_KERNEL);
		}
		if (err < 0) {
			i915->intel_iaf.index = err;
			drm_err(&i915->drm,
				"IAF: Failed to allocate fabric index: %d\n",
				err);
			irq_free_descs(i915->intel_iaf.irq_base,
				       i915->remote_tiles + 1);
			goto set_range;
		}
		i915->intel_iaf.index = index;
		i915->intel_iaf.dpa = (u64)index * MAX_DPA_SIZE * SZ_1G;
		drm_info(&i915->drm, "IAF: [dpa 0x%llx-0x%llx]\n",
			 i915->intel_iaf.dpa,
			 ((u64)index + 1) * MAX_DPA_SIZE * SZ_1G - 1);
	}

	/*
	 * Set range has to be done for all devices that support device
	 * address space, regardless of presence or error.
	 */
set_range:
	/* Set GAM address range registers */
	range = index * MAX_DPA_SIZE << PKG_ADDR_RANGE_BASE_SHIFT;
	range |= MAX_DPA_SIZE << PKG_ADDR_RANGE_RANGE_SHIFT;
	range |= PKG_ADDR_RANGE_ENABLE;

	/* set SGunit address range register */
	base = index * MAX_DPA_SIZE << PKG_ADDR_BASE_BASE_SHIFT;
	base |= MAX_DPA_SIZE << PKG_ADDR_BASE_RANGE_SHIFT;
	base |= PKG_ADDR_BASE_ENABLE;

	/* Needs to be set for each gt */
	for_each_gt(gt, i915, i) {
		intel_uncore_write(gt->uncore, PKG_ADDR_RANGE, range);
		intel_uncore_write(gt->uncore, PKG_ADDR_BASE, base);
	}
}

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
static void intel_iaf_release_dev(struct device *dev)
{
	struct auxiliary_device *aux = to_auxiliary_dev(dev);
	struct iaf_pdata *pd = container_of(aux, struct iaf_pdata, aux_dev);

	kfree(pd->resources);
	pd->resources = NULL;

	kfree(pd);
}

/**
 * intel_iaf_init_aux - Initialize resources and add auxiliary bus interface
 * @i915: valid i915 instance
 *
 */
void intel_iaf_init_aux(struct drm_i915_private *i915)
{
	struct device *dev = &to_pci_dev(i915->drm.dev)->dev;
	struct resource *res = NULL;
	struct iaf_pdata *pd;
	int err = -ENOMEM;
	u32 res_cnt;

	if (!i915->intel_iaf.present)
		return;

	if (i915->intel_iaf.index < 0) {
		err = i915->intel_iaf.index;
		goto fail;
	}

	WARN(IS_SRIOV_VF(i915), "Intel IAF doesn't support VF\n");

	pd = init_pd(i915);
	if (!pd)
		goto cleanup;

	res = init_resource(i915, &res_cnt);
	if (!res)
		goto cleanup;

	pd->resources = res;
	pd->num_resources = res_cnt;

	pd->aux_dev.name = "iaf";
	pd->aux_dev.id = pd->index;
	pd->aux_dev.dev.parent = dev;
	pd->aux_dev.dev.release = intel_iaf_release_dev;

	err = auxiliary_device_init(&pd->aux_dev);
	if (err)
		goto cleanup;

	err = auxiliary_device_add(&pd->aux_dev);
	if (err) {
		auxiliary_device_uninit(&pd->aux_dev);
		goto cleanup;
	}

	i915->intel_iaf.pd = pd;

	return;

cleanup:
	xa_erase(&intel_fdevs, i915->intel_iaf.index);
	irq_free_descs(i915->intel_iaf.irq_base, i915->remote_tiles + 1);
	kfree(res);
	kfree(pd);
	i915->intel_iaf.index = err;
fail:
	drm_err(&i915->drm, "IAF: Failed to initialize fabric: %d\n", err);
}

#else

/**
 * intel_iaf_init_mfd - Initialize resources and add MFD interface
 * @i915: valid i915 instance
 *
 * NOTE: MFD allows irq_base to be specified.  It will update the
 * IORESOURCE_IRQ with the base + instance.
 *
 */
void intel_iaf_init_mfd(struct drm_i915_private *i915)
{
	struct device *dev = &i915->drm.pdev->dev;
	struct resource *res = NULL;
	struct mfd_cell fcell = {};
	struct iaf_pdata *pd;
	int err = -ENOMEM;
	u32 res_cnt;

	if (!i915->intel_iaf.present)
		return;

	if (i915->intel_iaf.index < 0) {
		err = i915->intel_iaf.index;
		goto fail;
	}

	WARN(IS_SRIOV_VF(i915), "Intel IAF doesn't support VF\n");

	pd = init_pd(i915);
	if (!pd)
		goto cleanup;

	res = init_resource(i915, &res_cnt);
	if (!res)
		goto cleanup;

	fcell.name = "iaf";
	fcell.platform_data = pd;
	fcell.pdata_size = sizeof(*pd);
	fcell.resources = res;
	fcell.num_resources = res_cnt;

	err = mfd_add_devices(dev, pd->index, &fcell, 1, NULL,
			      i915->intel_iaf.irq_base, NULL);
	if (err)
		goto cleanup;

	/* mfd_add_devices copied this info,clean up*/
	kfree(pd);
	kfree(res);

	return;

cleanup:
	xa_erase(&intel_fdevs, i915->intel_iaf.index);
	irq_free_descs(i915->intel_iaf.irq_base, i915->remote_tiles + 1);
	kfree(res);
	kfree(pd);
	i915->intel_iaf.index = err;
fail:
	drm_err(&i915->drm, "IAF: Failed to initialize fabric: %d\n", err);
}
#endif

void intel_iaf_remove(struct drm_i915_private *i915)
{
	if (i915->intel_iaf.index < 0)
		return;

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	auxiliary_device_delete(&i915->intel_iaf.pd->aux_dev);
	auxiliary_device_uninit(&i915->intel_iaf.pd->aux_dev);
#endif
	xa_erase(&intel_fdevs, i915->intel_iaf.index);
	irq_free_descs(i915->intel_iaf.irq_base, i915->remote_tiles + 1);

	i915->intel_iaf.ops = &default_ops;
}

int intel_iaf_pcie_error_notify(struct drm_i915_private *i915)
{
	return i915->intel_iaf.ops->parent_event(i915->intel_iaf.handle,
						 IAF_PARENT_PCIE_ERR);
}

int intel_iaf_mapping_get(struct drm_i915_private *i915)
{
	return i915->intel_iaf.ops->parent_event(i915->intel_iaf.handle,
						 IAF_PARENT_MAPPING_GET);
}

int intel_iaf_mapping_put(struct drm_i915_private *i915)
{
	return i915->intel_iaf.ops->parent_event(i915->intel_iaf.handle,
						 IAF_PARENT_MAPPING_PUT);
}
