// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2023 Intel Corporation.
 */

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
#include <linux/auxiliary_bus.h>
#else
#include <linux/platform_device.h>
#endif
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/rwsem.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/xarray.h>
#include <generated/utsrelease.h>

#include <drm/intel_iaf_platform.h>

#include "csr.h"
#include "debugfs.h"
#include "dev_diag.h"
#include "fw.h"
#include "iaf_drv.h"
#include "mbdb.h"
#include "mbox.h"
#include "mei_iaf_user.h"
#include "netlink.h"
#include "port.h"
#include "routing_engine.h"
#include "routing_event.h"
#include "routing_p2p.h"
#if IS_ENABLED(CONFIG_IAF_DEBUG_SELFTESTS)
#include "selftests/selftest.h"
#endif
#include "sysfs.h"

#define MODULEDETAILS "Intel Corp. Intel fabric Driver"

/* xarray of IAF devices */
static DEFINE_XARRAY_ALLOC(intel_fdevs);

/* Protects routable_list access and data with shared/excl semantics */
DECLARE_RWSEM(routable_lock);
LIST_HEAD(routable_list);

/*
 * Used for creating unique xarray hash.  Products need to be defined
 * in the intel_iaf_platform.h file.
 */
#define PRODUCT_SHIFT 16

static enum iaf_startup_mode param_startup_mode = STARTUP_MODE_DEFAULT;

struct workqueue_struct *iaf_unbound_wq;

static const char *startup_mode_name(enum iaf_startup_mode m)
{
	switch (m) {
	case STARTUP_MODE_DEFAULT: return "default";
	case STARTUP_MODE_PRELOAD: return "preload";
#if IS_ENABLED(CONFIG_IAF_DEBUG_STARTUP)
	case STARTUP_MODE_DEBUG: return "debug";
	case STARTUP_MODE_FWDEBUG: return "fwdebug";
#endif
	}

	return "(unknown)";
}

static int mode_set(const char *val, const struct kernel_param *kp)
{
	enum iaf_startup_mode new_mode;

	if (sysfs_streq(val, "default"))
		new_mode = STARTUP_MODE_DEFAULT;
	else if (sysfs_streq(val, "preload"))
		new_mode = STARTUP_MODE_PRELOAD;
#if IS_ENABLED(CONFIG_IAF_DEBUG_STARTUP)
	else if (sysfs_streq(val, "debug"))
		new_mode = STARTUP_MODE_DEBUG;
	else if (sysfs_streq(val, "fwdebug"))
		new_mode = STARTUP_MODE_FWDEBUG;
#endif
	else
		return -EINVAL;

	param_startup_mode = new_mode;

	return 0;
}

static int mode_get(char *val, const struct kernel_param *kp)
{
	return snprintf(val, PAGE_SIZE, startup_mode_name(param_startup_mode));
}

static const struct kernel_param_ops startup_mode_ops = {
	.set = mode_set,
	.get = mode_get,
};

module_param_cb(startup_mode, &startup_mode_ops, &param_startup_mode, 0600);
#if IS_ENABLED(CONFIG_IAF_DEBUG_STARTUP)
MODULE_PARM_DESC(startup_mode,
		 "Operational mode for newly probed devices:\n"
		 "\t\t - default: Operate normally\n"
		 "\t\t - preload: Assume firmware/ini has already been loaded\n"
		 "\t\t - debug:   Do not interact with the device outside of L8sim debug netlink\n"
		 "\t\t - fwdebug: As per debug, but load firmware during device init"
		 );
#else
MODULE_PARM_DESC(startup_mode,
		 "Operational mode for newly probed devices:\n"
		 "\t\t - default: Operate normally\n"
		 "\t\t - preload: Assume firmware/ini has already been loaded"
		 );
#endif

/* BPS Link Speed indexed by bit # as returned by fls set in port info fields */
static u64 bps_link_speeds[] = {
	0, /* fls returns index 0 when no bits are set. */
	BPS_LINK_SPEED_12G,
	BPS_LINK_SPEED_25G,
	BPS_LINK_SPEED_53G,
	BPS_LINK_SPEED_90G,
	0,
	0,
	0,
	0,
};

/**
 * bps_link_speed - Returns link speed in bps for highest bit set in
 *                  port info link speed
 * @port_info_link_speed: speed as encoded in port info link speed
 *
 * Return: Speed in bits per second.
 * 0 when no bits are set or bit set is out of range
 */
u64 bps_link_speed(u8 port_info_link_speed)
{
	return bps_link_speeds[fls(port_info_link_speed)];
}

#define RELEASE_TIMEOUT (HZ * 60)

static void fdev_release(struct kref *kref)
{
	struct fdev *dev = container_of(kref, typeof(*dev), refs);

	xa_erase(&intel_fdevs, dev->fabric_id);
	complete(&dev->fdev_released);
}

void fdev_put(struct fdev *dev)
{
	kref_put(&dev->refs, fdev_release);
}

static struct fdev *fdev_get(struct fdev *dev)
{
	if (dev && kref_get_unless_zero(&dev->refs))
		return dev;

	return NULL;
}

/**
 * fdev_get_early - Acquire a reference unconditionally from probe context.
 * @dev: the device to operate on
 */
void fdev_get_early(struct fdev *dev)
{
	kref_get(&dev->refs);
}

int fdev_insert(struct fdev *dev)
{
	int err;

	kref_init(&dev->refs);
	init_completion(&dev->fdev_released);

	err = xa_insert(&intel_fdevs, dev->fabric_id, dev, GFP_KERNEL);
	if (err)
		dev_warn(fdev_dev(dev), "fabric_id 0x%08x already in use\n",
			 dev->fabric_id);

	return err;
}

int fdev_process_each(fdev_process_each_cb_t cb, void *args)
{
	struct fdev *dev;
	unsigned long i;

	xa_lock(&intel_fdevs);
	xa_for_each(&intel_fdevs, i, dev) {
		int ret;

		if (!fdev_get(dev))
			continue;

		xa_unlock(&intel_fdevs);
		ret = cb(dev, args);
		fdev_put(dev);
		if (ret)
			return ret;

		xa_lock(&intel_fdevs);
	}
	xa_unlock(&intel_fdevs);

	return 0;
}

static void fdev_wait_on_release(struct fdev *dev)
{
	fdev_put(dev);

	wait_for_completion_killable_timeout(&dev->fdev_released,
					     RELEASE_TIMEOUT);
}

/*
 * Returns with reference count for the fdev incremented when found.
 * It is the callers responsibility to decrement the reference count.
 */
struct fdev *fdev_find(u32 fabric_id)
{
	struct fdev *dev;

	xa_lock(&intel_fdevs);
	dev = fdev_get(xa_load(&intel_fdevs, fabric_id));
	xa_unlock(&intel_fdevs);

	return dev;
}

struct fdev *fdev_find_by_sd_guid(u64 guid)
{
	struct fdev *dev, *referenced_dev;
	unsigned long d;
	u8 s;

	xa_lock(&intel_fdevs);

	/* dev becomes NULL if xa_for_each() completes */
	xa_for_each(&intel_fdevs, d, dev)
		for (s = 0; s < dev->pd->sd_cnt; ++s)
			if (dev->sd[s].guid == guid)
				goto end_iteration;

end_iteration:

	referenced_dev = fdev_get(dev);

	xa_unlock(&intel_fdevs);

	return referenced_dev;
}

struct fsubdev *find_sd_id(u32 fabric_id, u8 sd_index)
{
	struct fdev *dev = fdev_find(fabric_id);

	if (!dev)
		return ERR_PTR(-ENODEV);

	if (sd_index < dev->pd->sd_cnt) {
		struct fsubdev *sd = &dev->sd[sd_index];

		/* prevents aggressive netlink use from accessing sd before we are ready */
		if (READ_ONCE(sd->fw_running))
			return sd;

		fdev_put(dev);

		return ERR_PTR(-EAGAIN);
	}

	fdev_put(dev);

	return ERR_PTR(-EINVAL);
}

struct fsubdev *find_routable_sd(u64 guid)
{
	struct fsubdev *sd = NULL;

	list_for_each_entry(sd, &routable_list, routable_link)
		if (sd->guid == guid)
			return sd;

	return NULL;
}

/* configured by request_irq() to refer to corresponding struct fsubdev */
static irqreturn_t handle_iaf_irq(int irq, void *arg)
{
	struct fsubdev *sd = arg;

	return mbdb_handle_irq(sd);
}

static struct query_info *handle_query(void *handle, u32 fabric_id)
{
	struct fdev *src, *dst;
	struct query_info *qi;

	/*
	 * src fdev is guaranteed to never be removed during this invocation by
	 * the caller
	 */
	src = handle;

	dst = fdev_find(fabric_id);
	if (!dst)
		return ERR_PTR(-ENODEV);

	qi = kmalloc(struct_size(qi, sd2sd, src->pd->sd_cnt * dst->pd->sd_cnt),
		     GFP_KERNEL);
	if (qi) {
		qi->src_cnt = src->pd->sd_cnt;
		qi->dst_cnt = dst->pd->sd_cnt;
		routing_p2p_lookup(src, dst, qi);
	} else {
		qi = ERR_PTR(-ENOMEM);
	}

	fdev_put(dst);

	return qi;
}

/**
 * mappings_ref_get - Increment mapping reference count
 * @dev: pointer to a valid fdev
 *
 * Allow the parent to indicate that a buffer has been mapped
 *
 * NOTE: NO other lock can be taken with mapping_ref lock held.
 * (see disable_<usage_>fports()).
 *
 * return: -1 reference count failed, do not allow mapping
 */
static int mappings_ref_get(struct fdev *dev)
{
	/* protect port_unroute_list read */
	down_write(&routable_lock); /* exclusive lock */
	mutex_lock(&dev->mappings_ref.lock);

	dev_dbg(fdev_dev(dev), "count: %d\n", dev->mappings_ref.count);

	if (!list_empty(&dev->port_unroute_list) ||
	    dev->mappings_ref.remove_in_progress) {
		mutex_unlock(&dev->mappings_ref.lock);
		up_write(&routable_lock);
		return -EBUSY;
	}

	dev->mappings_ref.count++;

	mutex_unlock(&dev->mappings_ref.lock);
	up_write(&routable_lock);

	return 0;
}

/**
 * mappings_ref_put - Decrement mapping reference count
 * @dev: pointer to a valid fdev
 *
 * Allow the parent to indicate that a buffer has been unmapped
 *
 * A negative value allows remove to proceed.
 *
 * return: -1 indicate an unbalanced put call
 */
static int mappings_ref_put(struct fdev *dev)
{
	int ret = 0;

	mutex_lock(&dev->mappings_ref.lock);

	dev_dbg(fdev_dev(dev), "count: %d\n", dev->mappings_ref.count);

	/*
	 * in the case of device removal, permit an extra decrement, which is
	 * used to signal completion to the remove context.
	 */
	if (dev->mappings_ref.count > 1 ||
	    dev->mappings_ref.remove_in_progress) {
		dev->mappings_ref.count--;

		if (dev->mappings_ref.count == 0)
			complete(&dev->mappings_ref.complete);

	} else {
		ret = -1;
	}

	mutex_unlock(&dev->mappings_ref.lock);

	return ret;
}

/**
 * mappings_ref_wait - Driver decrement (inited to 1), and wait for zero count
 * @dev: valid device
 *
 */
static void mappings_ref_wait(struct fdev *dev)
{
	mutex_lock(&dev->mappings_ref.lock);

	dev->mappings_ref.remove_in_progress = true;
	dev->mappings_ref.count--;

	if (dev->mappings_ref.count == 0) {
		mutex_unlock(&dev->mappings_ref.lock);
		return;
	}

	dev_warn(fdev_dev(dev), "mappings_ref_map != 0 (%d) will wait\n",
		 dev->mappings_ref.count);

	mutex_unlock(&dev->mappings_ref.lock);

	wait_for_completion_killable(&dev->mappings_ref.complete);
}

/**
 * mappings_ref_check - Determine if there are any pending mappings
 * @dev: valid device
 *
 * Return: true: mappings reference is in use
 */
bool mappings_ref_check(struct fdev *dev)
{
	int count;

	mutex_lock(&dev->mappings_ref.lock);
	count = dev->mappings_ref.count;
	mutex_unlock(&dev->mappings_ref.lock);

	return count > 1;
}

static int handle_parent_event(void *handle, enum iaf_parent_event event)
{
	struct fdev *dev = handle;

	switch (event) {
	case IAF_PARENT_PCIE_ERR:
		dev_err(fdev_dev(dev), "PCIE error received\n");
		WRITE_ONCE(dev->dev_disabled, true);
		return 0;

	case IAF_PARENT_MAPPING_GET:
		return mappings_ref_get(dev);

	case IAF_PARENT_MAPPING_PUT:
		return mappings_ref_put(dev);
	}

	return -EOPNOTSUPP;
}

static const struct iaf_ops iaf_ops = {
	.connectivity_query = handle_query,
	.parent_event = handle_parent_event,
};

static int validate_product(struct fsubdev *sd)
{
	if (dev_is_startup_debug(sd->fdev))
		return 0;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	switch (sd->fdev->pd->product) {
	case IAF_PONTEVECCHIO:
		sd->asic_rev_info = readq(sd->csr_base + CSR_ASIC_REV_INFO);
		if (!IS_ANR(sd)) {
			sd_err(sd, "Unsupported subdevice revision 0x%016llx\n",
			       sd->asic_rev_info);
			return -EINVAL;
		}
		sd_dbg(sd, "asic rev info 0x%016llx\n", sd->asic_rev_info);
		break;
	default:
		sd_err(sd, "Unrecognized product type %d for subdevice\n",
		       sd->fdev->pd->product);
		return -EINVAL;
	}

	return 0;
}

static struct resource *iaf_find_res(struct fdev *dev, unsigned int type,
				     int index)
{
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	int i;
	int offset;

	for (i = 0; i < dev->pd->num_resources; i++) {
		offset = index * dev->pd->sd_cnt + i;
		if (resource_type(&dev->pd->resources[offset]) == type)
			return &dev->pd->resources[offset];
	}

	return NULL;
#else
	return platform_get_resource(dev->pdev, type, index);
#endif
}

static int add_subdevice(struct fsubdev *sd, struct fdev *dev, int index)
{
	struct resource *res;
	int err;

	sd->fdev = dev;

	snprintf(sd->name, sizeof(sd->name), SD_ID_FMT, index);

	res = iaf_find_res(dev, IORESOURCE_MEM, index);
	if (!res) {
		sd_err(sd, "No MEM resource available\n");
		return -ENOMEM;
	}
	sd->csr_base = ioremap(res->start, resource_size(res));
	if (!sd->csr_base) {
		sd_err(sd, "Unable to map resource %pR to kvirt\n", res);
		return -ENOMEM;
	}
	sd_dbg(sd, "mapped resource %pR to mb_base %p\n", res, sd->csr_base);

	err = validate_product(sd);
	if (err) {
		iounmap(sd->csr_base);
		return err;
	}

	WRITE_ONCE(sd->fw_running, false);

	mutex_init(&sd->pm_work_lock);
	sd->ok_to_schedule_pm_work = false;

	res = iaf_find_res(dev, IORESOURCE_IRQ, index);
	if (!res) {
		sd_err(sd, "No IRQ resource available\n");
		iounmap(sd->csr_base);
		return -EINVAL;
	}
	sd_dbg(sd, "IRQ resource %pR\n", res);

	sd->irq = res->start;
	err = request_irq(sd->irq, handle_iaf_irq, 0, "intel_sd_irq", sd);
	if (err) {
		sd_err(sd, "failed to request_irq %d  err: %d\n", sd->irq, err);
		iounmap(sd->csr_base);
		return err;
	}

	mutex_init(&sd->cport_init_ctrl_reg_lock);

	create_dev_debugfs_dir(sd);

	err = create_mbdb(sd);

	if (err) {
		sd_err(sd, "Message Box allocation failure\n");
		free_irq(sd->irq, sd);
		iounmap(sd->csr_base);
		return err;
	}

	routing_sd_init(sd);

	sd_dbg(sd, "Adding IAF subdevice: %d\n", index);

	INIT_LIST_HEAD(&sd->routable_link);

	return 0;
}

static void remove_subdevice(struct fsubdev *sd)
{
	destroy_fports(sd);
	destroy_mbdb(sd);
	mutex_destroy(&sd->cport_init_ctrl_reg_lock);
	free_irq(sd->irq, sd);
	iounmap(sd->csr_base);
	sd_dbg(sd, "Removed IAF resource: %p\n", sd->csr_base);
	sd->csr_base = NULL;
}

/*
 * Cause a subdevice to (eventually) fail, synchronizing with routing. The full
 * effect may not be immediate.
 */
static void fail_subdevice(struct fsubdev *sd)
{
	if (test_and_set_bit(SD_ERROR_FAILED, sd->errors))
		return;

	sd_err(sd, "subdevice failed, shutting down\n");
}

/*
 * Indicate that there has been a subdevice error
 */
void indicate_subdevice_error(struct fsubdev *sd, enum sd_error err)
{
	if (test_and_set_bit(err, sd->errors))
		return;

	switch (err) {
	case SD_ERROR_FW:
		sd_err(sd, "FW error detected\n");
		break;

	case SD_ERROR_FAILED:
	default:
		sd_err(sd, "unknown error detected\n");
		break;
	}

	fail_subdevice(sd);
}

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
static void iaf_remove(struct auxiliary_device *pdev)
#else
static int iaf_remove(struct platform_device *pdev)
#endif
{
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	struct iaf_pdata *pd = container_of(pdev, struct iaf_pdata, aux_dev);
#else
	struct iaf_pdata *pd = dev_get_platdata(&pdev->dev);
#endif
	struct fdev *dev = dev_get_drvdata(&pdev->dev);
	u8 i;

	dev_dbg(&pdev->dev, "Removing %s\n", dev_name(&pdev->dev));

	/*
	 * let the parent know an unregister is coming and allow it
	 * to do some pre-cleanup if possible
	 */
	pd->dev_event(pd->parent, dev, IAF_DEV_REMOVE, NULL);

	mappings_ref_wait(dev);

	/*
	 * intentionally unregister with mei before the flush to maximize
	 * the chance that remove will prevent an outstanding async init
	 * from programming a new min SVN
	 */
	iaf_mei_stop(dev);

	/*
	 * NOTE: any remove steps performed prior to this flush will race
	 *       with the asynchronous device initialization
	 */
	flush_any_outstanding_fw_initializations(dev);

	remove_debugfs(dev);

	iaf_sysfs_remove(dev);

	pd->unregister_dev(pd->parent, dev);

	WRITE_ONCE(dev->registered, false);

	fdev_wait_on_release(dev);

	/*
	 * synchronize with routing to remove the device without disrupting
	 * traffic.  note that during driver unload, both the routing engine
	 * and event manager will be stopped, and this will not wait
	 */
	routing_dev_unroute(dev);

	for (i = 0; i < pd->sd_cnt; i++)
		remove_subdevice(&dev->sd[i]);

	routing_p2p_clear(dev);

	pm_runtime_put(fdev_dev(dev));
	pm_runtime_allow(fdev_dev(dev));
	pm_runtime_disable(fdev_dev(dev));

	WARN(kref_read(&dev->refs), "fabric_id 0x%08x has %u references",
	     dev->fabric_id, kref_read(&dev->refs));

	kfree(dev->psc.brand);
	kfree(dev->psc.product);
	for (i = 0; i < pd->sd_cnt; i++)
		if (!dev->psc.ini_buf[i].do_not_free)
			kfree(dev->psc.ini_buf[i].data);
	kfree(dev->psc.presence_rules);
	kfree(dev->psc.txcal);
	kfree(dev);
#if !IS_ENABLED(CONFIG_AUXILIARY_BUS)
	return 0;
#endif
}

static u32 dev_fabric_id(struct iaf_pdata *pd)
{
	return pd->product << PRODUCT_SHIFT | pd->index;
}

/**
 * iaf_complete_init_dev - Complete initialization for a device
 * @dev: the device to operate on
 *
 * called by load_and_init_subdev [fw.c] after last sd initialized
 * - set up sysfs
 * - indicate that all sd's for this device have been initialized
 * - commit software version number for anti-rollback
 */
void iaf_complete_init_dev(struct fdev *dev)
{
	/*
	 * currently works in all startup modes, could be conditioned here by
	 * skipping this call
	 */
	iaf_sysfs_init(dev);

	/*
	 * when appropriate, indicate that all sd's for this device have been
	 * fully initialized
	 */
	if (!dev_is_runtime_debug(dev))
		/* checked by netlink agent to reject early requests */
		smp_store_release(&dev->all_sds_inited, true);

	/*
	 * Poke the MEI submodule to indicate that the device is functioning properly. If enabled
	 * and currently allowed, automatic anti-rollback protection will be initiated.
	 */
	iaf_mei_indicate_device_ok(dev);

	dev_dbg(fdev_dev(dev), "device init complete\n");
}

static void read_gpio_link_config_pins(struct fdev *dev)
{
	u64 gpio_ctrl;

	if (unlikely(READ_ONCE(dev->dev_disabled)))
		return;

	gpio_ctrl = readq(dev->sd[0].csr_base + CSR_GPIO_CTRL);

	dev->link_config = FIELD_GET(GPIO_CTRL_PIN_LINK_CONFIG, gpio_ctrl);

	if (FIELD_GET(GPIO_CTRL_OE_LINK_CONFIG, gpio_ctrl))
		dev_warn(fdev_dev(dev), "Link config pins are configured as outputs: %02llx\n",
			 FIELD_GET(GPIO_CTRL_OE_LINK_CONFIG, gpio_ctrl));
}

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
static int iaf_probe(struct auxiliary_device *pdev, const struct auxiliary_device_id *id)
#else
static int iaf_probe(struct platform_device *pdev)
#endif
{
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	struct iaf_pdata *pd = container_of(pdev, struct iaf_pdata, aux_dev);
	struct device *dev_parent = pdev->dev.parent;
#else
	struct iaf_pdata *pd = dev_get_platdata(&pdev->dev);
	struct device *dev_parent = pdev->dev.parent;
#endif
	struct fdev *dev;
	u8 sds_added = 0;
	int err;
	u8 i;

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	dev_info(&pdev->dev,
		 "Probing %s, connection point for %s, fabric_id 0x%08x socket_id 0x%02x, sd_cnt %u\n",
		 dev_name(&pdev->dev),
		 dev_parent ? dev_name(dev_parent) : "UNKNOWN SOURCE",
		 dev_fabric_id(pd), pd->socket_id, pd->sd_cnt);
#else
	dev_info(&pdev->dev,
		 "Probing %s with MFD, connection point for %s, fabric_id 0x%08x socket_id 0x%02x, sd_cnt %u\n",
		 dev_name(&pdev->dev),
		 dev_parent ? dev_name(dev_parent) : "UNKNOWN SOURCE",
		 dev_fabric_id(pd), pd->socket_id, pd->sd_cnt);
#endif

	if (pd->version != IAF_VERSION)
		return -EPERM;

	if (!pd->register_dev || !pd->unregister_dev)
		return -EINVAL;

	if (pd->socket_id >= MAX_SOCKET_IDS)
		return -EINVAL;

	if (!pd->sd_cnt || pd->sd_cnt > IAF_MAX_SUB_DEVS)
		return -EINVAL;

	if (!pd->dpa.pkg_size)
		return -EINVAL;

	dev_dbg(&pdev->dev, "DPA offset: %dGB  size: %dGB\n",
		pd->dpa.pkg_offset, pd->dpa.pkg_size);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	kernel_param_lock(THIS_MODULE);
	dev->startup_mode = param_startup_mode;
	kernel_param_unlock(THIS_MODULE);

	if (dev->startup_mode != STARTUP_MODE_DEFAULT)
		dev_info(&pdev->dev, "startup_mode: %s",
			 startup_mode_name(dev->startup_mode));

	dev->fabric_id = dev_fabric_id(pd);
	dev->pd = pd;
	dev->pdev = pdev;

	dev_set_drvdata(&pdev->dev, dev);

	err = fdev_insert(dev);
	if (err) {
		kfree(dev);
		return err;
	}

	err = iaf_sysfs_probe(dev);
	if (err)
		goto sysfs_error;

	dev->mappings_ref.count = 1;
	mutex_init(&dev->mappings_ref.lock);
	init_completion(&dev->mappings_ref.complete);
	INIT_LIST_HEAD(&dev->port_unroute_list);

	/*
	 * The IAF cannot be suspended via runtime, but the parent could have
	 * the capability.  Set runtime to forbid to be explicit about what
	 * the requirement is.  Do a get to make sure that suspend cannot be
	 * started externally (i.e. sysfs), to keep the parent from going to
	 * sleep.
	 * If the parent is asleep at this time, pm_runtime_forbid will cause
	 * it to resume.
	 */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get(&pdev->dev);

	err = pd->register_dev(pd->parent, dev, dev->fabric_id, &iaf_ops);
	if (err) {
		iaf_sysfs_remove(dev);
		goto sysfs_error;
	}

	WRITE_ONCE(dev->registered, true);

	init_debugfs(dev);

	for (sds_added = 0; sds_added < pd->sd_cnt; ++sds_added) {
		err = add_subdevice(&dev->sd[sds_added], dev, sds_added);
		if (err)
			goto add_error;
	}

	err = iaf_mei_start(dev);
	if (err) {
		dev_err(fdev_dev(dev), "failed to register as MEI user\n");
		goto add_error;
	}

	/* read link config before initializing FW since it may also use GPIO */
	read_gpio_link_config_pins(dev);

	fw_init_dev(dev);

	err = load_and_init_fw(dev);
	if (err)
		goto load_error;

	return 0;

load_error:
	flush_any_outstanding_fw_initializations(dev);
	iaf_mei_stop(dev);

add_error:
	iaf_sysfs_remove(dev);
	for (i = 0; i < sds_added; ++i)
		remove_subdevice(&dev->sd[i]);

	pd->unregister_dev(pd->parent, dev);

sysfs_error:
	remove_debugfs(dev);
	dev->dir_node = NULL;
	fdev_wait_on_release(dev);
	pm_runtime_put(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	kfree(dev);
	return err;
}

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
static const struct auxiliary_device_id iaf_id_table[] = {
	{ .name = "i915.iaf", },
	{},
};

static struct auxiliary_driver iaf_driver = {
	.probe = iaf_probe,
	.remove = iaf_remove,
	.id_table = iaf_id_table,
};

#else

static struct platform_driver iaf_driver = {
	.driver = {
		.name           = DRIVER_NAME,
	},
	.probe                  = iaf_probe,
	.remove                 = iaf_remove,
};
#endif

/*
 * If the driver is waiting for the SPI driver to become available,
 * (to get PSCBIN information from the IFWI, short circuit the wait
 * time so unload can complete without waiting for the timeout.
 */
static void fw_abort(void)
{
	struct fdev *dev;
	unsigned long i;

	xa_lock(&intel_fdevs);
	xa_for_each(&intel_fdevs, i, dev)
		complete_all(&dev->psc.abort);
	xa_unlock(&intel_fdevs);
}

bool is_fdev_registered(struct fdev *dev)
{
	return READ_ONCE(dev->registered);
}

/*
 * Check that both ends of the port pair are registered.
 */
bool is_fport_registered(struct fport *port)
{
	if (is_fdev_registered(port->sd->fdev)) {
		struct fdev *neighbor_dev = fdev_find_by_sd_guid(port->portinfo->neighbor_guid);

		if (neighbor_dev) {
			bool registered = is_fdev_registered(neighbor_dev);

			fdev_put(neighbor_dev);
			return registered;
		}
	}

	return false;
}

static void __exit iaf_unload_module(void)
{
	pr_notice("Unloading %s\n", MODULEDETAILS);

	mbox_term_module();
	nl_term();

	fw_abort();

	flush_workqueue(iaf_unbound_wq);

	/* notify routing event manager to minimize delays since we are shutting down */
	rem_shutting_down();

	/* remove all existing iaf devices */
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	auxiliary_driver_unregister(&iaf_driver);
#else
	platform_driver_unregister(&iaf_driver);
#endif
	/* stop routing and destroy the network topology */
	rem_stop();
	routing_stop();
	routing_destroy();

	remove_debugfs_root_nodes();

	destroy_workqueue(iaf_unbound_wq);

	pr_notice("%s Unloaded\n", MODULEDETAILS);
}
module_exit(iaf_unload_module);

/*
 * \brief Loads the module into kernel space and does global initializations.
 * Called by insmod or modprobe.
 */
static int __init iaf_load_module(void)
{
	int err;

	pr_notice("Initializing %s\n", MODULEDETAILS);
	pr_debug("Built for Linux Kernel %s\n", UTS_RELEASE);

	iaf_unbound_wq = alloc_workqueue("iaf_unbound_wq", WQ_UNBOUND, WQ_UNBOUND_MAX_ACTIVE);
	if (!iaf_unbound_wq)
		return -ENOMEM;

	create_debugfs_root_nodes();

#if IS_ENABLED(CONFIG_IAF_DEBUG_SELFTESTS)
	if (selftests_run()) {
		err = -ENODEV;
		goto exit;
	}
#endif

	routing_init();
	rem_init();
	mbdb_init_module();
	mbox_init_module();
	err = nl_init();
	if (err)
		goto exit;
	fw_init_module();

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	err = auxiliary_driver_register(&iaf_driver);
	if (err)
		pr_err("Cannot register with auxiliary bus\n");
#else
	err = platform_driver_register(&iaf_driver);
	if (err)
		pr_err("Cannot register with platform bus\n");
#endif

exit:
	if (err) {
		mbox_term_module();
		nl_term();
		remove_debugfs_root_nodes();
		destroy_workqueue(iaf_unbound_wq);
	}

	return err;
}
module_init(iaf_load_module);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION(MODULEDETAILS);
MODULE_LICENSE("GPL and additional rights");
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
MODULE_ALIAS("auxiliary:i915.iaf");
#else
MODULE_ALIAS("platform:iaf");
#endif
MODULE_FIRMWARE("i915/pvc_iaf_ver1.bin");
MODULE_FIRMWARE("i915/pvc_iaf_ver1d.bin");
MODULE_FIRMWARE("i915/pvc_iaf_ver1e.bin");
MODULE_FIRMWARE("i915/default_iaf.pscbin");
