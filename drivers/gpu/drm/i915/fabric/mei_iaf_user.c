// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 */

#include <linux/device.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/module.h>
#include <drm/i915_component.h>
#include <drm/i915_mei_iaf_interface.h>

#include "iaf_drv.h"
#include "mei_iaf_user.h"

/*
 * This submodule provides anti-rollback (ARB) support using the MEI interface. ARB uses a simple
 * software version number (SVN) scheme and is checked automatically by the boot loader. The
 * required minimum SVN value is written by a protected processor which is accessed via the MEI
 * component-based interface.
 */

/*
 * module parameters
 */

/* MEI interaction timeout */
static unsigned short mei_timeout = 5 * 60;
module_param(mei_timeout, ushort, 0400);
MODULE_PARM_DESC(mei_timeout,
		 "Seconds to wait for MEI before warning (default: 5 minutes)");

/* MEI automatic SVN request on init */
static bool automatic_rollback_protection;
module_param(automatic_rollback_protection, bool, 0600);
MODULE_PARM_DESC(automatic_rollback_protection,
		 "Protect booted/initialized FW against rollback (default: N)");

/*
 * register access helper functions - can be called at any time
 */

/**
 * get_min_svn - query currently-programmed minimum required SVN value
 * @dev: device being queried
 *
 * Return: SVN value
 */
u16 get_min_svn(struct fdev *dev)
{
	void __iomem *fw_min_svn_reg_addr;
	u64 fw_min_svn_reg_val;

	if (unlikely(READ_ONCE(dev->dev_disabled)))
		return 0;

	/* tile 0 is the master for SVN */
	fw_min_svn_reg_addr = dev->sd[0].csr_base + FW_MIN_SVN_ADDR;

	fw_min_svn_reg_val = readq(fw_min_svn_reg_addr);

	return fw_min_svn_reg_val & FW_SVN_MASK;
}

/**
 * get_updated_svn - query currently-programmed requested new minimum SVN value
 * @dev: device being queried
 *
 * Return: SVN value
 */
static u16 get_updated_svn(struct fdev *dev)
{
	void __iomem *fw_updated_svn_reg_addr;
	u64 fw_updated_svn_reg_val;

	if (unlikely(READ_ONCE(dev->dev_disabled)))
		return 0;

	/* tile 0 is the master for SVN */
	fw_updated_svn_reg_addr = dev->sd[0].csr_base + FW_UPDATED_SVN_ADDR;

	fw_updated_svn_reg_val = readq(fw_updated_svn_reg_addr);

	return fw_updated_svn_reg_val & FW_SVN_MASK;
}

/*
 * SVN commit support - only used once requested by the ARB request interface
 */

/**
 * queue_commit_svn_work - trigger a work queue to complete SVN update
 * @dev: device being queried
 *
 * Called automatically as a "continuation function" if the MEI binds after an SVN update was
 * automatically triggered.
 */
static void queue_commit_svn_work(struct fdev *dev)
{
	queue_work(iaf_unbound_wq, &dev->mei_work);
}

/**
 * mei_iaf_user_continuation - set or clear a "continuation function" for a not-yet-bound MEI
 * @dev: device being registered
 * @fn: function to call upon successful MEI bind, NULL to clear
 *
 * When set, the "continuation function" is called by the component bind operation.
 *
 * Return: 0 if successful, -EPERM if continuation is already pending on set request
 */
static int mei_iaf_user_continuation(struct fdev *dev, void (*fn)(struct fdev *))
{
	if (fn && (dev->mei_bind_continuation || timer_pending(&dev->mei_continuation_timer)))
		return -EPERM;

	if (fn)
		mod_timer(&dev->mei_continuation_timer, jiffies + mei_timeout * HZ);
	else if (timer_pending(&dev->mei_continuation_timer))
		del_timer_sync(&dev->mei_continuation_timer);

	dev->mei_bind_continuation = fn;

	return 0;
}

/* non-error values for continuation results */
#define COMPLETED_IMMEDIATELY	(2)
#define CONTINUATION_DISALLOWED	(1)
#define CONTINUATION_SCHEDULED	(0)

/* convenience macros for specifying allow_continuation value */
#define AUTOMATICALLY_RETRY	(true)
#define TRY_ONCE		(false)

/**
 * try_commit_svn - try to commit current software version number
 * @dev: the device to operate on
 * @allow_continuation: whether to automatically retry when MEI is bound
 *
 * Use the MEI driver to commit the current active SVN. If the MEI driver is not currently bound to
 * this driver and @allow_continuation is set, schedule a contination request.
 *
 * Return: 0 on success, non-zero on failure
 */
static int try_commit_svn(struct fdev *dev, bool allow_continuation)
{
	int status = COMPLETED_IMMEDIATELY;
	int exp_mei_err = 0;
	int mei_err = 0;
	u16 old_min_svn;
	u16 updated_svn;
	u16 new_min_svn;
	u8 i;

	old_min_svn = get_min_svn(dev);

	for (i = 0; i < dev->pd->sd_cnt; i++)
		if (test_bit(SD_ERROR_FAILED, dev->sd[i].errors))
			return -ENODEV;

	mutex_lock(&dev->mei_ops_lock);

	if (dev->mei_ops)
		mei_err = dev->mei_ops->commit_svn(dev->mei_dev);
	else if (allow_continuation)
		status = mei_iaf_user_continuation(dev, queue_commit_svn_work);
	else
		status = CONTINUATION_DISALLOWED;

	mutex_unlock(&dev->mei_ops_lock);

	switch (status) {
	case COMPLETED_IMMEDIATELY:
		/* operation finished, go on and check mei results */
		break;

	case CONTINUATION_DISALLOWED:
		/* retry disallowed, caller reports when appropriate */
		return -ENODEV;

	case CONTINUATION_SCHEDULED:
		/* operation will be automatically tried again */
		dev_dbg(fdev_dev(dev), "MEI not ready, SVN update deferred\n");
		return 0;

	default:
		dev_err(fdev_dev(dev), "error continuing MEI, code %d\n",
			status);
		return status;
	}

	/* check and/or propagate result of MEI call */

	updated_svn = get_updated_svn(dev);

	if (updated_svn < old_min_svn)
		exp_mei_err = -EBADF;
	else if (updated_svn == old_min_svn)
		exp_mei_err = -EACCES;

	new_min_svn = get_min_svn(dev);

	switch (mei_err) {
	case 0:
		dev_info(fdev_dev(dev), "Minimum SVN updated to %u\n", new_min_svn);
		if (new_min_svn != updated_svn) {
			dev_err(fdev_dev(dev), "Confirmed SVN update not detected, %u != %u\n",
				new_min_svn, updated_svn);
			return -EIO;
		}
		break;

	case -ENOENT:
		dev_dbg(fdev_dev(dev), "SVN update (%u -> %u) disabled, remains %u\n",
			old_min_svn, updated_svn, new_min_svn);
		return mei_err;

	case -EACCES:
		dev_dbg(fdev_dev(dev), "SVN same (%u == %u), not updated, remains %u\n",
			old_min_svn, updated_svn, new_min_svn);
		break;

	case -EBADF:
		dev_dbg(fdev_dev(dev), "SVN smaller (%u > %u), not updated, remains %u\n",
			old_min_svn, updated_svn, new_min_svn);
		break;

	default:
		dev_err(fdev_dev(dev), "SVN update failed, code %d\n", mei_err);
		return mei_err;
	}

	/*
	 * non-exceptional case: SVN has been updated or not based on min_svn/updated_svn values;
	 * also verify that the driver and MEI/CSC views match since this is a security feature
	 */
	if (mei_err != exp_mei_err) {
		dev_err(fdev_dev(dev), "Expected MEI errcode %d, got %d\n", exp_mei_err, mei_err);
		return -EIO;
	}

	return 0;
}

/**
 * commit_svn_work - commit requested SVN from a work queue
 * @work: the MEI work queue for the device to commit
 *
 * Triggered if the MEI binds after an SVN update was requested.
 */
static void commit_svn_work(struct work_struct *work)
{
	struct fdev *dev;
	int err;

	dev = container_of(work, typeof(*dev), mei_work);
	err = try_commit_svn(dev, TRY_ONCE);
	if (err && err != -ENOENT)
		dev_err(fdev_dev(dev),
			"Error %d, bound MEI unable to commit IAF FW version\n",
			-err);
}

/*
 * component interface - for this driver to request services from the MEI driver
 */

static int bind_to_mei(struct device *dev, struct device *mei_dev, void *data)
{
	struct fdev *fdev = dev_get_drvdata(dev);
	struct i915_iaf_component_ops *mei_ops = data;

	dev_dbg(dev, "binding to MEI device %s\n", dev_name(mei_dev));

	mutex_lock(&fdev->mei_ops_lock);

	fdev->mei_dev = mei_dev;
	fdev->mei_ops = mei_ops;

	if (fdev->mei_bind_continuation)
		fdev->mei_bind_continuation(fdev);

	fdev->mei_bind_continuation = NULL;

	mutex_unlock(&fdev->mei_ops_lock);

	return 0;
}

static void unbind_from_mei(struct device *dev, struct device *mei_dev,
			    void *data)
{
	struct fdev *fdev = dev_get_drvdata(dev);
	struct i915_iaf_component_ops *mei_ops = data;

	dev_dbg(dev, "unbinding from MEI device %s\n", dev_name(mei_dev));

	mutex_lock(&fdev->mei_ops_lock);

	if (fdev->mei_bind_continuation)
		dev_err(dev, "MEI unbound while bind continuation pending\n");

	if (fdev->mei_ops != mei_ops)
		dev_warn(dev, "MEI unbound OPs differ from bound OPs\n");

	fdev->mei_bind_continuation = NULL;
	fdev->mei_dev = NULL;

	fdev->mei_ops = NULL;

	mutex_unlock(&fdev->mei_ops_lock);
}

static const struct component_ops bind_ops = {
	.bind = bind_to_mei,
	.unbind = unbind_from_mei,
};

/*
 * component system support interface
 */

/**
 * mei_continuation_timeout - indicate that no MEI driver has registered
 * @timer: MEI timer for device being registered
 *
 * If no MEI module has registered AND an automatic SVN update was requested, warn the
 * administrator. Currently the only action taken is logging a kernel warning.
 */
static void mei_continuation_timeout(struct timer_list *timer)
{
	struct fdev *dev = from_timer(dev, timer, mei_continuation_timer);

	if (dev->mei_bind_continuation)
		dev_warn(fdev_dev(dev),
			 "No MEI driver registered after %u seconds\n",
			 mei_timeout);
}

/**
 * mei_iaf_user_register - register for MEI access using component system
 * @dev: device to register
 *
 * Return: 0 if successful, non-zero if component add request fails
 */
static int mei_iaf_user_register(struct fdev *dev)
{
	dev_dbg(fdev_dev(dev), "Registering as MEI_IAF user\n");

	timer_setup(&dev->mei_continuation_timer, mei_continuation_timeout, 0);

	return component_add_typed(fdev_dev(dev), &bind_ops,
				   I915_COMPONENT_IAF);
}

/**
 * mei_iaf_user_unregister - unregister for MEI access using component system
 * @dev: device to register
 */
static void mei_iaf_user_unregister(struct fdev *dev)
{
	dev_dbg(fdev_dev(dev), "Unregistering as MEI_IAF user\n");

	if (timer_pending(&dev->mei_continuation_timer))
		del_timer_sync(&dev->mei_continuation_timer);

	component_del(fdev_dev(dev), &bind_ops);
}

/*
 * ARB request interface - external interface used to request SVN updates for ARB
 */

/**
 * iaf_mei_start - configure the driver for MEI requests
 * @dev: device to configure
 *
 * must be called before calling iaf_mei_indicate_device_ok or iaf_commit_svn
 *
 * Return: 0 if successful, non-zero if request fails
 */
int iaf_mei_start(struct fdev *dev)
{
	mutex_init(&dev->mei_ops_lock);

	INIT_WORK(&dev->mei_work, commit_svn_work);

	return mei_iaf_user_register(dev);
}

/**
 * iaf_mei_stop - deconfigure the driver for MEI requests
 * @dev: device to deconfigure
 *
 * called during device removal process
 */
void iaf_mei_stop(struct fdev *dev)
{
	cancel_work_sync(&dev->mei_work);
	mei_iaf_user_unregister(dev);
}

/**
 * iaf_mei_indicate_device_ok - indicate device is functioning, initiate ARB as appropriate
 * @dev: functioning device
 *
 * During regular booting, ARB is initiated once the device is deemed functioning properly if the
 * automatic_rollback_protection module parameter is set. It can also be initiated iaf_commit_svn.
 */
void iaf_mei_indicate_device_ok(struct fdev *dev)
{
	if (automatic_rollback_protection &&
	    dev->startup_mode == STARTUP_MODE_DEFAULT) {
		int err;

		err = try_commit_svn(dev, AUTOMATICALLY_RETRY);
		if (err && err != -ENOENT)
			dev_err(fdev_dev(dev), "unable to commit FW version, error %d\n", err);
	} else {
		if (is_fdev_registered(dev))
			pr_info_once("FW version not automatically committed\n");
	}
}

/**
 * iaf_commit_svn - commit current software version number for ARB
 * @dev: the device to operate on
 *
 * Use the MEI driver, if registered, to commit the current active software version number for
 * anti-rollback protection, and prevent automatic continuation if the MEI driver is not currently
 * bound to this driver. This is bound to a sysfs entry and errors propagate to the requestor.
 *
 * Return: 0 on success, non-zero if MEI not registered or on other failures
 */
int iaf_commit_svn(struct fdev *dev)
{
	mei_iaf_user_continuation(dev, NULL);
	return try_commit_svn(dev, TRY_ONCE);
}
