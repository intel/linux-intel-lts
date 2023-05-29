// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2019-2022, Intel Corporation. All rights reserved.
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 */

#include <linux/module.h>
#include <linux/mei_aux.h>
#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kthread.h>

#include "mei_dev.h"
#include "hw-me.h"
#include "hw-me-regs.h"

#include "mei-trace.h"

#define MEI_GSC_RPM_TIMEOUT 2000

static inline bool mei_gsc_hw_is_unavailable(const struct device *dev)
{
	return dev->offline;
}

static int mei_gsc_read_hfs(const struct mei_device *dev, int where, u32 *val)
{
	struct mei_me_hw *hw = to_me_hw(dev);

	*val = ioread32(hw->mem_addr + where + 0xC00);

	return 0;
}

static void mei_gsc_set_ext_op_mem(const struct mei_me_hw *hw, struct resource *mem)
{
	u32 low = lower_32_bits(mem->start);
	u32 hi  = upper_32_bits(mem->start);
	u32 limit = (resource_size(mem) / SZ_4K) | GSC_EXT_OP_MEM_VALID;

	iowrite32(low, hw->mem_addr + H_GSC_EXT_OP_MEM_BASE_ADDR_LO_REG);
	iowrite32(hi, hw->mem_addr + H_GSC_EXT_OP_MEM_BASE_ADDR_HI_REG);
	iowrite32(limit, hw->mem_addr + H_GSC_EXT_OP_MEM_LIMIT_REG);
}

/**
 * mei_gsc_mecbrw_read_null - read 32bit data from ME circular buffer (empty implementation)
 *  read window register
 *
 * @dev: the device structure
 *
 * Return: always 0
 */
static inline u32 mei_gsc_mecbrw_read_null(const struct mei_device *dev)
{
	return 0;
}

/**
 * mei_gsc_trc_status_null - read trc status register (empty implementation)
 *
 * @dev: mei device
 * @trc: trc status register value
 *
 * Return: always 0
 */
static int mei_gsc_trc_status_null(struct mei_device *dev, u32 *trc)
{
	*trc = 0;
	return 0;
}

/**
 * mei_gsc_fw_status_null - read fw status register from pci config space (empty implementation)
 *
 * @dev: mei device
 * @fw_status: fw status register values
 *
 * Return: always 0
 */
static int mei_gsc_fw_status_null(struct mei_device *dev,
				  struct mei_fw_status *fw_status)
{
	return 0;
}

/**
 * mei_gsc_hw_config_null - configure hw dependent settings (empty implementation)
 *
 * @dev: mei device
 *
 * Return: always 0
 *
 */
static int mei_gsc_hw_config_null(struct mei_device *dev)
{
	return 0;
}

/**
 * mei_gsc_pg_state_null  - translate internal pg state (empty implementation)
 *   to the mei power gating state
 *
 * @dev:  mei device
 *
 * Return: always MEI_PG_OFF
 */
static inline enum mei_pg_state mei_gsc_pg_state_null(struct mei_device *dev)
{
	return MEI_PG_OFF;
}

/**
 * mei_gsc_intr_clear_null - clear and stop interrupts (empty implementation)
 *
 * @dev: the device structure
 */
static void mei_gsc_intr_clear_null(struct mei_device *dev)
{
}

/**
 * mei_gsc_intr_enable_null - enables mei device interrupts (empty implementation)
 *
 * @dev: the device structure
 */
static void mei_gsc_intr_enable_null(struct mei_device *dev)
{
}

/**
 * mei_gsc_intr_disable_null - disables mei device interrupts (empty implementation)
 *
 * @dev: the device structure
 */
static void mei_gsc_intr_disable_null(struct mei_device *dev)
{
}

/**
 * mei_gsc_synchronize_irq_null - wait for pending IRQ handlers (empty implementation)
 *
 * @dev: the device structure
 */
static void mei_gsc_synchronize_irq_null(struct mei_device *dev)
{
}

/**
 * mei_gsc_host_is_ready_null - check whether the host has turned ready (empty implementation)
 *
 * @dev: mei device
 * Return: always true
 */
static bool mei_gsc_host_is_ready_null(struct mei_device *dev)
{
	return true;
}

/**
 * mei_gsc_hw_start_null - hw start routine (empty implementation)
 *
 * @dev: mei device
 * Return: always 0
 */
static int mei_gsc_hw_start_null(struct mei_device *dev)
{
	return 0;
}

/**
 * mei_gsc_hbuf_is_empty_null - checks if host buffer is empty (empty implementation)
 *
 * @dev: the device structure
 *
 * Return: always true
 */
static bool mei_gsc_hbuf_is_empty_null(struct mei_device *dev)
{
	return true;
}

/**
 * mei_gsc_hbuf_empty_slots_null - counts write empty slots (empty implementation)
 *
 * @dev: the device structure
 *
 * Return: always -EOVERFLOW
 */
static int mei_gsc_hbuf_empty_slots_null(struct mei_device *dev)
{
	return -EOVERFLOW;
}

/**
 * mei_gsc_hbuf_depth_null - returns depth of the hw buffer (empty implementation)
 *
 * @dev: the device structure
 *
 * Return: always 1
 */
static u32 mei_gsc_hbuf_depth_null(const struct mei_device *dev)
{
	return 0;
}

/**
 * mei_gsc_hbuf_write_null - writes a message to host hw buffer (empty implementation)
 *
 * @dev: the device structure
 * @hdr: header of message
 * @hdr_len: header length in bytes: must be multiplication of a slot (4bytes)
 * @data: payload
 * @data_len: payload length in bytes
 *
 * Return: always 0
 */
static int mei_gsc_hbuf_write_null(struct mei_device *dev,
				   const void *hdr, size_t hdr_len,
				   const void *data, size_t data_len)
{
	return 0;
}

/**
 * mei_gsc_count_full_read_slots_null - counts read full slots (empty implementation)
 *
 * @dev: the device structure
 *
 * Return: always -EOVERFLOW
 */
static int mei_gsc_count_full_read_slots_null(struct mei_device *dev)
{
	return -EOVERFLOW;
}

/**
 * mei_gsc_read_slots_null - reads a message from mei device (empty implementation)
 *
 * @dev: the device structure
 * @buffer: message buffer will be written
 * @buffer_length: message size will be read
 *
 * Return: always 0
 */
static int mei_gsc_read_slots_null(struct mei_device *dev, unsigned char *buffer,
				   unsigned long buffer_length)
{
	return 0;
}

/**
 * mei_gsc_pg_in_transition_null - is device now in pg transition (empty implementation)
 *
 * @dev: the device structure
 *
 * Return: always false
 */
static bool mei_gsc_pg_in_transition_null(struct mei_device *dev)
{
	return false;
}

/**
 * mei_gsc_pg_is_enabled_null - detect if PG is supported by HW (empty implementation)
 *
 * @dev: the device structure
 *
 * Return: always false
 */
static bool mei_gsc_pg_is_enabled_null(struct mei_device *dev)
{
	return false;
}

/**
 * mei_gsc_hw_is_ready_null - check whether the me(hw) has turned ready (empty implementation)
 *
 * @dev: mei device
 * Return: always true
 */
static bool mei_gsc_hw_is_ready_null(struct mei_device *dev)
{
	return true;
}

/**
 * mei_gsc_hw_reset_null - resets fw via mei csr register (empty implementation)
 *
 * @dev: the device structure
 * @intr_enable: if interrupt should be enabled after reset.
 *
 * Return: always 0
 */
static int mei_gsc_hw_reset_null(struct mei_device *dev, bool intr_enable)
{
	return 0;
}

/**
 * mei_gsc_forcewake_get_null - get forcewake counter (empty implementation)
 *
 * @dev: mei device
 * Return: always true
 */
static int mei_gsc_forcewake_get_null(struct mei_device *dev)
{
	return 0;
}

/**
 * mei_gsc_forcewake_put_null - put forcewake counter (empty implementation)
 *
 * @dev: mei device
 * Return: always true
 */
static int mei_gsc_forcewake_put_null(struct mei_device *dev)
{
	return 0;
}

static const struct mei_hw_ops mei_gsc_hw_ops_null = {
	.trc_status = mei_gsc_trc_status_null,
	.fw_status = mei_gsc_fw_status_null,
	.pg_state  = mei_gsc_pg_state_null,

	.host_is_ready = mei_gsc_host_is_ready_null,

	.hw_is_ready = mei_gsc_hw_is_ready_null,
	.hw_reset = mei_gsc_hw_reset_null,
	.hw_config = mei_gsc_hw_config_null,
	.hw_start = mei_gsc_hw_start_null,

	.pg_in_transition = mei_gsc_pg_in_transition_null,
	.pg_is_enabled = mei_gsc_pg_is_enabled_null,

	.intr_clear = mei_gsc_intr_clear_null,
	.intr_enable = mei_gsc_intr_enable_null,
	.intr_disable = mei_gsc_intr_disable_null,
	.synchronize_irq = mei_gsc_synchronize_irq_null,

	.hbuf_free_slots = mei_gsc_hbuf_empty_slots_null,
	.hbuf_is_ready = mei_gsc_hbuf_is_empty_null,
	.hbuf_depth = mei_gsc_hbuf_depth_null,

	.write = mei_gsc_hbuf_write_null,

	.rdbuf_full_slots = mei_gsc_count_full_read_slots_null,
	.read_hdr = mei_gsc_mecbrw_read_null,
	.read = mei_gsc_read_slots_null,

	.forcewake_get = mei_gsc_forcewake_get_null,
	.forcewake_put = mei_gsc_forcewake_put_null,
};

#define MEI_GSC_RESET_BEGIN_TIMEOUT 300
#define MEI_GSC_RESET_END_TIMEOUT 700
#define MEI_GSC_RESET_STEP 20

static int mei_gsc_forcewake_get_and_wait(struct mei_device *dev, bool need_runtime_pm)
{
	struct mei_fw_status fw_status;
	int timeout;
	int ret;

	if (!dev->forcewake_needed || dev->gt_forcewake_init_on)
		return 0;

	/* grab the GT forcewake bit to prevent RC6-exit which would cause fw reset */
	dev->ops->forcewake_get(dev);
	dev->gt_forcewake_init_on = true;
	/* wait for FW going to reset */
	for (timeout = MEI_GSC_RESET_BEGIN_TIMEOUT; timeout >= 0; timeout -= MEI_GSC_RESET_STEP) {
		ret = mei_fw_status(dev, &fw_status);
		if (ret) {
			dev_err(dev->dev, "failed to read fw sts: %d\n", ret);
			dev->gt_forcewake_init_on = false;
			dev->ops->forcewake_put(dev);
			return ret;
		}
		dev_dbg(dev->dev, "forcewake: fw sts: %d\n", fw_status.status[0]);
		if (!(fw_status.status[0] & PCI_CFG_HFS_1_INITSTATE))
			break;
		msleep(20);
	}
	dev_dbg(dev->dev, "forcewake: after wake fw sts: %d\n", fw_status.status[0]);

	/* wait to FW going out of reset */
	if (fw_status.status[0] & PCI_CFG_HFS_1_INITSTATE)
		goto runtime_pm;
	for (timeout = MEI_GSC_RESET_END_TIMEOUT; timeout >= 0; timeout -= MEI_GSC_RESET_STEP) {
		ret = mei_fw_status(dev, &fw_status);
		if (ret) {
			dev_err(dev->dev, "failed to read fw sts: %d\n", ret);
			dev->gt_forcewake_init_on = false;
			dev->ops->forcewake_put(dev);
			return ret;
		}
		dev_dbg(dev->dev, "forcewake: fw sts: %d\n", fw_status.status[0]);
		if (fw_status.status[0] & PCI_CFG_HFS_1_INITSTATE)
			break;
		msleep(20);
	}

	if (!(fw_status.status[0] & PCI_CFG_HFS_1_INITSTATE)) {
		dev_err(dev->dev, "forcewake: FW not back from reset: %d\n", fw_status.status[0]);
		dev->gt_forcewake_init_on = false;
		dev->ops->forcewake_put(dev);
		return -ENODEV;
	}

runtime_pm:
	if (need_runtime_pm) {
		/*
		 * Our runtime_pm configured to start as resumed.
		 * Take additional forcewake and runtime pm to avoid RC6 while initializing
		 */
		dev->ops->forcewake_get(dev);
		pm_runtime_get_noresume(dev->dev);
	}
	return 0;
}

static void mei_gsc_forcewake_put(struct mei_device *dev, bool need_runtime_pm)
{
	mutex_lock(&dev->device_lock);

	if (dev->forcewake_needed && dev->gt_forcewake_init_on) {
		dev->ops->forcewake_put(dev);
		dev->gt_forcewake_init_on = false;

		if (need_runtime_pm) {
			dev->ops->forcewake_put(dev);
			pm_runtime_put_noidle(dev->dev);
		}
	}

	mutex_unlock(&dev->device_lock);
}

static int mei_gsc_probe(struct auxiliary_device *aux_dev,
			 const struct auxiliary_device_id *aux_dev_id)
{
	struct mei_aux_device *adev = auxiliary_dev_to_mei_aux_dev(aux_dev);
	struct mei_device *dev;
	struct mei_me_hw *hw;
	struct device *device;
	const struct mei_cfg *cfg;
	int ret;

	cfg = mei_me_get_cfg(aux_dev_id->driver_data);
	if (!cfg)
		return -ENODEV;

	device = &aux_dev->dev;

	dev = mei_me_dev_init(device, cfg, adev->slow_firmware);
	if (!dev) {
		ret = -ENOMEM;
		goto err;
	}

	hw = to_me_hw(dev);
	hw->mem_addr = devm_ioremap_resource(device, &adev->bar);
	if (IS_ERR(hw->mem_addr)) {
		ret = PTR_ERR(hw->mem_addr);
		goto err;
	}

	hw->irq = adev->irq;
	hw->read_fws = mei_gsc_read_hfs;

	/* forcewake */
	dev->forcewake_needed = adev->forcewake_needed;
	hw->gsc = adev->gsc;
	hw->forcewake_get = adev->forcewake_get;
	hw->forcewake_put = adev->forcewake_put;

	dev_set_drvdata(device, dev);

	if (adev->ext_op_mem.start) {
		mei_gsc_set_ext_op_mem(hw, &adev->ext_op_mem);
		dev->pxp_mode = MEI_DEV_PXP_INIT;
	}

	ret = mei_gsc_forcewake_get_and_wait(dev, true);
	if (ret)
		goto err;

	/* use polling */
	if (mei_me_hw_use_polling(hw)) {
		mei_disable_interrupts(dev);
		mei_clear_interrupts(dev);
		init_waitqueue_head(&hw->wait_active);
		hw->is_active = true; /* start in active mode for initialization */
		hw->polling_thread = kthread_run(mei_me_polling_thread, dev,
						 "kmegscirqd/%s", dev_name(device));
		if (IS_ERR(hw->polling_thread)) {
			ret = PTR_ERR(hw->polling_thread);
			dev_err(device, "unable to create kernel thread: %d\n", ret);
			goto irq_err;
		}
	} else {
		ret = devm_request_threaded_irq(device, hw->irq,
						mei_me_irq_quick_handler,
						mei_me_irq_thread_handler,
						IRQF_ONESHOT, KBUILD_MODNAME, dev);
		if (ret) {
			dev_err(device, "irq register failed %d\n", ret);
			goto irq_err;
		}
	}

	pm_runtime_get_noresume(device);
	pm_runtime_set_active(device);
	pm_runtime_enable(device);

	/* Continue to char device setup in spite of firmware handshake failure.
	 * In order to provide access to the firmware status registers to the user
	 * space via sysfs.
	 */
	if (mei_start(dev))
		dev_warn(device, "init hw failure.\n");

	pm_runtime_set_autosuspend_delay(device, MEI_GSC_RPM_TIMEOUT);
	pm_runtime_use_autosuspend(device);
	dev_pm_set_driver_flags(device, DPM_FLAG_NO_DIRECT_COMPLETE);

	ret = mei_register(dev, device);
	if (ret)
		goto register_err;

	pm_runtime_put_noidle(device);
	return 0;

register_err:
	mei_stop(dev);
	if (!mei_me_hw_use_polling(hw))
		devm_free_irq(device, hw->irq, dev);

irq_err:
	mei_gsc_forcewake_put(dev, true);
err:
	dev_err(device, "probe failed: %d\n", ret);
	dev_set_drvdata(device, NULL);
	return ret;
}

static void mei_gsc_remove(struct auxiliary_device *aux_dev)
{
	struct mei_device *dev;
	struct mei_me_hw *hw;

	dev = dev_get_drvdata(&aux_dev->dev);
	hw = to_me_hw(dev);
	if (mei_gsc_hw_is_unavailable(&aux_dev->dev))
		dev->ops = &mei_gsc_hw_ops_null;

	mei_stop(dev);

	mei_gsc_forcewake_put(dev, true);

	hw = to_me_hw(dev);
	if (mei_me_hw_use_polling(hw))
		kthread_stop(hw->polling_thread);

	mei_deregister(dev);

	pm_runtime_disable(&aux_dev->dev);

	mei_disable_interrupts(dev);
	if (!mei_me_hw_use_polling(hw))
		devm_free_irq(&aux_dev->dev, hw->irq, dev);
}

static int __maybe_unused mei_gsc_pm_suspend(struct device *device)
{
	struct mei_device *dev = dev_get_drvdata(device);

	mei_stop(dev);

	mei_disable_interrupts(dev);

	return 0;
}

static int __maybe_unused mei_gsc_pm_resume(struct device *device)
{
	struct mei_device *dev = dev_get_drvdata(device);
	struct auxiliary_device *aux_dev;
	struct mei_aux_device *adev;
	int ret;
	struct mei_me_hw *hw;

	mutex_lock(&dev->device_lock);
	ret = mei_gsc_forcewake_get_and_wait(dev, false);
	mutex_unlock(&dev->device_lock);
	if (ret)
		return ret;

	hw = to_me_hw(dev);
	aux_dev = to_auxiliary_dev(device);
	adev = auxiliary_dev_to_mei_aux_dev(aux_dev);
	if (adev->ext_op_mem.start) {
		mei_gsc_set_ext_op_mem(hw, &adev->ext_op_mem);
		dev->pxp_mode = MEI_DEV_PXP_INIT;
	}

	ret = mei_restart(dev);
	if (ret) {
		mei_gsc_forcewake_put(dev, false);
		return ret;
	}

	/* Start timer if stopped in suspend */
	schedule_delayed_work(&dev->timer_work, HZ);

	return 0;
}

static int __maybe_unused mei_gsc_pm_runtime_idle(struct device *device)
{
	struct mei_device *dev = dev_get_drvdata(device);

	if (mei_write_is_idle(dev))
		pm_runtime_autosuspend(device);

	return -EBUSY;
}

static int  __maybe_unused mei_gsc_pm_runtime_suspend(struct device *device)
{
	struct mei_device *dev = dev_get_drvdata(device);
	struct mei_me_hw *hw;
	int ret;

	mutex_lock(&dev->device_lock);

	if (mei_write_is_idle(dev)) {
		hw = to_me_hw(dev);
		hw->pg_state = MEI_PG_ON;

		if (mei_me_hw_use_polling(hw))
			hw->is_active = false;

		mei_forcewake_put(dev);
		ret = 0;
	} else {
		ret = -EAGAIN;
	}

	mutex_unlock(&dev->device_lock);

	return ret;
}

static int __maybe_unused mei_gsc_pm_runtime_resume(struct device *device)
{
	struct mei_device *dev = dev_get_drvdata(device);
	struct mei_me_hw *hw;
	irqreturn_t irq_ret;

	mutex_lock(&dev->device_lock);

	hw = to_me_hw(dev);
	hw->pg_state = MEI_PG_OFF;

	if (mei_me_hw_use_polling(hw)) {
		hw->is_active = true;
		wake_up(&hw->wait_active);
	}

	mei_forcewake_get(dev);

	mutex_unlock(&dev->device_lock);

	irq_ret = mei_me_irq_thread_handler(1, dev);
	if (irq_ret != IRQ_HANDLED)
		dev_err(dev->dev, "thread handler fail %d\n", irq_ret);

	return 0;
}

static const struct dev_pm_ops mei_gsc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mei_gsc_pm_suspend,
				mei_gsc_pm_resume)
	SET_RUNTIME_PM_OPS(mei_gsc_pm_runtime_suspend,
			   mei_gsc_pm_runtime_resume,
			   mei_gsc_pm_runtime_idle)
};

static const struct auxiliary_device_id mei_gsc_id_table[] = {
	{
		.name = "i915.mei-gsc",
		.driver_data = MEI_ME_GSC_CFG,

	},
	{
		.name = "i915.mei-gscfi",
		.driver_data = MEI_ME_GSCFI_CFG,
	},
	{
		.name = "xe.mei-gscfi",
		.driver_data = MEI_ME_GSCFI_CFG,
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(auxiliary, mei_gsc_id_table);

static struct auxiliary_driver mei_gsc_driver = {
	.probe	= mei_gsc_probe,
	.remove = mei_gsc_remove,
	.driver = {
		/* auxiliary_driver_register() sets .name to be the modname */
		.pm = &mei_gsc_pm_ops,
	},
	.id_table = mei_gsc_id_table
};
module_auxiliary_driver(mei_gsc_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_ALIAS("auxiliary:i915.mei-gsc");
MODULE_ALIAS("auxiliary:i915.mei-gscfi");
MODULE_ALIAS("auxiliary:xe.mei-gscfi");
MODULE_DESCRIPTION("Intel(R) Graphics System Controller");
MODULE_LICENSE("GPL");
