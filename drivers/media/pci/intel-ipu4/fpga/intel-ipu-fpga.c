/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include "intel-ipu-fpga.h"

static int intel_ipu_get_sim_type(void)
{
	return SIM_FPGA;
}

static unsigned int intel_ipu_fpga_reset_prepare(struct intel_ipu4_device *isp)
{
	struct pci_dev *bridge;
	phys_addr_t phys;

	/* Search FPGA PCI bridge. Reset is done via bridge IO space */
	bridge = pci_get_device(PCI_VENDOR_ID_INTEL,
				INTEL_IPU5_HW_PCI_FPGA_BRIDGE, NULL);
	if (!bridge)
		return 0;

	dev_info(&isp->pdev->dev,
		 "Device 0x%x (rev: 0x%x) is a bridge for ipu\n",
		 bridge->device, bridge->revision);

	/*
	 * FPGA related process - search bridge IO resources and map a page
	 * from there to IPU io resources.
	 */
	phys = pci_resource_start(bridge, 0);

	pci_resource_start(isp->pdev, IPU5_BAR_FOR_BRIDGE) = phys +
		IPU5_FPGA_PCI_BRIDGE_RESET_START;
	pci_resource_end(isp->pdev,  IPU5_BAR_FOR_BRIDGE) = phys +
		IPU5_FPGA_PCI_BRIDGE_RESET_END;
	pci_resource_flags(isp->pdev, IPU5_BAR_FOR_BRIDGE) =
		pci_resource_flags(isp->pdev, 0);

	return 1 << IPU5_BAR_FOR_BRIDGE;
}

static void intel_ipu_fpga_reset(struct pci_dev *pci_dev)
{
	struct intel_ipu4_device *isp = pci_get_drvdata(pci_dev);

	dev_info(&pci_dev->dev, "IP reset for FPGA\n");

	pci_save_state(pci_dev);

	/* reset process for FPGA only */
	ipu_readl(isp->base2 + IPU5_FPGA_RESET_REG);
	ipu_writel(IPU5_FPGA_RESET_ACTIVE, isp->base2 + IPU5_FPGA_RESET_REG);
	usleep_range(1000, 1100);

	ipu_readl(isp->base2 + IPU5_FPGA_RESET_REG);
	ipu_writel(IPU5_FPGA_RESET_RELEASE, isp->base2 + IPU5_FPGA_RESET_REG);
	usleep_range(1000, 1100);

	ipu_readl(isp->base2 + IPU5_FPGA_RESET_REG);
	usleep_range(1000, 1100);

	pci_restore_state(pci_dev);

	dev_info(&pci_dev->dev, "IP reset for FPGA done\n");
}

static void intel_ipu_fpga_pmclite_btr_power(
			struct intel_ipu4_device *isp, bool on)
{
	/* FPGA uses PMC Lite to control IPU power */
	if (!on) {
		ipu_writel(0, isp->base + BUTTRESS_REG_PS_FREQ_CTL);
		usleep_range(1000, 1500);
		ipu_writel(0, isp->base + BUTTRESS_REG_IS_FREQ_CTL);
	} else {
		ipu_writel(IPU5_BTR_IS_ON, isp->base +
			   BUTTRESS_REG_IS_FREQ_CTL);
		usleep_range(1000, 1500);
		ipu_writel(IPU5_BTR_PS_ON, isp->base +
			   BUTTRESS_REG_PS_FREQ_CTL);
	}
	usleep_range(1000, 1500);

	dev_dbg(&isp->pdev->dev,
		"set buttress power to %d sts now is 0x%x\n",
		on, ipu_readl(isp->base + BUTTRESS_REG_PWR_STATE));
}

static void intel_ipu_buttress_disable_secure_touch(
		struct intel_ipu4_device *isp)
{
	u32 val;

	/* No CSE HW on FPGA, no security related features */
	val = ipu_readl(isp->base + BUTTRESS_REG_SECURE_TOUCH);
	val &= ~(1 << BUTTRESS_SECURE_TOUCH_SECURE_TOUCH_SHIFT);
	ipu_writel(val, isp->base + BUTTRESS_REG_SECURE_TOUCH);
}

static int intel_ipu_fpga_buttress_power(
	struct device *dev, struct intel_ipu4_buttress_ctrl *ctrl, bool on)
{
	struct intel_ipu4_device *isp = to_intel_ipu4_bus_device(dev)->isp;
	unsigned long tout_jfs;
	u32 pwr_sts, val;
	int ret = 0;

	if (!ctrl)
		return 0;

	/* Until FLR completion nothing is expected to work */
	if (isp->flr_done)
		return 0;

	mutex_lock(&isp->buttress.power_mutex);

	if (!on) {
		val = 0;
		pwr_sts = ctrl->pwr_sts_off << ctrl->pwr_sts_shift;
	} else {
		val = 1 << BUTTRESS_FREQ_CTL_START_SHIFT
			| ctrl->divisor
			| ctrl->qos_floor << BUTTRESS_FREQ_CTL_QOS_FLOOR_SHIFT;

		pwr_sts = ctrl->pwr_sts_on << ctrl->pwr_sts_shift;
	}

	intel_ipu_fpga_pmclite_btr_power(isp, on);

	tout_jfs = jiffies + msecs_to_jiffies(BUTTRESS_POWER_TIMEOUT);
	do {
		usleep_range(10, 40);
		val = ipu_readl(isp->base + BUTTRESS_REG_PWR_STATE);
		if ((val & ctrl->pwr_sts_mask) == pwr_sts) {
			dev_dbg(&isp->pdev->dev,
				"Rail state successfully changed\n");
			goto out;
		}
	} while (!time_after(jiffies, tout_jfs));

	dev_err(&isp->pdev->dev,
		"Timeout when trying to change state of the rail 0x%x\n", val);

	/*
	 * Return success always as psys power up is not working
	 * currently. This should be -ETIMEDOUT always when psys power
	 * up is fixed
	 * Also in FPGA case don't report time out. Depending on FPGA version
	 * the PM state transition may or may not work.
	 */
out:
	intel_ipu_buttress_disable_secure_touch(isp);
	ctrl->started = !ret && on;

	mutex_unlock(&isp->buttress.power_mutex);

	return ret;
}

/* Simplified PM for IPU5 FPGA until PM works better */
static int intel_ipu_fpga_runtime_suspend(struct device *dev)
{
	struct intel_ipu4_bus_device *adev = to_intel_ipu4_bus_device(dev);
	int rval;

	rval = pm_generic_runtime_suspend(dev);
	if (rval)
		return rval;

	if (!adev->ctrl) {
		dev_dbg(dev, "has no buttress control info, bailing out\n");
		return 0;
	}

	if (dev != adev->isp->isys_iommu) {
		dev_warn(dev, "not isys iommu suspend, just out\n");
		return 0;
	}

	rval = intel_ipu_fpga_buttress_power(dev, adev->ctrl, false);
	dev_dbg(dev, "%s: buttress power down %d\n", __func__, rval);
	if (!rval)
		return 0;

	dev_err(dev, "power down failed!\n");

	/* Powering down failed, attempt to resume device now */
	rval = pm_generic_runtime_resume(dev);
	if (!rval)
		return -EBUSY;

	return -EIO;
}

static int intel_ipu_fpga_runtime_resume(struct device *dev)
{
	struct intel_ipu4_bus_device *adev = to_intel_ipu4_bus_device(dev);
	int rval;

	if (adev->ctrl) {
		rval = intel_ipu_fpga_buttress_power(dev, adev->ctrl, true);
		dev_dbg(dev, "%s: buttress power up %d\n", __func__, rval);
		if (rval)
			return rval;
	}

	rval = pm_generic_runtime_resume(dev);
	dev_dbg(dev, "%s: resume %d\n", __func__, rval);
	if (rval)
		goto out_err;

	return 0;

out_err:
	if (adev->ctrl)
		intel_ipu_fpga_buttress_power(dev, adev->ctrl, false);

	return -EBUSY;
}

static void intel_ipu_fpga_sensor_config(struct intel_ipu4_device *isp)
{
	/*
	* To support sensor on FPGA, some registers defined to control
	* MIPI to connect sensor
	*/
	ipu_writel(0x0, isp->base + IPU5_BUTTRESS_REG_SENSOR_SUPPORT_CONTROL);
	ipu_writel(IPU5_BUTTRESS_SENSOR_SUPPORT_SW_RESET,
		isp->base + IPU5_BUTTRESS_REG_SENSOR_SUPPORT_CONTROL);
	ipu_writel(0x20202020,
		isp->base + IPU5_BUTTRESS_REG_SENSOR_SUPPORT_MIPI_TIMING0);
	ipu_writel(0x20,
		isp->base + IPU5_BUTTRESS_REG_SENSOR_SUPPORT_MIPI_TIMING1);
}

static int intel_ipu_fpga_get_secure_mode(void)
{
	/* skip handling for security related features as no CSE HW */
	return 0;
}

static int intel_ipu_fpga_ipc_reset(struct device *dev)
{
	/* skip handling for security related features as no CSE HW */
	return 0;
}

static int intel_ipu_fpga_start_tsc(void)
{
	/* Skip handling as no Hammock Harbor HW */
	return 0;
}

static int intel_ipu_fpga_get_config(int type)
{
	/* FPGA related configration for TPG mode */
	switch (type) {
	case ISYS_FREQ:
		return INTEL_IPU5_ISYS_FREQ_FPGA;
	case TPG_HBLANK:
		return 16384;
	case TPG_LLP:
		return 16384 + 4096;
	default:
		break;
	}

	return -EINVAL;
}

static bool intel_ipu_fpga_device_suspended(struct device *dev)
{
	return pm_runtime_status_suspended(dev);
}

const struct intel_ipu_sim_ctrl sim_ctrl_ops = {
	.get_sim_type = intel_ipu_get_sim_type,
	.reset_prepare = intel_ipu_fpga_reset_prepare,
	.reset = intel_ipu_fpga_reset,
	.runtime_suspend = intel_ipu_fpga_runtime_suspend,
	.runtime_resume = intel_ipu_fpga_runtime_resume,
	.sensor_config = intel_ipu_fpga_sensor_config,
	.get_secure_mode = intel_ipu_fpga_get_secure_mode,
	.ipc_reset = intel_ipu_fpga_ipc_reset,
	.start_tsc = intel_ipu_fpga_start_tsc,
	.get_config = intel_ipu_fpga_get_config,
	.device_suspended = intel_ipu_fpga_device_suspended,
};
EXPORT_SYMBOL_GPL(sim_ctrl_ops);

static int __init intel_ipu_fpga_init(void)
{
	return 0;
}

static void __exit intel_ipu_fpga_exit(void)
{
}

module_init(intel_ipu_fpga_init);
module_exit(intel_ipu_fpga_exit);

MODULE_AUTHOR("Yunliang Ding <yunliang.ding@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU FPGA library");
