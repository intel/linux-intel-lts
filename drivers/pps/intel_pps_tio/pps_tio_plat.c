// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel PPS TIO Platform Driver
 *
 * Copyright (C) 2026 Intel Corporation
 */

#include <linux/acpi.h>
#include <linux/auxiliary_bus.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "pps_tio_plat.h"

static DEFINE_IDA(tio_ida);

/*
 * Intel TIO Registers are 64 bit values in two 32 bit registers
 * Software needs to account for roll over possibility
 */
static inline u64 hi_lo_readq_safe(const void __iomem *addr)
{
	const u32 __iomem *p = addr;
	u32 low, high, high2;

	/*
	 * Read HIGH -> LOW -> HIGH
	 * Retry until both high reads match.
	 */
	do {
		high  = readl(p + 1);   /* high */
		low   = readl(p);       /* low */
		high2 = readl(p + 1);   /* high again */
	} while (high != high2);

	return ((u64)high << 32) | low;
}

static inline void pps_tio_read(struct pps_tio_plat_data *pdata,
				u32 offset, u64 *val)
{
	*val = hi_lo_readq_safe(pdata->sh_res_base + offset);
}

static inline void pps_ctl_write(struct pps_tio_plat_data *pdata,
				 u64 val)
{
	writeq(val, pdata->sh_res_base + pdata->tio_data->regs.ctl);
}

/*
 * For COMPV register, It's safer to write
 * higher 32-bit followed by lower 32-bit
 */
static inline void pps_compv_write(struct pps_tio_plat_data *pdata,
				   u64 value)
{
	hi_lo_writeq(value, pdata->sh_res_base + pdata->tio_data->regs.compv);
}

static void pps_tio_enable(struct pps_tio_plat_data *pdata)
{
	u64 ctrl;

	pps_tio_read(pdata, pdata->tio_data->regs.ctl, &ctrl);
	ctrl |= TIOCTL_EN;
	pps_ctl_write(pdata, ctrl);
	pdata->gen_cli_enabled = true;
}

static void pps_tio_disable(struct pps_tio_plat_data *pdata, u64 *ctrl)
{
	pps_tio_read(pdata, pdata->tio_data->regs.ctl, ctrl);
	pps_compv_write(pdata, 0);

	*ctrl &= ~TIOCTL_EN;
	pps_ctl_write(pdata, *ctrl);
	pdata->gen_cli_enabled = false;
}

static int pps_tio_configure(struct pps_tio_plat_data *pdata, int mode)
{
	u64 ctrl;

	pps_tio_disable(pdata, &ctrl);

	if (mode == TIO_GEN) {
		/*
		 * We enable the device, be sure that the
		 * 'compare' value is invalid
		 */
		pps_compv_write(pdata, 0);

		ctrl &= ~(TIOCTL_DIR | TIOCTL_EP);
		ctrl |= TIOCTL_EP_TOGGLE_EDGE;
	} else if (mode == TIO_CLIENT) {
		ctrl &= ~TIOCTL_EP;
		ctrl |= (TIOCTL_DIR | TIOCTL_EP_RISING_EDGE);
	} else {
		return -EINVAL;
	}

	pps_ctl_write(pdata, ctrl);
	pps_tio_enable(pdata);

	return 0;
}

int pps_ops_req(struct pps_tio_plat_data *pdata, struct pps_tio_ops_req *req)
{
	unsigned long flags;
	int ret;

	if (!pdata || !pdata->req || !req)
		return -EINVAL;

	ret = 0; //default
	spin_lock_irqsave(&pdata->res_lock, flags);

	switch (req->type) {
	case READ_REG:
		if (!pdata->req->read || !req->offset)
			goto ret_inv;

		req->value = 0;
		pdata->req->read(pdata, req->offset, &req->value);
		break;

	case WRITE_CTRL:
		if (!pdata->req->set_ctrl)
			goto ret_inv;

		pdata->req->set_ctrl(pdata, req->value);
		break;

	case WRITE_COMPV:
		if (!pdata->req->set_compv)
			goto ret_inv;

		pdata->req->set_compv(pdata, req->value);
		break;

	case ENABLE:
		if (!pdata->req->enable)
			goto ret_inv;

		pdata->req->enable(pdata);
		break;

	case DISABLE:
		if (!pdata->req->disable)
			goto ret_inv;

		req->value = 0;
		pdata->req->disable(pdata, &req->value);
		ret = req->value;
		break;

	case CONFIG:
		if (!pdata->req->config_tio)
			goto ret_inv;

		ret = pdata->req->config_tio(pdata, req->mode);
		break;

	default:
		goto ret_inv;
	}

	spin_unlock_irqrestore(&pdata->res_lock, flags);
	return ret;

ret_inv:
	spin_unlock_irqrestore(&pdata->res_lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_NS_GPL(pps_ops_req, "PPS_TIO");

static const struct  pps_tio_ops tio_ops = {
	.config_tio = pps_tio_configure,
	.enable = pps_tio_enable,
	.disable = pps_tio_disable,
	.read = pps_tio_read,
	.set_ctrl = pps_ctl_write,
	.set_compv = pps_compv_write,
};

static const struct pps_tio_data pmc_data = {
	.regs = {
		.ctl = TIOCTL_PMC,
		.compv = TIOCOMPV_PMC,
		.ec = TIOEC_PMC,
		.tcv = TIOTCV_PMC,
		.eccv = TIOECCV_PMC,
	},
};

static const struct acpi_device_id pps_tio_acpi_match[] = {
	/* Intel SOC's supporting TGPIO
	 * Max 2 or Min 1 pin on an SOC
	 */

	{ "INTC1023", (kernel_ulong_t)&pmc_data},
	{ "INTC1024", (kernel_ulong_t)&pmc_data},
	{}
};
MODULE_DEVICE_TABLE(acpi, pps_tio_acpi_match);

static void pps_tio_aux_release(struct device *dev)
{
	struct auxiliary_device *aux = container_of(dev,
				struct auxiliary_device, dev);

	kfree(aux);
}

static void pps_tio_aux_dest(struct pps_tio *tio,
			     enum tio_mode mode)
{
	struct auxiliary_device *auxdev;

	if (mode == TIO_GEN) {
		auxdev = tio->gen_aux;
		tio->gen_aux = NULL;
	} else {
		auxdev = tio->cl_aux;
		tio->cl_aux = NULL;
	}

	auxiliary_device_delete(auxdev);
	auxiliary_device_uninit(auxdev);
}

static int pps_tio_aux_create(struct pps_tio *pps_tio,
			      struct device *dev,
			      enum tio_mode mode)
{
	struct auxiliary_device *auxdev;
	int ret;

	auxdev = kzalloc(sizeof(*auxdev), GFP_KERNEL);
	if (!auxdev)
		return -ENOMEM;

	if (mode == TIO_GEN) {
		auxdev->name = "generator";
		pps_tio->gen_aux = auxdev;
	} else {
		auxdev->name = "client";
		pps_tio->cl_aux = auxdev;
	}

	auxdev->dev.platform_data = pps_tio->pdata;
	auxdev->dev.release = pps_tio_aux_release;
	auxdev->id = pps_tio->pdata->id;
	auxdev->dev.parent = dev;

	pps_tio->pdata->mode = mode;

	ret = auxiliary_device_init(auxdev);
	if (ret)
		goto err_aux_init;

	ret = auxiliary_device_add(auxdev);
	if (ret)
		goto err_aux_dev_add;

	return 0;

err_aux_dev_add:
	auxiliary_device_uninit(auxdev);
err_aux_init:
	kfree(auxdev);
	return ret;
}

static int pps_tio_switch(struct pps_tio *tio,
			  struct device *dev,
			  enum tio_mode mode)
{
	int ret;

	pps_tio_aux_dest(tio, tio->pdata->mode);

	ret = pps_tio_aux_create(tio, dev, mode);
	if (ret)
		return ret;

	return 0;
}

static int pps_tio_switch_mode(struct pps_tio *pps_tio,
			       struct device *dev, int new_mode)
{
	int ret;

	mutex_lock(&pps_tio->pdata->mode_lock);

	if (pps_tio->pdata->mode == new_mode) {
		ret = 0;
		goto ret;
	}

	if (pps_tio->pdata->gen_cli_enabled) {
		ret = -EBUSY;
		goto ret;
	}

	if (new_mode == TIO_GEN)
		ret = pps_tio_switch(pps_tio, dev, TIO_GEN);
	else
		ret = pps_tio_switch(pps_tio, dev, TIO_CLIENT);

ret:
	mutex_unlock(&pps_tio->pdata->mode_lock);
	return ret;
}

static ssize_t tio_mode_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct pps_tio *pps_tio = dev_get_drvdata(dev);

	if (pps_tio->pdata->mode == TIO_GEN)
		return sysfs_emit(buf, "generator\n");
	else
		return sysfs_emit(buf, "client\n");
}

static ssize_t tio_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct pps_tio *pps_tio = dev_get_drvdata(dev);
	int tio_mode;
	int ret;

	ret = kstrtoint(buf, 0, &tio_mode);
	if (ret)
		return ret;

	if (tio_mode != TIO_GEN && tio_mode != TIO_CLIENT)
		return -EINVAL;

	ret = pps_tio_switch_mode(pps_tio, dev, tio_mode);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(tio_mode);

static struct attribute *pps_tio_attrs[] = {
	&dev_attr_tio_mode.attr,
	NULL,
};

static const struct attribute_group pps_tio_attr_group = {
	.attrs = pps_tio_attrs,
};

static int pps_tio_probe(struct platform_device *pdev)
{
	const struct acpi_device_id *acpi_id;
	struct pps_tio *pps_tio;
	struct pps_tio_plat_data *pdata;
	struct resource *res;
	int ret;

	if (!(cpu_feature_enabled(X86_FEATURE_TSC_KNOWN_FREQ) &&
	      cpu_feature_enabled(X86_FEATURE_ART))) {
		dev_err(&pdev->dev, "TSC/ART is not enabled");
		return -ENODEV;
	}

	acpi_id = acpi_match_device(pps_tio_acpi_match, &pdev->dev);
	if (!acpi_id) {
		dev_err(&pdev->dev, "No node entry in ACPI table\n");
		return -ENODEV;
	}

	pps_tio = devm_kzalloc(&pdev->dev, sizeof(*pps_tio), GFP_KERNEL);
	if (!pps_tio)
		return -ENOMEM;

	platform_set_drvdata(pdev, pps_tio);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->req = &tio_ops;
	pdata->gen_cli_enabled = false;
	pps_tio->pdata = pdata;

	pdata->tio_data = (const struct pps_tio_data *)acpi_id->driver_data;

	spin_lock_init(&pdata->res_lock);
	mutex_init(&pdata->mode_lock);

	/* Map MMIO region (shared for all children) */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory region\n");
		ret = -EINVAL;
		goto err_ret;
	}

	pdata->sh_res_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->sh_res_base))
		return PTR_ERR(pdata->sh_res_base);

	ret = ida_alloc(&tio_ida, GFP_KERNEL);
	if (ret < 0) {
		dev_err(&pdev->dev, "ida out of range %d\n", ret);
		return ret;
	}

	pdata->id = ret;

	ret = pps_tio_aux_create(pps_tio, &pdev->dev, TIO_GEN);
	if (ret) {
		dev_err(&pdev->dev, "aux device creation failed\n");
		goto err_ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &pps_tio_attr_group);
	if (ret)
		return ret;

	return 0;

err_ret:
	ida_free(&tio_ida, pdata->id);
	return ret;
}

static void pps_tio_remove(struct platform_device *pdev)
{
	struct pps_tio *tio = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &pps_tio_attr_group);

	if (tio->gen_aux) {
		auxiliary_device_delete(tio->gen_aux);
		auxiliary_device_uninit(tio->gen_aux);
	}

	if (tio->cl_aux) {
		auxiliary_device_delete(tio->cl_aux);
		auxiliary_device_uninit(tio->cl_aux);
	}

	ida_free(&tio_ida, tio->pdata->id);
}

static struct platform_driver pps_tio_plat = {
	.probe          = pps_tio_probe,
	.remove         = pps_tio_remove,
	.driver         = {
		.name                   = "pps_tio_plat",
		.acpi_match_table       = pps_tio_acpi_match,
	},
};
module_platform_driver(pps_tio_plat);

MODULE_AUTHOR("Christopher Hall <christopher.s.hall@intel.com>");
MODULE_AUTHOR("Subramanian Mohan <subramanian.mohan@intel.com>");
MODULE_DESCRIPTION("Intel PMC Time-Aware IO Platform Driver");
MODULE_LICENSE("GPL");
