// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Multi-Functional driver for Elkhart Lake -
 * Programmable Service Engine (PSE) GPIO & TGPIO Functions
 *
 * Copyright(c) 2022 Intel Corporation.
 *
 * Intel Elkhart Lake PSE has 2 PCI devices which exposes 2 different
 * functions of GPIO and Timed-GPIO based on the BIOS configurations
 * and exposes the programmability through different offset from the
 * MMIO BAR of the PCI device.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>

#define PSE_GPIO_OFFSET		0x0000
#define PSE_TGPIO_OFFSET	0x1000
#define PSE_PRIV_OFFSET		0x2000
#define PSE_GPIO_SIZE		0x134
#define PSE_TGPIO_SIZE		0x6B0
#define PSE_PRIV_SIZE		0x8

#define PSE_GPIO_D0I3C		1000
#define PSE_GPIO_CGSR		1004
#define PSE_GPIO_D0I3_CIP	BIT(0)
#define PSE_GPIO_D0I3_IR	BIT(1)
#define PSE_GPIO_D0I3_EN	BIT(2)
#define PSE_GPIO_D0I3_RR	BIT(3)
#define PSE_GPIO_CGSR_CG	BIT(16)

static struct resource intel_ehl_pse_gpio_resources[] = {
	DEFINE_RES_MEM(PSE_GPIO_OFFSET, PSE_GPIO_SIZE),
	DEFINE_RES_IRQ(0),
};

static struct resource intel_ehl_pse_tgpio_resources[] = {
	DEFINE_RES_MEM(PSE_TGPIO_OFFSET, PSE_TGPIO_SIZE),
	DEFINE_RES_IRQ(1),
};

struct intel_ehl_pse_gpio_data {
	struct resource *mem;
	int irq;
	struct device *dev;
	void __iomem *priv;
};

struct intel_ehl_pse_gpio_priv {
	const struct intel_ehl_pse_gpio_data *info; // array for gpio & tgpio

	struct mfd_cell *cell; // array for gpio & tgpio
};

static struct mfd_cell intel_ehl_pse_gpio_mfd_devs[] = {
	{
		.name = "gpio-ehl-pse",
		.num_resources = ARRAY_SIZE(intel_ehl_pse_gpio_resources),
		.resources = intel_ehl_pse_gpio_resources,
		.ignore_resource_conflicts = true,
	},
	{
		.name = "intel-ehl-tgpio",
		.num_resources = ARRAY_SIZE(intel_ehl_pse_tgpio_resources),
		.resources = intel_ehl_pse_tgpio_resources,
		.ignore_resource_conflicts = true,
	},
};


static int intel_ehl_pse_gpio_mfd_probe(struct pci_dev *pci,
					const struct pci_device_id *id)
{
	struct intel_ehl_pse_gpio_data *pdata;
	int ret;

	pdata = devm_kzalloc(&pci->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pci_alloc_irq_vectors(pci, 2, 2, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	pdata->irq = pci_irq_vector(pci, 0);
	pdata->mem = &pci->resource[0];

	pdata->priv = devm_ioremap(&pci->dev, pdata->mem->start +
			PSE_PRIV_OFFSET, PSE_PRIV_SIZE);
	if (!pdata->priv)
		return -ENOMEM;

	pci_set_drvdata(pci, pdata);

	ret = mfd_add_devices(&pci->dev, PLATFORM_DEVID_AUTO,
			intel_ehl_pse_gpio_mfd_devs,
			ARRAY_SIZE(intel_ehl_pse_gpio_mfd_devs),
			pdata->mem, pdata->irq, NULL);

	pm_runtime_set_autosuspend_delay(&pci->dev, 1000);
	pm_runtime_use_autosuspend(&pci->dev);
	pm_runtime_put_autosuspend(&pci->dev);
	pm_runtime_allow(&pci->dev);

	return 0;
}

static void intel_ehl_pse_gpio_mfd_remove(struct pci_dev *pci)
{
	pm_runtime_forbid(&pci->dev);
	pm_runtime_get_noresume(&pci->dev);
}

#ifdef CONFIG_PM_SLEEP
static int intel_ehl_pse_gpio_mfd_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_ehl_pse_gpio_data	*pdata = pci_get_drvdata(pdev);
	u32 d0i3c_reg, cgsr_reg = 0;

	d0i3c_reg = readl(pdata->priv + PSE_GPIO_D0I3C);
	cgsr_reg = readl(pdata->priv + PSE_GPIO_CGSR);

	/* enable D0i3 BIT(2) & disable interrupt BIT(1)*/
	d0i3c_reg |= PSE_GPIO_D0I3_EN;
	d0i3c_reg &= ~PSE_GPIO_D0I3_IR;

	/* enable clock gating BIT(16)*/
	cgsr_reg |= PSE_GPIO_CGSR_CG;

	writel(d0i3c_reg, pdata->priv + PSE_GPIO_D0I3C);
	writel(cgsr_reg, pdata->priv + PSE_GPIO_CGSR);

	return 0;
}

static int intel_ehl_pse_gpio_mfd_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_ehl_pse_gpio_data	*pdata = pci_get_drvdata(pdev);
	u32 d0i3c_reg, cgsr_reg = 0;
	struct timespec64 *ts;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);

	d0i3c_reg = readl(pdata->priv + PSE_GPIO_D0I3C);
	cgsr_reg = readl(pdata->priv + PSE_GPIO_CGSR);

	/* disable D0i3 BIT(2) & disable interrupt BIT(1)*/
	d0i3c_reg &= ~(PSE_GPIO_D0I3_IR | PSE_GPIO_D0I3_EN);

	/* disable clock gating BIT(16)*/
	cgsr_reg &= ~PSE_GPIO_CGSR_CG;

	writel(d0i3c_reg, pdata->priv + PSE_GPIO_D0I3C);
	writel(cgsr_reg, pdata->priv + PSE_GPIO_CGSR);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int intel_ehl_pse_gpio_mfd_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_ehl_pse_gpio_data	*pdata = pci_get_drvdata(pdev);
	u32 d0i3c_reg, cgsr_reg = 0;

	d0i3c_reg = readl(pdata->priv + PSE_GPIO_D0I3C);
	cgsr_reg = readl(pdata->priv + PSE_GPIO_CGSR);

	/* enable D0i3 BIT(2) & disable interrupt BIT(1)*/
	d0i3c_reg |= PSE_GPIO_D0I3_EN;
	d0i3c_reg &= ~PSE_GPIO_D0I3_IR;

	/* enable clock gating BIT(16)*/
	cgsr_reg |= PSE_GPIO_CGSR_CG;

	writel(d0i3c_reg, pdata->priv + PSE_GPIO_D0I3C);
	writel(cgsr_reg, pdata->priv + PSE_GPIO_CGSR);

	return 0;
}

static int intel_ehl_pse_gpio_mfd_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct intel_ehl_pse_gpio_data	*pdata = pci_get_drvdata(pdev);
	u32 d0i3c_reg, cgsr_reg = 0;
	struct timespec64 *ts;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);

	d0i3c_reg = readl(pdata->priv + PSE_GPIO_D0I3C);
	cgsr_reg = readl(pdata->priv + PSE_GPIO_CGSR);

	/* disable D0i3 BIT(2) & disable interrupt BIT(1)*/
	d0i3c_reg &= ~(PSE_GPIO_D0I3_IR | PSE_GPIO_D0I3_EN);

	/* disable clock gating BIT(16)*/
	cgsr_reg &= ~PSE_GPIO_CGSR_CG;

	writel(d0i3c_reg, pdata->priv + PSE_GPIO_D0I3C);
	writel(cgsr_reg, pdata->priv + PSE_GPIO_CGSR);

	return 0;
}
#endif

static const struct dev_pm_ops intel_ehl_pse_gpio_mfd_pm_ops = {
	SET_RUNTIME_PM_OPS(intel_ehl_pse_gpio_mfd_runtime_suspend,
			   intel_ehl_pse_gpio_mfd_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(intel_ehl_pse_gpio_mfd_suspend,
				intel_ehl_pse_gpio_mfd_resume)
};

static const struct pci_device_id intel_ehl_pse_gpio_mfd_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x4b88), NULL},
	{ PCI_VDEVICE(INTEL, 0x4b89), NULL},
	{ },
};
MODULE_DEVICE_TABLE(pci, intel_ehl_pse_gpio_mfd_ids);

static struct pci_driver intel_ehl_pse_gpio_mfd_driver = {
	.name		= "intel_ehl_pse_gpio_mfd",
	.id_table	= intel_ehl_pse_gpio_mfd_ids,
	.probe		= intel_ehl_pse_gpio_mfd_probe,
	.remove		= intel_ehl_pse_gpio_mfd_remove,
	.driver = {
		.pm = &intel_ehl_pse_gpio_mfd_pm_ops,
	},
};

module_pci_driver(intel_ehl_pse_gpio_mfd_driver);

MODULE_AUTHOR("Raymond Tan <raymond.tan@intel.com>");
MODULE_DESCRIPTION("Intel MFD Driver for Elkhart Lake PSE TGPIO/GPIO");
MODULE_LICENSE("GPL v2");
