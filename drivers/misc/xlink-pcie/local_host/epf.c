// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/of.h>
#include <linux/platform_device.h>

#include "epf.h"

#define BAR2_MIN_SIZE			SZ_16K
#define BAR4_MIN_SIZE			SZ_16K

#define PCIE_REGS_PCIE_INTR_ENABLE	0x18
#define PCIE_REGS_PCIE_INTR_FLAGS	0x1C
#define LBC_CII_EVENT_FLAG		BIT(18)
#define PCIE_REGS_PCIE_ERR_INTR_FLAGS	0x24
#define LINK_REQ_RST_FLG		BIT(15)

static struct pci_epf_header xpcie_header = {
	.vendorid = PCI_VENDOR_ID_INTEL,
	.deviceid = PCI_DEVICE_ID_INTEL_KEEMBAY,
	.baseclass_code = PCI_BASE_CLASS_MULTIMEDIA,
	.subclass_code = 0x0,
	.subsys_vendor_id = 0x0,
	.subsys_id = 0x0,
};

static const struct pci_epf_device_id xpcie_epf_ids[] = {
	{
		.name = "mxlk_pcie_epf",
	},
	{},
};

static irqreturn_t intel_xpcie_err_interrupt(int irq, void *args)
{
	struct xpcie_epf *xpcie_epf;
	struct xpcie *xpcie = args;
	u32 val;

	xpcie_epf = container_of(xpcie, struct xpcie_epf, xpcie);
	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);
	if (val & LINK_REQ_RST_FLG)
		intel_xpcie_ep_dma_reset(xpcie_epf->epf);

	iowrite32(val, xpcie_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);

	return IRQ_HANDLED;
}

static irqreturn_t intel_xpcie_host_interrupt(int irq, void *args)
{
	struct xpcie_epf *xpcie_epf;
	struct xpcie *xpcie = args;
	u32 val;

	xpcie_epf = container_of(xpcie, struct xpcie_epf, xpcie);
	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	if (val & LBC_CII_EVENT_FLAG) {
		iowrite32(LBC_CII_EVENT_FLAG,
			  xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	}

	return IRQ_HANDLED;
}

static int intel_xpcie_check_bar(struct pci_epf *epf,
				 struct pci_epf_bar *epf_bar,
				 enum pci_barno barno,
				 size_t size, u8 reserved_bar)
{
	if (reserved_bar & (1 << barno)) {
		dev_err(&epf->dev, "BAR%d is already reserved\n", barno);
		return -EFAULT;
	}

	if (epf_bar->size != 0 && epf_bar->size < size) {
		dev_err(&epf->dev, "BAR%d fixed size is not enough\n", barno);
		return -ENOMEM;
	}

	return 0;
}

static int intel_xpcie_configure_bar(struct pci_epf *epf,
				     const struct pci_epc_features
					*epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int ret, i;

	for (i = BAR_0; i <= BAR_5; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			epf_bar->size = epc_features->bar_fixed_size[i];

		if (i == BAR_2) {
			ret = intel_xpcie_check_bar(epf, epf_bar, BAR_2,
						    BAR2_MIN_SIZE,
						    epc_features->reserved_bar);
			if (ret)
				return ret;
		}

		if (i == BAR_4) {
			ret = intel_xpcie_check_bar(epf, epf_bar, BAR_4,
						    BAR4_MIN_SIZE,
						    epc_features->reserved_bar);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void intel_xpcie_cleanup_bar(struct pci_epf *epf, enum pci_barno barno)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	if (xpcie_epf->vaddr[barno]) {
		pci_epc_clear_bar(epc, epf->func_no, &epf->bar[barno]);
		pci_epf_free_space(epf, xpcie_epf->vaddr[barno], barno);
		xpcie_epf->vaddr[barno] = NULL;
	}
}

static void intel_xpcie_cleanup_bars(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	intel_xpcie_cleanup_bar(epf, BAR_2);
	intel_xpcie_cleanup_bar(epf, BAR_4);
	xpcie_epf->xpcie.mmio = NULL;
	xpcie_epf->xpcie.bar4 = NULL;
}

static int intel_xpcie_setup_bar(struct pci_epf *epf, enum pci_barno barno,
				 size_t min_size, size_t align)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epf_bar *bar = &epf->bar[barno];
	struct pci_epc *epc = epf->epc;
	void *vaddr;
	int ret;

	bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	if (!bar->size)
		bar->size = min_size;

	if (barno == BAR_4)
		bar->flags |= PCI_BASE_ADDRESS_MEM_PREFETCH;

	vaddr = pci_epf_alloc_space(epf, bar->size, barno, align);
	if (!vaddr) {
		dev_err(&epf->dev, "Failed to map BAR%d\n", barno);
		return -ENOMEM;
	}

	ret = pci_epc_set_bar(epc, epf->func_no, bar);
	if (ret) {
		pci_epf_free_space(epf, vaddr, barno);
		dev_err(&epf->dev, "Failed to set BAR%d\n", barno);
		return ret;
	}

	xpcie_epf->vaddr[barno] = vaddr;

	return 0;
}

static int intel_xpcie_setup_bars(struct pci_epf *epf, size_t align)
{
	int ret;

	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	ret = intel_xpcie_setup_bar(epf, BAR_2, BAR2_MIN_SIZE, align);
	if (ret)
		return ret;

	ret = intel_xpcie_setup_bar(epf, BAR_4, BAR4_MIN_SIZE, align);
	if (ret) {
		intel_xpcie_cleanup_bar(epf, BAR_2);
		return ret;
	}

	xpcie_epf->comm_bar = BAR_2;
	xpcie_epf->xpcie.mmio = (void *)xpcie_epf->vaddr[BAR_2] +
				XPCIE_MMIO_OFFSET;

	xpcie_epf->bar4 = BAR_4;
	xpcie_epf->xpcie.bar4 = xpcie_epf->vaddr[BAR_4];

	return 0;
}

static int intel_xpcie_epf_get_platform_data(struct device *dev,
					     struct xpcie_epf *xpcie_epf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *soc_node, *version_node;
	struct resource *res;
	const char *prop;
	int prop_size;

	xpcie_epf->irq_dma = platform_get_irq_byname(pdev, "intr");
	if (xpcie_epf->irq_dma < 0) {
		dev_err(&xpcie_epf->epf->dev, "failed to get IRQ: %d\n",
			xpcie_epf->irq_dma);
		return -EINVAL;
	}

	xpcie_epf->irq_err = platform_get_irq_byname(pdev, "err_intr");
	if (xpcie_epf->irq_err < 0) {
		dev_err(&xpcie_epf->epf->dev, "failed to get erroe IRQ: %d\n",
			xpcie_epf->irq_err);
		return -EINVAL;
	}

	xpcie_epf->irq = platform_get_irq_byname(pdev, "ev_intr");
	if (xpcie_epf->irq < 0) {
		dev_err(&xpcie_epf->epf->dev, "failed to get event IRQ: %d\n",
			xpcie_epf->irq);
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	xpcie_epf->apb_base =
		devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->apb_base))
		return PTR_ERR(xpcie_epf->apb_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	xpcie_epf->dbi_base =
		devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->dbi_base))
		return PTR_ERR(xpcie_epf->dbi_base);

	memcpy(xpcie_epf->stepping, "B0", 2);
	soc_node = of_get_parent(pdev->dev.of_node);
	if (soc_node) {
		version_node = of_get_child_by_name(soc_node, "version-info");
		if (version_node) {
			prop = of_get_property(version_node, "stepping",
					       &prop_size);
			if (prop && prop_size <= KEEMBAY_XPCIE_STEPPING_MAXLEN)
				memcpy(xpcie_epf->stepping, prop, prop_size);
			of_node_put(version_node);
		}
		of_node_put(soc_node);
	}

	return 0;
}

static int intel_xpcie_epf_bind(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *features;
	struct pci_epc *epc = epf->epc;
	struct device *dev;
	size_t align = SZ_16K;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	dev = epc->dev.parent;
	features = pci_epc_get_features(epc, epf->func_no);
	xpcie_epf->epc_features = features;
	if (features) {
		align = features->align;
		ret = intel_xpcie_configure_bar(epf, features);
		if (ret)
			return ret;
	}

	ret = intel_xpcie_setup_bars(epf, align);
	if (ret) {
		dev_err(&epf->dev, "BAR initialization failed\n");
		return ret;
	}

	ret = intel_xpcie_epf_get_platform_data(dev, xpcie_epf);
	if (ret) {
		dev_err(&epf->dev, "Unable to get platform data\n");
		return -EINVAL;
	}

	if (!strcmp(xpcie_epf->stepping, "A0")) {
		xpcie_epf->xpcie.legacy_a0 = true;
		iowrite32(1, (void __iomem *)xpcie_epf->xpcie.mmio +
			     XPCIE_MMIO_LEGACY_A0);
	} else {
		xpcie_epf->xpcie.legacy_a0 = false;
		iowrite32(0, (void __iomem *)xpcie_epf->xpcie.mmio +
			     XPCIE_MMIO_LEGACY_A0);
	}

	/* Enable interrupt */
	writel(LBC_CII_EVENT_FLAG,
	       xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_ENABLE);
	ret = devm_request_irq(&epf->dev, xpcie_epf->irq,
			       &intel_xpcie_host_interrupt, 0,
			       XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request irq\n");
		goto err_cleanup_bars;
	}

	ret = devm_request_irq(&epf->dev, xpcie_epf->irq_err,
			       &intel_xpcie_err_interrupt, 0,
			       XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request error irq\n");
		goto err_cleanup_bars;
	}

	ret = intel_xpcie_ep_dma_init(epf);
	if (ret) {
		dev_err(&epf->dev, "DMA initialization failed\n");
		goto err_free_err_irq;
	}

	return 0;

err_free_err_irq:
	free_irq(xpcie_epf->irq_err, &xpcie_epf->xpcie);

err_cleanup_bars:
	intel_xpcie_cleanup_bars(epf);

	return ret;
}

static void intel_xpcie_epf_unbind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;

	intel_xpcie_ep_dma_uninit(epf);

	pci_epc_stop(epc);

	intel_xpcie_cleanup_bars(epf);
}

static int intel_xpcie_epf_probe(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct xpcie_epf *xpcie_epf;

	xpcie_epf = devm_kzalloc(dev, sizeof(*xpcie_epf), GFP_KERNEL);
	if (!xpcie_epf)
		return -ENOMEM;

	epf->header = &xpcie_header;
	xpcie_epf->epf = epf;
	epf_set_drvdata(epf, xpcie_epf);

	return 0;
}

static void intel_xpcie_epf_shutdown(struct device *dev)
{
	struct pci_epf *epf = to_pci_epf(dev);
	struct xpcie_epf *xpcie_epf;

	xpcie_epf = epf_get_drvdata(epf);

	/* Notify host in case PCIe hot plug not supported */
	if (xpcie_epf)
		pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
}

static struct pci_epf_ops ops = {
	.bind = intel_xpcie_epf_bind,
	.unbind = intel_xpcie_epf_unbind,
};

static struct pci_epf_driver xpcie_epf_driver = {
	.driver.name = "mxlk_pcie_epf",
	.driver.shutdown = intel_xpcie_epf_shutdown,
	.probe = intel_xpcie_epf_probe,
	.id_table = xpcie_epf_ids,
	.ops = &ops,
	.owner = THIS_MODULE,
};

static int __init intel_xpcie_epf_init(void)
{
	int ret;

	ret = pci_epf_register_driver(&xpcie_epf_driver);
	if (ret) {
		pr_err("Failed to register xlink pcie epf driver: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(intel_xpcie_epf_init);

static void __exit intel_xpcie_epf_exit(void)
{
	pci_epf_unregister_driver(&xpcie_epf_driver);
}
module_exit(intel_xpcie_epf_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION(XPCIE_DRIVER_DESC);
