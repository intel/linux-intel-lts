// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2021 Intel Corporation
 */

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>

#include "epf.h"

#define PCIE_REGS_PCIE_INTR_ENABLE	0x18
#define PCIE_REGS_PCIE_INTR_FLAGS	0x1C
#define LBC_CII_EVENT_FLAG		BIT(18)
#define PCIE_REGS_PCIE_ERR_INTR_FLAGS	0x24
#define LINK_REQ_RST_FLG		BIT(15)

#define PCIE_REGS_PCIE_SYS_CFG_CORE	0x7C
#define PCIE_CFG_PBUS_NUM_OFFSET	8
#define PCIE_CFG_PBUS_NUM_MASK		0xFF
#define PCIE_CFG_PBUS_DEV_NUM_OFFSET	16
#define PCIE_CFG_PBUS_DEV_NUM_MASK	0x1F

#define THB_IRQ_DOORBELL_IDX	2
#define THB_IRQ_WDMA_IDX	10
#define THB_IRQ_RDMA_IDX	18
#define THB_PRIME_RESV_MEM_IDX	8
#define THB_FULL_RESV_MEM_IDX	16

#define THB_DOORBELL_OFF	0x1000
#define THB_DOORBELL_CLR_OFF	0x14
#define THB_DOORBELL_CLR_SZ	0x4

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

u32 xlink_sw_id;

int intel_xpcie_copy_from_host_ll(struct xpcie *xpcie, int chan, int descs_num)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	return intel_xpcie_ep_dma_read_ll(epf, chan, descs_num);
}

int intel_xpcie_copy_to_host_ll(struct xpcie *xpcie, int chan, int descs_num)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	return intel_xpcie_ep_dma_write_ll(epf, chan, descs_num);
}

void intel_xpcie_register_host_irq(struct xpcie *xpcie, irq_handler_t func)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);

	xpcie_epf->core_irq_callback = func;
}

int intel_xpcie_raise_irq(struct xpcie *xpcie, enum xpcie_doorbell_type type)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, type, 1);

	return pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no, PCI_EPC_IRQ_MSI, 1);
}

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
	struct pci_epf *epf;
	u8 event;
	u32 val;

	xpcie_epf = container_of(xpcie, struct xpcie_epf, xpcie);
	epf = xpcie_epf->epf;

	if (epf->header->deviceid == PCI_DEVICE_ID_INTEL_KEEMBAY) {
		val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
		if (!(val & LBC_CII_EVENT_FLAG))
			return IRQ_HANDLED;

		iowrite32(LBC_CII_EVENT_FLAG,
			  xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	}

	if (xpcie_epf->doorbell_clear)
		writel(0x1, xpcie_epf->doorbell_clear);

	event = intel_xpcie_get_doorbell(xpcie, TO_DEVICE, DEV_EVENT);
	if (unlikely(event != NO_OP)) {
		intel_xpcie_set_doorbell(xpcie, TO_DEVICE,
					 DEV_EVENT, NO_OP);
		if (event == REQUEST_RESET)
			orderly_reboot();

		if (event == SWID_UPDATE_EVENT && !xpcie_epf->xpcie.sw_devid)
			xpcie_epf->xpcie.sw_devid =
						intel_xpcie_get_sw_devid(xpcie);

		return IRQ_HANDLED;
	}

	if (likely(xpcie_epf->core_irq_callback))
		xpcie_epf->core_irq_callback(irq, xpcie);

	return IRQ_HANDLED;
}

static void
intel_xpcie_configure_bar(struct pci_epf *epf,
			  const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;

	for (i = BAR_0; i <= BAR_5; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			epf_bar->size = epc_features->bar_fixed_size[i];
	}
}

static void intel_xpcie_cleanup_bar(struct pci_epf *epf, enum pci_barno barno)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	if (xpcie_epf->vaddr[barno]) {
		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, &epf->bar[barno]);
		if (xpcie_epf->vaddr_resv[barno])
			pci_epf_free_space(epf, xpcie_epf->vaddr[barno], barno, PRIMARY_INTERFACE);
		xpcie_epf->vaddr[barno] = NULL;
	}
}

static void intel_xpcie_cleanup_bars(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	intel_xpcie_cleanup_bar(epf, BAR_0);
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
	void *vaddr = NULL;
	int ret;

	if (!min_size)
		return 0;

	bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	if (!bar->size)
		bar->size = min_size;

	if (barno == BAR_0)
		bar->phys_addr = xpcie_epf->doorbell_start;

	if (barno == BAR_2)
		bar->phys_addr = xpcie_epf->mmr2.start;

	if (barno == BAR_4) {
		bar->flags |= PCI_BASE_ADDRESS_MEM_PREFETCH;
		bar->phys_addr = xpcie_epf->mmr4.start;
	}

	if (!bar->phys_addr) {
		vaddr = pci_epf_alloc_space(epf, bar->size, barno, align,
					    PRIMARY_INTERFACE);
		if (!vaddr) {
			dev_err(&epf->dev, "Failed to map BAR%d\n", barno);
			return -ENOMEM;
		}
	} else {
		vaddr = (void __force *)devm_ioremap(&epf->dev, bar->phys_addr,
							bar->size);
		if (IS_ERR(vaddr))
			return PTR_ERR(vaddr);
		xpcie_epf->vaddr_resv[barno] = true;
	}

	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, bar);
	if (ret) {
		pci_epf_free_space(epf, vaddr, barno, PRIMARY_INTERFACE);
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

	ret = intel_xpcie_setup_bar(epf, BAR_0, xpcie_epf->bar0_sz, align);
	if (ret)
		return ret;

	ret = intel_xpcie_setup_bar(epf, BAR_2, xpcie_epf->bar2_sz, align);
	if (ret) {
		intel_xpcie_cleanup_bar(epf, BAR_0);
		return ret;
	}

	ret = intel_xpcie_setup_bar(epf, BAR_4, xpcie_epf->bar4_sz, align);
	if (ret) {
		intel_xpcie_cleanup_bar(epf, BAR_2);
		intel_xpcie_cleanup_bar(epf, BAR_0);
		return ret;
	}

	xpcie_epf->comm_bar = BAR_2;
	xpcie_epf->xpcie.mmio = (void *)xpcie_epf->vaddr[BAR_2] +
				XPCIE_MMIO_OFFSET;

	xpcie_epf->bar4 = BAR_4;
	xpcie_epf->xpcie.bar4 = xpcie_epf->vaddr[BAR_4];

	return 0;
}

static int intel_xpcie_epf_get_thb_pf_data(struct platform_device *pdev,
					   struct xpcie_epf *xpcie_epf)
{
	struct pci_epf *epf = xpcie_epf->epf;
	struct pci_epc *epc = epf->epc;
	struct device_node *np;
	struct device *dma_dev;
	int resv_mem_idx, ret;
	resource_size_t start;
	struct resource *res;

	dma_dev = epc->dev.parent;

	res = platform_get_resource_byname(pdev,
					   IORESOURCE_MEM,
					   "doorbell");
	if (IS_ERR(res))
		return PTR_ERR(res);
	start = res->start + (epf->func_no * THB_DOORBELL_OFF);
	xpcie_epf->doorbell_start = start;

	res = platform_get_resource_byname(pdev,
					   IORESOURCE_MEM,
					   "doorbellclr");
	if (IS_ERR(res))
		return PTR_ERR(res);

	start = res->start + (epf->func_no * THB_DOORBELL_CLR_OFF);
	xpcie_epf->doorbell_clear = devm_ioremap(&pdev->dev, start,
						 THB_DOORBELL_CLR_SZ);
	if (IS_ERR(xpcie_epf->doorbell_clear))
		return PTR_ERR(xpcie_epf->doorbell_clear);

	xpcie_epf->irq_doorbell = irq_of_parse_and_map(pdev->dev.of_node,
						       epf->func_no +
						       THB_IRQ_DOORBELL_IDX);
	if (xpcie_epf->irq_doorbell < 0)
		return xpcie_epf->irq_doorbell;
	ret = devm_request_irq(&epf->dev, xpcie_epf->irq_doorbell,
			       &intel_xpcie_host_interrupt, 0,
			       XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request irq\n");
		return ret;
	}

	xpcie_epf->irq_wdma = irq_of_parse_and_map(pdev->dev.of_node,
						   epf->func_no +
						   THB_IRQ_WDMA_IDX);
	if (xpcie_epf->irq_wdma < 0)
		return xpcie_epf->irq_wdma;

	xpcie_epf->irq_rdma = irq_of_parse_and_map(pdev->dev.of_node,
						   epf->func_no +
						   THB_IRQ_RDMA_IDX);
	if (xpcie_epf->irq_rdma < 0)
		return xpcie_epf->irq_rdma;

	np = of_parse_phandle(pdev->dev.of_node,
			      "memory-region", epf->func_no * 2);
	ret = of_address_to_resource(np, 0, &xpcie_epf->mmr2);
	if (ret)
		return ret;

	np = of_parse_phandle(pdev->dev.of_node,
			      "memory-region", (epf->func_no * 2) + 1);
	ret = of_address_to_resource(np, 0, &xpcie_epf->mmr4);
	if (ret)
		return ret;

	if (epf->header->deviceid == PCI_DEVICE_ID_INTEL_THB_PRIME)
		resv_mem_idx = (epf->func_no >> 1) + THB_PRIME_RESV_MEM_IDX;
	else
		resv_mem_idx = (epf->func_no >> 1) + THB_FULL_RESV_MEM_IDX;
	ret = of_reserved_mem_device_init_by_idx(&epf->dev,
						 pdev->dev.of_node,
						 resv_mem_idx);
	if (ret)
		return ret;

	xpcie_epf->dma_dev = &epf->dev;
	epf->dev.dma_mask = dma_dev->dma_mask;
	epf->dev.coherent_dma_mask = dma_dev->coherent_dma_mask;
	ret = of_dma_configure(&epf->dev, pdev->dev.of_node, true);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	return 0;
}

static int intel_xpcie_epf_get_kmb_pf_data(struct platform_device *pdev,
					   struct xpcie_epf *xpcie_epf)
{
	struct device_node *soc_node, *version_node;
	struct pci_epf *epf = xpcie_epf->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dma_dev;
	struct resource *res;
	int prop_size, ret;
	const char *prop;

	dma_dev = epc->dev.parent;

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
		devm_ioremap(dma_dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->apb_base))
		return PTR_ERR(xpcie_epf->apb_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	xpcie_epf->dbi_base =
		devm_ioremap(dma_dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->dbi_base))
		return PTR_ERR(xpcie_epf->dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dma");
	xpcie_epf->dma_base =
		devm_ioremap(dma_dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->dma_base))
		return PTR_ERR(xpcie_epf->dma_base);

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

	if (!strcmp(xpcie_epf->stepping, "A0")) {
		xpcie_epf->xpcie.legacy_a0 = true;
		intel_xpcie_iowrite32(1, xpcie_epf->xpcie.mmio +
					 XPCIE_MMIO_LEGACY_A0);
	} else {
		xpcie_epf->xpcie.legacy_a0 = false;
		intel_xpcie_iowrite32(0, xpcie_epf->xpcie.mmio +
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
		return ret;
	}

	ret = devm_request_irq(&epf->dev, xpcie_epf->irq_err,
			       &intel_xpcie_err_interrupt, 0,
			       XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request error irq\n");
		return ret;
	}

	/* Initialize reserved memory resources */
	xpcie_epf->dma_dev = dma_dev;
	ret = of_reserved_mem_device_init(dma_dev);
	if (ret) {
		dev_err(&epf->dev, "Could not get reserved memory\n");
		return ret;
	}

	return 0;
}

static int intel_xpcie_epf_get_platform_data(struct device *dev,
					     struct xpcie_epf *xpcie_epf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pci_epf *epf = xpcie_epf->epf;
	struct pci_epc *epc = epf->epc;
	int ret;

	ret = of_property_read_u8(pdev->dev.of_node,
				  "max-functions",
				  &epc->max_functions);
	if (epc->max_functions == THB_FULL_MAX_PCIE_FNS)
		epf->header->deviceid = PCI_DEVICE_ID_INTEL_THB_FULL;
	else if (epc->max_functions == THB_PRIME_MAX_PCIE_FNS)
		epf->header->deviceid = PCI_DEVICE_ID_INTEL_THB_PRIME;

	if (epf->header->deviceid == PCI_DEVICE_ID_INTEL_KEEMBAY) {
		xpcie_epf->bar0_sz = 0;
		xpcie_epf->bar2_sz = SZ_16K;
		xpcie_epf->bar4_sz = SZ_16K;
		ret = intel_xpcie_epf_get_kmb_pf_data(pdev, xpcie_epf);
	} else {
		xpcie_epf->bar0_sz = SZ_4K;
		xpcie_epf->bar2_sz = SZ_16K;
		xpcie_epf->bar4_sz = SZ_8K;
		ret = intel_xpcie_epf_get_thb_pf_data(pdev, xpcie_epf);
	}

	return ret;
}

static int intel_xpcie_epf_bind(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *features;
	struct pci_epc *epc = epf->epc;
	size_t align = SZ_16K;
	u32 bus_num, dev_num;
	struct device *dev;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	/* Only even PCIe functions are used for communication */
	if ((epf->func_no & 0x1))
		return 0;

	dev = epc->dev.parent;
	ret = intel_xpcie_epf_get_platform_data(dev, xpcie_epf);
	if (ret) {
		dev_err(&epf->dev, "Unable to get platform data\n");
		return -EINVAL;
	}

	features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	xpcie_epf->epc_features = features;
	if (features) {
		align = features->align;
		intel_xpcie_configure_bar(epf, features);
	}

	ret = intel_xpcie_setup_bars(epf, align);
	if (ret) {
		dev_err(&epf->dev, "BAR initialization failed\n");
		return ret;
	}

	ret = intel_xpcie_ep_dma_init(epf);
	if (ret) {
		dev_err(&epf->dev, "DMA initialization failed\n");
		goto err_cleanup_bars;
	}

	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_READY);

	if (epf->header->deviceid == PCI_DEVICE_ID_INTEL_KEEMBAY) {
		ret = ioread32(xpcie_epf->apb_base +
			       PCIE_REGS_PCIE_SYS_CFG_CORE);
		bus_num = (ret >> PCIE_CFG_PBUS_NUM_OFFSET) &
			  PCIE_CFG_PBUS_NUM_MASK;
		dev_num = (ret >> PCIE_CFG_PBUS_DEV_NUM_OFFSET) &
			  PCIE_CFG_PBUS_DEV_NUM_MASK;

		xpcie_epf->xpcie.sw_devid =
				intel_xpcie_create_sw_id(epf->func_no,
							 epc->max_functions,
							 bus_num << 8 |
							 dev_num);
		xlink_sw_id = xpcie_epf->xpcie.sw_devid;
	}

	ret = intel_xpcie_core_init(&xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "Core component configuration failed\n");
		goto err_uninit_dma;
	}

	intel_xpcie_iowrite32(XPCIE_STATUS_UNINIT,
			      xpcie_epf->xpcie.mmio + XPCIE_MMIO_HOST_STATUS);
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_RUN);
	intel_xpcie_set_doorbell(&xpcie_epf->xpcie, FROM_DEVICE,
				 DEV_EVENT, NO_OP);
	memcpy(xpcie_epf->xpcie.mmio + XPCIE_MMIO_MAGIC_OFF, XPCIE_MAGIC_YOCTO,
	       strlen(XPCIE_MAGIC_YOCTO));

	return 0;

err_uninit_dma:
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_ERROR);
	memcpy(xpcie_epf->xpcie.mmio + XPCIE_MMIO_MAGIC_OFF, XPCIE_MAGIC_YOCTO,
	       strlen(XPCIE_MAGIC_YOCTO));

	intel_xpcie_ep_dma_uninit(epf);

err_cleanup_bars:
	intel_xpcie_cleanup_bars(epf);

	return ret;
}

static void intel_xpcie_epf_unbind(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	intel_xpcie_core_cleanup(&xpcie_epf->xpcie);
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_READY);

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
	if (xpcie_epf && xpcie_epf->xpcie.status == XPCIE_STATUS_RUN) {
		intel_xpcie_set_doorbell(&xpcie_epf->xpcie, FROM_DEVICE,
					 DEV_EVENT, DEV_SHUTDOWN);
		pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no, PCI_EPC_IRQ_MSI, 1);
	}
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
