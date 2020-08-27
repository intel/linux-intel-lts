// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/compiler.h>
#include <linux/pci_ids.h>
#include <linux/reboot.h>
#include <linux/xlink_drv_inf.h>
#include "../common/mxlk.h"
#include "../common/mxlk_core.h"
#include "../common/mxlk_util.h"
#include "../common/mxlk_boot.h"
#include "mxlk_struct.h"
#include "mxlk_dma.h"

#define BAR2_MIN_SIZE SZ_16K
#define BAR4_MIN_SIZE SZ_16K
#define KMB_EP_OUTBOUND_MAP_MIN_SIZE SZ_16K

#define PCIE_REGS_PCIE_SYS_CFG_CORE 0x7c
#define PCIE_CFG_PBUS_NUM_OFFSET 8
#define PCIE_CFG_PBUS_NUM_MASK 0xFF
#define PCIE_CFG_PBUS_DEV_NUM_OFFSET 16
#define PCIE_CFG_PBUS_DEV_NUM_MASK 0x1F

#define PCIE_REGS_PCIE_INTR_ENABLE 0x18
#define PCIE_REGS_PCIE_INTR_FLAGS 0x1c
#define LBC_CII_EVENT_FLAG BIT(18)
#define PCIE_REGS_MEM_ACCESS_IRQ_VECTOR	0x180
#define PCIE_REGS_PCIE_ERR_INTR_FLAGS 0x24
#define LINK_REQ_RST_FLG BIT(15)

static struct pci_epf_header mxlk_pcie_header = {
	.vendorid = PCI_VENDOR_ID_INTEL,
	.deviceid = PCI_DEVICE_ID_INTEL_KEEMBAY,
	.baseclass_code = PCI_BASE_CLASS_MULTIMEDIA,
	.subclass_code = 0x0,
	.subsys_vendor_id = 0x0,
	.subsys_id = 0x0,
};

static const struct pci_epf_device_id mxlk_pcie_epf_ids[] = {
	{
		.name = "mxlk_pcie_epf",
	},
	{},
};

u32 xlink_sw_id;

static irqreturn_t mxlk_err_interrupt(int irq, void *args)
{
	struct mxlk *mxlk = args;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	u32 val;

	val = ioread32(mxlk_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);
	if (val & LINK_REQ_RST_FLG)
		mxlk_ep_dma_reset(mxlk_epf->epf);

	iowrite32(val, mxlk_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);

	return IRQ_HANDLED;
}

static irqreturn_t mxlk_host_interrupt(int irq, void *args)
{
	struct mxlk *mxlk = args;
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	u32 val;
	u8 event;

	val = ioread32(mxlk_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	if (val & LBC_CII_EVENT_FLAG) {
		iowrite32(LBC_CII_EVENT_FLAG,
			  mxlk_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);

		event = mxlk_get_doorbell(mxlk, TO_DEVICE, DEV_EVENT);
		if (unlikely(event != NO_OP)) {
			mxlk_set_doorbell(mxlk, TO_DEVICE, DEV_EVENT, NO_OP);
			if (event == REQUEST_RESET)
				orderly_reboot();
			return IRQ_HANDLED;
		}

		if (likely(mxlk_epf->core_irq_callback))
			mxlk_epf->core_irq_callback(irq, mxlk);
	}

	return IRQ_HANDLED;
}

void mxlk_register_host_irq(struct mxlk *mxlk, irq_handler_t func)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);

	mxlk_epf->core_irq_callback = func;
}

int mxlk_raise_irq(struct mxlk *mxlk, enum mxlk_doorbell_type type)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct pci_epf *epf = mxlk_epf->epf;

	mxlk_set_doorbell(mxlk, FROM_DEVICE, type, 1);

	return pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
}

static void __iomem *mxlk_epc_alloc_addr(struct pci_epc *epc,
					 phys_addr_t *phys_addr, size_t size)
{
	void __iomem *virt_addr;
	unsigned long flags;

	spin_lock_irqsave(&epc->lock, flags);
	virt_addr = pci_epc_mem_alloc_addr(epc, phys_addr, size);
	spin_unlock_irqrestore(&epc->lock, flags);

	return virt_addr;
}

static void mxlk_epc_free_addr(struct pci_epc *epc, phys_addr_t phys_addr,
			       void __iomem *virt_addr, size_t size)
{
	unsigned long flags;

	spin_lock_irqsave(&epc->lock, flags);
	pci_epc_mem_free_addr(epc, phys_addr, virt_addr, size);
	spin_unlock_irqrestore(&epc->lock, flags);
}

int mxlk_copy_from_host_ll(struct mxlk *mxlk, int chan, int descs_num)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct pci_epf *epf = mxlk_epf->epf;

	return mxlk_ep_dma_read_ll(epf, chan, descs_num);
}

int mxlk_copy_to_host_ll(struct mxlk *mxlk, int chan, int descs_num)
{
	struct mxlk_epf *mxlk_epf = container_of(mxlk, struct mxlk_epf, mxlk);
	struct pci_epf *epf = mxlk_epf->epf;

	return mxlk_ep_dma_write_ll(epf, chan, descs_num);
}

static int mxlk_check_bar(struct pci_epf *epf, struct pci_epf_bar *epf_bar,
			  enum pci_barno barno, size_t size, u8 reserved_bar)
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

static int mxlk_configure_bar(struct pci_epf *epf,
			      const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;
	int ret;

	for (i = BAR_0; i <= BAR_5; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			epf_bar->size = epc_features->bar_fixed_size[i];

		if (i == BAR_2) {
			ret = mxlk_check_bar(epf, epf_bar, BAR_2,
					     BAR2_MIN_SIZE,
					     epc_features->reserved_bar);
			if (ret)
				return ret;
		}

		if (i == BAR_4) {
			ret = mxlk_check_bar(epf, epf_bar, BAR_4,
					     BAR4_MIN_SIZE,
					     epc_features->reserved_bar);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void mxlk_cleanup_bar(struct pci_epf *epf, enum pci_barno barno)
{
	struct pci_epc *epc = epf->epc;
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);

	if (mxlk_epf->vaddr[barno]) {
		pci_epc_clear_bar(epc, epf->func_no, &epf->bar[barno]);
		pci_epf_free_space(epf, mxlk_epf->vaddr[barno], barno);
	}

	mxlk_epf->vaddr[barno] = NULL;
}

static void mxlk_cleanup_bars(struct pci_epf *epf)
{
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);

	mxlk_cleanup_bar(epf, BAR_2);
	mxlk_cleanup_bar(epf, BAR_4);
	mxlk_epf->mxlk.io_comm = NULL;
	mxlk_epf->mxlk.mmio = NULL;
	mxlk_epf->mxlk.bar4 = NULL;
}

static int mxlk_setup_bar(struct pci_epf *epf, enum pci_barno barno,
			  size_t min_size, size_t align)
{
	int ret;
	void *vaddr = NULL;
	struct pci_epc *epc = epf->epc;
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);
	struct pci_epf_bar *bar = &epf->bar[barno];

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

	mxlk_epf->vaddr[barno] = vaddr;

	return 0;
}

static int mxlk_setup_bars(struct pci_epf *epf, size_t align)
{
	int ret;

	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);

	ret = mxlk_setup_bar(epf, BAR_2, BAR2_MIN_SIZE, align);
	if (ret)
		return ret;

	ret = mxlk_setup_bar(epf, BAR_4, BAR4_MIN_SIZE, align);
	if (ret) {
		mxlk_cleanup_bar(epf, BAR_2);
		return ret;
	}

	mxlk_epf->comm_bar = BAR_2;
	mxlk_epf->mxlk.io_comm = mxlk_epf->vaddr[BAR_2];
	mxlk_epf->mxlk.mmio = (void *)mxlk_epf->mxlk.io_comm + MXLK_MMIO_OFFSET;

	mxlk_epf->bar4 = BAR_4;
	mxlk_epf->mxlk.bar4 = mxlk_epf->vaddr[BAR_4];

	return 0;
}

static int epf_bind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keembay_pcie *keembay = to_keembay_pcie(pci);
	const struct pci_epc_features *features;
	bool msi_capable = true;
	size_t align = 0;
	int ret;
	u32 bus_num = 0;
	u32 dev_num = 0;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	features = pci_epc_get_features(epc, epf->func_no);
	mxlk_epf->epc_features = features;
	if (features) {
		msi_capable = features->msi_capable;
		align = features->align;
		ret = mxlk_configure_bar(epf, features);
		if (ret)
			return ret;
	}

	ret = mxlk_setup_bars(epf, align);
	if (ret) {
		dev_err(&epf->dev, "BAR initialization failed\n");
		return ret;
	}

	mxlk_epf->irq = keembay->ev_irq;
	mxlk_epf->irq_dma = keembay->irq;
	mxlk_epf->irq_err = keembay->err_irq;
	mxlk_epf->apb_base = keembay->apb_base;
	if (!strcmp(keembay->stepping, "A0")) {
		mxlk_epf->mxlk.legacy_a0 = true;
		mxlk_epf->mxlk.mmio->legacy_a0 = 1;
	} else {
		mxlk_epf->mxlk.legacy_a0 = false;
		mxlk_epf->mxlk.mmio->legacy_a0 = 0;
	}

	ret = mxlk_ep_dma_init(epf);
	if (ret) {
		dev_err(&epf->dev, "DMA initialization failed\n");
		goto bind_error;
	}

	mxlk_set_device_status(&mxlk_epf->mxlk, MXLK_STATUS_READY);

	ret = ioread32(mxlk_epf->apb_base + PCIE_REGS_PCIE_SYS_CFG_CORE);
	bus_num = (ret >> PCIE_CFG_PBUS_NUM_OFFSET) & PCIE_CFG_PBUS_NUM_MASK;
	dev_num = (ret >> PCIE_CFG_PBUS_DEV_NUM_OFFSET) &
			PCIE_CFG_PBUS_DEV_NUM_MASK;

	xlink_sw_id = (XLINK_DEV_INF_PCIE << XLINK_DEV_INF_TYPE_SHIFT) |
		   ((bus_num << 8 | dev_num) << XLINK_DEV_PHYS_ID_SHIFT) |
		   (XLINK_DEV_TYPE_KMB << XLINK_DEV_TYPE_SHIFT) |
		   (XLINK_DEV_SLICE_0 << XLINK_DEV_SLICE_ID_SHIFT) |
		   (XLINK_DEV_FUNC_VPU << XLINK_DEV_FUNC_SHIFT);

	ret = mxlk_core_init(&mxlk_epf->mxlk);
	if (ret) {
		dev_err(&epf->dev, "Core component configuration failed\n");
		goto bind_error;
	}

	/* Enable interrupt */
	writel(LBC_CII_EVENT_FLAG,
	       mxlk_epf->apb_base + PCIE_REGS_PCIE_INTR_ENABLE);
	ret = request_irq(mxlk_epf->irq, &mxlk_host_interrupt,
			  0, MXLK_DRIVER_NAME, &mxlk_epf->mxlk);
	if (ret) {
		dev_err(&epf->dev, "failed to request irq\n");
		goto bind_error;
	}

	ret = request_irq(mxlk_epf->irq_err, &mxlk_err_interrupt, 0,
			  MXLK_DRIVER_NAME, &mxlk_epf->mxlk);
	if (ret) {
		dev_err(&epf->dev, "failed to request error irq\n");
		free_irq(mxlk_epf->irq, &mxlk_epf->mxlk);
		goto bind_error;
	}

	if (!mxlk_ep_dma_enabled(mxlk_epf->epf))
		mxlk_ep_dma_reset(mxlk_epf->epf);

	mxlk_epf->mxlk.mmio->host_status = MXLK_STATUS_UNINIT;
	mxlk_set_device_status(&mxlk_epf->mxlk, MXLK_STATUS_RUN);
	mxlk_set_doorbell(&mxlk_epf->mxlk, FROM_DEVICE, DEV_EVENT, NO_OP);
	strncpy(mxlk_epf->mxlk.io_comm->magic, MXLK_BOOT_MAGIC_YOCTO,
		strlen(MXLK_BOOT_MAGIC_YOCTO));

	return 0;

bind_error:
	mxlk_set_device_status(&mxlk_epf->mxlk, MXLK_STATUS_ERROR);
	strncpy(mxlk_epf->mxlk.io_comm->magic, MXLK_BOOT_MAGIC_YOCTO,
		strlen(MXLK_BOOT_MAGIC_YOCTO));

	return ret;
}

static void epf_unbind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);

	free_irq(mxlk_epf->irq, &mxlk_epf->mxlk);
	free_irq(mxlk_epf->irq_err, &mxlk_epf->mxlk);

	mxlk_core_cleanup(&mxlk_epf->mxlk);
	mxlk_set_device_status(&mxlk_epf->mxlk, MXLK_STATUS_READY);

	mxlk_ep_dma_uninit(epf);

	pci_epc_stop(epc);

	mxlk_cleanup_bars(epf);
}

static void epf_linkup(struct pci_epf *epf)
{
}

static int epf_probe(struct pci_epf *epf)
{
	struct mxlk_epf *mxlk_epf;
	struct device *dev = &epf->dev;

	mxlk_epf = devm_kzalloc(dev, sizeof(*mxlk_epf), GFP_KERNEL);
	if (!mxlk_epf)
		return -ENOMEM;

	epf->header = &mxlk_pcie_header;
	mxlk_epf->epf = epf;

	epf_set_drvdata(epf, mxlk_epf);

	return 0;
}

static void epf_shutdown(struct device *dev)
{
	struct pci_epf *epf = to_pci_epf(dev);
	struct mxlk_epf *mxlk_epf = epf_get_drvdata(epf);

	/*
	 * Notify host in case PCIe hot plug not supported
	 */
	if (mxlk_epf && mxlk_epf->mxlk.status == MXLK_STATUS_RUN) {
		mxlk_set_doorbell(&mxlk_epf->mxlk, FROM_DEVICE, DEV_EVENT,
				  DEV_SHUTDOWN);
		pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
	}
}

static struct pci_epf_ops ops = {
	.bind = epf_bind,
	.unbind = epf_unbind,
	.linkup = epf_linkup,
};

static struct pci_epf_driver mxlk_pcie_epf_driver = {
	.driver.name = "mxlk_pcie_epf",
	.driver.shutdown = epf_shutdown,
	.probe = epf_probe,
	.id_table = mxlk_pcie_epf_ids,
	.ops = &ops,
	.owner = THIS_MODULE,
};

static int __init mxlk_epf_init(void)
{
	int ret = -EBUSY;

	ret = pci_epf_register_driver(&mxlk_pcie_epf_driver);
	if (ret) {
		pr_err("Failed to register xlink pcie epf driver: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(mxlk_epf_init);

static void __exit mxlk_epf_exit(void)
{
	pci_epf_unregister_driver(&mxlk_pcie_epf_driver);
}
module_exit(mxlk_epf_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel");
MODULE_DESCRIPTION(MXLK_DRIVER_DESC);
MODULE_VERSION(MXLK_DRIVER_VERSION);
