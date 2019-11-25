// SPDX-License-Identifier: GPL-2.0
/**
 * m_can_pci.c - PCI Specific M_CAN Glue
 *
 * Copyright (C) 2018 Intel Corporation - https://www.intel.com
 * Author: Felipe Balbi <felipe.balbi@linux.intel.com>
 */

#include <linux/bitops.h>
#include <linux/can/dev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>

#include "m_can.h"

#define PCI_DEVICE_ID_INTEL_EHL_1	 0x4bc1
#define PCI_DEVICE_ID_INTEL_EHL_2	 0x4bc2

#define M_CAN_PCI_MMIO_BAR		0
#define M_CAN_MRAM_OFFSET		0x800

#define M_CAN_CLOCK_FREQ_EHL		100000000
#define CTL_CSR_INT_CTL_OFFSET		0x508

struct m_can_pci_priv {
	void __iomem *base;
	void __iomem *mram_base;
	struct m_can_classdev *mcan_dev;
};

static u32 iomap_read_reg(struct m_can_classdev *cdev, int reg)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	return readl(priv->base + reg);
}

static u32 iomap_read_fifo(struct m_can_classdev *cdev, int offset)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	return readl(priv->base + offset);
}

static int iomap_write_reg(struct m_can_classdev *cdev, int reg, int val)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	writel(val, priv->base + reg);

	return 0;
}

static int iomap_write_fifo(struct m_can_classdev *cdev, int offset, int val)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	writel(val, priv->base + offset);

	return 0;
}

static struct m_can_ops m_can_pci_ops = {
	.read_reg = iomap_read_reg,
	.write_reg = iomap_write_reg,
	.write_fifo = iomap_write_fifo,
	.read_fifo = iomap_read_fifo,
};

static int m_can_pci_probe(struct pci_dev *pci,
		const struct pci_device_id *id)
{
	struct m_can_classdev *mcan_class;
	struct m_can_pci_priv *priv;
	struct device *dev;
	void __iomem *base;
	int ret;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pcim_iomap_regions(pci, BIT(M_CAN_PCI_MMIO_BAR), pci_name(pci));
	if (ret)
		return ret;

	dev = &pci->dev;

	base = pcim_iomap_table(pci)[M_CAN_PCI_MMIO_BAR];

	if (!base) {
		dev_err(dev, "failed to map BARs\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&pci->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mcan_class = m_can_class_allocate_dev(&pci->dev);
	if (!mcan_class)
		return -ENOMEM;

	/*
	 * "bosch,mram-cfg" has been parsed and updated the mcfg[] in m_can_class_allocate_dev
	 * Due to dby_can_wrapper design in EHL, we need to accommodate
	 * for the mram base from PCI MMIO BAR base + 0x800
	 */
	mcan_class->mcfg[MRAM_TXB].off += M_CAN_MRAM_OFFSET;
	mcan_class->mcfg[MRAM_TXE].off += M_CAN_MRAM_OFFSET;
	mcan_class->mcfg[MRAM_RXF0].off += M_CAN_MRAM_OFFSET;
	mcan_class->mcfg[MRAM_RXF1].off += M_CAN_MRAM_OFFSET;

	priv->base = base;
	priv->mram_base = base + M_CAN_MRAM_OFFSET;
	priv->mcan_dev = mcan_class;

	ret = pci_alloc_irq_vectors(pci, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	mcan_class->device_data = priv;
	mcan_class->dev = &pci->dev;
	mcan_class->net->irq = pci_irq_vector(pci, 0);
	mcan_class->pm_clock_support = 1;
	mcan_class->can.clock.freq = id->driver_data;
	mcan_class->ops = &m_can_pci_ops;

	pci_set_drvdata(pci, mcan_class->net);

	ret = m_can_class_register(mcan_class);
	if (ret)
		goto err;

	/* Enable interrupt control at CAN wrapper IP */
	writel(0x1, base + CTL_CSR_INT_CTL_OFFSET);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_allow(dev);

	return 0;

err:
	pm_runtime_disable(&pci->dev);
	pci_free_irq_vectors(pci);
	return ret;
}

static void m_can_pci_remove(struct pci_dev *pci)
{
	struct net_device *dev = pci_get_drvdata(pci);
	struct m_can_classdev *mcan_class = netdev_priv(dev);
	struct m_can_pci_priv *priv = mcan_class->device_data;

	pm_runtime_forbid(&pci->dev);
	pm_runtime_get_noresume(&pci->dev);

	/* Disable interrupt control at CAN wrapper IP */
	writel(0x0, priv->base + CTL_CSR_INT_CTL_OFFSET);

	pci_free_irq_vectors(pci);
	m_can_class_unregister(mcan_class);
}

static __maybe_unused int m_can_pci_suspend(struct device *dev)
{
	return m_can_class_suspend(dev);
}

static __maybe_unused int m_can_pci_resume(struct device *dev)
{
	return m_can_class_resume(dev);
}

static int __maybe_unused m_can_pci_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused m_can_pci_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops m_can_pci_pm_ops = {
	SET_RUNTIME_PM_OPS(m_can_pci_runtime_suspend,
			   m_can_pci_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(m_can_pci_suspend, m_can_pci_resume)
};

static const struct pci_device_id m_can_pci_id_table[] = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_EHL_1),
	  M_CAN_CLOCK_FREQ_EHL, },

	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_EHL_2),
	  M_CAN_CLOCK_FREQ_EHL, },
	{  }	/* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, m_can_pci_id_table);

static struct pci_driver m_can_pci_driver = {
	.name		= "m_can_pci",
	.probe		= m_can_pci_probe,
	.remove		= m_can_pci_remove,
	.id_table	= m_can_pci_id_table,
	.driver = {
		.pm = &m_can_pci_pm_ops,
	}
};

MODULE_AUTHOR("Felipe Balbi <felipe.balbi@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Bosch M_CAN controller on PCI bus");

module_pci_driver(m_can_pci_driver);
