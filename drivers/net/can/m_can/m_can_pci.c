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

/* Message RAM Configuration (in bytes) */
#define SIDF_ELEMENT_SIZE	4
#define XIDF_ELEMENT_SIZE	8
#define RXF0_ELEMENT_SIZE	72
#define RXF1_ELEMENT_SIZE	72
#define RXB_ELEMENT_SIZE	72
#define TXE_ELEMENT_SIZE	8
#define TXB_ELEMENT_SIZE	72

/* Rx FIFO 0/1 Configuration (RXF0C/RXF1C) */
#define RXFC_FWM_SHIFT	24
#define RXFC_FWM_MASK	(0x7f << RXFC_FWM_SHIFT)
#define RXFC_FS_SHIFT	16
#define RXFC_FS_MASK	(0x7f << RXFC_FS_SHIFT)

/* Rx FIFO 0/1 Status (RXF0S/RXF1S) */
#define RXFS_RFL	BIT(25)
#define RXFS_FF		BIT(24)
#define RXFS_FPI_SHIFT	16
#define RXFS_FPI_MASK	0x3f0000
#define RXFS_FGI_SHIFT	8
#define RXFS_FGI_MASK	0x3f00
#define RXFS_FFL_MASK	0x7f

/* Tx Buffer Configuration(TXBC) */
#define TXBC_NDTB_SHIFT		16
#define TXBC_NDTB_MASK		(0x3f << TXBC_NDTB_SHIFT)
#define TXBC_TFQS_SHIFT		24
#define TXBC_TFQS_MASK		(0x3f << TXBC_TFQS_SHIFT)

/* Tx FIFO/Queue Status (TXFQS) */
#define TXFQS_TFQF		BIT(21)
#define TXFQS_TFQPI_SHIFT	16
#define TXFQS_TFQPI_MASK	(0x1f << TXFQS_TFQPI_SHIFT)
#define TXFQS_TFGI_SHIFT	8
#define TXFQS_TFGI_MASK		(0x1f << TXFQS_TFGI_SHIFT)
#define TXFQS_TFFL_SHIFT	0
#define TXFQS_TFFL_MASK		(0x3f << TXFQS_TFFL_SHIFT)

/* Tx Buffer Element Size Configuration(TXESC) */
#define TXESC_TBDS_8BYTES	0x0
#define TXESC_TBDS_64BYTES	0x7

struct m_can_pci_priv {
	void __iomem *base;
	void __iomem *mram_base;
};

static void m_can_init_mram_conf(struct m_can_classdev *class)
{
	class->mcfg[MRAM_SIDF].off = 0;
	class->mcfg[MRAM_SIDF].num = 128;
	class->mcfg[MRAM_XIDF].off = class->mcfg[MRAM_SIDF].off +
		class->mcfg[MRAM_SIDF].num * SIDF_ELEMENT_SIZE;
	class->mcfg[MRAM_XIDF].num = 64;
	class->mcfg[MRAM_RXF0].off = class->mcfg[MRAM_XIDF].off +
		class->mcfg[MRAM_XIDF].num * XIDF_ELEMENT_SIZE;
	class->mcfg[MRAM_RXF0].num = 64 &
		(RXFC_FS_MASK >> RXFC_FS_SHIFT);
	class->mcfg[MRAM_RXF1].off = class->mcfg[MRAM_RXF0].off +
		class->mcfg[MRAM_RXF0].num * RXF0_ELEMENT_SIZE;
	class->mcfg[MRAM_RXF1].num = 0 &
		(RXFC_FS_MASK >> RXFC_FS_SHIFT);
	class->mcfg[MRAM_RXB].off = class->mcfg[MRAM_RXF1].off +
		class->mcfg[MRAM_RXF1].num * RXF1_ELEMENT_SIZE;
	class->mcfg[MRAM_RXB].num = 64;
	class->mcfg[MRAM_TXE].off = class->mcfg[MRAM_RXB].off +
		class->mcfg[MRAM_RXB].num * RXB_ELEMENT_SIZE;
	class->mcfg[MRAM_TXE].num = 0;
	class->mcfg[MRAM_TXB].off = class->mcfg[MRAM_TXE].off +
		class->mcfg[MRAM_TXE].num * TXE_ELEMENT_SIZE;
	class->mcfg[MRAM_TXB].num = 16 &
		(TXBC_NDTB_MASK >> TXBC_NDTB_SHIFT);

	m_can_init_ram(class);
}

static u32 m_can_pci_read_reg(struct m_can_classdev *cdev, int reg)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	return readl(priv->base + reg);
}

static u32 m_can_pci_read_fifo(struct m_can_classdev *cdev, int offset)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	return readl(priv->mram_base + offset);
}

static int m_can_pci_write_reg(struct m_can_classdev *cdev, int reg, int val)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	writel(val, priv->base + reg);

	return 0;
}

static int m_can_pci_write_fifo(struct m_can_classdev *cdev, int offset, int val)
{
	struct m_can_pci_priv *priv = cdev->device_data;

	writel(val, priv->mram_base + offset);

	return 0;
}

static struct m_can_ops m_can_pci_ops = {
	.read_reg = m_can_pci_read_reg,
	.write_reg = m_can_pci_write_reg,
	.write_fifo = m_can_pci_write_fifo,
	.read_fifo = m_can_pci_read_fifo,
};

static int m_can_pci_probe(struct pci_dev *pci,
		const struct pci_device_id *id)
{
	struct m_can_classdev *class;
	struct m_can_pci_priv *priv;
	struct net_device *net;
	struct device *dev;
	void __iomem *base;
	int ret;

	dev = &pci->dev;
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	class = m_can_class_allocate_dev(dev);
	if (!class)
		return -ENOMEM;

	ret = pcim_enable_device(pci);
	if (ret)
		return ret;

	pci_set_master(pci);

	ret = pcim_iomap_regions(pci, BIT(M_CAN_PCI_MMIO_BAR), pci_name(pci));
	if (ret)
		return ret;

	class->device_data = priv;
	base = pcim_iomap_table(pci)[M_CAN_PCI_MMIO_BAR];

	if (!base) {
		dev_err(dev, "failed to map BARs\n");
		return -ENOMEM;
	}

	priv->base = base;
	priv->mram_base = base + M_CAN_MRAM_OFFSET;

	ret = pci_alloc_irq_vectors(pci, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	priv = netdev_priv(net);
	net->irq = pci_irq_vector(pci, 0);
	class->can.clock.freq = id->driver_data;
	class->ops = &m_can_pci_ops;
	class->is_peripheral = false;

	pci_set_drvdata(pci, net);
	m_can_init_mram_conf(class);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_allow(dev);

	ret = m_can_class_register(class);
	if (ret)
		goto err;

	return 0;

err:
	pm_runtime_disable(&pci->dev);
	pci_free_irq_vectors(pci);
	return ret;
}

static void m_can_pci_remove(struct pci_dev *pci)
{
	struct net_device *dev = pci_get_drvdata(pci);
	struct m_can_classdev *class = netdev_priv(dev);

	pm_runtime_forbid(&pci->dev);
	pm_runtime_get_noresume(&pci->dev);

	pci_free_irq_vectors(pci);
	m_can_class_unregister(class);
}

static __maybe_unused int m_can_pci_suspend(struct device *dev)
{
	return m_can_class_suspend(dev);
}

static __maybe_unused int m_can_pci_resume(struct device *dev)
{
	return m_can_class_resume(dev);
}

static const struct dev_pm_ops m_can_pci_pm_ops = {
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
