/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_PCI_HEADER_
#define MXLK_PCI_HEADER_

#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/xlink_drv_inf.h>
#include "../common/mxlk.h"
#include "../common/mxlk_boot.h"
#include "../common/mxlk_util.h"

#define MXLK_MAX_NAME_LEN (32)

struct mxlk_pcie {
	struct list_head list;
	struct mutex lock;

	struct pci_dev *pci;
	char name[MXLK_MAX_NAME_LEN];
	u32 devid;
	char fw_name[MXLK_MAX_NAME_LEN];

	struct delayed_work wait_event;
	struct delayed_work shutdown_event;
	wait_queue_head_t waitqueue;
	bool irq_enabled;
	irq_handler_t core_irq_callback;

	char partition_name[MXLK_BOOT_DEST_STRLEN];
	unsigned long partition_offset;
	void *dma_buf;
	size_t dma_buf_offset;

	struct mxlk mxlk;
};

static inline struct device *mxlk_to_dev(struct mxlk *mxlk)
{
	struct mxlk_pcie *xdev = container_of(mxlk, struct mxlk_pcie, mxlk);

	return &xdev->pci->dev;
}

int mxlk_pci_init(struct mxlk_pcie *xdev, struct pci_dev *pdev);
int mxlk_pci_cleanup(struct mxlk_pcie *xdev);
int mxlk_pci_register_irq(struct mxlk_pcie *xdev, irq_handler_t irq_handler);
int mxlk_pci_raise_irq(struct mxlk_pcie *xdev, enum mxlk_doorbell_type type,
		       u8 value);

struct mxlk_pcie *mxlk_create_device(u32 sw_device_id, struct pci_dev *pdev);
void mxlk_remove_device(struct mxlk_pcie *xdev);
void mxlk_list_add_device(struct mxlk_pcie *xdev);
void mxlk_list_del_device(struct mxlk_pcie *xdev);
u32 mxlk_get_device_num(u32 *id_list);
struct mxlk_pcie *mxlk_get_device_by_id(u32 id);
int mxlk_get_device_name_by_id(u32 id, char *device_name, size_t name_size);
int mxlk_get_device_status_by_id(u32 id, u32 *status);

int mxlk_pci_boot_device(u32 id, const char *binary_name);
int mxlk_pci_connect_device(u32 id);
int mxlk_pci_read(u32 id, void *data, size_t *size, u32 timeout);
int mxlk_pci_write(u32 id, void *data, size_t *size, u32 timeout);
int mxlk_pci_reset_device(u32 id);

u64 mxlk_pci_hw_dev_id(struct mxlk_pcie *xdev);

#endif
