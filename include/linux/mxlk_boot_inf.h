/* SPDX-License-Identifier: GPL-2.0 only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2019 Intel Corporation
 *
 ****************************************************************************/

typedef int (*mxlk_pcie_boot_event)(uint32_t phys_dev_id);


int mxlk_pcie_connect_boot_device(const char *dev_name,
				  uint32_t *phys_dev_id,
				  mxlk_pcie_boot_event notif_fn);
int mxlk_pcie_boot_mmio_write(uint32_t phys_dev_id, uint32_t offset,
			      void *data, size_t size);
int mxlk_pcie_boot_mmio_read(uint32_t phys_dev_id, uint32_t offset,
			     void *status, size_t size);
int mxlk_pcie_disconnect_boot_device(uint32_t phys_dev_id);
void *xlink_pcie_alloc_dma_memory(u32 phys_dev_id,
				  size_t size, dma_addr_t *phys_addr);
int xlink_pcie_free_dma_memory(u32 phys_dev_id, size_t size,
			       void *dma_buf, dma_addr_t phys_addr);
