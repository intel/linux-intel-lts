/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2018 Intel Corporation */

#ifndef IPU_MMU_H
#define IPU_MMU_H

#include <linux/dma-mapping.h>

#include "ipu.h"
#include "ipu-pdata.h"

#define ISYS_MMID 1
#define PSYS_MMID 0

extern struct ipu_bus_driver ipu_mmu_driver;
/*
 * @pgtbl: virtual address of the l1 page table (one page)
 */
struct ipu_mmu_info {
	u32 __iomem *pgtbl;
	dma_addr_t aperture_start;
	dma_addr_t aperture_end;
	unsigned long pgsize_bitmap;

	spinlock_t lock;	/* Serialize access to users */
	unsigned int users;
	struct ipu_dma_mapping *dmap;
	u32 dummy_l2_tbl;
	u32 dummy_page;

	/* Reference to the trash address to unmap on domain destroy */
	dma_addr_t iova_addr_trash;
};

/*
 * @pgtbl: physical address of the l1 page table
 */
struct ipu_mmu {
	struct list_head node;
	unsigned int users;

	struct ipu_mmu_hw *mmu_hw;
	unsigned int nr_mmus;
	int mmid;

	phys_addr_t pgtbl;
	struct device *dev;

	struct ipu_dma_mapping *dmap;

	struct page *trash_page;
	dma_addr_t iova_addr_trash;

	bool ready;
	spinlock_t ready_lock;	/* Serialize access to bool ready */

	void (*tlb_invalidate)(struct ipu_mmu *mmu);
	void (*set_mapping)(struct ipu_mmu *mmu,
			     struct ipu_dma_mapping *dmap);
};

struct ipu_mmu_info *ipu_mmu_alloc(void);
void ipu_mmu_destroy(struct ipu_mmu_info *mmu_info);
int ipu_mmu_map(struct ipu_mmu_info *mmu_info, unsigned long iova,
	      phys_addr_t paddr, size_t size);
size_t ipu_mmu_unmap(struct ipu_mmu_info *mmu_info, unsigned long iova,
		      size_t size);
phys_addr_t ipu_mmu_iova_to_phys(struct ipu_mmu_info *mmu_info,
				dma_addr_t iova);
#endif
