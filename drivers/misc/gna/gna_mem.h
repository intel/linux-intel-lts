/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2020 Intel Corporation */

#ifndef __GNA_MEM_H__
#define __GNA_MEM_H__

#include <linux/mmu_notifier.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "gna_hw.h"

union gna_memory_map;

struct gna_file_private;

struct gna_xnn_descriptor {
	u32 labase;
	u16 lacount;
	u16 _rsvd;
};

struct gna_mmu {
	u32 vamaxaddr;
	u8 __res_204[12];
	u32 pagedir_n[GNA_PGDIRN_LEN];
};

struct gna_hw_descriptor {
	u8 __res_0000[256];
	struct gna_xnn_descriptor xnn_config;
	u8 __unused[248];
	struct gna_mmu mmu;
};

struct descriptor_addr {
	struct descriptor *descla;
	dma_addr_t descph;
};

struct gna_mmu_notifier {
	struct gna_file_private *file_priv;
	struct gna_private *gna_priv;
	struct gna_memory_object *mo;
	struct mmu_notifier mn;
	struct mm_struct *mm;
};

struct gna_memory_object {
	u64 memory_id;

	const struct gna_memory_operations *ops;

	struct gna_private *gna_priv;
	struct file *fd;

	void __user *user_ptr;
	u64 user_address;
	u64 memory_size;

	void *vaddr;

	struct page **pages;
	struct sg_table *sgt;
	int sgcount;
	int num_pages;
	int num_pinned;
	struct mutex page_lock;

	struct task_struct *task;

	struct list_head mem_list;

	struct list_head file_mem_list;

#if defined(CONFIG_MMU_NOTIFIER)
	struct gna_mmu_notifier *gna_mn;
	struct mutex mn_lock;
#endif

	struct work_struct work;

	struct wait_queue_head waitq;
};

struct gna_memory_operations {
	/* pins pages */
	int (*get_pages)(struct gna_memory_object *mo, u64 offset, u64 size);

	/* puts previously pinned pages */
	void (*put_pages)(struct gna_memory_object *mo);
};

int gna_buffer_get_size(u64 offset, u64 size);

int gna_priv_userptr(
	struct gna_file_private *file_priv, union gna_memory_map *gna_mem);

int gna_mmu_alloc(struct gna_private *gna_priv);

void gna_mmu_free(struct gna_private *gna_priv);

void gna_mmu_add(struct gna_private *gna_priv,
	struct gna_memory_object *object);

void gna_mmu_clear(struct gna_private *gna_priv);

void gna_memory_free(struct gna_private *gna_priv, struct gna_memory_object *mo);

#endif // __GNA_MEM_H__
