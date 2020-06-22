/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2020 Intel Corporation */

#ifndef __GNA_DRV_H__
#define __GNA_DRV_H__

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/wait.h>

#include "gna.h"

#include "gna_hw.h"
#include "gna_mem.h"

#define GNA_DRV_NAME	"gna"
#define GNA_DRV_VER	"1.0.35"

#define MAX_GNA_DEVICES		16

#define GNA_DEV_HWID_CNL	0x5A11
#define GNA_DEV_HWID_GLK	0x3190
#define GNA_DEV_HWID_EHL	0x4511
#define GNA_DEV_HWID_ICL	0x8A11
#define GNA_DEV_HWID_JSL	0x4E11
#define GNA_DEV_HWID_TGL	0x9A11

struct gna_request;

extern struct class *gna_class;

extern const struct dev_pm_ops gna_pm;

extern int recovery_timeout;

struct gna_driver_private {

	/* device major/minor number facitlities */
	DECLARE_BITMAP(dev_map, MAX_GNA_DEVICES);
	dev_t devt;
	int minor;

	/* protects this structure */
	struct mutex lock;
};

struct gna_file_private {
	struct file *fd;
	struct gna_private *gna_priv;

	struct list_head memory_list;
	struct mutex memlist_lock;

	struct list_head flist;
};

struct pci_bar {
	resource_size_t iostart;
	resource_size_t iosize;
	void __iomem *mem_addr;
};

struct gna_drv_info {
	u32 hwid;
	u32 num_pagetables;
	u32 num_page_entries;
	u32 max_layer_count;
	u64 max_hw_mem;

	struct gna_desc_info desc_info;
};

/* hardware descriptor & MMU */
struct gna_mmu_object {
	struct gna_hw_descriptor *hwdesc;

	dma_addr_t hwdesc_dma;

	u32 **pagetables;
	dma_addr_t *pagetables_dma;

	u32 num_pagetables;

	u32 filled_pts;
	u32 filled_pages;
};

struct gna_hw_info {
	u8 in_buf_s;
	u8 ce_num;
	u8 ple_num;
	u8 afe_num;
	u8 has_mmu;
	u8 hw_ver;
};

struct gna_device_operations {
	struct module *owner;

	int (*getparam)
		(struct gna_private *, union gna_parameter *);
	int (*open)
		(struct gna_private *, struct file *);
	void (*free)
		(struct file *);
	int (*score)
		(struct gna_request *);
	int (*userptr)
		(struct gna_file_private *, union gna_memory_map *);
};

struct gna_private {
	struct gna_driver_private *drv_priv;

	// character device info
	char name[8];
	int dev_num;

	/* lock protecting this very structure */
	struct mutex lock;

	/* list of files opened */
	struct list_head file_list;
	struct mutex filelist_lock;

	/* device objects */
	struct pci_dev *pdev;
	struct device *parent; /* pdev->dev */
	struct device dev;
	struct cdev cdev;

	/* hardware status set by interrupt handler */
	u32 hw_status;
	spinlock_t hw_lock;

	/* device related resources */
	struct pci_bar bar0;
	struct gna_drv_info info;
	struct gna_hw_info hw_info;
	unsigned int irq;

	struct gna_mmu_object mmu;
	struct mutex mmu_lock;

	/* device busy indicator */
	bool busy;
	spinlock_t busy_lock;
	struct wait_queue_head busy_waitq;

	/* device functions */
	/* should be called with acquired mutex */
	struct gna_device_operations *ops;

	/* request/free workqueue */
	struct workqueue_struct *request_wq;

	/* bottom half facilities */
	atomic_t isr_count;
	struct tasklet_struct request_tasklet;

	/* interrupt timer */
	struct timer_list isr_timer;

	/* score request related fields */
	atomic_t request_count;

	/* requests */
	struct list_head request_list;
	struct mutex reqlist_lock;

	/* memory objects */
	struct idr memory_idr;
	struct mutex memidr_lock;

};

extern struct gna_driver_private gna_drv_priv;

extern int gna_probe(
	struct pci_dev *pcidev, const struct pci_device_id *pci_id);

extern void gna_remove(struct pci_dev *pci);

#endif /* __GNA_DRV_H__ */
