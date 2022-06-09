/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright © 2023 Intel Corporation
 */

#include <linux/vfio_pci_core.h>
#include <linux/sizes.h>

#include <drm/i915_sriov.h>

#define I915_VFIO_MAX_DATA_SIZE SZ_32M
#define I915_VFIO_MAX_TILE 2

/**
 * struct i915_vfio_pci_migration_header - Migration header
 *
 * Header describing each individual iteration of device data.
 */
struct i915_vfio_pci_migration_header {
	/** @type: type of device state */
	u64 type;
	/** @tile: tile from which the device state comes from */
	u64 tile;
	/** @offset: foo */
	u64 offset;
	/** @size: size of device data that follows */
	u64 size;
	/** @flags: optional flags */
	u64 flags;
} __packed;

struct i915_vfio_pci_migration_data {
	struct i915_vfio_pci_migration_header hdr;
	bool hdr_processed;
	struct list_head link;
	void *buf;
	loff_t pos;
};

/**
 * struct i915_vfio_pci_migration_file
 */
struct i915_vfio_pci_migration_file {
	struct file *filp;
	/* Protects save_data / resume_data */
	struct mutex lock;
	struct list_head save_data;
	struct i915_vfio_pci_migration_data resume_data;
	struct i915_vfio_pci_core_device *i915_vdev;
	unsigned long (*copy_from)(void *to, const void __user *from, unsigned long n);
	unsigned long (*copy_to)(void __user *to, const void *from, unsigned long n);
};

struct i915_vfio_pci_mappable_resource {
	void *vaddr;
	ssize_t size;
};

/**
 * struct i915_vfio_pci_core_device - i915-specific vfio_pci_core_device
 *
 * Top level structure of i915_vfio_pci.
 */
struct i915_vfio_pci_core_device {
	/** @core_device: vendor-agnostic VFIO device */
	struct vfio_pci_core_device core_device;

	enum vfio_device_mig_state mig_state;

	/** @vfid: VF number used by PF, i915 uses 1-based indexing for vfid */
	unsigned int vfid;

	/** @pf: pointer to driver_private of physical function */
	struct pci_dev *pf;
	const struct i915_vfio_pci_migration_pf_ops *pf_ops;

	struct i915_vfio_pci_mappable_resource lmem[I915_VFIO_MAX_TILE];

	struct i915_vfio_pci_migration_file *fd;
};

struct i915_vfio_pci_mappable_resource_ops {
	size_t (*size)(struct pci_dev *pf, unsigned int vfid, unsigned int tile);
	void * (*map)(struct pci_dev *pf, unsigned int vfid, unsigned int tile);
	void (*unmap)(struct pci_dev *pf, unsigned int vfid, unsigned int tile);
};

struct i915_vfio_pci_resource_ops {
	size_t (*size)(struct pci_dev *pf, unsigned int vfid, unsigned int tile);
	ssize_t (*save)(struct pci_dev *pf, unsigned int vfid, unsigned int tile,
			void *buf, size_t size);
	int (*load)(struct pci_dev *pf, unsigned int vfid, unsigned int tile,
		    const void *buf, size_t size);
};

struct i915_vfio_pci_migration_pf_ops {
	int (*pause)(struct pci_dev *pf, unsigned int vfid);
	int (*resume)(struct pci_dev *pf, unsigned int vfid);
	int (*wait_flr_done)(struct pci_dev *pf, unsigned int vfid);
	struct i915_vfio_pci_resource_ops ggtt, fw;
	struct i915_vfio_pci_mappable_resource_ops lmem;
};

#define i915_vdev_to_dev(i915_vdev) (&(i915_vdev)->core_device.pdev->dev)
#define i915_vdev_to_pdev(i915_vdev) ((i915_vdev)->core_device.pdev)

void i915_vfio_pci_reset(struct i915_vfio_pci_core_device *i915_vdev);
ssize_t i915_vfio_data_read(struct i915_vfio_pci_migration_file *migf, char __user *buf,
			    size_t len);
ssize_t i915_vfio_data_write(struct i915_vfio_pci_migration_file *migf, const char __user *buf,
			     size_t len);
void i915_vfio_save_data_release(struct i915_vfio_pci_migration_file *migf);

int i915_vfio_pci_produce_save_data(struct i915_vfio_pci_migration_file *migf);
