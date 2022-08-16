/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _INTEL_VSEC_H
#define _INTEL_VSEC_H

#include <linux/auxiliary_bus.h>
#include <linux/bits.h>

/* Intel DVSEC offsets */
#define INTEL_DVSEC_ENTRIES		0xA
#define INTEL_DVSEC_SIZE		0xB
#define INTEL_DVSEC_TABLE		0xC
#define INTEL_DVSEC_TABLE_BAR(x)	((x) & GENMASK(2, 0))
#define INTEL_DVSEC_TABLE_OFFSET(x)	((x) & GENMASK(31, 3))
#define TABLE_OFFSET_SHIFT		3

struct pci_dev;
struct resource;

enum intel_vsec_id {
	VSEC_ID_TELEMETRY	= 2,
	VSEC_ID_WATCHER		= 3,
	VSEC_ID_CRASHLOG	= 4,
};

/**
 * struct intel_vsec_header - Common fields of Intel VSEC and DVSEC registers.
 * @rev:	Revision ID of the VSEC/DVSEC register space
 * @length:	Length of the VSEC/DVSEC register space
 * @id:		ID of the feature
 * @num_entries:Number of instances of the feature
 * @entry_size:	Size of the discovery table for each feature
 * @tbir:	BAR containing the discovery tables
 * @offset:	BAR offset of start of the first discovery table
 */
struct intel_vsec_header {
	u8	rev;
	u16	length;
	u16	id;
	u8	num_entries;
	u16	cap_id;
	u8	cap_ver;
	u32	next_cap_offset;
	u32	venid;
	u8	entry_size;
	u8	tbir;
	u32	offset;
};

enum intel_vsec_quirks {
	/* Watcher feature not supported */
	VSEC_QUIRK_NO_WATCHER	= BIT(0),

	/* Crashlog feature not supported */
	VSEC_QUIRK_NO_CRASHLOG	= BIT(1),

	/* Use shift instead of mask to read discovery table offset */
	VSEC_QUIRK_TABLE_SHIFT	= BIT(2),

	/* DVSEC not present (provided in driver data) */
	VSEC_QUIRK_NO_DVSEC	= BIT(3),

	/* Platforms requiring quirk in the auxiliary driver */
	VSEC_QUIRK_EARLY_HW     = BIT(4),
};

/* Platform specific data */
struct intel_vsec_platform_info {
	struct intel_vsec_header **capabilities;
	unsigned long quirks;
};

struct intel_vsec_device {
	struct auxiliary_device auxdev;
	struct pci_dev *pcidev;
	struct resource *resource;
	struct ida *ida;
	struct intel_vsec_platform_info *info;
	int num_resources;
};

static inline struct intel_vsec_device *dev_to_ivdev(struct device *dev)
{
	return container_of(dev, struct intel_vsec_device, auxdev.dev);
}

static inline struct intel_vsec_device *auxdev_to_ivdev(struct auxiliary_device *auxdev)
{
	return container_of(auxdev, struct intel_vsec_device, auxdev);
}

#if IS_ENABLED(CONFIG_INTEL_VSEC)
void intel_vsec_register(struct pci_dev *pdev,
			 struct intel_vsec_platform_info *info);
#else
static inline void
intel_vsec_register(struct pci_dev *pdev,
		    struct intel_vsec_platform_info *info) {}
#endif

#endif
