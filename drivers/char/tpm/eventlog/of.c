// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2012 IBM Corporation
 *
 * Author: Ashley Lai <ashleydlai@gmail.com>
 *         Nayna Jain <nayna@linux.vnet.ibm.com>
 *
 * Maintained by: <tpmdd-devel@lists.sourceforge.net>
 *
 * Read the event log created by the firmware on PPC64
 */

#include <linux/slab.h>
#include <linux/of.h>
#if defined(CONFIG_ARCH_KEEMBAY)
#include <linux/of_address.h>
#endif
#include <linux/tpm_eventlog.h>

#include "../tpm.h"
#include "common.h"

int tpm_read_log_of(struct tpm_chip *chip)
{
	struct device_node *np;
#if !defined(CONFIG_ARCH_KEEMBAY)
	const u32 *sizep;
	const u64 *basep;
#endif
	struct tpm_bios_log *log;
	u32 size;
	u64 base;
#if defined(CONFIG_ARCH_KEEMBAY)
	struct device_node *node;
	struct resource res;
	void *vaddr;
	int rc;
#endif

	log = &chip->log;
	if (chip->dev.parent && chip->dev.parent->of_node)
		np = chip->dev.parent->of_node;
	else
		return -ENODEV;

	if (of_property_read_bool(np, "powered-while-suspended"))
		chip->flags |= TPM_CHIP_FLAG_ALWAYS_POWERED;

#if defined(CONFIG_ARCH_KEEMBAY)
	node = of_parse_phandle(np, "event-log", 0);
	if (!node) {
		dev_err(&chip->dev, "Couldn't find event-log region.\n");
		return -EINVAL;
	}

	rc = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (rc) {
		dev_err(&chip->dev, "Couldn't resolve event-log region.\n");
		return rc;
	}

	base = res.start;
	size = resource_size(&res);

	dev_info(&chip->dev, "tpm2 event-log base:0x%llx size:0x%x\n", base, size);
	vaddr =	memremap(base, size, MEMREMAP_WB);
	if (!vaddr) {
		dev_err(&chip->dev, "Couldn't map event-log memory resource.\n");
		return -EADDRNOTAVAIL;
	}

	log->bios_event_log = kmemdup(vaddr, size, GFP_KERNEL);
	if (!log->bios_event_log)
		return -ENOMEM;

	memunmap(vaddr);
#else
	sizep = of_get_property(np, "linux,sml-size", NULL);
	basep = of_get_property(np, "linux,sml-base", NULL);

	if (sizep == NULL && basep == NULL)
		return -ENODEV;
	if (sizep == NULL || basep == NULL)
		return -EIO;
	/*
	 * For both vtpm/tpm, firmware has log addr and log size in big
	 * endian format. But in case of vtpm, there is a method called
	 * sml-handover which is run during kernel init even before
	 * device tree is setup. This sml-handover function takes care
	 * of endianness and writes to sml-base and sml-size in little
	 * endian format. For this reason, vtpm doesn't need conversion
	 * but physical tpm needs the conversion.
	 */
	if (of_property_match_string(np, "compatible", "IBM,vtpm") < 0) {
		size = be32_to_cpup((__force __be32 *)sizep);
		base = be64_to_cpup((__force __be64 *)basep);
	} else {
		size = *sizep;
		base = *basep;
	}

	if (size == 0) {
		dev_warn(&chip->dev, "%s: Event log area empty\n", __func__);
		return -EIO;
	}

	log->bios_event_log = kmemdup(__va(base), size, GFP_KERNEL);
	if (!log->bios_event_log)
		return -ENOMEM;
#endif

	log->bios_event_log_end = log->bios_event_log + size;

	if (chip->flags & TPM_CHIP_FLAG_TPM2)
		return EFI_TCG2_EVENT_LOG_FORMAT_TCG_2;
	return EFI_TCG2_EVENT_LOG_FORMAT_TCG_1_2;
}
