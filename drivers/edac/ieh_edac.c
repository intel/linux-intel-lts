// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Intel Integrated Error Handler (IEH)
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * IEH centralizes and standardizes how I/O device errors are reported.
 * They are PCIe devices which aggregate and report error events of different
 * severities (correctable, non-fatal uncorrectable, and fatal uncorrectable)
 * from various I/O devices, e.g., PCIe devices, legacy PCI devices.
 *
 * There is a global IEH and optional north/south satellite IEH(s) logically
 * connected to global IEH. The global IEH is the root to process all incoming
 * error messages from satellite IEH(s) and local devices (if some devices
 * are connected directly to the global IEH) and generate interrupts(SMI/NMI/MCE
 * configured by BIOS/platform firmware). The first IEH-supported platform is
 * Tiger Lake-U. This driver reads/prints the error severity and error source
 * (bus/device/function) logged in the IEH(s) and reboots the system on fatal
 * IEH errors.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/nmi.h>
#include <linux/irq_work.h>
#include <linux/edac.h>
#include <linux/processor.h>
#include <linux/reboot.h>
#include <asm/intel-family.h>
#include <asm/cpu_device_id.h>
#include <asm/mce.h>

#include "edac_mc.h"

#define IEH_REVISION	"v1.7"

#define EDAC_MOD_STR	"ieh_edac"
#define IEH_NMI_NAME	"ieh"

#define GET_BITFIELD(v, lo, hi) (((v) & GENMASK_ULL(hi, lo)) >> (lo))

/* Global correctable error status */
#define GCOERRSTS_OFFSET	0x200
/* Global non-fatal error status */
#define GNFERRSTS_OFFSET	0x210
/* Global fatal error status */
#define GFAERRSTS_OFFSET	0x220

/* Global correctable error mask */
#define GCOERRMSK_OFFSET	0x230
#define GCOERRMSK		0xffffffff
/* Global nonfatal error mask */
#define GNFERRMSK_OFFSET	0x234
#define GNFERRMSK		0xffffffff
/* Global fatal error mask */
#define GFAERRMSK_OFFSET	0x238
#define GFAERRMSK		0xffffffff

/* Global system event status */
#define GSYSEVTSTS_OFFSET	0x260

/* Global system event mask */
#define GSYSEVTMSK_OFFSET	0x264
#define GSYSEVTMSK		0x7
#define GSYSEVTMSK_CORR		BIT(0)
#define GSYSEVTMSK_NONFATAL	BIT(1)
#define GSYSEVTMSK_FATAL	BIT(2)

/* Global system event map */
#define GSYSEVTMAP_OFFSET	0x268
#define GSYSEVTMAP_CORR(m)	GET_BITFIELD(m, 0, 1)
#define GSYSEVTMAP_NONFATAL(m)	GET_BITFIELD(m, 2, 3)
#define GSYSEVTMAP_FATAL(m)	GET_BITFIELD(m, 4, 5)
#define GSYSEVTMAP_MCE		0x3f

/* IEH type and version */
#define IEHTYPEVER_OFFSET	0x26c
#define IEHTYPEVER_TYPE(t)	GET_BITFIELD(t, 0, 3)
#define IEHTYPEVER_VER(t)	GET_BITFIELD(t, 4, 7)
#define IEHTYPEVER_BUS(t)	GET_BITFIELD(t, 8, 15)

/* Bitmap field of satellite IEH */
#define BITMAP_OFFSET		0x27c
#define BITMAP(m)		GET_BITFIELD(m, 0, 4)

/* Local uncorrectable error mask */
#define LERRUNCMSK_OFFSET	0x298
#define LERRUNCMSK		0xffffffff
/* Local correctable error mask */
#define LERRCORMSK_OFFSET	0x2c0
#define LERRCORMSK		0xffffffff

/* Device number and function number of the device reporting to IEH */
#define DEVFUN_OFFSET		0x300
#define DEVFUN_FUN(d)		GET_BITFIELD(d, 0, 2)
#define DEVFUN_DEV(d)		GET_BITFIELD(d, 3, 7)

#define ieh_printk(level, fmt, arg...)	\
	edac_printk(level, "ieh", fmt, ##arg)

/*#define PCI_ADDR(sbdf)	(sbdf)->seg, (sbdf)->bus, (sbdf)->dev, (sbdf)->fun*/

/* Error notification methods */
enum evt_map {
	IEH_IGN,
	IEH_SMI,
	IEH_NMI,
	IEH_MCE,
};

enum severity_level {
	IEH_CORR_ERR,
	IEH_NONFATAL_ERR,
	IEH_FATAL_ERR,
};

enum ieh_type {
	/* Global IEH */
	IEH_GLOBAL,
	/* North satellite IEH logically connected to global IEH */
	IEH_NORTH,
	/* South satellite IEH logically connected to north IEH */
	IEH_SOUTH,
	/*
	 * Superset south satellite IEH with physical ERR[2:0] signals output.
	 * It's used as a global IEH (when it present, system has only one IEH).
	 */
	IEH_SUPERSET,
};

enum action_on_fatal_err {
	NOP,
	RESTART,
	POWER_OFF,
};

struct pci_sbdf {
	u32 seg : 16;
	u32 bus : 8;
	u32 dev : 5;
	u32 fun : 3;
};

struct ieh_dev {
	struct list_head list;
	struct pci_dev *pdev;
	struct pci_sbdf sbdf;
	enum ieh_type type;
	u8 ver;
	/* Global IEH fields */
	enum evt_map corr_map;
	enum evt_map nonfatal_map;
	enum evt_map fatal_map;
};

static struct ieh_config {
	u16 did;
	enum action_on_fatal_err action;
} *ieh_cfg;

struct decoded_res {
	enum severity_level sev;
	struct pci_sbdf sbdf;
};

static LIST_HEAD(global_ieh_list);
static LIST_HEAD(north_ieh_list);
static LIST_HEAD(south_ieh_list);

/* Tiger Lake-U SoC */
#define IEH_DID_TGL_U		0xa0af

static struct ieh_config tgl_u_cfg = {
	.did	= IEH_DID_TGL_U,
	.action	= RESTART,
};

static const char * const severities[] = {
	[IEH_CORR_ERR]		= "correctable",
	[IEH_NONFATAL_ERR]	= "non-fatal uncorrectable",
	[IEH_FATAL_ERR]		= "fatal uncorrectable",
};

static struct irq_work ieh_irq_work;

static int dev_idx(u32 status, int start)
{
	int i;

	for (i = start; i < 32; i++) {
		if (status & (1 << i))
			return i;
	}

	return -1;
}

static inline bool has_notification_by(enum evt_map map)
{
	struct ieh_dev *ieh;

	list_for_each_entry(ieh, &global_ieh_list, list) {
		if (ieh->corr_map == map || ieh->nonfatal_map == map ||
		    ieh->fatal_map == map)
			return true;
	}

	return false;
}

static void ieh_output_error(struct decoded_res *res)
{
	struct pci_sbdf *p = &res->sbdf;

	ieh_printk(KERN_ERR, "Device %04x:%02x:%02x.%x - %s error\n",
		   p->seg, p->bus, p->dev,
		   p->fun, severities[res->sev]);

	if (res->sev != IEH_FATAL_ERR)
		return;

	switch (ieh_cfg->action) {
	case RESTART:
		ieh_printk(KERN_EMERG, "Restart system on device fatal error!\n");
		kernel_restart(NULL);
		break;

	case POWER_OFF:
		ieh_printk(KERN_EMERG, "Power off system on device fatal error!\n");
		kernel_power_off();
		break;
	default:
		break;
	}

	/* TODO: Further report error information from the error source */
}

static bool is_same_pdev(struct pci_sbdf *p, struct pci_sbdf *q)
{
	return (p->seg == q->seg && p->bus == q->bus &&
		p->dev == q->dev && p->fun == q->fun);
}

static struct ieh_dev *__get_ieh(struct list_head *ieh_list,
				 struct pci_sbdf *sbdf)
{
	struct ieh_dev *ieh;

	list_for_each_entry(ieh, ieh_list, list) {
		if (is_same_pdev(sbdf, &ieh->sbdf))
			return ieh;
	}

	return NULL;
}

static struct ieh_dev *get_global_ieh(struct pci_sbdf *sbdf)
{
	return __get_ieh(&global_ieh_list, sbdf);
}

static inline struct ieh_dev *get_north_sat_ieh(struct pci_sbdf *sbdf)
{
	return __get_ieh(&north_ieh_list, sbdf);
}

static inline struct ieh_dev *get_south_sat_ieh(struct pci_sbdf *sbdf)
{
	return __get_ieh(&south_ieh_list, sbdf);
}

static int read_and_clear(struct pci_dev *pdev, int offset, u32 *val)
{
	if (pci_read_config_dword(pdev, offset, val)) {
		ieh_printk(KERN_ERR, "Failed to read 0x%x\n", offset);
		return -ENODEV;
	}

	/* Write 1s to clear status */
	if (pci_write_config_dword(pdev, offset, *val)) {
		ieh_printk(KERN_ERR, "Failed to write 0x%x\n", offset);
		return -ENODEV;
	}

	return 0;
}

#define UNMASK_ERR_EVENT(ieh, name)						\
	do {									\
		u32 val;							\
		if (pci_read_config_dword(ieh->pdev, name##MSK_OFFSET, &val))	\
			return -ENODEV;						\
		val &= ~name##MSK;						\
		if (pci_write_config_dword(ieh->pdev, name##MSK_OFFSET, val))	\
			return -ENODEV;						\
	} while (0)

static int unmask_all_err_events(void)
{
	struct ieh_dev *ieh;

	list_for_each_entry(ieh, &global_ieh_list, list) {
		UNMASK_ERR_EVENT(ieh, GFAERR);
		UNMASK_ERR_EVENT(ieh, GNFERR);
		UNMASK_ERR_EVENT(ieh, GCOERR);
		UNMASK_ERR_EVENT(ieh, LERRUNC);
		UNMASK_ERR_EVENT(ieh, LERRCOR);
		UNMASK_ERR_EVENT(ieh, GSYSEVT);
	}

	return 0;
}

#define MASK_ERR_EVENT(ieh, name)						\
	do {									\
		u32 val;							\
		if (pci_read_config_dword(ieh->pdev, name##MSK_OFFSET, &val))	\
			return -ENODEV;						\
		val |= name##MSK;						\
		if (pci_write_config_dword(ieh->pdev, name##MSK_OFFSET, val))	\
			return -ENODEV;						\
	} while (0)

static int mask_all_err_events(void)
{
	struct ieh_dev *ieh;

	list_for_each_entry(ieh, &global_ieh_list, list) {
		MASK_ERR_EVENT(ieh, GFAERR);
		MASK_ERR_EVENT(ieh, GNFERR);
		MASK_ERR_EVENT(ieh, GCOERR);
		MASK_ERR_EVENT(ieh, LERRUNC);
		MASK_ERR_EVENT(ieh, LERRCOR);
		MASK_ERR_EVENT(ieh, GSYSEVT);
	}

	return 0;
}

static int ieh_handle_error(struct ieh_dev *d, enum severity_level sev)
{
	struct decoded_res res;
	struct pci_sbdf *sbdf = &res.sbdf;
	struct ieh_dev *ieh;
	int i, start = 0;
	u32 sts, reg;

	switch (sev) {
	case IEH_CORR_ERR:
		if (read_and_clear(d->pdev, GCOERRSTS_OFFSET, &sts))
			return -ENODEV;
		ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x GCOERRSTS: 0x%x\n",
			   (&d->sbdf)->seg, (&d->sbdf)->bus, (&d->sbdf)->dev, (&d->sbdf)->fun, sts);
		break;
	case IEH_NONFATAL_ERR:
		if (read_and_clear(d->pdev, GNFERRSTS_OFFSET, &sts))
			return -ENODEV;
		ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x GNFERRSTS: 0x%x\n",
			   (&d->sbdf)->seg, (&d->sbdf)->bus, (&d->sbdf)->dev, (&d->sbdf)->fun, sts);
		break;
	case IEH_FATAL_ERR:
		if (read_and_clear(d->pdev, GFAERRSTS_OFFSET, &sts))
			return -ENODEV;
		ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x GFAERRSTS: 0x%x\n",
			   (&d->sbdf)->seg, (&d->sbdf)->bus, (&d->sbdf)->dev, (&d->sbdf)->fun, sts);
		break;
	}

	while ((i = dev_idx(sts, start)) != -1) {
		if (pci_read_config_dword(d->pdev, DEVFUN_OFFSET + i * 4, &reg)) {
			ieh_printk(KERN_ERR, "Failed to read DEVFUN %d\n", i);
			return -ENODEV;
		}
		ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x DEVFUN %d: 0x%x\n",
			   (&d->sbdf)->seg, (&d->sbdf)->bus, (&d->sbdf)->dev, (&d->sbdf)->fun, i, reg);

		memset(&res, 0, sizeof(res));
		res.sev = sev;
		sbdf->seg = d->sbdf.seg;
		sbdf->bus = d->sbdf.bus;
		sbdf->dev = DEVFUN_DEV(reg);
		sbdf->fun = DEVFUN_FUN(reg);

		switch (d->type) {
		case IEH_GLOBAL:
			ieh = get_north_sat_ieh(sbdf);
			if (!ieh)
				ieh_output_error(&res);
			else if (ieh->type == IEH_NORTH)
				ieh_handle_error(ieh, sev);
			else
				ieh_printk(KERN_ERR, "Invalid global IEH\n");
			break;
		case IEH_NORTH:
			ieh = get_south_sat_ieh(sbdf);
			if (!ieh)
				ieh_output_error(&res);
			else if (ieh->type == IEH_SOUTH)
				ieh_handle_error(ieh, sev);
			else
				ieh_printk(KERN_ERR, "Invalid north IEH\n");
			break;
		case IEH_SOUTH:
		case IEH_SUPERSET:
			ieh_output_error(&res);
			break;
		}

		start = i + 1;
	}

	return 0;
}

static void __ieh_check_error(struct ieh_dev *ieh)
{
	struct pci_dev *pdev = ieh->pdev;
	u32 sts;

	if (pci_read_config_dword(pdev, GSYSEVTSTS_OFFSET, &sts)) {
		ieh_printk(KERN_ERR, "Failed to read GSYSEVTSTS\n");
		return;
	}

	ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x GSYSEVTSTS: 0x%x\n",
		   (&ieh->sbdf)->seg, (&ieh->sbdf)->bus, (&ieh->sbdf)->dev, (&ieh->sbdf)->fun, sts);

	if ((sts & (1 << IEH_FATAL_ERR)) && ieh->fatal_map == IEH_NMI)
		ieh_handle_error(ieh, IEH_FATAL_ERR);

	if ((sts & (1 << IEH_NONFATAL_ERR)) && ieh->nonfatal_map == IEH_NMI)
		ieh_handle_error(ieh, IEH_NONFATAL_ERR);

	if ((sts & (1 << IEH_CORR_ERR)) && ieh->corr_map == IEH_NMI)
		ieh_handle_error(ieh, IEH_CORR_ERR);
}

static void ieh_check_error(void)
{
	struct ieh_dev *ieh;

	list_for_each_entry(ieh, &global_ieh_list, list) {
		__ieh_check_error(ieh);
	}
}

static void ieh_irq_work_cb(struct irq_work *irq_work)
{
	ieh_check_error();
}

static int ieh_nmi_handler(unsigned int cmd, struct pt_regs *regs)
{
	irq_work_queue(&ieh_irq_work);
	return 0;
}

static int mce_check_error(struct notifier_block *nb, unsigned long val,
			   void *data)
{
	struct mce *mce = (struct mce *)data;
	struct decoded_res res;
	struct pci_sbdf *sbdf = &res.sbdf;
	struct ieh_dev *ieh;
	u64 rid;

	/* TODO: For debug only. Remove them later. */
	ieh_printk(KERN_DEBUG, "MCi_STATUS  0x%llx\n", mce->status);
	ieh_printk(KERN_DEBUG, "MCi_MISC    0x%llx\n", mce->misc);
	ieh_printk(KERN_DEBUG, "MCi_ADDR    0x%llx\n", mce->addr);
	ieh_printk(KERN_DEBUG, "MCGSTATUS   0x%llx\n", mce->mcgstatus);
	ieh_printk(KERN_DEBUG, "MCGSCAP     0x%llx\n", mce->mcgcap);
	ieh_printk(KERN_DEBUG, "IP          0x%llx\n", mce->ip);
	ieh_printk(KERN_DEBUG, "MC bank     0x%x\n", mce->bank);

	if ((mce->status & MCACOD) != MCACOD_IOERR)
		return NOTIFY_DONE;

	if (!(mce->status & MCI_STATUS_MISCV))
		return NOTIFY_DONE;

	memset(&res, 0, sizeof(res));
	rid	  = MCI_MISC_PCIRID(mce->misc);
	sbdf->seg = MCI_MISC_PCISEG(mce->misc);
	sbdf->bus = GET_BITFIELD(rid, 8, 15);
	sbdf->dev = GET_BITFIELD(rid, 3, 7);
	sbdf->fun = GET_BITFIELD(rid, 0, 2);

	if (mce->status & MCI_STATUS_PCC)
		res.sev = IEH_FATAL_ERR;
	else if (mce->status & MCI_STATUS_UC)
		res.sev = IEH_NONFATAL_ERR;
	else
		res.sev = IEH_CORR_ERR;

	ieh = get_global_ieh(sbdf);
	if (ieh)
		goto handle;

	ieh = get_north_sat_ieh(sbdf);
	if (ieh)
		goto handle;

	ieh = get_south_sat_ieh(sbdf);
	if (ieh)
		goto handle;

	goto output;

handle:
	ieh_handle_error(ieh, res.sev);
	mce->kflags |= MCE_HANDLED_EDAC;
	return NOTIFY_DONE;

output:
	ieh_output_error(&res);
	return NOTIFY_DONE;
}

static struct notifier_block ieh_mce_dec = {
	.notifier_call	= mce_check_error,
	.priority	= MCE_PRIO_EDAC,
};

static const struct x86_cpu_id ieh_cpuids[] = {
	X86_MATCH_INTEL_FAM6_MODEL(TIGERLAKE_L,	&tgl_u_cfg),
	{}
};
MODULE_DEVICE_TABLE(x86cpu, ieh_cpuids);

static void __put_ieh(struct ieh_dev *ieh)
{
	if (!ieh)
		return;
	if (ieh->pdev) {
		pci_disable_device(ieh->pdev);
		pci_dev_put(ieh->pdev);
	}
	kfree(ieh);
}

static void __put_iehs(struct list_head *ieh_list)
{
	struct ieh_dev *ieh, *tmp;

	edac_dbg(0, "\n");

	list_for_each_entry_safe(ieh, tmp, ieh_list, list) {
		list_del(&ieh->list);
		__put_ieh(ieh);
	}
}

static void put_all_iehs(void)
{
	__put_iehs(&global_ieh_list);
	__put_iehs(&north_ieh_list);
	__put_iehs(&south_ieh_list);
}

static int __get_all_iehs(u16 did)
{
	struct pci_dev *pdev, *prev = NULL;
	int rc = -ENODEV, n = 0;
	struct pci_sbdf *sbdf;
	struct ieh_dev *ieh;
	u32 reg;

	edac_dbg(0, "\n");

	for (;;) {
		pdev = pci_get_device(PCI_VENDOR_ID_INTEL, did, prev);
		if (!pdev)
			break;

		if (pci_enable_device(pdev)) {
			ieh_printk(KERN_ERR, "Failed to enable %04x:%04x\n",
				   pdev->vendor, pdev->device);
			goto fail;
		}

		ieh = kzalloc(sizeof(*ieh), GFP_KERNEL);
		if (!ieh) {
			rc = -ENOMEM;
			goto fail2;
		}

		if (pci_read_config_dword(pdev, IEHTYPEVER_OFFSET, &reg)) {
			ieh_printk(KERN_ERR, "Failed to read IEHTYPEVER\n");
			return -ENODEV;
		}

		ieh->pdev = pdev;
		ieh->ver  = IEHTYPEVER_VER(reg);
		ieh->type = IEHTYPEVER_TYPE(reg);
		sbdf	  = &ieh->sbdf;
		sbdf->seg = pci_domain_nr(pdev->bus);
		sbdf->bus = IEHTYPEVER_BUS(reg);
		sbdf->dev = PCI_SLOT(pdev->devfn);
		sbdf->fun = PCI_FUNC(pdev->devfn);
		ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x IEHTYPEVER: 0x%x\n",
			   (sbdf)->seg, (sbdf)->bus, (sbdf)->dev, (sbdf)->fun, reg);

		if (sbdf->bus != pdev->bus->number) {
			ieh_printk(KERN_ERR, "Mismatched IEH bus\n");
			rc = -EINVAL;
			goto fail3;
		}

		switch (ieh->type) {
		case IEH_SUPERSET:
		case IEH_GLOBAL:
			/* Set notification to MCE */
			if (pci_read_config_dword(pdev, GSYSEVTMAP_OFFSET, &reg)) {
				ieh_printk(KERN_ERR, "Failed to read old GSYSEVTMAP\n");
				return -ENODEV;
			}

			reg |= GSYSEVTMAP_MCE;
			if (pci_write_config_dword(pdev, GSYSEVTMAP_OFFSET, reg)) {
				ieh_printk(KERN_ERR, "Failed to write GSYSEVTMAP\n");
				return -ENODEV;
			}

			if (pci_read_config_dword(pdev, GSYSEVTMAP_OFFSET, &reg)) {
				ieh_printk(KERN_ERR, "Failed to read new GSYSEVTMAP\n");
				return -ENODEV;
			}
			ieh_printk(KERN_DEBUG, "Read %04x:%02x:%02x.%x GSYSEVTMAP: 0x%x\n",
				   (sbdf)->seg, (sbdf)->bus, (sbdf)->dev, (sbdf)->fun, reg);

			ieh->corr_map	  = GSYSEVTMAP_CORR(reg);
			ieh->nonfatal_map = GSYSEVTMAP_NONFATAL(reg);
			ieh->fatal_map	  = GSYSEVTMAP_FATAL(reg);
			list_add_tail(&ieh->list, &global_ieh_list);
			ieh_printk(KERN_DEBUG, "Global/Superset IEH %04x:%02x:%02x.%x\n",
				   (sbdf)->seg, (sbdf)->bus, (sbdf)->dev, (sbdf)->fun);
			break;
		case IEH_NORTH:
			list_add_tail(&ieh->list, &north_ieh_list);
			ieh_printk(KERN_DEBUG, "North IEH %04x:%02x:%02x.%x\n",
				   (sbdf)->seg, (sbdf)->bus, (sbdf)->dev, (sbdf)->fun);
			break;
		case IEH_SOUTH:
			list_add_tail(&ieh->list, &south_ieh_list);
			ieh_printk(KERN_DEBUG, "South IEH %04x:%02x:%02x.%x\n",
				   (sbdf)->seg, (sbdf)->bus, (sbdf)->dev, (sbdf)->fun);
			break;
		}

		pci_dev_get(pdev);
		prev = pdev;
		n++;
	}

	return n;
fail3:
	kfree(ieh);
fail2:
	pci_disable_device(pdev);
fail:
	pci_dev_put(pdev);
	put_all_iehs();
	return rc;
}

static int get_all_iehs(u16 did)
{
	int rc;

	rc = __get_all_iehs(did);
	if (rc < 0)
		return rc;

	if (rc == 0) {
		ieh_printk(KERN_DEBUG, "No IEHs found\n");
		return -ENODEV;
	}

	if (list_empty(&global_ieh_list)) {
		ieh_printk(KERN_ERR, "No global IEH found\n");
		put_all_iehs();
		return -ENODEV;
	}

	return 0;
}

static int register_err_handler(void)
{
	bool os_visible = false;
	int rc;

	if (has_notification_by(IEH_NMI)) {
		init_irq_work(&ieh_irq_work, ieh_irq_work_cb);
		rc = register_nmi_handler(NMI_SERR, ieh_nmi_handler,
					  0, IEH_NMI_NAME);
		if (rc) {
			ieh_printk(KERN_ERR, "Can't register NMI handler\n");
			return rc;
		}

		os_visible = true;
	}

	if (has_notification_by(IEH_MCE)) {
		mce_register_decode_chain(&ieh_mce_dec);
		os_visible = true;
	}

	if (!os_visible) {
		ieh_printk(KERN_INFO, "No OS-visible IEH events\n");
		return -ENODEV;
	}

	return 0;
}

static void unregister_err_handler(void)
{
	if (has_notification_by(IEH_NMI)) {
		unregister_nmi_handler(NMI_SERR, IEH_NMI_NAME);
		irq_work_sync(&ieh_irq_work);
	}

	if (has_notification_by(IEH_MCE))
		mce_unregister_decode_chain(&ieh_mce_dec);
}

static int __init ieh_init(void)
{
	const struct x86_cpu_id *id;
	struct ieh_dev *ieh;
	int rc;

	edac_dbg(2, "\n");

	id = x86_match_cpu(ieh_cpuids);
	if (!id)
		return -ENODEV;
	ieh_cfg = (struct ieh_config *)id->driver_data;

	rc = get_all_iehs(ieh_cfg->did);
	if (rc)
		return rc;

	rc = register_err_handler();
	if (rc)
		goto fail;

	rc = unmask_all_err_events();
	if (rc)
		goto fail2;

	ieh = list_first_entry(&global_ieh_list, struct ieh_dev, list);
	ieh_printk(KERN_INFO, "hw v%d, drv %s\n", ieh->ver, IEH_REVISION);

	return 0;
fail2:
	unregister_err_handler();
fail:
	put_all_iehs();
	return rc;
}

static void __exit ieh_exit(void)
{
	edac_dbg(2, "\n");
	mask_all_err_events();
	unregister_err_handler();
	put_all_iehs();
}

module_init(ieh_init);
module_exit(ieh_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qiuxu Zhuo");
MODULE_DESCRIPTION("IEH Driver for Intel CPU using I/O IEH");
