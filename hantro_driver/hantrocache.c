/*
 *    Hantro cache controller hardware driver.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include "hantrocache.h"

/********variables declaration related with race condition**********/

//struct semaphore enc_core_sem;

/*******************PCIE CONFIG*************************/
#ifdef PCI_BUS
#define PCI_VENDOR_ID_HANTRO		0x10ee
#define PCI_DEVICE_ID_HANTRO_PCI	0x8014

/* Base address got control register */
#define PCI_H2_BAR	4

static struct pci_dev *gDev; /* PCI device structure. */
static unsigned long gBaseHdwr; /* PCI base register address (Hardware address) */
static u32 gBaseLen; /* Base register address Length */
#endif

/*------------------PORTING LAYER------------------------------------*/
/* Cache types */
#define CCLIENT_TYPE_VC8000E	"_VC8000E"
#define CCLIENT_TYPE_VC8000D0	"_VC8000D_0"
#define CCLIENT_TYPE_VC8000D1	"_VC8000D_1"
#define CCLIENT_TYPE_DECG10	"_DECODER_G1_0"
#define CCLIENT_TYPE_DECG11	"_DECODER_G1_1"
#define CCLIENT_TYPE_DECG20	"_DECODER_G2_0"
#define CCLIENT_TYPE_DECG21	"_DECODER_G2_1"
/* Cache directions */
#define CC_DIR_READ	"_DIRRD"
#define CC_DIR_WRITE	"_DIRWR"
#define CC_DIR_BIDIR	"_DIRBI"

#define RESOURCE_SHARED_INTER_CORES	0

/* the base addr of core. for PCIE, it refers to offset from bar */
#define CORE_0_IO_BASE	0x700000
#define CORE_1_IO_BASE	0x700000
#define CORE_2_IO_BASE	0x780000
#define CORE_3_IO_BASE	0x780000

#define CORE_0_IO_SIZE	0x800 /* bytes cache register size */
#define CORE_1_IO_SIZE	0x800 /* bytes shaper register size */

#define CORE_2_IO_SIZE	((6 + 4 * 8) * 4) /* bytes cache register size */
#define CORE_3_IO_SIZE	((5 + 5 * 8) * 4) /* bytes shaper register size */

#define INT_PIN_CORE_0	-1
#define INT_PIN_CORE_1	-1

#define INT_PIN_CORE_2	-1
#define INT_PIN_CORE_3	-1
/* for all cores, the core info should be listed here for later use */
/* base_addr, iosize, irq */
#ifndef USE_DTB_PROBE
static struct cache_core_config cache_core_array[] = {
	/*
	 * note parent address is copied from hx280enc.c's CORE_x_IO_ADDR
	 * and hantrodec.c's SOCLE_LOGIC_x_BASE
	 */
	{ VC8000D_0, 0x18553d000, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 0,
	  0x18553a000 },
	{ VC8000D_1, 0x18553d800, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 0,
	  0x18553b000 },
	{ VC8000D_0, 0x28553d000, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 1,
	  0x28553a000 },
	{ VC8000D_1, 0x28553d800, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 1,
	  0x28553b000 },
	{ VC8000D_0, 0x38553d000, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 2,
	  0x38553a000 },
	{ VC8000D_1, 0x38553d800, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 2,
	  0x38553b000 },
	{ VC8000D_0, 0x48553d000, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 3,
	  0x48553a000 },
	{ VC8000D_1, 0x48553d800, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_WR, 3,
	  0x48553b000 },
};
#endif
static int bcacheprobed;
extern bool disable_dec400;

/*------------------------------END-------------------------------------*/

/***************************TYPE AND FUNCTION DECLARATION****************/

/* here's all the must remember stuff */
static int ReserveIO(struct cache_dev_t *);
static void ReleaseIO(struct cache_dev_t *);
static void ResetAsic(struct cache_dev_t *dev);
#ifdef USE_IRQ
#ifndef PCI_BUS
static irqreturn_t cache_isr(int irq, void *dev_id);
#endif
#endif
/*********************local variable declaration*****************/

/******************************************************************************/
static int CheckCacheIrq(struct cache_dev_t *dev)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_CACHE);
	unsigned long flags;
	int rdy = 0;

	spin_lock_irqsave(&parentslice->cache_owner_lock, flags);
	if (dev->irq_received) {
		/* reset the wait condition(s) */
		dev->irq_received = 0;
		rdy = 1;
	}
	spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

	return rdy;
}

static unsigned int WaitCacheReady(struct cache_dev_t *dev)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_CACHE);

	if (wait_event_interruptible(parentslice->cache_wait_queue,
				     CheckCacheIrq(dev))) {
		PDEBUG("Cache wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	return 0;
}

static int CheckCoreOccupation(struct cache_dev_t *dev, struct file *filp)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_CACHE);
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&parentslice->cache_owner_lock, flags);
	if (!dev->is_reserved) {
		dev->is_reserved = 1;
		dev->cacheowner = filp;
		ret = 1;
	}

	spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

	return ret;
}

static int GetWorkableCore(struct cache_dev_t *dev, struct file *filp)
{
	return CheckCoreOccupation(dev, filp);
}

static long ReserveCore(struct cache_dev_t *dev, struct file *filp)
{
	struct slice_info *parentslice;

	parentslice = getparentslice(dev, CORE_CACHE);

	/* lock a core that has specified core id */
	if (wait_event_interruptible(parentslice->cache_hw_queue,
				     GetWorkableCore(dev, filp) != 0))
		return -ERESTARTSYS;

	return 0;
}

static void ReleaseCore(struct cache_dev_t *dev)
{
	unsigned long flags;
	struct slice_info *parentslice = getparentslice(dev, CORE_CACHE);

	/* release specified core id */
	spin_lock_irqsave(&parentslice->cache_owner_lock, flags);
	if (dev->is_reserved) {
		dev->cacheowner = NULL;
		dev->is_reserved = 0;
	}

	dev->irq_received = 0;
	dev->irq_status = 0;
	spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

	wake_up_interruptible_all(&parentslice->cache_hw_queue);
}

long hantrocache_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	u32 tmp, id, slice, node, type;
	u32 core_id;
	unsigned long long tmp64;
	struct cache_dev_t *pccore;
	if (disable_dec400)
		return -EFAULT;
	switch (cmd) {
	case CACHE_IOCGHWOFFSET:
		__get_user(id, (int *)arg);
		slice = SLICE(id);
		node = KCORE(id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		else
			__put_user(pccore->com_base_addr,
				   (unsigned long long *)arg);
		break;
	case CACHE_IOCGHWIOSIZE:
		id = (u32)arg;
		slice = SLICE(id);
		node = KCORE(id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		else
			return pccore->core_cfg.iosize;
		break;
	case CACHE_IOCG_CORE_NUM:
		id = arg;
		id = get_slicecorenum(id, CORE_CACHE);
		return id;
	case CACHE_IOCH_HW_RESERVE: {
		driver_cache_dir dir;
		cache_client_type client;
		/* it's a little danger here since here's no protection of the chain */
		__get_user(tmp64, (unsigned long long *)arg);
		id = tmp64 >> 32;
		slice = SLICE(id);
		type = NODETYPE(id);
		node = KCORE(id);
		core_id = (u32)tmp64; /* get client and direction info */
		dir = core_id & 0x01;
		client = (core_id & 0x06) >> 1;
		pccore = get_cachenodes(slice, 0);

		while (pccore != NULL) {
			/* a valid core supports such client and dir */
			if (pccore->core_cfg.client == client &&
			    pccore->core_cfg.dir == dir &&
			    pccore->parentid == node && pccore->is_valid &&
			    ((type == NODE_TYPE_DEC &&
			      pccore->parenttype == CORE_DEC) ||
			     (type == NODE_TYPE_ENC &&
			      pccore->parenttype == CORE_ENC)))
				break;
			pccore = pccore->next;
		}
		if (pccore == NULL)
			return -EFAULT;

		ret = ReserveCore(pccore, filp);
		if (ret == 0)
			return pccore->core_id;
		return ret;
	}
	case CACHE_IOCH_HW_RELEASE:
		core_id = (u32)arg;
		slice = SLICE(core_id);
		node = KCORE(core_id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		else
			ReleaseCore(pccore);
		break;
	case CACHE_IOCG_ABORT_WAIT:
		core_id = (u32)arg;
		slice = SLICE(core_id);
		node = KCORE(core_id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		tmp = WaitCacheReady(pccore);
		if (tmp == 0)
			return pccore->irq_status;
		break;
	}
	return 0;
}

int cache_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int cache_release(struct file *filp)
{
	int i, slicen = get_slicenumber();
	struct cache_dev_t *dev;

	for (i = 0; i < slicen; i++) {
		dev = get_cachenodes(i, 0);
		while (dev != NULL) {
			if (dev->cacheowner == filp && dev->is_reserved) {
				ResetAsic(dev);
				ReleaseCore(dev);
			}
			dev = dev->next;
		}
	}
	return 0;
}

#ifdef PCI_BUS
static int PcieInit(int corenum)
{
	int i = 0;

	gDev = pci_get_device(PCI_VENDOR_ID_HANTRO, PCI_DEVICE_ID_HANTRO_PCI,
			      gDev);
	if (gDev == NULL) {
		PDEBUG("Init: Hardware not found.\n");
		return -1;
	}

	if (pci_enable_device(gDev) < 0) {
		PDEBUG("Init: Device not enabled.\n");
		return -1;
	}

	gBaseHdwr = pci_resource_start(gDev, PCI_H2_BAR);
	if (gBaseHdwr < 0) {
		PDEBUG(KERN_INFO "Init: Base Address not set.\n");
		goto out_pci_disable_device;
	}
	gBaseLen = pci_resource_len(gDev, PCI_H2_BAR);

	for (i = 0; i < corenum; i++)
		cache_core_array[i].base_addr =
			gBaseHdwr +
			cache_core_array[i]
				.base_addr; /* the offset is based on which bus interface is chosen */

	return 0;

out_pci_disable_device:
	pci_disable_device(gDev);
	gDev = NULL;
}

static void PcieClose(void)
{
	if (gDev)
		pci_disable_device(gDev);
}
#else
static void PcieClose(void)
{
}
#endif

int __init cache_init(void)
{
	int result = 0;

	bcacheprobed = 0;
#ifdef PCI_BUS
	result = PcieInit(sizeof(cache_core_array) /
			  sizeof(struct cache_core_config));
#endif
	return result;
}
#ifdef USE_DTB_PROBE
static void cache_getcachetype(const char *name, int *client, int *dir)
{
	if (strstr(name, CCLIENT_TYPE_VC8000E) != NULL)
		*client = VC8000E;
	else if (strstr(name, CCLIENT_TYPE_VC8000D0) != NULL)
		*client = VC8000D_0;
	else if (strstr(name, CCLIENT_TYPE_VC8000D1) != NULL)
		*client = VC8000D_1;
	else if (strstr(name, CCLIENT_TYPE_DECG10) != NULL)
		*client = DECODER_G1_0;
	else if (strstr(name, CCLIENT_TYPE_DECG11) != NULL)
		*client = DECODER_G1_1;
	else if (strstr(name, CCLIENT_TYPE_DECG20) != NULL)
		*client = DECODER_G2_0;
	else if (strstr(name, CCLIENT_TYPE_DECG21) != NULL)
		*client = DECODER_G2_1;
	else
		*client = -1;

	if (strstr(name, CC_DIR_READ) != NULL)
		*dir = DIR_RD;
	else if (strstr(name, CC_DIR_WRITE) != NULL)
		*dir = DIR_WR;
	else if (strstr(name, CC_DIR_BIDIR) != NULL)
		*dir = DIR_RD;
	else
		*dir = -1;
}
#endif

int cache_probe(dtbnode *pnode)
{
	int result;
	int i;
	struct cache_dev_t *pccore;

#ifndef PCI_BUS
	int type, dir;

	cache_getcachetype(pnode->ofnode->name, &type, &dir);
	if (type == -1 || dir == -1)
		return -EINVAL;
	pccore = vmalloc(sizeof(struct cache_dev_t));
	if (pccore == NULL)
		return -ENOMEM;
	memset(pccore, 0, sizeof(struct cache_dev_t));
	pccore->com_base_addr = pccore->core_cfg.base_addr = pnode->ioaddr;
	pccore->core_cfg.iosize = pnode->iosize;
	pccore->core_cfg.client = type;
	pccore->core_cfg.dir = dir;

	result = ReserveIO(pccore);
	if (result < 0) {
		pr_err("cachecore: reserve reg 0x%llx-0x%llx fail\n",
		       pnode->ioaddr, pnode->iosize);
		vfree(pccore);
		return -ENODEV;
	}

	ResetAsic(pccore); /* reset hardware */
	pccore->is_valid = 1;
	for (i = 0; i < 4; i++)
		pccore->irqlist[i] = -1;
#ifdef USE_IRQ
	if (pnode->irq[0] > 0) {
		strcpy(pccore->irq_name[0], pnode->irq_name[0]);
		result = request_irq(pnode->irq[0], cache_isr, IRQF_SHARED,
				     pccore->irq_name[0], (void *)pccore);
		if (result == 0)
			pccore->irqlist[0] = pnode->irq[0];
		else {
			pr_err("cachecore: request IRQ <%d> fail\n",
			       pnode->irq[0]);
			ReleaseIO(pccore);
			vfree(pccore);
			return -EINVAL;
		}
	}
#endif
	pccore->core_cfg.parentaddr = pnode->parentaddr;
	add_cachenode(pnode->sliceidx, pccore);

#endif /* not def PCI_BUS */
	return 0;
}

void __exit cache_cleanup(void)
{
#ifndef PCI_BUS
	int i, k, slicen = get_slicenumber();
	struct cache_dev_t *pccore, *pnext;

	for (i = 0; i < slicen; i++) {
		pccore = get_cachenodes(i, 0);
		while (pccore != NULL) {
			pnext = pccore->next;
			writel(0, pccore->hwregs + 0x04); /* disable HW */
			writel(0xF, pccore->hwregs + 0x14); /* clear IRQ */

			/* free the encoder IRQ */
			for (k = 0; k < 4; k++)
				if (pccore->irqlist[k] > 0)
					free_irq(pccore->irqlist[k],
						 (void *)pccore);
			ReleaseIO(pccore);

			vfree(pccore);
			pccore = pnext;
		}
	}
#endif
	bcacheprobed = 0;
	PcieClose();
}

static int cache_get_hwid(unsigned long base_addr, int *hwid)
{
	u8 *hwregs = NULL;

	if (!request_mem_region(base_addr, 4, "hantro_cache")) {
		PDEBUG(KERN_INFO
		       "hantr_cache: failed to reserve HW regs,base_addr:%p\n",
		       (void *)base_addr);
		return -1;
	}
	hwregs = (u8 *)ioremap(base_addr, 4);
	if (hwregs == NULL) {
		PDEBUG(KERN_INFO "hantr_cache: failed to ioremap HW regs\n");
		release_mem_region(base_addr, 4);
		return -1;
	}

	*hwid = readl(hwregs + 0x00);
	PDEBUG(KERN_INFO "hantro_cache: hwid = %x, base_addr= %p\n", (int)*hwid,
	       (void *)base_addr);

	if (hwregs)
		iounmap((void *)hwregs);
	release_mem_region(base_addr, 4);

	return 0;
}

static int ReserveIO(struct cache_dev_t *pccore)
{
	int hwid, hw_cfg;

	if (cache_get_hwid(pccore->core_cfg.base_addr, &hwid) < 0)
		return -1;

	hw_cfg = (hwid & 0xF0000) >> 16;

	if (hw_cfg > 2)
		return -1;

	if (hw_cfg == 1 && pccore->core_cfg.dir == DIR_WR) /* cache only */
		pccore->is_valid = 0;
	else if (hw_cfg == 2 && pccore->core_cfg.dir == DIR_RD) /* shaper only */
		pccore->is_valid = 0;
	else
		pccore->is_valid = 1;

	if (pccore->is_valid == 0)
		return -1;

	if (hwid == 0 && pccore->core_cfg.dir == DIR_RD) {
		pccore->core_cfg.base_addr += CACHE_WITH_SHAPER_OFFSET;
	} else if (hwid != 0) {
		if (pccore->core_cfg.dir == DIR_WR)
			pccore->core_cfg.base_addr += SHAPER_OFFSET;
		else if (pccore->core_cfg.dir == DIR_RD && hw_cfg == 0)
			pccore->core_cfg.base_addr += CACHE_WITH_SHAPER_OFFSET;
		else if (pccore->core_cfg.dir == DIR_RD && hw_cfg == 1)
			pccore->core_cfg.base_addr += CACHE_ONLY_OFFSET;
	}

	if (!request_mem_region(pccore->core_cfg.base_addr,
				pccore->core_cfg.iosize, pccore->reg_name)) {
		PDEBUG(KERN_INFO
		       "hantr_cache: failed to reserve HW regs,core:%x\n",
		       hwid);
		pccore->is_valid = 0;
		return -1;
	}

	pccore->hwregs = (u8 *)ioremap(pccore->core_cfg.base_addr,
				       pccore->core_cfg.iosize);

	if (pccore->hwregs == NULL) {
		PDEBUG(KERN_INFO
		       "hantr_cache: failed to ioremap HW regs,core:%x\n",
		       hwid);
		release_mem_region(pccore->core_cfg.base_addr,
				   pccore->core_cfg.iosize);
		pccore->is_valid = 0;
		return -1;
	}

	if (pccore->core_cfg.dir == DIR_RD)
		PDEBUG("cache  reg[0x10]=%08x\n", readl(pccore->hwregs + 0x10));
	else
		PDEBUG("shaper reg[0x08]=%08x\n", readl(pccore->hwregs + 0x08));

	pr_info("hantrocache: HW at base <0x%llx> with ID 0x%x [mapped addr = 0x%llx]\n",
		pccore->core_cfg.base_addr, hwid,
		(unsigned long long)pccore->hwregs);

	return 0;
}

static void ReleaseIO(struct cache_dev_t *pccore)
{
	if (pccore->is_valid == 0)
		return;
	if (pccore->hwregs)
		iounmap((void *)pccore->hwregs);
	release_mem_region(pccore->core_cfg.base_addr, pccore->core_cfg.iosize);
}

#ifdef USE_IRQ
#ifndef PCI_BUS
static irqreturn_t cache_isr(int irq, void *dev_id)
{
	unsigned int handled = 0;
	struct cache_dev_t *dev = (struct cache_dev_t *)dev_id;
	u32 irq_status;
	unsigned long flags;
	u32 irq_triggered = 0;
	struct slice_info *parentslice;

	parentslice = getparentslice(dev, CORE_CACHE);
	/* If core is not reserved by any user, but irq is received, just ignore it */
	spin_lock_irqsave(&parentslice->cache_owner_lock, flags);
	if (!dev->is_reserved) {
		spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

	if (dev->core_cfg.dir == DIR_RD) {
		irq_status = readl(dev->hwregs + 0x04);
		if (irq_status & 0x28) {
			irq_triggered = 1;
			writel(irq_status, dev->hwregs + 0x04); /* clear irq */
		}
	} else {
		irq_status = readl(dev->hwregs + 0x0C);
		if (irq_status) {
			irq_triggered = 1;
			writel(irq_status, dev->hwregs + 0x0C); /* clear irq */
		}
	}
	if (irq_triggered == 1) {
		/* clear all IRQ bits. IRQ is cleared by writting 1 */
		spin_lock_irqsave(&parentslice->cache_owner_lock, flags);
		dev->irq_received = 1;
		dev->irq_status = irq_status;
		spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

		wake_up_interruptible_all(&parentslice->cache_wait_queue);
		handled++;
	}
	if (!handled)
		PDEBUG("IRQ received, but not cache's!\n");
	return IRQ_HANDLED;
}
#endif
#endif

static void ResetAsic(struct cache_dev_t *dev)
{
	int i;

	if (dev->is_valid == 0)
		return;
	for (i = 0; i < dev->core_cfg.iosize; i += 4)
		writel(0, dev->hwregs + i);
}
