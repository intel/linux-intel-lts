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

#include "hantrocache.h"

/********variables declaration related with race condition**********/

//struct semaphore enc_core_sem;

/*******************PCIE CONFIG*************************/
#ifdef PCI_BUS
#define PCI_VENDOR_ID_HANTRO            0x10ee//0x16c3
#define PCI_DEVICE_ID_HANTRO_PCI      0x8014// 0x7011// 0xabcd

/* Base address got control register */
#define PCI_H2_BAR              4

static struct pci_dev *gDev;    /* PCI device structure. */
static unsigned long gBaseHdwr;        /* PCI base register address (Hardware address) */
static u32 gBaseLen;                   /* Base register address Length */
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
#define CC_DIR_READ		"_DIRRD"
#define CC_DIR_WRITE	"_DIRWR"
#define CC_DIR_BIDIR	"_DIRBI"

#define RESOURCE_SHARED_INTER_CORES        0
#define CORE_0_IO_BASE                 0x700000//the base addr of core. for PCIE, it refers to offset from bar
#define CORE_1_IO_BASE                 0x700000//the base addr of core. for PCIE, it refers to offset from bar

#define CORE_2_IO_BASE                 0x780000//the base addr of core. for PCIE, it refers to offset from bar
#define CORE_3_IO_BASE                 0x780000//the base addr of core. for PCIE, it refers to offset from bar

#define CORE_0_IO_SIZE                 ((6+4*8) * 4)    //bytes cache register size
#define CORE_1_IO_SIZE                 ((5+5*8) * 4)    //bytes shaper register size


#define CORE_2_IO_SIZE                 ((6+4*8) * 4)    //bytes cache register size
#define CORE_3_IO_SIZE                 ((5+5*8) * 4)    // bytes shaper register size


#define INT_PIN_CORE_0                    -1
#define INT_PIN_CORE_1                    -1

#define INT_PIN_CORE_2                    -1
#define INT_PIN_CORE_3                    -1
/*for all cores, the core info should be listed here for later use*/
/*base_addr, iosize, irq*/
#ifndef USE_DTB_PROBE
static struct cache_core_config cache_core_array[] = {
	/*note parent address is copied from hx280enc.c's CORE_x_IO_ADDR and hantrodec.c's SOCLE_LOGIC_x_BASE */
	{VC8000E, CORE_0_IO_BASE, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_RD, 0, 0xc0000000}, //core_0 just support cache read for VC8000E
	{VC8000E, CORE_1_IO_BASE, CORE_1_IO_SIZE, INT_PIN_CORE_1, DIR_WR, 0, 0xd0000000}, //core_1 just support cache write for VC8000E
	{VC8000D_0, CORE_0_IO_BASE, CORE_0_IO_SIZE, INT_PIN_CORE_0, DIR_RD, 0, 0x20888000}, //core_0 just support cache read for VC8000D_0
	{VC8000D_0, CORE_1_IO_BASE, CORE_1_IO_SIZE, INT_PIN_CORE_1, DIR_WR, 0, 0x20888000}, //core_1 just support cache write for VC8000D_0
	{VC8000D_1, CORE_2_IO_BASE, CORE_2_IO_SIZE, INT_PIN_CORE_2, DIR_RD, 0, 0x20888800}, //core_0 just support cache read for VC8000D_1
	{VC8000D_1, CORE_3_IO_BASE, CORE_3_IO_SIZE, INT_PIN_CORE_3, DIR_WR, 0, 0x20888800} //core_1 just support cache write for VC8000D_1
};
#endif
static int bcacheprobed;

/*------------------------------END-------------------------------------*/

/***************************TYPE AND FUNCTION DECLARATION****************/

/* here's all the must remember stuff */
static int ReserveIO(struct cache_dev_t *);
static void ReleaseIO(struct cache_dev_t *);
static void ResetAsic(struct cache_dev_t *dev);
#ifndef PCI_BUS
#if (KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE)
static irqreturn_t cache_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
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

	if (wait_event_interruptible(parentslice->cache_wait_queue, CheckCacheIrq(dev))) {
		PDEBUG("Cache wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	return 0;
}

static int CheckCoreOccupation(struct cache_dev_t *dev)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_CACHE);
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&parentslice->cache_owner_lock, flags);
	if (!dev->is_reserved) {
		dev->is_reserved = 1;
		dev->pid = current->pid;
		ret = 1;
	}

	spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

	return ret;
}

static int GetWorkableCore(struct cache_dev_t *dev, u32 *core_id)
{
	int i = 0, ret = 0;
	driver_cache_dir dir = (*core_id) & 0x01;
	cache_client_type client = ((*core_id) & 0x06) >> 1;

	while (dev != NULL) {
	/* a valid free Core*/
		if (dev->is_valid && dev->core_cfg.client == client &&
			dev->core_cfg.dir == dir && CheckCoreOccupation(dev)) {
			ret = 1;
			*core_id = i;
			break;
		}
		i++;
		dev = dev->next;
	}
	return ret;
}

static long ReserveCore(struct cache_dev_t *dev, u32 *core_id)
{
	/*fixe me: dir bits is not enough for 3 choice */
	driver_cache_dir dir = (*core_id) & 0x01;
	cache_client_type client = ((*core_id) & 0x06) >> 1;
	int status = 0;
	struct cache_dev_t *pccore = dev;
	struct slice_info *parentslice;

	while (pccore != NULL) {
		/* a valid core supports such client and dir*/
		if (pccore->is_valid && pccore->core_cfg.client == client && pccore->core_cfg.dir == dir) {
			status = 1;
			break;
		}
		pccore = pccore->next;
	}
	if (status == 0) {
		PDEBUG(KERN_INFO "NO any core support client:%d,dir:%d.\n", dir, client);
		return -1;
	}
	parentslice = getparentslice(dev, CORE_CACHE);

	/* lock a core that has specified core id*/
	if (wait_event_interruptible(parentslice->cache_hw_queue, GetWorkableCore(dev, core_id) != 0))
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
		dev->pid = -1;
		dev->is_reserved = 0;
	}

	dev->irq_received = 0;
	dev->irq_status = 0;
	spin_unlock_irqrestore(&parentslice->cache_owner_lock, flags);

	wake_up_interruptible_all(&parentslice->cache_hw_queue);
}

long hantrocache_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;
	unsigned int tmp, id, slice, node;
	u32 core_id;
	unsigned long long tmp64;
	struct cache_dev_t *pccore;

	switch (cmd) {
	case CACHE_IOCGHWOFFSET:
		__get_user(id, (int *)arg);
		slice = SLICE(id);
		node = NODE(id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		else
			__put_user(pccore->com_base_addr, (unsigned long long*) arg);
		break;
	case CACHE_IOCGHWIOSIZE:
		__get_user(id, (int *)arg);
		slice = SLICE(id);
		node = NODE(id);
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
		break;
	case CACHE_IOCH_HW_RESERVE:
		__get_user(tmp64, (unsigned long long *)arg);
		slice = tmp64 >> 32;
		core_id = (u32)arg;//get client and direction info
		pccore = get_cachenodes(slice, 0);
		if (pccore == NULL)
			return -EFAULT;

		ret = ReserveCore(pccore, &core_id);
		if (ret == 0)
			return core_id;
		return ret;
	case CACHE_IOCH_HW_RELEASE:
		core_id = (u32)arg;
		slice = SLICE(core_id);
		node = NODE(core_id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		else
			ReleaseCore(pccore);
		break;
	case CACHE_IOCG_ABORT_WAIT:
		core_id = (u32)arg;
		slice = SLICE(core_id);
		node = NODE(core_id);
		pccore = get_cachenodes(slice, node);
		if (pccore == NULL)
			return -EFAULT;
		tmp = WaitCacheReady(pccore);
		if (tmp == 0)
			return pccore->irq_status;
		return 0;
	}
	return 0;
}

int cache_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int cache_release(void)
{
	int i, slicen = get_slicenumber();
	struct cache_dev_t *dev;

	for (i = 0; i < slicen; i++) {
		dev = get_cachenodes(i, 0);
		while (dev != NULL) {
			ReleaseCore(dev);
			dev = dev->next;
		}
	}
	return 0;
}

#ifdef PCI_BUS
static int PcieInit(int corenum)
{
	int i = 0;

	gDev = pci_get_device(PCI_VENDOR_ID_HANTRO, PCI_DEVICE_ID_HANTRO_PCI, gDev);
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
		cache_core_array[i].base_addr = gBaseHdwr + cache_core_array[i].base_addr;//the offset is based on which bus interface is chosen

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

int cache_init(void)
{
	int result = 0;

	bcacheprobed = 0;
#ifdef PCI_BUS
	result = PcieInit(sizeof(cache_core_array)/sizeof(struct cache_core_config));
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
int cache_probe(struct platform_device *pdev, struct hantro_core_info *prc, int corenum)
{
	int result;
	int i, irqnum, irqn = 0;
	struct cache_dev_t *pccore;

#ifndef USE_DTB_PROBE	/*simulate and compatible with old code*/
	if (bcacheprobed != 0)
		return 0;
	bcacheprobed = 1;
	for (i = 0; i < sizeof(cache_core_array) / sizeof(struct cache_core_config); i++) {
		pccore = vmalloc(sizeof(struct cache_dev_t));
		if (pccore == NULL)
			return -ENOMEM;

		memset(pccore, 0, sizeof(struct cache_dev_t));

		pccore->com_base_addr = pccore->core_cfg.base_addr = cache_core_array[i].base_addr;
		pccore->core_cfg.iosize = cache_core_array[i].iosize;
		pccore->core_cfg.client = cache_core_array[i].client;
		pccore->core_cfg.dir = cache_core_array[i].dir;
		pccore->core_cfg.sliceidx = cache_core_array[i].sliceidx;
		pccore->core_cfg.parentaddr = cache_core_array[i].parentaddr;
		//pccore->core_id = i;	//move to slice insertion

		result = ReserveIO(pccore);
		if (result < 0) {
			pr_info("cachecore: reserve reg 0x%llx-0x%llx fail\n",
				prc->mem->start, prc->mem->end);
			vfree(pccore);
			continue;
		}

		ResetAsic(pccore);  /* reset hardware */
#ifdef USE_IRQ
		irqn = 0;
		irqnum = cache_core_array[i].irq;
		if (irqnum > 0) {
			result = request_irq(irqnum, cache_isr,
#if (KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE)
				SA_INTERRUPT | SA_SHIRQ,
#else
				IRQF_SHARED,
#endif
				"hantro_cache", (void *)pccore);
			if (result == 0) {
				pccore->irqlist[irqn] = irqnum;
				irqn++;
			} else
				pr_info("cachecore: request IRQ <%d> fail\n", irqnum);
		}
#endif
		pccore->is_valid = 1;
		add_cachenode(pccore->core_cfg.sliceidx, pccore);
	}

#else	/*USE_DTB_PROBE*/
#ifndef PCI_BUS
	for (i = 0; i < corenum; i++) {
		int k, type, dir;

		if (strstr(prc->mem->name, RESNAME_CACHE) == prc->mem->name)
			goto looptail;
		cache_getcachetype(prc->mem->name, &type, &dir);
		if (type == -1 || dir == -1)
			goto looptail;
		pccore = vmalloc(sizeof(struct cache_dev_t));
		if (pccore == NULL)
			return -ENOMEM;

		memset(pccore, 0, sizeof(struct cache_dev_t));

		pccore->com_base_addr = pccore->core_cfg.base_addr = prc->mem->start;
		pccore->core_cfg.iosize = prc->mem->end - prc->mem->start + 1;
		pccore->core_cfg.client = type;
		pccore->core_cfg.dir = dir;
		//pccore->core_id = i;	////move to slice insertion

		result = ReserveIO(pccore);
		if (result < 0) {
			pr_info("cachecore: reserve reg 0x%llx-0x%llx fail\n",
				prc->mem->start, prc->mem->end);
			vfree(pccore);
			goto looptail;
		}

		ResetAsic(pccore);  /* reset hardware */
#ifdef USE_IRQ
		irqn = 0;
		for (k = 0; k < prc->irqnum; k++) {
			irqnum = platform_get_irq_byname(pdev, prc->irqlist[k]->name);
			if (irqnum > 0) {
				result = request_irq(irqnum, cache_isr,
#if (KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE)
					SA_INTERRUPT | SA_SHIRQ,
#else
					IRQF_SHARED,
#endif
					"hantro_cache", (void *)pccore);
				if (result == 0)
					pccore->irqlist[0] = irqnum;
				else
					pr_info("cachecore: request IRQ <%d> fail\n", irqnum);
			}
		}
#endif
		pccore->is_valid = 1;
		add_cachenode(0, pccore);
looptail:
		prc++;
	}
#endif	/*not def PCI_BUS*/
#endif	/*USE_DTB_PROBE*/
	return 0;
}

void cache_cleanup(void)
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
					free_irq(pccore->irqlist[k], (void *)pccore);
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
		PDEBUG(KERN_INFO "hantr_cache: failed to reserve HW regs,base_addr:%p\n", (void *)base_addr);
		return -1;
	}

	hwregs = (u8 *) ioremap_nocache(base_addr, 4);
	if (hwregs == NULL) {
		PDEBUG(KERN_INFO "hantr_cache: failed to ioremap HW regs\n");
		release_mem_region(base_addr, 4);
		return -1;
	}

	*hwid = readl(hwregs + 0x00);
	PDEBUG(KERN_INFO "hantro_cache: hwid = %x, base_addr= %p\n",
		(int)*hwid, (void *)base_addr);

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

	hw_cfg = (hwid & 0xF0000)>>16;

	if (hw_cfg > 2)
		return -1;

	if (hw_cfg == 1 && pccore->core_cfg.dir == DIR_WR)//cache only
		pccore->is_valid = 0;
	else if (hw_cfg == 2 && pccore->core_cfg.dir == DIR_RD)//shaper only
		pccore->is_valid = 0;
	else
		pccore->is_valid = 1;

	if (pccore->is_valid == 0)
		return -1;

	if (hwid == 0 && pccore->core_cfg.dir == DIR_RD)
		pccore->core_cfg.base_addr += CACHE_WITH_SHAPER_OFFSET;
	else if (hwid != 0) {
		if (pccore->core_cfg.dir == DIR_WR)
			pccore->core_cfg.base_addr += SHAPER_OFFSET;
		else if (pccore->core_cfg.dir == DIR_RD && hw_cfg == 0)
			pccore->core_cfg.base_addr += CACHE_WITH_SHAPER_OFFSET;
		else if (pccore->core_cfg.dir == DIR_RD && hw_cfg == 1)
			pccore->core_cfg.base_addr += CACHE_ONLY_OFFSET;
	}

	if (!request_mem_region(pccore->core_cfg.base_addr, pccore->core_cfg.iosize, "hantro_cache")) {
		PDEBUG(KERN_INFO "hantr_cache: failed to reserve HW regs,core:%d\n", i);
		pccore->is_valid = 0;
		return -1;
	}

	pccore->hwregs =
		(u8 *) ioremap_nocache(pccore->core_cfg.base_addr,
			pccore->core_cfg.iosize);

	if (pccore->hwregs == NULL) {
		PDEBUG(KERN_INFO "hantr_cache: failed to ioremap HW regs,core:%d\n", i);
		release_mem_region(pccore->core_cfg.base_addr, pccore->core_cfg.iosize);
		pccore->is_valid = 0;
		return -1;
	}

	if (pccore->core_cfg.dir == DIR_RD)
		PDEBUG("cache  reg[0x10]=%08x\n", readl(pccore->hwregs + 0x10));
	else
		PDEBUG("shaper reg[0x08]=%08x\n", readl(pccore->hwregs + 0x08));

	return 0;
}

static void ReleaseIO(struct cache_dev_t *pccore)
{
	if (pccore->is_valid == 0)
		return;
	if (pccore->hwregs)
		iounmap((void *) pccore->hwregs);
	release_mem_region(pccore->core_cfg.base_addr, pccore->core_cfg.iosize);
}

#ifndef PCI_BUS
#if (KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE)
static irqreturn_t cache_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t cache_isr(int irq, void *dev_id)
#endif
{
	unsigned int handled = 0;
	struct cache_dev_t *dev = (struct cache_dev_t *) dev_id;
	u32 irq_status;
	unsigned long flags;
	u32 irq_triggered = 0;
	struct slice_info *parentslice;

	parentslice = getparentslice(dev, CORE_CACHE);
	/*If core is not reserved by any user, but irq is received, just ignore it*/
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
			writel(irq_status, dev->hwregs + 0x04);//clear irq
		}
	} else {
		irq_status = readl(dev->hwregs + 0x0C);
		if (irq_status) {
			irq_triggered = 1;
			writel(irq_status, dev->hwregs + 0x0C);//clear irq
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

static void ResetAsic(struct cache_dev_t *dev)
{
	int i;

	if (dev->is_valid == 0)
		return;
	for (i = 0; i < dev->core_cfg.iosize; i += 4)
		writel(0, dev->hwregs + i);
}

