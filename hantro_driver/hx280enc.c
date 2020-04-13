/*
 *    Hantro encoder hardware driver.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License, version 2, as
 *    published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License version 2 for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 at the following locations:
 *    https://opensource.org/licenses/gpl-2.0.php
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
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include "hx280enc.h"
#include <linux/irq.h>

static u32 resouce_shared;

/*------------------------------------------------------------------------
 *****************************PORTING LAYER********************************
 *-------------------------------------------------------------------------
 */
/*0:no resource sharing inter cores 1: existing resource sharing*/
#define RESOURCE_SHARED_INTER_CORES        0

// slice 0
/*customer specify according to own platform*/
#define CORE_0_IO_ADDR                 0x185539000 // VCE
#define CORE_0_IO_SIZE                 (500 * 4)    /* bytes */
/*customer specify according to own platform*/
#define CORE_1_IO_ADDR                 0x185538000 // VCEJ
#define CORE_1_IO_SIZE                 (500 * 4)    /* bytes */

// slice 1
/*customer specify according to own platform*/
#define CORE_2_IO_ADDR                 0x285539000 // VCE
#define CORE_2_IO_SIZE                 (500 * 4)    /* bytes */
/*customer specify according to own platform*/
#define CORE_3_IO_ADDR                 0x285538000 // VCEJ
#define CORE_3_IO_SIZE                 (500 * 4)    /* bytes */

// slice 2
/*customer specify according to own platform*/
#define CORE_4_IO_ADDR                 0x385539000 // VCE
#define CORE_4_IO_SIZE                 (500 * 4)    /* bytes */
/*customer specify according to own platform*/
#define CORE_5_IO_ADDR                 0x385538000 // VCEJ
#define CORE_5_IO_SIZE                 (500 * 4)    /* bytes */

// slice 3
/*customer specify according to own platform*/
#define CORE_6_IO_ADDR                 0x485539000 // VCE
#define CORE_6_IO_SIZE                 (500 * 4)    /* bytes */
/*customer specify according to own platform*/
#define CORE_7_IO_ADDR                 0x485538000 // VCEJ
#define CORE_7_IO_SIZE                 (500 * 4)    /* bytes */

#define INT_PIN_CORE_0                    -1        /*IRQ pin of core 0*/
#define INT_PIN_CORE_1                    -1        /*IRQ pin of core 1*/

#define HANTRO_VC8KE_REG_BWREAD 216
#define HANTRO_VC8KE_REG_BWWRITE 220
#define VC8KE_BURSTWIDTH                 16

/*for all cores, the core info should be listed here for subsequent use*/
/*base_addr, iosize, irq, resource_shared*/
#ifndef USE_DTB_PROBE
static CORE_CONFIG core_array[] = {
	{CORE_0_IO_ADDR, CORE_0_IO_SIZE, INT_PIN_CORE_0, RESOURCE_SHARED_INTER_CORES, 0}, 
	{CORE_1_IO_ADDR, CORE_1_IO_SIZE, INT_PIN_CORE_1, RESOURCE_SHARED_INTER_CORES, 0},
	{CORE_2_IO_ADDR, CORE_2_IO_SIZE, INT_PIN_CORE_0, RESOURCE_SHARED_INTER_CORES, 1}, 
	{CORE_3_IO_ADDR, CORE_3_IO_SIZE, INT_PIN_CORE_1, RESOURCE_SHARED_INTER_CORES, 1},
	{CORE_4_IO_ADDR, CORE_4_IO_SIZE, INT_PIN_CORE_0, RESOURCE_SHARED_INTER_CORES, 2}, 
	{CORE_5_IO_ADDR, CORE_5_IO_SIZE, INT_PIN_CORE_1, RESOURCE_SHARED_INTER_CORES, 2},
	{CORE_6_IO_ADDR, CORE_6_IO_SIZE, INT_PIN_CORE_0, RESOURCE_SHARED_INTER_CORES, 3}, 
	{CORE_7_IO_ADDR, CORE_7_IO_SIZE, INT_PIN_CORE_1, RESOURCE_SHARED_INTER_CORES, 3},
};
#endif
static int bencprobed;

/*------------------------------END-------------------------------------*/

/***************************TYPE AND FUNCTION DECLARATION****************/

/* here's all the must remember stuff */

static int ReserveIO(struct hantroenc_t *pcore);
static void ReleaseIO(struct hantroenc_t *pcore);
static void ResetAsic(struct hantroenc_t *dev);
static int CheckCoreOccupation(struct hantroenc_t *dev);
static void ReleaseEncoder(struct hantroenc_t *dev, u32 *core_info, u32 nodenum);

/* IRQ handler */
#if KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE
static irqreturn_t hantroenc_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
static irqreturn_t hantroenc_isr(int irq, void *dev_id);
#endif

/*********************local variable declaration*****************/
unsigned long long sram_base;
unsigned int sram_size;
/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hantroenc_major;

/******************************************************************************/
static int CheckEncIrq(struct hantroenc_t *dev, u32 *core_info, u32 *irq_status, u32 nodenum)
{
	unsigned long flags;
	int rdy = 0;
	u32 i = 0;
	u8 core_mapping = 0;
	struct slice_info *parentslice = getparentslice(dev, CORE_ENC);

	core_mapping = (u8)(*core_info & 0xFF);

	while (core_mapping) {
		if (core_mapping & 0x1) {
			if (i >= nodenum)
				break;

			spin_lock_irqsave(&parentslice->enc_owner_lock, flags);

			if (dev->irq_received) {
				/* reset the wait condition(s) */
				PDEBUG("check %d irq ready\n", i);
				dev->irq_received = 0;
				rdy = 1;
				*core_info = i;
				*irq_status = dev->irq_status;
			}

			spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);
			break;
		}
		core_mapping = core_mapping >> 1;
		i++;
		dev = dev->next;
	}
	return rdy;
}
static unsigned int WaitEncReady(struct hantroenc_t *dev, u32 *core_info, u32 *irq_status, u32 nodenum)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_ENC);

	PDEBUG("%s\n", __func__);

	if (wait_event_interruptible(
		parentslice->enc_wait_queue,
		CheckEncIrq(dev, core_info, irq_status, nodenum))) {
		PDEBUG("ENC wait_event_interruptible interrupted\n");
		ReleaseEncoder(dev, core_info, nodenum);
		return -ERESTARTSYS;
	}

	return 0;
}

u32 hantroenc_readbandwidth(int sliceidx, int isreadBW)
{
	int i, slicen = get_slicenumber();
	u32 bandwidth = 0;
	struct hantroenc_t *pcore;

	if (sliceidx < 0) {
		for (i = 0; i < slicen; i++) {
			pcore = get_encnodes(i, 0);
			while (pcore != NULL) {
				if (isreadBW)
					bandwidth += ioread32((void *)(pcore->hwregs + HANTRO_VC8KE_REG_BWREAD * 4));
				else
					bandwidth += ioread32((void *)(pcore->hwregs + HANTRO_VC8KE_REG_BWWRITE * 4));
				pcore = pcore->next;
			}
		}
	} else {
		pcore = get_encnodes(sliceidx, 0);
		while (pcore != NULL) {
			if (isreadBW)
				bandwidth += ioread32((void *)(pcore->hwregs + HANTRO_VC8KE_REG_BWREAD * 4));
			else
				bandwidth += ioread32((void *)(pcore->hwregs + HANTRO_VC8KE_REG_BWWRITE * 4));
			pcore = pcore->next;
		}
	}
	return bandwidth * VC8KE_BURSTWIDTH;
}

static int CheckCoreOccupation(struct hantroenc_t *dev)
{
	int ret = 0;
	unsigned long flags;
	struct slice_info *parentslice = getparentslice(dev, CORE_ENC);
	
	spin_lock_irqsave(&parentslice->enc_owner_lock, flags);
	if (!dev->is_reserved) {
		dev->is_reserved = 1;
		dev->pid = current->pid;
		ret = 1;
		PDEBUG("%s pid=%d\n", __func__, dev->pid);
	}

	spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);

	return ret;
}

static int GetWorkableCore(struct hantroenc_t *dev, u32 *core_info, u32 *core_info_tmp, u32 nodenum)
{
	int ret = 0;
	u32 i = 0;
	u32 cores;
	u32 core_id = 0;
	u8 core_mapping = 0;
	u32 required_num = 0;

	cores = *core_info;
	required_num = ((cores >> CORE_INFO_AMOUNT_OFFSET) & 0x7) + 1;
	core_mapping = (u8)(cores & 0xFF);

	if (*core_info_tmp == 0)
		*core_info_tmp = required_num << 8;
	else
		required_num = ((*core_info_tmp & 0xF00) >> 8);

	PDEBUG("%s:required_num=%d,core_info=%x\n",
		__func__, required_num, *core_info);

	if (required_num) {
		/* a valid free Core that has specified core id */
		while (core_mapping) {
			if (core_mapping & 0x1) {
				if (i >= nodenum)
					break;
				core_id = i;
				if (CheckCoreOccupation(dev)) {
					*core_info_tmp = ((((*core_info_tmp & 0xF00) >> 8) - 1) << 8) | (*core_info_tmp & 0x0FF);
					*core_info_tmp = *core_info_tmp | (1 << core_id);
					if (((*core_info_tmp & 0xF00) >> 8) == 0) {
						ret = 1;
						*core_info = (*core_info & 0xFFFFFF00) | (*core_info_tmp & 0xFF);
						*core_info_tmp = 0;
						required_num = 0;
						break;
					}
				}
			}
			core_mapping = core_mapping >> 1;
			i++;
			dev = dev->next;
		}
	} else
		ret = 1;

	PDEBUG("*core_info = %x\n", *core_info);
	return ret;
}

static long ReserveEncoder(struct hantroenc_t *dev, u32 *core_info, u32 nodenum)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_ENC);
	u32 core_info_tmp = 0;
	/*If HW resources are shared inter cores, just make sure only one is using the HW*/
	if (resouce_shared) {
		if (down_interruptible(&parentslice->enc_core_sem))
			return -ERESTARTSYS;
	}

	/* lock a core that has specified core id*/
	if (wait_event_interruptible(parentslice->enc_hw_queue,
		GetWorkableCore(dev, core_info, &core_info_tmp, nodenum) != 0))
		return -ERESTARTSYS;

	return 0;
}

static void ReleaseEncoder(struct hantroenc_t *dev, u32 *core_info, u32 nodenum)
{
	unsigned long flags;
	u32 core_num = 0;
	u32 i = 0, core_id;
	u8 core_mapping = 0;
	struct slice_info *parentslice = getparentslice(dev, CORE_ENC);

	core_num = ((*core_info >> CORE_INFO_AMOUNT_OFFSET) & 0x7) + 1;

	core_mapping = (u8)(*core_info & 0xFF);

	PDEBUG("%s:core_num=%d,core_mapping=%x\n", __func__, core_num, core_mapping);
	/* release specified core id */
	while (core_mapping) {
		if (core_mapping & 0x1) {
			if (i >= nodenum)
				break;
			core_id = i;
			spin_lock_irqsave(&parentslice->enc_owner_lock, flags);
			PDEBUG("dev[core_id].pid=%d,current->pid=%d\n", dev->pid, current->pid);
			if (dev->is_reserved && dev->pid == current->pid) {
				dev->pid = -1;
				dev->is_reserved = 0;
				dev->irq_received = 0;
				dev->irq_status = 0;
			}
			spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);

			//wake_up_interruptible_all(&enc_hw_queue);
		}
		core_mapping = core_mapping >> 1;
		i++;
		dev = dev->next;
	}

	wake_up_interruptible_all(&parentslice->enc_hw_queue);

	if (resouce_shared)
		up(&parentslice->enc_core_sem);
}

long hantroenc_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	unsigned int id, tmp, node, slice;
	struct hantroenc_t *pcore;
	u32 core_info;

	switch (cmd) {
	case HX280ENC_IOCGHWOFFSET: {
		__get_user(id, (unsigned long long *)arg);
		node = KCORE(id);
		slice = SLICE(id);
		pcore = get_encnodes(slice, node);
		if (pcore == NULL)
			return -EFAULT;

		__put_user(pcore->core_cfg.base_addr, (unsigned long long *) arg);
		break;
	}

	case HX280ENC_IOCGHWIOSIZE: {
		u32 io_size;
		__get_user(id, (unsigned long *)arg);
		node = KCORE(id);
		slice = SLICE(id);
		pcore = get_encnodes(slice, node);
		if (pcore == NULL)
			return -EFAULT;
		io_size = pcore->core_cfg.iosize;
		__put_user(io_size, (u32 *) arg);
		return 0;
	}
	case HX280ENC_IOCGSRAMOFFSET:
		__put_user(sram_base, (unsigned long long *) arg);
		break;
	case HX280ENC_IOCGSRAMEIOSIZE:
		__put_user(sram_size, (unsigned int *) arg);
		break;
	case HX280ENC_IOCG_CORE_NUM:
		tmp = arg;
		return get_slicecorenum(tmp, CORE_ENC);
	case HX280ENC_IOCH_ENC_RESERVE: {
		int ret;
		PDEBUG("Reserve ENC Cores\n");
		__get_user(core_info, (unsigned long *)arg);
		slice = (core_info >> 16) & 0xff;
		pcore = get_encnodes(slice, 0);		/*from list header*/
		if (pcore == NULL) {
			printk("wrong slice num");
			return -EFAULT;
		}
		tmp = get_slicecorenum(slice, CORE_ENC);
		ret = ReserveEncoder(pcore, &core_info, tmp);
		if (ret == 0)
			__put_user(core_info, (u32 *) arg);
		return ret;
	}
	case HX280ENC_IOCH_ENC_RELEASE: {
		__get_user(core_info, (unsigned long *)arg);
		slice = (core_info >> 16) & 0xff;
		pcore = get_encnodes(slice, 0);		/*from list header*/
		if (pcore == NULL)
			return -EFAULT;
		PDEBUG("Release ENC Core\n");
		tmp = get_slicecorenum(slice, CORE_ENC);	
		ReleaseEncoder(pcore, &core_info, tmp);

		break;
	}

	case HX280ENC_IOCG_CORE_WAIT: {
		u32 irq_status;

		__get_user(core_info, (u32 *)arg);
		slice = SLICE(core_info);
		pcore = get_encnodes(slice, 0);
		if (pcore == NULL)
			return -EFAULT;
		tmp = get_slicecorenum(slice, CORE_ENC);

		tmp = WaitEncReady(pcore, &core_info, &irq_status, tmp);
		if (tmp == 0) {
			__put_user(irq_status, (unsigned int *)arg);
			return core_info;//return core_id
		}
		__put_user(0, (unsigned int *)arg);
		return -1;

		break;
	}
	}
	return 0;
}

int hantroenc_release(void)
{
	struct slice_info *parentslice;
	int i, slicen = get_slicenumber();
	struct hantroenc_t *dev;
	unsigned long flags;

	for (i = 0; i < slicen; i++) {
		dev = get_encnodes(i, 0);
		if (dev == NULL)
			continue;
		parentslice = getparentslice(dev, CORE_ENC);
		while (dev != NULL) {
			spin_lock_irqsave(&parentslice->enc_owner_lock, flags);
			if (dev->is_reserved == 1 && dev->pid == current->pid) {
				dev->pid = -1;
				dev->is_reserved = 0;
				dev->irq_received = 0;
				dev->irq_status = 0;
				PDEBUG("release reserved core\n");
			}
			spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);
			dev = dev->next;
		}
		wake_up_interruptible_all(&parentslice->enc_hw_queue);
		if (resouce_shared)
			up(&parentslice->enc_core_sem);
	}
	return 0;
}

int __init hantroenc_init(void)
{
	sram_base = 0;
	sram_size = 0;
	hantroenc_major = 0;
	resouce_shared = 0;
	bencprobed = 0;
	return 0;
}

int hantroenc_probe(dtbnode *pnode)
{
	int result = 0;
	struct hantroenc_t *pcore;
	int i;

#ifndef USE_DTB_PROBE		/*simulate and compatible with old code*/
	if (bencprobed != 0)
		return 0;
	bencprobed = 1;
	for (i = 0; i < ARRAY_SIZE(core_array); i++) {
		int irqnum;
		pcore = vmalloc(sizeof(struct hantroenc_t));
		if (pcore == NULL)
			continue;

		memset(pcore, 0, sizeof(struct hantroenc_t));
		pcore->core_cfg.base_addr = core_array[i].base_addr;
		pcore->core_cfg.iosize = core_array[i].iosize;
		pcore->core_cfg.sliceidx = core_array[i].sliceidx;

		result = ReserveIO(pcore);
		if (result < 0) {
			pr_err("hx280enc: reserve reg 0x%llx-0x%lld fail\n",
				pcore->core_cfg.base_addr, pcore->core_cfg.iosize);
			vfree(pcore);
			continue;
		}

		ResetAsic(pcore);  /* reset hardware */

#ifdef USE_IRQ
		irqnum = core_array[i].irq;
		if (irqnum > 0) {
			result = request_irq(irqnum, hantroenc_isr,
						IRQF_SHARED,
						"hx280enc", (void *)pcore);
			if (result == 0)
				pcore->irqlist[0] = irqnum;
			else {
				pr_err("hx280enc: request IRQ <%d> fail\n", irqnum);
				ReleaseIO(pcore);
				vfree(pcore);
				continue;
			}
		}
#endif
		add_encnode(pcore->core_cfg.sliceidx, pcore);
	}
#else	/*USE_DTB_PROBE*/
	{
		int irqn;

		pcore = vmalloc(sizeof(struct hantroenc_t));
		if (pcore == NULL)
			return -ENOMEM;

		memset(pcore, 0, sizeof(struct hantroenc_t));
		pcore->core_cfg.base_addr = pnode->ioaddr;
		pcore->core_cfg.iosize = pnode->iosize;

		result = ReserveIO(pcore);
		if (result < 0) {
			pr_err("hx280enc: reserve reg 0x%llx:%lldfail\n",
				pnode->ioaddr, pnode->iosize);
			vfree(pcore);
			return -ENODEV;
		}

		ResetAsic(pcore);  /* reset hardware */
		irqn = 0;
		for (i = 0; i < 4; i++)
			pcore->irqlist[i] = -1; 
#ifdef USE_IRQ
		for (i = 0; i < 4; i++) {
			if (pnode->irq[i] > 0) {
				strcpy(pcore->irq_name[i], pnode->irq_name[i]);
				result = request_irq(pnode->irq[i], hantroenc_isr,
							IRQF_SHARED,
							pcore->irq_name[i], (void *)pcore);
				if (result == 0) {
					pcore->irqlist[irqn] = pnode->irq[i];
					irqn++;
				} else {
					printk("hx280enc: request IRQ <%d> fail\n", pnode->irq[i]);
					ReleaseIO(pcore);
					vfree(pcore);
					return -EINVAL;
				}
			}
		}
#endif
		add_encnode(pnode->sliceidx, pcore);
		printk("add enc node %lx:%d @ slice %d:%lx", (unsigned long)pcore, pcore->core_id,
			pnode->sliceidx, (unsigned long)getslicenode(pnode->sliceidx));
	}
#endif	/*USE_DTB_PROBE*/
	pr_info("hx280enc: module inserted. Major <%d>\n", hantroenc_major);

	return 0;
}

void __exit hantroenc_cleanup(void)
{
	int i, k, slicen = get_slicenumber();
	struct hantroenc_t *pcore, *pnext;

	for (i = 0; i < slicen; i++) {
		pcore = get_encnodes(i, 0);
		while (pcore != NULL) {
			u32 hwId = pcore->hw_id;
			u32 majorId = (hwId & 0x0000FF00) >> 8;
			u32 wClr = (majorId >= 0x61) ? (0x1FD) : (0);

			pnext = pcore->next;
			iowrite32(0, (void *)(pcore->hwregs + 0x14)); /* disable HW */
			iowrite32(wClr, (void *)(pcore->hwregs + 0x04)); /* clear enc IRQ */

			/* free the encoder IRQ */
			for (k = 0; k < 4; k++)
				if (pcore->irqlist[k] > 0)
					free_irq(pcore->irqlist[k], (void *)pcore);
			ReleaseIO(pcore);
			vfree(pcore);
			pcore = pnext;
		}
	}
	bencprobed = 0;
	pr_info("hantroenc: module removed\n");
}

static int ReserveIO(struct hantroenc_t *pcore)
{
	u32 hwid;

	if (!request_mem_region
		(pcore->core_cfg.base_addr, pcore->core_cfg.iosize, pcore->reg_name)) {
		pr_info("hantroenc: failed to reserve HW regs\n");
		return -1;
	}

	pcore->hwregs = (u8 *) ioremap(pcore->core_cfg.base_addr,
							pcore->core_cfg.iosize);
	if (pcore->hwregs == NULL) {
		pr_info("hantroenc: failed to ioremap HW regs\n");
		release_mem_region(pcore->core_cfg.base_addr, pcore->core_cfg.iosize);
		return -1;
	}

	/*read hwid and check validness and store it*/
	hwid = (u32)ioread32((void *)pcore->hwregs);
	pr_info("hwid=0x%08x\n", hwid);

	/* check for encoder HW ID */
	if (((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF))) &&
		((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF)))) {
		pr_info("hantroenc: HW not found at %llx\n",
			pcore->core_cfg.base_addr);
		ReleaseIO(pcore);
		return -1;
	}
	pcore->hw_id = hwid;

	pr_info("hantroenc: HW at base <%llx> with ID <0x%08x>\n",
	       pcore->core_cfg.base_addr, hwid);

	return 0;
}

static void ReleaseIO(struct hantroenc_t *pcore)
{
	if (pcore->hwregs)
		iounmap((void *) pcore->hwregs);
	release_mem_region(pcore->core_cfg.base_addr, pcore->core_cfg.iosize);
}

#if KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE
static irqreturn_t hantroenc_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t hantroenc_isr(int irq, void *dev_id)
#endif
{
	unsigned int handled = 0;
	struct hantroenc_t *dev = (struct hantroenc_t *) dev_id;
	u32 irq_status;
	unsigned long flags;
	struct slice_info *parentslice = getparentslice(dev, CORE_ENC);

	/*If core is not reserved by any user, but irq is received, just ignore it*/
	spin_lock_irqsave(&parentslice->enc_owner_lock, flags);
	if (!dev->is_reserved) {
		printk("hantroenc_isr:received IRQ but core is not reserved!\n");
		irq_status = (u32)ioread32((void *)(dev->hwregs + 0x04));
		if (irq_status & 0x01) {
			/* clear all IRQ bits. (hwId >= 0x80006100) means IRQ is cleared by writting 1 */
			u32 hwId = ioread32((void *)dev->hwregs);
			u32 majorId = (hwId & 0x0000FF00) >> 8;
			u32 wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));

			/*  Disable HW when buffer over-flow happen
			*  HW behavior changed in over-flow
			*    in-pass, HW cleanup HWIF_ENC_E auto
			*    new version:  ask SW cleanup HWIF_ENC_E when buffer over-flow
			*/
			if (irq_status & 0x20)
				iowrite32(0, (void *)(dev->hwregs + 0x14));
			iowrite32(wClr, (void *)(dev->hwregs + 0x04));
		}
		spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);

	irq_status = (u32)ioread32((void *)(dev->hwregs + 0x04));
	if (irq_status & 0x01) {
		/* clear all IRQ bits. (hwId >= 0x80006100) means IRQ is cleared by writting 1 */
		u32 hwId = ioread32((void *)dev->hwregs);
		u32 majorId = (hwId & 0x0000FF00) >> 8;
		u32 wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));

		if (irq_status & 0x20)
			iowrite32(0, (void *)(dev->hwregs + 0x14));
		iowrite32(wClr, (void *)(dev->hwregs + 0x04));
		spin_lock_irqsave(&parentslice->enc_owner_lock, flags);
		dev->irq_received = 1;
		dev->irq_status = irq_status & (~0x01);
		spin_unlock_irqrestore(&parentslice->enc_owner_lock, flags);

		wake_up_interruptible_all(&parentslice->enc_wait_queue);
		handled++;
	}
	if (!handled)
		printk("IRQ received, but not hantro enc's!\n");

	return IRQ_HANDLED;
}

static void ResetAsic(struct hantroenc_t *dev)
{
	int i;

	iowrite32(0, (void *)(dev->hwregs + 0x14));
	for (i = 4; i < dev->core_cfg.iosize; i += 4)
		iowrite32(0, (void *)(dev->hwregs + i));
}

