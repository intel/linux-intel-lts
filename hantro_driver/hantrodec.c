/*
 *    Hantro decoder hardware driver.
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

#include "hantrodec.h"
#include "dwl_defs.h"
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

/*hantro G1 regs config including dec and pp*/
#define HANTRO_DEC_ORG_REGS             60
#define HANTRO_PP_ORG_REGS              41

#define HANTRO_DEC_EXT_REGS             27
#define HANTRO_PP_EXT_REGS              9

#define HANTRO_G1_DEC_TOTAL_REGS   (HANTRO_DEC_ORG_REGS + HANTRO_DEC_EXT_REGS)
#define HANTRO_PP_TOTAL_REGS         (HANTRO_PP_ORG_REGS + HANTRO_PP_EXT_REGS)

#define HANTRO_DEC_ORG_FIRST_REG        0
#define HANTRO_DEC_ORG_LAST_REG         59
#define HANTRO_DEC_EXT_FIRST_REG        119
#define HANTRO_DEC_EXT_LAST_REG         145

#define HANTRO_PP_ORG_FIRST_REG         60
#define HANTRO_PP_ORG_LAST_REG          100
#define HANTRO_PP_EXT_FIRST_REG         146
#define HANTRO_PP_EXT_LAST_REG          154

/*hantro G2 reg config*/

#define HANTRO_G2_DEC_FIRST_REG            0
#define HANTRO_G2_DEC_LAST_REG             (HANTRO_G2_DEC_REGS - 1)

/* hantro VC8000D reg config */
#define HANTRO_VC8000D_FIRST_REG        0
#define HANTRO_VC8000D_LAST_REG         (HANTRO_VC8000D_REGS-1)

#define HANTRO_VC8KD_REG_BWREAD     300
#define HANTRO_VC8KD_REG_BWWRITE   304
#define VC8KD_BURSTWIDTH                         16


/********************************************************************
 *                                              PORTING SEGMENT
 * NOTES: customer should modify these configuration if do porting to own
 * platform. Please guarantee the base_addr, io_size,dec_irq belong to
 * same core.
 ********************************************************************/

#define HXDEC_MAX_CORES                 8

/* Logic module base address */
/* slice 0 */
#define SOCLE_LOGIC_0_BASE              0x18553A000 // VCDA
#define SOCLE_LOGIC_1_BASE              0x18553B000 // VCDB
/* slice 1 */
#define SOCLE_LOGIC_2_BASE              0x28553A000 // VCDA
#define SOCLE_LOGIC_3_BASE              0x28553B000 // VCDB
/* slice 2 */
#define SOCLE_LOGIC_4_BASE              0x38553A000 // VCDA
#define SOCLE_LOGIC_5_BASE              0x38553B000 // VCDB
/* slice 3 */
#define SOCLE_LOGIC_6_BASE              0x48553A000 // VCDA
#define SOCLE_LOGIC_7_BASE              0x48553B000 // VCDB

#define VEXPRESS_LOGIC_0_BASE           0xFC010000
#define VEXPRESS_LOGIC_1_BASE           0xFC020000

#define DEC_IO_SIZE_0                   DEC_IO_SIZE_MAX /* bytes */
#define DEC_IO_SIZE_1                   DEC_IO_SIZE_MAX /* bytes */

/* define to use request_irq call */
//#define USE_IRQ
#ifdef USE_IRQ
#define DEC_IRQ_0                       36 //just a dummy
#define DEC_IRQ_1                       33 //just a dummy
#define DEC_IRQ_2			-1 //for other slices using polling
#else
#define DEC_IRQ_0                       -1
#define DEC_IRQ_1                       -1
#define DEC_IRQ_2                       -1
#endif

/***********************************************************************/

#define IS_G1(hw_id)                    (((hw_id) == 0x6731) ? 1 : 0)
#define IS_G2(hw_id)                    (((hw_id) == 0x6732) ? 1 : 0)
#define IS_VC8000D(hw_id)               (((hw_id) == 0x8001) ? 1 : 0)

static const int DecHwId[] = {
	0x6731, /* G1 */
	0x6732, /* G2 */
	0x8001
};

#ifndef USE_DTB_PROBE
static unsigned long long multicorebase[HXDEC_MAX_CORES] = {
	SOCLE_LOGIC_0_BASE,
	SOCLE_LOGIC_1_BASE,
	SOCLE_LOGIC_2_BASE,
	SOCLE_LOGIC_3_BASE,
	SOCLE_LOGIC_4_BASE,
	SOCLE_LOGIC_5_BASE,
	SOCLE_LOGIC_6_BASE,
	SOCLE_LOGIC_7_BASE
};

static int irq[HXDEC_MAX_CORES] = {
	DEC_IRQ_0,
	DEC_IRQ_1,
	DEC_IRQ_2,
	DEC_IRQ_2,
	DEC_IRQ_2,
	DEC_IRQ_2,
	DEC_IRQ_2,
	DEC_IRQ_2
};

static unsigned int iosize[HXDEC_MAX_CORES] = {
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_0
};

static int sliceidxtable[HXDEC_MAX_CORES] = {
	0,
	0,
	1,
	1,
	2,
	2,
	3,
	3
};
#endif
//static int elements = 2;
static int bdecprobed;

static struct device *parent_dev;
#ifdef ENABLE_HANTRO_CLK
static struct clk *hantro_clk_g1;
static struct clk *hantro_clk_g2;
static struct clk *hantro_clk_bus;
#endif
static int hantro_dbg = -1;
#undef PDEBUG
#define PDEBUG(fmt, arg...)     \
	do {                                      \
		if (hantro_dbg > 0)\
			dev_info(parent_dev, fmt, ## arg); \
	} while (0)


/* here's all the must remember stuff */
struct hantrodec_ctrl {
	int cores;
};

static int ReserveIO(struct hantrodec_t *core, struct hantrodec_t **auxcore);
static void ReleaseIO(struct hantrodec_t *);

static void ResetAsic(struct hantrodec_t *dev);

#ifdef HANTRODEC_DEBUG
static void dump_regs(struct hantrodec_t *dev);
#endif

/* IRQ handler */
static irqreturn_t hantrodec_isr(int irq, void *dev_id);

atomic_t irq_rx = ATOMIC_INIT(0);
atomic_t irq_tx = ATOMIC_INIT(0);
/* spinlock_t owner_lock = SPIN_LOCK_UNLOCKED; */

#define DWL_CLIENT_TYPE_H264_DEC         1U
#define DWL_CLIENT_TYPE_MPEG4_DEC        2U
#define DWL_CLIENT_TYPE_JPEG_DEC         3U
#define DWL_CLIENT_TYPE_PP               4U
#define DWL_CLIENT_TYPE_VC1_DEC          5U
#define DWL_CLIENT_TYPE_MPEG2_DEC        6U
#define DWL_CLIENT_TYPE_VP6_DEC          7U
#define DWL_CLIENT_TYPE_AVS_DEC          8U
#define DWL_CLIENT_TYPE_RV_DEC           9U
#define DWL_CLIENT_TYPE_VP8_DEC          10U
#define DWL_CLIENT_TYPE_VP9_DEC          11U
#define DWL_CLIENT_TYPE_HEVC_DEC         12U

static u32 timeout;

#ifdef ENABLE_HANTRO_CLK
static int hantro_clk_enable(void)
{
	clk_prepare(hantro_clk_g1);
	clk_enable(hantro_clk_g1);
	clk_prepare(hantro_clk_g2);
	clk_enable(hantro_clk_g2);
	clk_prepare(hantro_clk_bus);
	clk_enable(hantro_clk_bus);
	return 0;
}

static int hantro_clk_disable(void)
{
	if (hantro_clk_g1) {
		clk_disable(hantro_clk_g1);
		clk_unprepare(hantro_clk_g1);
	}
	if (hantro_clk_g2) {
		clk_disable(hantro_clk_g2);
		clk_unprepare(hantro_clk_g2);
	}
	if (hantro_clk_bus) {
		clk_disable(hantro_clk_bus);
		clk_unprepare(hantro_clk_bus);
	}
	return 0;
}

static int hantro_ctrlblk_reset(void)
{
	u8 *iobase;
	//config G1/G2
	hantro_clk_enable();
	iobase = (u8 *)ioremap_nocache(BLK_CTL_BASE, 0x10000);
	iowrite32(0x3, (void *)iobase); //VPUMIX G1/G2 block soft reset
	iowrite32(0x3, (void *)iobase + 4); //VPUMIX G1/G2 clock enable
	iowrite32(0xFFFFFFFF, (void *)iobase + 0x8); //all G1 fuse dec enable
	iowrite32(0xFFFFFFFF, (void *)iobase + 0xC); //all G1 fuse pp enable
	iowrite32(0xFFFFFFFF, (void *)iobase + 0x10); //all G2 fuse dec enable
	iounmap(iobase);
	hantro_clk_disable();

	return 0;
}
#endif

static struct hantrodec_t *getcoreCtrl(u32 id)
{
	struct hantrodec_t *pcore;
	u32 slice = SLICE(id);
	u32 node = NODE(id);

	pcore = get_decnodes(slice, node);
	return pcore;
}

u32 hantrodec_readbandwidth(int isreadBW)
{
	int i, slicen = get_slicenumber();
	u32 bandwidth = 0;
	struct hantrodec_t *dev;

	for (i = 0; i < slicen; i++) {
		dev = get_decnodes(i, 0);
		while(dev != NULL) {
			if (isreadBW)
				bandwidth += ioread32((void *)(dev->hwregs + HANTRO_VC8KD_REG_BWREAD * 4));
			else
				bandwidth += ioread32((void *)(dev->hwregs + HANTRO_VC8KD_REG_BWWRITE * 4));
			dev = dev->next;
		}
	}
	return bandwidth * VC8KD_BURSTWIDTH;
}

static void ReadCoreConfig(struct hantrodec_t *dev)
{
	int c = dev->core_id;
	u32 reg, tmp, mask;
	struct hantrodec_t *next;

	dev->cfg = 0;

	/* Decoder configuration */
	if (IS_G1(dev->hw_id)) {
		reg =
			ioread32((void *)
			(dev->hwregs + HANTRODEC_SYNTH_CFG * 4));

		tmp = (reg >> DWL_H264_E) & 0x3U;
		if (tmp)
			pr_info("hantrodec: core[%d] has H264\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

		tmp = (reg >> DWL_JPEG_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has JPEG\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

		tmp = (reg >> DWL_MPEG4_E) & 0x3U;
		if (tmp)
			pr_info("hantrodec: core[%d] has MPEG4\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

		tmp = (reg >> DWL_VC1_E) & 0x3U;
		if (tmp)
			pr_info("hantrodec: core[%d] has VC1\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

		tmp = (reg >> DWL_MPEG2_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has MPEG2\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

		tmp = (reg >> DWL_VP6_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has VP6\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

		reg =
			ioread32((void *)
			(dev->hwregs + HANTRODEC_SYNTH_CFG_2 * 4));

		/* VP7 and WEBP is part of VP8 */
		mask = (1 << DWL_VP8_E) |
				(1 << DWL_VP7_E) |
				(1 << DWL_WEBP_E);
		tmp = (reg & mask);
		if (tmp & (1 << DWL_VP8_E))
			pr_info("hantrodec: core[%d] has VP8\n", c);
		if (tmp & (1 << DWL_VP7_E))
			pr_info("hantrodec: core[%d] has VP7\n", c);
		if (tmp & (1 << DWL_WEBP_E))
			pr_info("hantrodec: core[%d] has WebP\n", c);
		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

		tmp = (reg >> DWL_AVS_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has AVS\n", c);
		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

		tmp = (reg >> DWL_RV_E) & 0x03U;
		if (tmp)
			pr_info("hantrodec: core[%d] has RV\n", c);
		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

		/* Post-processor configuration */
		reg =
			ioread32((void *)(dev->hwregs +
				HANTROPP_SYNTH_CFG * 4));

		tmp = (reg >> DWL_G1_PP_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has PP\n", c);
		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
	} else if ((IS_G2(dev->hw_id))) {
		reg = ioread32((void *)(dev->hwregs +
			HANTRODEC_CFG_STAT * 4));

		tmp = (reg >> DWL_G2_HEVC_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has HEVC\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

		tmp = (reg >> DWL_G2_VP9_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has VP9\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

		/* Post-processor configuration */
		reg = ioread32((void *)(dev->hwregs +
			HANTRODECPP_SYNTH_CFG * 4));

		tmp = (reg >> DWL_G2_PP_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has PP\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
	} else if ((IS_VC8000D(dev->hw_id)) &&
		dev->its_main_core_id == NULL) {
		reg = ioread32((void *)(dev->hwregs +
				HANTRODEC_SYNTH_CFG * 4));

		tmp = (reg >> DWL_H264_E) & 0x3U;
		if (tmp)
			pr_info("hantrodec: core[%d] has H264\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

		tmp = (reg >> DWL_H264HIGH10_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has H264HIGH10\n", c);
		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

		tmp = (reg >> DWL_JPEG_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has JPEG\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

		tmp = (reg >> DWL_MPEG4_E) & 0x3U;
		if (tmp)
			pr_info("hantrodec: core[%d] has MPEG4\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

		tmp = (reg >> DWL_VC1_E) & 0x3U;
		if (tmp)
			pr_info("hantrodec: core[%d] has VC1\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

		tmp = (reg >> DWL_MPEG2_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has MPEG2\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

		tmp = (reg >> DWL_VP6_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has VP6\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

		reg = ioread32((void *)(dev->hwregs +
				HANTRODEC_SYNTH_CFG_2 * 4));

		/* VP7 and WEBP is part of VP8 */
		mask = (1 << DWL_VP8_E) |
				(1 << DWL_VP7_E) |
				(1 << DWL_WEBP_E);
		tmp = (reg & mask);
		if (tmp & (1 << DWL_VP8_E))
			pr_info("hantrodec: core[%d] has VP8\n", c);
		if (tmp & (1 << DWL_VP7_E))
			pr_info("hantrodec: core[%d] has VP7\n", c);
		if (tmp & (1 << DWL_WEBP_E))
			pr_info("hantrodec: core[%d] has WebP\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

		tmp = (reg >> DWL_AVS_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has AVS\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

		tmp = (reg >> DWL_RV_E) & 0x03U;
		if (tmp)
			pr_info("hantrodec: core[%d] has RV\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

		reg = ioread32((void *)(dev->hwregs +
				HANTRODEC_SYNTH_CFG_3 * 4));

		tmp = (reg >> DWL_HEVC_E) & 0x07U;
		if (tmp)
			pr_info("hantrodec: core[%d] has HEVC\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

		tmp = (reg >> DWL_VP9_E) & 0x07U;
		if (tmp)
			pr_info("hantrodec: core[%d] has VP9\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

		/* Post-processor configuration */
		reg = ioread32((void *)(dev->hwregs +
				HANTRODECPP_CFG_STAT * 4));

		tmp = (reg >> DWL_PP_E) & 0x01U;
		if (tmp)
			pr_info("hantrodec: core[%d] has PP\n", c);
		dev->cfg |=
			tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;

		if (dev->its_aux_core_id != NULL) {
			/* set main_core_id and aux_core_id */
			next = dev->its_aux_core_id;
			reg = ioread32((void *)(next->hwregs +
					HANTRODEC_SYNTH_CFG_2 * 4));

			tmp = (reg >> DWL_H264_PIPELINE_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has pipeline H264\n", c);
			next->cfg |=
				tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_PIPELINE_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has pipeline JPEG\n", c);
			next->cfg |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;
		}
	}
	dev->cfg_backup = dev->cfg;
}

static int CoreHasFormat(const u32 cfg, u32 format)
{
	return (cfg & (1 << format)) ? 1 : 0;
}

static int GetDecCore(
	long core,
	struct hantrodec_t *dev,
	struct file *filp,
	unsigned long format)
{
	int success = 0;
	unsigned long flags;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	spin_lock_irqsave(&parentslice->owner_lock, flags);
	if (CoreHasFormat(dev->cfg, format) &&
		/*&& config.its_main_core_id[core] >= 0*/
		dev->dec_owner == NULL) {
		dev->dec_owner = filp;
		success = 1;

		/* If one main core takes one format which doesn't supported
		 * by aux core, set aux core's cfg to none video format support
		 */
		if (dev->its_aux_core_id != NULL &&
			!CoreHasFormat(
				dev->its_aux_core_id->cfg,
				format)) {
			dev->its_aux_core_id->cfg = 0;
		}
		/* If one aux core takes one format,
		 *set main core's cfg to aux core supported video format
		 */
		else if (dev->its_main_core_id != NULL) {
			dev->its_main_core_id->cfg =
				dev->cfg;
		}
	}

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	return success;
}

static int GetDecCoreAny(
	long *core,
	struct hantrodec_t *dev,
	struct file *filp,
	unsigned long format)
{
	int success = 0;
	long c = 0;

	*core = -1;

	while (dev != NULL) {
		/* a free core that has format */
		if (GetDecCore(c, dev, filp, format)) {
			success = 1;
			*core = c;
			break;
		}
		c++;
		dev = dev->next;
	}

	return success;
}

static int GetDecCoreID(
	struct hantrodec_t *dev,
	struct file *filp,
	unsigned long format)
{
	long c = 0;
	unsigned long flags;
	int core_id = -1;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	while (dev!= NULL) {
		/* a core that has format */
		spin_lock_irqsave(&parentslice->owner_lock, flags);
		if (CoreHasFormat(dev->cfg, format)) {
			core_id = c;
			spin_unlock_irqrestore(&parentslice->owner_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&parentslice->owner_lock, flags);
		dev = dev->next;
		c++;
	}
	return core_id;
}


static long ReserveDecoder(struct hantrodec_t *dev, struct file *filp, unsigned long format)
{
	long core = -1;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	/* reserve a core */
	if (down_interruptible(&parentslice->dec_core_sem))
		return -ERESTARTSYS;

	/* lock a core that has specific format*/
	if (wait_event_interruptible(parentslice->hw_queue,
		GetDecCoreAny(&core, dev, filp, format) != 0))
		return -ERESTARTSYS;

	return core;
}

static void ReleaseDecoder(struct hantrodec_t *dev, long core)
{
	u32 status;
	unsigned long flags;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	status = ioread32((void *)(dev->hwregs +
				HANTRODEC_IRQ_STAT_DEC_OFF));

	/* make sure HW is disabled */
	if (status & HANTRODEC_DEC_E) {
		pr_info("hantrodec: DEC[%li] still enabled -> reset\n", core);

		/* abort decoder */
		status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status, (void *)(dev->hwregs +
				HANTRODEC_IRQ_STAT_DEC_OFF));
	}

	spin_lock_irqsave(&parentslice->owner_lock, flags);

	/* If aux core released, revert main core's config back */
	if (dev->its_main_core_id != NULL)
		dev->its_main_core_id->cfg = dev->its_main_core_id->cfg_backup;

	/* If main core released, revert aux core's config back */
	if (dev->its_aux_core_id != NULL)
		dev->its_aux_core_id->cfg = dev->its_aux_core_id->cfg_backup;

	dev->dec_owner = NULL;

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	up(&parentslice->dec_core_sem);

	wake_up_interruptible_all(&parentslice->hw_queue);
}

static long ReservePostProcessor(struct hantrodec_t *dev, struct file *filp)
{
	unsigned long flags;
	long core = 0;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	/* single core PP only */
	if (down_interruptible(&parentslice->pp_core_sem))
		return -ERESTARTSYS;

	spin_lock_irqsave(&parentslice->owner_lock, flags);
	if (dev != NULL)
		dev->pp_owner = filp;

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	return core;
}

static void ReleasePostProcessor(struct hantrodec_t *dev, long core)
{
	unsigned long flags;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	u32 status = ioread32((void *)(dev->hwregs +
			HANTRO_IRQ_STAT_PP_OFF));

	/* make sure HW is disabled */
	if (status & HANTRO_PP_E) {
		pr_info("hantrodec: PP[%li] still enabled -> reset\n", core);

		/* disable IRQ */
		status |= HANTRO_PP_IRQ_DISABLE;

		/* disable postprocessor */
		status &= (~HANTRO_PP_E);
		iowrite32(0x10, (void *)(dev->hwregs +
				HANTRO_IRQ_STAT_PP_OFF));
	}

	spin_lock_irqsave(&parentslice->owner_lock, flags);

	dev->pp_owner = NULL;

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	up(&parentslice->pp_core_sem);
}
#if 0
static long ReserveDecPp(struct hantrodec_t *dev, struct file *filp, unsigned long format)
{
	/* reserve core 0, DEC+PP for pipeline */
	unsigned long flags;

	long core = 0;

	/* check that core has the requested dec format */
	if (!CoreHasFormat(dev->cfg, format))
		return -EFAULT;

	/* check that core has PP */
	if (!CoreHasFormat(dev->cfg, DWL_CLIENT_TYPE_PP))
		return -EFAULT;

	/* reserve a core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* wait until the core is available */
	if (wait_event_interruptible(hw_queue,
		GetDecCore(core, dev, filp, format) != 0)) {
		up(&dec_core_sem);
		return -ERESTARTSYS;
	}

	if (down_interruptible(&pp_core_sem)) {
		ReleaseDecoder(dev, core);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&owner_lock, flags);
	dev->pp_owner = filp;
	spin_unlock_irqrestore(&owner_lock, flags);

	return core;
}
#endif
static long DecFlushRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0, i;

	ret = copy_from_user(dev->dec_regs, core->regs, HANTRO_VC8000D_REGS * 4);
	if (ret) {
		PDEBUG("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	iowrite32(0x0, (void *)(dev->hwregs + 4));
	for (i = 2; i <= HANTRO_VC8000D_LAST_REG; i++)
		iowrite32(dev->dec_regs[i], (void *)(dev->hwregs + i * 4));

	/* write the status register, which may start the decoder */
	iowrite32(dev->dec_regs[1], (void *)(dev->hwregs + 4));

	return 0;
}

static long DecRefreshRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret, i;

	for (i = 0; i <= HANTRO_VC8000D_LAST_REG; i++)
		dev->dec_regs[i] = ioread32((void *)(dev->hwregs + i * 4));

	ret = copy_to_user(core->regs, dev->dec_regs,
			HANTRO_VC8000D_LAST_REG * 4);
	if (ret) {
		PDEBUG("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}
	return 0;
}

static int CheckDecIrq(struct hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);
	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&parentslice->owner_lock, flags);

	if (parentslice->dec_irq & irq_mask) {
		/* reset the wait condition(s) */
		parentslice->dec_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	return rdy;
}

static long WaitDecReadyAndRefreshRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	u32 id = NODE(Core->id);
	long ret;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	PDEBUG("wait_event_interruptible DEC[%d]\n", id);

	ret = wait_event_interruptible_timeout(parentslice->dec_wait_queue, CheckDecIrq(dev, id), msecs_to_jiffies(10));
	if (ret == -ERESTARTSYS) {
		pr_err("DEC[%d]  failed to wait_event_interruptible interrupted\n", id);
		return -ERESTARTSYS;
	} else if (ret == 0) {
		pr_err("DEC[%d]  wait_event_interruptible timeout\n", id);
		timeout = 1;
		return -EBUSY;
	}
	atomic_inc(&irq_tx);

	/* refresh registers */
	return DecRefreshRegs(dev, Core);
}

static long DecWriteRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0, i;

	i = core->reg_id;
	ret = copy_from_user(dev->dec_regs + core->reg_id,
			core->regs + core->reg_id, 4);
	if (ret) {
		PDEBUG("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}
	iowrite32(dev->dec_regs[i], (void *)dev->hwregs + i * 4);
	return 0;
}

u32 *hantrodec_getRegAddr(u32 coreid, u32 regid)
{
	int i;
	struct hantrodec_t *dev = get_decnodes(SLICE(coreid), NODE(coreid));

	if (dev == NULL)
		return NULL;

	for (i = 0; i < coreid; i++)
		dev = dev->next;
	if (regid * 4 >= dev->iosize)
		return NULL;
	return (u32 *)(dev->hwregs + regid * 4);
}

static long DecReadRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret, i;

	i = core->reg_id;
	/* user has to know exactly what they are asking for */
	//if(core->size != (HANTRO_VC8000D_REGS * 4))
	//  return -EFAULT;

	/* read specific registers from hardware */
	i = core->reg_id;
	dev->dec_regs[i] = ioread32((void *)dev->hwregs + i * 4);

	/* put registers to user space*/
	ret = copy_to_user(core->regs + core->reg_id,
			dev->dec_regs + core->reg_id, 4);
	if (ret) {
		PDEBUG("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}
	return 0;
}

static long PPFlushRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	long ret = 0;
	u32 i;

	/* copy original dec regs to kernal space*/
	ret = copy_from_user(dev->dec_regs + HANTRO_PP_ORG_FIRST_REG,
			Core->regs + HANTRO_PP_ORG_FIRST_REG,
			HANTRO_PP_ORG_REGS * 4);
#ifdef USE_64BIT_ENV
	/* copy extended dec regs to kernal space*/
	ret = copy_from_user(dev->dec_regs + HANTRO_PP_EXT_FIRST_REG,
			Core->regs + HANTRO_PP_EXT_FIRST_REG,
			HANTRO_PP_EXT_REGS * 4);
#endif
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	/* both original and extended regs need to be written */
	for (i = HANTRO_PP_ORG_FIRST_REG + 1;
		i <= HANTRO_PP_ORG_LAST_REG;
		i++)
		iowrite32(dev->dec_regs[i], (void *)dev->hwregs + i * 4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG;
		i <= HANTRO_PP_EXT_LAST_REG;
		i++)
		iowrite32(dev->dec_regs[i], (void *)dev->hwregs + i * 4);
#endif
	/* write the stat reg, which may start the PP */
	iowrite32(dev->dec_regs[HANTRO_PP_ORG_FIRST_REG],
		(void *)dev->hwregs +
		HANTRO_PP_ORG_FIRST_REG * 4);

	return 0;
}

static long PPRefreshRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	long i, ret;
#ifdef USE_64BIT_ENV
	/* user has to know exactly what they are asking for */
	if (Core->size != (HANTRO_PP_TOTAL_REGS * 4))
		return -EFAULT;
#else
	/* user has to know exactly what they are asking for */
	if (Core->size != (HANTRO_PP_ORG_REGS * 4))
		return -EFAULT;
#endif

	/* read all registers from hardware */
	/* both original and extended regs need to be read */
	for (i = HANTRO_PP_ORG_FIRST_REG; i <= HANTRO_PP_ORG_LAST_REG; i++)
		dev->dec_regs[i] = ioread32((void *)dev->hwregs + i * 4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
		dev->dec_regs[i] = ioread32((void *)dev->hwregs + i * 4);
#endif
	/* put registers to user space*/
	/* put original registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_ORG_FIRST_REG,
		dev->dec_regs + HANTRO_PP_ORG_FIRST_REG, HANTRO_PP_ORG_REGS * 4);
#ifdef USE_64BIT_ENV
	/* put extended registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_EXT_FIRST_REG,
			dev->dec_regs + HANTRO_PP_EXT_FIRST_REG,
			HANTRO_PP_EXT_REGS * 4);
#endif
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int CheckPPIrq(struct hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&parentslice->owner_lock, flags);

	if (parentslice->pp_irq & irq_mask) {
		/* reset the wait condition(s) */
		parentslice->pp_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	return rdy;
}

static long WaitPPReadyAndRefreshRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	u32 id = Core->id;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	PDEBUG("wait_event_interruptible PP[%d]\n", id);

	if (wait_event_interruptible(parentslice->pp_wait_queue, CheckPPIrq(dev, id))) {
		pr_err("PP[%d]  failed to wait_event_interruptible interrupted\n", id);
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);

	/* refresh registers */
	return PPRefreshRegs(dev, Core);
}

static int CheckCoreIrq(struct hantrodec_t *dev, const struct file *filp, u32 *id)
{
	unsigned long flags;
	int rdy = 0, n = 0;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	while (dev != NULL) {
		u32 irq_mask = (1 << n);

		spin_lock_irqsave(&parentslice->owner_lock, flags);

		if (parentslice->dec_irq & irq_mask) {
			if (*id == n) {	//if(pcore->dec_owner == filp)
				/* we have an IRQ for our client */

				/* reset the wait condition(s) */
				parentslice->dec_irq &= ~irq_mask;

				/* signal ready Core no. for our client */
				rdy = 1;
				spin_unlock_irqrestore(&parentslice->owner_lock, flags);
				break;
			} else if (dev->dec_owner == NULL) {
				/* zombie IRQ */
				pr_info("IRQ on Core[%d], but no owner!!!\n", n);

				/* reset the wait condition(s) */
				parentslice->dec_irq &= ~irq_mask;
			}
		}

		spin_unlock_irqrestore(&parentslice->owner_lock, flags);

		n++; /* next Core */
		dev = dev->next;
	}

	return rdy;
}

static long WaitCoreReady(struct hantrodec_t *dev, const struct file *filp, u32 *id)
{
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);
	PDEBUG("wait_event_interruptible CORE\n");

	if (wait_event_interruptible(parentslice->dec_wait_queue,
		CheckCoreIrq(dev, filp, id))) {
		pr_err("CORE  failed to wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);

	return 0;
}

/*-------------------------------------------------------------------------
 *Function name   : hantrodec_ioctl
 *Description     : communication method to/from the user space
 *
 *Return type     : long
 *-------------------------------------------------------------------------
 */

long hantrodec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int i;
	u32 hw_id, id;
	u32 slice, node;
	long tmp = 0;
	unsigned long long tmp64;
	struct core_desc core;
	struct hantrodec_t *pcore;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_CLI): {
		id = arg;
		pcore = getcoreCtrl(id);
		if (pcore == NULL)
			return -EFAULT;
		for (i = 0; i < 4; i++)
			if (pcore->irqlist[i] > 0)
				disable_irq(pcore->irqlist[i]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_STI): {
		id = arg;
		pcore = getcoreCtrl(id);
		if (pcore == NULL)
			return -EFAULT;
		for (i = 0; i < 4; i++)
			if (pcore->irqlist[i] > 0)
				enable_irq(pcore->irqlist[i]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET): {
		__get_user(id, (unsigned long *)arg);
		pcore = getcoreCtrl(id);
		if (pcore == NULL)
			return -EFAULT;

		__put_user(pcore->multicorebase_actual, (unsigned long long*) arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE): {
		__u32 io_size;

		__get_user(id, (__u32 *)arg);
		pcore = getcoreCtrl(id);
		if (pcore == NULL)
			return -EFAULT;
		io_size = pcore->iosize;
		__put_user(io_size, (u32 *) arg);

		return 0;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS): {
		__get_user(slice, (__u32 *)arg);
		pcore = get_decnodes(slice, 0);
		if (pcore == NULL)
			return -EFAULT;
		i = 0;
		while (pcore != NULL) {
			tmp = copy_to_user(((unsigned long long*) arg) + i,
					&pcore->multicorebase_actual,
					sizeof(pcore->multicorebase_actual));
			if (tmp) {
				pr_err("copy_to_user failed, returned %li\n", tmp);
				return -EFAULT;
			}
			pcore = pcore->next;
			i++;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_CORES):
		id = (u32) arg;
		id = get_slicecorenum(id, CORE_DEC);
		PDEBUG("cores=%d\n", id);
		return id;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
		/* get registers from user space*/
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;
		DecFlushRegs(pcore, &core);
		break;
	}

	case _IOC_NR(HANTRODEC_IOCS_DEC_WRITE_REG): {
		/* get registers from user space*/
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			PDEBUG("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;

		DecWriteRegs(pcore, &core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG): {
		/* get registers from user space*/
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;
		PPFlushRegs(pcore, &core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;
		return DecRefreshRegs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_READ_REG): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			PDEBUG("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;
		return DecReadRegs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;
		return PPRefreshRegs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
		__get_user(tmp64, (unsigned long long *)arg);
		slice = tmp64 >> 32;
		pcore = getcoreCtrl(slice<<16);
		if (pcore == NULL)
			return -EFAULT;
		i = ReserveDecoder(pcore, filp,(u32)tmp64);
		if (i < 0)
			return -EFAULT;
		else
			return i | (slice << 16);
	}
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
		pcore = getcoreCtrl((u32)arg);
		if (pcore == NULL)
			return -EFAULT;
		if (pcore->dec_owner != filp) {
			pr_err("bogus DEC release, Core = %li\n", arg);
			return -EFAULT;
		}

		PDEBUG("Release DEC, core = %li\n", arg);

		ReleaseDecoder(pcore, arg);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
		id = (u32)arg;
		if ((pcore = get_decnodes(SLICE(id), 0)) == NULL)
			return -EFAULT;
		return ReservePostProcessor(pcore, filp);
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE): {
		pcore = getcoreCtrl(arg);
		if (pcore == NULL)
			return -EFAULT;
		if (arg != 0 || pcore->pp_owner != filp) {
			pr_err("bogus PP release %li\n", arg);
			return -EFAULT;
		}
		ReleasePostProcessor(pcore, arg);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;

		return WaitDecReadyAndRefreshRegs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		pcore = getcoreCtrl(core.id);
		if (pcore == NULL)
			return -EFAULT;

		return WaitPPReadyAndRefreshRegs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT): {
		id = (u32)arg;
		slice = SLICE(id);
		node = NODE(id);
		if ((pcore = get_decnodes(slice, 0)) == NULL)
			return -EFAULT;
		tmp = WaitCoreReady(pcore, filp, &node);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID): {
		id = (u32)arg;
		pcore = getcoreCtrl(id);
		if (pcore == NULL)
			return 0;
		id = ioread32((void *)pcore->hwregs);
		return id;
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
		PDEBUG("Get DEC Core_id, format = %li\n", arg);
		__get_user(tmp64, (unsigned long long *)arg);
		slice = tmp64 >> 32;
		if ((pcore = get_decnodes(slice, 0)) == NULL)
			return -EFAULT;
		tmp = GetDecCoreID(pcore, filp, tmp64 & 0xffffffff);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_BUILD_ID): {
		__get_user(id, (int *)arg);
		pcore = getcoreCtrl(id);
		if (pcore == NULL)
			return -EFAULT;
		hw_id = ioread32((void *)(pcore->hwregs));
		if (IS_G1(hw_id >> 16) || IS_G2(hw_id >> 16))
			__put_user(hw_id, (u32 *) arg);
		else {
			hw_id = ioread32((void *)(pcore->hwregs + HANTRODEC_HW_BUILD_ID_OFF));
			__put_user(hw_id, (u32 *) arg);
		}
		return 0;
	}
	case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
		struct slice_info *parentslice;

		PDEBUG("hantrodec: IRQs received/sent2user = %d / %d\n",
	       	atomic_read(&irq_rx), atomic_read(&irq_tx));
		slice = get_slicenumber();
		for (i = 0; i < (int)slice; i++) {
			pcore = get_decnodes(i, 0);
			if (pcore == NULL)
				continue;
			parentslice = getparentslice(pcore, CORE_DEC);
			PDEBUG("hantrodec: slice %d dec_irq     = 0x%08x\n", i, parentslice->dec_irq);
			PDEBUG("hantrodec: slice %d pp_irq      = 0x%08x\n", i, parentslice->pp_irq);

			id = 0;
			while (pcore != NULL) {
				PDEBUG("hantrodec: slice %d dec_core[%i] %s\n",
				       i, id, pcore->dec_owner == NULL ?
				       "FREE" : "RESERVED");
				PDEBUG("hantrodec: slice %d pp_core[%i]  %s\n",
				       i, id, pcore->pp_owner == NULL ?
				       "FREE" : "RESERVED");
				pcore = pcore->next;
				id++;
			}
		}
		break;
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_release
 *Description     : Release driver
 *
 *Return type     : int
 *----------------------------------------------------------------------------
 */
int hantrodec_release(struct file *filp)
{
	int n, i, slicen = get_slicenumber();
	struct hantrodec_t *pcore;

	for (i = 0; i < slicen; i++) {
		pcore = get_decnodes(i, 0);
		n = 0;
		while (pcore != NULL) {
			if (pcore->dec_owner == filp) {
				PDEBUG("releasing slice %d dec Core %i lock\n", i, n);
				ReleaseDecoder(pcore, n);
			}
			n++;
			pcore = pcore->next;
		}

		pcore = get_decnodes(i, 0);
		if (pcore != NULL && pcore->pp_owner == filp) {
			PDEBUG("releasing slice %d pp Core %i lock\n", i, 0);
			ReleasePostProcessor(pcore, n);
		}
	}

	PDEBUG("closed\n");
	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_init
 *Description     : Initialize the driver
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */

int hantrodec_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int hantrodec_init(void)
{
	bdecprobed = 0;

	return 0;
}

int hantrodec_probe(struct platform_device *pdev, struct hantro_core_info *prc, int corenum)
{
	int result = 0;
	int irqnum, i;
#ifdef USE_DTB_PROBE
	int k, irqn;
#endif
	struct hantrodec_t *pcore, *auxcore;

	parent_dev = &pdev->dev;
#ifndef USE_DTB_PROBE	/*simulate and compatible with old code*/
	if (bdecprobed != 0)
		return 0;
	bdecprobed = 1;
	for (i = 0; i < sizeof(multicorebase) / sizeof(multicorebase[0]); i++) {
		if (multicorebase[i] == 0)
			break;
		pcore = vmalloc(sizeof(struct hantrodec_t));
		if (pcore == NULL)
			return -ENOMEM;

		memset(pcore, 0, sizeof(struct hantrodec_t));
		//pcore->core_id = i;	//move to insertion to slice
		pcore->multicorebase = pcore->multicorebase_actual = multicorebase[i];
		pcore->iosize = iosize[i];

		auxcore = NULL;
		result = ReserveIO(pcore, &auxcore);
		if (result < 0) {
			vfree(pcore);
			continue;
		}
		pr_info("reserveIO success\n");

		ReadCoreConfig(pcore);
		ResetAsic(pcore);
		pcore->dec_owner = pcore->pp_owner = NULL;
		pcore->sliceidx = sliceidxtable[i];

		if (auxcore != NULL) {
			ReadCoreConfig(auxcore);
			ResetAsic(auxcore);
			auxcore->dec_owner = auxcore->pp_owner = NULL;
			auxcore->sliceidx = pcore->sliceidx;
		}
		add_decnode(pcore->sliceidx, pcore);
		if (auxcore != NULL)
			add_decnode(auxcore->sliceidx, auxcore);

#ifdef USE_IRQ
		irqnum = irq[i];
		/* FIXME: To dynamically get the IRQ numbers from device-tree */
		if (irqnum > 0 && i==0) {
			int irq_num0 = platform_get_irq_byname(pdev, "irq_hantro_decoderA");
			result = request_irq(irq_num0, hantrodec_isr, IRQF_SHARED,
					"irq_hantro_decoderA", (void *)pcore);
			if (result != 0) {
				pr_info("dec can't reserve irq %d\n", irqnum);
			} else {
				pr_info("dec irq = %d for core <%d> success\n", irqnum, i);
				pcore->irqlist[0] = irqnum;
			}
		}

                if (irqnum > 0 && i==1) {
			int irq_num1 = platform_get_irq_byname(pdev, "irq_hantro_decoderB");
                       	result = request_irq(irq_num1, hantrodec_isr, IRQF_SHARED,
					"irq_hantro_decoderB", (void *)pcore);
                       if (result != 0) {
                               pr_info("dec can't reserve irq %d\n", irqnum);
                       } else {
                               pr_info("dec irq = %d for core <%d> success\n", irqnum, i);
                               pcore->irqlist[0] = irqnum;
                       }
                }

#endif
	}
#else	/*USE_DTB_PROBE*/
	for (i = 0; i < corenum; i++) {
		if (strstr(prc->mem->name, RESNAME_DECODER) == prc->mem->name)
			goto looptail;

		pcore = vmalloc(sizeof(struct hantrodec_t));
		if (pcore == NULL)
			return -ENOMEM;

		memset(pcore, 0, sizeof(struct hantrodec_t));
		pcore->multicorebase = pcore->multicorebase_actual = prc->mem->start;
		pcore->iosize = prc->mem->end - prc->mem->start + 1;

		auxcore = NULL;
		result = ReserveIO(pcore, &auxcore);
		if (result < 0) {
			vfree(pcore);
			goto looptail;
		}
		pr_info("reserveIO success\n");

		ReadCoreConfig(pcore);
		ResetAsic(pcore);
		pcore->dec_owner = pcore->pp_owner = NULL;
		/*fixme: only slice 0 for simulation until device tree is ready*/
		pcore->sliceidx = 0;

		if (auxcore != NULL) {
			ReadCoreConfig(auxcore);
			ResetAsic(auxcore);
			auxcore->dec_owner = auxcore->pp_owner = NULL;
			auxcore->sliceidx = pcore->sliceidx;
		}
		add_decnode(0, pcore);
		if (auxcore != NULL)
			add_decnode(0, auxcore);

		/* register irq for each core*/
#ifdef USE_IRQ
		irqn = 0;
		for (k = 0; k < prc->irqnum; k++) {
			irqnum = platform_get_irq_byname(pdev, prc->irqlist[k]->name);
			if (irqnum > 0) {
				result = request_irq(irqnum, hantrodec_isr, IRQF_SHARED,
					"irq_hantro_c1", (void *)pcore);
				if (result != 0) {
					pr_info("dec can't reserve irq %d\n", irqnum);
				} else {
					pr_info("dec irq = %d\n", irqnum);
					pcore->irqlist[irqn] = irqnum;
					irqn++;
				}
			}
		}
#endif
looptail:
		prc++;
	}
#endif	/*USE_DTB_PROBE*/

	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_cleanup
 *Description     : clean up
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
void hantrodec_cleanup(void)
{
	int slicen = get_slicenumber();
	struct hantrodec_t *dev, *next;
	int n, i;

	for (n = 0; n < slicen; n++) {
		/* free the IRQ */
		dev = get_decnodes(n, 0);
		while (dev != NULL) {
			/* reset hardware */
			ResetAsic(dev);
			for (i = 0; i < 4; i++)
				if (dev->irqlist[i] > 0)
					free_irq(dev->irqlist[i], (void *) dev);
			ReleaseIO(dev);
			next = dev->next;
			vfree(dev);
			dev = dev->next;
		}
	}
	bdecprobed = 0;
}

/*---------------------------------------------------------------------------
 *Function name   : CheckHwId
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int CheckHwId(struct hantrodec_t *dev)
{
	long hwid;
	size_t num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
	int found = 0;

	hwid = readl(dev->hwregs);
	hwid = (hwid >> 16) & 0xFFFF; /* product version only */

	while (num_hw--) {
		if (hwid == DecHwId[num_hw]) {
			pr_info("hantrodec: Supported HW found at 0x%16llx\n",
			       dev->multicorebase_actual);
			found = 1;
			dev->hw_id = hwid;
			break;
		}
	}
	if (!found) {
		pr_info("hantrodec: Unknown HW found at 0x%16llx\n",
		       dev->multicorebase_actual);
		return 0;
	}
	return 1;
}

/*---------------------------------------------------------------------------
 *Function name   : ReserveIO
 *Description     : IO reserve
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int ReserveIO(struct hantrodec_t *core, struct hantrodec_t **auxcore)
{
	int result;
	long hwid;
	u32 reg;

	if (!request_mem_region(core->multicorebase_actual,
			core->iosize,
			"hantrodec0")) {
		pr_info("hantrodec: failed to reserve HW regs\n");
		return -EBUSY;
	}

	core->hwregs =
		(u8 *) ioremap_nocache(
				core->multicorebase_actual,
				core->iosize);

	if (core->hwregs == NULL) {
		pr_info("hantrodec: failed to ioremap HW regs\n");
		release_mem_region(core->multicorebase_actual, core->iosize);
		return -EBUSY;
	}
	core->its_main_core_id = NULL;
	core->its_aux_core_id = NULL;

	/* product version only */
	hwid = ((readl(core->hwregs)) >> 16) & 0xFFFF;

	if (IS_VC8000D(hwid)) {
		reg = readl(core->hwregs + HANTRODEC_SYNTH_CFG_2_OFF);
		if (((reg >> DWL_H264_PIPELINE_E) & 0x01U) ||
			((reg >> DWL_JPEG_PIPELINE_E) & 0x01U)) {
			*auxcore = vmalloc(sizeof(struct hantrodec_t));
			if (*auxcore == NULL) {
				result = -ENOMEM;
				goto error;
			}
			memset(*auxcore, 0, sizeof(struct hantrodec_t));
			(*auxcore)->multicorebase_actual = core->multicorebase_actual + 0x800;
			(*auxcore)->multicorebase = (*auxcore)->multicorebase_actual;
			(*auxcore)->iosize = core->iosize;
			if (!request_mem_region(
					(*auxcore)->multicorebase_actual,
					(*auxcore)->iosize,
					"hantrodec0")) {
				pr_info("hantrodec: failed to reserve HW regs\n");
				result = -EBUSY;
				goto error;
			}

			(*auxcore)->hwregs = (u8 *) ioremap_nocache((*auxcore)->multicorebase_actual,
				(*auxcore)->iosize);

			if ((*auxcore)->hwregs == NULL) {
				pr_info("hantrodec: failed to ioremap HW regs\n");
				release_mem_region((*auxcore)->multicorebase_actual, (*auxcore)->iosize);
				result = -EBUSY;
				goto error;
			}

			core->its_aux_core_id = *auxcore;
			(*auxcore)->its_main_core_id = core;
			(*auxcore)->its_aux_core_id = NULL;
		}
	}

	/* check for correct HW */
	result = CheckHwId(core);
	if (!result) {
		result = -ENXIO;
		goto error;
	}
	if (*auxcore) {
		result = CheckHwId(*auxcore);
		if (!result) {
			result = -ENXIO;
			goto error;
		}
	}

	return 0;

error:
	ReleaseIO(core);
	if (*auxcore) {
		ReleaseIO(*auxcore);
		vfree(*auxcore);
		*auxcore = NULL;
	}
	return result;
}

/*---------------------------------------------------------------------------
 *Function name   : releaseIO
 *Description     : release
 *
 *Return type     : void
 *---------------------------------------------------------------------------
 */
static void ReleaseIO(struct hantrodec_t *dev)
{
	if (dev->hwregs)
		iounmap((void *)dev->hwregs);
	release_mem_region(dev->multicorebase_actual, dev->iosize);
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_isr
 *Description     : interrupt handler
 *
 *Return type     : irqreturn_t
 *---------------------------------------------------------------------------
 */
static irqreturn_t hantrodec_isr(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int handled = 0;
	int i = 0;

	u8 *hwregs;
	struct hantrodec_t *dev = (struct hantrodec_t *)dev_id;
	u32 irq_status_dec;
	struct slice_info *parentslice = getparentslice(dev, CORE_DEC);

	dev = get_decnodes(dev->sliceidx, 0);
	spin_lock_irqsave(&parentslice->owner_lock, flags);

	while (dev != NULL) {
		u8 *hwregs = dev->hwregs;

		/* interrupt status register read */
		irq_status_dec = ioread32((void *)hwregs +
			HANTRODEC_IRQ_STAT_DEC_OFF);
		pr_info("irq = %x\n", irq_status_dec);
		if (irq_status_dec & HANTRODEC_DEC_IRQ) {
			/* clear dec IRQ */
			irq_status_dec &= (~HANTRODEC_DEC_IRQ);
			iowrite32(irq_status_dec, (void *)hwregs +
					HANTRODEC_IRQ_STAT_DEC_OFF);

			PDEBUG("decoder IRQ received! Core %d\n", i);

			atomic_inc(&irq_rx);

			parentslice->dec_irq |= (1 << i);

			wake_up_interruptible_all(&parentslice->dec_wait_queue);
			handled++;
		}
		i++;
		dev = dev->next;
	}

	spin_unlock_irqrestore(&parentslice->owner_lock, flags);

	if (!handled)
		pr_info("IRQ received, but not hantrodec's!\n");

	(void)hwregs;
	return IRQ_RETVAL(handled);
}

/*---------------------------------------------------------------------------
 *Function name   : ResetAsic
 *Description     : reset asic
 *
 *Return type     :
 *---------------------------------------------------------------------------
 */
static void ResetAsic(struct hantrodec_t *dev)
{
	int i;
	u32 status;

	status = ioread32((void *)dev->hwregs +
			HANTRODEC_IRQ_STAT_DEC_OFF);

	if (status & HANTRODEC_DEC_E) {
		/* abort with IRQ disabled */
		status = HANTRODEC_DEC_ABORT |
				HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status, (void *)dev->hwregs +
				HANTRODEC_IRQ_STAT_DEC_OFF);
	}

	if (IS_G1(dev->hw_id))
		/* reset PP */
		iowrite32(0, (void *)dev->hwregs +
			HANTRO_IRQ_STAT_PP_OFF);

	for (i = 4; i < dev->iosize; i += 4)
		iowrite32(0, (void *)dev->hwregs + i);
}

