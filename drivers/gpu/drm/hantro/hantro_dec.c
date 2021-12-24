// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro decoder hardware driver.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

#include "hantro_priv.h"
#include "hantro_dec.h"
#include "hantro_dwl_defs.h"

#define KMB_VC8000D_PAGE_LUT           0x20889000

/* hantro G1 regs config including dec and pp */
#define HANTRO_PP_ORG_REGS		41

#define HANTRO_PP_EXT_REGS		9

#define HANTRO_PP_TOTAL_REGS		(HANTRO_PP_ORG_REGS + HANTRO_PP_EXT_REGS)

#define HANTRO_PP_ORG_FIRST_REG		60
#define HANTRO_PP_ORG_LAST_REG		100
#define HANTRO_PP_EXT_FIRST_REG		146
#define HANTRO_PP_EXT_LAST_REG		154

/* hantro VC8000D reg config */
#define HANTRO_VC8000D_LAST_REG		(HANTRO_VC8000D_REGS - 1)

#define HANTRO_VC8KD_REG_BWREAD		300
#define HANTRO_VC8KD_REG_BWWRITE	304
#define VC8KD_BURSTWIDTH		16

#define IS_G1(hw_id)		(((hw_id) == 0x6731) ? 1 : 0)
#define IS_G2(hw_id)		(((hw_id) == 0x6732) ? 1 : 0)
#define IS_VC8000D(hw_id)	(((hw_id) == 0x8001) ? 1 : 0)

static const int dec_hwid[] = { 0x6731, /* G1 */
				0x6732, /* G2 */
				0x8001 };
#undef PDEBUG
#define PDEBUG(fmt, arg...)                                                    \
	do {                                                                   \
		if (verbose)                                                   \
			pr_info(fmt, ##arg);                                   \
	} while (0)

static int reserve_io(struct hantrodec_t *core, struct hantrodec_t **auxcore);
static void release_io(struct hantrodec_t *dev);
static void reset_asic(struct hantrodec_t *dev);

/* IRQ handler */
static irqreturn_t hantrodec_isr(int irq, void *dev_id);

static atomic_t irq_rx = ATOMIC_INIT(0);
static atomic_t irq_tx = ATOMIC_INIT(0);

#define DWL_CLIENT_TYPE_H264_DEC	1U
#define DWL_CLIENT_TYPE_MPEG4_DEC	2U
#define DWL_CLIENT_TYPE_JPEG_DEC	3U
#define DWL_CLIENT_TYPE_PP		4U
#define DWL_CLIENT_TYPE_VC1_DEC		5U
#define DWL_CLIENT_TYPE_MPEG2_DEC	6U
#define DWL_CLIENT_TYPE_VP6_DEC		7U
#define DWL_CLIENT_TYPE_AVS_DEC		8U
#define DWL_CLIENT_TYPE_RV_DEC		9U
#define DWL_CLIENT_TYPE_VP8_DEC		10U
#define DWL_CLIENT_TYPE_VP9_DEC		11U
#define DWL_CLIENT_TYPE_HEVC_DEC	12U

static struct hantrodec_t *get_core_ctrl(u32 id)
{
	struct hantrodec_t *pcore;
	u32 deviceid = DEVICE_ID(id);
	u32 node = KCORE(id);

	PDEBUG("hantrodec: %s\n", __func__);
	pcore = get_dec_node_by_device_id(deviceid, node);
	return pcore;
}

u32 hantrodec_read_bandwidth(struct device_info *pdevinfo, int is_read_bw)
{
	int i, devcnt = get_device_count();
	u32 bandwidth = 0;
	struct hantrodec_t *dev;

	if (!pdevinfo) {
		for (i = 0; i < devcnt; i++) {
			dev = get_dec_node_by_device_id(i, 0);
			while (dev) {
				if (is_read_bw)
					bandwidth +=
						ioread32((dev->hwregs +
								  HANTRO_VC8KD_REG_BWREAD * 4));
				else
					bandwidth +=
						ioread32((dev->hwregs +
								  HANTRO_VC8KD_REG_BWWRITE * 4));

				dev = dev->next;
			}
		}
	} else {
		dev = get_dec_node(pdevinfo, 0);
		while (dev) {
			if (is_read_bw)
				bandwidth +=
					ioread32((dev->hwregs +
							  HANTRO_VC8KD_REG_BWREAD * 4));
			else
				bandwidth +=
					ioread32((dev->hwregs +
							  HANTRO_VC8KD_REG_BWWRITE * 4));

			dev = dev->next;
		}
	}
	return bandwidth * VC8KD_BURSTWIDTH;
}

static void read_core_config(struct hantrodec_t *dev)
{
	int c = dev->core_id;
	u32 reg, tmp, mask;
	struct hantrodec_t *next;

	PDEBUG("hantrodec: %s\n", __func__);
	dev->cfg = 0;

	/* Decoder configuration */
	if (IS_G1(dev->hw_id)) {
		reg = ioread32((dev->hwregs + HANTRODEC_SYNTH_CFG * 4));

		tmp = (reg >> DWL_H264_E) & 0x3U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has H264\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

		tmp = (reg >> DWL_JPEG_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has JPEG\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

		tmp = (reg >> DWL_MPEG4_E) & 0x3U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has MPEG4\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

		tmp = (reg >> DWL_VC1_E) & 0x3U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has VC1\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

		tmp = (reg >> DWL_MPEG2_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has MPEG2\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

		tmp = (reg >> DWL_VP6_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has VP6\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

		reg = ioread32((dev->hwregs +
					HANTRODEC_SYNTH_CFG_2 * 4));

		/* VP7 and WEBP is part of VP8 */
		mask = (1 << DWL_VP8_E) | (1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
		tmp = (reg & mask);
		if (tmp & (1 << DWL_VP8_E))
			PDEBUG("hantrodec: core[%d] has VP8\n", c);

		if (tmp & (1 << DWL_VP7_E))
			PDEBUG("hantrodec: core[%d] has VP7\n", c);

		if (tmp & (1 << DWL_WEBP_E))
			PDEBUG("hantrodec: core[%d] has WebP\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

		tmp = (reg >> DWL_AVS_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has AVS\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

		tmp = (reg >> DWL_RV_E) & 0x03U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has RV\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

		/* Post-processor configuration */
		reg = ioread32((dev->hwregs + HANTROPP_SYNTH_CFG * 4));

		tmp = (reg >> DWL_G1_PP_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has PP\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
	} else if ((IS_G2(dev->hw_id))) {
		reg = ioread32((dev->hwregs + HANTRODEC_CFG_STAT * 4));

		tmp = (reg >> DWL_G2_HEVC_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has HEVC\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

		tmp = (reg >> DWL_G2_VP9_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has VP9\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

		/* Post-processor configuration */
		reg = ioread32((dev->hwregs +
					HANTRODECPP_SYNTH_CFG * 4));

		tmp = (reg >> DWL_G2_PP_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has PP\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
	} else if ((IS_VC8000D(dev->hw_id)) && !dev->its_main_core_id) {
		reg = ioread32((dev->hwregs + HANTRODEC_SYNTH_CFG * 4));

		tmp = (reg >> DWL_H264_E) & 0x3U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has H264\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

		tmp = (reg >> DWL_H264HIGH10_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has H264HIGH10\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

		tmp = (reg >> DWL_JPEG_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has JPEG\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

		tmp = (reg >> DWL_MPEG4_E) & 0x3U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has MPEG4\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

		tmp = (reg >> DWL_VC1_E) & 0x3U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has VC1\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

		tmp = (reg >> DWL_MPEG2_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has MPEG2\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

		tmp = (reg >> DWL_VP6_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has VP6\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

		reg = ioread32((dev->hwregs +
					HANTRODEC_SYNTH_CFG_2 * 4));

		/* VP7 and WEBP is part of VP8 */
		mask = (1 << DWL_VP8_E) | (1 << DWL_VP7_E) | (1 << DWL_WEBP_E);
		tmp = (reg & mask);
		if (tmp & (1 << DWL_VP8_E))
			PDEBUG("hantrodec: core[%d] has VP8\n", c);

		if (tmp & (1 << DWL_VP7_E))
			PDEBUG("hantrodec: core[%d] has VP7\n", c);

		if (tmp & (1 << DWL_WEBP_E))
			PDEBUG("hantrodec: core[%d] has WebP\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

		tmp = (reg >> DWL_AVS_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has AVS\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

		tmp = (reg >> DWL_RV_E) & 0x03U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has RV\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

		reg = ioread32((dev->hwregs +
					HANTRODEC_SYNTH_CFG_3 * 4));

		tmp = (reg >> DWL_HEVC_E) & 0x07U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has HEVC\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

		tmp = (reg >> DWL_VP9_E) & 0x07U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has VP9\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

		/* Post-processor configuration */
		reg = ioread32((dev->hwregs +
					HANTRODECPP_CFG_STAT * 4));

		tmp = (reg >> DWL_PP_E) & 0x01U;
		if (tmp)
			PDEBUG("hantrodec: core[%d] has PP\n", c);

		dev->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;

		if (dev->its_aux_core_id) {
			/* set main_core_id and aux_core_id */
			next = dev->its_aux_core_id;
			reg = ioread32((next->hwregs +
						HANTRODEC_SYNTH_CFG_2 * 4));

			tmp = (reg >> DWL_H264_PIPELINE_E) & 0x01U;
			if (tmp)
				PDEBUG("hantrodec: core[%d] has pipeline H264\n",
				       c);

			next->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_PIPELINE_E) & 0x01U;
			if (tmp)
				PDEBUG("hantrodec: core[%d] has pipeline JPEG\n",
				       c);

			next->cfg |= tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;
		}
	}
	dev->cfg_backup = dev->cfg;
}

static int core_has_format(const u32 cfg, u32 format)
{
	return (cfg & (1 << format)) ? 1 : 0;
}

static int get_dec_core(long core, struct hantrodec_t *dev, struct file *file,
			unsigned long format)
{
	int success = 0;
	unsigned long flags;
	struct device_info *pdevinfo = dev->pdevinfo;

	PDEBUG("hantrodec: %s\n", __func__);
	spin_lock_irqsave(&pdevinfo->owner_lock, flags);
	if (core_has_format(dev->cfg, format) && !dev->dec_owner) {
		dev->dec_owner = file;
		((struct device_info *)(dev->pdevinfo))->dec_irq &=
			~(1 << core);
		success = 1;
		/*
		 * If one main core takes one format which doesn't supported
		 * by aux core, set aux core's cfg to none video format support,
		 * else if aux support, set aux core's cfg only support the format
		 * which main core takes
		 */
		if (dev->its_aux_core_id) {
			if (!core_has_format(dev->its_aux_core_id->cfg, format))
				dev->its_aux_core_id->cfg = 0;
			else
				dev->its_aux_core_id->cfg = (1 << format);
		}

		/*
		 * If one aux core takes one format,
		 * set main core's cfg only support the format which aux core takes
		 */
		else if (dev->its_main_core_id)
			dev->its_main_core_id->cfg = (1 << format);
	}

	spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
	return success;
}

static int get_dec_core_any(long *core, struct hantrodec_t *dev,
			    struct file *file, unsigned long format)
{
	int success = 0;
	long c = 0;

	*core = -1;

	while (dev) {
		/* a free core that has format */
		if (get_dec_core(c, dev, file, format)) {
			success = 1;
			*core = c;
			PDEBUG("get core %ld:%d,fp=%lx, pid=%d", c,
			       dev->core_id, (unsigned long)file,
			       (int)current->pid);
			break;
		}

		c++;
		dev = dev->next;
	}

	return success;
}

static int get_dec_core_id(struct hantrodec_t *dev, struct file *file,
			   unsigned long format)
{
	long c = 0;
	unsigned long flags;
	int core_id = -1;
	struct device_info *pdevinfo = dev->pdevinfo;

	PDEBUG("hantrodec: %s\n", __func__);
	while (dev) {
		/* a core that has format */
		spin_lock_irqsave(&pdevinfo->owner_lock, flags);
		if (core_has_format(dev->cfg_backup, format)) {
			core_id = c;
			spin_unlock_irqrestore(&pdevinfo->owner_lock,
					       flags);
			break;
		}

		spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
		dev = dev->next;
		c++;
	}

	return core_id;
}

static long reserve_decoder(struct hantrodec_t *dev, struct file *file,
			    unsigned long format)
{
	long core = -1;
	struct device_info *pdevinfo = dev->pdevinfo;
	struct hantrodec_t *reserved_core = NULL;

	START_TIME;
	/* reserve a core */
	if (down_interruptible(&pdevinfo->dec_core_sem)) {
		core = -ERESTARTSYS;
		goto out;
	}

	/* lock a core that has specific format*/
	if (wait_event_interruptible(pdevinfo->hw_queue,
				     get_dec_core_any(&core, dev, file,
						      format) != 0)) {
		core = -ERESTARTSYS;
		goto out;
	}

	reserved_core = get_dec_node(pdevinfo, KCORE(core));
	if (!reserved_core) {
		pr_err("Core not found");
		goto out;
	}

	if (reserved_core->dev_clk &&
	    pdevinfo->thermal_data.clk_freq != reserved_core->clk_freq) {
		PDEBUG("Reserve decoder:  setting to %ld for device %d, core %ld\n",
		       pdevinfo->thermal_data.clk_freq,
		       pdevinfo->deviceid, core);
		clk_set_rate(reserved_core->dev_clk,
			     pdevinfo->thermal_data.clk_freq);
		reserved_core->clk_freq = pdevinfo->thermal_data.clk_freq;
	}

	reserved_core->perf_data.last_resv = sched_clock();
out:
	trace_dec_reserve(pdevinfo->deviceid, core, (sched_clock() - start) / 1000);
	return core;
}

static void release_decoder(struct hantrodec_t *dev, long core)
{
	u32 status;
	unsigned long flags;
	struct device_info *pdevinfo = dev->pdevinfo;
	struct hantrodec_t *reserved_core = NULL;

	PDEBUG("hantrodec: %s\n", __func__);
	reserved_core = get_dec_node(pdevinfo, KCORE(core));
	reserved_core->perf_data.count++;
	reserved_core->perf_data.totaltime +=
		(sched_clock() - (reserved_core->perf_data.last_resv == 0 ?
					  sched_clock() :
					  reserved_core->perf_data.last_resv));
	status = ioread32((dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF));
	/* make sure HW is disabled */
	if (status & HANTRODEC_DEC_E) {
		pr_info("hantrodec: DEC[%lx] still enabled -> reset, status = 0x%x [offset=%x]\n",
			core, status, HANTRODEC_IRQ_STAT_DEC_OFF);
		/* abort decoder */
		status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status,
			  (dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF));
	}

	spin_lock_irqsave(&pdevinfo->owner_lock, flags);
	/* If aux core released, revert main core's config back */
	if (dev->its_main_core_id)
		dev->its_main_core_id->cfg = dev->its_main_core_id->cfg_backup;

	/* If main core released, revert aux core's config back */
	if (dev->its_aux_core_id)
		dev->its_aux_core_id->cfg = dev->its_aux_core_id->cfg_backup;

	dev->dec_owner = NULL;
	spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
	up(&pdevinfo->dec_core_sem);
	wake_up_interruptible_all(&pdevinfo->hw_queue);
	trace_dec_release(pdevinfo->deviceid, KCORE(core));
}

static long reserve_post_processor(struct hantrodec_t *dev, struct file *file)
{
	unsigned long flags;
	long core = 0;
	struct device_info *pdevinfo = dev->pdevinfo;

	/* single core PP only */
	if (down_interruptible(&pdevinfo->pp_core_sem))
		return -ERESTARTSYS;

	spin_lock_irqsave(&pdevinfo->owner_lock, flags);
	dev->pp_owner = file;
	spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
	return core;
}

static void release_post_processor(struct hantrodec_t *dev, long core)
{
	unsigned long flags;
	struct device_info *pdevinfo = dev->pdevinfo;

	u32 status = ioread32((dev->hwregs + HANTRO_IRQ_STAT_PP_OFF));

	/* make sure HW is disabled */
	if (status & HANTRO_PP_E) {
		pr_info("hantrodec: PP[%li] still enabled -> reset\n", core);
		iowrite32(0x10, (dev->hwregs + HANTRO_IRQ_STAT_PP_OFF));
	}

	spin_lock_irqsave(&pdevinfo->owner_lock, flags);
	dev->pp_owner = NULL;
	spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
	up(&pdevinfo->pp_core_sem);
}

static long dec_flush_regs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0;

	PDEBUG("hantrodec: %s\n", __func__);
	ret = copy_from_user(dev->dec_regs, core->regs,
			     HANTRO_VC8000D_REGS * 4);
	if (ret) {
		pr_info("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	iowrite32(0x0, (dev->hwregs + 4));
	memcpy(((dev->hwregs + 0x8)), &dev->dec_regs[2],
	       (HANTRO_VC8000D_LAST_REG - 2) * 4);

	/* write the status register, which may start the decoder */
	iowrite32(dev->dec_regs[1], (dev->hwregs + 4));
	return 0;
}

static long dec_refresh_regs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret;

	memcpy(dev->dec_regs, dev->hwregs, HANTRO_VC8000D_LAST_REG * 4);
	ret = copy_to_user(core->regs, dev->dec_regs,
			   HANTRO_VC8000D_LAST_REG * 4);
	dev->perf_data.hwcycles += ioread32(dev->hwregs + (63 * 4));
	if (ret) {
		pr_info("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static long dec_write_regs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0, i;

	PDEBUG("hantrodec: %s\n", __func__);
	i = core->reg_id;
	ret = copy_from_user(dev->dec_regs + core->reg_id,
			     core->regs + core->reg_id, 4);
	if (ret) {
		pr_info("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	iowrite32(dev->dec_regs[i], dev->hwregs + i * 4);
	return 0;
}

static long dec_read_regs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret, i;

	PDEBUG("hantrodec: %s\n", __func__);

	/* read specific registers from hardware */
	i = core->reg_id;
	dev->dec_regs[i] = ioread32(dev->hwregs + i * 4);

	/* put registers to user space */
	ret = copy_to_user(core->regs + core->reg_id,
			   dev->dec_regs + core->reg_id, 4);
	if (ret) {
		pr_info("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static long pp_flush_regs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0;
	u32 i;

	/* copy original dec regs to kernel space */
	ret = copy_from_user(dev->dec_regs + HANTRO_PP_ORG_FIRST_REG,
			     core->regs + HANTRO_PP_ORG_FIRST_REG,
			     HANTRO_PP_ORG_REGS * 4);
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	/* both original and extended regs need to be written */
	for (i = HANTRO_PP_ORG_FIRST_REG + 1; i <= HANTRO_PP_ORG_LAST_REG; i++)
		iowrite32(dev->dec_regs[i], dev->hwregs + i * 4);

	/* write the stat reg, which may start the PP */
	iowrite32(dev->dec_regs[HANTRO_PP_ORG_FIRST_REG],
		  dev->hwregs + HANTRO_PP_ORG_FIRST_REG * 4);
	return 0;
}

static long pp_refresh_regs(struct hantrodec_t *dev, struct core_desc *core)
{
	long i, ret;
	/* user has to know exactly what they are asking for */
	if (core->size != (HANTRO_PP_ORG_REGS * 4))
		return -EFAULT;

	/* read all registers from hardware */
	/* both original and extended regs need to be read */
	for (i = HANTRO_PP_ORG_FIRST_REG; i <= HANTRO_PP_ORG_LAST_REG; i++)
		dev->dec_regs[i] = ioread32(dev->hwregs + i * 4);

	/* put registers to user space*/
	/* put original registers to user space*/
	ret = copy_to_user(core->regs + HANTRO_PP_ORG_FIRST_REG,
			   dev->dec_regs + HANTRO_PP_ORG_FIRST_REG,
			   HANTRO_PP_ORG_REGS * 4);
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int check_pp_irq(struct hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;
	struct device_info *pdevinfo = dev->pdevinfo;
	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&pdevinfo->owner_lock, flags);
	if (pdevinfo->pp_irq & irq_mask) {
		/* reset the wait condition(s) */
		pdevinfo->pp_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
	return rdy;
}

static long wait_pp_ready_and_refresh_regs(struct hantrodec_t *dev,
					   struct core_desc *core)
{
	u32 id = core->id;
	struct device_info *pdevinfo = dev->pdevinfo;

	PDEBUG("wait_event_interruptible PP[%d]\n", id);
	if (wait_event_interruptible(pdevinfo->pp_wait_queue,
				     check_pp_irq(dev, id))) {
		pr_err("PP[%d]  failed to wait_event_interruptible interrupted\n",
		       id);
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);
	/* refresh registers */
	return pp_refresh_regs(dev, core);
}

static int check_core_irq(struct hantrodec_t *dev, const struct file *file,
			  u32 id)
{
	unsigned long flags;
	int rdy = 0, n = 0;
	struct device_info *pdevinfo = dev->pdevinfo;

	while (dev) {
		u32 irq_mask = (1 << n);

		spin_lock_irqsave(&pdevinfo->owner_lock, flags);
		if (pdevinfo->dec_irq & irq_mask) {
			if (id == n) {
				/* we have an IRQ for our client */
				/* reset the wait condition(s) */
				pdevinfo->dec_irq &= ~irq_mask;
				/* signal ready Core no. for our client */
				rdy = 1;
				spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
				break;
			} else if (!dev->dec_owner) {
				/* zombie IRQ */
				pr_info("IRQ on Core[%d], but no owner!!!\n",
					n);
				/* reset the wait condition(s) */
				pdevinfo->dec_irq &= ~irq_mask;
			}
		}

		spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
		n++; /* next Core */
		dev = dev->next;
	}

	return rdy;
}

static long wait_core_ready(struct hantrodec_t *dev, const struct file *file,
			    u32 id)
{
	struct device_info *pdevinfo = dev->pdevinfo;

	PDEBUG("wait_event_interruptible CORE\n");

	if (wait_event_interruptible(pdevinfo->dec_wait_queue,
				     check_core_irq(dev, file, id))) {
		pr_err("CORE  failed to wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);
	return 0;
}

long hantrodec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int i, ret;
	u32 hw_id, id;
	u32 deviceid, node;
	long tmp = 0;
	unsigned long long tmp64;
	struct core_desc core;
	struct hantrodec_t *pcore;

	if (enable_decode == 0)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_CLI): {
		id = arg;
		pcore = get_core_ctrl(id);
		if (!pcore)
			return -EFAULT;

		for (i = 0; i < 4; i++)
			if (pcore->irqlist[i] > 0)
				disable_irq(pcore->irqlist[i]);

		break;
	}
	case _IOC_NR(HANTRODEC_IOC_STI): {
		id = arg;
		pcore = get_core_ctrl(id);
		if (!pcore)
			return -EFAULT;

		for (i = 0; i < 4; i++)
			if (pcore->irqlist[i] > 0)
				enable_irq(pcore->irqlist[i]);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET): {
		__get_user(id, (unsigned long __user *)arg);
		pcore = get_core_ctrl(id);
		if (!pcore)
			return -EFAULT;

		__put_user(pcore->multicorebase_actual,
			   (unsigned long long __user *)arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE): {
		__u32 io_size;

		__get_user(id, (__u32 __user *)arg);
		pcore = get_core_ctrl(id);
		if (!pcore)
			return -EFAULT;

		io_size = pcore->iosize;
		__put_user(io_size, (u32 __user *)arg);

		return 0;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS): {
		__get_user(deviceid, (__u32 __user *)arg);
		pcore = get_dec_node_by_device_id(deviceid, 0);
		if (!pcore)
			return -EFAULT;

		i = 0;
		while (pcore) {
			tmp = copy_to_user(((unsigned long long __user *)arg) + i,
					   &pcore->multicorebase_actual,
					   sizeof(pcore->multicorebase_actual));
			if (tmp) {
				pr_err("copy_to_user failed, returned %li\n",
				       tmp);
				return -EFAULT;
			}

			pcore = pcore->next;
			i++;
		}

		break;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_CORES):
		__get_user(id, (__u32 __user *)arg);
		id = get_device_core_num(id, CORE_DEC);
		PDEBUG("cores=%d\n", id);
		__put_user(id, (u32 __user *)arg);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
		/* get registers from user space */
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_info("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		dec_flush_regs(pcore, &core);
		break;
	}

	case _IOC_NR(HANTRODEC_IOCS_DEC_WRITE_REG): {
		/* get registers from user space */
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_info("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		dec_write_regs(pcore, &core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG): {
		/* get registers from user space */
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		pp_flush_regs(pcore, &core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		return dec_refresh_regs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_READ_REG): {
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_info("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		return dec_read_regs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG): {
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		return pp_refresh_regs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
		__get_user(tmp64, (unsigned long long __user *)arg);
		deviceid = tmp64 >> 32;
		PDEBUG("Reserve DEC core, format = %i\n", (u32)tmp64);
		pcore = get_core_ctrl(deviceid << 16);
		if (!pcore)
			return -EFAULT;

		ret = reserve_decoder(pcore, file, tmp64 & 0xffffffff);
		if (ret < 0)
			return -EFAULT;

		__put_user((ret | (deviceid << 16)), (unsigned long long __user *)arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
		pcore = get_core_ctrl((u32)arg);
		if (!pcore)
			return -EFAULT;

		if (pcore->dec_owner != file) {
			pr_err("bogus DEC release, Core = %li\n", arg);
			return -EFAULT;
		}

		PDEBUG("Release DEC, core = %li\n", arg);
		release_decoder(pcore, arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
		id = (u32)arg;
		pcore = get_dec_node_by_device_id(DEVICE_ID(id), 0);
		if (!pcore)
			return -EFAULT;

		return reserve_post_processor(pcore, file);
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE): {
		pcore = get_core_ctrl(arg);
		if (!pcore)
			return -EFAULT;

		if (arg != 0 || pcore->pp_owner != file) {
			pr_err("bogus PP release %li\n", arg);
			return -EFAULT;
		}

		release_post_processor(pcore, arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT): {
		tmp = copy_from_user(&core, (void const __user *)arg,
				     sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		pcore = get_core_ctrl(core.id);
		if (!pcore)
			return -EFAULT;

		return wait_pp_ready_and_refresh_regs(pcore, &core);
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT): {
		id = (u32)arg;
		deviceid = DEVICE_ID(id);
		node = KCORE(id);
		pcore = get_dec_node_by_device_id(deviceid, 0);
		if (!pcore)
			return -EFAULT;

		tmp = wait_core_ready(pcore, file, node);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID):
		__get_user(id, (__u32 __user *)arg);
		pcore = get_core_ctrl(id);
		if (!pcore)
			return 0;

		id = ioread32(pcore->hwregs);
		__put_user(id, (u32 __user *)arg);
		break;
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
		PDEBUG("Get DEC Core_id, format = %li\n", arg);
		__get_user(tmp64, (unsigned long long __user *)arg);
		deviceid = tmp64 >> 32;
		pcore = get_dec_node_by_device_id(deviceid, 0);
		if (!pcore)
			return -EFAULT;

		tmp = get_dec_core_id(pcore, file, tmp64 & 0xffffffff);
		__put_user(tmp, (unsigned long long __user *)arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_BUILD_ID): {
		__get_user(id, (int __user *)arg);
		pcore = get_core_ctrl(id);
		if (!pcore)
			return -EFAULT;

		hw_id = ioread32((pcore->hwregs));
		if (IS_G1(hw_id >> 16) || IS_G2(hw_id >> 16)) {
			__put_user(hw_id, (u32 __user *)arg);
		} else {
			hw_id = ioread32((pcore->hwregs +
						  HANTRODEC_HW_BUILD_ID_OFF));
			__put_user(hw_id, (u32 __user *)arg);
		}

		return 0;
	}
	case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
		struct device_info *pdevinfo;

		PDEBUG("hantrodec: IRQs received/sent2user = %d / %d\n",
		       atomic_read(&irq_rx), atomic_read(&irq_tx));
		deviceid = get_device_count();
		for (i = 0; i < (int)deviceid; i++) {
			pcore = get_dec_node_by_device_id(i, 0);
			if (!pcore)
				continue;

			pdevinfo = pcore->pdevinfo;
			PDEBUG("hantrodec: device %d dec_irq     = 0x%08x\n", i,
			       pdevinfo->dec_irq);
			PDEBUG("hantrodec: device %d pp_irq      = 0x%08x\n", i,
			       pdevinfo->pp_irq);
			id = 0;
			while (pcore) {
				PDEBUG("hantrodec: device %d dec_core[%i] %s\n",
				       i, id,
				       !pcore->dec_owner ? "FREE" : "RESERVED");
				PDEBUG("hantrodec: device %d pp_core[%i]  %s\n",
				       i, id,
				       !pcore->pp_owner ? "FREE" : "RESERVED");
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

int hantrodec_release(struct file *file)
{
	int n, i, devicecnt = get_device_count();
	struct hantrodec_t *pcore;

	if (enable_decode == 0)
		return 0;

	PDEBUG("hantrodec: %s\n", __func__);
	for (i = 0; i < devicecnt; i++) {
		pcore = get_dec_node_by_device_id(i, 0);
		n = 0;
		while (pcore) {
			if (pcore->dec_owner == file) {
				PDEBUG("releasing device %d dec Core %i lock\n",
				       i, n);
				release_decoder(pcore, n);
			}

			n++;
			pcore = pcore->next;
		}

		pcore = get_dec_node_by_device_id(i, 0);
		if (pcore && pcore->pp_owner == file) {
			PDEBUG("releasing device %d pp Core %i lock\n", i, 0);
			release_post_processor(pcore, n);
		}
	}

	return 0;
}

int hantrodec_open(struct inode *inode, struct file *file)
{
	if (enable_decode == 0)
		return 0;

	return 0;
}

static void setup_dec_lut(void)
{
	u8 __iomem *dec_page_lut_regs;

	if (hantro_drm.dec_page_lut_regs) {
		pr_info("hantrodec: page_lut already reserved\n");
		return;
	}

	/* Register and set the page lookup table for read */
	if (!request_mem_region(KMB_VC8000D_PAGE_LUT, 0x100,
				"hantrodec_pagelut_read")) {
		pr_err("hantrodec: failed to reserve page lookup table registers\n");
		return;
	}

	dec_page_lut_regs = ioremap(KMB_VC8000D_PAGE_LUT, 0x100);
	if (!dec_page_lut_regs) {
		pr_err("hantrodec: failed to ioremap page lookup table registers\n");
		return;
	}

	/* Set VDEC RD Page LUT AXI ID 0-15 to 0x4 */
	iowrite32(0x04040404, dec_page_lut_regs);
	pr_info("hantrodec: RD AXI ID 3:0 = %x\n",
		ioread32(dec_page_lut_regs));
	iowrite32(0x04040404, dec_page_lut_regs + 0x4);
	pr_info("hantrodec: RD AXI ID 7:4 = %x\n",
		ioread32(dec_page_lut_regs + 0x4));
	iowrite32(0x04040404, dec_page_lut_regs + 0x8);
	pr_info("hantrodec: RD AXI ID 11:8 = %x\n",
		ioread32(dec_page_lut_regs + 0x8));
	iowrite32(0x04040404, dec_page_lut_regs + 0xc);
	pr_info("hantrodec: RD AXI ID 15:12 = %x\n",
		ioread32(dec_page_lut_regs + 0xc));

#ifdef STATIC_AXI_WR
	iowrite32(0x04, dec_page_lut_regs + 0x10);
	pr_info("hantrodec: WR AXI ID 0 = %x\n",
		ioread32(dec_page_lut_regs + 0x10));
#else /* dynamic WR AXI ID */
	/* Set WR Page LUT AXI ID 0-3, 6-15 to 0x4 and WR Page LUT AXI ID 4,5 to 0x0 */
	iowrite32(0x04040400, dec_page_lut_regs + 0x10);
	pr_info("hantrodec: page_lut_regs WR AXI ID 3:0= %x\n",
		ioread32(dec_page_lut_regs + 0x10));
	iowrite32(0x04040000, dec_page_lut_regs + 0x14);
	pr_info("hantrodec: page_lut_regs WR AXI ID 7:4= %x\n",
		ioread32(dec_page_lut_regs + 0x14));
	iowrite32(0x04040404, dec_page_lut_regs + 0x18);
	pr_info("hantrodec: page_lut_regs WR AXI ID 11:8= %x\n",
		ioread32(dec_page_lut_regs + 0x18));
	iowrite32(0x04040404, dec_page_lut_regs + 0x1c);
	pr_info("hantrodec: page_lut_regs WR AXI ID 15:12= %x\n",
		ioread32(dec_page_lut_regs + 0x1c));
#endif
	pr_info("hantrodec: page_lut reserved\n");

	hantro_drm.dec_page_lut_regs = dec_page_lut_regs;
}

int hantrodec_init(void)
{
	if (hantro_drm.device_type == DEVICE_KEEMBAY && enable_lut)
		setup_dec_lut();

	return 0;
}

int hantrodec_cleanup(void)
{
	if (hantro_drm.dec_page_lut_regs) {
		iounmap(hantro_drm.dec_page_lut_regs);
		hantro_drm.dec_page_lut_regs = NULL;
		release_mem_region(KMB_VC8000D_PAGE_LUT, 0x100);
	}

	return 0;
}

static void disable_dec_clock(struct hantrodec_t *pcore)
{
	if (!pcore->dev_clk)
		return;

	clk_disable_unprepare(pcore->dev_clk);
	clk_put(pcore->dev_clk);
	pcore->dev_clk = NULL;
}

static int init_dec_clock(struct hantrodec_t *pcore, struct dtbnode *pnode)
{
	if (strlen(pnode->clock_name) == 0)
		return 0;

	pcore->dev_clk = clk_get(pnode->pdevinfo->dev, pnode->clock_name);
	if (IS_ERR(pcore->dev_clk) || !pcore->dev_clk) {
		pr_err("%s: clock %s not found. err = %ld", __func__,
		       pnode->clock_name, PTR_ERR(pcore->dev_clk));
		pcore->dev_clk = NULL;
		return -EINVAL;
	}

	clk_prepare_enable(pcore->dev_clk);
	pcore->clk_freq = pnode->pdevinfo->thermal_data.clk_freq;
	clk_set_rate(pcore->dev_clk, pcore->clk_freq);
	return 0;
}

int hantrodec_probe(struct dtbnode *pnode)
{
	int i, result = 0;
	struct hantrodec_t *pcore, *auxcore;
	int irqn;

	if (enable_decode == 0)
		return 0;

	pcore = vmalloc(sizeof(*pcore));
	if (!pcore)
		return -ENOMEM;

	memset(pcore, 0, sizeof(struct hantrodec_t));
	pcore->multicorebase = pnode->ioaddr;
	pcore->multicorebase_actual = pnode->ioaddr;
	pcore->iosize = pnode->iosize;
	auxcore = NULL;
	result = reserve_io(pcore, &auxcore);
	if (result < 0) {
		vfree(pcore);
		return -ENODEV;
	}

	read_core_config(pcore);
	reset_asic(pcore);
	pcore->dec_owner = NULL;
	pcore->pp_owner = NULL;

	if (auxcore) {
		read_core_config(auxcore);
		reset_asic(auxcore);
		auxcore->dec_owner = NULL;
		auxcore->pp_owner = NULL;
	}

	irqn = 0;
	for (i = 0; i < 4; i++)
		pcore->irqlist[i] = -1;
	if (enable_irqmode == 1) {
		for (i = 0; i < 4; i++) {
			if (pnode->irq[i] > 0) {
				strcpy(pcore->irq_name[i], pnode->irq_name[i]);
				result = request_irq(pnode->irq[i],
						     hantrodec_isr, IRQF_SHARED,
						     pcore->irq_name[i],
						     (void *)pcore);
				if (result != 0) {
					pr_err("dec can't reserve irq %d\n",
					       pnode->irq[i]);
					release_io(pcore);
					vfree(pcore);
					if (auxcore) {
						release_io(auxcore);
						vfree(auxcore);
					}

					return -ENODEV;
				}

				pcore->irqlist[irqn] = pnode->irq[i];
				irqn++;
			}
		}
	}

	init_dec_clock(pcore, pnode);
	add_dec_node(pnode->pdevinfo, pcore);
	if (auxcore)
		add_dec_node(pnode->pdevinfo, auxcore);

	return 0;
}

void hantrodec_remove(struct device_info *pdevinfo)
{
	struct hantrodec_t *pcore, *next;
	int i;

	/* free the IRQ */
	pcore = get_dec_node(pdevinfo, 0);
	while (pcore) {
		/* reset hardware */
		disable_dec_clock(pcore);
		reset_asic(pcore);
		for (i = 0; i < 4; i++)
			if (pcore->irqlist[i] > 0)
				free_irq(pcore->irqlist[i], (void *)pcore);

		release_io(pcore);
		next = pcore->next;
		vfree(pcore);
		pcore = next;
	}
}

static int check_hw_id(struct hantrodec_t *dev)
{
	long hwid;
	size_t num_hw = sizeof(dec_hwid) / sizeof(*dec_hwid);
	int found = 0;

	hwid = readl(dev->hwregs);
	hwid = (hwid >> 16) & 0xFFFF;

	while (num_hw--) {
		if (hwid == dec_hwid[num_hw]) {
			pr_info("hantrodec: HW at base <0x%llx> with ID 0x%lx\n",
				dev->multicorebase_actual, hwid);
			found = 1;
			dev->hw_id = hwid;
			break;
		}
	}

	if (!found) {
		pr_info("hantrodec: HW at base <0x%llx> with ID 0x%lx\n",
			dev->multicorebase_actual, hwid);
		pr_info("hantrodec: Unknown HW found at 0x%llx\n",
			dev->multicorebase_actual);
		return 0;
	}
	return 1;
}

static int reserve_io(struct hantrodec_t *core, struct hantrodec_t **auxcore)
{
	int result;
	long hwid;
	u32 reg;
	struct hantrodec_t *auxcore_tmp = NULL;

	PDEBUG("hantrodec: %s\n", __func__);
	if (!request_mem_region(core->multicorebase_actual, core->iosize,
				core->reg_name)) {
		pr_info("hantrodec: failed to reserve HW regs %llx, %x\n",
			core->multicorebase_actual, core->iosize);
		return -EBUSY;
	}

	core->hwregs = ioremap(core->multicorebase_actual, core->iosize);
	if (!core->hwregs) {
		pr_info("hantrodec: failed to ioremap HW regs\n");
		release_mem_region(core->multicorebase_actual, core->iosize);
		return -EBUSY;
	}

	core->its_main_core_id = NULL;
	core->its_aux_core_id = NULL;
	/* check for correct HW */
	result = check_hw_id(core);
	if (!result) {
		result = -ENXIO;
		goto error;
	}

	hwid = ((readl(core->hwregs)) >> 16) & 0xFFFF;
	if (IS_VC8000D(hwid)) {
		reg = readl(core->hwregs + HANTRODEC_SYNTH_CFG_2_OFF);
		if (((reg >> DWL_H264_PIPELINE_E) & 0x01U) ||
		    ((reg >> DWL_JPEG_PIPELINE_E) & 0x01U)) {
			auxcore_tmp = vmalloc(sizeof(*auxcore_tmp));
			if (!auxcore_tmp) {
				result = -ENOMEM;
				goto error;
			}

			memset(auxcore_tmp, 0, sizeof(struct hantrodec_t));
			auxcore_tmp->multicorebase_actual =
				core->multicorebase_actual + 0x800;
			auxcore_tmp->multicorebase =
				auxcore_tmp->multicorebase_actual;
			auxcore_tmp->iosize = core->iosize;

			if (!request_mem_region(auxcore_tmp->multicorebase_actual,
						auxcore_tmp->iosize, "hantrodec0")) {
				pr_info("hantrodec: failed to reserve HW regs\n");
				result = -EBUSY;
				goto free_auxcore;
			}

			auxcore_tmp->hwregs =
				ioremap(auxcore_tmp->multicorebase_actual,
					auxcore_tmp->iosize);
			if (!auxcore_tmp->hwregs) {
				pr_info("hantrodec: failed to ioremap HW regs\n");
				release_mem_region(auxcore_tmp->multicorebase_actual,
						   auxcore_tmp->iosize);
				result = -EBUSY;
				goto free_auxcore;
			}

			core->its_aux_core_id = auxcore_tmp;
			auxcore_tmp->its_main_core_id = core;
			auxcore_tmp->its_aux_core_id = NULL;
		}
	}

	if (auxcore_tmp) {
		result = check_hw_id(auxcore_tmp);
		if (!result) {
			result = -ENXIO;
			goto release_auxcore;
		}
	}

	*auxcore = auxcore_tmp;
	return 0;

release_auxcore:
	release_io(auxcore_tmp);
free_auxcore:
	vfree(auxcore_tmp);
	auxcore_tmp = NULL;
error:
	release_io(core);
	return result;
}

static void release_io(struct hantrodec_t *dev)
{
	PDEBUG("hantrodec: %s\n", __func__);
	if (dev->hwregs)
		iounmap(dev->hwregs);

	release_mem_region(dev->multicorebase_actual, dev->iosize);
}

static irqreturn_t hantrodec_isr(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int handled = 0;
	int i = 0;
	struct hantrodec_t *dev = (struct hantrodec_t *)dev_id;
	u32 irq_status_dec;
	struct device_info *pdevinfo = dev->pdevinfo;

	dev = get_first_dec_nodes(pdevinfo);
	spin_lock_irqsave(&pdevinfo->owner_lock, flags);
	while (dev) {
		u8 __iomem *hwregs = dev->hwregs;

		/* interrupt status register read */
		irq_status_dec =
			ioread32(hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);
		if (irq_status_dec & HANTRODEC_DEC_IRQ) {
			/* clear dec IRQ */
			irq_status_dec &= (~HANTRODEC_DEC_IRQ);
			iowrite32(irq_status_dec,
				  hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);

			PDEBUG("decoder IRQ received! Core %d\n", i);

			atomic_inc(&irq_rx);

			pdevinfo->dec_irq |= (1 << i);

			wake_up_interruptible_all(&pdevinfo->dec_wait_queue);
			handled++;
		}

		i++;
		dev = dev->next;
	}

	spin_unlock_irqrestore(&pdevinfo->owner_lock, flags);
	if (!handled)
		PDEBUG("IRQ received, but not hantrodec's!\n");

	return IRQ_RETVAL(handled);
}

static void reset_asic(struct hantrodec_t *dev)
{
	int i;
	u32 status;
	int size = MIN(DEC_IO_SIZE_MAX, dev->iosize);

	PDEBUG("hantrodec: %s\n", __func__);
	status = ioread32(dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);
	if (status & HANTRODEC_DEC_E) {
		pr_info("hantrodec: %s abort with IRQ disabled\n", __func__);
		/* abort with IRQ disabled */
		status = HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status,
			  dev->hwregs + HANTRODEC_IRQ_STAT_DEC_OFF);
	}

	if (IS_G1(dev->hw_id))
		/* reset PP */
		iowrite32(0, dev->hwregs + HANTRO_IRQ_STAT_PP_OFF);

	for (i = 4; i < size; i += 4)
		iowrite32(0, dev->hwregs + i);
}
