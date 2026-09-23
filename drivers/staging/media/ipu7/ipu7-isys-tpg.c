// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2025 Intel Corporation
 */

#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/module.h>

#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#include "ipu7.h"
#include "ipu7-bus.h"
#include "ipu7-buttress-regs.h"
#include "ipu7-isys.h"
#include "ipu7-isys-subdev.h"
#include "ipu7-isys-tpg.h"
#include "ipu7-isys-video.h"
#include "ipu7-isys-csi2-regs.h"
#include "ipu7-platform-regs.h"

static const u32 tpg_supported_codes[] = {
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	0,
};

#define IPU_ISYS_FREQ		533000000UL

static u32 isys_mbus_code_to_bpp(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		return 24;
	case MEDIA_BUS_FMT_YUYV10_1X20:
		return 20;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_RGB565_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
		return 16;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 12;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	default:
		WARN_ON(1);
		return 0;
	}
}

static const struct v4l2_subdev_video_ops tpg_sd_video_ops = {
	.s_stream = tpg_set_stream,
};

static int ipu7_isys_tpg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ipu7_isys_tpg *tpg = container_of(container_of(ctrl->handler,
							      struct
							      ipu7_isys_subdev,
							      ctrl_handler),
						 struct ipu7_isys_tpg, asd);
	switch (ctrl->id) {
	case V4L2_CID_HBLANK:
		writel(ctrl->val, tpg->base + MGC_MG_HBLANK);
		break;
	case V4L2_CID_VBLANK:
		writel(ctrl->val, tpg->base + MGC_MG_VBLANK);
		break;
	case V4L2_CID_TEST_PATTERN:
		writel(ctrl->val, tpg->base + MGC_MG_TPG_MODE);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ipu7_isys_tpg_ctrl_ops = {
	.s_ctrl = ipu7_isys_tpg_s_ctrl,
};

static u64 ipu7_isys_tpg_rate(struct ipu7_isys_tpg *tpg, unsigned int bpp)
{
	return MGC_PPC * IPU_ISYS_FREQ / bpp;
}

static const char *const tpg_mode_items[] = {
	"Ramp",
	"Checkerboard",
	"Monochrome per frame",
	"Color palette",
};

static struct v4l2_ctrl_config tpg_mode = {
	.ops = &ipu7_isys_tpg_ctrl_ops,
	.id = V4L2_CID_TEST_PATTERN,
	.name = "Test Pattern",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = TPG_MODE_RAMP,
	.max = ARRAY_SIZE(tpg_mode_items) - 1,
	.def = TPG_MODE_COLOR_PALETTE,
	.menu_skip_mask = 0x2,
	.qmenu = tpg_mode_items,
};

static void ipu7_isys_tpg_init_controls(struct v4l2_subdev *sd)
{
	struct ipu7_isys_tpg *tpg = to_ipu7_isys_tpg(sd);
	int hblank;
	u64 default_pixel_rate;

	hblank = 1024;

	tpg->hblank = v4l2_ctrl_new_std(&tpg->asd.ctrl_handler,
					&ipu7_isys_tpg_ctrl_ops,
					V4L2_CID_HBLANK, 8, 65535, 1, hblank);

	tpg->vblank = v4l2_ctrl_new_std(&tpg->asd.ctrl_handler,
					&ipu7_isys_tpg_ctrl_ops,
					V4L2_CID_VBLANK, 8, 65535, 1, 1024);

	default_pixel_rate = ipu7_isys_tpg_rate(tpg, 8);
	tpg->pixel_rate = v4l2_ctrl_new_std(&tpg->asd.ctrl_handler,
					    &ipu7_isys_tpg_ctrl_ops,
					    V4L2_CID_PIXEL_RATE,
					    default_pixel_rate,
					    default_pixel_rate,
					    1, default_pixel_rate);
	if (tpg->pixel_rate) {
		tpg->pixel_rate->cur.val = default_pixel_rate;
		tpg->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	v4l2_ctrl_new_custom(&tpg->asd.ctrl_handler, &tpg_mode, NULL);
}

static int tpg_sd_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
		.source_pad = 0,
		.source_stream = 0,
		.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		}
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = 1,
		.routes = routes,
	};

	static const struct v4l2_mbus_framefmt format = {
		.width = 1920,
		.height = 1080,
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.field = V4L2_FIELD_NONE,
	};

	return v4l2_subdev_set_routing_with_fmt(sd, state, &routing, &format);
}

static const struct v4l2_subdev_internal_ops ipu7_isys_tpg_internal_ops = {
	.init_state = tpg_sd_init_cfg,
};

static const struct v4l2_subdev_pad_ops tpg_sd_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ipu7_isys_subdev_set_fmt,
	.enum_mbus_code = ipu7_isys_subdev_enum_mbus_code,
};

static int subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
			   struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 10, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
};

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops tpg_sd_core_ops = {
	.subscribe_event = subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops tpg_sd_ops = {
	.core = &tpg_sd_core_ops,
	.video = &tpg_sd_video_ops,
	.pad = &tpg_sd_pad_ops,
};

static struct media_entity_operations tpg_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

void ipu7_isys_tpg_sof_event_by_stream(struct ipu7_isys_stream *stream)
{
	struct ipu7_isys_tpg *tpg = ipu7_isys_subdev_to_tpg(stream->asd);
	struct video_device *vdev = tpg->asd.sd.devnode;
	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
	};

	ev.u.frame_sync.frame_sequence = atomic_fetch_inc(&stream->sequence);

	v4l2_event_queue(vdev, &ev);

	dev_dbg(&stream->isys->adev->auxdev.dev,
		"sof_event::tpg-%i sequence: %i\n", tpg->index,
		ev.u.frame_sync.frame_sequence);
}

void ipu7_isys_tpg_eof_event_by_stream(struct ipu7_isys_stream *stream)
{
	struct ipu7_isys_tpg *tpg = ipu7_isys_subdev_to_tpg(stream->asd);
	u32 frame_sequence = atomic_read(&stream->sequence);

	dev_dbg(&stream->isys->adev->auxdev.dev,
		"eof_event::tpg-%i sequence: %i\n",
		tpg->index, frame_sequence);
}

#define DEFAULT_VC_ID 0
static bool is_metadata_enabled(const struct ipu7_isys_tpg *tpg)
{
	return false;
}

static void ipu7_mipigen_regdump(const struct ipu7_isys_tpg *tpg,
				 void __iomem *mg_base)
{
	struct device *dev = &tpg->isys->adev->auxdev.dev;

	dev_dbg(dev, "---------MGC REG DUMP START----------");

	dev_dbg(dev, "MGC RX_TYPE_REG 0x%x = 0x%x",
		MGC_MG_CSI_ADAPT_LAYER_TYPE,
		readl(mg_base + MGC_MG_CSI_ADAPT_LAYER_TYPE));
	dev_dbg(dev, "MGC MG_MODE_REG 0x%x = 0x%x",
		MGC_MG_MODE, readl(mg_base + MGC_MG_MODE));
	dev_dbg(dev, "MGC MIPI_VC_REG 0x%x = 0x%x",
		MGC_MG_MIPI_VC, readl(mg_base + MGC_MG_MIPI_VC));
	dev_dbg(dev, "MGC MIPI_DTYPES_REG 0x%x = 0x%x",
		MGC_MG_MIPI_DTYPES, readl(mg_base + MGC_MG_MIPI_DTYPES));
	dev_dbg(dev, "MGC MULTI_DTYPES_REG 0x%x = 0x%x",
		MGC_MG_MULTI_DTYPES_MODE,
		readl(mg_base + MGC_MG_MULTI_DTYPES_MODE));
	dev_dbg(dev, "MGC NOF_FRAMES_REG 0x%x = 0x%x",
		MGC_MG_NOF_FRAMES, readl(mg_base + MGC_MG_NOF_FRAMES));
	dev_dbg(dev, "MGC FRAME_DIM_REG 0x%x = 0x%x",
		MGC_MG_FRAME_DIM, readl(mg_base + MGC_MG_FRAME_DIM));
	dev_dbg(dev, "MGC HBLANK_REG 0x%x = 0x%x",
		MGC_MG_HBLANK, readl(mg_base + MGC_MG_HBLANK));
	dev_dbg(dev, "MGC VBLANK_REG 0x%x = 0x%x",
		MGC_MG_VBLANK, readl(mg_base + MGC_MG_VBLANK));
	dev_dbg(dev, "MGC TPG_MODE_REG 0x%x = 0x%x",
		MGC_MG_TPG_MODE, readl(mg_base + MGC_MG_TPG_MODE));
	dev_dbg(dev, "MGC R0=0x%x G0=0x%x B0=0x%x",
		readl(mg_base + MGC_MG_TPG_R0),
		readl(mg_base + MGC_MG_TPG_G0),
		readl(mg_base + MGC_MG_TPG_B0));
	dev_dbg(dev, "MGC R1=0x%x G1=0x%x B1=0x%x",
		readl(mg_base + MGC_MG_TPG_R1),
		readl(mg_base + MGC_MG_TPG_G1),
		readl(mg_base + MGC_MG_TPG_B1));
	dev_dbg(dev, "MGC TPG_MASKS_REG 0x%x = 0x%x",
		MGC_MG_TPG_MASKS, readl(mg_base + MGC_MG_TPG_MASKS));
	dev_dbg(dev, "MGC TPG_XY_MASK_REG 0x%x = 0x%x",
		MGC_MG_TPG_XY_MASK, readl(mg_base + MGC_MG_TPG_XY_MASK));
	dev_dbg(dev, "MGC TPG_TILE_DIM_REG 0x%x = 0x%x",
		MGC_MG_TPG_TILE_DIM, readl(mg_base + MGC_MG_TPG_TILE_DIM));
	dev_dbg(dev, "MGC DTO_SPEED_CTRL_EN_REG 0x%x = 0x%x",
		MGC_MG_DTO_SPEED_CTRL_EN,
		readl(mg_base + MGC_MG_DTO_SPEED_CTRL_EN));
	dev_dbg(dev, "MGC DTO_SPEED_CTRL_INCR_VAL_REG 0x%x = 0x%x",
		MGC_MG_DTO_SPEED_CTRL_INCR_VAL,
		readl(mg_base + MGC_MG_DTO_SPEED_CTRL_INCR_VAL));
	dev_dbg(dev, "MGC MG_FRAME_NUM_STTS 0x%x = 0x%x",
		MGC_MG_FRAME_NUM_STTS,
		readl(mg_base + MGC_MG_FRAME_NUM_STTS));

	dev_dbg(dev, "---------MGC REG DUMP END----------");
}

#define TPG_STOP_TIMEOUT 500000
static int tpg_stop_stream(const struct ipu7_isys_tpg *tpg)
{
	struct device *dev = &tpg->isys->adev->auxdev.dev;
	int ret;
	unsigned int port;
	u32 status;
	void __iomem *reg;
	void __iomem *mgc_base = tpg->isys->pdata->base + IS_IO_MGC_BASE;
	void __iomem *mg_base = tpg->base;

	port = 1 << tpg->index;

	dev_dbg(dev, "MG%d generated %u frames", tpg->index,
		readl(mgc_base + MGC_MG_FRAME_NUM_STTS));
	writel(port, mgc_base + MGC_ASYNC_STOP);

	dev_dbg(dev, "wait for MG%d stop", tpg->index);

	reg = mg_base + MGC_MG_STOPPED_STTS;
	ret = readl_poll_timeout(reg, status, status & 0x1, 200,
				 TPG_STOP_TIMEOUT);
	if (ret < 0) {
		dev_err(dev, "mgc stop timeout");
		return ret;
	}

	dev_dbg(dev, "MG%d STOPPED", tpg->index);

	return 0;
}

#define IS_IO_CLK	(IPU7_IS_FREQ_CTL_DEFAULT_RATIO * 100 / 6)
#define TPG_FRAME_RATE	30
#define TPG_BLANK_RATIO	(4 / 3)
static void tpg_get_timing(const struct ipu7_isys_tpg *tpg, u32 *dto,
			   u32 *hblank_cycles, u32 *vblank_cycles)
{
	struct v4l2_mbus_framefmt format;
	u32 width, height;
	u32 code;
	u32 bpp;
	u32 bits_per_line;
	u64 line_time_ns, frame_time_us, cycles, ns_per_cycle, rate;
	u64 vblank_us, hblank_us;
	u32 ref_clk;
	struct device *dev = &tpg->isys->adev->auxdev.dev;
	u32 dto_incr_val = 0x100;
	int ret;

	ret = ipu7_isys_get_stream_pad_fmt((struct v4l2_subdev *)&tpg->asd.sd,
					   0, 0, &format);
	if (ret)
		return;

	width = format.width;
	height = format.height;
	code = format.code;

	bpp = isys_mbus_code_to_bpp(code);
	if (!bpp)
		return;

	dev_dbg(dev, "MG%d code = 0x%x bpp = %u\n", tpg->index, code, bpp);
	bits_per_line = width * bpp * TPG_BLANK_RATIO;

	cycles = div_u64(bits_per_line, 64);
	dev_dbg(dev, "MG%d bits_per_line = %u cycles = %llu\n", tpg->index,
		bits_per_line, cycles);

	do {
		dev_dbg(dev, "MG%d try dto_incr_val 0x%x\n", tpg->index,
			dto_incr_val);
		rate = div_u64(1 << 16, dto_incr_val);
		ns_per_cycle = div_u64(rate * 1000, IS_IO_CLK);
		dev_dbg(dev, "MG%d ns_per_cycles = %llu\n", tpg->index,
			ns_per_cycle);

		line_time_ns = cycles * ns_per_cycle;
		frame_time_us = line_time_ns * height / 1000;
		dev_dbg(dev, "MG%d line_time_ns = %llu frame_time_us = %llu\n",
			tpg->index, line_time_ns, frame_time_us);

		if (frame_time_us * TPG_FRAME_RATE < USEC_PER_SEC)
			break;

		/* dto incr val step 0x100 */
		dto_incr_val += 0x100;
	} while (dto_incr_val < (1 << 16));

	if (dto_incr_val >= (1 << 16)) {
		dev_warn(dev, "No DTO_INCR_VAL found\n");
		hblank_us = 10; /* 10us */
		vblank_us = 10000; /* 10ms */
		dto_incr_val = 0x1000;
	} else {
		hblank_us = line_time_ns * (TPG_BLANK_RATIO - 1) / 1000;
		vblank_us = div_u64(1000000, TPG_FRAME_RATE) - frame_time_us;
	}

	dev_dbg(dev, "hblank_us = %llu, vblank_us = %llu dto_incr_val = %u\n",
		hblank_us, vblank_us, dto_incr_val);

	ref_clk = tpg->isys->adev->isp->buttress.ref_clk;

	*dto = dto_incr_val;
	*hblank_cycles = hblank_us * ref_clk / 10;
	*vblank_cycles = vblank_us * ref_clk / 10;
	dev_dbg(dev, "hblank_cycles = %u, vblank_cycles = %u\n",
		*hblank_cycles, *vblank_cycles);
}

static int tpg_start_stream(const struct ipu7_isys_tpg *tpg)
{
	struct v4l2_mbus_framefmt format;
	u32 port_map;
	u32 csi_port;
	u32 code, bpp;
	u32 width, height;
	u32 dto, hblank, vblank;
	struct device *dev = &tpg->isys->adev->auxdev.dev;
	void __iomem *mgc_base = tpg->isys->pdata->base + IS_IO_MGC_BASE;
	void __iomem *mg_base = tpg->base;
	int ret;

	ret = ipu7_isys_get_stream_pad_fmt((struct v4l2_subdev *)&tpg->asd.sd,
					   0, 0, &format);
	if (ret)
		return ret;

	width = format.width;
	height = format.height;
	code = format.code;
	dev_dbg(dev, "MG%d code: 0x%x resolution: %ux%u\n",
		tpg->index, code, width, height);
	bpp = isys_mbus_code_to_bpp(code);
	if (!bpp)
		return -EINVAL;

	csi_port = tpg->index;
	if (csi_port >= 4)
		dev_err(dev, "invalid tpg index %u\n", tpg->index);

	dev_dbg(dev, "INSYS MG%d was mapped to CSI%d\n",
		DEFAULT_VC_ID, csi_port);

	/* config port map
	 * TODO: add VC support and TPG with multiple
	 * source pads. Currently, for simplicity, only map 1 mg to 1 csi port
	 */
	port_map = 1 << tpg->index;
	writel(port_map, mgc_base + MGC_CSI_PORT_MAP(csi_port));

	/* configure adapt layer type */
	writel(1, mg_base + MGC_MG_CSI_ADAPT_LAYER_TYPE);

	/* configure MGC mode
	 * 0 - Disable MGC
	 * 1 - Enable PRBS
	 * 2 - Enable TPG
	 * 3 - Reserved [Write phase: SW/FW debug]
	 */
	writel(2, mg_base + MGC_MG_MODE);

	/* config mg init counter */
	writel(0, mg_base + MGC_MG_INIT_COUNTER);

	/*
	 * configure virtual channel
	 * TODO: VC support if need
	 * currently each MGC just uses 1 virtual channel
	 */
	writel(DEFAULT_VC_ID, mg_base + MGC_MG_MIPI_VC);

	/*
	 * configure data type and multi dtypes mode
	 * TODO: it needs to add the metedata flow.
	 */
	if (is_metadata_enabled(tpg)) {
		writel(MGC_DTYPE_RAW(bpp) << 4 | MGC_DTYPE_RAW(bpp),
		       mg_base + MGC_MG_MIPI_DTYPES);
		writel(2, mg_base + MGC_MG_MULTI_DTYPES_MODE);
	} else {
		writel(MGC_DTYPE_RAW(bpp) << 4 | MGC_DTYPE_RAW(bpp),
		       mg_base + MGC_MG_MIPI_DTYPES);
		writel(0, mg_base + MGC_MG_MULTI_DTYPES_MODE);
	}

	/*
	 * configure frame information
	 */
	writel(0, mg_base + MGC_MG_NOF_FRAMES);
	writel(width | height << 16, mg_base + MGC_MG_FRAME_DIM);

	tpg_get_timing(tpg, &dto, &hblank, &vblank);
	writel(hblank, mg_base + MGC_MG_HBLANK);
	writel(vblank, mg_base + MGC_MG_VBLANK);

	/*
	 * configure tpg mode, colors, mask, tile dimension
	 * Mode was set by user configuration
	 * 0 - Ramp mode
	 * 1 - Checkerboard
	 * 2 - Monochrome per frame
	 * 3 - Color palette
	 */
	writel(TPG_MODE_COLOR_PALETTE, mg_base + MGC_MG_TPG_MODE);

	/* red and green for checkerboard, n/a for other modes */
	writel(58, mg_base + MGC_MG_TPG_R0);
	writel(122, mg_base + MGC_MG_TPG_G0);
	writel(46, mg_base + MGC_MG_TPG_B0);
	writel(123, mg_base + MGC_MG_TPG_R1);
	writel(85, mg_base + MGC_MG_TPG_G1);
	writel(67, mg_base + MGC_MG_TPG_B1);

	writel(0x0, mg_base + MGC_MG_TPG_FACTORS);

	/* hor_mask [15:0] ver_mask [31:16] */
	writel(0xffffffff, mg_base + MGC_MG_TPG_MASKS);
	/* xy_mask [11:0] */
	writel(0xfff, mg_base + MGC_MG_TPG_XY_MASK);
	writel(((MGC_TPG_TILE_WIDTH << 16) | MGC_TPG_TILE_HEIGHT),
	       mg_base + MGC_MG_TPG_TILE_DIM);

	writel(dto, mg_base + MGC_MG_DTO_SPEED_CTRL_INCR_VAL);
	writel(1, mg_base + MGC_MG_DTO_SPEED_CTRL_EN);

	/* disable err_injection */
	writel(0, mg_base + MGC_MG_ERR_INJECT);
	writel(0, mg_base + MGC_MG_ERR_LOCATION);

	ipu7_mipigen_regdump(tpg, mg_base);

	dev_dbg(dev, "starting MG%d streaming...\n", csi_port);

	/* kick and start */
	writel(port_map, mgc_base + MGC_KICK);

	return 0;
}

static void ipu7_isys_ungate_mgc(struct ipu7_isys_tpg *tpg, int enable)
{
	struct ipu7_isys_csi2 *csi2;
	u32 offset;
	struct ipu7_isys *isys = tpg->isys;

	csi2 = &isys->csi2[tpg->index];
	offset = IS_IO_GPREGS_BASE;

	/* MGC is in use by SW or not */
	if (enable)
		writel(1, csi2->base + offset + MGC_CLK_GATE);
	else
		writel(0, csi2->base + offset + MGC_CLK_GATE);
}

static void ipu7_isys_mgc_csi2_s_stream(struct ipu7_isys_tpg *tpg, int enable)
{
	struct device *dev = &tpg->isys->adev->auxdev.dev;
	struct ipu7_isys *isys = tpg->isys;
	struct ipu7_isys_csi2 *csi2;
	u32 port, offset, val;
	void __iomem *isys_base = isys->pdata->base;

	port = tpg->index;
	csi2 = &isys->csi2[port];

	offset = IS_IO_GPREGS_BASE;
	val = readl(isys_base + offset + CSI_PORT_CLK_GATE);
	dev_dbg(dev, "current CSI port %u clk gate 0x%x\n", port, val);

	if (!enable) {
		writel(~(1 << port) & val,
		       isys_base + offset + CSI_PORT_CLK_GATE);
		return;
	}

	/* set csi port is using by SW */
	writel(1 << port | val, isys_base + offset + CSI_PORT_CLK_GATE);
	/* input is coming from MGC */
	offset = IS_IO_CSI2_ADPL_PORT_BASE(port);
	writel(CSI_MIPIGEN_INPUT,
	       csi2->base + offset + CSI2_ADPL_INPUT_MODE);
}

/* TODO: add the processing of vc */
int tpg_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ipu7_isys_tpg *tpg = to_ipu7_isys_tpg(sd);
	struct ipu7_isys_stream *stream = tpg->av->stream;
	struct device *dev = &tpg->isys->adev->auxdev.dev;
	int ret;

	if (tpg->index >= IPU7_ISYS_CSI_PORT_NUM) {
		dev_err(dev, "invalid MGC index %d\n", tpg->index);
		return -EINVAL;
	}

	if (!enable) {
		/* Stop MGC */
		stream->asd->is_tpg = false;
		stream->asd = NULL;
		ipu7_isys_mgc_csi2_s_stream(tpg, enable);
		ret = tpg_stop_stream(tpg);
		ipu7_isys_ungate_mgc(tpg, enable);

		return ret;
	}

	stream->asd = &tpg->asd;
	/* ungate the MGC clock to program */
	ipu7_isys_ungate_mgc(tpg, enable);
	/* Start MGC */
	ret = tpg_start_stream(tpg);
	v4l2_ctrl_handler_setup(&tpg->asd.ctrl_handler);
	ipu7_isys_mgc_csi2_s_stream(tpg, enable);

	return ret;
}

void ipu7_isys_tpg_cleanup(struct ipu7_isys_tpg *tpg)
{
	v4l2_device_unregister_subdev(&tpg->asd.sd);
	ipu7_isys_subdev_cleanup(&tpg->asd);
}

int ipu7_isys_tpg_init(struct ipu7_isys_tpg *tpg, struct ipu7_isys *isys,
		       void __iomem *base, void __iomem *sel,
		       unsigned int index)
{
	struct device *dev = &isys->adev->auxdev.dev;
	int ret;

	tpg->isys = isys;
	tpg->base = base;
	tpg->sel = sel;
	tpg->index = index;

	tpg->asd.sd.entity.ops = &tpg_entity_ops;
	tpg->asd.ctrl_init = ipu7_isys_tpg_init_controls;
	tpg->asd.isys = isys;

	ret = ipu7_isys_subdev_init(&tpg->asd, &tpg_sd_ops, 5,
				    NR_OF_TPG_SINK_PADS, NR_OF_TPG_SOURCE_PADS);
	if (ret)
		return ret;

	tpg->asd.sd.flags &= ~V4L2_SUBDEV_FL_STREAMS;
	tpg->asd.sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	tpg->asd.pad[TPG_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	tpg->asd.source = IPU_INSYS_MIPI_PORT_0 + index;
	tpg->asd.supported_codes = tpg_supported_codes;
	tpg->asd.sd.internal_ops = &ipu7_isys_tpg_internal_ops;

	snprintf(tpg->asd.sd.name, sizeof(tpg->asd.sd.name),
		 IPU_ISYS_ENTITY_PREFIX " TPG %u", index);
	v4l2_set_subdevdata(&tpg->asd.sd, &tpg->asd);

	ret = v4l2_subdev_init_finalize(&tpg->asd.sd);
	if (ret) {
		dev_err(dev, "failed to finalize subdev (%d)\n", ret);
		goto fail;
	}

	ret = v4l2_device_register_subdev(&isys->v4l2_dev, &tpg->asd.sd);
	if (ret) {
		dev_info(dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	return 0;

fail:
	ipu7_isys_tpg_cleanup(tpg);

	return ret;
}
