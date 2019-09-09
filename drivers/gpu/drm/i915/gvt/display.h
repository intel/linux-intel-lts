/*
 * Copyright(c) 2011-2016 Intel Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Ke Yu
 *    Zhiyuan Lv <zhiyuan.lv@intel.com>
 *
 * Contributors:
 *    Terrence Xu <terrence.xu@intel.com>
 *    Changbin Du <changbin.du@intel.com>
 *    Bing Niu <bing.niu@intel.com>
 *    Zhi Wang <zhi.a.wang@intel.com>
 *
 */

#ifndef _GVT_DISPLAY_H_
#define _GVT_DISPLAY_H_

#include "display/intel_display_types.h"
#include "edid.h"

#define SBI_REG_MAX	20
#define DPCD_SIZE	0x700

#define intel_vgpu_display_has_monitor(disp_path) \
	((disp_path)->edid && (disp_path)->edid->data_valid)

#define intel_vgpu_display_is_dp(disp_path) \
	(((disp_path)->port_type == INTEL_OUTPUT_DP) || \
	((disp_path)->port_type == INTEL_OUTPUT_EDP) || \
	((disp_path)->port_type == INTEL_OUTPUT_DP_MST))

#define INTEL_GVT_MAX_UEVENT_VARS	3

/* DPCD start */
#define DPCD_SIZE	0x700

/* DPCD */
#define DP_SET_POWER            0x600
#define DP_SET_POWER_D0         0x1
#define AUX_NATIVE_WRITE        0x8
#define AUX_NATIVE_READ         0x9

#define AUX_NATIVE_REPLY_MASK   (0x3 << 4)
#define AUX_NATIVE_REPLY_ACK    (0x0 << 4)
#define AUX_NATIVE_REPLY_NAK    (0x1 << 4)
#define AUX_NATIVE_REPLY_DEFER  (0x2 << 4)

#define AUX_BURST_SIZE          20

/* DPCD addresses */
#define DPCD_REV			0x000
#define DPCD_MAX_LINK_RATE		0x001
#define DPCD_MAX_LANE_COUNT		0x002

#define DPCD_TRAINING_PATTERN_SET	0x102
#define	DPCD_SINK_COUNT			0x200
#define DPCD_LANE0_1_STATUS		0x202
#define DPCD_LANE2_3_STATUS		0x203
#define DPCD_LANE_ALIGN_STATUS_UPDATED	0x204
#define DPCD_SINK_STATUS		0x205

/* link training */
#define DPCD_TRAINING_PATTERN_SET_MASK	0x03
#define DPCD_LINK_TRAINING_DISABLED	0x00
#define DPCD_TRAINING_PATTERN_1		0x01
#define DPCD_TRAINING_PATTERN_2		0x02

#define DPCD_CP_READY_MASK		(1 << 6)

/* lane status */
#define DPCD_LANES_CR_DONE		0x11
#define DPCD_LANES_EQ_DONE		0x22
#define DPCD_SYMBOL_LOCKED		0x44

#define DPCD_INTERLANE_ALIGN_DONE	0x01

#define DPCD_SINK_IN_SYNC		0x03
/* DPCD end */

#define SBI_RESPONSE_MASK               0x3
#define SBI_RESPONSE_SHIFT              0x1
#define SBI_STAT_MASK                   0x1
#define SBI_STAT_SHIFT                  0x0
#define SBI_OPCODE_SHIFT                8
#define SBI_OPCODE_MASK			(0xff << SBI_OPCODE_SHIFT)
#define SBI_CMD_IORD                    2
#define SBI_CMD_IOWR                    3
#define SBI_CMD_CRRD                    6
#define SBI_CMD_CRWR                    7
#define SBI_ADDR_OFFSET_SHIFT           16
#define SBI_ADDR_OFFSET_MASK            (0xffff << SBI_ADDR_OFFSET_SHIFT)

struct intel_vgpu_sbi_register {
	unsigned int offset;
	u32 value;
};

struct intel_vgpu_sbi {
	int number;
	struct intel_vgpu_sbi_register registers[SBI_REG_MAX];
};

enum intel_gvt_plane_type {
	PRIMARY_PLANE = 0,
	CURSOR_PLANE,
	SPRITE_PLANE,
	MAX_PLANE
};

struct intel_vgpu_dpcd_data {
	bool data_valid;
	u8 data[DPCD_SIZE];
};

enum intel_vgpu_edid {
	GVT_EDID_1024_768,
	GVT_EDID_1920_1200,
	GVT_EDID_NUM,
};

struct vgpu_scaler_config {
	u32 win_pos[2];
	u32 win_size[2];
	u32 ctrl[2];
};

struct prec_pal_data {
	u32 val;
	bool dirty;
};

enum {
	INTEL_GVT_DIRECT_DISPLAY_HW_VSYNC = 0
};

/* Per-path display configuration */
struct intel_vgpu_display_path {
	/* virtual display path id */
	u32 id;
	/* resolution from fake EDID */
	u64 resolution;
	/* assigned host display port */
	enum port p_port;
	/* assigned host pipe corresponding to port */
	enum pipe p_pipe;
	/* current vgpu port, starts from PORT_B */
	enum port port;
	/* current vgpu port type */
	enum intel_output_type port_type;
	/* I2C EDID information */
	struct intel_vgpu_i2c_edid i2c_edid;
	/* EDID information */
	struct intel_vgpu_edid_data *edid;
	/* Resolution ID from QEMU */
	enum intel_vgpu_edid edid_id;
	/* DPCD information */
	struct intel_vgpu_dpcd_data *dpcd;
	/* current vgpu pipe, starts from PIPE_A */
	enum pipe pipe;
	/* current vgpu transcoder, starts from TRANSCODER_A */
	enum transcoder trans;
	/* overridden scaler settings for vgpu pipe */
	struct vgpu_scaler_config scaler_cfg;
	/* watermark for vgpu */
	struct skl_pipe_wm wm_cfg;
	/* Precision palette data */
	struct prec_pal_data prec_palette_split[PAL_PREC_INDEX_VALUE_MASK + 1];
	struct prec_pal_data prec_palette_nonsplit[PAL_PREC_INDEX_VALUE_MASK + 1];
	/* current foreground state of display */
	u32 foreground_state;
	/* request to switch this vgpu as foreground */
	u32 foreground;
	/* display status bits */
	u32 stat;
	/* vsync freq */
	u64 vsync_interval_ns;
	/* last hw vsync injected */
	u64 last_hwvsync_ns;
	/* number of emulated vsync injected */
	u64 sw_vsync_injected;
	/* pointer to prev or next path */
	struct list_head list;
};

struct intel_vgpu_display {
	/* SBI information */
	struct intel_vgpu_sbi sbi;
	/* Per-path virtual display configuration */
	struct list_head path_list;
	/* display switch lock */
	struct mutex sw_lock;
};

static inline char *vgpu_edid_str(enum intel_vgpu_edid id)
{
	switch (id) {
	case GVT_EDID_1024_768:
		return "1024x768";
	case GVT_EDID_1920_1200:
		return "1920x1200";
	default:
		return "";
	}
}

static inline unsigned int vgpu_edid_xres(enum intel_vgpu_edid id)
{
	switch (id) {
	case GVT_EDID_1024_768:
		return 1024;
	case GVT_EDID_1920_1200:
		return 1920;
	default:
		return 0;
	}
}

static inline unsigned int vgpu_edid_yres(enum intel_vgpu_edid id)
{
	switch (id) {
	case GVT_EDID_1024_768:
		return 768;
	case GVT_EDID_1920_1200:
		return 1200;
	default:
		return 0;
	}
}

void intel_gvt_emulate_vblank(struct intel_gvt *gvt);
void intel_gvt_check_vblank_emulation(struct intel_vgpu *vgpu, enum pipe pipe);

int intel_vgpu_init_display(struct intel_vgpu *vgpu, u64 resolution);
void intel_vgpu_reset_display(struct intel_vgpu *vgpu);
void intel_vgpu_clean_display(struct intel_vgpu *vgpu);
void intel_gvt_switch_display_pipe(struct intel_gvt *gvt, enum pipe pipe,
				   struct intel_vgpu *old_v,
				   struct intel_vgpu *new_v);
void intel_gvt_flush_pipe_color(struct intel_gvt *gvt, enum pipe pipe,
				struct intel_vgpu *vgpu);

int pipe_is_enabled(struct intel_vgpu *vgpu, enum pipe pipe);

#if IS_ENABLED(CONFIG_DRM_I915_GVT_ACRN_GVT)
int intel_vgpu_g2v_setup_gop(struct intel_vgpu *vgpu);
#endif
#endif
