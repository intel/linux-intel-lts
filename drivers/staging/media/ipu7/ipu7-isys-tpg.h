/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2025 Intel Corporation
 */

#ifndef IPU7_ISYS_TPG_H
#define IPU7_ISYS_TPG_H

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#include "ipu7-isys-subdev.h"
#include "ipu7-isys-video.h"
#include "ipu7-isys-queue.h"

struct ipu7_isys_tpg_pdata;
struct ipu7_isys;

#define TPG_PAD_SOURCE			0
#define NR_OF_TPG_PADS			1
#define NR_OF_TPG_SOURCE_PADS		1
#define NR_OF_TPG_SINK_PADS		0
#define NR_OF_TPG_STREAMS		1

enum isys_tpg_mode {
	TPG_MODE_RAMP = 0,
	TPG_MODE_CHECKERBOARD = 1,
	TPG_MODE_MONO = 2,
	TPG_MODE_COLOR_PALETTE = 3,
};

/*
 * struct ipu7_isys_tpg
 *
 * @nlanes: number of lanes in the receiver
 */
struct ipu7_isys_tpg {
	struct ipu7_isys_subdev asd;
	struct ipu7_isys_tpg_pdata *pdata;
	struct ipu7_isys *isys;
	struct ipu7_isys_video *av;

	/* MG base not MGC */
	void __iomem *base;
	void __iomem *sel;
	unsigned int index;

	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
};

#define ipu7_isys_subdev_to_tpg(__sd)			\
	container_of(__sd, struct ipu7_isys_tpg, asd)

#define to_ipu7_isys_tpg(sd)			\
	container_of(to_ipu7_isys_subdev(sd),	\
		     struct ipu7_isys_tpg, asd)

void ipu7_isys_tpg_sof_event_by_stream(struct ipu7_isys_stream *stream);
void ipu7_isys_tpg_eof_event_by_stream(struct ipu7_isys_stream *stream);
int ipu7_isys_tpg_init(struct ipu7_isys_tpg *tpg,
		       struct ipu7_isys *isys,
		       void __iomem *base, void __iomem *sel,
		       unsigned int index);
void ipu7_isys_tpg_cleanup(struct ipu7_isys_tpg *tpg);
int tpg_set_stream(struct v4l2_subdev *sd, int enable);

#endif /* IPU7_ISYS_TPG_H */
