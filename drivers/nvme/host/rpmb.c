// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018-2019 Intel Corporation.
 */
#include <linux/rpmb.h>
#include "nvme.h"
#define NVME_SECP_RPMB   0xEA /* Security Protocol EAh is assigned
			       * for NVMe use (refer to ACS-4)
			       */
#define NVME_SPSP_RPMB 0x0001 /* RPMB Target */
static int nvme_rpmb_cmd_seq(struct device *dev, u8 target,
			     struct rpmb_cmd *cmds, u32 ncmds)
{
	struct nvme_ctrl *ctrl;
	struct rpmb_cmd *cmd;
	u32 size;
	int ret;
	int i;

	ctrl = dev_get_drvdata(dev);

	for (ret = 0, i = 0; i < ncmds && !ret; i++) {
		cmd = &cmds[i];
		size = rpmb_ioc_frames_len_nvme(cmd->nframes);
		if (cmd->flags & RPMB_F_WRITE)
			ret = nvme_sec_send(ctrl, target,
					    NVME_SPSP_RPMB, NVME_SECP_RPMB,
					    cmd->frames, size);
		else
			ret = nvme_sec_recv(ctrl, target,
					    NVME_SPSP_RPMB, NVME_SECP_RPMB,
					    cmd->frames, size);
	}

	return ret;
}

static int nvme_rpmb_get_capacity(struct device *dev, u8 target)
{
	struct nvme_ctrl *ctrl;

	ctrl = dev_get_drvdata(dev);

	return ((ctrl->rpmbs >> 16) & 0xFF) + 1;
}

static struct rpmb_ops nvme_rpmb_dev_ops = {
	.cmd_seq = nvme_rpmb_cmd_seq,
	.get_capacity = nvme_rpmb_get_capacity,
	.type = RPMB_TYPE_NVME,
};

static void nvme_rpmb_set_cap(struct nvme_ctrl *ctrl,
			      struct rpmb_ops *ops)
{
	ops->wr_cnt_max = ((ctrl->rpmbs >> 24) & 0xFF) + 1;
	ops->rd_cnt_max = ops->wr_cnt_max;
	ops->block_size = 2; /* 1 sector == 2 half sectors */
	ops->auth_method = (ctrl->rpmbs >> 3) & 0x3;
}

static void nvme_rpmb_add(struct nvme_ctrl *ctrl)
{
	struct rpmb_dev *rdev;
	int ndevs = ctrl->rpmbs & 0x7;
	int i;

	nvme_rpmb_set_cap(ctrl, &nvme_rpmb_dev_ops);

	/* Add RPMB partitions */
	for (i = 0; i < ndevs; i++) {
		rdev = rpmb_dev_register(ctrl->device, i, &nvme_rpmb_dev_ops);
		if (IS_ERR(rdev)) {
			dev_warn(ctrl->device, "%s: cannot register to rpmb %ld\n",
				 dev_name(ctrl->device), PTR_ERR(rdev));
		}
		dev_set_drvdata(&rdev->dev, ctrl);
	}
}

static void nvme_rpmb_remove(struct nvme_ctrl *ctrl)
{
	int ndevs = ctrl->rpmbs & 0x7;
	int i;

	/* FIXME: target */
	for (i = 0; i < ndevs; i++)
		rpmb_dev_unregister_by_device(ctrl->device, i);
}

int nvme_init_rpmb(struct nvme_ctrl *ctrl)
{
	dev_err(ctrl->device, "RPMBS %X\n", ctrl->rpmbs);

	if ((ctrl->rpmbs & 0x7) == 0x0) {
		dev_err(ctrl->device, "RPMBS No partitions\n");
		return 0;
	}

	dev_err(ctrl->device, "RPMBS Number of partitions %d\n",
		ctrl->rpmbs & 0x7);
	dev_err(ctrl->device, "RPMBS Authentication Method: %d\n",
		(ctrl->rpmbs >> 3) & 0x3);
	dev_err(ctrl->device, "RPMBS Total Size: %d %dK",
		(ctrl->rpmbs >> 16) & 0xFF,
		(((ctrl->rpmbs >> 16) & 0xFF) + 1) *  128);
	dev_err(ctrl->device, "RPMBS Access Size: %d %dB",
		(ctrl->rpmbs >> 24) & 0xFF,
		(((ctrl->rpmbs >> 24) & 0xFF) + 1) * 512);

	nvme_rpmb_add(ctrl);

	return 0;
}

void nvme_exit_rpmb(struct nvme_ctrl *ctrl)
{
	nvme_rpmb_remove(ctrl);
}
