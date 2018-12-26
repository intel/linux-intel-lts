/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Intel Corporation
 */

#ifndef INTEL_IPU4_PARA_VIRT_PSYS_H_
#define INTEL_IPU4_PARA_VIRT_PSYS_H_

#include <linux/cdev.h>
#include <uapi/linux/ipu-psys.h>

#define IPU_MEDIA_DEV_MODEL_NAME	"ipu4/Broxton B"

struct virt_ipu_psys {
      struct cdev cdev;
      struct device dev;
      struct mutex mutex;

};

struct virt_ipu_psys_fh {
	struct virt_ipu_psys *psys;
	struct mutex mutex;	/* Protects bufmap & kcmds fields */
	struct list_head list;
	struct list_head bufmap;
	struct list_head kcmds[IPU_PSYS_CMD_PRIORITY_NUM];
	struct ipu_psys_kcmd
	*new_kcmd_tail[IPU_PSYS_CMD_PRIORITY_NUM];
	wait_queue_head_t wait;
	struct mutex bs_mutex;	/* Protects buf_set field */
	struct list_head buf_sets;
};
int virt_psys_init(void);
void virt_psys_exit(void);
#define dev_to_vpsys(dev) \
	container_of(dev, struct virt_ipu_psys, dev)

#endif /* INTEL_IPU4_PARA_VIRT_PSYS_H_ */
