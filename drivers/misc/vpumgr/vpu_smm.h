/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (C) 2020 Intel Corporation
 */
#ifndef _VPU_SMM_H
#define _VPU_SMM_H
#include <linux/kernel.h>
#include <linux/rbtree.h>

#include "vpu_common.h"

struct vpumgr_smm {
	struct vpumgr_device *vdev;
	struct rb_root	 imp_rb;
	struct mutex	 imp_rb_lock; /* protects imp_rb */
};

int smm_init(struct vpumgr_device *vdev);
int smm_fini(struct vpumgr_device *vdev);

int smm_open(struct vpumgr_smm *sess, struct vpumgr_device *vdev);
int smm_close(struct vpumgr_smm *sess);

int smm_alloc(struct vpumgr_smm *sess, struct vpumgr_args_alloc *arg);
int smm_import(struct vpumgr_smm *sess, struct vpumgr_args_import *arg);
int smm_unimport(struct vpumgr_smm *sess, int *p_dmabuf_fd);
int smm_ptr2vpu(struct vpumgr_smm *sess, unsigned long *arg);

int smm_debugfs_stats_show(struct seq_file *file, struct vpumgr_smm *sess);

#endif
