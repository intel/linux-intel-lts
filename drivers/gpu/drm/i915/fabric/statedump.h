/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2022 Intel Corporation.
 *
 */

#ifndef STATEDUMP_H_INCLUDED
#define STATEDUMP_H_INCLUDED

#include <linux/debugfs.h>
#include <linux/rwsem.h>
#include <linux/semaphore.h>

struct state_dump {
	struct debugfs_blob_wrapper blob;
	/*
	 * Blocks so only one process can state dump across open/read/release
	 * of the debugfs node
	 */
	struct semaphore state_dump_sem;
	/*
	 * Stops all new mailbox traffic and waits until all ops in the device
	 * have finished
	 */
	struct rw_semaphore state_dump_mbdb_sem;
};

struct fsubdev;

void statedump_node_init(struct fsubdev *sd);

#endif
