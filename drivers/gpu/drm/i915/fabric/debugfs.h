/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2022 Intel Corporation.
 *
 */

#ifndef DEBUGFS_H_INCLUDED
#define DEBUGFS_H_INCLUDED

#include <linux/dcache.h>

#include "iaf_drv.h"

#define F_DENTRY(filp) ((filp)->f_path.dentry)

__printf(4, 5)
void print_diag(char *buf, size_t *buf_offset, size_t buf_size, const char *fmt, ...);
struct dentry *get_debugfs_root_node(void);
ssize_t blob_read(struct file *file, char __user *user_buffer, size_t count, loff_t *ppos);
int blob_release(struct inode *inode, struct file *file);
void init_debugfs(struct fdev *dev);
void remove_debugfs(struct fdev *dev);
void create_debugfs_root_nodes(void);
void remove_debugfs_root_nodes(void);

#endif
