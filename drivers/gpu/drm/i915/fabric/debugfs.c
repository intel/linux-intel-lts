// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2022 Intel Corporation.
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "debugfs.h"

#define ROOT_NODE DRIVER_NAME

#define FABRIC_ID_NODE_NAME "fabric_id"
#define FABRIC_DPA_OFFSET "dpa_offset"
#define FABRIC_DPA_SIZE "dpa_size"

static struct dentry *root_node;
static struct dentry *debugfs_fabric;
static struct dentry *debugfs_dpa;

struct dentry *get_debugfs_root_node(void)
{
	return root_node;
}

void print_diag(char *buf, size_t *buf_offset, size_t buf_size, const char *fmt,  ...)
{
	va_list args;
	int i;

	va_start(args, fmt);
	i = vscnprintf(buf + *buf_offset, buf_size - *buf_offset, fmt, args);
	va_end(args);

	*buf_offset += i;
}

/*
 * This is a copy of the kernel's read_file_blob in fs/debugfs/file.c which is declared static.
 * If it every was exported this could be removed and the kernel's function could be used instead.
 */
ssize_t blob_read(struct file *file, char __user *user_buffer, size_t count, loff_t *ppos)
{
	struct debugfs_blob_wrapper *blob = file->private_data;
	struct dentry *dentry = F_DENTRY(file);
	ssize_t ret;

	ret = debugfs_file_get(dentry);
	if (ret)
		return ret;

	ret = simple_read_from_buffer(user_buffer, count, ppos, blob->data, blob->size);

	debugfs_file_put(dentry);

	return ret;
}

int blob_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);

	return 0;
}

void init_debugfs(struct fdev *dev)
{
	char name[128];
	char dest[128];

	dev->dir_node = debugfs_create_dir(dev_name(&dev->pdev->dev), root_node);

	debugfs_create_x32(FABRIC_ID_NODE_NAME, 0400, dev->dir_node, &dev->fabric_id);
	debugfs_create_x32(FABRIC_DPA_OFFSET, 0400, dev->dir_node, (u32 *)&dev->pd->dpa.pkg_offset);
	debugfs_create_x16(FABRIC_DPA_SIZE, 0400, dev->dir_node, (u16 *)&dev->pd->dpa.pkg_size);

	snprintf(dest, ARRAY_SIZE(dest), "../%s", dev_name(&dev->pdev->dev));

	snprintf(name, ARRAY_SIZE(name), "0x%08x", dev->fabric_id);
	dev->fabric_node = debugfs_create_symlink(name, debugfs_fabric, dest);

	snprintf(name, ARRAY_SIZE(name), "0x%016llx-0x%016llx",
		 (u64)dev->pd->dpa.pkg_offset * SZ_1G,
		 (u64)dev->pd->dpa.pkg_offset * SZ_1G +
		 (u64)dev->pd->dpa.pkg_size * SZ_1G - 1);
	dev->dpa_node = debugfs_create_symlink(name, debugfs_dpa, dest);
}

void remove_debugfs(struct fdev *dev)
{
	/* clean up the "global" entries */
	debugfs_remove(dev->fabric_node);
	debugfs_remove(dev->dpa_node);

	/* and now the device specific stuff */
	debugfs_remove_recursive(dev->dir_node);
}

void create_debugfs_root_nodes(void)
{
	root_node = debugfs_create_dir(ROOT_NODE, NULL);
	debugfs_fabric = debugfs_create_dir("fabric", root_node);
	debugfs_dpa = debugfs_create_dir("dpa", root_node);
}

void remove_debugfs_root_nodes(void)
{
	debugfs_remove_recursive(root_node);
}
