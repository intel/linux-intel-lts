/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef INTEL_GT_DEBUGFS_H
#define INTEL_GT_DEBUGFS_H

#include <linux/file.h>

struct intel_gt;
struct seq_file;
struct file;

int i915_gt_debugfs_single_open(struct file *file,
				int (*show)(struct seq_file *m, void *data),
				void *data);
int i915_gt_debugfs_single_open_size(struct file *file,
				     int (*show)(struct seq_file *m, void *data),
				      void *data, size_t size);
int i915_gt_debugfs_single_release(struct inode *inode, struct file *file);
int i915_gt_debugfs_raw_attr_open(struct inode *inode, struct file *file,
				  int (*open)(struct inode *inode, struct file *file));
int i915_gt_debugfs_raw_attr_close(struct inode *inode, struct file *file,
				   int (*close)(struct inode *inode, struct file *file));

int i915_gt_debugfs_simple_attr_open(struct inode *inode, struct file *file,
				     int (*get)(void *, u64 *), int (*set)(void *, u64),
				     const char *fmt);
int i915_gt_debugfs_simple_attr_release(struct inode *inode, struct file *file);

#define __GT_DEBUGFS_ATTRIBUTE_FOPS(__name)				\
static const struct file_operations __name ## _fops = {			\
	.owner = THIS_MODULE,						\
	.open = __name ## _open,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
	.release = i915_gt_debugfs_single_release,			\
}

#define DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE(__name)			              \
static int __name ## _open(struct inode *inode, struct file *file)	              \
{									              \
	return i915_gt_debugfs_single_open(file, __name ## _show, inode->i_private);  \
}									              \
__GT_DEBUGFS_ATTRIBUTE_FOPS(__name)

#define DEFINE_INTEL_GT_DEBUGFS_ATTRIBUTE_WITH_SIZE(__name, __size_vf)		         \
static int __name ## _open(struct inode *inode, struct file *file)		         \
{										         \
	return i915_gt_debugfs_single_open_size(file, __name ## _show, inode->i_private, \
			    __size_vf(inode->i_private));			         \
}										         \
__GT_DEBUGFS_ATTRIBUTE_FOPS(__name)

#define DEFINE_I915_GT_SIMPLE_ATTRIBUTE(__fops, __get, __set, __fmt)                     \
static int __fops ## _open(struct inode *inode, struct file *file)	                 \
{									                 \
	__simple_attr_check_format(__fmt, 0ull);			                 \
	return i915_gt_debugfs_simple_attr_open(inode, file, __get, __set, __fmt);	 \
}									                 \
static const struct file_operations __fops = {				                 \
	.owner	 = THIS_MODULE,								 \
	.open	 = __fops ## _open,					                 \
	.release = i915_gt_debugfs_simple_attr_release,			                 \
	.read	 = simple_attr_read,					                 \
	.write	 = simple_attr_write,					                 \
	.llseek	 = generic_file_llseek,					                 \
}

#define DEFINE_I915_GT_RAW_ATTRIBUTE(__fops, __open, __close, __read, __write, __llseek) \
static int __fops ## _open(struct inode *inode, struct file *file)	                 \
{									                 \
	return  i915_gt_debugfs_raw_attr_open(inode, file, __open);                      \
}									                 \
static int __fops ## _close(struct inode *inode, struct file *file)	                 \
{									                 \
	return i915_gt_debugfs_raw_attr_close(inode, file, __close);                     \
}									                 \
static const struct file_operations __fops = {				                 \
	.owner	 = THIS_MODULE,						                 \
	.open	 = __fops ## _open,					                 \
	.release = __fops ## _close,					                 \
	.read	 = __read,					                         \
	.write	 = __write,					                         \
	.llseek	 = __llseek,					                         \
}

void intel_gt_debugfs_register(struct intel_gt *gt);

struct intel_gt_debugfs_file {
	const char *name;
	const struct file_operations *fops;
	bool (*eval)(void *data);
};

void intel_gt_debugfs_register_files(struct dentry *root,
				     const struct intel_gt_debugfs_file *files,
				     unsigned long count, void *data);

/* functions that need to be accessed by the upper level non-gt interfaces */
int intel_gt_debugfs_reset_show(struct intel_gt *gt, u64 *val);
void intel_gt_debugfs_reset_store(struct intel_gt *gt, u64 val);

#endif /* INTEL_GT_DEBUGFS_H */
