/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __I915_DEBUGFS_H__
#define __I915_DEBUGFS_H__

#include <linux/file.h>

struct drm_connector;
struct drm_i915_gem_object;
struct drm_i915_private;
struct seq_file;
struct file;

#ifdef CONFIG_DEBUG_FS
void i915_debugfs_register(struct drm_i915_private *dev_priv);
void i915_debugfs_describe_obj(struct seq_file *m, struct drm_i915_gem_object *obj);
#else
static inline void i915_debugfs_register(struct drm_i915_private *dev_priv) {}
static inline void i915_debugfs_describe_obj(struct seq_file *m, struct drm_i915_gem_object *obj) {}
#endif

struct i915_debugfs_file {
	const char *name;
	const struct file_operations *fops;
	bool (*eval)(void *data);
};

int i915_debugfs_raw_attr_open(struct inode *inode, struct file *file,
			       int (*open)(struct inode*, struct file*));
int i915_debugfs_raw_attr_close(struct inode *inode, struct file *file,
				int (*close)(struct inode*, struct file*));

int i915_debugfs_simple_attr_open(struct inode *inode, struct file *file,
				  int (*)(void *, u64 *), int (*set)(void *, u64),
				  const char *fmt);
int i915_debugfs_simple_attr_release(struct inode *inode, struct file *file);
int i915_debugfs_single_open(struct file *file,
			     int (*show)(struct seq_file *m, void *data),
			     void *data);
int i915_debugfs_single_release(struct inode *inode, struct file *file);
void i915_register_debugfs_show_files(struct dentry *root,
				      const struct i915_debugfs_file *files,
				      unsigned long count, void *data);

#define DEFINE_I915_SIMPLE_ATTRIBUTE(__fops, __get, __set, __fmt)                \
static int __fops ## _open(struct inode *inode, struct file *file)	         \
{									         \
	__simple_attr_check_format(__fmt, 0ull);			         \
	return i915_debugfs_simple_attr_open(inode, file, __get, __set, __fmt);	 \
}									         \
static const struct file_operations __fops = {				         \
	.owner	 = THIS_MODULE,							 \
	.open	 = __fops ## _open,						 \
	.release = i915_debugfs_simple_attr_release,			         \
	.read	 = simple_attr_read,					         \
	.write	 = simple_attr_write,					         \
	.llseek	 = generic_file_llseek,					         \
}

#define DEFINE_I915_RAW_ATTRIBUTE(__fops, __open, __close, __read, __write, __llseek) \
static int __fops ## _open(struct inode *inode, struct file *file)	              \
{									              \
	return  i915_debugfs_raw_attr_open(inode, file, __open);                      \
}									              \
static int __fops ## _close(struct inode *inode, struct file *file)	              \
{									              \
	return i915_debugfs_raw_attr_close(inode, file, __close);                     \
}									              \
static const struct file_operations __fops = {				              \
	.owner	 = THIS_MODULE,						              \
	.open	 = __fops ## _open,					              \
	.release = __fops ## _close,					              \
	.read	 = __read,					                      \
	.write	 = __write,					                      \
	.llseek	 = __llseek,					                      \
}

#define DEFINE_I915_SHOW_ATTRIBUTE(__name)					      \
static int __name ## _open(struct inode *inode, struct file *file)	              \
{									              \
	return i915_debugfs_single_open(file, __name ## _show, inode->i_private);     \
}									              \
static const struct file_operations __name ## _fops = {			              \
	.owner		= THIS_MODULE,					              \
	.open		= __name ## _open,				              \
	.read		= seq_read,					              \
	.llseek		= seq_lseek,					              \
	.release	= i915_debugfs_single_release,				      \
}

#endif /* __I915_DEBUGFS_H__ */
