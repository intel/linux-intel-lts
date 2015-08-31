/*
 *  skl-debug.c - Debugfs for skl driver
 *
 *  Copyright (C) 2015 Intel Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/pci.h>
#include <linux/debugfs.h>
#include "skl.h"

struct skl_debug {
	struct skl *skl;
	struct device *dev;

	struct dentry *fs;
	struct dentry *nhlt;
};

static ssize_t nhlt_control_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char *state;

	state = d->skl->nhlt_override ? "enable\n" : "disable\n";
	return simple_read_from_buffer(user_buf, count, ppos,
			state, strlen(state));
}

static ssize_t nhlt_control_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char buf[16];
	int len = min(count, (sizeof(buf) - 1));


	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = 0;

	if (!strncmp(buf, "enable\n", len))
		d->skl->nhlt_override = true;
	else if (!strncmp(buf, "disable\n", len))
		d->skl->nhlt_override = false;
	else
		return -EINVAL;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	return len;
}

static const struct file_operations ssp_cntrl_nhlt_fops = {
	.open = simple_open,
	.read = nhlt_control_read,
	.write = nhlt_control_write,
	.llseek = default_llseek,
};

static int skl_init_nhlt(struct skl_debug *d)
{
	if (!debugfs_create_file("control",
				0644, d->nhlt,
				d, &ssp_cntrl_nhlt_fops)) {
		dev_err(d->dev, "nhlt control debugfs init failed\n");
		return -EIO;
	}

	return 0;
}

struct skl_debug *skl_debugfs_init(struct skl *skl)
{
	struct skl_debug *d;

	d = devm_kzalloc(&skl->pci->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;

	/* create the root dir first */
	d->fs = debugfs_create_dir("snd_soc_skl", NULL);
	if (IS_ERR(d->fs) || !d->fs) {
		dev_err(&skl->pci->dev, "debugfs root creation failed\n");
		return NULL;
	}

	d->skl = skl;
	d->dev = &skl->pci->dev;

	/* now create the NHLT dir */
	d->nhlt =  debugfs_create_dir("nhlt", d->fs);
	if (IS_ERR(d->nhlt) || !d->nhlt) {
		dev_err(&skl->pci->dev, "nhlt debugfs create failed\n");
		return NULL;
	}

	skl_init_nhlt(d);

	return d;
}

void skl_debugfs_exit(struct skl_debug *d)
{
	debugfs_remove_recursive(d->fs);

	kfree(d);

}
