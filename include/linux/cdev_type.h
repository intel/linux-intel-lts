/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_CDEV_TYPE_H
#define _LINUX_CDEV_TYPE_H

#include <linux/kobject.h>
#include <linux/kdev_t.h>
#include <linux/list.h>

struct file_operations;
struct inode;
struct module;

struct cdev {
	struct kobject kobj;
	struct module *owner;
	const struct file_operations *ops;
	struct list_head list;
	dev_t dev;
	unsigned int count;
} __randomize_layout;

#endif
