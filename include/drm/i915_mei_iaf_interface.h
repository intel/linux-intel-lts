/* SPDX-License-Identifier: (GPL-2.0+) */
/*
 * Copyright © 2020-2021 Intel Corporation
 */

#ifndef _I915_MEI_IAF_INTERFACE_H_
#define _I915_MEI_IAF_INTERFACE_H_

#include <linux/device.h>

/**
 * struct i915_iaf_component_ops- ops for IAF services.
 * @owner: Module providing the ops
 * @commit_svn: commits current FW SVN
 */
struct i915_iaf_component_ops {
	/**
	 * @owner: mei_iaf module
	 */
	struct module *owner;

	int (*commit_svn)(const struct device *dev);
};

#endif /* _I915_MEI_IAF_INTERFACE_H_ */
