/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef INTEL_GT_PM_UNPARK_WORK_H
#define INTEL_GT_PM_UNPARK_WORK_H

#include <linux/list.h>
#include <linux/workqueue.h>

struct intel_gt;

/**
 * struct intel_gt_pm_unpark_work - work to be scheduled when GT unparked
 */
struct intel_gt_pm_unpark_work {
	/**
	 * @link: link into gt->pm_unpark_work_list of workers that need to be
	 * scheduled when GT is unpark, protected by gt->pm_unpark_work_lock
	 */
	struct list_head link;
	/** @worker: will be scheduled when GT unparked */
	struct work_struct worker;
};

void intel_gt_pm_unpark_work_queue(struct intel_gt *gt);

void intel_gt_pm_unpark_work_add(struct intel_gt *gt,
				 struct intel_gt_pm_unpark_work *work);

static inline void
intel_gt_pm_unpark_work_init(struct intel_gt_pm_unpark_work *work,
			     work_func_t fn)
{
	INIT_LIST_HEAD(&work->link);
	INIT_WORK(&work->worker, fn);
}

#endif /* INTEL_GT_PM_UNPARK_WORK_H */
