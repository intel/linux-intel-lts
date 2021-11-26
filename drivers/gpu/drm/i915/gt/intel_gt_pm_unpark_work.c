// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "i915_drv.h"
#include "intel_runtime_pm.h"
#include "intel_gt_pm.h"

void intel_gt_pm_unpark_work_queue(struct intel_gt *gt)
{
	struct intel_gt_pm_unpark_work *work, *next;
	unsigned long flags;

	spin_lock_irqsave(&gt->pm_unpark_work_lock, flags);
	list_for_each_entry_safe(work, next,
				 &gt->pm_unpark_work_list, link) {
		list_del_init(&work->link);
		queue_work(system_unbound_wq, &work->worker);
	}
	spin_unlock_irqrestore(&gt->pm_unpark_work_lock, flags);
}

void intel_gt_pm_unpark_work_add(struct intel_gt *gt,
				 struct intel_gt_pm_unpark_work *work)
{
	unsigned long flags;

	spin_lock_irqsave(&gt->pm_unpark_work_lock, flags);
	if (intel_gt_pm_is_awake(gt))
		queue_work(system_unbound_wq, &work->worker);
	else if (list_empty(&work->link))
		list_add_tail(&work->link, &gt->pm_unpark_work_list);
	spin_unlock_irqrestore(&gt->pm_unpark_work_lock, flags);
}
