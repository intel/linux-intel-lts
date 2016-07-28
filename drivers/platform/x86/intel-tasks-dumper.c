/*
 * Intel Trace Hub to USB dvc-trace  driver
 *
 * Copyright (C) 2015, Intel Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/console.h>
#include <linux/sched.h>

static const char * const filter[] = {
	"Watchdog detected",	/* Hard lockup */
	"softlockup:",		/* Soft lockup */
	"Kernel Watchdog",	/* iTCO warning */
};

static int force_en;
module_param(force_en, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(force_en, "Force enable");

static int force_dis;
module_param(force_dis, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(force_dis, "Force disable");

/* 1 pass, 0 not*/
static int check_filter(const char *v)
{
	int i;

	if (force_dis)
		return 0;

	if (force_en)
		return 1;

	for (i = 0; i < (sizeof(filter) / sizeof(*filter)); i++) {
		if (!strncmp(v, filter[i], strlen(filter[i]))) {
			pr_debug("Matching filter[%d] %s", i, filter[i]);
			return 1;
		}
	}
	return 0;
}

static int intel_task_panic_hndl(struct notifier_block *n,
				 unsigned long val, void *v)
{
	char *buf = v;

	/* for panic val is hardcoded to 0 */
	if (val) {
		return NOTIFY_OK;
	} else if (!check_filter(buf)) {
		pr_info("Skip tasks dumper.\n");
		return NOTIFY_OK;
	}

	console_suspend_slow();
	pr_info(" --- tasks dumper [BEGIN] ---\n");
	show_state_filter(0);
	pr_info(" --- tasks dumper [END] ---\n");
	console_restore_slow();

       /* maybe NOTIFY_STOP_MASK ....*/
	return NOTIFY_OK;
}

static struct notifier_block panic_notifier = {
	.notifier_call = intel_task_panic_hndl,
	.priority = 2, /*Not the last one*/
};

static int __init intel_tasks_dumper_init(void)
{
	pr_info("Register tasks dumper\n");
	atomic_notifier_chain_register(&panic_notifier_list, &panic_notifier);
	return 0;
}

static void __exit intel_tasks_dumper_exit(void)
{
	atomic_notifier_chain_unregister(&panic_notifier_list, &panic_notifier);
}

module_init(intel_tasks_dumper_init);
module_exit(intel_tasks_dumper_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Traian Schiau <traianx.schiau@intel.com>");
