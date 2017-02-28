/*
 * Copyright (c) 2015 Intel Corporation
 * Author: Sylvain Chouleur <sylvain.chouleur@intel.com>
 *
 * Distributed under the terms of the GNU GPL, version 2
 */

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/efi.h>
#include <asm/efi.h>

static DEFINE_KTHREAD_WORKER(efi_kworker);
static bool in_panic = false;

#define efi_call_interruptible(f, ...)					\
({									\
	efi_status_t __s;						\
	efi_sync_low_kernel_mappings();					\
	__kernel_fpu_begin();						\
	__s = efi_call((void *)efi.systab->runtime->f, __VA_ARGS__);	\
	__kernel_fpu_end();						\
	__s;								\
})

struct efi_work {
	struct kthread_work work;
	efi_status_t status;
	unsigned long *name_size;
	efi_char16_t *name;
	efi_guid_t *vendor;
	u32 *attr;
	unsigned long *data_size;
	void *data;
};

static void do_get_next_variable_interruptible(struct kthread_work *work)
{
	struct efi_work *ew = container_of(work, struct efi_work, work);

	ew->status = efi_call_interruptible(get_next_variable, ew->name_size,
					    ew->name, ew->vendor);
}

static void do_set_variable_interruptible(struct kthread_work *work)
{
	struct efi_work *ew = container_of(work, struct efi_work, work);

	ew->status = efi_call_interruptible(set_variable, ew->name, ew->vendor,
					    *ew->attr, *ew->data_size,
					    ew->data);
}

static void do_get_variable_interruptible(struct kthread_work *work)
{
	struct efi_work *ew = container_of(work, struct efi_work, work);

	ew->status = efi_call_interruptible(get_variable, ew->name, ew->vendor,
					    ew->attr, ew->data_size, ew->data);
}

static efi_status_t execute_service(kthread_work_func_t service,
				    unsigned long *name_size,
				    efi_char16_t *name, efi_guid_t *vendor,
				    u32 *attr, unsigned long *data_size,
				    void *data)
{
	struct efi_work work = {
		.name_size = name_size,
		.name = name,
		.vendor = vendor,
		.attr = attr,
		.data_size = data_size,
		.data = data,
	};

	init_kthread_work(&work.work, service);
	queue_kthread_work(&efi_kworker, &work.work);

	flush_kthread_work(&work.work);
	return work.status;
}

static efi_status_t get_next_variable_interruptible(unsigned long *name_size,
						    efi_char16_t *name,
						    efi_guid_t *vendor)
{
	return execute_service(do_get_next_variable_interruptible,
			       name_size, name, vendor, NULL, NULL, NULL);
}

static efi_status_t set_variable_interruptible(efi_char16_t *name,
					       efi_guid_t *vendor,
					       u32 attr,
					       unsigned long data_size,
					       void *data)
{
	if (in_panic)
		return efi.set_variable(name, vendor, attr, data_size, data);
	return execute_service(do_set_variable_interruptible,
			       NULL, name, vendor, &attr, &data_size, data);
}

static efi_status_t non_blocking_not_allowed(efi_char16_t *name,
					     efi_guid_t *vendor,
					     u32 attr,
					     unsigned long data_size,
					     void *data)
{
	if (in_panic)
		return efi.set_variable_nonblocking(name, vendor, attr,
						    data_size, data);
	pr_err("efi_interruptible: non blocking operation is not allowed\n");
	return EFI_UNSUPPORTED;
}

static efi_status_t get_variable_interruptible(efi_char16_t *name,
					       efi_guid_t *vendor,
					       u32 *attr,
					       unsigned long *data_size,
					       void *data)
{
	if (in_panic)
		return efi.get_variable(name, vendor, attr, data_size, data);
	return execute_service(do_get_variable_interruptible,
			       NULL, name, vendor, attr, data_size, data);
}

static struct efivars interruptible_efivars;

static int efi_interruptible_panic_notifier_call(
	struct notifier_block *notifier,
	unsigned long what, void *data)
{
	in_panic = true;
	return NOTIFY_DONE;
}

static struct notifier_block panic_nb = {
	.notifier_call = efi_interruptible_panic_notifier_call,
	.priority = 100,
};

static struct task_struct *efi_kworker_task;
static struct efivar_operations interruptible_ops;
static __init int efi_interruptible_init(void)
{
	int ret;

	efi_kworker_task = kthread_create(kthread_worker_fn, &efi_kworker,
					  "efi_kthread");
	if (IS_ERR(efi_kworker_task)) {
		pr_err("efi_interruptible: Failed to create kthread\n");
		ret = PTR_ERR(efi_kworker_task);
		efi_kworker_task = NULL;
		return ret;
	}

	efi_kworker_task->mm = mm_alloc();
	efi_kworker_task->active_mm = efi_kworker_task->mm;
	efi_kworker_task->mm->pgd = efi_get_pgd();
	wake_up_process(efi_kworker_task);

	atomic_notifier_chain_register(&panic_notifier_list, &panic_nb);

	interruptible_ops.get_variable = get_variable_interruptible;
	interruptible_ops.set_variable = set_variable_interruptible;
	interruptible_ops.set_variable_nonblocking = non_blocking_not_allowed;
	interruptible_ops.get_next_variable = get_next_variable_interruptible;
	interruptible_ops.query_variable_store = efi_query_variable_store;
	return efivars_register(&interruptible_efivars, &interruptible_ops,
				efivars_kobject());
}

static void __exit efi_interruptible_exit(void)
{
	efivars_unregister(&interruptible_efivars);
	atomic_notifier_chain_unregister(&panic_notifier_list, &panic_nb);
	if (efi_kworker_task) {
		kthread_stop(efi_kworker_task);
		mmdrop(efi_kworker_task->mm);
	}
}

module_init(efi_interruptible_init);
module_exit(efi_interruptible_exit);

MODULE_AUTHOR("Sylvain Chouleur <sylvain.chouleur@intel.com>");
MODULE_LICENSE("GPL");
