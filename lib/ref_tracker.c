// SPDX-License-Identifier: GPL-2.0-or-later

#define pr_fmt(fmt) "ref_tracker: " fmt

#include <linux/export.h>
#include <linux/list_sort.h>
#include <linux/ref_tracker.h>
#include <linux/slab.h>
#include <linux/stacktrace.h>
#include <linux/stackdepot.h>

#define REF_TRACKER_STACK_ENTRIES 16
#define STACK_BUF_SIZE 1024

struct ref_tracker {
	struct list_head	head;   /* anchor into dir->list or dir->quarantine */
	bool			dead;
	depot_stack_handle_t	alloc_stack_handle;
	depot_stack_handle_t	free_stack_handle;
};

static int ref_tracker_cmp(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct ref_tracker *ta = list_entry(a, struct ref_tracker, head);
	struct ref_tracker *tb = list_entry(b, struct ref_tracker, head);

	return ta->alloc_stack_handle - tb->alloc_stack_handle;
}

struct ostream {
	char *buf;
	int size, used;
};

#define pr_ostream(stream, fmt, args...) \
({ \
	struct ostream *_s = (stream); \
\
	if (!_s->buf) { \
		pr_err(fmt, ##args); \
	} else { \
		int ret, len = _s->size - _s->used; \
		ret = snprintf(_s->buf + _s->used, len, pr_fmt(fmt), ##args); \
		_s->used += min(ret, len); \
	} \
})

static void
__ref_tracker_dir_pr_ostream(struct ref_tracker_dir *dir,
			     unsigned int display_limit, struct ostream *s)
{
	unsigned int i = 0, count = 0, total = 0;
	struct ref_tracker *tracker;
	depot_stack_handle_t stack;
	char *sbuf;

	lockdep_assert_held(&dir->lock);

	if (refcount_read(&dir->untracked) > 1)
		pr_ostream(s, "%s@%pK has %d untracked references\n",
			   dir->name, dir, refcount_read(&dir->untracked) - 1);

	if (list_empty(&dir->list))
		return;

	sbuf = kmalloc(STACK_BUF_SIZE, GFP_NOWAIT | __GFP_NOWARN);

	list_for_each_entry(tracker, &dir->list, head)
		++total;

	list_sort(NULL, &dir->list, ref_tracker_cmp);

	list_for_each_entry(tracker, &dir->list, head) {
		if (!count++)
			stack = tracker->alloc_stack_handle;
		if (stack == tracker->alloc_stack_handle &&
		    !list_is_last(&tracker->head, &dir->list))
			continue;
		if (i++ >= display_limit)
			continue;

		if (sbuf && !stack_depot_snprint(stack, sbuf, STACK_BUF_SIZE, 4))
			sbuf[0] = 0;
		pr_ostream(s, "%s@%pK has %d/%d users at\n%s\n",
			   dir->name, dir, count, total, sbuf);
		count = 0;
	}
	if (i > display_limit)
		pr_ostream(s, "%s@%pK skipped %d/%d reports with %d unique stacks.\n",
			   dir->name, dir, count, total, i - display_limit);

	kfree(sbuf);
}

void __ref_tracker_dir_print(struct ref_tracker_dir *dir,
			   unsigned int display_limit)
{
	struct ostream os = {};

	__ref_tracker_dir_pr_ostream(dir, display_limit, &os);
}
EXPORT_SYMBOL(__ref_tracker_dir_print);

void ref_tracker_dir_print(struct ref_tracker_dir *dir,
			   unsigned int display_limit)
{
	unsigned long flags;

	spin_lock_irqsave(&dir->lock, flags);
	__ref_tracker_dir_print(dir, display_limit);
	spin_unlock_irqrestore(&dir->lock, flags);
}
EXPORT_SYMBOL(ref_tracker_dir_print);

int ref_tracker_dir_snprint(struct ref_tracker_dir *dir, char *buf, size_t size)
{
	struct ostream os = { .buf = buf, .size = size };
	unsigned long flags;

	spin_lock_irqsave(&dir->lock, flags);
	__ref_tracker_dir_pr_ostream(dir, 16, &os);
	spin_unlock_irqrestore(&dir->lock, flags);

	return os.used;
}
EXPORT_SYMBOL(ref_tracker_dir_snprint);

void ref_tracker_dir_exit(struct ref_tracker_dir *dir)
{
	struct ref_tracker *tracker, *n;
	unsigned long flags;
	bool leak = false;

	spin_lock_irqsave(&dir->lock, flags);
	list_for_each_entry_safe(tracker, n, &dir->quarantine, head) {
		list_del(&tracker->head);
		kfree(tracker);
		dir->quarantine_avail++;
	}
	if (!list_empty(&dir->list)) {
		__ref_tracker_dir_print(dir, 16);
		leak = true;
		list_for_each_entry_safe(tracker, n, &dir->list, head) {
			list_del(&tracker->head);
			kfree(tracker);
		}
	}
	spin_unlock_irqrestore(&dir->lock, flags);
	WARN_ON_ONCE(leak);
	WARN_ON_ONCE(refcount_read(&dir->untracked) != 1);
}
EXPORT_SYMBOL(ref_tracker_dir_exit);

int ref_tracker_alloc(struct ref_tracker_dir *dir,
		      struct ref_tracker **trackerp,
		      gfp_t gfp)
{
	unsigned long entries[REF_TRACKER_STACK_ENTRIES];
	struct ref_tracker *tracker;
	unsigned int nr_entries;
	unsigned long flags;

	gfp |= __GFP_NOWARN;
	*trackerp = tracker = kzalloc(sizeof(*tracker), gfp);
	if (unlikely(!tracker)) {
		refcount_inc(&dir->untracked);
		return -ENOMEM;
	}
	nr_entries = stack_trace_save(entries, ARRAY_SIZE(entries), 1);
	nr_entries = filter_irq_stacks(entries, nr_entries);
	tracker->alloc_stack_handle = stack_depot_save(entries, nr_entries, gfp);

	spin_lock_irqsave(&dir->lock, flags);
	list_add(&tracker->head, &dir->list);
	spin_unlock_irqrestore(&dir->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(ref_tracker_alloc);

int ref_tracker_free(struct ref_tracker_dir *dir,
		     struct ref_tracker **trackerp)
{
	unsigned long entries[REF_TRACKER_STACK_ENTRIES];
	struct ref_tracker *tracker = *trackerp;
	depot_stack_handle_t stack_handle;
	unsigned int nr_entries;
	unsigned long flags;

	if (!tracker) {
		refcount_dec(&dir->untracked);
		return -EEXIST;
	}
	nr_entries = stack_trace_save(entries, ARRAY_SIZE(entries), 1);
	nr_entries = filter_irq_stacks(entries, nr_entries);
	stack_handle = stack_depot_save(entries, nr_entries,
					GFP_NOWAIT | __GFP_NOWARN);

	spin_lock_irqsave(&dir->lock, flags);
	if (tracker->dead) {
		pr_err("reference already released.\n");
		if (tracker->alloc_stack_handle) {
			pr_err("allocated in:\n");
			stack_depot_print(tracker->alloc_stack_handle);
		}
		if (tracker->free_stack_handle) {
			pr_err("freed in:\n");
			stack_depot_print(tracker->free_stack_handle);
		}
		spin_unlock_irqrestore(&dir->lock, flags);
		WARN_ON_ONCE(1);
		return -EINVAL;
	}
	tracker->dead = true;

	tracker->free_stack_handle = stack_handle;

	list_move_tail(&tracker->head, &dir->quarantine);
	if (!dir->quarantine_avail) {
		tracker = list_first_entry(&dir->quarantine, struct ref_tracker, head);
		list_del(&tracker->head);
	} else {
		dir->quarantine_avail--;
		tracker = NULL;
	}
	spin_unlock_irqrestore(&dir->lock, flags);

	kfree(tracker);
	return 0;
}
EXPORT_SYMBOL_GPL(ref_tracker_free);
