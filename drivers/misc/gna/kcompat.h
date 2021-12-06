/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright(c) 2017-2021 Intel Corporation */

#ifndef GNA_KCOMPAT_H
#define GNA_KCOMPAT_H

#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0))

#include <linux/rwsem.h>

#ifndef mmap_read_lock
#define mmap_read_lock(mm) _kc_mmap_read_lock(mm)
static inline void _kc_mmap_read_lock(struct mm_struct *mm)
{
	down_read(&mm->mmap_sem);
}
#endif

#ifndef mmap_read_unlock
#define mmap_read_unlock(mm) _kc_mmap_read_unlock(mm)
static inline void _kc_mmap_read_unlock(struct mm_struct *mm)
{
	up_read(&mm->mmap_sem);
}
#endif

#else /* LINUX_VERSION < 5.8.0 */

#include <linux/mmap_lock.h>

#endif /* LINUX_VERSION < 5.8.0 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 7, 0))

#ifndef FIELD_MAX
#define FIELD_MAX(_mask)						\
	({								\
		__BF_FIELD_CHECK(_mask, 0ULL, 0ULL, "FIELD_MAX: ");	\
		(typeof(_mask))((_mask) >> __bf_shf(_mask));		\
	})
#endif

#endif /* LINUX_VERSION < 5.7.0 */


#ifndef kc_get_user_pages_remote
#define kc_get_user_pages_remote(tsk, mm, start, nr_pages,	  \
				gup_flags, pages, vmas, locked)	  \
	_kc_get_user_pages_remote(tsk, mm, start, nr_pages,	  \
				gup_flags, pages, vmas, locked)

static inline long _kc_get_user_pages_remote(struct task_struct *tsk, struct mm_struct *mm,
					unsigned long start, unsigned long nr_pages,
					unsigned int gup_flags, struct page **pages,
					struct vm_area_struct **vmas, int *locked)
{
	return get_user_pages_remote(
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0))
		tsk,
#endif /* LINUX_VERSION < 5.9.0 */
		mm, start, nr_pages,
		gup_flags, pages, vmas, locked);
}
#endif

#ifndef kc_vm_map_ram
#define kc_vm_map_ram(pages, count, node) _kc_vm_map_ram(pages, count, node)
static inline void *_kc_vm_map_ram(struct page **pages, unsigned int count,
				int node)
{
	return vm_map_ram(
		pages, count, node
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0))
		, PAGE_KERNEL
#endif /* LINUX_VERSION < 5.8.0 */
		);
}

#endif

#endif /* GNA_KCOMPAT_H */
