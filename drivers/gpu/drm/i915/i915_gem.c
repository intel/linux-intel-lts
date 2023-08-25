/*
 * Copyright © 2008-2015 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *
 */

#include <drm/drm_vma_manager.h>
#include <linux/dma-fence-array.h>
#include <linux/kthread.h>
#include <linux/dma-resv.h>
#include <linux/shmem_fs.h>
#include <linux/slab.h>
#include <linux/stop_machine.h>
#include <linux/swap.h>
#include <linux/pci.h>
#include <linux/dma-buf.h>
#include <linux/mman.h>
#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
#include <linux/pid.h>
#include <linux/async.h>
#include <linux/sched/mm.h>
#include "../drm_internal.h"
#endif

#include "display/intel_display.h"
#include "display/intel_frontbuffer.h"

#include "gem/i915_gem_clflush.h"
#include "gem/i915_gem_context.h"
#include "gem/i915_gem_ioctls.h"
#include "gem/i915_gem_mman.h"
#include "gem/i915_gem_region.h"
#include "gt/intel_engine_user.h"
#include "gt/intel_gt.h"
#include "gt/intel_gt_pm.h"
#include "gt/intel_workarounds.h"

#include "i915_drv.h"
#include "i915_trace.h"
#include "i915_vgpu.h"

#include "intel_pm.h"

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
struct per_file_obj_mem_info {
	int num_obj;
	int num_obj_shared;
	int num_obj_private;
	int num_obj_gtt_bound;
	int num_obj_purged;
	int num_obj_purgeable;
	int num_obj_allocated;
	int num_obj_fault_mappable;
	int num_obj_stolen;
	size_t gtt_space_allocated_shared;
	size_t gtt_space_allocated_priv;
	size_t phys_space_allocated_shared;
	size_t phys_space_allocated_priv;
	size_t phys_space_purgeable;
	size_t phys_space_shared_proportion;
	size_t fault_mappable_size;
	size_t stolen_space_allocated;
	char *process_name;
};

struct name_entry {
	struct list_head head;
	struct drm_hash_item hash_item;
};

struct pid_stat_entry {
	struct list_head head;
	struct list_head namefree;
	struct drm_open_hash namelist;
	struct per_file_obj_mem_info stats;
	struct pid *tgid;
	int pid_num;
};

struct drm_i915_obj_virt_addr {
	struct list_head head;
	unsigned long user_virt_addr;
};

struct drm_i915_obj_pid_info {
	struct list_head head;
	pid_t tgid;
	int open_handle_count;
	struct list_head virt_addr_head;
};

struct get_obj_stats_buf {
	struct pid_stat_entry *entry;
	struct drm_i915_error_state_buf *m;
};

#define err_printf(e, ...) i915_error_printf(e, __VA_ARGS__)
#define err_puts(e, s) i915_error_puts(e, s)

static const char *get_tiling_flag(struct drm_i915_gem_object *obj)
{
	switch (i915_gem_object_get_tiling(obj)) {
	default:
	case I915_TILING_NONE:
		return " ";
	case I915_TILING_X:
		return "X";
	case I915_TILING_Y:
		return "Y";
	}
}

/*
 * If this mmput call is the last one, it will tear down the mmaps of the
 * process and calls drm_gem_vm_close(), which leads deadlock on i915 mutex.
 * Instead, asynchronously schedule mmput function here, to avoid recursive
 * calls to acquire i915_mutex.
 */
static void async_mmput_func(void *data, async_cookie_t cookie)
{
	struct mm_struct *mm = data;

	mmput(mm);
}

static void async_mmput(struct mm_struct *mm)
{
	async_schedule(async_mmput_func, mm);
}

int i915_get_pid_cmdline(struct task_struct *task, char *buffer)
{
	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(task);

	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;

	len = mm->arg_end - mm->arg_start;

	if (len > PAGE_SIZE)
		len = PAGE_SIZE;

	res = access_process_vm(task, mm->arg_start, buffer, len, 0);
	if (res < 0) {
		async_mmput(mm);
		return res;
	}

	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE)
		buffer[res-1] = '\0';
out_mm:
	async_mmput(mm);
out:
	return 0;
}

static int i915_obj_get_shmem_pages_alloced(struct drm_i915_gem_object *obj)
{
	if (obj->base.filp) {
		struct inode *inode = file_inode(obj->base.filp);

		if (!inode)
			return 0;
		return inode->i_mapping->nrpages;
	}
	return 0;
}

int i915_gem_obj_insert_pid(struct drm_i915_gem_object *obj)
{
	int found = 0;
	struct drm_i915_obj_pid_info *entry;
	pid_t current_tgid = task_tgid_nr(current);

	mutex_lock(&obj->base.dev->struct_mutex);

	list_for_each_entry(entry, &obj->pid_info, head) {
		if (entry->tgid == current_tgid) {
			entry->open_handle_count++;
			found = 1;
			break;
		}
	}
	if (found == 0) {
		entry = kzalloc(sizeof(*entry), GFP_KERNEL);
		if (entry == NULL) {
			DRM_ERROR("alloc failed\n");
			mutex_unlock(&obj->base.dev->struct_mutex);
			return -ENOMEM;
		}
		entry->tgid = current_tgid;
		entry->open_handle_count = 1;
		INIT_LIST_HEAD(&entry->virt_addr_head);
		list_add_tail(&entry->head, &obj->pid_info);
	}

	mutex_unlock(&obj->base.dev->struct_mutex);
	return 0;
}

void i915_gem_obj_remove_all_pids(struct drm_i915_gem_object *obj)
{
	struct drm_i915_obj_pid_info *pid_entry, *pid_next;
	struct drm_i915_obj_virt_addr *virt_entry, *virt_next;

	list_for_each_entry_safe(pid_entry, pid_next, &obj->pid_info, head) {
		list_for_each_entry_safe(virt_entry,
				virt_next,
				&pid_entry->virt_addr_head,
				head) {
			list_del(&virt_entry->head);
			kfree(virt_entry);
		}
		list_del(&pid_entry->head);
		kfree(pid_entry);
	}
}

int i915_obj_insert_virt_addr(struct drm_i915_gem_object *obj,
		unsigned long addr,
		bool is_map_gtt,
		bool is_mutex_locked)
{
	struct drm_i915_obj_pid_info *pid_entry;
	pid_t current_tgid = task_tgid_nr(current);
	int ret = 0, found = 0;

	if (is_map_gtt)
		addr |= 1;

	if (!is_mutex_locked) {
		if (mutex_lock_interruptible(&obj->base.dev->struct_mutex))
			return -EINTR;
	}

	list_for_each_entry(pid_entry, &obj->pid_info, head) {
		if (pid_entry->tgid == current_tgid) {
			struct drm_i915_obj_virt_addr *virt_entry, *new_entry;

			list_for_each_entry(virt_entry,
					&pid_entry->virt_addr_head,
					head) {
				if (virt_entry->user_virt_addr == addr) {
					found = 1;
					break;
				}
			}
			if (found)
				break;
			new_entry = kzalloc(sizeof(*new_entry), GFP_KERNEL);
			if (new_entry == NULL) {
				DRM_ERROR("alloc failed\n");
				ret = -ENOMEM;
				goto out;
			}
			new_entry->user_virt_addr = addr;
			list_add_tail(&new_entry->head,
					&pid_entry->virt_addr_head);
			break;
		}
	}

out:
	if (!is_mutex_locked)
		mutex_unlock(&obj->base.dev->struct_mutex);

	return ret;
}

static int i915_obj_virt_addr_is_invalid(struct drm_gem_object *obj,
		struct pid *tgid, unsigned long addr)
{
	struct task_struct *task;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	int locked, ret = 0;

	task = get_pid_task(tgid, PIDTYPE_PID);
	if (task == NULL) {
		DRM_DEBUG("null task for tgid=%d\n", pid_nr(tgid));
		return -EINVAL;
	}

	mm = get_task_mm(task);
	if (mm == NULL) {
		DRM_DEBUG("null mm for tgid=%d\n", pid_nr(tgid));
		ret = -EINVAL;
		goto out_task;
	}

	locked = down_read_trylock(&mm->mmap_lock);
	if (!locked)
		goto out_mm;

	vma = find_vma(mm, addr);
	if (vma) {
		if (addr & 1) { /* mmap_gtt case */
			if (vma->vm_pgoff*PAGE_SIZE == (unsigned long)
					drm_vma_node_offset_addr(&obj->vma_node))
				ret = 0;
			else
				ret = -EINVAL;
		} else { /* mmap case */
			if (vma->vm_file == obj->filp)
				ret = 0;
			else
				ret = -EINVAL;
		}
	} else
		ret = -EINVAL;

	up_read(&mm->mmap_lock);

out_mm:
	async_mmput(mm);
out_task:
	put_task_struct(task);
	return ret;
}

static void i915_obj_pidarray_validate(struct drm_gem_object *gem_obj)
{
	struct drm_i915_gem_object *obj = to_intel_bo(gem_obj);
	struct drm_device *dev = gem_obj->dev;
	struct drm_i915_obj_virt_addr *virt_entry, *virt_next;
	struct drm_i915_obj_pid_info *pid_entry, *pid_next;
	struct drm_file *file;
	struct drm_i915_file_private *file_priv;
	struct pid *tgid;
	int pid_num, present;

	/*
	 * Run a sanity check on pid_array. All entries in pid_array should
	 * be subset of the drm filelist pid entries.
	 */
	list_for_each_entry_safe(pid_entry, pid_next, &obj->pid_info, head) {
		if (pid_next == NULL) {
			DRM_ERROR(
					"Invalid pid info. obj:%p, size:%zdK, tiling:%s, stolen:%s, name:%d, handle_count=%d\n",
					&obj->base, obj->base.size/1024,
					get_tiling_flag(obj),
					obj->stolen ? "Y" : "N", obj->base.name,
					obj->base.handle_count);
			break;
		}

		present = 0;
		list_for_each_entry(file, &dev->filelist, lhead) {
			file_priv = file->driver_priv;
			tgid = file_priv->tgid;
			pid_num = pid_nr(tgid);

			if (pid_num == pid_entry->tgid) {
				present = 1;
				break;
			}
		}
		if (present == 0) {
			DRM_DEBUG("stale_tgid=%d\n", pid_entry->tgid);
			list_for_each_entry_safe(virt_entry, virt_next,
					&pid_entry->virt_addr_head,
					head) {
				list_del(&virt_entry->head);
				kfree(virt_entry);
			}
			list_del(&pid_entry->head);
			kfree(pid_entry);
		} else {
			/* Validate the virtual address list */
			struct task_struct *task =
				get_pid_task(tgid, PIDTYPE_PID);
			if (task == NULL)
				continue;

			list_for_each_entry_safe(virt_entry, virt_next,
					&pid_entry->virt_addr_head,
					head) {
				if (i915_obj_virt_addr_is_invalid(gem_obj, tgid,
							virt_entry->user_virt_addr)) {
					DRM_DEBUG("stale_addr=%ld\n",
							virt_entry->user_virt_addr);
					list_del(&virt_entry->head);
					kfree(virt_entry);
				}
			}
			put_task_struct(task);
		}
	}
}

static int i915_obj_find_insert_in_hash(struct drm_i915_gem_object *obj,
		struct pid_stat_entry *pid_entry,
		bool *found)
{
	struct drm_hash_item *hash_item;
	int ret;

	ret = drm_ht_find_item(&pid_entry->namelist,
			(unsigned long)&obj->base, &hash_item);
	/* Not found, insert in hash */
	if (ret) {
		struct name_entry *entry =
			kzalloc(sizeof(*entry), GFP_NOWAIT);
		if (entry == NULL) {
			DRM_ERROR("alloc failed\n");
			return -ENOMEM;
		}
		entry->hash_item.key = (unsigned long)&obj->base;
		drm_ht_insert_item(&pid_entry->namelist,
				&entry->hash_item);
		list_add_tail(&entry->head, &pid_entry->namefree);
		*found = false;
	} else
		*found = true;

	return 0;
}

static int i915_obj_shared_count(struct drm_i915_gem_object *obj,
		struct pid_stat_entry *pid_entry,
		bool *discard)
{
	struct drm_i915_obj_pid_info *pid_info_entry;
	int ret, obj_shared_count = 0;

	/*
	 * The object can be shared among different processes by either flink
	 * or dma-buf mechanism, leading to shared count more than 1. For the
	 * objects not shared , return the shared count as 1.
	 * In case of shared dma-buf objects, there's a possibility that these
	 * may be external to i915. Detect this condition through
	 * 'import_attach' field.
	 */
	if (!obj->base.name && !obj->base.dma_buf)
		return 1;
	else if (obj->base.import_attach) {
		/* not our GEM obj */
		*discard = true;
		return 0;
	}

	ret = i915_obj_find_insert_in_hash(obj, pid_entry, discard);
	if (ret)
		return ret;

	list_for_each_entry(pid_info_entry, &obj->pid_info, head)
		obj_shared_count++;

	if (WARN_ON(obj_shared_count == 0))
		return -EINVAL;

	return obj_shared_count;
}

	static int
i915_describe_obj(struct get_obj_stats_buf *obj_stat_buf,
		struct drm_i915_gem_object *obj)
{
	struct pid_stat_entry *pid_entry = obj_stat_buf->entry;
	struct per_file_obj_mem_info *stats = &pid_entry->stats;
	int obj_shared_count = 0;

	bool discard = false;

	obj_shared_count = i915_obj_shared_count(obj, pid_entry, &discard);
	if (obj_shared_count < 0)
		return obj_shared_count;

	if (!discard && !obj->stolen &&
			(obj->mm.madv != __I915_MADV_PURGED) &&
			(i915_obj_get_shmem_pages_alloced(obj) != 0)) {
		if (obj_shared_count > 1)
			stats->phys_space_shared_proportion +=
				obj->base.size/obj_shared_count;
		else
			stats->phys_space_allocated_priv +=
				obj->base.size;
	}

	return 0;
}

	static int
i915_drm_gem_obj_info(int id, void *ptr, void *data)
{
	struct drm_i915_gem_object *obj = ptr;
	struct get_obj_stats_buf *obj_stat_buf = data;

	if (obj->pid_info.next == NULL) {
		DRM_ERROR(
				"Invalid pid info. obj:%p, size:%zdK, tiling:%s, stolen:%s, name:%d, handle_count=%d\n",
				&obj->base, obj->base.size/1024,
				get_tiling_flag(obj),
				obj->stolen ? "Y" : "N", obj->base.name,
				obj->base.handle_count);
		return 0;
	}

	return i915_describe_obj(obj_stat_buf, obj);
}

bool i915_gem_obj_bound_any(struct drm_i915_gem_object *o)
{
	struct i915_vma *vma;

	list_for_each_entry(vma, &o->vma.list, obj_link)
		if (drm_mm_node_allocated(&vma->node))
			return true;

	return false;
}

static int
i915_drm_gem_object_per_file_summary(int id, void *ptr, void *data)
{
	struct pid_stat_entry *pid_entry = data;
	struct drm_i915_gem_object *obj = ptr;
	struct per_file_obj_mem_info *stats = &pid_entry->stats;
	int obj_shared_count = 0;
	bool discard = false;

	if (obj->pid_info.next == NULL) {
		DRM_ERROR(
				"Invalid pid info. obj:%p, size:%zdK, tiling:%s, stolen:%s, name:%d, handle_count=%d\n",
				&obj->base, obj->base.size/1024,
				get_tiling_flag(obj),
				obj->stolen ? "Y" : "N", obj->base.name,
				obj->base.handle_count);
		return 0;
	}

	i915_obj_pidarray_validate(&obj->base);

	stats->num_obj++;

	obj_shared_count = i915_obj_shared_count(obj, pid_entry, &discard);
	if (obj_shared_count < 0)
		return obj_shared_count;

	if (discard)
		return 0;

	if (obj_shared_count > 1)
		stats->num_obj_shared++;
	else
		stats->num_obj_private++;

	if (i915_gem_obj_bound_any(obj)) {
		stats->num_obj_gtt_bound++;
		if (obj_shared_count > 1)
			stats->gtt_space_allocated_shared += obj->base.size;
		else
			stats->gtt_space_allocated_priv += obj->base.size;
	}

	if (obj->stolen) {
		stats->num_obj_stolen++;
		stats->stolen_space_allocated += obj->base.size;
	} else if (obj->mm.madv == __I915_MADV_PURGED) {
		stats->num_obj_purged++;
	} else if (obj->mm.madv == I915_MADV_DONTNEED) {
		stats->num_obj_purgeable++;
		stats->num_obj_allocated++;
		if (i915_obj_get_shmem_pages_alloced(obj) != 0) {
			stats->phys_space_purgeable += obj->base.size;
			if (obj_shared_count > 1) {
				stats->phys_space_allocated_shared +=
					obj->base.size;
				stats->phys_space_shared_proportion +=
					obj->base.size/obj_shared_count;
			} else
				stats->phys_space_allocated_priv +=
					obj->base.size;
		} else
			WARN_ON(1);
	} else if (i915_obj_get_shmem_pages_alloced(obj) != 0) {
		stats->num_obj_allocated++;
		if (obj_shared_count > 1) {
			stats->phys_space_allocated_shared +=
				obj->base.size;
			stats->phys_space_shared_proportion +=
				obj->base.size/obj_shared_count;
		} else
			stats->phys_space_allocated_priv += obj->base.size;
	}
	return 0;
}

static int
__i915_get_drm_clients_info(struct drm_i915_error_state_buf *m,
		struct drm_device *dev)
{
	struct drm_file *file;
	struct drm_i915_private *i915 = to_i915(dev);
	struct name_entry *entry, *next;
	struct pid_stat_entry *pid_entry, *temp_entry;
	struct pid_stat_entry *new_pid_entry, *new_temp_entry;
	struct list_head per_pid_stats, sorted_pid_stats;
	int ret = 0;
	size_t total_shared_prop_space = 0, total_priv_space = 0;

	INIT_LIST_HEAD(&per_pid_stats);
	INIT_LIST_HEAD(&sorted_pid_stats);

	err_puts(m,
		"\n\n  pid   Total  Shared  Priv   Purgeable  Alloced  SharedPHYsize   SharedPHYprop    PrivPHYsize   PurgeablePHYsize   process\n");

	list_for_each_entry(file, &dev->filelist, lhead) {
		struct pid *tgid;
		struct drm_i915_file_private *file_priv = file->driver_priv;
		int pid_num, found = 0;

		tgid = file_priv->tgid;
		pid_num = pid_nr(tgid);

		list_for_each_entry(pid_entry, &per_pid_stats, head) {
			if (pid_entry->pid_num == pid_num) {
				found = 1;
				break;
			}
		}

		if (!found) {
			struct pid_stat_entry *new_entry =
				kzalloc(sizeof(*new_entry), GFP_KERNEL);
			if (new_entry == NULL) {
				DRM_ERROR("alloc failed\n");
				ret = -ENOMEM;
				break;
			}
			new_entry->tgid = tgid;
			new_entry->pid_num = pid_num;
			ret = drm_ht_create(&new_entry->namelist,
					DRM_MAGIC_HASH_ORDER);
			if (ret) {
				kfree(new_entry);
				break;
			}

			list_add_tail(&new_entry->head, &per_pid_stats);
			INIT_LIST_HEAD(&new_entry->namefree);
			new_entry->stats.process_name = file_priv->process_name;
			pid_entry = new_entry;
		}

		spin_lock(&file->table_lock);
		ret = idr_for_each(&file->object_idr,
				&i915_drm_gem_object_per_file_summary, pid_entry);
		spin_unlock(&file->table_lock);
		if (ret)
			break;
	}

	list_for_each_entry_safe(pid_entry, temp_entry, &per_pid_stats, head) {
		if (list_empty(&sorted_pid_stats)) {
			list_del(&pid_entry->head);
			list_add_tail(&pid_entry->head, &sorted_pid_stats);
			continue;
		}

		list_for_each_entry_safe(new_pid_entry, new_temp_entry,
				&sorted_pid_stats, head) {
			int prev_space =
				pid_entry->stats.phys_space_shared_proportion +
				pid_entry->stats.phys_space_allocated_priv;
			int new_space =
				new_pid_entry->stats.phys_space_shared_proportion +
				new_pid_entry->stats.phys_space_allocated_priv;
			if (prev_space > new_space) {
				list_del(&pid_entry->head);
				list_add_tail(&pid_entry->head,
						&new_pid_entry->head);
				break;
			}
			if (list_is_last(&new_pid_entry->head,
						&sorted_pid_stats)) {
				list_del(&pid_entry->head);
				list_add_tail(&pid_entry->head,
						&sorted_pid_stats);
			}
		}
	}

	list_for_each_entry_safe(pid_entry, temp_entry,
			&sorted_pid_stats, head) {
		struct task_struct *task = get_pid_task(pid_entry->tgid,
				PIDTYPE_PID);
		err_printf(m,
				"%5d %6d %6d %6d %9d %8d %14zdK %14zdK %14zdK  %14zdK     %s",
				pid_entry->pid_num,
				pid_entry->stats.num_obj,
				pid_entry->stats.num_obj_shared,
				pid_entry->stats.num_obj_private,
				pid_entry->stats.num_obj_purgeable,
				pid_entry->stats.num_obj_allocated,
				pid_entry->stats.phys_space_allocated_shared/1024,
				pid_entry->stats.phys_space_shared_proportion/1024,
				pid_entry->stats.phys_space_allocated_priv/1024,
				pid_entry->stats.phys_space_purgeable/1024,
				pid_entry->stats.process_name);

		if (task == NULL)
			err_puts(m, "*\n");
		else
			err_puts(m, "\n");

		total_shared_prop_space +=
			pid_entry->stats.phys_space_shared_proportion/1024;
		total_priv_space +=
			pid_entry->stats.phys_space_allocated_priv/1024;
		list_del(&pid_entry->head);

		list_for_each_entry_safe(entry, next,
				&pid_entry->namefree, head) {
			list_del(&entry->head);
			drm_ht_remove_item(&pid_entry->namelist,
					&entry->hash_item);
			kfree(entry);
		}
		drm_ht_remove(&pid_entry->namelist);
		kfree(pid_entry);
		if (task)
			put_task_struct(task);
	}

	err_puts(m,
			"\t\t\t\t\t\t\t\t--------------\t-------------\t--------\n");
	err_printf(m,
			"\t\t\t\t\t\t\t\t%13zdK\t%12zdK\tTotal\n",
			total_shared_prop_space, total_priv_space);

	err_printf(m, "\nTotal used GFX Shmem Physical space %8zdK\n",
			i915->mm.phys_mem_total/1024);

	if (ret)
		return ret;

	if (m->bytes == 0 && m->err)
		return m->err;

	return 0;
}

#define NUM_SPACES 100
#define INITIAL_SPACES_STR(x) #x
#define SPACES_STR(x) INITIAL_SPACES_STR(x)

static int
__i915_gem_get_obj_info(struct drm_i915_error_state_buf *m,
		struct drm_device *dev, struct pid *tgid)
{
	struct drm_file *file;
	struct drm_i915_file_private *file_priv_reqd = NULL;
	int bytes_copy, ret = 0;
	struct pid_stat_entry pid_entry;
	struct name_entry *entry, *next;

	pid_entry.stats.phys_space_shared_proportion = 0;
	pid_entry.stats.phys_space_allocated_priv = 0;
	pid_entry.tgid = tgid;
	pid_entry.pid_num = pid_nr(tgid);
	ret = drm_ht_create(&pid_entry.namelist, DRM_MAGIC_HASH_ORDER);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&pid_entry.namefree);

	/*
	 * Fill up initial few bytes with spaces, to insert summary data later
	 * on
	 */
	err_printf(m, "%"SPACES_STR(NUM_SPACES)"s\n", " ");

	list_for_each_entry(file, &dev->filelist, lhead) {
		struct drm_i915_file_private *file_priv = file->driver_priv;
		struct get_obj_stats_buf obj_stat_buf;

		obj_stat_buf.entry = &pid_entry;
		obj_stat_buf.m = m;

		if (file_priv->tgid != tgid)
			continue;

		file_priv_reqd = file_priv;
		spin_lock(&file->table_lock);
		ret = idr_for_each(&file->object_idr,
				&i915_drm_gem_obj_info, &obj_stat_buf);
		spin_unlock(&file->table_lock);
		if (ret)
			break;
	}

	if (file_priv_reqd) {
		int space_remaining;

		/* Reset the bytes counter to buffer beginning */
		bytes_copy = m->bytes;
		m->bytes = 0;

		err_printf(m, "\n  PID    GfxMem   Process\n");
		err_printf(m, "%5d %8zdK ", pid_nr(file_priv_reqd->tgid),
				(pid_entry.stats.phys_space_shared_proportion +
				 pid_entry.stats.phys_space_allocated_priv)/1024);

		space_remaining = NUM_SPACES - m->bytes - 1;
		if (strlen(file_priv_reqd->process_name) > space_remaining)
			file_priv_reqd->process_name[space_remaining] = '\0';

		err_printf(m, "%s\n", file_priv_reqd->process_name);

		/* Reinstate the previous saved value of bytes counter */
		m->bytes = bytes_copy;
	} else
		WARN(1, "drm file corresponding to tgid:%d not found\n",
				pid_nr(tgid));

	list_for_each_entry_safe(entry, next,
			&pid_entry.namefree, head) {
		list_del(&entry->head);
		drm_ht_remove_item(&pid_entry.namelist,
				&entry->hash_item);
		kfree(entry);
	}
	drm_ht_remove(&pid_entry.namelist);

	if (ret)
		return ret;
	if (m->bytes == 0 && m->err)
		return m->err;
	return 0;
}

int i915_get_drm_clients_info(struct drm_i915_error_state_buf *m,
		struct drm_device *dev)
{
	int ret = 0;

	/*
	 * Protect the access to global drm resources such as filelist. Protect
	 * against their removal under our noses, while in use.
	 */
	mutex_lock(&drm_global_mutex);
	if (mutex_lock_interruptible(&dev->filelist_mutex)) {
		mutex_unlock(&drm_global_mutex);
		return -EINTR;
	}

	ret = __i915_get_drm_clients_info(m, dev);

	mutex_unlock(&dev->filelist_mutex);
	mutex_unlock(&drm_global_mutex);

	return ret;
}

int i915_gem_get_obj_info(struct drm_i915_error_state_buf *m,
		struct drm_device *dev, struct pid *tgid)
{
	int ret = 0;

	if (mutex_lock_interruptible(&dev->filelist_mutex))
		return -EINTR;

	ret = __i915_gem_get_obj_info(m, dev, tgid);

	mutex_unlock(&dev->filelist_mutex);

	return ret;
}
#endif

static int
insert_mappable_node(struct i915_ggtt *ggtt, struct drm_mm_node *node, u32 size)
{
	int err;

	err = mutex_lock_interruptible(&ggtt->vm.mutex);
	if (err)
		return err;

	memset(node, 0, sizeof(*node));
	err = drm_mm_insert_node_in_range(&ggtt->vm.mm, node,
					  size, 0, I915_COLOR_UNEVICTABLE,
					  0, ggtt->mappable_end,
					  DRM_MM_INSERT_LOW);

	mutex_unlock(&ggtt->vm.mutex);

	return err;
}

static void
remove_mappable_node(struct i915_ggtt *ggtt, struct drm_mm_node *node)
{
	mutex_lock(&ggtt->vm.mutex);
	drm_mm_remove_node(node);
	mutex_unlock(&ggtt->vm.mutex);
}

int
i915_gem_get_aperture_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct i915_ggtt *ggtt = &to_i915(dev)->ggtt;
	struct drm_i915_gem_get_aperture *args = data;
	struct i915_vma *vma;
	u64 pinned;

	if (mutex_lock_interruptible(&ggtt->vm.mutex))
		return -EINTR;

	pinned = ggtt->vm.reserved;
	list_for_each_entry(vma, &ggtt->vm.bound_list, vm_link)
		if (i915_vma_is_pinned(vma))
			pinned += vma->node.size;

	mutex_unlock(&ggtt->vm.mutex);

	args->aper_size = ggtt->vm.total;
	args->aper_available_size = args->aper_size - pinned;

	return 0;
}

int i915_gem_object_unbind(struct drm_i915_gem_object *obj,
			   unsigned long flags)
{
	struct intel_runtime_pm *rpm = &to_i915(obj->base.dev)->runtime_pm;
	LIST_HEAD(still_in_list);
	intel_wakeref_t wakeref;
	struct i915_vma *vma;
	int ret;

	if (list_empty(&obj->vma.list))
		return 0;

	/*
	 * As some machines use ACPI to handle runtime-resume callbacks, and
	 * ACPI is quite kmalloc happy, we cannot resume beneath the vm->mutex
	 * as they are required by the shrinker. Ergo, we wake the device up
	 * first just in case.
	 */
	wakeref = intel_runtime_pm_get(rpm);

try_again:
	ret = 0;
	spin_lock(&obj->vma.lock);
	while (!ret && (vma = list_first_entry_or_null(&obj->vma.list,
						       struct i915_vma,
						       obj_link))) {
		struct i915_address_space *vm = vma->vm;

		list_move_tail(&vma->obj_link, &still_in_list);
		if (!i915_vma_is_bound(vma, I915_VMA_BIND_MASK))
			continue;

		if (flags & I915_GEM_OBJECT_UNBIND_TEST) {
			ret = -EBUSY;
			break;
		}

		ret = -EAGAIN;
		if (!i915_vm_tryopen(vm))
			break;

		/* Prevent vma being freed by i915_vma_parked as we unbind */
		vma = __i915_vma_get(vma);
		spin_unlock(&obj->vma.lock);

		if (vma) {
			ret = -EBUSY;
			if (flags & I915_GEM_OBJECT_UNBIND_ACTIVE ||
			    !i915_vma_is_active(vma)) {
				if (flags & I915_GEM_OBJECT_UNBIND_VM_TRYLOCK) {
					if (mutex_trylock(&vma->vm->mutex)) {
						ret = __i915_vma_unbind(vma);
						mutex_unlock(&vma->vm->mutex);
					} else {
						ret = -EBUSY;
					}
				} else {
					ret = i915_vma_unbind(vma);
				}
			}

			__i915_vma_put(vma);
		}

		i915_vm_close(vm);
		spin_lock(&obj->vma.lock);
	}
	list_splice_init(&still_in_list, &obj->vma.list);
	spin_unlock(&obj->vma.lock);

	if (ret == -EAGAIN && flags & I915_GEM_OBJECT_UNBIND_BARRIER) {
		rcu_barrier(); /* flush the i915_vm_release() */
		goto try_again;
	}

	intel_runtime_pm_put(rpm, wakeref);

	return ret;
}

static int
shmem_pread(struct page *page, int offset, int len, char __user *user_data,
	    bool needs_clflush)
{
	char *vaddr;
	int ret;

	vaddr = kmap(page);

	if (needs_clflush)
		drm_clflush_virt_range(vaddr + offset, len);

	ret = __copy_to_user(user_data, vaddr + offset, len);

	kunmap(page);

	return ret ? -EFAULT : 0;
}

static int
i915_gem_shmem_pread(struct drm_i915_gem_object *obj,
		     struct drm_i915_gem_pread *args)
{
	unsigned int needs_clflush;
	unsigned int idx, offset;
	char __user *user_data;
	u64 remain;
	int ret;

	ret = i915_gem_object_lock_interruptible(obj, NULL);
	if (ret)
		return ret;

	ret = i915_gem_object_pin_pages(obj);
	if (ret)
		goto err_unlock;

	ret = i915_gem_object_prepare_read(obj, &needs_clflush);
	if (ret)
		goto err_unpin;

	i915_gem_object_finish_access(obj);
	i915_gem_object_unlock(obj);

	remain = args->size;
	user_data = u64_to_user_ptr(args->data_ptr);
	offset = offset_in_page(args->offset);
	for (idx = args->offset >> PAGE_SHIFT; remain; idx++) {
		struct page *page = i915_gem_object_get_page(obj, idx);
		unsigned int length = min_t(u64, remain, PAGE_SIZE - offset);

		ret = shmem_pread(page, offset, length, user_data,
				  needs_clflush);
		if (ret)
			break;

		remain -= length;
		user_data += length;
		offset = 0;
	}

	i915_gem_object_unpin_pages(obj);
	return ret;

err_unpin:
	i915_gem_object_unpin_pages(obj);
err_unlock:
	i915_gem_object_unlock(obj);
	return ret;
}

static inline bool
gtt_user_read(struct io_mapping *mapping,
	      loff_t base, int offset,
	      char __user *user_data, int length)
{
	void __iomem *vaddr;
	unsigned long unwritten;

	/* We can use the cpu mem copy function because this is X86. */
	vaddr = io_mapping_map_atomic_wc(mapping, base);
	unwritten = __copy_to_user_inatomic(user_data,
					    (void __force *)vaddr + offset,
					    length);
	io_mapping_unmap_atomic(vaddr);
	if (unwritten) {
		vaddr = io_mapping_map_wc(mapping, base, PAGE_SIZE);
		unwritten = copy_to_user(user_data,
					 (void __force *)vaddr + offset,
					 length);
		io_mapping_unmap(vaddr);
	}
	return unwritten;
}

static struct i915_vma *i915_gem_gtt_prepare(struct drm_i915_gem_object *obj,
					     struct drm_mm_node *node,
					     bool write)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct i915_ggtt *ggtt = &i915->ggtt;
	struct i915_vma *vma;
	struct i915_gem_ww_ctx ww;
	int ret;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	vma = ERR_PTR(-ENODEV);
	ret = i915_gem_object_lock(obj, &ww);
	if (ret)
		goto err_ww;

	ret = i915_gem_object_set_to_gtt_domain(obj, write);
	if (ret)
		goto err_ww;

	if (!i915_gem_object_is_tiled(obj))
		vma = i915_gem_object_ggtt_pin_ww(obj, &ww, NULL, 0, 0,
						  PIN_MAPPABLE |
						  PIN_NONBLOCK /* NOWARN */ |
						  PIN_NOEVICT);
	if (vma == ERR_PTR(-EDEADLK)) {
		ret = -EDEADLK;
		goto err_ww;
	} else if (!IS_ERR(vma)) {
		node->start = i915_ggtt_offset(vma);
		node->flags = 0;
	} else {
		ret = insert_mappable_node(ggtt, node, PAGE_SIZE);
		if (ret)
			goto err_ww;
		GEM_BUG_ON(!drm_mm_node_allocated(node));
		vma = NULL;
	}

	ret = i915_gem_object_pin_pages(obj);
	if (ret) {
		if (drm_mm_node_allocated(node)) {
			ggtt->vm.clear_range(&ggtt->vm, node->start, node->size);
			remove_mappable_node(ggtt, node);
		} else {
			i915_vma_unpin(vma);
		}
	}

err_ww:
	if (ret == -EDEADLK) {
		ret = i915_gem_ww_ctx_backoff(&ww);
		if (!ret)
			goto retry;
	}
	i915_gem_ww_ctx_fini(&ww);

	return ret ? ERR_PTR(ret) : vma;
}

static void i915_gem_gtt_cleanup(struct drm_i915_gem_object *obj,
				 struct drm_mm_node *node,
				 struct i915_vma *vma)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct i915_ggtt *ggtt = &i915->ggtt;

	i915_gem_object_unpin_pages(obj);
	if (drm_mm_node_allocated(node)) {
		ggtt->vm.clear_range(&ggtt->vm, node->start, node->size);
		remove_mappable_node(ggtt, node);
	} else {
		i915_vma_unpin(vma);
	}
}

static int
i915_gem_gtt_pread(struct drm_i915_gem_object *obj,
		   const struct drm_i915_gem_pread *args)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct i915_ggtt *ggtt = &i915->ggtt;
	intel_wakeref_t wakeref;
	struct drm_mm_node node;
	void __user *user_data;
	struct i915_vma *vma;
	u64 remain, offset;
	int ret = 0;

	wakeref = intel_runtime_pm_get(&i915->runtime_pm);

	vma = i915_gem_gtt_prepare(obj, &node, false);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto out_rpm;
	}

	user_data = u64_to_user_ptr(args->data_ptr);
	remain = args->size;
	offset = args->offset;

	while (remain > 0) {
		/* Operation in this page
		 *
		 * page_base = page offset within aperture
		 * page_offset = offset within page
		 * page_length = bytes to copy for this page
		 */
		u32 page_base = node.start;
		unsigned page_offset = offset_in_page(offset);
		unsigned page_length = PAGE_SIZE - page_offset;
		page_length = remain < page_length ? remain : page_length;
		if (drm_mm_node_allocated(&node)) {
			ggtt->vm.insert_page(&ggtt->vm,
					     i915_gem_object_get_dma_address(obj, offset >> PAGE_SHIFT),
					     node.start, I915_CACHE_NONE, 0);
		} else {
			page_base += offset & PAGE_MASK;
		}

		if (gtt_user_read(&ggtt->iomap, page_base, page_offset,
				  user_data, page_length)) {
			ret = -EFAULT;
			break;
		}

		remain -= page_length;
		user_data += page_length;
		offset += page_length;
	}

	i915_gem_gtt_cleanup(obj, &node, vma);
out_rpm:
	intel_runtime_pm_put(&i915->runtime_pm, wakeref);
	return ret;
}

/**
 * Reads data from the object referenced by handle.
 * @dev: drm device pointer
 * @data: ioctl data blob
 * @file: drm file pointer
 *
 * On error, the contents of *data are undefined.
 */
int
i915_gem_pread_ioctl(struct drm_device *dev, void *data,
		     struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_pread *args = data;
	struct drm_i915_gem_object *obj;
	int ret;

	/* PREAD is disallowed for all platforms after TGL-LP.  This also
	 * covers all platforms with local memory.
	 */
	if (GRAPHICS_VER(i915) >= 12 && !IS_TIGERLAKE(i915))
		return -EOPNOTSUPP;

	if (args->size == 0)
		return 0;

	if (!access_ok(u64_to_user_ptr(args->data_ptr),
		       args->size))
		return -EFAULT;

	obj = i915_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	/* Bounds check source.  */
	if (range_overflows_t(u64, args->offset, args->size, obj->base.size)) {
		ret = -EINVAL;
		goto out;
	}

	trace_i915_gem_object_pread(obj, args->offset, args->size);
	ret = -ENODEV;
	if (obj->ops->pread)
		ret = obj->ops->pread(obj, args);
	if (ret != -ENODEV)
		goto out;

	ret = i915_gem_object_wait(obj,
				   I915_WAIT_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
	if (ret)
		goto out;

	ret = i915_gem_shmem_pread(obj, args);
	if (ret == -EFAULT || ret == -ENODEV)
		ret = i915_gem_gtt_pread(obj, args);

out:
	i915_gem_object_put(obj);
	return ret;
}

/* This is the fast write path which cannot handle
 * page faults in the source data
 */

static inline bool
ggtt_write(struct io_mapping *mapping,
	   loff_t base, int offset,
	   char __user *user_data, int length)
{
	void __iomem *vaddr;
	unsigned long unwritten;

	/* We can use the cpu mem copy function because this is X86. */
	vaddr = io_mapping_map_atomic_wc(mapping, base);
	unwritten = __copy_from_user_inatomic_nocache((void __force *)vaddr + offset,
						      user_data, length);
	io_mapping_unmap_atomic(vaddr);
	if (unwritten) {
		vaddr = io_mapping_map_wc(mapping, base, PAGE_SIZE);
		unwritten = copy_from_user((void __force *)vaddr + offset,
					   user_data, length);
		io_mapping_unmap(vaddr);
	}

	return unwritten;
}

/**
 * This is the fast pwrite path, where we copy the data directly from the
 * user into the GTT, uncached.
 * @obj: i915 GEM object
 * @args: pwrite arguments structure
 */
static int
i915_gem_gtt_pwrite_fast(struct drm_i915_gem_object *obj,
			 const struct drm_i915_gem_pwrite *args)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct i915_ggtt *ggtt = &i915->ggtt;
	struct intel_runtime_pm *rpm = &i915->runtime_pm;
	intel_wakeref_t wakeref;
	struct drm_mm_node node;
	struct i915_vma *vma;
	u64 remain, offset;
	void __user *user_data;
	int ret = 0;

	if (i915_gem_object_has_struct_page(obj)) {
		/*
		 * Avoid waking the device up if we can fallback, as
		 * waking/resuming is very slow (worst-case 10-100 ms
		 * depending on PCI sleeps and our own resume time).
		 * This easily dwarfs any performance advantage from
		 * using the cache bypass of indirect GGTT access.
		 */
		wakeref = intel_runtime_pm_get_if_in_use(rpm);
		if (!wakeref)
			return -EFAULT;
	} else {
		/* No backing pages, no fallback, we must force GGTT access */
		wakeref = intel_runtime_pm_get(rpm);
	}

	vma = i915_gem_gtt_prepare(obj, &node, true);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto out_rpm;
	}

	i915_gem_object_invalidate_frontbuffer(obj, ORIGIN_CPU);

	user_data = u64_to_user_ptr(args->data_ptr);
	offset = args->offset;
	remain = args->size;
	while (remain) {
		/* Operation in this page
		 *
		 * page_base = page offset within aperture
		 * page_offset = offset within page
		 * page_length = bytes to copy for this page
		 */
		u32 page_base = node.start;
		unsigned int page_offset = offset_in_page(offset);
		unsigned int page_length = PAGE_SIZE - page_offset;
		page_length = remain < page_length ? remain : page_length;
		if (drm_mm_node_allocated(&node)) {
			/* flush the write before we modify the GGTT */
			intel_gt_flush_ggtt_writes(ggtt->vm.gt);
			ggtt->vm.insert_page(&ggtt->vm,
					     i915_gem_object_get_dma_address(obj, offset >> PAGE_SHIFT),
					     node.start, I915_CACHE_NONE, 0);
			wmb(); /* flush modifications to the GGTT (insert_page) */
		} else {
			page_base += offset & PAGE_MASK;
		}
		/* If we get a fault while copying data, then (presumably) our
		 * source page isn't available.  Return the error and we'll
		 * retry in the slow path.
		 * If the object is non-shmem backed, we retry again with the
		 * path that handles page fault.
		 */
		if (ggtt_write(&ggtt->iomap, page_base, page_offset,
			       user_data, page_length)) {
			ret = -EFAULT;
			break;
		}

		remain -= page_length;
		user_data += page_length;
		offset += page_length;
	}

	intel_gt_flush_ggtt_writes(ggtt->vm.gt);
	i915_gem_object_flush_frontbuffer(obj, ORIGIN_CPU);

	i915_gem_gtt_cleanup(obj, &node, vma);
out_rpm:
	intel_runtime_pm_put(rpm, wakeref);
	return ret;
}

/* Per-page copy function for the shmem pwrite fastpath.
 * Flushes invalid cachelines before writing to the target if
 * needs_clflush_before is set and flushes out any written cachelines after
 * writing if needs_clflush is set.
 */
static int
shmem_pwrite(struct page *page, int offset, int len, char __user *user_data,
	     bool needs_clflush_before,
	     bool needs_clflush_after)
{
	char *vaddr;
	int ret;

	vaddr = kmap(page);

	if (needs_clflush_before)
		drm_clflush_virt_range(vaddr + offset, len);

	ret = __copy_from_user(vaddr + offset, user_data, len);
	if (!ret && needs_clflush_after)
		drm_clflush_virt_range(vaddr + offset, len);

	kunmap(page);

	return ret ? -EFAULT : 0;
}

static int
i915_gem_shmem_pwrite(struct drm_i915_gem_object *obj,
		      const struct drm_i915_gem_pwrite *args)
{
	unsigned int partial_cacheline_write;
	unsigned int needs_clflush;
	unsigned int offset, idx;
	void __user *user_data;
	u64 remain;
	int ret;

	ret = i915_gem_object_lock_interruptible(obj, NULL);
	if (ret)
		return ret;

	ret = i915_gem_object_pin_pages(obj);
	if (ret)
		goto err_unlock;

	ret = i915_gem_object_prepare_write(obj, &needs_clflush);
	if (ret)
		goto err_unpin;

	i915_gem_object_finish_access(obj);
	i915_gem_object_unlock(obj);

	/* If we don't overwrite a cacheline completely we need to be
	 * careful to have up-to-date data by first clflushing. Don't
	 * overcomplicate things and flush the entire patch.
	 */
	partial_cacheline_write = 0;
	if (needs_clflush & CLFLUSH_BEFORE)
		partial_cacheline_write = boot_cpu_data.x86_clflush_size - 1;

	user_data = u64_to_user_ptr(args->data_ptr);
	remain = args->size;
	offset = offset_in_page(args->offset);
	for (idx = args->offset >> PAGE_SHIFT; remain; idx++) {
		struct page *page = i915_gem_object_get_page(obj, idx);
		unsigned int length = min_t(u64, remain, PAGE_SIZE - offset);

		ret = shmem_pwrite(page, offset, length, user_data,
				   (offset | length) & partial_cacheline_write,
				   needs_clflush & CLFLUSH_AFTER);
		if (ret)
			break;

		remain -= length;
		user_data += length;
		offset = 0;
	}

	i915_gem_object_flush_frontbuffer(obj, ORIGIN_CPU);

	i915_gem_object_unpin_pages(obj);
	return ret;

err_unpin:
	i915_gem_object_unpin_pages(obj);
err_unlock:
	i915_gem_object_unlock(obj);
	return ret;
}

/**
 * Writes data to the object referenced by handle.
 * @dev: drm device
 * @data: ioctl data blob
 * @file: drm file
 *
 * On error, the contents of the buffer that were to be modified are undefined.
 */
int
i915_gem_pwrite_ioctl(struct drm_device *dev, void *data,
		      struct drm_file *file)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_pwrite *args = data;
	struct drm_i915_gem_object *obj;
	int ret;

	/* PWRITE is disallowed for all platforms after TGL-LP.  This also
	 * covers all platforms with local memory.
	 */
	if (GRAPHICS_VER(i915) >= 12 && !IS_TIGERLAKE(i915))
		return -EOPNOTSUPP;

	if (args->size == 0)
		return 0;

	if (!access_ok(u64_to_user_ptr(args->data_ptr), args->size))
		return -EFAULT;

	obj = i915_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	/* Bounds check destination. */
	if (range_overflows_t(u64, args->offset, args->size, obj->base.size)) {
		ret = -EINVAL;
		goto err;
	}

	/* Writes not allowed into this read-only object */
	if (i915_gem_object_is_readonly(obj)) {
		ret = -EINVAL;
		goto err;
	}

	trace_i915_gem_object_pwrite(obj, args->offset, args->size);

	ret = -ENODEV;
	if (obj->ops->pwrite)
		ret = obj->ops->pwrite(obj, args);
	if (ret != -ENODEV)
		goto err;

	ret = i915_gem_object_wait(obj,
				   I915_WAIT_INTERRUPTIBLE |
				   I915_WAIT_ALL,
				   MAX_SCHEDULE_TIMEOUT);
	if (ret)
		goto err;

	ret = -EFAULT;
	/* We can only do the GTT pwrite on untiled buffers, as otherwise
	 * it would end up going through the fenced access, and we'll get
	 * different detiling behavior between reading and writing.
	 * pread/pwrite currently are reading and writing from the CPU
	 * perspective, requiring manual detiling by the client.
	 */
	if (!i915_gem_object_has_struct_page(obj) ||
	    cpu_write_needs_clflush(obj))
		/* Note that the gtt paths might fail with non-page-backed user
		 * pointers (e.g. gtt mappings when moving data between
		 * textures). Fallback to the shmem path in that case.
		 */
		ret = i915_gem_gtt_pwrite_fast(obj, args);

	if (ret == -EFAULT || ret == -ENOSPC) {
		if (i915_gem_object_has_struct_page(obj))
			ret = i915_gem_shmem_pwrite(obj, args);
	}

err:
	i915_gem_object_put(obj);
	return ret;
}

/**
 * Called when user space has done writes to this buffer
 * @dev: drm device
 * @data: ioctl data blob
 * @file: drm file
 */
int
i915_gem_sw_finish_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file)
{
	struct drm_i915_gem_sw_finish *args = data;
	struct drm_i915_gem_object *obj;

	obj = i915_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	/*
	 * Proxy objects are barred from CPU access, so there is no
	 * need to ban sw_finish as it is a nop.
	 */

	/* Pinned buffers may be scanout, so flush the cache */
	i915_gem_object_flush_if_display(obj);
	i915_gem_object_put(obj);

	return 0;
}

void i915_gem_runtime_suspend(struct drm_i915_private *i915)
{
	struct drm_i915_gem_object *obj, *on;
	int i;

	/*
	 * Only called during RPM suspend. All users of the userfault_list
	 * must be holding an RPM wakeref to ensure that this can not
	 * run concurrently with themselves (and use the struct_mutex for
	 * protection between themselves).
	 */

	list_for_each_entry_safe(obj, on,
				 &i915->ggtt.userfault_list, userfault_link)
		__i915_gem_object_release_mmap_gtt(obj);

	/*
	 * The fence will be lost when the device powers down. If any were
	 * in use by hardware (i.e. they are pinned), we should not be powering
	 * down! All other fences will be reacquired by the user upon waking.
	 */
	for (i = 0; i < i915->ggtt.num_fences; i++) {
		struct i915_fence_reg *reg = &i915->ggtt.fence_regs[i];

		/*
		 * Ideally we want to assert that the fence register is not
		 * live at this point (i.e. that no piece of code will be
		 * trying to write through fence + GTT, as that both violates
		 * our tracking of activity and associated locking/barriers,
		 * but also is illegal given that the hw is powered down).
		 *
		 * Previously we used reg->pin_count as a "liveness" indicator.
		 * That is not sufficient, and we need a more fine-grained
		 * tool if we want to have a sanity check here.
		 */

		if (!reg->vma)
			continue;

		GEM_BUG_ON(i915_vma_has_userfault(reg->vma));
		reg->dirty = true;
	}
}

static void discard_ggtt_vma(struct i915_vma *vma)
{
	struct drm_i915_gem_object *obj = vma->obj;

	spin_lock(&obj->vma.lock);
	if (!RB_EMPTY_NODE(&vma->obj_node)) {
		rb_erase(&vma->obj_node, &obj->vma.tree);
		RB_CLEAR_NODE(&vma->obj_node);
	}
	spin_unlock(&obj->vma.lock);
}

struct i915_vma *
i915_gem_object_ggtt_pin_ww(struct drm_i915_gem_object *obj,
			    struct i915_gem_ww_ctx *ww,
			    const struct i915_ggtt_view *view,
			    u64 size, u64 alignment, u64 flags)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct i915_ggtt *ggtt = &i915->ggtt;
	struct i915_vma *vma;
	int ret;

	if (flags & PIN_MAPPABLE &&
	    (!view || view->type == I915_GGTT_VIEW_NORMAL)) {
		/*
		 * If the required space is larger than the available
		 * aperture, we will not able to find a slot for the
		 * object and unbinding the object now will be in
		 * vain. Worse, doing so may cause us to ping-pong
		 * the object in and out of the Global GTT and
		 * waste a lot of cycles under the mutex.
		 */
		if (obj->base.size > ggtt->mappable_end)
			return ERR_PTR(-E2BIG);

		/*
		 * If NONBLOCK is set the caller is optimistically
		 * trying to cache the full object within the mappable
		 * aperture, and *must* have a fallback in place for
		 * situations where we cannot bind the object. We
		 * can be a little more lax here and use the fallback
		 * more often to avoid costly migrations of ourselves
		 * and other objects within the aperture.
		 *
		 * Half-the-aperture is used as a simple heuristic.
		 * More interesting would to do search for a free
		 * block prior to making the commitment to unbind.
		 * That caters for the self-harm case, and with a
		 * little more heuristics (e.g. NOFAULT, NOEVICT)
		 * we could try to minimise harm to others.
		 */
		if (flags & PIN_NONBLOCK &&
		    obj->base.size > ggtt->mappable_end / 2)
			return ERR_PTR(-ENOSPC);
	}

new_vma:
	vma = i915_vma_instance(obj, &ggtt->vm, view);
	if (IS_ERR(vma))
		return vma;

	if (i915_vma_misplaced(vma, size, alignment, flags)) {
		if (flags & PIN_NONBLOCK) {
			if (i915_vma_is_pinned(vma) || i915_vma_is_active(vma))
				return ERR_PTR(-ENOSPC);

			if (flags & PIN_MAPPABLE &&
			    vma->fence_size > ggtt->mappable_end / 2)
				return ERR_PTR(-ENOSPC);
		}

		if (i915_vma_is_pinned(vma) || i915_vma_is_active(vma)) {
			discard_ggtt_vma(vma);
			goto new_vma;
		}

		ret = i915_vma_unbind(vma);
		if (ret)
			return ERR_PTR(ret);
	}

	if (ww)
		ret = i915_vma_pin_ww(vma, ww, size, alignment, flags | PIN_GLOBAL);
	else
		ret = i915_vma_pin(vma, size, alignment, flags | PIN_GLOBAL);

	if (ret)
		return ERR_PTR(ret);

	if (vma->fence && !i915_gem_object_is_tiled(obj)) {
		mutex_lock(&ggtt->vm.mutex);
		i915_vma_revoke_fence(vma);
		mutex_unlock(&ggtt->vm.mutex);
	}

	ret = i915_vma_wait_for_bind(vma);
	if (ret) {
		i915_vma_unpin(vma);
		return ERR_PTR(ret);
	}

	return vma;
}

int
i915_gem_madvise_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_madvise *args = data;
	struct drm_i915_gem_object *obj;
	int err;

	switch (args->madv) {
	case I915_MADV_DONTNEED:
	case I915_MADV_WILLNEED:
	    break;
	default:
	    return -EINVAL;
	}

	obj = i915_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	err = i915_gem_object_lock_interruptible(obj, NULL);
	if (err)
		goto out;

	if (i915_gem_object_has_pages(obj) &&
	    i915_gem_object_is_tiled(obj) &&
	    i915->quirks & QUIRK_PIN_SWIZZLED_PAGES) {
		if (obj->mm.madv == I915_MADV_WILLNEED) {
			GEM_BUG_ON(!i915_gem_object_has_tiling_quirk(obj));
			i915_gem_object_clear_tiling_quirk(obj);
			i915_gem_object_make_shrinkable(obj);
		}
		if (args->madv == I915_MADV_WILLNEED) {
			GEM_BUG_ON(i915_gem_object_has_tiling_quirk(obj));
			i915_gem_object_make_unshrinkable(obj);
			i915_gem_object_set_tiling_quirk(obj);
		}
	}

	if (obj->mm.madv != __I915_MADV_PURGED) {
		obj->mm.madv = args->madv;
		if (obj->ops->adjust_lru)
			obj->ops->adjust_lru(obj);
	}

	if (i915_gem_object_has_pages(obj)) {
		unsigned long flags;

		spin_lock_irqsave(&i915->mm.obj_lock, flags);
		if (!list_empty(&obj->mm.link)) {
			struct list_head *list;

			if (obj->mm.madv != I915_MADV_WILLNEED)
				list = &i915->mm.purge_list;
			else
				list = &i915->mm.shrink_list;
			list_move_tail(&obj->mm.link, list);

		}
		spin_unlock_irqrestore(&i915->mm.obj_lock, flags);
	}

	/* if the object is no longer attached, discard its backing storage */
	if (obj->mm.madv == I915_MADV_DONTNEED &&
	    !i915_gem_object_has_pages(obj))
		i915_gem_object_truncate(obj);

	args->retained = obj->mm.madv != __I915_MADV_PURGED;

	i915_gem_object_unlock(obj);
out:
	i915_gem_object_put(obj);
	return err;
}

int i915_gem_init(struct drm_i915_private *dev_priv)
{
	int ret;

	/* We need to fallback to 4K pages if host doesn't support huge gtt. */
	if (intel_vgpu_active(dev_priv) && !intel_vgpu_has_huge_gtt(dev_priv))
		mkwrite_device_info(dev_priv)->page_sizes =
			I915_GTT_PAGE_SIZE_4K;

	ret = i915_gem_init_userptr(dev_priv);
	if (ret)
		return ret;

	intel_uc_fetch_firmwares(&dev_priv->gt.uc);
	intel_wopcm_init(&dev_priv->wopcm);

	ret = i915_init_ggtt(dev_priv);
	if (ret) {
		GEM_BUG_ON(ret == -EIO);
		goto err_unlock;
	}

	/*
	 * Despite its name intel_init_clock_gating applies both display
	 * clock gating workarounds; GT mmio workarounds and the occasional
	 * GT power context workaround. Worse, sometimes it includes a context
	 * register workaround which we need to apply before we record the
	 * default HW state for all contexts.
	 *
	 * FIXME: break up the workarounds and apply them at the right time!
	 */
	intel_init_clock_gating(dev_priv);

	ret = intel_gt_init(&dev_priv->gt);
	if (ret)
		goto err_unlock;

	return 0;

	/*
	 * Unwinding is complicated by that we want to handle -EIO to mean
	 * disable GPU submission but keep KMS alive. We want to mark the
	 * HW as irrevisibly wedged, but keep enough state around that the
	 * driver doesn't explode during runtime.
	 */
err_unlock:
	i915_gem_drain_workqueue(dev_priv);

	if (ret != -EIO)
		intel_uc_cleanup_firmwares(&dev_priv->gt.uc);

	if (ret == -EIO) {
		/*
		 * Allow engines or uC initialisation to fail by marking the GPU
		 * as wedged. But we only want to do this when the GPU is angry,
		 * for all other failure, such as an allocation failure, bail.
		 */
		if (!intel_gt_is_wedged(&dev_priv->gt)) {
			i915_probe_error(dev_priv,
					 "Failed to initialize GPU, declaring it wedged!\n");
			intel_gt_set_wedged(&dev_priv->gt);
		}

		/* Minimal basic recovery for KMS */
		ret = i915_ggtt_enable_hw(dev_priv);
		i915_ggtt_resume(&dev_priv->ggtt);
		intel_init_clock_gating(dev_priv);
	}

	i915_gem_drain_freed_objects(dev_priv);

	return ret;
}

void i915_gem_driver_register(struct drm_i915_private *i915)
{
	i915_gem_driver_register__shrinker(i915);

	intel_engines_driver_register(i915);
}

void i915_gem_driver_unregister(struct drm_i915_private *i915)
{
	i915_gem_driver_unregister__shrinker(i915);
}

void i915_gem_driver_remove(struct drm_i915_private *dev_priv)
{
	intel_wakeref_auto_fini(&dev_priv->ggtt.userfault_wakeref);

	i915_gem_suspend_late(dev_priv);
	intel_gt_driver_remove(&dev_priv->gt);
	dev_priv->uabi_engines = RB_ROOT;

	/* Flush any outstanding unpin_work. */
	i915_gem_drain_workqueue(dev_priv);

	i915_gem_drain_freed_objects(dev_priv);
}

void i915_gem_driver_release(struct drm_i915_private *dev_priv)
{
	intel_gt_driver_release(&dev_priv->gt);

	intel_wa_list_free(&dev_priv->gt_wa_list);

	intel_uc_cleanup_firmwares(&dev_priv->gt.uc);

	i915_gem_drain_freed_objects(dev_priv);

	drm_WARN_ON(&dev_priv->drm, !list_empty(&dev_priv->gem.contexts.list));
}

static void i915_gem_init__mm(struct drm_i915_private *i915)
{
	spin_lock_init(&i915->mm.obj_lock);

	init_llist_head(&i915->mm.free_list);

	INIT_LIST_HEAD(&i915->mm.purge_list);
	INIT_LIST_HEAD(&i915->mm.shrink_list);

	i915_gem_init__objects(i915);
}

void i915_gem_init_early(struct drm_i915_private *dev_priv)
{
	i915_gem_init__mm(dev_priv);
	i915_gem_init__contexts(dev_priv);

	spin_lock_init(&dev_priv->fb_tracking.lock);
}

void i915_gem_cleanup_early(struct drm_i915_private *dev_priv)
{
	i915_gem_drain_freed_objects(dev_priv);
	GEM_BUG_ON(!llist_empty(&dev_priv->mm.free_list));
	GEM_BUG_ON(atomic_read(&dev_priv->mm.free_count));
	drm_WARN_ON(&dev_priv->drm, dev_priv->mm.shrink_count);
}

int i915_gem_open(struct drm_i915_private *i915, struct drm_file *file)
{
	struct drm_i915_file_private *file_priv;
	int ret;
#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
	intel_wakeref_t wakeref;
#endif

	DRM_DEBUG("\n");

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file->driver_priv = file_priv;
	file_priv->dev_priv = i915;
	file_priv->file = file;

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
	rcu_read_lock();
	file_priv->tgid = get_pid(find_vpid(task_tgid_nr(current)));
	rcu_read_unlock();

	file_priv->process_name =  kzalloc(PAGE_SIZE, GFP_ATOMIC);
	if (!file_priv->process_name) {
		ret = -ENOMEM;
		goto out_free_file;
	}

	ret = i915_get_pid_cmdline(current, file_priv->process_name);
	if (ret)
		goto out_free_name;
#endif

	file_priv->bsd_engine = -1;
	file_priv->hang_timestamp = jiffies;

#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
	wakeref = intel_runtime_pm_get(&i915->runtime_pm);
#endif

	ret = i915_gem_context_open(i915, file);
#if IS_ENABLED(CONFIG_DRM_I915_MEMTRACK)
	if (ret) {
		intel_runtime_pm_put(&i915->runtime_pm, wakeref);
		goto out_free_name;
	}
	intel_runtime_pm_put(&i915->runtime_pm, wakeref);

	ret = i915_gem_create_sysfs_file_entry(&i915->drm, file);
	if (ret) {
		i915_gem_context_close(file);
		goto out_free_name;
	}

	return 0;

out_free_name:
	kfree(file_priv->process_name);
out_free_file:
	put_pid(file_priv->tgid);
	kfree(file_priv);
#else
	if (ret)
		kfree(file_priv);
#endif

	return ret;
}

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/mock_gem_device.c"
#include "selftests/i915_gem.c"
#endif
