/* ****************************************************************************
 *  Copyright(C) 2009-2018 Intel Corporation.  All Rights Reserved.
 *
 *  This file is part of SEP Development Kit
 *
 *  SEP Development Kit is free software; you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *
 *  SEP Development Kit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  As a special exception, you may use this file as part of a free software
 *  library without restriction.  Specifically, if other files instantiate
 *  templates or use macros or inline functions from this file, or you
 *  compile this file and link it with other files to produce an executable
 *  this file does not by itself cause the resulting executable to be
 *  covered by the GNU General Public License.  This exception does not
 *  however invalidate any other reasons why the executable file might be
 *  covered by the GNU General Public License.
 * ****************************************************************************
 */

#include "lwpmudrv_defines.h"
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/mempool.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "lwpmudrv_types.h"
#include "rise_errors.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv.h"
#include "control.h"
#include "utility.h"
#include <linux/sched.h>
#include <linux/kallsyms.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define SMP_CALL_FUNCTION(func, ctx, retry, wait)                              \
	smp_call_function((func), (ctx), (wait))
#define SMP_CALL_FUNCTION_SINGLE(cpuid, func, ctx, retry, wait)                \
	smp_call_function_single((cpuid), (func), (ctx), (wait))
#define ON_EACH_CPU(func, ctx, retry, wait) on_each_cpu((func), (ctx), (wait))
#else
#define SMP_CALL_FUNCTION(func, ctx, retry, wait)                              \
	smp_call_function((func), (ctx), (retry), (wait))
#define SMP_CALL_FUNCTION_SINGLE(cpuid, func, ctx, retry, wait)                \
	smp_call_function_single((cpuid), (func), (ctx), (retry), (wait))
#define ON_EACH_CPU(func, ctx, retry, wait)                                    \
	on_each_cpu((func), (ctx), (retry), (wait))
#endif

#if defined(DRV_SEP_ACRN_ON)
void (*local_vfree_atomic)(const void *addr) = NULL;
#endif

/*
 */
GLOBAL_STATE_NODE driver_state;
MSR_DATA msr_data;
static MEM_TRACKER mem_tr_head; // start of the mem tracker list
static MEM_TRACKER mem_tr_tail; // end of mem tracker list
static spinlock_t mem_tr_lock; // spinlock for mem tracker list
static unsigned long flags;

/* ------------------------------------------------------------------------- */
/*!
 * @fn       VOID CONTROL_Invoke_Cpu (func, ctx, arg)
 *
 * @brief    Set up a DPC call and insert it into the queue
 *
 * @param    IN cpu_idx  - the core id to dispatch this function to
 *           IN func     - function to be invoked by the specified core(s)
 *           IN ctx      - pointer to the parameter block for each function
 *                         invocation
 *
 * @return   None
 *
 * <I>Special Notes:</I>
 *
 */
VOID CONTROL_Invoke_Cpu(int cpu_idx, VOID (*func)(PVOID), PVOID ctx)
{
	SEP_DRV_LOG_TRACE_IN("CPU: %d, function: %p, ctx: %p.", cpu_idx, func,
			     ctx);
	SMP_CALL_FUNCTION_SINGLE(cpu_idx, func, ctx, 0, 1);
	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID CONTROL_Invoke_Parallel_Service(func, ctx, blocking, exclude)
 *
 * @param    func     - function to be invoked by each core in the system
 * @param    ctx      - pointer to the parameter block for each function invocation
 * @param    blocking - Wait for invoked function to complete
 * @param    exclude  - exclude the current core from executing the code
 *
 * @returns  None
 *
 * @brief    Service routine to handle all kinds of parallel invoke on all CPU calls
 *
 * <I>Special Notes:</I>
 *           Invoke the function provided in parallel in either a blocking or
 *           non-blocking mode.  The current core may be excluded if desired.
 *           NOTE - Do not call this function directly from source code.
 *           Use the aliases CONTROL_Invoke_Parallel(), CONTROL_Invoke_Parallel_NB(),
 *           or CONTROL_Invoke_Parallel_XS().
 *
 */
VOID CONTROL_Invoke_Parallel_Service(VOID (*func)(PVOID), PVOID ctx,
					    int blocking, int exclude)
{
	SEP_DRV_LOG_TRACE_IN("Fn: %p, ctx: %p, block: %d, excl: %d.",
			     func, ctx, blocking, exclude);

	GLOBAL_STATE_cpu_count(driver_state) = 0;
	GLOBAL_STATE_dpc_count(driver_state) = 0;

	if (GLOBAL_STATE_num_cpus(driver_state) == 1) {
		if (!exclude) {
			func(ctx);
		}
		SEP_DRV_LOG_TRACE_OUT("");
		return;
	}
	if (!exclude) {
		ON_EACH_CPU(func, ctx, 0, blocking);
		SEP_DRV_LOG_TRACE_OUT("");
		return;
	}

	preempt_disable();
	SMP_CALL_FUNCTION(func, ctx, 0, blocking);
	preempt_enable();

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID control_Memory_Tracker_Delete_Node(mem_tr)
 *
 * @param    IN mem_tr    - memory tracker node to delete
 *
 * @returns  None
 *
 * @brief    Delete specified node in the memory tracker
 *
 * <I>Special Notes:</I>
 *           Assumes mem_tr_lock is already held while calling this function!
 */
static VOID control_Memory_Tracker_Delete_Node(MEM_TRACKER mem_tr)
{
	MEM_TRACKER prev_tr = NULL;
	MEM_TRACKER next_tr = NULL;
	U32 size = 0;

	SEP_DRV_LOG_ALLOC_IN("");

	if (!mem_tr) {
		SEP_DRV_LOG_ALLOC_OUT("mem_tr is NULL!");
		return;
	}
	size = MEM_TRACKER_max_size(mem_tr) * sizeof(MEM_EL_NODE);
	// update the linked list
	prev_tr = MEM_TRACKER_prev(mem_tr);
	next_tr = MEM_TRACKER_next(mem_tr);
	if (prev_tr) {
		MEM_TRACKER_next(prev_tr) = next_tr;
	}
	if (next_tr) {
		MEM_TRACKER_prev(next_tr) = prev_tr;
	}

	// free the allocated mem_el array (if any)
	if (MEM_TRACKER_mem(mem_tr)) {
		if (MEM_TRACKER_array_vmalloc(mem_tr)) {
			vfree(MEM_TRACKER_mem(mem_tr));
		} else {
			if (size < MAX_KMALLOC_SIZE) {
				kfree(MEM_TRACKER_mem(mem_tr));
			} else {
				free_pages(
					(unsigned long)MEM_TRACKER_mem(mem_tr),
					get_order(size));
			}
		}
	}

	// free the mem_tracker node
	if (MEM_TRACKER_node_vmalloc(mem_tr)) {
		vfree(mem_tr);
	} else {
		kfree(mem_tr);
	}
	SEP_DRV_LOG_ALLOC_OUT("");
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID control_Memory_Tracker_Create_Node(void)
 *
 * @param    None    - size of the memory to allocate
 *
 * @returns  OS_SUCCESS if successful, otherwise error
 *
 * @brief    Initialize the memory tracker
 *
 * <I>Special Notes:</I>
 *           Assumes mem_tr_lock is already held while calling this function!
 *
 *           Since this function can be called within either GFP_KERNEL or
 *           GFP_ATOMIC contexts, the most restrictive allocation is used
 *           (viz., GFP_ATOMIC).
 */
static U32 control_Memory_Tracker_Create_Node(void)
{
	U32 size = MEM_EL_MAX_ARRAY_SIZE * sizeof(MEM_EL_NODE);
	PVOID location = NULL;
	MEM_TRACKER mem_tr = NULL;

	SEP_DRV_LOG_ALLOC_IN("");

	// create a mem tracker node
	mem_tr = (MEM_TRACKER)kmalloc(sizeof(MEM_TRACKER_NODE), GFP_ATOMIC);
	if (!mem_tr) {
		mem_tr = (MEM_TRACKER)vmalloc(sizeof(MEM_TRACKER_NODE));
		if (mem_tr) {
			MEM_TRACKER_node_vmalloc(mem_tr) = 1;
		} else {
			SEP_DRV_LOG_ERROR_ALLOC_OUT(
				"Failed to allocate mem tracker node.");
			return OS_FAULT;
		}
	} else {
		MEM_TRACKER_node_vmalloc(mem_tr) = 0;
	}
	SEP_DRV_LOG_TRACE("Node %p, vmalloc %d.", mem_tr,
			  MEM_TRACKER_node_vmalloc(mem_tr));

	// create an initial array of mem_el's inside the mem tracker node
	MEM_TRACKER_array_vmalloc(mem_tr) = 0;
	if (size < MAX_KMALLOC_SIZE) {
		location = (PVOID)kmalloc(size, GFP_ATOMIC);
		SEP_DRV_LOG_ALLOC("Allocated small memory (0x%p, %d).",
				  location, (S32)size);
	} else {
		location = (PVOID)__get_free_pages(GFP_ATOMIC, get_order(size));
		SEP_DRV_LOG_ALLOC("Allocated large memory (0x%p, %d).",
				  location, (S32)size);
	}
	if (!location) {
		location = (PVOID)vmalloc(size);
		if (location) {
			MEM_TRACKER_array_vmalloc(mem_tr) = 1;
			SEP_DRV_LOG_ALLOC(
				"Allocated memory (vmalloc) (0x%p, %d).",
				location, (S32)size);
		} else {
			if (MEM_TRACKER_node_vmalloc(mem_tr)) {
				vfree(mem_tr);
			} else {
				kfree(mem_tr);
			}
			SEP_DRV_LOG_ERROR_ALLOC_OUT(
				"Failed to allocate mem_el array... deleting node.");
			return OS_FAULT;
		}
	}

	// initialize new mem tracker node
	MEM_TRACKER_mem(mem_tr) = location;
	MEM_TRACKER_prev(mem_tr) = NULL;
	MEM_TRACKER_next(mem_tr) = NULL;

	// initialize mem_tracker's mem_el array
	MEM_TRACKER_max_size(mem_tr) = MEM_EL_MAX_ARRAY_SIZE;
	MEM_TRACKER_elements(mem_tr) = 0;
	memset(MEM_TRACKER_mem(mem_tr), 0, size);

	// update the linked list
	if (!mem_tr_head) {
		mem_tr_head = mem_tr;
	} else {
		MEM_TRACKER_prev(mem_tr) = mem_tr_tail;
		MEM_TRACKER_next(mem_tr_tail) = mem_tr;
	}
	mem_tr_tail = mem_tr;

	SEP_DRV_LOG_ALLOC_OUT("Allocated node=0x%p, max_elements=%d, size=%d.",
			MEM_TRACKER_mem(mem_tr_tail),
			MEM_EL_MAX_ARRAY_SIZE, size);
	return OS_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID control_Memory_Tracker_Add(location, size, vmalloc_flag)
 *
 * @param    IN location     - memory location
 * @param    IN size         - size of the memory to allocate
 * @param    IN vmalloc_flag - flag that indicates if the allocation was done with vmalloc
 *
 * @returns  None
 *
 * @brief    Keep track of allocated memory with memory tracker
 *
 * <I>Special Notes:</I>
 *           Starting from first mem_tracker node, the algorithm
 *           finds the first "hole" in the mem_tracker list and
 *           tracks the memory allocation there.
 */
static U32 control_Memory_Tracker_Add(PVOID location, ssize_t size,
				      DRV_BOOL vmalloc_flag)
{
	S32 i, n;
	U32 status;
	DRV_BOOL found;
	MEM_TRACKER mem_tr;

	SEP_DRV_LOG_ALLOC_IN("Location: %p, size: %u, flag: %u.", location,
			     (U32)size, vmalloc_flag);

	spin_lock_irqsave(&mem_tr_lock, flags);

	// check if there is space in ANY of mem_tracker's nodes for the memory item
	mem_tr = mem_tr_head;
	found = FALSE;
	status = OS_SUCCESS;
	i = n = 0;
	while (mem_tr && (!found)) {
		if (MEM_TRACKER_elements(mem_tr) <
		    MEM_TRACKER_max_size(mem_tr)) {
			for (i = 0; i < MEM_TRACKER_max_size(mem_tr); i++) {
				if (!MEM_TRACKER_mem_address(mem_tr, i)) {
					SEP_DRV_LOG_ALLOC(
						"Found index %d of %d available.",
						i,
						MEM_TRACKER_max_size(mem_tr) -
							1);
					n = i;
					found = TRUE;
					break;
				}
			}
		}
		if (!found) {
			mem_tr = MEM_TRACKER_next(mem_tr);
		}
	}

	if (!found) {
		// extend into (i.e., create new) mem_tracker node ...
		status = control_Memory_Tracker_Create_Node();
		if (status != OS_SUCCESS) {
			SEP_DRV_LOG_ERROR("Unable to create mem tracker node.");
			goto finish_add;
		}
		// use mem tracker tail node and first available entry in mem_el array
		mem_tr = mem_tr_tail;
		n = 0;
	}

	// we now have a location in mem tracker to keep track of the memory item
	MEM_TRACKER_mem_address(mem_tr, n) = location;
	MEM_TRACKER_mem_size(mem_tr, n) = size;
	MEM_TRACKER_mem_vmalloc(mem_tr, n) = vmalloc_flag;
	MEM_TRACKER_elements(mem_tr)++;
	SEP_DRV_LOG_ALLOC("Tracking (0x%p, %d) in node %d of %d.", location,
			  (S32)size, n, MEM_TRACKER_max_size(mem_tr) - 1);

finish_add:
	spin_unlock_irqrestore(&mem_tr_lock, flags);

	SEP_DRV_LOG_ALLOC_OUT("Result: %u.", status);
	return status;
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID CONTROL_Memory_Tracker_Init(void)
 *
 * @param    None
 *
 * @returns  None
 *
 * @brief    Initializes Memory Tracker
 *
 * <I>Special Notes:</I>
 *           This should only be called when the driver is being loaded.
 */
VOID CONTROL_Memory_Tracker_Init(void)
{
	SEP_DRV_LOG_ALLOC_IN("Initializing mem tracker.");

	mem_tr_head = NULL;
	mem_tr_tail = NULL;

	spin_lock_init(&mem_tr_lock);

	SEP_DRV_LOG_ALLOC_OUT("");
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID CONTROL_Memory_Tracker_Free(void)
 *
 * @param    None
 *
 * @returns  None
 *
 * @brief    Frees memory used by Memory Tracker
 *
 * <I>Special Notes:</I>
 *           This should only be called when the driver is being unloaded.
 */
VOID CONTROL_Memory_Tracker_Free(void)
{
	S32 i;
	MEM_TRACKER temp;

	SEP_DRV_LOG_ALLOC_IN("Destroying mem tracker.");

	spin_lock_irqsave(&mem_tr_lock, flags);

	// check for any memory that was not freed, and free it
	while (mem_tr_head) {
		if (MEM_TRACKER_elements(mem_tr_head)) {
			for (i = 0; i < MEM_TRACKER_max_size(mem_tr_head);
			     i++) {
				if (MEM_TRACKER_mem_address(mem_tr_head, i)) {
					SEP_DRV_LOG_WARNING(
						"Index %d of %d, not freed (0x%p, %d) ... freeing now.",
						i,
						MEM_TRACKER_max_size(
							mem_tr_head) -
							1,
						MEM_TRACKER_mem_address(
							mem_tr_head, i),
						MEM_TRACKER_mem_size(
							mem_tr_head, i));

					if (MEM_TRACKER_mem_vmalloc(mem_tr_head,
								    i)) {
						vfree(MEM_TRACKER_mem_address(
							mem_tr_head, i));
					} else {
						free_pages(
							(unsigned long)
								MEM_TRACKER_mem_address(
									mem_tr_head,
									i),
							get_order(MEM_TRACKER_mem_size(
								mem_tr_head,
								i)));
					}
					MEM_TRACKER_mem_address(mem_tr_head,
								i) = NULL;
					MEM_TRACKER_mem_size(mem_tr_head, i) =
						0;
					MEM_TRACKER_mem_vmalloc(mem_tr_head,
								i) = 0;
				}
			}
		}
		temp = mem_tr_head;
		mem_tr_head = MEM_TRACKER_next(mem_tr_head);
		control_Memory_Tracker_Delete_Node(temp);
	}

	mem_tr_tail = NULL;

	spin_unlock_irqrestore(&mem_tr_lock, flags);

	SEP_DRV_LOG_ALLOC_OUT("Mem tracker destruction complete.");
}

/* ------------------------------------------------------------------------- */
/*
 * @fn VOID CONTROL_Memory_Tracker_Compaction(void)
 *
 * @param    None
 *
 * @returns  None
 *
 * @brief    Compacts the memory allocator if holes are detected
 *
 * <I>Special Notes:</I>
 *           The algorithm compacts mem_tracker nodes such that
 *           node entries are full starting from mem_tr_head
 *           up until the first empty node is detected, after
 *           which nodes up to mem_tr_tail will be empty.
 *           At end of collection (or at other safe sync point),
 *           we reclaim/compact space used by mem tracker.
 */
VOID CONTROL_Memory_Tracker_Compaction(void)
{
	S32 i, j, n, m, c, d;
	DRV_BOOL found, overlap;
	MEM_TRACKER mem_tr1, mem_tr2, empty_tr;

	SEP_DRV_LOG_FLOW_IN("");

	spin_lock_irqsave(&mem_tr_lock, flags);

	mem_tr1 = mem_tr_head;

	i = j = n = c = d = 0;

	/*
	 * step1: free up the track node which does not contain any elements.
	 */
	while (mem_tr1) {
		SEP_DRV_LOG_ALLOC("Node %p, index %d, elememts %d.", mem_tr1, n,
				  MEM_TRACKER_elements(mem_tr1));
		if (MEM_TRACKER_elements(mem_tr1)) {
			mem_tr1 = MEM_TRACKER_next(mem_tr1);
		} else {
			empty_tr = mem_tr1;
			mem_tr1 = MEM_TRACKER_next(mem_tr1);
			if (empty_tr == mem_tr_head) {
				mem_tr_head = mem_tr1;
			}
			if (empty_tr == mem_tr_tail) {
				mem_tr_tail = MEM_TRACKER_prev(empty_tr);
			}
			control_Memory_Tracker_Delete_Node(empty_tr);
			d++;
			SEP_DRV_LOG_ALLOC("Delete node %p.", mem_tr1);
		}
	}

	mem_tr1 = mem_tr_head;
	mem_tr2 = mem_tr_tail;

	/*
	 * there is no need to compact if memory tracker was never used, or only have one track node
	 */
	overlap = (mem_tr1 == mem_tr2);
	if (!mem_tr1 || !mem_tr2 || overlap) {
		goto finish_compact;
	}

	/*
	 * step2: there are more than 2 track node.
	 *        starting from head node, find an empty element slot in a node
	 *        if there is no empty slot or the node is tail, the compact is done.
	 *        find an element in tail node, and move it to the empty slot fount below.
	 *        if tail node is empty after moving, free it up.
	 *        repeat until only one node.
	 */
	m = MEM_TRACKER_max_size(mem_tr2) - 1;
	while (!overlap) {
		// find an empty node
		found = FALSE;
		while (!found && !overlap && mem_tr1) {
			SEP_DRV_LOG_TRACE(
				"Looking at mem_tr1 0x%p, index=%d, elements %d.",
				mem_tr1, n, MEM_TRACKER_elements(mem_tr1));
			if (MEM_TRACKER_elements(mem_tr1) <
			    MEM_TRACKER_max_size(mem_tr1)) {
				for (i = n; i < MEM_TRACKER_max_size(mem_tr1);
				     i++) {
					if (!MEM_TRACKER_mem_address(mem_tr1,
								     i)) {
						SEP_DRV_LOG_TRACE(
							"Found index %d of %d empty.",
							i,
							MEM_TRACKER_max_size(
								mem_tr1) -
								1);
						found = TRUE;
						break; // tentative
					}
				}
			}

			// if no overlap and an empty node was not found, then advance to next node
			if (!found) {
				mem_tr1 = MEM_TRACKER_next(mem_tr1);
				// check for overlap
				overlap = (mem_tr1 == mem_tr2);
				n = 0;
			}
		}
		// all nodes going in forward direction are full, so exit
		if (!found || overlap || !mem_tr1) {
			goto finish_compact;
		}

		// find a non-empty node
		found = FALSE;
		while (!found && !overlap && mem_tr2) {
			SEP_DRV_LOG_ALLOC(
				"Looking at mem_tr2 0x%p, index=%d, elements %d.",
				mem_tr2, m, MEM_TRACKER_elements(mem_tr2));
			if (MEM_TRACKER_elements(mem_tr2)) {
				for (j = m; j >= 0; j--) {
					if (MEM_TRACKER_mem_address(mem_tr2,
								    j)) {
						SEP_DRV_LOG_ALLOC(
							"Found index %d of %d non-empty.",
							j,
							MEM_TRACKER_max_size(
								mem_tr2) -
								1);
						found = TRUE;
						// Any reason why we are not 'breaking' here?
					}
				}
			}

			// if no overlap and no non-empty node was found, then retreat to prev node
			if (!found) {
				empty_tr = mem_tr2; // keep track of empty node
				mem_tr2 = MEM_TRACKER_prev(mem_tr2);
				m = MEM_TRACKER_max_size(mem_tr2) - 1;
				mem_tr_tail = mem_tr2; // keep track of new tail
				// reclaim empty mem_tracker node
				control_Memory_Tracker_Delete_Node(empty_tr);
				// keep track of number of node deletions performed
				d++;
				// check for overlap
				overlap = (mem_tr1 == mem_tr2);
			}
		}
		// all nodes going in reverse direction are empty, so exit
		if (!found || overlap || !mem_tr2) {
			goto finish_compact;
		}

		// swap empty node with non-empty node so that "holes" get bubbled towards the end of list
		MEM_TRACKER_mem_address(mem_tr1, i) =
			MEM_TRACKER_mem_address(mem_tr2, j);
		MEM_TRACKER_mem_size(mem_tr1, i) =
			MEM_TRACKER_mem_size(mem_tr2, j);
		MEM_TRACKER_mem_vmalloc(mem_tr1, i) =
			MEM_TRACKER_mem_vmalloc(mem_tr2, j);
		MEM_TRACKER_elements(mem_tr1)++;

		MEM_TRACKER_mem_address(mem_tr2, j) = NULL;
		MEM_TRACKER_mem_size(mem_tr2, j) = 0;
		MEM_TRACKER_mem_vmalloc(mem_tr2, j) = FALSE;
		MEM_TRACKER_elements(mem_tr2)--;

		SEP_DRV_LOG_ALLOC(
			"Node <%p, elemts %d, index %d> moved to <%p, elemts %d, index %d>.",
			mem_tr2, MEM_TRACKER_elements(mem_tr2), j, mem_tr1,
			MEM_TRACKER_elements(mem_tr1), i);

		// keep track of number of memory compactions performed
		c++;

		// start new search starting from next element in mem_tr1
		n = i + 1;

		// start new search starting from prev element in mem_tr2
		m = j - 1;
	}

finish_compact:
	spin_unlock_irqrestore(&mem_tr_lock, flags);

	SEP_DRV_LOG_FLOW_OUT(
		"Number of elements compacted = %d, nodes deleted = %d.", c, d);
}

/* ------------------------------------------------------------------------- */
/*
 * @fn PVOID CONTROL_Allocate_Memory(size)
 *
 * @param    IN size     - size of the memory to allocate
 *
 * @returns  char*       - pointer to the allocated memory block
 *
 * @brief    Allocate and zero memory
 *
 * <I>Special Notes:</I>
 *           Allocate memory in the GFP_KERNEL pool.
 *
 *           Use this if memory is to be allocated within a context where
 *           the allocator can block the allocation (e.g., by putting
 *           the caller to sleep) while it tries to free up memory to
 *           satisfy the request.  Otherwise, if the allocation must
 *           occur atomically (e.g., caller cannot sleep), then use
 *           CONTROL_Allocate_KMemory instead.
 */
PVOID CONTROL_Allocate_Memory(size_t size)
{
	U32 status;
	PVOID location = NULL;

	SEP_DRV_LOG_ALLOC_IN("Attempting to allocate %d bytes.", (S32)size);

	if (size <= 0) {
		SEP_DRV_LOG_WARNING_ALLOC_OUT(
			"Cannot allocate a number of bytes <= 0.");
		return NULL;
	}

	// determine whether to use mem_tracker or not
	if (size < MAX_KMALLOC_SIZE) {
		location = (PVOID)kmalloc(size, GFP_KERNEL);
		SEP_DRV_LOG_ALLOC("Allocated small memory (0x%p, %d)", location,
				  (S32)size);
	}
	if (!location) {
		location = (PVOID)vmalloc(size);
		if (location) {
			status = control_Memory_Tracker_Add(location, size,
							    TRUE);
			SEP_DRV_LOG_ALLOC("Allocated large memory (0x%p, %d)",
					  location, (S32)size);
			if (status != OS_SUCCESS) {
				// failed to track in mem_tracker, so free up memory and return NULL
				SEP_DRV_LOG_ERROR(
					"Allocated %db; failed to track w/ MEM_TRACKER. Freeing...",
					(S32)size);
				vfree(location);
				location = NULL;
			}
		}
	}

	if (!location) {
		SEP_DRV_LOG_ERROR("Failed to allocated %db.", (S32)size);
	} else {
		memset(location, 0, size);
	}

	SEP_DRV_LOG_ALLOC_OUT("Returning %p.", location);
	return location;
}

/* ------------------------------------------------------------------------- */
/*
 * @fn PVOID CONTROL_Allocate_KMemory(size)
 *
 * @param    IN size     - size of the memory to allocate
 *
 * @returns  char*       - pointer to the allocated memory block
 *
 * @brief    Allocate and zero memory
 *
 * <I>Special Notes:</I>
 *           Allocate memory in the GFP_ATOMIC pool.
 *
 *           Use this if memory is to be allocated within a context where
 *           the allocator cannot block the allocation (e.g., by putting
 *           the caller to sleep) as it tries to free up memory to
 *           satisfy the request.  Examples include interrupt handlers,
 *           process context code holding locks, etc.
 */
PVOID CONTROL_Allocate_KMemory(size_t size)
{
	U32 status;
	PVOID location;

	SEP_DRV_LOG_ALLOC_IN("Attempting to allocate %d bytes.", (S32)size);

	if (size <= 0) {
		SEP_DRV_LOG_ALLOC_OUT(
			"Cannot allocate a number of bytes <= 0.");
		return NULL;
	}

	if (size < MAX_KMALLOC_SIZE) {
		location = (PVOID)kmalloc(size, GFP_ATOMIC);
		SEP_DRV_LOG_ALLOC("Allocated small memory (0x%p, %d)", location,
				  (S32)size);
	} else {
		location = (PVOID)__get_free_pages(GFP_ATOMIC, get_order(size));
		if (location) {
			status = control_Memory_Tracker_Add(location, size,
							    FALSE);
			SEP_DRV_LOG_ALLOC("Allocated large memory (0x%p, %d)",
					  location, (S32)size);
			if (status != OS_SUCCESS) {
				// failed to track in mem_tracker, so free up memory and return NULL
				SEP_DRV_LOG_ERROR(
					"Allocated %db; failed to track w/ MEM_TRACKER. Freeing...",
					(S32)size);
				free_pages((unsigned long)location,
					   get_order(size));
				location = NULL;
			}
		}
	}

	if (!location) {
		SEP_DRV_LOG_ERROR("Failed to allocated %db.", (S32)size);
	} else {
		memset(location, 0, size);
	}

	SEP_DRV_LOG_ALLOC_OUT("Returning %p.", location);
	return location;
}

/* ------------------------------------------------------------------------- */
/*
 * @fn PVOID CONTROL_Free_Memory(location)
 *
 * @param    IN location  - size of the memory to allocate
 *
 * @returns  pointer to the allocated memory block
 *
 * @brief    Frees the memory block
 *
 * <I>Special Notes:</I>
 *           Does not try to free memory if fed with a NULL pointer
 *           Expected usage:
 *               ptr = CONTROL_Free_Memory(ptr);
 *           Does not do compaction ... can have "holes" in
 *           mem_tracker list after this operation.
 */
PVOID CONTROL_Free_Memory(PVOID location)
{
	S32 i;
	DRV_BOOL found;
	MEM_TRACKER mem_tr;

	SEP_DRV_LOG_ALLOC_IN("Attempting to free %p.", location);

	if (!location) {
		SEP_DRV_LOG_ALLOC_OUT("Cannot free NULL.");
		return NULL;
	}

#if defined(DRV_SEP_ACRN_ON)
	if (!local_vfree_atomic) {
		local_vfree_atomic = (PVOID)UTILITY_Find_Symbol("vfree_atomic");
		if (!local_vfree_atomic) {
			SEP_PRINT_ERROR("Could not find 'vfree_atomic'!\n");
		}
	}
#endif
	spin_lock_irqsave(&mem_tr_lock, flags);

	// scan through mem_tracker nodes for matching entry (if any)
	mem_tr = mem_tr_head;
	found = FALSE;
	while (mem_tr) {
		for (i = 0; i < MEM_TRACKER_max_size(mem_tr); i++) {
			if (location == MEM_TRACKER_mem_address(mem_tr, i)) {
				SEP_DRV_LOG_ALLOC(
					"Freeing large memory location 0x%p",
					location);
				found = TRUE;
				if (MEM_TRACKER_mem_vmalloc(mem_tr, i)) {
#if defined(DRV_SEP_ACRN_ON)
					if (unlikely(in_atomic() &&
						     local_vfree_atomic)) {
						local_vfree_atomic(location);
					} else {
#endif
						vfree(location);
					}

#if defined(DRV_SEP_ACRN_ON)
				}
#endif
				else {
					free_pages(
						(unsigned long)location,
						get_order(MEM_TRACKER_mem_size(
							mem_tr, i)));
				}
				MEM_TRACKER_mem_address(mem_tr, i) = NULL;
				MEM_TRACKER_mem_size(mem_tr, i) = 0;
				MEM_TRACKER_mem_vmalloc(mem_tr, i) = 0;
				MEM_TRACKER_elements(mem_tr)--;
				goto finish_free;
			}
		}
		mem_tr = MEM_TRACKER_next(mem_tr);
	}

finish_free:
	spin_unlock_irqrestore(&mem_tr_lock, flags);

	// must have been of smaller than the size limit for mem tracker nodes
	if (!found) {
		SEP_DRV_LOG_ALLOC("Freeing small memory location 0x%p",
				  location);
		kfree(location);
	}

	SEP_DRV_LOG_ALLOC_OUT("Success. Returning NULL.");
	return NULL;
}
