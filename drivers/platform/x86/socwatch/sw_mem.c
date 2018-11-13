/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2018 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1300 S Mopac Expwy,
  Austin, TX 78746

  BSD LICENSE

  Copyright(c) 2014 - 2018 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <linux/slab.h>

#include "sw_kernel_defines.h"
#include "sw_lock_defs.h"
#include "sw_mem.h"

/*
 * How do we behave if we ever
 * get an allocation error?
 * (a) Setting to '1' REFUSES ANY FURTHER
 * allocation requests.
 * (b) Setting to '0' treats each
 * allocation request as separate, and
 * handles them on an on-demand basis
 */
#define DO_MEM_PANIC_ON_ALLOC_ERROR 0

#if DO_MEM_PANIC_ON_ALLOC_ERROR
/*
 * If we ever run into memory allocation errors then
 * stop (and drop) everything.
 */
static atomic_t pw_mem_should_panic = ATOMIC_INIT(0);
/*
 * Macro to check if PANIC is on.
 */
#define MEM_PANIC()                                                            \
	do {                                                                   \
		atomic_set(&pw_mem_should_panic, 1);                           \
		smp_mb();                                                      \
	} while (0)
#define SHOULD_TRACE()                                                         \
	({                                                                     \
		bool __tmp = false;                                            \
		smp_mb();                                                      \
		__tmp = (atomic_read(&pw_mem_should_panic) == 0);              \
		__tmp;                                                         \
	})

#else // if !DO_MEM_PANIC_ON_ALLOC_ERROR

#define MEM_PANIC()
#define SHOULD_TRACE() (true)

#endif

/*
 * Variables to track memory usage.
 */
/*
 * TOTAL num bytes allocated.
 */
static u64 total_num_bytes_alloced;
/*
 * Num of allocated bytes that have
 * not yet been freed.
 */
static u64 curr_num_bytes_alloced;
/*
 * Max # of allocated bytes that
 * have not been freed at any point
 * in time.
 */
static u64 max_num_bytes_alloced;

u64 sw_get_total_bytes_alloced(void)
{
	return total_num_bytes_alloced;
};

u64 sw_get_max_bytes_alloced(void)
{
	return max_num_bytes_alloced;
};

u64 sw_get_curr_bytes_alloced(void)
{
	return curr_num_bytes_alloced;
};

/*
 * Allocate free pages.
 * TODO: add memory tracker?
 */
unsigned long sw_allocate_pages(gfp_t flags,
				unsigned int alloc_size_in_bytes)
{
	return __get_free_pages(flags, get_order(alloc_size_in_bytes));
}
/*
 * Free up previously allocated pages.
 * TODO: add memory tracker?
 */
void sw_release_pages(unsigned long addr, unsigned int alloc_size_in_bytes)
{
	free_pages(addr, get_order(alloc_size_in_bytes));
}

#if DO_TRACK_MEMORY_USAGE

/*
 * Lock to guard access to memory
 * debugging stats.
 */
static SW_DEFINE_SPINLOCK(sw_kmalloc_lock);

/*
 * Helper macros to print out
 * mem debugging stats.
 */
#define TOTAL_NUM_BYTES_ALLOCED() total_num_bytes_alloced
#define CURR_NUM_BYTES_ALLOCED() curr_num_bytes_alloced
#define MAX_NUM_BYTES_ALLOCED() max_num_bytes_alloced

/*
 * MAGIC number based memory tracker. Relies on
 * storing (a) a MAGIC marker and (b) the requested
 * size WITHIN the allocated block of memory. Standard
 * malloc-tracking stuff, really.
 *
 * Overview:
 * (1) ALLOCATION:
 * When asked to allocate a block of 'X' bytes, allocate
 * 'X' + 8 bytes. Then, in the FIRST 4 bytes, write the
 * requested size. In the NEXT 4 bytes, write a special
 * (i.e. MAGIC) number to let our deallocator know that
 * this block of memory was allocated using this technique.
 * Also, keep track of the number of bytes allocated.
 *
 * (2) DEALLOCATION:
 * When given an object to deallocate, we first check
 * the MAGIC number by decrementing the pointer by
 * 4 bytes and reading the (integer) stored there.
 * After ensuring the pointer was, in fact, allocated
 * by us, we then read the size of the allocated
 * block (again, by decrementing the pointer by 4
 * bytes and reading the integer size). We
 * use this size argument to decrement # of bytes
 * allocated.
 */
#define PW_MEM_MAGIC 0xdeadbeef

#define PW_ADD_MAGIC(x)                                                        \
	({                                                                     \
		char *__tmp1 = (char *)(x);                                    \
		*((int *)__tmp1) = PW_MEM_MAGIC;                               \
		__tmp1 += sizeof(int);                                         \
		__tmp1;                                                        \
	})
#define PW_ADD_SIZE(x, s)                                                      \
	({                                                                     \
		char *__tmp1 = (char *)(x);                                    \
		*((int *)__tmp1) = (s);                                        \
		__tmp1 += sizeof(int);                                         \
		__tmp1;                                                        \
	})
#define PW_ADD_STAMP(x, s) PW_ADD_MAGIC(PW_ADD_SIZE((x), (s)))

#define PW_IS_MAGIC(x)                                                         \
	({                                                                     \
		int *__tmp1 = (int *)((char *)(x) - sizeof(int));              \
		*__tmp1 == PW_MEM_MAGIC;                                       \
	})
#define PW_REMOVE_STAMP(x)                                                     \
	({                                                                     \
		char *__tmp1 = (char *)(x);                                    \
		__tmp1 -= sizeof(int) * 2;                                     \
		__tmp1;                                                        \
	})
#define PW_GET_SIZE(x) (*((int *)(x)))

void *sw_kmalloc(size_t size, gfp_t flags)
{
	size_t act_size = 0;
	void *retVal = NULL;
	/*
	 * No point in allocating if
	 * we were unable to allocate
	 * previously!
	 */
	{
		if (!SHOULD_TRACE()) {
			return NULL;
		}
	}
	/*
	 * (1) Allocate requested block.
	 */
	act_size = size + sizeof(int) * 2;
	retVal = kmalloc(act_size, flags);
	if (!retVal) {
		/*
		 * Panic if we couldn't allocate
		 * requested memory.
		 */
		printk(KERN_INFO "ERROR: could NOT allocate memory!\n");
		MEM_PANIC();
		return NULL;
	}
	/*
	 * (2) Update memory usage stats.
	 */
	LOCK(sw_kmalloc_lock);
	{
		total_num_bytes_alloced += size;
		curr_num_bytes_alloced += size;
		if (curr_num_bytes_alloced > max_num_bytes_alloced)
			max_num_bytes_alloced = curr_num_bytes_alloced;
	}
	UNLOCK(sw_kmalloc_lock);
	/*
	 * (3) And finally, add the 'size'
	 * and 'magic' stamps.
	 */
	return PW_ADD_STAMP(retVal, size);
};

void sw_kfree(const void *obj)
{
	void *tmp = NULL;
	size_t size = 0;

	/*
	 * (1) Check if this block was allocated
	 * by us.
	 */
	if (!PW_IS_MAGIC(obj)) {
		printk(KERN_INFO "ERROR: %p is NOT a PW_MAGIC ptr!\n", obj);
		return;
	}
	/*
	 * (2) Strip the magic num...
	 */
	tmp = PW_REMOVE_STAMP(obj);
	/*
	 * ...and retrieve size of block.
	 */
	size = PW_GET_SIZE(tmp);
	/*
	 * (3) Update memory usage stats.
	 */
	LOCK(sw_kmalloc_lock);
	{
		curr_num_bytes_alloced -= size;
	}
	UNLOCK(sw_kmalloc_lock);
	/*
	 * And finally, free the block.
	 */
	kfree(tmp);
};

#else // !DO_TRACK_MEMORY_USAGE

void *sw_kmalloc(size_t size, gfp_t flags)
{
	void *ret = NULL;

	if (SHOULD_TRACE()) {
		if (!(ret = kmalloc(size, flags))) {
			/*
			 * Panic if we couldn't allocate
			 * requested memory.
			 */
			MEM_PANIC();
		}
	}
	return ret;
};

void sw_kfree(const void *mem)
{
	kfree(mem);
};

#endif // DO_TRACK_MEMORY_USAGE
