/* SPDX-License-Identifier: GPL-2.0 AND BSD-3-Clause
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 - 2019 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information:
 * SoC Watch Developer Team <socwatchdevelopers@intel.com>
 * Intel Corporation,
 * 1300 S Mopac Expwy,
 * Austin, TX 78746
 *
 * BSD LICENSE
 *
 * Copyright(c) 2014 - 2019 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sw_internal.h"
#include "sw_output_buffer.h"
#include "sw_kernel_defines.h"
#include "sw_mem.h"
#include "sw_lock_defs.h"
#include "sw_overhead_measurements.h"

/* -------------------------------------------------
 * Compile time constants and macros.
 * -------------------------------------------------
 */
#define NUM_SEGS_PER_BUFFER 2 /* MUST be pow 2! */
#define NUM_SEGS_PER_BUFFER_MASK (NUM_SEGS_PER_BUFFER - 1)
/*
 * The size of the 'buffer' data array in each segment.
 */
#define SW_SEG_DATA_SIZE (sw_buffer_alloc_size)
/*
 * Min size of per-cpu output buffers.
 */
#define SW_MIN_SEG_SIZE_BYTES (1 << 10) /* 1kB */
#define SW_MIN_OUTPUT_BUFFER_SIZE (SW_MIN_SEG_SIZE_BYTES * NUM_SEGS_PER_BUFFER)
/*
 * A symbolic constant for an empty buffer index.
 */
#define EMPTY_SEG (-1)
/*
 * How much space is available in a given segment?
 */
#define EMPTY_TSC ((u64)-1)
#define SEG_IS_FULL(seg) ({bool __full = false; \
	smp_mb(); /* memory access ordering */\
	__full = ((seg)->is_full != EMPTY_TSC); \
	__full; })
#define SEG_SET_FULL(seg, tsc) do { \
	(seg)->is_full = (tsc); \
	smp_mb(); /* memory access ordering */\
} while (0)
#define SEG_SET_EMPTY(seg) do { \
	barrier(); \
	(seg)->bytes_written = 0; \
	SEG_SET_FULL(seg, EMPTY_TSC); \
} while (0)
#define SPACE_AVAIL(seg) (SW_SEG_DATA_SIZE - (seg)->bytes_written)
#define SEG_IS_EMPTY(seg) (SPACE_AVAIL(seg) == SW_SEG_DATA_SIZE)

#define GET_OUTPUT_BUFFER(cpu) (&per_cpu_output_buffers[(cpu)])
/*
 * Convenience macro: iterate over each segment in a per-cpu output buffer.
 */
#define for_each_segment(i) for (i = 0; i < NUM_SEGS_PER_BUFFER; ++i)
#define for_each_seg(buffer, seg)					 \
	for (int i = 0;							 \
		i < NUM_SEGS_PER_BUFFER && (seg = (buffer)->segments[i]);\
		++i)
/*
 * How many buffers are we using?
 */
#define GET_NUM_OUTPUT_BUFFERS() (sw_max_num_cpus + 1)
/*
 * Convenience macro: iterate over each per-cpu output buffer.
 */
#define for_each_output_buffer(i) for (i = 0; i < GET_NUM_OUTPUT_BUFFERS(); ++i)

/* -------------------------------------------------
 * Local data structures.
 * -------------------------------------------------
 */
struct sw_data_buffer {
	u64 is_full;
	u32 bytes_written;
	char *buffer;
} __packed;
#define SW_SEG_HEADER_SIZE() (sizeof(struct sw_data_buffer) - sizeof(char *))

struct sw_output_buffer {
	struct sw_data_buffer buffers[NUM_SEGS_PER_BUFFER];
	int buff_index;
	u32 produced_samples;
	u32 dropped_samples;
	int last_seg_read;
	unsigned int mem_alloc_size;
	unsigned long free_pages;
} ____cacheline_aligned_in_smp;

/* *************************************************
 * For circular buffer (continuous profiling)
 * *************************************************
 */
static char *output_buffer;

struct buffer {
	union {
		char *data;
		unsigned long free_pages;
	};
	size_t read_index, write_index;
	unsigned long size;
};
SW_DECLARE_RWLOCK(sw_continuous_lock);

static struct buffer buffer; /* TODO: rename */

/* -------------------------------------------------
 * Function declarations.
 * -------------------------------------------------
 */
extern u64 sw_timestamp(void);

/* -------------------------------------------------
 * Variable definitions.
 * -------------------------------------------------
 */
u64 sw_num_samples_produced = 0, sw_num_samples_dropped = 0;
int sw_max_num_cpus = -1;

DECLARE_OVERHEAD_VARS(sw_produce_generic_msg_i);
/*
 * Per-cpu output buffers.
 */
static struct sw_output_buffer *per_cpu_output_buffers;
/*
 * Variables for book keeping.
 */
static volatile int sw_last_cpu_read = -1;
static volatile s32 sw_last_mask = -1;
/*
 * Lock for the polled buffer.
 */
SW_DECLARE_SPINLOCK(sw_polled_lock);
/*
 * Buffer allocation size.
 */
unsigned long sw_buffer_alloc_size = (1 << 16); /* 64 KB */

/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */

/* *************************************************
 * For circular buffer (continuous profiling)
 * *************************************************
 */
#define MIN(x, y) ((x) <= (y) ? (x) : (y))

#define IS_BUFFER_EMPTY(buffer)					\
	((buffer).write_index == (buffer).read_index)
#define IS_BUFFER_FULL(buffer)					\
	((buffer).write_index ==				\
		((buffer).read_index + 1) & (buffer.size - 1))

static inline size_t get_space_available(struct buffer *buffer)
{
	size_t read = 0, write = 0;

	smp_mb(); /* memory access ordering */
	read = buffer->read_index;
	write = buffer->write_index;
	if (write < read)
		return read - write;

	return (buffer->size - write) + read;
}

static inline size_t get_data_available(struct buffer *buffer)
{
	size_t read = 0, write = 0;

	smp_mb(); /* memory access ordering */
	read = buffer->read_index;
	write = buffer->write_index;
	if (read <= write)
		return write - read;

	return (buffer->size - read) + write;
}

static void copy_wraparound(const char *src, size_t src_size, size_t *index)
{
	size_t buff_size_left = buffer.size - *index;
	size_t to_write = MIN(buff_size_left, src_size);
	size_t _index = *index;

	if (src_size < buff_size_left) {
		memcpy(&buffer.data[_index], src, src_size);
		_index += src_size;
	} else {
		memcpy(&buffer.data[_index], src, to_write);
		_index = 0;
		src += to_write;
		to_write = src_size - to_write;
		memcpy(&buffer.data[_index], src, to_write);
		_index += to_write;
		pw_pr_debug("DEBUG: wrap memcpy\n");
	}
	*index = (*index + src_size) & (buffer.size - 1);
}

static int enqueue_data(struct sw_driver_msg *msg, enum sw_wakeup_action action)
{
	size_t size = SW_DRIVER_MSG_HEADER_SIZE() + msg->payload_len;
	bool wrapped = false;

	msg->tsc = 0;

	READ_LOCK(sw_continuous_lock);
	while (true) {
		size_t old_write_index = buffer.write_index;
		size_t new_write_index = (old_write_index + size) &
						(buffer.size - 1);

		if (get_space_available(&buffer) < size)
			break;

		if (CAS32(&buffer.write_index, old_write_index,
				new_write_index)) {
			msg->tsc = sw_timestamp();
			wrapped = new_write_index <= old_write_index;
			/* First copy header */
			copy_wraparound((const char *)msg,
				SW_DRIVER_MSG_HEADER_SIZE(), &old_write_index);
			/* Then copy payload */
			copy_wraparound((const char *)msg->p_payload,
				msg->payload_len, &old_write_index);
			break;
		}
	}
	READ_UNLOCK(sw_continuous_lock);
	if (!msg->tsc)
		pw_pr_error("ERROR: couldn't enqueue data\n");
	if (wrapped)
		pw_pr_debug("DEBUG: wrapped!\n");

	return msg->tsc ? 0 : -1;
}

/*
 * Returns # of bytes successfully consumed on success
 * 0 on EOF (no error condition)
 */
static size_t consume_buffer(void *dest, size_t bytes_to_read)
{
	size_t read_index = 0, write_index = 0, dst_index = 0;
	size_t to_read = 0;
	bool wrapped = false;
	size_t read_size = bytes_to_read;
	unsigned long bytes_not_copied = 0;
	struct sw_driver_continuous_collect data = {0};

	WRITE_LOCK(sw_continuous_lock);
	smp_mb(); /* memory access ordering */
	read_index = buffer.read_index;
	write_index = buffer.write_index;
	/* EXE sends size as header + payload; we only want payload */
	read_size -= SW_DRIVER_CONTINUOUS_COLLECT_HEADER_SIZE();
	data.collection_size = to_read =
		MIN(read_size, get_data_available(&buffer));
	pw_pr_debug(
		"DEBUG: read = %zu, write = %zu, avail = %zu, to_read = %zu\n",
		read_index, write_index, get_data_available(&buffer), to_read);
	while (to_read) {
		size_t curr_read = to_read;

		if (read_index + to_read > buffer.size) {
			curr_read = buffer.size - read_index;
			wrapped = true;
			pw_pr_debug(
				"DEBUG: read = %zu, to_read = %zu, curr_read = %zu, buffer.size = %lu, WRAPPED!\n",
				read_index, to_read, curr_read, buffer.size);
		}
		memcpy(&output_buffer[dst_index],
			&buffer.data[read_index], curr_read);
		read_index = (read_index + curr_read) & (buffer.size - 1);
		to_read -= curr_read;
		dst_index += curr_read;
	}
	buffer.read_index = read_index;
	smp_mb(); /* memory access ordering */
	pw_pr_debug("DEBUG: read at end of while = %zu\n", buffer.read_index);
	WRITE_UNLOCK(sw_continuous_lock);

	/*
	 * Call 'copy_to_user' instead of 'sw_copy_to_user' since
	 * sw_copy_to_user expects to see a 'struct uio' while this
	 * is called from an IOCTL which does NOT have a 'struct uio'
	 */
	bytes_not_copied =
	copy_to_user(dest, (char *)&data,
		SW_DRIVER_CONTINUOUS_COLLECT_HEADER_SIZE());
	if (bytes_not_copied)
		return 0;

	pw_pr_debug("DEBUG: collection size = %u\n", data.collection_size);
	if (data.collection_size) {
		bytes_not_copied =
			copy_to_user(dest +
				SW_DRIVER_CONTINUOUS_COLLECT_HEADER_SIZE(),
				output_buffer, data.collection_size);
		if (bytes_not_copied)
			return 0;

	}
	return data.collection_size;
}

long initialize_circular_buffer(size_t size)
{
	size_t alloc_size = size, read_size = size;
	/*
	 * We require a power of two size
	 */
	pw_pr_debug("DEBUG: old alloc size = %zu\n", alloc_size);
	if ((alloc_size & (alloc_size - 1)) != 0)
		alloc_size = 1 << fls(alloc_size);

	pw_pr_debug("DEBUG: new alloc size = %zu\n", alloc_size);
	/* Create double-sized buffer */
	alloc_size <<= 1;
	pw_pr_debug("DEBUG: double alloc size = %zu\n", alloc_size);
	memset(&buffer, 0, sizeof(buffer));
	buffer.free_pages =
		sw_allocate_pages(GFP_KERNEL | __GFP_ZERO, alloc_size);
	if (!buffer.free_pages) {
		pw_pr_error("Couldn't allocate space for buffer!\n");
		return -ENOMEM;
	}
	buffer.read_index = buffer.write_index = 0;
	buffer.size = alloc_size;
	SW_INIT_RWLOCK(sw_continuous_lock);
	/*
	 * Create temp output buffer
	 */
	output_buffer = vmalloc(read_size);
	if (!output_buffer) {
		pw_pr_error(
			"Couldn't create temporary buffer for data output!\n");
		return -ENOMEM;
	}
	return 0;
}

void reset_output_buffers(void)
{
	buffer.read_index = buffer.write_index = 0;
}


void destroy_circular_buffer(void)
{
	if (buffer.free_pages) {
		sw_release_pages(buffer.free_pages, buffer.size);
		buffer.free_pages = 0;
	}
	if (output_buffer) {
		vfree(output_buffer);
		output_buffer = NULL;
	}
	SW_DESTROY_RWLOCK(sw_continuous_lock);
	pw_pr_debug("DEBUG: read = %zu, write = %zu\n", buffer.read_index,
	buffer.write_index);
}

/* *************************************************
 * For per-cpu buffers (non circular)
 * *************************************************
 */

static char *reserve_seg_space_i(size_t size, int cpu, bool *should_wakeup,
	u64 *reservation_tsc)
{
	struct sw_output_buffer *buffer = GET_OUTPUT_BUFFER(cpu);
	int i = 0;
	int buff_index = buffer->buff_index;
	char *dst = NULL;

	if (buff_index < 0 || buff_index >= NUM_SEGS_PER_BUFFER)
		goto prod_seg_done;

	for_each_segment(i) {
		struct sw_data_buffer *seg = &buffer->buffers[buff_index];

		if (SEG_IS_FULL(seg) == false) {
			if (SPACE_AVAIL(seg) >= size) {
				*reservation_tsc = sw_timestamp();
				dst = &seg->buffer[seg->bytes_written];
				seg->bytes_written += size;
				smp_mb(); /* memory access ordering */
				buffer->buff_index = buff_index;
				buffer->produced_samples++;
				goto prod_seg_done;
			}
			SEG_SET_FULL(seg, sw_timestamp());
		}
		buff_index = CIRCULAR_INC(buff_index, NUM_SEGS_PER_BUFFER_MASK);
		*should_wakeup = true;
	}
prod_seg_done:
	if (!dst)
		buffer->dropped_samples++;

	return dst;
};

#ifdef CONFIG_PREEMPT_COUNT
static int produce_polled_msg(struct sw_driver_msg *msg,
	enum sw_wakeup_action action)
{
	int cpu = GET_POLLED_CPU();
	bool should_wakeup = false;
	int retVal = PW_SUCCESS;

	if (!msg)
		return -PW_ERROR;

	pw_pr_debug("POLLED! cpu = %d\n", cpu);
	LOCK(sw_polled_lock);
	{
		size_t size = SW_DRIVER_MSG_HEADER_SIZE() + msg->payload_len;
		char *dst = reserve_seg_space_i(size, cpu,
						&should_wakeup, &msg->tsc);

		if (dst) {
			/*
			 * Assign a special CPU number to this CPU.
			 * This is OK, because messages enqueued in this buffer
			 * are always CPU agnostic (otherwise they would
			 * be invoked from within a preempt_disable()d context
			 * in 'sw_handle_collector_node_i()', which ensures
			 * they will be enqueued within the
			 * 'sw_produce_generic_msg_on_cpu()' function).
			 */
			msg->cpuidx = cpu;
			memcpy(dst, msg, SW_DRIVER_MSG_HEADER_SIZE());
			dst += SW_DRIVER_MSG_HEADER_SIZE();
			memcpy(dst, msg->p_payload, msg->payload_len);
		} else {
			pw_pr_debug("NO space in polled msg!\n");
			retVal = -PW_ERROR;
		}
	}
	UNLOCK(sw_polled_lock);
	if (unlikely(should_wakeup))
		sw_wakeup_reader(action);

	return retVal;
};
#endif /* CONFIG_PREEMPT_COUNT */

static int sw_produce_generic_msg_i(struct sw_driver_msg *msg,
	enum sw_wakeup_action action)
{
	int retval = PW_SUCCESS;
	bool should_wakeup = false;
	int cpu = -1;
	unsigned long flags = 0;

	if (!msg) {
		pw_pr_error("ERROR: CANNOT produce a NULL msg!\n");
		return -PW_ERROR;
	}

	/* Check if we need to use circular buffer */
	if (output_buffer)
		return enqueue_data(msg, action);

#ifdef CONFIG_PREEMPT_COUNT
	if (!in_atomic())
		return produce_polled_msg(msg, action);
#endif

	cpu = sw_get_cpu(&flags);
	{
		size_t size = msg->payload_len +
				SW_DRIVER_MSG_HEADER_SIZE();
		char *dst = reserve_seg_space_i(size, cpu, &should_wakeup,
						&msg->tsc);

		if (likely(dst)) {
			memcpy(dst, msg, SW_DRIVER_MSG_HEADER_SIZE());
			dst += SW_DRIVER_MSG_HEADER_SIZE();
			memcpy(dst, msg->p_payload, msg->payload_len);
		} else
			retval = -PW_ERROR;
	}
	sw_put_cpu(flags);

	if (unlikely(should_wakeup))
		sw_wakeup_reader(action);

	return retval;
};

int sw_produce_generic_msg(struct sw_driver_msg *msg,
	enum sw_wakeup_action action)
{
	return DO_PER_CPU_OVERHEAD_FUNC_RET(int, sw_produce_generic_msg_i,
		msg, action);
};

static int sw_init_per_cpu_buffers_i(unsigned long per_cpu_mem_size)
{
	int cpu = -1;

	per_cpu_output_buffers =
	(struct sw_output_buffer *)sw_kmalloc(sizeof(struct sw_output_buffer) *
	GET_NUM_OUTPUT_BUFFERS(), GFP_KERNEL | __GFP_ZERO);
	if (per_cpu_output_buffers == NULL) {
		pw_pr_error(
			"ERROR allocating space for per-cpu output buffers!\n");
		sw_destroy_per_cpu_buffers();
		return -PW_ERROR;
	}
	for_each_output_buffer(cpu) {
		struct sw_output_buffer *buffer = &per_cpu_output_buffers[cpu];
		char *buff = NULL;
		int i = 0;

		buffer->mem_alloc_size = per_cpu_mem_size;
		buffer->free_pages = sw_allocate_pages(GFP_KERNEL | __GFP_ZERO,
			(unsigned int)per_cpu_mem_size);
		if (buffer->free_pages == 0) {
			pw_pr_error("ERROR allocating pages for buffer [%d]!\n",
				cpu);
			sw_destroy_per_cpu_buffers();
			return -PW_ERROR;
		}
		buff = (char *)buffer->free_pages;
		for_each_segment(i) {
			buffer->buffers[i].buffer = (char *)buff;
			buff += SW_SEG_DATA_SIZE;
		}
	}
	pw_pr_debug("PER_CPU_MEM_SIZE = %lu, order = %u\n",
	(unsigned long)per_cpu_mem_size, get_order(per_cpu_mem_size));
	return PW_SUCCESS;
};

int sw_init_per_cpu_buffers(void)
{
	unsigned int per_cpu_mem_size = sw_get_output_buffer_size();

	pw_pr_debug("Buffer alloc size = %ld\n", sw_buffer_alloc_size);

	if (GET_NUM_OUTPUT_BUFFERS() <= 0) {
		pw_pr_error("ERROR: max # output buffers= %d\n",
			GET_NUM_OUTPUT_BUFFERS());
		return -PW_ERROR;
	}

	pw_pr_debug("DEBUG: sw_max_num_cpus = %d, num output buffers = %d\n",
	sw_max_num_cpus, GET_NUM_OUTPUT_BUFFERS());

	/*
	 * Try to allocate per-cpu buffers. If allocation fails, decrease
	 * buffer size and retry. Stop trying if size drops below 2KB
	 * (which means 1KB for each buffer).
	 */
	while (per_cpu_mem_size >= SW_MIN_OUTPUT_BUFFER_SIZE &&
		sw_init_per_cpu_buffers_i(per_cpu_mem_size)) {
		pw_pr_debug("WARNING: couldn't allocate per-cpu buffers with size %u -- trying smaller size!\n",
			per_cpu_mem_size);
		sw_buffer_alloc_size >>= 1;
		per_cpu_mem_size = sw_get_output_buffer_size();
	}

	if (unlikely(per_cpu_output_buffers == NULL)) {
		pw_pr_error("ERROR: couldn't allocate space for per-cpu output buffers!\n");
		return -PW_ERROR;
	}
	/*
	 * Initialize our locks.
	 */
	SW_INIT_SPINLOCK(sw_polled_lock);

	pw_pr_debug("OK, allocated per-cpu buffers with size = %lu\n",
		(unsigned long)per_cpu_mem_size);

	if (sw_init_reader_queue()) {
		pw_pr_error("ERROR initializing reader subsys\n");
		return -PW_ERROR;
	}

	return PW_SUCCESS;
};

void sw_destroy_per_cpu_buffers(void)
{
	int cpu = -1;

	/*
	 * Perform lock finalization.
	 */
	SW_DESTROY_SPINLOCK(sw_polled_lock);

	if (per_cpu_output_buffers != NULL) {
		for_each_output_buffer(cpu) {
			struct sw_output_buffer *buffer =
					&per_cpu_output_buffers[cpu];

			if (buffer->free_pages != 0) {
				sw_release_pages(buffer->free_pages,
					buffer->mem_alloc_size);
				buffer->free_pages = 0;
			}
		}
		sw_kfree(per_cpu_output_buffers);
		per_cpu_output_buffers = NULL;
	}
};

void sw_reset_per_cpu_buffers(void)
{
	int cpu = 0, i = 0;

	for_each_output_buffer(cpu) {
		struct sw_output_buffer *buffer = GET_OUTPUT_BUFFER(cpu);

		buffer->buff_index = buffer->dropped_samples =
			buffer->produced_samples = 0;
		buffer->last_seg_read = -1;

		for_each_segment(i) {
			struct sw_data_buffer *seg = &buffer->buffers[i];

			memset(seg->buffer, 0, SW_SEG_DATA_SIZE);
			SEG_SET_EMPTY(seg);
		}
	}
	sw_last_cpu_read = -1;
	sw_last_mask = -1;
	pw_pr_debug("OK, reset per-cpu output buffers!\n");
	/*
	 * Reset circular buffer if it has been allocated
	 */
	if (output_buffer)
		buffer.read_index = buffer.write_index = 0;

};

bool sw_any_seg_full(u32 *val, bool is_flush_mode)
{
	int num_visited = 0, i = 0;

	if (!val) {
		pw_pr_error("ERROR: NULL ptrs in %s!\n", __func__);
		return false;
	}

	*val = SW_NO_DATA_AVAIL_MASK;
	pw_pr_debug("Checking for full seg: val = %u, flush = %s\n",
		 *val, GET_BOOL_STRING(is_flush_mode));
	for_each_output_buffer(num_visited) {
		int min_seg = EMPTY_SEG, non_empty_seg = EMPTY_SEG;
		u64 min_tsc = EMPTY_TSC;
		struct sw_output_buffer *buffer = NULL;

		if (++sw_last_cpu_read >= GET_NUM_OUTPUT_BUFFERS())
			sw_last_cpu_read = 0;

		buffer = GET_OUTPUT_BUFFER(sw_last_cpu_read);
		for_each_segment(i) {
			struct sw_data_buffer *seg = &buffer->buffers[i];
			u64 seg_tsc = seg->is_full;

			if (SEG_IS_EMPTY(seg))
				continue;

			non_empty_seg = i;
			if (seg_tsc < min_tsc) {
				/*
				 * Can only happen if seg was full, provided
				 * 'EMPTY_TSC' is set to "(u64)-1"
				 */
				min_tsc = seg_tsc;
				min_seg = i;
			}
		}
		if (min_seg != EMPTY_SEG) {
			*val = (sw_last_cpu_read & 0xffff) << 16 |
				(min_seg & 0xffff);
			return true;
		} else if (is_flush_mode && non_empty_seg != EMPTY_SEG) {
			*val = (sw_last_cpu_read & 0xffff) << 16 |
				(non_empty_seg & 0xffff);
			return true;
		}
	}
	/*
	 * Reaches here only if there's no data to be read.
	 */
	if (is_flush_mode) {
		/*
		 * We've drained all buffers and need to tell the userspace
		 * application there isn't any data. Unfortunately, we can't
		 * just return a 'zero' value for the mask (because that could
		 * also indicate that segment # 0 of cpu #0 has data).
		 */
		*val = SW_ALL_WRITES_DONE_MASK;
		return true;
	}
	return false;
};

/*
 * Returns: number of bytes consumed on SUCCESS, 0 on EOF, negative
 * error code on FAILURE
 */
ssize_t sw_consume_data(u32 mask, void __user *buffer, size_t bytes_to_read)
{
	int which_cpu = -1, which_seg = -1;
	unsigned long bytes_not_copied = 0;
	struct sw_output_buffer *buff = NULL;
	struct sw_data_buffer *seg = NULL;
	size_t bytes_read = 0;

	/* Check if we need to use circular buffer */
	if (output_buffer)
		return (ssize_t)consume_buffer(buffer, bytes_to_read);

	if (!sw_check_output_buffer_params(buffer, bytes_to_read,
			SW_SEG_DATA_SIZE)) {
		pw_pr_error("ERROR: invalid params to \"%s\"!\n", __func__);
		return -EIO;
	}

	which_cpu = mask >> 16; which_seg = mask & 0xffff;
	pw_pr_debug("CONSUME: cpu = %d, seg = %d\n", which_cpu, which_seg);
	if (which_seg >= NUM_SEGS_PER_BUFFER) {
		pw_pr_error(
			"Error: which_seg (%d) >= NUM_SEGS_PER_BUFFER (%d)\n",
			which_seg, NUM_SEGS_PER_BUFFER);
		return -EIO;
	}
	/*
	 * OK to access unlocked; either the segment is FULL, or no collection
	 * is ongoing. In either case, we're GUARANTEED no producer is touching
	 * this segment.
	 */
	buff = GET_OUTPUT_BUFFER(which_cpu);
	seg = &buff->buffers[which_seg];

	bytes_not_copied = sw_copy_to_user(buffer,
		seg->buffer, seg->bytes_written); /* dst, src */

	if (likely(bytes_not_copied == 0))
		bytes_read = seg->bytes_written;
	else {
		pw_pr_error("Warning: couldn't copy %lu bytes\n",
			bytes_not_copied);
		bytes_read = 0;
	}
	SEG_SET_EMPTY(seg);
	return bytes_read;
}

unsigned int sw_get_output_buffer_size(void)
{
	return (sw_buffer_alloc_size * NUM_SEGS_PER_BUFFER);
};

void sw_count_samples_produced_dropped(void)
{
	int cpu = 0;

	sw_num_samples_produced = sw_num_samples_dropped = 0;
	if (per_cpu_output_buffers == NULL)
		return;

	for_each_output_buffer(cpu) {
		struct sw_output_buffer *buff = GET_OUTPUT_BUFFER(cpu);

		sw_num_samples_dropped += buff->dropped_samples;
		sw_num_samples_produced += buff->produced_samples;
	}
};

void sw_print_output_buffer_overheads(void)
{
	PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_produce_generic_msg_i,
		"PRODUCE_GENERIC_MSG");
	sw_print_reader_stats();
};
