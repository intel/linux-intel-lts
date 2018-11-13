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
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "lwpmudrv_types.h"
#include "lwpmudrv.h"
#include "lwpmudrv_ecb.h"
#include "lwpmudrv_struct.h"

#include "control.h"
#include "output.h"
#include "utility.h"
#include "inc/linuxos.h"
#define OTHER_C_DEVICES 1 // one for module

/*
 *  Global data: Buffer control structure
 */
static wait_queue_head_t flush_queue;
static atomic_t flush_writers;
static volatile int flush;
extern DRV_CONFIG drv_cfg;
extern DRV_BOOL multi_pebs_enabled;
extern DRV_BOOL sched_switch_enabled;
extern DRV_BOOL unc_buf_init;

static void output_NMI_Sample_Buffer(unsigned long data);

/*
 *  @fn output_Free_Buffers(output, size)
 *
 *  @param    IN  outbuf      - The output buffer to manipulate
 *
 *  @brief   Deallocate the memory associated with the buffer descriptor
 *
 */
static VOID output_Free_Buffers(BUFFER_DESC buffer, size_t size)
{
	int j;
	OUTPUT outbuf;

	SEP_DRV_LOG_TRACE_IN("Buffer: %p, size: %u.", buffer, size);

	if (buffer == NULL) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!buffer).");
		return;
	}
	outbuf = &BUFFER_DESC_outbuf(buffer);
	for (j = 0; j < OUTPUT_NUM_BUFFERS; j++) {
		CONTROL_Free_Memory(OUTPUT_buffer(outbuf, j));
		OUTPUT_buffer(outbuf, j) = NULL;
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/* ------------------------------------------------------------------------- */
/*!
 *  @fn  int OUTPUT_Reserve_Buffer_Space (OUTPUT      outbuf,
 *                                        U32         size,
 *                                        U8          in_notification)
 *
 *  @param  outbuf          IN output buffer to manipulate
 *  @param  size            IN The size of data to reserve
 *  @param  defer           IN wake up directly if FALSE.
 *                           Otherwise, see below.
 *  @param  in_notification IN 1 if in notification, 0 if not
 *
 *  @result outloc - to the location where data is to be written
 *
 *  Reserve space in the output buffers for data. The behavior of this function
 *  when a buffer is full will vary depending on the 'defer' and 'in_notification'
 *  parameters, as described in the special notes section.
 *
 * <I>Special Notes:</I>
 *  -----------------------------------------------------------------------------------------------------------------------
 *  defer | in_notification |                                         description
 *  -----------------------------------------------------------------------------------------------------------------------
 *  FALSE |    FALSE/TRUE   | directly signals the buffer's consumer with wake_up_interruptible_sync
 *  -----------------------------------------------------------------------------------------------------------------------
 *   TRUE |      FALSE      | defers the call to wake_up_interruptible_sync using tasklet_schedule [needed because calling
 *        |                 | it directly is not safe from an NMI]
 *  -----------------------------------------------------------------------------------------------------------------------
 *        |                 | do not signal -or explicitly schedule the signaling of- the buffer's consumer [needed because
 *   TRUE |       TRUE      | neither operation is safe from the sched_switch tracepoint callback in kernel version 4.13].
 *        |                 | Instead relies on the interrupt handler to do it next time there is an interrupt.
 *  -----------------------------------------------------------------------------------------------------------------------
 */
void *OUTPUT_Reserve_Buffer_Space(BUFFER_DESC bd, U32 size,
					 DRV_BOOL defer, U8 in_notification,
					 S32 cpu_idx)
{
	char *outloc = NULL;
	OUTPUT outbuf = &BUFFER_DESC_outbuf(bd);
	S32 this_cpu;

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(
		in_notification, "Bd: %p, size: %u, defer: %u, notif: %u.", bd,
		size, defer, in_notification);

	if (DRV_CONFIG_enable_cp_mode(drv_cfg) && flush) {
		SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(
			in_notification, "Res: NULL (cp_mode && flush).");
		return NULL;
	}

	if (OUTPUT_remaining_buffer_size(outbuf) >= size) {
		outloc = (char *)
			(OUTPUT_buffer(outbuf, OUTPUT_current_buffer(outbuf)) +
			(OUTPUT_total_buffer_size(outbuf) -
				OUTPUT_remaining_buffer_size(outbuf)));
	} else {
		U32 i, j, start;
		OUTPUT_buffer_full(outbuf, OUTPUT_current_buffer(outbuf)) =
			OUTPUT_total_buffer_size(outbuf) -
			OUTPUT_remaining_buffer_size(outbuf);

		//
		// Massive Naive assumption:  Must find a way to fix it.
		// In spite of the loop.
		// The next buffer to fill are monotonically increasing
		// indicies.
		//
		if (!DRV_CONFIG_enable_cp_mode(drv_cfg)) {
			OUTPUT_signal_full(outbuf) = TRUE;
		}

		start = OUTPUT_current_buffer(outbuf);
		for (i = start + 1; i < start + OUTPUT_NUM_BUFFERS; i++) {
			j = i % OUTPUT_NUM_BUFFERS;

			//don't check if buffer has data when doing CP
			if (!OUTPUT_buffer_full(outbuf, j) ||
			    (DRV_CONFIG_enable_cp_mode(drv_cfg))) {
				OUTPUT_current_buffer(outbuf) = j;
				OUTPUT_remaining_buffer_size(outbuf) =
					OUTPUT_total_buffer_size(outbuf);
				outloc = (char *)OUTPUT_buffer(outbuf, j);
				if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
					// discarding all the information in the new buffer in CP mode
					OUTPUT_buffer_full(outbuf, j) = 0;
					break;
				}
			}
#if !(defined(CONFIG_PREEMPT_RT) || defined(CONFIG_PREEMPT_RT_FULL))
			else {
				if (!defer) {
					OUTPUT_signal_full(outbuf) = FALSE;
					SEP_DRV_LOG_NOTIFICATION_WARNING(
						in_notification,
						"Output buffers are full. Might be dropping some samples!");
					break;
				}
			}
#endif
		}
	}

	if (outloc) {
		OUTPUT_remaining_buffer_size(outbuf) -= size;
		memset(outloc, 0, size);
	}

	if (OUTPUT_signal_full(outbuf)) {
		if (!defer) {
#if !(defined(CONFIG_PREEMPT_RT) || defined(CONFIG_PREEMPT_RT_FULL))
			SEP_DRV_LOG_NOTIFICATION_TRACE(
				in_notification,
				"Choosing direct wakeup approach.");
#if !defined(DRV_SEP_ACRN_ON)
			wake_up_interruptible_sync(&BUFFER_DESC_queue(bd));
#endif
			OUTPUT_signal_full(outbuf) = FALSE;
#endif
		} else {
			if (!OUTPUT_tasklet_queued(outbuf)) {
				if (cpu_idx == -1) {
					this_cpu = CONTROL_THIS_CPU();
				} else {
					this_cpu = cpu_idx;
				}
				if (!in_notification) {
					SEP_DRV_LOG_NOTIFICATION_TRACE(
						in_notification,
						"Scheduling the tasklet on cpu %u.",
						this_cpu);
					OUTPUT_tasklet_queued(outbuf) = TRUE;
#if !defined(DRV_SEP_ACRN_ON)
					tasklet_schedule(&CPU_STATE_nmi_tasklet(
						&pcb[this_cpu]));
#endif
				} else {
					static U32 cpt;

					if (!cpt) {
						SEP_DRV_LOG_WARNING(
							"Using interrupt-driven sideband buffer flushes for extra safety.");
						SEP_DRV_LOG_WARNING(
							"This may result in fewer context switches being recorded.");
					}
					SEP_DRV_LOG_TRACE(
						"Lost context switch information (for the %uth time).",
						++cpt);
				}
			}
		}
	}

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(in_notification, "Res: %p.", outloc);
	return outloc;
}

/* ------------------------------------------------------------------------- */
/*!
 *
 * @fn  int  OUTPUT_Buffer_Fill (BUFFER_DESC buf,
 *                               PVOID  data,
 *                               U16    size,
 *                               U8     in_notification)
 *
 * @brief     Place a record (can be module, marker, etc) in a buffer
 *
 * @param     data            - pointer to a buffer to copy
 * @param     size            - size of the buffer to cpu
 * @param     in_notification - 1 if in notification, 0 if not
 *
 * @return    number of bytes copied into buffer
 *
 * Start by ensuring that output buffer space is available.
 * If so, then copy the input data to the output buffer and make the necessary
 * adjustments to manage the output buffers.
 * If not, signal the read event for this buffer and get another buffer.
 *
 * <I>Special Notes:</I>
 *
 */
static int output_Buffer_Fill(BUFFER_DESC bd, PVOID data, U16 size,
			      U8 in_notification)
{
	char *outloc;

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(
		in_notification, "Bd: %p, data: %p, size: %u.", bd, data, size);

	outloc = (char *)OUTPUT_Reserve_Buffer_Space(bd, size,
			FALSE, in_notification, -1);
	if (outloc) {
		memcpy(outloc, data, size);
		SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(in_notification,
						   "Res: %d (outloc).", size);
		return size;
	}

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(in_notification,
					   "Res: 0 (!outloc).");
	return 0;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn  int  OUTPUT_Module_Fill (PVOID  data,
 *                               U16    size,
 *                               U8     in_notification)
 *
 * @brief     Place a module record in a buffer
 *
 * @param     data              - pointer to a buffer to copy
 * @param     size              - size of the buffer to cpu
 * @param     in_notification   - 1 if in notification, 0 if not
 *
 * @return    number of bytes copied into buffer
 *
 *
 */
int OUTPUT_Module_Fill(PVOID data, U16 size, U8 in_notification)
{
	int ret_size;
	OUTPUT outbuf = &BUFFER_DESC_outbuf(module_buf);

	SEP_DRV_LOG_NOTIFICATION_TRACE_IN(in_notification,
					  "Data: %p, size: %u.", data, size);

	spin_lock(&OUTPUT_buffer_lock(outbuf));
	ret_size = output_Buffer_Fill(module_buf, data, size, in_notification);
	spin_unlock(&OUTPUT_buffer_lock(outbuf));

	SEP_DRV_LOG_NOTIFICATION_TRACE_OUT(in_notification, "Res: %d.",
					   ret_size);
	return ret_size;
}

/* ------------------------------------------------------------------------- */
/*!
 *  @fn  ssize_t  output_Read(struct file  *filp,
 *                            char         *buf,
 *                            size_t        count,
 *                            loff_t       *f_pos,
 *                            BUFFER_DESC   kernel_buf)
 *
 *  @brief  Return a sample buffer to user-mode. If not full or flush, wait
 *
 *  @param *filp          a file pointer
 *  @param *buf           a sampling buffer
 *  @param  count         size of the user's buffer
 *  @param  f_pos         file pointer (current offset in bytes)
 *  @param  kernel_buf    the kernel output buffer structure
 *
 *  @return number of bytes read. zero indicates end of file. Neg means error
 *
 *  Place no more than count bytes into the user's buffer.
 *  Block if unavailable on "BUFFER_DESC_queue(buf)"
 *
 * <I>Special Notes:</I>
 *
 */
static ssize_t output_Read(struct file *filp, char __user *buf, size_t count,
			   loff_t *f_pos, BUFFER_DESC kernel_buf)
{
	ssize_t to_copy = 0;
	ssize_t uncopied;
	OUTPUT outbuf = &BUFFER_DESC_outbuf(kernel_buf);
	U32 cur_buf, i;
	/* Buffer is filled by output_fill_modules. */

	SEP_DRV_LOG_TRACE_IN(
		"Filp: %p, buf: %p, count: %u, f_pos: %p, kernel_buf: %p.",
		filp, buf, (U32)count, f_pos, kernel_buf);

	cur_buf = OUTPUT_current_buffer(outbuf);
	if (!DRV_CONFIG_enable_cp_mode(drv_cfg) || flush) {
		for (i = 0; i < OUTPUT_NUM_BUFFERS; i++) {
			//iterate through all buffers
			cur_buf++;
			if (cur_buf >= OUTPUT_NUM_BUFFERS) {
				cur_buf = 0;
			} //circularly
			to_copy = OUTPUT_buffer_full(outbuf, cur_buf);
			if (to_copy != 0) {
				if (flush &&
				    DRV_CONFIG_enable_cp_mode(drv_cfg) &&
				    cur_buf == OUTPUT_current_buffer(outbuf)) {
					OUTPUT_current_buffer(outbuf)++;
					if (OUTPUT_current_buffer(outbuf) >=
					    OUTPUT_NUM_BUFFERS) {
						OUTPUT_current_buffer(outbuf) =
							0;
					}
					OUTPUT_remaining_buffer_size(outbuf) =
						OUTPUT_total_buffer_size(
							outbuf);
				}
				break;
			}
		}
	}

	SEP_DRV_LOG_TRACE("buffer %d has %d bytes ready.", (S32)cur_buf,
			  (S32)to_copy);
	if (!flush && to_copy == 0) {
		unsigned long delay = msecs_to_jiffies(1000);

		while (1) {
			U32 res = wait_event_interruptible_timeout(
				BUFFER_DESC_queue(kernel_buf),
				flush || (OUTPUT_buffer_full(outbuf, cur_buf) &&
					  !DRV_CONFIG_enable_cp_mode(drv_cfg)),
				delay);

			if (GET_DRIVER_STATE() == DRV_STATE_TERMINATING) {
				SEP_DRV_LOG_INIT(
					"Switched to TERMINATING while waiting for BUFFER_DESC_queue!");
				break;
			}

			if (res == ERESTARTSYS || res == 0) {
				SEP_DRV_LOG_TRACE(
					"Wait_event_interruptible_timeout(BUFFER_DESC_queue): %u.",
					res);
				continue;
			}

			break;
		}

		if (DRV_CONFIG_enable_cp_mode(drv_cfg)) {
			// reset the current buffer index if in CP mode
			cur_buf = OUTPUT_current_buffer(outbuf);
			for (i = 0; i < OUTPUT_NUM_BUFFERS;
			     i++) { //iterate through all buffers
				cur_buf++;
				if (cur_buf >= OUTPUT_NUM_BUFFERS) {
					cur_buf = 0;
				} //circularly
				to_copy = OUTPUT_buffer_full(outbuf, cur_buf);
				if (to_copy != 0) {
					if (flush &&
					    DRV_CONFIG_enable_cp_mode(
						    drv_cfg) &&
					    cur_buf == OUTPUT_current_buffer(
							       outbuf)) {
						OUTPUT_current_buffer(outbuf)++;
						if (OUTPUT_current_buffer(
							    outbuf) >=
						    OUTPUT_NUM_BUFFERS) {
							OUTPUT_current_buffer(
								outbuf) = 0;
						}
						OUTPUT_remaining_buffer_size(
							outbuf) =
							OUTPUT_total_buffer_size(
								outbuf);
					}
					break;
				}
			}
		}
		SEP_DRV_LOG_TRACE("Get to copy %d.", (S32)cur_buf);
		to_copy = OUTPUT_buffer_full(outbuf, cur_buf);
		SEP_DRV_LOG_TRACE(
			"output_Read awakened, buffer %d has %d bytes.",
			cur_buf, (int)to_copy);
	}

	/* Ensure that the user's buffer is large enough */
	if (to_copy > count) {
		SEP_DRV_LOG_ERROR_TRACE_OUT(
			"OS_NO_MEM (user buffer is too small!).");
		return OS_NO_MEM;
	}

	/* Copy data to user space. Note that we use cur_buf as the source */
	if (GET_DRIVER_STATE() != DRV_STATE_TERMINATING) {
		uncopied = copy_to_user(buf, OUTPUT_buffer(outbuf, cur_buf),
					to_copy);
		/* Mark the buffer empty */
		OUTPUT_buffer_full(outbuf, cur_buf) = 0;
		*f_pos += to_copy - uncopied;
		if (uncopied) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Res: %u (only copied %u of %u bytes!).",
				(U32)(to_copy - uncopied), (U32)to_copy,
				(U32)uncopied);
			return (to_copy - uncopied);
		}
	} else {
		to_copy = 0;
		SEP_DRV_LOG_TRACE("To copy set to 0.");
	}

	// At end-of-file, decrement the count of active buffer writers

	if (to_copy == 0) {
		DRV_BOOL flush_val = atomic_dec_and_test(&flush_writers);
		SEP_DRV_LOG_TRACE("Decremented flush_writers.");
		if (flush_val == TRUE) {
			wake_up_interruptible_sync(&flush_queue);
		}
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)to_copy);
	return to_copy;
}

/* ------------------------------------------------------------------------- */
/*!
 *  @fn  ssize_t  OUTPUT_Module_Read(struct file  *filp,
 *                                   char         *buf,
 *                                   size_t        count,
 *                                   loff_t       *f_pos)
 *
 *  @brief  Return a module buffer to user-mode. If not full or flush, wait
 *
 *  @param *filp   a file pointer
 *  @param *buf    a sampling buffer
 *  @param  count  size of the user's buffer
 *  @param  f_pos  file pointer (current offset in bytes)
 *  @param  buf    the kernel output buffer structure
 *
 *  @return number of bytes read. zero indicates end of file. Neg means error
 *
 *  Place no more than count bytes into the user's buffer.
 *  Block on "BUFFER_DESC_queue(kernel_buf)" if buffer isn't full.
 *
 * <I>Special Notes:</I>
 *
 */
ssize_t OUTPUT_Module_Read(struct file *filp, char __user *buf, size_t count,
				  loff_t *f_pos)
{
	ssize_t res;

	SEP_DRV_LOG_TRACE_IN("");
	SEP_DRV_LOG_TRACE("Read request for modules on minor.");

	res = output_Read(filp, buf, count, f_pos, module_buf);

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)res);
	return res;
}

/* ------------------------------------------------------------------------- */
/*!
 *  @fn  ssize_t  OUTPUT_Sample_Read(struct file  *filp,
 *                                   char         *buf,
 *                                   size_t        count,
 *                                   loff_t       *f_pos)
 *
 *  @brief  Return a sample buffer to user-mode. If not full or flush, wait
 *
 *  @param *filp   a file pointer
 *  @param *buf    a sampling buffer
 *  @param  count  size of the user's buffer
 *  @param  f_pos  file pointer (current offset in bytes)
 *  @param  buf    the kernel output buffer structure
 *
 *  @return number of bytes read. zero indicates end of file. Neg means error
 *
 *  Place no more than count bytes into the user's buffer.
 *  Block on "BUFFER_DESC_queue(kernel_buf)" if buffer isn't full.
 *
 * <I>Special Notes:</I>
 *
 */
ssize_t OUTPUT_Sample_Read(struct file *filp, char __user *buf, size_t count,
				  loff_t *f_pos)
{
	int i;
	ssize_t res;

	SEP_DRV_LOG_TRACE_IN("");

	i = iminor(filp->DRV_F_DENTRY
			   ->d_inode); // kernel pointer - not user pointer
	SEP_DRV_LOG_TRACE("Read request for samples on minor %d.", i);
	res = output_Read(filp, buf, count, f_pos, &(cpu_buf[i]));

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)res);
	return res;
}

/* ------------------------------------------------------------------------- */
/*!
 *  @fn  ssize_t  OUTPUT_Sample_Read(struct file  *filp,
 *                                   char         *buf,
 *                                   size_t        count,
 *                                   loff_t       *f_pos)
 *
 *  @brief  Return a sample buffer to user-mode. If not full or flush, wait
 *
 *  @param *filp   a file pointer
 *  @param *buf    a sampling buffer
 *  @param  count  size of the user's buffer
 *  @param  f_pos  file pointer (current offset in bytes)
 *  @param  buf    the kernel output buffer structure
 *
 *  @return number of bytes read. zero indicates end of file. Neg means error
 *
 *  Place no more than count bytes into the user's buffer.
 *  Block on "BUFFER_DESC_queue(kernel_buf)" if buffer isn't full.
 *
 * <I>Special Notes:</I>
 *
 */
ssize_t OUTPUT_UncSample_Read(struct file *filp, char __user *buf,
			size_t count, loff_t *f_pos)
{
	int i;
	ssize_t res = 0;

	SEP_DRV_LOG_TRACE_IN("");

	i = iminor(filp->DRV_F_DENTRY
			   ->d_inode); // kernel pointer - not user pointer
	SEP_DRV_LOG_TRACE("Read request for samples on minor %d.", i);
	if (unc_buf_init) {
		res = output_Read(filp, buf, count, f_pos, &(unc_buf[i]));
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)res);
	return res;
}

/* ------------------------------------------------------------------------- */
/*!
 *  @fn  ssize_t  OUTPUT_SidebandInfo_Read(struct file  *filp,
 *                                         char         *buf,
 *                                         size_t        count,
 *                                         loff_t       *f_pos)
 *
 *  @brief  Return a sideband info buffer to user-mode. If not full or flush, wait
 *
 *  @param *filp   a file pointer
 *  @param *buf    a sideband info buffer
 *  @param  count  size of the user's buffer
 *  @param  f_pos  file pointer (current offset in bytes)
 *  @param  buf    the kernel output buffer structure
 *
 *  @return number of bytes read. zero indicates end of file. Neg means error
 *
 *  Place no more than count bytes into the user's buffer.
 *  Block on "BUFFER_DESC_queue(kernel_buf)" if buffer isn't full.
 *
 * <I>Special Notes:</I>
 *
 */
ssize_t OUTPUT_SidebandInfo_Read(struct file *filp, char __user *buf,
					size_t count, loff_t *f_pos)
{
	int i;
	ssize_t res = 0;

	SEP_DRV_LOG_TRACE_IN("");

	i = iminor(filp->DRV_F_DENTRY
			   ->d_inode); // kernel pointer - not user pointer
	SEP_DRV_LOG_TRACE("Read request for pebs process info on minor %d.", i);
	if (multi_pebs_enabled || sched_switch_enabled) {
		res = output_Read(filp, buf, count, f_pos,
				  &(cpu_sideband_buf[i]));
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)res);
	return res;
}

/*
 *  @fn output_Initialized_Buffers()
 *
 *  @result OUTPUT
 *  @param  BUFFER_DESC desc   - descriptor for the buffer being initialized
 *  @param  U32         factor - multiplier for OUTPUT_BUFFER_SIZE.
 *                               1 for cpu buffers, 2 for module buffers.
 *
 *  @brief  Allocate, initialize, and return an output data structure
 *
 * <I>Special Notes:</I>
 *     Multiple (OUTPUT_NUM_BUFFERS) buffers will be allocated
 *     Each buffer is of size (OUTPUT_BUFFER_SIZE)
 *     Each field in the buffer is initialized
 *     The event queue for the OUTPUT is initialized
 *
 */
static BUFFER_DESC output_Initialized_Buffers(BUFFER_DESC desc, U32 factor)
{
	OUTPUT outbuf;
	int j;

	SEP_DRV_LOG_TRACE_IN("Desc: %p, factor: %u.", desc, factor);

	/*
 *  Allocate the BUFFER_DESC, then allocate its buffers
 */
	if (desc == NULL) {
		desc = (BUFFER_DESC)CONTROL_Allocate_Memory(
			sizeof(BUFFER_DESC_NODE));
		if (desc == NULL) {
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Res: NULL (failed allocation for desc!).");
			return NULL;
		}
	}
	outbuf = &(BUFFER_DESC_outbuf(desc));
	spin_lock_init(&OUTPUT_buffer_lock(outbuf));
	for (j = 0; j < OUTPUT_NUM_BUFFERS; j++) {
		if (OUTPUT_buffer(outbuf, j) == NULL) {
			OUTPUT_buffer(outbuf, j) = CONTROL_Allocate_Memory(
				(size_t)OUTPUT_BUFFER_SIZE * factor);
		}
		OUTPUT_buffer_full(outbuf, j) = 0;
		if (!OUTPUT_buffer(outbuf, j)) {
			/*return NULL to tell the caller that allocation failed*/
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Res: NULL (failed alloc for OUTPUT_buffer(output, %d)!).",
				j);
			CONTROL_Free_Memory(desc);
			return NULL;
		}
	}
	/*
	 *  Initialize the remaining fields in the BUFFER_DESC
	 */
	OUTPUT_current_buffer(outbuf) = 0;
	OUTPUT_signal_full(outbuf) = FALSE;
	OUTPUT_remaining_buffer_size(outbuf) = OUTPUT_BUFFER_SIZE * factor;
	OUTPUT_total_buffer_size(outbuf) = OUTPUT_BUFFER_SIZE * factor;
	OUTPUT_tasklet_queued(outbuf) = FALSE;
	init_waitqueue_head(&BUFFER_DESC_queue(desc));

	SEP_DRV_LOG_TRACE_OUT("Res: %p.", desc);
	return desc;
}

/* ------------------------------------------------------------------------- */
/*!
 * @fn          VOID output_NMI_Sample_Buffer (
 *                  )
 *
 * @brief       Callback from NMI tasklet. The function checks if any buffers
 *              are full, and if full, signals the reader threads.
 *
 * @param       none
 *
 * @return      NONE
 *
 * <I>Special Notes:</I>
 *              This callback was added to handle out-of-band event delivery
 *              when running in NMI mode
 */
static void output_NMI_Sample_Buffer(unsigned long data)
{
	U32 cpu_id;
	OUTPUT outbuf;

	SEP_DRV_LOG_NOTIFICATION_IN("Data: %u.", (U32)data);

	if (data == (unsigned long)-1) {
		cpu_id = CONTROL_THIS_CPU();
	} else {
		cpu_id = data;
	}

	if (cpu_buf) {
		outbuf = &BUFFER_DESC_outbuf(&cpu_buf[cpu_id]);
		if (outbuf && OUTPUT_signal_full(outbuf)) {
			wake_up_interruptible_sync(
				&BUFFER_DESC_queue(&cpu_buf[cpu_id]));
			OUTPUT_signal_full(outbuf) = FALSE;
			OUTPUT_tasklet_queued(outbuf) = FALSE;
		}
	}

	if (cpu_sideband_buf) {
		outbuf = &BUFFER_DESC_outbuf(&cpu_sideband_buf[cpu_id]);
		if (outbuf && OUTPUT_signal_full(outbuf)) {
			wake_up_interruptible_sync(
				&BUFFER_DESC_queue(&cpu_sideband_buf[cpu_id]));
			OUTPUT_signal_full(outbuf) = FALSE;
			OUTPUT_tasklet_queued(outbuf) = FALSE;
		}
	}

	SEP_DRV_LOG_NOTIFICATION_OUT("");
}

/*
 *  @fn extern void OUTPUT_Initialize(void)
 *
 *  @returns OS_STATUS
 *  @brief  Allocate, initialize, and return all output data structure
 *
 * <I>Special Notes:</I>
 *      Initialize the output structures.
 *      For each CPU in the system, allocate the output buffers.
 *      Initialize a module buffer and temp file to hold module information
 *      Initialize the read queues for each sample buffer
 *
 */
OS_STATUS OUTPUT_Initialize(void)
{
	BUFFER_DESC unused;
	S32 i;
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_TRACE_IN("");

	flush = 0;
	if (saved_buffer_size != OUTPUT_BUFFER_SIZE) {
		if (saved_buffer_size > 0) {
			OUTPUT_Destroy();
		}
		saved_buffer_size = OUTPUT_BUFFER_SIZE;
	}
	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		unused = output_Initialized_Buffers(&cpu_buf[i], 1);
		if (!unused) {
			OUTPUT_Destroy();
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"OS_NO_MEM (failed to allocate cpu output buffers!).");
			return OS_NO_MEM;
		}
	}

	if (multi_pebs_enabled || sched_switch_enabled) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			unused = output_Initialized_Buffers(
				&cpu_sideband_buf[i], 1);
			if (!unused) {
				OUTPUT_Destroy();
				SEP_DRV_LOG_ERROR_TRACE_OUT(
					"OS_NO_MEM (failed to allocate pebs process info output buffers!).");
				return OS_NO_MEM;
			}
		}
	}

	/*
	 *  Just need one module buffer
	 */
	unused = output_Initialized_Buffers(module_buf, MODULE_BUFF_SIZE);
	if (!unused) {
		OUTPUT_Destroy();
		SEP_DRV_LOG_ERROR_TRACE_OUT(
			"OS_NO_MEM (failed to create module output buffers!).");
		return OS_NO_MEM;
	}

	SEP_DRV_LOG_TRACE("Set up the tasklet for NMI.");
	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
#if !defined(DRV_SEP_ACRN_ON)
		tasklet_init(&CPU_STATE_nmi_tasklet(&pcb[i]),
			     output_NMI_Sample_Buffer, (unsigned long)-1);
#else
		tasklet_init(&CPU_STATE_nmi_tasklet(&pcb[i]),
			     output_NMI_Sample_Buffer, (unsigned long)i);
#endif
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)status);
	return status;
}

#if defined(DRV_USE_TASKLET_WORKAROUND)
static struct tasklet_struct dummy_tasklet;

/*
 *  @fn extern void output_tasklet_waker (PVOID ptr)
 *
 *  @returns None
 *  @brief   Schedules a dummy tasklet to wake up the tasklet handler on the current core
 *
 * <I>Special Notes:</I>
 *      Workaround for a rare situation where some tasklets are scheduled, but the core's TASKLET softirq bit was reset.
 *      [NB: this may be caused by a kernel bug; so far, this issue was only observed on kernel version 3.10.0-123.el7]
 *      Scheduling a (new) tasklet raises a new softirq, and gives 'forgotten' tasklets another chance to be processed.
 *      This workaround is not fool-proof: if this new tasklet gets 'forgotten' too, the driver will get stuck in the
 *      Clean Up routine until it gets processed (thanks to an external event raising the TASKLET softirq on this core),
 *      which might never happen.
 *
 */
static void output_tasklet_waker(PVOID ptr)
{
	SEP_DRV_LOG_TRACE_IN("");
	tasklet_schedule(&dummy_tasklet);
	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 *  @fn extern void output_dummy_tasklet_handler (unsigned long dummy)
 *
 *  @returns None
 *  @brief   Dummy tasklet handler.
 *
 * <I>Special Notes:</I>
 *      If this gets executed, the aforementioned workaround was successful.
 *
 */
static void output_dummy_tasklet_handler(unsigned long dummy)
{
	SEP_DRV_LOG_NOTIFICATION_IN("Workaround was successful!");
	SEP_DRV_LOG_NOTIFICATION_OUT("");
}
#endif

/*
 *  @fn extern void OUTPUT_Cleanup (void)
 *
 *  @returns None
 *  @brief   Cleans up NMI tasklets if needed
 *
 * <I>Special Notes:</I>
 *      Waits until all NMI tasklets are complete.
 *
 */
void OUTPUT_Cleanup(void)
{
	SEP_DRV_LOG_TRACE_IN("");

	if (!pcb) {
		SEP_DRV_LOG_TRACE_OUT("Early exit (!pcb).");
		return;
	} else {
		int i;
		SEP_DRV_LOG_TRACE("Killing all NMI tasklets...");

		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			SEP_DRV_LOG_TRACE("Killing NMI tasklet %d...", i);

			if (CPU_STATE_nmi_tasklet(&pcb[i]).state) {
#if defined(DRV_USE_TASKLET_WORKAROUND)
				SEP_DRV_LOG_ERROR(
					"Tasklet %d is probably stuck! Trying workaround...",
					i);
				tasklet_init(&dummy_tasklet,
					     output_dummy_tasklet_handler, 0);
				CONTROL_Invoke_Cpu(i, output_tasklet_waker,
						   NULL);
				tasklet_kill(&dummy_tasklet);
				SEP_DRV_LOG_ERROR(
					"Workaround was successful for tasklet %d.",
					i);
#else
				SEP_DRV_LOG_ERROR(
					"Tasklet %d may be stuck. Try to set USE_TASKLET_WORKAROUND=YES in the Makefile if you observe unexpected behavior (e.g. cannot terminate a collection or initiate a new one).",
					i);
#endif
			}

			tasklet_kill(&CPU_STATE_nmi_tasklet(&pcb[i]));
		}
	}

	SEP_DRV_LOG_TRACE_OUT("");
}

/*
 *  @fn extern void OUTPUT_Initialize_UNC()
 *
 *  @returns OS_STATUS
 *  @brief  Allocate, initialize, and return all output data structure
 *
 * <I>Special Notes:</I>
 *      Initialize the output structures.
 *      For each CPU in the system, allocate the output buffers.
 *      Initialize a module buffer and temp file to hold module information
 *      Initialize the read queues for each sample buffer
 *
 */
OS_STATUS OUTPUT_Initialize_UNC(void)
{
	BUFFER_DESC unused;
	int i;
	OS_STATUS status = OS_SUCCESS;

	SEP_DRV_LOG_TRACE_IN("");

	for (i = 0; i < num_packages; i++) {
		unused = output_Initialized_Buffers(&unc_buf[i], 1);
		if (!unused) {
			OUTPUT_Destroy();
			SEP_DRV_LOG_ERROR_TRACE_OUT(
				"Failed to allocate package output buffers!");
			return OS_NO_MEM;
		}
	}

	SEP_DRV_LOG_TRACE_OUT("Res: %u.", (U32)status);
	return status;
}

/*
 *  @fn OS_STATUS  OUTPUT_Flush()
 *
 *  @brief  Flush the module buffers and sample buffers
 *
 *  @return OS_STATUS
 *
 *  For each CPU in the system, set buffer full to the byte count to flush.
 *  Flush the modules buffer, as well.
 *
 */
int OUTPUT_Flush(void)
{
	int i;
	int writers = 0;
	OUTPUT outbuf;

	SEP_DRV_LOG_TRACE_IN("");

	/*
	 *  Flush all remaining data to files
	 *  set up a flush event
	 */
	init_waitqueue_head(&flush_queue);
	SEP_DRV_LOG_TRACE(
		"Waiting for %d writers.",
		(GLOBAL_STATE_num_cpus(driver_state) + OTHER_C_DEVICES));
	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		if (CPU_STATE_initial_mask(&pcb[i]) == 0) {
			continue;
		}
		outbuf = &(cpu_buf[i].outbuf);
		writers += 1;

		OUTPUT_buffer_full(outbuf, OUTPUT_current_buffer(outbuf)) =
			OUTPUT_total_buffer_size(outbuf) -
			OUTPUT_remaining_buffer_size(outbuf);
	}

	if (unc_buf_init) {
		for (i = 0; i < num_packages; i++) {
			outbuf = &(unc_buf[i].outbuf);
			writers += 1;

			OUTPUT_buffer_full(outbuf,
					   OUTPUT_current_buffer(outbuf)) =
				OUTPUT_total_buffer_size(outbuf) -
				OUTPUT_remaining_buffer_size(outbuf);
		}
	}

	if (multi_pebs_enabled || sched_switch_enabled) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			if (CPU_STATE_initial_mask(&pcb[i]) == 0) {
				continue;
			}
			outbuf = &(cpu_sideband_buf[i].outbuf);
			writers += 1;

			OUTPUT_buffer_full(outbuf,
					   OUTPUT_current_buffer(outbuf)) =
				OUTPUT_total_buffer_size(outbuf) -
				OUTPUT_remaining_buffer_size(outbuf);
		}
	}

	atomic_set(&flush_writers, writers + OTHER_C_DEVICES);
	// Flip the switch to terminate the output threads
	// Do not do this earlier, as threads may terminate before all the data is flushed
	flush = 1;
	for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
		if (CPU_STATE_initial_mask(&pcb[i]) == 0) {
			continue;
		}
		outbuf = &BUFFER_DESC_outbuf(&cpu_buf[i]);
		OUTPUT_buffer_full(outbuf, OUTPUT_current_buffer(outbuf)) =
			OUTPUT_total_buffer_size(outbuf) -
			OUTPUT_remaining_buffer_size(outbuf);
		wake_up_interruptible_sync(&BUFFER_DESC_queue(&cpu_buf[i]));
	}

	if (unc_buf_init) {
		for (i = 0; i < num_packages; i++) {
			outbuf = &BUFFER_DESC_outbuf(&unc_buf[i]);
			OUTPUT_buffer_full(outbuf,
					   OUTPUT_current_buffer(outbuf)) =
				OUTPUT_total_buffer_size(outbuf) -
				OUTPUT_remaining_buffer_size(outbuf);
			wake_up_interruptible_sync(
				&BUFFER_DESC_queue(&unc_buf[i]));
		}
	}

	if (multi_pebs_enabled || sched_switch_enabled) {
		for (i = 0; i < GLOBAL_STATE_num_cpus(driver_state); i++) {
			if (CPU_STATE_initial_mask(&pcb[i]) == 0) {
				continue;
			}
			outbuf = &BUFFER_DESC_outbuf(&cpu_sideband_buf[i]);
			OUTPUT_buffer_full(outbuf,
					   OUTPUT_current_buffer(outbuf)) =
				OUTPUT_total_buffer_size(outbuf) -
				OUTPUT_remaining_buffer_size(outbuf);
			wake_up_interruptible_sync(
				&BUFFER_DESC_queue(&cpu_sideband_buf[i]));
		}
	}
	// Flush all data from the module buffers

	outbuf = &BUFFER_DESC_outbuf(module_buf);

	OUTPUT_buffer_full(outbuf, OUTPUT_current_buffer(outbuf)) =
		OUTPUT_total_buffer_size(outbuf) -
		OUTPUT_remaining_buffer_size(outbuf);

	SEP_DRV_LOG_TRACE("Waking up module_queue.");
	wake_up_interruptible_sync(&BUFFER_DESC_queue(module_buf));

	//Wait for buffers to empty
	while (atomic_read(&flush_writers) != 0) {
		unsigned long delay;
		U32 res;
		delay = msecs_to_jiffies(1000);
		res = wait_event_interruptible_timeout(
			flush_queue, atomic_read(&flush_writers) == 0, delay);

		if (res == ERESTARTSYS || res == 0) {
			SEP_DRV_LOG_TRACE(
				"Wait_event_interruptible_timeout(flush_queue): %u, %u writers.",
				res, atomic_read(&flush_writers));
			continue;
		}
	}

	SEP_DRV_LOG_TRACE("Awakened from flush_queue.");
	flush = 0;

	SEP_DRV_LOG_TRACE_OUT("Res: 0.");
	return 0;
}

/*
 *  @fn extern void OUTPUT_Destroy()
 *
 *  @param   buffer  -  seed name of the output file
 *  @param   len     -  length of the seed name
 *  @returns OS_STATUS
 *  @brief   Deallocate output structures
 *
 * <I>Special Notes:</I>
 *      Free the module buffers
 *      For each CPU in the system, free the sampling buffers
 */
int OUTPUT_Destroy(void)
{
	int i, n;
	OUTPUT outbuf;

	SEP_DRV_LOG_TRACE_IN("");

	if (module_buf) {
		outbuf = &BUFFER_DESC_outbuf(module_buf);
		output_Free_Buffers(module_buf,
				    OUTPUT_total_buffer_size(outbuf));
	}

	if (cpu_buf != NULL) {
		n = GLOBAL_STATE_num_cpus(driver_state);
		for (i = 0; i < n; i++) {
			outbuf = &BUFFER_DESC_outbuf(&cpu_buf[i]);
			output_Free_Buffers(&cpu_buf[i],
					    OUTPUT_total_buffer_size(outbuf));
		}
	}

	if (unc_buf != NULL) {
		n = num_packages;
		for (i = 0; i < n; i++) {
			outbuf = &BUFFER_DESC_outbuf(&unc_buf[i]);
			output_Free_Buffers(&unc_buf[i],
					    OUTPUT_total_buffer_size(outbuf));
		}
	}

	if (cpu_sideband_buf != NULL) {
		n = GLOBAL_STATE_num_cpus(driver_state);
		for (i = 0; i < n; i++) {
			outbuf = &BUFFER_DESC_outbuf(&cpu_sideband_buf[i]);
			output_Free_Buffers(&cpu_sideband_buf[i],
					    OUTPUT_total_buffer_size(outbuf));
		}
	}

	SEP_DRV_LOG_TRACE_OUT("Res: 0.");
	return 0;
}
