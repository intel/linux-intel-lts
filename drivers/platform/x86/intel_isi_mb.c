// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Safety Island(ISI) Mailbox communication
 * driver
 *
 * Copyright (c) 2019, Intel Corp.
 */
#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/semaphore.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "intel_isi_mb.h"

#define ISI_NR_DEVICES 1U

/* TX Length Register*/
#define TX_LEN_REG 0x28U

/* DoorBell OUT Register */
#define DB_OUT_REG 0x2cU
#define DB_OUT_BIT 0x0U

#define DB_IN_REG 0x14U
#define DB_IN_BIT 0x0U

/* Interrupt Status Register */
/* Interrupt Status Register */
#define INTR_STATUS_REG 0x34U
#define DB_IN_CLR_INTR_STS_BIT 0x0U
#define DB_OUT_SET_INTR_STS_BIT 0x1U

/* Interrupt Mask Register */
#define INTR_MSK_REG 0x3cU
#define DB_IN_CLR_INTR_MSK_BIT 0x0U
#define DB_OUT_SET_INTR_MSK_BIT 0x1U

/* TLP Count Register */
#define TLP_CNT_REG 0x40U
/* RX Length Register */
#define RX_LEN_REG 0x44U

#define PCIE_MB_RX_RAM_OFFSET 0x400U
#define PCIE_MB_TX_RAM_OFFSET 0x800U

#define MAX_UINT_VALUE 0xFFFFFF00U
/*
 * Defined own cricular buffer as the
 * one present in the linux/circ_buf.h has <char *> as the buf pointer
 * and that would have led to lot of typecasting in the code as we would
 * always be reading/writing uint32_t data from this circular buffer
 */
struct isi_circ_buf {
	uint32_t *buf;
	int head;
	int tail;
};

struct isi_driver_ctx {
	struct hlist_head sync_cmd_waitlist;
	struct hlist_head async_cmd_waitlist;
	/* Lock guarding the list of various procs waiting for a resp from ISI. */
	spinlock_t proc_waitlist_lock;
	struct completion tx_ack;
	struct isi_circ_buf ring_buf;
	struct hlist_head wl_list;
	/* Lock to serialize access to wl_list*/
	struct mutex wl_list_lock;
	/* Serializes circular buffer access from IRQ handler and from irq_thread. */
	spinlock_t ring_buf_lock;
	/* Lock to serialize access for Mailbox send path. */
	struct mutex tx_lock;
	struct pci_dev *pdev;
	void __iomem *mmio;
	struct device *dev;
	struct cdev c_dev;
	int active_wl_cnt;
};

struct isi_wl_ctx {
	unsigned int async_cmd_req_bitmap;
	struct isi_driver_ctx *isi_ctx;
	struct hlist_node wl_ctx_node;
	struct hlist_head wl_proc_list;
	/* Serialize access to wl_proc_list */
	struct mutex wl_ctx_lock;
	uint8_t wl_id;
	pid_t tgid;
};

struct isi_wl_proc {
	unsigned int async_cmd_req_bitmap;
	struct hlist_node wl_proc_node;
	struct isi_circ_buf async_cmd_ring_buf;
	struct isi_wl_ctx *p_wl_ctx;
	struct completion async_completion;
	struct completion sync_completion;
	struct hlist_node waiting_wl_proc_node_sync;
	struct hlist_node waiting_wl_proc_node_async;
	uint16_t seq_num;
	int rx_msg_cnt;
	void *tx_buf;
	void *rx_buf;
	int tx_size;
	bool async_queued;
	pid_t pid;
};

static dev_t isi_devt;
static struct class *isi_dev_cl;

/*
 * Function    :
 *           memcpy_s
 * Parameters  :
 *           void *destination  Pointer to destination buffer
 *           size_t dest_size   Size of destination buffer
 *           const void *source Constant pointer to source buffer
 *           size_t count       Size in bytes to be copied into the destination
 *                    buffer.
 * Return type : 0 - On Success -EINVAL - on Failure Description
 * This function is local implementation of a library function memcpy_s to fix
 * Klockwork issue.
 */
uint32_t memcpy_s(void *destination, size_t dest_size, void *source,
		  size_t count)
{
	uint32_t status;

	char *src = (char *)source;
	char *dest = (char *)destination;

	/* Input check. */
	if ((NULL != src) && (NULL != dest) && (dest_size > 0) && (count > 0) &&
	    (dest_size >= count) && (dest != src)) {
		/* Overlap condition. */
		if (((dest > src) && (dest < (src + count))) ||
		    ((src > dest) && (src < (dest + dest_size)))) {
			status = -EINVAL;
		} else {
			memcpy(destination, source, count);
			status = 0;
		}
	} else {
		status = -EINVAL;
	}

	return status;
}

static int isi_cmd_to_req_type(uint16_t command)
{
	int ret;

	switch (command) {
	case REBOOT_REQ_CMD:
		ret = REQ_HOST_REBOOT;
		break;
	case TIMER_EXPIRY_CMD:
		ret = REQ_TIMER_EXPIRY_NOTIFY;
		break;
	case NOK_WARN_NOTIFY_CMD:
		ret = REQ_NOK_WARNING_NOTIFY;
		break;
	case DIAG_DATA_CMD:
		ret = REQ_DIAG_DATA_TO_HOST;
		break;
	case START_DTI_CMD:
		ret = REQ_START_DTI;
		break;
	case IEH_ERR_GET_CMD:
		ret = REQ_IEH_ERROR_GET;
		break;
	case ODCC_SNAPSHOT_RESP:
		ret = REQ_ODCC_SNAPSHOT;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void send_data_wakeup_wl_proc(struct isi_wl_proc *wl_proc, void *buf,
				     uint32_t len, bool sync_cmd)
{
	struct isi_circ_buf *cb;
	int head = 0, tail = 0, cnt = 0;
	uint32_t buf_len;
	uint32_t *in_buf = (uint32_t *)buf;

	if (sync_cmd == true) {
		memcpy_s(wl_proc->rx_buf, (len * 4), (void *)buf, (len * 4));
		wl_proc->rx_msg_cnt = len * 4;
		hlist_del_init(&wl_proc->waiting_wl_proc_node_sync);
		complete(&wl_proc->sync_completion);
	} else {
		/* Add data to the async data ring buf of the
		   wl_procs that have registered for the async command
		 */
		cb = &wl_proc->async_cmd_ring_buf;
		head = cb->head;
		tail = READ_ONCE(cb->tail);
		buf_len = CIRC_SPACE(head, tail, (SZ_256));
		if (!buf_len || (buf_len < len)) {
			return;
		}

		while (len > 0) {
			cb->buf[head] = in_buf[cnt];
			head = (head + 1) & (SZ_256 - 1);
			len--;
			cnt++;
		}
		smp_store_release((&cb->head), head);
		complete(&wl_proc->async_completion);
	}
}

static uint32_t isi_rx_buf[SZ_256];

/**
 * isi_irq_thread_fn	Bottom half processing of the
 *						Mailbox IRQ.
 *
 * @param				ptr to isi_driver_ctx
 *
 * The IRQ handler, on receiving the Mailbox DB_Out
 * interrupt, reads the RX_RAM and puts the received
 * data in the circular buffer.
 * This irq_thread, reads the circular buffer contents,
 * finds out the wl_proc waiting in sync_cmd_waitlist
 * and puts the data in the waiting wl_proc->rx_buf.
 * There could be three types of received data:
 * 1. Async command
 * 2. Response to workload register cmd
 * 3. Sync cmd response
 *
 */
static irqreturn_t isi_irq_thread_fn(int irq, void *dev_id)
{
	uint32_t cnt = 0;
	uint32_t wl_id, len, seq_num, command;
	int bitmask, req_type, head, tail;
	unsigned long flags;
	struct isi_generic_packet_header *packet_header;
	struct isi_wl_proc *wl_proc;
	struct isi_driver_ctx *pctx = (struct isi_driver_ctx *)dev_id;
	struct isi_circ_buf *cb = &pctx->ring_buf;
	uint32_t *rx_buf = isi_rx_buf;
	struct hlist_node *h_node, *prev_node;

	memset((void *)rx_buf, 0, sizeof(uint32_t) * SZ_256);
	spin_lock_irqsave(&pctx->ring_buf_lock, flags);
	/* safely read the head */
	head = smp_load_acquire(&cb->head);
	tail = cb->tail;

	if (CIRC_CNT(head, tail, SZ_256) >= 1) {
		while (CIRC_CNT(head, tail, SZ_256)) {
			rx_buf[cnt] = cb->buf[tail];
			cnt++;
			tail = (tail + 1) & (SZ_256 - 1);
		}
		/* Safely update the tail */
		smp_store_release(&cb->tail, tail);
		spin_unlock_irqrestore(&pctx->ring_buf_lock, flags);
	} else {
		spin_unlock_irqrestore(&pctx->ring_buf_lock, flags);
		dev_err(pctx->dev, "[ISI_IRQ_THRD] no data in circ_buff\n");
		return IRQ_HANDLED;
	}
	do {
		packet_header = (struct isi_generic_packet_header *)rx_buf;
		wl_id = packet_header->wl_address;
		seq_num = packet_header->sequence_number;
		command = packet_header->command;
		len = packet_header->length;
		req_type = isi_cmd_to_req_type(command);

		if (len == 0)
			break;

		spin_lock_irqsave(&pctx->proc_waitlist_lock, flags);
		if (req_type >= 0) { /* Async command */
			if (!hlist_empty(&pctx->async_cmd_waitlist)) {
				hlist_for_each_safe (
					h_node, prev_node,
					&pctx->async_cmd_waitlist) {
					wl_proc = hlist_entry(
						h_node, typeof(*wl_proc),
						waiting_wl_proc_node_async);
					bitmask = (1 << req_type);
					if (wl_proc->async_cmd_req_bitmap &
					    bitmask) {
						send_data_wakeup_wl_proc(
							wl_proc, rx_buf, len,
							false);
					}
				}
			}
		} else {
			if (!hlist_empty(&pctx->sync_cmd_waitlist)) {
				hlist_for_each_entry (
					wl_proc, &pctx->sync_cmd_waitlist,
					waiting_wl_proc_node_sync) {
					if (command == WL_REG_CMD) {
						/* Find waiting proc with seq_num
						 * and wl_id = broadcast_id
						 */
						if ((wl_proc->seq_num ==
						     seq_num) &&
						    (wl_proc->p_wl_ctx->wl_id ==
						     BROADCAST_ID)) {
							send_data_wakeup_wl_proc(
								wl_proc, rx_buf,
								len, true);
							break;
						}
					} else {
						/* go through wl_proc list */
						/* find wl_proc with matching seq_num & wl_id */
						if ((wl_proc->seq_num ==
						     seq_num) &&
						    (wl_proc->p_wl_ctx->wl_id ==
						     wl_id)) {
							send_data_wakeup_wl_proc(
								wl_proc, rx_buf,
								len, true);
							break;
						}
					}
				}
			}
		}
		spin_unlock_irqrestore(&pctx->proc_waitlist_lock, flags);
		rx_buf += len;
	} while (1);
	return IRQ_HANDLED;
}

static void read_rx_data(struct isi_driver_ctx *pctx)
{
	struct isi_circ_buf *cb = &pctx->ring_buf;
	unsigned int recv_data_len = 0, cnt = 0;
	int head, tail, buf_len;

	spin_lock(&pctx->ring_buf_lock);
	head = cb->head;
	tail = READ_ONCE(cb->tail);
	buf_len = CIRC_SPACE(head, tail, (SZ_256));
	if (!buf_len)
		goto err_read_rx_data;

	recv_data_len = ioread32(pctx->mmio + PCIE_MB_TX_RAM_OFFSET);
	recv_data_len = (0x00FF0000U & recv_data_len) >> 16U;
	if (!recv_data_len) {
		dev_err(pctx->dev, "[ISI]Recv RX interrupt but no RX data\n");
		goto err_read_rx_data;
	}
	if (buf_len < recv_data_len) {
		dev_err(pctx->dev, "[ISI]Insufficient space for recv_data\n");
		goto err_read_rx_data;
	}

	while (recv_data_len > 0) {
		cb->buf[head] = ioread32(pctx->mmio + PCIE_MB_TX_RAM_OFFSET +
					 (cnt * 4));
		head = (head + 1) & (SZ_256 - 1);
		recv_data_len--;
		cnt++;
	}
	/* safely update the head */
	smp_store_release((&cb->head), head);
err_read_rx_data:
	spin_unlock(&pctx->ring_buf_lock);
}

/**
 * isi_pci_dev_irq_handler - IRQ handler for MB interrupts
 * @irq:	IRQ number
 * @ctx:   pointer to the isi_driver_ctx structure
 *
 * Based in the interrupt reason, handler does the following:
 * 1. DB_Out_Set	ISI has sent some data in doorbell
 *					Read data in Circular buffer and invoke
 *					irq_thread to do bottom half processing
 * 2. DB_In_Clear	Wake up the process that has sent the Txn
 *					signaling that ISI FW has received the
 *					Txn. This acts as an ack of sent txn
 */
irqreturn_t isi_pci_dev_irq_handler(int irq, void *ctx)
{
	irqreturn_t ret = IRQ_NONE;
	struct isi_driver_ctx *pctx = (struct isi_driver_ctx *)ctx;
	uint32_t intr_status;
	uint32_t clr_status;
	uint32_t intr_mask = { 0 };

	/* Mask the interrupts */
	intr_mask = BIT(DB_IN_CLR_INTR_MSK_BIT) | BIT(DB_OUT_SET_INTR_MSK_BIT);
	iowrite32(intr_mask, pctx->mmio + INTR_MSK_REG);
	/* Read the interrupt status register */
	intr_status = ioread32(pctx->mmio + INTR_STATUS_REG);
	/* Clear the interrupt mask */
	intr_mask = 0;
	iowrite32(intr_mask, pctx->mmio + INTR_MSK_REG);

	if (intr_status & BIT(DB_IN_CLR_INTR_STS_BIT)) {
		/* wake up wl_proc waiting for Ack */
		if (!completion_done(&pctx->tx_ack)) {
			complete(&pctx->tx_ack);
		}
		clr_status = 0;
		clr_status = BIT(DB_IN_CLR_INTR_STS_BIT);
		iowrite32(clr_status, pctx->mmio + INTR_STATUS_REG);
		ret = IRQ_HANDLED;
	}
	if (intr_status & BIT(DB_OUT_SET_INTR_STS_BIT)) {
		/* read data and put in ring buffer */
		read_rx_data(pctx);
		clr_status = 0;
		clr_status = BIT(DB_OUT_SET_INTR_STS_BIT);
		iowrite32(clr_status, pctx->mmio + INTR_STATUS_REG);
		iowrite32(0, pctx->mmio + DB_OUT_REG);
		ret = IRQ_WAKE_THREAD;
	}
	return ret;
}

static void isi_hw_send(struct isi_driver_ctx *pctx, void *tx_buf, int tx_size)
{
	int len = tx_size;
	uint32_t doorbell_in, tlp_cnt;
	mutex_lock(&pctx->tx_lock);
	memcpy_toio(pctx->mmio + PCIE_MB_RX_RAM_OFFSET, tx_buf, tx_size * 4);
	doorbell_in = (1 << DB_IN_BIT);
	iowrite32(doorbell_in, pctx->mmio + DB_IN_REG);

	/*wait for irq_handler to signal tx ack*/
	wait_for_completion(&pctx->tx_ack);
	mutex_unlock(&pctx->tx_lock);
}

inline void proc_waitlist_add_safe(struct isi_driver_ctx *pctx,
				   struct isi_wl_proc *wl_proc, bool is_async)
{
	unsigned long flags;

	spin_lock_irqsave(&pctx->proc_waitlist_lock, flags);
	if (is_async) {
		if (wl_proc->async_queued == false) {
			hlist_add_head(&wl_proc->waiting_wl_proc_node_async,
				       &pctx->async_cmd_waitlist);
			wl_proc->async_queued = true;
		}
	} else
		hlist_add_head(&wl_proc->waiting_wl_proc_node_sync,
			       &pctx->sync_cmd_waitlist);
	spin_unlock_irqrestore(&pctx->proc_waitlist_lock, flags);
}

inline void proc_waitlist_del_safe(struct isi_driver_ctx *pctx,
				   struct isi_wl_proc *wl_proc)
{
	unsigned long flags;

	spin_lock_irqsave(&pctx->proc_waitlist_lock, flags);
	hlist_del_init(&wl_proc->waiting_wl_proc_node_sync);
	spin_unlock_irqrestore(&pctx->proc_waitlist_lock, flags);
}

/**
 * workload_reg_dereg - Register/Deregister a workload
 *
 * @cmd			REGISTER/DERGISTER
 * @wl_proc		wl_proc associated with the thread that made the call
 * @wl_details	workload details as received from User application
 *
 * Each Safety application that wants to register itself with ISI
 * calls this API so that ISI FW can assign it a workload ID.
 * This workload_ID uniquely identifies a Safety workload.
 * Upon completion of safety workload, the User application
 * should call this ioctl with DEREGISTER command.
 */
static int workload_reg_dereg(unsigned int cmd, struct isi_wl_proc *wl_proc,
			      struct isi_workload_details *wl_details)
{
	int ret = 0;
	struct isi_driver_ctx *pctx;
	struct isi_generic_packet_header *header;
	uint8_t *dest_ptr;
	uint32_t rem_size;

	if ((!wl_proc) || (!wl_details))
		return -EINVAL;
	if ((!wl_proc->tx_buf) || (!wl_proc->rx_buf))
		return -ENOMEM;

	memcpy_s((void *)wl_proc->tx_buf,
		 sizeof(struct isi_generic_packet_header),
		 (void *)&wl_details->req_header,
		 sizeof(struct isi_generic_packet_header));
	wl_proc->tx_size = sizeof(struct isi_generic_packet_header) / 4;
	wl_proc->seq_num = wl_details->req_header.sequence_number;
	pctx = wl_proc->p_wl_ctx->isi_ctx;

	proc_waitlist_add_safe(pctx, wl_proc, false);
	isi_hw_send(pctx, wl_proc->tx_buf, wl_proc->tx_size);
	ret = wait_for_completion_interruptible(&wl_proc->sync_completion);

	if (ret != 0) {
		proc_waitlist_del_safe(pctx, wl_proc);
		return -EINTR;
	}

	if (wl_proc->rx_msg_cnt > 0) {
		header = (struct isi_generic_packet_header *)(wl_proc->rx_buf);
		if (cmd == ISI_DRIVER_WORKLOAD_REGISTER) {
			wl_proc->p_wl_ctx->wl_id = header->wl_address;
			wl_details->workload_id = header->wl_address;
		} else {
			wl_proc->p_wl_ctx->wl_id = 0xFFU;
		}
		memcpy_s((void *)&wl_details->resp_header,
			 sizeof(struct isi_generic_packet_header),
			 wl_proc->rx_buf,
			 sizeof(struct isi_generic_packet_header));
		dest_ptr = (uint8_t *)(wl_proc->rx_buf) +
			   sizeof(struct isi_generic_packet_header);
		rem_size = wl_proc->rx_msg_cnt -
			   sizeof(struct isi_generic_packet_header);
		if (copy_to_user(wl_details->resp_buffer, dest_ptr, rem_size)) {
			return -EINVAL;
		}
		wl_details->resp_buffer_size = (rem_size) / sizeof(uint32_t);
	}
	return ret;
}

static void cleanup_wl_proc_bufs(struct isi_wl_proc *wl_proc)
{
	memset(wl_proc->tx_buf, 0, (2 * SZ_1K));
	memset(wl_proc->rx_buf, 0, (2 * SZ_1K));
	wl_proc->tx_size = 0;
	wl_proc->rx_msg_cnt = 0;
}

/**
 * create_wl_proc - allocate and init a wl_proc instance
 * @p_wl_ctx:   pointer to the workload context that has spawned the current
 * thread.
 *
 * Each workloakd(process) can spawn a number of threads that can interact
 * with ISI, but they are all associated with the same Workload and share
 * the workload id and its context.
 *
 * called with assocated wl_ctx->wl_ctx_lock held.
 */
static struct isi_wl_proc *create_wl_proc(struct isi_wl_ctx *p_wl_ctx)
{
	struct isi_wl_proc *p_proc;

	p_proc = kzalloc(sizeof(*p_proc), GFP_KERNEL);
	if (!p_proc)
		return NULL;
	p_proc->pid = current->pid;
	p_proc->seq_num = 0;
	INIT_HLIST_NODE(&p_proc->wl_proc_node);
	INIT_HLIST_NODE(&p_proc->waiting_wl_proc_node_sync);
	INIT_HLIST_NODE(&p_proc->waiting_wl_proc_node_async);
	p_proc->p_wl_ctx = p_wl_ctx;
	init_completion(&p_proc->async_completion);
	init_completion(&p_proc->sync_completion);

	p_proc->rx_buf = kzalloc(2 * SZ_1K, GFP_KERNEL);
	if (!p_proc->rx_buf) {
		kfree(p_proc);
		return NULL;
	}
	p_proc->tx_buf = kzalloc(2 * SZ_1K, GFP_KERNEL);
	if (!p_proc->tx_buf) {
		kfree(p_proc->rx_buf);
		kfree(p_proc);
		return NULL;
	}

	p_proc->async_cmd_ring_buf.buf =
		kmalloc_array(SZ_256, SZ_4, GFP_KERNEL);
	if (!p_proc->async_cmd_ring_buf.buf) {
		kfree(p_proc->tx_buf);
		kfree(p_proc->rx_buf);
		kfree(p_proc);
		return NULL;
	}

	p_proc->async_queued = false;
	p_proc->tx_size = 0;
	p_proc->rx_msg_cnt = 0;
	p_proc->async_cmd_ring_buf.head = 0;
	p_proc->async_cmd_ring_buf.tail = 0;
	hlist_add_head(&p_proc->wl_proc_node, &p_wl_ctx->wl_proc_list);
	return p_proc;
}

/**
 * get_wl_proc - Get the wl_proc associated with the calling thread
 * @wl_ctx:   pointer to the wl_ctx structure
 *
 * Search the wl_proc_list of the input wl_ctx and see if an already
 * wl_proc exists with the same pid as current->pid.
 * If yes, return the wl_proc
 * Else, alloc and init a wl_proc
 * Aquires wl_ctx->wl_ctx_lock before searching the wl_proc_list
 */
static struct isi_wl_proc *get_wl_proc(struct isi_wl_ctx *wl_ctx)
{
	struct isi_wl_proc *wl_proc;
	struct isi_wl_proc *pos;
	bool found = false;

	pid_t pid = current->pid;

	mutex_lock(&wl_ctx->wl_ctx_lock);
	if (hlist_empty(&wl_ctx->wl_proc_list)) {
		wl_proc = create_wl_proc(wl_ctx);
	} else {
		hlist_for_each_entry (pos, &wl_ctx->wl_proc_list,
				      wl_proc_node) {
			if (pos->pid == pid) {
				found = true;
				break;
			}
		}
		if (true == found)
			wl_proc = pos;
		else
			wl_proc = create_wl_proc(wl_ctx);
	}
	mutex_unlock(&wl_ctx->wl_ctx_lock);
	return wl_proc;
}

/**
 * add_async_cmd_request_to_bitmap - convert input
 *
 * @requests_arr	pointer to array containing requests
 * @num_requests	number of requests in the array
 * @bitmap			output bitmap
 *
 * Converts the input requests array into a 32-bit bitmap
 */
static int add_async_cmd_requests_to_bitmap(uint32_t *requests_arr,
					    unsigned int num_requests,
					    unsigned int *bitmap)
{
	uint32_t *request;
	int ret = 0;

	if ((!requests_arr) || (num_requests == 0) || (num_requests > 32) ||
	    (!bitmap)) {
		return -EINVAL;
	}
	request = requests_arr;
	while (num_requests > 0) {
		switch (*request) {
		case REQ_HOST_REBOOT:
		case REQ_TIMER_EXPIRY_NOTIFY:
		case REQ_NOK_WARNING_NOTIFY:
		case REQ_DIAG_DATA_TO_HOST:
		case REQ_START_DTI:
		case REQ_IEH_ERROR_GET:
		case REQ_ODCC_SNAPSHOT:
			*bitmap |= (1u << *request);
			break;
		default:
			ret = -EINVAL;
			goto err_async_cmd_req;
		}
		num_requests--;
		request++;
	}
err_async_cmd_req:
	return ret;
}

/**
 * rem_async_cmd_request_to_bitmap - Remove bits from bitmap
 *
 * @requests_arr	pointer to array containing requests
 * @num_requests	number of requests in the array
 * @bitmap			output bitmap
 *
 * Unset the bits, corresponding to the input requests array,
 * from the input bitmap
 */
static int rem_async_cmd_requests_from_bitmap(uint32_t *requests_arr,
					      unsigned int num_requests,
					      unsigned int *bitmap)
{
	uint32_t *request;
	int ret = 0;

	if ((!requests_arr) || (num_requests == 0) || (num_requests > 32) ||
	    (!bitmap)) {
		return -EINVAL;
	}
	request = requests_arr;
	while (num_requests > 0) {
		switch (*request) {
		case REQ_HOST_REBOOT:
		case REQ_TIMER_EXPIRY_NOTIFY:
		case REQ_NOK_WARNING_NOTIFY:
		case REQ_DIAG_DATA_TO_HOST:
		case REQ_START_DTI:
		case REQ_IEH_ERROR_GET:
		case REQ_ODCC_SNAPSHOT:
			*bitmap &= ~(1u << *request);
			break;
		default:
			ret = -EINVAL;
			goto err_async_cmd_dereg_req;
		}
		num_requests--;
		request++;
	}
err_async_cmd_dereg_req:
	return ret;
}

/**
 * reg_dereg_async_notify	Register/Deregister for async
 *							notification
 *
 * @cmd				Register/Deregister request
 * @wl_proc			wl_proc corresponding to the
 *					thread that requested reg/dereg
 *					for async notification
 * async_req		pointer to the async request structure
 *
 * based on the input command, register or deregister for input
 * request types. It will upadate the async_cmd_req_bitmap for
 * for the workload.
 *
 */
static int reg_dereg_async_notify(unsigned int cmd, struct isi_wl_proc *wl_proc,
				  struct isi_asynch_cmd_request *async_req)
{
	int requests_size;

	unsigned int cmd_req_bitmap = 0;
	int ret = 0;

	if ((async_req->number_of_requests == 0) ||
	    (async_req->number_of_requests >= (2 * SZ_1K)))
		return -EINVAL;
	else {
		requests_size =
			async_req->number_of_requests * sizeof(uint32_t);
	}

	if ((2 * SZ_1K) <= requests_size || async_req->requests == NULL)
		return -EINVAL;

	if (copy_from_user(wl_proc->tx_buf, async_req->requests,
			   requests_size)) {
		return -EINVAL;
	}

	if (async_req->workload_id != wl_proc->p_wl_ctx->wl_id)
		return -EINVAL;

	if (cmd == ISI_DRIVER_ASYNCH_CMD_REGISTER) {
		ret = add_async_cmd_requests_to_bitmap(
			wl_proc->tx_buf, async_req->number_of_requests,
			&cmd_req_bitmap);
	} else {
		ret = rem_async_cmd_requests_from_bitmap(
			wl_proc->tx_buf, async_req->number_of_requests,
			&cmd_req_bitmap);
	}
	if (ret)
		return ret;

	mutex_lock(&wl_proc->p_wl_ctx->wl_ctx_lock);
	wl_proc->p_wl_ctx->async_cmd_req_bitmap = cmd_req_bitmap;
	mutex_unlock(&wl_proc->p_wl_ctx->wl_ctx_lock);

	return ret;
}

/**
 * sync_cmd_req_resp	Send a sync request to ISI and
 *						wait for its response
 *
 * @wl_proc			ptr to wl_proc corresponding to the
 *					thread that requested send-recv op
 * @sync_req		pointer to the request structure
 *
 * Send the request received from the User application
 * to ISI and wait for its response. The response is updated
 * by the irq_thread in the requesting thread's wl_proc->rx_buf
 */
static int sync_cmd_req_resp(struct isi_wl_proc *wl_proc,
			     struct isi_synch_data_request *sync_req)
{
	int ret = 0;
	struct isi_driver_ctx *pctx;
	unsigned long timeout_jiffies = 0;
	uint8_t *dest_ptr;
	uint32_t rem_size;

	if ((!wl_proc) || (!sync_req))
		return -EINVAL;
	if ((!wl_proc->tx_buf) || (!wl_proc->rx_buf))
		return -ENOMEM;

	memcpy_s(wl_proc->tx_buf, sizeof(struct isi_generic_packet_header),
		 (void *)&sync_req->req_header,
		 sizeof(struct isi_generic_packet_header));
	dest_ptr = (uint8_t *)(wl_proc->tx_buf) +
		   sizeof(struct isi_generic_packet_header);

	if ((sync_req->req_buffer != NULL) &&
	    (sync_req->req_buffer_size != 0) &&
	    (sync_req->req_buffer_size <
	     (SZ_1K - sizeof(struct isi_generic_packet_header)))) {
		if (copy_from_user((void *)dest_ptr, sync_req->req_buffer,
				   (sync_req->req_buffer_size * 4))) {
			return -EINVAL;
		}
	}
	if (wl_proc->p_wl_ctx->wl_id != sync_req->workload_id) {
		return -EINVAL;
	}

	if (sync_req->timeout > 0) {
		timeout_jiffies = msecs_to_jiffies(sync_req->timeout);
	}

	if ((sync_req->req_buffer_size >= 0U) &&
	    (MAX_UINT_VALUE >= sync_req->req_buffer_size)) {
		wl_proc->tx_size =
			(sizeof(struct isi_generic_packet_header) / 4) +
			sync_req->req_buffer_size;
		wl_proc->seq_num = sync_req->req_header.sequence_number;
		pctx = wl_proc->p_wl_ctx->isi_ctx;
	} else
		return -EINVAL;

	if (sync_req->is_resp_required)
		proc_waitlist_add_safe(pctx, wl_proc, false);

	isi_hw_send(pctx, wl_proc->tx_buf, wl_proc->tx_size);

	if (sync_req->is_resp_required) {
		if (timeout_jiffies) {
			ret = wait_for_completion_interruptible_timeout(
				&wl_proc->sync_completion, timeout_jiffies);
			if (ret < 0) {
				proc_waitlist_del_safe(pctx, wl_proc);
			} else if (ret == 0) {
				proc_waitlist_del_safe(pctx, wl_proc);
				ret = -EBUSY;
			} else
				ret = 0;
		} else {
			ret = wait_for_completion_interruptible(
				&wl_proc->sync_completion);
			if (ret < 0) {
				proc_waitlist_del_safe(pctx, wl_proc);
				ret = -EINTR;
			}
		}
	}

	if (wl_proc->rx_msg_cnt > 0) {
		memcpy_s((void *)&sync_req->resp_header,
			 sizeof(struct isi_generic_packet_header),
			 wl_proc->rx_buf,
			 sizeof(struct isi_generic_packet_header));
		dest_ptr = (uint8_t *)(wl_proc->rx_buf) +
			   sizeof(struct isi_generic_packet_header);
		rem_size = wl_proc->rx_msg_cnt -
			   sizeof(struct isi_generic_packet_header);

		if (copy_to_user(sync_req->resp_buffer, dest_ptr, rem_size)) {
			ret = -EINVAL;
		} else {
			sync_req->resp_buffer_size =
				(rem_size) / sizeof(uint32_t);
		}
	}
	return ret;
}

/**
 * get_async_cmd_notify	Get the async command notification
 *
 * @wl_proc				ptr to wl_proc corresponding to the
 *						thread that requested to get the
 *						async cmd notification
 * @async_data			ptr to the user app passed async_data
 *
 * Queue the calling thread in the sync_cmd_waitlist and
 * wait for the irq_thread to wakeup the thread once any of the
 * async command matching the wl_proc->async_cmd_req_bitmap
 * is received. The received command is returned to the
 * calling thread.
 */
static int get_async_cmd_notify(struct isi_wl_proc *wl_proc,
				struct isi_asynch_data *async_data)
{
	struct isi_driver_ctx *pctx;
	struct isi_wl_ctx *p_wl_ctx;
	struct isi_circ_buf *cb;
	struct isi_generic_packet_header *pkt_header;
	uint32_t rem_size, rx_len = 0;
	unsigned int async_cmd_bitmap;
	int head = 0, tail = 0, cnt = 0, ret = 0;
	uint8_t *dest_ptr;
	uint32_t *p_rx_buf;

	p_wl_ctx = wl_proc->p_wl_ctx;
	pctx = p_wl_ctx->isi_ctx;

	if (!wl_proc->async_cmd_req_bitmap) {
		async_cmd_bitmap = p_wl_ctx->async_cmd_req_bitmap;
		if (!async_cmd_bitmap) {
			return -EINVAL;
		}
		wl_proc->async_cmd_req_bitmap = async_cmd_bitmap;
	}
	if (wl_proc->p_wl_ctx->wl_id != async_data->workload_id) {
		return -EINVAL;
	}

	proc_waitlist_add_safe(pctx, wl_proc, true);

	/*Now wait for a response*/
	ret = wait_for_completion_interruptible(&wl_proc->async_completion);

	if (ret != 0) {
		proc_waitlist_del_safe(pctx, wl_proc);
		ret = -EINTR;
		goto err_async_notify_get;
	}
	/* Read data from async_cmd_ring_buf */
	cb = &wl_proc->async_cmd_ring_buf;
	head = smp_load_acquire(&cb->head);
	tail = cb->tail;

	if (CIRC_CNT(head, tail, SZ_256) >= 1) {
		pkt_header = (struct isi_generic_packet_header *)&cb->buf[tail];
		rx_len = pkt_header->length;
		wl_proc->rx_msg_cnt = rx_len * 4;
		p_rx_buf = (uint32_t *)wl_proc->rx_buf;
		while (rx_len > 0) {
			p_rx_buf[cnt] = cb->buf[tail];
			cnt++;
			rx_len--;
			tail = (tail + 1) & (SZ_256 - 1);
		}
		/* Safely update the tail */
		smp_store_release(&cb->tail, tail);

		memcpy_s((void *)&async_data->header_buffer,
			 sizeof(struct isi_generic_packet_header),
			 wl_proc->rx_buf,
			 sizeof(struct isi_generic_packet_header));
		async_data->request_type =
			isi_cmd_to_req_type(async_data->header_buffer.command);
		dest_ptr = (uint8_t *)(wl_proc->rx_buf) +
			   sizeof(struct isi_generic_packet_header);
		rem_size = wl_proc->rx_msg_cnt -
			   sizeof(struct isi_generic_packet_header);

		if (copy_to_user(async_data->data_buffer, dest_ptr, rem_size)) {
			ret = -EINVAL;
			goto err_async_notify_get;
		}
		async_data->data_size = (rem_size) / sizeof(uint32_t);
	} else
		ret = -EINVAL;
err_async_notify_get:
	return ret;
}

static long isi_dev_ioctl(struct file *p_file, unsigned int cmd,
			  unsigned long arg)
{
	long ret = 0;
	struct isi_driver_ctx *pctx;
	struct isi_wl_ctx *wl_ctx = p_file->private_data;
	struct isi_wl_proc *wl_proc;
	struct isi_workload_details wl_details;
	struct isi_asynch_cmd_request async_req;
	struct isi_synch_data_request sync_req;
	struct isi_asynch_data async_data;
	unsigned int size = _IOC_SIZE(cmd);
	void __user *ubuf = (void __user *)arg;

	pctx = wl_ctx->isi_ctx;

	wl_proc = get_wl_proc(wl_ctx);
	if (!wl_proc) {
		ret = PTR_ERR(wl_proc);
		return ret;
	}

	switch (cmd) {
	case ISI_DRIVER_WORKLOAD_REGISTER:
	case ISI_DRIVER_WORKLOAD_DE_REGISTER:
		if (size != sizeof(struct isi_workload_details))
			return -EINVAL;
		if (copy_from_user(&wl_details, ubuf,
				   sizeof(struct isi_workload_details))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		ret = workload_reg_dereg(cmd, wl_proc, &wl_details);
		if (ret)
			goto isi_ioctl_err;

		if (copy_to_user(ubuf, &wl_details,
				 sizeof(struct isi_workload_details))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		cleanup_wl_proc_bufs(wl_proc);
		break;
	case ISI_DRIVER_ASYNCH_CMD_REGISTER:
	case ISI_DRIVER_ASYNCH_CMD_DE_REGISTER:
		if (size != sizeof(struct isi_asynch_cmd_request))
			ret = -EINVAL;
		if (copy_from_user(&async_req, ubuf,
				   sizeof(struct isi_asynch_cmd_request))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		if ((async_req.number_of_requests > 0U) &&
		    (async_req.number_of_requests <= (2 * SZ_1K))) {
			ret = reg_dereg_async_notify(cmd, wl_proc, &async_req);
		} else
			ret = -EINVAL;
		if (ret)
			goto isi_ioctl_err;

		cleanup_wl_proc_bufs(wl_proc);
		break;
	case ISI_DRIVER_SYNCH_CMD_TX_RX:
		if (size != sizeof(struct isi_synch_data_request))
			ret = -EINVAL;
		if (copy_from_user(&sync_req, ubuf,
				   sizeof(struct isi_synch_data_request))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		ret = sync_cmd_req_resp(wl_proc, &sync_req);
		if (ret) {
			goto isi_ioctl_err;
		}

		if (copy_to_user(ubuf, &sync_req,
				 sizeof(struct isi_synch_data_request))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		cleanup_wl_proc_bufs(wl_proc);
		break;
	case ISI_DRIVER_ASYNCH_MESSAGE_GET:
		if (size != sizeof(struct isi_asynch_data))
			ret = -EINVAL;
		if (copy_from_user(&async_data, ubuf,
				   sizeof(struct isi_asynch_data))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		ret = get_async_cmd_notify(wl_proc, &async_data);
		if (ret)
			goto isi_ioctl_err;

		if (copy_to_user(ubuf, &async_data,
				 sizeof(struct isi_asynch_data))) {
			ret = -EINVAL;
			goto isi_ioctl_err;
		}
		cleanup_wl_proc_bufs(wl_proc);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
isi_ioctl_err:
	return ret;
}

/**
 * create_wl_ctx - Allocate and init a Work Load context instance
 * @ctx:   pointer to the isi_driver_ctx structure
 * @tgid:	TGID
 *
 * Also adds the wl_ctx to the workload list pctx->wl_list.
 * Called with pctx->wl_list_lock held.
 */
static struct isi_wl_ctx *create_wl_ctx(struct isi_driver_ctx *pctx, pid_t tgid)
{
	struct isi_wl_ctx *p_wl_ctx;

	p_wl_ctx = kzalloc(sizeof(*p_wl_ctx), GFP_KERNEL);
	if (!p_wl_ctx)
		return NULL;
	p_wl_ctx->tgid = tgid;
	p_wl_ctx->isi_ctx = pctx;
	p_wl_ctx->wl_id = BROADCAST_ID;
	p_wl_ctx->async_cmd_req_bitmap =
		BIT(NUMBER_OF_ASYNCH_REQUEST_TYPES) - 1;
	mutex_init(&p_wl_ctx->wl_ctx_lock);
	INIT_HLIST_HEAD(&p_wl_ctx->wl_proc_list);
	INIT_HLIST_NODE(&p_wl_ctx->wl_ctx_node);

	hlist_add_head(&p_wl_ctx->wl_ctx_node, &pctx->wl_list);
	return p_wl_ctx;
}

/**
 * isi_dev_open - ISI Device open call
 * @p_inode:  Inode ptr
 * @p_file:   file ptr
 *
 * Create and associate a wl_ctx to every process.
 * Each child process/sibling of the calling thread
 * that have the same tgid share the same wl_ctx.
 *
 */
static int isi_dev_open(struct inode *p_inode, struct file *p_file)
{
	struct isi_driver_ctx *pctx;
	pid_t tgid;
	struct isi_wl_ctx *pos;
	struct isi_wl_ctx *wl_ctx;
	bool found = false;
	int ret = 0;

	tgid = current->tgid;
	pctx = container_of(p_inode->i_cdev, struct isi_driver_ctx, c_dev);
	mutex_lock(&pctx->wl_list_lock);
	if (!hlist_empty(&pctx->wl_list)) {
		hlist_for_each_entry (pos, &pctx->wl_list, wl_ctx_node) {
			if (pos->tgid == tgid) {
				found = true;
				wl_ctx = pos;
				break;
			}
		}
	}
	if (found == false)
		wl_ctx = create_wl_ctx(pctx, tgid);

	mutex_unlock(&pctx->wl_list_lock);
	if (IS_ERR(wl_ctx)) {
		ret = PTR_ERR(wl_ctx);
		return ret;
	}
	p_file->private_data = wl_ctx;
	nonseekable_open(p_inode, p_file);
	return ret;
}

/**
 * isi_dev_release - ISI device release call
 * @p_inode:	Inode pointer
 * @p_file:		File pointer
 *
 * Free up wl_procs assocated with this workload
 * and then remove the wl_Ctxt for this workload.
 *
 */
static int isi_dev_release(struct inode *p_inode, struct file *p_file)
{
	/* clean all wl_proc in wl_proc_list */
	struct isi_driver_ctx *pctx;
	pid_t tgid;
	struct isi_wl_ctx *pos, *wl_ctxt;
	struct isi_wl_proc *wl_proc;
	struct hlist_node *h_node, *prev_node;

	/* Get current tgid */
	tgid = current->tgid;
	pctx = container_of(p_inode->i_cdev, struct isi_driver_ctx, c_dev);
	mutex_lock(&pctx->wl_list_lock);
	if (!hlist_empty(&pctx->wl_list)) {
		/*
		 * Iterate over isi ctx->wl_list and
		 * see if there is already a wl_ctx matching the tgid
		 */
		hlist_for_each_entry (pos, &pctx->wl_list, wl_ctx_node) {
			if (pos->tgid == tgid) {
				wl_ctxt = pos;
				mutex_lock(&wl_ctxt->wl_ctx_lock);
				if (!hlist_empty(&wl_ctxt->wl_proc_list)) {
					hlist_for_each_safe (
						h_node, prev_node,
						&wl_ctxt->wl_proc_list) {
						wl_proc = hlist_entry(
							h_node,
							typeof(*wl_proc),
							wl_proc_node);
						proc_waitlist_del_safe(pctx,
								       wl_proc);
						hlist_del_init(
							&wl_proc->wl_proc_node);
						kfree(wl_proc->tx_buf);
						kfree(wl_proc->rx_buf);
						kfree(wl_proc->async_cmd_ring_buf
							      .buf);
						kfree(wl_proc);
					}
				}
				mutex_unlock(&wl_ctxt->wl_ctx_lock);
				hlist_del_init(&wl_ctxt->wl_ctx_node);
				kfree(wl_ctxt);
				break;
			}
		}
	}
	mutex_unlock(&pctx->wl_list_lock);
	p_file->private_data = NULL;

	return 0;
}

static int isi_dev_ctx_init(struct isi_driver_ctx *pctx)
{
	pctx->active_wl_cnt = 0;
	pctx->ring_buf.head = 0;
	pctx->ring_buf.tail = 0;
	mutex_init(&pctx->tx_lock);
	mutex_init(&pctx->wl_list_lock);
	spin_lock_init(&pctx->proc_waitlist_lock);
	spin_lock_init(&pctx->ring_buf_lock);
	INIT_HLIST_HEAD(&pctx->wl_list);
	INIT_HLIST_HEAD(&pctx->sync_cmd_waitlist);
	INIT_HLIST_HEAD(&pctx->async_cmd_waitlist);
	init_completion(&pctx->tx_ack);
	pctx->ring_buf.buf = kmalloc_array(SZ_256, SZ_4, GFP_KERNEL);
	if (!pctx->ring_buf.buf)
		return -ENOMEM;
	return 0;
}

static const struct file_operations isi_dev_fops = {
	.owner = THIS_MODULE,
	.open = isi_dev_open,
	.release = isi_dev_release,
	.unlocked_ioctl = isi_dev_ioctl,
	.compat_ioctl = isi_dev_ioctl,
};

static const struct pci_device_id isi_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x4b4a) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x5c10) },
	{
		0,
	}
};

static int isi_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct isi_driver_ctx *pctx;
	struct cdev *pcdev;
	struct device *dev_ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);
	ret = pcim_iomap_regions(pdev, 1 << 0, KBUILD_MODNAME);
	if (ret)
		return ret;

	pctx = devm_kzalloc(&pdev->dev, sizeof(*pctx), GFP_KERNEL);
	if (!pctx)
		return -ENOMEM;

	pctx->pdev = pdev;
	pctx->mmio = pcim_iomap_table(pctx->pdev)[0];

	ret = isi_dev_ctx_init(pctx);
	if (ret)
		return ret;

	ret = pci_alloc_irq_vectors(pctx->pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	if (ret < 0)
		return ret;

	ret = devm_request_threaded_irq(&pctx->pdev->dev, pctx->pdev->irq,
					isi_pci_dev_irq_handler,
					isi_irq_thread_fn, 0, "isi-irq-handler",
					pctx);
	if (ret)
		return ret;

	pci_set_drvdata(pdev, pctx);
	pcdev = &pctx->c_dev;
	cdev_init(pcdev, &isi_dev_fops);
	pcdev->owner = THIS_MODULE;

	ret = cdev_add(pcdev, isi_devt, ISI_NR_DEVICES);
	if (ret)
		return ret;

	dev_ret = device_create(isi_dev_cl, NULL, isi_devt, NULL, DEVICE_NAME);
	if (IS_ERR(dev_ret)) {
		ret = PTR_ERR(dev_ret);
		cdev_del(&pctx->c_dev);
		return ret;
	}
	pctx->dev = dev_ret;
	return ret;
}

static void isi_pci_remove(struct pci_dev *pdev)
{
	struct isi_driver_ctx *pctx = pci_get_drvdata(pdev);

	kfree(pctx->ring_buf.buf);
	device_destroy(isi_dev_cl, isi_devt);
	cdev_del(&pctx->c_dev);
}

static struct pci_driver isi_pci_dev = {
	.name = "ISI mailbox driver",
	.id_table = isi_pci_tbl,
	.probe = isi_pci_probe,
	.remove = isi_pci_remove,
};

static int __init isi_dev_init(void)
{
	int status;

	status = alloc_chrdev_region(&isi_devt, 0, ISI_NR_DEVICES, DEVICE_NAME);
	if (status)
		return status;

	isi_dev_cl = class_create(THIS_MODULE, "isi");
	if (IS_ERR(isi_dev_cl)) {
		status = PTR_ERR(isi_dev_cl);
		goto err_isi_class_create;
	}
	status = pci_register_driver(&isi_pci_dev);
	if (status)
		goto err_isi_pci_register;

	return status;

err_isi_pci_register:
	class_destroy(isi_dev_cl);
err_isi_class_create:
	unregister_chrdev_region(isi_devt, ISI_NR_DEVICES);
	return status;
}

static void __exit isi_dev_exit(void)
{
	pci_unregister_driver(&isi_pci_dev);
	class_destroy(isi_dev_cl);
	unregister_chrdev_region(isi_devt, ISI_NR_DEVICES);
}

module_init(isi_dev_init);
module_exit(isi_dev_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Malhotra, Rahil <rahil.malhotra@intel.com>");
MODULE_DESCRIPTION("Driver to interact with ISI");
