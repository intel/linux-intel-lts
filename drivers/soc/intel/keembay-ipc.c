// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay IPC Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * This driver enables Inter-Processor Communication (IPC) between the
 * Computing Sub-System (CSS) and the Multimedia Sub-System (MSS) of Keem Bay.
 * CSS is where Linux is running; while MSS is where the VPU firmware runs.
 *
 * The IPC uses the following terminology:
 *
 * - Node:    A processing entity that can use the IPC to communicate
 *	      (currently, we just have two nodes, CSS and MSS).
 *
 * - Link:    Two nodes that can communicate over IPC form an IPC link
 *	      (currently, we just have one link, the one formed by CSS and MSS).
 *
 * - Channel: An IPC link can provide multiple IPC channels. IPC channels allow
 *            communication multiplexing, i.e., the same IPC link can be used
 *            by different applications for different communications. Each
 *            channel is identified by a channel ID, which must be unique
 *            within a single IPC link. Channels are divided in two categories,
 *            High-Speed (HS) channels and General-Purpose (GP) channels.
 *            HS channels have higher priority over GP channels.
 *
 * Keem Bay IPC mechanism is based on shared memory and hardware FIFOs. Both
 * CSS and MSS have their own hardware FIFO. When CSS wants to send an IPC
 * message to MSS, it writes to the MSS FIFO; similarly, when MSS wants to send
 * an IPC message to CSS, it writes to the CSS FIFO.
 *
 * A FIFO entry is simply a pointer to an IPC buffer (aka IPC header) stored in
 * a portion of memory shared between CSS and MSS. Specifically, the FIFO entry
 * contains the (VPU) physical address of the IPC buffer being transferred.
 *
 * In turn, the IPC buffer contains the (VPU) physical address of the payload
 * (which must be located in shared memory too) as well as other information
 * (payload size, IPC channel ID, etc.).
 *
 * Each IPC node instantiates a pool of IPC buffers from its own IPC buffer
 * memory region. When instantiated, IPC buffers are marked as free. When the
 * node needs to send an IPC message, it gets the first free buffer it finds
 * (from its own pool), marks it as allocated (used), and puts its physical
 * address into the IPC FIFO of the destination node. The destination node
 * (which is notified by an interrupt when there are entries pending in its
 * FIFO) extract the physical address from the FIFO and process the IPC buffer,
 * marking it as free once done (so that the sender can reuse the buffer).
 *
 * Note: Keem Bay IPC is not based on RPMsg, since MSS HW/FW does not support
 * Virtio and Virtqueues.
 */

#include <linux/circ_buf.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/soc/intel/keembay-ipc.h>

#define DRV_NAME		"keembay-ipc"

/* The alignment to be used for IPC Buffers and IPC Data. */
#define KMB_IPC_ALIGNMENT	64

/* Maximum number of channels per link. */
#define KMB_IPC_MAX_CHANNELS	1024

/* The number of high-speed channels per link. */
#define KMB_IPC_NUM_HS_CHANNELS	10

/* The IPC Node ID of this node. */
#define MY_NODE_ID		KMB_IPC_NODE_ARM_CSS
/*
 * This is used as index for retrieving reserved memory from the device
 * tree.
 */
#define LOCAL_IPC_BUFFER_IDX	0
#define REMOTE_IPC_BUFFER_IDX	1

/*
 * The length of the RX software FIFO.
 *
 * Note: This must be a power of 2 as required by the circular buffer API
 * defined in "linux/circ_buf.h".
 */
#define RX_SW_FIFO_LEN		256

/*
 * The IPC FIFO registers (offsets to the base address defined in device tree).
 */
/* Read/Write a word from/to FIFO. */
#define IPC_FIFO		0x00
/* Read from FIFO using ATM mode (currently not used by this driver). */
#define IPC_FIFO_ATM		0x04
/* FIFO Status. */
#define IPC_FIFO_STAT		0x0C
/* IPC FIFO overflow status, IDs 63-0 */
#define IPC_FIFO_OF_FLAG0	0x10
#define IPC_FIFO_OF_FLAG1	0x14

/* The possible states of an IPC buffer. */
enum {
	/*
	 * KMB_IPC_BUF_FREE must be set to 0 to ensure that buffers can be
	 * initialized with memset(&buf, 0, sizeof(buf)).
	 */
	KMB_IPC_BUF_FREE = 0,
	KMB_IPC_BUF_ALLOCATED,
};

/**
 * struct kmb_ipc_buf - The IPC buffer structure.
 * @data_addr:	The address where the IPC payload is located; NOTE: this is a
 *		VPU address (not a CPU one).
 * @data_size:	The size of the payload.
 * @channel:	The channel used.
 * @src_node:	The Node ID of the sender.
 * @dst_node:	The Node ID of the intended receiver.
 * @status:	Either free or allocated.
 */
struct kmb_ipc_buf {
	u32 data_addr;
	u32 data_size;
	u16 channel;
	u8  src_node;
	u8  dst_node;
	u8  status;
} __packed __aligned(KMB_IPC_ALIGNMENT);

/**
 * struct ipc_buf_mem - IPC Buffer Memory Region.
 * @dev:	Child device managing the memory region.
 * @vaddr:	The virtual address of the memory region.
 * @dma_handle:	The VPU address of the memory region.
 * @size:	The size of the memory region.
 */
struct ipc_buf_mem {
	struct device *dev;
	void *vaddr;
	dma_addr_t dma_handle;
	size_t size;
};

/**
 * struct ipc_buf_pool - IPC buffer pool.
 * @buffers:	Pointer to the array of buffers.
 * @buf_cnt:	Pool size (i.e., number of buffers).
 * @idx:	Current index.
 * @lock:	The lock protecting this pool.
 */
struct ipc_buf_pool {
	struct kmb_ipc_buf *buffers;
	size_t buf_cnt;
	size_t idx;
	spinlock_t lock; /* Protects 'idx' and 'buffers'. */
};

/**
 * struct ipc_chan - IPC channel.
 * @rx_data_list:	The list of incoming messages.
 * @rx_lock:		The lock for modifying the rx_data_list.
 * @rx_wait_queue:	The wait queue for RX Data (recv() waits on it).
 * @closing:		Closing flag, set when the channel is being closed.
 */
struct ipc_chan {
	struct list_head rx_data_list;
	spinlock_t rx_lock; /* Protects 'rx_data_list'. */
	wait_queue_head_t rx_wait_queue;
	bool closing;
};

/**
 * struct tx_data - Element of a TX queue.
 * @vpu_addr:	The VPU address of the data to be transferred.
 * @size:	The size of the data to be transferred.
 * @chan_id:	The channel to be used for the transfer.
 * @dst_node:	The destination node.
 * @retv:	The result of the transfer.
 * @list:	The list head used to concatenate TX data elements.
 * @tx_done:	The completion struct used by the sender to wait for the
 *		transfer to complete.
 */
struct tx_data {
	u32 vpu_addr;
	u32 size;
	u16 chan_id;
	u8  dst_node;
	int retv;
	struct list_head list;
	struct completion tx_done;
};

/**
 * struct tx_queue - The TX queue structure.
 * @tx_data_list: The list of pending TX data on this queue.
 * @lock:	  The lock protecting the TX data list.
 */
struct tx_queue {
	struct list_head tx_data_list;
	spinlock_t lock;	/* Protects tx_data_list. */
};

/**
 * struct ipc_link - An IPC link.
 * @ipc_chan:	 The channels associated with this link (the pointers to the
 *		 channels are RCU-protected).
 * @chan_lock:	 The lock for modifying the channels array.
 * @srcu_sp:	 The Sleepable RCU structs associated with the link's channels.
 * @tx_queues:	 The TX queues for this link (1 queue for each high-speed
 *		 channels + 1 shared among all the general-purpose channels).
 * @tx_qidx:	 The index of the next tx_queue to be check for outgoing data.
 * @tx_queued:	 The TX completion used to signal when new TX data is pending.
 * @tx_thread:	 The TX thread.
 * @tx_stopping: Flag signaling that the IPC Link is being closed.
 */
struct ipc_link {
	struct ipc_chan __rcu *channels[KMB_IPC_MAX_CHANNELS];
	spinlock_t chan_lock; /* Protects modifications to 'channels'. */
	struct srcu_struct srcu_sp[KMB_IPC_MAX_CHANNELS];
	struct tx_queue tx_queues[KMB_IPC_NUM_HS_CHANNELS + 1];
	int tx_qidx;
	struct completion tx_queued;
	struct task_struct *tx_thread;
	bool tx_stopping;
};

/*
 * RX Circular Buffer struct.
 *
 * The producer inserts elements to the head and the consumers extracts
 * elements from the tail. See Documentation/circular-buffers.txt for usage
 * details.
 */
struct rx_circ_buf {
	u32 buf[RX_SW_FIFO_LEN];
	int head;
	int tail;
};

/**
 * struct keembay_ipc_dev - IPC private data.
 *
 * @plat_dev: Platform device for this driver.
 * @local_ipc_mem:	    Local IPC Buffer memory region.
 * @remote_ipc_mem:	    Remove IPC Buffer memory region.
 * @ipc_buf_pool:	    The pool of IPC buffers.
 * @leon_mss_link:	    The ARM_CSS-Leon_MSS link.
 * @local_fifo_irq:	    The IRQ line used by the local hardware FIFO.
 * @local_fifo_irq_enabled: Whether or not the local hardware FIFO IRQ is
 *			    enabled.
 * @local_fifo_irq_lock:    The spinlock to protect the enabled status of the
 *			    local FIFO IRQ.
 * @local_fifo_reg:	    The base address of the local HW FIFO.
 * @remote_fifo_reg:	    The base address of the remote HW FIFO (i.e., the
 *			    Leon MSS FIFO).
 * @rx_sw_fifo		    The RX SW FIFO. The RX ISR writes to this FIFO and
 *			    the RX tasklet reads from it.
 */
struct keembay_ipc_dev {
	struct platform_device *plat_dev;
	struct ipc_buf_mem local_ipc_mem;
	struct ipc_buf_mem remote_ipc_mem;
	struct ipc_buf_pool ipc_buf_pool;
	struct ipc_link leon_mss_link;
	int local_fifo_irq;
	int local_fifo_irq_enabled;
	spinlock_t local_fifo_irq_lock; /* Protects status of FIFO IRQ. */
	void __iomem *local_fifo_reg;
	void __iomem *remote_fifo_reg;
	struct tasklet_struct rx_tasklet;
	struct rx_circ_buf rx_sw_fifo;
};

/*
 * RX Data Descriptor.
 *
 * Instances of this struct are meant to be inserted in the RX Data queue
 * (list) associated with each channel.
 */
struct rx_data {
	/* The VPU address of the received data. */
	u32 data_vpu_addr;
	/* The size of the received data. */
	u32 data_size;
	/* List head for creating a list of rx_data elements. */
	struct list_head list;
};

/* Forward declaration of TX thread function. */
static int tx_thread_fn(void *data);
/* Forward declaration of ISR function. */
static irqreturn_t local_fifo_irq_handler(int irq, void *dev_id);
/* Forward declaration of RX tasklet function. */
static void rx_tasklet_func(unsigned long);

/*
 * Functions related to reserved-memory sub-devices.
 */

/*
 * init_ipc_rsvd_mem() - Init the specified IPC reserved memory.
 * @dev:	The IPC device for which the memory will be initialized.
 * @mem:	Where to stored information about the initialized memory.
 * @mem_name:	The name of this IPC memory.
 * @mem_idx:	The index of the memory to initialize.
 *
 * Create a child device for 'dev' and use it to initialize the reserved
 * memory region specified in the device tree at index 'mem_idx'.
 * Also allocate DMA memory from the initialized memory region.
 *
 * Return:	0 on success, negative error code otherwise.
 */
static int init_ipc_rsvd_mem(struct device *dev, struct ipc_buf_mem *mem,
			     const char *mem_name, unsigned int mem_idx)
{
	struct device *mem_dev;
	struct device_node *np;
	struct resource mem_res;
	size_t mem_size;
	void *vaddr;
	dma_addr_t dma_handle;
	int rc;

	/* Create a child device (of dev) to own the reserved memory. */
	mem_dev = devm_kzalloc(dev, sizeof(struct device), GFP_KERNEL);
	if (!mem_dev)
		return -ENOMEM;

	device_initialize(mem_dev);
	dev_set_name(mem_dev, "%s:%s", dev_name(dev), mem_name);
	mem_dev->parent = dev;
	mem_dev->dma_mask = dev->dma_mask;
	mem_dev->coherent_dma_mask = dev->coherent_dma_mask;
	/* Set up DMA configuration using information from parent's DT node. */
	rc = of_dma_configure(mem_dev, dev->of_node, true);
	mem_dev->release = of_reserved_mem_device_release;

	rc = device_add(mem_dev);
	if (rc)
		goto err;

	/* Initialized the device reserved memory region. */
	rc = of_reserved_mem_device_init_by_idx(mem_dev, dev->of_node, mem_idx);
	if (rc) {
		dev_err(dev, "Couldn't get reserved memory with idx = %d, %d\n",
			mem_idx, rc);
		device_del(mem_dev);
		goto err;
	}

	/* Find out the size of the memory region. */
	np = of_parse_phandle(dev->of_node, "memory-region", mem_idx);
	if (!np) {
		dev_err(dev, "Couldn't find memory-region %d\n", mem_idx);
		rc = -EINVAL;
		goto err;
	}
	rc = of_address_to_resource(np, 0, &mem_res);
	if (rc) {
		dev_err(dev, "Couldn't map address to resource %d\n", mem_idx);
		goto err;
	}
	mem_size = resource_size(&mem_res);

	/* Allocate memory from the reserved memory regions */
	vaddr = dmam_alloc_coherent(mem_dev, mem_size, &dma_handle, GFP_KERNEL);
	if (!vaddr) {
		dev_err(mem_dev, "Failed to allocate from reserved memory.\n");
		rc = -ENOMEM;
		goto err;
	}

	mem->dev = mem_dev;
	mem->vaddr = vaddr;
	mem->dma_handle = dma_handle;
	mem->size = mem_size;

	return 0;

err:
	put_device(mem_dev);
	return rc;
}

/*
 * Keem Bay IPC HW FIFO API.
 *
 * We use two HW FIFOs: the local one and the remote one (Leon MSS FIFO).
 *
 * The local FIFO holds incoming IPC FIFO entries and therefore is used by RX
 * code.
 *
 * The remote FIFO is where we put outgoing FIFO entries and therefore is used
 * by TX code.
 */

/* Get number of entries in the FIFO. */
static int local_fifo_cnt(struct keembay_ipc_dev *ipc_dev)
{
	/*
	 * TIM_IPC_FIFO_STAT
	 *
	 * Bits 31:24: Reserved
	 * Bits 23:16: IPC FIFO fill level
	 * Bits 15:8:  IPC FIFO write pointer
	 * Bits  7:0:  IPC FIFO read pointer
	 */
	const u32 val = ioread32(ipc_dev->local_fifo_reg + IPC_FIFO_STAT);

	return (val >> 16) & 0xFF;
}

/* Extract an entry from the FIFO. */
static u32 local_fifo_get(struct keembay_ipc_dev *ipc_dev)
{
	/*
	 * TIM_IPC_FIFO_ATM
	 *
	 * Read a value from IPC FIFO.
	 *
	 * If FIFO is empty return 0xFFFFFFFF, otherwise return value from FIFO
	 * with 6 LSBs set to 0.
	 * Increment FIFO read pointer and decrement fill level.
	 */
	return ioread32(ipc_dev->local_fifo_reg + IPC_FIFO_ATM);
}

/* Disable local FIFO IRQ. */
static void local_fifo_irq_disable(struct keembay_ipc_dev *ipc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&ipc_dev->local_fifo_irq_lock, flags);
	if (ipc_dev->local_fifo_irq_enabled) {
		disable_irq_nosync(ipc_dev->local_fifo_irq);
		ipc_dev->local_fifo_irq_enabled = false;
	}
	spin_unlock_irqrestore(&ipc_dev->local_fifo_irq_lock, flags);
}

/* Enable local FIFO IRQ. */
static void local_fifo_irq_enable(struct keembay_ipc_dev *ipc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&ipc_dev->local_fifo_irq_lock, flags);
	if (!ipc_dev->local_fifo_irq_enabled) {
		enable_irq(ipc_dev->local_fifo_irq);
		ipc_dev->local_fifo_irq_enabled = true;
	}
	spin_unlock_irqrestore(&ipc_dev->local_fifo_irq_lock, flags);
}

/* Add an entry to a remote FIFO. */
static void remote_fifo_put(struct keembay_ipc_dev *ipc_dev, u32 entry)
{
	/*
	 * TIM_IPC_FIFO.
	 *
	 * Write a value to IPC FIFO if not full.
	 *
	 * Increment FIFO write pointer and fill level.
	 *
	 * In normal usage the 6 LSBs bits are reserved for the writing
	 * processor to include its processor ID, 0 <= x <= 62, so it can
	 * determine if the word was written correctly by checking the
	 * appropriate bit of register TIM_IPC_FIFO_OF_FLAGn.
	 */
	iowrite32(entry, ipc_dev->remote_fifo_reg + IPC_FIFO);
}

/*
 * Check if we have overflowed the remote FIFO.
 *
 * Check the overflow register of the remote FIFO to see if we have overflowed
 * it. If so clear our overflow.
 *
 * @return True if we have overflowed the FIFO, false otherwise.
 *
 */
static bool remote_fifo_overflow(struct keembay_ipc_dev *ipc_dev)
{
	/*
	 * TIM_IPC_FIFO_OF_FLAGn
	 *
	 * Read:
	 *
	 * A processor with ID = x can check that its writes to the IPC FIFO
	 * were successful by reading 0 from bit x of TIM_IPC_FIFO_OF_FLAG0 or
	 * TIM_IPC_FIFO_OF_FLAG1.
	 *
	 * Bit x, 0 <= x <= 31, of TIM_IPC_FIFO_OF_FLAG0 is set high if a write
	 * to TIM_IPC_FIFO by processor ID x failed because the FIFO was full.
	 *
	 * Bit x, 0 <= x <= 30, of TIM_IPC_FIFO_OF_FLAG1 is set high if a write
	 * to TIM_IPC_FIFO by processor ID x+32 failed because the FIFO was
	 * full.
	 *
	 * Processors are identified by the 6 LSBs of words written to
	 * TIM_IPC_FIFO, i.e. x = TIM_IPC_FIFO[5:0].  Processor ID = 0x3F is
	 * reserved to indicate a read of an empty FIFO has occurred.
	 *
	 * Write:
	 *
	 * Writing 1 to bit position x of TIM_IPC_FIFO_OF_FLAG0 clears the
	 * overflow flag corresponding to processor ID x.  Writing 1 to bit
	 * position x of TIM_IPC_FIFO_OF_FLAGn clears the overflow flag
	 * corresponding to processor ID x+32.
	 *
	 * Writing 0 to any bit position has not effect.
	 */
	const u32 offset = (MY_NODE_ID < 32) ? IPC_FIFO_OF_FLAG0
						  : IPC_FIFO_OF_FLAG1;
	const u32 mask = 1 << (MY_NODE_ID % 32);
	const bool rc = ioread32(ipc_dev->remote_fifo_reg + offset) & mask;
	/* Clear our overflow bit, if we overflowed. */
	if (rc)
		iowrite32(mask, ipc_dev->remote_fifo_reg + offset);

	return rc;
}

/*
 * IPC internal functions.
 */

/**
 * channel_close() - Close a channel and return whether it was open or not.
 * @link:	The link the channel belongs to.
 * @chan_id:	The channel ID of the channel to close.
 *
 * Return:	0 if the channel was already closed, 1 otherwise.
 */
static int channel_close(struct ipc_link *link, u16 chan_id)
{
	struct ipc_chan *chan;
	struct rx_data *pos, *nxt;
	unsigned long flags;

	/* Remove channel from channel array. */
	spin_lock_irqsave(&link->chan_lock, flags);
	chan = rcu_dereference_protected(link->channels[chan_id],
					 lockdep_is_held(&link->chan_lock));
	RCU_INIT_POINTER(link->channels[chan_id], NULL);
	spin_unlock_irqrestore(&link->chan_lock, flags);

	/* If channel was NULL, we are done. */
	if (!chan)
		return 0;
	/* Set closing flag and wake up user threads waiting on recv(). */
	chan->closing = true;
	wake_up_all(&chan->rx_wait_queue);
	/* Wait for channel users to drop the reference to the old channel. */
	synchronize_srcu(&link->srcu_sp[chan_id]);
	/* Free channel memory (rx_data queue and channel struct). */
	/*
	 * No need to get chan->rx_lock as we know that nobody is using the
	 * channel at this point.
	 */
	list_for_each_entry_safe(pos, nxt, &chan->rx_data_list, list) {
		list_del(&pos->list);
		kfree(pos);
	}
	kfree(chan);

	return 1;
}

/**
 * ipc_buf_tx_alloc() - Allocate an IPC buffer to be used for TX.
 * @pool:  The IPC buffer pool from which the IPC buffer will be allocated.
 *
 * Return: The pointer to the allocated buffer, or NULL if allocation fails.
 */
static struct kmb_ipc_buf *ipc_buf_tx_alloc(struct ipc_buf_pool *pool)
{
	struct kmb_ipc_buf *buf;
	size_t i = 0;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	/*
	 * Look for a free buffer starting from the last index (pointing to the
	 * next buffer after the last allocated one) and potentially going
	 * through all the buffers in the pool.
	 */
	for (i = 0; i < pool->buf_cnt; i++) {
		/*
		 * Get reference to current buffer and increment index (for
		 * next iteration or function call).
		 */
		buf = &pool->buffers[pool->idx++];
		if (pool->idx == pool->buf_cnt)
			pool->idx = 0;
		/* Use current buffer if free. */
		if (buf->status == KMB_IPC_BUF_FREE) {
			buf->status = KMB_IPC_BUF_ALLOCATED;
			spin_unlock_irqrestore(&pool->lock, flags);
			return buf;
		}
	}
	spin_unlock_irqrestore(&pool->lock, flags);
	/* We went through all the buffers and found none free; return error. */
	return NULL;
}

/**
 * init_ipc_buf_pool() - Init the IPC Buffer Pool.
 * @ipc_dev:	The IPC device the pool belongs to.
 *
 * Set up the IPC Buffer Pool to be used for allocating TX buffers.
 *
 * The pool uses the local IPC Buffer memory previously allocated.
 *
 * Return:	0 on success, negative error code otherwise.
 */
static int init_ipc_buf_pool(struct keembay_ipc_dev *ipc_dev)
{
	struct ipc_buf_mem *mem = &ipc_dev->local_ipc_mem;
	/* Initialize ipc_buf_poll global variable. */
	/*
	 * Start by setting everything to 0 to initialize the IPC Buffer array
	 * (this works because the value of KMB_IPC_BUF_FREE is 0).
	 */
	memset(mem->vaddr, 0, mem->size);
	ipc_dev->ipc_buf_pool.buffers = mem->vaddr;
	ipc_dev->ipc_buf_pool.buf_cnt = mem->size / sizeof(struct kmb_ipc_buf);
	ipc_dev->ipc_buf_pool.idx = 0;
	dev_info(&ipc_dev->plat_dev->dev, "IPC Buffer Pool size: %zu\n",
		 ipc_dev->ipc_buf_pool.buf_cnt);
	spin_lock_init(&ipc_dev->ipc_buf_pool.lock);

	return 0;
}

/*
 * ipc_link_init() - Initialize an IPC link.
 * @ipc_dev: The IPC device the link belongs to.
 * @link:    The link to initialize.
 *
 * This function is expected to be called during probing.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int ipc_link_init(struct keembay_ipc_dev *ipc_dev, struct ipc_link *link)
{
	struct tx_queue *queue;
	int i;

	spin_lock_init(&link->chan_lock);
	for (i = 0; i < ARRAY_SIZE(link->srcu_sp); i++)
		init_srcu_struct(&link->srcu_sp[i]);
	memset(link->channels, 0, sizeof(link->channels));
	/* Init TX queues. */
	for (i = 0; i < ARRAY_SIZE(link->tx_queues); i++) {
		queue = &link->tx_queues[i];
		INIT_LIST_HEAD(&queue->tx_data_list);
		spin_lock_init(&queue->lock);
	}
	link->tx_qidx = 0;
	link->tx_stopping = false;
	init_completion(&link->tx_queued);
	/* Start TX thread. */
	link->tx_thread = kthread_run(tx_thread_fn, ipc_dev,
				      "kmb_ipc_tx_thread");
	return PTR_ERR_OR_ZERO(link->tx_thread);
}

/**
 * ipc_link_deinit() - De-initialize an IPC link.
 * @ipc_dev:	The IPC device the link belongs to.
 * @link:	The link to de-initialize.
 */
static void ipc_link_deinit(struct keembay_ipc_dev *ipc_dev,
			    struct ipc_link *link)
{
	struct tx_queue *queue;
	struct tx_data *pos, *nxt;
	int i;

	/* Close all channels. */
	for (i = 0; i < ARRAY_SIZE(link->channels); i++)
		channel_close(link, i);
	/* Stop TX Thread. */
	link->tx_stopping = true;
	complete(&link->tx_queued);
	kthread_stop(link->tx_thread);
	/* Flush all TX queue. */
	for (i = 0; i < ARRAY_SIZE(link->tx_queues); i++) {
		queue = &link->tx_queues[i];
		list_for_each_entry_safe(pos, nxt, &queue->tx_data_list, list) {
			list_del(&pos->list);
			pos->retv = -EPIPE;
			complete(&pos->tx_done);
		}
	}
}

static int get_pdev_res_and_ioremap(struct platform_device *pdev,
				    const char *reg_name,
				    void __iomem **target_reg)
{
	struct resource *res;
	void __iomem *reg;
	struct device *dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, reg_name);
	if (!res) {
		dev_err(dev, "Couldn't get resource for %s\n", reg_name);
		return -EINVAL;
	}

	reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(reg)) {
		dev_err(dev, "Couldn't ioremap resource for %s\n", reg_name);
		return PTR_ERR(reg);
	}
	dev_info(dev, "Register base for %s: vaddr %p paddr: %pa\n", reg_name,
		 reg, &res->start);

	*target_reg = reg;

	return 0;
}

/**
 * ipc_hw_init() - Init IPC-related hardware.
 */
static int ipc_hw_init(struct platform_device *pdev,
		       struct keembay_ipc_dev *ipc_dev)
{
	struct device *dev = &pdev->dev;
	int rc, irq;

	rc = get_pdev_res_and_ioremap(pdev, "css_fifo",
				      &ipc_dev->local_fifo_reg);
	if (rc) {
		dev_err(dev, "Failed to get local FIFO registers\n");
		return rc;
	}

	rc = get_pdev_res_and_ioremap(pdev, "mss_fifo",
				      &ipc_dev->remote_fifo_reg);
	if (rc) {
		dev_err(dev, "Failed to get remote FIFO registers\n");
		return rc;
	}

	/*
	 * Register interrupt handler and initialize IRQ-related fields in IPC
	 * device struct.
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ipc_dev->local_fifo_irq = irq;
	ipc_dev->local_fifo_irq_enabled = true;
	tasklet_init(&ipc_dev->rx_tasklet, rx_tasklet_func, (uintptr_t)ipc_dev);
	spin_lock_init(&ipc_dev->local_fifo_irq_lock);
	dev_info(&pdev->dev, "Registering handler for IRQ %d\n", irq);
	rc = devm_request_irq(&pdev->dev, irq, local_fifo_irq_handler, 0,
			      dev_name(&pdev->dev), ipc_dev);
	return 0;
}

/**
 * ipc_hw_deinit() - De-initialize IPC-related hardware and memory.
 */
static void ipc_hw_deinit(struct keembay_ipc_dev *ipc_dev)
{
	/*
	 * Just kill the tasklet as iomem and irq have been requested with
	 * devm_* functions and, therefore, are freed automatically.
	 */
	tasklet_kill(&ipc_dev->rx_tasklet);
}

/* Driver probing. */
static int kmb_ipc_probe(struct platform_device *pdev)
{
	int rc;
	struct keembay_ipc_dev *ipc_dev;
	struct device *dev = &pdev->dev;

	ipc_dev = devm_kzalloc(dev, sizeof(*ipc_dev), GFP_KERNEL);
	if (!ipc_dev)
		return -ENOMEM;

	ipc_dev->plat_dev = pdev;

	/* Init the memory used for local IPC buffers. */
	rc = init_ipc_rsvd_mem(dev, &ipc_dev->local_ipc_mem,
			       "local_rsvd_mem", LOCAL_IPC_BUFFER_IDX);
	if (rc) {
		dev_err(dev, "Failed to set up local reserved memory.\n");
		return rc;
	}
	/* Init the memory used for remote IPC buffers. */
	rc = init_ipc_rsvd_mem(dev, &ipc_dev->remote_ipc_mem,
			       "remote_rsvd_mem", REMOTE_IPC_BUFFER_IDX);
	if (rc) {
		dev_err(dev, "Failed to set up remote reserved memory.\n");
		device_unregister(ipc_dev->local_ipc_mem.dev);
		return rc;
	}
	/* Print out some information about the configured memory. */
	dev_info(dev, "Local vaddr 0x%p vpu_addr %pad size 0x%zX\n",
		 ipc_dev->local_ipc_mem.vaddr,
		 &ipc_dev->local_ipc_mem.dma_handle,
		 ipc_dev->local_ipc_mem.size);
	dev_info(dev, "Remote vaddr 0x%p vpu_addr %pad size 0x%zX\n",
		 ipc_dev->remote_ipc_mem.vaddr,
		 &ipc_dev->remote_ipc_mem.dma_handle,
		 ipc_dev->remote_ipc_mem.size);

	/* Init the pool of IPC Buffers to be used to TX. */
	init_ipc_buf_pool(ipc_dev);
	/* Init the only link we have (ARM CSS -> LEON MSS). */
	rc = ipc_link_init(ipc_dev, &ipc_dev->leon_mss_link);
	if (rc)
		return rc;

	/* Init IPC HW FIFO. */
	ipc_hw_init(pdev, ipc_dev);

	platform_set_drvdata(pdev, ipc_dev);

	return 0;
}

/* Driver removal. */
static int kmb_ipc_remove(struct platform_device *pdev)
{
	struct keembay_ipc_dev *ipc_dev = platform_get_drvdata(pdev);

	ipc_hw_deinit(ipc_dev);

	ipc_link_deinit(ipc_dev, &ipc_dev->leon_mss_link);

	/*
	 * No need to de-alloc IPC memory (local_ipc_mem and remote_ipc_mem)
	 * since it was allocated with dmam_alloc.
	 */

	device_unregister(ipc_dev->local_ipc_mem.dev);
	device_unregister(ipc_dev->remote_ipc_mem.dev);

	return 0;
}

/*
 * ipc_vpu_to_virt() - Convert a VPU address to a CPU virtual address.
 *
 * @ipc_mem:  The IPC memory region where the VPU address is expected to be
 *	      mapped to.
 * @vpu_addr: The VPU address to be converted to a virtual one.
 *
 * Return: The corresponding CPU virtual address, or NULL if the VPU address
 *	   is not in the expected memory range.
 */
static void *ipc_vpu_to_virt(const struct ipc_buf_mem *ipc_mem,
			     u32 vpu_addr)
{
	if (unlikely(vpu_addr < ipc_mem->dma_handle ||
		     vpu_addr >= (ipc_mem->dma_handle + ipc_mem->size)))
		return NULL;
	return ipc_mem->vaddr + (vpu_addr - ipc_mem->dma_handle);
}

/*
 * ipc_virt_to_vpu() - Convert an CPU virtual address to a VPU address.
 * @ipc_mem:  [in]  The IPC memory region where the VPU address is expected to
 *		    be mapped to.
 * @vaddr:    [in]  The CPU virtual address to be converted to a VPU one.
 * @vpu_addr: [out] Where to store the computed VPU address.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int ipc_virt_to_vpu(struct ipc_buf_mem *ipc_mem, void *vaddr,
			   u32 *vpu_addr)
{
	if (unlikely((ipc_mem->dma_handle + ipc_mem->size) > 0xFFFFFFFF))
		return -EINVAL;
	if (unlikely(vaddr < ipc_mem->vaddr ||
		     vaddr >= (ipc_mem->vaddr + ipc_mem->size)))
		return -EINVAL;
	*vpu_addr = ipc_mem->dma_handle + (vaddr - ipc_mem->vaddr);

	return 0;
}

/**
 * process_rx_fifo_entry() - Process a FIFO entry.
 * @entry:	The FIFO entry to process.
 * @ipc_dev:	The IPC device to use.
 *
 * This function performs the following tasks:
 * - Check the source node id.
 * - Process the IPC buffer (locate it, validate it, read data info, release
 *   buffer).
 * - Add an RX Data descriptor (data ptr and data size) to the channel RX queue.
 */
static void process_rx_fifo_entry(u32 entry,
				  struct keembay_ipc_dev *ipc_dev)
{
	struct device *dev = &ipc_dev->plat_dev->dev;
	struct ipc_link *link = &ipc_dev->leon_mss_link;
	struct kmb_ipc_buf *ipc_buf;
	struct rx_data *rx_data;
	struct ipc_chan *chan;
	int idx;
	unsigned long flags;

	dev_dbg(dev, "RX: Processing entry: %x\n", entry);
	/* Get IPC buffer. */
	ipc_buf = ipc_vpu_to_virt(&ipc_dev->remote_ipc_mem, entry);
	if (unlikely(!ipc_buf)) {
		dev_warn(dev, "RX: Message out of expected memory range: %x\n",
			 entry);
		/* Return immediately (cannot mark the IPC buffer as free). */
		return;
	}
	if (unlikely(ipc_buf->src_node != KMB_IPC_NODE_LEON_MSS)) {
		dev_warn(dev, "RX: Message from unexpected source: %d\n",
			 ipc_buf->src_node);
		goto exit;
	}
	/* Check destination node. */
	if (unlikely(ipc_buf->dst_node != MY_NODE_ID)) {
		dev_warn(dev, "RX: Message is not for this node\n");
		goto exit;
	}
	/* Preliminary channel check. */
	if (unlikely(ipc_buf->channel >= KMB_IPC_MAX_CHANNELS)) {
		dev_warn(dev, "RX: Message for invalid channel\n");
		goto exit;
	}

	/* Access internal channel struct (this is protected by an SRCU). */
	idx = srcu_read_lock(&link->srcu_sp[ipc_buf->channel]);
	chan = srcu_dereference(link->channels[ipc_buf->channel],
				&link->srcu_sp[ipc_buf->channel]);
	if (unlikely(!chan)) {
		srcu_read_unlock(&link->srcu_sp[ipc_buf->channel], idx);
		dev_warn(dev, "RX: Message for closed channel.\n");
		goto exit;
	}
	rx_data = kmalloc(sizeof(*rx_data), GFP_ATOMIC);
	if (unlikely(!rx_data)) {
		/* If kmalloc fails, we are forced to discard the message. */
		srcu_read_unlock(&link->srcu_sp[ipc_buf->channel], idx);
		dev_err(dev, "RX: Message dropped: Cannot allocate RX Data.\n");
		goto exit;
	}
	/* Read data info. */
	rx_data->data_vpu_addr = ipc_buf->data_addr;
	rx_data->data_size = ipc_buf->data_size;
	/* Put data info in rx channel queue. */
	spin_lock_irqsave(&chan->rx_lock, flags);
	list_add_tail(&rx_data->list, &chan->rx_data_list);
	spin_unlock_irqrestore(&chan->rx_lock, flags);
	/* Wake up threads waiting on recv(). */
	wake_up_interruptible(&chan->rx_wait_queue);
	/* Exit SRCU region protecting chan struct. */
	srcu_read_unlock(&link->srcu_sp[ipc_buf->channel], idx);

exit:
	barrier(); /* Ensure IPC buffer is fully processed before release. */
	ipc_buf->status = KMB_IPC_BUF_FREE;
}

/*
 * The function implementing the RX tasklet.
 *
 * Go through the RX SW FIFO (filled by the RX ISR) and process every entry.
 */
static void rx_tasklet_func(unsigned long ipc_dev_ptr)
{
	u32 entry;
	struct keembay_ipc_dev *ipc_dev = (struct keembay_ipc_dev *)ipc_dev_ptr;
	struct rx_circ_buf *rx_sw_fifo = &ipc_dev->rx_sw_fifo;
	/*
	 * Memory barrier: make sure that we get buffer head before any other
	 * operation on the circular buffer.
	 */
	const unsigned int head = smp_load_acquire(&rx_sw_fifo->head);
	unsigned long tail = rx_sw_fifo->tail;
	const size_t size = ARRAY_SIZE(rx_sw_fifo->buf);

	while (CIRC_CNT(head, tail, size)) {
		/* Extract one item from the buffer. */
		entry = rx_sw_fifo->buf[tail];
		/* Consume the item. */
		process_rx_fifo_entry(entry, ipc_dev);
		/* Update tail index (wrapping it if needed) */
		tail = (tail + 1) & (size - 1);
	}
	/* Memory barrier ensuring tail is updated only at the end. */
	smp_store_release(&rx_sw_fifo->tail, tail);
	/* Enable Local FIFO interrupt. */
	local_fifo_irq_enable(ipc_dev);
}

/*
 * Local FIFO ISR.
 *
 * The HW FIFO produces interrupts when not full. This ISR is called to handle
 * such interrupts.
 *
 * The ISR moves FIFO entries from the HW FIFO to an internal SW FIFO
 * (implemented as a circular buffer) and then activates a tasklet to handle
 * the entries in the SW FIFO.
 *
 * The ISR tries to empty the HW FIFO, thus making it stop generating
 * interrupts. If it is not possible to empty the HW FIFO because the SW FIFO
 * is full, the ISR disables the HW FIFO interrupts, thus preventing ISR
 * re-activation and giving time to the tasklet to consume entries in the SW
 * FIFO. When the tasklet it is done, it re-enables the interrupts.
 */
static irqreturn_t local_fifo_irq_handler(int irq, void *ptr)
{
	struct keembay_ipc_dev *ipc_dev = ptr;
	struct rx_circ_buf *rx_sw_fifo = &ipc_dev->rx_sw_fifo;
	unsigned int head = rx_sw_fifo->head;
	const unsigned int tail = READ_ONCE(rx_sw_fifo->tail);
	const size_t size = ARRAY_SIZE(rx_sw_fifo->buf);

	/* Copy entries to internal SW FIFO. */
	while (likely(local_fifo_cnt(ipc_dev))) {
		/* If SW FIFO is full, disable HW FIFO interrupt. */
		if (unlikely(!CIRC_SPACE(head, tail, size))) {
			local_fifo_irq_disable(ipc_dev);
			goto exit;
		}
		rx_sw_fifo->buf[head] = local_fifo_get(ipc_dev);

		/* Update head index (wrapping it if needed). */
		head = (head + 1) & (size - 1);
	}

exit:
	/*
	 * Memory barrier: make sure that updating the buffer head is the last
	 * operation done on the circular buffer.
	 */
	smp_store_release(&rx_sw_fifo->head, head);
	tasklet_schedule(&ipc_dev->rx_tasklet);
	return IRQ_HANDLED;
}

/**
 * tx_data_send() - Send a TX data element.
 * @ipc_dev:	The IPC device to use.
 * @tx_data:	The TX data element to send.
 */
static int tx_data_send(struct keembay_ipc_dev *ipc_dev,
			struct tx_data *tx_data)
{
	struct kmb_ipc_buf *ipc_buf = NULL;
	struct device *dev = &ipc_dev->plat_dev->dev;
	u32 entry;
	int rc;

	dev_dbg(dev, "%s(chan=%u)\n", __func__, tx_data->chan_id);
	/* Allocate and set IPC buffer. */
	ipc_buf = ipc_buf_tx_alloc(&ipc_dev->ipc_buf_pool);
	if (unlikely(!ipc_buf)) {
		rc = -ENOMEM;
		goto exit;
	}

	/* Prepare IPC buffer. */
	ipc_buf->channel = tx_data->chan_id;
	ipc_buf->src_node = MY_NODE_ID;
	ipc_buf->dst_node = tx_data->dst_node;
	ipc_buf->data_addr = tx_data->vpu_addr;
	ipc_buf->data_size = tx_data->size;
	/* Ensure changes to IPC Buffer are performed before entry is sent. */
	wmb();

	/* Initialize entry to ipc_buf VPU address. */
	rc = ipc_virt_to_vpu(&ipc_dev->local_ipc_mem, ipc_buf, &entry);

	/*
	 * Check validity of IPC buffer VPU address (this error should never
	 * occur if IPC buffer region is defined properly in Device Tree).
	 */
	if (unlikely(rc)) {
		dev_err(dev, "Cannot convert IPC buf vaddr to vpu_addr: %p\n",
			ipc_buf);
		rc = -ENXIO;
		goto exit;
	}
	if (unlikely(!IS_ALIGNED(entry, KMB_IPC_ALIGNMENT))) {
		dev_err(dev, "Allocated IPC buf is not 64-byte aligned: %p\n",
			ipc_buf);
		rc = -EFAULT;
		goto exit;
	}

	/* Set Processor ID in entry. */
	entry |= (MY_NODE_ID & 0x3F);
	dev_dbg(dev, "TX IPC FIFO entry: %x\n", entry);

	/* Send entry and check for overflow. */
	remote_fifo_put(ipc_dev, entry);
	if (remote_fifo_overflow(ipc_dev)) {
		rc = -EBUSY;
		goto exit;
	}

exit:
	/* If an error occurred and a buffer was allocated, free it. */
	if (rc && ipc_buf)
		ipc_buf->status = KMB_IPC_BUF_FREE;
	return rc;
}

/**
 * tx_data_dequeue() - Dequeue the next TX data waiting for transfer.
 * @link:  The link from which we dequeue the TX Data.
 *
 * The de-queue policy is round robin between each high speed channel queue and
 * the one queue for all low speed channels.
 *
 * Return: The next TX data waiting to be transferred, or NULL if no TX is
 *	   pending.
 */
static struct tx_data *tx_data_dequeue(struct ipc_link *link)
{
	struct tx_queue *queue;
	struct tx_data *tx_data;
	int i;
	unsigned long flags;

	/*
	 * TX queues are logically organized in a circular array.
	 * We go through such an array until we find a non-empty queue.
	 * We start from where we left since last function invocation.
	 * If all queues are empty we return NULL.
	 */
	for (i = 0; i < ARRAY_SIZE(link->tx_queues); i++) {
		queue = &link->tx_queues[link->tx_qidx];
		link->tx_qidx++;
		if (link->tx_qidx == ARRAY_SIZE(link->tx_queues))
			link->tx_qidx = 0;
		if (!list_empty(&queue->tx_data_list)) {
			spin_lock_irqsave(&queue->lock, flags);
			tx_data = list_first_entry_or_null(
					&queue->tx_data_list,
					struct tx_data, list);
			/*
			 * There is a remote chance that tx_data is NULL; this
			 * can happen when tx_data_remove() is called between
			 * our list_empty() check and our list_first_entry()
			 * call. If tx_data is null, we simply continue.
			 */
			if (!tx_data) {
				spin_unlock_irqrestore(&queue->lock, flags);
				continue;
			}
			list_del(&tx_data->list);
			spin_unlock_irqrestore(&queue->lock, flags);
			return tx_data;
		}
	}

	return NULL;
}

/**
 * tx_data_enqueue() - Enqueue TX data for transfer into the specified link.
 * @link:	The link the data is enqueued to.
 * @tx_data:	The TX data to enqueue.
 */
static void tx_data_enqueue(struct ipc_link *link, struct tx_data *tx_data)
{
	struct tx_queue *queue;
	int qid;
	unsigned long flags;

	/*
	 * Find the right queue where to put TX data:
	 * - Each high-speed channel has a dedicated queue, whose index is the
	 *   same as the channel id (e.g., Channel 1 uses tx_queues[1]).
	 * - All the general-purpose channels use the same TX queue, which is
	 *   the last element in the tx_queues array.
	 *
	 * Note: tx_queues[] has KMB_IPC_NUM_HS_CHANNELS+1 elements)
	 */
	qid = tx_data->chan_id < ARRAY_SIZE(link->tx_queues) ?
	      tx_data->chan_id : (ARRAY_SIZE(link->tx_queues) - 1);

	queue = &link->tx_queues[qid];

	spin_lock_irqsave(&queue->lock, flags);
	list_add_tail(&tx_data->list, &queue->tx_data_list);
	spin_unlock_irqrestore(&queue->lock, flags);
}

/**
 * tx_data_remove() - Remove TX data element from specified link.
 * @link:	The link the data is currently enqueued to.
 * @tx_data:	The TX data element to be removed.
 *
 * This function is called by the main send function, when the send is
 * interrupted or has timed out.
 */
static void tx_data_remove(struct ipc_link *link, struct tx_data *tx_data)
{
	struct tx_queue *queue;
	int qid;
	unsigned long flags;

	/*
	 * Find the TX queue where TX data is currently located:
	 * - Each high-speed channel has a dedicated queue, whose index is the
	 *   same as the channel id (e.g., Channel 1 uses tx_queues[1]).
	 * - All the general-purpose channels use the same TX queue, which is
	 *   the last element in the tx_queues array.
	 *
	 * Note: tx_queues[] has KMB_IPC_NUM_HS_CHANNELS+1 elements)
	 */
	qid = tx_data->chan_id < ARRAY_SIZE(link->tx_queues) ?
	      tx_data->chan_id : (ARRAY_SIZE(link->tx_queues) - 1);

	queue = &link->tx_queues[qid];

	spin_lock_irqsave(&queue->lock, flags);
	list_del(&tx_data->list);
	spin_unlock_irqrestore(&queue->lock, flags);
}

/**
 * tx_thread_fn() - The function run by the TX thread.
 * @ptr: A pointer to the keembay_ipc_dev struct associated with the thread.
 *
 * This thread continuously dequeues and send TX data elements. The TX
 * semaphore is used to pause the loop when all the pending TX data elements
 * have been transmitted (the send function 'ups' the semaphore every time a
 * new TX data element is enqueued).
 */
static int tx_thread_fn(void *ptr)
{
	struct keembay_ipc_dev *ipc_dev = ptr;
	struct ipc_link *link = &ipc_dev->leon_mss_link;
	struct tx_data *tx_data;
	int rc;

	while (1) {
		rc = wait_for_completion_interruptible(&link->tx_queued);
		if (rc || link->tx_stopping)
			break;
		tx_data = tx_data_dequeue(link);
		/*
		 * We can get a null tx_data if tx_data_remove() has been
		 * called. Just ignore it and continue.
		 */
		if (!tx_data)
			continue;
		tx_data->retv = tx_data_send(ipc_dev, tx_data);
		complete(&tx_data->tx_done);
	}
	/* Wait until kthread_stop() is called. */
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);

	return rc;
}

/* Internal send. */
static int __ipc_send(struct keembay_ipc_dev *ipc_dev, u8 dst_node,
		      u16 chan_id, u32 vpu_addr, size_t size)
{
	struct ipc_link *link = &ipc_dev->leon_mss_link;
	struct tx_data *tx_data;
	int rc;

	/* Allocate and init TX data. */
	tx_data = kmalloc(sizeof(*tx_data), GFP_KERNEL);
	if (!tx_data)
		return -ENOMEM;
	tx_data->dst_node = dst_node;
	tx_data->chan_id = chan_id;
	tx_data->vpu_addr = vpu_addr;
	tx_data->size = size;
	tx_data->retv = 1;
	INIT_LIST_HEAD(&tx_data->list);
	init_completion(&tx_data->tx_done);
	/* Add tx_data to tx queues. */
	tx_data_enqueue(link, tx_data);
	/* Signal that we have a new pending TX. */
	complete(&link->tx_queued);
	/* Wait until data is transmitted. */
	rc = wait_for_completion_interruptible(&tx_data->tx_done);
	if (unlikely(rc)) {
		tx_data_remove(link, tx_data);
		goto exit;
	}
	rc = tx_data->retv;

exit:
	kfree(tx_data);
	return rc;
}

/*
 * Driver allocation.
 */

/* Device tree driver match. */
static const struct of_device_id kmb_ipc_of_match[] = {
	{
		.compatible = "intel,keembay-ipc",
	},
	{}
};

/* The IPC driver is a platform device. */
static struct platform_driver kmb_ipc_driver = {
	.probe  = kmb_ipc_probe,
	.remove = kmb_ipc_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = kmb_ipc_of_match,
	},
};

module_platform_driver(kmb_ipc_driver);

/*
 * Perform basic validity check on common API arguments.
 *
 * Verify that the specified device is a Keem Bay IPC device and that node ID
 * and the channel ID are within the allowed ranges.
 */
static int validate_api_args(struct device *dev, u8 node_id, u16 chan_id)
{
	if (!dev || dev->driver != &kmb_ipc_driver.driver)
		return -EINVAL;
	if (node_id != KMB_IPC_NODE_LEON_MSS) {
		dev_warn(dev, "Invalid Link ID\n");
		return -EINVAL;
	}
	if (chan_id >= KMB_IPC_MAX_CHANNELS) {
		dev_warn(dev, "Invalid Channel ID\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * IPC Kernel API.
 */

/**
 * intel_keembay_ipc_open_channel() - Open an IPC channel.
 * @dev:	The IPC device to use.
 * @node_id:	The node ID of the remote node (used to identify the link the
 *		channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *		allowed value for now.
 * @chan_id:	The ID of the channel to be opened.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int intel_keembay_ipc_open_channel(struct device *dev, u8 node_id, u16 chan_id)
{
	struct keembay_ipc_dev *ipc_dev;
	struct ipc_link *link;
	struct ipc_chan *chan, *cur_chan;
	unsigned long flags;
	int rc;

	rc = validate_api_args(dev, node_id, chan_id);
	if (rc)
		return rc;

	ipc_dev = dev_get_drvdata(dev);
	link = &ipc_dev->leon_mss_link;

	/* Create channel before getting lock. */
	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	INIT_LIST_HEAD(&chan->rx_data_list);
	spin_lock_init(&chan->rx_lock);
	init_waitqueue_head(&chan->rx_wait_queue);

	/* Add channel to the channel array (if not already present). */
	spin_lock_irqsave(&link->chan_lock, flags);
	cur_chan = rcu_dereference_protected(link->channels[chan_id],
					     lockdep_is_held(&link->chan_lock));
	if (cur_chan) {
		spin_unlock_irqrestore(&link->chan_lock, flags);
		kfree(chan);
		return -EEXIST;
	}
	rcu_assign_pointer(link->channels[chan_id], chan);
	spin_unlock_irqrestore(&link->chan_lock, flags);

	return 0;
}
EXPORT_SYMBOL(intel_keembay_ipc_open_channel);

/**
 * intel_keembay_ipc_close_channel() - Close an IPC channel.
 * @dev:	The IPC device to use.
 * @node_id:	The node ID of the remote node (used to identify the link the
 *		channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *		allowed value for now.
 * @chan_id:	The ID of the channel to be closed.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int intel_keembay_ipc_close_channel(struct device *dev, u8 node_id, u16 chan_id)
{
	struct keembay_ipc_dev *ipc_dev;
	struct ipc_link *link;
	int rc;

	rc = validate_api_args(dev, node_id, chan_id);
	if (rc)
		return rc;

	ipc_dev = dev_get_drvdata(dev);
	link = &ipc_dev->leon_mss_link;

	rc = channel_close(link, chan_id);
	if (!rc)
		dev_info(dev, "Channel was already closed\n");

	return 0;
}
EXPORT_SYMBOL(intel_keembay_ipc_close_channel);

/**
 * intel_keembay_ipc_send() - Send data via IPC.
 * @dev:	The IPC device to use.
 * @node_id:	The node ID of the remote node (used to identify the link the
 *		channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *		allowed value for now.
 * @chan_id:	The IPC channel to be used to send the message.
 * @vpu_addr:	The VPU address of the data to be transferred.
 * @size:	The size of the data to be transferred.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int intel_keembay_ipc_send(struct device *dev, u8 node_id, u16 chan_id,
			   u32 vpu_addr, size_t size)
{
	struct keembay_ipc_dev *ipc_dev;
	struct ipc_link *link;
	struct ipc_chan *chan;
	int idx, rc;

	rc = validate_api_args(dev, node_id, chan_id);
	if (rc)
		return rc;

	ipc_dev = dev_get_drvdata(dev);
	link = &ipc_dev->leon_mss_link;
	/*
	 * Start Sleepable RCU critical section (this prevents close() from
	 * destroying the channels struct while we are sending data)
	 */
	idx = srcu_read_lock(&link->srcu_sp[chan_id]);
	/* Get channel. */
	chan = srcu_dereference(link->channels[chan_id],
				&link->srcu_sp[chan_id]);
	if (unlikely(!chan)) {
		/* The channel is closed. */
		rc = -ENOENT;
		goto exit;
	}

	rc = __ipc_send(ipc_dev, node_id, chan_id, vpu_addr, size);

exit:
	/* End sleepable RCU critical section. */
	srcu_read_unlock(&link->srcu_sp[chan_id], idx);
	return rc;
}
EXPORT_SYMBOL(intel_keembay_ipc_send);

/**
 * intel_keembay_ipc_recv() - Read data via IPC
 * @dev:	The IPC device to use.
 * @node_id:	The node ID of the remote node (used to identify the link the
 *		channel must be added to). KMB_IPC_NODE_LEON_MSS is the only
 *		allowed value for now.
 * @chan_id:	The IPC channel to read from.
 * @vpu_addr:	[out] The VPU address of the received data.
 * @size:	[out] Where to store the size of the received data.
 * @timeout:	How long (in ms) the function will block waiting for an IPC
 *		message; if UINT32_MAX it will block indefinitely; if 0 it
 *		will not block.
 *
 * Return:	0 on success, negative error code otherwise
 */
int intel_keembay_ipc_recv(struct device *dev, u8 node_id, u16 chan_id,
			   u32 *vpu_addr, size_t *size, u32 timeout)
{
	struct keembay_ipc_dev *ipc_dev;
	struct ipc_link *link;
	struct ipc_chan *chan;
	struct rx_data *rx_entry;
	unsigned long flags;
	int idx, rc;

	rc = validate_api_args(dev, node_id, chan_id);
	if (rc)
		return rc;

	if (!vpu_addr || !size)
		return -EINVAL;

	ipc_dev = dev_get_drvdata(dev);
	link = &ipc_dev->leon_mss_link;

	/*
	 * Start Sleepable RCU critical section (this prevents close() from
	 * destroying the channels struct while we are using it)
	 */
	idx = srcu_read_lock(&link->srcu_sp[chan_id]);
	/* Get channel. */
	chan = srcu_dereference(link->channels[chan_id],
				&link->srcu_sp[chan_id]);
	if (unlikely(!chan)) {
		rc = -ENOENT;
		goto err;
	}
	/*
	 * Get the lock protecting rx_data_list; the lock will be released by
	 * wait_event_*_lock_irq() before going to sleep and automatically
	 * reacquired after wake up.
	 */
	spin_lock_irqsave(&chan->rx_lock, flags);
	/*
	 * Wait for RX data.
	 *
	 * Note: wait_event_interruptible_lock_irq_timeout() has different
	 * return values than wait_event_interruptible_lock_irq().
	 *
	 * The following if/then branch ensures that return values are
	 * consistent for the both cases, that is:
	 * - rc == 0 only if the wait was successfully (i.e., we were notified
	 *   of a message or of a channel closure)
	 * - rc < 0 if an error occurred (we got interrupted or the timeout
	 *   expired).
	 */
	if (timeout == U32_MAX) {
		rc = wait_event_interruptible_lock_irq(
					chan->rx_wait_queue,
					!list_empty(&chan->rx_data_list) ||
					chan->closing,
					chan->rx_lock);
	} else {
		rc = wait_event_interruptible_lock_irq_timeout(
					chan->rx_wait_queue,
					!list_empty(&chan->rx_data_list) ||
					chan->closing,
					chan->rx_lock,
					msecs_to_jiffies(timeout));
		if (!rc)
			rc = -ETIME;
		if (rc > 0)
			rc = 0;
	}
	/* Check if the channel was closed while waiting. */
	if (chan->closing)
		rc = -EPIPE;
	if (rc) {
		spin_unlock_irqrestore(&chan->rx_lock, flags);
		goto err;
	}
	/* Extract RX entry. */
	rx_entry = list_first_entry(&chan->rx_data_list, struct rx_data, list);
	list_del(&rx_entry->list);
	spin_unlock_irqrestore(&chan->rx_lock, flags);
	/* Set output parameters. */
	*vpu_addr =  rx_entry->data_vpu_addr;
	*size = rx_entry->data_size;
	/* Free RX entry. */
	kfree(rx_entry);

err:
	/* End sleepable RCU critical section. */
	srcu_read_unlock(&link->srcu_sp[chan_id], idx);
	return rc;
}
EXPORT_SYMBOL(intel_keembay_ipc_recv);

MODULE_DESCRIPTION("Keem Bay IPC Driver");
MODULE_AUTHOR("Daniele Alessandrelli <daniele.alessandrelli@intel.com>");
MODULE_AUTHOR("Paul Murphy <paul.j.murphy@intel.com>");
MODULE_LICENSE("GPL v2");
