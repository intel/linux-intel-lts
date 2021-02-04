// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Dispatcher.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/sched/signal.h>
#include <linux/platform_device.h>

#include "xlink-dispatcher.h"
#include "xlink-multiplexer.h"
#include "xlink-platform.h"

#ifdef CONFIG_XLINK_LOCAL_HOST
#include <linux/xlink-ipc.h>
#endif

#define DISPATCHER_RX_TIMEOUT_MSEC 0

/* state of a dispatcher servicing a link to a device*/
enum dispatcher_state {
	XLINK_DISPATCHER_INIT,		// dispatcher has been initialized but not used
	XLINK_DISPATCHER_RUNNING,	// dispatcher is currently servicing a link
	XLINK_DISPATCHER_STOPPED,	// dispatcher is no longer servicing a link
	XLINK_DISPATCHER_ERROR,		// dispatcher fatal error
};

/* queue for dispatcher tx thread event handling */
struct event_queue {
	uint32_t count;			// number of events in the queue
	uint32_t capacity;		// capacity of events in the queue
	struct list_head head;	// head of event linked list
	struct mutex lock;		// locks queue while accessing
};

/* dispatcher servicing a single link to a device */
struct dispatcher {
	uint32_t link_id; 				// id of link being serviced
	enum dispatcher_state state;	// state of the dispatcher
	struct xlink_handle *handle;	// xlink device handle
	int interface;					// underlying interface of link
	struct task_struct *rxthread;	// kthread servicing rx
	struct task_struct *txthread;	// kthread servicing tx
	struct event_queue queue;		// xlink event queue
	struct semaphore event_sem;		// signals tx kthread of available events
	struct completion rx_done;		// synchronizes start/stop of rx kthread
	struct completion tx_done;		// synchornizes start/stop of tx thread
};

/* xlink dispatcher system component */
struct xlink_dispatcher {
	struct dispatcher dispatchers[XLINK_MAX_CONNECTIONS];	// dispatcher queue
	struct device *dev;	// deallocate data
	struct mutex lock;	// locks structre when starting new dispatcher
};

/* global reference to the xlink dispatcher data structure */
static struct xlink_dispatcher *xlinkd;

/* global reference to the xlink ipc dispatcher data structure */
#ifdef CONFIG_XLINK_LOCAL_HOST
static struct dispatcher ipc_disp;
#endif

/*
 * Dispatcher Internal Functions
 *
 */

static struct dispatcher *get_dispatcher_by_id(u32 id)
{
	if (!xlinkd)
		return NULL;

	if (id >= XLINK_MAX_CONNECTIONS)
		return NULL;

	return &xlinkd->dispatchers[id];
}

static uint32_t event_generate_id(void)
{
	static uint32_t id = 0xa; // TODO: temporary solution
	return id++;
}

static struct xlink_event *event_dequeue(struct event_queue *queue)
{
	struct xlink_event *event = NULL;
	mutex_lock(&queue->lock);
	if (!list_empty(&queue->head)) {
		event = list_first_entry(&queue->head, struct xlink_event, list);
		list_del(&event->list);
		queue->count--;
	}
	mutex_unlock(&queue->lock);
	return event;
}

// static int event_enqueue(struct event_queue *queue, struct xlink_event *event)
// {
// 	int rc = -1;
// 	mutex_lock(&queue->lock);
// 	if (queue->count < ((queue->capacity/10)*7)) {
// 		list_add_tail(&event->list, &queue->head);
// 		queue->count++;
// 		rc = 0;
// 	}
// 	mutex_unlock(&queue->lock);
// 	return rc;
// }

static struct xlink_event *dispatcher_event_get(struct dispatcher *disp)
{
	int rc = 0;
	struct xlink_event *event = NULL;

	// wait until an event is available
	rc = down_interruptible(&disp->event_sem);
	// dequeue and return next event to process
	if (!rc)
		event = event_dequeue(&disp->queue);
	return event;
}

static int is_valid_event_header(struct xlink_event *event)
{
	if (event->header.magic != XLINK_EVENT_HEADER_MAGIC)
		return 0;
	else
		return 1;
}

static int dispatcher_event_send(struct xlink_event *event)
{
	int rc = 0;
	static int error_printed;
	size_t event_header_size = sizeof(event->header);

	// write event header
	// printk(KERN_DEBUG "Sending event: type = 0x%x, id = 0x%x\n",
			// event->header.type, event->header.id);
	rc = xlink_platform_write(event->interface,
			event->handle->sw_device_id, &event->header,
			&event_header_size, event->header.timeout, NULL);
	if (rc || (event_header_size != sizeof(event->header))) {
		if (!error_printed)
			pr_err("Write header failed %d\n", rc);
		error_printed = 1;
		return rc;
	}
	if ((event->header.type == XLINK_WRITE_REQ) ||
		(event->header.type == XLINK_WRITE_VOLATILE_REQ) ||
		(event->header.type == XLINK_PASSTHRU_VOLATILE_WRITE_REQ) ||
		(event->header.type == XLINK_PASSTHRU_WRITE_REQ)) {
		error_printed = 0;
		// write event data
		rc = xlink_platform_write(event->interface,
				event->handle->sw_device_id, event->data,
				&event->header.size, event->header.timeout,
				NULL);
		if (rc)
			pr_err("Write data failed %d\n", rc);
		if (event->user_data == 1) {
			if (event->paddr != 0) {
				xlink_platform_deallocate(xlinkd->dev,
					event->data, event->paddr,
					event->header.size,
					XLINK_PACKET_ALIGNMENT,
					XLINK_CMA_MEMORY);
			} else {
				xlink_platform_deallocate(xlinkd->dev,
					event->data, event->paddr,
					event->header.size,
					XLINK_PACKET_ALIGNMENT,
					XLINK_NORMAL_MEMORY);
			}
		}
	}
	return rc;
}

static int xlink_dispatcher_rxthread(void *context)
{
	int rc = 0;
	size_t size = 0;
	struct xlink_event *event = NULL;
	struct dispatcher *disp = (struct dispatcher *)context;

	// printk(KERN_DEBUG "dispatcher rxthread started\n");
	event = xlink_create_event(disp->link_id, 0, disp->handle, 0, 0, 0);
	if (!event)
		return -1;

	allow_signal(SIGTERM); // allow thread termination while waiting on sem
	complete(&disp->rx_done);
	while (!kthread_should_stop()) {
		size = sizeof(event->header);
		rc = xlink_platform_read(disp->interface,
				disp->handle->sw_device_id, &event->header, &size,
				DISPATCHER_RX_TIMEOUT_MSEC, NULL);
		if (rc || (size != (int)sizeof(event->header))) {
			continue;
		}
		if (is_valid_event_header(event)) {
			// printk(KERN_DEBUG "Incoming event: type = 0x%x, id = 0x%x\n",
					// event->header.type, event->header.id);
			event->link_id = disp->link_id;
			rc = xlink_multiplexer_rx(event);
			if (!rc) {
				event = xlink_create_event(disp->link_id, 0, disp->handle, 0,
						0, 0);
				if (!event)
					return -1;
			}
		}
	}
	// printk(KERN_INFO "dispatcher rxthread stopped\n");
	complete(&disp->rx_done);
	do_exit(0);
	return 0;
}

static int xlink_dispatcher_txthread(void *context)
{
	struct xlink_event *event = NULL;
	struct dispatcher *disp = (struct dispatcher *)context;

	// printk(KERN_DEBUG "dispatcher txthread started\n");
	allow_signal(SIGTERM); // allow thread termination while waiting on sem
	complete(&disp->tx_done);
	while (!kthread_should_stop()) {
		event = dispatcher_event_get(disp);
		if (!event)
			continue;

		dispatcher_event_send(event);
		xlink_destroy_event(event); // event is handled and can now be freed
	}
	// printk(KERN_INFO "dispatcher txthread stopped\n");
	complete(&disp->tx_done);
	do_exit(0);
	return 0;
}

#ifdef CONFIG_XLINK_LOCAL_HOST
static struct xlink_event *dispatcher_ipc_passthru_event_get(void)
{
	int rc = 0;
	struct xlink_event *event = NULL;

	// wait until an event is available
	rc = down_interruptible(&ipc_disp.event_sem);
	if (!rc) {
		// dequeue and return next event to process
		event = event_dequeue(&ipc_disp.queue);
	}
	return event;
}

static int xlink_dispatcher_ipc_passthru_rxthread(void *context)
{
	enum xlink_error rc = X_LINK_SUCCESS;
	struct xlink_event *event = NULL,  *resp_event = NULL;
	size_t size = 0;
	uint32_t message;
	struct xlink_ipc_context ipc = {0};
	void *data;

	allow_signal(SIGTERM); // allow thread termination while waiting on sem
	complete(&ipc_disp.rx_done);
	while (!kthread_should_stop()) {
		event = dispatcher_ipc_passthru_event_get();

		ipc.chan = event->header.chan;
		size = sizeof(event->header);

		if (event->header.type == XLINK_PASSTHRU_READ_TO_BUFFER_REQ) {
			ipc.is_volatile = 1;
			data = kzalloc(XLINK_MAX_BUF_SIZE, GFP_KERNEL);
			rc = xlink_platform_read(IPC_INTERFACE,
					event->handle->sw_device_id, data, &size,
					DISPATCHER_RX_TIMEOUT_MSEC, &ipc);
			if (!rc) {
				resp_event = xlink_create_event(ipc_disp.link_id,
						XLINK_WRITE_REQ, event->handle,	event->header.chan,
						size, event->header.timeout);
				resp_event->data = data;
				xlink_dispatcher_event_add(EVENT_RX, resp_event);
			} else
				kfree(data);
		} else {
			ipc.is_volatile = 0;
			rc = xlink_platform_read(IPC_INTERFACE,
					event->handle->sw_device_id, &message, &size,
					DISPATCHER_RX_TIMEOUT_MSEC, &ipc);
			if (!rc) {
				if (event->header.type == XLINK_PASSTHRU_READ_REQ) {
					resp_event = xlink_create_event(ipc_disp.link_id,
							XLINK_WRITE_REQ, event->handle, event->header.chan,
							size, event->header.timeout);
					resp_event->data = find_allocated_buffer(message);
					if (resp_event->data == NULL) {
						xlink_destroy_event(resp_event); // event is handled and can now be freed
					} else {
						resp_event->paddr = message;
						unregister_allocated_buffer(resp_event->data, resp_event->paddr);
						xlink_dispatcher_event_add(EVENT_RX, resp_event);
					}
				}
			}
		}
		if (rc) {
			// no data on channel - just re add event to back of queue
			rc = xlink_dispatcher_ipc_passthru_event_add(event);
			if (rc) {
				return X_LINK_ERROR;
			}
			continue;
		}
	}
	complete(&ipc_disp.rx_done);
	do_exit(0);
	return 0;
}

enum xlink_error xlink_dispatcher_ipc_passthru_event_add(struct xlink_event *event)
{
	int rc = 0;

	// only add events if the dispatcher is running
	if (ipc_disp.state != XLINK_DISPATCHER_RUNNING) {
		return X_LINK_ERROR;
	}
	rc = event_enqueue(&ipc_disp.queue, event);
	if (rc) {
		return X_LINK_ERROR;
	}
	// notify dispatcher tx thread of new event
	up(&ipc_disp.event_sem);
	return X_LINK_SUCCESS;
}
#else
enum xlink_error xlink_dispatcher_ipc_passthru_event_add(struct xlink_event *event)
{
	return X_LINK_SUCCESS;
}
#endif

/*
 * Dispatcher External Functions
 *
 */

enum xlink_error xlink_dispatcher_init(void *dev)
{
	int i = 0;
	struct platform_device *plat_dev = (struct platform_device *) dev;

	xlinkd = kzalloc(sizeof(*xlinkd), GFP_KERNEL);
	if (!xlinkd)
		return X_LINK_ERROR;

	xlinkd->dev = &plat_dev->dev;
	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		xlinkd->dispatchers[i].link_id = i;
		sema_init(&xlinkd->dispatchers[i].event_sem, 0);
		init_completion(&xlinkd->dispatchers[i].rx_done);
		init_completion(&xlinkd->dispatchers[i].tx_done);
		INIT_LIST_HEAD(&xlinkd->dispatchers[i].queue.head);
		mutex_init(&xlinkd->dispatchers[i].queue.lock);
		xlinkd->dispatchers[i].queue.count = 0;
		xlinkd->dispatchers[i].queue.capacity = XLINK_EVENT_QUEUE_CAPACITY;
		xlinkd->dispatchers[i].state = XLINK_DISPATCHER_INIT;
	}
	mutex_init(&xlinkd->lock);

#ifdef CONFIG_XLINK_LOCAL_HOST
	init_completion(&ipc_disp.rx_done);
	sema_init(&ipc_disp.event_sem, 0);
	ipc_disp.state = XLINK_DISPATCHER_INIT;
	INIT_LIST_HEAD(&ipc_disp.queue.head);
	mutex_init(&ipc_disp.queue.lock);
	ipc_disp.queue.count = 0;
	ipc_disp.queue.capacity = XLINK_EVENT_QUEUE_CAPACITY;
	ipc_disp.state = XLINK_DISPATCHER_INIT;
#endif
	return X_LINK_SUCCESS;
}

enum xlink_error xlink_dispatcher_start(int id, struct xlink_handle *handle)
{
	struct dispatcher *disp = NULL;

	mutex_lock(&xlinkd->lock);
	// get dispatcher by link id
	disp = get_dispatcher_by_id(id);
	if (!disp)
		goto r_error;

	// cannot start a running or failed dispatcher
	if ((disp->state == XLINK_DISPATCHER_RUNNING) ||
			(disp->state == XLINK_DISPATCHER_ERROR))
		goto r_error;

	// set the dispatcher context
	disp->handle = handle;
	disp->interface = get_interface_from_sw_device_id(handle->sw_device_id);

	// run dispatcher thread to handle and write outgoing packets
	disp->txthread = kthread_run(xlink_dispatcher_txthread,
			(void *)disp, "txthread");
	if (!disp->txthread) {
		printk(KERN_ERR "xlink txthread creation failed\n");
		goto r_txthread;
	}
	wait_for_completion(&disp->tx_done);
	disp->state = XLINK_DISPATCHER_RUNNING;
	// run dispatcher thread to read and handle incoming packets
	disp->rxthread = kthread_run(xlink_dispatcher_rxthread,
			(void *)disp, "rxthread");
	if (!disp->rxthread) {
		printk(KERN_ERR "xlink rxthread creation failed\n");
		goto r_rxthread;
	}
	wait_for_completion(&disp->rx_done);
	mutex_unlock(&xlinkd->lock);

#ifdef CONFIG_XLINK_LOCAL_HOST
	if (ipc_disp.state != XLINK_DISPATCHER_RUNNING) {
		// create dispatcher thread to read and handle incoming packets TODD sak better comment
		ipc_disp.link_id = disp->link_id;
		ipc_disp.rxthread = kthread_run(xlink_dispatcher_ipc_passthru_rxthread,
				(void *)&ipc_disp, "ipcthread");
		if (!ipc_disp.rxthread) {
			printk(KERN_ERR "ipb blk read thread creation failed\n");
			goto r_rxthread;
		}
		ipc_disp.state = XLINK_DISPATCHER_RUNNING;
		wait_for_completion(&ipc_disp.rx_done);
	}
#endif
	return X_LINK_SUCCESS;

r_rxthread:
	kthread_stop(disp->txthread);
r_txthread:
	disp->state = XLINK_DISPATCHER_STOPPED;
r_error:
	mutex_unlock(&xlinkd->lock);
	return X_LINK_ERROR;
}

enum xlink_error xlink_dispatcher_event_add(enum xlink_event_origin origin,
		struct xlink_event *event)
{
	int rc = 0;
	struct dispatcher *disp = NULL;

	// get dispatcher by handle
	disp = get_dispatcher_by_id(event->link_id);
	if (!disp)
		return X_LINK_ERROR;

	// only add events if the dispatcher is running
	if (disp->state != XLINK_DISPATCHER_RUNNING)
		return X_LINK_ERROR;

	// configure event and add to queue
	if (origin == EVENT_TX)
		event->header.id = event_generate_id();
	event->origin = origin;
	rc = event_enqueue(&disp->queue, event);
	if (rc)
		return X_LINK_CHAN_FULL;

	// notify dispatcher tx thread of new event
	up(&disp->event_sem);
	return X_LINK_SUCCESS;
}

enum xlink_error xlink_dispatcher_stop(int id)
{
	int rc = 0;
	struct dispatcher *disp = NULL;

	mutex_lock(&xlinkd->lock);
	// get dispatcher by link id
	disp = get_dispatcher_by_id(id);
	if (!disp)
		goto r_error;

	// don't stop dispatcher if not started
	if (disp->state != XLINK_DISPATCHER_RUNNING)
		goto r_error;

	if (disp->rxthread) {
		// stop dispatcher rx thread reading and handling incoming packets
		send_sig(SIGTERM, disp->rxthread, 0);
		rc = kthread_stop(disp->rxthread);
		if (rc)
			goto r_thread;
	}
	wait_for_completion(&disp->rx_done);
	if (disp->txthread) {
		// stop dispatcher tx thread handling and writing outgoing packets
		send_sig(SIGTERM, disp->txthread, 0);
		rc = kthread_stop(disp->txthread);
		if (rc)
			goto r_thread;
	}
	wait_for_completion(&disp->tx_done);
	disp->state = XLINK_DISPATCHER_STOPPED;
	mutex_unlock(&xlinkd->lock);
	return X_LINK_SUCCESS;

r_thread:
	// dispatcher now in error state and cannot be used
	disp->state = XLINK_DISPATCHER_ERROR;
r_error:
	mutex_unlock(&xlinkd->lock);
	return X_LINK_ERROR;
}

enum xlink_error xlink_dispatcher_destroy(void)
{
	int i = 0;
	struct dispatcher *disp = NULL;
	struct xlink_event *event = NULL;

	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		// get dispatcher by link id
		disp = get_dispatcher_by_id(i);
		if (!disp)
			continue;

		// stop all running dispatchers
		if (disp->state == XLINK_DISPATCHER_RUNNING)
			xlink_dispatcher_stop(i);

		// empty queues of all used dispatchers
		if (disp->state != XLINK_DISPATCHER_INIT) {
			// deallocate remaining events in queue
			while (!list_empty(&disp->queue.head)) {
				event = event_dequeue(&disp->queue);
				if (event) {
					if ((event->header.type == XLINK_WRITE_REQ) ||
							(event->header.type == XLINK_WRITE_VOLATILE_REQ)) {
						// free buffer allocated for event data
						xlink_platform_deallocate(xlinkd->dev, event->data,
								event->paddr, event->header.size,
								XLINK_PACKET_ALIGNMENT, XLINK_NORMAL_MEMORY);
					}
					xlink_destroy_event(event);
				}
			}
			// destroy dispatcher
			mutex_destroy(&disp->queue.lock);
		}
	}
	mutex_destroy(&xlinkd->lock);
	return X_LINK_SUCCESS;
}
