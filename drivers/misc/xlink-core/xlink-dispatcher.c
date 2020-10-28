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

#define DISPATCHER_RX_TIMEOUT_MSEC 0

/* state of a dispatcher servicing a link to a device*/
enum dispatcher_state {
	XLINK_DISPATCHER_INIT,		/* initialized but not used */
	XLINK_DISPATCHER_RUNNING,	/* currently servicing a link */
	XLINK_DISPATCHER_STOPPED,	/* no longer servicing a link */
	XLINK_DISPATCHER_ERROR,		/* fatal error */
};

/* queue for dispatcher tx thread event handling */
struct event_queue {
	u32 count;		/* number of events in the queue */
	u32 capacity;		/* capacity of events in the queue */
	struct list_head head;	/* head of event linked list */
	struct mutex lock;	/* locks queue while accessing */
};

/* dispatcher servicing a single link to a device */
struct dispatcher {
	u32 link_id;			/* id of link being serviced */
	enum dispatcher_state state;	/* state of the dispatcher */
	struct xlink_handle *handle;	/* xlink device handle */
	int interface;			/* underlying interface of link */
	struct task_struct *rxthread;	/* kthread servicing rx */
	struct task_struct *txthread;	/* kthread servicing tx */
	struct event_queue queue;	/* xlink event queue */
	struct semaphore event_sem;	/* signals tx kthread of events */
	struct completion rx_done;	/* sync start/stop of rx kthread */
	struct completion tx_done;	/* sync start/stop of tx thread */
};

/* xlink dispatcher system component */
struct xlink_dispatcher {
	struct dispatcher dispatchers[XLINK_MAX_CONNECTIONS];	/* disp queue */
	struct device *dev;					/* deallocate data */
	struct mutex lock;					/* locks when start new disp */
};

/* global reference to the xlink dispatcher data structure */
static struct xlink_dispatcher *xlinkd;

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

static u32 event_generate_id(void)
{
	static u32 id = 0xa; // TODO: temporary solution

	return id++;
}

static struct xlink_event *event_dequeue(struct event_queue *queue)
{
	struct xlink_event *event = NULL;

	mutex_lock(&queue->lock);
	if (!list_empty(&queue->head)) {
		event = list_first_entry(&queue->head, struct xlink_event,
					 list);
		list_del(&event->list);
		queue->count--;
	}
	mutex_unlock(&queue->lock);
	return event;
}

static int event_enqueue(struct event_queue *queue, struct xlink_event *event)
{
	int rc = -1;

	mutex_lock(&queue->lock);
	if (queue->count < ((queue->capacity / 10) * 7)) {
		list_add_tail(&event->list, &queue->head);
		queue->count++;
		rc = 0;
	}
	mutex_unlock(&queue->lock);
	return rc;
}

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
	return event->header.magic == XLINK_EVENT_HEADER_MAGIC;
}

static int dispatcher_event_send(struct xlink_event *event)
{
	size_t event_header_size = sizeof(event->header);
	int rc;

	// write event header
	// printk(KERN_DEBUG "Sending event: type = 0x%x, id = 0x%x\n",
			// event->header.type, event->header.id);
	rc = xlink_platform_write(event->interface,
				  event->handle->sw_device_id, &event->header,
				  &event_header_size, event->header.timeout, NULL);
	if (rc || event_header_size != sizeof(event->header)) {
		pr_err("Write header failed %d\n", rc);
		return rc;
	}
	if (event->header.type == XLINK_WRITE_REQ ||
	    event->header.type == XLINK_WRITE_VOLATILE_REQ) {
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
	struct dispatcher *disp = (struct dispatcher *)context;
	struct xlink_event *event;
	size_t size;
	int rc;

	// printk(KERN_DEBUG "dispatcher rxthread started\n");
	event = xlink_create_event(disp->link_id, 0, disp->handle, 0, 0, 0);
	if (!event)
		return -1;

	allow_signal(SIGTERM); // allow thread termination while waiting on sem
	complete(&disp->rx_done);
	while (!kthread_should_stop()) {
		size = sizeof(event->header);
		rc = xlink_platform_read(disp->interface,
					 disp->handle->sw_device_id,
					 &event->header, &size,
					 DISPATCHER_RX_TIMEOUT_MSEC, NULL);
		if (rc || size != (int)sizeof(event->header))
			continue;
		if (is_valid_event_header(event)) {
			event->link_id = disp->link_id;
			rc = xlink_multiplexer_rx(event);
			if (!rc) {
				event = xlink_create_event(disp->link_id, 0,
							   disp->handle, 0, 0,
							   0);
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
	struct dispatcher *disp = (struct dispatcher *)context;
	struct xlink_event *event;

	// printk(KERN_DEBUG "dispatcher txthread started\n");
	allow_signal(SIGTERM); // allow thread termination while waiting on sem
	complete(&disp->tx_done);
	while (!kthread_should_stop()) {
		event = dispatcher_event_get(disp);
		if (!event)
			continue;

		dispatcher_event_send(event);
		xlink_destroy_event(event); // free handled event
	}
	// printk(KERN_INFO "dispatcher txthread stopped\n");
	complete(&disp->tx_done);
	do_exit(0);
	return 0;
}

/*
 * Dispatcher External Functions
 *
 */

enum xlink_error xlink_dispatcher_init(void *dev)
{
	int i;

	xlinkd = kzalloc(sizeof(*xlinkd), GFP_KERNEL);
	if (!xlinkd)
		return X_LINK_ERROR;

	xlinkd->dev = (struct device *)dev;
	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		xlinkd->dispatchers[i].link_id = i;
		sema_init(&xlinkd->dispatchers[i].event_sem, 0);
		init_completion(&xlinkd->dispatchers[i].rx_done);
		init_completion(&xlinkd->dispatchers[i].tx_done);
		INIT_LIST_HEAD(&xlinkd->dispatchers[i].queue.head);
		mutex_init(&xlinkd->dispatchers[i].queue.lock);
		xlinkd->dispatchers[i].queue.count = 0;
		xlinkd->dispatchers[i].queue.capacity =
				XLINK_EVENT_QUEUE_CAPACITY;
		xlinkd->dispatchers[i].state = XLINK_DISPATCHER_INIT;
	}
	mutex_init(&xlinkd->lock);

	return X_LINK_SUCCESS;
}

enum xlink_error xlink_dispatcher_start(int id, struct xlink_handle *handle)
{
	struct dispatcher *disp;

	mutex_lock(&xlinkd->lock);
	// get dispatcher by link id
	disp = get_dispatcher_by_id(id);
	if (!disp)
		goto r_error;

	// cannot start a running or failed dispatcher
	if (disp->state == XLINK_DISPATCHER_RUNNING ||
	    disp->state == XLINK_DISPATCHER_ERROR)
		goto r_error;

	// set the dispatcher context
	disp->handle = handle;
	disp->interface = get_interface_from_sw_device_id(handle->sw_device_id);

	// run dispatcher thread to handle and write outgoing packets
	disp->txthread = kthread_run(xlink_dispatcher_txthread,
				     (void *)disp, "txthread");
	if (!disp->txthread) {
		pr_err("xlink txthread creation failed\n");
		goto r_txthread;
	}
	wait_for_completion(&disp->tx_done);
	disp->state = XLINK_DISPATCHER_RUNNING;
	// run dispatcher thread to read and handle incoming packets
	disp->rxthread = kthread_run(xlink_dispatcher_rxthread,
				     (void *)disp, "rxthread");
	if (!disp->rxthread) {
		pr_err("xlink rxthread creation failed\n");
		goto r_rxthread;
	}
	wait_for_completion(&disp->rx_done);
	mutex_unlock(&xlinkd->lock);

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
	struct dispatcher *disp;
	int rc;

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
	struct dispatcher *disp;
	int rc;

	mutex_lock(&xlinkd->lock);
	// get dispatcher by link id
	disp = get_dispatcher_by_id(id);
	if (!disp)
		goto r_error;

	// don't stop dispatcher if not started
	if (disp->state != XLINK_DISPATCHER_RUNNING)
		goto r_error;

	if (disp->rxthread) {
		// stop dispatcher rx thread
		send_sig(SIGTERM, disp->rxthread, 0);
		rc = kthread_stop(disp->rxthread);
		if (rc)
			goto r_thread;
	}
	wait_for_completion(&disp->rx_done);
	if (disp->txthread) {
		// stop dispatcher tx thread
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
	enum xlink_event_type type;
	struct xlink_event *event;
	struct dispatcher *disp;
	int i;

	for (i = 0; i < XLINK_MAX_CONNECTIONS; i++) {
		// get dispatcher by link id
		disp = get_dispatcher_by_id(i);
		if (!disp)
			continue;

		// stop all running dispatchers
		if (disp->state == XLINK_DISPATCHER_RUNNING)
			xlink_dispatcher_stop(i);

		// empty queues of all used dispatchers
		if (disp->state == XLINK_DISPATCHER_INIT)
			continue;

		// deallocate remaining events in queue
		while (!list_empty(&disp->queue.head)) {
			event = event_dequeue(&disp->queue);
			if (!event)
				continue;
			type = event->header.type;
			if (type == XLINK_WRITE_REQ ||
			    type == XLINK_WRITE_VOLATILE_REQ) {
				// deallocate event data
				xlink_platform_deallocate(xlinkd->dev,
							  event->data,
							  event->paddr,
							  event->header.size,
							  XLINK_PACKET_ALIGNMENT,
							  XLINK_NORMAL_MEMORY);
			}
			xlink_destroy_event(event);
		}
		// destroy dispatcher
		mutex_destroy(&disp->queue.lock);
	}
	mutex_destroy(&xlinkd->lock);
	return X_LINK_SUCCESS;
}
