/*
 * virtio and hyperviosr service module (VHM): ioreq multi client feature
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2017 Intel Corporation. All rights reserved.
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
 * BSD LICENSE
 *
 * Copyright (C) 2017 Intel Corporation. All rights reserved.
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
 *
 * Jason Chen CJ <jason.cj.chen@intel.com>
 *
 */

#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/vhm/acrn_common.h>
#include <linux/vhm/acrn_vhm_ioreq.h>
#include <linux/vhm/vhm_vm_mngt.h>
#include <linux/vhm/vhm_hypercall.h>
#include <linux/idr.h>

static DEFINE_SPINLOCK(client_lock);
static struct idr	idr_client;

struct ioreq_range {
	struct list_head list;
	uint32_t type;
	long start;
	long end;
};

enum IOREQ_CLIENT_BITS {
        IOREQ_CLIENT_DESTROYING = 0,
        IOREQ_CLIENT_EXIT,
};

struct ioreq_client {
	/* client name */
	char name[16];
	/* client id */
	int id;
	/* vm this client belongs to */
	unsigned long vmid;
	/* list node for this ioreq_client */
	struct list_head list;
	/*
	 * is this client fallback?
	 * there is only one fallback client in a vm - dm
	 * a fallback client shares IOReq buffer pages
	 * a fallback client handles all left IOReq not handled by other clients
	 * a fallback client does not need add io ranges
	 * a fallback client handles ioreq in its own context
	 */
	bool fallback;

	unsigned long flags;

	/* client covered io ranges - N/A for fallback client */
	struct list_head range_list;
	spinlock_t range_lock;

	/*
	 *   this req records the req number this client need handle
	 */
	DECLARE_BITMAP(ioreqs_map, VHM_REQUEST_MAX);

	/*
	 * client ioreq handler:
	 *   if client provides a handler, it means vhm need create a kthread
	 *   to call the handler while there is ioreq.
	 *   if client doesn't provide a handler, client should handle ioreq
	 *   in its own context when calls acrn_ioreq_attach_client.
	 *
	 *   NOTE: for fallback client, there is no ioreq handler.
	 */
	ioreq_handler_t handler;
	bool vhm_create_kthread;
	struct task_struct *thread;
	wait_queue_head_t wq;

	/* pci bdf trap */
	bool trap_bdf;
	int pci_bus;
	int pci_dev;
	int pci_func;
	atomic_t refcnt;
	/* Add the vhm_vm that contains the ioreq_client */
	struct vhm_vm *ref_vm;
};

#define MAX_CLIENT 1024
static void acrn_ioreq_notify_client(struct ioreq_client *client);

static inline bool is_range_type(uint32_t type)
{
	return (type == REQ_MMIO || type == REQ_PORTIO || type == REQ_WP);
}

static inline bool has_pending_request(struct ioreq_client *client)
{
	if (client)
		return !bitmap_empty(client->ioreqs_map, VHM_REQUEST_MAX);
	else
		return false;
}

static int alloc_client(void)
{
	struct ioreq_client *client;
	int ret;

	client = kzalloc(sizeof(struct ioreq_client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	atomic_set(&client->refcnt, 1);

	spin_lock_bh(&client_lock);
	ret = idr_alloc_cyclic(&idr_client, client, 1, MAX_CLIENT, GFP_NOWAIT);
	spin_unlock_bh(&client_lock);

	if (ret < 0) {
		kfree(client);
		return -EINVAL;
	}

	client->id = ret;
	set_bit(IOREQ_CLIENT_EXIT, &client->flags);

	return ret;
}

static struct ioreq_client *acrn_ioreq_get_client(int client_id)
{
	struct ioreq_client *obj;

	spin_lock_bh(&client_lock);
	obj = idr_find(&idr_client, client_id);
	if (obj)
		atomic_inc(&obj->refcnt);
	spin_unlock_bh(&client_lock);

	return obj;
}


static void acrn_ioreq_put_client(struct ioreq_client *client)
{
	if (atomic_dec_and_test(&client->refcnt)) {
		struct vhm_vm *ref_vm = client->ref_vm;
		/* The client should be released when refcnt = 0 */
		/* TBD: Do we need to free the other resources? */
		kfree(client);
		put_vm(ref_vm);
	}
}

int acrn_ioreq_create_client(unsigned long vmid, ioreq_handler_t handler,
	char *name)
{
	struct vhm_vm *vm;
	struct ioreq_client *client;
	unsigned long flags;
	int client_id;

	might_sleep();

	vm = find_get_vm(vmid);
	if (unlikely(vm == NULL)) {
		pr_err("vhm-ioreq: failed to find vm from vmid %ld\n",
			vmid);
		return -EINVAL;
	}
	if (unlikely(vm->req_buf == NULL)) {
		pr_err("vhm-ioreq: vm[%ld]'s reqbuf is not ready\n",
			vmid);
		put_vm(vm);
		return -EINVAL;
	}

	client_id = alloc_client();
	if (unlikely(client_id < 0)) {
		pr_err("vhm-ioreq: vm[%ld] failed to alloc ioreq "
			"client id\n", vmid);
		put_vm(vm);
		return -EINVAL;
	}

	client = acrn_ioreq_get_client(client_id);
	if (unlikely(client == NULL)) {
		pr_err("failed to get the client.\n");
		put_vm(vm);
		return -EINVAL;
	}

	if (handler) {
		client->handler = handler;
		client->vhm_create_kthread = true;
	}

	client->vmid = vmid;
	client->ref_vm = vm;
	if (name)
		strncpy(client->name, name, sizeof(client->name) - 1);
	spin_lock_init(&client->range_lock);
	INIT_LIST_HEAD(&client->range_list);
	init_waitqueue_head(&client->wq);

	/* When it is added to ioreq_client_list, the refcnt is increased */
	spin_lock_irqsave(&vm->ioreq_client_lock, flags);
	list_add(&client->list, &vm->ioreq_client_list);
	spin_unlock_irqrestore(&vm->ioreq_client_lock, flags);

	pr_info("vhm-ioreq: created ioreq client %d\n", client_id);

	return client_id;
}
EXPORT_SYMBOL_GPL(acrn_ioreq_create_client);

void acrn_ioreq_clear_request(struct vhm_vm *vm)
{
	struct ioreq_client *client;
	struct list_head *pos;
	bool has_pending = false;
	int retry_cnt = 10;
	int bit;

	/*
	 * Now, ioreq clearing only happens when do VM reset. Current
	 * implementation is waiting all ioreq clients except the DM
	 * one have no pending ioreqs in 10ms per loop
	 */

	do {
		spin_lock(&vm->ioreq_client_lock);
		list_for_each(pos, &vm->ioreq_client_list) {
			client = container_of(pos, struct ioreq_client, list);
			if (vm->ioreq_fallback_client == client->id)
				continue;
			has_pending = has_pending_request(client);
			if (has_pending)
				break;
		}
		spin_unlock(&vm->ioreq_client_lock);

		if (has_pending)
			schedule_timeout_interruptible(HZ / 100);
	} while (has_pending && --retry_cnt > 0);

	if (retry_cnt == 0)
		pr_warn("ioreq client[%d] cannot flush pending request!\n",
				client->id);

	/* Clear all ioreqs belong to DM. */
	if (vm->ioreq_fallback_client > 0) {
		bit = -1;
		client = acrn_ioreq_get_client(vm->ioreq_fallback_client);
		if (!client)
			return;

		while ((bit = find_next_bit(client->ioreqs_map,
				VHM_REQUEST_MAX, bit + 1)) < VHM_REQUEST_MAX)
			acrn_ioreq_complete_request(client->id, bit, NULL);

		acrn_ioreq_put_client(client);
	}
}

int acrn_ioreq_create_fallback_client(unsigned long vmid, char *name)
{
	struct vhm_vm *vm;
	int client_id;
	struct ioreq_client *client;

	vm = find_get_vm(vmid);
	if (unlikely(vm == NULL)) {
		pr_err("vhm-ioreq: failed to find vm from vmid %ld\n",
			vmid);
		return -EINVAL;
	}

	if (unlikely(vm->ioreq_fallback_client > 0)) {
		pr_err("vhm-ioreq: there is already fallback "
				"client exist for vm %ld\n",
				vmid);
		put_vm(vm);
		return -EINVAL;
	}

	client_id = acrn_ioreq_create_client(vmid, NULL, name);
	if (unlikely(client_id < 0)) {
		put_vm(vm);
		return -EINVAL;
	}

	client = acrn_ioreq_get_client(client_id);
	if (unlikely(client == NULL)) {
		pr_err("failed to get the client.\n");
		put_vm(vm);
		return -EINVAL;
	}

	client->fallback = true;
	vm->ioreq_fallback_client = client_id;

	acrn_ioreq_put_client(client);
	put_vm(vm);

	return client_id;
}

/* When one client is removed from VM, the refcnt is decreased */
static void acrn_ioreq_destroy_client_pervm(struct ioreq_client *client,
		struct vhm_vm *vm)
{
	struct list_head *pos, *tmp;
	unsigned long flags;

	set_bit(IOREQ_CLIENT_DESTROYING, &client->flags);
	acrn_ioreq_notify_client(client);

	while (client->vhm_create_kthread && !test_bit(IOREQ_CLIENT_EXIT, &client->flags))
		msleep(10);

	spin_lock_irqsave(&client->range_lock, flags);
	list_for_each_safe(pos, tmp, &client->range_list) {
		struct ioreq_range *range =
			container_of(pos, struct ioreq_range, list);
		list_del(&range->list);
		kfree(range);
	}
	spin_unlock_irqrestore(&client->range_lock, flags);

	spin_lock_irqsave(&vm->ioreq_client_lock, flags);
	list_del(&client->list);
	spin_unlock_irqrestore(&vm->ioreq_client_lock, flags);

	if (client->id == vm->ioreq_fallback_client)
		vm->ioreq_fallback_client = -1;

	acrn_ioreq_put_client(client);
}

void acrn_ioreq_destroy_client(int client_id)
{
	struct ioreq_client *client;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return;
	}

	spin_lock_bh(&client_lock);
	client = idr_remove(&idr_client, client_id);
	spin_unlock_bh(&client_lock);

	/* When the client_id is already released, just keep silience can returnd */
	if (!client)
		return;

	might_sleep();

	acrn_ioreq_destroy_client_pervm(client, client->ref_vm);
	acrn_ioreq_put_client(client);
}
EXPORT_SYMBOL_GPL(acrn_ioreq_destroy_client);

static void __attribute__((unused)) dump_iorange(struct ioreq_client *client)
{
	struct list_head *pos;
	unsigned long flags;

	spin_lock_irqsave(&client->range_lock, flags);
	list_for_each(pos, &client->range_list) {
		struct ioreq_range *range =
			container_of(pos, struct ioreq_range, list);
		pr_debug("\tio range: type %d, start 0x%lx, "
			"end 0x%lx\n", range->type, range->start, range->end);
	}
	spin_unlock_irqrestore(&client->range_lock, flags);
}

/*
 * NOTE: here just add iorange entry directly, no check for the overlap..
 * please client take care of it
 */
int acrn_ioreq_add_iorange(int client_id, uint32_t type,
	long start, long end)
{
	struct ioreq_client *client;
	struct ioreq_range *range;
	unsigned long flags;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EFAULT;
	}
	if (end < start) {
		pr_err("vhm-ioreq: end < start\n");
		return -EFAULT;
	}

	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EFAULT;
	}

	might_sleep();

	range = kzalloc(sizeof(struct ioreq_range), GFP_KERNEL);
	if (!range) {
		pr_err("vhm-ioreq: failed to alloc ioreq range\n");
		acrn_ioreq_put_client(client);
		return -ENOMEM;
	}
	range->type = type;
	range->start = start;
	range->end = end;

	spin_lock_irqsave(&client->range_lock, flags);
	list_add(&range->list, &client->range_list);
	spin_unlock_irqrestore(&client->range_lock, flags);
	acrn_ioreq_put_client(client);

	return 0;
}
EXPORT_SYMBOL_GPL(acrn_ioreq_add_iorange);

int acrn_ioreq_del_iorange(int client_id, uint32_t type,
	long start, long end)
{
	struct ioreq_client *client;
	struct ioreq_range *range;
	struct list_head *pos, *tmp;
	unsigned long flags;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EFAULT;
	}
	if (end < start) {
		pr_err("vhm-ioreq: end < start\n");
		return -EFAULT;
	}

	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EFAULT;
	}

	might_sleep();

	spin_lock_irqsave(&client->range_lock, flags);
	list_for_each_safe(pos, tmp, &client->range_list) {
		range = container_of(pos, struct ioreq_range, list);
		if (range->type == type) {
			if (is_range_type(type)) {
				if (start == range->start &&
					end == range->end) {
					list_del(&range->list);
					kfree(range);
					break;
				}
			} else {
				list_del(&range->list);
				kfree(range);
				break;
			}
		}
	}
	spin_unlock_irqrestore(&client->range_lock, flags);
	acrn_ioreq_put_client(client);

	return 0;
}
EXPORT_SYMBOL_GPL(acrn_ioreq_del_iorange);

static inline bool is_destroying(struct ioreq_client *client)
{
	if (client)
		return test_bit(IOREQ_CLIENT_DESTROYING, &client->flags);
	else
		return true;
}

struct vhm_request *acrn_ioreq_get_reqbuf(int client_id)
{
	struct ioreq_client *client;
	struct vhm_vm *vm;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return NULL;
	}
	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return NULL;
	}
	vm = client->ref_vm;
	if (unlikely(vm == NULL || vm->req_buf == NULL)) {
		pr_warn("vhm-ioreq: the req buf page not ready yet "
			"for vmid %ld\n", client->vmid);
	}
	acrn_ioreq_put_client(client);
	return (struct vhm_request *)vm->req_buf;
}
EXPORT_SYMBOL_GPL(acrn_ioreq_get_reqbuf);

static int ioreq_client_thread(void *data)
{
	struct ioreq_client *client;
	int ret, client_id = (unsigned long)data;
	struct vhm_vm *vm;

	client = acrn_ioreq_get_client(client_id);

	if (!client)
		return 0;

	vm = client->ref_vm;
	if (unlikely(vm == NULL)) {
		pr_err("vhm-ioreq: failed to find vm from vmid %ld\n",
			client->vmid);
		acrn_ioreq_put_client(client);
		return -EINVAL;
	}

	while (1) {
		if (is_destroying(client)) {
			pr_info("vhm-ioreq: client destroying->stop thread\n");
			break;
		}
		if (has_pending_request(client)) {
			if (client->handler) {
				ret = client->handler(client->id,
					client->ioreqs_map);
				if (ret < 0) {
					pr_err("vhm-ioreq: err:%d\n", ret);
					break;
				}
			} else {
				pr_err("vhm-ioreq: no ioreq handler\n");
				break;
			}
		} else
			wait_event_freezable(client->wq,
				(has_pending_request(client) ||
				is_destroying(client)));
	}

	set_bit(IOREQ_CLIENT_EXIT, &client->flags);
	acrn_ioreq_put_client(client);
	return 0;
}

int acrn_ioreq_attach_client(int client_id, bool check_kthread_stop)
{
	struct ioreq_client *client;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EFAULT;
	}
	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EFAULT;
	}

	if (client->vhm_create_kthread) {
		if (client->thread) {
			pr_warn("vhm-ioreq: kthread already exist"
					" for client %s\n", client->name);
			acrn_ioreq_put_client(client);
			return 0;
		}
		client->thread = kthread_run(ioreq_client_thread,
				(void *)(unsigned long)client_id,
				"ioreq_client[%ld]:%s",
				client->vmid, client->name);
		if (IS_ERR(client->thread)) {
			pr_err("vhm-ioreq: failed to run kthread "
					"for client %s\n", client->name);
			acrn_ioreq_put_client(client);
			return -ENOMEM;
		}
		clear_bit(IOREQ_CLIENT_EXIT, &client->flags);
	} else {
		clear_bit(IOREQ_CLIENT_EXIT, &client->flags);
		might_sleep();

		if (check_kthread_stop) {
			wait_event_freezable(client->wq,
				(kthread_should_stop() ||
				has_pending_request(client) ||
				is_destroying(client)));
			if (kthread_should_stop())
				set_bit(IOREQ_CLIENT_EXIT, &client->flags);
		} else {
			wait_event_freezable(client->wq,
				(has_pending_request(client) ||
				is_destroying(client)));
		}

		if (is_destroying(client)) {
			set_bit(IOREQ_CLIENT_EXIT, &client->flags);
			acrn_ioreq_put_client(client);
			return 1;
		}
	}

	acrn_ioreq_put_client(client);
	return 0;
}
EXPORT_SYMBOL_GPL(acrn_ioreq_attach_client);

void acrn_ioreq_intercept_bdf(int client_id, int bus, int dev, int func)
{
	struct ioreq_client *client;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return;
	}
	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return;
	}
	client->trap_bdf = true;
	client->pci_bus = bus;
	client->pci_dev = dev;
	client->pci_func = func;
	acrn_ioreq_put_client(client);
}
EXPORT_SYMBOL_GPL(acrn_ioreq_intercept_bdf);

void acrn_ioreq_unintercept_bdf(int client_id)
{
	struct ioreq_client *client;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return;
	}
	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return;
	}
	client->trap_bdf = false;
	client->pci_bus = -1;
	client->pci_dev = -1;
	client->pci_func = -1;
	acrn_ioreq_put_client(client);
}

static void acrn_ioreq_notify_client(struct ioreq_client *client)
{
	/* if client thread is in waitqueue, wake up it */
	if (waitqueue_active(&client->wq))
		wake_up_interruptible(&client->wq);
}

static int ioreq_complete_request(unsigned long vmid, int vcpu,
		struct vhm_request *vhm_req)
{
	bool polling_mode;

	polling_mode = vhm_req->completion_polling;
	smp_mb();
	atomic_set(&vhm_req->processed, REQ_STATE_COMPLETE);
	/*
	 * In polling mode, HV will poll ioreqs' completion.
	 * Once marked the ioreq as REQ_STATE_COMPLETE, hypervisor side
	 * can poll the result and continue the IO flow. Thus, we don't
	 * need to notify hypervisor by hypercall.
	 * Please note, we need get completion_polling before set the request
	 * as complete, or we will race with hypervisor.
	 */
	if (!polling_mode) {
		if (hcall_notify_req_finish(vmid, vcpu) < 0) {
			pr_err("vhm-ioreq: notify request complete failed!\n");
			return -EFAULT;
		}
	}

	return 0;
}

static bool req_in_range(struct ioreq_range *range, struct vhm_request *req)
{
	bool ret = false;

	if (range->type == req->type) {
		switch (req->type) {
		case REQ_MMIO:
		case REQ_WP:
		{
			if (req->reqs.mmio_request.address >= range->start &&
				(req->reqs.mmio_request.address +
				req->reqs.mmio_request.size - 1) <= range->end)
				ret = true;
			break;
		}
		case REQ_PORTIO: {
			if (req->reqs.pio_request.address >= range->start &&
				(req->reqs.pio_request.address +
				req->reqs.pio_request.size - 1) <= range->end)
				ret = true;
			break;
		}

		default:
			ret = false;
			break;
		}
	}

	return ret;
}

static bool is_cfg_addr(struct vhm_request *req)
{
	return (req->type == REQ_PORTIO &&
		(req->reqs.pio_request.address >= 0xcf8 &&
		req->reqs.pio_request.address < 0xcf8+4));
}

static bool is_cfg_data(struct vhm_request *req)
{
	return (req->type == REQ_PORTIO &&
		(req->reqs.pio_request.address >= 0xcfc &&
		req->reqs.pio_request.address < 0xcfc+4));
}

#define PCI_LOWREG_MASK 255     /* the low 8-bit of supported pci_reg addr.*/
#define PCI_HIGHREG_MASK 0xF00  /* the high 4-bit of supported pci_reg addr */
#define PCI_FUNCMAX	7       /* highest supported function number */
#define PCI_SLOTMAX	31      /* highest supported slot number */
#define PCI_BUSMAX	255     /* highest supported bus number */
#define CONF1_ENABLE	0x80000000ul
static int handle_cf8cfc(struct vhm_vm *vm, struct vhm_request *req, int vcpu)
{
	int req_handled = 0;
	int err = 0;

	/*XXX: like DM, assume cfg address write is size 4 */
	if (is_cfg_addr(req)) {
		if (req->reqs.pio_request.direction == REQUEST_WRITE) {
			if (req->reqs.pio_request.size == 4) {

				vm->pci_conf_addr = (uint32_t ) req->reqs.pio_request.value;
				req_handled = 1;
			}
		} else {
			if (req->reqs.pio_request.size == 4) {
				req->reqs.pio_request.value =
					vm->pci_conf_addr;
				req_handled = 1;
			}
		}
	} else if (is_cfg_data(req)) {
		if (!(vm->pci_conf_addr & CONF1_ENABLE)) {
			if (req->reqs.pio_request.direction == REQUEST_READ)
				req->reqs.pio_request.value = 0xffffffff;
			req_handled = 1;
		} else {
			/* pci request is same as io request at top */
			int offset = req->reqs.pio_request.address - 0xcfc;
			int pci_reg;

			req->type = REQ_PCICFG;
			req->reqs.pci_request.bus = (vm->pci_conf_addr >> 16) &
							PCI_BUSMAX;
			req->reqs.pci_request.dev = (vm->pci_conf_addr >> 11) &
							PCI_SLOTMAX;
			req->reqs.pci_request.func = (vm->pci_conf_addr >> 8) &
							PCI_FUNCMAX;
			pci_reg = (vm->pci_conf_addr & PCI_LOWREG_MASK) +
					((vm->pci_conf_addr >> 16) & PCI_HIGHREG_MASK);
			req->reqs.pci_request.reg = pci_reg + offset;
		}
	}

	if (req_handled)
		err = ioreq_complete_request(vm->vmid, vcpu, req);

	return err ? err: req_handled;
}

static bool bdf_match(struct vhm_vm *vm, struct ioreq_client *client)
{
	int cached_bus, cached_dev, cached_func;

	cached_bus = (vm->pci_conf_addr >> 16) & PCI_BUSMAX;
	cached_dev = (vm->pci_conf_addr >> 11) & PCI_SLOTMAX;
	cached_func = (vm->pci_conf_addr >> 8) & PCI_FUNCMAX;
	return (client->trap_bdf &&
		client->pci_bus == cached_bus &&
		client->pci_dev == cached_dev &&
		client->pci_func == cached_func);
}

static struct ioreq_client *acrn_ioreq_find_client_by_request(struct vhm_vm *vm,
	struct vhm_request *req)
{
	struct list_head *pos, *range_pos;
	struct ioreq_client *client;
	int target_client,fallback_client;
	struct ioreq_range *range;
	bool found = false;

	target_client = 0;
	fallback_client = 0;
	spin_lock(&vm->ioreq_client_lock);
	list_for_each(pos, &vm->ioreq_client_list) {
		client = container_of(pos, struct ioreq_client, list);

		if (client->fallback) {
			fallback_client = client->id;
			continue;
		}

		if (req->type == REQ_PCICFG) {
			if (bdf_match(vm, client)) { /* bdf match client */
				target_client = client->id;
				break;
			} else /* other or fallback client */
				continue;
		}

		spin_lock(&client->range_lock);
		list_for_each(range_pos, &client->range_list) {
			range =
			container_of(range_pos, struct ioreq_range, list);
			if (req_in_range(range, req)) {
				found = true;
				target_client = client->id;
				break;
			}
		}
		spin_unlock(&client->range_lock);

		if (found)
			break;
	}
	spin_unlock(&vm->ioreq_client_lock);

	if (target_client > 0)
		return acrn_ioreq_get_client(target_client);

	if (fallback_client > 0)
		return acrn_ioreq_get_client(fallback_client);

	return NULL;
}

int acrn_ioreq_distribute_request(struct vhm_vm *vm)
{
	struct vhm_request *req;
	struct list_head *pos;
	struct ioreq_client *client;
	int i, vcpu_num;

	vcpu_num = atomic_read(&vm->vcpu_num);
	for (i = 0; i < vcpu_num; i++) {
		req = vm->req_buf->req_queue + i;

		/* This function is called in tasklet only on SOS CPU0. Thus it
		 * is safe to read the state first and update it later as long
		 * as the update is atomic. */
		if (atomic_read(&req->processed) == REQ_STATE_PENDING) {
			if (handle_cf8cfc(vm, req, i))
				continue;
			client = acrn_ioreq_find_client_by_request(vm, req);
			if (client == NULL) {
				pr_err("vhm-ioreq: failed to "
						"find ioreq client\n");
				return -EINVAL;
			} else {
				req->client = client->id;
				atomic_set(&req->processed, REQ_STATE_PROCESSING);
				set_bit(i, client->ioreqs_map);
				acrn_ioreq_put_client(client);
			}
		}
	}

	spin_lock(&vm->ioreq_client_lock);
	list_for_each(pos, &vm->ioreq_client_list) {
		client = container_of(pos, struct ioreq_client, list);
		if (has_pending_request(client))
			acrn_ioreq_notify_client(client);
	}
	spin_unlock(&vm->ioreq_client_lock);

	return 0;
}

int acrn_ioreq_complete_request(int client_id, uint64_t vcpu,
		struct vhm_request *vhm_req)
{
	struct ioreq_client *client;
	int ret;

	if (client_id < 0 || client_id >= MAX_CLIENT) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EINVAL;
	}
	client = acrn_ioreq_get_client(client_id);
	if (!client) {
		pr_err("vhm-ioreq: no client for id %d\n", client_id);
		return -EINVAL;
	}

	clear_bit(vcpu, client->ioreqs_map);
	if (!vhm_req) {
		vhm_req = acrn_ioreq_get_reqbuf(client_id);
		if (!vhm_req) {
			acrn_ioreq_put_client(client);
			return -EINVAL;
		}
		vhm_req += vcpu;
	}

	ret = ioreq_complete_request(client->vmid, vcpu, vhm_req);
	acrn_ioreq_put_client(client);

	return ret;
}
EXPORT_SYMBOL_GPL(acrn_ioreq_complete_request);

unsigned int vhm_dev_poll(struct file *filep, poll_table *wait)
{
	struct vhm_vm *vm = filep->private_data;
	struct ioreq_client *fallback_client;
	unsigned int ret = 0;

	if (vm == NULL || vm->req_buf == NULL ||
		vm->ioreq_fallback_client <= 0) {
		pr_err("vhm: invalid VM !\n");
		ret = POLLERR;
		return ret;
	}

	fallback_client = acrn_ioreq_get_client(vm->ioreq_fallback_client);
	if (!fallback_client) {
		pr_err("vhm-ioreq: no client for id %d\n",
			vm->ioreq_fallback_client);
		return -EINVAL;
	}

	poll_wait(filep, &fallback_client->wq, wait);
	if (has_pending_request(fallback_client) ||
		is_destroying(fallback_client))
		ret = POLLIN | POLLRDNORM;

	acrn_ioreq_put_client(fallback_client);

	return ret;
}

int acrn_ioreq_init(struct vhm_vm *vm, unsigned long vma)
{
	struct acrn_set_ioreq_buffer set_buffer;
	struct page *page;
	int ret;

	if (vm->req_buf)
		return -EEXIST;

	ret = get_user_pages_fast(vma, 1, 1, &page);
	if (unlikely(ret != 1) || (page == NULL)) {
		pr_err("vhm-ioreq: failed to pin request buffer!\n");
		return -ENOMEM;
	}

	vm->req_buf = page_address(page);
	vm->pg = page;

	set_buffer.req_buf = page_to_phys(page);

	ret = hcall_set_ioreq_buffer(vm->vmid, virt_to_phys(&set_buffer));
	if (ret < 0) {
		pr_err("vhm-ioreq: failed to set request buffer !\n");
		return -EFAULT;
	}

	pr_info("vhm-ioreq: init request buffer @ %p!\n",
		vm->req_buf);

	return 0;
}

void acrn_ioreq_free(struct vhm_vm *vm)
{
	struct list_head *pos, *tmp;

	/* When acrn_ioreq_destory_client is called, it will be released
	 * and removed from vm->ioreq_client_list.
	 * The below is used to assure that the client is still released even when
	 * it is not called.
	 */
	if (!test_and_set_bit(VHM_VM_IOREQ, &vm->flags)) {
		get_vm(vm);
		list_for_each_safe(pos, tmp, &vm->ioreq_client_list) {
			struct ioreq_client *client =
				container_of(pos, struct ioreq_client, list);
			acrn_ioreq_destroy_client(client->id);
		}
		put_vm(vm);
	}

}

void acrn_ioreq_driver_init()
{
	idr_init(&idr_client);
}
