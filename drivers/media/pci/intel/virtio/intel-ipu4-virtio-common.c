// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Intel Corporation
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/hashtable.h>

#include "intel-ipu4-virtio-common.h"

DECLARE_HASHTABLE(ipu4_virtio_fe_hash, MAX_ENTRY_FE);

void ipu4_virtio_fe_table_init(void)
{
	hash_init(ipu4_virtio_fe_hash);
}

int ipu4_virtio_fe_add(struct ipu4_virtio_fe_info *fe_info)
{
	struct ipu4_virtio_fe_info_entry *info_entry;

	info_entry = kmalloc(sizeof(*info_entry), GFP_KERNEL);

	if (!info_entry)
		return -ENOMEM;

	info_entry->info = fe_info;

	hash_add(ipu4_virtio_fe_hash, &info_entry->node,
		info_entry->info->client_id);

	return 0;
}

struct ipu4_virtio_fe_info *ipu4_virtio_fe_find(int client_id)
{
	struct ipu4_virtio_fe_info_entry *info_entry;
	int bkt;

	hash_for_each(ipu4_virtio_fe_hash, bkt, info_entry, node)
		if (info_entry->info->client_id == client_id)
			return info_entry->info;

	return NULL;
}

struct ipu4_virtio_fe_info *ipu4_virtio_fe_find_by_vmid(int vmid)
{
	struct ipu4_virtio_fe_info_entry *info_entry;
	int bkt;

	hash_for_each(ipu4_virtio_fe_hash, bkt, info_entry, node)
		if (info_entry->info->vmid == vmid)
			return info_entry->info;

	return NULL;
}

int ipu4_virtio_fe_remove(int client_id)
{
	struct ipu4_virtio_fe_info_entry *info_entry;
	int bkt;

	hash_for_each(ipu4_virtio_fe_hash, bkt, info_entry, node)
		if (info_entry->info->client_id == client_id) {
			hash_del(&info_entry->node);
			kfree(info_entry);
			return 0;
		}

	return -ENOENT;
}

int ipu4_virtio_ring_init(struct ipu4_virtio_ring *ring,
			  int ring_size)
{
	ring->buffer = kcalloc(1, ring_size * sizeof(u64), GFP_KERNEL);

	if (!ring->buffer) {
		pr_err("%s: failed to allocate memory!", __func__);
		return -ENOMEM;
	}

	ring->head = 0;
	ring->tail = 0;
	ring->used = 0;
	ring->ring_size = ring_size;
	spin_lock_init(&ring->lock);

	return 0;
}

void ipu4_virtio_ring_free(struct ipu4_virtio_ring *ring)
{
	kfree(ring->buffer);
	ring->buffer = NULL;
}

int ipu4_virtio_ring_push(struct ipu4_virtio_ring *ring, void *data)
{
	int next;

	if (ring->used == ring->ring_size) {//ring full
		pr_err("%s: Ring is full!! %d", __func__, ring->used);
		return -1;
	}

	next = ring->head + 1;
	next %= ring->ring_size;
	ring->buffer[ring->head] = (u64)data;
	ring->head = next;
	ring->used++;

	return 0;
}

void *ipu4_virtio_ring_pop(struct ipu4_virtio_ring *ring)
{
	int next;
	void *data;

	if (ring->used == 0)
		return NULL;

	next = ring->tail + 1;
	next %= ring->ring_size;

	data = (void *) ring->buffer[ring->tail];
	ring->tail = next;

	ring->used--;

	return data;
}