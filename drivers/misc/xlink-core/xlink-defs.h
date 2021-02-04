// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Defines.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#ifndef __XLINK_DEFS_H
#define __XLINK_DEFS_H

#include <linux/slab.h>
#include <linux/xlink.h>

#define XLINK_MAX_BUF_SIZE 128U
#define XLINK_MAX_DATA_SIZE (1024U * 1024U * 1024U)
#define XLINK_MAX_CONTROL_DATA_SIZE 100U
#define XLINK_MAX_CONNECTIONS 16
#define XLINK_PACKET_ALIGNMENT 64
#define XLINK_INVALID_EVENT_ID 0xDEADBEEF
#define XLINK_INVALID_CHANNEL_ID 0xDEAD
#define XLINK_PACKET_QUEUE_CAPACITY 10000
#define XLINK_EVENT_QUEUE_CAPACITY 10000
#define XLINK_EVENT_HEADER_MAGIC 0x786C6E6B
#define XLINK_PING_TIMEOUT_MS 5000U
#define XLINK_MAX_DEVICE_NAME_SIZE 128
#define XLINK_MAX_DEVICE_LIST_SIZE 8
#define XLINK_INVALID_LINK_ID 0xDEADBEEF
#define XLINK_INVALID_SW_DEVICE_ID 0xDEADBEEF

#define NMB_CHANNELS 4096
#define IP_CONTROL_CHANNEL 0x0A
#define VPU_CONTROL_CHANNEL 0x400
#define CONTROL_CHANNEL_OPMODE RXB_TXB	// blocking
#define CONTROL_CHANNEL_DATASIZE 128U	// size of internal rx/tx buffers
#define CONTROL_CHANNEL_TIMEOUT_MS 0U	// wait indefinitely
#define SIGXLNK 44	// signal XLink uses for callback signalling
#define UNUSED(x) ((void)(x))

// the end of the IPC channel range (starting at zero)
#define XLINK_IPC_MAX_CHANNELS	1024

// used to extract the interface type from a sw device id
#define SW_DEVICE_ID_INTERFACE_SHIFT 24U
#define SW_DEVICE_ID_INTERFACE_MASK  0x7
#define GET_INTERFACE_FROM_SW_DEVICE_ID(id) \
		((id >> SW_DEVICE_ID_INTERFACE_SHIFT) & SW_DEVICE_ID_INTERFACE_MASK)
#define SW_DEVICE_ID_IPC_INTERFACE  0x0
#define SW_DEVICE_ID_PCIE_INTERFACE 0x1
#define SW_DEVICE_ID_USB_INTERFACE  0x2
#define SW_DEVICE_ID_ETH_INTERFACE  0x3

enum xlink_interface {
	NULL_INTERFACE = -1,
	IPC_INTERFACE = 0,
	PCIE_INTERFACE,
	USB_CDC_INTERFACE,
	ETH_INTERFACE,
	NMB_OF_INTERFACES
};

static inline int get_interface_from_sw_device_id(uint32_t sw_device_id)
{
	uint32_t interface = 0;
	interface = GET_INTERFACE_FROM_SW_DEVICE_ID(sw_device_id);
	switch (interface) {
	case SW_DEVICE_ID_IPC_INTERFACE:
		return IPC_INTERFACE;
	case SW_DEVICE_ID_PCIE_INTERFACE:
		return PCIE_INTERFACE;
	case SW_DEVICE_ID_USB_INTERFACE:
		return USB_CDC_INTERFACE;
	case SW_DEVICE_ID_ETH_INTERFACE:
		return ETH_INTERFACE;
	default:
		return NULL_INTERFACE;
	}
}

enum xlink_channel_status {
	CHAN_CLOSED			= 0x0000,
	CHAN_OPEN			= 0x0001,
	CHAN_BLOCKED_READ	= 0x0010,
	CHAN_BLOCKED_WRITE	= 0x0100,
	CHAN_OPEN_PEER		= 0x1000,
};

enum xlink_event_origin {
	EVENT_TX = 0, // outgoing events
	EVENT_RX, // incoming events
};

enum xlink_event_type {
	// request events
	XLINK_WRITE_REQ = 0x00,
	XLINK_WRITE_VOLATILE_REQ,
	XLINK_READ_REQ,
	XLINK_READ_TO_BUFFER_REQ,
	XLINK_RELEASE_REQ,
	XLINK_OPEN_CHANNEL_REQ,
	XLINK_CLOSE_CHANNEL_REQ,
	XLINK_PING_REQ,
	XLINK_WRITE_CONTROL_REQ,
	XLINK_DATA_READY_CALLBACK_REQ,
	XLINK_DATA_CONSUMED_CALLBACK_REQ,
	XLINK_PASSTHRU_WRITE_REQ,
	XLINK_PASSTHRU_VOLATILE_WRITE_REQ,
	XLINK_PASSTHRU_READ_REQ,
	XLINK_PASSTHRU_READ_TO_BUFFER_REQ,
	XLINK_REQ_LAST,
	// response events
	XLINK_WRITE_RESP = 0x10,
	XLINK_WRITE_VOLATILE_RESP,
	XLINK_READ_RESP,
	XLINK_READ_TO_BUFFER_RESP,
	XLINK_RELEASE_RESP,
	XLINK_OPEN_CHANNEL_RESP,
	XLINK_CLOSE_CHANNEL_RESP,
	XLINK_PING_RESP,
	XLINK_WRITE_CONTROL_RESP,
	XLINK_DATA_READY_CALLBACK_RESP,
	XLINK_DATA_CONSUMED_CALLBACK_RESP,
	XLINK_PASSTHRU_WRITE_RESP,
	XLINK_PASSTHRU_VOLATILE_WRITE_RESP,
	XLINK_PASSTHRU_READ_RESP,
	XLINK_PASSTHRU_READ_TO_BUFFER_RESP,
	XLINK_RESP_LAST,
};

struct xlink_event_header {
	uint32_t magic;
	uint32_t id;
	enum xlink_event_type type;
	uint16_t chan;
	size_t size;
	uint32_t timeout;
	uint8_t  control_data[XLINK_MAX_CONTROL_DATA_SIZE];
};

struct xlink_event {
	struct xlink_event_header header;
	enum xlink_event_origin origin;
	uint32_t link_id;
	struct xlink_handle *handle;
	int interface;
	void *data;
	struct task_struct *calling_pid;
	char callback_origin;
	char user_data;
	void **pdata;
	dma_addr_t paddr;
	uint32_t *length;
	struct list_head list;
};

static inline struct xlink_event *xlink_create_event(uint32_t link_id,
		enum xlink_event_type type,	struct xlink_handle *handle,
		uint16_t chan, uint32_t size, uint32_t timeout)
{
	struct xlink_event *new_event = NULL;

	// allocate new event
	new_event = kzalloc(sizeof(*new_event), GFP_KERNEL);
	if (!new_event)
		return NULL;

	// set event context
	new_event->link_id = link_id;
	new_event->handle = handle;
	new_event->interface = get_interface_from_sw_device_id(
			handle->sw_device_id);
	new_event->user_data = 0;

	// set event header
	new_event->header.magic = XLINK_EVENT_HEADER_MAGIC;
	new_event->header.id = XLINK_INVALID_EVENT_ID;
	new_event->header.type = type;
	new_event->header.chan = chan;
	new_event->header.size = size;
	new_event->header.timeout = timeout;
	return new_event;
}

static inline void xlink_destroy_event(struct xlink_event *event)
{
	if (event)
		kfree(event);
}

#endif /* __XLINK_DEFS_H */
