// SPDX-License-Identifier: GPL-2.0-only
/*
 * xlink Linux Kernel API
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */
#ifndef __XLINK_H
#define __XLINK_H

#include <uapi/misc/xlink_uapi.h>

enum xlink_dev_type {
	HOST_DEVICE = 0,	// used when communicating to either Remote or Local host
	VPUIP_DEVICE		// used when communicating to vpu ip
};

struct xlink_handle {
	uint32_t sw_device_id;			// sw device id, identifies a device in the system
	enum xlink_dev_type dev_type;	// device type, used to determine direction of communication
};

enum xlink_opmode {
	RXB_TXB = 0,	// blocking read, blocking write
	RXN_TXN,		// non-blocking read, non-blocking write
	RXB_TXN,		// blocking read, non-blocking write
	RXN_TXB 		// non-blocking read, blocking write
};

enum xlink_device_power_mode {
	POWER_DEFAULT_NOMINAL_MAX = 0,	// no load reduction, default mode
	POWER_SUBNOMINAL_HIGH,			// slight load reduction
	POWER_MEDIUM, 					// average load reduction
	POWER_LOW, 						// significant load reduction
	POWER_MIN, 						// maximum load reduction
	POWER_SUSPENDED 				// power off or device suspend
};

enum xlink_error {
	X_LINK_SUCCESS = 0,					// xlink operation completed successfully
	X_LINK_ALREADY_INIT,				// xlink already initialized
	X_LINK_ALREADY_OPEN,				// channel already open
	X_LINK_COMMUNICATION_NOT_OPEN,		// operation on a closed channel
	X_LINK_COMMUNICATION_FAIL,			// communication failure
	X_LINK_COMMUNICATION_UNKNOWN_ERROR,	// communication failure unknown
	X_LINK_DEVICE_NOT_FOUND,			// device specified not found
	X_LINK_TIMEOUT,						// operation timed out
	X_LINK_ERROR,						// parameter error
	X_LINK_CHAN_FULL					// channel has reached fill level
};

enum xlink_device_status {
	XLINK_DEV_OFF = 0,	// device is off
	XLINK_DEV_ERROR,	// device is busy and not available
	XLINK_DEV_BUSY,		// device is available for use
	XLINK_DEV_RECOVERY,	// device is in recovery mode
	XLINK_DEV_READY		// device HW failure is detected
};

/* xlink API */

typedef void (*xlink_event)(uint16_t chan);
typedef int (*xlink_device_event_cb)(uint32_t sw_device_id, uint32_t event_type);

enum xlink_error xlink_initialize(void);

enum xlink_error xlink_connect(struct xlink_handle *handle);

enum xlink_error xlink_open_channel(struct xlink_handle *handle,
		uint16_t chan, enum xlink_opmode mode, uint32_t data_size,
		uint32_t timeout);

enum xlink_error xlink_data_available_event(struct xlink_handle *handle,
		uint16_t chan, xlink_event data_available_event);

enum xlink_error xlink_data_consumed_event(struct xlink_handle *handle,
		uint16_t chan, xlink_event data_consumed_event);

enum xlink_error xlink_close_channel(struct xlink_handle *handle,
		uint16_t chan);

enum xlink_error xlink_write_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size);

enum xlink_error xlink_read_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t **message, uint32_t *size);

enum xlink_error xlink_write_control_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size);

enum xlink_error xlink_write_volatile(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, uint32_t size);

enum xlink_error xlink_read_data_to_buffer(struct xlink_handle *handle,
		uint16_t chan, uint8_t * const message, uint32_t *size);

enum xlink_error xlink_release_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t * const data_addr);

enum xlink_error xlink_disconnect(struct xlink_handle *handle);

enum xlink_error xlink_get_device_list(uint32_t *sw_device_id_list,
		uint32_t *num_devices);

enum xlink_error xlink_get_device_name(struct xlink_handle *handle, char *name,
		size_t name_size);

enum xlink_error xlink_get_device_status(struct xlink_handle *handle,
		uint32_t *device_status);

enum xlink_error xlink_boot_device(struct xlink_handle *handle,
		const char *binary_name);

enum xlink_error xlink_reset_device(struct xlink_handle *handle);

enum xlink_error xlink_set_device_mode(struct xlink_handle *handle,
		enum xlink_device_power_mode power_mode);

enum xlink_error xlink_get_device_mode(struct xlink_handle *handle,
		enum xlink_device_power_mode *power_mode);

enum xlink_error xlink_register_device_event(struct xlink_handle *handle,
		uint32_t *event_list, uint32_t num_events,
		xlink_device_event_cb event_notif_fn);

enum xlink_error xlink_unregister_device_event(struct xlink_handle *handle,
		uint32_t *event_list, uint32_t num_events);


enum xlink_error xlink_start_vpu(char *filename); /* depreciated */

enum xlink_error xlink_stop_vpu(void); /* depreciated */

/* API functions to be implemented

enum xlink_error xlink_write_crc_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t const *message, size_t * const size);

enum xlink_error xlink_read_crc_data(struct xlink_handle *handle,
		uint16_t chan, uint8_t **message, size_t * const size);

enum xlink_error xlink_read_crc_data_to_buffer(struct xlink_handle *handle,
		uint16_t chan, uint8_t * const message, size_t * const size);

enum xlink_error xlink_reset_all(void);

 */

#endif /* __XLINK_H */
