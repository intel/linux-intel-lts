/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2021 Intel Corporation
 */

#ifndef XPCIE_UTIL_HEADER_
#define XPCIE_UTIL_HEADER_

#include <linux/atomic.h>
#include "xpcie.h"

enum xpcie_doorbell_direction {
	TO_DEVICE,
	FROM_DEVICE
};

enum xpcie_doorbell_type {
	DATA_SENT,
	DATA_RECEIVED,
	DEV_EVENT,
	PARTIAL_DATA_RECEIVED,
	PHY_ID_UPDATED,
	RX_BD_COUNT
};

enum xpcie_event_type {
	NO_OP,
	REQUEST_RESET,
	DEV_SHUTDOWN,
	PHY_ID_RECIEVED_ACK
};

static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_mutex);

void intel_xpcie_set_doorbell(struct xpcie *xpcie,
			      enum xpcie_doorbell_direction dirt,
			      enum xpcie_doorbell_type type, u8 value);
u8 intel_xpcie_get_doorbell(struct xpcie *xpcie,
			    enum xpcie_doorbell_direction dirt,
			    enum xpcie_doorbell_type type);

void intel_xpcie_update_device_flwctl(struct xpcie *xpcie,
				      enum xpcie_doorbell_direction dirt,
				      enum xpcie_doorbell_type type,
				      int value);
u32 intel_xpcie_get_device_flwctl(struct xpcie *xpcie,
				  enum xpcie_doorbell_direction dirt,
				  enum xpcie_doorbell_type type);

void intel_xpcie_set_max_functions(struct xpcie *xpcie, u8 max_functions);
u8 intel_xpcie_get_max_functions(struct xpcie *xpcie);
u32 intel_xpcie_create_sw_device_id(u8 func_no, u16 phy_id, u8 max_functions);
void intel_xpcie_set_host_swdev_id(struct xpcie *xpcie, u32 h_sw_devid);
u32 intel_xpcie_get_host_swdev_id(struct xpcie *xpcie);

void intel_xpcie_set_device_status(struct xpcie *xpcie, u32 status);
u32 intel_xpcie_get_device_status(struct xpcie *xpcie);
u32 intel_xpcie_get_host_status(struct xpcie *xpcie);
void intel_xpcie_set_host_status(struct xpcie *xpcie, u32 status);
struct xpcie *intel_xpcie_get_device_by_name(const char *name);

struct xpcie_buf_desc *intel_xpcie_alloc_bd(size_t length);
struct xpcie_buf_desc *intel_xpcie_alloc_bd_reuse(size_t length, void *virt,
						  dma_addr_t phys);
void intel_xpcie_free_bd(struct xpcie_buf_desc *bd);

int intel_xpcie_list_init(struct xpcie_list *list);
void intel_xpcie_list_cleanup(struct xpcie_list *list);
int intel_xpcie_list_put(struct xpcie_list *list, struct xpcie_buf_desc *bd);
int intel_xpcie_list_put_head(struct xpcie_list *list,
			      struct xpcie_buf_desc *bd);
struct xpcie_buf_desc *intel_xpcie_list_get(struct xpcie_list *list);
void intel_xpcie_list_info(struct xpcie_list *list, size_t *bytes,
			   size_t *buffers);
bool intel_xpcie_list_empty(struct xpcie_list *list);

struct xpcie_buf_desc *intel_xpcie_alloc_rx_bd(struct xpcie *xpcie);
void intel_xpcie_free_rx_bd(struct xpcie *xpcie, struct xpcie_buf_desc *bd);
struct xpcie_buf_desc *intel_xpcie_alloc_tx_bd(struct xpcie *xpcie);
void intel_xpcie_free_tx_bd(struct xpcie *xpcie, struct xpcie_buf_desc *bd);

int intel_xpcie_interface_init(struct xpcie *xpcie, int id);
void intel_xpcie_interface_cleanup(struct xpcie_interface *inf);
void intel_xpcie_interfaces_cleanup(struct xpcie *xpcie);
int intel_xpcie_interfaces_init(struct xpcie *xpcie);
void intel_xpcie_add_bd_to_interface(struct xpcie *xpcie,
				     struct xpcie_buf_desc *bd);
void *intel_xpcie_cap_find(struct xpcie *xpcie, u32 start, u16 id);
void intel_xpcie_list_add_device(struct xpcie *xpcie);
void intel_xpcie_list_del_device(struct xpcie *xpcie);
u32 intel_xpcie_get_device_num(u32 *id_list);
struct xpcie *intel_xpcie_get_device_by_id(u32 id);
int intel_xpcie_get_device_status_by_id(u32 sw_devid, u32 *status);
int intel_xpcie_get_device_name_by_id(u32 sw_devid, char *device_name,
				      size_t name_size);
struct xpcie *intel_xpcie_get_device_by_phys_id(u32 phys_id);

bool intel_xpcie_list_try_packing(struct xpcie_list *list, void *data, size_t size);
#endif /* XPCIE_UTIL_HEADER_ */
