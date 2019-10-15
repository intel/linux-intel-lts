/* SPDX-License-Identifier: GPL-2.0 AND BSD-3-Clause
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2019 Intel Corporation.
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
 * Contact Information:
 * SoC Watch Developer Team <socwatchdevelopers@intel.com>
 * Intel Corporation,
 * 1300 S Mopac Expwy,
 * Austin, TX 78746
 *
 * BSD LICENSE
 *
 * Copyright(c) 2019 Intel Corporation.
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
 */

#include <linux/compiler.h>     /* Definition of __weak */
#include <linux/kref.h> /* struct kref */
#include <linux/notifier.h> /* struct notifier_block */

#include "sw_structs.h"      /* sw_driver_io_descriptor */
#include "sw_cta.h"

/* *********************************
 * Begin CTA driver import
 * *********************************
 */

/*
 * Struct definitions taken from CTA driver.
 */

enum intel_cta_ep_type {
        INTEL_CTA_EP_TYPE_ANY = 0,
        INTEL_CTA_EP_TYPE_PUNIT,
        INTEL_CTA_EP_TYPE_GPU,
        NUM_INTEL_CTA_EP_TYPES
};

struct telem_header {
        u8      access_type;
        u8      telem_type;
        u16     size;
        u32     guid;
        u32     base_offset;
        u32     tele_id;
};

struct discovery_entry {
        struct telem_header     header;
        void __iomem            *disc_offset;
        void __iomem            *counter_base;
};

/**
 * struct telem_endpoint - telemetry endpoint description
 * @pci_bus_no:      pci device bus number
 * @pci_devfn:       encoded pci device and function number
 * @num_entries:     Number of entries
 * @telem_ep_type:   Device specific telemetry type
 * @header_size:     Size in bytes of the discovery header
 * @discovery_entry: Pointer to list of telemetry entries. Used by
 *                   driver API to get direct access to the endpoint.
 *                   Do not dereference directly. Use driver API which
 *                   checks for device availability.
 * @kref:            Driver reference count - do not modify directly. Use
 *                   the register_endpoint and unregister_endpoint API's.
 */
struct telem_endpoint {
        u16                             pci_bus_no;
        u8                              pci_devfn;
        int                             num_entries;
        u32                             telem_ep_type;  //TODO: Implement
        u32                             header_size;
        struct discovery_entry          *entry;
        struct kref                     kref;
};

struct telem_header_info {
        u8      access_type;
        u16     size;
        u32     guid;
};

/*
 * Weak linkage of functions from the CTA driver
 */
/**
 * cta_telem_get_next_endpoint() - Get next id for a telemetry endpoint
 * @start:  starting index to look from
 * @type:   filter for type of endpoint
 *
 * Return:
 * * index       - index of next present endpoint after start
 * * 0           - when no more endpoints are present after start
 */
extern unsigned long __weak
cta_telem_get_next_endpoint(unsigned long start,
                            enum intel_cta_ep_type type);

/**
 * cta_telem_register_endpoint() - Register use of telemetry endpoint
 * @handle: ID associated with the telemetry endpoint
 *
 * Return:
 * * ep          - Succes
 * * -ENXIO      - telemetry endpoint not found
 */

extern struct telem_endpoint * __weak
cta_telem_register_endpoint(unsigned long handle);

/**
 * cta_telem_unregister_endpoint() - Unregister use of telemetry endpoint
 * @ep:   ep structure to populate.
 */
extern void __weak
cta_telem_unregister_endpoint(struct telem_endpoint *ep);


/**
 * cta_telem_get_header_info() - Get header info from a telem entry
 * @handle: ID associated with the telemetry endpoint
 * @idx:    Index of the entry
 * @info: Allocated header info structure to fill
 *
 * Return:
 * * 0           - Success
 * * -ENXIO      - telemetry endpoint not found
 * * -EINVAL     - bad argument (@idx out of range or null @info)
 */
extern int __weak
cta_telem_get_header_info(unsigned long handle, u8 idx,
                          struct telem_header_info *info);
/**
 * cta_telem_counter_read32() - Read dwords from telemetry sram into buffer
 * @ep:     Pointer to telemetry endpoint to access
 * @index:  Index of the entry
 * @offset: Register offset in bytes
 * @data:   Preallocated dword buffer to fill
 * @count:  Number of dwords requested
 *
 * Callers must ensure reads are aligned, and that both the entry index and
 * offset are within bounds. When the call returns -ENODEV, the device has
 * been removed and callers should unregister the telemetry endpoint.
 *
 * Context: RCU used to guard reads from device removal and unmap. Do not
 *          sleep.
 *
 * Return:
 * * 0           - Success
 * * -ENODEV     - The device had been removed.
 */
extern int __weak
cta_telem_counter_read32(struct telem_endpoint *ep, u32 index, u32 offset,
                         u32 *data, u32 count);
/**
 * cta_telem_counter_read64() - Read qwords from telemetry sram into buffer
 * @ep:     Pointer to telemetry endpoint to access
 * @index:  Index of the entry
 * @offset: Register offset in bytes
 * @data:   Preallocated buffer of qwords
 * @count:  Number of qwords requested
 *
 * Callers must ensure reads are aligned, and that both the entry index and
 * offset are within bounds. When the call returns -ENODEV, the device has
 * been removed and callers should unregister the telemetry endpoint.
 *
 * Context: RCU used to guard reads from device removal and unmap. Do not
 *          sleep.
 *
 * Return:
 * * 0           - Success
 * * -ENODEV     - The device had been removed.
 */
extern int __weak
cta_telem_counter_read64(struct telem_endpoint *ep, u32 index, u32 offset,
                         u64 *data, u32 count);

extern int __weak cta_telem_register_notifier(struct notifier_block *nb);
extern int __weak cta_telem_unregister_notifier(struct notifier_block *nb);

/* *********************************
 * End CTA driver import
 * *********************************
 */

#define MAX_TELEM_ENDPOINTS MAX_TELEM_AGGR_DEVICES /* For now */
static struct telem_endpoint *s_telem_endpoints[MAX_TELEM_ENDPOINTS]; /* TODO: make this a linked list instead */
size_t s_endpoint_index = 0;

static struct _sw_aggregator_msg s_telem_aggregators;

void sw_read_cta_info(char *dst, int cpu,
		const struct sw_driver_io_descriptor *descriptor,
		u16 counter_size_in_bytes)
{
	u64 *data64 = (u64 *)dst;
	u32 *data32 = (u32 *)dst;
	int retval = 0;
	const struct sw_driver_aggr_telem_io_descriptor *td =
		&(descriptor->aggr_telem_descriptor);
	u32 offset = (u32)td->offset;
	struct telem_endpoint *ep = s_telem_endpoints[0];

	/* We can only support one endpoint as of now */
	if (!ep) {
		return;
	}
	switch (descriptor->counter_size_in_bytes) {
		case 4:
			retval = cta_telem_counter_read32(ep, 0/*index*/, offset, data32, td->num_entries);
			break;
		case 8:
			retval = cta_telem_counter_read64(ep, 0/*index*/, offset, data64, td->num_entries);
			break;
		default:
			printk(KERN_ERR "Invalid CTA counter size %u\n", descriptor->counter_size_in_bytes);
			return;
	}
	if (retval) {
		printk(KERN_ERR "Error reading %u byte CTA value from offset 0x%x, val = %d\n", descriptor->counter_size_in_bytes, offset, retval);
	}
}

bool sw_cta_available(void)
{
	/* 1: check if the CTA driver is loaded */
	if (!cta_telem_get_next_endpoint) {
		return false;
	}
	/* 2: TODO: other checks here */
	/*
	 * Note: registering telemetry endpoints done in 'register' since
	 * those endpoints also need to be unregistered (Done in 'fini')
	 */
	return true;
}

bool sw_cta_register(void)
{
	unsigned long index = 0;
	if (!sw_cta_available()) {
		return false;
	}
        s_telem_aggregators.num_entries = 0;
        s_endpoint_index = 0;
	/*
	 * Retrieve list of telemetry endpoints.
	 * TODO: we can only support one endpoint as of now, so should we be
	 * checking the GUID to retrieve onl tthe endpoints of interest?
	 */
	s_endpoint_index = 0;
	while ((index = cta_telem_get_next_endpoint(index, INTEL_CTA_EP_TYPE_ANY)) && s_endpoint_index < (MAX_TELEM_ENDPOINTS-1)) {
		struct telem_header_info telem_info;
		u8 idx = 0;
		if (cta_telem_get_header_info(index, idx, &telem_info)) {
			printk(KERN_ERR "Could not retrieve telemetry header for CTA endpoint %lu\n", index);
			continue;
		}
		s_telem_endpoints[s_endpoint_index] = cta_telem_register_endpoint(index);
		s_telem_aggregators.info[s_telem_aggregators.num_entries++].globalUniqueID = telem_info.guid;
		++s_endpoint_index;
	}
	return s_endpoint_index > 0;
}

bool sw_cta_unregister(void)
{
	size_t i=0;
	if (!sw_cta_available()) {
		return false;
	}
	for (i=0; i<s_endpoint_index; ++i) {
		cta_telem_unregister_endpoint(s_telem_endpoints[i]);
	}
	s_endpoint_index = 0;
	s_telem_aggregators.num_entries = 0;
	return true;
}

struct _sw_aggregator_msg *sw_cta_aggregators(void)
{
	return &s_telem_aggregators;
}
