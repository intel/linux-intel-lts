/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_CAPABILITIES_HEADER_
#define MXLK_CAPABILITIES_HEADER_

#include "mxlk.h"
#include "mxlk_common.h"

#define MXLK_CAP_TTL (32)

static inline
void *mxlk_cap_find(struct mxlk *mxlk, u32 start, u16 id)
{
	int ttl = MXLK_CAP_TTL;
	struct mxlk_cap_hdr *hdr;
	struct mxlk_cap_hdr cur_hdr;

	// If user didn't specify start, assume start of mmio
	if (!start)
		start = mxlk_ioread32(&mxlk->mmio->cap_offset);

	// Read header info
#ifdef XLINK_PCIE_REMOTE
	hdr = (struct mxlk_cap_hdr *)((void __iomem *)mxlk->mmio + start);
#else
	hdr = (struct mxlk_cap_hdr *)((void *)mxlk->mmio + start);
#endif
	// Check if we still have time to live
	while (ttl--) {
#ifdef XLINK_PCIE_REMOTE
		memcpy_fromio(&cur_hdr, hdr, sizeof(struct mxlk_cap_hdr));
#else
		cur_hdr = *hdr;
#endif
		// If cap matches, return header
		if (cur_hdr.id == id)
			return hdr;
		// If cap is NULL, we are at the end of the list
		else if (cur_hdr.id == MXLK_CAP_NULL)
			return NULL;
		// If no match and no end of list, traverse the linked list
		else
#ifdef XLINK_PCIE_REMOTE
			hdr = (struct mxlk_cap_hdr *)
				((void __iomem *)mxlk->mmio + cur_hdr.next);
#else
			hdr = (struct mxlk_cap_hdr *)
				((void *)mxlk->mmio + cur_hdr.next);
#endif
	}

	// If we reached here, the capability list is corrupted
	return NULL;
}

static inline
void mxlk_set_td_address(struct mxlk_transfer_desc *td, u64 address)
{
	mxlk_iowrite64(address, &td->address);
}

static inline
u64 mxlk_get_td_address(struct mxlk_transfer_desc *td)
{
	return mxlk_ioread64(&td->address);
}

static inline
void mxlk_set_td_length(struct mxlk_transfer_desc *td, u32 length)
{
	mxlk_iowrite32(length, &td->length);
}

static inline
u32 mxlk_get_td_length(struct mxlk_transfer_desc *td)
{
	return mxlk_ioread32(&td->length);
}

static inline
void mxlk_set_td_interface(struct mxlk_transfer_desc *td, u16 interface)
{
	mxlk_iowrite16(interface, &td->interface);
}

static inline
u16 mxlk_get_td_interface(struct mxlk_transfer_desc *td)
{
	return mxlk_ioread16(&td->interface);
}

static inline
void mxlk_set_td_status(struct mxlk_transfer_desc *td, u16 status)
{
	mxlk_iowrite16(status, &td->status);
}

static inline
u16 mxlk_get_td_status(struct mxlk_transfer_desc *td)
{
	return mxlk_ioread16(&td->status);
}

static inline
void mxlk_set_tdr_head(struct mxlk_pipe *p, u32 head)
{
	mxlk_iowrite32(head, p->head);
}

static inline
u32 mxlk_get_tdr_head(struct mxlk_pipe *p)
{
	return mxlk_ioread32(p->head);
}

static inline
void mxlk_set_tdr_tail(struct mxlk_pipe *p, u32 tail)
{
	mxlk_iowrite32(tail, p->tail);
}

static inline
u32 mxlk_get_tdr_tail(struct mxlk_pipe *p)
{
	return mxlk_ioread32(p->tail);
}
#endif // MXLK_CAPABILITIES_HEADER_
