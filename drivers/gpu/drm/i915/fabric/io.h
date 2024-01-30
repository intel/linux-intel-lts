/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2020 Intel Corporation.
 *
 */

#ifndef IO_H_INCLUDED
#define IO_H_INCLUDED

#include <linux/io.h>
#include <linux/types.h>
#include <linux/stddef.h>

#include "iaf_drv.h"

#define LEN_BYTES_SHIFT 3
#define LEN_BYTES_MASK 0x0000000000000007ULL

static inline
void io_write(u64 __iomem *addr, const void *data, size_t len)
{
	size_t qwords = len >> LEN_BYTES_SHIFT;
	u64 *qw_data = (u64 *)data;

	for ( ; qwords; qwords--, addr++, qw_data++)
		writeq(*qw_data, addr);

	if (len & LEN_BYTES_MASK) {
		u64 remaining_data = 0;

		memcpy(&remaining_data, qw_data, len & LEN_BYTES_MASK);
		writeq(remaining_data, addr);
	}
}

static inline
void io_readq_aligned(u64 __iomem *addr, void *data, size_t len)
{
	size_t qwords = len >> LEN_BYTES_SHIFT;
	u64 *qw_data = (u64 *)data;

	for ( ; qwords; qwords--, addr++, qw_data++)
		*qw_data = readq(addr);

	if (len & LEN_BYTES_MASK) {
		u64 remaining_data = readq(addr);

		memcpy(qw_data, &remaining_data, len & LEN_BYTES_MASK);
	}
}

static inline
void io_readq_unaligned(void __iomem *addr, void *data, size_t len)
{
	u64 __iomem *addr_aligned_64;
	u64 leading_data;
	size_t leading_data_offset;
	size_t leading_data_sz;

	if (!len)
		return;

	addr_aligned_64 = PTR_ALIGN_DOWN(addr, sizeof(u64));

	if (addr_aligned_64 == addr) {
		io_readq_aligned(addr_aligned_64, data, len);
		return;
	}

	leading_data = readq(addr_aligned_64);
	leading_data_offset = (u8 __iomem *)addr - (u8 __iomem *)addr_aligned_64;
	leading_data_sz = sizeof(leading_data) - leading_data_offset;

	memcpy(data, (u8 *)&leading_data + leading_data_offset, min(leading_data_sz, len));

	if (len <= leading_data_sz)
		return;

	io_readq_aligned(++addr_aligned_64, (u8 *)data + leading_data_sz, len - leading_data_sz);
}

#endif
