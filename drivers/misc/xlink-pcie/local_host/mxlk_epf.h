/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_EPF_HEADER_
#define MXLK_EPF_HEADER_

#include "../common/mxlk.h"
#include "../common/mxlk_util.h"

extern u32 xlink_sw_id;

extern void mxlk_register_host_irq(struct mxlk *mxlk, irq_handler_t func);

extern int mxlk_raise_irq(struct mxlk *mxlk, enum mxlk_doorbell_type type);

/*
 * These two functions are for DMA linked list mode.
 *
 * Caller should set the dst/src addresses and length for DMA descriptors in
 * mxlk_epf.dma_ll_tx_descs/dma_ll_rx_descs.
 */
extern int mxlk_copy_from_host_ll(struct mxlk *mxlk, int chan, int descs_num);
extern int mxlk_copy_to_host_ll(struct mxlk *mxlk, int chan, int descs_num);

#endif // MXLK_EPF_HEADER_
