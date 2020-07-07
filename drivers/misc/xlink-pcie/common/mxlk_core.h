/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef MXLK_CORE_HEADER_
#define MXLK_CORE_HEADER_

#include "mxlk.h"

int mxlk_core_init(struct mxlk *mxlk);
void mxlk_core_cleanup(struct mxlk *mxlk);

/*
 * @brief Read buffer from mxlk. Function will block when no data.
 *
 * @param[in] mxlk          - pointer to mxlk instance
 * @param[in] buffer        - pointer to buffer
 * @param[in] length        - max bytes to copy into buffer
 * @param[in] timeout_ms    - timeout in ms for blocking when no data
 *
 * @return:
 *      >=0 - number of bytes read
 *      <0  - linux error code
 *              -ETIME - timeout
 *              -EINTR - interrupted
 */
int mxlk_core_read(struct mxlk *mxlk, void *buffer, size_t *length,
		   uint32_t timeout_ms);

/*
 * @brief Writes buffer to mxlk. Function will block when no buffer.
 *
 * @param[in] mxlk          - pointer to mxlk instance
 * @param[in] buffer        - pointer to buffer
 * @param[in] length        - length of buffer to copy from
 * @param[in] timeout_ms    - timeout in ms for blocking when no buffer
 *
 * @return:
 *      >=0 - number of bytes write
 *      <0  - linux error code
 *              -ETIME - timeout
 *              -EINTR - interrupted
 */
int mxlk_core_write(struct mxlk *mxlk, void *buffer, size_t *length,
		    uint32_t timeout_ms);

#ifdef XLINK_PCIE_LOCAL
struct mxlk *mxlk_core_get_by_id(uint32_t sw_device_id);
#endif

#endif
