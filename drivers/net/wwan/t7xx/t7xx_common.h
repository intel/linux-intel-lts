/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021, MediaTek Inc.
 * Copyright (c) 2021-2022, Intel Corporation.
 *
 * Authors:
 *  Haijun Liu <haijun.liu@mediatek.com>
 *  Ricardo Martinez<ricardo.martinez@linux.intel.com>
 *
 * Contributors:
 *  Andy Shevchenko <andriy.shevchenko@linux.intel.com>
 *  Chiranjeevi Rapolu <chiranjeevi.rapolu@intel.com>
 *  Eliot Lee <eliot.lee@intel.com>
 *  Moises Veleta <moises.veleta@intel.com>
 *  Sreehari Kancharla <sreehari.kancharla@intel.com>
 */

#ifndef __T7XX_COMMON_H__
#define __T7XX_COMMON_H__

#include <linux/skbuff.h>

enum md_state {
	MD_STATE_INVALID,
	MD_STATE_WAITING_FOR_HS1,
	MD_STATE_WAITING_FOR_HS2,
	MD_STATE_READY,
	MD_STATE_EXCEPTION,
	MD_STATE_WAITING_TO_STOP,
	MD_STATE_STOPPED,
};

enum mtk_txrx {
	MTK_TX,
	MTK_RX,
};

#ifdef NET_SKBUFF_DATA_USES_OFFSET
static inline unsigned int t7xx_skb_data_area_size(struct sk_buff *skb)
{
	return skb->head + skb->end - skb->data;
}
#else
static inline unsigned int t7xx_skb_data_area_size(struct sk_buff *skb)
{
	return skb->end - skb->data;
}
#endif

#endif
