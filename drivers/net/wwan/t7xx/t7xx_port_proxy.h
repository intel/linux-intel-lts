/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021, MediaTek Inc.
 * Copyright (c) 2021-2022, Intel Corporation.
 *
 * Authors:
 *  Amir Hanania <amir.hanania@intel.com>
 *  Haijun Liu <haijun.liu@mediatek.com>
 *  Moises Veleta <moises.veleta@intel.com>
 *  Ricardo Martinez<ricardo.martinez@linux.intel.com>
 *
 * Contributors:
 *  Chiranjeevi Rapolu <chiranjeevi.rapolu@intel.com>
 *  Eliot Lee <eliot.lee@intel.com>
 *  Sreehari Kancharla <sreehari.kancharla@intel.com>
 */

#ifndef __T7XX_PORT_PROXY_H__
#define __T7XX_PORT_PROXY_H__

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/skbuff.h>
#include <linux/types.h>

#include "t7xx_hif_cldma.h"
#include "t7xx_modem_ops.h"
#include "t7xx_port.h"

#define MTK_QUEUES		16
#define RX_QUEUE_MAXLEN		32
#define CTRL_QUEUE_MAXLEN	16

struct port_proxy {
	int				port_number;
	struct t7xx_port_static		*ports_shared;
	struct t7xx_port		*ports_private;
	struct list_head		rx_ch_ports[PORT_CH_ID_MASK + 1];
	struct list_head		queue_ports[CLDMA_NUM][MTK_QUEUES];
	struct device			*dev;
};

struct port_msg {
	__le32	head_pattern;
	__le32	info;
	__le32	tail_pattern;
};

#define PORT_INFO_RSRVD		GENMASK(31, 16)
#define PORT_INFO_ENFLG		BIT(15)
#define PORT_INFO_CH_ID		GENMASK(14, 0)

#define PORT_MSG_VERSION	GENMASK(31, 16)
#define PORT_MSG_PRT_CNT	GENMASK(15, 0)

#define PORT_ENUM_VER		0
#define PORT_ENUM_HEAD_PATTERN	0x5a5a5a5a
#define PORT_ENUM_TAIL_PATTERN	0xa5a5a5a5
#define PORT_ENUM_VER_MISMATCH	0x00657272

int t7xx_port_proxy_send_skb(struct t7xx_port *port, struct sk_buff *skb);
void t7xx_port_proxy_set_tx_seq_num(struct t7xx_port *port, struct ccci_header *ccci_h);
int t7xx_port_proxy_node_control(struct t7xx_modem *md, struct port_msg *port_msg);
void t7xx_port_proxy_reset(struct port_proxy *port_prox);
void t7xx_port_proxy_uninit(struct port_proxy *port_prox);
int t7xx_port_proxy_init(struct t7xx_modem *md);
void t7xx_port_proxy_md_status_notify(struct port_proxy *port_prox, unsigned int state);

#endif /* __T7XX_PORT_PROXY_H__ */
