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

struct ctrl_msg_header {
	__le32	ctrl_msg_id;
	__le32	ex_msg;
	__le32	data_length;
};

/* Control identification numbers for AP<->MD messages  */
#define CTL_ID_HS1_MSG		0x0
#define CTL_ID_HS2_MSG		0x1
#define CTL_ID_HS3_MSG		0x2
#define CTL_ID_MD_EX		0x4
#define CTL_ID_DRV_VER_ERROR	0x5
#define CTL_ID_MD_EX_ACK	0x6
#define CTL_ID_MD_EX_PASS	0x8
#define CTL_ID_PORT_ENUM	0x9

/* Modem exception check identification code - "EXCP" */
#define MD_EX_CHK_ID		0x45584350
/* Modem exception check acknowledge identification code - "EREC" */
#define MD_EX_CHK_ACK_ID	0x45524543

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

/* Port operations mapping */
extern struct port_ops wwan_sub_port_ops;
extern struct port_ops ctl_port_ops;

int t7xx_port_proxy_send_skb(struct t7xx_port *port, struct sk_buff *skb);
void t7xx_port_proxy_set_tx_seq_num(struct t7xx_port *port, struct ccci_header *ccci_h);
int t7xx_port_proxy_node_control(struct t7xx_modem *md, struct port_msg *port_msg);
void t7xx_port_proxy_reset(struct port_proxy *port_prox);
void t7xx_port_proxy_send_msg_to_md(struct port_proxy *port_prox, enum port_ch ch,
				    unsigned int msg, unsigned int ex_msg);
void t7xx_port_proxy_uninit(struct port_proxy *port_prox);
int t7xx_port_proxy_init(struct t7xx_modem *md);
void t7xx_port_proxy_md_status_notify(struct port_proxy *port_prox, unsigned int state);
void t7xx_ccci_header_init(struct ccci_header *ccci_h, unsigned int pkt_header,
			   size_t pkt_len, enum port_ch ch, unsigned int ex_msg);
void t7xx_ctrl_msg_header_init(struct ctrl_msg_header *ctrl_msg_h, unsigned int msg_id,
			       unsigned int ex_msg, unsigned int len);

#endif /* __T7XX_PORT_PROXY_H__ */
