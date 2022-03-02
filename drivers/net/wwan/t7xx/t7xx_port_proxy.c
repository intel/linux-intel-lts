// SPDX-License-Identifier: GPL-2.0-only
/*
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
 *  Andy Shevchenko <andriy.shevchenko@linux.intel.com>
 *  Chandrashekar Devegowda <chandrashekar.devegowda@intel.com>
 *  Chiranjeevi Rapolu <chiranjeevi.rapolu@intel.com>
 *  Eliot Lee <eliot.lee@intel.com>
 *  Sreehari Kancharla <sreehari.kancharla@intel.com>
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/wwan.h>

#include "t7xx_common.h"
#include "t7xx_hif_cldma.h"
#include "t7xx_modem_ops.h"
#include "t7xx_port.h"
#include "t7xx_port_proxy.h"
#include "t7xx_state_monitor.h"

#define Q_IDX_CTRL			0
#define Q_IDX_MBIM			2
#define Q_IDX_AT_CMD			5

#define INVALID_SEQ_NUM			GENMASK(15, 0)

#define for_each_proxy_port(i, p, proxy)	\
	for (i = 0, (p) = &(proxy)->ports_private[i];	\
	     i < (proxy)->port_number;		\
	     i++, (p) = &(proxy)->ports_private[i])

static struct t7xx_port_static t7xx_md_ports[1];

static struct t7xx_port *t7xx_proxy_get_port_by_ch(struct port_proxy *port_prox, enum port_ch ch)
{
	struct t7xx_port_static *port_static;
	struct t7xx_port *port;
	int i;

	for_each_proxy_port(i, port, port_prox) {
		port_static = port->port_static;
		if (port_static->rx_ch == ch || port_static->tx_ch == ch)
			return port;
	}

	return NULL;
}

void t7xx_port_proxy_set_tx_seq_num(struct t7xx_port *port, struct ccci_header *ccci_h)
{
	ccci_h->status &= cpu_to_le32(~CCCI_H_SEQ_FLD);
	ccci_h->status |= cpu_to_le32(FIELD_PREP(CCCI_H_SEQ_FLD, port->seq_nums[MTK_TX]));
	ccci_h->status |= cpu_to_le32(CCCI_H_AST_BIT);
}

static u16 t7xx_port_next_rx_seq_num(struct t7xx_port *port, struct ccci_header *ccci_h)
{
	u16 seq_num, next_seq_num, assert_bit;

	seq_num = FIELD_GET(CCCI_H_SEQ_FLD, le32_to_cpu(ccci_h->status));
	next_seq_num = (seq_num + 1) & FIELD_MAX(CCCI_H_SEQ_FLD);
	assert_bit = !!(le32_to_cpu(ccci_h->status) & CCCI_H_AST_BIT);
	if (!assert_bit || port->seq_nums[MTK_RX] > FIELD_MAX(CCCI_H_SEQ_FLD))
		return next_seq_num;

	if (seq_num != port->seq_nums[MTK_RX]) {
		dev_warn_ratelimited(port->dev,
				     "seq num out-of-order %u != %u (header %X, len %X)\n",
				     seq_num, port->seq_nums[MTK_RX],
				     le32_to_cpu(ccci_h->packet_header),
				     le32_to_cpu(ccci_h->packet_len));
	}

	return next_seq_num;
}

void t7xx_port_proxy_reset(struct port_proxy *port_prox)
{
	struct t7xx_port *port;
	int i;

	for_each_proxy_port(i, port, port_prox) {
		port->seq_nums[MTK_RX] = INVALID_SEQ_NUM;
		port->seq_nums[MTK_TX] = 0;
	}
}

static int t7xx_port_get_queue_no(struct t7xx_port *port)
{
	struct t7xx_port_static *port_static = port->port_static;
	struct t7xx_fsm_ctl *ctl = port->t7xx_dev->md->fsm_ctl;

	return t7xx_fsm_get_md_state(ctl) == MD_STATE_EXCEPTION ?
		port_static->txq_exp_index : port_static->txq_index;
}

static void t7xx_port_struct_init(struct t7xx_port *port)
{
	INIT_LIST_HEAD(&port->entry);
	INIT_LIST_HEAD(&port->queue_entry);
	skb_queue_head_init(&port->rx_skb_list);
	init_waitqueue_head(&port->rx_wq);
	port->seq_nums[MTK_RX] = INVALID_SEQ_NUM;
	port->seq_nums[MTK_TX] = 0;
	atomic_set(&port->usage_cnt, 0);
}

static void t7xx_port_adjust_skb(struct t7xx_port *port, struct sk_buff *skb)
{
	struct ccci_header *ccci_h = (struct ccci_header *)skb->data;
	struct t7xx_port_static *port_static = port->port_static;

	if (port->flags & PORT_F_USER_HEADER) {
		if (le32_to_cpu(ccci_h->packet_header) == CCCI_HEADER_NO_DATA) {
			if (skb->len > sizeof(*ccci_h)) {
				dev_err_ratelimited(port->dev,
						    "Recv unexpected data for %s, skb->len=%d\n",
						    port_static->name, skb->len);
				skb_trim(skb, sizeof(*ccci_h));
			}
		}
	} else {
		skb_pull(skb, sizeof(*ccci_h));
	}
}

/**
 * t7xx_port_recv_skb() - receive skb from modem or HIF.
 * @port: port to use.
 * @skb: skb to use.
 *
 * Used to receive native HIF RX data, which has same the RX receive flow.
 *
 * Return:
 * * 0		- Success.
 * * ERROR	- Error code.
 */
int t7xx_port_recv_skb(struct t7xx_port *port, struct sk_buff *skb)
{
	struct ccci_header *ccci_h;
	unsigned long flags;
	u32 channel;
	int ret = 0;

	spin_lock_irqsave(&port->rx_wq.lock, flags);
	if (port->rx_skb_list.qlen >= port->rx_length_th) {
		port->flags |= PORT_F_RX_FULLED;
		spin_unlock_irqrestore(&port->rx_wq.lock, flags);

		return -ENOBUFS;
	}
	ccci_h = (struct ccci_header *)skb->data;
	port->flags &= ~PORT_F_RX_FULLED;
	if (port->flags & PORT_F_RX_ADJUST_HEADER)
		t7xx_port_adjust_skb(port, skb);
	channel = FIELD_GET(CCCI_H_CHN_FLD, le32_to_cpu(ccci_h->status));
	if (channel == PORT_CH_STATUS_RX) {
		ret = port->skb_handler(port, skb);
	} else {
		if (port->wwan_port)
			wwan_port_rx(port->wwan_port, skb);
		else
			__skb_queue_tail(&port->rx_skb_list, skb);
	}
	spin_unlock_irqrestore(&port->rx_wq.lock, flags);

	wake_up_all(&port->rx_wq);
	return ret;
}

static struct cldma_ctrl *get_md_ctrl(struct t7xx_port *port)
{
	enum cldma_id id = port->port_static->path_id;

	return port->t7xx_dev->md->md_ctrl[id];
}

int t7xx_port_proxy_send_skb(struct t7xx_port *port, struct sk_buff *skb)
{
	struct ccci_header *ccci_h = (struct ccci_header *)(skb->data);
	struct cldma_ctrl *md_ctrl;
	unsigned char tx_qno;
	int ret;

	tx_qno = t7xx_port_get_queue_no(port);
	t7xx_port_proxy_set_tx_seq_num(port, ccci_h);

	md_ctrl = get_md_ctrl(port);
	ret = t7xx_cldma_send_skb(md_ctrl, tx_qno, skb);
	if (ret) {
		dev_err(port->dev, "Failed to send skb: %d\n", ret);
		return ret;
	}

	port->seq_nums[MTK_TX]++;

	return 0;
}

int t7xx_port_send_skb_to_md(struct t7xx_port *port, struct sk_buff *skb)
{
	struct t7xx_port_static *port_static = port->port_static;
	struct t7xx_fsm_ctl *ctl = port->t7xx_dev->md->fsm_ctl;
	struct cldma_ctrl *md_ctrl;
	enum md_state md_state;
	unsigned int fsm_state;

	md_state = t7xx_fsm_get_md_state(ctl);

	fsm_state = t7xx_fsm_get_ctl_state(ctl);
	if (fsm_state != FSM_STATE_PRE_START) {
		if (md_state == MD_STATE_WAITING_FOR_HS1 || md_state == MD_STATE_WAITING_FOR_HS2)
			return -ENODEV;

		if (md_state == MD_STATE_EXCEPTION && port_static->tx_ch != PORT_CH_MD_LOG_TX &&
		    port_static->tx_ch != PORT_CH_UART1_TX)
			return -EBUSY;

		if (md_state == MD_STATE_STOPPED || md_state == MD_STATE_WAITING_TO_STOP ||
		    md_state == MD_STATE_INVALID)
			return -ENODEV;
	}

	md_ctrl = get_md_ctrl(port);
	return t7xx_cldma_send_skb(md_ctrl, t7xx_port_get_queue_no(port), skb);
}

static void t7xx_proxy_setup_ch_mapping(struct port_proxy *port_prox)
{
	struct t7xx_port *port;

	int i, j;

	for (i = 0; i < ARRAY_SIZE(port_prox->rx_ch_ports); i++)
		INIT_LIST_HEAD(&port_prox->rx_ch_ports[i]);

	for (j = 0; j < ARRAY_SIZE(port_prox->queue_ports); j++) {
		for (i = 0; i < ARRAY_SIZE(port_prox->queue_ports[j]); i++)
			INIT_LIST_HEAD(&port_prox->queue_ports[j][i]);
	}

	for_each_proxy_port(i, port, port_prox) {
		struct t7xx_port_static *port_static = port->port_static;
		enum cldma_id path_id = port_static->path_id;
		u8 ch_id;

		ch_id = FIELD_GET(PORT_CH_ID_MASK, port_static->rx_ch);
		list_add_tail(&port->entry, &port_prox->rx_ch_ports[ch_id]);
		list_add_tail(&port->queue_entry,
			      &port_prox->queue_ports[path_id][port_static->rxq_index]);
	}
}

static struct t7xx_port *t7xx_port_proxy_find_port(struct t7xx_pci_dev *t7xx_dev,
						   struct cldma_queue *queue, u16 channel)
{
	struct port_proxy *port_prox = t7xx_dev->md->port_prox;
	struct list_head *port_list;
	struct t7xx_port *port;
	u8 ch_id;

	ch_id = FIELD_GET(PORT_CH_ID_MASK, channel);
	port_list = &port_prox->rx_ch_ports[ch_id];
	list_for_each_entry(port, port_list, entry) {
		struct t7xx_port_static *port_static = port->port_static;

		if (queue->md_ctrl->hif_id == port_static->path_id &&
		    channel == port_static->rx_ch)
			return port;
	}

	return NULL;
}

/**
 * t7xx_port_proxy_recv_skb() - Dispatch received skb.
 * @queue: CLDMA queue.
 * @skb: Socket buffer.
 *
 * Return:
 ** 0		- Packet consumed.
 ** -ERROR	- Failed to process skb.
 */
static int t7xx_port_proxy_recv_skb(struct cldma_queue *queue, struct sk_buff *skb)
{
	struct ccci_header *ccci_h = (struct ccci_header *)skb->data;
	struct t7xx_pci_dev *t7xx_dev = queue->md_ctrl->t7xx_dev;
	struct t7xx_fsm_ctl *ctl = t7xx_dev->md->fsm_ctl;
	struct device *dev = queue->md_ctrl->dev;
	struct t7xx_port_static *port_static;
	struct t7xx_port *port;
	u16 seq_num, channel;
	int ret;

	if (!skb)
		return -EINVAL;

	channel = FIELD_GET(CCCI_H_CHN_FLD, le32_to_cpu(ccci_h->status));
	if (t7xx_fsm_get_md_state(ctl) == MD_STATE_INVALID) {
		dev_err_ratelimited(dev, "Packet drop on channel 0x%x, modem not ready\n", channel);
		goto drop_skb;
	}

	port = t7xx_port_proxy_find_port(t7xx_dev, queue, channel);
	if (!port) {
		dev_err_ratelimited(dev, "Packet drop on channel 0x%x, port not found\n", channel);
		goto drop_skb;
	}

	seq_num = t7xx_port_next_rx_seq_num(port, ccci_h);
	port_static = port->port_static;
	ret = port_static->ops->recv_skb(port, skb);
	if (ret && port->flags & PORT_F_RX_ALLOW_DROP) {
		port->seq_nums[MTK_RX] = seq_num;
		dev_err_ratelimited(dev, "Packed drop on port %s, error %d\n",
				    port_static->name, ret);
		goto drop_skb;
	}

	if (ret)
		return ret;

	port->seq_nums[MTK_RX] = seq_num;
	return 0;

drop_skb:
	dev_kfree_skb_any(skb);
	return 0;
}

/**
 * t7xx_port_proxy_md_status_notify() - Notify all ports of state.
 *@port_prox: The port_proxy pointer.
 *@state: State.
 *
 * Called by t7xx_fsm. Used to dispatch modem status for all ports,
 * which want to know MD state transition.
 */
void t7xx_port_proxy_md_status_notify(struct port_proxy *port_prox, unsigned int state)
{
	struct t7xx_port *port;
	int i;

	for_each_proxy_port(i, port, port_prox) {
		struct t7xx_port_static *port_static = port->port_static;

		if (port_static->ops->md_state_notify)
			port_static->ops->md_state_notify(port, state);
	}
}

static void t7xx_proxy_init_all_ports(struct t7xx_modem *md)
{
	struct port_proxy *port_prox = md->port_prox;
	struct t7xx_port *port;
	int i;

	for_each_proxy_port(i, port, port_prox) {
		struct t7xx_port_static *port_static = port->port_static;

		t7xx_port_struct_init(port);

		port->t7xx_dev = md->t7xx_dev;
		port->dev = &md->t7xx_dev->pdev->dev;
		spin_lock_init(&port->port_update_lock);

		if (port->flags & PORT_F_CHAR_NODE_SHOW)
			port->chan_enable = true;
		else
			port->chan_enable = false;

		if (port_static->ops->init)
			port_static->ops->init(port);
	}

	t7xx_proxy_setup_ch_mapping(port_prox);
}

static int t7xx_proxy_alloc(struct t7xx_modem *md)
{
	unsigned int port_number = ARRAY_SIZE(t7xx_md_ports);
	struct device *dev = &md->t7xx_dev->pdev->dev;
	struct t7xx_port *ports_private;
	struct port_proxy *port_prox;
	int i;

	port_prox = devm_kzalloc(dev, sizeof(*port_prox), GFP_KERNEL);
	if (!port_prox)
		return -ENOMEM;

	md->port_prox = port_prox;
	port_prox->dev = dev;
	port_prox->ports_shared = t7xx_md_ports;

	ports_private = devm_kzalloc(dev, sizeof(*ports_private) * port_number, GFP_KERNEL);
	if (!ports_private)
		return -ENOMEM;

	for (i = 0; i < port_number; i++) {
		ports_private[i].port_static = &port_prox->ports_shared[i];
		ports_private[i].flags = port_prox->ports_shared[i].flags;
	}

	port_prox->ports_private = ports_private;
	port_prox->port_number = port_number;
	t7xx_proxy_init_all_ports(md);
	return 0;
};

/**
 * t7xx_port_proxy_init() - Initialize ports.
 * @md: Modem.
 *
 * Create all port instances.
 *
 * Return:
 * * 0		- Success.
 * * -ERROR	- Error code from failure sub-initializations.
 */
int t7xx_port_proxy_init(struct t7xx_modem *md)
{
	int ret;

	ret = t7xx_proxy_alloc(md);
	if (ret)
		return ret;

	t7xx_cldma_set_recv_skb(md->md_ctrl[CLDMA_ID_MD], t7xx_port_proxy_recv_skb);
	return 0;
}

void t7xx_port_proxy_uninit(struct port_proxy *port_prox)
{
	struct t7xx_port *port;
	int i;

	for_each_proxy_port(i, port, port_prox) {
		struct t7xx_port_static *port_static = port->port_static;

		if (port_static->ops->uninit)
			port_static->ops->uninit(port);
	}
}

/**
 * t7xx_port_proxy_node_control() - Create/remove node.
 * @md: Modem.
 * @port_msg: Message.
 *
 * Used to control create/remove device node.
 *
 * Return:
 * * 0		- Success.
 * * -EFAULT	- Message check failure.
 */
int t7xx_port_proxy_node_control(struct t7xx_modem *md, struct port_msg *port_msg)
{
	u32 *port_info_base = (void *)port_msg + sizeof(*port_msg);
	struct device *dev = &md->t7xx_dev->pdev->dev;
	unsigned int version, ports, i;

	version = FIELD_GET(PORT_MSG_VERSION, le32_to_cpu(port_msg->info));
	if (version != PORT_ENUM_VER ||
	    le32_to_cpu(port_msg->head_pattern) != PORT_ENUM_HEAD_PATTERN ||
	    le32_to_cpu(port_msg->tail_pattern) != PORT_ENUM_TAIL_PATTERN) {
		dev_err(dev, "Invalid port control message %x:%x:%x\n",
			version, le32_to_cpu(port_msg->head_pattern),
			le32_to_cpu(port_msg->tail_pattern));
		return -EFAULT;
	}

	ports = FIELD_GET(PORT_MSG_PRT_CNT, le32_to_cpu(port_msg->info));
	for (i = 0; i < ports; i++) {
		struct t7xx_port_static *port_static;
		u32 *port_info = port_info_base + i;
		struct t7xx_port *port;
		unsigned int ch_id;
		bool en_flag;

		ch_id = FIELD_GET(PORT_INFO_CH_ID, *port_info);
		port = t7xx_proxy_get_port_by_ch(md->port_prox, ch_id);
		if (!port) {
			dev_dbg(dev, "Port:%x not found\n", ch_id);
			continue;
		}

		en_flag = !!(PORT_INFO_ENFLG & *port_info);

		if (t7xx_fsm_get_md_state(md->fsm_ctl) == MD_STATE_READY) {
			port_static = port->port_static;

			if (en_flag) {
				if (port_static->ops->enable_chl)
					port_static->ops->enable_chl(port);
			} else {
				if (port_static->ops->disable_chl)
					port_static->ops->disable_chl(port);
			}
		} else {
			port->chan_enable = en_flag;
		}
	}

	return 0;
}
