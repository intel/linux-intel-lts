/*
 * pd_sink.c - USB PD (Power Delivery) sink port state machine driver
 *
 * This driver implements a simple USB PD sink port state machine.
 * It assumes the upper layer, i.e. the user of this driver, handles
 * the PD message receiving and transmitting. The upper layer receives
 * PD messages from the Source, queues them to us, and when processing
 * the received message we'll call upper layer's transmitting function
 * to send PD messages to the source.
 * The sink port state machine is maintained in this driver but we also
 * broadcast some important PD messages to upper layer as events.
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * Author: Bin Gao <bin.gao@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb/pd_sink.h>

#define MAKE_HEADER(port, header, msg, objs) \
do { \
	header->type = msg; \
	header->data_role = PD_DATA_ROLE_UFP; \
	header->revision = port->pd_rev; \
	header->power_role = PD_POWER_ROLE_SINK; \
	header->id = roll_msg_id(port); \
	header->nr_objs = objs; \
	header->extended = PD_MSG_NOT_EXTENDED; \
} while (0)

static struct pd_sink_port *sink_ports[MAX_NR_SINK_PORTS];
static int nr_ports;

BLOCKING_NOTIFIER_HEAD(pd_sink_notifier_list);

static char *state_strings[] = {
	"WAIT_FOR_SOURCE_CAPABILITY",
	"REQUEST_SENT",
	"ACCEPT_RECEIVED",
	"POWER_SUPPLY_READY",
};

/* Control messages */
static char *cmsg_strings[] = {
	"GOODCRC",			/* 1 */
	"GOTOMIN",			/* 2 */
	"ACCEPT",			/* 3 */
	"REJECT",			/* 4 */
	"PING",				/* 5 */
	"PS_RDY",			/* 6 */
	"GET_SRC_CAP",			/* 7 */
	"GET_SINK_CAP",			/* 8 */
	"DR_SWAP",			/* 9 */
	"PR_SWAP",			/* 10 */
	"VCONN_SWAP",			/* 11 */
	"WAIT",				/* 12 */
	"SOFT_RESET",			/* 13 */
	"RESERVED",			/* 14 */
	"RESERVED",			/* 15 */
	"NOT_SUPPORTED",		/* 16 */
	"GET_SOURCE_CAP_EXTENDED",	/* 17 */
	"GET_STATUS",			/* 18 */
	"FR_SWAP",			/* 19 */
	/* RESERVED			20 - 31 */
};

/* Data messages */
static char *dmsg_strings[] = {
	"SOURCE_CAP",		/* 1 */
	"REQUEST",		/* 2 */
	"BIST",			/* 3 */
	"SINK_CAP",		/* 4 */
	"BATTERY_STATUS",	/* 5 */
	"ALERT",		/* 6 */
	"RESERVED",		/* 7 */
	"RESERVED",		/* 8 */
	"RESERVED",		/* 9 */
	"RESERVED",		/* 10 */
	"RESERVED",		/* 11 */
	"RESERVED",		/* 12 */
	"RESERVED",		/* 13 */
	"RESERVED",		/* 14 */
	"VENDOR_DEFINED",	/* 15 */
	/* RESERVED		16 - 31 */
};

static char *state_to_string(enum pd_sink_state state)
{
	if (state < ARRAY_SIZE(state_strings))
		return state_strings[state];
	else
		return NULL;
}

static char *msg_to_string(bool is_cmsg, u8 msg)
{
	int nr = is_cmsg ? ARRAY_SIZE(cmsg_strings) : ARRAY_SIZE(dmsg_strings);

	if (msg <= nr)
		return is_cmsg ? cmsg_strings[msg - 1] : dmsg_strings[msg - 1];
	else
		return "RESERVED";
}

static void print_message(int port, bool is_cmsg, u8 msg, bool recv)
{
	pr_info("sink port %d: %s message %s %s\n", port,
				is_cmsg ? "Control" : "Data",
				msg_to_string(is_cmsg, msg),
		 recv ? "received" : "sent(wait GOODCRC)");
}

static inline bool fixed_ps_equal(struct sink_ps *p1,
				struct sink_ps *p2)
{
	return p1->ps_type == p2->ps_type &&
		p1->ps_fixed.voltage_fixed == p2->ps_fixed.voltage_fixed &&
		p1->ps_fixed.current_default == p2->ps_fixed.current_default;
}

/* The message ID increments each time we send out a new message */
static u8 roll_msg_id(struct pd_sink_port *port)
{
	u8 msg_id = port->msg_id;

	if (msg_id == PD_MSG_ID_MAX)
		msg_id = PD_MSG_ID_MIN;
	else
		msg_id++;

	port->msg_id = msg_id;
	return msg_id;
}

static inline bool is_waiting_goodcrc(struct pd_sink_port *port, int msg_id)
{
	return (port->waiting_goodcrc) && (msg_id == port->last_sent_msg);
}

static inline void clear_waiting_goodcrc(struct pd_sink_port *port)
{
	port->waiting_goodcrc = 0;
}

static void start_timer(struct pd_sink_port *port, int timeout,
		enum hrtimer_restart (*f)(struct hrtimer *))
{
	if (hrtimer_active(&port->tx_timer)) {
		pr_err("Error: previous timer is still active\n");
		return;
	}

	port->tx_timer.function = f;
	/* timeout comes with ms but ktime_set takes seconds and nanoseconds */
	hrtimer_start(&port->tx_timer, ktime_set(timeout / 1000,
		(timeout % 1000) * 1000000), HRTIMER_MODE_REL);
}

static inline void stop_timer(struct pd_sink_port *port)
{
	hrtimer_cancel(&port->tx_timer);
}

static enum hrtimer_restart goodcrc_timeout(struct hrtimer *timer)
{
	pr_err("GOODCRC message is not received in %d ms: timeout\n",
						PD_TIMEOUT_GOODCRC);
	return HRTIMER_NORESTART;
}

/*
 * For any message we send, we must get a GOODCRC message from the Source.
 * The USB PD spec says the time should be measured between the last bit
 * of the sending message's EOP has been transmitted and the last bit of
 * the receiving GOODCRC message's EOP has been received. The allowed time
 * is minimal 0.9 ms and maximal 1.1 ms. However, this measurement is
 * performed in physical layer. When it reaches to the OS and this driver,
 * the actual time is difficult to predict because of the scheduling,
 * context switch, interrupt preemption and nesting, etc. So we only define
 * a safe timeout value (PD_TIMEOUT_GOODCRC) which is large enough to take
 * account of all software related latency.
 */
static int send_message(struct pd_sink_port *port, void *buf, int len,
			u8 msg, bool ctrl_msg, enum sop_type sop_type)
{
	if (ctrl_msg && msg == PD_CMSG_GOODCRC) {
		port->tx(port->port, port->tx_priv, buf, len, sop_type);
		print_message(port->port, true, msg, false);
		return 0;
	}

	port->tx(port->port, port->tx_priv, buf, len, sop_type);
	print_message(port->port, ctrl_msg, msg, false);

	if (!port->hw_goodcrc_rx) {
		port->waiting_goodcrc = true;
		start_timer(port, PD_TIMEOUT_GOODCRC, goodcrc_timeout);
	}

	port->last_sent_msg = msg;
	port->last_msg_ctrl = ctrl_msg;

	return 0;
}

static void ack_message(struct pd_sink_port *port, int msg_id)
{
	struct pd_msg_header *header = kzalloc(PD_MSG_HEADER_LEN, GFP_KERNEL);

	if (!header) {
		MAKE_HEADER(port, header, PD_CMSG_GOODCRC, 0);
		send_message(port, header, PD_MSG_HEADER_LEN,
				PD_CMSG_GOODCRC, true, SOP);
}	}

static int handle_goodcrc(struct pd_sink_port *port, u8 msg_id)
{
	if (is_waiting_goodcrc(port, msg_id)) {
		hrtimer_cancel(&port->tx_timer);
		clear_waiting_goodcrc(port);
		switch (msg_id) {
		case PD_DMSG_REQUEST:
			port->state = PD_SINK_STATE_REQUEST_SENT;
			break;
		case PD_DMSG_SINK_CAP:
			pr_info("Got GOODCRC for SINK_CAPABILITY message\n");
		default:
			break;
		}
	} else
		pr_warn("Unexpected GOODCRC message received.\n");

	return 0;
}

static void handle_accept(struct pd_sink_port *port)
{
	enum pd_sink_state state = port->state;

	if (state != PD_SINK_STATE_REQUEST_SENT) {
		pr_err("ACCEPT received but not expected, sink state: %s\n",
						state_to_string(state));
		return;
	}

	port->state = PD_SINK_STATE_ACCEPT_RECEIVED;
}

/*
 * The Source may send REJECT message as a response to REQUEST, PR_SWAP,
 * DR_SWAP or VCONN_SWAP message. Since we only send REQUEST to the Source
 * so we're sure the REJECT is for our REQUEST message.
 */
static void handle_reject(struct pd_sink_port *port)
{
	enum pd_sink_state state = port->state;

	if (state == PD_SINK_STATE_REQUEST_SENT)
		/* Broadcast to upper layer */
		blocking_notifier_call_chain(&pd_sink_notifier_list,
				PD_SINK_EVENT_REJECT | port->port << 8, NULL);
	else
		pr_err("REJECT received but not expected, sink state: %s\n",
						state_to_string(state));
}

static void handle_not_supported(struct pd_sink_port *port)
{
	pr_err("sink port %d: %s message %s is not supported by Source\n",
			port->port, port->last_msg_ctrl ? "Control" : "Data",
				msg_to_string(port->last_msg_ctrl,
			port->last_sent_msg & PD_MSG_TYPE_MASK));
}

/*
 * The Wait Message is a valid response to a Request, a PR_Swap, DR_Swap or
 * VCONN_Swap Message. We don't support PR_Swap, DR_Swap and VCONN_Swap
 * messages so we only handle Wait message from Source responding to our
 * Request message.
 */
static void handle_wait(struct pd_sink_port *port)
{
	pr_info("WAIT message received\n");
}

/*
 * We repond the GET_SINK_CAPABILITY message by sending the SINK_CAPABILITY
 * message. The PDOs in the SINK_CAPABILITY message shall be in the following
 * order:
 * 1. The vSafe5V Fixed Supply Object shall always be the first object.
 * 2. The remaining Fixed Supply Objects, if present, shall be sent in voltage
 *    order; lowest to highest.
 * 3. The Battery Supply Objects, if present shall be sent in Minimum Voltage
 *    order; lowest to highest.
 * 4. The Variable Supply (non-Battery) Objects, if present, shall be sent in
 *    Minimum Voltage order; lowest to highest.
 *
 * The upper layer calling pd_sink_register_port() must ensure the profiles
 * provided come with the above order as we don't re-order the profiles here
 * when we compose the SINK_CAPABILITY message.
 */
static void handle_get_sink_cap(struct pd_sink_port *port)
{
	u8 *pdo;
	int i, nr_objs = 0;
	struct sink_ps *ps;
	struct pd_pdo_sink_fixed *fixed;
	struct pd_pdo_variable *variable;
	struct pd_pdo_battery *battery;
	struct pd_msg_header *header = kzalloc(PD_MSG_HEADER_LEN +
		port->nr_ps * PD_OBJ_SIZE, GFP_KERNEL);

	if (!header) {
		pr_crit("%s(): kzalloc() failed\n", __func__);
		return;
	}

	MAKE_HEADER(port, header, PD_DMSG_SINK_CAP, port->nr_ps);
	pdo = (u8 *)header + PD_MSG_HEADER_LEN;

	for (i = 0; i < port->nr_ps; i++) {
		ps = port->ps_reqs + i;
		switch (ps->ps_type) {
		case PS_TYPE_FIXED:
			fixed = (struct pd_pdo_sink_fixed *)pdo;
			fixed->current_max = ps->ps_fixed.current_max;
			fixed->voltage = ps->ps_fixed.voltage_fixed;
			fixed->fast_role_swap = FAST_ROLE_SWAP_DISABLE;
			fixed->usb_capable = 1;
			fixed->higher_capability =
				(ps->ps_fixed.voltage_fixed >
						PD_VSAFE5V) ? 1 : 0;
			fixed->type = PS_TYPE_FIXED;
			break;
		case PS_TYPE_BATTERY:
			battery = (struct pd_pdo_battery *)pdo;
			battery->power = ps->ps_battery.power;
			battery->voltage_max = ps->ps_battery.voltage_max;
			battery->voltage_min = ps->ps_battery.voltage_min;
			battery->type = PS_TYPE_BATTERY;
			break;
		case PS_TYPE_VARIABLE:
			variable = (struct pd_pdo_variable *)pdo;
			variable->current_mo =
				ps->ps_variable.current_default;
			variable->voltage_max = ps->ps_variable.voltage_max;
			variable->voltage_min = ps->ps_variable.voltage_min;
			variable->type = PS_TYPE_VARIABLE;
			break;
		default:
			continue;
		}

		pdo += PD_OBJ_SIZE;
		nr_objs++;
	}

	if (nr_objs == 0) {
		kfree(header);
		pr_err("no valid sink PDOs to send in SINK_CAPABILITY\n");
		return;
	}

	header->nr_objs = nr_objs;
	send_message(port, header, PD_MSG_HEADER_LEN + nr_objs * PD_OBJ_SIZE,
						PD_DMSG_SINK_CAP, false, SOP);
}

static void handle_ps_ready(struct pd_sink_port *port)
{
	if (port->state != PD_SINK_STATE_ACCEPT_RECEIVED) {
		pr_err("PS_READY received but sink state is not in %s\n",
			state_to_string(PD_SINK_STATE_ACCEPT_RECEIVED));
		return;
	}

	/* Broadcast to upper layer */
	blocking_notifier_call_chain(&pd_sink_notifier_list,
		PD_SINK_EVENT_PS_READY | port->port << 8, NULL);
}

static void handle_gotomin(struct pd_sink_port *port)
{
	/* Broadcast to upper layer */
	blocking_notifier_call_chain(&pd_sink_notifier_list,
		PD_SINK_EVENT_GOTOMIN | port->port << 8, NULL);
}

static void handle_soft_reset(struct pd_sink_port *port)
{
	struct pd_msg_header *header = kzalloc(PD_MSG_HEADER_LEN, GFP_KERNEL);

	if (!header)
		return;

	flush_workqueue(port->rx_wq);
	stop_timer(port);
	port->msg_id = PD_MSG_ID_MIN;
	port->state = PD_SINK_STATE_WAIT_FOR_SRC_CAP;
	MAKE_HEADER(port, header, PD_CMSG_ACCEPT, 0);
	send_message(port, header, PD_MSG_HEADER_LEN,
			PD_CMSG_ACCEPT, true, SOP);

	/* Broadcast to upper layer */
	blocking_notifier_call_chain(&pd_sink_notifier_list,
		PD_SINK_EVENT_SOFT_RESET | port->port << 8, NULL);
}

static void ctl_msg_handler(struct pd_sink_port *port, u8 msg_type)
{
	print_message(port->port, true, msg_type, true);

	switch (msg_type) {
	case PD_CMSG_ACCEPT:
		handle_accept(port);
		break;
	case PD_CMSG_GOTOMIN:
		handle_gotomin(port);
		break;
	case PD_CMSG_REJECT:
		handle_reject(port);
		break;
	case PD_CMSG_GET_SINK_CAP:
		handle_get_sink_cap(port);
		break;
	case PD_CMSG_PS_RDY:
		handle_ps_ready(port);
		break;
	case PD_CMSG_SOFT_RESET:
		handle_soft_reset(port);
		break;
	case PD_CMSG_NOT_SUPPORTED:
		handle_not_supported(port);
		break;
	case PD_CMSG_WAIT:
		/*
		 * Sent by Source to Sink to indicate it can't meet the
		 * requirement for the Request. The Sink is allowed to
		 * repeat the Request Message using the SinkRequestTimer
		 * to ensure that there is tSinkRequest between requests.
		 */
		handle_wait(port);
		break;
	/* Below messages are simply ignored */
	case PD_CMSG_GET_SOURCE_CAP_EXTENDED:
	case PD_CMSG_GET_STATUS:
	case PD_CMSG_PING:
	case PD_CMSG_GET_SRC_CAP:
	case PD_CMSG_DR_SWAP:
	case PD_CMSG_PR_SWAP:
	case PD_CMSG_VCONN_SWAP:
	case PD_CMSG_FR_SWAP:
		break;

	}
}

/**
 * send_request() - send REQUEST message to Source.
 * @port: the sink port number
 * Return: none
 */
static int send_request(struct pd_sink_port *port)
{
	int i;
	struct pd_request *request;
	struct pd_pdo_src_fixed *src;
	struct sink_ps *ps =
		&port->ps_reqs[port->active_ps];
	struct pd_source_cap *cap;
	struct pd_msg_header *header = kzalloc(PD_MSG_HEADER_LEN +
					PD_OBJ_SIZE, GFP_KERNEL);
	bool voltage_matched = false, current_matched = false;

	/* Only support fixed PDO for now */
	if (ps->ps_type != PS_TYPE_FIXED) {
		pr_err("%s(): We only support fixed PDO now\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < port->nr_source_caps; i++) {
		cap = &port->source_caps[i];
		if (cap->ps_type != PS_TYPE_FIXED)
			continue;
		src = &cap->fixed;
		if (src->voltage != ps->ps_fixed.voltage_fixed)
			continue;
		voltage_matched = true;
		/* We matched the voltage, now match the current */
		if (src->current_max >= ps->ps_fixed.current_max) {
			current_matched = true;
			break;
		}
	}

	if (!voltage_matched) {
		pr_err("Can't match any PDO from source caps\n");
		return -EINVAL;
	}

	MAKE_HEADER(port, header, PD_DMSG_REQUEST, 1);
	request = (struct pd_request *)((u8 *)header + PD_MSG_HEADER_LEN);
	request->pos = i + 1; /* PD protocol RDO index starts from 1 */
	request->give_back = port->support_gotomin ? 1 : 0;
	request->usb_comm_capable = port->usb_comm_capable ? 1 : 0;
	if (port->pd_rev == PD_REVISION_3)
		request->unchunked_ext_msg_supported = 1;

	/* If current_matched is false, we have to set the mismatch flag */
	if (!current_matched)
		request->cap_mismatch = 1;

	request->operating_default = ps->ps_fixed.current_default;
	request->operating_min_max = ps->ps_fixed.current_max;

	return send_message(port, header, PD_MSG_HEADER_LEN + PD_OBJ_SIZE,
						PD_DMSG_REQUEST, false, SOP);
}

static void handle_source_cap(struct pd_sink_port *port, u8 msg_revision,
						u8 nr_objs, u8 *buf)
{
	int i;
	u32 *obj;
	u8 type;
	struct pd_source_cap *cap = port->source_caps;

	/*
	 * The PD spec revision included in SOURCE_CAPABILITY message is the
	 * highest revision that the Source supports.
	 */
	port->pd_rev = msg_revision;

	/*
	 * First we need to save all PDOs - they may be used in the future.
	 * USB PD spec says we must use PDOs in the most recent
	 * SOURCE_CAPABILITY message. Here we replace old PDOs with new ones.
	 */
	port->nr_source_caps = 0;
	for (i = 0; i < nr_objs; i++) {
		obj = (u32 *)(buf + i * PD_OBJ_SIZE);
		type = (*obj >> SOURCE_CAP_TYPE_BIT) & SOURCE_CAP_TYPE_MASK;
		switch (type) {
		case PS_TYPE_FIXED:
			cap->ps_type = PS_TYPE_FIXED;
			cap->fixed = *(struct pd_pdo_src_fixed *)obj;
			break;
		case PS_TYPE_VARIABLE:
			cap->ps_type = PS_TYPE_VARIABLE;
			cap->variable = *(struct pd_pdo_variable *)obj;
			break;
		case PS_TYPE_BATTERY:
			cap->ps_type = PS_TYPE_BATTERY;
			cap->battery = *(struct pd_pdo_battery *)obj;
			break;
		default: /* shouldn't come here */
			pr_err("Invalid Source Capability type: %u.\n", type);
			continue;
		}
		port->nr_source_caps++;
		cap++;
	}

	if (port->nr_source_caps == 0) {
		pr_err("There is no valid PDOs in SOURCE_CAPABILITY message\n");
		return;
	}

	/* If a contract is not established, we need send a REQUEST message */
	if (port->state == PD_SINK_STATE_WAIT_FOR_SRC_CAP) {
		if (!send_request(port))
			port->state = PD_SINK_STATE_REQUEST_SENT;
	}
}

static void handle_alert(struct pd_sink_port *port, int nr_objs, u8 *buf)
{
	u8 type;
	struct alert_msg *alert;

	if (nr_objs != 1) {
		pr_err("Invalid ALERT message\n");
		return;
	}

	alert = (struct alert_msg *)buf;
	type = alert->type;

	pr_info("ALERT received on port %d:\n"
		"Battery Status Changes: %u\nOCP: %u\nOTP: %u\nOCC: %u\n"
		"Source Input Change: %u\nOVP: %u\n", port->port,
		type & PD_ALERT_BATTERY ? 1 : 0,
		type & PD_ALERT_OCP ? 1 : 0,
		type & PD_ALERT_OTP ? 1 : 0,
		type & PD_ALERT_OCC ? 1 : 0,
		type & PD_ALERT_SRC_INPUT ? 1 : 0,
		type & PD_ALERT_OVP ? 1 : 0);

	if (type & PD_ALERT_BATTERY)
		pr_info("Battery Status Change: Fixed Batteries 0x%x\n"
			"Battery Status Change: Hot Swappable Batteries 0x%x\n",
			alert->fixed_batteries, alert->hot_swappable_batteries);

	/* Broadcast to upper layer */
	blocking_notifier_call_chain(&pd_sink_notifier_list,
		PD_SINK_EVENT_ALERT | port->port << 8, buf);
}

static void data_msg_handler(struct pd_sink_port *port, u8 msg_revision,
					u8 msg_type, u8 nr_objs, u8 *buf)
{
	print_message(port->port, false, msg_type, true);

	switch (msg_type) {
	case PD_DMSG_SOURCE_CAP:
		/* Save source's power supply capability */
		handle_source_cap(port, msg_revision, nr_objs, buf);
		break;
	case PD_DMSG_REQUEST:
	case PD_DMSG_SINK_CAP:
		/* Sink shouldn't receive REQUEST, ignore */
		pr_warn("Ignored REQUEST message.\n");
		break;
	case PD_DMSG_BIST:
		pr_warn("Ignored BIST message.\n");
		break;
	case PD_DMSG_BATTERY_STATUS:
		/*
		 * The Battery_Status Message shall be sent in response
		 * to a Get_Battery_Status Message. We(Sink) don't send
		 * Get_Battery_Status message so we won't receive
		 * Battery_Status message.
		 */
		pr_warn("Ignored BATTERY_STATUS message.\n");
		break;
	case PD_DMSG_ALERT:
		handle_alert(port, nr_objs, buf);
		break;
	case PD_DMSG_VENDOR_DEFINED:
		pr_warn("Ignored VENDOR_DEFINED message.\n");
		break;
	default:
		pr_err("Unknown data message type: %u\n", msg_type);
	}
}

static inline bool is_extended_msg(struct pd_msg_header *header)
{
	return header->extended == PD_MSG_EXTENDED;
}

static inline bool is_ctrl_msg(struct pd_msg_header *header)
{
	return header->nr_objs == 0;
}

static inline bool is_data_msg(struct pd_msg_header *header)
{
	return header->nr_objs != 0;
}

static inline void free_msg(struct pd_sink_msg *msg)
{
	kfree(msg);
}

static int msg_handler(struct pd_sink_msg *msg)
{
	struct pd_sink_port *port;
	struct pd_msg_header *header;
	int len, ret = 0;

	len = msg->msg_len;
	if (len < PD_MSG_HEADER_LEN) {
		pr_err("Invalid message: len=%d(should be at least 2)\n", len);
		ret = -EINVAL;
		goto out;
	}

	port = sink_ports[msg->port];
	header = (struct pd_msg_header *)msg->buf;

	if (is_ctrl_msg(header) && (header->type == PD_CMSG_GOODCRC))
		handle_goodcrc(port, header->id);

	/*
	 * If HW supports auto GOODCRC generation and transmitting then we
	 * don't need to send GOODCRC by software. This is indicated by
	 * the .hw_goodcrc_tx field of the pd_sink_port structure.
	 */
	if (is_data_msg(header) || (is_ctrl_msg(header) &&
			(header->type != PD_CMSG_GOODCRC))) {
		if (!port->hw_goodcrc_tx)
			ack_message(port, header->id);
	}

	if (is_extended_msg(header)) {
		pr_err("Extended message received, but no support for it\n");
		goto out;
	}

	if (header->power_role != PD_POWER_ROLE_SOURCE) {
		pr_err("Message is not from a Source, ignored\n");
		goto out;
	}

	if (header->nr_objs == 0)
		ctl_msg_handler(port, header->type);
	else {
		if (PD_MSG_HEADER_LEN + header->nr_objs * PD_OBJ_SIZE != len) {
			pr_err("Invalid message: unexpected length.\n");
			ret = -EINVAL;
			goto out;
		}
		data_msg_handler(port, header->revision, header->type,
			header->nr_objs, msg->buf + PD_MSG_HEADER_LEN);
	}

out:
	free_msg(msg);
	return ret;
}

static void rx_msg_worker(struct work_struct *work)
{
	unsigned long flags;
	struct pd_sink_msg *msg, *n;
	struct pd_sink_port *port = container_of(work,
			struct pd_sink_port, rx_work);

	/*
	 * We process all rx messages in the list.
	 * Lock is held only when operating the list.
	 */
	spin_lock_irqsave(&port->rx_lock, flags);
	list_for_each_entry_safe(msg, n, &port->rx_list, list) {
		list_del(&msg->list);
		spin_unlock_irqrestore(&port->rx_lock, flags);
		msg_handler(msg);
		spin_lock_irqsave(&port->rx_lock, flags);
	}
	spin_unlock_irqrestore(&port->rx_lock, flags);
}

int pd_sink_register_port(struct pd_sink_profile *profile,
	int (*tx)(int port, void *tx_priv, void *buf, int len,
			enum sop_type sop_type), void *tx_priv)
{
	int i, nr_ps;
	struct pd_sink_port *port;

	if (nr_ports >= MAX_NR_SINK_PORTS) {
		pr_err("Can't register sink port: maximum number of ports reached\n");
		return -EINVAL;
	}

	if (!profile) {
		pr_err("%s(): profiles pointer can't be NULL\n", __func__);
		return -EINVAL;
	}

	/*
	 * USB PD spec says in SINK_CAPABILITY message(as a response to
	 * GET_SINK_CAPABILITY message) the Sink shall include one Power
	 * Data Object that reports vSafe5V even if the sink requires
	 * additional power to operate fully. So we'll always have the
	 * vSafe5V PDO as ps_reqs[0] unless the Sink provides the exact
	 * same PDO with vSafe5V.
	 */
	nr_ps = profile->nr_ps;
	if (nr_ps <= 0) {
		pr_err("At least 1 Sink power supply requirement required\n");
		return -EINVAL;
	}

	if (nr_ps >= MAX_NR_SINK_PS_REQS) {
		pr_warn("Shrink Sink power supply requirements size %d to %d\n",
						nr_ps, MAX_NR_SINK_PS_REQS);
		nr_ps = MAX_NR_SINK_PS_REQS;
	}

	if (profile->active_ps <= 0 || profile->active_ps >=
					 MAX_NR_SINK_PS_REQS) {
		pr_err("Invalid active_ps in the profile\n");
		return -EINVAL;
	}

	port = kmalloc(sizeof(struct pd_sink_port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->nr_ps = nr_ps;
	port->hw_goodcrc_tx = profile->hw_goodcrc_tx;
	port->hw_goodcrc_rx = profile->hw_goodcrc_rx;
	port->support_gotomin = profile->gotomin;
	port->usb_comm_capable = profile->usb_comm;
	port->pd_rev = profile->pd_rev;
	port->usb_spec = profile->spec;
	port->last_sent_msg = 0;
	port->waiting_goodcrc = false;
	port->tx = tx;
	port->tx_priv = tx_priv;
	port->active_ps = profile->active_ps;
	port->state = PD_SINK_STATE_WAIT_FOR_SRC_CAP;

	for (i = 0; i < nr_ps; i++)
		port->ps_reqs[i] = *(profile->ps + i);

	port->msg_id = PD_MSG_ID_MIN;
	hrtimer_init(&port->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	INIT_LIST_HEAD(&port->rx_list);
	INIT_WORK(&port->rx_work, rx_msg_worker);
	spin_lock_init(&port->rx_lock);

	port->rx_wq = alloc_workqueue("sink_wq: port %d",
				WQ_HIGHPRI, 0, nr_ports);
	if (!port->rx_wq) {
		pr_err(" Failed to allocate workqueue for sink port\n");
		kfree(port);
		return -EINVAL;
	}

	port->port = nr_ports;
	sink_ports[nr_ports] = port;
	nr_ports++;

	return port->port;
}
EXPORT_SYMBOL_GPL(pd_sink_register_port);

int pd_sink_unregister_port(int port)
{
	struct pd_sink_port *pport;

	if (port < 0 || port >= MAX_NR_SINK_PORTS) {
		pr_err("Invalid port number\n");
		return -EINVAL;
	}

	pport = sink_ports[port];
	flush_workqueue(pport->rx_wq);
	destroy_workqueue(pport->rx_wq);
	stop_timer(pport);
	kfree(pport);
	sink_ports[port] = NULL;
	nr_ports--;

	return 0;
}
EXPORT_SYMBOL_GPL(pd_sink_unregister_port);

struct pd_sink_msg *pd_sink_alloc_msg(int port, int msg_len)
{
	struct pd_sink_msg *msg = kmalloc(sizeof(struct pd_sink_msg) + msg_len,
								GFP_KERNEL);
	if (!msg)
		return NULL;

	msg->port = port;
	msg->msg_len = msg_len;
	msg->buf = (u8 *)msg + sizeof(struct pd_sink_msg);

	return msg;
}
EXPORT_SYMBOL_GPL(pd_sink_alloc_msg);

/*
 * pd_queue_msg() may be called in an interrupt handler, so we use
 * spin lock to protect the message list.
 */
int pd_sink_queue_msg(struct pd_sink_msg *msg)
{
	unsigned long flags;
	struct pd_sink_port *port;

	if (msg->port < 0 || msg->port >= MAX_NR_SINK_PORTS) {
		pr_err("Invalid port number\n");
		return -EINVAL;
	}

	port = sink_ports[msg->port];

	spin_lock_irqsave(&port->rx_lock, flags);
	list_add_tail(&msg->list, &port->rx_list);
	spin_unlock_irqrestore(&port->rx_lock, flags);

	queue_work(port->rx_wq, &port->rx_work);

	return 0;
}
EXPORT_SYMBOL_GPL(pd_sink_queue_msg);

/*
 * Upper layer requests to change to a different power supply from Source.
 * We send a new REQUEST message to the Source and the upper layer can start
 * using the new power supply once PD_SINK_EVENT_PS_READY event is received.
 */
int pd_sink_change_ps(int port, int ps)
{
	struct pd_sink_port *pport;

	if (ps < 0 || ps >= MAX_NR_SINK_PS_REQS) {
		pr_err("Invalid power supply index\n");
		return -EINVAL;
	}

	pport =	sink_ports[port];
	pport->active_ps = ps;

	if (!send_request(pport))
		pport->state = PD_SINK_STATE_REQUEST_SENT;

	return 0;
}
EXPORT_SYMBOL_GPL(pd_sink_change_ps);

/* Reset the port to initial state(PD_SINK_STATE_WAIT_FOR_SRC_CAP) */
int pd_sink_reset_state(int port)
{
	struct pd_sink_port *pport;

	if (port < 0 || port >= MAX_NR_SINK_PORTS) {
		pr_err("Invalid port number");
		return -EINVAL;
	}

	pport = sink_ports[port];
	flush_workqueue(pport->rx_wq);

	if (!pport->hw_goodcrc_rx) {
		stop_timer(pport);
		clear_waiting_goodcrc(pport);
	}

	pport->state = PD_SINK_STATE_WAIT_FOR_SRC_CAP;

	return 0;
}
EXPORT_SYMBOL_GPL(pd_sink_reset_state);

int pd_sink_tx_complete(int port)
{
	struct pd_sink_port *pport;

	if (port < 0 || port >= MAX_NR_SINK_PORTS) {
		pr_err("Invalid port number");
		return -EINVAL;
	}

	pport = sink_ports[port];

	if (!pport->hw_goodcrc_rx && pport->last_msg_ctrl &&
		pport->last_sent_msg == PD_CMSG_GOODCRC) {
		stop_timer(pport);
		clear_waiting_goodcrc(pport);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pd_sink_tx_complete);
