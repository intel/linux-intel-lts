/*
 * pd_sink.h - USB PD Sink port state machine header file.
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

#ifndef __USB_PD_SINK_H
#define __USB_PD_SINK_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/usb/pd_message.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>

#define MAX_NR_SINK_PORTS	4
#define MAX_NR_SINK_PS_REQS	6
#define MAX_NR_SOURCE_CAPS	6

enum usb_spec {
	USB_SPEC_1X = 1,
	USB_SPEC_2X,
	USB_SPEC_3X,
};

enum pd_sink_state {
	PD_SINK_STATE_WAIT_FOR_SRC_CAP,
	PD_SINK_STATE_REQUEST_SENT,
	PD_SINK_STATE_ACCEPT_RECEIVED,
	PD_SINK_STATE_PS_READY,
};

enum pd_sink_event {
	PD_SINK_EVENT_PS_READY,	/* PS_READY message received */
	PD_SINK_EVENT_GOTOMIN,	/* GOTOMIN message received */
	PD_SINK_EVENT_REJECT,	/* REJECT message received */
	PD_SINK_EVENT_ALERT,	/* ALERT message received */
	PD_SINK_EVENT_SOFT_RESET,	/* SOFT_RESET message received */
};

enum pd_tx_state {
	PD_TX_SUCCESS, /* PD PHY transmission succeeded */
	PD_TX_FAILURE, /* PD PHY transmission failed */
};

/**
 * struct pd_sink_msg - a structure describing a PD message
 * @port:	Sink port index
 * @buf:	pointer of the buffer having the message data
 * @msg_len:	length of the message
 * @sop_type:	SOP type - SOP, SOP' or SOP''
 * @list:	the list_head of the message list
 */
struct pd_sink_msg {
	int port;
	u8 *buf;
	int msg_len;
	enum sop_type sop_type;
	struct list_head list;
};

/* Sink port's power supply requirement */
struct sink_ps {
	u8 ps_type; /* fixed, variable or battery */
	union {
		struct ps_fixed {
			u16 voltage_fixed; /* Fixed voltage in 50mV units */
			/* Operational current in 10mA units */
			u16 current_default;
			/* Maximum current in 10mA units */
			u16 current_max;
		} ps_fixed;
		struct ps_variable {
			u16 voltage_max; /* Maximum voltage in 50mV units */
			u16 voltage_min; /* Minimum voltage in 50mV units */
			/* Operational current in 10mA units */
			u16 current_default;
		} ps_variable;
		struct ps_battery {
			u16 voltage_max; /* Maximum voltage in 50mV units */
			u16 voltage_min; /* Minimum voltage in 50mV units */
			u16 power; /* Operational power in 250mW units */
		} ps_battery;
	};
};

/* We save PDOs in SOURCE_CAPABILITY message into pd_source_cap structure */
struct pd_source_cap {
	u8 ps_type; /* fixed, variable or battery */
	union {
		struct pd_pdo_src_fixed fixed;
		struct pd_pdo_variable variable;
		struct pd_pdo_battery battery;
	};
};

/**
 * struct pd_sink_profile - a structure describing a Sink port's profile.
 * @hw_goodcrc_tx: outgoing GOODCRC is sent by HW automatically
 * @hw_goodcrc_rx: incoming GOODCRC is received and checked by HW, i.e.
 *                 GOODCRC message won't be reported to software
 * @gotomin: true if the port supports GOTOMIN message
 * @usb_comm: true if the port is usb data communication capable
 * @pd_rev: PD revision(rev 1.0, 2.0 or 3.0) the hardware supports
 * @spec: highest USB spec(1.x, 2.x or 3.x) the port supports
 * @ps: An array of power supply requirements
 * @nr_ps: size of the power supply requirement array
 * @active_ps: index to the array, indicating the power supply that will
 *	sent in the REQUEST message
 */
struct pd_sink_profile {
	bool hw_goodcrc_tx;
	bool hw_goodcrc_rx;
	bool gotomin;
	bool usb_comm;
	u8 pd_rev;
	enum usb_spec spec;
	struct sink_ps *ps;
	int nr_ps;
	int active_ps;
};

/**
 * struct pd_sink_port - the Sink port structure, used by PD sink only,
 *	not exposed to upper layer(e.g. type-C PHY driver).
 * @hw_goodcrc_tx: outgoing GOODCRC is sent by HW automatically
 * @hw_goodcrc_rx: incoming GOODCRC is received and checked by HW, i.e.
 *                 GOODCRC message won't be reported to software
 * @support_gotomin: the port will take action, e.g. reduce current
 *	drawing, when a GOTOMIN message is received
 * @usb_comm_capable: the port is capable of USB data communication
 * @pd_rev: PD revision(rev 1.0, 2.0 or 3.0) the hardware supports
 * @usb_spec: the highest USB spec the port supports(USB 1.X, 2.X or 3.X)
 * @ps_reqs[]: an array of power supply requirements of the port
 * @nr_ps: size of ps_reqs[]
 * @active_ps: index to ps_reqs[] indicating the perferred power supply
 *	requirement that will be used for the REQUEST message. It can be
 *	change later by calling pd_sink_change_ps().
 * @port: the port number (0 based) assigned by the PD sink driver
 * @last_sent_msg: the most recent message(message type) that was sent
 * @last_msg_ctrl: true if last_sent_msg is a control message
 * @msg_id: message ID of last_sent_msg
 * @state: port state, see the structrue enum pd_sink_state
 * @waiting_goodcrc: true if a message was sent but a GOODCRC for that
 *	message is not received yet
 * @source_caps[]: power supply capability extracted from SOURCE_CAPABILITY
 *	message sent by the Source
 * @nr_source_caps: size of the soruce_caps[] array
 * @rx_wq: workqueue for handing received messages
 * @rx_work: the work struct for handing received messages
 * @rx_list: message list head
 * @tx_timer: a timer for monitoring transmission timeout
 * @tx: the PD message transmission function, provided by upper layer
 *	during port registration
 * @tx_priv: a private data pointer provided by upper layer. This pointer
 *	will be passed down to the tx() function
 */
struct pd_sink_port {
	bool hw_goodcrc_tx;
	bool hw_goodcrc_rx;
	bool support_gotomin;
	bool usb_comm_capable;
	u8 pd_rev;
	enum usb_spec usb_spec;
	struct sink_ps ps_reqs[MAX_NR_SINK_PS_REQS];
	int nr_ps;
	int active_ps;
	int port;
	u8 last_sent_msg;
	bool last_msg_ctrl;
	u16 msg_id;
	int state;
	bool waiting_goodcrc;
	struct pd_source_cap source_caps[MAX_NR_SOURCE_CAPS];
	int nr_source_caps;
	struct workqueue_struct *rx_wq;
	struct work_struct rx_work;
	spinlock_t rx_lock;
	struct list_head rx_list;
	struct hrtimer tx_timer;
	int (*tx)(int port, void *tx_priv, void *buf, int len,
					enum sop_type sop_type);
	void *tx_priv;
};

/**
 * pd_sink_register_port() - Function to register a Sink port
 * @profile: pointer to a pd_sink_profile structure describing the port, see
 *	struct pd_sink_profile {} for details
 * @tx: the function for transmitting PD messages
 * @tx_priv: a private data pointer that is required by the tx function. This
 *	pointer will be passed down to the tx function when tx() is called.
 *
 * The type-C phy driver calls this function to register a Sink port.
 * PD message receiving interrupt can be enabled only after this function
 * call is returned.
 *
 * Return: a port number(>=0) if succeeded or a negative number if failed
 */
int pd_sink_register_port(struct pd_sink_profile *profile,
		int (*tx)(int port, void *tx_priv, void *buf, int len,
				enum sop_type sop_type), void *tx_priv);

/**
 * pd_sink_unregister_port() - Function to unregister a Sink port
 * @port: the port number
 *
 * The type-C phy driver is supposed to call this function to unregister
 * the port whenever it detects the cable is unplugged or the VBUS has gone.
 *
 * Return: 0 if succeeded or a negative number if failed
 */
int pd_sink_unregister_port(int port);

/**
 * pd_sink_alloc_msg() - allocate a message structure from the PD sink stack
 * @port: the port number
 * @msg_len: the length of the buffer to hold the message
 *
 * This function is used by Type-C phy driver during receiving a message from
 * the PD hardware. The received message is saved in the pd_sink_msg structure
 * which is then pushed to the PD sink stack by calling pd_sink_queue_msg().
 *
 * Return: pointer of a pd_sink_msg structure or NULL if failed
 */
struct pd_sink_msg *pd_sink_alloc_msg(int port, int msg_len);

/**
 * pd_sink_queue_msg() - Queue a received message to the PD sink stack
 * @msg: the message to queue
 *
 * The caller must fill the .sop_type field and copy the message data to
 * the buffer pointed by .buf field.
 *
 * Return: 0 if succeeded or a negative number if failed
 */
int pd_sink_queue_msg(struct pd_sink_msg *msg);

/**
 * pd_sink_change_ps() - Function to change a port's power supply requirement
 * @port: the port number
 * @ps: index to the power supply requirement array previously provided to
 *	the PD sink stack
 *
 * Type-C phy driver calls this function to request a different power supply
 * from Source. This may happen for example the port which is in regular
 * operation state starts charging the battery. In this case the port requests
 * to change to a power supply with higher power.
 *
 * Return: 0 if a REQUEST message is successfully sent, negative value if failed
 */
int pd_sink_change_ps(int port, int ps);

/**
 * pd_sink_reset_state() - Reset the port to initial
 * state(PD_SINK_STATE_WAIT_FOR_SRC_CAP)
 * @port: the port number
 *
 * Type-C phy driver typically calls this function when the VBUS of the
 * port has gone, the cable is unplugged or a hard reset is detected. This
 * function reset the port's state to a initial state(wiat for src_cap) in
 * the PD sink stack thus it can correctly process SOURCE_CAPABILITY message
 * once the port becomes active.
 *
 * Return: 0 on success or a negtive number on failure
 */
int pd_sink_reset_state(int port);

/**
 * pd_sink_tx_complete() - Type-C phy driver reports message transmission
 *                         is completed
 *
 * @port: the port number
 *
 * Return: 0 on success or a negtive number on failure
 */
int pd_sink_tx_complete(int port);

#endif /* __USB_PD_SINK_H */
