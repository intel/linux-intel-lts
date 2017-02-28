/*
 * pd_messae.h - USB PD message header file.
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

#ifndef __USB_PD_MESSAGE_H
#define __USB_PD_MESSAGE_H

/* All length definitions are in bytes */
#define PD_MAX_EXTENDED_MSG_LEN		260
#define PD_MAX_EXTENDED_MSG_CHUNK_LEN	26
#define PD_MAX_EXTENDED_MSG_LEGACY_LEN	26
#define PD_MSG_HEADER_LEN		2
#define PD_MSG_EXT_HEADER_LEN		2

/* Control Messages */
/* 0: reserved */
#define PD_CMSG_GOODCRC		1
#define PD_CMSG_GOTOMIN		2
#define PD_CMSG_ACCEPT		3
#define PD_CMSG_REJECT		4
#define PD_CMSG_PING		5
#define PD_CMSG_PS_RDY		6
#define PD_CMSG_GET_SRC_CAP	7
#define PD_CMSG_GET_SINK_CAP	8
#define PD_CMSG_DR_SWAP		9
#define PD_CMSG_PR_SWAP		10
#define PD_CMSG_VCONN_SWAP	11
#define PD_CMSG_WAIT		12
#define PD_CMSG_SOFT_RESET	13
/* 14-15: reserved */
/* 16-19: Revision 3.0 only */
#define PD_CMSG_NOT_SUPPORTED	16
#define PD_CMSG_GET_SOURCE_CAP_EXTENDED	17
#define PD_CMSG_GET_STATUS	18
#define PD_CMSG_FR_SWAP		19
/* 20-31: reserved */
#define PD_NR_CMSGS		32

/* Data messages */
/* 0: reserved */
#define PD_DMSG_SOURCE_CAP	1
#define PD_DMSG_REQUEST		2
#define PD_DMSG_BIST		3
#define PD_DMSG_SINK_CAP	4
/* 5-6: Revision 3.0 only */
#define PD_DMSG_BATTERY_STATUS	5
#define PD_DMSG_ALERT		6
/* 7-14: reserved */
#define PD_DMSG_VENDOR_DEFINED	15
/* 16-31: reserved */
#define PD_NR_DMSGS		32

#define PD_MSG_ID_MIN		0
#define PD_MSG_ID_MAX		7

/* Both control message and data message are 5-bit long */
#define PD_MSG_TYPE_MASK	0x1F

/* Source Capability bits: 31:30 */
#define SOURCE_CAP_TYPE_BIT	30
#define SOURCE_CAP_TYPE_MASK	0x3

#define PD_OBJ_SIZE		4 /* each data object is 32bit long */

/* All timeouts are in milli-seconds, unless speically commented */
#define PD_TIMEOUT_GOODCRC	500
#define PD_TIMEOUT_ACCEPT	500
#define PD_TIMEOUT_PS_READY	1000

#define PD_VSAFE5V			5 /* Default VBUS voltage: 5V */

/*
 * Message Header (16 bits)
 * When SOP type is SOP' or SOP'':
 * The 1bit data_role field is defined
 * as "Cable Plug": 1-Message originated from a Cable Plug
 *                  0-Message originated from a DFP or UFP
 * The 1bit Port Data Role is defined as "Reserved".
 */
struct pd_msg_header {
	/*
	 * All fields are for SOP type: SOP, except:
	 * When SOP type is SOP' or SOP'', the following bits have
	 * different meaning:
	 * power_role:1 - "Cable Plug"
	 *   1-Message originated from a Cable Plug
	 *   0-Message originated from a DFP or UFP
	 * data_role:1 - "Reserved"
	 */
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/*
	 * According to the USB PD spec Rev 3.0 V1.0a, extended bit is
	 * the most significant bit(bit 15) and message type bits are the
	 * least significant 5 bits (bit 4:0).
	 */
	u16 type:5;		/* Message type */
	u16 data_role:1;	/* Port data role: 1-DFP 0-UFP */
	u16 revision:2;		/* Specification revision */
	u16 power_role:1;	/* Port power role: 1-Source 0-Sink */
	u16 id:3;		/* Message ID */
	u16 nr_objs:3;		/* Number of 32bit data objects */
	u16 extended:1;		/* Revision 2.0 - reserved, Revision 3.0 -
				 * extended message: 1-Extended 0-NonExtended
				 */
#elif defined(__BIG_ENDIAN_BITFIELD)
	u16 extended:1;
	u16 nr_objs:3;
	u16 id:3;
	u16 power_role:1;
	u16 revision:2;
	u16 data_role:1;
	u16 type:5;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/* Extended message is only for PD Revision 3.0 */
#define PD_MSG_EXTENDED		1
#define PD_MSG_NOT_EXTENDED	0

#define PD_POWER_ROLE_SINK	0 /* Consume power */
#define PD_POWER_ROLE_SOURCE	1 /* Supply power */

#define PD_DATA_ROLE_UFP	0 /* Up facing port: device */
#define PD_DATA_ROLE_DFP	1 /* Down facing port: host */

#define PD_REVISION_1		0 /* USB PD Specification Revision 1.0 */
#define PD_REVISION_2		1 /* USB PD Specification Revision 2.0 */
#define PD_REVISION_3		2 /* USB PD Specification Revision 3.0 */

/* Fixed supply PDO peak current capability */
#define PEAK_CURRENT_DEFAULT	0 /* default operating current, no overload */
#define PEAK_CURRENT_1		1 /* low overload */
#define PEAK_CURRENT_2		2 /* medium overload */
#define PEAK_CURRENT_3		3 /* high overload */

/* Fast Role Swap coding in Fixed Supply PDO (for Sink) */
#define FAST_ROLE_SWAP_DISABLE	0
#define FAST_ROLE_SWAP_DEFAULT	1 /* Default USB Power */
#define FAST_ROLE_SWAP_1P5A_5V	2 /* 1.5A @ 5V */
#define FAST_ROLE_SWAP_3A_5V	3 /* 3.0A @ 5V */

/* ALERT message: PD Revision 3.0 only */
#define PD_ALERT_BATTERY	BIT(1) /* Battery status change */
#define PD_ALERT_OCP		BIT(2) /* Source only */
#define PD_ALERT_OTP		BIT(3)
#define PD_ALERT_OCC		BIT(4) /* Operating condition change */
#define PD_ALERT_SRC_INPUT	BIT(5) /* Source input change */
#define PD_ALERT_OVP		BIT(6) /* OVP, Sink only */

/*
 * Power supply type:
 * Fixed power Supply is the most commonly used to expose well-regulated fixed
 * voltage power supplies.
 * Variable power supply is used to expose very poorly regulated power supplies.
 * Battery power supply is used to expose batteries that can be directly
 * connected to VBUS.
 * The definition below is used in both SOURCE_CAPABILITY and SINK_CAPABILITY
 * messages.
 */
#define PS_TYPE_FIXED		0
#define PS_TYPE_BATTERY		1
#define PS_TYPE_VARIABLE	2

enum sop_type {
	SOP,	/* SOP */
	SOP_P,	/* SOP' */
	SOP_PP,	/* SOP'' */
};

/* Extended Message Header (16 bits), PD Rev 3.0 only */
struct pd_ext_msg_header {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u16 data_size:9;
	u16 reserved:1;
	u16 request_chunk:1;
	u16 chunk_number:4;
	u16 chunked:1;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u16 chunked:1;
	u16 chunk_number:4;
	u16 request_chunk:1;
	u16 reserved:1;
	u16 data_size:9;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/* Fixed power supply PDO(Power Data Object) in SOURCE_CAPABILITY message */
struct pd_pdo_src_fixed {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u32 current_max:10;	/* Maximum current in 10mA units */
	u32 voltage:10;		/* Voltage in 50mV units */
	u32 current_peak:2;	/* Peak Current */
	u32 reserved:2;		/* Reserved, shall be zero */
	u32 unchunked:1;	/* Unchunked extended message supported */
	u32 drd:1;		/* Dual-Role Data */
	u32 usb_capable:1;	/* USB communication capable */
	u32 ext_powered:1;	/* Externally powered */
	u32 usb_suspend:1;	/* USB suspend supported */
	u32 drw:1;		/* Dual-Role Power */
	u32 type:2;		/* Fixed supply PDO - 00b */
#elif defined(__BIG_ENDIAN_BITFIELD)
	u32 type:2;
	u32 drw:1;
	u32 usb_suspend:1;
	u32 ext_powered:1;
	u32 usb_capable:1;
	u32 drd:1;
	u32 unchunked:1;
	u32 reserved:2;
	u32 current_peak:2;
	u32 voltage:10;
	u32 current_max:10;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/* Fixed power supply PDO(Power Data Object) in SINK_CAPABILITY message */
struct pd_pdo_sink_fixed {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u32 current_max:10;	/* Maximum current in 10mA units */
	u32 voltage:10;		/* Voltage in 50mV units */
	u32 reserved:3;		/* Reserved, shall be zero */
	u32 fast_role_swap:2;	/* Fast Role Swap required USB Type-C Current*/
	u32 drd:1;		/* Dual-Role Data */
	u32 usb_capable:1;	/* USB communication capable */
	/* Externally powered(e.g. an AC supply is connected to the Sink) */
	u32 ext_powered:1;
	u32 higher_capability:1;	/* Sink needs more than vSafe5V */
	u32 drw:1;		/* Dual-Role Power */
	u32 type:2;		/* Fixed supply PDO - 00b */
#elif defined(__BIG_ENDIAN_BITFIELD)
	u32 type:2;
	u32 drw:1;
	u32 higher_capability:1;
	u32 ext_powered:1;
	u32 usb_capable:1;
	u32 drd:1;
	u32 fast_role_swap:2;
	u32 reserved:3;
	u32 voltage:10;
	u32 current_max:10;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/*
 * Variable power supply PDO in both SOURCE_CAPABILITY and
 * SINK_CAPABILITY messages.
 */
struct pd_pdo_variable {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/* current in 10mA units, Source: maximum, Sink: operational */
	u32 current_mo:10;
	u32 voltage_min:10;	/* Mimimum voltage in 50mV units */
	u32 voltage_max:10;	/* Maximum voltage in 50mV units */
	u32 type:2;		/* Variable supply PDO - 10b */
#elif defined(__BIG_ENDIAN_BITFIELD)
	u32 type:2;
	u32 voltage_max:10;
	u32 voltage_min:10;
	u32 current_max:10;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/*
 * Battery power supply PDO in both SOURCE_CAPABILITY and
 * SINK_CAPABILITY messages.
 */
struct pd_pdo_battery {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/*  Power in 250mW units: Source - maximum, Sink - operational */
	u32 power:10;
	u32 voltage_min:10;	/* Minimum voltage in 50mV units */
	u32 voltage_max:10;	/* Maximum voltage in 50mV units */
	u32 type:2;		/* Battery supply PDO - 01b */
#elif defined(__BIG_ENDIAN_BITFIELD)
	u32 type:2;
	u32 voltage_max:10;
	u32 voltage_min:10;
	u32 power_max:10;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/* Request Data Object for Fixed, Variable or Battery power supply */
struct pd_request {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/*
	 * Minimum operating current in 10mA units for Fixed and Variable PS.
	 * Maximum operating power in 250mW units for Battery PS.
	 */
	u32 operating_min_max:10;
	/*
	 * Default operating current in 10mA units for Fixed and Variable PS.
	 * Default operating power in 250mW units for Battery PS.
	 */
	u32 operating_default:10;
	u32 reserved2:3;
	/*
	 * 1 - Support unchunked extended message.
	 * 0 - No support for unchunked extended message.
	 * This bit is only for PD Rev 3.0.
	 */
	u32 unchunked_ext_msg_supported:1;
	/*
	 * 1 - Request to use power during USB suspend. This is for purposes
	 *     other than USB communication e.g. for charging a Battery.
	 * 0 - Don't use power during USB suspend.
	 */
	u32 no_suspend:1;
	u32 usb_comm_capable:1; /* USB communication capable from this port */
	/*
	 * 1 - No PDO from SOURCE_CAPABILITY fits Sink's requirement.
	 * 0 - One of PDOs fits.
	 */
	u32 cap_mismatch:1;
	u32 give_back:1; /* 1 - Support GOTOMIN message, 0 - No support */
	u32 pos:3; /* Index of the selected PDO in the SOURCE_CAPABILITY msg */
	u32 reserved1:1;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u32 reserved1:1;
	u32 pos:3;
	u32 give_back:1;
	u32 cap_mismatch:1;
	u32 usb_comm_capable:1;
	u32 no_suspend:1;
	u32 unchunked_ext_msg_supported:1;
	u32 reserved2:3;
	u32 operating_default:10;
	u32 operating_min_max:10;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

/* ALERT message: may sent by Source or Sink, PD Revision 3.0 only */
struct alert_msg {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u32 reserved:16;
	u32 hot_swappable_batteries:4;
	u32 fixed_batteries:4;
	u32 type:8;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u32 type:8;
	u32 fixed_batteries:4;
	u32 hot_swappable_batteries:4;
	u32 reserved:16;
#else
	#error "Please fix <asm/byteorder.h>"
#endif
} __packed;

#endif /* __USB_PD_MESSAGE_H */
