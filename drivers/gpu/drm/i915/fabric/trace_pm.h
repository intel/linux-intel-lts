/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#if !defined(__TRACE_PM_H) || defined(TRACE_HEADER_MULTI_READ)
#define __TRACE_PM_H

#include <linux/tracepoint.h>
#include <linux/trace_seq.h>

#include "iaf_drv.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM iaf_pm
#define PORT_STATE_CHANGE_PRN "D%uT%uP%d: type %u LMA %u PPS %x OLDRNNLQI %x " \
			      "nbr %016llx/%u/%u/%u LIR %u LDR %u FID %08x " \
			      "LDC %u PEA %u"

DECLARE_EVENT_CLASS(iaf_pm_template,
		    TP_PROTO(u8 dev,
			     u8 tile,
			     u8 port_num,
			     struct portinfo *portinfo),
		    TP_ARGS(dev, tile, port_num, portinfo),
		    TP_STRUCT__entry(__field(u8, dev)
				     __field(u8, tile)
				     __field(u8, port_num)
				     __field(u8, port_type)
				     __field(u8, port_link_mode_active)
				     __field(u8, port_state_port_physical_state)
				     __field(u8, oldr_nn_lqi)
				     __field(u64, neighbor_guid)
				     __field(u8, neighbor_port_number)
				     __field(u8, neighbor_link_down_reason)
				     __field(u8, neighbor_mtu)
				     __field(u8, link_init_reason)
				     __field(u8, link_down_reason)
				     __field(u16, fid)
				     __field(u32, link_down_count)
				     __field(u32, port_error_action)
				    ),
		    TP_fast_assign(__entry->dev = dev;
				   __entry->tile = tile;
				   __entry->port_num = port_num;
				   __entry->port_type = portinfo->port_type;
				   __entry->port_link_mode_active =
				   portinfo->port_link_mode_active;
				   __entry->port_state_port_physical_state =
				   portinfo->port_state_port_physical_state;
				   __entry->oldr_nn_lqi = portinfo->oldr_nn_lqi;
				   __entry->neighbor_guid =
				   portinfo->neighbor_guid;
				   __entry->neighbor_port_number =
				   portinfo->neighbor_port_number;
				   __entry->neighbor_link_down_reason =
				   portinfo->neighbor_link_down_reason;
				   __entry->neighbor_mtu =
				   portinfo->neighbor_mtu;
				   __entry->link_init_reason =
				   portinfo->link_init_reason;
				   __entry->link_down_reason =
				   portinfo->link_down_reason;
				   __entry->fid = portinfo->fid;
				   __entry->link_down_count =
				   portinfo->link_down_count;
				   __entry->port_error_action =
				   portinfo->port_error_action;
				  ),
		    TP_printk(PORT_STATE_CHANGE_PRN,
			      __entry->dev,
			      __entry->tile,
			      __entry->port_num,
			      __entry->port_type,
			      __entry->port_link_mode_active,
			      __entry->port_state_port_physical_state,
			      __entry->oldr_nn_lqi,
			      __entry->neighbor_guid,
			      __entry->neighbor_port_number,
			      __entry->neighbor_link_down_reason,
			      __entry->neighbor_mtu,
			      __entry->link_init_reason,
			      __entry->link_down_reason,
			      __entry->fid,
			      __entry->link_down_count,
			      __entry->port_error_action
			     )
		    );

DEFINE_EVENT(iaf_pm_template, pm_psc,
	     TP_PROTO(u8 dev, u8 tile, u8 port_num, struct portinfo *portinfo),
	     TP_ARGS(dev, tile, port_num, portinfo)
	    );

#endif /* __TRACE_PM_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_pm
#include <trace/define_trace.h>
