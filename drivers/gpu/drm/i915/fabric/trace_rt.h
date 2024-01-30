/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#if !defined(__TRACE_RT_H) || defined(TRACE_HEADER_MULTI_READ)
#define __TRACE_RT_H

#include <linux/tracepoint.h>
#include <linux/trace_seq.h>

#include "iaf_drv.h"
#include "routing_topology.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM iaf_rt
#define ROUTE_PAIR_LOCAL_PRN "uft: device %u sd %u fid 0x%04x bport %u fport %u"
#define ASSIGN_FIDS_PRN "device %u sd %u fid_mgmt 0x%04x fid_base 0x%04x"

DECLARE_EVENT_CLASS(iaf_rt_template,
		    TP_PROTO(u8 id, u8 sd_index, u16 fid, u8 bport, u8 fport),
		    TP_ARGS(id, sd_index, fid, bport, fport),
		    TP_STRUCT__entry(__field(u8, id)
				     __field(u8, sd_index)
				     __field(u16, fid)
				     __field(u8, bport)
				     __field(u8, fport)
				    ),
		    TP_fast_assign(__entry->id = id;
				   __entry->sd_index = sd_index;
				   __entry->fid = fid;
				   __entry->bport = bport;
				   __entry->fport = fport;
				  ),
		    TP_printk(ROUTE_PAIR_LOCAL_PRN,
			      __entry->id,
			      __entry->sd_index,
			      __entry->fid,
			      __entry->bport,
			      __entry->fport
			     )
		    );

DEFINE_EVENT(iaf_rt_template, rt_pair,
	     TP_PROTO(u8 id, u8 sd_index, u16 fid, u8 bport, u8 fport),
	     TP_ARGS(id, sd_index, fid, bport, fport)
	    );

DEFINE_EVENT(iaf_rt_template, rt_local,
	     TP_PROTO(u8 id, u8 sd_index, u16 fid, u8 bport, u8 fport),
	     TP_ARGS(id, sd_index, fid, bport, fport)
	    );

DECLARE_EVENT_CLASS(iaf_rt_fid_template,
		    TP_PROTO(struct fsubdev *sd),
		    TP_ARGS(sd),
		    TP_STRUCT__entry(__field(u32, id)
				     __field(u8, sd_index)
				     __field(u16, fid_mgmt)
				     __field(u16, fid_base)
				    ),
		    TP_fast_assign(__entry->id = sd->fdev->pd->index;
				   __entry->sd_index = sd_index(sd);
				   __entry->fid_mgmt = sd->routing.fid_mgmt;
				   __entry->fid_base = sd->routing.fid_base;
				  ),
		    TP_printk(ASSIGN_FIDS_PRN,
			      __entry->id,
			      __entry->sd_index,
			      __entry->fid_mgmt,
			      __entry->fid_base
			     )
		    );

DEFINE_EVENT(iaf_rt_fid_template, rt_assign_fids,
	     TP_PROTO(struct fsubdev *sd),
	     TP_ARGS(sd)
	    );

DECLARE_EVENT_CLASS(iaf_rt_plane_template,
		    TP_PROTO(struct fsubdev *root),
		    TP_ARGS(root),
		    TP_STRUCT__entry(__field(u8, index)
				     __field(u8, id)
				     __field(u8, sd_index)
				    ),
		    TP_fast_assign(__entry->index = root->routing.plane->index;
				   __entry->id = sd_index(root);
				   __entry->sd_index = sd_index(root);
				  ),
		    TP_printk("plane %u device %u sd %u",
			      __entry->index,
			      __entry->id,
			      __entry->sd_index
			     )
		    );

DEFINE_EVENT(iaf_rt_plane_template, rt_plane,
	     TP_PROTO(struct fsubdev *root),
	     TP_ARGS(root)
	    );

#endif /* __TRACE_RT_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_rt
#include <trace/define_trace.h>
