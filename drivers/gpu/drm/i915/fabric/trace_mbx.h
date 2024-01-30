/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#if !defined(__TRACE_MBX_H) || defined(TRACE_HEADER_MULTI_READ)
#define __TRACE_MBX_H

#include <linux/tracepoint.h>
#include <linux/trace_seq.h>

#include "iaf_drv.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM iaf_mbx

DECLARE_EVENT_CLASS(iaf_mbx_in_template,
		    TP_PROTO(u32 dev, u8 sd, u8 op_code, u8 expected_seq_no,
			     u8 seq_no, u8 rsp_status, u32 tid, u64 cw),
		    TP_ARGS(dev, sd, op_code, expected_seq_no, seq_no,
			    rsp_status, tid, cw),
		    TP_STRUCT__entry(__field(u32, dev)
				     __field(u8, sd)
				     __field(u8, op_code)
				     __field(u8, expected_seq_no)
				     __field(u8, seq_no)
				     __field(u8, rsp_status)
				     __field(u32, tid)
				     __field(u64, cw)
				    ),
		    TP_fast_assign(__entry->dev = dev;
				   __entry->sd = sd;
				   __entry->op_code = op_code;
				   __entry->expected_seq_no = expected_seq_no;
				   __entry->seq_no = seq_no;
				   __entry->rsp_status = rsp_status;
				   __entry->tid = tid;
				   __entry->cw = cw;
				  ),
		    TP_printk("dev %u sd %u op_code 0x%x expected_seq_no %u seq_no %u rsp_status %u tid %u cw 0x%llx",
			      __entry->dev,
			      __entry->sd,
			      __entry->op_code,
			      __entry->expected_seq_no,
			      __entry->seq_no,
			      __entry->rsp_status,
			      __entry->tid,
			      __entry->cw
			     )
		    );

DEFINE_EVENT(iaf_mbx_in_template, iaf_mbx_in,
	     TP_PROTO(u32 dev, u8 sd, u8 op_code, u8 expected_seq_no,
		      u8 seq_no, u8 rsp_status, u32 tid, u64 cw),
	     TP_ARGS(dev, sd, op_code, expected_seq_no, seq_no, rsp_status,
		     tid, cw)
	    );

	DECLARE_EVENT_CLASS(iaf_mbx_exec_template,
			    TP_PROTO(u32 dev, u8 sd, u8 op_code, u8 seq_no,
				     u32 tid, u64 cw),
			    TP_ARGS(dev, sd, op_code, seq_no, tid, cw),
			    TP_STRUCT__entry(__field(u32, dev)
					     __field(u8, sd)
					     __field(u8, op_code)
					     __field(u8, seq_no)
					     __field(u32, tid)
					     __field(u64, cw)
					    ),
			    TP_fast_assign(__entry->dev = dev;
					   __entry->sd = sd;
					   __entry->op_code = op_code;
					   __entry->seq_no = seq_no;
					   __entry->tid = tid;
					   __entry->cw = cw;
					  ),
			    TP_printk("dev %u sd %u op_code 0x%x seq_no %u tid %u cw 0x%llx",
				      __entry->dev,
				      __entry->sd,
				      __entry->op_code,
				      __entry->seq_no,
				      __entry->tid,
				      __entry->cw
				     )
			    );

DEFINE_EVENT(iaf_mbx_exec_template, iaf_mbx_exec,
	     TP_PROTO(u32 dev, u8 sd, u8 op_code, u8 seq_no, u32 tid, u64 cw),
	     TP_ARGS(dev, sd, op_code, seq_no, tid, cw)
	    );

#endif /* __TRACE_MBX_H */

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE trace_mbx
#include <trace/define_trace.h>
