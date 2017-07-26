/*
 * Copyright (c) 2015--2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM ipu4

#if !defined(INTEL_IPU4_TRACE_EVENT_CSI2_H) || defined(TRACE_HEADER_MULTI_READ)
#define INTEL_IPU4_EVENT_CSI2_H

#include <linux/tracepoint.h>

TRACE_EVENT(ipu4_sof_seqid,
		TP_PROTO(unsigned int seqid, unsigned int csiport,
			unsigned int csivc),
		TP_ARGS(seqid, csiport, csivc),
		TP_STRUCT__entry(
			__field(unsigned int, seqid)
			__field(unsigned int, csiport)
			__field(unsigned int, csivc)
		),
		TP_fast_assign(
			__entry->seqid = seqid;
			__entry->csiport = csiport;
			__entry->csivc = csivc;
		),
		TP_printk("seqid<%u>,csiport<%u>,csivc<%u>", __entry->seqid,
			__entry->csiport, __entry->csivc)
);

#endif

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE  intel-ipu4-trace-event-csi2
/* This part must be outside protection */
#include <trace/define_trace.h>
