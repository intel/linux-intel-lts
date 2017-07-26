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

#if !defined(INTEL_IPU4_TRACE_EVENT_PSYS_H) || defined(TRACE_HEADER_MULTI_READ)
#define INTEL_IPU4_TRACE_EVENT_PSYS_H

#include <linux/tracepoint.h>

TRACE_EVENT(ipu4_pg_kcmd,
		TP_PROTO(const char *func, unsigned int id,
			unsigned long long issue_id, unsigned int pri,
			unsigned int pg_id, unsigned int load_cycles,
			unsigned int init_cycles,
			unsigned int processing_cycles),
		TP_ARGS(func, id, issue_id, pri, pg_id, load_cycles,
			init_cycles, processing_cycles),
		TP_STRUCT__entry(
			__field(const char *, func)
			__field(unsigned int, id)
			__field(unsigned long long, issue_id)
			__field(unsigned int, pri)
			__field(unsigned int, pg_id)
			__field(unsigned int, load_cycles)
			__field(unsigned int, init_cycles)
			__field(unsigned int, processing_cycles)
		),
		TP_fast_assign(
			__entry->func = func;
			__entry->id = id;
			__entry->issue_id = issue_id;
			__entry->pri = pri;
			__entry->pg_id = pg_id;
			__entry->load_cycles = load_cycles;
			__entry->init_cycles = init_cycles;
			__entry->processing_cycles = processing_cycles;
		),
		TP_printk("pg-kcmd: func=%s,id=%u,issue_id=0x%llx,pri=%u,pg_id=%d,load_cycles=%u,init_cycles=%u,processing_cycles=%u",
			__entry->func, __entry->id, __entry->issue_id,
			__entry->pri, __entry->pg_id, __entry->load_cycles,
			__entry->init_cycles, __entry->processing_cycles)
);

#endif

#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE  intel-ipu4-trace-event-psys
/* This part must be outside protection */
#include <trace/define_trace.h>
