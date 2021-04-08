/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink Core Driver.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM xlink_trace

#if !defined(_XLINK_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _XLINK_TRACE_H

#include <linux/tracepoint.h>


TRACE_EVENT(xlink_open_channel,
		TP_PROTO(uint32_t swid, uint16_t chan),
		TP_ARGS(swid, chan),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
		),
		TP_printk("swid=%d chan=%d", __entry->swid, __entry->chan)
);
TRACE_EVENT(xlink_open_channel_completion,
		TP_PROTO(uint32_t swid, uint16_t chan),
		TP_ARGS(swid, chan),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
		),
		TP_printk("swid=%d chan=%d", __entry->swid, __entry->chan)
);

TRACE_EVENT(xlink_close_channel,
		TP_PROTO(uint32_t swid, uint16_t chan),
		TP_ARGS(swid, chan),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
		),
		TP_printk("swid=%d chan=%d", __entry->swid, __entry->chan)
);

TRACE_EVENT(xlink_close_channel_completion,
		TP_PROTO(uint32_t swid, uint16_t chan),
		TP_ARGS(swid, chan),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
		),
		TP_printk("swid=%d chan=%d", __entry->swid, __entry->chan)
);

TRACE_EVENT(xlink_write_control,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);

TRACE_EVENT(xlink_write_control_completion,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_write_data,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_write_data_completion,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_write_data_user,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_write_data_user_completion,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);

TRACE_EVENT(xlink_read,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_read_mutex,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_read_alloc_event,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);
TRACE_EVENT(xlink_read_channel_mutex,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);

TRACE_EVENT(xlink_read_data_completion,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t size),
		TP_ARGS(swid, chan, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d", __entry->swid,
				__entry->chan, __entry->size)
);

/*tx and rx events*/
TRACE_EVENT(xlink_write_message,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_read_pkt_tx_available,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_read_pkt_tx_consumed,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_rx_read_of_write,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_rx_read_pkt_data_arrived_event,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_rx_read_pkt_control_arrived_event,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_rx_read_consumed_arrived_event,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
/*dispatcher events*/
TRACE_EVENT(xlink_dispatcher_header,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_dispatcher_write,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
TRACE_EVENT(xlink_event_receive,
		TP_PROTO(uint32_t swid, uint16_t chan, uint32_t id,
				uint32_t size),
		TP_ARGS(swid, chan, id, size),
		TP_STRUCT__entry(
			__field(uint32_t, swid)
			__field(uint16_t, chan)
			__field(uint32_t, id)
			__field(uint32_t, size)
		),
		TP_fast_assign(
			__entry->swid = swid;
			__entry->chan = chan;
			__entry->size = id;
			__entry->size = size;
		),
		TP_printk("swid=%d chan=%d size=%d id=%d", __entry->swid,
				__entry->chan, __entry->size, __entry->id)
);
#endif /* _XLINK_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#ifdef CONFIG_XLINK_LOCAL_HOST
#define TRACE_INCLUDE_PATH ../../drivers/misc/xlink-core
#else
#define TRACE_INCLUDE_PATH .
#endif
#define TRACE_INCLUDE_FILE xlink-trace
#include <trace/define_trace.h>
