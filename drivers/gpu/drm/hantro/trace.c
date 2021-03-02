// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro driver trace file.
 *
 *    Copyright (c) 2017 - 2020, VeriSilicon Inc.
 *    Copyright (c) 2020 - 2021, Intel Corporation
 */

/* bug in tracepoint.h, it should include this */
#include <linux/module.h>

#define CREATE_TRACE_POINTS
#include "trace.h"

void __trace_hantro_msg(const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;

	trace_hantro_msg(&vaf);
	va_end(args);
}

void __trace_hantro_err(const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;

	trace_hantro_err(&vaf);
	va_end(args);
}
