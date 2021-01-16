/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*  Copyright (C) 2020 Intel Corporation
 *
 *  Sec Class: Intel Confidential (IC)
 *
 *  All rights reserved.
 *
 *  This document contains proprietary information belonging to Intel.
 *  Passing on and copying of this document, use and
 *  communication of its contents is not permitted without prior written/
 *  authorisation.
 *
 *  Purpose: KMB NOC header for user space access.
 *
 *  Authors:
 *
 */

#ifndef __NOC_UAPI_H
#define __NOC_UAPI_H

#include <linux/types.h>

#define NOC_MAGIC 'n'
#define NOC_SETUP                _IOW(NOC_MAGIC, 1, void*)
#define NOC_PROBE_START          _IOW(NOC_MAGIC, 2, void*)
#define NOC_COUNTER_CAPTURE      _IOW(NOC_MAGIC, 3, void*)

enum noc_ss_type {
	DSS_NOC = 0,
	CSS_NOC
};

struct flexnoc_setup {
	enum noc_ss_type noc_type;
	__u16 counter;
	__u16 traceport;
	__u16 ret_id;
};

struct flexnoc_probestart {
	enum noc_ss_type noc_type;
	__u32 captime;
	__u16 ret_id;
};

struct flexnoc_countercapture {
	enum noc_ss_type noc_type;
	__u16 counter;
	__u32 bw_res;
	__u16 ret_id;
};

#endif  /*_NOC__UAPI_H*/
