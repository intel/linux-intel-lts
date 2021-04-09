/* SPDX-License-Identifier: GPL-2.0-only */
/*  Copyright (C) 2020 Intel Corporation
 *
 *  Sec Class: Intel Confidential (IC)
 *
 *  All rights reserved.
 *
 *  This document contains proprietary information belonging to Intel.
 *  Passing on and copying of this document, use and
 *  communication of its contents is not permitted without prior written/
 *  authorization.
 *
 *  Purpose: KMB NOC probe bandwidth measurement interface
 *
 */
#ifndef LINUX_NOCDRIVER_H
#define LINUX_NOCDRIVER_H

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <uapi/linux/noc_uapi.h>

/**
 * enum noc_status - NOC probe status
 * @NOC_PROBE_CAPTURE_STARTED: Probe counters monitoring started
 * @NOC_PROBE_ERR_IN_PROGRESS: Probe counters are active, capture failure
 * @NOC_PROBE_ERR_INVALID_ARGS: NOC error, Invalid arguments passed
 * @NOC_PROBE_ERR_COMPLETED: NOC counter values captured successfully
 */
enum noc_status {
	NOC_PROBE_CAPTURE_STARTED,
	NOC_PROBE_ERR_IN_PROGRESS,
	NOC_PROBE_ERR_INVALID_ARGS,
	NOC_PROBE_COMPLETED,
};

/**
 * enum noc_counter - NOC counter
 * @NOC_COUNTER_0: NOC Counter 0
 * @NOC_COUNTER_1: NOC Counter 1
 */
enum noc_counter {
	NOC_COUNTER_0,
	NOC_COUNTER_1,
	NOC_COUNTER_2,
	NOC_COUNTER_3,
	NOC_COUNTER_MAX
};

/* NOC Control Register offsets */
#define ID_COREID		0x0000 /* MAIN_PROBE_ID_COREID */
#define ID_REVISIONID		0x0004 /* MAIN_PROBE_ID_REVISIONID */
#define MAINCTL			0x0008 /* Probe global control bits */
#define CFGCTL			0x000C /* MAIN_PROBE_CFGCTL */
#define TRACEPORTSEL		0x0010 /* MAIN_PROBE_TRACEPORTSEL */
#define FILTERLUT		0x0014 /* MAIN_PROBE_FILTERLUT */
#define TRACEALARMEN		0x0018 /* MAIN_PROBE_TRACEALARMEN */
#define TRACEALARMSTATUS	0x001C /* MAIN_PROBE_TRACEALARMSTATUS */
#define TRACEALARMCLR		0x0020 /* MAIN_PROBE_TRACEALARMCLR */
#define STATPERIOD		0x0024 /* MAIN_PROBE_STATPERIOD */
#define STATGO			0x0028 /* MAIN_PROBE_STATGO */
#define STATALARMMIN		0x002C /* MAIN_PROBE_STATALARMMIN */
#define STATALARMMAX		0x0030 /* MAIN_PROBE_STATALARMMAX */
#define STATALARMSTATUS		0x0034 /* MAIN_PROBE_STATALARMSTATUS */
#define STATALARMCLR		0x0038 /* MAIN_PROBE_STATALARMCLR */
#define STATALARMEN		0x003C /* MAIN_PROBE_STATALARMEN */

/* NOC Filter Registers offsets */
#define F_ROUTEIDBASE		0x0000 /* Offset for FILTERS_n_ROUTEIDBASE */
#define F_ROUTEIDMASK		0x0004 /* Offset for FILTERS_n_ROUTEIDMASK */
#define F_ADDRBASE_LOW		0x0008 /* Offset for FILTERS_n_ADDRBASE_LOW */
#define F_ADDRBASE_HIGH		0x000C /* Offset for FILTERS_n_ADDRBASE_HIGH */
#define F_WINDOWSIZE		0x0010 /* Offset for FILTERS_n_WINDOWSIZE */
#define F_OPCODE		0x001C /* Offset for FILTERS_n_OPCODE */
#define F_STATUS		0x0020 /* Offset for FILTERS_n_STATUS */
#define F_LENGTH		0x0024 /* Offset for FILTERS_n_LENGTH */
#define F_URGENCY		0x0028 /* Offset for FILTERS_n_URGENCY */
#define F_USERBASE		0x002C /* Offset for FILTERS_n_USERBASE */
#define F_USERMASK		0x0030 /* Offset for FILTERS_n_USERMASK */

/* NOC Counter Registers offsets */
#define C_PORTSEL		0x0000 /* Offset for COUNTERS_n_PORTSEL */
#define C_SRC			0x0004 /* Offset for COUNTERS_n_SRC */
#define C_ALARMMODE		0x0008 /* Offset for COUNTERS_n_ALARMMODE */
#define C_VAL			0x000C /* Offset for COUNTERS_n_VAL */

/* NOC register secure access r/w */
#define PLATFORM_SIP_SVC_DSS_NOC_PROBE_READ		(0x8200ff28)
#define PLATFORM_SIP_SVC_DSS_NOC_PROBE_WRITE		(0x8200ff29)

/* Timeout(msec) for checking active counters */
#define NOC_CAPTURE_TIMEOUT_MSEC	2000
#define NOC_STATPERIOD_VAL		0x1B

#define COUNTERS_0_SRC_VAL	0x14
#define COUNTERS_1_SRC_VAL	0x10
#define COUNTERS_ALARMMODE_VAL	0x02
#define FILTER_WINDOW_VAL	0xFFFFFFFF
#define FILTER_OPCODE_VAL	0x0F
#define FILTER_STATUS_VAL	0x03
#define FILTER_LENGTH_VAL	0x03

/* NOC Probe Main controli register fields */
#define MAINCTL_STATEN_POS		3
#define MAINCTL_ALARM_EN_POS		4
#define MAINCTL_ALWAYS_CHAINABLE_POS	7

struct noc_device {
	struct class *dev_class;
	struct cdev noc_cdev;
	dev_t cdev;
};

int flex_noc_setup(enum noc_ss_type noc, enum noc_counter counter, int trace_port);
enum noc_status flexnoc_probe_start(enum noc_ss_type noc);
enum noc_status flexnoc_counter_capture(enum noc_ss_type noc,
					enum noc_counter counter, u32 *value);
int intel_noc_cdev_init(struct noc_device *nocdev);

#endif
