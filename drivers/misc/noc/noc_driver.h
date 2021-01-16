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
 *  authorisation.
 *
 *  Purpose: KMB NOC probe bandwidth measurement interface
 *
 *
 */
#ifndef LINUX_NOCDRIVER_H
#define LINUX_NOCDRIVER_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>

/**
 * enum noc_status - NOC probe status
 * @NOC_PROBE_CAPTURE_STARTED: Probe counters monitoring started
 * @NOC_PROBE_IN_PROGRESS: Probe counters are active
 * @NOC_PROBE_ERROR: NOC error
 */
enum noc_status {
	NOC_PROBE_CAPTURE_STARTED,
	NOC_PROBE_ERR_IN_PROGRESS,
	NOC_PROBE_ERR_INVALID_ARGS,
	NOC_PROBE_COMPLETED,
};

/**
 * enum noc_type - NOC Type
 * @NOC_DSS: DSS NOC
 */
enum noc_type {
	NOC_DSS,
	NOC_TYPE_MAX
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

/**
 * flex_noc_setup() - Setup two counters for the NOC probe
 * @noc: NOC type to setup counters
 * @counter: Counter number to set up counter n and n+1
 * @trace_port: trace port number to setup counters
 *
 * This function will setup the counters for the trace port given.
 *
 *
 * TBD:  synhronize with spinlock ?
 */
int flex_noc_setup(enum noc_type noc, enum noc_counter counter, int trace_port);


/**
 * flexnoc_probe_start() - Start two counters for the NOC probe
 * @noc: NOC type to setup counters
 * @capture_time: Duration for probign counters (msec ?)
 *
 * This function will start the setup counters.  When this
 * function returns NOC_PROBE_CAPTURE_STARTED, it is guaranteed that NOC
 * is setup for probing counters.
 *
 */
enum noc_status flexnoc_probe_start(enum noc_type noc, int capture_time);

/**
 * flexnoc_counterp_capture() - Capture the counter statistic values
 * @noc: NOC type to setup counters
 * @counter:  Counter number to capture statistics values for n and n+1
 * @value: statistics values read are returned in this address passed
 *
 * This function will return the statistics value of started counters.
 * When this function returns NOC_PROBE_COMPLETED, it is guaranteed that NOC
 * counters are idle and finished probing.
 *
 */
enum noc_status flexnoc_counter_capture(enum noc_type noc,
				enum noc_counter counter, u32 *value);

struct noc_device {
	struct class *dev_class;
	struct cdev noc_cdev;
	dev_t cdev;
};

int intel_noc_cdev_init(struct noc_device *nocdev);

#endif
