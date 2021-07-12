/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Registration for IBECC error notification
 * Copyright (C) 2020 Intel Corporation
 */

#ifndef _IGEN6_EDAC_H
#define _IGEN6_EDAC_H

#include <linux/edac.h>
#include <linux/notifier.h>

struct ibecc_err_info {
	enum hw_event_mc_err_type type;
	u64 sys_addr;
	u64 ecc_log;
};

int ibecc_err_register_notifer(struct notifier_block *nb);
int ibecc_err_unregister_notifer(struct notifier_block *nb);

#endif /* _IGEN6_EDAC_H */
