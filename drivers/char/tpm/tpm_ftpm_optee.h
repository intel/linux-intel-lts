
/*
 * Copyright (C) 2018 Intel Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TPM_FTPM_OPTEE__
#define __TPM_FTPM_OPTEE__

#include "tpm.h"

#define FTPM_DEBUG_HEAD	"[fTPM]"
#define kdebug(FMT, ...) \
	pr_info(FTPM_DEBUG_HEAD " ==> %s(" FMT ")\n", \
	       __func__, ##__VA_ARGS__)

#define TA_FTPM_UUID \
		{ 0x71d950bc, 0xc9d4, 0xc442, \
			{ 0x82, 0xcb, 0x34, 0x3f, 0xb7, 0xf3, 0x78, 0x96 } }

struct teec_uuid {
	u32 timeLow;
	u16 timeMid;
	u16 timeHiAndVersion;
	u8 clockSeqAndNode[8];
};

enum fwtpm_status {
	FTPM_SESSION_INITED	= BIT(0),
};

struct fwtpm_device_priv {
	unsigned int status;
	struct tee_context *ctx;
	u32 cancel_id;
	struct tee_shm *cmd_buf_shm;
	struct tee_shm *resp_buf_shm;
	u32 sess;
	bool is_canceled;
	size_t cmd_len;
	size_t resp_len;
	char *cmd_buf;
	char *resp_buf;

};

enum fwtpm_pm_event {
	FTPM_PM_RESUME	= 0,
	FTPM_PM_SUSPEND	= 1,
};

struct fwtpm_pm_data {
	u64 pm_event;
};

enum fwtpm_optee_cmd {
	FTPM_HANDLE_CMD_SUBMIT = 0,
	FTPM_HANDLE_PPI	= 1,
	FTPM_HANDLE_PM	= 2,
};

#endif /* __TPM_FTPM_OPTEE__ */
