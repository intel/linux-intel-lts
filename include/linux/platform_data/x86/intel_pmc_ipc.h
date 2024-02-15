/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Intel Core SoC Power Management Controller Header File
 *
 * Copyright (c) 2023, Intel Corporation.
 * All Rights Reserved.
 *
 * Authors: Choong Yong Liang <yong.liang.choong@linux.intel.com>
 *          David E. Box <david.e.box@linux.intel.com>
 */
#ifndef INTEL_PMC_IPC_H
#define INTEL_PMC_IPC_H

#define IPC_SOC_REGISTER_ACCESS			0xAA
#define IPC_SOC_SUB_CMD_READ			0x00
#define IPC_SOC_SUB_CMD_WRITE			0x01

struct pmc_ipc_cmd {
	u32 cmd;
	u32 sub_cmd;
	u32 size;
	u32 wbuf[4];
};

/**
 * intel_pmc_ipc() - PMC IPC Mailbox accessor
 * @ipc_cmd:  struct pmc_ipc_cmd prepared with input to send
 * @rbuf:     Allocated u32[4] array for returned IPC data
 *
 * Return: 0 on success. Non-zero on mailbox error
 */
int intel_pmc_ipc(struct pmc_ipc_cmd *ipc_cmd, u32 *rbuf);

#endif /* INTEL_PMC_IPC_H */
