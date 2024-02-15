// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Core SoC Power Management Controller IPC mailbox
 *
 * Copyright (c) 2023, Intel Corporation.
 * All Rights Reserved.
 *
 * Authors: Choong Yong Liang <yong.liang.choong@linux.intel.com>
 *          David E. Box <david.e.box@linux.intel.com>
 */
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/platform_data/x86/intel_pmc_ipc.h>

#define PMC_IPCS_PARAM_COUNT           7

int intel_pmc_ipc(struct pmc_ipc_cmd *ipc_cmd, u32 *rbuf)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object params[PMC_IPCS_PARAM_COUNT] = {
		{.type = ACPI_TYPE_INTEGER,},
		{.type = ACPI_TYPE_INTEGER,},
		{.type = ACPI_TYPE_INTEGER,},
		{.type = ACPI_TYPE_INTEGER,},
		{.type = ACPI_TYPE_INTEGER,},
		{.type = ACPI_TYPE_INTEGER,},
		{.type = ACPI_TYPE_INTEGER,},
	};
	struct acpi_object_list arg_list = { PMC_IPCS_PARAM_COUNT, params };
	union acpi_object *obj;
	int status;

	if (!ipc_cmd || !rbuf)
		return -EINVAL;

	/*
	 * 0: IPC Command
	 * 1: IPC Sub Command
	 * 2: Size
	 * 3-6: Write Buffer for offset
	 */
	params[0].integer.value = ipc_cmd->cmd;
	params[1].integer.value = ipc_cmd->sub_cmd;
	params[2].integer.value = ipc_cmd->size;
	params[3].integer.value = ipc_cmd->wbuf[0];
	params[4].integer.value = ipc_cmd->wbuf[1];
	params[5].integer.value = ipc_cmd->wbuf[2];
	params[6].integer.value = ipc_cmd->wbuf[3];

	status = acpi_evaluate_object(NULL, "\\IPCS", &arg_list, &buffer);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	obj = buffer.pointer;
	/* Check if the number of elements in package is 5 */
	if (obj && obj->type == ACPI_TYPE_PACKAGE && obj->package.count == 5) {
		const union acpi_object *objs = obj->package.elements;

		if ((u8)objs[0].integer.value != 0)
			return -EINVAL;

		rbuf[0] = objs[1].integer.value;
		rbuf[1] = objs[2].integer.value;
		rbuf[2] = objs[3].integer.value;
		rbuf[3] = objs[4].integer.value;
	} else {
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(intel_pmc_ipc);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel PMC IPC Mailbox accessor");
