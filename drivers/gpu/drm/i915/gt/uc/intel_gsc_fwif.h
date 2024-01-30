/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2021 Intel Corporation
 */

#ifndef _INTEL_GSC_FWIF_H_
#define _INTEL_GSC_FWIF_H_

#include <linux/types.h>

struct intel_gsc_mtl_header {
	u32 validity_marker;
#define GSC_HECI_VALIDITY_MARKER 0xA578875A

	u8 gsc_address;
#define HECI_MEADDRESS_PXP 17
#define HECI_MEADDRESS_HDCP 18

	u8 reserved1;

	u16 header_version;
#define MTL_GSC_HEADER_VERSION 1

	u64 host_session_handle;
	u64 gsc_message_handle;

	u32 message_size; /* lower 20 bits only, upper 12 are reserved */

	/*
	 * Flags mask:
	 * Bit 0: Pending
	 * Bit 1: Session Cleanup;
	 * Bits 2-15: Flags
	 * Bits 16-31: Extension Size
	 */
	u32 flags;
#define INTEL_GSC_MSG_PENDING	1

	u32 status;
} __packed;

#endif
