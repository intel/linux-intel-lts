/* SPDX-License-Identifier: GPL-2.0 only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2019 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_BOOT_HEADER_
#define XPCIE_BOOT_HEADER_

#include <linux/types.h>

#define XPCIE_BOOT_MAGIC_ROM "VPUROM"
#define XPCIE_BOOT_MAGIC_EMMC "VPUEMMC"
#define XPCIE_BOOT_MAGIC_BL2 "VPUBL2"
#define XPCIE_BOOT_MAGIC_UBOOT "VPUUBOOT"
#define XPCIE_BOOT_MAGIC_RECOV "VPURECOV"
#define XPCIE_BOOT_MAGIC_YOCTO "VPUYOCTO"

enum xpcie_stage {
	STAGE_UNINIT,
	STAGE_ROM,
	STAGE_BL2,
	STAGE_UBOOT,
	STAGE_RECOV,
	STAGE_OS
};

#define XPCIE_BOOT_FIP_ID (0xFFFFFFFF)
#define XPCIE_BOOT_BOOT_ID (0xFFFFFF4F)
#define XPCIE_BOOT_SYSTEM_ID (0xFFFFFF46)
#define XPCIE_BOOT_RAW_ID (0xFFFFFF00)
#define XPCIE_BOOT_ERASE_ID (0xFFFFFF01)
#define XPCIE_BOOT_FLASH_ID (0xFFFFFF02)

#define XPCIE_BOOT_STATUS_START (0x55555555)
#define XPCIE_BOOT_STATUS_INVALID (0xDEADFFFF)
#define XPCIE_BOOT_STATUS_DOWNLOADED (0xDDDDDDDD)
#define XPCIE_BOOT_STATUS_ERROR (0xDEADAAAA)
#define XPCIE_BOOT_STATUS_DONE (0xBBBBBBBB)

#define XPCIE_INT_ENABLE (0x1)
#define XPCIE_INT_MASK (0x1)

#define XPCIE_BOOT_MAGIC_STRLEN (16)
#define XPCIE_BOOT_DEST_STRLEN (128)

#define XPCIE_IO_COMM_MAGIC_OFF (offsetof(struct xpcie_bootio, magic))
#define XPCIE_IO_COMM_MF_READY_OFF (offsetof(struct xpcie_bootio, mf_ready))
#define XPCIE_IO_COMM_MF_LEN_OFF (offsetof(struct xpcie_bootio, mf_len))
#define XPCIE_IO_COMM_MF_START_OFF (offsetof(struct xpcie_bootio, mf_start))
#define XPCIE_IO_COMM_INT_ENABLE_OFF (offsetof(struct xpcie_bootio, int_enable))
#define XPCIE_IO_COMM_INT_MASK_OFF (offsetof(struct xpcie_bootio, int_mask))
#define XPCIE_IO_COMM_INT_IDENTITY_OFF (offsetof(struct xpcie_bootio, int_identity))
#define XPCIE_IO_COMM_MF_OFF (offsetof(struct xpcie_bootio, mf_offset))
#define XPCIE_IO_COMM_MF_DEST_OFF (offsetof(struct xpcie_bootio, mf_dest))
#define XPCIE_IO_COMM_DEV_ID_OFF (offsetof(struct xpcie_bootio, dev_id))

struct xpcie_bootio {
	u8 magic[XPCIE_BOOT_MAGIC_STRLEN];
	u32 mf_ready;
	u32 mf_len;
	u64 reserved1;
	u64 mf_start;
	u32 int_enable;
	u32 int_mask;
	u32 int_identity;
	u32 reserved2;
	u64 mf_offset;
	u8 mf_dest[XPCIE_BOOT_DEST_STRLEN];
	u64 dev_id;
} __packed;

#endif // XPCIE_BOOT_HEADER_
