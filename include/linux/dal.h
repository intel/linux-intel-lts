/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#ifndef _DAL_H_
#define _DAL_H_

#include <linux/types.h>
#include <linux/uuid.h>

#define DAL_VERSION_LEN  32

/**
 * struct dal_version_info - dal version
 *
 * @version: current dal version
 * @reserved: reserved bytes for future use
 */
struct dal_version_info {
	char version[DAL_VERSION_LEN];
	u32 reserved[4];
};

#define DAL_KDI_SUCCESS                         0x000
#define DAL_KDI_STATUS_INTERNAL_ERROR           0xA00
#define DAL_KDI_STATUS_INVALID_PARAMS           0xA01
#define DAL_KDI_STATUS_INVALID_HANDLE           0xA02
#define DAL_KDI_STATUS_NOT_INITIALIZED          0xA03
#define DAL_KDI_STATUS_OUT_OF_MEMORY            0xA04
#define DAL_KDI_STATUS_BUFFER_TOO_SMALL         0xA05
#define DAL_KDI_STATUS_OUT_OF_RESOURCE          0xA06
#define DAL_KDI_STATUS_MAX_SESSIONS_REACHED     0xA07
#define DAL_KDI_STATUS_UNCAUGHT_EXCEPTION       0xA08
#define DAL_KDI_STATUS_WD_TIMEOUT               0xA09
#define DAL_KDI_STATUS_APPLET_CRASHED           0xA0A
#define DAL_KDI_STATUS_TA_NOT_FOUND             0xA0B
#define DAL_KDI_STATUS_TA_EXIST                 0xA0C
#define DAL_KDI_STATUS_INVALID_ACP              0xA0D

#define DAL_KDI_INVALID_HANDLE    0

int dal_get_version_info(struct dal_version_info *version_info);

int dal_create_session(u64 *session_handle, const char *app_id,
		       const u8 *acp_pkg, size_t acp_pkg_len,
		       const u8 *init_param, size_t init_param_len);

int dal_send_and_receive(u64 session_handle, int command_id, const u8 *input,
			 size_t input_len, u8 **output, size_t *output_len,
			 int *response_code);

int dal_close_session(u64 session_handle);

int dal_uuid_parse(const char *uuid_str, uuid_t *uuid);

#endif /* _DAL_H_ */
