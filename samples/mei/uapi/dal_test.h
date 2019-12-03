/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright(c) 2016-2019 Intel Corporation.
 */

#ifndef KDI_CMD_DEFS_H
#define KDI_CMD_DEFS_H

/**
 * enum kdi_command_id - cmd id to invoke in kdi module
 *
 * @KDI_SESSION_CREATE: call kdi "create session" function
 * @KDI_SESSION_CLOSE: call kdi "close session" function
 * @KDI_SEND_AND_RCV: call kdi "send and receive" function
 * @KDI_VERSION_GET_INFO: call kdi "get version" function
 * @KDI_EXCLUSIVE_ACCESS_SET: call kdi "set exclusive access" function
 * @KDI_EXCLUSIVE_ACCESS_REMOVE: call kdi "unset exclusive access" function
 */
enum kdi_command_id {
	KDI_SESSION_CREATE,
	KDI_SESSION_CLOSE,
	KDI_SEND_AND_RCV,
	KDI_VERSION_GET_INFO,
	KDI_EXCLUSIVE_ACCESS_SET,
	KDI_EXCLUSIVE_ACCESS_REMOVE
};

/**
 * struct kdi_test_command - contains the command received from user space
 *
 * @cmd_id: the command id
 * @data: the command data
 */
struct kdi_test_command {
	__u8 cmd_id;
	unsigned char data[0];
} __packed;

/**
 * struct session_create_cmd - create session cmd data
 *
 * @app_id_len: length of app_id arg
 * @acp_pkg_len: length of the acp_pkg arg
 * @init_param_len: length of init param arg
 * @is_session_handle_ptr: either send kdi a valid ptr to hold the
 *                         session handle or NULL
 * @data: buffer to hold the cmd arguments
 */
struct session_create_cmd {
	__u32 app_id_len;
	__u32 acp_pkg_len;
	__u32 init_param_len;
	__u8 is_session_handle_ptr;
	unsigned char data[0];
} __packed;

/**
 * struct session_create_resp - create session response
 *
 * @session_handle: the session handle
 * @test_mod_status: status returned from the test module
 * @status: status returned from kdi
 */
struct session_create_resp {
	__u64 session_handle;
	__s32 test_mod_status;
	__s32 status;
} __packed;

/**
 * struct session_close_cmd - close session cmd
 *
 * @session_handle: the session handle to close
 */
struct session_close_cmd {
	__u64 session_handle;
} __packed;

/**
 * struct session_close_resp - close session response
 *
 * @test_mod_status: status returned from the test module
 * @status: status returned from kdi
 */
struct session_close_resp {
	__s32 test_mod_status;
	__s32 status;
} __packed;

/**
 * struct send_and_rcv_cmd - send and receive cmd
 *
 * @session_handle: the session handle
 * @command_id: the cmd id to send the applet
 * @output_buf_len: the size of the output buffer
 * @is_output_buf: either send kdi a valid ptr to hold the output buffer or NULL
 * @is_output_len_ptr: either send kdi a valid ptr to hold
 *                     the output len or NULL
 * @is_response_code_ptr: either send kdi a valid ptr to hold
 *                        the applet response code or NULL
 * @input: the input data to send the applet
 */
struct send_and_rcv_cmd {
	__u64 session_handle;
	__u32 command_id;
	__u32 output_buf_len;
	__u8 is_output_buf;
	__u8 is_output_len_ptr;
	__u8 is_response_code_ptr;
	unsigned char input[0];
} __packed;

/**
 * struct send_and_rcv_resp - send and receive response
 *
 * @test_mod_status: status returned from the test module
 * @status: status returned from kdi
 * @response_code: response code returned from the applet
 * @output_len: length of output from the applet
 * @output: the output got from the applet
 */
struct send_and_rcv_resp {
	__s32 test_mod_status;
	__s32 status;
	__s32 response_code;
	__u32 output_len;
	unsigned char output[0];
} __packed;

/**
 * struct version_get_info_cmd - get version cmd
 *
 * @is_version_ptr: either send kdi a valid ptr to hold the version info or NULL
 */
struct version_get_info_cmd {
	__u8 is_version_ptr;
} __packed;

/**
 * struct version_get_info_resp - get version response
 *
 * @kdi_version: kdi version
 * @reserved: reserved bytes
 * @test_mod_status: status returned from the test module
 * @status: status returned from kdi
 */
struct version_get_info_resp {
	char kdi_version[32];
	__u32 reserved[4];
	__s32 test_mod_status;
	__s32 status;
} __packed;

/**
 * struct ta_access_set_remove_cmd - set/remove access cmd
 *
 * @app_id_len: length of app_id arg
 * @data: the cmd data. contains the app_id
 */
struct ta_access_set_remove_cmd {
	__u32 app_id_len;
	unsigned char data[0];
} __packed;

/**
 * struct ta_access_set_remove_resp - set/remove access response
 *
 * @test_mod_status: status returned from the test module
 * @status: status returned from kdi
 */
struct ta_access_set_remove_resp {
	__s32 test_mod_status;
	__s32 status;
} __packed;

#endif /* KDI_CMD_DEFS_H */
