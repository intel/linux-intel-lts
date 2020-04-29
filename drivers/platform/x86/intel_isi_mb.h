/*
 * PCI glue for ISI driver
 *
 * Copyright (c) 2019, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#ifndef ISI_MB_COMM_H
#define ISI_MB_COMM_H

#include <linux/types.h>
#include <linux/unistd.h>

#define DEVICE_NAME "ISI_DRIVER"

#define REBOOT_REQ_CMD (0x601U)
#define TIMER_EXPIRY_CMD (0x606U)
#define NOK_WARN_NOTIFY_CMD (0x618U)
#define DIAG_DATA_CMD (0x660U)
#define START_DTI_CMD (0x667U)
#define IEH_ERR_GET_CMD (0x67EU)
#define WL_REG_CMD (0x34FU)
#define WL_DE_REG_CMD (0x356U)
#define ODCC_SNAPSHOT_RESP (0x6A8U)
#define BROADCAST_ID (0xFFU)

enum isi_driver_instance {
	MAILBOX_INST_0,
	/* MAILBOX_INST_1,
		   MAILBOX_INST_2,
		   MAILBOX_INST_3,*/
	NUM_OF_ISI_DRV_INST
};

struct isi_generic_packet_header {
	uint32_t version : 4;
	uint32_t reserved0 : 4;
	uint32_t flags_reserved : 6;
	uint32_t flag_response_bit : 1;
	uint32_t flag_retry_bit : 1;

	uint32_t length : 8; /**< In Dwords. */
	uint32_t reserved1 : 8;
	uint32_t sequence_number : 16;
	uint32_t wl_address : 8;
	uint32_t isi_address : 8;
	uint32_t crc_16 : 16;
	uint32_t command : 16;
};

/**
 * These all requests are coming from ISI to HOST.
 */
enum icul_asynch_request_type {
	/**
     * This command will tell host to reboot itself.
     * Applies to: SCI/SNI*/
	REQ_HOST_REBOOT,

	/**
     * Whenever WCET expires then ISI notifies HOST the timer expiry. This
     * command also returns the payload with it.
     * Applies to: SCI */
	REQ_TIMER_EXPIRY_NOTIFY,

	/**
     * 	Whenever ISI generates NOK to safet supervisor, it will be notified with
     * this request. If the error logs are there, it will be sent with request.
     * This command also returns the payload with it.
     * 	Applies to: SCI/SNI */
	REQ_NOK_WARNING_NOTIFY,

	/**
     *		When  ISI sends the diagnostic data to host. Sometimes, the
     *diagnostic data is not immediately available when NOK/Warning is issued.
     *Under this situation, this command is invoked. This command also returns
     *the payload with it. Applies to: SCI */
	REQ_DIAG_DATA_TO_HOST,

	/**
     * This service is used by SCI to inform host safety workload to trigger
     *    the periodic STL execution. This is a broadcast message from SCI and
     * no response is expected by SCI. Host software stack has to ensure that
     * any safety workloads that needs this message receives it. WL can use API
     *     provided by ICUL to register with host ISI driver to forward this
     * message to it. Applies to: SCI */
	REQ_START_DTI,

	/**
     * Whenever ERR_0 or ERR1 is asserted, host is notified with this command,
     *      HOST can respond by calling icul_ieh_error_sen() API
     * Applies to: SCI */
	REQ_IEH_ERROR_GET,

	REQ_ODCC_SNAPSHOT,

	NUMBER_OF_ASYNCH_REQUEST_TYPES,

};

/*
 * This structure should be passed to
 * workload register and deregister commands.
 * In case of de-register command, buffer and size will be avoided.
 */
struct isi_workload_details {
	enum isi_driver_instance driver_inst;
	uint32_t workload_id;
	pid_t pid;
	struct isi_generic_packet_header resp_header;
	struct isi_generic_packet_header req_header;
	uint32_t resp_buffer_size; /**< In number of DWords. */
	uint32_t *resp_buffer;
};

/*
 * This structure should be passed to asynchronous command register
 * and deregister commands.
 */
struct isi_asynch_cmd_request {
	enum isi_driver_instance driver_inst;
	uint32_t workload_id;
	uint32_t number_of_requests;
	enum icul_asynch_request_type *requests;
};

/*
 * This structure can be passed to synchronous read or send command.
 * In case of READ command, header_buffer and header size will be avoided.
 */
struct isi_asynch_data {
	enum isi_driver_instance driver_inst;
	uint32_t workload_id;
	enum icul_asynch_request_type request_type;
	struct isi_generic_packet_header header_buffer;
	uint32_t data_size;
	uint32_t *data_buffer;
};

struct isi_synch_data_request {
	enum isi_driver_instance driver_inst;
	uint32_t workload_id;
	uint32_t timeout;
	uint32_t is_resp_required;

	struct isi_generic_packet_header req_header;
	struct isi_generic_packet_header resp_header;

	uint32_t req_buffer_size;
	uint32_t resp_buffer_size;

	uint32_t *req_buffer;
	uint32_t *resp_buffer;
};

#define ISI_DRIVER_WORKLOAD_REGISTER _IOWR('i', 1, struct isi_workload_details)

#define ISI_DRIVER_WORKLOAD_DE_REGISTER                                        \
	_IOR('i', 2, struct isi_workload_details)

#define ISI_DRIVER_ASYNCH_CMD_REGISTER                                         \
	_IOW('i', 3, struct isi_asynch_cmd_request)

#define ISI_DRIVER_ASYNCH_CMD_DE_REGISTER                                      \
	_IOW('i', 4, struct isi_asynch_cmd_request)

#define ISI_DRIVER_SYNCH_CMD_TX_RX _IOWR('i', 5, struct isi_synch_data_request)

#define ISI_DRIVER_ASYNCH_MESSAGE_GET _IOR('i', 6, struct isi_asynch_data)

#endif /* ISI_MB_COMM_H */
