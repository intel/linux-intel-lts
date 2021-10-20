// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022, Intel Corporation. */

#include <linux/module.h>
#include <linux/network_proxy.h>
#include <linux/network_proxy_common.h>
#include "ishtp/client.h"
#include "ishtp/ishtp-dev.h"

/* Rx ring buffer pool size */
#define NP_CL_RX_RING_SIZE	32
#define NP_CL_TX_RING_SIZE	16

struct ishtp_cl_np {
	struct ishtp_cl_device *cl_device;
	struct ishtp_cl *ishtp_cl;
	bool command_done;
	wait_queue_head_t ishtp_np_wait;
	struct np_ipcdev np_ipcdev;
};

static const guid_t np_ishtp_guid = GUID_INIT(0x1586a9d4, 0xae85, 0x4f4c,
					      0x90, 0x72, 0x57, 0x56, 0x0b,
					      0xd5, 0x27, 0x1e);

static int np_ishtp_wait_for_response(struct ishtp_cl_np *ishtp_cl_np)
{
	if (ishtp_cl_np->command_done)
		return 0;

	wait_event_interruptible_timeout(ishtp_cl_np->ishtp_np_wait,
					 ishtp_cl_np->command_done, 3 * HZ);

	if (!ishtp_cl_np->command_done) {
		dev_err(&ishtp_cl_np->ishtp_cl->device->dev,
			"Timeout waiting for response from NP Agent\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int np_ishtp_cl_send(void *ipc_cl, void *msg, int size, bool async)
{
	struct ishtp_cl_np *ishtp_cl_np;
	struct ishtp_cl *ishtp_cl;
	int status;

	ishtp_cl = ipc_cl;
	ishtp_cl_np = (struct ishtp_cl_np *)ishtp_cl->client_data;

	if (!async)
		ishtp_cl_np->command_done = false;

	/* ishtp message send out */
	status = ishtp_cl_send(ishtp_cl, (uint8_t *)msg, size);

	if (!async && !status) {
		/* wait for message send completely */
		return np_ishtp_wait_for_response(ishtp_cl_np);
	}

	return status;
}

static void np_ishtp_report_bad_packet(struct ishtp_cl *ishtp_cl,
				       void *recv_buf, size_t cur_pos,
				       size_t payload_len)
{
	struct np_ipc_hdr *recv_msg = recv_buf;
	struct ishtp_cl_np *ishtp_cl_np;

	ishtp_cl_np = (struct ishtp_cl_np *)ishtp_cl->client_data;

	dev_err(&ishtp_cl_np->ishtp_cl->device->dev, "BAD packet %02X\n"
		"cur_pos=%u\n"
		"[%02X %02X %02X %02X]\n"
		"payload_len=%u\n"
		"is_response=%02X\n",
		recv_msg->command,
		(unsigned int)cur_pos,
		((unsigned char *)recv_msg)[0], ((unsigned char *)recv_msg)[1],
		((unsigned char *)recv_msg)[2], ((unsigned char *)recv_msg)[3],
		(unsigned int)payload_len,
		recv_msg->command & ~NP_CMD_MASK);
}

static void np_ishtp_process_recv(struct ishtp_cl *ishtp_cl, void *recv_buf,
				  size_t data_len)
{
	size_t payload_len, total_len, cur_pos;
	struct ishtp_cl_np *ishtp_cl_np;
	struct np_ipc_hdr *recv_msg;
	unsigned char *payload;

	ishtp_cl_np = (struct ishtp_cl_np *)ishtp_cl->client_data;

	payload = recv_buf + sizeof(struct np_ipc_hdr);
	total_len = data_len;
	cur_pos = 0;

	dev_dbg(&ishtp_cl_np->ishtp_cl->device->dev,
		"np ipc receive ():+++ len=%zu\n", data_len);

	do {
		if (cur_pos + sizeof(struct np_ipc_hdr) > total_len) {
			dev_err(&ishtp_cl_np->ishtp_cl->device->dev,
				"np ipc: error, received %u < data hdr %u\n",
				(unsigned int)data_len,
				(unsigned int)sizeof(struct np_ipc_hdr));
			ish_hw_reset(ishtp_cl->dev);
			break;
		}

		recv_msg = (struct np_ipc_hdr *)(recv_buf + cur_pos);
		payload_len = recv_msg->size;

		/* Sanity checks */
		if (cur_pos + payload_len + sizeof(struct np_ipc_hdr) >
		    total_len) {
			np_ishtp_report_bad_packet(ishtp_cl, recv_msg,
						   cur_pos, payload_len);
			ish_hw_reset(ishtp_cl->dev);
			break;
		}

		dev_dbg(&ishtp_cl_np->ishtp_cl->device->dev,  "%s %d\n",
			__func__, recv_msg->command & NP_CMD_MASK);

		if (recv_msg->command & NP_IS_RESPONSE) {
			/* response from NP Agent, ack network proxy command */
			ishtp_cl_np->command_done = true;
			wake_up_interruptible(&ishtp_cl_np->ishtp_np_wait);
			dev_dbg(&ishtp_cl_np->ishtp_cl->device->dev,
				"%s: Received response from Proxy Agent.\n",
				__func__);
		} else if (recv_msg->command >= NP_A2H_CMD_AGENT_READY &&
				recv_msg->command < NP_A2H_CMD_MAX) {
			/* command from NP Agent,
			 * pass to network proxy framework
			 */
			netprox_ipc_recv(recv_msg->command, payload,
					 payload_len);
		} else {
			/* unrecognized message */
			np_ishtp_report_bad_packet(ishtp_cl, recv_msg,
						   cur_pos, payload_len);
			ish_hw_reset(ishtp_cl->dev);
		}

		cur_pos += payload_len + sizeof(struct np_ipc_hdr);
		payload += payload_len + sizeof(struct np_ipc_hdr);

	} while (cur_pos < total_len);
}

static void np_ishtp_cl_event_cb(struct ishtp_cl_device *device)
{
	struct ishtp_cl *ishtp_cl = ishtp_get_drvdata(device);
	struct ishtp_cl_rb *rb_in_proc;
	size_t r_length;

	if (!ishtp_cl)
		return;

	while ((rb_in_proc = ishtp_cl_rx_get_rb(ishtp_cl)) != NULL) {
		if (!rb_in_proc->buffer.data)
			return;

		r_length = rb_in_proc->buf_idx;

		/* decide what to do with received data */
		np_ishtp_process_recv(ishtp_cl, rb_in_proc->buffer.data,
				      r_length);

		ishtp_cl_io_rb_recycle(rb_in_proc);
	}
}

static int np_ishtp_cl_init(struct ishtp_cl *ishtp_cl)
{
	struct ishtp_fw_client *fw_client;
	struct ishtp_cl_np *ishtp_cl_np;
	int status;

	ishtp_cl_np = (struct ishtp_cl_np *)ishtp_cl->client_data;

	status = ishtp_cl_link(ishtp_cl);
	if (status) {
		dev_err(&ishtp_cl_np->cl_device->dev,
			"ishtp_cl_link failed\n");
		return	-ENOMEM;
	}

	/* Connect to IPC of Network Proxy Agent */
	ishtp_cl->rx_ring_size = NP_CL_RX_RING_SIZE;
	ishtp_cl->tx_ring_size = NP_CL_TX_RING_SIZE;

	fw_client = ishtp_fw_cl_get_client(ishtp_cl->dev, &np_ishtp_guid);
	if (!fw_client) {
		dev_err(&ishtp_cl_np->cl_device->dev,
			"Network Proxy Agent uuid not found\n");
		return -ENOENT;
	}

	ishtp_cl->fw_client_id = fw_client->client_id;
	ishtp_cl->state = ISHTP_CL_CONNECTING;

	status = ishtp_cl_connect(ishtp_cl);
	if (status) {
		dev_err(&ishtp_cl_np->cl_device->dev,
			"Network Proxy Agent connect fail\n");
		goto err_cl_unlink;
	}

	dev_dbg(&ishtp_cl_np->cl_device->dev,
		"Network Proxy Agent connected\n");

	/* Register read callback */
	ishtp_register_event_cb(ishtp_cl->device, np_ishtp_cl_event_cb);

	dev_dbg(&ishtp_cl_np->cl_device->dev, "successfully init\n");
	return 0;

err_cl_unlink:
	ishtp_cl_unlink(ishtp_cl);
	return status;
}

static void np_ishtp_cl_deinit(struct ishtp_cl *ishtp_cl)
{
	ishtp_cl_unlink(ishtp_cl);
	ishtp_cl_flush_queues(ishtp_cl);

	/* disband and free all Tx and Rx client-level rings */
	ishtp_cl_free(ishtp_cl);
}

static int np_ishtp_cl_probe(struct ishtp_cl_device *cl_device)
{
	struct ishtp_cl_np *ishtp_cl_np;
	struct ishtp_cl *ishtp_cl;
	int status;

	if (!cl_device)
		return	-ENODEV;

	ishtp_cl_np = devm_kzalloc(&cl_device->dev, sizeof(*ishtp_cl_np),
				   GFP_KERNEL);
	if (!ishtp_cl_np)
		return -ENOMEM;

	ishtp_cl = ishtp_cl_allocate(cl_device);
	if (!ishtp_cl)
		return -ENOMEM;

	ishtp_set_drvdata(cl_device, ishtp_cl);
	ishtp_cl->client_data = ishtp_cl_np;
	ishtp_cl_np->ishtp_cl = ishtp_cl;
	ishtp_cl_np->cl_device = cl_device;

	init_waitqueue_head(&ishtp_cl_np->ishtp_np_wait);

	status = np_ishtp_cl_init(ishtp_cl);

	if (status) {
		ishtp_cl_free(ishtp_cl);
		return status;
	}

	ishtp_get_device(cl_device);

	/* Register IPC device with Network Proxy Framework */
	ishtp_cl_np->np_ipcdev.ipc_cl = ishtp_cl;
	ishtp_cl_np->np_ipcdev.ipc_send = &np_ishtp_cl_send;
	netprox_register_ipcdev(&ishtp_cl_np->np_ipcdev);

	device_wakeup_enable(cl_device->ishtp_dev->devc);

	return 0;
}

static void np_ishtp_cl_remove(struct ishtp_cl_device *cl_device)
{
	struct ishtp_cl *ishtp_cl = ishtp_get_drvdata(cl_device);

	ishtp_cl->state = ISHTP_CL_DISCONNECTING;
	ishtp_cl_disconnect(ishtp_cl);
	ishtp_put_device(cl_device);
	np_ishtp_cl_deinit(ishtp_cl);

	ishtp_cl = NULL;
}

static struct ishtp_cl_driver np_ishtp_cl_driver = {
	.name = "ishtp-network-proxy",
	.guid = &np_ishtp_guid,
	.probe = np_ishtp_cl_probe,
	.remove = np_ishtp_cl_remove,
};

static int __init np_ishtp_ipc_init(void)
{
	int status;

	/* Register ISHTP client device driver with ISHTP Bus */
	status = ishtp_cl_driver_register(&np_ishtp_cl_driver, THIS_MODULE);

	return status;
}

static void __exit np_ishtp_ipc_deinit(void)
{
	ishtp_cl_driver_unregister(&np_ishtp_cl_driver);
}

module_init(np_ishtp_ipc_init);
module_exit(np_ishtp_ipc_deinit);

MODULE_DESCRIPTION("ISHTP Network Proxy");
MODULE_AUTHOR("Song Yoong Siang <yoong.siang.song@intel.com>");
MODULE_AUTHOR("Ong Boon Leong <boon.leong.ong@intel.com>");

MODULE_LICENSE("GPL");
MODULE_ALIAS("ishtp:*");
