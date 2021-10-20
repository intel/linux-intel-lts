/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022, Intel Corporation. */

#ifndef __NETWORK_PROXY_COMMON_H__
#define __NETWORK_PROXY_COMMON_H__

#define NP_CMD_MASK			0x7F
#define NP_IS_RESPONSE			0x80

/* IPC Message and Payload Size Limit */
#define NP_IPC_MSG_MAX		256
#define NP_IPC_PYLD_MAX		(NP_IPC_MSG_MAX - sizeof(struct np_ipc_hdr))

/* Network Proxy Agent to Host Packet Size Limit */
#define NP_A2H_PKT_MAX			512

#define NP_MAC_ADDR_BYTES		6
#define NP_IPV4_ADDR_BYTES		4
#define NP_IPV6_ADDR_ARRAY		10
#define NP_IPV6_ADDR_BYTES		16
#define NP_TCP_PORT_ARRAY		16

/* Network Proxy IPC Message Header */
struct np_ipc_hdr {
	unsigned char command; /* Bit 7 : is_response */
	unsigned char status;
	unsigned short size;
};

/* Network Proxy IPC Message */
struct np_ipc_msg {
	struct np_ipc_hdr ipc_hdr;
	char ipc_pyld[NP_IPC_PYLD_MAX];
};

/* MIB for Network Proxy Agent */
struct np_mib {
	unsigned char mac_addr[NP_MAC_ADDR_BYTES];
	unsigned char ipv4_addr[NP_IPV4_ADDR_BYTES];
	unsigned char ipv6_addr[NP_IPV6_ADDR_ARRAY][NP_IPV6_ADDR_BYTES];
	unsigned short tcp_port[NP_TCP_PORT_ARRAY];
	unsigned int ipv6_addr_sz;
	unsigned int tcp_port_sz;
};

/* Commands from Network Proxy Host to Agent */
enum np_h2a_cmd {
	/* Network Device for Network Proxy is ready */
	NP_H2A_CMD_NETDEV_READY = 1,
	/* Enter Network Proxy Mode */
	NP_H2A_CMD_PROXY_ENTER,
	/* Exit Network Proxy Mode */
	NP_H2A_CMD_PROXY_EXIT,
	NP_H2A_CMD_MAX,
};

/* Commands from Network Proxy Agent to Host */
enum np_a2h_cmd {
	/* Network Proxy Agent is ready */
	NP_A2H_CMD_AGENT_READY = 1,
	/* Is Host Awake? */
	NP_A2H_CMD_HOST_IS_AWAKE,
	/* Network Proxy Mode Exited */
	NP_A2H_CMD_HOST_IS_EXITED,
	NP_A2H_CMD_MAX,
};

/* Network Proxy Agent to Host Packet Passing Memory Design.
 *
 * A2H Memory Pool Header Format:-
 * ===============================
 *         2-byte              2-byte
 * +-------------------+--------------------+
 * |   total_packets   |     total_size     |
 * +-------------------+--------------------+
 * Whereby:-
 *    a) total_packets: Total number of A2P packets
 *    b) total_size:    Total memory size to be passed from Agent Host
 *                      including the 4-byte A2H Memory Pool Header and
 *                      Total_packet * (A2H Per-packet Header Format +
 *                      NP_A2H_PKT_MAX).
 *
 * When Agent passes packets to Host, the packets may be different in length.
 * Each of a packet is kept following the below A2P Per-packet format.
 *
 * A2H Per-packet Format:-
 * =======================
 *         2-byte              2-byte
 *  +-------------------+--------------------+
 *  |                pkt_desc                |
 *  +-------------------+--------------------+
 *  |                pkt_len                 |
 *  +-------------------+--------------------+
 *  |                pkt_info1               |
 *  +-------------------+--------------------+
 *  |                pkt_info2               |
 *  +-------------------+--------------------+
 *  |             Packet Content             |
 *  +-------------------+--------------------+
 *  |             Packet Content             |
 *  +-------------------+--------------------+
 *  |             Packet Content             |
 *  +-------------------+--------------------+
 *  Whereby:-
 *    a) pkt_desc: Packet descriptor for this packet.
 *    b) pkt_len:  The length (in Byte) of a packet, the maximum size of
 *                 packet content is limited by NP_A2H_PKT_MAX.
 *    c) pkt_info: Extra information about this packet.
 *    d) Packet content: The actual network packet to be passed from
 *                       Network Proxy Agent to Host.
 *                       Note: the actual length of a packet may vary but
 *                       it is always smaller than NP_A2H_PKT_MAX
 */
struct np_a2h_pool_header {
	unsigned short total_packets;
	unsigned short total_size;
};

struct np_a2h_packet_header {
	unsigned int pkt_desc;
	unsigned int pkt_len;
	unsigned int pkt_info1;
	unsigned int pkt_info2;
};

#endif /* __NETWORK_PROXY_COMMON_H__ */
