/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022, Intel Corporation. */

#ifndef __NETWORK_PROXY_COMMON_H__
#define __NETWORK_PROXY_COMMON_H__

#define NP_CMD_MASK			0x7F
#define NP_IS_RESPONSE			0x80

/* Network Proxy Rules Group */
#define NP_RL_G_CLS			1
#define NP_RL_G_RSP			2
#define NP_RL_G_MIB			3

/* Network Proxy Rules Type */
#define NP_RL_T_PROGAMMABLE		1
#define NP_RL_T_MAC_ADDR		2
#define NP_RL_T_IPV4			3
#define NP_RL_T_IPV6			4
#define NP_RL_T_TCP_WAKE_PORT		5
#define NP_RL_T_UDP_WAKE_PORT		6
#define NP_RL_T_SNMP			7
#define NP_RL_T_SNMP_COMMUNITY_STR	8
#define NP_RL_T_SNMP_WRITE_OID_TREE	9
#define NP_RL_T_SNMP_READ_OID_TREE	10
#define NP_RL_T_MDNS			11
#define NP_RL_T_MDNS_WRITE_RR		12
#define NP_RL_T_MDNS_READ_RR		13
#define NP_RL_T_IPV4_SUBNET		14

/* Network Proxy Fixed Classifier Value (2 bytes) */
#define NP_RL_CLS_ENABLE		BIT(15)
#define NP_RL_CLS_DROP			BIT(1)
#define NP_RL_CLS_RESP			BIT(2)
#define NP_RL_CLS_WAKE			BIT(3)
#define NP_RL_CLS_A2H			BIT(4)
/* Wake-up host when SNMP GetRequest packet with unknown OID is received */
#define NP_RL_CLS_SUPP_SNMP_GALL	BIT(5)
/* Wake-up host when SNMP SetRequest packet is received */
#define NP_RL_CLS_SUPP_SNMP_SET		BIT(6)

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
	unsigned char ipv4_subnet[NP_IPV4_ADDR_BYTES];
};

/* Network Proxy Agent Info */
struct np_agent_info {
	/* Version major.minor.revision */
	unsigned short major;
	unsigned short minor;
	unsigned short revision;
	unsigned short resv;
	/* Max # Classifier Rules */
	unsigned int max_cls_rules;
	/* Max # Responder Rules */
	unsigned int max_resp_rules;
};

/* Network Proxy Rules Info */
struct np_rules {
	/* Group NP_RL_G */
	unsigned short group;
	/* Type NP_RL_T */
	unsigned short type;
	/* Offset in Byte */
	unsigned int offset;
	/* Size in Byte */
	unsigned int size;
	/* Content */
	unsigned int value[0];
};

/* Commands from Network Proxy Host to Agent */
enum np_h2a_cmd {
	/* Network Device for Network Proxy is ready */
	NP_H2A_CMD_NETDEV_READY = 1,
	/* Enter Network Proxy Mode */
	NP_H2A_CMD_PROXY_ENTER,
	/* Exit Network Proxy Mode */
	NP_H2A_CMD_PROXY_EXIT,
	/* Read/Write Classifier Rule */
	NP_H2A_CMD_READ_CLS_RULE,
	NP_H2A_CMD_WRITE_CLS_RULE,
	/* Read/Write Responder rule */
	NP_H2A_CMD_READ_RESP_RULE,
	NP_H2A_CMD_WRITE_RESP_RULE,
	NP_H2A_CMD_MAX,
};

/* Commands from Network Proxy Agent to Host */
enum np_a2h_cmd {
	/* Network Proxy Agent is ready */
	NP_A2H_CMD_AGENT_READY = 1,
	/* Network Proxy Agent Firmware Version and Info */
	NP_A2H_CMD_AGENT_INFO,
	/* Network Proxy Reply Rule Result */
	NP_A2H_CMD_READ_CLS_RULE_RESULT,
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
