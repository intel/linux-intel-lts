/* SPDX-License-Identifier: (GPL-2.0 OR Apache-2.0) */
/*
 * Network Proxy Common Definitions between Network Proxy Host and Agent.
 *
 * GPL-2.0
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 */
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

/* Network Proxy Agent to Host Packet Size Limit
 * TODO: support bigger packet; partition memory pool
 */
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
 *                 TODO: to be expanded in future.
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
