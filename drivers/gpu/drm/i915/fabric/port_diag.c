// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2021 - 2023 Intel Corporation.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/stringify.h>
#include <linux/uaccess.h>

#include "debugfs.h"
#include "ops.h"
#include "port.h"
#include "port_diag.h"

static const char *port_health_name(enum fport_health s)
{
	switch (s) {
	case FPORT_HEALTH_OFF:      return "OFF";
	case FPORT_HEALTH_FAILED:   return "FAILED";
	case FPORT_HEALTH_DEGRADED: return "DEGRADED";
	case FPORT_HEALTH_HEALTHY:  return "HEALTHY";
	}

	return "UNKNOWN";
}

static const char *fw_phys_state_name(u8 s)
{
	static const char * const names[] = { "NOP", "UNKNOWN", "POLLING", "DISABLED", "TRAINING",
					      "LINKUP", "LINK ERROR RECOVERY", "UNKNOWN", "UNKNOWN",
					      "OFFLINE", "UNKNOWN", "TEST" };

	if (s >= ARRAY_SIZE(names))
		return "UNKNOWN";
	else
		return names[s];
}

static const char *link_width_name(u8 s)
{
	static const char * const names[] = { "N/A", "1X", "2X", "N/A", "3X", "N/A", "N/A", "N/A",
					      "4X" };

	if (s >= ARRAY_SIZE(names))
		return "N/A";
	else
		return names[s];
}

static const char *link_speed_name(u8 s)
{
	static const char * const names[] = { "N/A", "12G", "25G", "N/A", "53G", "N/A", "N/A",
					      "N/A", "90G" };

	if (s >= ARRAY_SIZE(names))
		return "N/A";
	else
		return names[s];
}

static const char *crc_mode_name(u8 s)
{
	static const char * const names[] = { "16b", "14b", "48b", "pLn" };

	if (s >= ARRAY_SIZE(names))
		return "N/A";
	else
		return names[s];
}

static const char *fec_mode_name(u8 s)
{
	static const char * const names[] = { "NONE",
					      "F132",
					      "F528",
					      "UNKNOWN" };
	if (s >= ARRAY_SIZE(names))
		return "N/A";
	else
		return names[s];
}

static const char *port_type_name(u8 s)
{
	static const char * const names[] = { "UNKNOWN", "DISCONNECTED", "FIXED", "VARIABLE",
					      "STANDARD", "SI_PHOTONICS" };

	if (s >= ARRAY_SIZE(names))
		return "UNKNOWN";
	else
		return names[s];
}

/*
 * port_show debugfs functions
 */

#define PORT_SHOW_FILE_NAME "port_show"
#define PORT_SHOW_BUF_SIZE 1024

enum port_show_csr_rsp_idx {
	TP_PRF_XMIT_PKTS_OFFSET_IDX,
	O_FPC_PORTRCV_PKT_CNT_IDX,
	O_LCB_STS_RX_CRC_FEC_MODE_IDX,
	O_LCB_ERR_INFO_SEQ_CRC_CNT_IDX,
	O_LCB_ERR_INFO_REINIT_FROM_PEER_CNT_IDX,
	O_LCB_ERR_INFO_TX_REPLAY_CNT_IDX,
	O_LCB_ERR_INFO_RX_REPLAY_CNT_IDX,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_1,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_2,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_3,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_4,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_5,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_6,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_7,
	O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_8,
	_RSP_IDX_COUNT,
};

struct port_show_query_fw_rsp {
	DECLARE_MBDB_OP_PORT_STATUS_GET_RSP(port_status_op, _RSP_IDX_COUNT);
	DECLARE_MBDB_OP_PORTINFO(portinfo_op, 1);
};

static int port_show_query_fw(struct fsubdev *sd, u8 lpn, struct port_show_query_fw_rsp *rsp)
{
	static const struct mbdb_op_csr_range port_show_csr_ranges_b0[] = {
		{ .offset = TP_PRF_XMIT_PKTS_OFFSET, .num_csrs = 1, },
		{ .offset = O_FPC_PORTRCV_PKT_CNT, .num_csrs = 1, },
		{ .offset = O_LCB_STS_RX_CRC_FEC_MODE, .num_csrs = 1, },
		{ .offset = O_LCB_ERR_INFO_SEQ_CRC_CNT, .num_csrs = 1, },
		{ .offset = O_LCB_ERR_INFO_REINIT_FROM_PEER_CNT, .num_csrs = 1, },
		{ .offset = O_LCB_ERR_INFO_TX_REPLAY_CNT, .num_csrs = 2, },
		{ .offset = O_LCB_ERR_INFO_FEC_CERR_CNT_1, .num_csrs = 8, }
	};
	const struct mbdb_op_csr_range *csr_ranges = port_show_csr_ranges_b0;
	u8 num_csr_ranges = ARRAY_SIZE(port_show_csr_ranges_b0);

	if (ops_port_status_get(sd, lpn, num_csr_ranges, csr_ranges,
				&rsp->port_status_op))
		return -EINVAL;

	if (ops_portinfo_get(sd, 1U << lpn, &rsp->portinfo_op))
		return -EINVAL;

	signal_pm_thread(sd, RESCAN_EVENT);

	return 0;
}

struct port_show_stats {
	u64 rx_fec_correction_count;
	u64 error_recovery_count;
	u64 tx_packet_count;
	u64 rx_packet_count;
	u64 tx_replay_count;
	u64 rx_replay_count;
	u64 neighbor_guid;
	s64 linkup_ms;
	struct fport_status status;
	u32 link_down_count;
	u8 lpn;
	u8 port_type;
	u8 log_state;
	u8 phys_state;
	u8 crc_mode;
	u8 fec_mode;
	u8 link_tx_width_active;
	u8 link_rx_width_active;
	u8 link_speed_active;
	u8 lqi;
	u8 neighbor_port_number;
};

static int port_show_get_stats(struct fsubdev *sd, u8 lpn, struct port_show_stats *stats)
{
	struct port_show_query_fw_rsp rsp = {};
	struct portinfo *port_info = rsp.portinfo_op.per_portinfo;
	u64 *regs = rsp.port_status_op.regs;

	if (port_show_query_fw(sd, lpn, &rsp))
		return -EINVAL;

	if (get_fport_status(sd, lpn, &stats->status))
		return -EINVAL;

	stats->lpn = lpn;
	stats->log_state = FIELD_GET(PS_PPS_PORT_STATE, port_info->port_state_port_physical_state);
	stats->phys_state = FIELD_GET(PS_PPS_PHYSICAL_STATE,
				      port_info->port_state_port_physical_state);
	stats->port_type = port_info->port_type;

	stats->link_down_count = port_info->link_down_count;
	stats->lqi = FIELD_GET(OLDR_NN_LQI_LINK_QUALITY_INDICATOR, port_info->oldr_nn_lqi);

	if (stats->log_state > IAF_FW_PORT_DOWN) {
		stats->link_tx_width_active = port_info->link_width_downgrade_tx_active;
		stats->link_rx_width_active = port_info->link_width_downgrade_rx_active;
		stats->link_speed_active = port_info->link_speed_active;
		stats->crc_mode = FIELD_GET(CRC_MODE, regs[O_LCB_STS_RX_CRC_FEC_MODE_IDX]);
		if (stats->crc_mode > LINK_CRC_MODE_48B)
			stats->crc_mode = LINK_CRC_MODE_PLN;
		stats->fec_mode = FIELD_GET(FEC_MODE, regs[O_LCB_STS_RX_CRC_FEC_MODE_IDX]);
		if (stats->fec_mode > LINK_FEC_MODE_UNKNOWN)
			stats->fec_mode = LINK_FEC_MODE_UNKNOWN;
		stats->neighbor_guid = port_info->neighbor_guid;
		stats->neighbor_port_number = port_info->neighbor_port_number;
		if (stats->status.linkup_since)
			stats->linkup_ms = ms_elapsed_since_boot() - stats->status.linkup_since;
		else
			stats->linkup_ms = 0;
	} else {
		stats->link_tx_width_active = 0xFF;
		stats->link_rx_width_active = 0xFF;
		stats->link_speed_active = 0xFF;
		stats->crc_mode = 0xFF;
		stats->fec_mode = 0xFF;
		stats->neighbor_guid = 0;
		stats->neighbor_port_number = 0xFF;
		stats->linkup_ms = 0;
	}

	stats->tx_packet_count = regs[TP_PRF_XMIT_PKTS_OFFSET_IDX];
	stats->rx_packet_count = regs[O_FPC_PORTRCV_PKT_CNT_IDX];
	stats->error_recovery_count = regs[O_LCB_ERR_INFO_SEQ_CRC_CNT_IDX] +
				      regs[O_LCB_ERR_INFO_REINIT_FROM_PEER_CNT_IDX];
	stats->tx_replay_count = regs[O_LCB_ERR_INFO_TX_REPLAY_CNT_IDX];
	stats->rx_replay_count = regs[O_LCB_ERR_INFO_RX_REPLAY_CNT_IDX];
	stats->rx_fec_correction_count = regs[O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_1] +
					 regs[O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_2];

	if (stats->fec_mode == LINK_FEC_MODE_F528) {
		int i;

		for (i = O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_3;
		     i <= O_LCB_ERR_INFO_FEC_CERR_CNT_IDX_8; i++)
			stats->rx_fec_correction_count += regs[i];
	}

	return 0;
}

static size_t port_show_print_stats(struct port_show_stats *stats, char *buf)
{
	size_t buf_offset = 0;

	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %u\n", "Port Number",
		   stats->lpn);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "FW Phys State",
		   fw_phys_state_name(stats->phys_state));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "FW Link State",
		   fw_log_state_name(stats->log_state));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Port Health",
		   port_health_name(stats->status.health));

	if (stats->status.health == FPORT_HEALTH_FAILED) {
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n",
			   "Port Error Failed",
			   test_bit(FPORT_ERROR_FAILED, stats->status.errors) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n",
			   "Port Error Isolated",
			   test_bit(FPORT_ERROR_ISOLATED, stats->status.errors) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n",
			   "Port Error Flapping",
			   test_bit(FPORT_ERROR_FLAPPING, stats->status.errors) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n",
			   "Port Error Link Down",
			   test_bit(FPORT_ERROR_LINK_DOWN, stats->status.errors) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n",
			   "Port Error Did Not Train",
			   test_bit(FPORT_ERROR_DID_NOT_TRAIN, stats->status.errors) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n",
			   "Port Error Loopback",
			   test_bit(FPORT_ERROR_LOOPBACK, stats->status.errors) ? "Y" : "N");
	}

	if (stats->status.health == FPORT_HEALTH_DEGRADED) {
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Port Issue LQI",
			   test_bit(FPORT_ISSUE_LQI, stats->status.issues) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Port Issue LWD",
			   test_bit(FPORT_ISSUE_LWD, stats->status.issues) ? "Y" : "N");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Port Issue RATE",
			   test_bit(FPORT_ISSUE_RATE, stats->status.issues) ? "Y" : "N");
	}

	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Tx Width",
		   link_width_name(stats->link_tx_width_active));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Rx Width",
		   link_width_name(stats->link_rx_width_active));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Link Speed",
		   link_speed_name(stats->link_speed_active));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "CRC Mode",
		   crc_mode_name(stats->crc_mode));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "FEC Mode",
		   fec_mode_name(stats->fec_mode));
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %llu\n", "TX Packet Count",
		   stats->tx_packet_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %llu\n", "RX Packet Count",
		   stats->rx_packet_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %u\n", "Link Down Count",
		   stats->link_down_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %llu\n", "Error Recovery Count",
		   stats->error_recovery_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %llu\n", "TX Replays",
		   stats->tx_replay_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %llu\n", "RX Replays",
		   stats->rx_replay_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %llu\n", "RX FEC Corr Count",
		   stats->rx_fec_correction_count);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %u\n", "Link Quality",
		   stats->lqi);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %s\n", "Port Type",
		   port_type_name(stats->port_type));
	if (stats->neighbor_port_number != 0xFF) {
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : 0x%0llx\n",
			   "Neighbor GUID", stats->neighbor_guid);
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %u\n",
			   "Neighbor Port Number", stats->neighbor_port_number);
	} else {
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : N/A\n", "Neighbor GUID");
		print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : N/A\n",
			   "Neighbor Port Number");
	}
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "%-24s : %lld\n", "Link Up Milliseconds",
		   stats->linkup_ms);
	print_diag(buf, &buf_offset, PORT_SHOW_BUF_SIZE, "\n");

	return buf_offset;
}

static int port_show_open(struct inode *inode, struct file *file)
{
	struct fport *port = inode->i_private;
	struct port_show_stats stats;
	struct port_show_info {
		struct debugfs_blob_wrapper blob;
		char buf[PORT_SHOW_BUF_SIZE];
	} *info;

	if (!port)
		return -EINVAL;

	if (port_show_get_stats(port->sd, port->lpn, &stats))
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->blob.data = info->buf;
	info->blob.size = port_show_print_stats(&stats, info->blob.data);
	file->private_data = info;

	return 0;
}

static const struct file_operations port_show_fops = {
	.owner = THIS_MODULE,
	.open = port_show_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define SERDES_HISTOGRAM_FILE_NAME "serdes_histogram"

#define LANE_COLUMN_WIDTH        9
#define NEWLINE_WIDTH            1
#define NULL_TERMINATOR_WIDTH    1
#define MAX_LINE_WIDTH           ((LANE_COLUMN_WIDTH * LANES) + NEWLINE_WIDTH)

#define SERDES_HISTOGRAM_HEADERS 2

#define LPN_HEADER_FMT   "Logical Port %d\n"
/* Each Lane # label is LANE_COLUMN_WIDTH in size and right justified */
#define LANE_HEADER_FMT  "   Lane 0   Lane 1   Lane 2   Lane 3\n"
#define DATA_ELEMENT_FMT "%" __stringify(LANE_COLUMN_WIDTH) "u"

#define HISTOGRAM_DISPLAY_BUF_SIZE ((MAX_LINE_WIDTH * SERDES_HISTOGRAM_HEADERS) + \
				    (MAX_LINE_WIDTH * LANE_DATA_ELEMENTS) + \
				    NULL_TERMINATOR_WIDTH)

static int serdes_histogram_open(struct inode *inode, struct file *file)
{
	struct fport *port = inode->i_private;
	struct serdes_histogram_info {
		struct debugfs_blob_wrapper blob;
		struct mbdb_serdes_histogram_rsp rsp;
		char buf[HISTOGRAM_DISPLAY_BUF_SIZE];
	} *info;
	struct mbdb_serdes_histogram_rsp *rsp;
	size_t buf_size;
	size_t buf_offset;
	char *buf;
	int lane;
	int data_element;
	int ret;

	if (!port)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	rsp = &info->rsp;

	ret = ops_serdes_histogram_get(port->sd, port->lpn, rsp);
	if (ret) {
		kfree(info);
		return ret;
	}

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;
	buf_offset = 0;

	print_diag(buf, &buf_offset, buf_size, LPN_HEADER_FMT, port->lpn);
	print_diag(buf, &buf_offset, buf_size, LANE_HEADER_FMT);

	for (data_element = 0; data_element < LANE_DATA_ELEMENTS; data_element++) {
		for (lane = 0; lane < LANES; lane++)
			print_diag(buf, &buf_offset, buf_size, DATA_ELEMENT_FMT,
				   rsp->lane[lane].data[data_element]);
		print_diag(buf, &buf_offset, buf_size, "\n");
	}

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations serdes_histogram_fops = {
	.owner = THIS_MODULE,
	.open = serdes_histogram_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define SERDES_EQINFO_FILE_NAME "serdes_eqinfo"

#define EQINFO_MAX_LINE_LENGTH 94
#define EQINFO_OUTPUT_LINES 113

#define EQINFO_DISPLAY_BUF_SIZE (EQINFO_OUTPUT_LINES * EQINFO_MAX_LINE_LENGTH)

#define EQINFO_LANE_HDR_FMT  "SerdesEqInfo       lane 0             lane 1             lane 2             lane 3\n"

#define EQINFO_8BIT_HDR_FMT  "%-16s %#-18hhx %#-18hhx %#-18hhx %#-18hhx\n"
#define EQINFO_16BIT_HDR_FMT "%-16s %#-18hx %#-18hx %#-18hx %#-18hx\n"
#define EQINFO_32BIT_HDR_FMT "%-16s %#-18x %#-18x %#-18x %#-18x\n"
#define EQINFO_64BIT_HDR_FMT "%-16s %#-18llx %#-18llx %#-18llx %#-18llx\n"

#define PRINT_LANES_EQINFO_FIELD_8(name, field, buf, buf_offset, buf_size) \
	print_diag(buf, buf_offset, buf_size, EQINFO_8BIT_HDR_FMT, name, \
		   (unsigned char)eq_info[0].field, (unsigned char)eq_info[1].field, \
		   (unsigned char)eq_info[2].field, (unsigned char)eq_info[3].field)

#define PRINT_LANES_EQINFO_FIELD_16(name, field, buf, buf_offset, buf_size) \
	print_diag(buf, buf_offset, buf_size, EQINFO_16BIT_HDR_FMT, name, \
		   (unsigned short)eq_info[0].field, (unsigned short)eq_info[1].field, \
		   (unsigned short)eq_info[2].field, (unsigned short)eq_info[3].field)

#define PRINT_LANES_EQINFO_FIELD_32(name, field, buf, buf_offset, buf_size) \
	print_diag(buf, buf_offset, buf_size, EQINFO_32BIT_HDR_FMT, name, \
		   (unsigned int)eq_info[0].field, (unsigned int)eq_info[1].field, \
		   (unsigned int)eq_info[2].field, (unsigned int)eq_info[3].field)

#define PRINT_LANES_EQINFO_FIELD_64(name, field, buf, buf_offset, buf_size) \
	print_diag(buf, buf_offset, buf_size, EQINFO_64BIT_HDR_FMT, name, \
		   eq_info[0].field, eq_info[1].field, \
		   eq_info[2].field, eq_info[3].field)

static void serdes_eqinfo_process(char *buf, size_t *buf_offset, size_t buf_size,
				  struct mbdb_serdes_eq_info *eq_info)
{
	char name[17];
	int i;

	PRINT_LANES_EQINFO_FIELD_32("eqP4Rev", eq_p4_rev, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_32("eqP4Time", eq_p4_time, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_32("rxdTxdP4Rev", rxd_txd_p4_rev, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_32("rxdTxdP4Time", rxd_txd_p4_time, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("eqCompileOptions", eq_compile_options, buf, buf_offset,
				   buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agcMode", agc_mode, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1LmsMu", agc1_lms_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1PeakNcycExp", agc1_peak_ncyc_exp, buf, buf_offset,
				   buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc2LmsMu", agc2_lms_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc2PeakNcycExp", agc2_peak_ncyc_exp, buf, buf_offset,
				   buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agcLpfMu", agc_lpf_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agcTarg", agc_targ, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1LmsEn", agc1_lms_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1LmsLd", agc1_lms_ld, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc2LmsEn", agc2_lms_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc2LmsLd", agc2_lms_ld, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1LmsLdVal", agc1_lms_ld_val, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1Ctl", agc1_ctl, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1Peak", agc1_peak, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc1Ppeak", agc1_ppeak, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("agc2LmsLdVal", agc2_lms_ld_val, buf, buf_offset, buf_size);

	for (i = 0; i < ARRAY_SIZE(eq_info[0].agc2_ctl); i++) {
		scnprintf(name, sizeof(name), "agc2Ctl[%d]", i);
		PRINT_LANES_EQINFO_FIELD_8(name, agc2_ctl[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].agc2_peak); i++) {
		scnprintf(name, sizeof(name), "agc2Peak[%d]", i);
		PRINT_LANES_EQINFO_FIELD_8(name, agc2_peak[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].agc2_ppeak); i++) {
		scnprintf(name, sizeof(name), "agc2Ppeak[%d]", i);
		PRINT_LANES_EQINFO_FIELD_8(name, agc2_ppeak[i], buf, buf_offset, buf_size);
	}

	PRINT_LANES_EQINFO_FIELD_8("cdrPropMu", cdr_prop_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrIntgMu", cdr_intg_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrFltMu", cdr_flt_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrPherrScale", cdr_pherr_scale, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrSsEn", cdr_ss_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrFltEn", cdr_flt_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrIntgEn", cdr_intg_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrPhase", cdr_phase, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cdrIntg", cdr_intg, buf, buf_offset, buf_size);

	PRINT_LANES_EQINFO_FIELD_16("cdrPhErrFlt", cdr_ph_err_flt, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_64("cntrIlvExclMsk", cntr_ilv_excl_msk, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_32("pphm", pphm, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("cntrSh", cntr_sh, buf, buf_offset, buf_size);

	for (i = 0; i < ARRAY_SIZE(eq_info[0].hcntr); i++) {
		scnprintf(name, sizeof(name), "hcntr[%d]", i);
		PRINT_LANES_EQINFO_FIELD_8(name, hcntr[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].cntr_ch_est); i++) {
		scnprintf(name, sizeof(name), "cntrChEst[%d]", i);
		PRINT_LANES_EQINFO_FIELD_16(name, cntr_ch_est[i], buf, buf_offset, buf_size);
	}

	PRINT_LANES_EQINFO_FIELD_8("ffeLmsMu", ffe_lms_mu, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("ffeLmsLkMuDelta", ffe_lms_lk_mu_delta, buf, buf_offset,
				   buf_size);
	PRINT_LANES_EQINFO_FIELD_8("ffeLmsLkEn", ffe_lms_lk_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("dfeLmsMu", dfe_lms_mu, buf, buf_offset, buf_size);

	for (i = 0; i < ARRAY_SIZE(eq_info[0].eq_targ); i++) {
		scnprintf(name, sizeof(name), "eqTarg[%d]", i);
		PRINT_LANES_EQINFO_FIELD_16(name, eq_targ[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].dfe_nthr); i++) {
		scnprintf(name, sizeof(name), "dfeNthr[%d]", i);
		PRINT_LANES_EQINFO_FIELD_16(name, dfe_nthr[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].dfe_zthr); i++) {
		scnprintf(name, sizeof(name), "dfeZthr[%d]", i);
		PRINT_LANES_EQINFO_FIELD_16(name, dfe_zthr[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].dfe_pthr); i++) {
		scnprintf(name, sizeof(name), "dfePthr[%d]", i);
		PRINT_LANES_EQINFO_FIELD_16(name, dfe_pthr[i], buf, buf_offset, buf_size);
	}

	for (i = 0; i < ARRAY_SIZE(eq_info[0].hffe); i++) {
		scnprintf(name, sizeof(name), "hffe[%d]", i);
		PRINT_LANES_EQINFO_FIELD_16(name, hffe[i], buf, buf_offset, buf_size);
	}

	PRINT_LANES_EQINFO_FIELD_32("gf0", gf0, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_16("hdfe", hdfe, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("nrzSliceEn", nrz_slice_en, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("rmtTxLane", rmt_tx_lane, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_16("lmsSumErr", lms_sum_err, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_16("lmsSumErrShf", lms_sum_err_shf, buf, buf_offset, buf_size);

	for (i = 0; i < ARRAY_SIZE(eq_info[0].tx_fir_eh); i++) {
		scnprintf(name, sizeof(name), "txFirEh[%d]", i);
		PRINT_LANES_EQINFO_FIELD_8(name, tx_fir_eh[i], buf, buf_offset, buf_size);
	}

	PRINT_LANES_EQINFO_FIELD_8("txFirEhM1", tx_fir_eh_m1, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_8("pllLolCnt", pll_lol_cnt, buf, buf_offset, buf_size);
	PRINT_LANES_EQINFO_FIELD_16("pmonUlvtFreq", pmon_ulvt_freq, buf, buf_offset, buf_size);
}

static int serdes_eqinfo_open(struct inode *inode, struct file *file)
{
	struct fport *port = inode->i_private;
	struct serdes_eqinfo_info {
		struct debugfs_blob_wrapper blob;
		struct mbdb_serdes_eq_info_get_rsp rsp;
		char buf[EQINFO_DISPLAY_BUF_SIZE];
	} *info;
	size_t buf_size;
	size_t buf_offset;
	char *buf;
	int ret;

	if (!port)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = ops_serdes_eqinfo_get(port->sd, port->lpn, &info->rsp);
	if (ret) {
		kfree(info);
		return ret;
	}

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;
	buf_offset = 0;

	print_diag(buf, &buf_offset, buf_size, LPN_HEADER_FMT, port->lpn);
	print_diag(buf, &buf_offset, buf_size, EQINFO_LANE_HDR_FMT);

	serdes_eqinfo_process(buf, &buf_offset, buf_size, info->rsp.eq_info);

	info->blob.data = buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations serdes_eqinfo_fops = {
	.owner = THIS_MODULE,
	.open = serdes_eqinfo_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define LCB_COUNTERS_FILE_NAME "lcb_ctrs"

#define LCB_ERR_INFO_NAMES_B0 \
	"TOTAL_CRC_ERR", \
	"CRC_ERR_LN0", \
	"CRC_ERR_LN1", \
	"CRC_ERR_LN2", \
	"CRC_ERR_LN3", \
	"CRC_ERR_MULTI_LN", \
	"TX_REPLAY", \
	"RX_REPLAY", \
	"SEQ_CRC", \
	"ESCAPE_0_ONLY", \
	"ESCAPE_0_PLUS1", \
	"ESCAPE_0_PLUS2", \
	"REINIT_FROM_PEER", \
	"SBE", \
	"MISC_FLG", \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	"FEC_CERR_1", \
	"FEC_CERR_2", \
	"FEC_CERR_3", \
	"FEC_CERR_4", \
	"FEC_CERR_5", \
	"FEC_CERR_6", \
	"FEC_CERR_7", \
	"FEC_CERR_8", \
	"FEC_UERR_CNT", \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	NULL, \
	"FEC_ERR_LN0", \
	"FEC_ERR_LN1", \
	"FEC_ERR_LN2", \
	"FEC_ERR_LN3", \
	"RX_RESYNC_CNT"

static const char * const lcb_err_info_names_b0[] = {
	LCB_ERR_INFO_NAMES_B0
};

static const char * const lcb_prf_names[] = {
	"GOOD_LTP",
	"ACCEPTED_LTP",
	"TX_RELIABLE_LTP",
	"RX_FLIT",
	"TX_FLIT",
	NULL,
	"GOOD_FECCW"
};

#define LCB_ERR_INFO_VALUES_B0 ARRAY_SIZE(lcb_err_info_names_b0)
#define LCB_ERR_INFO_VALUES_NUM LCB_ERR_INFO_VALUES_B0
#define LCB_PRF_VALUES ARRAY_SIZE(lcb_prf_names)

#define LCB_COUNTERS_DISPLAY_BUF_SIZE (PAGE_SIZE - sizeof(struct debugfs_blob_wrapper))

#define LCB_COUNTERS_FMT "%-16s %llu\n"

struct lcb_counters_regs_data {
	DECLARE_MBDB_OP_PORT_STATUS_GET_RSP(regs_op, LCB_ERR_INFO_VALUES_NUM + LCB_PRF_VALUES);
} __packed;

static int lcb_counters_open(struct inode *inode, struct file *file)
{
	struct fport *port = inode->i_private;
	struct mbdb_op_csr_range csr_ranges[] = {
		{ .offset = O_LCB_ERR_INFO_OFFSET,
		  .num_csrs = LCB_ERR_INFO_VALUES_B0
		},
		{ .offset = O_LCB_PRF_OFFSET,
		  .num_csrs = LCB_PRF_VALUES
		},
	};
	struct lcb_counters_regs_data regs = {};
	struct lcb_counters_info {
		struct debugfs_blob_wrapper blob;
		char buf[LCB_COUNTERS_DISPLAY_BUF_SIZE];
	} *info;
	const char * const *lcb_err_info_names = lcb_err_info_names_b0;
	size_t buf_size;
	size_t buf_offset;
	char *buf;
	int ret;
	int i;

	ret = ops_port_status_get(port->sd, port->lpn, ARRAY_SIZE(csr_ranges), csr_ranges,
				  &regs.regs_op);
	if (ret)
		return ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;
	buf_offset = 0;

	print_diag(buf, &buf_offset, buf_size, "%s %u\n", "LOGICAL_PORT", port->lpn);
	print_diag(buf, &buf_offset, buf_size, LCB_COUNTERS_FMT, "FR_RTC", regs.cp_free_run_rtc);

	for (i = 0; i < csr_ranges[0].num_csrs; i++)
		if (lcb_err_info_names[i])
			print_diag(buf, &buf_offset, buf_size, LCB_COUNTERS_FMT,
				   lcb_err_info_names[i], regs.regs[i]);

	for (i = 0; i < LCB_PRF_VALUES; i++)
		if (lcb_prf_names[i])
			print_diag(buf, &buf_offset, buf_size, LCB_COUNTERS_FMT, lcb_prf_names[i],
				   regs.regs[i + csr_ranges[0].num_csrs]);

	info->blob.data = buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations lcb_counters_fops = {
	.owner = THIS_MODULE,
	.open = lcb_counters_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define SERDES_CHANNEL_ESTIMATION_MAX_BUF_SIZE (PAGE_SIZE * 10)
#define SERDES_CHANNEL_ESTIMATION_MAX_ELEMENTS 1020
#define SERDES_CHANNEL_ESTIMATION_FILE_NAME "serdes_channel_estimation"
#define SERDES_CHANNEL_ESTIMATION_DATA_ELEMENT_FMT "%9hd"

static int serdes_channel_estimation_open(struct inode *inode, struct file *file)
{
	struct fport *port = inode->i_private;
	struct serdes_channel_estimation_info {
		struct debugfs_blob_wrapper blob;
		struct mbdb_serdes_ch_est_rsp rsp[LANES];
		char buf[SERDES_CHANNEL_ESTIMATION_MAX_BUF_SIZE];
	} *info;
	struct mbdb_serdes_ch_est_rsp *rsp;
	size_t buf_size;
	size_t buf_offset;
	char *buf;
	u8 lane;
	u16 elements;
	u16 data_element;
	int ret;

	if (!port)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	elements = SERDES_CHANNEL_ESTIMATION_MAX_ELEMENTS;

	for (lane = 0; lane < LANES; lane++) {
		rsp = &info->rsp[lane];
		ret = ops_serdes_channel_estimate_get(port->sd, port->lpn, lane, rsp);
		if (ret) {
			kfree(info);
			return ret;
		}

		elements = min_t(u16, elements, rsp->elements);
	}

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;
	buf_offset = 0;

	print_diag(buf, &buf_offset, buf_size, LPN_HEADER_FMT, port->lpn);
	print_diag(buf, &buf_offset, buf_size, LANE_HEADER_FMT);

	for (data_element = 0; data_element < elements; data_element++) {
		for (lane = 0; lane < LANES; lane++)
			print_diag(buf, &buf_offset, buf_size,
				   SERDES_CHANNEL_ESTIMATION_DATA_ELEMENT_FMT,
				   info->rsp[lane].data[data_element]);

		print_diag(buf, &buf_offset, buf_size, "\n");
	}

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations serdes_channel_estimation_fops = {
	.owner = THIS_MODULE,
	.open = serdes_channel_estimation_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

/*
 * Remote TX lanes
 *
 * Data is transmitted on up to four lanes and may be "swizzled" so that TX lanes are connected to
 * differently-numbered RX lanes. Report the source lane for all four lanes.
 */

#define REMOTE_TX_LANES_FILE_NAME "remote_tx_lanes"
#define TX_LANES_STRING_SIZE (10)

/* lane number as a character, replacing illegal lane values with x (lane is unconnected) */
static char lane_indicator(u8 lane)
{
	return lane < LANES ? '0' + lane : 'x';
}

static ssize_t remote_tx_lanes_read(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	struct fport *port = fp->private_data;
	char rd_buf[TX_LANES_STRING_SIZE];
	size_t siz;
	u64 value = 0;
	int err;

	err = ops_linkmgr_port_csr_rd(port->sd, port->lpn, O_LCB_STS_RX_LOGICAL_ID, sizeof(value),
				      &value);
	if (err)
		return err;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%c %c %c %c\n",
			lane_indicator(FIELD_GET(PEER_TX_ID_LN0, value)),
			lane_indicator(FIELD_GET(PEER_TX_ID_LN1, value)),
			lane_indicator(FIELD_GET(PEER_TX_ID_LN2, value)),
			lane_indicator(FIELD_GET(PEER_TX_ID_LN3, value)));

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static const struct file_operations remote_lanes_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = remote_tx_lanes_read
};

/*
 * Port enables
 *
 * "enable" identifies whether the port is used at all; "usage_enable" identifies whether routing
 * will configure it to carry data
 */

#define PORT_ENABLE_FILE_NAME "enable"
#define USAGE_ENABLE_FILE_NAME "usage_enable"
#define ENABLE_STRING_SIZE 3

/* port control as a character indicating boolean state */
static char control_flag(struct fport *port, enum PORT_CONTROL bit)
{
	return test_bit(bit, port->controls) ? 'Y' : 'N';
}

static ssize_t port_ena_read(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	struct fport *port = fp->private_data;
	char rd_buf[ENABLE_STRING_SIZE];
	size_t siz;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%c\n", control_flag(port, PORT_CONTROL_ENABLED));

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t usage_ena_read(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	struct fport *port = fp->private_data;
	char rd_buf[ENABLE_STRING_SIZE];
	size_t siz;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%c\n", control_flag(port, PORT_CONTROL_ROUTABLE));

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t func_ena_write(struct file *fp, const char __user *buf, size_t count, loff_t *fpos,
			      int (*enablefn)(struct fport *), int (*disablefn)(struct fport *))
{
	struct fport *port = fp->private_data;
	char *kbuf;
	bool set;
	int err;

	if (!count)
		return 0;

	kbuf = kzalloc(count + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	/* only proceed if entire string can be copied */
	if (copy_from_user(kbuf, buf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	err = strtobool(kbuf, &set);
	kfree(kbuf);
	if (err)
		return err;

	err = set ? enablefn(port) : disablefn(port);
	if (err)
		return err;

	*fpos += count;
	return count;
}

static ssize_t port_ena_write(struct file *fp, const char __user *buf, size_t count, loff_t *fpos)
{
	return func_ena_write(fp, buf, count, fpos, enable_port, disable_port);
}

static ssize_t usage_ena_write(struct file *fp, const char __user *buf, size_t count, loff_t *fpos)
{
	return func_ena_write(fp, buf, count, fpos, enable_usage_port, disable_usage_port);
}

static const struct file_operations port_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = port_ena_read,
	.write = port_ena_write
};

static const struct file_operations usage_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = usage_ena_read,
	.write = usage_ena_write
};

static void enable_nodes_init(struct fsubdev *sd, struct fport *port)
{
	debugfs_create_file(PORT_ENABLE_FILE_NAME, 0600, sd->debugfs_port_dir, port,
			    &port_enable_fops);
	debugfs_create_file(USAGE_ENABLE_FILE_NAME, 0600, sd->debugfs_port_dir, port,
			    &usage_enable_fops);
}

/*
 * Per-port TX tuning parameters
 *
 * There are two sets of TX tuning parameters for each port, based on speed class: FAST (>= 90G) and
 * SLOW (<= 53G). Each set contains one value per lane. A third set of parameters allows users to
 * query or set parameters corresponding to the currently-configured speed.
 */
#define TX_TUNING_FAST_FILE_NAME "tx_tuning_fast"
#define TX_TUNING_SLOW_FILE_NAME "tx_tuning_slow"
#define TX_TUNING_CURR_FILE_NAME "tx_tuning_current"

#define BAD_TUNE_IDX (~0)
#define UNSPECIFIED_TX_TUNING_INDICES { BAD_TUNE_IDX, BAD_TUNE_IDX, BAD_TUNE_IDX, BAD_TUNE_IDX }
#define LONGEST_TX_TUNING_STRING (48)
#define TUNING_SEPS " \t"

#define LEGAL_TX_TUNING_INDEX(_i) ((_i) < 256)

static int read_tx_tunings(struct fport *port, u32 link_speed, u32 idx[LANES])
{
	struct port_var_data var_data = {};
	int err;
	int i;

	err = ops_port_var_table_read(port->sd, port->lpn, link_speed, &var_data);
	if (err)
		return err;

	for (i = 0; i < LANES; ++i)
		idx[i] = var_data.tx_tuning[i];

	return 0;
}

static int write_tx_tunings(struct fport *port, u32 link_speed, const u32 idx[LANES])
{
	struct port_var_data var_data = {};
	int err;
	int i;

	for (i = 0; i < LANES; ++i)
		if (!LEGAL_TX_TUNING_INDEX(idx[i]))
			break;

	err = i < LANES ? ops_port_var_table_read(port->sd, port->lpn, link_speed, &var_data) : 0;
	if (err)
		return err;

	for (i = 0; i < LANES; ++i)
		if (LEGAL_TX_TUNING_INDEX(idx[i]))
			var_data.tx_tuning[i] = idx[i];

	return ops_port_var_table_write(port->sd, port->lpn, link_speed, &var_data, false);
}

static ssize_t tune_read_spd(struct file *fp, char __user *buf, size_t count, loff_t *fpos,
			     u32 link_speed)
{
	char rd_buf[LONGEST_TX_TUNING_STRING];
	u32 idx[LANES];
	size_t siz;
	int err;

	if (!link_speed) {
		siz = scnprintf(rd_buf, sizeof(rd_buf), "? ? ? ?\n");
		return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
	}

	err = read_tx_tunings(fp->private_data, link_speed, idx);
	if (err)
		return err;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%u %u %u %u\n", idx[0], idx[1], idx[2], idx[3]);
	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t tune_write_spd(struct file *fp, const char __user *buf, size_t count, loff_t *fpos,
			      u32 link_speed)
{
	u32 idx[LANES] = UNSPECIFIED_TX_TUNING_INDICES;
	char *next_token;
	char *curr_token;
	char *kbuf;
	int lane;
	int err;

	if (!count)
		return 0;

	/* do not try to process unreasonably long input */
	if (count > LONGEST_TX_TUNING_STRING)
		return -EINVAL;

	kbuf = kzalloc(count + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	if (copy_from_user(kbuf, buf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	/*
	 * process only fully-copied input, consisting of 4 unsigned numbers separated by
	 * single spaces/tabs: any illegal number (e.g., "-", "none") causes the existing
	 * corresponding value to be retained
	 */

	next_token = kbuf;

	for (lane = 0; next_token && *next_token && lane < LANES; ++lane) {
		curr_token = strsep(&next_token, TUNING_SEPS);
		err = kstrtou32(curr_token, 0, &idx[lane]);
		if (err)
			idx[lane] = BAD_TUNE_IDX;
	}

	kfree(kbuf);

	err = write_tx_tunings(fp->private_data, link_speed, idx);
	if (err < 0)
		return err;

	*fpos += count;

	return count;
}

static ssize_t tune_read_fast(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	return tune_read_spd(fp, buf, count, fpos, LINK_SPEED_FAST);
}

static ssize_t tune_read_slow(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	return tune_read_spd(fp, buf, count, fpos, LINK_SPEED_SLOW);
}

static ssize_t tune_read_current(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	struct fport *port = fp->private_data;
	u32 link_speed;

	/*
	 * read tuning parameters for active speed class, which must uniquely match either FAST or
	 * SLOW bitmap
	 *
	 * if active speed does not match this criterion (i.e., if no speed is active), fall back to
	 * enabled speed(s) if it/they uniquely match either FAST or SLOW bitmap
	 */

	link_speed = port->portinfo->link_speed_active;

	if ((bool)(link_speed & LINK_SPEED_SLOW) == (bool)(link_speed & LINK_SPEED_FAST)) {
		link_speed = port->portinfo->link_speed_enabled;

		if ((bool)(link_speed & LINK_SPEED_SLOW) == (bool)(link_speed & LINK_SPEED_FAST))
			link_speed = 0;
	}

	return tune_read_spd(fp, buf, count, fpos, link_speed);
}

static ssize_t tune_write_fast(struct file *fp, const char __user *buf, size_t count, loff_t *fpos)
{
	return tune_write_spd(fp, buf, count, fpos, LINK_SPEED_FAST);
}

static ssize_t tune_write_slow(struct file *fp, const char __user *buf, size_t count, loff_t *fpos)
{
	return tune_write_spd(fp, buf, count, fpos, LINK_SPEED_SLOW);
}

static ssize_t tune_write_current(struct file *fp, const char __user *buf, size_t count,
				  loff_t *fpos)
{
	struct fport *port = fp->private_data;
	u32 link_speed;

	/*
	 * write tuning parameters for active speed class which must match either FAST or SLOW
	 * bitmap
	 *
	 * if active speed does not match this criterion (i.e., if no speed is active), instead
	 * enabled speed(s)
	 */

	link_speed = port->portinfo->link_speed_active;

	if (!(link_speed & (LINK_SPEED_SLOW | LINK_SPEED_FAST)))
		link_speed = port->portinfo->link_speed_enabled;

	return tune_write_spd(fp, buf, count, fpos, link_speed);
}

static const struct file_operations tune_fast_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = tune_read_fast,
	.write = tune_write_fast
};

static const struct file_operations tune_slow_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = tune_read_slow,
	.write = tune_write_slow
};

static const struct file_operations tune_current_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = tune_read_current,
	.write = tune_write_current
};

static void tx_tuning_nodes_init(struct fport *port, struct dentry *debugfs_dir)
{
	debugfs_create_file(TX_TUNING_FAST_FILE_NAME, 0600, debugfs_dir, port, &tune_fast_fops);
	debugfs_create_file(TX_TUNING_SLOW_FILE_NAME, 0600, debugfs_dir, port, &tune_slow_fops);
	debugfs_create_file(TX_TUNING_CURR_FILE_NAME, 0600, debugfs_dir, port, &tune_current_fops);
}

/*
 * Serdes Margin query and associated subops
 */

#define TX_DCC_DIR_NAME "tx_dcc"
#define TX_DCC_MARGIN_PARAMS_FILE_NAME "margin_params"
#define TX_DCC_INTERP_FILE_NAME "interp"
#define TX_DCC_INDEX_FILE_NAME "index"
#define MAINT_MODE_FILE_NAME "maint_mode"
#define TX_DCC_INTERP_OVVL_FILE_NAME "interp_override"
#define TX_DCC_INTERP_OV_ENABLE_FILE_NAME "interp_override_enable"

#define LANE_SUFFIX 6
/* Max tx_dcc file name + _lanex + 1 */
#define MAX_TX_DCC_FILE_NAME_BUF_SIZE (22 + LANE_SUFFIX + 1)

#define MARGIN_PARAMS_BUF_SIZE 7
#define DCC_INTERP_BUF_SIZE 165
#define MAINT_MODE_BUF_SIZE 12
#define DCC_INTERP_OVERRIDE_BUF_SIZE 12
#define DCC_INTERP_OVERRIDE_ENABLE_BUF_SIZE 3

static ssize_t tx_dcc_margin_params_read(struct file *fp, char __user *buf, size_t count,
					 loff_t *fpos)
{
	struct fport *port = fp->private_data;
	struct tx_dcc_margin_param_get_rsp rsp = {};
	char rd_buf[MARGIN_PARAMS_BUF_SIZE];
	size_t siz;

	if (!port)
		return -EINVAL;

	if (ops_tx_dcc_margin_param_get(port->sd, port->lpn, &rsp))
		return -EINVAL;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%u\n", rsp.value);

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t tx_dcc_margin_params_write(struct file *fp, const char __user *buf, size_t count,
					  loff_t *fpos)
{
	struct fport *port = fp->private_data;
	u16 value;

	if (!port)
		return -EINVAL;

	if (!count)
		return 0;

	if (kstrtou16_from_user(buf, count, 0, &value))
		return -EINVAL;

	if (ops_tx_dcc_margin_param_set(port->sd, port->lpn, value, true))
		return -EINVAL;

	*fpos += count;
	return count;
}

static int tx_dcc_interp_open(struct inode *inode, struct file *file)
{
	struct fport **lane = inode->i_private;
	struct fport *port = lane ? *lane : NULL;
	struct dcc_interp_info {
		struct debugfs_blob_wrapper blob;
		struct tx_dcc_interp_get_rsp rsp;
		char buf[DCC_INTERP_BUF_SIZE];
	} *info;
	struct tx_dcc_interp_get_rsp *rsp;
	size_t buf_size;
	size_t buf_offset;
	char *buf;

	if (!port)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	rsp = &info->rsp;

	if (ops_tx_dcc_interp_get(port->sd, port->lpn, lane_number(lane), rsp)) {
		kfree(info);
		return -EINVAL;
	}

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;
	buf_offset = 0;

	print_diag(buf, &buf_offset, buf_size, "%-18s : %u\n", "dccp", rsp->dccp);
	print_diag(buf, &buf_offset, buf_size, "%-18s : %u\n", "dccnb", rsp->dccnb);
	print_diag(buf, &buf_offset, buf_size, "%-18s : %u\n", "tx_interp2_ctl",
		   rsp->tx_interp2_ctl);
	print_diag(buf, &buf_offset, buf_size, "%-18s : %u\n", "tx_mux_delay_ctrl2",
		   rsp->tx_mux_delay_ctrl2);
	print_diag(buf, &buf_offset, buf_size, "%-18s : %u\n", "reverse_pd", rsp->reverse_pd);

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static ssize_t tx_dcc_index_write(struct file *fp, const char __user *buf, size_t count,
				  loff_t *fpos)
{
	struct fport **lane = fp->private_data;
	struct fport *port = lane ? *lane : NULL;
	u32 index;

	if (!port)
		return -EINVAL;

	if (!count)
		return 0;

	if (kstrtou32_from_user(buf, count, 0, &index))
		return -EINVAL;

	if (ops_tx_dcc_index_set(port->sd, port->lpn, lane_number(lane), index, false))
		return -EINVAL;

	*fpos += count;
	return count;
}

static ssize_t maint_mode_read(struct file *fp, char __user *buf, size_t count, loff_t *fpos)
{
	struct fport *port = fp->private_data;
	struct maint_mode_get_rsp rsp = {};
	char rd_buf[MAINT_MODE_BUF_SIZE];
	size_t siz;

	if (!port)
		return -EINVAL;

	if (ops_maint_mode_get(port->sd, port->lpn, &rsp))
		return -EINVAL;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%u\n", rsp.mode);

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t maint_mode_write(struct file *fp, const char __user *buf, size_t count, loff_t *fpos)
{
	struct fport *port = fp->private_data;
	u32 mode;

	if (!port)
		return -EINVAL;

	if (!count)
		return 0;

	if (kstrtou32_from_user(buf, count, 0, &mode))
		return -EINVAL;

	if (ops_maint_mode_set(port->sd, port->lpn, mode, false))
		return -EINVAL;

	*fpos += count;
	return count;
}

static ssize_t tx_dcc_interp_override_read(struct file *fp, char __user *buf, size_t count,
					   loff_t *fpos)
{
	struct fport **lane = fp->private_data;
	struct fport *port = lane ? *lane : NULL;
	struct tx_dcc_interp_override_get_rsp rsp = {};
	char rd_buf[DCC_INTERP_OVERRIDE_BUF_SIZE];
	size_t siz;

	if (!port)
		return -EINVAL;

	if (ops_tx_dcc_interp_override_get(port->sd, port->lpn, lane_number(lane), &rsp))
		return -EINVAL;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%u\n", rsp.value);

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t tx_dcc_interp_override_write(struct file *fp, const char __user *buf, size_t count,
					    loff_t *fpos)
{
	struct fport **lane = fp->private_data;
	struct fport *port = lane ? *lane : NULL;
	u32 value;

	if (!port)
		return -EINVAL;

	if (!count)
		return 0;

	if (kstrtou32_from_user(buf, count, 0, &value))
		return -EINVAL;

	if (ops_tx_dcc_interp_override_set(port->sd, port->lpn, lane_number(lane), value, true))
		return -EINVAL;

	*fpos += count;
	return count;
}

static ssize_t tx_dcc_interp_override_enable_read(struct file *fp, char __user *buf, size_t count,
						  loff_t *fpos)
{
	struct fport **lane = fp->private_data;
	struct fport *port = lane ? *lane : NULL;
	struct tx_dcc_interp_override_enable_get_rsp rsp = {};
	char rd_buf[DCC_INTERP_OVERRIDE_ENABLE_BUF_SIZE];
	size_t siz;

	if (!port)
		return -EINVAL;

	if (ops_tx_dcc_interp_override_enable_get(port->sd, port->lpn, lane_number(lane), &rsp))
		return -EINVAL;

	siz = scnprintf(rd_buf, sizeof(rd_buf), "%s\n", rsp.enable ? "Y" : "N");

	return simple_read_from_buffer(buf, count, fpos, rd_buf, siz);
}

static ssize_t tx_dcc_interp_override_enable_write(struct file *fp, const char __user *buf,
						   size_t count, loff_t *fpos)
{
	struct fport **lane = fp->private_data;
	struct fport *port = lane ? *lane : NULL;
	bool enable;

	if (!port)
		return -EINVAL;

	if (!count)
		return 0;

	if (kstrtobool_from_user(buf, count, &enable))
		return -EINVAL;

	if (ops_tx_dcc_interp_override_enable_set(port->sd, port->lpn, lane_number(lane),
						  enable ? 1 : 0, true))
		return -EINVAL;

	*fpos += count;
	return count;
}

static const struct file_operations tx_dcc_margin_params_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = tx_dcc_margin_params_read,
	.write = tx_dcc_margin_params_write
};

static const struct file_operations tx_dcc_interp_fops = {
	.owner = THIS_MODULE,
	.open = tx_dcc_interp_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

static const struct file_operations tx_dcc_index_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.write = tx_dcc_index_write
};

static const struct file_operations maint_mode_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = maint_mode_read,
	.write = maint_mode_write
};

static const struct file_operations tx_dcc_interp_override_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = tx_dcc_interp_override_read,
	.write = tx_dcc_interp_override_write
};

static const struct file_operations tx_dcc_interp_override_enable_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = tx_dcc_interp_override_enable_read,
	.write = tx_dcc_interp_override_enable_write
};

static void tx_dcc_margin_nodes_init(struct fport *port, struct dentry *debugfs_port_dir)
{
	struct dentry *tx_dcc_dir = debugfs_create_dir(TX_DCC_DIR_NAME, debugfs_port_dir);
	u8 lane;

	debugfs_create_file(MAINT_MODE_FILE_NAME, 0600, debugfs_port_dir, port, &maint_mode_fops);

	debugfs_create_file(TX_DCC_MARGIN_PARAMS_FILE_NAME, 0600, tx_dcc_dir, port,
			    &tx_dcc_margin_params_fops);

	for (lane = 0; lane < LANES; lane++) {
		char buf[MAX_TX_DCC_FILE_NAME_BUF_SIZE];

		scnprintf(buf, sizeof(buf), TX_DCC_INTERP_FILE_NAME "_lane%u", lane);
		debugfs_create_file(buf, 0400, tx_dcc_dir, &port->lanes_port[lane],
				    &tx_dcc_interp_fops);

		scnprintf(buf, sizeof(buf), TX_DCC_INDEX_FILE_NAME "_lane%u", lane);
		debugfs_create_file(buf, 0200, tx_dcc_dir, &port->lanes_port[lane],
				    &tx_dcc_index_fops);

		scnprintf(buf, sizeof(buf), TX_DCC_INTERP_OVVL_FILE_NAME "_lane%u", lane);
		debugfs_create_file(buf, 0600, tx_dcc_dir, &port->lanes_port[lane],
				    &tx_dcc_interp_override_fops);

		scnprintf(buf, sizeof(buf), TX_DCC_INTERP_OV_ENABLE_FILE_NAME "_lane%u", lane);
		debugfs_create_file(buf, 0600, tx_dcc_dir, &port->lanes_port[lane],
				    &tx_dcc_interp_override_enable_fops);
	}
}

/**
 * create_fabric_port_debugfs_files() - Add diagnostic nodes to a fabric port debugfs hierarchy
 * @sd: sub device port directory (debugfs_port_dir) in debugfs to populate under
 * @port: fabric port to reference
 * Create debugfs nodes to query (and in the case of tx tuning parameters, set) SERDES-related
 * information and LCB counters. They are removed recursively, so no matching remove function is
 * needed.
 */
void create_fabric_port_debugfs_files(struct fsubdev *sd, struct fport *port)
{
	debugfs_create_file(PORT_SHOW_FILE_NAME, 0400, sd->debugfs_port_dir, port,
			    &port_show_fops);

	debugfs_create_file(LCB_COUNTERS_FILE_NAME, 0400, sd->debugfs_port_dir, port,
			    &lcb_counters_fops);

	if (test_bit(MBOX_OP_CODE_SERDES_HISTOGRAM_GET, port->sd->fw_version.supported_opcodes))
		debugfs_create_file(SERDES_HISTOGRAM_FILE_NAME, 0400, sd->debugfs_port_dir, port,
				    &serdes_histogram_fops);

	if (test_bit(MBOX_OP_CODE_SERDES_EQINFO_GET, port->sd->fw_version.supported_opcodes))
		debugfs_create_file(SERDES_EQINFO_FILE_NAME, 0400, sd->debugfs_port_dir, port,
				    &serdes_eqinfo_fops);

	if (test_bit(MBOX_OP_CODE_SERDES_CHEST_GET, port->sd->fw_version.supported_opcodes))
		debugfs_create_file(SERDES_CHANNEL_ESTIMATION_FILE_NAME, 0400, sd->debugfs_port_dir,
				    port, &serdes_channel_estimation_fops);

	debugfs_create_file(REMOTE_TX_LANES_FILE_NAME, 0400, sd->debugfs_port_dir, port,
			    &remote_lanes_fops);

	enable_nodes_init(sd, port);

	if (test_bit(MBOX_OP_CODE_VARIABLE_TABLE_READ, port->sd->fw_version.supported_opcodes))
		tx_tuning_nodes_init(port, sd->debugfs_port_dir);

	if (test_bit(MBOX_OP_CODE_SERDES_TX_DCC_MARGIN, port->sd->fw_version.supported_opcodes))
		tx_dcc_margin_nodes_init(port, sd->debugfs_port_dir);
}

/*
 * bridge port register debugfs functions
 */

#define BRG_FILE_NAME "stats_brg"
#define RTP_FILE_NAME "stats_rtp"
#define TPM_FILE_NAME "stats_tpm"
#define RPM_FILE_NAME "stats_rpm"

#define STATS_BUF_SIZE (PAGE_SIZE * 3)

#define BRG_PERF_COUNT (((BRG_PERF_END - BRG_PERF_OFFSET) / sizeof(u64)) + 1)

#define BRG_ERR_COUNT (((BRG_ERR_END - BRG_ERR_OFFSET) / sizeof(u64)) + 1)

#define BRG_1_ERR_COUNT (((BRG_1_ERR_END - BRG_1_ERR_OFFSET) / sizeof(u64)) + 1)
#define BRG_2_ERR_COUNT (((BRG_2_ERR_END - BRG_2_ERR_OFFSET) / sizeof(u64)) + 1)
#define BRG_3_ERR_COUNT (((BRG_3_ERR_END - BRG_3_ERR_OFFSET) / sizeof(u64)) + 1)

#define BRG_RTP_ERR_COUNT (((BRG_RTP_ERR_END - BRG_RTP_ERR_OFFSET) / sizeof(u64)) + 1)
#define BRG_RTP_STS_COUNT_1 (((BRG_RTP_STS_END_1 - BRG_RTP_STS_START_1) / sizeof(u64)) + 1)
#define BRG_RTP_STS_COUNT_2 (((BRG_RTP_STS_END_2 - BRG_RTP_STS_START_2) / sizeof(u64)) + 1)

#define TPM_ERR_COUNT (((TPM_ERR_END - TPM_ERR_START) / sizeof(u64)) + 1)
#define TPM_ERR_MBE_COUNT (((TPM_ERR_MBE_END - TPM_ERR_MBE_START) / sizeof(u64)) + 1)
#define TPM_PRF_COUNT (((TPM_PRF_END - TPM_PRF_START) / sizeof(u64)) + 1)

#define RPM_INQ_PORT0_ERR_COUNT \
	(((RPM_INQ_PORT0_ERR_END - RPM_INQ_PORT0_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT1_ERR_COUNT \
	(((RPM_INQ_PORT1_ERR_END - RPM_INQ_PORT1_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT2_ERR_COUNT \
	(((RPM_INQ_PORT2_ERR_END - RPM_INQ_PORT2_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT3_ERR_COUNT \
	(((RPM_INQ_PORT3_ERR_END - RPM_INQ_PORT3_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT4_ERR_COUNT \
	(((RPM_INQ_PORT4_ERR_END - RPM_INQ_PORT4_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT5_ERR_COUNT \
	(((RPM_INQ_PORT5_ERR_END - RPM_INQ_PORT5_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT6_ERR_COUNT \
	(((RPM_INQ_PORT6_ERR_END - RPM_INQ_PORT6_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_PORT7_ERR_COUNT \
	(((RPM_INQ_PORT7_ERR_END - RPM_INQ_PORT7_ERR_START) / sizeof(u64)) + 1)
#define RPM_INQ_FLSTOR_ERR_COUNT \
	(((RPM_INQ_FLSTOR_ERR_END - RPM_INQ_FLSTOR_ERR_START) / sizeof(u64)) + 1)

#define RPM_PORT_STS_COUNT (((RPM_PORT_STS_END - RPM_PORT_STS_START) / sizeof(u64)) + 1)

#define RPM_SBE_ERR_COUNTERS_COUNT \
	(((RPM_SBE_ERR_COUNTERS_END - RPM_SBE_ERR_COUNTERS_START) / sizeof(u64)) + 1)
#define RPM_MBE_ERR_COUNTERS_COUNT \
	(((RPM_MBE_ERR_COUNTERS_END - RPM_MBE_ERR_COUNTERS_START) / sizeof(u64)) + 1)

#define RPM_ARB_PERF_COUNTERS_COUNT \
	(((RPM_ARB_PERF_COUNTERS_END - RPM_ARB_PERF_COUNTERS_START) / sizeof(u64)) + 1)

#define RPM_PORT_ERR_COUNTERS_COUNT \
	(((RPM_PORT_ERR_COUNTERS_END - RPM_PORT_ERR_COUNTERS_START) / sizeof(u64)) + 1)

#define RPM_PERF_COUNTERS_COUNT \
	(((RPM_PERF_COUNTERS_END - RPM_PERF_COUNTERS_START) / sizeof(u64)) + 1)

static const char * const time_names[] = {
	"NO_TIME",
	"BRG_TIME",
	"RTP_TIME",
	"TPM_TIME",
	"RPM_PORT_ERRS_TIME",
	"RPM_TIME",
};

enum time_type {
	NO_TIME,
	BRG_TIME,
	RTP_TIME,
	TPM_TIME,
	RPM_PORT_ERRS_TIME,
	RPM_TIME,
};

struct regs_data {
	DECLARE_MBDB_OP_PORT_STATUS_GET_RSP(regs_op, MAX_CSRS);
} __packed;

static struct mbdb_op_csr_range brg_csr_ranges[] = {
	{ .offset = BRG_PERF_OFFSET, .num_csrs = BRG_PERF_COUNT, },
	{ .offset = BRG_ERR_OFFSET, .num_csrs = BRG_ERR_COUNT, },
	{ .offset = BRG_1_ERR_OFFSET, .num_csrs = BRG_1_ERR_COUNT, },
	{ .offset = BRG_2_ERR_OFFSET, .num_csrs = BRG_2_ERR_COUNT, },
	{ .offset = BRG_3_ERR_OFFSET, .num_csrs = BRG_3_ERR_COUNT, },
};

static struct mbdb_op_csr_range rtp_csr_ranges[] = {
	{ .offset = BRG_RTP_ERR_OFFSET, .num_csrs = BRG_RTP_ERR_COUNT, },
	{ .offset = BRG_RTP_STS_START_1, .num_csrs = BRG_RTP_STS_COUNT_1, },
	{ .offset = BRG_RTP_STS_START_2, .num_csrs = BRG_RTP_STS_COUNT_2, },
};

static struct mbdb_op_csr_range tpm_csr_ranges[] = {
	{ .offset = TPM_ERR_START, .num_csrs = TPM_ERR_COUNT, },
	{ .offset = TPM_ERR_MBE_START, .num_csrs = TPM_ERR_MBE_COUNT, },
	{ .offset = TPM_PRF_START, .num_csrs = TPM_PRF_COUNT, },
};

static struct mbdb_op_csr_range rpm_port_errs_csr_ranges[] = {
	{ .offset = RPM_PORT_ERR_COUNTERS_START, .num_csrs = RPM_PORT_ERR_COUNTERS_COUNT, },
};

static struct mbdb_op_csr_range rpm_csr_ranges[] = {
	{ .offset = RPM_INQ_PORT0_ERR_START, .num_csrs = RPM_INQ_PORT0_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT1_ERR_START, .num_csrs = RPM_INQ_PORT1_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT2_ERR_START, .num_csrs = RPM_INQ_PORT2_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT3_ERR_START, .num_csrs = RPM_INQ_PORT3_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT4_ERR_START, .num_csrs = RPM_INQ_PORT4_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT5_ERR_START, .num_csrs = RPM_INQ_PORT5_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT6_ERR_START, .num_csrs = RPM_INQ_PORT6_ERR_COUNT, },
	{ .offset = RPM_INQ_PORT7_ERR_START, .num_csrs = RPM_INQ_PORT7_ERR_COUNT, },
	{ .offset = RPM_INQ_FLSTOR_ERR_START, .num_csrs = RPM_INQ_FLSTOR_ERR_COUNT, },
	{ .offset = RPM_SBE_ERR_COUNTERS_START, .num_csrs = RPM_SBE_ERR_COUNTERS_COUNT, },
	{ .offset = RPM_MBE_ERR_COUNTERS_START, .num_csrs = RPM_MBE_ERR_COUNTERS_COUNT, },
	{ .offset = RPM_ARB_PERF_COUNTERS_START, .num_csrs = RPM_ARB_PERF_COUNTERS_COUNT, },
	{ .offset = RPM_PORT_STS_START, .num_csrs = RPM_PORT_STS_COUNT, },
	{ .offset = RPM_PERF_COUNTERS_START, .num_csrs = RPM_PERF_COUNTERS_COUNT, },
};

static void regs_print_ranges(char *buf, size_t *buf_offset, struct mbdb_op_csr_range *csr_ranges,
			      size_t csr_ranges_elements, struct regs_data *regs)
{
	size_t elem;
	size_t i = 0;

	for (elem = 0; elem < csr_ranges_elements; elem++) {
		u32 addr = csr_ranges[elem].offset;
		u32 end_addr = addr + (csr_ranges[elem].num_csrs * sizeof(u64));

		while (addr < end_addr) {
			print_diag(buf, buf_offset, STATS_BUF_SIZE, "0x%08x 0x%016llx\n",
				   addr, regs->regs_op.regs[i++]);
			addr += sizeof(u64);
		}
	}
}

static size_t regs_print(char *buf, size_t buf_offset, enum time_type time_name,
			 size_t csr_ranges_sz, struct mbdb_op_csr_range *csr_ranges,
			 struct regs_data *regs)
{
	print_diag(buf, &buf_offset, STATS_BUF_SIZE, "%-8s 0x%016llx\n", time_names[time_name],
		   regs->regs_op.cp_free_run_rtc);

	regs_print_ranges(buf, &buf_offset, csr_ranges, csr_ranges_sz, regs);

	return buf_offset;
}

static struct regs_data *regs_query_data(struct fsubdev *sd, u8 lpn, size_t csr_ranges_sz,
					 struct mbdb_op_csr_range *csr_ranges)
{
	struct regs_data *regs;

	regs = kzalloc(sizeof(*regs), GFP_KERNEL);
	if (!regs)
		return NULL;

	if (ops_port_status_get(sd, lpn, csr_ranges_sz, csr_ranges, &regs->regs_op)) {
		kfree(regs);
		return NULL;
	}

	return regs;
}

static int regs_open(struct inode *inode, struct file *file)
{
	struct fport *port = inode->i_private;
	struct regs_info {
		struct debugfs_blob_wrapper blob;
		char buf[STATS_BUF_SIZE];
	} *info;
	struct mbdb_op_csr_range *csr_ranges[] = { NULL, NULL, NULL, };
	enum time_type time_name[] = { NO_TIME, NO_TIME, NO_TIME, };
	struct regs_data *regs[] = { NULL, NULL, NULL, };
	struct dentry *dentry = F_DENTRY(file);
	size_t csr_range_sz[] = { 0, 0, 0, };
	int ret = 0;
	int i;

	if (!port)
		return -EINVAL;

	if (!strcmp(dentry->d_iname, BRG_FILE_NAME)) {
		csr_ranges[0] = brg_csr_ranges;
		csr_range_sz[0] = ARRAY_SIZE(brg_csr_ranges);
		time_name[0] = BRG_TIME;
	}

	if (!strcmp(dentry->d_iname, RTP_FILE_NAME)) {
		csr_ranges[0] = rtp_csr_ranges;
		csr_range_sz[0] = ARRAY_SIZE(rtp_csr_ranges);
		time_name[0] = RTP_TIME;
	}

	if (!strcmp(dentry->d_iname, TPM_FILE_NAME)) {
		csr_ranges[0] = tpm_csr_ranges;
		csr_range_sz[0] = ARRAY_SIZE(tpm_csr_ranges);
		time_name[0] = TPM_TIME;
	}

	if (!strcmp(dentry->d_iname, RPM_FILE_NAME)) {
		csr_ranges[0] = rpm_port_errs_csr_ranges;
		csr_range_sz[0] = ARRAY_SIZE(rpm_port_errs_csr_ranges);
		time_name[0] = RPM_PORT_ERRS_TIME;

		csr_ranges[1] = rpm_csr_ranges;
		csr_range_sz[1] = ARRAY_SIZE(rpm_csr_ranges);
		time_name[1] = RPM_TIME;
	}

	for (i = 0; csr_ranges[i]; i++) {
		regs[i] = regs_query_data(port->sd, port->lpn, csr_range_sz[i], csr_ranges[i]);
		if (!regs[i]) {
			ret = -EIO;
			goto exit;
		}
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto exit;
	}

	info->blob.data = &info->buf;

	print_diag(info->buf, &info->blob.size, STATS_BUF_SIZE, "%-8s 0x%08x\n", "FABRIC_ID",
		   port->sd->fdev->fabric_id);
	print_diag(info->buf, &info->blob.size, STATS_BUF_SIZE, "%-8s %-u\n", "TILE_NUMBER",
		   sd_index(port->sd));
	print_diag(info->buf, &info->blob.size, STATS_BUF_SIZE, "%-3s %-u\n", "LPN", port->lpn);

	for (i = 0; regs[i]; i++)
		info->blob.size = regs_print(info->buf, info->blob.size, time_name[i],
					     csr_range_sz[i], csr_ranges[i], regs[i]);

	file->private_data = info;

exit:
	for (i = 0; regs[i]; i++)
		kfree(regs[i]);

	return ret;
}

static const struct file_operations regs_fops = {
	.owner = THIS_MODULE,
	.open = regs_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

/**
 * create_bridge_port_debugfs_files() - Add diagnostic nodes to a bridge port debugfs hierarchy
 * @sd: sub device port directory (debugfs_port_dir) in debugfs to populate under
 * @port: bridge port to reference
 *
 * Create debugfs nodes to query They are removed recursively, so no matching remove function is
 * needed.
 */
void create_bridge_port_debugfs_files(struct fsubdev *sd, struct fport *port)
{
	debugfs_create_file(BRG_FILE_NAME, 0400, sd->debugfs_port_dir, port, &regs_fops);
	debugfs_create_file(RTP_FILE_NAME, 0400, sd->debugfs_port_dir, port, &regs_fops);
	debugfs_create_file(TPM_FILE_NAME, 0400, sd->debugfs_port_dir, port, &regs_fops);
	debugfs_create_file(RPM_FILE_NAME, 0400, sd->debugfs_port_dir, port, &regs_fops);
}

void create_port_debugfs_dir(struct fsubdev *sd, u8 lpn)
{
	char debugfs_dir_name[9] = {};

	snprintf(debugfs_dir_name, ARRAY_SIZE(debugfs_dir_name), "port.%u", lpn);
	sd->debugfs_port_dir = debugfs_create_dir(debugfs_dir_name, sd->debugfs_dir);
}
