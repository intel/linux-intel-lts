/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 *
 */

#ifndef OPS_H_INCLUDED
#define OPS_H_INCLUDED

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/types.h>

#include "iaf_drv.h"
#include "mbdb.h"

/* MBOX opcodes */
/* Firmware Load and Generic CSR Access opcodes */
#define MBOX_OP_CODE_FW_VERSION 0
#define MBOX_OP_CODE_CSR_RAW_RD 1
#define MBOX_OP_CODE_CSR_RAW_WR 2
#define MBOX_OP_CODE_FW_START 3

/* Configuration Required opcodes */
#define MBOX_OP_CODE_WALLOC 4
#define MBOX_OP_CODE_INI_TABLE_LOAD 6
#define MBOX_OP_CODE_INI_LOADED_SET 7
#define MBOX_OP_CODE_LINK_MGR_PORT_LINK_STATE_SET 9
#define MBOX_OP_CODE_LINK_MGR_PORT_MAJOR_PHYSICAL_STATE_SET 10
#define MBOX_OP_CODE_LINK_MGR_PORT_CSR_RD 11
#define MBOX_OP_CODE_LINK_MGR_PORT_CSR_WR 12

/* System Management Related opcodes */
#define MBOX_OP_CODE_RESET 13
#define MBOX_OP_CODE_QSFP_MGR_FAULT_TRAP_NOTIFICATION 22
#define MBOX_OP_CODE_QSFP_MGR_FAULT_TRAP_ACKNOWLEDGE 23
#define MBOX_OP_CODE_LINK_MGR_PORT_STATES_GET 29
#define MBOX_OP_CODE_LINK_MGR_PORT_BEACON_SET 31
#define MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_ENABLE_SET 33
#define MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_NOTIFICATION 34
#define MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_ACKNOWLEDGE 35

/* Fabric Discovery, Configuration and basic Monitoring  Related Opcodes */

#define MBOX_OP_CODE_SWITCHINFO_GET 53
#define MBOX_OP_CODE_SWITCHINFO_SET 54
#define MBOX_OP_CODE_PORTINFO_GET 55
#define MBOX_OP_CODE_RPIPE_GET 59
#define MBOX_OP_CODE_RPIPE_SET 60

/* trap related mbox messages relating to link health */

#define MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_ENABLE_SET 64
#define MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_NOTIFICATION 65
#define MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_ACKNOWLEDGE 66
#define MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_ENABLE_SET 68
#define MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_NOTIFICATION 69
#define MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_ACKNOWLEDGE 70
#define MBOX_OP_CODE_PORT_STATUS_GET 80
#define MBOX_OP_CODE_STATE_DUMP 81
#define MBOX_OP_CODE_SERDES_HISTOGRAM_GET 82
#define MBOX_OP_CODE_SERDES_EQINFO_GET 83
#define MBOX_OP_CODE_SERDES_CHEST_GET 84

#define MBOX_OP_CODE_VARIABLE_TABLE_READ 88
#define MBOX_OP_CODE_VARIABLE_TABLE_WRITE 89

/*  Firmware and System Related Debugging opcodes */

#define MBOX_OP_CODE_LINK_MGR_TRACE_DUMP 85
#define MBOX_OP_CODE_LINK_MGR_TRACE_MASK_GET 86
#define MBOX_OP_CODE_LINK_MGR_TRACE_MASK_SET 87

#define MBOX_OP_CODE_SERDES_TX_DCC_MARGIN 92

// Sub-Ops for SerDes Tx Dcc Margin Op
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_MARGIN_PARAM_GET 0
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_MARGIN_PARAM_SET 1
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_GET 2
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INDEX_SET 3
#define SERDES_MARGIN_SUB_OP_CODE_MAINT_MODE_GET 4
#define SERDES_MARGIN_SUB_OP_CODE_MAINT_MODE_SET 5
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OVVL_GET 6
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OVVL_SET 7
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OV_EN_GET 8
#define SERDES_MARGIN_SUB_OP_CODE_TX_DCC_INTERP_OV_EN_SET 9

#define MBOX_OP_CODE_FAULT_TRAP 254

#define MBOX_RSP_STATUS_OK            0
#define MBOX_RSP_STATUS_SEQ_NO_ERROR  1
#define MBOX_RSP_STATUS_OP_CODE_ERROR 2
#define MBOX_RSP_STATUS_LOGICAL_ERROR 3
#define MBOX_RSP_STATUS_RETRY         4
#define MBOX_RSP_STATUS_DENIED        5 // access was denied

// MBOX declarations
#define MBDB_GP_DATA_U64_LOCATIONS 512
#define MBOX_SIZE_IN_BYTES (MBDB_GP_DATA_U64_LOCATIONS * sizeof(u64) / 2)
#define MBOX_SIZE_IN_QWORDS (MBDB_GP_DATA_U64_LOCATIONS / 2)
#define MBOX_CW_FIELD_TYPE u64
#define MBOX_CW_SIZE_IN_BYTES sizeof(MBOX_CW_FIELD_TYPE)
#define MBOX_CW_SIZE_IN_QWORDS 1
#define MBOX_PARAM_AREA_IN_BYTES (MBOX_SIZE_IN_BYTES - MBOX_CW_SIZE_IN_BYTES)
#define MBOX_PARAM_AREA_IN_QWORDS (MBOX_SIZE_IN_QWORDS - MBOX_CW_SIZE_IN_QWORDS)
#define MBOX_READ_DATA_SIZE_IN_BYTES MBOX_PARAM_AREA_IN_BYTES
/* The first 8 bytes (u64) is the address where the data is to be written */
#define MBOX_WRITE_DATA_SIZE_IN_BYTES (MBOX_PARAM_AREA_IN_BYTES - sizeof(u64))

#define OP_CODE_POSTED         BIT_ULL(0)
#define OP_CODE_CHECK_IS_VALID BIT_ULL(1)

#define NON_POSTED          (0)
#define NON_POSTED_CHECK_OP (OP_CODE_CHECK_IS_VALID)
#define POSTED(p)           (p ? OP_CODE_POSTED : NON_POSTED)
#define POSTED_CHECK_OP(p)  (OP_CODE_CHECK_IS_VALID | POSTED(p))

struct mbox_msg {
	u64 cw;
	u64 param_area[MBOX_PARAM_AREA_IN_QWORDS];
} __packed;

struct mbdb_serdes_ch_est_req {
	u32 port;
	u32 lane;
	u32 ilv;
	u32 range_min;
	u32 range_max;
} __packed;

struct mbdb_serdes_ch_est_rsp {
	u16 elements;
	s16 data[MBOX_PARAM_AREA_IN_BYTES / sizeof(s16)];
} __packed;

#define LANE_DATA_ELEMENTS 128

struct mbdb_serdes_eq_info {
	u32 eq_p4_rev;
	u32 eq_p4_time;
	u32 rxd_txd_p4_rev;
	u32 rxd_txd_p4_time;
	u8 eq_compile_options;
	u8 agc_mode;
	u8 agc1_lms_mu;
	u8 agc1_peak_ncyc_exp;
	u8 agc2_lms_mu;
	u8 agc2_peak_ncyc_exp;
	u8 agc_lpf_mu;
	u8 agc_targ;
	u8 agc1_lms_en;
	u8 agc1_lms_ld;
	u8 agc2_lms_en;
	u8 agc2_lms_ld;
	u8 agc1_lms_ld_val;
	u8 agc1_ctl;
	u8 agc1_peak;
	u8 agc1_ppeak;
	u8 agc2_lms_ld_val;
	u8 agc2_ctl[4];
	u8 agc2_peak[4];
	u8 agc2_ppeak[4];
	u8 cdr_prop_mu;
	u8 cdr_intg_mu;
	u8 cdr_flt_mu;
	u8 cdr_pherr_scale;
	u8 cdr_ss_en;
	u8 cdr_flt_en;
	u8 cdr_intg_en;
	u8 cdr_phase;
	s8 cdr_intg;
	s16 cdr_ph_err_flt;
	u64 cntr_ilv_excl_msk;
	s32 pphm;
	u8 cntr_sh;
	u8 hcntr[5];
	s16 cntr_ch_est[5];
	u8 ffe_lms_mu;
	u8 ffe_lms_lk_mu_delta;
	u8 ffe_lms_lk_en;
	u8 dfe_lms_mu;
	s16 eq_targ[4];
	s16 dfe_nthr[2];
	s16 dfe_zthr[2];
	s16 dfe_pthr[2];
	s16 hffe[30];
	s16 reserved1[30];
	s32 gf0;
	s16 hdfe;
	u8 nrz_slice_en;
	u8 rmt_tx_lane;
	u16 lms_sum_err;
	u16 lms_sum_err_shf;
	u8 tx_fir_eh[4];
	u8 tx_fir_eh_m1;
	u8 pll_lol_cnt;
	u16 pmon_ulvt_freq;
} __packed;

struct mbdb_serdes_eq_info_get_rsp {
	struct mbdb_serdes_eq_info eq_info[LANES];
} __packed;

struct mbdb_serdes_histogram_data {
	u16 data[LANE_DATA_ELEMENTS];
} __packed;

struct mbdb_serdes_histogram_rsp {
	struct mbdb_serdes_histogram_data lane[LANES];
} __packed;

struct mbdb_statedump_rsp {
	u64 descs[MBOX_PARAM_AREA_IN_QWORDS];
	u16 desc_count;
} __packed;

struct mbdb_op_ini_table_load_req {
	u32 header1;
	u32 header2;
	u32 address;
	u32 crc;
} __packed;

struct mbdb_op_rpipe_get {
	u32 port_number;
	u16 start_index;
	u16 num_entries;
	u8 rpipe_data[];
} __packed;

struct mbdb_op_rpipe_set {
	u32 port_mask;
	u16 start_index;
	u16 num_entries;
	u8 rpipe_data[];
} __packed;

struct port_var_data {
	u8 tx_tuning[LANES];
} __packed;

struct tx_dcc_margin_error_rsp {
	u32 sub_op;
} __packed;

struct tx_dcc_margin_param_get_rsp {
	u32 sub_op;
	u16 value;
} __packed;

struct tx_dcc_interp_get_rsp {
	u32 sub_op;
	u32 dccp;
	u32 dccnb;
	u32 tx_interp2_ctl;
	u32 tx_mux_delay_ctrl2;
	u32 reverse_pd;
} __packed;

struct maint_mode_get_rsp {
	u32 sub_op;
	u32 mode;
} __packed;

struct tx_dcc_interp_override_get_rsp {
	u32 sub_op;
	u32 value;
} __packed;

struct tx_dcc_interp_override_enable_get_rsp {
	u32 sub_op;
	u32 enable;
} __packed;

#define QSFP_PAGE_SIZE (CINFO_PAGE_SIZE)

#define MAX_TRACE_ENTRIES ((MBOX_READ_DATA_SIZE_IN_BYTES - sizeof(u64)) / sizeof(u64))

struct mbdb_op_linkmgr_trace_dump_rsp {
	u32 cnt;
	u32 more;
	u64 entries[MAX_TRACE_ENTRIES];
} __packed;

#define MAX_CSRS 254

/**
 * struct mbdb_op_csr_range - range of registers to read
 * @offset: starting register offset
 * @num_csrs: number of registers to read
 */
struct mbdb_op_csr_range {
	u32 offset;
	u32 num_csrs;
} __packed;

struct mbdb_op_port_status_get_rsp {
	u64 cp_free_run_rtc;
	u64 regs[];
} __packed;

/*
 * Use this to declare an instance of mbdb_op_port_status_get_rsp sized to a given number
 * of registers
 */
#define DECLARE_MBDB_OP_PORT_STATUS_GET_RSP(_p, _n) \
	union { \
		struct mbdb_op_port_status_get_rsp _p; \
		struct { \
			u64 cp_free_run_rtc; \
			u64 regs[_n]; \
		} __packed; \
	}

struct mbdb_op_param {
	size_t len;
	const void *data;
};

struct mbdb_op_enable_param {
	u32 enabled;
};

void ops_rsp_handler_relaxed(struct mbdb_ibox *ibox);
void ops_rsp_handler_default(struct mbdb_ibox *ibox);

void ops_element_copy(void __iomem *src, void *dst, size_t src_sz, size_t dst_sz, size_t elements);

void ops_portinfo_rsp_handler(struct mbdb_ibox *ibox);

static inline void mbdb_op_enable_param_set(struct mbdb_op_enable_param *param,
					    bool enabled)
{
	/* encapsulate enable information for op data structures. */
	param->enabled = enabled ? 1 : 0;
}

struct mbdb_ibox *
mbdb_op_build_cw_and_acquire_ibox_set_hdlr(struct fsubdev *sd, const u8 op_code, size_t parm_len,
					   void *response, u32 rsp_len, u64 *cw,
					   unsigned long flags, op_response_handler op_rsp_handler)
	__attribute__((nonnull(1, 6, 8)));

struct mbdb_ibox *
mbdb_op_build_cw_and_acquire_ibox(struct fsubdev *sd, const u8 op_code, size_t parm_len,
				  void *response, u32 rsp_len, u64 *cw, unsigned long flags)
	__attribute__((nonnull(1, 6)));

int ops_tx_dcc_interp_override_enable_set(struct fsubdev *sd, u32 port, u32 lane, u32 enable,
					  bool posted)
	__attribute__((nonnull(1)));

int ops_tx_dcc_interp_override_enable_get(struct fsubdev *sd, u32 port, u32 lane,
					  struct tx_dcc_interp_override_enable_get_rsp *rsp)
	__attribute__((nonnull(1, 4)));

int ops_tx_dcc_interp_override_set(struct fsubdev *sd, u32 port, u32 lane, u32 value, bool posted)
	__attribute__((nonnull(1)));

int ops_tx_dcc_interp_override_get(struct fsubdev *sd, u32 port, u32 lane,
				   struct tx_dcc_interp_override_get_rsp *rsp)
	__attribute__((nonnull(1, 4)));

int ops_maint_mode_set(struct fsubdev *sd, u32 port, u32 mode, bool posted)
	__attribute__((nonnull(1)));

int ops_maint_mode_get(struct fsubdev *sd, u32 port, struct maint_mode_get_rsp *rsp)
	__attribute__((nonnull(1, 3)));

int ops_tx_dcc_index_set(struct fsubdev *sd, u32 port, u32 lane, u32 index, bool posted)
	__attribute__((nonnull(1)));

int ops_tx_dcc_interp_get(struct fsubdev *sd, u32 port, u32 lane, struct tx_dcc_interp_get_rsp *rsp)
	__attribute__((nonnull(1, 4)));

int ops_tx_dcc_margin_param_set(struct fsubdev *sd, u32 port, u16 value, bool posted)
	__attribute__((nonnull(1)));

int ops_tx_dcc_margin_param_get(struct fsubdev *sd, u32 port,
				struct tx_dcc_margin_param_get_rsp *rsp)
	__attribute__((nonnull(1, 3)));

int ops_serdes_channel_estimate_get(struct fsubdev *sd, u32 port, u32 lane,
				    struct mbdb_serdes_ch_est_rsp *rsp)
	__attribute__((nonnull(1, 4)));

int ops_port_var_table_write(struct fsubdev *sd, u32 port, u32 link_speed,
			     struct port_var_data *data, bool posted)
	__attribute__((nonnull(1, 4)));

int ops_port_var_table_read(struct fsubdev *sd, u32 port, u32 link_speed, struct port_var_data *rsp)
	__attribute__((nonnull(1, 4)));

int ops_serdes_eqinfo_get(struct fsubdev *sd, u32 port, struct mbdb_serdes_eq_info_get_rsp *rsp)
		__attribute__((nonnull(1, 3)));

int ops_serdes_histogram_get(struct fsubdev *sd, u32 port, struct mbdb_serdes_histogram_rsp *rsp)
	__attribute__((nonnull(1, 3)));

int ops_statedump(struct fsubdev *sd, u32 offset, struct mbdb_statedump_rsp *rsp)
	__attribute__((nonnull(1, 3)));

int ops_port_status_get(struct fsubdev *sd, u32 port, u8 num_csr_ranges,
			const struct mbdb_op_csr_range *csr_ranges,
			struct mbdb_op_port_status_get_rsp *rsp)
	__attribute__((nonnull(1, 4, 5)));

int ops_linkmgr_port_lqi_trap_ack(struct fsubdev *sd, bool posted)
	__attribute__((nonnull(1)));

int ops_linkmgr_port_lqi_trap_ena_set(struct fsubdev *sd, bool enabled,
				      bool posted)
	__attribute__((nonnull(1)));

int ops_linkmgr_port_lwd_trap_ack(struct fsubdev *sd, bool posted)
	__attribute__((nonnull(1)));

int ops_linkmgr_port_lwd_trap_ena_set(struct fsubdev *sd, bool enabled,
				      bool posted)
	__attribute__((nonnull(1)));

int ops_rpipe_get(struct fsubdev *sd, struct mbdb_op_rpipe_get *req,
		  struct mbdb_op_rpipe_get *rsp)
	__attribute__((nonnull(1, 2, 3)));

int ops_rpipe_set(struct fsubdev *sd, struct mbdb_op_rpipe_set *req,
		  struct mbdb_op_rpipe_set *rsp, bool posted)
	__attribute__((nonnull(1, 2)));

int ops_portinfo_get(struct fsubdev *sd, u32 port_mask,
		     struct mbdb_op_portinfo *rsp)
	__attribute__((nonnull(1, 3)));

int ops_switchinfo_set(struct fsubdev *sd, struct mbdb_op_switchinfo *req,
		       struct mbdb_op_switchinfo *rsp, bool posted)
	__attribute__((nonnull(1, 2)));

int ops_switchinfo_get(struct fsubdev *sd, struct mbdb_op_switchinfo *rsp)
	__attribute__((nonnull(1, 2)));

int ops_linkmgr_psc_trap_ack(struct fsubdev *sd, bool posted)
	__attribute__((nonnull(1)));

int ops_linkmgr_psc_trap_ena_set(struct fsubdev *sd, bool enabled, bool posted)
	__attribute__((nonnull(1)));

int ops_linkmgr_port_beacon_set(struct fsubdev *sd, u32 port, bool enabled)
	__attribute__((nonnull(1)));

int ops_linkmgr_ps_get(struct fsubdev *sd, u32 port, u32 *result)
	__attribute__((nonnull(1, 3)));

int ops_qsfpmgr_fault_trap_ack(struct fsubdev *sd, bool posted)
	__attribute__((nonnull(1)));

int ops_reset(struct fsubdev *sd, bool posted)
	__attribute__((nonnull(1)));

int ops_linkmgr_port_csr_wr(struct fsubdev *sd, u32 lpn, u32 offset,
			    const void *data, u16 len, bool posted)
	__attribute__((nonnull(1, 4)));

int ops_linkmgr_port_csr_rd(struct fsubdev *sd, u32 lpn, u32 offset, u16 len,
			    u64 *result)
	__attribute__((nonnull(1, 5)));

int ops_linkmgr_port_maj_phystate_set(struct fsubdev *sd, u32 port,
				      u32 new_state, u32 *result)
	__attribute__((nonnull(1, 4)));

int ops_linkmgr_port_link_state_set(struct fsubdev *sd, u32 port, u32 new_state,
				    u32 *result)
	__attribute__((nonnull(1, 4)));

int ops_ini_loaded_set(struct fsubdev *sd, bool posted)
	__attribute__((nonnull(1)));

int ops_ini_table_load(struct fsubdev *sd,
		       struct mbdb_op_ini_table_load_req *info, u32 *result)
	__attribute__((nonnull(1, 2, 3)));

int ops_walloc(struct fsubdev *sd, u16 dwords, u32 *addr)
	__attribute__((nonnull(1, 3)));

int ops_fw_start(struct fsubdev *sd)
	__attribute__((nonnull(1)));

int ops_csr_raw_write(struct fsubdev *sd, u32 addr, const void *data, u32 len,
		      bool posted)
	__attribute__((nonnull(1, 3)));

int ops_csr_raw_read_nolock(struct fsubdev *sd, u32 addr, size_t read_len, void *data)
	__attribute__((nonnull(1, 4)));

int ops_csr_raw_read(struct fsubdev *sd, u32 addr, size_t read_len, void *data)
	__attribute__((nonnull(1, 4)));

int ops_fw_version(struct fsubdev *sd,
		   struct mbdb_op_fw_version_rsp *fw_version)
	__attribute__((nonnull(1, 2)));

int ops_linkmgr_trace_dump(struct fsubdev *sd, u32 cnt, bool begin,
			   struct mbdb_op_linkmgr_trace_dump_rsp *rsp)
	__attribute__((nonnull(1, 4)));

int ops_linkmgr_trace_mask_get(struct fsubdev *sd, u64 *mask)
	__attribute__((nonnull(1, 2)));

int ops_linkmgr_trace_mask_set(struct fsubdev *sd, u64 mask)
	__attribute__((nonnull(1)));

int ops_mem_posted_wr(struct fsubdev *sd, u32 addr, const u8 *data, u32 len)
	__attribute__((nonnull(1)));

int ops_execute_nolock(struct fsubdev *sd, u64 *cw, u8 op_param_count,
		       struct mbdb_op_param *op_param, struct mbdb_ibox *ibox)
	__attribute__((nonnull(1, 2)));

int ops_execute(struct fsubdev *sd, u64 *cw, u8 op_param_count,
		struct mbdb_op_param *op_param, struct mbdb_ibox *ibox)
	__attribute__((nonnull(1, 2)));

#endif
