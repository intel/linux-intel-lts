// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2023 Intel Corporation.
 *
 */

#include "error.h"
#include "ops.h"

/* sequences multi-line log output between devices */
static DEFINE_MUTEX(report_lock);

/*
 * "error info" registers are generally asociated with a corresponding error
 * status register, offering more detail.  these are undefined when the status
 * register does not indicate a status that enables the info register.
 *
 * use this flag-set to track whether the triggering condition for each
 * info register has been met or not, and consume this during reporting
 * to omit these unless they've been armed by their preconditions.
 */
struct register_armed {
	u8 fpc_err_info_portrcv:1;
	u8 fpc_err_info_fmconfig:1;
	u8 fpc_err_info_flow_ctrl:1;
	u8 fpc_err_info_uncorrectable:1;
	u8 fpc_err_info_portrcvconstraint:1;
	u8 rtp_err_first_info:1;
	u8 tp_err_error_info:1;
	u8 tp_err_pkey_error_info:1;
	u8 brg_ctx_err_first_info:1;
	u8 brg_fidmiss_info:1;
	u8 brg_srcctxt_dup_rspinfo:1;
	u8 brg_dstctxt_dup_rspinfo:1;
	u8 brg_0_err_first_info:1;
	u8 brg_1_err_first_info:1;
	u8 brg_2_err_first_info:1;
	u8 tpm_err_rinfo_sbe_info:1;
	u8 tpm_err_rinfo_mbe_info:1;
	u8 tpm_err_qflitdata_sbe_info:1;
	u8 tpm_err_qflitdata_mbe_info:1;
	u8 tpm_err_rinfo_pe_info:1;
	u8 tpm_err_rsvd_vl_info:1;
	u8 tpm_err_inq_sbe_err_info:1;
	u8 tpm_err_inq_mbe_err_info:1;
	u8 brg_rtp_err_first_info:1;
};

/*
 * define a error register in the query set, including metadata about
 * how to clear this register.
 */
struct error_reg {
	const char *name;
	u32 offset;
	u32 clear_offset;
	u64 clear_value;
};

/* these INQ status bits can NOT be cleared while INQ is not in reset */
#define MASK_INQ_ERR_STS_PKT_NO_TAIL         BIT_ULL(35)
#define MASK_INQ_ERR_STS_PRE_READER_OVERFLOW BIT_ULL(28)
#define MASK_INQ_ERR_STS_USED_POINTER_RF_MBE BIT_ULL(26)
#define MASK_INQ_ERR_STS_FREE_POINTER_RF_MBE BIT_ULL(24)
#define MASK_INQ_ERR_STS_FREE_PTR_NOT_AVAIL  BIT_ULL(16)

#define MASK_INQ_ERR_CLR_EXEMPT (\
	MASK_INQ_ERR_STS_PKT_NO_TAIL         | \
	MASK_INQ_ERR_STS_PRE_READER_OVERFLOW | \
	MASK_INQ_ERR_STS_USED_POINTER_RF_MBE | \
	MASK_INQ_ERR_STS_FREE_POINTER_RF_MBE | \
	MASK_INQ_ERR_STS_FREE_PTR_NOT_AVAIL)

/*
 * define a register with "write 1 to clear" semantics, which requires an
 * associated clear register (which may or may not be the register itself)
 * and a write mask.
 */
#define MK_REG_W1C(name, clr_name, clr_mask) {#name, (name), (clr_name), (clr_mask)}

/*
 * define a register with "write 0 to clear" semantics, which simply
 * directly writes 0 to the register itself.
 */
#define MK_REG_W0C(name) {#name, (name), (name), (0ull)}

static const struct mbdb_op_csr_range s_fabric_query_ranges[] = {
	{O_FPC_ERR_STS,                   1},
	{O_FPC_ERR_FIRST_HOST,           10},
	{O_RTP_ERR_STS,                   1},
	{O_RTP_ERR_FIRST_HOST,            2},
	{O_INQ_ERR_STS,                   1},
	{O_INQ_ERR_FIRST_HOST,            1},
	{O_TP_ERR_STS_0,                  1},
	{O_TP_ERR_FIRST_HOST_0,           1},
	{O_TP_ERR_STS_1,                  1},
	{O_TP_ERR_FIRST_HOST_1,           1},
	{O_TP_ERR_ERROR_INFO,             1},
	{O_TP_ERR_PKEY_ERROR_INFO,        4},
	{O_8051_ERR_STS,                  1},
	{O_8051_FIRST_HOST,               1},
	{O_LCB_ERR_STS,                   1},
	{O_LCB_ERR_FIRST_HOST,            1},
	{O_LCB_ERR_INFO_SBE_CNT,         11},
};

static const struct error_reg s_fabric_regs[] = {
	MK_REG_W1C(O_FPC_ERR_STS,
		   O_FPC_ERR_CLR,
		   GENMASK_ULL(55, 0)),
	MK_REG_W0C(O_FPC_ERR_FIRST_HOST),
	MK_REG_W0C(O_FPC_ERR_INFO_PORTRCV),
	MK_REG_W0C(O_FPC_ERR_INFO_PORTRCV_HDR0_A),
	MK_REG_W0C(O_FPC_ERR_INFO_PORTRCV_HDR0_B),
	MK_REG_W0C(O_FPC_ERR_INFO_PORTRCV_HDR1_A),
	MK_REG_W0C(O_FPC_ERR_INFO_PORTRCV_HDR1_B),
	MK_REG_W0C(O_FPC_ERR_INFO_FMCONFIG),
	MK_REG_W0C(O_FPC_ERR_INFO_FLOW_CTRL),
	MK_REG_W0C(O_FPC_ERR_INFO_UNCORRECTABLE),
	MK_REG_W0C(O_FPC_ERR_INFO_PORTRCVCONSTRAINT),
	MK_REG_W1C(O_RTP_ERR_STS,
		   O_RTP_ERR_CLR,
		   GENMASK_ULL(31, 0)),
	MK_REG_W0C(O_RTP_ERR_FIRST_HOST),
	MK_REG_W0C(O_RTP_ERR_FIRST_INFO),
	MK_REG_W1C(O_INQ_ERR_STS,
		   O_INQ_ERR_CLR,
		   GENMASK_ULL(52, 0) & ~MASK_INQ_ERR_CLR_EXEMPT),
	MK_REG_W0C(O_INQ_ERR_FIRST_HOST),
	MK_REG_W1C(O_TP_ERR_STS_0,
		   O_TP_ERR_CLR_0,
		   GENMASK_ULL(23, 0)),
	MK_REG_W0C(O_TP_ERR_FIRST_HOST_0),
	MK_REG_W1C(O_TP_ERR_STS_1,
		   O_TP_ERR_CLR_1,
		   GENMASK_ULL(44, 0)),
	MK_REG_W0C(O_TP_ERR_FIRST_HOST_1),
	MK_REG_W0C(O_TP_ERR_ERROR_INFO),
	MK_REG_W0C(O_TP_ERR_PKEY_ERROR_INFO),
	MK_REG_W0C(O_TP_ERR_SBE_ERROR_CNT),
	MK_REG_W0C(O_TP_ERR_MBE_ERROR_CNT),
	MK_REG_W0C(O_TP_PE_ERROR_CNT),
	MK_REG_W1C(O_8051_ERR_STS,
		   O_8051_ERR_CLR,
		   GENMASK_ULL(8, 0)),
	MK_REG_W0C(O_8051_FIRST_HOST),
	MK_REG_W1C(O_LCB_ERR_STS,
		   O_LCB_ERR_CLR,
		   GENMASK_ULL(32, 0)),
	MK_REG_W0C(O_LCB_ERR_FIRST_HOST),
	MK_REG_W0C(O_LCB_ERR_INFO_SBE_CNT),
	MK_REG_W0C(O_LCB_ERR_INFO_MISC_FLG_CNT),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_INPUT_BUF),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_INPUT_BUF_HGH),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_INPUT_BUF_LOW),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_REPLAY_BUF),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_REPLAY_BUF_HGH),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_REPLAY_BUF_LOW),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_PM_TIME),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_PM_TIME_HGH),
	MK_REG_W0C(O_LCB_ERR_INFO_ECC_PM_TIME_LOW),
};

struct fabric_port_status_rsp {
	DECLARE_MBDB_OP_PORT_STATUS_GET_RSP(rsp, ARRAY_SIZE(s_fabric_regs));
};

static const struct mbdb_op_csr_range s_bridge_query_ranges[] = {
	{O_BRG_CTX_ERR_STS,                  1},
	{O_BRG_CTX_ERR_FIRST_HOST,           2},
	{O_SRC_CTXT_SBE_CNT,                24},
	{O_BRG_0_ERR_STS,                    1},
	{O_BRG_0_ERR_FIRST_HOST,            16},
	{O_BRG_1_ERR_STS,                    1},
	{O_BRG_1_ERR_FIRST_HOST,             2},
	{O_BRG_2_ERR_STS,                    1},
	{O_BRG_2_ERR_FIRST_HOST,             2},
	{O_TPM_ERR_STS,                      1},
	{O_TPM_ERR_FIRST_HOST,              14},
	{O_TPM_ERR_STORG_SBE_ERR_CNT_0,     16},
	{O_TPM_ERR_STORG_MBE_ERR_CNT_0,     16},
	{O_TPM_ERR_INQ_SBE_ERR_INFO,         2},
	{O_RPM_INQ_PORT0_ERR_STS,            1},
	{O_RPM_INQ_PORT0_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT1_ERR_STS,            1},
	{O_RPM_INQ_PORT1_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT2_ERR_STS,            1},
	{O_RPM_INQ_PORT2_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT3_ERR_STS,            1},
	{O_RPM_INQ_PORT3_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT4_ERR_STS,            1},
	{O_RPM_INQ_PORT4_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT5_ERR_STS,            1},
	{O_RPM_INQ_PORT5_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT6_ERR_STS,            1},
	{O_RPM_INQ_PORT6_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_PORT7_ERR_STS,            1},
	{O_RPM_INQ_PORT7_ERR_FIRST_HOST,     1},
	{O_RPM_INQ_FLSTOR_ERR_STS,           1},
	{O_RPM_INQ_FLSTOR_ERR_FIRST_HOST,    1},
	{O_BRG_RTP_ERR_STS,                  1},
	{O_BRG_RTP_ERR_FIRST_HOST,           2}
};

static const struct error_reg s_bridge_regs[] = {
	MK_REG_W1C(O_BRG_CTX_ERR_STS,
		   O_BRG_CTX_ERR_CLR,
		   GENMASK_ULL(17, 0)),
	MK_REG_W0C(O_BRG_CTX_ERR_FIRST_HOST),
	MK_REG_W0C(O_BRG_CTX_ERR_FIRST_INFO),
	MK_REG_W1C(O_SRC_CTXT_SBE_CNT,
		   O_SRC_CTXT_SBE_CNT,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_DST_CTXT_SBE_CNT,
		   O_DST_CTXT_SBE_CNT,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_SRC_CTXT_MBE_CNT,
		   O_SRC_CTXT_MBE_CNT,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_DST_CTXT_MBE_CNT,
		   O_DST_CTXT_MBE_CNT,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_INCMD_PKTPAR_ERR,
		   O_BRG_INCMD_PKTPAR_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_INPKT_POISON_SET,
		   O_BRG_INPKT_POISON_SET,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_INRSP_PKTPAR_ERR,
		   O_BRG_INRSP_PKTPAR_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_INDATA_PKTPAR_ERR,
		   O_BRG_INDATA_PKTPAR_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_OUTPKT_POISON_SET,
		   O_BRG_OUTPKT_POISON_SET,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_RPM_POISON_SET,
		   O_BRG_RPM_POISON_SET,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_TPM_BRG_POISON_SET,
		   O_TPM_BRG_POISON_SET,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_DROP_FABRIC_REQ,
		   O_BRG_DROP_FABRIC_REQ,
		   GENMASK_ULL(47, 0)),
	MK_REG_W1C(O_BRG_DROP_MDFI_REQ,
		   O_BRG_DROP_MDFI_REQ,
		   GENMASK_ULL(47, 0)),
	MK_REG_W1C(O_BRG_DROP_FABRIC_RSP,
		   O_BRG_DROP_FABRIC_RSP,
		   GENMASK_ULL(47, 0)),
	MK_REG_W1C(O_BRG_DROP_MDFI_RSP,
		   O_BRG_DROP_MDFI_RSP,
		   GENMASK_ULL(47, 0)),
	MK_REG_W1C(O_BRG_SRCCTXT_TO,
		   O_BRG_SRCCTXT_TO,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_DSTCTXT_TO,
		   O_BRG_DSTCTXT_TO,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_FIDMISS,
		   O_BRG_FIDMISS,
		   GENMASK_ULL(15, 0)),
	MK_REG_W0C(O_BRG_FIDMISS_INFO),
	MK_REG_W1C(O_BRG_SRCCTXT_DUP_RSP,
		   O_BRG_SRCCTXT_DUP_RSP,
		   GENMASK_ULL(15, 0)),
	MK_REG_W0C(O_BRG_SRCCTXT_DUP_RSPINFO),
	MK_REG_W1C(O_BRG_DSTCTXT_DUP_RSP,
		   O_BRG_DSTCTXT_DUP_RSP,
		   GENMASK_ULL(15, 0)),
	MK_REG_W0C(O_BRG_DSTCTXT_DUP_RSPINFO),
	MK_REG_W1C(O_BRG_SFID_FILTER_DROP,
		   O_BRG_SFID_FILTER_DROP,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_0_ERR_STS,
		   O_BRG_0_ERR_CLR,
		   GENMASK_ULL(63, 0)),
	MK_REG_W0C(O_BRG_0_ERR_FIRST_HOST),
	MK_REG_W0C(O_BRG_0_ERR_FIRST_INFO),
	MK_REG_W1C(O_BRG_SBE_ADDR2FID_ERR,
		   O_BRG_SBE_ADDR2FID_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_SBE_RSPARB_ERR,
		   O_BRG_SBE_RSPARB_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_SBE_EGRQ_ERR,
		   O_BRG_SBE_EGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_SBE_INGRQ_ERR,
		   O_BRG_SBE_INGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_SBE_TPMEGRQ_ERR,
		   O_BRG_SBE_TPMEGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_SBE_TPMINGRQ_ERR,
		   O_BRG_SBE_TPMINGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_ADDR2FID_ERR,
		   O_BRG_MBE_ADDR2FID_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_RSPARB_ERR,
		   O_BRG_MBE_RSPARB_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_EGRQ_ERR,
		   O_BRG_MBE_EGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_INGRQ_ERR,
		   O_BRG_MBE_INGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_TPMEGRQ_ERR,
		   O_BRG_MBE_TPMEGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_TPMINGRQ_ERR,
		   O_BRG_MBE_TPMINGRQ_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_SBE_TPMDECAP_ERR,
		   O_BRG_SBE_TPMDECAP_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_MBE_TPMDECAP_ERR,
		   O_BRG_MBE_TPMDECAP_ERR,
		   GENMASK_ULL(15, 0)),
	MK_REG_W1C(O_BRG_1_ERR_STS,
		   O_BRG_1_ERR_CLR,
		   GENMASK_ULL(63, 0)),
	MK_REG_W0C(O_BRG_1_ERR_FIRST_HOST),
	MK_REG_W0C(O_BRG_1_ERR_FIRST_INFO),
	MK_REG_W1C(O_BRG_2_ERR_STS,
		   O_BRG_2_ERR_CLR,
		   GENMASK_ULL(63, 0)),
	MK_REG_W0C(O_BRG_2_ERR_FIRST_HOST),
	MK_REG_W0C(O_BRG_2_ERR_FIRST_INFO),
	MK_REG_W1C(O_TPM_ERR_STS,
		   O_TPM_ERR_CLR,
		   GENMASK_ULL(47, 0)),
	MK_REG_W0C(O_TPM_ERR_FIRST_HOST),
	MK_REG_W0C(O_TPM_ERR_RINFO_SBE_COUNT),
	MK_REG_W0C(O_TPM_ERR_RINFO_SBE_INFO),
	MK_REG_W0C(O_TPM_ERR_RINFO_MBE_COUNT),
	MK_REG_W0C(O_TPM_ERR_RINFO_MBE_INFO),
	MK_REG_W0C(O_TPM_ERR_QFLITDATA_SBE_COUNT),
	MK_REG_W0C(O_TPM_ERR_QFLITDATA_SBE_INFO),
	MK_REG_W0C(O_TPM_ERR_QFLITDATA_MBE_COUNT),
	MK_REG_W0C(O_TPM_ERR_QFLITDATA_MBE_INFO),
	MK_REG_W0C(O_TPM_ERR_RINFO_PE_COUNT),
	MK_REG_W0C(O_TPM_ERR_RINFO_PE_INFO),
	MK_REG_W0C(O_TPM_ERR_TAILLESS_PKT_CNT),
	MK_REG_W0C(O_TPM_ERR_RSVD_VL_CNT),
	MK_REG_W0C(O_TPM_ERR_RSVD_VL_INFO),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_0),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_1),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_2),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_3),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_4),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_5),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_6),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_7),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_8),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_9),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_10),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_11),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_12),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_13),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_14),
	MK_REG_W0C(O_TPM_ERR_STORG_SBE_ERR_CNT_15),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_0),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_1),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_2),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_3),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_4),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_5),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_6),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_7),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_8),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_9),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_10),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_11),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_12),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_13),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_14),
	MK_REG_W0C(O_TPM_ERR_STORG_MBE_ERR_CNT_15),
	MK_REG_W0C(O_TPM_ERR_INQ_SBE_ERR_INFO),
	MK_REG_W0C(O_TPM_ERR_INQ_MBE_ERR_INFO),
	MK_REG_W1C(O_RPM_INQ_PORT0_ERR_STS,
		   O_RPM_INQ_PORT0_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT0_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT1_ERR_STS,
		   O_RPM_INQ_PORT1_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT1_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT2_ERR_STS,
		   O_RPM_INQ_PORT2_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT2_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT3_ERR_STS,
		   O_RPM_INQ_PORT3_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT3_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT4_ERR_STS,
		   O_RPM_INQ_PORT4_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT4_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT5_ERR_STS,
		   O_RPM_INQ_PORT5_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT5_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT6_ERR_STS,
		   O_RPM_INQ_PORT6_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT6_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_PORT7_ERR_STS,
		   O_RPM_INQ_PORT7_ERR_CLR,
		   GENMASK_ULL(27, 0)),
	MK_REG_W0C(O_RPM_INQ_PORT7_ERR_FIRST_HOST),
	MK_REG_W1C(O_RPM_INQ_FLSTOR_ERR_STS,
		   O_RPM_INQ_FLSTOR_ERR_CLR,
		   GENMASK_ULL(31, 0)),
	MK_REG_W0C(O_RPM_INQ_FLSTOR_ERR_FIRST_HOST),
	MK_REG_W1C(O_BRG_RTP_ERR_STS,
		   O_BRG_RTP_ERR_CLR,
		   GENMASK_ULL(31, 0)),
	MK_REG_W0C(O_BRG_RTP_ERR_FIRST_HOST),
	MK_REG_W0C(O_BRG_RTP_ERR_FIRST_INFO),
};

struct bridge_port_status_rsp {
	DECLARE_MBDB_OP_PORT_STATUS_GET_RSP(rsp, ARRAY_SIZE(s_bridge_regs));
};

/* clear helper macros */
#undef MK_REG_W0C
#undef MK_REG_W1C

static int clear_errors(struct fport *port, const u64 *errors,
			const struct error_reg *regs, size_t len)
{
	size_t i;
	int err;

	for (i = 0; i < len; ++i) {
		const struct error_reg *entry = regs + i;

		/* only clear non-zero error registers */
		if (!errors[i])
			continue;

		err = ops_linkmgr_port_csr_wr(port->sd, port->lpn, entry->clear_offset,
					      &entry->clear_value, 8, false);
		if (err) {
			fport_err(port, "failed to clear offset 0x%08x: %d\n", entry->offset, err);
			return -EIO;
		}
	}

	return 0;
}

static int read_fabric_errors(struct fport *port, struct mbdb_op_port_status_get_rsp *rsp)
{
	int err;

	err = ops_port_status_get(port->sd, port->lpn, ARRAY_SIZE(s_fabric_query_ranges),
				  s_fabric_query_ranges, rsp);
	if (err) {
		fport_err(port, "failed to get port status: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int read_bridge_errors(struct fport *port, struct mbdb_op_port_status_get_rsp *rsp)
{
	int err;

	err = ops_port_status_get(port->sd, port->lpn, ARRAY_SIZE(s_bridge_query_ranges),
				  s_bridge_query_ranges, rsp);
	if (err) {
		fport_err(port, "failed to get port status: %d\n", err);
		return -EIO;
	}

	return 0;
}

/*
 * Implement a set of DSL macros to simplify the error specifications.
 * Closure over the port and error value for retaining clarity.
 * Local use only.  Undef them afterward.
 */

/* report a single-bit named flag */
#define REPORT_FLAG(bit, desc) \
	do { \
		if (error & BIT_ULL(bit)) \
			fport_info(port, "sticky hw error:  flag: %s\n", (desc)); \
	} while (0) \

/* report a masked subfield as an integer value */
#define REPORT_VALUE(mask, desc) \
	do { \
		u64 __tmp = FIELD_GET((mask), error); \
		\
		if (__tmp) \
			fport_info(port, "sticky hw error:  field: %s: 0x%llx (%llu)\n", \
				   (desc), __tmp, __tmp); \
	} while (0)

/* report a masked subfield as an enumeration */
#define REPORT_ENUM(mask, value, name, desc) \
	do { \
		if (FIELD_GET((mask), error) == (value)) \
			fport_info(port, "sticky hw error:  field: %s: %s\n", \
				   (name), (desc)); \
	} while (0)

static void report_fabric_error(struct fport *port, u32 csr, u64 error)
{
	switch (csr) {
	case O_FPC_ERR_STS:
	case O_FPC_ERR_FIRST_HOST:
		REPORT_FLAG(55, "rvcport_err");
		REPORT_FLAG(54, "fmconfig_err");
		REPORT_FLAG(53, "uncorrectable_err");
		REPORT_FLAG(52, "portconstraint_err");
		REPORT_FLAG(38, "ltp_buf_ovflow");
		REPORT_FLAG(37, "ltp_buf_unflow");
		REPORT_FLAG(35, "late_long_err");
		REPORT_FLAG(34, "late_short_err");
		REPORT_FLAG(31, "length_mtu_err");
		REPORT_FLAG(30, "dlid_zero_err");
		REPORT_FLAG(29, "slid_zero_err");
		REPORT_FLAG(28, "perm_nvl15_err");
		REPORT_FLAG(26, "pkt_length_err");
		REPORT_FLAG(25, "vl15_multi_err");
		REPORT_FLAG(24, "nonvl15_state_err");
		REPORT_FLAG(23, "head_dist_err");
		REPORT_FLAG(22, "tail_dist_err");
		REPORT_FLAG(21, "ctrl_dist_err");
		REPORT_FLAG(20, "mkr_dist_err");
		REPORT_FLAG(19, "link_err");
		REPORT_FLAG(18, "event_cntr_rollover_err");
		REPORT_FLAG(17, "mbe_inbound");
		REPORT_FLAG(16, "crdt_sb_parity_err");
		REPORT_FLAG(15, "unsup_pkt_type");
		REPORT_FLAG(14, "crdt_ack_err");
		REPORT_FLAG(13, "mbe_outbound");
		REPORT_FLAG(8,  "unsup_vl_err");
		REPORT_FLAG(7,  "sc_mkr_err");
		REPORT_FLAG(6,  "l2hdr_err");
		REPORT_FLAG(5,  "preemt_same_vl_err");
		REPORT_FLAG(4,  "pkey_err");
		REPORT_FLAG(3,  "headless_err");
		REPORT_FLAG(2,  "sc_err");
		REPORT_FLAG(1,  "l2_err");
		break;
	case O_FPC_ERR_INFO_PORTRCV:
		REPORT_ENUM(GENMASK_ULL(7, 0), 14, "e_code", "PreemptL2Header");
		REPORT_ENUM(GENMASK_ULL(7, 0), 13, "e_code", "BadSC (marker)");
		REPORT_ENUM(GENMASK_ULL(7, 0), 12, "e_code", "PreemptVL15");
		REPORT_ENUM(GENMASK_ULL(7, 0), 11, "e_code", "PreemptError");
		REPORT_ENUM(GENMASK_ULL(7, 0),  9, "e_code", "Headless");
		REPORT_ENUM(GENMASK_ULL(7, 0),  7, "e_code", "BadSC (unsupported)");
		REPORT_ENUM(GENMASK_ULL(7, 0),  6, "e_code", "BadL2");
		REPORT_ENUM(GENMASK_ULL(7, 0),  5, "e_code", "BadDLID");
		REPORT_ENUM(GENMASK_ULL(7, 0),  4, "e_code", "BadSLID");
		REPORT_ENUM(GENMASK_ULL(7, 0),  3, "e_code", "PktLenTooShort");
		REPORT_ENUM(GENMASK_ULL(7, 0),  2, "e_code", "PktLenTooLong");
		REPORT_ENUM(GENMASK_ULL(7, 0),  1, "e_code", "BadPktLen");
		break;
	case O_FPC_ERR_INFO_FMCONFIG:
		REPORT_VALUE(GENMASK_ULL(14, 11), "vl");
		REPORT_VALUE(GENMASK_ULL(10,  8), "distance");
		REPORT_ENUM(GENMASK_ULL(7, 0), 8, "e_code", "BadMarkerDist");
		REPORT_ENUM(GENMASK_ULL(7, 0), 7, "e_code", "ExceedMulticastLimit");
		REPORT_ENUM(GENMASK_ULL(7, 0), 6, "e_code", "BadCtrlFlit");
		REPORT_ENUM(GENMASK_ULL(7, 0), 5, "e_code", "BadPreempt");
		REPORT_ENUM(GENMASK_ULL(7, 0), 4, "e_code", "UnsupportedVL");
		REPORT_ENUM(GENMASK_ULL(7, 0), 3, "e_code", "BadCrdtAck");
		REPORT_ENUM(GENMASK_ULL(7, 0), 2, "e_code", "BadCtrlDist");
		REPORT_ENUM(GENMASK_ULL(7, 0), 1, "e_code", "BadTailDist");
		REPORT_ENUM(GENMASK_ULL(7, 0), 0, "e_code", "BadHeadDist");
		break;
	case O_FPC_ERR_INFO_UNCORRECTABLE:
		REPORT_ENUM(GENMASK_ULL(3, 0), 4, "e_code", "Internal");
		REPORT_ENUM(GENMASK_ULL(3, 0), 3, "e_code", "BadCtrl");
		REPORT_ENUM(GENMASK_ULL(3, 0), 2, "e_code", "BadTail");
		REPORT_ENUM(GENMASK_ULL(3, 0), 1, "e_code", "BadBody");
		REPORT_ENUM(GENMASK_ULL(3, 0), 0, "e_code", "BadHead");
		break;
	case O_FPC_ERR_INFO_PORTRCVCONSTRAINT:
		REPORT_VALUE(GENMASK_ULL(47, 24), "slid");
		REPORT_VALUE(GENMASK_ULL(23, 8),  "pkey");
		REPORT_ENUM(GENMASK_ULL(1, 0), 3, "e_code", "SwitchPort0PkeyViolation");
		REPORT_ENUM(GENMASK_ULL(1, 0), 2, "e_code", "SlidSecurityViolation");
		REPORT_ENUM(GENMASK_ULL(1, 0), 1, "e_code", "PkeyViolation");
		break;
	case O_RTP_ERR_STS:
	case O_RTP_ERR_FIRST_HOST:
	case O_BRG_RTP_ERR_STS:
	case O_BRG_RTP_ERR_FIRST_HOST:
		REPORT_FLAG(31,                   "cport_dos_err_lweg_redir");
		REPORT_FLAG(30,                   "csr_rd_miss");
		REPORT_FLAG(29,                   "csr_wr_miss");
		REPORT_VALUE(GENMASK_ULL(28, 27), "tcam_parity_err");
		REPORT_FLAG(26,                   "loop_pkt_err");
		REPORT_FLAG(25,                   "zero_valid_oports_err");
		REPORT_FLAG(24,                   "adaptive_cport_err");
		REPORT_FLAG(23,                   "deterministic_gt1_err");
		REPORT_FLAG(22,                   "cport_dos_err");
		REPORT_VALUE(GENMASK_ULL(21, 16), "urt_top_err");
		REPORT_FLAG(15,                   "dlid_top_err");
		REPORT_FLAG(14,                   "mlid_top_err");
		REPORT_FLAG(13,                   "op4sel_mbe");
		REPORT_FLAG(12,                   "op4sel_sbe");
		REPORT_VALUE(GENMASK_ULL(11,  6), "urt_mbe");
		REPORT_VALUE(GENMASK_ULL(5,   0), "urt_sbe");
		break;
	case O_RTP_ERR_FIRST_INFO:
	case O_BRG_RTP_ERR_FIRST_INFO:
		REPORT_VALUE(GENMASK_ULL(63, 62), "l2_pkt");
		REPORT_FLAG(61,                   "mcast");
		REPORT_FLAG(60,                   "tcam_hit");
		REPORT_VALUE(GENMASK_ULL(59, 56), "vlr");
		REPORT_VALUE(GENMASK_ULL(55, 53), "rc");
		REPORT_VALUE(GENMASK_ULL(52, 48), "sc");
		REPORT_VALUE(GENMASK_ULL(47, 24), "dlid");
		REPORT_VALUE(GENMASK_ULL(23,  0), "slid");
		break;
	case O_INQ_ERR_STS:
	case O_INQ_ERR_FIRST_HOST:
		REPORT_FLAG(52, "magic_fifo_ovrfl");
		REPORT_FLAG(51, "blks_available_ovrfl");
		REPORT_FLAG(50, "invalid_decode_vlf");
		REPORT_FLAG(48, "qflits_used_cnt_vl_ovrfl");
		REPORT_FLAG(47, "qflits_used_cnt_ovrfl");
		REPORT_FLAG(45, "mc_data_fifo_ovrfl");
		REPORT_FLAG(44, "mc_fifo_err");
		REPORT_FLAG(43, "pkt_data_flit3_mbe");
		REPORT_FLAG(42, "pkt_data_flit3_sbe");
		REPORT_FLAG(41, "pkt_data_flit2_mbe");
		REPORT_FLAG(40, "pkt_data_flit2_sbe");
		REPORT_FLAG(39, "unexp_tail_err");
		REPORT_FLAG(38, "unexp_head_err");
		REPORT_FLAG(37, "mbe_active_vl");
		REPORT_FLAG(36, "mbe_inactive_vl");
		REPORT_FLAG(35, "pkt_no_tail");
		REPORT_FLAG(32, "vlr_out_of_range");
		REPORT_FLAG(30, "perf_perr");
		REPORT_FLAG(29, "perf_rollover");
		REPORT_FLAG(28, "pre_reader_overflow");
		REPORT_FLAG(27, "init_ram_complete");
		REPORT_FLAG(26, "used_pointer_rf_mbe");
		REPORT_FLAG(25, "used_pointer_rf_sbe");
		REPORT_FLAG(24, "free_pointer_rf_mbe");
		REPORT_FLAG(23, "free_pointer_rf_sbe");
		REPORT_FLAG(22, "sideband_storage_rf_mbe");
		REPORT_FLAG(21, "sideband_storage_rf_sbe");
		REPORT_FLAG(20, "pkt_data_flit1_mbe");
		REPORT_FLAG(19, "pkt_data_flit1_sbe");
		REPORT_FLAG(18, "pkt_data_flit0_mbe");
		REPORT_FLAG(17, "pkt_data_flit0_sbe");
		REPORT_FLAG(16, "free_ptr_not_avail");
		REPORT_FLAG(15, "rb_credit_err_col_7");
		REPORT_FLAG(14, "rb_credit_err_col_6");
		REPORT_FLAG(13, "rb_credit_err_col_5");
		REPORT_FLAG(12, "rb_credit_err_col_4");
		REPORT_FLAG(11, "rb_credit_err_col_3");
		REPORT_FLAG(10, "rb_credit_err_col_2");
		REPORT_FLAG(9,  "rb_credit_err_col_1");
		REPORT_FLAG(8,  "rb_credit_err_col_0");
		REPORT_FLAG(7,  "sc_tbl_perr_on_csr_rd");
		REPORT_FLAG(6,  "sc15_err");
		REPORT_FLAG(5,  "cp_dos_drop");
		REPORT_FLAG(4,  "sc_tbl_perr");
		REPORT_FLAG(3,  "rowbus_perr");
		REPORT_FLAG(2,  "invalid_oport_err");
		REPORT_FLAG(1,  "invalid_mc_err");
		REPORT_FLAG(0,  "epoch_to_err");
		break;
	case O_TP_ERR_STS_0:
	case O_TP_ERR_FIRST_HOST_0:
		REPORT_VALUE(GENMASK_ULL(23, 14), "vlp_sc_error");
		REPORT_FLAG(13,                   "congested");
		REPORT_FLAG(12,                   "l2_error");
		REPORT_FLAG(11,                   "invalid_vlt");
		REPORT_FLAG(10,                   "vlt_error");
		REPORT_FLAG(9,                    "mtu_error");
		REPORT_FLAG(8,                    "drop_data_pkt");
		REPORT_FLAG(7,                    "headless_pkt");
		REPORT_FLAG(6,                    "tailless_pkt");
		REPORT_FLAG(5,                    "short_grh_pkt");
		REPORT_FLAG(4,                    "long_pkt");
		REPORT_FLAG(3,                    "short_pkt");
		REPORT_FLAG(2,                    "subsw_pkt_ebp");
		REPORT_FLAG(1,                    "tport_pkt_ebp");
		REPORT_FLAG(0,                    "pkey_discard");
		break;
	case O_TP_ERR_STS_1:
	case O_TP_ERR_FIRST_HOST_1:
		REPORT_FLAG(44,                   "crdt_misc_ctrl_err");
		REPORT_FLAG(43,                   "crdt_rtrn_par_err");
		REPORT_FLAG(42,                   "crdt_rtrn_illegal_vl");
		REPORT_FLAG(41,                   "perf_cntr_rollover");
		REPORT_FLAG(40,                   "perf_cntr_perr");
		REPORT_VALUE(GENMASK_ULL(39, 30), "sll_timeout");
		REPORT_VALUE(GENMASK_ULL(29, 20), "hoq_discard");
		REPORT_FLAG(19,                   "oc_credit_underflow");
		REPORT_FLAG(18,                   "credit_overflow");
		REPORT_FLAG(17,                   "csr_addr_err");
		REPORT_FLAG(16,                   "dataq_spill_overflow");
		REPORT_FLAG(15,                   "dataq_spill_underflow");
		REPORT_FLAG(14,                   "dataq_underflow");
		REPORT_FLAG(13,                   "dataq_underflow");
		REPORT_FLAG(12,                   "rinfoq_spill_overflow");
		REPORT_FLAG(11,                   "rinfoq_spill_underflow");
		REPORT_FLAG(10,                   "rinfoq_spill_mbe");
		REPORT_FLAG(9,                    "rinfoq_spill_sbe");
		REPORT_FLAG(8,                    "rinfoq_overflow");
		REPORT_FLAG(7,                    "rinfoq_underflow");
		REPORT_FLAG(6,                    "rinfoq_pe");
		REPORT_FLAG(5,                    "rinfoq_mbe");
		REPORT_FLAG(4,                    "rinfoq_sbe");
		REPORT_FLAG(3,                    "subsw_pe");
		REPORT_FLAG(2,                    "subsw_mbe");
		REPORT_FLAG(1,                    "subsw_sbe");
		REPORT_FLAG(0,                    "request_to");
		break;
	case O_TP_ERR_ERROR_INFO:
		REPORT_VALUE(GENMASK_ULL(59, 50), "cb3_error_vl");
		REPORT_FLAG(49,                   "cb3_data_error");
		REPORT_FLAG(48,                   "cb3_pkt_error");
		REPORT_VALUE(GENMASK_ULL(43, 34), "cb2_error_vl");
		REPORT_FLAG(33,                   "cb2_data_error");
		REPORT_FLAG(32,                   "cb2_pkt_error");
		REPORT_VALUE(GENMASK_ULL(27, 18), "cb1_error_vl");
		REPORT_FLAG(17,                   "cb1_data_error");
		REPORT_FLAG(16,                   "cb1_pkt_error");
		REPORT_VALUE(GENMASK_ULL(11,  2), "cb0_error_vl");
		REPORT_FLAG(1,                    "cb0_data_error");
		REPORT_FLAG(0,                    "cb0_pkt_error");
		break;
	case O_TP_ERR_PKEY_ERROR_INFO:
		REPORT_VALUE(GENMASK_ULL(35, 20), "tx_first_pkey");
		REPORT_VALUE(GENMASK_ULL(19,  0), "tx_first_slid");
		break;
	case O_TP_ERR_SBE_ERROR_CNT:
	case O_TP_ERR_MBE_ERROR_CNT:
		REPORT_VALUE(GENMASK_ULL(23, 16), "synd");
		REPORT_VALUE(GENMASK_ULL(15,  0), "count");
		break;
	case O_TP_PE_ERROR_CNT:
		REPORT_VALUE(GENMASK_ULL(15,  0), "count");
		break;
	case O_8051_ERR_STS:
	case O_8051_FIRST_HOST:
		REPORT_FLAG(8, "unmatched_secure_msg_across_bcc_lanes");
		REPORT_FLAG(7, "iram_sbe");
		REPORT_FLAG(6, "iram_mbe");
		REPORT_FLAG(5, "dram_sbe");
		REPORT_FLAG(4, "dram_mbe");
		REPORT_FLAG(3, "cram_sbe");
		REPORT_FLAG(2, "cram_mbe");
		REPORT_FLAG(1, "lost8051_heart_beat");
		REPORT_FLAG(0, "set_by_8051");
		break;
	case O_LCB_ERR_STS:
	case O_LCB_ERR_FIRST_HOST:
		REPORT_FLAG(32, "fec_err_cnt_hit_limit");
		REPORT_FLAG(31, "pm_sbe");
		REPORT_FLAG(30, "pm_mbe");
		REPORT_FLAG(29, "csr_cport_address_miss");
		REPORT_FLAG(28, "csr_chain_parity_err");
		REPORT_FLAG(27, "quarantine");
		REPORT_FLAG(26, "unexpected_master_time_flit");
		REPORT_FLAG(25, "neg_edge_link_transfer_active");
		REPORT_FLAG(24, "hold_reinit");
		REPORT_FLAG(23, "rst_for_incomplt_rnd_trip");
		REPORT_FLAG(22, "rst_for_link_timeout");
		REPORT_FLAG(21, "replay_buf_sbe");
		REPORT_FLAG(20, "replay_buf_mbe");
		REPORT_FLAG(19, "flit_input_buf_sbe");
		REPORT_FLAG(18, "flit_input_buf_mbe");
		REPORT_FLAG(17, "vl_ack_input_wrong_crc_mode");
		REPORT_FLAG(16, "vl_ack_input_parity_err");
		REPORT_FLAG(15, "vl_ack_input_buf_oflw");
		REPORT_FLAG(14, "flit_input_buf_oflw");
		REPORT_FLAG(13, "illegal_flit_encoding");
		REPORT_FLAG(12, "illegal_null_ltp");
		REPORT_FLAG(11, "unexpected_round_trip_marker");
		REPORT_FLAG(10, "unexpected_replay_marker");
		REPORT_FLAG(9,  "rclk_stopped");
		REPORT_FLAG(8,  "crc_err_cnt_hit_limit");
		REPORT_FLAG(7,  "reinit_for_ln_degrade");
		REPORT_FLAG(6,  "reinit_from_peer");
		REPORT_FLAG(5,  "seq_crc_err");
		REPORT_FLAG(4,  "rx_less_than_four_lns");
		REPORT_FLAG(3,  "tx_less_than_four_lns");
		REPORT_FLAG(2,  "lost_reinit_stall_or_tos");
		REPORT_FLAG(1,  "all_lns_failed_reinit_test");
		REPORT_FLAG(0,  "rst_for_failed_deskew");
		break;
	case O_LCB_ERR_INFO_SBE_CNT:
		REPORT_VALUE(GENMASK_ULL(39, 32), "pm_time_flits");
		REPORT_VALUE(GENMASK_ULL(31, 24), "replay_buffer_side");
		REPORT_VALUE(GENMASK_ULL(23, 16), "replay_buffer1");
		REPORT_VALUE(GENMASK_ULL(15,  8), "replay_buffer0");
		REPORT_VALUE(GENMASK_ULL(7,   0), "input_buffer");
		break;
	case O_LCB_ERR_INFO_MISC_FLG_CNT:
		REPORT_VALUE(GENMASK_ULL(31, 24), "rst_for_incomplt_rnd_trip");
		REPORT_VALUE(GENMASK_ULL(23, 16), "rst_for_link_timeout");
		REPORT_VALUE(GENMASK_ULL(15,  8), "all_lns_failed_reinit_test");
		REPORT_VALUE(GENMASK_ULL(7,   0), "rst_for_failed_deskew");
		break;
	case O_LCB_ERR_INFO_ECC_INPUT_BUF:
		REPORT_FLAG(20,                     "csr_created");
		REPORT_ENUM(GENMASK_ULL(23, 16), 0, "location", "mbe in flit_0");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1, "location", "mbe in flit_1");
		REPORT_ENUM(GENMASK_ULL(23, 16), 2, "location", "mbe in flit_2");
		REPORT_ENUM(GENMASK_ULL(23, 16), 3, "location", "mbe in flit_3");
		REPORT_ENUM(GENMASK_ULL(23, 16), 4, "location", "sbe in flit_0");
		REPORT_ENUM(GENMASK_ULL(23, 16), 5, "location", "sbe in flit_1");
		REPORT_ENUM(GENMASK_ULL(23, 16), 6, "location", "sbe in flit_2");
		REPORT_ENUM(GENMASK_ULL(23, 16), 7, "location", "sbe in flit_3");
		REPORT_VALUE(GENMASK_ULL(15, 8),    "syndrome");
		REPORT_VALUE(GENMASK_ULL(7,  0),    "chk");
		break;
	case O_LCB_ERR_INFO_ECC_REPLAY_BUF:
		REPORT_FLAG(24,                      "csr_created");
		REPORT_ENUM(GENMASK_ULL(20, 16),  0, "location", "mbe ram 0 flit_0");
		REPORT_ENUM(GENMASK_ULL(20, 16),  1, "location", "mbe ram 0 flit_1");
		REPORT_ENUM(GENMASK_ULL(20, 16),  2, "location", "mbe ram 0 flit_2");
		REPORT_ENUM(GENMASK_ULL(20, 16),  3, "location", "mbe ram 0 flit_3");
		REPORT_ENUM(GENMASK_ULL(20, 16),  4, "location", "mbe ram 1 flit_0");
		REPORT_ENUM(GENMASK_ULL(20, 16),  5, "location", "mbe ram 1 flit_1");
		REPORT_ENUM(GENMASK_ULL(20, 16),  6, "location", "mbe ram 1 flit_2");
		REPORT_ENUM(GENMASK_ULL(20, 16),  7, "location", "mbe ram 1 flit_3");
		REPORT_ENUM(GENMASK_ULL(20, 16),  8, "location", "sbe ram 0 flit_0");
		REPORT_ENUM(GENMASK_ULL(20, 16),  9, "location", "sbe ram 0 flit_1");
		REPORT_ENUM(GENMASK_ULL(20, 16), 10, "location", "sbe ram 0 flit_2");
		REPORT_ENUM(GENMASK_ULL(20, 16), 11, "location", "sbe ram 0 flit_3");
		REPORT_ENUM(GENMASK_ULL(20, 16), 12, "location", "sbe ram 1 flit_0");
		REPORT_ENUM(GENMASK_ULL(20, 16), 13, "location", "sbe ram 1 flit_1");
		REPORT_ENUM(GENMASK_ULL(20, 16), 14, "location", "sbe ram 1 flit_2");
		REPORT_ENUM(GENMASK_ULL(20, 16), 15, "location", "sbe ram 1 flit_3");
		REPORT_ENUM(GENMASK_ULL(20, 16), 16, "location", "mbe side");
		REPORT_ENUM(GENMASK_ULL(20, 16), 17, "location", "sbe side");
		REPORT_VALUE(GENMASK_ULL(15, 8),     "syndrome");
		REPORT_VALUE(GENMASK_ULL(7,  0),     "chk");
		break;
	case O_LCB_ERR_INFO_ECC_PM_TIME:
		REPORT_FLAG(24,                          "csr_created");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 0, "location", "flit 0 sbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 1, "location", "flit 0 mbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 2, "location", "flit 1 sbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 3, "location", "flit 1 mbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 4, "location", "flit 2 sbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 5, "location", "flit 2 mbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 6, "location", "flit 3 sbe");
		REPORT_ENUM(GENMASK_ULL(23, 16), 1 << 7, "location", "flit 3 mbe");
		REPORT_VALUE(GENMASK_ULL(15, 8),         "syndrome");
		REPORT_VALUE(GENMASK_ULL(7,  0),         "chk");
		break;
	}
}

static void report_bridge_error(struct fport *port, u32 csr, u64 error)
{
	switch (csr) {
	case O_BRG_CTX_ERR_STS:
	case O_BRG_CTX_ERR_FIRST_HOST:
		REPORT_FLAG(17, "dst_ctxt_timeout");
		REPORT_FLAG(16, "dst_fl_mem_mbe");
		REPORT_FLAG(15, "dst_fl_mem_sbe");
		REPORT_FLAG(14, "dst_tmr_mem_mbe");
		REPORT_FLAG(13, "dst_tmr_mem_sbe");
		REPORT_FLAG(12, "dst_ctxt_mem_mbe");
		REPORT_FLAG(11, "dst_ctxt_mem_sbe");
		REPORT_FLAG(10, "dst_fl_underflow");
		REPORT_FLAG(9,  "dst_fl_overflow");
		REPORT_FLAG(8,  "src_ctxt_timeout");
		REPORT_FLAG(7,  "src_fl_mem_mbe");
		REPORT_FLAG(6,  "src_fl_mem_sbe");
		REPORT_FLAG(5,  "src_tmr_mem_mbe");
		REPORT_FLAG(4,  "src_tmr_mem_sbe");
		REPORT_FLAG(3,  "src_ctxt_mem_mbe");
		REPORT_FLAG(2,  "src_ctxt_mem_sbe");
		REPORT_FLAG(1,  "src_fl_underflow");
		REPORT_FLAG(0,  "src_fl_overflow");
		break;
	case O_BRG_CTX_ERR_FIRST_INFO:
		REPORT_VALUE(GENMASK_ULL(63, 56), "context mem output syndrome");
		REPORT_VALUE(GENMASK_ULL(55, 51), "timer mem output ecc");
		REPORT_VALUE(GENMASK_ULL(50, 46), "timer mem output syndrome");
		REPORT_VALUE(GENMASK_ULL(45, 40), "fl mem output ecc");
		REPORT_VALUE(GENMASK_ULL(39, 34), "fl mem output syndrome");
		REPORT_VALUE(GENMASK_ULL(33, 27), "context mem outpu tecc");
		REPORT_VALUE(GENMASK_ULL(26, 20), "context mem output syndrome");
		REPORT_VALUE(GENMASK_ULL(19, 15), "timer mem output ecc");
		REPORT_VALUE(GENMASK_ULL(14, 10), "timer mem output syndrome");
		REPORT_VALUE(GENMASK_ULL(9,   5), "fl mem output ecc");
		REPORT_VALUE(GENMASK_ULL(4,   0), "fl mem output syndrome");
		break;
	case O_BRG_0_ERR_STS:
	case O_BRG_0_ERR_FIRST_HOST:
	case O_BRG_1_ERR_STS:
	case O_BRG_1_ERR_FIRST_HOST:
	case O_BRG_2_ERR_STS:
	case O_BRG_2_ERR_FIRST_HOST:
		REPORT_FLAG(63, "brg_tpm_egrq_ctxs_cmp_overflow");
		REPORT_FLAG(62, "brg_tpm_egrq_rddata_overflow");
		REPORT_FLAG(61, "brg_tpm_egrq_cmp_overflow");
		REPORT_FLAG(60, "brg_tpm_egrq_nhcmd_overflow");
		REPORT_FLAG(59, "brg_tpm_egrq_hcmd_oveflow");
		REPORT_FLAG(58, "brg_egrq_lnep_wrcmp_overflow");
		REPORT_FLAG(57, "brg_egrq_lnep_rddata_overflow");
		REPORT_FLAG(56, "brg_egrq_lnep_h_wrdata_overflow");
		REPORT_FLAG(55, "brg_egrq_lnep_nh_wrdata_overflow");
		REPORT_FLAG(54, "brg_egrq_lnep_h_cmd_overflow");
		REPORT_FLAG(53, "brg_egrq_lnep_nh_cmd_overflow");
		REPORT_FLAG(52, "brg_egrq_lnep_ctxd_cmp_overflow");
		REPORT_FLAG(51, "brg_egrq_lnep_ctxd_rr_overflow");
		REPORT_FLAG(50, "brg_egrq_lnep_wrcmp_egr_overflow");
		REPORT_FLAG(49, "brg_egrq_lnep_rddata_egr_overflow");
		REPORT_FLAG(48, "brg_egrq_lnep_hegr_uport_overflow");
		REPORT_FLAG(47, "brg_egrq_lnep_nhegr_uport_overflow");
		REPORT_FLAG(46, "brg_egrq_lnep_hegr_fid_overflow");
		REPORT_FLAG(45, "brg_egrq_lnep_nhegr_fid_overflow");
		REPORT_FLAG(44, "brg_egrq_lnep_h_egrdata2nd_overflow");
		REPORT_FLAG(43, "brg_egrq_lnep_nh_egrdata2nd_overflow");
		REPORT_FLAG(42, "brg_egrq_lnep_h_egrdata_overflow");
		REPORT_FLAG(41, "brg_egrq_lnep_nh_egrdata_overflow");
		REPORT_FLAG(40, "brg_egrq_lnep_h_egrcmd_overflow");
		REPORT_FLAG(39, "brg_egrq_lnep_nh_egrcmd_overflow");
		REPORT_FLAG(38, "brg_tpm_egrq_rdata_sbe");
		REPORT_FLAG(37, "brg_tpm_egrq_nhdata2_jbe");
		REPORT_FLAG(36, "brg_tpm_egrq_hdata2_sbe");
		REPORT_FLAG(35, "brg_tpm_egrq_nhdata_sbe");
		REPORT_FLAG(34, "brg_tpm_egrq_hdata_sbe");
		REPORT_FLAG(33, "brg_tpm_egrq_nhcmd_sbe");
		REPORT_FLAG(32, "brg_tpm_egrq_hcmd_sbe");
		REPORT_FLAG(31, "brg_tpm_egrq_cmp_sbe");
		REPORT_FLAG(30, "brg_tpm_egrq_ctxs_rdata_to_sbe");
		REPORT_FLAG(29, "brg_tpm_egrq_ctxs_to_sbe");
		REPORT_FLAG(28, "brg_tpm_egrq_ctxs_rdata_sbe");
		REPORT_FLAG(27, "brg_tpm_egrq_tcmp_sbe");
		REPORT_FLAG(26, "brg_tpm_egrq_rdata_sbe");
		REPORT_FLAG(25, "brg_tpm_egrq_tcmpctx_sbe");
		REPORT_FLAG(24, "brg_tpm_egrq_nhcmd_sbe");
		REPORT_FLAG(23, "brg_tpm_egrq_hcmd_sbe");
		REPORT_FLAG(22, "brg_ingrq_cmpfifo_sbe");
		REPORT_FLAG(21, "brg_ingrq_rdatafifo_sbe");
		REPORT_FLAG(20, "brg_ingrq_h_datafifo_sbe");
		REPORT_FLAG(19, "brg_ingrq_nh_datafifo_sbe");
		REPORT_FLAG(18, "brg_ingrq_h_cmdfifo_sbe");
		REPORT_FLAG(17, "brg_ingrq_nh_cmdfifo_sbe");
		REPORT_FLAG(16, "brg_egrq_ctxd_cmp_sbe");
		REPORT_FLAG(15, "brg_egrq_ctxd_rr_sbe");
		REPORT_FLAG(14, "brg_egrq_cmp_mem_sbe");
		REPORT_FLAG(13, "brg_egrq_rr_mem_sbe");
		REPORT_FLAG(12, "brg_egrq_h_urt_sbe");
		REPORT_FLAG(11, "brg_egrq_nh_urt_sbe");
		REPORT_FLAG(10, "brg_egrq_h_fid_sbe");
		REPORT_FLAG(9,  "brg_egrq_nh_fid_sbe");
		REPORT_FLAG(8,  "brg_egrq_nh_data2_sbe");
		REPORT_FLAG(7,  "brg_egrq_h_data2_sbe");
		REPORT_FLAG(6,  "brg_egrq_nh_data_sbe");
		REPORT_FLAG(5,  "brg_egrq_h_data_sbe");
		REPORT_FLAG(4,  "brg_egrq_nh_cmd_sbe");
		REPORT_FLAG(3,  "brg_egrq_h_cmd_sbe");
		REPORT_FLAG(2,  "ctxd_rsparb_cmp_sbe");
		REPORT_FLAG(1,  "ctxd_rsparb_rr_sbe");
		REPORT_FLAG(0,  "addr2fid_sbe");
		break;
	case O_TPM_ERR_STS:
	case O_TPM_ERR_FIRST_HOST:
		REPORT_FLAG(47,                  "control_flit_except_idle_received");
		REPORT_FLAG(46,                  "routeinfo_head_not_received");
		REPORT_FLAG(45,                  "async_credit_fifo_ovflw_sync");
		REPORT_FLAG(44,                  "async_data_fifo_ovflw");
		REPORT_FLAG(43,                  "shallow_fifo_full_96b");
		REPORT_FLAG(42,                  "shallow_fifo_full_24b");
		REPORT_FLAG(41,                  "shallow_fifo_full_16b");
		REPORT_VALUE(GENMASK_ULL(40, 9), "flit_stor_err_sts");
		REPORT_FLAG(8,                   "address_allocation_err");
		REPORT_FLAG(7,                   "invalid_vl_err");
		REPORT_FLAG(6,                   "routeinfo_ecc_mbe_err");
		REPORT_FLAG(5,                   "routeinfo_ecc_sbe_err");
		REPORT_FLAG(4,                   "qflitdata_ecc_mbe_err");
		REPORT_FLAG(3,                   "qflitdata_ecc_sbe_err");
		REPORT_FLAG(2,                   "routeinfo_parity_err");
		REPORT_FLAG(1,                   "tailless_packet_err");
		REPORT_FLAG(0,                   "csr_addr_err");
		break;
	case O_TPM_ERR_RINFO_SBE_INFO:
	case O_TPM_ERR_RINFO_MBE_INFO:
		REPORT_VALUE(GENMASK_ULL(9, 7), "rport");
		REPORT_VALUE(GENMASK_ULL(5, 0), "syndrome");
		break;
	case O_TPM_ERR_QFLITDATA_SBE_INFO:
	case O_TPM_ERR_QFLITDATA_MBE_INFO:
		REPORT_VALUE(GENMASK_ULL(13, 11), "rport");
		REPORT_VALUE(GENMASK_ULL(9,   8), "domain");
		REPORT_VALUE(GENMASK_ULL(7,   0), "syndrome");
		break;
	case O_TPM_ERR_RINFO_PE_INFO:
		REPORT_VALUE(GENMASK_ULL(3, 1), "rport");
		break;
	case O_TPM_ERR_RSVD_VL_INFO:
		REPORT_VALUE(GENMASK_ULL(2, 0), "rsvd_vl_port");
		break;
	case O_RPM_INQ_PORT0_ERR_STS:
	case O_RPM_INQ_PORT0_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT1_ERR_STS:
	case O_RPM_INQ_PORT1_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT2_ERR_STS:
	case O_RPM_INQ_PORT2_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT3_ERR_STS:
	case O_RPM_INQ_PORT3_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT4_ERR_STS:
	case O_RPM_INQ_PORT4_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT5_ERR_STS:
	case O_RPM_INQ_PORT5_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT6_ERR_STS:
	case O_RPM_INQ_PORT6_ERR_FIRST_HOST:
	case O_RPM_INQ_PORT7_ERR_STS:
	case O_RPM_INQ_PORT7_ERR_FIRST_HOST:
		REPORT_FLAG(26, "qfft_12f_bad_err");
		REPORT_FLAG(25, "vl_ack_parity_err");
		REPORT_FLAG(24, "vlr_xlate_err");
		REPORT_FLAG(23, "vl2_crdt_err_uf");
		REPORT_FLAG(22, "vl1_crdt_err_uf");
		REPORT_FLAG(21, "vl0_crdt_err_uf");
		REPORT_FLAG(20, "vl2_crdt_err_of");
		REPORT_FLAG(19, "vl1_crdt_err_of");
		REPORT_FLAG(18, "vl0_crdt_err_of");
		REPORT_FLAG(17, "vl2_txq_err_of");
		REPORT_FLAG(16, "vl2_txq_err_uf");
		REPORT_FLAG(15, "vl2_bq_err_of");
		REPORT_FLAG(14, "vl2_bq_err_uf");
		REPORT_FLAG(13, "vl2_sq_err_of");
		REPORT_FLAG(12, "vl2_sq_err_uf");
		REPORT_FLAG(11, "vl1_txq_err_of");
		REPORT_FLAG(10, "vl1_txq_err_uf");
		REPORT_FLAG(9,  "vl1_bq_err_of");
		REPORT_FLAG(8,  "vl1_bq_err_uf");
		REPORT_FLAG(7,  "vl1_sq_err_of");
		REPORT_FLAG(6,  "vl1_sq_err_uf");
		REPORT_FLAG(5,  "vl0_txq_err_of");
		REPORT_FLAG(4,  "vl0_txq_err_uf");
		REPORT_FLAG(3,  "vl0_bq_err_of");
		REPORT_FLAG(2,  "vl0_bq_err_uf");
		REPORT_FLAG(1,  "vl0_sq_err_of");
		REPORT_FLAG(0,  "vl0_sq_err_uf");
		break;
	case O_RPM_INQ_FLSTOR_ERR_STS:
	case O_RPM_INQ_FLSTOR_ERR_FIRST_HOST:
		REPORT_FLAG(30, "sbe on 2f freelist");
		REPORT_FLAG(29, "sbe on 3f freelist");
		REPORT_FLAG(28, "sbe on 12f freelist");
		REPORT_FLAG(27, "sbe_2f flit data o/p to shfifo in port");
		REPORT_FLAG(26, "sbe_2f from freelist manager");
		REPORT_FLAG(25, "sbe_2f on link list reading entries/next pointer");
		REPORT_FLAG(24, "sbe_2f during flit data input to flit storage");
		REPORT_FLAG(23, "sbe_3f flit data o/p to shfifo in port");
		REPORT_FLAG(22, "sbe_3f from freelist manager");
		REPORT_FLAG(21, "sbe_3f on link list reading entries/next pointer");
		REPORT_FLAG(20, "sbe_3f during flit data input to flit storage");
		REPORT_FLAG(19, "sbe_12f flit data o/p to shfifo in port");
		REPORT_FLAG(18, "sbe_12f from freelist manager");
		REPORT_FLAG(17, "sbe_12f on link list reading entries/next pointer");
		REPORT_FLAG(16, "sbe_12f during flit data input to flit storage");
		REPORT_FLAG(15, "parity error");
		REPORT_FLAG(14, "mbe on 2f freelist");
		REPORT_FLAG(13, "mbe on 3f freelist");
		REPORT_FLAG(12, "mbe on 12f freelist");
		REPORT_FLAG(11, "mbe_2f flit data o/p to shfifo in port");
		REPORT_FLAG(10, "mbe_2f from freelist manager");
		REPORT_FLAG(9,  "mbe_2f on link list reading entries/next pointer");
		REPORT_FLAG(8,  "mbe_2f during flit data input to flit storage");
		REPORT_FLAG(7,  "mbe_3f flit data o/p to shfifo in port");
		REPORT_FLAG(6,  "mbe_3f from freelist manager");
		REPORT_FLAG(5,  "mbe_3f on link list reading entries/next pointer");
		REPORT_FLAG(4,  "mbe_3f during flit data input to flit storage");
		REPORT_FLAG(3,  "mbe_12f flit data o/p to shfifo in port");
		REPORT_FLAG(2,  "mbe_12f from freelist manager");
		REPORT_FLAG(1,  "mbe_12f on link list reading entries/next pointer");
		REPORT_FLAG(0,  "mbe_12f during flit data input to flit storage");
		break;
	case O_BRG_RTP_ERR_STS:
	case O_BRG_RTP_ERR_FIRST_HOST:
		REPORT_FLAG(31,                   "cport_dos_err_lweg_redir");
		REPORT_FLAG(30,                   "csr_rd_miss");
		REPORT_FLAG(29,                   "csr_wr_miss");
		REPORT_VALUE(GENMASK_ULL(28, 27), "tcam_parity_err");
		REPORT_FLAG(26,                   "loop_pkt_err");
		REPORT_FLAG(25,                   "zero_valid_oports_err");
		REPORT_FLAG(24,                   "adaptive_cport_err");
		REPORT_FLAG(23,                   "deterministic_gt1_err");
		REPORT_FLAG(22,                   "cport_dos_err");
		REPORT_VALUE(GENMASK_ULL(21, 16), "urt_top_err");
		REPORT_FLAG(15,                   "dlid_top_err");
		REPORT_FLAG(14,                   "mlid_top_err");
		REPORT_FLAG(13,                   "op4sel_mbe");
		REPORT_FLAG(12,                   "op4sel_sbe");
		REPORT_VALUE(GENMASK_ULL(11,  6), "urt_mbe");
		REPORT_VALUE(GENMASK_ULL(5,   0), "urt_sbe");
		break;
	case O_BRG_RTP_ERR_FIRST_INFO:
		REPORT_VALUE(GENMASK_ULL(63, 62), "l2_pkt");
		REPORT_FLAG(61,                   "mcast");
		REPORT_FLAG(60,                   "tcam_hit");
		REPORT_VALUE(GENMASK_ULL(59, 56), "vlr");
		REPORT_VALUE(GENMASK_ULL(55, 53), "rc");
		REPORT_VALUE(GENMASK_ULL(52, 48), "sc");
		REPORT_VALUE(GENMASK_ULL(47, 24), "dlid");
		REPORT_VALUE(GENMASK_ULL(23,  0), "slid");
		break;
	}
}

/* END use of error spec helpers */
#undef REPORT_FLAG
#undef REPORT_VALUE
#undef REPORT_ENUM

/* returns true if the csr should be reported as an error */
static bool is_error(struct fport *port, u64 error, const struct error_reg *result,
		     const struct register_armed *armed)
{
	if (test_bit(port->lpn, port->sd->fport_lpns)) {
		switch (result->offset) {
		case O_FPC_ERR_STS:
		case O_FPC_ERR_FIRST_HOST:
			/* "link_err" (bit 19) is not a fatal error */
			return error & ~BIT_ULL(19);
		case O_FPC_ERR_INFO_PORTRCV:
		case O_FPC_ERR_INFO_PORTRCV_HDR0_A:
		case O_FPC_ERR_INFO_PORTRCV_HDR0_B:
		case O_FPC_ERR_INFO_PORTRCV_HDR1_A:
		case O_FPC_ERR_INFO_PORTRCV_HDR1_B:
			return armed->fpc_err_info_portrcv;
		case O_FPC_ERR_INFO_FMCONFIG:
			return armed->fpc_err_info_fmconfig;
		case O_FPC_ERR_INFO_FLOW_CTRL:
			return armed->fpc_err_info_flow_ctrl;
		case O_FPC_ERR_INFO_UNCORRECTABLE:
			return armed->fpc_err_info_uncorrectable;
		case O_FPC_ERR_INFO_PORTRCVCONSTRAINT:
			return armed->fpc_err_info_portrcvconstraint;
		case O_RTP_ERR_FIRST_INFO:
			return armed->rtp_err_first_info;
		case O_TP_ERR_ERROR_INFO:
			return armed->tp_err_error_info;
		case O_TP_ERR_PKEY_ERROR_INFO:
			return armed->tp_err_pkey_error_info;
		case O_INQ_ERR_STS:
			/* "init_ram_complete" (bit 27) is not an error */
			return error & ~BIT_ULL(27);
		case O_TP_ERR_STS_0:
		case O_TP_ERR_FIRST_HOST_0:
			/* "congested" (bit 13) is not an error */
			return error & ~BIT_ULL(13);
		case O_8051_ERR_STS:
			/*
			 * "set_by_8051" (bit 0) is a status bit indicating
			 * the validity of a non-PAWR register (which itself
			 * may contain a non-error host command status)
			 *
			 * "lost_8051_heart_beat" (bit 1) can false positive due
			 * to reset flow timing.
			 */
			return error & ~(BIT_ULL(1) | BIT_ULL(0));
		case O_LCB_ERR_STS:
		case O_LCB_ERR_FIRST_HOST:
			/*
			 * "fec_err_cnt_hit_limit" (bit 32) is not a fatal error
			 * "crc_err_cnt_hit_limit" (bit 8) is not a fatal error
			 * "reinit_from_peer" (bit 6) is not a fatal error
			 * "seq_crc_err" (bit 5) is not a fatal error
			 */
			return error & ~(BIT_ULL(32) | BIT_ULL(8) | BIT_ULL(6) | BIT_ULL(5));
		case O_LCB_ERR_INFO_FEC_CERR_MASK_VLD:
			/*
			 * this csr has a non-zero power on default.
			 * it is only an error if "val" (bit 0) is set.
			 */
			return error & BIT_ULL(0);
		}
	}

	if (test_bit(port->lpn, port->sd->bport_lpns)) {
		switch (result->offset) {
		case O_BRG_CTX_ERR_FIRST_INFO:
			return armed->brg_ctx_err_first_info;
		case O_BRG_FIDMISS_INFO:
			return armed->brg_fidmiss_info;
		case O_BRG_SRCCTXT_DUP_RSPINFO:
			return armed->brg_srcctxt_dup_rspinfo;
		case O_BRG_DSTCTXT_DUP_RSPINFO:
			return armed->brg_dstctxt_dup_rspinfo;
		case O_BRG_0_ERR_FIRST_INFO:
			return armed->brg_0_err_first_info;
		case O_BRG_1_ERR_FIRST_INFO:
			return armed->brg_1_err_first_info;
		case O_BRG_2_ERR_FIRST_INFO:
			return armed->brg_2_err_first_info;
		case O_TPM_ERR_RINFO_SBE_INFO:
			return armed->tpm_err_rinfo_sbe_info;
		case O_TPM_ERR_RINFO_MBE_INFO:
			return armed->tpm_err_rinfo_mbe_info;
		case O_TPM_ERR_QFLITDATA_SBE_INFO:
			return armed->tpm_err_qflitdata_sbe_info;
		case O_TPM_ERR_QFLITDATA_MBE_INFO:
			return armed->tpm_err_qflitdata_mbe_info;
		case O_TPM_ERR_RINFO_PE_INFO:
			return armed->tpm_err_rinfo_pe_info;
		case O_TPM_ERR_RSVD_VL_INFO:
			return armed->tpm_err_rsvd_vl_info;
		case O_TPM_ERR_INQ_SBE_ERR_INFO:
			return armed->tpm_err_inq_sbe_err_info;
		case O_TPM_ERR_INQ_MBE_ERR_INFO:
			return armed->tpm_err_inq_mbe_err_info;
		case O_BRG_RTP_ERR_FIRST_INFO:
			return armed->brg_rtp_err_first_info;
		}
	}

	return true;
}

static bool report_errors(struct fport *port, const u64 *errors,
			  const struct error_reg *regs, size_t len,
			  const struct register_armed *armed)
{
	bool has_errors = false;
	size_t i;

	mutex_lock(&report_lock);

	for (i = 0; i < len; ++i) {
		if (!errors[i])
			continue;

		if (!is_error(port, errors[i], regs + i, armed))
			continue;

		fport_info(port, "sticky hw error: %s 0x%016llx (%llu)\n",
			   regs[i].name, errors[i], errors[i]);

		if (test_bit(port->lpn, port->sd->fport_lpns))
			report_fabric_error(port, regs[i].offset, errors[i]);
		else if (test_bit(port->lpn, port->sd->bport_lpns))
			report_bridge_error(port, regs[i].offset, errors[i]);

		has_errors = true;
	}

	mutex_unlock(&report_lock);

	return has_errors;
}

static void arm_fabric_errors(struct fport *port, const u64 *errors,
			      const struct error_reg *regs, size_t len,
			      struct register_armed *armed)
{
	int i;

	for (i = 0; i < len; ++i) {
		switch (regs[i].offset) {
		case O_FPC_ERR_STS:
		case O_FPC_ERR_FIRST_HOST:
			if (errors[i] & BIT_ULL(55))
				armed->fpc_err_info_portrcv = true;
			if (errors[i] & BIT_ULL(54))
				armed->fpc_err_info_fmconfig = true;
			if (errors[i] & BIT_ULL(53))
				armed->fpc_err_info_uncorrectable = true;
			if (errors[i] & BIT_ULL(52))
				armed->fpc_err_info_portrcvconstraint = true;
			break;
		case O_RTP_ERR_STS:
		case O_RTP_ERR_FIRST_HOST:
			if (errors[i])
				armed->rtp_err_first_info = true;
			break;
		case O_TP_ERR_STS_0:
		case O_TP_ERR_FIRST_HOST_0:
			if (errors[i])
				armed->tp_err_error_info = true;
			if (errors[i] & BIT_ULL(0))
				armed->tp_err_pkey_error_info = true;
			break;
		case O_TP_ERR_STS_1:
		case O_TP_ERR_FIRST_HOST_1:
			if (errors[i])
				armed->tp_err_error_info = true;
			break;
		}
	}
}

static void arm_bridge_errors(struct fport *port, const u64 *errors,
			      const struct error_reg *regs, size_t len,
			      struct register_armed *armed)
{
	int i;

	for (i = 0; i < len; ++i) {
		switch (regs[i].offset) {
		case O_BRG_CTX_ERR_STS:
		case O_BRG_CTX_ERR_FIRST_HOST:
			if (errors[i])
				armed->brg_ctx_err_first_info = true;
			break;
		case O_BRG_FIDMISS:
			if (errors[i])
				armed->brg_fidmiss_info = true;
			break;
		case O_BRG_SRCCTXT_DUP_RSP:
			if (errors[i])
				armed->brg_srcctxt_dup_rspinfo = true;
			break;
		case O_BRG_DSTCTXT_DUP_RSP:
			if (errors[i])
				armed->brg_dstctxt_dup_rspinfo = true;
			break;
		case O_BRG_0_ERR_STS:
		case O_BRG_0_ERR_FIRST_HOST:
			if (errors[i])
				armed->brg_0_err_first_info = true;
			break;
		case O_BRG_1_ERR_STS:
		case O_BRG_1_ERR_FIRST_HOST:
			if (errors[i])
				armed->brg_1_err_first_info = true;
			break;
		case O_BRG_2_ERR_STS:
		case O_BRG_2_ERR_FIRST_HOST:
			if (errors[i])
				armed->brg_2_err_first_info = true;
			break;
		case O_TPM_ERR_RINFO_SBE_COUNT:
			if (errors[i])
				armed->tpm_err_rinfo_sbe_info = true;
			break;
		case O_TPM_ERR_RINFO_MBE_COUNT:
			if (errors[i])
				armed->tpm_err_rinfo_mbe_info = true;
			break;
		case O_TPM_ERR_QFLITDATA_SBE_COUNT:
			if (errors[i])
				armed->tpm_err_qflitdata_sbe_info = true;
			break;
		case O_TPM_ERR_QFLITDATA_MBE_COUNT:
			if (errors[i])
				armed->tpm_err_qflitdata_mbe_info = true;
			break;
		case O_TPM_ERR_RINFO_PE_COUNT:
			if (errors[i])
				armed->tpm_err_rinfo_pe_info = true;
			break;
		case O_TPM_ERR_RSVD_VL_CNT:
			if (errors[i])
				armed->tpm_err_rsvd_vl_info = true;
			break;
		case O_TPM_ERR_STORG_SBE_ERR_CNT_0:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_1:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_2:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_3:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_4:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_5:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_6:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_7:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_8:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_9:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_10:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_11:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_12:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_13:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_14:
		case O_TPM_ERR_STORG_SBE_ERR_CNT_15:
			if (errors[i])
				armed->tpm_err_inq_sbe_err_info = true;
			break;
		case O_TPM_ERR_STORG_MBE_ERR_CNT_0:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_1:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_2:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_3:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_4:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_5:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_6:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_7:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_8:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_9:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_10:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_11:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_12:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_13:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_14:
		case O_TPM_ERR_STORG_MBE_ERR_CNT_15:
			if (errors[i])
				armed->tpm_err_inq_mbe_err_info = true;
			break;
		case O_BRG_RTP_ERR_STS:
		case O_BRG_RTP_ERR_FIRST_HOST:
			if (errors[i])
				armed->brg_rtp_err_first_info = true;
			break;
		}
	}
}

static void reset_fabric_errors(struct fsubdev *sd)
{
	struct fabric_port_status_rsp rsp = {};
	struct register_armed armed = {};
	struct fport *port;
	int err;
	u8 lpn;

	for_each_fabric_port(port, lpn, sd) {
		err = read_fabric_errors(port, &rsp.rsp);
		if (err) {
			sd_warn(sd, "unable to read fabric errors: %d\n", err);
			return;
		}

		arm_fabric_errors(port, rsp.regs, s_fabric_regs,
				  ARRAY_SIZE(s_fabric_regs), &armed);

		report_errors(port, rsp.regs, s_fabric_regs,
			      ARRAY_SIZE(s_fabric_regs), &armed);

		err = clear_errors(port, rsp.regs, s_fabric_regs,
				   ARRAY_SIZE(s_fabric_regs));
		if (err) {
			sd_warn(sd, "unable to clear fabric errors: %d\n", err);
			return;
		}
	}
}

static void reset_bridge_errors(struct fsubdev *sd)
{
	struct bridge_port_status_rsp *rsp;
	struct register_armed armed = {};
	struct fport *port;
	int err;
	u8 lpn;

	rsp = kmalloc(sizeof(*rsp), GFP_KERNEL);
	if (!rsp)
		return;

	for_each_bridge_port(port, lpn, sd) {
		err = read_bridge_errors(port, &rsp->rsp);
		if (err) {
			sd_warn(sd, "unable to read bridge errors: %d\n", err);
			goto end;
		}

		arm_bridge_errors(port, rsp->regs, s_bridge_regs,
				  ARRAY_SIZE(s_bridge_regs), &armed);

		report_errors(port, rsp->regs, s_bridge_regs,
			      ARRAY_SIZE(s_bridge_regs), &armed);

		err = clear_errors(port, rsp->regs, s_bridge_regs,
				   ARRAY_SIZE(s_bridge_regs));
		if (err) {
			sd_warn(sd, "unable to clear bridge errors: %d\n", err);
			goto end;
		}
	}

end:
	kfree(rsp);
}

void reset_errors(struct fsubdev *sd)
{
	reset_fabric_errors(sd);
	reset_bridge_errors(sd);
}

