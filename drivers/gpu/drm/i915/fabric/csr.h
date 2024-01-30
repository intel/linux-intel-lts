/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 *
 */

#ifndef IAF_CSR_H_INCLUDED
#define IAF_CSR_H_INCLUDED

#include <linux/types.h>

#define CSR_SIZE sizeof(u64)

/*
 * TOP CSRs
 */

#define CP_ADDR_TOP_BASE		(0)
#define CPORT_INIT_CTRL_ADDR		(CP_ADDR_TOP_BASE + 0)
#define RISC_RESET_BIT			BIT_ULL(0)
#define RISC_NMI_BIT			BIT_ULL(2)
#define RISC_PAUSED_BIT			BIT_ULL(3)
#define BOOTROM_PARITY_ERROR_BIT	BIT_ULL(4)
#define WATCHDOG_STATUS_BIT		BIT_ULL(5)
#define RISC_RESET_CAUSE_BIT		BIT_ULL(7)
#define DINIT_FSM_STATE_MASK		GENMASK_ULL(31, 8)
#define CSR_GPIO_CTRL			(CP_ADDR_TOP_BASE + 0x18)
#define GPIO_CTRL_PIN_LINK_CONFIG	GENMASK_ULL(7, 3)
#define GPIO_CTRL_OE_LINK_CONFIG	GENMASK_ULL(15, 11)
#define CSR_ASIC_REV_INFO		(CP_ADDR_TOP_BASE + 0x40)
#define MASK_ARI_PLATFORM		GENMASK_ULL(55, 40)
#define MASK_ARI_STEP			GENMASK_ULL(39, 32)
#define ARB_REG_BASE			(CP_ADDR_TOP_BASE + 0x100)
#define FW_MIN_SVN_ADDR			(ARB_REG_BASE + 0x0)
#define FW_UPDATED_SVN_ADDR		(ARB_REG_BASE + 0x8)
#define FW_SVN_MASK			GENMASK_ULL(15, 0)
#define CSR_CP_DEV_EFUSE_VERSION	0x18150
#define CP_DEV_EFUSE_VERSION_VARIANT_MASK	GENMASK_ULL(23, 16)
#define CP_DEV_EFUSE_VERSION_VARIANT_NORMAL	0
#define CP_DEV_EFUSE_VERSION_VARIANT_EXPORT	1

#define CSR_ROPTION				0x80000008
#define ROPTION_FORCE_FEC_HEAVY_ON		BIT(0)
#define ROPTION_CBUF_MGR_TRACE_XBAR		BIT(1)
#define ROPTION_CBUF_MGR_TRACE_PCI		BIT(2)
#define ROPTION_CBUF_MGR_TRACE_ATTRIBUTES	BIT(3)
#define ROPTION_PD_PHASE			GENMASK(5, 4)
#define ROPTION_STOP_IN_CONFIGLT_WITH_BCC	BIT(6)
#define ROPTION_TRACE_ERROR_RECOVERY		BIT(7)
#define ROPTION_EV_MODE				BIT(8)
#define ROPTION_TRACE_LNI_DEBUG			BIT(9)
#define ROPTION_TRACE_SBUS_OPS			BIT(10)
#define ROPTION_TRACE_STOP_ON_FULL		BIT(11)
#define ROPTION_12G_ALLOWED			BIT(12)
#define ROPTION_NO_DLA				BIT(13)
#define ROPTION_DYNAMIC_SCREEN			BIT(14)
#define ROPTION_UNUSED15			BIT(15)
#define ROPTION_LOOP_BANDWIDTH			GENMASK(20, 16)
#define ROPTION_1D_CODING			BIT(21)
#define ROPTION_LCB_LOOPBACK			GENMASK(23, 22)
#define ROPTION_TX_PRBS_GEN			BIT(24)
#define ROPTION_STOP_IN_DEBOUNCE		BIT(25)
#define ROPTION_NO_TUNING_IN_DEBOUNCE		BIT(26)
#define ROPTION_CONFIGLT_NO_HAMMERS		BIT(27)
#define ROPTION_FORCE_LNI			BIT(28)
#define ROPTION_ALLOW_MARGINAL_LINKS		BIT(29)
#define ROPTION_FORCE_FEC_LIGHT_ON		BIT(30)
#define ROPTION_SERDES_ILB			BIT(31)

/*
 * PORT CSRs
 */

#define CSR_PORTS_BASE 0x10000000

/*
 * Each port is at a 4MB offset from CSR_PORTS_BASE, index by physical port
 * number.
 */
#define CSR_PORT_OFFSET 0x400000

/*
 * Fabric ports are in memory region 0, while bridge ports are in region 1.
 *
 * When referencing bridge port addresses, both when using the raw CSR ops
 * as well as the LINK_MGR_PORT_CSR ops, you must add this offset to
 * addresses.
 */
#define CSR_REGION1_OFFSET 0x20000000

/**
 * get_raw_port_base - Calculates the base address of the given port for
 * use with the CSR_RAW ops.
 * @ppn: physical port number
 *
 * Return: The base address required for the CSR_RAW ops.
 */
static inline u32 get_raw_port_base(u8 ppn)
{
	return CSR_PORTS_BASE + ((ppn) - 1) * CSR_PORT_OFFSET;
}

#define CSR_FIDGEN_BASE 0

#define CSR_FIDGEN_MASK_A     (CSR_FIDGEN_BASE + 0x00)
#define CSR_FIDGEN_SHIFT_A    (CSR_FIDGEN_BASE + 0x08)
#define CSR_FIDGEN_MASK_B     (CSR_FIDGEN_BASE + 0x10)
#define CSR_FIDGEN_SHIFT_B    (CSR_FIDGEN_BASE + 0x18)
#define CSR_FIDGEN_RSVD0      (CSR_FIDGEN_BASE + 0x20)
#define CSR_FIDGEN_RSVD1      (CSR_FIDGEN_BASE + 0x28)
#define CSR_FIDGEN_MASK_H     (CSR_FIDGEN_BASE + 0x30)
#define CSR_FIDGEN_SHIFT_H    (CSR_FIDGEN_BASE + 0x38)
#define CSR_FIDGEN_MODULO     (CSR_FIDGEN_BASE + 0x40)
#define CSR_FIDGEN_WHOAMI     (CSR_FIDGEN_BASE + 0x48)
#define CSR_FIDGEN_MASK_D     (CSR_FIDGEN_BASE + 0x50)
#define CSR_FIDGEN_STATIC_RND (CSR_FIDGEN_BASE + 0x58)

/*
 * These apply to all of the shift registers.  Bit 6 is the direction of
 * shift, while bits 5 through 0 are the amount to shift by.
 */
#define MASK_FIDGEN_SHIFT_BY    GENMASK(5, 0)
#define MASK_FIDGEN_SHIFT_RIGHT GENMASK(6, 6)

/*
 * The LUT contains 8192 64-bit registers, each mapping a lookup index
 * to a 20-bit DFID.
 */
#define CSR_FIDGEN_LUT_BASE (CSR_FIDGEN_BASE + 0x10000)

#define CSR_FIDGEN_LUT_INDEX_WIDTH 13

#define CSR_BRIDGE_TOP_BASE 0x30000

#define CSR_BT_CTX_TRACKER_CFG CSR_BRIDGE_TOP_BASE
#define MASK_BT_CTC_ENABLE_SRC          GENMASK(0, 0)
#define MASK_BT_CTC_ENABLE_DST          GENMASK(1, 1)
#define MASK_BT_CTC_DISABLE_TIMEOUT_SRC GENMASK(2, 2)
#define MASK_BT_CTC_DISABLE_TIMEOUT_DST GENMASK(3, 3)

#define CSR_BT_PORT_CTRL (CSR_BRIDGE_TOP_BASE + 0x08)
#define MASK_BT_POC_DROP_FBRC_REQ     GENMASK(0, 0)
#define MASK_BT_POC_DROP_FBRC_REQ_ERR GENMASK(1, 1)
#define MASK_BT_POC_DROP_MDFI_REQ     GENMASK(2, 2)
#define MASK_BT_POC_DROP_MDFI_REQ_ERR GENMASK(3, 3)
#define MASK_BT_POC_DROP_FBRC_RSP     GENMASK(4, 4)
#define MASK_BT_POC_DROP_MDFI_RSP     GENMASK(5, 5)

#define CSR_BT_OUTSTANDING_TX (CSR_BRIDGE_TOP_BASE + 0x10)
#define MASK_BT_OT_FBRC GENMASK(15,  0)
#define MASK_BT_OT_MDFI GENMASK(31, 16)

#define CSR_BT_FLUSH_CONTEXT (CSR_BRIDGE_TOP_BASE + 0x18)
#define MASK_BT_FC_INITIATE    GENMASK(0, 0)
#define MASK_BT_FC_IN_PROGRESS GENMASK(1, 1)

#define CSR_BT_PAUSE_CTRL (CSR_BRIDGE_TOP_BASE + 0x20)
#define MASK_BT_PAC_FBRC_REQ GENMASK(0, 0)
#define MASK_BT_PAC_MDFI_REQ GENMASK(1, 1)
#define MASK_BT_PAC_FBRC_RSP GENMASK(2, 2)
#define MASK_BT_PAC_MDFI_RSP GENMASK(3, 3)

#define CSR_BT_PKG_ADDR_RANGE (CSR_BRIDGE_TOP_BASE + 0x28)
#define MASK_BT_PAR_BASE  GENMASK(18, 1)
#define MASK_BT_PAR_RANGE GENMASK(29, 20)

/* csr encodes address bits [45:32] in register bits [13:0] */
#define BT_ADDR_RANGE_SHIFT 32

#define CSR_BT_VC2SC_MAP (CSR_BRIDGE_TOP_BASE + 0x50)

#define CSR_BT_TILE0_RANGE (CSR_BRIDGE_TOP_BASE + 0x58)
#define MASK_BT_T0_VALID GENMASK(0, 0)
#define MASK_BT_T0_BASE  GENMASK(7, 1)
#define MASK_BT_T0_RANGE GENMASK(14, 8)

#define CSR_BT_TILE1_RANGE (CSR_BRIDGE_TOP_BASE + 0x60)
#define MASK_BT_T1_VALID GENMASK(0, 0)
#define MASK_BT_T1_BASE  GENMASK(7, 1)
#define MASK_BT_T1_RANGE GENMASK(14, 8)

/* xmit/recv flit/packet counter register offsets */
#define O_FPC_PORTRCV_DATA_CNT 0x100490
#define O_FPC_PORTRCV_PKT_CNT 0x100438
#define TP_PRF_XMIT_DATA_OFFSET 0x200458
#define TP_PRF_XMIT_PKTS_OFFSET 0x200400

/* LCB counter offsets */
#define O_LCB_ERR_INFO_OFFSET 0x308500
#define O_LCB_STS_RX_CRC_FEC_MODE 0x3087A8
#define O_LCB_STS_RX_LOGICAL_ID 0x3087B0
#define O_LCB_PRF_OFFSET 0x308700

#define BRG_PERF_OFFSET 0x34000
#define BRG_PERF_END (BRG_PERF_OFFSET + 0xB8)
#define BRG_ERR_OFFSET 0x36000
#define BRG_ERR_END (BRG_ERR_OFFSET + 0x198)
#define BRG_1_ERR_OFFSET (BRG_ERR_OFFSET + 0x200)
#define BRG_1_ERR_END (BRG_ERR_OFFSET + 0x228)
#define BRG_2_ERR_OFFSET (BRG_ERR_OFFSET + 0x300)
#define BRG_2_ERR_END (BRG_ERR_OFFSET + 0x328)
#define BRG_3_ERR_OFFSET (BRG_ERR_OFFSET + 0x400)
#define BRG_3_ERR_END (BRG_ERR_OFFSET + 0x428)
#define BRG_RTP_ERR_OFFSET (0x200000 + 0x1000)
#define BRG_RTP_ERR_END (BRG_RTP_ERR_OFFSET + 0x28)
#define BRG_RTP_STS_OFFSET (0x200000 + 0x2000)
#define BRG_RTP_STS_START_1 (BRG_RTP_STS_OFFSET + 0x10)
#define BRG_RTP_STS_END_1 (BRG_RTP_STS_OFFSET + 0x40)
#define BRG_RTP_STS_START_2 (BRG_RTP_STS_OFFSET + 0x80)
#define BRG_RTP_STS_END_2 (BRG_RTP_STS_OFFSET + 0xA0)
#define TPM_ERR_START (0x100000 + 0x300)
#define TPM_ERR_END (TPM_ERR_START + 0x108)
#define TPM_ERR_MBE_START (0x100000 + 0x4A0)
#define TPM_ERR_MBE_END (TPM_ERR_MBE_START + 0x88)
#define TPM_PRF_START (0x100000 + 0x840)
#define TPM_PRF_END (TPM_PRF_START + 0x108)

#define RPM_OFFSET 0x180000
#define RPM_INQ_ERR_OFFSET (RPM_OFFSET + 0x2000)
#define RPM_INQ_STS_OFFSET (RPM_OFFSET + 0x2900)
#define RPM_INQ_PRF_OFFSET (RPM_OFFSET + 0x2D00)

#define RPM_PORT_ERR_COUNTERS_START (RPM_INQ_PRF_OFFSET + 0x100)
#define RPM_PORT_ERR_COUNTERS_END (RPM_INQ_PRF_OFFSET + 0x7F8)

#define RPM_INQ_PORT0_ERR_START (RPM_INQ_ERR_OFFSET)
#define RPM_INQ_PORT0_ERR_END (RPM_INQ_ERR_OFFSET + 0x20)

#define RPM_INQ_PORT1_ERR_START (RPM_INQ_ERR_OFFSET + 0x80)
#define RPM_INQ_PORT1_ERR_END (RPM_INQ_ERR_OFFSET + 0xA0)

#define RPM_INQ_PORT2_ERR_START (RPM_INQ_ERR_OFFSET + 0x100)
#define RPM_INQ_PORT2_ERR_END (RPM_INQ_ERR_OFFSET + 0x120)

#define RPM_INQ_PORT3_ERR_START (RPM_INQ_ERR_OFFSET + 0x180)
#define RPM_INQ_PORT3_ERR_END (RPM_INQ_ERR_OFFSET + 0x1A0)

#define RPM_INQ_PORT4_ERR_START (RPM_INQ_ERR_OFFSET + 0x200)
#define RPM_INQ_PORT4_ERR_END (RPM_INQ_ERR_OFFSET + 0x220)

#define RPM_INQ_PORT5_ERR_START (RPM_INQ_ERR_OFFSET + 0x280)
#define RPM_INQ_PORT5_ERR_END (RPM_INQ_ERR_OFFSET + 0x2A0)

#define RPM_INQ_PORT6_ERR_START (RPM_INQ_ERR_OFFSET + 0x300)
#define RPM_INQ_PORT6_ERR_END (RPM_INQ_ERR_OFFSET + 0x320)

#define RPM_INQ_PORT7_ERR_START (RPM_INQ_ERR_OFFSET + 0x380)
#define RPM_INQ_PORT7_ERR_END (RPM_INQ_ERR_OFFSET + 0x3A0)

#define RPM_INQ_FLSTOR_ERR_START (RPM_INQ_ERR_OFFSET + 0x400)
#define RPM_INQ_FLSTOR_ERR_END (RPM_INQ_ERR_OFFSET + 0x420)

#define RPM_PORT_STS_START (RPM_INQ_STS_OFFSET + 0x200)
#define RPM_PORT_STS_END (RPM_INQ_STS_OFFSET + 0x2F8)

#define RPM_SBE_ERR_COUNTERS_START (RPM_INQ_PRF_OFFSET)
#define RPM_SBE_ERR_COUNTERS_END (RPM_INQ_PRF_OFFSET + 0x78)

#define RPM_MBE_ERR_COUNTERS_START (RPM_INQ_PRF_OFFSET + 0x80)
#define RPM_MBE_ERR_COUNTERS_END (RPM_INQ_PRF_OFFSET + 0xF8)

#define RPM_ARB_PERF_COUNTERS_START (RPM_INQ_PRF_OFFSET + 0x8C0)
#define RPM_ARB_PERF_COUNTERS_END (RPM_INQ_PRF_OFFSET + 0x938)

#define RPM_PERF_COUNTERS_START (RPM_INQ_PRF_OFFSET + 0x800)
#define RPM_PERF_COUNTERS_END (RPM_INQ_PRF_OFFSET + 0x8B8)

#define CRC_MODE GENMASK(1, 0)
#define FEC_MODE GENMASK(5, 4)

#define PEER_TX_ID_LN0 GENMASK(2, 0)
#define PEER_TX_ID_LN1 GENMASK(6, 4)
#define PEER_TX_ID_LN2 GENMASK(10, 8)
#define PEER_TX_ID_LN3 GENMASK(14, 12)

/* error csrs for sticky error reporting */
#define O_FPC_ERR_STS                         0x100000
#define O_FPC_ERR_CLR                         0x100008
#define O_FPC_ERR_FIRST_HOST                  0x100020
#define O_FPC_ERR_INFO_PORTRCV                0x100028
#define O_FPC_ERR_INFO_PORTRCV_HDR0_A         0x100030
#define O_FPC_ERR_INFO_PORTRCV_HDR0_B         0x100038
#define O_FPC_ERR_INFO_PORTRCV_HDR1_A         0x100040
#define O_FPC_ERR_INFO_PORTRCV_HDR1_B         0x100048
#define O_FPC_ERR_INFO_FMCONFIG               0x100050
#define O_FPC_ERR_INFO_FLOW_CTRL              0x100058
#define O_FPC_ERR_INFO_UNCORRECTABLE          0x100060
#define O_FPC_ERR_INFO_PORTRCVCONSTRAINT      0x100068
#define O_RTP_ERR_STS                         0x141000
#define O_RTP_ERR_CLR                         0x141008
#define O_RTP_ERR_FIRST_HOST                  0x141020
#define O_RTP_ERR_FIRST_INFO                  0x141028
#define O_INQ_ERR_STS                         0x182000
#define O_INQ_ERR_CLR                         0x182008
#define O_INQ_ERR_EN_HOST                     0x182018
#define O_INQ_ERR_FIRST_HOST                  0x182020
#define O_TP_ERR_STS_0                        0x200180
#define O_TP_ERR_CLR_0                        0x200188
#define O_TP_ERR_EN_HOST_0                    0x200198
#define O_TP_ERR_FIRST_HOST_0                 0x2001a0
#define O_TP_ERR_STS_1                        0x200200
#define O_TP_ERR_CLR_1                        0x200208
#define O_TP_ERR_EN_HOST_1                    0x200218
#define O_TP_ERR_FIRST_HOST_1                 0x200220
#define O_TP_ERR_ERROR_INFO                   0x200230
#define O_TP_ERR_PKEY_ERROR_INFO              0x200248
#define O_TP_ERR_SBE_ERROR_CNT                0x200250
#define O_TP_ERR_MBE_ERROR_CNT                0x200258
#define O_TP_PE_ERROR_CNT                     0x200260
#define O_8051_ERR_STS                        0x300100
#define O_8051_ERR_CLR                        0x300108
#define O_8051_FIRST_HOST                     0x300120
#define O_LCB_ERR_STS                         0x308480
#define O_LCB_ERR_CLR                         0x308488
#define O_LCB_ERR_FIRST_HOST                  0x3084a0
#define O_LCB_ERR_INFO_TOTAL_CRC_ERR          0x308500
#define O_LCB_ERR_INFO_CRC_ERR_LN0            0x308508
#define O_LCB_ERR_INFO_CRC_ERR_LN1            0x308510
#define O_LCB_ERR_INFO_CRC_ERR_LN2            0x308518
#define O_LCB_ERR_INFO_CRC_ERR_LN3            0x308520
#define O_LCB_ERR_INFO_CRC_ERR_MULTI_LN       0x308528
#define O_LCB_ERR_INFO_TX_REPLAY_CNT          0x308530
#define O_LCB_ERR_INFO_RX_REPLAY_CNT          0x308538
#define O_LCB_ERR_INFO_SEQ_CRC_CNT            0x308540
#define O_LCB_ERR_INFO_ESCAPE_0_ONLY_CNT      0x308548
#define O_LCB_ERR_INFO_ESCAPE_0_PLUS1_CNT     0x308550
#define O_LCB_ERR_INFO_ESCAPE_0_PLUS2_CNT     0x308558
#define O_LCB_ERR_INFO_REINIT_FROM_PEER_CNT   0x308560
#define O_LCB_ERR_INFO_SBE_CNT                0x308568
#define O_LCB_ERR_INFO_MISC_FLG_CNT           0x308570
#define O_LCB_ERR_INFO_ECC_INPUT_BUF          0x308578
#define O_LCB_ERR_INFO_ECC_INPUT_BUF_HGH      0x308580
#define O_LCB_ERR_INFO_ECC_INPUT_BUF_LOW      0x308588
#define O_LCB_ERR_INFO_ECC_REPLAY_BUF         0x308590
#define O_LCB_ERR_INFO_ECC_REPLAY_BUF_HGH     0x308598
#define O_LCB_ERR_INFO_ECC_REPLAY_BUF_LOW     0x3085A0
#define O_LCB_ERR_INFO_ECC_PM_TIME            0x3085A8
#define O_LCB_ERR_INFO_ECC_PM_TIME_HGH        0x3085B0
#define O_LCB_ERR_INFO_ECC_PM_TIME_LOW        0x3085B8
#define O_LCB_ERR_INFO_FEC_CERR_CNT_1         0x3085C0
#define O_LCB_ERR_INFO_FEC_CERR_CNT_2         0x3085C8
#define O_LCB_ERR_INFO_FEC_CERR_CNT_3         0x3085D0
#define O_LCB_ERR_INFO_FEC_CERR_CNT_4         0x3085D8
#define O_LCB_ERR_INFO_FEC_CERR_CNT_5         0x3085E0
#define O_LCB_ERR_INFO_FEC_CERR_CNT_6         0x3085E8
#define O_LCB_ERR_INFO_FEC_CERR_CNT_7         0x3085F0
#define O_LCB_ERR_INFO_FEC_CERR_CNT_8         0x3085F8
#define O_LCB_ERR_INFO_FEC_UERR_CNT           0x308600
#define O_LCB_ERR_INFO_FEC_CERR_MASK_0        0x308608
#define O_LCB_ERR_INFO_FEC_CERR_MASK_1        0x308610
#define O_LCB_ERR_INFO_FEC_CERR_MASK_2        0x308618
#define O_LCB_ERR_INFO_FEC_CERR_MASK_3        0x308620
#define O_LCB_ERR_INFO_FEC_CERR_MASK_4        0x308628
#define O_LCB_ERR_INFO_FEC_CERR_MASK_VLD      0x308630
#define O_LCB_ERR_INFO_FEC_CERR_MASK_MISS_CNT 0x308638
#define O_LCB_ERR_INFO_FEC_ERR_LN0            0x308640
#define O_LCB_ERR_INFO_FEC_ERR_LN1            0x308648
#define O_LCB_ERR_INFO_FEC_ERR_LN2            0x308650
#define O_LCB_ERR_INFO_FEC_ERR_LN3            0x308658
#define O_LCB_ERR_INFO_FX_RESYNC_CNT          0x308660
#define O_BRG_CTX_ERR_STS                     0x036000
#define O_BRG_CTX_ERR_CLR                     0x036008
#define O_BRG_CTX_ERR_FIRST_HOST              0x036020
#define O_BRG_CTX_ERR_FIRST_INFO              0x036028
#define O_SRC_CTXT_SBE_CNT                    0x036040
#define O_DST_CTXT_SBE_CNT                    0x036048
#define O_SRC_CTXT_MBE_CNT                    0x036050
#define O_DST_CTXT_MBE_CNT                    0x036058
#define O_BRG_INCMD_PKTPAR_ERR                0x036060
#define O_BRG_INPKT_POISON_SET                0x036068
#define O_BRG_INRSP_PKTPAR_ERR                0x036070
#define O_BRG_INDATA_PKTPAR_ERR               0x036078
#define O_BRG_OUTPKT_POISON_SET               0x036080
#define O_BRG_RPM_POISON_SET                  0x036088
#define O_TPM_BRG_POISON_SET                  0x036090
#define O_BRG_DROP_FABRIC_REQ                 0x036098
#define O_BRG_DROP_MDFI_REQ                   0x0360a0
#define O_BRG_DROP_FABRIC_RSP                 0x0360a8
#define O_BRG_DROP_MDFI_RSP                   0x0360b0
#define O_BRG_SRCCTXT_TO                      0x0360b8
#define O_BRG_DSTCTXT_TO                      0x0360c0
#define O_BRG_FIDMISS                         0x0360c8
#define O_BRG_FIDMISS_INFO                    0x0360d0
#define O_BRG_SRCCTXT_DUP_RSP                 0x0360d8
#define O_BRG_SRCCTXT_DUP_RSPINFO             0x0360e0
#define O_BRG_DSTCTXT_DUP_RSP                 0x0360e8
#define O_BRG_DSTCTXT_DUP_RSPINFO             0x0360f0
#define O_BRG_SFID_FILTER_DROP                0x0360f8
#define O_BRG_0_ERR_STS                       0x036100
#define O_BRG_0_ERR_CLR                       0x036108
#define O_BRG_0_ERR_FIRST_HOST                0x036120
#define O_BRG_0_ERR_FIRST_INFO                0x036128
#define O_BRG_SBE_ADDR2FID_ERR                0x036130
#define O_BRG_SBE_RSPARB_ERR                  0x036138
#define O_BRG_SBE_EGRQ_ERR                    0x036140
#define O_BRG_SBE_INGRQ_ERR                   0x036148
#define O_BRG_SBE_TPMEGRQ_ERR                 0x036150
#define O_BRG_SBE_TPMINGRQ_ERR                0x036158
#define O_BRG_MBE_ADDR2FID_ERR                0x036160
#define O_BRG_MBE_RSPARB_ERR                  0x036168
#define O_BRG_MBE_EGRQ_ERR                    0x036170
#define O_BRG_MBE_INGRQ_ERR                   0x036178
#define O_BRG_MBE_TPMEGRQ_ERR                 0x036180
#define O_BRG_MBE_TPMINGRQ_ERR                0x036188
#define O_BRG_SBE_TPMDECAP_ERR                0x036190
#define O_BRG_MBE_TPMDECAP_ERR                0x036198
#define O_BRG_1_ERR_STS                       0x036200
#define O_BRG_1_ERR_CLR                       0x036208
#define O_BRG_1_ERR_FIRST_HOST                0x036220
#define O_BRG_1_ERR_FIRST_INFO                0x036228
#define O_BRG_2_ERR_STS                       0x036300
#define O_BRG_2_ERR_CLR                       0x036308
#define O_BRG_2_ERR_FIRST_HOST                0x036320
#define O_BRG_2_ERR_FIRST_INFO                0x036328
#define O_TPM_ERR_STS                         0x100300
#define O_TPM_ERR_CLR                         0x100308
#define O_TPM_ERR_FIRST_HOST                  0x100320
#define O_TPM_ERR_RINFO_SBE_COUNT             0x100328
#define O_TPM_ERR_RINFO_SBE_INFO              0x100330
#define O_TPM_ERR_RINFO_MBE_COUNT             0x100338
#define O_TPM_ERR_RINFO_MBE_INFO              0x100340
#define O_TPM_ERR_QFLITDATA_SBE_COUNT         0x100348
#define O_TPM_ERR_QFLITDATA_SBE_INFO          0x100350
#define O_TPM_ERR_QFLITDATA_MBE_COUNT         0x100358
#define O_TPM_ERR_QFLITDATA_MBE_INFO          0x100360
#define O_TPM_ERR_RINFO_PE_COUNT              0x100368
#define O_TPM_ERR_RINFO_PE_INFO               0x100370
#define O_TPM_ERR_TAILLESS_PKT_CNT            0x100378
#define O_TPM_ERR_RSVD_VL_CNT                 0x100380
#define O_TPM_ERR_RSVD_VL_INFO                0x100388
#define O_TPM_ERR_STORG_SBE_ERR_CNT_0         0x100390
#define O_TPM_ERR_STORG_SBE_ERR_CNT_1         0x100398
#define O_TPM_ERR_STORG_SBE_ERR_CNT_2         0x1003a0
#define O_TPM_ERR_STORG_SBE_ERR_CNT_3         0x1003a8
#define O_TPM_ERR_STORG_SBE_ERR_CNT_4         0x1003b0
#define O_TPM_ERR_STORG_SBE_ERR_CNT_5         0x1003b8
#define O_TPM_ERR_STORG_SBE_ERR_CNT_6         0x1003c0
#define O_TPM_ERR_STORG_SBE_ERR_CNT_7         0x1003c8
#define O_TPM_ERR_STORG_SBE_ERR_CNT_8         0x1003d0
#define O_TPM_ERR_STORG_SBE_ERR_CNT_9         0x1003d8
#define O_TPM_ERR_STORG_SBE_ERR_CNT_10        0x1003e0
#define O_TPM_ERR_STORG_SBE_ERR_CNT_11        0x1003e8
#define O_TPM_ERR_STORG_SBE_ERR_CNT_12        0x1003f0
#define O_TPM_ERR_STORG_SBE_ERR_CNT_13        0x1003f8
#define O_TPM_ERR_STORG_SBE_ERR_CNT_14        0x100400
#define O_TPM_ERR_STORG_SBE_ERR_CNT_15        0x100408
#define O_TPM_ERR_STORG_MBE_ERR_CNT_0         0x1004a0
#define O_TPM_ERR_STORG_MBE_ERR_CNT_1         0x1004a8
#define O_TPM_ERR_STORG_MBE_ERR_CNT_2         0x1004b0
#define O_TPM_ERR_STORG_MBE_ERR_CNT_3         0x1004b8
#define O_TPM_ERR_STORG_MBE_ERR_CNT_4         0x1004c0
#define O_TPM_ERR_STORG_MBE_ERR_CNT_5         0x1004c8
#define O_TPM_ERR_STORG_MBE_ERR_CNT_6         0x1004d0
#define O_TPM_ERR_STORG_MBE_ERR_CNT_7         0x1004d8
#define O_TPM_ERR_STORG_MBE_ERR_CNT_8         0x1004e0
#define O_TPM_ERR_STORG_MBE_ERR_CNT_9         0x1004e8
#define O_TPM_ERR_STORG_MBE_ERR_CNT_10        0x1004f0
#define O_TPM_ERR_STORG_MBE_ERR_CNT_11        0x1004f8
#define O_TPM_ERR_STORG_MBE_ERR_CNT_12        0x100500
#define O_TPM_ERR_STORG_MBE_ERR_CNT_13        0x100508
#define O_TPM_ERR_STORG_MBE_ERR_CNT_14        0x100510
#define O_TPM_ERR_STORG_MBE_ERR_CNT_15        0x100518
#define O_TPM_ERR_INQ_SBE_ERR_INFO            0x100520
#define O_TPM_ERR_INQ_MBE_ERR_INFO            0x100528
#define O_RPM_INQ_PORT0_ERR_STS               0x182000
#define O_RPM_INQ_PORT0_ERR_CLR               0x182008
#define O_RPM_INQ_PORT0_ERR_FIRST_HOST        0x182020
#define O_RPM_INQ_PORT1_ERR_STS               0x182080
#define O_RPM_INQ_PORT1_ERR_CLR               0x182088
#define O_RPM_INQ_PORT1_ERR_FIRST_HOST        0x1820A0
#define O_RPM_INQ_PORT2_ERR_STS               0x182100
#define O_RPM_INQ_PORT2_ERR_CLR               0x182108
#define O_RPM_INQ_PORT2_ERR_FIRST_HOST        0x182120
#define O_RPM_INQ_PORT3_ERR_STS               0x182180
#define O_RPM_INQ_PORT3_ERR_CLR               0x182188
#define O_RPM_INQ_PORT3_ERR_FIRST_HOST        0x1821A0
#define O_RPM_INQ_PORT4_ERR_STS               0x182200
#define O_RPM_INQ_PORT4_ERR_CLR               0x182208
#define O_RPM_INQ_PORT4_ERR_FIRST_HOST        0x182220
#define O_RPM_INQ_PORT5_ERR_STS               0x182280
#define O_RPM_INQ_PORT5_ERR_CLR               0x182288
#define O_RPM_INQ_PORT5_ERR_FIRST_HOST        0x1822A0
#define O_RPM_INQ_PORT6_ERR_STS               0x182300
#define O_RPM_INQ_PORT6_ERR_CLR               0x182308
#define O_RPM_INQ_PORT6_ERR_FIRST_HOST        0x182320
#define O_RPM_INQ_PORT7_ERR_STS               0x182380
#define O_RPM_INQ_PORT7_ERR_CLR               0x182388
#define O_RPM_INQ_PORT7_ERR_FIRST_HOST        0x1823A0
#define O_RPM_INQ_FLSTOR_ERR_STS              0x182400
#define O_RPM_INQ_FLSTOR_ERR_CLR              0x182408
#define O_RPM_INQ_FLSTOR_ERR_FIRST_HOST       0x182420
#define O_BRG_RTP_ERR_STS                     0x201000
#define O_BRG_RTP_ERR_CLR                     0x201008
#define O_BRG_RTP_ERR_FIRST_HOST              0x201020
#define O_BRG_RTP_ERR_FIRST_INFO              0x201028

#endif /* IAF_CSR_H_INCLUDED */
