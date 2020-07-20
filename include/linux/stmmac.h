/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************

  Header file for stmmac platform data

  Copyright (C) 2009  STMicroelectronics Ltd


  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#ifndef __STMMAC_PLATFORM_DATA
#define __STMMAC_PLATFORM_DATA

#include <linux/phy.h>
#include <linux/platform_device.h>

#define MTL_MAX_RX_QUEUES	8
#define MTL_MAX_TX_QUEUES	8
#define STMMAC_CH_MAX		8

#define STMMAC_RX_COE_NONE	0
#define STMMAC_RX_COE_TYPE1	1
#define STMMAC_RX_COE_TYPE2	2

/* Define the macros for CSR clock range parameters to be passed by
 * platform code.
 * This could also be configured at run time using CPU freq framework. */

/* MDC Clock Selection define*/
#define	STMMAC_CSR_60_100M	0x0	/* MDC = clk_scr_i/42 */
#define	STMMAC_CSR_100_150M	0x1	/* MDC = clk_scr_i/62 */
#define	STMMAC_CSR_20_35M	0x2	/* MDC = clk_scr_i/16 */
#define	STMMAC_CSR_35_60M	0x3	/* MDC = clk_scr_i/26 */
#define	STMMAC_CSR_150_250M	0x4	/* MDC = clk_scr_i/102 */
#define	STMMAC_CSR_250_300M	0x5	/* MDC = clk_scr_i/122 */

/* MTL algorithms identifiers */
#define MTL_TX_ALGORITHM_WRR	0x0
#define MTL_TX_ALGORITHM_WFQ	0x1
#define MTL_TX_ALGORITHM_DWRR	0x2
#define MTL_TX_ALGORITHM_SP	0x3
#define MTL_RX_ALGORITHM_SP	0x4
#define MTL_RX_ALGORITHM_WSP	0x5

/* RX/TX Queue Mode */
#define MTL_QUEUE_AVB		0x0
#define MTL_QUEUE_DCB		0x1

/* The MDC clock could be set higher than the IEEE 802.3
 * specified frequency limit 0f 2.5 MHz, by programming a clock divider
 * of value different than the above defined values. The resultant MDIO
 * clock frequency of 12.5 MHz is applicable for the interfacing chips
 * supporting higher MDC clocks.
 * The MDC clock selection macros need to be defined for MDC clock rate
 * of 12.5 MHz, corresponding to the following selection.
 */
#define STMMAC_CSR_I_4		0x8	/* clk_csr_i/4 */
#define STMMAC_CSR_I_6		0x9	/* clk_csr_i/6 */
#define STMMAC_CSR_I_8		0xA	/* clk_csr_i/8 */
#define STMMAC_CSR_I_10		0xB	/* clk_csr_i/10 */
#define STMMAC_CSR_I_12		0xC	/* clk_csr_i/12 */
#define STMMAC_CSR_I_14		0xD	/* clk_csr_i/14 */
#define STMMAC_CSR_I_16		0xE	/* clk_csr_i/16 */
#define STMMAC_CSR_I_18		0xF	/* clk_csr_i/18 */

/* AXI DMA Burst length supported */
#define DMA_AXI_BLEN_4		(1 << 1)
#define DMA_AXI_BLEN_8		(1 << 2)
#define DMA_AXI_BLEN_16		(1 << 3)
#define DMA_AXI_BLEN_32		(1 << 4)
#define DMA_AXI_BLEN_64		(1 << 5)
#define DMA_AXI_BLEN_128	(1 << 6)
#define DMA_AXI_BLEN_256	(1 << 7)
#define DMA_AXI_BLEN_ALL (DMA_AXI_BLEN_4 | DMA_AXI_BLEN_8 | DMA_AXI_BLEN_16 \
			| DMA_AXI_BLEN_32 | DMA_AXI_BLEN_64 \
			| DMA_AXI_BLEN_128 | DMA_AXI_BLEN_256)

/* Platfrom data for platform device structure's platform_data field */

struct stmmac_mdio_bus_data {
	unsigned int phy_mask;
	int *irqs;
	int probed_phy_irq;
	bool needs_reset;
};

struct stmmac_dma_cfg {
	int pbl;
	int txpbl;
	int rxpbl;
	bool pblx8;
	int fixed_burst;
	int mixed_burst;
	bool aal;
	bool multi_msi_en;
	bool tgl_wa;
};

#define AXI_BLEN	7
struct stmmac_axi {
	bool axi_lpi_en;
	bool axi_xit_frm;
	u32 axi_wr_osr_lmt;
	u32 axi_rd_osr_lmt;
	bool axi_kbbe;
	u32 axi_blen[AXI_BLEN];
	bool axi_fb;
	bool axi_mb;
	bool axi_rb;
};

struct stmmac_rxq_cfg {
	u8 mode_to_use;
	u32 chan;
	u8 pkt_route;
	bool use_prio;
	u32 prio;
};

struct stmmac_txq_cfg {
	u32 weight;
	u8 mode_to_use;
	/* Credit Base Shaper parameters */
	u32 send_slope;
	u32 idle_slope;
	u32 high_credit;
	u32 low_credit;
	bool use_prio;
	u32 prio;
	int tbs_en;
};

struct plat_stmmacenet_data {
	int bus_id;
	int phy_addr;
	int intel_adhoc_addr;
	int interface;
	int phy_interface;
	struct pci_dev *pdev;
	struct stmmac_mdio_bus_data *mdio_bus_data;
	struct device_node *phy_node;
	struct device_node *phylink_node;
	struct device_node *mdio_node;
	struct stmmac_dma_cfg *dma_cfg;
	int clk_csr;
	int has_gmac;
	int clk_trail_n;
	int enh_desc;
	int tx_coe;
	int rx_coe;
	int bugged_jumbo;
	int pmt;
	int force_sf_dma_mode;
	int force_thresh_dma_mode;
	int riwt_off;
	int max_speed;
	int maxmtu;
	int multicast_filter_bins;
	int unicast_filter_entries;
	int tx_fifo_size;
	int rx_fifo_size;
	u32 rx_queues_to_use;
	u32 tx_queues_to_use;
	u32 num_queue_pairs;
	u32 normal_tx_queue_count;
	u32 max_combined;
	u8 rx_sched_algorithm;
	u8 tx_sched_algorithm;
	struct stmmac_rxq_cfg rx_queues_cfg[MTL_MAX_RX_QUEUES];
	struct stmmac_txq_cfg tx_queues_cfg[MTL_MAX_TX_QUEUES];
	void (*fix_mac_speed)(void *priv, unsigned int speed);
	int (*init)(struct platform_device *pdev, void *priv);
	void (*exit)(struct platform_device *pdev, void *priv);
	struct mac_device_info *(*setup)(void *priv);
	int (*setup_phy_conv)(struct mii_bus *bus,
			      struct mdio_board_info *bi);
	int (*remove_phy_conv)(struct mii_bus *bus,
			       struct mdio_board_info *bi);
	void *bsp_priv;
	struct clk *stmmac_clk;
	struct clk *pclk;
	struct clk *clk_ptp_ref;
	unsigned int eee_usecs_rate;
	unsigned int clk_ptp_rate;
	unsigned int clk_ref_rate;
	s32 ptp_max_adj;
	struct reset_control *stmmac_rst;
	struct stmmac_axi *axi;
	int has_gmac4;
	int has_serdes;
	int has_tbs;
	bool has_sun8i;
	bool tso_en;
	int rss_en;
	bool tsn_est_en;
	bool tsn_fpe_en;
	bool tsn_tbs_en;
	int mac_port_sel_speed;
	bool en_tx_lpi_clockgating;
	int has_xgmac;
#ifdef CONFIG_STMMAC_NETWORK_PROXY
	int has_netproxy;
#endif
	bool multi_msi_en;
	int msi_mac_vec;
	int msi_wol_vec;
	int msi_lpi_vec;
	int msi_phy_conv_vec;
	int msi_sfty_ce_vec;
	int msi_sfty_ue_vec;
	int msi_rx_base_vec;
	int msi_tx_base_vec;
#ifdef CONFIG_STMMAC_NETWORK_PROXY
	int msi_network_proxy_vec;
#endif
	bool vlan_fail_q_en;
	u8 vlan_fail_q;
	bool speed_2500_en;
	u32 ptov;
	u32 ctov;
	u32 tils;
	/*FPE */
	u32 fprq;
	u32 afsz;
	u32 hadv;
	u32 radv;
	/* TBS */
	u32 estm;
	u32 leos;
	u32 legos;
	u32 ftos;
	u32 fgos;
	bool has_art;
	int pmc_art_to_pse_art_ratio;
	int int_snapshot_num;
	bool int_snapshot_en;
	int ext_snapshot_num;
	bool ext_snapshot_en;
	bool has_safety_feat;
	bool is_hfpga;
	bool is_pse;
	bool ehl_ao_wa;
	bool serdes_pse_sgmii_wa;
	u32 dma_bit_mask;
	/* TX and RX PHY latency (ns) */
	u64 phy_tx_latency_2500;
	u64 phy_tx_latency_1000;
	u64 phy_tx_latency_100;
	u64 phy_tx_latency_10;
	u64 phy_rx_latency_2500;
	u64 phy_rx_latency_1000;
	u64 phy_rx_latency_100;
	u64 phy_rx_latency_10;
	/* xPCS TX and RX latency (ns) */
	u64 xpcs_tx_latency_2500;
	u64 xpcs_tx_latency_1000;
	u64 xpcs_tx_latency_100;
	u64 xpcs_tx_latency_10;
	u64 xpcs_rx_latency_2500;
	u64 xpcs_rx_latency_1000;
	u64 xpcs_rx_latency_100;
	u64 xpcs_rx_latency_10;
	bool eee_timer;
	struct mdio_board_info *intel_bi;
	struct dwxpcs_platform_data *xpcs_pdata;
};
#endif
