// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This contains the functions to handle the pci driver.

  Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd


  Author: Rayagond Kokatanur <rayagond@vayavyalabs.com>
  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/clk-provider.h>
#include <linux/phy.h>
#include <linux/pci.h>
#include <linux/dmi.h>
#include <linux/dwxpcs.h>
#include <linux/pm_runtime.h>
#include "stmmac.h"
#include "dwmac4.h"
#include "stmmac_ptp.h"

#ifdef CONFIG_X86
#define CPU_STEPPING	boot_cpu_data.x86_stepping
#endif

/*
 * This struct is used to associate PCI Function of MAC controller on a board,
 * discovered via DMI, with the address of PHY connected to the MAC. The
 * negative value of the address means that MAC controller is not connected
 * with PHY.
 */
struct stmmac_pci_func_data {
	unsigned int func;
	int phy_addr;
};

struct stmmac_pci_dmi_data {
	const struct stmmac_pci_func_data *func;
	size_t nfuncs;
};

struct stmmac_pci_info {
	int (*setup)(struct pci_dev *pdev, struct plat_stmmacenet_data *plat);
};

static int stmmac_pci_find_phy_addr(struct pci_dev *pdev,
				    const struct dmi_system_id *dmi_list)
{
	const struct stmmac_pci_func_data *func_data;
	const struct stmmac_pci_dmi_data *dmi_data;
	const struct dmi_system_id *dmi_id;
	int func = PCI_FUNC(pdev->devfn);
	size_t n;

	dmi_id = dmi_first_match(dmi_list);
	if (!dmi_id)
		return -ENODEV;

	dmi_data = dmi_id->driver_data;
	func_data = dmi_data->func;

	for (n = 0; n < dmi_data->nfuncs; n++, func_data++)
		if (func_data->func == func)
			return func_data->phy_addr;

	return -ENODEV;
}

static void ehl_sgmii_path_latency_data(struct plat_stmmacenet_data *plat)
{
	/* SGMII TX and RX PHY latency (ns) */
	plat->phy_tx_latency_10 = 5385;
	plat->phy_tx_latency_100 = 666;
	plat->phy_tx_latency_1000 = 219;
	plat->phy_rx_latency_10 = 5902;
	plat->phy_rx_latency_100 = 821;
	plat->phy_rx_latency_1000 = 343;

	/* xPCS TX and RX latency (ns) */
	plat->xpcs_tx_latency_10 = 856;
	plat->xpcs_tx_latency_100 = 136;
	plat->xpcs_tx_latency_1000 = 56;
	plat->xpcs_rx_latency_10 = 7084;
	plat->xpcs_rx_latency_100 = 784;
	plat->xpcs_rx_latency_1000 = 160;
}

static void ehl_rgmii_path_latency_data(struct plat_stmmacenet_data *plat)
{
	/* RGMII TX and RX PHY latency (ns) */
	plat->phy_tx_latency_10 = 6066;
	plat->phy_tx_latency_100 = 656;
	plat->phy_tx_latency_1000 = 224;
	plat->phy_rx_latency_10 = 2130;
	plat->phy_rx_latency_100 = 362;
	plat->phy_rx_latency_1000 = 231;
}

static void common_default_data(struct plat_stmmacenet_data *plat)
{
	plat->clk_csr = 2;	/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */
	plat->has_gmac = 1;
	plat->force_sf_dma_mode = 1;

	plat->mdio_bus_data->needs_reset = true;
	plat->mdio_bus_data->phy_mask = 0;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = JUMBO_LEN;

	/* Set default number of RX and TX queues to use */
	plat->tx_queues_to_use = 1;
	plat->rx_queues_to_use = 1;

	/* Disable Priority config by default */
	plat->tx_queues_cfg[0].use_prio = false;
	plat->rx_queues_cfg[0].use_prio = false;

	/* Disable RX queues routing by default */
	plat->rx_queues_cfg[0].pkt_route = 0x0;
}

static int stmmac_default_data(struct pci_dev *pdev,
			       struct plat_stmmacenet_data *plat)
{
	/* Set common default data first */
	common_default_data(plat);

	plat->bus_id = 1;
	plat->phy_addr = 0;
	plat->phy_interface = PHY_INTERFACE_MODE_GMII;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	/* TODO: AXI */

	return 0;
}

static const struct stmmac_pci_info stmmac_pci_info = {
	.setup = stmmac_default_data,
};

static struct dwxpcs_platform_data intel_mgbe_pdata = {
	.mode = DWXPCS_MODE_SGMII_AN,
};

static struct mdio_board_info intel_mgbe_bdinfo = {
	.bus_id = "stmmac-1",
	.modalias = "dwxpcs",
	.mdio_addr = 0x16,
	.platform_data = &intel_mgbe_pdata,
};

static int setup_intel_mgbe_phy_conv(struct mii_bus *bus, int irq,
				     int phy_addr, bool speed_2500_en)
{
	struct dwxpcs_platform_data *pdata = &intel_mgbe_pdata;

	pdata->irq = irq;
	pdata->ext_phy_addr = phy_addr;
	pdata->speed_2500_en = speed_2500_en;

	return mdiobus_create_device(bus, &intel_mgbe_bdinfo);
}

static int remove_intel_mgbe_phy_conv(struct mii_bus *bus)
{
	struct mdio_board_info *bdinfo = &intel_mgbe_bdinfo;
	struct mdio_device *mdiodev;

	mdiodev = mdiobus_get_mdio_device(bus, bdinfo->mdio_addr);

	if (!mdiodev)
		return -1;

	mdio_device_remove(mdiodev);
	mdio_device_free(mdiodev);

	return 0;
}

static int intel_mgbe_common_data(struct pci_dev *pdev,
				  struct plat_stmmacenet_data *plat)
{
	int i;

	plat->pdev = pdev;
	plat->bus_id = pci_dev_id(pdev);
	plat->phy_addr = -1;

	plat->clk_csr = 5;
	plat->clk_trail_n = 2;
	plat->has_gmac = 0;
	plat->has_gmac4 = 1;
	plat->has_tbs = 1;
	plat->force_sf_dma_mode = 0;
	plat->tso_en = 1;
	plat->tsn_est_en = 1;
	plat->tsn_fpe_en = 1;
	plat->tsn_tbs_en = 1;
	/* FPE HW Tunable */
	plat->fprq = 1;
	plat->afsz = 0;  /* Adjustable Fragment Size */
	plat->hadv = 0;  /* Hold Advance */
	plat->radv = 0;  /* Release Advance*/
	/* TBS HW Tunable */
	plat->estm = 0;  /* Absolute Mode */
	plat->leos = 0;  /* Launch Expiry Offset */
	plat->legos = 0; /* Launch Expiry GSN Offset */
	plat->ftos = 0;  /* Fetch Time Offset */
	plat->fgos = 0;  /* Fetch GSN Offset */

	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	for (i = 0; i < plat->rx_queues_to_use; i++) {
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;
		plat->rx_queues_cfg[i].chan = i;

		/* Enable Priority config by default */
		plat->rx_queues_cfg[i].use_prio = true;

		/* Disable RX queues routing by default */
		plat->rx_queues_cfg[i].pkt_route = 0x0;
	}

	for (i = 0; i < plat->tx_queues_to_use; i++) {
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

		/* Disable Priority config by default */
		plat->tx_queues_cfg[i].use_prio = false;

		/* Enable per queue TBS support on half of the Tx Queues.
		 * For examples, if tx_queue_to_use = 8, then Tx Queue 4, 5, 6,
		 * and 7 will have TBS support.
		 */
		if (plat->tsn_tbs_en && i >= (plat->tx_queues_to_use / 2))
			plat->tx_queues_cfg[i].tbs_en = 1;
	}

	/* FIFO size is 4096 bytes for 1 tx/rx queue */
	plat->tx_fifo_size = plat->tx_queues_to_use * 4096;
	plat->rx_fifo_size = plat->rx_queues_to_use * 4096;

	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;
	plat->tx_queues_cfg[0].weight = 0x09;
	plat->tx_queues_cfg[1].weight = 0x0A;
	plat->tx_queues_cfg[2].weight = 0x0B;
	plat->tx_queues_cfg[3].weight = 0x0C;
	plat->tx_queues_cfg[4].weight = 0x0D;
	plat->tx_queues_cfg[5].weight = 0x0E;
	plat->tx_queues_cfg[6].weight = 0x0F;
	plat->tx_queues_cfg[7].weight = 0x10;

	plat->mdio_bus_data->phy_mask = 0;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	plat->dma_cfg->fixed_burst = 0;
	plat->dma_cfg->mixed_burst = 0;
	plat->dma_cfg->aal = 0;

	plat->axi = devm_kzalloc(&pdev->dev, sizeof(*plat->axi),
				 GFP_KERNEL);
	if (!plat->axi)
		return -ENOMEM;

	plat->axi->axi_lpi_en = 0;
	plat->axi->axi_xit_frm = 0;
	plat->axi->axi_wr_osr_lmt = 1;
	plat->axi->axi_rd_osr_lmt = 1;
	plat->axi->axi_blen[0] = 4;
	plat->axi->axi_blen[1] = 8;
	plat->axi->axi_blen[2] = 16;

	plat->ptp_max_adj = plat->clk_ptp_rate;
	plat->eee_usecs_rate = plat->clk_ptp_rate;
	/* Set system clock */
	plat->stmmac_clk = clk_register_fixed_rate(&pdev->dev,
						   pci_name(pdev), NULL, 0,
						   plat->clk_ptp_rate);

	if (IS_ERR(plat->stmmac_clk)) {
		dev_warn(&pdev->dev, "Fail to register stmmac-clk\n");
		plat->stmmac_clk = NULL;
	}
	clk_prepare_enable(plat->stmmac_clk);

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = JUMBO_LEN;

	if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		plat->setup_phy_conv = setup_intel_mgbe_phy_conv;
		plat->remove_phy_conv = remove_intel_mgbe_phy_conv;
		plat->has_serdes = 1;
	}

	/* intel specific adhoc (mdio) address for serdes & etc */
	plat->intel_adhoc_addr = 0x15;

	/* Setup MSI vector offset specific to Intel mGbE controller */
	plat->msi_phy_conv_vec = 30;
	plat->msi_mac_vec = 29;
	plat->msi_lpi_vec = 28;
	plat->msi_sfty_ce_vec = 27;
	plat->msi_sfty_ue_vec = 26;
	plat->msi_rx_base_vec = 0;
	plat->msi_tx_base_vec = 1;

	/* TSN HW tunable data */
	plat->ctov = 0;
	plat->ptov = 0;
	plat->tils = 0;

	plat->int_snapshot_num = AUX_SNAPSHOT1;
	plat->ext_snapshot_num = AUX_SNAPSHOT0;
	plat->int_snapshot_en = 0;
	plat->ext_snapshot_en = 0;

	return 0;
}

static int ehl_common_data(struct pci_dev *pdev,
			   struct plat_stmmacenet_data *plat)
{
	int ret;

	plat->rx_queues_to_use = 8;
	plat->tx_queues_to_use = 8;
	plat->has_safety_feat = 1;
	/* Maximum TX XDP queue */
	plat->max_combined = 4;

	ret = intel_mgbe_common_data(pdev, plat);
	if (ret)
		return ret;

	return 0;
}

static int ehl_sgmii_data(struct pci_dev *pdev,
			  struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	ehl_sgmii_path_latency_data(plat);

	/* Set PTP clock rate for EHL as 200MHz */
	plat->clk_ptp_rate = 204860000;

	return ehl_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_sgmii1g_pci_info = {
	.setup = ehl_sgmii_data,
};

static int ehl_rgmii_data(struct pci_dev *pdev,
			  struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_RGMII;
	ehl_rgmii_path_latency_data(plat);

	/* Set PTP clock rate for EHL as 200MHz */
	plat->clk_ptp_rate = 200000000;

	return ehl_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_rgmii1g_pci_info = {
	.setup = ehl_rgmii_data,
};

#define EHL_PSE_ETH_DMA_MISC_OFFSET	0x10000
#define EHL_PSE_ETH_DMA_MISC_DTM_DRAM	3
#define EHL_PSE_ETH_DMA_TOTAL_CH	16
static void ehl_pse_work_around(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	void __iomem *tempaddr = pcim_iomap_table(pdev)[0];
	int i;
	u32 val;

	for (i = 0; i < EHL_PSE_ETH_DMA_TOTAL_CH; i++) {
		val = readl(tempaddr + EHL_PSE_ETH_DMA_MISC_OFFSET
			    + i * sizeof(u32));
		val |= EHL_PSE_ETH_DMA_MISC_DTM_DRAM;
		writel(val, tempaddr + EHL_PSE_ETH_DMA_MISC_OFFSET
		       + i * sizeof(u32));
	}
	plat->is_hfpga = 0;
	plat->ehl_ao_wa = 1;

	if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII)
		plat->serdes_pse_sgmii_wa = 1;
}

static int ehl_pse0_common_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
#ifdef CONFIG_X86
	if (boot_cpu_has(X86_FEATURE_ART)) {
		unsigned int unused[3], ecx_pmc_art_freq;
		/* Elkhart Lake PSE ART is 19.2MHz */
		int pse_art_freq = 19200000;

		cpuid(0x15, unused, unused + 1, &ecx_pmc_art_freq, unused + 2);
		plat->pmc_art_to_pse_art_ratio = ecx_pmc_art_freq /
						 pse_art_freq;
	}

	/* WA needed for EHL A0 */
	if (CPU_STEPPING == 0)
		ehl_pse_work_around(pdev, plat);
#endif

	plat->is_pse = 1;
	plat->dma_bit_mask = 32;

	if (plat->is_hfpga)
		plat->clk_ptp_rate = 20000000;
	else
		plat->clk_ptp_rate = 200000000;

	/* store A2H packets in L2 SRAM, access through BAR0 + 128KB */
#ifdef CONFIG_STMMAC_NETWORK_PROXY
#if (CONFIG_STMMAC_NETWORK_PROXY_PORT == 0)
	plat->has_netproxy = 1;
	plat->msi_network_proxy_vec = 24;
#endif /* CONFIG_STMMAC_NETWORK_PROXY_PORT */
#endif /* CONFIG_STMMAC_NETWORK_PROXY */

	return ehl_common_data(pdev, plat);
}

static int ehl_pse0_rgmii1g_data(struct pci_dev *pdev,
				 struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_RGMII_ID;
	ehl_rgmii_path_latency_data(plat);

	return ehl_pse0_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_pse0_rgmii1g_pci_info = {
	.setup = ehl_pse0_rgmii1g_data,
};

static int ehl_pse0_sgmii1g_data(struct pci_dev *pdev,
				 struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	ehl_sgmii_path_latency_data(plat);

	return ehl_pse0_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_pse0_sgmii1g_pci_info = {
	.setup = ehl_pse0_sgmii1g_data,
};

static int ehl_pse1_common_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
#ifdef CONFIG_X86
	if (boot_cpu_has(X86_FEATURE_ART)) {
		unsigned int unused[3], ecx_pmc_art_freq;
		/* Elkhart Lake PSE ART is 19.2MHz */
		int pse_art_freq = 19200000;

		cpuid(0x15, unused, unused + 1, &ecx_pmc_art_freq, unused + 2);
		plat->pmc_art_to_pse_art_ratio = ecx_pmc_art_freq /
						 pse_art_freq;
	}

	/* WA needed for EHL A0 */
	if (CPU_STEPPING == 0)
		ehl_pse_work_around(pdev, plat);
#endif

	plat->is_pse = 1;
	plat->dma_bit_mask = 32;

	if (plat->is_hfpga)
		plat->clk_ptp_rate = 20000000;
	else
		plat->clk_ptp_rate = 200000000;

	/* store A2H packets in L2 SRAM, access through BAR0 + 128KB */
#ifdef CONFIG_STMMAC_NETWORK_PROXY
#if (CONFIG_STMMAC_NETWORK_PROXY_PORT == 1)
	plat->has_netproxy = 1;
	plat->msi_network_proxy_vec = 24;
#endif /* CONFIG_STMMAC_NETWORK_PROXY_PORT */
#endif /* CONFIG_STMMAC_NETWORK_PROXY */

	return ehl_common_data(pdev, plat);
}

static int ehl_pse1_rgmii1g_data(struct pci_dev *pdev,
				 struct plat_stmmacenet_data *plat)
{
	plat->pdev = pdev;
	plat->phy_interface = PHY_INTERFACE_MODE_RGMII_ID;
	ehl_rgmii_path_latency_data(plat);

	return ehl_pse1_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_pse1_rgmii1g_pci_info = {
	.setup = ehl_pse1_rgmii1g_data,
};

static int ehl_pse1_sgmii1g_data(struct pci_dev *pdev,
				 struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	ehl_sgmii_path_latency_data(plat);

	return ehl_pse1_common_data(pdev, plat);
}

static struct stmmac_pci_info ehl_pse1_sgmii1g_pci_info = {
	.setup = ehl_pse1_sgmii1g_data,
};

static int tgl_common_data(struct pci_dev *pdev,
			   struct plat_stmmacenet_data *plat)
{
	int ret;

	plat->rx_queues_to_use = 6;
	plat->tx_queues_to_use = 4;
	/* Maximum TX XDP queue */
	plat->max_combined = 2;

	/* TX and RX Marvell 88E2110 PHY latency (ns) */
	plat->phy_tx_latency_10 = 6652;
	plat->phy_tx_latency_100 = 1152;
	plat->phy_tx_latency_1000 = 297;
	plat->phy_tx_latency_2500 = 2772;
	plat->phy_rx_latency_10 = 12490;
	plat->phy_rx_latency_100 = 1472;
	plat->phy_rx_latency_1000 = 405;
	plat->phy_rx_latency_2500 = 2638;

	plat->clk_ptp_rate = 200000000;
	ret = intel_mgbe_common_data(pdev, plat);
	if (ret)
		return ret;

	return 0;
}

static int tgl_sgmii_phy0_data(struct pci_dev *pdev,
			       struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	return tgl_common_data(pdev, plat);
}

static struct stmmac_pci_info tgl_sgmii1g_phy0_pci_info = {
	.setup = tgl_sgmii_phy0_data,
};

static int tgl_sgmii_phy1_data(struct pci_dev *pdev,
			       struct plat_stmmacenet_data *plat)
{
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
	return tgl_common_data(pdev, plat);
}

static struct stmmac_pci_info tgl_sgmii1g_phy1_pci_info = {
	.setup = tgl_sgmii_phy1_data,
};

static int synp_haps_sgmii_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	int ret;

	plat->bus_id = 1;
	plat->phy_addr = 0;
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;

	plat->rx_queues_to_use = 6;
	plat->tx_queues_to_use = 4;
	/* Set PTP clock rate for HAPS as 62.5MHz */
	plat->clk_ptp_rate = 62500000;
	ret = intel_mgbe_common_data(pdev, plat);
	if (ret)
		return ret;

	/* Override: HAPS does not have xPCS   */
	plat->setup_phy_conv = NULL;
	plat->has_serdes = 0;
	plat->intel_adhoc_addr = 0;

	/* Override: HAPS does not support MSI */
	plat->msi_phy_conv_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_mac_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_lpi_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_sfty_ce_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_sfty_ue_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_rx_base_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_tx_base_vec = STMMAC_MSI_VEC_MAX;

	return 0;
}

static struct stmmac_pci_info synp_haps_pci_info = {
	.setup = synp_haps_sgmii_data,
};

static int icl_sgmii_data(struct pci_dev *pdev,
			  struct plat_stmmacenet_data *plat)
{
	int ret;

	plat->bus_id = 1;
	plat->phy_interface = PHY_INTERFACE_MODE_SGMII;

	plat->rx_queues_to_use = 4;
	plat->tx_queues_to_use = 4;
	/* Set PTP clock rate for ICL as 200MHz */
	plat->clk_ptp_rate = 200000000;
	ret = intel_mgbe_common_data(pdev, plat);
	if (ret)
		return ret;

	/* Override: ICL B0 SoC does not have TBS */
	plat->has_tbs = 0;

	return 0;
}

static struct stmmac_pci_info icl_pci_info = {
	.setup = icl_sgmii_data,
};

static const struct stmmac_pci_func_data galileo_stmmac_func_data[] = {
	{
		.func = 6,
		.phy_addr = 1,
	},
};

static const struct stmmac_pci_dmi_data galileo_stmmac_dmi_data = {
	.func = galileo_stmmac_func_data,
	.nfuncs = ARRAY_SIZE(galileo_stmmac_func_data),
};

static const struct stmmac_pci_func_data iot2040_stmmac_func_data[] = {
	{
		.func = 6,
		.phy_addr = 1,
	},
	{
		.func = 7,
		.phy_addr = 1,
	},
};

static const struct stmmac_pci_dmi_data iot2040_stmmac_dmi_data = {
	.func = iot2040_stmmac_func_data,
	.nfuncs = ARRAY_SIZE(iot2040_stmmac_func_data),
};

static const struct dmi_system_id quark_pci_dmi[] = {
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "Galileo"),
		},
		.driver_data = (void *)&galileo_stmmac_dmi_data,
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "GalileoGen2"),
		},
		.driver_data = (void *)&galileo_stmmac_dmi_data,
	},
	/*
	 * There are 2 types of SIMATIC IOT2000: IOT2020 and IOT2040.
	 * The asset tag "6ES7647-0AA00-0YA2" is only for IOT2020 which
	 * has only one pci network device while other asset tags are
	 * for IOT2040 which has two.
	 */
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "SIMATIC IOT2000"),
			DMI_EXACT_MATCH(DMI_BOARD_ASSET_TAG,
					"6ES7647-0AA00-0YA2"),
		},
		.driver_data = (void *)&galileo_stmmac_dmi_data,
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "SIMATIC IOT2000"),
		},
		.driver_data = (void *)&iot2040_stmmac_dmi_data,
	},
	{}
};

static int quark_default_data(struct pci_dev *pdev,
			      struct plat_stmmacenet_data *plat)
{
	int ret;

	/* Set common default data first */
	common_default_data(plat);

	/*
	 * Refuse to load the driver and register net device if MAC controller
	 * does not connect to any PHY interface.
	 */
	ret = stmmac_pci_find_phy_addr(pdev, quark_pci_dmi);
	if (ret < 0) {
		/* Return error to the caller on DMI enabled boards. */
		if (dmi_get_system_info(DMI_BOARD_NAME))
			return ret;

		/*
		 * Galileo boards with old firmware don't support DMI. We always
		 * use 1 here as PHY address, so at least the first found MAC
		 * controller would be probed.
		 */
		ret = 1;
	}

	plat->bus_id = pci_dev_id(pdev);
	plat->phy_addr = ret;
	plat->phy_interface = PHY_INTERFACE_MODE_RMII;

	plat->dma_cfg->pbl = 16;
	plat->dma_cfg->pblx8 = true;
	plat->dma_cfg->fixed_burst = 1;
	/* AXI (TODO) */

	return 0;
}

static const struct stmmac_pci_info quark_pci_info = {
	.setup = quark_default_data,
};

static int snps_gmac5_default_data(struct pci_dev *pdev,
				   struct plat_stmmacenet_data *plat)
{
	int i;

	plat->clk_csr = 5;
	plat->has_gmac4 = 1;
	plat->force_sf_dma_mode = 1;
	plat->tso_en = 1;
	plat->pmt = 1;

	plat->mdio_bus_data->phy_mask = 0;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = JUMBO_LEN;

	/* Set default number of RX and TX queues to use */
	plat->tx_queues_to_use = 4;
	plat->rx_queues_to_use = 4;

	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;
	for (i = 0; i < plat->tx_queues_to_use; i++) {
		plat->tx_queues_cfg[i].use_prio = false;
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;
		plat->tx_queues_cfg[i].weight = 25;
	}

	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;
	for (i = 0; i < plat->rx_queues_to_use; i++) {
		plat->rx_queues_cfg[i].use_prio = false;
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;
		plat->rx_queues_cfg[i].pkt_route = 0x0;
		plat->rx_queues_cfg[i].chan = i;
	}

	plat->bus_id = 1;
	plat->phy_addr = -1;
	plat->phy_interface = PHY_INTERFACE_MODE_GMII;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;

	/* Axi Configuration */
	plat->axi = devm_kzalloc(&pdev->dev, sizeof(*plat->axi), GFP_KERNEL);
	if (!plat->axi)
		return -ENOMEM;

	plat->axi->axi_wr_osr_lmt = 31;
	plat->axi->axi_rd_osr_lmt = 31;

	plat->axi->axi_fb = false;
	plat->axi->axi_blen[0] = 4;
	plat->axi->axi_blen[1] = 8;
	plat->axi->axi_blen[2] = 16;
	plat->axi->axi_blen[3] = 32;

	return 0;
}

static const struct stmmac_pci_info snps_gmac5_pci_info = {
	.setup = snps_gmac5_default_data,
};

static int stmmac_config_single_msi(struct pci_dev *pdev,
				    struct plat_stmmacenet_data *plat,
				    struct stmmac_resources *res)
{
	int ret;

	ret = pci_alloc_irq_vectors(pdev, 1, 1,
				    PCI_IRQ_LEGACY | PCI_IRQ_MSI);
	if (ret < 0) {
		dev_info(&pdev->dev, "%s: Single IRQ enablement failed\n",
			 __func__);
		return ret;
	}

	res->irq = pci_irq_vector(pdev, 0);
	res->wol_irq = res->irq;
	res->wol_irq = res->irq;
	res->phy_conv_irq = res->irq;
	plat->multi_msi_en = 0;
	dev_info(&pdev->dev, "%s: Single IRQ enablement successful\n",
		 __func__);

	return 0;
}

static int stmmac_config_multi_msi(struct pci_dev *pdev,
				   struct plat_stmmacenet_data *plat,
				   struct stmmac_resources *res)
{
	int ret;
	int i;

	ret = pci_alloc_irq_vectors(pdev, 1, STMMAC_MSI_VEC_MAX,
				    PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_info(&pdev->dev, "%s: multi MSI enablement failed\n",
			 __func__);
		return ret;
	}

	if (plat->msi_rx_base_vec >= STMMAC_MSI_VEC_MAX ||
	    plat->msi_tx_base_vec >= STMMAC_MSI_VEC_MAX) {
		dev_info(&pdev->dev, "%s: Invalid RX & TX vector defined\n",
			 __func__);
		return -1;
	}

	/* For RX MSI */
	for (i = 0; i < plat->rx_queues_to_use; i++) {
		res->rx_irq[i] = pci_irq_vector(pdev,
						plat->msi_rx_base_vec + i * 2);
	}

	/* For TX MSI */
	for (i = 0; i < plat->tx_queues_to_use; i++) {
		res->tx_irq[i] = pci_irq_vector(pdev,
						plat->msi_tx_base_vec + i * 2);
	}

	if (plat->msi_mac_vec < STMMAC_MSI_VEC_MAX)
		res->irq = pci_irq_vector(pdev, plat->msi_mac_vec);
	if (plat->msi_wol_vec < STMMAC_MSI_VEC_MAX)
		res->wol_irq = pci_irq_vector(pdev, plat->msi_wol_vec);
	if (plat->msi_lpi_vec < STMMAC_MSI_VEC_MAX)
		res->lpi_irq = pci_irq_vector(pdev, plat->msi_lpi_vec);
	if (plat->msi_phy_conv_vec < STMMAC_MSI_VEC_MAX)
		res->phy_conv_irq = pci_irq_vector(pdev,
						   plat->msi_phy_conv_vec);
	if (plat->msi_sfty_ce_vec < STMMAC_MSI_VEC_MAX)
		res->sfty_ce_irq = pci_irq_vector(pdev, plat->msi_sfty_ce_vec);
	if (plat->msi_sfty_ue_vec < STMMAC_MSI_VEC_MAX)
		res->sfty_ue_irq = pci_irq_vector(pdev, plat->msi_sfty_ue_vec);
#ifdef CONFIG_STMMAC_NETWORK_PROXY
	if (plat->msi_network_proxy_vec < STMMAC_MSI_VEC_MAX &&
	    plat->has_netproxy)
		res->netprox_irq =
			pci_irq_vector(pdev, plat->msi_network_proxy_vec);
#endif /* CONFIG_STMMAC_NETWORK_PROXY */

	plat->multi_msi_en = 1;
	dev_info(&pdev->dev, "%s: multi MSI enablement successful\n", __func__);

	return 0;
}

/**
 * stmmac_pci_probe
 *
 * @pdev: pci device pointer
 * @id: pointer to table of device id/id's.
 *
 * Description: This probing function gets called for all PCI devices which
 * match the ID table and are not "owned" by other driver yet. This function
 * gets passed a "struct pci_dev *" for each device whose entry in the ID table
 * matches the device. The probe functions returns zero when the driver choose
 * to take "ownership" of the device or an error code(-ve no) otherwise.
 */
static int stmmac_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct stmmac_pci_info *info = (struct stmmac_pci_info *)id->driver_data;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res;
	int i;
	int ret;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return -ENOMEM;

	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data)
		return -ENOMEM;

	plat->dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*plat->dma_cfg),
				     GFP_KERNEL);
	if (!plat->dma_cfg)
		return -ENOMEM;

	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: ERROR: failed to enable device\n",
			__func__);
		return ret;
	}

	/* Get the base address of device */
	for (i = 0; i <= PCI_STD_RESOURCE_END; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		ret = pcim_iomap_regions(pdev, BIT(i), pci_name(pdev));
		if (ret)
			return ret;
		break;
	}

	pci_set_master(pdev);

	/* Initialize all MSI vectors to invalid so that it can be set
	 * according to platform data settings below.
	 * Note: MSI vector takes value from 0 upto 31 (STMMAC_MSI_VEC_MAX)
	 */
	plat->msi_mac_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_wol_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_lpi_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_phy_conv_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_sfty_ce_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_sfty_ue_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_rx_base_vec = STMMAC_MSI_VEC_MAX;
	plat->msi_tx_base_vec = STMMAC_MSI_VEC_MAX;
#ifdef CONFIG_STMMAC_NETWORK_PROXY
	plat->msi_network_proxy_vec = STMMAC_MSI_VEC_MAX;
#endif /* CONFIG_STMMAC_NETWORK_PROXY */

	ret = info->setup(pdev, plat);
	if (ret)
		return ret;

	memset(&res, 0, sizeof(res));
	res.addr = pcim_iomap_table(pdev)[i];

	if (plat->eee_usecs_rate > 0) {
		u32 tx_lpi_usec;

		tx_lpi_usec = (plat->eee_usecs_rate / 1000000) - 1;
		writel(tx_lpi_usec, res.addr + GMAC_1US_TIC_COUNTER);
	}

	ret = stmmac_config_multi_msi(pdev, plat, &res);
	if (!ret)
		goto msi_done;

	ret = stmmac_config_single_msi(pdev, plat, &res);
	if (!ret) {
		dev_err(&pdev->dev, "%s: ERROR: failed to enable IRQ\n",
			__func__);
		return ret;
	}

msi_done:
	return stmmac_dvr_probe(&pdev->dev, plat, &res);
}

/**
 * stmmac_pci_remove
 *
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and releases the PCI resources.
 */
static void stmmac_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int i;

	stmmac_dvr_remove(&pdev->dev);

	if (priv->plat->stmmac_clk)
		clk_unregister_fixed_rate(priv->plat->stmmac_clk);

	for (i = 0; i <= PCI_STD_RESOURCE_END; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		pcim_iounmap_regions(pdev, BIT(i));
		break;
	}

	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
}

static void stmmac_pci_shutdown(struct pci_dev *pdev)
{
	pci_wake_from_d3(pdev, true);
	pci_set_power_state(pdev, PCI_D3hot);
}

static int __maybe_unused stmmac_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int ret;

	ret = stmmac_suspend(dev);
	if (ret)
		return ret;

	ret = pci_save_state(pdev);
	if (ret)
		return ret;

	pci_disable_device(pdev);
	pci_wake_from_d3(pdev, true);
	pci_set_power_state(pdev, PCI_D3hot);

	return 0;
}

static int __maybe_unused stmmac_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int ret;

	pci_restore_state(pdev);
	pci_set_power_state(pdev, PCI_D0);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	return stmmac_resume(dev);
}

static int __maybe_unused stmmac_pci_runtime_suspend(struct device *dev)
{
	struct ethtool_wolinfo wol;
	struct stmmac_priv *priv;
	struct net_device *ndev;
	struct pci_dev *pdev;
	int ret;

	pdev = to_pci_dev(dev);
	ndev = dev_get_drvdata(&pdev->dev);
	priv = netdev_priv(ndev);

	rtnl_lock();
	/* Save current WoL operation */
	phylink_ethtool_get_wol(priv->phylink, &wol);
	priv->saved_wolopts = wol.wolopts;
	/* Enable WoL to wake on PHY activity */
	wol.wolopts = WAKE_PHY;
	phylink_ethtool_set_wol(priv->phylink, &wol);
	rtnl_unlock();

	device_set_wakeup_enable(priv->device, 1);

	ret = stmmac_pci_suspend(dev);
	if (!ret)
		dev_info(dev, "%s: Device is runtime suspended.\n", __func__);

	return ret;
}

static int __maybe_unused stmmac_pci_runtime_resume(struct device *dev)
{
	struct ethtool_wolinfo wol;
	struct stmmac_priv *priv;
	struct net_device *ndev;
	struct pci_dev *pdev;
	int ret;

	pdev = to_pci_dev(dev);
	ndev = dev_get_drvdata(&pdev->dev);
	priv = netdev_priv(ndev);

	rtnl_lock();
	/* Restore saved WoL operation */
	wol.wolopts = priv->saved_wolopts;
	phylink_ethtool_set_wol(priv->phylink, &wol);
	priv->saved_wolopts = 0;
	rtnl_unlock();

	ret = stmmac_pci_resume(dev);
	if (!ret)
		dev_info(dev, "%s: Device is runtime resumed.\n", __func__);

	return ret;
}

#define STMMAC_RUNTIME_SUSPEND_DELAY	2500

static int __maybe_unused stmmac_pci_runtime_idle(struct device *dev)
{
	struct ethtool_wolinfo wol;
	struct stmmac_priv *priv;
	struct net_device *ndev;
	struct pci_dev *pdev;

	pdev = to_pci_dev(dev);
	ndev = dev_get_drvdata(&pdev->dev);
	priv = netdev_priv(ndev);

	/* Allow runtime suspend only if link is down */
	if (priv->phylink_up)
		return -EBUSY;

	/* Allow runtime suspend only if PHY support wake on PHY activity */
	rtnl_lock();
	phylink_ethtool_get_wol(priv->phylink, &wol);
	rtnl_unlock();
	if (!(wol.supported & WAKE_PHY))
		return -EBUSY;

	/* Schedule the execution of delayed runtime suspend */
	pm_schedule_suspend(dev, STMMAC_RUNTIME_SUSPEND_DELAY);

	/* Return non-zero value to prevent PM core from calling autosuspend */
	return -EBUSY;
}

static const struct dev_pm_ops stmmac_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stmmac_pci_suspend, stmmac_pci_resume)
	SET_RUNTIME_PM_OPS(stmmac_pci_runtime_suspend,
			   stmmac_pci_runtime_resume, stmmac_pci_runtime_idle)
};

/* synthetic ID, no official vendor */
#define PCI_VENDOR_ID_STMMAC 0x700

#define STMMAC_QUARK_ID  0x0937
#define STMMAC_DEVICE_ID 0x1108
#define STMMAC_EHL_RGMII1G_ID	0x4b30
#define STMMAC_EHL_SGMII1G_ID	0x4b31
#define STMMAC_EHL_SGMII2G5_ID	0x4b32
#define STMMAC_EHL_PSE0_RGMII1G_ID	0x4ba0
#define STMMAC_EHL_PSE0_SGMII1G_ID	0x4ba1
#define STMMAC_EHL_PSE0_SGMII2G5_ID	0x4ba2
#define STMMAC_EHL_PSE1_RGMII1G_ID	0x4bb0
#define STMMAC_EHL_PSE1_SGMII1G_ID	0x4bb1
#define STMMAC_EHL_PSE1_SGMII2G5_ID	0x4bb2
#define STMMAC_TGL_SGMII1G_ID	0xa0ac
#define STMMAC_TGLH_SGMII1G_0_ID 0x43ac
#define STMMAC_TGLH_SGMII1G_1_ID 0x43a2
#define STMMAC_GMAC5_ID		0x7102
#define DEVICE_ID_HAPS_6X	0x7101
#define STMMAC_ICP_LP_ID	0x34ac

#define STMMAC_DEVICE(vendor_id, dev_id, info)	{	\
	PCI_VDEVICE(vendor_id, dev_id),			\
	.driver_data = (kernel_ulong_t)&info		\
	}

static const struct pci_device_id stmmac_id_table[] = {
	STMMAC_DEVICE(STMMAC, STMMAC_DEVICE_ID, stmmac_pci_info),
	STMMAC_DEVICE(STMICRO, PCI_DEVICE_ID_STMICRO_MAC, stmmac_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_QUARK_ID, quark_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_RGMII1G_ID, ehl_rgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_SGMII1G_ID, ehl_sgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_SGMII2G5_ID, ehl_sgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_PSE0_RGMII1G_ID,
		      ehl_pse0_rgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_PSE0_SGMII1G_ID,
		      ehl_pse0_sgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_PSE0_SGMII2G5_ID,
		      ehl_pse0_sgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_PSE1_RGMII1G_ID,
		      ehl_pse1_rgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_PSE1_SGMII1G_ID,
		      ehl_pse1_sgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_EHL_PSE1_SGMII2G5_ID,
		      ehl_pse1_sgmii1g_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_TGL_SGMII1G_ID,
		      tgl_sgmii1g_phy0_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_TGLH_SGMII1G_0_ID,
		      tgl_sgmii1g_phy0_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_TGLH_SGMII1G_1_ID,
		      tgl_sgmii1g_phy1_pci_info),
	STMMAC_DEVICE(SYNOPSYS, STMMAC_GMAC5_ID, snps_gmac5_pci_info),
	STMMAC_DEVICE(SYNOPSYS, DEVICE_ID_HAPS_6X, synp_haps_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_ICP_LP_ID, icl_pci_info),
	{}
};

MODULE_DEVICE_TABLE(pci, stmmac_id_table);

static struct pci_driver stmmac_pci_driver = {
	.name = STMMAC_RESOURCE_NAME,
	.id_table = stmmac_id_table,
	.probe = stmmac_pci_probe,
	.remove = stmmac_pci_remove,
	.shutdown = stmmac_pci_shutdown,
	.driver         = {
		.pm     = &stmmac_pm_ops,
	},
};

module_pci_driver(stmmac_pci_driver);

MODULE_DESCRIPTION("STMMAC 10/100/1000 Ethernet PCI driver");
MODULE_AUTHOR("Rayagond Kokatanur <rayagond.kokatanur@vayavyalabs.com>");
MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_LICENSE("GPL");
