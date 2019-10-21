// SPDX-License-Identifier: GPL-2.0
/* Intel DWMAC platform driver
 *
 * Copyright(C) 2020 Intel Corporation
 */

#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

#include "stmmac.h"
#include "stmmac_platform.h"

struct intel_dwmac {
	struct clk *tx_clk;
};

static void kmb_eth_fix_mac_speed(void *priv, unsigned int speed)
{
	struct intel_dwmac *dwmac = priv;
	unsigned long rate;

	rate = clk_get_rate(dwmac->tx_clk);

	switch (speed) {
	case SPEED_1000:
		rate = 125000000;
		break;

	case SPEED_100:
		rate = 25000000;
		break;

	case SPEED_10:
		rate = 2500000;
		break;

	default:
		break;
	}

	clk_set_rate(dwmac->tx_clk, rate);
}

static int intel_eth_plat_probe(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct device_node *np = pdev->dev.of_node;
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct intel_dwmac *dwmac;
	u32 clk_rate[3];
	int ret;

	plat_dat = priv->plat;
	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		dev_err(&pdev->dev, "dt configuration failed\n");
		return PTR_ERR(plat_dat);
	}

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	dev_info(&pdev->dev, "Setting Tx clock");
	dwmac->tx_clk = devm_clk_get(&pdev->dev, "tx_clk");
	if (IS_ERR(dwmac->tx_clk)) {
		if (PTR_ERR(dwmac->tx_clk) == -EPROBE_DEFER)
			goto err_remove_config_dt;

		dwmac->tx_clk = NULL;
	}

	clk_prepare_enable(dwmac->tx_clk);

	if (!of_property_read_u32_array(np, "clock-frequency", clk_rate, 3)) {
		ret = clk_set_rate(plat_dat->clk_ptp_ref, clk_rate[1]);
		if (ret) {
			dev_err(&pdev->dev, "Unable to set clk_ptp_ref rate\n");
			return ret;
		}

		ret = clk_set_rate(dwmac->tx_clk, clk_rate[2]);
		if (ret) {
			dev_err(&pdev->dev, "Unable to set tx_clk rate\n");
			return ret;
		}
	}

	plat_dat->bsp_priv = dwmac;
	plat_dat->fix_mac_speed = kmb_eth_fix_mac_speed;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret) {
		clk_disable_unprepare(dwmac->tx_clk);
		goto err_remove_config_dt;
	}

	return 0;

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static int intel_eth_plat_remove(struct platform_device *pdev)
{
	struct intel_dwmac *dwmac = get_stmmac_bsp_priv(&pdev->dev);
	int ret;

	ret = stmmac_pltfr_remove(pdev);
	clk_disable_unprepare(dwmac->tx_clk);

	return ret;
}

static const struct of_device_id intel_eth_plat_match[] = {
	{ .compatible = "snps,dwmac-4.10a" },
	{ .compatible = "snps,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, intel_eth_plat_match);

static struct platform_driver intel_eth_plat_driver = {
	.probe  = intel_eth_plat_probe,
	.remove = intel_eth_plat_remove,
	.driver = {
		.name		= "intel-eth-plat",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = intel_eth_plat_match,
	},
};
module_platform_driver(intel_eth_plat_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel DWMAC platform driver");
