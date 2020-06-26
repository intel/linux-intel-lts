// SPDX-License-Identifier: GPL-2.0-only
/* Keembay DWMAC platform driver
 *
 * Copyright(c) 2019 Intel Corporation.
 *
 */

#include <linux/ethtool.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

#include "stmmac.h"
#include "stmmac_platform.h"

struct keembay_dwmac {
	struct clk *tx_clk;
};

static void keembay_dwmac_fix_mac_speed(void *priv, unsigned int speed)
{
	struct keembay_dwmac *dwmac = priv;
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
	}

	clk_set_rate(dwmac->tx_clk, rate);
}

static int keembay_dwmac_probe(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct keembay_dwmac *dwmac;
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
	plat_dat->tx_clk = devm_clk_get(&pdev->dev, "tx_clk");
	if (IS_ERR(plat_dat->tx_clk)) {
		if (PTR_ERR(plat_dat->tx_clk) == -EPROBE_DEFER)
			goto error_tx_clk_get;

		plat_dat->tx_clk = NULL;
	}

	clk_prepare_enable(plat_dat->tx_clk);
	dwmac->tx_clk = plat_dat->tx_clk;

	plat_dat->bsp_priv = dwmac;
	plat_dat->fix_mac_speed = keembay_dwmac_fix_mac_speed;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_remove_config_dt;

	return 0;

error_tx_clk_get:
	clk_disable_unprepare(plat_dat->pclk);
	clk_disable_unprepare(plat_dat->stmmac_clk);

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id keembay_dwmac_match[] = {
	{ .compatible = "snps,dwmac-4.10a" },
	{ .compatible = "snps,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, keembay_dwmac_match);

static struct platform_driver keembay_dwmac_driver = {
	.probe  = keembay_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name		= "keembay-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = keembay_dwmac_match,
	},
};
module_platform_driver(keembay_dwmac_driver);

MODULE_DESCRIPTION("Keembay DWMAC Ethernet driver");
MODULE_LICENSE("GPL v2");

