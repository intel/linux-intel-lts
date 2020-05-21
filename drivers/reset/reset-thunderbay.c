// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Thunder Bay Harbor Reset driver.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/reset/reset-thunderbay.h>

#define get_thunderbay_reset_data(x)  container_of(x, struct thunderbay_rst, rcdev)

struct thunderbay_rst {
	struct reset_controller_dev rcdev;
	struct device *dev;
	void __iomem *reg_base;
	const struct thunderbay_rst_data *rst_data;
};

/**
 * thunderbay_control_assert() - assert device reset
 * @rcdev: reset controller entity
 * @id: Device reset ID to be asserted
 *
 * This function keeps a device under reset.
 *
 * Return: 0 for always.
 */
static int thunderbay_control_assert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct thunderbay_rst *data = get_thunderbay_reset_data(rcdev);
	const struct thunderbay_rst_map *mmap = &data->rst_data->resets[id];

	writel(1 << mmap->bit, data->reg_base + mmap->reg + THUNDERBAY_RESET_CLEAR_OFFSET);

	return 0;
}

/**
 * thunderbay_control_deassert() - deassert device reset
 * @rcdev: reset controller entity
 * @id: Device reset ID to be deasserted
 *
 * This function releases a device from reset.
 *
 * Return: 0 for always.
 */
static int thunderbay_control_deassert(struct reset_controller_dev *rcdev,
				       unsigned long id)
{
	struct thunderbay_rst *data = get_thunderbay_reset_data(rcdev);
	const struct thunderbay_rst_map *mmap = &data->rst_data->resets[id];

	writel(1 << mmap->bit, data->reg_base + mmap->reg + THUNDERBAY_RESET_SET_OFFSET);

	return 0;
}

static const struct reset_control_ops thunderbay_rst_ops = {
	.assert = thunderbay_control_assert,
	.deassert = thunderbay_control_deassert,
};

static int thunderbay_rst_probe(struct platform_device *pdev)
{
	const struct thunderbay_rst_data *data;
	struct device *dev = &pdev->dev;
	struct thunderbay_rst *rst;
	int ret;

	rst = devm_kzalloc(dev, sizeof(*rst), GFP_KERNEL);
	if (!rst)
		return -ENOMEM;

	rst->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rst->reg_base)) {
		dev_err(dev, "Failed to remap resource.\n");
		return PTR_ERR(rst->reg_base);
	}

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "Failed to get data.\n");
		return -EINVAL;
	}

	rst->rcdev.owner = THIS_MODULE;
	rst->rcdev.ops = &thunderbay_rst_ops;
	rst->rcdev.of_node = dev->of_node;
	rst->rcdev.nr_resets = data->n_resets;
	rst->dev = dev;
	rst->rst_data = data;

	ret = devm_reset_controller_register(dev, &rst->rcdev);
	if (ret) {
		dev_err(dev, "Failed to register reset device.\n");
		return ret;
	}

	dev_info(dev, "Reset Driver registered successfully.\n");

//Release reset to VPU.
	if ((!strncmp(rst->rcdev.of_node->full_name, "reset-controller@185520000", sizeof("reset-controller@185520000"))) ||
	    (!strncmp(rst->rcdev.of_node->full_name, "reset-controller@285520000", sizeof("reset-controller@285520000"))) ||
	    (!strncmp(rst->rcdev.of_node->full_name, "reset-controller@385520000", sizeof("reset-controller@385520000"))) ||
	    (!strncmp(rst->rcdev.of_node->full_name, "reset-controller@485520000", sizeof("reset-controller@485520000"))))
		writel(1, rst->reg_base + THUNDERBAY_RESET_SET_OFFSET);

	return 0;
}

static const struct of_device_id thunderbay_rst_match[] = {
	{
		.compatible = "intel,thunderbay-cpuss-rst",
		.data = &thunderbay_cpuss_reset_data,
	},
	{
		.compatible = "intel,thunderbay-pss-rst1",
		.data = &thunderbay_pss_reset_data1,
	},
	{
		.compatible = "intel,thunderbay-pss-rst2",
		.data = &thunderbay_pss_reset_data2,
	},
	{
		.compatible = "intel,thunderbay-pcie-rst",
		.data = &thunderbay_pcie_reset_data,
	},
	{
		.compatible = "intel,thunderbay-comss-rst",
		.data = &thunderbay_comss_reset_data,
	},
	{
		.compatible = "intel,thunderbay-memss-rst",
		.data = &thunderbay_memss_reset_data,
	},
	{ },
};

MODULE_DEVICE_TABLE(of, thunderbay_rst_match);

static struct platform_driver thunderbay_reset_driver = {
	.probe	= thunderbay_rst_probe,
	.driver	= {
		.name		= "thunderbay_reset",
		.of_match_table = thunderbay_rst_match,
	},
};

module_platform_driver(thunderbay_reset_driver);

MODULE_DESCRIPTION("Intel Thunder Bay Reset Driver");
MODULE_AUTHOR("Shruthi Sanil <shruthi.sanil@intel.com>");
MODULE_LICENSE("GPL v2");
