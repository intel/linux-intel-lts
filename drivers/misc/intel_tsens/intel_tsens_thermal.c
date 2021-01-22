// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * Intel tsens thermal Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include "intel_tsens_thermal.h"

struct intel_tsens_trip_info {
	enum thermal_trip_type trip_type;
	int temp;
};

struct intel_tsens {
	char name[20];
	u32 n_trips;
	u32 passive_delay;
	u32 polling_delay;
	u32 sensor_type;
	u64 addr;
	u64 size;
	u32 curr_temp;
	void __iomem *base_addr;
	struct intel_tsens_trip_info **trip_info;
	struct thermal_zone_device *tz;
	void *pdata;
	struct intel_tsens_plat_info plat_info;
};

struct intel_tsens_priv {
	int n_sens;
	bool global_clk_available;
	void __iomem *base_addr;
	struct clk *tsens_clk;
	u32 tsens_clk_rate;
	struct intel_tsens **intel_tsens;
	struct device *dev;
	struct platform_device *pdev;
	struct intel_tsens_plat_info plat_info;
};

static int intel_tsens_register_pdev(struct intel_tsens_plat_info *plat_info)
{
	struct intel_tsens_plat_data plat_data;
	struct platform_device_info pdevinfo;
	struct platform_device *dd;

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.name = plat_info->plat_name;
	pdevinfo.id = plat_info->id;
	plat_data.base_addr = plat_info->base_addr;
	plat_data.name = plat_info->plat_name;
	plat_data.get_temp = NULL;
	pdevinfo.data = &plat_data;
	pdevinfo.size_data = sizeof(plat_data);
	dd = platform_device_register_full(&pdevinfo);
	if (IS_ERR(dd))
		return -EINVAL;
	plat_info->pdev = dd;

	return 0;
}

static void intel_tsens_unregister_pdev(struct intel_tsens_priv *priv)
{
	int i;

	for (i = 0; i < priv->n_sens; i++) {
		if (priv->plat_info.pdev)
			platform_device_unregister(priv->plat_info.pdev);
	}
}

static int intel_tsens_add_pdev(struct intel_tsens_priv *priv)
{
	int i, ret;

	/*
	 * Register platform device for each sensor.
	 *
	 */
	if (priv->plat_info.plat_name) {
		priv->plat_info.base_addr = priv->base_addr;
		ret = intel_tsens_register_pdev(&priv->plat_info);
		if (ret) {
			dev_err(&priv->pdev->dev,
				"platform device register failed for %s\n",
				priv->plat_info.plat_name);
			return ret;
		}
	}
	for (i = 0; i < priv->n_sens; i++) {
		struct intel_tsens *tsens = priv->intel_tsens[i];

		if (!tsens->plat_info.plat_name)
			continue;
		tsens->plat_info.base_addr = tsens->base_addr;
		ret = intel_tsens_register_pdev(&tsens->plat_info);
		if (ret) {
			dev_err(&priv->pdev->dev,
				"platform device register failed for %s\n",
				tsens->name);
			return ret;
		}
	}

	return 0;
}

static int intel_tsens_thermal_get_temp(struct intel_tsens *tsens,
					int *temp)
{
	struct intel_tsens_priv *priv =
		(struct intel_tsens_priv *)tsens->pdata;
	struct intel_tsens_plat_data *plat_data;
	int type = tsens->sensor_type;
	struct platform_device *pdev;

	if (tsens->plat_info.plat_name) {
		pdev = tsens->plat_info.pdev;
		plat_data = pdev->dev.platform_data;

		if (!plat_data) {
			dev_err(&pdev->dev, "Platform data not found for %s\n",
				tsens->name);
			return -EINVAL;
		}
		if (!plat_data->get_temp) {
			*temp = 0;
			return -EINVAL;
		}
		if (plat_data->get_temp(pdev, type, temp))
			return -EINVAL;
		tsens->curr_temp = *temp;
		return 0;
	}
	if (priv->plat_info.plat_name) {
		pdev = priv->plat_info.pdev;
		plat_data = pdev->dev.platform_data;

		if (!plat_data) {
			dev_err(&pdev->dev, "Platform data not found for %s\n",
				tsens->name);
			return -EINVAL;
		}
		if (!plat_data->get_temp) {
			*temp = 0;
			return -EINVAL;
		}

		if (plat_data->get_temp(pdev, type, temp))
			return -EINVAL;
		tsens->curr_temp = *temp;
		return 0;
	}

	return -EINVAL;
}

static int intel_tsens_themal_of_get_temp(void *data, int *temp)
{
	struct intel_tsens *tsens = (struct intel_tsens *)data;

	return intel_tsens_thermal_get_temp(tsens, temp);
}

static struct thermal_zone_of_device_ops tsens_thermal_of_ops = {
	.get_temp = intel_tsens_themal_of_get_temp,
};

static int intel_tsens_get_temp(int type, int *temp, void *pdata)
{
	struct intel_tsens_priv *priv = (struct intel_tsens_priv *)pdata;

	if (!priv)
		return -EINVAL;
	if (type >= priv->n_sens) {
		dev_err(&priv->pdev->dev, "Invalid sensor type");
		return -EINVAL;
	}

	return intel_tsens_thermal_get_temp(priv->intel_tsens[type], temp);
}

struct intel_tsens_i2c_plat_data i2c_plat_data = {
	.get_temp	= intel_tsens_get_temp,
};

static void intel_tsens_remove_thermal_zones(struct intel_tsens_priv *priv)
{
	int i;

	for (i = 0; i < priv->n_sens; i++) {
		struct intel_tsens *tsens = priv->intel_tsens[i];

		if (tsens->tz) {
			thermal_zone_device_unregister(tsens->tz);
			tsens->tz = NULL;
		}
	}
}

static int intel_tsens_add_thermal_zones(struct intel_tsens_priv *priv)
{
	int i;

	for (i = 0; i < priv->n_sens; i++) {
		struct intel_tsens *tsens = priv->intel_tsens[i];

		tsens->tz =
		devm_thermal_zone_of_sensor_register(&priv->pdev->dev,
						     i,	priv->intel_tsens[i],
						     &tsens_thermal_of_ops);
		if (IS_ERR(tsens->tz)) {
			dev_err(&priv->pdev->dev,
				"failed to register thermal zone device %s\n",
				tsens->name);
			return PTR_ERR(tsens->tz);
		}
	}

	return 0;
}

static void intel_tsens_remove_clk(struct intel_tsens_priv *priv)
{
	struct platform_device *pdev = priv->pdev;

	clk_disable_unprepare(priv->tsens_clk);
	devm_clk_put(&pdev->dev, priv->tsens_clk);
}

static int intel_tsens_clk_config(struct intel_tsens_priv *priv)
{
	struct platform_device *pdev = priv->pdev;
	int ret;

	if (priv->global_clk_available) {
		priv->tsens_clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(priv->tsens_clk)) {
			ret = PTR_ERR(priv->tsens_clk);
			if (ret != -EPROBE_DEFER) {
				dev_err(&pdev->dev,
					"failed to get thermal clk: %d\n", ret);
			}
			return PTR_ERR(priv->tsens_clk);
		}
		ret = clk_set_rate(priv->tsens_clk, priv->tsens_clk_rate);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to set rate for thermal clk: %d\n",
				ret);
			devm_clk_put(&pdev->dev, priv->tsens_clk);
			return ret;
		}
		ret = clk_prepare_enable(priv->tsens_clk);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to enable thermal clk: %d\n",
				ret);
			devm_clk_put(&pdev->dev, priv->tsens_clk);
			return ret;
		}
	}

	return 0;
}

static int intel_tsens_config_sensors(struct device_node *s_node,
				      struct intel_tsens *tsens,
				      int sensor_type)
{
	struct intel_tsens_priv *priv = (struct intel_tsens_priv *)tsens->pdata;
	struct platform_device *pdev = priv->pdev;
	s32 trip_temp_count, trip_temp_type_c, i;

	of_property_read_string_index(s_node, "plat_name", 0,
				      &tsens->plat_info.plat_name);
	tsens->plat_info.id = 1 << sensor_type;
	tsens->sensor_type = sensor_type;
	if (of_property_read_u32(s_node, "passive_delay",
				 &tsens->passive_delay)) {
		dev_err(&pdev->dev,
			"passive_delay missing in dt for %s\n",
			tsens->name);
		return -EINVAL;
	}
	if (of_property_read_u32(s_node, "polling_delay",
				 &tsens->polling_delay)) {
		dev_err(&pdev->dev,
			"polling_delay missing in dt for %s\n",
			tsens->name);
		return -EINVAL;
	}
	trip_temp_count = of_property_count_u32_elems(s_node, "trip_temp");
	trip_temp_type_c = of_property_count_strings(s_node, "trip_type");
	if (trip_temp_count != trip_temp_type_c ||
	    trip_temp_count <= 0 || trip_temp_type_c <= 0) {
		dev_err(&pdev->dev,
			"trip temp config is missing in dt for %s\n",
			tsens->name);
		return -EINVAL;
	}

	tsens->trip_info =
		devm_kcalloc(&pdev->dev, trip_temp_count,
			     sizeof(struct intel_tsens_trip_info *),
			     GFP_KERNEL);
	if (!tsens->trip_info)
		return -ENOMEM;
	tsens->n_trips = trip_temp_count;
	for (i = 0; i < trip_temp_count; i++) {
		struct intel_tsens_trip_info *trip_info;
		const char *trip_name;

		trip_info = devm_kzalloc(&pdev->dev,
					 sizeof(struct intel_tsens_trip_info),
					 GFP_KERNEL);
		if (!trip_info)
			return -ENOMEM;

		of_property_read_u32_index(s_node, "trip_temp", i,
					   &trip_info->temp);
		of_property_read_string_index(s_node, "trip_type", i,
					      &trip_name);
		if (!strcmp(trip_name, "passive"))
			trip_info->trip_type = THERMAL_TRIP_PASSIVE;
		else if (!strcmp(trip_name, "critical"))
			trip_info->trip_type = THERMAL_TRIP_CRITICAL;
		else if (!strcmp(trip_name, "hot"))
			trip_info->trip_type = THERMAL_TRIP_HOT;
		else
			trip_info->trip_type = THERMAL_TRIP_ACTIVE;
		tsens->trip_info[i] = trip_info;
	}

	return 0;
}

static int intel_tsens_config_dt(struct intel_tsens_priv *priv)
{
	struct platform_device *pdev = priv->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *s_node = NULL, *node;
	struct resource *res;
	int i = 0, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base_addr = devm_ioremap_resource(&pdev->dev, res);
	node = of_parse_phandle(np, "soc-sensors", 0);
	if (!node)
		return -EINVAL;
	priv->n_sens = of_get_child_count(node);
	if (priv->n_sens == 0) {
		dev_err(&pdev->dev, "No sensors configured in dt\n");
		return -EINVAL;
	}
	priv->global_clk_available = of_property_read_bool(np, "clocks");
	if (priv->global_clk_available) {
		ret = of_property_read_u32(np, "clk-rate",
					   &priv->tsens_clk_rate);
		if (ret) {
			dev_err(&pdev->dev, "clk-rate not available in dt");
			return ret;
		}
	}
	of_property_read_string_index(np, "plat_name", 0,
				      &priv->plat_info.plat_name);
	priv->intel_tsens =
		devm_kcalloc(&pdev->dev, priv->n_sens,
			     sizeof(struct intel_tsens *),
			     GFP_KERNEL);
	if (!priv->intel_tsens)
		return -ENOMEM;
	for_each_child_of_node(node, s_node) {
		int r_count, size_count;
		struct intel_tsens *ts;

		ts = devm_kzalloc(&pdev->dev, sizeof(struct intel_tsens),
				  GFP_KERNEL);
		if (!ts) {
			of_node_put(s_node);
			return -ENOMEM;
		}
		strcpy(ts->name, s_node->name);
		if (!of_property_read_u32(s_node, "address-cells", &r_count) &&
		    !of_property_read_u32(s_node, "size-cells", &size_count)) {
			if (r_count > 1) {
				ret = of_property_read_u64_index(s_node, "reg",
								 0, &ts->addr);
			} else {
				u32 *addr = (u32 *)&ts->addr;

				ret = of_property_read_u32_index(s_node, "reg",
								 0, addr);
			}
			if (ret) {
				dev_err(&pdev->dev, "Invalid reg base address");
				of_node_put(s_node);
				return ret;
			}
			if (size_count > 1) {
				int index =
					(r_count > 1) ? (r_count / 2) :
					r_count;

				ret = of_property_read_u64_index(s_node, "reg",
								 index,
								 &ts->size);
			} else {
				u32 *size = (u32 *)&ts->size;

				ret = of_property_read_u32_index(s_node, "reg",
								 r_count, size);
			}
			if (ret) {
				dev_err(&pdev->dev, "Invalid size");
				of_node_put(s_node);
				return ret;
			}
			ts->base_addr = devm_ioremap(&pdev->dev,
						     ts->addr,
						     ts->size);
		} else {
			ts->base_addr = priv->base_addr;
		}
		if (!ts->base_addr) {
			dev_err(&pdev->dev, "ioremap failed for %s\n",
				ts->name);
			of_node_put(s_node);
			return -EINVAL;
		}
		ts->pdata = priv;
		if (intel_tsens_config_sensors(s_node, ts, i)) {
			dev_err(&pdev->dev,
				"Missing sensor info in dts for %s\n",
				ts->name);
			of_node_put(s_node);
			return -EINVAL;
		}
		priv->intel_tsens[i] = ts;
		i++;
	}

	return 0;
}

static int intel_tsens_thermal_probe(struct platform_device *pdev)
{
	struct intel_tsens_priv *intel_tsens_priv;
	int ret;

	intel_tsens_priv = devm_kzalloc(&pdev->dev,
					sizeof(struct intel_tsens_priv),
					GFP_KERNEL);
	if (!intel_tsens_priv)
		return -ENOMEM;
	intel_tsens_priv->pdev = pdev;
	if (pdev->dev.of_node) {
		ret = intel_tsens_config_dt(intel_tsens_priv);
		if (ret) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "Non Device Tree build is not supported\n");
		return -EINVAL;
	}
	ret = intel_tsens_clk_config(intel_tsens_priv);
	if (ret) {
		dev_err(&pdev->dev, "Thermal clk config failed\n");
		return ret;
	}
	ret = intel_tsens_add_pdev(intel_tsens_priv);
	if (ret) {
		dev_err(&pdev->dev, "platform device registration failed\n");
		goto remove_pdev;
	}
	ret = intel_tsens_add_thermal_zones(intel_tsens_priv);
	if (ret) {
		dev_err(&pdev->dev, "thermal zone configuration failed\n");
		goto remove_tz;
	}
	platform_set_drvdata(pdev, intel_tsens_priv);
	i2c_plat_data.pdata = intel_tsens_priv;
	return 0;

remove_tz:
	intel_tsens_remove_thermal_zones(intel_tsens_priv);
remove_pdev:
	intel_tsens_unregister_pdev(intel_tsens_priv);
	intel_tsens_remove_clk(intel_tsens_priv);
	return ret;
}

/* Device Exit */
static int intel_tsens_thermal_exit(struct platform_device *pdev)
{
	struct intel_tsens_priv *priv = platform_get_drvdata(pdev);

	if (!priv) {
		dev_err(&pdev->dev,
			"unable to get private data\n");
		return -EINVAL;
	}
	intel_tsens_remove_thermal_zones(priv);
	intel_tsens_unregister_pdev(priv);
	intel_tsens_remove_clk(priv);

	return 0;
}

static const struct of_device_id intel_tsens_thermal_id_table[] = {
	{ .compatible = "intel,intel-tsens" },
	{}
};
MODULE_DEVICE_TABLE(of, intel_tsens_thermal_id_table);

static struct platform_driver intel_tsens_thermal_driver = {
	.probe = intel_tsens_thermal_probe,
	.remove = intel_tsens_thermal_exit,
	.driver = {
		.name = "intel_tsens_thermal",
		.of_match_table = intel_tsens_thermal_id_table,
	},
};

module_platform_driver(intel_tsens_thermal_driver);

MODULE_DESCRIPTION("TSENS Thermal Driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
