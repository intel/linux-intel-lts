// SPDX-License-Identifier: GPL-2.0-only
/*
 * intel-tsens-thermal.c - Tsens Thermal Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/hddl_device.h>

struct intel_tsens_i2c {
	u16 buffer_idx;
	u8 idx_write_cnt;
	bool read_only;
	int sensor_type;
};

struct intel_tsens_priv {
	int n_sens;
	spinlock_t lock;
	bool global_clk_available;
	void __iomem *base_addr;
	struct clk *tsens_clk;
	u32 tsens_clk_rate;
	struct intel_tsens **intel_tsens;
	struct device *dev;
	struct platform_device *pdev;
	struct intel_tsens_plat_info plat_info;
	struct intel_tsens_i2c tsens_i2c;
};

static int g_nsens;
static struct intel_tsens **g_intel_tsens;

static int intel_i2c_tsens_slave_cb(struct i2c_client *client,
				enum i2c_slave_event event, u8 *val)
{
	struct intel_tsens_priv *priv = i2c_get_clientdata(client);
	struct intel_tsens_i2c *tsens_i2c = &priv->tsens_i2c;

	switch (event) {
	case I2C_SLAVE_WRITE_RECEIVED:
	{
		tsens_i2c->sensor_type = *val;
		break;
	}

	case I2C_SLAVE_READ_PROCESSED:
	{
		/* The previous byte made it to the bus, get next one */
	}
		/* fallthrough */
	case I2C_SLAVE_READ_REQUESTED:
	{
		*val = (priv->intel_tsens[tsens_i2c->sensor_type]->curr_temp >>
			(tsens_i2c->buffer_idx * 8));
		tsens_i2c->buffer_idx++;
		/*
		 * Do not increment buffer_idx here, because we don't know if
		 * this byte will be actually used. Read Linux I2C slave docs
		 * for details.
		 */
		break;
	}

	case I2C_SLAVE_STOP:
	{
		tsens_i2c->idx_write_cnt = 0;
		tsens_i2c->buffer_idx = 0;
		break;
	}
	case I2C_SLAVE_WRITE_REQUESTED:
	{
		tsens_i2c->idx_write_cnt = 0;
		tsens_i2c->buffer_idx = 0;
		break;
	}

	default:
		break;
	}

	return 0;
}

static int intel_i2c_tsens_slave_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	struct intel_tsens_priv *priv =
		(struct intel_tsens_priv *)id->driver_data;

	dev_info(&priv->pdev->dev, "Slave address %s is %d",
					client->name, client->addr);
	i2c_set_clientdata(client, priv);
	ret = i2c_slave_register(client, intel_i2c_tsens_slave_cb);
	if (ret) {
		dev_err(&priv->pdev->dev, "i2c slave register failed\n");
		return ret;
	}

	return 0;
};

static int intel_i2c_tsens_slave_remove(struct i2c_client *client)
{
	i2c_slave_unregister(client);
	return 0;
}

static struct i2c_device_id intel_i2c_tsens_slave_id[] = {
	{ "intel_tsens", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, intel_i2c_tsens_slave_id);

static struct i2c_driver intel_i2c_tsens_slave_driver = {
	.driver = {
		.name = "intel_tsens",
	},
	.probe = intel_i2c_tsens_slave_probe,
	.remove = intel_i2c_tsens_slave_remove,
	.id_table = intel_i2c_tsens_slave_id,
};


struct intel_tsens **intel_tsens_hddl_register(int *nsens)
{
	*nsens = g_nsens;
	return g_intel_tsens;
}
EXPORT_SYMBOL(intel_tsens_hddl_register);

static int intel_tsens_register_pdev(struct intel_tsens_plat_info *plat_info)
{
	struct platform_device_info pdevinfo;
	struct platform_device *dd;
	struct intel_tsens_plat_data plat_data, *pdata = NULL;
	static int id;

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.name = plat_info->plat_name;
	pdevinfo.id = id++;
	plat_data.base_addr = plat_info->base_addr;
	plat_data.name = plat_info->plat_name;
	plat_data.get_temp = NULL;
	pdevinfo.data = &plat_data;
	pdevinfo.size_data = sizeof(plat_data);
	dd = platform_device_register_full(&pdevinfo);
	if (IS_ERR(dd))
		return -EINVAL;

	pdata = dd->dev.platform_data;
	plat_info->pdev = dd;

	return 0;
}

static int intel_tsens_add_pdev(struct intel_tsens_priv *priv)
{
	int i, ret;

	if (priv->plat_info.plat_name != NULL) {
		priv->plat_info.base_addr = priv->base_addr;
		ret = intel_tsens_register_pdev(&priv->plat_info);
		if (ret)
			return ret;
	}
	for (i = 0; i < priv->n_sens; i++) {
		struct intel_tsens *tsens = priv->intel_tsens[i];

		if (tsens->plat_info.plat_name == NULL)
			continue;
		tsens->plat_info.base_addr = tsens->base_addr;
		ret = intel_tsens_register_pdev(&tsens->plat_info);
		if (ret)
			goto f_tsens_pdev;
	}
	return 0;
f_tsens_pdev:
	/*TODO: free pdev*/
	return ret;
}

static int intel_tsens_thermal_get_temp(struct thermal_zone_device *tz,
							int *temp)
{
	struct intel_tsens *tsens = (struct intel_tsens *)tz->devdata;
	struct intel_tsens_priv *priv = (struct intel_tsens_priv *)tsens->pdata;
	struct platform_device *pdev;
	struct intel_tsens_plat_data *plat_data;
	int type = tsens->sensor_type;

	if (tsens->plat_info.plat_name != NULL) {
		pdev = tsens->plat_info.pdev;
		plat_data = pdev->dev.platform_data;

		if (!plat_data) {
			dev_err(&pdev->dev, "Platform data not found for %s\n",
					tsens->name);
			return -EINVAL;
		}
		if (!plat_data->get_temp) {
			dev_dbg(&pdev->dev,
				"Platform driver not available for %s\n",
				tsens->name);
			*temp = 0;
			return 0;
		}
		if (plat_data->get_temp(pdev, type, temp))
			return -EINVAL;
		tsens->curr_temp = *temp;
		return 0;
	}
	if (priv->plat_info.plat_name != NULL) {
		pdev = priv->plat_info.pdev;
		plat_data = pdev->dev.platform_data;

		if (!plat_data) {
			dev_err(&pdev->dev, "Platform data not found for %s\n",
					tsens->name);
			return -EINVAL;
		}
		if (!plat_data->get_temp) {
			dev_dbg(&pdev->dev,
				"Platform driver not available for %s\n",
				tsens->name);
			*temp = 0;
			return 0;
		}

		if (plat_data->get_temp(pdev, type, temp))
			return -EINVAL;
		tsens->curr_temp = *temp;
		return 0;
	}

	return -EINVAL;
}

static int intel_tsens_thermal_get_trip_type(struct thermal_zone_device *tz,
			int trip, enum thermal_trip_type *type)
{
	struct intel_tsens *tsens = (struct intel_tsens *)tz->devdata;

	*type = tsens->trip_info[trip]->trip_type;
	return 0;
}


static int intel_tsens_thermal_get_trip_temp(struct thermal_zone_device *tz,
				int trip, int *temp)
{

	struct intel_tsens *tsens = (struct intel_tsens *)tz->devdata;

	*temp = tsens->trip_info[trip]->temp;
	return 0;
}

/* Refer https://lwn.net/Articles/242046/
 * how to receive this event in userspace
 */
static int intel_tsens_notify_user_space(struct thermal_zone_device *tz,
		int trip)
{
	char *thermal_prop[5];
	int i;

	mutex_lock(&tz->lock);
	thermal_prop[0] = kasprintf(GFP_KERNEL, "NAME=%s", tz->type);
	thermal_prop[1] = kasprintf(GFP_KERNEL, "TEMP=%d",
						tz->emul_temperature);
	thermal_prop[2] = kasprintf(GFP_KERNEL, "TRIP=%d", trip);
	thermal_prop[3] = kasprintf(GFP_KERNEL, "EVENT=%d", tz->notify_event);
	thermal_prop[4] = NULL;
	kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, thermal_prop);
	for (i = 0; i < 4; ++i)
		kfree(thermal_prop[i]);
	mutex_unlock(&tz->lock);
	return 0;
}


static int intel_tsens_thermal_notify(struct thermal_zone_device *tz,
			       int trip, enum thermal_trip_type type)
{
	struct intel_tsens *tsens = (struct intel_tsens *)tz->devdata;
	struct intel_tsens_priv *priv = (struct intel_tsens_priv *)tsens->pdata;

	intel_tsens_notify_user_space(tz, trip);
	switch (type) {
	case THERMAL_TRIP_PASSIVE:
		dev_warn(&priv->pdev->dev, "Thermal reached to passive temperature\n");
		break;
	case THERMAL_TRIP_CRITICAL:
		dev_warn(&priv->pdev->dev, "Thermal reached to critical temperature\n");
		break;
	default:
		dev_warn(&priv->pdev->dev, "Thermal not reached to passive temperature\n");
		break;
	}
	return 0;
}

static int intel_tsens_thermal_bind(struct thermal_zone_device *tz,
		    struct thermal_cooling_device *cdev)
{
	int ret;
	struct intel_tsens *tsens = (struct intel_tsens *)tz->devdata;
	struct intel_tsens_priv *priv = (struct intel_tsens_priv *)tsens->pdata;

	/*Check here thermal device zone name and*/
	/*cdev name to match, then call the bind device */
	if (strncmp(tz->type, cdev->type, THERMAL_NAME_LENGTH) == 0) {
		ret = thermal_zone_bind_cooling_device
				(tz,
				THERMAL_TRIP_PASSIVE,
				cdev,
				THERMAL_NO_LIMIT,
				THERMAL_NO_LIMIT,
				THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&priv->pdev->dev,
				"binding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static int intel_tsens_thermal_unbind(struct thermal_zone_device *tz,
		      struct thermal_cooling_device *cdev)
{
	int ret;

	ret = thermal_zone_unbind_cooling_device(tz, 0, cdev);
	if (ret) {
		dev_err(&tz->device,
			"unbinding zone %s with cdev %s failed:%d\n",
			tz->type, cdev->type, ret);
		return ret;
	}
	return 0;
}

static struct thermal_zone_device_ops tsens_thermal_ops = {
	.bind = intel_tsens_thermal_bind,
	.unbind = intel_tsens_thermal_unbind,
	.get_temp = intel_tsens_thermal_get_temp,
	.get_trip_type	= intel_tsens_thermal_get_trip_type,
	.get_trip_temp	= intel_tsens_thermal_get_trip_temp,
	.notify		= intel_tsens_thermal_notify,
/*	.set_emul_temp = tsens_thermal_emulation */

};

static int intel_tsens_add_thermal_zone(struct intel_tsens_priv *priv)
{
	int ret, i;

	for (i = 0; i < priv->n_sens; i++) {
		priv->intel_tsens[i]->tz =  thermal_zone_device_register(
			priv->intel_tsens[i]->name,
			priv->intel_tsens[i]->n_trips,
			0,
			priv->intel_tsens[i],
			&tsens_thermal_ops,
			NULL,
			priv->intel_tsens[i]->passive_delay,
			priv->intel_tsens[i]->polling_delay
			);
		if (IS_ERR(priv->intel_tsens[i]->tz)) {
			ret = PTR_ERR(priv->intel_tsens[i]->tz);
			dev_err(&priv->pdev->dev,
				"failed to register thermal zone device %s\n",
				priv->intel_tsens[i]->name);
			goto remove_thermal_zone;
		}
	}
	return 0;

remove_thermal_zone:
	/*TODO: unregister thermal zone */
	return ret;
}

static void intel_tsens_remove_clk_config(struct intel_tsens_priv *priv)
{
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

		ret = clk_prepare_enable(priv->tsens_clk);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to enable thermal clk: %d\n",
				ret);
			return ret;
		}
		ret = clk_set_rate(priv->tsens_clk, priv->tsens_clk_rate);
		ret = clk_prepare_enable(priv->tsens_clk);
		ret = clk_enable(priv->tsens_clk);
		if (ret) {
			dev_err(&pdev->dev,
				"failed to enable thermal clk: %d\n",
				ret);
			return ret;
		}
	}

	return 0;
}

static int intel_tsens_config_sensors(struct device_node *s_node,
			struct intel_tsens *tsens, int sensor_type)
{
	struct intel_tsens_priv *priv = (struct intel_tsens_priv *)tsens->pdata;
	struct platform_device *pdev = priv->pdev;
	s32 trip_temp_count, trip_temp_type_c, i, ret;

	of_property_read_string_index(s_node, "plat_name", 0,
				&tsens->plat_info.plat_name);
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
	if ((trip_temp_count != trip_temp_type_c) ||
		(trip_temp_count <= 0) || (trip_temp_type_c <= 0)) {
		dev_err(&pdev->dev,
			"trip temp config is missing in dt for %s\n",
			tsens->name);
		return -EINVAL;
	}

	tsens->trip_info =
		devm_kzalloc(&pdev->dev,
			(sizeof(struct intel_tsens_trip_info *) *
			 trip_temp_count),
			GFP_KERNEL);
	if (tsens->trip_info == NULL) {
		dev_err(&pdev->dev,
				"Temperature Memory alloc failed for %s\n",
				s_node->name);
		return -ENOMEM;
	}
	tsens->n_trips = trip_temp_count;
	for (i = 0; i < trip_temp_count; i++) {
		const char *trip_name;

		tsens->trip_info[i] = devm_kzalloc(&pdev->dev,
					(sizeof(struct intel_tsens_trip_info)),
					GFP_KERNEL);
		if (tsens->trip_info[i] == NULL) {
			dev_err(&pdev->dev, "Temperature Memory alloc failed for %s\n",
				s_node->name);
			ret = -ENOMEM;
			goto f_tsens_trip;
		}

		of_property_read_u32_index(s_node, "trip_temp", i,
				&tsens->trip_info[i]->temp);
		of_property_read_string_index(s_node, "trip_type", i,
				&trip_name);
		if (!strcmp(trip_name, "passive"))
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_PASSIVE;
		else if (!strcmp(trip_name, "critical"))
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_CRITICAL;
		else if (!strcmp(trip_name, "hot"))
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_HOT;
		else
			tsens->trip_info[i]->trip_type = THERMAL_TRIP_ACTIVE;
	}
	return 0;
f_tsens_trip:
	while (i >= 0) {
		if (tsens->trip_info[i] != NULL)
			devm_kfree(&pdev->dev, tsens->trip_info[i]);
		i--;
	}
	devm_kfree(&pdev->dev, tsens->trip_info);
	return ret;
}

static int intel_tsens_config_dt(struct intel_tsens_priv *priv)
{
	struct platform_device *pdev = priv->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *s_node;
	struct resource *res;
	int i = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base_addr = devm_ioremap_resource(&pdev->dev, res);
	priv->n_sens = of_get_child_count(np);
	priv->global_clk_available = of_property_read_bool(np, "clocks");
	of_property_read_u32(np, "clk-rate", &priv->tsens_clk_rate);
	of_property_read_string_index(np, "plat_name", 0,
				&priv->plat_info.plat_name);
	if (priv->n_sens == 0) {
		dev_err(&pdev->dev, "No sensors configured in dt\n");
		return -EINVAL;
	}
	priv->intel_tsens =
		devm_kzalloc(&pdev->dev,
			(sizeof(struct intel_tsens *) * priv->n_sens),
			GFP_KERNEL);
	g_intel_tsens = priv->intel_tsens;
	g_nsens = priv->n_sens;
	for_each_child_of_node(np, s_node) {
		int reg_count, size_count;

		priv->intel_tsens[i] = devm_kzalloc(&pdev->dev,
					sizeof(struct intel_tsens),
					GFP_KERNEL);
		if (priv->intel_tsens[i] == NULL) {
			dev_err(&pdev->dev, "Memory alloc failed for %s\n",
					s_node->name);
			i--;
			goto free_tsens_sensors;
		}
		strcpy(priv->intel_tsens[i]->name, s_node->name);
		if (!of_property_read_u32(s_node, "address-cells", &reg_count)
			&& !of_property_read_u32(s_node, "size-cells",
				&size_count)) {
			if (reg_count > 1) {
				of_property_read_u64_index(s_node, "reg", 0,
					  &priv->intel_tsens[i]->addr);
			} else {
				of_property_read_u32_index(s_node, "reg", 0,
					  (u32 *)&priv->intel_tsens[i]->addr);
			}
			if (size_count > 1) {
				int index =
					(reg_count > 1)?(reg_count/2) :
					reg_count;

				of_property_read_u64_index(s_node, "reg", index,
					  &priv->intel_tsens[i]->size);
			} else {
				of_property_read_u32_index(s_node, "reg",
					reg_count,
					(u32 *)&priv->intel_tsens[i]->size);

			}
			dev_info(&pdev->dev, "address %llx\n",
					priv->intel_tsens[i]->addr);
			dev_info(&pdev->dev, "size %llx\n",
					priv->intel_tsens[i]->size);
			priv->intel_tsens[i]->base_addr =
				devm_ioremap(&pdev->dev,
					priv->intel_tsens[i]->addr,
					priv->intel_tsens[i]->size);
		} else
			priv->intel_tsens[i]->base_addr = priv->base_addr;
		if (priv->intel_tsens[i]->base_addr == NULL) {
			dev_err(&pdev->dev, "ioremap failed for %s\n",
					priv->intel_tsens[i]->name);
			goto unmap;
		}
		priv->intel_tsens[i]->pdata = priv;
		if (intel_tsens_config_sensors(
					s_node, priv->intel_tsens[i], i)) {
			dev_err(&pdev->dev,
					"Missing sensor info in dts for %s\n",
					priv->intel_tsens[i]->name);
			goto unmap;
		}
		dev_info(&pdev->dev, "Parsing cfg for %s from dt is success\n",
				priv->intel_tsens[i]->name);
		i++;
	}

	return 0;
unmap:
free_tsens_sensors:
	while (i >= 0)
		devm_kfree(&pdev->dev, priv->intel_tsens[i--]);
	devm_kfree(&pdev->dev, priv->intel_tsens);
	return -EINVAL;
}

static void intel_tsens_remove_dt(struct intel_tsens_priv *priv)
{
	int i;
	struct platform_device *pdev = priv->pdev;

	for (i = 0; i < priv->n_sens; i++)
		devm_kfree(&pdev->dev, priv->intel_tsens[i]);
	devm_kfree(&pdev->dev, priv->intel_tsens);
}

static int intel_tsens_thermal_probe(struct platform_device *pdev)
{
	struct intel_tsens_priv *intel_tsens_priv;
	int ret;

	dev_info(&pdev->dev,
		"%s Entry\n", __func__);

	intel_tsens_priv = devm_kzalloc(&pdev->dev,
					sizeof(struct intel_tsens_priv),
					GFP_KERNEL);
	if (intel_tsens_priv == NULL) {
		dev_err(&pdev->dev, "No memory");
		return -ENOMEM;
	}
	intel_tsens_priv->pdev = pdev;
	spin_lock_init(&intel_tsens_priv->lock);
	if (pdev->dev.of_node) {
		ret = intel_tsens_config_dt(intel_tsens_priv);
		if (ret) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			devm_kfree(&pdev->dev, intel_tsens_priv);
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "Non Device Tree build is not supported\n");
		devm_kfree(&pdev->dev, intel_tsens_priv);
		return -EINVAL;
	}
	ret = intel_tsens_clk_config(intel_tsens_priv);
	if (ret) {
		intel_tsens_remove_dt(intel_tsens_priv);
		devm_kfree(&pdev->dev, intel_tsens_priv);
		return ret;
	}
	ret = intel_tsens_add_pdev(intel_tsens_priv);
	if (ret) {
		dev_err(&pdev->dev, "platform device registration failed\n");
		intel_tsens_remove_dt(intel_tsens_priv);
		intel_tsens_remove_clk_config(intel_tsens_priv);
		devm_kfree(&pdev->dev, intel_tsens_priv);
		return ret;

	}
	ret = intel_tsens_add_thermal_zone(intel_tsens_priv);
	if (ret) {
		dev_err(&pdev->dev, "thermal zone configuration failed\n");
		/*TODO: remove platform device */
		intel_tsens_remove_dt(intel_tsens_priv);
		intel_tsens_remove_clk_config(intel_tsens_priv);
		devm_kfree(&pdev->dev, intel_tsens_priv);
		return ret;
	}
	intel_i2c_tsens_slave_id[0].driver_data =
				(kernel_ulong_t)intel_tsens_priv;
	if (i2c_add_driver(&intel_i2c_tsens_slave_driver)) {
		dev_err(&pdev->dev, "I2C register driver failed\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev,
		"%d sensors registered\n", intel_tsens_priv->n_sens);
	dev_info(&pdev->dev,
		"%s Exit\n", __func__);
	return 0;
}

/* Device Exit */
static int intel_tsens_thermal_exit(struct platform_device *pdev)
{
	i2c_del_driver(&intel_i2c_tsens_slave_driver);

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
