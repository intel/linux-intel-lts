// SPDX-License-Identifier: GPL-2.0-only
/*
 * keembay-thermal.c - KeemBay Thermal Driver.
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
#include "keembay_tsens.h"

static int TempLookupSearch(int low, int high, int n)
{
	int mid;

	mid = low + (high - low) / 2;
	if (raw_kmb_data[mid].N == n)
		return raw_kmb_data[mid].temp;
	if (raw_kmb_data[mid].N > n)
		return TempLookupSearch(low, mid - 1, n);
	if (raw_kmb_data[mid].N < n)
		return TempLookupSearch(mid + 1, high, n);
	return 0;
}

static int kmb_sensor_read_temp(void __iomem *regs_val,
						int offset,
						int sample_valid_mask,
						int sample_value,
						int bit_shift,
						int *temp)
{
	int result;
	/* clear the bit of TSENS_EN and re-enable again */
	iowrite32(0x00, regs_val+AON_TSENS_CFG);
	iowrite32(CFG_MASK_MANUAL, regs_val+AON_TSENS_CFG);
	*temp = ioread32(regs_val+offset);
	if (*temp & sample_valid_mask) {
		*temp = (*temp >> bit_shift & sample_value);
		if (*temp >= Lower_Temp_Nrange && *temp <= Upper_Temp_Nrange) {
			result = TempLookupSearch(0, NUM_RAW_KMB - 1, *temp);
			*temp = result;
		} else {
			if (*temp < Lower_Temp_Nrange)
				*temp = Lower_Temp;
			else
				*temp = Upper_Temp;
		}
	} else {
		*temp = -255;
	}
	return 0;
}

int kmb_tj_temp_list[6];
static int keembay_get_temp(struct thermal_zone_device *thermal,
							int *temp)
{
	struct kmb_trip_point_info *kmb_zone_info = thermal->devdata;
	struct keembay_therm_info *ktherm = kmb_zone_info->thermal_info;

	spin_lock(&ktherm->lock);
	switch (kmb_zone_info->sensor_type) {
	case KEEMBAY_SENSOR_MSS:
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA0,
					MSS_T_SAMPLE_VALID,
					MSS_T_SAMPLE,
					MSS_BIT_SHIFT,
					temp);
			ktherm->mss = *temp;
			kmb_tj_temp_list[2] = ktherm->mss;
			break;

	case KEEMBAY_SENSOR_CSS:
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA0,
					CSS_T_SAMPLE_VALID,
					CSS_T_SAMPLE,
					CSS_BIT_SHIFT,
					temp);
			ktherm->css = *temp;
			kmb_tj_temp_list[3] = ktherm->css;
			break;

	case KEEMBAY_SENSOR_NCE:
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA1,
					NCE0_T_SAMPLE_VALID,
					NCE0_T_SAMPLE,
					NCE0_BIT_SHIFT,
					&ktherm->nce0);
			kmb_sensor_read_temp(ktherm->regs_val,
					AON_TSENS_DATA1,
					NCE1_T_SAMPLE_VALID,
					NCE1_T_SAMPLE,
					NCE1_BIT_SHIFT,
					&ktherm->nce1);
			kmb_tj_temp_list[4] = ktherm->nce0;
			kmb_tj_temp_list[5] = ktherm->nce1;
			ktherm->nce = ktherm->nce1;
			*temp = ktherm->nce1;
			if (ktherm->nce0 > ktherm->nce1) {
				ktherm->nce = ktherm->nce0;
				*temp = ktherm->nce0;
			}
			kmb_tj_temp_list[1] = *temp;
			break;

	case KEEMBAY_SENSOR_SOC:
			//temp = css > mss ? (css > nce ? css : nce)
			//: (mss > nce ? mss : nce);
			*temp = ktherm->css > ktherm->mss ?
					(ktherm->css > ktherm->nce ?
					ktherm->css : ktherm->nce)
					: (ktherm->mss > ktherm->css ?
					ktherm->mss : ktherm->css);
					kmb_tj_temp_list[0] = *temp;
			break;
	default:
			break;
	}
	spin_unlock(&ktherm->lock);
	return 0;
}
EXPORT_SYMBOL(kmb_tj_temp_list);

static int keembay_thermal_get_trip_type(struct thermal_zone_device *zone,
			int trip, enum thermal_trip_type *type)
{

	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;

	*type = kmb_zone_info->trip_info[trip].trip_type;
	return 0;
}


static int keembay_thermal_get_trip_temp(struct thermal_zone_device *zone,
				int trip, int *temp)
{

	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;

	*temp = kmb_zone_info->trip_info[trip].temperature;
	return 0;
}

/* Refer https://lwn.net/Articles/242046/ how to receive this event in userspace */
int notify_user_space(struct thermal_zone_device *tz, int trip)
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


static int keembay_thermal_notify(struct thermal_zone_device *zone,
			       int trip, enum thermal_trip_type type)
{
	struct kmb_trip_point_info *kmb_zone_info = zone->devdata;
	struct keembay_therm_info *ktherm = kmb_zone_info->thermal_info;

	notify_user_space(zone, trip);
	switch (type) {
	case THERMAL_TRIP_PASSIVE:
		dev_warn(ktherm->dev, "Thermal reached to passive temperature\n");
		break;
	case THERMAL_TRIP_CRITICAL:
		dev_warn(ktherm->dev, "Thermal reached to critical temperature\n");
		break;
	default:
		dev_warn(ktherm->dev, "Thermal not reached to passive temperature\n");
		break;
	}
	return 0;
}

static int keembay_bind(struct thermal_zone_device *tz,
		    struct thermal_cooling_device *cdev)
{
	int ret;

	/*Check here thermal device zone name and*/
	/*cdev name to match, then call the bind device */
	if (strncmp(tz->type, cdev->type, THERMAL_NAME_LENGTH) == 0) {
		ret = thermal_zone_bind_cooling_device
				(tz,
				KEEMBAY_TRIP_PASSIVE,
				cdev,
				THERMAL_NO_LIMIT,
				THERMAL_NO_LIMIT,
				THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&tz->device,
				"binding zone %s with cdev %s failed:%d\n",
				tz->type, cdev->type, ret);
			return ret;
		}
	}
	return 0;
}

static int keembay_unbind(struct thermal_zone_device *tz,
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

static struct thermal_zone_device_ops ops = {
	.bind = keembay_bind,
	.unbind = keembay_unbind,
	.get_temp = keembay_get_temp,
	.get_trip_type	= keembay_thermal_get_trip_type,
	.get_trip_temp	= keembay_thermal_get_trip_temp,
	.notify		= keembay_thermal_notify,
/*	.set_emul_temp = keembay_thermal_emulation */

};


static const struct of_device_id keembay_thermal_id_table[] = {
	{ .compatible = "intel,keembay-tsens" },
	{}
};

struct keembay_therm_info *g_thermal_data;


static int hddl_device_thermal_init(void);
static int hddl_device_thermal_exit(void);

static int keembay_thermal_probe(struct platform_device *pdev)
{
	int ret;
	int error;

	printk(KERN_INFO "Keembay thermal probe \n");

	g_thermal_data = devm_kzalloc(&pdev->dev,
					sizeof(struct keembay_therm_info),
					GFP_KERNEL);
	if (!g_thermal_data)
		return -ENOMEM;
	/* spin lock init */
	spin_lock_init(&g_thermal_data->lock);
	g_thermal_data->regs_val = ioremap(AON_INTERFACE, 32);
	/* getting clk */
	g_thermal_data->thermal_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(g_thermal_data->thermal_clk)) {
		ret = PTR_ERR(g_thermal_data->thermal_clk);
		if (ret != -EPROBE_DEFER) {
			dev_err(&pdev->dev,
					"failed to get thermal clk: %d\n", ret);
		}
		return PTR_ERR(g_thermal_data->thermal_clk);
	}
	ret = clk_prepare_enable(g_thermal_data->thermal_clk);
	if (ret) {
		dev_err(&pdev->dev,
		"failed to enable thermal clk: %d\n",
		ret);
	}
	//Temperature sensor clock must be in the range 1.25MHz.
	ret = clk_set_rate(g_thermal_data->thermal_clk, 1250000);
	ret = clk_prepare_enable(g_thermal_data->thermal_clk);
	error = clk_enable(g_thermal_data->thermal_clk);
	if (error)
		return error;

#if defined(MODULE)
	hddl_device_thermal_init();
#endif /* MODULE */

	return 0;
}

int keembay_thermal_zone_register(struct kmb_trip_point_info *zone_trip_info)
{
	int ret;

	zone_trip_info->thermal_info = g_thermal_data;
	zone_trip_info->tz =  thermal_zone_device_register(
		zone_trip_info->sensor_name,
		zone_trip_info->n_trips,
		0,
		zone_trip_info,
		&ops,
		NULL,
		zone_trip_info->passive_delay,
		zone_trip_info->polling_delay
		);
	if (IS_ERR(zone_trip_info->tz)) {
		ret = PTR_ERR(zone_trip_info->tz);
		dev_err(g_thermal_data->dev,
			"failed to register thermal zone device %d\n", ret);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(keembay_thermal_zone_register);

/* Zone Exit */
int keembay_thermal_zone_unregister(struct kmb_trip_point_info *zone_trip_info)
{
	thermal_zone_device_unregister(zone_trip_info->tz);
	return 0;
}
EXPORT_SYMBOL_GPL(keembay_thermal_zone_unregister);


/* Device Exit */
static int keembay_thermal_exit(struct platform_device *pdev)
{
	struct thermal_zone_device *keembay_thermal =
				platform_get_drvdata(pdev);

#if defined(MODULE)
	hddl_device_thermal_exit();
#endif /* MODULE */

	thermal_zone_device_unregister(keembay_thermal);
	clk_disable_unprepare(g_thermal_data->thermal_clk);

	return 0;
}
MODULE_DEVICE_TABLE(of, keembay_thermal_id_table);

static struct platform_driver keembay_thermal_driver = {
	.probe = keembay_thermal_probe,
	.remove = keembay_thermal_exit,
	.driver = {
		.name = "keembay_thermal",
		.of_match_table = keembay_thermal_id_table,
	},
};

module_platform_driver(keembay_thermal_driver);


struct kmb_trip_point_info mss_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_MSS,
	.sensor_name = "mss",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 3,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 40000 },
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info css_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_CSS,
	.sensor_name = "css",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 3,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 40000 },
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info nce_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_NCE,
	.sensor_name = "nce",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 3,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 40000 },
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

struct kmb_trip_point_info soc_zone_trip_info = {
	.sensor_type = KEEMBAY_SENSOR_SOC,
	.sensor_name = "keembay_thermal",
	.passive_delay = 1000,
	.polling_delay = 2000,
	.n_trips = 2,
	.trip_info = {
		{ THERMAL_TRIP_PASSIVE, 80000 },
		{ THERMAL_TRIP_CRITICAL, 1000000 },
	},
	NULL,
	NULL,
};

static int hddl_device_thermal_init(void)
{
	printk(KERN_INFO "Keembay thermal init \n");
	keembay_thermal_zone_register(&mss_zone_trip_info);
	keembay_thermal_zone_register(&css_zone_trip_info);
	keembay_thermal_zone_register(&nce_zone_trip_info);
	keembay_thermal_zone_register(&soc_zone_trip_info);
	return 0;
};

static int hddl_device_thermal_exit(void)
{
	return 0;
};

#if !defined(MODULE)
late_initcall(hddl_device_thermal_init);
late_initcall(hddl_device_thermal_exit);
#endif

MODULE_DESCRIPTION("KeemBay Thermal Driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_LICENSE("GPL v2");
