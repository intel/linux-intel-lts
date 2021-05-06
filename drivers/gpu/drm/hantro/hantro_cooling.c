// SPDX-License-Identifier: GPL-2.0
/*
 *    Hantro thermal cooling.
 *
 *    Copyright (c) 2021, Intel Corporation
 */

#include "hantro_cooling.h"

static int hantro_cooling_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct device_info *pdevinfo = cdev->devdata;

	if (!pdevinfo)
		return -EINVAL;

	*state = pdevinfo->thermal_data.media_clk_max_state;
	return 0;
}

static int hantro_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	struct device_info *pdevinfo = cdev->devdata;

	if (!pdevinfo || state > pdevinfo->thermal_data.media_clk_max_state)
		return -EINVAL;

	if (state == pdevinfo->thermal_data.media_clk_state ||
	    state > 2) //only	3 states supported
		return 0;

	pdevinfo->thermal_data.media_clk_state = state;

	if (hantro_drm.device_type == DEVICE_KEEMBAY)
		pdevinfo->thermal_data.clk_freq = kmb_freq_table[state];
	else if (hantro_drm.device_type == DEVICE_THUNDERBAY)
		pdevinfo->thermal_data.clk_freq = tbh_freq_table[state];

	pr_info("set_cur_state: %ld for device %d\n",
		pdevinfo->thermal_data.clk_freq, pdevinfo->deviceid);
	return 0;
}

static int hantro_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct device_info *pdevinfo = cdev->devdata;

	if (!pdevinfo)
		return -EINVAL;

	*state = pdevinfo->thermal_data.media_clk_state;
	return 0;
}

static const struct thermal_cooling_device_ops hantro_cooling_ops = {
	.get_max_state = hantro_cooling_get_max_state,
	.get_cur_state = hantro_cooling_get_cur_state,
	.set_cur_state = hantro_cooling_set_cur_state,
};

int setup_thermal_cooling(struct device_info *pdevinfo)
{
	int result;
	char thermal_str[64];
	struct device_node *np;
	struct thermal_cooling_device *cdev;

	if (!pdevinfo) {
		pr_warn("Device info NULL\n");
		return -EINVAL;
	}

	np = pdevinfo->dev->of_node;
	pdevinfo->thermal_data.media_clk_max_state = 3;
	if (hantro_drm.device_type == DEVICE_KEEMBAY)
		pdevinfo->thermal_data.clk_freq = kmb_freq_table[0];
	else if (hantro_drm.device_type == DEVICE_THUNDERBAY)
		pdevinfo->thermal_data.clk_freq = tbh_freq_table[0];

	sprintf(thermal_str, "media-cooling%d", pdevinfo->deviceid);
	cdev = devm_thermal_of_cooling_device_register(pdevinfo->dev, np, thermal_str, pdevinfo,
						       &hantro_cooling_ops);
	if (IS_ERR(cdev)) {
		result = PTR_ERR(cdev);
		dev_err(pdevinfo->dev,
			"failed to register thermal zone device %d", result);
	}

	pdevinfo->thermal_data.cooling_dev = cdev;
	return 0;
}
