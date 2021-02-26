// SPDX-License-Identifier: GPL-2.0-only
/*
 * VPU cooling Device Kernel module.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/xlink.h>
#include <linux/of_platform.h>

/* used to extract fields from xlink sw device id */
#define SW_DEVICE_ID_INTERFACE_SHIFT 24U
#define SW_DEVICE_ID_INTERFACE_MASK 0x7
#define SW_DEVICE_ID_INTERFACE_SMASK \
		(SW_DEVICE_ID_INTERFACE_MASK << SW_DEVICE_ID_INTERFACE_SHIFT)
#define SW_DEVICE_ID_INTERFACE_IPC_VALUE 0x0
#define SW_DEVICE_ID_INTERFACE_IPC_SVALUE \
		(SW_DEVICE_ID_INTERFACE_IPC_VALUE \
		<< SW_DEVICE_ID_INTERFACE_SHIFT)
#define SW_DEVICE_ID_VPU_ID_SHIFT 1U
#define SW_DEVICE_ID_VPU_ID_MASK 0x7
#define SW_DEVICE_ID_VPU_ID_SMASK \
	(SW_DEVICE_ID_VPU_ID_MASK << SW_DEVICE_ID_VPU_ID_SHIFT)
#define GET_VPU_ID_FROM_SW_DEVICE_ID(id) \
	(((id) >> SW_DEVICE_ID_VPU_ID_SHIFT) & SW_DEVICE_ID_VPU_ID_MASK)
#define GET_SW_DEVICE_ID_FROM_VPU_ID(id) \
	((((id) << SW_DEVICE_ID_VPU_ID_SHIFT) & SW_DEVICE_ID_VPU_ID_SMASK) \
	| SW_DEVICE_ID_INTERFACE_IPC_SVALUE)

/* flags to control repeated error/warning logs */
struct vpu_cd_err_flags {
	unsigned int s_err_status_logged : 1;
	unsigned int s_err_mode_logged : 1;
	unsigned int g_err_status_logged : 1;
	unsigned int g_err_mode_logged : 1;
};

struct vpu_cooling_data {
	struct thermal_cooling_device *cooling_dev;
	struct platform_device *pdev;
	u32 vpu_id;
	u32 sw_device_id;
	u32 phy_vpu_id;
	struct xlink_handle handle;
	struct vpu_cd_err_flags err_flags;
};

#define to_vpu_cooling_dev(x)	container_of((x), struct vpu_cooling_data, \
								cooling_dev)

/* Max number of supported power states */
#define MAX_POWER_STATE (6)

#define VPU_COOLING_SUCCESS (0)
#define VPU_COOLING_FAILED (-1)

static u32 no_of_vpu_devices;

static int vpu_cooling_get_max_state(struct thermal_cooling_device
				     *cooling_dev, unsigned long *state)
{
	struct vpu_cooling_data *vpu_cdev_priv_data = cooling_dev->devdata;
	struct platform_device *pdev = vpu_cdev_priv_data->pdev;

	dev_info(&pdev->dev, "Get max supported states function called\n");
	/* returning maximum supported vpu power states. */
	*state = (u32)MAX_POWER_STATE;

	return VPU_COOLING_SUCCESS;
}

static int vpu_cooling_set_cur_state(struct thermal_cooling_device
				      *cooling_dev, unsigned long state)
{
	struct vpu_cooling_data *vpu_cdev_priv_data = cooling_dev->devdata;
	enum xlink_error xlinkerror;
	enum xlink_device_power_mode mode;
	struct platform_device *pdev = vpu_cdev_priv_data->pdev;
	enum xlink_device_status device_status = XLINK_DEV_OFF;

	/* state to mode mapping is one-to-one */
	switch (state) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		mode = (enum xlink_device_power_mode)state;
		break;
	default:
		dev_err(&pdev->dev, "set current state=%u not supported\n",
			(unsigned int)state);
		return VPU_COOLING_FAILED;
	}

	xlinkerror = xlink_get_device_status(&vpu_cdev_priv_data->handle,
					     &device_status);
	if (xlinkerror == X_LINK_SUCCESS &&
	    device_status == XLINK_DEV_BUSY) {
		xlinkerror =
			xlink_set_device_mode(&vpu_cdev_priv_data->handle,
					      mode);
		if (xlinkerror != X_LINK_SUCCESS) {
			if (!vpu_cdev_priv_data->err_flags.s_err_mode_logged) {
				dev_err(&pdev->dev,
					"xlink_set_device_mode() failed with error %u\n",
					xlinkerror);
				vpu_cdev_priv_data->err_flags.s_err_mode_logged = 1;
			}
			return VPU_COOLING_FAILED;
		}
		vpu_cdev_priv_data->err_flags.s_err_mode_logged = 0;
		vpu_cdev_priv_data->err_flags.s_err_status_logged = 0;
	} else {
		if (!vpu_cdev_priv_data->err_flags.s_err_status_logged) {
			dev_err(&pdev->dev,
				"VPU not started. xlink_get_device_status() returned %u and xlink_device_status=%u\n",
				xlinkerror, device_status);
			vpu_cdev_priv_data->err_flags.s_err_status_logged = 1;
		}
		return VPU_COOLING_FAILED;
	}
	return VPU_COOLING_SUCCESS;
}

static int vpu_cooling_get_cur_state(struct thermal_cooling_device
				      *cooling_dev, unsigned long *state)
{
	struct vpu_cooling_data *vpu_cdev_priv_data = cooling_dev->devdata;
	enum xlink_error xlinkerror = X_LINK_ERROR;
	enum xlink_device_power_mode mode;
	struct platform_device *pdev = vpu_cdev_priv_data->pdev;
	enum xlink_device_status device_status = XLINK_DEV_OFF;

	xlinkerror = xlink_get_device_status(&vpu_cdev_priv_data->handle,
					     &device_status);

	if (xlinkerror == X_LINK_SUCCESS &&
	    device_status == XLINK_DEV_BUSY) {
		xlinkerror = xlink_get_device_mode(&vpu_cdev_priv_data->handle,
						   &mode);

		if (xlinkerror != X_LINK_SUCCESS) {
			if (!vpu_cdev_priv_data->err_flags.g_err_mode_logged) {
				dev_err(&pdev->dev,
					"xlink_get_device_mode() failed with error %d\n",
					xlinkerror);
				vpu_cdev_priv_data->err_flags.g_err_mode_logged = 1;
			}
			return VPU_COOLING_FAILED;
		}
		vpu_cdev_priv_data->err_flags.g_err_mode_logged = 0;
		vpu_cdev_priv_data->err_flags.g_err_status_logged = 0;

	} else {
		if (!vpu_cdev_priv_data->err_flags.g_err_status_logged) {
			dev_err(&pdev->dev,
				"VPU not started. xlink_get_device_status() returned %u and xlink_device_status=%u\n",
				xlinkerror, device_status);
			vpu_cdev_priv_data->err_flags.g_err_status_logged = 1;
		}
		return VPU_COOLING_FAILED;
	}

	/* mode to state mapping is one-to-one */
	switch (mode) {
	case POWER_DEFAULT_NOMINAL_MAX:
	case POWER_SUBNOMINAL_HIGH:
	case POWER_MEDIUM:
	case POWER_LOW:
	case POWER_MIN:
	case POWER_SUSPENDED:
		*state = (unsigned long)mode;
		break;
	default:
		dev_err(&pdev->dev, "mode %u not supported\n",
			(unsigned int)mode);

		return VPU_COOLING_FAILED;
	}
	return VPU_COOLING_SUCCESS;
}

static const struct thermal_cooling_device_ops vpu_cooling_dev_ops = {
	.get_max_state = vpu_cooling_get_max_state,
	.get_cur_state = vpu_cooling_get_cur_state,
	.set_cur_state = vpu_cooling_set_cur_state,

};

static int vpu_cooling_probe(struct platform_device *pdev)
{
	struct vpu_cooling_data *vpu_cdev_priv_data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int rc = VPU_COOLING_SUCCESS;

	vpu_cdev_priv_data = devm_kzalloc(&pdev->dev,
					  sizeof(struct vpu_cooling_data),
					  GFP_KERNEL);

	if (!vpu_cdev_priv_data) {
		dev_err(dev, "vpu_cooling_dev kzalloc failed\n");
		return -ENOMEM;
	}

	rc = of_property_read_u32(dev->of_node, "intel,vpu-ipc-id",
				  &vpu_cdev_priv_data->phy_vpu_id);

	if (rc) {
		dev_err(dev, "VPU ID not defined in Device Tree\n");
		goto r_cleanup;
	}

	vpu_cdev_priv_data->vpu_id = no_of_vpu_devices++;

	vpu_cdev_priv_data->sw_device_id =
		GET_SW_DEVICE_ID_FROM_VPU_ID(vpu_cdev_priv_data->vpu_id);

	vpu_cdev_priv_data->handle.dev_type = HOST_DEVICE;
	vpu_cdev_priv_data->handle.sw_device_id =
		vpu_cdev_priv_data->sw_device_id;

	vpu_cdev_priv_data->pdev = pdev;

	/* Set error flags to 0 */
	vpu_cdev_priv_data->err_flags.s_err_status_logged = 0;
	vpu_cdev_priv_data->err_flags.s_err_mode_logged = 0;
	vpu_cdev_priv_data->err_flags.g_err_status_logged = 0;
	vpu_cdev_priv_data->err_flags.g_err_mode_logged = 0;

	platform_set_drvdata(pdev, vpu_cdev_priv_data);
	vpu_cdev_priv_data->cooling_dev =
		devm_thermal_of_cooling_device_register(dev, np,
							"vpu-cooling",
							vpu_cdev_priv_data,
							&vpu_cooling_dev_ops);
	if (IS_ERR(vpu_cdev_priv_data->cooling_dev)) {
		rc = PTR_ERR(vpu_cdev_priv_data->cooling_dev);
		dev_err(dev,
			"failed to register cooling device to thermal zone %d\n",
			rc);

		return rc;
	}

	dev_info(dev, "VPU cooling device probe success\n");
	return VPU_COOLING_SUCCESS;

r_cleanup:
	dev_err(dev, "VPU cooling device probe failed.\n");

	return rc;
}

static int vpu_cooling_remove(struct platform_device *pdev)
{
	return VPU_COOLING_SUCCESS;
}

static const struct of_device_id vpu_cooling_id_table[] = {
	{ .compatible = "intel,vpu-cooling-dev" },
	{}
};

MODULE_DEVICE_TABLE(of, vpu_cooling_id_table);

static struct platform_driver vpu_cooling_driver = {
	.probe = vpu_cooling_probe,
	.remove = vpu_cooling_remove,
	.driver = {
		.name = "vpu_cooling_drv",
		.of_match_table = vpu_cooling_id_table,
	},
};

module_platform_driver(vpu_cooling_driver);

MODULE_DESCRIPTION("Movidius VPU Cooling Device Driver");
MODULE_AUTHOR("Demakkanavar, Kenchappa <kenchappa.demakkanavar@intel.com>");
MODULE_AUTHOR("S,Kiran Kumar <Kiran.Kumar1.S@intel.com>");
MODULE_LICENSE("GPL v2");
