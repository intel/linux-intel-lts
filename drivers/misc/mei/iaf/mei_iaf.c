// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2020-2021 Intel Corporation
 */

/**
 * DOC: MEI_IAF Client Driver
 *
 * The IAF (Intel Accelerator Fabric) component driver acts as an interface
 * between IAF i915 driver and GSC. The only api this interface provides is
 * the 'commit svn' call.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uuid.h>
#include <linux/mei_cl_bus.h>
#include <linux/component.h>
#include <drm/drm_connector.h>
#include <drm/intel/i915_component.h>
#include <drm/i915_mei_iaf_interface.h>

#include "mkhi.h"

#define MCA_ARBSVN_COMMIT_COMMAND_ID 0x1B

enum arbsvn_nvar_usage {
	ARBSVN_NVAR_USAGE_FW_MIN_VER = 0,
	ARBSVN_NVAR_USAGE_MAX
};

struct mca_arbsvn_commit_req {
	struct mkhi_msg_hdr mkhi_header;
	u8                  usage_id;
	u8                  reserved0;
	u16                 reserved1;
} __packed;

struct mca_arbsvn_commit_resp {
	struct mkhi_msg_hdr mkhi_header;
};

#define MCA_OK               0x0  /* on successful commit */
#define MCA_INVALID_INPUT    0xb  /* if usage id is invalid */
/* if disabled in the file or any other error (generic, reading or writing file) */
#define MCA_ARB_SVN_DISABLED 0x20
/* SVN was not updated, same value */
#define MCA_ARB_SVN_SAME     0x28
/* SVN was not updated, older value */
#define MCA_ARB_SVN_SMALLER  0x29

static int get_error_code(const struct device *dev, u8 result)
{
	int ret;

	switch (result) {
	case MCA_OK:
		ret = 0;
		break;
	case MCA_ARB_SVN_DISABLED:
		dev_dbg(dev, "Arb Svn disabled (error code 0x%x)\n",
			MCA_ARB_SVN_DISABLED);
		ret = -ENOENT;
	break;
	case MCA_INVALID_INPUT:
		dev_err(dev, "Wrong usage id(error code 0x%x)\n",
			MCA_INVALID_INPUT);
		ret = -EINVAL;
	break;
	case MCA_ARB_SVN_SAME:
		dev_dbg(dev, "SVN was not updated, same value(error code 0x%x)\n",
			MCA_ARB_SVN_SAME);
		ret = -EACCES;
	break;
	case MCA_ARB_SVN_SMALLER:
		dev_dbg(dev, "SVN was not updated, older value(error code 0x%x)\n",
			MCA_ARB_SVN_SMALLER);
		ret = -EBADF;
	break;
	default:
		dev_err(dev, "Unknown error code 0x%x\n", result);
		ret = -EIO;
	}

	return ret;
}

static int mei_iaf_check_response(const struct device *dev,
				  struct mkhi_msg_hdr *hdr)
{
	if (hdr->group_id != MCHI_GROUP_ID) {
		dev_err(dev, "Mismatch group id: 0x%x instead of 0x%x\n",
			hdr->group_id, MCHI_GROUP_ID);
		return -EINVAL;
	}

	if (hdr->command != (MCA_ARBSVN_COMMIT_COMMAND_ID | 0x80)) {
		dev_err(dev, "Mismatch command: 0x%x instead of 0x%x\n",
			hdr->command, MCA_ARBSVN_COMMIT_COMMAND_ID | 0x80);
		return -EINVAL;
	}

	return 0;
}

/**
 * mei_iaf_commit_svn() - Commits current SVN.
 * @dev: device corresponding to the mei_cl_device
 * Return: 0 on Success
 * *  -EINVAL : Invalid usage id parameter
 * *  -ENOENT : ARB SVN is disabled in the file or any other error
 *             (generic, reading or writing file)
 * *  -EIO    : Unknown I/O error
 */
static int mei_iaf_commit_svn(const struct device *dev)
{
	struct mei_cl_device *cldev;
	struct mca_arbsvn_commit_req commit_req = { };
	struct mca_arbsvn_commit_resp commit_resp = { };
	int ret;

	dev_dbg(dev, "in %s\n", __func__);

	if (!dev)
		return -EINVAL;

	cldev = to_mei_cl_device(dev);

	dev_dbg(dev, "after to_mei_cl_device cldev %p\n", cldev);

	ret = mei_cldev_enable(cldev);
	if (ret < 0) {
		dev_dbg(dev, "mei_cldev_enable Failed. %d\n", ret);
		return -EBUSY;
	}

	dev_dbg(dev, "after mei_cldev_enable, ret=%d\n", ret);
	commit_req.mkhi_header.group_id = MCHI_GROUP_ID;
	commit_req.mkhi_header.command = MCA_ARBSVN_COMMIT_COMMAND_ID;
	commit_req.usage_id = ARBSVN_NVAR_USAGE_FW_MIN_VER;

	ret = mei_cldev_send(cldev, (u8 *)&commit_req, sizeof(commit_req));
	if (ret < 0) {
		dev_err(dev, "mei_cldev_send failed. %d\n", ret);
		goto end;
	}
	dev_dbg(dev, "after send, ret=%d\n", ret);
	print_hex_dump_debug("sent svn commit message: ", DUMP_PREFIX_OFFSET,
			     16, 1, (u8 *)&commit_req, ret, false);

	ret = mei_cldev_recv(cldev, (u8 *)&commit_resp, sizeof(commit_resp));
	if (ret < 0) {
		dev_err(dev, "mei_cldev_recv failed. %d\n", ret);
		goto end;
	}
	dev_dbg(dev, "after recv, ret=%d\n", ret);
	print_hex_dump_debug("mei_iaf_commit_response ", DUMP_PREFIX_OFFSET,
			     16, 1, (u8 *)&commit_resp, ret, false);

	ret = mei_iaf_check_response(dev, &commit_resp.mkhi_header);
	if (ret) {
		dev_err(dev, "bad result response from the firmware: 0x%x\n",
			*(uint32_t *)&commit_resp.mkhi_header);
		goto end;
	}
	dev_dbg(dev, "after check_response\n");
	ret = get_error_code(dev, commit_resp.mkhi_header.result);

end:
	dev_dbg(dev, "returning with %d\n", ret);
	mei_cldev_disable(cldev);
	return ret;
}

static const struct i915_iaf_component_ops mei_iaf_ops = {
	.owner = THIS_MODULE,
	.commit_svn = mei_iaf_commit_svn,
};

static int mei_component_master_bind(struct device *dev)
{
	int ret;

	dev_dbg(dev, "mei_iaf_ops addr %p\n", &mei_iaf_ops);

	ret = component_bind_all(dev, (void *)&mei_iaf_ops);
	if (ret < 0)
		return ret;

	return 0;
}

static void mei_component_master_unbind(struct device *dev)
{
	dev_dbg(dev, "in %s\n", __func__);
	component_unbind_all(dev, (void *)&mei_iaf_ops);
}

static const struct component_master_ops mei_component_master_ops = {
	.bind = mei_component_master_bind,
	.unbind = mei_component_master_unbind,
};

/**
 * mei_iaf_component_match - compare function for matching mei iaf.
 *
 *    The function checks if the driver is i915, the subcomponent is IAF
 *    and the parent of iaf and the grand parent of mei_if are the same
 *    i915 device.
 *
 * @dev: master device
 * @subcomponent: subcomponent to match (I915_COMPONENT_IAF)
 * @data: compare data (mei iaf device)
 *
 * Return:
 * * 1 - if components match
 * * 0 - otherwise
 */
static int mei_iaf_component_match(struct device *dev, int subcomponent,
				   void *data)
{
	struct device *base = data;

	dev_dbg(dev, "trying to match %s\n", dev->driver->name);
	if (subcomponent != I915_COMPONENT_IAF)
		return 0;

	if (strcmp(dev->driver->name, "iaf"))
		return 0;

	base = base->parent;
	if (!base)
		return 0;

	base = base->parent;
	dev = dev->parent;

	return (base && dev && dev == base);
}

static int mei_iaf_probe(struct mei_cl_device *cldev,
			 const struct mei_cl_device_id *id)
{
	struct component_match *master_match;
	int ret;

	master_match = NULL;
	component_match_add_typed(&cldev->dev, &master_match,
				  mei_iaf_component_match, &cldev->dev);
	if (IS_ERR_OR_NULL(master_match)) {
		ret = -ENOMEM;
		goto err_exit;
	}

	ret = component_master_add_with_match(&cldev->dev,
					      &mei_component_master_ops,
					      master_match);
	if (ret < 0) {
		dev_err(&cldev->dev, "Master comp add failed %d\n", ret);
		goto err_exit;
	}

	return 0;

err_exit:
	return ret;
}

static void mei_iaf_remove(struct mei_cl_device *cldev)
{
	component_master_del(&cldev->dev, &mei_component_master_ops);
}

/* fe2af7a6-ef22-4b45-872f-176b0bbc8b43: MCHIF GUID */
#define MEI_GUID_MCHIF UUID_LE(0xfe2af7a6, 0xef22, 0x4b45, \
			       0x87, 0x2f, 0x17, 0x6b, 0x0b, 0xbc, 0x8b, 0x43)

static struct mei_cl_device_id mei_iaf_tbl[] = {
	{ .uuid = MEI_GUID_MCHIF, .version = MEI_CL_VERSION_ANY },
	{ }
};
MODULE_DEVICE_TABLE(mei, mei_iaf_tbl);

static struct mei_cl_driver mei_iaf_driver = {
	.id_table = mei_iaf_tbl,
	.name = KBUILD_MODNAME,
	.probe = mei_iaf_probe,
	.remove	= mei_iaf_remove,
};

module_mei_cl_driver(mei_iaf_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MEI IAF");
