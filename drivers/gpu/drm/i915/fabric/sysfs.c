// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2022 Intel Corporation.
 *
 */

#include <linux/device.h>
#include "csr.h"
#include "fw.h"
#include "iaf_drv.h"
#include "mbdb.h"
#include "mei_iaf_user.h"
#include "ops.h"
#include "port.h"
#include "sysfs.h"

static ssize_t link_failures_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fport *port;
	struct fport_status status;

	port = container_of(attr, struct fport, link_failures);

	get_fport_status(port->sd, port->lpn, &status);

	return sysfs_emit(buf, "%llu\n", status.link_failures);
}

static ssize_t link_degrades_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fport *port;
	struct fport_status status;

	port = container_of(attr, struct fport, link_degrades);

	get_fport_status(port->sd, port->lpn, &status);

	return sysfs_emit(buf, "%llu\n", status.link_degrades);
}

static ssize_t fw_comm_errors_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsubdev *sd;

	sd = container_of(attr, struct fsubdev, fw_comm_errors);

	return sysfs_emit(buf, "%llu\n", mbdb_get_mbox_comm_errors(sd));
}

static ssize_t fw_error_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsubdev *sd;

	sd = container_of(attr, struct fsubdev, fw_error);

	return sysfs_emit(buf, "%u\n", test_bit(SD_ERROR_FW, sd->errors));
}

static ssize_t sd_failure_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsubdev *sd;

	sd = container_of(attr, struct fsubdev, sd_failure);

	return sysfs_emit(buf, "%u\n", test_bit(SD_ERROR_FAILED, sd->errors));
}

static ssize_t firmware_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mbdb_op_fw_version_rsp fw_version = {};
	struct fsubdev *sd;
	int err;

	sd = container_of(attr, struct fsubdev, firmware_version);

	err = ops_fw_version(sd, &fw_version);
	if (err == MBOX_RSP_STATUS_SEQ_NO_ERROR)
		err = ops_fw_version(sd, &fw_version);
	if (err) {
		sd_err(sd, "unable to query firmware version\n");
		return -EIO;
	}

	return sysfs_emit(buf, "%.*s\n", (int)sizeof(fw_version.fw_version_string),
			  (fw_version.environment & FW_VERSION_ENV_BIT) ?
			  (char *)fw_version.fw_version_string : "UNKNOWN");
}

static ssize_t iaf_fabric_id_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct fdev *fdev = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%x\n", fdev->fabric_id);
}

static DEVICE_ATTR_RO(iaf_fabric_id);

static ssize_t pscbin_brand_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fdev *fdev = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", fdev->psc.brand ? fdev->psc.brand :
			  "UNKNOWN");
}

static ssize_t pscbin_product_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct fdev *fdev = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", fdev->psc.product ? fdev->psc.product :
			  "UNKNOWN");
}

static ssize_t pscbin_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct fdev *fdev = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", fdev->psc.version[0] ?
			  fdev->psc.version : "UNKNOWN");
}

static DEVICE_ATTR_RO(pscbin_brand);
static DEVICE_ATTR_RO(pscbin_product);
static DEVICE_ATTR_RO(pscbin_version);

static ssize_t min_svn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fdev *fdev = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", get_min_svn(fdev));
}

static DEVICE_ATTR_ADMIN_RO(min_svn);

/**
 * prevent_rollback_store - Initiate anti-rollback protection
 * @dev: device from sysfs call
 * @attr: device attribute from sysfs call
 * @buf: buffer pointer from sysfs call
 * @count: buffer count from sysfs call
 *
 * cause automatic rollback protection to be initiated and absorb whatever was
 * written to the prevent_rollback device attribute: if automatic rollback
 * protection is enabled, it is triggered after the device is successfully
 * initialized; otherwise, it is only triggered by writing (any data) to the
 * device's prevent_rollback sysfs entry (it is safe to do both)
 *
 * Return: @count on success, -EACCESS on failure
 */
static ssize_t prevent_rollback_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct fdev *fdev = dev_get_drvdata(dev);
	int err;

	err = iaf_commit_svn(fdev);

	if (err)
		return -EACCES;

	return count;
}

static DEVICE_ATTR_WO(prevent_rollback);

static const struct attribute *iaf_attrs[] = {
	&dev_attr_iaf_fabric_id.attr,
	&dev_attr_prevent_rollback.attr,
	&dev_attr_min_svn.attr,
	&dev_attr_pscbin_brand.attr,
	&dev_attr_pscbin_product.attr,
	&dev_attr_pscbin_version.attr,
	NULL,
};

static void iaf_sysfs_cleanup(struct fsubdev *sd)
{
	struct fport *port;
	u8 lpn;

	for_each_fabric_port(port, lpn, sd) {
		kobject_put(port->kobj);
		port->kobj = NULL;
	}

	kobject_put(sd->kobj);
	sd->kobj = NULL;
}

typedef ssize_t (*show_fn)(struct device *dev, struct device_attribute *attr, char *buf);

static int iaf_sysfs_add_node(const char *name, umode_t mode, show_fn show,
			      struct device_attribute *attr, struct kobject *parent)
{
	sysfs_attr_init(&attr->attr);
	attr->attr.name = name;
	attr->attr.mode = mode;
	attr->show = show;

	return sysfs_create_file(parent, &attr->attr);
}

static int iaf_sysfs_add_port_nodes(struct fsubdev *sd)
{
	struct fport *port;
	u8 lpn;

	for_each_fabric_port(port, lpn, sd) {
		int err;
		char port_name[9];

		snprintf(port_name, sizeof(port_name), "port.%u", lpn);

		port->kobj = kobject_create_and_add(port_name, sd->kobj);
		if (!port->kobj)
			return -ENOMEM;

		err = iaf_sysfs_add_node("link_failures", 0400, link_failures_show,
					 &port->link_failures, port->kobj);
		if (err) {
			sd_warn(sd, "Failed to add sysfs node %s for port %s\n", "link_failures",
				port_name);
			return err;
		}

		err = iaf_sysfs_add_node("link_degrades", 0400, link_degrades_show,
					 &port->link_degrades, port->kobj);
		if (err) {
			sd_warn(sd, "Failed to add sysfs node %s for port %s\n", "link_degrades",
				port_name);
			return err;
		}
	}

	return 0;
}

static int iaf_sysfs_add_sd_nodes(struct fsubdev *sd)
{
	int err;

	err = iaf_sysfs_add_node("fw_comm_errors", 0400, fw_comm_errors_show,
				 &sd->fw_comm_errors, sd->kobj);
	if (err) {
		sd_warn(sd, "Failed to add sysfs node %s for %s\n", "fw_comm_errors", sd->name);
		goto exit;
	}

	err = iaf_sysfs_add_node("fw_error", 0400, fw_error_show, &sd->fw_error, sd->kobj);
	if (err) {
		sd_warn(sd, "Failed to add sysfs node %s for %s\n", "fw_error", sd->name);
		goto exit;
	}

	err = iaf_sysfs_add_node("sd_failure", 0400, sd_failure_show, &sd->sd_failure, sd->kobj);
	if (err) {
		sd_warn(sd, "Failed to add sysfs node %s for %s\n", "sd_failure", sd->name);
		goto exit;
	}

	err = iaf_sysfs_add_node("firmware_version", 0444, firmware_version_show,
				 &sd->firmware_version, sd->kobj);
	if (err)
		sd_warn(sd, "Failed to add sysfs node %s for %s\n", "firmware_version", sd->name);

exit:
	return err;
}

static void iaf_sysfs_sd_init(struct fsubdev *sd)
{
	int err;

	sd->kobj = kobject_create_and_add(sd->name, &sd->fdev->pdev->dev.kobj);
	if (!sd->kobj) {
		sd_warn(sd, "Failed to add sysfs directory %s\n", sd->name);
		return;
	}

	err = iaf_sysfs_add_port_nodes(sd);
	if (err)
		goto error_return;

	err = iaf_sysfs_add_sd_nodes(sd);
	if (err)
		goto error_return;

	return;

error_return:
	iaf_sysfs_cleanup(sd);
}

void iaf_sysfs_init(struct fdev *fdev)
{
	u8 i;

	for (i = 0; i < fdev->pd->sd_cnt; i++)
		iaf_sysfs_sd_init(&fdev->sd[i]);
}

void iaf_sysfs_remove(struct fdev *fdev)
{
	u8 i;

	for (i = 0; i < fdev->pd->sd_cnt; i++)
		iaf_sysfs_cleanup(&fdev->sd[i]);

	sysfs_remove_files(&fdev->pdev->dev.kobj, iaf_attrs);
}

int iaf_sysfs_probe(struct fdev *fdev)
{
	int err;

	err = sysfs_create_files(&fdev->pdev->dev.kobj, iaf_attrs);
	if (err) {
		dev_err(&fdev->pdev->dev, "Failed to add sysfs\n");
		return err;
	}
	return 0;
}
