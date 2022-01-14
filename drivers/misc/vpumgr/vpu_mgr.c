// SPDX-License-Identifier: GPL-2.0-only
/*
 * VPU Manager Kernel module.
 *
 * Copyright (C) 2020-2021 Intel Corporation
 *
 */
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "vpu_common.h"
#include "vpu_smm.h"
#include "vpu_vcm.h"

#define DRIVER_NAME             "vpumgr"
#define MAX_DEV_CNT             32

/* Define the max xlink device number */
#define MAX_SW_DEV_CNT          20

/* Define the SW_DEVICE_ID bit mask and offset */
#define IPC_INTERFACE           0x00
#define PCIE_INTERFACE          0x01
#define MASK_INTERFACE          0x7000000
#define BIT_OFFSET_INTERFACE    24
#define MASK_VPU_IPC_ID         0x000E
#define BIT_OFFSET_VPU_ID       1

#define SWDEVID_INTERFACE(sw_dev_id) (((sw_dev_id) & MASK_INTERFACE) >> BIT_OFFSET_INTERFACE)
#define SWDEVID_VPU_IPC_ID(sw_dev_id) (((sw_dev_id) & MASK_VPU_IPC_ID) >> BIT_OFFSET_VPU_ID)

static dev_t vpumgr_devnum;
static struct class *vpumgr_class;

/**
 * struct vpumgr_fpriv - per-process context stored in FD private data.
 * @vdev: vpumgr device corresponding to the file
 * @smm: memory manager
 * @ctx: vpu context manager
 * @list: for global list of all opened file
 * @pid: process which opens the device file
 */
struct vpumgr_fpriv {
	struct vpumgr_device *vdev;
	struct vpumgr_smm smm;
	struct vpumgr_ctx ctx;
	struct list_head list;
	pid_t  pid;
};

static u32 get_sw_device_id(int vpu_ipc_id)
{
	u32 sw_id_list[MAX_SW_DEV_CNT];
	enum xlink_error rc;
	u32 num = 0;
	u32 swid;
	int i;

	rc = xlink_get_device_list(sw_id_list, &num);
	if (rc) {
		pr_err("XLINK get device list error %d in %s\n", rc, __func__);
		return XLINK_INVALID_SW_DEVID;
	}

	for (i = 0; i < num; i++) {
		swid = sw_id_list[i];
		if (SWDEVID_INTERFACE(swid) == IPC_INTERFACE &&
		    SWDEVID_VPU_IPC_ID(swid) ==  vpu_ipc_id)
			return swid;
	}
	return XLINK_INVALID_SW_DEVID;
}

static int vpumgr_open(struct inode *inode, struct file *filp)
{
	struct vpumgr_fpriv *vpriv;
	struct vpumgr_device *vdev;
	int rc;

	vpriv = kzalloc(sizeof(*vpriv), GFP_KERNEL);
	if (!vpriv)
		return -ENOMEM;

	vdev = container_of(inode->i_cdev, struct vpumgr_device, cdev);
	rc = smm_open(&vpriv->smm, vdev);
	if (rc)
		goto free_priv;

	rc = vcm_open(&vpriv->ctx, vdev);
	if (rc)
		goto close_smm;

	vpriv->vdev = vdev;
	vpriv->pid = task_pid_nr(current);
	INIT_LIST_HEAD(&vpriv->list);

	mutex_lock(&vdev->client_mutex);
	list_add_tail(&vpriv->list, &vdev->client_list);
	mutex_unlock(&vdev->client_mutex);

	filp->private_data = vpriv;
	return 0;

close_smm:
	smm_close(&vpriv->smm);
free_priv:
	kfree(vpriv);
	return rc;
}

static int vpumgr_release(struct inode *inode, struct file *filp)
{
	struct vpumgr_fpriv *vpriv = filp->private_data;
	struct vpumgr_device *vdev = container_of(inode->i_cdev, struct vpumgr_device, cdev);

	vcm_close(&vpriv->ctx);
	smm_close(&vpriv->smm);

	mutex_lock(&vdev->client_mutex);
	list_del(&vpriv->list);
	mutex_unlock(&vdev->client_mutex);

	kfree(vpriv);
	filp->private_data = NULL;
	return 0;
}

static long vpumgr_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct vpumgr_fpriv *vpriv = filp->private_data;
	const unsigned int io_dir = _IOC_DIR(cmd);
	const unsigned int io_size = _IOC_SIZE(cmd);
	struct vpumgr_vcm_submit *vs;
	struct vpumgr_vcm_wait *vw;
	char tmp[128];
	int rc = 0;

	if (_IOC_TYPE(cmd) != VPUMGR_MAGIC || _IOC_NR(cmd) >= _IOC_NR(VPUMGR_IOCTL_END))
		return -EINVAL;

	if (io_size > sizeof(tmp))
		return -EFAULT;

	if (io_dir & _IOC_READ) {
		if (copy_from_user(tmp, (void __user *)arg, io_size) != 0)
			return  -EFAULT;
	}

	switch (cmd) {
	case VPUMGR_IOCTL_DMABUF_ALLOC:
		rc = smm_alloc(&vpriv->smm, (void *)tmp);
		break;
	case VPUMGR_IOCTL_DMABUF_IMPORT:
		rc = smm_import(&vpriv->smm, (void *)tmp);
		break;
	case VPUMGR_IOCTL_DMABUF_UNIMPORT:
		rc = smm_unimport(&vpriv->smm, (void *)tmp);
		break;
	case VPUMGR_IOCTL_DMABUF_PTR2VPU:
		rc = smm_ptr2vpu(&vpriv->smm, (void *)tmp);
		break;
	case VPUMGR_IOCTL_VCM_SUBMIT:
		vs = (struct vpumgr_vcm_submit *)tmp;
		if (vs->cmd <= VCTX_KMD_RESERVED_CMD_LAST) {
			/*
			 * user-space can talk to vpu context lives in firmware
			 * with any commands other than those reserved for kernel
			 * mode.
			 */
			rc = -EACCES;
			break;
		}
		rc = vcm_submit(&vpriv->ctx, vs->cmd,
				(const void *)vs->in, vs->in_len, &vs->submit_id);
		break;
	case VPUMGR_IOCTL_VCM_WAIT:
		vw = (struct vpumgr_vcm_wait *)tmp;
		rc = vcm_wait(&vpriv->ctx, vw->submit_id, &vw->vpu_rc,
			      (void *)vw->out, &vw->out_len, vw->timeout_ms);
		break;
	}

	if (!rc) {
		if (io_dir & _IOC_WRITE) {
			if (copy_to_user((void __user *)arg, tmp, io_size) != 0)
				return -EFAULT;
		}
	}
	return rc;
}

static const struct file_operations vpumgr_devfile_fops = {
	.owner = THIS_MODULE,
	.open = vpumgr_open,
	.release = vpumgr_release,
	.unlocked_ioctl = vpumgr_ioctl,
};

static int vpumgr_debugfs_stats_show(struct seq_file *file, void *offset)
{
	struct vpumgr_device *vdev = dev_get_drvdata(file->private);
	struct vpumgr_fpriv *fpriv;
	int i = 0;

	mutex_lock(&vdev->client_mutex);
	list_for_each_entry(fpriv, &vdev->client_list, list) {
		seq_printf(file, "client #%d pid:%d\n", i++, fpriv->pid);
		vcm_debugfs_stats_show(file, &fpriv->ctx);
		smm_debugfs_stats_show(file, &fpriv->smm);
	}
	mutex_unlock(&vdev->client_mutex);
	return 0;
}

static ssize_t fwname_show(struct device *device,
			   struct device_attribute *attr,
			   char *buf)
{
	struct vpumgr_device *vdev = dev_get_drvdata(device);

	return scnprintf(buf, PAGE_SIZE, "%s", vdev->fwname);
}

static ssize_t fwname_store(struct device *device,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct vpumgr_device *vdev = dev_get_drvdata(device);
	int i;

	if (count > sizeof(vdev->fwname) - 1)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		if (buf[i] == '\r' || buf[i] == '\n')
			break;
		vdev->fwname[i] = buf[i];
	}
	vdev->fwname[i] = '\0';

	return count;
}

static const char *default_fwname = "vpu_nvr.bin";
static struct device_attribute fwname_attr = __ATTR_RW(fwname);

static const struct of_device_id keembay_vpumgr_of_match[] = {
	{ .compatible = "intel,keembay-vpu-mgr"},
	{ .compatible = "intel,keembay-vpusmm"},
	{}
};
MODULE_DEVICE_TABLE(of, keembay_vpumgr_of_match);

static int vpumgr_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vpumgr_device *vdev;
	u32 ipc_sw_device_id;
	u32 vpu_ipc_id = 0;
	int rc;

	/* get device id */
	rc = of_property_read_u32(dev->of_node, "intel,keembay-vpu-ipc-id",
				  &vpu_ipc_id);
	if (rc && rc != -EINVAL) {
		dev_err(dev, "%s: vpu-ipc-id read failed with rc %d\n", __func__, rc);
		return -EINVAL;
	}

	ipc_sw_device_id = get_sw_device_id(vpu_ipc_id);
	if (ipc_sw_device_id == XLINK_INVALID_SW_DEVID)
		dev_warn(dev, "%s: no xlink sw device for vpu_ipc_id %d\n",
			 __func__, vpu_ipc_id);

	vdev = devm_kzalloc(dev, sizeof(struct vpumgr_device), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

	vdev->devnum = MKDEV(MAJOR(vpumgr_devnum), vpu_ipc_id);
	vdev->pdev = pdev;
	vdev->dev = dev;
	scnprintf(vdev->fwname, sizeof(vdev->fwname), "%s", default_fwname);

	dev_dbg(dev, "dev->devnum %u, id %u, major %u\n",
		vdev->devnum, vpu_ipc_id,  MAJOR(vdev->devnum));

	vdev->sdev = device_create(vpumgr_class, dev, vdev->devnum,
				   NULL, DRIVER_NAME "%d", vpu_ipc_id);
	if (IS_ERR(vdev->sdev)) {
		dev_err(dev, "%s: device_create failed\n", __func__);
		return PTR_ERR(vdev->sdev);
	}

	cdev_init(&vdev->cdev, &vpumgr_devfile_fops);
	vdev->cdev.owner = THIS_MODULE;
	rc = cdev_add(&vdev->cdev, vdev->devnum, 1);
	if (rc) {
		dev_err(dev, "%s: cdev_add failed.\n", __func__);
		goto detroy_device;
	}

	vdev->debugfs_root = debugfs_create_dir(dev_name(vdev->sdev), NULL);

	debugfs_create_devm_seqfile(dev, "stats", vdev->debugfs_root,
				    vpumgr_debugfs_stats_show);

	rc = device_create_file(dev, &fwname_attr);
	if (rc)
		goto remove_debugfs;

	rc = smm_init(vdev);
	if (rc)
		goto remove_sysfs;

	rc = vcm_init(vdev, ipc_sw_device_id);
	if (rc)
		goto fini_smm;

	INIT_LIST_HEAD(&vdev->client_list);
	mutex_init(&vdev->client_mutex);

	dev_set_drvdata(dev, vdev);
	return 0;

fini_smm:
	smm_fini(vdev);
remove_sysfs:
	device_remove_file(dev, &fwname_attr);
remove_debugfs:
	debugfs_remove_recursive(vdev->debugfs_root);
	cdev_del(&vdev->cdev);
detroy_device:
	device_destroy(vpumgr_class, vdev->devnum);
	return rc;
}

static int vpumgr_driver_remove(struct platform_device *pdev)
{
	struct vpumgr_device *vdev = dev_get_drvdata(&pdev->dev);

	mutex_destroy(&vdev->client_mutex);
	vcm_fini(vdev);
	smm_fini(vdev);
	device_remove_file(vdev->dev, &fwname_attr);
	debugfs_remove_recursive(vdev->debugfs_root);
	cdev_del(&vdev->cdev);
	device_destroy(vpumgr_class, vdev->devnum);
	return 0;
}

static struct platform_driver vpumgr_driver = {
	.probe  = vpumgr_driver_probe,
	.remove = vpumgr_driver_remove,
	.driver = {
			.owner = THIS_MODULE,
			.name = "keembay-vpu-mgr",
			.of_match_table = keembay_vpumgr_of_match,
	 },
};

static int __init vpumgr_init(void)
{
	int rc;

	rc = alloc_chrdev_region(&vpumgr_devnum, 0, MAX_DEV_CNT, DRIVER_NAME);
	if (rc < 0) {
		pr_err("[%s] err: alloc_chrdev_region\n", __func__);
		return rc;
	}

	vpumgr_class = class_create(THIS_MODULE, DRIVER_NAME "_class");
	if (IS_ERR(vpumgr_class)) {
		rc = PTR_ERR(vpumgr_class);
		pr_err("[%s] err: class_create\n", __func__);
		goto unreg_chrdev;
	}

	rc = platform_driver_register(&vpumgr_driver);
	if (rc) {
		pr_err("[%s] err platform_driver_register\n", __func__);
		goto destroy_class;
	}

	return 0;

destroy_class:
	class_destroy(vpumgr_class);
unreg_chrdev:
	unregister_chrdev_region(vpumgr_devnum, MAX_DEV_CNT);
	return rc;
}

static void vpumgr_exit(void)
{
	platform_driver_unregister(&vpumgr_driver);
	class_destroy(vpumgr_class);
	unregister_chrdev_region(vpumgr_devnum, MAX_DEV_CNT);
}

module_init(vpumgr_init)
module_exit(vpumgr_exit)

MODULE_DESCRIPTION("VPU resource manager driver");
MODULE_AUTHOR("Tingqian Li <tingqian.li@intel.com>");
MODULE_AUTHOR("Luwei Zhou <luwie.zhou@intel.com>");
MODULE_LICENSE("GPL");
