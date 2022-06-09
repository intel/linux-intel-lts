// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2023 Intel Corporation
 */

#include <linux/anon_inodes.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/vfio.h>

#include "i915_vfio_pci.h"

static void i915_vfio_pci_reset_done(struct pci_dev *pdev)
{
	struct i915_vfio_pci_core_device *i915_vdev = pci_get_drvdata(pdev);
	int ret;

	ret = i915_vdev->pf_ops->wait_flr_done(i915_vdev->pf, i915_vdev->vfid);
	if (ret)
		dev_err(&pdev->dev, "Failed to wait for FLR: %d\n", ret);

	i915_vfio_pci_reset(i915_vdev);
}

static const struct pci_error_handlers i915_vfio_pci_err_handlers = {
	.reset_done = &i915_vfio_pci_reset_done,
};

static int i915_vfio_pci_open_device(struct vfio_device *core_vdev)
{
	struct i915_vfio_pci_core_device *i915_vdev =
		container_of(core_vdev, struct i915_vfio_pci_core_device, core_device.vdev);
	struct vfio_pci_core_device *vdev = &i915_vdev->core_device;
	int ret;

	ret = vfio_pci_core_enable(vdev);
	if (ret)
		return ret;

	vfio_pci_core_finish_enable(vdev);

	return 0;
}

static void i915_vfio_pci_disable_file(struct i915_vfio_pci_migration_file *migf)
{
	struct i915_vfio_pci_core_device *i915_vdev = migf->i915_vdev;

	mutex_lock(&migf->lock);
	i915_vfio_save_data_release(i915_vdev->fd);
	i915_vdev->fd = NULL;
	mutex_unlock(&migf->lock);
}

static int i915_vfio_pci_release_file(struct inode *inode, struct file *filp)
{
	struct i915_vfio_pci_migration_file *migf = filp->private_data;

	i915_vfio_pci_disable_file(migf);
	mutex_destroy(&migf->lock);
	kfree(migf);

	return 0;
}

static ssize_t i915_vfio_pci_save_read(struct file *filp, char __user *buf, size_t len, loff_t *pos)
{
	struct i915_vfio_pci_migration_file *migf = filp->private_data;
	ssize_t ret;

	if (pos)
		return -ESPIPE;

	mutex_lock(&migf->lock);
	ret = i915_vfio_data_read(migf, buf, len);
	mutex_unlock(&migf->lock);

	return ret;
}

static const struct file_operations i915_vfio_pci_save_fops = {
	.owner = THIS_MODULE,
	.read = i915_vfio_pci_save_read,
	.release = i915_vfio_pci_release_file,
	.llseek = no_llseek,
};

static ssize_t i915_vfio_pci_resume_write(struct file *filp, const char __user *buf,
					  size_t len, loff_t *pos)
{
	struct i915_vfio_pci_migration_file *migf = filp->private_data;
	ssize_t ret;

	if (pos)
		return -ESPIPE;

	mutex_lock(&migf->lock);
	ret = i915_vfio_data_write(migf, buf, len);
	mutex_unlock(&migf->lock);

	return ret;
}

static const struct file_operations i915_vfio_pci_resume_fops = {
	.owner = THIS_MODULE,
	.write = i915_vfio_pci_resume_write,
	.release = i915_vfio_pci_release_file,
	.llseek = no_llseek,
};

void i915_vfio_pci_reset(struct i915_vfio_pci_core_device *i915_vdev)
{
	if (i915_vdev->fd)
		i915_vfio_pci_disable_file(i915_vdev->fd);

	i915_vdev->mig_state = VFIO_DEVICE_STATE_RUNNING;
}

static const char *i915_vfio_dev_state_str(u32 state)
{
	switch (state) {
	case VFIO_DEVICE_STATE_RUNNING: return "running";
	case VFIO_DEVICE_STATE_STOP_COPY: return "stopcopy";
	case VFIO_DEVICE_STATE_STOP: return "stop";
	case VFIO_DEVICE_STATE_RESUMING: return "resuming";
	case VFIO_DEVICE_STATE_ERROR: return "error";
	default: return "";
	}
}

enum i915_vfio_pci_file_type {
	I915_VFIO_FILE_SAVE = 0,
	I915_VFIO_FILE_RESUME,
};

static struct i915_vfio_pci_migration_file *
i915_vfio_pci_alloc_file(struct i915_vfio_pci_core_device *i915_vdev,
			 enum i915_vfio_pci_file_type type)
{
	struct i915_vfio_pci_migration_file *migf;
	const struct file_operations *fops;
	int flags;

	migf = kzalloc(sizeof(*migf), GFP_KERNEL);
	if (!migf)
		return ERR_PTR(-ENOMEM);

	fops = type == I915_VFIO_FILE_SAVE ? &i915_vfio_pci_save_fops : &i915_vfio_pci_resume_fops;
	flags = type == I915_VFIO_FILE_SAVE ? O_RDONLY : O_WRONLY;
	migf->filp = anon_inode_getfile("i915_vfio_mig", fops, migf, flags);
	if (IS_ERR(migf->filp)) {
		kfree(migf);
		return ERR_CAST(migf->filp);
	}

	INIT_LIST_HEAD(&migf->save_data);
	mutex_init(&migf->lock);
	migf->i915_vdev = i915_vdev;
	migf->copy_from = copy_from_user;
	migf->copy_to = copy_to_user;
	i915_vdev->fd = migf;

	stream_open(migf->filp->f_inode, migf->filp);

	return migf;
}

static struct file *
i915_vfio_set_state(struct i915_vfio_pci_core_device *i915_vdev, u32 new)
{
	const struct i915_vfio_pci_migration_pf_ops *ops = i915_vdev->pf_ops;
	u32 cur = i915_vdev->mig_state;
	int ret;

	dev_dbg(i915_vdev_to_dev(i915_vdev),
		"state: %s->%s\n", i915_vfio_dev_state_str(cur), i915_vfio_dev_state_str(new));

	if (cur == VFIO_DEVICE_STATE_RUNNING && new == VFIO_DEVICE_STATE_STOP) {
		ret = ops->pause(i915_vdev->pf, i915_vdev->vfid);
		if (ret) {
			dev_dbg(i915_vdev_to_dev(i915_vdev),
				"Failed to transition state: %s->%s err=%d\n",
				i915_vfio_dev_state_str(cur), i915_vfio_dev_state_str(new), ret);
			return ERR_PTR(ret);
		}
		return NULL;
	}

	if (cur == VFIO_DEVICE_STATE_STOP && new == VFIO_DEVICE_STATE_RUNNING) {
		ret = ops->resume(i915_vdev->pf, i915_vdev->vfid);
		if (ret) {
			dev_dbg(i915_vdev_to_dev(i915_vdev),
				"Failed to transition state: %s->%s err=%d\n",
				i915_vfio_dev_state_str(cur), i915_vfio_dev_state_str(new), ret);
			return ERR_PTR(ret);
		}
		return NULL;
	}

	if (cur == VFIO_DEVICE_STATE_STOP && new == VFIO_DEVICE_STATE_STOP_COPY) {
		struct i915_vfio_pci_migration_file *migf;
		int ret;

		migf = i915_vfio_pci_alloc_file(i915_vdev, I915_VFIO_FILE_SAVE);
		if (IS_ERR(migf))
			return ERR_CAST(migf);

		ret = i915_vfio_pci_produce_save_data(migf);
		if (ret) {
			fput(migf->filp);
			return ERR_PTR(ret);
		}

		return migf->filp;
	}

	if ((cur == VFIO_DEVICE_STATE_STOP_COPY && new == VFIO_DEVICE_STATE_STOP)) {
		if (i915_vdev->fd)
			i915_vfio_pci_disable_file(i915_vdev->fd);

		return NULL;
	}

	if (cur == VFIO_DEVICE_STATE_STOP && new == VFIO_DEVICE_STATE_RESUMING) {
		struct i915_vfio_pci_migration_file *migf;

		migf = i915_vfio_pci_alloc_file(i915_vdev, I915_VFIO_FILE_RESUME);
		if (IS_ERR(migf))
			return ERR_CAST(migf);

		return migf->filp;
	}

	if (cur == VFIO_DEVICE_STATE_RESUMING && new == VFIO_DEVICE_STATE_STOP) {
		if (i915_vdev->fd)
			i915_vfio_pci_disable_file(i915_vdev->fd);

		return NULL;
	}

	WARN_ON(true);
	return ERR_PTR(-EINVAL);
}

static struct file *
i915_vfio_pci_set_device_state(struct vfio_device *core_vdev,
			       enum vfio_device_mig_state new_state)
{
	struct i915_vfio_pci_core_device *i915_vdev =
		container_of(core_vdev, struct i915_vfio_pci_core_device, core_device.vdev);
	enum vfio_device_mig_state next_state;
	struct file *f = NULL;
	int ret;

	while (new_state != i915_vdev->mig_state) {
		ret = vfio_mig_get_next_state(core_vdev, i915_vdev->mig_state,
					      new_state, &next_state);
		if (ret) {
			f = ERR_PTR(ret);
			break;
		}
		f = i915_vfio_set_state(i915_vdev, next_state);
		if (IS_ERR(f))
			break;

		i915_vdev->mig_state = next_state;

		/* Multiple state transitions with non-NULL file in the middle */
		if (f && new_state != i915_vdev->mig_state) {
			fput(f);
			f = ERR_PTR(-EINVAL);
			break;
		}
	}

	return f;
}

static int i915_vfio_pci_get_device_state(struct vfio_device *core_vdev,
					  enum vfio_device_mig_state *curr_state)
{
	struct i915_vfio_pci_core_device *i915_vdev =
		container_of(core_vdev, struct i915_vfio_pci_core_device, core_device.vdev);

	*curr_state = i915_vdev->mig_state;

	return 0;
}

static int i915_vfio_pci_get_data_size(struct vfio_device *vdev,
				       unsigned long *stop_copy_length)
{
	return 0;
}

static const struct vfio_migration_ops i915_vfio_pci_migration_ops = {
	.migration_set_state = i915_vfio_pci_set_device_state,
	.migration_get_state = i915_vfio_pci_get_device_state,
	.migration_get_data_size = i915_vfio_pci_get_data_size,
};

static const struct i915_vfio_pci_migration_pf_ops pf_ops = {
	.pause = i915_sriov_pause_vf,
	.resume = i915_sriov_resume_vf,
	.wait_flr_done = i915_sriov_wait_vf_flr_done,
	.ggtt.size = i915_sriov_ggtt_size,
	.ggtt.save = i915_sriov_ggtt_save,
	.ggtt.load = i915_sriov_ggtt_load,
	.fw.size = i915_sriov_fw_state_size,
	.fw.save = i915_sriov_fw_state_save,
	.fw.load = i915_sriov_fw_state_load,
};

static int i915_vfio_pci_init_dev(struct vfio_device *core_vdev)
{
	struct i915_vfio_pci_core_device *i915_vdev =
		container_of(core_vdev, struct i915_vfio_pci_core_device, core_device.vdev);
	struct pci_dev *pdev = to_pci_dev(core_vdev->dev);

	/* vfid starts from 1 for i915 */
	i915_vdev->vfid = pci_iov_vf_id(pdev) + 1;
	i915_vdev->pf = pdev->physfn;
	i915_vdev->pf_ops = &pf_ops;

	core_vdev->migration_flags = VFIO_MIGRATION_STOP_COPY;
	core_vdev->mig_ops = &i915_vfio_pci_migration_ops;

	return vfio_pci_core_init_dev(core_vdev);
}

static const struct vfio_device_ops i915_vfio_pci_ops = {
	.name		= "i915-vfio-pci",
	.init		= i915_vfio_pci_init_dev,
	.release	= vfio_pci_core_release_dev,
	.open_device	= i915_vfio_pci_open_device,
	.close_device	= vfio_pci_core_close_device,
	.ioctl		= vfio_pci_core_ioctl,
	.device_feature = vfio_pci_core_ioctl_feature,
	.read		= vfio_pci_core_read,
	.write		= vfio_pci_core_write,
	.mmap		= vfio_pci_core_mmap,
	.request	= vfio_pci_core_request,
	.match		= vfio_pci_core_match,
	.bind_iommufd	= vfio_iommufd_physical_bind,
	.unbind_iommufd = vfio_iommufd_physical_unbind,
	.attach_ioas	= vfio_iommufd_physical_attach_ioas,
};

static int i915_vfio_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct i915_vfio_pci_core_device *i915_vdev;
	int ret;

	if (!pdev->is_virtfn)
		return -EINVAL;

	if (strcmp(pdev->physfn->dev.driver->name, "i915"))
		return -EINVAL;

	i915_vdev = vfio_alloc_device(i915_vfio_pci_core_device, core_device.vdev, &pdev->dev,
				      &i915_vfio_pci_ops);
	if (IS_ERR(i915_vdev))
		return PTR_ERR(i915_vdev);

	dev_set_drvdata(&pdev->dev, &i915_vdev->core_device);

	ret = vfio_pci_core_register_device(&i915_vdev->core_device);
	if (ret) {
		vfio_put_device(&i915_vdev->core_device.vdev);
		return ret;
	}

	return 0;
}

static const struct pci_device_id i915_vfio_pci_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_ANY_ID),
	  .class = PCI_BASE_CLASS_DISPLAY << 8, .class_mask = 0xff << 16,
	  .override_only = PCI_ID_F_VFIO_DRIVER_OVERRIDE },
	{}
};
MODULE_DEVICE_TABLE(pci, i915_vfio_pci_table);

static struct pci_driver i915_vfio_pci_driver = {
	.name = "i915-vfio-pci",
	.id_table = i915_vfio_pci_table,
	.probe = i915_vfio_pci_probe,
	.err_handler = &i915_vfio_pci_err_handlers,
	.driver_managed_dma = true,
};
module_pci_driver(i915_vfio_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("VFIO PCI driver with migration support for Intel Graphics");
MODULE_IMPORT_NS(I915);
