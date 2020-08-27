// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2020 Intel Corporation

/*
 *  gna_drv.c - GNA Driver
 */

#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/types.h>

#include "gna.h"

#include "gna_drv.h"
#include "gna_ioctl.h"
#include "gna_irq.h"
#include "gna_request.h"
#include "gna_score.h"

struct class *gna_class;

struct gna_driver_private gna_drv_priv;

static bool msi_enable = true;
module_param(msi_enable, bool, 0444);
MODULE_PARM_DESC(msi_enable, "Enable MSI interrupts");

/* recovery timeout in seconds */
int recovery_timeout = 60;
module_param(recovery_timeout, int, 0644);
MODULE_PARM_DESC(recovery_timeout, "Recovery timeout");

static int gna_suspend(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int gna_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

static int gna_runtime_suspend(struct device *dev)
{
	struct gna_private *gna_priv = dev_get_drvdata(dev);
	void __iomem *addr = gna_priv->bar0.mem_addr;
	u32 val;
	int i;

	val = gna_reg_read(addr, GNAD0I3C);
	dev_dbg(dev, "D0I3 %.8x\n", gna_reg_read(addr, GNAD0I3C));

	/* Verify command in progress bit */
	i = 100;
	do {
		val = gna_reg_read(addr, GNAD0I3C);
		if ((val & 0x1) == 0)
			break;
	} while (--i);

	if (i == 0) {
		dev_err(dev, "command in progress - try again\n");
		return -EAGAIN;
	}

	gna_abort_hw(gna_priv, addr);
	gna_reg_write(addr, GNAD0I3C, GNA_D0I3_POWER_OFF);

	dev_dbg(dev, "%s: exit: D0I3 %.8x val 0x%x\n",
		__func__, gna_reg_read(addr, GNAD0I3C), val);

	return 0;
}

static int gna_runtime_resume(struct device *dev)
{
	struct gna_private *gna_priv = dev_get_drvdata(dev);
	void __iomem *addr = gna_priv->bar0.mem_addr;
	u32 val;

	dev_dbg(dev, "%s:\n", __func__);

	val = gna_reg_read(addr, GNAD0I3C);
	dev_dbg(dev, "D0I3 %.8x\n", gna_reg_read(addr, GNAD0I3C));

	/* put device in active D0 state */
	val = GNA_D0I3_POWER_ON;
	gna_reg_write(addr, GNAD0I3C, val);

	dev_dbg(dev, "D0I3 %.8x val 0x%x\n", gna_reg_read(addr, GNAD0I3C), val);
	return 0;
}

const struct dev_pm_ops gna_pm = {
	.suspend = gna_suspend,
	.resume = gna_resume,
	.runtime_suspend = gna_runtime_suspend,
	.runtime_resume = gna_runtime_resume,
};

static inline struct gna_private *inode_to_gna(struct inode *inode)
{
	return container_of(inode->i_cdev, struct gna_private, cdev);
}

static int gna_open(struct inode *inode, struct file *f)
{
	struct gna_private *gna_priv;
	int ret;
	int id;

	id = iminor(inode);

	gna_priv = inode_to_gna(inode);
	if (!gna_priv)
		return -ENODEV;

	dev_dbg(&gna_priv->dev, "%s: enter id=%d\n", __func__, id);

	mutex_lock(&gna_priv->lock);
	ret = gna_priv->ops->open(gna_priv, f);
	mutex_unlock(&gna_priv->lock);

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);

	return ret;
}

static int gna_release(struct inode *inode, struct file *f)
{
	struct gna_private *gna_priv;

	gna_priv = inode_to_gna(inode);
	if (!gna_priv)
		return -ENODEV;

	dev_dbg(&gna_priv->dev, "%s: enter\n", __func__);

	gna_priv->ops->free(f);

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);
	return 0;
}

static const struct file_operations gna_file_ops = {
	.owner		=	THIS_MODULE,
	.open		=	gna_open,
	.release	=	gna_release,
	.unlocked_ioctl =	gna_ioctl,
};

static int gna_priv_open(struct gna_private *gna_priv, struct file *fd)
{
	struct gna_file_private *file_priv;

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file_priv->fd = fd;
	file_priv->gna_priv = gna_priv;
	mutex_init(&file_priv->memlist_lock);

	fd->private_data = file_priv;

	mutex_lock(&gna_priv->filelist_lock);
	list_add_tail(&file_priv->flist, &gna_priv->file_list);
	mutex_unlock(&gna_priv->filelist_lock);
	INIT_LIST_HEAD(&file_priv->memory_list);

	return 0;
}

static void gna_priv_free(struct file *fd)
{
	struct gna_file_private *file_priv;
	struct gna_private *gna_priv;
	struct gna_memory_object *iter_mo, *temp_mo;
	struct gna_file_private *iter_file, *temp_file;

	file_priv = (struct gna_file_private *) fd->private_data;
	gna_priv = file_priv->gna_priv;

	// free all memory objects created by that file
	mutex_lock(&file_priv->memlist_lock);
	list_for_each_entry_safe(iter_mo, temp_mo,
				 &file_priv->memory_list, file_mem_list) {
		queue_work(gna_priv->request_wq, &iter_mo->work);
		wait_event(iter_mo->waitq, true);
		gna_memory_free(gna_priv, iter_mo);
	}
	mutex_unlock(&file_priv->memlist_lock);

	gna_delete_file_requests(fd, gna_priv);

	// delete itself from device's file list
	mutex_lock(&gna_priv->filelist_lock);
	list_for_each_entry_safe(iter_file, temp_file,
				 &gna_priv->file_list, flist) {
		if (iter_file->fd == fd) {
			list_del(&iter_file->flist);
			fd->private_data = NULL;
			kfree(iter_file);
			break;
		}
	}
	mutex_unlock(&gna_priv->filelist_lock);
}

static u32 gna_device_type_by_hwid(u32 hwid)
{
	switch (hwid) {
	case GNA_DEV_HWID_CNL:
		return GNA_DEV_TYPE_0_9;
	case GNA_DEV_HWID_GLK:
	case GNA_DEV_HWID_EHL:
	case GNA_DEV_HWID_ICL:
		return GNA_DEV_TYPE_1_0;
	case GNA_DEV_HWID_JSL:
	case GNA_DEV_HWID_TGL:
		return GNA_DEV_TYPE_2_0;
	default:
		return 0;
	}
}

static int gna_priv_getparam(struct gna_private *gna_priv,
		union gna_parameter *param)
{
	switch (param->in.id) {
	case GNA_PARAM_DEVICE_ID:
		param->out.value = gna_priv->info.hwid;
		break;
	case GNA_PARAM_RECOVERY_TIMEOUT:
		param->out.value = recovery_timeout;
		break;
	case GNA_PARAM_INPUT_BUFFER_S:
		param->out.value = gna_priv->hw_info.in_buf_s;
		break;
	case GNA_PARAM_DEVICE_TYPE:
		param->out.value = gna_device_type_by_hwid(gna_priv->info.hwid);
		break;
	default:
		dev_err(&gna_priv->dev,
				"unknown parameter id %llu\n", param->in.id);
		return -EINVAL;
	}

	return 0;
}

static struct gna_device_operations gna_drv_ops = {
	.owner		=	THIS_MODULE,
	.getparam	=	gna_priv_getparam,
	.open		=	gna_priv_open,
	.free		=	gna_priv_free,
	.score		=	gna_priv_score,
	.userptr	=	gna_priv_userptr,
};

void gna_dev_release(struct device *dev)
{
	struct gna_private *gna_priv;

	dev_dbg(dev, "%s enter\n", __func__);

	gna_priv = dev_get_drvdata(dev);

	__clear_bit(MINOR(dev->devt), gna_drv_priv.dev_map);
	flush_workqueue(gna_priv->request_wq);
	destroy_workqueue(gna_priv->request_wq);
	idr_destroy(&gna_priv->memory_idr);
	gna_mmu_free(gna_priv);
	dev_set_drvdata(dev, NULL);
	pci_set_drvdata(gna_priv->pdev, NULL);

	kfree(gna_priv);

	dev_dbg(dev, "%s exit\n", __func__);
}

static int gna_dev_create(struct gna_private *gna_priv)
{
	struct pci_dev *pcidev;
	struct device *dev;
	dev_t gna_devt;
	int dev_num;
	int major;
	int minor;
	int ret;

	pcidev = gna_priv->pdev;

	mutex_lock(&gna_drv_priv.lock);

	dev_num = find_first_zero_bit(gna_drv_priv.dev_map, MAX_GNA_DEVICES);
	if (dev_num == MAX_GNA_DEVICES) {
		mutex_unlock(&gna_drv_priv.lock);
		dev_err(&pcidev->dev, "number of gna devices reached maximum\n");
		return -ENODEV;
	}

	set_bit(dev_num, gna_drv_priv.dev_map);
	major = MAJOR(gna_drv_priv.devt);
	minor = gna_drv_priv.minor++;

	mutex_unlock(&gna_drv_priv.lock);

	gna_devt = MKDEV(major, minor);
	dev = &gna_priv->dev;
	device_initialize(dev);
	dev->devt = gna_devt;
	dev->class = gna_class;
	dev->parent = gna_priv->parent;
	dev->groups = NULL;
	dev->release = gna_dev_release;
	dev_set_drvdata(dev, gna_priv);
	dev_set_name(dev, "gna%d", dev_num);

	snprintf(gna_priv->name, sizeof(gna_priv->name), "gna%d", dev_num);
	gna_priv->dev_num = dev_num;

	cdev_init(&gna_priv->cdev, &gna_file_ops);
	gna_priv->cdev.owner = THIS_MODULE;

	ret = cdev_device_add(&gna_priv->cdev, &gna_priv->dev);
	if (ret) {
		mutex_lock(&gna_drv_priv.lock);
		__clear_bit(minor, gna_drv_priv.dev_map);
		mutex_unlock(&gna_drv_priv.lock);
		dev_err(&gna_priv->dev, "could not add gna%d char device\n",
			dev_num);
	} else {
		dev_info(&gna_priv->dev, "registered gna%d device: major %d, "
			"minor %d\n", dev_num, major, minor);
	}

	return ret;
}

static void gna_dev_read_bld_reg(struct gna_private *gna_priv)
{
	u32 bld_reg = gna_reg_read(gna_priv->bar0.mem_addr, GNAIBUFFS);

	gna_priv->hw_info.in_buf_s = bld_reg & GENMASK(7, 0);
	gna_priv->hw_info.ce_num = (bld_reg & GENMASK(11, 8)) >> 8;
	gna_priv->hw_info.ple_num = (bld_reg & GENMASK(15, 12)) >> 12;
	gna_priv->hw_info.afe_num = (bld_reg & GENMASK(19, 16)) >> 16;
	gna_priv->hw_info.has_mmu = (bld_reg & BIT_MASK(23)) >> 23;
	gna_priv->hw_info.hw_ver = (bld_reg & GENMASK(31, 24)) >> 24;
}

static int gna_dev_init(struct gna_private *gna_priv, struct pci_dev *pcidev,
		const struct pci_device_id *pci_id)
{
	struct gna_drv_info *gna_info;
	int ret;

	pci_set_drvdata(pcidev, gna_priv);

	gna_priv->irq = pcidev->irq;
	gna_priv->parent = &pcidev->dev;
	gna_priv->pdev = pci_dev_get(pcidev);

	gna_info = (struct gna_drv_info *)pci_id->driver_data;
	gna_priv->info = *gna_info;

	gna_dev_read_bld_reg(gna_priv);

	if (gna_mmu_alloc(gna_priv)) {
		dev_err(&gna_priv->dev, "gna mmu allocation failed\n");
		return -EFAULT;
	}

	dev_dbg(&pcidev->dev, "maximum memory size %llu num pd %d\n",
			gna_info->max_hw_mem, gna_info->num_pagetables);

	dev_dbg(&pcidev->dev, "desc gna_info %d mmu gna_info %d\n",
			gna_info->desc_info.rsvd_size,
			gna_info->desc_info.mmu_info.vamax_size);

	mutex_init(&gna_priv->lock);
	mutex_init(&gna_priv->mmu_lock);
	mutex_init(&gna_priv->filelist_lock);
	mutex_init(&gna_priv->reqlist_lock);
	spin_lock_init(&gna_priv->busy_lock);
	spin_lock_init(&gna_priv->hw_lock);
	init_waitqueue_head(&gna_priv->busy_waitq);

	gna_priv->drv_priv = &gna_drv_priv;

	INIT_LIST_HEAD(&gna_priv->file_list);
	INIT_LIST_HEAD(&gna_priv->request_list);

	timer_setup(&gna_priv->isr_timer, gna_isr_timeout, 0);

	atomic_set(&gna_priv->request_count, 0);
	atomic_set(&gna_priv->isr_count, 0);

	tasklet_init(&gna_priv->request_tasklet, gna_request_tasklet, 0);

	idr_init(&gna_priv->memory_idr);
	mutex_init(&gna_priv->memidr_lock);

	gna_priv->request_wq = create_singlethread_workqueue("gna_request_wq");
	if (!gna_priv->request_wq) {
		dev_err(&pcidev->dev, "could not create wq for gna device\n");
		ret = -EFAULT;
		goto err_pci_put;
	}

	gna_priv->busy = false;

	gna_priv->ops = &gna_drv_ops;

	ret = gna_dev_create(gna_priv);
	if (ret) {
		dev_err(&pcidev->dev, "could not create gna device\n");
		goto err_del_wq;
	}

	return 0;

err_del_wq:
	destroy_workqueue(gna_priv->request_wq);

err_pci_put:
	pci_dev_put(pcidev);
	pci_set_drvdata(pcidev, NULL);

	return ret;
}

int gna_probe(struct pci_dev *pcidev, const struct pci_device_id *pci_id)
{
	struct gna_private *gna_priv;
	int ret;

	dev_dbg(&pcidev->dev, "%s: enter\n", __func__);

	ret = pci_enable_device(pcidev);
	if (ret) {
		dev_err(&pcidev->dev, "pci device can't be enabled\n");
		goto end;
	}

	ret = pci_request_regions(pcidev, GNA_DRV_NAME);
	if (ret)
		goto err_disable_device;

	ret = pci_set_dma_mask(pcidev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(&pcidev->dev,
			"pci_set_dma_mask returned error %d\n", ret);
		goto err_release_regions;
	}

	pci_set_master(pcidev);

	/* register for interrupts */
	if (msi_enable) {
		ret = pci_enable_msi(pcidev);
		if (ret) {
			dev_err(&pcidev->dev, "could not enable msi interrupts\n");
			goto err_clear_master;
		}
		dev_info(&pcidev->dev, "msi interrupts enabled\n");
	}

	/* init gna device */
	gna_priv = kzalloc(sizeof(*gna_priv), GFP_KERNEL | GFP_ATOMIC);
	if (!gna_priv) {
		ret = PTR_ERR(gna_priv);
		dev_err(&pcidev->dev, "could not allocate gna private structure\n");
		goto err_disable_msi;
	}

	ret = request_threaded_irq(pcidev->irq, gna_interrupt,
			gna_irq_thread, IRQF_SHARED,
			GNA_DRV_NAME, gna_priv);

	if (ret) {
		dev_err(&pcidev->dev, "could not register for interrupt\n");
		goto err_free_priv;
	}

	dev_dbg(&pcidev->dev, "irq num %d\n", pcidev->irq);

	/* Map BAR0 */
	gna_priv->bar0.iostart = pci_resource_start(pcidev, 0);
	gna_priv->bar0.iosize = pci_resource_len(pcidev, 0);
	gna_priv->bar0.mem_addr = pci_iomap(pcidev, 0, 0);

	dev_dbg(&pcidev->dev,
		"bar0 io start: %p\n", (void *)gna_priv->bar0.iostart);
	dev_dbg(&pcidev->dev,
		"bar0 io size: %llu\n", gna_priv->bar0.iosize);
	dev_dbg(&pcidev->dev,
		"bar0 memory address: %p\n", (void *)gna_priv->bar0.mem_addr);

	ret = gna_dev_init(gna_priv, pcidev, pci_id);
	if (ret) {
		dev_err(&pcidev->dev,
			"could not initialize gna private structure\n");
		goto err_free_irq;
	}

	/* enable power management callbacks */
	pm_runtime_set_autosuspend_delay(&pcidev->dev, 2000);
	pm_runtime_use_autosuspend(&pcidev->dev);
	pm_runtime_allow(&pcidev->dev);
	pm_runtime_put_noidle(&pcidev->dev);

	dev_dbg(&pcidev->dev, "%s exit\n", __func__);

	return 0;

err_free_irq:
	pci_iounmap(pcidev, gna_priv->bar0.mem_addr);
	free_irq(pcidev->irq, gna_priv);
err_free_priv:
	kfree(gna_priv);
err_disable_msi:
	if (msi_enable)
		pci_disable_msi(pcidev);
err_clear_master:
	pci_clear_master(pcidev);
err_release_regions:
	pci_release_regions(pcidev);
err_disable_device:
	pci_disable_device(pcidev);
end:
	dev_err(&pcidev->dev, "gna probe failed with %d\n", ret);
	return ret;
}

void gna_remove(struct pci_dev *pcidev)
{
	struct gna_private *gna_priv;

	gna_priv = pci_get_drvdata(pcidev);
	if (IS_ERR(gna_priv)) {
		dev_err(&pcidev->dev, "could not get driver data from pci device\n");
		return;
	}

	dev_dbg(&gna_priv->dev, "%s: enter\n", __func__);

	cdev_device_del(&gna_priv->cdev, &gna_priv->dev);

	free_irq(pcidev->irq, gna_priv);

	if (msi_enable)
		pci_disable_msi(pcidev);

	pci_clear_master(pcidev);
	pci_iounmap(pcidev, gna_priv->bar0.mem_addr);
	pci_release_regions(pcidev);
	pci_disable_device(pcidev);
	pci_dev_put(pcidev);

	dev_dbg(&gna_priv->dev, "%s: exit\n", __func__);
}
