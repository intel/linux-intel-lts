// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include "mxlk_pci.h"
#include "../common/mxlk_core.h"
#include "../common/mxlk_util.h"

#define BOOT_TIMEOUT 60000
#define STATUS_TIMEOUT 5000
#define ERASE_TIMEOUT 30000

#define SECTION_SIZE (1 * 1024 * 1024)

static int aspm_enable;
module_param(aspm_enable, int, 0664);
MODULE_PARM_DESC(aspm_enable, "enable ASPM");

static int mxlk_pci_setup_recovery_sysfs(struct mxlk_pcie *xdev);
static void mxlk_pci_cleanup_recovery_sysfs(struct mxlk_pcie *xdev);

static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_mutex);

struct mxlk_pcie *mxlk_get_device_by_id(u32 id)
{
	struct mxlk_pcie *xdev;

	mutex_lock(&dev_list_mutex);

	if (list_empty(&dev_list)) {
		mutex_unlock(&dev_list_mutex);
		return NULL;
	}

	list_for_each_entry(xdev, &dev_list, list) {
		if (xdev->devid == id) {
			mutex_unlock(&dev_list_mutex);
			return xdev;
		}
	}

	mutex_unlock(&dev_list_mutex);

	return NULL;
}

struct mxlk_pcie *mxlk_create_device(u32 sw_device_id, struct pci_dev *pdev)
{
	struct mxlk_pcie *xdev = kzalloc(sizeof(struct mxlk_pcie), GFP_KERNEL);

	if (!xdev)
		return NULL;

	xdev->devid = sw_device_id;
	snprintf(xdev->name, MXLK_MAX_NAME_LEN, "%02x:%02x.%x",
		 pdev->bus->number,
		 PCI_SLOT(pdev->devfn),
		 PCI_FUNC(pdev->devfn));

	mutex_init(&xdev->lock);

	return xdev;
}

void mxlk_remove_device(struct mxlk_pcie *xdev)
{
	mutex_destroy(&xdev->lock);
	kfree(xdev);
}

void mxlk_list_add_device(struct mxlk_pcie *xdev)
{
	mutex_lock(&dev_list_mutex);

	list_add_tail(&xdev->list, &dev_list);

	mutex_unlock(&dev_list_mutex);
}

void mxlk_list_del_device(struct mxlk_pcie *xdev)
{
	mutex_lock(&dev_list_mutex);

	list_del(&xdev->list);

	mutex_unlock(&dev_list_mutex);
}

static void mxlk_pci_set_aspm(struct mxlk_pcie *xdev, int aspm)
{
	u8 cap_exp;
	u16 link_control;

	cap_exp = pci_find_capability(xdev->pci, PCI_CAP_ID_EXP);
	if (!cap_exp) {
		dev_err(&xdev->pci->dev, "failed to find pcie capability\n");
		return;
	}

	pci_read_config_word(xdev->pci, cap_exp + PCI_EXP_LNKCTL,
			     &link_control);
	link_control &= ~(PCI_EXP_LNKCTL_ASPMC);
	link_control |= (aspm & PCI_EXP_LNKCTL_ASPMC);
	pci_write_config_word(xdev->pci, cap_exp + PCI_EXP_LNKCTL,
			      link_control);
}

static void mxlk_pci_unmap_bar(struct mxlk_pcie *xdev)
{
	if (xdev->mxlk.bar0) {
		iounmap(xdev->mxlk.bar0);
		xdev->mxlk.bar0 = NULL;
	}

	if (xdev->mxlk.io_comm) {
		iounmap(xdev->mxlk.io_comm);
		xdev->mxlk.io_comm = NULL;
		xdev->mxlk.mmio = NULL;
	}

	if (xdev->mxlk.bar4) {
		iounmap(xdev->mxlk.bar4);
		xdev->mxlk.bar4 = NULL;
	}
}

static int mxlk_pci_map_bar(struct mxlk_pcie *xdev)
{
	if (pci_resource_len(xdev->pci, 2) < MXLK_IO_COMM_SIZE) {
		dev_err(&xdev->pci->dev, "device BAR region is too small\n");
		return -EIO;
	}

	xdev->mxlk.bar0 = pci_ioremap_bar(xdev->pci, 0);
	if (!xdev->mxlk.bar0) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR0\n");
		goto bar_error;
	}

	xdev->mxlk.io_comm = pci_ioremap_bar(xdev->pci, 2);
	if (!xdev->mxlk.io_comm) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR2\n");
		goto bar_error;
	}
	xdev->mxlk.mmio = (void __iomem *)xdev->mxlk.io_comm + MXLK_MMIO_OFFSET;

	xdev->mxlk.bar4 = pci_ioremap_wc_bar(xdev->pci, 4);
	if (!xdev->mxlk.bar4) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR4\n");
		goto bar_error;
	}

	return 0;

bar_error:
	mxlk_pci_unmap_bar(xdev);
	return -EIO;
}

#define STR_EQUAL(a, b) !strncmp(a, b, strlen(b))

static enum mxlk_stage mxlk_check_magic(struct mxlk_pcie *xdev)
{
	char magic[MXLK_BOOT_MAGIC_STRLEN];

	memcpy_fromio(magic, xdev->mxlk.io_comm->magic,
		      MXLK_BOOT_MAGIC_STRLEN);

	if (strlen(magic) == 0)
		return STAGE_UNINIT;

	if (STR_EQUAL(magic, MXLK_BOOT_MAGIC_ROM))
		return STAGE_ROM;

	if (STR_EQUAL(magic, MXLK_BOOT_MAGIC_EMMC))
		return STAGE_ROM;

	if (STR_EQUAL(magic, MXLK_BOOT_MAGIC_BL2))
		return STAGE_BL2;

	if (STR_EQUAL(magic, MXLK_BOOT_MAGIC_UBOOT))
		return STAGE_UBOOT;

	if (STR_EQUAL(magic, MXLK_BOOT_MAGIC_RECOV))
		return STAGE_RECOV;

	if (STR_EQUAL(magic, MXLK_BOOT_MAGIC_YOCTO))
		return STAGE_OS;

	return STAGE_UNINIT;
}

static irqreturn_t mxlk_interrupt(int irq, void *args)
{
	struct mxlk_pcie *xdev = args;
	enum mxlk_stage stage;
	u8 event;

	event = mxlk_get_doorbell(&xdev->mxlk, FROM_DEVICE, DEV_EVENT);
	if (event == DEV_SHUTDOWN || event == 0xFF) {
		schedule_delayed_work(&xdev->shutdown_event, 0);
		return IRQ_HANDLED;
	}

	if (likely(xdev->core_irq_callback))
		return xdev->core_irq_callback(irq, args);

	stage = mxlk_check_magic(xdev);
	if (stage == STAGE_ROM) {
		xdev->mxlk.status = MXLK_STATUS_BOOT_FW;
		wake_up_interruptible(&xdev->waitqueue);
	} else if (stage == STAGE_UBOOT) {
		xdev->mxlk.status = MXLK_STATUS_BOOT_OS;
		wake_up_interruptible(&xdev->waitqueue);
	} else if (stage == STAGE_RECOV) {
		xdev->mxlk.status = MXLK_STATUS_RECOVERY;
		wake_up_interruptible(&xdev->waitqueue);
	}

	return IRQ_HANDLED;
}

static void mxlk_pci_irq_cleanup(struct mxlk_pcie *xdev)
{
#if KERNEL_VERSION(4, 8, 0) <= LINUX_VERSION_CODE
	int irq = pci_irq_vector(xdev->pci, 0);

	if (irq < 0)
		return;

	synchronize_irq(irq);
	free_irq(irq, xdev);
	pci_free_irq_vectors(xdev->pci);
#else
	if (!pci_msi_enabled())
		return;

	synchronize_irq(xdev->pci->irq);
	free_irq(xdev->pci->irq, xdev);
	pci_disable_msi(xdev->pci);
#endif
}

#if KERNEL_VERSION(4, 8, 0) <= LINUX_VERSION_CODE
static int mxlk_pci_irq_init(struct mxlk_pcie *xdev)
{
	int irq;
	int rc;

	rc = pci_alloc_irq_vectors(xdev->pci, 1, 1, PCI_IRQ_MSI);
	if (rc < 0) {
		dev_err(&xdev->pci->dev,
			"failed to allocate %d MSI vectors\n", 1);
		return rc;
	}

	irq = pci_irq_vector(xdev->pci, 0);
	if (irq < 0) {
		dev_err(&xdev->pci->dev, "failed to get irq\n");
		rc = irq;
		goto error_irq;
	}
	rc = request_irq(irq, &mxlk_interrupt, 0, MXLK_DRIVER_NAME, xdev);
	if (rc) {
		dev_err(&xdev->pci->dev, "failed to request irqs\n");
		goto error_irq;
	}

	return 0;

error_irq:
	pci_free_irq_vectors(xdev->pci);
	return rc;
}
#else
static int mxlk_pci_irq_init(struct mxlk_pcie *xdev)
{
	int rc;

	rc = pci_enable_msi(xdev->pci);
	if (rc < 1) {
		dev_err(&xdev->pci->dev, "failed to allocate MSI vectors\n");
		return rc;
	}

	rc = request_irq(xdev->pci->irq, &mxlk_interrupt, 0,
			    MXLK_DRIVER_NAME, xdev);
	if (rc) {
		dev_err(&xdev->pci->dev, "failed to request irqs\n");
		goto error_irq;
	}

	return 0;

error_irq:
	pci_disable_msi(xdev->pci);
	return rc;
}
#endif

static int mxlk_device_wait_status(struct mxlk_pcie *xdev, u32 image_id,
				   u32 timeout_ms)
{
	u32 status = MXLK_BOOT_STATUS_START;
	int count = 0;

	if (timeout_ms == 0)
		timeout_ms = STATUS_TIMEOUT;

	iowrite32(image_id, &xdev->mxlk.io_comm->mf_ready);

	while (status != MXLK_BOOT_STATUS_DOWNLOADED) {
		mdelay(1);
		if (++count > timeout_ms) {
			dev_err(&xdev->pci->dev, "operation takes too long.\n");
			return -ETIME;
		}

		status = ioread32(&xdev->mxlk.io_comm->mf_ready);

		switch (status) {
		case MXLK_BOOT_STATUS_INVALID:
			dev_err(&xdev->pci->dev,
				"the firmware image data is invalid.\n");
			return -EINVAL;
		case MXLK_BOOT_STATUS_ERROR:
			dev_err(&xdev->pci->dev,
				"failed to download firmware image.\n");
			return -EINVAL;
		default:
			break;
		}
	}

	return 0;
}

static int mxlk_device_transfer(struct mxlk_pcie *xdev, u32 image_id,
				dma_addr_t addr, size_t size)
{
	int rc;

	_iowrite64(addr, &xdev->mxlk.io_comm->mf_start);
	iowrite32(size, &xdev->mxlk.io_comm->mf_len);

	if (image_id == MXLK_BOOT_RAW_ID)
		_iowrite64(xdev->partition_offset,
			   &xdev->mxlk.io_comm->mf_offset);

	rc = mxlk_device_wait_status(xdev, image_id, 0);
	if (!rc)
		xdev->partition_offset += size;

	return rc;
}

static int mxlk_device_download_common(struct mxlk_pcie *xdev, u32 image_id,
				       const void *buf, size_t buf_size,
				       bool no_copy)
{
	int rc = 0;
	size_t size;
	size_t size_left = buf_size;
	dma_addr_t phys_addr;
	struct device *dev = &xdev->pci->dev;

	while (size_left) {
		size = (size_left > SECTION_SIZE) ? SECTION_SIZE : size_left;

		if (!no_copy)
			memcpy(xdev->dma_buf, buf, size);

		phys_addr = dma_map_single(dev, xdev->dma_buf, size,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(dev, phys_addr))
			return -ENOMEM;

		rc = mxlk_device_transfer(xdev, image_id, phys_addr, size);

		dma_unmap_single(dev, phys_addr, size, DMA_TO_DEVICE);

		if (rc)
			return rc;

		size_left -= size;
		buf += size;
	}

	return rc;
}

static int mxlk_device_download_firmware(struct mxlk_pcie *xdev, u32 image_id,
					 const char *fw_image)
{
	const struct firmware *firmware;
	struct device *dev = &xdev->pci->dev;
	int rc = 0;

	switch (image_id) {
	case MXLK_BOOT_FIP_ID:
	case MXLK_BOOT_BOOT_ID:
	case MXLK_BOOT_SYSTEM_ID:
		break;
	default:
		dev_err(dev, "unknown firmware id\n");
		return -EINVAL;
	}

	if (request_firmware(&firmware, fw_image, dev) < 0)
		return -EIO;

	if (firmware->size == 0) {
		rc = -EIO;
		goto firmware_cleanup;
	}

	xdev->dma_buf = kmalloc(SECTION_SIZE, GFP_KERNEL);
	if (!xdev->dma_buf) {
		rc = -ENOMEM;
		goto firmware_cleanup;
	}

	rc = mxlk_device_download_common(xdev, image_id, firmware->data,
					 firmware->size, false);

	kfree(xdev->dma_buf);
	xdev->dma_buf = NULL;

firmware_cleanup:
	release_firmware(firmware);

	return rc;
}

static int mxlk_device_flashless_boot(struct mxlk_pcie *xdev)
{
	if (mxlk_device_download_firmware(xdev, MXLK_BOOT_BOOT_ID,
					  xdev->fw_name)) {
		dev_err(&xdev->pci->dev, "failed to download boot image\n");
		return -EIO;
	}

	iowrite32(MXLK_BOOT_STATUS_DONE, &xdev->mxlk.io_comm->mf_ready);

	return 0;
}

static int mxlk_device_fip(struct mxlk_pcie *xdev)
{
	if (mxlk_device_download_firmware(xdev, MXLK_BOOT_FIP_ID,
					  xdev->fw_name)) {
		dev_err(&xdev->pci->dev, "failed to download FIP image\n");
		return -EIO;
	}

	iowrite32(MXLK_BOOT_STATUS_DONE, &xdev->mxlk.io_comm->mf_ready);

	return 0;
}

static void mxlk_device_enable_irq(struct mxlk_pcie *xdev)
{
	iowrite32(MXLK_INT_ENABLE, &xdev->mxlk.io_comm->int_enable);
	iowrite32(~MXLK_INT_MASK, &xdev->mxlk.io_comm->int_mask);
}

static void mxlk_device_poll(struct work_struct *work)
{
	struct mxlk_pcie *xdev = container_of(work, struct mxlk_pcie,
					      wait_event.work);
	enum mxlk_stage stage = mxlk_check_magic(xdev);

	if (stage == STAGE_RECOV) {
		xdev->mxlk.status = MXLK_STATUS_RECOVERY;
		wake_up_interruptible(&xdev->waitqueue);
	} else if (stage == STAGE_OS) {
		xdev->mxlk.status = MXLK_STATUS_READY;
		wake_up_interruptible(&xdev->waitqueue);
		return;
	}

	schedule_delayed_work(&xdev->wait_event, msecs_to_jiffies(100));
}

static int mxlk_pci_prepare_dev_reset(struct mxlk_pcie *xdev, bool notify);

static void mxlk_device_shutdown(struct work_struct *work)
{
	struct mxlk_pcie *xdev = container_of(work, struct mxlk_pcie,
					      shutdown_event.work);

	mxlk_pci_prepare_dev_reset(xdev, false);
}

static int mxlk_device_init(struct mxlk_pcie *xdev)
{
	int rc;

	INIT_DELAYED_WORK(&xdev->wait_event, mxlk_device_poll);
	INIT_DELAYED_WORK(&xdev->shutdown_event, mxlk_device_shutdown);

	rc = mxlk_pci_irq_init(xdev);
	if (rc)
		return rc;

	pci_set_master(xdev->pci);

	xdev->mxlk.status = MXLK_STATUS_UNINIT;

	init_waitqueue_head(&xdev->waitqueue);
	schedule_delayed_work(&xdev->wait_event, 0);

	return rc;
}

int mxlk_pci_init(struct mxlk_pcie *xdev, struct pci_dev *pdev)
{
	int rc;

	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	xdev->pci = pdev;
	pci_set_drvdata(pdev, xdev);

	rc = pci_enable_device_mem(xdev->pci);
	if (rc) {
		dev_err(&pdev->dev, "failed to enable pci device\n");
		goto error_exit;
	}

	rc = pci_request_regions(xdev->pci, MXLK_DRIVER_NAME);
	if (rc) {
		dev_err(&pdev->dev, "failed to request mmio regions\n");
		goto error_req_mem;
	}

	rc = mxlk_pci_map_bar(xdev);
	if (rc)
		goto error_map;

	rc = dma_set_mask_and_coherent(&xdev->pci->dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_err(&pdev->dev, "failed to set dma mask\n");
		goto error_dma_mask;
	}

	mxlk_pci_set_aspm(xdev, aspm_enable);

	rc = mxlk_pci_setup_recovery_sysfs(xdev);
	if (rc) {
		dev_err(&pdev->dev,
			"failed to setup recovery sysfs facilities\n");
		goto error_dma_mask;
	}

	mxlk_init_debug(&xdev->mxlk, &xdev->pci->dev);

	rc = mxlk_device_init(xdev);
	if (!rc)
		goto init_exit;

error_dma_mask:
	mxlk_pci_unmap_bar(xdev);

error_map:
	pci_release_regions(xdev->pci);

error_req_mem:
	pci_disable_device(xdev->pci);

error_exit:
	xdev->mxlk.status = MXLK_STATUS_ERROR;

init_exit:
	mutex_unlock(&xdev->lock);
	if (rc)
		mutex_destroy(&xdev->lock);
	return rc;
}

int mxlk_pci_cleanup(struct mxlk_pcie *xdev)
{
	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	mxlk_pci_cleanup_recovery_sysfs(xdev);
	mxlk_uninit_debug(&xdev->mxlk, &xdev->pci->dev);

	cancel_delayed_work_sync(&xdev->wait_event);
	cancel_delayed_work_sync(&xdev->shutdown_event);
	xdev->core_irq_callback = NULL;
	mxlk_pci_irq_cleanup(xdev);

	kfree(xdev->dma_buf);
	xdev->dma_buf = NULL;
	xdev->dma_buf_offset = 0;

	mxlk_core_cleanup(&xdev->mxlk);

	mxlk_pci_unmap_bar(xdev);
	pci_release_regions(xdev->pci);
	pci_disable_device(xdev->pci);
	pci_set_drvdata(xdev->pci, NULL);
	xdev->mxlk.status = MXLK_STATUS_OFF;
	xdev->irq_enabled = false;

	mutex_unlock(&xdev->lock);

	return 0;
}

int mxlk_pci_register_irq(struct mxlk_pcie *xdev, irq_handler_t irq_handler)
{
	if (xdev->mxlk.status != MXLK_STATUS_READY)
		return -EINVAL;

	xdev->core_irq_callback = irq_handler;

	return 0;
}

int mxlk_pci_raise_irq(struct mxlk_pcie *xdev, enum mxlk_doorbell_type type,
		       u8 value)
{
	u16 pci_status;

	mxlk_set_doorbell(&xdev->mxlk, TO_DEVICE, type, value);
	pci_read_config_word(xdev->pci, PCI_STATUS, &pci_status);

	return 0;
}

u32 mxlk_get_device_num(u32 *id_list)
{
	u32 num = 0;
	struct mxlk_pcie *p;

	mutex_lock(&dev_list_mutex);

	if (list_empty(&dev_list)) {
		mutex_unlock(&dev_list_mutex);
		return 0;
	}

	list_for_each_entry(p, &dev_list, list) {
		*id_list++ = p->devid;
		num++;
	}
	mutex_unlock(&dev_list_mutex);

	return num;
}

int mxlk_get_device_name_by_id(u32 id, char *device_name, size_t name_size)
{
	struct mxlk_pcie *xdev;
	size_t size;

	xdev = mxlk_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	mutex_lock(&xdev->lock);

	size = (name_size > MXLK_MAX_NAME_LEN) ? MXLK_MAX_NAME_LEN : name_size;
	strncpy(device_name, xdev->name, size);

	mutex_unlock(&xdev->lock);

	return 0;
}

int mxlk_get_device_status_by_id(u32 id, u32 *status)
{
	struct mxlk_pcie *xdev = mxlk_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;

	mutex_lock(&xdev->lock);
	*status = xdev->mxlk.status;
	mutex_unlock(&xdev->lock);

	return 0;
}

#define mxlk_wait_event(cond)						\
({									\
	int rc = 0;							\
	int error = wait_event_interruptible_timeout(xdev->waitqueue,	\
			cond, msecs_to_jiffies(BOOT_TIMEOUT));		\
	if (error == 0)							\
		rc = -ETIME;						\
	if (error < 0)							\
		rc = -EAGAIN;						\
	rc;								\
})

int mxlk_pci_boot_device(u32 id, const char *binary_name)
{
	int rc = 0;
	u32 expected = MXLK_STATUS_ERROR;
	struct mxlk_pcie *xdev;

	xdev = mxlk_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	if (xdev->mxlk.status == MXLK_STATUS_OFF) {
		rc = -ENODEV;
		goto boot_cleanup;
	}

	strncpy(xdev->fw_name, binary_name, MXLK_MAX_NAME_LEN - 1);

	if (!xdev->irq_enabled) {
		mxlk_device_enable_irq(xdev);
		xdev->irq_enabled = true;

		rc = mxlk_wait_event(xdev->mxlk.status != MXLK_STATUS_UNINIT);
		if (rc)
			goto boot_cleanup;
	}

	switch (xdev->mxlk.status) {
	case MXLK_STATUS_BOOT_FW:
		xdev->mxlk.status = MXLK_STATUS_BOOT_PRE_OS;
		rc = mxlk_device_fip(xdev);
		goto boot_cleanup;
	case MXLK_STATUS_BOOT_PRE_OS:
		/*
		 * This fake stage is to avoid boot timeout after flashing FIP
		 * if the function only returns after entering BOOT_OS stage.
		 * It's because if host doesn't support PCIe hot plug the
		 * reset after flashing FIP cannot be detected so driver won't
		 * know the stage change at all.
		 * So let boot call on FIP return early and check stage in next
		 * boot call for OS.
		 */
		rc = mxlk_wait_event(xdev->mxlk.status == MXLK_STATUS_BOOT_OS);
		if (rc)
			goto boot_cleanup;
		rc = mxlk_device_flashless_boot(xdev);
		if (rc)
			goto boot_cleanup;
		expected = MXLK_STATUS_READY;
		break;
	case MXLK_STATUS_BOOT_OS:
		rc = mxlk_device_flashless_boot(xdev);
		if (rc)
			goto boot_cleanup;
		expected = MXLK_STATUS_READY;
		break;
	case MXLK_STATUS_READY:
	case MXLK_STATUS_RUN:
		rc = 0;
		goto boot_cleanup;
	case MXLK_STATUS_RECOVERY:
	case MXLK_STATUS_ERROR:
	default:
		rc = -EIO;
		goto boot_cleanup;
	}

	rc = mxlk_wait_event((xdev->mxlk.status == expected) ||
			     (xdev->mxlk.status == MXLK_STATUS_RECOVERY));

	if (xdev->mxlk.status == MXLK_STATUS_RECOVERY)
		rc = -EIO;

boot_cleanup:
	mutex_unlock(&xdev->lock);

	return rc;
}

int mxlk_pci_connect_device(u32 id)
{
	int rc = 0;
	struct mxlk_pcie *xdev;

	xdev = mxlk_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	if (xdev->mxlk.status == MXLK_STATUS_RUN)
		goto connect_cleanup;

	if (xdev->mxlk.status == MXLK_STATUS_OFF) {
		rc = -ENODEV;
		goto connect_cleanup;
	}

	if (xdev->mxlk.status != MXLK_STATUS_READY) {
		rc = -EBUSY;
		goto connect_cleanup;
	}

	rc = mxlk_core_init(&xdev->mxlk);
	if (rc < 0) {
		dev_err(&xdev->pci->dev, "failed to sync with device\n");
		goto connect_cleanup;
	}

connect_cleanup:
	mutex_unlock(&xdev->lock);
	return rc;
}

int mxlk_pci_read(u32 id, void *data, size_t *size, u32 timeout)
{
	struct mxlk_pcie *xdev = mxlk_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;

	return mxlk_core_read(&xdev->mxlk, data, size, timeout);
}

int mxlk_pci_write(u32 id, void *data, size_t *size, u32 timeout)
{
	struct mxlk_pcie *xdev = mxlk_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;

	return mxlk_core_write(&xdev->mxlk, data, size, timeout);
}

static int mxlk_pci_prepare_dev_reset(struct mxlk_pcie *xdev, bool notify)
{
	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	if (xdev->core_irq_callback) {
		xdev->core_irq_callback = NULL;
		mxlk_core_cleanup(&xdev->mxlk);
	}
	xdev->mxlk.status = MXLK_STATUS_OFF;
	if (notify)
		mxlk_pci_raise_irq(xdev, DEV_EVENT, REQUEST_RESET);

	mutex_unlock(&xdev->lock);

	return 0;
}

int mxlk_pci_reset_device(u32 id)
{
	struct mxlk_pcie *xdev = mxlk_get_device_by_id(id);

	if (!xdev)
		return -ENOMEM;

	return mxlk_pci_prepare_dev_reset(xdev, true);
}

u64 mxlk_pci_hw_dev_id(struct mxlk_pcie *xdev)
{
	return _ioread64(&xdev->mxlk.io_comm->dev_id);
}

static int mxlk_boot_access_enter(struct mxlk_pcie *xdev)
{
	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;
	if (xdev->mxlk.status != MXLK_STATUS_RECOVERY) {
		mutex_unlock(&xdev->lock);
		return -EROFS;
	}
	return 0;
}

static void mxlk_boot_access_exit(struct mxlk_pcie *xdev)
{
	mutex_unlock(&xdev->lock);
}

static int mxlk_recovery_send_left(struct mxlk_pcie *xdev, bool reset_offset)
{
	int rc;

	rc = mxlk_device_download_common(xdev, MXLK_BOOT_RAW_ID, NULL,
			xdev->dma_buf_offset, true);
	xdev->dma_buf_offset = 0;
	if (reset_offset)
		xdev->partition_offset = 0;

	return rc;
}

static int mxlk_pci_flash_gpt_table(struct mxlk_pcie *xdev)
{
	int rc = 0;

	rc = mxlk_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = mxlk_recovery_send_left(xdev, true);
		if (rc)
			goto create_error;
	}

	rc = mxlk_device_wait_status(xdev, MXLK_BOOT_FLASH_ID, 0);

create_error:
	mxlk_boot_access_exit(xdev);
	return rc;
}

static int mxlk_pci_erase_partition(struct mxlk_pcie *xdev,
				    const char *partition, size_t len)
{
	int rc = 0;

	rc = mxlk_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = mxlk_recovery_send_left(xdev, true);
		if (rc)
			goto erase_error;
	}

	memcpy_toio(xdev->mxlk.io_comm->mf_dest, partition, len);

	rc = mxlk_device_wait_status(xdev, MXLK_BOOT_ERASE_ID, ERASE_TIMEOUT);

erase_error:
	mxlk_boot_access_exit(xdev);
	return rc;
}

static int mxlk_pci_flash_partition_start(struct mxlk_pcie *xdev,
					  const char *partition,
					  size_t name_len)
{
	int rc = 0;

	rc = mxlk_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = mxlk_recovery_send_left(xdev, true);
		if (rc)
			goto start_error;
	}

	memset(xdev->partition_name, 0, MXLK_BOOT_DEST_STRLEN);
	memcpy(xdev->partition_name, partition,
		(name_len >= MXLK_BOOT_DEST_STRLEN) ?
			(MXLK_BOOT_DEST_STRLEN - 1) : name_len);
	xdev->partition_offset = 0;

	memcpy_toio(xdev->mxlk.io_comm->mf_dest, xdev->partition_name,
		    MXLK_BOOT_DEST_STRLEN);

start_error:
	mxlk_boot_access_exit(xdev);
	return rc;
}

static int mxlk_pci_flash_partition_send(struct mxlk_pcie *xdev,
					 const void *data, size_t size)
{
	int rc = 0;
	int size_left = size;

	rc = mxlk_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (!xdev->dma_buf) {
		xdev->dma_buf_offset = 0;
		xdev->dma_buf = kmalloc(SECTION_SIZE, GFP_KERNEL);
		if (!xdev->dma_buf) {
			rc = -ENOMEM;
			goto send_error;
		}
	}

	while (size_left) {
		size = (size > (SECTION_SIZE - xdev->dma_buf_offset)) ?
			(SECTION_SIZE - xdev->dma_buf_offset) : size;
		memcpy(xdev->dma_buf + xdev->dma_buf_offset,
			data, size);
		xdev->dma_buf_offset += size;
		size_left -= size;
		data += size;

		if (xdev->dma_buf_offset == SECTION_SIZE)
			rc = mxlk_recovery_send_left(xdev, false);
	}

send_error:
	mxlk_boot_access_exit(xdev);
	return rc;
}

static int mxlk_pci_flash_done(struct mxlk_pcie *xdev)
{
	int rc = 0;

	rc = mxlk_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = mxlk_recovery_send_left(xdev, true);
		if (rc)
			goto done_error;
	}

	iowrite32(MXLK_BOOT_STATUS_DONE, &xdev->mxlk.io_comm->mf_ready);

	kfree(xdev->dma_buf);
	xdev->dma_buf = NULL;
	xdev->dma_buf_offset = 0;

done_error:
	mxlk_boot_access_exit(xdev);
	return rc;
}

static ssize_t partition_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;

	struct mxlk_pcie *xdev = dev_get_drvdata(dev);

	rc = mxlk_pci_flash_partition_start(xdev, buf, count);
	if (rc) {
		dev_err(dev, "failed to flash partition\n");
		return rc;
	}

	return count;
}
static DEVICE_ATTR_WO(partition);

static ssize_t create_partitions_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int rc;
	long value;
	struct mxlk_pcie *xdev = dev_get_drvdata(dev);

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	if (value) {
		rc = mxlk_pci_flash_gpt_table(xdev);
		if (rc) {
			dev_err(dev, "failed to flash gpt table\n");
			return rc;
		}
	}

	return count;
}
static DEVICE_ATTR_WO(create_partitions);

static ssize_t erase_partition_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int rc;
	long value;
	struct mxlk_pcie *xdev = dev_get_drvdata(dev);

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	if (value) {
		rc = mxlk_pci_erase_partition(xdev, xdev->partition_name,
					      MXLK_BOOT_DEST_STRLEN);
		if (rc) {
			dev_err(dev, "failed to erase partition\n");
			return rc;
		}
	}

	return count;
}
static DEVICE_ATTR_WO(erase_partition);

static ssize_t recovery_done_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int rc;
	long value;
	struct mxlk_pcie *xdev = dev_get_drvdata(dev);

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	if (value) {
		rc = mxlk_pci_flash_done(xdev);
		if (rc) {
			dev_err(dev, "failed to recover\n");
			return rc;
		}
	}

	return count;
}
static DEVICE_ATTR_WO(recovery_done);

static ssize_t recov_write_data(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attr,
				char *buf, loff_t offset, size_t count)
{
	int rc;
	struct device *dev = kobj_to_dev(kobj);
	struct mxlk_pcie *xdev = dev_get_drvdata(dev);

	rc = mxlk_pci_flash_partition_send(xdev, buf, count);
	if (rc) {
		dev_err(dev, "failed to flash partition\n");
		return rc;
	}

	return count;
}
static BIN_ATTR(image_data, 0644, NULL, recov_write_data, 0);

static struct attribute *recovery_attrs[] = {
	&dev_attr_partition.attr,
	&dev_attr_create_partitions.attr,
	&dev_attr_erase_partition.attr,
	&dev_attr_recovery_done.attr,
	NULL,
};

static struct bin_attribute *recovery_bin_attrs[] = {
	&bin_attr_image_data,
	NULL,
};

static const struct attribute_group recovery_group = {
	.attrs = recovery_attrs,
	.bin_attrs = recovery_bin_attrs,
};

static const struct attribute_group *recovery_groups[] = {
	&recovery_group,
	NULL,
};

static int mxlk_pci_setup_recovery_sysfs(struct mxlk_pcie *xdev)
{
	return sysfs_create_groups(&xdev->pci->dev.kobj, recovery_groups);
}

static void mxlk_pci_cleanup_recovery_sysfs(struct mxlk_pcie *xdev)
{
	sysfs_remove_groups(&xdev->pci->dev.kobj, recovery_groups);
}
