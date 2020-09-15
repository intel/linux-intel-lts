// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/
#include <linux/firmware.h>
#include <linux/delay.h>

#include "pci.h"
#include "../common/boot.h"

#define SECTION_SIZE (1 * 1024 * 1024)

#define BOOT_TIMEOUT 60000
#define STATUS_TIMEOUT 5000
#define ERASE_TIMEOUT 30000

/* function declarations */
int intel_xpcie_pci_boot_device(u32 id, const char *binary_name);
int intel_xpcie_pci_setup_recovery_sysfs(struct xpcie_dev *xdev);
void intel_xpcie_pci_cleanup_recovery_sysfs(struct xpcie_dev *xdev);
enum xpcie_stage intel_xpcie_check_magic(struct xpcie_dev *xdev);

enum xpcie_stage intel_xpcie_check_magic(struct xpcie_dev *xdev)
{
	char magic[XPCIE_BOOT_MAGIC_STRLEN];

	memcpy_fromio(magic, xdev->xpcie.io_comm->magic,
		      XPCIE_BOOT_MAGIC_STRLEN);

	if (strlen(magic) == 0)
		return STAGE_UNINIT;

	if (!strncmp(magic, XPCIE_BOOT_MAGIC_ROM,
		     strlen(XPCIE_BOOT_MAGIC_ROM)))
		return STAGE_ROM;

	if (!strncmp(magic, XPCIE_BOOT_MAGIC_EMMC,
		     strlen(XPCIE_BOOT_MAGIC_EMMC)))
		return STAGE_ROM;

	if (!strncmp(magic, XPCIE_BOOT_MAGIC_BL2,
		     strlen(XPCIE_BOOT_MAGIC_BL2)))
		return STAGE_BL2;

	if (!strncmp(magic, XPCIE_BOOT_MAGIC_UBOOT,
		     strlen(XPCIE_BOOT_MAGIC_UBOOT)))
		return STAGE_UBOOT;

	if (!strncmp(magic, XPCIE_BOOT_MAGIC_RECOV,
		     strlen(XPCIE_BOOT_MAGIC_RECOV)))
		return STAGE_RECOV;

	if (!strncmp(magic, XPCIE_BOOT_MAGIC_YOCTO,
		     strlen(XPCIE_BOOT_MAGIC_YOCTO)))
		return STAGE_OS;

	return STAGE_UNINIT;
}

static int xpcie_device_wait_status(struct xpcie_dev *xdev, u32 image_id,
				    u32 timeout_ms)
{
	u32 status = XPCIE_BOOT_STATUS_START;
	int count = 0;

	if (timeout_ms == 0)
		timeout_ms = STATUS_TIMEOUT;

	iowrite32(image_id, &xdev->xpcie.io_comm->mf_ready);

	while (status != XPCIE_BOOT_STATUS_DOWNLOADED) {
		mdelay(1);
		if (++count > timeout_ms) {
			dev_err(&xdev->pci->dev, "operation takes too long.\n");
			return -ETIME;
		}

		status = ioread32(&xdev->xpcie.io_comm->mf_ready);

		switch (status) {
		case XPCIE_BOOT_STATUS_INVALID:
			dev_err(&xdev->pci->dev,
				"the firmware image data is invalid.\n");
			return -EINVAL;
		case XPCIE_BOOT_STATUS_ERROR:
			dev_err(&xdev->pci->dev,
				"failed to download firmware image.\n");
			return -EINVAL;
		default:
			break;
		}
	}

	return 0;
}

static void xpcie_device_enable_irq(struct xpcie_dev *xdev)
{
	iowrite32(XPCIE_INT_ENABLE, &xdev->xpcie.io_comm->int_enable);
	iowrite32(~XPCIE_INT_MASK, &xdev->xpcie.io_comm->int_mask);
}

static int xpcie_device_transfer(struct xpcie_dev *xdev, u32 image_id,
				 dma_addr_t addr, size_t size)
{
	int rc;

	_iowrite64(addr, &xdev->xpcie.io_comm->mf_start);
	iowrite32(size, &xdev->xpcie.io_comm->mf_len);

	if (image_id == XPCIE_BOOT_RAW_ID)
		_iowrite64(xdev->partition_offset,
			   &xdev->xpcie.io_comm->mf_offset);

	rc = xpcie_device_wait_status(xdev, image_id, 0);
	if (!rc)
		xdev->partition_offset += size;

	return rc;
}

static int xpcie_device_download_common(struct xpcie_dev *xdev, u32 image_id,
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

		rc = xpcie_device_transfer(xdev, image_id, phys_addr, size);

		dma_unmap_single(dev, phys_addr, size, DMA_TO_DEVICE);

		if (rc)
			return rc;

		size_left -= size;
		buf += size;
	}

	return rc;
}

static int xpcie_device_download_firmware(struct xpcie_dev *xdev, u32 image_id,
					  const char *fw_image)
{
	const struct firmware *firmware;
	struct device *dev = &xdev->pci->dev;
	int rc = 0;

	switch (image_id) {
	case XPCIE_BOOT_FIP_ID:
	case XPCIE_BOOT_BOOT_ID:
	case XPCIE_BOOT_SYSTEM_ID:
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

	rc = xpcie_device_download_common(xdev, image_id, firmware->data,
					  firmware->size, false);

	kfree(xdev->dma_buf);
	xdev->dma_buf = NULL;

firmware_cleanup:
	release_firmware(firmware);

	return rc;
}

static int xpcie_device_flashless_boot(struct xpcie_dev *xdev)
{
	if (xpcie_device_download_firmware(xdev, XPCIE_BOOT_BOOT_ID,
					   xdev->fw_name)) {
		dev_err(&xdev->pci->dev, "failed to download boot image\n");
		return -EIO;
	}

	iowrite32(XPCIE_BOOT_STATUS_DONE, &xdev->xpcie.io_comm->mf_ready);

	return 0;
}

static int xpcie_device_fip(struct xpcie_dev *xdev)
{
	if (xpcie_device_download_firmware(xdev, XPCIE_BOOT_FIP_ID,
					   xdev->fw_name)) {
		dev_err(&xdev->pci->dev, "failed to download FIP image\n");
		return -EIO;
	}

	iowrite32(XPCIE_BOOT_STATUS_DONE, &xdev->xpcie.io_comm->mf_ready);

	return 0;
}

#define intel_xpcie_wait_event(cond)					\
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

int intel_xpcie_pci_boot_device(u32 id, const char *binary_name)
{
	int rc = 0;
	u32 expected = XPCIE_STATUS_ERROR;
	struct xpcie_dev *xdev;

	xdev = intel_xpcie_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	if (xdev->xpcie.status == XPCIE_STATUS_OFF) {
		rc = -ENODEV;
		goto boot_cleanup;
	}

	strncpy(xdev->fw_name, binary_name, XPCIE_MAX_NAME_LEN - 1);

	if (!xdev->irq_enabled) {
		xpcie_device_enable_irq(xdev);
		xdev->irq_enabled = true;

		rc = intel_xpcie_wait_event(xdev->xpcie.status !=
					    XPCIE_STATUS_UNINIT);
		if (rc)
			goto boot_cleanup;
	}

	switch (xdev->xpcie.status) {
	case XPCIE_STATUS_BOOT_FW:
		xdev->xpcie.status = XPCIE_STATUS_BOOT_PRE_OS;
		rc = xpcie_device_fip(xdev);
		goto boot_cleanup;
	case XPCIE_STATUS_BOOT_PRE_OS:
		/*
		 * This fake stage is to avoid boot timeout after flashing FIP
		 * if the function only returns after entering BOOT_OS stage.
		 * It's because if host doesn't support PCIe hot plug the
		 * reset after flashing FIP cannot be detected so driver won't
		 * know the stage change at all.
		 * So let boot call on FIP return early and check stage in next
		 * boot call for OS.
		 */
		rc = intel_xpcie_wait_event(xdev->xpcie.status ==
					    XPCIE_STATUS_BOOT_OS);
		if (rc)
			goto boot_cleanup;
		rc = xpcie_device_flashless_boot(xdev);
		if (rc)
			goto boot_cleanup;
		expected = XPCIE_STATUS_READY;
		break;
	case XPCIE_STATUS_BOOT_OS:
		rc = xpcie_device_flashless_boot(xdev);
		if (rc)
			goto boot_cleanup;
		expected = XPCIE_STATUS_READY;
		break;
	case XPCIE_STATUS_READY:
	case XPCIE_STATUS_RUN:
		rc = 0;
		goto boot_cleanup;
	case XPCIE_STATUS_RECOVERY:
	case XPCIE_STATUS_ERROR:
	default:
		rc = -EIO;
		goto boot_cleanup;
	}

	rc = intel_xpcie_wait_event((xdev->xpcie.status == expected) ||
				    (xdev->xpcie.status ==
				     XPCIE_STATUS_RECOVERY));

	if (xdev->xpcie.status == XPCIE_STATUS_RECOVERY)
		rc = -EIO;

boot_cleanup:
	mutex_unlock(&xdev->lock);

	return rc;
}

static int intel_xpcie_boot_access_enter(struct xpcie_dev *xdev)
{
	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;
	if (xdev->xpcie.status != XPCIE_STATUS_RECOVERY) {
		mutex_unlock(&xdev->lock);
		return -EROFS;
	}
	return 0;
}

static void intel_xpcie_boot_access_exit(struct xpcie_dev *xdev)
{
	mutex_unlock(&xdev->lock);
}

static int intel_xpcie_recovery_send_left(struct xpcie_dev *xdev,
					  bool reset_offset)
{
	int rc;

	rc = xpcie_device_download_common(xdev, XPCIE_BOOT_RAW_ID, NULL,
					  xdev->dma_buf_offset, true);
	xdev->dma_buf_offset = 0;
	if (reset_offset)
		xdev->partition_offset = 0;

	return rc;
}

static int intel_xpcie_pci_flash_gpt_table(struct xpcie_dev *xdev)
{
	int rc = 0;

	rc = intel_xpcie_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = intel_xpcie_recovery_send_left(xdev, true);
		if (rc)
			goto create_error;
	}

	rc = xpcie_device_wait_status(xdev, XPCIE_BOOT_FLASH_ID, 0);

create_error:
	intel_xpcie_boot_access_exit(xdev);
	return rc;
}

static int intel_xpcie_pci_erase_partition(struct xpcie_dev *xdev,
					   const char *partition, size_t len)
{
	int rc = 0;

	rc = intel_xpcie_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = intel_xpcie_recovery_send_left(xdev, true);
		if (rc)
			goto erase_error;
	}

	memcpy_toio(xdev->xpcie.io_comm->mf_dest, partition, len);

	rc = xpcie_device_wait_status(xdev, XPCIE_BOOT_ERASE_ID, ERASE_TIMEOUT);

erase_error:
	intel_xpcie_boot_access_exit(xdev);
	return rc;
}

static int intel_xpcie_pci_flash_partition_start(struct xpcie_dev *xdev,
						 const char *partition,
						 size_t name_len)
{
	int rc = 0;

	rc = intel_xpcie_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = intel_xpcie_recovery_send_left(xdev, true);
		if (rc)
			goto start_error;
	}

	memset(xdev->partition_name, 0, XPCIE_BOOT_DEST_STRLEN);
	memcpy(xdev->partition_name, partition,
	       (name_len >= XPCIE_BOOT_DEST_STRLEN) ?
			(XPCIE_BOOT_DEST_STRLEN - 1) : name_len);
	xdev->partition_offset = 0;

	memcpy_toio(xdev->xpcie.io_comm->mf_dest, xdev->partition_name,
		    XPCIE_BOOT_DEST_STRLEN);

start_error:
	intel_xpcie_boot_access_exit(xdev);
	return rc;
}

static int intel_xpcie_pci_flash_partition_send(struct xpcie_dev *xdev,
						const void *data, size_t size)
{
	int rc = 0;
	int size_left = size;

	rc = intel_xpcie_boot_access_enter(xdev);
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
			rc = intel_xpcie_recovery_send_left(xdev, false);
	}

send_error:
	intel_xpcie_boot_access_exit(xdev);
	return rc;
}

static int intel_xpcie_pci_flash_done(struct xpcie_dev *xdev)
{
	int rc = 0;

	rc = intel_xpcie_boot_access_enter(xdev);
	if (rc)
		return rc;

	if (xdev->dma_buf_offset) {
		rc = intel_xpcie_recovery_send_left(xdev, true);
		if (rc)
			goto done_error;
	}

	iowrite32(XPCIE_BOOT_STATUS_DONE, &xdev->xpcie.io_comm->mf_ready);

	kfree(xdev->dma_buf);
	xdev->dma_buf = NULL;
	xdev->dma_buf_offset = 0;

done_error:
	intel_xpcie_boot_access_exit(xdev);
	return rc;
}

static ssize_t partition_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int rc;

	struct xpcie_dev *xdev = dev_get_drvdata(dev);

	rc = intel_xpcie_pci_flash_partition_start(xdev, buf, count);
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
	struct xpcie_dev *xdev = dev_get_drvdata(dev);

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	if (value) {
		rc = intel_xpcie_pci_flash_gpt_table(xdev);
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
	struct xpcie_dev *xdev = dev_get_drvdata(dev);

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	if (value) {
		rc = intel_xpcie_pci_erase_partition(xdev, xdev->partition_name,
						     XPCIE_BOOT_DEST_STRLEN);
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
	struct xpcie_dev *xdev = dev_get_drvdata(dev);

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return rc;

	if (value) {
		rc = intel_xpcie_pci_flash_done(xdev);
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
	struct xpcie_dev *xdev = dev_get_drvdata(dev);

	rc = intel_xpcie_pci_flash_partition_send(xdev, buf, count);
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

int intel_xpcie_pci_setup_recovery_sysfs(struct xpcie_dev *xdev)
{
	return sysfs_create_groups(&xdev->pci->dev.kobj, recovery_groups);
}

void intel_xpcie_pci_cleanup_recovery_sysfs(struct xpcie_dev *xdev)
{
	sysfs_remove_groups(&xdev->pci->dev.kobj, recovery_groups);
}
