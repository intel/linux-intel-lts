// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright © 2023 Intel Corporation
 */

#include <linux/list.h>

#include "i915_vfio_pci.h"

#define BITSTREAM_MAGIC 0x4f49465635313949ULL
#define BITSTREAM_VERSION 0x1

struct i915_vfio_data_device_desc {
	/** @magic: constant, driver specific value */
	u64 magic;
	/** @version: device data version */
	u64 version;
	u16 vendor;
	u16 device;
	u32 rsvd;
	/** @flags: optional flags */
	u64 flags;
} __packed;

enum i915_vfio_pci_migration_data_type {
	I915_VFIO_DATA_DESC = 0,
	I915_VFIO_DATA_GGTT,
	I915_VFIO_DATA_GUC,
	I915_VFIO_DATA_DONE,
};

static const char *i915_vfio_data_type_str(enum i915_vfio_pci_migration_data_type type)
{
	switch (type) {
	case I915_VFIO_DATA_DESC: return "DESC";
	case I915_VFIO_DATA_GGTT: return "GGTT";
	case I915_VFIO_DATA_GUC: return "GUC";
	case I915_VFIO_DATA_DONE: return "DONE";
	default: return "";
	}
}

static int
__i915_vfio_produce(struct i915_vfio_pci_migration_file *migf, unsigned int tile, u32 type)
{
	struct i915_vfio_pci_core_device *i915_vdev = migf->i915_vdev;
	struct device *dev = i915_vdev_to_dev(i915_vdev);
	const struct i915_vfio_pci_resource_ops *ops;
	struct i915_vfio_pci_migration_data *data;
	size_t size;
	void *buf;
	int ret;

	switch (type) {
	case I915_VFIO_DATA_GGTT:
		ops = &i915_vdev->pf_ops->ggtt;
		break;
	case I915_VFIO_DATA_GUC:
		ops = &i915_vdev->pf_ops->fw;
		break;
	default:
		return -EINVAL;
	}

	size = ops->size(i915_vdev->pf, i915_vdev->vfid, tile);
	if (!size) {
		dev_dbg(dev, "Skipping %s for tile%u, ret=%ld\n",
			i915_vfio_data_type_str(type), tile, size);

		return 0;
	}
	buf = kvmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto out_free_buf;
	}

	ret = ops->save(i915_vdev->pf, i915_vdev->vfid, tile, buf, size);
	if (ret < 0)
		goto out_free_data;

	data->hdr.type = type;
	data->hdr.tile = tile;
	data->hdr.offset = 0;
	data->hdr.size = size;
	data->hdr.flags = 0;

	data->pos = 0;
	data->buf = buf;

	dev_dbg(dev, "Producing %s for tile%u, size=%ld\n",
		i915_vfio_data_type_str(type), tile, size);

	list_add(&data->link, &migf->save_data);

	return 0;

out_free_data:
	kfree(data);
out_free_buf:
	kvfree(buf);

	return ret;
}

static int __i915_vfio_consume(struct i915_vfio_pci_migration_file *migf, unsigned int tile,
			       u32 type, const char __user *ubuf, size_t len)
{
	struct i915_vfio_pci_core_device *i915_vdev = migf->i915_vdev;
	struct i915_vfio_pci_migration_data *data = &migf->resume_data;
	const struct i915_vfio_pci_resource_ops *ops;
	int ret;

	switch (type) {
	case I915_VFIO_DATA_GGTT:
		ops = &i915_vdev->pf_ops->ggtt;
		break;
	case I915_VFIO_DATA_GUC:
		ops = &i915_vdev->pf_ops->fw;
		break;
	default:
		return -EINVAL;
	}

	if (data->pos + len > data->hdr.size)
		return -EINVAL;

	if (!data->buf) {
		data->buf = kvmalloc(data->hdr.size, GFP_KERNEL);
		if (!data->buf)
			return -ENOMEM;
	}

	if (migf->copy_from(data->buf + data->pos, ubuf, len)) {
		ret = -EFAULT;
		goto out_free;
	}

	if (data->pos + len == data->hdr.size) {
		ret = ops->load(i915_vdev->pf, i915_vdev->vfid, tile, data->buf, data->hdr.size);
		if (ret)
			goto out_free;
	}

	return 0;

out_free:
	kvfree(data->buf);

	return ret;
}

#define __resource(x, type) \
static int \
i915_vfio_produce_##x(struct i915_vfio_pci_migration_file *migf, unsigned int tile) \
{ \
	return __i915_vfio_produce(migf, tile, type); \
} \
static int \
i915_vfio_consume_##x(struct i915_vfio_pci_migration_file *migf, \
		      unsigned int tile, const char __user *ubuf, size_t len) \
{ \
	return __i915_vfio_consume(migf, tile, type, ubuf, len); \
}

__resource(ggtt, I915_VFIO_DATA_GGTT);
__resource(fw, I915_VFIO_DATA_GUC);

static int i915_vfio_produce_desc(struct i915_vfio_pci_migration_file *migf)
{
	struct i915_vfio_pci_migration_data *data;
	struct i915_vfio_data_device_desc desc;
	void *buf;

	desc.magic = BITSTREAM_MAGIC;
	desc.version = BITSTREAM_VERSION;
	desc.vendor = i915_vdev_to_pdev(migf->i915_vdev)->vendor;
	desc.device = i915_vdev_to_pdev(migf->i915_vdev)->device;
	desc.flags = 0x0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	buf = kvmalloc(sizeof(desc), GFP_KERNEL);
	if (!buf) {
		kfree(data);
		return -ENOMEM;
	}

	data->hdr.type = I915_VFIO_DATA_DESC;
	data->hdr.tile = 0;
	data->hdr.offset = 0;
	data->hdr.size = sizeof(desc);
	data->hdr.flags = 0;
	data->pos = 0;
	data->buf = buf;

	memcpy(data->buf, &desc, sizeof(desc));

	list_add(&data->link, &migf->save_data);

	return 0;
}

static int i915_vfio_consume_desc(struct i915_vfio_pci_migration_file *migf,
				  const char __user *ubuf, size_t len)
{
	struct i915_vfio_pci_migration_header *hdr = &migf->resume_data.hdr;
	struct i915_vfio_data_device_desc desc;

	if (hdr->size != sizeof(desc))
		return -EINVAL;

	if (sizeof(desc) != len)
		return -EINVAL;

	if (migf->copy_from(&desc, ubuf, len))
		return -EFAULT;

	if (desc.magic != BITSTREAM_MAGIC)
		return -EINVAL;

	if (desc.version != BITSTREAM_VERSION)
		return -EINVAL;

	if (desc.vendor != i915_vdev_to_pdev(migf->i915_vdev)->vendor)
		return -EINVAL;

	if (desc.device != i915_vdev_to_pdev(migf->i915_vdev)->device)
		return -EINVAL;

	return 0;
}

static int
i915_vfio_pci_produce_data(struct i915_vfio_pci_migration_file *migf,
			   enum i915_vfio_pci_migration_data_type type, unsigned int tile)
{
	switch (type) {
	case I915_VFIO_DATA_DESC:
		if (tile)
			return 0;
		return i915_vfio_produce_desc(migf);
	case I915_VFIO_DATA_GGTT:
		return i915_vfio_produce_ggtt(migf, tile);
	case I915_VFIO_DATA_GUC:
		return i915_vfio_produce_fw(migf, tile);
	default:
		return -EINVAL;
	}
}

static ssize_t
i915_vfio_consume_data(struct i915_vfio_pci_migration_file *migf, const char __user *ubuf,
		       size_t len)
{
	struct i915_vfio_pci_migration_header *hdr = &migf->resume_data.hdr;

	switch (hdr->type) {
	case I915_VFIO_DATA_DESC:
		return i915_vfio_consume_desc(migf, ubuf, len);
	case I915_VFIO_DATA_GGTT:
		return i915_vfio_consume_ggtt(migf, hdr->tile, ubuf, len);
	case I915_VFIO_DATA_GUC:
		return i915_vfio_consume_fw(migf, hdr->tile, ubuf, len);
	default:
		return -EINVAL;
	}
}

static void i915_vfio_save_data_free(struct i915_vfio_pci_migration_data *data)
{
	list_del_init(&data->link);
	kvfree(data->buf);
	kfree(data);
}

void i915_vfio_save_data_release(struct i915_vfio_pci_migration_file *migf)
{
	struct i915_vfio_pci_migration_data *data, *next;

	if (!migf)
		return;

	list_for_each_entry_safe(data, next, &migf->save_data, link)
		i915_vfio_save_data_free(data);
}

static void i915_vfio_resume_data_free(struct i915_vfio_pci_migration_data *data)
{
	data->hdr_processed = false;
	data->pos = 0;

	kvfree(data->buf);
	data->buf = NULL;
}

int i915_vfio_pci_produce_save_data(struct i915_vfio_pci_migration_file *migf)
{
	enum i915_vfio_pci_migration_data_type type;
	unsigned int tile;
	int ret;

	for (tile = 0; tile < I915_VFIO_MAX_TILE; tile++) {
		for (type = I915_VFIO_DATA_DESC; type < I915_VFIO_DATA_DONE; type++) {
			ret = i915_vfio_pci_produce_data(migf, type, tile);
			if (ret)
				goto out;
		}
	}

	return 0;

out:
	i915_vfio_save_data_release(migf);
	return ret;
}

ssize_t i915_vfio_data_read(struct i915_vfio_pci_migration_file *migf, char __user *ubuf,
			    size_t len)
{
	struct i915_vfio_pci_migration_data *data;
	size_t len_remain, len_hdr;
	int ret;

	data = list_first_entry_or_null(&migf->save_data, typeof(*data), link);
	if (!data)
		return 0;

	if (!data->hdr_processed) {
		if (len < sizeof(data->hdr))
			return -EINVAL;

		ret = migf->copy_to(ubuf, &data->hdr, sizeof(data->hdr));
		if (ret)
			return -EFAULT;

		len_hdr = sizeof(data->hdr);
		ubuf += sizeof(data->hdr);
		data->hdr_processed = true;
	} else {
		len_hdr = 0;
	}

	len_remain = len_hdr + data->hdr.size - data->pos;
	len = min(len, len_remain);

	if (migf->copy_to(ubuf, data->buf + data->pos, len - len_hdr))
		return -EFAULT;

	if (len < len_remain)
		data->pos += len - len_hdr;
	else
		i915_vfio_save_data_free(data);

	return len;
}

ssize_t i915_vfio_data_write(struct i915_vfio_pci_migration_file *migf, const char __user *ubuf,
			     size_t len)
{
	struct i915_vfio_pci_migration_data *data = &migf->resume_data;
	size_t len_remain, len_hdr;
	int ret;

	if (!data->hdr_processed) {
		if (len < sizeof(data->hdr))
			return -EINVAL;

		if (migf->copy_from(&data->hdr, ubuf, sizeof(data->hdr)))
			return -EFAULT;

		len_hdr = sizeof(data->hdr);
		ubuf += sizeof(data->hdr);
		data->hdr_processed = true;

		dev_dbg(i915_vdev_to_dev(migf->i915_vdev),
			"Consuming %s for tile%lld, size=%lld\n",
			i915_vfio_data_type_str(data->hdr.type), data->hdr.tile, data->hdr.size);
	} else {
		len_hdr = 0;
	}

	len_remain = len_hdr + data->hdr.size - data->pos;
	len = min(len, len_remain);

	ret = i915_vfio_consume_data(migf, ubuf, len - len_hdr);
	if (ret) {
		i915_vfio_resume_data_free(data);

		return ret;
	}

	if (len < len_remain)
		data->pos += len - len_hdr;
	else
		i915_vfio_resume_data_free(data);

	return len;
}

#if IS_ENABLED(CONFIG_I915_VFIO_PCI_TEST)
#include "test/data_test.c"
#endif
