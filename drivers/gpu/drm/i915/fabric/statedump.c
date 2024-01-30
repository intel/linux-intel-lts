// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2022 Intel Corporation.
 *
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "debugfs.h"
#include "iaf_drv.h"
#include "io.h"
#include "mbdb.h"
#include "ops.h"
#include "statedump.h"

#define STATEDUMP_FILE_NAME "statedump"

#define STATEDUMP_MAX_OFFSETS 64

/* State Dump descriptor fields */
#define DUMP_START_ADDR GENMASK(31, 0)
#define DUMP_LENGTH     GENMASK(55, 32)
#define DUMP_COMMENT    GENMASK(59, 56)
#define DUMP_TYPE       GENMASK(63, 60)

#define SD_MAGIC   0x46575344
#define SD_VERSION 1

#define DUMP_VERSION GENMASK(31, 0)
#define DUMP_MAGIC   GENMASK(63, 32)

enum DUMP_TYPE_t {
	DUMP_TYPE_XX,
	DUMP_TYPE_32,
	DUMP_TYPE_64,
	DUMP_TYPE_32_OF_64,
};

struct statedump_rsp {
	struct list_head rsp_list_link;
	struct mbdb_statedump_rsp mbdb_rsp;
};

static int statedump_read_desc_data(struct fsubdev *sd, u8 *dst, size_t desc_len, u32 addr)
{
	do {
		size_t xfer_len = min(MBOX_READ_DATA_SIZE_IN_BYTES, desc_len);
		int ret;

		ret = ops_csr_raw_read_nolock(sd, addr, xfer_len, dst);
		if (ret)
			return ret;

		dst += xfer_len;
		addr += xfer_len;
		desc_len -= xfer_len;
	} while (desc_len);

	return 0;
}

static u8 *statedump_process_desc_rsp(struct fsubdev *sd, struct statedump_rsp *rsp, u8 *data)
{
	u64 *desc;
	u16 desc_count;

	for (desc = rsp->mbdb_rsp.descs, desc_count = rsp->mbdb_rsp.desc_count; desc_count--;
	     desc++) {
		u32 addr = FIELD_GET(DUMP_START_ADDR, *desc);
		size_t desc_len = FIELD_GET(DUMP_LENGTH, *desc);
		int ret;

		if (FIELD_GET(DUMP_TYPE, *desc) == DUMP_TYPE_XX)
			break;

		memcpy(data, desc, sizeof(*desc));
		data += sizeof(*desc);

		if (!desc_len)
			continue;

		ret = statedump_read_desc_data(sd, data, desc_len, addr);
		if (likely(!ret)) {
			data += desc_len;
		} else {
			if (ret != MBOX_RSP_STATUS_DENIED)
				return ERR_PTR(ret);

			sd_warn(sd, "Access Denied for addr 0x%08x\n", addr);
			data -= sizeof(*desc);
			sd->statedump.blob.size -= sizeof(*desc) + desc_len;
		}
	}

	return data;
}

static int statedump_process_desc_list(struct fsubdev *sd, struct list_head *rsp_list)
{
	u8 *data = sd->statedump.blob.data;
	struct statedump_rsp *rsp;
	__be64 magic_ver = cpu_to_be64(FIELD_PREP(DUMP_VERSION, SD_VERSION) |
				       FIELD_PREP(DUMP_MAGIC, SD_MAGIC));

	memcpy(data, &magic_ver, sizeof(magic_ver));
	data += sizeof(magic_ver);

	memcpy(data, &sd->fw_version, sizeof(sd->fw_version));
	data += sizeof(sd->fw_version);

	sd->statedump.blob.size += sizeof(magic_ver) + sizeof(sd->fw_version);

	list_for_each_entry(rsp, rsp_list, rsp_list_link) {
		data = statedump_process_desc_rsp(sd, rsp, data);
		if (IS_ERR(data))
			return PTR_ERR(data);
	}

	return 0;
}

static int statedump_build_desc_list(struct fsubdev *sd, struct list_head *rsp_list)
{
	struct state_dump *statedump = &sd->statedump;
	u32 offset;
	int ret;

	statedump->blob.size = 0;

	for (offset = 0; offset < STATEDUMP_MAX_OFFSETS; offset++) {
		struct statedump_rsp *rsp;
		u64 *desc;
		u16 desc_count;

		rsp = kzalloc(sizeof(*rsp), GFP_KERNEL);
		if (!rsp)
			return -ENOMEM;

		INIT_LIST_HEAD(&rsp->rsp_list_link);
		list_add_tail(&rsp->rsp_list_link, rsp_list);

		ret = ops_statedump(sd, offset, &rsp->mbdb_rsp);
		if (ret)
			return ret;

		for (desc = rsp->mbdb_rsp.descs, desc_count = rsp->mbdb_rsp.desc_count;
		     desc_count--; desc++) {
			if (FIELD_GET(DUMP_TYPE, *desc) == DUMP_TYPE_XX)
				goto alloc_data;

			statedump->blob.size += sizeof(*desc) + FIELD_GET(DUMP_LENGTH, *desc);
		}
	}

	if (offset == STATEDUMP_MAX_OFFSETS)
		sd_warn(sd, "Not able to obtain all state dump data descriptors\n");

alloc_data:
	statedump->blob.data = vzalloc(statedump->blob.size);
	if (statedump->blob.data)
		return 0;

	return -ENOMEM;
}

static int statedump_process_dump(struct fsubdev *sd)
{
	struct list_head rsp_list;
	struct statedump_rsp *rsp;
	struct statedump_rsp *tmp;
	int ret;

	INIT_LIST_HEAD(&rsp_list);

	ret = statedump_build_desc_list(sd, &rsp_list);
	if (ret)
		goto cleanup;

	ret = statedump_process_desc_list(sd, &rsp_list);
	if (ret) {
		vfree(sd->statedump.blob.data);
		sd->statedump.blob.data = NULL;
		sd->statedump.blob.size = 0;
	}

cleanup:
	list_for_each_entry_safe(rsp, tmp, &rsp_list, rsp_list_link) {
		list_del(&rsp->rsp_list_link);
		kfree(rsp);
	}

	return ret;
}

static int statedump_open(struct inode *inode, struct file *file)
{
	void __iomem *cport_init_ctrl_reg_addr;
	struct state_dump *statedump;
	u64 cport_init_ctrl_reg_val;
	struct fsubdev *sd;
	int ret;

	if (!inode->i_private)
		return -EINVAL;

	sd = inode->i_private;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	statedump = &sd->statedump;
	file->private_data = statedump;

	ret = down_killable(&statedump->state_dump_sem);
	if (ret)
		return ret;

	ret = down_write_killable(&statedump->state_dump_mbdb_sem);
	if (ret) {
		up(&statedump->state_dump_sem);
		return ret;
	}

	mutex_lock(&sd->cport_init_ctrl_reg_lock);

	cport_init_ctrl_reg_addr = sd->csr_base + CPORT_INIT_CTRL_ADDR;
	cport_init_ctrl_reg_val = readq(cport_init_ctrl_reg_addr);

	writeq(cport_init_ctrl_reg_val | RISC_NMI_BIT, cport_init_ctrl_reg_addr);

	ret = statedump_process_dump(sd);

	writeq(cport_init_ctrl_reg_val, cport_init_ctrl_reg_addr);

	mutex_unlock(&sd->cport_init_ctrl_reg_lock);

	up_write(&statedump->state_dump_mbdb_sem);

	if (ret)
		up(&statedump->state_dump_sem);

	return ret;
}

static int statedump_release(struct inode *inode, struct file *file)
{
	if (inode->i_private) {
		struct fsubdev *sd = inode->i_private;
		struct state_dump *statedump = &sd->statedump;

		vfree(statedump->blob.data);
		statedump->blob.size = 0;
		statedump->blob.data = NULL;
		up(&statedump->state_dump_sem);
	}

	return 0;
}

static const struct file_operations statedump_fops = {
	.owner = THIS_MODULE,
	.open = statedump_open,
	.read = blob_read,
	.release = statedump_release,
	.llseek = default_llseek,
};

void statedump_node_init(struct fsubdev *sd)
{
	sema_init(&sd->statedump.state_dump_sem, 1);
	init_rwsem(&sd->statedump.state_dump_mbdb_sem);
	sd->statedump.blob.size = 0;
	sd->statedump.blob.data = NULL;

	debugfs_create_file(STATEDUMP_FILE_NAME, 0400, sd->debugfs_dir, sd, &statedump_fops);
}
