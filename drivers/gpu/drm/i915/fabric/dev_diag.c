// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2022 Intel Corporation.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/compiler.h>
#include <linux/compiler_types.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/stringify.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include "debugfs.h"
#include "dev_diag.h"
#include "fw.h"
#include "io.h"
#include "mbdb.h"
#include "ops.h"
#include "port.h"
#include "routing_engine.h"
#include "statedump.h"

#define CAPABILITIES_FILE_NAME "capabilities"

static int capabilities_open(struct inode *inode, struct file *file)
{
	/*
	 * New strings are placed here along with the associated index into capabilities_index when
	 * new capabilities are added
	 */
	static const char * const capabilities[] = { "serdes_margin2", "throughput" };
	static const char *cap_fmt = "%s\n";
	enum capabilities_index {
		CAP_SERDES_MARGIN2,
		CAP_THROUGHPUT,
		CAPABILITIES_MAX,
	};
	struct fsubdev *sd = inode->i_private;
	struct capabilities_info {
		struct debugfs_blob_wrapper blob;
		char buf[0];
	} *info;
	size_t buf_offset;
	size_t buf_size;
	char *buf;
	u8 i;

	if (!sd)
		return -EINVAL;

	for (i = 0, buf_size = 0; i < CAPABILITIES_MAX; i++)
		buf_size += strlen(capabilities[i]);

	/* Add space for newlines and terminating NULL */
	buf_size += i + 1;

	info = kzalloc(sizeof(*info) + buf_size, GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	buf = info->buf;
	buf_offset = 0;

	/* Check firmware for availability */
	if (test_bit(MBOX_OP_CODE_SERDES_TX_DCC_MARGIN, sd->fw_version.supported_opcodes))
		buf_offset += scnprintf(buf + buf_offset, buf_size, cap_fmt,
					capabilities[CAP_SERDES_MARGIN2]);

	buf_offset += scnprintf(buf + buf_offset, buf_size, cap_fmt, capabilities[CAP_THROUGHPUT]);

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations capabilities_fops = {
	.owner = THIS_MODULE,
	.open = capabilities_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define FW_VERSION_FILE_NAME "fw_version"
#define FW_VERSION_BUF_SIZE 256

static int fw_version_open(struct inode *inode, struct file *file)
{
	struct fsubdev *sd = inode->i_private;
	struct mbdb_op_fw_version_rsp fw_version = {};
	struct fw_version_info {
		struct debugfs_blob_wrapper blob;
		char buf[FW_VERSION_BUF_SIZE];
	} *info;
	ssize_t element_count;
	size_t buf_offset;
	size_t buf_size;
	char *buf;
	int ret;

	if (!sd)
		return -EINVAL;

	ret = ops_fw_version(sd, &fw_version);
	if (ret == MBOX_RSP_STATUS_SEQ_NO_ERROR)
		ret = ops_fw_version(sd, &fw_version);
	if (ret)
		return ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;

	buf_offset = scnprintf(buf, buf_size, "MBox Version  : %d\n",
			       fw_version.mbox_version);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"Environment   : %s%s\n",
				(fw_version.environment & FW_VERSION_ENV_BIT)
				? "run-time" : "bootloader",
				(fw_version.environment & FW_VERSION_INIT_BIT)
				? ", ready" : "");
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"FW Version    : %s\n",
				fw_version.fw_version_string);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"OPs supported : 0x");

	for (element_count = ARRAY_SIZE(fw_version.supported_opcodes) - 1;
	     element_count >= 0; element_count--)
		buf_offset +=
			scnprintf(buf + buf_offset, buf_size - buf_offset,
				  "%016lx",
				  fw_version.supported_opcodes[element_count]);

	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset, "\n");

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations fw_version_fops = {
	.owner = THIS_MODULE,
	.open = fw_version_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define SWITCHINFO_FILE_NAME "switchinfo"
#define SWITCHINFO_BUF_SIZE 1024

static int switchinfo_open(struct inode *inode, struct file *file)
{
	struct fsubdev *sd = inode->i_private;
	struct mbdb_op_switchinfo switchinfo = {};
	struct switchinfo_info {
		struct debugfs_blob_wrapper blob;
		char buf[SWITCHINFO_BUF_SIZE];
	} *info;
	size_t buf_size;
	size_t buf_offset;
	char *buf;
	int ret;

	if (!sd)
		return -EINVAL;

	ret = ops_switchinfo_get(sd, &switchinfo);
	if (ret)
		return ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;

	buf_offset = scnprintf(buf, buf_size, "IAF GUID               : 0x%0llx\n",
			       switchinfo.guid);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"Num Ports              : %d\n",
				switchinfo.num_ports);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"SLL                    : %ld\n",
				switchinfo.slt_psc_ep0 &
				SLT_PSC_EP0_SWITCH_LIFETIME);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"portStateChange        : %d\n",
				(switchinfo.slt_psc_ep0 &
				SLT_PSC_EP0_PORT_STATE_CHANGE) ? 1 : 0);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"enhancedPort0          : %d\n",
				(switchinfo.slt_psc_ep0 &
				SLT_PSC_EP0_ENHANCED_PORT_0) ? 1 : 0);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"routingModeSupported   : %d\n",
				switchinfo.routing_mode_supported);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"routingModeEnabled     : %d\n",
				switchinfo.routing_mode_enabled);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"lftTop                 : 0x%x\n",
				switchinfo.lft_top);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"unalignedCount         : %d\n",
				switchinfo.unaligned_cnt);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"unalignedPortWindow    : %d\n",
				switchinfo.unaligned_portwin);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"unalignedVirtAddr      : 0x%x\n",
				switchinfo.unaligned_vaddr);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"unalignedPC            : 0x%x\n",
				switchinfo.unaligned_pc);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifDataErrorCount      : %d\n",
				switchinfo.pif_data_error_cnt);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifDataErrorPortWindow : %d\n",
				switchinfo.pif_data_error_portwin);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifDataErrorVirtAddr   : 0x%x\n",
				switchinfo.pif_data_error_vaddr);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifDataErrorPC         : 0x%x\n",
				switchinfo.pif_data_error_pc);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifAddrErrorCount      : %d\n",
				switchinfo.pif_addr_error_cnt);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifAddrErrorPortWindow : %d\n",
				switchinfo.pif_addr_error_portwin);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifAddrErrorVirtAddr   : 0x%x\n",
				switchinfo.pif_addr_error_vaddr);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"pifAddrErrorPC         : 0x%x\n",
				switchinfo.pif_addr_error_pc);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"memScrubSize           : %d\n",
				switchinfo.mem_scrub_size);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"memScrubCount          : %d\n",
				switchinfo.mem_scrub_count);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"vffScrubDisable        : %s\n",
				switchinfo.vff_scrub_disable ? "yes" : "no");
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"vffScrubCount          : %d\n",
				switchinfo.vff_scrub_count);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"rpipeScrubDisable      : %s\n",
				switchinfo.rpipe_scrub_disable ? "yes" : "no");
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"rpipeScrubCount        : %d\n",
				switchinfo.rpipe_scrub_count);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"mrtScrubCount          : %d\n",
				switchinfo.mrt_scrub_count);
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"4opselScrubCount       : %d\n",
				switchinfo.fouropsel_scrub_count);

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations switchinfo_fops = {
	.owner = THIS_MODULE,
	.open = switchinfo_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define RISC_RESET_FILE_NAME "risc_reset"
#define RISC_NMI_FILE_NAME "risc_nmi"

static ssize_t risc_reset_write(struct file *fp, const char __user *buf,
				size_t count, loff_t *fpos)
{
	void __iomem *cport_init_ctrl_reg_addr;
	u64 cport_init_ctrl_reg_val;
	struct fsubdev *sd;

	/* always absorb everything written */
	*fpos += count;

	sd = fp->private_data;

	if (!sd)
		return -EBADF;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	if (!mutex_trylock(&sd->cport_init_ctrl_reg_lock))
		return -EAGAIN;

	routing_sd_destroy(sd);

	cport_init_ctrl_reg_addr = sd->csr_base + CPORT_INIT_CTRL_ADDR;
	cport_init_ctrl_reg_val = readq(cport_init_ctrl_reg_addr);

	cport_init_ctrl_reg_val |= RISC_RESET_BIT;
	writeq(cport_init_ctrl_reg_val, cport_init_ctrl_reg_addr);

	cport_init_ctrl_reg_val ^= RISC_RESET_BIT;
	writeq(cport_init_ctrl_reg_val, cport_init_ctrl_reg_addr);

	mutex_unlock(&sd->cport_init_ctrl_reg_lock);

	sd_info(sd, "RISC RESET requested\n");

	WRITE_ONCE(sd->fw_running, false);
	mbdb_reinit(sd);

	return count;
}

static const struct file_operations risc_reset_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.write = risc_reset_write
};

static ssize_t risc_nmi_read(struct file *fp, char __user *buf, size_t count,
			     loff_t *fpos)
{
	void __iomem *cport_init_ctrl_reg_addr;
	u64 cport_init_ctrl_reg_val;
	struct fsubdev *sd;
	char read_buf[10];
	bool nmi_val;
	size_t siz;

	sd = fp->private_data;

	if (!sd)
		return -EBADF;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	if (!mutex_trylock(&sd->cport_init_ctrl_reg_lock))
		return -EAGAIN;

	cport_init_ctrl_reg_addr = sd->csr_base + CPORT_INIT_CTRL_ADDR;
	cport_init_ctrl_reg_val = readq(cport_init_ctrl_reg_addr);
	nmi_val = (bool)FIELD_GET(RISC_NMI_BIT, cport_init_ctrl_reg_val);

	mutex_unlock(&sd->cport_init_ctrl_reg_lock);

	siz = scnprintf(read_buf, sizeof(read_buf), "%.1u\n", nmi_val);

	return simple_read_from_buffer(buf, count, fpos, read_buf, siz);
}

static ssize_t risc_nmi_write(struct file *fp, const char __user *buf,
			      size_t count, loff_t *fpos)
{
	void __iomem *cport_init_ctrl_reg_addr;
	u64 cport_init_ctrl_reg_val;
	struct fsubdev *sd;
	bool nmi_desired;
	bool nmi_val;
	int err;

	sd = fp->private_data;
	if (!sd)
		return -EBADF;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	err = kstrtobool_from_user(buf, count, &nmi_desired);
	if (err)
		return err;

	if (!mutex_trylock(&sd->cport_init_ctrl_reg_lock))
		return -EAGAIN;

	cport_init_ctrl_reg_addr = sd->csr_base + CPORT_INIT_CTRL_ADDR;
	cport_init_ctrl_reg_val = readq(cport_init_ctrl_reg_addr);
	nmi_val = (bool)FIELD_GET(RISC_NMI_BIT, cport_init_ctrl_reg_val);

	if (nmi_val != nmi_desired) {
		cport_init_ctrl_reg_val ^= RISC_NMI_BIT;
		writeq(cport_init_ctrl_reg_val, cport_init_ctrl_reg_addr);
	}

	mutex_unlock(&sd->cport_init_ctrl_reg_lock);

	*fpos += count;
	return count;
}

static const struct file_operations risc_nmi_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = risc_nmi_read,
	.write = risc_nmi_write
};

#define ASIC_REV_FILE_NAME "asic_rev"
#define ASIC_REV_BUF_SIZE 256

static int asic_rev_open(struct inode *inode, struct file *file)
{
	struct fsubdev *sd = inode->i_private;
	struct asci_rev_info {
		struct debugfs_blob_wrapper blob;
		char buf[ASIC_REV_BUF_SIZE];
	} *info;
	size_t buf_offset;
	size_t buf_size;
	char *buf;

	if (!sd)
		return -EINVAL;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;

	buf_offset = scnprintf(buf, buf_size, "Platform : 0x%04x\n",
			       (int)FIELD_GET(MASK_ARI_PLATFORM, sd->asic_rev_info));
	buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
				"Revision : 0x%02x\n",
				(int)FIELD_GET(MASK_ARI_STEP, sd->asic_rev_info));

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations asic_rev_fops = {
	.owner = THIS_MODULE,
	.open = asic_rev_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

static const char * const mbdb_counter_names[] = {
	"posted requests             : ",
	"non posted requests         : ",
	"timedout requests           : ",
	"handled received requests   : ",
	"unhandled received requests : ",
	"non error responses         : ",
	"error responses             : ",
	"unmatched responses         : ",
	"timedout responses          : ",
	"retry responses             : ",
	"outbound seqnum mismatches  : ",
	"inbound seqnum mismatches   : ",
};

#define MAILBOX_COUNTERS_FILE_NAME "mailbox_counters"
#define COUNTER_NAME_WIDTH 31
#define MAX_U64_WIDTH 21

#define MAILBOX_COUNTER_DISPLAY_BUF_SIZE ((COUNTER_NAME_WIDTH + MAX_U64_WIDTH) * MBDB_COUNTERS_MAX)

static int mbdb_counters_open(struct inode *inode, struct file *file)
{
	struct fsubdev *sd = inode->i_private;
	u64 *mailbox_counter_values;
	struct mbdb_counters_info {
		struct debugfs_blob_wrapper blob;
		char buf[MAILBOX_COUNTER_DISPLAY_BUF_SIZE];
	} *info;
	size_t buf_size;
	size_t buf_offset;
	char *buf;
	enum mbdb_counters i;

	if (!sd)
		return -EINVAL;

	mailbox_counter_values = mbdb_get_mailbox_counters(sd);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	buf_size = ARRAY_SIZE(info->buf);
	buf = info->buf;

	for (buf_offset = 0, i = MBDB_COUNTERS_FIRST; i < MBDB_COUNTERS_MAX; i++)
		buf_offset += scnprintf(buf + buf_offset, buf_size - buf_offset,
					"%s%llu\n", mbdb_counter_names[i],
					mailbox_counter_values[i]);

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations mbdb_counter_fops = {
	.owner = THIS_MODULE,
	.open = mbdb_counters_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

/*
 * LinkMgr Trace data
 *
 */
#define LM_MAGIC 0x4d4c4453
#define LM_VERSION 1
#define DUMP_VERSION GENMASK(31, 0)
#define DUMP_MAGIC GENMASK(63, 32)

struct linkmgr_trace_hdr {
	__be64 magic;
	ktime_t timestamp;
	u8 fw_version_string[24];
};

#define LINKMGR_TRACE_HDR_SIZE sizeof(struct linkmgr_trace_hdr)
#define LINKMGR_TRACE_MAX_BUF_SIZE (40 * 1024)
#define LINKMGR_TRACE_FILE_NAME "linkmgr_trace"

static int linkmgr_trace_open(struct inode *inode, struct file *file)
{
	struct linkmgr_trace_info {
		struct debugfs_blob_wrapper blob;
		struct mbdb_op_linkmgr_trace_dump_rsp rsp;
		char buf[LINKMGR_TRACE_MAX_BUF_SIZE + LINKMGR_TRACE_HDR_SIZE];
	} *info;
	struct fsubdev *sd = inode->i_private;

	struct linkmgr_trace_hdr *hdr;
	size_t buf_offset = LINKMGR_TRACE_HDR_SIZE;
	bool first = true;
	int err;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	if (!(sd->fw_version.environment & FW_VERSION_ENV_BIT))
		return -EIO;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	hdr = (struct linkmgr_trace_hdr *)info->buf;
	hdr->magic = cpu_to_be64(FIELD_PREP(DUMP_VERSION, LM_VERSION) |
				 FIELD_PREP(DUMP_MAGIC, LM_MAGIC));
	strncpy(hdr->fw_version_string, sd->fw_version.fw_version_string,
		sizeof(hdr->fw_version_string));

	hdr->timestamp = ktime_get_real();

	do {
		size_t len;

		err = ops_linkmgr_trace_dump(sd, MAX_TRACE_ENTRIES, first, &info->rsp);
		if (err) {
			kfree(info);
			return err;
		}

		/*
		 * minimum of max size of entries, the count or whatever is
		 * left of the buffer.
		 */
		len = min3(sizeof(info->rsp.entries),
			   info->rsp.cnt * sizeof(u64),
			   sizeof(info->buf) - buf_offset);
		if (len) {
			memcpy(&info->buf[buf_offset], &info->rsp.entries, len);
			buf_offset += len;
		}
		first = false;
	} while (info->rsp.more);

	info->blob.data = info->buf;
	info->blob.size = buf_offset;
	file->private_data = info;

	return 0;
}

static const struct file_operations linkmgr_trace_fops = {
	.owner = THIS_MODULE,
	.open = linkmgr_trace_open,
	.read = blob_read,
	.release = blob_release,
	.llseek = default_llseek,
};

#define LINKMGR_TRACE_MASK_FILE_NAME "linkmgr_trace_mask"

static ssize_t linkmgr_trace_mask_read(struct file *fp, char __user *buf, size_t count,
				       loff_t *fpos)
{
	struct fsubdev *sd;
	char read_buf[20];
	u64 mask = 0;
	size_t siz;
	int err;

	sd = fp->private_data;
	if (!sd)
		return -EBADF;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	err = ops_linkmgr_trace_mask_get(sd, &mask);

	if (err)
		return err;

	siz = scnprintf(read_buf, sizeof(read_buf), "%-18llx\n", mask);

	return simple_read_from_buffer(buf, count, fpos, read_buf, siz);
}

static ssize_t linkmgr_trace_mask_write(struct file *fp, const char __user *buf,
					size_t count, loff_t *fpos)
{
	struct fsubdev *sd;
	u64 mask = 0;
	int err;

	sd = fp->private_data;
	if (!sd)
		return -EBADF;

	if (unlikely(READ_ONCE(sd->fdev->dev_disabled)))
		return -EIO;

	err = kstrtoull_from_user(buf, count, 16, &mask);
	if (err)
		return err;

	err = ops_linkmgr_trace_mask_set(sd, mask);

	if (err)
		return err;

	*fpos += count;
	return count;
}

static const struct file_operations linkmgr_trace_mask_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.llseek = no_llseek,
	.read = linkmgr_trace_mask_read,
	.write = linkmgr_trace_mask_write
};

void create_optional_dev_debugfs_files(struct fsubdev *sd)
{
	if (test_bit(MBOX_OP_CODE_LINK_MGR_TRACE_DUMP, sd->fw_version.supported_opcodes))
		debugfs_create_file(LINKMGR_TRACE_FILE_NAME, 0400, sd->debugfs_dir, sd,
				    &linkmgr_trace_fops);
	if (test_bit(MBOX_OP_CODE_LINK_MGR_TRACE_MASK_GET, sd->fw_version.supported_opcodes) &&
	    test_bit(MBOX_OP_CODE_LINK_MGR_TRACE_MASK_SET, sd->fw_version.supported_opcodes))
		debugfs_create_file(LINKMGR_TRACE_MASK_FILE_NAME, 0600, sd->debugfs_dir, sd,
				    &linkmgr_trace_mask_fops);
}

void create_dev_debugfs_dir(struct fsubdev *sd)
{
	struct dentry *debugfs_dir = debugfs_create_dir(sd->name, sd->fdev->dir_node);
	/*
	 * currently works in all startup modes, could be conditioned here by
	 * setting sd->debugfs_dir = ERR_PTR(-ENODEV) instead
	 */
	sd->debugfs_dir = debugfs_dir;

	debugfs_create_file(FW_VERSION_FILE_NAME, 0400, debugfs_dir, sd, &fw_version_fops);
	debugfs_create_file(SWITCHINFO_FILE_NAME, 0400, debugfs_dir, sd, &switchinfo_fops);
	debugfs_create_file(RISC_RESET_FILE_NAME, 0200, debugfs_dir, sd, &risc_reset_fops);
	debugfs_create_file(RISC_NMI_FILE_NAME, 0644, debugfs_dir, sd, &risc_nmi_fops);
	debugfs_create_file(ASIC_REV_FILE_NAME, 0400, debugfs_dir, sd, &asic_rev_fops);
	debugfs_create_file(MAILBOX_COUNTERS_FILE_NAME, 0400, debugfs_dir, sd, &mbdb_counter_fops);
	debugfs_create_file(CAPABILITIES_FILE_NAME, 0400, debugfs_dir, sd, &capabilities_fops);

	statedump_node_init(sd);
}
