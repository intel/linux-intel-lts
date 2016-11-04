/*
 * ablbc: control ABL bootloaders
 * Copyright (c) 2013-2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/reboot.h>
#include <linux/mc146818rtc.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/kmod.h>

#define MODULE_NAME "ablbc"

/* RTC read and write */
static inline unsigned char cmos_read_ext_bank(u8 addr)
{
	outb(addr, RTC_PORT(4));
	return inb(RTC_PORT(5));
}
#define CMOS_READ_EXT(a) cmos_read_ext_bank(a)

static inline void cmos_write_ext_bank(u8 val, u8 addr)
{
	outb(addr, RTC_PORT(4));
	outb(val, RTC_PORT(5));
}
#define CMOS_WRITE_EXT(v, a) cmos_write_ext_bank(v, a)

/* ABL Conventions */
#define NVRAM_START_ADDRESS	0x10

#define _USERCMD_(cmd, len)		(((cmd) << 5) | ((len) & 0x1f))
#define USERCMD_END			_USERCMD_(0, 0)
#define USERCMD_ACTION			_USERCMD_(7, 1)
#define USERCMD_UPDATE_IFWI(len)	_USERCMD_(2, len)

#define CDATA_TAG_USER_CMD	0x4d
#define NVRAM_VALID_FLAG	0x12

#define CRC32C_POLYNOMIAL 0x82F63B78 /* CRC32C Castagnoli */

static bool capsule_request;

union _cdata_header {
	uint32_t data;
	struct {
		unsigned ncond	: 2;
		unsigned length	: 10;
		unsigned flags	: 4;
		unsigned version: 4;
		unsigned tag	: 12;
	};
};

struct nvram_capsule_cmd {
	char action;
	char device;
	char partition;
	char file_name[1];
} __packed;

struct nvram_reboot_cmd {
	char action;
	char target;
	char end;
	char padding;
} __packed;

struct name2id {
	const char *name;
	int id;
};

struct nvram_msg {
	char magic;
	char size;
	union _cdata_header cdata_header;
	char *cdata_payload;
	size_t cdata_payload_size;
	uint32_t crc;
} __packed;

static const struct name2id NAME2ID[] = {
	{ "main",	0x00 },
	{ "android",	0x00 },
	{ "bootloader",	0x01 },
	{ "fastboot",	0x01 },
	{ "elk",	0x02 },
	{ "recovery",	0x03 },
	{ "crashmode",	0x04 },
	{ "cli",	0x10 },
};

static size_t offset; /* memorize offset between each call */

static size_t write_data_to_nvram(char *data, size_t size)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	for (i = 0; i < size; i++)
		CMOS_WRITE_EXT(*(data + i), NVRAM_START_ADDRESS + offset + i);

	offset += size;
	spin_unlock_irqrestore(&rtc_lock, flags);

	return i;
}

static void write_msg_to_nvram(struct nvram_msg *nvram_msg)
{
	/* Ensure to start from top : only one command expected */
	offset = 0;
	write_data_to_nvram(nvram_msg,
				offsetof(struct nvram_msg, cdata_payload));
	write_data_to_nvram(nvram_msg->cdata_payload,
				nvram_msg->cdata_payload_size);
	write_data_to_nvram(&(nvram_msg->crc), sizeof(nvram_msg->crc));
}

/*  Compute CRC for one byte (shift register-based: one bit at a time). */
static uint32_t crc32c_byte(uint32_t crc, unsigned byte)
{
	int i;
	uint32_t c;

	for (i = 0 ; i < 8 ; i += 1) {
		c = (crc ^ byte) & 1;
		if (c)
			crc = (crc >> 1) ^ CRC32C_POLYNOMIAL;
		else
			crc = (crc >> 1);
		byte >>= 1;
	}

	return crc;
}

/*  Compute CRC for a given buffer. */
static uint32_t crc32c_buf(uint32_t crc, const void *addr, unsigned len)
{
	unsigned i;

	for (i = 0 ; i < len ; i += 1)
		crc = crc32c_byte(crc, *(uint8_t *)(addr + i));

	return crc;
}

static uint32_t crc32c_msg(struct nvram_msg *nvram_msg)
{
	uint32_t crc;

	crc = crc32c_buf(~0, nvram_msg,
				offsetof(struct nvram_msg, cdata_payload));
	crc = crc32c_buf(crc, nvram_msg->cdata_payload,
				nvram_msg->cdata_payload_size);
	return crc;
}

static struct kobject *capsule_kobject;

static ssize_t is_capsule_requested(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf, size_t count)
{
	return sprintf(buf, "%d\n", capsule_request);
}

enum capsule_device_type {
	EMMC = 2,
	SDCARD = 4
};

static ssize_t capsule_store(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf, size_t count)
{
	struct nvram_msg msg;
	struct nvram_capsule_cmd *capsule_cmd;
	char name[32], partition;
	enum capsule_device_type device;
	int ret, crc, padding;
	unsigned char size;
	union _cdata_header cdh;

	device = (buf[0] == 'm' ? EMMC : SDCARD);
	partition = buf[1] - '0';
	ret = sscanf(buf+3, "%s", name);
	pr_info(MODULE_NAME " capsule parameters (%d): DEVICE=%d PARTITION=%d NAME=%s\n",
				ret, device, partition, name);

	cdh.data = 0;
	cdh.tag = CDATA_TAG_USER_CMD;

	/* padding of filename on next dword */
	padding = (4 - (3 + strlen(name))%4)%4;
	size = 2 + sizeof(cdh) + 3 + strlen(name) + padding + 4;
	cdh.length = 1 + (3 + strlen(name) + padding) / 4;

	msg.magic = NVRAM_VALID_FLAG;
	msg.size = size;
	msg.cdata_header.data = cdh.data;

	capsule_cmd = kmalloc(size, GFP_KERNEL);
	if (!capsule_cmd)
		return -ENOMEM;

	capsule_cmd->action = USERCMD_UPDATE_IFWI(strlen(name) + 2);
	capsule_cmd->device = device;
	capsule_cmd->partition = partition;
	strncpy(capsule_cmd->file_name, name, strlen(name));
	msg.cdata_payload = capsule_cmd;
	msg.cdata_payload_size = 3 + strlen(name) + padding;
	msg.crc = crc32c_msg(&msg);
	write_msg_to_nvram(&msg);
	capsule_request = true;

	kfree(capsule_cmd);

	return count;
}

static struct kobj_attribute capsule_name_attribute =
		__ATTR(capsule_name, 0600, NULL, capsule_store);

static struct kobj_attribute capsule_requested_attribute =
		__ATTR(capsule_requested, 0400, is_capsule_requested, NULL);

static int reboot_target_name2id(const char *name)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(NAME2ID); i++)
		if (!strcmp(NAME2ID[i].name, name))
			return NAME2ID[i].id;

	return -EINVAL;
}

static int set_reboot_target(const char *name)
{
	int id;
	struct nvram_msg msg;
	struct nvram_reboot_cmd reboot_cmd;
	union _cdata_header cdh;

	if (name == NULL) {
		pr_err("Error in %s: NULL target\n", __func__);
		return -EINVAL;
	}

	id  = reboot_target_name2id(name);
	if (id < 0) {
		pr_err("Error in %s: '%s' is not a valid target\n",
		       __func__, name);
		return -EINVAL;
	}

	cdh.data = 0;
	cdh.length = 2; /* 2*32 bits, from header to padding */
	cdh.tag = CDATA_TAG_USER_CMD;

	memset(&reboot_cmd, 0, sizeof(reboot_cmd));
	memset(&msg, 0, sizeof(msg));
	msg.magic = NVRAM_VALID_FLAG;
	msg.cdata_header.data = cdh.data;
	reboot_cmd.action = USERCMD_ACTION;

	reboot_cmd.target = id;
	msg.cdata_payload = &reboot_cmd;
	msg.cdata_payload_size = sizeof(reboot_cmd);
	msg.size = offsetof(struct nvram_msg, cdata_payload) +
			sizeof(reboot_cmd) + sizeof(msg.crc);
	msg.crc = crc32c_msg(&msg);

	write_msg_to_nvram(&msg);

	return 0;
}

static const unsigned int DEFAULT_TARGET_INDEX;

static const char * const cold_reset[] = {
	"/sbin/cansend",
	"slcan0",
	"0000FFFF#05025555555555",
	NULL};
static const char * const cold_reset_capsule[] = {
	"/sbin/cansend",
	"slcan0",
	"0000FFFF#05035555555555",
	NULL};
static const char * const suppress_heartbeat[] = {
	"/sbin/cansend",
	"slcan0",
	"0000FFFF#01035555555555",
	NULL};
static const char * const reboot_request[] = {
	"/sbin/cansend",
	"slcan0",
	"0000FFFF#03015555555555",
	NULL};

static int execute_slcan_command(const char *cmd[])
{
	struct subprocess_info *sub_info;
	int ret = -1;

	sub_info = call_usermodehelper_setup(cmd[0],
		cmd, NULL, GFP_KERNEL,
		NULL, NULL, NULL);

	if (sub_info) {
		ret = call_usermodehelper_exec(sub_info,
			UMH_WAIT_PROC);
		pr_info("Exec cmd=%s ret=%d\n", cmd[0], ret);
	}

	if (ret)
		pr_err("Failure on cmd=%s ret=%d\n", cmd[0], ret);

	return ret;
}

static int ablbc_reboot_notifier_call(struct notifier_block *notifier,
				      unsigned long what, void *data)
{
	const char *target = (const char *)data;
	int ret;

	if (what != SYS_RESTART)
		return NOTIFY_DONE;

	ret = execute_slcan_command(suppress_heartbeat);
	if (ret)
		goto done;

	ret = execute_slcan_command(reboot_request);
	if (ret)
		goto done;
	if (target[0] != '\0') {
		ret = set_reboot_target(target);
		if (ret)
			pr_err("%s: Failed to set reboot target, ret=%d\n",
				__func__, ret);
		else {
			ret = execute_slcan_command(cold_reset);
			if (ret)
				goto done;
		}
	}
	if (capsule_request)
		ret = execute_slcan_command(cold_reset_capsule);

done:
	return NOTIFY_DONE;
}

static struct notifier_block ablbc_reboot_notifier = {
	.notifier_call = ablbc_reboot_notifier_call,
};

static int __init ablbc_init(void)
{
	int ret;

	ret = register_reboot_notifier(&ablbc_reboot_notifier);
	if (ret) {
		pr_err(MODULE_NAME ": unable to register reboot notifier\n");
		return ret;
	}

	capsule_kobject  = kobject_create_and_add("capsule", kernel_kobj);
	if (!capsule_kobject)
		return -ENOMEM;

	ret = sysfs_create_file(capsule_kobject,
				&capsule_name_attribute.attr);
	if (ret) {
		pr_err("failed to create the foo file in /sys/kernel/capsule/capsule_name\n");
		goto err;
	}

	ret = sysfs_create_file(capsule_kobject,
				&capsule_requested_attribute.attr);
	if (ret) {
		pr_err("failed to create the foo file in /sys/kernel/capsule/capsule_requested\n");
		goto err;
	}

	return 0;

err:
	kobject_put(capsule_kobject);
	return ret;
}

module_init(ablbc_init);

static void __exit ablbc_exit(void)
{
	unregister_reboot_notifier(&ablbc_reboot_notifier);
	kobject_put(capsule_kobject);
}
module_exit(ablbc_exit);

MODULE_AUTHOR("Guillaume Betous <guillaume.betous@intel.com>");
MODULE_DESCRIPTION("Automotive Bootloader boot control driver");
MODULE_LICENSE("GPL v2");
