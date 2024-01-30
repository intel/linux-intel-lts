// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2023 Intel Corporation.
 *
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/crc32c.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/overflow.h>
#include <linux/slab.h>

#include "dev_diag.h"
#include "error.h"
#include "fw.h"
#include "iaf_drv.h"
#include "ops.h"
#include "mbdb.h"
#include "port.h"
#include "routing_engine.h"
#include "sysfs.h"

/*
 * Pad bytes needed to reach _type boundary. If sizeof(_type) is a power of
 * two, this should optimize to -(_size) & (sizeof(_type) - 1)
 */
#define PAD_TO(_type, _size)    ((sizeof(_type) - (_size) % sizeof(_type)) % sizeof(_type))

/*
 * how many times to check FW_VERSION for completion of firmware initialization
 * before declaring failure (there is a 20 ms delay before each check -- 50
 * delays is a minimum of 1 second)
 */
#define MAX_20MS_DELAYS (50 * 6)

/*
 * how many times to retry PSC bin read waiting for SPI to become available
 * before declaring failure (there is a 200 ms delay between each check -- 5
 * delays is a minimum of 1 second)
 */
#define MAX_200MS_RETRIES (5 * 60 * 10)
#define ONE_MINUTE_COUNT 300 /* (300 * 200ms) = 1 min */

#define MODULE_TYPE_CSS_GENERIC	(6)
#define HEADER_VER_RSA		(0x00010000)
#define MODULE_VENDOR_NAME	GENMASK(15, 0)
#define MODULE_VENDOR_INTEL	(0x8086)
#define MODULE_ID_DEBUG_SIGNED	BIT(31)
#define KEY_SIZE_SHA384		(96)
#define MODULUS_SIZE_SHA384	(96)
#define EXPONENT_SIZE_SHA384	(1)
#define CSS_DATE_YEAR		GENMASK(31, 16)
#define CSS_DATE_MONTH		GENMASK(15, 8)
#define CSS_DATE_DAY		GENMASK(7, 0)
#define PSC_TIME_HOUR		GENMASK(31, 16)
#define PSC_TIME_MINUTE		GENMASK(15, 8)
#define PSC_TIME_SECOND		GENMASK(7, 0)

struct fw_css_header {
	u32 module_type;	/* always 0x00000006 */
	u32 header_len;		/* always 0x000000e1 */
	u32 header_ver;		/* always 0x00010000 */
	u32 module_id;		/* bit 31 = debug signed */
	u32 module_vendor;	/* always 0x00008086 */
	u32 date;		/* format 0xYYYYMMDD */
	u32 size;		/* size in dwords */
	u32 key_size;		/* typical 0x00000060, must be valid */
	u32 modulus_size;	/* typical 0x00000060, must be valid */
	u32 exponent_size;	/* typical 0x00000001, don't care */
	u32 reserved[22];
	u8 modulus[384];
	u8 exponent[4];
	u8 signature[384];
};

#define FW_MAX_SEGMENTS	(3)

#define BASE_LOAD_ADDRESS	(0xaffe0000)

struct fw_segment {
	u32 start;
	u32 end;
};

struct fw_module_header {
	u32 init_addr;
	struct fw_segment segment[FW_MAX_SEGMENTS];
	u32 fw_version;
	u32 fw_build;
};

#define PSCBIN_MAGIC_NUMBER (0x42435350)
#define MAX_INIS ((MAX_SOCKET_IDS) * 2)

/* Format of struct psc_data being used: */
enum {
	PSCBIN_VERSION_V0 = 0,	/* NULL PSC */
	PSCBIN_VERSION_V1,	/* unsupported */
	PSCBIN_VERSION_V2,
	PSCBIN_VERSION_V3,
	PSCBIN_VERSION_CURRENT = PSCBIN_VERSION_V3,
	PSCBIN_VERSION_MIN = PSCBIN_VERSION_V2,
	PSCBIN_VERSION_MAX = PSCBIN_VERSION_V3,
};

/* Form factor: */
enum {
	FORM_FACTOR_PCI = 0,
	FORM_FACTOR_OAM,
	FORM_FACTOR_A21,
	FORM_FACTOR_END
};

/* PSC must be at least as large as header and no greater than 1 MB */
#define MIN_PSC_SIZE sizeof(struct psc_data)
#define MAX_PSC_SIZE (1024 * 1024)
#define SPI_HEADER_SIZE (8 * 1024)

#define I915_SPI_DRIVER_NAME_AUX "i915_spi"
#define I915_SPI_DRIVER_NAME_MFD "i915-spi"
#define PSC_REGION_QUALIFIER	".PSC"

#define PVC_FW_IMAGE  "i915/pvc_iaf_ver1.bin"
#define PVC_FW_IMAGE_DEBUG_SIGNED  "i915/pvc_iaf_ver1d.bin"
#define PVC_FW_IMAGE_EXPORT_LIMITED  "i915/pvc_iaf_ver1e.bin"
/*
 * FW image
 */
#if IS_ENABLED(CONFIG_IAF_DEBUG_ALTERNATE_FW_LOAD)
static char *fw_image = "";
module_param(fw_image, charp, 0600);
MODULE_PARM_DESC(fw_image, "Retrieve FW image from specified FW file");
#endif

/**
 * struct psc_identifier - platform specific configuration identifier
 * @magic: must be PSCBIN_MAGIC_NUMBER
 * @psc_format_version: interface version being used
 *
 * This identifier is at the start of all psc data structures. It identifies
 * it as platform specific configuration data and identifies which version of
 * the structure is being used.
 */
struct psc_identifier {
	u32 magic;
	u32 psc_format_version;
};

/**
 * struct psc_item - Identify index/size of variably-sized data item
 * @idx: Index of item in data[] (in bytes)
 * @size: Size of item in data[] (in bytes)
 *
 * Used with &struct psc_data to identify variably-sized data elements in
 * &struct psc_data.data[]. If size is 0, the item is not present. It is
 * possible for two items to refer to the same data.
 */
struct psc_item {
	u32 idx;
	u32 size;
};

/**
 * struct psc_data - platform specific configuration data
 * @identifier: identify the version of this structure being used
 * @identifier.magic: must be PSCBIN_MAGIC_NUMBER
 * @identifier.psc_format_version: must be 0 or in [PSCBIN_VERSION_MIN:PSCBIN_VERSION_MAX]
 * @form_factor: type of the product (PCI, OAM, A21)
 * @cfg_version: should only increment to prevent unintentional downgrades
 * @date: UTC generation date in BCD (YYYYMMDD)
 * @time: UTC generation time in BCD (HHMMSS)
 * @flags: for routing and other extension options, should be 0 currently
 * @reserved1: reserved for future use, should be 0 currently
 * @reserved2: reserved for future use, should be 0 currently
 * @data_size: size of @data[] array
 * @brand_name: OEM-provided
 * @product_name: OEM-provided, shouldn't change when a new PSCBIN is burned
 * @comment: OEM-provided comment
 * @ini_name: names of ini_bin files used
 * @ini_bin: binary ini data to provide to FW
 * @ext_data: extra data, reserved for future use
 * @cust_data: arbitrary customer data in PSC file, not used or displayed
 * @presence_data: refers to presence configuration if present
 * @reserved3: forces data[] to be 64-bit aligned, should be 0 currently
 * @crc32c_hdr: CRC covering all other header fields
 * @data: array containing all variably-sized data items
 *
 * Identifies platform-specific configuration data corresponding to a device.
 * Retrieved from SPI flash (or optionally as a firmware file) during boot.
 * Normally the same PSC data would be present on all devices in the system,
 * both socket number and subdevice index are used to locate INI_BIN data,
 * which can differ between subdevices.
 *
 * Optional/variably-sized items are contained in @data[] and indexed using
 * fields of type &struct psc_item. &(struct psc_item).size is the size of the
 * actual data item, although as of PSCBIN_VERSION_V3 each item except presence
 * data is 0-padded to a 32-bit boundary in @data[] and followed immediately by
 * a CRC32C checksum (presence data is fully covered by its own checksums).
 *
 * @presence_data is introduced as of PSCBIN_VERSION_V3. If @presence_data is
 * configured, it should refer to an instance of &struct psc_presence_config
 * contained within @data[].
 *
 * This structure is defined identically in PSCBIN_VERSION_V2, except that
 * @presence_data becomes crc_data, which refers to a CRC32C at the end of
 * @data[] which covers all other items in @data[] instead of using per-item
 * CRC32C checksums.
 *
 * The first two fields are the same in all versions of this structure to
 * support versioning of this interface: all prior versions are either
 * deprecated or no longer supported.
 */
struct psc_data {
	struct psc_identifier identifier;
	u32 form_factor;
	u32 cfg_version;
	u32 date;
	u32 time;
	u32 flags;
	u32 reserved1;
	u32 reserved2;
	u32 data_size;
	struct psc_item brand_name;
	struct psc_item product_name;
	struct psc_item comment;
	struct psc_item ini_name[MAX_INIS];
	struct psc_item ini_bin[MAX_INIS];
	struct psc_item ext_data;
	struct psc_item cust_data;
	struct psc_item presence_data;
	u32 reserved3;
	u32 crc32c_hdr;
	u8 data[];
};

/**
 * struct psc_presence_config - presence configuration data
 * @rules: variably-sized per-device presence detection rules
 * @data_size: size of @data[] array
 * @crc32c_hdr: CRC covering all other header fields
 * @data: array containing variably-sized data items
 *
 * Identifies device presence configuration data. This is configured based
 * on socket_id. Should be referred to by &(struct psc_data).presence_data
 * and reside in &(struct psc_data).data[]
 *
 * Optional/variably-sized items are contained in @data[] and indexed using
 * fields of type &struct psc_item. Each populated @rules entry refers to
 * an array of &struct psc_presence_rule items contained in @data[]
 */
struct psc_presence_config {
	struct psc_item rules[MAX_SOCKET_IDS];
	u32 data_size;
	u32 crc32c_hdr;
	u8 data[];
};

/*
 * PSC image
 */
static char *psc_file_override = "";
module_param(psc_file_override, charp, 0600);
MODULE_PARM_DESC(psc_file_override,
		 "Get PSC data from file (via FW API) instead of flash");

/*
 * firmware roption override
 */
/* must match the module param description */
#define ROPTION_DEFAULT ( \
	ROPTION_FORCE_FEC_HEAVY_ON | \
	ROPTION_TRACE_LNI_DEBUG    | \
	ROPTION_FORCE_FEC_LIGHT_ON)

static unsigned int firmware_roption = ROPTION_DEFAULT;
module_param(firmware_roption, uint, 0600);
MODULE_PARM_DESC(firmware_roption,
		 "Set the firmware roption debug register (default: 0x40000201)");

/*
 * Workqueue and timer support functions
 */
static void wrk_load_and_init_subdev(struct work_struct *work);
static void queue_load_and_init_all_subdevs(struct fdev *dev);
static void wrk_load_psc_data(struct work_struct *work);

static DECLARE_BITMAP(load_fw_opcodes, MAILBOX_OPCODE_COUNT);
static DECLARE_BITMAP(required_runtime_opcodes, MAILBOX_OPCODE_COUNT);

static void init_opcodes(void)
{
	__set_bit(MBOX_OP_CODE_FW_VERSION, load_fw_opcodes);
	__set_bit(MBOX_OP_CODE_CSR_RAW_WR, load_fw_opcodes);
	__set_bit(MBOX_OP_CODE_FW_START, load_fw_opcodes);

	__set_bit(MBOX_OP_CODE_FW_VERSION, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_CSR_RAW_RD, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_CSR_RAW_WR, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_WALLOC, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_INI_TABLE_LOAD, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_INI_LOADED_SET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LINK_STATE_SET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_MAJOR_PHYSICAL_STATE_SET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_CSR_RD, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_CSR_WR, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_RESET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_STATES_GET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_BEACON_SET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_ENABLE_SET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_NOTIFICATION,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_STATE_CHANGE_TRAP_ACKNOWLEDGE,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_SWITCHINFO_GET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_SWITCHINFO_SET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_PORTINFO_GET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_RPIPE_GET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_RPIPE_SET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_ENABLE_SET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_NOTIFICATION,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LWD_TRAP_ACKNOWLEDGE,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_ENABLE_SET,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_NOTIFICATION,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_LINK_MGR_PORT_LQI_TRAP_ACKNOWLEDGE,
		  required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_PORT_STATUS_GET, required_runtime_opcodes);
	__set_bit(MBOX_OP_CODE_STATE_DUMP, required_runtime_opcodes);
}

/**
 * fw_init_module - Early device-independent fw module initialization
 */
void fw_init_module(void)
{
	init_opcodes();
}

/**
 * fw_init_dev - Early device-dependent fw module initialization
 * @dev: device to initialize
 */
void fw_init_dev(struct fdev *dev)
{
	u8 i;

	init_completion(&dev->psc.done);
	init_completion(&dev->psc.abort);

	INIT_WORK(&dev->psc.work, wrk_load_psc_data);

	atomic_set(&dev->fwinit_refcnt, dev->pd->sd_cnt);

	for (i = 0; i < dev->pd->sd_cnt; ++i)
		INIT_WORK(&dev->sd[i].fw_work, wrk_load_and_init_subdev);
}

static void report_fw_info(struct fsubdev *sd,
			   struct mbdb_op_fw_version_rsp *fw_version,
			   const char *when)
{
	sd_dbg(sd, "ROM/Firmware Version Info: (%s)\n", when);
	sd_dbg(sd, "    MBox Version:  %d\n", fw_version->mbox_version);
	sd_dbg(sd, "    Environment:   %s%s\n",
	       (fw_version->environment & FW_VERSION_ENV_BIT) ?
	       "run-time" :
	       "bootloader",
	       (fw_version->environment & FW_VERSION_INIT_BIT) ?
	       ", ready" :
	       "");
	sd_dbg(sd, "    FW Version:    %s\n", fw_version->fw_version_string);
}

/* convert BCD-coded unsigned number to decimal */
static u32 decode_bcd(u32 bcd_value)
{
	u32 value = 0;
	u32 place;
	u32 digit;

	for (place = 1; bcd_value; bcd_value >>= 4, place *= 10) {
		digit = bcd_value & 0xf;
		if (digit > 9)
			return -EINVAL;
		value += digit * place;
	}

	return value;
}

/* Leap year every 4 years unless year is divisible by 100 and not 400 */
static bool leap_year(u32 year)
{
	if (year % 4 != 0)
		return false;
	else if ((year % 100 == 0) || (year % 400 != 0))
		return false;
	else
		return true;
}

/* Verify that date is of form 0xYYYYMMDD (or 0 for NULL date code) */
static bool valid_bcd_date(u32 date)
{
	u32 month;
	u32 year;
	u32 day;

	if (!date)
		return true;

	year = decode_bcd(FIELD_GET(CSS_DATE_YEAR, date));
	month = decode_bcd(FIELD_GET(CSS_DATE_MONTH, date));
	day = decode_bcd(FIELD_GET(CSS_DATE_DAY, date));

	if (day < 1 || year < 2019)
		return false;

	switch (month) {
	case 1:
	case 3:
	case 5:
	case 7:
	case 8:
	case 10:
	case 12:
		if (day > 31)
			return false;
		break;

	case 4:
	case 6:
	case 9:
	case 11:
		if (day > 30)
			return false;
		break;

	case 2:
		if (day > 29 || (day == 29 && !leap_year(year)))
			return false;
		break;

	default:
		return false;
	}

	return true;
}

/* Leap seconds occur at the end of the UTC day */
static bool leap_second(u32 hour, u32 minute, u32 second)
{
	return hour == 23 && minute == 59 && second == 60;
}

/* Verify that date is of form 0xHHMMSS */
static bool valid_bcd_time(u32 time)
{
	u32 second;
	u32 minute;
	u32 hour;

	hour = decode_bcd(FIELD_GET(PSC_TIME_HOUR, time));
	minute = decode_bcd(FIELD_GET(PSC_TIME_MINUTE, time));
	second = decode_bcd(FIELD_GET(PSC_TIME_SECOND, time));

	return hour < 24 && minute < 60 && (second < 60 || leap_second(hour, minute, second));
}

/*
 * When loading the PSC as firmware via the psc_file_override parameter or with older formats that
 * did not support per-item CRCs, the entire PSC blob is loaded in memory. As of V3, when loading
 * from SPI, only required items are loaded to reduce expensive reads.
 */
static inline bool full_pscbin_in_memory(struct fdev *dev, u32 psc_format_version)
{
	return dev->psc.as_fw || psc_format_version < PSCBIN_VERSION_V3;
}

static inline bool psc_has_per_item_crc(u32 psc_format_version)
{
	return psc_format_version > PSCBIN_VERSION_V2;
}

static inline bool psc_has_presence_rules(u32 psc_format_version)
{
	return psc_format_version > PSCBIN_VERSION_V2;
}

static u32 crc32c_with_padding(const u8 *buf, u32 size)
{
	static const u8 zero_bytes[sizeof(u32)] = {};

	return crc32c(crc32c(0, buf, size), zero_bytes, PAD_TO(u32, size));
}

static bool psc_item_crc_ok(const struct psc_data *psc, const struct psc_item *item)
{
	u32 *crc;

	if (!psc_has_per_item_crc(psc->identifier.psc_format_version))
		return true;

	crc = (u32 *)&psc->data[item->idx + item->size + PAD_TO(u32, item->size)];

	return *crc == crc32c_with_padding(&psc->data[item->idx], item->size);
}

static int verify_fw(struct fdev *dev)
{
	struct fw_module_header *mod_hdr;
	struct fw_css_header *css_hdr;
	u8 *segment_data_end;
	u8 *segment_data;
	const u8 *data;
	size_t size;
	int i;

	dev_dbg(fdev_dev(dev), "Checking firmware header\n");

	data = dev->fw->data;
	size = dev->fw->size;

	css_hdr = (struct fw_css_header *)data;
	mod_hdr = (struct fw_module_header *)(css_hdr + 1);

	segment_data = (u8 *)(mod_hdr + 1);
	segment_data_end = segment_data;

	if (data + size < segment_data) {
		dev_err(fdev_dev(dev), "Incomplete firmware header\n");
		return -EINVAL;
	}

	if (css_hdr->module_type != MODULE_TYPE_CSS_GENERIC ||
	    css_hdr->header_ver != HEADER_VER_RSA ||
	    FIELD_GET(MODULE_VENDOR_NAME, css_hdr->module_vendor) !=
	     MODULE_VENDOR_INTEL) {
		dev_err(fdev_dev(dev),
			"Illegal firmware type/version/vendor: %d/0x%x/0x%x\n",
			css_hdr->module_type, css_hdr->header_ver,
			css_hdr->module_vendor);
		return -EINVAL;
	}

	for (i = 0; i < FW_MAX_SEGMENTS; ++i)
		if (mod_hdr->segment[i].end > mod_hdr->segment[i].start)
			segment_data_end += (mod_hdr->segment[i].end -
					     mod_hdr->segment[i].start);

	if (data + size < segment_data_end) {
		dev_err(fdev_dev(dev), "Incomplete fw data\n");
		return -EINVAL;
	}

	if (css_hdr->header_len * sizeof(u32) != sizeof(*css_hdr) ||
	    data + css_hdr->size * sizeof(u32) != segment_data_end ||
	    css_hdr->key_size != KEY_SIZE_SHA384 ||
	    css_hdr->modulus_size != MODULUS_SIZE_SHA384 ||
	    css_hdr->exponent_size != EXPONENT_SIZE_SHA384) {
		dev_err(fdev_dev(dev), "Mismatched size information in fw header\n");
		return -EINVAL;
	}

	dev_dbg(fdev_dev(dev), "Firmware available, dated: %08x\n",
		css_hdr->date);

	if (css_hdr->date == 0 || !valid_bcd_date(css_hdr->date)) {
		dev_err(fdev_dev(dev), "Invalid date format in firmware header\n");
		return -EINVAL;
	}

	if (css_hdr->module_id & MODULE_ID_DEBUG_SIGNED)
		dev_dbg(fdev_dev(dev), "Firmware is debug signed\n");

	return 0;
}

static int copy_fw_to_device(struct fsubdev *sd, const u8 *data, size_t size)
{
	struct fw_module_header *mod_hdr;
	struct fw_css_header *css_hdr;
	u8 *segment_data;
	u32 ld_addr;
	u32 len;
	int err;
	int i;

	sd_dbg(sd, "Downloading firmware\n");

	/* verify_fw() ensures that this data is all in the FW image */

	css_hdr = (struct fw_css_header *)data;
	mod_hdr = (struct fw_module_header *)(css_hdr + 1);
	segment_data = (u8 *)(mod_hdr + 1);

	ld_addr = BASE_LOAD_ADDRESS;

	err = ops_mem_posted_wr(sd, ld_addr, (u8 *)css_hdr, sizeof(*css_hdr));
	if (err) {
		sd_err(sd, "Could not write CSS hdr\n");
		goto end;
	}

	ld_addr += sizeof(*css_hdr);

	err = ops_mem_posted_wr(sd, ld_addr, (u8 *)mod_hdr, sizeof(*mod_hdr));
	if (err) {
		sd_err(sd, "Could not write module hdr\n");
		goto end;
	}

	for (i = 0; i < FW_MAX_SEGMENTS; ++i) {
		if (mod_hdr->segment[i].end <= mod_hdr->segment[i].start)
			continue;

		ld_addr = mod_hdr->segment[i].start;
		len = mod_hdr->segment[i].end - ld_addr;
		err = ops_mem_posted_wr(sd, ld_addr, segment_data, len);
		if (err) {
			sd_err(sd, "Could not write segment %d\n", i);
			goto end;
		}
		segment_data += len;
	}

end:
	return err;
}

bool is_opcode_valid(struct fsubdev *sd, const u8 op_code)
{
	struct mbdb_op_fw_version_rsp *fw_version = &sd->fw_version;
	u8 fw_environment = fw_version->environment & FW_VERSION_ENV_BIT;

	/* Validate op_code is one that is available in the current environment */
	if (test_bit(op_code, fw_version->supported_opcodes))
		return true;

	sd_warn(sd, "opcode %d not supported in %s mode by firmware version %s\n", op_code,
		fw_environment ? "runtime" : "firmware load", fw_version->fw_version_string);

	return false;
}

static bool has_all_opcodes(unsigned long *supported, unsigned long *needed)
{
	DECLARE_BITMAP(common, MAILBOX_OPCODE_COUNT);

	bitmap_and(common, supported, needed, MAILBOX_OPCODE_COUNT);
	return bitmap_equal(common, needed, MAILBOX_OPCODE_COUNT);
}

static int load_fw(struct fsubdev *sd)
{
	struct mbdb_op_fw_version_rsp *fw_version = &sd->fw_version;
	int err;

	report_fw_info(sd, fw_version, "at boot");

	if (fw_version->environment & FW_VERSION_ENV_BIT) {
		/*
		 * Found runtime FW, expecting boot loader
		 */

		sd_info(sd, "runtime firmware detected at boot\n");

		sd_info(sd, "resetting to boot loader\n");

		err = ops_reset(sd, false);
		if (err)
			goto load_failed;

		/* more than enough time for FW to do 1 write */
		msleep(20);

		mbdb_reinit(sd);

		err = ops_fw_version(sd, fw_version);

		if (err) {
			sd_err(sd, "error checking FW version\n");
			goto load_failed;
		}

		report_fw_info(sd, fw_version, "after RESET");
	}

	if (fw_version->environment & FW_VERSION_ENV_BIT) {
		sd_err(sd, "unable to reset to boot loader\n");
		err = -EPERM;
		goto load_failed;
	}

	if (!has_all_opcodes(fw_version->supported_opcodes, load_fw_opcodes)) {
		sd_err(sd, "FW LOAD opcodes not supported\n");
		err = -EPERM;
		goto load_failed;
	}

	/*
	 * This shouldn't be possible
	 */
	if (WARN_ON(!sd->fdev->fw)) {
		err = -ENOENT;
		goto load_failed;
	}

	err = copy_fw_to_device(sd, sd->fdev->fw->data, sd->fdev->fw->size);
	if (err) {
		sd_err(sd, "error copying firmware to device\n");
		goto load_failed;
	}

	return 0;

load_failed:

	sd_err(sd, "could not load firmware\n");
	return err;
}

static int start_fw(struct fsubdev *sd)
{
	struct mbdb_op_fw_version_rsp *fw_version = &sd->fw_version;
	int err;

	mbdb_tile_number_set(sd);

	sd_dbg(sd, "Starting firmware\n");

	err = ops_fw_start(sd);
	if (err)
		goto start_failed;

	err = ops_fw_version(sd, fw_version);
	if (err)
		goto start_failed;

	report_fw_info(sd, fw_version, "after firmware start");

	if (!has_all_opcodes(fw_version->supported_opcodes,
			     required_runtime_opcodes)) {
		sd_err(sd, "Required FW opcodes not supported\n");
		err = -EPERM;
		goto start_failed;
	}

	return 0;

start_failed:
	sd_err(sd, "could not start firmware\n");
	return err;
}

static int set_roption(struct fsubdev *sd)
{
	u32 reg = firmware_roption;
	int err;

	sd_dbg(sd, "setting roption: 0x%08x\n", reg);

	err = ops_csr_raw_write(sd, CSR_ROPTION, &reg, sizeof(reg), false);
	if (err)
		sd_err(sd, "failed to set firmware roption register: %d\n", err);

	return err;
}

#define OP_ERR_NOT_SET	(999)

static int copy_ini_to_device(struct fsubdev *sd, const u8 *data, size_t size)
{
	struct mbdb_op_ini_table_load_req ini = {};
	u32 *end = (u32 *)(data + size);
	u32 *dword = (u32 *)data;
	u32 op_err = OP_ERR_NOT_SET;
	int err = -EINVAL;
	u32 *crc;
	u32 cnt;

	sd_dbg(sd, "Loading INI file, size=%ld\n", size);

	while (dword + 2 < end) {
		ini.header1 = *dword++;
		ini.header2 = *dword++;

		sd_dbg(sd,
		       "HDR1/HDR2=0x%08x/0x%08x cnt=%d, left=%ld, index=%d, type=%d\n",
		       ini.header1, ini.header2, (ini.header1 >> 16) & 0xfff,
		       end - dword, ini.header1 & 0x3f, ini.header1 >> 28);

		if (ini.header1 != ~ini.header2) {
			sd_err(sd, "Invalid INIBIN header\n");
			goto end;
		}
		cnt = (ini.header1 >> 16) & 0xfff;
		crc = dword + cnt;
		if (crc >= end) {
			sd_err(sd, "Incomplete INIBIN block\n");
			goto end;
		}

		sd_dbg(sd, "WALLOC dwords=0x%x\n", cnt);
		err = ops_walloc(sd, cnt, &ini.address);
		if (err) {
			sd_err(sd, "FW could not allocate INIBIN space\n");
			goto end;
		}

		sd_dbg(sd, "CSR_RAW_WR address=ADDR len=%lu\n",
		       cnt * sizeof(u32));

		err = ops_mem_posted_wr(sd, ini.address, (u8 *)dword,
					cnt * sizeof(u32));
		if (err) {
			sd_err(sd, "Error writing INIBIN block\n");
			goto end;
		}

		sd_dbg(sd, "INI_TABLE_LOAD header=0x%08x crc=0x%08x\n",
		       ini.header1, *crc);

		ini.crc = *crc;

		err = ops_ini_table_load(sd, &ini, &op_err);
		if (err) {
			sd_err(sd, "Error loading INIBIN block, err %d, op_err %u\n", err, op_err);
			goto end;
		}

		if (op_err) {
			/* FW should return 0 if successful, 1 if no data or CRC error */
			sd_err(sd, "FW rejected INIBIN block, op_err %u\n", op_err);
			err = -ENODATA;
			goto end;
		}

		dword = (crc + 1);
	}

	err = set_roption(sd);
	if (err)
		goto end;

	sd_dbg(sd, "INI_LOADED_SET\n");
	err = ops_ini_loaded_set(sd, false);
	if (err)
		goto end;

	sd_dbg(sd, "ini load complete\n");

end:
	return err;
}

static int read_spi_data(struct fdev *dev, loff_t offset, size_t size, void *buf, const char *desc)
{
	size_t bytes_read = 0;
	int err = -ENODEV;

	if (dev->psc.mtd)
		err = mtd_read(dev->psc.mtd, offset, size, &bytes_read, (u_char *)buf);

	if (!err && bytes_read != size)
		err = -ENODATA;

	if (err)
		dev_err(fdev_dev(dev), "Error %d reading PSC %s, read %lu/%lu bytes\n", err, desc,
			bytes_read, size);

	return err;
}

/**
 * check_and_extract_ini_data() - Check and if necessary extract INIBIN data for subdevice
 *
 * @dev: fabric device object
 * @sd_idx: subdevice index
 * @item: reference to PSC index data for this subdevice
 *
 * If the entire PSCBIN is in memory, simply verify any section-specific CRC values.
 *
 * If not, make sure the INIBIN data is in @dev->psc.ini_buf[sd_idx] Data is loaded from NVMEM and
 * CRC-checked. If multiple subdevices refer to the same INIBIN data, buffer pointers are reused.
 * request_pscdata_from_spi must set dev->psc.mtd if reading from SPI.
 *
 * Return: 0 on success, negative error code on error
 */
static int check_and_extract_ini_data(struct fdev *dev, u8 sd_idx, const struct psc_item *item)
{
	const struct psc_data *psc = (const struct psc_data *)dev->psc.data;
	loff_t offset;
	u32 check_crc;
	u8 buf_idx;
	u8 *buf;
	int err;

	if (sd_idx >= IAF_MAX_SUB_DEVS) {
		dev_err(fdev_dev(dev), "Illegal subdevice index %u for INI_BIN data\n", sd_idx);
		return -EINVAL;
	}

	if (full_pscbin_in_memory(dev, psc->identifier.psc_format_version)) {
		if (psc_item_crc_ok(psc, item))
			return 0;

		dev_err(fdev_dev(dev), "INI_BIN CRC mismatch\n");
		return -EIO;
	}

	/*
	 * get INI_BIN data for this subdevice
	 */

	/* if INI_BIN was already extracted for another subdevice, use it */
	for (buf_idx = 0; buf_idx < sd_idx; ++buf_idx)
		if (dev->psc.ini_buf[buf_idx].idx == item->idx &&
		    dev->psc.ini_buf[buf_idx].size == item->size) {
			dev->psc.ini_buf[sd_idx].data = dev->psc.ini_buf[buf_idx].data;
			dev->psc.ini_buf[sd_idx].idx = item->idx;
			dev->psc.ini_buf[sd_idx].size = item->size;
			dev->psc.ini_buf[sd_idx].do_not_free = true;
			return 0;
		}

	/* otherwise extract the INI_BIN data for this subdevice */
	buf = kmalloc(item->size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	offset = SPI_HEADER_SIZE + sizeof(*psc) + item->idx;
	err = read_spi_data(dev, offset, item->size, buf, "INI_BIN data");
	if (err)
		goto fail;

	offset += item->size + PAD_TO(u32, item->size);
	err = read_spi_data(dev, offset, sizeof(u32), &check_crc, "INI_BIN CRC");
	if (err)
		goto fail;

	if (check_crc != crc32c_with_padding(buf, item->size)) {
		dev_err(fdev_dev(dev), "INI_BIN CRC mismatch\n");
		goto fail;
	}

	dev->psc.ini_buf[buf_idx].data = buf;
	dev->psc.ini_buf[buf_idx].idx = item->idx;
	dev->psc.ini_buf[buf_idx].size = item->size;
	dev->psc.ini_buf[sd_idx].do_not_free = false;

	return 0;

fail:
	kfree(buf);
	return -EIO;
}

static bool exceeds_psc_data_bounds(const struct psc_item *item,
				    const struct psc_identifier *psc_id, u32 data_size)
{
	if (!psc_has_per_item_crc(psc_id->psc_format_version))
		return (item->idx + item->size > data_size);

	return (item->idx + item->size + PAD_TO(u32, item->size) + sizeof(u32) > data_size);
}

/**
 * check_and_copy_psc_string() - Extract and copy a string from the PSC data
 *
 * @dev: fabric device object
 * @item: reference to PSC index data for this string
 *
 * If the entire PSCBIN is in memory, verify any section-specific CRC values and return a copy
 * of the string data.
 *
 * If not, extract the data from NVMEM and verify its CRC. request_pscdata_from_spi must set
 * dev->psc.mtd if reading from SPI.
 *
 * Return: pointer to a new NUL-terminated string on success, NULL on error
 */
static char *check_and_copy_psc_string(struct fdev *dev, const struct psc_item *item)
{
	const struct psc_data *psc = (const struct psc_data *)dev->psc.data;
	unsigned int offset;
	u32 check_crc;
	char *buf;
	int err;

	if (full_pscbin_in_memory(dev, psc->identifier.psc_format_version)) {
		if (psc_item_crc_ok(psc, item))
			return kmemdup_nul(&psc->data[item->idx], item->size, GFP_KERNEL);

		dev_err(fdev_dev(dev), "PSC string data CRC mismatch\n");
		return NULL;
	}

	buf = kmalloc(item->size + 1, GFP_KERNEL);
	if (!buf)
		return NULL;

	offset = SPI_HEADER_SIZE + sizeof(*psc) + item->idx;
	err = read_spi_data(dev, offset, item->size, buf, "string");
	if (err)
		goto fail;

	buf[item->size] = '\0';

	offset += item->size + PAD_TO(u32, item->size);
	err = read_spi_data(dev, offset, sizeof(u32), &check_crc, "string CRC");
	if (err)
		goto fail;

	if (check_crc != crc32c_with_padding((u8 *)buf, item->size)) {
		dev_err(fdev_dev(dev), "PSC string CRC mismatch\n");
		goto fail;
	}

	return buf;

fail:
	kfree(buf);
	return NULL;
}

/**
 * read_presence_config() - Copy presence config data from PSC
 *
 * @config: pointer to presence config header to populate
 * @dev: fabric device object
 * @item: reference to PSC index data for the presence config section
 *
 * If the entire PSCBIN is in memory, copy the presence config header from the identified section.
 *
 * If not, extract the data from NVMEM and verify its CRC. request_pscdata_from_spi must set
 * dev->psc.mtd if reading from SPI.
 *
 * Return: 0 on success, negative error code on error
 */
static int read_presence_config(struct psc_presence_config *config, struct fdev *dev,
				const struct psc_item *item)
{
	const struct psc_data *psc = (const struct psc_data *)dev->psc.data;
	unsigned int offset;
	int err;

	if (!psc_has_presence_rules(psc->identifier.psc_format_version)) {
		dev_err(fdev_dev(dev), "PSC version does not support presence rules\n");
		return -EINVAL;
	}

	if (item->size < sizeof(*config)) {
		dev_err(fdev_dev(dev), "Incomplete presence config header\n");
		return -ENODATA;
	}

	if (full_pscbin_in_memory(dev, psc->identifier.psc_format_version)) {
		memcpy(config, &psc->data[item->idx], sizeof(*config));
	} else {
		offset = SPI_HEADER_SIZE + sizeof(*psc) + item->idx;
		err = read_spi_data(dev, offset, sizeof(*config), config, "presence data");
		if (err)
			return -ENOENT;
	}

	if (config->crc32c_hdr != crc32c(0, config, offsetof(typeof(*config), crc32c_hdr))) {
		dev_err(fdev_dev(dev), "Presence config header CRC mismatch\n");
		return -EIO;
	}

	return 0;
}

/**
 * check_and_copy_presence_rules() - Extract and copy presence rules from PSC data
 *
 * @dev: fabric device object
 * @item: reference to PSC index data for the presence rules section
 * @subitem: reference to presence config index data for this device
 *
 * If the entire PSCBIN is in memory, verify section-specific CRC values and return a copy of the
 * presence rules for this device.
 *
 * If not, extract the data from NVMEM and verify its CRC. request_pscdata_from_spi must set
 * dev->psc.mtd if reading from SPI.
 *
 * Return: pointer to a new array of presence_data on success, NULL on error
 */
static struct psc_presence_rule *check_and_copy_presence_rules(struct fdev *dev,
							       const struct psc_item *item,
							       const struct psc_item *subitem)
{
	const struct psc_data *psc = (const struct psc_data *)dev->psc.data;
	struct psc_presence_config *config;
	struct psc_presence_rule *buf;
	unsigned int offset;
	u32 check_crc;
	int err;

	if (!psc_has_presence_rules(psc->identifier.psc_format_version)) {
		dev_err(fdev_dev(dev), "PSC version does not support presence rules\n");
		return NULL;
	}

	if (full_pscbin_in_memory(dev, psc->identifier.psc_format_version)) {
		u32 *crc;

		config = (struct psc_presence_config *)&psc->data[item->idx];
		crc = (u32 *)&config->data[subitem->idx + subitem->size];
		if (*crc != crc32c(0, &config->data[subitem->idx], subitem->size)) {
			dev_err(fdev_dev(dev), "Presence rules CRC mismatch\n");
			return NULL;
		}

		return kmemdup(&config->data[subitem->idx], subitem->size, GFP_KERNEL);
	}

	buf = kmalloc(item->size, GFP_KERNEL);
	if (!buf)
		return NULL;

	offset = SPI_HEADER_SIZE + sizeof(*psc) + item->idx + sizeof(*config) + subitem->idx;
	err = read_spi_data(dev, offset, subitem->size, buf, "presence rules");
	if (err)
		goto fail;

	offset += subitem->size;
	err = read_spi_data(dev, offset, sizeof(u32), &check_crc, "presence rules CRC");
	if (err)
		goto fail;

	if (check_crc != crc32c(0, buf, subitem->size)) {
		dev_err(fdev_dev(dev), "Presence rules CRC mismatch\n");
		goto fail;
	}

	return buf;

fail:
	kfree(buf);
	return NULL;
}

static int parse_presence_rules(struct fdev *dev, const struct psc_item *item)
{
	struct psc_presence_config config;
	struct psc_item *subitem;
	int err;

	err = read_presence_config(&config, dev, item);
	if (err)
		goto end;

	subitem = &config.rules[dev->pd->socket_id];
	if (!subitem->size)
		return 0;

	dev->psc.presence_rules = check_and_copy_presence_rules(dev, item, subitem);
	err = dev->psc.presence_rules ? 0 : -ENOENT;
	if (err)
		goto end;

	dev->psc.n_presence_rules = subitem->size / sizeof(struct psc_presence_rule);

	if (dev->psc.n_presence_rules) {
		struct psc_presence_rule *rule = dev->psc.presence_rules;
		int i;

		for (i = 0; i < dev->psc.n_presence_rules; ++i, ++rule) {
			dev_dbg(fdev_dev(dev), "Presence rule %d: %u:%u sd:%x p:%03x\n", i,
				rule->method, rule->index, rule->subdev_map, rule->port_map);
		}
	}

end:
	if (err)
		dev_err(fdev_dev(dev), "PSC invalid presence rules\n");

	return err;
}

static int validate_psc(struct fdev *dev)
{
	const struct psc_item *crc_data = NULL;
	const struct psc_item *presence_data;
	const struct psc_identifier *psc_id;
	const struct psc_item *product_name;
	const struct psc_item *brand_name;
	const struct psc_item *cust_data;
	const struct psc_item *ext_data;
	const struct psc_item *ini_name;
	const struct psc_item *comment;
	const struct psc_item *ini_bin;
	const u32 *hdr_crc32c;
	size_t hdr_crc_off;
	size_t hdr_size;
	u32 form_factor;
	const u8 *data;
	u32 data_size;
	u32 cfg_ver;
	int err = 0;
	u32 flags;
	u32 date;
	u32 time;
	u8 i;

	if (WARN_ON(dev->pd->socket_id >= MAX_SOCKET_IDS || dev->pd->sd_cnt > IAF_MAX_SUB_DEVS))
		return -EINVAL;

	if (dev->psc.size < sizeof(*psc_id) || !dev->psc.data) {
		dev_err(fdev_dev(dev), "No PSC header found\n");
		return -ENODATA;
	}

	psc_id = (const struct psc_identifier *)dev->psc.data;

	if (psc_id->magic != PSCBIN_MAGIC_NUMBER) {
		dev_err(fdev_dev(dev), "Invalid PSC header found\n");
		return -ENOENT;
	}

	switch (psc_id->psc_format_version) {
	case PSCBIN_VERSION_V0:
	case PSCBIN_VERSION_V2:
	case PSCBIN_VERSION_CURRENT:
		hdr_size = sizeof(struct psc_data);
		if (dev->psc.size >= hdr_size) {
			const struct psc_data *psc;

			psc = (const struct psc_data *)dev->psc.data;
			form_factor = psc->form_factor;
			cfg_ver = psc->cfg_version;
			date = psc->date;
			time = psc->time;
			flags = psc->flags;
			data_size = psc->data_size;
			brand_name = &psc->brand_name;
			product_name = &psc->product_name;
			comment = &psc->comment;
			ini_name = psc->ini_name;
			ini_bin = psc->ini_bin;
			ext_data = &psc->ext_data;
			cust_data = &psc->cust_data;
			presence_data = &psc->presence_data;
			hdr_crc32c = &psc->crc32c_hdr;
			hdr_crc_off = offsetof(struct psc_data, crc32c_hdr);
			data = psc->data;
		} else {
			dev_err(fdev_dev(dev), "Incomplete PSC header\n");
			return -ENODATA;
		}
		break;

	default:
		dev_err(fdev_dev(dev), "Unsupported PSC version %u detected\n",
			psc_id->psc_format_version);
		return -ENOENT;
	}

	/* V2 format has single data checksum (presence_data replaces it in V3) */
	if (psc_id->psc_format_version < PSCBIN_VERSION_V3) {
		crc_data = presence_data;
		presence_data = NULL;
	}

	if (full_pscbin_in_memory(dev, psc_id->psc_format_version) &&
	    hdr_size + data_size > dev->psc.size) {
		dev_err(fdev_dev(dev), "PSC data incomplete\n");
		return -ENODATA;
	}

	/* Check header CRC */
	if (*hdr_crc32c != crc32c(0, dev->psc.data, hdr_crc_off)) {
		dev_err(fdev_dev(dev), "PSC header CRC mismatch\n");
		return -ENOENT;
	}

	/* Check overall data CRC if present */
	if (crc_data) {
		const u32 *data_crc32c;

		if (crc_data->size != sizeof(u32) ||
		    crc_data->idx + crc_data->size > data_size) {
			dev_err(fdev_dev(dev), "PSC data CRC not found\n");
			return -ENOENT;
		}

		data_crc32c = (const u32 *)&data[crc_data->idx];
		if (*data_crc32c != crc32c(0, data, crc_data->idx)) {
			dev_err(fdev_dev(dev), "PSC data CRC mismatch\n");
			return -ENOENT;
		}
	}

	/*
	 * validate PSC fields
	 */

	if (form_factor >= FORM_FACTOR_END) {
		dev_err(fdev_dev(dev), "PSC invalid form factor specified\n");
		return -ENOENT;
	}

	if (!valid_bcd_date(date)) {
		dev_err(fdev_dev(dev), "PSC invalid date format\n");
		return -ENOENT;
	}

	if (!valid_bcd_time(time)) {
		dev_err(fdev_dev(dev), "PSC invalid time format\n");
		return -ENOENT;
	}

	if (flags)
		dev_warn(fdev_dev(dev), "Unrecognised PSC flags: %08x\n", flags);

	if (exceeds_psc_data_bounds(brand_name, psc_id, data_size)) {
		dev_err(fdev_dev(dev), "PSC invalid brand data\n");
		return -ENOENT;
	}

	if (exceeds_psc_data_bounds(product_name, psc_id, data_size)) {
		dev_err(fdev_dev(dev), "PSC invalid product data\n");
		return -ENOENT;
	}

	if (exceeds_psc_data_bounds(comment, psc_id, data_size)) {
		dev_err(fdev_dev(dev), "PSC invalid comment data\n");
		return -ENOENT;
	}

	scnprintf(dev->psc.version, sizeof(dev->psc.version),
		  "version %u : %04x/%02x/%02x %02x:%02x:%02x",
		  cfg_ver,
		  date >> 16, date >> 8 & 0xff, date & 0xff,
		  time >> 16, time >> 8 & 0xff, time & 0xff);

	/* Use as NULL terminated strings... */
	if (brand_name->size)
		dev->psc.brand = check_and_copy_psc_string(dev, brand_name);

	if (product_name->size)
		dev->psc.product = check_and_copy_psc_string(dev, product_name);

	dev_info(fdev_dev(dev), "PSC [%s] [%s] %s\n", dev->psc.brand,
		 dev->psc.product, dev->psc.version);

	if (psc_id->psc_format_version == PSCBIN_VERSION_V0) {
		dev_err(fdev_dev(dev), "SPI default NULL PSC provisioned, valid PSCBIN required\n");
		return -ENODATA;
	}

	for (i = 0; i < dev->pd->sd_cnt; ++i) {
		u8 n = dev->pd->socket_id * IAF_MAX_SUB_DEVS + i;

		if (exceeds_psc_data_bounds(&ini_name[n], psc_id, data_size)) {
			dev_err(fdev_dev(dev), "PSC invalid ini name %u\n", n);
			return -ENOENT;
		}

		if (!ini_bin[n].size) {
			dev_err(fdev_dev(dev), "PSC missing inibin[%u]\n", n);
			return -ENODATA;
		}

		if (exceeds_psc_data_bounds(&ini_bin[n], psc_id, data_size)) {
			dev_err(fdev_dev(dev), "PSC inibin[%u] outside of PSC data\n", n);
			return -EFBIG;
		}

		err = check_and_extract_ini_data(dev, i, &ini_bin[n]);

		if (err) {
			dev_err(fdev_dev(dev), "PSC inibin[%u] invalid\n", n);
			goto end;
		}
	}

	if (exceeds_psc_data_bounds(ext_data, psc_id, data_size)) {
		dev_err(fdev_dev(dev), "PSC invalid extra data\n");
		return -ENOENT;
	}

	if (exceeds_psc_data_bounds(cust_data, psc_id, data_size)) {
		dev_err(fdev_dev(dev), "PSC invalid customer data\n");
		return -ENOENT;
	}

	if (presence_data && presence_data->size)
		err = parse_presence_rules(dev, presence_data);

end:
	return err;
}

static int init_fw(struct fsubdev *sd)
{
	struct fdev *dev = sd->fdev;
	const struct psc_data *psc;
	struct mbdb_op_fw_version_rsp *fw_version = &sd->fw_version;
	u8 socket = sd->fdev->pd->socket_id;
	u8 sd_idx = sd_index(sd);
	const u8 *data;
	size_t size;
	int cnt;
	int err;

	err = wait_for_completion_killable(&dev->psc.done);
	if (err) {
		sd_err(sd, "Wait for psc done failed: %d\n", err);
		return err;
	}

	if (dev->psc.err)
		return dev->psc.err;

	/* psc is only valid after the completion and if no error occurred */
	psc = (const struct psc_data *)READ_ONCE(dev->psc.data);

	if (full_pscbin_in_memory(dev, psc->identifier.psc_format_version)) {
		size = psc->ini_bin[socket * IAF_MAX_SUB_DEVS + sd_idx].size;
		data = &psc->data[psc->ini_bin[socket * IAF_MAX_SUB_DEVS + sd_idx].idx];
	} else {
		size = dev->psc.ini_buf[sd_idx].size;
		data = dev->psc.ini_buf[sd_idx].data;
	}
	err = data && size ? 0 : -ENOENT;
	if (err) {
		sd_err(sd, "ini_bin data missing\n");
		goto init_failed;
	}

	err = copy_ini_to_device(sd, data, size);
	if (err)
		goto init_failed;

	/*
	 * FW documentation indicates to wait 20 ms, then verify that the INIT
	 * bit reported by FW_VERSION is set. Allow for slightly longer delays
	 * in case the FW is somehow busy.
	 */
	cnt = 1;
	do {
		msleep(20);
		err = ops_fw_version(sd, fw_version);
	} while (!err &&
		 !(fw_version->environment & FW_VERSION_INIT_BIT) &&
		 cnt++ < MAX_20MS_DELAYS);

	if (err)
		goto init_failed;

	sd_dbg(sd, "FW init took at least %d ms\n", 20 * cnt);

	if (!(fw_version->environment & FW_VERSION_INIT_BIT)) {
		sd_err(sd, "FW never finished initializing\n");
		err = -EBUSY;
		goto init_failed;
	}

	report_fw_info(sd, fw_version, "after firmware init");

	if (!has_all_opcodes(fw_version->supported_opcodes,
			     required_runtime_opcodes)) {
		sd_err(sd, "Required FW opcodes not supported\n");
		err = -EPERM;
		goto init_failed;
	}

	return 0;

init_failed:
	sd_err(sd, "could not initialize firmware\n");
	return err;
}

static int request_pscdata_from_fw(struct fdev *dev, const char *filename)
{
	int err;

	err = request_firmware(&dev->psc.as_fw, filename, fdev_dev(dev));
	if (err) {
		dev_err(fdev_dev(dev), "%s not found via FW API\n", filename);
		goto end;
	}

	dev->psc.data = dev->psc.as_fw->data;
	dev->psc.size = dev->psc.as_fw->size;
	dev_info(fdev_dev(dev), "Loaded %s via FW API\n", filename);

end:
	return err;
}

static int spi_driver_match_fn(struct device *dev, void *data)
{
	return strcmp(dev_driver_string(dev), I915_SPI_DRIVER_NAME_AUX) == 0 ||
		strcmp(dev_driver_string(dev), I915_SPI_DRIVER_NAME_MFD) == 0;
}

/**
 * request_pscdata_from_spi() - Read PSC data from SPI flash into device object
 *
 * @dev: Fabric device object
 *
 * Attempts to locate the PSC SPI region using the MTD infrastructure.
 *
 * Return: 0 on success, -ENODEV if SPI or MTD device not found (caller should retry after giving
 * SPI time to load), other values indicate a non-recoverable error.
 */
static int request_pscdata_from_spi(struct fdev *dev)
{
	struct psc_data *header;
	struct device *spi_dev;
	struct mtd_info *mtd;
	unsigned int offset;
	size_t mtd_name_len;
	char *mtd_name;
	u8 *buf = NULL;
	size_t size;
	int err = 0;

	/* see if the SPI device that is a child of our parent i915 device is ready */
	spi_dev = device_find_child(dev->pdev->dev.parent, NULL, spi_driver_match_fn);
	if (!spi_dev)
		return -ENODEV;

	/* sizeof() includes space for terminating NUL */
	mtd_name_len = strlen(dev_name(spi_dev)) + sizeof(PSC_REGION_QUALIFIER);
	mtd_name = kmalloc(mtd_name_len, GFP_KERNEL);
	if (!mtd_name) {
		put_device(spi_dev);
		return -ENOMEM;
	}

	/* MTD region name matches SPI device name with ".PSC" appended */
	mtd_name[0] = '\0';
	strncat(mtd_name, dev_name(spi_dev), mtd_name_len);
	put_device(spi_dev);
	strncat(mtd_name, PSC_REGION_QUALIFIER, mtd_name_len);

	/* get_mtd_device_nm returns -ENODEV if not found */
	mtd = get_mtd_device_nm(mtd_name);
	kfree(mtd_name);
	if (IS_ERR(mtd))
		return PTR_ERR(mtd);

	dev->psc.mtd = mtd;

	header = kmalloc(sizeof(*header), GFP_KERNEL);

	if (!header) {
		err = -ENOMEM;
		goto cleanup;
	}

	offset = SPI_HEADER_SIZE;
	err = read_spi_data(dev, offset, sizeof(*header), header, "header");
	if (err)
		goto cleanup;

	/*
	 * V2 or earlier: read the rest of the PSCBIN data now, required for CRC checking
	 * V3 or later: read only the header now, each data item has its own CRC
	 */
	if (header->identifier.psc_format_version < PSCBIN_VERSION_V3) {
		size = struct_size(header, data, header->data_size);
		if (size > MAX_PSC_SIZE) {
			err = -EBADF;
			goto cleanup;
		}

		buf = kmalloc(size, GFP_KERNEL);
		if (!buf) {
			err = -ENOMEM;
			goto cleanup;
		}

		memcpy(buf, header, sizeof(*header));
		offset += sizeof(*header);

		err = read_spi_data(dev, offset, header->data_size, buf + sizeof(*header), "data");
		if (err)
			goto cleanup;
	} else {
		size = sizeof(*header);
		buf = (u8 *)header;
		header = NULL;
	}

	dev->psc.size = size;
	dev->psc.data = buf;

cleanup:

	if (err) {
		dev_err(fdev_dev(dev), "Error %d loading PSC\n", err);
		kfree(buf);
	}

	kfree(header);

	return err;
}

static void release_pscdata(struct fdev *dev)
{
	if (dev->psc.as_fw)
		release_firmware(dev->psc.as_fw);
	else
		kfree(dev->psc.data);

	dev->psc.as_fw = NULL;
	dev->psc.data = NULL;
	dev->psc.size = 0;
}

static void fw_initialization_complete(struct fdev *dev)
{
	/*
	 * A failure before init_fw() will leave memory in use by the PSCBIN
	 * thread.  Wait for PSCBIN thread to complete before cleanup.
	 * The abort ensures clean up if an error has occurred by causing the psc thread to exit.
	 * Completing the abort completion wakes the PSC thread (if it exists) and ensures it
	 * reaches the done completion in a timely manner, avoiding the need to handle timeouts on
	 * the done completion here.
	 */

	complete_all(&dev->psc.abort);
	wait_for_completion(&dev->psc.done);

	if (is_fdev_registered(dev))
		dev_info(fdev_dev(dev), "Firmware Version: %s\n",
			 dev->sd[0].fw_version.fw_version_string);

	/* release resources required for FW initialization */
	release_pscdata(dev);
	release_firmware(dev->fw);

	/* complete remaining device initialization */
	iaf_complete_init_dev(dev);
}

static int verify_preload(struct fsubdev *sd)
{
	struct mbdb_op_fw_version_rsp *fw_version = &sd->fw_version;
	int err;

	/* initial query to preloaded firmware likely to be a seq error */
	err = ops_fw_version(sd, fw_version);
	if (err == MBOX_RSP_STATUS_SEQ_NO_ERROR)
		err = ops_fw_version(sd, fw_version);

	if (err) {
		sd_err(sd, "fw version query failed: %d\n", err);
		return err;
	}

	if (!(fw_version->environment & FW_VERSION_ENV_BIT)) {
		sd_err(sd, "FW not preloaded\n");
		return -EINVAL;
	}

	report_fw_info(sd, fw_version, "preloaded firmware");

	if (!(fw_version->environment & FW_VERSION_INIT_BIT)) {
		sd_err(sd, "FW not ready\n");
		return -EINVAL;
	}

	if (!has_all_opcodes(fw_version->supported_opcodes, required_runtime_opcodes)) {
		sd_err(sd, "Required FW opcodes not supported\n");
		return -EINVAL;
	}

	return 0;
}

static void load_and_init_subdev(struct fsubdev *sd)
{
	struct fdev *dev = sd->fdev;
	int err;

	if (dev_is_preload(sd->fdev)) {
		err = verify_preload(sd);
		if (err)
			goto cleanup;
	} else {
		err = load_fw(sd);
		if (err)
			goto cleanup;

		err = start_fw(sd);
		if (err)
			goto cleanup;

		err = init_fw(sd);
		if (err)
			goto cleanup;
	}

	WRITE_ONCE(sd->fw_running, true);

	if (!dev_is_runtime_debug(sd->fdev)) {
		err = initialize_fports(sd);
		if (err)
			goto cleanup;
	} else {
		sd_dbg(sd, "debug mode: skipping port init\n");
	}

	reset_errors(sd);

	sd_dbg(sd, "subdevice init complete\n");

cleanup:

	if (err)
		indicate_subdevice_error(sd, SD_ERROR_FW);

	/* clean up/complete device init if this is the last sd */
	if (atomic_dec_and_test(&dev->fwinit_refcnt))
		fw_initialization_complete(dev);

	/* release the per-sd initialization flow reference on the parent */
	fdev_put(dev);
}

static void fetch_tx_calibration(struct fdev *dev)
{
	const struct psc_data *psc_blob = (const struct psc_data *)dev->psc.data;
	u32 txcal_start = round_up(psc_blob->data_size, sizeof(u32));
	unsigned int spi_offset = SPI_HEADER_SIZE + struct_size(psc_blob, data, txcal_start);
	struct txcal_blob hdr_from_spi;
	const struct txcal_blob *hdr;
	size_t blob_size;
	int err;

	if (dev->psc.as_fw) {
		const struct txcal_settings *psc_buf_end;

		hdr = (const struct txcal_blob *)&psc_blob->data[txcal_start];
		psc_buf_end = (const struct txcal_settings *)&dev->psc.data[dev->psc.size];

		/* verify both header and data fit */
		if (hdr->data > psc_buf_end || &hdr->data[hdr->num_settings] > psc_buf_end) {
			dev_warn(fdev_dev(dev), "TX calibration header not in PSC image");
			return;
		}
	} else {
		/* attempt to read header from SPI */
		hdr = &hdr_from_spi;
		err = read_spi_data(dev, spi_offset, sizeof(hdr_from_spi), &hdr_from_spi,
				    "TX calibration header");
		if (err) {
			dev_warn(fdev_dev(dev), "TX calibration header does not fit in SPI");
			return;
		}
	}

	/* verify txcal_blob */

	blob_size = struct_size(hdr, data, hdr->num_settings);

	if (hdr->magic[0] != TXCAL_BLOB_MAGIC_0 || hdr->magic[1] != TXCAL_BLOB_MAGIC_1 ||
	    hdr->magic[2] != TXCAL_BLOB_MAGIC_2 || hdr->magic[3] != TXCAL_BLOB_MAGIC_3) {
		dev_warn(fdev_dev(dev), "TX calibration header not found");
		return;
	}

	if (hdr->crc32c_hdr != crc32c(0, hdr, offsetof(struct txcal_blob, crc32c_hdr))) {
		dev_warn(fdev_dev(dev), "TX calibration header CRC mismatch\n");
		goto error;
	}

	if (hdr->format_version < TXCAL_VERSION_MIN || hdr->format_version > TXCAL_VERSION_MAX) {
		dev_warn(fdev_dev(dev), "TX calibration unsupported version\n");
		goto error;
	}

	if (hdr->size < blob_size) {
		dev_warn(fdev_dev(dev), "TX calibration data size mismatch\n");
		goto error;
	}

	if (!valid_bcd_date(hdr->date)) {
		dev_warn(fdev_dev(dev), "TX calibration invalid date format\n");
		goto error;
	}

	if (!valid_bcd_time(hdr->time)) {
		dev_warn(fdev_dev(dev), "TX calibration invalid time format\n");
		goto error;
	}

	/* BCD-encoded date/time: upper time byte should be 0x00, but ensure all bits are shown */
	dev_info(fdev_dev(dev), "TX calibration version %u : %04x/%02x/%02x %02x:%02x:%02x",
		 hdr->cfg_version, hdr->date >> 16, hdr->date >> 8 & 0xff, hdr->date & 0xff,
		 hdr->time >> 16, hdr->time >> 8 & 0xff, hdr->time & 0xff);

	dev->psc.txcal = kmalloc(blob_size, GFP_KERNEL);
	if (!dev->psc.txcal)
		goto error;

	if (dev->psc.as_fw) {
		memcpy(dev->psc.txcal, hdr, blob_size);
	} else {
		memcpy(dev->psc.txcal, hdr, sizeof(*hdr));
		err = read_spi_data(dev,
				    spi_offset + offsetof(struct txcal_blob, data),
				    hdr->num_settings * sizeof(struct txcal_settings),
				    dev->psc.txcal->data,
				    "TX calibration data");
		if (err) {
			dev_warn(fdev_dev(dev), "TX calibration data does not fit in SPI");
			goto error;
		}
	}

	if (hdr->crc32c_data !=
	    crc32c(0, dev->psc.txcal->data, flex_array_size(hdr, data, hdr->num_settings))) {
		dev_warn(fdev_dev(dev), "TX calibration data CRC mismatch\n");
		goto error;
	}

	return;

error:
	kfree(dev->psc.txcal);
	dev->psc.txcal = NULL;
	dev_warn(fdev_dev(dev), "TX calibration data incomplete or corrupted");
}

static void fetch_platform_specific_config(struct fdev *dev)
{
	bool fetching_psc_from_spi;
	u16 cnt;

	dev_dbg(fdev_dev(dev), "Reading provisioned psc data\n");

	dev->psc.err = -ENOENT;

	/* cnt (u16) can hold MAX_200MS_RETRIES+1 */
	cnt = 1;

	kernel_param_lock(THIS_MODULE);
	fetching_psc_from_spi = !(psc_file_override && *psc_file_override);
	if (!fetching_psc_from_spi)
		dev->psc.err = request_pscdata_from_fw(dev, psc_file_override);
	kernel_param_unlock(THIS_MODULE);

	if (fetching_psc_from_spi)
		do {
			/* Only emit one time after 1 minute */
			if (cnt == ONE_MINUTE_COUNT)
				dev_info(fdev_dev(dev), "Waiting on psc data from spi device\n");
			dev->psc.err = request_pscdata_from_spi(dev);
		} while (dev->psc.err == -ENODEV &&
			 cnt++ < MAX_200MS_RETRIES &&
			 !wait_for_completion_killable_timeout(&dev->psc.abort,
							       msecs_to_jiffies(200)));

	if (dev->psc.err) {
		dev_err(fdev_dev(dev), "Error %d requesting PSC data\n", dev->psc.err);
		goto end;
	}

	dev->psc.err = validate_psc(dev);

	fetch_tx_calibration(dev);

end:

	if (dev->psc.mtd) {
		put_mtd_device(dev->psc.mtd);
		dev->psc.mtd = NULL;
	}

	/*
	 * Subdevice initialization threads only use the PSC data after waiting for this completion
	 * and only if psc.err is not set
	 *
	 * The last subdevice initialization thread to end (successfully or in error) then calls
	 * fw_initialization_complete(), which also waits for this completion (in case subdevice
	 * initialization fails early) before releasing PSC data
	 */

	complete_all(&dev->psc.done);

	fdev_put(dev);
}

/**
 * prep_ops_and_fw_version - set the FW version and make sure the sequence is in line
 * @dev: valid ANR device
 *
 * Make sure mailbox seq_nos are synced between KMD and FW.  If a good response
 * is received, the driver will check that it is communicating with the bootrom
 * and take appropriate actions if not
 *
 * ops_fw_version is the only ops request that does not need a ops valiation.
 * For any ops_xxx calls, the current firmware is need.
 *
 * NOTE: if this is not the bootloader, later init will reset the SD.
 *
 * Return: 0 on success, else negative error code
 */
static int prep_ops_and_fw_version(struct fdev *dev)
{
	struct mbdb_op_fw_version_rsp *fw_version;
	int err;
	int i;

	for (i = 0; i < dev->pd->sd_cnt; i++) {
		fw_version = &dev->sd[i].fw_version;
		err = ops_fw_version(&dev->sd[i], fw_version);
		if (err == MBOX_RSP_STATUS_SEQ_NO_ERROR)
			err = ops_fw_version(&dev->sd[i], fw_version);
		if (err)
			return err;
	}

	return 0;
}

static const char *select_pvc_fw(struct fsubdev *sd)
{
	u64 fuses = 0;
	int err;

	err = ops_csr_raw_read(sd, CSR_CP_DEV_EFUSE_VERSION, sizeof(fuses), &fuses);
	if (err) {
		sd_err(sd, "Failed to read efuse info: %d\n", err);
		return NULL;
	}
	switch (FIELD_GET(CP_DEV_EFUSE_VERSION_VARIANT_MASK, fuses)) {
	case CP_DEV_EFUSE_VERSION_VARIANT_NORMAL:
		/* WA: HSD-16011092478 */
		if (IS_ANR_STEP(sd, ANR_ARI_STEP_A0, ANR_ARI_STEP_A_LAST)) {
			sd_info(sd, "A0 firmware\n");
			return PVC_FW_IMAGE_DEBUG_SIGNED;
		}

		return PVC_FW_IMAGE;

	case CP_DEV_EFUSE_VERSION_VARIANT_EXPORT:
		return PVC_FW_IMAGE_EXPORT_LIMITED;

	default:
		sd_err(sd, "Invalid fuse info: %llu\n", fuses);
		break;
	}

	return NULL;
}

static const char *get_fw_image(struct fdev *dev)
{
#if IS_ENABLED(CONFIG_IAF_DEBUG_ALTERNATE_FW_LOAD)
	if (fw_image && fw_image[0])
		return fw_image;
#endif
	switch (dev->pd->product) {
	case IAF_PONTEVECCHIO:
		return select_pvc_fw(&dev->sd[0]);

	default:
		break;
	}

	return NULL;
}

/**
 * load_and_init_fw() - Loads and initializes the device with firmware
 *
 * @dev: Fabric device object
 *
 * Multiple work items are started with this function:
 *  1 per device to retrieve the device PSCBIN
 *  1 per sub-device for firmware load and initialization
 *
 * The firmware initialization is dependent on the PSCBIN and will block until the PSCBIN is
 * available.
 *
 * The PSCBIN load is put on a work queue because it may need to wait for the SPI driver to load.
 * If this was done inline, the wait could block the probe for MAX_200MS_RETRIES before failing.
 *
 * Return: 0 on success, -EINVAL if firmware cannot be found, other values indicate a
 * non-recoverable error.
 */
int load_and_init_fw(struct fdev *dev)
{
	const char *fw_image = NULL;
	int err;

	if (dev_is_startup_debug(dev)) {
		/* complete remaining device initialization immediately */
		iaf_complete_init_dev(dev);
		return 0;
	}

	err = prep_ops_and_fw_version(dev);
	if (err) {
		dev_err(fdev_dev(dev), "Ops sequence init failed: %d\n", err);
		return err;
	}

	fw_image = get_fw_image(dev);
	if (!fw_image) {
		dev_err(fdev_dev(dev), "Could not determine fw file\n");
		return -EINVAL;
	}

	kernel_param_lock(THIS_MODULE);
	dev_dbg(fdev_dev(dev), "Reading firmware file: %s\n", fw_image);
	err = request_firmware(&dev->fw, fw_image, fdev_dev(dev));
	kernel_param_unlock(THIS_MODULE);

	if (err) {
		dev_err(fdev_dev(dev), "Could not open fw file\n");
		return err;
	}

	err = verify_fw(dev);
	if (err) {
		dev_err(fdev_dev(dev), "Invalid fw file\n");
		release_firmware(dev->fw);
		return err;
	}

	queue_load_and_init_all_subdevs(dev);

	fdev_get_early(dev);
	queue_work(iaf_unbound_wq, &dev->psc.work);

	return 0;
}

void flush_any_outstanding_fw_initializations(struct fdev *dev)
{
	int i;

	complete_all(&dev->psc.abort);
	flush_work(&dev->psc.work);

	for (i = 0; i < dev->pd->sd_cnt; ++i)
		flush_work(&dev->sd[i].fw_work);
}

/*
 * Workqueue support functions
 */

static void queue_load_and_init_all_subdevs(struct fdev *dev)
{
	int i;

	for (i = 0; i < dev->pd->sd_cnt; ++i) {
		/*
		 * each subdevice holds an initialization reference for the
		 * lifetime of the subdevice init flow to prevent parent device
		 * remove races.
		 */
		fdev_get_early(dev);

		queue_work(iaf_unbound_wq, &dev->sd[i].fw_work);
	}
}

static void wrk_load_and_init_subdev(struct work_struct *work)
{
	struct fsubdev *sd = container_of(work, struct fsubdev, fw_work);

	load_and_init_subdev(sd);

	create_optional_dev_debugfs_files(sd);
}

static void wrk_load_psc_data(struct work_struct *work)
{
	fetch_platform_specific_config(container_of(work, struct fdev, psc.work));
}
