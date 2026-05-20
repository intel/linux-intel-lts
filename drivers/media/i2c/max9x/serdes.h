/*
 * serdes.h
 *
 * Copyright (c) 2018-2020 D3 Engineering.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Intel Corporation.

#ifndef _SERDES_H_
#define _SERDES_H_

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <uapi/linux/media-bus-format.h>

#include <media/serdes-pdata.h>
#include "max9x_pdata.h"

#define MAX9X_VDD_REGULATOR_NAME "vdd"
#define MAX9X_POC_REGULATOR_NAME "poc"
#define MAX9X_RESET_GPIO_NAME "reset"
#define MAX9X_DEV_ID 0xD
#define MAX9X_DEV_REV_FIELD GENMASK(3, 0)

/*Used for device attributes*/
#define ATTR_NAME_LEN (30) /* arbitrary number used to allocate an attribute */
#define ATTR_READ_ONLY (0444)

#define MAX9X_LINK_FREQ_MBPS_TO_HZ(mbps) (((unsigned long long)(mbps)*1000000ULL)/2ULL)
#define MAX9X_LINK_FREQ_HZ_TO_MBPS(hz) (((unsigned long long)(hz)*2ULL)/1000000ULL)
#define MAX9X_LINK_FREQ_MBPS_TO_REG(mbps) ((mbps)/100U)

#define MAX9X_FIELD_PREP(_mask, _val)					\
	({								\
		((typeof(_mask))(_val) << __bf_shf(_mask)) & (_mask);	\
	})

#define TRY(err, expr) \
	do {\
		err = expr; \
		if (err) { \
			return err; \
		} \
	} while (0)

#define TRY_DEV_HERE(err, expr, dev) \
	do { \
		err = expr; \
		if (err) { \
			dev_err((dev), \
				"%s failed (%d) in %s() at %s:%d", #expr, \
				(err), __func__, __FILE__, __LINE__); \
			return err; \
		} \
	} while (0)

/* dt is defined in soft_dt_x, such as BACKTOP15(0x316).[5:0] */
#define MIPI_CSI2_TYPE_YUV422_8		0x1e
#define MIPI_CSI2_TYPE_YUV422_10	0x1f
#define MIPI_CSI2_TYPE_RGB565		0x22
#define MIPI_CSI2_TYPE_RGB888		0x24
#define MIPI_CSI2_TYPE_RAW8		0x2a
#define MIPI_CSI2_TYPE_RAW10		0x2b
#define MIPI_CSI2_TYPE_RAW12		0x2c

static inline int mbus_code_to_csi_dt(int code)
{
	switch (code) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		return MIPI_CSI2_TYPE_RGB565;
	case MEDIA_BUS_FMT_RGB888_1X24:
		return MIPI_CSI2_TYPE_RGB888;
	case MEDIA_BUS_FMT_YUYV10_1X20:
		return MIPI_CSI2_TYPE_YUV422_10;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
		return MIPI_CSI2_TYPE_YUV422_8;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MIPI_CSI2_TYPE_RAW12;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MIPI_CSI2_TYPE_RAW10;
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MIPI_CSI2_TYPE_RAW8;
	default:
		return -EINVAL;
	}
}

static const struct regmap_config max9x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

enum max9x_chip_type {
	MAX9296 = 0,
	MAX96724 = 1,
	MAX9295,
	MAX96717,
	MAX96724F,
	MAX96724R,
};

enum max9x_serdes_type {
	MAX9X_DESERIALIZER = 0,
	MAX9X_SERIALIZER = 1,
};

struct max9x_serdes_rate_table {
	unsigned int val;
	unsigned int freq_mhz;
};

struct max9x_serdes_csi_config {
	struct max9x_serdes_phy_map *map;
	unsigned int num_maps;
	enum v4l2_mbus_type bus_type;
	unsigned int num_lanes;
	unsigned int freq_mhz;
	unsigned int initial_deskew_width;
	bool auto_init_deskew_enabled;
	bool auto_start;
};

struct max9x_serdes_pipe_config {
	// Deserializer
	unsigned int src_link;
	unsigned int src_pipe;
	struct max9x_serdes_mipi_map *map;
	unsigned int num_maps;

	// Serializer
	//TODO: dst_link?
	unsigned int src_csi;
	unsigned int *data_type;
	unsigned int num_data_types;
	//TODO: MIPI VC filter mask

	/*
	 * Which bpp value to double i.e. dbl_pixel_bpp = 10 will
	 * cause 10 bit data to be transmitted together.
	 */
	//TODO: Multiple dbl bpp values
	unsigned int dbl_pixel_bpp;

	/*
	 * Software override for bits per pixel. This is compatible with
	 * double bpp. These are the min/max values before padding
	 * and after doubling. Leave either 0 to disable.
	 */
	unsigned int soft_min_pixel_bpp;
	unsigned int soft_max_pixel_bpp;

	//TODO: MIPI DT override
};

struct max9x_serdes_serial_config {
	enum max9x_serdes_link_type link_type;
	unsigned int rx_freq_mhz; // Previously forward_freq_mhz
	unsigned int tx_freq_mhz; // Previously back_freq_mhz
};

struct max9x_serdes_v4l {
	struct mutex lock;
	struct max9x_common *common;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_handler;
	struct media_pad *pads;
	int num_pads;

	struct v4l2_ctrl *link_freq; // CSI link frequency, used to determine ISP clock
	struct v4l2_mbus_framefmt *ffmts;
	int ref_count;
};

struct max9x_serdes_csi_link {
	bool enabled;
	unsigned int usecount;
	struct max9x_serdes_csi_config config;
	struct mutex csi_mutex;
};

struct max9x_serdes_video_pipe {
	bool enabled;
	//TODO: Anything else need to be tracked?
	struct max9x_serdes_pipe_config config;
};

struct max9x_serdes_serial_link {
	bool enabled;
	bool detected;
	struct regulator *poc_regulator; /* feeds the serializer, imager */
	bool regulator_enabled;
	struct {
		struct i2c_client *client;
		struct max9x_subdev_pdata *pdata;
	} remote;
	struct regmap *map;
	struct max9x_serdes_serial_config config;
	struct device_attribute *link_lock_status;
};

struct max9x_serdes_line_fault {
	bool enabled;
	struct device_attribute *line_fault_status;
};

struct max9x_common {
	struct max9x_desc *des;
	struct device *dev;
	struct i2c_client *client;
	struct regmap *map;
	struct i2c_client *phys_client;
	struct regmap *phys_map;
	struct i2c_mux_core *muxc;
	struct gpio_chip gpio_chip;
	enum max9x_serdes_type type;

	struct gpio_desc *reset_gpio;
	struct regulator *vdd_regulator;
	bool regulator_enabled;

	struct max9x_common_ops *common_ops;
	struct max9x_serial_link_ops *serial_link_ops;
	struct max9x_csi_link_ops *csi_link_ops;
	struct max9x_line_fault_ops *line_fault_ops;
	struct max9x_translation_ops *translation_ops;

	struct max9x_serdes_csi_link *csi_link;
	int num_csi_links;

	struct max9x_serdes_video_pipe *video_pipe;
	int num_video_pipes;

	struct max9x_serdes_serial_link *serial_link;
	int num_serial_links;

	struct max9x_serdes_line_fault *line_fault;
	int num_line_faults;

	struct max9x_serdes_v4l v4l;

	struct mutex link_mutex;
	struct mutex isolate_mutex;
	int isolated_link;
	int selected_link;
	bool external_refclk_enable;
};

int max9x_serdes_mhz_to_rate(struct max9x_serdes_rate_table *table, int len, unsigned int freq_mhz);
struct max9x_common *max9x_sd_to_common(struct v4l2_subdev *sd);
struct max9x_common *max9x_client_to_common(struct i2c_client *client);

enum max9x_element_type {
	MAX9X_SERIAL_LINK = 0,
	MAX9X_VIDEO_PIPE,
	MAX9X_MIPI_MAP,
	MAX9X_CSI_LINK,
	MAX9X_LINE_FAULT,
	MAX9X_DATA_TYPES
};

struct max9x_common_ops {
	int (*soft_reset)(struct max9x_common *common);
	int (*enable)(struct max9x_common *common);
	int (*disable)(struct max9x_common *common);
	int (*max_elements)(struct max9x_common *common, enum max9x_element_type element);
	int (*verify_devid)(struct max9x_common *common);
	int (*remap_addr)(struct max9x_common *common);
	int (*remap_reset)(struct max9x_common *common);
	int (*setup_gpio)(struct max9x_common *common);
};

struct max9x_serial_link_ops {
	int (*enable)(struct max9x_common *common, unsigned int link);
	int (*disable)(struct max9x_common *common, unsigned int link);
	int (*select)(struct max9x_common *common, unsigned int link);
	int (*deselect)(struct max9x_common *common, unsigned int link);
	int (*get_locked)(struct max9x_common *common, unsigned int link, bool *locked);
	int (*isolate)(struct max9x_common *common, unsigned int link);
	int (*deisolate)(struct max9x_common *common, unsigned int link);
};

struct max9x_csi_link_ops {
	int (*enable)(struct max9x_common *common, unsigned int link);
	int (*disable)(struct max9x_common *common, unsigned int link);
};

struct max9x_translation_ops {
	int (*add)(struct max9x_common *common, unsigned int i2c_id, unsigned int src, unsigned int dst);
	int (*remove)(struct max9x_common *common, unsigned int i2c_id, unsigned int src, unsigned int dst);
};

struct max9x_line_fault_ops {
	int (*enable)(struct max9x_common *common, unsigned int line);
	int (*disable)(struct max9x_common *common, unsigned int line);
	int (*get_status)(struct max9x_common *common, unsigned int line);
};

struct max9x_desc {
	char dev_id;
	char rev_reg;
	enum max9x_serdes_type serdes_type;
	enum max9x_chip_type chip_type;
	int (*get_max9x_ops)(char dev_id,
		struct max9x_common_ops **common_ops,
		struct max9x_serial_link_ops **serial_ops,
		struct max9x_csi_link_ops **csi_ops,
		struct max9x_line_fault_ops **lf_ops,
		struct max9x_translation_ops **trans_ops);
};

int max9x_common_init_i2c_client(struct max9x_common *common,
	struct i2c_client *client,
	const struct regmap_config *regmap_config,
	struct max9x_common_ops *common_ops,
	struct max9x_serial_link_ops *serial_link_ops,
	struct max9x_csi_link_ops *csi_link_ops,
	struct max9x_line_fault_ops *lf_ops);
void max9x_destroy(struct max9x_common *common);
int max9x_common_resume(struct max9x_common *common);
int max9x_common_suspend(struct max9x_common *common);
int max9x_get_ops(char dev_id, struct max9x_common_ops **common_ops,
		struct max9x_serial_link_ops **serial_ops,
		struct max9x_csi_link_ops **csi_ops,
		struct max9x_line_fault_ops **lf_ops,
		struct max9x_translation_ops **trans_ops);
extern int max96724_get_ops(struct max9x_common_ops **common_ops,
			    struct max9x_serial_link_ops **serial_ops,
			    struct max9x_csi_link_ops **csi_ops,
			    struct max9x_line_fault_ops **lf_ops,
			    struct max9x_translation_ops **trans_ops);
extern int max96717_get_ops(struct max9x_common_ops **common_ops,
			    struct max9x_serial_link_ops **serial_ops,
			    struct max9x_csi_link_ops **csi_ops,
			    struct max9x_line_fault_ops **lf_ops,
			    struct max9x_translation_ops **trans_ops);
extern int max9296_get_ops(struct max9x_common_ops **common_ops,
			   struct max9x_serial_link_ops **serial_ops,
			   struct max9x_csi_link_ops **csi_ops,
			   struct max9x_line_fault_ops **lf_ops,
			   struct max9x_translation_ops **trans_ops);
extern int max9295_get_ops(struct max9x_common_ops **common_ops,
			   struct max9x_serial_link_ops **serial_ops,
			   struct max9x_csi_link_ops **csi_ops,
			   struct max9x_line_fault_ops **lf_ops,
			   struct max9x_translation_ops **trans_ops);
/*
 * Both DES and SER have their own i2c_mux_core:
 * 1. SER's muxc does not provide select/deselect and has at most one link
 *    (which itself has all children of the SER/DES link)
 * 2. DER's muxc has driver specific select/deselect, one or more links,
 *    each of which must have exactly one SER
 * 3. SER's probe() performs re-numbering and address translation (if needed:
 *    i2c-mux means this is no longer strictly required)
 *
 * des -> i2c-mux -> link@0 -> ser -> sensor, eeprom, etc
 *                   link@n -> ser -> sensor, eeprom, etc
 *
 * +-----+
 * | des |
 * +--+--+
 *    |
 *    |  +--------+
 *    +--+ link@0 |
 *    |  +------+-+
 *    |         |    +-----+
 *    |         +----+ ser |
 *    |              +--+--+
 *    |                 |
 *    |                 |  +--------+
 *    |                 +--+ link@0 |
 *    |                    +-----+--+
 *    |                          |
 *    |                          |    +--------+
 *    |                          +----+ sensor |
 *    |                          |    +--------+
 *    |                          |
 *    |                          |    +--------+
 *    |                          +----+ eeprom |
 *    |                               +--------+
 *    |
 *    |  +--------+
 *    +--+ link@n |
 *       +------+-+
 *              |    +-----+
 *              +----+ ser |
 *                   +--+--+
 *                      |
 *                      |  +--------+
 *                      +--+ link@0 |
 *                         +-----+--+
 *                               |
 *                               |    +--------+
 *                               +----+ sensor |
 *                               |    +--------+
 *                               |
 *                               |    +--------+
 *                               +----+ eeprom |
 *                                    +--------+
 */

// for pdata parse

#define SET_CSI_MAP(maps, i, _src_vc, _src_dt, _dst_vc, _dst_dt, _dst_csi) do { \
	(maps)[i].src_vc = _src_vc; \
	(maps)[i].src_dt = _src_dt; \
	(maps)[i].dst_vc = _dst_vc; \
	(maps)[i].dst_dt = _dst_dt; \
	(maps)[i].dst_csi = _dst_csi; \
} while (0)

#define SET_PHY_MAP(maps, i, _int_csi, _phy_ind, _phy_lane) do { \
	(maps)[i].int_csi = _int_csi; \
	(maps)[i].phy_ind = _phy_ind; \
	(maps)[i].phy_lane = _phy_lane; \
} while (0)

#endif // _SERDES_H_
