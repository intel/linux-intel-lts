// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Intel Corporation.

#ifndef _MAX9X_PDATA_H_
#define _MAX9X_PDATA_H_

#include <linux/kernel.h>
#include <linux/i2c.h>

struct max9x_common;

enum max9x_serdes_link_type {
	MAX9X_LINK_TYPE_GMSL1 = 0,
	MAX9X_LINK_TYPE_GMSL2,
	MAX9X_LINK_TYPE_FPDLINK,
};

struct max9x_subdev_pdata {
	unsigned int serial_link_id; // DES identify GMSL link/pipe id
	struct i2c_board_info board_info;
	unsigned short phys_addr; // Remap or translate subdev

	struct gpiod_lookup *gpio; // SER only
};

struct max9x_serial_link_pdata {
	unsigned int link_id;
	enum max9x_serdes_link_type link_type;
	unsigned int rx_freq_mhz;
	unsigned int tx_freq_mhz;
	char poc_regulator[32];

	// SER
	struct i2c_client *des_client;
	unsigned int des_link_id;
};

struct max9x_serdes_mipi_map {
	u16 src_vc;
	u16 src_dt;
	u16 dst_vc;
	u16 dst_dt;
	u16 dst_csi;
	bool is_csi_enabled;
};

struct max9x_video_pipe_pdata {
	unsigned int serial_link_id; // DES: src-link, SER: dst-link
	unsigned int pipe_id; // X, Y, Z, U

	// DES
	struct max9x_serdes_mipi_map *maps;
	unsigned int num_maps;
	unsigned int src_pipe_id;

	// SER
	unsigned int src_csi_id;
	unsigned int *data_types;
	unsigned int num_data_types;
};

struct max9x_serdes_phy_map {
	u8 int_csi; //  Internal CSI lane
	u8 phy_ind; //  PHY index
	u8 phy_lane; // PHY lane index
};

struct max9x_csi_link_pdata {
	unsigned int link_id;
	enum v4l2_mbus_type bus_type;
	unsigned int num_lanes;
	struct max9x_serdes_phy_map *maps;
	unsigned int num_maps;
	unsigned int tx_rate_mbps;
	bool auto_initial_deskew;
	unsigned int initial_deskew_width;
	bool auto_start;
};

struct max9x_gpio_pdata {
	const char *label;
	const char *const *names;
};

struct max9x_pdata {
	unsigned short phys_addr; // Remap self
	char suffix[5];

	struct max9x_subdev_pdata *subdevs; // DES: the serializers, 1 per link; SER: sensor, eeprom, etc
	unsigned int num_subdevs;

	struct max9x_serial_link_pdata *serial_links; // DES only. SER is currently presumed to have only 1 active link
	unsigned int num_serial_links;

	struct max9x_video_pipe_pdata *video_pipes;
	unsigned int num_video_pipes;

	struct max9x_csi_link_pdata *csi_links;
	unsigned int num_csi_links;

	struct max9x_gpio_pdata gpio;

	bool external_refclk_enable;
};

#endif
