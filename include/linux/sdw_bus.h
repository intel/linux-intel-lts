/*
 *  sdw_bus.h - Definition for SoundWire bus interface.
 *
 * This header file refers to the MIPI SoundWire 1.0. The comments try to
 * follow the same conventions with a capital letter for all standard
 * definitions such as Master, Slave, Data Port, etc. When possible, the
 * constant numeric values are kept the same as in the MIPI specifications
 *
 *  Copyright (C) 2016 Intel Corp
 *  Author:  Hardik Shah  <hardik.t.shah@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
#ifndef _LINUX_SDW_BUS_H
#define _LINUX_SDW_BUS_H

#include <linux/device.h>	/* for struct device */
#include <linux/mod_devicetable.h> /* For Name size */
#include <linux/rtmutex.h> /* For rt mutex */


#define SOUNDWIRE_MAX_DEVICES 11

#define SDW_NUM_DEV_ID_REGISTERS 6

/* Port flow mode, used to indicate what port flow mode
 * slave supports
 */
#define SDW_PORT_FLOW_MODE_ISOCHRONOUS		0x1
#define SDW_PORT_FLOW_MODE_TX_CONTROLLED	0x2
#define SDW_PORT_FLOW_MODE_RX_CONTROLLED	0x4
#define SDW_PORT_FLOW_MODE_ASYNCHRONOUS		0x8

/* Bit-mask used to indicate Port capability, OR both bits if
 * Port is bidirectional capable
 */
#define SDW_PORT_SOURCE				0x1
#define SDW_PORT_SINK				0x2

/* Mask to specify what type of sample packaging mode
 * is supported by port
 */
#define	SDW_PORT_BLK_PKG_MODE_BLK_PER_PORT_MASK	0x1
#define	SDW_PORT_BLK_PKG_MODE_BLK_PER_CH_MASK 0x2

/* Mask to specify data encoding supported by port */
#define SDW_PORT_ENCODING_TYPE_TWOS_CMPLMNT	0x1
#define SDW_PORT_ENCODING_TYPE_SIGN_MAGNITUDE	0x2
#define SDW_PORT_ENCODING_TYPE_IEEE_32_FLOAT	0x4

/* enum sdw_driver_type: There are different driver callbacks for slave and
 *			master. This is to differentiate between slave driver
 *			and master driver. Bus driver binds master driver to
 *			master device and slave driver to slave device using
 *			this field. Driver populates this field based on whether
 *			its handling slave or master device.
 */
enum sdw_driver_type {
	SDW_DRIVER_TYPE_MASTER = 0,
	SDW_DRIVER_TYPE_SLAVE = 1,
};

/**
 * enum sdw_block_pkg_mode: Block packing mode for the port.
 * @SDW_PORT_BLK_PKG_MODE_BLK_PER_PORT: Block packing per port
 * @SDW_PORT_BLK_PKG_MODE_BLK_PER_CH: Block packing per channel.
 */
enum sdw_block_pkg_mode {
	SDW_PORT_BLK_PKG_MODE_BLK_PER_PORT = 0,
	SDW_PORT_BLK_PKG_MODE_BLK_PER_CH = 1,
};


/**
 * enum sdw_command_response: Data Port type
 * @SDW_COMMAND_OK: Command is Ok.
 * @SDW_COMMAND_IGNORED: Command is ignored.
 * @SDW_COMMAND_FAILED: Command failed.
 */
enum sdw_command_response {
	SDW_COMMAND_OK = 0,
	SDW_COMMAND_IGNORED = 1,
	SDW_COMMAND_FAILED = 2,
};
/**
 * enum sdw_dpn_type: Data Port type
 * @SDW_FULL_DP: Full Data Port supported.
 * @SDW_SIMPLIFIED_DP: Simplified Data Port. Following registers are not
 *			implemented by simplified data port
 *			DPN_SampleCtrl2, DPN_OffsetCtrl2, DPN_HCtrl and
 *			DPN_BlockCtrl3
 */
enum sdw_dpn_type {
	SDW_FULL_DP = 0,
	SDW_SIMPLIFIED_DP = 1,
};

/**
 * enum sdw_dpn_grouping: Maximum block group count supported.
 * @SDW_BLOCKGROUPCOUNT_1: Maximum Group count 1 supported.
 * @SDW_BLOCKGROUPCOUNT_2: Maximum Group count 2 supported.
 * @SDW_BLOCKGROUPCOUNT_3: Maximum Group count 3 supported.
 * @SDW_BLOCKGROUPCOUNT_4: Maximum Group count 4 supported.
 */
enum sdw_dpn_grouping {
	SDW_BLOCKGROUPCOUNT_1 = 0,
	SDW_BLOCKGROUPCOUNT_2 = 1,
	SDW_BLOCKGROUPCOUNT_3 = 2,
	SDW_BLOCKGROUPCOUNT_4 = 3,
};

/**
 * enum sdw_prep_ch_behavior: Specifies the dependencies between
 *				Channel Prepare sequence and bus
 *				clock configuration.This property is not
 *				required for ports implementing a
 *				Simplified ChannelPrepare State Machine (SCPSM)
 * @SDW_CH_PREP_ANY_TIME: Channel Prepare can happen at any bus clock rate
 * @SDW_CH_PREP_AFTER_BUS_CLK_CHANGE: : Channel Prepare sequence needs to
 *				happen after bus clock is changed to a
 *				frequency supported by this mode or
 *				compatible modes described by the next field.
 *				This may be required, e.g. when the Slave
 *				internal audio clocks are derived from the
 *				bus clock.
 */
enum sdw_prep_ch_behavior {
	SDW_CH_PREP_ANY_TIME = 0,
	SDW_CH_PREP_AFTER_BUS_CLK_CHANGE = 1,
};

/**
 * enum sdw_slave_status: Slave status reported in PING frames
 * @SDW_SLAVE_STAT_NOT_PRESENT: Slave is not present.
 * @SDW_SLAVE_STAT_ATTACHED_OK: Slave is Attached to the bus.
 * @SDW_SLAVE_STAT_ALERT: Some alert condition on the Slave.
 * @SDW_SLAVE_STAT_RESERVED: Reserved.
 */
enum sdw_slave_status {
	SDW_SLAVE_STAT_NOT_PRESENT = 0,
	SDW_SLAVE_STAT_ATTACHED_OK = 1,
	SDW_SLAVE_STAT_ALERT = 2,
	SDW_SLAVE_STAT_RESERVED = 3,
};

enum sdw_stream_type {
	SDW_STREAM_PCM = 0,
	SDW_STREAM_PDM = 1,
};


enum sdw_rt_state {
	SDW_RT_INITIALIZED = 0,
	SDW_RT_CONFIGURED = 1,
};

/**
 * enum sdw_ch_prepare_mode: Channel prepare mode.
 * @SDW_SIMPLIFIED_CP_SM: Simplified channel prepare.
 * @SDW_CP_SM: Normal channel prepare.
 */
enum sdw_ch_prepare_mode {
	SDW_SIMPLIFIED_CP_SM = 0,
	SDW_CP_SM = 1,
};

/**
 * enums dfw_clk_stop_prepare: Clock Stop prepare mode.
 * @SDW_CLOCK_STOP_MODE_0: Clock Stop mode 0
 * @SDW_CLOCK_STOP_MODE_1: Clock Stop mode 1
 */
enum sdw_clk_stop_mode {
	SDW_CLOCK_STOP_MODE_0 = 0,
	SDW_CLOCK_STOP_MODE_1 = 1,
};

/**
 *  enum sdw_data_direction: Data direction w.r.t Port. For e.g for playback
 *				between the Master and Slave, where Slave
 *				is codec, data direction for the Master
 *				port will be OUT, since its transmitting
 *				the data, while for the Slave (codec) it
 *				will be IN, since its receiving the data.
 *  @SDW_DATA_DIR_IN: Data is input to Port.
 *  @SDW_DATA_DIR_OUT: Data is output from Port.
 */
enum sdw_data_direction {
	SDW_DATA_DIR_IN = 0,
	SDW_DATA_DIR_OUT = 1,
};

/* Forward declaration of the data structures */
struct sdw_master;
struct sdw_slave;
struct sdw_msg;
struct sdw_bra_block;
struct sdw_mstr_driver;

#endif /*  _LINUX_SDW_BUS_H */
