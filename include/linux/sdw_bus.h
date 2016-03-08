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

/**
 * struct port_audio_mode_properties: Audio properties for the Port
 *
 * @max_frequency: Maximum frequency Port can support for the clock.
 *		The use of max_ and min_ frequency requires num_freq_config
 *		to be zero
 * @min_frequency: Minimum frequency Port can support for the clock.
 * @num_freq_configs: Array size for the frequencies supported by Port.
 * @freq_supported: Array of frequencies supported by the Port.
 * @glitchless_transitions_mask: Glitch transition mask from one mode to
 *				other mode. Each bit refers to a mode
 *				number.
 */

struct port_audio_mode_properties {
	unsigned int max_frequency;
	unsigned int min_frequency;
	unsigned int num_freq_configs;
	unsigned int *freq_supported;
	unsigned int max_sampling_frequency;
	unsigned int min_sampling_frequency;
	unsigned int num_sampling_freq_configs;
	unsigned int *sampling_freq_config;
	enum sdw_prep_ch_behavior ch_prepare_behavior;
	unsigned int glitchless_transitions_mask;
};

/**
 * struct sdw_slv_addr: Structure representing the device_id and
 *			and SoundWire logical Slave address.
 * @dev_id: 6-byte device id of the Slave
 * @slv_number: Logical SoundWire Slave number, in the range [1..11]
 * @assigned: Logical address is assigned to some Slave or not
 * @status: What is the current state of the slave.
 *
 */
struct sdw_slv_addr {
	struct sdw_slave *slave;
	u8 dev_id[SDW_NUM_DEV_ID_REGISTERS];
	u8 slv_number;
	bool assigned;
	enum sdw_slave_status status;
};

/**
 * struct sdw_slv_dpn_capabilities: Capabilities of the Data Port, other than
 *				Data Port 0 for SoundWire Slave
 * @port_direction: Direction of the Port. Sink or Source or bidirectional.
 *			Set appropriate bit mak based on port capabilities.
 * @port_number: Port number.
 * @max_word_length: Maximum length of the sample word.
 * @min_word_length: Minimum length of sample word.
 * @num_word_length: Length of supported word length buffer.
 *		The use of max_ and min_ word length requires
 *		num_word_length to be zero
 * @word_length_buffer: Array of the supported word length.
 * @dpn_type: Type of Data Port. Simplified or Normal data port.
 * @dpn_grouping: Max Block group count supported for this Port.
 * @prepare_ch: Channel prepare scheme. Simplified channel prepare or Normal
 *		channel prepare.
 * @imp_def_intr_mask: Implementation defined interrupt mask.
 * @min_ch_num: Minimum number of channels supported.
 * @max_ch_num: Maximum number of channels supported.
 * @num_ch_supported: Buffer length for the channels supported.
 *			The use of max_ and min_ ch_num requires
 *			num_ch_supported to be zero
 * @ch_supported: Array of the channel supported.
 * @port_flow_mode_mask: Transport flow modes supported by Port.
 * @block_packing_mode_mask: Block packing mode mask.
 * @port_encoding_type_mask: Port Data encoding type mask.
 * @num_audio_modes: Number of audio modes supported by device.
 * @mode_properties: Port audio mode properties buffer of size num_audio_modes
 */

struct sdw_slv_dpn_capabilities {
	unsigned int port_direction;
	unsigned int port_number;
	unsigned int max_word_length;
	unsigned int min_word_length;
	unsigned int num_word_length;
	unsigned int *word_length_buffer;
	enum sdw_dpn_type dpn_type;
	enum sdw_dpn_grouping dpn_grouping;
	enum sdw_ch_prepare_mode prepare_ch;
	unsigned int imp_def_intr_mask;
	unsigned int min_ch_num;
	unsigned int max_ch_num;
	unsigned int num_ch_supported;
	unsigned int *ch_supported;
	unsigned int port_flow_mode_mask;
	unsigned int block_packing_mode_mask;
	unsigned int port_encoding_type_mask;
	unsigned int num_audio_modes;
	struct port_audio_mode_properties *mode_properties;
};

/**
 *  struct sdw_slv_bra_capabilities: BRA Capabilities of the Slave.
 *  @max_bus_frequency: Maximum bus frequency of this mode, in Hz
 *  @min_bus_frequency: Minimum bus frequency of this mode, in Hz
 *		When using min-max properties, all values in the defined
 *		range are allowed. Use the config list in the next field
 *		if only discrete values are supported.
 *  @num_bus_config_frequency:  Number of discrete bus frequency configurations
 *  @bus_config_frequencies: Array of bus frequency configs.
 *  @max_data_per_frame: Maximum Data payload, in bytes per frame.
 *		Excludes header, CRC, footer. Maximum value is 470
 *  @min_us_between_transactions: Amount of delay, in microseconds,
 *		required to be inserted between BRA transactions.
 *		Use if Slave needs idle time between BRA transactions.
 *  @max_bandwidth: Maximum bandwidth (in bytes/s) that can be written/read
 *		(header, CRCs, footer excluded)
 *  @mode_block_alignment: Size of basic block in bytes. The Data payload
 *		size needs to be a multiple of this basic block and
 *		padding/repeating of the same value is required for
 *		transactions smaller than this basic block.
 */

struct sdw_slv_bra_capabilities {
	unsigned int max_bus_frequency;
	unsigned int min_bus_frequency;
	unsigned int num_bus_config_frequency;
	unsigned int *bus_config_frequencies;
	unsigned int max_data_per_frame;
	unsigned int min_us_between_transactions;
	unsigned int max_bandwidth;
	unsigned int mode_block_alignment;
};

/**
 * struct sdw_slv_dp0_capabilities: Capabilities of the Data Port 0 of Slave.
 *
 * @max_word_length: Maximum word length supported by the Data Port.
 * @min_word_length: Minimum word length supported by the Data Port.
 * @num_word_length: Array size of the buffer containing the supported
 *			word lengths.
 *			The use of max_ and min_ word length requires
 *			num_word_length to be zero
 * @word_length_buffer: Array containing supported word length.
 * @bra_use_flow_control: Flow control is required or not for bra block
 *			transfer.
 * @bra_initiator_supported: Can Slave be BRA initiator.
 * @ch_prepare_mode: Type of channel prepare scheme. Simplified or Normal
 *			channel prepare.
 * @impl_def_response_supported;: If True (nonzero), implementation-defined
 *			response is supported. This information may be used
 *			by a device driver to request that a generic bus
 *			driver forwards the response to the client device
 *			driver.
 * @imp_def_intr_mask: Implementation defined interrupt mask for DP0 Port.
 * @impl_def_bpt_supported: If True (nonzero), implementation-defined
 *			Payload Type is supported. This information is used
 *			to bypass the BRA protocol and may only be of
 *			interest when a device driver is aware of the
 *			Capabilities of the Master controller and Slave
 *			devices.
 * @slave_bra_cap: BRA capabilities of the Slave.
 */

struct sdw_slv_dp0_capabilities {
	unsigned int max_word_length;
	unsigned int min_word_length;
	unsigned int num_word_length;
	unsigned int *word_length_buffer;
	unsigned int bra_use_flow_control;
	bool bra_initiator_supported;
	enum sdw_ch_prepare_mode ch_prepare_mode;
	bool impl_def_response_supported;
	unsigned int imp_def_intr_mask;
	bool impl_def_bpt_supported;
	struct sdw_slv_bra_capabilities slave_bra_cap;
};

/** struct sdw_slv_capabilities: Capabilities of the SoundWire Slave. This
 *				is public structure for slave drivers to
 *				updated its capability to bus driver.
 *
 * @wake_up_unavailable: Slave is capable of waking up the Master.
 * @test_mode_supported: Slave supports test modes.
 * @clock_stop1_mode_supported: Clock stop 1 mode supported by this Slave.
 * @simplified_clock_stop_prepare: Simplified clock stop prepare
 *				supported.
 * @highphy_capable: Slave is highphy_capable or not?
 * @paging_supported: Paging registers supported for Slave?
 * @bank_delay_support: Bank switching delay for Slave
 * @port_15_read_behavior: Slave behavior when the Master attempts a Read to
 *			the Port15 alias
 *			0: Command_Ignored
 *			1: Command_OK, Data is OR of all registers
 * @sdw_dp0_supported: DP0 is supported by Slave.
 * @sdw_dp0_cap: Data Port 0 Capabilities of the Slave.
 * @num_of_sdw_ports: Number of SoundWire Data ports present. The representation
 *			assumes contiguous Port numbers starting at 1.
 * @sdw_dpn_cap: Capabilities of the SoundWire Slave ports.
 */

struct sdw_slv_capabilities {
	bool wake_up_unavailable;
	bool test_mode_supported;
	bool clock_stop1_mode_supported;
	bool simplified_clock_stop_prepare;
	bool highphy_capable;
	bool paging_supported;
	bool bank_delay_support;
	unsigned int port_15_read_behavior;
	bool sdw_dp0_supported;
	struct sdw_slv_dp0_capabilities *sdw_dp0_cap;
	int num_of_sdw_ports;
	struct sdw_slv_dpn_capabilities *sdw_dpn_cap;
};


/**
 * struct sdw_slave: Represents SoundWire Slave device
 *				(similar to 'i2c_client' on I2C)
 *		This is not public structure. Maintained by
 *		bus driver internally.
 * @dev: Driver model representation of the device
 * @slave_cap_updated: Did slave device driver updated slave capabilties
 *			to bus.
 * @name: Name of the driver to use with the device.
 * @dev_id: 6-byte unique device identification.
 * @driver: Slave's driver, pointer to access routine.
 * @mstr: SoundWire Master, managing the bus on which this Slave is
 * @slv_number: Logical address of the Slave, assigned by bus driver
 * @node: Node to add the Slave to the list of Slave devices managed
 *		by same Master.
 * @port_ready: Port ready completion flag for each Port of the Slave;
 * @sdw_slv_cap: Slave Capabilities.
 */
struct sdw_slave {
	struct device		dev;
	bool			slave_cap_updated;
	char			name[SOUNDWIRE_NAME_SIZE];
	u8			dev_id[6];
	struct sdw_slv_addr	*slv_addr;
	struct sdw_slave_driver	*driver;
	struct sdw_master	*mstr;
	u8			slv_number;
	struct list_head	node;
	struct completion	*port_ready;
	struct sdw_slv_capabilities sdw_slv_cap;
};
#define to_sdw_slave(d) container_of(d, struct sdw_slave, dev)

/**
 *  struct sdw_bus_params: Bus params for the Slave to be ready for next
 *  bus changes.
 *  @num_rows: Number of rows in new frame to be effective.
 *  @num_cols: Number of columns in new frame to be effective.
 *  @bus_clk_freq: Clock frequency for the bus.
 *  @bank: Register bank, which Slave driver should program for
 *			implementation define Slave registers. This is the
 *			inverted value of the current bank.
 */

struct sdw_bus_params {
	int num_rows;
	int num_cols;
	int bus_clk_freq;
	int bank;
};

/**
 * struct sdw_slave_driver: Manage SoundWire generic/Slave device driver
 * @driver_type: To distinguish between master and slave driver. Set and
 *		used by bus driver.
 * @probe: Binds this driver to a SoundWire Slave.
 * @remove: Unbinds this driver from the SoundWire Slave.
 * @shutdown: Standard shutdown callback used during powerdown/halt.
 * @suspend: Standard suspend callback used during system suspend
 * @resume: Standard resume callback used during system resume
 * @driver: Generic driver structure, according to driver model.
 * @handle_impl_def_interrupts: Slave driver callback, for status of the
 *			Slave other than "REPORT_PRESENT". There may be
 *			jack detect, pll locked kind of status update
 *			interrupt required by Slave, which Slave need to
 *			handle in impl_defined way, using implementation
 *			defined interrupts. This is callback function to
 *			Slave to handle implementation defined interrupts.
 * @handle_bus_changes: Slave callback function to let Slave configure
 *			implementation defined registers prior to any bus
 *			configuration changes. Bus configuration changes
 *			will be signaled by a bank switch initiated by the bus
 *			driver once all Slaves drivers have performed their
 *			imp-def configuration sequence (if any).
 *			If this callback is not implemented the bus driver
 *			will assume the Slave can tolerate bus configurations
 *			changes at any time.
 *
 * @handle_pre_port_prepare: Slave driver callback to allow Slave Port to be
 *				prepared by configuring impl defined register
 *				as part of Port prepare state machine.
 *				This fn is called before DPn_Prepare ctrl is
 *				written. Before this function is
 *				called Port state is un-prepared (CP_Stopped).
 *				This is optional based on any impl
 *				defined register needs to be set by Slave
 *				driver before Port is prepared.
 * @handle_post_port_prepare: Slave driver callback to allow Slave Port to be
 *				prepared by configuring impl defined register
 *				as part of Port prepare state machine.
 *				This is called after DPn_Prepare
 *				ctrl is written, and DPn_status reports as
 *				Port prepared(CP_Ready). This is optional
 *				based on any impl defined register needs to
 *				be set by Slave driver once Port is ready.
 * @handle_pre_port_unprepare: Slave driver callback to allow Slave Port to be
 *				un-prepared by configuring impl defined register
 *				as part of Port un-prepare state machine.
 *				This is called before DPn_Prepare ctrl is
 *				written. Before this function is called
 *				Port state is ready (CP_Ready).
 *				This is optional based on any impl
 *				defined register needs to be set by Slave
 *				driver before Port is un-prepared.
 * @handle_post_port_unprepare: Slave driver callback to allow Slave Port to be
 *				un-prepared by configuring impl defined register
 *				as part of Port prepare state machine.
 *				This is called after DPn_Prepare
 *				ctrl is written, and DPn_status reports as
 *				Port un-prepared (CP_Stopped).
 *				This is optional based on any impl defined
 *				register needs to be set by Slave driver once
 *				Port is un-prepared.
 *
 * @id_table: List of SoundWire Slaves supported by this driver
 */
struct sdw_slave_driver {
	enum sdw_driver_type driver_type;
	int (*probe)(struct sdw_slave *swdev, const struct sdw_slave_id *);
	int (*remove)(struct sdw_slave *swdev);
	void (*shutdown)(struct sdw_slave *swdev);
	int (*suspend)(struct sdw_slave *swdev,
		pm_message_t pmesg);
	int (*resume)(struct sdw_slave *swdev);
	struct device_driver driver;
	int (*handle_impl_def_interrupts)(struct sdw_slave *swdev,
		unsigned int intr_status_mask);
	int (*handle_bus_changes)(struct sdw_slave *swdev,
			struct sdw_bus_params *params);
	int (*handle_pre_port_prepare)(struct sdw_slave *swdev,
			int port, int ch_mask, int bank);
	int (*handle_post_port_prepare)(struct sdw_slave *swdev,
			int port, int ch_mask, int bank);
	int (*handle_pre_port_unprepare)(struct sdw_slave *swdev,
			int port, int ch_mask, int bank);
	int (*handle_post_port_unprepare)(struct sdw_slave *swdev,
			int port, int ch_mask, int bank);
	const struct sdw_slave_id *id_table;
};
#define to_sdw_slave_driver(d) container_of(d, struct sdw_slave_driver, driver)

#endif /*  _LINUX_SDW_BUS_H */
