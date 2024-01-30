/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2020 - 2023 Intel Corporation.
 */

#ifndef IAF_DRV_H_INCLUDED
#define IAF_DRV_H_INCLUDED

#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
#include <linux/auxiliary_bus.h>
#else
#include <linux/platform_device.h>
#endif
#include <linux/dcache.h>
#include <linux/irqreturn.h>
#include <linux/mtd/mtd.h>
#include <linux/rwsem.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/kref.h>

#include <drm/i915_mei_iaf_interface.h>
#include <drm/intel_iaf_platform.h>

#include "csr.h"

#include "statedump.h"

#define DRIVER_NAME "iaf"

#undef pr_fmt
#define pr_fmt(fmt) DRIVER_NAME ": " fmt

#define MAILBOX_OPCODE_COUNT 256

/*
 * The maximum number of tiles for the PVC product type.
 *
 * Revisit if future product types or variations are developed that require
 * a product type table.
 */
#define IAF_MAX_SUB_DEVS 2

/*
 * The maximum characters kept from the PSCBIN "version" information
 */
#define MAX_PSCBIN_VERSION 64

/*
 * Currently, ports are numbered as follows (port numbers come from PSCBIN/INIBIN data):
 * 0 : reserved/CPORT
 * 1-8 : fabric ports
 * 9-11 : bridge ports
 *
 * The driver uses sd->fport_lpns and sd->bport_lpns to avoid hardcoding the numbering scheme,
 * although some external interfaces such as the TX calibration blob rely on it.
 */

/* current device/firmware allows only 3 bridge ports, historically up to 4 were supported */
#define PORT_PHYSICAL_START	(1)
#define PORT_FABRIC_COUNT	(8)
#define PORT_BRIDGE_COUNT	(4)
#define PORT_COUNT		(PORT_PHYSICAL_START + PORT_FABRIC_COUNT + PORT_BRIDGE_COUNT)

/* This limits the range of fabric port numbers to [PORT_PHYSICAL_START..TXCAL_PORT_COUNT]. */
#define TXCAL_PORT_COUNT	(8)

/* prevent overruns when calibration data is copied into sd->txcal[] */
#if TXCAL_PORT_COUNT + PORT_PHYSICAL_START >= PORT_COUNT
#error TXCAL_PORT_COUNT cannot exceed largest port number
#endif

#define CPORT_LPN_MASK BIT(0)

/*
 * Recognized discrete socket IDs
 */
#define MAX_SOCKET_IDS (32)

/*
 * Platforms and revisions.
 *
 * Note that these conform to the ANR ASIC_REV_INFO (ARI) register values,
 * not the PCI revision IDs reported by the packaged product.
 */

#define ANR_ARI_PLATFORM	0x0101

#define ANR_ARI_STEP_A0		0x00
#define ANR_ARI_STEP_A1		0x01
#define ANR_ARI_STEP_A_LAST	ANR_ARI_STEP_A1
#define ANR_ARI_STEP_B0		0x10
#define ANR_ARI_STEP_B_LAST	ANR_ARI_STEP_B0

#define IS_ANR(sd) \
	(FIELD_GET(MASK_ARI_PLATFORM, (sd)->asic_rev_info) == ANR_ARI_PLATFORM)

#define IS_ANR_STEP(sd, since, until) (IS_ANR(sd) && \
	FIELD_GET(MASK_ARI_STEP, (sd)->asic_rev_info) >= (since) && \
	FIELD_GET(MASK_ARI_STEP, (sd)->asic_rev_info) <= (until))

/*
 * Device, subdevice and port message formats
 *
 * Device expands dev->name, sd/port expand relevant indices
 * ": " separates info. *_FMT ends in ": ", *_ID_FMT does not
 */

#define DEV_ID_FMT "iaf.%d"
#define SD_ID_FMT "sd.%d"
#define PORT_ID_FMT "p.%d"

#define DEV_FMT DEV_ID_FMT ": "
#define SD_FMT SD_ID_FMT ": "
#define PORT_FMT PORT_ID_FMT ": "

#define DEV_SD_FMT DEV_FMT "" SD_FMT
#define SD_PORT_FMT SD_FMT "" PORT_FMT

#define DEV_SD_PORT_FMT DEV_FMT "" SD_PORT_FMT

/*
 * Subdevice-specific messaging
 */

#define sd_emerg(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_emerg(sd_dev(_sd), SD_FMT _fmt, \
				  sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_alert(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_alert(sd_dev(_sd), SD_FMT _fmt, \
				  sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_crit(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_crit(sd_dev(_sd), SD_FMT _fmt, \
				 sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_err(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_err(sd_dev(_sd), SD_FMT _fmt, \
				sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_warn(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_warn(sd_dev(_sd), SD_FMT _fmt, \
				 sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_notice(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_notice(sd_dev(_sd), SD_FMT _fmt, \
				   sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_info(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_info(sd_dev(_sd), SD_FMT _fmt, \
				 sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

#define sd_dbg(__sd, _fmt, ...) \
		do { \
			struct fsubdev *_sd = (__sd); \
			dev_dbg(sd_dev(_sd), SD_FMT _fmt, \
				sd_index(_sd), ##__VA_ARGS__); \
		} while (0)

/*
 * Port-specific messaging
 */

#define fport_emerg(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_emerg(fport_dev(_p), SD_PORT_FMT _fmt, \
				  sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_alert(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_alert(fport_dev(_p), SD_PORT_FMT _fmt, \
				  sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_crit(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_crit(fport_dev(_p), SD_PORT_FMT _fmt, \
				 sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_err(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_err(fport_dev(_p), SD_PORT_FMT _fmt, \
				sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_warn(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_warn(fport_dev(_p), SD_PORT_FMT _fmt, \
				 sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_notice(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_notice(fport_dev(_p), SD_PORT_FMT _fmt, \
				   sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_info(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_info(fport_dev(_p), SD_PORT_FMT _fmt, \
				 sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

#define fport_dbg(__p, _fmt, ...) \
		do { \
			struct fport *_p = (__p); \
			dev_dbg(fport_dev(_p), SD_PORT_FMT _fmt, \
				sd_index(_p->sd), _p->lpn, ##__VA_ARGS__); \
		} while (0)

/**
 * struct mbdb_op_fw_version_rsp - currently loaded firmware information
 * @mbox_version: this version will be 0
 * @environment: 0 = bootloader, 1 = run-time firmware
 * @fw_version_string: ASCII-NUL terminated version string. e.g "w.x.y.z"
 * @supported_opcodes: bit mask of opcodes that the environment supports as requests from the host
 */
struct mbdb_op_fw_version_rsp {
	u8 mbox_version;
	u8 environment;
	u8 fw_version_string[22];
	DECLARE_BITMAP(supported_opcodes, MAILBOX_OPCODE_COUNT);
} __packed;

/**
 * struct mbdb_op_switchinfo - Per-IAF subdevice information
 * @guid: RO, from FUSES
 * @num_ports: RO, fixed, 8, 12, or 13 (TBD)
 * @slt_psc_ep0: RW, bit 0-4: max in-switch packet lifetime
 *		 RO, bit 5: redundant to PSC notification
 *		 RO, bit 6: Q7 presence?
 * @routing_mode_supported: RO, hierarchical/linear routing enumeration
 * @routing_mode_enabled: RW, hierarchical/linear routing enumeration
 * @lft_top: RW, max FID value
 * @unaligned_cnt: unaligned error count
 * @unaligned_portwin: contents of port window csr - port being accessed
 * @unaligned_vaddr: address being accessed
 * @unaligned_pc: program counter
 * @pif_data_error_cnt: load store data error count
 * @pif_data_error_portwin: contents of port window csr - port being accessed
 * @pif_data_error_vaddr: address being accessed
 * @pif_data_error_pc: program counter
 * @pif_addr_error_cnt: load store address error count
 * @pif_addr_error_portwin: contents of port window csr - port being accessed
 * @pif_addr_error_vaddr: address being accessed
 * @pif_addr_error_pc: program counter
 * @mem_scrub_size: amount of memory to scrub per pass in dwords; 0 to disable memory scrubbing
 * @mem_scrub_count: number of times we have scrubbed memory
 * @vff_scrub_disable: disable VFF scrubbing
 * @vff_scrub_count: number of times we have scrubbed CSRs list
 * @rpipe_scrub_disable: disable Routing Tables scrubbing
 * @rpipe_scrub_count: number of times we have scrubbed URT tables
 * @mrt_scrub_count: number of times we have scrubbed MRT tables
 * @fouropsel_scrub_count: number of times we have scrubbed 4opsel tables
 *
 * Used with ops_switchinfo_set() and ops_switchinfo_get() to access
 * switch information for the entire subdevice. A copy is maintained in
 * &struct fsubdev.
 */
struct mbdb_op_switchinfo {
	u64 guid;
	u8 num_ports;
#define SLT_PSC_EP0_SWITCH_LIFETIME	GENMASK(4, 0)
#define SLT_PSC_EP0_PORT_STATE_CHANGE	BIT(5)
#define SLT_PSC_EP0_ENHANCED_PORT_0	BIT(6)
	u8 slt_psc_ep0;
	u8 routing_mode_supported;
	u8 routing_mode_enabled;
	u32 lft_top;
	u32 unaligned_cnt;
	u32 unaligned_portwin;
	u32 unaligned_vaddr;
	u32 unaligned_pc;
	u32 pif_data_error_cnt;
	u32 pif_data_error_portwin;
	u32 pif_data_error_vaddr;
	u32 pif_data_error_pc;
	u32 pif_addr_error_cnt;
	u32 pif_addr_error_portwin;
	u32 pif_addr_error_vaddr;
	u32 pif_addr_error_pc;
	u32 mem_scrub_size;
	u32 mem_scrub_count;
	u32 vff_scrub_disable;
	u32 vff_scrub_count;
	u32 rpipe_scrub_disable;
	u32 rpipe_scrub_count;
	u32 mrt_scrub_count;
	u32 fouropsel_scrub_count;
} __packed;

/**
 * struct portinfo - per-port information
 * @fid: RW, port Fabric ID set by routing logic
 * @link_down_count: RO, counter (saturating?, clearable), check at PSC
 * @neighbor_guid: RO, from neighbor, check valid at link up
 * @port_error_action: RW, policy bitmask, which errors cause port to bounce
 * @neighbor_port_number: RO, from neighbor, check valid at link up
 * @port_type: RO, disabled/variable/etc.
 * @port_link_mode_active: RO, enumeration, fabric (STL)/bridge port (FLIT BUS)
 * @neighbor_link_down_reason: RO, link_down_reason from neighbor, check when
 *			       working link goes down
 * @h_o_q_lifetime: RW, head of queue lifetime
 * @vl_cap: RO, max VL supported
 * @operational_vls: RW, how many VLs active, write once, routing sets??
 * @neighbor_mtu: RW, neighbor MTU, routing sets (default should be OK)
 * @crc_mode_supported: RO, bitfield of modes supported
 * @crc_mode_enabled: RW, bitfield of driver requested modes
 * @crc_mode_active: RO, 1-hot bitfield of mode in use, valid after link-up
 * @reserved1: reserved
 * @link_width_supported: RO, bitfield of initial link width supported
 * @link_width_enabled: RW, bitfield of driver-requested initial link width
 * @link_width_active: RO, 1-hot bitfield of link width, valid after link-up
 * @reserved2: reserved
 * @link_speed_supported: RO, bitfield of link speed supported
 * @link_speed_enabled: RW, bitfield of driver-requested link speed
 * @link_speed_active: RO, 1-hot bitfield of link speed in use
 * @reserved3: reserved
 * @link_width_downgrade_rx_active: RO, current active downgraded RX link
 *				    width (format TBD)
 * @link_width_downgrade_tx_active: RO, current active downgraded TX link
 *				    width (format TBD)
 * @link_init_reason: RW?, ?
 * @link_down_reason: RW, why shutting link down, enumeration, set before link
 *		      brought down (can be overwritten by FW)
 * @port_state_port_physical_state: RW, bit 0-3: port logical state (look for
 *				    transition to INIT, set CPORT to Down
 *				    to bounce)
 *				    RW, bit 4-7: port physical state (set to
 *				    POLLING to start training process)
 * @oldr_nn_lqi: RO?, bit 0-3: bitfield/enumeration (TBD) why disabling a link
 *		 RO, bit 4: whether neighbor is armed, used by routing
 *		 RO, bit 5-7: 0-5 link quality, degraded if less than 4
 * @reserved4: reserved
 * @lqi_change_count: RO, number of times link quality has changed
 * @reserved5: reserved for future expansion with mailbox
 * @reserved6: reserved for future expansion with mailbox
 * @???: RW, policy config for max allowable downgrade
 *
 * Included in &struct mbdb_op_portinfo.
 */
struct portinfo {
	u32 fid;
	u32 link_down_count;
	u64 neighbor_guid;
	u32 port_error_action;
	u8 neighbor_port_number;
	u8 port_type;
	u8 port_link_mode_active;
	u8 neighbor_link_down_reason;
	u8 h_o_q_lifetime;
	u8 vl_cap;
	u8 operational_vls;
	u8 neighbor_mtu;
	u8 crc_mode_supported;
	u8 crc_mode_enabled;
	u8 crc_mode_active;
	u8 reserved1;
	u8 link_width_supported;
	u8 link_width_enabled;
	u8 link_width_active;
	u8 reserved2;
	u8 link_speed_supported;
	u8 link_speed_enabled;
	u8 link_speed_active;
	u8 reserved3;
	u8 link_width_downgrade_rx_active;
	u8 link_width_downgrade_tx_active;
	u8 link_init_reason;
	u8 link_down_reason;
#define PS_PPS_PORT_STATE	GENMASK(3, 0)
#define PS_PPS_PHYSICAL_STATE	GENMASK(7, 4)
	u8 port_state_port_physical_state;
#define OLDR_NN_LQI_OFFLINE_DISABLED_REASON	GENMASK(3, 0)
#define OLDR_NN_LQI_NEIGHBOR_NORMAL		BIT(4)
#define OLDR_NN_LQI_LINK_QUALITY_INDICATOR	GENMASK(7, 5)
	u8 oldr_nn_lqi;
	u16 reserved4;
	u32 lqi_change_count;
	u32 reserved5;
	u64 reserved6;
} __packed;

/*
 * if FIELD_GET(OLDR_NN_LQI_LINK_QUALITY_INDICATOR, oldr_nn_lqi) is below
 * this threshold, the link is in poor health
 */
#define LQI_HEALTHY_THRESHOLD (4)

/*
 * These definitions are from the firmware
 */

#define IAF_FW_PORT_TYPE_UNKNOWN (0)
#define IAF_FW_PORT_TYPE_DISCONNECTED (1)
#define IAF_FW_PORT_TYPE_FIXED (2)
#define IAF_FW_PORT_TYPE_VARIABLE (3)
#define IAF_FW_PORT_TYPE_STANDARD (4)
#define IAF_FW_PORT_TYPE_SI_PHOTONICS (5)

#define IAF_FW_PORT_NOP (0)
#define IAF_FW_PORT_DOWN (1)
#define IAF_FW_PORT_INIT (2)
#define IAF_FW_PORT_ARMED (3)
#define IAF_FW_PORT_ACTIVE (4)

#define IAF_FW_PORT_PHYS_NOP (0)
#define IAF_FW_PORT_PHYS_POLLING (2)
#define IAF_FW_PORT_PHYS_DISABLED (3)
#define IAF_FW_PORT_PHYS_TRAINING (4)
#define IAF_FW_PORT_PHYS_LINKUP (5)
#define IAF_FW_PORT_PHYS_LINK_ERROR_RECOVERY (6)
#define IAF_FW_PORT_PHYS_OFFLINE (9)
#define IAF_FW_PORT_PHYS_TEST (11)

#define IAF_FW_PORT_LINK_MODE_NOP (0)
#define IAF_FW_PORT_LINK_MODE_FLIT_BUS (2)
#define IAF_FW_PORT_LINK_MODE_FABRIC (4)

enum pm_port_state {
	PM_PORT_STATE_INACTIVE,
	PM_PORT_STATE_REQUESTED,
	PM_PORT_STATE_ISOLATED,
	PM_PORT_STATE_ARMED,
	PM_PORT_STATE_ACTIVE
};

#define LINK_SPEED_12G 1
#define LINK_SPEED_25G 2
#define LINK_SPEED_53G 4
#define LINK_SPEED_90G 8

#define LINK_SPEED_SLOW	(0x07)
#define LINK_SPEED_FAST	(0x18)

#define BPS_LINK_SPEED_12G  12500000000ULL
#define BPS_LINK_SPEED_25G  25781250000ULL
#define BPS_LINK_SPEED_53G  53125000000ULL
#define BPS_LINK_SPEED_90G  90000000000ULL
#define BPS_LINK_SPEED_106G 106000000000ULL

#define LINK_WIDTH_1X 1
#define LINK_WIDTH_2X 2
#define LINK_WIDTH_3X 4
#define LINK_WIDTH_4X 8

#define LANES 4

enum iaf_startup_mode {
	STARTUP_MODE_DEFAULT = 0,
	STARTUP_MODE_PRELOAD = 1,
#if IS_ENABLED(CONFIG_IAF_DEBUG_STARTUP)
	STARTUP_MODE_DEBUG   = 2,
	STARTUP_MODE_FWDEBUG = 3,
#endif
};

struct fsubdev; /* from this file */

/**
 * enum PORT_CONTROL - control port behavior
 *
 * @PORT_CONTROL_ENABLED: port enabled for PM to set up
 * @PORT_CONTROL_ROUTABLE: port enabled for routing
 * @PORT_CONTROL_BEACONING: beaconing requested (flash LEDs)
 * @NUM_PORT_CONTROLS: number of controls (always last)
 *
 * Each maps to a bit accessed atomically in &struct fport->controls
 */
enum PORT_CONTROL {
	PORT_CONTROL_ENABLED,
	PORT_CONTROL_ROUTABLE,
	PORT_CONTROL_BEACONING,
	NUM_PORT_CONTROLS
};

/*
 * fport_routing - routing-specific port data
 * @routable: true if the port meets the port_is_routable condition
 * @neighbor: reference to neighbor port
 *
 * These values are saved under the exclusive routing lock, remain
 * valid under a downgrade to shared, and are invalided on unlock.
 */
struct fport_routing {
	bool routable;
	struct fport *neighbor;
};

/**
 * struct fport - Per-port state, used for fabric ports only
 * @sd: link to containing subdevice
 * @portinfo: link to relevant &struct fsubdev.portinfo_op.per_portinfo
 * @kobj: kobject for this port's sysfs directory node
 * @link_failures: attribute for link_failures sysfs file
 * @link_degrades: attribute for link_degrades sysfs file
 * @lpn: logical port number in firmware
 * @port_type: type of port (hardwired, QFSP, etc.)
 * @lanes_port: for each lane the port to which it belongs
 * @log_state: firmware logical state (DOWN, INIT, ARMED, ACTIVE)
 * @phys_state: firmware physical state (DISABLED, POLLING, ..., LINKUP)
 * @state: driver-abstracted high level view of port state
 * @bounce_count: number of bounces in current flap-check leaky bucket period for the most recent
 *		  successful sweep
 * @flap_check_since: timestamp indicating beginning of current flap-check leaky bucket period
 * @linkup_timer: to verify port link up timeliness
 * @routing: routing-specific data
 * @unroute_link: Entry into the list of ports that are waiting to be unrouted.
 * @controls: atomically-accessed control bits to enable/disable features
 * @routed: indicates whether this port was included in the routing logic
 *
 * Used throughout the driver (mostly by the port manager and routing engine) to maintain
 * information about a given port.
 *
 * Protection mechanisms used outside of init/destroy are documented inline. A single PM thread per
 * tile manages port-related data for that tile: per-tile data written by PM is written by a single
 * thread and not accessed by any other PM threads.
 */
struct fport {
	/* pointers const after async probe, content protection is by object type */
	struct fsubdev *sd;
	struct portinfo *portinfo;
	struct kobject *kobj;
	struct device_attribute link_failures;
	struct device_attribute link_degrades;

	/* values const after async probe */
	u8 lpn;
	u8 port_type;
	struct fport *lanes_port[LANES];

	/* protected by routable_lock, written by PM thread with shared lock */
	u8 log_state;
	u8 phys_state;
	enum pm_port_state state;

	/* private to PM, written by PM thread with shared routable_lock */
	u16 bounce_count;
	s64 flap_check_since;
	struct timer_list linkup_timer;

	/* protected by routable_lock, written by routing with exclusive lock */
	struct fport_routing routing;

	/* protected by routable_lock, written by routing or parent device with exclusive lock */
	struct list_head unroute_link;

	/* atomic with no need for additional barrier constraints */
	DECLARE_BITMAP(controls, NUM_PORT_CONTROLS);
	atomic_t routed;
};

/**
 * struct mbdb_op_portinfo - Consolidation of per-port information
 * @port_mask: bitmask of logical ports, 0=CPORT, 1-12=fabric/bridge (TBD)
 * @reserved: 4 byte pad
 * @per_portinfo: one &struct portinfo entry for each bit set in
 *		  &mbdb_op_portinfo.port_mask
 *
 * Used with ops_portinfo_set() and ops_portinfo_get() to access
 * information about sets of ports on a subdevice. A copy is maintained in
 * &struct fsubdev.
 */
struct mbdb_op_portinfo {
	u32 port_mask;
	u32 reserved;
	struct portinfo per_portinfo[];
} __packed;

/*
 * Use this to declare an instance of mbdb_op_portinfo sized to a given number
 * of ports
 */
#define DECLARE_MBDB_OP_PORTINFO(_p, _n) \
	union { \
		struct mbdb_op_portinfo _p; \
		struct { \
			u32 _port_mask; \
			u32 _reserved; \
			struct portinfo _per_portinfo[_n]; \
		} __packed; \
	}

struct mbdb; /* from mbdb.c */
struct fdev; /* from this file */

struct routing_topology; /* from routing_topology.h */
struct routing_plane; /* from routing_topology.h */
struct routing_dfid_map; /* from routing_topology.h */
struct routing_uft; /* from routing_topology.h */

/**
 * enum pm_trigger_reasons - Cause for PM thread trigger
 * @INIT_EVENT: Initialization of PM subsystem for this sd
 * @DEISOLATE_EVENT: Request to deisolate all ports (possible fabric change)
 * @PSC_TRAP: Port state change reported
 * @LWD_TRAP: Link width degrade reported
 * @LQI_TRAP: Link quality change reported
 * @QSFP_PRESENCE_TRAP: QSFP presence change reported
 * @QSFP_FAULT_TRAP: QSFP fault reported
 * @RESCAN_EVENT: PM-initiated trigger to rescan ports
 * @NL_PM_CMD_EVENT: Netlink request affecting PM processed
 * @NUM_PM_TRIGGERS: Number of trigger reasons (always last)
 *
 * Used as bit index into &fsubdev.pm_triggers
 */
enum pm_trigger_reasons {
	INIT_EVENT,
	DEISOLATE_EVENT,
	PSC_TRAP,
	LWD_TRAP,
	LQI_TRAP,
	QSFP_PRESENCE_TRAP,
	QSFP_FAULT_TRAP,
	RESCAN_EVENT,
	NL_PM_CMD_EVENT,
	NUM_PM_TRIGGERS
};

/**
 * enum sd_error - Subdevice error conditions
 * @SD_ERROR_FAILED: Subdevice has been marked as FAILED
 * @SD_ERROR_FW: Firmware error
 * @NUM_SD_ERRORS: Number of error conditions (always last)
 */
enum sd_error {
	SD_ERROR_FAILED,
	SD_ERROR_FW,
	NUM_SD_ERRORS
};

/**
 * enum fport_health - Port health indicator
 * @FPORT_HEALTH_OFF: disabled/not present
 * @FPORT_HEALTH_FAILED: not functional
 * @FPORT_HEALTH_DEGRADED: functional, degraded
 * @FPORT_HEALTH_HEALTHY: functional, healthy
 */
enum fport_health {
	FPORT_HEALTH_OFF,
	FPORT_HEALTH_FAILED,
	FPORT_HEALTH_DEGRADED,
	FPORT_HEALTH_HEALTHY
};

/**
 * enum fport_issue - Cause(s) for link degradation (YELLOW)
 * @FPORT_ISSUE_LQI: too many link errors (link quality < 4)
 * @FPORT_ISSUE_LWD: link width degraded
 * @FPORT_ISSUE_RATE: bit rate degraded
 * @NUM_FPORT_ISSUES: Number of FPORT degradation reasons (always last)
 *
 * Used as bit index into &fport_status.issues
 */
enum fport_issue {
	FPORT_ISSUE_LQI,
	FPORT_ISSUE_LWD,
	FPORT_ISSUE_RATE,
	NUM_FPORT_ISSUES
};

/**
 * enum fport_error - Cause for link failure (RED)
 * @FPORT_ERROR_FAILED: Driver operation on port failed
 * @FPORT_ERROR_ISOLATED: Invalid neighbor GUID, isolated by driver
 * @FPORT_ERROR_FLAPPING: Driver detected link flapping
 * @FPORT_ERROR_LINK_DOWN: Firmware reported link down (PSC)
 * @FPORT_ERROR_DID_NOT_TRAIN: Driver timed out link training
 * @FPORT_ERROR_LOOPBACK: Both ends of the link are the same device.
 * @NUM_FPORT_ERRORS: Number of FPORT error reasons (always last)
 */
enum fport_error {
	FPORT_ERROR_FAILED,
	FPORT_ERROR_ISOLATED,
	FPORT_ERROR_FLAPPING,
	FPORT_ERROR_LINK_DOWN,
	FPORT_ERROR_DID_NOT_TRAIN,
	FPORT_ERROR_LOOPBACK,
	NUM_FPORT_ERRORS
};

/**
 * struct fport_status - Logical port state for external query
 * @linkup_since: timestamp indicating when link up was noticed
 * @health: port health indicator
 * @link_failures: number of link failures detected
 * @link_degrades: number of link degrades detected
 * @issues: causes for link degradation
 * @errors: causes for link error
 */
struct fport_status {
	s64 linkup_since;
	enum fport_health health;
	u64 link_failures;
	u64 link_degrades;
	DECLARE_BITMAP(issues, NUM_FPORT_ISSUES);
	DECLARE_BITMAP(errors, NUM_FPORT_ERRORS);
};

struct routing_fidgen {
	/*
	 * mask over the full physical address (NOT 46-bit bridge address
	 * line)
	 *   [51:0] address mask
	 */
	u64 mask_a;
	u64 mask_b;
	u64 mask_d;
	u64 mask_h;

	/* [6:6] shift right flag */
	/* [5:0] shift amount */
	u8 shift_a;
	u8 shift_b;
	u8 shift_h;

	/* [2:0] hash mask modulo operand */
	u8 modulo;
};

/**
 * struct fsubdev_routing_info - Tracks per-sd routing state.
 * @topo: the topology context that tracks sweep state
 * @plane_link: entry in plane list
 * @plane: pointer to plane
 * @state: routing-level subdevice state
 * @uft: currently active routing tables
 * @uft_next: next uft being built by active sweep
 * @dfid_map: currently active DPA->DFID map
 * @dfid_map_next: next DFID map being built by active sweep
 * @fidgen: bridge fidgen register configuration
 * @dpa_idx_base: dpa lookup table index base
 * @dpa_idx_range: dpa lookup table range
 * @fid_group: abstract fid "group" which determines assigned fids
 * @fid_mgmt: management fid for cport
 * @fid_base: base fid of the bridge's fid block
 * @plane_index: per-plane index of this subdevice
 */
struct fsubdev_routing_info {
	struct routing_topology *topo;
	struct list_head plane_link;
	struct routing_plane *plane;
	enum fsubdev_routing_info_state {
		TILE_ROUTING_STATE_ERROR = 0,
		TILE_ROUTING_STATE_VALID,
	} state;
	struct routing_uft *uft;
	struct routing_uft *uft_next;
	struct routing_dfid_map *dfid_map;
	struct routing_dfid_map *dfid_map_next;
	struct routing_fidgen fidgen;
	u16 dpa_idx_base;
	u16 dpa_idx_range;
	u16 fid_group;
	u16 fid_mgmt;
	u16 fid_base;
	u16 plane_index;
};

/**
 * struct fsubdev - Per-subdevice state
 * @fdev: link to containing device
 * @mbdb: link to dedicated mailbox struct
 * @csr_base: base address of this subdevice's memory
 * @irq: assigned interrupt
 * @name: small string to describe SD
 * @asic_rev_info: raw contents of the asic rev info register
 * @fw_work: workitem for firmware programming
 * @pm_work: workitem for port management
 * @ue_work: workitem for uevent processing
 * @rescan_work: workitem to signal PM thread if ports need rescanning
 * @debugfs_dir: sd-level debugfs dentry
 * @debugfs_port_dir: debugfs_dir/port-level debugfs dentry
 * @kobj: kobject for this sd in the sysfs tree
 * @fw_comm_errors: attribute for fw_comm_errors sysfs file
 * @fw_error: attribute for fw_error sysfs file
 * @sd_failure: attribute for sd_failure sysfs file
 * @firmware_version: attribute for firmware_version sysfs file
 * @guid: GUID retrieved from firmware
 * @txcal: TX calibration data for all fabric ports
 * @switchinfo: switch information read directly from firmware
 * @extended_port_cnt: count of all ports including CPORT and bridge ports
 * @port_cnt: count of all fabric ports
 * @PORT_COUNT: number of supported ports
 * @fport_lpns: bitmap of which logical ports are fabric ports
 * @bport_lpns: bitmap of which logical ports are bridge ports
 * @fw_version: version information of firmware
 * @fw_running: whether runtime firmware is initialized/running
 * @errors: bitmap of active error states
 * @pm_work_lock: protects ok_to_schedule_pm_work
 * @ok_to_schedule_pm_work: indicates it is OK to request port management
 * @pm_triggers: event triggering for port management
 * @routable_link: link in global routable_list
 * @routing: routing information
 * @port: internal port state, includes references into @portinfo_op
 * @portinfo_op: all port information, read from firmware via MBDB op
 * @_portstatus: logical port status, access via @port_status/@next_port_status
 * @port_status: for querying port status via RCU (organized by lpn)
 * @next_port_status: for updating port status via RCU (organized by lpn)
 * @psc_trap_count: number of port state change trap notifications reported
 * @lwd_trap_count: number of link width degrade trap notifications reported
 * @lqi_trap_count: number of link quality issue trap notifications reported
 * @qsfp_fault_trap_count: number of qsfp faulted trap notifications reported
 * @qsfp_present_trap_count: number of qsfp present trap notifications reported
 * @cport_init_ctrl_reg_lock: controls access to the cport_init_ctrl_reg
 * @statedump: state dump data information
 *
 * Used throughout the driver to maintain information about a given subdevice.
 *
 * Protection mechanisms used outside of init/destroy are documented inline. Sync probe is the
 * context of the initial probe function. Async probe includes the initialization threads used to
 * load the firmware and platform configuration before enabling general processing.
 */
struct fsubdev {
	/* pointers const after sync probe, content protection is by object type */
	struct fdev *fdev;
	struct mbdb *mbdb;
	char __iomem *csr_base;

	/* values const after sync probe */
	int irq;
	char name[8];
	u64 asic_rev_info;

	/* work items for thread synchronization */
	struct work_struct fw_work;
	struct work_struct pm_work;
	struct delayed_work ue_work;
	struct work_struct rescan_work;

	/* values const after async probe */
	struct dentry *debugfs_dir;
	struct dentry *debugfs_port_dir;
	struct kobject *kobj;
	struct device_attribute fw_comm_errors;
	struct device_attribute fw_error;
	struct device_attribute sd_failure;
	struct device_attribute firmware_version;

	u64 guid;
	u16 txcal[PORT_COUNT];
	struct mbdb_op_switchinfo switchinfo;
	u8 extended_port_cnt;
	u8 port_cnt;
	DECLARE_BITMAP(fport_lpns, PORT_COUNT);
	DECLARE_BITMAP(bport_lpns, PORT_COUNT);
	struct mbdb_op_fw_version_rsp fw_version;

	/* essentially const after async probe (RISC RESET invalidates driver state anyway) */
	bool fw_running;

	/* atomic, never cleared after sync probe */
	DECLARE_BITMAP(errors, NUM_SD_ERRORS);

	/* protects ok_to_schedule_pm_work */
	struct mutex pm_work_lock;

	/* for PM thread interaction, protected by pm_work_lock */
	bool ok_to_schedule_pm_work;

	/* for PM thread interaction, protected by atomic bitops */
	DECLARE_BITMAP(pm_triggers, NUM_PM_TRIGGERS);

	/* protected by routable_lock, written by routing with exclusive lock */
	struct list_head routable_link;
	struct fsubdev_routing_info routing;

	/* protections are documented in &struct fport */
	struct fport port[PORT_COUNT];

	/* protected by routable_lock, written by PM thread with shared lock */
	DECLARE_MBDB_OP_PORTINFO(portinfo_op, PORT_COUNT);

	/* protected by RCU, owned by PM, ping-pong data structure accessed by pointer only */
	struct fport_status _portstatus[2 * PORT_COUNT]; /* backing data, owned by PM thread */
	struct fport_status __rcu *port_status; /* read by any */
	struct fport_status *next_port_status; /* written by PM thread */

	/*
	 * atomic, incremented by PM thread
	 */
	atomic64_t psc_trap_count;
	atomic64_t lwd_trap_count;
	atomic64_t lqi_trap_count;
	atomic64_t qsfp_fault_trap_count;
	atomic64_t qsfp_present_trap_count;

	/* protects cport_init_ctrl_reg */
	struct mutex cport_init_ctrl_reg_lock;

	/* protections are documented in &struct statedump */
	struct state_dump statedump;
};

/* to iterate over all fabric ports on a subdevice by logical port number */
#define for_each_fabric_lpn(bit, sd)	\
	for_each_set_bit(bit, (sd)->fport_lpns, PORT_COUNT)

/* to iterate over all bridge ports on a subdevice by logical port number */
#define for_each_bridge_lpn(bit, sd)	\
	for_each_set_bit(bit, (sd)->bport_lpns, PORT_COUNT)

#define for_each_masked_port(port, lpn, ports, mask, size)     \
	for ((lpn) = find_first_bit((mask), (size)),           \
	     (port) = (lpn) < (size) ? (ports) + (lpn) : NULL; \
	     (lpn) < (size);                                   \
	     (lpn) = find_next_bit((mask), (size), (lpn) + 1), \
	     (port) = (lpn) < (size) ? (ports) + (lpn) : NULL)

#define for_each_fabric_port(port_, lpn, sd) \
	for_each_masked_port(port_, lpn, (sd)->port, (sd)->fport_lpns, PORT_COUNT)

#define for_each_bridge_port(port_, lpn, sd) \
	for_each_masked_port(port_, lpn, (sd)->port, (sd)->bport_lpns, PORT_COUNT)

/**
 * get_fport_handle - Returns the fabric port for a given logical port number.
 * @sd: subdevice pointer
 * @lpn: logical port number
 *
 * Return: The fport pointer at the lpn if valid, otherwise NULL.
 */
static inline struct fport *get_fport_handle(struct fsubdev *sd, u8 lpn)
{
	return lpn < PORT_COUNT && test_bit(lpn, sd->fport_lpns) ? sd->port + lpn : NULL;
}

/**
 * struct psc_presence_rule - presence detection rule
 * @method: method used for presence detection
 * @index: method-specific index used for presence detection
 * @subdev_map: which subdevices this affects (bitmap, 0=all)
 * @reserved: forces port_map to be 32-bit aligned, should be 0 currently
 * @port_map: which ports this affects (bitmap, 0=all)
 *
 * Identifies a presence detection rule, used to identify which ports are
 * physically connected to another device so that only connected ports need be
 * enabled.
 *
 * Presence detection rules are given in sets, where each set contains all
 * rules for a given socket_id (there may be multiple devices in a system with
 * the same socket_id).
 *
 * If a socket_id has no presence-detection rules, the default KMD behavior
 * (automatically enabling FIXED or VARIABLE ports) applies. If any rules are
 * specified, these are used to determine which ports are enabled at startup
 * automatically, and by default no other ports are enabled on this device.
 * Presence rules are OR'ed together: a port is enabled at startup if any rule
 * enables it.
 *
 * For hardware detection, set @method to PRESENCE_RULE_LINK_CONTROL, in which
 * case the rule corresponds to the link_control bit identified by @index
 * (these bits are read via GPIO automatically at startup). If the bit is set,
 * the port is enabled. This is normally the only method used.
 *
 * Set @method to PRESENCE_RULE_DEFAULT to override the default rule, in which
 * case @index specifies behavior (disable on zero, else enable). This may be
 * used with @index=0 as the only presence rule to prevent ports from being
 * automatically enabled on that device. This may be used with @index=1 in
 * conjunction with other presence rules to allow specific ports to be enabled
 * by default while others use hardware detection rules.
 *
 * In all cases, subdev_map and port_map are bitfield maps corresponding to a
 * set of subdevices and ports (subdev_map=1/port_map=6 corresponds to
 * subdevice 0 ports 1 and 2, for instance), except that a value of 0
 * corresponds to all instances. Thus, subdev_map=0/port_map=0 maps to all
 * ports on all subdevices.
 *
 * It is possible for two rules to identify the same @method and @index, to
 * allow different sets of ports to be controlled on each subdevice, for
 * instance.
 */
struct psc_presence_rule {
	u8 method;
	u8 index;
	u8 subdev_map;
	u8 reserved;
	u32 port_map;
};

/*
 * TX calibration blob, optionally found after PSC
 */

/*
 * TXCAL BLOB magic number: corresponds to the string "Xe Tx Cal Blob\0\0"
 */
#define TXCAL_BLOB_MAGIC_0 (0x54206558)
#define TXCAL_BLOB_MAGIC_1 (0x61432078)
#define TXCAL_BLOB_MAGIC_2 (0x6c42206c)
#define TXCAL_BLOB_MAGIC_3 (0x0000626f)

/*
 * Format of struct txcal_blob being used
 */
#define TXCAL_VERSION_CURRENT (1)
#define TXCAL_VERSION_MIN (1)
#define TXCAL_VERSION_MAX (1)

/**
 * struct txcal_settings - TX calibration settings for a subdevice
 * @guid: GUID of subdevice
 * @port_settings: array of settings for each possible fabric port
 *
 * Identifies the SERDES TX DCC MARGIN parameters for each possible fabric
 * port on a subdevice. @port_settings[lpn] == 0 means there is no calibration
 * data for that fabric port. These are applied before bringing any ports up
 * via the TX_DCC_MARGIN_PARAM subopcode of the SERDES_MARGIN opcode.
 */
struct txcal_settings {
	u64 guid;
	u16 port_settings[TXCAL_PORT_COUNT];
};

/**
 * struct txcal_blob - TX calibration settings for all subdevices
 * @magic: must be 0x54206558, 0x61432078, 0x6c42206c, 0x0000626f ("Xe Tx Cal Blob\0\0")
 * @format_version: must be in range [TXCAL_VERSION_MIN : TXCAL_VERSION_MAX]
 * @cfg_version: configuration version, if specified
 * @date: UTC generation date in BCD (YYYYMMDD)
 * @time: UTC generation time in BCD (HHMMSS)
 * @size: size of entire structure including @data array
 * @num_settings: number of elements in @data array
 * @crc32c_data: CRC covering @data array
 * @crc32c_hdr: CRC covering all other header fields
 * @data: Array of per-subdevice TX calibration settings
 *
 * Identifies all TX calibration settings for a set of subdevices. The set
 * must include all subdevices for the device from which this blob was read
 * and will typically include all subdevices in the system (like the binary
 * PSC blob). This blob is normally located in SPI flash immediately after
 * the PSC blob, and is only used if present (as indicated by valid magic
 * and CRC fields).
 */
struct txcal_blob {
	u32 magic[4];
	u32 format_version;
	u32 cfg_version;
	u32 date;
	u32 time;
	u32 size;
	u32 num_settings;
	u32 crc32c_data;
	u32 crc32c_hdr;
	struct txcal_settings data[];
};

/**
 * struct fdev - Device structure for IAF/fabric device component
 * @registered: Registered with platform device
 * @sd: subdevice structures
 * @dev_disabled: On a PCIe error, disable access to the PCI bus
 * @fwinit_refcnt: number of subdevices needing firmware initialization
 * @all_sds_inited: indicates all subdevices have been initialized
 * @pdev: bus device passed in probe
 * @pd: platform specific data
 * @fabric_id: xarray index based on parent index and product type
 * @link_config: link config pin values read from GPIO at startup
 * @port_unroute_list: list of ports with pending unroute request
 * @mappings_ref.lock: protect the mappings_ref data
 * @mappings_ref.count: current mapped buffer count
 * @mappings_ref.remove_in_progress: indicate unmap should show completion
 * @mappings_ref.complete: completion after all buffers are unmapped
 * @mappings_ref: Reference count of parent mapped buffers
 * @fw: used for interacting with FW API
 * @mei_ops_lock: mutex lock for mei_ops/dev/bind_continuation/work
 * @mei_ops: bound MEI operation functions (if not NULL)
 * @mei_dev: device to use with @mei_ops
 * @mei_bind_continuation: set when unbound @mei_ops needed, called by bind
 * @mei_continuation_timer: used to timeout receipt of MEI bind continuation
 * @mei_work: work struct for MEI completion
 * @psc.work: work struct for PSC processing
 * @psc.done: signals that PSC processing is done
 * @psc.abort: indicates whether to stop waiting for PSC availability
 * @psc.err: indicates whether PSC processing encountered an error
 * @psc.data: PSC data, from flash or FW API
 * @psc.size: size of PSC data, from flash or FW API
 * @psc.as_fw: used to access PSC override via FW API
 * @psc.mtd: used to access PSC data from NVMEM via MTD API
 * @psc.brand: copied, NUL-terminated brand string extracted from PSC data
 * @psc.product: copied, NUL-terminated product string extracted from PSC data
 * @psc.version: copied, NUL-terminated version string constructed from PSC data
 * @psc.ini_buf: buffers for extracting ini_bin data from NVMEM
 * @psc.ini_buf.data: copied PSC data of @psc.ini_buf
 * @psc.ini_buf.idx: PSC data indices of @psc.ini_buf to identify buffers for reuse
 * @psc.ini_buf.size: PSC data size of @psc.ini_buf to identify buffers for reuse
 * @psc.ini_buf.do_not_free: @psc.ini_buf.data refers to another buffer that will be freed
 * @psc.n_presence_rules: number of presence rules for this device in PSC data
 * @psc.presence_rules: copied presence rules for this device, extracted from PSC data
 * @psc: PSC processing info
 * @p2p: the active cached peer connectivity results (rcu protected)
 * @p2p_next: the pending peer connectivity results
 * @dir_node: debugfs directory node for this device
 * @fabric_node: debugfs file fabric symbolic link
 * @dpa_node: debugfs file dpa symbolic link
 * @refs: references on this instance
 * @fdev_released: signals fdev has been erased from the xarray
 * @startup_mode: startup mode
 *
 * Used throughout the driver to maintain information about a given device.
 */
struct fdev {
	bool registered;
	struct fsubdev sd[IAF_MAX_SUB_DEVS];
	bool dev_disabled;
	atomic_t fwinit_refcnt;
	bool all_sds_inited;
#if IS_ENABLED(CONFIG_AUXILIARY_BUS)
	struct auxiliary_device *pdev;
#else
	struct platform_device *pdev;
#endif
	const struct iaf_pdata *pd;
	u32 fabric_id;
	u32 link_config;
	struct list_head port_unroute_list;

	struct {
		/* protect the mapping count and remove_in_progress flag */
		struct mutex lock;
		int count;
		bool remove_in_progress;
		struct completion complete;
	} mappings_ref;

	const struct firmware *fw;
	/* protects mei_ops/mei_dev/mei_bind_continuation */
	struct mutex mei_ops_lock;
	const struct i915_iaf_component_ops *mei_ops;
	struct device *mei_dev;
	void (*mei_bind_continuation)(struct fdev *dev);
	struct timer_list mei_continuation_timer;
	struct work_struct mei_work;
	struct {
		struct work_struct work;
		struct completion done;
		struct completion abort;
		int err;
		const u8 *data;
		size_t size;
		const struct firmware *as_fw;
		struct mtd_info *mtd;
		char *brand;
		char *product;
		char version[MAX_PSCBIN_VERSION];
		struct {
			const u8 *data;
			u32 idx;
			u32 size;
			bool do_not_free;
		} ini_buf[IAF_MAX_SUB_DEVS];
		size_t n_presence_rules;
		struct psc_presence_rule *presence_rules;
		struct txcal_blob *txcal;
	} psc;
	struct routing_p2p_entry __rcu *p2p;
	struct routing_p2p_entry *p2p_next;
	struct dentry *dir_node;
	struct dentry *fabric_node;
	struct dentry *dpa_node;
	struct kref refs;
	struct completion fdev_released;
	enum iaf_startup_mode startup_mode;
};

u64 bps_link_speed(u8 port_info_link_speed);

void fdev_put(struct fdev *dev);
void fdev_get_early(struct fdev *dev);
int fdev_insert(struct fdev *dev);

/*
 * This is the fdev_process_each callback function signature
 * Returning 0 indicates continue
 * Any other return value indicates terminate
 */
typedef int (*fdev_process_each_cb_t)(struct fdev *dev, void *args);

int fdev_process_each(fdev_process_each_cb_t cb, void *args);

struct fdev *fdev_find(u32 fabric_id);

/*
 * Returns the sd index/offset relative to its device.
 */
static inline u8 sd_index(struct fsubdev *sd)
{
	return sd - sd->fdev->sd;
}

/*
 * dev_is_startup_debug - Test for full debug startup mode.
 * @dev: device
 *
 * Return: True if we're actually starting up straight into debug mode,
 * bypassing all normal device init behavior.
 */
static inline bool dev_is_startup_debug(struct fdev *dev)
{
#if IS_ENABLED(CONFIG_IAF_DEBUG_STARTUP)
	return dev && dev->startup_mode == STARTUP_MODE_DEBUG;
#else
	return false;
#endif
}

/**
 * dev_is_runtime_debug - Test for runtime debug startup mode.
 * @dev: device
 *
 * Return: True if, regardless of the startup logic, we end up in debug mode
 * once we're past firmware init.
 */
static inline bool dev_is_runtime_debug(struct fdev *dev)
{
#if IS_ENABLED(CONFIG_IAF_DEBUG_STARTUP)
	return dev && (dev->startup_mode == STARTUP_MODE_DEBUG ||
		       dev->startup_mode == STARTUP_MODE_FWDEBUG);
#else
	return false;
#endif
}

/*
 * dev_is_preload - Test for preload startup mode.
 * @dev: device
 *
 * Return: True if the device is in preload startup mode.
 */
static inline bool dev_is_preload(struct fdev *dev)
{
	return dev && dev->startup_mode == STARTUP_MODE_PRELOAD;
}

/*
 * routable_lock affects ALL devices in the system and protects data structures that must not
 * change during routing, specifically all routing fields of all elements in the routable list. It
 * also protects port state written by port manager instances that affect routing. Individual port
 * managers must be blocked from updating their state while they are being used by routing but must
 * not be blocked from updating their own state by port managers working on other tiles.
 *
 * rw_semaphore down_write operations take the lock exclusively (traditionally protecting the data
 * for writing) and down_read operations take it in a shared manner (traditionally protecting the
 * data for reading). Port managers exclusively access their own data in single threads are thus
 * allowed to update their own data while using a shared read lock. All other consumers must use an
 * exclusive write lock to access that data.
 *
 * Routing and general consumers must use down_write and treat routable_lock as a mutex to acquire
 * read/write access to all data protected by the lock. Port managers (only) may use down_read to
 * access ONE tile's data and must be guaranteed to be race free independent of the lock.
 *
 * Routing data structures that are not written by port managers can be protected for read-only
 * access using down_read() to obtain a shared lock in the traditional manner. During a routing
 * sweep, after routing has computed new routing state, that state is frozen and the exclusive
 * write lock is downgraded to a shared read lock for remaining device programming. A shared lock is
 * also used to protect debugfs read access of frozen routing state.
 */
extern struct rw_semaphore routable_lock;
extern struct list_head routable_list;

static inline struct device *sd_dev(const struct fsubdev *sd)
{
	return &sd->fdev->pdev->dev;
}

static inline struct device *fdev_dev(const struct fdev *dev)
{
	return &dev->pdev->dev;
}

static inline struct device *fport_dev(const struct fport *port)
{
	return &port->sd->fdev->pdev->dev;
}

#define BYTES_PER_FLIT 8

static inline u64 flits_to_bytes(u64 flits)
{
	u64 bytes;

	return __builtin_umulll_overflow(flits, BYTES_PER_FLIT, &bytes) ? ULLONG_MAX : bytes;
}

static inline u32 lane_number(struct fport **port)
{
	return port - (*port)->lanes_port;
}

void indicate_subdevice_error(struct fsubdev *sd, enum sd_error err);

/* The following two functions increase device reference count: */
struct fdev *fdev_find_by_sd_guid(u64 guid);
struct fsubdev *find_sd_id(u32 fabric_id, u8 sd_index);

/* routable_lock must be held across this (shared OK) */
struct fsubdev *find_routable_sd(u64 guid);

void iaf_complete_init_dev(struct fdev *dev);

bool is_fdev_registered(struct fdev *dev);

bool is_fport_registered(struct fport *port);

bool mappings_ref_check(struct fdev *dev);

extern struct workqueue_struct *iaf_unbound_wq;

#endif
