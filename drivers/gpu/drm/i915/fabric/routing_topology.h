/* SPDX-License-Identifier: MIT */
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#ifndef ROUTING_TOPO_H_INCLUDED
#define ROUTING_TOPO_H_INCLUDED

#include <linux/sizes.h>
#include <linux/bitfield.h>

#include "csr.h"
#include "iaf_drv.h"

#define ROUTING_UFT_SIZE (48u * SZ_1K)

/* the alignment of the top-level devices in the DPA space */
#define ROUTING_DEV_DPA_ALIGNMENT (128ull * SZ_1G)

/* the minimum size of the dpa range for a subdevice */
#define ROUTING_MIN_DPA_PER_SD (8ull * SZ_1G)

/*
 * FID Structure
 *
 * [19:6] base fid
 *          - pulled from the whoami register (source) or the bridge fid
 *            lookup table (destination)
 * [ 5:3] path id
 *          - pulled from the hash/modulo logic applied to the destination
 *            physical address
 *          - note that this is configurable from 0 to 3 bits
 * [ 2:1] mdfi channel
 *          - set by the originator... source and destination always use
 *            the same mdfi channel
 * [ 0:0] host flag
 *          - not currently used; spec'd for a method of indication that
 *            this traffic is bound for a host memory interface
 *
 * Due to the host flag being unused, all odd FIDs are invalid within a
 * block.
 */
#define ROUTING_FID_BASE_MASK GENMASK(19, 6)
#define ROUTING_FID_PATH_MASK GENMASK(5, 3)
#define ROUTING_FID_MDFI_MASK GENMASK(2, 1)
#define ROUTING_FID_HOST_MASK GENMASK(0, 0)

#define ROUTING_FID_NUM_PATHS 8
#define ROUTING_FID_NUM_CHANNELS 3

static inline bool is_valid_fid(u16 fid)
{
	bool host_fid =  FIELD_GET(ROUTING_FID_HOST_MASK, fid);
	u16 channel = FIELD_GET(ROUTING_FID_MDFI_MASK, fid);

	return !host_fid && channel < ROUTING_FID_NUM_CHANNELS;
}

/*
 * FID Address Space
 *
 * The following shows the layout of the FID address space.
 *
 * Tiles are assumed to be 64 GB aligned; thus the upper bits of the DPA
 * starting at bit 36 determine the address range index.  This index then maps
 * twice into the FID space: once for the management FID, and once for the
 * contiguous block of FIDs assigned to the bridge endpoints.
 *
 * The first 64 FIDs are reserved to ensure any that any a DFID lookup that
 * emits 0 can not overlap any valid FIDs.
 *
 * The second reserved block merely pads to the next by-64 alignment to begin
 * addressing bridge blocks.
 *
 * ---------
 *      0 : reserved
 *    ...
 *     63 : reserved
 * ---------
 *     64 : Mgmt FID for sd with DPA index 0 (0 GB)
 *     65 : Mgmt FID for sd with DPA index 1 (64 GB)
 *     66 : Mgmt FID for sd with DPA index 2 (128 GB)
 *    ...
 *    818 : Mgmt FID for sd with DPA index 755 (48320 GB)
 * ---------
 *    819 : reserved
 *    ...
 *    831 : reserved
 * ---------
 *    832 : Starting bridge FID for sd with DPA index 0
 *    896 : Starting bridge FID for sd with DPA index 1
 *    960 : Starting bridge FID for sd with DPA index 2
 *    ...
 *  49088 : Starting bridge FID for sd with DPA index 755
 * ---------
 */

/* The size of a FID block assigned to a sd for bridge ports. */
#define ROUTING_FID_BLOCK_SIZE 64

/*
 * Generates a FID mask for a FID block consisting of only valid DFIDs
 * (those without the host flag set and those avoiding the fourth
 * MDFI channel).
 */
#define ROUTING_FID_BLOCK_MASK 0x1515151515151515ull

/* The number of valid (non-host) FIDs per block. */
#define ROUTING_VALID_FIDS_PER_BLOCK (ROUTING_FID_NUM_PATHS * ROUTING_FID_NUM_CHANNELS)

/* Limit on maximum path length through a fabric. */
#define ROUTING_HOP_LIMIT 64

/*
 * The starting FID for management port assignment.
 *
 * Defaulted to the block size so that all FIDs in the "zero" block are
 * unused.  This allows a zero output in the DPA->DFID LUT to refer
 * to an entire invalid block, rather than just an invalid base FID.
 */
#define ROUTING_FID_CPORT_BASE ROUTING_FID_BLOCK_SIZE

/*
 * The starting FID for bridge port assignment.
 *
 * Must be aligned to the block size.
 */
#define ROUTING_FID_BLOCK_BASE 832

/*
 * The maximum number of supported devices.
 *
 * A device with a DPA range index less than this is guaranteed to receive a
 * FID and be mappable by the DFID LUT.
 *
 * Determined by the minimum of:
 *   - the number of CPORT FIDs that fit into the FID address space
 *   - the number of bridge FID blocks that fit into the FID address space
 *   - the number of entries available in the DPA->DFID lookup table
 */
#define ROUTING_MAX_DEVICES 755

/*
 * The number of entries in the bridge DFID lookup table.
 */
#define ROUTING_DPA_DFID_MAP_SIZE 8192

/*
 * The FIDGEN LUT maps DPA indexes to only the upper bits of the DFID, which
 * varies based on the modulo in use.  Since we're statically configuring a
 * maximum modulo, this is statically 6 bits of shift to convert between
 * LUT entries and DFIDs.
 *
 * DFID == (LUT[i] << 6) | (PATH << 3) | (MDFI << 1) | HOST
 */
#define ROUTING_DPA_DFID_MAP_SHIFT 6

struct routing_topology;

struct routing_dfid_map {
	u16 size;

	/*
	 * each entry:
	 *   [19:0] base DFID, after shifting out the lower path/mdfi bits
	 */
	u64 dfid[];
};

/*
 * invalid port value for the uncompressed 8-bit hw rpipe used by the mbdb
 * mailbox operations
 *
 * don't set the upper 'X' bit used for override tables or adaptive routing,
 * as these features are disabled by HW
 */
#define ROUTING_UFT_INVALID_PORT8 0x7f

/*
 * invalid port value for the compressed 4-bit driver uft structure used by
 * the in-memory driver structures
 */
#define ROUTING_UFT_INVALID_PORT4 0xf

/*
 * Routing Table Structure
 *
 * A unicast forwarding table (UFT) is a mapping from destination FID (DFID)
 * to output port.
 *
 * On devices, this is stored in the route pipe IP block (RPIPE), with 48k
 * entries, indexed by DFID, each mapping to an 8-bit port.
 *
 * In the driver all management FIDs are allocated as a single contiguous
 * block.  Each subdevice is additionally allocated a single contiguous block
 * for its bridge endpoint FIDs, and these blocks are stored in an xarray.
 *
 * The driver builds two (potentially) different UFTs for each subdevice: one
 * for CPORT and bridge ports, and another for fabric ports.  This lets the
 * driver route traffic entering the fabric differently than traffic that is
 * making a hop through an intermediate switch.
 *
 * Since the driver only needs 4 bits to indicate a valid fabric port, and it
 * builds only two UFTs, it can map each FID to both ports in a single byte.
 * In the result::
 *  - the lower nibble represents the entry for the "origin" UFT, and...
 *  - the upper nibble represents the entry for the "hop" UFT.
 *
 * The routing_entry_* functions abstract away this encoding, taking as a
 * parameter a selector to distinguish between the two UFTs.
 */

/**
 * struct routing_uft - Sparsely stores all of the unicast forwarding table
 * (UFT) entries for a sd.
 * @mgmt: A single UFT block mapping mgmt DFIDs to ports.
 * @bridges: An xarray of UFT blocks mapping bridge DFIDs to ports, one block
 *   per destination sd, indexed by DPA index.  Entries are allocated
 *   on demand during routing.
 */
struct routing_uft {
	u8 *mgmt;
	struct xarray bridges;
};

u8 *routing_uft_block_alloc(u16 num_fids);
u8 *routing_uft_bridge_get(struct routing_uft *uft, struct fsubdev *sd_dst);

struct routing_uft *routing_uft_alloc(void);
void routing_uft_destroy(struct routing_uft *uft);
void routing_uft_update(struct fsubdev *sd);

void routing_dfid_map_update(struct fsubdev *sd);

static inline void routing_update(struct fsubdev *sd)
{
	routing_uft_update(sd);
	routing_dfid_map_update(sd);
}

/* UFT source selector (fabric vs bridge UFTs are programmed differently) */
enum uft_sel {
	/* used for traffic arriving locally */
	UFT_SEL_ORIGIN = 1,
	/* used for traffic arriving from the fabric */
	UFT_SEL_HOP = 2,
	/* used for all traffic */
	UFT_SEL_BOTH = 3,
};

static inline void routing_uft_entry_set(u8 *block, enum uft_sel sel,
					 u16 fid_offset, u8 port)
{
	switch (sel) {
	case UFT_SEL_ORIGIN:
		block[fid_offset] = (block[fid_offset] & 0xf0) | (port & 0xf);
		break;
	case UFT_SEL_HOP:
		block[fid_offset] = ((port & 0xf) << 4) |
				    (block[fid_offset] & 0xf);
		break;
	case UFT_SEL_BOTH:
		block[fid_offset] = ((port & 0xf) << 4) | (port & 0xf);
		break;
	}
}

static inline u8 routing_uft_entry_get(const u8 *block, enum uft_sel sel,
				       u16 fid_offset)
{
	u8 port = ROUTING_UFT_INVALID_PORT4;

	switch (sel) {
	case UFT_SEL_ORIGIN:
		port = block[fid_offset] & 0xf;
		break;
	case UFT_SEL_HOP:
		port = (block[fid_offset] >> 4) & 0xf;
		break;
	case UFT_SEL_BOTH:
		break;
	}

	return port;
}

static inline int routing_sd_is_error(struct fsubdev *sd)
{
	return sd->routing.state == TILE_ROUTING_STATE_ERROR;
}

#define ROUTING_COST_INFINITE 0xffff

struct routing_plane {
	struct list_head topo_link;
	struct list_head sd_list;
	struct routing_topology *topo;
	u16 index;
	u16 num_subdevs;
	u16 *cost;
};

enum routing_policy {
	ROUTING_POLICY_DEFAULT,
	ROUTING_POLICY_RESILIENCY,
	ROUTING_POLICY_ROUTE_THROUGH,
};

/**
 * struct routing_topology - Top-level fabric context maintained during a
 * routing sweep.
 * @plane_list: list of planes discovered
 * @num_planes: number of planes in plane_list
 * @max_dpa_index: maximum DPA index (inclusive)
 * @sd_error_signal: true if an sd transitioned to error during the sweep
 * @policy: routing policy enablement flags
 * @fid_groups: bit mask of in-use fid groups
 */
struct routing_topology {
	struct list_head plane_list;
	u16 num_planes;
	u16 max_dpa_index;
	bool sd_error_signal;
	enum routing_policy policy;
	unsigned long fid_groups[BITS_TO_LONGS(ROUTING_MAX_DEVICES)];
};

/*
 * The following API methods wrap the error flag in a tiny API that clarifies
 * its usage: to allow the topology context for a sweep to be signalled of any
 * subdevices transitioning to error, which guides error recovery behavior in
 * the routing engine.
 */

static inline void routing_topo_reset_sd_error(struct routing_topology *topo)
{
	topo->sd_error_signal = false;
}

static inline void routing_topo_signal_sd_error(struct routing_topology *topo)
{
	topo->sd_error_signal = true;
}

static inline bool routing_topo_check_sd_error(struct routing_topology *topo)
{
	return topo->sd_error_signal;
}

/**
 * neighbor_of - Returns the neighbor port associated with the specified port.
 * @port: The local port to query the neighbor of.
 *
 * Only valid if the routing.neighbor field has been populated, and only
 * remains valid as long as the routing lock is held.
 *
 * Return: The port neighboring the specified port.
 */
static inline struct fport *neighbor_of(struct fport *port)
{
	return port->routing.neighbor;
}

static inline bool port_is_routable(struct fport *port)
{
	return port->state == PM_PORT_STATE_ACTIVE &&
			test_bit(PORT_CONTROL_ROUTABLE, port->controls);
}

void routing_sd_transition_error(struct fsubdev *sd);

struct fsubdev *routing_sd_iter(int all);
struct fsubdev *routing_sd_next(struct fsubdev *sd, int all);

#endif /* ROUTING_TOPO_H_INCLUDED */
