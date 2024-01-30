// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#include <linux/bitfield.h>
#include <linux/delay.h>

#include "csr.h"
#include "ops.h"
#include "parallel.h"
#include "routing_io.h"
#include "routing_pause.h"

#define MAX_UFT_ENTRIES \
	(MBOX_PARAM_AREA_IN_BYTES - sizeof(struct mbdb_op_rpipe_set))

static void block_to_rpipe(u8 *rpipe, u8 *block, enum uft_sel sel, u16 len)
{
	u16 i;
	u8 lpn;

	for (i = 0; i < len; ++i) {
		lpn = routing_uft_entry_get(block, sel, i);
		rpipe[i] = lpn == ROUTING_UFT_INVALID_PORT4 ?
			   ROUTING_UFT_INVALID_PORT8 : lpn;
	}
}

static int rpipe_clear_block(struct fsubdev *sd, u32 port_mask,
			     struct mbdb_op_rpipe_set *set, u16 fid_base,
			     u16 fid_range)
{
	int err;

	sd_dbg(sd, "writing base 0x%04x range %u\n", fid_base,
	       fid_range);

	if (fid_range > MAX_UFT_ENTRIES)
		return -EINVAL;

	set->port_mask = port_mask;
	set->start_index = fid_base;
	set->num_entries = fid_range;
	memset(set->rpipe_data, ROUTING_UFT_INVALID_PORT8, fid_range);

	err = ops_rpipe_set(sd, set, set, false);
	if (err) {
		sd_err(sd, "rpipe clear: set failed: %d\n", err);
		return err;
	}

	return 0;
}

static u32 filter_mask(struct fsubdev *sd, enum uft_sel sel, u32 port_mask)
{
	u8 lpn;

	switch (sel) {
	case UFT_SEL_ORIGIN:
		/* local sources only; exclude fabric ports */
		for_each_fabric_lpn(lpn, sd)
			port_mask &= ~BIT(lpn);
		return port_mask;
	case UFT_SEL_HOP:
		/* fabric sources only; exclude cport and bridge ports */
		port_mask &= ~CPORT_LPN_MASK;
		for_each_bridge_lpn(lpn, sd)
			port_mask &= ~BIT(lpn);
		return port_mask;
	case UFT_SEL_BOTH:
		return port_mask;
	}

	return 0;
}

static int rpipe_write_block_sel(struct fsubdev *sd, u32 port_mask,
				 struct mbdb_op_rpipe_set *set,
				 u8 *block, enum uft_sel sel,
				 u16 fid_base, u16 fid_range)
{
	int err;

	sd_dbg(sd, "writing mask %08x sel %u base 0x%04x range %u\n",
	       port_mask, sel, fid_base, fid_range);

	set->port_mask = filter_mask(sd, sel, port_mask);
	if (!set->port_mask)
		return 0;

	set->start_index = fid_base;
	set->num_entries = fid_range;

	block_to_rpipe(set->rpipe_data, block, sel, fid_range);

	err = ops_rpipe_set(sd, set, set, false);
	if (err) {
		sd_err(sd, "rpipe write: set failed: %d\n", err);
		return err;
	}

	return 0;
}

static int rpipe_write_block(struct fsubdev *sd, u32 port_mask,
			     struct mbdb_op_rpipe_set *set,
			     u8 *block, u16 fid_base, u16 fid_range)
{
	int err;

	if (fid_range > MAX_UFT_ENTRIES)
		return -EINVAL;

	err = rpipe_write_block_sel(sd, port_mask, set, block, UFT_SEL_ORIGIN,
				    fid_base, fid_range);
	if (err)
		return err;

	err = rpipe_write_block_sel(sd, port_mask, set, block, UFT_SEL_HOP,
				    fid_base, fid_range);
	if (err)
		return err;

	return 0;
}

static bool rpipe_bridge_changed(u8 *block_curr, unsigned long block_idx,
				 struct routing_uft *uft_prev)
{
	u8 *block_prev;

	if (!uft_prev)
		return true;

	block_prev = xa_load(&uft_prev->bridges, block_idx);
	if (!block_prev)
		return true;

	return memcmp(block_curr, block_prev, ROUTING_FID_BLOCK_SIZE) != 0;
}

/**
 * rpipe_write_bridge_update - Finds and writes rpipe entries for blocks in the
 * newly computed UFT if they differ from the previous UFT.
 * @sd: The sd to write to.
 * @rpipe_buf: The mbdb op buffer to use.
 * @port_mask: The mask of ports to write to.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int rpipe_write_bridge_update(struct fsubdev *sd,
				     struct mbdb_op_rpipe_set *rpipe_buf,
				     u32 port_mask)
{
	u8 *block;
	u16 base;
	unsigned long i;
	int err;

	xa_for_each(&sd->routing.uft_next->bridges, i, block) {
		base = ROUTING_FID_BLOCK_BASE + i * ROUTING_FID_BLOCK_SIZE;

		if (!rpipe_bridge_changed(block, i, sd->routing.uft))
			continue;

		err = rpipe_write_block(sd, port_mask, rpipe_buf, block, base,
					ROUTING_FID_BLOCK_SIZE);
		if  (err) {
			sd_err(sd, "failed to write rpipe bridge block %u: %d",
			       base, err);
			return err;
		}
	}

	return 0;
}

/**
 * rpipe_write_bridge_remove - Finds and clears rpipe entries for blocks in the
 * old/previous UFT that no longer exist in the newly computed UFT.
 * @sd: The sd to write to.
 * @rpipe_buf: The mbdb op buffer to use.
 * @port_mask: The mask of ports to write to.
 *
 * Return: Zero on success, non-zero otherwise.
 */
static int rpipe_write_bridge_remove(struct fsubdev *sd,
				     struct mbdb_op_rpipe_set *rpipe_buf,
				     u32 port_mask)
{
	u8 *block;
	u16 base;
	unsigned long i;
	int err;

	xa_for_each(&sd->routing.uft->bridges, i, block) {
		if (xa_load(&sd->routing.uft_next->bridges, i))
			continue;

		base = ROUTING_FID_BLOCK_BASE + i * ROUTING_FID_BLOCK_SIZE;

		err = rpipe_clear_block(sd, port_mask, rpipe_buf, base,
					ROUTING_FID_BLOCK_SIZE);
		if  (err) {
			sd_err(sd, "failed to write rpipe bridge block %u: %d",
			       base, err);
			return err;
		}
	}

	return 0;
}

static int rpipe_write(struct fsubdev *sd)
{
	struct mbdb_op_rpipe_set *rpipe_buf;
	size_t max_block_size = max(ROUTING_MAX_DEVICES,
				    ROUTING_FID_BLOCK_SIZE);
	size_t rpipe_size = struct_size(rpipe_buf, rpipe_data, max_block_size);
	u32 port_mask = ~(~(u32)0 << sd->extended_port_cnt);
	int err;

	rpipe_buf = kzalloc(rpipe_size, GFP_KERNEL);
	if (!rpipe_buf)
		return -ENOMEM;

	err = rpipe_write_block(sd, port_mask, rpipe_buf,
				sd->routing.uft_next->mgmt,
				ROUTING_FID_CPORT_BASE, ROUTING_MAX_DEVICES);
	if (err) {
		sd_err(sd, "failed to write rpipe mgmt block: %d\n", err);
		goto exit;
	}

	err = rpipe_write_bridge_update(sd, rpipe_buf, port_mask);
	if (err)
		goto exit;

	if (sd->routing.uft) {
		err = rpipe_write_bridge_remove(sd, rpipe_buf, port_mask);
		if (err)
			goto exit;
	}

exit:
	kfree(rpipe_buf);
	return err;
}

static int write_port_pkgaddr(struct fsubdev *sd, u8 lpn)
{
	u64 addr_base = sd->fdev->pd->dpa.pkg_offset;
	u64 addr_range = sd->fdev->pd->dpa.pkg_size;
	u64 addr_csr = FIELD_PREP(MASK_BT_PAR_BASE, addr_base) |
		       FIELD_PREP(MASK_BT_PAR_RANGE, addr_range);
	int err;

	err = ops_linkmgr_port_csr_wr(sd, lpn, CSR_BT_PKG_ADDR_RANGE,
				      &addr_csr, sizeof(addr_csr), false);
	if (err) {
		dev_err(sd_dev(sd), "failed to set package address range: %d\n",
			err);
		return err;
	}

	return 0;
}

#define FIDGEN_BLOCK_SIZE 12

/*
 * Note that the FID isn't specific to current/next because it's invariant.
 */
static void build_fidgen_block(struct fsubdev *sd, u64 *data)
{
	data[0] = sd->routing.fidgen.mask_a;
	data[1] = sd->routing.fidgen.shift_a;
	data[2] = sd->routing.fidgen.mask_b;
	data[3] = sd->routing.fidgen.shift_b;
	data[4] = 0; /* rsvd0 */
	data[5] = 0; /* rsvd1 */
	data[6] = sd->routing.fidgen.mask_h;
	data[7] = sd->routing.fidgen.shift_h;
	data[8] = sd->routing.fidgen.modulo;
	data[9] = sd->routing.fid_base >> ROUTING_DPA_DFID_MAP_SHIFT;
	data[10] = sd->routing.fidgen.mask_d;
	data[11] = 0; /* static random */
}

static int write_port_fidgen(struct fsubdev *sd, u8 lpn)
{
	u64 data[FIDGEN_BLOCK_SIZE];
	u16 len = sizeof(data);
	int err;

	build_fidgen_block(sd, data);

	err = ops_linkmgr_port_csr_wr(sd, lpn, CSR_FIDGEN_MASK_A, data, len, false);
	if (err) {
		dev_err(sd_dev(sd),
			"failed to write fidgen block: %d\n", err);
		return err;
	}

	return 0;
}

/**
 * check_port_activation - Checks that the port logical state is active.
 * @sd: the subdevice to operate on
 * @lpn: the bridge port lpn to check
 *
 * Return: 0 on success, negative errno otherwise.
 *
 * This only warns if a port is inactive, as this is an expected case for
 * platforms running in standalone mode.  The KMD has no explicit dependencies
 * on port state, and will continue to operate normally.
 */
static int check_port_activation(struct fsubdev *sd, u8 lpn)
{
	u32 result = 0;
	int err;
	u8 state;

	err = ops_linkmgr_ps_get(sd, lpn, &result);
	if (err) {
		sd_err(sd, "p.%u: failed to get port state: %d\n",
		       lpn, err);
		return err;
	}

	state = FIELD_GET(PS_PPS_PORT_STATE, result);
	if (state != IAF_FW_PORT_ACTIVE)
		sd_info(sd, "p.%u: bridge port not active: result 0x%08x\n", lpn, result);

	return 0;
}

static int write_port_dfid_map(struct fsubdev *sd, u8 lpn)
{
	u8 *data = sd->routing.dfid_map ? (u8 *)sd->routing.dfid_map->dfid
					: NULL;
	u8 *data_next = (u8 *)sd->routing.dfid_map_next->dfid;
	u32 addr = CSR_FIDGEN_LUT_BASE;
	u32 remaining = (sd->routing.topo->max_dpa_index + 1) * sizeof(u64);
	u32 len;
	int err;

	while (remaining) {
		len = min_t(u32, remaining, MBOX_WRITE_DATA_SIZE_IN_BYTES);

		if (!data || memcmp(data, data_next, len)) {
			err = ops_linkmgr_port_csr_wr(sd, lpn, addr, data_next,
						      len, false);
			if (err) {
				dev_err(sd_dev(sd),
					"failed to write fidgen top csrs: %d\n",
					err);
				return err;
			}
		}

		addr += len;
		data_next += len;
		remaining -= len;

		if (data)
			data += len;
	}

	return 0;
}

static int bridge_write(struct fsubdev *sd)
{
	int err;
	u8 lpn;

	if (!sd->routing.dfid_map_next) {
		sd_err(sd, "invalid dfid map");
		return -EINVAL;
	}

	for_each_bridge_lpn(lpn, sd) {
		err = write_port_dfid_map(sd, lpn);
		if (err)
			return err;
	}

	return 0;
}

static int switchinfo_write(struct fsubdev *sd)
{
	struct mbdb_op_switchinfo si = sd->switchinfo;
	int err;

	si.lft_top = ROUTING_UFT_SIZE - 1;
	err = ops_switchinfo_set(sd, &si, &si, false);
	if (err) {
		sd_err(sd, "failed to set switchinfo: %d\n", err);
		return err;
	}

	sd->switchinfo = si;

	return 0;
}

static void io_work_fn(void *ctx)
{
	struct fsubdev *sd = ctx;
	int err;

	err = rpipe_write(sd);
	if (err)
		goto err;

	err = bridge_write(sd);
	if (err)
		goto err;

	routing_update(sd);
	return;

err:
	routing_sd_transition_error(sd);
}

int routing_io_run(void)
{
	struct routing_pause_ctx *quiesce_ctx;
	struct par_group group;
	struct fsubdev *sd;

	quiesce_ctx = routing_pause_init();
	if (!quiesce_ctx) {
		pr_err("unable to initialize quiesce context; abandoning fabric programming\n");
		return -EIO;
	}

	routing_pause_start(quiesce_ctx);

	par_start(&group);

	for (sd = routing_sd_iter(0); sd; sd = routing_sd_next(sd, 0))
		par_work_queue(&group, io_work_fn, sd);

	par_wait(&group);

	routing_pause_end(quiesce_ctx);

	return 0;
}

/**
 * routing_io_sd_once - Performs one-time programming of invariant data.
 * @sd: the subdevice to initialize
 *
 * Return: 0 on success, non-zero otherwise.
 *
 * The specified subdevice must not yet be on the routable list, or routing
 * must be blocked via the routing lock.
 */
int routing_io_sd_once(struct fsubdev *sd)
{
	int err;
	u8 lpn;

	err = switchinfo_write(sd);
	if (err)
		goto fail;

	for_each_bridge_lpn(lpn, sd) {
		err = write_port_fidgen(sd, lpn);
		if (err)
			goto fail;

		err = write_port_pkgaddr(sd, lpn);
		if (err)
			goto fail;

		err = check_port_activation(sd, lpn);
		if (err)
			goto fail;
	}

	return 0;

fail:
	routing_sd_transition_error(sd);
	return err;
}
