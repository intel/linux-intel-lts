// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2020 - 2022 Intel Corporation.
 *
 */

#include <linux/seq_file.h>

#include "iaf_drv.h"
#include "ops.h"
#include "routing_debug.h"
#include "routing_topology.h"

#define BLOCK_UNINITIALIZED (~0ul)

static struct fport *find_neighbor_by_lpn(struct fsubdev *sd, u8 lpn)
{
	struct fport *port;
	struct fsubdev *nsd;

	port = get_fport_handle(sd, lpn);
	if (!port)
		return NULL;

	if (!port->portinfo->neighbor_guid || !port->portinfo->neighbor_port_number)
		return NULL;

	nsd = find_routable_sd(port->portinfo->neighbor_guid);
	if (!nsd)
		return NULL;

	return get_fport_handle(nsd, port->portinfo->neighbor_port_number);
}

struct uft_iter {
	unsigned long block;
};

static void uft_iter_end(struct uft_iter *iter)
{
	up_read(&routable_lock);
	kfree(iter);
}

static void *uft_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct fport *port = s->private;
	struct uft_iter *iter = v;
	void *elem;

	if (WARN_ON(IS_ERR_OR_NULL(iter)))
		return iter;

	if (iter->block == BLOCK_UNINITIALIZED) {
		iter->block = 0;
		elem = xa_find(&port->sd->routing.uft->bridges, &iter->block,
			       ROUTING_DPA_DFID_MAP_SIZE, XA_PRESENT);
	} else {
		elem = xa_find_after(&port->sd->routing.uft->bridges, &iter->block,
				     ROUTING_DPA_DFID_MAP_SIZE, XA_PRESENT);
	}

	if (!elem) {
		uft_iter_end(iter);
		iter = NULL;
	}

	if (pos)
		(*pos)++;

	return iter;
}

static void *uft_seq_to_pos(struct seq_file *s, void *v, const loff_t *pos)
{
	loff_t i = *pos;

	while (i--) {
		v = uft_seq_next(s, v, NULL);
		if (!v)
			return NULL;
	}

	return v;
}

static void *uft_seq_start(struct seq_file *s, loff_t *pos)
{
	struct fport *port = s->private;
	struct uft_iter *iter;

	if (!port->sd->routing.uft)
		return NULL; /* eof */

	iter = kmalloc(sizeof(*iter), GFP_KERNEL);
	if (!iter)
		return ERR_PTR(-ENOMEM);

	iter->block = BLOCK_UNINITIALIZED;

	down_read(&routable_lock); /* shared lock */

	/*
	 * from this point on, we must either next() to a final state, or
	 * stop(), in order to clean up resources (via end())
	 */

	iter = uft_seq_to_pos(s, iter, pos);
	if (!iter)
		return NULL; /* eof */

	return iter;
}

static void uft_seq_stop(struct seq_file *s, void *v)
{
	struct uft_iter *iter = v;

	if (!IS_ERR_OR_NULL(v))
		uft_iter_end(iter);
}

struct uft_seq_show_ctx {
	struct seq_file *s;
	struct fport *port;
	struct uft_iter *iter;
	struct mbdb_op_rpipe_get *rpipe;
	u32 num_entries;
	u32 len_rpipe;
};

static int uft_seq_show_init(struct uft_seq_show_ctx *ctx)
{
	/* allocate space for an rpipe for every port */
	if (ctx->iter->block == BLOCK_UNINITIALIZED) {
		/* first time through; print the mgmt block */
		ctx->num_entries = ROUTING_MAX_DEVICES;
		ctx->len_rpipe = sizeof(struct mbdb_op_rpipe_get) + ctx->num_entries;
	} else {
		/* after the mgmt block; print the nth bridge block */
		ctx->num_entries = ROUTING_FID_BLOCK_SIZE;
		ctx->len_rpipe = sizeof(struct mbdb_op_rpipe_get) + ctx->num_entries;
	}

	ctx->rpipe = kzalloc(sizeof(*ctx->rpipe) + ctx->num_entries, GFP_KERNEL);
	if (!ctx->rpipe)
		return -ENOMEM;

	return 0;
}

static int uft_seq_show_query(struct uft_seq_show_ctx *ctx)
{
	int err;

	if (ctx->iter->block == BLOCK_UNINITIALIZED) {
		/* first time through; print the mgmt block */
		ctx->rpipe->port_number = ctx->port->lpn;
		ctx->rpipe->start_index = ROUTING_FID_CPORT_BASE;
		ctx->rpipe->num_entries = ctx->num_entries;
	} else {
		/* after the mgmt block; print the nth bridge block */
		ctx->rpipe->port_number = ctx->port->lpn;
		ctx->rpipe->start_index = ROUTING_FID_BLOCK_BASE
					+ ctx->iter->block * ROUTING_FID_BLOCK_SIZE;
		ctx->rpipe->num_entries = ctx->num_entries;
	}

	err = ops_rpipe_get(ctx->port->sd, ctx->rpipe, ctx->rpipe);
	if (err)
		return err;

	return 0;
}

static void uft_seq_show_render(struct uft_seq_show_ctx *ctx)
{
	struct fsubdev *nsd;
	struct fport *nport;
	u16 fid_offset;
	u8 olpn;
	u8 i;

	/* print each dfid on one line */
	for (fid_offset = 0; fid_offset < ctx->num_entries; ++fid_offset) {
		/* don't print unrouted dfids */
		olpn = ctx->rpipe->rpipe_data[fid_offset];
		if (olpn == ROUTING_UFT_INVALID_PORT8)
			continue;

		/* print dfid row header */
		seq_printf(ctx->s, "0x%04x %02u ", ctx->rpipe->start_index + fid_offset, olpn);

		nport = find_neighbor_by_lpn(ctx->port->sd, olpn);
		if (nport) {
			/* case: external neighbor */
			seq_printf(ctx->s, "%03u.%01u.%02u",
				   nport->sd->fdev->pd->index, sd_index(nport->sd), nport->lpn);
		} else if (olpn == 0) {
			/* case: cport */
			seq_printf(ctx->s, "%03u.%01u.cp",
				   ctx->port->sd->fdev->pd->index, sd_index(ctx->port->sd));
		} else if (olpn > PORT_FABRIC_COUNT) {
			/* case: bridge: try to report the particular tile owning the dfid */
			nsd = NULL;
			for (i = 0; i < ctx->port->sd->fdev->pd->sd_cnt; ++i) {
				if (ctx->port->sd->routing.fid_base == ctx->rpipe->start_index) {
					nsd = ctx->port->sd;
					break;
				}
			}
			if (nsd) {
				/* case: local package ("base die") */
				seq_printf(ctx->s, "%03u.%01u.bd",
					   nsd->fdev->pd->index, sd_index(nsd));
			} else {
				/* case: no owning tile? at least report the dev */
				seq_printf(ctx->s, "%03u.-.bd",
					   ctx->port->sd->fdev->pd->index);
			}
		} else {
			/* case: invalid */
			seq_puts(ctx->s, "---.-.--");
		}

		seq_puts(ctx->s, "\n");
	}
}

static int uft_seq_show(struct seq_file *s, void *v)
{
	struct uft_seq_show_ctx ctx = {};
	struct fport *port = s->private;
	int err;

	if (WARN_ON(!v || !port))
		return -EINVAL;

	/* no warn here; just means the read came in a bit early */
	if (!port->sd)
		return -EINVAL;

	ctx.s = s;
	ctx.port = port;
	ctx.iter = v;

	err = uft_seq_show_init(&ctx);
	if (err)
		goto end;

	err = uft_seq_show_query(&ctx);
	if (err)
		goto end;

	uft_seq_show_render(&ctx);

end:
	kfree(ctx.rpipe);
	return err;
}

static const struct seq_operations uft_seq_ops = {
	.start = uft_seq_start,
	.next = uft_seq_next,
	.stop = uft_seq_stop,
	.show = uft_seq_show,
};

static int uft_open(struct inode *inode, struct file *file)
{
	struct fport *port = WARN_ON(!inode) ? NULL : inode->i_private;
	struct seq_file *s;
	int err;

	if (WARN_ON(!port))
		return -EINVAL;

	err = seq_open(file, &uft_seq_ops);
	if (err)
		return err;

	s = file->private_data;
	if (WARN_ON(!s)) {
		seq_release(inode, file);
		return -EINVAL;
	}

	s->private = port;
	return 0;
}

/*
 * outputs the unicast forwarding table with columns:
 *   1. destination FID
 *   2. output port
 *   3. <dev_idx>.<subdev_idx>.<lpn> of the neighbor of this entry
 *        - if the neighbor is over a fabric link: lpn will be the neighbor port
 *        - if the neighbor is the cport: lpn will be "cp"
 *        - if the neighbor is the base die: lpn will be "bd"
 */
static const struct file_operations uft_ops = {
	.open = uft_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

struct dpa_iter {
	struct fsubdev *sd_dst;
};

static void dpa_iter_end(struct dpa_iter *iter)
{
	up_read(&routable_lock);
	kfree(iter);
}

static void *dpa_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct dpa_iter *iter = v;

	if (WARN_ON(IS_ERR_OR_NULL(iter)))
		return iter;

	iter->sd_dst = routing_sd_next(iter->sd_dst, 0);
	if (!iter->sd_dst) {
		dpa_iter_end(iter);
		iter = NULL;
	}

	if (pos)
		(*pos)++;

	return iter;
}

static void *dpa_seq_to_pos(struct seq_file *s, void *v, const loff_t *pos)
{
	loff_t i = *pos;

	while (i--) {
		v = dpa_seq_next(s, v, NULL);
		if (!v)
			return NULL;
	}

	return v;
}

static void *dpa_seq_start(struct seq_file *s, loff_t *pos)
{
	struct dpa_iter *iter;

	iter = kmalloc(sizeof(*iter), GFP_KERNEL);
	if (!iter)
		return ERR_PTR(-ENOMEM);

	iter->sd_dst = routing_sd_iter(0);
	if (!iter->sd_dst) {
		kfree(iter);
		return ERR_PTR(-EINVAL);
	}

	down_read(&routable_lock); /* shared lock */

	/*
	 * from this point on, we must either next() to a final state, or
	 * stop(), in order to clean up resources (via end())
	 */

	iter = dpa_seq_to_pos(s, iter, pos);
	if (!iter)
		return NULL; /* eof */

	return iter;
}

static void dpa_seq_stop(struct seq_file *s, void *v)
{
	struct dpa_iter *iter = v;

	if (!IS_ERR_OR_NULL(v))
		dpa_iter_end(iter);
}

static int dpa_seq_show(struct seq_file *s, void *v)
{
	struct dpa_iter *iter = v;
	struct fport *port = s->private;
	size_t size = ROUTING_DEV_DPA_ALIGNMENT / ROUTING_MIN_DPA_PER_SD / 2 * sizeof(u64);
	u64 *map;
	u32 addr;
	u32 len;
	u64 dpa;
	u16 dfid;
	u16 i;
	int err;

	if (WARN_ON(!iter || !port))
		return -EINVAL;

	/* no warn here; just means the read came in a bit early */
	if (!port->sd)
		return -EINVAL;

	map = kzalloc(size, GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	addr = CSR_FIDGEN_LUT_BASE + iter->sd_dst->routing.dpa_idx_base * sizeof(u64);
	len = iter->sd_dst->routing.dpa_idx_range * sizeof(u64);

	err = ops_linkmgr_port_csr_rd(port->sd, port->lpn, addr, len, map);
	if (err) {
		kfree(map);
		return err;
	}

	for (i = 0; i < iter->sd_dst->routing.dpa_idx_range; ++i) {
		dpa = (iter->sd_dst->routing.dpa_idx_base + i) * ROUTING_MIN_DPA_PER_SD;
		dfid = (map[i] << ROUTING_DPA_DFID_MAP_SHIFT) & 0xffff;
		seq_printf(s, "0x%016llx 0x%04x %03u.%01u",
			   dpa, dfid, iter->sd_dst->fdev->pd->index,
			   sd_index(iter->sd_dst));

		seq_puts(s, "\n");
	}

	kfree(map);
	return 0;
}

static const struct seq_operations dpa_seq_ops = {
	.start = dpa_seq_start,
	.next = dpa_seq_next,
	.stop = dpa_seq_stop,
	.show = dpa_seq_show,
};

static int dpa_open(struct inode *inode, struct file *file)
{
	struct fport *port = WARN_ON(!inode) ? NULL : inode->i_private;
	struct seq_file *s;
	int err;

	if (WARN_ON(!port))
		return -EINVAL;

	err = seq_open(file, &dpa_seq_ops);
	if (err)
		return err;

	s = file->private_data;
	if (WARN_ON(!s)) {
		seq_release(inode, file);
		return -EINVAL;
	}

	s->private = port;
	return 0;
}

/*
 * outputs the dpa to fid map with columns:
 *   1. full byte-addressed device physical address
 *   2. destination FID
 *   3. <dev_idx>.<subdev_idx> of the sd the driver associates with this entry
 */
static const struct file_operations dpa_ops = {
	.open = dpa_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

void routing_debug_port_init(struct fsubdev *sd, u8 lpn)
{
	struct fport *port = &sd->port[lpn];

	debugfs_create_file("uft", 0400, sd->debugfs_port_dir, port, &uft_ops);

	if (test_bit(lpn, sd->bport_lpns))
		debugfs_create_file("dpa", 0400, sd->debugfs_port_dir, port, &dpa_ops);
}
