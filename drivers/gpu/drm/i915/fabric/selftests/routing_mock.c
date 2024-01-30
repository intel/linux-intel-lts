// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2021 Intel Corporation.
 *
 */

#include "routing_mock.h"
#include "../routing_engine.h"
#include "../routing_logic.h"
#include "../routing_p2p.h"

struct spec {
	u16 num_devices;
	u8 num_subdevs;
};

static void routing_mock_populate_a21_spec(struct spec *spec)
{
	spec->num_devices = 6;
	spec->num_subdevs = 2;
}

static void init_port_maps(struct fsubdev *sd)
{
	u8 lpn;

	for (lpn = PORT_FABRIC_START; lpn <= PORT_FABRIC_END; ++lpn)
		__set_bit(lpn, sd->fport_lpns);

	for (lpn = PORT_BRIDGE_START; lpn <= PORT_BRIDGE_END; ++lpn)
		__set_bit(lpn, sd->bport_lpns);
}

static int routing_mock_sd(struct spec *spec,
			   struct routing_topology *topo,
			   struct fdev *dev, int dev_id,
			   struct fsubdev *sd, int sd_index)
{
	struct portinfo *pi;
	u8 lpn;

	WARN_ON(dev_id >= spec->num_devices);
	WARN_ON(sd_index >= spec->num_subdevs);

	sd->fdev = dev;
	sd->port_cnt = PORT_FABRIC_COUNT;
	sd->guid = (0xffull << 56) | (dev_id << 8) | sd_index;
	sd->switchinfo.guid = sd->guid;
	sd->asic_rev_info = FIELD_PREP(MASK_ARI_PLATFORM, ANR_ARI_PLATFORM)
			  | FIELD_PREP(MASK_ARI_STEP, ANR_ARI_STEP_B0);

	init_port_maps(sd);

	routing_sd_init(sd);

	list_add_tail(&sd->routable_link, &routable_list);

	for (lpn = 1; lpn <= PORT_FABRIC_COUNT; ++lpn) {
		pi = &sd->portinfo_op.per_portinfo[lpn];
		pi->link_speed_active = LINK_SPEED_90G;
		pi->link_width_active = LINK_WIDTH_4X;
		pi->link_width_downgrade_rx_active = LINK_WIDTH_4X;
		pi->link_width_downgrade_tx_active = LINK_WIDTH_4X;

		sd->port[lpn].sd = sd;
		sd->port[lpn].lpn = lpn;
		sd->port[lpn].portinfo = pi;
		set_bit(PORT_CONTROL_ROUTABLE, sd->port[lpn].controls);
		if (lpn < spec->num_devices) {
			/* initialize first N-1 ports as ISLs */
			sd->port[lpn].state = PM_PORT_STATE_ACTIVE;
			sd->port[lpn].phys_state = IAF_FW_PORT_PHYS_LINKUP;
			sd->port[lpn].log_state = IAF_FW_PORT_ACTIVE;
		} else {
			/* rest of ports are offline */
			sd->port[lpn].state = PM_PORT_STATE_INACTIVE;
			sd->port[lpn].phys_state = IAF_FW_PORT_PHYS_DISABLED;
			sd->port[lpn].log_state = IAF_FW_PORT_DOWN;
		}
	}

	return 0;
}

static int routing_mock_device(struct spec *spec,
			       struct routing_topology *topo,
			       struct fdev *dev, int dev_id)
{
	int sd_idx;
	int err;

	WARN_ON(dev_id >= spec->num_devices);

	for (sd_idx = 0; sd_idx < spec->num_subdevs; ++sd_idx) {
		err = routing_mock_sd(spec, topo, dev, dev_id,
				      &dev->sd[sd_idx], sd_idx);
		if (err)
			return err;
	}

	return 0;
}

struct port_vec {
	struct fdev *dev;
	int sd;
	u8 lpn;
};

static void routing_mock_interconnect_port(struct spec *spec,
					   struct routing_topology *topo,
					   struct port_vec *src)
{
	struct port_vec dst;
	struct portinfo *pi_src;

	/*
	 * target device is chosen by contiguous assignment toward increasing
	 * indices (port 1 to src.d+1, port 2 to src.d+2, etc.)
	 */
	dst.dev = fdev_find((src->dev->pd->index + src->lpn) %
			    spec->num_devices);

	/* target sd is always the same as the source sd */
	dst.sd = src->sd;

	/*
	 * target port uses the same assignment rule, which ends up counting
	 * backwards relative to the source (1->5, 2->4, 3->3, etc.)
	 */
	dst.lpn = spec->num_devices - src->lpn;

	pi_src = src->dev->sd[src->sd].port[src->lpn].portinfo;
	pi_src->neighbor_guid = dst.dev->sd[dst.sd].switchinfo.guid;
	pi_src->neighbor_port_number = dst.lpn;

	fdev_put(dst.dev);
}

/*
 * Connects devices in an all-to-all topology.
 *
 * Each individual device assigns outbound port 1 to their first neighbor
 * to the right, then port 2 to their second neighbor to the right, and
 * so on.
 *
 * So given N devices, the jth port of the ith device goes to device
 * ((i + j) % N).
 */
static void routing_mock_interconnect(struct spec *spec,
				      struct routing_topology *topo)
{
	struct port_vec src;
	int dev_id;

	for (dev_id = 0; dev_id < spec->num_devices; ++dev_id) {
		src.dev = fdev_find(dev_id);
		for (src.sd = 0; src.sd < spec->num_subdevs; ++src.sd)
			for (src.lpn = PORT_FABRIC_START;
			     src.lpn < spec->num_devices; ++src.lpn)
				routing_mock_interconnect_port(spec, topo,
							       &src);
		fdev_put(src.dev);
	}
}

static struct iaf_pdata *routing_mock_pd_create(struct spec *spec, int dev_id)
{
	struct iaf_pdata *pd;
	u64 dpa_per_sd;

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return ERR_PTR(-ENOMEM);

	pd->index = dev_id;
	pd->sd_cnt = spec->num_subdevs;

	/* initially determine maximum DPA per SD... */
	dpa_per_sd = ROUTING_DEV_DPA_ALIGNMENT / pd->sd_cnt;

	/*
	 * ...then use as much memory as available to exercise logic for
	 * multiple dpa->fid lut entries, but reduced by one so that we also
	 * have to deal with holes
	 */
	dpa_per_sd = max(ROUTING_MIN_DPA_PER_SD,
			 dpa_per_sd - ROUTING_MIN_DPA_PER_SD);

	pd->dpa.pkg_offset = dev_id * ROUTING_DEV_DPA_ALIGNMENT / SZ_1G;
	pd->dpa.pkg_size = dpa_per_sd * pd->sd_cnt / SZ_1G;

	return pd;
}

int routing_mock_create_topology(struct routing_topology *topo)
{
	struct spec spec;
	struct fdev *dev;
	int dev_id;
	int err;

	routing_mock_populate_a21_spec(&spec);

	if (spec.num_devices > ROUTING_MAX_DEVICES) {
		pr_err("%s: invalid topology spec: devices exceeds limit: %u > %u",
		       __func__, spec.num_devices, ROUTING_MAX_DEVICES);
		return -EINVAL;
	}

	memset(topo, 0, sizeof(*topo));
	INIT_LIST_HEAD(&topo->plane_list);

	for (dev_id = 0; dev_id < spec.num_devices; ++dev_id) {
		dev = kzalloc(sizeof(*dev), GFP_KERNEL);
		if (!dev)
			return -ENOMEM;

		dev->pd = routing_mock_pd_create(&spec, dev_id);
		if (IS_ERR(dev->pd)) {
			err = PTR_ERR(dev->pd);
			kfree(dev);
			return err;
		}

		err = routing_mock_device(&spec, topo, dev, dev_id);
		if (err) {
			kfree(dev->pd);
			kfree(dev);
			return err;
		}

		dev->fabric_id = dev->pd->index;
		err = fdev_insert(dev);
		if (err) {
			kfree(dev->pd);
			kfree(dev);
			return err;
		}
	}

	routing_mock_interconnect(&spec, topo);

	return 0;
}

static int routing_mock_destroy_cb(struct fdev *dev, void *args)
{
	int i;
	struct xarray *fdevs_to_free = args;

	for (i = 0; i < dev->pd->sd_cnt; ++i)
		routing_sd_destroy(dev->sd + i);

	routing_p2p_clear(dev);

	if (xa_insert(fdevs_to_free, dev->fabric_id, dev, GFP_KERNEL))
		pr_warn("fabric_id 0x%08x already in use\n", dev->fabric_id);

	return 0;
}

void routing_mock_destroy(struct routing_topology *topo)
{
	struct routing_plane *plane, *plane_tmp;
	struct xarray fdevs_to_free;
	struct fdev *dev;
	unsigned long i;
	int refcount;

	xa_init(&fdevs_to_free);

	fdev_process_each(routing_mock_destroy_cb, &fdevs_to_free);

	xa_for_each(&fdevs_to_free, i, dev) {
		for (refcount = refcount_read(&dev->refs.refcount); refcount;
		     refcount = refcount_read(&dev->refs.refcount)) {
			if (refcount != 1)
				pr_warn("fabric_id 0x%08x refcount %d\n", dev->fabric_id, refcount);
			fdev_put(dev);
		}

		kfree(dev->pd);
		kfree(dev);
	}

	xa_destroy(&fdevs_to_free);

	list_for_each_entry_safe(plane, plane_tmp, &topo->plane_list, topo_link)
		routing_plane_destroy(plane);
}
