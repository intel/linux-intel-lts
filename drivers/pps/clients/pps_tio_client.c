// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel PPS TIO Client Driver
 *
 * Copyright (C) 2026 Intel Corporation
 */

#include <linux/auxiliary_bus.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pps_kernel.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>

#include "pps_tio_plat.h"

#define INTERVAL_MS	ms_to_ktime(100)

struct pps_tio_client {
	struct auxiliary_device *aux_dev;
	struct pps_source_info client_info;
	struct pps_tio_plat_data *pdata;
	struct pps_device *pps;
	spinlock_t client_lock; /* client lock resource */
	struct hrtimer timer;
	u64 prev_count;
};

static u64 prev_art;
static struct system_time_snapshot cur_snap, prev_snap;

static inline ktime_t first_event(struct pps_tio_client *tio_cli)
{
	return ktime_add(ktime_get_real(), INTERVAL_MS);
}

static int pps_tio_get_time(ktime_t *device,
			    struct system_counterval_t *system, void *ctx)
{
	u64 *art_cycles = ctx;

	*device = *art_cycles;
	system->cycles = *art_cycles;
	system->cs_id  = CSID_X86_ART;

	return 0;
}

static int pps_tio_convert_cs(struct pps_tio_client *tio_cli, u64 art_cycles,
			      struct system_time_snapshot snap)
{
	struct system_device_crosststamp xts = { };
	struct pps_event_time ets = { };
	int ret;

	ret = get_device_system_crosststamp(pps_tio_get_time,
					    &art_cycles, &snap, &xts);

	if (ret)
		return ret;

	ets.ts_real = ktime_to_timespec64(xts.sys_realtime);
	pps_event(tio_cli->pps, &ets, PPS_CAPTUREASSERT, NULL);

	return 0;
}

/*
 *  To co-relate/convert art to system time, we need to read
 *  device art cycles and system time snapshot @ same time.
 *  Due to polling we are ahead of time in reading art cycles.
 *  we need X-1 snapshot time to interpolate in croststamp api.
 */
static enum hrtimer_restart hrtimer_callback(struct hrtimer *timer)
{
	struct pps_tio_client *tio_cli = container_of(timer,
					     struct pps_tio_client, timer);
	struct auxiliary_device *aux_dev = tio_cli->aux_dev;
	struct pps_tio_ops_req req;
	u64 cur_art_cycles, ec_cap;
	int ret;

	guard(spinlock)(&tio_cli->client_lock);

	ktime_get_snapshot(&cur_snap);

	/* capture device art cycles */
	req = PPS_TIO_OPS_REQ(READ_REG, tio_cli->pdata->tio_data->regs.tcv, 0, 0);
	ret = pps_ops_req(tio_cli->pdata, &req);
	if (ret < 0) {
		dev_err(&aux_dev->dev, "TCV Read Error %d", ret);
		goto err;
	}

	cur_art_cycles = req.value;

	req = PPS_TIO_OPS_REQ(READ_REG, tio_cli->pdata->tio_data->regs.eccv, 0, 0);
	ret = pps_ops_req(tio_cli->pdata, &req);
	if (ret < 0) {
		dev_err(&aux_dev->dev, "ECCV Read Error %d", ret);
		goto err;
	}

	ec_cap = req.value;

	if (cur_art_cycles != prev_art)
		pps_tio_convert_cs(tio_cli, cur_art_cycles, prev_snap);

	tio_cli->prev_count = ec_cap;
	prev_art = cur_art_cycles;
	memcpy(&prev_snap, &cur_snap, sizeof(cur_snap));

	hrtimer_forward_now(timer, INTERVAL_MS);
	return HRTIMER_RESTART;
err:
	req = PPS_TIO_OPS_REQ(DISABLE, 0, 0, 0);
	pps_ops_req(tio_cli->pdata, &req);
	tio_cli->pps->is_poll_enabled = false;
	tio_cli->prev_count = 0;
	return HRTIMER_NORESTART;
}

static int pps_tio_client_poll(struct pps_device *pps, bool enable)
{
	struct pps_tio_client *tio_cli = container_of(pps->info,
					     struct pps_tio_client, client_info);
	struct auxiliary_device *aux_dev = tio_cli->aux_dev;
	struct pps_tio_ops_req req;
	int ret;

	guard(spinlock)(&tio_cli->client_lock);

	if (enable && !pps->is_poll_enabled) {
		tio_cli->pps->is_poll_enabled = false;
		tio_cli->prev_count = 0;
		req = PPS_TIO_OPS_REQ(CONFIG, 0, 0, TIO_CLIENT);
		ret = pps_ops_req(tio_cli->pdata, &req);
		if (ret < 0) {
			dev_err(&aux_dev->dev, "client functionality not enabled\n");
			return ret;
		}
		tio_cli->pps->is_poll_enabled = true;
		hrtimer_start(&tio_cli->timer, first_event(tio_cli),
			      HRTIMER_MODE_ABS_PINNED);
	} else if (!enable && pps->is_poll_enabled) {
		hrtimer_cancel(&tio_cli->timer);
		req = PPS_TIO_OPS_REQ(DISABLE, 0, 0, 0);
		pps_ops_req(tio_cli->pdata, &req);
		tio_cli->pps->is_poll_enabled = false;
		tio_cli->prev_count = 0;
	}

	return 0;
}

static int pps_tio_client_probe(struct auxiliary_device *aux_dev,
				const struct auxiliary_device_id *id)
{
	struct pps_tio_client *tio_cli;
	struct pps_tio_ops_req req;

	tio_cli = devm_kzalloc(&aux_dev->dev, sizeof(*tio_cli), GFP_KERNEL);
	if (!tio_cli)
		return -ENOMEM;

	dev_set_drvdata(&aux_dev->dev, tio_cli);

	tio_cli->aux_dev = aux_dev;

	tio_cli->client_info.mode = PPS_CAPTUREASSERT | PPS_CANWAIT |
				       PPS_CANPOLL | PPS_TSFMT_TSPEC;
	tio_cli->client_info.owner = THIS_MODULE;
	tio_cli->client_info.enable_poll = pps_tio_client_poll;
	strscpy(tio_cli->client_info.name, "intel_tio_client",
		sizeof(tio_cli->client_info.name));
	tio_cli->pdata = dev_get_platdata(&aux_dev->dev);

	tio_cli->pps = pps_register_source(&tio_cli->client_info,
					   tio_cli->client_info.mode);

	if (IS_ERR(tio_cli->pps))
		return PTR_ERR(tio_cli->pps);

	spin_lock_init(&tio_cli->client_lock);

	req = PPS_TIO_OPS_REQ(DISABLE, 0, 0, 0);
	pps_ops_req(tio_cli->pdata, &req);

	hrtimer_setup(&tio_cli->timer, hrtimer_callback, CLOCK_REALTIME,
		      HRTIMER_MODE_ABS_PINNED);

	return 0;
}

static void pps_tio_client_remove(struct auxiliary_device *aux_dev)
{
	struct pps_tio_client *tio_cli;
	struct pps_tio_ops_req req;

	tio_cli = dev_get_drvdata(&aux_dev->dev);

	hrtimer_cancel(&tio_cli->timer);

	req = PPS_TIO_OPS_REQ(DISABLE, 0, 0, 0);
	pps_ops_req(tio_cli->pdata, &req);
	tio_cli->pps->is_poll_enabled = false;

	pps_unregister_source(tio_cli->pps);
}

static const struct auxiliary_device_id pps_tio_client_id[] = {
	{ .name = "pps_tio_plat.client" },
	{ },
};
MODULE_DEVICE_TABLE(auxiliary, pps_tio_client_id);

static struct auxiliary_driver pps_tio_client = {
	.name = "client",
	.probe = pps_tio_client_probe,
	.remove = pps_tio_client_remove,
	.id_table = pps_tio_client_id,
};

module_auxiliary_driver(pps_tio_client);

MODULE_AUTHOR("Christopher Hall <christopher.s.hall@intel.com>");
MODULE_AUTHOR("Subramanian Mohan <subramanian.mohan@intel.com>");
MODULE_DESCRIPTION("Intel PMC Time-Aware IO Client Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("PPS_TIO");
