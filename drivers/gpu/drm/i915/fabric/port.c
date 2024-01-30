// SPDX-License-Identifier: MIT
/*
 * Copyright(c) 2019 - 2023 Intel Corporation.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/dcache.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/timekeeping.h>

#include "debugfs.h"
#include "fw.h"
#include "port.h"
#include "port_diag.h"
#include "ops.h"
#include "routing_debug.h"
#include "routing_engine.h"
#include "routing_event.h"
#include "trace.h"

/*
 * Presence detection types used in &struct psc_presence_rule.method
 */
enum {
	PRESENCE_RULE_DEFAULT = 0,
	PRESENCE_RULE_LINK_CONTROL
};

/*
 * Port initialization timeout
 */
static unsigned short linkup_timeout = 5 * 60;
module_param(linkup_timeout, ushort, 0400);
MODULE_PARM_DESC(linkup_timeout, "Seconds to wait for port link up (default: 5 minutes)");

static void start_linkup_timer(struct fsubdev *sd, struct fport *p)
{
	mutex_lock(&sd->pm_work_lock);
	if (sd->ok_to_schedule_pm_work)
		mod_timer(&p->linkup_timer, jiffies + linkup_timeout * HZ);
	/* else we are shutting down so don't time out linkup */
	mutex_unlock(&sd->pm_work_lock);
}

s64 ms_elapsed_since_boot(void)
{
	return ktime_to_ms(ktime_get_boottime());
}

static const char *state_name(enum pm_port_state s)
{
	switch (s) {
	case PM_PORT_STATE_INACTIVE:  return "INACTIVE";
	case PM_PORT_STATE_REQUESTED: return "REQUESTED";
	case PM_PORT_STATE_ISOLATED:  return "ISOLATED";
	case PM_PORT_STATE_ARMED:     return "ARMED";
	case PM_PORT_STATE_ACTIVE:    return "ACTIVE";
	}

	return "UNKNOWN";
}

const char *fw_log_state_name(u8 s)
{
	static const char * const names[] = { "NOP", "DOWN", "INIT", "ARMED",
					      "ACTIVE" };

	if (s >= ARRAY_SIZE(names))
		return "UNKNOWN";
	else
		return names[s];
}

#define UEVENT_DELAY (HZ)

static bool param_reset = true;
module_param_named(reset, param_reset, bool, 0600);
MODULE_PARM_DESC(reset, "Disable to prevent resetting devices on shutdown (default: Y)");

/**
 * struct fsm_io - Port FSM I/O
 * @deisolating: input - are we currently deisolating ports?
 * @request_deisolation: output - should PM request all ports be deisolated?
 * @routing_changed: output - should PM notify routing event manager?
 * @status_changed: output - has an event potentially changed sd->port_status?
 *
 * I/O from FSM updates, used to exchange requests with update_ports_work().
 * Before each FSM update, deisolating is set to indicate if a DEISOLATE_EVENT
 * was requested and the outputs are set to false. The former indicates whether
 * ports should be deisolated. The outputs are used to request followup
 * actions.
 */
struct fsm_io {
	bool deisolating;
	bool request_deisolation;
	bool routing_changed;
	bool status_changed;
};

/*
 * This wakes the PM thread
 */
int signal_pm_thread(struct fsubdev *sd, enum pm_trigger_reasons event)
{
	int err = 0;

	set_bit(event, sd->pm_triggers);

	mutex_lock(&sd->pm_work_lock);
	if (sd->ok_to_schedule_pm_work)
		queue_work(iaf_unbound_wq, &sd->pm_work); /* implies a full memory barrier */
	else
		err = -EAGAIN;
	mutex_unlock(&sd->pm_work_lock);

	return err;
}

/*
 * number of isolated ports in the entire system
 */
static atomic_t isolated_port_cnt = ATOMIC_INIT(0);

static void deisolate_all_ports(void)
{
	if (atomic_read(&isolated_port_cnt) > 0) {
		struct fsubdev *sd;

		list_for_each_entry(sd, &routable_list, routable_link)
			signal_pm_thread(sd, DEISOLATE_EVENT); /* ignore (per-device) errors */
	}
}

static bool port_manual_bringup;
module_param(port_manual_bringup, bool, 0600);
MODULE_PARM_DESC(port_manual_bringup,
		 "Disable automatic port bringup (default: N)");

/*
 * These will be useful at least during early hardware validation.
 */

static bool port_enable_loopback = true;
module_param_named(enable_loopback, port_enable_loopback, bool, 0400);
MODULE_PARM_DESC(enable_loopback, "Enable loopback fabric configurations (default: Y)");

/*
 * Helpers to update port health
 */

static void set_health_off(struct fport_status *status)
{
	status->health = FPORT_HEALTH_OFF;
	bitmap_zero(status->issues, NUM_FPORT_ISSUES);
	bitmap_zero(status->errors, NUM_FPORT_ERRORS);
}

static void set_health_failed(struct fport_status *status, enum fport_error error_reason)
{
	if (status->health != FPORT_HEALTH_FAILED && status->health != FPORT_HEALTH_DEGRADED)
		status->link_degrades++;
	if (status->health != FPORT_HEALTH_FAILED)
		status->link_failures++;
	status->health = FPORT_HEALTH_FAILED;
	bitmap_zero(status->issues, NUM_FPORT_ISSUES);
	set_bit(error_reason, status->errors);
}

static void set_health_degraded(struct fport_status *status, enum fport_issue issue)
{
	if (status->health != FPORT_HEALTH_FAILED && status->health != FPORT_HEALTH_DEGRADED)
		status->link_degrades++;
	status->health = FPORT_HEALTH_DEGRADED;
	set_bit(issue, status->issues);
	bitmap_zero(status->errors, NUM_FPORT_ERRORS);
}

static void set_health_healthy(struct fport_status *status)
{
	status->health = FPORT_HEALTH_HEALTHY;
	bitmap_zero(status->issues, NUM_FPORT_ISSUES);
	bitmap_zero(status->errors, NUM_FPORT_ERRORS);
}

static void clear_status_linkup_detected(struct fport_status *status)
{
	status->linkup_since = 0;
}

static void clear_status_link_failures(struct fport_status *status)
{
	status->link_failures = 0;
}

static void clear_status_link_degrades(struct fport_status *status)
{
	status->link_degrades = 0;
}

/*
 * Flapping period parameters
 */

static unsigned short link_flapping_leaky_bucket_period = 5 * 60;
module_param(link_flapping_leaky_bucket_period, ushort, 0600);
MODULE_PARM_DESC(link_flapping_leaky_bucket_period,
		 "Seconds to auto-decrement link bounce count (default: 5 minutes, 0 disables)");

static unsigned short link_flapping_threshold = 5;
module_param(link_flapping_threshold, ushort, 0600);
MODULE_PARM_DESC(link_flapping_threshold,
		 "Bounce count to declare a link flapping (default: 5, 0 disables)");

/**
 * linkup_detected_now - get timestamp and perform timestamp-based checks when linkup is detected
 * @sd: subdevice containing port
 * @p: port being updated/checked
 *
 * Indicate that a linkup detected was just detected and maintain a leaky bucket counter of port
 * bounces. Time is indicated as ms elapsed since boot. The leaky bucket models this behavior:
 * - while the leaky bucket is active, the bounce count is decremented every bucket period
 * - the leaky bucket becomes inactive one period after the bounce count reaches zero
 * - if the leaky bucket is considered active now, the bounce count is incremented
 * - if the leaky bucket is considered not active, it is started now
 * - if the bounce count exceeds the flapping threshold, the port is declared flapping
 * - once a port is flapping, an SMI enable operation is needed to clear its state
 *
 * No timer triggers these events, they are all computed here. Timestamp calculations are used to
 * determine which events occurred and update the model. Note that the bucket end condition is
 * defined such that if a port is flapping every flapping period, the leaky bucket will remain
 * continuously active and the bounce count will remain 1 rather than bouncing between 0 and 1.
 *
 * Return: true if the port is OK, false if it is considered flapping
 */
static bool linkup_detected_now(struct fsubdev *sd, struct fport *p)
{
	s64 now = ms_elapsed_since_boot();
	unsigned short flapping_threshold;
	s64 flapping_period_ms;

	sd->next_port_status[p->lpn].linkup_since = now;

	kernel_param_lock(THIS_MODULE);
	flapping_threshold = link_flapping_threshold;
	/* s64 is much larger than (unsigned short) link_flapping_leaky_bucket_period */
	flapping_period_ms = (s64)link_flapping_leaky_bucket_period * 1000;
	kernel_param_unlock(THIS_MODULE);

	/* we divide by flapping_period_ms later, so ensure the non-zero check remains */
	if (!flapping_threshold || !flapping_period_ms)
		return true;

	if (test_bit(FPORT_ERROR_FLAPPING, sd->next_port_status[p->lpn].errors))
		return false;

	if (p->flap_check_since && now - p->flap_check_since >= flapping_period_ms) {
		u64 periods_elapsed = (now - p->flap_check_since) / flapping_period_ms;

		if (periods_elapsed > p->bounce_count) {
			p->flap_check_since = 0;
		} else {
			p->flap_check_since += periods_elapsed * flapping_period_ms;
			p->bounce_count -= periods_elapsed;
		}
	}

	if (p->flap_check_since)
		return ++p->bounce_count < flapping_threshold;

	p->flap_check_since = now;
	p->bounce_count = 0;

	return true;
}

static void linkdown_detected(struct fsubdev *sd, struct fport *p)
{
	clear_status_linkup_detected(&sd->next_port_status[p->lpn]);
}

/*
 * FSM Actions
 */

/* PM state and port health are related but there is not a direct mapping.
 *
 * Port health is as follows:
 *
 * FPORT_HEALTH_OFF: port disabled by PSC bin or SMI or in initial training
 * FPORT_HEALTH_FAILED: port in error or went down (PSC trap)
 * FPORT_HEALTH_DEGRADED: port ACTIVE but at reduced width/speed/quality (LWD/LQI trap)
 * FPORT_HEALTH_HEALTHY: port ACTIVE at full width/speed/quality
 *
 * Thus when a port first comes up, it remains OFF until it becomes active at which point
 * it becomes HEALTHY (or DEGRADED if RX/TX width is reduced or LQI < 4: later width/LQI
 * changes can transition between HEALTHY/DEGRADED).
 *
 * Ports transition to FAILED when ISOLATED, timed out, in error, or if they go down
 * for some reason and can be transitioned to/from OFF by port disable/enable requests.
 * Ports may also be marked FAILED if they are flapping (disconnecting and reconnecting
 * repeatedly).
 */

static void port_in_error(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	u32 state;
	int err;

	del_timer(&p->linkup_timer);

	if (p->state == PM_PORT_STATE_ISOLATED)
		atomic_dec(&isolated_port_cnt);

	clear_bit(PORT_CONTROL_ENABLED, p->controls);

	p->state = PM_PORT_STATE_INACTIVE;
	fio->routing_changed = true;

	set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_FAILED);
	fio->status_changed = true;

	state = IAF_FW_PORT_PHYS_DISABLED;

	err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, state, &op_err);

	/* op_err should never be set when DISABLING */
	if (err || op_err) {
		fport_err(p, "failed mailbox error %d %u\n", err, op_err);
	} else {
		/* update local state to be consistent with op on success */
		p->log_state = IAF_FW_PORT_DOWN;
		p->phys_state = IAF_FW_PORT_PHYS_DISABLED;
		linkdown_detected(sd, p);
	}
}

static void port_disable(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	u32 state;
	int err;

	if (p->state == PM_PORT_STATE_ISOLATED)
		atomic_dec(&isolated_port_cnt);

	state = IAF_FW_PORT_PHYS_DISABLED;

	err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, state, &op_err);

	/* op_err should never be set when DISABLING */
	if (err || op_err)
		goto failed;

	/* update local state to be consistent with op on success */
	p->log_state = IAF_FW_PORT_DOWN;
	p->phys_state = IAF_FW_PORT_PHYS_DISABLED;
	linkdown_detected(sd, p);

	p->state = PM_PORT_STATE_INACTIVE;
	fio->routing_changed = true;

	set_health_off(&sd->next_port_status[p->lpn]);
	fio->status_changed = true;

	fport_dbg(p, "port disabled\n");
	return;

failed:
	fport_err(p, "disable request failed\n");
	port_in_error(sd, p, fio);
}

static void port_enable_polling(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	int err;

	if (sd->next_port_status[p->lpn].health == FPORT_HEALTH_FAILED) {
		err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, IAF_FW_PORT_PHYS_DISABLED,
							&op_err);

		/* op_err should never be set when DISABLING */
		if (err || op_err)
			goto failed;

		set_health_off(&sd->next_port_status[p->lpn]);
		fio->status_changed = true;
	}

	err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, IAF_FW_PORT_PHYS_POLLING, &op_err);

	if (err)
		goto failed;

	if (op_err) {
		u32 state = 0;

		/*
		 * op_err could be set if port is somehow already enabled and not OFFLINE -- bounce
		 * the port and give it one more try
		 */
		fport_warn(p, "bouncing port since POLL request failed\n");

		err = ops_linkmgr_ps_get(sd, p->lpn, &state);
		if (err)
			goto failed;

		switch (FIELD_GET(PS_PPS_PHYSICAL_STATE, state)) {
		case IAF_FW_PORT_PHYS_POLLING:
		case IAF_FW_PORT_PHYS_TRAINING:
		case IAF_FW_PORT_PHYS_LINKUP:
		case IAF_FW_PORT_PHYS_LINK_ERROR_RECOVERY:
		case IAF_FW_PORT_PHYS_OFFLINE:
			/* recoverable PHY state during error recovery */
			err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn,
								IAF_FW_PORT_PHYS_DISABLED, &op_err);

			/* op_err should never be set when DISABLING */
			if (err || op_err)
				goto failed;
			break;

		case IAF_FW_PORT_PHYS_DISABLED:
			/* already down, should be able to bring port up now */
			break;

		case IAF_FW_PORT_PHYS_TEST:
		default:
			/* ports should not be able to get here on their own */
			goto failed;
		}

		set_health_off(&sd->next_port_status[p->lpn]);
		fio->status_changed = true;

		err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, IAF_FW_PORT_PHYS_POLLING,
							&op_err);

		/* op_err should not be set now since the port should be disabled */
		if (err || op_err)
			goto failed;
	}

	/* update local state to be consistent with op on success */
	p->log_state = IAF_FW_PORT_DOWN;
	p->phys_state = IAF_FW_PORT_PHYS_POLLING;
	linkdown_detected(sd, p);

	p->state = PM_PORT_STATE_REQUESTED;
	start_linkup_timer(sd, p);
	fio->request_deisolation = true;

	fport_dbg(p, "polling enabled\n");
	return;

failed:
	fport_err(p, "polling request failed, error %d, op_err %d\n", err, op_err);
	port_in_error(sd, p, fio);
}

static void port_isolate(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	bool loopback;
	u32 op_err = 0;
	u32 state;
	int err;

	state = IAF_FW_PORT_PHYS_DISABLED;

	err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, state, &op_err);

	/* op_err should never be set when DISABLING */
	if (err || op_err)
		goto failed;

	/* update local state to be consistent with op on success */
	p->log_state = IAF_FW_PORT_DOWN;
	p->phys_state = IAF_FW_PORT_PHYS_DISABLED;
	linkdown_detected(sd, p);

	p->state = PM_PORT_STATE_ISOLATED;

	set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_ISOLATED);
	fio->status_changed = true;

	atomic_inc(&isolated_port_cnt);

	loopback = test_bit(FPORT_ERROR_LOOPBACK, sd->next_port_status[p->lpn].errors);

	if (is_fport_registered(p))
		fport_warn(p, "isolated: neighbor 0x%016llx port %u%s\n",
			   p->portinfo->neighbor_guid,
			   p->portinfo->neighbor_port_number,
			   loopback ? " (loopback)" : "");
	else
		fport_dbg(p, "isolated: neighbor 0x%016llx port %u%s\n",
			  p->portinfo->neighbor_guid,
			  p->portinfo->neighbor_port_number,
			  loopback ? " (loopback)" : "");

	return;

failed:
	fport_err(p, "isolate request failed\n");
	port_in_error(sd, p, fio);
}

static void port_deisolate(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	int err;

	atomic_dec(&isolated_port_cnt);

	err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, IAF_FW_PORT_PHYS_POLLING, &op_err);

	if (err)
		goto failed;

	if (op_err) {
		u32 state = 0;

		/*
		 * op_err could be set if port is somehow already enabled and not OFFLINE -- bounce
		 * the port and give it one more try
		 */
		fport_warn(p, "bouncing port since POLL request failed\n");

		err = ops_linkmgr_ps_get(sd, p->lpn, &state);
		if (err)
			goto failed;

		switch (FIELD_GET(PS_PPS_PHYSICAL_STATE, state)) {
		case IAF_FW_PORT_PHYS_POLLING:
		case IAF_FW_PORT_PHYS_TRAINING:
		case IAF_FW_PORT_PHYS_LINKUP:
		case IAF_FW_PORT_PHYS_LINK_ERROR_RECOVERY:
		case IAF_FW_PORT_PHYS_OFFLINE:
			/* recoverable PHY state during error recovery */
			err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn,
								IAF_FW_PORT_PHYS_DISABLED, &op_err);

			/* op_err should never be set when DISABLING */
			if (err || op_err)
				goto failed;
			break;

		case IAF_FW_PORT_PHYS_DISABLED:
			/* already down, should be able to bring port up now */
			break;

		case IAF_FW_PORT_PHYS_TEST:
		default:
			/* ports should not be able to get here on their own */
			goto failed;
		}

		set_health_off(&sd->next_port_status[p->lpn]);
		fio->status_changed = true;

		err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, IAF_FW_PORT_PHYS_POLLING,
							&op_err);

		/* op_err should not be set now since the port should be disabled */
		if (err || op_err)
			goto failed;
	}

	/* update local state to be consistent with op on success */
	p->log_state = IAF_FW_PORT_DOWN;
	p->phys_state = IAF_FW_PORT_PHYS_POLLING;
	linkdown_detected(sd, p);

	p->state = PM_PORT_STATE_REQUESTED;
	start_linkup_timer(sd, p);
	fio->status_changed = true;
	fport_dbg(p, "deisolated, polling enabled\n");
	return;

failed:
	fport_err(p, "deisolate request failed, error %d, op_err %d\n", err, op_err);
	port_in_error(sd, p, fio);
}

static void port_did_not_train(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_DID_NOT_TRAIN);
	fio->status_changed = true;
	if (is_fport_registered(p))
		fport_warn(p, "port failed to train in %u seconds\n", linkup_timeout);
	else
		fport_dbg(p, "port failed to train in %u seconds\n", linkup_timeout);
}

static void port_linkup_timedout(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (test_bit(FPORT_ERROR_ISOLATED, sd->next_port_status[p->lpn].errors))
		port_isolate(sd, p, fio);
	else
		port_did_not_train(sd, p, fio);
}

static bool lqi_ok(u8 lqi)
{
	return lqi >= LQI_HEALTHY_THRESHOLD;
}

static void set_active_health(struct fsubdev *sd, struct fport *p)
{
	struct fport_status *status = &sd->next_port_status[p->lpn];
	struct portinfo *pi = p->portinfo;
	u8 oldr_nn_lqi = pi->oldr_nn_lqi;

	set_health_healthy(status);

	if (!lqi_ok(FIELD_GET(OLDR_NN_LQI_LINK_QUALITY_INDICATOR, oldr_nn_lqi)))
		set_health_degraded(status, FPORT_ISSUE_LQI);

	if (fls(pi->link_width_enabled) > fls(pi->link_width_active) ||
	    pi->link_width_downgrade_rx_active != pi->link_width_active ||
	    pi->link_width_downgrade_tx_active != pi->link_width_active)
		set_health_degraded(status, FPORT_ISSUE_LWD);

	if (fls(pi->link_speed_enabled) > fls(pi->link_speed_active))
		set_health_degraded(status, FPORT_ISSUE_RATE);
}

static void port_activate(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	u32 state;
	int err;

	state = IAF_FW_PORT_ACTIVE;

	err = ops_linkmgr_port_link_state_set(sd, p->lpn, state, &op_err);

	if (err)
		goto failed;

	if (op_err) {
		/*
		 * op_err could be set if link state is not ARMED (e.g., if it just bounced)
		 */
		fport_dbg(p, "querying port since ACTIVATE request failed\n");

		err = ops_linkmgr_ps_get(sd, p->lpn, &state);

		if (err)
			goto failed;

		switch (FIELD_GET(PS_PPS_PHYSICAL_STATE, state)) {
		case IAF_FW_PORT_PHYS_POLLING:
		case IAF_FW_PORT_PHYS_TRAINING:
		case IAF_FW_PORT_PHYS_LINKUP:
		case IAF_FW_PORT_PHYS_LINK_ERROR_RECOVERY:
		case IAF_FW_PORT_PHYS_OFFLINE:
			/* possible PHY state during error recovery */
			break;

		case IAF_FW_PORT_PHYS_DISABLED:
		case IAF_FW_PORT_PHYS_TEST:
		default:
			/* ports should not be able to get here on their own */
			goto failed;
		}

		switch (FIELD_GET(PS_PPS_PORT_STATE, state)) {
		case IAF_FW_PORT_DOWN:
		case IAF_FW_PORT_INIT:
			/* port bounced, notify the port manager */
			signal_pm_thread(sd, RESCAN_EVENT);
			return;

		case IAF_FW_PORT_ACTIVE:
			/* port already activated */
			break;

		case IAF_FW_PORT_ARMED:
		default:
			/* wrongly rejected and/or in unknown state */
			goto failed;
		}
	}

	p->state = PM_PORT_STATE_ACTIVE;

	set_active_health(sd, p);

	fio->status_changed = true;
	fio->routing_changed = true;

	fport_dbg(p, "ACTIVE\n");

	return;

failed:
	fport_err(p, "activate request failed\n");
	port_in_error(sd, p, fio);
}

enum neighbor_result {
	NEIGHBOR_VALID,
	NEIGHBOR_INVALID,
	NEIGHBOR_LOOP_DISALLOWED,
};

/**
 * valid_neighbor() - Is this connection to a valid neighbor?
 * @sd: subdevice being checked
 * @p: port being checked
 *
 * Indicates whether the connection is to a valid neighbor. To be a valid
 * neighbor, the neighbor GUID must correspond to a known device. If it is a
 * loopback (connected to same device), then whether it is valid is controlled
 * by the enable_loopback module parameter.
 *
 * return: true if this is a valid neighbor, false if it should be isolated
 */
static enum neighbor_result valid_neighbor(struct fsubdev *sd, struct fport *p)
{
	struct fdev *neighbor_dev;

	/* due to init races, we don't enforce isolation in preload mode */
	if (sd->fdev->startup_mode == STARTUP_MODE_PRELOAD)
		return NEIGHBOR_VALID;

	neighbor_dev = fdev_find_by_sd_guid(p->portinfo->neighbor_guid);

	if (!neighbor_dev)
		return NEIGHBOR_INVALID;

	/*
	 * It's safe to decrement neighbor_dev's reference count immediately
	 * since we're only testing whether it's our own device pointer
	 */
	fdev_put(neighbor_dev);

	if (neighbor_dev != sd->fdev)
		return NEIGHBOR_VALID;

	return port_enable_loopback ? NEIGHBOR_VALID : NEIGHBOR_LOOP_DISALLOWED;
}

static void port_arm(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	u32 state;
	int err;

	state = IAF_FW_PORT_ARMED;

	err = ops_linkmgr_port_link_state_set(sd, p->lpn, state, &op_err);

	if (err)
		goto failed;

	if (op_err) {
		/*
		 * op_err could be set if link state is not INIT (e.g., if it just bounced)
		 */
		fport_dbg(p, "querying port since ARM request failed\n");

		err = ops_linkmgr_ps_get(sd, p->lpn, &state);

		if (err)
			goto failed;

		switch (FIELD_GET(PS_PPS_PHYSICAL_STATE, state)) {
		case IAF_FW_PORT_PHYS_POLLING:
		case IAF_FW_PORT_PHYS_TRAINING:
		case IAF_FW_PORT_PHYS_LINKUP:
		case IAF_FW_PORT_PHYS_LINK_ERROR_RECOVERY:
		case IAF_FW_PORT_PHYS_OFFLINE:
			/* possible PHY state during error recovery */
			break;

		case IAF_FW_PORT_PHYS_DISABLED:
		case IAF_FW_PORT_PHYS_TEST:
		default:
			/* ports should not be able to get here on their own */
			goto failed;
		}

		if (FIELD_GET(PS_PPS_PORT_STATE, state) != IAF_FW_PORT_ARMED) {
			/* port bounced or already activated, notify the port manager */
			signal_pm_thread(sd, RESCAN_EVENT);
			return;
		}
	}

	if (sd->next_port_status[p->lpn].health == FPORT_HEALTH_FAILED) {
		set_health_off(&sd->next_port_status[p->lpn]);
		fio->status_changed = true;
	}

	p->state = PM_PORT_STATE_ARMED;

	if (FIELD_GET(OLDR_NN_LQI_NEIGHBOR_NORMAL, p->portinfo->oldr_nn_lqi))
		port_activate(sd, p, fio);

	return;

failed:
	fport_err(p, "arm request failed\n");
	port_in_error(sd, p, fio);
}

static void port_arm_or_isolate(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	del_timer(&p->linkup_timer);

	switch (valid_neighbor(sd, p)) {
	case NEIGHBOR_VALID:
		port_arm(sd, p, fio);
		break;
	case NEIGHBOR_INVALID:
		port_isolate(sd, p, fio);
		break;
	case NEIGHBOR_LOOP_DISALLOWED:
		set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_LOOPBACK);
		port_isolate(sd, p, fio);
		break;
	}
}

static void port_retry(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	/* downed port. retry by re-init and re-arm */
	if (is_fport_registered(p))
		fport_warn(p, "link went down, attempting to reconnect\n");
	else
		fport_dbg(p, "link went down, attempting to reconnect\n");

	set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_LINK_DOWN);
	fio->status_changed = true;

	p->state = PM_PORT_STATE_REQUESTED;
	start_linkup_timer(sd, p);
}

static void port_rearm(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	/* port reverted to INIT. retry by re-arm */
	if (is_fport_registered(p))
		fport_warn(p, "link reinitialized itself, attempting to reconnect\n");
	else
		fport_dbg(p, "link reinitialized itself, attempting to reconnect\n");

	set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_LINK_DOWN);
	fio->status_changed = true;

	port_arm(sd, p, fio);
}

static void port_retry_active(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (is_fport_registered(p))
		fport_warn(p, "active link went down...\n");
	else
		fport_dbg(p, "active link went down...\n");

	port_retry(sd, p, fio);
	fio->routing_changed = true;
}

static void port_rearm_active(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (is_fport_registered(p))
		fport_warn(p, "active link reinitialized itself...\n");
	else
		fport_dbg(p, "active link reinitialized itself...\n");

	port_rearm(sd, p, fio);
	fio->routing_changed = true;
}

static void port_flapping(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u32 op_err = 0;
	u32 state;
	int err;

	del_timer(&p->linkup_timer);

	set_health_failed(&sd->next_port_status[p->lpn], FPORT_ERROR_FLAPPING);

	clear_bit(PORT_CONTROL_ENABLED, p->controls);

	state = IAF_FW_PORT_PHYS_DISABLED;
	err = ops_linkmgr_port_maj_phystate_set(sd, p->lpn, state, &op_err);

	/* op_err should never be set when DISABLING */
	if (err || op_err)
		goto failed;

	/* update local state to be consistent with op on success */
	p->log_state = IAF_FW_PORT_DOWN;
	p->phys_state = IAF_FW_PORT_PHYS_DISABLED;
	linkdown_detected(sd, p);

	p->state = PM_PORT_STATE_INACTIVE;
	fio->routing_changed = true;
	fio->status_changed = true;

	fport_warn(p, "flapping port disabled\n");
	return;

failed:
	fport_err(p, "flapping disable request failed\n");
	port_in_error(sd, p, fio);
}

/*
 * FSM States
 */

static void fsm_INACTIVE(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (test_bit(PORT_CONTROL_ENABLED, p->controls))
		port_enable_polling(sd, p, fio);
}

static void fsm_REQUESTED(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (!test_bit(PORT_CONTROL_ENABLED, p->controls)) {
		port_disable(sd, p, fio);
		return;
	}

	switch (p->log_state) {
	case IAF_FW_PORT_DOWN:
		linkdown_detected(sd, p);
		if (!timer_pending(&p->linkup_timer))
			port_linkup_timedout(sd, p, fio);
		break;

	case IAF_FW_PORT_INIT:
		if (linkup_detected_now(sd, p))
			port_arm_or_isolate(sd, p, fio);
		else
			port_flapping(sd, p, fio);
		break;

	case IAF_FW_PORT_ARMED:
	case IAF_FW_PORT_ACTIVE:
	default:
		fport_err(p, "unexpected FW startup state %s\n",
			  fw_log_state_name(p->log_state));
		port_in_error(sd, p, fio);
		indicate_subdevice_error(sd, SD_ERROR_FW);
		break;
	}
}

static void fsm_ISOLATED(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (!test_bit(PORT_CONTROL_ENABLED, p->controls)) {
		port_disable(sd, p, fio);
		return;
	}

	if (fio->deisolating)
		port_deisolate(sd, p, fio);
}

static void fsm_ARMED(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	u8 oldr_nn_lqi = p->portinfo->oldr_nn_lqi;

	if (!test_bit(PORT_CONTROL_ENABLED, p->controls)) {
		port_disable(sd, p, fio);
		return;
	}

	switch (p->log_state) {
	case IAF_FW_PORT_DOWN:
		linkdown_detected(sd, p);
		port_retry(sd, p, fio);
		break;

	case IAF_FW_PORT_INIT:
		if (linkup_detected_now(sd, p))
			port_rearm(sd, p, fio);
		else
			port_flapping(sd, p, fio);
		break;

	case IAF_FW_PORT_ARMED:
	case IAF_FW_PORT_ACTIVE:
	default:
		if (FIELD_GET(OLDR_NN_LQI_NEIGHBOR_NORMAL, oldr_nn_lqi))
			port_activate(sd, p, fio);
		break;
	}
}

static void fsm_ACTIVE(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	if (!test_bit(PORT_CONTROL_ENABLED, p->controls)) {
		port_disable(sd, p, fio);
		return;
	}

	switch (p->log_state) {
	case IAF_FW_PORT_DOWN:
		linkdown_detected(sd, p);
		port_retry_active(sd, p, fio);
		break;

	case IAF_FW_PORT_INIT:
		if (linkup_detected_now(sd, p))
			port_rearm_active(sd, p, fio);
		else
			port_flapping(sd, p, fio);
		break;

	case IAF_FW_PORT_ARMED:
	case IAF_FW_PORT_ACTIVE:
	default:
		set_active_health(sd, p);

		fio->status_changed = true;
		break;
	}
}

/*
 * FSM driver function
 */

static void kick_fsms(struct fsubdev *sd, struct fsm_io *fio)
{
	struct fport *p;
	u8 lpn;

	for_each_fabric_port(p, lpn, sd) {
		fport_dbg(p, "state: %s\n", state_name(p->state));
		switch (p->state) {
		case PM_PORT_STATE_INACTIVE:
			fsm_INACTIVE(sd, p, fio);
			break;

		case PM_PORT_STATE_REQUESTED:
			fsm_REQUESTED(sd, p, fio);
			break;

		case PM_PORT_STATE_ISOLATED:
			fsm_ISOLATED(sd, p, fio);
			break;

		case PM_PORT_STATE_ARMED:
			fsm_ARMED(sd, p, fio);
			break;

		case PM_PORT_STATE_ACTIVE:
			fsm_ACTIVE(sd, p, fio);
			break;

		default:
			port_in_error(sd, p, fio);
			break;
		}
		fport_dbg(p, "new state: %s\n", state_name(p->state));
	}
}

/*
 * Module service functions
 */

static int read_portinfo(struct fsubdev *sd)
{
	u32 port_mask;

	/* Fetch all ports including CPORT */
	port_mask = ~(~0UL << sd->extended_port_cnt);

	return ops_portinfo_get(sd, port_mask, &sd->portinfo_op);
}

static void publish_status_updates(struct fsubdev *sd)
{
	struct fport_status *old_unpublished = sd->next_port_status;
	struct fport_status *new_unpublished;
	u8 lpn;

	/* Swap published state pointer */
	sd->next_port_status = rcu_replace_pointer(sd->port_status,
						   sd->next_port_status, true);
	synchronize_rcu();

	/* copy new unpublished data */
	new_unpublished = sd->next_port_status;

	for_each_fabric_lpn(lpn, sd)
		new_unpublished[lpn] = old_unpublished[lpn];

	sd_dbg(sd, "Port status change published");
}

static void handle_rescan_work(struct work_struct *work)
{
	struct fsubdev *sd = container_of(work, struct fsubdev, rescan_work);

	signal_pm_thread(sd, RESCAN_EVENT);
}

static void issue_sub_device_trap_count_work(struct work_struct *work)
{
	struct fsubdev *sd = container_of(to_delayed_work(work), struct fsubdev, ue_work);
	static char *type_msg = "TYPE=PORT_CHANGE";
	char fabric_id_msg[21];
	char sd_index_msg[11];
	u64 psc_trap_count = atomic64_read(&sd->psc_trap_count);
	u64 lwd_trap_count = atomic64_read(&sd->lwd_trap_count);
	u64 lqi_trap_count = atomic64_read(&sd->lqi_trap_count);
	u64 qsfp_fault_trap_count = atomic64_read(&sd->qsfp_fault_trap_count);
	u64 qsfp_present_trap_count = atomic64_read(&sd->qsfp_present_trap_count);
	char trap_count_msg[32];
	char psc_trap_count_msg[36];
	char lwd_trap_count_msg[36];
	char lqi_trap_count_msg[36];
	char qsfp_fault_trap_count_msg[43];
	char qsfp_present_trap_count_msg[45];
	char *envp[10] = { type_msg, fabric_id_msg, sd_index_msg, trap_count_msg,
			   psc_trap_count_msg, lwd_trap_count_msg, lqi_trap_count_msg,
			   qsfp_fault_trap_count_msg, qsfp_present_trap_count_msg, NULL, };

	snprintf(fabric_id_msg, sizeof(fabric_id_msg), "FABRIC_ID=0x%x", sd->fdev->fabric_id);
	snprintf(sd_index_msg, sizeof(sd_index_msg), "SD_INDEX=%u", sd_index(sd));
	snprintf(trap_count_msg, sizeof(trap_count_msg), "TRAP_COUNT=%llu",
		 psc_trap_count + lwd_trap_count + lqi_trap_count +
		 qsfp_fault_trap_count + qsfp_present_trap_count);
	snprintf(psc_trap_count_msg, sizeof(psc_trap_count_msg),
		 "PSC_TRAP_COUNT=%llu", psc_trap_count);
	snprintf(lwd_trap_count_msg, sizeof(lwd_trap_count_msg),
		 "LWD_TRAP_COUNT=%llu", lwd_trap_count);
	snprintf(lqi_trap_count_msg, sizeof(lqi_trap_count_msg),
		 "LQI_TRAP_COUNT=%llu", lqi_trap_count);
	snprintf(qsfp_fault_trap_count_msg, sizeof(qsfp_fault_trap_count_msg),
		 "QSFP_FAULT_TRAP_COUNT=%llu", qsfp_fault_trap_count);
	snprintf(qsfp_present_trap_count_msg, sizeof(qsfp_present_trap_count_msg),
		 "QSFP_PRESENT_TRAP_COUNT=%llu", qsfp_present_trap_count);

	kobject_uevent_env(&sd->fdev->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static void detect_preload_state(struct fsubdev *sd, struct fport *p, struct fsm_io *fio)
{
	switch (p->log_state) {
	case IAF_FW_PORT_DOWN:
		fport_info(p, "preload: DOWN\n");
		if (p->phys_state == IAF_FW_PORT_PHYS_DISABLED) {
			p->state = PM_PORT_STATE_INACTIVE;
		} else {
			p->state = PM_PORT_STATE_REQUESTED;
			start_linkup_timer(sd, p);
		}
		break;

	case IAF_FW_PORT_INIT:
		fport_info(p, "preload: INIT\n");
		p->state = PM_PORT_STATE_REQUESTED;
		linkup_detected_now(sd, p);
		port_arm(sd, p, fio);
		break;

	case IAF_FW_PORT_ARMED:
		fport_info(p, "preload: ARMED\n");
		p->state = PM_PORT_STATE_ARMED;
		linkup_detected_now(sd, p);
		port_activate(sd, p, fio);
		break;

	case IAF_FW_PORT_ACTIVE:
		fport_info(p, "preload: ACTIVE\n");
		p->state = PM_PORT_STATE_ACTIVE;
		set_active_health(sd, p);
		linkup_detected_now(sd, p);

		fio->status_changed = true;
		fio->routing_changed = true;
		break;

	default:
		fport_err(p, "unexpected FW preload state: log %s phys %u\n",
			  fw_log_state_name(p->log_state), p->phys_state);
		indicate_subdevice_error(sd, SD_ERROR_FW);
		break;
	}

	/* even if port_manual_bringup, default to enabled if it was preloaded as such */
	if (p->state != PM_PORT_STATE_INACTIVE)
		set_bit(PORT_CONTROL_ENABLED, p->controls);
}

static void check_init_state(struct fsubdev *sd, struct fport *p, bool initializing)
{
	if (initializing && p->log_state != IAF_FW_PORT_DOWN) {
		fport_err(p, "unexpected FW initial state %s\n",
			  fw_log_state_name(p->log_state));
		indicate_subdevice_error(sd, SD_ERROR_FW);
	}
}

/* Main PM thread runs on WQ, max of one running and one queued at a time */
static void update_ports_work(struct work_struct *work)
{
	struct fsubdev *sd = container_of(work, struct fsubdev, pm_work);
	bool initializing = false;
	bool port_change = false;
	bool qsfp_change = false;
	bool rescanning = false;
	struct fsm_io fio;
	struct fport *p;
	int err;
	u8 lpn;
	u8 pspps;

	/* PM should never be running against a debug device */
	if (WARN_ON(dev_is_runtime_debug(sd->fdev)))
		return;

	/* FSM-affecting/affected state */
	fio.deisolating = false;
	fio.request_deisolation = false;
	fio.routing_changed = false;
	fio.status_changed = false;

	/* lock may take a while to acquire if routing is running */
	down_read(&routable_lock); /* shared lock */

	/*
	 * Note that it is possible that update_ports could be queued again
	 * while it was waiting to get the lock. If so it will rerun and
	 * discover that nothing has changed.
	 *
	 * In that case, we could skip the kick_fsms() call since they
	 * shouldn't have any effect.
	 *
	 * Also, the odds of that happening could have been greatly reduced by
	 * calling cancel_work() here, though that function was removed from
	 * kernel 4.16 when (according to the commit msg) it was discovered by
	 * accident that it wasn't being used. It might be worth investigating
	 * alternatives in the future, since the underlying support functions
	 * appear to still support it.
	 */

	if (test_and_clear_bit(INIT_EVENT, sd->pm_triggers)) {
		initializing = true;
		port_change = true; /* might have missed PSC trap */
		qsfp_change = true;
	}

	if (test_and_clear_bit(DEISOLATE_EVENT, sd->pm_triggers)) {
		for_each_fabric_port(p, lpn, sd)
			if (p->state == PM_PORT_STATE_ISOLATED)
				fio.deisolating = true;

		if (fio.deisolating) {
			rescanning = true;
			port_change = true;
			qsfp_change = true;
		}
	}

	if (test_and_clear_bit(PSC_TRAP, sd->pm_triggers)) {
		port_change = true;
		err = ops_linkmgr_psc_trap_ack(sd, false);
		if (err)
			sd_err(sd, "failed to ACK PSC trap\n");
		atomic64_inc(&sd->psc_trap_count);
	}

	if (test_and_clear_bit(LWD_TRAP, sd->pm_triggers)) {
		port_change = true;
		err = ops_linkmgr_port_lwd_trap_ack(sd, false);
		if (err)
			sd_err(sd, "failed to ACK LWD trap\n");
		atomic64_inc(&sd->lwd_trap_count);
	}

	if (test_and_clear_bit(LQI_TRAP, sd->pm_triggers)) {
		port_change = true;
		err = ops_linkmgr_port_lqi_trap_ack(sd, false);
		if (err)
			sd_err(sd, "failed to ACK LQI trap\n");
		atomic64_inc(&sd->lqi_trap_count);
	}

	if (test_and_clear_bit(QSFP_PRESENCE_TRAP, sd->pm_triggers)) {
		qsfp_change = true;
		/*
		 * Not implemented yet:
		 *
		 * err = ops_qsfp_mgr_fault_trap_acknowledge(sd, 0, 0, false);
		 * if (err)
		 *	sd_err(sd, "failed to ACK QSFP presence trap\n");
		 * atomic64_inc(&sd->qsfp_present_trap_count);
		 */
	}

	if (test_and_clear_bit(QSFP_FAULT_TRAP, sd->pm_triggers)) {
		qsfp_change = true;
		err = ops_qsfpmgr_fault_trap_ack(sd, false);
		if (err)
			sd_err(sd, "failed to ACK qsfp fault trap\n");
		atomic64_inc(&sd->qsfp_fault_trap_count);
	}

	if (test_and_clear_bit(RESCAN_EVENT, sd->pm_triggers)) {
		rescanning = true;
		port_change = true;
		qsfp_change = true;
	}

	if (test_and_clear_bit(NL_PM_CMD_EVENT, sd->pm_triggers))
		rescanning = true;

	sd_dbg(sd, "Updating ports:%s%s%s%s%s\n",
	       initializing ? " INIT" : "",
	       port_change ? " PSC" : "",
	       qsfp_change ? " QSFP" : "",
	       rescanning ? " RESCAN" : "",
	       fio.deisolating ? " DEISOLATE" : "");

	/*
	 * Read state as needed
	 */
	if (initializing || port_change) {
		cancel_work_sync(&sd->rescan_work);

		err = read_portinfo(sd);
		if (err)
			sd_err(sd, "failed to update portinfo\n");

		for_each_fabric_port(p, lpn, sd) {
			pspps = p->portinfo->port_state_port_physical_state;

			p->log_state = FIELD_GET(PS_PPS_PORT_STATE, pspps);
			p->phys_state = FIELD_GET(PS_PPS_PHYSICAL_STATE, pspps);

			if (sd->fdev->startup_mode == STARTUP_MODE_PRELOAD)
				detect_preload_state(sd, p, &fio);
			else
				check_init_state(sd, p, initializing);

			fport_dbg(p,
				  "info: type %u LMA %u PPS %x OLDRNNLQI %x neighbor %016llx/%u/%u/%u LIR %u LDR %u FID %08x LDC %u PEA %u LQC %u\n",
				  p->portinfo->port_type,
				  p->portinfo->port_link_mode_active,
				  p->portinfo->port_state_port_physical_state,
				  p->portinfo->oldr_nn_lqi,
				  p->portinfo->neighbor_guid,
				  p->portinfo->neighbor_port_number,
				  p->portinfo->neighbor_link_down_reason,
				  p->portinfo->neighbor_mtu,
				  p->portinfo->link_init_reason,
				  p->portinfo->link_down_reason,
				  p->portinfo->fid,
				  p->portinfo->link_down_count,
				  p->portinfo->port_error_action,
				  p->portinfo->lqi_change_count);

			trace_pm_psc(sd->fdev->pd->index, sd_index(sd), lpn, p->portinfo);
		}
	}

	/*
	 * Not implemented yet:
	 *
	 * if (qsfp_change)
	 *	read_qsfp_data;
	 */

	/* Kick the FSMs if any request was seen */
	if (initializing || port_change || qsfp_change || rescanning)
		kick_fsms(sd, &fio);

	if (fio.request_deisolation)
		deisolate_all_ports();

	up_read(&routable_lock);

	if (initializing || fio.status_changed)
		publish_status_updates(sd);

	if (fio.routing_changed)
		rem_request();

	if (initializing || port_change || qsfp_change || rescanning) {
		mutex_lock(&sd->pm_work_lock);
		if (sd->ok_to_schedule_pm_work)
			queue_delayed_work(iaf_unbound_wq, &sd->ue_work, UEVENT_DELAY);
		/* else we are shutting down so don't send a state change UEVENT */
		mutex_unlock(&sd->pm_work_lock);
	}

	sd_dbg(sd, "Ports updated%s%s%s\n",
	       fio.request_deisolation ? " DEISOLATION_REQUESTED" : "",
	       fio.routing_changed ? " ROUTING_CHANGED" : "",
	       fio.status_changed ? " STATUS_CHANGED" : "");
}

static int initial_switchinfo_state(struct fsubdev *sd)
{
	/* num_ports does not include CPORT, which is always present */
	sd->extended_port_cnt = sd->switchinfo.num_ports + 1;

	sd_dbg(sd, "Detected %d ports\n", sd->extended_port_cnt);

	if (sd->extended_port_cnt > PORT_COUNT) {
		sd_err(sd, "Too many overall ports detected\n");
		return -ENOENT;
	}

	return 0;
}

static void linkup_timer_expired(struct timer_list *timer)
{
	struct fport *p = from_timer(p, timer, linkup_timer);

	/*
	 * signal_pm_thread() cannot be called from atomic context, so use another workitem to wake
	 * the PM thread
	 *
	 * it is only safe to do this since del_timer_sync is called on all timers before canceling
	 * rescan_work when cleaning up
	 */
	queue_work(iaf_unbound_wq, &p->sd->rescan_work);
}

/**
 * ports_to_automatically_enable() - Determine which ports to enable at startup
 * @sd: fabric subdevice object
 *
 * If port manual bringup is specified, no ports are brought up automatically, If not and PSC
 * presence rules are found, these control which eligible ports are brought up (based on link
 * config pins and/or default rules). Otherwise, all eligible ports are brought up by default.
 *
 * Presence rules by default affect all subdevices--a non-zero subdev_map is a bitfield indexed
 * by subdevice number identifying which are affected by the rule. For DEFAULT rules, a non-zero
 * index means to enable these ports by default. For LINK_CONTROL rules, index identifies which
 * link control bit enables these ports. Either way, port_map is a bitmap indexed by port number
 * identifying which port enables are controlled.
 *
 * In addition to the return value of this function, port type is also used to determine which
 * ports are eligible. Currently only FIXED or VARIABLE ports are brought up automatically.
 *
 * Context: device initialization
 * Return: bitmap indexed by port number indicating which ports may be enabled
 */
static u32 ports_to_automatically_enable(struct fsubdev *sd)
{
	struct psc_presence_rule *rule = sd->fdev->psc.presence_rules;
	size_t rule_count = sd->fdev->psc.n_presence_rules;
	u32 link_config = sd->fdev->link_config;
	u32 port_enable_map;
	size_t i;

	if (port_manual_bringup)
		return 0;

	if (!rule)
		return ~0;

	port_enable_map = 0;

	for (i = 0; i < rule_count; ++i, ++rule) {
		if (rule->subdev_map && !(rule->subdev_map & BIT(sd_index(sd))))
			continue;
		if ((rule->method == PRESENCE_RULE_DEFAULT      && rule->index) ||
		    (rule->method == PRESENCE_RULE_LINK_CONTROL && link_config & BIT(rule->index)))
			port_enable_map |= rule->port_map ? rule->port_map : ~0;
	}

	return port_enable_map;
}

/*
 * Module Entry points
 */

/*
 * Go through extended port structures, identifying all fabric ports. Set up
 * ports array (e.g., refer to relevant portinfo pointers) and set port_cnt to
 * number of fabric ports.
 */
static int initial_port_state(struct fsubdev *sd)
{
	struct fport_status *init_status;
	struct portinfo *curr_portinfo;
	u32 port_enable_map;
	struct fport *p;
	u8 port_cnt;
	u8 lpn;
	int err;

	p = sd->port + PORT_PHYSICAL_START;
	port_cnt = 0;

	init_status = sd->_portstatus;

	port_enable_map = ports_to_automatically_enable(sd);

	for (lpn = PORT_PHYSICAL_START, curr_portinfo = &sd->portinfo_op.per_portinfo[lpn];
	     lpn < sd->extended_port_cnt;
	     ++lpn, ++p, ++curr_portinfo) {
		u8 mode = curr_portinfo->port_link_mode_active;
		u8 type = curr_portinfo->port_type;
		u8 lane;

		sd_dbg(sd, PORT_FMT "mode %u type %u\n", lpn, mode, type);

		p->lpn = lpn;
		p->portinfo = curr_portinfo;
		p->sd = sd;
		p->port_type = type;
		for (lane = 0; lane < LANES; lane++)
			p->lanes_port[lane] = p;
		timer_setup(&p->linkup_timer, linkup_timer_expired, 0);

		INIT_LIST_HEAD(&p->unroute_link);

		create_port_debugfs_dir(sd, lpn);

		switch (mode) {
		case IAF_FW_PORT_LINK_MODE_FABRIC:
			if (port_cnt >= PORT_FABRIC_COUNT)
				return -ENOENT;

			if (lpn > TXCAL_PORT_COUNT) {
				sd_err(sd, "p.%u: fabric port numbers cannot exceed %u\n",
				       lpn, TXCAL_PORT_COUNT);

				return -EINVAL;
			}

			bitmap_zero(p->controls, NUM_PORT_CONTROLS);

			if (port_enable_map & BIT(lpn) &&
			    (type == IAF_FW_PORT_TYPE_FIXED || type == IAF_FW_PORT_TYPE_VARIABLE))
				set_bit(PORT_CONTROL_ENABLED, p->controls);

			set_bit(PORT_CONTROL_ROUTABLE, p->controls);

			p->state = PM_PORT_STATE_INACTIVE;

			sd->port_cnt = ++port_cnt;

			set_health_off(&init_status[lpn]);
			clear_status_linkup_detected(&init_status[lpn]);
			clear_status_link_failures(&init_status[lpn]);
			clear_status_link_degrades(&init_status[lpn]);
			set_bit(lpn, sd->fport_lpns);

			create_fabric_port_debugfs_files(sd, p);

			if (sd->fdev->psc.txcal) {
				if (sd->txcal[lpn]) {
					err = ops_tx_dcc_margin_param_set(sd, lpn, sd->txcal[lpn],
									  true);
					if (err) {
						sd_err(sd, "p.%u: TX calibration write failed\n",
						       lpn);
						return err;
					}
				} else if (type != IAF_FW_PORT_TYPE_DISCONNECTED &&
					type != IAF_FW_PORT_TYPE_UNKNOWN) {
					if (is_fdev_registered(sd->fdev))
						sd_warn(sd, "p.%u: missing TX calibration data\n",
							lpn);
				}
			}

			break;

		case IAF_FW_PORT_LINK_MODE_FLIT_BUS:
			set_bit(lpn, sd->bport_lpns);
			create_bridge_port_debugfs_files(sd, p);

			if (sd->txcal[lpn])
				sd_warn(sd, "p.%u: has unexpected TX calibration data\n", lpn);

			break;

		default:
			sd_err(sd, "p.%u: Invalid port mode %u\n", lpn, mode);
			return -ENOENT;
		}

		routing_debug_port_init(sd, lpn);
	}

	/* published and unpublished queryable port state */

	rcu_assign_pointer(sd->port_status, init_status);

	sd->next_port_status = &sd->_portstatus[PORT_COUNT];
	for_each_fabric_lpn(lpn, sd)
		sd->next_port_status[lpn] = init_status[lpn];

	return 0;
}

int initialize_fports(struct fsubdev *sd)
{
	struct fdev *dev = sd->fdev;
	int err;
	u32 i;

	bitmap_zero(sd->pm_triggers, NUM_PM_TRIGGERS);

	mutex_lock(&sd->pm_work_lock);
	INIT_WORK(&sd->pm_work, update_ports_work);
	INIT_DELAYED_WORK(&sd->ue_work, issue_sub_device_trap_count_work);
	INIT_WORK(&sd->rescan_work, handle_rescan_work);
	sd->ok_to_schedule_pm_work = true;
	mutex_unlock(&sd->pm_work_lock);

	atomic64_set(&sd->psc_trap_count, 0);
	atomic64_set(&sd->lwd_trap_count, 0);
	atomic64_set(&sd->lqi_trap_count, 0);
	atomic64_set(&sd->qsfp_fault_trap_count, 0);
	atomic64_set(&sd->qsfp_present_trap_count, 0);

	err = ops_switchinfo_get(sd, &sd->switchinfo);
	if (err)
		goto init_failed;

	sd->guid = sd->switchinfo.guid;
	sd_dbg(sd, "guid 0x%016llx\n", sd->guid);

	/*
	 * Look for this subdevice's GUID in TX calibration blob data and extract settings for
	 * all fabric ports. No need to warn on failure: ports warn when initialized
	 */
	if (dev->psc.txcal)
		for (i = 0; i < dev->psc.txcal->num_settings; ++i)
			if (dev->psc.txcal->data[i].guid == sd->guid)
				memcpy(&sd->txcal[PORT_PHYSICAL_START],
				       dev->psc.txcal->data[i].port_settings,
				       sizeof(dev->psc.txcal->data[i].port_settings));

	err = initial_switchinfo_state(sd);
	if (err)
		goto init_failed;

	err = read_portinfo(sd);
	if (err)
		goto init_failed;

	err = initial_port_state(sd);
	if (err)
		goto init_failed;

	err = ops_linkmgr_psc_trap_ena_set(sd, true, false);
	if (err)
		goto init_failed;
	err = ops_linkmgr_port_lwd_trap_ena_set(sd, true, false);
	if (err)
		goto init_failed;
	err = ops_linkmgr_port_lqi_trap_ena_set(sd, true, false);
	if (err)
		goto init_failed;

	err = routing_sd_once(sd);
	if (err)
		goto init_failed;

	signal_pm_thread(sd, INIT_EVENT);

	return 0;

init_failed:

	sd_err(sd, "Could not initialize ports: %d\n", err);
	return err;
}

void destroy_fports(struct fsubdev *sd)
{
	struct mbdb_op_fw_version_rsp *fw_version = &sd->fw_version;
	bool pm_wq_was_active;
	struct fport *p;
	u8 lpn;

	mutex_lock(&sd->pm_work_lock);
	pm_wq_was_active = sd->ok_to_schedule_pm_work;
	if (pm_wq_was_active)
		sd->ok_to_schedule_pm_work = false;
	mutex_unlock(&sd->pm_work_lock);

	/*
	 * must cancel/sync timers (or anything else capable of triggering rescan_work) after
	 * clearing ok_to_schedule_pm_work (so pm_work does not reschedule a timer) and before
	 * canceling rescan_work (in case canceling a timer triggers it)
	 */
	for_each_fabric_port(p, lpn, sd)
		del_timer_sync(&p->linkup_timer);

	if (pm_wq_was_active) {
		cancel_delayed_work_sync(&sd->ue_work);
		cancel_work_sync(&sd->rescan_work);
		cancel_work_sync(&sd->pm_work);
	}
	/* else INIT_WORK calls were not completed */

	if (READ_ONCE(sd->fw_running)) {
		/* ignore MBDB errors here */
		ops_linkmgr_port_lqi_trap_ena_set(sd, false, true);
		ops_linkmgr_port_lwd_trap_ena_set(sd, false, true);
		ops_linkmgr_psc_trap_ena_set(sd, false, true);
	}

	routing_sd_destroy(sd);

	if (param_reset && (fw_version->environment & FW_VERSION_INIT_BIT))
		ops_reset(sd, true);

	for_each_fabric_port(p, lpn, sd)
		if (p->state == PM_PORT_STATE_ISOLATED)
			atomic_dec(&isolated_port_cnt);

	rem_request();
}

int enable_fports(struct fsubdev *sd, unsigned long *lpnmask, u8 max_ports)
{
	int unchanged = 1;
	struct fport *p;
	u8 lpn;

	for_each_masked_port(p, lpn, sd->port, lpnmask, max_ports)
		unchanged &= test_and_set_bit(PORT_CONTROL_ENABLED, p->controls);

	return unchanged ? 0 : signal_pm_thread(sd, NL_PM_CMD_EVENT);
}

int disable_fports(struct fsubdev *sd, unsigned long *lpnmask, u8 max_ports)
{
	bool any_was_enabled = false;
	struct fport *p;
	u8 lpn;

	/* routing lock, then mappings lock */
	down_write(&routable_lock); /* exclusive lock */
	if (mappings_ref_check(sd->fdev)) {
		up_write(&routable_lock);
		return -EAGAIN;
	}

	for_each_masked_port(p, lpn, sd->port, lpnmask, max_ports) {
		bool enabled = test_and_clear_bit(PORT_CONTROL_ENABLED, p->controls);

		if (enabled && test_bit(PORT_CONTROL_ROUTABLE, p->controls) &&
		    list_empty(&p->unroute_link))
			list_add_tail(&p->unroute_link, &sd->fdev->port_unroute_list);
		any_was_enabled |= enabled;
	}
	up_write(&routable_lock);

	return any_was_enabled ? signal_pm_thread(sd, NL_PM_CMD_EVENT) : 0;
}

int enable_usage_fports(struct fsubdev *sd, unsigned long *lpnmask, u8 max_ports)
{
	int unchanged = 1;
	struct fport *p;
	u8 lpn;

	for_each_masked_port(p, lpn, sd->port, lpnmask, max_ports)
		unchanged &= test_and_set_bit(PORT_CONTROL_ROUTABLE, p->controls);

	if (!unchanged)
		rem_request();

	return 0;
}

int disable_usage_fports(struct fsubdev *sd, unsigned long *lpnmask, u8 max_ports)
{
	bool any_was_routable = false;
	struct fport *p;
	u8 lpn;

	down_write(&routable_lock); /* exclusive lock */
	if (mappings_ref_check(sd->fdev)) {
		up_write(&routable_lock);
		return -EAGAIN;
	}

	for_each_masked_port(p, lpn, sd->port, lpnmask, max_ports) {
		bool routable = test_and_clear_bit(PORT_CONTROL_ROUTABLE, p->controls);

		if (routable && list_empty(&p->unroute_link))
			list_add_tail(&p->unroute_link, &sd->fdev->port_unroute_list);
		any_was_routable |= routable;
	}
	up_write(&routable_lock);

	if (any_was_routable)
		rem_request();

	return 0;
}

int enable_port(struct fport *p)
{
	unsigned long lpnmask = 1 << p->lpn;

	return enable_fports(p->sd, &lpnmask, PORT_COUNT);
}

int disable_port(struct fport *p)
{
	unsigned long lpnmask = 1 << p->lpn;

	return disable_fports(p->sd, &lpnmask, PORT_COUNT);
}

int enable_usage_port(struct fport *p)
{
	unsigned long lpnmask = 1 << p->lpn;

	return enable_usage_fports(p->sd, &lpnmask, PORT_COUNT);
}

int disable_usage_port(struct fport *p)
{
	unsigned long lpnmask = 1 << p->lpn;

	return disable_usage_fports(p->sd, &lpnmask, PORT_COUNT);
}

int get_fport_status(struct fsubdev *sd, u8 lpn, struct fport_status *status)
{
	struct fport_status *curr_status;
	int err = 0;

	if (!get_fport_handle(sd, lpn))
		return -EINVAL;

	rcu_read_lock();
	curr_status = rcu_dereference(sd->port_status);

	if (curr_status)
		*status = curr_status[lpn];
	else
		err = -EINVAL;

	rcu_read_unlock();
	return err;
}

void port_state_change_trap_handler(struct fsubdev *sd)
{
	signal_pm_thread(sd, PSC_TRAP);
}

void port_link_width_degrade_trap_handler(struct fsubdev *sd)
{
	signal_pm_thread(sd, LWD_TRAP);
}

void port_link_quality_indicator_trap_handler(struct fsubdev *sd)
{
	signal_pm_thread(sd, LQI_TRAP);
}

void port_qsfp_presence_trap_handler(struct fsubdev *sd)
{
	signal_pm_thread(sd, QSFP_PRESENCE_TRAP);
}

void port_qsfp_fault_trap_handler(struct fsubdev *sd)
{
	signal_pm_thread(sd, QSFP_FAULT_TRAP);
}
