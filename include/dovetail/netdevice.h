/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DOVETAIL_NETDEVICE_H
#define _DOVETAIL_NETDEVICE_H

/*
 * Placeholder for per-device state information otherwise defined by
 * the out-of-band network stack.
 */
struct oob_netdev_state {
};

struct oob_netqueue_state {
};

static inline void netqueue_init_oob(struct oob_netqueue_state *qs)
{
}

static inline void netqueue_destroy_oob(struct oob_netqueue_state *qs)
{
}

#endif /* !_DOVETAIL_NETDEVICE_H */
