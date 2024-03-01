/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _NET_OOBNET_H
#define _NET_OOBNET_H

#include <dovetail/net.h>
#include <dovetail/netdevice.h>

/* Netdevice context extension for oob handling. */
struct oob_netdev_context {
	struct oob_netdev_state dev_state;
};

#endif /* !_NET_OOBNET_H */
