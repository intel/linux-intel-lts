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


#endif /*  _LINUX_SDW_BUS_H */
