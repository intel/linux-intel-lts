/*
 *  sdw_bwcalc.c - SoundWire Bus BW calculation & CHN Enabling implementation
 *
 *  Copyright (C) 2015-2016 Intel Corp
 *  Author:  Sanyog Kale <sanyog.r.kale@intel.com>
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
 */

#include <linux/kernel.h>
#include <linux/sdw_bus.h>
#include "sdw_priv.h"


/**
 * sdw_bus_bw_init - returns Success
 *
 *
 * This function is called from sdw_init function when bus driver
 * gets intitalized. This function performs all the generic
 * intializations required for BW control.
 */
int sdw_bus_bw_init(void)
{

	return 0;
}
EXPORT_SYMBOL_GPL(sdw_bus_bw_init);


/**
 * sdw_mstr_bw_init - returns Success
 *
 *
 * This function is called from sdw_register_master function
 * for each master controller gets register. This function performs
 * all the intializations per master controller required for BW control.
 */
int sdw_mstr_bw_init(struct sdw_bus *sdw_bs)
{

	return 0;
}
EXPORT_SYMBOL_GPL(sdw_mstr_bw_init);


/**
 * sdw_bus_calc_bw - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function is called from sdw_prepare_and_enable
 * whenever new stream is processed. The function based
 * on the stream associated with controller calculates
 * required bandwidth, clock, frameshape, computes
 * all transport params for a given port, enable channel
 * & perform bankswitch.
 */
int sdw_bus_calc_bw(struct sdw_stream_tag *stream_tag, bool enable)
{

	return 0;
}
EXPORT_SYMBOL_GPL(sdw_bus_calc_bw);


/**
 * sdw_bus_calc_bw_dis - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function is called from sdw_disable_and_unprepare
 * whenever stream is ended. The function based disables/
 * unprepare port/channel of associated stream and computes
 * required bandwidth, clock, frameshape, computes
 * all transport params for a given port, enable channel
 * & perform bankswitch for remaining streams on given
 * controller.
 */
int sdw_bus_calc_bw_dis(struct sdw_stream_tag *stream_tag, bool unprepare)
{

	return 0;
}
EXPORT_SYMBOL_GPL(sdw_bus_calc_bw_dis);
