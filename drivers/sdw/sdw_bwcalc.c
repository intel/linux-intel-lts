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
 * sdw_get_col_to_num
 *
 * Returns column number from the mapping.
 */
int sdw_get_col_to_num(int col)
{

	return 0;
}


/**
 * sdw_get_row_to_num
 *
 * Returns row number from the mapping.
 */
int sdw_get_row_to_num(int row)
{

	return 0;
}

/*
 * sdw_lcm - returns LCM of two numbers
 *
 *
 * This function is called BW calculation function to find LCM
 * of two numbers.
 */
int sdw_lcm(int num1, int num2)
{

	return 0;
}


/*
 * sdw_cfg_slv_params - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function configures slave registers for
 * transport and port parameters.
 */
int sdw_cfg_slv_params(struct sdw_bus *mstr_bs,
		struct sdw_slave_runtime *slv_rt,
		struct sdw_transport_params *t_slv_params,
		struct sdw_port_params *p_slv_params)
{

	return 0;
}


/*
 * sdw_cfg_mstr_params - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function configures master registers for
 * transport and port parameters.
 */
int sdw_cfg_mstr_params(struct sdw_bus *mstr_bs,
		struct sdw_transport_params *t_mstr_params,
		struct sdw_port_params *p_mstr_params)
{

	return 0;
}


/*
 * sdw_cfg_mstr_slv - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function call master/slave transport/port
 * params configuration API's, called from sdw_bus_calc_bw
 * & sdw_bus_calc_bw_dis API's.
 */
int sdw_cfg_mstr_slv(struct sdw_bus *sdw_mstr_bs,
		struct sdw_mstr_runtime *sdw_mstr_bs_rt,
		bool is_master)
{

	return 0;
}


/*
 * sdw_cpy_params_mstr_slv - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function copies/configure master/slave transport &
 * port params to alternate bank.
 *
 */
int sdw_cpy_params_mstr_slv(struct sdw_bus *sdw_mstr_bs,
		struct sdw_mstr_runtime *sdw_mstr_bs_rt)
{

	return 0;
}


/*
 * sdw_cfg_slv_enable_disable - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function enable/disable slave port channels.
 */
int sdw_cfg_slv_enable_disable(struct sdw_bus *mstr_bs,
	struct sdw_slave_runtime *slv_rt_strm,
	struct sdw_port_runtime *port_slv_strm,
	struct port_chn_en_state *chn_en)
{

	return 0;
}


/*
 * sdw_cfg_mstr_activate_disable - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function enable/disable master port channels.
 */
int sdw_cfg_mstr_activate_disable(struct sdw_bus *mstr_bs,
		struct sdw_mstr_runtime *mstr_rt_strm,
		struct sdw_port_runtime *port_mstr_strm,
		struct port_chn_en_state *chn_en)
{

	return 0;
}


/*
 * sdw_en_dis_mstr_slv - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function call master/slave enable/disable
 * channel API's.
 */
int sdw_en_dis_mstr_slv(struct sdw_bus *sdw_mstr_bs,
		struct sdw_runtime *sdw_rt, bool is_act)
{

	return 0;
}


/*
 * sdw_en_dis_mstr_slv_state - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function call master/slave enable/disable
 * channel API's based on runtime state.
 */
int sdw_en_dis_mstr_slv_state(struct sdw_bus *sdw_mstr_bs,
	struct sdw_mstr_runtime *sdw_mstr_bs_rt,
	struct port_chn_en_state *chn_en)
{

	return 0;
}


/*
 * sdw_get_clock_frmshp - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function computes clock and frame shape based on
 * clock frequency.
 */
int sdw_get_clock_frmshp(struct sdw_bus *sdw_mstr_bs, int *frame_int,
		int *col, int *row)
{

	return 0;
}

/*
 * sdw_compute_sys_interval - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function computes system interval.
 */
int sdw_compute_sys_interval(struct sdw_bus *sdw_mstr_bs,
		struct sdw_master_capabilities *sdw_mstr_cap,
		int frame_interval)
{

	return 0;
}


/*
 * sdw_compute_hstart_hstop - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function computes hstart and hstop for running
 * streams per master & slaves.
 */
int sdw_compute_hstart_hstop(struct sdw_bus *sdw_mstr_bs, int sel_col)
{

	return 0;
}


/*
 * sdw_compute_blk_subblk_offset - returns Success
 *
 *
 * This function computes block offset and sub block
 * offset for running streams per master & slaves.
 */
int sdw_compute_blk_subblk_offset(struct sdw_bus *sdw_mstr_bs)
{

	return 0;
}


/*
 * sdw_configure_frmshp_bnkswtch - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function broadcast frameshape on framectrl
 * register and performs bank switch.
 */
int sdw_configure_frmshp_bnkswtch(struct sdw_bus *mstr_bs, int col, int row)
{

	return 0;
}


/*
 * sdw_cfg_bs_params - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function performs master/slave transport
 * params config, set SSP interval, set Clock
 * frequency, enable channel. This API is called
 * from sdw_bus_calc_bw & sdw_bus_calc_bw_dis API.
 *
 */
int sdw_cfg_bs_params(struct sdw_bus *sdw_mstr_bs,
		struct sdw_mstr_runtime *sdw_mstr_bs_rt,
		bool is_strm_cpy)
{

	return 0;
}

/*
 * sdw_dis_chan - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function disables channel on alternate
 * bank. This API is called from sdw_bus_calc_bw
 * & sdw_bus_calc_bw_dis when channel on current
 * bank is enabled.
 *
 */
int sdw_dis_chan(struct sdw_bus *sdw_mstr_bs,
	struct sdw_mstr_runtime *sdw_mstr_bs_rt)
{

	return 0;
}


/*
 * sdw_cfg_slv_prep_unprep - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function prepare/unprepare slave ports.
 */
int sdw_cfg_slv_prep_unprep(struct sdw_bus *mstr_bs,
	struct sdw_slave_runtime *slv_rt_strm,
	struct sdw_port_runtime *port_slv_strm,
	bool prep)
{

	return 0;
}


/*
 * sdw_cfg_mstr_prep_unprep - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function prepare/unprepare master ports.
 */
int sdw_cfg_mstr_prep_unprep(struct sdw_bus *mstr_bs,
	struct sdw_mstr_runtime *mstr_rt_strm,
	struct sdw_port_runtime *port_mstr_strm,
	bool prep)
{

	return 0;
}


/*
 * sdw_prep_unprep_mstr_slv - returns Success
 * -EINVAL - In case of error.
 *
 *
 * This function call master/slave prepare/unprepare
 * port configuration API's, called from sdw_bus_calc_bw
 * & sdw_bus_calc_bw_dis API's.
 */
int sdw_prep_unprep_mstr_slv(struct sdw_bus *sdw_mstr_bs,
		struct sdw_runtime *sdw_rt, bool is_prep)
{

	return 0;
}


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
