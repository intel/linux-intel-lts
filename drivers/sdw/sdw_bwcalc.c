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

#define MAXCLOCKFREQ           6

/* TBD: Currently we are using 100x2 as frame shape. to be removed later */
int rows[MAX_NUM_ROWS] = {100, 48, 50, 60, 64, 72, 75, 80, 90,
		     96, 125, 144, 147, 120, 128, 150,
		     160, 180, 192, 200, 240, 250, 256};

int cols[MAX_NUM_COLS] = {2, 4, 6, 8, 10, 12, 14, 16};

/*
 * TBD: Get supported clock frequency from ACPI and store
 * it in master data structure.
 */
/* Currently only 9.6MHz clock frequency used */
int clock_freq[MAXCLOCKFREQ] = {9600000, 9600000,
				9600000, 9600000,
				9600000, 9600000};

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
	int r, c, rowcolcount = 0;
	int control_bits = 48;

	for (c = 0; c < MAX_NUM_COLS; c++) {

		for (r = 0; r < MAX_NUM_ROWS; r++) {
			sdw_core.rowcolcomb[rowcolcount].col = cols[c];
			sdw_core.rowcolcomb[rowcolcount].row = rows[r];
			sdw_core.rowcolcomb[rowcolcount].control_bits =
				control_bits;
			sdw_core.rowcolcomb[rowcolcount].data_bits =
				(cols[c] * rows[r]) - control_bits;
			rowcolcount++;
		}
	}

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
	struct sdw_master_capabilities *sdw_mstr_cap = NULL;

	/* Initialize required parameters in bus structure */
	sdw_bs->bandwidth = 0;
	sdw_bs->system_interval = 0;
	sdw_bs->frame_freq = 0;
	/* TBD: Base Clock frequency should be read from
	 * master capabilities
	 * Currenly hardcoding to 9.6MHz
	 */
	sdw_bs->clk_freq = 9.6*1000*1000;
	sdw_bs->clk_state = SDW_CLK_STATE_ON;

	/* TBD: to be removed later */
	/* Assumption is these should be already filled */
	sdw_mstr_cap = &sdw_bs->mstr->mstr_capabilities;
	sdw_mstr_cap->base_clk_freq = 9.6 * 1000 * 1000;
	sdw_mstr_cap->monitor_handover_supported = false;
	sdw_mstr_cap->highphy_capable = false;

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

	struct sdw_runtime *sdw_rt = stream_tag->sdw_rt;
	struct sdw_stream_params *stream_params = &sdw_rt->stream_params;
	struct sdw_mstr_runtime *sdw_mstr_rt = NULL, *sdw_mstr_bs_rt = NULL;
	struct sdw_bus *sdw_mstr_bs = NULL;
	struct sdw_master *sdw_mstr = NULL;
	struct sdw_master_capabilities *sdw_mstr_cap = NULL;
	struct sdw_stream_params *mstr_params;
	int stream_frame_size;
	int frame_interval = 0, sel_row = 0, sel_col = 0;
	int ret = 0;

	/* TBD: Add PCM/PDM flag in sdw_config_stream */

	/*
	 * TBD: check for mstr_rt is in configured state or not
	 * If yes, then configure masters as well
	 * If no, then do not configure/enable master related parameters
	 */

	/* BW calulation for active master controller for given stream tag */
	list_for_each_entry(sdw_mstr_rt, &sdw_rt->mstr_rt_list, mstr_sdw_node) {

		if (sdw_mstr_rt->mstr == NULL)
			break;

		/* Get bus structure for master */
		list_for_each_entry(sdw_mstr_bs, &sdw_core.bus_list, bus_node) {

			/* Match master structure pointer */
			if (sdw_mstr_bs->mstr != sdw_mstr_rt->mstr)
				continue;


			sdw_mstr = sdw_mstr_bs->mstr;
			break;
		}

		/*
		 * All data structures required available,
		 * lets calculate BW for master controller
		 */

		/* Check for isochronous mode plus other checks if required */
		sdw_mstr_cap = &sdw_mstr_bs->mstr->mstr_capabilities;
		mstr_params = &sdw_mstr_rt->stream_params;

		if ((sdw_rt->stream_state == SDW_STATE_CONFIG_STREAM) ||
				(sdw_rt->stream_state ==
					SDW_STATE_UNPREPARE_STREAM)) {

			/* we do not support asynchronous mode Return Error */
			if ((sdw_mstr_cap->base_clk_freq % mstr_params->rate)
					!= 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Asynchronous mode not supported\n");
				return -EINVAL;
			}

			/* Check for sampling frequency */
			if (stream_params->rate != mstr_params->rate) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Sampling frequency mismatch\n");
				return -EINVAL;
			}

			/*
			 * Calculate stream bandwidth, frame size and
			 * total BW required for master controller
			 */
			sdw_mstr_rt->stream_bw = mstr_params->rate *
				mstr_params->channel_count * mstr_params->bps;
			stream_frame_size = mstr_params->channel_count *
				mstr_params->bps;

			sdw_mstr_bs->bandwidth += sdw_mstr_rt->stream_bw;

			ret = sdw_get_clock_frmshp(sdw_mstr_bs,
					&frame_interval, &sel_col, &sel_row);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev, "clock/frameshape config failed\n");
				return ret;
			}


			/*
			 * TBD: find right place to run sorting on
			 * master rt_list. Below sorting is done based on
			 * bps from low to high, that means PDM streams
			 * will be placed before PCM.
			 */

			/*
			 * TBD Should we also perform sorting based on rate
			 * for PCM stream check. if yes then how??
			 * creating two different list.
			 */

			/* Compute system interval */
			ret = sdw_compute_sys_interval(sdw_mstr_bs,
					sdw_mstr_cap, frame_interval);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev, "compute system interval failed\n");
				return ret;
			}

			/* Compute hstart/hstop */
			ret = sdw_compute_hstart_hstop(sdw_mstr_bs, sel_col);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"compute hstart/hstop failed\n");
				return ret;
			}

			/* Compute block offset */
			ret = sdw_compute_blk_subblk_offset(sdw_mstr_bs);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(
						&sdw_mstr_bs->mstr->dev,
						"compute block offset failed\n");
				return ret;
			}

			/* Change Stream State */
			sdw_rt->stream_state = SDW_STATE_COMPUTE_STREAM;

			/* Configure bus parameters */
			ret = sdw_cfg_bs_params(sdw_mstr_bs,
					sdw_mstr_bs_rt, true);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"xport params config failed\n");
				return ret;
			}

			sel_col = sdw_mstr_bs->col;
			sel_row = sdw_mstr_bs->row;

			/* Configure Frame Shape/Switch Bank */
			ret = sdw_configure_frmshp_bnkswtch(sdw_mstr_bs,
					sel_col, sel_row);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"bank switch failed\n");
				return ret;
			}

			/* Disable all channels enabled on previous bank */
			ret = sdw_dis_chan(sdw_mstr_bs, sdw_mstr_bs_rt);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Channel disabled failed\n");
				return ret;
			}

			/* Prepare new port for master and slave */
			ret = sdw_prep_unprep_mstr_slv(sdw_mstr_bs,
					sdw_rt, true);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Channel prepare failed\n");
				return ret;
			}

			/* change stream state to prepare */
			sdw_rt->stream_state = SDW_STATE_PREPARE_STREAM;
		}

		if ((enable) && (SDW_STATE_PREPARE_STREAM
					== sdw_rt->stream_state)) {

			ret = sdw_cfg_bs_params(sdw_mstr_bs,
					sdw_mstr_bs_rt, false);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"xport params config failed\n");
				return ret;
			}

			/* Enable new port for master and slave */
			ret = sdw_en_dis_mstr_slv(sdw_mstr_bs, sdw_rt, true);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Channel enable failed\n");
				return ret;
			}

			/* change stream state to enable */
			sdw_rt->stream_state = SDW_STATE_ENABLE_STREAM;

			sel_col = sdw_mstr_bs->col;
			sel_row = sdw_mstr_bs->row;

			/* Configure Frame Shape/Switch Bank */
			ret = sdw_configure_frmshp_bnkswtch(sdw_mstr_bs,
					sel_col, sel_row);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"bank switch failed\n");
				return ret;
			}

			/* Disable all channels enabled on previous bank */
			ret = sdw_dis_chan(sdw_mstr_bs, sdw_mstr_bs_rt);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Channel disabled faile\n");
				return ret;
			}
		}
	}

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
	struct sdw_runtime *sdw_rt = stream_tag->sdw_rt;
	struct sdw_mstr_runtime *sdw_mstr_rt = NULL, *sdw_mstr_bs_rt = NULL;
	struct sdw_bus *sdw_mstr_bs = NULL;
	struct sdw_master *sdw_mstr = NULL;
	struct sdw_master_capabilities *sdw_mstr_cap = NULL;
	struct sdw_stream_params *mstr_params;
	int stream_frame_size;
	int frame_interval = 0, sel_row = 0, sel_col = 0;
	int ret = 0;


	/* BW calulation for active master controller for given stream tag */
	list_for_each_entry(sdw_mstr_rt, &sdw_rt->mstr_rt_list, mstr_sdw_node) {

		if (sdw_mstr_rt->mstr == NULL)
			break;

		/* Get bus structure for master */
		list_for_each_entry(sdw_mstr_bs, &sdw_core.bus_list, bus_node) {

			/* Match master structure pointer */
			if (sdw_mstr_bs->mstr != sdw_mstr_rt->mstr)
				continue;


			sdw_mstr = sdw_mstr_bs->mstr;
			break;
		}


		sdw_mstr_cap = &sdw_mstr_bs->mstr->mstr_capabilities;
		mstr_params = &sdw_mstr_rt->stream_params;

		if (sdw_rt->stream_state == SDW_STATE_ENABLE_STREAM) {

			/* Lets do disabling of port for stream to be freed */
			list_for_each_entry(sdw_mstr_bs_rt,
					&sdw_mstr->mstr_rt_list, mstr_node) {

				if (sdw_mstr_bs_rt->mstr == NULL)
					continue;

				/*
				 * Disable channel for slave and
				 * master on current bank
				 */
				ret = sdw_en_dis_mstr_slv(sdw_mstr_bs,
						sdw_rt, false);
				if (ret < 0) {
					/* TBD: Undo all the computation */
					dev_err(&sdw_mstr_bs->mstr->dev,
							"Channel dis failed\n");
					return ret;
				}

				/* Change stream state to disable */
				sdw_rt->stream_state = SDW_STATE_DISABLE_STREAM;
			}

			ret = sdw_cfg_bs_params(sdw_mstr_bs,
					sdw_mstr_bs_rt, false);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"xport params config failed\n");
				return ret;
			}

			sel_col = sdw_mstr_bs->col;
			sel_row = sdw_mstr_bs->row;

			/* Configure frame shape/Switch Bank  */
			ret = sdw_configure_frmshp_bnkswtch(sdw_mstr_bs,
					sel_col, sel_row);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"bank switch failed\n");
				return ret;
			}

			/* Disable all channels enabled on previous bank */
			ret = sdw_dis_chan(sdw_mstr_bs, sdw_mstr_bs_rt);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Channel disabled failed\n");
				return ret;
			}
		}

		if ((unprepare) &&
				(SDW_STATE_DISABLE_STREAM ==
				 sdw_rt->stream_state)) {

			/* 1. Un-prepare master and slave port */
			list_for_each_entry(sdw_mstr_bs_rt,
					&sdw_mstr->mstr_rt_list, mstr_node) {

				if (sdw_mstr_bs_rt->mstr == NULL)
					continue;

				ret = sdw_prep_unprep_mstr_slv(sdw_mstr_bs,
						sdw_rt, false);
				if (ret < 0) {
					/* TBD: Undo all the computation */
					dev_err(&sdw_mstr_bs->mstr->dev,
							"Chan unprep failed\n");
					return ret;
				}

				/* change stream state to unprepare */
				sdw_rt->stream_state =
					SDW_STATE_UNPREPARE_STREAM;
			}

			/*
			 * Calculate new bandwidth, frame size
			 * and total BW required for master controller
			 */
			sdw_mstr_rt->stream_bw = mstr_params->rate *
				mstr_params->channel_count * mstr_params->bps;
			stream_frame_size = mstr_params->channel_count *
				mstr_params->bps;

			sdw_mstr_bs->bandwidth -= sdw_mstr_rt->stream_bw;

			/* Something went wrong in bandwidth calulation */
			if (sdw_mstr_bs->bandwidth < 0) {
				dev_err(&sdw_mstr_bs->mstr->dev,
						"BW calculation failed\n");
				return -EINVAL;
			}

			if (!sdw_mstr_bs->bandwidth) {
				/*
				 * Last stream on master should
				 * return successfully
				 */
				sdw_rt->stream_state =
					SDW_STATE_UNCOMPUTE_STREAM;
				return 0;
			}

			ret = sdw_get_clock_frmshp(sdw_mstr_bs,
					&frame_interval, &sel_col, &sel_row);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"clock/frameshape failed\n");
				return ret;
			}

			/* Compute new transport params for running streams */
			/* No sorting required here */

			/* Compute system interval */
			ret = sdw_compute_sys_interval(sdw_mstr_bs,
					sdw_mstr_cap, frame_interval);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"compute SI failed\n");
				return ret;
			}

			/* Compute hstart/hstop */
			ret = sdw_compute_hstart_hstop(sdw_mstr_bs, sel_col);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"compute hstart/hstop fail\n");
				return ret;
			}

			/* Compute block offset */
			ret = sdw_compute_blk_subblk_offset(sdw_mstr_bs);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"compute block offset failed\n");
				return ret;
			}

			/* Configure bus params */
			ret = sdw_cfg_bs_params(sdw_mstr_bs,
					sdw_mstr_bs_rt, true);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"xport params config failed\n");
				return ret;
			}

			/* Configure Frame Shape/Switch Bank */
			ret = sdw_configure_frmshp_bnkswtch(sdw_mstr_bs,
					sel_col, sel_row);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"bank switch failed\n");
				return ret;
			}

			/* Change stream state to uncompute */
			sdw_rt->stream_state = SDW_STATE_UNCOMPUTE_STREAM;

			/* Disable all channels enabled on previous bank */
			ret = sdw_dis_chan(sdw_mstr_bs, sdw_mstr_bs_rt);
			if (ret < 0) {
				/* TBD: Undo all the computation */
				dev_err(&sdw_mstr_bs->mstr->dev,
						"Channel disabled failed\n");
				return ret;
			}
		}

	}

	return 0;
}
EXPORT_SYMBOL_GPL(sdw_bus_calc_bw_dis);
