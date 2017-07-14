/*
 * Support for Intel Camera Imaging ISP subsystem.
* Copyright (c) 2010 - 2017, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
 */

#ifndef __IPU_DEVICE_CELL_DEVICES_H
#define __IPU_DEVICE_CELL_DEVICES_H

#define SPC0_CELL  ipu_sp_control_tile_ps_sp
#define SPP0_CELL  ipu_sp_proxy_tile_ps_0_sp
#define SPP1_CELL  ipu_sp_proxy_tile_ps_1_sp
#define ISP0_CELL  ipu_par_idsp_0_idsp_tile_top_idsp
#define ISP1_CELL  ipu_par_idsp_1_idsp_tile_top_idsp
#define ISP2_CELL  ipu_par_idsp_2_idsp_tile_top_idsp

enum ipu_device_psys_cell_id {
	SPC0,
	SPP0,
	SPP1,
	ISP0,
	ISP1,
	ISP2
};
#define NUM_CELLS     (ISP2 + 1)
#define NUM_ISP_CELLS 3

#endif /* __IPU_DEVICE_CELL_DEVICES_H */
