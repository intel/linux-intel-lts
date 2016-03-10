/*
 *  sdw_cnl.c - Intel SoundWire master controller driver implementation.
 *
 *  Copyright (C) 2015-2016 Intel Corp
 *  Author:  Hardik T Shah <hardik.t.shah@intel.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/rtmutex.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/mod_devicetable.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/sdw_bus.h>
#include <linux/sdw/sdw_registers.h>
#include <linux/sdw/sdw_cnl.h>
#include "sdw_cnl_priv.h"

static inline int cnl_sdw_reg_readl(void __iomem *base, int offset)
{
	int value;

	value = readl(base + offset);
	return value;
}

static inline void cnl_sdw_reg_writel(void __iomem *base, int offset, int value)
{
	writel(value, base + offset);
}

static inline u16 cnl_sdw_reg_readw(void __iomem *base, int offset)
{
	int value;

	value = readw(base + offset);
	return value;
}

static inline void cnl_sdw_reg_writew(void __iomem *base, int offset, u16 value)
{
	writew(value, base + offset);
}

static inline int cnl_sdw_port_reg_readl(void __iomem *base, int offset,
						int port_num)
{
	return cnl_sdw_reg_readl(base, offset + port_num * 128);
}

static inline void cnl_sdw_port_reg_writel(u32 __iomem *base, int offset,
						int port_num, int value)
{
	return cnl_sdw_reg_writel(base, offset + port_num * 128, value);
}

struct cnl_sdw {
	struct cnl_sdw_data data;
	struct sdw_master *mstr;
	irqreturn_t (*thread)(int irq, void *context);
	void *thread_context;
	struct completion tx_complete;
	struct cnl_sdw_port port[CNL_SDW_MAX_PORTS];
	int num_pcm_streams;
	struct cnl_sdw_pdi_stream *pcm_streams;
	int num_in_pcm_streams;
	struct cnl_sdw_pdi_stream *in_pcm_streams;
	int num_out_pcm_streams;
	struct cnl_sdw_pdi_stream *out_pcm_streams;
	int num_pdm_streams;
	struct cnl_sdw_pdi_stream *pdm_streams;
	int num_in_pdm_streams;
	struct cnl_sdw_pdi_stream *in_pdm_streams;
	int num_out_pdm_streams;
	struct cnl_sdw_pdi_stream *out_pdm_streams;
	struct mutex	stream_lock;
	spinlock_t ctrl_lock;
	u32 response_buf[0x80];
	bool sdw_link_status;

};


static struct sdw_mstr_driver cnl_sdw_mstr_driver = {
	.driver_type = SDW_DRIVER_TYPE_MASTER,
	.driver = {
		.name   = "cnl_sdw_mstr",
	},
};

static int __init cnl_sdw_init(void)
{
	return sdw_mstr_driver_register(&cnl_sdw_mstr_driver);
}
module_init(cnl_sdw_init);

static void cnl_sdw_exit(void)
{
	sdw_mstr_driver_unregister(&cnl_sdw_mstr_driver);
}
module_exit(cnl_sdw_exit);

MODULE_DESCRIPTION("Intel SoundWire Master Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hardik Shah <hardik.t.shah@intel.com>");
