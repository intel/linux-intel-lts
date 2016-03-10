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

static int sdw_power_up_link(struct cnl_sdw *sdw)
{
	volatile int link_control;
	struct sdw_master *mstr = sdw->mstr;
	struct cnl_sdw_data *data = &sdw->data;
	/* Try 10 times before timing out */
	int timeout = 10;
	int spa_mask, cpa_mask;

	link_control = cnl_sdw_reg_readl(data->sdw_shim, SDW_CNL_LCTL);
	spa_mask = (CNL_LCTL_SPA_MASK << (data->inst_id + CNL_LCTL_SPA_SHIFT));
	cpa_mask = (CNL_LCTL_CPA_MASK << (data->inst_id + CNL_LCTL_CPA_SHIFT));
	link_control |=  spa_mask;
	cnl_sdw_reg_writel(data->sdw_shim, SDW_CNL_LCTL, link_control);
	do {
		link_control = cnl_sdw_reg_readl(data->sdw_shim, SDW_CNL_LCTL);
		if (link_control & cpa_mask)
			break;
		timeout--;
		/* Wait 20ms before each time */
		msleep(20);
	} while (timeout != 0);
	/* Read once again to confirm */
	link_control = cnl_sdw_reg_readl(data->sdw_shim, SDW_CNL_LCTL);
	if (link_control & cpa_mask) {
		dev_info(&mstr->dev, "SoundWire ctrl %d Powered Up\n",
						data->inst_id);
		sdw->sdw_link_status = 1;
		return 0;
	}
	dev_err(&mstr->dev, "Failed to Power Up the SDW ctrl %d\n",
								data->inst_id);
	return -EIO;
}

static void sdw_power_down_link(struct cnl_sdw *sdw)
{
	volatile int link_control;
	struct sdw_master *mstr = sdw->mstr;
	struct cnl_sdw_data *data = &sdw->data;
	/* Retry 10 times before giving up */
	int timeout = 10;
	int spa_mask, cpa_mask;

	link_control = cnl_sdw_reg_readl(data->sdw_shim, SDW_CNL_LCTL);
	spa_mask = ~(CNL_LCTL_SPA_MASK << (data->inst_id + CNL_LCTL_SPA_SHIFT));
	cpa_mask = (CNL_LCTL_CPA_MASK << (data->inst_id + CNL_LCTL_CPA_SHIFT));
	link_control &=  spa_mask;
	cnl_sdw_reg_writel(data->sdw_shim, SDW_CNL_LCTL, link_control);
	do {
		link_control = cnl_sdw_reg_readl(data->sdw_shim, SDW_CNL_LCTL);
		if (!(link_control & cpa_mask))
			break;
		timeout--;
		/* Wait for 20ms before each retry */
		msleep(20);
	} while (timeout != 0);
	/* Read once again to confirm */
	link_control = cnl_sdw_reg_readl(data->sdw_shim, SDW_CNL_LCTL);
	if (!(link_control & cpa_mask)) {
		dev_info(&mstr->dev, "SoundWire ctrl %d Powered Down\n",
						data->inst_id);
		sdw->sdw_link_status = 0;
		return;
	}
	dev_err(&mstr->dev, "Failed to Power Down the SDW ctrl %d\n",
								data->inst_id);
}

static void sdw_init_phyctrl(struct cnl_sdw *sdw)
{
	/* TODO: Initialize based on hardware requirement */

}

static void sdw_init_shim(struct cnl_sdw *sdw)
{
	struct cnl_sdw_data *data = &sdw->data;
	int act_offset = SDW_CNL_CTMCTL + (data->inst_id *
					SDW_CNL_CTMCTL_REG_OFFSET);
	int ioctl_offset = SDW_CNL_IOCTL + (data->inst_id *
					SDW_CNL_IOCTL_REG_OFFSET);
	u16 act = 0;
	u16 ioctl = 0;


	ioctl |= CNL_IOCTL_MIF_MASK << CNL_IOCTL_MIF_SHIFT;
	ioctl |= CNL_IOCTL_WPDD_MASK << CNL_IOCTL_WPDD_SHIFT;
	cnl_sdw_reg_writew(data->sdw_shim,  ioctl_offset, ioctl);

	act |= 0x1 << CNL_CTMCTL_DOAIS_SHIFT;
	act |= CNL_CTMCTL_DACTQE_MASK << CNL_CTMCTL_DACTQE_SHIFT;
	act |= CNL_CTMCTL_DODS_MASK << CNL_CTMCTL_DODS_SHIFT;
	cnl_sdw_reg_writew(data->sdw_shim,  act_offset, act);
}

static int sdw_config_update(struct cnl_sdw *sdw)
{
	struct cnl_sdw_data *data = &sdw->data;
	struct sdw_master *mstr = sdw->mstr;

	volatile int config_update = 0;
	/* Try 10 times before giving up on configuration update */
	int timeout = 10;
	int config_updated = 0;

	config_update |= MCP_CONFIGUPDATE_CONFIGUPDATE_MASK <<
				MCP_CONFIGUPDATE_CONFIGUPDATE_SHIFT;
	/* Bit is self-cleared when configuration gets updated. */
	cnl_sdw_reg_writel(data->sdw_regs,  SDW_CNL_MCP_CONFIGUPDATE,
			config_update);
	do {
		config_update = cnl_sdw_reg_readl(data->sdw_regs,
				SDW_CNL_MCP_CONFIGUPDATE);
		if ((config_update &
				MCP_CONFIGUPDATE_CONFIGUPDATE_MASK) == 0) {
			config_updated = 1;
			break;
		}
		timeout--;
		/* Wait for 20ms between each try */
		msleep(20);

	} while (timeout != 0);
	if (!config_updated) {
		dev_err(&mstr->dev, "SoundWire update failed\n");
		return -EIO;
	}
	return 0;
}

static void sdw_enable_interrupt(struct cnl_sdw *sdw)
{
	struct cnl_sdw_data *data = &sdw->data;
	int int_mask = 0;

	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_SLAVEINTMASK0,
						MCP_SLAVEINTMASK0_MASK);
	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_SLAVEINTMASK1,
						MCP_SLAVEINTMASK1_MASK);
	/* Enable slave interrupt mask */
	int_mask |= MCP_INTMASK_SLAVERESERVED_MASK <<
				MCP_INTMASK_SLAVERESERVED_SHIFT;
	int_mask |= MCP_INTMASK_SLAVEALERT_MASK <<
				MCP_INTMASK_SLAVEALERT_SHIFT;
	int_mask |= MCP_INTMASK_SLAVEATTACHED_MASK <<
				MCP_INTMASK_SLAVEATTACHED_SHIFT;
	int_mask |= MCP_INTMASK_SLAVENOTATTACHED_MASK <<
				MCP_INTMASK_SLAVENOTATTACHED_SHIFT;
	int_mask |= MCP_INTMASK_CONTROLBUSCLASH_MASK <<
				MCP_INTMASK_CONTROLBUSCLASH_SHIFT;
	int_mask |= MCP_INTMASK_DATABUSCLASH_MASK <<
				MCP_INTMASK_DATABUSCLASH_SHIFT;
	int_mask |= MCP_INTMASK_RXWL_MASK <<
				MCP_INTMASK_RXWL_SHIFT;
	int_mask |= MCP_INTMASK_IRQEN_MASK <<
				MCP_INTMASK_IRQEN_SHIFT;
	int_mask |= MCP_INTMASK_DPPDIINT_MASK <<
				MCP_INTMASK_DPPDIINT_SHIFT;
	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_INTMASK, int_mask);
}

static int sdw_port_pdi_init(struct cnl_sdw *sdw)
{
	return 0;
}
static int sdw_init(struct cnl_sdw *sdw)
{
	struct sdw_master *mstr = sdw->mstr;
	struct cnl_sdw_data *data = &sdw->data;
	int mcp_config;
	int ret = 0;

	/* Power up the link controller */
	ret = sdw_power_up_link(sdw);
	if (ret)
		return ret;

	/* Read shim registers for getting capability */
	sdw_init_shim(sdw);


	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_FRAMESHAPEINIT, 0x48);

	mcp_config = cnl_sdw_reg_readl(data->sdw_regs, SDW_CNL_MCP_CONFIG);
	/* Set Max cmd retry to 15 times */
	mcp_config |= (CNL_SDW_MAX_CMD_RETRIES <<
				MCP_CONFIG_MAXCMDRETRY_SHIFT);

	/* Set Ping request to ping delay to 15 frames.
	 * Spec supports 32 max frames
	 */
	mcp_config |= (CNL_SDW_MAX_PREQ_DELAY <<
					MCP_CONFIG_MAXPREQDELAY_SHIFT);

	/* If master is synchronized to some other master set Multimode */
	if (mstr->link_sync_mask) {
		mcp_config |= (MCP_CONFIG_MMMODEEN_MASK <<
						MCP_CONFIG_MMMODEEN_SHIFT);
		mcp_config |= (MCP_CONFIG_SSPMODE_MASK <<
						MCP_CONFIG_SSPMODE_SHIFT);
	} else {
		mcp_config &= ~(MCP_CONFIG_MMMODEEN_MASK <<
						MCP_CONFIG_MMMODEEN_SHIFT);
		mcp_config &= ~(MCP_CONFIG_SSPMODE_MASK <<
						MCP_CONFIG_SSPMODE_SHIFT);
	}

	/* Disable automatic bus release */
	mcp_config &= ~(MCP_CONFIG_BRELENABLE_MASK <<
				MCP_CONFIG_BRELENABLE_SHIFT);

	/* Disable sniffer mode now */
	mcp_config &= ~(MCP_CONFIG_SNIFFEREN_MASK <<
				MCP_CONFIG_SNIFFEREN_SHIFT);

	/* Set the command mode for Tx and Rx command */
	mcp_config &= ~(MCP_CONFIG_CMDMODE_MASK <<
				MCP_CONFIG_CMDMODE_SHIFT);

	/* Set operation mode to normal */
	mcp_config &= ~(MCP_CONFIG_OPERATIONMODE_MASK <<
				MCP_CONFIG_OPERATIONMODE_SHIFT);
	mcp_config |= ((MCP_CONFIG_OPERATIONMODE_NORMAL &
			MCP_CONFIG_OPERATIONMODE_MASK) <<
			MCP_CONFIG_OPERATIONMODE_SHIFT);

	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_CONFIG, mcp_config);
	/* Set the SSP interval to 32 for both banks */
	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_SSPCTRL0,
					SDW_CNL_DEFAULT_SSP_INTERVAL);
	cnl_sdw_reg_writel(data->sdw_regs, SDW_CNL_MCP_SSPCTRL1,
					SDW_CNL_DEFAULT_SSP_INTERVAL);

	/* Initialize the phy control registers. */
	sdw_init_phyctrl(sdw);

	/* Initlaize the ports */
	ret = sdw_port_pdi_init(sdw);
	if (ret) {
		dev_err(&mstr->dev, "SoundWire controller init failed %d\n",
				data->inst_id);
		sdw_power_down_link(sdw);
		return ret;
	}

	/* Lastly enable interrupts */
	sdw_enable_interrupt(sdw);

	/* Update soundwire configuration */
	return sdw_config_update(sdw);
}

irqreturn_t cnl_sdw_irq_handler(int irq, void *context)
{
	return IRQ_HANDLED;
}

static int cnl_sdw_probe(struct sdw_master *mstr,
				const struct sdw_master_id *sdw_id)
{
	struct cnl_sdw *sdw;
	int ret = 0;
	struct cnl_sdw_data *data = mstr->dev.platform_data;

	sdw = devm_kzalloc(&mstr->dev, sizeof(*sdw), GFP_KERNEL);
	if (!sdw) {
		ret = -ENOMEM;
		return ret;
	}
	dev_info(&mstr->dev,
		"Controller Resources ctrl_base = %p shim=%p irq=%d inst_id=%d\n",
		data->sdw_regs, data->sdw_shim, data->irq, data->inst_id);
	sdw->data.sdw_regs = data->sdw_regs;
	sdw->data.sdw_shim = data->sdw_shim;
	sdw->data.irq = data->irq;
	sdw->data.inst_id = data->inst_id;
	sdw->data.alh_base = data->alh_base;
	sdw->mstr = mstr;
	spin_lock_init(&sdw->ctrl_lock);
	sdw_master_set_drvdata(mstr, sdw);
	init_completion(&sdw->tx_complete);
	mutex_init(&sdw->stream_lock);
	ret = sdw_init(sdw);
	if (ret) {
		dev_err(&mstr->dev, "SoundWire controller init failed %d\n",
				data->inst_id);
		return ret;
	}
	ret = devm_request_irq(&mstr->dev,
		sdw->data.irq, cnl_sdw_irq_handler, IRQF_SHARED, "SDW", sdw);
	if (ret) {
		dev_err(&mstr->dev, "unable to grab IRQ %d, disabling device\n",
			       sdw->data.irq);
		sdw_power_down_link(sdw);
		return ret;
	}

	return ret;
}

static int cnl_sdw_remove(struct sdw_master *mstr)
{
	struct cnl_sdw *sdw = sdw_master_get_drvdata(mstr);

	sdw_power_down_link(sdw);

	return 0;
}

static struct sdw_mstr_driver cnl_sdw_mstr_driver = {
	.driver_type = SDW_DRIVER_TYPE_MASTER,
	.driver = {
		.name   = "cnl_sdw_mstr",
	},
	.probe          = cnl_sdw_probe,
	.remove         = cnl_sdw_remove,
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
