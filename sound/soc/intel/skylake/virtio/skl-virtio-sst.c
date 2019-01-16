// SPDX-License-Identifier: GPL-2.0+
//
// skl-virtio-sst.c  --  DSP paravirtualization for SKL architecture
//
// Copyright (C) 2018 Intel Corporation.
//
// Authors: Furtak, Pawel <pawel.furtak@intel.com>
//          Janca, Grzegorz <grzegorz.janca@intel.com>
//
//  Dummy implementation of HDA DSP library used on FE side.

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>

#include "skl-virtio-fe.h"
#include "skl-virtio.h"
#include "../../common/sst-dsp.h"
#include "../skl-fwlog.h"
#include "../skl-sst-ipc.h"

#define BXT_ADSP_SRAM0_BASE	0x80000

#define BXT_ADSP_SRAM1_BASE	0xA0000

#define BXT_ADSP_FW_BIN_HDR_OFFSET 0x2000

static unsigned int vfe_get_errorcode(struct sst_dsp *ctx)
{
	return 0;
}

int
vfe_load_library(struct sst_dsp *ctx, struct skl_lib_info *linfo, int lib_count)
{
	struct firmware stripped_fw;
	struct skl_sst *skl = ctx->thread_context;
	int ret = 0, i;

	/* library indices start from 1 to N. 0 represents base FW */
	for (i = 1; i < lib_count; i++) {
		ret = skl_prepare_lib_load(skl, &skl->lib_info[i], &stripped_fw,
					BXT_ADSP_FW_BIN_HDR_OFFSET, i);
		if (ret < 0)
			break;
	}

	return ret;
}

static int vfe_load_base_firmware(struct sst_dsp *ctx)
{
	struct skl_sst *skl = ctx->thread_context;
	int ret;

	dev_dbg(ctx->dev, "Request FW name:%s\n", ctx->fw_name);
	if (ctx->fw == NULL) {
		ret = request_firmware(&ctx->fw, ctx->fw_name, ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "Request firmware failed %d\n", ret);
			return ret;
		}
	}

	/* prase uuids on first boot */
	if (skl->is_first_boot) {
		ret = snd_skl_parse_uuids(ctx, ctx->fw,
				BXT_ADSP_FW_BIN_HDR_OFFSET, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}



int vfe_schedule_dsp_D0i3(struct sst_dsp *ctx)
{
	return 0;
}

int vfe_set_dsp_D0i0(struct sst_dsp *ctx)
{
	return 0;
}

static int vfe_set_dsp_D0(struct sst_dsp *ctx, unsigned int core_id)
{
	return 0;
}

static int vfe_set_dsp_D3(struct sst_dsp *ctx, unsigned int core_id)
{
	return 0;
}

irqreturn_t vfe_dsp_irq_thread_handler(int irq, void *context)
{
	return IRQ_HANDLED;
}

irqreturn_t vfe_dsp_sst_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

void vfe_enable_miscbdcge(struct device *dev, bool enable)
{
}

void vfe_clock_power_gating(struct device *dev, bool enable)
{
}

static const struct skl_dsp_fw_ops vfe_fw_ops = {
	.set_state_D0 = vfe_set_dsp_D0,
	.set_state_D3 = vfe_set_dsp_D3,
	.set_state_D0i3 = vfe_schedule_dsp_D0i3,
	.set_state_D0i0 = vfe_set_dsp_D0i0,
	.load_fw = vfe_load_base_firmware,
	.get_fw_errcode = vfe_get_errorcode,
	.load_library = vfe_load_library,
};

static struct sst_ops vfe_ops = {
	.irq_handler = vfe_dsp_sst_interrupt,
	.write = sst_shim32_write,
	.read = sst_shim32_read,
	.ram_read = sst_memcpy_fromio_32,
	.ram_write = sst_memcpy_toio_32,
	.free = skl_dsp_free,
};

static struct sst_dsp_device vfe_dev = {
	.thread = vfe_dsp_irq_thread_handler,
	.ops = &vfe_ops,
};

void vfe_ipc_tx_data_copy(struct ipc_message *msg, char *tx_data,
		size_t tx_size)
{
	if (tx_size)
		memcpy(msg->tx_data, tx_data, tx_size);
}

static void vfe_ipc_tx_msg(struct sst_generic_ipc *ipc, struct ipc_message *msg)
{
	struct skl_sst *skl_sst = container_of(ipc, struct skl_sst, ipc);
	struct snd_skl_vfe *vfe = dev_get_drvdata(skl_sst->dev);

	vfe->send_dsp_ipc_msg(vfe, msg);
}

int vfe_ipc_init(struct device *dev, struct skl_sst *skl)
{
	struct sst_generic_ipc *ipc;
	int err;

	ipc = &skl->ipc;
	ipc->dsp = skl->dsp;
	ipc->dev = dev;

	ipc->tx_data_max_size = SKL_ADSP_W1_SZ;
	ipc->rx_data_max_size = SKL_ADSP_W0_UP_SZ;

	err = sst_ipc_init(ipc);
	if (err)
		return err;

	ipc->ops.tx_msg = vfe_ipc_tx_msg;
	ipc->ops.tx_data_copy = vfe_ipc_tx_data_copy;

	return 0;
}

struct sst_dsp *vfe_dsp_ctx_init(struct device *dev,
		struct sst_dsp_device *sst_dev, int irq)
{
	int ret;
	struct sst_dsp *sst;

	sst = devm_kzalloc(dev, sizeof(*sst), GFP_KERNEL);
	if (sst == NULL)
		return NULL;

	spin_lock_init(&sst->spinlock);
	mutex_init(&sst->mutex);
	sst->dev = dev;
	sst->sst_dev = sst_dev;
	sst->irq = irq;
	sst->ops = sst_dev->ops;
	sst->thread_context = sst_dev->thread_context;

	/* Initialise SST Audio DSP */
	if (sst->ops->init) {
		ret = sst->ops->init(sst, NULL);
		if (ret < 0)
			return NULL;
	}

	return sst;
}

int vfe_sst_ctx_init(struct device *dev, int irq, const char *fw_name,
	struct skl_dsp_loader_ops dsp_ops, struct skl_sst **dsp,
	struct sst_dsp_device *skl_dev)
{
	struct skl_sst *skl;
	struct sst_dsp *sst;

	skl = devm_kzalloc(dev, sizeof(*skl), GFP_KERNEL);
	if (skl == NULL)
		return -ENOMEM;

	skl->dev = dev;
	skl_dev->thread_context = skl;
	INIT_LIST_HEAD(&skl->uuid_list);
	skl->dsp = vfe_dsp_ctx_init(dev, skl_dev, irq);
	if (!skl->dsp) {
		dev_err(skl->dev, "%s: no device\n", __func__);
		return -ENODEV;
	}

	sst = skl->dsp;
	sst->fw_name = fw_name;
	sst->dsp_ops = dsp_ops;
	init_waitqueue_head(&skl->mod_load_wait);
	INIT_LIST_HEAD(&sst->module_list);

	skl->is_first_boot = true;
	if (dsp)
		*dsp = skl;

	return 0;
}

int vfe_sst_dsp_init(struct device *dev, void __iomem *mmio_base, int irq,
			const char *fw_name, struct skl_dsp_loader_ops dsp_ops,
			struct skl_sst **dsp, void *ptr)
{
	struct skl_sst *skl;
	struct sst_dsp *sst;
	int ret;

	ret = vfe_sst_ctx_init(dev, irq, fw_name, dsp_ops, dsp, &vfe_dev);
	if (ret < 0) {
		dev_err(dev, "%s: no device\n", __func__);
		return ret;
	}

	skl = *dsp;
	sst = skl->dsp;
	sst->fw_ops = vfe_fw_ops;
	sst->addr.lpe = mmio_base;
	sst->addr.shim = mmio_base;
	sst->addr.sram0_base = BXT_ADSP_SRAM0_BASE;
	sst->addr.sram1_base = BXT_ADSP_SRAM1_BASE;
	sst->addr.w0_stat_sz = SKL_ADSP_W0_STAT_SZ;
	sst->addr.w0_up_sz = SKL_ADSP_W0_UP_SZ;

	skl->enable_miscbdcge = vfe_enable_miscbdcge;
	skl->clock_power_gating = vfe_clock_power_gating;

	ret = vfe_ipc_init(dev, skl);
	if (ret) {
		skl_dsp_free(sst);
		return ret;
	}

	skl->boot_complete = false;
	init_waitqueue_head(&skl->boot_wait);
	skl->d0i3.state = SKL_DSP_D0I3_NONE;

	return 0;
}
EXPORT_SYMBOL_GPL(vfe_sst_dsp_init);

int vfe_sst_init_fw(struct device *dev, struct skl_sst *ctx)
{
	int ret;
	struct sst_dsp *sst = ctx->dsp;

	ret = sst->fw_ops.load_fw(sst);
	if (ret < 0) {
		dev_err(dev, "Load base fw failed: %x\n", ret);
		return ret;
	}

	if (ctx->lib_count > 1) {
		ret = sst->fw_ops.load_library(sst, ctx->lib_info,
						ctx->lib_count);
		if (ret < 0) {
			dev_err(dev, "Load Library failed : %x\n", ret);
			return ret;
		}
	}
	ctx->is_first_boot = false;

	return 0;
}
EXPORT_SYMBOL_GPL(vfe_sst_init_fw);

void vfe_sst_dsp_cleanup(struct device *dev, struct skl_sst *ctx)
{
}
EXPORT_SYMBOL_GPL(vfe_sst_dsp_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Virtio FE IPC driver");
