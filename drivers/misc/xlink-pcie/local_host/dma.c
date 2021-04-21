// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include "epf.h"

#define DMA_DBI_OFFSET			(0x380000)

/* PCIe DMA control 1 register definitions. */
#define DMA_CH_CONTROL1_CB_SHIFT	(0)
#define DMA_CH_CONTROL1_TCB_SHIFT	(1)
#define DMA_CH_CONTROL1_LLP_SHIFT	(2)
#define DMA_CH_CONTROL1_LIE_SHIFT	(3)
#define DMA_CH_CONTROL1_CS_SHIFT	(5)
#define DMA_CH_CONTROL1_CCS_SHIFT	(8)
#define DMA_CH_CONTROL1_LLE_SHIFT	(9)
#define DMA_CH_CONTROL1_CB_MASK		(BIT(DMA_CH_CONTROL1_CB_SHIFT))
#define DMA_CH_CONTROL1_TCB_MASK	(BIT(DMA_CH_CONTROL1_TCB_SHIFT))
#define DMA_CH_CONTROL1_LLP_MASK	(BIT(DMA_CH_CONTROL1_LLP_SHIFT))
#define DMA_CH_CONTROL1_LIE_MASK	(BIT(DMA_CH_CONTROL1_LIE_SHIFT))
#define DMA_CH_CONTROL1_CS_MASK		(0x3 << DMA_CH_CONTROL1_CS_SHIFT)
#define DMA_CH_CONTROL1_CCS_MASK	(BIT(DMA_CH_CONTROL1_CCS_SHIFT))
#define DMA_CH_CONTROL1_LLE_MASK	(BIT(DMA_CH_CONTROL1_LLE_SHIFT))

/* DMA control 1 register Channel Status */
#define DMA_CH_CONTROL1_CS_RUNNING	(0x1 << DMA_CH_CONTROL1_CS_SHIFT)
#define DMA_CH_CONTROL1_CS_HALTED	(0x2 << DMA_CH_CONTROL1_CS_SHIFT)
#define DMA_CH_CONTROL1_CS_STOPPED	(0x3 << DMA_CH_CONTROL1_CS_SHIFT)

/* PCIe DMA Engine enable register definitions. */
#define DMA_ENGINE_EN_SHIFT		(0)
#define DMA_ENGINE_EN_MASK		(BIT(DMA_ENGINE_EN_SHIFT))

/* PCIe DMA interrupt registers definitions. */
#define DMA_ABORT_INTERRUPT_SHIFT	(16)
#define DMA_ABORT_INTERRUPT_MASK	(0xFF << DMA_ABORT_INTERRUPT_SHIFT)
#define DMA_ABORT_INTERRUPT_CH_MASK(_c) (BIT(_c) << DMA_ABORT_INTERRUPT_SHIFT)
#define DMA_DONE_INTERRUPT_MASK		(0xFF)
#define DMA_DONE_INTERRUPT_CH_MASK(_c)	(BIT(_c))
#define DMA_ALL_INTERRUPT_MASK \
	(DMA_ABORT_INTERRUPT_MASK | DMA_DONE_INTERRUPT_MASK)

#define DMA_LL_ERROR_SHIFT		(16)
#define DMA_CPL_ABORT_SHIFT		(8)
#define DMA_CPL_TIMEOUT_SHIFT		(16)
#define DMA_DATA_POI_SHIFT		(24)
#define DMA_AR_ERROR_CH_MASK(_c)	(BIT(_c))
#define DMA_LL_ERROR_CH_MASK(_c)	(BIT(_c) << DMA_LL_ERROR_SHIFT)
#define DMA_UNREQ_ERROR_CH_MASK(_c)	(BIT(_c))
#define DMA_CPL_ABORT_ERROR_CH_MASK(_c)	(BIT(_c) << DMA_CPL_ABORT_SHIFT)
#define DMA_CPL_TIMEOUT_ERROR_CH_MASK(_c) (BIT(_c) << DMA_CPL_TIMEOUT_SHIFT)
#define DMA_DATA_POI_ERROR_CH_MASK(_c)	(BIT(_c) << DMA_DATA_POI_SHIFT)

#define DMA_LLLAIE_SHIFT		(16)
#define DMA_LLLAIE_MASK			(0xF << DMA_LLLAIE_SHIFT)

#define DMA_CHAN_WRITE_MAX_WEIGHT	(0x7)
#define DMA_CHAN_READ_MAX_WEIGHT	(0x3)
#define DMA_CHAN0_WEIGHT_OFFSET		(0)
#define DMA_CHAN1_WEIGHT_OFFSET		(5)
#define DMA_CHAN2_WEIGHT_OFFSET		(10)
#define DMA_CHAN3_WEIGHT_OFFSET		(15)
#define DMA_CHAN_WRITE_ALL_MAX_WEIGHT					\
	((DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN0_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN1_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN2_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN3_WEIGHT_OFFSET))
#define DMA_CHAN_READ_ALL_MAX_WEIGHT					\
	((DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN0_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN1_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN2_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN3_WEIGHT_OFFSET))

#define PCIE_REGS_PCIE_APP_CNTRL	0x8
#define APP_XFER_PENDING		BIT(6)
#define PCIE_REGS_PCIE_SII_PM_STATE_1	0xb4
#define PM_LINKST_IN_L1			BIT(10)

#define DMA_POLLING_TIMEOUT		1000000
#define DMA_ENABLE_TIMEOUT		1000
#define DMA_PCIE_PM_L1_TIMEOUT		20

struct __packed pcie_dma_reg {
	u32 dma_ctrl_data_arb_prior;
	u32 reserved1;
	u32 dma_ctrl;
	u32 dma_write_engine_en;
	u32 dma_write_doorbell;
	u32 reserved2;
	u32 dma_write_channel_arb_weight_low;
	u32 dma_write_channel_arb_weight_high;
	u32 reserved3[3];
	u32 dma_read_engine_en;
	u32 dma_read_doorbell;
	u32 reserved4;
	u32 dma_read_channel_arb_weight_low;
	u32 dma_read_channel_arb_weight_high;
	u32 reserved5[3];
	u32 dma_write_int_status;
	u32 reserved6;
	u32 dma_write_int_mask;
	u32 dma_write_int_clear;
	u32 dma_write_err_status;
	u32 dma_write_done_imwr_low;
	u32 dma_write_done_imwr_high;
	u32 dma_write_abort_imwr_low;
	u32 dma_write_abort_imwr_high;
	u16 dma_write_ch_imwr_data[8];
	u32 reserved7[4];
	u32 dma_write_linked_list_err_en;
	u32 reserved8[3];
	u32 dma_read_int_status;
	u32 reserved9;
	u32 dma_read_int_mask;
	u32 dma_read_int_clear;
	u32 reserved10;
	u32 dma_read_err_status_low;
	u32 dma_rd_err_sts_h;
	u32 reserved11[2];
	u32 dma_read_linked_list_err_en;
	u32 reserved12;
	u32 dma_read_done_imwr_low;
	u32 dma_read_done_imwr_high;
	u32 dma_read_abort_imwr_low;
	u32 dma_read_abort_imwr_high;
	u16 dma_read_ch_imwr_data[8];
};

struct __packed pcie_dma_chan {
	u32 dma_ch_control1;
	u32 reserved1;
	u32 dma_transfer_size;
	u32 dma_sar_low;
	u32 dma_sar_high;
	u32 dma_dar_low;
	u32 dma_dar_high;
	u32 dma_llp_low;
	u32 dma_llp_high;
};

enum xpcie_ep_engine_type {
	WRITE_ENGINE,
	READ_ENGINE
};

static u32 dma_chan_offset[2][DMA_CHAN_NUM] = {
	{ 0x200, 0x400, 0x600, 0x800 },
	{ 0x300, 0x500, 0x700, 0x900 }
};

/* Function declarations */
int intel_xpcie_ep_dma_reset_write_engine(struct pci_epf *epf);
int intel_xpcie_ep_dma_reset_read_engine(struct pci_epf *epf);

static void __iomem *intel_xpcie_ep_get_dma_base(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct xpcie_epf *xpcie_epf = (struct xpcie_epf *)dev->driver_data;

	return xpcie_epf->dbi_base + DMA_DBI_OFFSET;
}

static int intel_xpcie_ep_dma_disable(void __iomem *dma_base,
				      enum xpcie_ep_engine_type rw)
{
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)dma_base;
	void __iomem *int_mask, *int_clear;
	void __iomem *engine_en, *ll_err;
	int i;

	if (rw == WRITE_ENGINE) {
		engine_en = (void __iomem *)&dma_reg->dma_write_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_write_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_write_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_write_linked_list_err_en;
	} else {
		engine_en = (void __iomem *)&dma_reg->dma_read_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_read_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_read_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_read_linked_list_err_en;
	}

	iowrite32(0x0, engine_en);

	/* Mask all interrupts. */
	iowrite32(DMA_ALL_INTERRUPT_MASK, int_mask);

	/* Clear all interrupts. */
	iowrite32(DMA_ALL_INTERRUPT_MASK, int_clear);

	/* Disable LL abort interrupt (LLLAIE). */
	iowrite32(0, ll_err);

	/* Wait until the engine is disabled. */
	for (i = 0; i < DMA_ENABLE_TIMEOUT; i++) {
		if (!(ioread32(engine_en) & DMA_ENGINE_EN_MASK))
			return 0;
		msleep(20);
	}

	return -EBUSY;
}

static void intel_xpcie_ep_dma_enable(void __iomem *dma_base,
				      enum xpcie_ep_engine_type rw)
{
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)(dma_base);
	void __iomem *engine_en, *ll_err, *arb_weight;
	struct __iomem pcie_dma_chan * dma_chan;
	void __iomem *int_mask, *int_clear;
	u32 offset, weight;
	int i;

	if (rw == WRITE_ENGINE) {
		engine_en = (void __iomem *)&dma_reg->dma_write_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_write_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_write_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_write_linked_list_err_en;
		arb_weight = (void __iomem *)
			     &dma_reg->dma_write_channel_arb_weight_low;
		weight = DMA_CHAN_WRITE_ALL_MAX_WEIGHT;
	} else {
		engine_en = (void __iomem *)&dma_reg->dma_read_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_read_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_read_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_read_linked_list_err_en;
		arb_weight = (void __iomem *)
			     &dma_reg->dma_read_channel_arb_weight_low;
		weight = DMA_CHAN_READ_ALL_MAX_WEIGHT;
	}

	iowrite32(DMA_ENGINE_EN_MASK, engine_en);

	/* Unmask all interrupts, so that the interrupt line gets asserted. */
	iowrite32(~(u32)DMA_ALL_INTERRUPT_MASK, int_mask);

	/* Clear all interrupts. */
	iowrite32(DMA_ALL_INTERRUPT_MASK, int_clear);

	/* Set channel round robin weight. */
	iowrite32(weight, arb_weight);

	/* Enable LL abort interrupt (LLLAIE). */
	iowrite32(DMA_LLLAIE_MASK, ll_err);

	/* Enable linked list mode. */
	for (i = 0; i < DMA_CHAN_NUM; i++) {
		offset = dma_chan_offset[rw][i];
		dma_chan = (struct __iomem pcie_dma_chan *)(dma_base + offset);
		iowrite32(DMA_CH_CONTROL1_LLE_MASK,
			  (void __iomem *)&dma_chan->dma_ch_control1);
	}
}

/*
 * Make sure EP is not in L1 state when DMA doorbell.
 * The DMA controller may start the wrong channel if doorbell occurs at the
 * same time as controller is transitioning to L1.
 */
static int intel_xpcie_ep_dma_doorbell(struct xpcie_epf *xpcie_epf, int chan,
				       void __iomem *doorbell)
{
	int i = DMA_PCIE_PM_L1_TIMEOUT, rc = 0;
	u32 val, pm_val;

	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
	iowrite32(val | APP_XFER_PENDING,
		  xpcie_epf->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
	pm_val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_SII_PM_STATE_1);
	while (pm_val & PM_LINKST_IN_L1) {
		if (i-- < 0) {
			rc = -ETIME;
			break;
		}
		udelay(5);
		pm_val = ioread32(xpcie_epf->apb_base +
				  PCIE_REGS_PCIE_SII_PM_STATE_1);
	}

	iowrite32((u32)chan, doorbell);

	iowrite32(val & ~APP_XFER_PENDING,
		  xpcie_epf->apb_base + PCIE_REGS_PCIE_APP_CNTRL);

	return rc;
}

static int intel_xpcie_ep_dma_err_status(void __iomem *err_status, int chan)
{
	if (ioread32(err_status) &
	    (DMA_AR_ERROR_CH_MASK(chan) | DMA_LL_ERROR_CH_MASK(chan)))
		return -EIO;

	return 0;
}

static int intel_xpcie_ep_dma_rd_err_sts_h(void __iomem *err_status,
					   int chan)
{
	if (ioread32(err_status) &
	    (DMA_UNREQ_ERROR_CH_MASK(chan) |
	     DMA_CPL_ABORT_ERROR_CH_MASK(chan) |
	     DMA_CPL_TIMEOUT_ERROR_CH_MASK(chan) |
	     DMA_DATA_POI_ERROR_CH_MASK(chan)))
		return -EIO;

	return 0;
}

static void
intel_xpcie_ep_dma_setup_ll_descs(struct __iomem pcie_dma_chan * dma_chan,
				  struct xpcie_dma_ll_desc_buf *desc_buf,
				  int descs_num)
{
	struct xpcie_dma_ll_desc *descs = desc_buf->virt;
	int i;

	/* Setup linked list descriptors */
	for (i = 0; i < descs_num - 1; i++)
		descs[i].dma_ch_control1 = DMA_CH_CONTROL1_CB_MASK;
	descs[descs_num - 1].dma_ch_control1 = DMA_CH_CONTROL1_LIE_MASK |
						DMA_CH_CONTROL1_CB_MASK;
	descs[descs_num].dma_ch_control1 = DMA_CH_CONTROL1_LLP_MASK |
					   DMA_CH_CONTROL1_TCB_MASK;
	descs[descs_num].src_addr = (phys_addr_t)desc_buf->phys;

	/* Setup linked list settings */
	iowrite32(DMA_CH_CONTROL1_LLE_MASK | DMA_CH_CONTROL1_CCS_MASK,
		  (void __iomem *)&dma_chan->dma_ch_control1);
	iowrite32((u32)desc_buf->phys, (void __iomem *)&dma_chan->dma_llp_low);
	iowrite32((u64)desc_buf->phys >> 32,
		  (void __iomem *)&dma_chan->dma_llp_high);
}

enum hrtimer_restart free_rx_dma_timer_cb(struct hrtimer *free_rx_dma_timer)
{
	struct xpcie_epf *xpcie_epf =
		container_of(free_rx_dma_timer, struct xpcie_epf, free_rx_dma_timer);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
	int func_no = xpcie_epf->epf->func_no;
	void __iomem *err_status;

	pr_info("RX_DMA_TIMER_CB---------> START \n");
	err_status = &dma_reg->dma_read_int_status;
	pr_info("RX_DMA_TIMER_CB: dma_read_int_status: 0x%x, func_no: 0x%x\n",
		ioread32(err_status), func_no);
	err_status = &dma_reg->dma_read_err_status_low;
	pr_info("RX_DMA_TIMER_CB: dma_read_err_status_low: 0x%x, func_no: 0x%x\n",
		ioread32(err_status), func_no);

	/* RESET DMA READ_ENGINE */
	intel_xpcie_ep_dma_reset_read_engine(xpcie_epf->epf);

	xpcie_epf->rx_dma_waitcomplete = true;
	pr_info("RX_DMA_TIMER_CB ---------> END \n");

	return HRTIMER_NORESTART;
}

enum hrtimer_restart free_tx_dma_timer_cb(struct hrtimer *free_tx_dma_timer)
{
	struct xpcie_epf *xpcie_epf =
		container_of(free_tx_dma_timer, struct xpcie_epf, free_tx_dma_timer);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
	int func_no = xpcie_epf->epf->func_no;
	void __iomem *err_status;
	int rc = 0;

	pr_info("TX_DMA_TIMER_CB---------> START \n");
	err_status = &dma_reg->dma_write_int_status;
	pr_info("TX_DMA_TIMER_CB: dma_write_int_status: 0x%x, func_no: 0x%x\n",
			ioread32(err_status), func_no);
	rc = intel_xpcie_ep_dma_err_status(&dma_reg->dma_write_err_status, func_no);
	if (rc != 0) {
		pr_info("TX_DMA_TIMER_CB: dma err \n");
	}
	err_status = &dma_reg->dma_write_err_status;
	pr_info("TX_DMA_TIMER_CB: dma_write_err_status: 0x%x, func_no: 0x%x \n",
			ioread32(err_status), func_no);

	/* RESET DMA WRITE_ENGINE */
	intel_xpcie_ep_dma_reset_write_engine(xpcie_epf->epf);

	xpcie_epf->tx_dma_waitcomplete = true;
	pr_info("TX_DMA_TIMER_CB ---------> END \n");

	return HRTIMER_NORESTART;
}

int intel_xpcie_ep_dma_write_ll(struct pci_epf *epf, int chan, int descs_num)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	unsigned long dma_time_period_us = 2000 * 1000; //2 sec delay
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct __iomem pcie_dma_chan * dma_chan;
	struct xpcie_dma_ll_desc_buf *desc_buf;
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)(dma_base);
	ktime_t free_tx_dma_tp;
	int i, rc;

	if (descs_num <= 0 || descs_num > XPCIE_NUM_TX_DESCS)
		return -EINVAL;

	if (chan < 0 || chan >= DMA_CHAN_NUM)
		return -EINVAL;

	dma_chan = (struct __iomem pcie_dma_chan *)
		(dma_base + dma_chan_offset[WRITE_ENGINE][chan]);

	desc_buf = &xpcie_epf->tx_desc_buf[chan];

	intel_xpcie_ep_dma_setup_ll_descs(dma_chan, desc_buf, descs_num);

	if (!hrtimer_active(&xpcie_epf->free_tx_dma_timer)) {
		free_tx_dma_tp = ktime_set(0, dma_time_period_us * 1000ULL);
		hrtimer_start(&xpcie_epf->free_tx_dma_timer,
			      free_tx_dma_tp, HRTIMER_MODE_REL);
		xpcie_epf->tx_dma_waitcomplete = false;
	}

	/* Start DMA transfer. */
	rc = intel_xpcie_ep_dma_doorbell(xpcie_epf, chan,
					 (void __iomem *)
					 &dma_reg->dma_write_doorbell);
	if (rc)
		return rc;

	/* Wait for DMA transfer to complete. */
	for (i = 0; i < DMA_POLLING_TIMEOUT; i++) {
		if ((ioread32((void __iomem *)&dma_reg->dma_write_int_status) &
		     (DMA_DONE_INTERRUPT_CH_MASK(chan) |
		      DMA_ABORT_INTERRUPT_CH_MASK(chan))) ||
			(xpcie_epf->tx_dma_waitcomplete == true)) {
			if (hrtimer_active(&xpcie_epf->free_tx_dma_timer)) {
				hrtimer_cancel(&xpcie_epf->free_tx_dma_timer);
				xpcie_epf->tx_dma_waitcomplete = false;
			}
			break;
		}
		usleep_range(5, 10);
	}

	if (i == DMA_POLLING_TIMEOUT) {
		dev_err(&xpcie_epf->epf->dev, "DMA Wr timeout\n");
		rc = -ETIME;
		goto cleanup;
	}

	rc = intel_xpcie_ep_dma_err_status((void __iomem *)
					   &dma_reg->dma_write_err_status,
					   chan);

cleanup:
	/* Clear the done/abort interrupt. */
	iowrite32((DMA_DONE_INTERRUPT_CH_MASK(chan) |
		   DMA_ABORT_INTERRUPT_CH_MASK(chan)),
		  (void __iomem *)&dma_reg->dma_write_int_clear);

	if (rc) {
		if (intel_xpcie_ep_dma_disable(dma_base, WRITE_ENGINE)) {
			dev_err(&xpcie_epf->epf->dev,
				"failed to disable WR DMA\n");
			return rc;
		}
		intel_xpcie_ep_dma_enable(dma_base, WRITE_ENGINE);
	}

	return rc;
}

int intel_xpcie_ep_dma_read_ll(struct pci_epf *epf, int chan, int descs_num)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	unsigned long dma_time_period_us = 2000 * 1000; //2 sec delay
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct xpcie_dma_ll_desc_buf *desc_buf;
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)(dma_base);
	struct __iomem pcie_dma_chan * dma_chan;
	ktime_t free_rx_dma_tp;
	int i, rc;

	if (descs_num <= 0 || descs_num > XPCIE_NUM_RX_DESCS)
		return -EINVAL;

	if (chan < 0 || chan >= DMA_CHAN_NUM)
		return -EINVAL;

	dma_chan = (struct __iomem pcie_dma_chan *)
		(dma_base + dma_chan_offset[READ_ENGINE][chan]);

	desc_buf = &xpcie_epf->rx_desc_buf[chan];

	intel_xpcie_ep_dma_setup_ll_descs(dma_chan, desc_buf, descs_num);

	if (!hrtimer_active(&xpcie_epf->free_rx_dma_timer)) {
		free_rx_dma_tp = ktime_set(0, dma_time_period_us * 1000ULL);
		hrtimer_start(&xpcie_epf->free_rx_dma_timer,
			      free_rx_dma_tp, HRTIMER_MODE_REL);
		xpcie_epf->rx_dma_waitcomplete = false;
	}

	/* Start DMA transfer. */
	rc = intel_xpcie_ep_dma_doorbell(xpcie_epf, chan,
					 (void __iomem *)
					 &dma_reg->dma_read_doorbell);
	if (rc)
		return rc;

	/* Wait for DMA transfer to complete. */
	for (i = 0; i < DMA_POLLING_TIMEOUT; i++) {
		if ((ioread32((void __iomem *)&dma_reg->dma_read_int_status) &
		     (DMA_DONE_INTERRUPT_CH_MASK(chan) |
		      DMA_ABORT_INTERRUPT_CH_MASK(chan))) ||
			(xpcie_epf->rx_dma_waitcomplete == true)) {
			if (hrtimer_active(&xpcie_epf->free_rx_dma_timer)) {
				hrtimer_cancel(&xpcie_epf->free_rx_dma_timer);
				xpcie_epf->rx_dma_waitcomplete = false;
			}
			break;
		}
		usleep_range(5, 10);
	}
	if (i == DMA_POLLING_TIMEOUT) {
		dev_err(&xpcie_epf->epf->dev, "DMA Rd timeout\n");
		rc = -ETIME;
		goto cleanup;
	}

	rc = intel_xpcie_ep_dma_err_status((void __iomem *)
					   &dma_reg->dma_read_err_status_low,
					   chan);
	if (!rc) {
		rc =
		intel_xpcie_ep_dma_rd_err_sts_h((void __iomem *)
						&dma_reg->dma_rd_err_sts_h,
						chan);
	}
cleanup:
	/* Clear the done/abort interrupt. */
	iowrite32((DMA_DONE_INTERRUPT_CH_MASK(chan) |
		   DMA_ABORT_INTERRUPT_CH_MASK(chan)),
		  (void __iomem *)&dma_reg->dma_read_int_clear);

	if (rc) {
		if (intel_xpcie_ep_dma_disable(dma_base, READ_ENGINE)) {
			dev_err(&xpcie_epf->epf->dev,
				"failed to disable RD DMA\n");
			return rc;
		}
		intel_xpcie_ep_dma_enable(dma_base, READ_ENGINE);
	}

	return rc;
}

static void intel_xpcie_ep_dma_free_ll_descs_mem(struct xpcie_epf *xpcie_epf)
{
	struct device *dma_dev = xpcie_epf->epf->epc->dev.parent;
	int i;

	for (i = 0; i < DMA_CHAN_NUM; i++) {
		if (xpcie_epf->tx_desc_buf[i].virt) {
			dma_free_coherent(dma_dev,
					  xpcie_epf->tx_desc_buf[i].size,
					  xpcie_epf->tx_desc_buf[i].virt,
					  xpcie_epf->tx_desc_buf[i].phys);
		}
		if (xpcie_epf->rx_desc_buf[i].virt) {
			dma_free_coherent(dma_dev,
					  xpcie_epf->rx_desc_buf[i].size,
					  xpcie_epf->rx_desc_buf[i].virt,
					  xpcie_epf->rx_desc_buf[i].phys);
		}

		memset(&xpcie_epf->tx_desc_buf[i], 0,
		       sizeof(struct xpcie_dma_ll_desc_buf));
		memset(&xpcie_epf->rx_desc_buf[i], 0,
		       sizeof(struct xpcie_dma_ll_desc_buf));
	}
}

static int intel_xpcie_ep_dma_alloc_ll_descs_mem(struct xpcie_epf *xpcie_epf)
{
	struct device *dma_dev = xpcie_epf->epf->epc->dev.parent;
	int tx_num = XPCIE_NUM_TX_DESCS + 1;
	int rx_num = XPCIE_NUM_RX_DESCS + 1;
	size_t tx_size, rx_size;
	int i;

	tx_size = tx_num * sizeof(struct xpcie_dma_ll_desc);
	rx_size = rx_num * sizeof(struct xpcie_dma_ll_desc);

	for (i = 0; i < DMA_CHAN_NUM; i++) {
		xpcie_epf->tx_desc_buf[i].virt =
			dma_alloc_coherent(dma_dev, tx_size,
					   &xpcie_epf->tx_desc_buf[i].phys,
					   GFP_KERNEL);
		xpcie_epf->rx_desc_buf[i].virt =
			dma_alloc_coherent(dma_dev, rx_size,
					   &xpcie_epf->rx_desc_buf[i].phys,
					   GFP_KERNEL);

		if (!xpcie_epf->tx_desc_buf[i].virt ||
		    !xpcie_epf->rx_desc_buf[i].virt) {
			intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);
			return -ENOMEM;
		}

		xpcie_epf->tx_desc_buf[i].size = tx_size;
		xpcie_epf->rx_desc_buf[i].size = rx_size;
	}
	return 0;
}

int intel_xpcie_ep_dma_reset_read_engine(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Disable the DMA write engine. */
	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, READ_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, READ_ENGINE);

	return 0;
}

int intel_xpcie_ep_dma_reset_write_engine(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Disable the DMA write engine. */
	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, WRITE_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, WRITE_ENGINE);

	return 0;
}

int intel_xpcie_ep_dma_reset(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Disable the DMA read/write engine. */
	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, WRITE_ENGINE) ||
	    intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, READ_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, WRITE_ENGINE);
	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, READ_ENGINE);

	return 0;
}

int intel_xpcie_ep_dma_uninit(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, WRITE_ENGINE) ||
	    intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, READ_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);

	return 0;
}

int intel_xpcie_ep_dma_init(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	int rc;

	xpcie_epf->dma_base = intel_xpcie_ep_get_dma_base(epf);

	rc = intel_xpcie_ep_dma_alloc_ll_descs_mem(xpcie_epf);
	if (rc)
		return rc;

	return intel_xpcie_ep_dma_reset(epf);
}
