// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay OCS AES Crypto Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "ocs-aes.h"

#define AES_COMMAND_OFFSET			0x0000
#define AES_KEY_0_OFFSET			0x0004
#define AES_KEY_1_OFFSET			0x0008
#define AES_KEY_2_OFFSET			0x000C
#define AES_KEY_3_OFFSET			0x0010
#define AES_KEY_4_OFFSET			0x0014
#define AES_KEY_5_OFFSET			0x0018
#define AES_KEY_6_OFFSET			0x001C
#define AES_KEY_7_OFFSET			0x0020
#define AES_IV_0_OFFSET				0x0024
#define AES_IV_1_OFFSET				0x0028
#define AES_IV_2_OFFSET				0x002C
#define AES_IV_3_OFFSET				0x0030
#define AES_ACTIVE_OFFSET			0x0034
#define AES_STATUS_OFFSET			0x0038
#define AES_KEY_SIZE_OFFSET			0x0044
#define AES_IER_OFFSET				0x0048
#define AES_ISR_OFFSET				0x005C
#define AES_MULTIPURPOSE1_0_OFFSET		0x0200
#define AES_MULTIPURPOSE1_1_OFFSET		0x0204
#define AES_MULTIPURPOSE1_2_OFFSET		0x0208
#define AES_MULTIPURPOSE1_3_OFFSET		0x020C
#define AES_MULTIPURPOSE2_0_OFFSET		0x0220
#define AES_MULTIPURPOSE2_1_OFFSET		0x0224
#define AES_MULTIPURPOSE2_2_OFFSET		0x0228
#define AES_MULTIPURPOSE2_3_OFFSET		0x022C
#define AES_BYTE_ORDER_CFG_OFFSET		0x02C0
#define AES_TLEN_OFFSET				0x0300
#define AES_T_MAC_0_OFFSET			0x0304
#define AES_T_MAC_1_OFFSET			0x0308
#define AES_T_MAC_2_OFFSET			0x030C
#define AES_T_MAC_3_OFFSET			0x0310
#define AES_PLEN_OFFSET				0x0314
#define AES_A_DMA_SRC_ADDR_OFFSET		0x0400
#define AES_A_DMA_DST_ADDR_OFFSET		0x0404
#define AES_A_DMA_SRC_SIZE_OFFSET		0x0408
#define AES_A_DMA_DST_SIZE_OFFSET		0x040C
#define AES_A_DMA_DMA_MODE_OFFSET		0x0410
#define AES_A_DMA_NEXT_SRC_DESCR_OFFSET		0x0418
#define AES_A_DMA_NEXT_DST_DESCR_OFFSET		0x041C
#define AES_A_DMA_WHILE_ACTIVE_MODE_OFFSET	0x0420
#define AES_A_DMA_LOG_OFFSET			0x0424
#define AES_A_DMA_STATUS_OFFSET			0x0428
#define AES_A_DMA_PERF_CNTR_OFFSET		0x042C
#define AES_A_DMA_MSI_ISR_OFFSET		0x0480
#define AES_A_DMA_MSI_IER_OFFSET		0x0484
#define AES_A_DMA_MSI_MASK_OFFSET		0x0488
#define AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET	0x0600
#define AES_A_DMA_OUTBUFFER_READ_FIFO_OFFSET	0x0700

#define AES_DISABLE_INT			0x00000000
#define AES_DMA_CPD_ERR_INT		BIT(8)
#define AES_DMA_OUTBUF_RD_ERR_INT	BIT(7)
#define AES_DMA_OUTBUF_WR_ERR_INT	BIT(6)
#define AES_DMA_INBUF_RD_ERR_INT	BIT(5)
#define AES_DMA_INBUF_WR_ERR_INT	BIT(4)
#define AES_DMA_BAD_COMP_INT		BIT(3)
#define AES_DMA_SAI_INT			BIT(2)
#define AES_DMA_SRC_DONE_INT		BIT(0)
#define AES_COMPLETE_INT		BIT(1)

#define AES_DMA_MSI_MASK_CLEAR	BIT(0)

#define AES_128_BIT_KEY		0x00000000
#define AES_256_BIT_KEY		BIT(0)

#define AES_DEACTIVATE_PERF_CNTR	0x00000000
#define AES_ACTIVATE_PERF_CNTR		BIT(0)

#define AES_MAX_TAG_SIZE_U32	(4)

/*
 * There is an inconsistency in the documentation. This is
 * documented as a 11-bit value, but it is actually 10-bits.
 */
#define AES_DMA_STATUS_INPUT_BUFFER_OCCUPANCY_MASK (0x3FF)

/* During CCM decrypt, the OCS block needs to finish
 * processing the ciphertext before the tag is
 * written. For 128-bit mode this required delay
 * is 28 OCS clock cycles. For 256-bit mode it is
 * 36 OCS clock cycles.
 */
#define CCM_DECRYPT_DELAY_TAG_CLK_COUNT	(36UL)

/* During CCM decrypt there must be a delay of at least 42 OCS clock cycles
 * between setting the TRIGGER bit in AES_ACTIVE and setting the
 * LAST_CCM_GCM bit in the same register (as stated in the OCS databook)
 */
#define CCM_DECRYPT_DELAY_LAST_GCX_CLK_COUNT (42UL)

#define OCS_DWORD_SWAP(value) \
	(((value & 0x000000ff) << 24) | ((value & 0x0000ff00) << 8) | \
	 ((value & 0xff000000) >> 24) | ((value & 0x00ff0000) >> 8))

/* for all macros below aes_dev is of type 'struct ocs_aes_dev *' */

/* Set endianness of inputs and outputs
 * AES_BYTE_ORDER_CFG
 * default 0x00000000
 * bit [10] - KEY_HI_LO_SWAP
 * bit [9] - KEY_HI_SWAP_DWORDS_IN_OCTWORD
 * bit [8] - KEY_HI_SWAP_BYTES_IN_DWORD
 * bit [7] - KEY_LO_SWAP_DWORDS_IN_OCTWORD
 * bit [6] - KEY_LO_SWAP_BYTES_IN_DWORD
 * bit [5] - IV_SWAP_DWORDS_IN_OCTWORD
 * bit [4] - IV_SWAP_BYTES_IN_DWORD
 * bit [3] - DOUT_SWAP_DWORDS_IN_OCTWORD
 * bit [2] - DOUT_SWAP_BYTES_IN_DWORD
 * bit [1] - DOUT_SWAP_DWORDS_IN_OCTWORD
 * bit [0] - DOUT_SWAP_BYTES_IN_DWORD
 */
#define AES_A_SET_ENDIANNESS(aes_dev) \
	iowrite32(BIT(10) | BIT(9) | BIT(8) | BIT(7) | BIT(6) | \
			BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0), \
			aes_dev->base_reg + AES_BYTE_ORDER_CFG_OFFSET)

/* Write TRIGGER bit
 * AES_ACTIVE
 * default 0x00000000
 * bit [0] - Trigger AES process start
 */
#define AES_A_OP_TRIGGER(aes_dev) \
	iowrite32(BIT(0), aes_dev->base_reg + AES_ACTIVE_OFFSET)

/* Write TERMINATION bit
 * AES_ACTIVE
 * default 0x00000000
 * bit [1] - Indicate last bulk of data
 */
#define AES_A_OP_TERMINATION(aes_dev) \
	iowrite32(BIT(1), aes_dev->base_reg + AES_ACTIVE_OFFSET)

/* Write LAST_CCM_GCM bit
 * AES_ACTIVE
 * default 0x00000000
 * bit [8] - LAST_CCM_GCM
 */
#define AES_A_SET_LAST_GCX(aes_dev) \
	iowrite32(BIT(8), aes_dev->base_reg + AES_ACTIVE_OFFSET)

#define AES_A_WAIT_LAST_GCX(aes_dev) \
	{ \
		u32 aes_active_reg; \
		do { \
			aes_active_reg = \
				ioread32(aes_dev->base_reg + \
						AES_ACTIVE_OFFSET); \
		} while (aes_active_reg & BIT(8)); \
	}


/* Wait for 10 bits of input occupancy. */
#define AES_A_DMA_WAIT_INPUT_BUFFER_OCCUPANCY(aes_dev) \
	{ \
		u32 dma_status_reg; \
		do { \
			dma_status_reg = \
				ioread32(aes_dev->base_reg + \
						AES_A_DMA_STATUS_OFFSET); \
		} while (dma_status_reg & \
				AES_DMA_STATUS_INPUT_BUFFER_OCCUPANCY_MASK); \
	}

/* Write LAST_CCM_GCM bit and LAST_ADATA bit
 * AES_ACTIVE
 * default 0x00000000
 * bit [9] - LAST_ADATA
 * bit [8] - LAST_CCM_GCM
 */
#define AES_A_SET_LAST_GCX_AND_ADATA(aes_dev) \
	iowrite32(BIT(9) | BIT(8), aes_dev->base_reg + AES_ACTIVE_OFFSET)

/* Set DMA src and dst transfer size to 0 */
#define AES_A_DMA_SET_XFER_SIZE_ZERO(aes_dev) \
	{\
		iowrite32(0, aes_dev->base_reg + AES_A_DMA_SRC_SIZE_OFFSET); \
		iowrite32(0, aes_dev->base_reg + AES_A_DMA_DST_SIZE_OFFSET); \
	}

/* Activate DMA
 * DMA_DMA_MODE
 * default 0x00000000
 * bit [31] - ACTIVE
 * bit [25] - SRC_LINK_LIST_EN
 * bit [24] - DST_LINK_LIST_EN
 */
#define AES_A_DMA_ACTIVATE(aes_dev) \
	iowrite32(BIT(31), aes_dev->base_reg + AES_A_DMA_DMA_MODE_OFFSET)

/* Activate DMA and enable src linked list */
#define AES_A_DMA_ACTIVATE_SRC_LL_EN(aes_dev) \
	iowrite32(BIT(31) | BIT(25), aes_dev->base_reg + \
			AES_A_DMA_DMA_MODE_OFFSET)

/* Activate DMA and enable dst linked list */
#define AES_A_DMA_ACTIVATE_DST_LL_EN(aes_dev) \
	iowrite32(BIT(31) | BIT(24), aes_dev->base_reg + \
			AES_A_DMA_DMA_MODE_OFFSET)

/* Activate DMA and enable src and dst linked lists */
#define AES_A_DMA_ACTIVATE_SRC_DST_LL_EN(aes_dev) \
	iowrite32(BIT(31) | BIT(25) | BIT(24), \
			aes_dev->base_reg + AES_A_DMA_DMA_MODE_OFFSET)

/* reset PERF_CNTR to 0 and activate it */
#define AES_A_DMA_RESET_AND_ACTIVATE_PERF_CNTR(aes_dev) \
	{\
		iowrite32(0x00000000, aes_dev->base_reg + \
				AES_A_DMA_PERF_CNTR_OFFSET); \
		iowrite32(AES_ACTIVATE_PERF_CNTR, aes_dev->base_reg + \
				AES_A_DMA_WHILE_ACTIVE_MODE_OFFSET); \
	}

/* wait until PERF_CNTR is > delay, then deactivate it */
#define AES_A_DMA_WAIT_AND_DEACTIVATE_PERF_CNTR(aes_dev, delay) \
	{\
		while (ioread32(aes_dev->base_reg + \
					AES_A_DMA_PERF_CNTR_OFFSET) < delay) \
			; \
		iowrite32(AES_DEACTIVATE_PERF_CNTR, aes_dev->base_reg + \
				AES_A_DMA_WHILE_ACTIVE_MODE_OFFSET); \
	}

/* Defines and macros for AES state variable */
#define AES_IDLE_VAL (0)

/* Bits 0 & 1 used by AES mode ops that require a ISR managed state machine. */
#define AES_STATE_MODE_FSM_BP (0)
#define AES_STATE_MODE_FSM_MASK (0x3 << AES_STATE_MODE_FSM_BP)
#define AES_STATE_CLR_MODE_FSM(state) ((state) &= ~(AES_STATE_MODE_FSM_MASK))
#define AES_STATE_MODE_FSM_1_MASK (0x1 << AES_STATE_MODE_FSM_BP)
#define AES_STATE_MODE_FSM_2_MASK (0x2 << AES_STATE_MODE_FSM_BP)
#define AES_STATE_MODE_FSM_3_MASK (0x3 << AES_STATE_MODE_FSM_BP)

#define AES_STATE_WAIT_AAD_DMA_SRC_DONE \
	AES_STATE_MODE_FSM_1_MASK

#define AES_STATE_WAIT_IN_DMA_SRC_DONE \
	AES_STATE_MODE_FSM_2_MASK

#define AES_STATE_WAIT_OP_DONE AES_STATE_MODE_FSM_3_MASK

#define AES_STATE_IS_WAIT_AAD_DMA_SRC_DONE(state) \
	(((state)&AES_STATE_MODE_FSM_MASK) == \
	 AES_STATE_WAIT_AAD_DMA_SRC_DONE)

#define AES_STATE_IS_WAIT_IN_DMA_SRC_DONE(state) \
	(((state)&AES_STATE_MODE_FSM_MASK) == \
	 AES_STATE_WAIT_IN_DMA_SRC_DONE)

#define AES_STATE_IS_WAIT_OP_DONE(state) \
	(((state)&AES_STATE_MODE_FSM_MASK) == \
	 AES_STATE_WAIT_OP_DONE)

#define AES_STATE_ACTIVE_OP (BIT(7)) /* Bit7 set if AES op active */

#define AES_STATE_IS_OP_ACTIVE(state) \
	((((state)&AES_STATE_ACTIVE_OP)) != 0)


static inline void ocs_aes_xcm_payload_stage(struct ocs_aes_dev *aes_dev,
		const enum aes_mode mode,
		const enum aes_instruction instruction,
		const dma_addr_t src_descriptor, const u32 src_size,
		const dma_addr_t dst_descriptor);
static inline void ocs_aes_gcm_read_tag(struct ocs_aes_dev *aes_dev,
		u8 *const tag, const u32 tag_size);
static inline void ocs_aes_ccm_decrypt_write_tag(struct ocs_aes_dev *aes_dev,
		const u8 *in_tag, const u32 tag_size);
static inline int ccm_compare_tag_to_yr(struct ocs_aes_dev *aes_dev,
		const u8 tag_size_bytes);

enum aes_counter_mode {
	AES_CTR_M_NO_INC = 0,
	AES_CTR_M_32_INC = 1,
	AES_CTR_M_64_INC = 2,
	AES_CTR_M_128_INC = 3,
};

struct B0_formatting {
	union {
		u8 flag;
		struct {
			u8 q_coding : 3;
			u8 t_coding : 3;
			u8 Adata : 1;
			u8 Reserved : 1;
		} flags_st;
	} flag_u;
	u8 N_and_Q[15];
};

struct ocs_aes_t {
	u8 state;
	const enum aes_mode mode;
	const enum aes_instruction instruction;
	u8 *const out_iv;
	const dma_addr_t src_descriptor;
	const u32 src_size;
	const dma_addr_t dst_descriptor;
	const u32 tag_size;
	u8 *const tag;
	u8 *const ccm_auth_res;
};

static struct ocs_aes_t ocs_aes = {
	AES_IDLE_VAL, AES_MODE_ECB, AES_ENCRYPT, NULL, 0, 0, 0, 0, NULL, NULL};

static inline void ocs_aes_context_reset(void)
{
	memzero_explicit(&ocs_aes, sizeof(ocs_aes));
}

/* Disable AES and DMA IRQ. */
static void aes_irq_disable(struct ocs_aes_dev *aes_dev)
{
	u32 isr_val = 0;

	/* disable interrupt */
	iowrite32(AES_DISABLE_INT, aes_dev->base_reg +
			AES_A_DMA_MSI_IER_OFFSET);
	iowrite32(AES_DISABLE_INT, aes_dev->base_reg + AES_IER_OFFSET);

	/* Clear any pending interrupts */
	isr_val = ioread32(aes_dev->base_reg + AES_A_DMA_MSI_ISR_OFFSET);
	if (isr_val)
		iowrite32(isr_val, aes_dev->base_reg +
				AES_A_DMA_MSI_ISR_OFFSET);

	isr_val = ioread32(aes_dev->base_reg + AES_A_DMA_MSI_MASK_OFFSET);
	if (isr_val)
		iowrite32(isr_val, aes_dev->base_reg +
				AES_A_DMA_MSI_MASK_OFFSET);

	isr_val = ioread32(aes_dev->base_reg + AES_ISR_OFFSET);
	if (isr_val)
		iowrite32(isr_val, aes_dev->base_reg + AES_ISR_OFFSET);
}

/* Enable AES or DMA IRQ.
 * IRQ is disabled once fired
 */
static void aes_irq_enable(struct ocs_aes_dev *aes_dev, u8 irq)
{
	if (irq == AES_COMPLETE_INT) {
		/* ensure DMA error interrupts are enabled */
		iowrite32(AES_DMA_CPD_ERR_INT |
				AES_DMA_OUTBUF_RD_ERR_INT |
				AES_DMA_OUTBUF_WR_ERR_INT |
				AES_DMA_INBUF_RD_ERR_INT |
				AES_DMA_INBUF_WR_ERR_INT |
				AES_DMA_BAD_COMP_INT |
				AES_DMA_SAI_INT,
				aes_dev->base_reg + AES_A_DMA_MSI_IER_OFFSET);

		/* AES_IER
		 * default 0x00000000
		 * bits [31:3] - reserved
		 * bit [2] - EN_SKS_ERR
		 * bit [1] - EN_AES_COMPLETE
		 * bit [0] - reserved
		 */
		iowrite32(AES_COMPLETE_INT, aes_dev->base_reg + AES_IER_OFFSET);
	} else if (irq == AES_DMA_SRC_DONE_INT) {
		/* ensure AES interrupts are disabled */
		iowrite32(AES_DISABLE_INT, aes_dev->base_reg + AES_IER_OFFSET);

		/* DMA_MSI_IER
		 * default 0x00000000
		 * bits [31:9] - reserved
		 * bit [8] - CPD_ERR_INT_EN
		 * bit [7] - OUTBUF_RD_ERR_INT_EN
		 * bit [6] - OUTBUF_WR_ERR_INT_EN
		 * bit [5] - INBUF_RD_ERR_INT_EN
		 * bit [4] - INBUF_WR_ERR_INT_EN
		 * bit [3] - BAD_COMP_INT_EN
		 * bit [2] - SAI_INT_EN
		 * bit [1] - DST_DONE_INT_EN
		 * bit [0] - SRC_DONE_INT_EN
		 */
		iowrite32(AES_DMA_CPD_ERR_INT |
				AES_DMA_OUTBUF_RD_ERR_INT |
				AES_DMA_OUTBUF_WR_ERR_INT |
				AES_DMA_INBUF_RD_ERR_INT |
				AES_DMA_INBUF_WR_ERR_INT |
				AES_DMA_BAD_COMP_INT |
				AES_DMA_SAI_INT |
				AES_DMA_SRC_DONE_INT,
				aes_dev->base_reg + AES_A_DMA_MSI_IER_OFFSET);
	}
}

irqreturn_t ocs_aes_irq_handler(int irq, void *dev_id)
{
	struct ocs_aes_dev *aes_dev = dev_id;
	irqreturn_t ret = IRQ_HANDLED;
	u32 aes_dma_isr = ioread32(aes_dev->base_reg +
			AES_A_DMA_MSI_ISR_OFFSET);

	aes_irq_disable(aes_dev);

	aes_dev->dma_err_mask = aes_dma_isr &
		(AES_DMA_CPD_ERR_INT | AES_DMA_OUTBUF_RD_ERR_INT |
		 AES_DMA_OUTBUF_WR_ERR_INT | AES_DMA_INBUF_RD_ERR_INT |
		 AES_DMA_INBUF_WR_ERR_INT | AES_DMA_BAD_COMP_INT |
		 AES_DMA_SAI_INT);
	if (aes_dev->dma_err_mask) {
		/* wake up thread to report error */
		ret = IRQ_WAKE_THREAD;
	} else if (AES_STATE_IS_OP_ACTIVE(ocs_aes.state)) {
		if (ocs_aes.mode == AES_MODE_CCM ||
				ocs_aes.mode == AES_MODE_GCM) {

			if (AES_STATE_IS_WAIT_AAD_DMA_SRC_DONE(ocs_aes.state)) {
				/* AAD data written.
				 * Proceed with next operation
				 */
				ocs_aes_xcm_payload_stage(aes_dev, ocs_aes.mode,
						ocs_aes.instruction,
						ocs_aes.src_descriptor,
						ocs_aes.src_size,
						ocs_aes.dst_descriptor);

				if ((ocs_aes.mode == AES_MODE_CCM) &&
						(ocs_aes.instruction ==
						 AES_DECRYPT)) {
					if (ocs_aes.src_size) {
						 /* CT has been written,
						  * so wait for dma_src_done
						  * interrupt. Go to a state
						  * that is only valid for
						  * this particular CCM decrypt
						  * scenario
						  */
						ocs_aes.state =
							AES_STATE_WAIT_IN_DMA_SRC_DONE
							| AES_STATE_ACTIVE_OP;
						aes_irq_enable(aes_dev,
								AES_DMA_SRC_DONE_INT);
					} else {
						/* No CT, so no
						 * dma_src_done int
						 * generated.
						 * So proceed to writing
						 * tag and go to state
						 * waiting for aes_complete
						 * interrupt
						 */
						ocs_aes.state =
							AES_STATE_WAIT_OP_DONE
							| AES_STATE_ACTIVE_OP;

						ocs_aes_ccm_decrypt_write_tag(
								aes_dev,
								ocs_aes.tag,
								ocs_aes.tag_size);

						aes_irq_enable(aes_dev,
								AES_COMPLETE_INT);
					}
				} else {
					ocs_aes.state = AES_STATE_WAIT_OP_DONE |
						AES_STATE_ACTIVE_OP;
					aes_irq_enable(aes_dev,
							AES_COMPLETE_INT);
				}

				return IRQ_HANDLED;
			}

			if (AES_STATE_IS_WAIT_IN_DMA_SRC_DONE(ocs_aes.state)) {
				/* only get here for CCM decrypt */

				ocs_aes.state = AES_STATE_WAIT_OP_DONE |
					AES_STATE_ACTIVE_OP;
				/* Timeout to be handled at a higher level
				 * as even if timeout fired there's no apparent
				 * way to reset/recover
				 */

				ocs_aes_ccm_decrypt_write_tag(aes_dev,
						ocs_aes.tag, ocs_aes.tag_size);

				aes_irq_enable(aes_dev, AES_COMPLETE_INT);

				return IRQ_HANDLED;
			}

			if (AES_STATE_IS_WAIT_OP_DONE(ocs_aes.state)) {
				if (ocs_aes.mode == AES_MODE_GCM)
					ocs_aes_gcm_read_tag(aes_dev,
							ocs_aes.tag,
							ocs_aes.tag_size);

				if ((ocs_aes.mode == AES_MODE_CCM) &&
						(ocs_aes.instruction ==
						AES_DECRYPT))
					*ocs_aes.ccm_auth_res =
						ccm_compare_tag_to_yr(aes_dev,
								ocs_aes.tag_size
								);
			}
		} else if (ocs_aes.mode == AES_MODE_CTR && ocs_aes.out_iv) {
			/* Read back IV for streaming mode */
			((u32 *)ocs_aes.out_iv)[0] =
				ioread32(aes_dev->base_reg + AES_IV_0_OFFSET);
			((u32 *)ocs_aes.out_iv)[1] =
				ioread32(aes_dev->base_reg + AES_IV_1_OFFSET);
			((u32 *)ocs_aes.out_iv)[2] =
				ioread32(aes_dev->base_reg + AES_IV_2_OFFSET);
			((u32 *)ocs_aes.out_iv)[3] =
				ioread32(aes_dev->base_reg + AES_IV_3_OFFSET);
		}

		/* wake up thread to report result */
		ret = IRQ_WAKE_THREAD;
	} else {
		/* unexpected interrupt
		 * so reset context (below)
		 */
		dev_err(aes_dev->dev, "Received unexpected interrupt\n");
	}

	/* all done, so reset context */
	ocs_aes_context_reset();

	return ret;
}

int ocs_aes_set_key(struct ocs_aes_dev *aes_dev, const u32 key_size,
		const u8 *key, const enum ocs_cipher cipher)
{
	/* OCS AES supports 128-bit and 256-bit keys only
	 * SM4 supports 128-bit keys
	 */
	if ((cipher == OCS_AES) && !(key_size == 32 || key_size == 16)) {
		dev_err(aes_dev->dev, "ERROR: Key size %d-bit not supported for AES cipher\n",
				key_size * 8);
		return OCS_EINVAL;
	}

	if ((cipher == OCS_SM4) && !(key_size == 16)) {
		dev_err(aes_dev->dev, "ERROR: Key size %d-bit not supported for SM4 cipher\n",
				key_size * 8);
		return OCS_EINVAL;
	}

	if (key == NULL)
		return OCS_EINVAL;

	iowrite32(((u32 *)key)[0], aes_dev->base_reg + AES_KEY_0_OFFSET);
	iowrite32(((u32 *)key)[1], aes_dev->base_reg + AES_KEY_1_OFFSET);
	iowrite32(((u32 *)key)[2], aes_dev->base_reg + AES_KEY_2_OFFSET);
	iowrite32(((u32 *)key)[3], aes_dev->base_reg + AES_KEY_3_OFFSET);
	if (key_size == 32) {
		iowrite32(((u32 *)key)[4], aes_dev->base_reg +
				AES_KEY_4_OFFSET);
		iowrite32(((u32 *)key)[5], aes_dev->base_reg +
				AES_KEY_5_OFFSET);
		iowrite32(((u32 *)key)[6], aes_dev->base_reg +
				AES_KEY_6_OFFSET);
		iowrite32(((u32 *)key)[7], aes_dev->base_reg +
				AES_KEY_7_OFFSET);

		/* Write key size
		 * bits [31:1] - reserved
		 * bit [0] - AES_KEY_SIZE
		 *           0 - 128 bit key
		 *           1 - 256 bit key
		 */
		iowrite32(AES_256_BIT_KEY, aes_dev->base_reg +
				AES_KEY_SIZE_OFFSET);
	} else {
		iowrite32(AES_128_BIT_KEY, aes_dev->base_reg +
				AES_KEY_SIZE_OFFSET);
	}

	return OCS_OK;
}

/* Write AES_COMMAND */
static inline void set_ocs_aes_command(struct ocs_aes_dev *aes_dev,
		const enum ocs_cipher cipher, const enum aes_mode mode,
		const enum aes_instruction instruction)
{
	u32 val;

	/* AES_COMMAND
	 * default 0x000000CC
	 * bit [14] - CIPHER_SELECT
	 *            0 - AES
	 *            1 - SM4
	 * bits [11:8] - AES_MODE
	 *               0000 - ECB
	 *               0001 - CBC
	 *               0010 - CTR
	 *               0110 - CCM
	 *               0111 - GCM
	 *               1001 - CTS
	 * bits [7:6] - AES_INSTRUCTION
	 *              00 - ENCRYPT
	 *              01 - DECRYPT
	 *              10 - EXPAND
	 *              11 - BYPASS
	 * bits [3:2] - CTR_M_BITS
	 *              00 - No increment
	 *              01 - Least significant 32 bits are incremented
	 *              10 - Least significant 64 bits are incremented
	 *              11 - Full 128 bits are incremented
	 */
	val = (cipher<<14) + (mode<<8) + (instruction<<6) +
		(AES_CTR_M_128_INC<<2);
	iowrite32(val, aes_dev->base_reg + AES_COMMAND_OFFSET);
}

/* Configure DMA to OCS, linked list mode */
static inline void dma_to_ocs_aes_ll(
		struct ocs_aes_dev *aes_dev, const dma_addr_t descriptor)
{
	iowrite32(0, aes_dev->base_reg + AES_A_DMA_SRC_SIZE_OFFSET);
	iowrite32(descriptor, aes_dev->base_reg +
			AES_A_DMA_NEXT_SRC_DESCR_OFFSET);
}

/* Configure DMA from OCS, linked list mode */
static inline void dma_from_ocs_aes_ll(
		struct ocs_aes_dev *aes_dev, const dma_addr_t descriptor)
{
	iowrite32(0, aes_dev->base_reg + AES_A_DMA_DST_SIZE_OFFSET);
	iowrite32(descriptor, aes_dev->base_reg +
			AES_A_DMA_NEXT_DST_DESCR_OFFSET);
}

/* Write length of last data block */
static inline void write_ocs_aes_last_full_data_blk(
		struct ocs_aes_dev *aes_dev, const u32 size)
{
	u32 val;

	if (size == 0)
		val = 0;
	else {
		val = size % AES_BLOCK_SIZE;
		if (val == 0)
			val = AES_BLOCK_SIZE;
	}

	iowrite32(val, aes_dev->base_reg + AES_PLEN_OFFSET);
}

/* Validate inputs according to mode
 * If OK return OCS_OK
 * else return OCS_EINVAL
 */
static int ocs_aes_validate_inputs(
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv, const u32 iv_size,
		const dma_addr_t aad_descriptor, const u32 aad_size,
		const u8 *tag, const u32 tag_size,
		const enum ocs_cipher cipher,
		const enum aes_mode mode,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor, u8 *ccm_auth_res)
{
	/* ensure cipher, mode and instruction are valid */
	if (!(cipher == OCS_AES || cipher == OCS_SM4))
		return OCS_EINVAL;

	if (!(mode == AES_MODE_ECB || mode == AES_MODE_CBC ||
				mode == AES_MODE_CTR || mode == AES_MODE_CCM ||
				mode == AES_MODE_GCM || mode == AES_MODE_CTS))
		return OCS_EINVAL;

	if (!(instruction == AES_ENCRYPT ||
				instruction == AES_DECRYPT ||
				instruction == AES_EXPAND ||
				instruction == AES_BYPASS))
		return OCS_EINVAL;

	/* for performance reasons switch based on mode to limit
	 * unnecessary conditionals for each mode
	 */
	switch (mode) {
	case AES_MODE_ECB:
		/* ensure input length is multiple of block size */
		if ((src_size % AES_BLOCK_SIZE) != 0)
			return OCS_EINVAL;

		/* ensure source and destination linked lists are created */
		if (src_descriptor == 0 || dst_descriptor == 0)
			return OCS_EINVAL;

		break;

	case AES_MODE_CBC:
		/* ensure input length is multiple of block size */
		if ((src_size % AES_BLOCK_SIZE) != 0)
			return OCS_EINVAL;

		/* ensure source and destination linked lists are created */
		if (src_descriptor == 0 || dst_descriptor == 0)
			return OCS_EINVAL;

		/* ensure IV is present and block size in length */
		if (!iv || iv_size != AES_BLOCK_SIZE)
			return OCS_EINVAL;

		break;

	case AES_MODE_CTR:
		/* ensure input length of 1 byte or greater */
		if (src_size == 0)
			return OCS_EINVAL;

		/* ensure source and destination linked lists are created */
		if (src_descriptor == 0 || dst_descriptor == 0)
			return OCS_EINVAL;

		/* ensure IV is present and block size in length */
		if (!iv || iv_size != AES_BLOCK_SIZE)
			return OCS_EINVAL;

		break;

	case AES_MODE_CTS:
		/* ensure input length >= block size */
		if (src_size < AES_BLOCK_SIZE)
			return OCS_EINVAL;

		/* ensure source and destination linked lists are created */
		if (src_descriptor == 0 || dst_descriptor == 0)
			return OCS_EINVAL;

		/* ensure IV is present and block size in length */
		if (!iv || iv_size != AES_BLOCK_SIZE)
			return OCS_EINVAL;

		break;

	case AES_MODE_GCM:
		/* ensure IV is present and GCM_AES_IV_SIZE in length */
		if (!iv || iv_size != GCM_AES_IV_SIZE)
			return OCS_EINVAL;

		/* if input data present ensure source and destination linked
		 * lists are created
		 */
		if (src_size) {
			if (src_descriptor == 0 || dst_descriptor == 0)
				return OCS_EINVAL;
		}

		/* if aad present ensure aad linked list is created */
		if (aad_size) {
			if (aad_descriptor == 0)
				return OCS_EINVAL;
		}

		/* ensure tag destination is set */
		if (!tag)
			return OCS_EINVAL;

		/* ensure tag size is valid
		 * valid tag sizes for GCM are 16, 15, 14, 13, 12, 8 or 4 bytes
		 */
		if (!(tag_size == 16 || tag_size == 15 || tag_size == 14 ||
				tag_size == 13 || tag_size == 12 ||
				tag_size == 8 || tag_size == 4))
			return OCS_EINVAL;

		break;

	case AES_MODE_CCM:
		/* ensure IV is present and block size in length */
		if (!iv || iv_size != AES_BLOCK_SIZE)
			return OCS_EINVAL;

		/* if aad present ensure aad linked list is created */
		if (aad_size) {
			if (aad_descriptor == 0)
				return OCS_EINVAL;
		}

		/* ensure tag size >= 4 bytes and <= 16 bytes */
		if (tag_size < 4 || tag_size > AES_BLOCK_SIZE)
			return OCS_EINVAL;

		if (instruction == AES_DECRYPT) {
			/* if input data present ensure source and
			 * destination linked lists are created
			 */
			if (src_size) {
				if (src_descriptor == 0 || dst_descriptor == 0)
					return OCS_EINVAL;
			}

			/* ensure there is a place to put the result
			 * of the tag (authentication) check
			 */
			if (!ccm_auth_res)
				return OCS_EINVAL;

			/* ensure input tag is present */
			if (!tag)
				return OCS_EINVAL;
		} else {
			/* AES_ENCRYPT */

			/* destination linked list always required
			 * (for tag even if no input data)
			 */
			if (dst_descriptor == 0)
				return OCS_EINVAL;

			/* if input data present ensure src
			 * linked list is created
			 */
			if (src_size) {
				if (src_descriptor == 0)
					return OCS_EINVAL;
			}
		}

		break;

	default:
		return OCS_EINVAL;
	}

	return OCS_OK;
}

int ocs_aes_op(struct ocs_aes_dev *aes_dev,
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv, const u32 iv_size,
		const enum ocs_cipher cipher,
		const enum aes_mode mode,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor)
{
	int ret;

	ret = ocs_aes_validate_inputs(src_descriptor, src_size, iv,
			iv_size, 0, 0, NULL, 0, cipher, mode, instruction,
			dst_descriptor, NULL);
	if (ret != OCS_OK)
		return ret;

	*((enum aes_mode *)&ocs_aes.mode) = mode;
	*((void **)&ocs_aes.out_iv) = (void *)iv;
	ocs_aes.state = AES_STATE_ACTIVE_OP;

	/* Ensure interrupts are disabled and
	 * any pending interrupts are cleared
	 */
	aes_irq_disable(aes_dev);

	AES_A_SET_ENDIANNESS(aes_dev);
	set_ocs_aes_command(aes_dev, cipher, mode, instruction);

	if (mode == AES_MODE_CTS)
		write_ocs_aes_last_full_data_blk(aes_dev, src_size);

	if ((mode != AES_MODE_ECB) && iv && (iv_size == AES_BLOCK_SIZE)) {
		iowrite32(((u32 *)iv)[0], aes_dev->base_reg + AES_IV_0_OFFSET);
		iowrite32(((u32 *)iv)[1], aes_dev->base_reg + AES_IV_1_OFFSET);
		iowrite32(((u32 *)iv)[2], aes_dev->base_reg + AES_IV_2_OFFSET);
		iowrite32(((u32 *)iv)[3], aes_dev->base_reg + AES_IV_3_OFFSET);
	}

	AES_A_OP_TRIGGER(aes_dev);

	dma_to_ocs_aes_ll(aes_dev, src_descriptor);
	dma_from_ocs_aes_ll(aes_dev, dst_descriptor);
	AES_A_DMA_ACTIVATE_SRC_DST_LL_EN(aes_dev);

	if (mode == AES_MODE_CTS)
		AES_A_SET_LAST_GCX(aes_dev);
	else
		AES_A_OP_TERMINATION(aes_dev);

	aes_irq_enable(aes_dev, AES_COMPLETE_INT);

	return OCS_OK;
}

/* Write data portion to OCS
 * i.e. plaintext for encrypt operation
 * ciphertext for decrypt operation
 */
static inline void ocs_aes_xcm_payload_stage(struct ocs_aes_dev *aes_dev,
		const enum aes_mode mode,
		const enum aes_instruction instruction,
		const dma_addr_t src_descriptor, const u32 src_size,
		const dma_addr_t dst_descriptor)
{
	AES_A_WAIT_LAST_GCX(aes_dev);
	AES_A_DMA_WAIT_INPUT_BUFFER_OCCUPANCY(aes_dev);

	if (src_size) {
		/* Fetch plain text */
		dma_to_ocs_aes_ll(aes_dev, src_descriptor);
		/* Set output address */
		dma_from_ocs_aes_ll(aes_dev, dst_descriptor);
		AES_A_DMA_ACTIVATE_SRC_DST_LL_EN(aes_dev);
	} else {
		if ((mode == AES_MODE_CCM) && (instruction == AES_ENCRYPT)) {
			/* Set output address */
			dma_from_ocs_aes_ll(aes_dev, dst_descriptor);
			AES_A_DMA_ACTIVATE_DST_LL_EN(aes_dev);
		} else {
			AES_A_DMA_SET_XFER_SIZE_ZERO(aes_dev);
			AES_A_DMA_ACTIVATE(aes_dev);
		}
	}

	AES_A_SET_LAST_GCX(aes_dev);
}

static void gcm_construct_j0(const u8 *iv, u32 *j0)
{
	/* IV must be 12 bytes
	 * Other sizes not supported as Linux crypto
	 * API does only expects/allows 12 byte IV
	 * for GCM
	 */
	memcpy(j0, iv, GCM_AES_IV_SIZE);
	j0[0] = OCS_DWORD_SWAP(j0[0]);
	j0[1] = OCS_DWORD_SWAP(j0[1]);
	j0[2] = OCS_DWORD_SWAP(j0[2]);
	j0[3] = 0x00000001;
}

int ocs_aes_gcm_op(struct ocs_aes_dev *aes_dev,
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv,
		const dma_addr_t aad_descriptor, const u32 aad_size,
		const u32 tag_size, const enum ocs_cipher cipher,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor, u8 *tag)
{
	u32 J0[4], val;
	int ret;

	ret = ocs_aes_validate_inputs(src_descriptor, src_size, iv,
			GCM_AES_IV_SIZE, aad_descriptor, aad_size, tag,
			tag_size, cipher, AES_MODE_GCM, instruction,
			dst_descriptor, NULL);
	if (ret != OCS_OK)
		return ret;

	*((enum aes_mode *)&ocs_aes.mode) = AES_MODE_GCM;
	*((enum aes_instruction *)&ocs_aes.instruction) = instruction;
	*((dma_addr_t *)&ocs_aes.src_descriptor) = src_descriptor;
	*((u32 *)&ocs_aes.src_size) = src_size;
	*((dma_addr_t *)&ocs_aes.dst_descriptor) = dst_descriptor;
	*((u32 *)&ocs_aes.tag_size) = tag_size;
	*((void **)&ocs_aes.tag) = (void *)tag;

	/* Ensure interrupts are disabled and
	 * any pending interrupts are cleared
	 */
	aes_irq_disable(aes_dev);

	AES_A_SET_ENDIANNESS(aes_dev);
	set_ocs_aes_command(aes_dev, cipher, AES_MODE_GCM, instruction);

	gcm_construct_j0(iv, J0);
	iowrite32(J0[3], aes_dev->base_reg + AES_IV_0_OFFSET);
	iowrite32(J0[2], aes_dev->base_reg + AES_IV_1_OFFSET);
	iowrite32(J0[1], aes_dev->base_reg + AES_IV_2_OFFSET);
	iowrite32(J0[0], aes_dev->base_reg + AES_IV_3_OFFSET);

	/* Write tag length */
	iowrite32(tag_size, aes_dev->base_reg + AES_TLEN_OFFSET);

	write_ocs_aes_last_full_data_blk(aes_dev, src_size);

	/* write ciphertext length */
	val = src_size * 8;
	iowrite32(val, aes_dev->base_reg + AES_MULTIPURPOSE2_0_OFFSET);

	val = (u32)((((uint64_t)(src_size)) * 8) >> 32);
	iowrite32(val, aes_dev->base_reg + AES_MULTIPURPOSE2_1_OFFSET);

	/* write aad length */
	val = aad_size * 8;
	iowrite32(val, aes_dev->base_reg + AES_MULTIPURPOSE2_2_OFFSET);

	val = (u32)((((uint64_t)(aad_size)) * 8) >> 32);
	iowrite32(val, aes_dev->base_reg + AES_MULTIPURPOSE2_3_OFFSET);

	AES_A_OP_TRIGGER(aes_dev);

	if (aad_size) {
		ocs_aes.state = AES_STATE_WAIT_AAD_DMA_SRC_DONE |
			AES_STATE_ACTIVE_OP;

		/* Fetch AAD */
		dma_to_ocs_aes_ll(aes_dev, aad_descriptor);
		AES_A_DMA_ACTIVATE_SRC_LL_EN(aes_dev);

		AES_A_SET_LAST_GCX_AND_ADATA(aes_dev);

		aes_irq_enable(aes_dev, AES_DMA_SRC_DONE_INT);
	} else {
		/* Since no aad the LAST_GCX bit can be set now */
		AES_A_SET_LAST_GCX_AND_ADATA(aes_dev);

		ocs_aes.state = AES_STATE_WAIT_OP_DONE | AES_STATE_ACTIVE_OP;

		ocs_aes_xcm_payload_stage(aes_dev, AES_MODE_GCM, instruction,
				src_descriptor, src_size, dst_descriptor);

		aes_irq_enable(aes_dev, AES_COMPLETE_INT);
	}

	return OCS_OK;
}

static inline void ocs_aes_gcm_read_tag(
		struct ocs_aes_dev *aes_dev, u8 *const tag, const u32 tag_size)
{
	u32 tag_u32[AES_MAX_TAG_SIZE_U32];

	tag_u32[0] = ioread32(aes_dev->base_reg + AES_T_MAC_3_OFFSET);
	tag_u32[1] = ioread32(aes_dev->base_reg + AES_T_MAC_2_OFFSET);
	tag_u32[2] = ioread32(aes_dev->base_reg + AES_T_MAC_1_OFFSET);
	tag_u32[3] = ioread32(aes_dev->base_reg + AES_T_MAC_0_OFFSET);

	/* Not swapping during initial assignment as macro would translate
	 * to 4 reg reads each
	 */
	tag_u32[0] = OCS_DWORD_SWAP(tag_u32[0]);
	tag_u32[1] = OCS_DWORD_SWAP(tag_u32[1]);
	tag_u32[2] = OCS_DWORD_SWAP(tag_u32[2]);
	tag_u32[3] = OCS_DWORD_SWAP(tag_u32[3]);

	if (tag)
		memcpy(tag, tag_u32, tag_size);
}

static inline int ccm_compare_tag_to_yr(
		struct ocs_aes_dev *aes_dev, const u8 tag_size_bytes)
{
	u8 i = 0;
	u32 tag_u32[AES_MAX_TAG_SIZE_U32] = {0};
	u32 yr_u32[AES_MAX_TAG_SIZE_U32] = {0};
	const u8 *const tag_u8 = (const u8 *const)tag_u32;
	const u8 *const yr_u8 = (const u8 *const)yr_u32;

	if (tag_size_bytes > (AES_MAX_TAG_SIZE_U32 * 4))
		return OCS_CCM_TAG_INVALID;

	/* read Tag */
	tag_u32[0] = ioread32(aes_dev->base_reg + AES_T_MAC_0_OFFSET);
	tag_u32[1] = ioread32(aes_dev->base_reg + AES_T_MAC_1_OFFSET);
	tag_u32[2] = ioread32(aes_dev->base_reg + AES_T_MAC_2_OFFSET);
	tag_u32[3] = ioread32(aes_dev->base_reg + AES_T_MAC_3_OFFSET);

	/* read Yr */
	yr_u32[0] = ioread32(aes_dev->base_reg + AES_MULTIPURPOSE2_0_OFFSET);
	yr_u32[1] = ioread32(aes_dev->base_reg + AES_MULTIPURPOSE2_1_OFFSET);
	yr_u32[2] = ioread32(aes_dev->base_reg + AES_MULTIPURPOSE2_2_OFFSET);
	yr_u32[3] = ioread32(aes_dev->base_reg + AES_MULTIPURPOSE2_3_OFFSET);

	/* compare Tag and Yr */
	for (i = 0; i < tag_size_bytes; i++) {
		if (tag_u8[i] != yr_u8[i])
			return OCS_CCM_TAG_INVALID;
	}
	return OCS_CCM_TAG_OK;
}

static inline void ocs_aes_ccm_decrypt_write_tag(struct ocs_aes_dev *aes_dev,
		const u8 *in_tag, const u32 tag_size)
{
	int i;

	/* Ensure DMA input buffer is empty */
	AES_A_DMA_WAIT_INPUT_BUFFER_OCCUPANCY(aes_dev);

	/* During CCM decrypt, the OCS block needs to finish
	 * processing the ciphertext before the tag is written.
	 * So delay needed after DMA has completed writing
	 * the ciphertext
	 */
	AES_A_DMA_RESET_AND_ACTIVATE_PERF_CNTR(aes_dev);
	AES_A_DMA_WAIT_AND_DEACTIVATE_PERF_CNTR(aes_dev,
			CCM_DECRYPT_DELAY_TAG_CLK_COUNT);

	/* write encrypted tag */
	for (i = 0; i < tag_size; i++) {
		iowrite8(in_tag[i], aes_dev->base_reg +
				AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	}
}

int ocs_aes_ccm_op(struct ocs_aes_dev *aes_dev,
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv,
		const dma_addr_t aad_descriptor, const u32 aad_size,
		const u8 *in_tag, const u32 tag_size,
		const enum ocs_cipher cipher,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor, u8 *auth_res)
{
	u32 val, i;
	u32 *ctr_ptr = NULL;
	struct B0_formatting B0;
	int ret;

	ret = ocs_aes_validate_inputs(src_descriptor, src_size, iv,
			AES_BLOCK_SIZE, aad_descriptor, aad_size, in_tag,
			tag_size, cipher, AES_MODE_CCM, instruction,
			dst_descriptor, auth_res);
	if (ret != OCS_OK)
		return ret;

	*((enum aes_mode *)&ocs_aes.mode) = AES_MODE_CCM;
	*((enum aes_instruction *)&ocs_aes.instruction) = instruction;
	*((dma_addr_t *)&ocs_aes.src_descriptor) = src_descriptor;
	*((u32 *)&ocs_aes.src_size) = src_size;
	*((dma_addr_t *)&ocs_aes.dst_descriptor) = dst_descriptor;
	*((u32 *)&ocs_aes.tag_size) = tag_size;
	*((void **)&ocs_aes.tag) = (void *)in_tag;
	*((void **)&ocs_aes.ccm_auth_res) = (void *)auth_res;

	/* Ensure interrupts are disabled and
	 * any pending interrupts are cleared
	 */
	aes_irq_disable(aes_dev);

	AES_A_SET_ENDIANNESS(aes_dev);
	set_ocs_aes_command(aes_dev, cipher, AES_MODE_CCM, instruction);

	/* nonce is already converted to ctr0 before
	 * being passed into this function (as iv)
	 */
	ctr_ptr = (u32 *)&iv[0];
	iowrite32(OCS_DWORD_SWAP(*ctr_ptr), aes_dev->base_reg +
			AES_MULTIPURPOSE1_3_OFFSET);
	ctr_ptr = (u32 *)&iv[4];
	iowrite32(OCS_DWORD_SWAP(*ctr_ptr), aes_dev->base_reg +
			AES_MULTIPURPOSE1_2_OFFSET);
	ctr_ptr = (u32 *)&iv[8];
	iowrite32(OCS_DWORD_SWAP(*ctr_ptr), aes_dev->base_reg +
			AES_MULTIPURPOSE1_1_OFFSET);
	ctr_ptr = (u32 *)&iv[12];
	iowrite32(OCS_DWORD_SWAP(*ctr_ptr), aes_dev->base_reg +
			AES_MULTIPURPOSE1_0_OFFSET);

	/* Write tag length */
	iowrite32(tag_size, aes_dev->base_reg + AES_TLEN_OFFSET);

	write_ocs_aes_last_full_data_blk(aes_dev, src_size);

	AES_A_OP_TRIGGER(aes_dev);

	AES_A_DMA_RESET_AND_ACTIVATE_PERF_CNTR(aes_dev);

	/* Write B0 */
	memset(&B0, 0, 16);
	B0.flag_u.flags_st.Adata = (aad_size == 0) ? 0 : 1;
	B0.flag_u.flags_st.t_coding = (u8)(((tag_size-2)/2) & 0x07);
	B0.flag_u.flags_st.q_coding = iv[0];
	for (i = 0; i < (AES_BLOCK_SIZE - 2 - iv[0]); i++)
		B0.N_and_Q[i] = iv[i + 1];
	/* Assigning size value from dword(val) to byte array till val
	 * becomes zero. Start at end of array and work back.
	 */
	val = src_size;
	i = sizeof(B0.N_and_Q) - 1;
	while (val) {
		B0.N_and_Q[i] = val & 0xff;
		val = val >> 8; // Divide by 256
		i--;
	}

	iowrite8(B0.flag_u.flag, aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[0], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[1], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[2], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[3], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[4], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[5], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[6], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[7], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[8], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[9], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[10], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[11], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[12], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[13], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
	iowrite8(B0.N_and_Q[14], aes_dev->base_reg +
			AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);

	if (aad_size) {
		ocs_aes.state = AES_STATE_WAIT_AAD_DMA_SRC_DONE |
			AES_STATE_ACTIVE_OP;

		/* Write AAD size */
		iowrite8((u8)(aad_size/0x100), aes_dev->base_reg +
				AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);
		iowrite8((u8)(aad_size%0x100), aes_dev->base_reg +
				AES_A_DMA_INBUFFER_WRITE_FIFO_OFFSET);

		/* Fetch AAD */
		dma_to_ocs_aes_ll(aes_dev, aad_descriptor);

		/* Ensure there has been at least
		 * CCM_DECRYPT_DELAY_LAST_GCX_CLK_COUNT
		 * clock cycles since TRIGGER bit was set
		 */
		AES_A_DMA_WAIT_AND_DEACTIVATE_PERF_CNTR(aes_dev,
				CCM_DECRYPT_DELAY_LAST_GCX_CLK_COUNT);

		AES_A_DMA_ACTIVATE_SRC_LL_EN(aes_dev);
		AES_A_SET_LAST_GCX_AND_ADATA(aes_dev);

		aes_irq_enable(aes_dev, AES_DMA_SRC_DONE_INT);
	} else {
		/* Ensure there has been at least
		 * CCM_DECRYPT_DELAY_LAST_GCX_CLK_COUNT
		 * clock cycles since TRIGGER bit was set
		 */
		AES_A_DMA_WAIT_AND_DEACTIVATE_PERF_CNTR(aes_dev,
				CCM_DECRYPT_DELAY_LAST_GCX_CLK_COUNT);

		/* Since no aad the LAST_GCX bit can be set now */
		AES_A_SET_LAST_GCX_AND_ADATA(aes_dev);

		ocs_aes_xcm_payload_stage(aes_dev, AES_MODE_CCM, instruction,
				src_descriptor, src_size, dst_descriptor);

		if (instruction == AES_DECRYPT) {
			if (src_size) {
				ocs_aes.state =	AES_STATE_WAIT_IN_DMA_SRC_DONE
					| AES_STATE_ACTIVE_OP;
				aes_irq_enable(aes_dev, AES_DMA_SRC_DONE_INT);
			} else {
				/* no dma_src_done int generated */
				ocs_aes.state = AES_STATE_WAIT_OP_DONE
					| AES_STATE_ACTIVE_OP;

				ocs_aes_ccm_decrypt_write_tag(aes_dev,
						in_tag, tag_size);

				aes_irq_enable(aes_dev, AES_COMPLETE_INT);
			}
		} else {
			ocs_aes.state = AES_STATE_WAIT_OP_DONE |
				AES_STATE_ACTIVE_OP;
			aes_irq_enable(aes_dev, AES_COMPLETE_INT);
		}
	}

	return OCS_OK;
}

void ocs_create_linked_list_from_sg(struct ocs_aes_dev *aes_dev,
		struct scatterlist *sgl, u32 num_sgl_entries,
		u8 **aad_buf, dma_addr_t *aad_descriptor,
		u32 aad_size, u32 *aad_desc_size,
		u8 **data_buf, dma_addr_t *data_descriptor,
		u32 data_size, u32 *data_desc_size)
{
	struct ocs_dma_linked_list *ll = NULL;
	u32 data_offset = 0;
	struct scatterlist *sg;
	int num_aad_ents, i;

	if (num_sgl_entries == 0)
		goto ret_err;

	sg = sgl;

	if (aad_size) {
		num_aad_ents = sg_nents_for_len(sgl, aad_size);
		if (num_aad_ents < 0)
			goto ret_err;

		*aad_desc_size = sizeof(struct ocs_dma_linked_list) *
			num_aad_ents;

		/* HW requirement: descriptor must be 8 byte aligned */
		*aad_buf = kmalloc(*aad_desc_size, GFP_KERNEL | GFP_DMA);
		if (!*aad_buf)
			goto ret_err;

		ll = (struct ocs_dma_linked_list *)(*aad_buf);

		*aad_descriptor = dma_map_single(aes_dev->dev, *aad_buf,
				*aad_desc_size, DMA_TO_DEVICE);
		if (dma_mapping_error(aes_dev->dev, *aad_descriptor)) {
			dev_err(aes_dev->dev, "DMA mapping error\n");
			*aad_descriptor = 0;
			goto ret_err;
		}

		dma_sync_single_for_cpu(aes_dev->dev, *aad_descriptor,
				*aad_desc_size, DMA_TO_DEVICE);

		i = 0;
		while (true) {
			ll[i].address = sg_dma_address(sg);
			ll[i].byte_count = (sg_dma_len(sg) < aad_size) ?
				sg_dma_len(sg) : aad_size;
			aad_size -= ll[i].byte_count;
			ll[i].freeze = 0;
			ll[i].next = *aad_descriptor +
				(sizeof(struct ocs_dma_linked_list) * (i+1));
			ll[i].reserved = 0;
			ll[i].terminate = 0;
			i++;
			if (aad_size && (i < num_aad_ents))
				sg = sg_next(sg);
			else
				break;
		}
		ll[i-1].next = 0;
		ll[i-1].terminate = 1;
		data_offset = ll[i-1].byte_count;

		dma_sync_single_for_device(aes_dev->dev, *aad_descriptor,
				*aad_desc_size, DMA_TO_DEVICE);
	} else {
		num_aad_ents = 0;
	}

	if (data_size) {
		/* +1 for case where aad and data overlap in one sgl node */
		num_sgl_entries = num_sgl_entries - num_aad_ents + 1;

		*data_desc_size = sizeof(struct ocs_dma_linked_list) *
			num_sgl_entries;

		/* HW requirement: descriptor must be 8 byte aligned */
		*data_buf = kmalloc(*data_desc_size, GFP_KERNEL | GFP_DMA);
		if (!*data_buf)
			goto ret_err;

		ll = (struct ocs_dma_linked_list *)(*data_buf);

		*data_descriptor = dma_map_single(aes_dev->dev, *data_buf,
				*data_desc_size, DMA_TO_DEVICE);
		if (dma_mapping_error(aes_dev->dev, *data_descriptor)) {
			dev_err(aes_dev->dev, "DMA mapping error\n");
			data_descriptor = 0;
			goto ret_err;
		}

		dma_sync_single_for_cpu(aes_dev->dev, *data_descriptor,
				*data_desc_size, DMA_TO_DEVICE);

		if (data_offset == sg_dma_len(sg)) {
			sg = sg_next(sg);
			data_offset = 0;
		}

		for (i = 0; (i < num_sgl_entries) && data_size;
				i++, sg = sg_next(sg)) {
			ll[i].address = sg_dma_address(sg) + data_offset;
			ll[i].byte_count =
				((sg_dma_len(sg) - data_offset) < data_size) ?
				(sg_dma_len(sg) - data_offset) : data_size;
			data_offset = 0;
			data_size -= ll[i].byte_count;
			ll[i].freeze = 0;
			ll[i].next = *data_descriptor +
				(sizeof(struct ocs_dma_linked_list) * (i+1));
			ll[i].reserved = 0;
			ll[i].terminate = 0;
		}
		ll[i-1].next = 0;
		ll[i-1].terminate = 1;

		dma_sync_single_for_device(aes_dev->dev, *data_descriptor,
				*data_desc_size, DMA_TO_DEVICE);
	}

	return;

ret_err:
		if (data_buf && *data_buf)
			kfree(*data_buf);
		if (data_buf)
			*data_buf = NULL;
		*data_desc_size = 0;

		if (*aad_descriptor)
			dma_unmap_single(aes_dev->dev, *aad_descriptor,
					*aad_desc_size, DMA_TO_DEVICE);
		if (aad_buf && *aad_buf)
			kfree(*aad_buf);
		if (aad_buf)
			*aad_buf = NULL;
		*aad_desc_size = 0;

		return;
}
