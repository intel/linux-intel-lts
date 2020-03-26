/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay OCS AES Crypto Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#ifndef _CRYPTO_OCS_AES_H
#define _CRYPTO_OCS_AES_H

/* System common return values. */
#define OCS_OK (0)
#define OCS_EGENERIC (-EIO)  /* generic error */
#define OCS_EINVAL (-EINVAL) /* Invalid arg error */

/* Return types for CCM tag check */
#define OCS_CCM_TAG_OK (0)
#define OCS_CCM_TAG_INVALID (1)

#define AES_BLOCK_SIZE	(16)
#define GCM_AES_IV_SIZE	(12)

/**
 * AES device context
 * @list: List of device contexts
 * @dev: OCS AES device
 * @irq: IRQ number
 * @base_reg: IO base address of OCS AES
 * @req_is_aead: Active request is AEAD cipher
 * @*_req: Request being operated on
 * @wait_dma_copy: Waiting for an OCS DMA copy to complete
 * @dma_copy_completion: Completion to indicate DMA copy is done
 * @dma_err_mask: Error reported by OCS DMA interrupts
 * @engine: Crypto engine for the device
 */
struct ocs_aes_dev {
	struct list_head list;
	struct device *dev;
	int irq;
	void __iomem *base_reg;
	bool req_is_aead;
	union {
		struct skcipher_request *sk_req;
		struct aead_request *aead_req;
	};
	bool wait_dma_copy;
	struct completion dma_copy_completion;
	u32 dma_err_mask;
	struct crypto_engine *engine;
};

enum aes_instruction {
	AES_ENCRYPT = 0,
	AES_DECRYPT = 1,
	AES_EXPAND = 2,
	AES_BYPASS = 3,
};

enum aes_mode {
	AES_MODE_ECB = 0,
	AES_MODE_CBC = 1,
	AES_MODE_CTR = 2,
	AES_MODE_CCM = 6,
	AES_MODE_GCM = 7,
	AES_MODE_CTS = 9,
};

enum ocs_cipher {
	OCS_AES = 0,
	OCS_SM4 = 1,
};

struct ocs_dma_linked_list {
	u32 address;
	u32 byte_count;
	u32 next;
	u32 reserved  :30;
	u32 freeze    :1;
	u32 terminate :1;
};

int ocs_aes_set_key(struct ocs_aes_dev *aes_dev,
		const u32 key_size, const u8 *key,
		const enum ocs_cipher cipher);

/* if present IV must be 16 bytes */
int ocs_aes_op(struct ocs_aes_dev *aes_dev,
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv, const u32 iv_size,
		const enum ocs_cipher cipher,
		const enum aes_mode mode,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor);

/* IV must be 12 bytes */
int ocs_aes_gcm_op(struct ocs_aes_dev *aes_dev,
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv,
		const dma_addr_t aad_descriptor, const u32 aad_size,
		const u32 tag_size, const enum ocs_cipher cipher,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor, u8 *tag);

/* nonce must be converted to CCM IV (expected/provided by crypto API)
 * For encrypt the tag is appended to the ciphertext in destination
 * For decrypt the authentication result is found in auth_res
 *	0 = authentication successful
 *	1 = authentication failed
 */
int ocs_aes_ccm_op(struct ocs_aes_dev *aes_dev,
		const dma_addr_t src_descriptor, const u32 src_size,
		const u8 *iv,
		const dma_addr_t aad_descriptor, const u32 aad_size,
		const u8 *in_tag, const u32 tag_size,
		const enum ocs_cipher cipher,
		const enum aes_instruction instruction,
		const dma_addr_t dst_descriptor, u8 *auth_res);

/* run kfree() on data_buf and aad_buf when finished with descriptor
 * data_descriptor, data_desc_size, aad_descriptor and aad_desc_size are output
 */
void ocs_create_linked_list_from_sg(struct ocs_aes_dev *aes_dev,
		struct scatterlist *sgl, u32 num_sgl_entries, u8 **aad_buf,
		dma_addr_t *aad_descriptor, u32 aad_size,
		u32 *aad_desc_size, u8 **data_buf,
		dma_addr_t *data_descriptor, u32 data_size,
		u32 *data_desc_size);

irqreturn_t ocs_aes_irq_handler(int irq, void *dev_id);

#endif
