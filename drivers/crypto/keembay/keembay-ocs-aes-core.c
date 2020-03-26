// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay OCS AES Crypto Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <crypto/engine.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>

#include "ocs-aes.h"

#define KMB_OCS_PRIORITY (350)
#define DRV_NAME "keembay-ocs-aes-driver"

#define AES_MIN_KEY_SIZE	16
#define AES_MAX_KEY_SIZE	32
#define AES_KEYSIZE_128		16
#define AES_KEYSIZE_256		32
#define SM4_KEY_SIZE		16

/* Transform context */
struct ocs_aes_ctx {
	struct crypto_engine_ctx engine_ctx;
	struct ocs_aes_dev *aes_dev;
	u8 key[AES_KEYSIZE_256];
	unsigned int key_len;
	enum ocs_cipher cipher;
};

struct ocs_aes_reqctx {
	enum aes_instruction instruction;
	enum aes_mode mode;

	/* internal state variables */
	int num_src;
	int num_dst;
	int in_place;
	u32 src_desc_size;
	u8 *src_desc_buf;
	dma_addr_t src_desc;
	u32 dst_desc_size;
	u8 *dst_desc_buf;
	dma_addr_t dst_desc;

	/* CBC specific */
	u8 last_ct_blk[AES_BLOCK_SIZE]; /* last ciphertext block */

	/* CTS specific */
	int cts_swap;

	/* AEAD specific */
	u32 aad_src_desc_size;
	u8 *aad_src_desc_buf;
	dma_addr_t aad_src_desc;
	u32 aad_dst_desc_size;
	u8 *aad_dst_desc_buf;
	dma_addr_t aad_dst_desc;
	u8 in_tag[AES_BLOCK_SIZE];

	/* GCM specific */
	u8 out_tag[AES_BLOCK_SIZE];

	/* CCM specific */
	u8 ccm_auth_res;
};

/* Driver data. */
struct ocs_aes_drv {
	struct list_head dev_list;
	spinlock_t lock;
};

static struct ocs_aes_drv ocs_aes = {
	.dev_list = LIST_HEAD_INIT(ocs_aes.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(ocs_aes.lock),
};

/* Ensure key is 128-bit or 256-bit for AES
 * or 128-bit for SM4
 * and an actual key is being passed in
 * return 0 if key is OK
 * else return -EINVAL
 */
static inline int check_key(const u8 *in_key, unsigned int key_len,
		enum ocs_cipher cipher)
{
	if (!in_key)
		return -EINVAL;

	if ((cipher == OCS_AES) &&
			!((key_len == AES_KEYSIZE_128) ||
				(key_len == AES_KEYSIZE_256)))
		return -EINVAL;

	if ((cipher == OCS_SM4) && !(key_len == AES_KEYSIZE_128))
		return -EINVAL;

	return 0;
}

static inline int save_key(struct ocs_aes_ctx *ctx, const u8 *in_key,
		unsigned int key_len, enum ocs_cipher cipher)
{
	int ret;

	ret = check_key(in_key, key_len, cipher);
	if (!ret) {
		memcpy(ctx->key, in_key, key_len);
		ctx->key_len = key_len;
		ctx->cipher = cipher;
	}

	return ret;
}

static inline int sk_set_key(struct crypto_skcipher *tfm, const u8 *in_key,
		       unsigned int key_len, enum ocs_cipher cipher)
{
	struct ocs_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	return save_key(ctx, in_key, key_len, cipher);
}

static inline int aead_set_key(struct crypto_aead *tfm, const u8 *in_key,
		       unsigned int key_len, enum ocs_cipher cipher)
{
	struct ocs_aes_ctx *ctx = crypto_aead_ctx(tfm);

	return save_key(ctx, in_key, key_len, cipher);
}

static inline void sg_swap_blocks(struct scatterlist *sgl, unsigned int nents,
		off_t blk1_offset, off_t blk2_offset)
{
	u8 tmp_buf1[AES_BLOCK_SIZE], tmp_buf2[AES_BLOCK_SIZE];

	/* No easy way to copy within sg list, so copy both
	 * blocks to temporary buffers first
	 */
	sg_pcopy_to_buffer(sgl, nents, tmp_buf1, AES_BLOCK_SIZE, blk1_offset);
	sg_pcopy_to_buffer(sgl, nents, tmp_buf2, AES_BLOCK_SIZE, blk2_offset);
	sg_pcopy_from_buffer(sgl, nents, tmp_buf1, AES_BLOCK_SIZE, blk2_offset);
	sg_pcopy_from_buffer(sgl, nents, tmp_buf2, AES_BLOCK_SIZE, blk1_offset);
}

static struct ocs_aes_dev *kmb_ocs_aes_find_dev(struct ocs_aes_ctx *tctx)
{
	struct ocs_aes_dev *aes_dev;

	spin_lock_bh(&ocs_aes.lock);

	if (!tctx->aes_dev) {
		/* Only a single OCS device available */
		aes_dev = list_first_entry(&ocs_aes.dev_list,
				struct ocs_aes_dev, list);
		tctx->aes_dev = aes_dev;
	} else
		aes_dev = tctx->aes_dev;

	spin_unlock_bh(&ocs_aes.lock);

	return aes_dev;
}

static int aes_skcipher_common(struct skcipher_request *req,
		const enum ocs_cipher cipher,
		const enum aes_instruction instruction,
		const enum aes_mode mode)
{
	struct ocs_aes_ctx *ctx =
		crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct ocs_aes_reqctx *rctx = skcipher_request_ctx(req);
	struct ocs_aes_dev *dd;

	dd = kmb_ocs_aes_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	if (cipher != ctx->cipher)
		return -EINVAL;

	rctx->instruction = instruction;
	rctx->mode = mode;

	return crypto_transfer_skcipher_request_to_engine(dd->engine, req);
}

static void aes_skcipher_cleanup(struct skcipher_request *req)
{
	struct ocs_aes_ctx *ctx =
		crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct ocs_aes_reqctx *rctx = skcipher_request_ctx(req);

	if (rctx->num_src > 0)
		dma_unmap_sg(ctx->aes_dev->dev, req->src,
				rctx->num_src, DMA_TO_DEVICE);

	if (rctx->num_dst > 0) {
		if (rctx->in_place)
			dma_unmap_sg(ctx->aes_dev->dev, req->dst,
					rctx->num_dst, DMA_BIDIRECTIONAL);
		else
			dma_unmap_sg(ctx->aes_dev->dev, req->dst,
					rctx->num_dst, DMA_FROM_DEVICE);
	}

	if (rctx->src_desc_size) {
		dma_unmap_single(ctx->aes_dev->dev, rctx->src_desc,
				rctx->src_desc_size, DMA_TO_DEVICE);
		kfree(rctx->src_desc_buf);
	}

	if (rctx->dst_desc_size) {
		dma_unmap_single(ctx->aes_dev->dev, rctx->dst_desc,
				rctx->dst_desc_size, DMA_TO_DEVICE);
		kfree(rctx->dst_desc_buf);
	}
}

static int aes_skcipher_finalize(struct skcipher_request *req)
{
	struct ocs_aes_reqctx *rctx = skcipher_request_ctx(req);
	int iv_size = crypto_skcipher_ivsize(crypto_skcipher_reqtfm(req));

	aes_skcipher_cleanup(req);

	if (rctx->mode == AES_MODE_CBC) {
		if (rctx->instruction == AES_ENCRYPT) {
			scatterwalk_map_and_copy(req->iv, req->dst,
				req->cryptlen - iv_size, iv_size, 0);
		} else {
			if (rctx->in_place) {
				memcpy(req->iv, rctx->last_ct_blk, iv_size);
			} else {
				scatterwalk_map_and_copy(req->iv, req->src,
					req->cryptlen - iv_size, iv_size, 0);
			}
		}
	}

	if (rctx->cts_swap && (rctx->instruction == AES_ENCRYPT)) {
		sg_swap_blocks(req->dst, rctx->num_dst,
				(req->cryptlen - AES_BLOCK_SIZE),
				(req->cryptlen - (2 * AES_BLOCK_SIZE)));
	}

	return 0;
}

static int aes_skcipher_run(struct skcipher_request *req)
{
	int ret = 0, iv_size;

	struct ocs_aes_ctx *ctx =
		crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct ocs_aes_reqctx *rctx = skcipher_request_ctx(req);

	iv_size = crypto_skcipher_ivsize(crypto_skcipher_reqtfm(req));

	rctx->num_dst =  sg_nents_for_len(req->dst, req->cryptlen);
	if (rctx->num_dst < 0)
		return -EBADMSG;

	if ((rctx->mode == AES_MODE_CTS) &&
			(req->cryptlen > AES_BLOCK_SIZE) &&
			(!(req->cryptlen % AES_BLOCK_SIZE))) {
		/* If 2 blocks or greater, and multiple of block size
		 * swap last two blocks to be compatible with other
		 * crypto API CTS implementations
		 * i.e. from https://nvlpubs.nist.gov/nistpubs/Legacy/SP/
		 * nistspecialpublication800-38a-add.pdf OCS mode uses
		 * CBC-CS2, whereas other crypto API implementations use
		 * CBC-CS3
		 */
		rctx->cts_swap = 1;
	} else
		rctx->cts_swap = 0;

	if (sg_virt(req->src) == sg_virt(req->dst)) {
		/* src and dst buffers are the same,
		 * so must use bidirectional DMA mapping
		 */
		rctx->in_place = 1;

		if ((rctx->instruction == AES_DECRYPT) &&
				(rctx->mode == AES_MODE_CBC))
			scatterwalk_map_and_copy(rctx->last_ct_blk, req->src,
					(req->cryptlen - iv_size), iv_size, 0);

		if (rctx->cts_swap && (rctx->instruction == AES_DECRYPT))
			sg_swap_blocks(req->dst, rctx->num_dst,
					(req->cryptlen - AES_BLOCK_SIZE),
					(req->cryptlen - (2 * AES_BLOCK_SIZE)));

		ret = dma_map_sg(ctx->aes_dev->dev, req->dst,
				rctx->num_dst, DMA_BIDIRECTIONAL);

	} else {
		rctx->in_place = 0;

		rctx->num_src =  sg_nents_for_len(req->src, req->cryptlen);
		if (rctx->num_src < 0)
			return -EBADMSG;

		ret = dma_map_sg(ctx->aes_dev->dev, req->src,
				rctx->num_src, DMA_TO_DEVICE);
		if (ret != rctx->num_src) {
			dev_err(ctx->aes_dev->dev, "Failed to map source sg\n");
			ret = -ENOMEM;
			rctx->num_src = 0; /* to prevent unmapping in cleanup */
			goto cleanup;
		}
		ocs_create_linked_list_from_sg(ctx->aes_dev,
				req->src, rctx->num_src,
				NULL, NULL, 0, NULL,
				&(rctx->src_desc_buf), &(rctx->src_desc),
				req->cryptlen, &(rctx->src_desc_size));

		ret = dma_map_sg(ctx->aes_dev->dev, req->dst,
				rctx->num_dst, DMA_FROM_DEVICE);
	}

	if (ret != rctx->num_dst) {
		dev_err(ctx->aes_dev->dev, "Failed to map destination sg\n");
		ret = -ENOMEM;
		rctx->num_dst = 0; /* to prevent unmapping in cleanup */
		goto cleanup;
	}
	ocs_create_linked_list_from_sg(ctx->aes_dev, req->dst, rctx->num_dst,
			NULL, NULL, 0, NULL,
			&(rctx->dst_desc_buf), &(rctx->dst_desc),
			req->cryptlen, &(rctx->dst_desc_size));

	if (rctx->in_place && rctx->dst_desc_size) {
		rctx->src_desc = rctx->dst_desc;
	} else if (!rctx->src_desc_size || !rctx->dst_desc_size) {
		ret = -EINVAL;
		goto cleanup;
	}

	if ((!rctx->in_place) && rctx->cts_swap &&
			(rctx->instruction == AES_DECRYPT)) {
		/* Not in place, so have to copy src to dst (as we can't
		 * modify src)
		 * The mode (AES_MODE_ECB) and cipher (OCS_AES) is ignored
		 * when the instruction is AES_BYPASS
		 * TODO: for anything other than small data sizes this is
		 * very inefficient. Ideally everything other than last 2
		 * blocks would be read from src for decryption, and last
		 * 2 blocks swapped and read from elsewhere. But currently
		 * implemented OCS API does not allow this
		 */

		reinit_completion(&ctx->aes_dev->dma_copy_completion);

		/* Setting this before starting copy to prevent potential
		 * race condition with interrupt handler thread
		 */
		ctx->aes_dev->wait_dma_copy = true;

		ret = ocs_aes_op(ctx->aes_dev,
				rctx->src_desc, req->cryptlen,
				NULL, 0, OCS_AES, AES_MODE_ECB,
				AES_BYPASS, rctx->dst_desc);
		if (ret)
			goto cleanup;

		wait_for_completion(&ctx->aes_dev->dma_copy_completion);
		if (ctx->aes_dev->dma_err_mask) {
			/* OCS DMA HW error encountered. Abort */
			ret = -EIO;
			goto cleanup;
		}

		/* unmap src and dst */
		dma_unmap_sg(ctx->aes_dev->dev, req->src,
				rctx->num_src, DMA_TO_DEVICE);
		rctx->num_src = 0; /* no longer needed */
		dma_unmap_sg(ctx->aes_dev->dev, req->dst,
				rctx->num_dst, DMA_FROM_DEVICE);

		sg_swap_blocks(req->dst, rctx->num_dst,
				(req->cryptlen - AES_BLOCK_SIZE),
				(req->cryptlen - (2 * AES_BLOCK_SIZE)));

		/* remap dst, but now in_place */
		ret = dma_map_sg(ctx->aes_dev->dev, req->dst,
				rctx->num_dst, DMA_BIDIRECTIONAL);
		rctx->in_place = 1;
		if (ret != rctx->num_dst) {
			dev_err(ctx->aes_dev->dev, "Failed to map destination sg\n");
			ret = -ENOMEM;
			rctx->num_dst = 0; /* to prevent unmapping in cleanup */
			goto cleanup;
		}

		/* discard existing linked lists */
		if (rctx->src_desc_size) {
			dma_unmap_single(ctx->aes_dev->dev, rctx->src_desc,
					rctx->src_desc_size, DMA_TO_DEVICE);
			kfree(rctx->src_desc_buf);
			rctx->src_desc_size = 0;
		}
		if (rctx->dst_desc_size) {
			dma_unmap_single(ctx->aes_dev->dev, rctx->dst_desc,
					rctx->dst_desc_size, DMA_TO_DEVICE);
			kfree(rctx->dst_desc_buf);
			rctx->dst_desc_buf = NULL;
			rctx->dst_desc_size = 0;
		}

		/* need to recreate dst linked list */
		ocs_create_linked_list_from_sg(ctx->aes_dev,
				req->dst, rctx->num_dst,
				NULL, NULL, 0, NULL,
				&(rctx->dst_desc_buf), &(rctx->dst_desc),
				req->cryptlen, &(rctx->dst_desc_size));

		if (rctx->dst_desc_size) {
			/* If descriptor creation was successful,
			 * point the src_desc at the dst_desc,
			 * as we do in-place AES operation on the src
			 */
			rctx->src_desc = rctx->dst_desc;
		} else {
			ret = -1;
			goto cleanup;
		}
	}

	if (rctx->mode == AES_MODE_ECB)
		ret = ocs_aes_op(ctx->aes_dev, rctx->src_desc, req->cryptlen,
				NULL, 0, ctx->cipher, rctx->mode,
				rctx->instruction, rctx->dst_desc);
	else
		ret = ocs_aes_op(ctx->aes_dev, rctx->src_desc, req->cryptlen,
				req->iv, AES_BLOCK_SIZE, ctx->cipher,
				rctx->mode, rctx->instruction, rctx->dst_desc);

	if (!ret)
		return ret;

cleanup:
	aes_skcipher_cleanup(req);

	return ret;
}

static int aes_aead_cipher_common(struct aead_request *req,
		const enum ocs_cipher cipher,
		const enum aes_instruction instruction,
		const enum aes_mode mode)
{
	struct ocs_aes_ctx *ctx =
		crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct ocs_aes_reqctx *rctx = aead_request_ctx(req);
	struct ocs_aes_dev *dd;

	dd = kmb_ocs_aes_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	if (cipher != ctx->cipher)
		return -EINVAL;

	rctx->instruction = instruction;
	rctx->mode = mode;

	return crypto_transfer_aead_request_to_engine(dd->engine, req);
}

static void aes_aead_cipher_cleanup(struct aead_request *req)
{
	struct ocs_aes_ctx *ctx =
		crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct ocs_aes_reqctx *rctx = aead_request_ctx(req);

	if (rctx->num_src > 0)
		dma_unmap_sg(ctx->aes_dev->dev, req->src,
				rctx->num_src, DMA_TO_DEVICE);

	if (rctx->num_dst > 0) {
		if (rctx->in_place)
			dma_unmap_sg(ctx->aes_dev->dev, req->dst,
					rctx->num_dst, DMA_BIDIRECTIONAL);
		else
			dma_unmap_sg(ctx->aes_dev->dev, req->dst,
					rctx->num_dst, DMA_FROM_DEVICE);
	}

	if (rctx->aad_src_desc_size) {
		dma_unmap_single(ctx->aes_dev->dev, rctx->aad_src_desc,
				rctx->aad_src_desc_size, DMA_TO_DEVICE);
		kfree(rctx->aad_src_desc_buf);
	}

	if (rctx->aad_dst_desc_size) {
		dma_unmap_single(ctx->aes_dev->dev, rctx->aad_dst_desc,
				rctx->aad_dst_desc_size, DMA_TO_DEVICE);
		kfree(rctx->aad_dst_desc_buf);
	}

	if (rctx->src_desc_size) {
		dma_unmap_single(ctx->aes_dev->dev, rctx->src_desc,
				rctx->src_desc_size, DMA_TO_DEVICE);
		kfree(rctx->src_desc_buf);
	}

	if (rctx->dst_desc_size) {
		dma_unmap_single(ctx->aes_dev->dev, rctx->dst_desc,
				rctx->dst_desc_size, DMA_TO_DEVICE);
		kfree(rctx->dst_desc_buf);
	}
}

/*
 * For encrypt
 *		return 0
 * For decrypt
 *		if tag check passes return 0
 *		else return -EBADMSG
 */
static int aes_aead_cipher_finalize(struct aead_request *req)
{
	struct ocs_aes_reqctx *rctx = aead_request_ctx(req);
	int tag_size, ret = 0;

	tag_size = crypto_aead_authsize(crypto_aead_reqtfm(req));

	aes_aead_cipher_cleanup(req);

	if (rctx->mode == AES_MODE_GCM) {
		if (rctx->instruction == AES_DECRYPT) {
			if (memcmp(rctx->in_tag, rctx->out_tag, tag_size))
				ret = -EBADMSG;
		} else {
			/* For encrypt
			 * copy tag to destination sg
			 * after AAD and CT
			 */
			sg_pcopy_from_buffer(req->dst, rctx->num_dst,
					rctx->out_tag, tag_size,
					req->assoclen + req->cryptlen);
		}
	} else if ((rctx->mode == AES_MODE_CCM) &&
			(rctx->instruction == AES_DECRYPT)) {
		ret = (rctx->ccm_auth_res == OCS_CCM_TAG_OK) ? 0 : -EBADMSG;
	}

	return ret;
}

static int aes_aead_cipher_run(struct aead_request *req)
{
	u32 tag_size, in_size, out_size, dst_size;
	int ret;

	struct ocs_aes_ctx *ctx =
		crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct ocs_aes_reqctx *rctx = aead_request_ctx(req);

	/* For encrypt:
	 *		src sg list:
	 *			AAD|PT
	 *		dst sg list expects:
	 *			AAD|CT|tag
	 * For decrypt:
	 *		src sg list:
	 *			AAD|CT|tag
	 *		dst sg list expects:
	 *			AAD|PT
	 */

	tag_size = crypto_aead_authsize(crypto_aead_reqtfm(req));

	rctx->num_src = sg_nents_for_len(req->src,
			(req->assoclen + req->cryptlen));
	if (rctx->num_src < 0)
		return -EBADMSG;

	/* for decrypt req->cryptlen includes both ciphertext size and
	 * tag size. However for encrypt it only contains the plaintext
	 * size
	 */
	in_size = req->cryptlen;
	if (rctx->instruction == AES_DECRYPT) {
		in_size -= tag_size;

		/* copy tag from source scatter-gather list to
		 * in_tag buffer, before DMA mapping, as will
		 * be required later
		 */
		sg_pcopy_to_buffer(req->src, rctx->num_src, rctx->in_tag,
				tag_size, req->assoclen + in_size);

		out_size = in_size;

		dst_size = req->assoclen + in_size;
	} else {
		/* In CCM mode the OCS engine appends the tag to the ciphertext,
		 * but in GCM mode the tag must be read from the tag registers
		 * and appended manually below
		 */
		out_size = (rctx->mode == AES_MODE_CCM) ?
			(in_size + tag_size) : in_size;

		dst_size = req->assoclen + in_size + tag_size;
	}

	rctx->num_dst = sg_nents_for_len(req->dst, dst_size);
	if (rctx->num_dst < 0)
		return -EBADMSG;

	if (sg_virt(req->src) == sg_virt(req->dst)) {
		/* src and dst buffers are the same,
		 * so must use bidirectional DMA mapping
		 */
		rctx->in_place = 1;

		/* no longer needed as not mapping src separately */
		rctx->num_src = 0;

		ret = dma_map_sg(ctx->aes_dev->dev, req->dst,
				rctx->num_dst, DMA_BIDIRECTIONAL);
	} else {
		rctx->in_place = 0;

		ret = dma_map_sg(ctx->aes_dev->dev, req->src,
				rctx->num_src, DMA_TO_DEVICE);
		if (ret != rctx->num_src) {
			dev_err(ctx->aes_dev->dev, "Failed to map source sg\n");
			ret = -ENOMEM;
			rctx->num_src = 0; /* to prevent unmapping in cleanup */
			goto cleanup;
		}
		ocs_create_linked_list_from_sg(ctx->aes_dev,
				req->src, rctx->num_src,
				&(rctx->aad_src_desc_buf),
				&(rctx->aad_src_desc),
				req->assoclen, &(rctx->aad_src_desc_size),
				&(rctx->src_desc_buf), &(rctx->src_desc),
				in_size, &(rctx->src_desc_size));

		ret = dma_map_sg(ctx->aes_dev->dev, req->dst,
				rctx->num_dst, DMA_FROM_DEVICE);
	}

	if (ret != rctx->num_dst) {
		dev_err(ctx->aes_dev->dev, "Failed to map destination sg\n");
		ret = -ENOMEM;
		rctx->num_dst = 0; /* to prevent unmapping in cleanup */
		goto cleanup;
	}

	/* aad_dst_desc to be used to copy aad from src to dst */
	ocs_create_linked_list_from_sg(ctx->aes_dev, req->dst, rctx->num_dst,
			&(rctx->aad_dst_desc_buf), &(rctx->aad_dst_desc),
			req->assoclen, &(rctx->aad_dst_desc_size),
			&(rctx->dst_desc_buf), &(rctx->dst_desc),
			out_size, &(rctx->dst_desc_size));

	if (rctx->in_place && in_size &&
			(rctx->mode == AES_MODE_CCM) &&
			(rctx->instruction == AES_ENCRYPT)) {
		/* For CCM encrypt the input and output linked lists contain
		 * different amounts of data.
		 * If !in_place already dealt with above
		 * If !in_size the source linked list is never used
		 */
		ocs_create_linked_list_from_sg(ctx->aes_dev,
				req->dst, rctx->num_dst,
				&(rctx->aad_src_desc_buf),
				&(rctx->aad_src_desc),
				req->assoclen, &(rctx->aad_src_desc_size),
				&(rctx->src_desc_buf), &(rctx->src_desc),
				in_size, &(rctx->src_desc_size));
	} else if (rctx->in_place && ((rctx->dst_desc_size && in_size) ||
				(rctx->aad_dst_desc_size && req->assoclen))) {
		/* in place operation, so src and dst are same */
		rctx->src_desc = rctx->dst_desc;
		rctx->aad_src_desc = rctx->aad_dst_desc;
	} else if ((in_size && (!rctx->src_desc_size || !rctx->dst_desc_size))
			|| (req->assoclen && !rctx->aad_src_desc_size)) {
		/* there is data and/or aad, but linked list(s) not created */
		ret = -EINVAL;
		goto cleanup;
	} else if (req->assoclen && rctx->aad_src_desc_size &&
			rctx->aad_dst_desc_size) {
		/* copy aad from src to dst using OCS DMA
		 * the mode (AES_MODE_ECB) and cipher (OCS_AES) is
		 * ignored when the instruction is AES_BYPASS
		 */
		reinit_completion(&ctx->aes_dev->dma_copy_completion);

		/* Setting this before starting copy
		 * to prevent potential race condition
		 * with interrupt handler thread
		 */
		ctx->aes_dev->wait_dma_copy = true;

		ret = ocs_aes_op(ctx->aes_dev, rctx->aad_src_desc,
				in_size, NULL, 0, OCS_AES, AES_MODE_ECB,
				AES_BYPASS, rctx->aad_dst_desc);
		if (ret) {
			dev_err(ctx->aes_dev->dev, "Unable to copy source AAD to destination AAD\n");
			goto cleanup;
		}

		wait_for_completion(&ctx->aes_dev->dma_copy_completion);
		if (ctx->aes_dev->dma_err_mask) {
			/* OCS DMA HW error encountered. Abort */
			ret = -EIO;
			goto cleanup;
		}
	}

	if (rctx->mode == AES_MODE_GCM) {
		ret = ocs_aes_gcm_op(ctx->aes_dev,
				rctx->src_desc, in_size, req->iv,
				rctx->aad_src_desc, req->assoclen, tag_size,
				ctx->cipher, rctx->instruction, rctx->dst_desc,
				rctx->out_tag);
	} else {
		ret = ocs_aes_ccm_op(ctx->aes_dev,
				rctx->src_desc, in_size, req->iv,
				rctx->aad_src_desc, req->assoclen,
				rctx->in_tag, tag_size,
				ctx->cipher, rctx->instruction, rctx->dst_desc,
				&(rctx->ccm_auth_res));
	}

	if (!ret)
		return ret;

cleanup:
	aes_aead_cipher_cleanup(req);

	return ret;
}

/* called to indicate operation completion */
static irqreturn_t kmb_ocs_aes_irq_thread(int irq, void *dev_id)
{
	struct ocs_aes_dev *aes_dev = (struct ocs_aes_dev *)dev_id;
	int err;

	if (aes_dev->wait_dma_copy) {
		aes_dev->wait_dma_copy = false;
		complete(&aes_dev->dma_copy_completion);
	} else if (aes_dev->req_is_aead) {
		if (aes_dev->dma_err_mask) {
			aes_aead_cipher_cleanup(aes_dev->aead_req);
			err = -EIO;
		} else
			err = aes_aead_cipher_finalize(aes_dev->aead_req);

		crypto_finalize_aead_request(aes_dev->engine,
				aes_dev->aead_req, err);
	} else {
		if (aes_dev->dma_err_mask) {
			aes_skcipher_cleanup(aes_dev->sk_req);
			err = -EIO;
		} else
			err = aes_skcipher_finalize(aes_dev->sk_req);

		crypto_finalize_skcipher_request(aes_dev->engine,
				aes_dev->sk_req, err);
	}

	return IRQ_HANDLED;
}

static int kmb_ocs_aes_sk_prepare_request(struct crypto_engine *engine,
		void *areq)
{
	struct skcipher_request *req =
		container_of(areq, struct skcipher_request, base);
	struct ocs_aes_ctx *ctx =
		crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct ocs_aes_reqctx *rctx = skcipher_request_ctx(req);

	ctx->aes_dev->req_is_aead = false;
	ctx->aes_dev->sk_req = req;
	ctx->aes_dev->wait_dma_copy = false;

	rctx->num_src = 0;
	rctx->src_desc_size = 0;
	rctx->src_desc_buf = NULL;
	rctx->dst_desc_size = 0;
	rctx->dst_desc_buf = NULL;
	memset(rctx->last_ct_blk, 0, AES_BLOCK_SIZE);

	return 0;
}

static int kmb_ocs_aes_sk_do_one_request(struct crypto_engine *engine,
		void *areq)
{
	struct skcipher_request *req =
		container_of(areq, struct skcipher_request, base);
	struct ocs_aes_ctx *ctx =
		crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	int err;

	if (!ctx->aes_dev)
		return -ENODEV;

	err = ocs_aes_set_key(ctx->aes_dev, ctx->key_len,
			ctx->key, ctx->cipher);
	if (err)
		goto ret_err;

	err = aes_skcipher_run(req);
	if (err)
		goto ret_err;

	return 0;

ret_err:
	crypto_finalize_skcipher_request(ctx->aes_dev->engine,
			ctx->aes_dev->sk_req, err);

	return 0;
}

static int kmb_ocs_aes_aead_prepare_request(struct crypto_engine *engine,
		void *areq)
{
	struct aead_request *req =
		container_of(areq, struct aead_request, base);
	struct ocs_aes_ctx *ctx =
		crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct ocs_aes_reqctx *rctx = aead_request_ctx(req);

	ctx->aes_dev->req_is_aead = true;
	ctx->aes_dev->aead_req = req;
	ctx->aes_dev->wait_dma_copy = false;

	rctx->num_src = 0;
	rctx->src_desc_size = 0;
	rctx->src_desc_buf = NULL;
	rctx->src_desc = 0;
	rctx->aad_src_desc_size = 0;
	rctx->aad_src_desc_buf = NULL;
	rctx->aad_src_desc = 0;
	rctx->aad_dst_desc_size = 0;
	rctx->aad_dst_desc_buf = NULL;
	rctx->dst_desc_size = 0;
	rctx->dst_desc_buf = NULL;

	memset(rctx->out_tag, 0, AES_BLOCK_SIZE);
	rctx->ccm_auth_res = OCS_CCM_TAG_INVALID;

	return 0;
}

static int kmb_ocs_aes_aead_do_one_request(struct crypto_engine *engine,
		void *areq)
{
	struct aead_request *req =
		container_of(areq, struct aead_request, base);
	struct ocs_aes_ctx *ctx =
		crypto_aead_ctx(crypto_aead_reqtfm(req));
	int err;

	if (!ctx->aes_dev)
		return -ENODEV;

	err = ocs_aes_set_key(ctx->aes_dev, ctx->key_len,
			ctx->key, ctx->cipher);
	if (err)
		goto ret_err;

	err = aes_aead_cipher_run(req);
	if (err)
		goto ret_err;

	return 0;

ret_err:
	crypto_finalize_aead_request(ctx->aes_dev->engine,
			ctx->aes_dev->aead_req, err);

	return 0;
}

#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES
static int kmb_ocs_aes_set_key(struct crypto_skcipher *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	return sk_set_key(tfm, in_key, key_len, OCS_AES);
}

static int kmb_ocs_aes_aead_set_key(struct crypto_aead *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	return aead_set_key(tfm, in_key, key_len, OCS_AES);
}

#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_ECB
static int kmb_ocs_aes_ecb_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_ENCRYPT, AES_MODE_ECB);
}

static int kmb_ocs_aes_ecb_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_DECRYPT, AES_MODE_ECB);
}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_ECB */

static int kmb_ocs_aes_cbc_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_ENCRYPT, AES_MODE_CBC);
}

static int kmb_ocs_aes_cbc_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_DECRYPT, AES_MODE_CBC);
}

static int kmb_ocs_aes_ctr_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_ENCRYPT, AES_MODE_CTR);
}

static int kmb_ocs_aes_ctr_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_DECRYPT, AES_MODE_CTR);
}

#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_CTS
static int kmb_ocs_aes_cts_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_ENCRYPT, AES_MODE_CTS);
}

static int kmb_ocs_aes_cts_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_AES, AES_DECRYPT, AES_MODE_CTS);
}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_CTS */

static int kmb_ocs_aes_gcm_encrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_AES, AES_ENCRYPT, AES_MODE_GCM);
}

static int kmb_ocs_aes_gcm_decrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_AES, AES_DECRYPT, AES_MODE_GCM);
}

static int kmb_ocs_aes_ccm_encrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_AES, AES_ENCRYPT, AES_MODE_CCM);
}

static int kmb_ocs_aes_ccm_decrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_AES, AES_DECRYPT, AES_MODE_CCM);
}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES */

#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4
static int kmb_ocs_sm4_set_key(struct crypto_skcipher *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	return sk_set_key(tfm, in_key, key_len, OCS_SM4);
}

static int kmb_ocs_sm4_aead_set_key(struct crypto_aead *tfm, const u8 *in_key,
		       unsigned int key_len)
{
	return aead_set_key(tfm, in_key, key_len, OCS_SM4);
}

#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_ECB
static int kmb_ocs_sm4_ecb_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_ENCRYPT, AES_MODE_ECB);
}

static int kmb_ocs_sm4_ecb_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_DECRYPT, AES_MODE_ECB);
}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_ECB */

static int kmb_ocs_sm4_cbc_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_ENCRYPT, AES_MODE_CBC);
}

static int kmb_ocs_sm4_cbc_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_DECRYPT, AES_MODE_CBC);
}

static int kmb_ocs_sm4_ctr_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_ENCRYPT, AES_MODE_CTR);
}

static int kmb_ocs_sm4_ctr_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_DECRYPT, AES_MODE_CTR);
}

#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_CTS
static int kmb_ocs_sm4_cts_encrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_ENCRYPT, AES_MODE_CTS);
}

static int kmb_ocs_sm4_cts_decrypt(struct skcipher_request *req)
{
	return aes_skcipher_common(req, OCS_SM4, AES_DECRYPT, AES_MODE_CTS);
}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_CTS */

static int kmb_ocs_sm4_gcm_encrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_SM4, AES_ENCRYPT, AES_MODE_GCM);
}

static int kmb_ocs_sm4_gcm_decrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_SM4, AES_DECRYPT, AES_MODE_GCM);
}

static int kmb_ocs_sm4_ccm_encrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_SM4, AES_ENCRYPT, AES_MODE_CCM);
}

static int kmb_ocs_sm4_ccm_decrypt(struct aead_request *req)
{
	return aes_aead_cipher_common(req, OCS_SM4, AES_DECRYPT, AES_MODE_CCM);
}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4 */

static int ocs_aes_init_tfm(struct crypto_skcipher *tfm)
{
	struct ocs_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct ocs_aes_reqctx));

	ctx->engine_ctx.op.prepare_request = kmb_ocs_aes_sk_prepare_request;
	ctx->engine_ctx.op.do_one_request = kmb_ocs_aes_sk_do_one_request;
	ctx->engine_ctx.op.unprepare_request = NULL;

	return 0;
}

static inline void clear_key(struct ocs_aes_ctx *ctx)
{
	memzero_explicit(ctx->key, AES_KEYSIZE_256);

	/* zero key registers if set */
	if (ctx->aes_dev)
		ocs_aes_set_key(ctx->aes_dev, AES_KEYSIZE_256,
				ctx->key, OCS_AES);

}

static void ocs_aes_exit_tfm(struct crypto_skcipher *tfm)
{
	struct ocs_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	clear_key(ctx);
}

static int ocs_aes_aead_cra_init(struct crypto_aead *tfm)
{
	struct ocs_aes_ctx *ctx = crypto_aead_ctx(tfm);

	tfm->reqsize = sizeof(struct ocs_aes_reqctx);

	ctx->engine_ctx.op.prepare_request = kmb_ocs_aes_aead_prepare_request;
	ctx->engine_ctx.op.do_one_request = kmb_ocs_aes_aead_do_one_request;
	ctx->engine_ctx.op.unprepare_request = NULL;

	return 0;
}

static void ocs_aes_aead_cra_exit(struct crypto_aead *tfm)
{
	struct ocs_aes_ctx *ctx = crypto_aead_ctx(tfm);

	clear_key(ctx);
}

static struct skcipher_alg algs[] = {
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_ECB
	{
		.base.cra_name = "ecb(aes)",
		.base.cra_driver_name = "ecb-aes-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.setkey = kmb_ocs_aes_set_key,
		.encrypt = kmb_ocs_aes_ecb_encrypt,
		.decrypt = kmb_ocs_aes_ecb_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_ECB */
	{
		.base.cra_name = "cbc(aes)",
		.base.cra_driver_name = "cbc-aes-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_aes_set_key,
		.encrypt = kmb_ocs_aes_cbc_encrypt,
		.decrypt = kmb_ocs_aes_cbc_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
	{
		.base.cra_name = "ctr(aes)",
		.base.cra_driver_name = "ctr-aes-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = 1,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_aes_set_key,
		.encrypt = kmb_ocs_aes_ctr_encrypt,
		.decrypt = kmb_ocs_aes_ctr_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_CTS
	{
		.base.cra_name = "cts(cbc(aes))",
		.base.cra_driver_name = "cts-aes-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = AES_MIN_KEY_SIZE,
		.max_keysize = AES_MAX_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_aes_set_key,
		.encrypt = kmb_ocs_aes_cts_encrypt,
		.decrypt = kmb_ocs_aes_cts_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_CTS */
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES */
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_ECB
	{
		.base.cra_name = "ecb(sm4)",
		.base.cra_driver_name = "ecb-sm4-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.setkey = kmb_ocs_sm4_set_key,
		.encrypt = kmb_ocs_sm4_ecb_encrypt,
		.decrypt = kmb_ocs_sm4_ecb_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_ECB */
	{
		.base.cra_name = "cbc(sm4)",
		.base.cra_driver_name = "cbc-sm4-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_sm4_set_key,
		.encrypt = kmb_ocs_sm4_cbc_encrypt,
		.decrypt = kmb_ocs_sm4_cbc_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
	{
		.base.cra_name = "ctr(sm4)",
		.base.cra_driver_name = "ctr-sm4-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = 1,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_sm4_set_key,
		.encrypt = kmb_ocs_sm4_ctr_encrypt,
		.decrypt = kmb_ocs_sm4_ctr_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	},
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_CTS
	{
		.base.cra_name = "cts(cbc(sm4))",
		.base.cra_driver_name = "cts-sm4-keembay-ocs",
		.base.cra_priority = KMB_OCS_PRIORITY,
		.base.cra_flags = CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize = AES_BLOCK_SIZE,
		.base.cra_ctxsize = sizeof(struct ocs_aes_ctx),
		.base.cra_module = THIS_MODULE,
		.base.cra_alignmask = 0,

		.min_keysize = SM4_KEY_SIZE,
		.max_keysize = SM4_KEY_SIZE,
		.ivsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_sm4_set_key,
		.encrypt = kmb_ocs_sm4_cts_encrypt,
		.decrypt = kmb_ocs_sm4_cts_decrypt,
		.init = ocs_aes_init_tfm,
		.exit = ocs_aes_exit_tfm,
	}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_CTS */
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4 */
};

static struct aead_alg algs_aead[] = {
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES
	{
		.base = {
			.cra_name = "gcm(aes)",
			.cra_driver_name = "gcm-aes-keembay-ocs",
			.cra_priority = KMB_OCS_PRIORITY,
			.cra_flags = CRYPTO_ALG_ASYNC |
						 CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_blocksize = 1,
			.cra_ctxsize = sizeof(struct ocs_aes_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
		},
		.init = ocs_aes_aead_cra_init,
		.exit = ocs_aes_aead_cra_exit,
		.ivsize = GCM_AES_IV_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_aes_aead_set_key,
		.encrypt = kmb_ocs_aes_gcm_encrypt,
		.decrypt = kmb_ocs_aes_gcm_decrypt,
	},
	{
		.base = {
			.cra_name = "ccm(aes)",
			.cra_driver_name = "ccm-aes-keembay-ocs",
			.cra_priority = KMB_OCS_PRIORITY,
			.cra_flags = CRYPTO_ALG_ASYNC |
						 CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_blocksize = 1,
			.cra_ctxsize = sizeof(struct ocs_aes_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
		},
		.init = ocs_aes_aead_cra_init,
		.exit = ocs_aes_aead_cra_exit,
		.ivsize = AES_BLOCK_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_aes_aead_set_key,
		.encrypt = kmb_ocs_aes_ccm_encrypt,
		.decrypt = kmb_ocs_aes_ccm_decrypt,
	},
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES */
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4
	{
		.base = {
			.cra_name = "gcm(sm4)",
			.cra_driver_name = "gcm-sm4-keembay-ocs",
			.cra_priority = KMB_OCS_PRIORITY,
			.cra_flags = CRYPTO_ALG_ASYNC |
						 CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_blocksize = 1,
			.cra_ctxsize = sizeof(struct ocs_aes_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
		},
		.init = ocs_aes_aead_cra_init,
		.exit = ocs_aes_aead_cra_exit,
		.ivsize = GCM_AES_IV_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_sm4_aead_set_key,
		.encrypt = kmb_ocs_sm4_gcm_encrypt,
		.decrypt = kmb_ocs_sm4_gcm_decrypt,
	},
	{
		.base = {
			.cra_name = "ccm(sm4)",
			.cra_driver_name = "ccm-sm4-keembay-ocs",
			.cra_priority = KMB_OCS_PRIORITY,
			.cra_flags = CRYPTO_ALG_ASYNC |
						 CRYPTO_ALG_KERN_DRIVER_ONLY,
			.cra_blocksize = 1,
			.cra_ctxsize = sizeof(struct ocs_aes_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
		},
		.init = ocs_aes_aead_cra_init,
		.exit = ocs_aes_aead_cra_exit,
		.ivsize = AES_BLOCK_SIZE,
		.maxauthsize = AES_BLOCK_SIZE,
		.setkey = kmb_ocs_sm4_aead_set_key,
		.encrypt = kmb_ocs_sm4_ccm_encrypt,
		.decrypt = kmb_ocs_sm4_ccm_decrypt,
	}
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4 */
};

static void unregister_aes_algs(struct ocs_aes_dev *aes_dev)
{
	crypto_unregister_skciphers(algs, ARRAY_SIZE(algs));
	crypto_unregister_aeads(algs_aead, ARRAY_SIZE(algs_aead));
}

static int register_aes_algs(struct ocs_aes_dev *aes_dev)
{
	int ret;

	/* If any algorithm fails to register, all preceding
	 * algorithms that were successfully registered will
	 * be automatically unregistered
	 */
	ret = crypto_register_skciphers(algs, ARRAY_SIZE(algs));
	if (ret)
		return ret;

	ret = crypto_register_aeads(algs_aead, ARRAY_SIZE(algs_aead));
	if (ret)
		crypto_unregister_skciphers(algs, ARRAY_SIZE(algs));

	return ret;
}

/* Device tree driver match. */
static const struct of_device_id kmb_ocs_aes_of_match[] = {
	{
		.compatible = "intel,keembay-ocs-aes",
	},
	{}
};

static int kmb_ocs_aes_remove(struct platform_device *pdev)
{
	struct ocs_aes_dev *aes_dev;

	aes_dev = platform_get_drvdata(pdev);
	if (!aes_dev)
		return -ENODEV;

	unregister_aes_algs(aes_dev);

	spin_lock(&ocs_aes.lock);
	list_del(&aes_dev->list);
	spin_unlock(&ocs_aes.lock);

	crypto_engine_exit(aes_dev->engine);

	return 0;
}

static int kmb_ocs_aes_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct resource *aes_mem;
	struct ocs_aes_dev *aes_dev;

	aes_dev = devm_kzalloc(dev, sizeof(*aes_dev), GFP_KERNEL);
	if (!aes_dev)
		return -ENOMEM;

	aes_dev->dev = dev;

	platform_set_drvdata(pdev, aes_dev);

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(dev, "Failed to set 32 bit dma mask %d\n", rc);
		return -ENODEV;
	}

	INIT_LIST_HEAD(&aes_dev->list);

	/* Get base register address. */
	aes_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!aes_mem) {
		dev_err(dev, "Could not retrieve io mem resource\n");
		rc = -ENODEV;
		goto list_del;
	}

	aes_dev->base_reg = devm_ioremap_resource(&pdev->dev, aes_mem);
	if (IS_ERR(aes_dev->base_reg)) {
		dev_err(dev, "Failed to get base address\n");
		rc = PTR_ERR(aes_dev->base_reg);
		goto list_del;
	}

	/* Get and request IRQ */
	aes_dev->irq = platform_get_irq(pdev, 0);
	if (aes_dev->irq < 0) {
		dev_err(dev, "Could not retrieve IRQ\n");
		rc = aes_dev->irq;
		goto list_del;
	}

	rc = devm_request_threaded_irq(dev, aes_dev->irq,
			ocs_aes_irq_handler, kmb_ocs_aes_irq_thread,
			0, "keembay-ocs-aes", aes_dev);
	if (rc < 0) {
		dev_err(dev, "Could not request IRQ\n");
		goto list_del;
	}

	spin_lock(&ocs_aes.lock);
	list_add_tail(&aes_dev->list, &ocs_aes.dev_list);
	spin_unlock(&ocs_aes.lock);

	/* Initialize crypto engine */
	aes_dev->engine = crypto_engine_alloc_init(dev, 1);
	if (!aes_dev->engine)
		goto list_del;

	rc = crypto_engine_start(aes_dev->engine);
	if (rc) {
		dev_err(dev, "Could not start crypto engine\n");
		goto cleanup;
	}

	rc = register_aes_algs(aes_dev);
	if (rc) {
		dev_err(dev, "Could not register OCS algorithms with Crypto API\n");
		goto cleanup;
	}

	init_completion(&aes_dev->dma_copy_completion);

	return 0;

cleanup:
	crypto_engine_exit(aes_dev->engine);
list_del:
	spin_lock(&ocs_aes.lock);
	list_del(&aes_dev->list);
	spin_unlock(&ocs_aes.lock);

	return rc;
}

/* The OCS driver is a platform device. */
static struct platform_driver kmb_ocs_aes_driver = {
	.probe = kmb_ocs_aes_probe,
	.remove = kmb_ocs_aes_remove,
	.driver = {
			.name = DRV_NAME,
			.of_match_table = kmb_ocs_aes_of_match,
		},
};

module_platform_driver(kmb_ocs_aes_driver);

MODULE_DESCRIPTION("Keembay Offload and Crypto Subsystem (OCS) Symmetric Cipher Driver");
MODULE_LICENSE("GPL v2");
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_ECB
MODULE_ALIAS_CRYPTO("ecb(aes)");
MODULE_ALIAS_CRYPTO("ecb-aes-keembay-ocs");
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_ECB */
MODULE_ALIAS_CRYPTO("cbc(aes)");
MODULE_ALIAS_CRYPTO("cbc-aes-keembay-ocs");
MODULE_ALIAS_CRYPTO("ctr(aes)");
MODULE_ALIAS_CRYPTO("ctr-aes-keembay-ocs");
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_CTS
MODULE_ALIAS_CRYPTO("cts(cbc(aes))");
MODULE_ALIAS_CRYPTO("cts-aes-keembay-ocs");
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES_CTS */
MODULE_ALIAS_CRYPTO("gcm(aes)");
MODULE_ALIAS_CRYPTO("gcm-aes-keembay-ocs");
MODULE_ALIAS_CRYPTO("ccm(aes)");
MODULE_ALIAS_CRYPTO("ccm-aes-keembay-ocs");
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_AES */
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_ECB
MODULE_ALIAS_CRYPTO("ecb(sm4)");
MODULE_ALIAS_CRYPTO("ecb-sm4-keembay-ocs");
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_ECB */
MODULE_ALIAS_CRYPTO("cbc(sm4)");
MODULE_ALIAS_CRYPTO("cbc-sm4-keembay-ocs");
MODULE_ALIAS_CRYPTO("ctr(sm4)");
MODULE_ALIAS_CRYPTO("ctr-sm4-keembay-ocs");
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_CTS
MODULE_ALIAS_CRYPTO("cts(cbc(sm4))");
MODULE_ALIAS_CRYPTO("cts-sm4-keembay-ocs");
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4_CTS */
MODULE_ALIAS_CRYPTO("gcm(sm4)");
MODULE_ALIAS_CRYPTO("gcm-sm4-keembay-ocs");
MODULE_ALIAS_CRYPTO("ccm(sm4)");
MODULE_ALIAS_CRYPTO("ccm-sm4-keembay-ocs");
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_SYM_CIPHER_SM4 */
