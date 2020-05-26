// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay OCS HCU Crypto Driver.
 *
 * Copyright (C) 2018-2020 Intel Corporation
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include <crypto/engine.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha.h>
#include <crypto/sm3.h>
#include <crypto/hmac.h>
#include <crypto/internal/hash.h>

#include "ocs-hcu.h"

#define DRV_NAME "keembay-ocs-hcu-driver"

/* Request flags */
#define REQ_FLAGS_HASH_MASK (REQ_FLAGS_SHA_224 | \
			     REQ_FLAGS_SHA_256 | \
			     REQ_FLAGS_SHA_384 | \
			     REQ_FLAGS_SHA_512 | \
			     REQ_FLAGS_SM3)
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224
#define REQ_FLAGS_SHA_224 BIT(0)
#else /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224 */
#define REQ_FLAGS_SHA_224 0
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224 */
#define REQ_FLAGS_SHA_256 BIT(1)
#define REQ_FLAGS_SHA_384 BIT(2)
#define REQ_FLAGS_SHA_512 BIT(3)
#define REQ_FLAGS_SM3 BIT(4)
#define REQ_FLAGS_HMAC BIT(5)
#define REQ_FLAGS_HASH_KEY BIT(6)
#define REQ_FLAGS_HMAC_KEY_SET BIT(7)
#define REQ_FLAGS_HASH_PIO BIT(8)
#define REQ_FLAGS_INTERMEDIATE_DATA BIT(9)
#define REQ_FLAGS_FINAL_DATA BIT(10)
#define REQ_FLAGS_HMAC_HW BIT(12)
#define REQ_UPDATE BIT(16)
#define REQ_FINAL BIT(17)
#define REQ_CMD_MASK (REQ_UPDATE | REQ_FINAL)
#define REQ_HMAC_NO_SET_KEY_MASK (REQ_FLAGS_HASH_KEY | REQ_FLAGS_HMAC_KEY_SET)

#define REQ_FLAGS_HMAC_TYPE_MASK (REQ_FLAGS_HMAC | REQ_FLAGS_HMAC_HW)
#define REQ_FLAGS_HMAC_TYPE_SW REQ_FLAGS_HMAC
#define REQ_FLAGS_HMAC_TYPE_HW (REQ_FLAGS_HMAC | REQ_FLAGS_HMAC_HW)

#define KMB_HCU_BUF_SIZE 256
#define KMB_OCS_HCU_ALIGN_MASK 0x7

#define KMB_OCS_HCU_MAX_KEYLEN 512

/* Transform context. */
struct ocs_hcu_ctx {
	struct crypto_engine_ctx engine_ctx;
	struct ocs_hcu_dev *hcu_dev;
	u32 flags;
	u8 key[KMB_OCS_HCU_MAX_KEYLEN] __aligned(sizeof(u64));
	u8 opad[SHA512_BLOCK_SIZE + SHA512_DIGEST_SIZE];
	u8 ipad[SHA512_BLOCK_SIZE] __aligned(sizeof(u64));
	/* Length ot the key set by the setkey function. */
	size_t key_len;
};

/* Context for the request. */
struct ocs_hcu_rctx {
	u32 flags;
	size_t blk_sz;

	/* Head of the unhandled scatterlist entries containing data. */
	struct scatterlist *sg;
	unsigned int sg_dma_nents;
	dma_addr_t ll_dma_addr;
	/* OCS DMA linked list head. */
	struct ocs_hcu_dma_desc *dma_list_head;
	/* OCS DMA linked list tail. */
	struct ocs_hcu_dma_desc *dma_list_tail;
	/* The size of the allocated buffer to contain the DMA linked list. */
	size_t dma_list_size;

	struct ocs_hcu_dev *hcu_dev;
	struct ocs_hcu_idata_desc idata;
	u32 algo;
	u32 dig_sz;

	/* Total data in the SG list at any time. */
	unsigned int sg_data_total;
	/* Offset into the data of an individual SG node. */
	unsigned int sg_data_offset;
	/* The amount of data in bytes in each buffer. */
	unsigned int buf_cnt;
	/* The statig length of each buffer. */
	unsigned int buf_len;

	u8 buffer[KMB_HCU_BUF_SIZE] __aligned(sizeof(u64));
};

/* Driver data. */
struct ocs_hcu_drv {
	struct list_head dev_list;
	spinlock_t lock;
};

static struct ocs_hcu_drv ocs_hcu = {
	.dev_list = LIST_HEAD_INIT(ocs_hcu.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(ocs_hcu.lock),
};

static inline unsigned int kmb_get_total_data(struct ocs_hcu_rctx *rctx)
{
	return rctx->sg_data_total + rctx->buf_cnt;
}

static void kmb_ocs_hcu_copy_sg(struct ocs_hcu_rctx *rctx,
				void *buf, size_t count)
{
	scatterwalk_map_and_copy(buf, rctx->sg, rctx->sg_data_offset,
				 count, 0);

	rctx->sg_data_offset += count;
	rctx->sg_data_total -= count;
	rctx->buf_cnt += count;


	if (rctx->sg_data_offset == rctx->sg->length) {
		rctx->sg = sg_next(rctx->sg);
		rctx->sg_data_offset = 0;
		if (!rctx->sg)
			rctx->sg_data_total = 0;
	}
}

static int kmb_ocs_hcu_append_sg(struct ocs_hcu_rctx *rctx, u32 max_sz)
{
	unsigned int count;

	if (unlikely(rctx->buf_cnt > rctx->buf_len) ||
	    unlikely(rctx->buf_len < max_sz)) {
		dev_err(rctx->hcu_dev->dev,
			"No space left in buffer.\n");
		return -EINVAL;
	}

	/* Only ever get a DMA block size in the buffer. */
	while ((rctx->buf_cnt < max_sz) && rctx->sg_data_total) {
		if (!rctx->sg)
			return 0;
		/* Determine the maximum data available to copy from the node.
		 * Minimum of the length left in the sg node, or the total data
		 * in the request.
		 * Then ensure that the buffer has the space available, so
		 * determine the minimum of the previous minimum with the
		 * remaining buffer size.
		 */
		count = min(rctx->sg->length - rctx->sg_data_offset,
			    rctx->sg_data_total);
		count = min(count, max_sz - rctx->buf_cnt);

		if (count <= 0) {
			if ((rctx->sg->length == 0) && !sg_is_last(rctx->sg)) {
				rctx->sg = sg_next(rctx->sg);
				continue;
			} else {
				break;
			}
		}
		kmb_ocs_hcu_copy_sg(rctx, rctx->buffer +
				    rctx->buf_cnt, count);
	}

	return 0;
}

static struct ocs_hcu_dev *kmb_ocs_hcu_find_dev(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ocs_hcu_ctx *tctx = crypto_ahash_ctx(tfm);
	struct ocs_hcu_dev *hcu_dev = NULL, *tmp;

	spin_lock_bh(&ocs_hcu.lock);
	if (!tctx->hcu_dev) {
		list_for_each_entry(tmp, &ocs_hcu.dev_list, list) {
			hcu_dev = tmp;
			break;
		}
		tctx->hcu_dev = hcu_dev;
	} else {
		hcu_dev = tctx->hcu_dev;
	}

	spin_unlock_bh(&ocs_hcu.lock);

	return hcu_dev;
}

static void kmb_ocs_free_dma_list(struct ocs_hcu_rctx *rctx)
{
	struct ocs_hcu_dev *hcu_dev = rctx->hcu_dev;
	struct device *dev = hcu_dev->dev;

	if (!rctx->dma_list_head || !rctx->dma_list_tail)
		return;

	if (rctx->sg_dma_nents > 0)
		dma_unmap_sg(dev, hcu_dev->req->src, rctx->sg_dma_nents,
			     DMA_TO_DEVICE);

	dma_free_coherent(dev, rctx->dma_list_size, rctx->dma_list_head,
			  rctx->ll_dma_addr);

	rctx->dma_list_head = 0;
	rctx->dma_list_tail = 0;
	rctx->ll_dma_addr = 0;
}

static int kmb_ocs_add_dma_tail(struct ocs_hcu_rctx *rctx,
				dma_addr_t addr, size_t len)
{
	if (addr & KMB_OCS_HCU_ALIGN_MASK || addr > OCS_HCU_DMA_MAX_ADDR_MASK)
		return -EINVAL;

	if (!len)
		return 0;

	rctx->dma_list_tail->src_adr = (u32)addr;
	rctx->dma_list_tail->src_len = (u32)len;
	rctx->dma_list_tail->ll_flags = 0;
	rctx->dma_list_tail->nxt_desc = rctx->ll_dma_addr +
					(virt_to_phys(rctx->dma_list_tail) -
					virt_to_phys(rctx->dma_list_head)) +
					sizeof(*rctx->dma_list_tail);

	rctx->dma_list_tail++;

	return 0;
}

static void kmb_ocs_terminate_dma_tail(struct ocs_hcu_rctx *rctx, int n_nodes)
{
	struct ocs_hcu_dma_desc *final = rctx->dma_list_head +
					 (n_nodes - 1);

	final->nxt_desc = 0;
	final->ll_flags = OCS_LL_DMA_FLAG_TERMINATE;
}

/* This function will always have a total DMA size aligned to 64B. */
static int kmb_ocs_init_dma_list(struct ocs_hcu_dev *hcu_dev)
{
	struct ahash_request *req = hcu_dev->req;
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);
	struct device *dev = hcu_dev->dev;
	struct scatterlist *sg = req->src;
	unsigned int nents = sg_nents(sg);
	unsigned int total = kmb_get_total_data(rctx);
	unsigned int remainder = 0;
	long sgt = (int)rctx->sg_data_total;
	size_t count;
	int n_nodes;
	int rc;
	int i;

	if (!total)
		return 0;

	rctx->sg_dma_nents = 0;

	/* If this is not a final DMA (terminated DMA), the data passed to the
	 * HCU must be algigned to the block size.
	 */
	if (!(rctx->flags & REQ_FLAGS_FINAL_DATA))
		remainder = (total) % rctx->blk_sz;

	/* Determine the number of scatter gather list nodes to map. */
	for_each_sg(req->src, sg, nents, i) {
		if (remainder >= sgt)
			break;

		rctx->sg_dma_nents++;

		if (sg->length > (sgt - remainder))
			break;

		sgt -= sg->length;
	}

	/* If there is data in the buffer, increase the number of
	 * individual nodes in the linked list to be allocated.
	 */
	n_nodes = rctx->sg_dma_nents;

	/* Size of the total number of descriptors to allocate. */
	rctx->dma_list_size = sizeof(*rctx->dma_list_head) * n_nodes;

	rctx->dma_list_head = dma_alloc_coherent(dev, rctx->dma_list_size,
						 &rctx->ll_dma_addr,
						 GFP_KERNEL);
	if (!rctx->dma_list_head)
		return -ENOMEM;

	rctx->dma_list_tail = rctx->dma_list_head;

	rctx->sg_dma_nents = dma_map_sg(hcu_dev->dev, req->src,
					rctx->sg_dma_nents, DMA_TO_DEVICE);

	if (!rctx->sg_dma_nents) {
		rc = -ENOMEM;
		goto cleanup;
	}

	/* Add the SG Nodes to the DMA linked list. */
	for_each_sg(req->src, rctx->sg, rctx->sg_dma_nents, i) {
		count = min(rctx->sg_data_total - remainder,
			    rctx->sg->length - rctx->sg_data_offset);

		/* Do not create a zero length DMA descriptor. Check in case of
		 * zero length SG node.
		 */
		if (count > 0) {
			rc = kmb_ocs_add_dma_tail(rctx, rctx->sg->dma_address,
						  count);
			if (rc)
				goto cleanup;

			rctx->sg_data_offset += count;
			rctx->sg_data_total -= count;

			if (rctx->sg_data_offset == rctx->sg->length)
				rctx->sg_data_offset = 0;

			if (rctx->sg_data_total <= remainder)
				break;
		}
	}

	/* Make sure rctx->sg is pointing to the next data. */
	if (!rctx->sg_data_offset)
		rctx->sg = sg_next(rctx->sg);

	kmb_ocs_terminate_dma_tail(rctx, n_nodes);

	return 0;
cleanup:
	dev_err(dev, "Failed to map DMA buffer.\n");
	kmb_ocs_free_dma_list(rctx);

	return rc;
}

static void kmb_ocs_hcu_finish_request(struct ocs_hcu_dev *hcu_dev, int *error)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(hcu_dev->req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);

	/* Gets the intermediate data (including digest). */
	ocs_hcu_finish_req(hcu_dev, rctx->algo, &rctx->idata, error);

	/* If the request is not update, it must be either FINAL or FINUP. */
	if (!(rctx->flags & REQ_UPDATE) || *error) {
		if (*error == 0 && hcu_dev->req->result)
			memcpy(hcu_dev->req->result,
			       rctx->idata.digest, rctx->dig_sz);
		/* Ensure all key data is deleted. */
		memzero_explicit(ctx->opad, ARRAY_SIZE(ctx->opad));
		memzero_explicit(ctx->ipad, ARRAY_SIZE(ctx->ipad));
		memzero_explicit(ctx->key, ARRAY_SIZE(ctx->key));
		ctx->key_len = 0;
		/* Clear buffer of any data. */
		memzero_explicit(rctx->buffer, ARRAY_SIZE(rctx->buffer));
		/* Clear the key in HW as well.
		 * Key size is guaranteed lower than maximum at this point,
		 * as request completed.
		 */
		ocs_hcu_write_key(hcu_dev, ctx->key, HCU_MAX_KEYLEN);
	}

	ocs_hcu_hw_disable(hcu_dev);

	/* Copy residual data for next transfer. */
	*error |= kmb_ocs_hcu_append_sg(rctx, KMB_HCU_BUF_SIZE);

	if (!(rctx->flags & REQ_FLAGS_HASH_PIO))
		kmb_ocs_free_dma_list(rctx);

	/* Clear any flags that are complete at this time. */
	rctx->flags &= ~REQ_FLAGS_HASH_PIO;
	crypto_finalize_hash_request(hcu_dev->engine, hcu_dev->req, *error);
}

static inline int kmb_ocs_set_key(struct ocs_hcu_dev *hcu_dev)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(hcu_dev->req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);
	int rc;

	rc = ocs_hcu_write_key(hcu_dev, ctx->key, HCU_MAX_KEYLEN);

	if (rc)
		return rc;

	rctx->flags |= REQ_FLAGS_HMAC_KEY_SET;

	return 0;
}

static int kmb_ocs_hcu_set_hw(struct ocs_hcu_dev *hcu_dev, int algo)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	bool is_hmac_hw = ((rctx->flags & REQ_FLAGS_HMAC_TYPE_MASK) ==
			   REQ_FLAGS_HMAC_TYPE_HW);
	bool busy = ocs_hcu_wait_busy(hcu_dev);
	int rc;

	if (busy)
		return -EBUSY;

	/* Configure the hardware for the current request. */
	rc = ocs_hcu_hw_cfg(hcu_dev, algo);
	if (rc)
		return rc;

	if (rctx->flags & REQ_FLAGS_INTERMEDIATE_DATA) {
		ocs_hcu_set_intermediate_data(hcu_dev, &rctx->idata, algo);
		rctx->flags &= ~REQ_FLAGS_INTERMEDIATE_DATA;
	}

	if (is_hmac_hw && !(rctx->flags & REQ_HMAC_NO_SET_KEY_MASK))
		rc = kmb_ocs_set_key(hcu_dev);

	return rc;
}

static int kmb_ocs_hcu_hash_pio(struct ocs_hcu_dev *hcu_dev,
				u8 *buf, u32 sz, u32 algo, bool term)
{
	int rc = 0;

	/* Configure the hardware for the current request. */
	rc = kmb_ocs_hcu_set_hw(hcu_dev, algo);

	if (rc)
		return rc;

	if (sz != ocs_hcu_hash_cpu(hcu_dev, buf, sz, algo, term))
		return -EIO;

	if (term)
		ocs_hcu_tx_data_done(hcu_dev);

	return 0;
}

/* This function will finish the request if the transfer is not terminating. */
static int kmb_ocs_hcu_tx_pio(struct ocs_hcu_dev *hcu_dev)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	bool is_final = !!(rctx->flags & REQ_FLAGS_FINAL_DATA);
	unsigned int total;
	u32 cpu_tx_sz;
	int rc = 0;

	/* Configure the hardware for the current request. */
	rc = kmb_ocs_hcu_set_hw(hcu_dev, rctx->algo);

	if (rc)
		return rc;

	ocs_hcu_start_hash(hcu_dev);

	if (is_final && kmb_get_total_data(rctx) < KMB_HCU_BUF_SIZE)
		goto final;

	do {
		total = kmb_get_total_data(rctx);
		cpu_tx_sz = total > KMB_HCU_BUF_SIZE ? KMB_HCU_BUF_SIZE :
						       total - (total %
						       rctx->blk_sz);
		rc = kmb_ocs_hcu_append_sg(rctx, cpu_tx_sz);
		if (rc)
			return rc;

		/* Hash the contents of the buffer, if not a final transfer
		 * ensure the buffer does not have any data left.
		 */
		rctx->buf_cnt -= ocs_hcu_hash_cpu(hcu_dev, rctx->buffer,
						  rctx->buf_cnt, rctx->algo,
						  false);
		if (rctx->buf_cnt != 0)
			return -EIO;
	} while (kmb_get_total_data(rctx) / rctx->blk_sz);

	if (!is_final) {
		rctx->flags |= REQ_FLAGS_INTERMEDIATE_DATA;
		if (ocs_hcu_wait_busy(hcu_dev))
			return -EBUSY;
		/* No interrupt in this case, indicate it is done. */
		hcu_dev->flags |= HCU_FLAGS_HCU_DONE;
		kmb_ocs_hcu_finish_request(hcu_dev, &rc);
		return 0;
	}

final:
	rc = kmb_ocs_hcu_append_sg(rctx, KMB_HCU_BUF_SIZE);
	if (rc)
		return rc;
	rctx->buf_cnt -= ocs_hcu_hash_cpu(hcu_dev, rctx->buffer, rctx->buf_cnt,
					  rctx->algo, true);
	if (rctx->buf_cnt != 0) {
		dev_err(hcu_dev->dev, "%sFAILED final PIO hash\n", __func__);
		return -EIO;
	}
	/* Completing transfer. */
	ocs_hcu_tx_data_done(hcu_dev);

	return 0;
}

static int kmb_ocs_hcu_tx_dma(struct ocs_hcu_dev *hcu_dev)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	bool term = !!(rctx->flags & REQ_FLAGS_FINAL_DATA);
	int rc;

	if (!rctx->ll_dma_addr)
		return 0;

	/* Configure the hardware for the current request. */
	rc = kmb_ocs_hcu_set_hw(hcu_dev, rctx->algo);

	if (rc)
		return rc;

	/* Start the DMA engine with the descriptor address stored. */
	ocs_hcu_ll_dma_start(hcu_dev, rctx->ll_dma_addr, term);
	if (!term)
		rctx->flags |= REQ_FLAGS_INTERMEDIATE_DATA;

	return 0;
}

static int kmb_ocs_hcu_tx(struct ocs_hcu_dev *hcu_dev)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	int rc = 0;

	/* Hash the contents of the buffer rctx buffer, otherwise handle the
	 * DMA linked list if it has been created.
	 */
	if (rctx->flags & REQ_FLAGS_HASH_PIO)
		rc = kmb_ocs_hcu_tx_pio(hcu_dev);
	else
		rc = kmb_ocs_hcu_tx_dma(hcu_dev);

	return rc;
}

static int kmb_ocs_hcu_handle_queue(struct ahash_request *req)
{
	struct ocs_hcu_dev *hcu_dev = kmb_ocs_hcu_find_dev(req);

	if (!hcu_dev)
		return -ENOENT;

	return crypto_transfer_hash_request_to_engine(hcu_dev->engine,
						      req);
}

/* If the data size is below the KMB_HCU_BUF_SIZE threshold, setup PIO hashing.
 * Otherwise, hash using the DMA engine.
 */
static int kmb_ocs_hcu_prepare_request(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);
	struct ocs_hcu_dev *hcu_dev = kmb_ocs_hcu_find_dev(req);
	unsigned int total = kmb_get_total_data(rctx);

	if (!hcu_dev)
		return -ENOENT;

	hcu_dev->req = req;

	/* Prepare data for a PIO hash.
	 * This is for the HMAC 0 size corner case where IPAD and OPAD must
	 * be hashed.
	 */
	if (rctx->buf_cnt || total <= KMB_HCU_BUF_SIZE)
		rctx->flags |= REQ_FLAGS_HASH_PIO;
	else if (total)
		return kmb_ocs_init_dma_list(hcu_dev);

	return 0;
}

static int kmb_ocs_hcu_do_final(struct ocs_hcu_dev *hcu_dev)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	int rc;
	bool is_hmac_sw = ((rctx->flags & REQ_FLAGS_HMAC_TYPE_MASK) ==
			   REQ_FLAGS_HMAC_TYPE_SW);

	if (is_hmac_sw)
		rctx->flags |= REQ_FLAGS_FINAL_DATA;

	rc = kmb_ocs_hcu_set_hw(hcu_dev, rctx->algo);
	if (rc)
		return rc;

	ocs_hcu_tx_data_done(hcu_dev);

	return 0;
}

static int kmb_ocs_hcu_do_data(struct ocs_hcu_dev *hcu_dev)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(hcu_dev->req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);
	bool is_hmac_sw = ((rctx->flags & REQ_FLAGS_HMAC_TYPE_MASK) ==
			   REQ_FLAGS_HMAC_TYPE_SW);
	int rc = 0;
	int i;

	/* Compute the ipad for HMAC if the key is valid and the first hash. */
	if (is_hmac_sw && (ctx->key_len <= rctx->blk_sz) &&
	    !(rctx->flags & REQ_FLAGS_INTERMEDIATE_DATA)) {
		if (rctx->flags & REQ_FINAL)
			rctx->flags |= REQ_FLAGS_FINAL_DATA;

		/* Prepare IPAD/OPAD for HMAC. Only done at start and end.
		 * HMAC(k,m) = H(k ^ opad || H(k ^ ipad || m))
		 * ipad will be first block of HMAC DMA.
		 * opad will be calculated in the final request.
		 * Only needed if not using HW HMAC
		 */
		memset(ctx->ipad, HMAC_IPAD_VALUE, rctx->blk_sz);
		for (i = 0; i < ctx->key_len; i++)
			ctx->ipad[i] ^= ctx->key[i];

		rc = kmb_ocs_hcu_hash_pio(hcu_dev, ctx->ipad, rctx->blk_sz,
					    rctx->algo, false);

		if (rc)
			return rc;

		rc = ocs_hcu_get_intermediate_data(hcu_dev, &rctx->idata,
						   rctx->algo);
		if (rc)
			return rc;

		rctx->flags |= REQ_FLAGS_INTERMEDIATE_DATA;
	}

	/* If the REQ_FLAGS_FINAL_DATA flag is set the transform is
	 * terminated or there is data to perform a final hash.
	 * Otherwise terminate the transform to obtain the final hash.
	 */
	if (rctx->flags & (REQ_UPDATE | REQ_FLAGS_FINAL_DATA))
		rc = kmb_ocs_hcu_tx(hcu_dev);
	else if (rctx->flags & REQ_FINAL)
		rc = kmb_ocs_hcu_do_final(hcu_dev);

	return rc;
}

static int kmb_ocs_hcu_do_one_request(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct ocs_hcu_dev *hcu_dev = kmb_ocs_hcu_find_dev(req);
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);

	if (!hcu_dev)
		return -ENOENT;

	hcu_dev->req = req;

	/* Initialize the hardware. */
	ocs_hcu_hw_init(hcu_dev);

	if (rctx->flags & REQ_FLAGS_HASH_KEY)
		return kmb_ocs_hcu_hash_pio(hcu_dev, ctx->key,
					    ctx->key_len,
					    (rctx->algo &
					    ~OCS_HCU_ALGO_HMAC_MASK), true);

	return kmb_ocs_hcu_do_data(hcu_dev);
}

static int kmb_ocs_hcu_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);
	struct ocs_hcu_dev *hcu_dev = kmb_ocs_hcu_find_dev(req);

	if (!hcu_dev)
		return -ENOENT;

	rctx->flags = ctx->flags;
	rctx->hcu_dev = hcu_dev;
	rctx->dig_sz = crypto_ahash_digestsize(tfm);

	switch (rctx->dig_sz) {
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224
	case SHA224_DIGEST_SIZE:
		rctx->flags |= REQ_FLAGS_SHA_224;
		rctx->blk_sz = SHA224_BLOCK_SIZE;
		rctx->algo = OCS_HCU_ALGO_SHA224;
		break;
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224 */
	case SHA256_DIGEST_SIZE:
		rctx->blk_sz = SHA256_BLOCK_SIZE;
		if (rctx->flags & REQ_FLAGS_SM3) {
			rctx->algo = OCS_HCU_ALGO_SM3;
		} else {
			rctx->flags |= REQ_FLAGS_SHA_256;
			rctx->algo = OCS_HCU_ALGO_SHA256;
		}
		break;
	case SHA384_DIGEST_SIZE:
		rctx->flags |= REQ_FLAGS_SHA_384;
		rctx->blk_sz = SHA384_BLOCK_SIZE;
		rctx->algo = OCS_HCU_ALGO_SHA384;
		break;
	case SHA512_DIGEST_SIZE:
		rctx->flags |= REQ_FLAGS_SHA_512;
		rctx->blk_sz = SHA512_BLOCK_SIZE;
		rctx->algo = OCS_HCU_ALGO_SHA512;
		break;
	default:
		return -EINVAL;
	}

	rctx->buf_len = KMB_HCU_BUF_SIZE;
	rctx->ll_dma_addr = 0;
	rctx->dma_list_tail = 0;
	rctx->dma_list_head = 0;
	rctx->dma_list_size = 0;
	/* Clear the total transfer size. */
	rctx->sg_data_total = 0;
	rctx->sg_data_offset = 0;
	rctx->buf_cnt = 0;
	rctx->sg = NULL;

	if (!(rctx->flags & REQ_FLAGS_HMAC)) {
		memzero_explicit(ctx->key, KMB_OCS_HCU_MAX_KEYLEN);
		ctx->key_len = 0;
	} else if (ctx->key_len > rctx->blk_sz) {
		/* Set the hash key flag, copied in init function. */
		rctx->flags |= REQ_FLAGS_HASH_KEY;
	}

	/* Clear the intermediate data. */
	memzero_explicit((void *)&rctx->idata, sizeof(rctx->idata));

	return 0;
}

static int kmb_ocs_hcu_update(struct ahash_request *req)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);

	if (!req->nbytes)
		return 0;

	rctx->sg_data_offset = 0;

	/* Check for overflow */
	if (check_add_overflow(rctx->sg_data_total, req->nbytes,
	    &rctx->sg_data_total))
		return -EINVAL;

	rctx->sg = req->src;
	rctx->flags &= ~REQ_CMD_MASK;
	rctx->flags |= REQ_UPDATE;

	if (rctx->sg_data_total <= (KMB_HCU_BUF_SIZE - rctx->buf_cnt))
		return kmb_ocs_hcu_append_sg(rctx, KMB_HCU_BUF_SIZE);

	return kmb_ocs_hcu_handle_queue(req);
}

static int kmb_ocs_hcu_final(struct ahash_request *req)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);

	rctx->sg = req->src;
	rctx->flags &= ~REQ_CMD_MASK;
	rctx->flags |= REQ_FINAL;

	/* If no intermediate hashes completed, can perform HMAC in one. */
	if ((rctx->flags & REQ_FLAGS_HMAC) &&
	    !(rctx->flags & REQ_FLAGS_INTERMEDIATE_DATA) &&
	    (rctx->flags & REQ_FLAGS_HASH_KEY ||
	    ctx->key_len < HCU_MAX_KEYLEN) && kmb_get_total_data(rctx)) {
		rctx->algo |= OCS_HCU_ALGO_HMAC_MASK;
		rctx->flags |= REQ_FLAGS_HMAC_HW;
	}

	/* If there is any data left OR we are in an intermediate
	 * HMAC transfer, there is a final hash to perform.
	 */
	if (kmb_get_total_data(rctx))
		rctx->flags |= REQ_FLAGS_FINAL_DATA;

	return kmb_ocs_hcu_handle_queue(req);
}

static int kmb_ocs_hcu_finup(struct ahash_request *req)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);

	/* Check for overflow */
	if (check_add_overflow(rctx->sg_data_total, req->nbytes,
	    &rctx->sg_data_total))
		return -EINVAL;

	return kmb_ocs_hcu_final(req);
}

static int kmb_ocs_hcu_digest(struct ahash_request *req)
{
	int rc = 0;

	rc = kmb_ocs_hcu_init(req);
	if (rc)
		return rc;

	rc = kmb_ocs_hcu_finup(req);

	return rc;
}

static int kmb_ocs_hcu_export(struct ahash_request *req, void *out)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);

	/* Intermediate data is always stored and applied per request. */
	memcpy(out, rctx, sizeof(*rctx));

	return 0;
}

static int kmb_ocs_hcu_import(struct ahash_request *req, const void *in)
{
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(req);

	/* Intermediate data is always stored and applied per request. */
	memcpy(rctx, in, sizeof(*rctx));

	return 0;
}

static int kmb_ocs_hcu_setkey(struct crypto_ahash *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);

	if (keylen <= KMB_OCS_HCU_MAX_KEYLEN) {
		memzero_explicit(ctx->key, KMB_OCS_HCU_MAX_KEYLEN);
		memcpy(ctx->key, key, keylen);
		ctx->key_len = keylen;
	} else {
		return -ENOMEM;
	}

	return 0;
}

static int kmb_ocs_hcu_sha_cra_init(struct crypto_tfm *tfm)
{
	struct ocs_hcu_ctx *ctx = crypto_tfm_ctx(tfm);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct ocs_hcu_rctx));

	ctx->engine_ctx.op.do_one_request = kmb_ocs_hcu_do_one_request;
	ctx->engine_ctx.op.prepare_request = kmb_ocs_hcu_prepare_request;

	ctx->flags = 0;

	return 0;
}

static int kmb_ocs_hcu_sm3_cra_init(struct crypto_tfm *tfm)
{
	struct ocs_hcu_ctx *ctx = crypto_tfm_ctx(tfm);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct ocs_hcu_rctx));

	ctx->engine_ctx.op.do_one_request = kmb_ocs_hcu_do_one_request;
	ctx->engine_ctx.op.prepare_request = kmb_ocs_hcu_prepare_request;

	ctx->flags = REQ_FLAGS_SM3;

	return 0;
}

static int kmb_ocs_hcu_hmac_sm3_cra_init(struct crypto_tfm *tfm)
{
	struct ocs_hcu_ctx *ctx = crypto_tfm_ctx(tfm);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct ocs_hcu_rctx));

	ctx->engine_ctx.op.do_one_request = kmb_ocs_hcu_do_one_request;
	ctx->engine_ctx.op.prepare_request = kmb_ocs_hcu_prepare_request;

	ctx->flags = REQ_FLAGS_SM3 | REQ_FLAGS_HMAC;

	return 0;
}

static int kmb_ocs_hcu_hmac_cra_init(struct crypto_tfm *tfm)
{
	struct ocs_hcu_ctx *ctx = crypto_tfm_ctx(tfm);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct ocs_hcu_rctx));

	ctx->engine_ctx.op.do_one_request = kmb_ocs_hcu_do_one_request;
	ctx->engine_ctx.op.prepare_request = kmb_ocs_hcu_prepare_request;

	ctx->flags = REQ_FLAGS_HMAC;

	return 0;
}

static struct ahash_alg ocs_hcu_algs[] = {
#ifdef CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.halg = {
		.digestsize	= SHA224_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "sha224",
			.cra_driver_name	= "sha224-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA224_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_sha_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.setkey		= kmb_ocs_hcu_setkey,
	.halg = {
		.digestsize	= SHA224_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "hmac(sha224)",
			.cra_driver_name	= "hmac-sha224-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA224_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_hmac_cra_init,
		}
	}
},
#endif /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_HCU_HMAC_SHA224 */
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.halg = {
		.digestsize	= SHA256_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "sha256",
			.cra_driver_name	= "sha256-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA256_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_sha_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.setkey		= kmb_ocs_hcu_setkey,
	.halg = {
		.digestsize	= SHA256_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "hmac(sha256)",
			.cra_driver_name	= "hmac-sha256-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA256_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_hmac_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.halg = {
		.digestsize	= SM3_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "sm3",
			.cra_driver_name	= "sm3-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SM3_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_sm3_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.setkey		= kmb_ocs_hcu_setkey,
	.halg = {
		.digestsize	= SM3_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "hmac(sm3)",
			.cra_driver_name	= "hmac-sm3-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SM3_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_hmac_sm3_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.halg = {
		.digestsize	= SHA384_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "sha384",
			.cra_driver_name	= "sha384-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA384_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_sha_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.setkey		= kmb_ocs_hcu_setkey,
	.halg = {
		.digestsize	= SHA384_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "hmac(sha384)",
			.cra_driver_name	= "hmac-sha384-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA384_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_hmac_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.halg = {
		.digestsize	= SHA512_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "sha512",
			.cra_driver_name	= "sha512-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA512_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_sha_cra_init,
		}
	}
},
{
	.init		= kmb_ocs_hcu_init,
	.update		= kmb_ocs_hcu_update,
	.final		= kmb_ocs_hcu_final,
	.finup		= kmb_ocs_hcu_finup,
	.digest		= kmb_ocs_hcu_digest,
	.export		= kmb_ocs_hcu_export,
	.import		= kmb_ocs_hcu_import,
	.setkey		= kmb_ocs_hcu_setkey,
	.halg = {
		.digestsize	= SHA512_DIGEST_SIZE,
		.statesize	= sizeof(struct ocs_hcu_rctx),
		.base	= {
			.cra_name		= "hmac(sha512)",
			.cra_driver_name	= "hmac-sha512-keembay-ocs",
			.cra_priority		= 255,
			.cra_flags		= CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA512_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct ocs_hcu_ctx),
			.cra_alignmask		= KMB_OCS_HCU_ALIGN_MASK,
			.cra_module		= THIS_MODULE,
			.cra_init		= kmb_ocs_hcu_hmac_cra_init,
		}
	}
}
};

static irqreturn_t kmb_ocs_hcu_irq_thread(int irq, void *dev_id)
{
	struct ocs_hcu_dev *hcu_dev = (struct ocs_hcu_dev *)dev_id;
	struct ocs_hcu_rctx *rctx = ahash_request_ctx(hcu_dev->req);
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(hcu_dev->req);
	struct ocs_hcu_ctx *ctx = crypto_ahash_ctx(tfm);
	bool is_hmac_sw = ((rctx->flags & REQ_FLAGS_HMAC_TYPE_MASK) ==
			   REQ_FLAGS_HMAC_TYPE_SW);
	int rc = 0;
	int i;

	if (hcu_dev->flags & HCU_FLAGS_HCU_ERROR_MASK) {
		rc = -EIO;
		goto finish;
	}

	/* Check if this is the final HMAC data after a streamed HMAC
	 * computation. This mode doesn't use the HW HMAC setting and requires
	 * SW to handle IPAD and OPAD, thus one further hash is required of
	 * H(k ^ opad || digest) where digest obtained here is H(k ^ ipad || m).
	 */
	if (is_hmac_sw && (rctx->flags & REQ_FLAGS_FINAL_DATA)) {
		/* Gets the intermediate data (including digest). */
		ocs_hcu_finish_req(hcu_dev, rctx->algo, &rctx->idata, &rc);
		if (rc)
			goto finish;
		/* Prepare OPAD for HMAC.
		 * HMAC(k,m) = H(k ^ opad || H(k ^ ipad || m))
		 * We now have the H(k ^ ipad || m), create final data for hash
		 */
		memset(ctx->opad, HMAC_OPAD_VALUE, rctx->blk_sz);
		for (i = 0; i < ctx->key_len; i++)
			ctx->opad[i] ^= ctx->key[i];

		for (i = 0; (i < rctx->dig_sz); i++)
			ctx->opad[rctx->blk_sz + i] = rctx->idata.digest[i];

		rctx->flags &= ~(REQ_FLAGS_FINAL_DATA | REQ_FLAGS_HASH_PIO);

		rc = kmb_ocs_hcu_hash_pio(hcu_dev, ctx->opad,
					  rctx->blk_sz + rctx->dig_sz,
					  rctx->algo, true);
		if (rc)
			goto finish;

		return IRQ_HANDLED;
	}

	/* Check if the interrupt is due to hashing an oversized key. */
	if (rctx->flags & REQ_FLAGS_HASH_KEY) {
		/* Gets the intermediate data (including digest). */
		ocs_hcu_finish_req(hcu_dev, rctx->algo, &rctx->idata, &rc);
		/* Copy the result into the sw_key. */
		memcpy(ctx->key, rctx->idata.digest, rctx->dig_sz);
		/* Key length will always be greater than digest size. */
		memzero_explicit(ctx->key + rctx->dig_sz,
				 ctx->key_len - rctx->dig_sz);
		ctx->key_len = rctx->dig_sz;
		rctx->flags &= ~REQ_FLAGS_HASH_KEY;

		/* Continue processing the data. */
		rc = kmb_ocs_hcu_do_data(hcu_dev);
		if (rc)
			goto finish;

		return IRQ_HANDLED;
	}

finish:
	kmb_ocs_hcu_finish_request(hcu_dev, &rc);

	return IRQ_HANDLED;
}

static int kmb_ocs_hcu_unregister_algs(struct ocs_hcu_dev *hcu_dev)
{
	int i = 0;
	int rc = 0;
	int ret = 0;

	for (; i < ARRAY_SIZE(ocs_hcu_algs); i++) {
		rc = crypto_unregister_ahash(&ocs_hcu_algs[i]);
		if (rc) {
			ret = rc;
			dev_err(hcu_dev->dev, "Failed to unregister %s.\n",
				ocs_hcu_algs[i].halg.base.cra_name);
		}
	}

	return ret;
}

static int kmb_ocs_hcu_register_algs(struct ocs_hcu_dev *hcu_dev)
{
	int rc = 0;
	int err_rc = 0;
	int i;
	int j;

	for (i = 0; i < ARRAY_SIZE(ocs_hcu_algs); i++) {
		rc = crypto_register_ahash(&ocs_hcu_algs[i]);
		if (rc)
			goto cleanup;
	}

	return 0;
cleanup:
	dev_err(hcu_dev->dev, "Failed to register algo.\n");
	for (j = 0; j < i; j++) {
		err_rc = crypto_unregister_ahash(&ocs_hcu_algs[i]);
		if (err_rc) {
			dev_err(hcu_dev->dev, "Failed to unregister %s.\n",
				ocs_hcu_algs[i].halg.base.cra_name);
		}
	}

	return rc;
}

/* Device tree driver match. */
static const struct of_device_id kmb_ocs_hcu_of_match[] = {
	{
		.compatible = "intel,keembay-ocs-hcu",
	},
	{}
};

static int kmb_ocs_hcu_remove(struct platform_device *pdev)
{
	struct ocs_hcu_dev *hcu_dev;
	int rc;

	hcu_dev = platform_get_drvdata(pdev);
	if (!hcu_dev)
		return -ENODEV;

	rc = kmb_ocs_hcu_unregister_algs(hcu_dev);

	spin_lock(&ocs_hcu.lock);
	list_del(&hcu_dev->list);
	spin_unlock(&ocs_hcu.lock);

	rc |= crypto_engine_exit(hcu_dev->engine);

	return rc;
}

static int kmb_ocs_hcu_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct resource *hcu_mem;
	struct ocs_hcu_dev *hcu_dev;

	hcu_dev = devm_kzalloc(dev, sizeof(*hcu_dev),
				       GFP_KERNEL);
	if (!hcu_dev)
		return -ENOMEM;

	hcu_dev->dev = dev;

	platform_set_drvdata(pdev, hcu_dev);
	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));

	if (rc)
		return rc;

	INIT_LIST_HEAD(&hcu_dev->list);

	/* Get the memory address and remap. */
	hcu_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!hcu_mem) {
		dev_err(dev, "Could not retrieve io mem resource.\n");
		rc = -ENODEV;
		goto list_del;
	}

	hcu_dev->io_base = devm_ioremap_resource(dev, hcu_mem);
	if (IS_ERR(hcu_dev->io_base)) {
		dev_err(dev, "Could not io-remap mem resource.\n");
		rc = PTR_ERR(hcu_dev->io_base);
		goto list_del;
	}

	/* Get and request IRQ. */
	hcu_dev->irq = platform_get_irq(pdev, 0);
	if (hcu_dev->irq < 0) {
		dev_err(dev, "Could not retrieve IRQ.\n");
		rc = hcu_dev->irq;
		goto list_del;
	}

	rc = devm_request_threaded_irq(&pdev->dev, hcu_dev->irq,
				       ocs_hcu_irq_handler,
				       kmb_ocs_hcu_irq_thread,
				       0, "keembay-ocs-hcu", hcu_dev);
	if (rc < 0) {
		dev_err(dev, "Could not request IRQ.\n");
		goto list_del;
	}

	spin_lock(&ocs_hcu.lock);
	list_add_tail(&hcu_dev->list, &ocs_hcu.dev_list);
	spin_unlock(&ocs_hcu.lock);

	/* Initialize crypto engine */
	hcu_dev->engine = crypto_engine_alloc_init(dev, 1);
	if (!hcu_dev->engine)
		goto list_del;

	rc = crypto_engine_start(hcu_dev->engine);
	if (rc) {
		dev_err(dev, "Could not start engine.\n");
		goto cleanup;
	}

	/* Security infrastructure guarantees OCS clock is enabled. */

	rc = kmb_ocs_hcu_register_algs(hcu_dev);
	if (rc) {
		dev_err(dev, "Could not register algorithms.\n");
		goto cleanup;
	}

	return 0;
cleanup:
	crypto_engine_exit(hcu_dev->engine);
list_del:
	spin_lock(&ocs_hcu.lock);
	list_del(&hcu_dev->list);
	spin_unlock(&ocs_hcu.lock);

	return rc;

}

/* The OCS driver is a platform device. */
static struct platform_driver kmb_ocs_hcu_driver = {
	.probe = kmb_ocs_hcu_probe,
	.remove = kmb_ocs_hcu_remove,
	.driver = {
			.name = DRV_NAME,
			.of_match_table = kmb_ocs_hcu_of_match,
		},
};

module_platform_driver(kmb_ocs_hcu_driver);

MODULE_LICENSE("GPL v2");
