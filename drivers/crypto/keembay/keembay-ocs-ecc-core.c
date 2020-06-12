// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay OCS ECC Crypto Driver.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/fips.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <crypto/ecdh.h>
#include <crypto/engine.h>
#include <crypto/rng.h>
#include <crypto/internal/kpp.h>
#include <crypto/kpp.h>

#include "../crypto/ecc.h"
#include "ocs-ecc-curve-defs.h"

#define DRV_NAME	"keembay-ocs-ecc-driver"

#define KMB_OCS_ECC_PRIORITY	350

#define HW_OFFS_OCS_ECC_COMMAND	0x00000000
#define HW_OFFS_OCS_ECC_STATUS	0x00000004
#define HW_OFFS_OCS_ECC_DATA_IN	0x00000080
#define HW_OFFS_OCS_ECC_CX_DATA_OUT	0x00000100
#define HW_OFFS_OCS_ECC_CY_DATA_OUT	0x00000180
#define HW_OFFS_OCS_ECC_ISR	0x00000400
#define HW_OFFS_OCS_ECC_IER	0x00000404

#define HW_OCS_ECC_COMMAND_START_VAL	BIT(0)
#define HW_OCS_ECC_ISR_INT_STATUS_DONE	BIT(0)
#define HW_OCS_ECC_COMMAND_INS_BP	BIT(0)
#define HW_OCS_ECC_COMMAND_SIZE_MASK	BIT(3)

/* ECC Instruction : for ECC_COMMAND */
#define ECC_INS_WRITE_AX	0x01
#define ECC_INS_WRITE_AY	0x02
#define ECC_INS_WRITE_BX_D	0x03
#define ECC_INS_WRITE_BY_L	0x04
#define ECC_INS_WRITE_P	0x05
#define ECC_INS_WRITE_A	0x06
#define ECC_INS_CALC_D_IDX_A	0x08
#define ECC_INS_CALC_A_POW_B_MODP	0xB
#define ECC_INS_CALC_A_MUL_B_MODP	0xC
#define ECC_INS_CALC_A_ADD_B_MODP	0xD

#define ECC_COMMAND_INS(ins)	((ins) << HW_OCS_ECC_COMMAND_INS_BP)

#define data_size_u64_to_u8(n)	(n << ECC_DIGITS_TO_BYTES_SHIFT)

#define ECC_ENABLE_INTR	1

#define POLL_USEC	100
#define TIMEOUT_USEC	100000

#define KMB_ECC_MAX_DIGITS	(768 / 64)
#define DATA_SIZE_U8_384BITS	(384 / 8)
#define DATA_SIZE_U32_384BITS	(384 / 32)
#define DATA_SIZE_U64_384BITS	(384 / 64)
#define ECC_CURVE_NIST_P384_DIGITS	DATA_SIZE_U64_384BITS
#define POW_CUBE	3

/* State machine used in shared secret computation. */
enum state {
	SS_NO_OP = 0,
	SS_Y_SQUARE,
	SS_X_CUBE,
	SS_A_MUL_X,
	SS_A_MUL_X_PLUS_B,
	SS_CHECK_PUB_KEY,
	SS_ECC_POINT_MUL,
};

/**
 * ECC device context
 * @list: List of device contexts
 * @dev: OCS ECC device
 * @irq: IRQ number
 * @base_reg: IO base address of OCS ECC
 * @engine: Crypto engine for the device
 * @req: Incoming kpp request
 */
struct ocs_ecc_dev {
	struct list_head list;
	struct device *dev;
	int irq;
	void __iomem *base_reg;
	struct crypto_engine *engine;
	struct kpp_request *req;
};

/*
 * Transformation context
 * @engine_ctx: Crypto engine ctx
 * @ecc_dev: ecc device context
 * @curve_id: elliptic curve id
 * @result_ptr: Result ptr for storing point multiplication.
 * @req: kpp request structure.
 * @pk: Public key point ptr stored off for shared secret computation.
 * @ecc_next_state: Variable to keep track of state in shared secret
 *		    computation.
 * @gfp: Store request flag.
 */
struct ocs_ecc_ctx {
	struct crypto_engine_ctx engine_ctx;
	struct ocs_ecc_dev *ecc_dev;
	unsigned int curve_id;
	unsigned int ndigits;
	u64 private_key[KMB_ECC_MAX_DIGITS];
	struct ecc_point *result_ptr;
	struct ecc_point *pk;
	enum state ecc_next_state;
	gfp_t gfp;
};

/* Driver data. */
struct ocs_ecc_drv {
	struct list_head dev_list;
	spinlock_t lock;
};

static struct ocs_ecc_drv ocs_ecc = {
	.dev_list = LIST_HEAD_INIT(ocs_ecc.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(ocs_ecc.lock),
};

/* Device tree driver match. */
static const struct of_device_id kmb_ocs_ecc_of_match[] = {
	{
		.compatible = "intel,keembay-ocs-ecc",
	},
	{}
};


static inline void *kmb_ocs_kpp_ctx(struct kpp_request *req)
{
	return kpp_tfm_ctx(crypto_kpp_reqtfm(req));
}

static inline const struct ecc_curve *ecc_get_curve(unsigned int curve_id)
{
	switch (curve_id) {
	case ECC_CURVE_NIST_P256:
		return &nist_p256;
	case ECC_CURVE_NIST_P384:
		return &nist_p384;
	default:
		return NULL;
	}
}

/* Returns true if point is the point at infinity, false otherwise. */
static bool ecc_point_is_zero(const struct ecc_point *point)
{
	return (vli_is_zero(point->x, point->ndigits) &&
		vli_is_zero(point->y, point->ndigits));
}

static struct ecc_point *ecc_alloc_point(unsigned int ndigits, gfp_t gfp)
{
	struct ecc_point *p;

	p = kzalloc(sizeof(*p), gfp);

	if (!p)
		return NULL;

	p->x = kcalloc(ndigits, sizeof(u64), gfp);
	if (!p->x)
		goto err_alloc_x;

	p->y = kcalloc(ndigits, sizeof(u64), gfp);
	if (!p->y)
		goto err_alloc_y;

	p->ndigits = ndigits;

	return p;

err_alloc_y:
	memzero_explicit(p->x, ndigits * sizeof(u64));
	kzfree(p->x);
err_alloc_x:
	kzfree(p);
	return NULL;
}

static void ecc_free_point(struct ecc_point *p)
{
	if (!p)
		return;

	if (p->x) {
		memzero_explicit(p->x, p->ndigits * sizeof(u64));
		kzfree(p->x);
	}

	if (p->y) {
		memzero_explicit(p->y, p->ndigits * sizeof(u64));
		kzfree(p->y);
	}

	kzfree(p);
}

static inline void ecc_swap_digits(const u64 *in, u64 *out,
				   unsigned int ndigits)
{
	unsigned int i;

	const __be64 *src = (__force __be64 *)in;

	for (i = 0; i < ndigits; i++)
		out[i] = be64_to_cpu(src[ndigits - 1 - i]);
}

/*
 * Wait for ECC idle i.e when an operation (other than write operations)
 * is done.
 */
static inline int ecc_wait_idle(struct ocs_ecc_dev *dev)
{
	u32 value;

	return readl_poll_timeout((dev->base_reg + HW_OFFS_OCS_ECC_STATUS),
				  value, !(value &
					   HW_OCS_ECC_ISR_INT_STATUS_DONE),
				  POLL_USEC, TIMEOUT_USEC);
}

/* Direct write of u32 buffer to ECC engine with associated instruction. */
static inline void write_ecc_cmd_and_data(struct ocs_ecc_dev *dev,
					  const u32 write_cmd_inst,
					  const u32 *const data32,
					  const long data_u8_cnt,
					  const u32 en_384)
{
	/*
	 * Write to ECC Command for data write instructions, enable 384 bit
	 * operation if required and keep the start flag set for the operation
	 * in progress.
	 */
	iowrite32(en_384 | write_cmd_inst, (dev->base_reg +
					    HW_OFFS_OCS_ECC_COMMAND));

	/* MMIO Write src uint32 to dst. */
	memcpy_toio((dev->base_reg + HW_OFFS_OCS_ECC_DATA_IN), data32,
			data_u8_cnt);
}

static inline void ecc_op_trigger(struct ocs_ecc_dev *dev, u32 en_384,
				  u32 write_cmd_inst)
{
	iowrite32(ECC_ENABLE_INTR, (dev->base_reg + HW_OFFS_OCS_ECC_IER));
	iowrite32(en_384 | ECC_COMMAND_INS(write_cmd_inst),
		  (dev->base_reg + HW_OFFS_OCS_ECC_COMMAND));
}

/* Read the CX and CY data output buffer. */
static inline void ecc_read_cx_cy_out(struct ocs_ecc_dev *dev,
				      const long data_u8_cnt,
				      u64 *const cx_out,
				      u64 *const cy_out)
{
	/* Read cx output. */
	memcpy_fromio(cx_out, (dev->base_reg + HW_OFFS_OCS_ECC_CX_DATA_OUT),
		      data_u8_cnt);

	/* Read cy output. */
	if (cy_out)
		memcpy_fromio(cy_out,
			      (dev->base_reg + HW_OFFS_OCS_ECC_CY_DATA_OUT),
			      data_u8_cnt);
}

static struct ocs_ecc_dev *kmb_ocs_ecc_find_dev(struct ocs_ecc_ctx *tctx)
{
	struct ocs_ecc_dev *ecc_dev;

	spin_lock_bh(&ocs_ecc.lock);

	if (!tctx->ecc_dev) {
		/* Only a single OCS device available */
		ecc_dev = list_first_entry(&ocs_ecc.dev_list,
					   struct ocs_ecc_dev, list);
		tctx->ecc_dev = ecc_dev;
	} else
		ecc_dev = tctx->ecc_dev;

	spin_unlock_bh(&ocs_ecc.lock);

	return ecc_dev;
}

static int ecc_point_mult(struct ocs_ecc_dev *dd,
			  struct ecc_point *result,
			  const struct ecc_point *point, const u64 *scalar,
			  const struct ecc_curve *curve, unsigned int ndigits)
{
	u32 sca[DATA_SIZE_U32_384BITS]; /* Use the maximum data size. */

	struct ocs_ecc_ctx *ctx = kmb_ocs_kpp_ctx(dd->req);
	int ret = 0;
	u32 size_384_en = (ndigits / DATA_SIZE_U64_384BITS)
			  << HW_OCS_ECC_COMMAND_SIZE_MASK;
	long nbytes = data_size_u64_to_u8(ndigits);

	/* Storing result ptr in context.*/
	ctx->result_ptr = result;
	ctx->result_ptr->x = result->x;
	ctx->result_ptr->y = result->y;
	ctx->result_ptr->ndigits = result->ndigits;

	/*
	 * Generate random nbytes for Simple and Differential SCA protection.
	 */
	ret = crypto_get_default_rng();
	if (ret)
		return ret;

	ret = crypto_rng_get_bytes(crypto_default_rng, (u8 *)sca, nbytes);
	crypto_put_default_rng();
	if (ret)
		return ret;

	/* Wait engine to be idle before starting new operation. */
	ret = ecc_wait_idle(dd);
	if (ret)
		return ret;

	/* Send ecc_start pulse as well as indicating operation size. */
	iowrite32(size_384_en | HW_OCS_ECC_COMMAND_START_VAL,
		  (dd->base_reg + HW_OFFS_OCS_ECC_COMMAND));

	/* Write ax param (Base point (Gx).*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_AX),
		(u32 *)point->x, nbytes, size_384_en);

	/* Write ay param Base point (Gy).*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_AY),
		(u32 *)point->y, nbytes, size_384_en);

	/*
	 * Write the private key into DATA_IN reg.
	 * Since DATA_IN register is used to write different values during
	 * the computation private Key value is overwritten with
	 * side-channel-resistance value.
	 */
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_BX_D),
		(u32 *)scalar, nbytes, size_384_en);

	/* Write operand by/l.*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_BY_L),
		(u32 *)sca, nbytes, size_384_en);

	/* Write p = curve prime(GF modulus).*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_P),
		(u32 *)curve->p, nbytes, size_384_en);

	/* Write a = curve coefficient.*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_A),
			       (u32 *)curve->a, nbytes, size_384_en);

	/* Give instruction d[A] to ECC engine. */
	ecc_op_trigger(dd, size_384_en, ECC_INS_CALC_D_IDX_A);

	memzero_explicit(sca, DATA_SIZE_U8_384BITS);

	return 0;
}

static int ecc_ab_or_a_plus_b_modp(struct ocs_ecc_dev *dd,
				   struct ecc_point *result,
				   const u64 *scalar_a, const u64 *scalar_b,
				   const struct ecc_curve *curve,
				   unsigned int ndigits,
				   const u32 write_cmd_inst)
{
	struct ocs_ecc_ctx *ctx = kmb_ocs_kpp_ctx(dd->req);
	int ret = 0;
	u32 size_384_en = (ndigits / DATA_SIZE_U64_384BITS)
			  << HW_OCS_ECC_COMMAND_SIZE_MASK;

	long nbytes = data_size_u64_to_u8(ndigits);

	/* Storing result ptr in context.*/
	ctx->result_ptr = result;
	ctx->result_ptr->x = result->x;
	ctx->result_ptr->y = NULL;
	ctx->result_ptr->ndigits = ndigits;

	/* Wait engine to be idle before starting new operation. */
	ret = ecc_wait_idle(dd);

	if (ret)
		return ret;

	/* Send ecc_start pulse as well as indicating operation size. */
	iowrite32(size_384_en | HW_OCS_ECC_COMMAND_START_VAL,
		  (dd->base_reg + HW_OFFS_OCS_ECC_COMMAND));

	/* Write ax param (Base point (Gx).*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_AX),
			       (u32 *)scalar_a, nbytes, size_384_en);

	/* Write ay param Base point (Gy).*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_AY),
			       (u32 *)scalar_b, nbytes, size_384_en);

	/* Write p = curve prime(GF modulus).*/
	write_ecc_cmd_and_data(dd, ECC_COMMAND_INS(ECC_INS_WRITE_P),
			       (u32 *)curve->p, nbytes, size_384_en);

	/* Give instruction A.B or A+B to ECC engine. */
	ecc_op_trigger(dd, size_384_en, write_cmd_inst);

	return 0;
}

/* SP800-56A section 5.6.2.3.4 partial verification: ephemeral keys only */
static int kmb_ocs_ecc_is_pubkey_valid_partial(struct ocs_ecc_dev *dev,
					       const struct ecc_curve *curve,
					       struct ecc_point *pk)
{
	struct ecc_point *yy;

	struct ocs_ecc_ctx *ctx = kmb_ocs_kpp_ctx(dev->req);
	int ret = 0;

	yy = ecc_alloc_point(pk->ndigits, ctx->gfp);
	if (!yy) {
		ret = -ENOMEM;
		goto err_alloc_point;
	}

	if (WARN_ON(pk->ndigits != curve->g.ndigits)) {
		ret = -EINVAL;
		goto free_all;
	}

	/* Check 1: Verify key is not the zero point. */
	if (ecc_point_is_zero(pk)) {
		ret = -EINVAL;
		goto free_all;
	}

	/* Check 2: Verify key is in the range [0, p-1]. */
	if (vli_cmp(curve->p, pk->x, pk->ndigits) != 1) {
		ret = -EINVAL;
		goto free_all;
	}
	if (vli_cmp(curve->p, pk->y, pk->ndigits) != 1) {
		ret = -EINVAL;
		goto free_all;
	}

	/* Check 3: Verify that y^2 == (x^3 + a·x + b) mod p */

	 /* y^2 */
	ctx->ecc_next_state = SS_Y_SQUARE;
	ret = ecc_ab_or_a_plus_b_modp(dev, yy, pk->y, pk->y, curve,
				      pk->ndigits, ECC_INS_CALC_A_MUL_B_MODP);
	if (!ret)
		return ret;

free_all:
	ecc_free_point(yy);
err_alloc_point:
	return ret;
}

static unsigned int kmb_ocs_ecdh_supported_curve(unsigned int curve_id)
{
	switch (curve_id) {
	case ECC_CURVE_NIST_P256: return ECC_CURVE_NIST_P256_DIGITS;
	case ECC_CURVE_NIST_P384: return ECC_CURVE_NIST_P384_DIGITS;
	default: return 0;
	}
}

static int kmb_ecc_is_key_valid(unsigned int curve_id, unsigned int ndigits,
				const u64 *private_key,
				unsigned int private_key_len)
{
	u64 res[KMB_ECC_MAX_DIGITS];

	const struct ecc_curve *curve = ecc_get_curve(curve_id);
	u64 one[KMB_ECC_MAX_DIGITS] = {1};
	u32 nbytes = data_size_u64_to_u8(ndigits);

	if (!curve)
		return -EINVAL;

	if (private_key_len != nbytes)
		return -EINVAL;

	if (!private_key)
		return -EINVAL;

	if (curve->g.ndigits != ndigits)
		return -EINVAL;

	/* Make sure the private key is in the range [2, n-3]. */
	if (vli_cmp(one, private_key, ndigits) != -1)
		return -EINVAL;

	vli_sub(res, curve->n, one, ndigits);
	vli_sub(res, res, one, ndigits);
	if (vli_cmp(res, private_key, ndigits) != 1)
		return -EINVAL;

	return 0;
}

/* Counts the number of 64-bit "digits" in vli. */
static unsigned int vli_num_digits(const u64 *vli, unsigned int ndigits)
{
	int i;

	/* Search from the end until we find a non-zero digit.
	 * We do it in reverse because we expect that most digits will
	 * be nonzero.
	 */
	for (i = ndigits - 1; (i >= 0) && (vli[i] == 0); i--)
		;	/* Do Nothing. */

	return (i + 1);
}

/* Counts the number of bits required for vli. */
static unsigned int vli_num_bits(const u64 *vli, unsigned int ndigits)
{
	unsigned int i;
	unsigned int num_digits;
	u64 digit;

	num_digits = vli_num_digits(vli, ndigits);
	if (num_digits == 0)
		return 0;

	digit = vli[num_digits - 1];
	for (i = 0; digit; i++)
		digit >>= 1;

	return ((num_digits - 1) * 64 + i);
}

/*
 * ECC private keys are generated using the method of extra random bits,
 * equivalent to that described in FIPS 186-4, Appendix B.4.1.
 *
 * d = (c mod(n–1)) + 1    where c is a string of random bits, 64 bits longer
 *                         than requested
 * 0 <= c mod(n-1) <= n-2  and implies that
 * 1 <= d <= n-1
 *
 * This method generates a private key uniformly distributed in the range
 * [1, n-1].
 */
int kmb_ecc_gen_privkey(unsigned int curve_id, unsigned int ndigits,
			u64 *privkey)
{
	u64 priv[KMB_ECC_MAX_DIGITS];

	const struct ecc_curve *curve = ecc_get_curve(curve_id);
	unsigned int nbytes = ndigits << ECC_DIGITS_TO_BYTES_SHIFT;
	unsigned int nbits = vli_num_bits(curve->n, ndigits);
	int err = 0;

	/* Check that N is included in Table 1 of FIPS 186-4, section 6.1.1 */
	if (nbits < 160 || ndigits > ARRAY_SIZE(priv))
		return -EINVAL;

	/*
	 * FIPS 186-4 recommends that the private key should be obtained from a
	 * RBG with a security strength equal to or greater than the security
	 * strength associated with N.
	 *
	 * The maximum security strength identified by NIST SP800-57pt1r4 for
	 * ECC is 256 (N >= 512).
	 *
	 * This condition is met by the default RNG because it selects a favored
	 * DRBG with a security strength of 256.
	 */
	if (crypto_get_default_rng())
		return -EFAULT;

	err = crypto_rng_get_bytes(crypto_default_rng, (u8 *)priv, nbytes);
	crypto_put_default_rng();
	if (err)
		goto cleanup;

	err = kmb_ecc_is_key_valid(curve_id, ndigits, priv, nbytes);
	if (err)
		goto cleanup;

	memcpy(privkey, priv, nbytes);

cleanup:
	memzero_explicit(&priv, sizeof(priv));

	return err;
}

static int kmb_ocs_ecdh_set_secret(struct crypto_kpp *tfm, const void *buf,
				   unsigned int len)
{
	struct ecdh params;
	unsigned int ndigits;

	int ret = 0;
	struct ocs_ecc_ctx *ctx = kpp_tfm_ctx(tfm);

	ret = crypto_ecdh_decode_key(buf, len, &params);
	if (ret)
		goto cleanup;

	ndigits = kmb_ocs_ecdh_supported_curve(params.curve_id);

	if (!ndigits) {
		ret = -EOPNOTSUPP;
		goto cleanup;
	}

	ctx->curve_id = params.curve_id;
	ctx->ndigits = ndigits;

	if (!params.key || !params.key_size) {
		ret = -EINVAL;
#ifdef	CONFIG_CRYPTO_DEV_KEEMBAY_OCS_ECDH_GEN_PRIV_KEY_SUPPORT
		ret = kmb_ecc_gen_privkey(ctx->curve_id, ctx->ndigits,
					  ctx->private_key);
#endif  /* CONFIG_CRYPTO_DEV_KEEMBAY_OCS_ECDH_GEN_PRIV_KEY_SUPPORT */
		if (ret)
			goto cleanup;
		goto swap_digits;
	}

	ret = kmb_ecc_is_key_valid(ctx->curve_id, ctx->ndigits,
				   (const u64 *)params.key, params.key_size);
	if (ret)
		goto cleanup;

swap_digits:
	ecc_swap_digits((u64 *)params.key, ctx->private_key, ctx->ndigits);
cleanup:
	memzero_explicit(&params, sizeof(params));

	return ret;
}

static int kmb_ocs_ecc_do_one_request(struct crypto_engine *engine,
				      void *areq)
{
	size_t copied, nbytes;
	struct ecc_point *pk;

	u64 *public_key = NULL;
	int ret = -ENOMEM;
	size_t public_key_sz = 0;
	struct kpp_request *req = container_of(areq, struct kpp_request,
					       base);
	struct ocs_ecc_ctx *ctx = kmb_ocs_kpp_ctx(req);
	const struct ecc_curve *curve = ecc_get_curve(ctx->curve_id);

	if (!ctx->private_key || !curve ||
	    (ctx->ndigits > KMB_ECC_MAX_DIGITS)) {
		ret = -EINVAL;
		goto out;
	}

	/* No spurious request checked at top level.*/
	if ((!req->src) && (!req->dst)) {
		ret = -EINVAL;
		goto out;
	}

	/* Store the request flag in ctx. */
	ctx->gfp = (req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP) ? GFP_KERNEL
								: GFP_ATOMIC;

	pk = ecc_alloc_point(ctx->ndigits, ctx->gfp);
	if (!pk) {
		ret = -ENOMEM;
		goto out;
	}

	/* Store the kpp_request struct in the device context. */
	ctx->ecc_dev->req = req;

	/* In case shared_secret branch not taken. */
	ctx->pk = NULL;

	nbytes = data_size_u64_to_u8(ctx->ndigits);
	/* Public part is a point thus it has both coordinates */
	public_key_sz = 2 * nbytes;

	public_key = kzalloc(public_key_sz, ctx->gfp);
	if (!public_key) {
		ret = -ENOMEM;
		goto free_all;
	}

	if (req->src) {
		/* from here on it's invalid parameters */
		ret = -EINVAL;

		/* must have exactly two points to be on the curve */
		if (public_key_sz != req->src_len)
			goto free_all;

		copied = sg_copy_to_buffer(req->src,
					   sg_nents_for_len(req->src,
							    public_key_sz),
					   public_key, public_key_sz);
		if (copied != public_key_sz)
			goto free_all;
		/* Store pk in the device context. */
		ctx->pk = pk;
		ecc_swap_digits(public_key, pk->x, ctx->ndigits);
		ecc_swap_digits(&public_key[ctx->ndigits], pk->y, ctx->ndigits);
		/*
		 * Check the public key for following
		 * Check 1: Verify key is not the zero point.
		 * Check 2: Verify key is in the range [1, p-1].
		 * Check 3: Verify that y^2 == (x^3 + a·x + b) mod p
		 */
		ret = kmb_ocs_ecc_is_pubkey_valid_partial(ctx->ecc_dev, curve,
							  pk);
	} else {
		/* Public Key(pk) = priv * G. */
		ret = ecc_point_mult(ctx->ecc_dev, pk, &curve->g,
				     ctx->private_key, curve, ctx->ndigits);
	}

	if (ret)
		goto free_all;
	goto return_success;

	/* follow through */
free_all:
	ecc_free_point(pk);
out:
	crypto_finalize_kpp_request(ctx->ecc_dev->engine,
				    req, ret);
return_success:
	if (public_key) {
		memzero_explicit(public_key, public_key_sz);
		kzfree(public_key);
	}
	return 0;
}

static int kmb_ocs_ecdh_compute_value(struct kpp_request *req)
{
	struct ocs_ecc_ctx *ctx = kmb_ocs_kpp_ctx(req);

	return crypto_transfer_kpp_request_to_engine(ctx->ecc_dev->engine,
						     req);
}

static int kmb_ocs_ecdh_init_tfm(struct crypto_kpp *tfm)
{
	struct ocs_ecc_ctx *ctx = kpp_tfm_ctx(tfm);

	ctx->ecc_dev = kmb_ocs_ecc_find_dev(ctx);

	if (IS_ERR(ctx->ecc_dev)) {
		pr_err("Failed to find the device : %ld\n",
		       PTR_ERR(ctx->ecc_dev));
		return PTR_ERR(ctx->ecc_dev);
	}

	ctx->engine_ctx.op.prepare_request = NULL;
	ctx->engine_ctx.op.do_one_request = kmb_ocs_ecc_do_one_request;
	ctx->engine_ctx.op.unprepare_request = NULL;

	return 0;
}

static void kmb_ocs_ecdh_exit_tfm(struct crypto_kpp *tfm)
{
	struct ocs_ecc_ctx *ctx = kpp_tfm_ctx(tfm);

	memzero_explicit(ctx->private_key, sizeof(*ctx->private_key));
}

static unsigned int kmb_ocs_ecdh_max_size(struct crypto_kpp *tfm)
{
	struct ocs_ecc_ctx *ctx = kpp_tfm_ctx(tfm);

	/* Public key is made of two coordinates, add one to the left shift */
	return ctx->ndigits << (ECC_DIGITS_TO_BYTES_SHIFT + 1);
}

static struct kpp_alg ocs_ecc_algs = {
	.set_secret = kmb_ocs_ecdh_set_secret,
	.generate_public_key = kmb_ocs_ecdh_compute_value,
	.compute_shared_secret = kmb_ocs_ecdh_compute_value,
	.init = kmb_ocs_ecdh_init_tfm,
	.exit = kmb_ocs_ecdh_exit_tfm,
	.max_size = kmb_ocs_ecdh_max_size,
	.base = {
		.cra_name = "ecdh",
		.cra_driver_name = "ecdh-keembay-ocs",
		.cra_priority = KMB_OCS_ECC_PRIORITY,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct ocs_ecc_ctx),
	},
};

/* called to indicate operation completion */
static irqreturn_t kmb_ocs_ecc_irq_thread(int irq, void *dev_id)
{
	size_t copied;
	static u64 yy[DATA_SIZE_U64_384BITS];
	static u64 x_cube[DATA_SIZE_U64_384BITS];

	u64 *pk_or_ss = NULL;
	struct ocs_ecc_dev *ecc_dev = dev_id;
	struct ocs_ecc_ctx *ctx = kmb_ocs_kpp_ctx(ecc_dev->req);
	const struct ecc_curve *curve = ecc_get_curve(ctx->curve_id);
	struct ecc_point *result = ctx->result_ptr;
	struct kpp_request *req = ecc_dev->req;
	size_t nbytes = data_size_u64_to_u8(result->ndigits);
	size_t public_key_sz = 2 * nbytes;
	int ret = 0;
	irqreturn_t irq_status = IRQ_HANDLED;

	long data_size_u8 = data_size_u64_to_u8(result->ndigits);

	/* Use the same ptr for shared_secret and pub_key.*/
	pk_or_ss = kzalloc(public_key_sz, ctx->gfp);
	if (!pk_or_ss) {
		ret = -ENOMEM;
		goto err_free_all;
	}

	if (!curve) {
		ret = -EINVAL;
		goto err_free_all;
	}

	ecc_read_cx_cy_out(ecc_dev, data_size_u8, result->x, result->y);

	if (req->src) {
		switch (ctx->ecc_next_state) {
		case SS_Y_SQUARE:
			ecc_read_cx_cy_out(ecc_dev, data_size_u8, result->x,
					   NULL);
			if (vli_is_zero(result->x, ctx->pk->ndigits)) {
				ret = -EINVAL;
				goto err_free_all;
			}
			memcpy(yy, result->x, nbytes);
			memzero_explicit(result->x, nbytes);
			/* Assigning result->x = 3, used for calcualting x^3.*/
			result->x[0] = POW_CUBE;
			/* Load the next stage.*/
			ctx->ecc_next_state = SS_X_CUBE;
			ret = ecc_ab_or_a_plus_b_modp(ecc_dev, result,
				ctx->pk->x, result->x, curve, ctx->pk->ndigits,
				ECC_INS_CALC_A_POW_B_MODP);
			if (!ret)
				goto return_irq_status;
			break;
		case SS_X_CUBE:
			ecc_read_cx_cy_out(ecc_dev, data_size_u8, result->x,
					   NULL);
			if (vli_is_zero(result->x, ctx->pk->ndigits)) {
				ret = -EINVAL;
				goto err_free_all;
			}
			memcpy(x_cube, result->x, nbytes);
			ctx->ecc_next_state = SS_A_MUL_X;
			ret = ecc_ab_or_a_plus_b_modp(ecc_dev, result,
				curve->a, ctx->pk->x, curve, ctx->pk->ndigits,
				ECC_INS_CALC_A_MUL_B_MODP);
			if (!ret)
				goto return_irq_status;
			break;
		case SS_A_MUL_X:
			ecc_read_cx_cy_out(ecc_dev, data_size_u8, result->x,
					   NULL);
			if (vli_is_zero(result->x, ctx->pk->ndigits)) {
				ret = -EINVAL;
				goto err_free_all;
			}
			ctx->ecc_next_state = SS_A_MUL_X_PLUS_B;
			ret = ecc_ab_or_a_plus_b_modp(ecc_dev, result,
				result->x, curve->b, curve, ctx->pk->ndigits,
				ECC_INS_CALC_A_ADD_B_MODP);
			if (!ret)
				goto return_irq_status;
			break;
		case SS_A_MUL_X_PLUS_B:
			ecc_read_cx_cy_out(ecc_dev, data_size_u8, result->x,
					   NULL);
			if (vli_is_zero(result->x, ctx->pk->ndigits)) {
				ret = -EINVAL;
				goto err_free_all;
			}
			ctx->ecc_next_state = SS_CHECK_PUB_KEY;
			ret = ecc_ab_or_a_plus_b_modp(ecc_dev, result, x_cube,
				result->x, curve, ctx->pk->ndigits,
				ECC_INS_CALC_A_ADD_B_MODP);
			if (!ret)
				goto return_irq_status;
			break;
		case SS_CHECK_PUB_KEY:
			ecc_read_cx_cy_out(ecc_dev, data_size_u8, result->x,
					   NULL);
			if (vli_is_zero(result->x, ctx->pk->ndigits)) {
				ret = -EINVAL;
				goto err_free_all;
			}
			ret = vli_cmp(yy, result->x, ctx->pk->ndigits);
			if (ret)
				goto err_free_all;
			/* Prepare for calculating the shared Secret.*/
			ctx->ecc_next_state = SS_ECC_POINT_MUL;
			ret = ecc_point_mult(ecc_dev, result, ctx->pk,
					     ctx->private_key, curve,
					     ctx->ndigits);
			if (!ret)
				goto return_irq_status;
			break;
		case SS_ECC_POINT_MUL:
			ctx->ecc_next_state = SS_NO_OP;
			/* The nbytes is same as setup above. */
			ecc_swap_digits(result->x, pk_or_ss,
					result->ndigits);
			if (ecc_point_is_zero(result))
				ret = -EFAULT;
			break;
		default:
			ret = -EIO;
			irq_status = IRQ_NONE;
			goto err_free_all;
		}

		/* Computation failed. Return Error back to caller.*/
		if (ret)
			goto err_free_all;

	} else {
		/* Public key generation branch. */
		if (ecc_point_is_zero(result)) {
			ret = -EAGAIN;
			goto err_free_all;
		}
		ecc_swap_digits(result->x, pk_or_ss,
				result->ndigits);
		ecc_swap_digits(result->y,
				&pk_or_ss[result->ndigits],
				result->ndigits);
		/*
		 * Reinitializing nbytes setup above
		 * to 2 * nbytes as public key comprises of
		 * point_x and point_y both of nbytes each.
		 */
		nbytes = public_key_sz;
	}

	/* might want less than we've got */
	nbytes = min_t(size_t, nbytes, req->dst_len);

	copied = sg_copy_from_buffer(req->dst, sg_nents_for_len(req->dst,
								nbytes),
				     (void *)pk_or_ss, nbytes);

	if (copied != nbytes)
		ret = -EINVAL;

err_free_all:
	if (ctx->pk)
		ecc_free_point(ctx->pk);
	ecc_free_point(result);
	crypto_finalize_kpp_request(ctx->ecc_dev->engine,
				    ctx->ecc_dev->req, ret);
return_irq_status:
	if (pk_or_ss) {
		memzero_explicit(pk_or_ss, public_key_sz);
		kzfree(pk_or_ss);
	}
	return irq_status;
}

static irqreturn_t ocs_ecc_irq_handler(int irq, void *dev_id)
{
	struct ocs_ecc_dev *ecc_dev = dev_id;
	u32 status = 0;

	/*
	 * Read the status register
	 * Write the status back to clear the DONE_INT_STATUS bit
	 */

	status = ioread32(ecc_dev->base_reg + HW_OFFS_OCS_ECC_ISR);

	iowrite32(status, ecc_dev->base_reg + HW_OFFS_OCS_ECC_ISR);

	if (!(status & HW_OCS_ECC_ISR_INT_STATUS_DONE))
		return IRQ_NONE;

	return IRQ_WAKE_THREAD;
}

static int kmb_ocs_ecc_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct ocs_ecc_dev *ecc_dev;

	dev_info(dev, "Probing Keembay OCS ECC Driver\n");

	ecc_dev = devm_kzalloc(dev, sizeof(*ecc_dev), GFP_KERNEL);
	if (!ecc_dev)
		return -ENOMEM;

	ecc_dev->dev = dev;

	platform_set_drvdata(pdev, ecc_dev);

	INIT_LIST_HEAD(&ecc_dev->list);

	/* Get base register address. */
	ecc_dev->base_reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ecc_dev->base_reg)) {
		dev_err(dev, "Failed to get base address\n");
		rc = PTR_ERR(ecc_dev->base_reg);
		goto list_del;
	}

	/* Get and request IRQ */
	ecc_dev->irq = platform_get_irq(pdev, 0);
	if (ecc_dev->irq < 0) {
		dev_err(dev, "Could not retrieve IRQ\n");
		rc = ecc_dev->irq;
		goto list_del;
	}

	rc = devm_request_threaded_irq(dev, ecc_dev->irq, ocs_ecc_irq_handler,
				       kmb_ocs_ecc_irq_thread, 0,
				       "keembay-ocs-ecc", ecc_dev);
	if (rc < 0) {
		dev_err(dev, "Could not request IRQ\n");
		goto list_del;
	}

	spin_lock(&ocs_ecc.lock);
	list_add_tail(&ecc_dev->list, &ocs_ecc.dev_list);
	spin_unlock(&ocs_ecc.lock);

	/* Initialize crypto engine */
	ecc_dev->engine = crypto_engine_alloc_init(dev, 1);
	if (!ecc_dev->engine) {
		dev_err(dev, "Could not allocate crypto engine\n");
		goto list_del;
	}

	rc = crypto_engine_start(ecc_dev->engine);
	if (rc) {
		dev_err(dev, "Could not start crypto engine\n");
		goto cleanup;
	}

	/* Register the KPP algo. */
	rc = crypto_register_kpp(&ocs_ecc_algs);
	if (rc) {
		dev_err(dev, "Could not register OCS algorithms with Crypto API\n");
		goto list_del;
	}

	return 0;

cleanup:
	crypto_engine_exit(ecc_dev->engine);
list_del:
	spin_lock(&ocs_ecc.lock);
	list_del(&ecc_dev->list);
	spin_unlock(&ocs_ecc.lock);

	return rc;
}

static int kmb_ocs_ecc_remove(struct platform_device *pdev)
{
	struct ocs_ecc_dev *ecc_dev;

	ecc_dev = platform_get_drvdata(pdev);
	if (!ecc_dev)
		return -ENODEV;

	crypto_unregister_kpp(&ocs_ecc_algs);

	spin_lock(&ocs_ecc.lock);
	list_del(&ecc_dev->list);
	spin_unlock(&ocs_ecc.lock);

	crypto_engine_exit(ecc_dev->engine);

	return 0;
}


/* The OCS driver is a platform device. */
static struct platform_driver kmb_ocs_ecc_driver = {
	.probe = kmb_ocs_ecc_probe,
	.remove = kmb_ocs_ecc_remove,
	.driver = {
			.name = DRV_NAME,
			.of_match_table = kmb_ocs_ecc_of_match,
		},
};

module_platform_driver(kmb_ocs_ecc_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Keembay Offload and Crypto Subsystem (OCS) Elliptic Curve Cryptography Driver");
MODULE_ALIAS_CRYPTO("ecdh");
MODULE_ALIAS_CRYPTO("ecdh-keembay-ocs");
