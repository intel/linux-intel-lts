// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Support for AES Key Locker instructions. This file contains glue
 * code, the real AES implementation will be in aeskl-intel_asm.S.
 *
 * Most code is based on AES-NI glue code, aesni-intel_glue.c
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/err.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/xts.h>
#include <asm/cpu_device_id.h>
#include <asm/fpu/api.h>
#include <asm/simd.h>
#include <asm/keylocker.h>
#include <crypto/internal/skcipher.h>
#include <crypto/internal/simd.h>

#include <crypto/b128ops.h>
#include <crypto/gf128mul.h>
#include <crypto/scatterwalk.h>

#include "aes-intel_glue.h"

asmlinkage int _aeskl_setkey(struct crypto_aes_ctx *ctx, const u8 *in_key, unsigned int key_len);

asmlinkage int _aeskl_enc(const void *ctx, u8 *out, const u8 *in);
asmlinkage int _aeskl_dec(const void *ctx, u8 *out, const u8 *in);

asmlinkage int _aeskl_ecb_enc(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len);
asmlinkage int _aeskl_ecb_dec(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len);

asmlinkage int _aeskl_cbc_enc(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len,
			      u8 *iv);
asmlinkage int _aeskl_cbc_dec(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len,
			      u8 *iv);

#ifdef CONFIG_X86_64
asmlinkage int _aeskl_ctr_enc(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len,
			      u8 *iv);
#endif

asmlinkage int _aeskl_xts_crypt8(const struct crypto_aes_ctx *ctx, u8 *out,
				  const u8 *in, bool enc, u8 *iv);

static int aeskl_setkey_common(struct crypto_tfm *tfm, void *raw_ctx, const u8 *in_key,
			       unsigned int key_len)
{
	struct crypto_aes_ctx *ctx = aes_ctx(raw_ctx);
	int err;

	if (!crypto_simd_usable())
		return -EBUSY;

	/*
	 * 192-bit key is not supported by Key Locker. Fall back to
	 * the AES-NI implementation.
	 */
	if (unlikely(key_len == AES_KEYSIZE_192)) {
		kernel_fpu_begin();
		err = aesni_set_key(ctx, in_key, key_len);
		kernel_fpu_end();
		return err;
	}

	if (key_len != AES_KEYSIZE_128 && key_len != AES_KEYSIZE_256)
		return -EINVAL;

	kernel_fpu_begin();
	/* Encode the key to a handle usable only in kernel mode. */
	err = _aeskl_setkey(ctx, in_key, key_len);
	kernel_fpu_end();
	return err;
}

static inline u32 keylength(const void *raw_ctx)
{
	struct crypto_aes_ctx *ctx = aes_ctx((void *)raw_ctx);

	return ctx->key_length;
}

static inline int aeskl_enc(const void *ctx, u8 *out, const u8 *in)
{
	if (unlikely(keylength(ctx) == AES_KEYSIZE_192))
		return aesni_enc(ctx, out, in);
	return _aeskl_enc(ctx, out, in);
}

static inline int aeskl_dec(const void *ctx, u8 *out, const u8 *in)
{
	if (unlikely(keylength(ctx) == AES_KEYSIZE_192))
		return aesni_dec(ctx, out, in);
	return _aeskl_dec(ctx, out, in);
}

static int aeskl_skcipher_setkey(struct crypto_skcipher *tfm, const u8 *key,
				 unsigned int len)
{
	return aeskl_setkey_common(crypto_skcipher_tfm(tfm),
				   crypto_skcipher_ctx(tfm), key, len);
}

static int aeskl_ecb_enc(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len)
{
	if (unlikely(ctx->key_length == AES_KEYSIZE_192))
		return aesni_ecb_enc(ctx, out, in, len);

	if (_aeskl_ecb_enc(ctx, out, in, len))
		return -EINVAL;
	return 0;
}

static int aeskl_ecb_dec(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in, unsigned int len)
{
	if (unlikely(ctx->key_length == AES_KEYSIZE_192))
		return aesni_ecb_dec(ctx, out, in, len);

	if (_aeskl_ecb_dec(ctx, out, in, len))
		return -EINVAL;
	return 0;
}

static int aeskl_cbc_enc(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in,
			unsigned int len, u8 *iv)
{
	if (unlikely(ctx->key_length == AES_KEYSIZE_192))
		return aesni_cbc_enc(ctx, out, in, len, iv);

	if (_aeskl_cbc_enc(ctx, out, in, len, iv))
		return -EINVAL;
	return 0;
}

static int aeskl_cbc_dec(struct crypto_aes_ctx *ctx, u8 *out, const u8 *in,
			unsigned int len, u8 *iv)
{
	if (unlikely(ctx->key_length == AES_KEYSIZE_192))
		return aesni_cbc_dec(ctx, out, in, len, iv);

	if (_aeskl_cbc_dec(ctx, out, in, len, iv))
		return -EINVAL;
	return 0;
}

static int ecb_encrypt(struct skcipher_request *req)
{
	return ecb_crypt_common(req, aeskl_ecb_enc);
}

static int ecb_decrypt(struct skcipher_request *req)
{
	return ecb_crypt_common(req, aeskl_ecb_dec);
}

static int cbc_encrypt(struct skcipher_request *req)
{
	return cbc_crypt_common(req, aeskl_cbc_enc);
}

static int cbc_decrypt(struct skcipher_request *req)
{
	return cbc_crypt_common(req, aeskl_cbc_dec);
}

#ifdef CONFIG_X86_64

static int aeskl_ctr_enc1(const void *ctx, u8 *out, const u8 *in)
{
	int err = 0;

	if (unlikely(keylength(ctx) == AES_KEYSIZE_192))
		aesni_enc(ctx, out, in);
	else
		err = _aeskl_enc(ctx, out, in);

	return err;
}

static int aeskl_ctr_enc(struct crypto_aes_ctx *ctx, u8 *out,
			      const u8 *in, unsigned int len, u8 *iv)
{
	if (unlikely(ctx->key_length == AES_KEYSIZE_192))
		return aesni_ctr_enc(ctx, out, in, len, iv);

	if (_aeskl_ctr_enc(ctx, out, in, len, iv))
		return -EINVAL;
	return 0;
}

static int ctr_crypt(struct skcipher_request *req)
{
	return ctr_crypt_common(req, aeskl_ctr_enc, aeskl_ctr_enc1);
}

#endif /* CONFIG_X86_64 */

static int xts_aeskl_setkey(struct crypto_skcipher *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct aes_xts_ctx *ctx = crypto_skcipher_ctx(tfm);
	int err;

	err = xts_verify_key(tfm, key, keylen);
	if (err)
		return err;

	keylen /= 2;

	/* first half of xts-key is for crypt */
	err = aeskl_setkey_common(crypto_skcipher_tfm(tfm), ctx->raw_crypt_ctx, key, keylen);
	if (err)
		return err;

	/* second half of xts-key is for tweak */
	return aeskl_setkey_common(crypto_skcipher_tfm(tfm), ctx->raw_tweak_ctx, key + keylen,
				   keylen);
}

static inline bool glue_fpu_begin(unsigned int bsize, int fpu_blocks_limit,
				  struct skcipher_walk *walk,
				  bool fpu_enabled, unsigned int nbytes)
{
	if (likely(fpu_blocks_limit < 0))
		return false;

	if (fpu_enabled)
		return true;

	/*
	 * Vector-registers are only used when chunk to be processed is large
	 * enough, so do not enable FPU until it is necessary.
	 */
	if (nbytes < bsize * (unsigned int)fpu_blocks_limit)
		return false;

	/* prevent sleeping if FPU is in use */
	walk->flags &= ~(1 << 4); //SKCIPHER_WALK_SLEEP;

	kernel_fpu_begin();
	return true;
}

static int aeskl_xts_crypt1(const void *ctx, u8 *dst, const u8 *src,
			       bool encrypt, le128 *iv)
{
	int err;
	le128 ivblk = *iv;

	/* generate next IV */
	gf128mul_x_ble(iv, &ivblk);

	/* CC <- T xor C */
	u128_xor((u128 *)dst, (const u128 *)src, (u128 *)&ivblk);

	/* PP <- D(Key2,CC) */
	if (encrypt)
		err = aeskl_enc(ctx, dst, dst);
	else
		err = aeskl_dec(ctx, dst, dst);

	/* P <- T xor PP */
	u128_xor((u128 *)dst, (u128 *)dst, (u128 *)&ivblk);

	return err;
}

#define NUM_XTS_FUNCS	2

static int aeskl_xts_crypt(void *ctx,
					  struct skcipher_walk *walk,
					  unsigned int *nbytes,
					  bool encrypt)
{
	u128 *src = walk->src.virt.addr;
	u128 *dst = walk->dst.virt.addr;
	unsigned int i;
	int err = 0;

	*nbytes = walk->nbytes;

	/* Process multi-block batch */
	for (i = 0; i < NUM_XTS_FUNCS; i++) {
		unsigned int num_blocks, func_bytes;

		num_blocks = i == 0 ? 8 : 1;
		func_bytes = AES_BLOCK_SIZE * num_blocks;

		if (*nbytes >= func_bytes) {
			do {
				if (num_blocks == 8)
					err = _aeskl_xts_crypt8(ctx, (u8 *)dst, (const u8 *)src,
								encrypt, walk->iv);
				else
					err = aeskl_xts_crypt1(ctx, (u8 *)dst, (const u8 *)src,
								encrypt, walk->iv);
				src += num_blocks;
				dst += num_blocks;
				*nbytes -= func_bytes;

				if (err)
					goto done;

			} while (*nbytes >= func_bytes);

			if (*nbytes < AES_BLOCK_SIZE)
				goto done;
		}
	}

done:
	return err;
}

static int xts_crypt(struct skcipher_request *req, bool encrypt)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct aes_xts_ctx *ctx = crypto_skcipher_ctx(tfm);
	const bool cts = (req->cryptlen % XTS_BLOCK_SIZE);
	struct skcipher_request subreq;
	struct skcipher_walk walk;
	unsigned int nbytes, tail;
	int err;

	if (req->cryptlen < XTS_BLOCK_SIZE)
		return -EINVAL;

	if (unlikely(cts)) {
		struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);

		tail = req->cryptlen % XTS_BLOCK_SIZE + XTS_BLOCK_SIZE;

		skcipher_request_set_tfm(&subreq, tfm);
		skcipher_request_set_callback(&subreq,
					      crypto_skcipher_get_flags(tfm),
					      NULL, NULL);
		skcipher_request_set_crypt(&subreq, req->src, req->dst,
					   req->cryptlen - tail, req->iv);
		req = &subreq;
	}

	err = skcipher_walk_virt(&walk, req, false);
	nbytes = walk.nbytes;
	if (err)
		return err;

	kernel_fpu_begin();

	/* calculate first value of T */
	aeskl_enc(aes_ctx(ctx->raw_tweak_ctx), walk.iv, walk.iv);

	while (nbytes) {
		err = aeskl_xts_crypt(aes_ctx(ctx->raw_crypt_ctx), &walk, &nbytes, encrypt);
		if (err) {
			kernel_fpu_end();
			return err;
		}

		err = skcipher_walk_done(&walk, nbytes);
		if (err)
			return err;

		nbytes = walk.nbytes;
		if (nbytes)
			kernel_fpu_begin();
	}

	if (unlikely(cts)) {
		u8 *next_tweak, *final_tweak = req->iv;
		struct scatterlist *src, *dst;
		struct scatterlist s[2], d[2];
		le128 b[2];

		dst = src = scatterwalk_ffwd(s, req->src, req->cryptlen);
		if (req->dst != req->src)
			dst = scatterwalk_ffwd(d, req->dst, req->cryptlen);

		if (encrypt) {
			next_tweak = req->iv;
		} else {
			next_tweak = memcpy(b, req->iv, XTS_BLOCK_SIZE);
			gf128mul_x_ble(b, b);
		}

		skcipher_request_set_crypt(&subreq, src, dst, XTS_BLOCK_SIZE,
					   next_tweak);

		err = skcipher_walk_virt(&walk, req, false);
		if (err)
			return err;

		kernel_fpu_begin();
		err = aeskl_xts_crypt(aes_ctx(ctx->raw_crypt_ctx), &walk, &nbytes, encrypt);
		kernel_fpu_end();
		if (err)
			return err;

		err = skcipher_walk_done(&walk, nbytes);
		if (err)
			return err;

		scatterwalk_map_and_copy(b, dst, 0, XTS_BLOCK_SIZE, 0);
		memcpy(b + 1, b, tail - XTS_BLOCK_SIZE);
		scatterwalk_map_and_copy(b, src, XTS_BLOCK_SIZE,
					 tail - XTS_BLOCK_SIZE, 0);
		scatterwalk_map_and_copy(b, dst, 0, tail, 1);

		skcipher_request_set_crypt(&subreq, dst, dst, XTS_BLOCK_SIZE,
					   final_tweak);

		err = skcipher_walk_virt(&walk, req, false);
		if (err)
			return err;

		kernel_fpu_begin();
		err = aeskl_xts_crypt(aes_ctx(ctx->raw_crypt_ctx), &walk, &nbytes, encrypt);
		kernel_fpu_end();
		if (err)
			return err;

		err = skcipher_walk_done(&walk, nbytes);
		if (err)
			return err;
	}

	return err;
}

static int xts_aeskl_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct aes_xts_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (unlikely(keylength(ctx->raw_crypt_ctx) == AES_KEYSIZE_192))
		return xts_aesni_encrypt(req);

	return xts_crypt(req, true);
}

static int xts_aeskl_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct aes_xts_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (unlikely(keylength(ctx->raw_crypt_ctx) == AES_KEYSIZE_192))
		return xts_aesni_decrypt(req);

	return xts_crypt(req, false);
}

static struct skcipher_alg aeskl_skciphers[] = {
	{
		.base = {
			.cra_name		= "__ecb(aes)",
			.cra_driver_name	= "__ecb-aes-aeskl",
			.cra_priority		= 401,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.setkey		= aeskl_skcipher_setkey,
		.encrypt	= ecb_encrypt,
		.decrypt	= ecb_decrypt,
	}, {
		.base = {
			.cra_name		= "__cbc(aes)",
			.cra_driver_name	= "__cbc-aes-aeskl",
			.cra_priority		= 401,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= aeskl_skcipher_setkey,
		.encrypt	= cbc_encrypt,
		.decrypt	= cbc_decrypt,
#ifdef CONFIG_X86_64
	}, {
		.base = {
			.cra_name		= "__ctr(aes)",
			.cra_driver_name	= "__ctr-aes-aeskl",
			.cra_priority		= 401,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= 1,
			.cra_ctxsize		= CRYPTO_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= AES_MIN_KEY_SIZE,
		.max_keysize	= AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.chunksize	= AES_BLOCK_SIZE,
		.setkey		= aeskl_skcipher_setkey,
		.encrypt	= ctr_crypt,
		.decrypt	= ctr_crypt,
	}, {
		.base = {
			.cra_name		= "__xts(aes)",
			.cra_driver_name	= "__xts-aes-aeskl",
			.cra_priority		= 402,
			.cra_flags		= CRYPTO_ALG_INTERNAL,
			.cra_blocksize		= AES_BLOCK_SIZE,
			.cra_ctxsize		= XTS_AES_CTX_SIZE,
			.cra_module		= THIS_MODULE,
		},
		.min_keysize	= 2 * AES_MIN_KEY_SIZE,
		.max_keysize	= 2 * AES_MAX_KEY_SIZE,
		.ivsize		= AES_BLOCK_SIZE,
		.setkey		= xts_aeskl_setkey,
		.encrypt	= xts_aeskl_encrypt,
		.decrypt	= xts_aeskl_decrypt,
#endif
	}
};

static struct simd_skcipher_alg *aeskl_simd_skciphers[ARRAY_SIZE(aeskl_skciphers)];

static const struct x86_cpu_id aes_keylocker_cpuid[] = {
	X86_MATCH_FEATURE(X86_FEATURE_AES, NULL),
	X86_MATCH_FEATURE(X86_FEATURE_KEYLOCKER, NULL),
	{}
};

static int __init aeskl_init(void)
{
	u32 eax, ebx, ecx, edx;
	int err;

	if (!x86_match_cpu(aes_keylocker_cpuid))
		return -ENODEV;

	cpuid_count(KEYLOCKER_CPUID, 0, &eax, &ebx, &ecx, &edx);
	if (!(ebx & KEYLOCKER_CPUID_EBX_AESKLE) ||
	    !(eax & KEYLOCKER_CPUID_EAX_SUPERVISOR) ||
	    !(ebx & KEYLOCKER_CPUID_EBX_WIDE))
		return -ENODEV;

	err = simd_register_skciphers_compat(aeskl_skciphers, ARRAY_SIZE(aeskl_skciphers),
					     aeskl_simd_skciphers);
	if (err)
		return err;

	return 0;
}

static void __exit aeskl_exit(void)
{
	simd_unregister_skciphers(aeskl_skciphers, ARRAY_SIZE(aeskl_skciphers),
				  aeskl_simd_skciphers);
}

late_initcall(aeskl_init);
module_exit(aeskl_exit);

MODULE_DESCRIPTION("Rijndael (AES) Cipher Algorithm, AES Key Locker implementation");
MODULE_LICENSE("GPL");
MODULE_ALIAS_CRYPTO("aes");
