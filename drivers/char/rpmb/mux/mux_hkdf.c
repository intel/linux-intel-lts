// SPDX-License-Identifier: GPL-2.0
/*
 * RPMB Mux HKDF
 *
 * Copyright (c) 2018-2019 Intel Corporation.
 */

#include <linux/compat.h>
#include <crypto/hash.h>
#include "mux_hkdf.h"

static int mux_sha256_extract(u8 *out_key, size_t out_len,
			      struct shash_desc *desc,
			      const u8 *secret, size_t secret_len,
			      const u8 *salt, size_t salt_len)
{
	int ret;
	u8 salt0[SHA256_HASH_SIZE];

	if (!salt || !salt_len) {
		memset(salt0, 0, sizeof(salt0));
		salt = salt0;
		salt_len = sizeof(salt0);
	}

	ret = crypto_shash_setkey(desc->tfm, salt, salt_len);
	if (ret) {
		pr_err("set key failed = %d\n", ret);
		goto out;
	}

	ret = crypto_shash_init(desc);
	if (ret)
		goto out;

	ret = crypto_shash_update(desc, secret, secret_len);
	if (ret)
		goto out;

	ret = crypto_shash_final(desc, out_key);
	if (ret)
		goto out;

out:
	return ret;
}

static int mux_sha256_expand(u8 *out_key, size_t out_len,
			     struct shash_desc *desc,
			     const u8 *prk, size_t prk_len,
			     const u8 *info, size_t info_len)
{
	const size_t digest_len = SHA256_HASH_SIZE;
	u8 previous[SHA256_HASH_SIZE];
	size_t n, done = 0;
	unsigned int i;
	int ret = 0;

	n = (out_len + digest_len - 1) / digest_len;

	/* check for possible integer overflow */
	if (out_len + digest_len < out_len)
		return 0;

	if (n > 255)
		return 0;

	for (i = 0; i < n; i++) {
		u8 ctr = i + 1;
		size_t todo;

		ret = crypto_shash_setkey(desc->tfm, prk, prk_len);
		if (ret)
			goto out;

		ret = crypto_shash_init(desc);
		if (ret)
			goto out;

		if (i != 0 && crypto_shash_update(desc, previous, digest_len))
			goto out;

		if (crypto_shash_update(desc, info, info_len) ||
		    crypto_shash_update(desc, &ctr, 1) ||
		    crypto_shash_final(desc, previous)) {
			ret = -EPERM;
			goto out;
		}

		todo = digest_len;
		/* Check if the length of left buffer is smaller than
		 * 32 to make sure no buffer overflow in below memcpy
		 */
		if (done + todo > out_len)
			todo = out_len - done;

		memcpy(out_key + done, previous, todo);
		done += todo;
	}

out:
	memset(previous, 0, sizeof(previous));

	return ret;
}

static struct shash_desc *mux_hkdf_init_hmac_sha256_desc(void)
{
	struct shash_desc *desc;
	struct crypto_shash *tfm;

	tfm = crypto_alloc_shash("hmac(sha256)", 0, 0);
	if (IS_ERR(tfm))
		return ERR_PTR(-EFAULT);

	desc = kzalloc(sizeof(*desc) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (!desc) {
		crypto_free_shash(tfm);
		return ERR_PTR(-ENOMEM);
	}
	desc->tfm = tfm;

	return desc;
}

int mux_hkdf_sha256(u8 *out_key, size_t out_len,
		    const u8 *secret, size_t secret_len,
		    const u8 *salt, size_t salt_len,
		    const u8 *info, size_t info_len)
{
	u8 prk[SHA256_HASH_SIZE];
	size_t prk_len = SHA256_HASH_SIZE;
	int ret;
	struct shash_desc *desc;

	if (!out_key || !out_len)
		return -EINVAL;

	if (!secret || !secret_len)
		return -EINVAL;

	if (!info && info_len)
		return -EINVAL;

	desc = mux_hkdf_init_hmac_sha256_desc();
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	memset(prk, 0, sizeof(prk));

	ret = mux_sha256_extract(prk, prk_len, desc,
				 secret, secret_len,
				 salt, salt_len);
	if (ret)
		goto err_free_shash;

	ret = mux_sha256_expand(out_key, out_len, desc,
				prk, prk_len,
				info, info_len);

err_free_shash:
	crypto_free_shash(desc->tfm);
	kfree(desc);

	return ret;
}
