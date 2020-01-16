/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Intel Corp.
 */
#ifndef _MUX_HKDF_H
#define _MUX_HKDF_H

#define SHA256_HASH_SIZE 32

int mux_hkdf_sha256(u8 *out_key, size_t out_len,
		    const u8 *secret, size_t secret_len,
		    const u8 *salt, size_t salt_len,
		    const u8 *info, size_t info_len);
#endif /* !_MUX_HKDF_H */
