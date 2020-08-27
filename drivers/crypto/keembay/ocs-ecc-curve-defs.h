/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay OCS ECC Curve Definitions.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */
#ifndef _CRYPTO_KEEMBAY_OCS_ECC_CURVE_DEFS_H
#define _CRYPTO_KEEMBAY_OCS_ECC_CURVE_DEFS_H

/* NIST P-256: a = p - 3 */
static u64 nist_p256_g_x[] = { 0xF4A13945D898C296ull, 0x77037D812DEB33A0ull,
				0xF8BCE6E563A440F2ull, 0x6B17D1F2E12C4247ull };
static u64 nist_p256_g_y[] = { 0xCBB6406837BF51F5ull, 0x2BCE33576B315ECEull,
				0x8EE7EB4A7C0F9E16ull, 0x4FE342E2FE1A7F9Bull };
static u64 nist_p256_p[] = { 0xFFFFFFFFFFFFFFFFull, 0x00000000FFFFFFFFull,
				0x0000000000000000ull, 0xFFFFFFFF00000001ull };
static u64 nist_p256_n[] = { 0xF3B9CAC2FC632551ull, 0xBCE6FAADA7179E84ull,
				0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFF00000000ull };
static u64 nist_p256_a[] = { 0xFFFFFFFFFFFFFFFCull, 0x00000000FFFFFFFFull,
				0x0000000000000000ull, 0xFFFFFFFF00000001ull };
static u64 nist_p256_b[] = { 0x3BCE3C3E27D2604Bull, 0x651D06B0CC53B0F6ull,
				0xB3EBBD55769886BCull, 0x5AC635D8AA3A93E7ull };
static struct ecc_curve nist_p256 = {
	.name = "nist_256",
	.g = {
		.x = nist_p256_g_x,
		.y = nist_p256_g_y,
		.ndigits = 4,
	},
	.p = nist_p256_p,
	.n = nist_p256_n,
	.a = nist_p256_a,
	.b = nist_p256_b
};

/* NIST P-384: a = p - 3 */
static u64 nist_p384_g_x[] = { 0x3A545E3872760AB7ull, 0x5502F25DBF55296Cull,
				0x59F741E082542A38ull, 0x6E1D3B628BA79B98ull,
				0x8EB1C71EF320AD74ull, 0xAA87CA22BE8B0537ull };
static u64 nist_p384_g_y[] = { 0x7A431D7C90EA0E5F, 0x0A60B1CE1D7E819Dull,
				0xE9DA3113B5F0B8C0ull, 0xF8F41DBD289A147Cull,
				0x5D9E98BF9292DC29ull, 0x3617DE4A96262C6Full };
static u64 nist_p384_p[] = { 0x00000000FFFFFFFFull, 0xFFFFFFFF00000000ull,
				0xFFFFFFFFFFFFFFFEull, 0xFFFFFFFFFFFFFFFFull,
				0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFFull };
static u64 nist_p384_n[] = { 0xECEC196ACCC52973ull, 0x581A0DB248B0A77Aull,
				0xC7634D81F4372DDFull, 0xFFFFFFFFFFFFFFFF,
				0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFFull};
static u64 nist_p384_a[] = { 0x00000000FFFFFFFCull, 0xFFFFFFFF00000000ull,
				0xFFFFFFFFFFFFFFFEull, 0xFFFFFFFFFFFFFFFFull,
				0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFFull};
static u64 nist_p384_b[] = { 0x2A85C8EDD3EC2AEFull, 0xC656398D8A2ED19Dull,
				0x0314088F5013875Aull, 0x181D9C6EFE814112ull,
				0x988E056BE3F82D19ull, 0xB3312FA7E23EE7E4ull };
static struct ecc_curve nist_p384 = {
	.name = "nist_384",
	.g = {
		.x = nist_p384_g_x,
		.y = nist_p384_g_y,
		.ndigits = 6,
	},
	.p = nist_p384_p,
	.n = nist_p384_n,
	.a = nist_p384_a,
	.b = nist_p384_b
};

#endif
