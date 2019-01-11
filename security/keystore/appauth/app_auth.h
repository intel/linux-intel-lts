/* SPDX-License-Identifier: GPL-2.0 */
/* Intel Keystore Linux driver, Copyright (c) 2018, Intel Corporatio. */
#ifndef _APP_AUTH_H_
#define _APP_AUTH_H_

#include <linux/file.h>
#include <linux/crypto.h>
#include <linux/key-type.h>
#include <linux/mpi.h>
#include <linux/kernel.h>
#include <linux/sched/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <crypto/hash.h>
#include <crypto/hash_info.h>
#include <crypto/public_key.h>
#include <keys/asymmetric-type.h>
#include <keys/system_keyring.h>
#include <keys/asymmetric-subtype.h>
#include "../../integrity/ima/ima.h"
#include "../../crypto/asymmetric_keys/x509_parser.h"
#include "manifest_verify.h"
#include "../keystore_debug.h"

#define  DEBUG_APPAUTH  APP_AUTH
#define  DEBUG_APPAUTH_STR  "APP_AUTH"
#define  APP_AUTH_DIGEST_MAX 64
#define  manifest_hash_algo  HASH_ALGO_SHA1
#define  manifest_default_hash_algo  HASH_ALGO_SHA1
#define  default_sig_hash_algo HASH_ALGO_SHA256
#define  KEYID_MAX_LEN 64
/* maximum size of any files in the application */
#define  MAX_FILE_SIZE (64 * 1024 * 1024)
/* maximum length of the application manifest file */
#define  MANIFEST_MAX_LEN 5000

struct appauth_digest {
	int algo;
	unsigned int len;
	unsigned char digest[APP_AUTH_DIGEST_MAX];
};

struct crypto_shash *appauth_alloc_tfm(enum hash_algo algo);

void appauth_free_tfm(struct crypto_shash *tfm);

int appauth_kernel_read(struct file *file, loff_t offset,
				char *addr, unsigned long count);
int read_manifest(const char *filename, char **manifest_buf, int *manifest_len);

char *get_exe_name(char **buf);

void debug_public_key(struct public_key *key);

void appauth_free_buf(char **manifest_buf);

int compute_file_hash(const char *filename, uint8_t *digest,
					uint8_t digest_algo_id);

const char *get_keyid_from_cert(struct x509_certificate *cert);

int verify_manifest(const char *sig, const char *cert, const char *data,
				int sig_len, int cert_len, int data_len);

int manifest_sanity_check(char *manifest_buf, uint16_t manifest_len);
#endif /* _APP_AUTH_H_ */
