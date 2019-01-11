/* SPDX-License-Identifier: GPL-2.0 */
/* Intel Keystore Linux driver, Copyright (c) 2018, Intel Corporatio. */
#ifndef _MANIFEST_VERIFY_H_
#define _MANIFEST_VERIFY_H_

#include "manifest_parser.h"

#define MANIFEST_CACHE_TTL		300
#define MANIFEST_DEFAULT_CAPS		(CAPABILITY_KEYSTORE)

enum APP_AUTH_ERROR {
	NO_ERROR,
	MALFORMED_MANIFEST = 300,
	CERTIFICATE_FAILURE,
	CERTIFICATE_EXPIRED,
	CAPS_FAILURE,
	SIGNATURE_FAILURE,
	EXE_NOT_FOUND,
	FILE_TOO_BIG,
	HASH_FAILURE,
	KEY_LOAD_ERROR,
	KEY_RETRIEVE_ERROR,
	KEYID_NOT_FOUND
};

int verify_manifest_file(char *manifest_file_path,
					int timeout, uint16_t caps);
#endif /* _MANIFEST_VERIFY_H_ */
