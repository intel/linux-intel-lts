/*
 *
 * Intel Keystore Linux driver
 * Copyright (c) 2018, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef _KEYSTORE_API_COMMON_H_
#define _KEYSTORE_API_COMMON_H_

#include <linux/types.h>

/**
 * DOC: Introduction
 *
 * Common constants and structures common to both user- and kernel-space
 * clients are listed here.
 *
 */

/* Version numbers of the Keystore API
 * Follows the Apache versioning scheme
 *
 * Major versions represent large scale changes in the API.
 * Minor changes return API compatibility with older minor versions.
 * Patch changes are forwards and backwards compatible.
 *
 * Ensure that version numbers are updated if changes are made to
 * the API!
 */
#define KEYSTORE_VERSION_MAJOR 2
#define KEYSTORE_VERSION_MINOR 1
#define KEYSTORE_VERSION_PATCH 1

/**
 * KEYSTORE_CLIENT_TICKET_SIZE - client_ticket size in bytes
 */
#define KEYSTORE_CLIENT_TICKET_SIZE	8

/**
 * KEYSTORE_MAX_CLIENT_ID_SIZE - size of the client ID
 *
 * The client ID is unique per application and is not normally
 * needed outside of Keystore. It is only required when migrated
 * from one application to another during the backup and migration
 * proceedure.
 */
#define KEYSTORE_MAX_CLIENT_ID_SIZE	32

/**
 * KEYSTORE_CLIENT_KEY_SIZE - size of the client key
 *
 * The client keys are used to wrap application keys in keystore.
 * This variable is only needed outside of keystore during the
 * backup and migration proceedure.
 */
#define KEYSTORE_CLIENT_KEY_SIZE	32

/**
 * KEYSTORE_MAX_IV_SIZE - Maximum size of the Initialization Vector
 */
#define KEYSTORE_MAX_IV_SIZE	16

/**
 * enum keystore_seed_type - User/device seed type
 * @SEED_TYPE_DEVICE: The keys should be associated to the device.
 *                    SEED will only change if the device SEED is
 *                    compromised.
 * @SEED_TYPE_USER:   The keys should be associated to the user. The
 *                    SEED can be changed by the user if requested.
 */
enum keystore_seed_type {
	SEED_TYPE_DEVICE = 0,
	SEED_TYPE_USER   = 1
};

/**
 * enum keystore_key_spec - The key specification
 * @KEYSPEC_INVALID: Invalid keyspec
 * @KEYSPEC_LENGTH_128: 128-bit raw key (for AES)
 * @KEYSPEC_LENGTH_256: 256-bit raw key (for AES)
 * @KEYSPEC_LENGTH_ECC_PAIR: 1664-bit raw key pair (for ECC)
 */
enum keystore_key_spec {
	KEYSPEC_INVALID = 0,
	KEYSPEC_LENGTH_128 = 1,
	KEYSPEC_LENGTH_256 = 2,
	KEYSPEC_LENGTH_ECC_PAIR = 128,
};

/**
 * enum keystore_algo_spec - The encryption algorithm specification
 * @ALGOSPEC_INVALID: Invalid Algospec
 * @ALGOSPEC_AES_CCM: AES_CCM Algorithm (128/256 bit depending on key length)
 * @ALGOSPEC_AES_GCM: AES_GCM Algorithm (128/256 bit depending on key length)
 * @ALGOSPEC_ECIES: ECC/ECIES Encryption Algorithm (using secp521r1)
 * @ALGOSPEC_ECDSA: ECC/ECDSA Signature Algorithm (using secp521r1)
 */
enum keystore_algo_spec {
	ALGOSPEC_INVALID = 0,
	ALGOSPEC_AES_CCM = 1,
	ALGOSPEC_AES_GCM = 2,
	ALGOSPEC_ECIES = 128,
	ALGOSPEC_ECDSA = 129
};

#endif /* _KEYSTORE_API_COMMON_H_ */
