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

#ifndef KEYSTORE_CLIENT_ID
#define KEYSTORE_CLIENT_ID

#include <linux/types.h>

/**
 * DOC: Introduction
 *
 * The client operations include anything to do with user-space
 * client identification and (in the future) authentication.
 *
 */

/**
 * keystore_calc_clientid() - Derive the client ID
 *
 * @client_id: output array containing the client ID
 * @client_id_size: size of the output array
 * @timeout: timeout parameter for signed manifest caching
 * @caps: capabilities parameter for signed manifest verification
 *
 * Derive the client ID of a client from the path by
 * calculating SHA-256 on the directory name plus
 * file name.
 *
 * client_id = sha256( path_name + file_name )
 *
 * Returns: 0 if OK or negative error code.
 */
int keystore_calc_clientid(u8 *client_id, const unsigned int client_id_size);

#endif /* KEYSTORE_CLIENT_ID */
