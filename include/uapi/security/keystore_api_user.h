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

#ifndef _KEYSTORE_API_USER_H_
#define _KEYSTORE_API_USER_H_

#include <linux/types.h>
#include <security/keystore_api_common.h>

/**
 * DOC: Introduction
 *
 * Keystore is a key-wrapping service running inside the kernel. It allows a
 * client to wrap (encrypt) secret keys for local storage on a filesystem or
 * other medium. Keystore also provides interfaces to perfrom cryptographic
 * operations with the wrapped keys.
 *
 */

/**
 * DOC: Configuration
 *
 * The following configuration options are related to keystore:
 *
 * KEYSTORE (default = n)
 * The option enables keystore.
 *
 * KEYSTORE_DEBUG (default = n)
 * The option enables additional debug prints, hexdumps during keystore
 * operation. It includes hexdump of sensitive cryptographic data structures
 * such as public keys and for this reason should not be enabled in production.
 */

/**
 * DOC: Key Wrapping
 *
 * The main function of keystore is to wrap keys (encrypt) from an application.
 * The application can safely store these wrapped keys on a filesystem.
 * Cryptographic operations on these keys are also performed within keystore
 * so that the bare keys are not exposed outside of the kernel.
 *
 * An application must first register with keystore by calling:
 *
 * keystore_register()
 *
 * The application can then either import or generate keys using the functions:
 *
 * keystore_generate_key() and keystore_wrap_key()
 *
 * The wrapped keys can be stored in non-volatile memory for future use.
 * Once a key has been wrapped, it can be loaded into a client "slot" where it
 * is internally wrapped:
 *
 * keystore_load_key()
 *
 * Following loading, data can be encrypted or decrypted using the key:
 *
 * keystore_encrypt() and keystore_decrypt()
 *
 * Finally, the slot can be freed and session ended using:
 *
 * keystore_unload_key() and keystore_unregister()
 *
 * For more details see the descriptions of the relevant functions.
 */

/**
 * DOC: Keystore Device
 *
 * Keystore is controlled from user-space via ioctl commands to the
 * /dev/keystore device.
 *
 */

/**
 * DOC: Keystore ioctl structs
 *
 * The keystore ioctls pass the following structs from user- to kernel-space.
 *
 */

/**
 * struct ias_keystore_version - The keystore version
 * @major: Major version number
 * @minor: Minor version number
 * @patch: Patch version number
 *
 * The keystore API version, following the Apache versioning system.
 *
 * Major versions represent large scale changes in the API.
 * Minor changes return API compatibility with older minor versions.
 * Patch changes are forwards and backwards compatible.
 */
struct ias_keystore_version {
	/* output */
	__u32 major;
	__u32 minor;
	__u32 patch;
};

/**
 * struct ias_keystore_register - Register a keystore client.
 * @seed_type:       Type of SEED to use for client key generation.
 * @client_ticket:   Ticket used to identify this client session.
 *
 * Register a keystore client. The client will be registered under an
 * internal client ID which is computed by taking the SHA-256 hash of
 * the absolute path name.
 *
 * On registration a client key is computed by combining the client ID
 * with either the device or user SEED using HMAC.
 *
 * The @seed_type determines whether the keys are wrapped using the
 * keystore device or user SEED. The choice depends on the type of
 * data being encrypted. Device keys will be used to encrypt data
 * associated with the device, whereas user keys are associated
 * to user data. The device SEED value can only be updated by the
 * device manufacturer, whereas the user SEED can be reset by a
 * system user.
 *
 * As the client ID is computed from the application path and name,
 * it is important to maintain the same path across releases.
 */
struct ias_keystore_register {
	/* input */
	enum keystore_seed_type seed_type;

	/* output */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
};

/**
 * struct ias_keystore_unregister - Unregister a keystore client..
 * @client_ticket:   Ticket used to identify this client session.
 */
struct ias_keystore_unregister {
	/* input */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
};

/**
 * struct ias_keystore_wrapped_key_size - Gets size of a wrapped key in bytes.
 * @key_spec:       The key type to get the size for.
 * @key_size:       The size of the wrapped key in bytes.
 * @unwrapped_key_size: Size of the unwrapped key.
 *
 * Returns the size of a wrapped key for a given key spec. This
 * should be called before a wrapped key is generated or imported
 * in order to allocate memory for the wrapped key buffer.
 *
 * The unwrapped key size will also be returned to be used when
 * importing exisiting keys or retrieving public keys.
 */
struct ias_keystore_wrapped_key_size {
	/* input */
	__u32 key_spec;

	/* output */
	__u32 key_size;
	__u32 unwrapped_key_size;
};

/**
 * struct ias_keystore_generate_key - Generate a keystore key
 * @client_ticket:   Ticket used to identify this client session
 * @key_spec:        Key type to be generated (enum keystore_key_spec)
 * @wrapped_key:     The generated key wrapped by keystore
 *
 * Keystore will generate a random key according to the given
 * key-spec and wrap it before returning.
 * The caller must ensure that wrapped_key points to a buffer with the
 * correct size for the given key_spec. This size can be calculated
 * by first calling the ias_wrapped_key_size.
 */
struct ias_keystore_generate_key {
	/* input */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
	__u32 key_spec;

	/* output */
	__u8 __user *wrapped_key;
};

/**
 * struct ias_keystore_wrap_key - Wrap an existing key
 * @client_ticket:   Ticket used to identify this client session
 * @key_spec:        Key type to be generated
 * @app_key:         The bare application key to be wrapped
 * @app_key_size:    Size of the bare key.
 * @wrapped_key:     The generated key wrapped by keystore
 *
 * Keystore checks the key spec and size before wrapping it.
 * The caller must ensure that wrapped_key points to a buffer with the
 * correct size for the given key_spec. This size can be calculated
 * by first calling the %KEYSTORE_IOC_WRAPPED_KEYSIZE ioctl.
 *
 * Keys are wrapped using the AES-SIV algorithm. AES-SIV was chosen
 * as it does not require an Initialisation Vector.
 */
struct ias_keystore_wrap_key {
	/* input */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
	__u32 key_spec;
	const __u8 __user *app_key;
	__u32 app_key_size;

	/* output */
	__u8 __user *wrapped_key;
};

/**
 * struct ias_keystore_load_key - Load a key into a slot
 * @client_ticket:    Ticket used to identify this client session
 * @wrapped_key:      The wrapped key
 * @wrapped_key_size: Size of the wrapped key
 * @slot_id:          The assigned slot
 *
 * Loads a wrapped key into the next available slot for
 * this client ticket.
 */
struct ias_keystore_load_key {
	/* input */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
	__u8 __user *wrapped_key;
	__u32 wrapped_key_size;

	/* output */
	__u32 slot_id;
};

/**
 * struct ias_keystore_unload_key - Unload a key from a slot
 * @client_ticket:    Ticket used to identify this client session
 * @slot_id:          The assigned slot
 *
 * Unloads a key from the given slot.
 */
struct ias_keystore_unload_key {
	/* input */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
	__u32 slot_id;
};

/**
 * struct ias_keystore_crypto_size - Get the size of output buffer.
 * @algospec:    The encryption algorithm to use.
 * @input_size:  Size of the input buffer.
 * @output_size: Size of the output buffer.
 *
 * This struct is used by the client to calculate the required size of
 * an output buffer passed to the Keystore encrypt and decrypt operations.
 */
struct ias_keystore_crypto_size {
	/* input */
	__u32 algospec;
	__u32 input_size;

	/* output */
	__u32 output_size;
};

/**
 * struct ias_keystore_encrypt_decrypt - Encrypt or Decrypt using a loaded key
 * @client_ticket:    Ticket used to identify this client session
 * @slot_id:          The assigned slot
 * @algospec:         The encryption algorithm to use
 * @iv:               The initialisation vector (IV)
 * @iv_size:          Size of the IV.
 * @input:            Pointer to the cleartext input
 * @input_size:       Size of the input data
 * @output:           Pointer to an output buffer
 *
 * Encrypt a block of data using the key stored in the given slot.
 * The caller must assure that the output points to a buffer with
 * at enough space. The correct size can be calculated by calling
 * ias_keystore_crypto_size.
 */
struct ias_keystore_encrypt_decrypt {
	/* input */
	__u8 client_ticket[KEYSTORE_CLIENT_TICKET_SIZE];
	__u32 slot_id;
	__u32 algospec;
	const __u8 __user *iv;
	__u32 iv_size;
	const __u8 __user *input;
	__u32 input_size;

	/* output */
	__u8 __user *output;  /* notice: pointer */
};

/**
 * DOC: Keystore IOCTLs
 *
 * A list of the keystore ioctls can be found here. Each ioctl
 * is more or less mapped to a corresponding function in
 * keystore_api_kernel.h.
 *
 * Although documented as functions, the ioctls are preprocessor
 * defines to be used in the ioctl() function.
 *
 */

#define KEYSTORE_IOC_MAGIC '7'

/**
 * KEYSTORE_IOC_VERSION - Keystore version
 *
 * Returns the keystore version in a &struct ias_keystore_version
 */
#define KEYSTORE_IOC_VERSION\
	_IOR(KEYSTORE_IOC_MAGIC,   0, struct ias_keystore_version)

/**
 * KEYSTORE_IOC_REGISTER - Register a client with keystore
 *
 * Calls the keystore_register() function with &struct ias_keystore_register.
 */
#define KEYSTORE_IOC_REGISTER\
	_IOWR(KEYSTORE_IOC_MAGIC,  1, struct ias_keystore_register)

/**
 * KEYSTORE_IOC_REGISTER - Register a client with keystore
 *
 * Calls the keystore_unregister() function with
 * &struct ias_keystore_unregister.
 */
#define KEYSTORE_IOC_UNREGISTER\
	_IOW(KEYSTORE_IOC_MAGIC,   2, struct ias_keystore_unregister)

/**
 * KEYSTORE_IOC_WRAPPED_KEYSIZE - Gets the wrapped keysize for a given key.
 *
 * Calls the keystore_wrapped_key_size() function with
 * &struct ias_keystore_wrapped_key_size.
 */
#define KEYSTORE_IOC_WRAPPED_KEYSIZE\
	_IOWR(KEYSTORE_IOC_MAGIC,  3, struct ias_keystore_wrapped_key_size)

/**
 * KEYSTORE_IOC_GENERATE_KEY - Generate a random key and wrap it.
 *
 * Calls the keystore_generate_key() function with
 * &struct ias_keystore_generate_key.
 */
#define KEYSTORE_IOC_GENERATE_KEY\
	_IOW(KEYSTORE_IOC_MAGIC,  4, struct ias_keystore_generate_key)

/**
 * KEYSTORE_IOC_WRAP_KEY - Wrap the application key.
 *
 * Calls the keystore_wrap_key() function with
 * &struct ias_keystore_wrap_key.
 */
#define KEYSTORE_IOC_WRAP_KEY\
	_IOW(KEYSTORE_IOC_MAGIC,  5, struct ias_keystore_wrap_key)

/**
 * KEYSTORE_IOC_LOAD_KEY - Unwrap the application key and store in a slot.
 *
 * Calls the keystore_load_key() function with
 * &struct ias_keystore_load_key.
 */
#define KEYSTORE_IOC_LOAD_KEY\
	_IOWR(KEYSTORE_IOC_MAGIC,   6, struct ias_keystore_load_key)

/**
 * KEYSTORE_IOC_UNLOAD_KEY - Remove a key from a slot
 *
 * Calls the keystore_unload_key() function with
 * &struct ias_keystore_unload_key.
 */
#define KEYSTORE_IOC_UNLOAD_KEY\
	_IOW(KEYSTORE_IOC_MAGIC,   7, struct ias_keystore_unload_key)

/**
 * KEYSTORE_IOC_ENCRYPT_SIZE - Get the required size of an encrypted buffer.
 *
 * Calls the keystore_encrypt_size() function with
 * &struct ias_keystore_crypto_size.
 */
#define KEYSTORE_IOC_ENCRYPT_SIZE\
	_IOWR(KEYSTORE_IOC_MAGIC,   8, struct ias_keystore_crypto_size)

/**
 * KEYSTORE_IOC_ENCRYPT - Encrypt plaintext using AppKey/IV according to
 *                        AlgoSpec.
 *
 * Calls the keystore_encrypt() function with
 * &struct ias_keystore_encrypt_decrypt.
 */
#define KEYSTORE_IOC_ENCRYPT\
	_IOW(KEYSTORE_IOC_MAGIC,   9, struct ias_keystore_encrypt_decrypt)

/**
 * KEYSTORE_IOC_DECRYPT_SIZE - Get the required size of an decrypted buffer.
 *
 * Calls the keystore_decrypt_size() function with
 * &struct ias_keystore_crypto_size.
 */
#define KEYSTORE_IOC_DECRYPT_SIZE\
	_IOWR(KEYSTORE_IOC_MAGIC,  10, struct ias_keystore_crypto_size)

/**
 * KEYSTORE_IOC_DECRYPT - Decrypt cyphertext using AppKey/IV according to
 *                        AlgoSpec.
 *
 * Calls the keystore_decrypt() function with
 * &struct ias_keystore_encrypt_decrypt.
 */
#define KEYSTORE_IOC_DECRYPT\
	_IOW(KEYSTORE_IOC_MAGIC,  11, struct ias_keystore_encrypt_decrypt)

#endif /* _KEYSTORE_API_USER_H_ */
