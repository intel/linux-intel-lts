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

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <security/keystore_api_user.h>

#include "keystore_debug.h"
#include "api_dal.h"

/* Universal operation data union */
union keystore_ops_union {
	struct ias_keystore_version version;
	struct ias_keystore_register register_client_type;
	struct ias_keystore_unregister unregister_client;
	struct ias_keystore_wrapped_key_size wrapped_key_size;
	struct ias_keystore_generate_key generate_key;
	struct ias_keystore_wrap_key wrap_key;
	struct ias_keystore_load_key load_key;
	struct ias_keystore_unload_key unload_key;
	struct ias_keystore_crypto_size crypto_size;
	struct ias_keystore_encrypt_decrypt encrypt_decrypt;
};

static unsigned int is_cmd_supported(unsigned int cmd)
{
	switch (cmd) {
	case KEYSTORE_IOC_VERSION:
	case KEYSTORE_IOC_REGISTER:
	case KEYSTORE_IOC_UNREGISTER:
	case KEYSTORE_IOC_WRAPPED_KEYSIZE:
	case KEYSTORE_IOC_GENERATE_KEY:
	case KEYSTORE_IOC_WRAP_KEY:
	case KEYSTORE_IOC_LOAD_KEY:
	case KEYSTORE_IOC_UNLOAD_KEY:
	case KEYSTORE_IOC_ENCRYPT_SIZE:
	case KEYSTORE_IOC_ENCRYPT:
	case KEYSTORE_IOC_DECRYPT_SIZE:
	case KEYSTORE_IOC_DECRYPT:
		return 1;
	default:
		return 0;
	}
}

#if defined(CONFIG_KEYSTORE_DEBUG)
static unsigned char *getcmdstr(unsigned int cmd)
{
	switch (cmd) {
	case KEYSTORE_IOC_VERSION:
		return "KEYSTORE_IOC_VERSION";
	case KEYSTORE_IOC_REGISTER:
		return "KEYSTORE_IOC_REGISTER";
	case KEYSTORE_IOC_UNREGISTER:
		return "KEYSTORE_IOC_UNREGISTER";
	case KEYSTORE_IOC_WRAPPED_KEYSIZE:
		return "KEYSTORE_IOC_WRAPPED_KEYSIZE";
	case KEYSTORE_IOC_GENERATE_KEY:
		return "KEYSTORE_IOC_GENERATE_KEY";
	case KEYSTORE_IOC_WRAP_KEY:
		return "KEYSTORE_IOC_WRAP_KEY";
	case KEYSTORE_IOC_LOAD_KEY:
		return "KEYSTORE_IOC_LOAD_KEY";
	case KEYSTORE_IOC_UNLOAD_KEY:
		return "KEYSTORE_IOC_UNLOAD_KEY";
	case KEYSTORE_IOC_ENCRYPT_SIZE:
		return "KEYSTORE_IOC_ENCRYPT_SIZE";
	case KEYSTORE_IOC_ENCRYPT:
		return "KEYSTORE_IOC_ENCRYPT";
	case KEYSTORE_IOC_DECRYPT_SIZE:
		return "KEYSTORE_IOC_DECRYPT_SIZE";
	case KEYSTORE_IOC_DECRYPT:
		return "KEYSTORE_IOC_DECRYPT";
	default:
		return "not-supported";
	}
}
#endif

static int version_op(struct ias_keystore_version *user_data)
{
	if (!user_data)
		return -EFAULT;

	user_data->major = KEYSTORE_VERSION_MAJOR;
	user_data->minor = KEYSTORE_VERSION_MINOR;
	user_data->patch = KEYSTORE_VERSION_PATCH;

	return 0;
}

static int register_op(struct ias_keystore_register *user_data)
{

	if (!user_data)
		return -EFAULT;

	return dal_keystore_register(user_data->seed_type,
		user_data->client_ticket);

}

static int unregister_op(struct ias_keystore_unregister *user_data)
{
	if (!user_data)
		return -EFAULT;

	return dal_keystore_unregister(user_data->client_ticket);
}

static int wrapped_keysize_op(struct ias_keystore_wrapped_key_size *user_data)
{
	if (!user_data)
		return -EFAULT;

	return dal_keystore_wrapped_key_size(user_data->key_spec,
		&user_data->key_size,
		&user_data->unwrapped_key_size);
}

static int generate_key_op(struct ias_keystore_generate_key *user_data)
{
	int res = 0;
	unsigned int wrapped_key_size = 0;
	uint8_t *wrapped_key = NULL;

	if (!user_data)
		return -EFAULT;

	res = dal_keystore_wrapped_key_size(user_data->key_spec,
			&wrapped_key_size, NULL);
	if (res)
		return -EINVAL;

	wrapped_key = kmalloc(wrapped_key_size, GFP_KERNEL);
	if (!wrapped_key)
		return -ENOMEM;

	res = dal_keystore_generate_key(user_data->client_ticket,
		user_data->key_spec, wrapped_key);
	if (res)
		goto free_buf;

	res = copy_to_user(user_data->wrapped_key, wrapped_key,
			   wrapped_key_size);

free_buf:
	kzfree(wrapped_key);
	return res;
}

static int wrap_key_op(struct ias_keystore_wrap_key *user_data)
{
	int res = 0;
	uint8_t *wrapped_key = NULL;
	uint8_t *app_key = NULL;
	unsigned int wrapped_key_size;
	unsigned int unwrapped_key_size;

	if (!user_data)
		return -EFAULT;

	res = dal_keystore_wrapped_key_size(user_data->key_spec,
			&wrapped_key_size, &unwrapped_key_size);
	if (res)
		return -EINVAL;

	wrapped_key = kmalloc(wrapped_key_size, GFP_KERNEL);
	app_key = kmalloc(user_data->app_key_size, GFP_KERNEL);
	if (!wrapped_key || !app_key) {
		res = -ENOMEM;
		goto free_buf;
	}

	res = copy_from_user(app_key, user_data->app_key,
			     user_data->app_key_size);

	res = dal_keystore_wrap_key(user_data->client_ticket,
			app_key,
			user_data->app_key_size,
			user_data->key_spec,
			wrapped_key);
	if (res)
		goto free_buf;

	res = copy_to_user(user_data->wrapped_key, wrapped_key,
			   wrapped_key_size);

free_buf:
	kzfree(wrapped_key);
	kzfree(app_key);
	return res;
}

static int load_key_op(struct ias_keystore_load_key *user_data)
{
	int res = 0;
	uint8_t *wrapped_key = NULL;

	if (!user_data)
		return -EFAULT;

	wrapped_key = kmalloc(user_data->wrapped_key_size, GFP_KERNEL);
	if (!wrapped_key)
		return -ENOMEM;

	res = copy_from_user(wrapped_key,
			     user_data->wrapped_key,
			     user_data->wrapped_key_size);
	if (res) {
		res = -EINVAL;
		goto free_buf;
	}

	res = dal_keystore_load_key(user_data->client_ticket,
			wrapped_key,
			user_data->wrapped_key_size,
			&user_data->slot_id);

	if (res == -EAGAIN) {
		res = copy_to_user(user_data->wrapped_key, wrapped_key,
				user_data->wrapped_key_size);
		if (res == 0)
			res = -EAGAIN;
	}

free_buf:
	kzfree(wrapped_key);
	return res;
}

static int unload_key_op(struct ias_keystore_unload_key *user_data)
{
	int res = 0;

	if (!user_data)
		return -EFAULT;

	res = dal_keystore_unload_key(user_data->client_ticket,
			user_data->slot_id);

	return res;
}

static int encrypt_size_op(struct ias_keystore_crypto_size *user_data)
{
	if (!user_data)
		return -EFAULT;

	return dal_keystore_encrypt_size(user_data->algospec,
		user_data->input_size, &user_data->output_size);
}

static int encrypt_op(struct ias_keystore_encrypt_decrypt *user_data)
{
	int res;
	unsigned int output_size = 0;
	uint8_t *iv = NULL;
	uint8_t *input = NULL;
	uint8_t *output = NULL;

	if (!user_data || !user_data->input || !user_data->output)
		return -EFAULT;

	if (!user_data->input_size)
		return -EINVAL;

	res = dal_keystore_encrypt_size(user_data->algospec,
			user_data->input_size, &output_size);
	if (res)
		return res;

	input = kmalloc(user_data->input_size, GFP_KERNEL);
	output = kmalloc(output_size, GFP_KERNEL);
	if (!input || !output) {
		res = -ENOMEM;
		goto free_buf;
	}

	res = copy_from_user(input, user_data->input, user_data->input_size);
	if (res)
		goto free_buf;

	if (user_data->iv && user_data->iv_size > 0) {
		iv = kmalloc(user_data->iv_size, GFP_KERNEL);
		if (!iv) {
			res = -ENOMEM;
			goto free_buf;
		}

		res = copy_from_user(iv, user_data->iv, user_data->iv_size);
		if (res)
			goto free_buf;
	}

	res = dal_keystore_encrypt(user_data->client_ticket,
			user_data->slot_id,
			user_data->algospec, iv,
			user_data->iv_size,
			input, user_data->input_size, output);

	if (res)
		goto free_buf;

	res = copy_to_user(user_data->output, output, output_size);

free_buf:
	kzfree(input);
	kzfree(output);
	kzfree(iv);

	return res;
}

static int decrypt_size_op(struct ias_keystore_crypto_size *user_data)
{
	if (!user_data)
		return -EFAULT;

	return dal_keystore_decrypt_size(user_data->algospec,
		user_data->input_size, &user_data->output_size);
}

static int decrypt_op(struct ias_keystore_encrypt_decrypt *user_data)
{
	int res;
	unsigned int output_size = 0;
	uint8_t *iv = NULL;
	uint8_t *input = NULL;
	uint8_t *output = NULL;

	if (!user_data || !user_data->input || !user_data->output)
		return -EFAULT;

	if (!user_data->input_size)
		return -EINVAL;

	res = dal_keystore_decrypt_size(user_data->algospec,
		user_data->input_size, &output_size);
	if (res)
		return res;

	input = kmalloc(user_data->input_size, GFP_KERNEL);
	output = kmalloc(output_size, GFP_KERNEL);
	if (!input || !output) {
		res = -ENOMEM;
		goto free_buf;
	}

	res = copy_from_user(input, user_data->input, user_data->input_size);
	if (res)
		goto free_buf;

	if (user_data->iv && user_data->iv_size > 0) {
		iv = kmalloc(user_data->iv_size, GFP_KERNEL);
		if (!iv) {
			res = -ENOMEM;
			goto free_buf;
		}

		res = copy_from_user(iv, user_data->iv, user_data->iv_size);
		if (res)
			goto free_buf;
	}

	res = dal_keystore_decrypt(user_data->client_ticket,
		user_data->slot_id, user_data->algospec, iv,
		user_data->iv_size,
		input, user_data->input_size, output);

	if (res)
		goto free_buf;

	res = copy_to_user(user_data->output, output, output_size);

free_buf:
	kzfree(input);
	kzfree(output);
	kzfree(iv);

	return res;
}

/**
 * keystore_ioctl - the ioctl function
 *
 * @param file pointer to file structure
 * @param cmd command to execute
 * @param arg user space pointer to arguments structure
 *
 * @returns 0 on success, <0 on error
 */
long keystore_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	union keystore_ops_union op;
	unsigned int size;
	int res = 0;

	FUNC_BEGIN;

	ks_debug(KBUILD_MODNAME ": %s - cmd = 0x%x - %s\n",
		 __func__, cmd, getcmdstr(cmd));

	/* copy data structure from user space to system */
	size = _IOC_SIZE(cmd);

	if (!is_cmd_supported(cmd) ||
	    (size > sizeof(union keystore_ops_union))) {
		ks_err(KBUILD_MODNAME ": %s - cmd=%u unknown, size:%d, size(keystore_ops_union): %zu\n",
		       __func__, cmd, size, sizeof(union keystore_ops_union));
		return -ENOIOCTLCMD;
	}
	if (!access_ok(VERIFY_READ, (void *)arg, size)) {
		ks_err(KBUILD_MODNAME ": %s - cmd=%u no read access\n",
		       __func__, cmd);
		return -EFAULT;
	}
	if ((_IOC_DIR(cmd) & _IOC_READ) &&
	    (!access_ok(VERIFY_WRITE, (void *)arg, size))) {
		ks_err(KBUILD_MODNAME ": %s - cmd=%u no write access\n",
		       __func__, cmd);
		return -EFAULT;
	}
	if (copy_from_user(&op, (void *)arg, size)) {
		ks_err(KBUILD_MODNAME ": %s - cannot copy data\n",
		       __func__);
		return -EFAULT;
	}

	switch (cmd) {
	case KEYSTORE_IOC_VERSION:
		res = version_op(&op.version);
	case KEYSTORE_IOC_REGISTER:
		res = register_op(&op.register_client_type);
		break;

	case KEYSTORE_IOC_UNREGISTER:
		res = unregister_op(&op.unregister_client);
		break;

	case KEYSTORE_IOC_WRAPPED_KEYSIZE:
		res = wrapped_keysize_op(&op.wrapped_key_size);
		break;

	case KEYSTORE_IOC_GENERATE_KEY:
		res = generate_key_op(&op.generate_key);
		break;

	case KEYSTORE_IOC_WRAP_KEY:
		res = wrap_key_op(&op.wrap_key);
		break;

	case KEYSTORE_IOC_LOAD_KEY:
		res = load_key_op(&op.load_key);
		break;

	case KEYSTORE_IOC_UNLOAD_KEY:
		res = unload_key_op(&op.unload_key);
		break;

	case KEYSTORE_IOC_ENCRYPT_SIZE:
		res = encrypt_size_op(&op.crypto_size);
		break;

	case KEYSTORE_IOC_ENCRYPT:
		res = encrypt_op(&op.encrypt_decrypt);
		break;

	case KEYSTORE_IOC_DECRYPT_SIZE:
		res = decrypt_size_op(&op.crypto_size);
		break;

	case KEYSTORE_IOC_DECRYPT:
		res = decrypt_op(&op.encrypt_decrypt);
		break;
	default:
		res = -ENOIOCTLCMD;
		ks_err(KBUILD_MODNAME ": %s - cmd=%u not known\n",
		       __func__, cmd);
		break;
	}

	/* check if OK and we need to return some data in memory block */
	if ((res >= 0) && (_IOC_DIR(cmd) & _IOC_READ)) {
		/* then copy data from system to user space */
		if (copy_to_user((void *)arg, &op, size)) {
			ks_err(KBUILD_MODNAME ": %s - cannot copy data\n",
			       __func__);
			res = -EFAULT;
		}
	}

	memset(&op, 0, sizeof(op));
	FUNC_RES(res);
	return res;
}
