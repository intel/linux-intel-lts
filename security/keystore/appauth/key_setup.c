// SPDX-License-Identifier: GPL-2.0
// Intel Keystore Linux driver, Copyright (c) 2018, Intel Corporatio.
#include "app_auth.h"

#ifdef CONFIG_KEYSTORE_DEBUG
/**
 * Prints the public key in hex form.
 *
 * @param key    - public key to be printed.
 *
 */
void debug_public_key(struct public_key *key)
{
	ks_debug("DEBUG_APPAUTH: Public key\n");
	keystore_hexdump("", key->key, key->keylen);
}
#else
void debug_public_key(struct public_key *key)
{
}
#endif



/**
 * Provides the process file path.
 *
 * @param buf    -  The buffer to store the file path.
 * @param buflen -  The size of the buffer.
 *
 * @returns the address of the file path.
 */
static char *get_exe_path(char *buf, int buflen)
{
	struct file *exe_file = NULL;
	char *result = NULL;
	struct mm_struct *mm = NULL;

	mm = get_task_mm(current);
	if (!mm) {
		ks_info(KBUILD_MODNAME ": %s error get_task_mm\n", __func__);
		goto out;
	}

	down_read(&mm->mmap_sem);
	exe_file = mm->exe_file;

	if (exe_file)
		path_get(&exe_file->f_path);

	up_read(&mm->mmap_sem);
	mmput(mm);
	if (exe_file) {
		result = d_path(&exe_file->f_path, buf, buflen);
		path_put(&exe_file->f_path);
	}
out:
	return result;
}

/**
 * Provides the process file path.
 * Performs error check on the retrieved path.
 *
 * @param buf    -  The buffer to store the file path.
 *
 * @returns the address of the file path if success or nul otherwise.
 */
char *get_exe_name(char **buf)
{
	char *f_path = NULL;

	/* alloc mem for pwd##app, use linux defined limits */
	*buf = kmalloc(PATH_MAX + NAME_MAX, GFP_KERNEL);
	if (*buf != NULL) {
		/* clear the buf */
		memset(*buf, 0, PATH_MAX + NAME_MAX);

		f_path = get_exe_path(*buf,
					(PATH_MAX + NAME_MAX));

		if (!f_path)
			return NULL;

		/* check if it was an error */
		if (f_path && IS_ERR(f_path)) {
			/* error case, do not register */
			ks_err(KBUILD_MODNAME "%s failed\n", __func__);
			return NULL;
		}
		ks_debug(KBUILD_MODNAME "absolute path of exe: %s\n", f_path);
	} else
		return NULL;

	return f_path;
}
