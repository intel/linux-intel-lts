// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2016-2019, Intel Corporation.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uuid.h>

#include <linux/mei_cl_bus.h>
#include "dal_dev.h"

/* Spooler UUID */
static const uuid_t spooler_ta_id = UUID_INIT(0xba8d1643, 0x50b6, 0x49cc,
					      0x86, 0x1d, 0x2c, 0x01,
					      0xbe, 0xd1, 0x4b, 0xe8);

/**
 * struct dal_access_policy - ta access information node
 *
 * @list: link in access list
 * @ta_id: trusted application id
 * @owner: owner of ta
 */
struct dal_access_policy {
	struct list_head list;
	uuid_t ta_id;
	void *owner;
};

/**
 * dal_dev_get_access_list - get access list of dal device
 *
 * @ddev: dal device
 *
 * Return: pointer to access list
 */
static struct list_head *dal_dev_get_access_list(struct dal_device *ddev)
{
	return dev_get_drvdata(&ddev->dev);
}

/**
 * dal_access_policy_alloc - allocate memory and initialize access list node
 *
 * @ta_id: trusted application id
 * @owner: owner of ta
 *
 * Return: pointer to the new initialized access list node
 *
 * Locking: called under "kdi_lock" lock
 */
static struct dal_access_policy *
dal_access_policy_alloc(const uuid_t *ta_id, void *owner)
{
	struct dal_access_policy *e;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return NULL;

	INIT_LIST_HEAD(&e->list);
	e->ta_id = *ta_id;
	e->owner = owner;

	return e;
}

/**
 * dal_access_policy_find - find ta id in access list
 *
 * @access_list: access list
 * @ta_id: trusted application id
 *
 * Return: pointer to access list node of ta
 *         NULL if ta is not found in access list
 */
static struct dal_access_policy *
dal_access_policy_find(struct list_head *access_list, const uuid_t *ta_id)
{
	struct dal_access_policy *e;

	list_for_each_entry(e, access_list, list) {
		if (uuid_equal(&e->ta_id, ta_id))
			return e;
	}
	return NULL;
}

/**
 * dal_access_policy_add - add access information of ta and its owner
 *
 * @ddev: dal device
 * @ta_id: trusted application id
 * @owner: owner of ta
 *
 * Return: 0 on success
 *         -ENOMEM on memory allocation failure
 *         -EPERM when ta already has another owner
 *         -EEXIST when access information already exists (same ta and owner)
 *
 * Locking: called under "kdi_lock" lock
 */
int dal_access_policy_add(struct dal_device *ddev,
			  const uuid_t *ta_id, void *owner)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e;

	e = dal_access_policy_find(access_list, ta_id);
	if (e) {
		if (!e->owner)
			return -EPERM;

		return -EEXIST;
	}

	e = dal_access_policy_alloc(ta_id, owner);
	if (!e)
		return -ENOMEM;

	list_add_tail(&e->list, access_list);
	return 0;
}

/**
 * dal_access_policy_remove - remove access information of ta and its owner
 *
 * @ddev: dal device
 * @ta_id: trusted application id
 * @owner: owner of ta
 *
 * Return: 0 on success
 *         -ENOENT when ta isn't found in access list
 *         -EPERM when ta has another owner
 *
 * Locking: called under "kdi_lock" lock
 */
int dal_access_policy_remove(struct dal_device *ddev,
			     const uuid_t *ta_id, void *owner)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e;

	e = dal_access_policy_find(access_list, ta_id);
	if (!e)
		return -ENOENT;

	if (!e->owner || e->owner != owner)
		return -EPERM;

	list_del(&e->list);
	kfree(e);
	return 0;
}

/**
 * dal_access_policy_allowed - check if owner is allowed to use ta
 *
 * @ddev: dal device
 * @ta_id: trusted application id
 * @owner: owner
 *
 * Return: 0 on success
 *         -EPERM when owner is not allowed to use ta
 *
 * Locking: called under "ddev->write_lock" lock
 */
int dal_access_policy_allowed(struct dal_device *ddev,
			      const uuid_t *ta_id, void *owner)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e;

	e = dal_access_policy_find(access_list, ta_id);
	if (!e)
		return 0;

	if (e->owner && e->owner != owner)
		return -EPERM;

	return 0;
}

/**
 * dal_access_list_free - free memory of access list
 *
 * @ddev: dal device
 */
void dal_access_list_free(struct dal_device *ddev)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e, *n;

	if  (!access_list)
		return;

	list_for_each_entry_safe(e, n, access_list, list) {
		list_del(&e->list);
		kfree(e);
	}

	kfree(access_list);
	dev_set_drvdata(&ddev->dev, NULL);
}

/**
 * dal_access_list_init - initialize an empty access list
 *
 * @ddev: dal device
 *
 * Note: Add spooler ta id with blank owner to the list.
 * This will prevent any user from setting itself owner of the spooler,
 * which will block others from openning session to it.
 *
 * Return: 0 on success
 *         -ENOMEM on memory allocation failure
 */
int dal_access_list_init(struct dal_device *ddev)
{
	struct list_head *access_list;

	access_list = kzalloc(sizeof(*access_list), GFP_KERNEL);
	if (!access_list)
		return -ENOMEM;

	INIT_LIST_HEAD(access_list);
	dev_set_drvdata(&ddev->dev, access_list);

	/* Nobody can own SPOOLER TA */
	dal_access_policy_add(ddev, &spooler_ta_id, NULL);

	return 0;
}
