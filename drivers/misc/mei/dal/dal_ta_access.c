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

/* dal_access_list, list of TAs with access restriction */
static struct list_head dal_access_list;

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
	enum dal_intf owner;
};

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
dal_access_policy_alloc(const uuid_t *ta_id, enum dal_intf owner)
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
int dal_access_policy_add(const uuid_t *ta_id, enum dal_intf owner)
{
	struct dal_access_policy *e;

	e = dal_access_policy_find(&dal_access_list, ta_id);
	if (e) {
		if (e->owner != owner)
			return -EPERM;

		return -EEXIST;
	}

	e = dal_access_policy_alloc(ta_id, owner);
	if (!e)
		return -ENOMEM;

	list_add_tail(&e->list, &dal_access_list);
	return 0;
}

/**
 * dal_access_policy_remove - remove access information of ta and its owner
 *
 * @ta_id: trusted application id
 * @owner: owner of ta
 *
 * Return: 0 on success
 *         -ENOENT when ta isn't found in access list
 *         -EPERM when ta has another owner
 *
 * Locking: called under "kdi_lock" lock
 */
int dal_access_policy_remove(const uuid_t *ta_id, enum dal_intf owner)
{
	struct dal_access_policy *e;

	e = dal_access_policy_find(&dal_access_list, ta_id);
	if (!e)
		return -ENOENT;

	if (e->owner != owner)
		return -EPERM;

	list_del(&e->list);
	kfree(e);
	return 0;
}

/**
 * dal_access_policy_allowed - check if owner is allowed to use ta
 *
 * @ta_id: trusted application id
 * @owner: owner
 *
 * Return: 0 on success
 *         -EPERM when owner is not allowed to use ta
 *
 * Locking: called under "ddev->write_lock" lock
 */
int dal_access_policy_allowed(const uuid_t *ta_id, enum dal_intf owner)
{
	struct dal_access_policy *e;

	e = dal_access_policy_find(&dal_access_list, ta_id);
	if (!e)
		return 0;

	/*
	 * owner is DAL_INTF_MAX if the ta cannot be blocked
	 * (currently only the spooler ta)
	 */
	if (e->owner != DAL_INTF_MAX && e->owner != owner)
		return -EPERM;

	return 0;
}

/**
 * dal_access_list_free - free memory of access list
 */
void dal_access_list_free(void)
{
	struct dal_access_policy *e, *n;

	list_for_each_entry_safe(e, n, &dal_access_list, list) {
		list_del(&e->list);
		kfree(e);
	}

	INIT_LIST_HEAD(&dal_access_list);
}

/**
 * dal_access_list_init - initialize an empty access list
 *
 * Note: Add spooler ta id with blank owner to the list.
 * This will prevent any user from setting itself owner of the spooler,
 * which will block others from openning session to it.
 *
 * Return: 0 on success
 */
int dal_access_list_init(void)
{
	INIT_LIST_HEAD(&dal_access_list);

	/* Nobody can own SPOOLER TA */
	dal_access_policy_add(&spooler_ta_id, DAL_INTF_MAX);

	return 0;
}
