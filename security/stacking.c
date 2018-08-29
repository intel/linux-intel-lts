/*
 * Security secid functions
 *
 * Copyright (C) 2018 Casey Schaufler <casey@schaufler-ca.com>
 * Copyright (C) 2018 Intel
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */
#include <linux/security.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <net/netlabel.h>

/*
 * A secids structure contains all of the modules specific
 * secids and the secmark used to represent the combination
 * of module specific secids. Code that uses secmarks won't
 * know or care about module specific secids, and won't have
 * set them in the secids nor will it look at the module specific
 * values. Modules won't care about the secmark. If there's only
 * one module that uses secids the mapping is one-to-one. The
 * general case is not so simple.
 */

void secid_from_skb(struct secids *secid, const struct sk_buff *skb)
{
	struct secids *se;

	se = skb->sk->sk_security;
	if (se)
		*secid = *se;
}
EXPORT_SYMBOL(secid_from_skb);

void secid_to_skb(struct secids *secid, struct sk_buff *skb)
{
	struct secids *se;

	se = skb->sk->sk_security;
	if (se)
		*se = *secid;
}
EXPORT_SYMBOL(secid_to_skb);

bool secid_valid(const struct secids *secid)
{
#ifdef CONFIG_SECURITY_SELINUX
	if (secid->selinux)
		return true;
#endif
#ifdef CONFIG_SECURITY_SMACK
	if (secid->smack)
		return true;
#endif
	return false;
}

#ifdef CONFIG_NETLABEL
/**
 * lsm_sock_vet_attr - does the netlabel agree with what other LSMs want
 * @sk: the socket in question
 * @secattr: the desired netlabel security attributes
 * @flags: which LSM is making the request
 *
 * Determine whether the calling LSM can set the security attributes
 * on the socket without interferring with what has already been set
 * by other LSMs. The first LSM calling will always be allowed. An
 * LSM that resets itself will also be allowed. It will require careful
 * configuration for any other case to succeed.
 *
 * If @secattr is NULL the check is for deleting the attribute.
 *
 * Returns 0 if there is agreement, -EACCES if there is conflict,
 * and any error from the netlabel system.
 */
int lsm_sock_vet_attr(struct sock *sk, struct netlbl_lsm_secattr *secattr,
		      u32 flags)
{
	struct secids *se = sk->sk_security;
	struct netlbl_lsm_secattr asis;
	int rc;

	/*
	 * First in always shows as allowed.
	 * Changing what this module has set is OK, too.
	 */
	if (se->flags == 0 || se->flags == flags) {
		se->flags = flags;
		return 0;
	}

	netlbl_secattr_init(&asis);
	rc = netlbl_sock_getattr(sk, &asis);

	switch (rc) {
	case 0:
		/*
		 * Can't delete another modules's attributes or
		 * change them if they don't match well enough.
		 */
		if (secattr == NULL || !netlbl_secattr_equal(secattr, &asis))
			rc = -EACCES;
		else
			se->flags = flags;
		break;
	case -ENOMSG:
		se->flags = flags;
		rc = 0;
		break;
	default:
		break;
	}
	netlbl_secattr_destroy(&asis);
	return rc;
}
#endif /* CONFIG_NETLABEL */
