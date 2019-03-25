// SPDX-License-Identifier: GPL-2.0+
//
// skl-virtio-kctl.c  -- kcontrol synchronization mechanism
//
// Copyright (C) 2019 Intel Corporation.
//
// Authors: Furtak, Pawel <pawel.furtak@intel.com>
//          Janca, Grzegorz <grzegorz.janca@intel.com>
//          Jablonski, Wojciech <wojciech.jablonski@intel.com>
//
// Synchronization of kcontrols between GOS and SOS by wrapping put callback
// with function that forwards kcontrol name and value to the other OS GOS/SOS

#include <linux/vmalloc.h>
#include <linux/device.h>
#include "skl-virtio-common.h"

static struct kctl_proxy ctl_proxy;

static struct kctl_proxy *get_kctl_proxy(void)
{
	return &ctl_proxy;
}

static struct kctl_domain *find_domain(struct kctl_proxy *proxy,
		u32 domain_id)
{
	struct kctl_domain *domain;

	list_for_each_entry(domain, &proxy->domain_list, list) {
		if (domain->domain_id == domain_id)
			return domain;
	}
	return NULL;
}

static struct kctl_domain *get_domain(struct kctl_proxy *proxy,
		u32 domain_id)
{
	struct kctl_domain *domain = find_domain(proxy, domain_id);

	if (domain == NULL) {
		domain = devm_kzalloc(proxy->alloc_dev,
				sizeof(*domain), GFP_KERNEL);

		if (!domain)
			return NULL;

		domain->domain_id = domain_id;
		list_add(&domain->list, &proxy->domain_list);
		INIT_LIST_HEAD(&domain->kcontrols_list);
	}

	return domain;
}

static struct kctl_wrapper *kctl_find_by_name(struct kctl_proxy *proxy,
		u32 domain_id, const char *kcontrol_name)
{
	struct kctl_domain *domain = find_domain(proxy, domain_id);
	struct kctl_wrapper *kwrapper;
	struct snd_kcontrol *kcontrol;

	if (domain) {
		list_for_each_entry(kwrapper, &domain->kcontrols_list, list) {
			kcontrol = kwrapper->kcontrol;
			if (strncmp(kcontrol->id.name, kcontrol_name,
				ARRAY_SIZE(kcontrol->id.name)) == 0)
				return kwrapper;
		}
	}
	return NULL;
}

void kctl_init_proxy(struct device *dev, struct kctl_ops *kt_ops)
{
	struct kctl_proxy *proxy = get_kctl_proxy();

	INIT_LIST_HEAD(&proxy->domain_list);
	proxy->ops = *kt_ops;
	proxy->alloc_dev = dev;
}

struct kctl_wrapper *kctl_find_by_address(struct kctl_proxy *proxy,
		u32 domain_id, struct snd_kcontrol *kcontrol)
{
	struct kctl_domain *domain = find_domain(proxy, domain_id);
	struct kctl_wrapper *kwrapper;

	if (domain) {
		list_for_each_entry(kwrapper, &domain->kcontrols_list, list) {
			if (kcontrol == kwrapper->kcontrol)
				return kwrapper;
		}
	}
	return NULL;
}

int kctl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	u32 domain_id;
	struct vfe_kctl_result result;
	struct kctl_proxy *proxy = get_kctl_proxy();
	struct kctl_wrapper *vfe_kcontrol;
	int ret;

	ret = proxy->ops.get_domain_id(kcontrol, &domain_id);
	if (ret != 0)
		return ret;

	vfe_kcontrol = kctl_find_by_address(proxy, domain_id, kcontrol);

	if (!vfe_kcontrol)
		return -EPERM;

	ret = proxy->ops.send_noti(kcontrol, ucontrol, &result);
	if (ret < 0)
		return ret;

	if (result.ret < 0)
		return result.ret;

	if (vfe_kcontrol->put)
		ret = vfe_kcontrol->put(kcontrol, ucontrol);

	return ret;
}

int kctl_wrap_kcontrol(struct kctl_proxy *proxy,
		struct snd_kcontrol *kcontrol)
{
	u32 domain_id;
	struct kctl_wrapper *vfe_kcontrol;
	struct kctl_domain *domain;
	int ret = proxy->ops.get_domain_id(kcontrol, &domain_id);

	if ((ret != 0) || (domain_id == 0))
		return ret;

	domain = get_domain(proxy, domain_id);

	if (!domain)
		return -ENOMEM;

	vfe_kcontrol = devm_kzalloc(proxy->alloc_dev,
		sizeof(*vfe_kcontrol), GFP_KERNEL);

	if (!vfe_kcontrol)
		return -ENOMEM;

	vfe_kcontrol->kcontrol = kcontrol;
	vfe_kcontrol->put = kcontrol->put;
	kcontrol->put = kctl_put;
	//kcontrol->id.device = domain_id;

	list_add(&vfe_kcontrol->list, &domain->kcontrols_list);
	return 0;
}

static void kctl_clean_list(struct list_head *kcontrols_list,
		struct device *alloc_dev)
{
	struct kctl_wrapper *kwrapper;

	while (!list_empty(kcontrols_list)) {
		kwrapper = list_first_entry(kcontrols_list,
				struct kctl_wrapper, list);
		list_del(&kwrapper->list);
		devm_kfree(alloc_dev, kwrapper);

	}
}

static void kctl_clean_domain_list(struct kctl_proxy *proxy)
{
	struct kctl_domain *domain;

	while (!list_empty(&proxy->domain_list)) {
		domain = list_first_entry(&proxy->domain_list,
				struct kctl_domain, list);
		kctl_clean_list(&domain->kcontrols_list, proxy->alloc_dev);
		list_del(&domain->kcontrols_list);
		devm_kfree(proxy->alloc_dev, domain);

	}
}

void kctl_notify_machine_ready(struct snd_soc_card *card)
{
	struct snd_kcontrol *kctl;
	struct kctl_proxy *proxy = get_kctl_proxy();
	int ret;

	//to be sure if lis is empty
	kctl_clean_domain_list(proxy);

	list_for_each_entry(kctl, &card->snd_card->controls, list) {
		ret = kctl_wrap_kcontrol(proxy, kctl);
		if (ret < 0)
			return;
	}
}
EXPORT_SYMBOL(kctl_notify_machine_ready);


int kctl_ipc_handle(u32 domain_id, const struct vfe_kctl_info *kctl_info,
	struct vfe_kctl_value *kcontrol_val, struct vfe_kctl_result *result)
{
	struct kctl_proxy *proxy = get_kctl_proxy();
	struct kctl_wrapper *kcontrol =
		kctl_find_by_name(proxy, domain_id, kctl_info->kcontrol_id);
	int ret = 0;

	if (!kcontrol) {
		dev_err(proxy->alloc_dev,
				"Can not find kcontrol [name=\"%s\", domain_id=%u].\n",
				kctl_info->kcontrol_id, domain_id);
		ret = -EPERM;
		goto ret_result;
	}

	if (kcontrol->put)
		ret = kcontrol->put(kcontrol->kcontrol, &kcontrol_val->value);

ret_result:
	if (result)
		result->ret = ret;

	return ret;
}
