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

static struct kctl_wrapper *kctl_find_by_name(
		struct kctl_proxy *proxy, const char *kcontrol_name)
{
	struct kctl_wrapper *kwrapper;
	struct snd_kcontrol *kcontrol;

	list_for_each_entry(kwrapper, &proxy->kcontrols_list, list) {
		kcontrol = kwrapper->kcontrol;
		if (strncmp(kcontrol->id.name, kcontrol_name,
			ARRAY_SIZE(kcontrol->id.name)) == 0)
			return kwrapper;
	}
	return NULL;
}

void kctl_init_proxy(struct device *dev, struct kctl_ops *kt_ops)
{
	struct kctl_proxy *proxy = get_kctl_proxy();

	INIT_LIST_HEAD(&proxy->kcontrols_list);
	proxy->ops = *kt_ops;
	proxy->alloc_dev = dev;
}

struct kctl_wrapper *kctl_find_by_address(struct kctl_proxy *proxy,
	struct snd_kcontrol *kcontrol)
{
	struct kctl_wrapper *kwrapper;

	list_for_each_entry(kwrapper, &proxy->kcontrols_list, list) {
		if (kcontrol == kwrapper->kcontrol)
			return kwrapper;
	}
	return NULL;
}

int kctl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct vfe_kctl_result result;
	struct kctl_proxy *proxy = get_kctl_proxy();
	struct kctl_wrapper *vfe_kcontrol =
			kctl_find_by_address(proxy, kcontrol);
	int ret;

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
	struct kctl_wrapper *vfe_kcontrol = devm_kzalloc(proxy->alloc_dev,
		sizeof(*vfe_kcontrol), GFP_KERNEL);

	if (!vfe_kcontrol)
		return -ENOMEM;

	vfe_kcontrol->kcontrol = kcontrol;
	vfe_kcontrol->put = kcontrol->put;
	kcontrol->put = kctl_put;

	list_add(&vfe_kcontrol->list, &proxy->kcontrols_list);
	return 0;
}

void kctl_notify_machine_ready(struct snd_soc_card *card)
{
	struct snd_kcontrol *kctl;
	struct kctl_proxy *proxy = get_kctl_proxy();
	int ret;

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
		kctl_find_by_name(proxy, kctl_info->kcontrol_id);
	int ret;

	if (!kcontrol) {
		dev_err(proxy->alloc_dev, "Can not find kcontrol [%s].\n",
				kctl_info->kcontrol_id);
		ret = -ENODEV;
		goto ret_result;
	}

	ret = proxy->ops.check_permission(domain_id, kcontrol->kcontrol);
	if (ret < 0)
		goto ret_result;

	if (kcontrol->put)
		ret = kcontrol->put(kcontrol->kcontrol, &kcontrol_val->value);

ret_result:
	if (result)
		result->ret = ret;

	return ret;
}
