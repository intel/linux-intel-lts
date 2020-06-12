// SPDX-License-Identifier: GPL-2.0+
//
// skl-virtio-miscdev.c  --  Miscellaneous Virtio device for BE service
//
// Copyright (C) 2018 Intel Corporation.
//
// Authors: Furtak, Pawel <pawel.furtak@intel.com>
//          Janca, Grzegorz <grzegorz.janca@intel.com>
//
//  This module registers a device node /dev/vbs_k_audio, that handle
//  the communication between Device Model and the virtio backend service.
//  The device model can control the backend to : set the status,
//  set the vq account and etc. The config of the DM and VBS must be accordance.

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/pci.h>

#include "skl-virtio-be.h"
#include "skl-virtio.h"
#include "../skl-sst-ipc.h"


static struct vskl *virtio_audio;

struct vskl *get_virtio_audio(void)
{
	return virtio_audio;
}

/* find client from client ID */
struct snd_skl_vbe_client *vbe_client_find(struct snd_skl_vbe *vbe,
	int client_id)
{
	struct snd_skl_vbe_client *client;

	list_for_each_entry(client, &vbe->client_list, list) {
		if (client_id == client->vhm_client_id)
			return client;
	}

	return NULL;
}

static int vskl_vbs_handle_kick(int client_id, unsigned long *ioreqs_map)
{
	struct vhm_request *req;
	struct snd_skl_vbe_client *client;
	int vcpu, handle;
	struct vskl *vskl = get_virtio_audio();
	struct snd_skl_vbe *vbe = &vskl->vbe;

	if (!vskl) {
		pr_err("err: virtualization service not registred for skl");
		return -EINVAL;
	}

	dev_dbg(vskl->dev, "virtio audio kick handling!\n");

	/* get the client this notification is for/from? */
	client = vbe_client_find(vbe, client_id);
	if (!client) {
		dev_err(vskl->dev, "Ooops! client %d not found!\n",
				client_id);
		return -EINVAL;
	}

	/* go through all vcpu for the valid request buffer */
	while (1) {
		vcpu = find_first_bit(ioreqs_map, client->max_vcpu);
		if (vcpu == client->max_vcpu)
			break;
		req = &client->req_buf[vcpu];
		if (atomic_read(&req->processed) != REQ_STATE_PROCESSING ||
				req->client != client->vhm_client_id)
			continue;

		handle = 0;
		dev_dbg(vskl->dev,
			"ioreq type %d, direction %d, addr 0x%llx, size 0x%llx, value 0x%x\n",
			 req->type,
			 req->reqs.pio_request.direction,
			 req->reqs.pio_request.address,
			 req->reqs.pio_request.size,
			 req->reqs.pio_request.value);

		if (req->reqs.pio_request.direction == REQUEST_READ) {
			/*
			 * currently we handle kick only,
			 * so read will return 0
			 */
			req->reqs.pio_request.value = 0;
		} else {
			req->reqs.pio_request.value >= 0 ?
				(handle = 1) : (handle = 0);
		}

		acrn_ioreq_complete_request(client->vhm_client_id, vcpu, req);

		/* handle VQ kick if needed */
		if (handle)
			vbe_skl_handle_kick(vbe, req->reqs.pio_request.value);
	}

	return 0;
}

int vskl_vbs_init_be(struct vskl *vskl, struct snd_skl_vbe *vbe)
{
	struct virtio_vq_info *vqs;
	struct device *dev = vskl->dev;
	int i;

	INIT_LIST_HEAD(&vbe->client_list);
	INIT_LIST_HEAD(&vbe->pending_msg_list);
	spin_lock_init(&vbe->posn_lock);
	vbe->dev = dev;

	vqs = vbe->vqs;
	for (i = 0; i < SKL_VIRTIO_NUM_OF_VQS; i++) {
		vqs[i].dev = &vbe->dev_info;
		vqs[i].vq_notify = NULL;
	}

	/* link dev and vqs */
	vbe->dev_info.vqs = vqs;
	virtio_dev_init(&vbe->dev_info, vqs, SKL_VIRTIO_NUM_OF_VQS);

	return 0;
}

/*
 * register vhm client with virtio.
 * vhm use the client to handle the io access from FE
 */
int vskl_vbs_register_client(struct snd_skl_vbe *vbe)
{
	struct virtio_dev_info *dev_info = &vbe->dev_info;
	struct vm_info info;
	struct snd_skl_vbe_client *client;
	unsigned int vmid;
	int ret;

	client = list_first_entry_or_null(&vbe->client_list,
		struct snd_skl_vbe_client, list);
	if (client != NULL) {
		dev_info(vbe->dev, "Assign VBE Audio client id:%d\n",
			client->vhm_client_id);
		return 0;
	}

	/*
	 * vbs core has mechanism to manage the client
	 * there is no need to handle this in the special BE driver
	 * let's use the vbs core client management later
	 */
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	client->vbe = vbe;

	vmid = dev_info->_ctx.vmid;
	client->vhm_client_id = acrn_ioreq_create_client(vmid,
		vskl_vbs_handle_kick,
		"snd_skl_vbe kick init\n");
	if (client->vhm_client_id < 0) {
		dev_err(vbe->dev, "failed to create client of acrn ioreq!\n");
		goto err;
	}

	ret = acrn_ioreq_add_iorange(client->vhm_client_id, REQ_PORTIO,
						 dev_info->io_range_start,
						 dev_info->io_range_start +
						 dev_info->io_range_len - 1);
	if (ret < 0) {
		dev_err(vbe->dev, "failed to add iorange to acrn ioreq!\n");
		goto err;
	}

	/*
	 * setup the vm information, such as max_vcpu and max_gfn
	 * BE need this information to handle the vqs
	 */
	ret = vhm_get_vm_info(vmid, &info);
	if (ret < 0) {
		dev_err(vbe->dev, "failed in vhm_get_vm_info!\n");
		goto err;
	}
	client->max_vcpu = info.max_vcpu;

	/* TODO: comment what this is doing */
	client->req_buf = acrn_ioreq_get_reqbuf(client->vhm_client_id);
	if (!client->req_buf) {
		dev_err(vbe->dev, "failed in acrn_ioreq_get_reqbuf!\n");
		goto err;
	}

	/* just attach once as vhm will kick kthread */
	acrn_ioreq_attach_client(client->vhm_client_id, 0);

	INIT_LIST_HEAD(&client->substr_info_list);
	/* complete client init and add to list */
	list_add(&client->list, &vbe->client_list);

	dev_info(vbe->dev, "VBS Audio client:%d had been created\n",
		client->vhm_client_id);
	return 0;
err:
	if (client != NULL && client->vhm_client_id >= 0)
		acrn_ioreq_destroy_client(client->vhm_client_id);

	if (client != NULL) {
		kfree(client);
		client = NULL;
	}
	return -EINVAL;
}

static void vskl_vbs_close_client(struct snd_skl_vbe *vbe)
{
	struct snd_skl_vbe_client *client;

	if (!list_empty(&vbe->client_list)) {
		client = list_first_entry(&vbe->client_list,
				struct snd_skl_vbe_client, list);
		vbe_skl_pcm_close_all(vbe, client);
		acrn_ioreq_destroy_client(client->vhm_client_id);
		list_del(&client->list);

		if (client != NULL) {
			dev_info(vbe->dev, "Delete VBS Audio client. id:%d\n",
				client->vhm_client_id);
			kfree(client);
			client = NULL;
		}

	} else {
		pr_err("%s: vbs client not present!\n", __func__);
	}

}
static int vskl_vbs_audio_open(struct inode *inode, struct file *f)
{
	struct vskl *vskl = get_virtio_audio();

	return vbe_skl_attach(&vskl->vbe, vskl->skl);
}

static long vskl_vbs_audio_ioctl(struct file *f, unsigned int ioctl,
			    unsigned long arg)
{
	struct vskl *vskl = get_virtio_audio();
	struct snd_skl_vbe *vbe = &vskl->vbe;
	void __user *argp = (void __user *)arg;
	int ret;

	switch (ioctl) {
	case VBS_SET_DEV:
		ret = virtio_dev_ioctl(&vbe->dev_info, ioctl, argp);
		if (!ret)
			vbe->vmid = vbe->dev_info._ctx.vmid;
		break;
	case VBS_SET_VQ:
		ret = virtio_vqs_ioctl(&vbe->dev_info, ioctl, argp);
		if (ret)
			return ret;
		ret = vskl_vbs_register_client(vbe);
		if (ret)
			return ret;
		break;
	default:
		pr_err("%s: Unsupported ioctl cmd[%d].\n",
				__func__, ioctl);
		return -ENOIOCTLCMD;
	}

	return ret;
}

static int vskl_vbs_audio_release(struct inode *inode, struct file *f)
{
	struct vskl *vskl = get_virtio_audio();

	vskl_vbs_close_client(&vskl->vbe);
	return 0;
}

static const struct file_operations vskl_vbs_audio_fops = {
	.owner          = THIS_MODULE,
	.release        = vskl_vbs_audio_release,
	.unlocked_ioctl = vskl_vbs_audio_ioctl,
	.open           = vskl_vbs_audio_open,
	.llseek         = noop_llseek,
};

static struct miscdevice vskl_vbs_audio_k = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vbs_k_audio",
	.fops = &vskl_vbs_audio_fops,
};

static int vskl_vbs_init(struct vskl *vskl)
{
	int ret;

	ret = misc_register(&vskl_vbs_audio_k);
	if (ret < 0) {
		dev_err(vskl->dev, "misc device register failed %d\n", ret);
		return ret;
	}

	ret = vskl_vbs_init_be(vskl, &vskl->vbe);
	if (ret)
		return ret;

	return 0;
}

static int vskl_vbs_close(struct vskl *vskl)
{
	misc_deregister(&vskl_vbs_audio_k);
	vbe_skl_unbind(&vskl->vbe, vskl->skl);

	return 0;
}

static int vskl_init(struct vskl *vskl, struct skl *skl, struct device *dev)
{
	int ret;

	vskl->dev = dev;
	vskl->skl = skl;

	ret = vskl_vbs_init(vskl);
	if (ret < 0) {
		dev_err(vskl->dev,
			"Failed to initialize BE service (error: %d)\n", ret);
		return ret;
	}
	vbe_skl_bind(&vskl->vbe, vskl->skl);
	virtio_audio = vskl;

	return 0;
}

static int vskl_close(struct vskl *vskl)
{
	/* For future use e.g. vhost implementation */
	return vskl_vbs_close(vskl);
}

static int vskl_probe(struct platform_device *pdev)
{
	struct vskl *vskl;
	int ret;
	struct skl_virt_pdata *pdata = dev_get_platdata(&pdev->dev);

	if (!pdata || !pdata->skl) {
		dev_err(&pdev->dev, "Failed to find native Skylake audio driver");
		return -ENODEV;
	}

	vskl = devm_kzalloc(&pdev->dev, sizeof(*vskl), GFP_KERNEL);
	if (!vskl)
		return -ENOMEM;

	ret = vskl_init(vskl, pdata->skl, &pdev->dev);
	if (ret < 0)
		return ret;

	pdata->private_data = vskl;

	return 0;
}

static int vskl_remove(struct platform_device *pdev)
{
	int ret;
	struct vskl *vskl;
	struct skl_virt_pdata *pdata = dev_get_platdata(&pdev->dev);

	vskl = pdata->private_data;
	if (!vskl)
		return -EINVAL;

	ret = vskl_close(vskl);
	pdata->private_data = NULL;

	return ret;
}

static struct platform_driver vskl_audio = {
	.probe = vskl_probe,
	.remove = vskl_remove,
	.driver = {
		.name = "skl-virt-audio",
	},
};
module_platform_driver(vskl_audio)

MODULE_DESCRIPTION("ASoC cAVS virtualization driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:skl-virt-audio");
