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
#include "../skl-sst-ipc.h"


static struct virtio_miscdev *virtio_audio;
static int snd_audio_virtio_miscdev_register(struct device *dev, void *data,
					     struct virtio_miscdev **va);
static int snd_audio_virtio_miscdev_unregister(void);

static struct virtio_miscdev *get_virtio_audio(void)
{
	return virtio_audio;
}

void *snd_skl_get_virtio_audio(void)
{
	struct virtio_miscdev *vaudio = get_virtio_audio();

	if (vaudio)
		return vaudio->priv;

	return NULL;
}

/* find client from client ID */
static struct snd_skl_vbe_client *vbe_client_find(struct skl *sdev,
								int client_id)
{
	struct snd_skl_vbe_client *client;
	struct snd_skl_vbe *vbe;

	list_for_each_entry(vbe, &sdev->vbe_list, list) {
		list_for_each_entry(client, &vbe->client_list, list) {
			if (client_id == client->vhm_client_id)
				return client;
		}
	}

	return NULL;
}

static int handle_kick(int client_id, unsigned long *ioreqs_map)
{
	struct vhm_request *req;
	struct snd_skl_vbe_client *client;
	struct snd_skl_vbe *vbe;
	struct skl *sdev = snd_skl_get_virtio_audio();
	int i, handle;

	if (!sdev) {
		pr_err("error: no BE registered for SOF!\n");
		return -EINVAL;
	}

	dev_dbg(sdev->skl_sst->dev, "virtio audio kick handling!\n");

	/* get the client this notification is for/from? */
	client = vbe_client_find(sdev, client_id);
	if (!client) {
		dev_err(sdev->skl_sst->dev, "Ooops! client %d not found!\n",
				client_id);
		return -EINVAL;
	}
	vbe = client->vbe;

	/* go through all vcpu for the valid request buffer */
	for (i = 0; i < client->max_vcpu; i++) {
		req = &client->req_buf[i];
		handle = 0;

		/* ignore if not processing state */
		if (atomic_read(&req->processed) != REQ_STATE_PROCESSING)
			continue;

		dev_dbg(sdev->skl_sst->dev,
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

		atomic_set(&req->processed, REQ_STATE_COMPLETE);
		acrn_ioreq_complete_request(client->vhm_client_id, i);

		/* handle VQ kick if needed */
		if (handle)
			vbe_skl_handle_kick(vbe, req->reqs.pio_request.value);
	}

	return 0;
}

int snd_skl_virtio_register_vbe(struct skl *sdev, struct snd_skl_vbe **svbe)
{
	struct snd_skl_vbe *vbe;
	struct virtio_vq_info *vqs;
	struct device *dev = &sdev->pci->dev;
	int i;

	vbe = devm_kzalloc(dev, sizeof(*vbe), GFP_KERNEL);
	if (!vbe)
		return -ENOMEM;

	INIT_LIST_HEAD(&vbe->client_list);
	INIT_LIST_HEAD(&vbe->substr_info_list);
	INIT_LIST_HEAD(&vbe->posn_list);
	spin_lock_init(&vbe->posn_lock);
	vbe->sdev = sdev;
	vbe->dev = dev;

	vqs = vbe->vqs;
	for (i = 0; i < SKL_VIRTIO_NUM_OF_VQS; i++) {
		vqs[i].dev = &vbe->dev_info;
		/*
		 * currently relies on VHM to kick us,
		 * thus vq_notify not used
		 */
		vqs[i].vq_notify = NULL;
	}

	/* link dev and vqs */
	vbe->dev_info.vqs = vqs;

	virtio_dev_init(&vbe->dev_info, vqs, SKL_VIRTIO_NUM_OF_VQS);

	*svbe = vbe;

	return 0;
}

/*
 * register vhm client with virtio.
 * vhm use the client to handle the io access from FE
 */
int snd_skl_virtio_register_client(struct snd_skl_vbe *vbe)
{
	struct virtio_dev_info *dev_info = &vbe->dev_info;
	struct vm_info info;
	struct snd_skl_vbe_client *client;
	unsigned int vmid;
	int ret;

	/*
	 * vbs core has mechanism to manage the client
	 * there is no need to handle this in the special BE driver
	 * let's use the vbs core client management later
	 */
	client = devm_kzalloc(vbe->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -EINVAL;
	client->vbe = vbe;

	vmid = dev_info->_ctx.vmid;
	client->vhm_client_id = acrn_ioreq_create_client(vmid, handle_kick,
							 "snd_skl_vbe kick init\n");
	if (client->vhm_client_id < 0) {
		dev_err(vbe->dev, "failed to create client of acrn ioreq!\n");
		return client->vhm_client_id;
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

	/* complete client init and add to list */
	list_add(&client->list, &vbe->client_list);

	return 0;
err:
	acrn_ioreq_destroy_client(client->vhm_client_id);
	return -EINVAL;
}

static int snd_skl_virtio_open(struct file *f, void *data)
{
	struct skl *sdev = (struct skl *)data;
	struct snd_skl_vbe *vbe;
	int ret;

	ret = snd_skl_virtio_register_vbe(sdev, &vbe);
	if (ret)
		return ret;

	/*
	 * link to sdev->vbe_list
	 * Maybe virtio_miscdev managing the list is more reasonable.
	 * Let's use sdev to manage the FE audios now.
	 */
	list_add(&vbe->list, &sdev->vbe_list);
	f->private_data = vbe;

	return 0;
}

static long snd_skl_virtio_ioctl(struct file *f, void *data, unsigned int ioctl,
			     unsigned long arg)
{
	struct snd_skl_vbe *vbe = f->private_data;
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

		ret = snd_skl_virtio_register_client(vbe);
		if (ret)
			return ret;
		/*
		 * TODO: load tplg and send to FE here
		 *
		 *  The better method is FE driver send FE-tplg id
		 *  and request FE-tplg.
		 *  Then BE loads the corresponding tplg based on
		 *  the FE-tplg id and send to FE driver.
		 */
		break;
	default:
		pr_err("%s: _snd_virtio_miscdevice Unsupported ioctl cmd[%d].\n",
				__func__, ioctl);
		return -ENOIOCTLCMD;
	}

	return ret;
}

static int snd_skl_virtio_release(struct file *f, void *data)
{
	struct snd_skl_vbe *vbe = f->private_data;

	list_del(&vbe->list);
	devm_kfree(vbe->sdev->skl_sst->dev, vbe);
	f->private_data = NULL;

	return 0;
}

/**
 * snd_soc_skl_virtio_miscdev_register() - init the virtio be audio driver
 * @sdev: the skl structure
 *
 * This function registers the misc device, which will be used
 * by the user space to communicate with the audio driver.
 *
 * Return: 0 for success or negative value for err
 */
int snd_soc_skl_virtio_miscdev_register(struct skl *sdev)
{
	struct virtio_miscdev *vaudio;
	int ret;

	ret = snd_audio_virtio_miscdev_register(sdev->skl_sst->dev,
			sdev, &vaudio);
	if (ret) {
		dev_err(vaudio->dev, "failed to register misc device %d\n",
				ret);
		return ret;
	}

	INIT_LIST_HEAD(&sdev->vbe_list);

	vaudio->open = snd_skl_virtio_open;
	vaudio->ioctl = snd_skl_virtio_ioctl;
	vaudio->release = snd_skl_virtio_release;

	dev_info(vaudio->dev, "register misc device success\n");
	return 0;
}
EXPORT_SYMBOL(snd_soc_skl_virtio_miscdev_register);

/**
 * snd_soc_skl_virtio_miscdev_unregister() - release the virtio be audio driver
 *
 * This function deregisters the misc device, and free virtio_miscdev
 *
 */
int snd_soc_skl_virtio_miscdev_unregister(void)
{
	return snd_audio_virtio_miscdev_unregister();
}
EXPORT_SYMBOL(snd_soc_skl_virtio_miscdev_unregister);

static int vbs_audio_open(struct inode *inode, struct file *f)
{
	struct virtio_miscdev *vaudio = get_virtio_audio();

	if (!vaudio)
		return -ENODEV;	/* This should never happen */

	dev_dbg(vaudio->dev, "virtio audio open\n");
	if (vaudio->open)
		return vaudio->open(f, virtio_audio->priv);

	return 0;
}

static long vbs_audio_ioctl(struct file *f, unsigned int ioctl,
			    unsigned long arg)
{
	struct virtio_miscdev *vaudio = get_virtio_audio();

	if (!vaudio)
		return -ENODEV;	/* This should never happen */

	dev_dbg(vaudio->dev, "virtio audio ioctl\n");
	if (vaudio->ioctl)
		return vaudio->ioctl(f, vaudio->priv, ioctl, arg);
	else
		return -ENXIO;
}

static int vbs_audio_release(struct inode *inode, struct file *f)
{
	struct virtio_miscdev *vaudio = get_virtio_audio();

	if (!vaudio)
		return -ENODEV;	/* This should never happen */

	dev_dbg(vaudio->dev, "release virtio audio\n");

	if (vaudio->release)
		vaudio->release(f, vaudio->priv);

	return 0;
}

static const struct file_operations vbs_audio_fops = {
	.owner          = THIS_MODULE,
	.release        = vbs_audio_release,
	.unlocked_ioctl = vbs_audio_ioctl,
	.open           = vbs_audio_open,
	.llseek         = noop_llseek,
};

static struct miscdevice vbs_audio_k = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vbs_k_audio",
	.fops = &vbs_audio_fops,
};

static int snd_audio_virtio_miscdev_register(struct device *dev, void *data,
					     struct virtio_miscdev **va)
{
	struct virtio_miscdev *vaudio;
	int ret;

	ret = misc_register(&vbs_audio_k);
	if (ret) {
		dev_err(dev, "misc device register failed %d\n", ret);
		return ret;
	}

	vaudio = kzalloc(sizeof(*vaudio), GFP_KERNEL);
	if (!vaudio) {
		misc_deregister(&vbs_audio_k);
		return -ENOMEM;
	}

	vaudio->priv = data;
	vaudio->dev = dev;
	virtio_audio = vaudio;
	*va = vaudio;

	return 0;
}

static int snd_audio_virtio_miscdev_unregister(void)
{
	if (virtio_audio) {
		misc_deregister(&vbs_audio_k);
		kfree(virtio_audio);
		virtio_audio = NULL;
	}

	return 0;
}
