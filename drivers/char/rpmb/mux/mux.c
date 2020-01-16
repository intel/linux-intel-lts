// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2019 Intel Corporation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/rpmb.h>
#include <crypto/hash.h>

#include "key.h"

/**
 * struct rpmb_mux_dev - device which can support RPMB partition
 * @lock           : the device lock
 * @rdev           : point to the rpmb device
 * @cdev           : character dev
 * @rpmb_interface : rpmb class interface
 * @write_counter  : write counter of RPMB
 * @wc_inited      : write counter is initialized
 * @rpmb_key       : RPMB authentication key
 * @hash_desc      : hmac(sha256) shash descriptor
 */
struct rpmb_mux_dev {
	struct mutex lock; /* device serialization lock */
	struct rpmb_dev *rdev;
	struct cdev cdev;
	struct class_interface rpmb_interface;

	u32 write_counter;
	u32 wc_inited;
	u8 rpmb_key[32];
	struct shash_desc *hash_desc;
};

static dev_t rpmb_mux_devt;
static struct rpmb_mux_dev *__mux_dev;
static struct class *rpmb_mux_class;
/* from MMC_IOC_MAX_CMDS */
#define RPMB_MAX_FRAMES 255

static int rpmb_mux_open(struct inode *inode, struct file *fp)
{
	struct rpmb_mux_dev *mux_dev;

	mux_dev = container_of(inode->i_cdev, struct rpmb_mux_dev, cdev);
	if (!mux_dev)
		return -ENODEV;

	mutex_lock(&mux_dev->lock);

	fp->private_data = mux_dev;

	mutex_unlock(&mux_dev->lock);

	return nonseekable_open(inode, fp);
}

static int rpmb_mux_release(struct inode *inode, struct file *fp)
{
	return 0;
}

static int rpmb_mux_hmac_256_alloc(struct rpmb_mux_dev *mux_dev)
{
	struct shash_desc *desc;
	struct crypto_shash *tfm;

	tfm = crypto_alloc_shash("hmac(sha256)", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	desc = kzalloc(sizeof(*desc) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (!desc) {
		crypto_free_shash(tfm);
		return -ENOMEM;
	}

	desc->tfm = tfm;
	mux_dev->hash_desc = desc;

	return 0;
}

static void rpmb_mux_hmac_256_free(struct rpmb_mux_dev *mux_dev)
{
	struct shash_desc *desc = mux_dev->hash_desc;

	crypto_free_shash(desc->tfm);
	kfree(desc);

	mux_dev->hash_desc = NULL;
}

static int rpmb_mux_calc_hmac(struct rpmb_mux_dev *mux_dev,
			      struct rpmb_frame_jdec *frames,
			      unsigned int blks, u8 *mac)
{
	struct shash_desc *desc = mux_dev->hash_desc;
	int ret;
	unsigned int i;

	ret = crypto_shash_init(desc);
	if (ret)
		return ret;

	for (i = 0; i < blks; i++) {
		ret = crypto_shash_update(desc, frames[i].data,
					  rpmb_jdec_hmac_data_len);
		if (ret)
			return ret;
	}

	ret = crypto_shash_final(desc, mac);

	return ret;
}

static int rpmb_program_key(struct rpmb_mux_dev *mux_dev)
{
	struct rpmb_frame_jdec *frame_write, *frame_rel, *frame_out;
	struct rpmb_cmd *cmds;
	int ret;

	frame_write = kzalloc(sizeof(*frame_write), GFP_KERNEL);
	frame_rel = kzalloc(sizeof(*frame_rel), GFP_KERNEL);
	frame_out = kzalloc(sizeof(*frame_out), GFP_KERNEL);
	cmds = kcalloc(3, sizeof(*cmds), GFP_KERNEL);
	if (!frame_write || !frame_rel || !frame_out || !cmds) {
		ret = -ENOMEM;
		goto out;
	}

	/* fill rel write frame */
	memcpy(frame_rel->key_mac, mux_dev->rpmb_key,
	       sizeof(mux_dev->rpmb_key));
	frame_rel->req_resp = cpu_to_be16(RPMB_PROGRAM_KEY);

	/* fill write frame */
	frame_write->req_resp = cpu_to_be16(RPMB_RESULT_READ);

	/* fill io cmd */
	cmds[0].flags = RPMB_F_WRITE | RPMB_F_REL_WRITE;
	cmds[0].nframes = 1;
	cmds[0].frames = frame_rel;
	cmds[1].flags = RPMB_F_WRITE;
	cmds[1].nframes = 1;
	cmds[1].frames = frame_write;
	cmds[2].flags = 0;
	cmds[2].nframes = 1;
	cmds[2].frames = frame_out;

	ret = rpmb_cmd_seq(mux_dev->rdev, cmds, 3);
	if (ret)
		goto out;

	if (be16_to_cpu(frame_out->result) != RPMB_ERR_OK) {
		ret = -EPERM;
		dev_err(&mux_dev->rdev->dev, "rpmb program key failed(0x%X).\n",
			be16_to_cpu(frame_out->result));
	}

out:
	kfree(frame_write);
	kfree(frame_rel);
	kfree(frame_out);
	kfree(cmds);

	return ret;
}

static int rpmb_get_counter(struct rpmb_mux_dev *mux_dev)
{
	struct rpmb_frame_jdec *in_frame, *out_frame;
	struct rpmb_cmd *cmds;
	int ret;
	u8 mac[32];

	in_frame = kzalloc(sizeof(*in_frame), GFP_KERNEL);
	out_frame = kzalloc(sizeof(*out_frame), GFP_KERNEL);
	cmds = kcalloc(2, sizeof(*cmds), GFP_KERNEL);
	if (!in_frame || !out_frame || !cmds) {
		ret = -ENOMEM;
		goto out;
	}

	in_frame->req_resp = cpu_to_be16(RPMB_GET_WRITE_COUNTER);
	cmds[0].flags = RPMB_F_WRITE;
	cmds[0].nframes = 1;
	cmds[0].frames = in_frame;
	cmds[1].flags = 0;
	cmds[1].nframes = 1;
	cmds[1].frames = out_frame;

	ret = rpmb_cmd_seq(mux_dev->rdev, cmds, 2);
	if (ret)
		goto out;

	ret = rpmb_mux_calc_hmac(mux_dev, out_frame, 1, mac);
	if (ret) {
		dev_err(&mux_dev->rdev->dev, "MAC calculation failed for read counter\n");
		goto out;
	}

	if (memcmp(mac, out_frame->key_mac, sizeof(mac))) {
		ret = -EPERM;
		dev_err(&mux_dev->rdev->dev, "MAC check failed for read counter\n");
		goto out;
	}

	if (be16_to_cpu(out_frame->result) == RPMB_ERR_NO_KEY) {
		dev_dbg(&mux_dev->rdev->dev, "Start to program key...\n");
		ret = rpmb_program_key(mux_dev);
		if (ret)
			goto out;
	} else if (be16_to_cpu(out_frame->result) != RPMB_ERR_OK) {
		ret = -EPERM;
		dev_err(&mux_dev->rdev->dev, "get rpmb counter failed(0x%X).\n",
			be16_to_cpu(out_frame->result));
		goto out;
	}

	mux_dev->write_counter = be32_to_cpu(out_frame->write_counter);

out:
	kfree(in_frame);
	kfree(out_frame);
	kfree(cmds);

	return ret;
}

static size_t rpmb_ioc_frames_len(struct rpmb_dev *rdev, size_t nframes)
{
	return rpmb_ioc_frames_len_jdec(nframes);
}

/**
 * rpmb_mux_copy_from_user - copy rpmb command from the user space
 *
 * @rdev: rpmb device
 * @cmd:  internal cmd structure
 * @ucmd: user space cmd structure
 *
 * Return: 0 on success, <0 on error
 */
static int rpmb_mux_copy_from_user(struct rpmb_dev *rdev,
				   struct rpmb_cmd *cmd,
				   struct rpmb_ioc_cmd __user *ucmd)
{
	void *frames;
	u64 frames_ptr;

	if (get_user(cmd->flags, &ucmd->flags))
		return -EFAULT;

	if (get_user(cmd->nframes, &ucmd->nframes))
		return -EFAULT;

	if (cmd->nframes > RPMB_MAX_FRAMES)
		return -EOVERFLOW;

	/* some archs have issues with 64bit get_user */
	if (copy_from_user(&frames_ptr, &ucmd->frames_ptr, sizeof(frames_ptr)))
		return -EFAULT;

	frames = memdup_user(u64_to_user_ptr(frames_ptr),
			     rpmb_ioc_frames_len(rdev, cmd->nframes));
	if (IS_ERR(frames))
		return PTR_ERR(frames);

	cmd->frames = frames;
	return 0;
}

/**
 * rpmb_mux_copy_to_user - copy rpmb command to the user space
 *
 * @rdev: rpmb device
 * @ucmd: user space cmd structure
 * @cmd:  internal cmd structure
 *
 * Return: 0 on success, <0 on error
 */
static int rpmb_mux_copy_to_user(struct rpmb_dev *rdev,
				 struct rpmb_ioc_cmd __user *ucmd,
				 struct rpmb_cmd *cmd)
{
	u64 frames_ptr;

	if (copy_from_user(&frames_ptr, &ucmd->frames_ptr, sizeof(frames_ptr)))
		return -EFAULT;

	/* some archs have issues with 64bit get_user */
	if (copy_to_user(u64_to_user_ptr(frames_ptr), cmd->frames,
			 rpmb_ioc_frames_len(rdev, cmd->nframes)))
		return -EFAULT;

	return 0;
}

static int rpmb_replace_write_frame(struct rpmb_mux_dev *mux_dev,
				    struct rpmb_cmd *cmds, u32 ncmd)
{
	u32 i;
	u32 frame_cnt;
	__be32 write_counter;
	struct rpmb_frame_jdec *in_frames = cmds[0].frames;

	if (in_frames->req_resp != cpu_to_be16(RPMB_WRITE_DATA)) {
		dev_err(&mux_dev->rdev->dev, "rpmb ioctl frame is unsupported(0x%X).\n",
			in_frames->req_resp);
		return -EINVAL;
	}

	frame_cnt = cmds[0].nframes;
	write_counter = cpu_to_be32(mux_dev->write_counter);
	for (i = 0; i < frame_cnt; i++)
		in_frames[i].write_counter = write_counter;

	if (rpmb_mux_calc_hmac(mux_dev, in_frames, frame_cnt,
			       in_frames[frame_cnt - 1].key_mac)) {
		dev_err(&mux_dev->rdev->dev, "MAC calculation failed for rpmb write\n");
		return -ERANGE;
	}

	return 0;
}

static int rpmb_check_mac(struct rpmb_mux_dev *mux_dev, struct rpmb_cmd *cmds)
{
	u32 frame_cnt;
	u8 mac[32];
	struct rpmb_frame_jdec *in_frames = cmds[0].frames;

	frame_cnt = cmds[0].nframes;

	if (rpmb_mux_calc_hmac(mux_dev, in_frames, frame_cnt, mac)) {
		dev_err(&mux_dev->rdev->dev, "MAC calculation failed for rpmb write\n");
		return -ERANGE;
	}

	if (memcmp(mac, in_frames[frame_cnt - 1].key_mac, sizeof(mac))) {
		dev_err(&mux_dev->rdev->dev, "MAC check failed for write data\n");
		return -EPERM;
	}

	return 0;
}

static int rpmb_check_result(struct rpmb_mux_dev *mux_dev,
			     struct rpmb_cmd *cmds, u32 ncmd)
{
	struct rpmb_frame_jdec *out_frames = cmds[ncmd - 1].frames;
	int ret;

	ret = rpmb_check_mac(mux_dev, cmds);
	if (ret) {
		dev_err(&mux_dev->rdev->dev, "rpmb check mac fail!\n");
		return ret;
	}

	/* write retry */
	if (out_frames->result == cpu_to_be16(RPMB_ERR_COUNTER)) {
		dev_err(&mux_dev->rdev->dev, "rpmb counter error, write retry!\n");
		memset(out_frames, 0, sizeof(*out_frames));

		ret = rpmb_get_counter(mux_dev);
		if (ret) {
			dev_err(&mux_dev->rdev->dev, "rpmb_get_counter failed!\n");
			return ret;
		}

		/* Since phy_counter has changed,
		 * so we have to generate mac again
		 */
		ret = rpmb_replace_write_frame(mux_dev, cmds, ncmd);
		if (ret) {
			dev_err(&mux_dev->rdev->dev, "rpmb replace write frame failed\n");
			return ret;
		}

		ret = rpmb_cmd_seq(mux_dev->rdev, cmds, ncmd);
		if (ret) {
			dev_err(&mux_dev->rdev->dev, "rpmb write retry failed\n");
			return ret;
		}

		ret = rpmb_check_mac(mux_dev, cmds);
		if (ret) {
			dev_err(&mux_dev->rdev->dev, "write retry rpmb check mac fail!\n");
			return ret;
		}
	}

	if (out_frames->result == cpu_to_be16(RPMB_ERR_OK)) {
		dev_dbg(&mux_dev->rdev->dev, "write_counter =%d\n",
			mux_dev->write_counter);
		mux_dev->write_counter++;
	} else {
		dev_err(&mux_dev->rdev->dev, "ERR result is 0x%X.\n",
			be16_to_cpu(out_frames->result));
	}

	return 0;
}

/**
 * rpmb_ioctl_seq_cmd() - issue an rpmb command sequence
 * @mux_dev: rpmb mux_device
 * @ptr: rpmb cmd sequence
 *
 * RPMB_IOC_SEQ_CMD handler
 *
 * Return: 0 on success, <0 on error
 */
static long rpmb_ioctl_seq_cmd(struct rpmb_mux_dev *mux_dev,
			       struct rpmb_ioc_seq_cmd __user *ptr)
{
	struct rpmb_dev *rdev = mux_dev->rdev;
	__u64 ncmds;
	struct rpmb_cmd *cmds;
	struct rpmb_ioc_cmd __user *ucmds;
	unsigned int i;
	int ret;

	/* The caller must have CAP_SYS_RAWIO, like mmc ioctl */
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	/* some archs have issues with 64bit get_user */
	if (copy_from_user(&ncmds, &ptr->num_of_cmds, sizeof(ncmds)))
		return -EFAULT;

	if (ncmds > 3) {
		dev_err(&rdev->dev, "supporting up to 3 packets (%llu)\n",
			ncmds);
		return -EINVAL;
	}

	cmds = kcalloc(ncmds, sizeof(*cmds), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;

	ucmds = (struct rpmb_ioc_cmd __user *)ptr->cmds;
	for (i = 0; i < ncmds; i++) {
		ret = rpmb_mux_copy_from_user(rdev, &cmds[i], &ucmds[i]);
		if (ret)
			goto out;
	}

	if (cmds->flags & RPMB_F_REL_WRITE) {
		ret = rpmb_replace_write_frame(mux_dev, cmds, ncmds);
		if (ret)
			goto out;
	}

	ret = rpmb_cmd_seq(rdev, cmds, ncmds);
	if (ret)
		goto out;

	if (cmds->flags & RPMB_F_REL_WRITE) {
		ret = rpmb_check_result(mux_dev, cmds, ncmds);
		if (ret)
			goto out;
	}

	for (i = 0; i < ncmds; i++) {
		ret = rpmb_mux_copy_to_user(rdev, &ucmds[i], &cmds[i]);
		if (ret)
			goto out;
	}

out:
	for (i = 0; i < ncmds; i++)
		kfree(cmds[i].frames);
	kfree(cmds);

	return ret;
}

static long rpmb_mux_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	long ret;
	struct rpmb_mux_dev *mux_dev = fp->private_data;
	void __user *ptr = (void __user *)arg;

	mutex_lock(&mux_dev->lock);

	if (!mux_dev->rdev) {
		pr_err("rpmb dev is NULL!\n");
		ret = -EINVAL;
		goto out;
	}

	if (!mux_dev->wc_inited) {
		ret = rpmb_get_counter(mux_dev);
		if (ret) {
			dev_err(&mux_dev->rdev->dev,
				"init counter failed = %ld\n", ret);
			goto out;
		}

		mux_dev->wc_inited = true;
	}

	switch (cmd) {
	case RPMB_IOC_SEQ_CMD:
		ret = rpmb_ioctl_seq_cmd(mux_dev, ptr);
		break;
	default:
		dev_err(&mux_dev->rdev->dev, "unsupport:0x%X!!!\n", cmd);
		ret = -ENOIOCTLCMD;
	}

out:
	mutex_unlock(&mux_dev->lock);

	return ret;
}

static int rpmb_mux_start(struct rpmb_mux_dev *mux_dev, struct rpmb_dev *rdev)
{
	if (mux_dev->rdev == rdev)
		return 0;

	if (mux_dev->rdev) {
		dev_err(&rdev->dev, "rpmb device already registered\n");
		return -EEXIST;
	}

	mux_dev->rdev = rpmb_dev_get(rdev);
	dev_dbg(&rdev->dev, "rpmb partition created\n");
	return 0;
}

static int rpmb_mux_stop(struct rpmb_mux_dev *mux_dev, struct rpmb_dev *rdev)
{
	if (!mux_dev->rdev) {
		dev_err(&rdev->dev, "Already stopped\n");
		return -EPROTO;
	}

	if (rdev && mux_dev->rdev != rdev) {
		dev_err(&rdev->dev, "Wrong RPMB on stop\n");
		return -EINVAL;
	}

	rpmb_dev_put(mux_dev->rdev);
	mux_dev->rdev = NULL;

	dev_dbg(&rdev->dev, "rpmb partition removed\n");
	return 0;
}

static int rpmb_add_device(struct device *dev, struct class_interface *intf)
{
	struct rpmb_mux_dev *mux_dev;
	struct rpmb_dev *rdev = to_rpmb_dev(dev);
	u8 rpmb_key[RPMB_MAX_PARTITION_NUMBER][RPMB_KEY_LENGTH];
	int ret;

	mux_dev = container_of(intf, struct rpmb_mux_dev, rpmb_interface);

	if (!rdev->ops)
		return -EINVAL;

	if (rdev->ops->type != RPMB_TYPE_EMMC) {
		dev_err(&rdev->dev, "support RPMB_TYPE_EMMC only.\n");
		return -ENOENT;
	}

	mutex_lock(&mux_dev->lock);

	ret = rpmb_mux_start(mux_dev, rdev);
	if (ret) {
		dev_err(&rdev->dev, "fail in rpmb_mux_start.\n");
		mutex_unlock(&mux_dev->lock);
		return ret;
	}

	mutex_unlock(&mux_dev->lock);

	memset(rpmb_key, 0, sizeof(rpmb_key));
	ret = rpmb_key_get(mux_dev->rdev->ops->dev_id,
			   mux_dev->rdev->ops->dev_id_len,
			   RPMB_MAX_PARTITION_NUMBER,
			   rpmb_key);
	if (ret) {
		dev_err(&rdev->dev, "rpmb_key_get failed: %d.\n", ret);
		goto err_rpmb_key_get;
	}
	memcpy(mux_dev->rpmb_key, &rpmb_key[0], sizeof(mux_dev->rpmb_key));
	memset(rpmb_key, 0, sizeof(rpmb_key));

	ret = crypto_shash_setkey(mux_dev->hash_desc->tfm,
				  mux_dev->rpmb_key, 32);
	if (ret) {
		dev_err(&rdev->dev, "set key failed = %d\n", ret);
		goto err_crypto_shash_setkey;
	}

	return 0;

err_crypto_shash_setkey:
	memset(mux_dev->rpmb_key, 0, sizeof(mux_dev->rpmb_key));
err_rpmb_key_get:
	rpmb_mux_hmac_256_free(mux_dev);
	device_destroy(rpmb_mux_class, rpmb_mux_devt);
	class_destroy(rpmb_mux_class);
	cdev_del(&mux_dev->cdev);
	kfree(mux_dev);
	unregister_chrdev_region(rpmb_mux_devt, 0);

	return ret;
}

static void rpmb_remove_device(struct device *dev, struct class_interface *intf)
{
	struct rpmb_mux_dev *mux_dev;
	struct rpmb_dev *rdev = to_rpmb_dev(dev);

	mux_dev = container_of(intf, struct rpmb_mux_dev, rpmb_interface);

	mutex_lock(&mux_dev->lock);
	if (rpmb_mux_stop(mux_dev, rdev))
		dev_err(&rdev->dev, "fail in rpmb_mux_stop.\n");
	mutex_unlock(&mux_dev->lock);
}

static const struct file_operations rpmb_mux_fops = {
	.open           = rpmb_mux_open,
	.release        = rpmb_mux_release,
	.unlocked_ioctl = rpmb_mux_ioctl,
	.llseek         = noop_llseek,
	.owner          = THIS_MODULE,
};

static int __init rpmb_mux_init(void)
{
	int ret;
	struct device *class_dev;
	struct rpmb_mux_dev *mux_dev;

	ret = alloc_chrdev_region(&rpmb_mux_devt, 0, MINORMASK, "rpmbmux");
	if (ret < 0) {
		pr_err("unable to allocate char dev region\n");
		return ret;
	}

	mux_dev = kzalloc(sizeof(*mux_dev), GFP_KERNEL);
	if (!mux_dev) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}
	__mux_dev = mux_dev;

	cdev_init(&mux_dev->cdev, &rpmb_mux_fops);
	mux_dev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&mux_dev->cdev, rpmb_mux_devt, 1);
	if (ret) {
		pr_err("unable to cdev_add.\n");
		goto err_cdev_add;
	}

	rpmb_mux_class = class_create(THIS_MODULE, "rpmbmux");
	if (IS_ERR(rpmb_mux_class)) {
		ret = PTR_ERR(rpmb_mux_class);
		goto err_class_create;
	}

	class_dev = device_create(rpmb_mux_class, NULL,
				  rpmb_mux_devt, mux_dev, "rpmbmux");
	if (IS_ERR(class_dev)) {
		pr_err("failed to device_create!!!\n");
		ret = PTR_ERR(class_dev);
		goto err_device_create;
	}

	ret = rpmb_mux_hmac_256_alloc(mux_dev);
	if (ret) {
		pr_err("failed to set rpmb_mux_hmac_256_alloc.\n");
		goto err_rpmb_mux_hmac_256_alloc;
	}

	mux_dev->rpmb_interface.add_dev    = rpmb_add_device;
	mux_dev->rpmb_interface.remove_dev = rpmb_remove_device;
	mux_dev->rpmb_interface.class      = &rpmb_class;

	ret = class_interface_register(&mux_dev->rpmb_interface);
	if (ret) {
		pr_err("Can't register interface\n");
		goto err_class_interface_register;
	}

	return 0;

err_class_interface_register:
err_rpmb_mux_hmac_256_alloc:
	device_destroy(rpmb_mux_class, rpmb_mux_devt);
err_device_create:
	class_destroy(rpmb_mux_class);
err_class_create:
	cdev_del(&mux_dev->cdev);
err_cdev_add:
	kfree(mux_dev);
err_kzalloc:
	unregister_chrdev_region(rpmb_mux_devt, 0);
	return ret;
}

static void __exit rpmb_mux_exit(void)
{
	struct rpmb_mux_dev *mux_dev = __mux_dev;

	class_interface_unregister(&mux_dev->rpmb_interface);
	device_destroy(rpmb_mux_class, rpmb_mux_devt);
	class_destroy(rpmb_mux_class);
	cdev_del(&mux_dev->cdev);
	unregister_chrdev_region(rpmb_mux_devt, 0);

	rpmb_mux_hmac_256_free(mux_dev);
	memset(mux_dev->rpmb_key, 0, sizeof(mux_dev->rpmb_key));
	kfree(mux_dev);
}

module_init(rpmb_mux_init);
module_exit(rpmb_mux_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("RPMB Mux kernel module");
MODULE_LICENSE("GPL v2");
