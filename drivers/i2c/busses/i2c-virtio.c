// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/*
 * Virtio I2C Front End Driver
 *
 * Copyright (c) 2019 Intel Corporation. All rights reserved.
 */
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/acpi.h>

#define VIRTIO_I2C_TIMEOUT	msecs_to_jiffies(100)
#define I2C_VIRTIO_F_BUS_NUM	1

struct virtio_i2c_hdr {
	__virtio16 addr;	/* slave address */
	__virtio16 flags;
	__virtio16 len;		/*msg length*/
} __packed;

struct virtio_i2c_msg {
	struct virtio_i2c_hdr hdr;
	char *buf;
	u8 status;
#define VIRTIO_I2C_MSG_OK	0
#define VIRTIO_I2C_MSG_ERR	1
	struct i2c_msg *msg;
};

#define VQ_NAME_LEN  256
struct virtio_i2c_vq {
	struct virtqueue *vq;
	char name[VQ_NAME_LEN];
} ____cacheline_aligned_in_smp;

struct virtio_i2c {
	struct virtio_device *vdev;
	struct completion completion;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	struct mutex i2c_lock;
	struct virtio_i2c_vq msg_vq;
};

static inline struct virtio_i2c *virtio_i2c_adapter(struct virtio_device *vdev)
{
	return vdev->priv;
}

static void virti2c_msg_done(struct virtqueue *vq)
{
	struct virtio_i2c *i2c = virtio_i2c_adapter(vq->vdev);

	dev_dbg(&vq->vdev->dev, "virti2c_msg_done\n");
	complete(&i2c->completion);
}

static int virtio_queue_add_msg(struct virtqueue *vq,
			struct virtio_i2c_msg *vmsg,
			struct i2c_msg *msg)
{
	struct scatterlist *sgs[3], hdr, bout, bin, status;
	int outcnt = 0, incnt = 0;

	vmsg->hdr.addr = msg->addr;
	vmsg->hdr.flags = msg->flags;
	vmsg->hdr.len = msg->len;

	if (vmsg->hdr.len)
		vmsg->buf = kzalloc(vmsg->hdr.len, GFP_KERNEL);

	sg_init_one(&hdr, &vmsg->hdr, sizeof(struct virtio_i2c_hdr));
	sgs[outcnt++] = &hdr;
	if (vmsg->buf) {
		if (vmsg->hdr.flags & I2C_M_RD) {
			sg_init_one(&bin, vmsg->buf, msg->len);
			sgs[outcnt + incnt++] = &bin;
		} else {
			memcpy(vmsg->buf, msg->buf, msg->len);
			sg_init_one(&bout, vmsg->buf, msg->len);
			sgs[outcnt++] = &bout;
		}
	}
	sg_init_one(&status, &vmsg->status, 1);
	sgs[outcnt + incnt++] = &status;

	return virtqueue_add_sgs(vq, sgs, outcnt, incnt, vmsg, GFP_KERNEL);
}

static int virtio_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct virtio_i2c *virtio_i2c = i2c_get_adapdata(adap);
	struct virtqueue *vq = virtio_i2c->msg_vq.vq;
	struct virtio_i2c_msg *vmsg, *msg_r;
	int len, i, ret;
	unsigned long time_left;

	ret = 0;
	if (unlikely(!vq))
		return ret;

	vmsg = kzalloc(sizeof(*vmsg), GFP_KERNEL);
	if (unlikely(!vmsg))
		return ret;

	mutex_lock(&virtio_i2c->i2c_lock);
	vmsg->buf = NULL;
	for (i = 0; i < num; i++) {
		dev_dbg(&adap->dev,
			"start to add msg[%d]: addr:0x%x \n",
			i,
			msgs[i].addr);

		ret = virtio_queue_add_msg(vq, vmsg, &msgs[i]);
		if (ret) {
			dev_err(&adap->dev,
				"failed to add msg[%d] to virtqueue\n",
				i);
			ret = 0;
			goto err;
		}

		virtqueue_kick(vq);
		dev_dbg(&adap->dev, "wait for complete...\n");
		/*wait for complete*/
		time_left = wait_for_completion_timeout(&virtio_i2c->completion,
						adap->timeout);
		if (!time_left) {
			dev_err(&adap->dev,
				"msg[%d]: addr=0x%x timeout\n",
				i,
				msgs[i].addr);
			ret = i ? (i - 1) : 0;
			goto err;
		}
		msg_r = (struct virtio_i2c_msg *)virtqueue_get_buf(vq, &len);
		if (msg_r) {
			/* msg_r should point to the same address with vmsg */
			if (msg_r != vmsg) {
				dev_err(&adap->dev,
					"msg[%d]: addr=0x%x virtqueue error \n",
					i,
					msg_r->hdr.addr);
				ret = i - 1;
				goto err;
			}

			if (msg_r->status != VIRTIO_I2C_MSG_OK) {
				dev_dbg(&adap->dev,
					"msg[%d]: addr=0x%x error=%d \n",
					 i,
					 msg_r->hdr.addr,
					 msg_r->status);
				ret = i - 1;
				goto err;
			}
			if ((msg_r->hdr.flags & I2C_M_RD) && msg_r->hdr.len)
				memcpy(msgs[i].buf, msg_r->buf, msg_r->hdr.len);
			kfree(msg_r->buf);
			msg_r->buf = NULL;
		}
		reinit_completion(&virtio_i2c->completion);
		dev_dbg(&adap->dev, "msg[%d] ok \n", i);
	}
	if (i == num)
		ret = num;

err:
	mutex_unlock(&virtio_i2c->i2c_lock);
	if (vmsg) {
		kfree(vmsg->buf);
		vmsg->buf = NULL;
		kfree(vmsg);
	}
	return ret;
}

static void virtio_i2c_init_vq(struct virtio_i2c_vq *virtio_i2c_vq,
			       struct virtqueue *vq)
{
	virtio_i2c_vq->vq = vq;
}

static void virtio_i2c_remove_vqs(struct virtio_device *vdev)
{
	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);
}

static int virtio_i2c_init(struct virtio_i2c *virtio_i2c)
{
	int err;
	struct virtio_device *vdev = virtio_i2c->vdev;
	vq_callback_t *callbacks[1] = {virti2c_msg_done};
	const char *names[1] = {"i2c-msg"};
	struct virtqueue *vqs[1];

	err = virtio_find_vqs(vdev, 1, vqs, callbacks, names, NULL);
	if (err)
		return err;
	virtio_i2c_init_vq(&virtio_i2c->msg_vq, vqs[0]);
	return 0;
}

static u32 virtio_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm virtio_algorithm = {
	.master_xfer = virtio_xfer,
	.functionality = virtio_func,
};

static struct i2c_adapter virtio_adapter = {
	.owner = THIS_MODULE,
	.name = "i2c-virtio",
	.class = I2C_CLASS_DEPRECATED,
	.algo = &virtio_algorithm,
};

static int virtio_i2c_probe(struct virtio_device *vdev)
{
	struct virtio_i2c *virtio_i2c;
	int ret;
	struct device *pdev = vdev->dev.parent;

	virtio_i2c = devm_kzalloc(&vdev->dev, sizeof(*virtio_i2c), GFP_KERNEL);
	if (!virtio_i2c)
		return -ENOMEM;

	mutex_init(&virtio_i2c->i2c_lock);
	init_completion(&virtio_i2c->completion);
	virtio_i2c->vdev = vdev;
	ret = virtio_i2c_init(virtio_i2c);
	if (ret)
		return ret;

	virtio_i2c->adap = virtio_adapter;
	/* set adapter data, used by i2c_get_adapdata */
	i2c_set_adapdata(&virtio_i2c->adap, virtio_i2c);

	virtio_i2c->adap.dev.parent = &vdev->dev;
	/* set priv, used by virtio_i2c_adapter*/
	vdev->priv = virtio_i2c;

	/* setup acpi node for slaves device which probed through acpi */
	ACPI_COMPANION_SET(&virtio_i2c->adap.dev, ACPI_COMPANION(pdev));
	virtio_i2c->adap.timeout = VIRTIO_I2C_TIMEOUT;

	ret = i2c_add_adapter(&virtio_i2c->adap);
	if (ret) {
		dev_err(&vdev->dev, "failed to add virtio-i2c adapter\n");
		return ret;
	}

	dev_info(&vdev->dev, "virtio i2c adapter probe done!\n");
	return ret;
}

static void virtio_i2c_remove(struct virtio_device *vdev)
{
	struct virtio_i2c *i2c = virtio_i2c_adapter(vdev);

	i2c_del_adapter(&i2c->adap);
	virtio_i2c_remove_vqs(vdev);
}

#ifndef VIRTIO_ID_I2C
#define VIRTIO_ID_I2C	0xFFF6
#endif
static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_I2C, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

#ifdef CONFIG_PM_SLEEP
static int virtio_i2c_freeze(struct virtio_device *vdev)
{
	virtio_i2c_remove_vqs(vdev);
	return 0;
}

static int virtio_i2c_restore(struct virtio_device *vdev)
{
	return virtio_i2c_init(vdev->priv);
}
#endif

static struct virtio_driver virtio_i2c_driver = {
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.probe = virtio_i2c_probe,
	.remove = virtio_i2c_remove,
#ifdef CONFIG_PM_SLEEP
	.freeze = virtio_i2c_freeze,
	.restore = virtio_i2c_restore,
#endif
};

static int __init init(void)
{
	return register_virtio_driver(&virtio_i2c_driver);
}

static void __exit fini(void)
{
	unregister_virtio_driver(&virtio_i2c_driver);
}

module_init(init);
module_exit(fini);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio i2c adpater driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");
