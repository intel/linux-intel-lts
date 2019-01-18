// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/*
 * Virtio GPIO Front End Driver
 *
 * Copyright (c) 2019 Intel Corporation. All rights reserved.
 */
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/acpi.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/gpio/driver.h>

#ifndef VIRTIO_ID_GPIO
#define	VIRTIO_ID_GPIO	0xFFF7
#endif

#define GPIO_VIRTIO_F_CHIP	0
#define GPIO_VIRTIO_TIMEOUT	500

enum gpio_virtio_request_command {
	GPIO_REQ_SET_VALUE		= 0,
	GPIO_REQ_GET_VALUE		= 1,
	GPIO_REQ_INPUT_DIRECTION	= 2,
	GPIO_REQ_OUTPUT_DIRECTION	= 3,
	GPIO_REQ_GET_DIRECTION		= 4,
	GPIO_REQ_SET_CONFIG		= 5,

	GPIO_REQ_MAX
};

struct gpio_virtio_request {
	uint8_t		cmd;
	uint8_t		offset;
	uint64_t	data;
} __packed;

struct gpio_virtio_response {
	int8_t	err;
	uint8_t	data;
} __packed;

struct gpio_virtio_info {
	struct gpio_virtio_request req;
	struct gpio_virtio_response rsp;
} __packed;

struct gpio_virtio_config {
	uint16_t	base;	/* base number */
	uint16_t	ngpio;	/* number of gpios */
} __packed;

struct gpio_virtio_data {
	char	name[32];
} __packed;

struct gpio_virtio {
	struct device *dev;
	struct virtio_device *vdev;
	struct virtqueue *gpio_vq;
	struct gpio_chip chip;
	struct gpio_virtio_data *data;
	const char **names;
	struct mutex gpio_lock;
};

static unsigned int features[] = {GPIO_VIRTIO_F_CHIP};

static int gpio_virtio_update(struct gpio_virtio *vgpio,
		unsigned int cmd, unsigned int offset, unsigned long data)
{
	struct gpio_virtio_info *info;
	struct scatterlist out, in, *sgs[2];
	unsigned long timeout;
	int rc = 0;
	int len;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		rc = -ENOMEM;
		goto out;
	}

	info->req.cmd = cmd;
	info->req.offset = offset;
	if (cmd == GPIO_REQ_SET_VALUE || cmd == GPIO_REQ_OUTPUT_DIRECTION
			|| cmd == GPIO_REQ_SET_CONFIG)
		info->req.data = data;

	sg_init_one(&out, &info->req, sizeof(info->req));
	sg_init_one(&in, &info->rsp, sizeof(info->rsp));
	sgs[0] = &out;
	sgs[1] = &in;
	mutex_lock(&vgpio->gpio_lock);
	rc = virtqueue_add_sgs(vgpio->gpio_vq, sgs, 1, 1, info, GFP_KERNEL);
	if (rc) {
		mutex_unlock(&vgpio->gpio_lock);
		goto out;
	}
	virtqueue_kick(vgpio->gpio_vq);

	/* polling for the virtqueue */
	rc = -1;
	timeout = jiffies + msecs_to_jiffies(GPIO_VIRTIO_TIMEOUT);
	do {
		if (virtqueue_get_buf(vgpio->gpio_vq, &len)) {
			if (len == sizeof(info->rsp) && !info->rsp.err)
				rc = 0;
			else
				rc = -EINVAL;
			break;
		}

		/* polling interval is 5 - 10 ms */
		usleep_range(5000, 10000);
	} while (time_before(jiffies, timeout));
	if (rc == 0) {
		if (info->req.cmd == GPIO_REQ_GET_DIRECTION ||
				info->req.cmd == GPIO_REQ_GET_VALUE)
			rc = info->rsp.data;
	} else if (rc == -EINVAL) {
		dev_err(&vgpio->vdev->dev, "gpio response error %d, len %d\n",
					info->rsp.err, len);
	} else {
		/* Try to get buf */
		if (virtqueue_get_buf(vgpio->gpio_vq, &len)) {
			if (len == sizeof(info->rsp) && !info->rsp.err)
				rc = 0;
			else
				dev_err(&vgpio->vdev->dev,
					"gpio response error %d, len %d\n",
					info->rsp.err, len);
		} else {
			dev_err(&vgpio->vdev->dev,
				"gpio repsonse timeout %d ms\n",
				GPIO_VIRTIO_TIMEOUT);
		}
	}
	mutex_unlock(&vgpio->gpio_lock);
out:
	kfree(info);
	return rc;
}

static int gpio_virtio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	return gpio_virtio_update(gpiochip_get_data(chip),
			GPIO_REQ_GET_DIRECTION, offset, 0);
}

static int gpio_virtio_direction_input(struct gpio_chip *chip,
		unsigned int offset)
{
	return gpio_virtio_update(gpiochip_get_data(chip),
			GPIO_REQ_INPUT_DIRECTION, offset, 0);
}

static int gpio_virtio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	return gpio_virtio_update(gpiochip_get_data(chip),
			GPIO_REQ_OUTPUT_DIRECTION, offset, value);
}

static int gpio_virtio_get(struct gpio_chip *chip,
		unsigned int offset)
{
	return gpio_virtio_update(gpiochip_get_data(chip),
			GPIO_REQ_GET_VALUE, offset, 0);
}

static void gpio_virtio_set(struct gpio_chip *chip, unsigned int offset,
		int value)
{
	gpio_virtio_update(gpiochip_get_data(chip),
			GPIO_REQ_SET_VALUE, offset, value);
}

static int gpio_virtio_set_config(struct gpio_chip *chip, unsigned int offset,
			    unsigned long config)
{
	return gpio_virtio_update(gpiochip_get_data(chip),
			GPIO_REQ_SET_CONFIG, offset, config);
}

static const struct gpio_chip gpio_virtio_chip = {
	.owner = THIS_MODULE,
	.label = "gpio_virtio",
	.get_direction = gpio_virtio_get_direction,
	.direction_input = gpio_virtio_direction_input,
	.direction_output = gpio_virtio_direction_output,
	.get = gpio_virtio_get,
	.set = gpio_virtio_set,
	.set_config = gpio_virtio_set_config,
};

static int gpio_virtio_register_chip(struct gpio_virtio *vgpio,
		struct device *pdev)
{
	struct scatterlist sg;
	unsigned long timeout;
	int err, i, len;
	u16 base, n = 0;

	if (virtio_has_feature(vgpio->vdev, GPIO_VIRTIO_F_CHIP)) {
		virtio_cread(vgpio->vdev, struct gpio_virtio_config,
				base, &base);
		virtio_cread(vgpio->vdev, struct gpio_virtio_config,
				ngpio, &n);
	} else {
		dev_err(&vgpio->vdev->dev, "failed to get virtio feature\n");
		return -ENODEV;
	}

	if (n == 0) {
		dev_err(&vgpio->vdev->dev, "number of gpio is invalid\n");
		return -EINVAL;
	}

	vgpio->chip = gpio_virtio_chip;
	vgpio->chip.base = base;
	vgpio->chip.ngpio = n;
	if (pdev && ACPI_COMPANION(pdev->parent))
		vgpio->chip.parent = pdev->parent;

	/* initialize gpio names */
	vgpio->names = kcalloc(n, sizeof(*vgpio->names), GFP_KERNEL);
	vgpio->data = kcalloc(n, sizeof(*vgpio->data), GFP_KERNEL);
	if (!vgpio->data || !vgpio->names) {
		dev_err(&vgpio->vdev->dev, "failed to alloc names and data\n");
		err = -ENOMEM;
		goto out;
	}
	sg_init_one(&sg, vgpio->data, n * sizeof(*vgpio->data));
	err = virtqueue_add_inbuf(vgpio->gpio_vq, &sg, 1, vgpio->data,
			GFP_KERNEL);
	if (err)
		goto out;

	virtqueue_kick(vgpio->gpio_vq);

	/* polling for the virtqueue */
	err = -1;
	timeout = jiffies + msecs_to_jiffies(GPIO_VIRTIO_TIMEOUT);
	do {
		if (virtqueue_get_buf(vgpio->gpio_vq, &len)) {
			if (n == len)
				err = 0;
			break;
		}

		/* polling interval is 5 - 10 ms */
		usleep_range(5000, 10000);
	} while (time_before(jiffies, timeout));

	if (err) {
		dev_err(&vgpio->vdev->dev, "gpio name is invalid,n=%u,len=%d\n",
				n, len);
		err = -EINVAL;
		goto out;
	}

	/*
	 * The buffer that allocates n gpio names, the backend will copy len
	 * gpio names inito it. If the buffer return NULL or n and len are
	 * not match, the initialization fails.
	 */
	for (i = 0; i < n; i++)
		vgpio->names[i] = vgpio->data[i].name;

	vgpio->chip.names = vgpio->names;
	err = gpiochip_add_data(&vgpio->chip, vgpio);
	if (err)
		goto out;

	return 0;
out:
	kfree(vgpio->names);
	kfree(vgpio->data);
	return err;
}

static int init_vqs(struct gpio_virtio *vgpio)
{
	struct virtqueue *vqs[1];
	vq_callback_t *callbacks[1] = {NULL};
	const char * const names[1] = {"gpio"};
	int err;

	err = virtio_find_vqs(vgpio->vdev, 1, vqs, callbacks, names, NULL);
	if (err)
		return err;

	vgpio->gpio_vq = vqs[0];
	return 0;
}

static int gpio_virtio_probe(struct virtio_device *vdev)
{
	struct gpio_virtio *vgpio;
	struct device *pdev;
	int err;

	vgpio = kzalloc(sizeof(*vgpio), GFP_KERNEL);
	if (!vgpio)
		return -ENOMEM;

	pdev = &vdev->dev;
	vdev->priv = vgpio;
	vgpio->vdev = vdev;
	mutex_init(&vgpio->gpio_lock);
	err = init_vqs(vgpio);
	if (err)
		goto out;

	err = gpio_virtio_register_chip(vgpio, pdev);
	if (err)
		goto out;

	return 0;
out:
	dev_err(&vgpio->vdev->dev, "failed to initialize gpio virtio\n");
	kfree(vgpio);
	return err;
}

static void gpio_virtio_remove(struct virtio_device *vdev)
{
	struct gpio_virtio *gpio = vdev->priv;

	/* Disable virtqueues. */
	if (vdev->config->reset)
		vdev->config->reset(vdev);
	if (vdev->config->del_vqs)
		vdev->config->del_vqs(vdev);

	gpiochip_remove(&gpio->chip);

	kfree(gpio->data);
	kfree(gpio->names);
	kfree(gpio);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_GPIO, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

#ifdef CONFIG_PM_SLEEP
static int gpio_virtio_freeze(struct virtio_device *vdev)
{
	/* Disable virtqueues. */
	if (vdev->config->reset)
		vdev->config->reset(vdev);
	if (vdev->config->del_vqs)
		vdev->config->del_vqs(vdev);
	return 0;
}

static int gpio_virtio_restore(struct virtio_device *vdev)
{
	return init_vqs(vdev->priv);
}
#endif

static struct virtio_driver gpio_virtio_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	gpio_virtio_probe,
	.remove =	gpio_virtio_remove,
#ifdef CONFIG_PM_SLEEP
	.freeze =	gpio_virtio_freeze,
	.restore =	gpio_virtio_restore,
#endif
};

static int __init gpio_virtio_init(void)
{
	return register_virtio_driver(&gpio_virtio_driver);
}

static void __exit gpio_virtio_exit(void)
{
	unregister_virtio_driver(&gpio_virtio_driver);
}

subsys_initcall(gpio_virtio_init);
module_exit(gpio_virtio_exit);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("GPIO virtio frontend driver");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");
