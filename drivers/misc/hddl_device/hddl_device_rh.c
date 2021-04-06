// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * High Density Deep Learning Kernel module.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 */

#include <asm/page.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hddl_device.h>
#include <linux/i2c.h>
#include <linux/intel_tsens_host.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/xlink.h>

#include <uapi/linux/stat.h>

#include "hddl_device_util.h"

#define DRIVER_NAME "hddl_device_server"

/*
 * I2C client Reserved addr: 0x00 - 0x0f
 *			     0xf0 - 0xff
 */
#define HDDL_FREE_CLIENT_ADDR_START	0x10
#define HDDL_FREE_CLIENT_ADDR_END	0xf0
#define HDDL_FREE_CLIENT_ADDR_SIZE	(HDDL_FREE_CLIENT_ADDR_END - \
					HDDL_FREE_CLIENT_ADDR_START)
/* Xlink channel reserved for HDDL device management
 * HDDL_NODE_XLINK_CHANNEL - Default channel for HDDL device
 *				Management communication.
 * HDDL_I2C_XLINK_CHANNEL - channel used for xlink I2C
 *				communication.
 */
#define HDDL_NODE_XLINK_CHANNEL	1080
#define HDDL_I2C_XLINK_CHANNEL		1081

#define HDDL_RESET_SUCCESS	1
#define HDDL_RESET_FAILED	0

static const int hddl_host_reserved_addrs[] = {
	0x42,
	0x52,
	0x54,
	0x60
};

struct intel_hddl_server_plat_data {
	u32 xlink_chan;
	u32 i2c_xlink_chan;
};

struct intel_hddl_device_priv {
	u32 xlink_chan;
	u32 i2c_xlink_chan;
	u32 ndevs;
	DECLARE_BITMAP(client_addr, HDDL_FREE_CLIENT_ADDR_SIZE);
	/* HDDL device lock */
	struct mutex lock;
	struct platform_device *pdev;
	struct intel_hddl_clients **hddl_client;
	struct task_struct *hddl_dev_init_task;
	struct intel_hddl_server_plat_data *plat_data;
	struct i2c_adapter *smbus_adap;
	struct class *dev_class;
	struct cdev hddl_cdev;
	dev_t cdev;
};

static struct intel_hddl_device_priv *g_priv;

static inline int intel_hddl_get_xlink_data(struct device *dev,
					    struct xlink_handle *xlink,
					    int chan_num, u8 *msg,
					    int *size)
{
	int rc;

	rc = xlink_read_data_to_buffer(xlink, chan_num,
				       msg, size);
	if (rc) {
		dev_err(dev,
			"HDDL: xlink read data failed rc = %d\n",
			rc);
		return -EFAULT;
	}
	rc = xlink_release_data(xlink, chan_num, NULL);
	if (rc) {
		dev_err(dev,
			"HDDL: xlink release failed rc = %d\n",
			rc);
		return -EFAULT;
	}
	return rc;
}

struct intel_hddl_clients **intel_hddl_get_clients(int *n_devs)
{
	if (!g_priv)
		return NULL;
	*n_devs = g_priv->ndevs;
	return g_priv->hddl_client;
}

static long hddl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct intel_hddl_device_priv *priv = file->private_data;
	u32 __user *user_ptr = (u32 __user *)arg;
	struct device *dev = &priv->pdev->dev;
	struct sw_id_hddl_status swid_status;
	struct intel_hddl_clients **clients;
	struct sw_id_soft_reset soft_reset;
	struct intel_hddl_clients *client;
	struct sw_id_hddl_data swid_data;
	int i, rc;

	if (!user_ptr) {
		dev_err(dev, "Null pointer from user\n");
		return -EINVAL;
	}
	if (!priv) {
		dev_err(dev, "Device ioctl failed\n");
		return -ENODEV;
	}
	clients = priv->hddl_client;
	if (!clients) {
		dev_err(dev, "Device ioctl failed\n");
		return -ENODEV;
	}
	switch (cmd) {
	case HDDL_SOFT_RESET:
		if (copy_from_user(&soft_reset,
				   user_ptr,
				   sizeof(struct sw_id_soft_reset)))
			return -EFAULT;
		for (i = 0; i < priv->ndevs; i++) {
			if (clients[i]->xlink_dev.sw_device_id ==
					soft_reset.sw_id) {
				client = clients[i];
				break;
			}
		}

		if (!client) {
			dev_err(dev, "target device to reset not found %d",
				soft_reset.sw_id);
			return -ENODEV;
		}
		/* xlink-reset */
		rc =  xlink_reset_device(&client->xlink_dev);
		if (rc > 0) {
			dev_err(dev, "xlink_reset_device failed");
			soft_reset.return_id = HDDL_RESET_FAILED;
		} else {
			soft_reset.return_id = HDDL_RESET_SUCCESS;
		}
		if (copy_to_user(user_ptr,
				 &soft_reset, sizeof(struct sw_id_soft_reset)))
			return -EFAULT;
		/* xlink-rest */
		break;
	case HDDL_READ_SW_ID_DATA:
		if (copy_from_user(&swid_data, user_ptr,
				   sizeof(struct sw_id_hddl_data)))
			return -EFAULT;
		for (i = 0; i < priv->ndevs; i++) {
			if (clients[i]->xlink_dev.sw_device_id ==
					swid_data.sw_id) {
				client = clients[i];
				break;
			}
		}

		if (!client) {
			dev_err(dev, "target device to reset not found %d",
				swid_data.sw_id);
			return -ENODEV;
		}
		strcpy(swid_data.board_type, d->board_info.board_type);
		swid_data.board_id = client->board_info.board_id;
		swid_data.soc_id = client->board_info.soc_id;
		swid_data.iox_addr = client->board_info.iox_addr;
		swid_data.iox_pin = client->board_info.iox_pin;
		swid_data.pci_pin = client->board_info.pci_pin;
		strcpy(swid_data.iox_name, client->board_info.iox_name);
		if (client->adap[0])
			swid_data.soc_adaptor_no[0] = client->adap[0]->nr;
		if (client->adap[1])
			swid_data.soc_adaptor_no[1] = client->adap[1]->nr;
		swid_data.return_id = 1;
		if (copy_to_user(user_ptr,
				 &swid_data, sizeof(struct sw_id_hddl_data)))
			return -EFAULT;
		break;
	case HDDL_READ_STATUS:
		if (copy_from_user(&swid_status, user_ptr,
				   sizeof(struct sw_id_hddl_status)))
			return -EFAULT;
		for (i = 0; i < priv->ndevs; i++) {
			if (clients[i]->xlink_dev.sw_device_id ==
					swid_status.sw_id) {
				client = clients[i];
				break;
			}
		}
		if (!client) {
			dev_err(dev, "device to get status not found %d",
				swid_status.sw_id);
			return -ENODEV;
		}
		swid_status.status = client->status;
		swid_status.return_id = 1;
		if (copy_to_user(user_ptr, &swid_status,
				 sizeof(struct sw_id_hddl_status)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int hddl_open(struct inode *inode, struct file *filp)
{
	struct intel_hddl_device_priv *priv;

	priv = container_of(inode->i_cdev,
			    struct intel_hddl_device_priv, hddl_cdev);
	if (!priv) {
		dev_err(&priv->pdev->dev, "Device open failed\n");
		return -ENODEV;
	}
	filp->private_data = priv;
	return 0;
}

static const struct file_operations hddl_fops = {
	.owner	= THIS_MODULE,
	.open = hddl_open,
	.unlocked_ioctl = hddl_ioctl,
};

static int intel_hddl_cdev_init(struct intel_hddl_device_priv *priv)
{
	/*Allocating Major number*/
	if ((alloc_chrdev_region(&priv->cdev, 0, 1, "hddl_dev")) < 0) {
		dev_err(&priv->pdev->dev, "Cannot allocate major number\n");
		return -EINVAL;
	}
	dev_err(&priv->pdev->dev, "Major = %d Minor = %d\n", MAJOR(priv->cdev),
		MINOR(priv->cdev));
	/*Creating cdev structure*/
	cdev_init(&priv->hddl_cdev, &hddl_fops);
	/*Adding character device to the system*/
	if ((cdev_add(&priv->hddl_cdev, priv->cdev, 1)) < 0) {
		dev_err(&priv->pdev->dev,
			"Cannot add the device to the system\n");
		goto r_region;
	}
	/*Creating struct class*/
	priv->dev_class = class_create(THIS_MODULE, "hddl_class");
	if (!priv->dev_class) {
		dev_err(&priv->pdev->dev, "Cannot create the struct class\n");
		goto r_device;
	}
	/*Creating device*/
	if (!(device_create(priv->dev_class, NULL, priv->cdev, NULL,
			    "hddl_device"))) {
		dev_err(&priv->pdev->dev, "Cannot create the Device\n");
		goto r_class;
	}
	return 0;

r_class:
	class_destroy(priv->dev_class);
r_device:
	cdev_del(&priv->hddl_cdev);
r_region:
	unregister_chrdev_region(priv->cdev, 1);
	return -EINVAL;
}

static void intel_hddl_cdev_remove(struct intel_hddl_device_priv *priv)
{
	device_destroy(priv->dev_class, priv->cdev);
	class_destroy(priv->dev_class);
	cdev_del(&priv->hddl_cdev);
	unregister_chrdev_region(priv->cdev, 1);
}

void intel_hddl_unregister_pdev(struct intel_hddl_clients *c)
{
	struct intel_hddl_device_priv *priv = c->pdata;

	intel_hddl_xlink_remove_i2c_adap(&priv->pdev->dev, c);
}

void intel_hddl_free_i2c_client(struct intel_hddl_clients *d,
				struct intel_hddl_i2c_devs *i2c_dev)
{
	struct intel_hddl_device_priv *priv = d->pdata;
	int bit_pos = i2c_dev->addr - HDDL_FREE_CLIENT_ADDR_START;

	if (i2c_dev->xlk_client)
		i2c_unregister_device(i2c_dev->xlk_client);
	if (i2c_dev->i2c_client)
		i2c_unregister_device(i2c_dev->i2c_client);
	if (i2c_dev->smbus_client)
		i2c_unregister_device(i2c_dev->smbus_client);
	i2c_dev->xlk_client = NULL;
	i2c_dev->i2c_client = NULL;
	i2c_dev->smbus_client = NULL;
	mutex_lock(&priv->lock);
	clear_bit(bit_pos, priv->client_addr);
	mutex_unlock(&priv->lock);
}

/** hddl_get_free_client - get free client address,
 *
 * https://i2c.info/i2c-bus-specification
 * below client address are reserved as per i2c bus specification.
 * I2C client Reserved addr: 0x00 - 0x0f
 *			     0xf0 - 0xff
 *
 * Get free client address other than standard i2c clients reserved and
 * i2c client address used by host. If any free client address found,
 * mark it as reserved by setting the bit corresponding to the address,
 * and return client address.
 */
static int hddl_get_free_client(struct intel_hddl_device_priv *priv)
{
	unsigned long bit_pos;
	int client_addr;

	bit_pos = find_first_zero_bit(priv->client_addr,
				      HDDL_FREE_CLIENT_ADDR_SIZE);
	if (bit_pos >= HDDL_FREE_CLIENT_ADDR_SIZE)
		return -EINVAL;
	client_addr = bit_pos + HDDL_FREE_CLIENT_ADDR_START;
	set_bit(bit_pos, priv->client_addr);
	return client_addr;
}

static int intel_hddl_i2c_register_clients(struct device *dev,
					   struct intel_hddl_clients *c)
{
	struct intel_hddl_device_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_hddl_i2c_devs **i2c_devs;
	struct intel_hddl_tsens_msg msg;
	int rc, ndevs, size, i;

	/* Get N I2C devices */
	msg.msg_type = HDDL_GET_N_I2C_DEVS;
	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&msg, sizeof(msg));
	if (rc) {
		dev_err(dev,
			"xlink write data failed rc = %d\n",
			rc);
		return rc;
	}
	rc = intel_hddl_get_xlink_data(dev,
				       xlink, c->chan_num,
				       (u8 *)&ndevs, &size);
	if (rc)
		return rc;
	c->n_clients = ndevs;
	i2c_devs = devm_kcalloc(dev, ndevs,
				sizeof(struct intel_hddl_i2c_devs *),
				GFP_KERNEL);
	if (!i2c_devs)
		return -ENOMEM;
	c->i2c_devs = i2c_devs;
	for (i = 0; i < ndevs; i++) {
		struct intel_hddl_i2c_devs *i2c;
		struct intel_hddl_i2c_devs_data i2c_data;

		i2c = devm_kzalloc(dev,
				   sizeof(struct intel_hddl_i2c_devs),
				   GFP_KERNEL);
		if (!i2c)
			return -ENOMEM;
		i2c_devs[i] = i2c;

		/* Get Details*/
		msg.msg_type = HDDL_GET_I2C_DEVS;
		msg.sensor_type = i;
		rc = xlink_write_volatile(xlink, c->chan_num,
					  (u8 *)&msg, sizeof(msg));
		if (rc) {
			dev_err(dev, "xlink write data failed rc = %d\n", rc);
			return rc;
		}
		rc = intel_hddl_get_xlink_data(dev,
					       xlink, c->chan_num,
					       (u8 *)&i2c_data, &size);
		if (rc)
			return rc;

		strcpy(i2c->name, i2c_data.name);
		i2c->addr = i2c_data.addr;
		i2c->bus = i2c_data.bus;
		i2c->enabled = i2c_data.enabled;
		i2c->local_host = i2c_data.local_host;
		i2c->remote_host = i2c_data.remote_host;
	}

	mutex_lock(&priv->lock);
	for (i = 0; i < ndevs; i++) {
		if (i2c_devs[i]->addr & (1 << 30))
			i2c_devs[i]->addr = hddl_get_free_client(priv);

		strcpy(i2c_devs[i]->board_info.type,
		       i2c_devs[i]->name);
		i2c_devs[i]->board_info.addr = i2c_devs[i]->addr;
	}
	mutex_unlock(&priv->lock);
	/* Send Complete */
	msg.msg_type = HDDL_GET_SENS_COMPLETE;
	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&msg, sizeof(msg));
	if (rc) {
		dev_err(dev, "xlink write data failed rc = %d\n", rc);
		return rc;
	}

	mutex_lock(&priv->lock);

	/* Get msg type */
	rc = intel_hddl_get_xlink_data(dev,
				       xlink, c->chan_num,
				       (u8 *)&msg, &size);
	if (rc) {
		mutex_unlock(&priv->lock);
		return rc;
	}

	while (msg.msg_type != HDDL_GET_SENS_COMPLETE) {
		switch (msg.msg_type) {
		case HDDL_GET_I2C_DEV_ADDR:
		{
			i = msg.sensor_type;
			rc = xlink_write_volatile(xlink, c->chan_num,
						  (u8 *)&i2c_devs[i]->addr,
						  sizeof(i2c_devs[i]->addr));
			if (rc) {
				dev_err(dev,
					"xlink write data failed rc = %d\n",
					rc);
				mutex_unlock(&priv->lock);
				return rc;
			}
		}
		break;
		default:
			break;
		}
		rc = intel_hddl_get_xlink_data(dev,
					       xlink, c->chan_num,
					       (u8 *)&msg, &size);
		if (rc) {
			mutex_unlock(&priv->lock);
			return rc;
		}
	}
	intel_hddl_add_xlink_i2c_clients(dev, c, c->i2c_devs,
					 c->n_clients, 1);
	mutex_unlock(&priv->lock);
	return 0;
}

static int intel_hddl_tsens_data(struct intel_hddl_clients *c)
{
	struct intel_hddl_device_priv *priv = c->pdata;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct intel_tsens_host **p_tsens;
	struct intel_hddl_tsens_msg msg;
	u32 size, i, j;
	u32 nsens;
	int rc;

	/* Get Nsens */
	msg.msg_type = HDDL_GET_NSENS;
	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&msg, sizeof(msg));
	if (rc) {
		dev_err(&priv->pdev->dev,
			"xlink write data failed rc = %d\n",
			rc);
			return rc;
	}
	rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
				       xlink, c->chan_num,
				       (u8 *)&nsens, &size);
	if (rc)
		return rc;

	c->nsens = nsens;
	p_tsens = devm_kcalloc(&priv->pdev->dev, nsens,
			       sizeof(struct intel_tsens_host *),
			       GFP_KERNEL);
	if (!p_tsens)
		return -ENOMEM;
	c->tsens = (void **)p_tsens;
	for (i = 0; i < nsens; i++) {
		struct intel_tsens_host *tsens;
		struct intel_tsens_data *tsens_data;

		tsens = devm_kzalloc(&priv->pdev->dev,
				     sizeof(struct intel_tsens_host),
				     GFP_KERNEL);
		if (!tsens)
			return -ENOMEM;
		tsens_data = devm_kzalloc(&priv->pdev->dev,
					  sizeof(struct intel_tsens_data),
					  GFP_KERNEL);
		if (!tsens_data)
			return -ENOMEM;
		tsens->t_data = tsens_data;

		/* Get Details*/
		msg.msg_type = HDDL_GET_SENS_DETAILS;
		msg.sensor_type = i;
		rc = xlink_write_volatile(xlink, c->chan_num,
					  (u8 *)&msg, sizeof(msg));
		if (rc) {
			dev_err(&priv->pdev->dev,
				"xlink write data failed rc = %d\n",
				rc);
			return rc;
		}
		rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
					       xlink, c->chan_num,
					       (u8 *)tsens_data, &size);
		if (rc)
			return rc;

		/* Get trip info*/
		tsens->trip_info =
		devm_kcalloc(&priv->pdev->dev, tsens_data->n_trips,
			     sizeof(struct intel_tsens_host_trip_info *),
			     GFP_KERNEL);
		if (!tsens->trip_info)
			return -ENOMEM;
		for (j = 0; j < tsens_data->n_trips; j++) {
			struct intel_tsens_host_trip_info *t_info;

			t_info =
			devm_kzalloc(&priv->pdev->dev,
				     sizeof(struct intel_tsens_host_trip_info),
				     GFP_KERNEL);
			if (!t_info)
				return -ENOMEM;
			tsens->trip_info[j] = t_info;
			msg.msg_type = HDDL_GET_SENS_TRIP_INFO;
			msg.sensor_type = i;
			msg.trip_info_idx = j;
			rc = xlink_write_volatile(xlink, c->chan_num,
						  (u8 *)&msg, sizeof(msg));
			if (rc) {
				dev_err(&priv->pdev->dev,
					"xlink write data failed rc = %d\n",
					rc);
				return rc;
			}
			rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
						       xlink, c->chan_num,
						       (u8 *)t_info, &size);
			if (rc)
				return rc;
		}
		p_tsens[i] = tsens;
	}
	/* Send Complete */
	msg.msg_type = HDDL_GET_SENS_COMPLETE;
	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&msg, sizeof(msg));
	if (rc) {
		dev_err(&priv->pdev->dev,
			"xlink write data failed rc = %d\n",
			rc);
		return rc;
	}

	return 0;
}

static int intel_hddl_device_connect_task(void *data)
{
	struct intel_hddl_clients *c = (struct intel_hddl_clients *)data;
	struct intel_hddl_device_priv *priv = c->pdata;
	struct intel_hddl_board_info board_info_rcvd;
	struct xlink_handle *xlink = &c->xlink_dev;
	struct timespec64 ts;
	u32 size, rc;

	c->chan_num = priv->xlink_chan;
	c->i2c_chan_num = priv->i2c_xlink_chan;
	c->smbus_adap = priv->smbus_adap;
	c->status = HDDL_DEV_STATUS_START;
	if (intel_hddl_open_xlink_device(&priv->pdev->dev, c)) {
		dev_err(&priv->pdev->dev, "HDDL open xlink dev failed\n");
		return -ENODEV;
	}
	c->status = HDDL_DEV_STATUS_XLINK_OPENED;
	ktime_get_real_ts64(&ts);
	rc = xlink_write_volatile(xlink, c->chan_num, (u8 *)&ts,
				  sizeof(struct timespec64));
	if (rc) {
		dev_err(&priv->pdev->dev,
			"xlink write data failed rc = %d\n",
			rc);
		return rc;
	}
	c->status = HDDL_DEV_STATUS_UPDATED_TIMESTAMP;
	size = sizeof(c->board_info);
	rc = intel_hddl_get_xlink_data(&priv->pdev->dev,
				       xlink, c->chan_num,
				       (u8 *)&c->board_info, &size);
	if (rc)
		return rc;
	board_info_rcvd.board_id = ~(c->board_info.board_id);
	rc = xlink_write_volatile(xlink, c->chan_num,
				  (u8 *)&board_info_rcvd,
				  sizeof(board_info_rcvd));
	if (rc) {
		dev_err(&priv->pdev->dev,
			"xlink write data failed rc = %d\n",
			rc);
		return rc;
	}
	c->status = HDDL_DEV_STATUS_UPDATED_BOARD_INFO;
	rc = intel_hddl_tsens_data(c);
	if (rc) {
		dev_err(&priv->pdev->dev, "HDDL: tsens data not rcvd\n");
		goto close_xlink_dev;
	}
	c->status = HDDL_DEV_STATUS_UPDATED_THERMAL_INFO;
	rc = intel_hddl_register_xlink_i2c_adap(&priv->pdev->dev, c);
	if (rc) {
		dev_err(&priv->pdev->dev,
			"HDDL: register xlink i2c adapter failed\n");
		goto close_xlink_dev;
	}
	c->status = HDDL_DEV_STATUS_UPDATED_I2C_ADAPTERS;
	rc = intel_hddl_i2c_register_clients(&priv->pdev->dev, c);
	if (rc) {
		dev_err(&priv->pdev->dev,
			"HDDL: register i2c clients failed\n");
		goto remove_xlink_i2c_adap;
	}
	c->status = HDDL_DEV_STATUS_CONNECTED;
	while (!kthread_should_stop())
		msleep_interruptible(HDDL_NEW_DEV_POLL_TIME);

	return 0;

remove_xlink_i2c_adap:
	intel_hddl_xlink_remove_i2c_adap(&priv->pdev->dev, c);
close_xlink_dev:
	intel_hddl_close_xlink_device(&priv->pdev->dev, c);
	return rc;
}

static int intel_hddl_check_for_new_device(struct intel_hddl_device_priv *priv)
{
	struct intel_hddl_clients **hddl_clients;

	hddl_clients =
		intel_hddl_setup_device(&priv->pdev->dev,
					intel_hddl_device_connect_task,
					&priv->ndevs, priv->hddl_client,
					priv);

	if (!hddl_clients) {
		dev_err(&priv->pdev->dev,
			"intel_hddl_setup_device returned NULL\n");
		return 0;
	}
	priv->hddl_client = hddl_clients;
	return 1;
}

static int intel_hddl_device_init_task(void *data)
{
	struct intel_hddl_device_priv *priv =
		(struct intel_hddl_device_priv *)data;

	while (!kthread_should_stop()) {
		if (!intel_hddl_check_for_new_device(priv)) {
			dev_err(&priv->pdev->dev,
				"Error while checking for new device\n");
			return -EFAULT;
		}
		msleep_interruptible(HDDL_NEW_DEV_POLL_TIME);
	}

	return 0;
}

static int intel_hddl_device_init(struct intel_hddl_device_priv *priv)
{
	struct i2c_adapter *temp;
	int j = 0;

	while ((temp = i2c_get_adapter(j))) {
		if (strstr(temp->name, "SMBus I801"))
			priv->smbus_adap = temp;
		i2c_put_adapter(temp);
		j++;
	}
	priv->hddl_dev_init_task = kthread_run(intel_hddl_device_init_task,
					       (void *)priv,
					       "hddl_device_init");
	if (!priv->hddl_dev_init_task) {
		dev_err(&priv->pdev->dev, "failed to create thread\n");
		return -EINVAL;
	}

	return 0;
}

static int intel_hddl_server_probe(struct platform_device *pdev)
{
	struct intel_hddl_server_plat_data *plat_data;
	struct intel_hddl_device_priv *priv;
	int ret, i;

	plat_data = pdev->dev.platform_data;
	if (!plat_data) {
		dev_err(&pdev->dev, "Platform data not found\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct intel_hddl_device_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pdev = pdev;
	priv->plat_data = plat_data;
	priv->xlink_chan = plat_data->xlink_chan;
	priv->i2c_xlink_chan = plat_data->i2c_xlink_chan;
	mutex_init(&priv->lock);
	g_priv = priv;
	ret = intel_hddl_cdev_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "HDDL char device init failed\n");
		return -EINVAL;
	}
	/*
	 * https://i2c.info/i2c-bus-specification
	 * below client address are reserved as per i2c bus specification.
	 * I2C client Reserved addr: 0x00 - 0x0f
	 *			     0xf0 - 0xff
	 *
	 * hddl_get_free_client will not use standard i2c client
	 * reserved address, so no need to mark them as reserved.
	 * mark the address used by i2c clients connected to the host
	 * as used.
	 */
	for (i = 0; i < ARRAY_SIZE(hddl_host_reserved_addrs); i++) {
		int bit_pos = hddl_host_reserved_addrs[i] -
				HDDL_FREE_CLIENT_ADDR_START;
		set_bit(bit_pos, priv->client_addr);
	}
	ret = intel_hddl_device_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "HDDL device init failed\n");
		ret = -EINVAL;
		goto free_cdev;
	}
	platform_set_drvdata(pdev, priv);

	return 0;
free_cdev:
	intel_hddl_cdev_remove(priv);
	return ret;
}

/* Device Exit */
static int intel_hddl_server_remove(struct platform_device *pdev)
{
	struct intel_hddl_device_priv *priv = platform_get_drvdata(pdev);
	int i;

	if (!priv)
		return -EINVAL;
	intel_hddl_cdev_remove(priv);
	for (i = 0; i < priv->ndevs; i++)
		intel_hddl_device_remove(priv->hddl_client[i]);
	kthread_stop(priv->hddl_dev_init_task);

	return 0;
}

static struct platform_driver intel_hddl_server_driver = {
	.probe = intel_hddl_server_probe,
	.remove = intel_hddl_server_remove,
	.driver = {
		.name = "intel_hddl_server",
	},
};

static struct platform_device *intel_hddl_server_pdev;

static void intel_hddl_server_exit(void)
{
	platform_driver_unregister(&intel_hddl_server_driver);
	platform_device_unregister(intel_hddl_server_pdev);
}

static int __init intel_hddl_server_init(void)
{
	struct intel_hddl_server_plat_data plat;
	struct platform_device_info pdevinfo;
	struct platform_device *dd;
	int ret;

	ret = platform_driver_register(&intel_hddl_server_driver);
	if (ret) {
		pr_err("HDDL SERVER: platform_driver_register failed %d", ret);
		return ret;
	}
	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.name = "intel_hddl_server";
	pdevinfo.data = &plat;
	plat.xlink_chan = HDDL_NODE_XLINK_CHANNEL;
	plat.i2c_xlink_chan = HDDL_I2C_XLINK_CHANNEL;
	pdevinfo.size_data = sizeof(struct  intel_hddl_server_plat_data);
	dd = platform_device_register_full(&pdevinfo);
	if (IS_ERR(dd)) {
		pr_err("HDDL SERVER: platform device register failed\n");
		platform_driver_unregister(&intel_hddl_server_driver);
		return -EINVAL;
	}
	intel_hddl_server_pdev = dd;
	return 0;
}

module_init(intel_hddl_server_init);
module_exit(intel_hddl_server_exit);

MODULE_DESCRIPTION("Intel HDDL Device host driver");
MODULE_AUTHOR("Sandeep Singh <sandeep1.singh@intel.com>");
MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_AUTHOR("Udhayakumar C <udhayakumar.c@intel.com>");
MODULE_LICENSE("GPL v2");
