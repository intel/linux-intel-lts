// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Xlink SMBus Driver
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <linux/hddl_device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/time.h>
#include <linux/xlink.h>

struct xlink_msg {
	u16			addr;
	u16			flags;
	u8			read_write;
	u8			command;
	u16			padding;
	u32			protocol;
	union i2c_smbus_data	data;
	s32			status;
	struct list_head	node;
};

struct xlink_adapter_data {
	struct	xlink_handle *xhandle;
	struct	completion work;
	struct	task_struct *task_recv;
	struct	i2c_client *slave;
	struct	list_head head;
	struct	i2c_adapter *adap;
	u32     channel;
};

#if defined(CONFIG_XLINK_SMBUS_PROXY)
/*
 * PROXY the commands using existing adapter
 * I2C2 is fixed for Keem Bay, it has all sensors connected
 */
#define proxy_i2c_adapter_info() i2c_get_adapter(2)
#else
/*
 * This is an adapter by itself
 * It doesn't proxy transfer on another adapter
 */
#define proxy_i2c_adapter_info() ((void *)0)
#endif

#if IS_ENABLED(CONFIG_I2C_SLAVE)
/*
 * The complete slave protocol is implemented in one shot here as
 * the whole chunk of data is transferred or received via xlink,
 * not byte-by-byte
 * Refer https://lwn.net/Articles/640346/ for protocol
 */
static s32 handle_slave_mode(struct i2c_client *slave, struct xlink_msg *msg)
{
	struct device *dev = &slave->dev;
	u8 temp;

	/* Send the command as first write */
	i2c_slave_event(slave, I2C_SLAVE_WRITE_REQUESTED, NULL);
	i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED, &msg->command);

	/* Now handle specifics to read/write */
	if (msg->read_write == I2C_SMBUS_WRITE) {
		if (msg->protocol == I2C_SMBUS_BYTE_DATA) {
			i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED,
					&msg->data.byte);
		} else if (msg->protocol == I2C_SMBUS_WORD_DATA) {
			temp = msg->data.word & 0xFF;
			i2c_slave_event(slave,
					I2C_SLAVE_WRITE_RECEIVED,
					&temp);
			temp = (msg->data.word >> 8) & 0xFF;
			i2c_slave_event(slave,
					I2C_SLAVE_WRITE_RECEIVED,
					&temp);
		} else if (msg->protocol == I2C_SMBUS_BLOCK_DATA) {
			int i;

			if (msg->data.block[0] > I2C_SMBUS_BLOCK_MAX)
				return -EPROTO;

			for (i = 1; (i < msg->data.block[0] ||
				     i <= I2C_SMBUS_BLOCK_MAX); ++i) {
				i2c_slave_event(slave,
						I2C_SLAVE_WRITE_RECEIVED,
						&msg->data.block[i]);
			}
		} else {
			dev_err(dev,
				"unknown protocol (%d) received in %s\n",
				msg->protocol,
				__func__
				);
			return -EOPNOTSUPP;
		}
	} else {
		if (msg->protocol == I2C_SMBUS_BYTE_DATA) {
			i2c_slave_event(slave,
					I2C_SLAVE_READ_REQUESTED,
					&msg->data.byte);
		} else if (msg->protocol == I2C_SMBUS_WORD_DATA) {
			i2c_slave_event(slave,
					I2C_SLAVE_READ_REQUESTED,
					&temp);
			msg->data.word = temp << 8;
			i2c_slave_event(slave,
					I2C_SLAVE_READ_REQUESTED,
					&temp);
			msg->data.word |= temp;
		} else if (msg->protocol == I2C_SMBUS_BLOCK_DATA) {
			int i;

			if (msg->data.block[0] > I2C_SMBUS_BLOCK_MAX)
				return -EPROTO;

			for (i = 1; (i < msg->data.block[0] ||
				     i <= I2C_SMBUS_BLOCK_MAX); ++i) {
				i2c_slave_event(slave,
						I2C_SLAVE_READ_REQUESTED,
						&msg->data.block[i]);
			}
		} else {
			dev_err(dev,
				"unknown protocol (%d) received in %s\n",
				msg->protocol,
				__func__);
			return -EOPNOTSUPP;
		}
		i2c_slave_event(slave, I2C_SLAVE_READ_PROCESSED, &temp);
	}
	i2c_slave_event(slave, I2C_SLAVE_STOP, NULL);
	return 0;
}
#endif /* CONFIG_I2C_SLAVE */

static s32 xlink_smbus_xfer(struct i2c_adapter *adap, u16 addr,
			    unsigned short flags, char read_write,
			    u8 command, int protocol,
			    union i2c_smbus_data *data)
{
	struct xlink_adapter_data *adapt_data = NULL;
	struct device *dev = NULL;
	struct xlink_msg tx_msg, *rx_msg;
	enum xlink_error xerr;
	s32 rc = 0;

	if (!adap)
		return -ENODEV;
	adapt_data = i2c_get_adapdata(adap);
	dev = &adapt_data->adap->dev;

	if (!data)
		return -EINVAL;

	tx_msg.addr = addr;
	tx_msg.flags = flags;
	tx_msg.read_write = read_write;
	tx_msg.command = command;
	tx_msg.protocol = protocol;
	tx_msg.data = *data;
	tx_msg.status = 0;

	xerr = xlink_write_data(adapt_data->xhandle, adapt_data->channel,
				(u8 *)&tx_msg,
				sizeof(struct xlink_msg));

	if (xerr != X_LINK_SUCCESS) {
		dev_err_ratelimited(dev,
				    "xlink_write_data failed (%d) dropping packet.\n",
				    xerr);
		return -EIO;
	}

	/*
	 * wait for getting the response from the peer host device
	 * message is received by xlinki2c_receive_thread
	 * and notified here through completion trigger
	 */
	if (wait_for_completion_interruptible_timeout(&adapt_data->work,
						      4 * HZ) > 0) {
		rx_msg = list_first_entry(&adapt_data->head,
					  struct xlink_msg,
					  node);
		list_del(&rx_msg->node);

		/* Update the data and status from the xlink message received */
		*data = rx_msg->data;
		rc = rx_msg->status;

		/* free the response received from Proxy */
		kfree(rx_msg);
	} else {
		WARN_ONCE(1, "VPU not responding");
		rc = -ETIMEDOUT;
	}

	return rc;
}

static int xlinki2c_receive_thread(void *param)
{
	struct xlink_adapter_data *adapt_data = param;
	struct device *dev = &adapt_data->adap->dev;
	struct i2c_adapter *adap;
	enum xlink_error xerr;
	struct xlink_msg *msg;
	u32 size;

	while (!kthread_should_stop()) {
		/* msg will be freed in this context or other */
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		if (!msg)
			return -ENOMEM;

		/* Wait to receive xlink message from the peer device */
		xerr = xlink_read_data_to_buffer(adapt_data->xhandle,
						 adapt_data->channel,
						 (uint8_t *)msg, &size);
		if (xerr != X_LINK_SUCCESS) {
			if (xerr != X_LINK_TIMEOUT) {
				dev_warn_ratelimited(dev,
						     "[%d] Error (%d) dropping packet.\n",
						     adapt_data->adap->nr, xerr);
			}
			kfree(msg);
			continue;
		}
		xlink_release_data(adapt_data->xhandle, adapt_data->channel,
				   NULL);
		adap = proxy_i2c_adapter_info();

		if (adap) {
#if IS_ENABLED(CONFIG_I2C_SLAVE)
			if (adapt_data->slave) {
				msg->status = handle_slave_mode
					(adapt_data->slave, msg);
				goto send_resp;
			}
#endif
			/*
			 * This is a proxy for an existing adapter.
			 * call the local adapter to receive the data
			 * from the hardware.
			 */
			msg->status = i2c_smbus_xfer(adap,
						     msg->addr,
						     msg->flags,
						     msg->read_write,
						     msg->command,
						     msg->protocol,
						     &msg->data);

			/*
			 * Send back the complete message that
			 * carries status, back to sender which is
			 * waiting on xlinki2c_receive_thread
			 */
#if IS_ENABLED(CONFIG_I2C_SLAVE)
send_resp:
#endif
			xlink_write_data(adapt_data->xhandle,
					 adapt_data->channel, (u8 *)msg,
					 sizeof(struct xlink_msg));
			kfree(msg);
		} else {
			/*
			 * This is an adapter on its own.
			 * Receives the status and data over xlink (msg).
			 * Indicate the data received to the component
			 * which is waiting in xlink_smbus_xfer
			 */
			list_add_tail(&msg->node, &adapt_data->head);
			complete(&adapt_data->work);
		}
	} /* thread loop */
	dev_dbg(dev, "[%d] %s stopped\n", adapt_data->adap->nr, __func__);

	return 0;
}

static inline u32 xlink_smbus_func(struct i2c_adapter *adapter)
{
	u32 func = I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_DATA;

	return func;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)

/*
 * This will be called when slave client driver
 * register itself to an adapter
 */
static int xlink_smbus_reg_slave(struct i2c_client *slave)
{
	struct xlink_adapter_data *adapt_data =
				i2c_get_adapdata(slave->adapter);

	adapt_data->slave = slave;

	return 0;
}

static int xlink_smbus_unreg_slave(struct i2c_client *slave)
{
	struct xlink_adapter_data *adapt_data =
				i2c_get_adapdata(slave->adapter);

	adapt_data->slave = NULL;

	return 0;
}
#endif

static struct i2c_algorithm xlink_algorithm = {
	.smbus_xfer     = xlink_smbus_xfer,
	.functionality  = xlink_smbus_func,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave      = xlink_smbus_reg_slave,
	.unreg_slave    = xlink_smbus_unreg_slave,
#endif
};

static int xlink_i2c_probe(struct platform_device *pdev)
{
	struct intel_hddl_clients *c = pdev->dev.platform_data;
	struct xlink_handle *devhandle = &c->xlink_dev;
	struct xlink_adapter_data *adapt_data;
	struct device *dev = &pdev->dev;
	struct i2c_adapter *adap;
	u32 rc;

	dev_dbg(dev, "Registering xlink SMBus adapter...\n");

	adap = kzalloc(sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;

	c->adap[pdev->id & 0x3] = adap;
	memset(adap, 0, sizeof(struct i2c_adapter));
	adap->owner  = THIS_MODULE;
	adap->algo   = &xlink_algorithm;
	strcpy(adap->name, "xlink adapter");
	platform_set_drvdata(pdev, adap);

	adapt_data = kzalloc(sizeof(*adapt_data), GFP_KERNEL);
	if (!adapt_data) {
		kfree(adap);
		return -ENOMEM;
	}

	init_completion(&adapt_data->work);

	INIT_LIST_HEAD(&adapt_data->head);
	adapt_data->channel = c->xlink_i2c_ch[pdev->id & 0x3];
	adapt_data->xhandle = devhandle;
	adapt_data->adap = adap;

	rc = xlink_open_channel(devhandle,
				adapt_data->channel,
				RXB_TXB,  /* mode */
				64 * 1024,
				100);  /* timeout */
	if (rc != X_LINK_SUCCESS) {
		dev_err(dev, "xlink_open_channel failed[%d][%d][%p]\n", rc,
			adapt_data->channel,
			adapt_data->xhandle);
		goto err_kfree;
	}

	i2c_set_adapdata(adap, adapt_data);

	rc = i2c_add_adapter(adap);
	if (rc)
		goto err_exit;

	/* Create receiver thread */
	adapt_data->task_recv = kthread_run(xlinki2c_receive_thread,
					    adapt_data,
					    "xlinki2c_receive_thread");
	if (!adapt_data->task_recv) {
		dev_err(dev, "%s Thread creation failed", __func__);
		i2c_del_adapter(adapt_data->adap);
		goto err_exit;
	}
	return 0;

err_exit:
	xlink_close_channel(adapt_data->xhandle, adapt_data->channel);
err_kfree:
	kfree(adap);
	kfree(adapt_data);
	return rc;
}

static int xlink_i2c_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adap = platform_get_drvdata(pdev);
	struct xlink_adapter_data *adapt_data = i2c_get_adapdata(adap);

	kthread_stop(adapt_data->task_recv);

	dev_info(&adap->dev, "Delete the adapter[%d]\n", adap->nr);
	/* Close the channel and disconnect */
	xlink_close_channel(adapt_data->xhandle, adapt_data->channel);
	/* This will block the dynamic registration */
	i2c_del_adapter(adapt_data->adap);
	kfree(adapt_data);

	return 0;
}

static struct platform_driver xlink_i2c_driver = {
	.probe = xlink_i2c_probe,
	.remove = xlink_i2c_remove,
	.driver = {
		.name   = "i2c_xlink"
	}
};

/* Define the xlink debug device structures to be used with dev_dbg() et al */

static struct device_driver dbg_name = {
		.name = "xlink_i2c_dbg"
};

static struct device dbg_subname = {
		.init_name = "xlink_i2c_dbg",
		.driver = &dbg_name
};

static struct device *dbgxi2c = &dbg_subname;

static void __exit xlink_adapter_exit(void)
{
	dev_dbg(dbgxi2c, "Unloading XLink I2C module...\n");
	platform_driver_unregister(&xlink_i2c_driver);
}

static int __init xlink_adapter_init(void)
{
	dev_dbg(dbgxi2c, "Loading XLink I2C module...\n");
	platform_driver_register(&xlink_i2c_driver);
	return 0;
}

module_init(xlink_adapter_init);
module_exit(xlink_adapter_exit);

MODULE_AUTHOR("Raja Subramanian, Lakshmi Bai <lakshmi.bai.raja.subramanian@intel.com>");
MODULE_AUTHOR("Thalaiappan, Rathina <rathina.thalaiappan@intel.com>");
MODULE_AUTHOR("Karanth, Ramya P <ramya.p.karanth@intel.com>");
MODULE_DESCRIPTION("xlink i2c adapter");
MODULE_LICENSE("GPL");
