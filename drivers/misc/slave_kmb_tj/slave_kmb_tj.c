// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C slave mode Keembay Junction Tj temperature.
 *
 * Copyright (C) 2019-2020 Intel Corporation
 */


#include <asm/page.h>
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/sched/mm.h>
#include <linux/of_address.h>
#include "../drivers/misc/thermal/keembay_tsens.h"

#define I2C_SLAVE_BYTELEN GENMASK(15, 0)
#define I2C_SLAVE_FLAG_ADDR16 BIT(16)
#define I2C_SLAVE_FLAG_RO BIT(17)
#define I2C_SLAVE_DEVICE_MAGIC(_len, _flags) ((_flags) | (_len))

#define DEBUG_SLAVE_KMB_TJ 0

#if IS_ENABLED(CONFIG_I2C_SLAVE)

struct kmb_tj_data {
	struct bin_attribute bin;
	spinlock_t buffer_lock;
	u16 buffer_idx;
	u16 address_mask;
	u8 num_address_bytes;
	u8 idx_write_cnt;
	bool read_only;
	u8 buffer[];
};

static int i2c_slave_kmb_tj_slave_cb(struct i2c_client *client,
				     enum i2c_slave_event event, u8 *val)
{
	struct kmb_tj_data *kmb_tj = i2c_get_clientdata(client);
	unsigned char *temp;

	switch (event) {
	case I2C_SLAVE_WRITE_RECEIVED:
	{
		if (kmb_tj->idx_write_cnt < kmb_tj->num_address_bytes) {
			if (kmb_tj->idx_write_cnt == 0)
				kmb_tj->buffer_idx = 0;

			kmb_tj->buffer_idx = *val | (kmb_tj->buffer_idx << 8);
			kmb_tj->idx_write_cnt++;
			spin_lock(&kmb_tj->buffer_lock);
			temp = (unsigned char *)kmb_tj_get_temp_base();
			kmb_tj->buffer[kmb_tj->buffer_idx]
				= temp[kmb_tj->buffer_idx];
			spin_unlock(&kmb_tj->buffer_lock);
		} else {
			if (!kmb_tj->read_only) {
				dev_info(&client->dev,
					"I2C_SLAVE_WRITE_RECEIVED KMB TJ reg are readonly\n");
				return -1; /* write not allowed */
			}
		}
		break;
	}

	case I2C_SLAVE_READ_PROCESSED:
	{
		/* The previous byte made it to the bus, get next one */
		kmb_tj->buffer_idx++;
	}
		/* fallthrough */
	case I2C_SLAVE_READ_REQUESTED:
	{
		if (kmb_tj->buffer_idx >= (6 * sizeof(int))) {
			dev_info(&client->dev, "Error Wrong Offset\n");
			/* only first four bytes correspond to Tj Temperature */
			return -1;
		}

		spin_lock(&kmb_tj->buffer_lock);
		*val = kmb_tj->buffer[kmb_tj->buffer_idx];
		spin_unlock(&kmb_tj->buffer_lock);
		/*
		 * Do not increment buffer_idx here, because we don't know if
		 * this byte will be actually used. Read Linux I2C slave docs
		 * for details.
		 */
		break;
	}

	case I2C_SLAVE_STOP:
	{
		kmb_tj->idx_write_cnt = 0;
		break;
	}
	case I2C_SLAVE_WRITE_REQUESTED:
	{

		kmb_tj->idx_write_cnt = 0;
		break;
	}

	default:
		break;
	}

	return 0;
}

static int hddl_id_read(uint32_t *bid, uint32_t *kmbid, struct device *dev)
{
	char *gpio_base_address;

	gpio_base_address = ioremap(0x20320000, 2048);


	/* Configure the GPIOs */

	gpio_base_address = ioremap(0x20320000, 2048);

	/* Configure the GPIOs */

	writel(0x1C0F, gpio_base_address + 0x2CC);
	writel(0x1C0F, gpio_base_address + 0x2D0);
	writel(0x1C0F, gpio_base_address + 0x2D4);

#if DEBUG_SLAVE_KMB_TJ
	dev_info(dev, "0x2CC = %x\n", readl(gpio_base_address + 0x2CC));
	dev_info(dev, "0x2D0 = %x\n", readl(gpio_base_address + 0x2D0));
	dev_info(dev, "0x2D4 = %x\n", readl(gpio_base_address + 0x2D4));
#endif

	writel(0x1C0F, gpio_base_address + 0x328);
	writel(0x1C0F, gpio_base_address + 0x32C);
	writel(0x1C0F, gpio_base_address + 0x330);

#if DEBUG_SLAVE_KMB_TJ
	dev_info(dev, "0x328 = %x\n", readl(gpio_base_address + 0x328));
	dev_info(dev, "0x32C = %x\n", readl(gpio_base_address + 0x32C));
	dev_info(dev, "0x330 = %x\n", readl(gpio_base_address + 0x330));
#endif /*DEBUG_SLAVE_KMB_TJ*/

	*bid = readl(gpio_base_address + 0x24);
	*bid = (*bid >> 19) & 0x7;
	*kmbid = readl(gpio_base_address + 0x28);
	*kmbid = (*kmbid >> 10) & 0x7;
#if DEBUG_SLAVE_KMB_TJ
	dev_info(dev, "HDDL: GPIO BOARD ID = %u\n", *bid);
	dev_info(dev, "HDDL: GPIO KEEMBAY ID = %u\n", *kmbid);
#endif
	if (*kmbid > 2) {
		*kmbid = 0;
		dev_info(dev, "HDDL: GPIO KEEMBAY ID > 2, ");
		dev_info(dev, "Hence setting KEEMBAY ID = 0\n");
	}
	return 0;
}

static int i2c_slave_kmb_tj_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kmb_tj_data *kmb_tj;
	int ret;
	unsigned int size =
		FIELD_GET(I2C_SLAVE_BYTELEN, id->driver_data);
	unsigned int flag_addr16 =
		FIELD_GET(I2C_SLAVE_FLAG_ADDR16, id->driver_data);
	unsigned int slave_addr;
	uint32_t board_id, kmb_id;

	hddl_id_read(&board_id, &kmb_id, &client->dev);
	if (board_id <= 4) {
		/* Slave address range 0x10 -- 0x1F */
		slave_addr = kmb_id + 0x10 + (board_id * 3);
	} else {
		/* slave address range 0x60 -- 0x6F */
		slave_addr = kmb_id + 0x60 + ((board_id - 5) * 3);
	}

	kmb_tj = devm_kzalloc(&client->dev,
			sizeof(struct kmb_tj_data) + size, GFP_KERNEL);
	if (!kmb_tj)
		return -ENOMEM;

	client->addr = slave_addr;
	kmb_tj->idx_write_cnt = 0;
	kmb_tj->num_address_bytes = flag_addr16 ? 2 : 1;
	kmb_tj->address_mask = size - 1;
	kmb_tj->read_only = FIELD_GET(I2C_SLAVE_FLAG_RO, id->driver_data);
	spin_lock_init(&kmb_tj->buffer_lock);
	i2c_set_clientdata(client, kmb_tj);

	ret = i2c_slave_register(client, i2c_slave_kmb_tj_slave_cb);
#if DEBUG_SLAVE_KMB_TJ
	dev_info(&client->dev, "Slave Kmb Probe Success[%x]\n", slave_addr);
#endif

	return ret;
};

static int i2c_slave_kmb_tj_remove(struct i2c_client *client)
{
	struct kmb_tj_data *kmb_tj = i2c_get_clientdata(client);

	/* kfree(kmb_tj);  not required as for devm_kzalloc */
	i2c_slave_unregister(client);
	sysfs_remove_bin_file(&client->dev.kobj, &kmb_tj->bin);

	return 0;
}

static const struct i2c_device_id i2c_slave_kmb_tj_id[] = {
	{ "slave-kmb-tj", 16 },
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_slave_kmb_tj_id);

static struct i2c_driver i2c_slave_kmb_tj_driver = {
	.driver = {
		.name = "slave-kmb-tj",
	},
	.probe = i2c_slave_kmb_tj_probe,
	.remove = i2c_slave_kmb_tj_remove,
	.id_table = i2c_slave_kmb_tj_id,
};
module_i2c_driver(i2c_slave_kmb_tj_driver);

#endif /* CONFIG_I2C_SLAVE */

MODULE_AUTHOR("Vaidya, Mahesh R <mahesh.r.vaidya@intel.com>");
MODULE_DESCRIPTION("I2C slave mode Keembay Junction temperature Driver");
MODULE_LICENSE("GPL v2");
