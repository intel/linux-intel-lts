// SPDX-License-Identifier: GPL-2.0-only
/*
 * kmb-gt24c64a.c - EEPROM Driver for Keem Bay Camera.
 *
 * Copyright (C) 2021 Intel Corporation
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>

#define KMB_GT24C64A_DRV_NAME "kmb-gt24c64a"
#define KMB_GT24C64A_SIZE 8192
#define KMB_GT24C64A_MAX_WRITE_SIZE 32

/**
 * struct kmb_gt24c64a - KMB Camera GT24C64A EEPROM device structure
 * @client: pointer to i2c client
 * @regmap: pointer to regmap
 * @nvmem: pointer to nvmem device
 * @dev: pointer to basic device structure
 * @lock: mutex
 * @subdev: V4L2 sub-device
 */
struct kmb_gt24c64a {
	struct i2c_client *client;
	struct regmap *regmap;
	struct nvmem_device *nvmem;
	struct device *dev;
};

/**
 * kmb_gt24c64a_read - NVMEM reg_read cb
 * @priv: nvmem_config private data - struct kmb_gt24c64a
 * @off: offset in nvmem device.
 * @val: buffer to read into.
 * @count: number of bytes to read.
 *
 * Return: 0 if successful
 */
static int kmb_gt24c64a_read(void *priv, unsigned int off,
			     void *val, size_t count)
{
	struct kmb_gt24c64a *eeprom = priv;
	struct regmap *regmap = eeprom->regmap;
	char *buf = val;

	if (off + count > KMB_GT24C64A_SIZE)
		return -EINVAL;

	return regmap_bulk_read(regmap, off, buf, count);
}

/**
 * kmb_gt24c64a_write - NVMEM reg_write cb
 * @priv: nvmem_config private data - struct kmb_gt24c64a
 * @off: offset in nvmem device.
 * @val: buffer to write from.
 * @count: number of bytes to write.
 *
 * Return: 0 if successful
 */
static int kmb_gt24c64a_write(void *priv, unsigned int off,
			      void *val, size_t count)
{
	struct kmb_gt24c64a *eeprom = priv;
	struct regmap *regmap =
		eeprom->regmap;
	size_t write_count;
	char *buf = val;
	int ret;

	if (count > KMB_GT24C64A_MAX_WRITE_SIZE)
		write_count = KMB_GT24C64A_MAX_WRITE_SIZE;
	else
		write_count = count;

	while (count) {
		ret = regmap_bulk_write(regmap, off, buf, write_count);
		if (ret < 0)
			return ret;

		buf += write_count;
		off += write_count;
		count -= write_count;
		if (count < write_count)
			write_count = count;
		/* Need some time between writes */
		usleep_range(1000, 1500);
	}
	return 0;
}

/**
 * kmb_gt24c64a_probe - I2C client device binding
 * @client: pointer to i2c client device
 * @id: pointer to i2c device id
 *
 * Return: 0 if successful
 */
static int kmb_gt24c64a_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct regmap_config regmap_config;
	struct nvmem_config nvmem_config;
	struct kmb_gt24c64a *eeprom;
	struct regmap *regmap;

	eeprom = devm_kzalloc(&client->dev, sizeof(*eeprom), GFP_KERNEL);
	if (!eeprom)
		return -ENOMEM;

	memset(&regmap_config, 0, sizeof(regmap_config));
	memset(&nvmem_config, 0, sizeof(nvmem_config));

	regmap_config.val_bits = 8;
	regmap_config.reg_bits = 16;
	regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	eeprom->regmap = regmap;

	nvmem_config.name = dev_name(&client->dev);
	nvmem_config.dev = &client->dev;
	nvmem_config.owner = THIS_MODULE;
	nvmem_config.compat = true;
	nvmem_config.base_dev = &client->dev;
	nvmem_config.reg_read = kmb_gt24c64a_read;
	nvmem_config.reg_write = kmb_gt24c64a_write;
	nvmem_config.type = NVMEM_TYPE_EEPROM;
	nvmem_config.priv = eeprom;
	nvmem_config.stride = 1;
	nvmem_config.word_size = 1;
	nvmem_config.size = KMB_GT24C64A_SIZE;
	eeprom->nvmem = devm_nvmem_register(&client->dev,
					    &nvmem_config);
	if (IS_ERR(eeprom->nvmem)) {
		dev_err(&client->dev, "Can not create nvmem!");
		return PTR_ERR(eeprom->nvmem);
	}
	dev_info(&client->dev, "Probe success!");
	return 0;
}

/**
 * kmb_gt24c64a_remove - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful
 */
static int kmb_gt24c64a_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id kmb_gt24c64a_id_table[] = {
	{"kmb-gt24c64a", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, kmb_gt24c64a_id_table);

static struct i2c_driver kmb_gt24c64a_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "kmb-gt24c64a",
	},
	.probe = kmb_gt24c64a_probe,
	.remove = kmb_gt24c64a_remove,
	.id_table = kmb_gt24c64a_id_table,
};

module_i2c_driver(kmb_gt24c64a_i2c_driver);

MODULE_DESCRIPTION("Keem Bay gt24c64 I2C EEPROM driver");
MODULE_LICENSE("GPL v2");
