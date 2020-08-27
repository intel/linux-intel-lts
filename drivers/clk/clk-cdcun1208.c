// SPDX-License-Identifier: GPL-2.0-only
/*
 * clk-cdcun1208.c- external clock buffer driver.
 *
 * Copyright (C) 2019 Intel Corporation
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#include <dt-bindings/clock/ti,cdcun1208.h>

#define CDCUN1208_MODULE_REG_ID	0x0B
#define CDCUN1208_MAX_CHANELS	0x08
#define CDCUN1208_ENABLE_VAL	0x0212
#define CDCUN1208_DISABLE_VAL	0x0000

struct cdcun1208_driver_data;

struct cdcun1208_clk_out {
	struct clk_hw hw;
	struct clk_init_data init;
	u16 out_address;
	struct cdcun1208_driver_data *driver_data;
};

struct cdcun1208_driver_data {
	struct clk *soc_clk;
	struct i2c_client *client;
	struct mutex mutex;
	u32 ref;
	struct regulator *supply;
	struct cdcun1208_clk_out out[CDCUN1208_MAX_CHANELS];
};

struct cdcun1208_clock_reg {
	u16 address;
	u16 val;
};

struct cdcun1208_clk {
	const char *name;
	u16 out_address;
	const struct clk_ops ops;
};

static struct cdcun1208_clock_reg cdcun1208_init_data[] = {
	{0x0000, 0x0000},
	{0x0001, 0x0000},
	{0x0002, 0x0000},
	{0x0003, 0x0000},
	{0x0004, 0x0000},
	{0x0005, 0x0000},
	{0x0006, 0x0000},
	{0x0007, 0x0000},
	{0x000b, 0x0004},
	{0x000f, 0x0000},
};

static struct cdcun1208_clock_reg cdcun1208_deinit_data[] = {
	{0x000f, 0x0001},
};

static int
cdcun1208_clock_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buff[1] = { reg & 0xf };
	u8 data_buff[2] = { 0 };
	int ret = 0;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buff);
	msgs[0].buf = addr_buff;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = ARRAY_SIZE(data_buff);
	msgs[1].buf = data_buff;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be16(data_buff);

	return ret;
}

static int
cdcun1208_clock_write_reg(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msgs[1];
	u8 buff[3] = { 0 };
	int ret = 0;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 3;
	msgs[0].buf = buff;

	/* Write register address */
	buff[0] = reg & 0x0f;

	/* Write register value */
	put_unaligned_be16(val, &buff[1]);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	return ret;
}

static int
cdcun1208_clock_write_regs(struct i2c_client *client,
			   const struct cdcun1208_clock_reg *regs,
			   u32 len)
{
	int ret;
	u32 i;

	for (i = 0; i < len; i++) {
		ret = cdcun1208_clock_write_reg(client, regs[i].address,
						regs[i].val);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int
cdcun1208_init_clock_module(struct cdcun1208_driver_data *data)
{
	return cdcun1208_clock_write_regs(data->client,
		cdcun1208_init_data,
		ARRAY_SIZE(cdcun1208_init_data));
}

static void
cdcun1208_deinit_clock_module(struct cdcun1208_driver_data *data)
{
	int ret;

	ret = cdcun1208_clock_write_regs(data->client, cdcun1208_deinit_data,
					 ARRAY_SIZE(cdcun1208_deinit_data));
	if (ret < 0)
		pr_err("%s:%d error deinit cdcun1208",
		       __func__, __LINE__);
}

static int
cdcun1208_clock_prepare(struct clk_hw *clk)
{
	int ret = 0;
	struct cdcun1208_clk_out *clk_out;
	struct cdcun1208_driver_data *data;
	struct cdcun1208_clock_reg prepare_data[1];

	clk_out = container_of(clk, struct cdcun1208_clk_out, hw);
	data = clk_out->driver_data;

	prepare_data[0].address = clk_out->out_address;
	prepare_data[0].val = CDCUN1208_ENABLE_VAL;

	mutex_lock(&data->mutex);
	if (!data->ref++) {
		ret = cdcun1208_init_clock_module(data);
		if (ret < 0) {
			pr_err("%s:%d error init cdcun1208",
			       __func__, __LINE__);
			goto mutex_unlock;
		}
	}
	mutex_unlock(&data->mutex);

	ret = cdcun1208_clock_write_regs(data->client, prepare_data,
					 ARRAY_SIZE(prepare_data));
	if (ret < 0) {
		pr_err("%s:%d error get cdcun1208 ch: %d",
		       __func__, __LINE__, clk_out->out_address);
		goto out;
	}

mutex_unlock:
	mutex_unlock(&data->mutex);
out:
	return (ret < 0) ? ret : 0;
}

static void
cdcun1208_clock_unprepare(struct clk_hw *clk)
{
	int ret;
	struct cdcun1208_clk_out *clk_out;
	struct cdcun1208_driver_data *data;
	struct cdcun1208_clock_reg unprepare_data[1];

	clk_out = container_of(clk, struct cdcun1208_clk_out, hw);
	data = clk_out->driver_data;

	unprepare_data[0].address = clk_out->out_address;
	unprepare_data[0].val = CDCUN1208_DISABLE_VAL;

	ret = cdcun1208_clock_write_regs(data->client, unprepare_data,
					 ARRAY_SIZE(unprepare_data));
	if (ret < 0)
		pr_err("%s:%d error put cdcun1208 ch: %d",
		       __func__, __LINE__, clk_out->out_address);

	mutex_lock(&data->mutex);
	if (!--data->ref)
		cdcun1208_deinit_clock_module(data);

	mutex_unlock(&data->mutex);
}

static const struct cdcun1208_clk cdcun1208_clks[CDCUN1208_MAX_CHANELS] = {
	[CDCUN_1208_OUT1] = {
		.name = "clkout1",
		.out_address = 0x00,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT2] = {
		.name = "clkout2",
		.out_address = 0x01,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT3] = {
		.name = "clkout3",
		.out_address = 0x02,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT4] = {
		.name = "clkout4",
		.out_address = 0x03,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT5] = {
		.name = "clkout5",
		.out_address = 0x04,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT6] = {
		.name = "clkout6",
		.out_address = 0x05,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT7] = {
		.name = "clkout7",
		.out_address = 0x06,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
	[CDCUN_1208_OUT8] = {
		.name = "clkout8",
		.out_address = 0x07,
		.ops = {
			.prepare	= cdcun1208_clock_prepare,
			.unprepare	= cdcun1208_clock_unprepare,
		}
	},
};

static struct clk_hw *
cdcun1208_clk_buff_of_get(struct of_phandle_args *clkspec,
			  void *data)
{
	struct cdcun1208_driver_data *d = data;
	unsigned int idx = clkspec->args[0];

	return &d->out[idx].hw;
}

static int
cdcun1208_clk_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i, ret;
	struct device *dev = &client->dev;
	const char *gpio_out_clk_name;
	struct cdcun1208_driver_data *data;
	u16 val;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->mutex);

	data->supply = devm_regulator_get(dev, "supply-out-clk");
	if (IS_ERR(data->supply))
		return PTR_ERR(data->supply);

	ret = regulator_enable(data->supply);
	if (ret < 0)
		return ret;

	data->soc_clk = devm_clk_get(dev, "gpio-out-clk");
	if (IS_ERR(data->soc_clk))
		return PTR_ERR(data->soc_clk);

	gpio_out_clk_name = __clk_get_name(data->soc_clk);

	data->client = client;
	i2c_set_clientdata(client, data);

	ret = cdcun1208_clock_read_reg(data->client, CDCUN1208_MODULE_REG_ID,
				       &val);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to find clock buffrer module: %d", ret);
		return ret;
	}

	for (i = 0; i < CDCUN1208_MAX_CHANELS; i++) {
		data->out[i].init.name = cdcun1208_clks[i].name;
		data->out[i].init.ops = &cdcun1208_clks[i].ops;
		data->out[i].init.num_parents = 1;
		data->out[i].init.flags = CLK_SET_RATE_PARENT;
		data->out[i].init.parent_names = &gpio_out_clk_name;

		data->out[i].hw.init = &data->out[i].init;

		data->out[i].out_address = cdcun1208_clks[i].out_address;
		data->out[i].driver_data = data;

		ret = devm_clk_hw_register(dev, &data->out[i].hw);
		if (ret < 0)
			return ret;
	}

	return devm_of_clk_add_hw_provider(dev,
			cdcun1208_clk_buff_of_get, data);
}

static const struct of_device_id cdcun_1208_dt_ids[] = {
	{ .compatible = "ti,cdcun1208", },
	{}
};
MODULE_DEVICE_TABLE(of, cdcun_1208_dt_ids);

static const struct i2c_device_id cdcun_1208_i2c_id[] = {
	{.name = "cdcun1208", },
	{}
};
MODULE_DEVICE_TABLE(i2c, cdcun_1208_i2c_id);

static struct i2c_driver cdcun1208_clock_i2c_driver = {
	.driver = {
		.name = "cdcun1208",
		.of_match_table = cdcun_1208_dt_ids,
	},
	.probe = cdcun1208_clk_probe,
	.id_table = cdcun_1208_i2c_id,
};

module_i2c_driver(cdcun1208_clock_i2c_driver);
MODULE_DESCRIPTION("cdcun1208 external clock buffer driver");
MODULE_LICENSE("GPL v2");
