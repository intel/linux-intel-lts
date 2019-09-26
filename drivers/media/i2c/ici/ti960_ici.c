// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <media/ti960.h>
#include <media/crlmodule-lite.h>
#include <media/ici.h>

#include "ti960-reg-ici.h"

struct ti960_subdev {
	struct ici_ext_subdev *sd;
	unsigned short rx_port;
	unsigned short fsin_gpio;
	unsigned short phy_i2c_addr;
	unsigned short alias_i2c_addr;
	unsigned short ser_i2c_addr;
	char sd_name[ICI_MAX_NODE_NAME];
	struct i2c_client *sensor_client;
};

struct ti960 {
	struct ici_ext_subdev sd;
	struct ici_ext_subdev_register reg;
	struct ti960_pdata *pdata;
	struct ti960_subdev sub_devs[NR_OF_TI960_SINK_PADS];
	struct crlmodule_lite_platform_data subdev_pdata[NR_OF_TI960_SINK_PADS];
	const char *name;

	struct mutex mutex;

	struct regmap *regmap8;
	struct regmap *regmap16;

	struct ici_pad_framefmt *ffmts[NR_OF_TI960_SINK_PADS];
	struct ici_rect *crop;
	struct ici_rect *compose;

	unsigned int nsinks;
	unsigned int nsources;
	unsigned int nstreams;
	unsigned int npads;

	struct gpio_chip gc;

	int (*create_link)(
		struct ici_isys_node *src,
		u16 src_pad,
		struct ici_isys_node *sink,
		u16 sink_pad,
		u32 flags);
	//struct v4l2_ctrl *link_freq;
	//struct v4l2_ctrl *test_pattern;
};

/*
 * V4L2 uses v4l2_subdev_route struct to calculate
 * vc value to program CSI_VC_MAP register.
 * Implement something similar to v4l2_subdev_route
 * and create a vc_map table.
 */
struct ti960_vc_map {
	u32 rx_port;
	u32 sink_stream;
	u32 source_stream;
};

static struct ti960_vc_map vc_map[] = {
	{0, 0, 0},
	{1, 0, 1},
	{2, 0, 2},
	{3, 0, 3},
};

static int init_ext_sd(struct i2c_client *client,
	struct ti960_subdev *sd,
	int idx);
static int ti960_find_subdev_index(struct ti960 *va, struct ici_ext_subdev *sd);
static int create_link(struct ici_isys_node *src_node, u16 srcpad,
	struct ici_isys_node *sink_node, u16 sinkpad, u32 flag);
static int ti960_get_param(struct ici_ext_sd_param *param);
static int ti960_get_menu_item(struct ici_ext_sd_param *param, u32 idx);
static int ti960_set_param(struct ici_ext_sd_param *param);
static int ti960_set_power(struct ici_isys_node *node, int on);
static int ti960_set_stream(struct ici_isys_node *node, void *ip, int enable);
static int ti960_enum_mbus_code(struct ici_isys_node *node,
	struct ici_pad_supported_format_desc *psfd);

#define to_ti960(_sd) container_of(_sd, struct ti960, sd)
#define to_ici_ext_subdev(_node) \
	container_of(_node, struct ici_ext_subdev, node)
#define TI960_SRC_PAD 1

static const s64 ti960_op_sys_clock[] =  {400000000, 800000000};
static const u8 ti960_op_sys_clock_reg_val[] = {
	TI960_MIPI_800MBPS,
	TI960_MIPI_1600MBPS
};

/*
 * Order matters.
 *
 * 1. Bits-per-pixel, descending.
 * 2. Bits-per-pixel compressed, descending.
 * 3. Pixel order, same as in pixel_order_str. Formats for all four pixel
 *    orders must be defined.
 */
static const struct ti960_csi_data_format va_csi_data_formats[] = {
	{ ICI_FORMAT_YUYV, 16, 16, PIXEL_ORDER_GBRG, 0x1e },
	{ ICI_FORMAT_UYVY, 16, 16, PIXEL_ORDER_GBRG, 0x1e },
};

static struct regmap_config ti960_reg_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct regmap_config ti960_reg_config16 = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
};

static int ti953_reg_write(struct ti960 *va, unsigned short rx_port,
	unsigned short ser_alias, unsigned char reg, unsigned char val)
{
	int ret;
	int retry, timeout = 10;
	struct i2c_client *client = va->sd.client;

	pr_debug("%s port %d, ser_alias %x, reg %x, val %x",
		__func__, rx_port, ser_alias, reg, val);
	client->addr = ser_alias;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			pr_err("ti953 reg write ret=%x", ret);
			usleep_range(5000, 6000);
		} else
			break;
	}

	client->addr = TI960_I2C_ADDRESS;
	if (retry >= timeout) {
		pr_err("%s:write reg failed: port=%2x, addr=%2x, reg=%2x\n",
			__func__, rx_port, ser_alias, reg);
		return -EREMOTEIO;
	}

	return 0;
}

static int ti953_reg_read(struct ti960 *va, unsigned short rx_port,
	unsigned short ser_alias, unsigned char reg, unsigned char *val)
{
	int retry, timeout = 10;
	struct i2c_client *client = va->sd.client;

	client->addr = ser_alias;
	for (retry = 0; retry < timeout; retry++) {
		*val = i2c_smbus_read_byte_data(client, reg);
		if (*val < 0)
			usleep_range(5000, 6000);
		else
			break;
	}

	client->addr = TI960_I2C_ADDRESS;
	if (retry >= timeout) {
		pr_err("%s:read reg failed: port=%2x, addr=%2x, reg=%2x\n",
			__func__, rx_port, ser_alias, reg);
		return -EREMOTEIO;
	}

	return 0;
}

static bool ti953_detect(struct ti960 *va,
	unsigned short rx_port,
	unsigned short ser_alias)
{
	bool ret = false;
	int i;
	int rval;
	unsigned char val;

	for (i = 0; i < ARRAY_SIZE(ti953_FPD3_RX_ID); i++) {
		rval = ti953_reg_read(va, rx_port, ser_alias,
			ti953_FPD3_RX_ID[i].reg, &val);
		if (rval) {
			pr_err("port %d, ti953 write timeout %d\n",
				rx_port, rval);
			break;
		}
		if (val != ti953_FPD3_RX_ID[i].val_expected)
			break;
	}

	if (i == ARRAY_SIZE(ti953_FPD3_RX_ID))
		ret = true;

	return ret;
}

static int ti960_reg_read(struct ti960 *va,
	unsigned char reg,
	unsigned int *val)
{
	int ret, retry, timeout = 10;

	for (retry = 0; retry < timeout; retry++) {
		ret = regmap_read(va->regmap8, reg, val);
		if (ret < 0) {
			pr_err("960 reg read ret=%x", ret);
			usleep_range(5000, 6000);
		} else {
			break;
		}
	}

	if (retry >= timeout) {
		pr_err("%s:devid read failed: reg=%2x, ret=%d\n",
			__func__, reg, ret);
		return -EREMOTEIO;
	}

	return 0;
}

static int ti960_reg_set_bit(struct ti960 *va, unsigned char reg,
	unsigned char bit, unsigned char val)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(va->regmap8, reg, &reg_val);
	if (ret)
		return ret;
	if (val)
		reg_val |= 1 << bit;
	else
		reg_val &= ~(1 << bit);

	return regmap_write(va->regmap8, reg, reg_val);
}

static int ti960_map_phy_i2c_addr(struct ti960 *va, unsigned short rx_port,
			      unsigned short addr)
{
	int rval;

	rval = regmap_write(va->regmap8, TI960_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI960_SLAVE_ID0, addr);
}

static int ti960_map_alias_i2c_addr(struct ti960 *va, unsigned short rx_port,
			      unsigned short addr)
{
	int rval;

	rval = regmap_write(va->regmap8, TI960_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI960_SLAVE_ALIAS_ID0, addr);
}

static int ti960_map_ser_alias_addr(struct ti960 *va, unsigned short rx_port,
			      unsigned short ser_alias)
{
	int rval;

	pr_debug("%s port %d, ser_alias %x\n", __func__, rx_port, ser_alias);
	rval = regmap_write(va->regmap8, TI960_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI960_SER_ALIAS_ID, ser_alias);
}

static int ti960_enum_mbus_code(struct ici_isys_node *node,
	struct ici_pad_supported_format_desc *psfd)
{
	psfd->color_format = 0x1e; // for ICI_FORMAT_UYVY
	// sensor->sensor_ds->csi_fmts[psfd->idx].code;
	psfd->min_width = TI960_MIN_WIDTH;
	//sensor->sensor_ds->sensor_limits->x_addr_min;
	psfd->max_width = TI960_MAX_WIDTH;
	//sensor->sensor_ds->sensor_limits->x_addr_max;
	psfd->min_height = TI960_MIN_HEIGHT;
	//sensor->sensor_ds->sensor_limits->y_addr_min;
	psfd->max_height = TI960_MAX_HEIGHT;
	//sensor->sensor_ds->sensor_limits->y_addr_max;
	return 0;
}

static void __ti960_get_format(struct ici_ext_subdev *subdev,
	struct ici_pad_framefmt *pff)
{
	struct i2c_client *client = subdev->client;
	struct ici_ext_subdev *sd = i2c_get_clientdata(client);
	struct ti960 *va = to_ti960(sd);
	int index = ti960_find_subdev_index(va, subdev);

	pff->ffmt.width = va->ffmts[index]->ffmt.width;
	pff->ffmt.height = va->ffmts[index]->ffmt.height;
	pff->ffmt.pixelformat =  va->ffmts[index]->ffmt.pixelformat;
	pff->ffmt.field = ((va->ffmts[index]->ffmt.field == ICI_FIELD_ANY) ?
		ICI_FIELD_NONE : va->ffmts[index]->ffmt.field);
}

static void __ti960_set_format(struct ici_ext_subdev *subdev,
	struct ici_pad_framefmt *pff)
{
	struct i2c_client *client = subdev->client;
	struct ici_ext_subdev *sd = i2c_get_clientdata(client);
	struct ti960 *va = to_ti960(sd);
	int index = ti960_find_subdev_index(va, subdev);

	va->ffmts[index]->ffmt.width = pff->ffmt.width;
	va->ffmts[index]->ffmt.height = pff->ffmt.height;
	va->ffmts[index]->ffmt.pixelformat = pff->ffmt.pixelformat;
	va->ffmts[index]->ffmt.field = pff->ffmt.field;
}

static int ti960_set_format(struct ici_isys_node *node,
	struct ici_pad_framefmt *pff)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct ti960 *va = to_ti960(subdev);

	mutex_lock(&va->mutex);
	__ti960_set_format(subsubdev, pff);
	mutex_unlock(&va->mutex);

	pr_debug("framefmt: width: %d, height: %d, code: 0x%x.\n",
		pff->ffmt.width, pff->ffmt.height, pff->ffmt.pixelformat);

	return 0;
}

static int ti960_get_format(struct ici_isys_node *node,
	struct ici_pad_framefmt *pff)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct ti960 *va = to_ti960(subdev);

	mutex_lock(&va->mutex);
	__ti960_get_format(subsubdev, pff);
	mutex_unlock(&va->mutex);

	return 0;
}

static int ti960_set_selection(struct ici_isys_node *node,
	struct ici_pad_selection *ps)
{
	return 0;
}

static int ti960_get_selection(struct ici_isys_node *node,
	struct ici_pad_selection *ps)
{
	return 0;
}

static int ti960_map_subdevs_addr(struct ti960 *va)
{
	unsigned short rx_port, phy_i2c_addr, alias_i2c_addr;
	int i, rval;

	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		rx_port = va->sub_devs[i].rx_port;
		phy_i2c_addr = va->sub_devs[i].phy_i2c_addr;
		alias_i2c_addr = va->sub_devs[i].alias_i2c_addr;

		if (!phy_i2c_addr || !alias_i2c_addr)
			continue;

		rval = ti960_map_phy_i2c_addr(va, rx_port, phy_i2c_addr);
		if (rval)
			return rval;

		/* set 7bit alias i2c addr */
		rval = ti960_map_alias_i2c_addr(va, rx_port,
						alias_i2c_addr << 1);
		if (rval)
			return rval;
	}

	return 0;
}

static int ti960_init_ext_subdev(struct ti960_subdev_info *info,
				struct ti960_subdev *subdev,
				struct ici_ext_subdev_register *reg,
				struct i2c_client *client,
				struct ici_ext_subdev_register *sd_register)
{
	struct i2c_client *client2;
	struct ici_ext_subdev *sensor_sd;
	int rval = 0;

	request_module(I2C_MODULE_PREFIX "%s", info->board_info.type);
	client2 = i2c_new_device(client->adapter, &info->board_info);

	if (client2 == NULL || client2->dev.driver == NULL) {
		pr_err("%s, No new i2c device\n", __func__);
		return -EINVAL;
	}

	/* Get the clientdata set by the sensor driver */
	sensor_sd = i2c_get_clientdata(client2);
	if (!sensor_sd) {
		pr_err("%s, Failed to get client data\n", __func__);
		return -EINVAL;
	}

	subdev->sensor_client = client2;
	sd_register->ipu_data = reg->ipu_data;
	sd_register->sd = sensor_sd;
	sd_register->setup_node = reg->setup_node;
	sd_register->create_link = reg->create_link;

	rval = sensor_sd->do_register(sd_register);
	return rval;
}

static int ti960_registered(struct ici_ext_subdev_register *reg)
{
	struct ici_ext_subdev *subdev = reg->sd;
	struct ti960 *va = to_ti960(subdev);
	struct ti960_subdev *sd;
	struct i2c_client *client = subdev->client;
	struct ici_ext_subdev_register sd_register = {0};
	int i, k, rval;

	if (!reg->sd || !reg->setup_node || !reg->create_link) {
		pr_err("%s, error\n", __func__);
		return -EINVAL;
	}

	va->reg = *reg;
	va->create_link = reg->create_link;

	/*
	 * ti960->sd represents the ti960 and ti960->sub_devs
	 * represents every port/vc/sensor
	 */
	subdev->get_param = ti960_get_param;
	subdev->set_param = ti960_set_param;
	subdev->get_menu_item = ti960_get_menu_item;

	for (i = 0, k = 0; i < va->pdata->subdev_num; i++) {
		struct ti960_subdev_info *info =
			&va->pdata->subdev_info[i];
		struct crlmodule_lite_platform_data *pdata =
			(struct crlmodule_lite_platform_data *)
			info->board_info.platform_data;

		if (k >= va->nsinks)
			break;

		rval = ti960_map_ser_alias_addr(va, info->rx_port,
				info->ser_alias << 1);
		if (rval)
			return rval;


		if (!ti953_detect(va, info->rx_port, info->ser_alias)) {
			k++;
			continue;
		}

		/*
		 * The sensors should not share the same pdata structure.
		 * Clone the pdata for each sensor.
		 */
		memcpy(&va->subdev_pdata[k], pdata, sizeof(*pdata));

		va->sub_devs[k].fsin_gpio = va->subdev_pdata[k].fsin;

		/* Spin sensor subdev suffix name */
		va->subdev_pdata[k].suffix = info->suffix;

		/*
		 * Change the gpio value to have xshutdown
		 * and rx port included, so in gpio_set those
		 * can be caculated from it.
		 */
		va->subdev_pdata[k].xshutdown += va->gc.base +
					info->rx_port * NR_OF_GPIOS_PER_PORT;
		info->board_info.platform_data = &va->subdev_pdata[k];

		if (!info->phy_i2c_addr || !info->board_info.addr) {
			pr_err("can't find the physical and alias addr.\n");
			return -EINVAL;
		}

		/* Map PHY I2C address. */
		rval = ti960_map_phy_i2c_addr(va, info->rx_port,
					info->phy_i2c_addr);
		if (rval)
			return rval;

		/* Map 7bit ALIAS I2C address. */
		rval = ti960_map_alias_i2c_addr(va, info->rx_port,
				info->board_info.addr << 1);
		if (rval)
			return rval;

		/* Initialize sensor connected to TI960 */
		rval = ti960_init_ext_subdev(info, &va->sub_devs[k],
			reg, client, &sd_register);
		if (rval) {
			pr_err("%s, Failed to register external subdev\n",
				__func__);
			continue;
		}

		/* Allocate ici_ext_subdev for each TI960 port */
		va->sub_devs[k].sd = devm_kzalloc(&client->dev,
			sizeof(struct ici_ext_subdev),
			GFP_KERNEL);
		if (!va->sub_devs[k].sd) {
			pr_err("%s, Can't create new i2c subdev %d-%04x\n",
				__func__,
				info->i2c_adapter_id,
				info->board_info.addr);
			continue;
		}

		va->sub_devs[k].rx_port = info->rx_port;
		va->sub_devs[k].phy_i2c_addr = info->phy_i2c_addr;
		va->sub_devs[k].alias_i2c_addr = info->board_info.addr;
		va->sub_devs[k].ser_i2c_addr = info->ser_alias;
		memcpy(va->sub_devs[k].sd_name,
				va->subdev_pdata[k].module_name,
				min(sizeof(va->sub_devs[k].sd_name) - 1,
				sizeof(va->subdev_pdata[k].module_name) - 1));

		sd = &va->sub_devs[k];
		rval = init_ext_sd(va->sd.client, sd, k);

		if (rval)
			return rval;

		rval = sd_register.create_link(&sd_register.sd->node,
						sd_register.sd->src_pad,
						&sd->sd->node, 0, 0);

		if (rval) {
			pr_err("%s, error creating link\n", __func__);
			return rval;
		}
		k++;
	}

	/*
	 * Replace existing create_link address with TI960 create_link
	 * implementation to create link between TI960 node and CSI2 node
	 */
	reg->create_link = create_link;

	rval = ti960_map_subdevs_addr(va);
	if (rval)
		return rval;

	return 0;
}

static int ti960_get_param(struct ici_ext_sd_param *param)
{
	param->val = 400000000;
	return 0;
}

static int ti960_get_menu_item(struct ici_ext_sd_param *param, u32 idx)
{
	return 0;
}

static int ti960_set_param(struct ici_ext_sd_param *param)
{
	return 0;
}

static int init_ext_sd(struct i2c_client *client,
	struct ti960_subdev *ti_sd,
	int idx)
{
	struct ti960 *va;
	char name[ICI_MAX_NODE_NAME];
	int rval;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);

	if (!subdev)
		return -EINVAL;

	va = to_ti960(subdev);

	if (!va)
		return -EINVAL;

	if (va->pdata->suffix) {
		snprintf(name, sizeof(name), "TI960 %c %d",
			va->pdata->suffix,
			idx);
	} else {
		snprintf(name, sizeof(name), "TI960 %4.4x %d",
			i2c_adapter_id(client->adapter),
			idx);
	}

	ti_sd->sd->client = client;
	ti_sd->sd->num_pads = 2;
	ti_sd->sd->set_param = ti960_set_param;
	ti_sd->sd->get_param = ti960_get_param;
	ti_sd->sd->get_menu_item = ti960_get_menu_item;
	if (va->reg.setup_node) {
		rval = va->reg.setup_node(va->reg.ipu_data,
			ti_sd->sd, name);
		if (rval)
			return rval;
	} else {
		pr_err("%s, node is not registered\n", __func__);
	}
	ti_sd->sd->node.node_set_power = ti960_set_power;
	ti_sd->sd->node.node_set_streaming = ti960_set_stream;
	ti_sd->sd->node.node_get_pad_supported_format = ti960_enum_mbus_code;
	ti_sd->sd->node.node_set_pad_ffmt = ti960_set_format;
	ti_sd->sd->node.node_get_pad_ffmt = ti960_get_format;
	ti_sd->sd->node.node_set_pad_sel = ti960_set_selection;
	ti_sd->sd->node.node_get_pad_sel = ti960_get_selection;

	return 0;
}

static int create_link(struct ici_isys_node *src_node,
	u16 srcpad,
	struct ici_isys_node *sink_node,
	u16 sinkpad,
	u32 flag)
{
	struct ici_ext_subdev *sd, *ssd;
	struct ti960 *va;
	struct ti960_subdev *subdev;
	int i, ret;

	if (!src_node || !sink_node)
		return -EINVAL;

	sd = to_ici_ext_subdev(src_node);
	if (!sd)
		return -EINVAL;

	va = to_ti960(sd);
	if (!va)
		return -EINVAL;

	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		subdev = &va->sub_devs[i];
		if (!subdev)
			continue;
		ssd = subdev->sd;
		ret = va->create_link(&ssd->node,
			TI960_SRC_PAD,
			sink_node,
			sinkpad,
			0);
		if (ret)
			return ret;
	}
	return 0;
}
static void ti960_unregistered(struct ici_ext_subdev *subdev)
{
	pr_debug("%s\n", __func__);
}

static int ti960_set_power(struct ici_isys_node *node, int on)
{
	struct ici_ext_subdev *ssd = node->sd;
	struct ti960 *va;
	int ret;
	u8 val;
	struct ici_ext_subdev *sd = i2c_get_clientdata(ssd->client);

	if (!sd)
		return -EINVAL;

	va = to_ti960(sd);

	if (!va)
		return -EINVAL;

	pr_debug("%s, %d\n", __func__, on);
	ret = regmap_write(va->regmap8, TI960_RESET,
			   (on) ? TI960_POWER_ON : TI960_POWER_OFF);
	if (ret || !on)
		return ret;

	ret = regmap_write(va->regmap8, TI960_CSI_PLL_CTL,
				ti960_op_sys_clock_reg_val[0]);

	if (ret)
		return ret;
	val = TI960_CSI_ENABLE;

	//TODO: pegging to 0.8 Gbps for now
	return regmap_write(va->regmap8, TI960_CSI_CTL, val);
}

static int ti960_rx_port_config(struct ti960 *va, int rx_port)
{
	int rval, i;
	unsigned int csi_vc_map;

	/* Select RX port. */
	rval = regmap_write(va->regmap8, TI960_RX_PORT_SEL,
			(rx_port << 4) + (1 << rx_port));
	if (rval) {
		pr_err("Failed to select RX port.\n");
		return rval;
	}

	rval = regmap_write(va->regmap8, TI960_PORT_CONFIG,
		TI960_FPD3_CSI);
	if (rval) {
		pr_err("Failed to set port config.\n");
		return rval;
	}

	/*
	 * CSI VC MAPPING.
	 */
	rval = regmap_read(va->regmap8, TI960_CSI_VC_MAP, &csi_vc_map);
	if (rval < 0) {
		pr_err("960 reg read ret=%x", rval);
		return rval;
	}

	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		if (rx_port != vc_map[i].rx_port)
			continue;
		csi_vc_map &= ~(0x3 << (vc_map[i].sink_stream & 0x3) * 2);
		csi_vc_map |= (vc_map[i].source_stream & 0x3)
			<< (vc_map[i].sink_stream & 0x3) * 2;
	}

	pr_debug("%s, rx_port: %d, csi_vc_map: %x\n",
		__func__, rx_port, csi_vc_map);
	rval = regmap_write(va->regmap8, TI960_CSI_VC_MAP,
		csi_vc_map);

	if (rval) {
		pr_err("Failed to set port config.\n");
		return rval;
	}

	return 0;
}

static int ti960_find_subdev_index(struct ti960 *va, struct ici_ext_subdev *sd)
{
	int i;

	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		if (va->sub_devs[i].sd == sd)
			return i;
	}

	WARN_ON(1);

	return -EINVAL;
}

static int ti960_set_stream(struct ici_isys_node *node, void *ip, int enable)
{
	struct ti960 *va;
	struct ici_ext_subdev *ssd, *sd, *sensor_sd;
	int j, rval;
	unsigned short rx_port;
	unsigned short ser_alias;
	DECLARE_BITMAP(rx_port_enabled, 32);

	pr_debug("TI960 set stream, enable %d\n", enable);

	ssd = node->sd;
	if (!ssd)
		return -EINVAL;

	sd = i2c_get_clientdata(ssd->client);
	if (!sd)
		return -EINVAL;

	va = to_ti960(sd);
	if (!va)
		return -EINVAL;

	bitmap_zero(rx_port_enabled, 32);

	j = ti960_find_subdev_index(va, ssd);
	if (j < 0)
		return -EINVAL;

	rx_port = va->sub_devs[j].rx_port;
	ser_alias = va->sub_devs[j].ser_i2c_addr;
	rval = ti960_rx_port_config(va, rx_port);
	if (rval < 0)
		return rval;

	bitmap_set(rx_port_enabled, rx_port, 1);

	pr_info("%s, set stream for %s, enable %d\n", __func__,
		va->sub_devs[j].sd_name, enable);

	/*
	 * FIXME: For now we only turn on 2 rx port.
	 * Port 0 and port 1.
	 */
	rval = regmap_write(va->regmap8, 0x0c, 0x03);
	if (rval) {
		pr_err("Failed to turn on port 0 and port 1\n");
		return rval;
	}

	sensor_sd = i2c_get_clientdata(va->sub_devs[j].sensor_client);
	if (!sensor_sd)
		return -EINVAL;

	if (!sensor_sd->node.node_set_streaming)
		return -EINVAL;

	sensor_sd->node.node_set_streaming(&sensor_sd->node, NULL, enable);

	/*
	 * FIXME: workaround for ov495 block issue.
	 * reset Ser TI953, to avoid ov495 block,
	 * only do reset for ov495, then it won't
	 * break other sensors.
	 */
	ti953_reg_write(va, rx_port, ser_alias, 0x0e, 0xf0);
	msleep(50);
	ti953_reg_write(va, rx_port, ser_alias, 0x0d, 00);
	msleep(50);
	ti953_reg_write(va, rx_port, ser_alias, 0x0d, 0x1);

	/* RX port forward */
	rval = ti960_reg_set_bit(va, TI960_FWD_CTL1, rx_port + 4, !enable);
	if (rval) {
		pr_err("Failed to forward RX port %d. Enable %d\n", rx_port, enable);
		return rval;
	}

	return 0;
}

static int ti960_register_subdev(struct i2c_client *client, struct ti960 *va)
{
	int i;

	va->sd.client = client;
	va->sd.do_register = ti960_registered;
	va->sd.do_unregister = ti960_unregistered;
	i2c_set_clientdata(client, &va->sd);
	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		va->ffmts[i]->ffmt.width = 0;
		va->ffmts[i]->ffmt.height = 0;
		va->ffmts[i]->ffmt.pixelformat = ICI_FORMAT_UYVY;
	}
	return 0;
}

struct slave_register_devid {
	u16 reg;
	u8 val_expected;
};

#define OV495_I2C_PHY_ADDR	0x48
#define OV495_I2C_ALIAS_ADDR	0x30

static const struct slave_register_devid ov495_devid[] = {
	{0x3000, 0x51},
	{0x3001, 0x49},
	{0x3002, 0x56},
	{0x3003, 0x4f},
};

/*
 * read sensor id reg of 16 bit addr, and 8 bit val
 */
static int slave_id_read(struct i2c_client *client, u8 i2c_addr,
				u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	unsigned char data[2];
	int rval;

	/* override i2c_addr */
	msg[0].addr = i2c_addr;
	msg[0].flags = 0;
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	msg[0].buf = data;
	msg[0].len = 2;

	msg[1].addr = i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 1;

	rval = i2c_transfer(client->adapter, msg, 2);

	if (rval < 0)
		return rval;

	*val = data[0];

	return 0;
}

static bool slave_detect(struct ti960 *va, u8 i2c_addr,
		const struct slave_register_devid *slave_devid, u8 len)
{
	struct i2c_client *client = va->sd.client;
	int i;
	int rval;
	unsigned char val;

	for (i = 0; i < len; i++) {
		rval = slave_id_read(client, i2c_addr,
			slave_devid[i].reg, &val);
		if (rval) {
			pr_err("slave id read fail %d\n", rval);
			break;
		}
		if (val != slave_devid[i].val_expected)
			break;
	}

	if (i == len)
		return true;

	return false;
}

static int ti960_init(struct ti960 *va)
{
	unsigned int reset_gpio = va->pdata->reset_gpio;
	int i, rval;
	unsigned int val;
	int m;
	int rx_port = 0;
	int ser_alias = 0;
	bool ov495_detected;

	gpio_set_value(reset_gpio, 1);
	usleep_range(2000, 3000);
	pr_err("Setting reset gpio %d to 1.\n", reset_gpio);

	rval = ti960_reg_read(va, TI960_DEVID, &val);
	if (rval) {
		pr_err("Failed to read device ID of TI960!\n");
		return rval;
	}
	pr_info("TI960 device ID: 0x%X\n", val);

	for (i = 0; i < ARRAY_SIZE(ti960_gpio_settings); i++) {
		rval = regmap_write(va->regmap8,
			ti960_gpio_settings[i].reg,
			ti960_gpio_settings[i].val);
		if (rval) {
			pr_err("Failed to write TI960 gpio setting, reg %2x, val %2x\n",
				ti960_gpio_settings[i].reg,
				ti960_gpio_settings[i].val);
			return rval;
		}
	}
	usleep_range(10000, 11000);

	/*
	 * fixed value of sensor phy, ser_alias, port config
	 * for ti960 each port, not yet known sensor platform data here.
	 */
	ser_alias = 0x58;
	for (i = 0; i < ARRAY_SIZE(ti960_init_settings); i++) {
		rval = regmap_write(va->regmap8,
			ti960_init_settings[i].reg,
			ti960_init_settings[i].val);
		if (rval) {
			pr_err("Failed to write TI960 init setting, reg %2x, val %2x\n",
				ti960_init_settings[i].reg,
				ti960_init_settings[i].val);
			return rval;
		}
	}

	/* wait for ti953 ready */
	msleep(200);

	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		unsigned short rx_port, phy_i2c_addr, alias_i2c_addr;

		rx_port = i;
		phy_i2c_addr = OV495_I2C_PHY_ADDR;
		alias_i2c_addr = OV495_I2C_ALIAS_ADDR;

		rval = ti960_map_phy_i2c_addr(va, rx_port, phy_i2c_addr);
		if (rval)
			return rval;

		rval = ti960_map_alias_i2c_addr(va, rx_port,
						alias_i2c_addr << 1);
		if (rval)
			return rval;

		ov495_detected = slave_detect(va, alias_i2c_addr,
					ov495_devid, ARRAY_SIZE(ov495_devid));

		/* unmap to clear i2c addr space */
		rval = ti960_map_phy_i2c_addr(va, rx_port, 0);
		if (rval)
			return rval;

		rval = ti960_map_alias_i2c_addr(va, rx_port, 0);
		if (rval)
			return rval;

		if (ov495_detected) {
			pr_info("ov495 detected on port %d\n", rx_port);
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(ti953_init_settings); i++) {
		if (ov495_detected)
			break;
		rval = ti953_reg_write(va, rx_port, ser_alias,
			ti953_init_settings[i].reg,
			ti953_init_settings[i].val);
		if (rval) {
			pr_err("port %d, ti953 write timeout %d\n", 0, rval);
			break;
		}
	}

	for (m = 0; m < ARRAY_SIZE(ti960_init_settings_2); m++) {
		rval = regmap_write(va->regmap8,
			ti960_init_settings_2[m].reg,
			ti960_init_settings_2[m].val);
		if (rval) {
			pr_err("Failed to write TI960 init setting 2, reg %2x, val %2x\n",
				ti960_init_settings_2[m].reg,
				ti960_init_settings_2[m].val);
			break;
		}
	}

	rval = regmap_write(va->regmap8, TI960_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;
	for (m = 1; m < ARRAY_SIZE(ti960_init_settings_3); m++) {
		rval = regmap_write(va->regmap8,
			ti960_init_settings_3[m].reg,
			ti960_init_settings_3[m].val);
		if (rval) {
			pr_err("Failed to write TI960 init setting 2, reg %2x, val %2x\n",
				ti960_init_settings_3[m].reg,
				ti960_init_settings_3[m].val);
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(ti953_init_settings_2); i++) {
		if (ov495_detected)
			break;
		rval = ti953_reg_write(va, rx_port, ser_alias,
			ti953_init_settings_2[i].reg,
			ti953_init_settings_2[i].val);
		if (rval) {
			pr_err("port %d, ti953 write timeout %d\n", 0, rval);
			break;
		}
	}

	/* reset and power for ti953 */
	if (!ov495_detected) {
		ti953_reg_write(va, 0, ser_alias, 0x0d, 00);
		msleep(50);
		ti953_reg_write(va, 0, ser_alias, 0x0d, 0x3);
	}

	rval = ti960_map_subdevs_addr(va);
	if (rval)
		return rval;

	return 0;
}

static void ti960_gpio_set(struct gpio_chip *chip,
	unsigned int gpio,
	int value)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	struct i2c_client *client = to_i2c_client(chip->dev);
#else
	struct i2c_client *client = to_i2c_client(chip->parent);
#endif
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);
	struct ti960 *va = to_ti960(subdev);
	unsigned int reg_val;
	int rx_port, gpio_port;
	int ret;

	if (gpio >= NR_OF_TI960_GPIOS)
		return;

	rx_port = gpio / NR_OF_GPIOS_PER_PORT;
	gpio_port = gpio % NR_OF_GPIOS_PER_PORT;

	ret = regmap_write(va->regmap8, TI960_RX_PORT_SEL,
			  (rx_port << 4) + (1 << rx_port));
	if (ret) {
		pr_debug("Failed to select RX port.\n");
		return;
	}
	ret = regmap_read(va->regmap8, TI960_BC_GPIO_CTL0, &reg_val);
	if (ret) {
		pr_debug("Failed to read gpio status.\n");
		return;
	}

	if (gpio_port == 0) {
		reg_val &= ~TI960_GPIO0_MASK;
		reg_val |= value ? TI960_GPIO0_HIGH : TI960_GPIO0_LOW;
	} else {
		reg_val &= ~TI960_GPIO1_MASK;
		reg_val |= value ? TI960_GPIO1_HIGH : TI960_GPIO1_LOW;
	}

	ret = regmap_write(va->regmap8, TI960_BC_GPIO_CTL0, reg_val);
	if (ret)
		pr_debug("Failed to set gpio.\n");
}

static int ti960_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int gpio, int level)
{
	return 0;
}

static int ti960_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct ti960 *va;
	int i, rval = 0;

	if (client->dev.platform_data == NULL)
		return -ENODEV;

	va = devm_kzalloc(&client->dev, sizeof(*va), GFP_KERNEL);
	if (!va)
		return -ENOMEM;

	va->pdata = client->dev.platform_data;

	va->nsources = NR_OF_TI960_SOURCE_PADS;
	va->nsinks = NR_OF_TI960_SINK_PADS;
	va->npads = NR_OF_TI960_PADS;
	va->nstreams = NR_OF_TI960_STREAMS;

	va->crop = devm_kcalloc(&client->dev, va->npads,
				sizeof(struct ici_rect), GFP_KERNEL);

	va->compose = devm_kcalloc(&client->dev, va->npads,
				   sizeof(struct ici_rect), GFP_KERNEL);

	if (!va->crop || !va->compose)
		return -ENOMEM;

	for (i = 0; i < va->npads; i++) {
		va->ffmts[i] = devm_kcalloc(&client->dev, va->nstreams,
					    sizeof(struct ici_pad_framefmt),
					    GFP_KERNEL);
		if (!va->ffmts[i])
			return -ENOMEM;
	}

	va->regmap8 = devm_regmap_init_i2c(client,
					   &ti960_reg_config8);
	if (IS_ERR(va->regmap8)) {
		pr_err("Failed to init regmap8!\n");
		return -EIO;
	}

	va->regmap16 = devm_regmap_init_i2c(client,
					    &ti960_reg_config16);
	if (IS_ERR(va->regmap16)) {
		pr_err("Failed to init regmap16!\n");
		return -EIO;
	}

	mutex_init(&va->mutex);
	rval = ti960_register_subdev(client, va);
	if (rval) {
		pr_err("Failed to register va subdevice!\n");
		return rval;
	}

	if (devm_gpio_request_one(&client->dev, va->pdata->reset_gpio, 0,
				  "ti960 reset") != 0) {
		pr_err("Unable to acquire gpio %d\n",
			va->pdata->reset_gpio);
		return -ENODEV;
	}

	rval = ti960_init(va);
	if (rval) {
		pr_err("Failed to init TI960!\n");
		return rval;
	}

	/*
	 * TI960 has several back channel GPIOs.
	 * We export GPIO0 and GPIO1 to control reset or fsin.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	va->gc.dev = &client->dev;
#else
	va->gc.parent = &client->dev;
#endif
	va->gc.owner = THIS_MODULE;
	va->gc.label = "TI960 GPIO";
	va->gc.ngpio = NR_OF_TI960_GPIOS;
	va->gc.base = -1;
	va->gc.set = ti960_gpio_set;
	va->gc.direction_output = ti960_gpio_direction_output;
	rval = gpiochip_add(&va->gc);
	if (rval) {
		pr_err("Failed to add gpio chip!\n");
		return -EIO;
	}

	return 0;
}

static int ti960_remove(struct i2c_client *client)
{
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);
	struct ti960 *va;
	struct i2c_client *sub_client;
	int i;

	if (!subdev)
		return -EINVAL;

	va = to_ti960(subdev);

	if (!va)
		return 0;

	mutex_destroy(&va->mutex);

	for (i = 0; i < NR_OF_TI960_SINK_PADS; i++) {
		if (va->sub_devs[i].sd) {
			sub_client = va->sub_devs[i].sd->client;
			i2c_unregister_device(sub_client);
		}
		va->sub_devs[i].sd = NULL;
	}
	gpiochip_remove(&va->gc);

	return 0;
}

#ifdef CONFIG_PM
static int ti960_suspend(struct device *dev)
{
	return 0;
}

static int ti960_resume(struct device *dev)
{
	struct ti960 *va = NULL;
	struct i2c_client *client = to_i2c_client(dev);
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);

	if (!subdev)
		return -EINVAL;

	if (!va)
		return -EINVAL;

	return ti960_init(va);
}
#else
#define ti960_suspend	NULL
#define ti960_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id ti960_id_table[] = {
	{ TI960_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ti960_id_table);

static const struct dev_pm_ops ti960_pm_ops = {
	.suspend = ti960_suspend,
	.resume = ti960_resume,
};

static struct i2c_driver ti960_i2c_driver = {
	.driver = {
		.name = TI960_NAME,
		.pm = &ti960_pm_ops,
	},
	.probe	= ti960_probe,
	.remove	= ti960_remove,
	.id_table = ti960_id_table,
};
module_i2c_driver(ti960_i2c_driver);

MODULE_AUTHOR("Puunithaaraj Gopal <puunithaaraj.gopalintel.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("TI960 CSI2-Aggregator driver");
