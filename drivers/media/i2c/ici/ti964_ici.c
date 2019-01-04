// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2016 - 2018 Intel Corporation

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <media/ti964.h>
#include <media/crlmodule-lite.h>
#include <media/ici.h>
#include <linux/version.h>
#include "../ti964-reg.h"

struct ti964_subdev {
	struct ici_ext_subdev *sd;
	unsigned short rx_port;
	unsigned short fsin_gpio;
	unsigned short phy_i2c_addr;
	unsigned short alias_i2c_addr;
	char sd_name[ICI_MAX_NODE_NAME];
};

struct ti964 {
	struct ici_ext_subdev sd;
	struct ici_ext_subdev_register reg;
	struct ti964_pdata *pdata;
	struct ti964_subdev sub_devs[NR_OF_TI964_SINK_PADS];
	struct crlmodule_lite_platform_data subdev_pdata[NR_OF_TI964_SINK_PADS];
	const char *name;

	struct mutex mutex;

	struct regmap *regmap8;
	struct regmap *regmap16;

	struct ici_pad_framefmt *ffmts[NR_OF_TI964_SINK_PADS];
	struct ici_rect *crop;
	struct ici_rect *compose;

	struct {
		unsigned int *stream_id;
	} *stream; /* stream enable/disable status, indexed by pad */
	struct {
		unsigned int sink;
		unsigned int source;
		int flags;
	} *route; /* pad level info, indexed by stream */

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
};

static int init_ext_sd(struct i2c_client *client, struct ti964_subdev *sd, int idx);
static int ti964_find_subdev_index(struct ti964 *va, struct ici_ext_subdev *sd);
static int create_link(struct ici_isys_node *src_node, u16 srcpad,
	struct ici_isys_node *sink_node, u16 sinkpad, u32 flag);
static int ti964_get_param(struct ici_ext_sd_param *param);
static int ti964_get_menu_item(struct ici_ext_sd_param *param, u32 idx);
static int ti964_set_param(struct ici_ext_sd_param *param);

#define to_ti964(_sd) container_of(_sd, struct ti964, sd)
#define to_ici_ext_subdev(_node) container_of(_node, struct ici_ext_subdev, node)
#define TI964_SRC_PAD 1

static const s64 ti964_op_sys_clock[] =  {400000000, 800000000};
static const u8 ti964_op_sys_clock_reg_val[] = {
	TI964_MIPI_800MBPS,
	TI964_MIPI_1600MBPS
};


static const struct ti964_csi_data_format va_csi_data_formats[] = {
		{ ICI_FORMAT_YUYV, 16, 16, PIXEL_ORDER_GBRG, 0x1e },
		{ ICI_FORMAT_UYVY, 16, 16, PIXEL_ORDER_GBRG, 0x1e },
		{ ICI_FORMAT_SGRBG12, 12, 12, PIXEL_ORDER_GRBG, 0x2c },
		{ ICI_FORMAT_SRGGB12, 12, 12, PIXEL_ORDER_RGGB, 0x2c },
		{ ICI_FORMAT_SBGGR12, 12, 12, PIXEL_ORDER_BGGR, 0x2c },
		{ ICI_FORMAT_SGBRG12, 12, 12, PIXEL_ORDER_GBRG, 0x2c },
		{ ICI_FORMAT_SGRBG10, 10, 10, PIXEL_ORDER_GRBG, 0x2b },
		{ ICI_FORMAT_SRGGB10, 10, 10, PIXEL_ORDER_RGGB, 0x2b },
		{ ICI_FORMAT_SBGGR10, 10, 10, PIXEL_ORDER_BGGR, 0x2b },
		{ ICI_FORMAT_SGBRG10, 10, 10, PIXEL_ORDER_GBRG, 0x2b },
		{ ICI_FORMAT_SGRBG8, 8, 8, PIXEL_ORDER_GRBG, 0x2a },
		{ ICI_FORMAT_SRGGB8, 8, 8, PIXEL_ORDER_RGGB, 0x2a },
		{ ICI_FORMAT_SBGGR8, 8, 8, PIXEL_ORDER_BGGR, 0x2a },
		{ ICI_FORMAT_SGBRG8, 8, 8, PIXEL_ORDER_GBRG, 0x2a },
};


static struct regmap_config ti964_reg_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct regmap_config ti964_reg_config16 = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
};

static int ti964_reg_set_bit(struct ti964 *va, unsigned char reg,
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

static int ti964_map_phy_i2c_addr(struct ti964 *va, unsigned short rx_port,
				  unsigned short addr)
{
	int rval;

	rval = regmap_write(va->regmap8, TI964_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI964_SLAVE_ID0, addr);
}

static int ti964_map_alias_i2c_addr(struct ti964 *va, unsigned short rx_port,
				  unsigned short addr)
{
	int rval;

	rval = regmap_write(va->regmap8, TI964_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;

	return regmap_write(va->regmap8, TI964_SLAVE_ALIAS_ID0, addr);
}

static int ti964_fsin_gpio_init(struct ti964 *va, unsigned short rx_port,
					unsigned short fsin_gpio)
{
	int rval;
	int reg_val;

	rval = regmap_read(va->regmap8, TI964_FS_CTL, &reg_val);
	if (rval) {
		pr_err("Failed to read gpio status.\n");
		return rval;
	}

	if (!reg_val & TI964_FSIN_ENABLE) {
		pr_warn("FSIN not enabled, skip config FSIN GPIO.\n");
		return 0;
	}

	rval = regmap_write(va->regmap8, TI964_RX_PORT_SEL,
		(rx_port << 4) + (1 << rx_port));
	if (rval)
		return rval;

	rval = regmap_read(va->regmap8, TI964_BC_GPIO_CTL0, &reg_val);
	if (rval) {
		pr_err("Failed to read gpio status.\n");
		return rval;
	}

	if (fsin_gpio == 0) {
		reg_val &= ~TI964_GPIO0_MASK;
		reg_val |= TI964_GPIO0_FSIN;
	} else {
		reg_val &= ~TI964_GPIO1_MASK;
		reg_val |= TI964_GPIO1_FSIN;
	}

	rval = regmap_write(va->regmap8, TI964_BC_GPIO_CTL0, reg_val);
	if (rval)
		pr_err("Failed to set gpio.\n");

	return rval;
}

/*
 * Function main code replicated from /drivers/media/i2c/smiapp/smiapp-core.c
 * Slightly modified based on the CRL Module changes
 */
static int ti964_enum_mbus_code(struct ici_isys_node *node, struct ici_pad_supported_format_desc *psfd)
{

	psfd->color_format = 0x1e; // for ICI_FORMAT_UYVY
		// sensor->sensor_ds->csi_fmts[psfd->idx].code;
	psfd->min_width = TI964_MIN_WIDTH; //sensor->sensor_ds->sensor_limits->x_addr_min;
	psfd->max_width = TI964_MAX_WIDTH; //sensor->sensor_ds->sensor_limits->x_addr_max;
	psfd->min_height = TI964_MIN_HEIGHT; //sensor->sensor_ds->sensor_limits->y_addr_min;
	psfd->max_height = TI964_MAX_HEIGHT; //sensor->sensor_ds->sensor_limits->y_addr_max;
	return 0;
}

static const struct ti964_csi_data_format
		*ti964_validate_csi_data_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(va_csi_data_formats); i++) {
		if (va_csi_data_formats[i].code == code)
			return &va_csi_data_formats[i];
	}

	return &va_csi_data_formats[0];
}

static int __ti964_set_format(struct ici_ext_subdev *subdev, struct ici_pad_framefmt *pff)
{
	struct i2c_client *client = subdev->client;
	struct ici_ext_subdev *sd = i2c_get_clientdata(client);
	struct ti964 *va = to_ti964(sd);
	int index = ti964_find_subdev_index(va, subdev);


	va->ffmts[index]->ffmt.width = pff->ffmt.width;
	va->ffmts[index]->ffmt.height = pff->ffmt.height;
	va->ffmts[index]->ffmt.pixelformat = pff->ffmt.pixelformat;
	va->ffmts[index]->ffmt.field = pff->ffmt.field;

	return 0;
}

static int __ti964_get_format(struct ici_ext_subdev *subdev, struct ici_pad_framefmt *pff)
{
	struct i2c_client *client = subdev->client;
	struct ici_ext_subdev *sd = i2c_get_clientdata(client);
	struct ti964 *va = to_ti964(sd);
	int index = ti964_find_subdev_index(va, subdev);

// TODO hardocded same format for all pads of TI964
	pff->ffmt.width = va->ffmts[index]->ffmt.width;
	pff->ffmt.height = va->ffmts[index]->ffmt.height;
	pff->ffmt.pixelformat = va->ffmts[index]->ffmt.pixelformat;
//				  sensor->sensor_ds->csi_fmts[sensor->fmt_index].code;
	pff->ffmt.field =
				((va->ffmts[index]->ffmt.field == ICI_FIELD_ANY) ?
				ICI_FIELD_NONE : va->ffmts[index]->ffmt.field);
	return 0;

}

static int ti964_set_selection(struct ici_isys_node *node, struct ici_pad_selection *ps)
{
// TODO place holder
	return 0;
}

static int ti964_get_selection(struct ici_isys_node *node, struct ici_pad_selection *ps)
{
// TODO place holder
	return 0;
}

static int ti964_get_format(struct ici_isys_node *node, struct ici_pad_framefmt *pff)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct ti964 *va = to_ti964(subdev);

	mutex_lock(&va->mutex);
	__ti964_get_format(subsubdev, pff);
	mutex_unlock(&va->mutex);

//	pr_debug(subdev->dev, "subdev_format: which: %s, pad: %d, stream: %d.\n",
//		 fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE ?
//		 "V4L2_SUBDEV_FORMAT_ACTIVE" : "V4L2_SUBDEV_FORMAT_TRY",
//		 fmt->pad, fmt->stream);

//	pr_debug("framefmt: width: %d, height: %d, code: 0x%x.\n",
//		   fmt->format.width, fmt->format.height, fmt->format.code);

	return 0;
}

static int ti964_set_format(struct ici_isys_node *node, struct ici_pad_framefmt *pff)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct ti964 *va = to_ti964(subdev);

	mutex_lock(&va->mutex);
	__ti964_set_format(subsubdev, pff);

	mutex_unlock(&va->mutex);

	pr_debug("framefmt: width: %d, height: %d, code: 0x%x.\n",
		   pff->ffmt.width, pff->ffmt.height, pff->ffmt.pixelformat);

	return 0;
}
/* Initialize sensor connected to TI964 */
static int ti964_init_ext_subdev(struct ti964_subdev_info *info,
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
		return -ENODEV;
	}

	/* Get the clientdata set by the sensor driver */
	sensor_sd = i2c_get_clientdata(client2);
	if (!sensor_sd) {
		pr_err("%s, Failed to get client data\n", __func__);
		return -EINVAL;
	}

	sd_register->ipu_data = reg->ipu_data;
	sd_register->sd = sensor_sd;
	sd_register->setup_node = reg->setup_node;
	sd_register->create_link = reg->create_link;

	rval = sensor_sd->do_register(sd_register);

	return rval;
}

static int ti964_registered(struct ici_ext_subdev_register *reg)
{
	struct ici_ext_subdev *subdev = reg->sd;
	struct ti964 *va = to_ti964(subdev);
	struct ti964_subdev *sd, *prev_sd = NULL;
	struct i2c_client *client = subdev->client;
	struct ici_ext_subdev_register sd_register = {0};
	int i, k, rval;

		if (!reg->sd || !reg->setup_node || !reg->create_link) {
		pr_err("ti964_registered error\n");
				return -EINVAL;
	}

	va->reg = *reg;
	va->create_link = reg->create_link;

	/* ti964->subdev represents the ti964 itself and
	 ti964->sub_devs represents every port/vc */
	subdev->get_param = ti964_get_param;
	subdev->set_param = ti964_set_param;
	subdev->get_menu_item = ti964_get_menu_item;

	for (i = 0, k = 0; i < va->pdata->subdev_num; i++) {
		struct ti964_subdev_info *info =
			&va->pdata->subdev_info[i];
		struct crlmodule_lite_platform_data *pdata =
			(struct crlmodule_lite_platform_data *)
			info->board_info.platform_data;

		if (k >= va->nsinks)
			break;

		/*
		 * The sensors should not share the same pdata structure.
		 * Clone the pdata for each sensor.
		 */
		memcpy(&va->subdev_pdata[k], pdata, sizeof(*pdata));
		if (va->subdev_pdata[k].xshutdown != 0 &&
			va->subdev_pdata[k].xshutdown != 1) {
			pr_err("xshutdown(%d) must be 0 or 1 to connect.\n",
				va->subdev_pdata[k].xshutdown);
			return -EINVAL;
		}

		/* If 0 is xshutdown, then 1 would be FSIN, vice versa. */
		va->sub_devs[k].fsin_gpio = 1 - va->subdev_pdata[k].xshutdown;

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
			pr_err("No physical i2c address and alias i2c address found\n");
			return -EINVAL;
		}

		/* Map PHY I2C address. */
		rval = ti964_map_phy_i2c_addr(va, info->rx_port,
					info->phy_i2c_addr);
		if (rval)
			return rval;

		/* Map 7bit ALIAS I2C address. */
		rval = ti964_map_alias_i2c_addr(va, info->rx_port,
				info->board_info.addr << 1);
		if (rval)
			return rval;

		/* Initialize sensor connected to TI964 */
		rval  = ti964_init_ext_subdev(info, reg, client,
						  &sd_register);
		if (rval) {
			pr_err("%s, Failed to register external subdev\n", __func__);
			continue;
		}

		/* Config FSIN GPIO */
		rval = ti964_fsin_gpio_init(va, info->rx_port,
				va->sub_devs[k].fsin_gpio);
		if (rval)
			return rval;

		/* Allocate ici_ext_subdev for each TI964 port */
		va->sub_devs[k].sd = devm_kzalloc(&client->dev, sizeof(struct ici_ext_subdev), GFP_KERNEL);
		if (!va->sub_devs[k].sd) {
			pr_err("can't create new i2c subdev %d-%04x\n",
				info->i2c_adapter_id,
				info->board_info.addr);
			continue;
		}
		va->sub_devs[k].rx_port = info->rx_port;
		va->sub_devs[k].phy_i2c_addr = info->phy_i2c_addr;
		va->sub_devs[k].alias_i2c_addr = info->board_info.addr;
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
				if (prev_sd == NULL) {
						prev_sd = sd;
			k++;
						continue;
				}
				prev_sd = sd;

		k++;
	}
	/* Replace existing create_link address with TI964 create_link implementation
	   to create link between TI964 node and CSI2 node */
	reg->create_link = create_link;
	return 0;
}


static int create_link(struct ici_isys_node *src_node,
	u16 srcpad,
	struct ici_isys_node *sink_node,
	u16 sinkpad,
	u32 flag)
{
	struct ici_ext_subdev *sd, *ssd;
	struct ti964 *va;
	struct ti964_subdev *subdev;
	int i, ret;
	if (!src_node || !sink_node)
		return -EINVAL;

	sd = to_ici_ext_subdev(src_node);
	if (!sd)
		return -EINVAL;

	va = to_ti964(sd);
	if (!va)
		return -EINVAL;

	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {
		subdev = &va->sub_devs[i];
		if (!subdev)
			continue;
		ssd = subdev->sd;
		ret = va->create_link(&ssd->node,
			TI964_SRC_PAD,
			sink_node,
			sinkpad,
			0);
		if (ret)
			return ret;
	}
	return 0;
}

static void ti964_unregistered(struct ici_ext_subdev *subdev)
{
	pr_debug("%s\n", __func__);
}

static int ti964_set_param(struct ici_ext_sd_param *param)
{
	return 0;
}

static int ti964_set_power(struct ici_isys_node *node, int on)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ti964 *va;
	int ret;
	u8 val;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	if (!subdev)
		return -EINVAL;

	va = to_ti964(subdev);

	if (!va)
		return -EINVAL;

	pr_debug("%s %d\n", __func__, on);
	ret = regmap_write(va->regmap8, TI964_RESET,
			   (on) ? TI964_POWER_ON : TI964_POWER_OFF);
	if (ret || !on)
		return ret;

	/* Configure MIPI clock bsaed on control value. */
	ret = regmap_write(va->regmap8, TI964_CSI_PLL_CTL,
				ti964_op_sys_clock_reg_val[0]);
//				ti964_op_sys_clock_reg_val[
//				v4l2_ctrl_g_ctrl(va->link_freq)]);
	if (ret)
		return ret;
	val = TI964_CSI_ENABLE;
//	val |= TI964_CSI_CONTS_CLOCK;
	/* Enable skew calculation when 1.6Gbps output is enabled. */
// TODO pegging to 0.8 Gbps for now
//	if (v4l2_ctrl_g_ctrl(va->link_freq))
//		val |= TI964_CSI_SKEWCAL;
	return regmap_write(va->regmap8, TI964_CSI_CTL, val);
}

#ifdef TEST_PATTERN
static int ti964_tp_set_stream(struct ici_ext_subdev *subdev, int enable)
{
	struct ti964 *va = to_ti964(subdev);
	int i, rval;

	for (i = 0; i < ARRAY_SIZE(ti964_tp_settings); i++) {
		rval = regmap_write(va->regmap8,
			ti964_tp_settings[i].reg,
			ti964_tp_settings[i].val);
		if (rval) {
			pr_err("Register write error.\n");
			return rval;
		}
	}

	rval = regmap_write(va->regmap8, TI964_IND_ACC_DATA, enable);
	if (rval) {
		pr_err("Register write error.\n");
		return rval;
	}

	return 0;
}
#endif

static int ti964_rx_port_config(struct ti964 *va, int sink, int rx_port)
{
	int rval;
	u8 bpp;
	int port_cfg2_val;
	int vc_mode_reg_index;
	int vc_mode_reg_val;
	int mipi_dt_type;
	int high_fv_flags = va->subdev_pdata[sink].high_framevalid_flags;

	/* Select RX port. */
	rval = regmap_write(va->regmap8, TI964_RX_PORT_SEL,
			(rx_port << 4) + (1 << rx_port));
	if (rval) {
		pr_err("Failed to select RX port.\n");
		return rval;
	}

	/* Set RX port mode. */
	bpp = ti964_validate_csi_data_format(
		va->ffmts[0]->ffmt.pixelformat)->width;
	rval = regmap_write(va->regmap8, TI964_PORT_CONFIG,
		(bpp == 12) ?
		TI964_FPD3_RAW12_75MHz : TI964_FPD3_RAW10_100MHz);
	if (rval) {
		pr_err("Failed to set port config.\n");
		return rval;
	}

	mipi_dt_type = ti964_validate_csi_data_format(
		va->ffmts[0]->ffmt.pixelformat)->mipi_dt_code;
	/*
	 * RAW8 and YUV422 need to enable RAW10 bit mode.
	 * RAW12 need to set the RAW10_8bit to reserved.
	 */
	switch (bpp) {
	case 8:
	case 16:
		port_cfg2_val = TI964_RAW10_8BIT & (~high_fv_flags);
		vc_mode_reg_index = TI964_RAW10_ID;
		break;
	case 12:
		port_cfg2_val = TI964_RAW12;
		vc_mode_reg_index = TI964_RAW12_ID;
		break;
	default:
		port_cfg2_val = TI964_RAW10_NORMAL & (~high_fv_flags);
		vc_mode_reg_index = TI964_RAW10_ID;
		break;
	}

	vc_mode_reg_val =  mipi_dt_type | sink << 6;
#if 0
	rval = regmap_write(va->regmap8, vc_mode_reg_index, vc_mode_reg_val);
	if (rval) {
		pr_err("Failed to set virtual channel & data type.\n");
		return rval;
	}
#endif

	rval = regmap_write(va->regmap8, TI964_PORT_CONFIG2, port_cfg2_val);
	if (rval) {
		pr_err("Failed to set port config2.\n");
		return rval;
	}

	return 0;
}

static int ti964_map_subdevs_addr(struct ti964 *va)
{
	unsigned short rx_port, phy_i2c_addr, alias_i2c_addr;
	int i, rval;

	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {
		rx_port = va->sub_devs[i].rx_port;
		phy_i2c_addr = va->sub_devs[i].phy_i2c_addr;
		alias_i2c_addr = va->sub_devs[i].alias_i2c_addr;

		if (!phy_i2c_addr || !alias_i2c_addr)
			continue;

		rval = ti964_map_phy_i2c_addr(va, rx_port, phy_i2c_addr);
		if (rval)
			return rval;

		/* set 7bit alias i2c addr */
		rval = ti964_map_alias_i2c_addr(va, rx_port,
						alias_i2c_addr << 1);
		if (rval)
			return rval;
	}

	return 0;
}

static int ti964_find_subdev_index(struct ti964 *va, struct ici_ext_subdev *sd)
{
	int i;

	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {
		if (va->sub_devs[i].sd == sd)
			return i;
	}

	WARN_ON(1);

	return -EINVAL;
}

static int ti964_set_stream(struct ici_isys_node *node, void *ip, int enable)
{
	struct ti964 *va;
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	int i, j, rval;
	unsigned int rx_port;
	DECLARE_BITMAP(rx_port_enabled, 32);

	if (!subdev)
		return -EINVAL;

	va = to_ti964(subdev);

	if (!va)
		return -EINVAL;

	pr_debug("TI964 set stream, enable %d\n", enable);
#ifdef TEST_PATTERN
		return ti964_tp_set_stream(subsubdev, enable);
#endif

	bitmap_zero(rx_port_enabled, 32);
	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {
		j = ti964_find_subdev_index(va, subsubdev);
		if (j < 0)
			return -EINVAL;
		rx_port = va->sub_devs[j].rx_port;

		rval = ti964_rx_port_config(va, i, rx_port);
		if (rval < 0)
			return rval;

		bitmap_set(rx_port_enabled, rx_port, 1);
			/* RX port fordward */
		rval = ti964_reg_set_bit(va, TI964_FWD_CTL1,
					rx_port + 4, !enable);
		if (rval) {
			pr_err("Failed to forward RX port%d. enable %d\n",
				i, enable);
			return rval;
		}

	}

	return 0;
}

static int ti964_get_param(struct ici_ext_sd_param *param)
{
// TODO this is hard-coded for now

	param->val = 400000000;
// or param->val = 800000000;
	return 0;
}

static int ti964_get_menu_item(struct ici_ext_sd_param *param, u32 idx)
{
	return 0;
}

static int init_ext_sd(struct i2c_client *client, struct ti964_subdev *ti_sd, int idx)
{
	struct ti964 *va;
	char name[ICI_MAX_NODE_NAME];
	int rval;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);;

	if (!subdev)
		return -EINVAL;

	va = to_ti964(subdev);

	if (!va)
		return -EINVAL;

	if (va->pdata->suffix) {
		snprintf(name,
					sizeof(name), "TI964 %c %d",
					va->pdata->suffix, idx);
	} else
		snprintf(name,
					sizeof(name), "TI964 %4.4x %d",
					i2c_adapter_id(client->adapter), idx);

	ti_sd->sd->client = client;
	ti_sd->sd->num_pads = 2;
	//ti_sd->sd->pads[0].pad_id = 0;
	//ti_sd->sd->pads[0].flags = ICI_PAD_FLAGS_SINK;
	//ti_sd->sd->pads[1].pad_id = 1;
	//ti_sd->sd->pads[1].flags = ICI_PAD_FLAGS_SOURCE;
	// TODO
	//sd->src_pad = ssd->source_pad;
	// below fnctions invoked by csi2 fe code
	ti_sd->sd->set_param = ti964_set_param; // meant to execute CTRL-IDs/CIDs
	ti_sd->sd->get_param = ti964_get_param; // meant to execute CTRLIDs/CIDs
	ti_sd->sd->get_menu_item = ti964_get_menu_item; // get LINK FREQ
	if (va->reg.setup_node) {
		rval = va->reg.setup_node(va->reg.ipu_data,
									ti_sd->sd, name);
		if (rval)
				return rval;
	} else
		pr_err("node not registered\n");

// below invoked by stream code
	ti_sd->sd->node.node_set_power = ti964_set_power;
	ti_sd->sd->node.node_set_streaming = ti964_set_stream;
// below invoked by pipeline-device code
	ti_sd->sd->node.node_get_pad_supported_format =
				ti964_enum_mbus_code; //needs modification
	ti_sd->sd->node.node_set_pad_ffmt = ti964_set_format;
	ti_sd->sd->node.node_get_pad_ffmt = ti964_get_format;
	ti_sd->sd->node.node_set_pad_sel = ti964_set_selection;
	ti_sd->sd->node.node_get_pad_sel = ti964_get_selection;
	return 0;
}

static int ti964_register_subdev(struct i2c_client *client, struct ti964 *va)
{
	int rval = 0;
	int i = 0;

	va->sd.client = client;
	va->sd.do_register = ti964_registered;
	va->sd.do_unregister = ti964_unregistered;
	i2c_set_clientdata(client, &va->sd);
	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {
		va->ffmts[i]->ffmt.width = 1280;
		va->ffmts[i]->ffmt.height = 720;
		va->ffmts[i]->ffmt.pixelformat = ICI_FORMAT_UYVY;
	}
	return rval;
}

static int ti964_init(struct ti964 *va)
{
	unsigned int reset_gpio = va->pdata->reset_gpio;
	int i, rval;
	unsigned int val;

	gpio_set_value(reset_gpio, 1);
	usleep_range(2000, 3000);
	pr_debug("Setting reset gpio %d to 1.\n", reset_gpio);

	rval = regmap_read(va->regmap8, TI964_DEVID, &val);
	if (rval) {
		pr_err("Failed to read device ID of TI964!\n");
		return rval;
	}
	pr_info("TI964 device ID: 0x%X\n", val);

	for (i = 0; i < ARRAY_SIZE(ti964_init_settings); i++) {
		rval = regmap_write(va->regmap8,
			ti964_init_settings[i].reg,
			ti964_init_settings[i].val);
		if (rval)
			return rval;
	}

	rval = ti964_map_subdevs_addr(va);
	if (rval)
		return rval;

	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {
		rval = ti964_fsin_gpio_init(va, va->sub_devs[i].rx_port,
					va->sub_devs[i].fsin_gpio);
		if (rval)
			return rval;
	}

	return 0;
}

static void ti964_gpio_set(struct gpio_chip *chip, unsigned gpio, int value)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	struct i2c_client *client = to_i2c_client(chip->dev);
#else
	struct i2c_client *client = to_i2c_client(chip->parent);
#endif
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);
	struct ti964 *va = to_ti964(subdev);
	unsigned int reg_val;
	int rx_port, gpio_port;
	int ret;

	if (gpio >= NR_OF_TI964_GPIOS)
		return;

	rx_port = gpio / NR_OF_GPIOS_PER_PORT;
	gpio_port = gpio % NR_OF_GPIOS_PER_PORT;

	ret = regmap_write(va->regmap8, TI964_RX_PORT_SEL,
			  (rx_port << 4) + (1 << rx_port));
	if (ret) {
		pr_debug("Failed to select RX port.\n");
		return;
	}
	ret = regmap_read(va->regmap8, TI964_BC_GPIO_CTL0, &reg_val);
	if (ret) {
		pr_debug("Failed to read gpio status.\n");
		return;
	}

	if (gpio_port == 0) {
		reg_val &= ~TI964_GPIO0_MASK;
		reg_val |= value ? TI964_GPIO0_HIGH : TI964_GPIO0_LOW;
	} else {
		reg_val &= ~TI964_GPIO1_MASK;
		reg_val |= value ? TI964_GPIO1_HIGH : TI964_GPIO1_LOW;
	}

	ret = regmap_write(va->regmap8, TI964_BC_GPIO_CTL0, reg_val);
	if (ret)
		pr_debug("Failed to set gpio.\n");
}

static int ti964_gpio_direction_output(struct gpio_chip *chip,
					   unsigned gpio, int level)
{
	return 0;
}

static int ti964_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct ti964 *va;
	int i, rval = 0;

	if (client->dev.platform_data == NULL)
		return -ENODEV;

	va = devm_kzalloc(&client->dev, sizeof(*va), GFP_KERNEL);
	if (!va)
		return -ENOMEM;

	va->pdata = client->dev.platform_data;

	va->nsources = NR_OF_TI964_SOURCE_PADS;
	va->nsinks = NR_OF_TI964_SINK_PADS;
	va->npads = NR_OF_TI964_PADS;
	va->nstreams = NR_OF_TI964_STREAMS;

	va->crop = devm_kcalloc(&client->dev, va->npads,
				sizeof(struct ici_rect), GFP_KERNEL);

	va->compose = devm_kcalloc(&client->dev, va->npads,
				   sizeof(struct ici_rect), GFP_KERNEL);

	va->route = devm_kcalloc(&client->dev, va->nstreams,
					   sizeof(*va->route), GFP_KERNEL);

	va->stream = devm_kcalloc(&client->dev, va->npads,
					   sizeof(*va->stream), GFP_KERNEL);

	if (!va->crop || !va->compose || !va->route || !va->stream)
		return -ENOMEM;

	for (i = 0; i < va->npads; i++) {
		va->ffmts[i] = devm_kcalloc(&client->dev, va->nstreams,
						sizeof(struct ici_pad_framefmt),
						GFP_KERNEL);
		if (!va->ffmts[i])
			return -ENOMEM;

		va->stream[i].stream_id =
			devm_kcalloc(&client->dev, va->nsinks,
			sizeof(*va->stream[i].stream_id), GFP_KERNEL);
		if (!va->stream[i].stream_id)
			return -ENOMEM;
	}

	for (i = 0; i < va->nstreams; i++) {
		va->route[i].sink = i;
		va->route[i].source = TI964_PAD_SOURCE;
		va->route[i].flags = 0;
	}

	for (i = 0; i < va->nsinks; i++) {
		va->stream[i].stream_id[0] = i;
		va->stream[TI964_PAD_SOURCE].stream_id[i] = i;
	}

	va->regmap8 = devm_regmap_init_i2c(client,
					   &ti964_reg_config8);
	if (IS_ERR(va->regmap8)) {
		pr_err("Failed to init regmap8!\n");
		return -EIO;
	}

	va->regmap16 = devm_regmap_init_i2c(client,
						&ti964_reg_config16);
	if (IS_ERR(va->regmap16)) {
		pr_err("Failed to init regmap16!\n");
		return -EIO;
	}

	mutex_init(&va->mutex);
	rval = ti964_register_subdev(client, va);
	if (rval) {
		pr_err("Failed to register va subdevice!\n");
		return rval;
	}

	if (devm_gpio_request_one(&client->dev, va->pdata->reset_gpio, 0,
				  "ti964 reset") != 0) {
		pr_err("Unable to acquire gpio %d\n",
			va->pdata->reset_gpio);
		return -ENODEV;
	}

	rval = ti964_init(va);
	if (rval) {
		pr_err("Failed to init TI964!\n");
		return rval;
	}

	/*
	 * TI964 has several back channel GPIOs.
	 * We export GPIO0 and GPIO1 to control reset or fsin.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	va->gc.dev = &client->dev;
#else
	va->gc.parent = &client->dev;
#endif
	va->gc.owner = THIS_MODULE;
	va->gc.label = "TI964 GPIO";
	va->gc.ngpio = NR_OF_TI964_GPIOS;
	va->gc.base = -1;
	va->gc.set = ti964_gpio_set;
	va->gc.direction_output = ti964_gpio_direction_output;
	rval = gpiochip_add(&va->gc);
	if (rval) {
		pr_err("Failed to add gpio chip!\n");
		return -EIO;
	}

	return 0;
}

static int ti964_remove(struct i2c_client *client)
{
	struct ti964 *va;
	struct i2c_client *sub_client;
	int i;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);;

	if (!subdev)
		return -EINVAL;

	va = to_ti964(subdev);

	if (!va)
		return 0;

	mutex_destroy(&va->mutex);

	for (i = 0; i < NR_OF_TI964_SINK_PADS; i++) {

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
static int ti964_suspend(struct device *dev)
{
	return 0;
}

static int ti964_resume(struct device *dev)
{
	struct ti964 *va;
	struct i2c_client *client = to_i2c_client(dev);
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);;

	if (!subdev)
		return -EINVAL;

	va = to_ti964(subdev);

	if (!va)
		return -EINVAL;

	return ti964_init(va);
}
#else
#define ti964_suspend	NULL
#define ti964_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id ti964_id_table[] = {
	{ TI964_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ti964_id_table);

static const struct dev_pm_ops ti964_pm_ops = {
	.suspend = ti964_suspend,
	.resume = ti964_resume,
};

static struct i2c_driver ti964_i2c_driver = {
	.driver = {
		.name = TI964_NAME,
		.pm = &ti964_pm_ops,
	},
	.probe	= ti964_probe,
	.remove	= ti964_remove,
	.id_table = ti964_id_table,
};
module_i2c_driver(ti964_i2c_driver);

MODULE_AUTHOR("Karthik Gopalakrishnan <karthik.l.gopalakrishnan@intel.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("TI964 CSI2-Aggregator driver for RTOS reference");

