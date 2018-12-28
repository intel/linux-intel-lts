/* SPDX-LIcense_Identifier: GPL-2.0 */
/* Copyright (C) 2018 Intel Corporation */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <media/ici.h>
#include <media/max9286.h>
#include <media/crlmodule-lite.h>

#include "../max9286-reg-settings.h"

struct max9286_subdev {
	struct ici_ext_subdev *sd;
	unsigned short rx_port;
	unsigned short fsin_gpio;
	unsigned short phy_i2c_addr;
	unsigned short alias_i2c_addr;
	char sd_name[ICI_MAX_NODE_NAME];
};

struct max9286 {
	struct ici_ext_subdev ici_sd;
	struct ici_ext_subdev_register reg;
	struct max9286_pdata *pdata;
	struct crlmodule_lite_platform_data subdev_pdata[NR_OF_MAX_SINK_PADS];
	unsigned char sensor_present;
	unsigned int total_sensor_num;
	unsigned int nsources;
	unsigned int nsinks;
	unsigned int npads;
	unsigned int nstreams;
	const char *name;
	struct max9286_subdev sub_devs[NR_OF_MAX_SINK_PADS];
	struct ici_framefmt *ffmts[NR_OF_MAX_PADS];

	struct rect *crop;
	struct rect *compose;
	struct {
		unsigned int *stream_id;
	} *stream; /* stream enable/disable status, indexed by pad */
	struct {
		unsigned int sink;
		unsigned int source;
		int flags;
	} *route; /* pad level info, indexed by stream */

	struct regmap *regmap8;
	struct mutex max_mutex;
	int (*create_link)(
		struct ici_isys_node *src,
		u16 src_pad,
		struct ici_isys_node *sink,
		u16 sink_pad,
		u32 flags);
};

#define to_max_9286(_sd) container_of(_sd, struct max9286, ici_sd)
#define to_ici_ext_subdev(_node) container_of(_node, struct ici_ext_subdev, node)

/*
 * Order matters.
 *
 * 1. Bits-per-pixel, descending.
 * 2. Bits-per-pixel compressed, descending.
 * 3. Pixel order, same as in pixel_order_str. Formats for all four pixel
 *	  orders must be defined.
 */
static const struct max9286_csi_data_format max_csi_data_formats[] = {
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

static struct regmap_config max9286_reg_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
};

/* Serializer register write */
static int max96705_write_register(struct max9286 *max,
	unsigned int offset, u8 reg, u8 val)
{
	int ret;
	int retry, timeout = 10;
	struct i2c_client *client = max->ici_sd.client;

	client->addr = S_ADDR_MAX96705 + offset;
	for (retry = 0; retry < timeout; retry++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (val < 0)
			usleep_range(5000, 6000);
		else
			break;
	}

	client->addr = DS_ADDR_MAX9286;
	if (retry >= timeout) {
		pr_err("%s:write reg failed: reg=%2x\n", __func__, reg);
		return -EREMOTEIO;
	}

	return 0;
}

/* Serializer register read */
static int
max96705_read_register(struct max9286 *max, unsigned int i, u8 reg)
{
	int val;
	int retry, timeout = 10;
	struct i2c_client *client = max->ici_sd.client;

	client->addr = S_ADDR_MAX96705 + i;
	for (retry = 0; retry < timeout; retry++) {
		val = i2c_smbus_read_byte_data(client, reg);
		if (val >= 0)
			break;
		usleep_range(5000, 6000);
	}

	client->addr = DS_ADDR_MAX9286;
	if (retry >= timeout) {
		pr_err("%s:read reg failed: reg=%2x\n", __func__, reg);
		return -EREMOTEIO;
	}

	return val;
}

/* Validate csi_data_format */
static const struct max9286_csi_data_format *
max9286_validate_csi_data_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(max_csi_data_formats); i++) {
		if (max_csi_data_formats[i].code == code)
			return &max_csi_data_formats[i];
	}

	return &max_csi_data_formats[0];
}

/* Initialize image sensors and set stream on registers */
static int max9286_set_stream(
		struct ici_isys_node *node,
		void *ip,
		int enable)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct max9286 *max = to_max_9286(subdev);

	int i, rval, j;
	unsigned int val;
	u8 slval = 0xE0;
	u8 dtval = 0xF7;
	const struct max9286_register_write *max9286_byte_order_settings;

	pr_info("MAX9286 set stream. enable = %d\n", enable);
	/* Disable I2C ACK */
	rval = regmap_write(max->regmap8, DS_I2CLOCACK, 0xB6);
	if (rval) {
		pr_err("Failed to disable I2C ACK!\n");
		return rval;
	}
	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		if (((0x01 << (i)) &  max->sensor_present) == 0)
			continue;

		if (strncmp(node->name, max->sub_devs[i].sd_name, ICI_MAX_NODE_NAME))
			continue;

		if (enable) {
			/*
			 * Enable CSI-2 lanes D0, D1, D2, D3
			 * Enable CSI-2 DBL (Double Input Mode)
			 * Enable GMSL DBL for RAWx2
			 * Enable RAW10/RAW12 data type
			 */
			u8 bpp;
			const struct max9286_csi_data_format *csi_format =
				max9286_validate_csi_data_format(max->ffmts[i]->pixelformat);

			bpp = csi_format->compressed;

			if (bpp == 10) {
				dtval = 0xF6;
				max9286_byte_order_settings =
					&max9286_byte_order_settings_10bit[0];
			} else if (bpp == 12) {
				dtval = 0xF7;
				max9286_byte_order_settings =
					&max9286_byte_order_settings_12bit[0];
			} else {
				pr_err("Only support RAW10/12, current bpp is %d!\n", bpp);
				return -EINVAL;
			}

			rval = regmap_write(max->regmap8, DS_CSI_DBL_DT, dtval);
			if (rval) {
				pr_err("Failed to set data type!\n");
				return rval;
			}

			for (j = 0; j < bpp * 2; j++) {
				rval = max96705_write_register(max,
					S_ADDR_MAX96705_BROADCAST - S_ADDR_MAX96705,
					(max9286_byte_order_settings + j)->reg,
					(max9286_byte_order_settings + j)->val);
				if (rval) {
					pr_err("Failed to set max9286 byte order\n");
					return rval;
				}
			}
			usleep_range(2000, 3000);
		}

		/* Enable link */
		slval |= (0x0F & (1 << i));
		rval = regmap_write(max->regmap8, DS_LINK_ENABLE, slval);
		if (rval) {
			pr_err("Failed to enable GMSL links!\n");
			return rval;
		}

		rval = regmap_write(max->regmap8, DS_ATUO_MASK_LINK, 0x30);
		if (rval) {
			pr_err("Failed to write 0x69\n");
			return rval;
		}
	}
#if 0
	/* Enable I2C ACK */
	rval = regmap_write(max->regmap8, DS_I2CLOCACK, 0x36);
	if (rval) {
		pr_err("Failed to enable I2C ACK!\n");
		return rval;
	}
#endif
	/* Check if valid PCLK is available for the links */
	for (i = 1; i <= NR_OF_MAX_SINK_PADS; i++) {
		if (((0x01 << (i - 1)) &  max->sensor_present) == 0)
			continue;

		val = max96705_read_register(max, i, S_INPUT_STATUS);
		if ((val != -EREMOTEIO) && (val & 0x01))
			pr_info("Valid PCLK detected for link %d\n", i);
		else if (val != -EREMOTEIO)
			pr_info("Failed to read PCLK reg for link %d\n", i);
	}

	/* Set preemphasis settings for all serializers (set to 3.3dB)*/
	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_CMLLVL_PREEMP, 0xAA);
	usleep_range(5000, 6000);

	/* Set VSYNC Delay */
	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_SYNC_GEN_CONFIG, 0x21);
	usleep_range(5000, 6000);

	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_VS_DLY_2, 0x06);
	usleep_range(5000, 6000);

	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_VS_DLY_1, 0xD8);
	usleep_range(5000, 6000);

	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_VS_H_2, 0x26);
	usleep_range(5000, 6000);

	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_VS_H_1, 0x00);
	usleep_range(5000, 6000);

	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_VS_H_0, 0x00);
	usleep_range(5000, 6000);

	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_DBL_ALIGN_TO, 0xC4);
	usleep_range(5000, 6000);

	/* Enable link equalizers */
	rval = regmap_write(max->regmap8, DS_ENEQ, 0x0F);
	if (rval) {
		pr_err("Failed to automatically detect serial data rate!\n");
		return rval;
	}
	usleep_range(5000, 6000);
	rval = regmap_write(max->regmap8, DS_HS_VS, 0x91);

	/* Enable serial links and desable configuration */
	max96705_write_register(max, S_ADDR_MAX96705_BROADCAST -
		S_ADDR_MAX96705, S_MAIN_CTL, 0x83);
	/* Wait for more than 2 Frames time from each sensor */
	usleep_range(100000, 101000);

	/*
	 * Poll frame synchronization bit of deserializer
	 * All the cameras should work in SYNC mode
	 * MAX9286 sends a pulse to each camera, then each camera sends out
	 * one frame. The VSYNC for each camera should appear in almost same
	 * time for the deserializer to lock FSYNC
	 */
	rval = regmap_read(max->regmap8, DS_FSYNC_LOCKED, &val);
	if (rval) {
		pr_info("Frame SYNC not locked!\n");
		return rval;
	} else if (val & (0x01 << 6))
		pr_info("Deserializer Frame SYNC locked\n");

	/*
	 * Enable/set bit[7] of DS_CSI_VC_CTL register for VC operation
	 * Set VC according to the link number
	 * Enable CSI-2 output
	 */
	if (!enable) {
		rval = regmap_write(max->regmap8, DS_CSI_VC_CTL, 0x93);
		if (rval) {
			pr_err("Failed to disable CSI output!\n");
			return rval;
		}
	} else {
		rval = regmap_write(max->regmap8, DS_CSI_VC_CTL, 0x9B);
		if (rval) {
			pr_err("Failed to enable CSI output!\n");
			return rval;
		}
	}

	return 0;
}

/* callback for VIDIOC_SUBDEV_G_FMT ioctl handler code */
static int max9286_get_format(struct ici_isys_node *node,
	struct ici_pad_framefmt *fmt)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct max9286 *max = to_max_9286(subdev);
	int i;


	mutex_lock(&max->max_mutex);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {

		if (strncmp(node->name, max->sub_devs[i].sd_name, ICI_MAX_NODE_NAME))
				continue;


		fmt->ffmt.width = max->ffmts[i]->width;
		fmt->ffmt.height = max->ffmts[i]->height;
		fmt->ffmt.pixelformat = max->ffmts[i]->pixelformat;
		fmt->ffmt.field = max->ffmts[i]->field;
		fmt->ffmt.colorspace = max->ffmts[i]->colorspace;
		fmt->ffmt.flags = max->ffmts[i]->flags;

		mutex_unlock(&max->max_mutex);

		pr_info("framefmt: width: %d, height: %d, code: 0x%x.\n",
			fmt->ffmt.width, fmt->ffmt.height, fmt->ffmt.pixelformat);

		return 0;
	}

	mutex_unlock(&max->max_mutex);

	pr_err("max9286_get_format: unknown node name \n");

	return -1;
}

/* Enumerate media bus formats available at a given sub-device pad */
static int max9286_enum_mbus_code(struct ici_isys_node *node,
	struct ici_pad_supported_format_desc *psfd)
{
//		  struct ici_ext_subdev *subsubdev = node->sd;
//		  struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);

	pr_err(" TBD !\n");

	return 0;
}

static int max9286_get_param(struct ici_ext_sd_param *param)
{

	if (param->id == ICI_EXT_SD_PARAM_ID_LINK_FREQ) {
		param->val = 87750000;
	}

	return 0;
}

static int max9286_set_param(struct ici_ext_sd_param *param)
{
	return 0;
}

static int max9286_get_menu_item(struct ici_ext_sd_param *param, u32 idx)
{
	return 0;
}

static int max9286_set_power(struct ici_isys_node *node, int on)
{
	return 0;
}

/* callback for VIDIOC_SUBDEV_S_FMT ioctl handler code */
static int max9286_set_format(struct ici_isys_node *node,
	struct ici_pad_framefmt *fmt)
{
	struct ici_ext_subdev *subsubdev = node->sd;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(subsubdev->client);
	struct max9286 *max = to_max_9286(subdev);
	const struct max9286_csi_data_format *csi_format;
	int i;

	csi_format = max9286_validate_csi_data_format(fmt->ffmt.colorspace);

	mutex_lock(&max->max_mutex);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {

		if (strncmp(node->name, max->sub_devs[i].sd_name, ICI_MAX_NODE_NAME))
				continue;


		max->ffmts[i]->width = fmt->ffmt.width;
		max->ffmts[i]->height = fmt->ffmt.height;
		max->ffmts[i]->pixelformat = fmt->ffmt.pixelformat;
		max->ffmts[i]->field = fmt->ffmt.field;
		max->ffmts[i]->colorspace = fmt->ffmt.colorspace;
		max->ffmts[i]->flags = fmt->ffmt.flags;

		mutex_unlock(&max->max_mutex);

		pr_info("framefmt: width: %d, height: %d, code: 0x%x.\n",
				fmt->ffmt.width, fmt->ffmt.height, fmt->ffmt.pixelformat);

		return 0;
	}

	mutex_unlock(&max->max_mutex);

	pr_err("max9286_set_format: unknown node name\n");

	return 0;
}

static int max9286_set_selection(struct ici_isys_node *node, struct ici_pad_selection *ps)
{
// TODO place holder
	pr_err(" TBD!!! \n");
	return 0;
}

static int max9286_get_selection(struct ici_isys_node *node, struct ici_pad_selection *ps)
{
// TODO place holder
	pr_err(" TBD!!! \n");
	return 0;
}

static int init_ext_sd(struct i2c_client *client, struct max9286_subdev *max_sd, int idx)
{
	int rval;
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);;
	struct max9286 *max = to_max_9286(subdev);
	char name[ICI_MAX_NODE_NAME];

	snprintf(name, sizeof(name), "MAX9286 %d", idx);

	strncpy(max->sub_devs[idx].sd_name, name, sizeof(name));

	max_sd->sd->client = client;
	max_sd->sd->num_pads = 2;
	max_sd->sd->pads[0].pad_id = 0;
	max_sd->sd->pads[0].flags = ICI_PAD_FLAGS_SINK;
	max_sd->sd->pads[1].pad_id = 1;
	max_sd->sd->pads[1].flags = ICI_PAD_FLAGS_SOURCE;
// TODO
//		  sd->src_pad = ssd->source_pad;
// below fnctions invoked by csi2 fe code
	max_sd->sd->set_param = max9286_set_param; // meant to execute CTRL-IDs/CIDs
	max_sd->sd->get_param = max9286_get_param; // meant to execute CTRLIDs/CIDs
	max_sd->sd->get_menu_item = max9286_get_menu_item; // get LINK FREQ
	if (max->reg.setup_node) {
			rval = max->reg.setup_node(max->reg.ipu_data,
					max_sd->sd, name);
			if (rval)
					return rval;
	} else {
		pr_err("node not registered\n");
	}

// below invoked by stream code
	max_sd->sd->node.node_set_power = max9286_set_power;
	max_sd->sd->node.node_set_streaming = max9286_set_stream;
// below invoked by pipeline-device code
	max_sd->sd->node.node_get_pad_supported_format =
			max9286_enum_mbus_code; //needs modification
	max_sd->sd->node.node_set_pad_ffmt = max9286_set_format;
	max_sd->sd->node.node_get_pad_ffmt = max9286_get_format;
	max_sd->sd->node.node_set_pad_sel = max9286_set_selection;
	max_sd->sd->node.node_get_pad_sel = max9286_get_selection;


	return 0;
}

static int create_link(struct ici_isys_node *src_node,
	u16 srcpad,
	struct ici_isys_node *sink_node,
	u16 sinkpad,
	u32 flag)
{
	struct ici_ext_subdev *sd, *ssd;
	struct max9286 *max;
	struct max9286_subdev *subdev;
	int i, ret;
	if (!src_node || !sink_node)
		return -EINVAL;

	sd = to_ici_ext_subdev(src_node);
	if (!sd)
		return -EINVAL;

	max = to_max_9286(sd);
	if (!max)
		return -EINVAL;

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		subdev = &max->sub_devs[i];
		if (!subdev)
			continue;
		ssd = subdev->sd;
		ret = max->create_link(&ssd->node,
			1,
			sink_node,
			sinkpad,
			0);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * called when this subdev is registered.
 */
static int max9286_registered(struct ici_ext_subdev_register *reg)
{
	struct ici_ext_subdev *subdev = reg->sd;
	struct i2c_client *client = subdev->client;
	struct ici_isys *isys = reg->ipu_data;
	struct max9286 *max = to_max_9286(subdev);
	struct max9286_subdev *sd, *prev_sd = NULL;
	int i, k, rval, num, nsinks;


	num = max->pdata->subdev_num;
	nsinks = max->nsinks;

	max->reg = *reg;
	max->create_link = reg->create_link;

	subdev->get_param = max9286_get_param;
	subdev->set_param = max9286_set_param;
	subdev->get_menu_item = max9286_get_menu_item;

	for (i = 0, k = 0; (i < num) && (k < nsinks); i++, k++) {
		struct max9286_subdev_i2c_info *info =
			&max->pdata->subdev_info[i];
		struct crlmodule_lite_platform_data *pdata =
			(struct crlmodule_lite_platform_data *)
			info->board_info.platform_data;

		if (i >= nsinks)
			break;

		/* Spin the sensor subdev name suffix */
//		pdata->suffix = info->suffix;

		memcpy(&max->subdev_pdata[i], pdata, sizeof(*pdata));

		max->subdev_pdata[i].suffix = info->suffix;
		info->board_info.platform_data = &max->subdev_pdata[i];

		struct i2c_client *client2;
		struct ici_ext_subdev *sensor_sd;
		struct ici_ext_subdev_register sd_register = {0};

		request_module(I2C_MODULE_PREFIX "%s", info->board_info.type);

		client2 = i2c_new_device(client->adapter, &info->board_info);

		if (client2 == NULL || client2->dev.driver == NULL) {
			pr_err("@%s, No new i2c device\n", __func__);
			continue;
		}

		/* Get the clientdata set by the sensor driver */
		sensor_sd = i2c_get_clientdata(client2);
		if (!sensor_sd)
			pr_err("@%s, Failed to get client data\n", __func__);

		sd_register.ipu_data = isys;
		sd_register.sd = sensor_sd;
		sd_register.setup_node = reg->setup_node;
		sd_register.create_link = reg->create_link;
		rval = sensor_sd->do_register(&sd_register);
		if (rval) {
			pr_err("@%s, Failed to register external subdev\n", __func__);
					continue;
		}



		max->sub_devs[k].sd = devm_kzalloc(&client->dev, sizeof(struct ici_ext_subdev), GFP_KERNEL);
		if (!max->sub_devs[k].sd) {
			pr_err("can't create MAX9286 subdev %d\n", i);
			continue;
		}
//		max->sub_devs[k].rx_port = info->rx_port;
//		max->sub_devs[k].phy_i2c_addr = info->phy_i2c_addr;
		max->sub_devs[k].alias_i2c_addr = info->board_info.addr;

		sd = &max->sub_devs[k];
		rval = init_ext_sd(max->ici_sd.client, sd, k);
		if (rval)
				return rval;

		rval = sd_register.create_link(&sensor_sd->node,
			sensor_sd->src_pad,
			&sd->sd->node, 0, 0);
		if (rval) {
			pr_err("@%s, error creating link\n", __func__);
			return rval;
		}

				prev_sd = sd;
	}

	/* Replace existing create_link address with MAX9286 create_link implementation
	   to create link between MAX9286 node and CSI2 node */
	reg->create_link = create_link;

	return 0;
}

static void max9286_unregistered(struct ici_ext_subdev *subdev)
{
		pr_debug("%s DO NOTHING ?? \n", __func__);
}

static const s64 max9286_op_sys_clock[] = { 87750000, };
/* Registers MAX9286 sub-devices (Image sensors) */
static int max9286_register_subdev(struct max9286 *max, struct i2c_client *client)
{
	int i;

	max->ici_sd.client = client;
	max->ici_sd.do_register = max9286_registered;
	max->ici_sd.do_unregister = max9286_unregistered;

	i2c_set_clientdata(client, &max->ici_sd);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		max->ffmts[i]->width = 1920;
		max->ffmts[i]->height = 1088;
		max->ffmts[i]->pixelformat = ICI_FORMAT_SGRBG10;
		max->ffmts[i]->field = ICI_FIELD_NONE;
		snprintf(max->sub_devs[i].sd_name, sizeof(max->sub_devs[i].sd_name),
			"MAX9286 %d", i);
	}
	return 0;
}

/*
 * Get the output link order
 * By default:
 * bits[7:6] 11: Link 3 is 4th in the CSI-2 output order
 * bits[5:4] 10: Link 2 is 3rd in the CSI-2 output order
 * bits[3:2] 01: Link 1 is 2nd in the CSI-2 output order
 * bits[1:0] 00: Link 0 is 1st in the CSI-2 output order
 */
static u8 get_output_link_order(struct max9286 *max)
{
	u8 val = 0xE4, i;
	u8 order_config[14][3] = {
		{1, 8, 0x27},
		{1, 4, 0xC6},
		{1, 2, 0xE1},
		{1, 1, 0xE4},
		{2, 0xC, 0x4E},
		{2, 0xA, 0x72},
		{2, 0x9, 0x78},
		{2, 0x6, 0xD2},
		{2, 0x5, 0xD8},
		{2, 0x3, 0xE4},
		{3, 0xE, 0x93},
		{3, 0xD, 0x9C},
		{3, 0xB, 0xB4},
		{3, 0x7, 0xE4},
	};

	if (max->total_sensor_num < 4) {
		for (i = 0; i < 14; i++) {
			if ((max->total_sensor_num == order_config[i][0])
				&& (max->sensor_present == order_config[i][1]))
				return	order_config[i][2];
		}
	}

	/* sensor_num = 4 will return 0xE4 */
	return val;
}

/* MAX9286 initial setup and Reverse channel setup */
static int max9286_init(struct max9286 *max, struct i2c_client *client)
{
	int i, rval;
	unsigned int val, lval;
	u8 mval, slval, tmval;

	usleep_range(10000, 11000);

	rval = regmap_read(max->regmap8, DS_MAX9286_DEVID, &val);
	if (rval) {
		pr_err("Failed to read device ID of MAX9286!\n");
		return rval;
	}
	pr_info("MAX9286 device ID: 0x%X\n", val);

	rval = regmap_write(max->regmap8, DS_CSI_VC_CTL, 0x93);
	if (rval) {
		pr_err("Failed to disable CSI output!\n");
		return rval;
	}
	/* All the links are working in Legacy reverse control-channel mode */
	/* Enable Custom Reverse Channel and First Pulse Length */
	rval = regmap_write(max->regmap8, DS_ENCRC_FPL, 0x4F);
	if (rval) {
		pr_err("Failed to disable PRBS test!\n");
		return rval;
	}
	/*
	 * 2ms of delay is required after any analog change to reverse control
	 * channel for bus timeout and I2C state machine to settle from any
	 * glitches
	 */
	usleep_range(2000, 3000);
	/* First pulse length rise time changed from 300ns to 200ns */
	rval = regmap_write(max->regmap8, DS_FPL_RT, 0x1E);
	if (rval) {
		pr_err("Failed to disable PRBS test!\n");
		return rval;
	}
	usleep_range(2000, 3000);

	/* Enable configuration links */
	max96705_write_register(max, 0, S_MAIN_CTL, 0x43);
	usleep_range(5000, 6000);

	/*
	 * Enable high threshold for reverse channel input buffer
	 * This increases immunity to power supply noise when the
	 * coaxial link is used for power as well as signal
	 */
	max96705_write_register(max, 0, S_RSVD_8, 0x01);
	/* Enable change of reverse control parameters */

	max96705_write_register(max, 0, S_RSVD_97, 0x5F);

	/* Wait 2ms after any change to reverse control channel */
	usleep_range(2000, 3000);

	/* Increase reverse amplitude from 100mV to 170mV to compensate for
	 * higher threshold
	 */
	rval = regmap_write(max->regmap8, DS_FPL_RT, 0x19);
	if (rval) {
		pr_err("Failed to disable PRBS test!\n");
		return rval;
	}
	usleep_range(2000, 3000);

	/*
	 * Enable CSI-2 lanes D0, D1, D2, D3
	 * Enable CSI-2 DBL (Double Input Mode)
	 * Enable GMSL DBL for RAWx2
	 * Enable RAW12 data type by default
	 */
	rval = regmap_write(max->regmap8, DS_CSI_DBL_DT, 0xF7); //RAW12
	if (rval) {
		pr_err("Failed to set data type!\n");
		return rval;
	}
	usleep_range(2000, 3000);

	/* Enable Frame sync Auto-mode for row/column reset on frame sync
	 * sensors
	 */
	rval = regmap_write(max->regmap8, DS_FSYNCMODE, 0x00);
	if (rval) {
		pr_err("Failed to set frame sync mode!\n");
		return rval;
	}
	usleep_range(2000, 3000);
	rval = regmap_write(max->regmap8, DS_OVERLAP_WIN_LOW, 0x00);
	rval = regmap_write(max->regmap8, DS_OVERLAP_WIN_HIGH, 0x00);

	rval = regmap_write(max->regmap8, DS_FSYNC_PERIOD_LOW, 0x55);
	rval = regmap_write(max->regmap8, DS_FSYNC_PERIOD_MIDDLE, 0xc2);
	rval = regmap_write(max->regmap8, DS_FSYNC_PERIOD_HIGH, 0x2C);

	rval = regmap_write(max->regmap8, DS_HIGHIMM, 0x06);

	/*
	 * Enable DBL
	 * Edge select: Rising Edge
	 * Enable HS/VS encoding
	 */
	max96705_write_register(max, 0, S_CONFIG, 0xD4);
	usleep_range(2000, 3000);

	for (i = 0; i < ARRAY_SIZE(max9286_byte_order_settings_12bit); i++) {
		rval = max96705_write_register(max, 0,
				max9286_byte_order_settings_12bit[i].reg,
				max9286_byte_order_settings_12bit[i].val);
		if (rval) {
			pr_err("Failed to set max9286 byte order\n");
			return rval;
		}
	}

	/* Detect video links */
	rval = regmap_read(max->regmap8, DS_CONFIGL_VIDEOL_DET, &lval);
	if (rval) {
		pr_err("Failed to read register 0x49!\n");
		return rval;
	}

	/*
	 * Check on which links the sensors are connected
	 * And also check total number of sensors connected to the deserializer
	 */
	max->sensor_present = ((lval >> 4) & 0xF) | (lval & 0xF);

	for (i = 0; i < NR_OF_MAX_STREAMS; i++) {
		if (max->sensor_present & (0x1 << i)) {
			pr_info("Sensor present on deserializer link %d\n", i);
			max->total_sensor_num += 1;
		}
	}

	pr_info("total sensor present = %d", max->total_sensor_num);
	pr_info("sensor present on links = %d", max->sensor_present);

	if (!max->total_sensor_num) {
		pr_err("No sensors connected!\n");
	} else {
		pr_info("Total number of sensors connected = %d\n",
			max->total_sensor_num);
	}

	slval = get_output_link_order(max);

	/* Set link output order */
	rval = regmap_write(max->regmap8, DS_LINK_OUTORD, slval);
	if (rval) {
		pr_err("Failed to set Link output order!\n");
		return rval;
	}

	slval = 0xE0 | max->sensor_present;

	mval = 0;
	tmval = 0;
	/*
	 * Setup each serializer individually and their respective I2C slave
	 * address changed to a unique value by enabling one reverse channel
	 * at a time via deserializer's DS_FWDCCEN_REVCCEN control register.
	 * Also create broadcast slave address for MAX96705 serializer.
	 * After this stage, i2cdetect on I2C-ADAPTER should display the
	 * below devices
	 * 10: Sensor address
	 * 11, 12, 13, 14: Sensors alias addresses
	 * 41, 42, 43, 44: Serializers alias addresses
	 * 45: Serializer's broadcast address
	 * 48: Deserializer's address
	 */

	for (i = 1; i <= NR_OF_MAX_SINK_PADS; i++) {
		/* Setup the link when the sensor is connected to the link */
		if (((0x1 << (i - 1)) &  max->sensor_present) == 0)
			continue;

		/* Enable only one reverse channel at a time */
		mval = (0x11 << (i - 1));
		tmval |= (0x11 << (i - 1));
		rval = regmap_write(max->regmap8, DS_FWDCCEN_REVCCEN, mval);
		if (rval) {
			pr_err("Failed to enable channel for %d!\n", i);
			return rval;
		}
		/* Wait 2ms after enabling reverse channel */
		usleep_range(2000, 3000);

		/* Change Serializer slave address */
		max96705_write_register(max, 0, S_SERADDR,
			(S_ADDR_MAX96705 + i) << 1);
		/* Unique link 'i' image sensor slave address */
		max96705_write_register(max, i, S_I2C_SOURCE_IS,
			(ADDR_AR0231AT_SENSOR + i) << 1);
		/* Link 'i' image sensor slave address */
		max96705_write_register(max, i, S_I2C_DST_IS,
			ADDR_AR0231AT_SENSOR << 1);
		/* Serializer broadcast address */
		max96705_write_register(max, i, S_I2C_SOURCE_SER,
			S_ADDR_MAX96705_BROADCAST << 1);
		/* Link 'i' serializer address */
		max96705_write_register(max, i, S_I2C_DST_SER,
			(S_ADDR_MAX96705 + i) << 1);
	}

	/* Enable I2c reverse channels */
	rval = regmap_write(max->regmap8, DS_FWDCCEN_REVCCEN, tmval);
	if (rval) {
		pr_err("Failed to enable channel for %d!\n", i);
		return rval;
	}
	usleep_range(2000, 3000);

	return 0;
}

/* Unbind the MAX9286 device driver from the I2C client */
static int max9286_remove(struct i2c_client *client)
{
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);
	struct max9286 *max = to_max_9286(subdev);
	int i;

	mutex_destroy(&max->max_mutex);

	for (i = 0; i < NR_OF_MAX_SINK_PADS; i++) {
		max->sub_devs[i].sd = NULL;
	}

	return 0;
}

/* Called by I2C probe */
static int max9286_probe(struct i2c_client *client,
	const struct i2c_device_id *devid)
{
	struct max9286 *max;
	int i = 0;
	int rval = 0;

	if (client->dev.platform_data == NULL)
		return -ENODEV;

	dev_err(&client->dev, "MAX9286 probe!\n");
	max = devm_kzalloc(&client->dev, sizeof(*max), GFP_KERNEL);
	if (!max)
		return -ENOMEM;

	max->pdata = client->dev.platform_data;

	max->nsources = NR_OF_MAX_SOURCE_PADS;
	max->nsinks = NR_OF_MAX_SINK_PADS;
	max->npads = NR_OF_MAX_PADS;
	max->nstreams = NR_OF_MAX_STREAMS;

	max->crop = devm_kcalloc(&client->dev, max->npads,
		sizeof(struct ici_rect), GFP_KERNEL);
	max->compose = devm_kcalloc(&client->dev, max->npads,
		sizeof(struct ici_rect), GFP_KERNEL);
	max->route = devm_kcalloc(&client->dev, max->nstreams,
		sizeof(*max->route), GFP_KERNEL);
	max->stream = devm_kcalloc(&client->dev, max->npads,
		sizeof(*max->stream), GFP_KERNEL);

	if (!max->crop || !max->compose || !max->route || !max->stream)
		return -ENOMEM;

	for (i = 0; i < max->npads; i++) {
		max->ffmts[i] =
			devm_kcalloc(&client->dev, max->nstreams,
				sizeof(struct ici_framefmt), GFP_KERNEL);
		if (!max->ffmts[i])
			return -ENOMEM;

		max->stream[i].stream_id =
			devm_kcalloc(&client->dev, max->nsinks,
				sizeof(int), GFP_KERNEL);
		if (!max->stream[i].stream_id)
			return -ENOMEM;
	}

	for (i = 0; i < max->nstreams; i++) {
		max->route[i].sink = i;
		max->route[i].source = MAX_PAD_SOURCE;
		max->route[i].flags = 0;
	}

	for (i = 0; i < max->nsinks; i++) {
		max->stream[i].stream_id[0] = i;
		max->stream[MAX_PAD_SOURCE].stream_id[i] = i;
	}

	max->regmap8 = devm_regmap_init_i2c(client, &max9286_reg_config8);
	if (IS_ERR(max->regmap8)) {
		dev_err(&client->dev, "Failed to init regmap8!\n");
		return -EIO;
	}

	mutex_init(&max->max_mutex);

	rval = max9286_register_subdev(max, client);
	if (rval) {
		dev_err(&client->dev,
			"Failed to register MAX9286 subdevice!\n");
		goto error_mutex_destroy;
	}

	rval = max9286_init(max, client);
	if (rval) {
		dev_err(&client->dev, "Failed to initialise MAX9286!\n");
		goto error_media_entity;
	}

	return 0;

error_media_entity:
error_mutex_destroy:
	mutex_destroy(&max->max_mutex);

	return rval;
}

#ifdef CONFIG_PM
static int max9286_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);
	struct max9286 *max = to_max_9286(subdev);

	return max9286_init(max, client);
}
#else
#define max9286_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id max9286_id_table[] = {
	{ MAX9286_NAME, 0 },
	{},
};

static const struct dev_pm_ops max9286_pm_ops = {
	.resume = max9286_resume,
};

static struct i2c_driver max9286_i2c_driver = {
	.driver = {
		.name = MAX9286_NAME,
		.pm = &max9286_pm_ops,
	},
	.probe = max9286_probe,
	.remove = max9286_remove,
	.id_table = max9286_id_table,
};

module_i2c_driver(max9286_i2c_driver);

MODULE_AUTHOR("Karthik Gopalakrishnan <karthik.l.gopalakrishnan@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Maxim96705 serializer and Maxim9286 deserializer driver");
