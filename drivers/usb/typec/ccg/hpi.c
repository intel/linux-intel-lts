// SPDX-License-Identifier: GPL-2.0
/*
 * Cypress CCG Host Processor Interface driver
 *
 * Copyright (C) 2019 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/usb/typec.h>

/* Response Codes */
#define HPI_NO_RESPONSE			0x00
#define HPI_SUCCESS			0x02

#define HPI_RESPONSE_IS_EVENT(r)	((r) & BIT(7))

/* Event Codes */
#define HPI_RESET_COMPLETE		0x80
#define HPI_CONNECT			0x84
#define HPI_DISCONNECT			0x85
#define HPI_PD_CONTRACT_COMPLETE	0x86

/* Common registers */
#define HPI_DEVICE_MODE			0x00
#define HPI_SILICON_ID			0x02
#define HPI_INTR_REG			0x06
#define HPI_RESET			0x08
#define HPI_READ_ALL_VERSION		0x10
#define HPI_POWER_STATUS		0x2e
#define HPI_RESPONSE			0x7e
/* HPIv2 only */
#define HPI_PDPORT_ENABLE		0x2c
#define HPI_VERSION			0x3c

/*
 * The following registers are port specific registers. With HPIv1 there is only
 * one for each, as HPIv1 only supports a single port, but with HPIv2 there is
 * separate register for every port the CCG has. HPI_PORT_REG() is helper for
 * accessing them.
 */

#define HPI_PORT_REG(_num, _reg)					\
		((_num) ? ((_num) << 12) | HPIv2_##_reg : HPIv1_##_reg)

/* HPIv1 port registers */
#define HPIv1_PD_CONTROL		0x28
#define HPIv1_PD_STATUS			0x2c
#define HPIv1_TYPEC_STATUS		0x30
#define HPIv1_EVENT_MASK		0x48
#define HPIv1_PD_RESPONSE		HPI_RESPONSE

/* HPIv2 port registers without the port mask */
#define HPIv2_PD_CONTROL		0x06
#define HPIv2_PD_STATUS			0x08
#define HPIv2_TYPEC_STATUS		0x0c
#define HPIv2_EVENT_MASK		0x24
#define HPIv2_PD_RESPONSE		0x400

/* Register bits / commands */

/* DEVICE_MODE */
#define HPI_DEVICE_MODE_VER(m)		(((m) & BIT(7)) >> 7)
#define HPI_DEVICE_MODE_NUM_PORTS(m)	((((m) & 0x0c) >> 2) + 1)

/* INTR_REG */
#define HPI_INTR_DEV			BIT(0)
#define HPI_INTR_PORT0			BIT(1)
#define HPI_INTR_PORT1			BIT(2)

/* PD_CONTROL */
#define HPI_PD_CONTROL_INIT_COMPLETE	0x10
#define HPI_PD_CONTROL_PORT_DISABLE	0x11

/* PD_STATUS */
#define HPI_PD_STATUS_DATA_ROLE_DEF(p)	((p) & 3)
#define HPI_PD_STATUS_PWR_ROLE_DEF(p)	(((p) & GENMASK(4, 3)) >> 3)
#define HPI_PD_STATUS_REVISION(p)	(((p) & BIT(6)) ? 0x300 : 0x200)
#define HPI_PD_STATUS_DATA_ROLE(p)	(!!((p) & BIT(6)))
#define HPI_PD_STATUS_PWR_ROLE(p)	(!!((p) & BIT(8)))
#define HPI_PD_STATUS_CONTRACT		BIT(10)
#define HPI_PD_STATUS_VCONN(p)		(!!((p) & BIT(12)))

/* TYPEC_STATUS */
#define HPI_TYPEC_STATUS_CONNECTED	BIT(0)
#define HPI_TYPEC_STATUS_TYPE(p)	(((p) & GENMASK(4, 2)) >> 2)
#define   HPI_PARTNER_NONE		0
#define   HPI_PARTNER_SINK		1
#define   HPI_PARTNER_SOURCE		2
#define   HPI_PARTNER_DEBUG		3
#define   HPI_PARTNER_AUDIO		4
#define   HPI_PARTNER_POWERED_ACC	5
#define   HPI_PARTNER_UNSUPPORTED_ACC	6
#define HPI_TYPEC_STATUS_PWRMODE(p)	(((p) & GENMASK(7, 6)) >> 6)

/* EVENT_MASK */
#define HPI_EVENT_OVER_CURRENT		BIT(1)
#define HPI_EVENT_OVER_VOLTAGE		BIT(2)
#define HPI_EVENT_CONNECT		BIT(3)
#define HPI_EVENT_DISCONNECT		BIT(4)
#define HPI_EVENT_PD_CONTRACT		BIT(5)

#define HPI_EVENTS (HPI_EVENT_CONNECT | HPI_EVENT_DISCONNECT |		\
		    HPI_EVENT_PD_CONTRACT)

enum hpi_protocol {
	HPIv1,
	HPIv2
};

enum {
	HPI_DATA_ROLE_UFP,
	HPI_DATA_ROLE_DFP,
	HPI_DATA_ROLE_DRP
};

enum {
	HPI_PWR_ROLE_SNK,
	HPI_PWR_ROLE_SRC,
	HPI_PWR_ROLE_DRP
};

#define HPI_PORT_MAX			2

#define port_to_hpi(_port) container_of(_port, struct ccg_hpi, \
					port[_port->num - 1])

struct ccg_hpi_port {
	int num;
	struct typec_port *port;
	struct typec_partner *partner;
	struct typec_capability cap;

	struct completion complete;

	/* Register cache */
	u32 pd_status;
	u8 typec_status;
	u8 response[4];
};

struct ccg_hpi {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock; /* device lock */

	struct ccg_hpi_port port[HPI_PORT_MAX];

	u8 device_mode;
};

static inline int hpi_read(struct ccg_hpi *hpi, u16 reg, u8 *val)
{
	return regmap_raw_read(hpi->regmap, reg, val, sizeof(u8));
}

static inline int hpi_read16(struct ccg_hpi *hpi, u16 reg, u16 *val)
{
	return regmap_raw_read(hpi->regmap, reg, val, sizeof(u16));
}

static inline int hpi_read32(struct ccg_hpi *hpi, u16 reg, u32 *val)
{
	return regmap_raw_read(hpi->regmap, reg, val, sizeof(u32));
}

static inline int
hpi_block_read(struct ccg_hpi *hpi, u16 reg, void *val, size_t len)
{
	return regmap_raw_read(hpi->regmap, reg, val, len);
}

static inline int hpi_write(struct ccg_hpi *hpi, u16 reg, u8 val)
{
	return regmap_raw_write(hpi->regmap, reg, (void *)&val, sizeof(u8));
}

static inline int hpi_write16(struct ccg_hpi *hpi, u16 reg, u16 val)
{
	return regmap_raw_write(hpi->regmap, reg, (void *)&val, sizeof(u16));
}

static inline int hpi_write32(struct ccg_hpi *hpi, u16 reg, u32 val)
{
	return regmap_raw_write(hpi->regmap, reg, (void *)&val, sizeof(u32));
}

static int ccg_hpi_reset(struct ccg_hpi *hpi, bool device_reset)
{
	u8 reset[2];

	reset[0] = 'R';
	reset[1] = device_reset;

	return regmap_raw_write(hpi->regmap, HPI_RESET, reset, sizeof(reset));
}

static int ccg_hpi_connect(struct ccg_hpi_port *p)
{
	struct typec_partner_desc desc;
	struct typec_partner *partner;

	if (p->partner)
		return 0;

	desc.usb_pd = !!(p->pd_status & HPI_PD_STATUS_CONTRACT);

	if (HPI_TYPEC_STATUS_TYPE(p->typec_status) == HPI_PARTNER_AUDIO)
		desc.accessory = TYPEC_ACCESSORY_AUDIO;
	else if (HPI_TYPEC_STATUS_TYPE(p->typec_status) == HPI_PARTNER_DEBUG)
		desc.accessory = TYPEC_ACCESSORY_DEBUG;
	else
		desc.accessory = TYPEC_ACCESSORY_NONE;

	desc.identity = NULL; /* FIXME */

	if (desc.usb_pd)
		typec_set_pwr_opmode(p->port, TYPEC_PWR_MODE_PD);
	else
		typec_set_pwr_opmode(p->port,
				     HPI_TYPEC_STATUS_PWRMODE(p->typec_status));

	typec_set_pwr_role(p->port, HPI_PD_STATUS_PWR_ROLE(p->pd_status));
	typec_set_vconn_role(p->port, HPI_PD_STATUS_VCONN(p->pd_status));
	typec_set_data_role(p->port, HPI_PD_STATUS_DATA_ROLE(p->pd_status));

	partner = typec_register_partner(p->port, &desc);
	if (IS_ERR(partner))
		return PTR_ERR(partner);

	p->partner = partner;

	return 0;
}

static void ccg_hpi_disconnect(struct ccg_hpi_port *p)
{
	typec_unregister_partner(p->partner);
	p->partner = NULL;

	typec_set_pwr_opmode(p->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(p->port, HPI_PD_STATUS_PWR_ROLE(p->pd_status));
	typec_set_vconn_role(p->port, HPI_PD_STATUS_VCONN(p->pd_status));
	typec_set_data_role(p->port, HPI_PD_STATUS_DATA_ROLE(p->pd_status));
}

static void ccg_hpi_response(struct ccg_hpi_port *port)
{
	struct ccg_hpi *hpi = port_to_hpi(port);
	int ret;

	if (!HPI_RESPONSE_IS_EVENT(port->response[0])) {
		complete(&port->complete);
		return;
	}

	ret = hpi_read32(hpi, HPI_PORT_REG(port->num, PD_STATUS),
			 &port->pd_status);
	if (ret < 0)
		return;

	ret = hpi_read(hpi, HPI_PORT_REG(port->num, TYPEC_STATUS),
		       &port->typec_status);
	if (ret < 0)
		return;

	/* Process events */
	switch (port->response[0]) {
	case HPI_RESET_COMPLETE:
		if (!completion_done(&port->complete))
			complete(&port->complete);
		break;
	case HPI_CONNECT:
		if (ccg_hpi_connect(port))
			dev_err(hpi->dev, "Failed to register partner\n");
		break;
	case HPI_DISCONNECT:
		ccg_hpi_disconnect(port);
		break;
	case HPI_PD_CONTRACT_COMPLETE:
		typec_set_pwr_opmode(port->port, TYPEC_PWR_MODE_PD);
		break;
	default:
		break;
	}
}

static irqreturn_t ccg_hpi_irq(int irq, void *data)
{
	struct ccg_hpi *hpi = data;
	struct ccg_hpi_port *port = &hpi->port[0];
	u8 intr;
	int ret;

	mutex_lock(&hpi->lock);

	ret = hpi_read(hpi, HPI_INTR_REG, &intr);
	if (ret < 0) {
		dev_err(hpi->dev, "failed to read interrupt status\n");
		goto err_unlock;
	}

	if (intr & HPI_INTR_DEV) {
		ret = hpi_read16(hpi, HPI_RESPONSE, (void *)port->response);
		if (ret < 0) {
			dev_err(hpi->dev, "failed to read response register\n");
			goto err_clear_intr;
		}
		ccg_hpi_response(port);
	}

	if (intr & HPI_INTR_PORT0) {
		ret = hpi_read32(hpi, HPI_PORT_REG(1, PD_RESPONSE),
				 (void *)port->response);
		if (ret < 0) {
			dev_err(hpi->dev, "failed to read response register\n");
			goto err_clear_intr;
		}
		ccg_hpi_response(port);
	}

	if (intr & HPI_INTR_PORT1) {
		port = &hpi->port[1];

		ret = hpi_read32(hpi, HPI_PORT_REG(2, PD_RESPONSE),
				 (void *)port->response);
		if (ret < 0) {
			dev_err(hpi->dev, "failed to read response register\n");
			goto err_clear_intr;
		}
		ccg_hpi_response(port);
	}

err_clear_intr:
	ret = hpi_write(hpi, HPI_INTR_REG, intr);
	if (ret < 0)
		dev_err(hpi->dev, "failed to clear interrupts\n");

err_unlock:
	mutex_unlock(&hpi->lock);

	return IRQ_HANDLED;
}

static int ccg_hpi_register_port(struct ccg_hpi *hpi, int index)
{
	struct ccg_hpi_port *port = &hpi->port[index];
	int ret;

	if (HPI_DEVICE_MODE_VER(hpi->device_mode) == HPIv2)
		port->num = index + 1;

	ret = hpi_read32(hpi, HPI_PORT_REG(port->num, PD_STATUS),
			 &port->pd_status);
	if (ret < 0)
		return ret;

	ret = hpi_read(hpi, HPI_PORT_REG(port->num, TYPEC_STATUS),
		       &port->typec_status);
	if (ret < 0)
		return ret;

	port->cap.revision = USB_TYPEC_REV_1_2;
	port->cap.pd_revision = HPI_PD_STATUS_REVISION(port->pd_status);
	port->cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;

	switch (HPI_PD_STATUS_PWR_ROLE_DEF(port->pd_status)) {
	case HPI_PWR_ROLE_SNK:
		port->cap.type = TYPEC_PORT_SNK;
		break;
	case HPI_PWR_ROLE_SRC:
		port->cap.type = TYPEC_PORT_SRC;
		break;
	case HPI_PWR_ROLE_DRP:
		port->cap.type = TYPEC_PORT_DRP;
		break;
	default:
		return -ENODEV;
	}

	switch (HPI_PD_STATUS_DATA_ROLE_DEF(port->pd_status)) {
	case HPI_DATA_ROLE_UFP:
		port->cap.data = TYPEC_PORT_UFP;
		break;
	case HPI_DATA_ROLE_DFP:
		port->cap.data = TYPEC_PORT_DFP;
		break;
	case HPI_DATA_ROLE_DRP:
		port->cap.data = TYPEC_PORT_DRD;
		break;
	default:
		return -ENODEV;
	}

	ret = hpi_write32(hpi, HPI_PORT_REG(port->num, EVENT_MASK), HPI_EVENTS);
	if (ret < 0) {
		dev_err(hpi->dev, "failed to enable port\n");
		return ret;
	}

	port->port = typec_register_port(hpi->dev, &port->cap);
	if (IS_ERR(port->port))
		return PTR_ERR(port->port);

	init_completion(&port->complete);

	if (port->typec_status & HPI_TYPEC_STATUS_CONNECTED) {
		ret = ccg_hpi_connect(port);
		if (ret)
			dev_err(hpi->dev, "failed to register partner\n");
	}

	return 0;
}

static struct regmap_config hpi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int ccg_hpi_probe(struct i2c_client *client)
{
	struct ccg_hpi *hpi;
	int ret;
	int i;

	ret = i2c_smbus_read_byte_data(client, HPI_DEVICE_MODE);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read DEVICE_MODE (%d)\n", ret);
		return ret;
	}

	hpi = devm_kzalloc(&client->dev, sizeof(*hpi), GFP_KERNEL);
	if (!hpi)
		return -ENOMEM;

	hpi->device_mode = ret;
	hpi->dev = &client->dev;
	mutex_init(&hpi->lock);

	if (HPI_DEVICE_MODE_VER(hpi->device_mode) == HPIv2)
		hpi_regmap_config.reg_bits = 16;

	hpi->regmap = devm_regmap_init_i2c(client, &hpi_regmap_config);
	if (IS_ERR(hpi->regmap))
		return PTR_ERR(hpi->regmap);

	for (i = 0; i < HPI_DEVICE_MODE_NUM_PORTS(hpi->device_mode); i++) {
		ret = ccg_hpi_register_port(hpi, i);
		if (ret)
			goto err_unregister_ports;
	}

	/* REVISIT: Silently clearing pending events - Probable not needed */
	ccg_hpi_reset(hpi, false);

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					ccg_hpi_irq, IRQF_ONESHOT,
					dev_name(&client->dev), hpi);
	if (ret)
		goto err_unregister_ports;

	i2c_set_clientdata(client, hpi);

	return 0;

err_unregister_ports:
	for (i = 0; i < HPI_PORT_MAX; i++)
		typec_unregister_port(hpi->port[i].port);

	return ret;
}

static int ccg_hpi_remove(struct i2c_client *client)
{
	struct ccg_hpi *hpi = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < HPI_DEVICE_MODE_NUM_PORTS(hpi->device_mode); i++) {
		hpi_write32(hpi, HPI_PORT_REG(hpi->port[i].num, EVENT_MASK), 0);
		typec_unregister_partner(hpi->port[i].partner);
		typec_unregister_port(hpi->port[i].port);
	}

	return 0;
}

static const struct i2c_device_id ccg_hpi_id[] = {
	{ "ccg-hpi" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ccg_hpi_id);

static const struct acpi_device_id ccg_hpi_acpi_match[] = {
	{ "INTC1030", },
	{ "INTC1031", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ccg_hpi_acpi_match);

static struct i2c_driver ccg_hpi_driver = {
	.driver = {
		.name = "ccg_hpi",
		.acpi_match_table = ACPI_PTR(ccg_hpi_acpi_match),
	},
	.probe_new = ccg_hpi_probe,
	.remove = ccg_hpi_remove,
	.id_table = ccg_hpi_id,
};
module_i2c_driver(ccg_hpi_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Cypress CCG Host Processor Interface driver");
