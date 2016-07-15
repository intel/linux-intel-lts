/**
 * typec_wcove.c - WhiskeyCove PMIC USB Type-C PHY driver
 *
 * Copyright (C) 2016 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 * Author: Chandra Sekhar Anagani <chandra.sekhar.anagani@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/usb/typec.h>
#include <linux/usb/pd_sink.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_soc_pmic.h>

/* Register offsets */
#define WCOVE_CHGRIRQ0		0x4e09
#define WCOVE_PHYCTRL		0x5e07

#define USBC_CONTROL1		0x7001
#define USBC_CONTROL2		0x7002
#define USBC_CONTROL3		0x7003
#define USBC_CC1_CTRL		0x7004
#define USBC_CC2_CTRL		0x7005
#define USBC_CC_SEL		0x7006
#define USBC_STATUS1		0x7007
#define USBC_STATUS2		0x7008
#define USBC_STATUS3		0x7009
#define USBC_IRQ1		0x7015
#define USBC_IRQ2		0x7016
#define USBC_IRQMASK1		0x7017
#define USBC_IRQMASK2		0x7018
#define USBC_PD_CFG1		0x7019
#define USBC_PD_CFG2		0x701a
#define USBC_PD_CFG3		0x701b
#define USBC_PD_STATUS		0x701c
#define USBC_RX_STATUS		0x701d
#define USBC_RX_INFO		0x701e
#define USBC_TX_CMD		0x701f
#define USBC_TX_INFO		0x7020
#define USBC_RX_DATA_START	0x7028
#define USBC_TX_DATA_START	0x7047
/* Register bits */

#define USBC_CONTROL1_MODE_DRP(r)	((r & ~0x7) | 4)

#define USBC_CONTROL2_UNATT_SNK		BIT(0)
#define USBC_CONTROL2_UNATT_SRC		BIT(1)
#define USBC_CONTROL2_DIS_ST		BIT(2)

#define USBC_CONTROL3_PD_DIS		BIT(1)

#define USBC_CC_CTRL_VCONN_EN		BIT(1)
#define USBC_CC_CTRL_TX_EN		BIT(2)

#define USBC_CC_SEL_CCSEL		(BIT(0) | BIT(1))
#define USBC_STATUS1_DET_ONGOING	BIT(6)
#define USBC_STATUS1_RSLT(r)		(r & 0xf)
#define USBC_RSLT_NOTHING		0
#define USBC_RSLT_SRC_DEFAULT		1
#define USBC_RSLT_SRC_1_5A		2
#define USBC_RSLT_SRC_3_0A		3
#define USBC_RSLT_SNK			4
#define USBC_RSLT_DEBUG_ACC		5
#define USBC_RSLT_AUDIO_ACC		6
#define USBC_RSLT_UNDEF			15
#define USBC_STATUS1_ORIENT(r)		((r >> 4) & 0x3)
#define USBC_ORIENT_NORMAL		1
#define USBC_ORIENT_REVERSE		2

#define USBC_STATUS2_VBUS_REQ		BIT(5)

#define USBC_IRQ1_ADCDONE1		BIT(2)
#define USBC_IRQ1_OVERTEMP		BIT(1)
#define USBC_IRQ1_SHORT			BIT(0)

#define USBC_IRQ2_CC_CHANGE		BIT(7)
#define USBC_IRQ2_RX_PD			BIT(6)
#define USBC_IRQ2_RX_HR			BIT(5)
#define USBC_IRQ2_RX_CR			BIT(4)
#define USBC_IRQ2_TX_SUCCESS		BIT(3)
#define USBC_IRQ2_TX_FAIL		BIT(2)

#define USBC_IRQMASK1_ALL	(USBC_IRQ1_ADCDONE1 | USBC_IRQ1_OVERTEMP | \
				 USBC_IRQ1_SHORT)

#define USBC_IRQMASK2_ALL	(USBC_IRQ2_CC_CHANGE | USBC_IRQ2_RX_PD | \
				 USBC_IRQ2_RX_HR | USBC_IRQ2_RX_CR | \
				 USBC_IRQ2_TX_SUCCESS | USBC_IRQ2_TX_FAIL)

#define USBC_PD_CFG1_ID_FILL		BIT(7)

#define USBC_PD_CFG2_SOP_RX		BIT(0)

#define USBC_PD_CFG3_SR_SOP2		(BIT(7) | BIT(6))
#define USBC_PD_CFG3_SR_SOP1		(BIT(5) | BIT(4))
#define USBC_PD_CFG3_SR_SOP0		(BIT(3) | BIT(2))
#define USBC_PD_CFG3_DATAROLE		BIT(1)
#define USBC_PD_CFG3_PWRROLE		BIT(0)

#define USBC_TX_CMD_TXBUF_RDY		BIT(0)
#define USBC_TX_CMD_TX_START		BIT(1)
#define USBC_TX_CMD_TXBUF_CMD(r)	((r >> 5) & 0x7)

#define USBC_TX_INFO_TX_SOP		(BIT(0) | BIT(1) | BIT(2))
#define USBC_TX_INFO_TX_RETRIES		(BIT(3) | BIT(4) | BIT(5))

#define USBC_RX_STATUS_RX_DATA		BIT(7)
#define USBC_RX_STATUS_RX_OVERRUN	BIT(6)
#define USBC_RX_STATUS_RX_CLEAR		BIT(0)

#define USBC_PD_STATUS_RX_RSLT(r)	((r >> 3) & 0x7)
#define USBC_PD_STATUS_TX_RSLT(r)	(r & 0x7)

#define USBC_RX_INFO_RXBYTES(r)		((r >> 3) & 0x1f)
#define USBC_RX_INFO_RX_SOP(r)		(r & 0x7)

#define USBC_PD_RX_BUF_LEN		30
#define USBC_PD_TX_BUF_LEN		30

struct wcove_typec {
	int pd_port_num;
	struct mutex lock; /* device lock */
	struct device *dev;
	struct regmap *regmap;
	struct typec_port *port;
	struct pd_sink_port pd_port;
	struct completion complete;
	struct typec_capability cap;
	struct typec_connection con;
	struct typec_partner partner;
};

enum wcove_typec_func {
	WCOVE_FUNC_DRIVE_VBUS = 1,
	WCOVE_FUNC_ORIENTATION,
	WCOVE_FUNC_ROLE,
	WCOVE_FUNC_DRIVE_VCONN,
};

enum wcove_typec_orientation {
	WCOVE_ORIENTATION_NORMAL,
	WCOVE_ORIENTATION_REVERSE,
};

enum wcove_typec_role {
	WCOVE_ROLE_HOST,
	WCOVE_ROLE_DEVICE,
};

static struct sink_ps profiles[] = {

	{
		.ps_type = PS_TYPE_FIXED,
		.ps_fixed = {
			.voltage_fixed	= 100,	/* 5V/50mV = 100 */
			.current_default = 90,	/* 900mA/10mA = 90 */
			.current_max	= 90,	/* 900mA/10mA = 90 */
		},

	},

	{
		.ps_type = PS_TYPE_FIXED,
		.ps_fixed = {
			.voltage_fixed	= 100,
			.current_default = 300,
			.current_max	= 300,
		},
	},

	{
		.ps_type = PS_TYPE_FIXED,
		.ps_fixed = {
			.voltage_fixed	= 240,
			.current_default = 300,
			.current_max	= 300,
		},
	},

};

static struct pd_sink_profile profile = {
	.hw_goodcrc_tx = true,
	.hw_goodcrc_rx = true,
	.gotomin = true,
	.usb_comm = true,
	.spec = USB_SPEC_3X,
	.pd_rev = PD_REVISION_2,
	.ps = profiles,
	.nr_ps = ARRAY_SIZE(profiles),
	.active_ps = 2, /* voltage = 5V, current = 3A */
};

static uuid_le uuid = UUID_LE(0x482383f0, 0x2876, 0x4e49,
			      0x86, 0x85, 0xdb, 0x66, 0x21, 0x1a, 0xf0, 0x37);

static int wcove_typec_func(struct wcove_typec *wcove,
			    enum wcove_typec_func func, int param)
{
	union acpi_object *obj;
	union acpi_object tmp;
	union acpi_object argv4 = ACPI_INIT_DSM_ARGV4(1, &tmp);

	tmp.type = ACPI_TYPE_INTEGER;
	tmp.integer.value = param;

	obj = acpi_evaluate_dsm(ACPI_HANDLE(wcove->dev), uuid.b, 1, func,
				&argv4);
	if (!obj) {
		dev_err(wcove->dev, "%s: failed to evaluate _DSM\n", __func__);
		return -EIO;
	}

	ACPI_FREE(obj);
	return 0;
}

static void wcove_typec_device_mode(struct wcove_typec *wcove)
{
	wcove->partner.type = TYPEC_PARTNER_USB;
	wcove->con.partner = &wcove->partner;
	wcove->con.pwr_role = TYPEC_SINK;
	wcove->con.vconn_role = TYPEC_SINK;
	wcove_typec_func(wcove, WCOVE_FUNC_ROLE, WCOVE_ROLE_DEVICE);
	typec_connect(wcove->port, &wcove->con);
}

static int wcove_typec_pd_recv_pkt_handler(struct wcove_typec *wcove)
{
	unsigned int rx_status;
	unsigned int rx_info;
	unsigned int temp;
	int len;
	int ret, i;
	struct pd_sink_msg *msg;
	char buf[USBC_PD_RX_BUF_LEN];

	ret = regmap_read(wcove->regmap, USBC_RX_STATUS, &rx_status);
	if (ret)
		goto err;

	while (rx_status & USBC_RX_STATUS_RX_DATA) {
		ret = regmap_read(wcove->regmap, USBC_RX_INFO, &rx_info);
		if (ret)
			goto err;

		len = (USBC_RX_INFO_RXBYTES(rx_info));

		for (i = 0; i < len; i++) {
			ret = regmap_read(wcove->regmap, USBC_RX_DATA_START + i,
									&temp);
			buf[i] = (char)temp;
			if (ret)
				goto err;
		}

		msg = pd_sink_alloc_msg(wcove->pd_port_num, len);
		memcpy(msg->buf, buf, len);

		switch (USBC_RX_INFO_RX_SOP(rx_info)) {
		case SOP:
			msg->sop_type = SOP;
			break;
		case SOP_P:
			msg->sop_type = SOP_P;
			break;
		case SOP_PP:
			msg->sop_type = SOP_PP;
			break;
		default:
			pr_warn("Packet type not supported\n");
		}

		pd_sink_queue_msg(msg);

		/* Clear RX status */
		regmap_update_bits(wcove->regmap, USBC_RX_STATUS,
			USBC_RX_STATUS_RX_CLEAR, USBC_RX_STATUS_RX_CLEAR);

		ret = regmap_read(wcove->regmap, USBC_RX_STATUS, &rx_status);
		if (ret)
			goto err;
	}

	return 0;

err:
	return ret;
}

static int wcove_typec_pd_tx_pkt_handler(int port_num, void *data,
				void *buf, int len, enum sop_type pkt_type)
{
	unsigned int tx_cmd;
	unsigned int val;
	int ret, i;
	char *buf1 = buf;
	struct wcove_typec *wcove = data;

	ret = regmap_read(wcove->regmap, USBC_TX_CMD, &tx_cmd);
	if (ret)
		goto err;

	if (!(tx_cmd & USBC_TX_CMD_TXBUF_RDY))
		return -EBUSY;

	for (i = 0; i < len; i++)
		ret = regmap_write(wcove->regmap, USBC_TX_DATA_START + i,
								buf1[i]);
		if (ret)
			goto err;

	regmap_read(wcove->regmap, USBC_TX_INFO, &val);
	ret = regmap_write(wcove->regmap, USBC_TX_INFO, 0x71);
	if (ret)
		goto err;

	ret = regmap_write(wcove->regmap, USBC_TX_CMD,
			USBC_TX_CMD_TX_START | (1 << 5));
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_TX_CMD, &tx_cmd);
	if (ret)
		goto err;

err:
	kfree(buf1);
	return ret;
}

static irqreturn_t  wcove_typec_irq(int irq, void *data)
{
	struct wcove_typec *wcove = data;
	unsigned int cc1_ctrl;
	unsigned int cc2_ctrl;
	unsigned int cc_irq1;
	unsigned int cc_irq2;
	unsigned int status1;
	unsigned int status2;
	unsigned int rx_status;
	int ret;

	mutex_lock(&wcove->lock);
	ret = regmap_read(wcove->regmap, USBC_IRQ1, &cc_irq1);
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_IRQ2, &cc_irq2);
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_STATUS1, &status1);
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_STATUS2, &status2);
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_CC1_CTRL, &cc1_ctrl);
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_CC2_CTRL, &cc2_ctrl);
	if (ret)
		goto err;

	ret = regmap_read(wcove->regmap, USBC_RX_STATUS, &rx_status);
	if (ret)
		goto err;

	if (cc_irq1) {
		if (cc_irq1 & USBC_IRQ1_OVERTEMP)
			dev_err(wcove->dev, "VCONN Switch Over Temperature!\n");
		if (cc_irq1 & USBC_IRQ1_SHORT)
			dev_err(wcove->dev, "VCONN Switch Short Circuit!\n");
		regmap_write(wcove->regmap, USBC_IRQ1, cc_irq1);
	}

	if (status1 & USBC_STATUS1_DET_ONGOING)
		goto out;

	if (USBC_STATUS1_RSLT(status1) == USBC_RSLT_NOTHING) {
		if (wcove->con.partner) {
			typec_disconnect(wcove->port);
			memset(&wcove->con, 0, sizeof(wcove->con));
			memset(&wcove->partner, 0, sizeof(wcove->partner));
		}

		wcove_typec_func(wcove, WCOVE_FUNC_ORIENTATION,
				 WCOVE_ORIENTATION_NORMAL);
		/* Host mode by default */
		wcove_typec_func(wcove, WCOVE_FUNC_ROLE, WCOVE_ROLE_HOST);

		/* reset the pd sink state */
		if (wcove->pd_port_num >= 0)
			pd_sink_reset_state(wcove->pd_port_num);

		goto out;
	}

	switch (USBC_STATUS1_ORIENT(status1)) {
	case USBC_ORIENT_NORMAL:
		wcove_typec_func(wcove, WCOVE_FUNC_ORIENTATION,
				 WCOVE_ORIENTATION_NORMAL);
		regmap_update_bits(wcove->regmap, USBC_CC_SEL,
					USBC_CC_SEL_CCSEL, 0x1);
		break;
	case USBC_ORIENT_REVERSE:
		wcove_typec_func(wcove, WCOVE_FUNC_ORIENTATION,
				 WCOVE_ORIENTATION_REVERSE);
		regmap_update_bits(wcove->regmap, USBC_CC_SEL,
					USBC_CC_SEL_CCSEL, 0x2);
	default:
		break;
	}

	if (cc_irq2 & USBC_IRQ2_RX_PD ||
		rx_status & USBC_RX_STATUS_RX_DATA)
		wcove_typec_pd_recv_pkt_handler(wcove);

	if (cc_irq2 & USBC_IRQ2_RX_HR)
		pr_debug("RX HR not implemented\n");

	if (cc_irq2 & USBC_IRQ2_RX_CR)
		pr_debug("RX CR not implemented\n");

	if (cc_irq2 & USBC_IRQ2_TX_SUCCESS) {
		pd_sink_tx_complete(wcove->pd_port_num);
		pr_debug("TX_SENT\n");
	}

	if (cc_irq2 & USBC_IRQ2_TX_FAIL)
		pr_debug("TX_FAIL\n");

	if (wcove->con.partner)
		goto out;

	switch (USBC_STATUS1_RSLT(status1)) {
	case USBC_RSLT_SRC_DEFAULT:
		wcove->con.pwr_opmode = TYPEC_PWR_MODE_USB;
		wcove_typec_device_mode(wcove);
		break;
	case USBC_RSLT_SRC_1_5A:
		wcove->con.pwr_opmode = TYPEC_PWR_MODE_1_5A;
		wcove_typec_device_mode(wcove);
		break;
	case USBC_RSLT_SRC_3_0A:
		wcove->con.pwr_opmode = TYPEC_PWR_MODE_3_0A;
		wcove_typec_device_mode(wcove);
		break;
	case USBC_RSLT_SNK:
		wcove->partner.type = TYPEC_PARTNER_USB;
		wcove->con.partner = &wcove->partner;
		wcove->con.data_role = TYPEC_HOST;
		wcove->con.pwr_role = TYPEC_SOURCE;
		wcove->con.vconn_role = TYPEC_SOURCE;
		wcove_typec_func(wcove, WCOVE_FUNC_ROLE, WCOVE_ROLE_HOST);
		typec_connect(wcove->port, &wcove->con);
		break;
	case USBC_RSLT_DEBUG_ACC:
		wcove->partner.accessory = TYPEC_ACCESSORY_DEBUG;
		wcove->partner.type = TYPEC_PARTNER_ACCESSORY;
		wcove->con.partner = &wcove->partner;
		typec_connect(wcove->port, &wcove->con);
		break;
	case USBC_RSLT_AUDIO_ACC:
		wcove->partner.accessory = TYPEC_ACCESSORY_AUDIO;
		wcove->partner.type = TYPEC_PARTNER_ACCESSORY;
		wcove->con.partner = &wcove->partner;
		typec_connect(wcove->port, &wcove->con);
		break;
	default:
		dev_WARN(wcove->dev, "%s Undefined result\n", __func__);
		goto err;
	}

	if (!completion_done(&wcove->complete))
		complete(&wcove->complete);
out:
	/* If either CC pins is requesting VCONN, we turn it on */
	if ((cc1_ctrl & USBC_CC_CTRL_VCONN_EN) ||
	    (cc2_ctrl &	USBC_CC_CTRL_VCONN_EN))
		wcove_typec_func(wcove, WCOVE_FUNC_DRIVE_VCONN, true);
	else
		wcove_typec_func(wcove, WCOVE_FUNC_DRIVE_VCONN, false);

	/* Relying on the FSM to know when we need to drive VBUS. */
	wcove_typec_func(wcove, WCOVE_FUNC_DRIVE_VBUS,
			 !!(status2 & USBC_STATUS2_VBUS_REQ));
err:
	/* REVISIT: Clear WhiskeyCove CHGR Type-C interrupt */
	regmap_write(wcove->regmap, WCOVE_CHGRIRQ0, BIT(5) | BIT(4) |
						    BIT(3) | BIT(0));
	regmap_write(wcove->regmap, USBC_IRQ1, cc_irq1);
	regmap_write(wcove->regmap, USBC_IRQ2, cc_irq2);

	mutex_unlock(&wcove->lock);
	return IRQ_HANDLED;
}

static int wcove_typec_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	struct wcove_typec *wcove;
	unsigned int val;
	int ret;

	wcove = devm_kzalloc(&pdev->dev, sizeof(*wcove), GFP_KERNEL);
	if (!wcove)
		return -ENOMEM;

	init_completion(&wcove->complete);
	mutex_init(&wcove->lock);
	wcove->dev = &pdev->dev;
	wcove->regmap = pmic->regmap;

	ret = regmap_irq_get_virq(pmic->irq_chip_data_level2,
				  platform_get_irq(pdev, 0));
	if (ret < 0)
		return ret;

	ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
				wcove_typec_irq, IRQF_ONESHOT,
					"wcove_typec", wcove);
	if (ret)
		return ret;

	wcove->cap.type = TYPEC_PORT_DRP;
	wcove->port = typec_register_port(&pdev->dev, &wcove->cap);
	if (IS_ERR(wcove->port))
		return PTR_ERR(wcove->port);

	/* PD receive packet handler */
	wcove->pd_port_num = pd_sink_register_port(&profile,
				wcove_typec_pd_tx_pkt_handler, wcove);
	if (wcove->pd_port_num) {
		pr_err("Register pd sink port failed\n");
		return -EIO;
	}

	if (!acpi_check_dsm(ACPI_HANDLE(&pdev->dev), uuid.b, 0, 0x1f)) {
		dev_err(&pdev->dev, "Missing _DSM functions\n");
		return -ENODEV;
	}

	/* DRP mode without accessory support */
	regmap_read(wcove->regmap, USBC_CONTROL1, &val);
	regmap_write(wcove->regmap, USBC_CONTROL1, USBC_CONTROL1_MODE_DRP(val));

	/* Unmask everything */
	regmap_read(wcove->regmap, USBC_IRQMASK1, &val);
	regmap_write(wcove->regmap, USBC_IRQMASK1, val & ~USBC_IRQMASK1_ALL);
	regmap_read(wcove->regmap, USBC_IRQMASK2, &val);
	regmap_write(wcove->regmap, USBC_IRQMASK2, val & ~USBC_IRQMASK2_ALL);

	/*Set HW control the ID of outgoing messages*/
	regmap_write(wcove->regmap, USBC_PD_CFG1, BIT(7));

	/* Enable SOP messages for now */
	regmap_write(wcove->regmap, USBC_PD_CFG2, BIT(0));

	/*Set the PD revision */
	regmap_read(wcove->regmap, USBC_PD_CFG3, &val);
	val = 0x14;
	regmap_write(wcove->regmap, USBC_PD_CFG3, val);

	platform_set_drvdata(pdev, wcove);
	return 0;
}

static int wcove_typec_remove(struct platform_device *pdev)
{
	struct wcove_typec *wcove = platform_get_drvdata(pdev);
	unsigned int val;

	/* Mask everything */
	regmap_read(wcove->regmap, USBC_IRQMASK1, &val);
	regmap_write(wcove->regmap, USBC_IRQMASK1, val | USBC_IRQMASK1_ALL);
	regmap_read(wcove->regmap, USBC_IRQMASK2, &val);
	regmap_write(wcove->regmap, USBC_IRQMASK2, val | USBC_IRQMASK2_ALL);

	typec_unregister_port(wcove->port);
	pd_sink_unregister_port(wcove->pd_port_num);
	return 0;
}

static struct platform_driver wcove_typec_driver = {
	.driver = {
		.name		= "bxt_wcove_usbc",
	},
	.probe			= wcove_typec_probe,
	.remove			= wcove_typec_remove,
};

module_platform_driver(wcove_typec_driver);

MODULE_ALIAS("platform:bxt_wcove_usbc");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WhiskeyCove PMIC USB Type-C PHY driver");
