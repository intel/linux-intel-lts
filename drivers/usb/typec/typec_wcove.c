/**
 * typec_wcove.c - WhiskeyCove PMIC USB Type-C PHY driver
 *
 * Copyright (C) 2016 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/usb/typec.h>
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
#define USBC_STATUS1		0x7007
#define USBC_STATUS2		0x7008
#define USBC_STATUS3		0x7009
#define USBC_IRQ1		0x7015
#define USBC_IRQ2		0x7016
#define USBC_IRQMASK1		0x7017
#define USBC_IRQMASK2		0x7018

/* Register bits */

#define USBC_CONTROL1_MODE_DRP(r)	((r & ~0x7) | 4)

#define USBC_CONTROL2_UNATT_SNK		BIT(0)
#define USBC_CONTROL2_UNATT_SRC		BIT(1)
#define USBC_CONTROL2_DIS_ST		BIT(2)

#define USBC_CONTROL3_PD_DIS		BIT(1)

#define USBC_CC_CTRL_VCONN_EN		BIT(1)

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

struct wcove_typec {
	struct mutex lock; /* device lock */
	struct device *dev;
	struct regmap *regmap;
	struct typec_port *port;
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

static irqreturn_t wcove_typec_irq(int irq, void *data)
{
	struct wcove_typec *wcove = data;
	unsigned int cc1_ctrl;
	unsigned int cc2_ctrl;
	unsigned int cc_irq1;
	unsigned int cc_irq2;
	unsigned int status1;
	unsigned int status2;
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

	if (cc_irq1) {
		if (cc_irq1 & USBC_IRQ1_OVERTEMP)
			dev_err(wcove->dev, "VCONN Switch Over Temperature!\n");
		if (cc_irq1 & USBC_IRQ1_SHORT)
			dev_err(wcove->dev, "VCONN Switch Short Circuit!\n");
		regmap_write(wcove->regmap, USBC_IRQ1, cc_irq1);
	}

	if (cc_irq2) {
		regmap_write(wcove->regmap, USBC_IRQ2, cc_irq2);
		/*
		 * Ingoring any PD communication interrupts until the PD stack
		 * is in place
		 */
		if (cc_irq2 & ~USBC_IRQ2_CC_CHANGE) {
			dev_WARN(wcove->dev, "USB PD handling missing\n");
			goto err;
		}
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
		goto out;
	}

	if (wcove->con.partner)
		goto out;

	switch (USBC_STATUS1_ORIENT(status1)) {
	case USBC_ORIENT_NORMAL:
		wcove_typec_func(wcove, WCOVE_FUNC_ORIENTATION,
				 WCOVE_ORIENTATION_NORMAL);
		break;
	case USBC_ORIENT_REVERSE:
		wcove_typec_func(wcove, WCOVE_FUNC_ORIENTATION,
				 WCOVE_ORIENTATION_REVERSE);
	default:
		break;
	}

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
	regmap_write(wcove->regmap, WCOVE_CHGRIRQ0, BIT(5));

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

	if (!acpi_check_dsm(ACPI_HANDLE(&pdev->dev), uuid.b, 0, 0x1f)) {
		dev_err(&pdev->dev, "Missing _DSM functions\n");
		return -ENODEV;
	}

	/* Make sure the PD PHY is disabled until PD stack is ready */
	regmap_read(wcove->regmap, USBC_CONTROL3, &val);
	regmap_write(wcove->regmap, USBC_CONTROL3, val | USBC_CONTROL3_PD_DIS);

	/* DRP mode without accessory support */
	regmap_read(wcove->regmap, USBC_CONTROL1, &val);
	regmap_write(wcove->regmap, USBC_CONTROL1, USBC_CONTROL1_MODE_DRP(val));

	/* Unmask everything */
	regmap_read(wcove->regmap, USBC_IRQMASK1, &val);
	regmap_write(wcove->regmap, USBC_IRQMASK1, val & ~USBC_IRQMASK1_ALL);
	regmap_read(wcove->regmap, USBC_IRQMASK2, &val);
	regmap_write(wcove->regmap, USBC_IRQMASK2, val & ~USBC_IRQMASK2_ALL);

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

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WhiskeyCove PMIC USB Type-C PHY driver");
