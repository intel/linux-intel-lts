// SPDX-License-Identifier: GPL-2.0
/**
 * Intel WhiskeyCove PMIC Charger Driver
 *
 * Copyright (C) 2019 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/usb/role.h>
#include <linux/mfd/intel_soc_pmic.h>

/* Registers */
#define WCOVE_CHRGIRQ0			0x4e09
#define WCOVE_CHRGIRQ1			0x4e0a
#define WCOVE_PWRSRC			0x4e20
#define WCOVE_USBSRC			0x4e29
#define WCOVE_USBC_STATUS1		0x7007

/* IRQ masks */
#define WCOVE_CHRGIRQ0_MASK		0x10
#define WCOVE_CHRGIRQ1_MASK		0x1f

/* Register Bits */

#define WCOVE_CHRGIRQ1_VBUS		BIT(0)
#define WCOVE_CHRGIRQ1_DC		BIT(1)
#define WCOVE_CHRGIRQ1_BATT		BIT(2)
#define WCOVE_CHRGIRQ1_USBIDFLD		BIT(3)
#define WCOVE_CHRGIRQ1_USBIDGND		BIT(4)

#define WCOVE_PWRSRC_VBUS		BIT(0)
#define WCOVE_PWRSRC_DC			BIT(1)
#define WCOVE_PWRSRC_BATT		BIT(2)
#define WCOVE_PWRSRC_USBID(p)		(GENMASK(4, 3) >> 3)
#define   WCOVE_PWRSRC_USBID_ACA	0
#define   WCOVE_PWRSRC_USBID_GND	1
#define   WCOVE_PWRSRC_USBID_FLOAT	2

#define WCOVE_USBSRC_DET(p)		GENMASK(1, 0)
#define   WCOVE_USBSRC_DET_NOT_STARTED	0
#define   WCOVE_USBSRC_DET_ON_GOING	1
#define   WCOVE_USBSRC_DET_COMPLETE	2
#define   WCOVE_USBSRC_DET_FAILED	3
#define WCOVE_USBSRC_RESULT(p)		(GENMASK(5, 2) >> 2)
#define   WCOVE_USBSRC_RESULT_NOT_DET	0
#define   WCOVE_USBSRC_RESULT_SDP	1
#define   WCOVE_USBSRC_RESULT_DCP	2
#define   WCOVE_USBSRC_RESULT_CDP	3
#define   WCOVE_USBSRC_RESULT_ACA	4
#define   WCOVE_USBSRC_RESULT_SE1	5
#define   WCOVE_USBSRC_RESULT_MHL	6
#define   WCOVE_USBSRC_RESULT_FLOAT	7
#define   WCOVE_USBSRC_RESULT_DCP_EXT	9

struct wcove_charger {
	struct mutex lock; /* device lock */
	struct device *dev;
	struct regmap *regmap;
	struct usb_role_switch *sw;

	struct power_supply *usb;
	struct power_supply *mains;
};

static irqreturn_t wcove_chrg_irq(int irq, void *data)
{
	struct wcove_charger *wcove = data;
	u32 chrgirq;
	u32 usbsrc;
	int ret;

	mutex_lock(&wcove->lock);

	ret = regmap_read(wcove->regmap, WCOVE_CHRGIRQ0, &chrgirq);
	if (ret)
		goto err_unlock;

	ret = regmap_read(wcove->regmap, WCOVE_USBSRC, &usbsrc);
	if (ret)
		goto err_clear_irq;

	if (WCOVE_USBSRC_DET(usbsrc) != WCOVE_USBSRC_DET_COMPLETE)
		goto err_clear_irq;

	switch (WCOVE_USBSRC_DET(usbsrc)) {
	case WCOVE_USBSRC_RESULT_DCP:
	case WCOVE_USBSRC_RESULT_DCP_EXT:
		if (usb_role_switch_set_role(wcove->sw, USB_ROLE_NONE))
			dev_err(wcove->dev, "failed to set USB role\n");
		break;
	default:
		break;
	}

	power_supply_changed(wcove->usb);

err_clear_irq:
	regmap_write(wcove->regmap, WCOVE_CHRGIRQ0,
		     chrgirq & WCOVE_CHRGIRQ0_MASK);

err_unlock:
	mutex_unlock(&wcove->lock);

	return IRQ_HANDLED;
}

static irqreturn_t wcove_chrg1_irq(int irq, void *data)
{
	struct wcove_charger *wcove = data;
	enum usb_role role = USB_ROLE_NONE;
	u32 chrgirq1;
	u32 pwrsrc;
	u32 typec;
	int ret;

	mutex_lock(&wcove->lock);

	ret = regmap_read(wcove->regmap, WCOVE_CHRGIRQ1, &chrgirq1);
	if (ret)
		goto err_unlock;

	ret = regmap_read(wcove->regmap, WCOVE_PWRSRC, &pwrsrc);
	if (ret)
		goto err_clear_irq;

	if (chrgirq1 & WCOVE_CHRGIRQ1_DC)
		power_supply_changed(wcove->mains);

	/* USB Type-C connector is handled separately. */
	ret = regmap_read(wcove->regmap, WCOVE_USBC_STATUS1, &typec);
	if (!ret && typec)
		goto err_clear_irq;

	if (chrgirq1 & WCOVE_CHRGIRQ1_USBIDGND &&
	    WCOVE_PWRSRC_USBID(pwrsrc) == WCOVE_PWRSRC_USBID_GND) {
		role = USB_ROLE_HOST;
	} else if (chrgirq1 & WCOVE_CHRGIRQ1_VBUS) {
		if (pwrsrc & WCOVE_PWRSRC_VBUS)
			role = USB_ROLE_DEVICE;
		power_supply_changed(wcove->usb);
	}

	if (usb_role_switch_set_role(wcove->sw, role))
		dev_err(wcove->dev, "failed to set USB role\n");

err_clear_irq:
	regmap_write(wcove->regmap, WCOVE_CHRGIRQ1,
		     chrgirq1 & WCOVE_CHRGIRQ1_MASK);

err_unlock:
	mutex_unlock(&wcove->lock);

	return IRQ_HANDLED;
}

static int wcove_mains_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct wcove_charger *wcove = power_supply_get_drvdata(psy);
	int ret = 0;
	u32 pwrsrc;

	mutex_lock(&wcove->lock);

	ret = regmap_read(wcove->regmap, WCOVE_PWRSRC, &pwrsrc);
	if (ret)
		goto err_unlock;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(pwrsrc & WCOVE_PWRSRC_DC);
		break;
	default:
		ret = -EINVAL;
		break;
	}

err_unlock:
	mutex_unlock(&wcove->lock);

	return ret;
}

static enum power_supply_property wcove_mains_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc dc_desc = {
	.name = "wcove_mains",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = wcove_mains_props,
	.num_properties = ARRAY_SIZE(wcove_mains_props),
	.get_property = wcove_mains_get_prop,
};

static int wcove_usb_get_prop(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct wcove_charger *wcove = power_supply_get_drvdata(psy);
	int ret = 0;
	u32 pwrsrc;
	u32 usbsrc;

	mutex_lock(&wcove->lock);

	ret = regmap_read(wcove->regmap, WCOVE_PWRSRC, &pwrsrc);
	if (ret)
		goto err_unlock;

	ret = regmap_read(wcove->regmap, WCOVE_USBSRC, &usbsrc);
	if (ret)
		goto err_unlock;

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		switch (WCOVE_USBSRC_RESULT(usbsrc)) {
		case WCOVE_USBSRC_RESULT_SDP:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case WCOVE_USBSRC_RESULT_DCP:
		case WCOVE_USBSRC_RESULT_DCP_EXT:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case WCOVE_USBSRC_RESULT_CDP:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		case WCOVE_USBSRC_RESULT_ACA:
			val->intval = POWER_SUPPLY_USB_TYPE_ACA;
			break;
		case WCOVE_USBSRC_RESULT_SE1:
			val->intval = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
			break;
		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* Make sure we are not the one driving VBUS */
		if (WCOVE_PWRSRC_USBID(pwrsrc) != WCOVE_PWRSRC_USBID_GND)
			val->intval = !!(pwrsrc & WCOVE_PWRSRC_VBUS);
		else
			val->intval = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

err_unlock:
	mutex_unlock(&wcove->lock);

	return ret;
}

static enum power_supply_usb_type wcove_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};

static enum power_supply_property wcove_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc usb_desc = {
	.name = "wcove_usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = wcove_usb_types,
	.num_usb_types = ARRAY_SIZE(wcove_usb_types),
	.properties = wcove_usb_props,
	.num_properties = ARRAY_SIZE(wcove_usb_props),
	.get_property = wcove_usb_get_prop,
};

static int wcove_charger_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config cfg = {};
	struct wcove_charger *wcove;
	int irq1;
	int irq;
	int ret;

	irq = regmap_irq_get_virq(pmic->irq_chip_data_chgr,
				  platform_get_irq_byname(pdev, "CHARGER"));
	if (irq < 0)
		return irq;

	irq1 = regmap_irq_get_virq(pmic->irq_chip_data_chgr,
				   platform_get_irq_byname(pdev, "CHARGER1"));
	if (irq1 < 0)
		return irq1;

	wcove = devm_kzalloc(&pdev->dev, sizeof(*wcove), GFP_KERNEL);
	if (!wcove)
		return -ENOMEM;

	mutex_init(&wcove->lock);
	wcove->regmap = pmic->regmap;
	wcove->dev = &pdev->dev;

	wcove->sw = usb_role_switch_get(&pdev->dev);
	if (IS_ERR(wcove->sw))
		return PTR_ERR(wcove->sw);

	cfg.drv_data = wcove;

	wcove->mains = devm_power_supply_register(&pdev->dev, &dc_desc, &cfg);
	if (IS_ERR(wcove->mains)) {
		ret = PTR_ERR(wcove->mains);
		goto err_release_switch;
	}

	wcove->usb = devm_power_supply_register(&pdev->dev, &usb_desc, &cfg);
	if (IS_ERR(wcove->usb)) {
		ret = PTR_ERR(wcove->usb);
		goto err_release_switch;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					wcove_chrg_irq, IRQF_ONESHOT,
					"wcove_charger", wcove);
	if (ret)
		goto err_release_switch;

	ret = devm_request_threaded_irq(&pdev->dev, irq1, NULL,
					wcove_chrg1_irq, IRQF_ONESHOT,
					"wcove_charger1", wcove);
	if (ret)
		goto err_release_switch;

	platform_set_drvdata(pdev, wcove);

	return 0;

err_release_switch:
	usb_role_switch_put(wcove->sw);

	return ret;
}

static int wcove_charger_remove(struct platform_device *pdev)
{
	struct wcove_charger *wcove = platform_get_drvdata(pdev);

	usb_role_switch_put(wcove->sw);

	return 0;
}

static struct platform_driver wcove_charger_driver = {
	.driver = {
		.name = "bxt_wcove_ext_charger",
	},
	.probe = wcove_charger_probe,
	.remove = wcove_charger_remove,
};
module_platform_driver(wcove_charger_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_DESCRIPTION("Intel WhiskeyCove PMIC Charger Driver");
MODULE_LICENSE("GPL v2");
