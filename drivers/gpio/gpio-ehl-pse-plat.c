// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Elkhart Lake PSE GPIO driver
 *
 * Copyright (c) 2016 Intel Corporation.
 * Author: N. Pandith <pandith.n@intel.com>
 */

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/core.h>

#define GCCR		0x0000	/* controller configuration */
#define GPLR		0x0004	/* pin level r/o */
#define GPDR		0x001c	/* pin direction */
#define GPSR		0x0034	/* pin set w/o */
#define GPCR		0x004c	/* pin clear w/o */
#define GRER		0x0064	/* rising edge detect */
#define GFER		0x007c	/* falling edge detect */
#define GFBR		0x0094	/* glitch filter bypass */
#define GIMR		0x00ac	/* interrupt mask */
#define GISR		0x00c4	/* interrupt source */
#define GWMR		0x0100	/* wake mask */
#define GWSR		0x0118	/* wake source */
#define GSIR		0x0130	/* secure input */

/* Each Intel EHL PSE GPIO Controller has 30 GPIO pins */
#define EHL_PSE_NGPIO		30

#define TGPIO_D0I3C 0x2000
#define TGPIO_CGSR 0x2004

#define TGPIO_D0I3_CIP BIT(0)
#define TGPIO_D0I3_EN BIT(2)
#define TGPIO_D0I3_RR BIT(3)
#define TGPIO_CGSR_CG BIT(16)

struct ehl_pse_gpio_context {
	u32 gplr;
	u32 gpdr;
	u32 grer;
	u32 gfer;
	u32 gimr;
	u32 gwmr;
};

struct ehl_pse_gpio {
	struct gpio_chip		chip;
	void __iomem			*reg_base;
	raw_spinlock_t			lock;
	struct device			*dev;
	struct ehl_pse_gpio_context	*ctx;
};

static inline u32 intel_gpio_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void intel_gpio_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static int ehl_pse_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	/* increase runtime_usage counter */
	pm_runtime_get_sync(chip->parent);
	return 0;
}

static void ehl_pse_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	/* decrease runtime_usage counter */
	pm_runtime_put_sync(chip->parent);
}

static void __iomem *gpio_reg(struct gpio_chip *chip, unsigned int offset,
				unsigned int reg_type_offset)
{
	struct ehl_pse_gpio *priv = gpiochip_get_data(chip);
	u8 reg = offset / 32;

	return priv->reg_base + reg_type_offset + reg * 4;
}

static int ehl_pse_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ehl_pse_gpio *priv = gpiochip_get_data(chip);
	u32 reg_gplr;

	reg_gplr = intel_gpio_readl(priv->reg_base, GPLR);

	return !!(reg_gplr & BIT(offset));
}

static void ehl_pse_gpio_set(struct gpio_chip *chip, unsigned int offset,
				int value)
{
	struct ehl_pse_gpio *priv = gpiochip_get_data(chip);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);

	if (value)
		intel_gpio_writel(priv->reg_base, GPSR, BIT(offset));

	else
		intel_gpio_writel(priv->reg_base, GPCR, BIT(offset));


	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static int ehl_pse_gpio_direction_input(struct gpio_chip *chip,
					unsigned int offset)
{
	struct ehl_pse_gpio *priv = gpiochip_get_data(chip);
	unsigned long flags;
	u32 value;

	pm_runtime_get_sync(priv->dev->parent);

	raw_spin_lock_irqsave(&priv->lock, flags);

	value = intel_gpio_readl(priv->reg_base, GPDR);
	value &= ~BIT(offset);
	intel_gpio_writel(priv->reg_base, GPDR, value);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	pm_runtime_put(priv->dev->parent);

	return 0;
}

static int ehl_pse_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct ehl_pse_gpio *priv = gpiochip_get_data(chip);
	unsigned long flags;

	pm_runtime_get_sync(priv->dev->parent);

	ehl_pse_gpio_set(chip, offset, value);

	raw_spin_lock_irqsave(&priv->lock, flags);

	value = intel_gpio_readl(priv->reg_base, GPDR);
	value |= BIT(offset);
	intel_gpio_writel(priv->reg_base, GPDR, value);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	pm_runtime_put(priv->dev->parent);

	return 0;
}

static int ehl_pse_gpio_get_direction(struct gpio_chip *chip,
			unsigned int offset)
{
	struct ehl_pse_gpio *priv = gpiochip_get_data(chip);
	int direction = 0;

	pm_runtime_get_sync(priv->dev->parent);

	u32 reg_gpdr = intel_gpio_readl(priv->reg_base, GPDR);

	direction = !(reg_gpdr & BIT(offset));

	pm_runtime_put(priv->dev->parent);

	return direction;
}

static void ehl_pse_irq_ack(struct irq_data *d)
{
	struct ehl_pse_gpio *priv = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	void __iomem *gisr = gpio_reg(&priv->chip, gpio, GISR);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);

	writel(BIT(gpio % 32), gisr);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void ehl_pse_irq_unmask_mask(struct irq_data *d, bool unmask)
{
	struct ehl_pse_gpio *priv = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	void __iomem *gimr = gpio_reg(&priv->chip, gpio, GIMR);
	unsigned long flags;
	u32 value;

	raw_spin_lock_irqsave(&priv->lock, flags);

	if (unmask)
		value = readl(gimr) | BIT(gpio % 32);
	else
		value = readl(gimr) & ~BIT(gpio % 32);
	writel(value, gimr);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void ehl_pse_irq_mask(struct irq_data *d)
{
	ehl_pse_irq_unmask_mask(d, false);
}

static void ehl_pse_irq_unmask(struct irq_data *d)
{
	ehl_pse_irq_unmask_mask(d, true);
}

static int ehl_pse_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ehl_pse_gpio *priv = gpiochip_get_data(gc);
	u32 gpio = irqd_to_hwirq(d);
	void __iomem *grer = gpio_reg(&priv->chip, gpio, GRER);
	void __iomem *gfer = gpio_reg(&priv->chip, gpio, GFER);
	unsigned long flags;
	u32 value;

	pm_runtime_get_sync(priv->dev->parent);

	raw_spin_lock_irqsave(&priv->lock, flags);

	if (type & IRQ_TYPE_EDGE_RISING)
		value = readl(grer) | BIT(gpio % 32);
	else
		value = readl(grer) & ~BIT(gpio % 32);
	writel(value, grer);

	if (type & IRQ_TYPE_EDGE_FALLING)
		value = readl(gfer) | BIT(gpio % 32);
	else
		value = readl(gfer) & ~BIT(gpio % 32);
	writel(value, gfer);

	/* We only support edge IRQs */
	if (type & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(d, handle_edge_irq);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	pm_runtime_put(priv->dev->parent);

	return 0;
}

static int ehl_pse_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct ehl_pse_gpio *priv = gpiochip_get_data(gc);
	u32 gpio = irqd_to_hwirq(d);
	void __iomem *gwmr = gpio_reg(&priv->chip, gpio, GWMR);
	void __iomem *gwsr = gpio_reg(&priv->chip, gpio, GWSR);
	unsigned long flags;
	u32 value;

	raw_spin_lock_irqsave(&priv->lock, flags);

	/* Clear the existing wake status */
	writel(BIT(gpio % 32), gwsr);

	if (on)
		value = readl(gwmr) | BIT(gpio % 32);
	else
		value = readl(gwmr) & ~BIT(gpio % 32);
	writel(value, gwmr);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	dev_dbg(priv->dev, "%sable wake for gpio %u\n", on ?
			"en" : "dis", gpio);
	return 0;
}

static struct irq_chip ehl_pse_irqchip = {
	.name		= "gpi-ehl-pse",
	.irq_ack	= ehl_pse_irq_ack,
	.irq_mask	= ehl_pse_irq_mask,
	.irq_unmask	= ehl_pse_irq_unmask,
	.irq_set_type	= ehl_pse_irq_set_type,
	.irq_set_wake	= ehl_pse_irq_set_wake,
};



static void ehl_pse_irq_handler(struct irq_desc *desc)
{

	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct ehl_pse_gpio *priv = gpiochip_get_data(gc);
	unsigned long base, gpio;

	chained_irq_enter(irqchip, desc);

	for (base = 0; base < priv->chip.ngpio; base += 32) {
		void __iomem *gisr = gpio_reg(&priv->chip, base, GISR);
		void __iomem *gimr = gpio_reg(&priv->chip, base, GIMR);
		unsigned long pending, enabled;

		pending = readl(gisr);
		enabled = readl(gimr);

		/* Only interrupts that are enabled */
		pending &= enabled;

		if (pending) {
			for_each_set_bit(gpio, &pending, 32) {
				unsigned int irq =
					irq_find_mapping(gc->irq.domain, gpio);

				generic_handle_irq(irq);
			}
		}
	}

	chained_irq_exit(irqchip, desc);
}

static int ehl_pse_irq_init_hw(struct gpio_chip *chip)
{
	void __iomem *reg;
	unsigned int base;

	for (base = 0; base < chip->ngpio; base += 32) {
		/* Clear the interrupt-mask register */
		reg = gpio_reg(chip, base, GIMR);
		writel(0, reg);
		/* Clear the interrupt-source register */
		reg = gpio_reg(chip, base, GISR);
		writel(0, reg);
		/* Clear the rising-edge detect register */
		reg = gpio_reg(chip, base, GRER);
		writel(0, reg);
		/* Clear the rising-edge detect register */
		reg = gpio_reg(chip, base, GRER);
		writel(0, reg);
	}
	return 0;
}

static int ehl_pse_gpio_probe(struct platform_device *pdev)
{
	struct gpio_irq_chip *girq;
	struct ehl_pse_gpio *priv;
	struct resource *res;
	void __iomem *base;
	u32 irq;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ctx = devm_kzalloc(&pdev->dev, sizeof(*priv->ctx), GFP_KERNEL);
	if (!priv->ctx)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return NULL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return NULL;

	priv->dev = &pdev->dev;
	priv->reg_base = devm_ioremap_resource(&pdev->dev, res);

	priv->chip.label = dev_name(&pdev->dev);
	priv->chip.parent = &pdev->dev;
	priv->chip.request = ehl_pse_gpio_request;
	priv->chip.free = ehl_pse_gpio_free;
	priv->chip.direction_input = ehl_pse_gpio_direction_input;
	priv->chip.direction_output = ehl_pse_gpio_direction_output;
	priv->chip.get = ehl_pse_gpio_get;
	priv->chip.set = ehl_pse_gpio_set;
	priv->chip.get_direction = ehl_pse_gpio_get_direction;
	priv->chip.base = -1;
	/* 30 pins only per instance according to spec */
	priv->chip.ngpio = EHL_PSE_NGPIO;
	priv->chip.can_sleep = false;
	priv->chip.add_pin_ranges = NULL;

	raw_spin_lock_init(&priv->lock);

	girq = &priv->chip.irq;
	girq->chip = &ehl_pse_irqchip;
	girq->init_hw = ehl_pse_irq_init_hw;
	girq->parent_handler = ehl_pse_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(&pdev->dev, girq->num_parents,
				sizeof(*girq->parents), GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;

	girq->parents[0] = irq;
	girq->first = 0;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;

	platform_set_drvdata(pdev, priv);

	ret = devm_gpiochip_add_data(&pdev->dev, &priv->chip, priv);
	if (ret) {
		dev_err(&pdev->dev, "gpiochip_add error %d\n", ret);
		return ret;
	}

	return 0;
}

static int ehl_pse_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static int ehl_pse_gpio_suspend(struct device *dev)
{
	struct ehl_pse_gpio *priv = dev_get_drvdata(dev);
	struct ehl_pse_gpio_context *ctx = priv->ctx;
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);

	ctx->gplr = intel_gpio_readl(priv->reg_base, GPLR);
	ctx->gpdr = intel_gpio_readl(priv->reg_base, GPDR);
	ctx->grer = intel_gpio_readl(priv->reg_base, GRER);
	ctx->gfer = intel_gpio_readl(priv->reg_base, GFER);
	ctx->gimr = intel_gpio_readl(priv->reg_base, GIMR);
	ctx->gwmr = intel_gpio_readl(priv->reg_base, GWMR);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int ehl_pse_gpio_resume(struct device *dev)
{
	struct ehl_pse_gpio *priv = dev_get_drvdata(dev);
	struct ehl_pse_gpio_context *ctx = priv->ctx;
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);

	/* GPLR is RO, values read will restore using GPSR */
	intel_gpio_writel(priv->reg_base, GPSR, ctx->gplr);
	intel_gpio_writel(priv->reg_base, GPDR, ctx->gpdr);
	intel_gpio_writel(priv->reg_base, GRER, ctx->grer);
	intel_gpio_writel(priv->reg_base, GFER, ctx->gfer);
	intel_gpio_writel(priv->reg_base, GIMR, ctx->gimr);
	intel_gpio_writel(priv->reg_base, GWMR, ctx->gwmr);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ehl_pse_gpio_pm_ops, ehl_pse_gpio_suspend,
			 ehl_pse_gpio_resume);


static struct platform_driver ehl_pse_gpio_driver = {
	.driver		= {
		.name		= "gpio-ehl-pse",
		.pm	= &ehl_pse_gpio_pm_ops,
	},
	.probe		= ehl_pse_gpio_probe,
	.remove		= ehl_pse_gpio_remove,
};

module_platform_driver(ehl_pse_gpio_driver);

MODULE_AUTHOR("N. Pandith <pandith.n@intel.com>");
MODULE_DESCRIPTION("Intel Elkhart Lake PSE GPIO driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gpio-ehl-pse");
