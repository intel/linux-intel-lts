// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay Timer driver
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <linux/bitops.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "timer-of.h"

/* Timer register offset */
#define TIM_CNT_VAL_OFFSET		0x0
#define TIM_RELOAD_VAL_OFFSET		0x4
#define TIM_CONFIG_OFFSET		0x8

/* Bit fields of timer general config register */
#define TIM_CONFIG_PRESCALER_ENABLE	BIT(2)
#define TIM_CONFIG_COUNTER_ENABLE	BIT(0)

/* Bit fields of timer config register */
#define TIM_CONFIG_INTERRUPT_PENDING	BIT(4)
#define TIM_CONFIG_INTERRUPT_ENABLE	BIT(2)
#define TIM_CONFIG_RESTART		BIT(1)
#define TIM_CONFIG_ENABLE		BIT(0)

#define TIM_GEN_MASK			GENMASK(31, 12)
#define TIM_RATING			200
#define TIM_CLKSRC_MASK_BITS		64

#define TIMER_NAME_SIZE			25

/* Provides a unique ID for each timer */
static DEFINE_IDA(keembay_timer_ida);

static inline void keembay_timer_enable(void __iomem *base, u32 flags)
{
	writel(TIM_CONFIG_ENABLE | flags, base + TIM_CONFIG_OFFSET);
}

static inline void keembay_timer_disable(void __iomem *base)
{
	writel(0x0, base + TIM_CONFIG_OFFSET);
}

static inline void keembay_timer_update_counter(void __iomem *base, u32 val)
{
	writel(val, base + TIM_CNT_VAL_OFFSET);
	writel(val, base + TIM_RELOAD_VAL_OFFSET);
}

static inline void keembay_timer_clear_pending_int(void __iomem *base)
{
	u32 val;

	val = readl(base + TIM_CONFIG_OFFSET);
	val &= ~TIM_CONFIG_INTERRUPT_PENDING;
	writel(val, base + TIM_CONFIG_OFFSET);
}

static int keembay_timer_set_next_event(unsigned long evt, struct clock_event_device *ce)
{
	u32 flags = TIM_CONFIG_INTERRUPT_ENABLE;
	struct timer_of *to = to_timer_of(ce);
	void __iomem *tim_base = timer_of_base(to);

	keembay_timer_disable(tim_base);
	keembay_timer_update_counter(tim_base, evt);
	keembay_timer_enable(tim_base, flags);

	return 0;
}

static int keembay_timer_periodic(struct clock_event_device *ce)
{
	u32 flags = TIM_CONFIG_INTERRUPT_ENABLE | TIM_CONFIG_RESTART;
	struct timer_of *to = to_timer_of(ce);
	void __iomem *tim_base = timer_of_base(to);

	keembay_timer_disable(tim_base);
	keembay_timer_update_counter(tim_base, timer_of_period(to));
	keembay_timer_enable(tim_base, flags);

	return 0;
}

static int keembay_timer_shutdown(struct clock_event_device *ce)
{
	struct timer_of *to = to_timer_of(ce);

	keembay_timer_disable(timer_of_base(to));

	return 0;
}

static irqreturn_t keembay_timer_isr(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	struct timer_of *to = to_timer_of(evt);
	void __iomem *tim_base = timer_of_base(to);
	u32 val;

	val = readl(tim_base + TIM_CONFIG_OFFSET);

	if (val & TIM_CONFIG_RESTART) {
		/* Clear interrupt for periodic timer*/
		keembay_timer_clear_pending_int(tim_base);
	} else {
		/* Disable the timer for one shot timer */
		keembay_timer_disable(tim_base);
	}

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static int __init keembay_clockevent_init(struct device_node *np)
{
	struct device_node *gpt_node = np->parent;
	struct timer_of *keembay_ce_to;
	void __iomem *gpt_base;
	char *timer_name;
	int timer_id;
	int ret;
	u32 val;

	gpt_base = of_iomap(gpt_node, 0);
	if (!gpt_base) {
		pr_err("%pOF: Failed to get general config base address\n", np);
		return -ENXIO;
	}

	/* Prescaler must be enabled for the timer to operate */
	val = readl(gpt_base + TIM_CONFIG_OFFSET);
	if (!(val & TIM_CONFIG_PRESCALER_ENABLE)) {
		pr_err("%pOF: Prescaler is not enabled\n", np);
		ret = -ENODEV;
		goto err_iounmap;
	}

	keembay_ce_to = kzalloc(sizeof(*keembay_ce_to), GFP_KERNEL);
	if (!keembay_ce_to) {
		ret = -ENOMEM;
		goto err_iounmap;
	}

	timer_id = ida_alloc(&keembay_timer_ida, GFP_KERNEL);
	if (timer_id < 0) {
		ret = timer_id;
		goto err_keembay_ce_to_free;
	}

	timer_name = kasprintf(GFP_KERNEL, "keembay_timer%d", timer_id);
	if (!timer_name) {
		ret = -ENOMEM;
		goto err_free_ida;
	}

	keembay_ce_to->flags = TIMER_OF_IRQ | TIMER_OF_BASE | TIMER_OF_CLOCK;
	keembay_ce_to->clkevt.name = timer_name;
	keembay_ce_to->clkevt.cpumask = cpumask_of(0);
	keembay_ce_to->clkevt.features = CLOCK_EVT_FEAT_PERIODIC |
					 CLOCK_EVT_FEAT_ONESHOT  |
					 CLOCK_EVT_FEAT_DYNIRQ;
	keembay_ce_to->clkevt.rating = TIM_RATING;
	keembay_ce_to->clkevt.set_next_event = keembay_timer_set_next_event;
	keembay_ce_to->clkevt.set_state_periodic = keembay_timer_periodic;
	keembay_ce_to->clkevt.set_state_shutdown = keembay_timer_shutdown;
	keembay_ce_to->of_irq.handler = keembay_timer_isr;
	keembay_ce_to->of_irq.flags = IRQF_TIMER;

	ret = timer_of_init(np, keembay_ce_to);
	if (ret)
		goto err_timer_name_free;

	val = readl(gpt_base + TIM_RELOAD_VAL_OFFSET);
	iounmap(gpt_base);

	keembay_ce_to->of_clk.rate = keembay_ce_to->of_clk.rate / (val + 1);

	clockevents_config_and_register(&keembay_ce_to->clkevt,
					timer_of_rate(keembay_ce_to),
					1,
					U32_MAX);

	return 0;

err_timer_name_free:
	kfree(timer_name);
err_free_ida:
	ida_free(&keembay_timer_ida, timer_id);
err_keembay_ce_to_free:
	kfree(keembay_ce_to);
err_iounmap:
	iounmap(gpt_base);

	return ret;
}

static struct timer_of keembay_cs_to = {
	.flags	= TIMER_OF_BASE | TIMER_OF_CLOCK,
};

static u64 notrace keembay_clocksource_read(struct clocksource *cs)
{
	return lo_hi_readq(timer_of_base(&keembay_cs_to));
}

static struct clocksource keembay_counter = {
	.name	= "keembay_sys_counter",
	.rating	= TIM_RATING,
	.read	= keembay_clocksource_read,
	.mask	= CLOCKSOURCE_MASK(TIM_CLKSRC_MASK_BITS),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS |
		  CLOCK_SOURCE_SUSPEND_NONSTOP,
};

static int __init keembay_clocksource_init(struct device_node *np)
{
	struct device_node *gpt_node = np->parent;
	void __iomem *gpt_base;
	u32 val;
	int ret;

	gpt_base = of_iomap(gpt_node, 0);
	if (!gpt_base) {
		pr_err("%pOF: Failed to get general config base address\n", np);
		return -ENXIO;
	}

	/* Free Running Counter must be enabled */
	val = readl(gpt_base + TIM_CONFIG_OFFSET);
	iounmap(gpt_base);
	if (!(val & TIM_CONFIG_COUNTER_ENABLE)) {
		pr_err("%pOF: free running counter is not enabled\n", np);
		return -ENODEV;
	}

	ret = timer_of_init(np, &keembay_cs_to);
	if (ret)
		return ret;

	return clocksource_register_hz(&keembay_counter, timer_of_rate(&keembay_cs_to));
}

TIMER_OF_DECLARE(keembay_clockevent, "intel,keembay-timer", keembay_clockevent_init);
TIMER_OF_DECLARE(keembay_clocksource, "intel,keembay-counter", keembay_clocksource_init);
