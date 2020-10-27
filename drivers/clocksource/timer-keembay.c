// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay Timer driver
 *
 * Copyright (C) 2020 Intel Corporation
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/sizes.h>

#include "timer-of.h"

/* Registers offset */
#define TIM_CNT_VAL_OFFSET		0
#define TIM_RELOAD_VAL_OFFSET		SZ_4
#define TIM_CONFIG_OFFSET		SZ_8

/* Bit fields of TIM_GEN_CONFIG register */
#define TIM_CONFIG_PRESCALER_ENABLE	BIT(2)

/* Bit fields of TIM_CONFIG registers */
#define TIM_CONFIG_INTERRUPT_PENDING	BIT(4)
#define TIM_CONFIG_INTERRUPT_ENABLE	BIT(2)
#define TIM_CONFIG_RESTART		BIT(1)
#define TIM_CONFIG_ENABLE		BIT(0)

#define TIM_RATING			200
#define TIM_CLKSRC_BITS			SZ_64

struct keembay_init_data {
	struct timer_of	*cfg;
	void __iomem	*base;
	u32		mask;
};

static inline void keembay_timer_disable(void __iomem *base)
{
	writel(0, base + TIM_CONFIG_OFFSET);
}

static inline void keembay_timer_enable(void __iomem *base, u32 flags)
{
	writel(TIM_CONFIG_ENABLE | flags, base + TIM_CONFIG_OFFSET);
}

static inline void keembay_timer_update_counter(void __iomem *base, u32 val)
{
	writel(val, base + TIM_CNT_VAL_OFFSET);
	writel(val, base + TIM_RELOAD_VAL_OFFSET);
}

static int keembay_timer_set_next_event(unsigned long evt,
					struct clock_event_device *ce)
{
	u32 flags = TIM_CONFIG_INTERRUPT_ENABLE;
	struct timer_of *to = to_timer_of(ce);

	/* setup and enable oneshot timer */
	keembay_timer_disable(timer_of_base(to));
	keembay_timer_update_counter(timer_of_base(to), evt);
	keembay_timer_enable(timer_of_base(to), flags);

	return 0;
}

static int keembay_timer_periodic(struct clock_event_device *ce)
{
	u32 flags = TIM_CONFIG_INTERRUPT_ENABLE | TIM_CONFIG_RESTART;
	struct timer_of *to = to_timer_of(ce);

	/* setup and enable periodic timer */
	keembay_timer_disable(timer_of_base(to));
	keembay_timer_update_counter(timer_of_base(to), timer_of_period(to));
	keembay_timer_enable(timer_of_base(to), flags);

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
	u32 val;

	val = readl(timer_of_base(to) + TIM_CONFIG_OFFSET);
	val &= ~TIM_CONFIG_INTERRUPT_PENDING;
	writel(val, timer_of_base(to) + TIM_CONFIG_OFFSET);

	if (clockevent_state_oneshot(evt))
		keembay_timer_disable(timer_of_base(to));

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct timer_of keembay_ce_to = {
	.flags	= TIMER_OF_IRQ | TIMER_OF_BASE | TIMER_OF_CLOCK,
	.clkevt = {
		.name			= "keembay_timer",
		.features		= CLOCK_EVT_FEAT_PERIODIC |
					  CLOCK_EVT_FEAT_ONESHOT,
		.rating			= TIM_RATING,
		.set_next_event		= keembay_timer_set_next_event,
		.set_state_periodic	= keembay_timer_periodic,
		.set_state_shutdown	= keembay_timer_shutdown,
	},
	.of_irq = {
		.handler = keembay_timer_isr,
		.flags = IRQF_TIMER | IRQF_IRQPOLL,
	},
};

static int __init keembay_init_pre(struct device_node *np,
				   struct keembay_init_data *data)
{
	u32 val;
	int ret;

	data->base = of_iomap(np, 1);
	if (!data->base)
		return -ENXIO;

	val = readl(data->base + TIM_CONFIG_OFFSET);
	if (!(val & data->mask)) {
		iounmap(data->base);
		return -ENODEV;
	}

	ret = timer_of_init(np, data->cfg);
	if (ret)
		iounmap(data->base);

	return ret;
}

static int __init keembay_timer_init(struct device_node *np)
{
	struct keembay_init_data data;
	u32 val;
	int ret;

	data.mask = TIM_CONFIG_PRESCALER_ENABLE;
	data.cfg = &keembay_ce_to;
	ret = keembay_init_pre(np, &data);
	if (ret)
		return ret;

	val = readl(data.base + TIM_RELOAD_VAL_OFFSET);
	keembay_ce_to.of_clk.rate = keembay_ce_to.of_clk.rate / (val + 1);

	keembay_timer_disable(timer_of_base(&keembay_ce_to));

	keembay_ce_to.clkevt.cpumask = cpumask_of(0);
	clockevents_config_and_register(&keembay_ce_to.clkevt,
					timer_of_rate(&keembay_ce_to), 1,
					U32_MAX);
	return 0;
}

static struct timer_of keembay_cs_to = {
	.flags	= TIMER_OF_BASE | TIMER_OF_CLOCK,
};

static u64 notrace keembay_clocksource_read(struct clocksource *cs)
{
	return lo_hi_readq(timer_of_base(&keembay_cs_to));
}

static struct clocksource keembay_counter = {
	.name			= "keembay_sys_counter",
	.rating			= TIM_RATING,
	.read			= keembay_clocksource_read,
	.mask			= CLOCKSOURCE_MASK(TIM_CLKSRC_BITS),
	.flags			= CLOCK_SOURCE_IS_CONTINUOUS |
				  CLOCK_SOURCE_SUSPEND_NONSTOP,
};

static int __init keembay_counter_init(struct device_node *np)
{
	struct keembay_init_data data;
	int ret;

	data.mask = TIM_CONFIG_ENABLE;
	data.cfg = &keembay_cs_to;
	ret = keembay_init_pre(np, &data);
	if (ret)
		return ret;

	if (of_device_is_compatible(np, "intel,keembay-counter"))
		keembay_counter.name = "keembay_sys_counter";

	return clocksource_register_hz(&keembay_counter,
				       timer_of_rate(&keembay_cs_to));
}

TIMER_OF_DECLARE(keembay_timer, "intel,keembay-timer", keembay_timer_init);
TIMER_OF_DECLARE(keembay_sys_counter, "intel,keembay-counter",
		 keembay_counter_init);
