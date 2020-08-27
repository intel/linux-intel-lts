// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay Timer driver
 * Copyright (C) 2019 Intel Corporation
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include "timer-of.h"

#define TIM_BASE_ADDRESS		0x20330000

/* Registers offset */
#define TIM_SCALER_VAL_OFFSET		0x1000
#define TIM_SCALER_RELOAD_OFFSET	0x1004
#define TIM_GEN_CONFIG_OFFSET		0x1008

#define TIM_CNT_VAL_OFFSET		0x0
#define TIM_RELOAD_VAL_OFFSET		0x4
#define TIM_CONFIG_OFFSET		0x8

#define TIM_FREE_CNT0			0x1090
#define TIM_FREE_CNT1			0x1094
#define TIM_FREE_CNT2			0x00e8
#define TIM_FREE_CNT3			0x00ec

/* Bit fields of TIM_GEN_CONFIG register */
#define TIM_GEN_CONFIG_PRESCALER_ENABLE	BIT(2)
#define TIM_GEN_CONFIG_COUNTER_ENABLE	BIT(0)

/* Bit fields of TIM_CONFIG registers */
#define TIM_CONFIG_FORCED_RELOAD	BIT(5)
#define TIM_CONFIG_INTERRUPT_PENDING	BIT(4)
#define TIM_CONFIG_CHAIN		BIT(3)
#define TIM_CONFIG_INTERRUPT_ENABLE	BIT(2)
#define TIM_CONFIG_RESTART		BIT(1)
#define TIM_CONFIG_ENABLE		BIT(0)

static DEFINE_SPINLOCK(keembay_clocksource_lock);

static inline void keembay_timer_disable(void __iomem *base)
{
	writel(0x0, base + TIM_CONFIG_OFFSET);
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
	struct timer_of *to = to_timer_of(ce);
	u32 flags = TIM_CONFIG_INTERRUPT_ENABLE;

	/* setup and enable oneshot timer */
	keembay_timer_disable(timer_of_base(to));
	keembay_timer_update_counter(timer_of_base(to), evt);
	keembay_timer_enable(timer_of_base(to), flags);

	return 0;
}

static int keembay_timer_periodic(struct clock_event_device *ce)
{
	struct timer_of *to = to_timer_of(ce);
	u32 flags = TIM_CONFIG_INTERRUPT_ENABLE | TIM_CONFIG_RESTART;

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
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;
	struct timer_of *to = to_timer_of(evt);
	u32 val;

	/* clear interrupt */
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
		.rating			= 200,
		.set_next_event		= keembay_timer_set_next_event,
		.set_state_periodic	= keembay_timer_periodic,
		.set_state_shutdown	= keembay_timer_shutdown,
	},

	.of_irq = {
		.handler = keembay_timer_isr,
		.flags = IRQF_TIMER | IRQF_IRQPOLL,
	},
};

static int __init keembay_timer_init(struct device_node *np)
{
	void __iomem *reg;
	u32 val;
	int ret;

	reg = ioremap(TIM_BASE_ADDRESS + TIM_SCALER_VAL_OFFSET, 0xc);

	/* prescaler must be enabled for the timer to operate */
	val = readl(reg - TIM_SCALER_VAL_OFFSET + TIM_GEN_CONFIG_OFFSET);
	if (!(val & TIM_GEN_CONFIG_PRESCALER_ENABLE)) {
		iounmap(reg);
		pr_err("%s: prescaler in not enabled\n", __func__);
		return -ENODEV;
	}

	iounmap(reg);

	ret = timer_of_init(np, &keembay_ce_to);
	if (ret)
		return ret;

	/*
	 * Clock input is divided by PRESCALER + 1 before it is fed
	 * to the counter
	 */
	val = readl(reg - TIM_SCALER_VAL_OFFSET + TIM_SCALER_RELOAD_OFFSET);
	keembay_ce_to.of_clk.rate = keembay_ce_to.of_clk.rate / (val + 1);

	keembay_timer_disable(timer_of_base(&keembay_ce_to));

	keembay_ce_to.clkevt.cpumask = cpumask_of(0);
	clockevents_config_and_register(&keembay_ce_to.clkevt,
					timer_of_rate(&keembay_ce_to), 0x1,
					0xffffffff);

	return 0;
}

static struct timer_of keembay_cs_to = {
	.flags	= TIMER_OF_BASE | TIMER_OF_CLOCK,
};

static u64 notrace keembay_clocksource_read(struct clocksource *cs)
{
	u64 val_hi, val_lo, val_lo1;
	unsigned long flags;

	spin_lock_irqsave(&keembay_clocksource_lock, flags);

	/* read lower 32-bit first, then read upper 32-bit next */
	val_lo1 = readl(timer_of_base(&keembay_cs_to));
	val_hi = readl(timer_of_base(&keembay_cs_to) + 4);
	val_lo = readl(timer_of_base(&keembay_cs_to));

	/* check for rollover, reread upper 32-bit if rollover */
	if (val_lo < val_lo1)
		val_hi = readl(timer_of_base(&keembay_cs_to) + 4);

	spin_unlock_irqrestore(&keembay_clocksource_lock, flags);

	return ((val_hi << 32) | (val_lo & 0xffffffff));
}

static struct clocksource keembay_counter = {
	.name			= "keembay_sys_counter",
	.rating			= 200,
	.read			= keembay_clocksource_read,
	.mask			= CLOCKSOURCE_MASK(64),
	.flags			= CLOCK_SOURCE_IS_CONTINUOUS |
				  CLOCK_SOURCE_SUSPEND_NONSTOP,
};

static int __init keembay_counter_init(struct device_node *np)
{
	int ret;
	int val;
	void __iomem *reg;

	reg = ioremap(TIM_BASE_ADDRESS + TIM_SCALER_VAL_OFFSET, 0xc);

	/* free running counter must be enabled */
	val = readl(reg - TIM_SCALER_VAL_OFFSET + TIM_GEN_CONFIG_OFFSET);
	if (!(val & TIM_GEN_CONFIG_COUNTER_ENABLE)) {
		iounmap(reg);
		pr_err("%s: free running counter is not enabled\n", __func__);
		return -ENODEV;
	}

	iounmap(reg);

	ret = timer_of_init(np, &keembay_cs_to);
	if (ret)
		return ret;

	if (of_device_is_compatible(np, "intel,keembay-counter"))
		keembay_counter.name = "keembay_sys_counter";

	ret = clocksource_register_hz(&keembay_counter,
				      timer_of_rate(&keembay_cs_to));
	if (ret)
		return ret;

	return 0;
}

TIMER_OF_DECLARE(keembay_timer, "intel,keembay-timer", keembay_timer_init);
TIMER_OF_DECLARE(keembay_sys_counter, "intel,keembay-counter",
		 keembay_counter_init);
