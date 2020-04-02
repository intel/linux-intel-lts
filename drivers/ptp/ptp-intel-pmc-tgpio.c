// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Timed GPIO Controller Driver
 *
 * Copyright (C) 2022 Intel Corporation
 * Author: Christopher Hall <christopher.s.hall@intel.com>
 */

#include <asm/tsc.h>
#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>

#define TGPIOCTL		0x00
#define TGPIOCOMPV31_0		0x10
#define TGPIOCOMPV63_32		0x14
#define TGPIOPIV31_0		0x18
#define TGPIOPIV63_32		0x1c
#define TGPIOTCV31_0		0x20
#define TGPIOTCV63_32		0x24
#define TGPIOECCV31_0		0x28
#define TGPIOECCV63_32		0x2c
#define TGPIOEC31_0		0x30
#define TGPIOEC63_32		0x34

/* Control Register */
#define TGPIOCTL_EN			BIT(0)
#define TGPIOCTL_DIR			BIT(1)
#define TGPIOCTL_EP			GENMASK(3, 2)
#define TGPIOCTL_EP_RISING_EDGE		(0 << 2)
#define TGPIOCTL_EP_FALLING_EDGE	(1 << 2)
#define TGPIOCTL_EP_TOGGLE_EDGE		(2 << 2)
#define TGPIOCTL_PM			BIT(4)

#define DRIVER_NAME		"intel-pmc-tgpio"
#define MAX_PINS_PER_PLAT	(2)

typedef char *plat_acpi_resource[MAX_PINS_PER_PLAT];

/* Each row represent a platform that supports PMC TGPIO */
static const plat_acpi_resource acpi_plat_map[] = {
	{ "INTC1021", "INTC1022" },
	{ "INTC1023", "INTC1024" },
};

#define PLATFORM_COUNT ARRAY_SIZE(acpi_plat_map)

struct intel_pmc_tgpio_t {
	struct mutex lock;
	struct ptp_clock_info info;
	struct ptp_clock *clock;
	const plat_acpi_resource *plat_res;

	struct intel_pmc_tgpio_pin {
		void __iomem *base;
		bool busy;
		struct completion transact_comp;

		ktime_t	curr_ns;
		u64 curr_art;

		struct dentry *root;
		struct debugfs_regset32 *regset;
		struct platform_device *pdev;
	} pin[MAX_PINS_PER_PLAT];
};

static struct intel_pmc_tgpio_t *intel_pmc_tgpio;

#define to_intel_pmc_tgpio(i) \
	(container_of((i), struct intel_pmc_tgpio_t, info))

#define ts64_to_ptp_clock_time(x) ((struct ptp_clock_time){.sec = (x).tv_sec, \
			.nsec = (x).tv_nsec})

#define ptp_clock_time_to_ts64(x) ((struct timespec64){.tv_sec = (x).sec, \
						       .tv_nsec = (x).nsec})

static inline u32 intel_pmc_tgpio_readl(struct intel_pmc_tgpio_t *tgpio,
					u32 offset,
					unsigned int index)
{
	return readl(tgpio->pin[index].base + offset);
}

static inline void intel_pmc_tgpio_writel(struct intel_pmc_tgpio_t *tgpio,
					  u32 offset,
					  unsigned int index, u32 value)
{
	writel(value, tgpio->pin[index].base + offset);
}

#define INTEL_PMC_TGPIO_RD_REG(offset, index)			\
	(intel_pmc_tgpio_readl((tgpio), (offset), (index)))
#define INTEL_PMC_TGPIO_WR_REG(offset, index, value)		\
	(intel_pmc_tgpio_writel((tgpio), (offset), (index), (value)))

static const struct debugfs_reg32 intel_pmc_tgpio_regs[] = {
	{
		.name = "TGPIOCTL",
		.offset = TGPIOCTL
	},
	{
		.name = "TGPIOCOMPV31_0",
		.offset = TGPIOCOMPV31_0
	},
	{
		.name = "TGPIOCOMPV63_32",
		.offset = TGPIOCOMPV63_32
	},
	{
		.name = "TGPIOPIV31_0",
		.offset = TGPIOPIV31_0
	},
	{
		.name = "TGPIOPIV63_32",
		.offset = TGPIOPIV63_32
	},
	{
		.name = "TGPIOECCV31_0",
		.offset = TGPIOECCV31_0
	},
	{
		.name = "TGPIOECCV63_32",
		.offset = TGPIOECCV63_32
	},
	{
		.name = "TGPIOEC31_0",
		.offset = TGPIOEC31_0
	},
	{
		.name = "TGPIOEC63_32",
		.offset = TGPIOEC63_32
	},
};

static int intel_pmc_tgpio_gettime64(struct ptp_clock_info *info,
		struct timespec64 *ts)
{
	*ts = get_tsc_ns_now(NULL);

	return 0;
}

static int intel_pmc_tgpio_settime64(struct ptp_clock_info *info,
		const struct timespec64 *ts)
{
	return -EOPNOTSUPP;
}

static void intel_pmc_tgpio_pre_config(struct intel_pmc_tgpio_t *tgpio,
				       unsigned int index,
				       unsigned int ctrl,
				       unsigned int ctrl_new,
				       int *enable_toggle)
{
	if (ctrl_new != ctrl) {
		INTEL_PMC_TGPIO_WR_REG(TGPIOCTL, index, ctrl);
		INTEL_PMC_TGPIO_WR_REG(TGPIOCTL, index, ctrl_new);
		*enable_toggle = *enable_toggle > 0 ? 2 : -1;
	}
}

static void intel_pmc_tgpio_post_config(struct intel_pmc_tgpio_t *tgpio,
					unsigned int index,
					unsigned int ctrl,
					int enable_toggle)
{
	if (enable_toggle > 0)
		ctrl |= TGPIOCTL_EN;
	if (enable_toggle > 1 || enable_toggle < -1)
		INTEL_PMC_TGPIO_WR_REG(TGPIOCTL, index, ctrl);
}

static int intel_pmc_tgpio_config_input(struct intel_pmc_tgpio_t *tgpio,
					struct ptp_extts_request *extts,
					int on)
{
	u32 ctrl, ctrl_new;
	int enable_toggle;

	/*
	 * enable_toggle meaning:
	 *
	 *	-2: Change from enabled state to disabled state
	 *	-1: Keep disabled state
	 *	 1: Keep enabled state
	 *	 2: Change from disabled state to enabled state
	 */
	ctrl = INTEL_PMC_TGPIO_RD_REG(TGPIOCTL, extts->index);
	enable_toggle = ctrl & TGPIOCTL_EN ? 1 : -1;
	ctrl &= ~TGPIOCTL_EN;
	ctrl_new = ctrl;

	if (on) {
		int rising_cap, falling_cap;

		ctrl_new &= ~TGPIOCTL_EP;

		rising_cap = extts->flags & PTP_RISING_EDGE;
		falling_cap = extts->flags & PTP_FALLING_EDGE;

		/* By default capture rising and falling edges */
		if (rising_cap && !falling_cap)
			ctrl_new |= TGPIOCTL_EP_RISING_EDGE;
		else if (!rising_cap && falling_cap)
			ctrl_new |= TGPIOCTL_EP_FALLING_EDGE;
		else
			ctrl_new |= TGPIOCTL_EP_TOGGLE_EDGE;

		ctrl_new |= TGPIOCTL_DIR;
		enable_toggle = enable_toggle < 0 ? 2 : 1;
	} else {
		enable_toggle = enable_toggle > 0 ? -2 : -1;
	}

	intel_pmc_tgpio_pre_config
		(tgpio, extts->index, ctrl, ctrl_new, &enable_toggle);
	intel_pmc_tgpio_post_config
		(tgpio, extts->index, ctrl_new, enable_toggle);

	return 0;
}

#define MAX_ATOMIC_PERIOD		((u32)-1/*cycles*/)
#define MIN_OUTPUT_PERIOD		(3/*cycles*/)
#define FREQ_CHANGE_BLACKOUT_THRESH	((NSEC_PER_SEC/1000)*4/*ms*/)
#define FREQ_CHANGE_SLEEP_MIN		(20/*us*/)
#define FREQ_CHANGE_SLEEP_WINDOW	(FREQ_CHANGE_SLEEP_MIN*5)

static int _intel_pmc_tgpio_config_output(struct intel_pmc_tgpio_t *tgpio,
					  unsigned int index,
					  struct timespec64 new_start_ns,
					  struct timespec64 new_period_ns,
					  unsigned int flags,
					  int on)
{
	u32 ctrl, ctrl_new;
	u64 new_period;
	int enable_toggle;
	bool enable_preempt = false;

	/*
	 * enable_toggle meaning:
	 *
	 *	-2: Change from enabled state to disabled state
	 *	-1: Keep disabled state
	 *	 1: Keep enabled state
	 *	 2: Change from disabled state to enabled state
	 */
	ctrl = INTEL_PMC_TGPIO_RD_REG(TGPIOCTL, index);
	enable_toggle = ctrl & TGPIOCTL_EN ? 1 : -1;
	ctrl &= ~TGPIOCTL_EN;
	ctrl_new = ctrl;

	/* Frequency adjustment flag is only valid if we're already running */
	if (flags & PTP_PEROUT_FREQ_ADJ &&
	   (enable_toggle != 1 || !(ctrl & TGPIOCTL_PM)))
		return -EINVAL;

	new_period = convert_tsc_ns_to_art_duration(&new_period_ns);

	/* Don't use a period less than the minimum */
	if (!(flags & PTP_PEROUT_ONE_SHOT) && new_period < MIN_OUTPUT_PERIOD)
		return -EINVAL;

	/*
	 * In the unlikely case of an adjustment from a small period (< 4ms)
	 * to a large period (>>4 sec) change to a "transitory" period first
	 */
	if (flags & PTP_PEROUT_FREQ_ADJ &&
	    tgpio->pin[index].curr_ns < FREQ_CHANGE_BLACKOUT_THRESH &&
	    new_period >> 32) {
		struct timespec64 transit_period_ns;
		int err;

		transit_period_ns =
			convert_art_to_tsc_ns_duration(MAX_ATOMIC_PERIOD);
		/* If transit period is too small we recurse infinitely */
		if (timespec64_to_ktime(transit_period_ns) <
		    FREQ_CHANGE_BLACKOUT_THRESH)
			return -EINVAL;
		err = _intel_pmc_tgpio_config_output
			(tgpio, index, (struct timespec64){0, 0},
			 transit_period_ns, flags, on);
		if (err)
			return err;
	}

	/* Don't make a change to/from 64 bit period across an output edge */
	if (flags & PTP_PEROUT_FREQ_ADJ &&
	    tgpio->pin[index].curr_ns >= FREQ_CHANGE_BLACKOUT_THRESH &&
	    (new_period >> 32 || tgpio->pin[index].curr_art >> 32)) {
		u64 next_edge;
		u32 next_edge_lo1, next_edge_lo2;
		struct timespec64 next_edge_tsc;
		struct timespec64 tsc_now, tsc_tmp;
		ktime_t delta;

		preempt_disable();
		enable_preempt = true;

		/* Calculate time delta until next edge */
		tsc_tmp = get_tsc_ns_now(NULL);
		next_edge_lo2 =
			INTEL_PMC_TGPIO_RD_REG(TGPIOCOMPV31_0, index);
		do {
			tsc_now = tsc_tmp;
			next_edge_lo1 = next_edge_lo2;
			next_edge =
				INTEL_PMC_TGPIO_RD_REG(TGPIOCOMPV63_32, index);
			tsc_tmp = get_tsc_ns_now(NULL);
			next_edge_lo2 =
				INTEL_PMC_TGPIO_RD_REG(TGPIOCOMPV31_0, index);
		} while (next_edge_lo1 != next_edge_lo2);
		next_edge <<= 32;
		next_edge |= next_edge_lo1;

		next_edge_tsc = convert_art_to_tsc_ns(next_edge);
		delta = timespec64_to_ktime
			(timespec64_sub(next_edge_tsc, tsc_now));

		/* If there's a chance our write will get stepped on, wait */
		if (delta < FREQ_CHANGE_BLACKOUT_THRESH) {
			unsigned long		min, max;

			min = delta;
			min /= 1000;
			max = min + FREQ_CHANGE_SLEEP_WINDOW;
			min = min < FREQ_CHANGE_SLEEP_MIN ?
				FREQ_CHANGE_SLEEP_MIN : min;
			usleep_range(min, max);
		}
	}

	if (on || flags & PTP_PEROUT_ONE_SHOT) {
		/* We only use toggle edge */
		ctrl_new &= ~TGPIOCTL_EP;
		ctrl_new |= TGPIOCTL_EP_TOGGLE_EDGE;
		ctrl_new &= ~TGPIOCTL_DIR;

		if (flags & PTP_PEROUT_ONE_SHOT)
			ctrl_new &= ~TGPIOCTL_PM;
		else
			ctrl_new |= TGPIOCTL_PM;

		enable_toggle = enable_toggle < 0 ?  2 :  1;
	} else {
		enable_toggle = enable_toggle > 0 ? -2 : -1;
	}

	intel_pmc_tgpio_pre_config
		(tgpio, index, ctrl, ctrl_new, &enable_toggle);

	if (enable_toggle >= 0 && !(flags & PTP_PEROUT_FREQ_ADJ)) {
		u64 new_start = convert_tsc_ns_to_art(&new_start_ns);

		INTEL_PMC_TGPIO_WR_REG
			(TGPIOCOMPV63_32, index, new_start >> 32);
		INTEL_PMC_TGPIO_WR_REG
			(TGPIOCOMPV31_0, index, new_start & 0xFFFFFFFF);
	}

	if (enable_toggle >= 0 && !(flags & PTP_PEROUT_ONE_SHOT)) {
		INTEL_PMC_TGPIO_WR_REG
			(TGPIOPIV31_0, index, new_period & 0xFFFFFFFF);
		INTEL_PMC_TGPIO_WR_REG
			(TGPIOPIV63_32, index, new_period >> 32);
		if (enable_preempt)
			preempt_enable();
		tgpio->pin[index].curr_ns = timespec64_to_ktime(new_period_ns);
		tgpio->pin[index].curr_art = new_period;
	}

	intel_pmc_tgpio_post_config(tgpio, index, ctrl_new, enable_toggle);

	return 0;
}

static int intel_pmc_tgpio_config_output(struct intel_pmc_tgpio_t *tgpio,
					 struct ptp_perout_request *perout,
					 int on)
{
	struct timespec64 new_start_ns;
	struct timespec64 new_period_ns;

	if (on || perout->flags & PTP_PEROUT_ONE_SHOT) {
		new_start_ns = ptp_clock_time_to_ts64(perout->start);
		new_period_ns = ptp_clock_time_to_ts64(perout->period);
		new_period_ns = ktime_to_timespec64
		  (ktime_divns(timespec64_to_ktime(new_period_ns), 2));
	}

	return _intel_pmc_tgpio_config_output
		(tgpio, perout->index, new_start_ns, new_period_ns,
		 perout->flags, on);
}

static int intel_pmc_tgpio_enable(struct ptp_clock_info *info,
				  struct ptp_clock_request *req,
				  int on)
{
	struct intel_pmc_tgpio_t *tgpio = to_intel_pmc_tgpio(info);
	int ret = -EOPNOTSUPP;
	unsigned int index;

	switch (req->type) {
	case PTP_CLK_REQ_EXTTS:
		index = req->extts.index;
		break;
	case PTP_CLK_REQ_PEROUT:
		index = req->perout.index;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&intel_pmc_tgpio->lock);
	while (intel_pmc_tgpio->pin[index].busy) {
		mutex_unlock(&intel_pmc_tgpio->lock);
		wait_for_completion
			(&intel_pmc_tgpio->pin[index].transact_comp);
		mutex_lock(&intel_pmc_tgpio->lock);
	}
	intel_pmc_tgpio->pin[index].busy = true;
	mutex_unlock(&intel_pmc_tgpio->lock);

	if (intel_pmc_tgpio->pin[index].base != NULL) {
		switch (req->type) {
		case PTP_CLK_REQ_EXTTS:
			ret = intel_pmc_tgpio_config_input
				(tgpio, &req->extts, on);
			break;
		case PTP_CLK_REQ_PEROUT:
			ret = intel_pmc_tgpio_config_output
				(tgpio, &req->perout, on);
			break;
		default:
			ret = -EINVAL;
			break;
		}
	} else {
		ret = -ENODEV;
	}

	mutex_lock(&intel_pmc_tgpio->lock);
	intel_pmc_tgpio->pin[index].busy = false;
	complete(&intel_pmc_tgpio->pin[index].transact_comp);
	mutex_unlock(&intel_pmc_tgpio->lock);

	return ret;
}


static int intel_pmc_tgpio_get_time_fn(ktime_t *device_time,
				       struct system_counterval_t *system_counter,
				       void *_tgpio)
{
	struct timespec64 now_ns;

	now_ns = get_tsc_ns_now(system_counter);
	*device_time = timespec64_to_ktime(now_ns);
	return 0;
}

static int intel_pmc_tgpio_getcrosststamp(struct ptp_clock_info *info,
					  struct system_device_crosststamp *cts)
{
	struct intel_pmc_tgpio_t *tgpio = to_intel_pmc_tgpio(info);

	return get_device_system_crosststamp
		(intel_pmc_tgpio_get_time_fn, tgpio, NULL, cts);
}

static int intel_pmc_tgpio_counttstamp(struct ptp_clock_info *info,
				       struct ptp_event_count_tstamp *count)
{
	struct intel_pmc_tgpio_t *tgpio = to_intel_pmc_tgpio(info);
	u32 dt_hi_s;
	u32 dt_hi_e;
	u32 dt_lo;
	struct timespec64 dt_ts;
	struct timespec64 tsc_now;

	mutex_lock(&intel_pmc_tgpio->lock);
	while (intel_pmc_tgpio->pin[count->index].busy) {
		mutex_unlock(&intel_pmc_tgpio->lock);
		wait_for_completion
			(&intel_pmc_tgpio->pin[count->index].transact_comp);
		mutex_lock(&intel_pmc_tgpio->lock);
	}
	intel_pmc_tgpio->pin[count->index].busy = true;
	mutex_unlock(&intel_pmc_tgpio->lock);

	tsc_now = get_tsc_ns_now(NULL);
	dt_hi_s = convert_tsc_ns_to_art(&tsc_now) >> 32;

	/* Reading lower 32-bit word of Time Capture Value (TCV) loads */
	/* the event time and event count capture */
	dt_lo = INTEL_PMC_TGPIO_RD_REG(TGPIOTCV31_0, count->index);
	count->event_count =
		INTEL_PMC_TGPIO_RD_REG(TGPIOECCV63_32, count->index);
	count->event_count <<= 32;
	count->event_count |=
		INTEL_PMC_TGPIO_RD_REG(TGPIOECCV31_0, count->index);
	dt_hi_e = INTEL_PMC_TGPIO_RD_REG(TGPIOTCV63_32, count->index);

	if (dt_hi_e != dt_hi_s && dt_lo >> 31)
		dt_ts = convert_art_to_tsc_ns(((u64)dt_hi_s << 32) | dt_lo);
	else
		dt_ts = convert_art_to_tsc_ns(((u64)dt_hi_e << 32) | dt_lo);

	count->device_time = ts64_to_ptp_clock_time(dt_ts);

	mutex_lock(&intel_pmc_tgpio->lock);
	intel_pmc_tgpio->pin[count->index].busy = false;
	complete(&intel_pmc_tgpio->pin[count->index].transact_comp);
	mutex_unlock(&intel_pmc_tgpio->lock);

	return 0;
}

static int intel_pmc_tgpio_verify(struct ptp_clock_info *ptp,
				  unsigned int pin,
				  enum ptp_pin_function func,
				  unsigned int chan)
{
	struct intel_pmc_tgpio_t *tgpio = to_intel_pmc_tgpio(ptp);
	int ret = 0;

	if (func == PTP_PF_PHYSYNC)
		return -1;

	/* pin/channel is fixed for TGPIO hardware */
	if (ptp->pin_config[pin].chan != chan)
		return -1;

	mutex_lock(&tgpio->lock);
	/* The pin has been removed, but the PTP interface is still here */
	if (intel_pmc_tgpio->pin[pin].base == NULL)
		ret = -1;
	mutex_unlock(&tgpio->lock);

	return ret;
}

static struct ptp_pin_desc intel_pmc_tgpio_pin_config[MAX_PINS_PER_PLAT];

static const struct ptp_clock_info intel_pmc_tgpio_info = {
	.owner		= THIS_MODULE,
	.name		= "Intel PMC TGPIO",
	.max_adj	= 0,
	.n_pins		= 0,
	.n_ext_ts	= 0,
	.n_per_out	= 0,
	.pin_config	= intel_pmc_tgpio_pin_config,
	.gettime64	= intel_pmc_tgpio_gettime64,
	.settime64	= intel_pmc_tgpio_settime64,
	.enable		= intel_pmc_tgpio_enable,
	.getcrosststamp	= intel_pmc_tgpio_getcrosststamp,
	.counttstamp	= intel_pmc_tgpio_counttstamp,
	.verify		= intel_pmc_tgpio_verify,
};

static int ptp_device_register(struct platform_device *pdev)
{
	intel_pmc_tgpio->clock = ptp_clock_register
		(&intel_pmc_tgpio->info, NULL);
	if (IS_ERR(intel_pmc_tgpio->clock))
		return PTR_ERR(intel_pmc_tgpio->clock);

	return 0;
}

static const plat_acpi_resource *find_plat_acpi_resource(struct platform_device *pdev,
							 int *n_pins)
{
	bool found = false;
	unsigned int index;
	unsigned int pin_index;
	const plat_acpi_resource *ret;

	for (pin_index = 0; pin_index < MAX_PINS_PER_PLAT; ++pin_index) {
		for (index = 0; index < PLATFORM_COUNT; ++index) {
			char *res_name = acpi_plat_map[index][pin_index];

			if (res_name == NULL)
				continue;
			if (!strncmp(pdev->name, res_name, strlen(res_name))) {
				found = true;
				break;
			}
		}
		if (found)
			break;
	}
	ret = acpi_plat_map+index;
	if (pin_index == MAX_PINS_PER_PLAT)
		return ERR_PTR(-ENODEV);

	/* Count the number of pins */
	*n_pins = 0;
	for (pin_index = 0; pin_index < MAX_PINS_PER_PLAT; ++pin_index) {
		if ((*ret)[pin_index] == NULL)
			break;
		*n_pins += 1;
	}

	return ret;
}

static int intel_pmc_tgpio_probe(struct platform_device *pdev)
{
	struct intel_pmc_tgpio_t *tgpio = intel_pmc_tgpio;
	unsigned int pin_index;
	unsigned int pin_count;
	struct intel_pmc_tgpio_pin *tgpio_pin;
	struct ptp_pin_desc *tgpio_pin_desc;
	struct resource *res;
	char pinname_tmpl[] = DRIVER_NAME"-pin//";
	int ret = 0;

	mutex_lock(&tgpio->lock);

	/* Find and cache the list of ACPI resources for this platform */
	if (tgpio->plat_res == NULL) {
		tgpio->plat_res = find_plat_acpi_resource
			(pdev, &tgpio->info.n_pins);
		tgpio->info.n_ext_ts = tgpio->info.n_per_out
			= tgpio->info.n_pins;
	}
	if (IS_ERR(tgpio->plat_res)) {
		ret = PTR_ERR(tgpio->plat_res);
		goto unlock;
	}

	/* Find an available pin */
	for (pin_index = 0; pin_index < MAX_PINS_PER_PLAT; ++pin_index) {
		if (tgpio->pin[pin_index].base == NULL)
			break;
	}
	if (pin_index == MAX_PINS_PER_PLAT) {
		ret = -ENODEV;
		goto unlock;
	}

	tgpio_pin = tgpio->pin+pin_index;
	tgpio_pin_desc = tgpio->info.pin_config+pin_index;
	tgpio_pin->pdev = pdev;
	platform_set_drvdata(pdev, tgpio);
	pin_count = pin_index + 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tgpio_pin->base = devm_ioremap_resource(&pdev->dev, res);
	if (!tgpio_pin->base) {
		ret = -ENOMEM;
		goto unlock;
	}

	/* Find the pin corresponding to the ACPI ID */
	for (pin_index = 0; pin_index < MAX_PINS_PER_PLAT; ++pin_index) {
		const char *cmpr = (const char *)(*tgpio->plat_res)[pin_index];

		if (strncmp(cmpr, pdev->name, strlen(cmpr)) == 0)
			break;
	}
	if (pin_index == MAX_PINS_PER_PLAT) {
		ret = -ENODEV;
		goto unlock;
	}

	tgpio_pin->regset = devm_kzalloc
		(&pdev->dev, sizeof(*tgpio_pin->regset), GFP_KERNEL);
	if (!tgpio_pin->regset) {
		ret = -ENOMEM;
		goto unlock;
	}

	tgpio_pin->regset->regs = intel_pmc_tgpio_regs;
	tgpio_pin->regset->nregs = ARRAY_SIZE(intel_pmc_tgpio_regs);
	tgpio_pin->regset->base = tgpio_pin->base;

	/* Pin names are 1 indexed (+1) because hardware works like that */
	snprintf(strchr(pinname_tmpl, '/'), 3, "%02u", pin_index+1);
	tgpio_pin->root = debugfs_create_dir(pinname_tmpl, NULL);
	debugfs_create_regset32("regdump", 0444, tgpio_pin->root,
				tgpio_pin->regset);
	strncpy(tgpio_pin_desc->name, pinname_tmpl,
		sizeof(tgpio_pin_desc->name));

	if (tgpio->info.n_pins == pin_count)
		ret = ptp_device_register(pdev);

unlock:
	mutex_unlock(&tgpio->lock);

	return ret;
}

static int intel_pmc_tgpio_remove(struct platform_device *pdev)
{
	struct intel_pmc_tgpio_t *tgpio = platform_get_drvdata(pdev);
	unsigned int pin_index;
	struct intel_pmc_tgpio_pin *tgpio_pin;
	int ret = 0;

	mutex_lock(&tgpio->lock);

	for (pin_index = 0; pin_index < MAX_PINS_PER_PLAT; ++pin_index) {
		tgpio_pin = tgpio->pin+pin_index;
		if (tgpio_pin->pdev == pdev || tgpio_pin->pdev == NULL)
			break;
	}
	if (pin_index == MAX_PINS_PER_PLAT || tgpio_pin->pdev == NULL) {
		ret = -ENODEV;
		goto unlock;
	}

	while (tgpio_pin->busy) {
		mutex_unlock(&tgpio->lock);
		wait_for_completion(&tgpio_pin->transact_comp);
		mutex_lock(&tgpio->lock);
	}
	tgpio_pin->busy = true;
	mutex_unlock(&tgpio->lock);

	tgpio_pin->base = NULL;
	debugfs_remove_recursive(tgpio_pin->root);

	mutex_lock(&tgpio->lock);
	tgpio_pin->busy = false;
	complete(&tgpio_pin->transact_comp);

unlock:
	mutex_unlock(&tgpio->lock);
	return ret;
}

static const struct acpi_device_id intel_pmc_acpi_match[] = {
	{ "INTC1021", 0 }, /* EHL */
	{ "INTC1022", 0 }, /* EHL */
	{ "INTC1023", 0 }, /* TGL */
	{ "INTC1024", 0 }, /* TGL */
	{  },
};

MODULE_ALIAS("acpi*:INTC1021:*");
MODULE_ALIAS("acpi*:INTC1022:*");
MODULE_ALIAS("acpi*:INTC1023:*");
MODULE_ALIAS("acpi*:INTC1024:*");

static struct platform_driver intel_pmc_tgpio_driver = {
	.probe		= intel_pmc_tgpio_probe,
	.remove		= intel_pmc_tgpio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static struct acpi_device_id *construct_acpi_match_table(void)
{
	int acpi_id_count = 0;
	int plat_iter;
	int pin_iter;
	struct acpi_device_id *table_iter;
	struct acpi_device_id *ret;

	/* Walk through all ACPI IDs constructing driver table */
	for (plat_iter = 0; plat_iter < PLATFORM_COUNT; ++plat_iter) {
		for (pin_iter = 0; pin_iter < MAX_PINS_PER_PLAT; ++pin_iter) {
			if (acpi_plat_map[plat_iter][pin_iter])
				++acpi_id_count;
			else
				break;
		}
	}
	/* Leave NULL entry at the end */
	ret = kzalloc(sizeof(*ret)*(acpi_id_count+1), GFP_KERNEL);
	if (ret == NULL)
		ret = ERR_PTR(-ENOMEM);

	table_iter = ret;
	for (plat_iter = 0; plat_iter < PLATFORM_COUNT; ++plat_iter) {
		for (pin_iter = 0; pin_iter < MAX_PINS_PER_PLAT; ++pin_iter) {
			if (acpi_plat_map[plat_iter][pin_iter]) {
				strncpy(table_iter->id,
					acpi_plat_map[plat_iter][pin_iter],
					sizeof(table_iter->id));
				++table_iter;
			} else {
				break;
			}
		}
	}

	return ret;
}

static int intel_pmc_tgpio_init(void)
{
	unsigned int iter;
	int ret;

	if (!boot_cpu_has(X86_FEATURE_TSC_KNOWN_FREQ))
		return -ENXIO;

	intel_pmc_tgpio_driver.driver.acpi_match_table =
		ACPI_PTR(construct_acpi_match_table());
	if (IS_ERR(intel_pmc_tgpio_driver.driver.acpi_match_table)) {
		ret = PTR_ERR(intel_pmc_tgpio_driver.driver.acpi_match_table);
		goto alloc_acpi_table;
	}

	intel_pmc_tgpio =
		kmalloc(sizeof(struct intel_pmc_tgpio_t), GFP_KERNEL);
	if (intel_pmc_tgpio == NULL) {
		ret = -ENOMEM;
		goto alloc_tgpio;
	}

	mutex_init(&intel_pmc_tgpio->lock);
	intel_pmc_tgpio->info = intel_pmc_tgpio_info;
	intel_pmc_tgpio->plat_res = NULL;
	for (iter = 0; iter < MAX_PINS_PER_PLAT; ++iter) {
		intel_pmc_tgpio->info.pin_config[iter].name[0] = '\0';
		intel_pmc_tgpio->info.pin_config[iter].index   = iter;
		intel_pmc_tgpio->info.pin_config[iter].chan    = iter;
		intel_pmc_tgpio->info.pin_config[iter].func    = PTP_PF_NONE;
		intel_pmc_tgpio->info.pin_config[iter].flags   =
			PTP_PINDESC_INPUTPOLL | PTP_PINDESC_EVTCNTVALID;
		intel_pmc_tgpio->pin[iter].busy = false;
		init_completion(&intel_pmc_tgpio->pin[iter].transact_comp);
		intel_pmc_tgpio->pin[iter].root = NULL;
		intel_pmc_tgpio->pin[iter].base = NULL;
	}

	ret = platform_driver_register(&intel_pmc_tgpio_driver);
	if (ret)
		goto driver_register;

	return 0;

driver_register:
	kfree(intel_pmc_tgpio);

alloc_tgpio:
	kfree(intel_pmc_tgpio_driver.driver.acpi_match_table);

alloc_acpi_table:
	return ret;
}

static void intel_pmc_tgpio_exit(void)
{
	ptp_clock_unregister(intel_pmc_tgpio->clock);
	platform_driver_unregister(&intel_pmc_tgpio_driver);
	mutex_destroy(&intel_pmc_tgpio->lock);
	kfree(intel_pmc_tgpio);
	kfree(intel_pmc_tgpio_driver.driver.acpi_match_table);
}

module_init(intel_pmc_tgpio_init);
module_exit(intel_pmc_tgpio_exit);


MODULE_AUTHOR("Christopher Hall <christopher.s.hall@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel PMC Timed GPIO Controller Driver");
