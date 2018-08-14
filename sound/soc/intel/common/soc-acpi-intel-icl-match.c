// SPDX-License-Identifier: GPL-2.0
/*
 * soc-apci-intel-icl-match.c - tables and support for ICL ACPI enumeration.
 *
 * Copyright (c) 2018, Intel Corporation.
 *
 */

#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../skylake/skl.h"

static struct skl_machine_pdata icl_pdata = {
	.use_tplg_pcm = true,
};

struct snd_soc_acpi_mach snd_soc_acpi_intel_icl_machines[] = {
#if IS_ENABLED(CONFIG_SND_SOC_RT700)
	{
		.id = "dummy",
		.drv_name = "icl_rt700",
		.fw_filename = "intel/dsp_fw_icl.bin",
		.pdata = &icl_pdata,
	},
#elif IS_ENABLED(CONFIG_SND_SOC_WM5110)
	{
		.id = "dummy",
		.drv_name = "icl_wm8281",
		.fw_filename = "intel/dsp_fw_icl.bin",
		.pdata = &icl_pdata,
	},
#else
	{
		.id = "dummy",
		.drv_name = "icl_rt274",
		.fw_filename = "intel/dsp_fw_icl.bin",
		.pdata = &icl_pdata,
	},

#endif
	{}
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_icl_machines);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Common ACPI Match module");
