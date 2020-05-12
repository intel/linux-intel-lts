// SPDX-License-Identifier: GPL-2.0

/*
 *  Intel Elkhart Lake I2S Machine driver with RT5660 Codec
 *
 *  Copyright (C) 2019 Intel Corp
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/acpi.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>

#include "../../codecs/rt5660.h"
#include "../../codecs/hdac_hdmi.h"

#define EHL_BE_FIXUP_RATE	48000
#define RT5660_CODEC_DAI	"rt5660-aif1"
#define DUAL_CHANNEL 2

struct ehl_card_private {
	struct list_head hdmi_pcm_list;
};

static const struct snd_kcontrol_new ehl_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic2"),
	SOC_DAPM_PIN_SWITCH("Line Out"),
};

static const struct snd_soc_dapm_widget ehl_rt5660_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic2", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
};

static const struct snd_soc_pcm_stream dai_params_codec = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_dapm_route ehl_map[] = {

	{"Speaker", NULL, "SPO"},

	{"Headset Mic", NULL, "MICBIAS1"},
	{"Headset Mic2", NULL, "MICBIAS2"},

	{"IN1P", NULL, "Headset Mic"},
	{"IN2P", NULL, "Headset Mic2"},

	{"Line Out", NULL, "LOUTR"},
	{"Line Out", NULL, "LOUTL"},

	{"DMic", NULL, "SoC DMIC"},

};

static int ehl_card_late_probe(struct snd_soc_card *card)
{
	return 0;
}

static int ehl_be_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{

	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* The ADSP will convert the FE rate to 48k, stereo */
	rate->min = rate->max = EHL_BE_FIXUP_RATE;
	channels->min = channels->max = DUAL_CHANNEL;

	/* set SSP0 to 24 bit */
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static int ehl_rt5660_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai,
				     RT5660_SCLK_S_PLL1,
				     params_rate(params) * 512,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "snd_soc_dai_set_sysclk err = %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0,
				  RT5660_PLL1_S_BCLK,
				  params_rate(params) * 50,
				  params_rate(params) * 512);
	if (ret < 0)
		dev_err(codec_dai->dev, "can't set codec pll: %d\n", ret);

	return ret;
}

static struct snd_soc_ops ehl_rt5660_ops = {
	.hw_params = ehl_rt5660_hw_params,
};

static const unsigned int rates[] = {
	48000,
};

static const struct snd_pcm_hw_constraint_list constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list  = rates,
	.mask = 0,
};

static const unsigned int channels[] = {
	DUAL_CHANNEL,
};

static const struct snd_pcm_hw_constraint_list constraints_channels = {
	.count = ARRAY_SIZE(channels),
	.list = channels,
	.mask = 0,
};

#if IS_ENABLED(CONFIG_SND_SOC_INTEL_EHL_RT5660_FPGA)
static const char pname[] = "0000:02:1f.3";
static const char cname[] = "i2c-INT34C2:00";
#else
static const char pname[] = "0000:00:1f.3";
static const char cname[] = "i2c-INTC1027:00"; /* EHL Board */
#endif

SND_SOC_DAILINK_DEF(ssp0_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP0 Pin")));

SND_SOC_DAILINK_DEF(rt5660_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC(cname, RT5660_CODEC_DAI)));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM(pname)));

SND_SOC_DAILINK_DEF(dmic_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("DMIC01 Pin")));
SND_SOC_DAILINK_DEF(dmic_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("dmic-codec", "dmic-hifi")));
SND_SOC_DAILINK_DEF(dmic16k,
	DAILINK_COMP_ARRAY(COMP_CPU("DMIC16k Pin")));

static struct snd_soc_dai_link ehl_rt5660_msic_dailink[] = {
	/* back ends */
	{
		.name = "SSP0-Codec",
		.id = 0,
		.init = NULL,
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.ops = &ehl_rt5660_ops,
		.be_hw_params_fixup = ehl_be_fixup,
		.nonatomic = true,
		SND_SOC_DAILINK_REG(ssp0_pin, rt5660_codec, platform),
	},
	{
		.name = "dmic01",
		.id = 1,
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(dmic_pin, dmic_codec, platform),
	},
	{
		.name = "dmic16k",
		.id = 2,
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(dmic16k, dmic_codec, platform),
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_ehl = {
	.name = "ehl-rt5660",
	.owner = THIS_MODULE,
	.dai_link = ehl_rt5660_msic_dailink,
	.num_links = ARRAY_SIZE(ehl_rt5660_msic_dailink),
	.dapm_widgets = ehl_rt5660_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ehl_rt5660_widgets),
	.dapm_routes = ehl_map,
	.num_dapm_routes = ARRAY_SIZE(ehl_map),
	.controls = ehl_controls,
	.num_controls = ARRAY_SIZE(ehl_controls),
	.fully_routed = true,
	.late_probe = ehl_card_late_probe,
};

static int snd_ehl_rt5660_probe(struct platform_device *pdev)
{
	struct snd_soc_acpi_mach *mach;
	struct snd_soc_card *card = &snd_soc_card_ehl;
	int ret;

	card->dev = &pdev->dev;

	mach = (&pdev->dev)->platform_data;
	ret = snd_soc_fixup_dai_links_platform_name(card,
						    mach->mach_params.platform);
	if (ret)
		return ret;

	return devm_snd_soc_register_card(&pdev->dev, card);
}

static const struct platform_device_id ehl_board_ids[] = {
	{ .name = "ehl_rt5660" },
	{ }
};

static struct platform_driver snd_ehl_rt5660_driver = {
	.driver = {
		.name = "ehl_rt5660",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_ehl_rt5660_probe,
	.id_table = ehl_board_ids,
};

module_platform_driver(snd_ehl_rt5660_driver);

MODULE_DESCRIPTION("Machine driver-RT5660 in I2S mode");
MODULE_AUTHOR("poey.seng.lai@intel.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ehl_rt5660");
