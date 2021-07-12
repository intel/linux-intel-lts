// SPDX-License-Identifier: GPL-2.0
//
// Copyright(c) 2021 Intel Corporation. All rights reserved.

/*
 * tgl_rt1308.c - ASoc Machine driver for Intel platforms
 * with RT1308 codec.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>

#include "../../codecs/rt1308.h"
#include "../../codecs/hdac_hdmi.h"

static int tgl_rt1308_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int clk_id, clk_freq, pll_out;
	int err;

	clk_id = RT1308_PLL_S_MCLK;
	clk_freq = 38400000;

	pll_out = params_rate(params) * 512;

	/* Set rt1308 pll */
	err = snd_soc_dai_set_pll(codec_dai, 0, clk_id, clk_freq, pll_out);
	if (err < 0) {
		dev_err(card->dev, "Failed to set RT1308 PLL: %d\n", err);
		return err;
	}

	/* Set rt1308 sysclk */
	err = snd_soc_dai_set_sysclk(codec_dai, RT1308_FS_SYS_S_PLL, pll_out,
				     SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "Failed to set RT1308 SYSCLK: %d\n", err);
		return err;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops tgl_rt1308_ops = {
	.hw_params = tgl_rt1308_hw_params,
};

static const struct snd_soc_dapm_widget tgl_rt1308_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speakers", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
};

static const struct snd_kcontrol_new tgl_rt1308_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speakers"),
};

static const struct snd_soc_dapm_route tgl_rt1308_dapm_routes[] = {
	{ "Speakers", NULL, "SPOL" },
	{ "Speakers", NULL, "SPOR" },

	/* digital mics */
	{"DMic", NULL, "SoC DMIC"},
};

SND_SOC_DAILINK_DEF(ssp1_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP1 Pin")));

SND_SOC_DAILINK_DEF(ssp1_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")));

SND_SOC_DAILINK_DEF(ssp2_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("SSP2 Pin")));

SND_SOC_DAILINK_DEF(ssp2_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("i2c-10EC1308:00", "rt1308-aif")));

SND_SOC_DAILINK_DEF(platform,
	DAILINK_COMP_ARRAY(COMP_PLATFORM("0000:00:1f.3")));

SND_SOC_DAILINK_DEF(dmic_pin,
	DAILINK_COMP_ARRAY(COMP_CPU("DMIC01 Pin")));
SND_SOC_DAILINK_DEF(dmic_codec,
	DAILINK_COMP_ARRAY(COMP_CODEC("dmic-codec", "dmic-hifi")));
SND_SOC_DAILINK_DEF(dmic16k,
	DAILINK_COMP_ARRAY(COMP_CPU("DMIC16k Pin")));

static struct snd_soc_dai_link tgl_rt1308_dailink[] = {
	{
		.name = "NoCodec-0",
		.id = 0,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp1_pin, ssp1_codec, platform),
	},
	{
		.name		= "SSP2-Codec",
		.id		= 1,
		.no_pcm		= 1,
		.ops		= &tgl_rt1308_ops,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(ssp2_pin, ssp2_codec, platform),
	},
	{
		.name = "dmic01",
		.id = 2,
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(dmic_pin, dmic_codec, platform),
	},
	{
		.name = "dmic16k",
		.id = 3,
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
		SND_SOC_DAILINK_REG(dmic16k, dmic_codec, platform),
	},
};

/* audio machine driver */
static struct snd_soc_card tgl_rt1308_card = {
	.name         = "tgl_rt1308",
	.owner        = THIS_MODULE,
	.dai_link     = tgl_rt1308_dailink,
	.num_links = ARRAY_SIZE(tgl_rt1308_dailink),
	.controls = tgl_rt1308_controls,
	.num_controls = ARRAY_SIZE(tgl_rt1308_controls),
	.dapm_widgets = tgl_rt1308_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tgl_rt1308_dapm_widgets),
	.dapm_routes = tgl_rt1308_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(tgl_rt1308_dapm_routes),
};

static int tgl_rt1308_probe(struct platform_device *pdev)
{
	struct snd_soc_acpi_mach *mach;
	struct snd_soc_card *card = &tgl_rt1308_card;
	int ret;

	card->dev = &pdev->dev;

	mach = (&pdev->dev)->platform_data;

	ret = snd_soc_fixup_dai_links_platform_name(card,
						    mach->mach_params.platform);
	if (ret)
		return ret;

	snd_soc_card_set_drvdata(card, NULL);

	return devm_snd_soc_register_card(&pdev->dev, card);
}

static struct platform_driver tgl_rt1308_driver = {
	.driver = {
		.name   = "tgl_rt1308",
		.pm = &snd_soc_pm_ops,
	},
	.probe          = tgl_rt1308_probe,
};

module_platform_driver(tgl_rt1308_driver);

MODULE_AUTHOR("Xiuli Pan");
MODULE_DESCRIPTION("ASoC Intel(R) Tiger Lake + RT1308 Machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tgl_rt1308");
