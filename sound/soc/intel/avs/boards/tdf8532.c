// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright(c) 2021-2022 Intel Corporation. All rights reserved.
//
// Authors: Cezary Rojewski <cezary.rojewski@intel.com>
//          Amadeusz Slawinski <amadeuszx.slawinski@linux.intel.com>
//

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>

static const struct snd_kcontrol_new card_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static const struct snd_soc_dapm_widget card_widgets[] = {
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_MIC("HdmiIn", NULL),
	SND_SOC_DAPM_MIC("DiranaCp", NULL),
	SND_SOC_DAPM_HP("DiranaPb", NULL),
	SND_SOC_DAPM_MIC("ModemDl", NULL),
	SND_SOC_DAPM_HP("ModemUl", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("TestPinCp", NULL),
	SND_SOC_DAPM_HP("TestPinPb", NULL),
};

static const struct snd_soc_dapm_route card_routes[] = {
	{ "ssp0 Rx", NULL, "BtHfpDl" },
	{ "BtHfpUl", NULL, "ssp0 Tx" },

	{ "ssp1 Rx", NULL, "HdmiIn" },

	{ "DiranaPb", NULL, "ssp2:4 Tx" },
	{ "ssp2 Rx", NULL, "DiranaCp" },
	{ "ssp2:2 Rx", NULL, "DiranaCp" },
	{ "ssp2:4 Rx", NULL, "DiranaCp" },

	{ "ssp3 Rx", NULL, "ModemDl" },
	{ "ModemUl", NULL, "ssp3 Tx" },

	{ "Speaker", NULL, "ssp4 Tx" },

	{ "TestPinPb", NULL, "ssp5 Tx" },
	{ "ssp5 Rx", NULL, "TestPinCp" },
};

static int
avs_tdf8532_ssp2_fixup(struct snd_soc_pcm_runtime *runtime, struct snd_pcm_hw_params *params)
{
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* Set SSP to 32 bit */
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);

	return 0;
}

SND_SOC_DAILINK_DEF(ssp0, DAILINK_COMP_ARRAY(COMP_CPU("SSP0 Pin")));
SND_SOC_DAILINK_DEF(ssp1, DAILINK_COMP_ARRAY(COMP_CPU("SSP1 Pin")));
SND_SOC_DAILINK_DEF(ssp2, DAILINK_COMP_ARRAY(COMP_CPU("SSP2 Pin")));
SND_SOC_DAILINK_DEF(ssp2_2, DAILINK_COMP_ARRAY(COMP_CPU("SSP2:2 Pin")));
SND_SOC_DAILINK_DEF(ssp2_4, DAILINK_COMP_ARRAY(COMP_CPU("SSP2:4 Pin")));
SND_SOC_DAILINK_DEF(ssp3, DAILINK_COMP_ARRAY(COMP_CPU("SSP3 Pin")));
SND_SOC_DAILINK_DEF(ssp4, DAILINK_COMP_ARRAY(COMP_CPU("SSP4 Pin")));
SND_SOC_DAILINK_DEF(ssp5, DAILINK_COMP_ARRAY(COMP_CPU("SSP5 Pin")));

SND_SOC_DAILINK_DEF(dummy, DAILINK_COMP_ARRAY(COMP_DUMMY()));
/* Name overridden on probe */
SND_SOC_DAILINK_DEF(platform, DAILINK_COMP_ARRAY(COMP_PLATFORM("")));
SND_SOC_DAILINK_DEF(tdf8532, DAILINK_COMP_ARRAY(COMP_CODEC("i2c-INT34C3:00", "tdf8532-hifi")));

static struct snd_soc_dai_link card_dai_links[] = {
	/* Back End DAI links */
	{
		.name = "SSP0-Codec", /* SSP0 - BT */
		.id = 0,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp0, dummy, platform),
	},
	{
		.name = "SSP1-Codec", /* SSP1 - HDMI-In */
		.id = 1,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp1, dummy, platform),
	},
	{
		.name = "SSP2-Codec", /* SSP2 - Dirana */
		.id = 2,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.be_hw_params_fixup = avs_tdf8532_ssp2_fixup,
		SND_SOC_DAILINK_REG(ssp2_4, dummy, platform),
	},
	{
		.name = "SSP2-Aux-Codec", /* SSP2 - Dirana Aux */
		.id = 2,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_playback = 0,
		.dpcm_capture = 1,
		.be_hw_params_fixup = avs_tdf8532_ssp2_fixup,
		SND_SOC_DAILINK_REG(ssp2_2, dummy, platform),
	},
	{
		.name = "SSP2-Tuner-Codec", /* SSP2 - Dirana Tuner */
		.id = 2,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_playback = 0,
		.dpcm_capture = 1,
		.be_hw_params_fixup = avs_tdf8532_ssp2_fixup,
		SND_SOC_DAILINK_REG(ssp2, dummy, platform),
	},
	{
		.name = "SSP3-Codec", /* SSP3 - Modem */
		.id = 3,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp3, dummy, platform),
	},
	{
		.name = "SSP4-Codec", /* SSP4 - Amplifier */
		.id = 4,
		.no_pcm = 1,
		.dpcm_playback = 1,
		SND_SOC_DAILINK_REG(ssp4, tdf8532, platform),
	},
	{
		.name = "SSP5-Codec", /* SSP5 - TestPin */
		.id = 5,
		.ignore_pmdown_time = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(ssp5, dummy, platform),
	},
};

static int avs_tdf8532_probe(struct platform_device *pdev)
{
	struct snd_soc_acpi_mach *mach;
	struct snd_soc_card *card;
	struct device *dev = &pdev->dev;
	int ret;

	mach = dev_get_platdata(&pdev->dev);

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->name = "avs_tdf8532";
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->dai_link = card_dai_links;
	card->num_links = ARRAY_SIZE(card_dai_links);
	card->controls = card_controls;
	card->num_controls = ARRAY_SIZE(card_controls);
	card->dapm_widgets = card_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(card_widgets);
	card->dapm_routes = card_routes;
	card->num_dapm_routes = ARRAY_SIZE(card_routes);
	card->fully_routed = true;

	ret = snd_soc_fixup_dai_links_platform_name(card, mach->mach_params.platform);
	if (ret)
		return ret;

	return devm_snd_soc_register_card(dev, card);
}

static struct platform_driver avs_tdf8532_driver = {
	.probe = avs_tdf8532_probe,
	.driver = {
		.name = "avs_tdf8532",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(avs_tdf8532_driver)

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:avs_tdf8532");
