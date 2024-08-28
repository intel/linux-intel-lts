// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright(c) 2024 Intel Corporation. All rights reserved.
//
// Author: Cezary Rojewski <cezary.rojewski@intel.com>
//

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../utils.h"

static const struct snd_soc_dapm_widget card_widgets[] = {
	SND_SOC_DAPM_HP("CPB Stereo HP 1", NULL),
	SND_SOC_DAPM_HP("CPB Stereo HP 2", NULL),
	SND_SOC_DAPM_HP("CPB Stereo HP 3", NULL),
	SND_SOC_DAPM_LINE("CPB Line Out", NULL),
	SND_SOC_DAPM_MIC("CPB Stereo Mic 1", NULL),
	SND_SOC_DAPM_MIC("CPB Stereo Mic 2", NULL),
	SND_SOC_DAPM_LINE("CPB Line In", NULL),
};

static const struct snd_soc_dapm_route card_routes[] = {
	{"CPB Stereo HP 1", NULL, "AOUT1L"},
	{"CPB Stereo HP 1", NULL, "AOUT1R"},
	{"CPB Stereo HP 2", NULL, "AOUT2L"},
	{"CPB Stereo HP 2", NULL, "AOUT2R"},
	{"CPB Stereo HP 3", NULL, "AOUT3L"},
	{"CPB Stereo HP 3", NULL, "AOUT3R"},
	{"CPB Line Out", NULL, "AOUT4L"},
	{"CPB Line Out", NULL, "AOUT4R"},

	{"AIN1L", NULL, "CPB Stereo Mic 1"},
	{"AIN1R", NULL, "CPB Stereo Mic 1"},
	{"AIN2L", NULL, "CPB Stereo Mic 2"},
	{"AIN2R", NULL, "CPB Stereo Mic 2"},
	{"AIN3L", NULL, "CPB Line In"},
	{"AIN3R", NULL, "CPB Line In"},
};

static int avs_pcm3168a_be_fixup(struct snd_soc_pcm_runtime *runtime,
				 struct snd_pcm_hw_params *params)
{
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* Set SSP to 24 bit. */
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);

	return 0;
}

static int avs_pcm3168a_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 24576000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		dev_err(rtd->dev, "Set codec sysclk failed: %d\n", ret);

	return ret;
}

static const struct snd_soc_ops avs_pcm3168a_ops = {
	.hw_params = avs_pcm3168a_hw_params,
};

SND_SOC_DAILINK_DEF(pcm3168a_dac,
		    DAILINK_COMP_ARRAY(COMP_CODEC("i2c-PCM3168A:00", "pcm3168a-dac")));
SND_SOC_DAILINK_DEF(pcm3168a_adc,
		    DAILINK_COMP_ARRAY(COMP_CODEC("i2c-PCM3168A:00", "pcm3168a-adc")));

/* Name overridden durign probe(). */
SND_SOC_DAILINK_DEF(platform, DAILINK_COMP_ARRAY(COMP_PLATFORM("")));
SND_SOC_DAILINK_DEF(cpu_ssp0, DAILINK_COMP_ARRAY(COMP_CPU("SSP0 Pin")));
SND_SOC_DAILINK_DEF(cpu_ssp2, DAILINK_COMP_ARRAY(COMP_CPU("SSP2 Pin")));

static struct snd_soc_dai_link card_links[] = {
	{
		.name = "SSP0-Codec-dac",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBP_CFP,
		.be_hw_params_fixup = avs_pcm3168a_be_fixup,
		.nonatomic = 1,
		.no_pcm = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(cpu_ssp0, pcm3168a_dac, platform),
	},
	{
		.name = "SSP2-Codec-adc",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBP_CFP,
		.be_hw_params_fixup = avs_pcm3168a_be_fixup,
		.nonatomic = 1,
		.no_pcm = 1,
		.dpcm_capture = 1,
		SND_SOC_DAILINK_REG(cpu_ssp2, pcm3168a_adc, platform),
	},
};

static int avs_pcm3168a_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_acpi_mach *mach;
	struct snd_soc_card *card;
	int ret;

	mach = dev_get_platdata(dev);

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->name = "avs_pcm3168a";
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->dai_link = card_links;
	card->num_links = ARRAY_SIZE(card_links);
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

static const struct platform_device_id avs_pcm3168a_driver_ids[] = {
	{
		.name = "avs_pcm3168a",
	},
	{},
};
MODULE_DEVICE_TABLE(platform, avs_pcm3168a_driver_ids);

static struct platform_driver avs_pcm3168a_driver = {
	.probe = avs_pcm3168a_probe,
	.driver = {
		.name = "avs_pcm3168a",
		.pm = &snd_soc_pm_ops,
	},
	.id_table = avs_pcm3168a_driver_ids,
};

module_platform_driver(avs_pcm3168a_driver);

MODULE_AUTHOR("Cezary Rojewski <cezary.rojewski@intel.com>");
MODULE_LICENSE("GPL");
