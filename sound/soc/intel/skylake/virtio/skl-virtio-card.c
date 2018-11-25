// SPDX-License-Identifier: GPL-2.0+
//
// skl-virtio-card.c  --  ASoC cAVS virtio sound card
//
// Copyright (C) 2018 Intel Corporation.
//
// Authors: Furtak, Pawel <pawel.furtak@intel.com>
//          Janca, Grzegorz <grzegorz.janca@intel.com>
//
//  This module registers virtio sound card as a machine driver

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "skl-virtio-fe.h"

struct snd_kcontrol_new skl_virtio_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static const struct snd_soc_dapm_widget skl_virtio_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("DiranaCp", NULL),
	SND_SOC_DAPM_HP("DiranaPb", NULL),
	SND_SOC_DAPM_MIC("HdmiIn", NULL),
	SND_SOC_DAPM_MIC("TestPinCp", NULL),
	SND_SOC_DAPM_HP("TestPinPb", NULL),
	SND_SOC_DAPM_MIC("BtHfpDl", NULL),
	SND_SOC_DAPM_HP("BtHfpUl", NULL),
	SND_SOC_DAPM_MIC("ModemDl", NULL),
	SND_SOC_DAPM_HP("ModemUl", NULL),
};

static const struct snd_soc_dapm_route skl_virtio_map[] = {
	/* Speaker BE connections */
	{ "Speaker", NULL, "ssp4 Tx"},
	{ "ssp4 Tx", NULL, "codec0_out"},

	{ "dirana_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx", NULL, "DiranaCp"},

	{ "dirana_aux_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx", NULL, "DiranaCp"},

	{ "dirana_tuner_in", NULL, "ssp2 Rx"},
	{ "ssp2 Rx", NULL, "DiranaCp"},

	{ "DiranaPb", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "dirana_out"},

	{ "hdmi_ssp1_in", NULL, "ssp1 Rx"},
	{ "ssp1 Rx", NULL, "HdmiIn"},

	{ "TestPin_ssp5_in", NULL, "ssp5 Rx"},
	{ "ssp5 Rx", NULL, "TestPinCp"},

	{ "TestPinPb", NULL, "ssp5 Tx"},
	{ "ssp5 Tx", NULL, "TestPin_ssp5_out"},

	{ "BtHfp_ssp0_in", NULL, "ssp0 Rx"},
	{ "ssp0 Rx", NULL, "BtHfpDl"},

	{ "BtHfpUl", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "BtHfp_ssp0_out"},

	{ "Modem_ssp3_in", NULL, "ssp3 Rx"},
	{ "ssp3 Rx", NULL, "ModemDl"},

	{ "ModemUl", NULL, "ssp3 Tx"},
	{ "ssp3 Tx", NULL, "Modem_ssp3_out"},
};

static struct snd_soc_dai_link skl_virtio_dais[] = {
	/* Front End DAI links */
	{
		.name = "Speaker Port",
		.stream_name = "Speaker",
		.cpu_dai_name = "Speaker Pin",
		.platform_name = "virtio4",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 1,
	},
	{
		.name = "Dirana Capture Port",
		.stream_name = "Dirana Cp",
		.cpu_dai_name = "Dirana Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
		{
		.name = "Dirana Playback Port",
		.stream_name = "Dirana Pb",
		.cpu_dai_name = "Dirana Pb Pin",
		.platform_name = "virtio4",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 1,
	},
	{
		.name = "TestPin Capture Port",
		.stream_name = "TestPin Cp",
		.cpu_dai_name = "TestPin Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "TestPin Playback Port",
		.stream_name = "TestPin Pb",
		.cpu_dai_name = "TestPin Pb Pin",
		.platform_name = "virtio4",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 1,
	},
	{
		.name = "BtHfp Capture Port",
		.stream_name = "BtHfp Cp",
		.cpu_dai_name = "BtHfp Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "BtHfp Playback Port",
		.stream_name = "BtHfp Pb",
		.cpu_dai_name = "BtHfp Pb Pin",
		.platform_name = "virtio4",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 1,
	},
	{
		.name = "Modem Capture Port",
		.stream_name = "Modem Cp",
		.cpu_dai_name = "Modem Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Modem Playback Port",
		.stream_name = "Modem Pb",
		.cpu_dai_name = "Modem Pb Pin",
		.platform_name = "virtio4",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dpcm_playback = 1,
	},
	{
		.name = "HDMI Capture Port",
		.stream_name = "HDMI Cp",
		.cpu_dai_name = "HDMI Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Dirana Aux Capture Port",
		.stream_name = "Dirana Aux Cp",
		.cpu_dai_name = "Dirana Aux Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Dirana Tuner Capture Port",
		.stream_name = "Dirana Tuner Cp",
		.cpu_dai_name = "Dirana Tuner Cp Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},


	/* Back End DAI links */
	{
		/* SSP0 - BT */
		.name = "SSP0-Codec",
		.id = 0,
		.cpu_dai_name = "SSP0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP1 - HDMI-In */
		.name = "SSP1-Codec",
		.id = 1,
		.cpu_dai_name = "SSP1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
	{
		/* SSP2 - Dirana */
		.name = "SSP2-Codec",
		.id = 2,
		.cpu_dai_name = "SSP2 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
		{
		/* SSP3 - Modem */
		.name = "SSP3-Codec",
		.id = 3,
		.cpu_dai_name = "SSP3 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP4 - Amplifier */
		.name = "SSP4-Codec",
		.id = 4,
		.cpu_dai_name = "SSP4 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.ignore_suspend = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},
	{
		/* SSP5 - TestPin */
		.name = "SSP5-Codec",
		.id = 5,
		.cpu_dai_name = "SSP5 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "virtio4",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
		.no_pcm = 1,
	},

};

static int skl_virtio_add_dai_link(struct snd_soc_card *card,
			    struct snd_soc_dai_link *link)
{
	link->platform_name = "virtio4";
	link->nonatomic = 1;
	return 0;
}

static struct snd_soc_card skl_virtio_card = {
	.name = "skl_virtio_card",
	.dai_link = skl_virtio_dais,
	.num_links = ARRAY_SIZE(skl_virtio_dais),
	.controls = skl_virtio_controls,
	.num_controls = ARRAY_SIZE(skl_virtio_controls),
	.dapm_widgets = skl_virtio_widgets,
	.num_dapm_widgets = ARRAY_SIZE(skl_virtio_widgets),
	.dapm_routes = skl_virtio_map,
	.num_dapm_routes = ARRAY_SIZE(skl_virtio_map),
	.fully_routed = true,
	.add_dai_link = skl_virtio_add_dai_link,
};

static int skl_virtio_card_probe(struct platform_device *pdev)
{
	struct snd_skl_vfe *vfe;
	int ret;
	struct snd_soc_card *card = &skl_virtio_card;

	card->dev = &pdev->dev;
	vfe = dev_get_drvdata(&pdev->dev);
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret < 0)
		return ret;

	vfe->notify_machine_probe(vfe, pdev, card);

	return ret;
}

static int skl_virtio_card_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&skl_virtio_card);
	return 0;
}

static struct platform_driver skl_virtio_audio = {
	.probe = skl_virtio_card_probe,
	.remove = skl_virtio_card_remove,
	.driver = {
		.name = "skl_virtio_card",
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(skl_virtio_audio)

MODULE_DESCRIPTION("ASoC cAVS virtio sound card");
MODULE_AUTHOR("Pawel Furtak");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:skl-virtio-card");
