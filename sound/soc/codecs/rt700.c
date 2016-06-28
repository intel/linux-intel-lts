/*
 * rt700.c -- rt700 ALSA SoC audio driver
 *
 * Copyright 2016 Realtek, Inc.
 *
 * Author: Bard Liao <bardliao@realtek.com>
 * ALC700 ASoC Codec Driver based Intel Dummy SdW codec driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/sdw_bus.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rt700.h"

struct sdw_stream_data {
	int stream_tag;
};

static int rt700_index_write(struct regmap *regmap,
		unsigned int reg, unsigned int value)
{
	int ret;
	unsigned int val_h, val_l;

	val_h = (reg >> 8) & 0xff;
	val_l = reg & 0xff;
	ret = regmap_write(regmap, RT700_PRIV_INDEX_W_H, val_h);
	if (ret < 0) {
		pr_err("Failed to set private addr: %d\n", ret);
		goto err;
	}
	ret = regmap_write(regmap, RT700_PRIV_INDEX_W_L, val_l);
	if (ret < 0) {
		pr_err("Failed to set private addr: %d\n", ret);
		goto err;
	}
	val_h = (value >> 8) & 0xff;
	val_l = value & 0xff;
	ret = regmap_write(regmap, RT700_PRIV_DATA_W_H, val_h);
	if (ret < 0) {
		pr_err("Failed to set private value: %d\n", ret);
		goto err;
	}
	ret = regmap_write(regmap, RT700_PRIV_DATA_W_L, val_l);
	if (ret < 0) {
		pr_err("Failed to set private value: %d\n", ret);
		goto err;
	}
	return 0;

err:
	return ret;
}

/* For Verb-Set Amplifier Gain (Verb ID = 3h) */
static int rt700_set_amp_gain_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct rt700_priv *rt700 = snd_soc_component_get_drvdata(component);
	unsigned int addr_h, addr_l, val_h, val_l;
	unsigned int read_ll, read_rl;


	/* Can't use update bit function, so read the original value first */
	addr_h = (mc->reg + 0x2000) | 0x800;
	addr_l = (mc->rreg + 0x2000) | 0x800;
	if (mc->shift == RT700_DIR_OUT_SFT) /* output */
		val_h = 0x80;
	else /* input */
		val_h = 0x0;
	/* R Channel */
	regmap_write(rt700->regmap, addr_h, val_h);
	pr_debug("%s write %04x %02x\n", __func__, addr_h, val_h);
	regmap_write(rt700->regmap, addr_l, 0);
	pr_debug("%s write %04x %02x\n", __func__, addr_l, 0);
	regmap_read(rt700->regmap, RT700_READ_HDA_0, &read_rl);
	pr_debug("%s read %04x %02x\n", __func__, RT700_READ_HDA_0, read_rl);

	/* L Channel */
	val_h |= 0x20;
	regmap_write(rt700->regmap, addr_h, val_h);
	pr_debug("%s write %04x %02x\n", __func__, addr_h, val_h);
	regmap_write(rt700->regmap, addr_l, 0);
	pr_debug("%s write %04x %02x\n", __func__, addr_l, 0);
	regmap_read(rt700->regmap, RT700_READ_HDA_0, &read_ll);
	pr_debug("%s read %04x %02x\n", __func__, RT700_READ_HDA_0, read_ll);


	/* Now set value */
	addr_h = mc->reg;
	addr_l = mc->rreg;

	/*pr_debug("%s val = %d, %d\n", ucontrol->value.integer.value[0],
					ucontrol->value.integer.value[1]);*/
	pr_debug("%s val = %d, %d\n", __func__, ucontrol->value.integer.value[0],
					ucontrol->value.integer.value[1]);
	/* L Channel */
	val_h = (1 << mc->shift) | (1 << 5);

	if (mc->invert) {
		/* for mute */
		val_l = (mc->max - ucontrol->value.integer.value[0]) << 7;
		/* keep gain */
		read_ll = read_ll & 0x7f;
		val_l |= read_rl;
	} else {
		/* for gain */
		val_l = ((ucontrol->value.integer.value[0]) & mc->max);
		/* keep mute status */
		read_ll = read_ll & 0x80;
		val_l |= read_rl;
	}

	regmap_write(rt700->regmap, addr_h, val_h);
	pr_debug("%s write %04x %02x\n", __func__, addr_h, val_h);
	regmap_write(rt700->regmap, addr_l, val_l);
	pr_debug("%s write %04x %02x\n", __func__, addr_l, val_l);

	/* R Channel */
	val_h = (1 << mc->shift) | (1 << 4);

	if (mc->invert) {
		/* for mute */
		val_l = (mc->max - ucontrol->value.integer.value[1]) << 7;
		/* keep gain */
		read_rl = read_rl & 0x7f;
		val_l |= read_rl;
	} else {
		/* for gain */
		val_l = ((ucontrol->value.integer.value[1]) & mc->max);
		/* keep mute status */
		read_rl = read_rl & 0x80;
		val_l |= read_rl;
	}
	val_h = (1 << mc->shift) | (1 << 4);
	regmap_write(rt700->regmap, addr_h, val_h);
	pr_debug("%s write %04x %02x\n", __func__, addr_h, val_h);
	regmap_write(rt700->regmap, addr_l, val_l);
	pr_debug("%s write %04x %02x\n", __func__, addr_l, val_l);

	return 0;
}

static int rt700_set_amp_gain_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt700_priv *rt700 = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int addr_h, addr_l, val_h;
	unsigned int read_ll, read_rl;

	addr_h = (mc->reg + 0x2000) | 0x800;
	addr_l = (mc->rreg + 0x2000) | 0x800;
	if (mc->shift == RT700_DIR_OUT_SFT) /* output */
		val_h = 0x80;
	else /* input */
		val_h = 0x0;
	/* R Channel */
	regmap_write(rt700->regmap, addr_h, val_h);
	pr_debug("%s write %04x %02x\n", __func__, addr_h, val_h);
	regmap_write(rt700->regmap, addr_l, 0);
	pr_debug("%s write %04x %02x\n", __func__, addr_l, 0);
	regmap_read(rt700->regmap, RT700_READ_HDA_0, &read_rl);
	pr_debug("%s read %04x %02x\n", __func__, RT700_READ_HDA_0, read_rl);

	/* L Channel */
	val_h |= 0x20;
	regmap_write(rt700->regmap, addr_h, val_h);
	pr_debug("%s write %04x %02x\n", __func__, addr_h, val_h);
	regmap_write(rt700->regmap, addr_l, 0);
	pr_debug("%s write %04x %02x\n", __func__, addr_l, 0);
	regmap_read(rt700->regmap, RT700_READ_HDA_0, &read_ll);
	pr_debug("%s read %04x %02x\n", __func__, RT700_READ_HDA_0, read_ll);

	if (mc->invert) {
		/* for mute status */
		read_ll = !((read_ll & 0x80) >> RT700_MUTE_SFT);
		read_rl = !((read_rl & 0x80) >> RT700_MUTE_SFT);
	} else {
		/* for gain */
		read_ll = read_ll & 0x7f;
		read_rl = read_rl & 0x7f;
	}
	ucontrol->value.integer.value[0] = read_ll;
	ucontrol->value.integer.value[1] = read_rl;


	return 0;
}

static const DECLARE_TLV_DB_SCALE(out_vol_tlv, -6525, 75, 0);
static const DECLARE_TLV_DB_SCALE(in_vol_tlv, -1725, 75, 0);
static const DECLARE_TLV_DB_SCALE(mic_vol_tlv, 0, 1000, 0);

#define SOC_DOUBLE_R_EXT(xname, reg_left, reg_right, xshift, xmax, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
					    xmax, xinvert) }

static const struct snd_kcontrol_new rt700_snd_controls[] = {
	SOC_DOUBLE_R_EXT_TLV("DAC Front Playback Volume", RT700_SET_GAIN_DAC1_H,
			    RT700_SET_GAIN_DAC1_L, RT700_DIR_OUT_SFT, 0x57, 0,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put,
			    out_vol_tlv),
	SOC_DOUBLE_R_EXT("ADC 08 Capture Switch", RT700_SET_GAIN_ADC2_H,
			    RT700_SET_GAIN_ADC2_L, RT700_DIR_IN_SFT, 1, 1,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put),
	SOC_DOUBLE_R_EXT("ADC 09 Capture Switch", RT700_SET_GAIN_ADC2_H,
			    RT700_SET_GAIN_ADC2_L, RT700_DIR_IN_SFT, 1, 1,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put),
	SOC_DOUBLE_R_EXT_TLV("ADC 08 Capture Volume", RT700_SET_GAIN_ADC2_H,
			    RT700_SET_GAIN_ADC2_L, RT700_DIR_IN_SFT, 0x3f, 0,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put,
			    in_vol_tlv),
	SOC_DOUBLE_R_EXT_TLV("ADC 09 Capture Volume", RT700_SET_GAIN_ADC1_H,
			    RT700_SET_GAIN_ADC1_L, RT700_DIR_IN_SFT, 0x3f, 0,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put,
			    in_vol_tlv),
	SOC_DOUBLE_R_EXT_TLV("AMIC Volume", RT700_SET_GAIN_AMIC_H,
			    RT700_SET_GAIN_AMIC_L, RT700_DIR_IN_SFT, 3, 0,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put,
			    mic_vol_tlv),
	SOC_DOUBLE_R_EXT("Speaker Playback Switch", RT700_SET_GAIN_SPK_H,
			    RT700_SET_GAIN_SPK_L, RT700_DIR_OUT_SFT, 1, 1,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put),
	SOC_DOUBLE_R_EXT("Headphone Playback Switch", RT700_SET_GAIN_HP_H,
			    RT700_SET_GAIN_HP_L, RT700_DIR_OUT_SFT, 1, 1,
			    rt700_set_amp_gain_get, rt700_set_amp_gain_put),
};

static int rt700_mux_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg, val;

	/* nid = e->reg, vid = 0xf01 */
	reg = RT700_VERB_GET_CONNECT_SEL | e->reg;
	snd_soc_write(codec, reg, 0x0);
	pr_debug("%s write %04x %02x\n", __func__, reg, 0x0);
	val = snd_soc_read(codec, RT700_READ_HDA_0);
	pr_debug("%s read %04x %02x\n", __func__, RT700_READ_HDA_0, val);
	ucontrol->value.enumerated.item[0] = val;

	return 0;
}

static int rt700_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm =
				snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_card *card = dapm->card;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	unsigned int val, val2, change, reg;

	if (item[0] >= e->items)
		return -EINVAL;

	/* Verb ID = 0x701h, nid = e->reg */
	val = snd_soc_enum_item_to_val(e, item[0]) << e->shift_l;
	pr_debug("%s val=%x e->reg=%x item[0]=%d\n",
		__func__, val, e->reg, item[0]);

	reg = RT700_VERB_GET_CONNECT_SEL | e->reg;
	snd_soc_write(codec, reg, 0x0);
	pr_debug("%s write %04x %02x\n", __func__, reg, 0x0);
	val2 = snd_soc_read(codec, RT700_READ_HDA_0);
	pr_debug("%s read %04x %02x\n", __func__, RT700_READ_HDA_0, val2);
	if (val == val2)
		change = 0;
	else
		change = 1;

	pr_debug("change=%d\n", change);

	if (change) {
		reg = RT700_VERB_SET_CONNECT_SEL | e->reg;
		snd_soc_write(codec, reg, val);
		pr_debug("%s write %04x %02x\n", __func__, reg, val);
		soc_dpcm_runtime_update(card);
	}

	return change;
}

static const char * const adc_mux_text[] = {
	"MIC2",
	"LINE1",
	"LINE2",
	"DMIC",
};

static const SOC_ENUM_SINGLE_DECL(
	rt700_adc22_enum, RT700_MIXER_IN1, 0, adc_mux_text);

static const SOC_ENUM_SINGLE_DECL(
	rt700_adc23_enum, RT700_MIXER_IN2, 0, adc_mux_text);

static const struct snd_kcontrol_new rt700_adc22_mux =
	SOC_DAPM_ENUM_EXT("ADC 22 Mux", rt700_adc22_enum,
			rt700_mux_put, rt700_mux_get);

static const struct snd_kcontrol_new rt700_adc23_mux =
	SOC_DAPM_ENUM_EXT("ADC 23 Mux", rt700_adc23_enum,
			rt700_mux_put, rt700_mux_get);

static const char * const out_mux_text[] = {
	"Front",
	"Surround",
};

static const SOC_ENUM_SINGLE_DECL(
	rt700_hp_enum, RT700_HP_OUT, 0, out_mux_text);

static const struct snd_kcontrol_new rt700_hp_mux =
	SOC_DAPM_ENUM_EXT("HP Mux", rt700_hp_enum,
			rt700_mux_put, rt700_mux_get);

static const struct snd_soc_dapm_widget rt700_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("HP"),
	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("LINE1"),
	SND_SOC_DAPM_INPUT("LINE2"),
	SND_SOC_DAPM_DAC("DAC Front", NULL, RT700_SET_STREAMID_DAC1, 4, 0),
	SND_SOC_DAPM_DAC("DAC Surround", NULL, RT700_SET_STREAMID_DAC2, 4, 0),
	SND_SOC_DAPM_MUX("HPO Mux", SND_SOC_NOPM, 0, 0, &rt700_hp_mux),
	SND_SOC_DAPM_PGA("SPK PGA", SND_SOC_NOPM, 0, 0,	NULL, 0),
	SND_SOC_DAPM_ADC("ADC 09", NULL, RT700_SET_STREAMID_ADC1, 4, 0),
	SND_SOC_DAPM_ADC("ADC 08", NULL, RT700_SET_STREAMID_ADC2, 4, 0),
	SND_SOC_DAPM_MUX("ADC 22 Mux", SND_SOC_NOPM, 0, 0,
		&rt700_adc22_mux),
	SND_SOC_DAPM_MUX("ADC 23 Mux", SND_SOC_NOPM, 0, 0,
		&rt700_adc23_mux),
	SND_SOC_DAPM_AIF_IN("DP1RX", "DP1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("DP3RX", "DP3 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DP2TX", "DP2 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DP4TX", "DP4 Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route rt700_audio_map[] = {
	{"DAC Front", NULL, "DP1RX"},
	{"DAC Surround", NULL, "DP3RX"},
	{"DP2TX", NULL, "ADC 09"},
	{"ADC 09", NULL, "ADC 22 Mux"},
	{"ADC 08", NULL, "ADC 23 Mux"},
	{"ADC 22 Mux", "DMIC", "DMIC1"},
	{"ADC 22 Mux", "LINE1", "LINE1"},
	{"ADC 22 Mux", "LINE2", "LINE2"},
	{"ADC 22 Mux", "MIC2", "MIC2"},
	{"ADC 23 Mux", "DMIC", "DMIC2"},
	{"ADC 23 Mux", "LINE1", "LINE1"},
	{"ADC 23 Mux", "LINE2", "LINE2"},
	{"ADC 23 Mux", "MIC2", "MIC2"},
	{"HPO Mux", "Front", "DAC Front"},
	{"HPO Mux", "Surround", "DAC Surround"},
	{"HP", NULL, "HPO Mux"},
	{"SPK PGA", NULL, "DAC Front"},
	{"SPK", NULL, "SPK PGA"},
};

static int rt700_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level)
{
	return 0;
}

static const struct snd_soc_codec_driver soc_codec_dev_rt700 = {
	.set_bias_level = rt700_set_bias_level,
	.controls = rt700_snd_controls,
	.num_controls = ARRAY_SIZE(rt700_snd_controls),
	.dapm_widgets = rt700_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt700_dapm_widgets),
	.dapm_routes = rt700_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rt700_audio_map),
};

static int rt700_program_stream_tag(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai, int stream_tag)
{
	struct sdw_stream_data *stream_data;

	stream_data = kzalloc(sizeof(*stream_data), GFP_KERNEL);
	if (!stream_data)
		return -ENOMEM;
	stream_data->stream_tag = stream_tag;
	snd_soc_dai_set_dma_data(dai, substream, stream_data);
	return 0;
}

static int rt700_remove_stream_tag(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)

{
	struct sdw_stream_data *stream_data;

	stream_data = snd_soc_dai_get_dma_data(dai, substream);
	kfree(stream_data);
	return 0;
}



static int rt700_pcm_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt700_priv *rt700 = snd_soc_codec_get_drvdata(codec);
	int retval;
	enum sdw_data_direction direction;
	struct sdw_stream_config stream_config;
	struct sdw_port_config port_config;
	struct sdw_port_cfg port_cfg;
	struct sdw_stream_data *stream;
	int port;
	int num_channels;
	int upscale_factor = 16;
	unsigned int val = 0;

	stream = snd_soc_dai_get_dma_data(dai, substream);

	if (!rt700->sdw)
		return 0;

	/* SoundWire specific configuration */
	/* This code assumes port 1 for playback and port 2 for capture */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		direction = SDW_DATA_DIR_IN;
		port = 1;
	} else {
		direction = SDW_DATA_DIR_OUT;
		port = 2;
	}
	switch (dai->id) {
	case RT700_AIF1:
		break;
	case RT700_AIF2:
		port += 2;
		break;
	default:
		dev_err(codec->dev, "Invalid DAI id %d\n", dai->id);
		return -EINVAL;
	}
	stream_config.frame_rate =  params_rate(params);
	stream_config.frame_rate *= upscale_factor;
	stream_config.channel_count = params_channels(params);
	stream_config.bps =
			snd_pcm_format_width(params_format(params));
	stream_config.direction = direction;
	retval = sdw_config_stream(rt700->sdw->mstr,
			rt700->sdw, &stream_config, stream->stream_tag);
	if (retval) {
		dev_err(dai->dev, "Unable to configure the stream\n");
		return retval;
	}
	port_config.num_ports = 1;
	port_config.port_cfg = &port_cfg;
	port_cfg.port_num = port;
	num_channels = params_channels(params);
	port_cfg.ch_mask = (1 << (num_channels))  - 1;
	retval = sdw_config_port(rt700->sdw->mstr, rt700->sdw,
		&port_config, stream->stream_tag);
	if (retval) {
		dev_err(dai->dev, "Unable to configure port\n");
		return retval;
	}

	switch (params_rate(params)) {
	/* bit 14 0:48K 1:44.1K */
	/* bit 15 Stream Type 0:PCM 1:Non-PCM, should always be PCM */
	case 44100:
		snd_soc_write(codec, RT700_DAC_FORMAT_H, 0x40);
		snd_soc_write(codec, RT700_ADC_FORMAT_H, 0x40);
		break;
	case 48000:
		snd_soc_write(codec, RT700_DAC_FORMAT_H, 0x0);
		snd_soc_write(codec, RT700_ADC_FORMAT_H, 0x0);
		break;
	default:
		dev_err(codec->dev, "Unsupported sample rate %d\n",
					params_rate(params));
		return -EINVAL;
	}

	if (params_channels(params) <= 16) {
		/* bit 3:0 Number of Channel */
		val |= (params_channels(params) - 1);
	} else {
		dev_err(codec->dev, "Unsupported channels %d\n",
					params_channels(params));
		return -EINVAL;
	}

	switch (params_width(params)) {
	/* bit 6:4 Bits per Sample */
	case 8:
		break;
	case 16:
		val |= (0x1 << 4);
		break;
	case 20:
		val |= (0x2 << 4);
		break;
	case 24:
		val |= (0x3 << 4);
		break;
	case 32:
		val |= (0x4 << 4);
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(codec->dev, "format val = 0x%x\n", val);

	snd_soc_write(codec, RT700_DAC_FORMAT_L, val);
	snd_soc_write(codec, RT700_ADC_FORMAT_L, val);

	return retval;
}

int rt700_pcm_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt700_priv *rt700 = snd_soc_codec_get_drvdata(codec);
	struct sdw_stream_data *stream = snd_soc_dai_get_dma_data(dai,
			substream);
	if (!rt700->sdw)
		return 0;
	sdw_release_stream(rt700->sdw->mstr, rt700->sdw, stream->stream_tag);
	return 0;
}

#define RT700_STEREO_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#define RT700_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8)

static struct snd_soc_dai_ops rt700_ops = {
	.hw_params	= rt700_pcm_hw_params,
	.hw_free	= rt700_pcm_hw_free,
	.program_stream_tag     = rt700_program_stream_tag,
	.remove_stream_tag      = rt700_remove_stream_tag,
};

static struct snd_soc_dai_driver rt700_dai[] = {
	{
		.name = "rt700-aif1",
		.id = RT700_AIF1,
		.playback = {
			.stream_name = "DP1 Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RT700_STEREO_RATES,
			.formats = RT700_FORMATS,
		},
		.capture = {
			.stream_name = "DP2 Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RT700_STEREO_RATES,
			.formats = RT700_FORMATS,
		},
		.ops = &rt700_ops,
	},
	{
		.name = "rt700-aif2",
		.id = RT700_AIF2,
		.playback = {
			.stream_name = "DP3 Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RT700_STEREO_RATES,
			.formats = RT700_FORMATS,
		},
		.capture = {
			.stream_name = "DP4 Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RT700_STEREO_RATES,
			.formats = RT700_FORMATS,
		},
		.ops = &rt700_ops,
	},
};

static ssize_t rt700_hda_cmd_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct rt700_priv *rt700 = dev_get_drvdata(dev);
	unsigned int sdw_addr_h, sdw_addr_l;
	unsigned int sdw_data_3, sdw_data_2, sdw_data_1, sdw_data_0;
	int cnt = 0;

	hda_to_sdw(rt700->dbg_nid, rt700->dbg_vid, rt700->dbg_payload,
		&sdw_addr_h, &sdw_data_1, &sdw_addr_l, &sdw_data_0);

	regmap_write(rt700->regmap, sdw_addr_h, sdw_data_1);
	if (!sdw_addr_l)
		regmap_write(rt700->regmap, sdw_addr_l, sdw_data_0);


	sdw_data_3 = 0;
	sdw_data_2 = 0;
	sdw_data_1 = 0;
	sdw_data_0 = 0;
	if (rt700->dbg_vid & 0x800) { /* get command */
		regmap_read(rt700->regmap, RT700_READ_HDA_3, &sdw_data_3);
		regmap_read(rt700->regmap, RT700_READ_HDA_2, &sdw_data_2);
		regmap_read(rt700->regmap, RT700_READ_HDA_1, &sdw_data_1);
		regmap_read(rt700->regmap, RT700_READ_HDA_0, &sdw_data_0);
		pr_info("read (%02x %03x %04x) = %02x%02x%02x%02x\n",
			rt700->dbg_nid, rt700->dbg_vid, rt700->dbg_payload,
			sdw_data_3, sdw_data_2, sdw_data_1, sdw_data_0);
	}
	cnt += snprintf(buf, PAGE_SIZE - 1,
			"read (%02x %03x %04x) = %02x%02x%02x%02x\n",
			rt700->dbg_nid, rt700->dbg_vid, rt700->dbg_payload,
			sdw_data_3, sdw_data_2, sdw_data_1, sdw_data_0);


	if (cnt >= PAGE_SIZE)
		cnt = PAGE_SIZE - 1;

	return cnt;
}

static ssize_t rt700_hda_cmd_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct rt700_priv *rt700 = dev_get_drvdata(dev);
	unsigned int sdw_addr_h, sdw_addr_l, sdw_data_h, sdw_data_l;
	unsigned int sdw_data_3, sdw_data_2, sdw_data_1, sdw_data_0;
	int i;

	pr_debug("register \"%s\" count=%zu\n", buf, count);
	for (i = 0; i < count; i++) {	/*rt700->dbg_nidess */
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			rt700->dbg_nid = (rt700->dbg_nid << 4) |
						(*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			rt700->dbg_nid = (rt700->dbg_nid << 4) |
						((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			rt700->dbg_nid = (rt700->dbg_nid << 4) |
						((*(buf + i) - 'A') + 0xa);
		else
			break;
	}

	for (i = i + 1; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			rt700->dbg_vid = (rt700->dbg_vid << 4) |
						(*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			rt700->dbg_vid = (rt700->dbg_vid << 4) |
						((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			rt700->dbg_vid = (rt700->dbg_vid << 4) |
						((*(buf + i) - 'A') + 0xa);
		else
			break;
	}

	if (rt700->dbg_vid < 0xf)
		rt700->dbg_vid = rt700->dbg_vid << 8;

	for (i = i + 1; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			rt700->dbg_payload = (rt700->dbg_payload << 4) |
						(*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			rt700->dbg_payload = (rt700->dbg_payload << 4) |
						((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			rt700->dbg_payload = (rt700->dbg_payload << 4) |
						((*(buf + i) - 'A') + 0xa);
		else
			break;
	}
	pr_debug("dbg_nid=0x%x dbg_vid=0x%x dbg_payload=0x%x\n",
		rt700->dbg_nid, rt700->dbg_vid, rt700->dbg_payload);

	hda_to_sdw(rt700->dbg_nid, rt700->dbg_vid, rt700->dbg_payload,
		&sdw_addr_h, &sdw_data_h, &sdw_addr_l, &sdw_data_l);

	regmap_write(rt700->regmap, sdw_addr_h, sdw_data_h);
	if (!sdw_addr_l)
		regmap_write(rt700->regmap, sdw_addr_l, sdw_data_l);


	sdw_data_3 = 0;
	sdw_data_2 = 0;
	sdw_data_1 = 0;
	sdw_data_0 = 0;
	if (rt700->dbg_vid & 0x800) { /* get command */
		regmap_read(rt700->regmap, RT700_READ_HDA_3, &sdw_data_3);
		regmap_read(rt700->regmap, RT700_READ_HDA_2, &sdw_data_2);
		regmap_read(rt700->regmap, RT700_READ_HDA_1, &sdw_data_1);
		regmap_read(rt700->regmap, RT700_READ_HDA_0, &sdw_data_0);
		pr_info("read (%02x %03x %04x) = %02x%02x%02x%02x\n",
			rt700->dbg_nid, rt700->dbg_vid, rt700->dbg_payload,
			sdw_data_3, sdw_data_2, sdw_data_1, sdw_data_0);
	}


	return count;
}

static DEVICE_ATTR(hda_reg, 0664, rt700_hda_cmd_show, rt700_hda_cmd_store);

int rt700_probe(struct device *dev, struct regmap *regmap,
					struct sdw_slave *slave)
{
	struct rt700_priv *rt700;
	int ret;

	rt700 = devm_kzalloc(dev, sizeof(struct rt700_priv),
			       GFP_KERNEL);
	if (!rt700)
		return -ENOMEM;

	dev_set_drvdata(dev, rt700);

	rt700->regmap = regmap;
	rt700->sdw = slave;

	ret =  snd_soc_register_codec(dev,
		&soc_codec_dev_rt700, rt700_dai, ARRAY_SIZE(rt700_dai));
	dev_info(&slave->dev, "%s\n", __func__);

	/* Set Tx route */
	/* Filter 02: index 91[13:12] 07[3] */
	/* Filter 03: index 5f[15:14] 07[4] */
	/* DAC (02) -> Front ->  SPK (14)*/
	/* DAC (03) -> Surr ->  HP (14)*/

	/* Set Rx route */
	/* Rx_09: index 91[8] */
	/* Rx_08: index 91[6] */
	/* Mic2 (19) -> Mux (22) -> ADC (09) */
	regmap_write(rt700->regmap, 0x3122, 0x0); /* Mic2 (19) -> Mux (22)*/

	/* Assign stream ID */
	/* do it in dapm widget
	regmap_write(rt700->regmap,  RT700_SET_STREAMID_DAC1, 0x10);
	regmap_write(rt700->regmap,  RT700_SET_STREAMID_DAC2, 0x10);
	regmap_write(rt700->regmap,  RT700_SET_STREAMID_ADC2, 0x10);
	regmap_write(rt700->regmap,  RT700_SET_STREAMID_ADC1, 0x10);
	*/

	/* Set Pin Widget */
	regmap_write(rt700->regmap,  RT700_SET_PIN_HP, 0x40);
	regmap_write(rt700->regmap,  RT700_SET_PIN_SPK, 0x40);
	regmap_write(rt700->regmap,  0x3c14, 0x02); /* 14 70c 02 */
	regmap_write(rt700->regmap,  RT700_SET_PIN_DMIC1, 0x20);
	regmap_write(rt700->regmap,  RT700_SET_PIN_DMIC2, 0x20);
	regmap_write(rt700->regmap,  RT700_SET_PIN_MIC2, 0x20);
	regmap_write(rt700->regmap,  RT700_SET_PIN_LINE1, 0x20);
	regmap_write(rt700->regmap,  RT700_SET_PIN_LINE2, 0x20);

	/* Set index */
	rt700_index_write(rt700->regmap, 0x4a, 0x201b);
	rt700_index_write(rt700->regmap, 0x38, 0x4921);


	ret = device_create_file(&slave->dev, &dev_attr_hda_reg);
	if (ret != 0) {
		dev_err(&slave->dev,
			"Failed to create codex_reg sysfs files: %d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(rt700_probe);

int rt700_remove(struct device *dev)
{

	snd_soc_unregister_codec(dev);

	dev_info(dev, "Removing\n");

	return 0;
}
EXPORT_SYMBOL(rt700_remove);

#ifdef CONFIG_PM
static int rt700_runtime_suspend(struct device *dev)
{
	return 0;
}

static int rt700_runtime_resume(struct device *dev)
{

	return 0;
}
#endif

const struct dev_pm_ops rt700_runtime_pm = {
	SET_RUNTIME_PM_OPS(rt700_runtime_suspend, rt700_runtime_resume,
			   NULL)
};
EXPORT_SYMBOL(rt700_runtime_pm);

MODULE_DESCRIPTION("ASoC rt700 driver");
MODULE_DESCRIPTION("ASoC rt700 driver SDW");
MODULE_AUTHOR("Bard Liao <bardliao@realtek.com>");
MODULE_LICENSE("GPL");
