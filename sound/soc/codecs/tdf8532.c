/*
 * Codec driver for NXP Semiconductors - TDF8532
 * Copyright (c) 2017, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "tdf8532.h"

static DEFINE_MUTEX(tdf8532_lock);

static int __tdf8532_build_pkt(struct tdf8532_priv *priv, va_list valist, u8 *payload)
{
	const u8 cmd_offset = 3;
	int param;
	u8 *cmd_payload, len;

	payload[HEADER_TYPE] = MSG_TYPE_STX;
	payload[HEADER_PKTID] = priv->pkt_id;

	cmd_payload = &(payload[cmd_offset]);
	param = va_arg(valist, int);
	len = 0;

	while (param != END) {
		cmd_payload[len] = param;
		len++;
		param = va_arg(valist, int);
	}

	payload[HEADER_LEN] = len;

	return len + cmd_offset;
}

static int __tdf8532_single_write(struct tdf8532_priv *priv, int dummy, ...)
{
	va_list valist;
	int ret;
	u8 payload[255];
	u8 len;

	va_start(valist, dummy);
	len = __tdf8532_build_pkt(priv, valist, payload);
	va_end(valist);

	print_hex_dump_debug("tdf8532: Tx:", DUMP_PREFIX_NONE, 32, 1, payload, len, false);
	ret = i2c_master_send(priv->i2c, payload, len);
	priv->pkt_id++;

	if (ret < 0) {
		dev_err(&priv->i2c->dev, "i2c send packet returned: %d\n", ret);
		return ret;
	}

	return 0;
}

static int tdf8532_read_wait_ack(struct tdf8532_priv *priv, unsigned long timeout)
{
	unsigned long timeout_point = jiffies + timeout;
	uint8_t ack_repl[HEADER_SIZE] = {0, 0, 0};
	int ret;

	usleep_range(10000, 20000);

	do {
		ret = i2c_master_recv(priv->i2c, ack_repl, HEADER_SIZE);
		if (ret < 0)
			return ret;
	} while (time_before(jiffies, timeout_point) && ack_repl[0] != MSG_TYPE_ACK);

	if (ack_repl[0] != MSG_TYPE_ACK)
		return -ETIME;
	return ack_repl[2];
}

static int tdf8532_single_read(struct tdf8532_priv *priv, struct get_dev_status_repl *reply)
{
	struct device *dev = &priv->i2c->dev;
	size_t len;
	char *buf;
	int ret;

	ret = tdf8532_read_wait_ack(priv, msecs_to_jiffies(ACK_TIMEOUT));
	if (ret < 0) {
		dev_err(dev, "Error waiting for ACK reply: %d\n", ret);
		return ret;
	}

	len = ret + HEADER_SIZE;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = i2c_master_recv(priv->i2c, buf, len);
	print_hex_dump_debug("tdf8532: Rx:", DUMP_PREFIX_NONE, 32, 1, buf, len, false);

	if (ret < 0 || ret != len) {
		dev_err(dev, "i2c recv packet returned: %d (expected: %zu)\n", ret, len);
		ret = -EINVAL;
	} else {
		memcpy(reply, buf, min(sizeof(*reply), len));
		ret = len;
	}

	kfree(buf);
	return ret;
}

static int tdf8532_get_state(struct tdf8532_priv *priv, struct get_dev_status_repl *reply)
{
	int ret;

	ret = tdf8532_amp_write(priv, GET_DEV_STATUS);
	if (ret < 0)
		return ret;

	return tdf8532_single_read(priv, reply);
}

static int tdf8532_wait_state(struct tdf8532_priv *priv, u8 req_state, unsigned long timeout)
{
	struct get_dev_status_repl status;
	unsigned long timeout_point;
	int ret;

	timeout_point = jiffies + msecs_to_jiffies(timeout);

	do {
		memset(&status, 0, sizeof(status));

		ret = tdf8532_get_state(priv, &status);
		if (ret < 0)
			goto out;

		print_hex_dump_debug("tdf8532: wait_state: ", DUMP_PREFIX_NONE, 32, 1, &status,
				     sizeof(status), false);

		if (status.state == req_state)
			return 0;
	} while (time_before(jiffies, timeout_point));

out:
	dev_err(&priv->i2c->dev, "tdf8532: state: %u, req_state: %u, ret: %d\n", status.state,
		req_state, ret);
	return -ETIME;
}

static int tdf8532_start_play(struct tdf8532_priv *tdf8532)
{
	int ret;

	mutex_lock(&tdf8532_lock);
	ret = tdf8532_amp_write(tdf8532, SET_CLK_STATE, CLK_CONNECT);
	if (ret < 0)
		goto out;

	ret = tdf8532_amp_write(tdf8532, SET_CHNL_ENABLE, CHNL_MASK(tdf8532->channels));
	if (ret < 0)
		goto out;

	ret = tdf8532_wait_state(tdf8532, STATE_PLAY, ACK_TIMEOUT);

out:
	mutex_unlock(&tdf8532_lock);
	return ret;
}

static int tdf8532_stop_play(struct tdf8532_priv *tdf8532)
{
	int ret;

	mutex_lock(&tdf8532_lock);
	ret = tdf8532_amp_write(tdf8532, SET_CHNL_DISABLE, CHNL_MASK(tdf8532->channels));
	if (ret < 0)
		goto out;

	tdf8532_wait_state(tdf8532, STATE_STBY, ACK_TIMEOUT);

	ret = tdf8532_amp_write(tdf8532, SET_CLK_STATE, CLK_DISCONNECT);
	if (ret < 0)
		goto out;

	ret = tdf8532_wait_state(tdf8532, STATE_IDLE, ACK_TIMEOUT);

out:
	mutex_unlock(&tdf8532_lock);
	return ret;
}

static int tdf8532_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct tdf8532_priv *tdf8532 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "%s: cmd: %d, stream dir: %d\n", __func__, cmd, substream->stream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		ret = tdf8532_start_play(tdf8532);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		/*
		 * WA on unexpected codec down during S3
		 * SNDRV_PCM_TRIGGER_STOP fails so skip assigning ret
		 */
		tdf8532_stop_play(tdf8532);
		break;
	}

	return ret;
}

static int tdf8532_dai_mute_stream(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct tdf8532_priv *tdf8532 = snd_soc_component_get_drvdata(component);
	int ret;

	dev_dbg(component->dev, "%s: mute: %d\n", __func__, mute);

	mutex_lock(&tdf8532_lock);
	ret = tdf8532_amp_write(tdf8532, mute ? SET_CHNL_MUTE : SET_CHNL_UNMUTE,
				CHNL_MASK(CHNL_MAX));
	mutex_unlock(&tdf8532_lock);

	return ret;
}

static const struct snd_soc_dai_ops tdf8532_dai_ops = {
	.trigger	= tdf8532_dai_trigger,
	.mute_stream	= tdf8532_dai_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_component_driver soc_component_tdf8532;

static struct snd_soc_dai_driver tdf8532_dais[] = {
	{
		.name = "tdf8532-hifi",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 4,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tdf8532_dai_ops,
	}
};

static int tdf8532_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct tdf8532_priv *dev_data;
	struct device *dev = &i2c->dev;
	int ret;

	dev_dbg(&i2c->dev, "%s\n", __func__);

	dev_data = devm_kzalloc(dev, sizeof(struct tdf8532_priv), GFP_KERNEL);
	if (!dev_data)
		return -ENOMEM;

	dev_data->i2c = i2c;
	dev_data->pkt_id = 0;
	dev_data->channels = 4;

	i2c_set_clientdata(i2c, dev_data);

	ret = devm_snd_soc_register_component(&i2c->dev, &soc_component_tdf8532, tdf8532_dais,
					      ARRAY_SIZE(tdf8532_dais));
	if (ret)
		dev_err(&i2c->dev, "Failed to register codec: %d\n", ret);

	return ret;
}

static const struct i2c_device_id tdf8532_i2c_id[] = {
	{ "tdf8532", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tdf8532_i2c_id);

static const struct acpi_device_id tdf8532_acpi_match[] = {
	{ "INT34C3", 0 },
	{}
};
MODULE_DEVICE_TABLE(acpi, tdf8532_acpi_match);

static struct i2c_driver tdf8532_i2c_driver = {
	.driver = {
		.name = "tdf8532",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(tdf8532_acpi_match),
	},
	.probe = tdf8532_i2c_probe,
	.id_table = tdf8532_i2c_id,
};
module_i2c_driver(tdf8532_i2c_driver);

MODULE_DESCRIPTION("ASoC NXP Semiconductors TDF8532 driver");
MODULE_AUTHOR("Steffen Wagner <steffen.wagner@intel.com>");
MODULE_AUTHOR("Cezary Rojewski <cezary.rojewski@intel.com>");
MODULE_LICENSE("GPL");
