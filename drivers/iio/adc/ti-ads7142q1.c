// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI ADS7142 12-bit ADC driver
 *
 * Copyright (C) 2021 Intel
 *
 * Datasheets:
 *	https://www.ti.com/lit/ds/symlink/ads7142-q1.pdf
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regulator/consumer.h>

/* ADS7142 Opcodes for commands */
/* Single Register Read */
#define ADS7142_SINGLE_REG_READ			0x10
/* Single Register Write */
#define ADS7142_SINGLE_REG_WRITE		0x08

/* ADS7142 registers */
#define ADS7142_REG_ABORT_SEQUENCE                                  0x1F
#define ADS7142_VAL_ABORT_SEQUENCE                                  0x01
#define ADS7142_REG_OFFSET_CAL                                      0x15
#define ADS7142_VAL_TRIG_OFFCAL                                     0x01
#define ADS7142_REG_CHANNEL_INPUT_CFG                               0x24
#define ADS7142_VAL_CHANNEL_INPUT_CFG_2_CHANNEL_SINGLE_ENDED        0x03
#define ADS7142_REG_OPMODE_SEL                                      0x1C
#define ADS7142_VAL_OPMODE_SEL_AUTONOMOUS_MONITORING_MODE           0x06
#define ADS7142_REG_AUTO_SEQ_CHEN                                   0x20
#define ADS7142_VAL_AUTO_SEQ_CHENAUTO_SEQ_CH0_CH1                   0x03
#define ADS7142_REG_OSC_SEL                                         0x18
#define ADS7142_VAL_OSC_SEL_HSZ_HSO                                 0x00
#define ADS7142_REG_nCLK_SEL                                        0x19
#define ADS7142_REG_DOUT_FORMAT_CFG                                 0x28
#define ADS7142_VAL_DOUT_FORMAT_CFG_DOUT_FORMAT2                    0x02
#define ADS7142_REG_DATA_BUFFER_OPMODE                              0x2C
#define ADS7142_VAL_DATA_BUFFER_STARTSTOP_CNTRL_STARTBURST          0x01
#define ADS7142_REG_START_SEQUENCE                                  0x1E
#define ADS7142_VAL_START_SEQUENCE                                  0x01
#define ADS7142_REG_DATA_BUFFER_STATUS                              0x01
#define ADS7142_REG_SEQUENCE_STATUS                                 0x04

struct ads7142 {
	struct i2c_client *i2c;
	int bits;
	struct {
		u16 channel;
		s64 ts __aligned(8);
	} scan;
};

static int ads7142_reg_write(const struct i2c_client *client, u8 reg, u8 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = ADS7142_SINGLE_REG_WRITE;
	buf[1] = reg;
	buf[2] = data;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret >= 0 ? 0 : ret;
}

static int ads7142_reg_read(const struct i2c_client *client, u8 reg, u8 *data)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = ADS7142_SINGLE_REG_READ;
	buf[1] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	ret = i2c_transfer(client->adapter, msg, 2);

	return ret >= 0 ? 0 : ret;
}

static int ads7142_data_buffer_read(const struct i2c_client *client, int length, void *data)
{
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret >= 0 ? 0 : ret;
}

static void ads7142_raw_data_avg_value(u8 *adc_data, int *value)
{
	int count;
	int avg_value;
	int avg_count;
	u16 adc_16bit_data[16];

	count = 0;
	do {
		adc_16bit_data[count / 2] = (((adc_data[count] << 8) | (adc_data[count + 1])) >> 4);
		count = (count + 2);
	} while (count < 32);

	count = 0;
	avg_value = 0;
	avg_count = 0;
	do {
		/* add only non-zero adc raw value for average manipulation */
		if (0 != adc_16bit_data[count]) {
			avg_value += adc_16bit_data[count];
			avg_count++;
		}
		count++;
	} while (count < 16);

	avg_value = (avg_value/avg_count);

	*value = avg_value;
}

static int ads7142_read_raw(struct iio_dev *iio,
			    struct iio_chan_spec const *channel, int *value,
			    int *value2, long mask)
{
	struct ads7142 *adc = iio_priv(iio);
	u8 adc_data[32];
	u8 data;
	int err;

	switch (mask) {
	case IIO_CHAN_INFO_AVERAGE_RAW:
		//mutex_lock(&priv->lock);
		/* Abort the present sequence */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_ABORT_SEQUENCE, ADS7142_VAL_ABORT_SEQUENCE);
		err = ads7142_reg_read(adc->i2c, ADS7142_REG_ABORT_SEQUENCE, &data);

		/* Perform Offset Calibration */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_OFFSET_CAL, ADS7142_VAL_TRIG_OFFCAL);

		/* Select Autonomous Mode with both Channels enabled */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_CHANNEL_INPUT_CFG, ADS7142_VAL_CHANNEL_INPUT_CFG_2_CHANNEL_SINGLE_ENDED);

		/* Select the operation mode of the device */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_OPMODE_SEL, ADS7142_VAL_OPMODE_SEL_AUTONOMOUS_MONITORING_MODE);

		/* Select the channel for auto sequencing */
		/* channel_0 -> bit 0 to be set and channel_1 -> bit 1 to be set */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_AUTO_SEQ_CHEN, (1 << channel->channel));

		/* Select the low power or high speed oscillator */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_OSC_SEL, ADS7142_VAL_OSC_SEL_HSZ_HSO);

		/* Set the minimum nCLK value for one conversion to maximize sampling speed */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_nCLK_SEL, 21);

		/* Select the Data Buffer output data Configuration */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_DOUT_FORMAT_CFG, ADS7142_VAL_DOUT_FORMAT_CFG_DOUT_FORMAT2);

		/* Select the Data Buffer opmode for Start Burst mode */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_DATA_BUFFER_OPMODE, ADS7142_VAL_DATA_BUFFER_STARTSTOP_CNTRL_STARTBURST);

		/* Set the SEQ_START Bit to begin the sampling sequence */
		err = ads7142_reg_write(adc->i2c, ADS7142_REG_START_SEQUENCE, ADS7142_VAL_START_SEQUENCE);

		msleep(500);

		/* read sampling raw data from internal data buffer */
		err = ads7142_data_buffer_read(adc->i2c, 32, adc_data);

		/* do average of raw data value */
		ads7142_raw_data_avg_value(adc_data, value);
		return IIO_VAL_INT;

	default:
		break;

	}

	return -EINVAL;
}

#define RAW_DATA_BITS 12

#define ADS7142_CHAN(_index) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW),	\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = RAW_DATA_BITS,			\
		.storagebits = 16,				\
		.shift = 12 - RAW_DATA_BITS,			\
		.endianness = IIO_CPU,				\
	},							\
}

static const struct iio_chan_spec ads7142_channels[] = {
	ADS7142_CHAN(0),
	ADS7142_CHAN(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

#define ADS7142_NUM_CHANNELS 3

static const struct iio_info ads7142_info = {
	.read_raw = ads7142_read_raw,
};

static int ads7142_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct ads7142 *adc;
	int err;

	printk(KERN_INFO "Message: %s Entry\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	iio = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!iio)
		return -ENOMEM;

	adc = iio_priv(iio);
	adc->i2c = client;
	adc->bits = RAW_DATA_BITS;

	iio->name = dev_name(&client->dev);
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &ads7142_info;

	iio->channels = ads7142_channels;
	iio->num_channels = ADS7142_NUM_CHANNELS;

	err = iio_triggered_buffer_setup(iio, NULL, NULL, NULL);
	if (err < 0) {
		dev_err(&client->dev, "iio triggered buffer setup failed\n");
		return err;
	}

	err = iio_device_register(iio);
	if (err < 0)
		goto err_buffer_cleanup;

	i2c_set_clientdata(client, iio);

	printk(KERN_INFO "Message: %s Exit_1\n", __func__);

	return 0;

err_buffer_cleanup:
	iio_triggered_buffer_cleanup(iio);

	printk(KERN_INFO "Message: %s Exit_2\n", __func__);
	return err;
}

static int ads7142_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);

	iio_device_unregister(iio);
	iio_triggered_buffer_cleanup(iio);

	return 0;
}

enum ads7142_model_id {
	ADS7142,
};

static const struct i2c_device_id ads7142_id[] = {
	{ "ads7142", ADS7142 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads7142_id);

static const struct of_device_id ads7142_of_match[] = {
	{ .compatible = "ti,ads7142" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads7142_of_match);

static struct i2c_driver ads7142_driver = {
	.driver = {
		.name = "ads7142",
		.of_match_table = ads7142_of_match,
	},
	.probe = ads7142_probe,
	.remove = ads7142_remove,
	.id_table = ads7142_id,
};
module_i2c_driver(ads7142_driver);

MODULE_AUTHOR("MP, Sureshkumar <sureshkumar.mp@intel.com>");
MODULE_DESCRIPTION("Texas Instruments ADS7142 driver");
MODULE_LICENSE("GPL v2");
