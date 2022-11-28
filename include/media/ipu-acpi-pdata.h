/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 Intel Corporation */
#include <linux/interrupt.h>
#include <media/ipu-acpi.h>
#include <media/ar0234.h>
#include <media/lt6911uxc.h>
#include <media/ti960.h>
#include <media/imx390.h>
#include <media/d4xx.h>

#define DUMMY_SERDES_BASE_ADDR 0x40
#define CL_EMPTY 0
#define CL_DISCRETE 1
#define SERDES_MAX_PORT 4
#define SERDES_MAX_GPIO_POWERUP_SEQ 4
#define LOOP_SIZE 10

int get_sensor_pdata(struct i2c_client *client,
			struct ipu_camera_module_data *data,
			struct ipu_i2c_helper *helper,
			void *priv, size_t size,
			enum connection_type connect,
			const char *serdes_name);

struct ipu_isys_subdev_pdata *get_acpi_subdev_pdata(void);

struct sensor_platform_data {
	unsigned int port;
	unsigned int lanes;
	uint32_t i2c_slave_address;
	int irq_pin;
	unsigned int irq_pin_flags;
	char irq_pin_name[IPU_SPLATA_IRQ_PIN_NAME_LEN];
	int reset_pin;
	int detect_pin;
	char suffix;
	int gpios[IPU_SPLATA_GPIO_NUM];
};

struct serdes_platform_data {
	unsigned int subdev_num;
	struct serdes_subdev_info *subdev_info;
	unsigned int reset_gpio;
	unsigned int FPD_gpio;
	char suffix;
};

struct serdes_subdev_info {
	struct i2c_board_info board_info;
	int i2c_adapter_id;
	unsigned short rx_port;
	unsigned short phy_i2c_addr;
	unsigned short ser_alias;
	char suffix; /* suffix for subdevs */
};

struct serdes_module_pdata {
	unsigned short i2c_addr;
	unsigned short i2c_adapter;
	unsigned int lanes;
	int xshutdown;
	int fsin;
	int reset;
	char gpio_powerup_seq[SERDES_MAX_GPIO_POWERUP_SEQ];
	unsigned int module_flags;
	char module_name[I2C_NAME_SIZE];
	char suffix;
};

struct serdes_local {
	unsigned int rx_port;
	unsigned short sensor_addr;
	unsigned int gpio_powerup_seq;
	unsigned int module_flags;
	unsigned int sensor_num;
};
