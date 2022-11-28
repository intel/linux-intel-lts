// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016--2022 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <media/ipu-acpi.h>
#include <media/ipu-acpi-pdata.h>

#define MIN_SENSOR_I2C 1
#define MIN_SERDES_I2C 2
#define SUFFIX_BASE 96
#define MSG_LEN 128

static struct ipu_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

struct ipu_isys_subdev_pdata acpi_subdev_pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
		NULL,
	},
	.clk_map = clk_mapping,
};

struct serdes_local serdes_info;

struct serdes_subdev_info serdes_sdinfo[SERDES_MAX_PORT];

struct ipu_isys_subdev_pdata *get_acpi_subdev_pdata(void)
{
	struct ipu_isys_subdev_pdata *ptr;
	ptr = &acpi_subdev_pdata;
	return ptr;
}
EXPORT_SYMBOL(get_acpi_subdev_pdata);

void print_serdes_sdinfo(struct serdes_subdev_info *sdinfo)
{
	int j;
	struct serdes_module_pdata *sd_mpdata = sdinfo->board_info.platform_data;

	if (!sd_mpdata) {
		pr_err("Empty serdes module pdata");
		return;
	}

	pr_debug("\t\trx_port \t\t= %d", sdinfo->rx_port);
	pr_debug("\t\tphy_i2c_addr \t\t= 0x%x", sdinfo->phy_i2c_addr);
	pr_debug("\t\tser_alias \t\t= 0x%x", sdinfo->ser_alias);
	pr_debug("\t\tsuffix \t\t\t= %c", sdinfo->suffix);
	pr_debug("\t\tboard_info.type \t= %s", sdinfo->board_info.type);
	pr_debug("\t\tboard_info.addr \t= 0x%x", sdinfo->board_info.addr);

	pr_debug("serdes board_info.platform_data");
	pr_debug("\t\tlanes \t\t\t= %d", sd_mpdata->lanes);
	pr_debug("\t\tmodule_name \t\t= %s", sd_mpdata->module_name);
	pr_debug("\t\tfsin \t\t\t= %d", sd_mpdata->fsin);

	if (serdes_info.gpio_powerup_seq > 0)
		for (j = 0; j < serdes_info.gpio_powerup_seq; j++)
			pr_debug("\t\t gpio_powerup_seq[%d] \t= %d", j,
				(int)sd_mpdata->gpio_powerup_seq[j]);
}

void print_serdes_subdev(struct ipu_isys_subdev_info *sd)
{
	struct serdes_platform_data *sd_pdata = sd->i2c.board_info.platform_data;
	int i;
	struct serdes_subdev_info *sd_sdinfo;
	struct serdes_module_pdata *sd_mpdata;

	if (!sd_pdata) {
		pr_err("Empty serdes subdev pdata");
		return;
	}

	pr_debug("IPU6 ACPI %s", __func__);
	pr_debug("sd_csi2");
	pr_debug("\t\tnlanes \t\t\t= %d", sd->csi2->nlanes);
	pr_debug("\t\tport \t\t\t= %d", sd->csi2->port);

	pr_debug("sd->i2c");
	pr_debug("\t\ti2c_adapter_bdf \t= %s", sd->i2c.i2c_adapter_bdf);
	pr_debug("\t\tboard_info.type \t= %s", sd->i2c.board_info.type);
	pr_debug("\t\tboard_info.addr \t= 0x%x", sd->i2c.board_info.addr);

	pr_debug("sd->i2c.board_info.platform_data");
	pr_debug("\t\treset_gpio \t\t= %d", sd_pdata->reset_gpio);
	pr_debug("\t\tFPD_gpio \t\t= %d", sd_pdata->FPD_gpio);
	pr_debug("\t\tsuffix \t\t\t= %c", sd_pdata->suffix);

	for (i = 0; i < serdes_info.rx_port; i++) {
		sd_sdinfo = &sd_pdata->subdev_info[i];
		sd_mpdata = sd_sdinfo->board_info.platform_data;

		if (!sd_mpdata)
			continue;

		pr_debug("serdes subdev_info[%d]", i);
		print_serdes_sdinfo(sd_sdinfo);
	}

}

void print_subdev(struct ipu_isys_subdev_info *sd)
{
	struct sensor_platform_data *spdata = sd->i2c.board_info.platform_data;
	int i;

	if (!spdata) {
		pr_err("Empty sensor subdev");
		return;
	}

	pr_debug("IPU6 ACPI %s", __func__);
	pr_debug("sd->csi2");
	pr_debug("\t\tnlanes \t\t\t= %d", sd->csi2->nlanes);
	pr_debug("\t\tport \t\t\t= %d", sd->csi2->port);

	pr_debug("sd->i2c");
	pr_debug("\t\ti2c_adapter_bdf \t= %s", sd->i2c.i2c_adapter_bdf);
	pr_debug("\t\tboard_info.type \t= %s", sd->i2c.board_info.type);
	pr_debug("\t\tboard_info.addr \t= 0x%x", sd->i2c.board_info.addr);

	pr_debug("sd->i2c.platform_data");
	pr_debug("\t\tport \t\t\t= %d", spdata->port);
	pr_debug("\t\tlanes \t\t\t= %d", spdata->lanes);
	pr_debug("\t\ti2c_slave_address \t= 0x%x", spdata->i2c_slave_address);
	pr_debug("\t\tirq_pin \t\t= %d", spdata->irq_pin);
	pr_debug("\t\tirq_pin_name \t\t= %s", spdata->irq_pin_name);
	pr_debug("\t\tsuffix \t\t\t= %c", spdata->suffix);
	pr_debug("\t\treset_pin \t\t= %d", spdata->reset_pin);
	pr_debug("\t\tdetect_pin \t\t= %d", spdata->detect_pin);

	for (i = 0; i < IPU_SPLATA_GPIO_NUM; i++)
		pr_debug("\t\tgpios[%d] \t\t= %d", i, spdata->gpios[i]);
}

void add_local_subdevs(struct ipu_isys_subdev_info *new_subdev_info)
{
	struct ipu_isys_subdev_pdata *ptr_acpi_subdev_pdata = &acpi_subdev_pdata;
	int i = 0;

	while (i <= MAX_ACPI_SENSOR_NUM) {
		if (!ptr_acpi_subdev_pdata->subdevs[i]) {
			ptr_acpi_subdev_pdata->subdevs[i] = new_subdev_info;
			ptr_acpi_subdev_pdata->subdevs[i+1] = NULL;
			break;
		}
		i++;
	}
}

void update_short(struct device *dev,
		char msg[MSG_LEN],
		unsigned short *old_short,
		unsigned int new_short)
{
	if (*old_short != new_short) {
		dev_info(dev, "%s 0x%x -> 0x%x", msg, *old_short, new_short);
		*old_short = new_short;
	}
}

void update_hex(struct device *dev,
		char msg[MSG_LEN],
		unsigned int *old_hex,
		unsigned int new_hex)
{
	if (*old_hex != new_hex) {
		dev_info(dev, "%s 0x%x -> 0x%x", msg, *old_hex, new_hex);
		*old_hex = new_hex;
	}
}

void update_int(struct device *dev,
		char msg[MSG_LEN],
		unsigned int *old_int,
		unsigned int new_int)
{
	if (*old_int != new_int) {
		dev_info(dev, "%s %d -> %d", msg, *old_int, new_int);
		*old_int = new_int;
	}
}

void update_inta(struct device *dev,
		char msg[MSG_LEN],
		int old_int[MSG_LEN],
		int new_int[MSG_LEN],
		size_t size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (old_int[i] != new_int[i]) {
			dev_info(dev, "%s %d -> %d", msg, old_int[i], new_int[i]);
			old_int[i] = new_int[i];
		}
	}
}

void update_str(struct device *dev,
		char msg[MSG_LEN],
		char old_str[MSG_LEN],
		char new_str[MSG_LEN])
{
	if (strcmp(old_str, new_str) != 0) {
		dev_info(dev, "%s %s -> %s", msg, old_str, new_str);
		strlcpy(old_str, new_str, strlen(new_str)+1);
	}
}

void update_subdev(struct device *dev,
			struct ipu_isys_subdev_info *new_sd,
			struct ipu_isys_subdev_info **old_sd)
{
	struct sensor_platform_data *old_pdata =
					(*old_sd)->i2c.board_info.platform_data;

	struct sensor_platform_data *new_pdata =
					new_sd->i2c.board_info.platform_data;

	/* csi2 */
	update_int(dev, "CSI2 port", &(*old_sd)->csi2->port, new_sd->csi2->port);
	update_int(dev, "CSI2 nlanes", &(*old_sd)->csi2->nlanes, new_sd->csi2->nlanes);

	/* i2c */
	update_short(dev, "I2C board_info addr", &(*old_sd)->i2c.board_info.addr,
		new_sd->i2c.board_info.addr);
	update_str(dev, "I2C i2c_adapter_bdf", (*old_sd)->i2c.i2c_adapter_bdf,
		new_sd->i2c.i2c_adapter_bdf);

	/* platform data */
	update_int(dev, "pdata port", &(old_pdata)->port, new_pdata->port);
	update_int(dev, "pdata lanes", &(old_pdata)->lanes, new_pdata->lanes);
	update_hex(dev, "pdata I2C slave addr", &(old_pdata)->i2c_slave_address,
		new_pdata->i2c_slave_address);
	update_int(dev, "pdata irq_pin", &(old_pdata)->irq_pin, new_pdata->irq_pin);
	update_str(dev, "pdata irq_pin_name", old_pdata->irq_pin_name, new_pdata->irq_pin_name);
	update_int(dev, "pdata reset_pin", &(old_pdata)->reset_pin, new_pdata->reset_pin);
	update_int(dev, "pdata detect_pin", &(old_pdata)->detect_pin, new_pdata->detect_pin);
	update_inta(dev, "pdata gpios", old_pdata->gpios, new_pdata->gpios, IPU_SPLATA_GPIO_NUM);
}

void update_serdes_subdev(struct device *dev,
			struct ipu_isys_subdev_info *new_sd,
			struct ipu_isys_subdev_info **old_sd)
{
	struct serdes_platform_data *old_pdata =
					(*old_sd)->i2c.board_info.platform_data;

	struct serdes_platform_data *new_pdata =
					new_sd->i2c.board_info.platform_data;

	int i;
	struct serdes_subdev_info *old_sdinfo, *new_sdinfo;
	struct serdes_module_pdata *old_mpdata, *new_mpdata;

	/* csi2 */
	update_int(dev, "CSI2 port", &(*old_sd)->csi2->port, new_sd->csi2->port);
	update_int(dev, "CSI2 nlanes", &(*old_sd)->csi2->nlanes, new_sd->csi2->nlanes);

	/* i2c */
	update_short(dev, "I2C board_info addr", &(*old_sd)->i2c.board_info.addr,
		new_sd->i2c.board_info.addr);
	update_str(dev, "I2C i2c_adapter_bdf", (*old_sd)->i2c.i2c_adapter_bdf,
		new_sd->i2c.i2c_adapter_bdf);

	update_int(dev, "I2C Pdata reset_gpio", &old_pdata->reset_gpio,
		new_pdata->reset_gpio);
	update_int(dev, "I2C Pdata FPD_gpio", &old_pdata->FPD_gpio, new_pdata->FPD_gpio);

	/* platform data */
	for (i = 0; i < SERDES_MAX_PORT; i++) {
		old_sdinfo = &old_pdata->subdev_info[i];
		old_mpdata = old_sdinfo->board_info.platform_data;

		new_sdinfo = &new_pdata->subdev_info[i];
		new_mpdata = new_sdinfo->board_info.platform_data;

		if (!strcmp(old_sdinfo->board_info.type, new_sdinfo->board_info.type) &&
			old_sdinfo->suffix == new_sdinfo->suffix) {
			update_short(dev, "SdInfo port", &old_sdinfo->rx_port,
				new_sdinfo->rx_port);
			update_short(dev, "SdInfo phy_i2c_addr", &old_sdinfo->phy_i2c_addr,
				new_sdinfo->phy_i2c_addr);
			update_short(dev, "SdInfo ser_alias", &old_sdinfo->ser_alias,
				new_sdinfo->ser_alias);
			update_short(dev, "SdInfo board_info.addr", &old_sdinfo->board_info.addr,
				new_sdinfo->board_info.addr);

			if (!strcmp(old_mpdata->module_name, new_mpdata->module_name)) {
				update_int(dev, "mPdata lanes", &old_mpdata->lanes,
					new_mpdata->lanes);
				update_int(dev, "mPdata fsin", &old_mpdata->fsin,
					new_mpdata->fsin);
				update_inta(dev, "mPdata gpio_powerup_seq",
						(int *)old_mpdata->gpio_powerup_seq,
						(int *)new_mpdata->gpio_powerup_seq,
						SERDES_MAX_GPIO_POWERUP_SEQ);
			}
		}
	}
}

int compare_subdev(struct device *dev,
			struct ipu_isys_subdev_info *new_subdev,
			struct ipu_isys_subdev_info *old_subdev,
			enum connection_type connect)
{
	/* check for ACPI HID in existing pdata */
	if (old_subdev->acpi_hid) {
		/* compare with HID for User Custom */
		if (!strcmp(old_subdev->acpi_hid, dev_name(dev))) {
			dev_info(dev, "Found matching sensor : %s", dev_name(dev));
			return 0;
		}
	}
	/* compare sensor type */
	if (!strcmp(old_subdev->i2c.board_info.type,
			new_subdev->i2c.board_info.type)) {

		if (connect == TYPE_DIRECT) {
			struct sensor_platform_data *old_pdata, *new_pdata;

			old_pdata = (struct sensor_platform_data *)
					old_subdev->i2c.board_info.platform_data;

			new_pdata = (struct sensor_platform_data *)
					new_subdev->i2c.board_info.platform_data;

			if (old_pdata->suffix == new_pdata->suffix) {
				dev_info(dev, "Found matching sensor : %s %c",
					old_subdev->i2c.board_info.type,
					old_pdata->suffix);
				return 0;
			}
		} else if (connect == TYPE_SERDES) {
			struct serdes_platform_data *old_pdata, *new_pdata;

			old_pdata = (struct serdes_platform_data *)
					old_subdev->i2c.board_info.platform_data;

			new_pdata = (struct serdes_platform_data *)
					new_subdev->i2c.board_info.platform_data;

			if (old_pdata->suffix == new_pdata->suffix) {
				dev_info(dev, "Found matching sensor : %s %c",
					old_subdev->i2c.board_info.type,
					old_pdata->suffix);
				return 0;
			}
		}
	}
	return -1;
}

void update_pdata(struct device *dev,
			struct ipu_isys_subdev_info *new_subdev,
			enum connection_type connect)
{
	struct ipu_isys_subdev_info *acpi_subdev;
	struct ipu_isys_subdev_pdata *ptr_built_in_pdata;
	bool found = false;

	acpi_subdev = new_subdev;

	/* update local ipu_isys_subdev_pdata */
	add_local_subdevs(acpi_subdev);

	ptr_built_in_pdata = get_built_in_pdata();

	/* if there is existing pdata, update the existing one */
	if (ptr_built_in_pdata) {
		struct ipu_isys_subdev_info **subdevs, *sd_info;

		for (subdevs = ptr_built_in_pdata->subdevs; *subdevs; subdevs++) {
			sd_info = *subdevs;

			if (!compare_subdev(dev, acpi_subdev, sd_info, connect)) {
				if (connect == TYPE_DIRECT) {
					dev_dbg(dev, "Old sensor subdev\n");
					print_subdev(sd_info);
					dev_dbg(dev, "New sensor subdev\n");
					print_subdev(acpi_subdev);
				} else if (connect == TYPE_SERDES) {
					dev_dbg(dev, "Old serdes subdev\n");
					print_serdes_subdev(sd_info);
					dev_dbg(dev, "New serdes subdev\n");
					print_serdes_subdev(acpi_subdev);
				}

				if (connect == TYPE_DIRECT)
					update_subdev(dev, acpi_subdev, &sd_info);
				else if (connect == TYPE_SERDES)
					update_serdes_subdev(dev, acpi_subdev, &sd_info);

				found = true;
				break;
			}
		}

		if (!found) {
			if (connect == TYPE_DIRECT) {
				struct sensor_platform_data *acpi_pdata;

				acpi_pdata = (struct sensor_platform_data *)
					acpi_subdev->i2c.board_info.platform_data;

				dev_err(dev, "Pdata does not contain %s %c\n",
					acpi_subdev->i2c.board_info.type,
					acpi_pdata->suffix);
			} else if (connect == TYPE_DIRECT) {
				struct serdes_platform_data *acpi_pdata;

				acpi_pdata = (struct serdes_platform_data *)
					acpi_subdev->i2c.board_info.platform_data;

				dev_err(dev, "Pdata does not contain %s %c\n",
					acpi_subdev->i2c.board_info.type,
					acpi_pdata->suffix);
			}
		}
	}

	if (!serdes_info.sensor_num)
		serdes_info.sensor_num = serdes_info.rx_port;
	else
		serdes_info.sensor_num += serdes_info.rx_port;
}

void set_ti960_gpio(struct control_logic_data *ctl_data, struct serdes_platform_data **pdata)
{
	int i;

	(*pdata)->reset_gpio = 0;
	(*pdata)->FPD_gpio = -1;

	if (ctl_data->completed && ctl_data->gpio_num > 0) {
		for (i = 0; i < ctl_data->gpio_num; i++) {
			/* check for RESET selection in BIOS */
			if (ctl_data->gpio[i].valid && ctl_data->gpio[i].func == GPIO_RESET)
				(*pdata)->reset_gpio = ctl_data->gpio[i].pin;

			/* check for POWER_EN selection in BIOS */
			if (ctl_data->gpio[i].valid && ctl_data->gpio[i].func == GPIO_POWER_EN)
				(*pdata)->FPD_gpio = ctl_data->gpio[i].pin;
		}
	}
}

void set_lt_gpio(struct control_logic_data *ctl_data, struct sensor_platform_data **pdata,
			bool is_dummy)
{
	int i;

	(*pdata)->irq_pin = -1;
	(*pdata)->reset_pin = -1;
	(*pdata)->detect_pin = -1;

	if (ctl_data->completed && ctl_data->gpio_num > 0 && !is_dummy) {
		for (i = 0; i < ctl_data->gpio_num; i++) {
			/* check for RESET selection in BIOS */
			if (ctl_data->gpio[i].valid && ctl_data->gpio[i].func == GPIO_RESET)
				(*pdata)->reset_pin = ctl_data->gpio[i].pin;

			/* check for READY_STAT selection in BIOS */
			if (ctl_data->gpio[i].valid && ctl_data->gpio[i].func == GPIO_READY_STAT) {
				(*pdata)->irq_pin = ctl_data->gpio[i].pin;
				(*pdata)->irq_pin_flags = IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT;
				strlcpy((*pdata)->irq_pin_name, "READY_STAT", sizeof("READY_STAT"));
			}

			/* check for HDMI_DETECT selection in BIOS */
			if (ctl_data->gpio[i].valid && ctl_data->gpio[i].func == GPIO_HDMI_DETECT)
				(*pdata)->detect_pin = ctl_data->gpio[i].pin;
		}
	}
}

void set_ar0234_gpio(struct sensor_platform_data **pdata)
{
	(*pdata)->gpios[0] = -1;
	(*pdata)->gpios[1] = 0;
	(*pdata)->gpios[2] = 0;
	(*pdata)->gpios[3] = 0;
}

int set_csi2(struct ipu_isys_subdev_info **sensor_sd,
		unsigned int lanes,
		unsigned int port)
{
	struct ipu_isys_csi2_config *csi2_config;

	csi2_config = kzalloc(sizeof(*csi2_config), GFP_KERNEL);
	if (!csi2_config)
		return -ENOMEM;

	csi2_config->nlanes = lanes;
	csi2_config->port = port;
	(*sensor_sd)->csi2 = csi2_config;

	return 0;
}

void set_i2c(struct ipu_isys_subdev_info **sensor_sd,
		struct device *dev,
		const char sensor_name[I2C_NAME_SIZE],
		unsigned int addr)
{
	(*sensor_sd)->i2c.board_info.addr = addr;
	strlcpy((*sensor_sd)->i2c.board_info.type, sensor_name, I2C_NAME_SIZE);
	strlcpy((*sensor_sd)->i2c.i2c_adapter_bdf, dev_name(dev->parent->parent->parent),
		sizeof((*sensor_sd)->i2c.i2c_adapter_bdf));
}

void set_serdes_sd_pdata(struct serdes_module_pdata **module_pdata, char sensor_name[I2C_NAME_SIZE],
			unsigned int lanes)
{
	/* general */
	(*module_pdata)->lanes = lanes;
	strlcpy((*module_pdata)->module_name, sensor_name, I2C_NAME_SIZE);

	/* TI960 and IMX390 specific */
	if (!strcmp(sensor_name, IMX390_NAME)) {
		(*module_pdata)->gpio_powerup_seq[0] = 0;
		(*module_pdata)->gpio_powerup_seq[1] = 0x9;
		(*module_pdata)->gpio_powerup_seq[2] = -1;
		(*module_pdata)->gpio_powerup_seq[3] = -1;
		(*module_pdata)->module_flags = TI960_FL_POWERUP | TI960_FL_INIT_SER_CLK;
		(*module_pdata)->fsin = 3;
	}
}

#define PORT_NR 8

int set_serdes_subdev(struct ipu_isys_subdev_info **serdes_sd,
		struct device *dev,
		struct serdes_platform_data **pdata,
		char sensor_name[I2C_NAME_SIZE],
		unsigned int lanes,
		unsigned int addr,
		unsigned int subdev_port)
{
	int i;
	struct serdes_module_pdata *module_pdata[PORT_NR];

	for (i = 0; i < subdev_port; i++) {
		module_pdata[i] = kzalloc(sizeof(*module_pdata[i]), GFP_KERNEL);
		if (!module_pdata[i])
			return -ENOMEM;

		set_serdes_sd_pdata(&module_pdata[i], sensor_name, lanes);

		/* board info */
		strlcpy(serdes_sdinfo[i].board_info.type, sensor_name, I2C_NAME_SIZE);
		serdes_sdinfo[i].board_info.addr = DUMMY_SERDES_BASE_ADDR + subdev_port + i;
		serdes_sdinfo[i].board_info.platform_data = module_pdata[i];

		/* serdes_subdev_info */
		serdes_sdinfo[i].rx_port = i;
		serdes_sdinfo[i].phy_i2c_addr = (addr << 1);
		serdes_sdinfo[i].ser_alias = DUMMY_SERDES_BASE_ADDR + i;
		serdes_sdinfo[i].suffix = i + 1 + SUFFIX_BASE + serdes_info.sensor_num;
	}

	(*pdata)->subdev_info = serdes_sdinfo;
	(*pdata)->subdev_num = ARRAY_SIZE(serdes_sdinfo);

	return 0;
}

int set_pdata(struct ipu_isys_subdev_info **sensor_sd,
		struct device *dev,
		char sensor_name[I2C_NAME_SIZE],
		struct control_logic_data *ctl_data,
		unsigned int port,
		unsigned int lanes,
		unsigned int addr,
		unsigned int rx_port,
		bool is_dummy,
		enum connection_type connect)
{
	if (connect == TYPE_DIRECT) {
		struct sensor_platform_data *pdata;

		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		/* use ascii */
		/* port for D457 start from 0 */
		if (!strcmp(sensor_name, D457_NAME) && port >= 0)
			pdata->suffix = port + SUFFIX_BASE + 1;
		else if (port > 0)
			pdata->suffix = port + SUFFIX_BASE;
		else
			dev_err(dev, "INVALID MIPI PORT");

		pdata->port = port;
		pdata->lanes = lanes;
		pdata->i2c_slave_address = addr;

		/* gpio */
		if (!strcmp(sensor_name, LT6911UXC_NAME))
			set_lt_gpio(ctl_data, &pdata, is_dummy);
		else if (!strcmp(sensor_name, AR0234_NAME))
			set_ar0234_gpio(&pdata);

		(*sensor_sd)->i2c.board_info.platform_data = pdata;
	} else if (connect == TYPE_SERDES) {
		struct serdes_platform_data *pdata;

		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		/* use ascii */
		if (port > 0)
			pdata->suffix = port + SUFFIX_BASE;
		else
			dev_err(dev, "INVALID MIPI PORT");

		if (!strcmp(sensor_name, IMX390_NAME))
			set_ti960_gpio(ctl_data, &pdata);

		set_serdes_subdev(sensor_sd, dev, &pdata, sensor_name, lanes, addr, rx_port);

		(*sensor_sd)->i2c.board_info.platform_data = pdata;
	}

	return 0;
}

void set_serdes_info(struct device *dev, const char *serdes_name, struct sensor_bios_data *cam_data)
{
	/* pprunit as num of sensor connected to deserializer*/
	serdes_info.rx_port = cam_data->pprunit;

	/* init sensor_num */
	if (!serdes_info.sensor_num)
		serdes_info.sensor_num = 0;

	/* save sensor addr */
	if (!serdes_info.sensor_addr)
		serdes_info.sensor_addr = cam_data->i2c[1].addr;

	/* TI960 specific */
	if (!strcmp(serdes_name, TI960_NAME))
		serdes_info.gpio_powerup_seq = TI960_MAX_GPIO_POWERUP_SEQ;
	else
		serdes_info.gpio_powerup_seq = 0;
}

int populate_dummy(struct device *dev,
			char sensor_name[I2C_NAME_SIZE],
			struct sensor_bios_data *cam_data,
			struct control_logic_data *ctl_data,
			enum connection_type connect)
{
	struct ipu_isys_subdev_info *dummy;
	unsigned short addr_dummy = 0x11;
	int ret;

	dummy = kzalloc(sizeof(*dummy), GFP_KERNEL);
	if (!dummy)
		return -ENOMEM;

	ret = set_csi2(&dummy, cam_data->lanes, cam_data->pprval);
	if (ret) {
		kfree(dummy);
		return ret;
	}

	set_i2c(&dummy, dev, sensor_name, addr_dummy);

	ret = set_pdata(&dummy, dev, sensor_name, ctl_data, cam_data->pprval,
		cam_data->lanes, addr_dummy, 0, true, connect);
	if (ret) {
		kfree(dummy);
		return ret;
	}

	update_pdata(dev, dummy, connect);

	return 0;
}

int populate_sensor_pdata(struct device *dev,
			struct ipu_isys_subdev_info **sensor_sd,
			char sensor_name[I2C_NAME_SIZE],
			struct sensor_bios_data *cam_data,
			struct control_logic_data *ctl_data,
			enum connection_type connect,
			const char *serdes_name)
{
	int ret;

	if (connect == TYPE_DIRECT) {
		/* sensor csi2 info */
		ret = set_csi2(sensor_sd, cam_data->lanes, cam_data->link);
		if (ret)
			return ret;

		/* sensor i2c info */
		if (cam_data->i2c_num == MIN_SENSOR_I2C) {
			set_i2c(sensor_sd, dev, sensor_name, cam_data->i2c[0].addr);
		} else {
			dev_err(dev, "IPU6 ACPI: Incorrect number of I2C device for sensor (%lld)",
				cam_data->i2c_num);
			return -1;
		}

	} else if (connect == TYPE_SERDES) {
		/* serdes csi2 info. pprval as deserializer lane */
		ret = set_csi2(sensor_sd, cam_data->pprval, cam_data->link);
		if (ret)
			return ret;

		/* Use DISCRETE Control Logic or No Control Logic for serdes */
		if (ctl_data->type != CL_DISCRETE && ctl_data->type != CL_EMPTY) {
			dev_err(dev, "IPU6 ACPI: Incorrect Control Logic Type for serdes (%d)",
				ctl_data->type);
			return -1;
		}

		/* serdes i2c info */
		if (cam_data->i2c_num == MIN_SERDES_I2C) {
			dev_info(dev, "IPU6 ACPI: correct i2c device for serdes");
			set_i2c(sensor_sd, dev, serdes_name, cam_data->i2c[0].addr);
		} else {
			dev_err(dev, "IPU6 ACPI: Incorrect number of I2C device for serdes (%lld)",
				cam_data->i2c_num);
			return -1;
		}

		/* local serdes info */
		set_serdes_info(dev, serdes_name, cam_data);
	}

	/* Use last I2C device */
	ret = set_pdata(sensor_sd, dev, sensor_name, ctl_data, cam_data->link,
		cam_data->lanes, cam_data->i2c[cam_data->i2c_num - 1].addr,
		cam_data->pprunit, false, connect);

	if (ret)
		return ret;

	update_pdata(dev, *sensor_sd, connect);

	/* Lontium specific */
	if (!strcmp(sensor_name, LT6911UXC_NAME)) {
		if (cam_data->pprval != cam_data->link) {
			ret = populate_dummy(dev, sensor_name, cam_data, ctl_data, connect);
			if (ret)
				return ret;
		}
	}

	return 0;
}

int get_sensor_pdata(struct i2c_client *client,
			struct ipu_camera_module_data *data,
			struct ipu_i2c_helper *helper,
			void *priv, size_t size,
			enum connection_type connect, const char *serdes_name)
{
	struct sensor_bios_data *cam_data;
	struct control_logic_data *ctl_data;
	struct ipu_isys_subdev_info *sensor_sd;
	int rval;

	cam_data = kzalloc(sizeof(*cam_data), GFP_KERNEL);
	if (!cam_data)
		return -ENOMEM;

	cam_data->dev = &client->dev;

	ctl_data = kzalloc(sizeof(*ctl_data), GFP_KERNEL);
	if (!ctl_data) {
		kfree(cam_data);
		return -ENOMEM;
	}

	sensor_sd = kzalloc(sizeof(*sensor_sd), GFP_KERNEL);
	if (!sensor_sd) {
		kfree(cam_data);
		kfree(ctl_data);
		return -ENOMEM;
	}

	/* camera info */
	rval = ipu_acpi_get_cam_data(&client->dev, cam_data);
	if (rval) {
		kfree(sensor_sd);
		kfree(cam_data);
		kfree(ctl_data);
		return rval;
	}

	/* control logic info */
	rval = ipu_acpi_get_dep_data(&client->dev, ctl_data);
	if (rval) {
		kfree(sensor_sd);
		kfree(cam_data);
		kfree(ctl_data);
		return rval;
	}

	/* populate pdata */
	rval = populate_sensor_pdata(&client->dev, &sensor_sd,
				client->name, cam_data, ctl_data, connect, serdes_name);
	if (rval) {
		kfree(sensor_sd);
		kfree(cam_data);
		kfree(ctl_data);
		return rval;
	}

	client->dev.platform_data = sensor_sd;

	kfree(cam_data);
	kfree(ctl_data);
	return rval;
}
EXPORT_SYMBOL(get_sensor_pdata);

MODULE_AUTHOR("Khai Wen, Ng <khai.wen.ng@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IPU6 ACPI support");
