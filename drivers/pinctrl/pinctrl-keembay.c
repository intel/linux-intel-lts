// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Keem Bay SoC pinctrl/GPIO driver
 *
 * Copyright (C) 2020 Intel Corporation
 * Authors: Vineetha G. Jaya Kumaran <vineetha.g.jaya.kumaran@intel.com>
 *	    Muhammad Husaini Zulkifli <muhammad.husaini.zulkifli@intel.com>
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "core.h"
#include "pinconf.h"
#include "pinctrl-utils.h"
#include "pinmux.h"

/* GPIO data registers */
#define GPIO_DATA_OUT		0x000
#define GPIO_DATA_IN		0x020
#define GPIO_DATA_IN_RAW	0x040
#define GPIO_DATA_HIGH		0x060
#define GPIO_DATA_LOW		0x080

/* GPIO interrupt and mode registers */
#define GPIO_INT_CFG		0x000
#define GPIO_MODE		0x070

#define GPIO_INT_CFG_SHIFT_0	0
#define GPIO_INT_CFG_SHIFT_1	8
#define GPIO_INT_CFG_SHIFT_2	16
#define GPIO_INT_CFG_SHIFT_3	24
#define MAX_PINS_PER_IRQ	4
#define IRQ_ENABLE_BIT_MASK	GENMASK(6, 0)
#define GPIO_INT_PIN_0_MASK	GENMASK(7, 0)
#define GPIO_INT_PIN_1_MASK	GENMASK(15, 8)
#define GPIO_INT_PIN_2_MASK	GENMASK(23, 16)
#define GPIO_INT_PIN_3_MASK	GENMASK(31, 24)

/* GPIO mode register bit fields */
#define GPIO_MODE_DIR_OVR	BIT(15)
#define GPIO_MODE_PULLUP_MASK	GENMASK(13, 12)
#define GPIO_MODE_REN		BIT(11)
#define GPIO_MODE_SCHMITT_EN	BIT(10)
#define GPIO_MODE_SLEW_RATE	BIT(9)
#define GPIO_MODE_DRIVE_MASK	GENMASK(8, 7)
#define GPIO_MODE_EN_INV	BIT(5)
#define GPIO_MODE_DATA_INV	BIT(4)
#define GPIO_MODE_DIR		BIT_MASK(3)
#define GPIO_MODE_DIR_INPUT	(1 << 3)
#define GPIO_MODE_DIR_OUTPUT	(0 << 3)
#define GPIO_MODE_SELECT_MASK	GENMASK(2, 0)

/**
 * struct keembay_mux_desc - Mux properties of each GPIO pin
 * @mode: Pin mode when operating in this function
 * @name: Pin function name
 */
struct keembay_mux_desc {
	u8 mode;
	const char *name;
};

#define KEEMBAY_PIN_DESC(pin_number, pin_name, ...) {	\
	.number = pin_number,				\
	.name =	pin_name,				\
	.drv_data = &(struct keembay_mux_desc[]) {	\
			__VA_ARGS__, { } },		\
}							\

#define KEEMBAY_MUX(pin_mode, pin_function) {		\
	.mode = pin_mode,				\
	.name = pin_function,				\
}							\

#define KEEMBAY_NUM_IRQ_LINES	8

/**
 * struct keembay_pinctrl - Intel Keem Bay pinctrl structure
 * @pctrl: Pointer to the pin controller device
 * @base0: First register base address
 * @base1: Second register base address
 * @dev: Pointer to the device structure
 * @chip: GPIO chip used by this pin controller
 * @soc: Pin control configuration data based on SoC
 * @lock: Spinlock to be used for register access
 * @ngroups: Number of pin groups available
 * @nfuncs: Number of pin functions available
 */
struct keembay_pinctrl {
	struct pinctrl_dev *pctrl;
	void __iomem *base0;
	void __iomem *base1;
	struct device *dev;
	struct gpio_chip chip;
	const struct keembay_pin_soc *soc;
	raw_spinlock_t lock;
	unsigned int ngroups;
	unsigned int nfuncs;
};

/**
 * struct keembay_pin_soc - Pin control configuration data based on SoC
 * @pinctrl_pin_desc: Pin description structure
 * @npins: Number of GPIO pins available
 */
struct keembay_pin_soc {
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
};

/**
 * struct gpio_irq_source - Configuration of each of the 8 GPIO IRQ sources
 * @active: State of IRQ line. Active if it is being used by a GPIO pin
 * @source: Interrupt source number (0 - 7)
 * @line: Actual IRQ line number
 * @pins: Array of GPIO pins using this IRQ line
 * @trigger: Interrupt trigger type for this line
 * @gpios_conn: Number of pins currently using this IRQ line (0 - 4)
 */
struct gpio_irq_source {
	bool active;
	unsigned int source;
	unsigned int line;
	unsigned int pins[4];
	unsigned int trigger;
	unsigned int gpios_conn;
};

static const struct pinctrl_pin_desc keembay_pins[] = {
	KEEMBAY_PIN_DESC(0, "GPIO0",
		KEEMBAY_MUX(0x0, "I2S0_M0"),
		KEEMBAY_MUX(0x1, "SD0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(1, "GPIO1",
		KEEMBAY_MUX(0x0, "I2S0_M0"),
		KEEMBAY_MUX(0x1, "SD0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(2, "GPIO2",
		KEEMBAY_MUX(0x0, "I2S0_M0"),
		KEEMBAY_MUX(0x1, "I2S0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(3, "GPIO3",
		KEEMBAY_MUX(0x0, "I2S0_M0"),
		KEEMBAY_MUX(0x1, "I2S0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(4, "GPIO4",
		KEEMBAY_MUX(0x0, "I2S0_M0"),
		KEEMBAY_MUX(0x1, "I2S0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C2_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(5, "GPIO5",
		KEEMBAY_MUX(0x0, "I2S0_M0"),
		KEEMBAY_MUX(0x1, "I2S0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C2_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(6, "GPIO6",
		KEEMBAY_MUX(0x0, "I2S1_M0"),
		KEEMBAY_MUX(0x1, "SD0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C3_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(7, "GPIO7",
		KEEMBAY_MUX(0x0, "I2S1_M0"),
		KEEMBAY_MUX(0x1, "SD0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "I2C3_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(8, "GPIO8",
		KEEMBAY_MUX(0x0, "I2S1_M0"),
		KEEMBAY_MUX(0x1, "I2S1_M1"),
		KEEMBAY_MUX(0x2, "SLVDS0_M2"),
		KEEMBAY_MUX(0x3, "UART0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(9, "GPIO9",
		KEEMBAY_MUX(0x0, "I2S1_M0"),
		KEEMBAY_MUX(0x1, "I2S1_M1"),
		KEEMBAY_MUX(0x2, "PWM_M2"),
		KEEMBAY_MUX(0x3, "UART0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(10, "GPIO10",
		KEEMBAY_MUX(0x0, "I2S2_M0"),
		KEEMBAY_MUX(0x1, "SD0_M1"),
		KEEMBAY_MUX(0x2, "PWM_M2"),
		KEEMBAY_MUX(0x3, "UART0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(11, "GPIO11",
		KEEMBAY_MUX(0x0, "I2S2_M0"),
		KEEMBAY_MUX(0x1, "SD0_M1"),
		KEEMBAY_MUX(0x2, "PWM_M2"),
		KEEMBAY_MUX(0x3, "UART0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(12, "GPIO12",
		KEEMBAY_MUX(0x0, "I2S2_M0"),
		KEEMBAY_MUX(0x1, "I2S2_M1"),
		KEEMBAY_MUX(0x2, "PWM_M2"),
		KEEMBAY_MUX(0x3, "SPI0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(13, "GPIO13",
		KEEMBAY_MUX(0x0, "I2S2_M0"),
		KEEMBAY_MUX(0x1, "I2S2_M1"),
		KEEMBAY_MUX(0x2, "PWM_M2"),
		KEEMBAY_MUX(0x3, "SPI0_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(14, "GPIO14",
		KEEMBAY_MUX(0x0, "UART0_M0"),
		KEEMBAY_MUX(0x1, "I2S3_M1"),
		KEEMBAY_MUX(0x2, "PWM_M2"),
		KEEMBAY_MUX(0x3, "SD1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "ETH_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(15, "GPIO15",
		KEEMBAY_MUX(0x0, "UART0_M0"),
		KEEMBAY_MUX(0x1, "I2S3_M1"),
		KEEMBAY_MUX(0x2, "UART0_M2"),
		KEEMBAY_MUX(0x3, "SD1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "SPI1_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(16, "GPIO16",
		KEEMBAY_MUX(0x0, "UART0_M0"),
		KEEMBAY_MUX(0x1, "I2S3_M1"),
		KEEMBAY_MUX(0x2, "UART0_M2"),
		KEEMBAY_MUX(0x3, "SD1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "SPI1_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(17, "GPIO17",
		KEEMBAY_MUX(0x0, "UART0_M0"),
		KEEMBAY_MUX(0x1, "I2S3_M1"),
		KEEMBAY_MUX(0x2, "I2S3_M2"),
		KEEMBAY_MUX(0x3, "SD1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "SPI1_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(18, "GPIO18",
		KEEMBAY_MUX(0x0, "UART1_M0"),
		KEEMBAY_MUX(0x1, "SPI0_M1"),
		KEEMBAY_MUX(0x2, "I2S3_M2"),
		KEEMBAY_MUX(0x3, "SD1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "SPI1_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(19, "GPIO19",
		KEEMBAY_MUX(0x0, "UART1_M0"),
		KEEMBAY_MUX(0x1, "LCD_M1"),
		KEEMBAY_MUX(0x2, "DEBUG_M2"),
		KEEMBAY_MUX(0x3, "SD1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "SPI1_M5"),
		KEEMBAY_MUX(0x6, "LCD_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(20, "GPIO20",
		KEEMBAY_MUX(0x0, "UART1_M0"),
		KEEMBAY_MUX(0x1, "LCD_M1"),
		KEEMBAY_MUX(0x2, "DEBUG_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "SPI1_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(21, "GPIO21",
		KEEMBAY_MUX(0x0, "UART1_M0"),
		KEEMBAY_MUX(0x1, "LCD_M1"),
		KEEMBAY_MUX(0x2, "DEBUG_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(22, "GPIO22",
		KEEMBAY_MUX(0x0, "I2C0_M0"),
		KEEMBAY_MUX(0x1, "UART2_M1"),
		KEEMBAY_MUX(0x2, "DEBUG_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(23, "GPIO23",
		KEEMBAY_MUX(0x0, "I2C0_M0"),
		KEEMBAY_MUX(0x1, "UART2_M1"),
		KEEMBAY_MUX(0x2, "DEBUG_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C1_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(24, "GPIO24",
		KEEMBAY_MUX(0x0, "I2C1_M0"),
		KEEMBAY_MUX(0x1, "UART2_M1"),
		KEEMBAY_MUX(0x2, "DEBUG_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C1_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(25, "GPIO25",
		KEEMBAY_MUX(0x0, "I2C1_M0"),
		KEEMBAY_MUX(0x1, "UART2_M1"),
		KEEMBAY_MUX(0x2, "SPI0_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C2_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(26, "GPIO26",
		KEEMBAY_MUX(0x0, "SPI0_M0"),
		KEEMBAY_MUX(0x1, "I2C2_M1"),
		KEEMBAY_MUX(0x2, "UART0_M2"),
		KEEMBAY_MUX(0x3, "DSU_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C2_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(27, "GPIO27",
		KEEMBAY_MUX(0x0, "SPI0_M0"),
		KEEMBAY_MUX(0x1, "I2C2_M1"),
		KEEMBAY_MUX(0x2, "UART0_M2"),
		KEEMBAY_MUX(0x3, "DSU_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(28, "GPIO28",
		KEEMBAY_MUX(0x0, "SPI0_M0"),
		KEEMBAY_MUX(0x1, "I2C3_M1"),
		KEEMBAY_MUX(0x2, "UART0_M2"),
		KEEMBAY_MUX(0x3, "PWM_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C1_M5"),
		KEEMBAY_MUX(0x6, "SLVDS0_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(29, "GPIO29",
		KEEMBAY_MUX(0x0, "SPI0_M0"),
		KEEMBAY_MUX(0x1, "I2C3_M1"),
		KEEMBAY_MUX(0x2, "UART0_M2"),
		KEEMBAY_MUX(0x3, "PWM_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I3C2_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(30, "GPIO30",
		KEEMBAY_MUX(0x0, "SPI0_M0"),
		KEEMBAY_MUX(0x1, "I2S0_M1"),
		KEEMBAY_MUX(0x2, "I2C4_M2"),
		KEEMBAY_MUX(0x3, "PWM_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(31, "GPIO31",
		KEEMBAY_MUX(0x0, "SPI0_M0"),
		KEEMBAY_MUX(0x1, "I2S0_M1"),
		KEEMBAY_MUX(0x2, "I2C4_M2"),
		KEEMBAY_MUX(0x3, "PWM_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "UART1_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(32, "GPIO32",
		KEEMBAY_MUX(0x0, "SD0_M0"),
		KEEMBAY_MUX(0x1, "SPI0_M1"),
		KEEMBAY_MUX(0x2, "UART1_M2"),
		KEEMBAY_MUX(0x3, "PWM_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "PCIE_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(33, "GPIO33",
		KEEMBAY_MUX(0x0, "SD0_M0"),
		KEEMBAY_MUX(0x1, "SPI0_M1"),
		KEEMBAY_MUX(0x2, "UART1_M2"),
		KEEMBAY_MUX(0x3, "PWM_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "PCIE_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(34, "GPIO34",
		KEEMBAY_MUX(0x0, "SD0_M0"),
		KEEMBAY_MUX(0x1, "SPI0_M1"),
		KEEMBAY_MUX(0x2, "I2C0_M2"),
		KEEMBAY_MUX(0x3, "UART1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I2S0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(35, "GPIO35",
		KEEMBAY_MUX(0x0, "SD0_M0"),
		KEEMBAY_MUX(0x1, "PCIE_M1"),
		KEEMBAY_MUX(0x2, "I2C0_M2"),
		KEEMBAY_MUX(0x3, "UART1_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I2S0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(36, "GPIO36",
		KEEMBAY_MUX(0x0, "SD0_M0"),
		KEEMBAY_MUX(0x1, "SPI3_M1"),
		KEEMBAY_MUX(0x2, "I2C1_M2"),
		KEEMBAY_MUX(0x3, "DEBUG_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I2S0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(37, "GPIO37",
		KEEMBAY_MUX(0x0, "SD0_M0"),
		KEEMBAY_MUX(0x1, "SPI3_M1"),
		KEEMBAY_MUX(0x2, "I2C1_M2"),
		KEEMBAY_MUX(0x3, "DEBUG_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "I2S0_M5"),
		KEEMBAY_MUX(0x6, "SLVDS1_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(38, "GPIO38",
		KEEMBAY_MUX(0x0, "I3C1_M0"),
		KEEMBAY_MUX(0x1, "SPI3_M1"),
		KEEMBAY_MUX(0x2, "UART3_M2"),
		KEEMBAY_MUX(0x3, "DEBUG_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2C2_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(39, "GPIO39",
		KEEMBAY_MUX(0x0, "I3C1_M0"),
		KEEMBAY_MUX(0x1, "SPI3_M1"),
		KEEMBAY_MUX(0x2, "UART3_M2"),
		KEEMBAY_MUX(0x3, "DEBUG_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2C2_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(40, "GPIO40",
		KEEMBAY_MUX(0x0, "I2S2_M0"),
		KEEMBAY_MUX(0x1, "SPI3_M1"),
		KEEMBAY_MUX(0x2, "UART3_M2"),
		KEEMBAY_MUX(0x3, "DEBUG_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2C3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(41, "GPIO41",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI3_M1"),
		KEEMBAY_MUX(0x2, "SPI3_M2"),
		KEEMBAY_MUX(0x3, "DEBUG_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2C3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(42, "GPIO42",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SD1_M1"),
		KEEMBAY_MUX(0x2, "SPI3_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "CAM_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2C4_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(43, "GPIO43",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SD1_M1"),
		KEEMBAY_MUX(0x2, "SPI3_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "I2S0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2C4_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(44, "GPIO44",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SD1_M1"),
		KEEMBAY_MUX(0x2, "SPI0_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "I2S0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(45, "GPIO45",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SD1_M1"),
		KEEMBAY_MUX(0x2, "SPI0_M2"),
		KEEMBAY_MUX(0x3, "CPR_M3"),
		KEEMBAY_MUX(0x4, "I2S0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(46, "GPIO46",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SD1_M1"),
		KEEMBAY_MUX(0x2, "SPI0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(47, "GPIO47",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SD1_M1"),
		KEEMBAY_MUX(0x2, "SPI0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(48, "GPIO48",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "UART2_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(49, "GPIO49",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "UART2_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(50, "GPIO50",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "UART2_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(51, "GPIO51",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "UART2_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(52, "GPIO52",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "SD0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(53, "GPIO53",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "SD0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(54, "GPIO54",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "SD0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(55, "GPIO55",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "SD1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(56, "GPIO56",
		KEEMBAY_MUX(0x0, "ETH_M0"),
		KEEMBAY_MUX(0x1, "SPI2_M1"),
		KEEMBAY_MUX(0x2, "SD1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I2S2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(57, "GPIO57",
		KEEMBAY_MUX(0x0, "SPI1_M0"),
		KEEMBAY_MUX(0x1, "I2S1_M1"),
		KEEMBAY_MUX(0x2, "SD1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(58, "GPIO58",
		KEEMBAY_MUX(0x0, "SPI1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SD0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(59, "GPIO59",
		KEEMBAY_MUX(0x0, "SPI1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SD0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(60, "GPIO60",
		KEEMBAY_MUX(0x0, "SPI1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "I3C1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(61, "GPIO61",
		KEEMBAY_MUX(0x0, "SPI1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SD0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(62, "GPIO62",
		KEEMBAY_MUX(0x0, "SPI1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SD1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(63, "GPIO63",
		KEEMBAY_MUX(0x0, "I2S1_M0"),
		KEEMBAY_MUX(0x1, "SPI1_M1"),
		KEEMBAY_MUX(0x2, "SD1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(64, "GPIO64",
		KEEMBAY_MUX(0x0, "I2S2_M0"),
		KEEMBAY_MUX(0x1, "SPI1_M1"),
		KEEMBAY_MUX(0x2, "ETH_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "UART1_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(65, "GPIO65",
		KEEMBAY_MUX(0x0, "I3C0_M0"),
		KEEMBAY_MUX(0x1, "SPI1_M1"),
		KEEMBAY_MUX(0x2, "SD1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SPI0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(66, "GPIO66",
		KEEMBAY_MUX(0x0, "I3C0_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "I2C0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SPI0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "CAM_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(67, "GPIO67",
		KEEMBAY_MUX(0x0, "I3C1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "I2C0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SPI0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2S3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(68, "GPIO68",
		KEEMBAY_MUX(0x0, "I3C1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "I2C1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SPI0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2S3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(69, "GPIO69",
		KEEMBAY_MUX(0x0, "I3C2_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "I2C1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SPI0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2S3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(70, "GPIO70",
		KEEMBAY_MUX(0x0, "I3C2_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SPI0_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SD0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2S3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(71, "GPIO71",
		KEEMBAY_MUX(0x0, "I3C0_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SD0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "I2S3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(72, "GPIO72",
		KEEMBAY_MUX(0x0, "I3C1_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SD0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART2_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(73, "GPIO73",
		KEEMBAY_MUX(0x0, "I3C2_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SD0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART2_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(74, "GPIO74",
		KEEMBAY_MUX(0x0, "I3C0_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SD0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART2_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(75, "GPIO75",
		KEEMBAY_MUX(0x0, "I3C0_M0"),
		KEEMBAY_MUX(0x1, "ETH_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "SD0_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART2_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(76, "GPIO76",
		KEEMBAY_MUX(0x0, "I2C2_M0"),
		KEEMBAY_MUX(0x1, "I3C0_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "ETH_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(77, "GPIO77",
		KEEMBAY_MUX(0x0, "PCIE_M0"),
		KEEMBAY_MUX(0x1, "I3C1_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I3C2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(78, "GPIO78",
		KEEMBAY_MUX(0x0, "PCIE_M0"),
		KEEMBAY_MUX(0x1, "I3C2_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I3C2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
	KEEMBAY_PIN_DESC(79, "GPIO79",
		KEEMBAY_MUX(0x0, "PCIE_M0"),
		KEEMBAY_MUX(0x1, "I2C2_M1"),
		KEEMBAY_MUX(0x2, "SLVDS1_M2"),
		KEEMBAY_MUX(0x3, "TPIU_M3"),
		KEEMBAY_MUX(0x4, "I3C2_M4"),
		KEEMBAY_MUX(0x5, "LCD_M5"),
		KEEMBAY_MUX(0x6, "UART3_M6"),
		KEEMBAY_MUX(0x7, "GPIO_M7")),
};

/**
 * Read the GPIO_DATA register value for one pin
 * @address: First register base address
 * @offset: GPIO pin number
 */
static inline u32 read_gpio_pin_data(void __iomem *address, unsigned int offset)
{
	u32 reg;

	reg = readl(address + (offset / 32) * 4);

	return !!(reg & BIT(offset % 32));
}

/**
 * Read the GPIO_DATA register values for 32 pins
 * @address: First register base address
 * @offset: GPIO pin number
 */
static inline u32 read_gpio_data(void __iomem *address, unsigned int offset)
{
	return readl(address + (offset / 32) * 4);
}

static inline void write_gpio_data(u32 data, void __iomem *address,
				   unsigned int offset)
{
	writel(data, address + (offset / 32) * 4);
}

/**
 * Read the GPIO_MODE or GPIO_INT_CFG register
 * @address: Second register base address
 * @offset: GPIO pin number
 */
static inline u32 read_reg(void __iomem *address, unsigned int offset)
{
	return readl(address + offset * 4);
}

static inline void write_reg(u32 data, void __iomem *address,
			     unsigned int offset)
{
	writel(data, address + offset * 4);
}

static void set_inversion_gpio_output(struct keembay_pinctrl *kpc,
				      unsigned int pin)
{
	u32 reg;

	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	reg |= GPIO_MODE_EN_INV | GPIO_MODE_DATA_INV;
	write_reg(reg, kpc->base1 + GPIO_MODE, pin);
}

static void clear_inversion_gpio_output(struct keembay_pinctrl *kpc,
					unsigned int pin)
{
	u32 reg;

	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	if ((reg & (GPIO_MODE_EN_INV | GPIO_MODE_DATA_INV))) {
		reg &= ~(GPIO_MODE_EN_INV | GPIO_MODE_DATA_INV);
		write_reg(reg, kpc->base1 + GPIO_MODE, pin);
	}
}

static int keembay_set_mux(struct pinctrl_dev *pctldev,
			   unsigned int func_select,
			   unsigned int group_select)
{
	struct keembay_pinctrl *kpc = pinctrl_dev_get_drvdata(pctldev);
	struct function_desc *function;
	struct group_desc *group;
	unsigned long flags;
	u8 pin_mode;
	int pin;
	u32 reg;

	group = pinctrl_generic_get_group(pctldev, group_select);
	if (!group)
		return -EINVAL;

	function = pinmux_generic_get_function(pctldev, func_select);
	if (!function)
		return -EINVAL;

	/* Change modes for pins in the selected group */
	pin = *group->pins;
	pin_mode = *(u8 *)(function->data);

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	reg &= ~GPIO_MODE_SELECT_MASK;
	reg |= FIELD_PREP(GPIO_MODE_SELECT_MASK, pin_mode);
	write_reg(reg, kpc->base1 + GPIO_MODE, pin);
	raw_spin_unlock_irqrestore(&kpc->lock, flags);

	return 0;
}

static int keembay_request_gpio(struct pinctrl_dev *pctldev,
				struct pinctrl_gpio_range *range,
				unsigned int pin)
{
	struct keembay_pinctrl *kpc = pinctrl_dev_get_drvdata(pctldev);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	reg &= GPIO_MODE_SELECT_MASK;
	raw_spin_unlock_irqrestore(&kpc->lock, flags);

	/*
	 * Allow GPIO request for a pin if it is in default GPIO mode (7).
	 * Request fails if the pin has been muxed into any other modes (0-6).
	 */

	if (reg != (reg | GPIO_MODE_SELECT_MASK))
		return -EBUSY;

	return 0;
}

static int keembay_pinconf_get_pull(struct keembay_pinctrl *kpc,
				    unsigned int pin,
				    unsigned int *pull)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	*pull = FIELD_GET(GPIO_MODE_PULLUP_MASK, reg);

	return 0;
}

static int keembay_pinconf_set_pull(struct keembay_pinctrl *kpc,
				    unsigned int pin,
				    unsigned int pull)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	reg &= ~GPIO_MODE_PULLUP_MASK;
	reg |= FIELD_PREP(GPIO_MODE_PULLUP_MASK, pull);
	write_reg(reg, kpc->base1 + GPIO_MODE, pin);

	return 0;
}

static int keembay_pinconf_get_drive(struct keembay_pinctrl *kpc,
				     unsigned int pin, u32 *drive)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	reg = FIELD_GET(GPIO_MODE_DRIVE_MASK, reg);

	/*
	 * Drive strengths:
	 *   0:	2mA
	 *   1: 4mA
	 *   2: 8mA
	 *   3: 12mA
	 */
	switch (reg) {
	case 0:
		*drive = 2;
		break;

	case 1:
		*drive = 4;
		break;

	case 2:
		*drive = 8;
		break;

	case 3:
		*drive = 12;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int keembay_pinconf_set_drive(struct keembay_pinctrl *kpc,
				     unsigned int pin,
				     unsigned int drive)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	unsigned int value;

	/*
	 * Supported drive strengths:
	 * 2mA, 4mA, 8mA, 12mA
	 */

	switch (drive) {
	case 2:
		value = 0;
		break;

	case 4:
		value = 1;
		break;

	case 8:
		value = 2;
		break;

	case 12:
		value = 3;
		break;

	default:
		return -EINVAL;
	}

	reg &= ~GPIO_MODE_DRIVE_MASK;
	reg |= FIELD_PREP(GPIO_MODE_DRIVE_MASK, value);
	write_reg(reg, kpc->base1 + GPIO_MODE, pin);

	return 0;
}

static int keembay_pinconf_get_slew_rate(struct keembay_pinctrl *kpc,
					 unsigned int pin, u32 *arg)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	reg &= GPIO_MODE_SLEW_RATE;
	if (reg)
		*arg = 1;
	else
		*arg = 0;

	return 0;
}

static int keembay_pinconf_set_slew_rate(struct keembay_pinctrl *kpc,
					 unsigned int pin, u32 arg)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	reg &= ~GPIO_MODE_SLEW_RATE;

	if (arg)
		reg |= GPIO_MODE_SLEW_RATE;

	write_reg(reg, kpc->base1 + GPIO_MODE, pin);

	return 0;
}

static int keembay_pinconf_get_schmitt(struct keembay_pinctrl *kpc,
				       unsigned int pin, u32 *arg)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	reg &= GPIO_MODE_SCHMITT_EN;
	if (reg)
		*arg = 1;
	else
		*arg = 0;

	return 0;
}

static int keembay_pinconf_set_schmitt(struct keembay_pinctrl *kpc,
				       unsigned int pin, u32 arg)
{
	u32 reg = read_reg(kpc->base1 + GPIO_MODE, pin);

	reg &= ~GPIO_MODE_SCHMITT_EN;

	if (arg)
		reg |= GPIO_MODE_SCHMITT_EN;

	write_reg(reg, kpc->base1 + GPIO_MODE, pin);

	return 0;
}

static int keembay_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *config)
{
	struct keembay_pinctrl *kpc = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int pull;
	u32 arg;
	int ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		keembay_pinconf_get_pull(kpc, pin, &pull);
		if (pull != 0)
			return -EINVAL;
		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		keembay_pinconf_get_pull(kpc, pin, &pull);
		if (pull != 1)
			return -EINVAL;
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		keembay_pinconf_get_pull(kpc, pin, &pull);
		if (pull != 2)
			return -EINVAL;
		break;

	case PIN_CONFIG_BIAS_BUS_HOLD:
		keembay_pinconf_get_pull(kpc, pin, &pull);
		if (pull != 3)
			return -EINVAL;
		break;

	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		keembay_pinconf_get_schmitt(kpc, pin, &arg);
		if (!arg)
			return -EINVAL;
		break;

	case PIN_CONFIG_SLEW_RATE:
		keembay_pinconf_get_slew_rate(kpc, pin, &arg);
		*config = pinconf_to_config_packed(param, arg);
		break;

	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = keembay_pinconf_get_drive(kpc, pin, &arg);
		if (ret < 0)
			return -EINVAL;
		*config = pinconf_to_config_packed(param, arg);
		break;

	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int keembay_pinconf_set(struct pinctrl_dev *pctldev,
			       unsigned int pin,
			       unsigned long *configs,
			       unsigned int num_configs)
{
	struct keembay_pinctrl *kpc = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	unsigned int pinconf;
	int ret = 0;
	u32 arg;

	for (pinconf = 0; pinconf < num_configs; pinconf++) {

		param = pinconf_to_config_param(configs[pinconf]);
		arg = pinconf_to_config_argument(configs[pinconf]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			ret = keembay_pinconf_set_pull(kpc, pin, 0);
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			ret = keembay_pinconf_set_pull(kpc, pin, 1);
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = keembay_pinconf_set_pull(kpc, pin, 2);
			break;

		case PIN_CONFIG_BIAS_BUS_HOLD:
			ret = keembay_pinconf_set_pull(kpc, pin, 3);
			break;

		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			ret = keembay_pinconf_set_schmitt(kpc, pin, arg);
			break;

		case PIN_CONFIG_SLEW_RATE:
			ret = keembay_pinconf_set_slew_rate(kpc, pin, arg);
			break;

		case PIN_CONFIG_DRIVE_STRENGTH:
			ret = keembay_pinconf_set_drive(kpc, pin, arg);
			break;

		default:
			return -ENOTSUPP;
		}
	}

	return ret;
}

static const struct pinmux_ops keembay_pmxops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name   = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.gpio_request_enable = keembay_request_gpio,
	.set_mux             = keembay_set_mux,
};

static const struct pinctrl_ops keembay_pctlops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name   = pinctrl_generic_get_group_name,
	.get_group_pins   = pinctrl_generic_get_group_pins,
	.dt_node_to_map   = pinconf_generic_dt_node_to_map_all,
	.dt_free_map      = pinconf_generic_dt_free_map,
};

static const struct pinconf_ops keembay_confops = {
	.is_generic	= true,
	.pin_config_get	= keembay_pinconf_get,
	.pin_config_set	= keembay_pinconf_set,
};

static struct pinctrl_desc keembay_pinctrl_desc = {
	.name    = "keembay-pinmux",
	.pctlops = &keembay_pctlops,
	.pmxops  = &keembay_pmxops,
	.confops = &keembay_confops,
	.owner   = THIS_MODULE,
};

static u32 int_cfg_shift[4] = {
	GPIO_INT_CFG_SHIFT_0,
	GPIO_INT_CFG_SHIFT_1,
	GPIO_INT_CFG_SHIFT_2,
	GPIO_INT_CFG_SHIFT_3,
};

static u32 int_masks[4] = {
	GPIO_INT_PIN_0_MASK,
	GPIO_INT_PIN_1_MASK,
	GPIO_INT_PIN_2_MASK,
	GPIO_INT_PIN_3_MASK,
};

static struct gpio_irq_source keembay_irq[KEEMBAY_NUM_IRQ_LINES];
static int max_gpios_level_type;
static int max_gpios_edge_type;

static int keembay_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_reg(kpc->base1 + GPIO_MODE, pin) & GPIO_MODE_DIR;
	raw_spin_unlock_irqrestore(&kpc->lock, flags);

	if (reg)
		return read_gpio_pin_data(kpc->base0 + GPIO_DATA_IN, pin);
	else
		return read_gpio_pin_data(kpc->base0 + GPIO_DATA_OUT, pin);
}

static void keembay_gpio_set(struct gpio_chip *chip, unsigned int pin,
			     int value)
{
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_gpio_data(kpc->base0 + GPIO_DATA_OUT, pin);

	/*
	 * Write 1 to GPIO_DATA_HIGH if value = 1.
	 * Write 1 to GPIO_DATA_LOW if value = 0.
	 * GPIO_DATA_OUT will reflect the actual output of each pin.
	 */

	if (value)
		write_gpio_data(reg | BIT(pin % 32),
				kpc->base0 + GPIO_DATA_HIGH, pin);
	else
		write_gpio_data(~reg | BIT(pin % 32),
				kpc->base0 + GPIO_DATA_LOW, pin);

	raw_spin_unlock_irqrestore(&kpc->lock, flags);
}

static int keembay_gpio_get_direction(struct gpio_chip *chip,
				      unsigned int pin)
{
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	raw_spin_unlock_irqrestore(&kpc->lock, flags);

	return !!(reg & GPIO_MODE_DIR);
}

static int keembay_gpio_set_direction_input(struct gpio_chip *chip,
					    unsigned int pin)
{
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	reg |= GPIO_MODE_DIR_INPUT;
	write_reg(reg, kpc->base1 + GPIO_MODE, pin);
	raw_spin_unlock_irqrestore(&kpc->lock, flags);

	return 0;
}

static int keembay_gpio_set_direction_output(struct gpio_chip *chip,
					     unsigned int pin, int value)
{
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned long flags;
	u32 reg;

	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg = read_reg(kpc->base1 + GPIO_MODE, pin);
	reg &= GPIO_MODE_DIR_OUTPUT;
	write_reg(reg, kpc->base1 + GPIO_MODE, pin);
	raw_spin_unlock_irqrestore(&kpc->lock, flags);
	keembay_gpio_set(chip, pin, value);

	return 0;
}

static unsigned int keembay_gpio_irq_get_pin(struct gpio_chip *chip,
					     unsigned int slot, u32 reg)
{
	unsigned int pin = 0;

	switch (slot) {
	case 7:
		pin = FIELD_GET(GPIO_INT_PIN_0_MASK, reg);
		pin &= IRQ_ENABLE_BIT_MASK;
		break;
	case 15:
		pin = FIELD_GET(GPIO_INT_PIN_1_MASK, reg);
		pin &= IRQ_ENABLE_BIT_MASK;
		break;
	case 23:
		pin = FIELD_GET(GPIO_INT_PIN_2_MASK, reg);
		pin &= IRQ_ENABLE_BIT_MASK;
		break;
	case 31:
		pin = FIELD_GET(GPIO_INT_PIN_3_MASK, reg);
		pin &= IRQ_ENABLE_BIT_MASK;
		break;
	default:
		break;
	}

	return pin;
}

static void keembay_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct irq_chip *parent_chip = irq_desc_get_chip(desc);
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned int kmb_irq, source, trig, pin, val;
	unsigned long reg, reg_enable;
	int i;

	kmb_irq = irq_desc_get_irq(desc);

	for (source = 0; source < KEEMBAY_NUM_IRQ_LINES; source++) {
		if (kmb_irq == chip->irq.parents[source])
			break;
	}

	if (source == KEEMBAY_NUM_IRQ_LINES)
		return;

	chained_irq_enter(parent_chip, desc);
	reg = read_reg(kpc->base1 + GPIO_INT_CFG, source);
	reg_enable = reg & (BIT(31) | BIT(23) | BIT(15) | BIT(7));
	trig = keembay_irq[source].trigger;

	/*
	 * Each IRQ line can have up to 4 GPIO pins using it.
	 * Identify which GPIO pin this irq is coming from by
	 * checking each pin's enable bit and input value.
	 * The enable bits checked are bits 7, 15, 23 and 31.
	 */
	for_each_set_bit(i, &reg_enable, 32) {
		unsigned int edge, level;

		pin = keembay_gpio_irq_get_pin(chip, i, reg);
		val = read_gpio_pin_data(kpc->base0 + GPIO_DATA_IN, pin);
		kmb_irq = irq_linear_revmap(chip->irq.domain, pin);
		edge  = trig & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
		level = trig & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW);

		if ((val == 1 && edge) || (val == 1 && level))
			generic_handle_irq(kmb_irq);
	}

	chained_irq_exit(parent_chip, desc);
}

static int keembay_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);

	if (max_gpios_edge_type == 0 &&
	   (type == IRQ_TYPE_EDGE_RISING || type == IRQ_TYPE_EDGE_FALLING)) {
		dev_err(kpc->dev, "IRQ_TYPE_EDGE_RISING interrupt property not specified in Device Tree\n");
		type = IRQ_TYPE_NONE;
	}

	if (max_gpios_level_type == 0 &&
	   (type == IRQ_TYPE_LEVEL_HIGH || type == IRQ_TYPE_LEVEL_LOW)) {
		dev_err(kpc->dev, "IRQ_TYPE_LEVEL_HIGH interrupt property not specified in  Device Tree\n");
		type = IRQ_TYPE_NONE;
	}

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		irq_set_handler_locked(data, handle_edge_irq);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		irq_set_handler_locked(data, handle_level_irq);
		break;

	case IRQ_TYPE_NONE:
	default:
		return -EINVAL;
	}

	return 0;
}

static void keembay_gpio_irq_ack(struct irq_data *data)
{
	/*
	 * IRQ ack is not required from HW perspective,
	 * but this function is needed for handle_edge_irq.
	 */
}

static void keembay_gpio_irq_disable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	unsigned int source, pin, trig, regval = 0, slot = 0;
	struct keembay_pinctrl *kpc = gpiochip_get_data(gc);
	unsigned long flags;
	u32 reg = 0;

	pin = irqd_to_hwirq(data);
	trig = irq_get_trigger_type(data->irq);

	for (source = 0; source < KEEMBAY_NUM_IRQ_LINES; source++) {
		if (keembay_irq[source].active == false)
			continue;

		reg = read_reg(kpc->base1 + GPIO_INT_CFG, source);

		for (slot = 0; slot < ARRAY_SIZE(int_masks); slot++) {
			regval = reg & int_masks[slot];
			if (((regval >> int_cfg_shift[slot]) &
			   IRQ_ENABLE_BIT_MASK) == pin)
				goto out;
		}
	}

	if (source == KEEMBAY_NUM_IRQ_LINES)
		return;

out:	raw_spin_lock_irqsave(&kpc->lock, flags);
	reg &= ~(pin << int_cfg_shift[slot]);
	reg &= ~BIT(slot * 8 + 7);
	write_reg(reg, kpc->base1 + GPIO_INT_CFG, source);
	raw_spin_unlock_irqrestore(&kpc->lock, flags);
	keembay_irq[source].gpios_conn--;
	keembay_irq[source].pins[slot] = 0;
	if (!keembay_irq[source].gpios_conn)
		keembay_irq[source].active = false;

	if (trig == IRQ_TYPE_LEVEL_LOW || trig == IRQ_TYPE_EDGE_FALLING)
		clear_inversion_gpio_output(kpc, pin);

	if (keembay_irq[source].trigger == IRQ_TYPE_LEVEL_HIGH)
		max_gpios_level_type++;
	else if (keembay_irq[source].trigger == IRQ_TYPE_EDGE_RISING)
		max_gpios_edge_type++;
}

static int keembay_find_free_slot(struct keembay_pinctrl *kpc,
				  unsigned int source)
{
	u32 reg;
	int slot;

	reg = read_reg(kpc->base1 + GPIO_INT_CFG, source);

	for (slot = 0; slot < MAX_PINS_PER_IRQ; slot++) {
		reg &= int_masks[slot];
		if (!reg)
			return slot;
	}

	return -EBUSY;
}

static int keembay_find_free_source(struct keembay_pinctrl *kpc,
				    unsigned int trig)
{
	int source, type = 0;

	switch (trig) {
	case IRQ_TYPE_LEVEL_LOW:
	case IRQ_TYPE_LEVEL_HIGH:
		type = IRQ_TYPE_LEVEL_HIGH;
		break;

	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_RISING:
		type = IRQ_TYPE_EDGE_RISING;
		break;
	}

	for (source = 0; source < KEEMBAY_NUM_IRQ_LINES; source++) {
		if (keembay_irq[source].line != 0 &&
		    keembay_irq[source].trigger == type) {
			unsigned int irq_gpios;
			u32 reg;

			reg = read_reg(kpc->base1 + GPIO_INT_CFG, source);
			if (!reg)
				return source;

			irq_gpios = keembay_irq[source].gpios_conn;
			if (irq_gpios != MAX_PINS_PER_IRQ)
				return source;
		}
	}

	return -EBUSY;
}

static void keembay_gpio_irq_enable(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct keembay_pinctrl *kpc = gpiochip_get_data(chip);
	unsigned int trig, pin;
	unsigned long flags;
	int source, slot;
	u32 reg;

	pin = irqd_to_hwirq(data);
	trig = irq_get_trigger_type(data->irq);

	/*
	 * Each IRQ source/line can be shared by 4 GPIO pins.
	 * Check which IRQ source and slot are available.
	 */
	source = keembay_find_free_source(kpc, trig);
	slot = keembay_find_free_slot(kpc, source);

	if (source < 0 || slot < 0)
		return;

	raw_spin_lock_irqsave(&kpc->lock, flags);

	if (trig == IRQ_TYPE_LEVEL_LOW || trig == IRQ_TYPE_EDGE_FALLING)
		set_inversion_gpio_output(kpc, pin);

	reg = read_reg(kpc->base1 + GPIO_INT_CFG, source);
	reg |= (pin << int_cfg_shift[slot]);
	reg |= BIT(slot * 8 + 7);
	write_reg(reg, kpc->base1 + GPIO_INT_CFG, source);

	if (keembay_irq[source].trigger == IRQ_TYPE_LEVEL_HIGH)
		max_gpios_level_type--;
	else if (keembay_irq[source].trigger == IRQ_TYPE_EDGE_RISING)
		max_gpios_edge_type--;

	keembay_irq[source].source = source;
	keembay_irq[source].pins[slot] = pin;
	keembay_irq[source].gpios_conn++;
	keembay_irq[source].active = true;
	raw_spin_unlock_irqrestore(&kpc->lock, flags);
}

static struct irq_chip keembay_gpio_irqchip = {
	.name = "keembay-gpio",
	.irq_enable = keembay_gpio_irq_enable,
	.irq_disable = keembay_gpio_irq_disable,
	.irq_set_type = keembay_gpio_irq_set_type,
	.irq_ack = keembay_gpio_irq_ack,
};

static void keembay_update_gpios_allocation(int level, int edge)
{
	max_gpios_level_type = level * 4;
	max_gpios_edge_type  = edge * 4;
}

static int keembay_gpiochip_probe(struct keembay_pinctrl *kpc,
				  struct platform_device *pdev)
{
	int ret, i, level_line = 0, edge_line = 0;
	struct gpio_chip *chip = &kpc->chip;
	struct gpio_irq_chip *girq;

	/* Setup GPIO chip */
	chip->label		= dev_name(kpc->dev);
	chip->parent		= kpc->dev;
	chip->request		= gpiochip_generic_request;
	chip->free		= gpiochip_generic_free;
	chip->get_direction	= keembay_gpio_get_direction;
	chip->direction_input	= keembay_gpio_set_direction_input;
	chip->direction_output  = keembay_gpio_set_direction_output;
	chip->get		= keembay_gpio_get;
	chip->set               = keembay_gpio_set;
	chip->base		= -1;
	chip->ngpio		= kpc->soc->npins;

	/* Setup GPIO IRQ chip */
	girq			= &kpc->chip.irq;
	girq->chip		= &keembay_gpio_irqchip;
	girq->parent_handler	= keembay_gpio_irq_handler;
	girq->num_parents	= KEEMBAY_NUM_IRQ_LINES;
	girq->parents		= devm_kcalloc(kpc->dev,
					       KEEMBAY_NUM_IRQ_LINES,
					       sizeof(*girq->parents),
					       GFP_KERNEL);

	if (!girq->parents)
		return -ENOMEM;

	for (i = 0; i < KEEMBAY_NUM_IRQ_LINES; i++) {
		girq->parents[i] = platform_get_irq(pdev, i);
		if (girq->parents[i] < 0) {
			dev_err(kpc->dev, "Failed to map IRQ line %d\n", i);
			continue;
		}

		keembay_irq[i].line = girq->parents[i];
		keembay_irq[i].source = i;
		keembay_irq[i].trigger = irq_get_trigger_type(girq->parents[i]);
		keembay_irq[i].gpios_conn = 0;
		keembay_irq[i].active = false;

		if (keembay_irq[i].trigger == IRQ_TYPE_LEVEL_HIGH)
			level_line++;
		else
			edge_line++;
	}

	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;

	keembay_update_gpios_allocation(level_line, edge_line);

	ret = devm_gpiochip_add_data(kpc->dev, chip, kpc);
	if (ret) {
		dev_err(kpc->dev, "Failed to add gpiochip\n");
		return ret;
	}

	ret = gpiochip_add_pin_range(chip, dev_name(kpc->dev), 0, 0,
				     chip->ngpio);
	if (ret) {
		gpiochip_remove(chip);
		return ret;
	}

	return 0;
}

static int keembay_build_groups(struct keembay_pinctrl *kpc)
{
	struct group_desc *keembay_groups;
	int i;

	kpc->ngroups = kpc->soc->npins;
	keembay_groups = devm_kcalloc(kpc->dev, kpc->ngroups,
				      sizeof(*keembay_groups), GFP_KERNEL);
	if (!keembay_groups)
		return -ENOMEM;

	/* Each pin is categorised as one group */
	for (i = 0; i < kpc->ngroups; i++) {
		struct group_desc *group = keembay_groups + i;
		const struct pinctrl_pin_desc *pin_info = keembay_pins + i;

		group->name = pin_info->name;
		group->pins = (int *)&pin_info->number;
		pinctrl_generic_add_group(kpc->pctrl, group->name,
					  group->pins, 1, NULL);
	}

	return 0;
}

static int keembay_add_functions(struct keembay_pinctrl *kpc,
				struct function_desc *funcs)
{
	struct function_desc *function = funcs;
	int i;

	/* Assign the groups for each function */
	for (i = 0; i < kpc->soc->npins; i++) {
		const struct pinctrl_pin_desc *pin_info = keembay_pins + i;
		struct keembay_mux_desc *pin_mux = pin_info->drv_data;

		while (pin_mux->name) {
			const char **grp;
			int j, grp_num, match = 0;
			size_t grp_size;
			struct function_desc *func;

			for (j = 0; j < kpc->nfuncs; j++) {
				if (!strcmp(pin_mux->name, function[j].name)) {
					match = 1;
					break;
				}
			}

			if (!match)
				return -EINVAL;

			func = function + j;
			grp_num = func->num_group_names;
			grp_size = sizeof(*func->group_names);

			if (!func->group_names) {
				func->group_names = devm_kcalloc(kpc->dev,
								grp_num,
								grp_size,
								GFP_KERNEL);
				if (!func->group_names) {
					devm_kfree(kpc->dev, func);
					return -ENOMEM;
				}
			}

			grp = func->group_names;
			while (*grp)
				grp++;

			*grp = pin_info->name;
			pin_mux++;
		}
	}

	/* Add all functions */
	for (i = 0; i < kpc->nfuncs; i++) {
		pinmux_generic_add_function(kpc->pctrl,
					    function[i].name,
					    function[i].group_names,
					    function[i].num_group_names,
					    function[i].data);
	}

	return 0;
}

static int keembay_build_functions(struct keembay_pinctrl *kpc)
{
	struct function_desc *keembay_funcs, *kmb_funcs;
	int i;

	/*
	 * Total number of functions is unknown at this point.
	 * Allocate first.
	 */
	kpc->nfuncs = 0;
	keembay_funcs = kcalloc(kpc->soc->npins * 8,
				sizeof(*keembay_funcs), GFP_KERNEL);
	if (!keembay_funcs)
		return -ENOMEM;

	/* Find total number of functions and each's properties */
	for (i = 0; i < kpc->soc->npins; i++) {
		const struct pinctrl_pin_desc *pin_info = keembay_pins + i;
		struct keembay_mux_desc *pin_mux = pin_info->drv_data;

		while (pin_mux->name) {
			struct function_desc *func = keembay_funcs;

			while (func->name) {
				if (!strcmp(pin_mux->name, func->name)) {
					func->num_group_names++;
					break;
				}

				func++;
			}

			if (!func->name) {
				func->name = pin_mux->name;
				func->num_group_names = 1;
				func->data = (u8 *)&pin_mux->mode;
				kpc->nfuncs++;
			}

			pin_mux++;
		}
	}

	/* Reallocate memory based on actual number of functions */
	kmb_funcs = krealloc(keembay_funcs,
				 kpc->nfuncs * sizeof(*keembay_funcs),
				 GFP_KERNEL);
	if (!keembay_funcs)
		return -ENOMEM;

	return keembay_add_functions(kpc, kmb_funcs);
}

static const struct keembay_pin_soc keembay_data = {
	.pins    = keembay_pins,
	.npins   = ARRAY_SIZE(keembay_pins),
};

static const struct of_device_id keembay_pinctrl_match[] = {
	{ .compatible = "intel,keembay-pinctrl",
	  .data = &keembay_data },
	{ }
};
MODULE_DEVICE_TABLE(of, keembay_pinctrl_match);

static int keembay_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct keembay_pinctrl *kpc;
	int ret;

	kpc = devm_kzalloc(dev, sizeof(*kpc), GFP_KERNEL);
	if (!kpc)
		return -ENOMEM;

	kpc->dev = dev;
	kpc->soc = (struct keembay_pin_soc *)device_get_match_data(dev);

	kpc->base0 =  devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(kpc->base0))
		return PTR_ERR(kpc->base0);

	kpc->base1 = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(kpc->base1))
		return PTR_ERR(kpc->base1);

	keembay_pinctrl_desc.pins = kpc->soc->pins;
	keembay_pinctrl_desc.npins = kpc->soc->npins;

	kpc->pctrl = devm_pinctrl_register(dev, &keembay_pinctrl_desc, kpc);
	if (IS_ERR(kpc->pctrl))
		return PTR_ERR(kpc->pctrl);

	ret = keembay_build_groups(kpc);
	if (ret)
		return ret;

	ret = keembay_build_functions(kpc);
	if (ret)
		return ret;

	raw_spin_lock_init(&kpc->lock);

	ret = keembay_gpiochip_probe(kpc, pdev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, kpc);

	return 0;
}

static struct platform_driver keembay_pinctrl_driver = {
	.driver = {
		.name = "keembay-pinctrl",
		.of_match_table = keembay_pinctrl_match,
	},
	.probe = keembay_pinctrl_probe,
};
module_platform_driver(keembay_pinctrl_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vineetha G. Jaya Kumaran <vineetha.g.jaya.kumaran@intel.com>");
MODULE_AUTHOR("Muhammad Husaini Zulkifli <muhammad.husaini.zulkifli@intel.com>");
MODULE_DESCRIPTION("Intel Keem Bay SoC pinctrl/GPIO driver");
