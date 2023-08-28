// SPDX-License-Identifier: GPL-2.0+
/* Synopsys DesignWare 8250 library. */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/property.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>

#include "8250_dwlib.h"

/* Offsets for the DesignWare specific registers */
#define DW_UART_TCR	0xac /* Transceiver Control Register (RS485) */
#define DW_UART_DE_EN	0xb0 /* Driver Output Enable Register */
#define DW_UART_RE_EN	0xb4 /* Receiver Output Enable Register */
#define DW_UART_DLF	0xc0 /* Divisor Latch Fraction Register */
#define DW_UART_RAR	0xc4 /* Receive Address Register */
#define DW_UART_TAR	0xc8 /* Transmit Address Register */
#define DW_UART_LCR_EXT	0xcc /* Line Extended Control Register */
#define DW_UART_CPR	0xf4 /* Component Parameter Register */
#define DW_UART_UCV	0xf8 /* UART Component Version */

/* Trasceiver Control Register bits */
#define DW_UART_TCR_RS485_EN		BIT(0)
#define DW_UART_TCR_RE_POL		BIT(1)
#define DW_UART_TCR_DE_POL		BIT(2)
#define DW_UART_TCR_XFER_MODE(_mode_)	((_mode_) << 3)

/* Line Extended Control Register bits */
#define DW_UART_LCR_EXT_DLS_E		BIT(0)
#define DW_UART_LCR_EXT_ADDR_MATCH	BIT(1)
#define DW_UART_LCR_EXT_SEND_ADDR	BIT(2)
#define DW_UART_LCR_EXT_TRANSMIT_MODE	BIT(3)

/* Component Parameter Register bits */
#define DW_UART_CPR_ABP_DATA_WIDTH	(3 << 0)
#define DW_UART_CPR_AFCE_MODE		(1 << 4)
#define DW_UART_CPR_THRE_MODE		(1 << 5)
#define DW_UART_CPR_SIR_MODE		(1 << 6)
#define DW_UART_CPR_SIR_LP_MODE		(1 << 7)
#define DW_UART_CPR_ADDITIONAL_FEATURES	(1 << 8)
#define DW_UART_CPR_FIFO_ACCESS		(1 << 9)
#define DW_UART_CPR_FIFO_STAT		(1 << 10)
#define DW_UART_CPR_SHADOW		(1 << 11)
#define DW_UART_CPR_ENCODED_PARMS	(1 << 12)
#define DW_UART_CPR_DMA_EXTRA		(1 << 13)
#define DW_UART_CPR_FIFO_MODE		(0xff << 16)

/* Helper for FIFO size calculation */
#define DW_UART_CPR_FIFO_SIZE(a)	(((a >> 16) & 0xff) * 16)

static inline u32 dw8250_readl_ext(struct uart_port *p, int offset)
{
	if (p->iotype == UPIO_MEM32BE)
		return ioread32be(p->membase + offset);
	return readl(p->membase + offset);
}

static inline void dw8250_writel_ext(struct uart_port *p, int offset, u32 reg)
{
	if (p->iotype == UPIO_MEM32BE)
		iowrite32be(reg, p->membase + offset);
	else
		writel(reg, p->membase + offset);
}

/*
 * divisor = div(I) + div(F)
 * "I" means integer, "F" means fractional
 * quot = div(I) = clk / (16 * baud)
 * frac = div(F) * 2^dlf_size
 *
 * let rem = clk % (16 * baud)
 * we have: div(F) * (16 * baud) = rem
 * so frac = 2^dlf_size * rem / (16 * baud) = (rem << dlf_size) / (16 * baud)
 */
static unsigned int dw8250_get_divisor(struct uart_port *p, unsigned int baud,
				       unsigned int *frac)
{
	unsigned int quot, rem, base_baud = baud * 16;
	struct dw8250_port_data *d = p->private_data;

	quot = p->uartclk / base_baud;
	rem = p->uartclk % base_baud;
	*frac = DIV_ROUND_CLOSEST(rem << d->dlf_size, base_baud);

	return quot;
}

static void dw8250_set_divisor(struct uart_port *p, unsigned int baud,
			       unsigned int quot, unsigned int quot_frac)
{
	dw8250_writel_ext(p, DW_UART_DLF, quot_frac);
	serial8250_do_set_divisor(p, baud, quot, quot_frac);
}

void dw8250_do_set_termios(struct uart_port *p, struct ktermios *termios, struct ktermios *old)
{
	p->status &= ~UPSTAT_AUTOCTS;
	if (termios->c_cflag & CRTSCTS)
		p->status |= UPSTAT_AUTOCTS;

	serial8250_do_set_termios(p, termios, old);
}
EXPORT_SYMBOL_GPL(dw8250_do_set_termios);
static int dw8250_rs485_config(struct uart_port *p, struct serial_rs485 *rs485)
{
	u32 re_en, de_en;
	u32 lcr = 0;
	u32 tcr;

	/* Clearing unsupported flags. */
	rs485->flags &= SER_RS485_ENABLED | SER_RS485_9BIT_ENABLED |
			SER_RS485_9BIT_TX_ADDR | SER_RS485_9BIT_RX_ADDR |
			SER_RS485_RX_DURING_TX | SER_RS485_RX_OR_TX |
			SER_RS485_SW_TX_MODE;

	tcr = dw8250_readl_ext(p, DW_UART_TCR);
	/* Reset previous Transfer Mode */
	tcr &= ~DW_UART_TCR_XFER_MODE(3);

	/* REVISIT: Only supporting Hardware Controlled Half Duplex & Duplex mode. */
	if (rs485->flags & SER_RS485_ENABLED) {

		/* Using SER_RS485_RX_DURING_TX to indicate Full Duplex Mode */
		if (rs485->flags & SER_RS485_RX_DURING_TX) {
			tcr |= DW_UART_TCR_RS485_EN | DW_UART_TCR_XFER_MODE(0);
			re_en = 1;
			de_en = 1;
		}
		/* Using SER_RS485_RX_OR_TX to indicate SW Half Duplex Mode */
		else if (rs485->flags & SER_RS485_RX_OR_TX) {
			tcr |= DW_UART_TCR_RS485_EN | DW_UART_TCR_XFER_MODE(1);
			if (rs485->flags & SER_RS485_SW_TX_MODE) {
				re_en = 0;
				de_en = 1;
			} else {
				re_en = 1;
				de_en = 0;
			}
		} else {
			tcr |= DW_UART_TCR_RS485_EN | DW_UART_TCR_XFER_MODE(2);
			re_en = 1;
			de_en = 1;
		}
		dw8250_writel_ext(p, DW_UART_DE_EN, de_en);
		dw8250_writel_ext(p, DW_UART_RE_EN, re_en);
	} else {
		tcr &= ~(DW_UART_TCR_RS485_EN | DW_UART_TCR_XFER_MODE(3));
		dw8250_writel_ext(p, DW_UART_DE_EN, 0);
		dw8250_writel_ext(p, DW_UART_RE_EN, 0);
	}

	/* Resetting the default DE_POL & RE_POL */
	tcr &= ~(DW_UART_TCR_DE_POL | DW_UART_TCR_RE_POL);

	if (device_property_read_bool(p->dev, "snps,de-active-high"))
		tcr |= DW_UART_TCR_DE_POL;
	if (device_property_read_bool(p->dev, "snps,re-active-high"))
		tcr |= DW_UART_TCR_RE_POL;

	dw8250_writel_ext(p, DW_UART_TCR, tcr);

	/*
	 * XXX: Though we could interpret the "RTS" timings as Driver Enable
	 * (DE) assertion/de-assertion timings, initially not supporting that.
	 * Ideally we should have timing values for the Driver instead of the
	 * RTS signal.
	 */
	rs485->delay_rts_before_send = 0;
	rs485->delay_rts_after_send = 0;

	/* XXX: Proof of concept for 9-bit transfer mode. */
	if (rs485->flags & SER_RS485_9BIT_ENABLED) {
		/* Clear TAR & RAR of any previous values */
		dw8250_writel_ext(p, DW_UART_TAR, 0x0);
		dw8250_writel_ext(p, DW_UART_RAR, 0x0);
		lcr = DW_UART_LCR_EXT_DLS_E;
		dw8250_writel_ext(p, DW_UART_LCR_EXT, lcr);
		if (rs485->flags & SER_RS485_9BIT_TX_ADDR) {
			dw8250_writel_ext(p, DW_UART_TAR, rs485->padding[0]);
			lcr |= DW_UART_LCR_EXT_SEND_ADDR;
		} else if (rs485->flags & SER_RS485_9BIT_RX_ADDR) {
			dw8250_writel_ext(p, DW_UART_RAR, rs485->padding[0]);
			lcr |= DW_UART_LCR_EXT_ADDR_MATCH;
		}
	}

	dw8250_writel_ext(p, DW_UART_LCR_EXT, lcr);

	p->rs485 = *rs485;

	return 0;
}

void dw8250_setup_port(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	u32 reg, old_dlf;

	if (device_property_read_bool(p->dev, "snps,rs485-interface-en"))
		p->rs485_config = dw8250_rs485_config;

	/*
	 * If the Component Version Register returns zero, we know that
	 * ADDITIONAL_FEATURES are not enabled. No need to go any further.
	 */
	reg = dw8250_readl_ext(p, DW_UART_UCV);
	if (!reg)
		return;

	dev_dbg(p->dev, "Designware UART version %c.%c%c\n",
		(reg >> 24) & 0xff, (reg >> 16) & 0xff, (reg >> 8) & 0xff);

	/* Preserve value written by firmware or bootloader  */
	old_dlf = dw8250_readl_ext(p, DW_UART_DLF);
	dw8250_writel_ext(p, DW_UART_DLF, ~0U);
	reg = dw8250_readl_ext(p, DW_UART_DLF);
	dw8250_writel_ext(p, DW_UART_DLF, old_dlf);

	if (reg) {
		struct dw8250_port_data *d = p->private_data;

		d->dlf_size = fls(reg);
		p->get_divisor = dw8250_get_divisor;
		p->set_divisor = dw8250_set_divisor;
	}

	reg = dw8250_readl_ext(p, DW_UART_CPR);
	if (!reg)
		return;

	/* Select the type based on FIFO */
	if (reg & DW_UART_CPR_FIFO_MODE) {
		p->type = PORT_16550A;
		p->flags |= UPF_FIXED_TYPE;
		p->fifosize = DW_UART_CPR_FIFO_SIZE(reg);
		up->capabilities = UART_CAP_FIFO;
	}

	if (reg & DW_UART_CPR_AFCE_MODE)
		up->capabilities |= UART_CAP_AFE;

	if (reg & DW_UART_CPR_SIR_MODE)
		up->capabilities |= UART_CAP_IRDA;
}
EXPORT_SYMBOL_GPL(dw8250_setup_port);
