/* SPDX-License-Identifier: GPL-2.0 only */
/*
 * vpu-cdc-acm.h - XLink USB Remote Host driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CMSPAR
#define CMSPAR			0
#endif

/*
 * Major and minor numbers.
 */

#define ACM_TTY_MAJOR		166
#define ACM_TTY_MINORS		256

/*
 * Requests.
 */

#define USB_RT_ACM		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)

/*
 * Output control lines.
 */

#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02

/*
 * Input control lines and line errors.
 */

#define ACM_CTRL_DCD		0x01
#define ACM_CTRL_DSR		0x02
#define ACM_CTRL_BRK		0x04
#define ACM_CTRL_RI		0x08

#define ACM_CTRL_FRAMING	0x10
#define ACM_CTRL_PARITY		0x20
#define ACM_CTRL_OVERRUN	0x40

int vpu_write_host(u32 id, const char *buf,
					size_t *size, unsigned int timeout_ms);
int vpu_close_host(u32 id, int len);
int vpu_read_host(u32 id, void *data, size_t *size, unsigned int timeout_ms);
int usb_get_device_num(uint32_t *id_list);
int usb_get_device_name_by_id(u32 id, char *device_name, size_t name_size);
int usb_get_device_status_by_id(u32 id, uint32_t *status);
int usb_device_reset(u8 cmd);
int vpu_write_reset_host(u32 id);
int vpu_close_reset_host(u32 id);
int usb_connect_device(uint32_t sw_device_id);
/*
 * Internal driver structures.
 */

/*
 * The only reason to have several buffers is to accommodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define ACM_NW  16
#define ACM_NR  16

struct acm_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb		*urb;
	struct acm		*instance;
};

struct acm_rb {
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
	int			index;
	struct acm		*instance;
};



#define CDC_DATA_INTERFACE_TYPE	0x0a

/* constants describing various quirks and errors */
#define NO_UNION_NORMAL			BIT(0)
#define SINGLE_RX_URB			BIT(1)
#define NO_CAP_LINE			BIT(2)
#define NO_DATA_INTERFACE		BIT(4)
#define IGNORE_DEVICE			BIT(5)
#define QUIRK_CONTROL_LINE_STATE	BIT(6)
#define CLEAR_HALT_CONDITIONS		BIT(7)
#define SEND_ZERO_PACKET		BIT(8)
#define DISABLE_ECHO			BIT(9)
