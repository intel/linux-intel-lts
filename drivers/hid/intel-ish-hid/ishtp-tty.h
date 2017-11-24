#ifndef __ISHTP_TTY_H
#define __ISHTP_TTY_H

#include <uapi/linux/uuid.h>

static const uuid_le tty_ishtp_guid = UUID_LE(0x6f2647c7, 0x3e16, 0x4d79,
						0xb4, 0xff, 0x02, 0x89, 0x28,
						0xee, 0xeb, 0xca);

#define	ISH_DEBUG	1
#if ISH_DEBUG
#define	ISH_DBG_PRINT	printk
#else
#define	ISH_DBG_PRINT	no_printk
#endif

//extern wait_queue_head_t	ishtp_tty_wait;

#define ISHTP_MAX_MSG_SIZE	4000


#define UART_GET_CONFIG	1
#define UART_SET_CONFIG 2
#define UART_SEND_DATA	3
#define UART_RECV_DATA	4
#define UART_ABORT_WRITE 5
#define UART_ABORT_READ 6

struct ishtp_tty_msg_hdr {
	uint8_t command;	/* Bit 7 : is_response */
#define CMD_MASK	0x7F
#define IS_RESPONSE	0x80
	uint8_t status;
	uint16_t size;
} __packed;

struct ishtp_tty_msg {
	struct ishtp_tty_msg_hdr hdr;
//	void payload[0];
} __packed;


struct uart_config {
	uint32_t baud;
	uint8_t bits_length:4;
	uint8_t stop_bits:2;
	uint8_t parity:1;
	uint8_t even_parity:1;
	uint8_t flow_control:1;
	uint8_t reserved:7;
};

#endif

