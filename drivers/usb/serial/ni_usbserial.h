/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NI USB Serial adapter driver
 *
 * Copyright (C) 2017-2022 National Instruments
 *
 * The NI USB-232 and USB-485 products are based around a TUSB5052 chipset,
 * but with their own firmware and protocol (thus the ti_usb_3410_5052 driver
 * does not work with them).
 *
 * The protocol is message-based, using the following endpoints:
 *
 *        EP0 IN/OUT - Ctrl - Device Config
 *   Ifc0 EP1 IN/OUT - Bulk - Device Commands/Status
 *   Ifc0 EP2 IN/OUT - Bulk - Port A Commands/Status
 *   Ifc0 EP3 IN/OUT - Bulk - Port A RX/TX Data
 *   Ifc1 EP4 IN/OUT - Bulk - Port B Commands/Status
 *   Ifc1 EP5 IN/OUT - Bulk - Port B RX/TX Data
 *
 * There are no useful "Device Commands" that can't also be sent to the Port-
 * specific endpoints, so that pair of endpoints can be safely ignored.
 *
 * Single-port devices omit the second interface and EP4/EP5 endpoint pairs.
 * Quad-port devices are implemented as a pair of dual-port devices, with
 * the "port 1 and 2" device also acting as an upstream USB hub for the
 * "port 3 and 4" device.
 */

#ifndef __LINUX_USB_SERIAL_NI_USBSERIAL_H
#define __LINUX_USB_SERIAL_NI_USBSERIAL_H

#define NI_VENDOR_ID		0x3923
#define NI_PRELOADER_ID		0x703C /* Pre-enumeration device */
#define NI_USB232x1_ID		0x7014 /* NI USB-232/1 */
#define NI_USB232x2_ID		0x7010 /* NI USB-232/2 */
#define NI_USB232x4_12_ID	0x7011 /* NI USB-232/4 (Ports 1&2) */
#define NI_USB232x4_34_ID	0x7025 /* NI USB-232/4 (Ports 3&4) */
#define NI_USB485x1_ID		0x7015 /* NI USB-232/1 */
#define NI_USB485x2_ID		0x7012 /* NI USB-232/2 */
#define NI_USB485x4_12_ID	0x7013 /* NI USB-232/4 (Ports 1&2) */
#define NI_USB485x4_34_ID	0x7026 /* NI USB-232/4 (Ports 3&4) */
#define NI_EXPRESSCARD8420_ID	0x7188 /* NI ExpressCard-8420 (RS-232/2) */
#define NI_EXPRESSCARD8421_ID	0x7189 /* NI ExpressCard-8421 (RS-485/2) */

/* Loader commands */
#define LOADER_CMD_READ_EEPROM	0x92
#define EEPROM_PID_HIGH		0x16
#define EEPROM_PID_LOW		0x17
#define MAX_FIRMWARE_SIZE	16384
#define MAX_FWLOAD_PACKET_SIZE	64

/* Command/Status codes (EP 2 and EP 4) */
#define NISER_CMD_NULL		0x00
#define NISER_STS_BAD_COMMAND	0x32	/* FW didn't like a cmd (arg: u8) */

#define NISER_CMD_ENABLE_FEATURES	0x01	/* Enable features (u8) */
#define   NISER_FEATURE_LEDS	BIT(0)	/* enable LED activity */
#define   NISER_FEATURE_RX	BIT(1)	/* receive RX data */
#define   NISER_FEATURE_MODEM	BIT(2)	/* modem statuses + TX_DATA_DONE */
#define NISER_CMD_DISABLE_FEATURES	0x02	/* Disable features (arg: u8) */
#define NISER_CMD_GET_FEATURES		0x4A
#define NISER_STS_FEATURES		0x4B

#define NISER_STS_TX_DATA_DONE	0x03	/* TX DMA complete (no arg) */
#define NISER_CMD_RESET_UART	0x05	/* reset UART config (no arg) */

#define NISER_CMD_SET_LOOPBACK	0x06	/* Set loopback (no args) */
#define NISER_CMD_CLR_LOOPBACK	0x07	/* Clear loopback (no args) */
#define NISER_CMD_SET_DTR	0x0A	/* Set DTR (no args) */
#define NISER_CMD_CLR_DTR	0x0B	/* Clear DTR (no args) */
#define NISER_CMD_SET_RTS	0x0C	/* Set RTS (no args) */
#define NISER_CMD_CLR_RTS	0x0D	/* Clear RTS (no args) */
#define NISER_CMD_SET_RI	0x0E	/* Set RI when loopback/DCE */
#define NISER_CMD_CLR_RI	0x0F	/* Clear RI (no args) */
#define NISER_CMD_SET_DCD	0x10	/* Set DCD when loopback/DCE */
#define NISER_CMD_CLR_DCD	0x11	/* Clear RI (no args) */

#define NISER_CMD_SET_BAUDRATE	0x12	/* Set baudrate (arg: be32) */
#define NISER_CMD_GET_BAUDRATE	0x13	/* Request baudrate (no args) */
#define NISER_STS_BAUDRATE	0x14	/* Current baudrate (arg: be32) */

#define NISER_CMD_SET_CHARSIZE	0x15	/* Set char size (arg: u8) */
#define   NISER_CHARSIZE_5BITS	0
#define   NISER_CHARSIZE_6BITS	1
#define   NISER_CHARSIZE_7BITS	2
#define   NISER_CHARSIZE_8BITS	3

#define NISER_CMD_SET_STOPBITS	0x16	/* Set stop bits (arg: u8) */
#define   NISER_STOPBITS_1	0
#define   NISER_STOPBITS_15_2	1	/* 1.5 if 5-bit, else 2 */

#define NISER_CMD_ENABLE_PARITY	0x17	/* Enables parity check/gen (arg: u8) */
#define   NISER_NOPARITY	0
#define   NISER_PARITY		1
#define NISER_CMD_SET_PARITY	0x18	/* Sets parity mode (arg: u8) */
#define   NISER_PARITY_ODD	0
#define   NISER_PARITY_EVEN	1
#define NISER_CMD_SET_PARITY_FORCED	0x19 /* Sets forced parity (u8) */
#define   NISER_PARITY_NOT_FORCED	0 /* not forced (ODD/EVEN) */
#define   NISER_PARITY_FORCED		1 /* forced (MARK/SPACE) */

#define NISER_CMD_SET_BREAK	0x1A	/* Set TX break condition (no arg) */
#define NISER_CMD_CLR_BREAK	0x1B	/* Clear TX break condition */

#define NISER_CMD_SET_TXOF	0x1C	/* Set XON/XOFF TX flow control */
#define NISER_CMD_CLR_TXOF	0x1D	/* Clear XON/XOFF TX flow control */
#define NISER_CMD_SET_TXOA	0x1E	/* Set XON-on-any/XOFF flow control */
#define NISER_CMD_CLR_TXOA	0x1F	/* Clear XON-on-any/XOFF flow control */
#define NISER_CMD_SET_CTSFC	0x20	/* Set CTS flow control */
#define NISER_CMD_CLR_CTSFC	0x21	/* Clear CTS flow control */
#define NISER_CMD_SET_DSRFC	0x22	/* Set DSR flow control */
#define NISER_CMD_CLR_DSRFC	0x23	/* Clear DSR flow control */
#define NISER_CMD_SET_RXOF	0x24	/* Set XON/XOFF RX flow control */
#define NISER_CMD_CLR_RXOF	0x25	/* Clear XON/XOFF RX flow control */
#define NISER_CMD_SET_RTSFC	0x26	/* Set RTS flow control */
#define NISER_CMD_CLR_RTSFC	0x27	/* Clear RTS flow control */
#define NISER_CMD_SET_DTRFC	0x28	/* Set DTR flow control */
#define NISER_CMD_CLR_DTRFC	0x29	/* Clear DTR flow control */
#define NISER_CMD_SET_XONCHAR	0x2A	/* Set XON character (arg: u8) */
#define NISER_CMD_SET_XOFFCHAR	0x2B	/* Set XOFF character (arg: u8) */

#define NISER_CMD_GET_MSR	0x2F	/* Request modem status (no arg) */
#define NISER_STS_MSR		0x30	/* Current modem status (arg: u8) */
#define   NISER_MSR_CTSON	BIT(0)
#define   NISER_MSR_DSRON	BIT(1)
#define   NISER_MSR_RION	BIT(2)
#define   NISER_MSR_DCDON	BIT(3)

#define NISER_STS_LSR		0x31	/* Current line status (arg: u8) */
#define   NISER_LSR_RXOVERRUN	BIT(0)	/* RX overrun */
#define   NISER_LSR_RXPARITY	BIT(1)	/* RX parity error */
#define   NISER_LSR_RXFRAMING	BIT(2)	/* RX framing error */
#define   NISER_LSR_RXBREAK	BIT(3)	/* RX break condition */
#define NISER_CMD_CLR_LSR	0x08	/* Clear error status (no args) */

/* RS-422/488-specific functionality */
#define NISER_CMD_SET_WIREMODE	0x2C	/* Set RS-422/485 wire mode */
#define   NISER_WIREMODE_RS422		0
#define   NISER_WIREMODE_ECHO_RS485	1
#define   NISER_WIREMODE_DTR_RS485	2
#define   NISER_WIREMODE_AUTO_RS485	3
#define NISER_CMD_GET_WIREMODE	0x2D	/* Request wire mode update (no arg) */
#define NISER_STS_WIREMODE	0x2E	/* Current wire mode (arg: u8) */
#define NISER_CMD_SET_485BIAS	0x80	/* Enable 485 bias resistors */
#define NISER_CMD_CLR_485BIAS	0x81	/* Disable 485 bias resistors */

#endif
