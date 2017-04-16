// SPDX-License-Identifier: GPL-2.0
/*
 * NI USB-232 and USB-485 serial adapter driver
 *
 * Copyright (C) 2017-2022 National Instruments Corp.
 *
 * See Documentation/usb/usb-serial.rst for more information on using this
 * driver
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <asm/unaligned.h>

#include "ni_usbserial.h"

#define DRIVER_AUTHOR "Brenda Streiff <brenda.streiff@ni.com>"
#define DRIVER_DESC "NI USB Serial driver"

struct niser_port_private {
	u32	max_baudrate;	/* Maximum baudrate of device */
	int	is_485:1;	/* Device is RS-485 (else, is RS-232) */

	/* USB state information */
	int		cmd_pipe;
	int		sts_pipe;
	struct urb	*sts_urbs[2];

	spinlock_t port_lock;	/* protects all of the following */
	struct serial_rs485 rs485;
	u8	msr;		/* Shadow of NISER_MSR_* flags */
	u8	lsr;		/* Shadow of NISER_LSR_* flags */
	int	tiocm_mcr;	/* TIOCM_* flags set via tiocmset */
};

static void niser_port_remove(struct usb_serial_port *port);
static void niser_set_termios(struct tty_struct *tty,
			      struct usb_serial_port *port,
			      struct ktermios *old_termios);

static void niser_port_sts_callback(struct urb *urb);

static int niser_identify_features(struct usb_serial_port *port)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	u16 product = le16_to_cpu(port->serial->dev->descriptor.idProduct);

	switch (product) {
	case NI_USB232x1_ID:
	case NI_USB232x2_ID:
	case NI_USB232x4_12_ID:
	case NI_USB232x4_34_ID:
	case NI_EXPRESSCARD8420_ID:
		priv->is_485 = 0;
		priv->max_baudrate = 230400;
		break;
	case NI_USB485x1_ID:
	case NI_USB485x2_ID:
	case NI_USB485x4_12_ID:
	case NI_USB485x4_34_ID:
	case NI_EXPRESSCARD8421_ID:
		priv->is_485 = 1;
		priv->max_baudrate = 460800;
		break;
	}

	return 0;
}

static int niser_port_setup_status_urbs(struct usb_serial_port *port)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	const int buffer_size = 64;
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->sts_urbs); ++i) {
		unsigned char *buf = kzalloc(buffer_size, GFP_KERNEL);

		if (!buf)
			goto free_urbs;

		priv->sts_urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!priv->sts_urbs[i]) {
			kfree(buf);
			goto free_urbs;
		}

		usb_fill_bulk_urb(priv->sts_urbs[i], port->serial->dev,
				  priv->sts_pipe, buf, buffer_size,
				  niser_port_sts_callback, port);
		priv->sts_urbs[i]->transfer_flags |= URB_FREE_BUFFER;
	}

	return 0;

free_urbs:
	for (i = 0; i < ARRAY_SIZE(priv->sts_urbs); ++i)
		usb_free_urb(priv->sts_urbs[i]);
	return -ENOMEM;
}

static int niser_port_submit_status_urb(struct usb_serial_port *port,
					struct urb *urb, gfp_t mem_flags)
{
	int res;

	res = usb_submit_urb(urb, mem_flags);
	if (res) {
		if (res != -EPERM && res != -ENODEV) {
			dev_err(&port->dev, "%s - usb_submit_urb failed: %d\n",
				__func__, res);
		}
		return res;
	}

	return 0;
}

static int niser_port_submit_status_urbs(struct usb_serial_port *port)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	int status;
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->sts_urbs); ++i) {
		status = niser_port_submit_status_urb(port, priv->sts_urbs[i],
						      GFP_KERNEL);
		if (status)
			goto err;
	}

	return 0;

err:
	for (; i >= 0; --i)
		usb_kill_urb(priv->sts_urbs[i]);

	return status;
}

static int niser_send_cmd(struct usb_serial_port *port, void *cmd, int cmd_len)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	char *buffer;
	int status;
	int actual;

	buffer = kzalloc(cmd_len, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	memcpy(buffer, cmd, cmd_len);

	status = usb_bulk_msg(port->serial->dev, priv->cmd_pipe,
			      buffer, cmd_len, &actual, 1000);

	kfree(buffer);

	return status;
}

static int niser_port_probe(struct usb_serial_port *port)
{
	struct niser_port_private *priv;
	u8 cmd_ep_addr, sts_ep_addr;
	u8 cmd_reset_uart = NISER_CMD_RESET_UART;
	int status;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	usb_set_serial_port_data(port, priv);

	spin_lock_init(&priv->port_lock);

	niser_identify_features(port);

	/*
	 * The endpoint pair for command/status in this interface
	 * precedes the endpoint pair for TX/RX.
	 */
	cmd_ep_addr = port->bulk_out_endpointAddress - 1;
	sts_ep_addr = port->bulk_in_endpointAddress - 1;

	priv->cmd_pipe = usb_sndbulkpipe(port->serial->dev, cmd_ep_addr);
	priv->sts_pipe = usb_rcvbulkpipe(port->serial->dev, sts_ep_addr);

	/* Create the read URBs for getting status back */
	status = niser_port_setup_status_urbs(port);
	if (status < 0)
		goto failure;

	status = niser_port_submit_status_urbs(port);
	if (status < 0)
		goto failure;

	/* Reset UART to initial state. */
	status = niser_send_cmd(port, &cmd_reset_uart, sizeof(cmd_reset_uart));

	if (status < 0)
		goto failure;

	return 0;

failure:
	dev_err(&port->serial->dev->dev, "%s: failure %d\n", __func__, status);
	niser_port_remove(port);
	return status;
}

static void niser_port_remove(struct usb_serial_port *port)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->sts_urbs); ++i) {
		usb_kill_urb(priv->sts_urbs[i]);
		usb_free_urb(priv->sts_urbs[i]);
	}

	kfree(priv);
}

struct niser_port_set_features_cfg_cmd {
	u8	cmd_enable_disable_features;	/* NISER_CMD_ENABLE_FEATURES */
	u8	features;
} __packed;

static int niser_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct niser_port_set_features_cfg_cmd open_cfg = {};
	struct usb_device *dev;
	int status = 0;

	if (!priv)
		return -ENODEV;

	dev = port->serial->dev;

	if (tty)
		niser_set_termios(tty, port, &tty->termios);

	open_cfg.cmd_enable_disable_features = NISER_CMD_ENABLE_FEATURES;
	open_cfg.features = (NISER_FEATURE_LEDS | NISER_FEATURE_RX |
			     NISER_FEATURE_MODEM);
	status = niser_send_cmd(port, &open_cfg, sizeof(open_cfg));
	if (status < 0)
		return status;

	/* Submit the read URB that the core set up for us */
	status = usb_serial_generic_submit_read_urbs(port, GFP_KERNEL);

	return status;
}

static void niser_close(struct usb_serial_port *port)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct niser_port_set_features_cfg_cmd close_cfg = {};
	struct usb_device *dev;

	if (!priv)
		return;

	dev = port->serial->dev;

	close_cfg.cmd_enable_disable_features = NISER_CMD_DISABLE_FEATURES;
	close_cfg.features = (NISER_FEATURE_LEDS | NISER_FEATURE_RX |
			      NISER_FEATURE_MODEM);
	niser_send_cmd(port, &close_cfg, sizeof(close_cfg));

	usb_kill_urb(port->read_urb);
}

static int niser_calc_num_ports(struct usb_serial *serial,
				struct usb_serial_endpoints *epds)
{
	u8 ifnum;
	u8 numendpoints;

	ifnum = serial->interface->cur_altsetting->desc.bInterfaceNumber;
	numendpoints = serial->interface->cur_altsetting->desc.bNumEndpoints;

	/*
	 * We have to do a bit of rearrangement here. The USB Serial core
	 * found a bunch of bulk endpoint pairs, but this device uses extra
	 * bulk endpoint pairs for device-wide and port command/status
	 * messages.
	 */

	if ((ifnum == 0 && numendpoints == 6) ||
	    (ifnum == 1 && numendpoints == 4)) {
		const int last_ep_pair = (numendpoints / 2) - 1;

		/*
		 * The last pair of endpoints in the interface are the tx/rx
		 * endpoints. Swap it to be the first one.
		 */
		epds->num_bulk_in = 1;
		epds->num_bulk_out = 1;
		swap(epds->bulk_in[0], epds->bulk_in[last_ep_pair]);
		swap(epds->bulk_out[0], epds->bulk_out[last_ep_pair]);

		return 1;
	}

	dev_err(&serial->dev->dev,
		"%s - unknown endpoint layout for product 0x%4X\n",
		__func__,
		le16_to_cpu(serial->dev->descriptor.idProduct));
	return -ENODEV;
}

/*
 * This structure is actually a stream of commands to be sent in
 * one big transaction.  "0x00" is a null command with no parameters,
 * so leaving commands empty is fine.
 */
struct niser_termios_cfg_cmd {
	u8	cmd_set_charsize;	/* NISER_CMD_SET_CHARSIZE(u8) */
	u8	charsize;

	u8	cmd_enable_parity;	/* NISER_CMD_ENABLE_PARITY(u8) */
	u8	parity;

	u8	cmd_set_parity;		/* NISER_CMD_SET_PARITY(u8) */
	u8	parity_mode;

	u8	cmd_set_parity_forced;	/* NISER_CMD_SET_FORCED_PARITY(u8) */
	u8	parity_force_mode;

	u8	cmd_set_stopbits;	/* NISER_CMD_SET_STOPBITS(u8) */
	u8	stopbits;

	u8	cmd_set_or_clr_ctsfc;	/* NISER_CMD_{SET|CLR}_CTSFC() */
	u8	cmd_set_or_clr_rtsfc;	/* NISER_CMD_{SET|CLR}_RTSFC() */

	u8	cmd_set_or_clr_rxof;	/* NISER_CMD_{SET|CLR}_RXOF() */
	u8	cmd_set_or_clr_txof;	/* NISER_CMD_{SET|CLR}_TXOF() */
	u8	cmd_set_or_clr_txoa;	/* NISER_CMD_{SET|CLR}_TXOA() */

	u8	cmd_set_xonchar;	/* NISER_CMD_SET_XONCHAR(u8) */
	u8	xonchar;

	u8	cmd_set_xoffchar;	/* NISER_CMD_SET_XOFFCHAR(u8) */
	u8	xoffchar;

	u8	cmd_set_baudrate;	/* NISER_CMD_SET_BAUDRATE(u32) */
	__be32	baudrate;
} __packed;

static void niser_set_termios(struct tty_struct *tty,
			      struct usb_serial_port *port,
			      struct ktermios *old_termios)
{
	struct usb_device *dev = port->serial->dev;
	struct niser_termios_cfg_cmd cfg = {};
	int status;

	/* set the requested char size */
	cfg.cmd_set_charsize = NISER_CMD_SET_CHARSIZE;
	switch (C_CSIZE(tty)) {
	case CS5:
		cfg.charsize = NISER_CHARSIZE_5BITS;
		break;
	case CS6:
		cfg.charsize = NISER_CHARSIZE_6BITS;
		break;
	case CS7:
		cfg.charsize = NISER_CHARSIZE_7BITS;
		break;
	default:
	case CS8:
		cfg.charsize = NISER_CHARSIZE_8BITS;
		break;
	}

	/* determine the parity */
	cfg.cmd_enable_parity = NISER_CMD_ENABLE_PARITY;
	if (C_PARENB(tty)) {
		cfg.cmd_set_parity = NISER_CMD_SET_PARITY;
		cfg.parity = NISER_PARITY;
		if (C_PARODD(tty)) {
			/* odd parity (or mark, with CMSPAR) */
			cfg.parity_mode = NISER_PARITY_ODD;
		} else {
			/* even parity (or space, with CMSPAR) */
			cfg.parity_mode = NISER_PARITY_EVEN;
		}

		cfg.cmd_set_parity_forced = NISER_CMD_SET_PARITY_FORCED;
		if (C_CMSPAR(tty)) {
			/* mark/space parity */
			cfg.parity_force_mode = NISER_PARITY_FORCED;
		} else {
			/* odd/even parity */
			cfg.parity_force_mode = NISER_PARITY_NOT_FORCED;
		}
	} else {
		/* no parity */
		cfg.parity = NISER_NOPARITY;
	}

	/* figure out the stop bits requested */
	cfg.cmd_set_stopbits = NISER_CMD_SET_STOPBITS;
	if (C_CSTOPB(tty))
		cfg.stopbits = NISER_STOPBITS_15_2;
	else
		cfg.stopbits = NISER_STOPBITS_1;

	/* figure out hardware flow control settings */
	if (C_CRTSCTS(tty)) {
		cfg.cmd_set_or_clr_ctsfc = NISER_CMD_SET_CTSFC;
		cfg.cmd_set_or_clr_rtsfc = NISER_CMD_SET_RTSFC;
	} else {
		cfg.cmd_set_or_clr_ctsfc = NISER_CMD_CLR_CTSFC;
		cfg.cmd_set_or_clr_rtsfc = NISER_CMD_CLR_RTSFC;
	}

	if (I_IXOFF(tty) || I_IXON(tty) || I_IXANY(tty)) {
		cfg.cmd_set_xonchar = NISER_CMD_SET_XONCHAR;
		cfg.xonchar = START_CHAR(tty);
		cfg.cmd_set_xoffchar = NISER_CMD_SET_XOFFCHAR;
		cfg.xoffchar = STOP_CHAR(tty);

		/* inbound XON/XOFF */
		if (I_IXOFF(tty))
			cfg.cmd_set_or_clr_rxof = NISER_CMD_SET_RXOF;
		else
			cfg.cmd_set_or_clr_rxof = NISER_CMD_CLR_RXOF;

		/* outbound XON/XOFF */
		if (I_IXOFF(tty))
			cfg.cmd_set_or_clr_txof = NISER_CMD_SET_TXOF;
		else
			cfg.cmd_set_or_clr_txof = NISER_CMD_CLR_TXOF;

		/* XON-on-any */
		if (I_IXANY(tty))
			cfg.cmd_set_or_clr_txoa = NISER_CMD_SET_TXOA;
		else
			cfg.cmd_set_or_clr_txoa = NISER_CMD_CLR_TXOA;
	}

	/* get the desired baud rate */
	cfg.cmd_set_baudrate = NISER_CMD_SET_BAUDRATE;
	cfg.baudrate = cpu_to_be32(tty_get_baud_rate(tty));

	status = niser_send_cmd(port, &cfg, sizeof(cfg));

	if (status < 0) {
		dev_err(&dev->dev, "%s: usb_bulk_msg returned %d\n",
			__func__, status);
	}
}


static int niser_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;
	unsigned int result = 0;
	u8 msr;
	int mcr;

	spin_lock_irqsave(&priv->port_lock, flags);
	msr = priv->msr;
	mcr = priv->tiocm_mcr;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	if (msr & NISER_MSR_CTSON)
		result |= TIOCM_CTS;
	if (msr & NISER_MSR_DSRON)
		result |= TIOCM_DSR;
	if (msr & NISER_MSR_RION)
		result |= TIOCM_RI;
	if (msr & NISER_MSR_DCDON)
		result |= TIOCM_CD;

	/* This can add DTR, RTS, OUT1, OUT2, and LOOP */
	result |= mcr;

	return result;
}

/*
 * This is a stream of commands for changing modem state.
 */
struct niser_tiocmset_cfg_cmd {
	u8	cmd_set_or_clr_dtr;
	u8	cmd_set_or_clr_rts;
	u8	cmd_set_or_clr_dcd;
	u8	cmd_set_or_clr_ri;
	u8	cmd_set_or_clr_loopback;
} __packed;

static int niser_tiocmset(struct tty_struct *tty, unsigned int set,
			  unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct niser_tiocmset_cfg_cmd cfg = {};
	unsigned long flags;
	int status;

	spin_lock_irqsave(&priv->port_lock, flags);
	priv->tiocm_mcr |= set;
	priv->tiocm_mcr &= ~clear;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	if (set & TIOCM_DTR)
		cfg.cmd_set_or_clr_dtr = NISER_CMD_SET_DTR;
	if (set & TIOCM_RTS)
		cfg.cmd_set_or_clr_rts = NISER_CMD_SET_RTS;
	if (set & TIOCM_LOOP)
		cfg.cmd_set_or_clr_loopback = NISER_CMD_SET_LOOPBACK;
	if (set & TIOCM_OUT1)
		cfg.cmd_set_or_clr_ri = NISER_CMD_SET_RI;
	if (set & TIOCM_OUT2)
		cfg.cmd_set_or_clr_dcd = NISER_CMD_SET_DCD;

	if (clear & TIOCM_DTR)
		cfg.cmd_set_or_clr_dtr = NISER_CMD_CLR_DTR;
	if (clear & TIOCM_RTS)
		cfg.cmd_set_or_clr_rts = NISER_CMD_CLR_RTS;
	if (clear & TIOCM_LOOP)
		cfg.cmd_set_or_clr_loopback = NISER_CMD_CLR_LOOPBACK;
	if (clear & TIOCM_OUT1)
		cfg.cmd_set_or_clr_ri = NISER_CMD_CLR_RI;
	if (clear & TIOCM_OUT2)
		cfg.cmd_set_or_clr_dcd = NISER_CMD_CLR_DCD;

	status = niser_send_cmd(port, &cfg, sizeof(cfg));

	return status;
}

static int niser_tiocgserial(struct usb_serial_port *port,
			     struct serial_struct __user *user_serial)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct serial_struct serial;
	unsigned int cwait;

	cwait = port->port.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = jiffies_to_msecs(cwait) / 10;

	memset(&serial, 0x00, sizeof(serial));

	serial.type = PORT_16550A;
	serial.line = port->minor;
	serial.port = port->port_number;
	serial.xmit_fifo_size = kfifo_size(&port->write_fifo);
	serial.baud_base = priv->max_baudrate;
	serial.closing_wait = cwait;

	if (copy_to_user(user_serial, &serial, sizeof(serial)))
		return -EFAULT;
	return 0;
}

static int niser_tiocsserial(struct usb_serial_port *port,
			     struct serial_struct __user *user_serial)
{
	struct serial_struct serial;
	unsigned int cwait;

	if (copy_to_user(&serial, user_serial, sizeof(serial)))
		return -EFAULT;

	cwait = serial.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = msecs_to_jiffies(10 * serial.closing_wait);

	port->port.closing_wait = cwait;

	return 0;
}

static int niser_tiocgrs485(struct usb_serial_port *port,
			    struct serial_rs485 __user *user_rs485)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct serial_rs485 rs485;
	unsigned long flags;

	if (!priv->is_485)
		return -ENOIOCTLCMD;

	memset(&rs485, 0x00, sizeof(rs485));

	spin_lock_irqsave(&priv->port_lock, flags);
	rs485 = priv->rs485;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	if (copy_to_user(user_rs485, &rs485, sizeof(rs485)))
		return -EFAULT;

	return 0;
}

/*
 * This is a stream of commands for changing modem state.
 */
struct niser_tiocsrs485_cfg_cmd {
	u8	cmd_set_wiremode;
	u8	wiremode;
	u8	cmd_set_or_clr_485bias;
} __packed;

static int niser_tiocsrs485(struct usb_serial_port *port,
			    struct serial_rs485 __user *user_rs485)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct usb_device *dev = port->serial->dev;
	struct niser_tiocsrs485_cfg_cmd cfg = {};
	struct serial_rs485 rs485;
	unsigned long flags;
	int status;

	if (!priv->is_485)
		return -ENOIOCTLCMD;

	if (copy_from_user(&rs485, user_rs485, sizeof(rs485)))
		return -EFAULT;

	cfg.cmd_set_wiremode = NISER_CMD_SET_WIREMODE;
	if (rs485.flags & SER_RS485_ENABLED) {
		/* RS-485 */
		if ((rs485.flags & SER_RS485_RX_DURING_TX) &&
		    (rs485.flags & SER_RS485_RTS_ON_SEND)) {
			dev_dbg(&dev->dev, "%s: Invalid 2-wire mode\n",
				__func__);
			status = -EINVAL;
			goto end;
		}

		if (rs485.flags & SER_RS485_RX_DURING_TX) {
			/* 2-wire with echo */
			cfg.wiremode = NISER_WIREMODE_ECHO_RS485;
		} else {
			if (rs485.flags & SER_RS485_RTS_ON_SEND) {
				/* 2-wire Auto */
				cfg.wiremode = NISER_WIREMODE_AUTO_RS485;
			} else {
				/* 2-wire DTR no echo */
				cfg.wiremode = NISER_WIREMODE_DTR_RS485;
			}
		}
	} else {
		/* RS-422, 4-wire */
		cfg.wiremode = NISER_WIREMODE_RS422;
	}

	if (rs485.flags & SER_RS485_TERMINATE_BUS)
		cfg.cmd_set_or_clr_485bias = NISER_CMD_SET_485BIAS;
	else
		cfg.cmd_set_or_clr_485bias = NISER_CMD_CLR_485BIAS;

	spin_lock_irqsave(&priv->port_lock, flags);
	priv->rs485 = rs485;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	status = niser_send_cmd(port, &cfg, sizeof(cfg));

	if (copy_to_user(user_rs485, &priv->rs485, sizeof(priv->rs485)))
		status = -EFAULT;

end:
	return status;
}

static int niser_ioctl(struct tty_struct *tty, unsigned int cmd,
		       unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;

	switch (cmd) {
	case TIOCGSERIAL:
		return niser_tiocgserial(port,
					 (struct serial_struct __user *)arg);
	case TIOCSSERIAL:
		return niser_tiocsserial(port,
					 (struct serial_struct __user *)arg);
	case TIOCGRS485:
		return niser_tiocgrs485(port,
					(struct serial_rs485 __user *)arg);
	case TIOCSRS485:
		return niser_tiocsrs485(port,
					(struct serial_rs485 __user *)arg);
	}

	return -ENOIOCTLCMD;
}

static void niser_break(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	u8 cmd_break;

	cmd_break = (break_state ? NISER_CMD_SET_BREAK : NISER_CMD_CLR_BREAK);

	niser_send_cmd(port, &cmd_break, sizeof(cmd_break));
}

static void niser_update_msr(struct usb_serial_port *port, u8 new_msr)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	struct tty_struct *tty;
	unsigned long flags;
	u8 delta;

	spin_lock_irqsave(&priv->port_lock, flags);
	delta = new_msr ^ priv->msr;
	priv->msr = new_msr;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	if (!delta)
		return;

	if (delta & NISER_MSR_CTSON)
		port->icount.cts++;
	if (delta & NISER_MSR_DSRON)
		port->icount.dsr++;
	if (delta & NISER_MSR_RION)
		port->icount.rng++;
	if (delta & NISER_MSR_DCDON) {
		port->icount.dcd++;
		tty = tty_port_tty_get(&port->port);
		if (tty) {
			usb_serial_handle_dcd_change(port, tty,
						     new_msr & NISER_MSR_DCDON);
			tty_kref_put(tty);
		}
	}

	wake_up_interruptible(&port->port.delta_msr_wait);
}

static void niser_update_lsr(struct usb_serial_port *port, u8 lsr)
{
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;

	spin_lock_irqsave(&priv->port_lock, flags);
	priv->lsr |= lsr;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	if (lsr & NISER_LSR_RXOVERRUN)
		port->icount.overrun++;
	if (lsr & NISER_LSR_RXPARITY)
		port->icount.parity++;
	if (lsr & NISER_LSR_RXFRAMING)
		port->icount.frame++;
	if (lsr & NISER_LSR_RXBREAK)
		port->icount.brk++;
}

/* Return the length of this message (including the identifier byte) */
static int niser_msg_length(u8 cmd)
{
	switch (cmd) {
	case NISER_STS_TX_DATA_DONE:
		return 1; /* id byte, no payload */
	case NISER_STS_BAD_COMMAND:
	case NISER_STS_WIREMODE:
	case NISER_STS_MSR:
	case NISER_STS_LSR:
	case NISER_STS_FEATURES:
		return 2; /* id byte, u8 payload */
	case NISER_STS_BAUDRATE:
		return 5; /* id byte, u32 payload */
	default:
		return 0;
	}
}

static int niser_port_handle_status(struct usb_serial_port *port,
				    u8 *buffer, int remaining)
{
	struct device *dev = &port->serial->dev->dev;
	u8 cmd = buffer[0];
	int msglen = niser_msg_length(cmd);

	/* make sure that the status makes sense */
	if (msglen == 0) {
		dev_dbg(dev, "%s: unknown status msg 0x%02x\n", __func__, cmd);
		return -EINVAL;
	} else if (msglen > remaining) {
		dev_dbg(dev, "%s: cmd 0x%02x had parameters truncated!\n",
			__func__, cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case NISER_STS_TX_DATA_DONE:
		port->icount.tx++;
		break;

	case NISER_STS_BAD_COMMAND:
		dev_err(dev, "%s: programmer error: %02x was sent, is bad cmd\n",
			__func__, buffer[1]);
		break;

	case NISER_STS_MSR:
		niser_update_msr(port, buffer[1]);
		break;

	case NISER_STS_LSR:
		niser_update_lsr(port, buffer[1]);
		break;

	case NISER_STS_WIREMODE:
	case NISER_STS_FEATURES:
	case NISER_STS_BAUDRATE:
		/*
		 * We don't make requests that generate these responses, so
		 * don't do anything.
		 */
		break;

	default:
		dev_err(dev, "%s: unhandled status msg 0x%02x\n",
			__func__, cmd);
		return -EINVAL;
	}

	return msglen;
}

static void niser_port_sts_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct device *dev = &urb->dev->dev;
	u8 *data = urb->transfer_buffer;
	int status = urb->status;

	switch (status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dev_dbg(dev, "%s: urb shutting down, %d\n", __func__, status);
		return;
	case -EPIPE:
		dev_dbg(dev, "%s: urb stopped, %d\n", __func__, status);
		return;
	default:
		dev_dbg(dev, "%s: nonzero urb status, %d\n", __func__, status);
		goto resubmit;
	}

	/*
	 * If we have too many status messages to fit into a single packet,
	 * the last byte is replaced with 0xFF and data is lost.
	 */
	if (urb->actual_length == 64 && data[63] == 0xFF) {
		dev_err(dev, "%s: status overrun occurred!\n", __func__);
		urb->actual_length--;
	}

	if (urb->actual_length) {
		int offset = 0;

		while (offset < urb->actual_length) {
			int remaining = urb->actual_length - offset;
			int used;

			used = niser_port_handle_status(port,
							&data[offset],
							remaining);

			if (used <= 0) {
				/*
				 * message parse error; this is non-fatal
				 * with regard to the fate of this URB.
				 */
				break;
			}

			offset += used;
		}
	}

resubmit:
	niser_port_submit_status_urb(port, urb, GFP_ATOMIC);
}

static void niser_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct niser_port_private *priv = usb_get_serial_port_data(port);
	unsigned char *data = urb->transfer_buffer;
	char tty_flag = TTY_NORMAL;
	unsigned long flags;
	u8 line_status;
	int i;

	/* update line status */
	spin_lock_irqsave(&priv->port_lock, flags);
	line_status = priv->lsr;
	priv->lsr = 0;
	spin_unlock_irqrestore(&priv->port_lock, flags);

	if (!urb->actual_length)
		return;

	if (line_status & NISER_LSR_RXBREAK)
		tty_flag = TTY_BREAK;
	else if (line_status & NISER_LSR_RXPARITY)
		tty_flag = TTY_PARITY;
	else if (line_status & NISER_LSR_RXFRAMING)
		tty_flag = TTY_FRAME;

	if (tty_flag != TTY_NORMAL)
		dev_info(&port->dev, "%s: tty_flag = %d\n", __func__,
			 tty_flag);

	if (line_status & NISER_LSR_RXOVERRUN)
		tty_insert_flip_char(&port->port, 0, TTY_OVERRUN);

	if (port->port.console && port->sysrq) {
		for (i = 0; i < urb->actual_length; ++i) {
			if (!usb_serial_handle_sysrq_char(port, data[i])) {
				tty_insert_flip_char(&port->port, data[i],
						     tty_flag);
			}
		}
	} else {
		tty_insert_flip_string_fixed_flag(&port->port, data, tty_flag,
						  urb->actual_length);
	}

	tty_flip_buffer_push(&port->port);
}

/*
 * For firmware loading, we use a vendor request to read the EEPROM to get
 * the real product ID. After we determine this, transfer the firmware image
 * to the device using the EP1 out endpoint.
 */
static int niser_loader_get_pid(struct usb_serial *serial)
{
	u8 *pidbuf;
	int i;
	int ret;

	pidbuf = kzalloc(sizeof(u16), GFP_KERNEL);
	if (!pidbuf)
		return -ENOMEM;

	/* Read two bytes from the EEPROM, one at a time. */
	for (i = 0; i < sizeof(u16); ++i) {
		u16 offset = EEPROM_PID_HIGH + i;

		ret = usb_control_msg(
			serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			LOADER_CMD_READ_EEPROM,
			(USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN),
			0, offset, &pidbuf[i], 1, USB_CTRL_GET_TIMEOUT);

		/* expect a one-byte result */
		if (ret < 0) {
			dev_err(&serial->dev->dev,
				"%s: usb_control_msg returned %d\n",
				__func__, ret);
			goto end;
		}
		if (ret != 1) {
			ret = -EIO;
			goto end;
		}
	}

	ret = (pidbuf[0] << 8) | (pidbuf[1]);

end:
	kfree(pidbuf);
	return ret;
}

static int niser_loader_download(struct usb_serial *serial,
				 const struct firmware *fw)
{
	const int fw_size = fw->size;
	int pos;
	int len;
	int sent = 0;
	int ret = 0;
	u8 *fw_buf;

	fw_buf = kzalloc(MAX_FWLOAD_PACKET_SIZE, GFP_KERNEL);
	if (!fw_buf)
		return -ENOMEM;

	for (pos = 0; pos < fw_size; pos += sent) {
		len = min(fw_size - pos, MAX_FWLOAD_PACKET_SIZE);

		memcpy(fw_buf, &fw->data[pos], len);

		ret = usb_bulk_msg(serial->dev,
				   usb_sndbulkpipe(serial->dev, 0x01),
				   fw_buf, len, &sent, 1000);
		if (ret)
			break;
	}

	kfree(fw_buf);
	return ret;
}

static int niser_loader_probe(struct usb_serial *serial,
			      const struct usb_device_id *id)
{
	int ret = -ENOENT;
	int pid = 0;
	const struct firmware *fw;
	const char *fw_name = NULL;


	/* Get the real product ID so we know which firmware to load. */
	pid = niser_loader_get_pid(serial);
	if (pid < 0) {
		dev_err(&serial->dev->dev,
			"%s: Failed to get product ID, err %d\n",
			__func__, pid);
		return pid;
	}

	switch (pid) {
	case NI_USB232x1_ID:
		fw_name = "ni/serial/ni_usb232.fw";
		break;
	case NI_USB232x2_ID:
	case NI_USB232x4_12_ID:
	case NI_USB232x4_34_ID:
		fw_name = "ni/serial/ni_usb232_multi.fw";
		break;
	case NI_USB485x1_ID:
		fw_name = "ni/serial/ni_usb485.fw";
		break;
	case NI_USB485x2_ID:
	case NI_USB485x4_12_ID:
	case NI_USB485x4_34_ID:
		fw_name = "ni/serial/ni_usb485_multi.fw";
		break;
	case NI_EXPRESSCARD8420_ID:
		fw_name = "ni/serial/ni_expresscard232.fw";
		break;
	case NI_EXPRESSCARD8421_ID:
		fw_name = "ni/serial/ni_expresscard485.fw";
		break;
	default:
		dev_err(&serial->dev->dev,
			"%s: Unknown firmware for dev 0x%04x\n",
			__func__, pid);
		return -ENOENT;
	}

	ret = request_firmware(&fw, fw_name, &serial->dev->dev);
	if (ret) {
		dev_err(&serial->dev->dev,
			"%s: Failed to find image \"%s\" err %d\n",
			__func__, fw_name, ret);
		goto release_fw;
	}

	if (fw->size > MAX_FIRMWARE_SIZE) {
		dev_err(&serial->dev->dev,
			"%s: Firmware \"%s\" too large\n",
			__func__, fw_name);
		ret = -ENOENT;
		goto release_fw;
	}

	ret = niser_loader_download(serial, fw);
	if (ret) {
		dev_err(&serial->dev->dev,
			"%s: Failed to download image \"%s\" err %d\n",
			__func__, fw_name, ret);
		goto release_fw;
	}

	dev_dbg(&serial->dev->dev, "%s: Download of \"%s\" successful.\n",
		__func__, fw_name);
	/* device about to reset */
	ret = -ENODEV;

release_fw:
	release_firmware(fw);
	return ret;
}

/*
 * For the loader device, the probe function downloads the firmware. The
 * hardware automatically reenumerates upon download completion.
 */
static int niser_probe(struct usb_serial *serial,
		       const struct usb_device_id *id)
{
	/* Only upload firmware if this is the loader device PID. */
	if (id->idVendor == NI_VENDOR_ID && id->idProduct == NI_PRELOADER_ID)
		return niser_loader_probe(serial, id);

	return 0;
}

static const struct usb_device_id ni_usbserial_ids[] = {
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB232x1_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB232x2_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB232x4_12_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB232x4_34_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB485x1_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB485x2_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB485x4_12_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_USB485x4_34_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_EXPRESSCARD8420_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_EXPRESSCARD8421_ID) },
	{ USB_DEVICE(NI_VENDOR_ID, NI_PRELOADER_ID) },
	{ }
};

static struct usb_serial_driver ni_usbserial_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"ni_usbserial",
	},
	.description =		"NI USB-232/485",
	.id_table =		ni_usbserial_ids,
	.probe =		niser_probe,
	.calc_num_ports =	niser_calc_num_ports,
	.port_probe =		niser_port_probe,
	.port_remove =		niser_port_remove,
	.open =			niser_open,
	.close =		niser_close,
	.ioctl =		niser_ioctl,
	.set_termios =		niser_set_termios,
	.break_ctl =		niser_break,
	.tiocmget =		niser_tiocmget,
	.tiocmset =		niser_tiocmset,
	.process_read_urb =	niser_process_read_urb,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ni_usbserial_device,
	NULL
};

MODULE_DEVICE_TABLE(usb, ni_usbserial_ids);
module_usb_serial_driver(serial_drivers, ni_usbserial_ids);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("ni/serial/ni_usb232.fw");
MODULE_FIRMWARE("ni/serial/ni_usb232_multi.fw");
MODULE_FIRMWARE("ni/serial/ni_usb485.fw");
MODULE_FIRMWARE("ni/serial/ni_usb485_multi.fw");
MODULE_FIRMWARE("ni/serial/ni_expresscard232.fw");
MODULE_FIRMWARE("ni/serial/ni_expresscard485.fw");
