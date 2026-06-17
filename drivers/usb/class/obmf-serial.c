// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-serial.c - OBMF-ICP Serial/UART via standalone tty_driver (Type 03h)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Cannot use drivers/usb/serial/ framework because OBMF owns the USB
 * interface — must use standalone tty_driver + tty_port.
 */

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/unaligned.h>

#include "obmf.h"

/*
 * Serial Optimised Channel payload (v0.9):
 *   Request:  Operation/Event(1) + CharCount(2, u16 LE) + CharData(N)
 *   Response: Status in header; payload: ACK/NACK(1) + AcceptedCharCount(2, u16 LE)
 */

#define OBMF_SERIAL_MAX_PORTS	4

struct obmf_serial_port {
	struct obmf_channel	*ch;
	struct tty_port		port;
	int			index;
};

/* ------------------------------------------------------------------ */
/* tty_port_operations                                                  */
/* ------------------------------------------------------------------ */

static int obmf_serial_port_activate(struct tty_port *port,
				     struct tty_struct *tty)
{
	return 0;
}

static void obmf_serial_port_shutdown(struct tty_port *port)
{
}

/*
 * Called by the tty layer when the port's last reference is dropped (i.e.
 * after both the driver has released its reference and any open fd has been
 * closed).  Freeing the container here — instead of with a bare kfree() at
 * unregister time — guarantees the tty_port (and its buffer work) stays
 * valid as long as userspace still has the device open.
 */
static void obmf_serial_port_destruct(struct tty_port *port)
{
	struct obmf_serial_port *sp =
		container_of(port, struct obmf_serial_port, port);

	kfree(sp);
}

static const struct tty_port_operations obmf_serial_port_ops = {
	.activate = obmf_serial_port_activate,
	.shutdown = obmf_serial_port_shutdown,
	.destruct = obmf_serial_port_destruct,
};

/* ------------------------------------------------------------------ */
/* tty_operations                                                       */
/* ------------------------------------------------------------------ */

static int obmf_serial_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void obmf_serial_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(tty->port, tty, filp);
}

static ssize_t obmf_serial_write(struct tty_struct *tty, const u8 *buf,
				 size_t count)
{
	struct obmf_serial_port *sp = tty->driver_data;
	struct obmf_channel *ch = sp->ch;
	struct obmf_device *odev = ch->odev;
	u8 req[3 + 512]; /* op/event + charcount(2) + data */
	u8 resp[3];      /* ack/nack(1) + accepted_count(2) */
	int send_len;
	int rv;
	u16 accepted;

	if (count == 0)
		return 0;

	if (count > 512)
		count = 512;

	req[0] = 0; /* Operation/Event = 0 (normal TX) */
	put_unaligned_le16((u16)count, &req[1]);
	memcpy(&req[3], buf, count);
	send_len = 3 + count;

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch, OBMF_TYPE_SERIAL,
			       req, send_len, resp, sizeof(resp),
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);

	if (rv < 0)
		return rv;

	/* Parse ACK/NACK response */
	if (rv >= 3) {
		accepted = get_unaligned_le16(&resp[1]);
		if (resp[0] & OBMF_SERIAL_NACK) {
			/* NACK: return accepted count (may be partial) */
			return accepted > 0 ? accepted : -EAGAIN;
		}
		return accepted > 0 ? accepted : count;
	}

	return count;
}

static unsigned int obmf_serial_write_room(struct tty_struct *tty)
{
	return 256;
}

static int obmf_serial_install(struct tty_driver *driver,
			       struct tty_struct *tty)
{
	struct obmf_device *odev = driver->driver_state;
	struct obmf_serial_port *sp;
	int idx = tty->index;
	int rv;

	if (!odev || idx >= odev->num_serial)
		return -ENODEV;

	sp = ((struct obmf_serial_port **)odev->tty_ports)[idx];
	if (!sp)
		return -ENODEV;

	/*
	 * Hold a port reference for the lifetime of this open.  It is
	 * released in obmf_serial_cleanup() when the tty is finally
	 * released.  This keeps the port (and its buffer work) alive even
	 * if the device is unplugged while obmc-console still has it open.
	 */
	tty_port_get(&sp->port);

	tty->driver_data = sp;
	tty->port = &sp->port;

	rv = tty_standard_install(driver, tty);
	if (rv) {
		tty_port_put(&sp->port);
		return rv;
	}

	return 0;
}

static void obmf_serial_cleanup(struct tty_struct *tty)
{
	struct obmf_serial_port *sp = tty->driver_data;

	/* Drop the reference taken in obmf_serial_install(). */
	tty_port_put(&sp->port);
}

static void obmf_serial_set_termios(struct tty_struct *tty,
				    const struct ktermios *old)
{
	/* OBMF serial is a virtual channel — no hardware baud rate to
	 * configure.  Accept any termios setting silently so that tools
	 * like stty work without error. */
}

static const struct tty_operations obmf_serial_ops = {
	.install      = obmf_serial_install,
	.cleanup      = obmf_serial_cleanup,
	.open         = obmf_serial_open,
	.close        = obmf_serial_close,
	.write        = obmf_serial_write,
	.write_room   = obmf_serial_write_room,
	.set_termios  = obmf_serial_set_termios,
};

/* ------------------------------------------------------------------ */
/* RX path — called from transport demux                               */
/* ------------------------------------------------------------------ */

void obmf_serial_rx(struct obmf_channel *ch, const u8 *data, int len)
{
	struct obmf_serial_port *sp = ch->priv;
	u16 char_count;

	if (!sp || len < 3)
		return;

	/* data[0] = operation/event, data[1..2] = char_count (u16 LE) */
	if (data[0] & OBMF_SERIAL_EVT_BREAK_DETECT)
		tty_insert_flip_char(&sp->port, 0, TTY_BREAK);

	char_count = get_unaligned_le16(&data[1]);
	if (char_count > 0 && len >= 3 + char_count) {
		tty_insert_flip_string(&sp->port, &data[3], char_count);
		tty_flip_buffer_push(&sp->port);
	}
}

/* ------------------------------------------------------------------ */
/* Device-initiated request handler (v0.9 — needs ACK/NACK response)   */
/* ------------------------------------------------------------------ */

void obmf_serial_handle_dev_request(struct obmf_channel *ch,
				    const u8 *data, int len)
{
	struct obmf_device *odev = ch->odev;
	struct obmf_serial_port *sp = ch->priv;
	u16 char_count;
	u8 resp[3]; /* ACK/NACK(1) + AcceptedCharCount(2) */
	int accepted = 0;

	if (!sp || len < 3) {
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_SERIAL,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	char_count = get_unaligned_le16(&data[1]);

	/* Handle Operation/Event bits */
	if (data[0] & OBMF_SERIAL_EVT_BREAK_DETECT)
		tty_insert_flip_char(&sp->port, 0, TTY_BREAK);

	/* Insert received characters */
	if (char_count > 0 && len >= 3 + char_count) {
		accepted = tty_insert_flip_string(&sp->port,
						  &data[3], char_count);
		tty_flip_buffer_push(&sp->port);
	}

	/* Build ACK response */
	resp[0] = (accepted == char_count) ? OBMF_SERIAL_ACK : OBMF_SERIAL_NACK;
	put_unaligned_le16((u16)accepted, &resp[1]);

	obmf_send_response(odev, ch->channel_id,
			   OBMF_TYPE_SERIAL,
			   OBMF_STATUS_SUCCESS, resp, sizeof(resp));
}

/* ------------------------------------------------------------------ */
/* Init / exit tty_driver (called once per OBMF device)                */
/* ------------------------------------------------------------------ */

int obmf_serial_init(struct obmf_device *odev)
{
	struct obmf_serial_port **ports;
	struct tty_driver *drv;
	int rv, i;

	/* Pre-count serial channels for tty_alloc_driver() */
	odev->num_serial = 0;
	for (i = 1; i < odev->num_channels; i++) {
		if (odev->channels[i].channel_type == OBMF_TYPE_SERIAL)
			odev->num_serial++;
	}

	if (odev->num_serial == 0)
		return 0;

	/* Pre-allocate the array of port pointers for all serial channels.
	 * Each port is allocated individually in obmf_serial_register() so its
	 * lifetime can be managed by the tty layer's reference count. */
	ports = kcalloc(odev->num_serial, sizeof(*ports), GFP_KERNEL);
	if (!ports)
		return -ENOMEM;
	odev->tty_ports = ports;

	drv = tty_alloc_driver(odev->num_serial,
			       TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(drv)) {
		kfree(ports);
		odev->tty_ports = NULL;
		return PTR_ERR(drv);
	}

	drv->driver_name  = "obmf_serial";
	snprintf(odev->tty_drv_name, sizeof(odev->tty_drv_name),
		 "ttyOBMF%d_", odev->device_index);
	drv->name         = odev->tty_drv_name;
	drv->type         = TTY_DRIVER_TYPE_SERIAL;
	drv->subtype      = SERIAL_TYPE_NORMAL;
	drv->init_termios = tty_std_termios;
	drv->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	drv->init_termios.c_iflag = 0;
	drv->init_termios.c_oflag = ONLCR;
	drv->init_termios.c_lflag = ECHOE | ECHOK | ECHOCTL | ECHOKE;
	drv->init_termios.c_cc[VMIN] = 1;
	drv->init_termios.c_cc[VTIME] = 0;
	drv->driver_state = odev;

	tty_set_operations(drv, &obmf_serial_ops);

	rv = tty_register_driver(drv);
	if (rv) {
		tty_driver_kref_put(drv);
		return rv;
	}

	odev->tty_drv = drv;
	return 0;
}

void obmf_serial_exit(struct obmf_device *odev)
{
	if (odev->tty_drv) {
		tty_unregister_driver(odev->tty_drv);
		tty_driver_kref_put(odev->tty_drv);
		odev->tty_drv = NULL;
	}

	kfree(odev->tty_ports);
	odev->tty_ports = NULL;
}

/* ------------------------------------------------------------------ */
/* Per-channel register / unregister                                   */
/* ------------------------------------------------------------------ */

int obmf_serial_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_serial_port **ports = odev->tty_ports;
	struct obmf_serial_port *sp;
	int idx;
	struct device *dev;

	if (!ports || !odev->tty_drv)
		return -EINVAL;

	/* Find next free slot by scanning ports array */
	for (idx = 0; idx < odev->num_serial; idx++) {
		if (!ports[idx])
			break;
	}
	if (idx >= odev->num_serial)
		return -ENOSPC;

	sp = kzalloc(sizeof(*sp), GFP_KERNEL);
	if (!sp)
		return -ENOMEM;

	sp->ch    = ch;
	sp->index = idx;
	tty_port_init(&sp->port);
	sp->port.ops = &obmf_serial_port_ops;

	ch->priv   = sp;
	ports[idx] = sp;

	dev = tty_port_register_device(&sp->port, odev->tty_drv, idx,
				       &odev->intf->dev);
	if (IS_ERR(dev)) {
		ports[idx] = NULL;
		ch->priv   = NULL;
		/* Drops the initial reference; .destruct frees sp. */
		tty_port_put(&sp->port);
		return PTR_ERR(dev);
	}

	ch->sysfs_dev = dev;

	dev_info(&odev->intf->dev, "ch%u: registered ttyOBMF%d\n",
		 ch->channel_id, idx);
	return 0;
}

void obmf_serial_unregister(struct obmf_channel *ch)
{
	struct obmf_serial_port *sp = ch->priv;
	struct obmf_device *odev;

	if (!sp)
		return;

	odev = sp->ch->odev;
	if (!odev->tty_drv)
		return;

	/*
	 * Hang up any process that still holds the tty open (e.g.
	 * obmc-console) so it observes EOF/HUP, then remove the device node.
	 * The port is freed via .destruct only after the last fd is closed,
	 * so a concurrent poll()/flush on a still-open fd can never touch
	 * freed memory.
	 */
	tty_port_tty_hangup(&sp->port, false);
	tty_port_unregister_device(&sp->port, odev->tty_drv, sp->index);

	((struct obmf_serial_port **)odev->tty_ports)[sp->index] = NULL;
	ch->priv = NULL;
	/* Drops the driver's reference; .destruct frees sp once unused. */
	tty_port_put(&sp->port);
}
