// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-io-misc.c - OBMF-ICP I/O Port Channel (type 09h) via miscdevice
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Implements OBMF_TYPE_IO (0x09) as defined in OBMF-ICP v0.9.2 §4.31.
 * This channel carries x86-style I/O port transactions and is the
 * dedicated channel for BIOS POST code reporting via port 0x0080.
 *
 * The MMIO channel (type 01h) retains its own POST code path via MMIO
 * address 0x1c000 for backwards compatibility.  This driver handles the
 * new I/O Port Channel path introduced in v0.9.2.
 *
 * Provides per-channel miscdevice /dev/obmf<N>-io-<CH>:
 *   ioctl(OBMF_IO_IOC_XFER) — host-initiated I/O port transactions
 *   read()  — receive device-initiated requests (POST codes, etc.)
 *   write() — send response to device-initiated requests
 *   poll()  — check for pending device-initiated requests
 *
 * Kernel manages IO Sub-Header (Transaction + Tag) in both directions.
 *
 * POST code fast-path:
 *   When the device writes to port 0x0080 (Sequential or Fixed Write,
 *   8- or 32-bit), the kernel auto-acknowledges the request and exposes
 *   the raw POST code byte(s) via read().
 */

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/unaligned.h>

#include "obmf.h"

/* Well-known I/O port address for BIOS POST codes (standard x86 port 80h) */
#define OBMF_IO_PORT_POSTCODE	0x0080U

struct obmf_io_data {
	struct miscdevice	mdev;
	struct obmf_channel	*ch;
	char			name[32];

	/* Device-initiated request state (protected by flock) */
	u8			dev_req_buf[512];
	int			dev_req_len;
	u8			dev_req_transaction;
	u8			dev_req_tag;
	bool			dev_req_ready;
	bool			dev_resp_pending;	/* write() expected */

	struct mutex		flock;
	wait_queue_head_t	read_wait;
};

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static bool obmf_io_is_write_trans(u8 transaction)
{
	switch (transaction) {
	case OBMF_IO_TRANS_SEQ_WRITE_8:
	case OBMF_IO_TRANS_SEQ_WRITE_16:
	case OBMF_IO_TRANS_SEQ_WRITE_32:
	case OBMF_IO_TRANS_FIXED_WRITE_8:
	case OBMF_IO_TRANS_FIXED_WRITE_16:
	case OBMF_IO_TRANS_FIXED_WRITE_32:
		return true;
	default:
		return false;
	}
}

/* Returns the data-bus width (bytes) for an I/O transaction type */
static int obmf_io_data_width(u8 transaction)
{
	switch (transaction) {
	case OBMF_IO_TRANS_SEQ_READ_16:
	case OBMF_IO_TRANS_SEQ_WRITE_16:
	case OBMF_IO_TRANS_FIXED_READ_16:
	case OBMF_IO_TRANS_FIXED_WRITE_16:
		return 2;
	case OBMF_IO_TRANS_SEQ_READ_32:
	case OBMF_IO_TRANS_SEQ_WRITE_32:
	case OBMF_IO_TRANS_FIXED_READ_32:
	case OBMF_IO_TRANS_FIXED_WRITE_32:
		return 4;
	default:
		return 1;
	}
}

/* ------------------------------------------------------------------ */
/* ioctl — host-initiated I/O port transaction                         */
/* ------------------------------------------------------------------ */

static long obmf_io_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct obmf_io_data *id = file->private_data;
	struct obmf_channel *ch = id->ch;
	struct obmf_device *odev = ch->odev;
	struct obmf_io_xfer xfer;
	void __user *argp = (void __user *)arg;
	u8 wr_buf[256];
	u8 rd_buf[256];
	void *wr_data = NULL;
	int width;
	int rv;

	if (cmd != OBMF_IO_IOC_XFER)
		return -ENOTTY;

	if (copy_from_user(&xfer, argp, sizeof(xfer)))
		return -EFAULT;

	if (xfer.transaction > OBMF_IO_TRANS_FIXED_WRITE_32)
		return -EINVAL;

	width = obmf_io_data_width(xfer.transaction);

	/* Copy write data from userspace */
	if (obmf_io_is_write_trans(xfer.transaction)) {
		if (xfer.wr_len == 0 || xfer.wr_len > sizeof(wr_buf))
			return -EINVAL;
		if (width > 1 && (xfer.wr_len % width) != 0)
			return -EINVAL;
		if (copy_from_user(wr_buf, u64_to_user_ptr(xfer.wr_data_ptr),
				   xfer.wr_len))
			return -EFAULT;
		wr_data = wr_buf;
	} else {
		if (xfer.rd_len == 0 || xfer.rd_len > sizeof(rd_buf))
			return -EINVAL;
		if (width > 1 && (xfer.rd_len % width) != 0)
			return -EINVAL;
	}

	mutex_lock(&ch->lock);
	rv = obmf_send_io_request(odev, ch, xfer.transaction, xfer.port_addr,
				  wr_data, xfer.wr_len,
				  rd_buf, xfer.rd_len);
	mutex_unlock(&ch->lock);

	if (rv < 0) {
		xfer.status = OBMF_STATUS_PERMANENT_ERROR;
		if (copy_to_user(argp, &xfer, sizeof(xfer)))
			return -EFAULT;
		return rv;
	}

	xfer.status = OBMF_STATUS_SUCCESS;
	if (!obmf_io_is_write_trans(xfer.transaction) && rv > 0) {
		if (copy_to_user(u64_to_user_ptr(xfer.rd_data_ptr),
				 rd_buf, rv))
			return -EFAULT;
		xfer.rd_len = rv;
	}

	if (copy_to_user(argp, &xfer, sizeof(xfer)))
		return -EFAULT;

	return 0;
}

/* ------------------------------------------------------------------ */
/* read — receive device-initiated I/O port request                    */
/*                                                                     */
/* dev_req_buf layout (after IO sub-header stripped by transport):     */
/*   [0-1]: port_addr (LE u16)                                        */
/*   [2]:   size (u8, total data bytes)                                */
/*   [3+]:  data bytes                                                 */
/*                                                                     */
/* POST code fast-path: writes to port 0x0080 are auto-ACKed;         */
/* read() returns the raw POST code bytes directly to userspace.       */
/* Other requests are passed through with dev_resp_pending set.        */
/* ------------------------------------------------------------------ */

static ssize_t obmf_io_read(struct file *file, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct obmf_io_data *id = file->private_data;
	struct obmf_channel *ch = id->ch;
	struct obmf_device *odev = ch->odev;
	u8 transaction;
	int rv;

	mutex_lock(&id->flock);

	while (!id->dev_req_ready) {
		mutex_unlock(&id->flock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		rv = wait_event_interruptible(id->read_wait, id->dev_req_ready);
		if (rv)
			return rv;

		mutex_lock(&id->flock);
	}

	transaction = id->dev_req_transaction;

	/*
	 * POST code fast-path: device writes to port 0x0080.
	 * Payload: port_addr(2B LE) + size(1B) + data(size bytes).
	 * The kernel sends an empty-payload success response immediately
	 * and returns just the POST code data to userspace.
	 */
	if (obmf_io_is_write_trans(transaction) && id->dev_req_len >= 3) {
		u16 port_addr = get_unaligned_le16(id->dev_req_buf);
		unsigned int data_size = id->dev_req_buf[2];
		const u8 *payload = id->dev_req_buf + 3;
		int min_len = 3 + (int)data_size;

		if (port_addr == OBMF_IO_PORT_POSTCODE) {
			int width = obmf_io_data_width(transaction);
			u8 resp[OBMF_IO_SUBHDR_SIZE];
			struct obmf_io_subhdr *ihdr =
				(struct obmf_io_subhdr *)resp;

			if (data_size == 0 || (data_size % width) != 0 ||
			    id->dev_req_len < min_len) {
				mutex_unlock(&id->flock);
				return -EIO;
			}

			if (count < data_size) {
				mutex_unlock(&id->flock);
				return -EINVAL;
			}

			if (copy_to_user(buf, payload, data_size)) {
				mutex_unlock(&id->flock);
				return -EFAULT;
			}

			/* Build IO sub-header for response */
			ihdr->transaction = transaction;
			ihdr->tag         = id->dev_req_tag;
			id->dev_req_ready = false;
			mutex_unlock(&id->flock);

			/* ACK: success, IO sub-header only, no data payload */
			rv = obmf_send_response(odev, ch->channel_id,
						OBMF_TYPE_IO,
						OBMF_STATUS_SUCCESS,
						resp, sizeof(resp));
			return rv ? rv : (ssize_t)data_size;
		}
	}

	/* Non-postcode request: pass full payload to userspace.
	 * Userspace must call write() to send the response. */
	if (count < (size_t)id->dev_req_len) {
		mutex_unlock(&id->flock);
		return -EINVAL;
	}

	if (copy_to_user(buf, id->dev_req_buf, id->dev_req_len)) {
		mutex_unlock(&id->flock);
		return -EFAULT;
	}

	id->dev_req_ready = false;
	id->dev_resp_pending = true;
	mutex_unlock(&id->flock);

	return id->dev_req_len;
}

/* ------------------------------------------------------------------ */
/* write — send response to device-initiated request                   */
/*                                                                     */
/* Userspace writes: Status(1B) [Data(NB)]                             */
/* Kernel prepends IO sub-header and calls obmf_send_response().        */
/* ------------------------------------------------------------------ */

static ssize_t obmf_io_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct obmf_io_data *id = file->private_data;
	struct obmf_channel *ch = id->ch;
	struct obmf_device *odev = ch->odev;
	u8 resp[OBMF_IO_SUBHDR_SIZE + 512];
	struct obmf_io_subhdr *ihdr = (struct obmf_io_subhdr *)resp;
	int payload_len;
	int rv;

	if (count == 0 || count > 512)
		return -EINVAL;

	mutex_lock(&id->flock);

	if (!id->dev_resp_pending) {
		mutex_unlock(&id->flock);
		return -EINVAL;
	}

	ihdr->transaction = id->dev_req_transaction;
	ihdr->tag         = id->dev_req_tag;

	/* Copy userspace response: [Status(1B)][Data(NB)] */
	if (copy_from_user(resp + OBMF_IO_SUBHDR_SIZE, buf, count)) {
		mutex_unlock(&id->flock);
		return -EFAULT;
	}

	/*
	 * Status moves to Common Header byte 2[7:1] (v0.9 onwards).
	 * Userspace still writes Status(1B) + optional Data(NB).
	 * Extract status from first byte; payload = sub-header + data only.
	 */
	{
		u8 user_status = resp[OBMF_IO_SUBHDR_SIZE];

		if (count > 1)
			memmove(resp + OBMF_IO_SUBHDR_SIZE,
				resp + OBMF_IO_SUBHDR_SIZE + 1, count - 1);

		payload_len = OBMF_IO_SUBHDR_SIZE + (int)count - 1;

		id->dev_resp_pending = false;
		mutex_unlock(&id->flock);

		rv = obmf_send_response(odev, ch->channel_id,
					OBMF_TYPE_IO, user_status,
					resp, payload_len);
	}

	if (rv)
		return rv;

	return count;
}

/* ------------------------------------------------------------------ */
/* poll                                                                */
/* ------------------------------------------------------------------ */

static __poll_t obmf_io_poll(struct file *file, poll_table *wait)
{
	struct obmf_io_data *id = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &id->read_wait, wait);

	if (id->dev_req_ready)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

/* ------------------------------------------------------------------ */
/* open                                                                */
/* ------------------------------------------------------------------ */

static int obmf_io_open(struct inode *inode, struct file *file)
{
	struct obmf_io_data *id = container_of(file->private_data,
					       struct obmf_io_data, mdev);
	file->private_data = id;
	return 0;
}

static const struct file_operations obmf_io_fops = {
	.owner		= THIS_MODULE,
	.open		= obmf_io_open,
	.read		= obmf_io_read,
	.write		= obmf_io_write,
	.poll		= obmf_io_poll,
	.unlocked_ioctl	= obmf_io_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

/* ------------------------------------------------------------------ */
/* register / unregister                                               */
/* ------------------------------------------------------------------ */

int obmf_io_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_io_data *id;
	int rv;

	id = kzalloc(sizeof(*id), GFP_KERNEL);
	if (!id)
		return -ENOMEM;

	id->ch = ch;
	mutex_init(&id->flock);
	init_waitqueue_head(&id->read_wait);

	snprintf(id->name, sizeof(id->name), "obmf%d-io-%u",
		 odev->device_index, ch->channel_id);
	id->mdev.minor = MISC_DYNAMIC_MINOR;
	id->mdev.name  = id->name;
	id->mdev.fops  = &obmf_io_fops;

	rv = misc_register(&id->mdev);
	if (rv) {
		kfree(id);
		return rv;
	}

	if (ch->kobj && id->mdev.this_device)
		sysfs_create_link(ch->kobj, &id->mdev.this_device->kobj, "io");

	ch->priv = id;
	dev_info(&odev->intf->dev, "ch%u: registered /dev/%s (I/O Port)\n",
		 ch->channel_id, id->name);
	return 0;
}

void obmf_io_unregister(struct obmf_channel *ch)
{
	struct obmf_io_data *id = ch->priv;

	if (id) {
		if (ch->kobj)
			sysfs_remove_link(ch->kobj, "io");
		misc_deregister(&id->mdev);
		kfree(id);
		ch->priv = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* Device-initiated request handler (called from workqueue)            */
/* ------------------------------------------------------------------ */

void obmf_io_handle_dev_request(struct obmf_channel *ch,
				u8 transaction, u8 tag,
				const u8 *data, int len)
{
	struct obmf_io_data *id = ch->priv;

	if (!id)
		return;

	mutex_lock(&id->flock);
	id->dev_req_transaction = transaction;
	id->dev_req_tag         = tag;
	id->dev_req_len = min_t(int, len, (int)sizeof(id->dev_req_buf));
	if (id->dev_req_len > 0)
		memcpy(id->dev_req_buf, data, id->dev_req_len);

	id->dev_req_ready = true;

	wake_up_interruptible(&id->read_wait);
	mutex_unlock(&id->flock);
}
