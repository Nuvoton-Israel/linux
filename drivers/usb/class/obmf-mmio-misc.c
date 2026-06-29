// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-mmio.c - OBMF-ICP MMIO sub-type channels via miscdevice
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Provides:
 *   ioctl(OBMF_MMIO_IOC_XFER) — host-initiated MMIO transactions
 *   read()  — receive device-initiated requests
 *   write() — send response to device-initiated requests
 *   poll()  — check for pending device-initiated requests
 *
 * Kernel manages MMIO Sub-Header (Transaction + Tag) in both directions.
 */

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

/* Well-known MMIO addresses */
#define OBMF_MMIO_ADDR_POSTCODE	0x1c000ULL

struct obmf_mmio_data {
	struct miscdevice	mdev;
	struct obmf_channel	*ch;
	char			name[32];

	/* Device-initiated request state */
	u8			dev_req_buf[512];
	int			dev_req_len;
	u8			dev_req_transaction;
	u8			dev_req_tag;
	bool			dev_req_ready;
	bool			dev_resp_pending; /* write() expected */

	struct mutex		flock;
	wait_queue_head_t	read_wait;
};

/* ------------------------------------------------------------------ */
/* ioctl — host-initiated MMIO transaction                             */
/* ------------------------------------------------------------------ */

static long obmf_mmio_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct obmf_mmio_data *md = file->private_data;
	struct obmf_channel *ch = md->ch;
	struct obmf_device *odev = ch->odev;
	struct obmf_mmio_xfer xfer;
	void __user *argp = (void __user *)arg;
	u8 wr_buf[256];
	u8 rd_buf[256];
	void *wr_data = NULL;
	int rv;

	if (cmd != OBMF_MMIO_IOC_XFER)
		return -ENOTTY;

	if (copy_from_user(&xfer, argp, sizeof(xfer)))
		return -EFAULT;

	/* Validate transaction type */
	if (xfer.transaction > OBMF_TRANS_LONG_WRITE)
		return -EINVAL;

	/* Copy write data from userspace if needed */
	if (xfer.transaction == OBMF_TRANS_SHORT_WRITE ||
	    xfer.transaction == OBMF_TRANS_LONG_WRITE) {
		if (xfer.wr_len == 0 || xfer.wr_len > sizeof(wr_buf))
			return -EINVAL;
		if (copy_from_user(wr_buf,
				   u64_to_user_ptr(xfer.wr_data_ptr),
				   xfer.wr_len))
			return -EFAULT;
		wr_data = wr_buf;
	}

	/* Validate read length */
	if (xfer.transaction == OBMF_TRANS_SHORT_READ ||
	    xfer.transaction == OBMF_TRANS_LONG_READ) {
		if (xfer.rd_len == 0 || xfer.rd_len > sizeof(rd_buf))
			return -EINVAL;
	}

	mutex_lock(&ch->lock);
	rv = obmf_send_mmio_request(odev, ch, xfer.transaction, xfer.address,
				    wr_data, xfer.wr_len,
				    rd_buf, xfer.rd_len);
	mutex_unlock(&ch->lock);

	if (rv < 0) {
		xfer.status = OBMF_STATUS_PERMANENT_ERROR;
		if (copy_to_user(argp, &xfer, sizeof(xfer)))
			return -EFAULT;
		return rv;
	}

	/* Copy read data to userspace */
	xfer.status = OBMF_STATUS_SUCCESS;
	if ((xfer.transaction == OBMF_TRANS_SHORT_READ ||
	     xfer.transaction == OBMF_TRANS_LONG_READ) && rv > 0) {
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
/* read — receive device-initiated request                             */
/* ------------------------------------------------------------------ */

static ssize_t obmf_mmio_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct obmf_mmio_data *md = file->private_data;
	struct obmf_channel *ch = md->ch;
	struct obmf_device *odev = ch->odev;
	u8 transaction;
	int rv;

	mutex_lock(&md->flock);

	while (!md->dev_req_ready) {
		mutex_unlock(&md->flock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		rv = wait_event_interruptible(md->read_wait,
					      md->dev_req_ready);
		if (rv)
			return rv;

		mutex_lock(&md->flock);
	}

	/*
	 * dev_req_buf layout (stored by handle_dev_request):
	 *   [addr(4B LE for SHORT, 8B LE for LONG)] [size(1B/2B)] [data...]
	 */
	transaction = md->dev_req_transaction;

	/* ---- Postcode: WRITE to 0x1c000, 1 or 4 byte data ---------------- */
	{
		u64 address = ~0ULL;
		unsigned int data_size = 0;
		const u8 *payload = NULL;

		if (transaction == OBMF_TRANS_SHORT_WRITE &&
		    md->dev_req_len >= (int)(4 + 1 + 1)) {
			/* SHORT: addr32(4) + size_u8(1) + data */
			address   = get_unaligned_le32(md->dev_req_buf);
			data_size = md->dev_req_buf[4];
			payload   = md->dev_req_buf + 5;
		} else if (transaction == OBMF_TRANS_LONG_WRITE &&
			   md->dev_req_len >= (int)(8 + 2 + 1)) {
			/* LONG: addr64(8) + size_u16(2) + data */
			address   = get_unaligned_le64(md->dev_req_buf);
			data_size = get_unaligned_le16(md->dev_req_buf + 8);
			payload   = md->dev_req_buf + 10;
		}

		if (address == OBMF_MMIO_ADDR_POSTCODE) {
			u8 resp[OBMF_MMIO_SUBHDR_SIZE];
			struct obmf_mmio_subhdr *mhdr =
				(struct obmf_mmio_subhdr *)resp;
			int min_len = (int)(payload - md->dev_req_buf) +
				      (int)data_size;

			if ((data_size != 1 && data_size != 4) ||
			    md->dev_req_len < min_len) {
				mutex_unlock(&md->flock);
				return -EIO;
			}
			if (count < data_size) {
				mutex_unlock(&md->flock);
				return -EINVAL;
			}
			if (copy_to_user(buf, payload, data_size)) {
				mutex_unlock(&md->flock);
				return -EFAULT;
			}
			mhdr->transaction = transaction;
			mhdr->tag         = md->dev_req_tag;
			md->dev_req_ready = false;
			mutex_unlock(&md->flock);
			rv = obmf_send_response(odev, ch->channel_id,
						OBMF_TYPE_MMIO, OBMF_STATUS_SUCCESS,
						resp, sizeof(resp));
			return rv ? rv : (ssize_t)data_size;
		}
	}

	/* ---- Other MMIO types: pass full subhdr+payload to userspace ------- */
	if (count < md->dev_req_len) {
		mutex_unlock(&md->flock);
		return -EINVAL;
	}

	if (copy_to_user(buf, md->dev_req_buf, md->dev_req_len)) {
		mutex_unlock(&md->flock);
		return -EFAULT;
	}

	md->dev_req_ready = false;
	md->dev_resp_pending = true;
	mutex_unlock(&md->flock);

	return md->dev_req_len;
}

/* ------------------------------------------------------------------ */
/* write — send response to device-initiated request                   */
/* ------------------------------------------------------------------ */

static ssize_t obmf_mmio_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct obmf_mmio_data *md = file->private_data;
	struct obmf_channel *ch = md->ch;
	struct obmf_device *odev = ch->odev;
	u8 resp[OBMF_MMIO_SUBHDR_SIZE + 512];
	struct obmf_mmio_subhdr *mhdr = (struct obmf_mmio_subhdr *)resp;
	int payload_len;
	int rv;

	if (count == 0 || count > 512)
		return -EINVAL;

	mutex_lock(&md->flock);

	if (!md->dev_resp_pending) {
		mutex_unlock(&md->flock);
		return -EINVAL;
	}

	/* Build MMIO Sub-Header with saved transaction and tag */
	mhdr->transaction = md->dev_req_transaction;
	mhdr->tag         = md->dev_req_tag;

	/* Copy userspace response: [Status(1B)][Data(NB)] */
	if (copy_from_user(resp + OBMF_MMIO_SUBHDR_SIZE, buf, count)) {
		mutex_unlock(&md->flock);
		return -EFAULT;
	}

	/*
	 * v0.9: Status moves to Common Header byte 2[7:1].
	 * Userspace still writes Status(1B) + Data(NB).
	 * Extract status from first byte; payload is sub-header + data only.
	 */
	{
		u8 user_status = resp[OBMF_MMIO_SUBHDR_SIZE];

		/* Shift data left by 1 to remove status byte from payload */
		if (count > 1)
			memmove(resp + OBMF_MMIO_SUBHDR_SIZE,
				resp + OBMF_MMIO_SUBHDR_SIZE + 1, count - 1);

		payload_len = OBMF_MMIO_SUBHDR_SIZE + count - 1;

		md->dev_resp_pending = false;
		mutex_unlock(&md->flock);

		rv = obmf_send_response(odev, ch->channel_id,
					OBMF_TYPE_MMIO, user_status,
					resp, payload_len);
	}
	if (rv)
		return rv;

	return count;
}

/* ------------------------------------------------------------------ */
/* poll                                                                */
/* ------------------------------------------------------------------ */

static __poll_t obmf_mmio_poll(struct file *file, poll_table *wait)
{
	struct obmf_mmio_data *md = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &md->read_wait, wait);

	if (md->dev_req_ready)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

/* ------------------------------------------------------------------ */
/* open                                                                */
/* ------------------------------------------------------------------ */

static int obmf_mmio_open(struct inode *inode, struct file *file)
{
	struct obmf_mmio_data *md = container_of(file->private_data,
						struct obmf_mmio_data, mdev);
	file->private_data = md;
	return 0;
}

static const struct file_operations obmf_mmio_fops = {
	.owner		= THIS_MODULE,
	.open		= obmf_mmio_open,
	.read		= obmf_mmio_read,
	.write		= obmf_mmio_write,
	.poll		= obmf_mmio_poll,
	.unlocked_ioctl	= obmf_mmio_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
};

/* ------------------------------------------------------------------ */
/* register / unregister                                               */
/* ------------------------------------------------------------------ */

int obmf_mmio_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_mmio_data *md;
	int rv;

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	md->ch = ch;
	mutex_init(&md->flock);
	init_waitqueue_head(&md->read_wait);

	snprintf(md->name, sizeof(md->name), "obmf%d-mmio-%u",
		 odev->device_index, ch->channel_id);
	md->mdev.minor = MISC_DYNAMIC_MINOR;
	md->mdev.name  = md->name;
	md->mdev.fops  = &obmf_mmio_fops;

	rv = misc_register(&md->mdev);
	if (rv) {
		kfree(md);
		return rv;
	}

	/* Create sysfs symlink: obmf/channel/<N>/mmio -> misc device */
	if (ch->kobj && md->mdev.this_device)
		sysfs_create_link(ch->kobj, &md->mdev.this_device->kobj, "mmio");

	ch->priv = md;
	dev_info(&odev->intf->dev, "ch%u: registered /dev/%s\n",
		 ch->channel_id, md->name);
	return 0;
}

void obmf_mmio_unregister(struct obmf_channel *ch)
{
	struct obmf_mmio_data *md = ch->priv;

	if (md) {
		sysfs_remove_link(ch->kobj, "mmio");
		misc_deregister(&md->mdev);
		kfree(md);
		ch->priv = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* Device-initiated request handler (called from workqueue)            */
/* ------------------------------------------------------------------ */

void obmf_mmio_handle_dev_request(struct obmf_channel *ch,
				  u8 transaction, u8 tag,
				  const u8 *data, int len)
{
	struct obmf_mmio_data *md = ch->priv;

	if (!md)
		return;

	mutex_lock(&md->flock);
	md->dev_req_transaction = transaction;
	md->dev_req_tag         = tag;
	md->dev_req_len = min_t(int, len, (int)sizeof(md->dev_req_buf));
	if (md->dev_req_len > 0)
		memcpy(md->dev_req_buf, data, md->dev_req_len);

	md->dev_req_ready = true;

	wake_up_interruptible(&md->read_wait);
	mutex_unlock(&md->flock);

	/* Do NOT send response here — read() will respond for known types;
	 * for others, userspace responds via write(). */
}
