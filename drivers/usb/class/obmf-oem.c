// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-oem.c - OBMF-ICP OEM extension channels (F8h-FFh) via miscdevice
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Provides raw read/write access — userspace builds/parses the
 * channel-specific payload directly.
 */

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/uaccess.h>

#include "obmf.h"

struct obmf_oem_data {
	struct miscdevice	mdev;
	struct obmf_channel	*ch;
	char			name[32];

	/* Response buffer for host-initiated read() */
	u8			resp_buf[512];
	int			resp_len;
	bool			resp_ready;

	/* Device-initiated request buffer */
	u8			dev_req_buf[512];
	int			dev_req_len;
	bool			dev_req_ready;
	bool			dev_resp_pending; /* waiting for userspace response */

	struct mutex		flock;
	wait_queue_head_t	read_wait;
};

static ssize_t obmf_oem_read(struct file *file, char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct obmf_oem_data *od = file->private_data;
	const u8 *src;
	int src_len;
	bool *flag;
	int rv;

	mutex_lock(&od->flock);

	while (!od->dev_req_ready && !od->resp_ready) {
		mutex_unlock(&od->flock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		rv = wait_event_interruptible(od->read_wait,
					      od->dev_req_ready ||
					      od->resp_ready);
		if (rv)
			return rv;

		mutex_lock(&od->flock);
	}

	/* Prioritise device-initiated requests */
	if (od->dev_req_ready) {
		src     = od->dev_req_buf;
		src_len = od->dev_req_len;
		flag    = &od->dev_req_ready;
	} else {
		src     = od->resp_buf;
		src_len = od->resp_len;
		flag    = &od->resp_ready;
	}

	if (count > src_len)
		count = src_len;

	if (copy_to_user(buf, src, count)) {
		mutex_unlock(&od->flock);
		return -EFAULT;
	}

	*flag = false;
	mutex_unlock(&od->flock);

	return count;
}

static ssize_t obmf_oem_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct obmf_oem_data *od = file->private_data;
	struct obmf_channel *ch = od->ch;
	struct obmf_device *odev = ch->odev;
	u8 req[512];
	int rv;

	if (count == 0 || count > sizeof(req))
		return -EINVAL;

	if (copy_from_user(req, buf, count))
		return -EFAULT;

	mutex_lock(&od->flock);

	if (od->dev_resp_pending) {
		/*
		 * Userspace is responding to a device-initiated request.
		 * Send as response (Host = Responder).
		 */
		od->dev_resp_pending = false;
		mutex_unlock(&od->flock);

		rv = obmf_send_response(odev, ch->channel_id,
					ch->channel_type,
					OBMF_STATUS_SUCCESS, req, count);
		return rv < 0 ? rv : count;
	}

	mutex_unlock(&od->flock);

	/* Host-initiated request (existing behavior) */
	mutex_lock(&ch->lock);
	mutex_lock(&od->flock);

	rv = obmf_send_request(odev, ch, ch->channel_type,
			       req, count,
			       od->resp_buf, sizeof(od->resp_buf),
			       OBMF_DEFAULT_TIMEOUT_MS);

	if (rv > 0) {
		od->resp_len = rv;
		od->resp_ready = true;
		wake_up_interruptible(&od->read_wait);
	}

	mutex_unlock(&od->flock);
	mutex_unlock(&ch->lock);

	return rv < 0 ? rv : count;
}

static __poll_t obmf_oem_poll(struct file *file, poll_table *wait)
{
	struct obmf_oem_data *od = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &od->read_wait, wait);

	if (od->resp_ready || od->dev_req_ready)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static int obmf_oem_open(struct inode *inode, struct file *file)
{
	struct obmf_oem_data *od = container_of(file->private_data,
						struct obmf_oem_data, mdev);
	file->private_data = od;
	return 0;
}

static const struct file_operations obmf_oem_fops = {
	.owner	= THIS_MODULE,
	.open	= obmf_oem_open,
	.read	= obmf_oem_read,
	.write	= obmf_oem_write,
	.poll	= obmf_oem_poll,
};

int obmf_oem_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_oem_data *od;
	int rv;

	od = kzalloc(sizeof(*od), GFP_KERNEL);
	if (!od)
		return -ENOMEM;

	od->ch = ch;
	mutex_init(&od->flock);
	init_waitqueue_head(&od->read_wait);

	snprintf(od->name, sizeof(od->name), "obmf%d-oem-%u",
		 odev->device_index, ch->channel_id);
	od->mdev.minor = MISC_DYNAMIC_MINOR;
	od->mdev.name  = od->name;
	od->mdev.fops  = &obmf_oem_fops;

	rv = misc_register(&od->mdev);
	if (rv) {
		kfree(od);
		return rv;
	}

	ch->priv = od;
	ch->sysfs_dev = od->mdev.this_device;
	dev_info(&odev->intf->dev, "ch%u: registered /dev/%s\n",
		 ch->channel_id, od->name);
	return 0;
}

void obmf_oem_handle_dev_request(struct obmf_channel *ch,
				 const u8 *data, int len)
{
	struct obmf_oem_data *od = ch->priv;
	struct obmf_device *odev = ch->odev;

	if (!od) {
		obmf_send_response(odev, ch->channel_id,
				   ch->channel_type,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	mutex_lock(&od->flock);
	od->dev_req_len = min_t(int, len, (int)sizeof(od->dev_req_buf));
	if (od->dev_req_len > 0)
		memcpy(od->dev_req_buf, data, od->dev_req_len);
	od->dev_req_ready = true;
	od->dev_resp_pending = true;
	wake_up_interruptible(&od->read_wait);
	mutex_unlock(&od->flock);
}

void obmf_oem_unregister(struct obmf_channel *ch)
{
	struct obmf_oem_data *od = ch->priv;

	if (od) {
		misc_deregister(&od->mdev);
		kfree(od);
		ch->priv = NULL;
	}
}
