// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-ipmi.c - OBMF-ICP IPMI/SSIF via miscdevice (Channel Type 07h)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 */

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/uaccess.h>

#include "obmf.h"

/*
 * IPMI Optimised Channel payload:
 *   Request:  Command(1) + NetFn/LUN(1) + Payload(N)
 *   Response: CompletionCode(1) + NetFn/LUN(1) + Payload(N)
 */

struct obmf_ipmi_data {
	struct miscdevice	mdev;
	struct obmf_channel	*ch;
	char			name[32];

	/* Response buffer for host-initiated read() */
	u8			resp_buf[256];
	int			resp_len;
	bool			resp_ready;

	/* Device-initiated request buffer */
	u8			dev_req_buf[256];
	int			dev_req_len;
	bool			dev_req_ready;
	bool			dev_resp_pending; /* waiting for userspace response */

	struct mutex		flock;
	wait_queue_head_t	read_wait;
};

static ssize_t obmf_ipmi_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct obmf_ipmi_data *id = file->private_data;
	const u8 *src;
	int src_len;
	bool *flag;
	int rv;

	mutex_lock(&id->flock);

	while (!id->dev_req_ready && !id->resp_ready) {
		mutex_unlock(&id->flock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		rv = wait_event_interruptible(id->read_wait,
					      id->dev_req_ready ||
					      id->resp_ready);
		if (rv)
			return rv;

		mutex_lock(&id->flock);
	}

	/* Prioritise device-initiated requests */
	if (id->dev_req_ready) {
		src     = id->dev_req_buf;
		src_len = id->dev_req_len;
		flag    = &id->dev_req_ready;
	} else {
		src     = id->resp_buf;
		src_len = id->resp_len;
		flag    = &id->resp_ready;
	}

	if (count > src_len)
		count = src_len;

	if (copy_to_user(buf, src, count)) {
		mutex_unlock(&id->flock);
		return -EFAULT;
	}

	*flag = false;
	mutex_unlock(&id->flock);

	return count;
}

static ssize_t obmf_ipmi_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct obmf_ipmi_data *id = file->private_data;
	struct obmf_channel *ch = id->ch;
	struct obmf_device *odev = ch->odev;
	u8 req[256];
	int rv;

	if (count == 0 || count > sizeof(req))
		return -EINVAL;

	if (copy_from_user(req, buf, count))
		return -EFAULT;

	mutex_lock(&id->flock);

	if (id->dev_resp_pending) {
		/*
		 * Userspace is responding to a device-initiated request.
		 * Send as response (Host = Responder).
		 */
		id->dev_resp_pending = false;
		mutex_unlock(&id->flock);

		rv = obmf_send_response(odev, ch->channel_id,
					OBMF_TYPE_IPMI,
					OBMF_STATUS_SUCCESS, req, count);
		return rv < 0 ? rv : count;
	}

	mutex_unlock(&id->flock);

	/* Host-initiated request (existing behavior) */
	mutex_lock(&ch->lock);
	mutex_lock(&id->flock);

	rv = obmf_send_request(odev, ch, OBMF_TYPE_IPMI,
			       req, count,
			       id->resp_buf, sizeof(id->resp_buf),
			       OBMF_DEFAULT_TIMEOUT_MS);

	if (rv > 0) {
		id->resp_len = rv;
		id->resp_ready = true;
		wake_up_interruptible(&id->read_wait);
	}

	mutex_unlock(&id->flock);
	mutex_unlock(&ch->lock);

	return rv < 0 ? rv : count;
}

static __poll_t obmf_ipmi_poll(struct file *file, poll_table *wait)
{
	struct obmf_ipmi_data *id = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &id->read_wait, wait);

	if (id->resp_ready || id->dev_req_ready)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static int obmf_ipmi_open(struct inode *inode, struct file *file)
{
	struct obmf_ipmi_data *id = container_of(file->private_data,
						  struct obmf_ipmi_data, mdev);
	file->private_data = id;
	return 0;
}

static const struct file_operations obmf_ipmi_fops = {
	.owner	= THIS_MODULE,
	.open	= obmf_ipmi_open,
	.read	= obmf_ipmi_read,
	.write	= obmf_ipmi_write,
	.poll	= obmf_ipmi_poll,
};

int obmf_ipmi_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_ipmi_data *id;
	int rv;

	id = kzalloc(sizeof(*id), GFP_KERNEL);
	if (!id)
		return -ENOMEM;

	id->ch = ch;
	mutex_init(&id->flock);
	init_waitqueue_head(&id->read_wait);

	snprintf(id->name, sizeof(id->name), "obmf%d-ipmi-%u",
		 odev->device_index, ch->channel_id);
	id->mdev.minor = MISC_DYNAMIC_MINOR;
	id->mdev.name  = id->name;
	id->mdev.fops  = &obmf_ipmi_fops;

	rv = misc_register(&id->mdev);
	if (rv) {
		kfree(id);
		return rv;
	}

	ch->priv = id;
	ch->sysfs_dev = id->mdev.this_device;
	dev_info(&odev->intf->dev, "ch%u: registered /dev/%s\n",
		 ch->channel_id, id->name);
	return 0;
}

void obmf_ipmi_handle_dev_request(struct obmf_channel *ch,
				  const u8 *data, int len)
{
	struct obmf_ipmi_data *id = ch->priv;
	struct obmf_device *odev = ch->odev;

	if (!id) {
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_IPMI,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	mutex_lock(&id->flock);
	id->dev_req_len = min_t(int, len, (int)sizeof(id->dev_req_buf));
	if (id->dev_req_len > 0)
		memcpy(id->dev_req_buf, data, id->dev_req_len);
	id->dev_req_ready = true;
	id->dev_resp_pending = true;
	wake_up_interruptible(&id->read_wait);
	mutex_unlock(&id->flock);
}

void obmf_ipmi_unregister(struct obmf_channel *ch)
{
	struct obmf_ipmi_data *id = ch->priv;

	if (id) {
		misc_deregister(&id->mdev);
		kfree(id);
		ch->priv = NULL;
	}
}
