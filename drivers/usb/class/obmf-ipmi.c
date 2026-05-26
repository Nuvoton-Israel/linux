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
 * IPMI Optimised Channel payload (on the wire):
 *   Request:  Command(1) + NetFn/LUN(1) + IPMI_Cmd(1) + Payload(N)
 *   Response: Command(1) + NetFn/LUN(1) + IPMI_Cmd(1) + CC(1) + Payload(N)
 *
 * Userspace interface (/dev/obmf*-ipmi-*):
 *   read():  [NetFn/LUN, IPMI_Cmd, Data...]      (OBMF Command byte stripped)
 *   write(): [NetFn/LUN, IPMI_Cmd, CC, Data...]   (OBMF Command byte auto-prepended)
 */

struct obmf_ipmi_data {
	struct miscdevice	mdev;
	struct obmf_channel	*ch;
	char			name[32];
	unsigned long		flags;	/* bit 0: exclusive open */
#define OBMF_IPMI_OPEN		0

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
	int rv;

	if (file->f_flags & O_NONBLOCK) {
		if (!id->dev_req_ready)
			return -EAGAIN;
	} else {
		rv = wait_event_interruptible(id->read_wait, id->dev_req_ready);
		if (rv)
			return rv;
	}

	mutex_lock(&id->flock);
	count = min_t(size_t, count, id->dev_req_len);
	if (copy_to_user(buf, id->dev_req_buf, count)) {
		mutex_unlock(&id->flock);
		return -EFAULT;
	}
	id->dev_req_ready = false;
	mutex_unlock(&id->flock);

	return count;
}

static ssize_t obmf_ipmi_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct obmf_ipmi_data *id = file->private_data;
	struct obmf_channel *ch = id->ch;
	struct obmf_device *odev = ch->odev;
	u8 resp[1 + 256]; /* [Command=0x00] + IPMI payload from userspace */
	int rv;

	if (count == 0 || count > sizeof(resp) - 1)
		return -EINVAL;

	mutex_lock(&id->flock);

	if (!id->dev_resp_pending) {
		mutex_unlock(&id->flock);
		return -ENOENT; /* no outstanding request to respond to */
	}

	id->dev_resp_pending = false;
	mutex_unlock(&id->flock);

	/*
	 * Userspace writes pure IPMI: [NetFn/LUN, Cmd, CC, Data...].
	 * Prepend the OBMF Command byte (0x00 = Send IPMI Message) to
	 * form the spec-compliant OBMF IPMI response payload.
	 */
	resp[0] = 0x00; /* OBMF IPMI Command: Send IPMI Message */
	if (copy_from_user(resp + 1, buf, count))
		return -EFAULT;

	rv = obmf_send_response(odev, ch->channel_id,
				OBMF_TYPE_IPMI,
				OBMF_STATUS_SUCCESS, resp, count + 1);
	return rv < 0 ? rv : count;
}

static __poll_t obmf_ipmi_poll(struct file *file, poll_table *wait)
{
	struct obmf_ipmi_data *id = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &id->read_wait, wait);

	if (id->dev_req_ready)
		mask |= EPOLLIN | EPOLLRDNORM;
	if (id->dev_resp_pending)
		mask |= EPOLLOUT | EPOLLWRNORM;

	return mask;
}

static int obmf_ipmi_open(struct inode *inode, struct file *file)
{
	struct obmf_ipmi_data *id = container_of(file->private_data,
						  struct obmf_ipmi_data, mdev);

	if (test_and_set_bit(OBMF_IPMI_OPEN, &id->flags))
		return -EBUSY;

	file->private_data = id;
	return 0;
}

static int obmf_ipmi_release(struct inode *inode, struct file *file)
{
	struct obmf_ipmi_data *id = file->private_data;

	clear_bit(OBMF_IPMI_OPEN, &id->flags);
	return 0;
}

static const struct file_operations obmf_ipmi_fops = {
	.owner		= THIS_MODULE,
	.open		= obmf_ipmi_open,
	.release	= obmf_ipmi_release,
	.read		= obmf_ipmi_read,
	.write		= obmf_ipmi_write,
	.poll		= obmf_ipmi_poll,
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

	snprintf(id->name, sizeof(id->name), "ipmi-obmf%d-%u",
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

	if (!id || len < 2) {
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_IPMI,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	/*
	 * Strip the OBMF Command byte (data[0], always 0x00 = Send IPMI
	 * Message) so userspace sees pure IPMI: [NetFn/LUN, Cmd, Data...].
	 */

	mutex_lock(&id->flock);
	id->dev_req_len = min_t(int, len - 1, (int)sizeof(id->dev_req_buf));
	memcpy(id->dev_req_buf, data + 1, id->dev_req_len);
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
