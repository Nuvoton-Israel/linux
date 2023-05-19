// SPDX-License-Identifier: GPL-2.0+
/*
 * The driver for BMC side of SSIF interface
 *
 * Copyright (c) 2021, Ampere Computing LLC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include "ssif_bmc.h"

#define POLY    (0x1070U << 3)
static u8 crc8(u16 data)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (data & 0x8000)
			data = data ^ POLY;
		data = data << 1;
	}
	return (u8)(data >> 8);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
static u8 i2c_calculate_pec(u8 crc, u8 *p, size_t count)
{
	int i;

	for (i = 0; i < count; i++)
		crc = crc8((crc ^ p[i]) << 8);
	return crc;
}

static u8 i2c_8bit_addr(u8 addr_7bit)
{
	return (addr_7bit << 1);
}

/*
 * Call in WRITE context
 */
static int send_ssif_bmc_response(struct ssif_bmc_ctx *ssif_bmc, bool non_blocking)
{
	unsigned long flags;
	int ret;

	if (!non_blocking) {
retry:
		ret = wait_event_interruptible(ssif_bmc->wait_queue,
				!ssif_bmc->response_in_progress);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&ssif_bmc->lock, flags);
	if (ssif_bmc->response_in_progress) {
		spin_unlock_irqrestore(&ssif_bmc->lock, flags);
		if (non_blocking)
			return -EAGAIN;

		goto retry;
	}

	/*
	 * Check the response data length from userspace to determine the type
	 * of the response message whether it is single-part or multi-part.
	 */
	ssif_bmc->is_multi_part_read =
		(ssif_msg_len(&ssif_bmc->response) >
		 (MAX_PAYLOAD_PER_TRANSACTION + 1)) ?
		true : false; /* 1: byte of length */

	ssif_bmc->response_in_progress = true;
	spin_unlock_irqrestore(&ssif_bmc->lock, flags);

	return 0;
}

/*
 * Call in READ context
 */
static int receive_ssif_bmc_request(struct ssif_bmc_ctx *ssif_bmc, bool non_blocking)
{
	unsigned long flags;
	int ret;

	if (!non_blocking) {
retry:
		ret = wait_event_interruptible(
				ssif_bmc->wait_queue,
				ssif_bmc->request_available);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&ssif_bmc->lock, flags);
	if (!ssif_bmc->request_available) {
		spin_unlock_irqrestore(&ssif_bmc->lock, flags);
		if (non_blocking)
			return -EAGAIN;
		goto retry;
	}
	spin_unlock_irqrestore(&ssif_bmc->lock, flags);

	return 0;
}

/* Handle SSIF message that will be sent to user */
static ssize_t ssif_bmc_read(struct file *file, char __user *buf, size_t count,
					loff_t *ppos)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);
	unsigned long flags;
	ssize_t ret;

	mutex_lock(&ssif_bmc->file_mutex);

	ret = receive_ssif_bmc_request(ssif_bmc, file->f_flags & O_NONBLOCK);
	if (ret < 0)
		goto out;

	spin_lock_irqsave(&ssif_bmc->lock, flags);
	count = min_t(ssize_t, count, ssif_msg_len(&ssif_bmc->request));
	ret = copy_to_user(buf, &ssif_bmc->request, count);
	if (!ret)
		ssif_bmc->request_available = false;
	spin_unlock_irqrestore(&ssif_bmc->lock, flags);
out:
	mutex_unlock(&ssif_bmc->file_mutex);

	return (ret < 0) ? ret : count;
}

/* Handle SSIF message that is written by user */
static ssize_t ssif_bmc_write(struct file *file, const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);
	unsigned long flags;
	ssize_t ret;

	if (count > sizeof(struct ssif_msg))
		return -EINVAL;

	mutex_lock(&ssif_bmc->file_mutex);

	spin_lock_irqsave(&ssif_bmc->lock, flags);
	ret = copy_from_user(&ssif_bmc->response, buf, count);
	if ( ret || count < ssif_msg_len(&ssif_bmc->response)) {
		spin_unlock_irqrestore(&ssif_bmc->lock, flags);
		ret = -EINVAL;
		goto out;
	}
	spin_unlock_irqrestore(&ssif_bmc->lock, flags);

	ret = send_ssif_bmc_response(ssif_bmc, file->f_flags & O_NONBLOCK);
	if (!ret) {
		if (ssif_bmc->set_ssif_bmc_status)
			ssif_bmc->set_ssif_bmc_status(ssif_bmc, SSIF_BMC_READY);
	}
out:
	mutex_unlock(&ssif_bmc->file_mutex);

	return (ret < 0) ? ret : count;
}

static long ssif_bmc_ioctl(struct file *file, unsigned int cmd, unsigned long param)
{
	return 0;
}

static unsigned int ssif_bmc_poll(struct file *file, poll_table *wait)
{
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);
	unsigned int mask = 0;

	mutex_lock(&ssif_bmc->file_mutex);
	poll_wait(file, &ssif_bmc->wait_queue, wait);

	/*
	 * The request message is now available so userspace application can
	 * get the request
	 */
	if (ssif_bmc->request_available)
		mask |= POLLIN;

	mutex_unlock(&ssif_bmc->file_mutex);
	return mask;
}

static int ssif_bmc_open(struct inode *node, struct file *file)
{
	// reset status for avoid driver state error
	struct ssif_bmc_ctx *ssif_bmc = to_ssif_bmc(file);

	mutex_lock(&ssif_bmc->file_mutex);
	ssif_bmc->request_available = false;
	ssif_bmc->response_in_progress = false;
	// will reset automatic at I2C driver get data
	//ssif_bmc->msg_idx = 0;
	//ssif_bmc->msg_idwx = 0;
	mutex_unlock(&ssif_bmc->file_mutex);
	// avoid while reopen sysfs, the i2c cannot receive data (in master mode)
	if (ssif_bmc->set_ssif_bmc_status)
		ssif_bmc->set_ssif_bmc_status(ssif_bmc, SSIF_BMC_READY);
	printk(KERN_INFO "ssif open\n");

	return 0;
}

/*
 * System calls to device interface for user apps
 */
static const struct file_operations ssif_bmc_fops = {
	.owner		= THIS_MODULE,
	.read		= ssif_bmc_read,
	.write		= ssif_bmc_write,
	.poll		= ssif_bmc_poll,
	.unlocked_ioctl	= ssif_bmc_ioctl,
	.open		= ssif_bmc_open,
};

/* Called with ssif_bmc->lock held. */
static int handle_request(struct ssif_bmc_ctx *ssif_bmc)
{
	if (ssif_bmc->set_ssif_bmc_status)
		ssif_bmc->set_ssif_bmc_status(ssif_bmc, SSIF_BMC_BUSY);

	/* Request message is available to process */
	ssif_bmc->request_available = true;
	/*
	 * This is the new READ request.
	 * Clear the response buffer of the previous transaction
	 */
	memset(&ssif_bmc->response, 0, sizeof(struct ssif_msg));
	wake_up_all(&ssif_bmc->wait_queue);
	return 0;
}

/* Called with ssif_bmc->lock held. */
static int complete_response(struct ssif_bmc_ctx *ssif_bmc)
{
	/* Invalidate response in buffer to denote it having been sent. */
	ssif_bmc->response.len = 0;
	ssif_bmc->response_in_progress = false;
	ssif_bmc->num_bytes_processed = 0;
	ssif_bmc->remain_data_len = 0;
	memset(&ssif_bmc->response_buffer, 0, MAX_PAYLOAD_PER_TRANSACTION);
	wake_up_all(&ssif_bmc->wait_queue);
	return 0;
}

static void set_response_buffer(struct ssif_bmc_ctx *ssif_bmc)
{
	u8 response_data_len = 0;
	int idx = 0;
	u8 data_len;
	ssif_bmc->bytes_to_send = 0;

	if(ssif_bmc->response.len == 0)
		return;
	if (!ssif_bmc->is_multi_part_read) {
		switch (ssif_bmc->smbus_cmd) {
		case SSIF_IPMI_RESPONSE:
			memcpy(ssif_bmc->response_buffer,
				&ssif_bmc->response,
				MAX_PAYLOAD_PER_TRANSACTION + 1);
			ssif_bmc->bytes_to_send = ssif_bmc->response.len + 1;
			ssif_bmc->block_num = 0xFF;
			break;

		default:
			/* Do not expect to go to this case */
			pr_err("Error: Unexpected SMBus command received 0x%x\n",
					ssif_bmc->smbus_cmd);
			break;
		}
		return;
	}

	data_len = ssif_bmc->response.len;
	switch (ssif_bmc->smbus_cmd) {
	case SSIF_IPMI_RESPONSE:
		/*
		 * Read Start length is 32 bytes.
		 * Read Start transfer first 30 bytes of IPMI response
		 * and 2 special code 0x00, 0x01.
		 */
		ssif_bmc->remain_data_len =
			data_len - MAX_IPMI_DATA_PER_START_TRANSACTION;
		ssif_bmc->block_num = 0;

		ssif_bmc->response_buffer[idx++] = MAX_PAYLOAD_PER_TRANSACTION;
		ssif_bmc->response_buffer[idx++] = 0x00; /* Start Flag */
		ssif_bmc->response_buffer[idx++] = 0x01; /* Start Flag */
		ssif_bmc->response_buffer[idx++] = ssif_bmc->response.netfn_lun;
		ssif_bmc->response_buffer[idx++] = ssif_bmc->response.cmd;

		response_data_len = MAX_PAYLOAD_PER_TRANSACTION - idx + 1;

		memcpy(&ssif_bmc->response_buffer[idx],
			ssif_bmc->response.payload,
			response_data_len);
		ssif_bmc->bytes_to_send = MAX_PAYLOAD_PER_TRANSACTION;
		break;

	case SSIF_IPMI_MULTI_PART_RESPONSE_MIDDLE:
		/*
		 * IPMI READ Middle or READ End messages can carry up to 31 bytes
		 * IPMI data plus block number byte.
		 */
		if (ssif_bmc->remain_data_len <=
				MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION) {
			/*
			 * This is READ End message
			 *  Return length is the remaining response data length
			 *  plus block number
			 *  Block number 0xFF is to indicate this is last message
			 *
			 * Return length is: remain response plus block number
			 */
			ssif_bmc->block_num = 0xFF;
			ssif_bmc->response_buffer[idx++] = ssif_bmc->remain_data_len + 1;
			ssif_bmc->response_buffer[idx++] = ssif_bmc->block_num;
			response_data_len = ssif_bmc->remain_data_len;
		} else {
			/*
			 * This is READ Middle message
			 *  Response length is the maximum SMBUS transfer length
			 *  Block number byte is incremented
			 * Return length is maximum SMBUS transfer length
			 */
			ssif_bmc->remain_data_len -= MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION;
			response_data_len = MAX_IPMI_DATA_PER_MIDDLE_TRANSACTION;
			ssif_bmc->response_buffer[idx++] = MAX_PAYLOAD_PER_TRANSACTION;
			ssif_bmc->response_buffer[idx++] = ssif_bmc->block_num;
			ssif_bmc->block_num++;
		}

		ssif_bmc->bytes_to_send = response_data_len + 2;
		memcpy(&ssif_bmc->response_buffer[idx],
			ssif_bmc->response.payload + ssif_bmc->num_bytes_processed,
			response_data_len);
		break;

	default:
		/* Do not expect to go to this case */
		pr_err("Error: Unexpected SMBus command received 0x%x\n",
				ssif_bmc->smbus_cmd);
		break;
	}

	ssif_bmc->num_bytes_processed += response_data_len;

	return;
}

/* Process the IPMI response that will be read by master */
static void event_request_read(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	u8 *buf;
	if(ssif_bmc->msg_idx == 0 &&
		ssif_bmc->response_buffer[ssif_bmc->msg_idx] == 0) {
		*val = 1;
	} else {
		*val = ssif_bmc->response_buffer[ssif_bmc->msg_idx];
	}
}

static void event_received_write(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	u8 *buf;
	u8 index;
	u8 smbus_cmd;

	buf = (u8 *) &ssif_bmc->request;
	if (ssif_bmc->msg_idwx >= sizeof(struct ssif_msg))
		return;

	smbus_cmd = ssif_bmc->smbus_cmd;
	switch (smbus_cmd) {
	case SSIF_IPMI_REQUEST:
		/* Single-part write */
		buf[ssif_bmc->msg_idwx - 1] = *val;
		ssif_bmc->msg_idwx++;
		break;
	case SSIF_IPMI_MULTI_PART_REQUEST_START:
	case SSIF_IPMI_MULTI_PART_REQUEST_MIDDLE:
	case SSIF_IPMI_MULTI_PART_REQUEST_END:
		/* Multi-part write */
		if (ssif_bmc->msg_idwx == 1) {
			/* 2nd byte received is length */
			if (smbus_cmd == SSIF_IPMI_MULTI_PART_REQUEST_START) {
				/* Reset length to zero */
				ssif_bmc->request.len = 0;
			}
			ssif_bmc->request.len += *val;
			ssif_bmc->recv_data_len = *val;
			ssif_bmc->msg_idwx++;
			if((smbus_cmd == SSIF_IPMI_MULTI_PART_REQUEST_MIDDLE)
				&& *val < 0x20)
				ssif_bmc->smbus_cmd = SSIF_IPMI_MULTI_PART_REQUEST_END;
			break;
		}
		index = ssif_bmc->request.len - ssif_bmc->recv_data_len;
		buf[ssif_bmc->msg_idwx - 1 + index] = *val;
		ssif_bmc->msg_idwx++;
		break;
	default:
		/* Do not expect to go to this case */
		pr_err("Error: Unexpected SMBus command received 0x%x\n",
				ssif_bmc->smbus_cmd);
		break;
	}
}

static void complete_received(struct ssif_bmc_ctx *ssif_bmc)
{
	u8 cmd = ssif_bmc->smbus_cmd;

	if ((cmd == SSIF_IPMI_REQUEST) ||
	    (cmd == SSIF_IPMI_MULTI_PART_REQUEST_END))
		handle_request(ssif_bmc);

	return;
}

static void initialize_transfer(struct ssif_bmc_ctx *ssif_bmc, u8 *val)
{
	/* SMBUS command can vary (single or multi-part) */
	ssif_bmc->smbus_cmd = *val;
	ssif_bmc->msg_idwx++;

	if((ssif_bmc->smbus_cmd == SSIF_IPMI_REQUEST) ||
	   (ssif_bmc->smbus_cmd == SSIF_IPMI_MULTI_PART_REQUEST_START))
	{
		/* The response can be delayed in BMC causing host SSIF driver
		   to timeout and send a new request once BMC slave is ready.
		   In that case check for pending response and clear it
		 */
		if(ssif_bmc->response_in_progress)
			complete_response(ssif_bmc);
	}
	if((ssif_bmc->smbus_cmd == SSIF_IPMI_RESPONSE) ||
	   (ssif_bmc->smbus_cmd == SSIF_IPMI_MULTI_PART_RESPONSE_MIDDLE) )
	{
		set_response_buffer(ssif_bmc);
	}
}

/*
 * Callback function to handle I2C slave events
 */
static int ssif_bmc_cb(struct i2c_client *client,
				enum i2c_slave_event event, u8 *val)
{
	struct ssif_bmc_ctx *ssif_bmc = i2c_get_clientdata(client);

	spin_lock(&ssif_bmc->lock);

	/* I2C Event Handler:
	 *   I2C_SLAVE_READ_REQUESTED	0x0
	 *   I2C_SLAVE_WRITE_REQUESTED	0x1
	 *   I2C_SLAVE_READ_PROCESSED	0x2
	 *   I2C_SLAVE_WRITE_RECEIVED	0x3
	 *   I2C_SLAVE_STOP		0x4
	 */
	switch (event) {;

	case I2C_SLAVE_WRITE_REQUESTED:
		ssif_bmc->prev_event = I2C_SLAVE_WRITE_REQUESTED;
		/* Reset message index */
		ssif_bmc->msg_idx = 0;
		ssif_bmc->msg_idwx = 0;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		ssif_bmc->prev_event = I2C_SLAVE_WRITE_RECEIVED;
		/*
		 * First byte is SMBUS command, not a part of SSIF message.
		 * SSIF request buffer starts with msg_idx 1 for the first
		 *  buffer byte.
		 */
		if (ssif_bmc->msg_idwx == 0) {
			initialize_transfer(ssif_bmc, val);
		} else {
			event_received_write(ssif_bmc, val);
		}
		break;

	case I2C_SLAVE_READ_PROCESSED:
		ssif_bmc->prev_event = I2C_SLAVE_READ_PROCESSED;
		if(ssif_bmc->response_in_progress) {
			ssif_bmc->msg_idx++;
			if((ssif_bmc->msg_idx == ssif_bmc->bytes_to_send) &&
				(ssif_bmc->block_num == 0xFF)) {
				complete_response(ssif_bmc);
				break;
			}
		}
	case I2C_SLAVE_READ_REQUESTED:
		ssif_bmc->prev_event = I2C_SLAVE_READ_REQUESTED;
		event_request_read(ssif_bmc, val);
		break;

	case I2C_SLAVE_STOP:
		/*
		 * PEC byte is appended at the end of each transaction.
		 * Detect PEC is support or not after receiving write request
		 * completely.
		 */
		if (ssif_bmc->prev_event == I2C_SLAVE_WRITE_RECEIVED)
			complete_received(ssif_bmc);
		break;

	default:
		break;
	}

	spin_unlock(&ssif_bmc->lock);

	return 0;
}

struct ssif_bmc_ctx *ssif_bmc_alloc(struct i2c_client *client, int sizeof_priv)
{
	struct ssif_bmc_ctx *ssif_bmc;
	int ret;

	ssif_bmc = devm_kzalloc(&client->dev, sizeof(*ssif_bmc) + sizeof_priv, GFP_KERNEL);
	if (!ssif_bmc)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&ssif_bmc->lock);

	init_waitqueue_head(&ssif_bmc->wait_queue);
	ssif_bmc->request_available = false;
	ssif_bmc->response_in_progress = false;

	mutex_init(&ssif_bmc->file_mutex);

	/* Register misc device interface */
	ssif_bmc->miscdev.minor = MISC_DYNAMIC_MINOR;
	ssif_bmc->miscdev.name = DEVICE_NAME;
	ssif_bmc->miscdev.fops = &ssif_bmc_fops;
	ssif_bmc->miscdev.parent = &client->dev;
	ret = misc_register(&ssif_bmc->miscdev);
	if (ret)
		goto out;

	ssif_bmc->client = client;
	ssif_bmc->client->flags |= I2C_CLIENT_SLAVE;

	/* Register I2C slave */
	i2c_set_clientdata(client, ssif_bmc);
	ret = i2c_slave_register(client, ssif_bmc_cb);
	if (ret) {
		misc_deregister(&ssif_bmc->miscdev);
		goto out;
	}

	return ssif_bmc;

out:
	devm_kfree(&client->dev, ssif_bmc);
	return ERR_PTR(ret);;
}
EXPORT_SYMBOL(ssif_bmc_alloc);

MODULE_AUTHOR("Chuong Tran <chuong@os.amperecomputing.com>");
MODULE_AUTHOR("Quan Nguyen <quan@os.amperecomputing.com>");
MODULE_DESCRIPTION("Linux device driver of the BMC IPMI SSIF interface.");
MODULE_LICENSE("GPL");
