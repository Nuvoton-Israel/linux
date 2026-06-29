// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-i2c.c - OBMF-ICP I2C virtual adapter (Channel Type 04h)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/usb.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

/*
 * I2C Controller Optimised Channel payload (v0.9):
 *
 *   Request:
 *     Byte 0: [6:0]=Command (0=Read, 1=Write, 2=SMBus Block Read,
 *              3=SMBus Write Read, 4=SMBus Host Notify)
 *             [7]=NoStop (0=send STOP, 1=do NOT send STOP)
 *     Byte 1: I2C/SMBus target address
 *     Byte 2-3: Read Length (u16 LE; 0 for write-only)
 *     Byte 4..N: Write Data
 *
 *   Response:
 *     Byte 0: [6:0]=Command echo, [7]=STOP flag
 *     Byte 1: Address echo
 *     Byte 2-3: Actual Read Length (u16 LE)
 *     Byte 4..N: Read Data
 */

#define OBMF_I2C_REQ_HDR_SIZE	4	/* Command(1) + Addr(1) + ReadLen(2) */

static int obmf_i2c_xfer(struct i2c_adapter *adap,
			 struct i2c_msg *msgs, int num)
{
	struct obmf_channel *ch = i2c_get_adapdata(adap);
	struct obmf_device *odev = ch->odev;
	u8 req_buf[OBMF_I2C_REQ_HDR_SIZE + 256];
	u8 resp_buf[OBMF_I2C_RESP_HDR_SIZE + 256];
	int i, rv;

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		int req_len;
		u8 cmd_byte;

		/*
		 * Build byte 0: [6:0]=Command, [7]=NoStop.
		 * Spec: bit[7]=0 → send STOP, bit[7]=1 → do NOT send STOP.
		 * Send STOP only on the last message in the sequence.
		 */
		if (msg->flags & I2C_M_RD) {
			cmd_byte = OBMF_I2C_CMD_READ;
			if (i < num - 1)
				cmd_byte |= OBMF_I2C_NO_STOP;

			req_buf[0] = cmd_byte;
			req_buf[1] = msg->addr;
			put_unaligned_le16(msg->len, &req_buf[2]);
			req_len = OBMF_I2C_REQ_HDR_SIZE;
		} else {
			cmd_byte = OBMF_I2C_CMD_WRITE;
			if (i < num - 1)
				cmd_byte |= OBMF_I2C_NO_STOP;

			req_buf[0] = cmd_byte;
			req_buf[1] = msg->addr;
			put_unaligned_le16(0, &req_buf[2]); /* ReadLen=0 */
			memcpy(&req_buf[OBMF_I2C_REQ_HDR_SIZE],
			       msg->buf, msg->len);
			req_len = OBMF_I2C_REQ_HDR_SIZE + msg->len;
		}

		mutex_lock(&ch->lock);
		rv = obmf_send_request(odev, ch, OBMF_TYPE_I2C,
				       req_buf, req_len,
				       resp_buf, sizeof(resp_buf),
				       OBMF_DEFAULT_TIMEOUT_MS);
		mutex_unlock(&ch->lock);

		if (rv < 0)
			return rv;

		/* v0.9: status checked by transport layer, rv is data length */

		/*
		 * Parse response: Command(1) + Address(1) + ReadLen(2) + Data(N).
		 * Extract actual read length from bytes 2-3.
		 */
		if ((msg->flags & I2C_M_RD) && rv > OBMF_I2C_RESP_HDR_SIZE) {
			u16 actual_rd = get_unaligned_le16(&resp_buf[2]);
			int data_len = min_t(u16, actual_rd, msg->len);

			if (data_len > rv - OBMF_I2C_RESP_HDR_SIZE)
				data_len = rv - OBMF_I2C_RESP_HDR_SIZE;
			memcpy(msg->buf, resp_buf + OBMF_I2C_RESP_HDR_SIZE,
			       data_len);
		}
	}

	return num;
}

static u32 obmf_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm obmf_i2c_algo = {
	.master_xfer   = obmf_i2c_xfer,
	.functionality = obmf_i2c_func,
};

int obmf_i2c_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct i2c_adapter *adap;
	int rv;

	adap = kzalloc(sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;

	adap->owner = THIS_MODULE;
	adap->algo  = &obmf_i2c_algo;
	adap->dev.parent = &odev->intf->dev;
	snprintf(adap->name, sizeof(adap->name),
		 "OBMF%d I2C ch%u", odev->device_index, ch->channel_id);
	i2c_set_adapdata(adap, ch);

	rv = i2c_add_adapter(adap);
	if (rv) {
		kfree(adap);
		return rv;
	}

	ch->priv = adap;
	ch->sysfs_dev = &adap->dev;
	dev_info(&odev->intf->dev, "ch%u: registered I2C adapter %s\n",
		 ch->channel_id, dev_name(&adap->dev));
	return 0;
}

void obmf_i2c_unregister(struct obmf_channel *ch)
{
	struct i2c_adapter *adap = ch->priv;

	if (adap) {
		i2c_del_adapter(adap);
		kfree(adap);
		ch->priv = NULL;
	}
}
