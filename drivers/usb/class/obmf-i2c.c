// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-i2c.c - OBMF-ICP I2C virtual adapter (Channel Type 04h)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 */

#include <linux/i2c.h>
#include <linux/of.h>
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

/**
 * obmf_i2c_get_of_node - locate the OF node for an OBMF I2C channel's adapter.
 * @odev: OBMF device
 * @ch:   OBMF I2C channel
 *
 * Obtain the USB device's OF node via udev->dev.of_node (set by the USB core
 * through usb_of_get_device_node(), which already handles arbitrary hub
 * topologies), falling back to obmf_find_udev_of_node() for cases where the
 * USB core has not assigned an OF node.  Then descend to the channel sub-node
 * (reg == ch->channel_id) and return its "i2c" child:
 *
 *   smc@1 {
 *       reg = <1>;
 *       i2c-ch@10 {
 *           reg = <10>;           // channel_id = 10 = OBMF I2C channel
 *           i2c27: i2c {
 *               #address-cells = <1>;
 *               #size-cells = <0>;
 *               tmp100@48 { compatible = "ti,tmp100"; reg = <0x48>; };
 *           };
 *       };
 *   };
 *
 * Returns a device_node with elevated refcount (caller must of_node_put()),
 * or NULL if the DTS does not describe an i2c bus for this channel.
 */
static struct device_node *obmf_i2c_get_of_node(struct obmf_device *odev,
					     struct obmf_channel *ch)
{
	struct usb_device *udev = odev->udev;
	struct device_node *udev_np, *ch_np = NULL, *tmp, *i2c_np;

	udev_np = of_node_get(udev->dev.of_node);
	if (!udev_np)
		udev_np = obmf_find_udev_of_node(udev);
	if (!udev_np)
		return NULL;

	for_each_child_of_node(udev_np, tmp) {
		u32 reg;

		if (!of_property_read_u32(tmp, "reg", &reg) &&
		    reg == ch->channel_id) {
			ch_np = tmp; /* inherits ref from loop on break */
			break;
		}
	}
	of_node_put(udev_np);

	if (!ch_np)
		return NULL;

	i2c_np = of_get_child_by_name(ch_np, "i2c");
	of_node_put(ch_np);
	return i2c_np;
}

/*
 * Send one pre-built I2C channel request and copy read data (if any) into
 * @rd_buf.  Returns the raw response length on success or a negative errno.
 */
static int obmf_i2c_do_request(struct obmf_channel *ch,
				const u8 *req, int req_len,
				u8 *rd_buf, u16 rd_maxlen)
{
	struct obmf_device *odev = ch->odev;
	u8 resp[OBMF_I2C_RESP_HDR_SIZE + 256];
	int rv;

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch, OBMF_TYPE_I2C,
			       req, req_len, resp, sizeof(resp),
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);
	if (rv < 0)
		return rv;

	/* Response: Command(1) + Address(1) + ReadLen(2) + Data(N) */
	if (rd_buf && rv > OBMF_I2C_RESP_HDR_SIZE) {
		u16 actual_rd = get_unaligned_le16(&resp[2]);
		int data_len = min_t(u16, actual_rd, rd_maxlen);

		if (data_len > rv - OBMF_I2C_RESP_HDR_SIZE)
			data_len = rv - OBMF_I2C_RESP_HDR_SIZE;
		memcpy(rd_buf, resp + OBMF_I2C_RESP_HDR_SIZE, data_len);
	}
	return rv;
}

static int obmf_i2c_xfer(struct i2c_adapter *adap,
			 struct i2c_msg *msgs, int num)
{
	struct obmf_channel *ch = i2c_get_adapdata(adap);
	u8 req[OBMF_I2C_REQ_HDR_SIZE + 256];
	int i, rv;

	/*
	 * Optimise write-then-read into a single SMBus Write Read request
	 * (cmd=3, OBMF-ICP §4.6.1) when both messages share the same address.
	 * This saves one full USB round-trip compared to separate Write + Read.
	 */
	if (num == 2 &&
	    !(msgs[0].flags & I2C_M_RD) &&
	      (msgs[1].flags & I2C_M_RD) &&
	    msgs[0].addr == msgs[1].addr &&
	    msgs[0].len > 0 && msgs[0].len <= 256 &&
	    msgs[1].len > 0 && msgs[1].len <= 256) {
		req[0] = OBMF_I2C_CMD_SMBUS_WRITE_READ; /* bit[7]=0: send STOP */
		req[1] = msgs[0].addr;
		put_unaligned_le16(msgs[1].len, &req[2]);
		memcpy(&req[OBMF_I2C_REQ_HDR_SIZE], msgs[0].buf, msgs[0].len);

		rv = obmf_i2c_do_request(ch, req,
					 OBMF_I2C_REQ_HDR_SIZE + msgs[0].len,
					 msgs[1].buf, msgs[1].len);
		return rv < 0 ? rv : num;
	}

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool is_read = !!(msg->flags & I2C_M_RD);
		int req_len;

		/*
		 * Byte 0: [6:0]=Command, [7]=NoStop.
		 * Send STOP only on the last message in the sequence.
		 */
		req[0] = is_read ? OBMF_I2C_CMD_READ : OBMF_I2C_CMD_WRITE;
		if (i < num - 1)
			req[0] |= OBMF_I2C_NO_STOP;
		req[1] = msg->addr;

		if (is_read) {
			put_unaligned_le16(msg->len, &req[2]);
			req_len = OBMF_I2C_REQ_HDR_SIZE;
		} else {
			put_unaligned_le16(0, &req[2]); /* ReadLen=0 */
			memcpy(&req[OBMF_I2C_REQ_HDR_SIZE], msg->buf, msg->len);
			req_len = OBMF_I2C_REQ_HDR_SIZE + msg->len;
		}

		rv = obmf_i2c_do_request(ch, req, req_len,
					 is_read ? msg->buf : NULL,
					 is_read ? msg->len  : 0);
		if (rv < 0)
			return rv;
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
	adap->dev.parent  = &odev->intf->dev;
	adap->dev.of_node = obmf_i2c_get_of_node(odev, ch);
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
		of_node_put(adap->dev.of_node);
		i2c_del_adapter(adap);
		kfree(adap);
		ch->priv = NULL;
	}
}
