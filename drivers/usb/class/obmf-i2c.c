// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-i2c.c - OBMF-ICP I2C channels (OBMF-ICP v1.0.0 RC1)
 *               - I2C Controller (Channel Type 04h)
 *               - I2C Target     (Channel Type 05h)
 *
 * Architecture overview
 * ---------------------
 * The I2C Controller channel registers a standard Linux i2c_adapter so any
 * kernel I2C driver can use it via the normal I2C subsystem APIs.
 *
 * When an I2C slave driver (e.g. mctp-i2c) calls i2c_slave_register() on
 * that adapter, the I2C Target channel—configured on the same physical SMBus
 * in the SMC firmware—takes over receive traffic.  Incoming write transactions
 * forwarded by the SMC are translated into i2c_slave_event() calls, giving
 * mctp-i2c a transparent MCTP over SMBus path with no userspace involvement.
 *
 * DTS pairing
 * -----------
 * The I2C Target channel DTS node must carry:
 *   obmf,paired-controller = <&i2c_bus_label>;
 * pointing to the "i2c" child node of the paired I2C Controller channel
 * (the same node that becomes the controller adapter's dev.of_node).
 *
 *   smc@1 {
 *       i2c-ch@10 {
 *           reg = <10>;
 *           i2c27: i2c {
 *               #address-cells = <1>;
 *               #size-cells = <0>;
 *               mctp@10 {
 *                   compatible = "mctp-i2c-controller";
 *                   reg = <0x10>;
 *               };
 *           };
 *       };
 *       i2c-tgt@14 {
 *           reg = <14>;
 *           obmf,paired-controller = <&i2c27>;
 *       };
 *   };
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 */

#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/usb.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

/*
 * I2C Controller Channel payload (OBMF-ICP v1.0.0 RC1 §4.6)
 *
 * Request (unchanged from v0.9.2):
 *   Byte 0 [6:0]: Command (0=Read, 1=Write, 2=SMBus Block Read,
 *                          3=SMBus Write Read, 4=SMBus Host Notify)
 *   Byte 0 [7]:   NoStop (0=send STOP, 1=hold SCL low between transactions)
 *   Byte 1:       I2C/SMBus Target Address (7-bit)
 *   Bytes 2-3:    Read Length (u16 LE; 0 for write-only)
 *   Bytes 4..N:   Write Data
 *
 * Response (v1.0.0 RC1 — changed from v0.9.2):
 *   Byte 0 [6:0]: Command echo
 *   Byte 0 [7]:   Reserved (set to 0; was STOP flag in v0.9.2)
 *   Bytes 1-2:    Read Length (u16 LE; 0 for write-only) ← moved from 2-3
 *   Byte 3:       Reserved  (v0.9.2 had Address echo at Byte 1)
 *   Bytes 4..N:   Read Data
 */

#define OBMF_I2C_REQ_HDR_SIZE          4       /* Command(1)+Addr(1)+ReadLen(2) */
#define OBMF_I2C_RESP_RDLEN_OFF                1       /* ReadLen offset in v1.0 response */

/* ====================================================================
 * Private controller state
 * ==================================================================== */

/**
 * struct obmf_i2c_ctrl_data - private state for an I2C Controller channel.
 * @adap:        Linux i2c_adapter (embedded; addr via i2c_set_adapdata).
 * @ch:          Back-pointer to the OBMF channel (04h).
 * @slave:       i2c_client registered via reg_slave (CONFIG_I2C_SLAVE).
 * @slave_lock:  Spinlock protecting @slave.
 */
struct obmf_i2c_ctrl_data {
	struct i2c_adapter       adap;
	struct obmf_channel     *ch;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client       *slave;
	spinlock_t               slave_lock;
#endif
};

/* ====================================================================
 * I2C Controller helpers
 * ==================================================================== */

/**
 * obmf_i2c_get_of_node - locate the OF node for an OBMF I2C channel's adapter.
 * @odev: OBMF device
 * @ch:   OBMF I2C Controller channel
 *
 * Descends to the channel sub-node (reg == ch->channel_id) and returns its
 * "i2c" child node.  The caller must of_node_put() the result.
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
 * Parses response per OBMF-ICP v1.0.0 RC1: ReadLen at bytes 1-2.
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

	/*
	 * v1.0.0 RC1: ReadLen is at bytes 1-2 (OBMF_I2C_RESP_RDLEN_OFF).
	 * Read Data starts at byte 4 (OBMF_I2C_RESP_HDR_SIZE).
	 */
	if (rd_buf && rv > OBMF_I2C_RESP_HDR_SIZE) {
		u16 actual_rd = get_unaligned_le16(&resp[OBMF_I2C_RESP_RDLEN_OFF]);
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
	struct obmf_i2c_ctrl_data *cd = i2c_get_adapdata(adap);
	struct obmf_channel *ch = cd->ch;
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
	u32 f = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	f |= I2C_FUNC_SLAVE;
#endif
	return f;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static int obmf_i2c_reg_slave(struct i2c_client *client)
{
	struct obmf_i2c_ctrl_data *cd = i2c_get_adapdata(client->adapter);

	spin_lock(&cd->slave_lock);
	if (cd->slave) {
		spin_unlock(&cd->slave_lock);
		return -EBUSY;
	}
	cd->slave = client;
	spin_unlock(&cd->slave_lock);
	return 0;
}

static int obmf_i2c_unreg_slave(struct i2c_client *client)
{
	struct obmf_i2c_ctrl_data *cd = i2c_get_adapdata(client->adapter);

	spin_lock(&cd->slave_lock);
	cd->slave = NULL;
	spin_unlock(&cd->slave_lock);
	return 0;
}
#endif /* CONFIG_I2C_SLAVE */

static const struct i2c_algorithm obmf_i2c_algo = {
	.master_xfer   = obmf_i2c_xfer,
	.functionality = obmf_i2c_func,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave     = obmf_i2c_reg_slave,
	.unreg_slave   = obmf_i2c_unreg_slave,
#endif
};

int obmf_i2c_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_i2c_ctrl_data *cd;
	int rv;

	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	cd->ch = ch;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	spin_lock_init(&cd->slave_lock);
#endif

	cd->adap.owner      = THIS_MODULE;
	cd->adap.algo       = &obmf_i2c_algo;
	cd->adap.dev.parent = &odev->intf->dev;
	cd->adap.dev.of_node = obmf_i2c_get_of_node(odev, ch);
	snprintf(cd->adap.name, sizeof(cd->adap.name),
		 "OBMF%d I2C ch%u", odev->device_index, ch->channel_id);
	i2c_set_adapdata(&cd->adap, cd);

	rv = i2c_add_adapter(&cd->adap);
	if (rv) {
		of_node_put(cd->adap.dev.of_node);
		kfree(cd);
		return rv;
	}

	ch->priv = cd;
	ch->sysfs_dev = &cd->adap.dev;
	dev_info(&odev->intf->dev, "ch%u: registered I2C adapter %s\n",
		 ch->channel_id, dev_name(&cd->adap.dev));
	return 0;
}

void obmf_i2c_unregister(struct obmf_channel *ch)
{
	struct obmf_i2c_ctrl_data *cd = ch->priv;

	if (cd) {
		of_node_put(cd->adap.dev.of_node);
		i2c_del_adapter(&cd->adap);
		kfree(cd);
		ch->priv = NULL;
	}
}

/* ====================================================================
 * I2C Target Channel (Channel Type 05h)
 *
 * Physical I2C master --write--> SMC (physical I2C slave)
 *    SMC --OBMF I2C Target request--> BMC (this driver, Responder)
 *    BMC --OBMF I2C Target response--> SMC --> physical master
 *
 * Slave-mode path (when paired and i2c_slave_register() was called):
 *   Incoming write bytes are delivered as i2c_slave_event() calls so that
 *   kernel drivers like mctp-i2c can receive MCTP frames transparently.
 *   The event sequence is:
 *     I2C_SLAVE_WRITE_REQUESTED
 *     I2C_SLAVE_WRITE_RECEIVED  (once per wire byte after the I2C address)
 *     I2C_SLAVE_STOP
 *
 *   For MCTP over SMBus the wire frame after the I2C address is:
 *     [command=0x0F] [byte_count] [src_addr<<1] [MCTP header...] [payload...] [PEC]
 *   These arrive verbatim in data[4..N] and are fed as WRITE_RECEIVED events.
 *   mctp_i2c_slave_cb prepends the destination address itself when it handles
 *   the WRITE_REQUESTED event (rx_buffer[0] = mcli->lladdr << 1).
 *
 * When no slave is registered the write is NACKed immediately.
 * ==================================================================== */

/**
 * struct obmf_i2c_tgt_data - private state for an I2C Target channel.
 * @ch:          Back-pointer to the OBMF channel (05h).
 * @paired_ctrl: Controller channel data for slave-mode path (NULL if none).
 */
struct obmf_i2c_tgt_data {
	struct obmf_channel             *ch;
	struct obmf_i2c_ctrl_data       *paired_ctrl;
};

/* ---- DTS controller-pairing lookup --------------------------------- */

/**
 * obmf_i2c_find_paired_ctrl - resolve the "obmf,paired-controller" phandle.
 * @odev: OBMF device
 * @ch:   I2C Target channel to pair
 *
 * Reads the "obmf,paired-controller" phandle from the target channel's DTS
 * node.  The phandle must point to the "i2c" child node of the paired I2C
 * Controller channel (i.e. the same node assigned to the controller adapter's
 * dev.of_node by obmf_i2c_get_of_node()).
 *
 * Returns the paired obmf_i2c_ctrl_data pointer, or NULL if unavailable.
 */
#if IS_ENABLED(CONFIG_OF)
static struct obmf_i2c_ctrl_data *
obmf_i2c_find_paired_ctrl(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct usb_device *udev = odev->udev;
	struct device_node *udev_np, *tgt_ch_np = NULL, *tmp;
	struct device_node *ctrl_i2c_np;
	struct i2c_adapter *adap;
	struct obmf_i2c_ctrl_data *cd = NULL;

	/* 1. Find this target channel's DTS node (reg == channel_id). */
	udev_np = of_node_get(udev->dev.of_node);
	if (!udev_np)
		udev_np = obmf_find_udev_of_node(udev);
	if (!udev_np)
		return NULL;

	for_each_child_of_node(udev_np, tmp) {
		u32 reg;

		if (!of_property_read_u32(tmp, "reg", &reg) &&
		    reg == ch->channel_id) {
			tgt_ch_np = tmp; /* inherits ref from loop on break */
			break;
		}
	}
	of_node_put(udev_np);

	if (!tgt_ch_np)
		return NULL;

	/* 2. Read "obmf,paired-controller" phandle → i2c adapter OF node. */
	ctrl_i2c_np = of_parse_phandle(tgt_ch_np, "obmf,paired-controller", 0);
	of_node_put(tgt_ch_np);

	if (!ctrl_i2c_np)
		return NULL;

	/* 3. Locate the registered i2c_adapter for that OF node. */
	adap = of_find_i2c_adapter_by_node(ctrl_i2c_np);
	of_node_put(ctrl_i2c_np);

	if (!adap)
		return NULL;

	/* 4. Extract ctrl_data; verify it belongs to this odev. */
	cd = i2c_get_adapdata(adap);
	if (cd && cd->ch->odev != odev)
		cd = NULL;

	put_device(&adap->dev); /* release ref from of_find_i2c_adapter_by_node */
	return cd;
}
#else
static struct obmf_i2c_ctrl_data *
obmf_i2c_find_paired_ctrl(struct obmf_device *odev, struct obmf_channel *ch)
{
	return NULL;
}
#endif /* CONFIG_OF */

/* ---- Registration / unregistration --------------------------------- */

int obmf_i2c_target_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_i2c_tgt_data *td;

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td)
		return -ENOMEM;

	td->ch = ch;

	/*
	 * Try DTS pairing immediately.  If the paired controller channel was
	 * assigned a higher channel_id, it may not be registered yet; in that
	 * case obmf_i2c_target_finalize_pairing() will retry after all channels
	 * are registered.
	 */
	td->paired_ctrl = obmf_i2c_find_paired_ctrl(odev, ch);

	ch->priv = td;
	if (td->paired_ctrl)
		dev_info(&odev->intf->dev,
			 "ch%u: registered I2C target (paired with i2c-%d)\n",
			 ch->channel_id, td->paired_ctrl->adap.nr);
	else
		dev_info(&odev->intf->dev,
			 "ch%u: registered I2C target (no controller paired)\n",
			 ch->channel_id);
	return 0;
}

/* ---- Device-initiated write request handler ----------------------- */

void obmf_i2c_target_handle_dev_request(struct obmf_channel *ch,
					const u8 *data, int len)
{
	struct obmf_i2c_tgt_data *td = ch->priv;
	struct obmf_device *odev = ch->odev;
	u8 cmd_echo = OBMF_I2C_TGT_CMD_WRITE;
	u8 cmd;

	if (!td || len < OBMF_I2C_TGT_REQ_HDR_SIZE) {
		obmf_send_response(odev, ch->channel_id, OBMF_TYPE_I2C_TARGET,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	cmd = data[0] & OBMF_I2C_TGT_CMD_MASK;
	if (cmd != OBMF_I2C_TGT_CMD_WRITE) {
		obmf_send_response(odev, ch->channel_id, OBMF_TYPE_I2C_TARGET,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	/*
	 * Slave-mode path: deliver the incoming write to a registered kernel
	 * I2C slave driver (e.g. mctp-i2c) via i2c_slave_event().
	 *
	 * OBMF Target request layout (v1.0 §4.7):
	 *   data[0]: Command (1 = I2C Write)
	 *   data[1]: I2C/SMBus Target Address (7-bit)
	 *   data[2-3]: Reserved
	 *   data[4..N]: Write Data (raw bytes from wire, after the I2C address)
	 *
	 * data[1] must match slave->addr (7-bit).  mctp_i2c_slave_cb fills
	 * rx_buffer[0] = (mcli->lladdr << 1) itself on WRITE_REQUESTED; the
	 * driver does NOT receive the destination address as a WRITE_RECEIVED
	 * byte — only the payload starting from data[4] is fed in.
	 */
	if (td->paired_ctrl) {
		struct i2c_client *slave;
		int wr_len = len - OBMF_I2C_TGT_REQ_HDR_SIZE;
		const u8 *wr_data = data + OBMF_I2C_TGT_REQ_HDR_SIZE;
		u8 dummy = 0;
		int i;

		spin_lock(&td->paired_ctrl->slave_lock);
		slave = td->paired_ctrl->slave;
		spin_unlock(&td->paired_ctrl->slave_lock);

		if (slave) {
			if (data[1] != slave->addr) {
				/* Address mismatch — NACK */
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_I2C_TARGET,
						   OBMF_I2C_TGT_STATUS_TRANSACTION,
						   &cmd_echo, 1);
				return;
			}

			/*
			 * Emulate byte-level I2C slave protocol as required by
			 * OBMF-ICP v1.0 §4.7 for MCTP over SMBus compatibility.
			 */
			i2c_slave_event(slave, I2C_SLAVE_WRITE_REQUESTED, &dummy);
			for (i = 0; i < wr_len; i++) {
				u8 b = wr_data[i];

				i2c_slave_event(slave,
						I2C_SLAVE_WRITE_RECEIVED, &b);
			}
			i2c_slave_event(slave, I2C_SLAVE_STOP, &dummy);

			/* ACK: spec §4.7 requires acknowledging all written bytes */
			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_I2C_TARGET,
					   OBMF_STATUS_SUCCESS, &cmd_echo, 1);
			return;
		}
	}
#endif /* CONFIG_I2C_SLAVE */

	/* No slave registered — NACK the write */
	obmf_send_response(odev, ch->channel_id, OBMF_TYPE_I2C_TARGET,
			   OBMF_I2C_TGT_STATUS_TRANSACTION, &cmd_echo, 1);
}

void obmf_i2c_target_unregister(struct obmf_channel *ch)
{
	struct obmf_i2c_tgt_data *td = ch->priv;

	if (td) {
		kfree(td);
		ch->priv = NULL;
	}
}

/**
 * obmf_i2c_target_finalize_pairing - retry controller pairing for all targets.
 * @odev: OBMF device
 *
 * Called by obmf-core.c after all channels are registered.  Handles the case
 * where a target channel's paired controller had a higher channel_id and was
 * therefore not yet registered when obmf_i2c_target_register() ran.
 */
void obmf_i2c_target_finalize_pairing(struct obmf_device *odev)
{
	int i;

	for (i = 1; i < odev->num_channels; i++) {
		struct obmf_channel *ch = &odev->channels[i];
		struct obmf_i2c_tgt_data *td;

		if (ch->channel_type != OBMF_TYPE_I2C_TARGET || !ch->priv)
			continue;

		td = ch->priv;
		if (td->paired_ctrl)
			continue; /* already paired */

		td->paired_ctrl = obmf_i2c_find_paired_ctrl(odev, ch);
		if (td->paired_ctrl)
			dev_info(&odev->intf->dev,
				 "ch%u: I2C target paired with i2c-%d (deferred)\n",
				 ch->channel_id, td->paired_ctrl->adap.nr);
	}
}
