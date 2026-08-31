// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-i3c.c - OBMF-ICP I3C Controller Channel (Channel Type 06h)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Registers a virtual i3c_master_controller over USB.  All I3C bus
 * operations are forwarded to the SMC via OBMF EXECUTE_SEQUENCE /
 * DO_DAA requests (OBMF-ICP v1.0.0 RC1 §4.8).
 *
 * v1 scope:
 *   - Mandatory ops: bus_init, do_daa, send_ccc_cmd, priv_xfers, i2c_xfers
 *   - SET_ASSOCIATED_I2C during bus_init when a sibling I2C channel exists
 *   - Producer events: IBI / Hot-Join / Bus-Error receive and dispatch
 *   - IBI request_ibi / enable_ibi: deferred to v2
 *
 * Wire-format reference (§4.8):
 *
 *   EXECUTE_SEQUENCE request:
 *     Byte 0:    OBMF_I3C_CMD_EXECUTE_SEQ
 *     Byte 1:    num_ops
 *     Byte 2+:   array of op descriptors
 *
 *   Each op descriptor (8-byte fixed header + transfer-specific):
 *     u8   op_type   (OBMF_I3C_OP_*)
 *     u8   send_stop (1 = P after this op, 0 = Sr)
 *     le16 od_khz   (0 = default)
 *     le16 pp_khz   (0 = default)
 *     le16 desc_len (bytes that follow in this descriptor)
 *     ... transfer-specific fields + write data ...
 *
 *   EXECUTE_SEQUENCE response:
 *     Byte 0:    command echo (0x00)
 *     Byte 1:    result_count
 *     Byte 2+:   array of result descriptors
 *       u8   op_status  (OBMF_I3C_OP_*)
 *       le16 actual_len
 *       u8[] read_data  (actual_len bytes, only for read ops)
 */

#include <linux/i3c/master.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

/* Fixed header size per op descriptor */
#define OBMF_I3C_OP_HDR_SZ	8
/* EXECUTE_SEQUENCE request prefix: cmd(1) + num_ops(1) */
#define OBMF_I3C_REQ_PFX_SZ	2
/* Scratch buffer limit for one EXECUTE_SEQUENCE exchange */
#define OBMF_I3C_BUF_SZ		(256 + OBMF_I3C_OP_HDR_SZ + OBMF_I3C_REQ_PFX_SZ)

/* ------------------------------------------------------------------ */
/* Private state                                                       */
/* ------------------------------------------------------------------ */

struct obmf_i3c_priv {
	struct i3c_master_controller  master;		/* MUST be first */
	struct obmf_device	     *odev;
	struct obmf_channel	     *ch;
	struct obmf_channel	     *assoc_i2c_ch;	/* sibling I2C ch or NULL */

	/* Capabilities from channel configuration data (§4.13.2) */
	u8	max_chained_ops;	/* MAX_CHAINED_OPS */
	u16	max_payload_size;	/* MAX_PAYLOAD_SIZE */
	u16	max_ibi_payload;	/* MAX_IBI_PAYLOAD_SIZE */
	u16	max_scl_pp_khz;		/* MAX_SCL_PP_KHZ */
	u16	max_scl_od_khz;		/* MAX_SCL_OD_KHZ */
	u8	feature_flags;		/* I3C_FEATURE_FLAGS */

	struct work_struct	hotjoin_work;	/* deferred from demux */
};

static inline struct obmf_i3c_priv *
to_i3c_priv(struct i3c_master_controller *m)
{
	return container_of(m, struct obmf_i3c_priv, master);
}

/* ------------------------------------------------------------------ */
/* OF / DTS helper                                                     */
/* ------------------------------------------------------------------ */

/*
 * Locate the "i3c" child node for this OBMF channel in the DT.
 *
 * Expected layout:
 *
 *   &ehci1 {
 *       smc@1 {
 *           reg = <1>;
 *           i3c-ch@5 {
 *               reg = <5>;   // channel_id of this I3C channel
 *               i3c {
 *                   #address-cells = <3>;
 *                   #size-cells    = <0>;
 *                   // per-device nodes with reg = <SA PID_HI PID_LO>
 *               };
 *           };
 *       };
 *   };
 *
 * Returns a device_node with elevated refcount; caller must of_node_put().
 */
static struct device_node *
obmf_i3c_get_of_node(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct usb_device   *udev = odev->udev;
	struct device_node  *udev_np, *ch_np = NULL, *tmp, *i3c_np;

	udev_np = of_node_get(udev->dev.of_node);
	if (!udev_np)
		udev_np = obmf_find_udev_of_node(udev);
	if (!udev_np)
		return NULL;

	for_each_child_of_node(udev_np, tmp) {
		u32 reg;

		if (!of_property_read_u32(tmp, "reg", &reg) &&
		    reg == ch->channel_id) {
			ch_np = tmp;	/* inherits ref from loop */
			break;
		}
	}
	of_node_put(udev_np);

	if (!ch_np)
		return NULL;

	i3c_np = of_get_child_by_name(ch_np, "i3c");
	of_node_put(ch_np);
	return i3c_np;
}

/* ------------------------------------------------------------------ */
/* Config data reader                                                  */
/* ------------------------------------------------------------------ */

static int obmf_i3c_read_config(struct obmf_i3c_priv *priv)
{
	struct obmf_device  *odev = priv->odev;
	struct obmf_channel *ch   = priv->ch;
	struct obmf_channel *ch0  = &odev->channels[0];
	u8  cfg[OBMF_I3C_CFG_DATA_SIZE];
	int rv;

	if (!ch->config_offset || ch->config_size < OBMF_I3C_CFG_DATA_SIZE)
		return -ENODATA;

	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
				    ch->config_offset + OBMF_CHCFG_CONFIG_DATA,
				    NULL, 0, cfg, sizeof(cfg));
	if (rv < 0)
		return rv;

	priv->max_scl_pp_khz   = get_unaligned_le16(&cfg[OBMF_I3C_CFG_MAX_SCL_PP_KHZ]);
	priv->max_scl_od_khz   = get_unaligned_le16(&cfg[OBMF_I3C_CFG_MAX_SCL_OD_KHZ]);
	priv->feature_flags    = cfg[OBMF_I3C_CFG_FEATURE_FLAGS];
	priv->max_chained_ops  = cfg[OBMF_I3C_CFG_MAX_CHAINED_OPS];
	priv->max_payload_size = get_unaligned_le16(&cfg[OBMF_I3C_CFG_MAX_PAYLOAD]);
	priv->max_ibi_payload  = get_unaligned_le16(&cfg[OBMF_I3C_CFG_MAX_IBI_PAYLOAD]);

	/* Sanity: enforce lower bounds */
	if (!priv->max_chained_ops)
		priv->max_chained_ops = 1;
	if (!priv->max_payload_size)
		priv->max_payload_size = 256;

	dev_err(&odev->intf->dev,
		"ch%u I3C config: pp=%ukHz od=%ukHz max_ops=%u payload=%u ibi=%u feat=0x%02x\n",
		ch->channel_id,
		priv->max_scl_pp_khz, priv->max_scl_od_khz,
		priv->max_chained_ops, priv->max_payload_size,
		priv->max_ibi_payload, priv->feature_flags);
	return 0;
}

/* ------------------------------------------------------------------ */
/* EXECUTE_SEQUENCE wire helpers                                       */
/* ------------------------------------------------------------------ */

/*
 * Append one op descriptor to @req at byte offset @off.
 *
 * Returns the new offset on success, or a negative error code.
 *
 * @op_type:  OBMF_I3C_OP_CCC or OBMF_I3C_OP_PRIVATE_SDR
 * @send_stop: 1 = STOP after this op, 0 = Repeated-Start
 * @addr:     7-bit dynamic address (I3C_BROADCAST_ADDR for broadcast CCC)
 * @ccc_id:   CCC command ID (ignored for PRIVATE_SDR)
 * @rnw:      true = read, false = write
 * @data:     write payload pointer (NULL for reads)
 * @data_len: write payload length (or read expected length)
 */
static int obmf_i3c_append_op(u8 *req, int off, int buf_sz,
			       u8 op_type, u8 send_stop,
			       u8 addr, u8 ccc_id, bool rnw,
			       const void *data, u16 data_len)
{
	u16 desc_len;

	/*
	 * Calculate descriptor_len: number of transfer-specific bytes
	 * that follow the 8-byte fixed header.
	 */
	switch (op_type) {
	case OBMF_I3C_OP_CCC:
		/* ccc_id(1) + addr(1) + is_write(1) + data_len(2) + write_data */
		desc_len = 5 + (rnw ? 0 : data_len);
		break;
	case OBMF_I3C_OP_PRIVATE_SDR:
		/* addr(1) + is_write(1) + data_len(2) + write_data */
		desc_len = 4 + (rnw ? 0 : data_len);
		break;
	default:
		return -EINVAL;
	}

	if (off + OBMF_I3C_OP_HDR_SZ + desc_len > buf_sz)
		return -ENOSPC;

	/* 8-byte fixed op header */
	req[off++] = op_type;
	req[off++] = send_stop;
	put_unaligned_le16(0, &req[off]); off += 2;	/* od_khz: default */
	put_unaligned_le16(0, &req[off]); off += 2;	/* pp_khz: default */
	put_unaligned_le16(desc_len, &req[off]); off += 2;

	/* Transfer-specific fields */
	if (op_type == OBMF_I3C_OP_CCC) {
		req[off++] = ccc_id;
		req[off++] = addr;
		req[off++] = rnw ? 0 : 1;		/* is_write */
		put_unaligned_le16(data_len, &req[off]); off += 2;
	} else { /* OBMF_I3C_OP_PRIVATE_SDR */
		req[off++] = addr;
		req[off++] = rnw ? 0 : 1;		/* is_write */
		put_unaligned_le16(data_len, &req[off]); off += 2;
	}

	if (!rnw && data && data_len) {
		memcpy(&req[off], data, data_len);
		off += data_len;
	}

	return off;
}

/*
 * Parse one operation result from the EXECUTE_SEQUENCE response stream.
 *
 * @resp / @resp_len: pointer + remaining bytes in response ops array
 * @is_read:  whether this op was a read (determines read_data presence)
 * @rd_buf:   destination for read data (NULL if not needed)
 * @rd_len:   capacity of @rd_buf
 * @err:      set to I3C_ERROR_Mx on recoverable bus errors (may be NULL)
 *
 * Returns bytes consumed from @resp on success, or negative errno.
 * Errno indicates the op-level failure; callers should propagate upward.
 */
static int obmf_i3c_parse_op_result(const u8 *resp, int resp_len,
				    bool is_read, void *rd_buf, u16 rd_len,
				    enum i3c_error_code *err, u16 *ret_actual_len)
{
	u8  op_status;
	u16 actual_len;
	int consumed;

	if (resp_len < 3)
		return -EMSGSIZE;

	op_status  = resp[0];
	actual_len = get_unaligned_le16(&resp[1]);
	consumed   = 3;

	if (is_read) {
		/*
		 * Read ops: actual_len read bytes follow in the response wire.
		 */
		if (actual_len > (u16)(resp_len - 3))
			return -EMSGSIZE;
		if (rd_buf && actual_len > 0)
			memcpy(rd_buf, resp + 3, min_t(u16, actual_len, rd_len));
		consumed += actual_len;
	}
	/*
	 * Write ops: the SMC sets actual_len = bytes-written as a confirmation
	 * count but does NOT echo those bytes back in the response wire frame
	 * (e.g. DISEC response has actual_len=1 but no trailing byte).
	 * consumed stays at 3 — do not advance past non-existent data.
	 */

	/*
	 * Report the SMC-reported actual_len back to the caller so it can
	 * update the in/out length field (e.g. i3c_priv_xfer.len) — the i3c
	 * core and client drivers (e.g. mctp-i3c.c) rely on this field being
	 * updated to the real number of bytes transferred, not the requested
	 * length.  Cap to rd_len for reads so callers never see a bogus
	 * length larger than their buffer.
	 */
	if (ret_actual_len)
		*ret_actual_len = is_read ? min_t(u16, actual_len, rd_len) : actual_len;

	switch (op_status) {
	case OBMF_I3C_OP_SUCCESS:
		return consumed;
	case OBMF_I3C_OP_NACK:
		if (err)
			*err = I3C_ERROR_M2;
		return -ENXIO;
	case OBMF_I3C_OP_ARB_LOST:
		if (err)
			*err = I3C_ERROR_M1;
		return -EAGAIN;
	case OBMF_I3C_OP_TIMEOUT:
		if (err)
			*err = I3C_ERROR_M2;
		return -ETIMEDOUT;
	case OBMF_I3C_OP_BUS_ERR:
		if (err)
			*err = I3C_ERROR_M0;
		return -EIO;
	default:
		return -EIO;
	}
}

/* ------------------------------------------------------------------ */
/* i3c_master_controller_ops                                           */
/* ------------------------------------------------------------------ */

/*
 * bus_init - initialise the I3C bus.
 *
 * Called synchronously by i3c_master_register() → i3c_master_bus_init().
 * Must call i3c_master_set_info() before returning.
 */
static int obmf_i3c_bus_init(struct i3c_master_controller *master)
{
	struct obmf_i3c_priv  *priv = to_i3c_priv(master);
	struct obmf_device    *odev = priv->odev;
	struct obmf_channel   *ch   = priv->ch;
	struct i3c_device_info info  = {};
	u8   req[6];
	u8   resp[4];
	int  rv;

	dev_warn(&odev->intf->dev,
		 "ch%u: I3C bus_init() called\n", ch->channel_id);

	/*
	 * Set of_node here (before device_add fires notifiers) so that
	 * mctp-i3c and other consumers see the correct DT node.
	 * We do this in bus_init rather than before i3c_master_register()
	 * to avoid needing the "if (!master->dev.of_node)" guard in
	 * i3c_master_register().
	 */
	if (!master->dev.of_node)
		master->dev.of_node = obmf_i3c_get_of_node(odev, ch);
	/* Read hardware capability from channel config data */
	rv = obmf_i3c_read_config(priv);
	if (rv) {
		dev_warn(&odev->intf->dev,
			 "ch%u: config unreadable (%d), using defaults\n",
			 ch->channel_id, rv);
		priv->max_chained_ops  = 1;
		priv->max_payload_size = 256;
		priv->max_scl_pp_khz   = 12500;
		priv->max_scl_od_khz   = 400;
	}

	/*
	 * Set bus mode and SCL rates.  We set the fields directly because
	 * i3c_bus_set_mode() is static in the framework.
	 */
	if (priv->assoc_i2c_ch && priv->max_scl_od_khz) {
		master->bus.mode = I3C_BUS_MODE_MIXED_FAST;
		master->bus.scl_rate.i2c =
			(unsigned long)priv->max_scl_od_khz * 1000;
	} else {
		master->bus.mode = I3C_BUS_MODE_PURE;
	}
	if (priv->max_scl_pp_khz)
		master->bus.scl_rate.i3c =
			(unsigned long)priv->max_scl_pp_khz * 1000;

	/*
	 * Register this virtual master on the bus.
	 *
	 * The PID is synthesised from USB VID/PID + channel_id so it is
	 * deterministic per SMC and unique per I3C channel.
	 * BCR = I3C_BCR_I3C_MASTER (bit 6 = primary master role).
	 */
	info.dyn_addr = 0x08;	/* first non-reserved address */
	info.bcr      = I3C_BCR_I3C_MASTER;
	info.dcr      = 0x00;
	info.pid      =
		((u64)le16_to_cpu(odev->udev->descriptor.idVendor)  << 32) |
		((u64)le16_to_cpu(odev->udev->descriptor.idProduct) << 16) |
		ch->channel_id;

	rv = i3c_master_set_info(master, &info);
	if (rv)
		return rv;

	/*
	 * Optionally associate a sibling OBMF I2C channel so the SMC can
	 * handle legacy I2C devices on the mixed I3C bus.
	 * Byte layout: cmd(1) + param1(1) + param2(2=i2c_ch_id) + param3(2)
	 */
	if (priv->assoc_i2c_ch) {
		memset(req, 0, sizeof(req));
		req[0] = OBMF_I3C_CMD_SET_ASSOC_I2C;
		put_unaligned_le16(priv->assoc_i2c_ch->channel_id, &req[2]);

		dev_warn(&odev->intf->dev,
			 "ch%u: SET_ASSOC_I2C(ch%u)\n",
			 ch->channel_id, priv->assoc_i2c_ch->channel_id);
		mutex_lock(&ch->lock);
		rv = obmf_send_request(odev, ch, OBMF_TYPE_I3C,
				       req, sizeof(req),
				       resp, sizeof(resp),
				       OBMF_DEFAULT_TIMEOUT_MS);
		mutex_unlock(&ch->lock);
		if (rv < 0)
			dev_warn(&odev->intf->dev,
				 "ch%u: SET_ASSOC_I2C(ch%u) failed: %d\n",
				 ch->channel_id,
				 priv->assoc_i2c_ch->channel_id, rv);
	}

	return 0;
}

/*
 * do_daa - perform Dynamic Address Assignment (ENTDAA).
 *
 * Sends I3C_CMD_DO_DAA to the SMC, then registers every discovered device
 * with i3c_master_add_i3c_dev_locked() so the framework issues GETPID /
 * GETBCR / GETDCR and creates /dev/i3c-<bus>-<pid> nodes via i3cdev.
 *
 * Called with bus maintenance lock held by the framework.
 */
static int obmf_i3c_do_daa(struct i3c_master_controller *master)
{
	struct obmf_i3c_priv *priv = to_i3c_priv(master);
	struct obmf_device   *odev = priv->odev;
	struct obmf_channel  *ch   = priv->ch;
	u8  *req, *resp;
	int  rv, i;

	if (odev->disconnected)
		return -ENODEV;

	req  = kzalloc(OBMF_I3C_BUF_SZ, GFP_KERNEL);
	resp = kzalloc(OBMF_I3C_BUF_SZ, GFP_KERNEL);
	if (!req || !resp) {
		rv = -ENOMEM;
		goto out;
	}

	/*
	 * I3C_CMD_DO_DAA request:
	 *   Byte 0: 0x01
	 *   Byte 1: num_excluded_addresses = 0
	 *   Byte 2-3: target_scl_od_khz = 0 (default)
	 *   Byte 4-5: target_scl_pp_khz = 0 (default)
	 */
	req[0] = OBMF_I3C_CMD_DO_DAA;
	/* bytes 1–5 remain zero */

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch, OBMF_TYPE_I3C,
			       req, 6,
			       resp, OBMF_I3C_BUF_SZ,
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);
	if (rv < 0)
		goto out;

	/*
	 * DO_DAA response:
	 *   Byte 0: command echo (0x01)
	 *   Byte 1: num_discovered_devices
	 *   Byte 2+: 9-byte entries {dyn_addr(1), pid[6], bcr(1), dcr(1)}
	 *
	 * We rely on i3c_master_add_i3c_dev_locked() to call back into
	 * send_ccc_cmd (GETPID / GETBCR / GETDCR) to validate each device.
	 * The PID / BCR / DCR from the response are therefore informational
	 * and not used directly.
	 */
	if (rv < 2) {
		rv = -EPROTO;
		goto out;
	}

	{
		u8 ndev = resp[1];

		if (rv < 2 + (int)ndev * 9) {
			dev_warn(&odev->intf->dev,
				 "ch%u: DO_DAA truncated: got %d need %d\n",
				 ch->channel_id, rv, 2 + ndev * 9);
			rv = -EPROTO;
			goto out;
		}

		dev_dbg(&odev->intf->dev, "ch%u: DAA found %u device(s)\n",
			ch->channel_id, ndev);

		for (i = 0; i < ndev; i++) {
			u8 dyn_addr = resp[2 + i * 9];
			int ret;

			ret = i3c_master_add_i3c_dev_locked(master, dyn_addr);
			if (ret)
				dev_warn(&odev->intf->dev,
					 "ch%u: add dyn_addr=0x%02x failed: %d\n",
					 ch->channel_id, dyn_addr, ret);
		}
	}
	rv = 0;

out:
	kfree(req);
	kfree(resp);
	return rv;
}

/*
 * send_ccc_cmd - send one CCC command (broadcast or unicast).
 *
 * Builds an EXECUTE_SEQUENCE with one OBMF_I3C_OP_CCC op per dest.
 * For v1, if ndests > max_chained_ops we cap and warn; full batching
 * across multiple USB round-trips can be added in v2.
 *
 * The framework guarantees this is called with the bus lock held.
 */
static int obmf_i3c_send_ccc_cmd(struct i3c_master_controller *master,
				  struct i3c_ccc_cmd *cmd)
{
	struct obmf_i3c_priv *priv = to_i3c_priv(master);
	struct obmf_device   *odev = priv->odev;
	struct obmf_channel  *ch   = priv->ch;
	u8  *req, *resp;
	int  off, rv, i;
	u8   num_ops;
	const u8 *rops;
	int       rops_len;

	if (odev->disconnected)
		return -ENODEV;
	if (cmd->ndests == 0)
		return -EINVAL;

	num_ops = (u8)min_t(unsigned int, cmd->ndests, priv->max_chained_ops);
	if (num_ops < cmd->ndests)
		dev_warn_once(&odev->intf->dev,
			      "ch%u: CCC ndests=%u > max_ops=%u; truncating\n",
			      ch->channel_id, cmd->ndests, priv->max_chained_ops);

	req  = kzalloc(OBMF_I3C_BUF_SZ, GFP_KERNEL);
	resp = kzalloc(OBMF_I3C_BUF_SZ, GFP_KERNEL);
	if (!req || !resp) {
		rv = -ENOMEM;
		goto out;
	}

	req[0] = OBMF_I3C_CMD_EXECUTE_SEQ;
	req[1] = num_ops;
	off    = OBMF_I3C_REQ_PFX_SZ;

	for (i = 0; i < num_ops; i++) {
		struct i3c_ccc_cmd_dest *dest       = &cmd->dests[i];
		bool                     is_last    = (i == num_ops - 1);
		bool                     is_read    = !!cmd->rnw;
		u16                      payload_len = (u16)dest->payload.len;

		/*
		 * payload_len is passed for both read and write ops:
		 *   - write: tells the SMC how many bytes follow as write data
		 *   - read:  tells the SMC how many bytes to clock back
		 * The write_data bytes are only appended to the descriptor
		 * for write ops (obmf_i3c_append_op skips them when rnw=true).
		 */
		off = obmf_i3c_append_op(req, off, OBMF_I3C_BUF_SZ,
					  OBMF_I3C_OP_CCC,
					  is_last ? 1 : 0,
					  dest->addr, cmd->id,
					  is_read,
					  is_read ? NULL : dest->payload.data,
					  payload_len);
		if (off < 0) {
			rv = off;
			goto out;
		}
	}

	dev_dbg(&odev->intf->dev,
		"ch%u: CCC cmd=0x%02x ndests=%u rnw=%d\n",
		ch->channel_id, cmd->id, num_ops, !!cmd->rnw);

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch, OBMF_TYPE_I3C,
			       req, off,
			       resp, OBMF_I3C_BUF_SZ,
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);
	if (rv < 0)
		goto out;

	if (rv < 2) {
		rv = -EPROTO;
		goto out;
	}

	rops     = resp + 2;
	rops_len = rv - 2;

	for (i = 0; i < num_ops; i++) {
		struct i3c_ccc_cmd_dest *dest    = &cmd->dests[i];
		bool                     is_read = !!cmd->rnw;
		int                      consumed;
		u16                      actual_len = 0;

		consumed = obmf_i3c_parse_op_result(rops, rops_len,
						     is_read,
						     is_read ? dest->payload.data : NULL,
						     is_read ? (u16)dest->payload.len : 0,
						     &cmd->err, &actual_len);
		if (consumed < 0) {
			rv = consumed;
			goto out;
		}
		/* Update to the actual number of bytes the SMC reported. */
		dest->payload.len = actual_len;
		rops     += consumed;
		rops_len -= consumed;
	}
	rv = 0;

out:
	kfree(req);
	kfree(resp);
	return rv;
}

/*
 * priv_xfers - perform private SDR transfers.
 *
 * Each i3c_priv_xfer maps to one OBMF_I3C_OP_PRIVATE_SDR op.
 * All ops are chained in a single EXECUTE_SEQUENCE (up to max_chained_ops).
 * Large nxfers that exceed max_chained_ops are sent as multiple requests.
 */
static int obmf_i3c_priv_xfers(struct i3c_dev_desc *dev,
				struct i3c_priv_xfer *xfers, int nxfers)
{
	struct i3c_master_controller *master   = i3c_dev_get_master(dev);
	struct obmf_i3c_priv         *priv     = to_i3c_priv(master);
	struct obmf_device           *odev     = priv->odev;
	struct obmf_channel          *ch       = priv->ch;
	u8   dyn_addr = dev->info.dyn_addr;
	u8  *req, *resp;
	int  rv       = 0;
	int  sent     = 0;	/* xfers dispatched so far */

	if (odev->disconnected)
		return -ENODEV;
	if (nxfers <= 0)
		return 0;

	req  = kzalloc(OBMF_I3C_BUF_SZ, GFP_KERNEL);
	resp = kzalloc(OBMF_I3C_BUF_SZ, GFP_KERNEL);
	if (!req || !resp) {
		rv = -ENOMEM;
		goto out;
	}

	while (sent < nxfers) {
		int  batch   = min_t(int, nxfers - sent, priv->max_chained_ops);
		int  off     = OBMF_I3C_REQ_PFX_SZ;
		const u8 *rops;
		int       rops_len;

		memset(req, 0, OBMF_I3C_REQ_PFX_SZ);
		req[0] = OBMF_I3C_CMD_EXECUTE_SEQ;
		req[1] = (u8)batch;

		for (int i = 0; i < batch; i++) {
			struct i3c_priv_xfer *x       = &xfers[sent + i];
			bool                  is_last = (i == batch - 1);
			bool                  is_read = !!x->rnw;

			off = obmf_i3c_append_op(req, off, OBMF_I3C_BUF_SZ,
						  OBMF_I3C_OP_PRIVATE_SDR,
						  is_last ? 1 : 0,
						  dyn_addr, 0,
						  is_read,
						  is_read ? NULL : x->data.out,
						  x->len);
			if (off < 0) {
				rv = off;
				goto out;
			}
		}

		mutex_lock(&ch->lock);
		rv = obmf_send_request(odev, ch, OBMF_TYPE_I3C,
				       req, off,
				       resp, OBMF_I3C_BUF_SZ,
				       OBMF_DEFAULT_TIMEOUT_MS);
		mutex_unlock(&ch->lock);
		if (rv < 0)
			goto out;

		if (rv < 2) {
			rv = -EPROTO;
			goto out;
		}

		rops     = resp + 2;
		rops_len = rv - 2;

		for (int i = 0; i < batch; i++) {
			struct i3c_priv_xfer *x       = &xfers[sent + i];
			bool                  is_read = !!x->rnw;
			int                   consumed;
			u16                   actual_len = 0;

			consumed = obmf_i3c_parse_op_result(
						rops, rops_len,
						is_read,
						is_read ? x->data.in : NULL,
						is_read ? x->len : 0,
						NULL, &actual_len);
			if (consumed < 0) {
				rv = consumed;
				goto out;
			}
			/*
			 * Update x->len to the actual number of bytes
			 * transferred.  i3c_priv_xfer.len is an in/out field:
			 * client drivers (e.g. mctp-i3c.c) use the post-transfer
			 * value to size/trim their receive buffer.  Without this,
			 * reads always report the originally requested length,
			 * causing callers to read past the real data into
			 * uninitialized memory (e.g. failing PEC/CRC checks and
			 * silently dropping the packet).
			 */
			x->len = actual_len;
			rops     += consumed;
			rops_len -= consumed;
		}

		sent += batch;
		rv    = 0;
	}

out:
	kfree(req);
	kfree(resp);
	return rv;
}

/*
 * i2c_xfers - I2C compat transfers on a mixed I3C bus.
 *
 * Delegates to the associated OBMF I2C channel adapter so the I2C
 * subsystem handles all the framing.  Returns -ENOTSUPP if no I2C
 * channel was associated during bus_init.
 */
static int obmf_i3c_i2c_xfers(struct i2c_dev_desc *dev,
			       const struct i2c_msg *msgs, int nxfers)
{
	struct i3c_master_controller *master = i2c_dev_get_master(dev);
	struct obmf_i3c_priv         *priv   = to_i3c_priv(master);
	struct i2c_adapter           *adap;

	if (!priv->assoc_i2c_ch || !priv->assoc_i2c_ch->priv)
		return -ENOTSUPP;

	adap = priv->assoc_i2c_ch->priv;
	return i2c_transfer(adap, (struct i2c_msg *)msgs, nxfers);
}

/* ------------------------------------------------------------------ */
/* Per-device data                                                     */
/* ------------------------------------------------------------------ */

struct obmf_i3c_dev_data {
	struct i3c_generic_ibi_pool *ibi_pool;
};

static int obmf_i3c_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct obmf_i3c_dev_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i3c_dev_set_master_data(dev, data);
	return 0;
}

static void obmf_i3c_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct obmf_i3c_dev_data *data = i3c_dev_get_master_data(dev);

	kfree(data);
	i3c_dev_set_master_data(dev, NULL);
}

/* ------------------------------------------------------------------ */
/* IBI ops                                                             */
/* ------------------------------------------------------------------ */

static int obmf_i3c_request_ibi(struct i3c_dev_desc *dev,
				const struct i3c_ibi_setup *req)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct obmf_i3c_priv *priv = to_i3c_priv(m);
	struct obmf_i3c_dev_data *data = i3c_dev_get_master_data(dev);

	if (dev->ibi->max_payload_len > priv->max_ibi_payload) {
		dev_err(&priv->odev->intf->dev,
			"ch%u: IBI payload %u exceeds hw max %u\n",
			priv->ch->channel_id,
			dev->ibi->max_payload_len, priv->max_ibi_payload);
		return -ERANGE;
	}

	data->ibi_pool = i3c_generic_ibi_alloc_pool(dev, req);
	return PTR_ERR_OR_ZERO(data->ibi_pool);
}

static void obmf_i3c_free_ibi(struct i3c_dev_desc *dev)
{
	struct obmf_i3c_dev_data *data = i3c_dev_get_master_data(dev);

	i3c_generic_ibi_free_pool(data->ibi_pool);
	data->ibi_pool = NULL;
}

static int obmf_i3c_enable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);

	return i3c_master_enec_locked(m, dev->info.dyn_addr,
				      I3C_CCC_EVENT_SIR);
}

static int obmf_i3c_disable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct obmf_i3c_priv *priv = to_i3c_priv(m);

	/*
	 * USB already gone: there is no bus left to send DISEC on, and the
	 * device is gone too, so treat IBI as already disabled rather than
	 * failing, which would leave dev->ibi->enabled set and trip the
	 * WARN_ON()s in i3c_dev_free_ibi_locked() during device teardown.
	 */
	if (priv->odev->disconnected)
		return 0;

	return i3c_master_disec_locked(m, dev->info.dyn_addr,
				       I3C_CCC_EVENT_SIR);
}

static void obmf_i3c_recycle_ibi_slot(struct i3c_dev_desc *dev,
				      struct i3c_ibi_slot *slot)
{
	struct obmf_i3c_dev_data *data = i3c_dev_get_master_data(dev);

	i3c_generic_ibi_recycle_slot(data->ibi_pool, slot);
}

/* ------------------------------------------------------------------ */
/* Hot-Join deferred work                                              */
/* ------------------------------------------------------------------ */

static void obmf_i3c_hotjoin_work_fn(struct work_struct *work)
{
	struct obmf_i3c_priv *priv =
		container_of(work, struct obmf_i3c_priv, hotjoin_work);

	if (priv->odev->disconnected)
		return;

	dev_dbg(&priv->odev->intf->dev,
		"ch%u: Hot-Join: starting DAA\n", priv->ch->channel_id);
	i3c_master_do_daa(&priv->master);
}

/* ------------------------------------------------------------------ */
/* ops table                                                           */
/* ------------------------------------------------------------------ */

static const struct i3c_master_controller_ops obmf_i3c_ops = {
	.bus_init          = obmf_i3c_bus_init,
	.attach_i3c_dev    = obmf_i3c_attach_i3c_dev,
	.detach_i3c_dev    = obmf_i3c_detach_i3c_dev,
	.do_daa            = obmf_i3c_do_daa,
	.send_ccc_cmd      = obmf_i3c_send_ccc_cmd,
	.priv_xfers        = obmf_i3c_priv_xfers,
	.i2c_xfers         = obmf_i3c_i2c_xfers,
	.request_ibi       = obmf_i3c_request_ibi,
	.free_ibi          = obmf_i3c_free_ibi,
	.enable_ibi        = obmf_i3c_enable_ibi,
	.disable_ibi       = obmf_i3c_disable_ibi,
	.recycle_ibi_slot  = obmf_i3c_recycle_ibi_slot,
};

/* ------------------------------------------------------------------ */
/* Device-initiated event demux (called from transport RX)            */
/* ------------------------------------------------------------------ */

/**
 * obmf_i3c_handle_dev_request - dispatch SMC-initiated I3C events.
 * @ch:   the I3C channel
 * @data: payload starting at the command byte (byte 0)
 * @len:  total payload length
 *
 * Invoked by the transport demux when the SMC sends a device-initiated
 * request on an I3C channel (typically via Interrupt IN).
 */
void obmf_i3c_handle_dev_request(struct obmf_channel *ch,
				  const u8 *data, int len)
{
	struct obmf_i3c_priv *priv = ch->priv;
	u8 resp[2];
	u8 ack_status = OBMF_STATUS_SUCCESS;

	if (!priv || len < 1)
		return;

	switch (data[0]) {
	case OBMF_I3C_EVT_IBI: {
		/*
		 * IBI notification from SMC (producer → BMC).
		 *
		 * Byte 0: OBMF_I3C_EVT_IBI
		 * Byte 1: dynamic_addr of triggering device
		 * Byte 2-3: payload_len (LE)
		 * Byte 4-5: pending_read (LE, unused)
		 * Byte 6+: payload_data
		 */
		struct i3c_dev_desc *desc = NULL, *tmp;
		struct obmf_i3c_dev_data *devdata;
		struct i3c_ibi_slot *slot;
		u8 dyn_addr;
		u16 payload_len;
		unsigned int copy_len;

		if (len < 6)
			break;

		dyn_addr    = data[1];
		payload_len = get_unaligned_le16(&data[2]);

		dev_dbg(&priv->odev->intf->dev,
				 "ch%u: IBI from 0x%02x\n",
				 ch->channel_id, dyn_addr);

		down_read(&priv->master.bus.lock);
		i3c_bus_for_each_i3cdev(&priv->master.bus, tmp) {
			if (tmp->info.dyn_addr == dyn_addr) {
				desc = tmp;
				break;
			}
		}
		up_read(&priv->master.bus.lock);

		if (!desc || !desc->ibi)
			break;

		devdata = i3c_dev_get_master_data(desc);
		if (!devdata || !devdata->ibi_pool)
			break;

		slot = i3c_generic_ibi_get_free_slot(devdata->ibi_pool);
		if (!slot) {
			dev_warn_ratelimited(&priv->odev->intf->dev,
					 "ch%u: IBI from 0x%02x: no free slot\n",
					 ch->channel_id, dyn_addr);
			break;
		}

		copy_len = min_t(unsigned int, payload_len,
				 desc->ibi->max_payload_len);
		slot->len = min_t(unsigned int, copy_len,
				  (unsigned int)(len - 6));
		if (slot->len && slot->data)
			memcpy(slot->data, &data[6], slot->len);

		i3c_master_queue_ibi(desc, slot);
		break;
	}

	case OBMF_I3C_EVT_HOTJOIN:
		/*
		 * Hot-Join: a new I3C device appeared on the bus.
		 * i3c_master_do_daa() must be called from a sleepable
		 * context, so we queue it on the framework's workqueue.
		 */
		dev_dbg(&priv->odev->intf->dev,
			"ch%u: Hot-Join received\n", ch->channel_id);
		queue_work(priv->master.wq, &priv->hotjoin_work);
		break;

	case OBMF_I3C_EVT_BUS_ERROR:
		/*
		 * Byte 1: error_code
		 *   1 = Line Lockup
		 *   2 = Arbitration Loss
		 */
		dev_warn_ratelimited(&priv->odev->intf->dev,
				     "ch%u: Bus Error event code=0x%02x\n",
				     ch->channel_id,
				     len >= 2 ? data[1] : 0);
		break;

	default:
		dev_dbg(&priv->odev->intf->dev,
			"ch%u: unknown dev request cmd=0x%02x\n",
			ch->channel_id, data[0]);
		ack_status = OBMF_STATUS_INVALID_CMD;
		break;
	}

	/*
	 * Every device-initiated request on this channel MUST be ACKed so
	 * the SMC's outstanding-request tag (channel_tags[].is_outstanding)
	 * is cleared and the next Producer-initiated event can be sent.
	 * Without this ACK, the SMC's obmf_send_request() keeps returning
	 * -EBUSY for this channel after the very first event.
	 *
	 * Response payload per §4.8.2: Byte 0 = command echo, Byte 1 =
	 * result_count (reserved/0 for producer-initiated events).
	 */
	resp[0] = data[0];
	resp[1] = 0;
	obmf_send_response(priv->odev, ch->channel_id, OBMF_TYPE_I3C,
			   ack_status, resp, sizeof(resp));
}

/* ------------------------------------------------------------------ */
/* Associated I2C channel search                                       */
/* ------------------------------------------------------------------ */

/*
 * Scan sibling channels for the first I2C channel with a registered
 * adapter.  A future version could use a DT phandle or a vendor config
 * register to make the pairing explicit.
 */
static struct obmf_channel *
obmf_i3c_find_assoc_i2c(struct obmf_device *odev)
{
	int i;

	for (i = 1; i < odev->num_channels; i++) {
		struct obmf_channel *c = &odev->channels[i];

		if (c->channel_type == OBMF_TYPE_I2C && c->priv)
			return c;
	}
	return NULL;
}

/* ------------------------------------------------------------------ */
/* Register / Unregister                                               */
/* ------------------------------------------------------------------ */

int obmf_i3c_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_i3c_priv *priv;
	int rv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->odev         = odev;
	priv->ch           = ch;
	priv->assoc_i2c_ch = obmf_i3c_find_assoc_i2c(odev);

	INIT_WORK(&priv->hotjoin_work, obmf_i3c_hotjoin_work_fn);

	/*
	 * i3c_master_register() takes ownership of master.dev.of_node on
	 * both success and failure: i3c_masterdev_release() (dev->release)
	 * puts it when the device's refcount drops to zero, so we must not
	 * put it again here.
	 */
	rv = i3c_master_register(&priv->master, &odev->intf->dev,
				  &obmf_i3c_ops, false /* primary master */);
	if (rv) {
		kfree(priv);
		return rv;
	}

	ch->priv      = priv;
	ch->sysfs_dev = &priv->master.dev;

	dev_info(&odev->intf->dev,
		 "ch%u: registered I3C master (bus-%d)%s\n",
		 ch->channel_id, priv->master.bus.id,
		 priv->assoc_i2c_ch ? " with assoc I2C" : "");
	return 0;
}

void obmf_i3c_unregister(struct obmf_channel *ch)
{
	struct obmf_i3c_priv *priv = ch->priv;

	if (!priv)
		return;

	/*
	 * Cancel Hot-Join work before tearing down the master.  If hotjoin
	 * work is queued it could call i3c_master_do_daa() after the master
	 * is gone.
	 */
	cancel_work_sync(&priv->hotjoin_work);

	/* i3c_masterdev_release() puts master.dev.of_node; don't double-put. */
	i3c_master_unregister(&priv->master);

	kfree(priv);
	ch->priv      = NULL;
	ch->sysfs_dev = NULL;
}
