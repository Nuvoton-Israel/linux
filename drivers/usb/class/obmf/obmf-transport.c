// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-transport.c - OBMF-ICP USB transport layer
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * TX mux: subsystem callback -> build Common Header -> usb_bulk_msg()
 * RX demux: Bulk IN URB callback -> parse channel_id -> complete()
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

/* ------------------------------------------------------------------ */
/* obmf_find_channel — look up channel by channel_id (linear search)   */
/* ------------------------------------------------------------------ */

static struct obmf_channel *obmf_find_channel(struct obmf_device *odev,
					      u8 channel_id)
{
	int i;

	for (i = 0; i < odev->num_channels; i++) {
		if (odev->channels[i].channel_id == channel_id)
			return &odev->channels[i];
	}
	return NULL;
}

/*
 * Give up on the outstanding request of @ch: a response that still arrives
 * is dropped instead of being copied into the requester's buffer.
 */
static void obmf_stop_waiting(struct obmf_channel *ch)
{
	unsigned long flags;

	spin_lock_irqsave(&ch->resp_lock, flags);
	ch->waiting  = false;
	ch->resp_buf = NULL;
	spin_unlock_irqrestore(&ch->resp_lock, flags);
}

/* Allocate a work item with an inline variable-length payload buffer. */
static struct obmf_dev_req *obmf_alloc_dev_req(int payload_len)
{
	if (payload_len < 0)
		return NULL;

	return kmalloc(struct_size((struct obmf_dev_req *)NULL, data,
				   payload_len), GFP_ATOMIC);
}

/* ------------------------------------------------------------------ */
/* obmf_send_response — send a Response message (Host as Responder)     */
/*                                                                     */
/* Can be called from process context (workqueue).  Uses tx_lock to    */
/* serialise with obmf_send_request().                                 */
/* ------------------------------------------------------------------ */

int obmf_send_response(struct obmf_device *odev, u8 channel_id,
		       u8 status,
		       const void *payload, int payload_len)
{
	struct obmf_common_hdr *hdr;
	int seg_data_max, offset = 0, actual_len, rv = 0;

	if (odev->disconnected)
		return -ENODEV;

	/*
	 * Segment the response into USB-packet-sized chunks so the device
	 * always sees a complete OBMF Common Header at the start of each
	 * USB packet.  The *first* packet's hdr->size carries the total
	 * payload length so the device knows how many bytes to accumulate;
	 * each subsequent segment's hdr->size carries the length of that
	 * segment's chunk only.
	 */
	seg_data_max = odev->max_wr_transfer_size - OBMF_COMMON_HDR_SIZE;
	if (seg_data_max <= 0)
		return -EINVAL;

	mutex_lock(&odev->tx_lock);

	hdr = (struct obmf_common_hdr *)odev->tx_buf;
	hdr->channel      = channel_id;
	OBMF_HDR_SET_RESPONSE(hdr, status);

	do {
		int chunk = min(payload_len - offset, seg_data_max);

		/* First packet advertises the total; later packets the chunk */
		hdr->size = cpu_to_le16(offset == 0 ? payload_len : chunk);

		if (chunk > 0)
			memcpy(odev->tx_buf + OBMF_COMMON_HDR_SIZE,
			       (const u8 *)payload + offset, chunk);

		rv = usb_bulk_msg(odev->udev,
				  usb_sndbulkpipe(odev->udev, odev->bulk_out_ep),
				  odev->tx_buf, OBMF_COMMON_HDR_SIZE + chunk,
				  &actual_len, OBMF_DEFAULT_TIMEOUT_MS);
		if (rv) {
			dev_err(&odev->intf->dev,
				"send response ch%u seg@%d failed: %d\n",
				channel_id, offset, rv);
			break;
		}
		offset += chunk;
	} while (offset < payload_len);

	mutex_unlock(&odev->tx_lock);

	return rv;
}

/* ------------------------------------------------------------------ */
/* Device-initiated request handler (runs in workqueue context)        */
/* ------------------------------------------------------------------ */

static void obmf_dev_request_work(struct work_struct *work)
{
	struct obmf_dev_req *dreq = container_of(work, struct obmf_dev_req, work);
	struct obmf_device *odev = dreq->odev;
	struct obmf_channel *ch;

	if (odev->disconnected)
		goto out;

	/*
	 * The Host never exposes Channel 0 (Table 1, requirement 28), and an
	 * unknown channel has nothing to serve it: answer, don't drop.
	 */
	ch = obmf_find_channel(odev, dreq->channel_id);
	if (!ch || dreq->channel_id == 0) {
		obmf_send_response(odev, dreq->channel_id,
				   OBMF_STATUS_UNKNOWN_CHANNEL, NULL, 0);
		goto out;
	}

	/*
	 * The channel is enabled before its subsystem is set up, so the device
	 * can already send requests; the handlers need ch->priv.  A channel
	 * whose registration failed stays unserved.
	 */
	if (!smp_load_acquire(&ch->registered)) {
		obmf_send_response(odev, dreq->channel_id,
				   OBMF_STATUS_NOT_READY, NULL, 0);
		goto out;
	}

	switch (dreq->channel_type) {
	case OBMF_TYPE_MMIO:
	case OBMF_TYPE_CONFIG: {
		/*
		 * MMIO device-initiated request.
		 * Short transactions use 32-bit address, Long transactions
		 * use 64-bit address; reads add a u16 Read Length, writes
		 * carry the data with no length field.
		 * Dispatch to MMIO misc handler.
		 */
		obmf_mmio_handle_dev_request(ch, dreq->transaction,
					     dreq->data, dreq->data_len);
		break;
	}
	case OBMF_TYPE_IO:
		/*
		 * I/O Port Channel device-initiated request (v0.9.2).
		 * Sub-header already stripped; dreq->data contains:
		 *   [0-1]: port_addr (LE u16)
		 *   [2]:   size (u8)
		 *   [3+]:  data bytes
		 * Fast-path for POST codes (port 0x0080) is in the misc handler.
		 */
		obmf_io_handle_dev_request(ch, dreq->transaction,
					   dreq->data, dreq->data_len);
		break;
	case OBMF_TYPE_GPIO:
		obmf_gpio_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	case OBMF_TYPE_SERIAL: {
		obmf_serial_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	}
	case OBMF_TYPE_SPI:
		obmf_spi_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	case OBMF_TYPE_MEM_DEV:
		obmf_mem_dev_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	case OBMF_TYPE_I2C_TARGET:
		obmf_i2c_target_handle_dev_request(ch, dreq->data,
						   dreq->data_len);
		break;
	case OBMF_TYPE_IPMI:
		obmf_ipmi_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	case OBMF_TYPE_I3C:
		/*
		 * I3C Controller Channel device-initiated request (§4.8):
		 * IBI / Hot-Join / Bus-Error notifications from the SMC.
		 */
		obmf_i3c_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	default:
		if (dreq->channel_type >= OBMF_TYPE_OEM_MIN &&
		    dreq->channel_type <= OBMF_TYPE_OEM_MAX) {
			obmf_oem_handle_dev_request(ch, dreq->data,
						    dreq->data_len);
		} else {
			obmf_send_response(odev, dreq->channel_id,
					   OBMF_STATUS_INVALID_CMD,
					   NULL, 0);
		}
		break;
	}

out:
	kfree(dreq);
}

/* ------------------------------------------------------------------ */
/* obmf_dispatch_message — process a fully reassembled OBMF message     */
/*                                                                     */
/* Called from URB callback context (obmf_rx_complete).  The payload    */
/* pointer and payload_len refer to the data AFTER the Common Header.  */
/* ------------------------------------------------------------------ */

/*
 * Map a response Status (spec §5.2) to an errno.  0x00-0x3F are the generic
 * codes of Table 4; 0x40-0x7F are defined per channel type, so the same
 * value means different things on different channels.  Errnos follow the
 * conventions of the subsystem each channel is exposed through (e.g.
 * Documentation/i2c/fault-codes.rst for I2C).
 */
static int obmf_status_to_errno(u8 channel_type, u8 status)
{
	switch (status) {
	case OBMF_STATUS_INVALID_CMD:		return -EOPNOTSUPP;
	case OBMF_STATUS_TIMEOUT:		return -ETIMEDOUT;
	case OBMF_STATUS_NOT_READY:		return -EAGAIN;
	case OBMF_STATUS_PERMANENT_ERROR:	return -EIO;
	case OBMF_STATUS_UNKNOWN_CHANNEL:	return -ENXIO;
	case OBMF_STATUS_SIZE_NOT_SUPPORTED:	return -EMSGSIZE;
	}

	switch (channel_type) {
	case OBMF_TYPE_CONFIG:
	case OBMF_TYPE_MMIO:
	case OBMF_TYPE_IO:
		/* Tables 7 and 54 */
		switch (status) {
		case OBMF_MMIO_STATUS_ADDR_OUT_OF_RANGE: return -EFAULT;
		case OBMF_MMIO_STATUS_ACCESS_DENIED:	 return -EACCES;
		}
		break;
	case OBMF_TYPE_SPI:
		/* Table 49 */
		switch (status) {
		case OBMF_SPI_STATUS_MODE_UNSUPPORTED:	return -EOPNOTSUPP;
		case OBMF_SPI_STATUS_TRANSFER_ERROR:	return -EIO;
		}
		break;
	case OBMF_TYPE_GPIO:
		/* Table 17 */
		switch (status) {
		case OBMF_GPIO_STATUS_INDEX_NOT_SUPPORTED: return -EINVAL;
		case OBMF_GPIO_STATUS_INT_NOT_SUPPORTED:   return -EOPNOTSUPP;
		case OBMF_GPIO_STATUS_INVALID_OPERATION:   return -EPERM;
		}
		break;
	case OBMF_TYPE_SERIAL:
		/* Table 21 */
		if (status == OBMF_SERIAL_STATUS_LINE_TIMEOUT)
			return -ETIMEDOUT;
		break;
	case OBMF_TYPE_I2C:
		/* Table 25 */
		switch (status) {
		case OBMF_I2C_STATUS_TIMEOUT:	return -ETIMEDOUT;
		case OBMF_I2C_STATUS_ARB_LOST:	return -EAGAIN;
		case OBMF_I2C_STATUS_BUS_BUSY:	return -EBUSY;
		case OBMF_I2C_STATUS_NACK:	return -ENXIO;
		}
		break;
	case OBMF_TYPE_I3C:
		/* Table 34 */
		switch (status) {
		case OBMF_I3C_STATUS_BUS_TIMEOUT: return -ETIMEDOUT;
		case OBMF_I3C_STATUS_ARB_LOST:	  return -EAGAIN;
		case OBMF_I3C_STATUS_BUS_BUSY:	  return -EBUSY;
		case OBMF_I3C_STATUS_NACK:	  return -ENXIO;
		case OBMF_I3C_STATUS_ILLEGAL_CCC: return -EPROTO;
		}
		break;
	}

	/* Bus/framing/parity faults, I2C Target errors, reserved codes */
	return -EIO;
}

/*
 * Whether an MMIO / Channel 0 response can answer the outstanding request.
 * The response header echoes the request's Transaction (Table 5), and a
 * successful read carries exactly Read Length bytes while a write carries
 * none (Tables 9, 11, 13, 15).  A response that fails either test belongs
 * to an earlier request that timed out, or that a previous driver instance
 * left in flight.  Called with ch->resp_lock held.
 */
static bool obmf_mmio_rsp_matches(struct obmf_channel *ch, const u8 *payload,
				  int payload_len, bool success)
{
	if (ch->expect_trans == OBMF_EXPECT_ANY)
		return true;
	if (payload_len < OBMF_MMIO_SUBHDR_SIZE)
		return !success;	/* an error may come without a header */
	if ((payload[0] & 0x07) != ch->expect_trans)
		return false;
	return !success ||
	       payload_len - (int)OBMF_MMIO_SUBHDR_SIZE == ch->expect_len;
}

static void obmf_dispatch_message(struct obmf_device *odev,
				  struct obmf_common_hdr *hdr,
				  const u8 *payload, int payload_len)
{
	struct obmf_channel *ch;
	u8 channel_id = hdr->channel;

	ch = obmf_find_channel(odev, channel_id);
	if (!ch || (channel_id == 0 && !OBMF_HDR_IS_RESPONSE(hdr))) {
		struct obmf_dev_req *dreq;

		dev_err(&odev->intf->dev, "rx: %s on unknown channel %u\n",
			OBMF_HDR_IS_RESPONSE(hdr) ? "response" : "request",
			channel_id);
		if (OBMF_HDR_IS_RESPONSE(hdr))
			return;

		/* Let the work item answer it; we can't send from here */
		dreq = obmf_alloc_dev_req(0);
		if (!dreq)
			return;
		INIT_WORK(&dreq->work, obmf_dev_request_work);
		dreq->odev = odev;
		dreq->channel_id = channel_id;
		dreq->channel_type = OBMF_TYPE_CONFIG;
		dreq->transaction = 0;
		dreq->data_len = 0;
		queue_work(odev->dev_req_wq, &dreq->work);
		return;
	}

	if (OBMF_HDR_IS_RESPONSE(hdr)) {
		/* Response to our earlier request */
		int copy_len;
		u8 hdr_status = OBMF_HDR_STATUS(hdr);
		unsigned long flags;

		/*
		 * The requester may have timed out and returned, taking its
		 * resp_buf with it: only a waiting requester gets the response.
		 */
		spin_lock_irqsave(&ch->resp_lock, flags);
		if (!ch->waiting) {
			spin_unlock_irqrestore(&ch->resp_lock, flags);
			dev_warn_ratelimited(&odev->intf->dev,
					     "ch%u: response with no request waiting, dropped\n",
					     channel_id);
			return;
		}
		if (!obmf_mmio_rsp_matches(ch, payload, payload_len,
					   hdr_status == OBMF_STATUS_SUCCESS)) {
			spin_unlock_irqrestore(&ch->resp_lock, flags);
			dev_warn_ratelimited(&odev->intf->dev,
					     "ch%u: response does not answer the outstanding request, dropped\n",
					     channel_id);
			return;
		}
		ch->waiting = false;

		/* Check response status from header byte 2[7:1] */
		if (hdr_status != OBMF_STATUS_SUCCESS) {
			dev_dbg(&odev->intf->dev,
				"ch%u: response status 0x%02x\n",
				channel_id, hdr_status);
			ch->status = obmf_status_to_errno(ch->channel_type,
							  hdr_status);
			complete(&ch->done);
			spin_unlock_irqrestore(&ch->resp_lock, flags);
			return;
		}

		/* IO: skip the 1-byte header */
		if (ch->channel_type == OBMF_TYPE_IO &&
		    payload_len >= OBMF_IO_SUBHDR_SIZE) {
			/* Skip sub-header for payload copy */
			payload     += OBMF_IO_SUBHDR_SIZE;
			payload_len -= OBMF_IO_SUBHDR_SIZE;
		} else if ((ch->channel_type == OBMF_TYPE_MMIO ||
			    ch->channel_type == OBMF_TYPE_CONFIG) &&
			   payload_len >= OBMF_MMIO_SUBHDR_SIZE) {
			/* MMIO / CONFIG (Discovery): 2-byte header, no tag */
			payload     += OBMF_MMIO_SUBHDR_SIZE;
			payload_len -= OBMF_MMIO_SUBHDR_SIZE;
		}

		copy_len = min_t(int, payload_len, ch->resp_len);
		if (ch->resp_buf && copy_len > 0)
			memcpy(ch->resp_buf, payload, copy_len);
		ch->resp_len = copy_len;
		ch->status = 0;
		complete(&ch->done);
		spin_unlock_irqrestore(&ch->resp_lock, flags);
	} else {
		dev_dbg(&odev->intf->dev,
			"OBMF RX: channel=%u ch_type=0x%02x status_rqresp=0x%02x size=%u\n",
			channel_id, ch->channel_type, hdr->status_rqresp,
			payload_len);
		/*
		 * RqResp=0 on Bulk IN: device-initiated request.
		 * Host must respond (Host = Responder).
		 * All channel types are deferred to workqueue.
		 */
		if (ch->channel_type == OBMF_TYPE_IO &&
		    payload_len >= OBMF_IO_SUBHDR_SIZE) {
			struct obmf_io_subhdr *ihdr =
				(struct obmf_io_subhdr *)payload;
			u8 trans = ihdr->transaction & 0x0F;

			{
				struct obmf_dev_req *dreq;
				int req_payload_len;

				dev_dbg(&odev->intf->dev,
					"ch%u: IO dev-req (trans=%u)\n",
					channel_id, trans);

				req_payload_len = payload_len - OBMF_IO_SUBHDR_SIZE;
				dreq = obmf_alloc_dev_req(req_payload_len);
				if (dreq) {
					INIT_WORK(&dreq->work,
						  obmf_dev_request_work);
					dreq->odev = odev;
					dreq->channel_id = channel_id;
					dreq->channel_type = OBMF_TYPE_IO;
					dreq->transaction = trans;
					dreq->data_len = req_payload_len;
					if (dreq->data_len > 0)
						memcpy(dreq->data,
						       payload + OBMF_IO_SUBHDR_SIZE,
						       dreq->data_len);
					queue_work(odev->dev_req_wq, &dreq->work);
				} else {
					dev_err(&odev->intf->dev,
						"ch%u: alloc dev-req failed (len=%d)\n",
						channel_id, req_payload_len);
				}
			}
		} else if ((ch->channel_type == OBMF_TYPE_MMIO ||
			    ch->channel_type == OBMF_TYPE_CONFIG) &&
			   payload_len >= OBMF_MMIO_SUBHDR_SIZE) {
			struct obmf_mmio_subhdr *mhdr =
				(struct obmf_mmio_subhdr *)payload;
			u8 trans = mhdr->transaction & 0x07;

			{
				struct obmf_dev_req *dreq;
				int req_payload_len;

				dev_dbg(&odev->intf->dev,
					"ch%u: MMIO dev-req (trans=%u)\n",
					channel_id, trans);

				req_payload_len = payload_len - OBMF_MMIO_SUBHDR_SIZE;
				dreq = obmf_alloc_dev_req(req_payload_len);
				if (dreq) {
					INIT_WORK(&dreq->work,
						  obmf_dev_request_work);
					dreq->odev = odev;
					dreq->channel_id = channel_id;
					dreq->channel_type = OBMF_TYPE_MMIO;
					dreq->transaction = trans;
					dreq->data_len = req_payload_len;
					if (dreq->data_len > 0)
						memcpy(dreq->data,
						       payload + OBMF_MMIO_SUBHDR_SIZE,
						       dreq->data_len);
					queue_work(odev->dev_req_wq, &dreq->work);
				} else {
					dev_err(&odev->intf->dev,
						"ch%u: alloc dev-req failed (len=%d)\n",
						channel_id, req_payload_len);
				}
			}
		} else {
			/*
			 * Optimised channel device-initiated request.
			 * Defer to workqueue for response handling.
			 */
			struct obmf_dev_req *dreq;

			dreq = obmf_alloc_dev_req(payload_len);
			if (dreq) {
				INIT_WORK(&dreq->work, obmf_dev_request_work);
				dreq->odev = odev;
				dreq->channel_id = channel_id;
				dreq->channel_type = ch->channel_type;
				dreq->transaction = 0;
				dreq->data_len = payload_len;
				if (dreq->data_len > 0)
					memcpy(dreq->data, payload,
					       dreq->data_len);
				queue_work(odev->dev_req_wq, &dreq->work);
			} else {
				dev_err(&odev->intf->dev,
					"ch%u: alloc dev-req failed (len=%d)\n",
					channel_id, payload_len);
			}
		}
	}
}

/* ------------------------------------------------------------------ */
/* RX Bulk IN callback — demux incoming responses/notifications        */
/*                                                                     */
/* Handles segment reassembly: if hdr->size > payload in this URB,     */
/* accumulate segments in the channel's reasm_buf until complete, then  */
/* received, then dispatch via obmf_dispatch_message().                 */
/* ------------------------------------------------------------------ */

static void obmf_reasm_reset(struct obmf_channel *ch)
{
	kfree(ch->reasm_buf);
	ch->reasm_buf = NULL;
	ch->reasm_total = 0;
	ch->reasm_offset = 0;
	ch->reasm_active = false;
}

static void obmf_rx_complete(struct urb *urb)
{
	struct obmf_device *odev = urb->context;
	struct obmf_common_hdr *hdr;
	struct obmf_channel *ch;
	u16 total_payload;
	int actual_payload, chunk;
	int status = urb->status;

	switch (status) {
	case 0:
		odev->bulk_in_stall_count = 0;
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	case -EPIPE:
		odev->bulk_in_stall_count++;
		if (odev->bulk_in_stall_count >= OBMF_STALL_THRESHOLD)
			set_bit(OBMF_STALL_RESET_PENDING, &odev->stall_flags);
		else
			set_bit(OBMF_STALL_BULK_IN, &odev->stall_flags);
		schedule_work(&odev->stall_work);
		goto resubmit;
	default:
		dev_err(&odev->intf->dev, "rx urb failed: %d\n", status);
		goto resubmit;
	}

	if (urb->actual_length < OBMF_COMMON_HDR_SIZE) {
		dev_dbg(&odev->intf->dev, "short rx: %d bytes\n",
			urb->actual_length);
		goto resubmit;
	}

	hdr = (struct obmf_common_hdr *)odev->rx_buf;
	ch = obmf_find_channel(odev, hdr->channel);
	if (!ch) {
		/*
		 * No channel to hold reassembly state, so follow the Size
		 * field here and let the dispatcher answer a request once,
		 * when its last fragment arrives.
		 */
		actual_payload = urb->actual_length - OBMF_COMMON_HDR_SIZE;
		if (odev->unk_active &&
		    odev->unk_channel == hdr->channel &&
		    odev->unk_status_rqresp == hdr->status_rqresp) {
			odev->unk_remaining -= actual_payload;
		} else {
			odev->unk_remaining = le16_to_cpu(hdr->size) -
					      actual_payload;
			odev->unk_channel = hdr->channel;
			odev->unk_status_rqresp = hdr->status_rqresp;
		}
		if (odev->unk_remaining > 0) {
			odev->unk_active = true;
			goto resubmit;
		}
		odev->unk_active = false;

		obmf_dispatch_message(odev, hdr,
				      odev->rx_buf + OBMF_COMMON_HDR_SIZE,
				      actual_payload);
		goto resubmit;
	}

	if (ch->reasm_active) {
		/*
		 * Continuation segment — validate that channel and
		 * status/rqresp match the first segment, then append
		 * payload data.
		 */
		if (hdr->status_rqresp != ch->reasm_hdr.status_rqresp) {
			dev_err(&odev->intf->dev,
				"rx reasm: header mismatch on ch%u (status_rqresp got 0x%02x exp 0x%02x), reset\n",
				hdr->channel,
				hdr->status_rqresp,
				ch->reasm_hdr.status_rqresp);
			obmf_reasm_reset(ch);
			goto resubmit;
		}

		chunk = urb->actual_length - OBMF_COMMON_HDR_SIZE;
		if (chunk <= 0)
			goto resubmit;

		if (ch->reasm_offset + chunk > ch->reasm_total)
			chunk = ch->reasm_total - ch->reasm_offset;

		memcpy(ch->reasm_buf + ch->reasm_offset,
		       odev->rx_buf + OBMF_COMMON_HDR_SIZE, chunk);
		ch->reasm_offset += chunk;

		if (ch->reasm_offset >= ch->reasm_total) {
			/* Reassembly complete — dispatch full message */
			obmf_dispatch_message(odev, &ch->reasm_hdr,
					      ch->reasm_buf,
					      ch->reasm_total);
			obmf_reasm_reset(ch);
		}
		goto resubmit;
	}

	/* First (or only) segment */
	total_payload  = le16_to_cpu(hdr->size);
	actual_payload = urb->actual_length - OBMF_COMMON_HDR_SIZE;

	if (actual_payload >= total_payload) {
		/* Complete message in a single URB — fast path */
		obmf_dispatch_message(odev, hdr,
				      odev->rx_buf + OBMF_COMMON_HDR_SIZE,
				      total_payload);
	} else {
		/* First segment of a multi-segment message */
		ch->reasm_buf = kmalloc(total_payload, GFP_ATOMIC);
		if (!ch->reasm_buf) {
			dev_err(&odev->intf->dev,
				"rx: alloc reasm buffer failed on ch%u (len=%u)\n",
				hdr->channel, total_payload);
			goto resubmit;
		}

		ch->reasm_hdr    = *hdr;
		ch->reasm_total  = total_payload;
		ch->reasm_offset = actual_payload;
		ch->reasm_active = true;

		if (actual_payload > 0)
			memcpy(ch->reasm_buf,
			       odev->rx_buf + OBMF_COMMON_HDR_SIZE,
			       actual_payload);
	}

resubmit:
	if (!odev->disconnected) {
		status = usb_submit_urb(urb, GFP_ATOMIC);
		if (status && status != -EPERM && status != -ENODEV)
			dev_err(&odev->intf->dev,
				"rx resubmit failed: %d\n", status);
	}
}

/* ------------------------------------------------------------------ */
/* Transport init / exit                                               */
/* ------------------------------------------------------------------ */

/*
 * Transfers queued at the device before this driver started: responses to
 * requests that a previous driver instance left in flight, and requests the
 * device raised while no host driver was listening (e.g. POST codes).
 * Read them before discovery so that responses are not taken as answers to
 * its Channel 0 reads.  Responses are dropped; single-transfer requests on
 * a channel other than 0 are kept and answered by obmf_replay_stale_in()
 * once the channels are registered.
 */
static void obmf_drain_stale_in(struct obmf_device *odev)
{
	int i, len, dropped = 0, kept = 0;

	for (i = 0; i < OBMF_DRAIN_MAX; i++) {
		struct obmf_common_hdr *hdr = (struct obmf_common_hdr *)odev->rx_buf;
		struct obmf_stale_req *sr;

		if (usb_bulk_msg(odev->udev,
				 usb_rcvbulkpipe(odev->udev, odev->bulk_in_ep),
				 odev->rx_buf, odev->max_rd_transfer_size,
				 &len, OBMF_DRAIN_TIMEOUT_MS))
			break;

		if (len < OBMF_COMMON_HDR_SIZE || hdr->channel == 0 ||
		    OBMF_HDR_IS_RESPONSE(hdr) ||
		    len - OBMF_COMMON_HDR_SIZE != le16_to_cpu(hdr->size)) {
			dropped++;
			continue;
		}
		sr = kmalloc(struct_size(sr, data, len), GFP_KERNEL);
		if (!sr) {
			dropped++;
			continue;
		}
		sr->len = len;
		memcpy(sr->data, odev->rx_buf, len);
		list_add_tail(&sr->node, &odev->stale_reqs);
		kept++;
	}
	if (dropped || kept)
		dev_info(&odev->intf->dev,
			 "stale Bulk IN transfers: %d dropped, %d kept for replay\n",
			 dropped, kept);
}

static void obmf_free_stale_in(struct obmf_device *odev)
{
	struct obmf_stale_req *sr, *tmp;

	list_for_each_entry_safe(sr, tmp, &odev->stale_reqs, node) {
		list_del(&sr->node);
		kfree(sr);
	}
}

/* Dispatch the requests kept by obmf_drain_stale_in() */
void obmf_replay_stale_in(struct obmf_device *odev)
{
	struct obmf_stale_req *sr, *tmp;

	list_for_each_entry_safe(sr, tmp, &odev->stale_reqs, node) {
		obmf_dispatch_message(odev, (struct obmf_common_hdr *)sr->data,
				      sr->data + OBMF_COMMON_HDR_SIZE,
				      sr->len - OBMF_COMMON_HDR_SIZE);
		list_del(&sr->node);
		kfree(sr);
	}
}

int obmf_transport_init(struct obmf_device *odev)
{
	int rv;

	INIT_LIST_HEAD(&odev->stale_reqs);

	/* Workqueue for device-initiated requests */
	odev->dev_req_wq = alloc_workqueue("obmf_devreq", WQ_UNBOUND, 0);
	if (!odev->dev_req_wq)
		return -ENOMEM;

	/* Allocate TX buffer — sized by host_tx_size (BMC's effective TX
	 * capability, capped at device's max_wr_transfer_size during probe)
	 */
	odev->tx_buf = kmalloc(odev->host_tx_size, GFP_KERNEL);
	if (!odev->tx_buf) {
		rv = -ENOMEM;
		goto err_wq;
	}

	/* Allocate RX URB and buffer */
	odev->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!odev->rx_urb) {
		rv = -ENOMEM;
		goto err_tx;
	}

	odev->rx_buf = kmalloc(odev->host_rx_size, GFP_KERNEL);
	if (!odev->rx_buf) {
		rv = -ENOMEM;
		goto err_rx_urb;
	}

	obmf_drain_stale_in(odev);

	/* Fill and submit continuous Bulk IN URB */
	usb_fill_bulk_urb(odev->rx_urb, odev->udev,
			  usb_rcvbulkpipe(odev->udev, odev->bulk_in_ep),
			  odev->rx_buf, odev->max_rd_transfer_size,
			  obmf_rx_complete, odev);

	rv = usb_submit_urb(odev->rx_urb, GFP_KERNEL);
	if (rv) {
		dev_err(&odev->intf->dev,
			"failed to submit rx URB: %d\n", rv);
		goto err_rx_buf;
	}

	/* Allocate interrupt IN resources if available */
	if (odev->has_int_in) {
		odev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
		odev->int_in_buf = kmalloc(odev->max_rd_int_size, GFP_KERNEL);
		if (!odev->int_in_urb || !odev->int_in_buf) {
			dev_warn(&odev->intf->dev,
				 "int IN alloc failed, continuing without\n");
			odev->has_int_in = false;
		}
	}

	/* Allocate interrupt OUT buffer if available */
	if (odev->has_int_out) {
		odev->int_out_buf = kmalloc(odev->max_wr_int_size, GFP_KERNEL);
		if (!odev->int_out_buf) {
			dev_warn(&odev->intf->dev,
				 "int OUT alloc failed, continuing without\n");
			odev->has_int_out = false;
		}
	}

	return 0;

err_rx_buf:
	kfree(odev->rx_buf);
	odev->rx_buf = NULL;
err_rx_urb:
	usb_free_urb(odev->rx_urb);
	odev->rx_urb = NULL;
err_tx:
	kfree(odev->tx_buf);
	odev->tx_buf = NULL;
err_wq:
	destroy_workqueue(odev->dev_req_wq);
	odev->dev_req_wq = NULL;
	return rv;
}

void obmf_transport_exit(struct obmf_device *odev)
{
	usb_kill_urb(odev->rx_urb);
	usb_kill_urb(odev->int_in_urb);

	if (odev->dev_req_wq) {
		drain_workqueue(odev->dev_req_wq);
		destroy_workqueue(odev->dev_req_wq);
		odev->dev_req_wq = NULL;
	}

	obmf_free_stale_in(odev);

	usb_free_urb(odev->rx_urb);
	usb_free_urb(odev->int_in_urb);

	kfree(odev->rx_buf);
	kfree(odev->tx_buf);
	kfree(odev->int_in_buf);
	kfree(odev->int_out_buf);

	odev->rx_urb    = NULL;
	odev->int_in_urb = NULL;
	odev->rx_buf    = NULL;
	odev->tx_buf    = NULL;
	odev->int_in_buf = NULL;
	odev->int_out_buf = NULL;
}

/* ------------------------------------------------------------------ */
/* obmf_send_request — synchronous request/response for all channels   */
/* ------------------------------------------------------------------ */

static int obmf_send_request_expect(struct obmf_device *odev,
				    struct obmf_channel *ch,
				    const void *payload, int payload_len,
				    void *resp_buf, int resp_buf_len,
				    unsigned long timeout_ms,
				    u8 expect_trans, int expect_len)
{
	struct obmf_common_hdr *hdr;
	int total_len;
	int actual_len;
	int rv;
	unsigned long remaining, flags;

	if (odev->disconnected)
		return -ENODEV;

	total_len = OBMF_COMMON_HDR_SIZE + payload_len;
	if (total_len > odev->max_wr_transfer_size) {
		dev_err(&odev->intf->dev,
			"ch: %d request size %d exceeds max_wr_transfer_size %d\n",
			ch->channel_id, total_len, odev->max_wr_transfer_size);
		return -EMSGSIZE;
	}

	mutex_lock(&odev->tx_lock);

	/* Build Common Header in tx_buf */
	hdr = (struct obmf_common_hdr *)odev->tx_buf;
	hdr->channel      = ch->channel_id;
	OBMF_HDR_SET_REQUEST(hdr);
	hdr->size         = cpu_to_le16(payload_len);

	/* Copy payload after header */
	if (payload_len > 0)
		memcpy(odev->tx_buf + OBMF_COMMON_HDR_SIZE, payload, payload_len);

	/* Prepare channel for response */
	spin_lock_irqsave(&ch->resp_lock, flags);
	ch->resp_buf = resp_buf;
	ch->resp_len = resp_buf_len;
	ch->status   = -ETIMEDOUT;
	ch->waiting  = true;
	ch->expect_trans = expect_trans;
	ch->expect_len   = expect_len;
	reinit_completion(&ch->done);
	spin_unlock_irqrestore(&ch->resp_lock, flags);

	/* Send via synchronous Bulk OUT */
	rv = usb_bulk_msg(odev->udev,
			  usb_sndbulkpipe(odev->udev, odev->bulk_out_ep),
			  odev->tx_buf, total_len, &actual_len,
			  timeout_ms);

	mutex_unlock(&odev->tx_lock);

	if (rv) {
		obmf_stop_waiting(ch);
		if (rv == -EPIPE) {
			odev->bulk_out_stall_count++;
			if (odev->bulk_out_stall_count >= OBMF_STALL_THRESHOLD)
				set_bit(OBMF_STALL_RESET_PENDING,
					&odev->stall_flags);
			else
				set_bit(OBMF_STALL_BULK_OUT,
					&odev->stall_flags);
			schedule_work(&odev->stall_work);
		}
		return rv;
	}

	odev->bulk_out_stall_count = 0;

	/* Wait for RX callback to deliver the response */
	remaining = wait_for_completion_timeout(&ch->done,
						msecs_to_jiffies(timeout_ms));
	if (!remaining) {
		obmf_stop_waiting(ch);
		return -ETIMEDOUT;
	}

	if (odev->disconnected)
		return -ENODEV;

	return ch->status ? ch->status : ch->resp_len;
}

int obmf_send_request(struct obmf_device *odev, struct obmf_channel *ch,
		      const void *payload, int payload_len,
		      void *resp_buf, int resp_buf_len,
		      unsigned long timeout_ms)
{
	return obmf_send_request_expect(odev, ch, payload, payload_len,
					resp_buf, resp_buf_len, timeout_ms,
					OBMF_EXPECT_ANY, -1);
}

/* ------------------------------------------------------------------ */
/* obmf_send_mmio_request — MMIO read/write.  The 2-byte header carries */
/* only the transaction type (Table 5).  Reads carry a u16 Read Length; */
/* writes carry no length, the data runs to the end (Tables 10, 14).   */
/* ------------------------------------------------------------------ */

int obmf_send_mmio_request(struct obmf_device *odev, struct obmf_channel *ch,
			   u8 transaction, u64 address,
			   const void *wr_data, int wr_len,
			   void *rd_data, int rd_len)
{
	u8 payload[OBMF_MMIO_SUBHDR_SIZE + 8 + 256]; /* hdr + addr + size/data */
	struct obmf_mmio_subhdr *mhdr = (struct obmf_mmio_subhdr *)payload;
	int payload_len;

	if (wr_len < 0 || wr_len > 256 || rd_len < 0 || rd_len > 256)
		return -EINVAL;

	/* Build MMIO header */
	mhdr->transaction = transaction;
	mhdr->reserved    = 0;

	switch (transaction) {
	case OBMF_TRANS_SHORT_READ:
		/* Short Read: Address(4B, 32-bit LE) + Size(2B, u16 LE) */
		put_unaligned_le32((u32)address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 4;
		put_unaligned_le16(rd_len, payload + payload_len);
		payload_len += 2;
		break;
	case OBMF_TRANS_SHORT_WRITE:
		/* Short Write: Address(4B, 32-bit LE) + Data(N) */
		put_unaligned_le32((u32)address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 4;
		if (wr_data && wr_len > 0) {
			memcpy(payload + payload_len, wr_data, wr_len);
			payload_len += wr_len;
		}
		break;
	case OBMF_TRANS_LONG_READ:
		/* Long Read: Address(8B, 64-bit LE) + Size(2B, u16 LE) */
		put_unaligned_le64(address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 8;
		put_unaligned_le16(rd_len, payload + payload_len);
		payload_len += 2;
		break;
	case OBMF_TRANS_LONG_WRITE:
		/* Long Write: Address(8B, 64-bit LE) + Data(N) */
		put_unaligned_le64(address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 8;
		if (wr_data && wr_len > 0) {
			memcpy(payload + payload_len, wr_data, wr_len);
			payload_len += wr_len;
		}
		break;
	default:
		return -EINVAL;
	}

	return obmf_send_request_expect(odev, ch, payload, payload_len,
					rd_data, rd_len,
					OBMF_DEFAULT_TIMEOUT_MS, transaction,
					(transaction == OBMF_TRANS_SHORT_READ ||
					 transaction == OBMF_TRANS_LONG_READ) ?
					rd_len : 0);
}

/* ------------------------------------------------------------------ */
/* obmf_send_io_request — I/O port read/write with the 1-byte header  */
/*                                                                     */
/* Payload layout after IO sub-header:                                 */
/*   port_addr (2B LE) + size (1B) + [write data (N)]                 */
/*                                                                     */
/* For writes: wr_data != NULL, rd_data/rd_len ignored.                */
/* For reads:  wr_data == NULL, rd_data receives the returned bytes.   */
/* Returns number of bytes read (>=0) or negative errno.               */
/* ------------------------------------------------------------------ */
int obmf_send_io_request(struct obmf_device *odev, struct obmf_channel *ch,
			 u8 transaction, u16 port_addr,
			 const void *wr_data, int wr_len,
			 void *rd_data, int rd_len)
{
	/* IO header(1) + port_addr(2) + size(1) + data(up to 255) */
	u8 payload[OBMF_IO_SUBHDR_SIZE + 2 + 1 + 255];
	struct obmf_io_subhdr *ihdr = (struct obmf_io_subhdr *)payload;
	int payload_len;
	bool is_write = (wr_data != NULL && wr_len > 0);

	if (is_write && wr_len > 255)
		return -EINVAL;
	if (!is_write && (rd_len <= 0 || rd_len > 255))
		return -EINVAL;

	/* Build IO header (reserved bits zero) */
	ihdr->transaction = transaction & 0x0F;

	/* port_addr (2B LE) */
	put_unaligned_le16(port_addr, payload + OBMF_IO_SUBHDR_SIZE);
	payload_len = OBMF_IO_SUBHDR_SIZE + 2;

	if (is_write) {
		/* Size (1B) + data */
		payload[payload_len++] = (u8)wr_len;
		memcpy(payload + payload_len, wr_data, wr_len);
		payload_len += wr_len;
	} else {
		/* Size (1B) — requested read length */
		payload[payload_len++] = (u8)rd_len;
	}

	return obmf_send_request(odev, ch,
				 payload, payload_len,
				 rd_data, rd_len,
				 OBMF_DEFAULT_TIMEOUT_MS);
}
