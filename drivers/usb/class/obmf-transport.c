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
#include <linux/unaligned.h>

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
		       u8 channel_type, u8 status,
		       const void *payload, int payload_len)
{
	struct obmf_common_hdr *hdr;
	int seg_data_max, offset = 0, actual_len, rv = 0;

	if (odev->disconnected)
		return -ENODEV;

	/*
	 * Segment the response into USB-packet-sized chunks so the device
	 * always sees a complete OBMF Common Header at the start of each
	 * USB packet.  hdr->size carries the *total* payload length across
	 * all segments; the device accumulates until it has that many bytes.
	 */
	seg_data_max = odev->max_wr_transfer_size - OBMF_COMMON_HDR_SIZE;
	if (seg_data_max <= 0)
		return -EINVAL;

	mutex_lock(&odev->tx_lock);

	hdr = (struct obmf_common_hdr *)odev->tx_buf;
	hdr->channel      = channel_id;
	hdr->channel_type = channel_type;
	OBMF_HDR_SET_RESPONSE(hdr, status);
	hdr->size         = cpu_to_le16(payload_len);

	do {
		int chunk = min(payload_len - offset, seg_data_max);

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

	ch = obmf_find_channel(odev, dreq->channel_id);
	if (!ch)
		goto out;

	switch (dreq->channel_type) {
	case OBMF_TYPE_MMIO:
	case OBMF_TYPE_CONFIG: {
		/*
		 * MMIO device-initiated request.
		 * v0.9: Short transactions use 32-bit address,
		 *        Long transactions use 64-bit address.
		 * Dispatch to MMIO misc handler.
		 */
		obmf_mmio_handle_dev_request(ch, dreq->transaction,
					     dreq->tag,
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
					   dreq->tag,
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
	case OBMF_TYPE_IPMI:
		obmf_ipmi_handle_dev_request(ch, dreq->data, dreq->data_len);
		break;
	default:
		if (dreq->channel_type >= OBMF_TYPE_OEM_MIN &&
		    dreq->channel_type <= OBMF_TYPE_OEM_MAX) {
			obmf_oem_handle_dev_request(ch, dreq->data,
						    dreq->data_len);
		} else {
			obmf_send_response(odev, dreq->channel_id,
					   dreq->channel_type,
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

static void obmf_dispatch_message(struct obmf_device *odev,
				  struct obmf_common_hdr *hdr,
				  const u8 *payload, int payload_len)
{
	struct obmf_channel *ch;
	u8 channel_id = hdr->channel;

	ch = obmf_find_channel(odev, channel_id);
	if (!ch) {
		dev_err(&odev->intf->dev, "rx: unknown channel %u\n",
			channel_id);
		return;
	}

	if (OBMF_HDR_IS_RESPONSE(hdr)) {
		/* Response to our earlier request */
		int copy_len;
		u8 hdr_status = OBMF_HDR_STATUS(hdr);

		ch->status = hdr_status;
		/* Check response status from header byte 2[7:1] */
		if (hdr_status != OBMF_STATUS_SUCCESS) {
			dev_err(&odev->intf->dev,
				"ch%u: response status 0x%02x\n",
				channel_id, hdr_status);
			switch (hdr_status) {
			case OBMF_STATUS_INVALID_CMD:
				ch->status = -EOPNOTSUPP;
				break;
			case OBMF_STATUS_TIMEOUT:
				ch->status = -ETIMEDOUT;
				break;
			case OBMF_STATUS_NOT_READY:
				ch->status = -EAGAIN;
				break;
			case OBMF_STATUS_PERMANENT_ERROR:
				ch->status = -EIO;
				break;
			case OBMF_STATUS_UNKNOWN_CHANNEL:
				ch->status = -ENXIO;
				break;
			case OBMF_STATUS_SIZE_NOT_SUPPORTED:
				ch->status = -EMSGSIZE;
				break;
			case OBMF_MMIO_STATUS_ADDR_OUT_OF_RANGE:
				ch->status = -EFAULT;
				break;
			case OBMF_MMIO_STATUS_ACCESS_DENIED:
				ch->status = -EACCES;
				break;
			default:
				ch->status = -EIO;
				break;
			}
			complete(&ch->done);
			return;
		}

		/* For MMIO / CONFIG (Discovery) / IO: validate tag in sub-header */
		if ((hdr->channel_type == OBMF_TYPE_MMIO ||
		     hdr->channel_type == OBMF_TYPE_CONFIG ||
		     hdr->channel_type == OBMF_TYPE_IO) &&
		    payload_len >= OBMF_MMIO_SUBHDR_SIZE) {
			struct obmf_mmio_subhdr *mhdr =
				(struct obmf_mmio_subhdr *)payload;
			const char *type_str =
				(hdr->channel_type == OBMF_TYPE_IO) ? "IO" : "MMIO";

			if (mhdr->tag != ch->tag) {
				dev_err(&odev->intf->dev,
					"ch%u: %s tag mismatch (got %u exp %u)\n",
					channel_id, type_str, mhdr->tag, ch->tag);
				ch->status = -EIO;
				complete(&ch->done);
				return;
			}
			ch->tag ^= 1;
			/* Skip sub-header for payload copy */
			payload     += OBMF_MMIO_SUBHDR_SIZE;
			payload_len -= OBMF_MMIO_SUBHDR_SIZE;
		}

		copy_len = min_t(int, payload_len, ch->resp_len);
		if (ch->resp_buf && copy_len > 0)
			memcpy(ch->resp_buf, payload, copy_len);
		ch->resp_len = copy_len;
		ch->status = 0;
		complete(&ch->done);
	} else {
		dev_dbg(&odev->intf->dev,
			"OBMF RX: channel=%u type=0x%02x rqresp_status=0x%02x size=%u\n",
			channel_id, hdr->channel_type, hdr->rqresp_status,
			payload_len);
		/*
		 * RqResp=0 on Bulk IN: device-initiated request.
		 * Host must respond (Host = Responder).
		 * All channel types are deferred to workqueue.
		 */
		if ((hdr->channel_type == OBMF_TYPE_MMIO ||
		     hdr->channel_type == OBMF_TYPE_CONFIG ||
		     hdr->channel_type == OBMF_TYPE_IO) &&
		    payload_len >= OBMF_MMIO_SUBHDR_SIZE) {
			struct obmf_mmio_subhdr *mhdr =
				(struct obmf_mmio_subhdr *)payload;
			/*
			 * IO channel has 12 transaction types (0-11), so use
			 * 0x0F mask; MMIO uses 0x07 mask (3 bits, 4 types).
			 */
			u8 trans = (hdr->channel_type == OBMF_TYPE_IO)
				   ? (mhdr->transaction & 0x0F)
				   : (mhdr->transaction & 0x07);
			u8 disp_type = (hdr->channel_type == OBMF_TYPE_IO)
					? OBMF_TYPE_IO : OBMF_TYPE_MMIO;

			/* Validate incoming tag from device */
			if (mhdr->tag != ch->dev_tag) {
				dev_err(&odev->intf->dev,
					"ch%u: %s dev-req tag mismatch (got %u exp %u), dropping\n",
					channel_id,
					(disp_type == OBMF_TYPE_IO) ? "IO" : "MMIO",
					mhdr->tag, ch->dev_tag);
				return;
			}
			ch->dev_tag ^= 1;

			{
				struct obmf_dev_req *dreq;
				int req_payload_len;

				dev_dbg(&odev->intf->dev,
					"ch%u: %s dev-req (trans=%u)\n",
					channel_id,
					(disp_type == OBMF_TYPE_IO) ? "IO" : "MMIO",
					trans);

				req_payload_len = payload_len - OBMF_MMIO_SUBHDR_SIZE;
				dreq = obmf_alloc_dev_req(req_payload_len);
				if (dreq) {
					INIT_WORK(&dreq->work,
						  obmf_dev_request_work);
					dreq->odev = odev;
					dreq->channel_id = channel_id;
					dreq->channel_type = disp_type;
					dreq->transaction = trans;
					dreq->tag = mhdr->tag;
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
				dreq->channel_type = hdr->channel_type;
				dreq->transaction = 0;
				dreq->tag = 0;
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
		dev_err(&odev->intf->dev, "rx: unknown channel %u\n",
			hdr->channel);
		goto resubmit;
	}

	if (ch->reasm_active) {
		/*
		 * Continuation segment — validate that channel and type
		 * match the first segment, then append payload data.
		 */
		if (hdr->channel_type != ch->reasm_hdr.channel_type ||
		    hdr->rqresp_status != ch->reasm_hdr.rqresp_status) {
			dev_err(&odev->intf->dev,
				"rx reasm: header mismatch on ch%u (type/status got 0x%02x/0x%02x exp 0x%02x/0x%02x), reset\n",
				hdr->channel,
				hdr->channel_type, hdr->rqresp_status,
				ch->reasm_hdr.channel_type,
				ch->reasm_hdr.rqresp_status);
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

int obmf_transport_init(struct obmf_device *odev)
{
	int rv;

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

int obmf_send_request(struct obmf_device *odev, struct obmf_channel *ch,
		      u8 channel_type, const void *payload, int payload_len,
		      void *resp_buf, int resp_buf_len,
		      unsigned long timeout_ms)
{
	struct obmf_common_hdr *hdr;
	int total_len;
	int actual_len;
	int rv;
	unsigned long remaining;

	if (odev->disconnected)
		return -ENODEV;

	total_len = OBMF_COMMON_HDR_SIZE + payload_len;
	if (total_len > odev->max_wr_transfer_size)
		return -EMSGSIZE;

	mutex_lock(&odev->tx_lock);

	/* Build Common Header in tx_buf */
	hdr = (struct obmf_common_hdr *)odev->tx_buf;
	hdr->channel      = ch->channel_id;
	hdr->channel_type = channel_type;
	OBMF_HDR_SET_REQUEST(hdr);
	hdr->size         = cpu_to_le16(payload_len);

	/* Copy payload after header */
	if (payload_len > 0)
		memcpy(odev->tx_buf + OBMF_COMMON_HDR_SIZE, payload, payload_len);

	/* Prepare channel for response */
	ch->resp_buf = resp_buf;
	ch->resp_len = resp_buf_len;
	ch->status   = -ETIMEDOUT;
	reinit_completion(&ch->done);

	/* Send via synchronous Bulk OUT */
	rv = usb_bulk_msg(odev->udev,
			  usb_sndbulkpipe(odev->udev, odev->bulk_out_ep),
			  odev->tx_buf, total_len, &actual_len,
			  timeout_ms);

	mutex_unlock(&odev->tx_lock);

	if (rv) {
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
	if (!remaining)
		return -ETIMEDOUT;

	if (odev->disconnected)
		return -ENODEV;

	return ch->status ? ch->status : ch->resp_len;
}

/* ------------------------------------------------------------------ */
/* obmf_send_mmio_request — MMIO read/write with sub-header + tag      */
/* ------------------------------------------------------------------ */

int obmf_send_mmio_request(struct obmf_device *odev, struct obmf_channel *ch,
			   u8 transaction, u64 address,
			   const void *wr_data, int wr_len,
			   void *rd_data, int rd_len)
{
	u8 payload[OBMF_MMIO_SUBHDR_SIZE + 8 + 2 + 256]; /* sub-hdr + addr + size + data */
	struct obmf_mmio_subhdr *mhdr = (struct obmf_mmio_subhdr *)payload;
	int payload_len;

	/* Build MMIO sub-header */
	mhdr->transaction = transaction;
	mhdr->tag         = ch->tag;

	switch (transaction) {
	case OBMF_TRANS_SHORT_READ:
		/* Short Read: Address(4B, 32-bit LE) + Size(1B) */
		put_unaligned_le32((u32)address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 4;
		payload[payload_len++] = (u8)rd_len;
		break;
	case OBMF_TRANS_SHORT_WRITE:
		/* Short Write: Address(4B, 32-bit LE) + Size(1B) + Data(N) */
		put_unaligned_le32((u32)address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 4;
		payload[payload_len++] = (u8)wr_len;
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
		/* Long Write: Address(8B, 64-bit LE) + Size(2B, u16 LE) + Data(N) */
		put_unaligned_le64(address, payload + OBMF_MMIO_SUBHDR_SIZE);
		payload_len = OBMF_MMIO_SUBHDR_SIZE + 8;
		put_unaligned_le16(wr_len, payload + payload_len);
		payload_len += 2;
		if (wr_data && wr_len > 0) {
			memcpy(payload + payload_len, wr_data, wr_len);
			payload_len += wr_len;
		}
		break;
	default:
		return -EINVAL;
	}

	return obmf_send_request(odev, ch, ch->channel_type,
				 payload, payload_len,
				 rd_data, rd_len,
				 OBMF_DEFAULT_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/* obmf_send_io_request — I/O port read/write with sub-header + tag    */
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
	/* IO sub-header(2) + port_addr(2) + size(1) + data(up to 255) */
	u8 payload[OBMF_IO_SUBHDR_SIZE + 2 + 1 + 255];
	struct obmf_io_subhdr *ihdr = (struct obmf_io_subhdr *)payload;
	int payload_len;
	bool is_write = (wr_data != NULL && wr_len > 0);

	if (is_write && wr_len > 255)
		return -EINVAL;
	if (!is_write && (rd_len <= 0 || rd_len > 255))
		return -EINVAL;

	/* Build IO sub-header */
	ihdr->transaction = transaction;
	ihdr->tag         = ch->tag;

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

	return obmf_send_request(odev, ch, OBMF_TYPE_IO,
				 payload, payload_len,
				 rd_data, rd_len,
				 OBMF_DEFAULT_TIMEOUT_MS);
}
