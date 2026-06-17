// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-core.c - OBMF-ICP over USB class driver — probe / disconnect / module
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Virtual-adapter mux/demux architecture per OBMF-ICP v0.9 + USB 1.0.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/idr.h>
#include <linux/unaligned.h>

#include "obmf.h"

#define DRIVER_AUTHOR	"Nuvoton Technology Corp."
#define DRIVER_DESC	"OBMF-ICP over USB class driver"

static DEFINE_IDA(obmf_ida);

/* ------------------------------------------------------------------ */
/* kref destructor                                                     */
/* ------------------------------------------------------------------ */

static void obmf_delete(struct kref *kref)
{
	struct obmf_device *odev = container_of(kref, struct obmf_device, kref);

	usb_put_intf(odev->intf);
	usb_put_dev(odev->udev);
	kfree(odev);
}

/* ------------------------------------------------------------------ */
/* Parse OCP_OBMF_FUNCTIONAL descriptor                                */
/* ------------------------------------------------------------------ */

static int obmf_parse_functional_desc(struct usb_interface *intf,
				      struct obmf_device *odev)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	const u8 *extra = alt->extra;
	int extralen = alt->extralen;
	const struct obmf_functional_desc *fd;

	while (extralen >= 3) {
		u8 len  = extra[0];
		u8 type = extra[1];

		if (len < 2 || len > extralen)
			break;

		if (type == USB_DT_CS_INTERFACE && len >= 3 &&
		    extra[2] == OBMF_SUBTYPE_FUNCTIONAL) {
			if (len < OBMF_FUNCTIONAL_DESC_SIZE) {
				dev_err(&intf->dev,
					"OBMF functional desc too short: %u\n", len);
				return -EINVAL;
			}

			fd = (const struct obmf_functional_desc *)extra;

			if (fd->bMultimessageSupport != 0x00) {
				dev_err(&intf->dev,
					"bMultimessageSupport=0x%02x unsupported\n",
					fd->bMultimessageSupport);
				return -EINVAL;
			}

			odev->max_wr_transfer_size = le16_to_cpu(fd->wMaxWrTransferSize);
			odev->max_rd_transfer_size = le16_to_cpu(fd->wMaxRdTransferSize);
			odev->max_wr_int_size      = le16_to_cpu(fd->wMaxWrInterruptSize);
			odev->max_rd_int_size      = le16_to_cpu(fd->wMaxRdInterruptSize);
			odev->bcd_version          = le16_to_cpu(fd->bcdOCPOBMFVersion);

			dev_info(&intf->dev,
				 "OBMF functional: wr=%u rd=%u wr_int=%u rd_int=%u ver=0x%04x\n",
				 odev->max_wr_transfer_size,
				 odev->max_rd_transfer_size,
				 odev->max_wr_int_size,
				 odev->max_rd_int_size,
				 odev->bcd_version);
			return 0;
		}

		extra    += len;
		extralen -= len;
	}

	dev_err(&intf->dev, "OCP_OBMF_FUNCTIONAL descriptor not found\n");
	return -ENODEV;
}

/* ------------------------------------------------------------------ */
/* Set CHANNEL_CONTROL.ENABLE for a successfully registered channel     */
/* ------------------------------------------------------------------ */

static void obmf_channel_set_enabled(struct obmf_device *odev,
				     struct obmf_channel *ch)
{
	struct obmf_channel *ch0 = &odev->channels[0];
	u32 ctrl_addr;
	u8 ctrl;
	int rv;

	if (ch->config_offset == 0)
		return;

	ctrl_addr = ch->config_offset + OBMF_CHCFG_CONTROL;

	/* Read current CHANNEL_CONTROL (1B) via Channel 0 */
	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
				    ctrl_addr, NULL, 0, &ctrl, 1);
	if (rv < 0) {
		dev_warn(&odev->intf->dev,
			 "ch%u: failed to read CHANNEL_CONTROL: %d\n",
			 ch->channel_id, rv);
		return;
	}

	if (ctrl & OBMF_CHCTL_ENABLE)
		return; /* already enabled */

	ctrl |= OBMF_CHCTL_ENABLE;

	/* Write back CHANNEL_CONTROL with ENABLE bit set */
	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_WRITE,
				    ctrl_addr, &ctrl, 1, NULL, 0);
	if (rv < 0)
		dev_warn(&odev->intf->dev,
			 "ch%u: failed to set CHANNEL_CONTROL.ENABLE: %d\n",
			 ch->channel_id, rv);
	else
		dev_dbg(&odev->intf->dev,
			"ch%u: CHANNEL_CONTROL.ENABLE set (ctrl=0x%02x)\n",
			ch->channel_id, ctrl);
}

/* ------------------------------------------------------------------ */
/* Register subsystem adapters for all discovered channels             */
/* ------------------------------------------------------------------ */

static void obmf_register_channel(struct obmf_device *odev,
				  struct obmf_channel *ch)
{
	int rv = 0;

	switch (ch->channel_type) {
	case OBMF_TYPE_I2C:
		rv = obmf_i2c_register(odev, ch);
		break;
	case OBMF_TYPE_GPIO:
		rv = obmf_gpio_register(odev, ch);
		break;
	case OBMF_TYPE_SPI:
		rv = obmf_spi_register(odev, ch);
		break;
	case OBMF_TYPE_SERIAL:
		rv = obmf_serial_register(odev, ch);
		break;
	case OBMF_TYPE_IPMI:
		rv = obmf_ipmi_register(odev, ch);
		break;
	case OBMF_TYPE_CONFIG:
		/* Channel 0 — discovery only, nothing to register */
		break;
	case OBMF_TYPE_MMIO:
		/*
		 * MMIO channel — register misc device for userspace access.
		 * VW, UART, and Flash sub-handlers removed in v0.9
		 * (replaced by GPIO, Serial, and SPI optimised channels).
		 */
		rv = obmf_mmio_register(odev, ch);
		break;
	default:
		if (ch->channel_type >= OBMF_TYPE_OEM_MIN &&
		    ch->channel_type <= OBMF_TYPE_OEM_MAX) {
			rv = obmf_oem_register(odev, ch);
		} else if (ch->channel_type != 0xFF) {
			dev_info(&odev->intf->dev,
				 "ch%u: unsupported type 0x%02x, skipping\n",
				 ch->channel_id, ch->channel_type);
		}
		break;
	}

	if (rv)
		dev_err(&odev->intf->dev,
			"ch%u: failed to register type 0x%02x: %d\n",
			ch->channel_id, ch->channel_type, rv);
	else
		obmf_channel_set_enabled(odev, ch);

	/* Create sysfs "device" symlink for non-MMIO channels */
	if (!rv && ch->sysfs_dev && ch->kobj &&
	    ch->channel_type != OBMF_TYPE_MMIO)
		sysfs_create_link(ch->kobj, &ch->sysfs_dev->kobj, "device");
}

static void obmf_unregister_channel(struct obmf_channel *ch)
{
	/* Remove sysfs "device" symlink for non-MMIO channels */
	if (ch->sysfs_dev && ch->kobj && ch->channel_type != OBMF_TYPE_MMIO)
		sysfs_remove_link(ch->kobj, "device");

	switch (ch->channel_type) {
	case OBMF_TYPE_I2C:
		obmf_i2c_unregister(ch);
		break;
	case OBMF_TYPE_GPIO:
		obmf_gpio_unregister(ch);
		break;
	case OBMF_TYPE_SPI:
		obmf_spi_unregister(ch);
		break;
	case OBMF_TYPE_SERIAL:
		obmf_serial_unregister(ch);
		break;
	case OBMF_TYPE_IPMI:
		obmf_ipmi_unregister(ch);
		break;
	case OBMF_TYPE_MMIO:
		obmf_mmio_unregister(ch);
		break;
	default:
		if (ch->channel_type >= OBMF_TYPE_OEM_MIN &&
		    ch->channel_type <= OBMF_TYPE_OEM_MAX)
			obmf_oem_unregister(ch);
		break;
	}
}

/* ------------------------------------------------------------------ */
/* USB probe                                                           */
/* ------------------------------------------------------------------ */

static int obmf_probe(struct usb_interface *intf,
		      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *alt = intf->cur_altsetting;
	struct usb_endpoint_descriptor *ep;
	struct usb_endpoint_descriptor *ep_bulk_in = NULL;
	struct usb_endpoint_descriptor *ep_bulk_out = NULL;
	struct usb_endpoint_descriptor *ep_int_in = NULL;
	struct usb_endpoint_descriptor *ep_int_out = NULL;
	struct obmf_device *odev;
	int i, rv;
	char iface_string[16];

	dev_info(&intf->dev, "OBMF-ICP device detected\n");

	/* Validate iInterface string */
	if (alt->desc.iInterface) {
		rv = usb_string(udev, alt->desc.iInterface,
				iface_string, sizeof(iface_string));
		if (rv > 0) {
			if (strcmp(iface_string, OBMF_IINTERFACE_STRING) != 0)
				dev_warn(&intf->dev,
					 "iInterface \"%s\", expected \"%s\"\n",
					 iface_string, OBMF_IINTERFACE_STRING);
		}
	}

	/* bAlternateSetting must be 0 */
	if (alt->desc.bAlternateSetting != 0) {
		dev_err(&intf->dev, "bAlternateSetting=%u, expected 0\n",
			alt->desc.bAlternateSetting);
		return -ENODEV;
	}

	/* Scan endpoints: need exactly 1 Bulk IN + 1 Bulk OUT, optional int */
	if (alt->desc.bNumEndpoints < 2 || alt->desc.bNumEndpoints > 4) {
		dev_err(&intf->dev, "unexpected endpoint count: %d\n",
			alt->desc.bNumEndpoints);
		return -ENODEV;
	}

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		ep = &alt->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(ep)) {
			if (ep_bulk_out)
				return -ENODEV;
			ep_bulk_out = ep;
		} else if (usb_endpoint_is_bulk_in(ep)) {
			if (ep_bulk_in)
				return -ENODEV;
			ep_bulk_in = ep;
		} else if (usb_endpoint_is_int_in(ep)) {
			if (ep_int_in)
				return -ENODEV;
			ep_int_in = ep;
		} else if (usb_endpoint_is_int_out(ep)) {
			if (ep_int_out)
				return -ENODEV;
			ep_int_out = ep;
		}
	}

	if (!ep_bulk_in || !ep_bulk_out) {
		dev_err(&intf->dev, "missing required Bulk IN/OUT endpoints\n");
		return -ENODEV;
	}

	/* Allocate device */
	odev = kzalloc(sizeof(*odev), GFP_KERNEL);
	if (!odev)
		return -ENOMEM;

	odev->udev = usb_get_dev(udev);
	odev->intf = usb_get_intf(intf);
	kref_init(&odev->kref);
	mutex_init(&odev->tx_lock);
	INIT_WORK(&odev->stall_work, obmf_stall_recovery_work);

	odev->device_index = ida_alloc(&obmf_ida, GFP_KERNEL);
	if (odev->device_index < 0) {
		rv = odev->device_index;
		goto err_put;
	}

	/* Store endpoints */
	odev->bulk_in_ep   = ep_bulk_in->bEndpointAddress;
	odev->bulk_out_ep  = ep_bulk_out->bEndpointAddress;
	odev->bulk_in_maxp = usb_endpoint_maxp(ep_bulk_in);
	odev->bulk_out_maxp = usb_endpoint_maxp(ep_bulk_out);

	if (ep_int_in) {
		odev->int_in_ep       = ep_int_in->bEndpointAddress;
		odev->int_in_ep_size  = usb_endpoint_maxp(ep_int_in);
		odev->int_in_interval = ep_int_in->bInterval;
	}
	if (ep_int_out) {
		odev->int_out_ep       = ep_int_out->bEndpointAddress;
		odev->int_out_ep_size  = usb_endpoint_maxp(ep_int_out);
		odev->int_out_interval = ep_int_out->bInterval;
	}

	/* Parse functional descriptor */
	rv = obmf_parse_functional_desc(intf, odev);
	if (rv)
		goto err_put;

	/* Cross-check interrupt endpoint usability */
	odev->has_int_in  = (ep_int_in  && odev->max_rd_int_size > 0);
	odev->has_int_out = (ep_int_out && odev->max_wr_int_size > 0);

	/*
	 * Determine host (BMC) effective TX/RX sizes.
	 * The BMC has no inherent memory limit; the ceiling is set by
	 * what the device can receive/send per transfer.
	 * host_tx_size → used to allocate tx_buf and advertise READ_SIZE.PRI
	 * host_rx_size → used to allocate rx_buf and advertise WRITE_SIZE.PRI
	 */
	odev->host_tx_size = odev->max_wr_transfer_size;
	odev->host_rx_size = odev->max_rd_transfer_size;

	/* Initialize transport (allocates URBs and buffers) */
	rv = obmf_transport_init(odev);
	if (rv)
		goto err_ida;

	/* Discover channels via Channel 0 */
	rv = obmf_discover_channels(odev);
	if (rv) {
		dev_err(&intf->dev, "channel discovery failed: %d\n", rv);
		goto err_transport;
	}

	/* Initialize tty driver before registering individual ports */
	rv = obmf_serial_init(odev);
	if (rv) {
		dev_err(&intf->dev, "serial init failed: %d\n", rv);
		goto err_transport;
	}

	/* Create sysfs hierarchy: obmf/channel/<N>/ */
	odev->obmf_kobj = kobject_create_and_add("obmf", &intf->dev.kobj);
	if (odev->obmf_kobj)
		odev->channel_kobj = kobject_create_and_add("channel",
							    odev->obmf_kobj);

	/* Register subsystem adapters for each discovered channel */
	for (i = 1; i < odev->num_channels; i++) {
		struct obmf_channel *ch = &odev->channels[i];
		char name[8];

		if (odev->channel_kobj && ch->channel_type != 0xFF) {
			snprintf(name, sizeof(name), "%d", i);
			ch->kobj = kobject_create_and_add(name,
							  odev->channel_kobj);
		}
		obmf_register_channel(odev, ch);
	}

	usb_set_intfdata(intf, odev);

	dev_info(&intf->dev,
		 "OBMF-ICP obmf%d ready: %d channels, ver 0x%04x\n",
		 odev->device_index, odev->num_channels - 1, odev->bcd_version);
	return 0;

err_transport:
	obmf_transport_exit(odev);
err_ida:
	ida_free(&obmf_ida, odev->device_index);
err_put:
	usb_put_intf(odev->intf);
	usb_put_dev(odev->udev);
	kfree(odev);
	return rv;
}

/* ------------------------------------------------------------------ */
/* USB disconnect                                                      */
/* ------------------------------------------------------------------ */

static void obmf_disconnect(struct usb_interface *intf)
{
	struct obmf_device *odev = usb_get_intfdata(intf);
	int i;

	if (!odev)
		return;

	dev_info(&intf->dev, "OBMF-ICP disconnecting\n");

	odev->disconnected = true;

	/* Unregister all subsystem adapters (reverse order) */
	for (i = odev->num_channels - 1; i >= 1; i--) {
		obmf_unregister_channel(&odev->channels[i]);
		kobject_put(odev->channels[i].kobj);
	}

	/* Tear down sysfs hierarchy */
	kobject_put(odev->channel_kobj);
	kobject_put(odev->obmf_kobj);

	/* Wake up any waiters */
	for (i = 0; i < odev->num_channels; i++)
		complete_all(&odev->channels[i].done);

	cancel_work_sync(&odev->stall_work);

	obmf_serial_exit(odev);
	obmf_transport_exit(odev);
	obmf_free_channels(odev);

	ida_free(&obmf_ida, odev->device_index);
	usb_set_intfdata(intf, NULL);
	kref_put(&odev->kref, obmf_delete);
}

/* ------------------------------------------------------------------ */
/* STALL recovery work (exported for obmf.h forward declaration)       */
/* ------------------------------------------------------------------ */

void obmf_stall_recovery_work(struct work_struct *work)
{
	struct obmf_device *odev = container_of(work, struct obmf_device,
						stall_work);
	int rv;

	if (odev->disconnected)
		return;

	if (test_and_clear_bit(OBMF_STALL_RESET_PENDING, &odev->stall_flags)) {
		dev_err(&odev->intf->dev,
			"repeated STALLs, attempting USB device reset\n");
		rv = usb_reset_device(odev->udev);
		if (rv)
			dev_err(&odev->intf->dev,
				"USB device reset failed: %d\n", rv);
		odev->bulk_in_stall_count  = 0;
		odev->bulk_out_stall_count = 0;
		return;
	}

	if (test_and_clear_bit(OBMF_STALL_BULK_IN, &odev->stall_flags)) {
		rv = usb_clear_halt(odev->udev,
				    usb_rcvbulkpipe(odev->udev,
						   odev->bulk_in_ep));
		if (rv)
			dev_err(&odev->intf->dev,
				"clear halt Bulk IN failed: %d\n", rv);
	}

	if (test_and_clear_bit(OBMF_STALL_BULK_OUT, &odev->stall_flags)) {
		rv = usb_clear_halt(odev->udev,
				    usb_sndbulkpipe(odev->udev,
						   odev->bulk_out_ep));
		if (rv)
			dev_err(&odev->intf->dev,
				"clear halt Bulk OUT failed: %d\n", rv);
	}

	if (test_and_clear_bit(OBMF_STALL_INT_IN, &odev->stall_flags)) {
		rv = usb_clear_halt(odev->udev,
				    usb_rcvintpipe(odev->udev,
						  odev->int_in_ep));
		if (rv)
			dev_err(&odev->intf->dev,
				"clear halt Int IN failed: %d\n", rv);
	}

	if (test_and_clear_bit(OBMF_STALL_INT_OUT, &odev->stall_flags)) {
		rv = usb_clear_halt(odev->udev,
				    usb_sndintpipe(odev->udev,
						  odev->int_out_ep));
		if (rv)
			dev_err(&odev->intf->dev,
				"clear halt Int OUT failed: %d\n", rv);
	}
}

/* ------------------------------------------------------------------ */
/* USB reset handlers                                                  */
/*                                                                     */
/* Implementing pre_reset/post_reset keeps the interface bound across  */
/* usb_reset_device() (used by STALL recovery).  Without them the USB  */
/* core would unbind+rebind the interface, tearing down the tty ports  */
/* while obmc-console still has ttyOBMF open — leading to flushes of a  */
/* freed tty_port work item (workqueue.c WARN in __flush_work).        */
/* ------------------------------------------------------------------ */

static int obmf_pre_reset(struct usb_interface *intf)
{
	struct obmf_device *odev = usb_get_intfdata(intf);

	if (!odev)
		return 0;

	/*
	 * Quiesce I/O before the reset: stop the continuous Bulk IN URB and
	 * take tx_lock so no request is in flight while the device resets.
	 * tx_lock is released again in obmf_post_reset().
	 */
	usb_kill_urb(odev->rx_urb);
	mutex_lock(&odev->tx_lock);
	return 0;
}

static int obmf_post_reset(struct usb_interface *intf)
{
	struct obmf_device *odev = usb_get_intfdata(intf);
	int i, rv;

	if (!odev)
		return 0;

	mutex_unlock(&odev->tx_lock);

	if (odev->disconnected)
		return 0;

	/* Resubmit the continuous Bulk IN URB so RX resumes. */
	rv = usb_submit_urb(odev->rx_urb, GFP_KERNEL);
	if (rv)
		dev_err(&intf->dev,
			"post_reset: failed to resubmit rx URB: %d\n", rv);

	/*
	 * A device reset clears CHANNEL_CONTROL.ENABLE, so re-enable every
	 * channel that was registered to resume data flow (e.g. the serial
	 * channel backing ttyOBMF).
	 */
	for (i = 1; i < odev->num_channels; i++) {
		struct obmf_channel *ch = &odev->channels[i];

		if (ch->channel_type != 0xFF)
			obmf_channel_set_enabled(odev, ch);
	}

	odev->bulk_in_stall_count  = 0;
	odev->bulk_out_stall_count = 0;
	return 0;
}

/* ------------------------------------------------------------------ */
/* USB driver registration                                             */
/* ------------------------------------------------------------------ */

static const struct usb_device_id obmf_ids[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_OBMF, USB_OBMF_SUBCLASS_ICP,
			     USB_OBMF_PROTOCOL_V1) },
	{ }
};
MODULE_DEVICE_TABLE(usb, obmf_ids);

static struct usb_driver obmf_driver = {
	.name			= "obmf",
	.probe			= obmf_probe,
	.disconnect		= obmf_disconnect,
	.pre_reset		= obmf_pre_reset,
	.post_reset		= obmf_post_reset,
	.id_table		= obmf_ids,
	.supports_autosuspend	= 1,
	.disable_hub_initiated_lpm = 1,
};
module_usb_driver(obmf_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
