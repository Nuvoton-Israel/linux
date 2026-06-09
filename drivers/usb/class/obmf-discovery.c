// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-discovery.c - OBMF-ICP Channel 0 Discovery protocol (v0.9)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Enumerates channels via MMIO reads on Channel 0, reads Channel Config
 * Common Headers, and allocates per-channel state structures.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/unaligned.h>

#include "obmf.h"

/* ------------------------------------------------------------------ */
/* Helpers: MMIO read via Channel 0                                    */
/* ------------------------------------------------------------------ */

static int obmf_disc_read32(struct obmf_device *odev, u32 offset, u32 *val)
{
	struct obmf_channel *ch0 = &odev->channels[0];
	u8 resp[4];
	int rv;

	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
				    offset, NULL, 0, resp, sizeof(resp));
	if (rv < 0)
		return rv;

	*val = get_unaligned_le32(resp);
	return 0;
}

static int obmf_disc_read16(struct obmf_device *odev, u32 offset, u16 *val)
{
	struct obmf_channel *ch0 = &odev->channels[0];
	u8 resp[2];
	int rv;

	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
				    offset, NULL, 0, resp, sizeof(resp));
	if (rv < 0)
		return rv;

	*val = get_unaligned_le16(resp);
	return 0;
}

static int obmf_disc_read8(struct obmf_device *odev, u32 offset, u8 *val)
{
	struct obmf_channel *ch0 = &odev->channels[0];
	int rv;

	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
				    offset, NULL, 0, val, 1);
	if (rv < 0)
		return rv;

	return 0;
}

static int obmf_disc_read_bytes(struct obmf_device *odev, u32 offset,
				void *buf, int len)
{
	struct obmf_channel *ch0 = &odev->channels[0];
	int rv;

	/*
	 * v0.9: Short Read supports 32-bit address with u8 Size (up to 255B).
	 * Use Long Read only for reads > 255 bytes (u16 Size, 64-bit address).
	 */
	if (len <= 255)
		rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
					    offset, NULL, 0, buf, len);
	else
		rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_LONG_READ,
					    offset, NULL, 0, buf, len);
	if (rv < 0)
		return rv;

	return 0;
}

/* ------------------------------------------------------------------ */
/* obmf_discover_channels (v0.9)                                       */
/* ------------------------------------------------------------------ */

int obmf_discover_channels(struct obmf_device *odev)
{
	u16 obmf_ver, vendor_id, device_id;
	u32 device_role;
	u8 num_channels;
	char device_name[OBMF_DISC_DEVICE_NAME_LEN + 1];
	int rv, i;

	/*
	 * Allocate Channel 0 first — needed for discovery MMIO reads.
	 */
	odev->channels = kzalloc(sizeof(struct obmf_channel), GFP_KERNEL);
	if (!odev->channels)
		return -ENOMEM;

	odev->num_channels = 1;
	odev->channels[0].channel_id   = 0;
	odev->channels[0].channel_type = OBMF_TYPE_CONFIG;
	odev->channels[0].odev         = odev;
	mutex_init(&odev->channels[0].lock);
	init_completion(&odev->channels[0].done);

	/* Step 1: Read OBMF_VER (2B at 0x00) */
	rv = obmf_disc_read16(odev, OBMF_DISC_OBMF_VER, &obmf_ver);
	if (rv) {
		dev_err(&odev->intf->dev,
			"discovery: failed to read OBMF_VER: %d\n", rv);
		goto err;
	}

	dev_info(&odev->intf->dev, "OBMF_VER = 0x%04x\n", obmf_ver);

	if (obmf_ver < OBMF_MIN_SPEC_VERSION) {
		dev_err(&odev->intf->dev,
			"OBMF version 0x%04x too old (need >= 0x%04x)\n",
			obmf_ver, OBMF_MIN_SPEC_VERSION);
		rv = -ENODEV;
		goto err;
	}

	/* Step 2: Read device info */
	rv = obmf_disc_read16(odev, OBMF_DISC_VENDOR_ID, &vendor_id);
	if (rv == 0)
		rv = obmf_disc_read16(odev, OBMF_DISC_DEVICE_ID, &device_id);
	if (rv == 0)
		rv = obmf_disc_read32(odev, OBMF_DISC_DEVICE_ROLE, &device_role);
	if (rv) {
		dev_warn(&odev->intf->dev,
			 "discovery: failed to read device info: %d\n", rv);
		/* Non-fatal, continue */
	} else {
		dev_info(&odev->intf->dev,
			 "Vendor=0x%04x Device=0x%04x Role=%u\n",
			 vendor_id, device_id, device_role);
	}

	memset(device_name, 0, sizeof(device_name));
	rv = obmf_disc_read_bytes(odev, OBMF_DISC_DEVICE_NAME,
				  device_name, OBMF_DISC_DEVICE_NAME_LEN);
	if (rv == 0)
		dev_info(&odev->intf->dev, "Device Name: %s\n", device_name);

	/* Step 3: Read NUM_OF_CHANNELS (1B at 0x2A) */
	rv = obmf_disc_read8(odev, OBMF_DISC_NUM_CHANNELS, &num_channels);
	if (rv) {
		dev_err(&odev->intf->dev,
			"discovery: failed to read NUM_OF_CHANNELS: %d\n", rv);
		goto err;
	}

	dev_info(&odev->intf->dev, "NUM_OF_CHANNELS = %u\n", num_channels);

	if (num_channels == 0) {
		dev_err(&odev->intf->dev, "no channels reported\n");
		rv = -ENODEV;
		goto err;
	}

	/* Step 4: Reallocate channels array (CH0 + num_channels) */
	{
		struct obmf_channel *new_chs;
		int total = num_channels + 1;

		new_chs = kcalloc(total, sizeof(struct obmf_channel),
				  GFP_KERNEL);
		if (!new_chs) {
			rv = -ENOMEM;
			goto err;
		}

		/* Preserve channel 0 state */
		new_chs[0] = odev->channels[0];
		kfree(odev->channels);
		odev->channels     = new_chs;
		odev->num_channels = total;

		mutex_init(&new_chs[0].lock);
		init_completion(&new_chs[0].done);
		new_chs[0].odev = odev;
	}

	/* Initialize all channel structs */
	for (i = 1; i < odev->num_channels; i++) {
		odev->channels[i].channel_id = i;
		odev->channels[i].odev       = odev;
		mutex_init(&odev->channels[i].lock);
		init_completion(&odev->channels[i].done);
	}

	/* Step 5: Read CHANNEL_OFFSET[i] and Channel Config Common Headers */
	for (i = 0; i < num_channels; i++) {
		struct obmf_channel *ch = &odev->channels[i + 1];
		u32 ch_offset;
		u8 ch_type, ch_number;
		char ch_name[OBMF_CHCFG_NAME_LEN + 1];
		u32 config_size;

		/* Read CHANNEL_OFFSET[i] (4B at 0x30 + i*4) */
		rv = obmf_disc_read32(odev,
				      OBMF_DISC_CHANNEL_OFFSET_BASE + i * 4,
				      &ch_offset);
		if (rv) {
			dev_warn(&odev->intf->dev,
				 "ch%d: failed to read CHANNEL_OFFSET: %d\n",
				 i + 1, rv);
			ch->channel_type = 0xFF;
			continue;
		}

		if (ch_offset == 0) {
			ch->channel_type = 0xFF;
			continue;
		}

		ch->config_offset = ch_offset;

		/* Read Channel Config Common Header fields */
		rv = obmf_disc_read8(odev, ch_offset + OBMF_CHCFG_TYPE,
				     &ch_type);
		if (rv) {
			dev_warn(&odev->intf->dev,
				 "ch%d: failed to read CHANNEL_TYPE: %d\n",
				 i + 1, rv);
			ch->channel_type = 0xFF;
			continue;
		}

		rv = obmf_disc_read8(odev, ch_offset + OBMF_CHCFG_NUMBER,
				     &ch_number);
		if (rv) {
			ch_number = i + 1;
		}

		memset(ch_name, 0, sizeof(ch_name));
		obmf_disc_read_bytes(odev, ch_offset + OBMF_CHCFG_NAME,
				     ch_name, OBMF_CHCFG_NAME_LEN);

		rv = obmf_disc_read32(odev, ch_offset + OBMF_CHCFG_CONFIG_SIZE,
				      &config_size);
		if (rv)
			config_size = 0;

		ch->channel_id   = ch_number;
		ch->channel_type = ch_type;
		ch->config_size  = config_size;

		dev_info(&odev->intf->dev,
			 "ch%u: type=0x%02x name=\"%s\" config_offset=0x%08x config_size=%u\n",
			 ch_number, ch_type, ch_name, ch_offset, config_size);
	}

	return 0;

err:
	for (i = 0; i < odev->num_channels; i++) {
		kfree(odev->channels[i].reasm_buf);
		odev->channels[i].reasm_buf = NULL;
	}
	kfree(odev->channels);
	odev->channels = NULL;
	odev->num_channels = 0;
	return rv;
}

void obmf_free_channels(struct obmf_device *odev)
{
	int i;

	for (i = 0; i < odev->num_channels; i++) {
		kfree(odev->channels[i].reasm_buf);
		odev->channels[i].reasm_buf = NULL;
	}
	kfree(odev->channels);
	odev->channels = NULL;
	odev->num_channels = 0;
}
