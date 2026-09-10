// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-mem-dev.c - OBMF-ICP Memory Device Channel (Type 0Ah, v1.0.0 RC1)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Implements the Memory Device Channel (spec §4.12): a byte-addressable
 * read/write/erase interface aligned with the Linux MTD driver model.
 * This channel is always Producer/Responder in this driver — the SMC
 * pushes Memory Read/Write/Erase commands, and this driver maps them
 * onto a locally-named MTD device (moved here from obmf-spi.c's former
 * "espi" mode, which used SPI NOR opcodes to carry the same semantics
 * before this channel type existed in the spec).
 *
 * Memory Device Request format (OBMF Type 0Ah):
 *   Command(1B)[1:0]: 0=Read, 1=Write, 2=Erase
 *   Address(4B LE): byte offset into the device address space
 *   DataSize(2B LE): bytes for Read/Write, KB for Erase
 *   Data(N): write payload, Write only
 *
 * Memory Device Response format:
 *   Status in Common Header byte 1[6:0], repeated in payload byte 0 (§4.12.2)
 *   Data(N): read payload from byte 1, Read + Success only
 */

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/usb.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

#define OBMF_MEM_DEV_MTD_NAME		"npcm-espi-flash"
#define OBMF_MEM_DEV_MTD_NAME_PROP	"nuvoton,mtd-name"

struct obmf_mem_dev_data {
	struct obmf_channel	*ch;
	struct mtd_info		*mtd;
};

/**
 * obmf_mem_dev_find_ch_node - find and return the DT child node for this channel.
 * @ch: OBMF channel
 *
 * Caller must call of_node_put() on the returned node.
 */
static struct device_node *obmf_mem_dev_find_ch_node(struct obmf_channel *ch)
{
	struct usb_device *udev = ch->odev->udev;
	struct device_node *udev_np, *ch_np = NULL, *tmp;

	udev_np = of_node_get(udev->dev.of_node);
	if (!udev_np)
		udev_np = obmf_find_udev_of_node(udev);
	if (!udev_np)
		return NULL;

	for_each_child_of_node(udev_np, tmp) {
		u32 reg;

		if (!of_property_read_u32(tmp, "reg", &reg) &&
		    reg == ch->channel_id) {
			ch_np = tmp;
			break;
		}
	}
	of_node_put(udev_np);
	return ch_np;
}

/**
 * obmf_mem_dev_get_mtd_name - resolve the MTD name for this channel from DTS.
 * @ch:    OBMF channel
 * @buf:   output buffer for the MTD name
 * @bufsz: size of @buf
 *
 * Reads the "nuvoton,mtd-name" property from the channel DT node.
 * Falls back to OBMF_MEM_DEV_MTD_NAME when not found.
 */
static void obmf_mem_dev_get_mtd_name(struct obmf_channel *ch,
				      char *buf, size_t bufsz)
{
	struct device_node *ch_np;
	const char *name = OBMF_MEM_DEV_MTD_NAME;

	ch_np = obmf_mem_dev_find_ch_node(ch);
	if (ch_np) {
		of_property_read_string(ch_np, OBMF_MEM_DEV_MTD_NAME_PROP, &name);
		of_node_put(ch_np);
	}
	strscpy(buf, name, bufsz);
}

/* ------------------------------------------------------------------ */
/* Device-initiated Memory Device request handler                      */
/* ------------------------------------------------------------------ */

/*
 * obmf_mem_dev_reply - send a response with the §4.12.2 payload Status
 * byte prepended, in addition to the Common Header status.
 * @data: when non-NULL, must already have status stored at data[0].
 */
static void obmf_mem_dev_reply(struct obmf_channel *ch, u8 status,
			       const void *data, int len)
{
	if (!data || len <= 0) {
		obmf_send_response(ch->odev, ch->channel_id, status,
				   &status, 1);
		return;
	}
	obmf_send_response(ch->odev, ch->channel_id, status, data, len);
}

void obmf_mem_dev_handle_dev_request(struct obmf_channel *ch,
				     const u8 *data, int len)
{
	struct obmf_mem_dev_data *md = ch->priv;
	struct mtd_info *mtd;
	u8 cmd;
	u32 addr;
	u16 size;
	u8 *resp;
	size_t retlen;
	int rv;

	if (!md || len < OBMF_MEM_DEV_REQ_SUBHDR_SIZE) {
		obmf_mem_dev_reply(ch, OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	mtd = md->mtd;
	cmd = data[0] & OBMF_MEM_DEV_CMD_MASK;
	addr = get_unaligned_le32(&data[1]);
	size = get_unaligned_le16(&data[5]);

	switch (cmd) {
	case OBMF_MEM_DEV_CMD_READ:
		if ((u64)addr + size > mtd->size) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_ACCESS_DENIED,
					   NULL, 0);
			return;
		}

		resp = kmalloc(1 + size, GFP_KERNEL);
		if (!resp) {
			obmf_mem_dev_reply(ch, OBMF_STATUS_PERMANENT_ERROR,
					   NULL, 0);
			return;
		}

		rv = mtd_read(mtd, addr, size, &retlen, resp + 1);
		if (rv && rv != -EUCLEAN) {
			kfree(resp);
			obmf_mem_dev_reply(ch, OBMF_STATUS_PERMANENT_ERROR,
					   NULL, 0);
			return;
		}

		resp[0] = OBMF_STATUS_SUCCESS;
		obmf_mem_dev_reply(ch, OBMF_STATUS_SUCCESS, resp, 1 + retlen);
		kfree(resp);
		return;

	case OBMF_MEM_DEV_CMD_WRITE:
		if (len < OBMF_MEM_DEV_REQ_SUBHDR_SIZE + size) {
			obmf_mem_dev_reply(ch, OBMF_STATUS_SIZE_NOT_SUPPORTED,
					   NULL, 0);
			return;
		}
		if (!(mtd->flags & MTD_WRITEABLE)) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_WRITE_PROTECTED,
					   NULL, 0);
			return;
		}
		if ((u64)addr + size > mtd->size) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_ACCESS_DENIED,
					   NULL, 0);
			return;
		}

		rv = mtd_write(mtd, addr, size, &retlen,
			       data + OBMF_MEM_DEV_REQ_SUBHDR_SIZE);
		obmf_mem_dev_reply(ch,
				   rv ? OBMF_STATUS_PERMANENT_ERROR
				      : OBMF_STATUS_SUCCESS,
				   NULL, 0);
		return;

	case OBMF_MEM_DEV_CMD_ERASE: {
		struct erase_info ei = {};
		u64 erase_len = (u64)size << 10;	/* size is in KB */

		if (!size) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_UNSUPPORTED_SIZE,
					   NULL, 0);
			return;
		}
		if (!(mtd->flags & MTD_WRITEABLE)) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_WRITE_PROTECTED,
					   NULL, 0);
			return;
		}
		if (addr % mtd->erasesize || erase_len % mtd->erasesize) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_ERASE_NOT_ALIGNED,
					   NULL, 0);
			return;
		}
		if (addr + erase_len > mtd->size) {
			obmf_mem_dev_reply(ch, OBMF_MEM_DEV_STATUS_ACCESS_DENIED,
					   NULL, 0);
			return;
		}

		ei.addr = addr;
		ei.len = erase_len;
		rv = mtd_erase(mtd, &ei);
		obmf_mem_dev_reply(ch,
				   rv ? OBMF_STATUS_PERMANENT_ERROR
				      : OBMF_STATUS_SUCCESS,
				   NULL, 0);
		return;
	}

	default:
		obmf_mem_dev_reply(ch, OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}
}

/* ------------------------------------------------------------------ */
/* register / unregister                                               */
/* ------------------------------------------------------------------ */

int obmf_mem_dev_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_mem_dev_data *md;
	struct mtd_info *mtd;
	char mtd_name[32];

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	obmf_mem_dev_get_mtd_name(ch, mtd_name, sizeof(mtd_name));

	mtd = get_mtd_device_nm(mtd_name);
	if (IS_ERR(mtd)) {
		dev_err(&odev->intf->dev,
			"ch%u: MTD '%s' not found: %ld\n",
			ch->channel_id, mtd_name, PTR_ERR(mtd));
		kfree(md);
		return PTR_ERR(mtd);
	}

	md->ch = ch;
	md->mtd = mtd;
	ch->priv = md;
	ch->sysfs_dev = &mtd->dev;

	if (ch->kobj)
		sysfs_create_link(ch->kobj, &mtd->dev.kobj, "mtd");

	dev_info(&odev->intf->dev,
		 "ch%u: registered Memory Device (MTD: %s, %llu bytes, erasesize %u)\n",
		 ch->channel_id, mtd_name, mtd->size, mtd->erasesize);
	return 0;
}

void obmf_mem_dev_unregister(struct obmf_channel *ch)
{
	struct obmf_mem_dev_data *md = ch->priv;

	if (!md)
		return;

	if (ch->kobj)
		sysfs_remove_link(ch->kobj, "mtd");
	if (md->mtd)
		put_mtd_device(md->mtd);

	kfree(md);
	ch->priv = NULL;
}
