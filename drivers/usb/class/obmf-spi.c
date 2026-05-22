// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-spi.c - OBMF-ICP SPI Controller Channel (Type 08h, v0.9)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Implements the SPI Controller Optimised Channel using v0.9 protocol.
 * Handles device-initiated SPI requests by mapping SPI NOR commands to
 * Linux MTD operations on the backing flash device.
 *
 * SPI Request format:
 *   Command(1B): [7:6]=CS#, [5]=Assert CS, [4]=Deassert CS, [3:0]=SPI cmd
 *   WriteDataSize(2B LE) + ReadDataSize(2B LE) + WriteData(N)
 *
 * SPI Response format:
 *   Command(1B) + ReadDataSize(2B LE) + ReadData(N)
 *   Status in Common Header byte 2[7:1]
 */

#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/usb.h>
#include <linux/unaligned.h>

#include "obmf.h"

#define OBMF_SPI_MTD_NAME	"npcm-espi-flash"

struct obmf_spi_data {
	struct obmf_channel	*ch;
	struct mtd_info		*mtd;
	u32			flash_offset;	/* current flash R/W offset */
};

/* SPI NOR opcodes */
#define SPI_NOR_READ		0x03
#define SPI_NOR_PAGE_PROG	0x02
#define SPI_NOR_SECTOR_ERASE	0x20
#define SPI_NOR_BLOCK_ERASE	0xD8
#define SPI_NOR_CHIP_ERASE	0xC7
#define SPI_NOR_RDSR		0x05
#define SPI_NOR_WREN		0x06

/* ------------------------------------------------------------------ */
/* Host-initiated SPI transfer                                         */
/* ------------------------------------------------------------------ */

static int obmf_spi_xfer(struct obmf_channel *ch, u8 cmd_byte,
			 const u8 *wr_data, u16 wr_len,
			 u8 *rd_data, u16 rd_len)
{
	struct obmf_device *odev = ch->odev;
	u8 req[5 + 512]; /* cmd(1) + wr_size(2) + rd_size(2) + wr_data */
	u8 resp[3 + 512]; /* cmd(1) + rd_size(2) + rd_data */
	int req_len, rv;

	req[0] = cmd_byte;
	put_unaligned_le16(wr_len, &req[1]);
	put_unaligned_le16(rd_len, &req[3]);
	req_len = 5;

	if (wr_data && wr_len > 0) {
		memcpy(req + 5, wr_data, wr_len);
		req_len += wr_len;
	}

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch, OBMF_TYPE_SPI,
			       req, req_len, resp, sizeof(resp),
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);

	if (rv < 0)
		return rv;

	/* Parse response: cmd(1) + rd_size(2) + rd_data(N) */
	if (rv >= 3 && rd_data && rd_len > 0) {
		u16 actual_rd = get_unaligned_le16(&resp[1]);
		int copy = min_t(u16, actual_rd, rd_len);

		if (copy > 0 && rv >= 3 + copy)
			memcpy(rd_data, resp + 3, copy);
		return copy;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Device-initiated SPI request handler                                */
/* ------------------------------------------------------------------ */

static u32 obmf_spi_extract_addr(const u8 *cmd_data, int cmd_len)
{
	/* SPI NOR commands: opcode(1) + address(3 or 4 bytes) */
	if (cmd_len >= 4)
		return ((u32)cmd_data[1] << 16) |
		       ((u32)cmd_data[2] << 8) |
		       (u32)cmd_data[3];
	return 0;
}

void obmf_spi_handle_dev_request(struct obmf_channel *ch,
				 const u8 *data, int len)
{
	struct obmf_device *odev = ch->odev;
	struct obmf_spi_data *sd = ch->priv;
	u8 cmd_byte, spi_cmd;
	u16 wr_size, rd_size;
	const u8 *wr_data;
	u8 *resp = NULL;
	int resp_len;
	u8 status = OBMF_STATUS_SUCCESS;

	if (!sd || len < 5) {
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_SPI,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	cmd_byte = data[0];
	spi_cmd = cmd_byte & OBMF_SPI_CMD_MASK;
	wr_size = get_unaligned_le16(&data[1]);
	rd_size = get_unaligned_le16(&data[3]);
	wr_data = data + 5;

	if (len < 5 + wr_size) {
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_SPI,
				   OBMF_STATUS_SIZE_NOT_SUPPORTED, NULL, 0);
		return;
	}

	switch (spi_cmd) {
	case OBMF_SPI_CMD_READ: {
		/* Read from flash at current offset */
		struct mtd_info *mtd;
		size_t retlen;
		int rv;

		resp = kmalloc(3 + rd_size, GFP_KERNEL);
		if (!resp) {
			status = OBMF_STATUS_PERMANENT_ERROR;
			break;
		}

		mtd = get_mtd_device_nm(OBMF_SPI_MTD_NAME);
		if (IS_ERR(mtd)) {
			status = OBMF_STATUS_PERMANENT_ERROR;
			break;
		}

		rv = mtd_read(mtd, sd->flash_offset, rd_size, &retlen,
			      resp + 3);
		put_mtd_device(mtd);

		if (rv && rv != -EUCLEAN) {
			status = OBMF_SPI_STATUS_TRANSFER_ERROR;
			break;
		}

		resp[0] = cmd_byte;
		put_unaligned_le16((u16)retlen, &resp[1]);
		resp_len = 3 + retlen;

		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_SPI, status,
				   resp, resp_len);
		kfree(resp);
		return;
	}

	case OBMF_SPI_CMD_WRITE: {
		/* Check if write data contains SPI NOR commands */
		if (wr_size >= 1) {
			u8 opcode = wr_data[0];

			if (opcode == SPI_NOR_SECTOR_ERASE ||
			    opcode == SPI_NOR_BLOCK_ERASE) {
				/* Erase command */
				struct mtd_info *mtd;
				struct erase_info ei = {};
				u32 addr = obmf_spi_extract_addr(wr_data, wr_size);
				int rv;

				mtd = get_mtd_device_nm(OBMF_SPI_MTD_NAME);
				if (IS_ERR(mtd)) {
					status = OBMF_STATUS_PERMANENT_ERROR;
					break;
				}

				ei.addr = addr;
				ei.len = (opcode == SPI_NOR_SECTOR_ERASE) ?
					 4096 : 65536;

				rv = mtd_erase(mtd, &ei);
				put_mtd_device(mtd);

				if (rv)
					status = OBMF_SPI_STATUS_TRANSFER_ERROR;
			} else if (opcode == SPI_NOR_CHIP_ERASE) {
				struct mtd_info *mtd;
				struct erase_info ei = {};
				int rv;

				mtd = get_mtd_device_nm(OBMF_SPI_MTD_NAME);
				if (IS_ERR(mtd)) {
					status = OBMF_STATUS_PERMANENT_ERROR;
					break;
				}

				ei.addr = 0;
				ei.len = mtd->size;
				rv = mtd_erase(mtd, &ei);
				put_mtd_device(mtd);

				if (rv)
					status = OBMF_SPI_STATUS_TRANSFER_ERROR;
			} else if (opcode == SPI_NOR_PAGE_PROG && wr_size >= 4) {
				/* Page program: opcode(1) + addr(3) + data(N) */
				struct mtd_info *mtd;
				size_t retlen;
				u32 addr = obmf_spi_extract_addr(wr_data, wr_size);
				int data_len = wr_size - 4;
				int rv;

				if (data_len <= 0)
					break;

				mtd = get_mtd_device_nm(OBMF_SPI_MTD_NAME);
				if (IS_ERR(mtd)) {
					status = OBMF_STATUS_PERMANENT_ERROR;
					break;
				}

				rv = mtd_write(mtd, addr, data_len, &retlen,
					       wr_data + 4);
				put_mtd_device(mtd);

				if (rv < 0)
					status = OBMF_SPI_STATUS_TRANSFER_ERROR;
			}
			/* WREN, RDSR etc. — no-op, just ACK */
		}
		break;
	}

	case OBMF_SPI_CMD_WRITE_READ: {
		/* Write then read — handle SPI NOR read command */
		if (wr_size >= 4 && wr_data[0] == SPI_NOR_READ) {
			struct mtd_info *mtd;
			size_t retlen;
			u32 addr = obmf_spi_extract_addr(wr_data, wr_size);
			int rv;

			resp = kmalloc(3 + rd_size, GFP_KERNEL);
			if (!resp) {
				status = OBMF_STATUS_PERMANENT_ERROR;
				break;
			}

			mtd = get_mtd_device_nm(OBMF_SPI_MTD_NAME);
			if (IS_ERR(mtd)) {
				status = OBMF_STATUS_PERMANENT_ERROR;
				break;
			}

			rv = mtd_read(mtd, addr, rd_size, &retlen,
				      resp + 3);
			put_mtd_device(mtd);

			if (rv && rv != -EUCLEAN) {
				status = OBMF_SPI_STATUS_TRANSFER_ERROR;
				break;
			}

			resp[0] = cmd_byte;
			put_unaligned_le16((u16)retlen, &resp[1]);
			resp_len = 3 + retlen;

			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_SPI, status,
					   resp, resp_len);
			kfree(resp);
			return;
		}
		/* Other write-read combos: unsupported */
		status = OBMF_STATUS_INVALID_CMD;
		break;
	}

	case OBMF_SPI_CMD_POSTED_WRITE:
		/* Posted write: no response needed */
		kfree(resp);
		return;

	default:
		status = OBMF_STATUS_INVALID_CMD;
		break;
	}

	/* Send response (for non-read commands) */
	{
		u8 simple_resp[3];

		simple_resp[0] = cmd_byte;
		put_unaligned_le16(0, &simple_resp[1]);
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_SPI, status,
				   simple_resp, sizeof(simple_resp));
	}
	kfree(resp);
}

/* ------------------------------------------------------------------ */
/* register / unregister                                               */
/* ------------------------------------------------------------------ */

int obmf_spi_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_spi_data *sd;
	struct mtd_info *mtd;

	mtd = get_mtd_device_nm(OBMF_SPI_MTD_NAME);
	if (IS_ERR(mtd)) {
		dev_err(&odev->intf->dev,
			"ch%u: MTD '%s' not found: %ld\n",
			ch->channel_id, OBMF_SPI_MTD_NAME, PTR_ERR(mtd));
		return PTR_ERR(mtd);
	}

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd) {
		put_mtd_device(mtd);
		return -ENOMEM;
	}

	sd->ch  = ch;
	sd->mtd = mtd;
	ch->priv = sd;

	if (ch->kobj)
		sysfs_create_link(ch->kobj, &mtd->dev.kobj, "spi_flash");

	dev_info(&odev->intf->dev,
		 "ch%u: registered SPI Controller (MTD: %s)\n",
		 ch->channel_id, OBMF_SPI_MTD_NAME);
	return 0;
}

void obmf_spi_unregister(struct obmf_channel *ch)
{
	struct obmf_spi_data *sd = ch->priv;

	if (sd) {
		sysfs_remove_link(ch->kobj, "spi_flash");
		put_mtd_device(sd->mtd);
		kfree(sd);
		ch->priv = NULL;
	}
}
