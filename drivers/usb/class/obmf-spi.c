// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-spi.c - OBMF-ICP SPI Controller Channel (Type 08h, v0.9)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Implements the SPI Controller Optimised Channel using v0.9 protocol.
 * This channel is always Consumer/Requester in this driver: it registers
 * a plain virtual MTD device (a self-contained struct mtd_info with
 * custom _read/_write/_erase callbacks; no dependency on the mtdram
 * driver), with geometry taken from DTS.  Each MTD op is translated into
 * a single SPI NOR opcode (READ/PP/SE/BE) sent through OBMF to the SMC.
 * The SMC side owns its own FIU (Flash Interface Unit) SPI controller
 * driver and constructs the complete bus sequence itself (CS assert,
 * WREN before PP/Erase, WIP polling, CS deassert) before replying —
 * i.e. an OBMF request here is equivalent to calling flash read/write/
 * erase from SMC-side userspace.  This driver never sends WREN/RDSR.
 * No local RAM/flash backing is used — every op is forwarded over USB
 * to the SMC in real time.
 *
 * Device-initiated requests (the SMC pushing a Producer-side memory
 * access) belong to the Memory Device Channel (Type 0Ah); see
 * obmf-mem-dev.c.  A device-initiated request on this channel is
 * therefore always rejected here.
 *
 * SPI Request format (OBMF Type 08h):
 *   Command(1B): [7:6]=CS#, [5]=Assert CS, [4]=Deassert CS, [3:0]=SPI cmd
 *   WriteDataSize(2B LE) + ReadDataSize(2B LE) + WriteData(N)
 *
 * SPI Response format:
 *   Command(1B) + ReadDataSize(2B LE) + ReadData(N)
 *   Status in Common Header byte 2[7:1]
 *
 * NOTE: the req/resp field layout above reflects the current driver
 * implementation.  Known divergences from the v1.0.0 spec are listed at
 * the bottom of this file.
 */

#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/usb.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

/* Maximum total SPI channel payload (sub-header + data) for a single
 * OBMF Type 08h transfer; directly sizes the req/resp stack buffers
 * inside obmf_spi_xfer().  The usable data portion is smaller by
 * OBMF_SPI_REQ_SUBHDR_SIZE (write) or OBMF_SPI_RESP_SUBHDR_SIZE (read).
 */
#define OBMF_SPI_MAX_XFER	448

/* SPI Type 08h sub-header sizes, in addition to the OBMF Common Header
 * (OBMF_COMMON_HDR_SIZE) that obmf_send_request() wraps every request
 * and response with:
 *   Request  sub-header: cmd_byte(1) + WriteDataSize(2) + ReadDataSize(2)
 *   Response sub-header: cmd_echo(1) + ReadDataSize(2)
 */
#define OBMF_SPI_REQ_SUBHDR_SIZE	5
#define OBMF_SPI_RESP_SUBHDR_SIZE	3

struct obmf_spi_data {
	struct obmf_channel	*ch;

	/* mtd is self-allocated by this driver; no local data backing —
	 * every _read/_write/_erase forwards a single SPI NOR opcode over
	 * OBMF.  The SMC's own FIU driver constructs the full bus sequence
	 * (CS, WREN, WIP-wait) internally; we never send WREN/RDSR ourselves.
	 */
	struct mtd_info		*mtd;
	u8			cs;		/* chip-select number (cmd_byte[7:6]) */
	u8			addr_nbytes;	/* 3 or 4, based on flash size */
	u8			erase_opcode;	/* SPI_NOR_SECTOR_ERASE or _BLOCK_ERASE */
};

/**
 * obmf_spi_find_ch_node - find and return the DT child node for this channel.
 * @ch: OBMF channel
 *
 * Caller must call of_node_put() on the returned node.
 */
static struct device_node *obmf_spi_find_ch_node(struct obmf_channel *ch)
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

/* SPI NOR opcodes.  Only opcodes that carry addr/data over the wire are
 * needed here — WREN/RDSR are handled internally by the SMC's own FIU
 * (Flash Interface Unit) driver and are never sent by this driver; see
 * the file header for details.
 */
#define SPI_NOR_READ		0x03
#define SPI_NOR_PAGE_PROG	0x02
#define SPI_NOR_SECTOR_ERASE	0x20
#define SPI_NOR_BLOCK_ERASE	0xD8

/* ------------------------------------------------------------------ */
/* Host-initiated SPI transfer                                         */
/* ------------------------------------------------------------------ */

static int obmf_spi_xfer(struct obmf_channel *ch, u8 cmd_byte,
			 const u8 *wr_data, u16 wr_len,
			 u8 *rd_data, u16 rd_len)
{
	struct obmf_device *odev = ch->odev;
	/* req:  sub-header(5) + wr_data; total <= OBMF_SPI_MAX_XFER
	 * resp: sub-header(4) + rd_data; total <= OBMF_SPI_MAX_XFER
	 */
	u8 req[OBMF_SPI_MAX_XFER];
	u8 resp[OBMF_SPI_MAX_XFER];
	int req_len, rv;

	if (WARN_ON(wr_len > OBMF_SPI_MAX_XFER - OBMF_SPI_REQ_SUBHDR_SIZE ||
		    rd_len > OBMF_SPI_MAX_XFER - OBMF_SPI_RESP_SUBHDR_SIZE))
		return -EINVAL;

	req[0] = cmd_byte;
	put_unaligned_le16(wr_len, &req[1]);
	put_unaligned_le16(rd_len, &req[3]);
	req_len = 5;

	if (wr_data && wr_len > 0) {
		memcpy(req + 5, wr_data, wr_len);
		req_len += wr_len;
	}

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch,
			       req, req_len, resp, sizeof(resp),
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);

	if (rv < 0)
		return rv;

	/* Parse response: cmd(1) + rd_size(2) + rd_data(N) */
	if (rv >= OBMF_SPI_RESP_SUBHDR_SIZE && rd_data && rd_len > 0) {
		u16 actual_rd = get_unaligned_le16(&resp[1]);
		int copy = min_t(u16, actual_rd, rd_len);

		if (copy > 0) {
			/* resp[] must actually contain the rd_size bytes it
			 * claims; if the transport truncated the response
			 * (e.g. Common Header size didn't cover the read
			 * data), do NOT report success with an untouched
			 * rd_data buffer — that would silently return stale/
			 * uninitialised caller memory as if it were valid
			 * flash data.
			 */
			if (rv < OBMF_SPI_RESP_SUBHDR_SIZE + copy)
				return -EIO;
			memcpy(rd_data, resp + OBMF_SPI_RESP_SUBHDR_SIZE, copy);
		}
		return copy;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Device-initiated SPI request handler                                */
/* ------------------------------------------------------------------ */

void obmf_spi_handle_dev_request(struct obmf_channel *ch,
				 const u8 *data, int len)
{
	/* This channel is always Consumer/Requester in this driver; a
	 * device-initiated memory access belongs to the Memory Device
	 * Channel (Type 0Ah), see obmf-mem-dev.c.
	 */
	obmf_send_response(ch->odev, ch->channel_id,
			   OBMF_STATUS_INVALID_CMD, NULL, 0);
}

/* ------------------------------------------------------------------ */
/* spi-nor virtual MTD device (host-initiated, no local data backing)  */
/* ------------------------------------------------------------------ */

/*
 * obmf_nor_cmd_byte - build the OBMF cmd_byte for a given SPI command
 * type.  Every op is a self-contained CS cycle (Assert+Deassert set),
 * see the discussion in the file header.
 */
static inline u8 obmf_nor_cmd_byte(struct obmf_spi_data *sd, u8 cmd_type)
{
	return cmd_type | (sd->cs << OBMF_SPI_CS_NUM_SHIFT) |
	       OBMF_SPI_CS_ASSERT | OBMF_SPI_CS_DEASSERT;
}

/* obmf_nor_fill_addr - encode @addr as sd->addr_nbytes big-endian bytes */
static void obmf_nor_fill_addr(struct obmf_spi_data *sd, u8 *buf, u32 addr)
{
	int i;

	for (i = 0; i < sd->addr_nbytes; i++)
		buf[i] = addr >> (8 * (sd->addr_nbytes - i - 1));
}

/*
 * obmf_spi_max_wr_payload - largest SPI request payload (opcode + addr +
 * write data) that fits in a single Bulk OUT transfer, after subtracting
 * the OBMF Common Header and SPI request sub-header from the device's
 * negotiated max_wr_transfer_size.  Always capped at OBMF_SPI_MAX_XFER,
 * which sizes the stack buffer in obmf_spi_xfer().
 */
static u32 obmf_spi_max_wr_payload(struct obmf_channel *ch)
{
	u32 overhead = OBMF_COMMON_HDR_SIZE + OBMF_SPI_REQ_SUBHDR_SIZE;
	u32 avail;

	if (ch->odev->max_wr_transfer_size <= overhead)
		return 0;
	avail = ch->odev->max_wr_transfer_size - overhead;
	return min_t(u32, avail, OBMF_SPI_MAX_XFER - OBMF_SPI_REQ_SUBHDR_SIZE);
}

/*
 * obmf_spi_max_rd_payload - largest SPI response payload (read data) that
 * fits in a single Bulk IN transfer, after subtracting the OBMF Common
 * Header and SPI response sub-header from the device's negotiated
 * max_rd_transfer_size.  Always capped at OBMF_SPI_MAX_XFER, which sizes
 * the stack buffer in obmf_spi_xfer().
 */
static u32 obmf_spi_max_rd_payload(struct obmf_channel *ch)
{
	u32 overhead = OBMF_COMMON_HDR_SIZE + OBMF_SPI_RESP_SUBHDR_SIZE;
	u32 avail;

	if (ch->odev->max_rd_transfer_size <= overhead)
		return 0;
	avail = ch->odev->max_rd_transfer_size - overhead;
	return min_t(u32, avail, OBMF_SPI_MAX_XFER - OBMF_SPI_RESP_SUBHDR_SIZE);
}

static int obmf_nor_read(struct mtd_info *mtd, loff_t from, size_t len,
			 size_t *retlen, u_char *buf)
{
	struct obmf_spi_data *sd = mtd->priv;
	u32 max_data = obmf_spi_max_rd_payload(sd->ch);

	if (!max_data)
		return -EMSGSIZE;

	*retlen = 0;
	while (len > 0) {
		u8 req[1 + 4];
		u16 chunk = min_t(size_t, len, max_data);
		int rv;

		req[0] = SPI_NOR_READ;
		obmf_nor_fill_addr(sd, req + 1, (u32)from);

		rv = obmf_spi_xfer(sd->ch,
				   obmf_nor_cmd_byte(sd, OBMF_SPI_CMD_WRITE_READ),
				   req, 1 + sd->addr_nbytes, buf, chunk);
		if (rv < 0)
			return rv;
		if (rv == 0)
			break;

		buf   += rv;
		from  += rv;
		len   -= rv;
		*retlen += rv;
	}
	return 0;
}

static int obmf_nor_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const u_char *buf)
{
	struct obmf_spi_data *sd = mtd->priv;
	u32 max_payload = obmf_spi_max_wr_payload(sd->ch);
	u32 max_chunk;

	if (max_payload <= 1 + sd->addr_nbytes)
		return -EMSGSIZE;
	max_chunk = max_payload - 1 - sd->addr_nbytes;

	*retlen = 0;
	while (len > 0) {
		/* opcode + addr(<=4) + data(<=max_chunk); max_chunk is capped
		 * to fit within OBMF_SPI_MAX_XFER, so this buffer always fits.
		 */
		u8 req[OBMF_SPI_MAX_XFER];
		u32 addr = (u32)to;
		u32 chunk = min_t(u32, len, max_chunk);
		int rv;

		/* Single self-contained page-program request: the SMC's FIU
		 * driver owns the flash's page geometry and handles CS, WREN,
		 * page-wrap-safe splitting and WIP-wait internally — this
		 * driver only chunks to fit the OBMF transport, not to any
		 * flash page boundary.
		 */
		req[0] = SPI_NOR_PAGE_PROG;
		obmf_nor_fill_addr(sd, req + 1, addr);
		memcpy(req + 1 + sd->addr_nbytes, buf, chunk);

		rv = obmf_spi_xfer(sd->ch, obmf_nor_cmd_byte(sd, OBMF_SPI_CMD_WRITE),
				   req, 1 + sd->addr_nbytes + chunk, NULL, 0);
		if (rv < 0)
			return rv;

		buf   += chunk;
		to    += chunk;
		len   -= chunk;
		*retlen += chunk;
	}
	return 0;
}

static int obmf_nor_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct obmf_spi_data *sd = mtd->priv;
	u32 erasesize = mtd->erasesize;
	u32 addr = (u32)instr->addr;
	u32 len = (u32)instr->len;
	int rv = 0;

	if ((addr % erasesize) || (len % erasesize)) {
		instr->fail_addr = addr;
		return -EINVAL;
	}

	while (len > 0) {
		u8 req[1 + 4];

		/* Single self-contained erase request; the SMC's FIU driver
		 * handles WREN + WIP polling internally, see obmf_nor_write().
		 */
		req[0] = sd->erase_opcode;
		obmf_nor_fill_addr(sd, req + 1, addr);

		rv = obmf_spi_xfer(sd->ch, obmf_nor_cmd_byte(sd, OBMF_SPI_CMD_WRITE),
				   req, 1 + sd->addr_nbytes, NULL, 0);
		if (rv < 0)
			goto fail;

		addr += erasesize;
		len  -= erasesize;
	}
	return 0;

fail:
	instr->fail_addr = addr;
	return rv;
}

/*
 * obmf_spi_register_nor - register a virtual MTD device whose
 * _read/_write/_erase callbacks forward a single SPI NOR opcode over
 * OBMF to the SMC (no local data backing).  The SMC's own FIU driver
 * constructs the full flash-access sequence (CS, WREN, WIP-wait)
 * before replying, so this driver never sends WREN/RDSR itself.
 *
 * Geometry is taken from the channel DT node:
 *   nuvoton,flash-size       (required, bytes)
 *   nuvoton,flash-erasesize  (optional, default 4096; use 65536 for
 *                             block-erase-only flash)
 *   nuvoton,mtd-name         (optional, default "npcm-spi-flash")
 *   nuvoton,flash-cs         (optional, default 0)
 *
 * mtd->writesize is fixed at 1, per the Linux MTD convention for NOR
 * flash ("minimal writable flash unit"; NOR can be written at byte
 * granularity).  Page-boundary handling for Page Program is entirely
 * the SMC FIU driver's responsibility — this driver only chunks writes
 * to fit the OBMF transport (OBMF_SPI_MAX_XFER), not to any flash page
 * geometry.
 *
 * NOTE: addr_nbytes is derived from flash-size (>16MiB => 4-byte address).
 * 4-byte addressing requires the flash to already be in 4-byte address
 * mode (EN4B) or to natively default to it; this driver does not yet send
 * EN4B, see TODO at the bottom of this file.
 */
static int obmf_spi_register_nor(struct obmf_device *odev,
				 struct obmf_channel *ch,
				 struct obmf_spi_data *sd)
{
	struct device_node *ch_np;
	struct mtd_info *mtd;
	const char *name = "npcm-spi-flash";
	u32 size = 0, erasesize = SZ_4K, cs = 0;
	int rv;

	ch_np = obmf_spi_find_ch_node(ch);
	if (ch_np) {
		of_property_read_u32(ch_np, "nuvoton,flash-size", &size);
		of_property_read_u32(ch_np, "nuvoton,flash-erasesize", &erasesize);
		of_property_read_u32(ch_np, "nuvoton,flash-cs", &cs);
		of_property_read_string(ch_np, "nuvoton,mtd-name", &name);
		of_node_put(ch_np);
	}

	if (!size) {
		dev_err(&odev->intf->dev,
			"ch%u: missing nuvoton,flash-size for spi-nor mode\n",
			ch->channel_id);
		return -EINVAL;
	}

	sd->cs           = (u8)cs;
	sd->addr_nbytes  = (size > SZ_16M) ? 4 : 3;
	sd->erase_opcode = (erasesize == SZ_64K) ? SPI_NOR_BLOCK_ERASE
						  : SPI_NOR_SECTOR_ERASE;

	mtd = kzalloc(sizeof(*mtd), GFP_KERNEL);
	if (!mtd)
		return -ENOMEM;

	mtd->owner     = THIS_MODULE;
	mtd->type      = MTD_NORFLASH;
	mtd->flags     = MTD_CAP_NORFLASH;
	mtd->size      = size;
	mtd->erasesize = erasesize;
	mtd->writesize = 1;
	mtd->name      = name;
	mtd->dev.parent = &odev->intf->dev;
	mtd->_read     = obmf_nor_read;
	mtd->_write    = obmf_nor_write;
	mtd->_erase    = obmf_nor_erase;
	mtd->priv      = sd;

	rv = mtd_device_register(mtd, NULL, 0);
	if (rv) {
		dev_err(&odev->intf->dev,
			"ch%u: mtd_device_register failed: %d\n",
			ch->channel_id, rv);
		kfree(mtd);
		return rv;
	}

	sd->mtd = mtd;
	ch->sysfs_dev = &mtd->dev;

	dev_info(&odev->intf->dev,
		 "ch%u: registered virtual SPI-NOR MTD '%s' (%u bytes, erasesize %u, cs%u)\n",
		 ch->channel_id, name, size, erasesize, cs);
	return 0;
}

/* ------------------------------------------------------------------ */
/* register / unregister                                               */
/* ------------------------------------------------------------------ */

int obmf_spi_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_spi_data *sd;
	int rv;

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	sd->ch   = ch;
	ch->priv = sd;

	rv = obmf_spi_register_nor(odev, ch, sd);
	if (rv) {
		kfree(sd);
		ch->priv = NULL;
	}
	return rv;
}

void obmf_spi_unregister(struct obmf_channel *ch)
{
	struct obmf_spi_data *sd = ch->priv;

	if (!sd)
		return;

	/*
	 * mtd_device_unregister() unbinds any bound higher-layer users
	 * (UBI, etc.); the mtd_info itself was self-allocated in
	 * obmf_spi_register_nor() and must be freed here.
	 */
	if (sd->mtd) {
		mtd_device_unregister(sd->mtd);
		kfree(sd->mtd);
	}

	kfree(sd);
	ch->priv = NULL;
}
