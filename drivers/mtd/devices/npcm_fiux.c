// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Nuvoton Technology corporation.

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

#include <mtd/mtd-abi.h>

struct npcm_fiux {
	struct device	*dev;
	struct spi_mem	*spimem;
	struct mtd_info mtd_ram;
	u32		rx_dummy;
	struct {
		struct spi_mem_dirmap_desc *rdesc;
		struct spi_mem_dirmap_desc *wdesc;
	} dirmap;
};

#define MAP_SIZE_8MB			0x800000
#define FIUX_DRD_MAX_DUMMY_NUMBER	3

static int fiux_create_write_dirmap(struct npcm_fiux *flash)
{
	struct spi_mem_dirmap_info info = {
		.op_tmpl = SPI_MEM_OP(SPI_MEM_OP_CMD(0, 1),
				       SPI_MEM_OP_ADDR(1, 0, 0),
				       SPI_MEM_OP_NO_DUMMY,
				       SPI_MEM_OP_DATA_OUT(0, NULL, 1)),
		.offset = 0,
		.length = MAP_SIZE_8MB,
	};

	flash->dirmap.wdesc = spi_mem_dirmap_create(flash->spimem, &info);
	if (IS_ERR(flash->dirmap.wdesc))
		return PTR_ERR(flash->dirmap.wdesc);

	return 0;
}

static int fiux_create_read_dirmap(struct npcm_fiux *flash, u32 rx_dummy)
{
	struct spi_mem_dirmap_info info = {
		.op_tmpl = SPI_MEM_OP(SPI_MEM_OP_CMD(0, 1),
				      SPI_MEM_OP_ADDR(1, 0, 0),
				      SPI_MEM_OP_NO_DUMMY,
				      SPI_MEM_OP_DATA_IN(0, NULL, 1)),
		.offset = 0,
		.length = MAP_SIZE_8MB,
	};

	info.op_tmpl.dummy.nbytes = rx_dummy;
	flash->dirmap.rdesc = spi_mem_dirmap_create(flash->spimem, &info);
	if (IS_ERR(flash->dirmap.rdesc))
		return PTR_ERR(flash->dirmap.rdesc);

	return 0;
}

static int npcm_fiux_write(struct mtd_info *mtd, loff_t to, size_t len,
			       size_t *retlen, const u_char *buf)
{
	struct npcm_fiux *flash = mtd->priv;

	*retlen = spi_mem_dirmap_write(flash->dirmap.wdesc, to, len, buf);

	return *retlen;
}

static int npcm_fiux_read(struct mtd_info *mtd, loff_t from, size_t len,
			      size_t *retlen, u_char *buf)
{
	struct npcm_fiux *flash = mtd->priv;

	*retlen = spi_mem_dirmap_read(flash->dirmap.rdesc, from, len, buf);

	return *retlen;
}

static int npcm_fiux_probe(struct spi_mem *spimem)
{
	struct spi_device *spi = spimem->spi;
	struct flash_platform_data *data = dev_get_platdata(&spi->dev);
	struct npcm_fiux *flash;
	struct mtd_info *mtd;
	u32 rx_dummy;
	int ret;

	flash = devm_kzalloc(&spimem->spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	of_property_read_u32(spi->dev.of_node, "npcm,fiu-spix-rx-dummy-num",
			     &rx_dummy);
	if (rx_dummy > FIUX_DRD_MAX_DUMMY_NUMBER) {
		dev_warn(&spimem->spi->dev, "npcm,fiu-spix-rx-dummy-num %lu not supported\n",
			 (unsigned long)rx_dummy);
		rx_dummy = 0;
	}

	mtd = &flash->mtd_ram;

	spi_mem_set_drvdata(spimem, flash);
	flash->spimem = spimem;
	flash->dev = &spi->dev;

	/* Populate mtd_info data structure */
	*mtd = (struct mtd_info) {
		.name		= "mtd-ram-fiux",
		.type		= MTD_RAM,
		.priv		= flash,
		.size		= MAP_SIZE_8MB,
		.writesize	= 1,
		.writebufsize	= 1,
		.flags		= MTD_CAP_RAM,
		._read		= npcm_fiux_read,
		._write		= npcm_fiux_write,
		.owner 		= THIS_MODULE,
	};

	ret = fiux_create_write_dirmap(flash);
	if (ret)
		return ret;

	ret = fiux_create_read_dirmap(flash, rx_dummy);
	if (ret)
		goto err_destroy_write_dirmap;


	ret = mtd_device_register(mtd, data ? data->parts : NULL,
				   data ? data->nr_parts : 0);
	if (ret)
		goto err_destroy_read_dirmap;

	pr_info("NPCM fiux MTD RAM probed\n");

	return 0;

err_destroy_read_dirmap:
	spi_mem_dirmap_destroy(flash->dirmap.rdesc);

err_destroy_write_dirmap:
	spi_mem_dirmap_destroy(flash->dirmap.wdesc);

	return ret;
}


static int npcm_fiux_remove(struct spi_mem *spimem)
{
	struct npcm_fiux *flash = spi_mem_get_drvdata(spimem);

	/* Clean up MTD stuff. */
	return mtd_device_unregister(&flash->mtd_ram);
}

static const struct spi_device_id npcm_fiux_ids[] = {
	/* Flashes that can't be detected using JEDEC */
	{"ram-nonjedec"},
	{ },
};
MODULE_DEVICE_TABLE(spi, npcm_fiux_ids);

static const struct of_device_id npcm_fiux_of_table[] = {
	/*
	 * Generic compatibility for SPI NOR that can be identified by the
	 * JEDEC READ ID opcode (0x9F). Use this, if possible.
	 */
	{ .compatible = "nuvoton,npcm-fiux" },
	{}
};
MODULE_DEVICE_TABLE(of, npcm_fiux_of_table);

static struct spi_mem_driver npcm_fiux_driver = {
	.spidrv = {
		.driver = {
			.name = "npcm-fiux",			
			.of_match_table = npcm_fiux_of_table,
		},
		.id_table = npcm_fiux_ids,
	},
	.probe	= npcm_fiux_probe,
	.remove	= npcm_fiux_remove,
};

module_spi_mem_driver(npcm_fiux_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomer Maimon");
MODULE_DESCRIPTION("MTD RAM driver for NPCM FIUx");
