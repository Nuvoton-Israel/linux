// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Nuvoton Technology corporation.

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/mailbox_client.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>

#define NPCM_FIU_TIP_CMD	0x0
#define NPCM_FIU_TIP_SRC	0x4
#define NPCM_FIU_TIP_DST	0x8
#define NPCM_FIU_TIP_SIZE	0xC

#define NPCM_FIU_TIP_FLASH_SIZE	0x8
#define NPCM_FIU_TIP_PAGE_SIZE	0xC
#define NPCM_FIU_TIP_BLOCK_SIZE	0x10
#define NPCM_FIU_TIP_ID		0x18

#define	NPCM_FIU_TIP_WRITE_CMD	1
#define	NPCM_FIU_TIP_READ_CMD	2
#define	NPCM_FIU_TIP_ERASE_CMD	4	
#define NPCM_FIU_TIP_DETAIL_CMD	5

#define FIU_TIP_READ_TIMEOUT	500
#define FIU_TIP_WRITE_TIMEOUT	1000
#define FIU_TIP_ERASE_TIMEOUT	1500

#define MIN_FIU_BUFFER		0x2000

#define MAP_SIZE_128MB		0x8000000
#define MAP_SIZE_16MB		0x1000000
#define MAP_SIZE_8MB		0x800000

enum {
	FIU0 = 0,
	FIU1,
	FIU3,
	FIUX,
};

struct npcm_fiu_tip_info {
	char *name;
	u32 fiu_id;
	u32 offset;
	u32 max_map_size;
	u32 max_cs;
};

struct fiu_data {
	const struct npcm_fiu_tip_info *npcm_fiu_data_info;
	int fiu_max;
};

static const struct npcm_fiu_tip_info npxm8xx_fiu_info[] = {
	{.name = "FIU0", .fiu_id = FIU0, .offset = 0x80000000,
		.max_map_size = MAP_SIZE_128MB, .max_cs = 2},
	{.name = "FIU1", .fiu_id = FIU1, .offset = 0x90000000,
		.max_map_size = MAP_SIZE_16MB, .max_cs = 4}, 
	{.name = "FIU3", .fiu_id = FIU3, .offset = 0xA0000000,
		.max_map_size = MAP_SIZE_128MB, .max_cs = 4},
	{.name = "FIUX", .fiu_id = FIUX, .offset = 0xF8000000,
		.max_map_size = MAP_SIZE_16MB, .max_cs = 2} };

static const struct fiu_data npxm8xx_fiu_data = {
	.npcm_fiu_data_info = npxm8xx_fiu_info,
	.fiu_max = 4,
};

struct npcm_fiu_tip_header {
	struct device			*dev;
	struct mbox_client		cl;
	struct mbox_chan		*chan;
	struct mutex 			mutex;
	void __iomem			*reg;
	bool				event_received;
	struct completion 		wait_event;
	resource_size_t 		phy_start;
	resource_size_t 		phy_size;
	u8 __iomem			*virt_off;
};

struct npcm_fiu_tip {
	struct npcm_fiu_tip_header	*head;
	u32				fiu_id;
	u32				cs;
	u32 				offset;
	const struct npcm_fiu_tip_info	*info;
	struct mtd_info			mtd;
};

static int npcm_fiu_tip_write(struct mtd_info *mtd, loff_t to, size_t len,
			       size_t *retlen, const u_char *buf)
{
	struct npcm_fiu_tip *flash = mtd->priv;
	int err;

	mutex_lock(&flash->head->mutex);

	memcpy(flash->head->virt_off + (flash->head->phy_size/2), buf, len);
	writel(NPCM_FIU_TIP_WRITE_CMD, flash->head->reg + NPCM_FIU_TIP_CMD);
	writel((u32)(flash->head->phy_start + (flash->head->phy_size/2)), flash->head->reg + NPCM_FIU_TIP_SRC);
	writel((u32)to + flash->offset, flash->head->reg + NPCM_FIU_TIP_DST);
	writel(len, flash->head->reg + NPCM_FIU_TIP_SIZE);
	*retlen = len;

	err = mbox_send_message(flash->head->chan, flash->head->reg);
	if (err >= 0) {
		if (!wait_for_completion_timeout(&flash->head->wait_event,
						 msecs_to_jiffies(FIU_TIP_WRITE_TIMEOUT))) {
			mutex_unlock(&flash->head->mutex);
			dev_err(flash->head->dev, "Flash write timeout\n");
			return -ETIMEDOUT;
		}
	} else {
		mutex_unlock(&flash->head->mutex);
		dev_err(flash->head->dev, "mbox_send_message returned %d\n", err);
		return err;
	}

	mutex_unlock(&flash->head->mutex);

	return readl(flash->head->reg + NPCM_FIU_TIP_CMD);
}

static int npcm_fiu_tip_read(struct mtd_info *mtd, loff_t from, size_t len,
			      size_t *retlen, u_char *buf)
{
	struct npcm_fiu_tip *flash = mtd->priv;
	int err;

	mutex_lock(&flash->head->mutex);

	writel(NPCM_FIU_TIP_READ_CMD, flash->head->reg + NPCM_FIU_TIP_CMD);
	writel((u32)from + flash->offset, flash->head->reg + NPCM_FIU_TIP_SRC);
	writel((u32)flash->head->phy_start, flash->head->reg + NPCM_FIU_TIP_DST);
	writel(len, flash->head->reg + NPCM_FIU_TIP_SIZE);
	*retlen = len;

	err = mbox_send_message(flash->head->chan, flash->head->reg);
	if (err >= 0) {
		if (!wait_for_completion_timeout(&flash->head->wait_event,
						 msecs_to_jiffies(FIU_TIP_READ_TIMEOUT))) {
			mutex_unlock(&flash->head->mutex);
			dev_err(flash->head->dev, "Flash read timeout\n");
			return -ETIMEDOUT;
		}
	} else {
		mutex_unlock(&flash->head->mutex);
		dev_err(flash->head->dev, "mbox_send_message returned %d\n", err);
		return err;
	}

	memcpy(buf, flash->head->virt_off, len);

	mutex_unlock(&flash->head->mutex);

	return readl(flash->head->reg + NPCM_FIU_TIP_CMD);
}

static int npcm_fiu_tip_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct npcm_fiu_tip *flash = mtd->priv;
	u32 addr, len, err;

	addr = instr->addr - (instr->addr % mtd->erasesize);
	len = instr->len;

	mutex_lock(&flash->head->mutex);

	/* "sector"-at-a-time erase */
	while (len) {
		writel(NPCM_FIU_TIP_ERASE_CMD, flash->head->reg + NPCM_FIU_TIP_CMD);
		writel((u32)addr + flash->offset, flash->head->reg + NPCM_FIU_TIP_DST);
		writel(mtd->erasesize, flash->head->reg + NPCM_FIU_TIP_SIZE);

		err = mbox_send_message(flash->head->chan, flash->head->reg);
		if (err >= 0) {
			if (!wait_for_completion_timeout(&flash->head->wait_event,
							 msecs_to_jiffies(FIU_TIP_ERASE_TIMEOUT))) {
				mutex_unlock(&flash->head->mutex);
				dev_err(flash->head->dev, "Flash erase timeout\n");
				return -ETIMEDOUT;
			}
		} else {
			mutex_unlock(&flash->head->mutex);
			dev_err(flash->head->dev, "mbox_send_message returned %d\n", err);
			return err;
		}

		addr += mtd->erasesize;
		len -= mtd->erasesize;

		err = readl(flash->head->reg + NPCM_FIU_TIP_CMD);
		if (err) {
			mutex_unlock(&flash->head->mutex);
			return err;
		}
	}

	mutex_unlock(&flash->head->mutex);

	return err;
}

static void msg_from_tip_fiu(struct mbox_client *cl, void *msg)
{
	struct npcm_fiu_tip_header *head = container_of(cl, struct npcm_fiu_tip_header, cl);

	complete(&head->wait_event);
}


static int npcm_fiu_tip_set_mtd_info(struct npcm_fiu_tip *flash)
{
	struct mtd_info *mtd = &flash->mtd;
	uint32_t flash_id, flash_id_le;
	int err;

	writel(NPCM_FIU_TIP_DETAIL_CMD, flash->head->reg + NPCM_FIU_TIP_CMD);
	writel((u32)flash->offset, flash->head->reg + NPCM_FIU_TIP_SRC);
	writel(0xFFFFFFFF, flash->head->reg + NPCM_FIU_TIP_DST);
	writel(0x2, flash->head->reg + NPCM_FIU_TIP_SIZE);

	err = mbox_send_message(flash->head->chan, flash->head->reg);
	if (err >= 0) {
		if (!wait_for_completion_timeout(&flash->head->wait_event,
						 msecs_to_jiffies(FIU_TIP_ERASE_TIMEOUT))) {
			dev_err(flash->head->dev, "Flash write timeout\n");
			return -ETIMEDOUT;
		}
	} else {
		dev_warn(flash->head->dev, "mbox_send_message returned %d\n", err);
		return err;
	}

	err = readl(flash->head->reg + NPCM_FIU_TIP_CMD);
	if (err)
		return err;

	mtd->size = readl(flash->head->reg + NPCM_FIU_TIP_FLASH_SIZE);
	mtd->writebufsize = readl(flash->head->reg + NPCM_FIU_TIP_PAGE_SIZE);
	mtd->erasesize = readl(flash->head->reg + NPCM_FIU_TIP_BLOCK_SIZE);
	flash_id_le = readl(flash->head->reg + NPCM_FIU_TIP_ID);
	flash_id = ((flash_id_le >> 16) & 0xFF) | ((flash_id_le << 16) & 0xFF0000) | (flash_id_le & 0xFF00);

	mtd->writesize = 1;
	if ((!mtd->size) || (mtd->size > flash->info->max_map_size) || (!mtd->writebufsize) || (!mtd->erasesize)) {
		dev_warn(flash->head->dev,
			 "Flash not found 0x%x "
			 ".size = 0x%llx (%lldMiB), .erasesize = 0x%.8x (%uKiB)\n",
			 flash_id, (long long)mtd->size, 
			 (long long)(mtd->size >> 20),
			 mtd->erasesize, mtd->erasesize / 1024);
		return -EINVAL;
	}

	dev_info(flash->head->dev, "Flash ID 0x%x (%lld Kbytes)\n", 
		 flash_id, (long long)mtd->size >> 10);

	dev_dbg(flash->head->dev,
		"Flash ID 0x%x .size = 0x%llx (%lldMiB), "
		".erasesize = 0x%.8x (%uKiB)\n",
		flash_id, (long long)mtd->size, (long long)(mtd->size >> 20),
		mtd->erasesize, mtd->erasesize / 1024);

	return err;
}

static int npcm_fiu_tip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct fiu_data *fiu_data_match;
	struct npcm_fiu_tip_header *head;
	struct fwnode_handle *child;
	struct device_node *tipsh;
	bool flash_attach = false;
	struct resource t_res;
	int ret, cs, id;

	head = devm_kzalloc(dev, sizeof(*head), GFP_KERNEL);
	if (!head)
		return -ENOMEM;

	head->dev = dev;

	head->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(head->reg))
		return PTR_ERR(head->reg);

	fiu_data_match = of_device_get_match_data(dev);
	if (!fiu_data_match) {
		dev_err(dev, "No compatible OF match\n");
		return -ENODEV;
	}

	if (device_get_child_node_count(dev) == 0) {
		dev_err(dev, "flash is missing\n");
		return -EINVAL;
	}

	mutex_init(&head->mutex);
	init_completion(&head->wait_event);
	/* set up and request mailbox channel */
	head->cl.dev = dev;
	head->cl.rx_callback = msg_from_tip_fiu; /* read non-blocking */
	head->cl.tx_done = NULL;
	head->cl.tx_block = true; /* write blocking */
	head->cl.tx_tout = 500; /* msec */
	head->chan = mbox_request_channel_byname(&head->cl, "fiu_tip");
	if (IS_ERR(head->chan)) {
		dev_err(dev, "mbox channel request failed\n");
		ret = PTR_ERR(head->chan);
	}

	platform_set_drvdata(pdev, head);

	tipsh = of_parse_phandle(np, "memory-region", 0);
	if (tipsh) {
		ret = of_address_to_resource(tipsh, 0, &t_res);
		of_node_put(tipsh);
		if (ret) {
			dev_err(dev, "Failed to get shared mem resource\n");
			return ret;
		}

		head->phy_start = t_res.start;
		head->phy_size = resource_size(&t_res);
		if (head->phy_size < MIN_FIU_BUFFER) {
			pr_info("Error, size of the shared memory is less then 2KB\n");
			return -ENODEV;
		}
		head->virt_off = devm_ioremap_resource(dev, &t_res);
		if (IS_ERR(head->virt_off)) {
			dev_err(dev, "device mem io remap failed\n");
			return PTR_ERR(head->virt_off);
		}
	} else {
		pr_info("Error finding shared memory\n");
		return -ENODEV;
	}

	device_for_each_child_node(dev, child) {
		struct npcm_fiu_tip *flash;
		struct mtd_info *mtd;

		flash = devm_kzalloc(dev, sizeof(*flash), GFP_KERNEL);
		if (!flash)
			return -ENOMEM;

		mtd = &flash->mtd;
		flash->head = head;
		if (fwnode_property_read_u32(child, "nuvoton,npcm-fiu-tip-num", &id)) {
			dev_err(dev, "FIU TIP number error\n");
			fwnode_handle_put(child);
			kfree(flash);
			continue;
		}
		if (id >= fiu_data_match->fiu_max) {
			dev_err(dev, "FIU TIP ecxeed chip FIU number error id %d\n",id);
			kfree(flash);
			continue;
		}
		flash->fiu_id = id;
		flash->info = &fiu_data_match->npcm_fiu_data_info[id];
		if (fwnode_property_read_u32(child, "nuvoton,npcm-fiu-tip-cs", &cs)) {
			dev_err(dev, "FIU TIP chip select error\n");
			fwnode_handle_put(child);
			kfree(flash);
			continue;
		}
		if (cs > flash->info->max_cs) {
			dev_err(dev, "FIU TIP ecxeed chip cs %d\n",cs);
			kfree(flash);
			continue;
		}
		flash->cs = cs;

		if (fwnode_property_read_string(child, "label", &mtd->name)) {
			dev_warn(dev, "FIU id->%d cs->%d without label, set name as device\n", id, cs);
			mtd->name = NULL;
		}

		np = to_of_node(child);
		if (!np) {
			dev_err(dev, "child device node error\n");
			fwnode_handle_put(child);
			kfree(flash);
			continue;
		}

		flash->offset = flash->info->offset + (flash->info->max_map_size * cs);

		mtd_set_of_node(&flash->mtd, np);
		if (npcm_fiu_tip_set_mtd_info(flash)) {
			dev_err(dev, "npcm_fiu_tip_set_mtd_info failed\n");
			fwnode_handle_put(child);
			kfree(flash);
			continue;
		}

		mtd->dev.parent = dev;
		if (!mtd->name)
			mtd->name = dev_name(dev);
		mtd->type = MTD_NORFLASH;
		mtd->flags = MTD_CAP_NORFLASH;
		mtd->_erase = npcm_fiu_tip_erase;
		mtd->_read = npcm_fiu_tip_read;
		mtd->_write = npcm_fiu_tip_write;
		mtd->priv = flash;

		if (mtd_device_register(&flash->mtd, NULL, 0)) {
			dev_err(dev, "mtd_device_register error\n");
			fwnode_handle_put(child);
			kfree(flash);
		}

		flash_attach = true;
	}

	if (!flash_attach) {
		dev_info(dev, "No Flash correct, freeing mbox chan");
		mbox_free_channel(head->chan);
		return -ENODEV;
	}

	return 0;
}

static int npcm_fiu_tip_remove(struct platform_device *pdev)
{
	struct npcm_fiu_tip *flash = platform_get_drvdata(pdev);

	mbox_free_channel(flash->head->chan);

	return 0;
}

static const struct of_device_id npcm_fiu_tip_of_table[] = {
	{ .compatible = "nuvoton,npcm845-fiu-tip", .data = &npxm8xx_fiu_data },
	{}
};
MODULE_DEVICE_TABLE(of, npcm_fiu_tip_of_table);

static struct platform_driver npcm_fiu_tip_driver = {
	.probe    =  npcm_fiu_tip_probe,
	.remove   =  npcm_fiu_tip_remove,
	.driver   = {
	    .name  = "npcm_fiu_tip",
	    .of_match_table = npcm_fiu_tip_of_table,
	    .owner = THIS_MODULE,
	},
};

module_platform_driver(npcm_fiu_tip_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("tomer.maimon@nuvoton.com");
MODULE_DESCRIPTION("Nuvoton NPCM MTD TIP FIUx driver");
