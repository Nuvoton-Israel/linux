#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>

#define CREATE_TRACE_POINTS
#include <trace/events/npcm_flash.h>

#define DEVICE_NAME "npcm-espi-flash"
#define ESPIIE 0x0C
#define  ESPIIE_CFGUPDIE BIT(1)
#define  ESPIIE_FLASHRXIE BIT(4)
#define  ESPIIE_SFLASHRDIE BIT(5)

#define ESPICFG 0x04
#define  ESPICFG_HFLASHCHANEN BIT(7)
#define  ESPICFG_FLASHCHANEN BIT(3)

#define ESPISTS 0x08
#define  ESPISTS_FLASHRX BIT(4)
#define  ESPISTS_SFLASHRD BIT(5)
#define  ESPISTS_CFGUPD BIT(1)

#define FLASHRXRDHEAD 0x28
#define FLASHTXWRHEAD 0x2C

#define FLASHCFG 0x34
#define  TRGFLASHEBLKSIZE(x)			FIELD_PREP(GENMASK(25, 18), (x))
#define  FLASHCAPA(x)				FIELD_PREP(GENMASK(17, 16), (x))
#define  FLASHBLERSSIZE_MASK GENMASK(9, 7)
#define  FLASHBLERSSIZE_SHIFT 7
#define  FLASHBLERSSIZE_SIZE_4KB 1
#define  FLASHBLERSSIZE_SIZE_64KB 2
#define  FLASHBLERSSIZE_SIZE_4KB_64KB 3
#define  FLASHBLERSSIZE_SIZE_128KB 4
#define  FLASHBLERSSIZE_SIZE_256KB 5

#define FLASHCTL 0x38
#define  FLASHCTL_SAF_AUTO_READ BIT(18)
#define  FLASHCTL_RSTBUFHEADS BIT(13)
#define  FLASHCTL_FLASH_ACC_TX_AVAIL BIT(1)
#define  FLASHCTL_FLASH_ACC_NP_FREE BIT(0)

#define INTCR4 0xC0
#define  INTCR4_ESPI_SAFSEN			BIT(15)

#define ESPI_FLASH_PRTR_BADDRn(n)		(0x600 + (4*(n)))
#define	 FRGN_WPR				BIT(31)
#define	 FRGN_RPR				BIT(30)
#define	 FLASH_PRTR_BADDR_MASK			GENMASK(26, 12)
#define ESPI_FLASH_PRTR_HADDRn(n)		(0x640 + (4*(n)))
#define  FLASH_PRTR_HADDR_MASK			GENMASK(26, 12)
#define ESPI_FLASH_FLASH_RGN_TAG_OVRn(n)	(0x680 + (4*(n)))
#define	 FRNG_RPR_TOVR				GENMASK(31, 16)
#define	 FRNG_WPR_TOVR				GENMASK(15, 0)

#define ESPI_PLD_LEN_MIN        (1UL << 6)
#define ESPI_PLD_LEN_MAX        (1UL << 12)

// eSPI CYCLE TYPES
#define CYC_MEM_RD_32 				0x00
#define CYC_MEM_RD_64 				0x02
#define CYC_MEM_WR_32				0x01
#define CYC_MEM_WR_64				0x03
#define CYC_MSG					0x10
#define CYC_MSG_WITH_DATA			0x11
#define CYC_SCS_CMP_WITHOUT_DATA		0x06
#define CYC_SCS_CMP_WITH_DATA			0x0F
#define CYC_UNSCS_CMP_WITHOUT_DATA		0x0E
#define CYC_OOB					0x21
#define CYC_FLASH_RD				0x00
#define CYC_FLASH_WR				0x01
#define CYC_FLASH_ERASE				0x02
#define CYC_FLASH_RPMC_OP1			0x03
#define CYC_FLASH_RPMC_OP2			0x04

#define ESPI_FLASH_SAF_TAG_RANGE_NUM	8
#define ESPI_FLASH_SAF_PROT_MEM_NUM	16
#define FLASH_MAX_PAYLOAD_REQ_SIZE	64

typedef enum ESPI_SAF_CMD
{
	ESPI_SAF_CMD_READ = 0,
	ESPI_SAF_CMD_WRITE,
	ESPI_SAF_CMD_ERASE,
	ESPI_SAF_CMD_RPMC_OP1,
	ESPI_SAF_CMD_RPMC_OP2,
	ESPI_SAF_CMD_UNDEF
} ESPI_SAF_CMD;

typedef enum ESPI_FLASH_BLOCK_ERASE_SIZE
{
	ESPI_FLASH_BLOCK_ERASE_4KB = 1,     //(default)
	ESPI_FLASH_BLOCK_ERASE_64KB,
	ESPI_FLASH_BLOCK_ERASE_4KB_AND_64KB,
	ESPI_FLASH_BLOCK_ERASE_128KB,
	ESPI_FLASH_BLOCK_ERASE_256KB,
	ESPI_FLASH_BLOCK_ERASE_NUM,
} ESPI_FLASH_BLOCK_ERASE_SIZE;

typedef enum ESPI_FLASH_ERASE
{
	ESPI_FLASH_ERASE_4K  = 0,
	ESPI_FLASH_ERASE_32K = 1,
	ESPI_FLASH_ERASE_64K = 2,
} ESPI_FLASH_ERASE;

struct npcm_espi_flash
{
	struct device		*dev;
	struct regmap		*espi_regmap;
	struct regmap		*gcr_regmap;
	unsigned short		page_offset;	/* offset in flash address */
	unsigned int		page_size;	/* of bytes per page */
	struct mutex		lock;
	struct mtd_info		mtd;
	uint8_t			erase_mask;
	uint32_t		tx_sts;
	uint32_t		rx_sts;
	wait_queue_head_t	wq;
	spinlock_t		spinlock;
	dma_addr_t		phys_addr;
	resource_size_t		flash_size;
	u8 __iomem		*vaddr;
	int			irq;
	struct miscdevice	miscdev;
};

static bool npcm_espi_check_tag_ovr(struct npcm_espi_flash *priv,
				    uint32_t prtr_baddr,
				    uint8_t command, uint8_t tag, int idx)
{
	uint16_t frgn_wpr_tovr, frgn_rpr_tovr;
	bool frgn_wpr, frgn_rpr;
	uint32_t frgn_tag_tovr;

	if (idx < ESPI_FLASH_SAF_TAG_RANGE_NUM) {
		regmap_read(priv->espi_regmap,
		    	    ESPI_FLASH_FLASH_RGN_TAG_OVRn(idx),
			    &frgn_tag_tovr);
		if (command == ESPI_SAF_CMD_READ) {
			frgn_rpr = FIELD_GET(FRGN_RPR, prtr_baddr);
			if (frgn_rpr) {
				frgn_rpr_tovr = FIELD_GET(FRNG_RPR_TOVR,
							  frgn_tag_tovr);
				if ((frgn_rpr_tovr & BIT(tag)) == 0)
					return false;
			}
		} else if (command == ESPI_SAF_CMD_WRITE ||
			   command == ESPI_SAF_CMD_ERASE) {
			frgn_wpr = FIELD_GET(FRGN_WPR, prtr_baddr);
			if (frgn_wpr) {
				frgn_wpr_tovr = FIELD_GET(FRNG_WPR_TOVR,
							  frgn_tag_tovr);
				if ((frgn_wpr_tovr & BIT(tag)) == 0)
					return false;
			}
		}
	} else {
		if (command == ESPI_SAF_CMD_READ) {
			frgn_rpr = FIELD_GET(FRGN_RPR, prtr_baddr);
			if (frgn_rpr)
				return false;
		} else if (command == ESPI_SAF_CMD_WRITE ||
			   command == ESPI_SAF_CMD_ERASE) {
			frgn_wpr = FIELD_GET(FRGN_WPR, prtr_baddr);
			if (frgn_wpr)
				return false;
		}
	}

	return true;
}

static bool npcm_espi_check_addr(struct npcm_espi_flash *priv, uint8_t command,
				 uint8_t tag, uint32_t addr)
{
	uint32_t prtr_haddr, high_addr, prtr_baddr, base_addr;
	bool ret = true;
	int i;

	for (i = 0; i < ESPI_FLASH_SAF_PROT_MEM_NUM; i++) {
		regmap_read(priv->espi_regmap,
			    ESPI_FLASH_PRTR_HADDRn(i), &prtr_haddr);
		high_addr = FIELD_GET(FLASH_PRTR_HADDR_MASK, prtr_haddr);
		if (high_addr != 0) {
			regmap_read(priv->espi_regmap,
				    ESPI_FLASH_PRTR_BADDRn(i), &prtr_baddr);
			base_addr = FIELD_GET(FLASH_PRTR_BADDR_MASK,
					      prtr_baddr);
			if ((addr >= base_addr) && (addr <= high_addr)) {
				ret = npcm_espi_check_tag_ovr(priv, prtr_baddr,
							      command, tag, i);
				if (!ret)
					break;
			}
		}
	}

	return ret;
}

static long npcm_espi_flash_read(struct npcm_espi_flash *priv, int tag,
				 loff_t from, size_t len, bool tovr)
{
	uint32_t *buffer = (uint32_t *)(priv->vaddr + from);
	int cyc = CYC_SCS_CMP_WITH_DATA;
	int ret = 0, tx_cnt, i;
	uint32_t reg;

	/* Test if flash channel is ready */
	regmap_read(priv->espi_regmap, FLASHCTL, &reg);
	if ((reg & FLASHCTL_FLASH_ACC_TX_AVAIL)) {
		dev_err(priv->dev, "eSPI flash tx is not ready\n");
		return -EIO;
	}

	if (len > FLASH_MAX_PAYLOAD_REQ_SIZE || !tovr)
		cyc = CYC_UNSCS_CMP_WITHOUT_DATA;

	reg = (reg | 3) + len;
	reg = reg | (cyc << 8);
	reg = reg | ((len & 0x0F00) << 16) | (u32)(tag << 20);
	reg = reg | ((len & 0x00FF) << 24); 

	regmap_write(priv->espi_regmap, FLASHTXWRHEAD, reg);

	if (cyc == CYC_SCS_CMP_WITH_DATA) {
		tx_cnt =  len /4;
		if (len % 4)
			tx_cnt++;

		for (i = 0; i < tx_cnt; i++) {
			reg = buffer[i];
			regmap_write(priv->espi_regmap, FLASHTXWRHEAD, reg);
		}
	}

	regmap_update_bits(priv->espi_regmap, FLASHCTL,
			   FLASHCTL_FLASH_ACC_TX_AVAIL,
			   FLASHCTL_FLASH_ACC_TX_AVAIL);

	return ret;
}

static long npcm_espi_flash_write(struct npcm_espi_flash *priv, int tag,
				  loff_t from, size_t len, bool tovr)
{
	uint32_t *buffer = (uint32_t *)(priv->vaddr + from);
	int cyc = CYC_SCS_CMP_WITHOUT_DATA;
	int ret = 0, tx_cnt, i;
	size_t last_len;
	uint32_t reg;

	/* Test if flash channel is ready */
	regmap_read(priv->espi_regmap, FLASHCTL, &reg);
	if ((reg & FLASHCTL_FLASH_ACC_TX_AVAIL)) {
		dev_err(priv->dev, "eSPI flash tx is not ready\n");
		return -EIO;
	}

	if (len > FLASH_MAX_PAYLOAD_REQ_SIZE || !tovr)
		cyc = CYC_UNSCS_CMP_WITHOUT_DATA;

	if (cyc == CYC_SCS_CMP_WITHOUT_DATA) {
		tx_cnt =  len /4;
		last_len = len % 4;

		for (i = 0; i < tx_cnt; i++) {
			regmap_read(priv->espi_regmap, FLASHRXRDHEAD, &reg);
			buffer[i] = reg;
		}

		/* be aware of less than 4 bytes */
		if (last_len) {
			regmap_read(priv->espi_regmap, FLASHRXRDHEAD, &reg);
			memcpy((char *)priv->vaddr + from + len - last_len,
				&reg, last_len);
		}
	}

	len = 0;	
	reg =  3 + len;
	reg = reg | (cyc << 8);
	reg = reg | ((len & 0x0F00) << 16) | (u32)(tag << 20);
	reg = reg | ((len & 0x00FF) << 24); 

	regmap_write(priv->espi_regmap, FLASHTXWRHEAD, reg);

	regmap_update_bits(priv->espi_regmap, FLASHCTL,
			   FLASHCTL_FLASH_ACC_TX_AVAIL,
			   FLASHCTL_FLASH_ACC_TX_AVAIL);

	return ret;
}

static long npcm_espi_flash_erase(struct npcm_espi_flash *priv, int tag,
				  loff_t from, size_t len, bool tovr)
{
	uint32_t *buffer = (uint32_t *)(priv->vaddr + from);
	int cyc = CYC_SCS_CMP_WITHOUT_DATA;
	int erase_size = 0;
	int ret = 0, i;
	uint32_t reg;

	/* Test if flash channel is ready */
	regmap_read(priv->espi_regmap, FLASHCTL, &reg);
	if ((reg & FLASHCTL_FLASH_ACC_TX_AVAIL)) {
		dev_err(priv->dev, "eSPI flash tx is not ready\n");
		return -EIO;
	}

	regmap_read(priv->espi_regmap, FLASHCFG, &reg);
	reg = (reg & FLASHBLERSSIZE_MASK) >> FLASHBLERSSIZE_SHIFT;

	switch (len) {
		case ESPI_FLASH_ERASE_4K:
			if (reg == ESPI_FLASH_BLOCK_ERASE_4KB ||
			    reg == ESPI_FLASH_BLOCK_ERASE_4KB_AND_64KB)
				erase_size = 0x1000;
			else
				cyc = CYC_UNSCS_CMP_WITHOUT_DATA;
			break;
		case ESPI_FLASH_ERASE_64K:
			if (reg == ESPI_FLASH_BLOCK_ERASE_64KB ||
			    reg == ESPI_FLASH_BLOCK_ERASE_4KB_AND_64KB)
				erase_size = 0x10000;
			else
				cyc = CYC_UNSCS_CMP_WITHOUT_DATA;
			break;
		case ESPI_FLASH_ERASE_32K:
		default:
			cyc = CYC_UNSCS_CMP_WITHOUT_DATA;
			break;
	}

	if (tovr) {
		if (erase_size)
			for (i = 0; i < (erase_size >> 2); i++)
				buffer[i] = 0xffffffff;
	} else
		cyc = CYC_UNSCS_CMP_WITHOUT_DATA;

	len = 0;	
	reg =  3 + len;
	reg = reg | (cyc << 8);
	reg = reg | ((len & 0x0F00) << 16) | (u32)(tag << 20);
	reg = reg | ((len & 0x00FF) << 24); 

	regmap_write(priv->espi_regmap, FLASHTXWRHEAD, reg);

	regmap_update_bits(priv->espi_regmap, FLASHCTL,
			   FLASHCTL_FLASH_ACC_TX_AVAIL,
			   FLASHCTL_FLASH_ACC_TX_AVAIL);

	return ret;
}

static irqreturn_t npcm_espi_flash_irq_handler(int irq, void *arg)
{
	struct npcm_espi_flash *priv = arg;
	uint32_t val, address, reg;
	uint8_t command, tag;
	uint16_t length;
	bool tovr;

	regmap_read(priv->espi_regmap, ESPISTS, &val);

	if (val & ESPISTS_CFGUPD) {
		regmap_write(priv->espi_regmap, ESPISTS, ESPISTS_CFGUPD);
		regmap_read(priv->espi_regmap, ESPICFG, &reg);
		if (reg & ESPICFG_HFLASHCHANEN) {
			regmap_update_bits(priv->espi_regmap, FLASHCTL,
					   FLASHCTL_SAF_AUTO_READ, 0);
			regmap_update_bits(priv->espi_regmap, FLASHCFG,
					   FLASHCAPA(3) | TRGFLASHEBLKSIZE(68),
					   FLASHCAPA(3) | TRGFLASHEBLKSIZE(68));
			regmap_read(priv->espi_regmap, FLASHCTL, &reg);
			regmap_update_bits(priv->espi_regmap, ESPICFG,
					   ESPICFG_FLASHCHANEN,
					   ESPICFG_FLASHCHANEN);
		}	
	}

	if (val & ESPISTS_SFLASHRD) {
		regmap_write(priv->espi_regmap, ESPISTS, ESPISTS_SFLASHRD);
		regmap_update_bits(priv->espi_regmap, FLASHCTL,
				   FLASHCTL_RSTBUFHEADS, FLASHCTL_RSTBUFHEADS);

		/*
		 * In case the FLASHRX and SFLASHRD irq arrive at same time,
		 * set FLASHCTL_FLASH_ACC_NP_FREE bit after the SFLASHRD occurred.
		 */
		regmap_update_bits(priv->espi_regmap, FLASHCTL,
				   FLASHCTL_FLASH_ACC_NP_FREE,
				   FLASHCTL_FLASH_ACC_NP_FREE);
	}

	if (val & ESPISTS_FLASHRX) {
		regmap_write(priv->espi_regmap, ESPISTS, ESPISTS_FLASHRX);

		regmap_read(priv->espi_regmap, FLASHRXRDHEAD, &val);

		// Handle type. tag and length
		command = (uint8_t)((val & 0x0000FF00) >> 8);
		tag = (uint8_t)((val & 0x00F00000) >> 20);
		length = (uint16_t)(((val & 0xFF000000) >> 24) |
			 ((val & 0x000F0000) >> 8));

		regmap_read(priv->espi_regmap, FLASHRXRDHEAD, &val);

		// Handle address
		address = ((val & 0xFF000000) >> 24) |
			  ((val & 0x00FF0000) >> 8) |
			  ((val & 0x0000FF00) << 8) |
			  ((val & 0x000000FF) << 24);

		tovr = npcm_espi_check_addr(priv, command, tag, address);

		trace_npcm_flash(command, address, length);

		if (command == ESPI_SAF_CMD_READ)
			npcm_espi_flash_read(priv, tag, address, length, tovr);
		else if (command == ESPI_SAF_CMD_WRITE)
			npcm_espi_flash_write(priv, tag, address, length, tovr);
		else if (command == ESPI_SAF_CMD_ERASE)
			npcm_espi_flash_erase(priv, tag, address, length, tovr);
	}

	return IRQ_HANDLED;
}

static int npcm_espi_flash_mtd_read(struct mtd_info *mtd,
				    loff_t from, size_t len,
				    size_t *retlen, u_char *buf)
{
	struct npcm_espi_flash *priv = mtd->priv;
	int ret = 0;

	mutex_lock(&priv->lock);

	memcpy(buf, priv->vaddr + from, len);
	*retlen = len;

	mutex_unlock(&priv->lock);
	return ret;
}

static int npcm_espi_flash_mtd_erase(struct mtd_info *mtd,
				     struct erase_info *instr)
{
	struct npcm_espi_flash *priv = mtd->priv;
	int ret = 0;

	mutex_lock(&priv->lock);

	memset((char *)priv->vaddr + instr->addr, 0xff, instr->len);

	mutex_unlock(&priv->lock);
	return ret;
}

static int npcm_espi_flash_mtd_write(struct mtd_info *mtd,
				     loff_t to, size_t len,
				     size_t *retlen, const u_char *buf)
{
	struct npcm_espi_flash *priv = mtd->priv;
	int ret = 0;

	mutex_lock(&priv->lock);

	memcpy((char *)priv->vaddr + to, buf, len);
	*retlen = len;

	mutex_unlock(&priv->lock);
	return ret;
}

static void npcm_espi_flash_enable(struct npcm_espi_flash *priv)
{
	regmap_update_bits(priv->gcr_regmap, INTCR4,
			   INTCR4_ESPI_SAFSEN, INTCR4_ESPI_SAFSEN);
	regmap_update_bits(priv->espi_regmap, ESPIIE, ESPIIE_FLASHRXIE |
			   ESPIIE_SFLASHRDIE | ESPIIE_CFGUPDIE,
			   ESPIIE_FLASHRXIE | ESPIIE_SFLASHRDIE |
			   ESPIIE_CFGUPDIE);
	regmap_update_bits(priv->espi_regmap, FLASHCTL,
			   FLASHCTL_SAF_AUTO_READ, 0);
	regmap_update_bits(priv->espi_regmap, FLASHCFG,
			   FLASHCAPA(3) | TRGFLASHEBLKSIZE(68),
			   FLASHCAPA(3) | TRGFLASHEBLKSIZE(68));
}

static int npcm_espi_flash_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node;
	struct resource resm;
	struct mtd_info *mtd;
	struct npcm_espi_flash *priv;
	struct device *dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(struct npcm_espi_flash),
			GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->espi_regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(priv->espi_regmap)) {
		dev_err(dev, "Couldn't get espiregmap\n");
		return -ENODEV;
	}

	priv->gcr_regmap = syscon_regmap_lookup_by_phandle(dev->of_node,
			"nuvoton,sysgcr");
	if (IS_ERR(priv->gcr_regmap)) {
		dev_err(dev, "Couldn't get gcr regmap\n");
		return -ENODEV;
	}

	/* If memory-region is described in device tree then store */
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (node) {
		ret = of_address_to_resource(node, 0, &resm);
		of_node_put(node);
		if (!ret) {
			priv->flash_size = resource_size(&resm);
			priv->phys_addr = resm.start;
			priv->vaddr = devm_ioremap_resource_wc(dev, &resm);
			if (IS_ERR(priv->vaddr)) {
				dev_err(dev, "device mem io remap failed\n");
				return PTR_ERR(priv->vaddr);
			}
		} else {
			dev_err(dev, "No memory region\n");
			return -EINVAL;
		}
	} else {
		dev_err(dev,
			"No DTS config, assign default flash mem Address\n");
		return -EINVAL;
	}

	mtd = &priv->mtd;
	mtd->dev.parent = dev;
	mtd->size = priv->flash_size;
	mtd->flags = MTD_CAP_RAM;
	mtd->_erase = npcm_espi_flash_mtd_erase;
	mtd->_read = npcm_espi_flash_mtd_read;
	mtd->_write = npcm_espi_flash_mtd_write;
	mtd->type = MTD_RAM;
	mtd->name = DEVICE_NAME;
	mtd->erasesize = 4*1024;
	mtd->writesize = 1;
	mtd->owner = THIS_MODULE;
	mtd->priv = priv;

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_notice(dev,
			   "npcm-espi-flash: Failed to register mtd device\n");
		return ret;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	ret = devm_request_irq(dev, priv->irq, npcm_espi_flash_irq_handler,
			       IRQF_SHARED, "espi-flash", priv);
	if (ret) {
		dev_err(dev, "failed to request IRQ\n");
		return ret;
	}

	priv->dev = &pdev->dev;
	npcm_espi_flash_enable(priv);

	/* Bus lock */
	mutex_init(&priv->lock);
	init_waitqueue_head(&priv->wq);
	spin_lock_init(&priv->spinlock);

	dev_set_drvdata(dev, priv);

	return 0;
}

static int npcm_espi_flash_remove(struct platform_device *pdev)
{
	struct npcm_espi_flash *priv = platform_get_drvdata(pdev);
	mtd_device_unregister(&priv->mtd);
	kfree(priv->miscdev.name);
	mutex_destroy(&priv->lock);
	kfree(priv);
	return 0;
}

static const struct of_device_id npcm_espi_flash_match[] = {
	{ .compatible = "nuvoton,npcm845-espi-flash" },
	{}
};
MODULE_DEVICE_TABLE(of, npcm_espi_flash_match);

static struct platform_driver npcm_espi_flash_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = npcm_espi_flash_match,
	},
	.probe  = npcm_espi_flash_probe,
	.remove = npcm_espi_flash_remove,
};
module_platform_driver(npcm_espi_flash_driver);

MODULE_AUTHOR("Joseph Liu <kwliu@nuvoton.com>");
MODULE_AUTHOR("Ban Feng <kcfeng0@nuvoton.com>");
MODULE_DESCRIPTION("NPCM eSPI Flash driver");
MODULE_LICENSE("GPL v2");
