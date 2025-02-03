// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for GDMA controller present on Nuvoton NPCM SoCs.
 *
 * Copyright (C) 2025 Nuvoton Technologies
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/dma/npcm_gdma.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include "../dmaengine.h"

#define DEVICE_NAME		"npcm-gdma"
#define GDMA_CHAN_NUM		2

/* GDMA registers */
#define GDMA_CTL(c)		(c * 0x20 + 0x0000)
#define  GDMA_CTL_GDMAEN	BIT(0)
#define  GDMA_CTL_GDMAMS	GENMASK(3, 2)
#define  GDMA_CTL_DAFIX		BIT(6)
#define  GDMA_CTL_SAFIX		BIT(7)
#define  GDMA_CTL_TWS		GENMASK(13, 12)

#define GDMA_SRCB(c)		(c * 0x20 + 0x0004)
#define GDMA_DSTB(c)		(c * 0x20 + 0x0008)
#define GDMA_TCNT(c)		(c * 0x20 + 0x000c)
#define GDMA_CTCNT(c)		(c * 0x20 + 0x0018)

#define GDMA_CH_CONN_SEL(c, x)	(x << (c * 16))
#define GDMA_CH_REQ_EN(c)	BIT(5 + c * 16)
#define GDMA_CH_ACK_EN(c)	BIT(6 + c * 16)

struct npcm_gdma_desc {
	struct dma_async_tx_descriptor tx;
	dma_addr_t addr;
	unsigned int len;
};

struct npcm_gdma_chan {
	struct dma_chan chan;
	struct dma_slave_config cfg;
	struct npcm_gdma *gdma;
	struct npcm_gdma_desc gdma_desc;
	enum dma_transfer_direction dir;
	unsigned int residue;
	unsigned int connectivity;
};

#define to_npcm_gdma_chan(x) \
	container_of((x), struct npcm_gdma_chan, chan)

struct npcm_gdma {
	struct dma_device ddev;
	struct npcm_gdma_chan *gdma_chan;

	void __iomem *gdmac_reg;
	void __iomem *reqc_reg;

	int irq;
	spinlock_t lock;
};

static int npcm_gdma_alloc_chan_resources(struct dma_chan *chan)
{
	dma_cookie_init(chan);

	return 0;
}

static void npcm_gdma_free_chan_resources(struct dma_chan *chan)
{
	struct npcm_gdma_chan *gdma_chan = to_npcm_gdma_chan(chan);
	struct npcm_gdma *gdma = gdma_chan->gdma;
	unsigned long flags;

	spin_lock_irqsave(&gdma->lock, flags);
	writel(0, gdma->reqc_reg);
	spin_unlock_irqrestore(&gdma->lock, flags);
}

static dma_cookie_t npcm_gdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	return dma_cookie_assign(tx);
}

static struct dma_async_tx_descriptor *npcm_gdma_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction dir, unsigned long flags, void *context)
{
	struct npcm_gdma_chan *gdma_chan = to_npcm_gdma_chan(chan);
	struct npcm_gdma_desc *gdma_desc = &gdma_chan->gdma_desc;

	if (!is_slave_direction(dir) || sg_len != 1)
		return NULL;

	gdma_desc->addr = sg_dma_address(sgl);
	gdma_desc->len = sg_dma_len(sgl);
	gdma_chan->dir = dir;

	dma_async_tx_descriptor_init(&gdma_desc->tx, &gdma_chan->chan);
	gdma_desc->tx.tx_submit = npcm_gdma_tx_submit;

	return &gdma_desc->tx;
}

static int npcm_gdma_device_config(struct dma_chan *chan,
				   struct dma_slave_config *config)
{
	struct npcm_gdma_chan *gdma_chan = to_npcm_gdma_chan(chan);
	struct npcm_gdma_peripheral_config *pcfg = config->peripheral_config;

	memcpy(&gdma_chan->cfg, config, sizeof(*config));
	if (config->peripheral_size == sizeof(*pcfg))
		gdma_chan->connectivity = pcfg->connectivity;

	return 0;
}

static int npcm_gdma_terminate_all(struct dma_chan *chan)
{
	struct npcm_gdma_chan *gdma_chan = to_npcm_gdma_chan(chan);
	struct npcm_gdma *gdma = gdma_chan->gdma;
	unsigned long flags;

	spin_lock_irqsave(&gdma->lock, flags);
	writel(0, gdma->gdmac_reg + GDMA_CTL(chan->chan_id));
	spin_unlock_irqrestore(&gdma->lock, flags);

	return 0;
}

static enum dma_status npcm_gdma_tx_status(struct dma_chan *chan,
					   dma_cookie_t cookie,
					   struct dma_tx_state *txstate)
{
	struct npcm_gdma_chan *gdma_chan = to_npcm_gdma_chan(chan);
	struct npcm_gdma *gdma = gdma_chan->gdma;
	enum dma_status status = dma_cookie_status(chan, cookie, txstate);

	dma_set_residue(txstate, readl(gdma->gdmac_reg + GDMA_CTCNT(chan->chan_id)));
	return status;
}

static void npcm_gdma_issue_pending(struct dma_chan *chan)
{
	struct npcm_gdma_chan *gdma_chan = to_npcm_gdma_chan(chan);
	struct npcm_gdma_desc *gdma_desc = &gdma_chan->gdma_desc;
	struct npcm_gdma *gdma = gdma_chan->gdma;
	unsigned int chan_id = chan->chan_id, val, ctl;
	unsigned long flags;

	spin_lock_irqsave(&gdma->lock, flags);

	/* Set REQ_EN/ACK_EN and channel connectivity */
	val = GDMA_CH_REQ_EN(chan_id) | GDMA_CH_ACK_EN(chan_id) |
	      GDMA_CH_CONN_SEL(chan_id, gdma_chan->connectivity);

	writel(val, gdma->reqc_reg);

	/* Set source/destination base address */
	if (gdma_chan->dir == DMA_MEM_TO_DEV) {
		writel(gdma_desc->addr, gdma->gdmac_reg + GDMA_SRCB(chan_id));
		writel(gdma_chan->cfg.dst_addr, gdma->gdmac_reg + GDMA_DSTB(chan_id));
		ctl = GDMA_CTL_DAFIX | FIELD_PREP(GDMA_CTL_TWS, 2);
	} else if (gdma_chan->dir == DMA_DEV_TO_MEM) {
		writel(gdma_chan->cfg.src_addr, gdma->gdmac_reg + GDMA_SRCB(chan_id));
		writel(gdma_desc->addr, gdma->gdmac_reg + GDMA_DSTB(chan_id));
		ctl = GDMA_CTL_SAFIX;
	}

	/* Set transfer count */
	writel(gdma_desc->len, gdma->gdmac_reg + GDMA_TCNT(chan_id));

	/* Start DMA */
	ctl |= FIELD_PREP(GDMA_CTL_GDMAMS, chan_id + 1) | GDMA_CTL_GDMAEN;
	writel(ctl, gdma->gdmac_reg + GDMA_CTL(chan_id));

	spin_unlock_irqrestore(&gdma->lock, flags);
}

static irqreturn_t npcm_gdma_isr(int irq, void *arg)
{
	struct npcm_gdma *gdma = arg;
	struct npcm_gdma_chan *gdma_chan = gdma->gdma_chan;
	struct dma_async_tx_descriptor *tx = &gdma_chan->gdma_desc.tx;
	struct dmaengine_result result;

	/* GDMA operation is completed */
	result.result = DMA_TRANS_NOERROR;
	result.residue = readl(gdma->gdmac_reg + GDMA_CTCNT(gdma_chan->chan.chan_id));
	dma_cookie_complete(tx);
	dmaengine_desc_get_callback_invoke(tx, &result);

	return IRQ_HANDLED;
}

static int npcm_gdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct npcm_gdma *gdma;
	int rc, i;

	gdma = devm_kzalloc(dev, sizeof(*gdma), GFP_KERNEL);
	if (!gdma)
		return -ENOMEM;

	INIT_LIST_HEAD(&gdma->ddev.channels);
	dma_cap_set(DMA_SLAVE, gdma->ddev.cap_mask);
	gdma->ddev.dev = dev;
	gdma->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	gdma->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	gdma->ddev.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	gdma->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	gdma->ddev.device_alloc_chan_resources = npcm_gdma_alloc_chan_resources;
	gdma->ddev.device_free_chan_resources = npcm_gdma_free_chan_resources;
	gdma->ddev.device_prep_slave_sg = npcm_gdma_prep_slave_sg;
	gdma->ddev.device_config = npcm_gdma_device_config;
	gdma->ddev.device_terminate_all = npcm_gdma_terminate_all;
	gdma->ddev.device_tx_status = npcm_gdma_tx_status;
	gdma->ddev.device_issue_pending = npcm_gdma_issue_pending;

	gdma->gdmac_reg = devm_platform_ioremap_resource_byname(pdev, "gdmac");
	if (IS_ERR(gdma->gdmac_reg))
		return PTR_ERR(gdma->gdmac_reg);

	gdma->reqc_reg = devm_platform_ioremap_resource_byname(pdev, "reqc");
	if (IS_ERR(gdma->reqc_reg))
		return PTR_ERR(gdma->reqc_reg);

	gdma->irq = platform_get_irq(pdev, 0);
	if (gdma->irq < 0)
		return gdma->irq;

	rc = devm_request_irq(dev, gdma->irq, npcm_gdma_isr, IRQF_NO_SUSPEND,
			      DEVICE_NAME, gdma);
	if (rc) {
		dev_err(dev, "Failed to request IRQ\n");
		return rc;
	}

	gdma->gdma_chan = devm_kzalloc(dev, sizeof(struct npcm_gdma_chan) *
				       GDMA_CHAN_NUM, GFP_KERNEL);
	if (!gdma->gdma_chan)
		return -ENOMEM;

	for (i = 0; i < GDMA_CHAN_NUM; i++) {
		gdma->gdma_chan[i].chan.device = &gdma->ddev;
		gdma->gdma_chan[i].gdma = gdma;
		list_add_tail(&gdma->gdma_chan[i].chan.device_node,
			      &gdma->ddev.channels);
	}

	rc = dma_async_device_register(&gdma->ddev);
	if (rc)
		return rc;

	rc = of_dma_controller_register(dev->of_node, of_dma_xlate_by_chan_id,
					&gdma->ddev);
	if (rc)
		return rc;

	spin_lock_init(&gdma->lock);
	platform_set_drvdata(pdev, gdma);

	dev_info(&pdev->dev, "module loaded\n");
	return 0;
}

static int npcm_gdma_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct npcm_gdma *gdma = platform_get_drvdata(pdev);

	of_dma_controller_free(dev->of_node);
	dma_async_device_unregister(&gdma->ddev);

	dev_info(&pdev->dev, "module removed\n");
	return 0;
}

static const struct of_device_id npcm_gdma_match[] = {
	{ .compatible = "nuvoton,npcm-gdma" },
	{ },
};

MODULE_DEVICE_TABLE(of, npcm_gdma_match);

static struct platform_driver npcm_gdma_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = npcm_gdma_match,
	},
	.probe = npcm_gdma_probe,
	.remove = npcm_gdma_remove,
};

module_platform_driver(npcm_gdma_driver);

MODULE_AUTHOR("Marvin Lin <kflin@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton NPCM GDMA controller driver");
MODULE_LICENSE("GPL");
