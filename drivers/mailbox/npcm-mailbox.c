// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, Microsoft Corporation
// Copyright (c) 2022 Nuvoton Technology corporation.

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define NPCM_CP2BST_REG_OFFSET 0x04
#define NPCM_B2CPNT_REG_OFFSET 0x08
#define NPCM_NUM_DOORBELLS 32

struct npcm_mbox_hw {
	struct device		*dev;
	void __iomem		*base;
	struct mbox_controller	*controller;
};

/**
 * NPCM mailbox allocated channel information
 *
 * @mhw: Pointer to parent mailbox device
 * @doorbell: doorbell number pertaining to this channel
 */
struct npcm_mbox_channel {
	struct npcm_mbox_hw	*mhw;
	unsigned int		pchan;
	unsigned int		doorbell;
};

static inline struct mbox_chan *
npcm_mbox_to_channel(struct mbox_controller *controller, unsigned int doorbell)
{
	struct npcm_mbox_channel *chan_info = controller->chans[doorbell].con_priv;
	if (chan_info && chan_info->doorbell == doorbell)
		return &controller->chans[doorbell];

	return NULL;
}

static void npcm_mbox_clear_irq(struct mbox_chan *chan)
{
	struct npcm_mbox_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhw->base;

	writel_relaxed(BIT(chan_info->doorbell), base + NPCM_CP2BST_REG_OFFSET);
}

static struct mbox_chan *npcm_mbox_irq_to_channel(struct npcm_mbox_hw *mhw)
{
	unsigned long bits;
	unsigned int doorbell;
	struct mbox_chan *chan = NULL;
	struct mbox_controller *controller = mhw->controller;
	void __iomem *base = mhw->base;

	bits = readl_relaxed(base + NPCM_CP2BST_REG_OFFSET);

	/* No IRQs fired in specified physical channel */
	if (!bits)
		return NULL;

	/* An IRQ has fired, find the associated channel */
	for (doorbell = 0; bits; doorbell++) {
		if (!test_and_clear_bit(doorbell, &bits))
			continue;

		chan = npcm_mbox_to_channel(controller, doorbell);
		if (chan)
			break;
		dev_err(controller->dev,
			"Channel 0 not registered yet, doorbell: %d\n",
			doorbell);
	}
	return chan;
}

static irqreturn_t npcm_mbox_rx_handler(int irq, void *data)
{
	struct npcm_mbox_hw *mhw = data;
	struct mbox_chan *chan = NULL;

	while ((chan = npcm_mbox_irq_to_channel(mhw)) != NULL) {
		mbox_chan_received_data(chan, NULL);
		npcm_mbox_clear_irq(chan);
	}
	return IRQ_HANDLED;
}

static int npcm_send_data(struct mbox_chan *chan, void *data)
{
	struct npcm_mbox_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhw->base;

	/* Send event to co-processor */
	writel_relaxed(BIT(chan_info->doorbell),
		       base + NPCM_B2CPNT_REG_OFFSET);

	return 0;
}

static int npcm_startup(struct mbox_chan *chan)
{
	npcm_mbox_clear_irq(chan);
	return 0;
}

static void npcm_shutdown(struct mbox_chan *chan)
{
	struct npcm_mbox_channel *chan_info = chan->con_priv;
	struct mbox_controller *controller = chan_info->mhw->controller;

	/* Reset channel */
	npcm_mbox_clear_irq(chan);
	devm_kfree(controller->dev, chan->con_priv);
	chan->con_priv = NULL;
}

static bool npcm_last_tx_done(struct mbox_chan *chan)
{
	struct npcm_mbox_channel *chan_info = chan->con_priv;
	void __iomem *base = chan_info->mhw->base;

	if (readl_relaxed(base + NPCM_B2CPNT_REG_OFFSET) &
	    BIT(chan_info->doorbell))
		return false;

	return true;
}

static struct mbox_chan *npcm_mbox_xlate(struct mbox_controller *controller,
					 const struct of_phandle_args *spec)
{
	struct npcm_mbox_hw *mhw = dev_get_drvdata(controller->dev);
	struct npcm_mbox_channel *chan_info;
	struct mbox_chan *chan = NULL;
	unsigned int pchan, doorbell;

	if (spec->args_count != 2) {
		dev_err(controller->dev, "Invalid argumment count: %d\n",
			spec->args_count);
		return ERR_PTR(-EINVAL);
	}

	pchan = spec->args[0];
	doorbell = spec->args[1];
	/* Bounds checking */
	if (pchan != 0 || doorbell >= NPCM_NUM_DOORBELLS) {
		dev_err(controller->dev,
			"Invalid channel requested pchan: %d doorbell: %d\n",
			pchan, doorbell);
		return ERR_PTR(-EINVAL);
	}

	/* Check if the requested channel free */
	chan = npcm_mbox_to_channel(controller, doorbell);
	if (chan) {
		dev_err(controller->dev,
			"Channel in use: pchan: %d doorbell: %d\n", pchan,
			doorbell);
		return ERR_PTR(-EBUSY);
	}

	chan = &controller->chans[doorbell];
	chan_info =
		devm_kzalloc(controller->dev, sizeof(*chan_info), GFP_KERNEL);
	if (!chan_info)
		return ERR_PTR(-ENOMEM);

	chan_info->mhw = mhw;
	chan_info->doorbell = doorbell;
	chan_info->pchan = pchan;
	chan->con_priv = chan_info;

	dev_info(controller->dev,
		 "controller: created channel 0, doorbell: %d\n", doorbell);

	return chan;
}

static const struct mbox_chan_ops npcm_ops = {
	.send_data = npcm_send_data,
	.startup = npcm_startup,
	.shutdown = npcm_shutdown,
	.last_tx_done = npcm_last_tx_done,
};

static int npcm_mbox_probe(struct platform_device *pdev)
{
	struct mbox_controller *mbox;
	struct npcm_mbox_hw *mhw;
	struct mbox_chan *chans;
	struct resource *res;
	int err = 0;

	mhw = devm_kzalloc(&pdev->dev, sizeof(*mhw), GFP_KERNEL);
	if (!mhw)
		return -ENOMEM;

	platform_set_drvdata(pdev, mhw);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mhw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mhw->base))
		return PTR_ERR(mhw->base);

	mbox = devm_kcalloc(&pdev->dev, 1, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	chans = devm_kcalloc(&pdev->dev,
			     NPCM_NUM_DOORBELLS, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	mhw->dev	= &pdev->dev; 
	mhw->controller	= mbox;

	mbox->dev = mhw->dev;
	mbox->chans = chans;
	mbox->num_chans = NPCM_NUM_DOORBELLS;
	mbox->txdone_irq = false;
	mbox->txdone_poll = true;
	mbox->txpoll_period = 1;
	mbox->of_xlate = npcm_mbox_xlate;
	mbox->ops = &npcm_ops;

	err = devm_mbox_controller_register(&pdev->dev, mbox);
	if (err) {
		dev_err(&pdev->dev, "Failed to register mailboxes %d\n", err);
		return err;
	}

	/* Clear all IRQs before claiming IRQ handler */
	writel_relaxed(0xFFFFFFFF, mhw->base + NPCM_CP2BST_REG_OFFSET);

	err = devm_request_threaded_irq(&pdev->dev, platform_get_irq(pdev, 0),
					NULL, npcm_mbox_rx_handler,
					IRQF_ONESHOT, "npcm_mbox", mhw);
	if (err) {
		dev_err(&pdev->dev, "Can't claim IRQ handler %d\n", err);
		mbox_controller_unregister(mbox);
		return err;
	}

	dev_info(&pdev->dev, "Mailbox registered\n");
	return 0;
}

static const struct of_device_id npcm_mbox_match[] = {
	{ .compatible = "nuvoton,npcm750-mbox" },
	{ .compatible = "nuvoton,npcm845-mbox" },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm_mbox_match);

static struct platform_driver npcm_mbox_driver = {
	.probe	= npcm_mbox_probe,
	.driver	= {
		.name = "npcm-mailbox",
		.of_match_table	= npcm_mbox_match,
	},
};
module_platform_driver(npcm_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NPCM Mailbox controller Driver");
MODULE_AUTHOR("Xiling Sun <xiling.sun@microsoft.com>");
