// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Nuvoton Technology corporation.

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
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/mailbox_client.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/hw_random.h>
#include <linux/pm_runtime.h>

#define NPCM_RNG_TIP_CMD	0x0
#define NPCM_RNG_TIP_SIZE	0x4
#define NPCM_RNG_TIP_DST	0x8

#define	NPCM_RNG_TIP_READ_CMD	6

#define RNG_TIP_READ_TIMEOUT	500

#define MIN_RNG_BUFFER		0x1000

struct npcm_rng_tip {
	struct device			*dev;
	struct mbox_client		cl;
	struct mbox_chan		*chan;
	void __iomem			*reg;
	bool				event_received;
	struct completion 		wait_event;
	resource_size_t 		phy_start;
	resource_size_t 		phy_size;
	u8 __iomem			*virt_off;
	struct hwrng			rng;
};

#define to_npcm_rng_tip(p)	container_of(p, struct npcm_rng_tip, rng)

static int npcm_rng_tip_init(struct hwrng *rng)
{
	return 0;
}

static void npcm_rng_tip_cleanup(struct hwrng *rng)
{
	return;
}

static int npcm_rng_tip_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct npcm_rng_tip *priv = to_npcm_rng_tip(rng);
	int err;

	if (!wait)
		return -EIO;

	writel(NPCM_RNG_TIP_READ_CMD, priv->reg + NPCM_RNG_TIP_CMD);
	writel(max, priv->reg + NPCM_RNG_TIP_SIZE);
	writel((u32)priv->phy_start, priv->reg + NPCM_RNG_TIP_DST);

	err = mbox_send_message(priv->chan, priv->reg);
	if (err >= 0) {
		if (!wait_for_completion_timeout(&priv->wait_event,
						 msecs_to_jiffies(RNG_TIP_READ_TIMEOUT))) {
			dev_err(priv->dev, "HW random read timeout\n");
			return -ETIMEDOUT;
		}
	} else {
		dev_err(priv->dev, "mbox_send_message returned %d\n", err);
		return err;
	}

	memcpy(buf, priv->virt_off, max);

	if (readl(priv->reg + NPCM_RNG_TIP_CMD))
		return -EIO;
	
	return max;
}

static void msg_from_tip_rng(struct mbox_client *cl, void *msg)
{
	struct npcm_rng_tip *priv = container_of(cl, struct npcm_rng_tip, cl);

	complete(&priv->wait_event);
}

static int npcm_rng_tip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct npcm_rng_tip *priv;
	struct device_node *tipsh;
	struct resource t_res;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->reg))
		return PTR_ERR(priv->reg);

	dev_set_drvdata(dev, priv);

	init_completion(&priv->wait_event);
	/* set up and request mailbox channel */
	priv->cl.dev = dev;
	priv->cl.rx_callback = msg_from_tip_rng; /* read non-blocking */
	priv->cl.tx_done = NULL;
	priv->cl.tx_block = true; /* write blocking */
	priv->cl.tx_tout = 500; /* msec */
	priv->chan = mbox_request_channel_byname(&priv->cl, "rng_tip");
	if (IS_ERR(priv->chan)) {
		dev_err(dev, "mbox channel request failed\n");
		return PTR_ERR(priv->chan);
	}

	tipsh = of_parse_phandle(np, "memory-region", 0);
	if (tipsh) {
		ret = of_address_to_resource(tipsh, 0, &t_res);
		of_node_put(tipsh);
		if (ret) {
			dev_err(dev, "Failed to get shared mem resource\n");
			return ret;
		}

		priv->phy_start = t_res.start;
		priv->phy_size = resource_size(&t_res);
		if (priv->phy_size < MIN_RNG_BUFFER) {
			pr_info("Error, size of the shared memory is less then 2KB\n");
			return -ENODEV;
		}
		priv->virt_off = devm_ioremap_resource(dev, &t_res);
		if (IS_ERR(priv->virt_off)) {
			dev_err(dev, "device mem io remap failed\n");
			return PTR_ERR(priv->virt_off);
		}
	} else {
		pr_info("Error finding shared memory\n");
		return -ENODEV;
	}

	priv->rng.init = npcm_rng_tip_init;
	priv->rng.cleanup = npcm_rng_tip_cleanup;
	priv->rng.name = pdev->name;
	priv->rng.read = npcm_rng_tip_read;
	priv->rng.priv = (unsigned long)dev;
	priv->rng.quality = 1000;

	ret = devm_hwrng_register(dev, &priv->rng);
	if (ret) {
		dev_err(dev, "Failed to register rng device: %d\n", ret);
		return ret;
	}

	dev_info(dev, "client driver initialized\n");
	return 0;
}

static void npcm_rng_tip_remove(struct platform_device *pdev)
{
	struct npcm_rng_tip *priv = platform_get_drvdata(pdev);

	devm_hwrng_unregister(&pdev->dev, &priv->rng);
	mbox_free_channel(priv->chan);
}

static const struct of_device_id npcm_rng_tip_of_table[] __maybe_unused = {
	{ .compatible = "nuvoton,npcm845-rng-tip",  },
	{}
};
MODULE_DEVICE_TABLE(of, npcm_rng_tip_of_table);

static struct platform_driver npcm_rng_tip_driver = {
	.probe		=  npcm_rng_tip_probe,
	.remove =  npcm_rng_tip_remove,
	.driver   = {
	    .name  = "npcm_rng_tip",
	    .of_match_table = npcm_rng_tip_of_table,
	    .owner = THIS_MODULE,
	},
};

module_platform_driver(npcm_rng_tip_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("tomer.maimon@nuvoton.com");
MODULE_DESCRIPTION("Nuvoton NPCM HWRNG TIP driver");

