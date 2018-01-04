/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/random.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/hw_random.h>
#include <linux/delay.h>
#include <linux/of_irq.h>

#define RNGCS_REG		0x00	/* Control and status register */
#define RNGD_REG		0x04	/* DATA register */
#define RNGTST_REG		0x08	/* TEST register */

#define RNG_CLK_SET             (0x06 << 2)    /* 20-25 MHz */
#define RNG_DATA_VALID          (0x01 << 1)
#define RNG_ENABLE              0x01

#define RNG_DEBUG

static void __iomem *rng_base;
static struct platform_device *rng_dev;

static inline u32 npcm750_rng_read_reg(int reg)
{
	return __raw_readb(rng_base + reg);
}

static inline void npcm750_rng_write_reg(int reg, u32 val)
{
	__raw_writeb(val, rng_base + reg);
}

static int npcm750_rng_data_present(struct hwrng *rng, int wait)
{
	int data, i;

	for (i = 0; i < 20; i++) {
		data = (npcm750_rng_read_reg(RNGCS_REG) & RNG_DATA_VALID) ?
			1 : 0;
		if (data || !wait)
			break;
		/* RNG produces data fast enough (2+ MBit/sec, even
		 * during "rngtest" loads, that these delays don't
		 * seem to trigger.  We *could* use the RNG IRQ, but
		 * that'd be higher overhead ... so why bother?
		 */
		udelay(10);
	}

	return data;
}

static int npcm750_rng_data_read(struct hwrng *rng, u32 *data)
{
	*data = npcm750_rng_read_reg(RNGD_REG);

	return 1;
}

static struct hwrng npcm750_rng_ops = {
	.name		= "npcm750",
	.data_present	= npcm750_rng_data_present,
	.data_read	= npcm750_rng_data_read,
};

static int npcm750_rng_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	/*
	 * A bit ugly, and it will never actually happen but there can
	 * be only one RNG and this catches any bork
	 */
	if (rng_dev)
		return -EBUSY;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		ret = -ENOENT;
		goto err_region;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		ret = -EBUSY;
		goto err_region;
	}

	dev_set_drvdata(&pdev->dev, res);
	rng_base = ioremap(res->start, resource_size(res));
	if (!rng_base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = hwrng_register(&npcm750_rng_ops);
	if (ret)
		goto err_register;

	npcm750_rng_write_reg(RNGTST_REG, 0x02);
	npcm750_rng_write_reg(RNGCS_REG, RNG_CLK_SET | RNG_ENABLE);

	rng_dev = pdev;

#ifdef RNG_DEBUG
	mdelay(10);
	pr_info("RNG-Random Number Generator 0x%x\n",
	       npcm750_rng_read_reg(RNGD_REG));
#else
	pr_info("RNG-Random Number Generator\n");
#endif

	return 0;

err_register:
	iounmap(rng_base);
	rng_base = NULL;
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_region:

	return ret;
}

static int __exit npcm750_rng_remove(struct platform_device *pdev)
{
	struct resource *res = dev_get_drvdata(&pdev->dev);

	hwrng_unregister(&npcm750_rng_ops);
	npcm750_rng_write_reg(RNGCS_REG, 0x0);
	iounmap(rng_base);
	release_mem_region(res->start, resource_size(res));

	rng_base = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int npcm750_rng_suspend(struct platform_device *pdev,
			       pm_message_t message)
{
	u32 val;

	val = npcm750_rng_read_reg(RNGCS_REG);
	val &= 0xfffffffe;
	npcm750_rng_write_reg(RNGCS_REG, val);
	return 0;
}

static int npcm750_rng_resume(struct platform_device *pdev)
{
	u32 val;

	val = npcm750_rng_read_reg(RNGCS_REG);
	val |= 0x00000001;
	npcm750_rng_write_reg(RNGCS_REG, val);
	return 0;
}

#else

#define	npcm750_rng_suspend	NULL
#define	npcm750_rng_resume	NULL

#endif

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750-rng");

static const struct of_device_id rng_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-rng",  },
	{},
};
MODULE_DEVICE_TABLE(of, rng_dt_id);

static struct platform_driver npcm750_rng_driver = {
	.driver = {
		.name		= "npcm750-rng",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(rng_dt_id),
	},
	.probe		= npcm750_rng_probe,
	.remove		= __exit_p(npcm750_rng_remove),
	.suspend	= npcm750_rng_suspend,
	.resume		= npcm750_rng_resume
};

module_platform_driver(npcm750_rng_driver);

MODULE_AUTHOR("Deepak Saxena (and others)");
MODULE_LICENSE("GPL");
