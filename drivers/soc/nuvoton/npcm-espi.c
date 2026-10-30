// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NPCM eSPI interrupt controller
 *
 * The eSPI block raises one interrupt line for every channel it
 * implements, and reports them through a single status register shared by
 * drivers that live in different subsystems: the peripheral channel in
 * drivers/soc, the flash channel in drivers/spi and virtual wires in
 * drivers/gpio.  Letting each of them drive that register directly leaves
 * no way to say who clears what, and a bit nobody clears locks the CPU in
 * interrupt context.
 *
 * Own the register here instead and hand the children one interrupt each,
 * so a child can neither see nor acknowledge a channel that is not its
 * own.  Configuration updates are the exception: they are a notification
 * rather than work, and every channel needs to hear about them, so they
 * are kept here and broadcast.
 *
 * Copyright (c) 2026 Nuvoton Technology corporation.
 */

#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/soc/nuvoton/npcm-espi.h>

struct npcm_espi {
	struct device			*dev;
	struct regmap			*map;
	struct regmap_irq_chip_data	*irq_data;
	struct blocking_notifier_head	notifier;
};

static const struct regmap_irq npcm_espi_irqs[] = {
	REGMAP_IRQ_REG(NPCM_ESPI_IRQ_LINE_CFGUPD,   0, NPCM_ESPI_CFGUPD),
	REGMAP_IRQ_REG(NPCM_ESPI_IRQ_LINE_FLASHRX,  0, NPCM_ESPI_FLASHRX),
	REGMAP_IRQ_REG(NPCM_ESPI_IRQ_LINE_SFLASHRD, 0, NPCM_ESPI_SFLASHRD),
	REGMAP_IRQ_REG(NPCM_ESPI_IRQ_LINE_VWUPD,    0, NPCM_ESPI_VWUPD),
	REGMAP_IRQ_REG(NPCM_ESPI_IRQ_LINE_ESPIRST,  0, NPCM_ESPI_ESPIRST),
	REGMAP_IRQ_REG(NPCM_ESPI_IRQ_LINE_PLTRST,   0, NPCM_ESPI_PLTRST),
};

/*
 * ESPIIE enables rather than masks, and ESPISTS is write-one-to-clear, so
 * regmap-irq can drive both directly.  Everything starts disabled and is
 * switched on only by a child asking for its interrupt, which is what
 * keeps a channel with no driver from ever raising the line.
 */
static const struct regmap_irq_chip npcm_espi_irq_chip = {
	.name		= "npcm-espi",
	.status_base	= NPCM_ESPI_ESPISTS,
	.unmask_base	= NPCM_ESPI_ESPIIE,
	.ack_base	= NPCM_ESPI_ESPISTS,
	.num_regs	= 1,
	.irqs		= npcm_espi_irqs,
	.num_irqs	= ARRAY_SIZE(npcm_espi_irqs),
};

struct npcm_espi_notifier {
	struct npcm_espi	*espi;
	struct notifier_block	*nb;
};

static void npcm_espi_remove_notifier(void *data)
{
	struct npcm_espi_notifier *n = data;

	blocking_notifier_chain_unregister(&n->espi->notifier, n->nb);
}

/**
 * devm_npcm_espi_register_notifier() - hear about eSPI configuration updates
 * @dev: a child of the eSPI block
 * @nb: caller's notifier, invoked with %NPCM_ESPI_EVENT_CFGUPD and a
 *	pointer to the new ESPICFG value
 *
 * The host sends a configuration update whenever it changes the channels
 * it has enabled.  That is one event several drivers have to react to, so
 * it is not an interrupt any single one of them can own.  Callbacks run in
 * process context and are removed when @dev is unbound.
 */
int devm_npcm_espi_register_notifier(struct device *dev,
				     struct notifier_block *nb)
{
	struct npcm_espi_notifier *n;
	struct npcm_espi *espi;
	int ret;

	if (!dev->parent)
		return -ENODEV;

	espi = dev_get_drvdata(dev->parent);
	if (!espi)
		return -EPROBE_DEFER;

	n = devm_kzalloc(dev, sizeof(*n), GFP_KERNEL);
	if (!n)
		return -ENOMEM;

	n->espi = espi;
	n->nb = nb;

	ret = blocking_notifier_chain_register(&espi->notifier, nb);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, npcm_espi_remove_notifier, n);
}
EXPORT_SYMBOL_GPL(devm_npcm_espi_register_notifier);

static irqreturn_t npcm_espi_cfgupd_thread(int irq, void *arg)
{
	struct npcm_espi *espi = arg;
	unsigned int cfg;

	if (regmap_read(espi->map, NPCM_ESPI_ESPICFG, &cfg))
		return IRQ_NONE;

	dev_dbg(espi->dev, "configuration update, ESPICFG = %08x\n", cfg);

	blocking_notifier_call_chain(&espi->notifier,
				     NPCM_ESPI_EVENT_CFGUPD, &cfg);

	return IRQ_HANDLED;
}

static int npcm_espi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct npcm_espi *espi;
	int irq, virq, ret;

	espi = devm_kzalloc(dev, sizeof(*espi), GFP_KERNEL);
	if (!espi)
		return -ENOMEM;

	espi->dev = dev;
	BLOCKING_INIT_NOTIFIER_HEAD(&espi->notifier);

	espi->map = syscon_node_to_regmap(dev->of_node);
	if (IS_ERR(espi->map))
		return dev_err_probe(dev, PTR_ERR(espi->map),
				     "couldn't get the eSPI regmap\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/*
	 * Mask the block before taking the line.  The status register is
	 * left alone on purpose: it latches while masked, so anything the
	 * host sent before we got here is replayed the moment a child asks
	 * for that interrupt, and a bit no child asks for is never enabled.
	 */
	regmap_write(espi->map, NPCM_ESPI_ESPIIE, 0);

	ret = devm_regmap_add_irq_chip_fwnode(dev, dev_fwnode(dev), espi->map,
					      irq, 0, 0, &npcm_espi_irq_chip,
					      &espi->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add the irq chip\n");

	/*
	 * Take the configuration update for ourselves.  Nothing else may
	 * claim it, which is the point: it is the one event more than one
	 * driver needs, and it used to be the one nobody agreed to clear.
	 */
	virq = regmap_irq_get_virq(espi->irq_data, NPCM_ESPI_IRQ_LINE_CFGUPD);
	if (virq < 0)
		return virq;

	ret = devm_request_threaded_irq(dev, virq, NULL,
					npcm_espi_cfgupd_thread, IRQF_ONESHOT,
					"espi-cfgupd", espi);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request the cfgupd irq\n");

	platform_set_drvdata(pdev, espi);

	/*
	 * Children are created here rather than by simple-mfd so that they
	 * cannot probe, and so cannot ask for an interrupt, before the
	 * domain above is able to give them one.
	 */
	return devm_of_platform_populate(dev);
}

static const struct of_device_id npcm_espi_match[] = {
	{ .compatible = "nuvoton,npcm845-espi" },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm_espi_match);

static struct platform_driver npcm_espi_driver = {
	.driver = {
		.name		= "npcm-espi",
		.of_match_table	= npcm_espi_match,
	},
	.probe	= npcm_espi_probe,
};
module_platform_driver(npcm_espi_driver);

MODULE_AUTHOR("Nuvoton Technology corporation");
MODULE_DESCRIPTION("Nuvoton NPCM eSPI interrupt controller");
MODULE_LICENSE("GPL");
