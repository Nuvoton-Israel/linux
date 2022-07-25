// SPDX-License-Identifier: GPL-2.0+
/*
 * NPCM SDHC MMC host controller driver.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/module.h>
#include <linux/of.h>

#include "sdhci-pltfm.h"

static const struct sdhci_pltfm_data npcm_sdhci_pdata = {
	.quirks  = SDHCI_QUIRK_DELAY_AFTER_POWER,
	.quirks2 = SDHCI_QUIRK2_STOP_WITH_TC |
		   SDHCI_QUIRK2_NO_1_8_V,
};

static int npcm_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	u32 caps;
	int ret;

	host = sdhci_pltfm_init(pdev, &npcm_sdhci_pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);

	if (!IS_ERR(pltfm_host->clk))
		clk_prepare_enable(pltfm_host->clk);

	caps = sdhci_readl(host, SDHCI_CAPABILITIES);
	if (caps & SDHCI_CAN_DO_8BIT)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	clk_disable_unprepare(pltfm_host->clk);
	sdhci_pltfm_free(pdev);
	return ret;
}

static const struct of_device_id npcm_sdhci_of_match[] = {
	{ .compatible = "nuvoton,npcm750-sdhci" },
	{ .compatible = "nuvoton,npcm845-sdhci" },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm_sdhci_of_match);

static struct platform_driver npcm_sdhci_driver = {
	.driver = {
		.name	= "npcm-sdhci",
		.of_match_table = npcm_sdhci_of_match,
		.pm	= &sdhci_pltfm_pmops,
	},
	.probe		= npcm_sdhci_probe,
	.remove		= sdhci_pltfm_unregister,
};

module_platform_driver(npcm_sdhci_driver);

MODULE_DESCRIPTION("NPCM Secure Digital Host Controller Interface driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
