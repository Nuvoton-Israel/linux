// SPDX-License-Identifier: GPL-2.0+
/*
 * NPCM Thermal Sensor Driver
 *
 * Copyright (c) 2021 Nuvoton Technology corporation.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include "thermal_core.h"
#include "thermal_hwmon.h"

/* NPCM7xx GCR module */
#define NPCM8XX_THRTL_OFFSET		0xC0
#define NPCM8XX_THRTL_TH_DIV_MASK	GENMASK(1, 0)
#define NPCM8XX_THRTL_TH_FULL		0
#define NPCM8XX_THRTL_TH_DIV_HALF	BIT(0)
#define NPCM8XX_THRTL_TH_DIV_QUARTER	BIT(1)

/* The NPCM-THRM registers */
#define NPCM_THRM_REG_BASE(base, n)    ((base) + ((n) * 0x10L))

#define NPCM_THRM_CTL_REG		0x0
#define NPCM_THRM_STAT_REG		0x4
#define NPCM_THRM_INTEN_REG		0x8
#define NPCM_THRM_ANCTL_REG		0x10
#define NPCM_THRM_OUT_REG(base, n)	(NPCM_THRM_REG_BASE(base, n) + 0x40)
#define NPCM_THRM_THA_REG(base, n)	(NPCM_THRM_REG_BASE(base, n) + 0x44)
#define NPCM_THRM_THB_REG(base, n)	(NPCM_THRM_REG_BASE(base, n) + 0x48)
	      
#define NPCM_THRM_CTL_EN		BIT(0)
#define NPCM_THRM_CTL_START		BIT(1)
#define NPCM_THRM_CTL_RESETN		BIT(4)
#define NPCM_THRM_CTL_SLEEP		BIT(12)
#define NPCM_THRM_CTL_SNSCLKDIV		GENMASK(23, 16)
#define NPCM_THRM_CTL_TEMP_MODE		GENMASK(25, 24)
	      
#define NPCM_THRM_STAT_DA0		BIT(0)
#define NPCM_THRM_STAT_DA1		BIT(1)
#define NPCM_THRM_STAT_ATH0		BIT(8)
#define NPCM_THRM_STAT_ATH1		BIT(9)
#define NPCM_THRM_STAT_BTH0		BIT(12)
#define NPCM_THRM_STAT_BTH1		BIT(13)
	      
#define NPCM_THRM_INTEN_DAIE0		BIT(0)
#define NPCM_THRM_INTEN_DAIE1		BIT(1)
#define NPCM_THRM_INTEN_ATHIE0		BIT(8)
#define NPCM_THRM_INTEN_ATHIE1		BIT(9)
#define NPCM_THRM_INTEN_BTHIE0		BIT(12)
#define NPCM_THRM_INTEN_BTHIE1		BIT(13)
	      
#define NPCM_THRM_ANCTL_DEFAULT		0x4
	      
#define NPCM_THRM_DATA_OUT_MASK		GENMASK(9, 0)
#define NPCM_THRM_THA_MASK		GENMASK(9, 0)
#define NPCM_THRM_THB_MASK		GENMASK(9, 0)
	      
#define NPCM_THRM_TEMP_MIN		(-273820)
#define NPCM_THRM_TEMP_MAX		370670
	      
#define NPCM_THRM_START			1
#define NPCM_THRM_STOP			0

struct npcm_thermal_priv {
	const struct npcm_thrm_data *data;
	struct thermal_zone_device *overheat_sensor;
	int 		current_channel;
	u8		current_conf;	
	struct regmap 	*clk_regmap;
	u32		orig_conf;
	struct device 	*dev;
	void __iomem	*base;
	struct clk 	*clk;
	u8		enable;
	u8		start;
};

struct npcm_thermal_sensor {
	struct npcm_thermal_priv *priv;
	int id;
};

void npcm_thermal_initialize(struct npcm_thermal_priv *tmps)
{
	u32 ctl;

	/* init fault registers */
	iowrite32(0, NPCM_THRM_THA_REG(tmps->base, 0));
	iowrite32(0, NPCM_THRM_THA_REG(tmps->base, 1));
	iowrite32(0x3FF, NPCM_THRM_THB_REG(tmps->base, 0));
	iowrite32(0x3FF, NPCM_THRM_THB_REG(tmps->base, 1));

	/* init sequence */
	ctl = ioread32(tmps->base + NPCM_THRM_CTL_REG);
	ctl |= NPCM_THRM_CTL_RESETN;
	iowrite32(ctl, tmps->base + NPCM_THRM_CTL_REG);
	udelay(100);
	ctl &= ~NPCM_THRM_CTL_SLEEP;
	iowrite32(ctl, tmps->base + NPCM_THRM_CTL_REG);
	udelay(10);
	iowrite32(NPCM_THRM_ANCTL_DEFAULT, tmps->base + NPCM_THRM_ANCTL_REG);
	udelay(10);
	ctl |= NPCM_THRM_CTL_EN;
	iowrite32(ctl, tmps->base + NPCM_THRM_CTL_REG);
	udelay(100);
	ctl |= NPCM_THRM_CTL_START;
	iowrite32(ctl, tmps->base + NPCM_THRM_CTL_REG);
}

static int npcm_thermal_enable(struct npcm_thermal_priv *tmps)
{
	u32 ctl;

	ctl = ioread32(tmps->base + NPCM_THRM_CTL_REG);
	ctl |= NPCM_THRM_CTL_START;
	iowrite32(ctl, tmps->base + NPCM_THRM_CTL_REG);

	return 0;
}

static int npcm_thermal_disable(struct npcm_thermal_priv *tmps)
{
	u32 ctl;

	ctl = ioread32(tmps->base + NPCM_THRM_CTL_REG);
	ctl &= ~NPCM_THRM_CTL_START;
	iowrite32(ctl, tmps->base + NPCM_THRM_CTL_REG);

	return 0;
}

static irqreturn_t npcm_thermal_isr(int irq, void *data)
{
	struct npcm_thermal_priv *tmps = data;
	int regval;

	/* Notify the core in thread context */
	thermal_zone_device_update(tmps->overheat_sensor,
				   THERMAL_EVENT_UNSPECIFIED);

	regval = ioread32(tmps->base + NPCM_THRM_STAT_REG);
	if (regval & (NPCM_THRM_STAT_ATH0 | NPCM_THRM_STAT_ATH1)) {
		iowrite32(NPCM_THRM_INTEN_BTHIE0 | NPCM_THRM_INTEN_BTHIE1,
			  tmps->base + NPCM_THRM_INTEN_REG);
		regmap_update_bits(tmps->clk_regmap, NPCM8XX_THRTL_OFFSET,
				   NPCM8XX_THRTL_TH_DIV_MASK,
				   NPCM8XX_THRTL_TH_DIV_HALF);
	}
	if (regval & (NPCM_THRM_STAT_BTH0 | NPCM_THRM_STAT_BTH1)) {
		iowrite32(NPCM_THRM_INTEN_ATHIE0 | NPCM_THRM_INTEN_ATHIE1,
			  tmps->base + NPCM_THRM_INTEN_REG);
		regmap_update_bits(tmps->clk_regmap, NPCM8XX_THRTL_OFFSET,
				   NPCM8XX_THRTL_TH_DIV_MASK,
				   NPCM8XX_THRTL_TH_FULL);
	}
	iowrite32(regval, tmps->base + NPCM_THRM_STAT_REG);

	/* Notify the thermal core that the temperature is acceptable again */
	thermal_zone_device_update(tmps->overheat_sensor,
				   THERMAL_EVENT_UNSPECIFIED);

	return IRQ_HANDLED;
}

static int npcm_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct npcm_thermal_sensor *sensor = tz->devdata;
	struct npcm_thermal_priv *tmps = sensor->priv;
	int regval;

	if (!tz->devdata)
		return -EINVAL;

	regval = ioread32(NPCM_THRM_OUT_REG(tmps->base, sensor->id));
	regval &= NPCM_THRM_DATA_OUT_MASK;
	*temp = (630 * (509 - regval)) + 50000;

	return 0;
}

static const struct thermal_zone_device_ops npcm_thermal_ops = {
	.get_temp	= npcm_thermal_get_temp,
};

struct npcm_thrm_data {
	int num_mask_bit;
	int max_channel;
	int max_temp;
	int min_temp;
};

static const struct npcm_thrm_data npxm8xx_tmps_data = {
	.num_mask_bit = 10,
	.max_channel = 2,
	.max_temp = 370670,
	.min_temp = -273820,
};

static const struct of_device_id of_npcm_thermal_match[] = {
	{
		.compatible = "nuvoton,npcm845-thermal",
		.data = &npxm8xx_tmps_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_npcm_thermal_match);

static void npcm_set_overheat_thresholds(struct npcm_thermal_priv *tmps,
					 int thresh_mc, int hyst_mc, 
					 int channel)
{
	int threshold, hysteresis;

	threshold = clamp_val(thresh_mc, NPCM_THRM_TEMP_MIN, NPCM_THRM_TEMP_MAX);
	iowrite32(DIV_ROUND_CLOSEST(370670-threshold, 630),
		  NPCM_THRM_THA_REG(tmps->base, channel));

	hysteresis = clamp_val(hyst_mc, NPCM_THRM_TEMP_MIN, NPCM_THRM_TEMP_MAX);
	iowrite32(DIV_ROUND_CLOSEST(370670-(threshold + hysteresis), 630),
		  NPCM_THRM_THB_REG(tmps->base, channel));
}

static int npcm_configure_overheat_int(struct npcm_thermal_priv *tmps,
				       struct thermal_zone_device *tz,
				       int channel)
{
	/* Retrieve the critical trip point to enable the overheat interrupt */
	const struct thermal_trip *trips = of_thermal_get_trip_points(tz);
	int ctl;
	int i;

	if (!trips)
		return -EINVAL;

	for (i = 0; i < of_thermal_get_ntrips(tz); i++)
		if (trips[i].type == THERMAL_TRIP_CRITICAL)
			break;

	if (i == of_thermal_get_ntrips(tz))
		return -EINVAL;

	tmps->current_channel = channel;
	npcm_set_overheat_thresholds(tmps, trips[channel].temperature,
				     trips[channel].hysteresis, channel);
	tmps->overheat_sensor = tz;

	ctl = ioread32(tmps->base + NPCM_THRM_INTEN_REG);
	if (channel == 0)
		ctl |= NPCM_THRM_INTEN_ATHIE0 | NPCM_THRM_INTEN_BTHIE0;
	if (channel == 1)
		ctl |= NPCM_THRM_INTEN_ATHIE1 | NPCM_THRM_INTEN_BTHIE1;
	iowrite32(ctl, tmps->base + NPCM_THRM_INTEN_REG);

	return 0;
}

static int npcm_thermal_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *tz;
	struct npcm_thermal_sensor *sensor;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct npcm_thermal_priv *priv;
	int sensor_id, irq;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct npcm_thermal_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	irq = platform_get_irq(pdev, 0);
	if (irq == -EPROBE_DEFER)
		return irq;

	match = of_match_device(of_npcm_thermal_match, dev);
	if (!match || !match->data) {
		dev_err(dev, "No compatible OF match\n");
		return -ENODEV;
	}
	priv->data = (struct npcm_thrm_data *)match->data;

	priv->current_channel = -1;

	/* The overheat interrupt feature is not mandatory */
	if (irq > 0) {
		ret = devm_request_irq(dev, irq, npcm_thermal_isr, 0,
				       "NPCM_THERMAL", priv);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed requesting interrupt\n");
			return ret;
		}
		if (of_device_is_compatible(dev->of_node, "nuvoton,npcm845-thermal")) {
			priv->clk_regmap =
				syscon_regmap_lookup_by_compatible("nuvoton,npcm845-rst");
			if (IS_ERR(priv->clk_regmap)) {
				dev_warn(dev, "Didn't find nuvoton,npcm845-rst, cpu throttling disabled\n");
			}
		}
	}

	npcm_thermal_initialize(priv);

	 /* There is two channels for BMC */
	for (sensor_id = 0; sensor_id < priv->data->max_channel; sensor_id++) {
		sensor = devm_kzalloc(&pdev->dev,
				      sizeof(struct npcm_thermal_sensor),
				      GFP_KERNEL);
		if (!sensor)
			return -ENOMEM;

		/* Register the sensor */
		sensor->priv = priv;
		sensor->id = sensor_id;
		tz = devm_thermal_of_zone_register(&pdev->dev,
						   sensor->id, sensor,
						   &npcm_thermal_ops);
		if (IS_ERR(tz)) {
			dev_info(&pdev->dev, "Thermal sensor %d unavailable\n",
				 sensor_id);
			devm_kfree(&pdev->dev, sensor);
			continue;
		}

		/*
		 * The first channel that has a critical trip point registered
		 * in the DT will serve as interrupt source. Others possible
		 * critical trip points will simply be ignored by the driver.
		 */
		if (irq > 0 && !priv->overheat_sensor)
			npcm_configure_overheat_int(priv, tz, sensor->id);

		if (devm_thermal_add_hwmon_sysfs(tz))
			dev_warn(&pdev->dev, "failed to add hwmon sysfs attributes\n");

	}

	/* Just complain if no overheat interrupt was set up */
	if (!priv->overheat_sensor)
		dev_warn(&pdev->dev, "Overheat interrupt not available\n");

	pr_info("npcm_thermal_probe Done\n");

	return 0;
}

static int npcm_thermal_remove(struct platform_device *pdev)
{
	struct npcm_thermal_sensor *sensor = platform_get_drvdata(pdev);

	return npcm_thermal_disable(sensor->priv);
}

static int __maybe_unused npcm_thermal_suspend(struct device *dev)
{
	struct npcm_thermal_sensor *sensor = dev_get_drvdata(dev);

	return npcm_thermal_disable(sensor->priv);
}

static int __maybe_unused npcm_thermal_resume(struct device *dev)
{
	struct npcm_thermal_sensor *sensor = dev_get_drvdata(dev);

	return npcm_thermal_enable(sensor->priv);
}

static SIMPLE_DEV_PM_OPS(npcm_thermal_pm_ops,
			 npcm_thermal_suspend, npcm_thermal_resume);

static struct platform_driver npcm_thermal_driver = {
	.driver = {
		.name		= "npcm_thermal",
		.pm		= &npcm_thermal_pm_ops,
		.of_match_table = of_npcm_thermal_match,
	},
	.probe	= npcm_thermal_probe,
	.remove	= npcm_thermal_remove,
};

module_platform_driver(npcm_thermal_driver);

MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_DESCRIPTION("NPCM thermal driver");
MODULE_LICENSE("GPL v2");

