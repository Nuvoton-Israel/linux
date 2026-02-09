// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C NPCM multiplexer 
 */

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "../../pinctrl/core.h"

#define NPCM_GCR_I2CSEGCTL	0xE4
#define MAX_MUX_NUMBER		4

struct i2c_mux_npcm {
	u32 i2c_number;
	struct regmap *gcr_regmap;
	u32 i2c_segment[MAX_MUX_NUMBER];
};

static int i2c_mux_npcm_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct i2c_mux_npcm *mux = i2c_mux_priv(muxc);
	u32 smbxss = 0;

	if (chan >= MAX_MUX_NUMBER) {
		dev_err(muxc->dev, "Channel index out of bounds: %u\n", chan);
		return -EINVAL;
	}
	/* Adding WENxSS */
	if (mux->i2c_number < 6)
		smbxss = BIT(mux->i2c_number + 12) | 
			(mux->i2c_segment[chan] << (mux->i2c_number * 2));
	if (mux->i2c_number == 6)
		smbxss = BIT(28) | (mux->i2c_segment[chan] << 26);
	if (mux->i2c_number == 7)
		smbxss = BIT(31) | (mux->i2c_segment[chan] << 29);

	/* Setting smb segment select */
	regmap_write(mux->gcr_regmap, NPCM_GCR_I2CSEGCTL, smbxss);
	return 0;
}

static struct i2c_adapter *i2c_mux_npcm_parent_adapter(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *parent_np;
	struct i2c_adapter *parent;

	parent_np = of_parse_phandle(np, "i2c-parent", 0);
	if (!parent_np) {
		dev_err(dev, "Cannot parse i2c-parent\n");
		return ERR_PTR(-ENODEV);
	}
	parent = of_find_i2c_adapter_by_node(parent_np);
	of_node_put(parent_np);
	if (!parent)
		return ERR_PTR(-EPROBE_DEFER);

	return parent;
}

static int i2c_mux_npcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct i2c_mux_core *muxc;
	struct i2c_mux_npcm *mux;
	struct i2c_adapter *parent;
	int num_segments, i, ret;

	num_segments = of_property_count_u32_elems(np, "i2c_segment_number");
	if (num_segments < 0) {
		dev_err(dev, "Cannot parse i2c_segment_number: %d\n",
			num_segments);
		return num_segments;
	}

	parent = i2c_mux_npcm_parent_adapter(dev);
	if (IS_ERR(parent))
		return PTR_ERR(parent);

	muxc = i2c_mux_alloc(parent, dev, num_segments,
			     struct_size(mux, i2c_segment, num_segments), 0,
			     i2c_mux_npcm_select, NULL);
	if (!muxc) {
		ret = -ENOMEM;
		goto err_put_parent;
	}
	mux = i2c_mux_priv(muxc);

	mux->gcr_regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "syscon");
	if (IS_ERR(mux->gcr_regmap)) {
		dev_err(&pdev->dev, "Failed to find syscon\n");
		ret = PTR_ERR(mux->gcr_regmap);
		goto err_put_parent;
	}

	if (of_property_read_u32(pdev->dev.of_node, "nuvoton,i2c-number",
				  &mux->i2c_number)) {
		dev_err(&pdev->dev, "nuvoton,i2c-number not found");
		ret =  -ENODEV;
		goto err_put_parent;
	}

	platform_set_drvdata(pdev, muxc);

	for (i = 0; i < num_segments; i++) {
		ret = of_property_read_u32_index(np, "i2c_segment_number", i,
						 &mux->i2c_segment[i]);
		if (ret < 0) {
			dev_err(dev, "Cannot i2c_segment_number: %d\n", ret);
			goto err_put_parent;
		}
	}

	muxc->mux_locked = true;
	dev_info(dev, "mux-locked i2c mux\n");

	/* Do not add any adapter for the idle state (if it's there at all). */
	for (i = 0; i < num_segments - !!muxc->deselect; i++) {
		ret = i2c_mux_add_adapter(muxc, 0, i);
		if (ret)
			goto err_del_adapter;
	}

	return 0;

err_del_adapter:
	i2c_mux_del_adapters(muxc);
err_put_parent:
	i2c_put_adapter(parent);

	return ret;
}

static void i2c_mux_npcm_remove(struct platform_device *pdev)
{
	struct i2c_mux_core *muxc = platform_get_drvdata(pdev);

	i2c_mux_del_adapters(muxc);
	i2c_put_adapter(muxc->parent);
}

static const struct of_device_id i2c_mux_npcm_of_match[] = {
	{ .compatible = "nuvoton,i2c-mux-npcm", },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_mux_npcm_of_match);

static struct platform_driver i2c_mux_npcm_driver = {
	.driver	= {
		.name	= "i2c-mux-npcm",
		.of_match_table = of_match_ptr(i2c_mux_npcm_of_match),
	},
	.probe		= i2c_mux_npcm_probe,
	.remove_new	= i2c_mux_npcm_remove,
};
module_platform_driver(i2c_mux_npcm_driver);

MODULE_DESCRIPTION("Nuvoton NPCM I2C multiplexer driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-mux-npcm");
