// SPDX-License-Identifier: GPL-2.0
/*
 * Description   : NPCM7XX MCU ADC Driver
 *
 * Copyright (C) 2021 Nuvoton Corporation
 *
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#define DEVICE_NAME		"npcm7xx-mcu-adc"

/* NPCM7XX MCU ADC registers */
#define ADC_CMD_SD_SE	0x80	    /* Single ended inputs */
#define ADC_CMD_PD3		0x0C	    /* Internal vref ON && A/D ON */
#define ADC_INT_VREF_MV	2500	    /* Internal vref is 2.5V, 2500mV */

/* Client specific data */
struct npcm7xx_mcu_adc_data {
	struct regmap *regmap;
	u8 cmd_byte;			    /* Command byte without channel bits */
	unsigned int lsb_resol;		/* Resolution of the ADC sample LSB */
};

/* Command byte C2,C1,C0 */
static inline u8 npcm7xx_mcu_adc_cmd_byte(u8 cmd, int ch)
{
	return cmd | (((ch >> 1) | (ch & 0x01) << 2) << 4);
}

/* sysfs callback function */
static ssize_t npcm7xx_mcu_adc_in_show(struct device *dev,
			       struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct npcm7xx_mcu_adc_data *data = dev_get_drvdata(dev);
	u8 cmd = npcm7xx_mcu_adc_cmd_byte(data->cmd_byte, attr->index);
	unsigned int regval;
	int err;

	err = regmap_read(data->regmap, cmd, &regval);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n",
		       DIV_ROUND_CLOSEST(regval * data->lsb_resol, 1000));
}

static SENSOR_DEVICE_ATTR_RO(in0_input, npcm7xx_mcu_adc_in, 0);
static SENSOR_DEVICE_ATTR_RO(in1_input, npcm7xx_mcu_adc_in, 1);
static SENSOR_DEVICE_ATTR_RO(in2_input, npcm7xx_mcu_adc_in, 2);
static SENSOR_DEVICE_ATTR_RO(in3_input, npcm7xx_mcu_adc_in, 3);
static SENSOR_DEVICE_ATTR_RO(in4_input, npcm7xx_mcu_adc_in, 4);
static SENSOR_DEVICE_ATTR_RO(in5_input, npcm7xx_mcu_adc_in, 5);
static SENSOR_DEVICE_ATTR_RO(in6_input, npcm7xx_mcu_adc_in, 6);
static SENSOR_DEVICE_ATTR_RO(in7_input, npcm7xx_mcu_adc_in, 7);

static struct attribute *npcm7xx_mcu_adc_attrs[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(npcm7xx_mcu_adc);

static const struct regmap_config npcm7xx_mcu_adc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int npcm7xx_mcu_adc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct npcm7xx_mcu_adc_data *data;
	struct device *hwmon_dev;
	unsigned int vref_mv = ADC_INT_VREF_MV;
	unsigned int regval;

	data = devm_kzalloc(dev, sizeof(struct npcm7xx_mcu_adc_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	of_device_get_match_data(&client->dev);

	/* NPCM7XX MCU ADC uses 8-bit samples */
	data->lsb_resol = DIV_ROUND_CLOSEST(vref_mv * 1000, 256);
	data->regmap = devm_regmap_init_i2c(client,
						    &npcm7xx_mcu_adc_regmap_config);

	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	/* NPCM7XX MCU ADC uses internal vref */
	data->cmd_byte |= ADC_CMD_PD3;

	/* NPCM7XX MCU ADC uses single Ended mode */
	data->cmd_byte |= ADC_CMD_SD_SE;

	/* Perform a dummy read to enable the internal reference voltage */
	regmap_read(data->regmap, data->cmd_byte, &regval);

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data,
							   npcm7xx_mcu_adc_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct of_device_id npcm7xx_mcu_adc_of_match[] = {
	{ .compatible = "nuvoton,mcu71adc", },
	{},
};

MODULE_DEVICE_TABLE(of, npcm7xx_mcu_adc_of_match);

static struct i2c_driver npcm7xx_mcu_adc_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = npcm7xx_mcu_adc_of_match,
	},
	.probe_new = npcm7xx_mcu_adc_probe,
};

module_i2c_driver(npcm7xx_mcu_adc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim Lee <chli30@nuvoton.com>");
MODULE_DESCRIPTION("NPCM7XX MCU ADC Driver");
