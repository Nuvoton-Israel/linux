// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 Nuvoton Technology corporation.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define NCT7363_REG_GPIO0_INPUT		0x00
#define NCT7363_REG_GPIO0_OUTPUT	0x01
#define NCT7363_REG_GPIO0_IOCFG		0x03
#define NCT7363_REG_GPIO0_INT_CTL	0x06
#define NCT7363_REG_GPIO0_INT_STS	0x07
#define NCT7363_REG_GPIO1_INPUT		0x10
#define NCT7363_REG_GPIO1_OUTPUT	0x11
#define NCT7363_REG_GPIO1_IOCFG		0x13
#define NCT7363_REG_GPIO1_INT_CTL	0x16
#define NCT7363_REG_GPIO1_INT_STS	0x17
#define NCT7363_REG_FUNC_CFG_BASE(x)	(0x20 + (x))
#define NCT7363_REG_PWMEN_BASE(x)	(0x38 + (x))
#define NCT7363_REG_FANINEN_BASE(x)	(0x41 + (x))
#define NCT7363_REG_FANINX_HVAL(x)	(0x48 + ((x) * 2))
#define NCT7363_REG_FANINX_LVAL(x)	(0x49 + ((x) * 2))
#define NCT7363_REG_FSCPXDUTY(x)	(0x90 + ((x) * 2))

#define PWM_SEL(x)			(BIT(0) << ((x) * 2))
#define FANIN_SEL(x)			(BIT(1) << ((x < 8) ? \
					 (((x) + 8) * 2) : \
					 (((x) % 8) * 2)))
#define VALUE_TO_REG(x, y)		(((x) >> ((y) * 8)) & 0xFF)

#define NCT7363_FANINX_LVAL_MASK	GENMASK(4, 0)
#define NCT7363_FANIN_MASK		GENMASK(12, 0)

#define NCT7363_PWM_DUTY_MAX		GENMASK(7, 0)
#define NCT7363_PWM_COUNT		16

static inline unsigned int FAN_FROM_REG(u16 val)
{
	if (val == NCT7363_FANIN_MASK || val == 0)
		return	0;

	return (1350000UL / val);
}

static const struct i2c_device_id nct7363_id[] = {
	{ "nct7363", },
	{ },
};
MODULE_DEVICE_TABLE(i2c, nct7363_id);

static const struct of_device_id nct7363_of_match[] = {
	{ .compatible = "nuvoton,nct7363" },
	{ },
};
MODULE_DEVICE_TABLE(of, nct7363_of_match);

struct nct7363_data {
	struct gpio_chip	gpio;
	struct regmap		*regmap;
	struct mutex		lock; /* protect register access */
	struct i2c_client	*client;

	u16			fanin_mask;
	u16			pwm_mask;
};

static int nct7363_read_fan(struct device *dev, u32 attr, int channel,
			    long *val)
{
	struct nct7363_data *data = dev_get_drvdata(dev);
	unsigned int hi, lo, rpm;
	int ret = 0;
	u16 cnt;

	switch (attr) {
	case hwmon_fan_input:
		/*
		 * High-byte register should be read first to latch
		 * synchronous low-byte value
		 */
		mutex_lock(&data->lock);
		ret = regmap_read(data->regmap,
				  NCT7363_REG_FANINX_HVAL(channel), &hi);
		if (ret)
			return ret;

		ret = regmap_read(data->regmap,
			NCT7363_REG_FANINX_LVAL(channel), &lo);
		if (ret)
			return ret;
		mutex_unlock(&data->lock);

		cnt = (hi << 5) | (lo & NCT7363_FANINX_LVAL_MASK);
		rpm = FAN_FROM_REG(cnt);
		*val = (long)rpm;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t nct7363_fan_is_visible(const void *_data, u32 attr, int channel)
{
	const struct nct7363_data *data = _data;

	switch (attr) {
	case hwmon_fan_input:
		if (data->fanin_mask & BIT(channel))
			return 0444;
		break;
	default:
		break;
	}

	return 0;
}

static int nct7363_read_pwm(struct device *dev, u32 attr, int channel,
			    long *val)
{
	struct nct7363_data *data = dev_get_drvdata(dev);
	unsigned int regval;
	int ret;

	switch (attr) {
	case hwmon_pwm_input:
		ret = regmap_read(data->regmap,
				  NCT7363_REG_FSCPXDUTY(channel), &regval);
		if (ret)
			return ret;

		*val = (long)regval;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int nct7363_write_pwm(struct device *dev, u32 attr, int channel,
			     long val)
{
	struct nct7363_data *data = dev_get_drvdata(dev);
	int ret;

	switch (attr) {
	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;

		ret = regmap_write(data->regmap,
				   NCT7363_REG_FSCPXDUTY(channel), val);

		return ret;

	default:
		return -EOPNOTSUPP;
	}
}

static umode_t nct7363_pwm_is_visible(const void *_data, u32 attr, int channel)
{
	const struct nct7363_data *data = _data;

	switch (attr) {
	case hwmon_pwm_input:
		if (data->pwm_mask & BIT(channel))
			return 0644;
		break;
	default:
		break;
	}

	return 0;
}

static int nct7363_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return nct7363_read_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return nct7363_read_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int nct7363_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_pwm:
		return nct7363_write_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t nct7363_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return nct7363_fan_is_visible(data, attr, channel);
	case hwmon_pwm:
		return nct7363_pwm_is_visible(data, attr, channel);
	default:
		return 0;
	}
}

static const struct hwmon_channel_info *nct7363_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT),
	NULL
};

static const struct hwmon_ops nct7363_hwmon_ops = {
	.is_visible = nct7363_is_visible,
	.read = nct7363_read,
	.write = nct7363_write,
};

static const struct hwmon_chip_info nct7363_chip_info = {
	.ops = &nct7363_hwmon_ops,
	.info = nct7363_info,
};

static void nct7363_irq_set_mask(struct irq_data *d, bool set)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct nct7363_data *data = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned int inbits;

	if (hwirq < 8)
		regmap_read(data->regmap, NCT7363_REG_GPIO0_INT_CTL, &inbits);
	else
		regmap_read(data->regmap, NCT7363_REG_GPIO1_INT_CTL, &inbits);

	if (set)
		inbits |= BIT(hwirq % 8);
	else
		inbits &= ~BIT(hwirq % 8);

	if (hwirq < 8)
		regmap_write(data->regmap, NCT7363_REG_GPIO0_INT_CTL, inbits);
	else
		regmap_write(data->regmap, NCT7363_REG_GPIO1_INT_CTL, inbits);
}

static void nct7363_irq_mask(struct irq_data *d)
{
	nct7363_irq_set_mask(d, false);
}

static void nct7363_irq_unmask(struct irq_data *d)
{
	nct7363_irq_set_mask(d, true);
}

static int nct7363_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct nct7363_data *data = gpiochip_get_data(gc);

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&data->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

        return 0;
}

static const struct irq_chip nct7363_irq_chip = {
	.name			= "NCT7363-GPIO-IRQ",
	.irq_mask		= nct7363_irq_mask,
	.irq_unmask		= nct7363_irq_unmask,
	.irq_set_type		= nct7363_irq_set_type,
	.flags			= IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_IMMUTABLE,
};

static int nct7363_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	struct nct7363_data *data = gpiochip_get_data(gc);
	u8 iocfg = offset < 8 ? NCT7363_REG_GPIO0_IOCFG
			      : NCT7363_REG_GPIO1_IOCFG;
	u8 bit = BIT(offset % 8);

	return regmap_update_bits(data->regmap, iocfg, bit, bit);
}

static int nct7363_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int offset, int val)
{
	struct nct7363_data *data = gpiochip_get_data(gc);
	u8 outreg = offset < 8 ? NCT7363_REG_GPIO0_OUTPUT
			       : NCT7363_REG_GPIO1_OUTPUT;
	u8 iocfg = offset < 8 ? NCT7363_REG_GPIO0_IOCFG
			      : NCT7363_REG_GPIO1_IOCFG;
	u8 bit = BIT(offset % 8);
	int status;

	status = regmap_update_bits(data->regmap, iocfg, bit, 0);
	if (status)
		return status;

	return regmap_update_bits(data->regmap, outreg, bit, val ? bit : 0);
}

static int nct7363_gpio_get_direction(struct gpio_chip *gc,
				      unsigned int offset)
{
	struct nct7363_data *data = gpiochip_get_data(gc);
	u8 iocfg = offset < 8 ? NCT7363_REG_GPIO0_IOCFG
			      : NCT7363_REG_GPIO1_IOCFG;
	unsigned int iobits;
	int status;

	status = regmap_read(data->regmap, iocfg, &iobits);
	return status ? : !!(iobits & BIT(offset % 8));
}

static int nct7363_gpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct nct7363_data *data = gpiochip_get_data(gc);
	u8 inreg = offset < 8 ? NCT7363_REG_GPIO0_INPUT
			      : NCT7363_REG_GPIO1_INPUT;
	unsigned int inbits;
	int status;

	status = regmap_read(data->regmap, inreg, &inbits);
	return status ? : !!(inbits & BIT(offset % 8));
}

static void nct7363_gpio_set_value(struct gpio_chip *gc,
				   unsigned int offset, int val)
{
	struct nct7363_data *data = gpiochip_get_data(gc);
	u8 outreg = offset < 8 ? NCT7363_REG_GPIO0_OUTPUT
			       : NCT7363_REG_GPIO1_OUTPUT;
	u8 bit = BIT(offset % 8);

	regmap_update_bits(data->regmap, outreg, bit, val ? bit : 0);
}

static irqreturn_t nct7363_irq_handler(int irq, void *devid)
{
	struct nct7363_data *data = devid;
	struct gpio_chip *gc = &data->gpio;
	unsigned long pending;
	unsigned int intsts0bits, intsts1bits, level;
	int status;
	u16 intstsbits;

	status = regmap_read(data->regmap,
			     NCT7363_REG_GPIO0_INT_STS,
			     &intsts0bits);
	if (status)
		goto out;

	status = regmap_read(data->regmap,
			     NCT7363_REG_GPIO1_INT_STS,
			     &intsts1bits);
	if (!status) {
		intstsbits = (intsts1bits << 8) | (intsts0bits & 0xFF);
		pending = intstsbits;
		for_each_set_bit(level, &pending, gc->ngpio) {
			int nested_irq = irq_find_mapping(gc->irq.domain,
							  level);

			if (unlikely(nested_irq <= 0)) {
				dev_warn_ratelimited(gc->parent,
					"unmapped interrupt %d\n", level);
				continue;
			}

			handle_nested_irq(nested_irq);
		}
	}

out:
	return IRQ_HANDLED;
}

static int nct7363_init_gpio(struct nct7363_data *data,
			     u32 gpio0_mask, u32 gpio1_mask)
{
	struct gpio_chip *gpio = &data->gpio;
	struct i2c_client *client = data->client;
	struct gpio_irq_chip *girq;
	unsigned int val;
	int ret;

	/* Initialize GPIO0/1 in, out, iocfg registers */
	val = gpio0_mask & 0xFF;
	regmap_write(data->regmap, NCT7363_REG_GPIO0_INPUT, val);
	val = (gpio0_mask >> 8) & 0xFF;
	regmap_write(data->regmap, NCT7363_REG_GPIO0_OUTPUT, val);
	val = (gpio0_mask >> 16) & 0xFF;
	regmap_write(data->regmap, NCT7363_REG_GPIO0_IOCFG, val);
	val = gpio1_mask & 0xFF;
	regmap_write(data->regmap, NCT7363_REG_GPIO1_INPUT, val);
	val = (gpio1_mask >> 8) & 0xFF;
	regmap_write(data->regmap, NCT7363_REG_GPIO1_OUTPUT, val);
	val = (gpio1_mask >> 16) & 0xFF;
	regmap_write(data->regmap, NCT7363_REG_GPIO1_IOCFG, val);

	/* Initialize gpiochip */
	gpio->label = "NCT7363_GPIO";
	gpio->base = -1;
	gpio->parent = &data->client->dev;
	gpio->owner = THIS_MODULE;
	gpio->ngpio = 16;

	gpio->can_sleep = 0;
	gpio->get = nct7363_gpio_get_value;
	gpio->set = nct7363_gpio_set_value;
	gpio->direction_input = nct7363_gpio_direction_input;
	gpio->direction_output = nct7363_gpio_direction_output;
	gpio->get_direction = nct7363_gpio_get_direction;
	gpio->get = nct7363_gpio_get_value;
	gpio->set = nct7363_gpio_set_value;

	/* Initialize irqchip */
	if (!client->irq) {
		dev_err(&data->client->dev, "no client irq\n");
		return 0;
	}

	girq = &data->gpio.irq;
	gpio_irq_chip_set_chip(girq, &nct7363_irq_chip);
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_edge_irq;
	girq->threaded = true;

	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, nct7363_irq_handler,
					IRQF_ONESHOT | IRQF_SHARED,
					"nct7363-gpio", data);
	if (ret) {
		dev_err(&client->dev, "failed to request irq %d\n",
			client->irq);
		return ret;
	}

	return devm_gpiochip_add_data(&data->client->dev, gpio, data);
}

static int nct7363_init_chip(struct nct7363_data *data, u32 fscpxduty)
{
	u32 func_config = 0;
	int i, ret;

	/* Pin Function Configuration */
	for (i = 0; i < NCT7363_PWM_COUNT; i++) {
		if (data->pwm_mask & BIT(i)) {
			func_config |= PWM_SEL(i);
			if (fscpxduty > NCT7363_PWM_DUTY_MAX)
				regmap_write(data->regmap,
					     NCT7363_REG_FSCPXDUTY(i),
					     NCT7363_PWM_DUTY_MAX);
			else
				regmap_write(data->regmap,
					     NCT7363_REG_FSCPXDUTY(i),
					     fscpxduty);
		}
		if (data->fanin_mask & BIT(i))
			func_config |= FANIN_SEL(i);
	}

	for (i = 0; i < 4; i++) {
		ret = regmap_write(data->regmap, NCT7363_REG_FUNC_CFG_BASE(i),
				   VALUE_TO_REG(func_config, i));
		if (ret < 0)
			return ret;
	}

	/* PWM and FANIN Monitoring Enable */
	for (i = 0; i < 2; i++) {
		ret = regmap_write(data->regmap, NCT7363_REG_PWMEN_BASE(i),
				   VALUE_TO_REG(data->pwm_mask, i));
		if (ret < 0)
			return ret;

		ret = regmap_write(data->regmap, NCT7363_REG_FANINEN_BASE(i),
				   VALUE_TO_REG(data->fanin_mask, i));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int nct7363_present_pwm_fanin(struct device *dev,
				     struct device_node *child,
				     struct nct7363_data *data)
{
	u8 fanin_ch[NCT7363_PWM_COUNT];
	struct of_phandle_args args;
	int ret, fanin_cnt;
	u8 ch, index;

	ret = of_parse_phandle_with_args(child, "pwms", "#pwm-cells",
					 0, &args);
	if (ret)
		return ret;

	if (args.args[0] >= NCT7363_PWM_COUNT)
		return -EINVAL;
	else
		data->pwm_mask |= BIT(args.args[0]);

	fanin_cnt = of_property_count_u8_elems(child, "tach-ch");
	if (fanin_cnt < 1 || fanin_cnt > NCT7363_PWM_COUNT)
		return -EINVAL;

	ret = of_property_read_u8_array(child, "tach-ch", fanin_ch, fanin_cnt);
	if (ret)
		return ret;

	for (ch = 0; ch < fanin_cnt; ch++) {
		index = fanin_ch[ch];
		if (index >= NCT7363_PWM_COUNT)
			return -EINVAL;
		else
			data->fanin_mask |= BIT(index);
	}

	return 0;
}

static const struct regmap_config nct7363_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int nct7363_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *child;
	struct nct7363_data *data;
	struct device *hwmon_dev;
	u32 gpio0_mask, gpio1_mask;
	u32 fscpxduty;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	i2c_set_clientdata(client, data);

	if (of_property_read_u32(dev->of_node,
				 "nuvoton,gpio0-mask", &gpio0_mask))
		gpio0_mask = 0x00FF00FF;
	if (of_property_read_u32(dev->of_node,
				 "nuvoton,gpio1-mask", &gpio1_mask))
		gpio1_mask = 0x00FF00FF;
	if (of_property_read_u32(dev->of_node,
				 "nuvoton,fscpxduty", &fscpxduty))
		fscpxduty = 0x7F;

	data->regmap = devm_regmap_init_i2c(client, &nct7363_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	mutex_init(&data->lock);

	for_each_child_of_node(dev->of_node, child) {
		ret = nct7363_present_pwm_fanin(dev, child, data);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	/* Initialize the chip */
	ret = nct7363_init_chip(data, fscpxduty);
	if (ret)
		return ret;

	/* Initialize the gpiochip and irqchip */
	ret = nct7363_init_gpio(data, gpio0_mask, gpio1_mask);
	if (ret)
		return ret;

	hwmon_dev =
		devm_hwmon_device_register_with_info(dev, client->name, data,
						     &nct7363_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct i2c_driver nct7363_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "nct7363",
		.of_match_table = nct7363_of_match,
	},
	.probe = nct7363_probe,
	.id_table = nct7363_id,
};

module_i2c_driver(nct7363_driver);

MODULE_AUTHOR("CW Ho <cwho@nuvoton.com>");
MODULE_AUTHOR("Ban Feng <kcfeng0@nuvoton.com>");
MODULE_DESCRIPTION("NCT7363 Hardware Monitoring Driver");
MODULE_LICENSE("GPL");
