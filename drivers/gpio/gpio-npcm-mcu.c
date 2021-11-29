// SPDX-License-Identifier: GPL-2.0
/*
 * Description   : NPCM7XX MCU EXPANDER Driver
 *
 * Copyright (C) 2021 Nuvoton Corporation
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_gpio.h>

#define IO_8I		0xAAAA	/* I7 I6 I5 I4 I3 I2 I1 I0 */

#define INT_NO_MASK	0x1	/* Has interrupts, no mask */
#define INT_MERGED_MASK 0x3	/* Has interrupts, merged mask */
#define INT_CAPS(x)	(((unsigned int)(x)) << 16)

struct npcm_mcu_gpi_platform {
	unsigned    gpio_base;  /* number of the first GPIO */
	int			irq_base;  /* interrupt base */
	void 		*context;       /* param to setup/teardown */
	int 		(*setup)(struct i2c_client *client,
					unsigned gpio, unsigned ngpio,
					void *context);
	int 		(*teardown)(struct i2c_client *client,
					unsigned gpio, unsigned ngpio,
					void *context);
};

enum {
	npcm750_mcu_gpi,
};

static unsigned int npcm_mcu_gpi_features[] = {
	[npcm750_mcu_gpi] = IO_8I | INT_CAPS(INT_MERGED_MASK),
};

static const struct i2c_device_id npcm_mcu_gpi_id[] = {
	{ "npcm_i2c", npcm750_mcu_gpi },
	{ },
};
MODULE_DEVICE_TABLE(i2c, npcm_mcu_gpi_id);

static const struct of_device_id npcm_mcu_gpi_of_table[] = {
	{ .compatible = "nuvoton,npcm7xx_mcu_gpi" },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm_mcu_gpi_of_table);

struct attiny_chip {
	struct gpio_chip gpio_chip;
	struct i2c_client *client;	/* "main" client */
	unsigned int	dir_input;
	struct mutex	lock;

	struct mutex	irq_lock;
	u8			irq_mask;
	u8			irq_mask_cur;
	u8			irq_trig_raise;
	u8			irq_trig_fall;
	u8			irq_features;
	u8			irq_pin;
};

static int npcm_mcu_gpi_get_value(struct gpio_chip *gc, unsigned off)
{
	struct attiny_chip *chip = gpiochip_get_data(gc);
	struct i2c_client *client;
	u8 reg_val;

	client = chip->client;
	reg_val = i2c_smbus_read_byte(client);
	if (reg_val < 0)
		return reg_val;

	return !!(reg_val & (1 << (off & 0x7)));
}
static int npcm_mcu_gpi_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct attiny_chip *chip = gpiochip_get_data(gc);
	unsigned int mask = 1 << off;

	if ((mask & chip->dir_input) == 0) {
		dev_dbg(&chip->client->dev, "%s port %d is output only\n",
			chip->client->name, off);
		return -EACCES;
	}

	return 0;
}
static int npcm_mcu_gpi_writeb(struct attiny_chip *chip, u8 val)
{
	struct i2c_client *client;
	int ret;

	ret = i2c_smbus_write_byte(chip->client, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing\n");
		return ret;
	}

	return 0;
}
static int npcm_mcu_gpi_readw(struct attiny_chip *chip, u16 *val)
{
	int ret;

	ret = i2c_master_recv(chip->client, (char *)val, 2);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading\n");
		return ret;
	}

	*val = le16_to_cpu(*val);
	return 0;
}
static void npcm_mcu_gpi_irq_update_mask(struct attiny_chip *chip)
{
	if (chip->irq_mask == chip->irq_mask_cur)
		return;

	chip->irq_mask = chip->irq_mask_cur;

	if (chip->irq_features == INT_NO_MASK)
		return;

	mutex_lock(&chip->lock);

	if (chip->irq_features == INT_MERGED_MASK)
		npcm_mcu_gpi_writeb(chip, chip->irq_mask);

	mutex_unlock(&chip->lock);
}
static void npcm_mcu_gpi_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct attiny_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
	chip->irq_mask_cur = chip->irq_mask;
}

static void npcm_mcu_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct attiny_chip *chip = gpiochip_get_data(gc);
	u16 new_irqs;
	u16 level;

	npcm_mcu_gpi_irq_update_mask(chip);

	new_irqs = chip->irq_trig_fall | chip->irq_trig_raise;

	while (new_irqs) {
		level = __ffs(new_irqs);
		npcm_mcu_gpi_direction_input(&chip->gpio_chip, level);
		new_irqs &= ~(1 << level);
	}

	mutex_unlock(&chip->irq_lock);
}
static u8 npcm_mcu_gpi_irq_pending(struct attiny_chip *chip)
{
	u8 cur_stat;
	u8 old_stat;
	u8 trigger;
	u8 pending;
	u16 status;
	int ret;

	ret = npcm_mcu_gpi_readw(chip, &status);
	if (ret)
		return 0;

	trigger = status >> 8;
	trigger &= chip->irq_mask;

	if (!trigger)
		return 0;

	cur_stat = status & 0xFF;
	cur_stat &= chip->irq_mask;

	old_stat = cur_stat ^ trigger;

	pending = (old_stat & chip->irq_trig_fall) |
		  (cur_stat & chip->irq_trig_raise);
	pending &= trigger;

	return pending;
}
static irqreturn_t npcm_mcu_gpi_irq_handler(int irq, void *devid)
{
	struct attiny_chip *chip = devid;
	u8 pending;
	u8 level;

	pending = npcm_mcu_gpi_irq_pending(chip);

	if (!pending)
		return IRQ_HANDLED;

	do {
		level = __ffs(pending);
		handle_nested_irq(irq_find_mapping(chip->gpio_chip.irq.domain,
						   level));
		pending &= ~(1 << level);
	} while (pending);

	return IRQ_HANDLED;
}
static void npcm_mcu_gpi_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct attiny_chip *chip = gpiochip_get_data(gc);
	chip->irq_mask_cur &= ~(1 << d->hwirq);
}

static void npcm_mcu_gpi_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct attiny_chip *chip = gpiochip_get_data(gc);
	chip->irq_mask_cur |= 1 << d->hwirq;
}

static int npcm_mcu_gpi_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct attiny_chip *chip = gpiochip_get_data(gc);
	u16 off = d->hwirq;
	u16 mask = 1 << off;

	if ((mask & chip->dir_input) == 0) {
		dev_dbg(&chip->client->dev, "%s port %d is output only\n",
			chip->client->name, off);
		return -EACCES;
	}

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall |= mask;
	else
		chip->irq_trig_fall &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise |= mask;
	else
		chip->irq_trig_raise &= ~mask;

	return 0;
}
static struct irq_chip attiny_irq_chip = {
	.name			= "npcm7xx_mcu_gpi",
	.irq_mask		= npcm_mcu_gpi_irq_mask,
	.irq_unmask		= npcm_mcu_gpi_irq_unmask,
	.irq_bus_lock		= npcm_mcu_gpi_irq_bus_lock,
	.irq_bus_sync_unlock	= npcm_mcu_irq_bus_sync_unlock,
	.irq_set_type		= npcm_mcu_gpi_irq_set_type,
};
static int npcm_mcu_irq_setup(struct attiny_chip *chip,
			     const struct i2c_device_id *id)
{
	struct i2c_client *client = chip->client;
	struct npcm_mcu_gpi_platform *pdata = dev_get_platdata(&client->dev);
	int has_irq = npcm_mcu_gpi_features[id->driver_data] >> 16;
	int irq_base = 0;
	int ret;
	struct gpio_irq_chip *girq;

	if (pdata)
		irq_base = pdata->irq_base;

	chip->irq_features = has_irq;
	client->irq = gpio_to_irq(chip->irq_pin);
	mutex_init(&chip->irq_lock);

	ret = devm_request_threaded_irq(&client->dev, client->irq,
			NULL, npcm_mcu_gpi_irq_handler, IRQF_ONESHOT |
			IRQF_TRIGGER_LOW | IRQF_SHARED,
			dev_name(&client->dev), chip);
	if (ret) {
		dev_err(&client->dev, "failed to request irq %d\n",
			client->irq);
		return ret;
	}

	girq = &chip->gpio_chip.irq;
	girq->chip = &attiny_irq_chip;
	/* This will let us handle the parent IRQ in the driver */
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;
	girq->first = irq_base; /* FIXME: get rid of this */

	return 0;
}
static int npcm_mcu_gpi_setup(struct attiny_chip *chip,
					const struct i2c_device_id *id,
					unsigned gpio_start)
{
	struct gpio_chip *gc = &chip->gpio_chip;
	int i, port = 0;
	unsigned int id_data = npcm_mcu_gpi_features[id->driver_data];

	for (i = 0; i < 16; i++, id_data >>= 2) {
		unsigned int mask = 1 << port;

		if (id_data & 0x3)
			chip->dir_input |= mask;

		port++;
	}

	gc->direction_input = npcm_mcu_gpi_direction_input;
	gc->get = npcm_mcu_gpi_get_value;
	gc->can_sleep = true;

	gc->base = gpio_start;
	gc->ngpio = port;
	gc->label = chip->client->name;
	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;

	return port;
}
static int npcm_mcu_gpi_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct npcm_mcu_gpi_platform *pdata;
	struct device_node *node;
	struct attiny_chip *chip;
	u8 addr;
	int ret, nr_port;

	pdata = dev_get_platdata(&client->dev);
	node = client->dev.of_node;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->gpio_base = -1;

	if (!pdata) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;
	chip->client = client;
	chip->irq_pin = of_get_named_gpio(node, "interrupts", 0);

	nr_port = npcm_mcu_gpi_setup(chip, id, pdata->gpio_base);
	chip->gpio_chip.parent = &client->dev;

	mutex_init(&chip->lock);

	addr = i2c_smbus_read_byte(client);
	if (addr < 0) {
		dev_err(&client->dev, "failed reading\n");
		return addr;
	}

	ret = npcm_mcu_irq_setup(chip, id);
	if (ret)
		return ret;

	ret = devm_gpiochip_add_data(&client->dev, &chip->gpio_chip, chip);
	if (ret)
		return ret;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	return 0;
}

static int npcm_mcu_gpi_remove(struct i2c_client *client)
{
	struct npcm_mcu_gpi_platform *pdata = dev_get_platdata(&client->dev);
	struct attiny_chip *chip = i2c_get_clientdata(client);

	if (pdata && pdata->teardown) {
		int ret;

		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	return 0;
}

static struct i2c_driver npcm_mcu_gpi_driver = {
	.driver = {
		.name		= "npcm_mcu_gpi",
		.of_match_table	= of_match_ptr(npcm_mcu_gpi_of_table),
	},
	.probe		= npcm_mcu_gpi_probe,
	.remove		= npcm_mcu_gpi_remove,
	.id_table	= npcm_mcu_gpi_id,
};

static int __init npcm_mcu_gpi_init(void)
{
	return i2c_add_driver(&npcm_mcu_gpi_driver);
}
subsys_initcall(npcm_mcu_gpi_init);

static void __exit npcm_mcu_gpi_exit(void)
{
	i2c_del_driver(&npcm_mcu_gpi_driver);
}
module_exit(npcm_mcu_gpi_exit);

MODULE_AUTHOR("Jim Liu <JJLIU0@nuvoton.com>");
MODULE_DESCRIPTION("GPI expander driver for nuvoton mcu");
MODULE_LICENSE("GPL");
