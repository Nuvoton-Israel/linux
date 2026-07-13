// SPDX-License-Identifier: GPL-2.0+
/*
 * obmf-gpio.c - OBMF-ICP GPIO Optimised Channel (Type 02h, v0.9)
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * Implements the GPIO Optimised Channel using command-based protocol
 * with Index/Data pairs per OBMF-ICP v0.9 spec.
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/of.h>
#include <linux/version.h>
#if __has_include(<linux/unaligned.h>)
#include <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#include "obmf.h"

#define OBMF_GPIO_MAX_LINES	4096

struct obmf_gpio_data {
	struct gpio_chip	gc;
	struct obmf_channel	*ch;
	char			label[24];
	const char		**gpio_names;
	struct mutex		irq_lock;
	DECLARE_BITMAP(irq_enabled, OBMF_GPIO_MAX_LINES);
	DECLARE_BITMAP(irq_mask_buf, OBMF_GPIO_MAX_LINES);
	DECLARE_BITMAP(dir_out, OBMF_GPIO_MAX_LINES);
	DECLARE_BITMAP(output_val, OBMF_GPIO_MAX_LINES);
	u8			irq_type[OBMF_GPIO_MAX_LINES];
};

/* ------------------------------------------------------------------ */
/* GPIO optimised channel transfer helper                              */
/* ------------------------------------------------------------------ */

static int obmf_gpio_xfer(struct obmf_channel *ch, u8 cmd,
			  const u16 *pairs_in, int num_pairs_in,
			  u16 *pairs_out, int max_pairs_out)
{
	struct obmf_device *odev = ch->odev;
	u8 req[1 + 256 * 2];
	u8 resp[1 + 256 * 2];
	int req_len, rv, i;
	int resp_pairs;

	req[0] = cmd;
	req_len = 1;

	for (i = 0; i < num_pairs_in && req_len + 2 <= (int)sizeof(req); i++) {
		put_unaligned_le16(pairs_in[i], req + req_len);
		req_len += 2;
	}

	mutex_lock(&ch->lock);
	rv = obmf_send_request(odev, ch, OBMF_TYPE_GPIO,
			       req, req_len, resp, sizeof(resp),
			       OBMF_DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ch->lock);

	if (rv < 0)
		return rv;

	/* Response: Command(1B) + Index/Data pairs */
	if (rv < 1)
		return -EIO;

	resp_pairs = (rv - 1) / 2;
	if (pairs_out) {
		int copy = min(resp_pairs, max_pairs_out);

		for (i = 0; i < copy; i++)
			pairs_out[i] = get_unaligned_le16(resp + 1 + i * 2);
		return copy;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* gpio_chip callbacks                                                 */
/* ------------------------------------------------------------------ */

static int obmf_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);
	u16 pair_in, pair_out;
	int rv;

	pair_in = OBMF_GPIO_PACK(offset, 0);
	rv = obmf_gpio_xfer(gd->ch, OBMF_GPIO_CMD_GET_VALUES,
			    &pair_in, 1, &pair_out, 1);
	if (rv < 1)
		return rv < 0 ? rv : -EIO;

	/* Data nibble: 0=HIGH, 1=LOW */
	return OBMF_GPIO_UNPACK_DATA(pair_out) == OBMF_GPIO_VAL_HIGH ? 1 : 0;
}

static void obmf_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);
	u16 pair_in;

	if (value)
		set_bit(offset, gd->output_val);
	else
		clear_bit(offset, gd->output_val);

	pair_in = OBMF_GPIO_PACK(offset,
				 value ? OBMF_GPIO_VAL_HIGH : OBMF_GPIO_VAL_LOW);
	obmf_gpio_xfer(gd->ch, OBMF_GPIO_CMD_SET_VALUES,
		       &pair_in, 1, NULL, 0);
}

static void obmf_gpio_set_multiple(struct gpio_chip *gc,
				   unsigned long *mask, unsigned long *bits)
{
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);
	u16 pairs[256];
	int n = 0;
	unsigned int i;

	for_each_set_bit(i, mask, gc->ngpio) {
		int val = test_bit(i, bits);

		if (val)
			set_bit(i, gd->output_val);
		else
			clear_bit(i, gd->output_val);

		pairs[n++] = OBMF_GPIO_PACK(i,
				val ? OBMF_GPIO_VAL_HIGH : OBMF_GPIO_VAL_LOW);

		if (n == 256) {
			obmf_gpio_xfer(gd->ch, OBMF_GPIO_CMD_SET_VALUES,
				       pairs, n, NULL, 0);
			n = 0;
		}
	}

	if (n > 0)
		obmf_gpio_xfer(gd->ch, OBMF_GPIO_CMD_SET_VALUES,
			       pairs, n, NULL, 0);
}

static int obmf_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);

	return test_bit(offset, gd->dir_out) ? GPIO_LINE_DIRECTION_OUT
					      : GPIO_LINE_DIRECTION_IN;
}

static int obmf_gpio_set_direction(struct obmf_gpio_data *gd,
				   unsigned int offset, u8 direction)
{
	struct obmf_channel *ch = gd->ch;
	struct obmf_device *odev = ch->odev;
	struct obmf_channel *ch0 = &odev->channels[0];
	u32 addr;
	int rv;

	/* Write gpio_direction byte in config entry via Channel 0 MMIO */
	addr = ch->config_offset + OBMF_CHCFG_CONFIG_DATA + 2 +
	       (u32)offset * OBMF_GPIO_CONFIG_ENTRY_SIZE +
	       OBMF_GPIO_CFG_DIRECTION;

	rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_WRITE,
				    addr, &direction, 1, NULL, 0);
	if (rv < 0)
		return rv;

	if (direction == OBMF_GPIO_DIR_OUTPUT)
		set_bit(offset, gd->dir_out);
	else
		clear_bit(offset, gd->dir_out);

	return 0;
}

static int obmf_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);

	return (!test_bit(offset, gd->dir_out)) ? 0 : -EINVAL;
}

static int obmf_gpio_direction_output(struct gpio_chip *gc,
				      unsigned int offset, int value)
{
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);

	if (!test_bit(offset, gd->dir_out)) {
		return -EINVAL;
	}

	gc->set(gc, offset, value);
	return 0;
}

/* ------------------------------------------------------------------ */
/* irq_chip callbacks (bus_lock pattern for sleeping USB transport)     */
/* ------------------------------------------------------------------ */

static void obmf_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);

	clear_bit(d->hwirq, gd->irq_mask_buf);
}

static void obmf_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);

	set_bit(d->hwirq, gd->irq_mask_buf);
}

static int obmf_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = d->hwirq;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		gd->irq_type[hwirq] = OBMF_GPIO_IRQ_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		gd->irq_type[hwirq] = OBMF_GPIO_IRQ_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		gd->irq_type[hwirq] = OBMF_GPIO_IRQ_BOTH;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		gd->irq_type[hwirq] = OBMF_GPIO_IRQ_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		gd->irq_type[hwirq] = OBMF_GPIO_IRQ_LEVEL_LOW;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void obmf_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);

	mutex_lock(&gd->irq_lock);
}

static void obmf_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct obmf_gpio_data *gd = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = d->hwirq;
	bool want = test_bit(hwirq, gd->irq_mask_buf);
	bool have = test_bit(hwirq, gd->irq_enabled);
	u8 cfg;
	u16 pair;

	if (want == have) {
		mutex_unlock(&gd->irq_lock);
		return;
	}

	cfg = want ? gd->irq_type[hwirq] : OBMF_GPIO_IRQ_DISABLE;
	pair = OBMF_GPIO_PACK(hwirq, cfg);
	obmf_gpio_xfer(gd->ch, OBMF_GPIO_CMD_SET_IRQ_CFG, &pair, 1, NULL, 0);

	if (want)
		set_bit(hwirq, gd->irq_enabled);
	else
		clear_bit(hwirq, gd->irq_enabled);

	mutex_unlock(&gd->irq_lock);
}

static const struct irq_chip obmf_gpio_irq_chip = {
	.name			= "obmf-gpio",
	.irq_mask		= obmf_gpio_irq_mask,
	.irq_unmask		= obmf_gpio_irq_unmask,
	.irq_set_type		= obmf_gpio_irq_set_type,
	.irq_bus_lock		= obmf_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= obmf_gpio_irq_bus_sync_unlock,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

/* ------------------------------------------------------------------ */
/* Device-initiated request handler                                    */
/* ------------------------------------------------------------------ */

void obmf_gpio_handle_dev_request(struct obmf_channel *ch,
				  const u8 *data, int len)
{
	struct obmf_device *odev = ch->odev;
	struct obmf_gpio_data *gd = ch->priv;
	u8 cmd;
	int i, num_pairs;

	if (len < 1) {
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_GPIO,
				   OBMF_STATUS_INVALID_CMD, NULL, 0);
		return;
	}

	cmd = data[0];
	num_pairs = (len - 1) / 2;

	switch (cmd) {
	case OBMF_GPIO_CMD_GET_VALUES: {
		u8 resp[1 + 256 * 2];
		int resp_len = 1;

		resp[0] = cmd;

		if (!gd) {
			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_GPIO,
					   OBMF_STATUS_INVALID_CMD,
					   resp, 1);
			return;
		}

		for (i = 0; i < num_pairs; i++) {
			u16 pair = get_unaligned_le16(data + 1 + i * 2);
			u16 idx = OBMF_GPIO_UNPACK_IDX(pair);
			u8 val;

			if (idx >= gd->gc.ngpio) {
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_GPIO,
						   OBMF_STATUS_GPIO_IDX_NOT_SUPPORTED,
						   resp, 1);
				return;
			}

			if (!test_bit(idx, gd->dir_out)) {
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_GPIO,
						   OBMF_STATUS_GPIO_INVALID_OP,
						   resp, 1);
				return;
			}

			val = test_bit(idx, gd->output_val) ?
			      OBMF_GPIO_VAL_HIGH : OBMF_GPIO_VAL_LOW;
			if (resp_len + 2 <= (int)sizeof(resp)) {
				put_unaligned_le16(OBMF_GPIO_PACK(idx, val),
						   resp + resp_len);
				resp_len += 2;
			}
		}

		obmf_send_response(odev, ch->channel_id, OBMF_TYPE_GPIO,
				   OBMF_STATUS_SUCCESS, resp, resp_len);
		return;
	}

	case OBMF_GPIO_CMD_SET_VALUES: {
		u8 resp_cmd = cmd;

		if (!gd) {
			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_GPIO,
					   OBMF_STATUS_INVALID_CMD,
					   &resp_cmd, 1);
			return;
		}

		for (i = 0; i < num_pairs; i++) {
			u16 pair = get_unaligned_le16(data + 1 + i * 2);
			u16 idx = OBMF_GPIO_UNPACK_IDX(pair);

			if (idx >= gd->gc.ngpio) {
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_GPIO,
						   OBMF_STATUS_GPIO_IDX_NOT_SUPPORTED,
						   &resp_cmd, 1);
				return;
			}

			if (!test_bit(idx, gd->dir_out) && test_bit(idx, gd->irq_enabled))
				handle_nested_irq(
					irq_find_mapping(
						gd->gc.irq.domain,
						idx));
		}

		obmf_send_response(odev, ch->channel_id, OBMF_TYPE_GPIO,
				   OBMF_STATUS_SUCCESS, &resp_cmd, 1);
		return;
	}

	case OBMF_GPIO_CMD_GET_IRQ_CFG: {
		u8 resp[1 + 256 * 2];
		int resp_len = 1;

		resp[0] = cmd;

		if (!gd) {
			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_GPIO,
					   OBMF_STATUS_INVALID_CMD,
					   resp, 1);
			return;
		}

		for (i = 0; i < num_pairs; i++) {
			u16 pair = get_unaligned_le16(data + 1 + i * 2);
			u16 idx = OBMF_GPIO_UNPACK_IDX(pair);
			u8 cfg;

			if (idx >= gd->gc.ngpio) {
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_GPIO,
						   OBMF_STATUS_GPIO_IDX_NOT_SUPPORTED,
						   resp, 1);
				return;
			}

			cfg = test_bit(idx, gd->irq_enabled) ?
			      gd->irq_type[idx] : OBMF_GPIO_IRQ_DISABLE;
			if (resp_len + 2 <= (int)sizeof(resp)) {
				put_unaligned_le16(OBMF_GPIO_PACK(idx, cfg),
						   resp + resp_len);
				resp_len += 2;
			}
		}

		obmf_send_response(odev, ch->channel_id, OBMF_TYPE_GPIO,
				   OBMF_STATUS_SUCCESS, resp, resp_len);
		return;
	}

	case OBMF_GPIO_CMD_SET_IRQ_CFG: {
		u8 resp_cmd = cmd;

		if (!gd) {
			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_GPIO,
					   OBMF_STATUS_INVALID_CMD,
					   &resp_cmd, 1);
			return;
		}

		for (i = 0; i < num_pairs; i++) {
			u16 pair = get_unaligned_le16(data + 1 + i * 2);
			u16 idx = OBMF_GPIO_UNPACK_IDX(pair);
			u8 cfg = OBMF_GPIO_UNPACK_DATA(pair);

			if (idx >= gd->gc.ngpio) {
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_GPIO,
						   OBMF_STATUS_GPIO_IDX_NOT_SUPPORTED,
						   &resp_cmd, 1);
				return;
			}

			if (cfg > OBMF_GPIO_IRQ_BOTH) {
				obmf_send_response(odev, ch->channel_id,
						   OBMF_TYPE_GPIO,
						   OBMF_STATUS_INVALID_CMD,
						   &resp_cmd, 1);
				return;
			}

			gd->irq_type[idx] = cfg;
			if (cfg != OBMF_GPIO_IRQ_DISABLE)
				set_bit(idx, gd->irq_enabled);
			else
				clear_bit(idx, gd->irq_enabled);
		}

		obmf_send_response(odev, ch->channel_id, OBMF_TYPE_GPIO,
				   OBMF_STATUS_SUCCESS, &resp_cmd, 1);
		return;
	}

	case OBMF_GPIO_CMD_IRQ_NOTIFY:
	case OBMF_GPIO_CMD_IRQ_NOTIFY_DG:
		if (gd) {
			for (i = 0; i < num_pairs; i++) {
				u16 pair = get_unaligned_le16(data + 1 + i * 2);
				u16 idx = OBMF_GPIO_UNPACK_IDX(pair);

				if (idx < gd->gc.ngpio &&
				    test_bit(idx, gd->irq_enabled))
					handle_nested_irq(
						irq_find_mapping(
							gd->gc.irq.domain,
							idx));
			}
		}

		if (cmd == OBMF_GPIO_CMD_IRQ_NOTIFY) {
			u8 resp_cmd = OBMF_GPIO_CMD_IRQ_NOTIFY;

			obmf_send_response(odev, ch->channel_id,
					   OBMF_TYPE_GPIO,
					   OBMF_STATUS_SUCCESS,
					   &resp_cmd, 1);
		}
		break;

	default:
		obmf_send_response(odev, ch->channel_id,
				   OBMF_TYPE_GPIO,
				   OBMF_STATUS_INVALID_CMD, &cmd, 1);
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Registration                                                        */
/* ------------------------------------------------------------------ */

/**
 * obmf_gpio_populate_names_from_of - read gpio-line-names for a channel from DTS
 * @gd:    gpio data struct to populate
 * @ch:    OBMF channel
 * @ngpio: number of GPIO lines
 *
 * Locates the USB device's OF node via obmf_find_udev_of_node(), which walks
 * the physical port-chain against the DT tree and therefore works for both
 * flat topologies (host directly to device) and multi-level hub topologies
 * (host -- hub -- device) without any special DT properties.
 *
 * Once the device node is found, locates a child node whose "reg" cell equals
 * @ch->channel_id and reads its "gpio-line-names" property into
 * @gd->gpio_names.
 *
 * Returns true if names were populated from DTS; the caller should fall back
 * to reading names from SMC config data when this returns false.
 */
static bool obmf_gpio_populate_names_from_of(struct obmf_gpio_data *gd,
					     struct obmf_channel *ch,
					     int ngpio)
{
	struct usb_device *udev = ch->odev->udev;
	struct device_node *udev_np, *ch_np = NULL, *tmp;
	int i, count;
	bool found = false;

	/*
	 * Obtain the USB device's OF node.  Prefer udev->dev.of_node when the
	 * USB core has already matched it; otherwise walk the port-chain down
	 * the DT to support both flat and hub topologies transparently.
	 */
	udev_np = of_node_get(udev->dev.of_node);
	if (!udev_np)
		udev_np = obmf_find_udev_of_node(udev);
	if (!udev_np)
		return false;

	/* Find the child node whose "reg" cell matches this channel's ID */
	for_each_child_of_node(udev_np, tmp) {
		u32 reg;

		if (!of_property_read_u32(tmp, "reg", &reg) &&
		    reg == ch->channel_id) {
			ch_np = tmp;
			break;
		}
	}
	of_node_put(udev_np);

	if (!ch_np)
		return false;

	count = of_property_count_strings(ch_np, "gpio-line-names");
	if (count > 0) {
		gd->gpio_names = kcalloc(ngpio, sizeof(char *), GFP_KERNEL);
		if (gd->gpio_names) {
			int copy = min(count, ngpio);

			if (copy != ngpio)
				pr_warn("count (%d) does not match ngpio (%d) for channel %u\n",
						count, ngpio, ch->channel_id);

			for (i = 0; i < copy; i++) {
				const char *name;

				if (!of_property_read_string_index(ch_np,
								   "gpio-line-names",
								   i, &name) &&
				    name[0])
					gd->gpio_names[i] = kstrdup(name,
								    GFP_KERNEL);
			}
			gd->gc.names = (const char *const *)gd->gpio_names;
			found = true;
		}
	}

	of_node_put(ch_np);
	return found;
}

int obmf_gpio_register(struct obmf_device *odev, struct obmf_channel *ch)
{
	struct obmf_gpio_data *gd;
	int ngpio;
	int rv;

	/* Read gpio_count from channel config data via CH0 MMIO */
	if (ch->config_offset && ch->config_size >= 2) {
		struct obmf_channel *ch0 = &odev->channels[0];
		u8 buf[2];

		rv = obmf_send_mmio_request(odev, ch0, OBMF_TRANS_SHORT_READ,
					    ch->config_offset + OBMF_CHCFG_CONFIG_DATA,
					    NULL, 0, buf, 2);
		if (rv < 0) {
			return rv;
		}

		ch->gpio_count = get_unaligned_le16(buf);
	}

	gd = kzalloc(sizeof(*gd), GFP_KERNEL);
	if (!gd)
		return -ENOMEM;

	gd->ch = ch;
	mutex_init(&gd->irq_lock);

	ngpio = ch->gpio_count ? ch->gpio_count : 32;
	if (ngpio > OBMF_GPIO_MAX_LINES)
		ngpio = OBMF_GPIO_MAX_LINES;

	snprintf(gd->label, sizeof(gd->label), "obmf%d-gpio-ch%u",
		 odev->device_index, ch->channel_id);
	gd->gc.label            = gd->label;
	gd->gc.parent           = &odev->intf->dev;
	gd->gc.owner            = THIS_MODULE;
	gd->gc.get              = obmf_gpio_get;
	gd->gc.set              = obmf_gpio_set;
	gd->gc.set_multiple     = obmf_gpio_set_multiple;
	gd->gc.get_direction    = obmf_gpio_get_direction;
	gd->gc.direction_input  = obmf_gpio_direction_input;
	gd->gc.direction_output = obmf_gpio_direction_output;
	gd->gc.base             = -1;
	gd->gc.ngpio            = ngpio;
	gd->gc.can_sleep        = true;

	/* Read GPIO config entries to populate names and direction bitmap */
	if (ch->config_offset && ch->gpio_count > 0) {
		struct obmf_channel *ch0 = &odev->channels[0];
		int j;

		/*
		 * GPIO names: prefer DTS (gpio-line-names in channel child node)
		 * so that two SMC devices connected simultaneously don't collide
		 * on the names provided by each SMC's config data.  Fall back to
		 * reading the names from the SMC config data only when the DTS
		 * does not declare them.
		 */
		if (!obmf_gpio_populate_names_from_of(gd, ch, ngpio)) {
			gd->gpio_names = kcalloc(ngpio, sizeof(char *), GFP_KERNEL);
			if (gd->gpio_names) {
				for (j = 0; j < ngpio; j++) {
					u32 entry_addr = ch->config_offset +
							 OBMF_CHCFG_CONFIG_DATA + 2 +
							 (u32)j * OBMF_GPIO_CONFIG_ENTRY_SIZE;
					char namebuf[32];

					rv = obmf_send_mmio_request(odev, ch0,
								    OBMF_TRANS_SHORT_READ,
								    entry_addr + OBMF_GPIO_CFG_NAME,
								    NULL, 0, namebuf, 32);
					if (rv >= 0) {
						namebuf[31] = '\0';
						if (namebuf[0])
							gd->gpio_names[j] = kstrdup(namebuf,
										GFP_KERNEL);
					}
				}
				gd->gc.names = (const char *const *)gd->gpio_names;
			}
		}

		for (j = 0; j < ngpio; j++) {
			u32 entry_addr = ch->config_offset +
					 OBMF_CHCFG_CONFIG_DATA + 2 +
					 (u32)j * OBMF_GPIO_CONFIG_ENTRY_SIZE;
			u8 direction;

			rv = obmf_send_mmio_request(odev, ch0,
						    OBMF_TRANS_SHORT_READ,
						    entry_addr + OBMF_GPIO_CFG_DIRECTION,
						    NULL, 0, &direction, 1);
			if (rv < 0) {
				dev_dbg(&odev->intf->dev,
					"ch%u: failed to read GPIO%d direction\n",
					ch->channel_id, j);
				continue;
			}
			if (direction == OBMF_GPIO_DIR_OUTPUT) {
				u8 default_out = 0;

				set_bit(j, gd->dir_out);
				rv = obmf_send_mmio_request(odev, ch0,
							    OBMF_TRANS_SHORT_READ,
							    entry_addr + OBMF_GPIO_CFG_DEFAULT_OUT,
							    NULL, 0, &default_out, 1);
				if (rv >= 0 && default_out)
					set_bit(j, gd->output_val);
			}
		}
	}

	/* Set up IRQ chip for GPIO events from device */
	{
		struct gpio_irq_chip *girq = &gd->gc.irq;

		gpio_irq_chip_set_chip(girq, &obmf_gpio_irq_chip);
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_simple_irq;
		girq->threaded = true;
	}

	rv = gpiochip_add_data(&gd->gc, gd);
	if (rv) {
		kfree(gd);
		return rv;
	}

	ch->priv = gd;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
	/* 6.12+: use public helper APIs and point to gpiochipX device */
	{
		struct gpio_device *gdev;

		gdev = gpio_device_find_by_label(gd->label);
		if (gdev) {
			ch->sysfs_dev = gpio_device_to_device(gdev);
			gpio_device_put(gdev);
		}
	}
#else
	/* fallback: avoid touching gpiodev internals */
	ch->sysfs_dev = NULL;
#endif
	dev_info(&odev->intf->dev, "ch%u: registered GPIO chip (%d lines)\n",
		 ch->channel_id, ngpio);
	return 0;
}

void obmf_gpio_unregister(struct obmf_channel *ch)
{
	struct obmf_gpio_data *gd = ch->priv;

	if (gd) {
		int j;

		gpiochip_remove(&gd->gc);
		if (gd->gpio_names) {
			for (j = 0; j < gd->gc.ngpio; j++)
				kfree(gd->gpio_names[j]);
			kfree(gd->gpio_names);
		}
		kfree(gd);
		ch->priv = NULL;
	}
}
