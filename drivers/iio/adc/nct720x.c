// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Nuvoton nct7201 and nct7202 power monitor chips.
 *
 * Copyright (c) 2022 Nuvoton Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>

#define REG_CHIP_ID				0xFD
#define NCT720X_ID				0xD7
#define REG_VENDOR_ID				0xFE
#define NUVOTON_ID				0x50
#define REG_DEVICE_ID				0xFF
#define NCT720X_DEVICE_ID			0x12
#define VIN_MAX					12	/* Counted from 1 */
#define NCT7201_VIN_MAX				8
#define NCT7202_VIN_MAX				12
#define NCT720X_IN_SCALING			4995

#define REG_INTERRUPT_STATUS_1			0x0C
#define REG_INTERRUPT_STATUS_2			0x0D
#define REG_VOLT_LOW_BYTE			0x0F
#define REG_CONFIGURATION			0x10
#define  BIT_CONFIGURATION_START		BIT(0)
#define  BIT_CONFIGURATION_ALERT_MSK		BIT(1)
#define  BIT_CONFIGURATION_CONV_RATE		BIT(2)
#define  BIT_CONFIGURATION_INIT			BIT(7)

#define REG_ADVANCED_CONFIGURATION		0x11
#define  BIT_ADVANCED_CONF_MOD_ALERT		BIT(0)
#define  BIT_ADVANCED_CONF_MOD_STS		BIT(1)
#define  BIT_ADVANCED_CONF_FAULT_QUEUE		BIT(2)
#define  BIT_ADVANCED_CONF_EN_DEEP_SHUTDOWN	BIT(4)
#define  BIT_ADVANCED_CONF_EN_SMB_TIMEOUT	BIT(5)
#define  BIT_ADVANCED_CONF_MOD_RSTIN		BIT(7)

#define REG_CHANNEL_INPUT_MODE			0x12
#define REG_CHANNEL_ENABLE_1			0x13
#define REG_CHANNEL_ENABLE_2			0x14
#define REG_INTERRUPT_MASK_1			0x15
#define REG_INTERRUPT_MASK_2			0x16
#define REG_BUSY_STATUS				0x1E
#define REG_ONE_SHOT				0x1F
#define REG_SMUS_ADDRESS			0xFC

static const u8 REG_VIN[VIN_MAX] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
				     0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};
static const u8 REG_VIN_HIGH_LIMIT[VIN_MAX] = { 0x20, 0x22, 0x24, 0x26, 0x28, 0x2A,
						0x2C, 0x2E, 0x30, 0x32, 0x34, 0x36};
static const u8 REG_VIN_LOW_LIMIT[VIN_MAX] = { 0x21, 0x23, 0x25, 0x27, 0x29, 0x2B,
					       0x2D, 0x2F, 0x31, 0x33, 0x35, 0x37};
static const u8 REG_VIN_HIGH_LIMIT_LSB[VIN_MAX] = { 0x40, 0x42, 0x44, 0x46, 0x48, 0x4A,
						    0x4C, 0x4E, 0x50, 0x52, 0x54, 0x56};
static const u8 REG_VIN_LOW_LIMIT_LSB[VIN_MAX] = { 0x41, 0x43, 0x45, 0x47, 0x49, 0x4B,
						   0x4D, 0x4F, 0x51, 0x53, 0x55, 0x57};
static u8 nct720x_chan_to_index[] = {
	0,	/* Not used */
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};


/* List of supported devices */
enum nct720x_chips {
	nct7201, nct7202
};

struct nct720x_chip_info {
	struct i2c_client	*client;
	enum nct720x_chips	type;
	struct mutex		access_lock;	/* for multi-byte read and write operations */
	int vin_max;				/* number of VIN channels */
	u32 vin_mask;
	bool use_read_byte_vin;
};

static const struct iio_event_spec nct720x_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	},
};

#define NCT720X_VOLTAGE_CHANNEL(chan, addr)				\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = chan,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
		.address = addr,					\
		.event_spec = nct720x_events,				\
		.num_event_specs = ARRAY_SIZE(nct720x_events),		\
	}

#define NCT720X_VOLTAGE_CHANNEL_DIFF(chan1, chan2, addr)		\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = (chan1),					\
		.channel2 = (chan2),					\
		.differential = 1,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
		.address = addr,					\
		.event_spec = nct720x_events,				\
		.num_event_specs = ARRAY_SIZE(nct720x_events),		\
	}

static const struct iio_chan_spec nct720x_channels[] = {
	NCT720X_VOLTAGE_CHANNEL(1, 1),
	NCT720X_VOLTAGE_CHANNEL(2, 2),
	NCT720X_VOLTAGE_CHANNEL(3, 3),
	NCT720X_VOLTAGE_CHANNEL(4, 4),
	NCT720X_VOLTAGE_CHANNEL(5, 5),
	NCT720X_VOLTAGE_CHANNEL(6, 6),
	NCT720X_VOLTAGE_CHANNEL(7, 7),
	NCT720X_VOLTAGE_CHANNEL(8, 8),
	NCT720X_VOLTAGE_CHANNEL(9, 9),
	NCT720X_VOLTAGE_CHANNEL(10, 10),
	NCT720X_VOLTAGE_CHANNEL(11, 11),
	NCT720X_VOLTAGE_CHANNEL(12, 12),
	NCT720X_VOLTAGE_CHANNEL_DIFF(1, 2, 1),
	NCT720X_VOLTAGE_CHANNEL_DIFF(3, 4, 3),
	NCT720X_VOLTAGE_CHANNEL_DIFF(5, 6, 5),
	NCT720X_VOLTAGE_CHANNEL_DIFF(7, 8, 7),
	NCT720X_VOLTAGE_CHANNEL_DIFF(9, 10, 9),
	NCT720X_VOLTAGE_CHANNEL_DIFF(11, 12, 11),
};

/* Read 1-byte register. Returns unsigned byte data or -ERRNO on error. */
static int nct720x_read_reg(struct nct720x_chip_info *chip, u8 reg)
{
	struct i2c_client *client = chip->client;

	return i2c_smbus_read_byte_data(client, reg);
}

/* Read 1-byte register. Returns unsigned word data or -ERRNO on error. */
static int nct720x_read_word_swapped_reg(struct nct720x_chip_info *chip, u8 reg)
{
	struct i2c_client *client = chip->client;

	return i2c_smbus_read_word_swapped(client, reg);
}

/*
 * Read 2-byte register. Returns register in big-endian format or
 * -ERRNO on error.
 */
static int nct720x_read_reg16(struct nct720x_chip_info *chip, u8 reg)
{
	struct i2c_client *client = chip->client;
	int ret, low;

	mutex_lock(&chip->access_lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		low = ret;
		ret = i2c_smbus_read_byte_data(client, reg + 1);
		if (ret >= 0)
			ret = low | (ret << 8);
	}

	mutex_unlock(&chip->access_lock);
	return ret;
}

/* Write 1-byte register. Returns 0 or -ERRNO on error. */
static int nct720x_write_reg(struct nct720x_chip_info *chip, u8 reg, u8 val)
{
	struct i2c_client *client = chip->client;
	int err;

	err = i2c_smbus_write_byte_data(client, reg, val);
	mdelay(10);

	return err;
}

static int nct720x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	int index = nct720x_chan_to_index[chan->address];
	int v1, v2, volt, err;
	struct nct720x_chip_info *chip = iio_priv(indio_dev);

	if (chan->type != IIO_VOLTAGE)
		return -EOPNOTSUPP;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&chip->access_lock);
		if (chip->use_read_byte_vin) {
			/* MNTVIN Low Byte together with MNTVIN High Byte
			   forms the 13-bit count value. If MNTVIN High
			   Byte readout is read successively, the
			   NCT7201/NCT7202 will latch the MNTVIN Low Byte
			   for next read.
			 */
			v1 = nct720x_read_reg(chip, REG_VIN[index]);
			if (v1 < 0) {
				err = v1;
				goto abort;
			}

			v2 = nct720x_read_reg(chip, REG_VOLT_LOW_BYTE);
			if (v2 < 0) {
				err = v2;
				goto abort;
			}
			volt = (v1 << 8) | v2;	/* Convert back to 16-bit value */
		} else {
			/* NCT7201/NCT7202 also supports read word-size data */
			volt = nct720x_read_word_swapped_reg(chip, REG_VIN[index]);
		}

		/* Voltage(V) = 13bitCountValue * 0.0004995 */
		volt = (volt >> 3) * NCT720X_IN_SCALING;
		*val = volt / 10000;
		mutex_unlock(&chip->access_lock);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
abort:
	mutex_unlock(&chip->access_lock);
	return err;
}

static int nct720x_read_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct nct720x_chip_info *chip = iio_priv(indio_dev);
	int v1, v2, volt, err;
	int index = nct720x_chan_to_index[chan->address];

	if (chan->type != IIO_VOLTAGE)
		return -EOPNOTSUPP;

	if (info == IIO_EV_INFO_VALUE) {
		if (dir == IIO_EV_DIR_FALLING) {
			if (chip->use_read_byte_vin) {
				/* Low limit VIN Low Byte together with Low limit VIN High Byte
				   forms the 13-bit count value */
				mutex_lock(&chip->access_lock);
				v1 = nct720x_read_reg(chip, REG_VIN_LOW_LIMIT[index]);
				if (v1 < 0) {
					err = v1;
					goto abort;
				}

				v2 = nct720x_read_reg(chip, REG_VIN_LOW_LIMIT_LSB[index]);
				if (v2 < 0) {
					err = v2;
					goto abort;
				}
				mutex_unlock(&chip->access_lock);
				volt = (v1 << 8) | v2;	/* Convert back to 16-bit value */
			} else {
				/* NCT7201/NCT7202 also supports read word-size data */
				volt = nct720x_read_word_swapped_reg(chip, REG_VIN_LOW_LIMIT[index]);
			}
		} else {
			if (chip->use_read_byte_vin) {
				/* High limit VIN Low Byte together with high limit VIN High Byte
				   forms the 13-bit count value */
				mutex_lock(&chip->access_lock);
				v1 = nct720x_read_reg(chip, REG_VIN_HIGH_LIMIT[index]);
				if (v1 < 0) {
					err = v1;
					goto abort;
				}

				v2 = nct720x_read_reg(chip, REG_VIN_HIGH_LIMIT_LSB[index]);
				if (v2 < 0) {
					err = v2;
					goto abort;
				}
				mutex_unlock(&chip->access_lock);
				volt = (v1 << 8) | v2;	/* Convert back to 16-bit value */
			} else {
				/* NCT7201/NCT7202 also supports read word-size data */
				volt = nct720x_read_word_swapped_reg(chip, REG_VIN_HIGH_LIMIT[index]);
			}
		}
	}
	/* Voltage(V) = 13bitCountValue * 0.0004995 */
	volt = (volt >> 3) * NCT720X_IN_SCALING;
	*val = volt / 10000;

	return IIO_VAL_INT;
abort:
	mutex_unlock(&chip->access_lock);
	return err;
}

static int nct720x_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct nct720x_chip_info *chip = iio_priv(indio_dev);
	int index, err = 0;
	long v1, v2, volt;

	index = nct720x_chan_to_index[chan->address];
	volt = (val * 10000) / NCT720X_IN_SCALING;
	v1 = volt >> 5;
	v2 = (volt & 0x1f) << 3;

	if (chan->type != IIO_VOLTAGE)
		return -EOPNOTSUPP;

	if (info == IIO_EV_INFO_VALUE) {
		if (dir == IIO_EV_DIR_FALLING) {
			mutex_lock(&chip->access_lock);
			err = nct720x_write_reg(chip, REG_VIN_LOW_LIMIT[index], v1);
			if (err < 0) {
				pr_err("Failed to write REG_VIN%d_LOW_LIMIT\n", index + 1);
				goto abort;
			}

			err = nct720x_write_reg(chip, REG_VIN_LOW_LIMIT_LSB[index], v2);
			if (err < 0) {
				pr_err("Failed to write REG_VIN%d_LOW_LIMIT_LSB\n", index + 1);
				goto abort;
			}
		} else {
			mutex_lock(&chip->access_lock);
			err = nct720x_write_reg(chip, REG_VIN_HIGH_LIMIT[index], v1);
			if (err < 0) {
				pr_err("Failed to write REG_VIN%d_HIGH_LIMIT\n", index + 1);
				goto abort;
			}

			err = nct720x_write_reg(chip, REG_VIN_HIGH_LIMIT_LSB[index], v2);
			if (err < 0) {
				pr_err("Failed to write REG_VIN%d_HIGH_LIMIT_LSB\n", index + 1);
				goto abort;
			}
		}
	}
abort:
	mutex_unlock(&chip->access_lock);
	return err;
}

static int nct720x_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct nct720x_chip_info *chip = iio_priv(indio_dev);
	int index = nct720x_chan_to_index[chan->address];

	if (chan->type != IIO_VOLTAGE)
		return -EOPNOTSUPP;

	return !!(chip->vin_mask & BIT(index));
}

static int nct720x_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	int err = 0;
	struct nct720x_chip_info *chip = iio_priv(indio_dev);
	int index = nct720x_chan_to_index[chan->address];
	unsigned int mask;

	mask = BIT(index);

	if (chan->type != IIO_VOLTAGE)
		return -EOPNOTSUPP;

	if (!state && (chip->vin_mask & mask))
		chip->vin_mask &= ~mask;
	else if (state && !(chip->vin_mask & mask))
		chip->vin_mask |= mask;

	mutex_lock(&chip->access_lock);

	err = nct720x_write_reg(chip, REG_CHANNEL_ENABLE_1, chip->vin_mask & 0xff);
	if (err < 0) {
		pr_err("Failed to write REG_CHANNEL_ENABLE_1\n");
		goto abort;
	}

	if (chip->type == nct7202) {
		err = nct720x_write_reg(chip, REG_CHANNEL_ENABLE_2, chip->vin_mask >> 8);
		if (err < 0) {
			pr_err("Failed to write REG_CHANNEL_ENABLE_2\n");
			goto abort;
		}
	}
abort:
	mutex_unlock(&chip->access_lock);
	return err;
}

static int nct720x_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	/* Determine the chip type. */
	if (i2c_smbus_read_byte_data(client, REG_VENDOR_ID) != NUVOTON_ID ||
	    i2c_smbus_read_byte_data(client, REG_CHIP_ID) != NCT720X_ID ||
	    i2c_smbus_read_byte_data(client, REG_DEVICE_ID) != NCT720X_DEVICE_ID)
		return -ENODEV;

	strscpy(info->type, "nct720x", I2C_NAME_SIZE);

	return 0;
}

static const struct iio_info nct720x_info = {
	.read_raw = &nct720x_read_raw,
	.read_event_config = &nct720x_read_event_config,
	.write_event_config = &nct720x_write_event_config,
	.read_event_value = &nct720x_read_event_value,
	.write_event_value = &nct720x_write_event_value,
};

static const struct i2c_device_id nct720x_id[];

static int nct720x_init_chip(struct nct720x_chip_info *chip)
{
	int value = 0;
	int err = 0;

	/* Initial reset */
	err = nct720x_write_reg(chip, REG_CONFIGURATION, BIT_CONFIGURATION_INIT);
	if (err) {
		pr_err("Failed to write REG_CONFIGURATION\n");
		return err;
	}

	/* Enable Channel */
	err = nct720x_write_reg(chip, REG_CHANNEL_ENABLE_1, 0xff);
	if (err) {
		pr_err("Failed to write REG_CHANNEL_ENABLE_1\n");
		return err;
	}

	if (chip->type == nct7202) {
		err = nct720x_write_reg(chip, REG_CHANNEL_ENABLE_2, 0xf);
		if (err) {
			pr_err("Failed to write REG_CHANNEL_ENABLE_2\n");
			return err;
		}
	}

	value = nct720x_read_reg16(chip, REG_CHANNEL_ENABLE_1);
	if (value < 0)
		return value;
	chip->vin_mask = value;

	/* Start monitoring if needed */
	value = nct720x_read_reg(chip, REG_CONFIGURATION);
	if (value < 0) {
		pr_err("Failed to read REG_CONFIGURATION\n");
		return value;
	}

	value |= BIT_CONFIGURATION_START;
	err = nct720x_write_reg(chip, REG_CONFIGURATION, value);
	if (err < 0) {
		pr_err("Failed to write REG_CONFIGURATION\n");
		return err;
	}

	return 0;
}

static int nct720x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct nct720x_chip_info *chip;
	struct iio_dev *indio_dev;
	int ret;
	u32 tmp;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;
	chip = iio_priv(indio_dev);

	if (client->dev.of_node)
		chip->type = (enum nct720x_chips)device_get_match_data(&client->dev);
	else
		chip->type = i2c_match_id(nct720x_id, client)->driver_data;

	chip->vin_max = (chip->type == nct7201) ? NCT7201_VIN_MAX : NCT7202_VIN_MAX;

	ret = of_property_read_u32(client->dev.of_node, "read-vin-data-size", &tmp);
	if (ret < 0) {
		pr_err("read-vin-data-size property not found\n");
		return ret;
	}

	if (tmp == 8) {
		chip->use_read_byte_vin = true;
	} else if (tmp == 16) {
		chip->use_read_byte_vin = false;
	} else {
		pr_err("invalid read-vin-data-size (%d)\n", tmp);
		return -EINVAL;
	}

	mutex_init(&chip->access_lock);

	/* this is only used for device removal purposes */
	i2c_set_clientdata(client, indio_dev);

	chip->client = client;

	ret = nct720x_init_chip(chip);
	if (ret < 0)
		return ret;

	indio_dev->name = id->name;
	indio_dev->channels = nct720x_channels;
	indio_dev->num_channels = ARRAY_SIZE(nct720x_channels);
	indio_dev->info = &nct720x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	iio_device_register(indio_dev);

	return 0;
}

static int nct720x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);

	return 0;
}

static const unsigned short nct720x_address_list[] = {
	0x1d, 0x1e, 0x35, 0x36, I2C_CLIENT_END
};

static const struct i2c_device_id nct720x_id[] = {
	{ "nct7201", nct7201 },
	{ "nct7202", nct7202 },
	{}
};
MODULE_DEVICE_TABLE(i2c, nct720x_id);

static const struct of_device_id nct720x_of_match[] = {
	{
		.compatible = "nuvoton,nct7201",
		.data = (void *)nct7201
	},
	{
		.compatible = "nuvoton,nct7202",
		.data = (void *)nct7202
	},
	{ },
};

MODULE_DEVICE_TABLE(of, nct720x_of_match);

static struct i2c_driver nct720x_driver = {
	.driver = {
		.name	= "nct720x",
		.of_match_table = nct720x_of_match,
	},
	.probe = nct720x_probe,
	.remove = nct720x_remove,
	.id_table	= nct720x_id,
	.detect = nct720x_detect,
	.address_list = nct720x_address_list,
};

module_i2c_driver(nct720x_driver);

MODULE_AUTHOR("Eason Yang <YHYANG2@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton NCT720x voltage monitor driver");
MODULE_LICENSE("GPL");
