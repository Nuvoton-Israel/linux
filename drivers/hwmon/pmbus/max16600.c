#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include "pmbus.h"

static int max16600_identify(struct i2c_client *client,
			     struct pmbus_driver_info *info)
{
	if (pmbus_check_byte_register(client, 0, PMBUS_VOUT_MODE)) {
		u8 vout_mode;
		int ret;

		/* Read the register with VOUT scaling value.*/
		ret = pmbus_read_byte_data(client, 0, PMBUS_VOUT_MODE);
		if (ret < 0)
			return ret;

		vout_mode = ret & GENMASK(4, 0);

		switch (vout_mode) {
		case 2:
			info->vrm_version = vr13;
			break;
		default:
			return -ENODEV;
		}
	}

	return 0;
}

static struct pmbus_driver_info max16600_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = vid,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_IIN |  PMBUS_HAVE_PIN |
	    PMBUS_HAVE_IOUT | PMBUS_HAVE_POUT | PMBUS_HAVE_VOUT |
		PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_VOUT |
		PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP,
    .identify = max16600_identify,
};

static int max16600_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	return pmbus_do_probe(client, id, &max16600_info);
}

static const struct i2c_device_id max16600_id[] = {
	{"max16600", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max16600_id);

static const struct of_device_id max16600_of_match[] = {
	{.compatible = "max16600"},
	{}
};
/* This is the driver that will be inserted */
static struct i2c_driver max16600_driver = {
	.driver = {
		   .name = "max16600",
		   .of_match_table = of_match_ptr(max16600_of_match),
	   },
	.probe = max16600_probe,
	.remove = pmbus_do_remove,
	.id_table = max16600_id,
};

module_i2c_driver(max16600_driver);

MODULE_AUTHOR("Duke Du <Duke.Du@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for MAX16600");
MODULE_LICENSE("GPL");
