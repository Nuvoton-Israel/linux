// SPDX-License-Identifier: GPL-2.0-only
/*
 * An I2C driver for the Nuvoton NCT3015Y RTC
 *
 * based on the other drivers in this same directory.
 *
 */

#include <linux/clk-provider.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/err.h>

#define NCT3015Y_REG_SC		0x00 /* seconds */
#define NCT3015Y_REG_SCA	0x01 /* alarm */
#define NCT3015Y_REG_MN		0x02
#define NCT3015Y_REG_MNA	0x03 /* alarm */
#define NCT3015Y_REG_HR		0x04
#define NCT3015Y_REG_HRA	0x05 /* alarm */
#define NCT3015Y_REG_DW		0x06
#define NCT3015Y_REG_DM		0x07
#define NCT3015Y_REG_MO		0x08
#define NCT3015Y_REG_YR		0x09
#define NCT3015Y_REG_CTRL	0x0A /* timer control */
#define NCT3015Y_REG_ST		0x0B /* status */
#define NCT3015Y_REG_CLKO	0x0C /* clock out */

#define NCT3015Y_BIT_AF		BIT(7)
#define NCT3015Y_BIT_ST		BIT(7)
#define NCT3015Y_BIT_DM		BIT(6)
#define NCT3015Y_BIT_HF		BIT(5)
#define NCT3015Y_BIT_DSM	BIT(4)
#define NCT3015Y_BIT_AIE	BIT(3)
#define NCT3015Y_BIT_OFIE	BIT(2)
#define NCT3015Y_BIT_CIE	BIT(1)
#define NCT3015Y_BIT_TWO	BIT(0)

#define NCT3015Y_REG_CLKO_F_MASK	0x03 /* frequenc mask */
#define NCT3015Y_REG_CLKO_CKE		0x80 /* clock out enabled */

static struct i2c_driver nct3015y_driver;

struct nct3015y {
	struct rtc_device *rtc;
	struct i2c_client *client;
	struct gpio_desc *wakeup_host;
#ifdef CONFIG_COMMON_CLK
	struct clk_hw		clkout_hw;
#endif
	int rtc_bus;
};

static int nct3015y_read_block_data(struct i2c_client *client, unsigned char reg,
				   unsigned char length, unsigned char *buf)
{
	struct i2c_msg msgs[] = {
		{/* setup read ptr */
			.addr = client->addr,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buf
		},
	};

	if ((i2c_transfer(client->adapter, msgs, 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	return 0;
}

static int nct3015y_write_block_data(struct i2c_client *client,
				   unsigned char reg, unsigned char length,
				   unsigned char *buf)
{
	int i, err;

	for (i = 0; i < length; i++) {
		unsigned char data[2] = { reg + i, buf[i] };

		err = i2c_master_send(client, data, sizeof(data));
		if (err != sizeof(data)) {
			dev_err(&client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
			return -EIO;
		}
	}

	return 0;
}

static int nct3015y_set_alarm_mode(struct i2c_client *client, bool on)
{
	unsigned char buf;
	int err;

	dev_dbg(&client->dev, "%s:on:%d\n", __func__, on);
	err = nct3015y_read_block_data(client, NCT3015Y_REG_CTRL, 1, &buf);
	if (err < 0)
		return err;

	if (on)
		buf |= NCT3015Y_BIT_AIE;
	else
		buf &= ~NCT3015Y_BIT_AIE;

	buf |= NCT3015Y_BIT_CIE;
	err = nct3015y_write_block_data(client, NCT3015Y_REG_CTRL, 1, &buf);
	if (err < 0) {
		dev_err(&client->dev, "%s: write NCT3015Y_REG_ST error\n", __func__);
		return -EIO;
	}

	err = nct3015y_read_block_data(client, NCT3015Y_REG_ST, 1, &buf);
	if (err < 0)
		return err;

	buf &= ~(NCT3015Y_BIT_AF);
	err = nct3015y_write_block_data(client, NCT3015Y_REG_ST, 1, &buf);
	if (err < 0) {
		dev_err(&client->dev, "%s: write NCT3015Y_REG_ST error\n", __func__);
		return -EIO;
	}

	return 0;
}

static int nct3015y_get_alarm_mode(struct i2c_client *client, unsigned char *alarm_enable,
				  unsigned char *alarm_flag)
{
	unsigned char buf;
	int err;

	if (alarm_enable) {
		dev_dbg(&client->dev, "%s:NCT3015Y_REG_CTRL\n", __func__);
		err = nct3015y_read_block_data(client, NCT3015Y_REG_CTRL, 1, &buf);
		if (err)
			return err;
		*alarm_enable = buf & NCT3015Y_BIT_AIE;
	}

	if (alarm_flag) {
		dev_dbg(&client->dev, "%s:NCT3015Y_REG_ST\n", __func__);
		err = nct3015y_read_block_data(client, NCT3015Y_REG_ST, 1, &buf);
		if (err)
			return err;
		*alarm_flag = buf & NCT3015Y_BIT_AF;
	}

	dev_dbg(&client->dev, "%s:alarm_enable:%x alarm_flag:%x\n",
		__func__, *alarm_enable, *alarm_flag);

	return 0;
}

static irqreturn_t nct3015y_irq(int irq, void *dev_id)
{
	struct nct3015y *nct3015y = i2c_get_clientdata(dev_id);
	struct i2c_client *client = nct3015y->client;
	int err;
	unsigned char alarm_flag;
	unsigned char alarm_enable;

	dev_dbg(&client->dev, "%s:irq:%d\n", __func__, irq);
	err = nct3015y_get_alarm_mode(nct3015y->client, &alarm_enable, &alarm_flag);
	if (err)
		return IRQ_NONE;

	if (alarm_flag) {
		dev_dbg(&client->dev, "%s:alarm flag:%x\n",
			__func__, alarm_flag);
		rtc_update_irq(nct3015y->rtc, 1, RTC_IRQF | RTC_AF);
		if(nct3015y->rtc_bus == 1)
			nct3015y_set_alarm_mode(nct3015y->client, 1);
		dev_dbg(&client->dev, "%s:IRQ_HANDLED\n", __func__);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/*
 * In the routines that deal directly with the nct3015y hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
static int nct3015y_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct3015y *nct3015y = i2c_get_clientdata(client);
	unsigned char buf[10];
	int err;

	err = nct3015y_read_block_data(client, NCT3015Y_REG_ST, 1, buf);
	if (err)
		return err;

	if (!buf[0]) {
		dev_err(&client->dev,
			" voltage <=1.7, date/time is not reliable.\n");
		return -EINVAL;
	}

	err = nct3015y_read_block_data(client, NCT3015Y_REG_SC, 10, buf);
	if (err)
		return err;

	dev_dbg(&client->dev,
		"%s: raw data is sec=%02x, secA=%02x, min=%02x, minA=%02x, hr=%02x, hrA=%02x "
		"wday=%02x, mday=%02x,  mon=%02x, year=%02x\n",
		__func__,
		buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9]);

	tm->tm_sec = bcd2bin(buf[0] & 0x7F);
	tm->tm_min = bcd2bin(buf[2] & 0x7F);
	tm->tm_hour = bcd2bin(buf[4] & 0x3F); /* rtc hr 0-24 */
	tm->tm_wday = buf[6] & 0x07;
	tm->tm_mday = bcd2bin(buf[7] & 0x3F);
	tm->tm_mon = bcd2bin(buf[8] & 0x1F) -1 ; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(buf[9]) + 100;

	dev_dbg(&client->dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	return 0;
}

static int nct3015y_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct3015y *nct3015y = i2c_get_clientdata(client);
	unsigned char buf[10] = {0};
	int err;

	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);

	err = nct3015y_read_block_data(client, NCT3015Y_REG_CTRL, 1, buf);
	if (err)
		return err;

	if (!(buf[0]& NCT3015Y_BIT_TWO)) {
		dev_err(&client->dev,
			" TWO is not set.\n");
		return -EINVAL;
	}

	/* hours, minutes and seconds */
	buf[NCT3015Y_REG_SC] = bin2bcd(tm->tm_sec);
	buf[NCT3015Y_REG_MN] = bin2bcd(tm->tm_min);
	buf[NCT3015Y_REG_HR] = bin2bcd(tm->tm_hour);
	buf[NCT3015Y_REG_DW] = tm->tm_wday & 0x07;
	buf[NCT3015Y_REG_DM] = bin2bcd(tm->tm_mday);

	/* month, 1 - 12 */
	buf[NCT3015Y_REG_MO] = bin2bcd(tm->tm_mon+1);

	/* year and century */
	buf[NCT3015Y_REG_YR] = bin2bcd(tm->tm_year - 100);

	return nct3015y_write_block_data(client, NCT3015Y_REG_SC,
				10, buf);
}

static int nct3015y_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char buf[5];
	int err;

	err = nct3015y_read_block_data(client, NCT3015Y_REG_SCA, 5, buf);
	if (err)
		return err;

	dev_dbg(&client->dev,
		"%s: raw data is sec=%02x, min=%02x hr=%02x\n",
		__func__, buf[0], buf[2], buf[4]);

	tm->time.tm_sec = bcd2bin(buf[0] & 0x7F);
	tm->time.tm_min = bcd2bin(buf[2] & 0x7F);
	tm->time.tm_hour = bcd2bin(buf[4] & 0x3F);

	err = nct3015y_get_alarm_mode(client, &tm->enabled, &tm->pending);
	if (err < 0)
		return err;

	dev_dbg(&client->dev, "%s: tm is sec=%d mins=%d, hours=%d,"
		" enabled=%d, pending=%d\n", __func__, tm->time.tm_sec, tm->time.tm_min,
		tm->time.tm_hour, tm->enabled, tm->pending);

	return 0;
}

static int nct3015y_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct3015y *nct3015y = i2c_get_clientdata(client);
	unsigned char buf[3];
	int err;

	if(nct3015y->rtc_bus == 2){
		dev_err(dev, "%s, bus2 doesn't support set_alarm\n"
		, __func__);
		return -EIO;
	}

	dev_dbg(dev, "%s, sec=%d, min=%d hour=%d tm->enabled:%d\n"
		, __func__,
		tm->time.tm_sec, tm->time.tm_min, tm->time.tm_hour,
		tm->enabled);

	buf[0] = bin2bcd(tm->time.tm_sec);
	buf[1] = bin2bcd(tm->time.tm_min);
	buf[2] = bin2bcd(tm->time.tm_hour);

	err = nct3015y_write_block_data(client, NCT3015Y_REG_SCA, 1, buf);
	if (err)
		return err;

	err = nct3015y_write_block_data(client, NCT3015Y_REG_MNA, 1, buf+1);
	if (err)
		return err;

	err = nct3015y_write_block_data(client, NCT3015Y_REG_HRA, 1, buf+2);
	if (err)
		return err;

	return nct3015y_set_alarm_mode(client, tm->enabled);
}

static int nct3015y_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct3015y *nct3015y = i2c_get_clientdata(client);

	dev_dbg(dev, "%s: alarm enable=%d\n", __func__, enabled);
	if(nct3015y->rtc_bus == 2) {
		dev_err(dev, "%s, bus2 doesn't support set_alarm_mode\n"
		, __func__);
		return -EIO;
	}

	return nct3015y_set_alarm_mode(to_i2c_client(dev), enabled);
}

#ifdef CONFIG_COMMON_CLK
/*
 * Handling of the clkout
 */

#define clkout_hw_to_nct3015y(_hw) container_of(_hw, struct nct3015y, clkout_hw)

static const int clkout_rates[] = {
	32768,
	1024,
	32,
	1,
};

static unsigned long nct3015y_clkout_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct nct3015y *nct3015y = clkout_hw_to_nct3015y(hw);
	struct i2c_client *client = nct3015y->client;
	unsigned char buf;
	int ret = nct3015y_read_block_data(client, NCT3015Y_REG_CLKO, 1, &buf);

	if (ret < 0)
		return 0;

	buf &= NCT3015Y_REG_CLKO_F_MASK;
	return clkout_rates[buf];
}

static long nct3015y_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *prate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clkout_rates); i++)
		if (clkout_rates[i] <= rate)
			return clkout_rates[i];

	return 0;
}

static int nct3015y_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct nct3015y *nct3015y = clkout_hw_to_nct3015y(hw);
	struct i2c_client *client = nct3015y->client;
	unsigned char buf;
	int ret = nct3015y_read_block_data(client, NCT3015Y_REG_CLKO, 1, &buf);
	int i;

	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(clkout_rates); i++)
		if (clkout_rates[i] == rate) {
			buf &= ~NCT3015Y_REG_CLKO_F_MASK;
			buf |= i;
			ret = nct3015y_write_block_data(client,
						       NCT3015Y_REG_CLKO, 1,
						       &buf);
			return ret;
		}

	return -EINVAL;
}

static int nct3015y_clkout_control(struct clk_hw *hw, bool enable)
{
	struct nct3015y *nct3015y = clkout_hw_to_nct3015y(hw);
	struct i2c_client *client = nct3015y->client;
	unsigned char buf;
	int ret = nct3015y_read_block_data(client, NCT3015Y_REG_CLKO, 1, &buf);

	if (ret < 0)
		return ret;

	if (enable)
		buf |= NCT3015Y_REG_CLKO_CKE;
	else
		buf &= ~NCT3015Y_REG_CLKO_CKE;

	ret = nct3015y_write_block_data(client, NCT3015Y_REG_CLKO, 1, &buf);
	return ret;
}

static int nct3015y_clkout_prepare(struct clk_hw *hw)
{
	return nct3015y_clkout_control(hw, 1);
}

static void nct3015y_clkout_unprepare(struct clk_hw *hw)
{
	nct3015y_clkout_control(hw, 0);
}

static int nct3015y_clkout_is_prepared(struct clk_hw *hw)
{
	struct nct3015y *nct3015y = clkout_hw_to_nct3015y(hw);
	struct i2c_client *client = nct3015y->client;
	unsigned char buf;
	int ret = nct3015y_read_block_data(client, NCT3015Y_REG_CLKO, 1, &buf);

	if (ret < 0)
		return ret;

	return buf & NCT3015Y_REG_CLKO_CKE;
}

static const struct clk_ops nct3015y_clkout_ops = {
	.prepare = nct3015y_clkout_prepare,
	.unprepare = nct3015y_clkout_unprepare,
	.is_prepared = nct3015y_clkout_is_prepared,
	.recalc_rate = nct3015y_clkout_recalc_rate,
	.round_rate = nct3015y_clkout_round_rate,
	.set_rate = nct3015y_clkout_set_rate,
};

static struct clk *nct3015y_clkout_register_clk(struct nct3015y *nct3015y)
{
	struct i2c_client *client = nct3015y->client;
	struct device_node *node = client->dev.of_node;
	struct clk *clk;
	struct clk_init_data init;
	int ret;
	unsigned char buf;

	/* disable the clkout output */
	buf = 0;
	ret = nct3015y_write_block_data(client, NCT3015Y_REG_CLKO, 1, &buf);
	if (ret < 0)
		return ERR_PTR(ret);

	init.name = "nct3015y-clkout";
	init.ops = &nct3015y_clkout_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;
	nct3015y->clkout_hw.init = &init;

	/* optional override of the clockname */
	of_property_read_string(node, "clock-output-names", &init.name);

	/* register the clock */
	clk = devm_clk_register(&client->dev, &nct3015y->clkout_hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return clk;
}
#endif

static const struct rtc_class_ops nct3015y_rtc_ops = {
	.read_time	= nct3015y_rtc_read_time,
	.set_time	= nct3015y_rtc_set_time,
	.read_alarm	= nct3015y_rtc_read_alarm,
	.set_alarm	= nct3015y_rtc_set_alarm,
	.alarm_irq_enable = nct3015y_irq_enable,
};

static int nct3015y_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct nct3015y *nct3015y;
	int err;
	unsigned char buf;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: ENODEV\n", __func__);
		return -ENODEV;
	}

	nct3015y = devm_kzalloc(&client->dev, sizeof(struct nct3015y),
				GFP_KERNEL);
	if (!nct3015y)
		return -ENOMEM;

	i2c_set_clientdata(client, nct3015y);
	nct3015y->client = client;
	nct3015y->rtc_bus = 1;
	device_set_wakeup_capable(&client->dev, 1);

	err = nct3015y_read_block_data(client, NCT3015Y_REG_CTRL, 1, &buf);
	if (err < 0) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return err;
	} else if(buf & NCT3015Y_BIT_TWO) {
		dev_dbg(&client->dev, "%s: NCT3015Y_BIT_TWO is :%d\n",
			__func__, buf & NCT3015Y_BIT_TWO);
	}
	/*Try to Write to CTRL to check bus1 or bus2*/
	buf = 0 | NCT3015Y_BIT_TWO;
	err = nct3015y_write_block_data(client, NCT3015Y_REG_CTRL, 1, &buf);
	if (err < 0) {
		dev_err(&client->dev, "%s: write fail, so ReadOnly\n", __func__);
		nct3015y->rtc_bus=2;
	}
	if(nct3015y->rtc_bus == 1) {
		buf = 0;
		err = nct3015y_write_block_data(client, NCT3015Y_REG_ST, 1, &buf);
		if (err < 0) {
			dev_err(&client->dev, "%s: write error\n", __func__);
			return err;
		}
	}

	nct3015y->rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(nct3015y->rtc))
		return PTR_ERR(nct3015y->rtc);

	nct3015y->rtc->ops = &nct3015y_rtc_ops;
	nct3015y->rtc->uie_unsupported = 1;
	nct3015y->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	nct3015y->rtc->range_max = RTC_TIMESTAMP_END_2099;
	nct3015y->rtc->set_start_time = true;

	nct3015y->wakeup_host = devm_gpiod_get(&client->dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(nct3015y->wakeup_host)) {
		err = PTR_ERR(nct3015y->wakeup_host);
		dev_err(&client->dev, "could not get host wakeup gpio: %d", err);
		return err;
	}
	client->irq = gpiod_to_irq(nct3015y->wakeup_host);

	dev_dbg(&client->dev, "%s: client->irq:%d\n", __func__, client->irq);

	if (client->irq > 0) {
		err = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, nct3015y_irq,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				nct3015y_driver.driver.name, client);
		if (err) {
			dev_err(&client->dev, "unable to request IRQ %d\n",
								client->irq);
			return err;
		}
	}

	err = rtc_register_device(nct3015y->rtc);
	if (err)
		return err;

#ifdef CONFIG_COMMON_CLK
	/* register clk in common clk framework */
	nct3015y_clkout_register_clk(nct3015y);
#endif

	return 0;
}

static const struct i2c_device_id nct3015y_id[] = {
	{ "nct3015y", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct3015y_id);


static const struct of_device_id nct3015y_of_match[] = {
	{ .compatible = "nuvoton,nct3015y" },
	{}
};
MODULE_DEVICE_TABLE(of, nct3015y_of_match);

static struct i2c_driver nct3015y_driver = {
	.driver		= {
		.name	= "rtc-nct3015y",
		.of_match_table = of_match_ptr(nct3015y_of_match),
	},
	.probe		= nct3015y_probe,
	.id_table	= nct3015y_id,
};

module_i2c_driver(nct3015y_driver);

MODULE_AUTHOR("Medad <ctcchien@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton NCT3015Y RTC driver");
MODULE_LICENSE("GPL");
