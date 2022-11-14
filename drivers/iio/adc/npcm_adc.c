// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Nuvoton Technology corporation.

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/reset.h>

struct npcm_adc_info {
	u32 data_mask;
	u32 internal_vref;
	u32 res_bits;
	u32 min_val;
	u32 max_val;
	u32 const_r1;
	u32 const_r2;
};

struct npcm_adc {
	u32 R05;
	u32 R15;
	bool int_status;
	u32 adc_sample_hz;
	struct device *dev;
	void __iomem *regs;
	struct clk *adc_clk;
	wait_queue_head_t wq;
	struct regulator *vref;
	struct reset_control *reset;
	/*
	 * Lock to protect the device state during a potential concurrent
	 * read access from userspace. Reading a raw value requires a sequence
	 * of register writes, then a wait for a event and finally a register
	 * read, during which userspace could issue another read request.
	 * This lock protects a read access from ocurring before another one
	 * has finished.
	 */
	struct mutex lock;
	const struct npcm_adc_info *data;
};

/* ADC registers */
#define NPCM_ADCCON	 0x00
#define NPCM_ADCDATA	 0x04

/* ADCCON Register Bits */
#define NPCM_ADCCON_ADC_INT_EN		BIT(21)
#define NPCM_ADCCON_REFSEL		BIT(19)
#define NPCM_ADCCON_ADC_INT_ST		BIT(18)
#define NPCM_ADCCON_ADC_EN		BIT(17)
#define NPCM_ADCCON_ADC_RST		BIT(16)
#define NPCM_ADCCON_ADC_CONV		BIT(13)

#define NPCM_ADCCON_CH_MASK		GENMASK(27, 24)
#define NPCM_ADCCON_CH(x)		((x) << 24)
#define NPCM_ADCCON_DIV_SHIFT		1
#define NPCM_ADCCON_DIV_MASK		GENMASK(8, 1)

#define NPCM_ADC_ENABLE		(NPCM_ADCCON_ADC_EN | NPCM_ADCCON_ADC_INT_EN)

/* FUSE registers */
#define NPCM_FUSE_FST		0x00
#define NPCM_FUSE_FADDR		0x04
#define NPCM_FUSE_FDATA		0x08
#define NPCM_FUSE_FCFG		0x0C
#define NPCM_FUSE_FCTL		0x14

/* FST Register Bits */
#define NPCM_FUSE_FST_RDY	BIT(0)
#define NPCM_FUSE_FST_RDST	BIT(1)

/* FADDR Register Bits */
#define NPCM_FUSE_FADDR_BYTEADDR	BIT(0)
#define NPCM_FUSE_FADDR_BYTEADDR_MASK	GENMASK(9, 0)

/* FADDR Register Bits */
#define NPCM_FUSE_FDATA_DATA		BIT(0)
#define NPCM_FUSE_FDATA_CLEAN_VALUE	BIT(1)
#define NPCM_FUSE_FDATA_DATA_MASK	GENMASK(7, 0)

/* FCTL Register Bits */
#define NPCM_FUSE_FCTL_RDST		BIT(1)

/* ADC Calibration Definition */
#define FUSE_CALIB_ADDR		24
#define FUSE_CALIB_SIZE		8
#define DATA_CALIB_SIZE		4
#define FUSE_READ_SLEEP		500
#define FUSE_READ_TIMEOUT	1000000

static const struct npcm_adc_info npxm7xx_adc_info = {
	.data_mask = GENMASK(9, 0), 
	.internal_vref = 2048,
	.res_bits = 10,
	.min_val = 0,
	.max_val = 1023,
	.const_r1 = 512,
	.const_r2 = 768
};

static const struct npcm_adc_info npxm8xx_adc_info = {
	.data_mask = GENMASK(11, 0), 
	.internal_vref = 1229,
	.res_bits = 12,
	.min_val = 0,
	.max_val = 4095,
	.const_r1 = 1024,
	.const_r2 = 3072
};

#define NPCM_ADC_CHAN(ch) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = ch,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec npcm_adc_iio_channels[] = {
	NPCM_ADC_CHAN(0),
	NPCM_ADC_CHAN(1),
	NPCM_ADC_CHAN(2),
	NPCM_ADC_CHAN(3),
	NPCM_ADC_CHAN(4),
	NPCM_ADC_CHAN(5),
	NPCM_ADC_CHAN(6),
	NPCM_ADC_CHAN(7),
};

static void npcm_fuse_read(struct regmap *fuse_regmap, u32 addr, u8 *data)
{
	u32 val;
	u32 fstreg;

	regmap_read_poll_timeout(fuse_regmap, NPCM_FUSE_FST, fstreg,
				 fstreg & NPCM_FUSE_FST_RDY, FUSE_READ_SLEEP,
				 FUSE_READ_TIMEOUT);
	regmap_write_bits(fuse_regmap, NPCM_FUSE_FST,
			  NPCM_FUSE_FST_RDST, NPCM_FUSE_FST_RDST);

	regmap_write_bits(fuse_regmap, NPCM_FUSE_FADDR,
			  NPCM_FUSE_FADDR_BYTEADDR_MASK, addr);
	regmap_read(fuse_regmap, NPCM_FUSE_FADDR, &val);
	regmap_write(fuse_regmap, NPCM_FUSE_FCTL, NPCM_FUSE_FCTL_RDST);

	regmap_read_poll_timeout(fuse_regmap, NPCM_FUSE_FST, fstreg,
				 fstreg & NPCM_FUSE_FST_RDY, FUSE_READ_SLEEP,
				 FUSE_READ_TIMEOUT);
	regmap_write_bits(fuse_regmap, NPCM_FUSE_FST,
			  NPCM_FUSE_FST_RDST, NPCM_FUSE_FST_RDST);

	regmap_read(fuse_regmap, NPCM_FUSE_FDATA, &val);
	*data = (u8)val;

	regmap_write_bits(fuse_regmap, NPCM_FUSE_FDATA, NPCM_FUSE_FDATA_DATA_MASK,
			  NPCM_FUSE_FDATA_CLEAN_VALUE);
}

static int npcm_ECC_to_nibble(u8 ECC, u8 nibble)
{
	u8 nibble_b0 = (nibble >> 0) & BIT(0);
	u8 nibble_b1 = (nibble >> 1) & BIT(0);
	u8 nibble_b2 = (nibble >> 2) & BIT(0);
	u8 nibble_b3 = (nibble >> 3) & BIT(0);
	u8 tmp_ECC = nibble;

	tmp_ECC |= (nibble_b0 ^ nibble_b1) << 4 | (nibble_b2 ^ nibble_b3) << 5 |
		(nibble_b0 ^ nibble_b2) << 6  | (nibble_b1 ^ nibble_b3) << 7;

	if (tmp_ECC != ECC)
		return -EINVAL;

	return 0;
}

static int npcm_ECC_to_byte(u16 ECC, u8 *Byte)
{
	u8 nibble_L, nibble_H;
	u8 ECC_L, ECC_H;

	ECC_H = ECC >> 8;
	nibble_H = ECC_H & 0x0F;
	ECC_L = ECC >> 0;
	nibble_L = ECC_L & 0x0F;

	if (npcm_ECC_to_nibble(ECC_H, nibble_H) != 0 ||
	    npcm_ECC_to_nibble(ECC_L, nibble_L) != 0)
		return -EINVAL;

	*Byte = nibble_H << 4 | nibble_L << 0;

	return 0;
}

static int npcm_read_nibble_parity(u8 *block_ECC, u8 *ADC_calib)
{
	int i;
	u16 ECC;

	for (i = 0; i < DATA_CALIB_SIZE; i++) {
		memcpy(&ECC, block_ECC + (i * 2), 2);
		if (npcm_ECC_to_byte(ECC, &ADC_calib[i]) != 0)
			return -EINVAL;
	}

	return 0;
}

static int npcm_fuse_calibration_read(struct platform_device *pdev,
					 struct npcm_adc *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct regmap *fuse_regmap;
	ssize_t bytes_read = 0;
	u8 read_buf[8];
	u32 ADC_calib;
	u32 addr = FUSE_CALIB_ADDR;

	fuse_regmap = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(fuse_regmap)) {
		dev_warn(&pdev->dev, "Failed to find syscon\n");
		return PTR_ERR(fuse_regmap);
	}

	while (bytes_read < FUSE_CALIB_SIZE) {
		npcm_fuse_read(fuse_regmap, addr,
				  &read_buf[bytes_read]);
		bytes_read++;
		addr++;
	}

	if (npcm_read_nibble_parity(read_buf, (u8 *)&ADC_calib)) {
		dev_warn(info->dev, "FUSE Calibration read failed\n");
		return -EINVAL;
	}

	info->R05 = ADC_calib & 0xFFFF;
	info->R15 = ADC_calib >> 16;

	return 0;
}

static irqreturn_t npcm_adc_isr(int irq, void *data)
{
	u32 regtemp;
	struct iio_dev *indio_dev = data;
	struct npcm_adc *info = iio_priv(indio_dev);

	regtemp = ioread32(info->regs + NPCM_ADCCON);
	if (regtemp & NPCM_ADCCON_ADC_INT_ST) {
		iowrite32(regtemp, info->regs + NPCM_ADCCON);
		wake_up_interruptible(&info->wq);
		info->int_status = true;
	}

	return IRQ_HANDLED;
}

static int npcm_adc_read(struct npcm_adc *info, int *val, u8 channel)
{
	int ret;
	u32 regtemp;

	/* Select ADC channel */
	regtemp = ioread32(info->regs + NPCM_ADCCON);
	regtemp &= ~NPCM_ADCCON_CH_MASK;
	info->int_status = false;
	iowrite32(regtemp | NPCM_ADCCON_CH(channel) |
		  NPCM_ADCCON_ADC_CONV, info->regs + NPCM_ADCCON);

	ret = wait_event_interruptible_timeout(info->wq, info->int_status,
					       msecs_to_jiffies(10));
	if (ret == 0) {
		regtemp = ioread32(info->regs + NPCM_ADCCON);
		if (regtemp & NPCM_ADCCON_ADC_CONV) {
			/* if conversion failed - reset ADC module */
			reset_control_assert(info->reset);
			msleep(100);
			reset_control_deassert(info->reset);
			msleep(100);

			/* Enable ADC and start conversion module */
			iowrite32(NPCM_ADC_ENABLE | NPCM_ADCCON_ADC_CONV,
				  info->regs + NPCM_ADCCON);
			dev_err(info->dev, "RESET ADC Complete\n");
		}
		return -ETIMEDOUT;
	}
	if (ret < 0)
		return ret;

	*val = ioread32(info->regs + NPCM_ADCDATA);
	*val &= info->data->data_mask;

	return 0;
}

static void npcm_adc_calibration(int *val, struct npcm_adc *info)
{
	int mul_val;
	int offset_val;

	mul_val = info->data->const_r1 * (*val - info->R15);
	if (mul_val < 0) {
		mul_val = mul_val * -1;
		offset_val = DIV_ROUND_CLOSEST(mul_val,
					       (info->R15 - info->R05));
		*val = info->data->const_r2 - offset_val;
	} else {
		offset_val = DIV_ROUND_CLOSEST(mul_val,
					       (info->R15 - info->R05));
		*val = info->data->const_r2 + offset_val;
	}

	if (*val < info->data->min_val)
		*val = info->data->min_val;
	if (*val > info->data->max_val)
		*val = info->data->max_val;
}

static int npcm_adc_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	int ret;
	int vref_uv;
	struct npcm_adc *info = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&info->lock);
		ret = npcm_adc_read(info, val, chan->channel);
		mutex_unlock(&info->lock);
		if (ret) {
			if (ret != -ERESTARTSYS)
				dev_err(info->dev, "NPCM ADC read failed\n");
			return ret;
		}

		if ((info->R05 || info->R15) && IS_ERR(info->vref))
			npcm_adc_calibration(val, info);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (!IS_ERR(info->vref)) {
			vref_uv = regulator_get_voltage(info->vref);
			*val = vref_uv / 1000;
		} else {
			*val = info->data->internal_vref;
		}
		*val2 = info->data->res_bits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = info->adc_sample_hz;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct iio_info npcm_adc_iio_info = {
	.read_raw = &npcm_adc_read_raw,
};

static const struct of_device_id npcm_adc_match[] = {
	{ .compatible = "nuvoton,npcm750-adc", .data = &npxm7xx_adc_info},
	{ .compatible = "nuvoton,npcm845-adc", .data = &npxm8xx_adc_info},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, npcm_adc_match);

static int npcm_adc_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	u32 div;
	u32 reg_con;
	struct npcm_adc *info;
	struct iio_dev *indio_dev;
	struct device *dev = &pdev->dev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;
	info = iio_priv(indio_dev);

	info->data = device_get_match_data(dev);
	if (!info->data)
		return -EINVAL;

	mutex_init(&info->lock);

	info->dev = &pdev->dev;

	info->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);

	info->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(info->reset))
		return PTR_ERR(info->reset);

	info->adc_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(info->adc_clk)) {
		dev_warn(&pdev->dev, "ADC clock failed: can't read clk\n");
		return PTR_ERR(info->adc_clk);
	}

	/* calculate ADC clock sample rate */
	reg_con = ioread32(info->regs + NPCM_ADCCON);
	div = reg_con & NPCM_ADCCON_DIV_MASK;
	div = div >> NPCM_ADCCON_DIV_SHIFT;
	info->adc_sample_hz = clk_get_rate(info->adc_clk) / ((div + 1) * 2);

	ret = clk_prepare_enable(info->adc_clk);
	if (ret) {
		dev_warn(&pdev->dev, "failed to enable the clock\n");
		goto err_disable_clk;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		ret = -EINVAL;
		goto err_disable_clk;
	}

	ret = devm_request_irq(&pdev->dev, irq, npcm_adc_isr, 0,
			       "NPCM_ADC", indio_dev);
	if (ret < 0) {
		dev_err(dev, "failed requesting interrupt\n");
		goto err_disable_clk;
	}

	reg_con = ioread32(info->regs + NPCM_ADCCON);
	info->vref = devm_regulator_get_optional(&pdev->dev, "vref");
	if (!IS_ERR(info->vref)) {
		ret = regulator_enable(info->vref);
		if (ret) {
			dev_err(&pdev->dev, "Can't enable ADC reference voltage\n");
			goto err_disable_clk;
		}

		iowrite32(reg_con & ~NPCM_ADCCON_REFSEL,
			  info->regs + NPCM_ADCCON);
	} else {
		/*
		 * Any error which is not ENODEV indicates the regulator
		 * has been specified and so is a failure case.
		 */
		if (PTR_ERR(info->vref) != -ENODEV) {
			ret = PTR_ERR(info->vref);
			goto err_disable_clk;
		}

		/* Use internal reference */
		iowrite32(reg_con | NPCM_ADCCON_REFSEL,
			  info->regs + NPCM_ADCCON);
	}

	npcm_fuse_calibration_read(pdev, info);
	init_waitqueue_head(&info->wq);

	reg_con = ioread32(info->regs + NPCM_ADCCON);
	reg_con |= NPCM_ADC_ENABLE;

	/* Enable the ADC Module */
	iowrite32(reg_con, info->regs + NPCM_ADCCON);

	/* Start ADC conversion */
	iowrite32(reg_con | NPCM_ADCCON_ADC_CONV, info->regs + NPCM_ADCCON);

	platform_set_drvdata(pdev, indio_dev);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->info = &npcm_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = npcm_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(npcm_adc_iio_channels);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto err_iio_register;
	}

	pr_info("NPCM ADC driver probed\n");

	return 0;

err_iio_register:
	iowrite32(reg_con & ~NPCM_ADCCON_ADC_EN, info->regs + NPCM_ADCCON);
	if (!IS_ERR(info->vref))
		regulator_disable(info->vref);
err_disable_clk:
	clk_disable_unprepare(info->adc_clk);

	return ret;
}

static int npcm_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct npcm_adc *info = iio_priv(indio_dev);
	u32 regtemp;

	iio_device_unregister(indio_dev);

	regtemp = ioread32(info->regs + NPCM_ADCCON);
	iowrite32(regtemp & ~NPCM_ADCCON_ADC_EN, info->regs + NPCM_ADCCON);
	if (!IS_ERR(info->vref))
		regulator_disable(info->vref);
	clk_disable_unprepare(info->adc_clk);

	return 0;
}

static struct platform_driver npcm_adc_driver = {
	.probe		= npcm_adc_probe,
	.remove		= npcm_adc_remove,
	.driver		= {
		.name	= "npcm_adc",
		.of_match_table = npcm_adc_match,
	},
};

module_platform_driver(npcm_adc_driver);

MODULE_DESCRIPTION("Nuvoton NPCM ADC Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
