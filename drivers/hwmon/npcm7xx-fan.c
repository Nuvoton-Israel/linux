/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/types.h>
#include <linux/uaccess.h>

typedef struct {
	u8 u8ChannelNum;
	u8 u8FanPulsePerRev;
	u16 u16FanSpeedReading;
	u32 u32InputClock;
} sFanTachData;

#define NPCM750_MFT_CLKPS   255

/* PLL input */
#define PLL_INPUT_CLOCK  25000000

/*
 * Get Fan Tach Timeout (base on clock 214843.75Hz, 1 cnt = 4.654us)
 * Timeout 94ms ~= 0x5000
 * (The minimum FAN speed could to support ~640RPM/pulse 1,
 * 320RPM/pulse 2, ...-- 10.6Hz)
 */
#define FAN_TACH_TIMEOUT   ((u16) 0x5000)

/*
 * Enable a background timer to poll fan tach value, (200ms * 4)
 * to polling all fan)
 */

/* 1 = 1 jiffies = 10 ms */
#define FAN_TACH_POLLING_INTERVAL 20

/* MFT General Defintion */
#define NPCM750_MFT_MAX_MODULE	8
#define NPCM750_CMPA	0
#define NPCM750_CMPB	1

#define NPCM750_MFT_MODE_5	4 /* Dual Independent Input Capture */

#define NPCM750_MFT_TCNT    ((u16) 0xFFFF)
#define NPCM750_MFT_TCPA    ((u16) (NPCM750_MFT_TCNT - FAN_TACH_TIMEOUT))
#define NPCM750_MFT_TCPB    ((u16) (NPCM750_MFT_TCNT - FAN_TACH_TIMEOUT))

#define NPCM750_MFT_NO_CLOCK_MODE		0
#define NPCM750_MFT_APB_CLOCK_MODE		1

#define DEFAULT_PULSE_PER_REVOLUTION	2

#define MFT_REGS_BASE(n)    NPCMX50_MFT_BASE_ADDR(n)
#define MFT_MFSEL2_FLSEL(n) (1<<n)

/* Fantach MFT registers */
#define MFT_REG_TCNT1(n)    ((void *) (MFT_REGS_BASE(n) + 0x00))
#define MFT_REG_TCRA(n)     ((void *) (MFT_REGS_BASE(n) + 0x02))
#define MFT_REG_TCRB(n)     ((void *) (MFT_REGS_BASE(n) + 0x04))
#define MFT_REG_TCNT2(n)    ((void *) (MFT_REGS_BASE(n) + 0x06))
#define MFT_REG_TPRSC(n)    ((void *) (MFT_REGS_BASE(n) + 0x08))
#define MFT_REG_TCKC(n)     ((void *) (MFT_REGS_BASE(n) + 0x0A))
#define MFT_REG_TMCTRL(n)   ((void *) (MFT_REGS_BASE(n) + 0x0C))
#define MFT_REG_TICTRL(n)   ((void *) (MFT_REGS_BASE(n) + 0x0E))
#define MFT_REG_TICLR(n)    ((void *) (MFT_REGS_BASE(n) + 0x10))
#define MFT_REG_TIEN(n)     ((void *) (MFT_REGS_BASE(n) + 0x12))
#define MFT_REG_TCPA(n)     ((void *) (MFT_REGS_BASE(n) + 0x14))
#define MFT_REG_TCPB(n)     ((void *) (MFT_REGS_BASE(n) + 0x16))
#define MFT_REG_TCPCFG(n)   ((void *) (MFT_REGS_BASE(n) + 0x18))
#define MFT_REG_TINASEL(n)  ((void *) (MFT_REGS_BASE(n) + 0x1A))
#define MFT_REG_TINBSEL(n)  ((void *) (MFT_REGS_BASE(n) + 0x1C))

#define NPCM750_TCKC_C2CSEL(mode)	    (((mode) & 0x7) << 3)
#define NPCM750_TCKC_C1CSEL(mode)	    ((mode) & 0x7)

#define NPCM750_TMCTRL_TBEN		(1<<6)
#define NPCM750_TMCTRL_TAEN		(1<<5)
#define NPCM750_TMCTRL_TBEDG	        (1<<4)
#define NPCM750_TMCTRL_TAEDG	        (1<<3)
#define NPCM750_TMCTRL_MDSEL(mode)	    ((mode) & 0x7)

#define NPCM750_TICLR_CLEAR_ALL         (0x3F)
#define NPCM750_TICLR_TFCLR             (1<<5)
#define NPCM750_TICLR_TECLR             (1<<4)
#define NPCM750_TICLR_TDCLR             (1<<3)
#define NPCM750_TICLR_TCCLR             (1<<2)
#define NPCM750_TICLR_TBCLR             (1<<1)
#define NPCM750_TICLR_TACLR             (1<<0)

#define NPCM750_TIEN_ENABLE_ALL         (0x3F)
#define NPCM750_TIEN_TFIEN              (1<<5)
#define NPCM750_TIEN_TEIEN              (1<<4)
#define NPCM750_TIEN_TDIEN              (1<<3)
#define NPCM750_TIEN_TCIEN              (1<<2)
#define NPCM750_TIEN_TBIEN              (1<<1)
#define NPCM750_TIEN_TAIEN              (1<<0)

#define NPCM750_TICTRL_TFPND            (1<<5)
#define NPCM750_TICTRL_TEPND            (1<<4)
#define NPCM750_TICTRL_TDPND            (1<<3)
#define NPCM750_TICTRL_TCPND            (1<<2)
#define NPCM750_TICTRL_TBPND            (1<<1)
#define NPCM750_TICTRL_TAPND            (1<<0)

#define NPCM750_TCPCFG_HIBEN            (1<<7)
#define NPCM750_TCPCFG_EQBEN            (1<<6)
#define NPCM750_TCPCFG_LOBEN            (1<<5)
#define NPCM750_TCPCFG_CPBSEL           (1<<4)
#define NPCM750_TCPCFG_HIAEN            (1<<3)
#define NPCM750_TCPCFG_EQAEN            (1<<2)
#define NPCM750_TCPCFG_LOAEN            (1<<1)
#define NPCM750_TCPCFG_CPASEL           (1<<0)

#define NPCM750_TINASEL_FANIN_DEFAULT   (0x0)

#define FAN_TACH_DISABLE                            0xFF
#define FAN_TACH_INIT                               0x00
#define FAN_TACH_PREPARE_TO_GET_FIRST_CAPTURE       0x01
#define FAN_TACH_ENOUGH_SAMPLE                      0x02

/* maximum fan tach input support */
#define NPCM750_MAX_FAN_TACH	 16

/* Obtain the fan number */
#define NPCM750_FAN_TACH_INPUT(mft, cmp)	((mft << 1) + (cmp))

/* Debugging Message */
#ifdef FAN_DEBUG
#define DEBUG_MSG(fmt, args...)  pr_info("PWM: %s() " fmt, __func__, ##args)
#else
#define DEBUG_MSG(fmt, args...)
#endif
#define PERROR(fmt, args...)  pr_err("PWM: %s() " fmt, __func__, ##args)

typedef struct {
	u8 u8FanStatusFlag;
	u8 u8FanPulsePerRev;
	u16 u16FanTachCnt;
	u32 u32FanTachCntTemp;
} sFanTachDev;

static int npcm750_fan_read(sFanTachData *pFanTachData);
static int mft_virt_addr;

#undef MFT_REGS_BASE
#define MFT_REGS_BASE(n)    (mft_virt_addr + ((n) * 0x1000L))

/* for request irq use */
static u8 u8dummy;
int mft_irq[8];

/* Input clock */
static u32 u32InputClock;
static sFanTachDev S_npcm750_fantach[NPCM750_MAX_FAN_TACH];
static u8 S_npcm750_fantach_select;
static struct timer_list npcm750_fantach_timer;
static struct clk *mft_clk;

struct npcm750_fan_data {
	unsigned long clk_freq;
	const struct attribute_group *groups[2];
};

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int index = sensor_attr->index;
	sFanTachData FanTachData;

	FanTachData.u8ChannelNum = index;
	npcm750_fan_read(&FanTachData);
	if (FanTachData.u16FanSpeedReading < 0)
		return FanTachData.u16FanSpeedReading;

	return sprintf(buf, "%d\n", (int)FanTachData.u16FanSpeedReading);
}

static umode_t fan_dev_is_visible(struct kobject *kobj,
				  struct attribute *a, int index)
{
	/*struct device *dev = container_of(kobj, struct device, kobj);*/
	return a->mode;
}

static SENSOR_DEVICE_ATTR(fan1_input, 0444, show_rpm, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, 0444, show_rpm, NULL, 1);
static SENSOR_DEVICE_ATTR(fan3_input, 0444, show_rpm, NULL, 2);
static SENSOR_DEVICE_ATTR(fan4_input, 0444, show_rpm, NULL, 3);
static SENSOR_DEVICE_ATTR(fan5_input, 0444, show_rpm, NULL, 4);
static SENSOR_DEVICE_ATTR(fan6_input, 0444, show_rpm, NULL, 5);
static SENSOR_DEVICE_ATTR(fan7_input, 0444, show_rpm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan8_input, 0444, show_rpm, NULL, 7);
static SENSOR_DEVICE_ATTR(fan9_input, 0444, show_rpm, NULL, 8);
static SENSOR_DEVICE_ATTR(fan10_input, 0444, show_rpm, NULL, 9);
static SENSOR_DEVICE_ATTR(fan11_input, 0444, show_rpm, NULL, 10);
static SENSOR_DEVICE_ATTR(fan12_input, 0444, show_rpm, NULL, 11);
static SENSOR_DEVICE_ATTR(fan13_input, 0444, show_rpm, NULL, 12);
static SENSOR_DEVICE_ATTR(fan14_input, 0444, show_rpm, NULL, 13);
static SENSOR_DEVICE_ATTR(fan15_input, 0444, show_rpm, NULL, 14);
static SENSOR_DEVICE_ATTR(fan16_input, 0444, show_rpm, NULL, 15);

static struct attribute *fan_dev_attrs[] = {
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan4_input.dev_attr.attr,
	&sensor_dev_attr_fan5_input.dev_attr.attr,
	&sensor_dev_attr_fan6_input.dev_attr.attr,
	&sensor_dev_attr_fan7_input.dev_attr.attr,
	&sensor_dev_attr_fan8_input.dev_attr.attr,
	&sensor_dev_attr_fan9_input.dev_attr.attr,
	&sensor_dev_attr_fan10_input.dev_attr.attr,
	&sensor_dev_attr_fan11_input.dev_attr.attr,
	&sensor_dev_attr_fan12_input.dev_attr.attr,
	&sensor_dev_attr_fan13_input.dev_attr.attr,
	&sensor_dev_attr_fan14_input.dev_attr.attr,
	&sensor_dev_attr_fan15_input.dev_attr.attr,
	&sensor_dev_attr_fan16_input.dev_attr.attr,
	NULL
};

static const struct attribute_group fan_dev_group = {
	.attrs = fan_dev_attrs,
	.is_visible = fan_dev_is_visible,
};

static inline void npcm750_fantach_start_capture(u8 mft, u8 cmp)
{
	u8 fan_id = 0;
	u8 reg_mode = 0;
	u8 reg_int = 0;

	fan_id = NPCM750_FAN_TACH_INPUT(mft, cmp);

	/* to check whether any fan tach is enable */
	if (S_npcm750_fantach[fan_id].u8FanStatusFlag != FAN_TACH_DISABLE) {
		/* reset status */
		S_npcm750_fantach[fan_id].u8FanStatusFlag = FAN_TACH_INIT;
		reg_int = ioread8((void *)MFT_REG_TIEN(mft));

		if (cmp == NPCM750_CMPA) {
			/* enable interrupt */
			iowrite8((u8) (reg_int | (NPCM750_TIEN_TAIEN |
						  NPCM750_TIEN_TEIEN)),
				 (void *)MFT_REG_TIEN(mft));

			reg_mode =
				NPCM750_TCKC_C1CSEL(NPCM750_MFT_APB_CLOCK_MODE)
				| ioread8((void *)MFT_REG_TCKC(mft));

			/* start to Capture */
			iowrite8(reg_mode, (void *)MFT_REG_TCKC(mft));
		} else {
			/* enable interrupt */
			iowrite8((u8) (reg_int | (NPCM750_TIEN_TBIEN |
						  NPCM750_TIEN_TFIEN)),
				 (void *)MFT_REG_TIEN(mft));

			reg_mode =
				NPCM750_TCKC_C2CSEL(NPCM750_MFT_APB_CLOCK_MODE)
				| ioread8((void *)MFT_REG_TCKC(mft));

			/* start to Capture */
			iowrite8(reg_mode, (void *)MFT_REG_TCKC(mft));
		}
	}
}

static void npcm750_fantach_polling(unsigned long data)
{
	int i;

	/* Polling two module per one round,
	 * MFT0 & MFT4 / MFT1 & MFT5 / MFT2 & MFT6 / MFT3 & MFT7
	 */
	for (i = S_npcm750_fantach_select; i < NPCM750_MFT_MAX_MODULE;
	      i = i+4) {
		/* clear the flag and reset the counter (TCNT) */
		iowrite8((u8) NPCM750_TICLR_CLEAR_ALL,
			 (void *) MFT_REG_TICLR(i));

		iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT1(i));
		iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT2(i));

		npcm750_fantach_start_capture(i, NPCM750_CMPA);
		npcm750_fantach_start_capture(i, NPCM750_CMPB);
	}

	S_npcm750_fantach_select++;
	S_npcm750_fantach_select &= 0x3;

	/* reset the timer interval */
	npcm750_fantach_timer.expires = jiffies + FAN_TACH_POLLING_INTERVAL;
	add_timer(&npcm750_fantach_timer);
}

static int npcm750_fan_read(sFanTachData *pFanTachData)
{
	u8 fan_id = 0;

	fan_id = pFanTachData->u8ChannelNum;

	if (S_npcm750_fantach[fan_id].u16FanTachCnt != 0)
		pFanTachData->u16FanSpeedReading =
		S_npcm750_fantach[fan_id].u16FanTachCnt;
	else
		pFanTachData->u16FanSpeedReading = 0;

	return  0;
}

static inline void npcm750_fantach_compute(u8 mft, u8 cmp, u8 fan_id,
					   u8 flag_int, u8 flag_mode,
					   u8 flag_clear)
{
	u8  reg_int  = 0;
	u8  reg_mode = 0;
	u16 fan_cap  = 0;

	if (cmp == NPCM750_CMPA)
		fan_cap = ioread16((void *) MFT_REG_TCRA(mft));
	else
		fan_cap = ioread16((void *) MFT_REG_TCRB(mft));

	/* clear capature flag, H/W will auto reset the NPCM750_TCNTx */
	iowrite8((u8) flag_clear, (void *) MFT_REG_TICLR(mft));

	if (S_npcm750_fantach[fan_id].u8FanStatusFlag == FAN_TACH_INIT) {
		/* First capture, drop it */
		S_npcm750_fantach[fan_id].u8FanStatusFlag =
			FAN_TACH_PREPARE_TO_GET_FIRST_CAPTURE;

		/* reset counter */
		S_npcm750_fantach[fan_id].u32FanTachCntTemp = 0;
	} else if (S_npcm750_fantach[fan_id].u8FanStatusFlag <
		   FAN_TACH_ENOUGH_SAMPLE) {
		/*
		 * collect the enough sample,
		 * (ex: 2 pulse fan need to get 2 sample)
		 */
		S_npcm750_fantach[fan_id].u32FanTachCntTemp +=
			(NPCM750_MFT_TCNT - fan_cap);
		/*
		 * DEBUG_MSG("step 1, fan %d cnt %d total %x\n",
		 *	fan_id, (NPCM750_MFT_TCNT - fan_cap),
		 *	(u32) S_npcm750_fantach[fan_id].u32FanTachCntTemp);
		 */
		S_npcm750_fantach[fan_id].u8FanStatusFlag++;
	} else {
		/* get enough sample or fan disable */
		if (S_npcm750_fantach[fan_id].u8FanStatusFlag ==
		    FAN_TACH_ENOUGH_SAMPLE) {
			S_npcm750_fantach[fan_id].u32FanTachCntTemp +=
				(NPCM750_MFT_TCNT - fan_cap);
			/*
			 * DEBUG_MSG("step 2, fan %d cnt %d total %x\n",
			 *   fan_id, (NPCM750_MFT_TCNT - fan_cap),
			 *   (u32)S_npcm750_fantach[fan_id].u32FanTachCntTemp);
			 */

			/* compute finial average cnt per pulse */
			S_npcm750_fantach[fan_id].u16FanTachCnt
				= S_npcm750_fantach[fan_id].u32FanTachCntTemp /
				FAN_TACH_ENOUGH_SAMPLE;

			/*
			 * DEBUG_MSG("step 3 fan %d avg %d\n\n",
			 *   fan_id, S_npcm750_fantach[fan_id].u16FanTachCnt);
			 */
			S_npcm750_fantach[fan_id].u8FanStatusFlag =
				FAN_TACH_INIT;
		}

		reg_int =  ioread8((void *)MFT_REG_TIEN(mft));

		/* disable interrupt */
		iowrite8((u8) (reg_int & ~flag_int), (void *)MFT_REG_TIEN(mft));
		reg_mode =  ioread8((void *)MFT_REG_TCKC(mft));

		/* stop capturing */
		iowrite8((u8) (reg_mode & ~flag_mode),
			 (void *) MFT_REG_TCKC(mft));
	}
}

static inline void npcm750_check_cmp(u8 mft, u8 cmp, u8 flag)
{
	u8 reg_int = 0;
	u8 reg_mode = 0;
	u8 flag_timeout;
	u8 flag_cap;
	u8 flag_clear;
	u8 flag_int;
	u8 flag_mode;
	u8 fan_id;

	fan_id = NPCM750_FAN_TACH_INPUT(mft, cmp);

	if (cmp == NPCM750_CMPA) {
		flag_cap = NPCM750_TICTRL_TAPND;
		flag_timeout = NPCM750_TICTRL_TEPND;
		flag_int = (NPCM750_TIEN_TAIEN | NPCM750_TIEN_TEIEN);
		flag_mode = NPCM750_TCKC_C1CSEL(NPCM750_MFT_APB_CLOCK_MODE);
		flag_clear = NPCM750_TICLR_TACLR | NPCM750_TICLR_TECLR;
	} else {
		flag_cap = NPCM750_TICTRL_TBPND;
		flag_timeout = NPCM750_TICTRL_TFPND;
		flag_int = (NPCM750_TIEN_TBIEN | NPCM750_TIEN_TFIEN);
		flag_mode = NPCM750_TCKC_C2CSEL(NPCM750_MFT_APB_CLOCK_MODE);
		flag_clear = NPCM750_TICLR_TBCLR | NPCM750_TICLR_TFCLR;
	}

	if (flag & flag_timeout) {
		reg_int =  ioread8((void *)MFT_REG_TIEN(mft));

		/** disable interrupt */
		iowrite8((u8) (reg_int & ~flag_int), (void *)MFT_REG_TIEN(mft));

		/** clear interrup flag */
		iowrite8((u8) flag_clear, (void *) MFT_REG_TICLR(mft));

		reg_mode =  ioread8((void *)MFT_REG_TCKC(mft));

		/** stop capturing */
		iowrite8((u8) (reg_mode & ~flag_mode),
			 (void *) MFT_REG_TCKC(mft));

		/*
		 *  If timeout occurs (FAN_TACH_TIMEOUT), the fan doesn't
		 *  connect or speed is lower than 10.6Hz (320RPM/pulse2).
		 *  In these situation, the RPM output should be zero.
		 */
		S_npcm750_fantach[fan_id].u16FanTachCnt = 0;
		DEBUG_MSG("%s : it is timeout fan_id %d\n", __func__, fan_id);
	} else {
	    /** input capture is occurred */
		if (flag & flag_cap)
			npcm750_fantach_compute(mft, cmp, fan_id, flag_int,
						flag_mode, flag_clear);
	}
}

static irqreturn_t npcm750_mft0_isr(int irq, void *dev_id)
{
	u8 flag = 0;
	int module;

	module = irq - mft_irq[0];
	flag = ioread8((void *)(void *) MFT_REG_TICTRL(module));
	if (flag > 0) {
		npcm750_check_cmp(module, NPCM750_CMPA, flag);
		npcm750_check_cmp(module, NPCM750_CMPB, flag);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int npcm750_fan_probe(struct platform_device *pdev)
{
	u32 apb_clk_src;
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct npcm750_fan_data *priv;
	struct resource res;
	struct device *hwmon;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	np = dev->of_node;

	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		pr_err("\t\t\t of_address_to_resource fail ret %d\n", ret);
		return -EINVAL;
	}

	mft_virt_addr = (int)ioremap(res.start, resource_size(&res));

	if (!mft_virt_addr) {
		pr_err("\t\t\t mft_virt_addr fail\n");
		return -ENOMEM;
	}

	DEBUG_MSG("MFT base is 0x%08X ,res.start 0x%08X\n",
		  (u32)mft_virt_addr, res.start);

	mft_clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(mft_clk)) {
		pr_err(" MFT (FAN) probe failed: can't read clk.\n");
		return -ENODEV;
	}

	clk_prepare_enable(mft_clk);

	for (i = 0; i < NPCM750_MFT_MAX_MODULE; i++) {
		/* stop MFT0~7 clock */
		iowrite8((u8) NPCM750_MFT_NO_CLOCK_MODE,
			 (void *)MFT_REG_TCKC(i));

		/* disable all interrupt */
		iowrite8((u8) 0x00, (void *)MFT_REG_TIEN(i));

		/* clear all interrupt */
		iowrite8((u8) NPCM750_TICLR_CLEAR_ALL,
			 (void *)MFT_REG_TICLR(i));

		/* set MFT0~7 clock prescaler */
		iowrite8((u8) NPCM750_MFT_CLKPS, (void *)MFT_REG_TPRSC(i));

		/* set MFT0~7 mode (high-to-low transition) */
		iowrite8(
			(u8) (
			      NPCM750_TMCTRL_MDSEL(NPCM750_MFT_MODE_5) |
			      NPCM750_TMCTRL_TBEN |
			      NPCM750_TMCTRL_TAEN
			      ),
			(void *) MFT_REG_TMCTRL(i)
			);

		/* set MFT0~7 Initial Count/Cap */
		iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT1(i));
		iowrite16(NPCM750_MFT_TCNT, (void *)MFT_REG_TCNT2(i));

		/* set MFT0~7 compare (equal to count) */
		iowrite8((u8)(NPCM750_TCPCFG_EQAEN | NPCM750_TCPCFG_EQBEN),
			  (void *)MFT_REG_TCPCFG(i));

		/* set MFT0~7 compare value */
		iowrite16(NPCM750_MFT_TCPA, (void *)MFT_REG_TCPA(i));
		iowrite16(NPCM750_MFT_TCPB, (void *)MFT_REG_TCPB(i));

		/* set MFT0~7 fan input FANIN 0~15 */
		iowrite8((u8) NPCM750_TINASEL_FANIN_DEFAULT,
			 (void *)MFT_REG_TINASEL(i));
		iowrite8((u8) NPCM750_TINASEL_FANIN_DEFAULT,
			 (void *)MFT_REG_TINBSEL(i));
	}

	/** fan tach structure initialization */
	S_npcm750_fantach_select = 0;
	for (i = 0; i < NPCM750_MAX_FAN_TACH; i++) {
		S_npcm750_fantach[i].u8FanStatusFlag = FAN_TACH_DISABLE;
		S_npcm750_fantach[i].u8FanPulsePerRev =
			DEFAULT_PULSE_PER_REVOLUTION;
		S_npcm750_fantach[i].u16FanTachCnt = 0;
	}

	for (i = 0; i < 8; i++) {
		mft_irq[i] = platform_get_irq(pdev, i);
		if (!mft_irq[i]) {
			pr_err("%s - failed to map irq %d\n", __func__, i);
			return (-EAGAIN);
		}
	}

	if (request_irq(mft_irq[0], (irq_handler_t) npcm750_mft0_isr, 0,
			"NPCM750-MFT0", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT0 failed\n");
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[1], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT1", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT1 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[2], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT2", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT2 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		free_irq(mft_irq[1], (void *) &u8dummy);
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[3], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT3", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT3 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		free_irq(mft_irq[1], (void *) &u8dummy);
		free_irq(mft_irq[2], (void *) &u8dummy);
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[4], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT4", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT4 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		free_irq(mft_irq[1], (void *) &u8dummy);
		free_irq(mft_irq[2], (void *) &u8dummy);
		free_irq(mft_irq[3], (void *) &u8dummy);
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[5], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT5", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT5 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		free_irq(mft_irq[1], (void *) &u8dummy);
		free_irq(mft_irq[2], (void *) &u8dummy);
		free_irq(mft_irq[3], (void *) &u8dummy);
		free_irq(mft_irq[4], (void *) &u8dummy);
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[6], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT6", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT6 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		free_irq(mft_irq[1], (void *) &u8dummy);
		free_irq(mft_irq[2], (void *) &u8dummy);
		free_irq(mft_irq[3], (void *) &u8dummy);
		free_irq(mft_irq[4], (void *) &u8dummy);
		free_irq(mft_irq[5], (void *) &u8dummy);
		return (-EAGAIN);
	}

	if (request_irq(mft_irq[7], (irq_handler_t) npcm750_mft0_isr, 0,
		    "NPCM750-MFT7", (void *) &u8dummy)) {
		pr_err("NPCM750: register irq MFT7 failed\n");
		free_irq(mft_irq[0], (void *) &u8dummy);
		free_irq(mft_irq[1], (void *) &u8dummy);
		free_irq(mft_irq[2], (void *) &u8dummy);
		free_irq(mft_irq[3], (void *) &u8dummy);
		free_irq(mft_irq[4], (void *) &u8dummy);
		free_irq(mft_irq[5], (void *) &u8dummy);
		free_irq(mft_irq[6], (void *) &u8dummy);
		return (-EAGAIN);
	}

	/** initialize fan tach polling timer */
	npcm750_fantach_timer.data = 0;
	npcm750_fantach_timer.function = &npcm750_fantach_polling;

	/** set timer interval */
	npcm750_fantach_timer.expires = jiffies + FAN_TACH_POLLING_INTERVAL;

	init_timer(&npcm750_fantach_timer);
	add_timer(&npcm750_fantach_timer);

	    apb_clk_src = clk_get_rate(mft_clk);
	pr_info("[FAN] APB4: %d\n", (int)apb_clk_src);
	/* Fan tach input clock = APB clock / prescalar, default is 255. */
	u32InputClock = apb_clk_src / (NPCM750_MFT_CLKPS + 1);

	pr_info("[FAN] PWM: %d\n", (int) u32InputClock);
	pr_info("[FAN] InputClock: %d\n", (int) u32InputClock);

	priv->groups[0] = &fan_dev_group;
	priv->groups[1] = NULL;
	hwmon = devm_hwmon_device_register_with_groups(dev, "npcm750_fan",
						       priv, priv->groups);
	if (IS_ERR(hwmon)) {
		pr_err("FAN Driver failed - "
		       "devm_hwmon_device_register_with_groups failed\n");
		return PTR_ERR(hwmon);
	}

	pr_info("NPCM750 FAN Driver probed\n");

	for (i = 0; i < NPCM750_MAX_FAN_TACH; i++)
		S_npcm750_fantach[i].u8FanStatusFlag = FAN_TACH_INIT;

	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750-fan");

static const struct of_device_id of_fan_match_table[] = {
	{ .compatible = "nuvoton,npcm750-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_fan_match_table);

static struct platform_driver npcm750_fan_driver = {
	.probe		= npcm750_fan_probe,
	.driver		= {
		.name	= "npcm750_fan",
		.of_match_table = of_fan_match_table,
	},
};

module_platform_driver(npcm750_fan_driver);

MODULE_DESCRIPTION("Nuvoton NPCM750 FAN Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
