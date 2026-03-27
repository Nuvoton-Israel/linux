// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Intel Corporation.*/

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <linux/i3c/device.h>
#include <linux/i3c/master.h>

#include "internals.h"
#define I3C_HUB_TP_MAX_COUNT				0x08

/* I3C HUB REGISTERS */

/*
 * In this driver Controller - Target convention is used. All the abbreviations are
 * based on this convention. For instance: CP - Controller Port, TP - Target Port.
 */

/* Device Information Registers */
#define I3C_HUB_DEV_INFO_0				0x00
#define I3C_HUB_DEV_INFO_1				0x01
#define I3C_HUB_PID_5					0x02
#define I3C_HUB_PID_4					0x03
#define I3C_HUB_PID_3					0x04
#define I3C_HUB_PID_2					0x05
#define I3C_HUB_PID_1					0x06
#define I3C_HUB_PID_0					0x07
#define I3C_HUB_BCR					0x08
#define I3C_HUB_DCR					0x09
#define I3C_HUB_DEV_CAPAB				0x0A
#define I3C_HUB_DEV_REV					0x0B

/* Device Configuration Registers */
#define I3C_HUB_PROTECTION_CODE				0x10
#define  REGISTERS_LOCK_CODE				0x00
#define  REGISTERS_UNLOCK_CODE				0x69
#define  CP1_REGISTERS_UNLOCK_CODE			0x6A

#define I3C_HUB_CP_CONF					0x11
#define I3C_HUB_TP_ENABLE				0x12
#define  TPn_ENABLE(n)					BIT(n)

#define I3C_HUB_DEV_CONF				0x13
#define I3C_HUB_IO_STRENGTH				0x14
#define  TP0145_IO_STRENGTH_MASK			GENMASK(1, 0)
#define  TP0145_IO_STRENGTH(x)				(((x) << 0) & TP0145_IO_STRENGTH_MASK)
#define  TP2367_IO_STRENGTH_MASK			GENMASK(3, 2)
#define  TP2367_IO_STRENGTH(x)				(((x) << 2) & TP2367_IO_STRENGTH_MASK)
#define  CP0_IO_STRENGTH_MASK				GENMASK(5, 4)
#define  CP0_IO_STRENGTH(x)				(((x) << 4) & CP0_IO_STRENGTH_MASK)
#define  CP1_IO_STRENGTH_MASK				GENMASK(7, 6)
#define  CP1_IO_STRENGTH(x)				(((x) << 6) & CP1_IO_STRENGTH_MASK)
#define  IO_STRENGTH_20_OHM				0x00
#define  IO_STRENGTH_30_OHM				0x01
#define  IO_STRENGTH_40_OHM				0x02
#define  IO_STRENGTH_50_OHM				0x03

#define I3C_HUB_NET_OPER_MODE_CONF			0x15
#define I3C_HUB_LDO_CONF				0x16
#define  CP0_LDO_VOLTAGE_MASK				GENMASK(1, 0)
#define  CP0_LDO_VOLTAGE(x)				(((x) << 0) & CP0_LDO_VOLTAGE_MASK)
#define  CP1_LDO_VOLTAGE_MASK				GENMASK(3, 2)
#define  CP1_LDO_VOLTAGE(x)				(((x) << 2) & CP1_LDO_VOLTAGE_MASK)
#define  TP0145_LDO_VOLTAGE_MASK			GENMASK(5, 4)
#define  TP0145_LDO_VOLTAGE(x)				(((x) << 4) & TP0145_LDO_VOLTAGE_MASK)
#define  TP2367_LDO_VOLTAGE_MASK			GENMASK(7, 6)
#define  TP2367_LDO_VOLTAGE(x)				(((x) << 6) & TP2367_LDO_VOLTAGE_MASK)
#define  LDO_VOLTAGE_1_0V				0x00
#define  LDO_VOLTAGE_1_1V				0x01
#define  LDO_VOLTAGE_1_2V				0x02
#define  LDO_VOLTAGE_1_8V				0x03

#define I3C_HUB_TP_IO_MODE_CONF				0x17
#define I3C_HUB_TP_SMBUS_AGNT_EN			0x18
#define  TPn_SMBUS_MODE_EN(n)				BIT(n)

#define I3C_HUB_LDO_AND_PULLUP_CONF			0x19
#define  CP0_LDO_EN					BIT(0)
#define  CP1_LDO_EN					BIT(1)
/*
 * I3C HUB does not provide a way to control LDO or pull-up for individual ports. It is possible
 * for group of ports TP0/TP1/TP4/TP5 and TP2/TP3/TP6/TP7.
 */
#define  TP0145_LDO_EN					BIT(2)
#define  TP2367_LDO_EN					BIT(3)
#define  TP0145_PULLUP_CONF_MASK			GENMASK(7, 6)
#define  TP0145_PULLUP_CONF(x)				(((x) << 6) & TP0145_PULLUP_CONF_MASK)
#define  TP2367_PULLUP_CONF_MASK			GENMASK(5, 4)
#define  TP2367_PULLUP_CONF(x)				(((x) << 4) & TP2367_PULLUP_CONF_MASK)
#define  PULLUP_250R					0x00
#define  PULLUP_500R					0x01
#define  PULLUP_1K					0x02
#define  PULLUP_2K					0x03

#define I3C_HUB_CP_IBI_CONF				0x1A
#define I3C_HUB_TP_IBI_CONF				0x1B
#define I3C_HUB_IBI_MDB_CUSTOM				0x1C
#define I3C_HUB_JEDEC_CONTEXT_ID			0x1D
#define I3C_HUB_TP_GPIO_MODE_EN				0x1E
#define  TPn_GPIO_MODE_EN(n)				BIT(n)

/* Device Status and IBI Registers */
#define I3C_HUB_DEV_AND_IBI_STS				0x20
#define I3C_HUB_TP_SMBUS_AGNT_IBI_STS			0x21

/* Use the Scratch register as SW-assigned ID */
#define I3C_HUB_ID					0x30
#define I3C_HUB_PORT_STATUS				0x31

/* Controller Port Control/Status Registers */
#define I3C_HUB_CP_MUX_SET				0x38
#define I3C_HUB_CP_MUX_STS				0x39

/* Target Ports Control Registers */
#define I3C_HUB_TP_SMBUS_AGNT_TRANS_START		0x50
#define I3C_HUB_TP_NET_CON_CONF				0x51
#define  TPn_NET_CON(n)					BIT(n)

#define I3C_HUB_TP_PULLUP_EN				0x53
#define  TPn_PULLUP_EN(n)				BIT(n)

#define I3C_HUB_TP_SCL_OUT_EN				0x54
#define I3C_HUB_TP_SDA_OUT_EN				0x55
#define I3C_HUB_TP_SCL_OUT_LEVEL			0x56
#define I3C_HUB_TP_SDA_OUT_LEVEL			0x57
#define I3C_HUB_TP_IN_DETECT_MODE_CONF			0x58
#define I3C_HUB_TP_SCL_IN_DETECT_IBI_EN			0x59
#define I3C_HUB_TP_SDA_IN_DETECT_IBI_EN			0x5A

/* Target Ports Status Registers */
#define I3C_HUB_TP_SCL_IN_LEVEL_STS			0x60
#define I3C_HUB_TP_SDA_IN_LEVEL_STS			0x61
#define I3C_HUB_TP_SCL_IN_DETECT_FLG			0x62
#define I3C_HUB_TP_SDA_IN_DETECT_FLG			0x63

/* SMBus Agent Configuration and Status Registers */
#define HUB_REG_TP_SMBUS_AGNT_STS(p)			(0x64 + (p))
#define I3C_HUB_TP0_SMBUS_AGNT_STS			0x64
#define I3C_HUB_TP1_SMBUS_AGNT_STS			0x65
#define I3C_HUB_TP2_SMBUS_AGNT_STS			0x66
#define I3C_HUB_TP3_SMBUS_AGNT_STS			0x67
#define I3C_HUB_TP4_SMBUS_AGNT_STS			0x68
#define I3C_HUB_TP5_SMBUS_AGNT_STS			0x69
#define I3C_HUB_TP6_SMBUS_AGNT_STS			0x6A
#define I3C_HUB_TP7_SMBUS_AGNT_STS			0x6B
#define I3C_HUB_ONCHIP_TD_AND_SMBUS_AGNT_CONF		0x6C
#define SMBUS_TIMEOUT_DISABLE				BIT(3)

#define HUB_REG_AGENT_CNTRL_STATUS_FINISH		1
#define HUB_REG_AGENT_CNTRL_STATUS_RX_BUF0		2
#define HUB_REG_AGENT_CNTRL_STATUS_RX_BUF1		4
#define HUB_REG_AGENT_CNTRL_STATUS_RX_BUF_OVF		8

/* page numbers, per port */
#define HUB_PAGE_AGENT_RX_BUF(p, n)	(16 + (4 * p) + 2 + n)

/* Special Function Registers */
#define I3C_HUB_LDO_AND_CPSEL_STS			0x79
#define  CP_SDA1_LEVEL					BIT(7)
#define  CP_SCL1_LEVEL					BIT(6)
#define  CP_SEL_PIN_INPUT_CODE_MASK			GENMASK(5, 4)
#define  CP_SEL_PIN_INPUT_CODE_GET(x)			(((x) & CP_SEL_PIN_INPUT_CODE_MASK) >> 4)
#define  CP_SDA1_SCL1_PINS_CODE_MASK			GENMASK(7, 6)
#define  CP_SDA1_SCL1_PINS_CODE_GET(x)			(((x) & CP_SDA1_SCL1_PINS_CODE_MASK) >> 6)

#define I3C_HUB_BUS_RESET_SCL_TIMEOUT			0x7A
#define I3C_HUB_ONCHIP_TD_PROTO_ERR_FLG			0x7B
#define I3C_HUB_DEV_CMD					0x7C
#define I3C_HUB_ONCHIP_TD_STS				0x7D
#define I3C_HUB_ONCHIP_TD_ADDR_CONF			0x7E
#define I3C_HUB_PAGE_PTR				0x7F

/* LDO DT settings */
#define I3C_HUB_DT_LDO_DISABLED				0x00
#define I3C_HUB_DT_LDO_1_0V				0x01
#define I3C_HUB_DT_LDO_1_1V				0x02
#define I3C_HUB_DT_LDO_1_2V				0x03
#define I3C_HUB_DT_LDO_1_8V				0x04
#define I3C_HUB_DT_LDO_NOT_DEFINED			0xFF

/* VIO Source settings */
#define I3C_HUB_DT_VIO_SOURCE_INTERNAL			0x00
#define I3C_HUB_DT_VIO_SOURCE_EXTERNAL			0x01

/* Pull-up DT settings */
#define I3C_HUB_DT_PULLUP_DISABLED			0x00
#define I3C_HUB_DT_PULLUP_250R				0x01
#define I3C_HUB_DT_PULLUP_500R				0x02
#define I3C_HUB_DT_PULLUP_1K				0x03
#define I3C_HUB_DT_PULLUP_2K				0x04
#define I3C_HUB_DT_PULLUP_NOT_DEFINED			0xFF

/* TP DT setting */
#define I3C_HUB_DT_TP_MODE_DISABLED			0x00
#define I3C_HUB_DT_TP_MODE_I3C				0x01
#define I3C_HUB_DT_TP_MODE_I3C_PERF			0x02
#define I3C_HUB_DT_TP_MODE_SMBUS			0x03
#define I3C_HUB_DT_TP_MODE_GPIO				0x04
#define I3C_HUB_DT_TP_MODE_NOT_DEFINED			0xFF

/* TP pull-up status */
#define I3C_HUB_DT_TP_PULLUP_DISABLED			0x00
#define I3C_HUB_DT_TP_PULLUP_ENABLED			0x01
#define I3C_HUB_DT_TP_PULLUP_NOT_DEFINED		0xFF

/* CP/TP IO strength */
#define I3C_HUB_DT_IO_STRENGTH_20_OHM			0x00
#define I3C_HUB_DT_IO_STRENGTH_30_OHM			0x01
#define I3C_HUB_DT_IO_STRENGTH_40_OHM			0x02
#define I3C_HUB_DT_IO_STRENGTH_50_OHM			0x03
#define I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED		0xFF

/* TP connection setting */
#define I3C_HUB_DT_TP_CONNECT_ENABLED			0x00
#define I3C_HUB_DT_TP_CONNECT_DISABLED			0x01

/* Analog switch setting */
#define I3C_HUB_DT_ANALOG_SWITCH_DISABLED		0x00
#define I3C_HUB_DT_ANALOG_SWITCH_ENABLED		0x01
#define ANALOG_SWITCH_EN				BIT(0)

/* Paged Transaction Registers */
#define I3C_HUB_CONTROLLER_BUFFER_PAGE			0x10
#define I3C_HUB_CONTROLLER_AGENT_BUFF			0x80
#define I3C_HUB_CONTROLLER_AGENT_BUFF_DATA		0x84
#define I3C_HUB_TARGET_BUFF_LENGTH			0x80

/* Transaction status checking mask */
#define I3C_HUB_XFER_SUCCESS				0x01
#define I3C_HUB_SMBUS_MASTER_STATUS_MASK		0xF1
#define I3C_HUB_TP_BUFFER_STATUS_MASK			0xFF
#define I3C_HUB_TP_TRANSACTION_CODE_MASK		0xF0

/* SMBus transaction types fields */
#define I3C_HUB_SMBUS_400kHz				BIT(2)

/* Hub buffer size */
#define I3C_HUB_CONTROLLER_BUFFER_SIZE			88
#define I3C_HUB_SMBUS_DESCRIPTOR_SIZE			4
#define I3C_HUB_SMBUS_PAYLOAD_SIZE			84

struct tp_setting {
	u8 mode;
	u8 pullup_en;
	u8 connect;
};

struct dt_settings {
	u8 cp0_ldo;
	u8 cp1_ldo;
	u8 tp0145_ldo;
	u8 tp2367_ldo;
	u8 cp0_vio_source;
	u8 cp1_vio_source;
	u8 tp0145_vio_source;
	u8 tp2367_vio_source;
	u8 tp0145_pullup;
	u8 tp2367_pullup;
	u8 cp0_io_strength;
	u8 cp1_io_strength;
	u8 tp0145_io_strength;
	u8 tp2367_io_strength;
	struct tp_setting tp[I3C_HUB_TP_MAX_COUNT];
};

struct smbus_device {
	struct i2c_client *client;
	struct list_head list;
};

struct smbus_agent {
	struct i2c_adapter adap;
	struct list_head devs; /* i2c device list */
	void *hub;
	struct completion completion;

	u32 port_id;
	/* target handling */
	struct i2c_client *client;
	u8 target_rx_buf[I3C_HUB_CONTROLLER_BUFFER_SIZE];
	int next_buf_idx;

	u8 tx_res;
};

struct i3c_hub_ibi_payload {
	u8 dev_port_status;
	u8 target_agent_status;
} __packed;

struct i3c_hub_agent_tx_hdr {
	u8 addr_rnw;
	u8 type;
	u8 wr_len;
	u8 rd_len;
};

struct i3c_hub_agent_rx_hdr {
	u8 len;
	u8 addr;
};

struct i3c_hub {
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	struct dt_settings settings;
	int hub_pin_sel_id;
	int hub_pin_cp1_id;
	int hub_sw_id;

	/* Offset for reading HUB's register. */
	u8 reg_addr;
	struct dentry *debug_dir;
	struct delayed_work delayed_work;
	struct device_node *node;
	struct device_node *child_nodes[I3C_HUB_TP_MAX_COUNT];
	struct smbus_agent agents[I3C_HUB_TP_MAX_COUNT];

	/* Element of hubdevs */
	struct list_head list;

	bool ibi_enabled;
	unsigned int cur_page;
	/* protects page access */
	struct mutex lock;
	/* Sequential execution of IBI handler*/
	struct mutex ibi_lock;
};

/* List of i3c_hub devices */
static LIST_HEAD(hubdevs);
static DEFINE_MUTEX(hubdevs_lock);

static struct workqueue_struct *i3chub_wq;

struct hub_setting {
	const char * const name;
	const u8 value;
};

static const struct hub_setting ldo_settings[] = {
	{"disabled",	I3C_HUB_DT_LDO_DISABLED},
	{"1.0V",	I3C_HUB_DT_LDO_1_0V},
	{"1.1V",	I3C_HUB_DT_LDO_1_1V},
	{"1.2V",	I3C_HUB_DT_LDO_1_2V},
	{"1.8V",	I3C_HUB_DT_LDO_1_8V},
};

static const struct hub_setting vio_source_settings[] = {
	{"internal",	I3C_HUB_DT_VIO_SOURCE_INTERNAL},
	{"external",	I3C_HUB_DT_VIO_SOURCE_EXTERNAL},
};

static const struct hub_setting pullup_settings[] = {
	{"disabled",	I3C_HUB_DT_PULLUP_DISABLED},
	{"250R",	I3C_HUB_DT_PULLUP_250R},
	{"500R",	I3C_HUB_DT_PULLUP_500R},
	{"1k",		I3C_HUB_DT_PULLUP_1K},
	{"2k",		I3C_HUB_DT_PULLUP_2K},
};

static const struct hub_setting tp_mode_settings[] = {
	{"disabled",	I3C_HUB_DT_TP_MODE_DISABLED},
	{"i3c",		I3C_HUB_DT_TP_MODE_I3C},
	{"i3c-perf",	I3C_HUB_DT_TP_MODE_I3C_PERF},
	{"smbus",	I3C_HUB_DT_TP_MODE_SMBUS},
	{"gpio",	I3C_HUB_DT_TP_MODE_GPIO},
};

static const struct hub_setting tp_pullup_settings[] = {
	{"disabled",	I3C_HUB_DT_TP_PULLUP_DISABLED},
	{"enabled",	I3C_HUB_DT_TP_PULLUP_ENABLED},
};

static const struct hub_setting io_strength_settings[] = {
	{ "20Ohms",	I3C_HUB_DT_IO_STRENGTH_20_OHM },
	{ "30Ohms",	I3C_HUB_DT_IO_STRENGTH_30_OHM },
	{ "40Ohms",	I3C_HUB_DT_IO_STRENGTH_40_OHM },
	{ "50Ohms",	I3C_HUB_DT_IO_STRENGTH_50_OHM },
};
static const struct hub_setting tp_connect_settings[] = {
	{"disabled",	I3C_HUB_DT_TP_CONNECT_DISABLED},
	{"enabled",	I3C_HUB_DT_TP_CONNECT_ENABLED},
};
static const struct hub_setting analog_switch_settings[] = {
	{"disabled",	I3C_HUB_DT_ANALOG_SWITCH_DISABLED},
	{"enabled",	I3C_HUB_DT_ANALOG_SWITCH_ENABLED},
};

static u8 i3c_hub_ldo_dt_to_reg(u8 dt_value)
{
	switch (dt_value) {
	case I3C_HUB_DT_LDO_1_1V:
		return LDO_VOLTAGE_1_1V;
	case I3C_HUB_DT_LDO_1_2V:
		return LDO_VOLTAGE_1_2V;
	case I3C_HUB_DT_LDO_1_8V:
		return LDO_VOLTAGE_1_8V;
	default:
		return LDO_VOLTAGE_1_0V;
	}
}

static u8 i3c_hub_pullup_dt_to_reg(u8 dt_value)
{
	switch (dt_value) {
	case I3C_HUB_DT_PULLUP_250R:
		return PULLUP_250R;
	case I3C_HUB_DT_PULLUP_500R:
		return PULLUP_500R;
	case I3C_HUB_DT_PULLUP_1K:
		return PULLUP_1K;
	default:
		return PULLUP_2K;
	}
}

static u8 i3c_hub_io_strength_dt_to_reg(u8 dt_value)
{
	switch (dt_value) {
	case I3C_HUB_DT_IO_STRENGTH_50_OHM:
		return IO_STRENGTH_50_OHM;
	case I3C_HUB_DT_IO_STRENGTH_40_OHM:
		return IO_STRENGTH_40_OHM;
	case I3C_HUB_DT_IO_STRENGTH_30_OHM:
		return IO_STRENGTH_30_OHM;
	default:
		return IO_STRENGTH_20_OHM;
	}
}

static int i3c_hub_of_get_setting(const struct device_node *node, const char *setting_name,
				  const struct hub_setting settings[], const u8 settings_count,
				  u8 *setting_value)
{
	const char *sval;
	int ret;
	int i;

	ret = of_property_read_string(node, setting_name, &sval);
	if (ret)
		return ret;

	for (i = 0; i < settings_count; ++i) {
		const struct hub_setting * const setting = &settings[i];

		if (!strcmp(setting->name, sval)) {
			*setting_value = setting->value;
			return 0;
		}
	}

	return -EINVAL;
}

static void i3c_hub_tp_of_get_setting(struct device *dev, const struct device_node *node,
				      struct tp_setting tp_setting[])
{
	struct i3c_hub *hub = dev_get_drvdata(dev);
	struct device_node *tp_node;
	int id;

	for_each_available_child_of_node(node, tp_node) {
		int ret;

		if (!tp_node->name || of_node_cmp(tp_node->name, "target-port"))
			continue;

		if (!tp_node->full_name ||
		    (sscanf(tp_node->full_name, "target-port@%i", &id) != 1)) {
			dev_warn(dev, "Invalid target port node found in DT - %s\n",
				 tp_node->full_name);
			continue;
		}

		if (id >= I3C_HUB_TP_MAX_COUNT) {
			dev_warn(dev, "Invalid target port index found in DT - %i\n", id);
			continue;
		}
		ret = i3c_hub_of_get_setting(tp_node, "mode", tp_mode_settings,
					     ARRAY_SIZE(tp_mode_settings), &tp_setting[id].mode);
		if (ret)
			dev_warn(dev, "Invalid or not specified setting for target port[%i].mode\n",
				 id);

		ret = i3c_hub_of_get_setting(tp_node, "pullup", tp_pullup_settings,
					     ARRAY_SIZE(tp_pullup_settings),
					     &tp_setting[id].pullup_en);
		if (ret)
			dev_warn(dev,
				 "Invalid or not specified setting for target port[%i].pullup\n",
				 id);

		/* TP connect default enabled */
		tp_setting[id].connect = I3C_HUB_DT_TP_CONNECT_ENABLED;
		i3c_hub_of_get_setting(tp_node, "connect", tp_connect_settings,
				       ARRAY_SIZE(tp_connect_settings),
				       &tp_setting[id].connect);

		/* Save the device node */
		hub->child_nodes[id] = tp_node;
	}
}

static void i3c_hub_of_get_configuration(struct device *dev, const struct device_node *node)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	int ret;

	ret = i3c_hub_of_get_setting(node, "cp0-ldo", ldo_settings, ARRAY_SIZE(ldo_settings),
				     &priv->settings.cp0_ldo);
	if (ret)
		dev_warn(dev, "Invalid or not specified setting for cp0-ldo\n");

	ret = i3c_hub_of_get_setting(node, "cp1-ldo", ldo_settings, ARRAY_SIZE(ldo_settings),
				     &priv->settings.cp1_ldo);
	if (ret)
		dev_warn(dev, "Invalid or not specified setting for cp1-ldo\n");

	ret = i3c_hub_of_get_setting(node, "tp0145-ldo", ldo_settings, ARRAY_SIZE(ldo_settings),
				     &priv->settings.tp0145_ldo);
	if (ret)
		dev_warn(dev, "Invalid or not specified setting for tp0145-ldo\n");

	ret = i3c_hub_of_get_setting(node, "tp2367-ldo", ldo_settings, ARRAY_SIZE(ldo_settings),
				     &priv->settings.tp2367_ldo);
	if (ret)
		dev_warn(dev, "Invalid or not specified setting for tp2367-ldo\n");

	ret = i3c_hub_of_get_setting(node, "tp0145-pullup", pullup_settings,
				     ARRAY_SIZE(pullup_settings), &priv->settings.tp0145_pullup);
	if (ret)
		dev_warn(dev, "Invalid or not specified setting for tp0145-pullup\n");

	ret = i3c_hub_of_get_setting(node, "tp2367-pullup", pullup_settings,
				     ARRAY_SIZE(pullup_settings), &priv->settings.tp2367_pullup);
	if (ret)
		dev_warn(dev, "Invalid or not specified setting for tp2367-pullup\n");

	i3c_hub_of_get_setting(node, "cp0-vio-source", vio_source_settings,
			       ARRAY_SIZE(vio_source_settings), &priv->settings.cp0_vio_source);
	i3c_hub_of_get_setting(node, "cp1-vio-source", vio_source_settings,
			       ARRAY_SIZE(vio_source_settings), &priv->settings.cp1_vio_source);
	i3c_hub_of_get_setting(node, "tp0145-vio-source", vio_source_settings,
			       ARRAY_SIZE(vio_source_settings), &priv->settings.tp0145_vio_source);
	i3c_hub_of_get_setting(node, "tp2367-vio-source", vio_source_settings,
			       ARRAY_SIZE(vio_source_settings), &priv->settings.tp2367_vio_source);

	i3c_hub_of_get_setting(node, "cp0-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings), &priv->settings.cp0_io_strength);
	i3c_hub_of_get_setting(node, "cp1-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings), &priv->settings.cp1_io_strength);
	i3c_hub_of_get_setting(node, "tp0145-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings), &priv->settings.tp0145_io_strength);
	i3c_hub_of_get_setting(node, "tp2367-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings), &priv->settings.tp2367_io_strength);

	i3c_hub_tp_of_get_setting(dev, node, priv->settings.tp);
}

static void i3c_hub_of_default_configuration(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	int id;

	priv->settings.cp0_ldo = I3C_HUB_DT_LDO_NOT_DEFINED;
	priv->settings.cp1_ldo = I3C_HUB_DT_LDO_NOT_DEFINED;
	priv->settings.tp0145_ldo = I3C_HUB_DT_LDO_NOT_DEFINED;
	priv->settings.tp2367_ldo = I3C_HUB_DT_LDO_NOT_DEFINED;
	priv->settings.cp0_vio_source = I3C_HUB_DT_VIO_SOURCE_INTERNAL;
	priv->settings.cp1_vio_source = I3C_HUB_DT_VIO_SOURCE_INTERNAL;
	priv->settings.tp0145_vio_source = I3C_HUB_DT_VIO_SOURCE_INTERNAL;
	priv->settings.tp2367_vio_source = I3C_HUB_DT_VIO_SOURCE_INTERNAL;
	priv->settings.tp0145_pullup = I3C_HUB_DT_PULLUP_NOT_DEFINED;
	priv->settings.tp2367_pullup = I3C_HUB_DT_PULLUP_NOT_DEFINED;
	priv->settings.cp0_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;
	priv->settings.cp1_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;
	priv->settings.tp0145_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;
	priv->settings.tp2367_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;

	for (id = 0; id < I3C_HUB_TP_MAX_COUNT; ++id) {
		priv->settings.tp[id].mode = I3C_HUB_DT_TP_MODE_NOT_DEFINED;
		priv->settings.tp[id].pullup_en = I3C_HUB_DT_TP_PULLUP_NOT_DEFINED;
	}
}

static int i3c_hub_hw_configure_pullup(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 mask = 0, value = 0;

	if (priv->settings.tp0145_pullup != I3C_HUB_DT_PULLUP_NOT_DEFINED) {
		mask |= TP0145_PULLUP_CONF_MASK;
		value |= TP0145_PULLUP_CONF(i3c_hub_pullup_dt_to_reg(priv->settings.tp0145_pullup));
	}

	if (priv->settings.tp2367_pullup != I3C_HUB_DT_PULLUP_NOT_DEFINED) {
		mask |= TP2367_PULLUP_CONF_MASK;
		value |= TP2367_PULLUP_CONF(i3c_hub_pullup_dt_to_reg(priv->settings.tp2367_pullup));
	}

	return regmap_update_bits(priv->regmap, I3C_HUB_LDO_AND_PULLUP_CONF, mask, value);
}

static int i3c_hub_hw_configure_ldo(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 mask_all = 0, val_all = 0;
	u8 ldo_dis = 0, ldo_en = 0;
	u32 reg_val;
	u8 val;
	int ret;

	/* Get LDOs configuration to figure out what is going to be changed */
	ret = regmap_read(priv->regmap, I3C_HUB_LDO_CONF, &reg_val);
	if (ret)
		return ret;

	if (priv->settings.cp0_ldo != I3C_HUB_DT_LDO_NOT_DEFINED) {
		val = CP0_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.cp0_ldo));
		if ((reg_val & CP0_LDO_VOLTAGE_MASK) != val)
			ldo_dis |= CP0_LDO_EN;
		if (priv->settings.cp0_ldo != I3C_HUB_DT_LDO_DISABLED)
			ldo_en |= CP0_LDO_EN;
		mask_all |= CP0_LDO_VOLTAGE_MASK;
		val_all |= val;
	}
	if (priv->settings.cp1_ldo != I3C_HUB_DT_LDO_NOT_DEFINED) {
		val = CP1_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.cp1_ldo));
		if ((reg_val & CP1_LDO_VOLTAGE_MASK) != val)
			ldo_dis |= CP1_LDO_EN;
		if (priv->settings.cp1_ldo != I3C_HUB_DT_LDO_DISABLED)
			ldo_en |= CP1_LDO_EN;
		mask_all |= CP1_LDO_VOLTAGE_MASK;
		val_all |= val;
	}
	if (priv->settings.tp0145_ldo != I3C_HUB_DT_LDO_NOT_DEFINED) {
		val = TP0145_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.tp0145_ldo));
		if ((reg_val & TP0145_LDO_VOLTAGE_MASK) != val)
			ldo_dis |= TP0145_LDO_EN;
		if (priv->settings.tp0145_ldo != I3C_HUB_DT_LDO_DISABLED)
			ldo_en |= TP0145_LDO_EN;
		mask_all |= TP0145_LDO_VOLTAGE_MASK;
		val_all |= val;
	}
	if (priv->settings.tp2367_ldo != I3C_HUB_DT_LDO_NOT_DEFINED) {
		val = TP2367_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.tp2367_ldo));
		if ((reg_val & TP2367_LDO_VOLTAGE_MASK) != val)
			ldo_dis |= TP2367_LDO_EN;
		if (priv->settings.tp2367_ldo != I3C_HUB_DT_LDO_DISABLED)
			ldo_en |= TP2367_LDO_EN;
		mask_all |= TP2367_LDO_VOLTAGE_MASK;
		val_all |= val;
	}

	/* Disalbe LDO if using external LDO */
	if (priv->settings.cp0_vio_source == I3C_HUB_DT_VIO_SOURCE_EXTERNAL) {
		ldo_dis |= CP0_LDO_EN;
		ldo_en &= ~CP0_LDO_EN;
	}
	if (priv->settings.cp1_vio_source == I3C_HUB_DT_VIO_SOURCE_EXTERNAL) {
		ldo_dis |= CP1_LDO_EN;
		ldo_en &= ~CP1_LDO_EN;
	}
	if (priv->settings.tp0145_vio_source == I3C_HUB_DT_VIO_SOURCE_EXTERNAL) {
		ldo_dis |= TP0145_LDO_EN;
		ldo_en &= ~TP0145_LDO_EN;
	}
	if (priv->settings.tp2367_vio_source == I3C_HUB_DT_VIO_SOURCE_EXTERNAL) {
		ldo_dis |= TP2367_LDO_EN;
		ldo_en &= ~TP2367_LDO_EN;
	}

	/* Disable all LDOs if LDO configuration is going to be changed. */
	ret = regmap_update_bits(priv->regmap, I3C_HUB_LDO_AND_PULLUP_CONF, ldo_dis, 0);
	if (ret)
		return ret;

	/* Set LDOs configuration */
	ret = regmap_update_bits(priv->regmap, I3C_HUB_LDO_CONF, mask_all, val_all);
	if (ret)
		return ret;

	/* Re-enable LDOs if needed */
	return regmap_update_bits(priv->regmap, I3C_HUB_LDO_AND_PULLUP_CONF, ldo_en, ldo_en);
}

static int i3c_hub_hw_configure_io_strength(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 mask_all = 0, val_all = 0;
	u32 reg_val;
	u8 val;
	struct dt_settings tmp;
	int ret;

	/* Get IO strength configuration to figure out what needs to be changed */
	ret = regmap_read(priv->regmap, I3C_HUB_IO_STRENGTH, &reg_val);
	if (ret)
		return ret;

	tmp = priv->settings;
	if (tmp.cp0_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = CP0_IO_STRENGTH(i3c_hub_io_strength_dt_to_reg(tmp.cp0_io_strength));
		mask_all |= CP0_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (tmp.cp1_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = CP1_IO_STRENGTH(i3c_hub_io_strength_dt_to_reg(tmp.cp1_io_strength));
		mask_all |= CP1_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (tmp.tp0145_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = TP0145_IO_STRENGTH(i3c_hub_io_strength_dt_to_reg(tmp.tp0145_io_strength));
		mask_all |= TP0145_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (tmp.tp2367_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = TP2367_IO_STRENGTH(i3c_hub_io_strength_dt_to_reg(tmp.tp2367_io_strength));
		mask_all |= TP2367_IO_STRENGTH_MASK;
		val_all |= val;
	}

	/* Set IO strength if required */
	return regmap_update_bits(priv->regmap, I3C_HUB_IO_STRENGTH, mask_all, val_all);
}

static int i3c_hub_hw_configure_tp(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 pullup_mask = 0, pullup_val = 0;
	u8 smbus_mask = 0, smbus_val = 0;
	u8 gpio_mask = 0, gpio_val = 0;
	u8 i3c_mask = 0, i3c_val = 0;
	int ret;
	int i;

	/* TBD: Read type of HUB from register I3C_HUB_DEV_INFO_0 to learn target ports count. */
	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		if (priv->settings.tp[i].mode != I3C_HUB_DT_TP_MODE_NOT_DEFINED) {
			i3c_mask |= TPn_NET_CON(i);
			smbus_mask |= TPn_SMBUS_MODE_EN(i);
			gpio_mask |= TPn_GPIO_MODE_EN(i);

			if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_I3C)
				i3c_val |= TPn_NET_CON(i);
			else if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_SMBUS)
				smbus_val |= TPn_SMBUS_MODE_EN(i);
			else if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_GPIO)
				gpio_val |= TPn_GPIO_MODE_EN(i);
		}
		if (priv->settings.tp[i].pullup_en != I3C_HUB_DT_TP_PULLUP_NOT_DEFINED) {
			pullup_mask |= TPn_PULLUP_EN(i);
			if (priv->settings.tp[i].pullup_en == I3C_HUB_DT_TP_PULLUP_ENABLED)
				pullup_val |= TPn_PULLUP_EN(i);
		}
	}

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_NET_CON_CONF, i3c_mask, i3c_val);
	if (ret)
		return ret;

	/* Set Open-Drain / Push-Pull compatible for I3C/GPIO mode */
	ret = regmap_clear_bits(priv->regmap, I3C_HUB_TP_IO_MODE_CONF, i3c_val | gpio_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_SMBUS_AGNT_EN, smbus_mask, smbus_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_GPIO_MODE_EN, gpio_mask, gpio_val);
	if (ret)
		return ret;

	/* Enable TP here in case TP was configured */
	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_ENABLE, i3c_mask | smbus_mask | gpio_mask,
				 i3c_val | smbus_val | gpio_val);
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, I3C_HUB_TP_PULLUP_EN, pullup_mask, pullup_val);
}

static int i3c_hub_configure_hw(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 setting;
	int ret;

	ret = i3c_hub_hw_configure_pullup(dev);
	if (ret)
		return ret;

	ret = i3c_hub_hw_configure_ldo(dev);
	if (ret)
		return ret;

	ret = i3c_hub_hw_configure_io_strength(dev);
	if (ret)
		return ret;

	setting = I3C_HUB_DT_ANALOG_SWITCH_DISABLED;
	i3c_hub_of_get_setting(priv->node, "analog-switch", analog_switch_settings,
			       ARRAY_SIZE(analog_switch_settings),
			       &setting);
	if (setting == I3C_HUB_DT_ANALOG_SWITCH_ENABLED) {
		ret = regmap_update_bits(priv->regmap, I3C_HUB_NET_OPER_MODE_CONF, ANALOG_SWITCH_EN, ANALOG_SWITCH_EN);
		if (ret)
			return ret;
	}

	if (priv->node && of_property_read_bool(priv->node, "smbus-timeout-disable")) {
		if (regmap_update_bits(priv->regmap, I3C_HUB_ONCHIP_TD_AND_SMBUS_AGNT_CONF,
				       SMBUS_TIMEOUT_DISABLE, SMBUS_TIMEOUT_DISABLE))
			dev_err(dev, "Failed to disable SMBus timeout\n");
	}

	return i3c_hub_hw_configure_tp(dev);
}

static const struct i3c_device_id i3c_hub_ids[] = {
	I3C_CLASS(I3C_DCR_HUB, NULL),
	{ },
};

static int i3c_hub_read_id(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u32 reg_val;
	int ret;

	ret = regmap_read(priv->regmap, I3C_HUB_LDO_AND_CPSEL_STS, &reg_val);
	if (ret) {
		dev_err(dev, "Failed to read status register\n");
		return -1;
	}

	priv->hub_pin_sel_id = CP_SEL_PIN_INPUT_CODE_GET(reg_val);
	priv->hub_pin_cp1_id = CP_SDA1_SCL1_PINS_CODE_GET(reg_val);
	if (regmap_read(priv->regmap, I3C_HUB_ID, &priv->hub_sw_id))
		priv->hub_sw_id = -1;

	return 0;
}

static struct device_node *i3c_hub_get_dt_hub_node(struct device *dev, struct i3c_hub *priv)
{
	struct device_node *parent_node = dev->parent->of_node;
	struct device_node *matched_node = NULL;
	struct device_node *hub_node;
	int id_matched = 0;
	int hub_dt_sel_id;
	int hub_dt_cp1_id;
	int hub_dt_sw_id;
	int matched;

	/*
	 * HW ID definition:
	 * CP SEL pin:
	 *  - Low:		"id" = 0
	 *  - Floating:		"id" = 1
	 *  - High:		"id" = 3
	 *
	 * CP1 SDA/SCL pins:
	 *  - SDA=0,SCL=0:	"id-cp1" = 0
	 *  - SDA=0,SCL=1:	"id-cp1" = 1
	 *  - SDA=1,SCL=0:	"id-cp1" = 2
	 *  - SDA=1,SCL=1:	"id-cp1" = 3
	 */
	dev_info(dev, "Hub ID: sel=%d, cp1=%d, sw=%d\n", priv->hub_pin_sel_id,
		 priv->hub_pin_cp1_id, priv->hub_sw_id);

	for_each_available_child_of_node(parent_node, hub_node) {
		if (!of_node_name_eq(hub_node, "hub"))
			continue;

		matched = 0;

		/* Match optional "id" property */
		if (!of_property_read_u32(hub_node, "id", &hub_dt_sel_id)) {
			dev_dbg(dev, "DT node '%s': id=%u\n",
				of_node_full_name(hub_node), hub_dt_sel_id);

			if (hub_dt_sel_id != priv->hub_pin_sel_id)
				continue;

			matched++;
		}

		/* Match optional "id-cp1" property */
		if (!of_property_read_u32(hub_node, "id-cp1", &hub_dt_cp1_id)) {
			dev_dbg(dev, "DT node '%s': id-cp1=%u\n",
				of_node_full_name(hub_node), hub_dt_cp1_id);

			if (hub_dt_cp1_id != priv->hub_pin_cp1_id)
				continue;

			matched++;
		}

		/* Match optional "id-sw" property */
		if (!of_property_read_u32(hub_node, "id-sw", &hub_dt_sw_id)) {
			dev_dbg(dev, "DT node '%s': id-sw=%u\n",
				of_node_full_name(hub_node), hub_dt_sw_id);

			if (hub_dt_sw_id != priv->hub_sw_id)
				continue;

			matched++;
		}
		/*
		 * Selection policy:
		 * - Prefer more matches.
		 * - If no candidate, use the first "hub" node.
		 */
		if (!matched_node || matched > id_matched) {
			dev_dbg(dev, "DT %s selected (matched=%d)\n",
				of_node_full_name(hub_node), matched);
			id_matched = matched;
			if (matched_node)
				of_node_put(matched_node);
			matched_node = of_node_get(hub_node);
		}
	}

	return matched_node;
}

static int fops_access_reg_get(void *ctx, u64 *val)
{
	struct i3c_hub *priv = ctx;
	u32 reg_val;
	int ret;

	ret = regmap_read(priv->regmap, priv->reg_addr, &reg_val);
	if (ret)
		return ret;

	*val = reg_val & 0xFF;
	return 0;
}

static int fops_access_reg_set(void *ctx, u64 val)
{
	struct i3c_hub *priv = ctx;

	return regmap_write(priv->regmap, priv->reg_addr, val & 0xFF);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_access_reg, fops_access_reg_get, fops_access_reg_set, "0x%llX\n");

static int i3c_hub_debugfs_init(struct i3c_hub *priv, const char *hub_id)
{
	struct dentry  *entry, *dt_conf_dir, *reg_dir;
	int i;

	entry = debugfs_create_dir(hub_id, NULL);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	priv->debug_dir = entry;

	entry = debugfs_create_dir("dt-conf", priv->debug_dir);
	if (IS_ERR(entry))
		goto err_remove;

	dt_conf_dir = entry;

	debugfs_create_u8("cp0-ldo", 0400, dt_conf_dir, &priv->settings.cp0_ldo);
	debugfs_create_u8("cp1-ldo", 0400, dt_conf_dir, &priv->settings.cp1_ldo);
	debugfs_create_u8("tp0145-ldo", 0400, dt_conf_dir, &priv->settings.tp0145_ldo);
	debugfs_create_u8("tp2367-ldo", 0400, dt_conf_dir, &priv->settings.tp2367_ldo);
	debugfs_create_u8("tp0145-pullup", 0400, dt_conf_dir, &priv->settings.tp0145_pullup);
	debugfs_create_u8("tp2367-pullup", 0400, dt_conf_dir, &priv->settings.tp2367_pullup);

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		char file_name[32];

		sprintf(file_name, "tp%i.mode", i);
		debugfs_create_u8(file_name, 0400, dt_conf_dir, &priv->settings.tp[i].mode);
		sprintf(file_name, "tp%i.pullup_en", i);
		debugfs_create_u8(file_name, 0400, dt_conf_dir, &priv->settings.tp[i].pullup_en);
	}

	entry = debugfs_create_dir("reg", priv->debug_dir);
	if (IS_ERR(entry))
		goto err_remove;

	reg_dir = entry;

	entry = debugfs_create_file_unsafe("access", 0600, reg_dir, priv, &fops_access_reg);
	if (IS_ERR(entry))
		goto err_remove;

	debugfs_create_u8("offset", 0600, reg_dir, &priv->reg_addr);

	return 0;

err_remove:
	debugfs_remove_recursive(priv->debug_dir);
	return PTR_ERR(entry);
}

static ssize_t tp_connect_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u32 reg_val;
	int ret;
	ret = regmap_read(priv->regmap, I3C_HUB_TP_NET_CON_CONF, &reg_val);
	if (ret)
		return -EIO;

	return sprintf(buf, "%02x", reg_val & 0xFF);
}

static ssize_t tp_connect_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 tp_en;
	int ret;

	if (kstrtou8(buf, 0, &tp_en))
		return -EINVAL;

	/* Unlock access to protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_UNLOCK_CODE);
	if (ret)
		return -EIO;

	ret = regmap_write(priv->regmap, I3C_HUB_TP_NET_CON_CONF, tp_en);
	if (ret)
		return -EIO;

	/* Lock access to protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_LOCK_CODE);
	if (ret)
		return -EIO;

	return count;
}
static DEVICE_ATTR_RW(tp_connect);

static int i3c_hub_connect_tp(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u32 tp_dis_val = 0;
	int i;

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i)
		if (priv->settings.tp[i].connect == I3C_HUB_DT_TP_CONNECT_DISABLED)
			tp_dis_val |= TPn_NET_CON(i);

	return regmap_clear_bits(priv->regmap, I3C_HUB_TP_NET_CON_CONF, tp_dis_val);
}

static int i3c_hub_write_paged(struct i3c_hub *hub, unsigned int page,
			       unsigned int addr, const void *data, size_t size)
{
	int ret;

	mutex_lock(&hub->lock);

	if (hub->cur_page != page) {
		ret = regmap_write(hub->regmap, I3C_HUB_PAGE_PTR, page);
		if (ret)
			goto exit_unlock;
		hub->cur_page = page;
	}

	ret = regmap_bulk_write(hub->regmap, 128 + addr, data, size);

exit_unlock:
	mutex_unlock(&hub->lock);

	return ret;
}

static int i3c_hub_read_paged(struct i3c_hub *hub, unsigned int page,
			      unsigned int addr, void *data, size_t size)
{
	int ret;

	mutex_lock(&hub->lock);

	if (hub->cur_page != page) {
		ret = regmap_write(hub->regmap, I3C_HUB_PAGE_PTR, page);
		if (ret)
			goto exit_unlock;
		hub->cur_page = page;
	}

	ret = regmap_bulk_read(hub->regmap, 128 + addr, data, size);

exit_unlock:
	mutex_unlock(&hub->lock);

	return ret;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static void i3c_hub_slave_agent_rx(struct smbus_agent *agent, int buf_idx)
{
	struct i3c_hub_agent_rx_hdr hdr;
	struct i3c_hub *hub = agent->hub;
	u8 tmp, len, addr;
	unsigned int i, page;
	int ret;

	if (!agent->client)
		goto ack;

	/* Switch to RX BUF page */
	page = HUB_PAGE_AGENT_RX_BUF(agent->port_id, buf_idx);

	/* We need the length to figure out the size of our read. But we also
	 * read the first byte of i2c data in the same read; the hardware has
	 * no facility for filtering on incoming local addresses, so we have a
	 * fast-path to aborting the transaction if it's not targeted to us.
	 */
	ret = i3c_hub_read_paged(hub, page, 0, &hdr, sizeof(hdr));
	if (ret)
		goto ack;

	len = min_t(u8, hdr.len, I3C_HUB_TARGET_BUFF_LENGTH);
	if (len == 0)
		goto ack;

	if (hdr.addr & 0x1) {
		dev_dbg(&hub->i3cdev->dev, "unsupported read requested\n");
		goto ack;
	}

	/* not for us? discard and ack */
	addr = hdr.addr >> 1;
	if (addr != (agent->client->addr & 0x7f))
		goto ack;

	memset(agent->target_rx_buf, 0, sizeof(agent->target_rx_buf));
	ret = i3c_hub_read_paged(hub, page, 2, agent->target_rx_buf, len - 1);
	if (ret)
		goto ack;

	/* synthesize i2c target events from the target write */
	tmp = 0;
	ret = i2c_slave_event(agent->client, I2C_SLAVE_WRITE_REQUESTED, &tmp);
	if (ret)
		goto stop;

	/* len includes the address byte, which we have already read */
	for (i = 0; i < len - 1; i++) {
		tmp = agent->target_rx_buf[i];
		i2c_slave_event(agent->client, I2C_SLAVE_WRITE_RECEIVED, &tmp);
	}

stop:
	tmp = 0;
	i2c_slave_event(agent->client, I2C_SLAVE_STOP, &tmp);

ack:
	tmp = buf_idx ? HUB_REG_AGENT_CNTRL_STATUS_RX_BUF1 :
		HUB_REG_AGENT_CNTRL_STATUS_RX_BUF0;

	if (regmap_write(hub->regmap, HUB_REG_TP_SMBUS_AGNT_STS(agent->port_id), tmp))
		dev_warn(&hub->i3cdev->dev, "TP[%d]: Failed to clear RX status: %d\n",
			 agent->port_id, ret);
	agent->next_buf_idx = !agent->next_buf_idx;
}
#endif

static void i3c_hub_agent_ibi(struct smbus_agent *agent)
{
	struct i3c_hub *hub = agent->hub;
	unsigned int stat = 0;
	int ret;

	/* Read SMBus agent status */
	ret = regmap_read(hub->regmap,
			  HUB_REG_TP_SMBUS_AGNT_STS(agent->port_id), &stat);
	if (ret) {
		dev_err(&hub->i3cdev->dev,
			"TP[%d] - failed to read agent status\n", agent->port_id);
		return;
	}

	/* Master Agent IBI */
	if (stat & HUB_REG_AGENT_CNTRL_STATUS_FINISH) {
		/* Clear Master Agent Finish flag */
		ret = regmap_write(hub->regmap,
				   HUB_REG_TP_SMBUS_AGNT_STS(agent->port_id),
				   HUB_REG_AGENT_CNTRL_STATUS_FINISH);
		if (ret)
			dev_warn(&hub->i3cdev->dev,
				 "TP[%d] - failed to clear finish status\n", agent->port_id);
		agent->tx_res = stat;
		complete(&agent->completion);
	}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	/* Slave Agent IBI */
	if (stat & (HUB_REG_AGENT_CNTRL_STATUS_RX_BUF0 | HUB_REG_AGENT_CNTRL_STATUS_RX_BUF1)) {
		if (agent->next_buf_idx == 0) {
			/* Check BUF0 first */
			if (stat & HUB_REG_AGENT_CNTRL_STATUS_RX_BUF0)
				i3c_hub_slave_agent_rx(agent, 0);

			if (stat & HUB_REG_AGENT_CNTRL_STATUS_RX_BUF1)
				i3c_hub_slave_agent_rx(agent, 1);
		} else {
			/* Check BUF1 first */
			if (stat & HUB_REG_AGENT_CNTRL_STATUS_RX_BUF1)
				i3c_hub_slave_agent_rx(agent, 1);
			if (stat & HUB_REG_AGENT_CNTRL_STATUS_RX_BUF0)
				i3c_hub_slave_agent_rx(agent, 0);
		}
	}

	if (stat & HUB_REG_AGENT_CNTRL_STATUS_RX_BUF_OVF) {
		dev_info(&agent->adap.dev, "rx overflow\n");
		ret = regmap_write(hub->regmap,
				   HUB_REG_TP_SMBUS_AGNT_STS(agent->port_id),
				   HUB_REG_AGENT_CNTRL_STATUS_RX_BUF_OVF);
		if (ret)
			dev_warn(&hub->i3cdev->dev,
				 "Port[%d] - failed to clear rx overflow status\n", agent->port_id);
	}
#endif
}

static void i3c_hub_ibi(struct i3c_device *i3c,
			const struct i3c_ibi_payload *payload)
{
	struct i3c_hub *hub = i3cdev_get_drvdata(i3c);
	const struct i3c_hub_ibi_payload *p = NULL;
	int i;

	mutex_lock(&hub->ibi_lock);

	if (payload->len == sizeof(*p))
		p = payload->data;

	if (!p)
		goto exit;

	/* Clear MsgPending/ParityErr/PecErr flags */
	if (p->dev_port_status & 0x07)
		regmap_write(hub->regmap, I3C_HUB_DEV_AND_IBI_STS, 0x07);

	/* Check SMBus agent event status */
	if ((p->dev_port_status & 0x10) == 0)
		goto exit;

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		if ((hub->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_SMBUS) &&
		    (p->target_agent_status & BIT(i))) {
			dev_dbg(&hub->i3cdev->dev, "target port %d generates ibi\n", i);
			i3c_hub_agent_ibi(&hub->agents[i]);
		}
	}
exit:
	mutex_unlock(&hub->ibi_lock);
}

static int i3c_hub_reset_smbus_agent(struct smbus_agent *agent)
{
	struct i3c_hub *hub = agent->hub;
	int ret;

	/* Unlock register access */
	regmap_write(hub->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_UNLOCK_CODE);

	/* Disable Agent */
	ret = regmap_update_bits(hub->regmap, I3C_HUB_TP_SMBUS_AGNT_EN, BIT(agent->port_id), 0);
	if (ret)
		goto err_exit;

	/* Enable Agent */
	ret = regmap_update_bits(hub->regmap, I3C_HUB_TP_SMBUS_AGNT_EN, BIT(agent->port_id),
				 BIT(agent->port_id));
	if (ret)
		goto err_exit;

err_exit:
	if (ret)
		dev_err(&hub->i3cdev->dev, "Failed to reset smbus agent:%d\n", ret);

	/* Lock register access */
	regmap_write(hub->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_LOCK_CODE);

	return ret;
}

static int i3c_hub_smbus_xfer_one(struct i2c_adapter *adap, struct i2c_msg *wr_msg,
				  struct i2c_msg *rd_msg)
{
	struct smbus_agent *agent = adap->algo_data;
	struct i3c_hub *hub = agent->hub;
	int port_id = agent->port_id;
	int ret;

	u8 desc[I3C_HUB_SMBUS_DESCRIPTOR_SIZE] = { 0 };
	u8 *out = NULL, *in = NULL;
	u32 wr_len = 0, rd_len = 0;
	u8 page = I3C_HUB_CONTROLLER_BUFFER_PAGE + 4 * port_id;
	u8 reg_status = HUB_REG_TP_SMBUS_AGNT_STS(port_id);

	/*
	 * The len is only 1 in the smbus block read transfer, need to
	 * extend the read length.
	 */
	if (rd_msg && (rd_msg->flags & I2C_M_RECV_LEN))
		rd_msg->len += I2C_SMBUS_BLOCK_MAX;

	if (wr_msg && rd_msg) {
		if (wr_msg->addr != rd_msg->addr) {
			dev_err(&adap->dev, "different addr in i2c wr and rd msgs\n");
			return -EINVAL;
		}
		desc[0] = wr_msg->addr << 1;
		desc[1] = I3C_HUB_SMBUS_400kHz | BIT(0); /* A write followed by a read*/
		desc[2] = wr_len = wr_msg->len;
		desc[3] = rd_len = rd_msg->len;
		out = wr_msg->buf;
		in = rd_msg->buf;
	} else if (wr_msg) {
		desc[0] = wr_msg->addr << 1;
		desc[1] = I3C_HUB_SMBUS_400kHz;
		desc[2] = wr_len = wr_msg->len;
		desc[3] = 0;
		out = wr_msg->buf;
	} else if (rd_msg) {
		desc[0] = rd_msg->addr << 1 | BIT(0);
		desc[1] = I3C_HUB_SMBUS_400kHz;
		desc[2] = 0;
		desc[3] = rd_len = rd_msg->len;
		in = rd_msg->buf;
	}

	if (wr_len + rd_len > I3C_HUB_SMBUS_PAYLOAD_SIZE) {
		dev_err(&adap->dev, "Message length too long.\n");
		return -EINVAL;
	}

	/* Fill descriptor */
	ret = i3c_hub_write_paged(hub, page, 0, desc, I3C_HUB_SMBUS_DESCRIPTOR_SIZE);
	if (ret) {
		dev_err(&adap->dev, "Write descriptor failed %d\n", ret);
		return ret;
	}
	if (wr_msg && wr_len) {
		/* Fill payload for write */
		ret = i3c_hub_write_paged(hub, page, 4, out, wr_len);
		if (ret) {
			dev_err(&adap->dev, "write data failed %d\n", ret);
			return ret;
		}
	}

	reinit_completion(&agent->completion);
	/* Clear master agent status */
	regmap_write(hub->regmap, reg_status, I3C_HUB_SMBUS_MASTER_STATUS_MASK);

	/* Start the transaction */
	ret = regmap_write(hub->regmap, I3C_HUB_TP_SMBUS_AGNT_TRANS_START, BIT(port_id));
	if (ret)
		return ret;

	if (wait_for_completion_timeout(&agent->completion, agent->adap.timeout) < 0) {
		dev_info(&adap->dev, "wait_for_complete timeout\n");
		i3c_hub_reset_smbus_agent(agent);
		ret = -ETIMEDOUT;
		return ret;
	}

	if ((agent->tx_res & I3C_HUB_SMBUS_MASTER_STATUS_MASK) != I3C_HUB_XFER_SUCCESS) {
		dev_dbg(&adap->dev, "TX error: status = 0x%x\n", agent->tx_res);
		ret = -EIO;
		return ret;
	}

	if (rd_msg) {
		/* Read the data of read transaction */
		ret = i3c_hub_read_paged(hub, page, 4 + wr_len,
					 in, rd_len);
		/* Update the actual read length for smbus block read */
		if (rd_msg->flags & I2C_M_RECV_LEN)
			rd_msg->len = min_t(unsigned int, in[0], I2C_SMBUS_BLOCK_MAX) + 1;
	}

	return ret;
}

static int i3c_hub_smbus_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			      int num)
{
	struct i2c_msg *wr_msg, *rd_msg;
	int i = 0;
	int ret;

	while (i < num) {
		if (!(msgs[i].flags & I2C_M_RD)) {
			wr_msg = &msgs[i++];
			rd_msg = NULL;
			/* If a read msg followed by write msg is to the same address, combine it*/
			if (i < num && msgs[i].addr == wr_msg->addr &&
			    (msgs[i].flags & I2C_M_RD)) {
				rd_msg = &msgs[i++];
			}
		} else {
			wr_msg = NULL;
			rd_msg = &msgs[i++];
		}

		ret = i3c_hub_smbus_xfer_one(adap, wr_msg, rd_msg);
		if (ret)
			return ret;
	}

	return num;
}

static u32 i3c_hub_i2c_funcs(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_I2C | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static int i3c_hub_agent_i2c_reg_target(struct i2c_client *client)
{
	struct smbus_agent *agent = i2c_get_adapdata(client->adapter);

	if (agent->client)
		return -EBUSY;

	agent->client = client;

	return 0;
}

static int i3c_hub_agent_i2c_unreg_target(struct i2c_client *client)
{
	struct smbus_agent *agent = i2c_get_adapdata(client->adapter);

	agent->client = NULL;

	return 0;
}

static const struct i2c_algorithm i3c_hub_i2c_algo = {
	.master_xfer = i3c_hub_smbus_xfer,
	.functionality = i3c_hub_i2c_funcs,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave = i3c_hub_agent_i2c_reg_target,
	.unreg_slave = i3c_hub_agent_i2c_unreg_target,
#endif
};

static const struct i3c_ibi_setup i3c_hub_ibi_setup = {
	.max_payload_len = 2, /* no MDB, two status registers */
	.num_slots = 6, /* two target buffers, one controller status */
	.handler = i3c_hub_ibi,
};

static int i3c_hub_i2c_write(struct i2c_adapter *i2c_adap, u8 addr, u8 reg, u8 val)
{
	struct i2c_msg xfer;
	char buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;
	xfer.addr = addr;
	xfer.flags = 0;
	xfer.len = 2;
	xfer.buf = (void *)buf;
	ret = i2c_transfer(i2c_adap, &xfer, 1);
	dev_dbg(&i2c_adap->dev, "%s: addr 0x%x reg 0x%x val 0x%x ret %d\n",
		__func__, addr, reg, val, ret);
	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int i3c_hub_register_i2c_devices(struct i3c_hub *hub, int port)
{
	struct i2c_adapter *adap;
	struct device_node *tp_node, *child;
	struct i2c_board_info info;
	struct smbus_device *device;
	int ret;

	tp_node = hub->child_nodes[port];
	adap = &hub->agents[port].adap;
	for_each_child_of_node(tp_node, child) {
		u32 assigned_id;

		dev_dbg(&adap->dev, "of_i2c: register %pOF\n", child);

		if (of_device_is_compatible(child, "i3c-hub") &&
		    !of_property_read_u32(child, "assigned-id", &assigned_id)) {
			u32 addr;

			ret = of_property_read_u32(child, "reg", &addr);
			if (ret) {
				dev_warn(&adap->dev, "%pOF missing reg, skip assigned-id\n", child);
				continue;
			}

			dev_info(&adap->dev, "assigned id %d to downstream hub\n", assigned_id);
			i3c_hub_i2c_write(adap, addr & 0x7f, I3C_HUB_ID, assigned_id);
			continue;
		}

		ret = of_i2c_get_board_info(&adap->dev, child, &info);
		if (ret)
			continue;

		device = kzalloc(sizeof(*device), GFP_KERNEL);
		if (!device)
			continue;

		device->client = i2c_new_client_device(adap, &info);
		if (IS_ERR(device->client)) {
			dev_err(&adap->dev, "of_i2c: Failure registering %pOF\n", child);
			kfree(device);
			continue;
		}
		list_add_tail(&device->list, &hub->agents[port].devs);
	}
	return 0;
}

static int i3c_hub_add_smbus_adapter(struct i3c_hub *hub, int port)
{
	struct device *dev = &hub->i3cdev->dev;
	struct i3c_device *i3cdev = hub->i3cdev;
	struct i2c_adapter *adap;
	int ret, id = -ENODEV;
	int i = port;

	/* Disconnect slave port from hub network */
	ret = regmap_update_bits(hub->regmap, I3C_HUB_TP_NET_CON_CONF, BIT(i), 0);
	if (ret)
		return ret;

	/* Unlock access to protected registers */
	ret = regmap_write(hub->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_UNLOCK_CODE);
	if (ret) {
		dev_err(dev, "Failed to unlock HUB's protected registers\n");
		return ret;
	}
	/* Disable TP */
	ret = regmap_clear_bits(hub->regmap, I3C_HUB_TP_ENABLE, BIT(i));
	if (ret)
		return ret;

	/* Clear Agent flags */
	ret = regmap_write(hub->regmap, HUB_REG_TP_SMBUS_AGNT_STS(i), 0x0F);
	if (ret)
		return ret;

	/* Set OD-Only */
	ret = regmap_set_bits(hub->regmap, I3C_HUB_TP_IO_MODE_CONF, BIT(i));
	if (ret)
		return ret;
	/* Enable agent IBI */
	ret = regmap_update_bits(hub->regmap, I3C_HUB_TP_IBI_CONF, BIT(i), BIT(i));
	if (ret)
		return ret;

	/* Enable TP */
	ret = regmap_set_bits(hub->regmap, I3C_HUB_TP_ENABLE, BIT(i));
	if (ret)
		return ret;

	init_completion(&hub->agents[i].completion);
	INIT_LIST_HEAD(&hub->agents[i].devs);
	hub->agents[i].hub = hub;
	hub->agents[i].port_id = i;
	adap = &hub->agents[i].adap;
	adap->dev.parent = dev->parent;
	adap->owner = THIS_MODULE;
	adap->algo = &i3c_hub_i2c_algo;
	adap->algo_data = &hub->agents[i];
	adap->timeout = 1000;
	adap->retries = 3;
	snprintf(adap->name, sizeof(adap->name), "hub%s.port%d",
		 dev_name(&hub->i3cdev->dev), i);

	i2c_set_adapdata(adap, &hub->agents[i]);
	id = of_alias_get_id(hub->child_nodes[i], "i2c");
	if (id >= 0) {
		adap->nr = id;
		ret = i2c_add_numbered_adapter(adap);
	} else {
		ret = i2c_add_adapter(adap);
	}
	if (ret < 0) {
		dev_err(dev, "failed to add i2c-adapter %u (error=%d)\n", i, ret);
		regmap_update_bits(hub->regmap, I3C_HUB_TP_IBI_CONF, BIT(i), 0);
	}

	ret = regmap_write(hub->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_LOCK_CODE);
	if (ret)
		dev_err(dev, "Failed to lock HUB's protected registers\n");

	if (!hub->ibi_enabled) {
		ret = i3c_device_request_ibi(i3cdev, &i3c_hub_ibi_setup);
		if (ret) {
			dev_err(&i3cdev->dev, "Failed requesting IBI\n");
			return ret;
		}
		ret = i3c_device_enable_ibi(i3cdev);
		if (ret) {
			i3c_device_free_ibi(i3cdev);
			dev_err(&i3cdev->dev, "Failed enabling IBI\n");
			return ret;
		}
		hub->ibi_enabled = true;
	}

	i3c_hub_register_i2c_devices(hub, port);

	return ret;
}

static void i3c_hub_del_smbus_adapter(struct i3c_hub *hub)
{
	struct smbus_device *dev;
	bool use_ibi = false;
	int i;

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		if (hub->settings.tp[i].mode != I3C_HUB_DT_TP_MODE_SMBUS)
			continue;

		list_for_each_entry(dev, &hub->agents[i].devs, list) {
			i2c_unregister_device(dev->client);
			kfree(dev);
		}
		i2c_del_adapter(&hub->agents[i].adap);
		use_ibi = true;
	}
}

static void i3c_hub_init_gpio(struct i3c_hub *hub, int port)
{
	struct device_node *tp_node = hub->child_nodes[port];
	u32 scl, sda;
	int ret;

	ret = of_property_read_u32(tp_node, "scl-output", &scl);
	if (!ret) {
		regmap_update_bits(hub->regmap, I3C_HUB_TP_SCL_OUT_LEVEL, (1 << port), (scl << port));
		regmap_update_bits(hub->regmap, I3C_HUB_TP_SCL_OUT_EN, (1 << port), (1 << port));
	}

	ret = of_property_read_u32(tp_node, "sda-output", &sda);
	if (!ret) {
		regmap_update_bits(hub->regmap, I3C_HUB_TP_SDA_OUT_LEVEL, (1 << port), (sda << port));
		regmap_update_bits(hub->regmap, I3C_HUB_TP_SDA_OUT_EN, (1 << port), (1 << port));
	}
}

static int i3c_hub_assign_id(struct i3c_hub *hub, int port, u8 addr, u32 hub_id)
{
	struct i3c_master_controller *master = hub->i3cdev->desc->common.master;
	struct i2c_adapter *i2c_adap = &master->i2c;
	u32 port_status = 0;
	int tp_save;
	int ret;

	/* Check if the downstream hub is already configured */
	ret = regmap_read(hub->regmap, I3C_HUB_PORT_STATUS, &port_status);
	if (ret)
		return ret;
	if (port_status & (1 << port)) {
		dev_dbg(&hub->i3cdev->dev, "downstream port%d already configured\n", port);
		return -EEXIST;
	}

	/* Unlock access to protected registers */
	ret = regmap_write(hub->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_UNLOCK_CODE);
	if (ret)
		return -EIO;

	/* Save the hub network connection state */
	regmap_read(hub->regmap, I3C_HUB_TP_NET_CON_CONF, &tp_save);
	regmap_write(hub->regmap, I3C_HUB_TP_NET_CON_CONF, 0);

	/* Set master port OD-only */
	regmap_update_bits(hub->regmap, I3C_HUB_CP_CONF, BIT(4), BIT(4));

	/* Set slave port OD-only */
	regmap_update_bits(hub->regmap, I3C_HUB_TP_IO_MODE_CONF, (1 << port), (1 << port));

	/* Enable the connection only to the slave port where the Hub is attached */
	regmap_write(hub->regmap, I3C_HUB_TP_NET_CON_CONF, (1 << port));

	ret = i3c_hub_i2c_write(i2c_adap, addr, I3C_HUB_ID, hub_id);
	if (!ret) {
		/* Update the downstream port configured status */
		port_status |= 1 << port;
		regmap_write(hub->regmap, I3C_HUB_PORT_STATUS, port_status);
	}

	/* Restore master/slave port mode */
	regmap_update_bits(hub->regmap, I3C_HUB_CP_CONF, BIT(4), 0);
	regmap_update_bits(hub->regmap, I3C_HUB_TP_IO_MODE_CONF, (1 << port), 0);

	/* Restore the Hub netowrk connection state */
	regmap_write(hub->regmap, I3C_HUB_TP_NET_CON_CONF, tp_save);

	/* Lock access to protected registers */
	regmap_write(hub->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_LOCK_CODE);

	return ret;
}

static int i3c_hub_init_downstream_hub(struct i3c_hub *hub, int port)
{
	struct i3c_master_controller *master = hub->i3cdev->desc->common.master;
	struct device_node *child, *tp_node = hub->child_nodes[port];
	struct i2c_adapter *adap = &master->i2c;
	struct i2c_board_info info;
	struct i2c_client *i2cdev;
	u32 assigned_id, addr;
	int ret;

	for_each_child_of_node(tp_node, child) {
		if (!of_device_is_compatible(child, "i3c-hub"))
			continue;

		if (of_property_read_u32(child, "assigned-id", &assigned_id))
			continue;

		ret = of_property_read_u32(child, "reg", &addr);
		if (ret)
			continue;

		ret = of_i2c_get_board_info(&adap->dev, child, &info);
		if (ret)
			continue;

		i2cdev = i2c_new_client_device(adap, &info);
		if (IS_ERR(i2cdev)) {
			dev_err(&adap->dev, "of_i2c: Failure registering %pOF\n", child);
			continue;
		}

		ret = i3c_hub_assign_id(hub, port, addr & 0x7f, assigned_id);
		if (!ret)
			dev_info(&hub->i3cdev->dev, "Assigned id %d to hub at port%d\n",
				 assigned_id, port);
		else if (ret != -EEXIST)
			dev_err(&hub->i3cdev->dev, "Fail to assigned id %d to hub at port%d\n",
				 assigned_id, port);

		i2c_unregister_device(i2cdev);
	}
	return 0;
}

static int i3c_hub_setup_child_nodes(struct i3c_hub *hub)
{
	int i;

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		if (hub->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_SMBUS)
			i3c_hub_add_smbus_adapter(hub, i);
		else if (hub->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_GPIO)
			i3c_hub_init_gpio(hub, i);
		else if (hub->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_I3C)
			i3c_hub_init_downstream_hub(hub, i);
	}

	return 0;
}

static void i3c_hub_delayed_work(struct work_struct *work)
{
	struct i3c_hub *priv = container_of(to_delayed_work(work), struct i3c_hub, delayed_work);
	struct i3c_master_controller *master = priv->i3cdev->desc->common.master;

	i3c_hub_setup_child_nodes(priv);

	if (priv->node && of_property_read_bool(priv->node, "do-setdasa"))
		i3c_master_do_setdasa(master);

	if (priv->node && of_property_read_bool(priv->node, "do-setaasa")) {
		i3c_device_send_ccc_cmd(priv->i3cdev, I3C_CCC_SETHID);
		i3c_device_send_ccc_cmd(priv->i3cdev, I3C_CCC_SETAASA);
	}

	if (priv->node && of_property_read_bool(priv->node, "do-entdaa"))
		i3c_master_do_daa(master);
}

static int i3c_hub_probe(struct i3c_device *i3cdev)
{
	struct regmap_config i3c_hub_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	struct device *dev = &i3cdev->dev;
	struct device_node *node;
	struct regmap *regmap;
	struct i3c_hub *priv;
	char hub_id[32];
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i3cdev = i3cdev;
	i3cdev_set_drvdata(i3cdev, priv);

	sprintf(hub_id, "i3c-hub-%d-%llx", i3cdev->bus->id, i3cdev->desc->info.pid);
	ret = i3c_hub_debugfs_init(priv, hub_id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialized DebugFS.\n");

	i3c_hub_of_default_configuration(dev);

	regmap = devm_regmap_init_i3c(i3cdev, &i3c_hub_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to register I3C HUB regmap\n");
		goto error;
	}

	priv->regmap = regmap;
	mutex_init(&priv->lock);
	mutex_init(&priv->ibi_lock);

	ret = regmap_write(priv->regmap, I3C_HUB_CP_MUX_SET, BIT(0));
	if (ret) {
		dev_err(dev, "Failed to request hub control\n");
		goto error;
	}

	if (dev->of_node) {
		node = of_node_get(dev->of_node);
	} else {
		/* Find the hub node by hub_id */
		ret = i3c_hub_read_id(dev);
		if (ret)
			goto error;

		if (priv->hub_pin_cp1_id >= 0 && priv->hub_pin_sel_id >= 0)
			/* Find hub node in DT matching HW ID or just first without ID provided in DT */
			node = i3c_hub_get_dt_hub_node(dev, priv);

	}
	if (!node) {
		dev_warn(dev, "Failed to find DT entry for the driver. Running with defaults.\n");
	} else {
		dev_info(dev, "Use %s DT node\n", of_node_full_name(node));
		i3c_hub_of_get_configuration(dev, node);
		priv->node = node;
	}

	/* Unlock access to protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_UNLOCK_CODE);
	if (ret) {
		dev_err(dev, "Failed to unlock HUB's protected registers\n");
		goto error;
	}

	ret = i3c_hub_configure_hw(dev);
	if (ret) {
		dev_err(dev, "Failed to configure the HUB\n");
		goto error;
	}

	/* Setup hub network connection according to dts setting */
	i3c_hub_connect_tp(dev);

	/* Lock access to protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_LOCK_CODE);
	if (ret) {
		dev_err(dev, "Failed to lock HUB's protected registers\n");
		goto error;
	}

	ret = sysfs_create_file(&dev->kobj,
				&dev_attr_tp_connect.attr);
	if (ret) {
		dev_err(dev, "Failed to create tp_connect sysfs file\n");
		goto error;
	}

	mutex_lock(&hubdevs_lock);
	list_add(&priv->list, &hubdevs);
	mutex_unlock(&hubdevs_lock);
	INIT_DELAYED_WORK(&priv->delayed_work, i3c_hub_delayed_work);
	queue_delayed_work(i3chub_wq, &priv->delayed_work, msecs_to_jiffies(100));
	/* TBD: Apply special/security lock here using DEV_CMD register */

	return 0;

error:
	debugfs_remove_recursive(priv->debug_dir);
	return ret;
}

static void i3c_hub_remove(struct i3c_device *i3cdev)
{
	struct i3c_hub *priv = i3cdev_get_drvdata(i3cdev);
	struct i3c_dev_desc *desc = priv->i3cdev->desc;

	mutex_lock(&hubdevs_lock);
	list_del(&priv->list);
	if (priv->ibi_enabled) {
		i3c_device_disable_ibi(priv->i3cdev);
		if (desc && desc->ibi)
			desc->ibi->enabled = false;
		i3c_device_free_ibi(priv->i3cdev);
		priv->ibi_enabled = false;
	}
	mutex_unlock(&hubdevs_lock);
	i3c_hub_del_smbus_adapter(priv);
	debugfs_remove_recursive(priv->debug_dir);
	sysfs_remove_file(&i3cdev->dev.kobj, &dev_attr_tp_connect.attr);
	if (priv->node)
		of_node_put(priv->node);
}

static struct i3c_driver i3c_hub = {
	.driver.name = "i3c-hub",
	.id_table = i3c_hub_ids,
	.probe = i3c_hub_probe,
	.remove = i3c_hub_remove,
};

static void i3c_hub_notify_bus_remove(struct i3c_bus *bus)
{
	struct i3c_hub *hub = NULL, *tmp;
	struct i3c_dev_desc *desc;

	mutex_lock(&hubdevs_lock);
	list_for_each_entry_safe(hub, tmp, &hubdevs, list) {
		if (hub->i3cdev->bus == bus && hub->ibi_enabled) {
			i3c_device_disable_ibi(hub->i3cdev);
			desc = hub->i3cdev->desc;
			if (desc && desc->ibi)
				desc->ibi->enabled = false;
			i3c_device_free_ibi(hub->i3cdev);
			hub->ibi_enabled = false;
		}
	}
	mutex_unlock(&hubdevs_lock);
}

static int i3c_hub_notifier_call(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	switch (action) {
	case I3C_NOTIFY_BUS_REMOVE:
		i3c_hub_notify_bus_remove((struct i3c_bus *)data);
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block i3c_hub_notifier = {
	.notifier_call = i3c_hub_notifier_call,
};

static __init int i3c_hub_init(void)
{
	int rc;

	i3chub_wq = alloc_ordered_workqueue("i3chub", 0);
	if (!i3chub_wq)
		return -ENOMEM;

	i3c_register_notifier(&i3c_hub_notifier);

	rc = i3c_driver_register(&i3c_hub);
	if (rc < 0)
		return rc;

	return 0;
}

static __exit void i3c_hub_exit(void)
{
	i3c_driver_unregister(&i3c_hub);

	i3c_unregister_notifier(&i3c_hub_notifier);

	destroy_workqueue(i3chub_wq);
}

module_init(i3c_hub_init);
module_exit(i3c_hub_exit);

MODULE_AUTHOR("Zbigniew Lukwinski <zbigniew.lukwinski@linux.intel.com>");
MODULE_DESCRIPTION("I3C HUB driver");
MODULE_LICENSE("GPL");
