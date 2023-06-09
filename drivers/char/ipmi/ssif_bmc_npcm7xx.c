// SPDX-License-Identifier: GPL-2.0+
/*
 * The driver for BMC side of NPCM7xx SSIF interface
 *
 * Copyright (c) 2021, Ampere Computing LLC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/iopoll.h>

#include "ssif_bmc.h"

/* Common regs */
 #define NPCM_I2CCTL1            0x06
 #define NPCM_I2CADDR1            0x08
 #define NPCM_I2CCTL2            0x0A
 #define NPCM_I2CADDR2            0x0C
 #define NPCM_I2CCTL3            0x0E
 #define I2C_VER                0x1F

 /* NPCM_I2CCTL3 reg fields */
 #define NPCM_I2CCTL3_ARPMEN        BIT(2)
 #define NPCM_I2CCTL3_BNK_SEL        BIT(5)

 /* NPCM_I2CCTL1 reg fields */
 #define NPCM_I2CCTL1_START        BIT(0)
 #define NPCM_I2CCTL1_STOP        BIT(1)
 #define NPCM_I2CCTL1_INTEN        BIT(2)
 #define NPCM_I2CCTL1_EOBINTE        BIT(3)
 #define NPCM_I2CCTL1_ACK        BIT(4)
 #define NPCM_I2CCTL1_GCMEN        BIT(5)
 #define NPCM_I2CCTL1_NMINTE        BIT(6)
 #define NPCM_I2CCTL1_STASTRE        BIT(7)

 #define I2C_HW_FIFO_SIZE        16

 #define NPCM_I2CADDR_SAEN        BIT(7)

 /* RW1S fields (inside a RW reg): */
#define NPCM_I2CCTL1_RWS   \
	(NPCM_I2CCTL1_START | NPCM_I2CCTL1_STOP | NPCM_I2CCTL1_ACK)

#if IS_ENABLED(CONFIG_I2C_SLAVE)
/*
 * npcm_i2caddr array:
 * The module supports having multiple own slave addresses.
 * Since the addr regs are sprinkled all over the address space,
 * use this array to get the address or each register.
 */
#define I2C_NUM_OWN_ADDR 10
static const int npcm_i2caddr[I2C_NUM_OWN_ADDR] = {
	NPCM_I2CADDR1,
};
#endif

/* I2C Bank (module had 2 banks of registers) */
enum i2c_bank {
	I2C_BANK_0 = 0,
	I2C_BANK_1,
};

/* Module supports setting multiple own slave addresses */
enum i2c_addr {
	I2C_SLAVE_ADDR1 = 0,
	I2C_SLAVE_ADDR2,
	I2C_SLAVE_ADDR3,
	I2C_SLAVE_ADDR4,
	I2C_SLAVE_ADDR5,
	I2C_SLAVE_ADDR6,
	I2C_SLAVE_ADDR7,
	I2C_SLAVE_ADDR8,
	I2C_SLAVE_ADDR9,
	I2C_SLAVE_ADDR10,
	I2C_GC_ADDR,
	I2C_ARP_ADDR,
};

enum i2c_mode {
	I2C_MASTER,
	I2C_SLAVE,
};

enum i2c_state_ind {
	I2C_NO_STATUS_IND = 0,
	I2C_SLAVE_RCV_IND,
	I2C_SLAVE_XMIT_IND,
	I2C_SLAVE_XMIT_MISSING_DATA_IND,
	I2C_SLAVE_RESTART_IND,
	I2C_SLAVE_DONE_IND,
	I2C_MASTER_DONE_IND,
	I2C_NACK_IND,
	I2C_BUS_ERR_IND,
	I2C_WAKE_UP_IND,
	I2C_BLOCK_BYTES_ERR_IND,
	I2C_SLAVE_RCV_MISSING_DATA_IND,
};

/* Internal I2C states values (for the I2C module state machine). */
enum i2c_state {
	I2C_DISABLE = 0,
	I2C_IDLE,
	I2C_MASTER_START,
	I2C_SLAVE_MATCH,
	I2C_OPER_STARTED,
	I2C_STOP_PENDING,
};

/*
 * Operation type values (used to define the operation currently running)
 * module is interrupt driven, on each interrupt the current operation is
 * checked to see if the module is currently reading or writing.
 */
enum i2c_oper {
	I2C_NO_OPER = 0,
	I2C_WRITE_OPER,
	I2C_READ_OPER,
};

struct npcm_i2c_data {
	u8 fifo_size;
	u32 segctl_init_val;
	u8 txf_sts_tx_bytes;
	u8 rxf_sts_rx_bytes;
	u8 rxf_ctl_last_pec;
};

static const struct npcm_i2c_data npxm7xx_i2c_data = {
	.fifo_size = 16,
	.segctl_init_val = 0x0333F000,
	.txf_sts_tx_bytes = GENMASK(4, 0),
	.rxf_sts_rx_bytes = GENMASK(4, 0),
	.rxf_ctl_last_pec = BIT(5),
};

struct npcm_i2c {
	struct i2c_adapter	adap;
	struct device		*dev;
	unsigned char __iomem	*reg;
	const struct npcm_i2c_data *data;
	spinlock_t		lock;  /* IRQ synchronization */
	struct completion	cmd_complete;
	int			cmd_err;
	struct i2c_msg		*msgs;
	int			msgs_num;
	int			num;
	u32			apb_clk;
	struct i2c_bus_recovery_info	rinfo;
	enum i2c_state		state;
	enum i2c_oper		operation;
	enum i2c_mode		master_or_slave;
	enum i2c_state_ind	stop_ind;
	u8			dest_addr;
	u8			*rd_buf;
	u16			rd_size;
	u16			rd_ind;
	u8			*wr_buf;
	u16			wr_size;
	u16			wr_ind;
	bool			fifo_use;
	u16			PEC_mask; /* PEC bit mask per slave address */
	bool			PEC_use;
	bool			read_block_use;
	unsigned long		int_time_stamp;
	unsigned long		bus_freq; /* in Hz */
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	u8			own_slave_addr;
	struct i2c_client	*slave;
	int			slv_rd_size;
	int			slv_rd_ind;
	int			slv_wr_size;
	int			slv_wr_ind;
	u8			slv_rd_buf[I2C_HW_FIFO_SIZE];
	u8			slv_wr_buf[I2C_HW_FIFO_SIZE];
#endif
	struct dentry		*debugfs; /* debugfs device directory */
	u64			ber_cnt;
	u64			rec_succ_cnt;
	u64			rec_fail_cnt;
	u64			nack_cnt;
	u64			timeout_cnt;
	u64 tx_complete_cnt;
	bool ber_state;
};

static inline void npcm_i2c_select_bank(struct npcm_i2c *bus,
		enum i2c_bank bank)
{
	u8 i2cctl3 = ioread8(bus->reg + NPCM_I2CCTL3);

	if (bank == I2C_BANK_0)
		i2cctl3 = i2cctl3 & ~NPCM_I2CCTL3_BNK_SEL;
	else
		i2cctl3 = i2cctl3 | NPCM_I2CCTL3_BNK_SEL;

	iowrite8(i2cctl3, bus->reg + NPCM_I2CCTL3);
}

static void npcm_i2c_slave_int_enable(struct npcm_i2c *bus, bool enable)
{
	u8 i2cctl1;

	/* enable interrupt on slave match: */
	i2cctl1 = ioread8(bus->reg + NPCM_I2CCTL1);
	i2cctl1 &= ~NPCM_I2CCTL1_RWS;
	if (enable)
		i2cctl1 |= NPCM_I2CCTL1_NMINTE;
	else
		i2cctl1 &= ~NPCM_I2CCTL1_NMINTE;

	iowrite8(i2cctl1, bus->reg + NPCM_I2CCTL1);
}

static int npcm_i2c_slave_enable(struct npcm_i2c *bus, enum i2c_addr addr_type,
		u8 addr, bool enable)
{
	u8 i2cctl1;
	u8 i2cctl3;
	u8 sa_reg;
	int ret = 0;

	sa_reg = (addr & 0x7F) | FIELD_PREP(NPCM_I2CADDR_SAEN, enable);
	if (addr_type == I2C_GC_ADDR) {
		i2cctl1 = ioread8(bus->reg + NPCM_I2CCTL1);
		if (enable)
			i2cctl1 |= NPCM_I2CCTL1_GCMEN;
		else
			i2cctl1 &= ~NPCM_I2CCTL1_GCMEN;

		iowrite8(i2cctl1, bus->reg + NPCM_I2CCTL1);
		goto out;
	}

	if (addr_type == I2C_ARP_ADDR) {
		i2cctl3 = ioread8(bus->reg + NPCM_I2CCTL3);
		if (enable)
			i2cctl3 |= NPCM_I2CCTL3_ARPMEN;
		else
			i2cctl3 &= ~NPCM_I2CCTL3_ARPMEN;
		iowrite8(i2cctl3, bus->reg + NPCM_I2CCTL3);
		goto out;
	}

	if (addr_type >= I2C_ARP_ADDR) {
		ret = -EFAULT;
		goto out;
	}

	/* select bank 0 for address 3 to 10 */
	if (addr_type > I2C_SLAVE_ADDR2)
		npcm_i2c_select_bank(bus, I2C_BANK_0);
	/* Set and enable the address */
	iowrite8(sa_reg, bus->reg + npcm_i2caddr[addr_type]);
	npcm_i2c_slave_int_enable(bus, enable);
	if (addr_type > I2C_SLAVE_ADDR2)
		npcm_i2c_select_bank(bus, I2C_BANK_1);

out:
	return 0;

}

void npcm_set_ssif_bmc_status(struct ssif_bmc_ctx *ssif_bmc, unsigned int status)
{
	struct npcm_i2c *bus;
	unsigned long flags;

	bus = (struct npcm_i2c *)ssif_bmc->priv;
	if (!bus)
		return;

	spin_lock_irqsave(&bus->lock, flags);

	if (status & SSIF_BMC_BUSY)
		npcm_i2c_slave_enable(bus, I2C_SLAVE_ADDR1, ssif_bmc->client->addr, false);
	else if (status & SSIF_BMC_READY)
		npcm_i2c_slave_enable(bus, I2C_SLAVE_ADDR1, ssif_bmc->client->addr, true);

	spin_unlock_irqrestore(&bus->lock, flags);
}

static int ssif_bmc_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ssif_bmc_ctx *ssif_bmc;

	ssif_bmc = ssif_bmc_alloc(client, sizeof(struct npcm_i2c));
	if (IS_ERR(ssif_bmc))
		return PTR_ERR(ssif_bmc);

	ssif_bmc->priv = i2c_get_adapdata(client->adapter);
	ssif_bmc->set_ssif_bmc_status = npcm_set_ssif_bmc_status;

	return 0;
}
/*
static int ssif_bmc_remove(struct i2c_client *client)
{
	struct ssif_bmc_ctx *ssif_bmc = i2c_get_clientdata(client);

	i2c_slave_unregister(client);
	misc_deregister(&ssif_bmc->miscdev);

	return 0;
}
*/
static const struct of_device_id ssif_bmc_match[] = {
	{ .compatible = "nuvoton,npcm7xx-ssif-bmc", .data = &npxm7xx_i2c_data },
	{ },
};

static const struct i2c_device_id ssif_bmc_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ssif_bmc_id);

static struct i2c_driver ssif_bmc_driver = {
	.driver		= {
		.name		= DEVICE_NAME,
		.of_match_table = ssif_bmc_match,
	},
	.probe		= ssif_bmc_probe,
	//.remove		= ssif_bmc_remove,
	.id_table	= ssif_bmc_id,
};

module_i2c_driver(ssif_bmc_driver);

MODULE_AUTHOR("Mohaimen <mohaimen.alsamarai@foxconn.com>");
MODULE_DESCRIPTION("Device driver of the Nuvoton BMC IPMI SSIF interface.");
MODULE_LICENSE("GPL v2");
