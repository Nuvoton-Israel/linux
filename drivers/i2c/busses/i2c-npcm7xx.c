/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk/nuvoton.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

static struct regmap *gcr_regmap = NULL;
static struct regmap *clk_regmap = NULL;

#define NPCM7XX_SECCNT          (0x68)
#define NPCM7XX_CNTR25M         (0x6C)

#define  I2CSEGCTL_OFFSET 0xE4
#define  I2CSEGCTL_VAL	  0x0333F000

#define ENABLE      1
#define DISABLE     0

#define _1Hz_           1UL
#define _1KHz_          (1000 * _1Hz_)
#define _1MHz_          (1000 * _1KHz_)
#define _1GHz_          (1000 * _1MHz_)

#ifndef ASSERT
#ifdef DEBUG
#define ASSERT(cond)  {if (!(cond)) for (;;) ; }           /* infinite loop                                  */
#else
#define ASSERT(cond)
#endif
#endif

#define ROUND_UP(val, n)     (((val)+(n)-1) & ~((n)-1))
#define DIV_CEILING(a, b)       (((a) + ((b)-1)) / (b))

#define I2C_VERSION "0.0.1"

//#define CONFIG_NPCM750_I2C_DEBUG
#ifdef CONFIG_NPCM750_I2C_DEBUG
#define dev_err(a, f, x...) pr_err("NPCM750-I2C: %s() dev_err:" f, __func__, \
					## x)
#define I2C_DEBUG(f, x...) pr_info("NPCM750-I2C: %s():%d " f, __func__, \
					__LINE__, ## x)
#else
#define I2C_DEBUG(f, x...)
#endif
#define HAL_PRINT(f, x...)          printk(f, ## x)


typedef struct bit_field {
	u8 offset;
	u8 size;
} bit_field_t;

#ifdef REG_READ
#undef REG_READ
#endif
static inline u8 REG_READ(unsigned char __iomem *mem)
{
	return ioread8(mem);
}

#ifdef REG_WRITE
#undef REG_WRITE
#endif
static inline void REG_WRITE(unsigned char __iomem *mem, u8 val)
{
	iowrite8(val, mem);
}

#ifdef SET_REG_FIELD
#undef SET_REG_FIELD
#endif
static inline void SET_REG_FIELD(unsigned char __iomem *mem,
				 bit_field_t bit_field, u8 val)
{
	u8 tmp = ioread8(mem);
	tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset); // mask the field size
	tmp |= val << bit_field.offset;  // or with the requested value
	iowrite8(tmp, mem);
}

#ifdef SET_VAR_FIELD
#undef SET_VAR_FIELD
#endif
// bit_field should be of bit_field_t type
#define SET_VAR_FIELD(var, bit_field, value) { \
    typeof(var) tmp = var;                 \
    tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset); /* mask the field size */ \
    tmp |= value << bit_field.offset;  /* or with the requested value */               \
    var = tmp;                                                                         \
}

#ifdef READ_REG_FIELD
#undef READ_REG_FIELD
#endif
static inline u8 READ_REG_FIELD(unsigned char __iomem *mem,
				bit_field_t bit_field)
{
	u8 tmp = ioread8(mem);
	tmp = tmp >> bit_field.offset;     // shift right the offset
	tmp &= (1 << bit_field.size) - 1;  // mask the size
	return tmp;
}

#ifdef READ_VAR_FIELD
#undef READ_VAR_FIELD
#endif
// bit_field should be of bit_field_t type
#define READ_VAR_FIELD(var, bit_field) ({ \
    typeof(var) tmp = var;           \
    tmp = tmp >> bit_field.offset;     /* shift right the offset */ \
    tmp &= (1 << bit_field.size) - 1;  /* mask the size */          \
    tmp;                                                     \
})

#ifdef MASK_FIELD
#undef MASK_FIELD
#endif
#define MASK_FIELD(bit_field) \
    (((1 << bit_field.size) - 1) << bit_field.offset) /* mask the field size */

#ifdef BUILD_FIELD_VAL
#undef BUILD_FIELD_VAL
#endif
#define BUILD_FIELD_VAL(bit_field, value)  \
    ((((1 << bit_field.size) - 1) & (value)) << bit_field.offset)


#ifdef SET_REG_MASK
#undef SET_REG_MASK
#endif
static inline void SET_REG_MASK(unsigned char __iomem *mem, u8 val)
{
	iowrite8(ioread8(mem) | val, mem);
}

#ifndef CONFIG_I2C_SLAVE
#define SMB_MASTER_ONLY
#endif

//#define SMB_CAPABILITY_WAKEUP_SUPPORT
#define SMB_CAPABILITY_FAST_MODE_SUPPORT
#define SMB_CAPABILITY_FAST_MODE_PLUS_SUPPORT
#define SMB_CAPABILITY_END_OF_BUSY_SUPPORT
#define SMB_CAPABILITY_TIMEOUT_SUPPORT

// Using SW PEC instead of HW PEC:
//#define SMB_CAPABILITY_HW_PEC_SUPPORT
//#define SMB_STALL_TIMEOUT_SUPPORT
#define SMB_RECOVERY_SUPPORT

// override issue #614:  TITLE :CP_FW: SMBus may fail to supply stop condition in Master Write operation
#define SMB_SW_BYPASS_HW_ISSUE_SMB_STOP

// if end device reads more data than avalilable, ask issuer or request for more data.
#define SMB_WRAP_AROUND_BUFFER

#define SMB_BYTES_QUICK_PROT                        0xFFFF
#define SMB_BYTES_BLOCK_PROT                        0xFFFE
#define SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER    0xFFFD

#define ARP_ADDRESS_VAL            0x61

typedef enum {
	SMB_SLAVE = 1,
	SMB_MASTER
} SMB_MODE_T;

/*
 * External SMB Interface driver states values, which indicate to the
 * upper-level layer the status of the
 * operation it initiated or wake up events from one of the buses
 */
typedef enum {
	SMB_NO_STATUS_IND,
	SMB_SLAVE_RCV_IND,
	SMB_SLAVE_XMIT_IND,
#ifdef SMB_WRAP_AROUND_BUFFER
	SMB_SLAVE_XMIT_MISSING_DATA_IND,
#endif
	SMB_SLAVE_RESTART_IND,
	SMB_SLAVE_DONE_IND,
	SMB_MASTER_DONE_IND,
	SMB_NO_DATA_IND,
	SMB_NACK_IND,
	SMB_BUS_ERR_IND,
	SMB_WAKE_UP_IND,
	SMB_MASTER_PEC_ERR_IND,
	SMB_MASTER_BLOCK_BYTES_ERR_IND,
	SMB_SLAVE_PEC_ERR_IND
} SMB_STATE_IND_T;

typedef enum {
	SMB_SLAVE_ADDR1,
	SMB_SLAVE_ADDR2,
	SMB_SLAVE_ADDR3,
	SMB_SLAVE_ADDR4,
	SMB_SLAVE_ADDR5,
	SMB_SLAVE_ADDR6,
	SMB_SLAVE_ADDR7,
	SMB_SLAVE_ADDR8,
	SMB_SLAVE_ADDR9,
	SMB_SLAVE_ADDR10,
	SMB_GC_ADDR,
	SMB_ARP_ADDR
} SMB_ADDR_T;

#ifdef SMB_CAPABILITY_FORCE_SCL_SDA
typedef enum {
	SMB_LEVEL_LOW  = 0,
	SMB_LEVEL_HIGH = 1
} SMB_LEVEL_T;
#endif // SMB_CAPABILITY_FORCE_SCL_SDA


// Common registers
#define SMBSDA(bus)               (bus->base + 0x000)
#define SMBST(bus)                (bus->base + 0x002)
#define SMBCST(bus)               (bus->base + 0x004)
#define SMBCTL1(bus)              (bus->base + 0x006)
#define SMBADDR1(bus)             (bus->base + 0x008)
#define SMBCTL2(bus)              (bus->base + 0x00A)
#define SMBADDR2(bus)             (bus->base + 0x00C)
#define SMBCTL3(bus)              (bus->base + 0x00E)
#define SMBCST2(bus)              (bus->base + 0x018)  // Control Status 2
#define SMBCST3(bus)              (bus->base + 0x019)  // Control Status 3 Register
#define SMB_VER(bus)              (bus->base + 0x01F)  // SMB Version Register

// BANK 0 registers
#define SMBADDR3(bus)             (bus->base + 0x010)
#define SMBADDR7(bus)             (bus->base + 0x011)
#define SMBADDR4(bus)             (bus->base + 0x012)
#define SMBADDR8(bus)             (bus->base + 0x013)
#define SMBADDR5(bus)             (bus->base + 0x014)
#define SMBADDR9(bus)             (bus->base + 0x015)
#define SMBADDR6(bus)             (bus->base + 0x016)
#define SMBADDR10(bus)            (bus->base + 0x017)

#define SMBADDR(bus, i)           (bus->base + 0x008 + (u32)(((int)i*4) + (((int)i < 2) ? 0 : ((int)i-2)*(-2)) + (((int)i < 6) ? 0 : (-7))))

#define SMBCTL4(bus)              (bus->base + 0x01A)
#define SMBCTL5(bus)              (bus->base + 0x01B)
#define SMBSCLLT(bus)             (bus->base + 0x01C)  // SMB SCL Low Time (Fast-Mode)
#define SMBFIF_CTL(bus)           (bus->base + 0x01D)  // FIFO Control
#define SMBSCLHT(bus)             (bus->base + 0x01E)  // SMB SCL High Time (Fast-Mode)

// BANK 1 registers
#define SMBFIF_CTS(bus)           (bus->base + 0x010)  // FIFO Control and Status
#define SMBTXF_CTL(bus)           (bus->base + 0x012)  // Tx-FIFO Control
#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)
#define SMBT_OUT(bus)             (bus->base + 0x014)  // Bus Time-Out
#endif
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
#define SMBPEC(bus)               (bus->base + 0x016)  // PEC Data
#endif
#define SMBTXF_STS(bus)           (bus->base + 0x01A)  // Tx-FIFO Status
#define SMBRXF_STS(bus)           (bus->base + 0x01C)  // Rx-FIFO Status
#define SMBRXF_CTL(bus)           (bus->base + 0x01E)  // Rx-FIFO Control

#ifdef SMB_CAPABILITY_WAKEUP_SUPPORT
#define SMB_SBD                 ((GLUE_BASE_ADDR + 0x002), GLUE_ACCESS, 8)
#define SMB_EEN                 ((GLUE_BASE_ADDR + 0x003), GLUE_ACCESS, 8)
#endif


/* SMBST register fields */
static const bit_field_t SMBST_XMIT              = { 0,  1 };
static const bit_field_t SMBST_MASTER            = { 1, 1 };
static const bit_field_t SMBST_NMATCH            = { 2, 1 };
static const bit_field_t SMBST_STASTR            = { 3, 1 };
static const bit_field_t SMBST_NEGACK            = { 4, 1 };
static const bit_field_t SMBST_BER               = { 5, 1 };
static const bit_field_t SMBST_SDAST             = { 6, 1 };
static const bit_field_t SMBST_SLVSTP            = { 7, 1 };

/* SMBCST register fields */
static const bit_field_t SMBCST_BUSY             = { 0, 1 };
static const bit_field_t SMBCST_BB               = { 1, 1 };
static const bit_field_t SMBCST_MATCH            = { 2, 1 };
static const bit_field_t SMBCST_GCMATCH          = { 3, 1 };
static const bit_field_t SMBCST_TSDA             = { 4, 1 };
static const bit_field_t SMBCST_TGSCL            = { 5, 1 };
static const bit_field_t SMBCST_MATCHAF          = { 6, 1 };
static const bit_field_t SMBCST_ARPMATCH         = { 7, 1 };

/* SMBCTL1 register fields */
static const bit_field_t SMBCTL1_START           = { 0, 1 };
static const bit_field_t SMBCTL1_STOP            = { 1, 1 };
static const bit_field_t SMBCTL1_INTEN           = { 2, 1 };
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
static const bit_field_t SMBCTL1_EOBINTE         = { 3, 1 };
#endif
static const bit_field_t SMBCTL1_ACK             = { 4, 1 };
static const bit_field_t SMBCTL1_GCMEN           = { 5, 1 };
static const bit_field_t SMBCTL1_NMINTE          = { 6, 1 };
static const bit_field_t SMBCTL1_STASTRE         = { 7, 1 };

/* SMBADDRx register fields */
static const bit_field_t SMBADDRx_ADDR           = { 0, 7 };
static const bit_field_t SMBADDRx_SAEN           = { 7, 1 };

/* SMBCTL2 register fields                                                                                 */
static const bit_field_t SMBCTL2_ENABLE          = { 0, 1 };
static const bit_field_t SMBCTL2_SCLFRQ6_0       = { 1, 7 };

/* SMBCTL3 register fields */
static const bit_field_t SMBCTL3_SCLFRQ8_7       = { 0, 2 };
static const bit_field_t SMBCTL3_ARPMEN          = { 2, 1 };
static const bit_field_t SMBCTL3_IDL_START       = { 3, 1 };
static const bit_field_t SMBCTL3_400K_MODE       = { 4, 1 };
static const bit_field_t SMBCTL3_BNK_SEL         = { 5, 1 };
static const bit_field_t SMBCTL3_SDA_LVL         = { 6, 1 };
static const bit_field_t SMBCTL3_SCL_LVL         = { 7, 1 };

/* SMBCST2 register fields */
static const bit_field_t SMBCST2_MATCHA1F        = { 0, 1 };
static const bit_field_t SMBCST2_MATCHA2F        = { 1, 1 };
static const bit_field_t SMBCST2_MATCHA3F        = { 2, 1 };
static const bit_field_t SMBCST2_MATCHA4F        = { 3, 1 };
static const bit_field_t SMBCST2_MATCHA5F        = { 4, 1 };
static const bit_field_t SMBCST2_MATCHA6F        = { 5, 1 };
static const bit_field_t SMBCST2_MATCHA7F        = { 5, 1 };
static const bit_field_t SMBCST2_INTSTS          = { 7, 1 };

/* SMBCST3 register fields */
static const bit_field_t SMBCST3_MATCHA8F        = { 0, 1 };
static const bit_field_t SMBCST3_MATCHA9F        = { 1, 1 };
static const bit_field_t SMBCST3_MATCHA10F       = { 2, 1 };
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
static const bit_field_t SMBCST3_EO_BUSY         = { 7, 1 };
#endif

/* SMBCTL4 register fields */
static const bit_field_t SMBCTL4_HLDT            = { 0, 6 };
#ifdef SMB_CAPABILITY_FORCE_SCL_SDA
static const bit_field_t SMBCTL4_LVL_WE          = { 7, 1 };
#endif

/* SMBCTL5 register fields */
static const bit_field_t SMBCTL5_DBNCT           = { 0, 4 };

/* SMBFIF_CTS register fields */
static const bit_field_t SMBFIF_CTS_RXF_TXE      = { 1, 1 };
static const bit_field_t SMBFIF_CTS_RFTE_IE      = { 3, 1 };
static const bit_field_t SMBFIF_CTS_CLR_FIFO     = { 6, 1 };
static const bit_field_t SMBFIF_CTS_SLVRSTR      = { 7, 1 };

/* SMBTXF_CTL register fields */
#ifdef SMB_CAPABILITY_32B_FIFO
static const bit_field_t SMBTXF_CTL_TX_THR       = { 0, 6 };
#else
static const bit_field_t SMBTXF_CTL_TX_THR       = { 0, 5 };
#endif
static const bit_field_t SMBTXF_CTL_THR_TXIE     = { 6, 1 };

#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)

/* SMBT_OUT register fields */
static const bit_field_t SMBT_OUT_TO_CKDIV       = { 0, 6 };
static const bit_field_t SMBT_OUT_T_OUTIE        = { 6, 1 };
static const bit_field_t SMBT_OUT_T_OUTST        = { 7, 1 };
#endif

/* SMBTXF_STS register fields  */
#ifdef SMB_CAPABILITY_32B_FIFO
static const bit_field_t SMBTXF_STS_TX_BYTES     = { 0, 6 };
#else
static const bit_field_t SMBTXF_STS_TX_BYTES     = { 0, 5 };
#endif
static const bit_field_t SMBTXF_STS_TX_THST      = { 6, 1 };

/* SMBRXF_STS register fields */
#ifdef SMB_CAPABILITY_32B_FIFO
static const bit_field_t SMBRXF_STS_RX_BYTES     = { 0, 6 };
#else
static const bit_field_t SMBRXF_STS_RX_BYTES     = { 0, 5 };
#endif
static const bit_field_t SMBRXF_STS_RX_THST      = { 6, 1 };

/* SMBFIF_CTL register fields */
static const bit_field_t SMBFIF_CTL_FIFO_EN      = { 4, 1 };

/* SMBRXF_CTL register fields */
#ifdef SMB_CAPABILITY_32B_FIFO
static const bit_field_t SMBRXF_CTL_RX_THR       = { 0, 6 };
static const bit_field_t SMBRXF_CTL_THR_RXIE     = { 6, 1 };
static const bit_field_t SMBRXF_CTL_LAST_PEC     = { 7, 1 };
#else
static const bit_field_t SMBRXF_CTL_RX_THR       = { 0, 5 };
static const bit_field_t SMBRXF_CTL_LAST_PEC     = { 5, 1 };
static const bit_field_t SMBRXF_CTL_THR_RXIE     = { 6, 1 };
#endif

/* SMB_VER register fields */
static const bit_field_t SMB_VER_VERSION         = { 0, 7 };
static const bit_field_t SMB_VER_FIFO_EN         = { 7, 1 };

#ifdef SMB_CAPABILITY_WAKEUP_SUPPORT

/* SMB_SBD register fields */
#define SMB_SBD_SMBnSBD(n)      ((n),  1)


/* SMB_EEN register fields */
#define SMB_EEN_SMBnEEN(n)      ((n),  1)
#endif // SMB_CAPABILITY_WAKEUP_SUPPORT


/* Module Dependencies */
#if defined (MIWU_MODULE_TYPE) && defined (SMB_CAPABILITY_WAKEUP_SUPPORT)
#include __MODULE_IF_HEADER_FROM_DRV(miwu)
#endif


#if defined (CLK_MODULE_TYPE)
#include __MODULE_IF_HEADER_FROM_DRV(clk)
#endif





/* TYPES & DEFINITIONS */
#ifdef SMB_SLAVE_ONLY
//The Stall Timeout feature is only relevant in Master mode.
#undef SMB_STALL_TIMEOUT_SUPPORT
#endif

#ifdef SMB_STALL_TIMEOUT_SUPPORT

/* stall/stuck timeout                                                                                     */
#define DEFAULT_STALL_COUNT         25
#endif

/* Data abort timeout                                                                                      */
#define ABORT_TIMEOUT       1000

/* SMBus spec. values in KHz                                                                               */
#define SMBUS_FREQ_MIN      10

#if defined SMB_CAPABILITY_FAST_MODE_PLUS_SUPPORT
#define SMBUS_FREQ_MAX      1000
#elif defined SMB_CAPABILITY_FAST_MODE_SUPPORT
#define SMBUS_FREQ_MAX      400
#else
#define SMBUS_FREQ_MAX      100
#endif

#define SMBUS_FREQ_100KHz   100
#define SMBUS_FREQ_400KHz   400
#define SMBUS_FREQ_1MHz     1000



/* SMBus FIFO SIZE (when FIFO hardware exist)                                                              */
#ifdef SMB_CAPABILITY_32B_FIFO
#define SMBUS_FIFO_SIZE     32
#else
#define SMBUS_FIFO_SIZE     16
#endif


/* SCLFRQ min/max field values                                                                             */
#define SCLFRQ_MIN          10
#define SCLFRQ_MAX          511

/* SCLFRQ field position                                                                                   */
static const bit_field_t SCLFRQ_0_TO_6       = { 0, 7 };
static const bit_field_t SCLFRQ_7_TO_8       = { 7, 2 };

/* SMB Maximum Retry Trials (on Bus Arbitration Loss)                                                      */
#define SMB_RETRY_MAX_COUNT     3

/* SMBus Operation type values                                                                             */
typedef enum {
	SMB_NO_OPER     = 0,
	SMB_WRITE_OPER  = 1,
	SMB_READ_OPER   = 2
} SMB_OPERATION_T;



/* SMBus Bank (FIFO mode)                                                                                  */

typedef enum {
	SMB_BANK_0  = 0,
	SMB_BANK_1  = 1
} SMB_BANK_T;

/* Internal SMBus Interface driver states values, which reflect events which occurred on the bus           */
typedef enum {
	SMB_DISABLE,
	SMB_IDLE,
	SMB_MASTER_START,
	SMB_SLAVE_MATCH,
	SMB_OPER_STARTED,
	SMB_REPEATED_START,
	SMB_STOP_PENDING
} SMB_OPERATION_STATE_T;


#define SMB_NUM_OF_ADDR                  10 // TBD move to device tree
#define SMB_FIFO(bus)                    true   /* All modules support FIFO */
void  npcm750_clk_GetTimeStamp(u32 time_quad[2]);


/* Status of one SMBus module                                                                              */
typedef struct nuvoton_i2c_bus {
	struct i2c_adapter              adap;
	struct device			*dev;
	unsigned char __iomem	        *base;
	/* Synchronizes I/O mem access to base. */
	spinlock_t			lock;
	spinlock_t			bank_lock;
	struct completion		cmd_complete;
	int				irq;
	int				cmd_err;
	struct i2c_msg                  *msgs;
	int                             msgs_num;
	int                             module__num;
	u32                             apb_clk;
#ifdef CONFIG_I2C_SLAVE
	struct i2c_client		*slave;
#endif /* CONFIG_I2C_SLAVE */

	/* Current state of SMBus */
	volatile SMB_OPERATION_STATE_T  operation_state;

	/* Type of the last SMBus operation */
	SMB_OPERATION_T        operation;

	/* Mode of operation on SMBus */
	SMB_MODE_T             master_or_slave;
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
	/* The indication to the hi level after Master Stop */
	SMB_STATE_IND_T        stop_indication;
#endif
	/* SMBus slave device's Slave Address in 8-bit format - for master transactions */
	u8                  dest_addr;

	/* Buffer where read data should be placed */
	u8 *read_data_buf;

	/* Number of bytes to be read */
	u16                 read_size;

	/* Number of bytes already read */
	u16                 read_index;

	/* Buffer with data to be written */
	u8 *write_data_buf;

	/* Number of bytes to write */
	u16                 write_size;

	/* Number of bytes already written */
	u16                 write_index;

	/* use fifo hardware or not */
	bool                fifo_use;

	/* fifo threshold size */
	u8                  threshold_fifo;

	/* PEC bit mask per slave address.
	   1: use PEC for this address,
	   0: do not use PEC for this address */
	u16                PEC_mask;

	/* Use PEC CRC  */
	bool                PEC_use;

	/* PEC CRC data */
	u8                  crc_data;

	/* Use read block */
	bool                read_block_use;

	/* Number of retries remaining */
	u8                  retry_count;

#if !defined SMB_MASTER_ONLY
	u8 SMB_CurSlaveAddr;
#endif

#ifdef SMB_STALL_TIMEOUT_SUPPORT
	u8                  stall_counter;
	u8                  stall_threshold;
#endif


// override issue #614:  TITLE :CP_FW: SMBus may fail to supply stop condition in Master Write operation. If needed : define it at hal_cfg.h
#ifdef SMB_SW_BYPASS_HW_ISSUE_SMB_STOP
	/* The indication to the hi level after Master Stop */
	u32                clk_period_us;
	u32                interrupt_time_stamp[2];
#endif
} nuvoton_i2c_bus_t;


#ifdef SMB_SW_BYPASS_HW_ISSUE_SMB_STOP

static void inline _npcm7xx_get_time_stamp(u32 time_quad[2]);
static u32  inline _npcm7xx_delay_relative(u32 microSecDelay, u32 t0_time[2]);



static void inline _npcm7xx_get_time_stamp(u32 time_quad[2])
{
	u32 seconds, seconds_last;
	u32 ref_clock;

	regmap_read(clk_regmap, NPCM7XX_SECCNT, &seconds_last);

	do{
		regmap_read(clk_regmap, NPCM7XX_SECCNT, &seconds);
		regmap_read(clk_regmap, NPCM7XX_CNTR25M, &ref_clock);
		regmap_read(clk_regmap, NPCM7XX_SECCNT, &seconds_last);
	} while (seconds_last != seconds);

	time_quad[0] = ref_clock;
	time_quad[1] = seconds;
}

#define EXT_CLOCK_FREQUENCY_MHZ 25
#define  CNTR25M_ACCURECY                 EXT_CLOCK_FREQUENCY_MHZ  /* minimum accurecy 1us which is 5 cycles */



// Function:        _npcm7xx_delay_relative
// Parameters:
//                  microSecDelay -  number of microseconds to delay since t0_time. if zero: no delay.
//                  t0_time       - start time , to measure time from.
// get a time stamp, delay microSecDelay from it. If microSecDelay has already passed
// since the time stamp , then no delay is executed. returns the time that elapsed since
// t0_time .
static u32  inline _npcm7xx_delay_relative(u32 microSecDelay, u32 t0_time[2])
{
	u32 iUsCnt2[2];
	u32 timeElapsedSince;  // Acctual delay generated by FW
	u32 minimum_delay = (microSecDelay * EXT_CLOCK_FREQUENCY_MHZ) + CNTR25M_ACCURECY; /* this is equivalent to microSec/0.64 + minimal tic length.*/

	do {
		_npcm7xx_get_time_stamp(iUsCnt2);
		timeElapsedSince =  ((EXT_CLOCK_FREQUENCY_MHZ * _1MHz_) * (iUsCnt2[1] - t0_time[1])) + (iUsCnt2[0] - t0_time[0]);
	}
	while(timeElapsedSince < minimum_delay);

	// return elapsed time
	return (u32)(timeElapsedSince / EXT_CLOCK_FREQUENCY_MHZ);
}



#endif // SMB_SW_BYPASS_HW_ISSUE_SMB_STOP


/* GLOBAL VARIABLES */
/* Callback function provided by next-higher level driver or application,
   implementing operation handling  */
/* state-machine                                                                                           */

//static SMB_CALLBACK_T SMB_callback;



/*                                           INTERFACE FUNCTIONS                                           */



static bool SMB_InitModule(nuvoton_i2c_bus_t *bus, SMB_MODE_T mode, u16 bus_freq);

#if defined (MIWU_MODULE_TYPE) && defined (SMB_CAPABILITY_WAKEUP_SUPPORT)
static void SMB_WakeupEnable(nuvoton_i2c_bus_t *bus, bool enable);
#endif  /* (MIWU_MODULE_TYPE) && (SMB_CAPABILITY_WAKEUP_SUPPORT) */

#if !defined SMB_SLAVE_ONLY
static bool SMB_StartMasterTransaction(nuvoton_i2c_bus_t *bus, u8 slave_addr, u16 nwrite, u16 nread,
				       u8 *write_data, u8 *read_data, bool use_PEC);
static void SMB_MasterAbort(nuvoton_i2c_bus_t *bus);

#ifdef TBD
static void SMB_Recovery(nuvoton_i2c_bus_t *bus);
#endif //TBD

#endif  /* !SMB_SLAVE_ONLY */

#if !defined SMB_MASTER_ONLY
static DEFS_STATUS SMB_SlaveGlobalCallEnable(nuvoton_i2c_bus_t *bus, bool enable);
static DEFS_STATUS SMB_SlaveARPEnable(nuvoton_i2c_bus_t *bus, bool enable);
static bool SMB_StartSlaveReceive(nuvoton_i2c_bus_t *bus, u16 nread, u8 *read_data);
static bool SMB_StartSlaveTransmit(nuvoton_i2c_bus_t *bus, u16 nwrite, u8 *write_data);
static DEFS_STATUS SMB_GetCurrentSlaveAddress(nuvoton_i2c_bus_t *bus, u8 *currSlaveAddr);
static DEFS_STATUS SMB_RemSlaveAddress(nuvoton_i2c_bus_t *bus, u8 slaveAddrToRemove);
static DEFS_STATUS SMB_AddSlaveAddress(nuvoton_i2c_bus_t *bus, u8 slaveAddrToAssign, bool use_PEC);
static bool SMB_IsSlaveAddressExist(nuvoton_i2c_bus_t *bus, u8 addr);
static void SMB_Disable(nuvoton_i2c_bus_t *bus);
#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)
static void SMB_EnableTimeout(nuvoton_i2c_bus_t *bus, bool enable);
#endif
#if defined (SMB_CAPABILITY_WAKEUP_SUPPORT)
static void SMB_SetStallAfterStartIdle(nuvoton_i2c_bus_t *bus, bool enable);
#endif
#endif  /* !SMB_MASTER_ONLY */

#ifdef SMB_STALL_TIMEOUT_SUPPORT
static void SMB_ConfigStallThreshold(nuvoton_i2c_bus_t *bus, u8 threshold);
static void SMB_StallHandler(nuvoton_i2c_bus_t *bus);
#endif

#ifdef TBD
static void SMB_Init(SMB_CALLBACK_T operation_done);
static bool SMB_ModuleIsBusy(nuvoton_i2c_bus_t *bus);
static bool SMB_BusIsBusy(nuvoton_i2c_bus_t *bus);
static void SMB_ReEnableModule(nuvoton_i2c_bus_t *bus);
static bool SMB_InterruptIsPending(void);
#endif

#ifdef SMB_CAPABILITY_FORCE_SCL_SDA
static void SMB_WriteSCL(nuvoton_i2c_bus_t *bus, SMB_LEVEL_T level);
static void SMB_WriteSDA(nuvoton_i2c_bus_t *bus, SMB_LEVEL_T level);
#endif // SMB_CAPABILITY_FORCE_SCL_SDA

#ifdef CONFIG_NPCM750_I2C_DEBUG_PRINT
static void SMB_PrintRegs(nuvoton_i2c_bus_t *bus);
static void SMB_PrintModuleRegs(nuvoton_i2c_bus_t *bus);
static void SMB_PrintVersion(void);
#endif


typedef void (*SMB_CALLBACK_T)(nuvoton_i2c_bus_t *bus, SMB_STATE_IND_T op_status, u16 info);

#ifdef SMB_SAMPLE
void SMB_callback(nuvoton_i2c_bus_t *bus, SMB_STATE_IND_T op_status, u16 info)
{
	switch (op_status) {
	case SMB_SLAVE_RCV_IND:
		// Slave got an address match with direction bit clear so it should receive data
		//     the interrupt must call SMB_StartSlaveReceive()
		// info: the enum SMB_ADDR_T address match
		extern u16 read_size;
		extern u8 *read_data_buf;
		SMB_StartSlaveReceive(bus, read_size, read_data_buf);
		break;
	case SMB_SLAVE_XMIT_IND:
		// Slave got an address match with direction bit set so it should transmit data
		//     the interrupt must call SMB_StartSlaveTransmit()
		// info: the enum SMB_ADDR_T address match
		extern u16 write_size;
		extern u8 *write_data_buf;
		SMB_StartSlaveTransmit(bus, write_size, write_data_buf);
		break;
	case SMB_SLAVE_DONE_IND:
		// Slave done transmitting or receiving
		// info:
		//     on receive: number of actual bytes received
		//     on transmit: number of actual bytes transmitted,
		//                  when PEC is used 'info' should be (nwrite+1) which means that 'nwrite' bytes
		//                     were sent + the PEC byte
		//                     'nwrite' is the second parameter SMB_StartSlaveTransmit()
		break;
	case SMB_MASTER_DONE_IND:
		// Master transaction finished and all transmit bytes were sent
		// info: number of bytes actually received after the Master receive operation
		//       (if Master didn't issue receive it should be 0)
		break;
	case SMB_NO_DATA_IND:
		// Notify that not all data was received on Master or Slave
		// info:
		//     on receive: number of actual bytes received
		//                 when PEC is used even if 'info' is the expected number of bytes,
		//                     it means that PEC error occured.
		break;
	case SMB_NACK_IND:
		// MASTER transmit got a NAK before transmitting all bytes
		// info: number of transmitted bytes
		break;
	case SMB_BUS_ERR_IND:
		// Bus error occured
		// info: has no meaning
		break;
	case SMB_WAKE_UP_IND:
		// SMBus wake up occured
		// info: has no meaning
		break;
	default:
		break;
	}
}
#endif    /* SMB_SAMPLE */





/*                                  LOCAL FUNCTIONS FORWARD DECLARATIONS                                   */


static inline void SMB_WriteByte(nuvoton_i2c_bus_t *bus, u8 data);
static inline bool SMB_ReadByte(nuvoton_i2c_bus_t *bus, u8 *data);
static inline void SMB_SelectBank(nuvoton_i2c_bus_t *bus, SMB_BANK_T bank);
static inline u16 SMB_GetIndex(nuvoton_i2c_bus_t *bus);

#if !defined SMB_SLAVE_ONLY
static inline void SMB_Start(nuvoton_i2c_bus_t *bus);
static inline void SMB_Stop(nuvoton_i2c_bus_t *bus);
static inline void SMB_AbortData(nuvoton_i2c_bus_t *bus);
static inline void SMB_StallAfterStart(nuvoton_i2c_bus_t *bus, bool stall);
static inline void SMB_Nack(nuvoton_i2c_bus_t *bus);
#endif  /* !SMB_SLAVE_ONLY */

static          void SMB_Reset(nuvoton_i2c_bus_t *bus);
static          void SMB_InterruptEnable(nuvoton_i2c_bus_t *bus, bool enable);
#if defined (MIWU_MODULE_TYPE) && defined (SMB_CAPABILITY_WAKEUP_SUPPORT)
static          void SMB_WakeupHandler(MIWU_SRC_T source);
#endif

static          bool SMB_InitClock(nuvoton_i2c_bus_t *bus, SMB_MODE_T mode,
				   u16 bus_freq);
static          void SMB_InterruptHandler(nuvoton_i2c_bus_t *bus);
#if !defined SMB_MASTER_ONLY
static          u8 SMB_GetSlaveAddress_l(nuvoton_i2c_bus_t *bus,
					 SMB_ADDR_T addrEnum);
#endif //!defined SMB_MASTER_ONLY
static          void SMB_WriteToFifo(nuvoton_i2c_bus_t *bus,
				     u16 max_bytes_to_send);

static          void SMB_CalcPEC(nuvoton_i2c_bus_t *bus, u8 data);
#ifdef CONFIG_NPCM750_I2C_DEBUG_PRINT
static          void SMBF_PrintModuleRegs(nuvoton_i2c_bus_t *bus);
#endif
static          void SMB_callback(nuvoton_i2c_bus_t *bus,
				  SMB_STATE_IND_T op_status, u16 info);


/* SMB Recovery of the SMBus interface driver */
#if !defined SMB_MASTER_ONLY && defined SMB_RECOVERY_SUPPORT
static          void    SMB_SlaveAbort(nuvoton_i2c_bus_t *bus);  /* SMB slave abort data        */
#endif



/* INTERFACE FUNCTIONS */

static inline void SMB_WriteByte(nuvoton_i2c_bus_t *bus, u8 data)
{
	REG_WRITE(SMBSDA(bus), data);
	SMB_CalcPEC(bus, data);
#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter = 0;
#endif
}

static inline bool SMB_ReadByte(nuvoton_i2c_bus_t *bus, u8 *data)
{
	/* Read data                                                                                           */
	*data = REG_READ(SMBSDA(bus));
	SMB_CalcPEC(bus, *data);
#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter = 0;
#endif

	return true;
}

static inline void SMB_SelectBank(nuvoton_i2c_bus_t *bus, SMB_BANK_T bank)
{
	if (bus->fifo_use == true)
		SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_BNK_SEL, bank);
}

static inline u16 SMB_GetIndex(nuvoton_i2c_bus_t *bus)
{
	u16 index = 0;

	if (bus->operation == SMB_READ_OPER)
		index = bus->read_index;
	else
		if (bus->operation == SMB_WRITE_OPER)
			index = bus->write_index;

	return index;
}

#if !defined SMB_SLAVE_ONLY

static inline void SMB_Start(nuvoton_i2c_bus_t *bus)
{
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_START, true);
#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter = 0;
#endif
}
static inline void SMB_Stop(nuvoton_i2c_bus_t *bus)
{
#ifdef SMB_SW_BYPASS_HW_ISSUE_SMB_STOP
	// override issue #614:  TITLE :CP_FW: SMBus may fail to supply stop condition in Master Write operation. If needed : define it at hal_cfg.h
    bus->clk_period_us = 0;
	_npcm7xx_delay_relative(bus->clk_period_us, bus->interrupt_time_stamp);
#endif //   SMB_SW_BYPASS_HW_ISSUE_SMB_STOP

	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_STOP, true);

	if (bus->fifo_use) {
		u8 smbfif_cts;
		SET_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_THST, 1);
		smbfif_cts = REG_READ(SMBFIF_CTS(bus));
		SET_VAR_FIELD(smbfif_cts, SMBFIF_CTS_SLVRSTR, 1);
		SET_VAR_FIELD(smbfif_cts, SMBFIF_CTS_RXF_TXE, 1);
		REG_WRITE(SMBFIF_CTS(bus), smbfif_cts);
		SET_REG_MASK(SMBFIF_CTS(bus), MASK_FIELD(SMBFIF_CTS_SLVRSTR) |
			     MASK_FIELD(SMBFIF_CTS_RXF_TXE));

		REG_WRITE(SMBTXF_CTL(bus), 0);
	}

#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter = 0;
#endif

}

static inline void SMB_AbortData(nuvoton_i2c_bus_t *bus)
{
	unsigned int timeout = ABORT_TIMEOUT;

	/* Generate a STOP condition                                                                           */
	SMB_Stop(bus);

	/* Clear NEGACK, STASTR and BER bits                                                                   */
	REG_WRITE(SMBST(bus), (MASK_FIELD(SMBST_STASTR) |
			       MASK_FIELD(SMBST_NEGACK) |
			       MASK_FIELD(SMBST_BER)));

	/* Wait till STOP condition is generated                                                               */
	while (--timeout)
		if (!READ_REG_FIELD(SMBCTL1(bus), SMBCTL1_STOP))
			break;

	/* Clear BB (BUS BUSY) bit                                                                             */
	REG_WRITE(SMBCST(bus), MASK_FIELD(SMBCST_BB));
}

static inline void SMB_StallAfterStart(nuvoton_i2c_bus_t *bus, bool stall)
{
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_STASTRE, stall);
}

static inline void SMB_Nack(nuvoton_i2c_bus_t *bus)
{
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_ACK, true);
}
#endif  /* !SMB_SLAVE_ONLY */

static void SMB_Disable(nuvoton_i2c_bus_t *bus)
{
	int i;

	/* Slave Addresses Removal                                                                             */
	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++)
		REG_WRITE(SMBADDR(bus, i), 0);

	/* Disable module.                                                                                     */
	SET_REG_FIELD(SMBCTL2(bus), SMBCTL2_ENABLE, DISABLE);


	/* Set module disable                                                                                  */
	bus->operation_state = SMB_DISABLE;
}

static bool SMB_Enable(nuvoton_i2c_bus_t *bus)
{
	SET_REG_FIELD(SMBCTL2(bus), SMBCTL2_ENABLE, ENABLE);
	return true;
}

static bool SMB_InitModule(nuvoton_i2c_bus_t *bus, SMB_MODE_T mode,
			   u16 bus_freq)
{
	unsigned long bank_flags;
	int     i;

	/* Check whether module already enabled or frequency is out of bounds                                  */
	if (((bus->operation_state != SMB_DISABLE) &&
	     (bus->operation_state != SMB_IDLE)) ||
	    (bus_freq < SMBUS_FREQ_MIN) || (bus_freq > SMBUS_FREQ_MAX))
		return false;

	/* Mux SMB module pins                                                                                 */
	//lint -e{792} suppress PC-Lint warning on 'void cast of void expression'
#ifdef TBD
	SMB_MUX(bus);
#endif
	/* Configure FIFO mode                                                                                 */
	//lint -e{774, 506} suppress PC-Lint warning on 'bool within 'left side of && within if' always evaluates to true'
	if (SMB_FIFO(bus) && READ_REG_FIELD(SMB_VER(bus), SMB_VER_FIFO_EN)) {
		bus->fifo_use = true;
		bus->threshold_fifo = SMBUS_FIFO_SIZE;
		SET_REG_FIELD(SMBFIF_CTL(bus), SMBFIF_CTL_FIFO_EN, 1);
	} else
		bus->fifo_use = false;

	/* Configure SMB module clock frequency                                                                */
	if (!SMB_InitClock(bus, mode, bus_freq)) {
		I2C_DEBUG("SMB_InitClock failed\n");
		return false;
	}

	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0); // select bank 0 for SMB addresses

	/* Configure slave addresses (by default they are disabled)                                            */
	for (i = 0; i < SMB_NUM_OF_ADDR; i++)
		REG_WRITE(SMBADDR(bus, i), 0);

	SMB_SelectBank(bus, SMB_BANK_1); // by default most access is in bank 1
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

	/* Enable module - before configuring CTL1 !                                                           */
	if (!SMB_Enable(bus))
		return false;
	else
		bus->operation_state = SMB_IDLE;

	/* Enable SMB interrupt and New Address Match interrupt source                                         */
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_NMINTE, ENABLE);
	SMB_InterruptEnable(bus, true);

	return true;
}

#if defined (MIWU_MODULE_TYPE) && defined (SMB_CAPABILITY_WAKEUP_SUPPORT)
static void SMB_WakeupEnable(nuvoton_i2c_bus_t *bus, bool enable)
{
	const MIWU_SRC_T SMbusToMiwu[] = SMB_WAKEUP_SRC;
	MIWU_SRC_T miwu_src = SMbusToMiwu[bus];

	if (enable) {
		/* Configure MIWU module to generate an interrupt to the ICU following SMBus wake-up conditions    */
		MIWU_Config(miwu_src, MIWU_RISING_EDGE, SMB_WakeupHandler);

		/* Configure SMBus Wake-up (in System Glue Function)                                               */
		REG_WRITE(SMB_SBD, MASK_BIT(bus));   /* Clear Start condition detection                     */
		SET_REG_BIT(SMB_EEN, bus);           /* Enable Event assertion                              */
		SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_IDL_START, 1); /* Enable start detect in IDLE           */
	} else {    /* Disable */
		/*  Disable SMB Wake-Up indication via MIWU                                                        */
		MIWU_EnableChannel(miwu_src, false);

		/* Disable SMBus Wake-up (in System Glue Function)                                                 */
		CLEAR_REG_BIT(SMB_EEN, bus);        /* Disable Event assertion                              */

		SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_IDL_START, 0); /* Disable start detect in IDLE          */
	}
}
#endif  /* (MIWU_MODULE_TYPE) && (SMB_CAPABILITY_WAKEUP_SUPPORT) */


#if !defined SMB_MASTER_ONLY
static DEFS_STATUS SMB_SlaveEnable_l(nuvoton_i2c_bus_t *bus,
				     SMB_ADDR_T addr_type, u8 addr, bool enable)
{
	unsigned long bank_flags;
	u8 SmbAddrX_Addr = BUILD_FIELD_VAL(SMBADDRx_ADDR, addr) |
		BUILD_FIELD_VAL(SMBADDRx_SAEN, enable);

	if (addr_type == SMB_GC_ADDR) {
		SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_GCMEN, enable);
		return DEFS_STATUS_OK;
	}
	if (addr_type == SMB_ARP_ADDR) {
		SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_ARPMEN, enable);
		return DEFS_STATUS_OK;
	}
	if (addr_type >= SMB_NUM_OF_ADDR)
		return DEFS_STATUS_FAIL;

	/* Disable interrupts and select bank 0 for address 3 to ...                                           */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

	/* Set and enable the address                                                                          */
	REG_WRITE(SMBADDR(bus, addr_type), SmbAddrX_Addr);

	/* return to bank 1 and enable interrupts (if needed)                                                  */
	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

	return DEFS_STATUS_OK;
}

static u8 SMB_GetSlaveAddress_l(nuvoton_i2c_bus_t *bus, SMB_ADDR_T addrEnum)
{
	unsigned long bank_flags;
	u8 slaveAddress;

	/* disable interrupts and select bank 0 for address 3 to ...                                           */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

	slaveAddress = REG_READ(SMBADDR(bus, addrEnum));

	/* return to bank 1 and enable interrupts (if needed)                                                  */
	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

	return  slaveAddress;
}

static bool SMB_IsSlaveAddressExist(nuvoton_i2c_bus_t *bus, u8 addr)
{
	int i;

	addr |= 0x80; //Set the enable bit

	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++)
		if (addr == SMB_GetSlaveAddress_l(bus, (SMB_ADDR_T)i))
			return true;

	return false;
}

static DEFS_STATUS SMB_AddSlaveAddress(nuvoton_i2c_bus_t *bus,
				       u8 slaveAddrToAssign, bool use_PEC)
{
	int i;
	DEFS_STATUS ret = DEFS_STATUS_FAIL;

	slaveAddrToAssign |= 0x80; //set the enable bit

	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++) {
		u8 currentSlaveAddr = SMB_GetSlaveAddress_l(bus, (SMB_ADDR_T)i);
		if (currentSlaveAddr == slaveAddrToAssign) {
			ret = DEFS_STATUS_OK;
			break;
		} else if ((currentSlaveAddr & 0x7F) == 0) {
			ret = SMB_SlaveEnable_l(bus, (SMB_ADDR_T)i, slaveAddrToAssign, true);
			break;
		}
	}

	if (ret == DEFS_STATUS_OK) {
		if (use_PEC)
			SET_VAR_BIT(bus->PEC_mask, i);
		else
			CLEAR_VAR_BIT(bus->PEC_mask, i);
	}
	return ret;
}

static DEFS_STATUS SMB_RemSlaveAddress(nuvoton_i2c_bus_t *bus,
				       u8 slaveAddrToRemove)
{
	int i;
	unsigned long bank_flags;

	slaveAddrToRemove |= 0x80; //Set the enable bit

	/* disable interrupts and select bank 0 for address 3 to ...                                           */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

	for (i = SMB_SLAVE_ADDR1; i < SMB_NUM_OF_ADDR; i++) {
		if (REG_READ(SMBADDR(bus, i)) == slaveAddrToRemove)
			REG_WRITE(SMBADDR(bus, i), 0);
	}

	/* return to bank 1 and enable interrupts (if needed)                                                  */
	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

	return DEFS_STATUS_OK;
}

static DEFS_STATUS SMB_SlaveGlobalCallEnable(nuvoton_i2c_bus_t *bus,
					     bool enable)
{
	return SMB_SlaveEnable_l(bus, SMB_GC_ADDR, 0, enable);
}

static DEFS_STATUS SMB_SlaveARPEnable(nuvoton_i2c_bus_t *bus, bool enable)
{
	return SMB_SlaveEnable_l(bus, SMB_ARP_ADDR, 0, enable);
}

#endif  /* !SMB_MASTER_ONLY */


#if !defined SMB_SLAVE_ONLY

static bool SMB_StartMasterTransaction(nuvoton_i2c_bus_t *bus, u8 slave_addr,
				       u16 nwrite, u16 nread, u8 *write_data,
				       u8 *read_data, bool use_PEC)
{
    unsigned long lock_flags;

#ifdef CONFIG_NPCM750_I2C_DEBUG
	I2C_DEBUG("bus=%d slave_addr=%x nwrite=%d nread=%d write_data=%p "
		  "read_data=%p use_PEC=%d\n", bus, slave_addr, nwrite, nread,
		  write_data, read_data, use_PEC);

	if (nwrite && nwrite != SMB_BYTES_QUICK_PROT) {
		int i;
		char str[32 * 3 + 4];
		char *s = str;

		for (i = 0; (i < nwrite && i < 32); i++)
			s += sprintf(s, "%02x ", write_data[i]);

		printk("write_data = %s\n", str);
	}
#endif


	/* Allow only if bus is not busy                                                                       */
	if ((bus->operation_state != SMB_IDLE)
#if defined SMBUS_SIZE_CHECK
	    ||
	    ((nwrite >= _32KB_) && (nwrite != SMB_BYTES_QUICK_PROT)) ||
	    ((nread >= _32KB_) && (nread != SMB_BYTES_BLOCK_PROT) &&
	     (nread != SMB_BYTES_QUICK_PROT))
#endif
	    )
		return false;

    spin_lock_irqsave(&bus->lock, lock_flags);


	/* Update driver state                                                                                 */
	bus->master_or_slave = SMB_MASTER;
	bus->operation_state = SMB_MASTER_START;
	if (nwrite > 0)
		bus->operation = SMB_WRITE_OPER;
	else
		bus->operation = SMB_READ_OPER;

	bus->dest_addr      = (u8)(slave_addr << 1);  /* Translate 7-bit to 8-bit format  */
	bus->write_data_buf = write_data;
	bus->write_size     = nwrite;
	bus->write_index    = 0;
	bus->read_data_buf  = read_data;
	bus->read_size      = nread;
	bus->read_index     = 0;
	bus->PEC_use        = use_PEC;
	bus->read_block_use = false;
	bus->retry_count    = SMB_RETRY_MAX_COUNT;

	/* Check if transaction uses Block read protocol                                                       */
	if ((bus->read_size == SMB_BYTES_BLOCK_PROT) ||
	    (bus->read_size == SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER)) {
		bus->read_block_use = true;

		/* Change nread in order to configure recieve threshold to 1                                       */
		nread = 1;
	}

	/* clear BER just in case it is set due to a previous transaction                                      */
	REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_BER));

	/* Initiate SMBus master transaction                                                                   */
	/* Generate a Start condition on the SMBus                                                             */
	if (bus->fifo_use == true) {
        unsigned long bank_flags;
		/* select bank 1 for FIFO registers                                                                */
        spin_lock_irqsave(&bus->bank_lock, bank_flags);
		SMB_SelectBank(bus, SMB_BANK_1);
        spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

		/* clear FIFO and relevant status bits.                                                            */
		SET_REG_MASK(SMBFIF_CTS(bus), MASK_FIELD(SMBFIF_CTS_SLVRSTR) |
			     MASK_FIELD(SMBFIF_CTS_CLR_FIFO) |
			     MASK_FIELD(SMBFIF_CTS_RXF_TXE));

		if (nwrite == 0) {
			/* This is a read only operation. Configure the FIFO                                           */
			/* threshold according to the needed number of bytes to read.                                  */
			if (nread > SMBUS_FIFO_SIZE)
				SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, SMBUS_FIFO_SIZE);
			else {
				SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, (u8)(nread));

				if ((bus->read_size != SMB_BYTES_BLOCK_PROT) &&
				    (bus->read_size != SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER))
					SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_LAST_PEC, 1);
			}
		}
	}

	SMB_Start(bus);

    spin_unlock_irqrestore(&bus->lock, lock_flags);

	return true;
}
#endif  /* !SMB_SLAVE_ONLY */


#if !defined SMB_MASTER_ONLY
static bool SMB_StartSlaveReceive(nuvoton_i2c_bus_t *bus, u16 nread,
				  u8 *read_data)
{

	/* Allow only if bus is not busy                                                                       */
	if ((bus->operation_state != SMB_SLAVE_MATCH)
#if defined SMBUS_SIZE_CHECK
	    ||
	    ((nread >= _32KB_) && (nread != SMB_BYTES_BLOCK_PROT) && (nread != SMB_BYTES_QUICK_PROT))
#endif
	    )
		return false;

	/* Update driver state                                                                                 */
	bus->operation_state = SMB_OPER_STARTED;
	bus->operation       = SMB_READ_OPER;
	bus->read_data_buf   = read_data;
	bus->read_size       = nread;
	bus->read_index      = 0;
	bus->write_size      = 0;
	bus->write_index     = 0;

	if (bus->fifo_use == true) {
		if (nread > 0) {
			u8 smbrxf_ctl;

			if (nread <= SMBUS_FIFO_SIZE) {
				smbrxf_ctl = BUILD_FIELD_VAL(SMBRXF_CTL_THR_RXIE, 0);
				smbrxf_ctl |= BUILD_FIELD_VAL(SMBRXF_CTL_RX_THR, nread);
			} else {
				/* if threshold_fifo != SMBUS_FIFO_SIZE set SMBRXF_CTL.THR_RXIE to 1 otherwise to 0        */
				smbrxf_ctl = BUILD_FIELD_VAL(SMBRXF_CTL_RX_THR, bus->threshold_fifo) |
				    BUILD_FIELD_VAL(SMBRXF_CTL_THR_RXIE, (bool)(bus->threshold_fifo != SMBUS_FIFO_SIZE));
			}
			REG_WRITE(SMBRXF_CTL(bus), smbrxf_ctl);
		}

		/* triggers new data reception                                                                     */
		REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_NMATCH));
	}

	return true;
}

static bool SMB_StartSlaveTransmit(nuvoton_i2c_bus_t *bus, u16 nwrite,
				   u8 *write_data)
{

	/* Allow only if bus is not busy                                                                       */
	if ((bus->operation_state != SMB_SLAVE_MATCH) || (nwrite == 0))
		return false;


	/* Update driver state                                                                                 */
	if (bus->PEC_use)
		nwrite++;

	bus->operation_state = SMB_OPER_STARTED;
	bus->operation       = SMB_WRITE_OPER;
	bus->write_data_buf  = write_data;
	bus->write_size      = nwrite;
	bus->write_index     = 0;

	if (bus->fifo_use == true) {
		/* triggers new data reception                                                                     */
		REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_NMATCH));

		if (nwrite > 0) {
			u8 smbtxf_ctl;

			if (nwrite <= SMBUS_FIFO_SIZE)
				smbtxf_ctl = BUILD_FIELD_VAL
				(SMBTXF_CTL_THR_TXIE, 0) |
				BUILD_FIELD_VAL(SMBTXF_CTL_TX_THR, 0);
			else
				/* if threshold_fifo != SMBUS_FIFO_SIZE set SMBTXF_CTL.THR_TXIE to 1 otherwise to 0        */
				smbtxf_ctl = BUILD_FIELD_VAL
				(SMBTXF_CTL_THR_TXIE,
				 (bool)(bus->threshold_fifo != SMBUS_FIFO_SIZE))
				| BUILD_FIELD_VAL(SMBTXF_CTL_TX_THR,
						  SMBUS_FIFO_SIZE -
						  bus->threshold_fifo);

			REG_WRITE(SMBTXF_CTL(bus), smbtxf_ctl);

			/* Fill the FIFO with data                                                                     */
			SMB_WriteToFifo(bus, MIN(SMBUS_FIFO_SIZE, nwrite));
		}
	}

	return true;
}
#endif  /* !SMB_MASTER_ONLY */

#ifdef TBD
static bool SMB_ModuleIsBusy(nuvoton_i2c_bus_t *bus)
{
	return (READ_REG_FIELD(SMBCST(bus), SMBCST_BUSY) ||
		READ_REG_FIELD(SMBST(bus), SMBST_SLVSTP));
}

static bool SMB_BusIsBusy(nuvoton_i2c_bus_t *bus)
{
	return READ_REG_FIELD(SMBCST(bus), SMBCST_BB);
}
#endif //TBD

#if !defined SMB_MASTER_ONLY
static DEFS_STATUS SMB_GetCurrentSlaveAddress(nuvoton_i2c_bus_t *bus,
					      u8 *currSlaveAddr)
{
	if (currSlaveAddr != NULL) {
		*currSlaveAddr = bus->SMB_CurSlaveAddr;
		return DEFS_STATUS_OK;
	}

	return DEFS_STATUS_INVALID_PARAMETER;
}
#endif

#if defined (MIWU_MODULE_TYPE) && defined (SMB_CAPABILITY_WAKEUP_SUPPORT)

static void SMB_WakeupHandler(MIWU_SRC_T source)
{
	nuvoton_i2c_bus_t *bus;    /* Module whose bus generated the wake-up */

	/* SMB module is enabled already (otherwise a wakeup signal isn't generated) -                         */
	/* so no need to enable the SMB. SMB HW responds with a negative acknowledge to the Start Condition -  */
	/* so either the Master Device will re-issue a Start Condition, or the upper layer will initiate a     */
	/* transaction as a master                                                                             */

	/* Check wake-up source                                                                                */
	for (bus = 0; bus < SMB_NUM_OF_MODULES; bus++) {
		if (READ_REG_BIT(SMB_SBD, bus) && READ_REG_BIT(SMB_EEN, bus)) {
			/* Clear start-bit-detected status                                                             */
			REG_WRITE(SMB_SBD, MASK_BIT(bus));

			/* Restore SMBnCTL1 register values, because the register is being reseted when the core       */
			/* switches to Idle or Deep-Idle mode.                                                         */
			SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_NMINTE, ENABLE);
			SMB_InterruptEnable(bus, true);

			/* Notify upper layer of wake-up                                                               */
			SMB_callback(bus, SMB_WAKE_UP_IND, 0);
		}
	}

}
#endif  /* (MIWU_MODULE_TYPE) && (SMB_CAPABILITY_WAKEUP_SUPPORT) */

static void SMB_ReadFromFifo(nuvoton_i2c_bus_t *bus, u8 bytes_in_fifo)
{
	while (bytes_in_fifo--) {
		/* Keep read data                                                                                  */
		u8 data = REG_READ(SMBSDA(bus));

		SMB_CalcPEC(bus, data);
		if (bus->read_index < bus->read_size) {
			bus->read_data_buf[bus->read_index++] = data;
			if ((bus->read_index == 1) && bus->read_size == SMB_BYTES_BLOCK_PROT)
				/* First byte indicates length in block protocol                                           */
				bus->read_size = data;
		}
	}
}

static void SMB_MasterFifoRead(nuvoton_i2c_bus_t *bus)
{
	u16 rcount;
	u8 fifo_bytes;
	SMB_STATE_IND_T ind = SMB_MASTER_DONE_IND;

	rcount = bus->read_size - bus->read_index;


	/* In order not to change the RX_TRH during transaction (we found that this might                      */
	/* be problematic if it takes too much time to read the FIFO) we read the data in the                  */
	/* following way. If the number of bytes to read == FIFO Size + C (where C < FIFO Size)                */
	/* then first read C bytes and in the next interrupt we read rest of the data.                         */
	if ((rcount < (2 * SMBUS_FIFO_SIZE)) && (rcount > SMBUS_FIFO_SIZE))
		fifo_bytes = (u8)(rcount - SMBUS_FIFO_SIZE);
	else
		fifo_bytes = READ_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_BYTES);

	if (rcount - fifo_bytes == 0) {
		/* last byte is about to be read - end of transaction.                                             */
		/* Stop should be set before reading last byte.                                                    */
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
		/* Enable "End of Busy" interrupt.                                                                 */
		SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE, 1);
#endif
		SMB_Stop(bus);

		SMB_ReadFromFifo(bus, fifo_bytes);

#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
		if ((bus->PEC_use == true) && (REG_READ(SMBPEC(bus)) != 0))
#else
			if ((bus->PEC_use == true) && (bus->crc_data != 0))
#endif
				ind = SMB_MASTER_PEC_ERR_IND;

#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
		bus->operation_state = SMB_STOP_PENDING;
		bus->stop_indication = ind;
#else
		/* Reset state for new transaction                                                                 */
		bus->operation_state = SMB_IDLE;

		/* Notify upper layer of transaction completion                                                    */
		SMB_callback(bus, ind, bus->read_index);
#endif
	} else {
		SMB_ReadFromFifo(bus, fifo_bytes);
		rcount = bus->read_size - bus->read_index;

		if (rcount > 0) {
			SET_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_THST, 1);
			SET_REG_FIELD(SMBFIF_CTS(bus), SMBFIF_CTS_RXF_TXE, 1);

			if (rcount > SMBUS_FIFO_SIZE)
				SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, SMBUS_FIFO_SIZE);
			else {
				SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, (u8)(rcount));
				SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_LAST_PEC, 1);
			}
		}
	}

#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter = 0;
#endif
}

static void SMB_WriteToFifo(nuvoton_i2c_bus_t *bus, u16 max_bytes_to_send)
{

	/* Fill the FIFO , while the FIFO is not full and there are more bytes to write                        */
	while ((max_bytes_to_send--) && (SMBUS_FIFO_SIZE -
					 READ_REG_FIELD(SMBTXF_STS(bus),
							SMBTXF_STS_TX_BYTES))) {
		/* write the data                                                                                  */
		if (bus->write_index < bus->write_size) {
			if ((bus->PEC_use == true) &&
			    ((bus->write_index + 1) == bus->write_size) &&
			    ((bus->read_size == 0) ||
			     (bus->master_or_slave == SMB_SLAVE))) {
				/* Master send PEC in write protocol, Slave send PEC in read protocol.                     */
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
				REG_WRITE(SMBSDA(bus), REG_READ(SMBPEC(bus)));
#else
				REG_WRITE(SMBSDA(bus), bus->crc_data);
#endif
				bus->write_index++;
			} else
				SMB_WriteByte(bus, bus->write_data_buf[bus->write_index++]);
		} else {

/* define this at hal_cfg or chip file, if one wishes to use this feature. Otherwise driver will xmit 0xFF */
#ifdef SMB_WRAP_AROUND_BUFFER
			/* We're out of bytes. Ask the higher level for more bytes. Let it know that driver used all its' bytes */

			/* clear the status bits                                                                       */
			SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1);

			/* Reset state for the remaining bytes transaction                                             */
			bus->operation_state = SMB_SLAVE_MATCH;

			/* Notify upper layer of transaction completion                                                */
			SMB_callback(bus, SMB_SLAVE_XMIT_MISSING_DATA_IND,
				     bus->write_index);

			REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_SDAST));
#else
			SMB_WriteByte(bus, 0xFF);
#endif
		}
	}
}

static bool SMB_InitClock(nuvoton_i2c_bus_t *bus, SMB_MODE_T mode, u16 bus_freq)
{
#if defined (SMB_CAPABILITY_FAST_MODE_SUPPORT) || defined (SMB_CAPABILITY_FAST_MODE_PLUS_SUPPORT)
	u16  k1          = 0;
	u16  k2          = 0;
	u8   dbnct       = 0;
#endif
	u16  sclfrq      = 0;
	u8   hldt        = 7;
	bool fastMode    = false;
	unsigned long bank_flags;
	u32  source_clock_freq;

	source_clock_freq = bus->apb_clk;


	/* Frequency is less or equal to 100 KHz                                                               */
	if (bus_freq <= SMBUS_FREQ_100KHz) {
		/* Set frequency:                                                                                  */
		/* SCLFRQ = T(SCL)/4/T(CLK) = FREQ(CLK)/4/FREQ(SCL) = FREQ(CLK) / ( FREQ(SCL)*4 )                  */
		sclfrq = (u16)((source_clock_freq / ((u32)bus_freq * _1KHz_ * 4)));   // bus_freq is KHz

		/* Check whether requested frequency can be achieved in current CLK                                */
		if ((sclfrq < SCLFRQ_MIN) || (sclfrq > SCLFRQ_MAX))
			return false;

		if (source_clock_freq >= 40000000)
			hldt = 17;
		else if (source_clock_freq >= 12500000)
			hldt = 15;
		else
			hldt = 7;
	}

#ifdef SMB_CAPABILITY_FAST_MODE_SUPPORT

	/* Frequency equal to 400 KHz                                                                          */

	else if (bus_freq == SMBUS_FREQ_400KHz) {
		sclfrq       = 0;
		fastMode     = true;

		if ((mode == SMB_MASTER && source_clock_freq < 7500000) ||
		    (mode == SMB_SLAVE  && source_clock_freq < 10000000))
			/* 400KHz cannot be supported for master core clock < 7.5 MHz or slave core clock < 10 MHz     */
			return false;

		/* Master or Slave with frequency > 25 MHz                                                         */
		if (mode == SMB_MASTER || source_clock_freq > 25000000) {
			/* Set HLDT:                                                                                   */
			/* SDA hold time:  (HLDT-7) * T(CLK) >= 300                                                    */
			/* HLDT = 300/T(CLK) + 7 = 300 * FREQ(CLK) + 7                                                 */
			hldt = (u8)DIV_CEILING((300 * (source_clock_freq / _1KHz_)), ((u32)_1MHz_)) + 7;

			if (mode == SMB_MASTER) {
				/* Set k1:                                                                                 */
				/* Clock low time: k1 * T(CLK) - T(SMBFO) >= 1300                                          */
				/* T(SMBRO) = T(SMBFO) = 300                                                               */
				/* k1 = (1300 + T(SMBFO)) / T(CLK) = 1600 * FREQ(CLK)                                      */
				k1 = ROUND_UP(((u16)DIV_CEILING((1600 * (source_clock_freq / _1KHz_)), ((u32)_1MHz_))), 2);

				/* Set k2:                                                                                 */
				/* START setup: (k2 - 1) * T(CLK) - T(SMBFO) >= 600                                        */
				/* T(SMBRO) = T(SMBFO) = 300                                                               */
				/* k2 = (600 + T(SMBFO)) / T(CLK) + 1 = 900 * FREQ(CLK) + 1                                */
				k2 = ROUND_UP(((u16)DIV_CEILING((900 * (source_clock_freq / _1KHz_)), ((u32)_1MHz_)) + 1), 2);

				/* Check whether requested frequency can be achieved in current CLK                        */
				if ((k1 < SCLFRQ_MIN) || (k1 > SCLFRQ_MAX) || (k2 < SCLFRQ_MIN) || (k2 > SCLFRQ_MAX))
					return false;
			}
		}

		/* Slave with frequency 10-25 MHz                                                                  */
		else {
			hldt  = 7;
			dbnct = 2;
		}
	}
#endif //SMB_CAPABILITY_FAST_MODE_SUPPORT

#ifdef SMB_CAPABILITY_FAST_MODE_PLUS_SUPPORT

	/* Frequency equal to 1 MHz                                                                            */
	else if (bus_freq == SMBUS_FREQ_1MHz) {
		sclfrq       = 0;
		fastMode     = true;

		if ((mode == SMB_MASTER && source_clock_freq < 15000000) ||
		    (mode == SMB_SLAVE  && source_clock_freq < 24000000))

			/* 1MHz cannot be supported for master core clock < 15 MHz or slave core clock < 24 MHz        */
			return false;

		/* Master or Slave with frequency > 40 MHz                                                         */
		if (mode == SMB_MASTER || source_clock_freq > 40000000) {

			/* Set HLDT:                                                                                   */
			/* SDA hold time:  (HLDT-7) * T(CLK) >= 120                                                    */
			/* HLDT = 120/T(CLK) + 7 = 120 * FREQ(CLK) + 7                                                 */
			hldt = (u8)DIV_CEILING((120 * (source_clock_freq / _1KHz_)), ((u32)_1MHz_)) + 7;

			if (mode == SMB_MASTER) {

				/* Set k1:                                                                                 */
				/* Clock low time: k1 * T(CLK) - T(SMBFO) >= 500                                           */
				/* T(SMBRO) = T(SMBFO) = 120                                                               */
				/* k1 = (500 + T(SMBFO)) / T(CLK) = 620 * FREQ(CLK)                                        */
				k1 = ROUND_UP(((u16)DIV_CEILING((620 * (source_clock_freq / _1KHz_)), ((u32)_1MHz_))), 2);


				/* Set k2:                                                                                 */
				/* START setup: (k2 - 1) * T(CLK) - T(SMBFO) >= 260                                        */
				/* T(SMBRO) = T(SMBFO) = 120                                                               */
				/* k2 = (260 + T(SMBFO)) / T(CLK) + 1 = 380 * FREQ(CLK) + 1                                */
				k2 = ROUND_UP(((u16)DIV_CEILING((380 * (source_clock_freq / _1KHz_)), ((u32)_1MHz_)) + 1), 2);


				/* Check whether requested frequency can be achieved in current CLK                        */
				if ((k1 < SCLFRQ_MIN) || (k1 > SCLFRQ_MAX) || (k2 < SCLFRQ_MIN) || (k2 > SCLFRQ_MAX)) {
					return false;
				}
			}
		}

		/* Slave with frequency 24-40 MHz                                                                  */
		else {
			hldt  = 7;
			dbnct = 2;
		}
	}
#endif //SMB_CAPABILITY_FAST_MODE_PLUS_SUPPORT


	/* Frequency larger than 1 MHz                                                                         */
	else
		return false;



	/* After clock parameters calculation update the register                                              */
	SET_REG_FIELD(SMBCTL2(bus), SMBCTL2_SCLFRQ6_0,
		      READ_VAR_FIELD(sclfrq, SCLFRQ_0_TO_6));
	SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_SCLFRQ8_7,
		      READ_VAR_FIELD(sclfrq, SCLFRQ_7_TO_8));
	SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_400K_MODE, fastMode);


	/* Select Bank 0 to access SMBCTL4/SMBCTL5                                                             */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

#if defined (SMB_CAPABILITY_FAST_MODE_SUPPORT) || defined (SMB_CAPABILITY_FAST_MODE_PLUS_SUPPORT)
	if (bus_freq >= SMBUS_FREQ_400KHz) {

		/* k1 and k2 are relevant for master mode only                                                     */
		if (mode == SMB_MASTER) {

			/* Set SCL Low/High Time:                                                                      */
			/* k1 = 2 * SCLLT7-0 -> Low Time  = k1 / 2                                                     */
			/* k2 = 2 * SCLLT7-0 -> High Time = k2 / 2                                                     */
			REG_WRITE(SMBSCLLT(bus), (u8)k1 / 2);
			REG_WRITE(SMBSCLHT(bus), (u8)k2 / 2);
		}

		/* DBNCT is relevant for slave mode only                                                           */
		else
			SET_REG_FIELD(SMBCTL5(bus), SMBCTL5_DBNCT, dbnct);
	}
#endif

	SET_REG_FIELD(SMBCTL4(bus), SMBCTL4_HLDT, hldt);


	/* Return to Bank 1                                                                                    */
	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

	return true;
}


#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)

static void SMB_EnableTimeout(nuvoton_i2c_bus_t *bus, bool enable)
{
	u8 toCkDiv;
	u8 smbEnabled;
	u8 smbctl1 = 0;

	if (enable) {

		/* TO_CKDIV may be changed only when the SMB is disabled                                           */
		smbEnabled = READ_REG_FIELD(SMBCTL2(bus), SMBCTL2_ENABLE);

		/* If SMB is enabled - disable the SMB module                                                      */
		if (smbEnabled) {

			/* Save smbctl1 relevant bits. It is being cleared when the module is disabled                 */
			smbctl1 = REG_READ(SMBCTL1(bus)) & (MASK_FIELD(SMBCTL1_GCMEN) | MASK_FIELD(SMBCTL1_INTEN) | MASK_FIELD(SMBCTL1_NMINTE));

			/* Disable the SMB module                                                                      */
			SET_REG_FIELD(SMBCTL2(bus), SMBCTL2_ENABLE, DISABLE);
		}

		/* Clear EO_BUSY pending bit                                                                       */
		SET_REG_FIELD(SMBT_OUT(bus), SMBT_OUT_T_OUTST, 1);

		/* Configure the division of the SMB Module Basic clock (BCLK) to generate the 1 KHz clock of the  */
		/* timeout detector.                                                                               */
		/* The timeout detector has an “n+1” divider, controlled by TO_CKDIV and a fixed divider by 1000.  */
		/* Together they generate the 1 ms clock cycle                                                     */
		toCkDiv = (u8)(((bus->apb_clk / _1KHz_) / 1000) - 1);

		/* Set the bus timeout clock divisor                                                               */
		SET_REG_FIELD(SMBT_OUT(bus), SMBT_OUT_TO_CKDIV, toCkDiv);

		/* If SMB was enabled - re-enable the SMB module                                                   */
		if (smbEnabled) {

			/* Enable the SMB module                                                                       */
			(void)SMB_Enable(bus);

			/* Restore smbctl1 status                                                                      */
			REG_WRITE(SMBCTL1(bus),  smbctl1);
		}
	}


	/* Enable/Disable the bus timeout interrupt                                                            */
	SET_REG_FIELD(SMBT_OUT(bus), SMBT_OUT_T_OUTIE, enable);
}
#endif

static void SMB_InterruptHandler(nuvoton_i2c_bus_t *bus)
{

	/* A negative acknowledge has occurred                                                                 */
	if (READ_REG_FIELD(SMBST(bus), SMBST_NEGACK)) {
		if (bus->fifo_use) {

			/* if there are still untransmitted bytes in TX FIFO reduce them from write_index              */
			bus->write_index -= READ_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_BYTES);

			/* clear the FIFO                                                                              */
			REG_WRITE(SMBFIF_CTS(bus), MASK_FIELD(SMBFIF_CTS_CLR_FIFO));
		}

		/* In slave write operation, NACK is OK, otherwise it is a problem                                 */
		if (!((bus->master_or_slave == SMB_SLAVE) &&
		      (bus->write_index != 0) &&
		      (bus->write_index == bus->write_size)))
		/* Either not slave, or number of bytes sent to master less than required                          */
		/* In either case notify upper layer. If we are slave - the upper layer                            */
		/* should still wait for a Slave Stop.                                                             */
		{
#if !defined SMB_SLAVE_ONLY
			if ((bus->master_or_slave == SMB_MASTER) &&
			    READ_REG_FIELD(SMBST(bus), SMBST_MASTER)) {
				/* Only current master is allowed to issue Stop Condition                                  */
				SMB_MasterAbort(bus);
			}
#endif  /* !SMB_SLAVE_ONLY */

			//REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_NEGACK));
			bus->operation_state = SMB_IDLE;
			SMB_callback(bus, SMB_NACK_IND, bus->write_index);
		}

		/* else:                                                                                           */
		/* Slave has to wait for SMB_STOP to decide this is the end of the transaction.                    */
		/* Therefore transaction is not yet considered as done                                             */
		/*                                                                                                 */
		/* In Master mode, NEGACK should be cleared only after generating STOP.                            */
		/* In such case, the bus is released from stall only after the software clears NEGACK              */
		/* bit. Then a Stop condition is sent.                                                             */
		REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_NEGACK));

		return;
	}


	/* A Bus Error has been identified                                                                     */
	if (READ_REG_FIELD(SMBST(bus), SMBST_BER)) {

		/* Check whether bus arbitration or Start or Stop during data transfer                             */
#if !defined SMB_SLAVE_ONLY

		/* Bus arbitration problem should not result in recovery                                           */
		if ((bus->master_or_slave == SMB_MASTER)) {
			if (READ_REG_FIELD(SMBST(bus), SMBST_MASTER)) {

				/* Only current master is allowed to issue Stop Condition                                  */
				SMB_MasterAbort(bus);
			} else {

				/* Bus arbitration loss                                                                    */
				if (--bus->retry_count > 0) {
					/* Perform a retry (generate a Start condition as soon as the SMBus is free)           */
					REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_BER));
					SMB_Start(bus);
					return;
				}
			}
		}
#if !defined SMB_MASTER_ONLY
else
#endif
#endif  /* !SMB_SLAVE_ONLY */
#if !defined SMB_MASTER_ONLY
			if (bus->master_or_slave == SMB_SLAVE)

				/* Reset the module                                                                            */
				SMB_Reset(bus);
#endif

		REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_BER));
		bus->operation_state = SMB_IDLE;
		SMB_callback(bus, SMB_BUS_ERR_IND, SMB_GetIndex(bus));
		return;
	}

#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)

	/* A Bus Timeout has been identified                                                                   */
	if ((READ_REG_FIELD(SMBT_OUT(bus), SMBT_OUT_T_OUTIE) == 1) &&  /* bus timeout interrupt is on   */
	    (READ_REG_FIELD(SMBT_OUT(bus), SMBT_OUT_T_OUTST))) {         /* and bus timeout status is set */
#if !defined SMB_SLAVE_ONLY
		if (bus->master_or_slave == SMB_MASTER) {

			/* Only current master is allowed to issue Stop Condition                                      */
			SMB_MasterAbort(bus);
		}
#if !defined SMB_MASTER_ONLY
else
#endif
#endif  /* !SMB_SLAVE_ONLY */
#if !defined SMB_MASTER_ONLY
			if (bus->master_or_slave == SMB_SLAVE) {

				/* Reset the module                                                                            */

				SMB_Reset(bus);
			}
#endif

		SET_REG_FIELD(SMBT_OUT(bus), SMBT_OUT_T_OUTST, 1); /* Clear EO_BUSY pending bit             */
		bus->operation_state = SMB_IDLE;
		SMB_callback(bus, SMB_BUS_ERR_IND, SMB_GetIndex(bus));
		return;
	}
#endif

#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT

	/* A Master End of Busy (meaning Stop Condition happened)                                              */

	if ((READ_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE) == 1) &&  /* End of Busy interrupt is on     */
	    (READ_REG_FIELD(SMBCST3(bus), SMBCST3_EO_BUSY))) {        /* and End of Busy is set          */
		SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE, 0); /* Disable "End of Busy" interrupt         */
		SET_REG_FIELD(SMBCST3(bus), SMBCST3_EO_BUSY, 1); /* Clear EO_BUSY pending bit               */

		bus->operation_state = SMB_IDLE;

		if ((bus->write_size == SMB_BYTES_QUICK_PROT) ||
		    (bus->read_size == SMB_BYTES_QUICK_PROT) ||
		    (bus->read_size == 0)) {
			SMB_callback(bus, bus->stop_indication, 0);
		} else {
			SMB_callback(bus, bus->stop_indication,
				     bus->read_index);
		}
		return;
	}
#endif

#if !defined SMB_MASTER_ONLY

	/* A Slave Stop Condition has been identified                                                          */

	if (READ_REG_FIELD(SMBST(bus), SMBST_SLVSTP)) {
		SMB_STATE_IND_T ind;
		ASSERT(bus->master_or_slave == SMB_SLAVE);
		REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_SLVSTP));


		/* Check whether bus arbitration or Start or Stop during data transfer                             */
		bus->operation_state = SMB_IDLE;
		if (bus->fifo_use) {
			if (bus->operation == SMB_READ_OPER) {
				SMB_ReadFromFifo(bus, READ_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_BYTES));

				/* Be prepared for new transactions                                                        */
				//bus->operation_state = SMB_IDLE;

				/* if PEC is not used or PEC is used and PEC is correct                                    */
				if (bus->PEC_use == false ||
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
				    (REG_READ(SMBPEC(bus)) == 0)
#else
				    (bus->crc_data == 0)
#endif
				   ){
					ind = SMB_SLAVE_DONE_IND;
				}

				/* PEC value is not correct                                                                */
				else {
					ind = SMB_SLAVE_PEC_ERR_IND;
				}
				SMB_callback(bus,
							    /* Notify upper layer that illegal data received */
							    ind,
							    bus->read_index);
			}
			if (bus->operation == SMB_WRITE_OPER) {
				//bus->operation_state = SMB_IDLE;
				SMB_callback(bus, SMB_SLAVE_DONE_IND,
					     bus->write_index);
			}

			SET_REG_MASK(SMBFIF_CTS(bus), MASK_FIELD(SMBFIF_CTS_SLVRSTR) |
				     MASK_FIELD(SMBFIF_CTS_CLR_FIFO) | MASK_FIELD(SMBFIF_CTS_RXF_TXE));
		}

		/* FIFO is not used                                                                                */
		else {
			if (bus->operation == SMB_READ_OPER) {

				/* if PEC is not used or PEC is used and PEC is correct                                    */
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
				if (bus->PEC_use == false ||
				    (REG_READ(SMBPEC(bus)) == 0))
#else
					if (bus->PEC_use == false ||
					    (bus->crc_data == 0))
#endif
						/* Notify upper layer of missing data or all data received */
						ind = SMB_SLAVE_DONE_IND;
					/* PEC value is not correct                                                                */
				else
					ind = SMB_SLAVE_PEC_ERR_IND;

				SMB_callback(bus, ind, bus->read_index);
			} else
				//bus->operation_state = SMB_IDLE;
				SMB_callback(bus, SMB_SLAVE_DONE_IND,
					     bus->write_index);
		}

		return;
	}

	/* A Slave restart Condition has been identified                                                       */
	if (bus->fifo_use && READ_REG_FIELD(SMBFIF_CTS(bus), SMBFIF_CTS_SLVRSTR)) {
		ASSERT(bus->master_or_slave == SMB_SLAVE);

		if (bus->operation == SMB_READ_OPER) {
			SMB_ReadFromFifo(bus, READ_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_BYTES));
		}
		REG_WRITE(SMBFIF_CTS(bus), MASK_FIELD(SMBFIF_CTS_SLVRSTR));
	}

	/* A Slave Address Match has been identified                                                           */
	if (READ_REG_FIELD(SMBST(bus), SMBST_NMATCH)) {
		bool slave_tx;
		SMB_STATE_IND_T ind = SMB_NO_STATUS_IND;
		u8 info = 0;

		if (bus->fifo_use == false)
			REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_NMATCH));

		if (READ_REG_FIELD(SMBST(bus), SMBST_XMIT))
			slave_tx = true;
		else
			slave_tx = false;

		if (bus->operation_state == SMB_IDLE) {
			/* Indicate Slave Mode                                                                         */
			if (slave_tx)
				ind = SMB_SLAVE_XMIT_IND;
			else
				ind = SMB_SLAVE_RCV_IND;

			/* Check which type of address match                                                           */
			if (READ_REG_FIELD(SMBCST(bus), SMBCST_MATCH)) {
				u16 address_match = ((REG_READ(SMBCST3(bus)) & 0x7) << 7) |
				    (REG_READ(SMBCST2(bus)) & 0x7F);
				info = 0;
				ASSERT(address_match);
				while (address_match) {
					if (address_match & 1)
						break;
					info++;
					address_match = address_match >> 1;
				}

				bus->SMB_CurSlaveAddr = READ_VAR_FIELD(SMB_GetSlaveAddress_l(bus, (SMB_ADDR_T)info), SMBADDRx_ADDR);
				if (READ_VAR_BIT(bus->PEC_mask, info) == 1) {
					bus->PEC_use = true;
					bus->crc_data = 0;
					if (slave_tx)
						SMB_CalcPEC(bus, (bus->SMB_CurSlaveAddr & 0x7F) << 1 | 1);
					else
						SMB_CalcPEC(bus, (bus->SMB_CurSlaveAddr & 0x7F) << 1);
				} else
					bus->PEC_use = false;
			} else {
				if (READ_REG_FIELD(SMBCST(bus), SMBCST_GCMATCH)) {
					info = (u8)SMB_GC_ADDR;
					bus->SMB_CurSlaveAddr = 0;
				} else {
					if (READ_REG_FIELD(SMBCST(bus), SMBCST_ARPMATCH)) {
						info = (u8)SMB_ARP_ADDR;
						bus->SMB_CurSlaveAddr = 0x61;
					}
				}
			}
		} else {
			/*  Slave match can happen in two options:                                                     */
			/*  1. Start, SA, read    ( slave read without further ado).                                   */
			/*  2. Start, SA, read , data , restart, SA, read,  ... ( salve read in fragmented mode)       */
			/*  3. Start, SA, write, data, restart, SA, read, .. ( regular write-read mode)                */
			if (((bus->operation_state == SMB_OPER_STARTED) &&
			     (bus->operation == SMB_READ_OPER) &&
			     (bus->master_or_slave == SMB_SLAVE) &&
			     slave_tx) ||
			    ((bus->master_or_slave == SMB_SLAVE) &&
			     !slave_tx))
			/* slave transmit after slave receive w/o Slave Stop implies repeated start */
			{
				ind = SMB_SLAVE_RESTART_IND;
				info = (u8)(bus->read_index);
				SMB_CalcPEC(bus, (bus->SMB_CurSlaveAddr & 0x7F) << 1 | 1);
			}
		}

		/* Address match automatically implies slave mode                                                  */
		ASSERT(!READ_REG_FIELD(SMBST(bus), SMBST_MASTER));
		bus->master_or_slave = SMB_SLAVE;
		bus->operation_state = SMB_SLAVE_MATCH;

		/* Notify upper layer                                                                              */
		/* Upper layer must at this stage call the driver routine for slave tx or rx,                      */
		/* to eliminate a condition of slave being notified but not yet starting                           */
		/* transaction - and thus an endless interrupt from SDAST for the slave RCV or TX !                */
		SMB_callback(bus, ind, info);

#ifdef SMB_RECOVERY_SUPPORT

		/* By now, SMB operation state should have been changed from MATCH to SMB_OPER_STARTED.            */
		/* If state hasn't been changed already, this may suggest that the SMB slave is not ready to       */
		/* transmit or receive data.                                                                       */
		/*                                                                                                 */
		/* In addition, when using FIFO, NMATCH bit is cleared only when moving to SMB_OPER_STARTED state. */
		/* If NMATCH is not cleared, we would get an endless SMB interrupt.                                */
		/* Therefore, Abort the slave, such that SMB HW and state machine return to a default, functional  */
		/* state.                                                                                          */
		if (bus->operation_state == SMB_SLAVE_MATCH) {
			SMB_SlaveAbort(bus);
			SMB_callback(bus, SMB_BUS_ERR_IND, SMB_GetIndex(bus));
			return;
		}

		/* Slave abort data                                                                                */
		/* if the SMBus's status is not match current status register of XMIT                              */
		/* the Slave device will enter dead-lock and stall bus forever                                     */
		/* Add this check rule to avoid this condition                                                     */
		if ((bus->operation == SMB_READ_OPER  && ind == SMB_SLAVE_XMIT_IND) ||
		    (bus->operation == SMB_WRITE_OPER && ind == SMB_SLAVE_RCV_IND)) {
			SMB_SlaveAbort(bus);
			SMB_callback(bus, SMB_BUS_ERR_IND, SMB_GetIndex(bus));
			return;
		}
#endif

		/* If none of the above - BER should occur                                                         */
	}
#endif  /* !SMB_MASTER_ONLY */

#if !defined SMB_SLAVE_ONLY

	/* Address sent and requested stall occurred (Master mode)                                             */
	if (READ_REG_FIELD(SMBST(bus), SMBST_STASTR)) {
		ASSERT(READ_REG_FIELD(SMBST(bus), SMBST_MASTER));
		ASSERT(bus->master_or_slave == SMB_MASTER);

		/* Check for Quick Command SMBus protocol */
		if ((bus->write_size == SMB_BYTES_QUICK_PROT) ||
		    (bus->read_size  == SMB_BYTES_QUICK_PROT)) {

			/* No need to write any data bytes - reached here only in Quick Command                        */
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT

			/* Enable "End of Busy" interrupt before issuing a STOP condition.                             */
			SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE, 1);
#endif
			SMB_Stop(bus);


			/* Update status                                                                               */
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
			bus->operation_state = SMB_STOP_PENDING;
			bus->stop_indication = SMB_MASTER_DONE_IND;
#else
			bus->operation_state = SMB_IDLE;


			/* Notify upper layer                                                                          */
			SMB_callback(bus, SMB_MASTER_DONE_IND, 0);
#endif
		} else if (bus->read_size == 1)

			/* Receiving one byte only - set NACK after ensuring slave ACKed the address byte              */
			SMB_Nack(bus);


		/* Reset stall-after-address-byte                                                                  */
		SMB_StallAfterStart(bus, false);


		/* Clear stall only after setting STOP                                                             */
		REG_WRITE(SMBST(bus), MASK_FIELD(SMBST_STASTR));
		return;
	}
#endif  /* !SMB_SLAVE_ONLY */


	/* SDA status is set - transmit or receive, master or slave                                            */
	if (READ_REG_FIELD(SMBST(bus), SMBST_SDAST) ||
	    (bus->fifo_use &&
	     (READ_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_THST) || READ_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST)))) {
		/* Status Bit is cleared by writing to or reading from SDA (depending on current direction)        */
#if !defined SMB_SLAVE_ONLY

		/* Handle successful bus mastership                                                                */
		if (bus->master_or_slave == SMB_MASTER) {
			if (bus->operation_state == SMB_IDLE) {

				/* Perform SMB recovery in Master mode, where state is IDLE, which is an illegal state     */
				SMB_MasterAbort(bus);
				SMB_callback(bus, SMB_BUS_ERR_IND, 0);
				return;
			} else if (bus->operation_state == SMB_MASTER_START) {
				if (READ_REG_FIELD(SMBST(bus), SMBST_MASTER)) {
					u8 addr_byte = bus->dest_addr;

					bus->crc_data = 0;
					/* Check for Quick Command SMBus protocol */
					if ((bus->write_size == SMB_BYTES_QUICK_PROT) ||
					    (bus->read_size  == SMB_BYTES_QUICK_PROT))
						/* Need to stall after successful completion of sending address byte */
						SMB_StallAfterStart(bus, true);
					/* Prepare address byte */
					if (bus->write_size == 0) {
						if (bus->read_size == 1)
							/* Receiving one byte only - stall after successful completion of sending      */
							/* address byte. If we NACK here, and slave doesn't ACK the address, we might  */
							/* unintentionally NACK the next multi-byte read                               */
							SMB_StallAfterStart(bus, true);

						/* Set direction to Read */
						addr_byte |= (u8)0x1;
						bus->operation = SMB_READ_OPER;
					} else
						bus->operation = SMB_WRITE_OPER;
					/* Write the address to the bus */
					SMB_WriteByte(bus, addr_byte);
					bus->operation_state = SMB_OPER_STARTED;
				}
			} else

				/* SDA status is set - transmit or receive: Handle master mode                                 */
				if (bus->operation_state == SMB_OPER_STARTED) {
					if (bus->operation == SMB_WRITE_OPER) {
						u16 wcount;

						if ((bus->fifo_use == true))
							SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1);


						/* Master write operation - perform write of required number of bytes                  */
						if (bus->write_index == bus->write_size) {
							if ((bus->fifo_use == true) && (READ_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_BYTES) > 0))
								/* No more bytes to send (to add to the FIFO), however the FIFO is not empty   */
								/* yet. It is still in the middle of transmitting. Currency there is nothing   */
								/* to do except for waiting to the end of the transmission.                    */
								/* We will get an interrupt when the FIFO will get empty.                      */
								return;

							if (bus->read_size == 0) {
								/* all bytes have been written, in a pure write operation */
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
								/* Enable "End of Busy" interrupt. */
								SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE, 1);
#endif
								// Issue a STOP condition on the bus
								SMB_Stop(bus);
								// Clear SDA Status bit (by writing dummy byte)
								SMB_WriteByte(bus, 0xFF);

#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
								bus->operation_state = SMB_STOP_PENDING;
								bus->stop_indication = SMB_MASTER_DONE_IND;
#else
								// Reset state for new transaction
								bus->operation_state = SMB_IDLE;
								// Notify upper layer of transaction completion
								SMB_callback(bus, SMB_MASTER_DONE_IND, 0);
#endif
							} else {
								/* last write-byte written on previous interrupt - need to restart & send slave address */
								if ((bus->PEC_use == true) &&
								    (bus->read_size < SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER))   // PEC is used but the protocol is not block read protocol
																     // then we add extra bytes for PEC support
									bus->read_size += 1;

								if (bus->fifo_use == true) {
									if (((bus->read_size == 1) ||
									     bus->read_size == SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER ||
									     bus->read_size == SMB_BYTES_BLOCK_PROT)) {   // SMBus Block read transaction.

										REG_WRITE(SMBTXF_CTL(bus), 0);
										REG_WRITE(SMBRXF_CTL(bus), 1);
									} else {

										if (bus->read_size > SMBUS_FIFO_SIZE)
											SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, SMBUS_FIFO_SIZE);
										else {
											// clear the status bits
											SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, (u8)bus->read_size);
											SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_LAST_PEC, 1);
										}
									}
								}


								/* Generate (Repeated) Start upon next write to SDA */
								SMB_Start(bus);

								if (bus->read_size == 1)

									/* Receiving one byte only - stall after successful completion of sending  */
									/* address byte. If we NACK here, and slave doesn't ACK the address, we    */
									/* might unintentionally NACK the next multi-byte read                     */

									SMB_StallAfterStart(bus, true);

								/* send the slave address in read direction                                    */
								SMB_WriteByte(bus, bus->dest_addr | 0x1);

								/* Next interrupt will occur on read                                           */
								bus->operation = SMB_READ_OPER;

							}
						} else {
							if ((bus->PEC_use == true) && (bus->write_index == 0)
							    && (bus->read_size == 0))// extra bytes for PEC support
								bus->write_size += 1;

							/* write next byte not last byte and not slave address                             */
							if ((bus->fifo_use == false) || (bus->write_size == 1)) {
								if ((bus->PEC_use == true) && (bus->read_size == 0) &&
								    (bus->write_index + 1 == bus->write_size)) { // Master write protocol to send PEC byte.
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
									REG_WRITE(SMBSDA(bus), REG_READ(SMBPEC(bus)));
#else
									REG_WRITE(SMBSDA(bus), bus->crc_data);
#endif
									bus->write_index++;
								} else
									SMB_WriteByte(bus, bus->write_data_buf[bus->write_index++]);
							}
							// FIFO is used
							else {
								wcount = bus->write_size - bus->write_index;
								if (wcount > SMBUS_FIFO_SIZE)
									/* data to send is more then FIFO size.                                    */
									/* Configure the FIFO interrupt to be mid of FIFO.                         */
									REG_WRITE(SMBTXF_CTL(bus), BUILD_FIELD_VAL(SMBTXF_CTL_THR_TXIE, 1) | (SMBUS_FIFO_SIZE / 2));
								else if ((wcount > SMBUS_FIFO_SIZE / 2) && (bus->write_index != 0))
									/* write_index != 0 means that this is not the first write.                */
									/* since interrupt is in the mid of FIFO, only half of the fifo is empty.  */
									/* Continue to configure the FIFO interrupt to be mid of FIFO.             */
									REG_WRITE(SMBTXF_CTL(bus), BUILD_FIELD_VAL(SMBTXF_CTL_THR_TXIE, 1) | (SMBUS_FIFO_SIZE / 2));
								else {
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
									if ((bus->PEC_use) && (wcount > 1))
										wcount--; //put the PEC byte last after the FIFO becomes empty.
#endif
									/* This is the first write (write_index = 0) and data to send is less or   */
									/* equal to FIFO size.                                                     */
									/* Or this is the last write and data to send is less or equal half FIFO   */
									/* size.                                                                   */
									/* In both cases disable the FIFO threshold interrupt.                     */
									/* The next interrupt will happen after the FIFO will get empty.           */
									REG_WRITE(SMBTXF_CTL(bus), (u8)0);
								}

								SMB_WriteToFifo(bus, wcount);
								SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1); //clear status bit
#ifdef SMB_STALL_TIMEOUT_SUPPORT
								bus->stall_counter = 0;
#endif
							}
						}
					} else if (bus->operation == SMB_READ_OPER) {
						u16 block_zero_bytes;
						/* Master read operation (pure read or following a write operation).                   */

						/* Initialize number of bytes to include only the first byte (presents a case where    */
						/* number of bytes to read is zero); add PEC if applicable                             */
						block_zero_bytes = 1;
						if (bus->PEC_use == true)
							block_zero_bytes++;

						/* Perform master read, distinguishing between last byte and the rest of the           */
						/* bytes. The last byte should be read when the clock is stopped                       */
						if ((bus->read_index < (bus->read_size - 1)) ||
						    bus->fifo_use == true) {
							u8 data;

							/* byte to be read is not the last one                                             */
							/* Check if byte-before-last is about to be read                                   */
							if ((bus->read_index == (bus->read_size - 2)) &&
							    bus->fifo_use == false)

								/* Set nack before reading byte-before-last, so that nack will be generated    */
								/* after receive of last byte                                                  */
								SMB_Nack(bus);

							//if (!SMB_ReadByte(bus, &data))
							if (!READ_REG_FIELD(SMBST(bus), SMBST_SDAST)) {
								/* No data available - reset state for new transaction                         */
								bus->operation_state = SMB_IDLE;

								/* Notify upper layer of transaction completion                                */
								SMB_callback(bus, SMB_NO_DATA_IND, bus->read_index);
							} else if (bus->read_index == 0) {
								if (bus->read_size == SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER ||
								    bus->read_size == SMB_BYTES_BLOCK_PROT) {
									(void)SMB_ReadByte(bus, &data);

									/* First byte indicates length in block protocol                           */
									if (bus->read_size == SMB_BYTES_EXCLUDE_BLOCK_SIZE_FROM_BUFFER)
										bus->read_size = data;
									else {
										bus->read_data_buf[bus->read_index++] = data;
										bus->read_size = data + 1;
									}

									if (bus->PEC_use == true) {
										bus->read_size += 1;
										data += 1;
									}

									if (bus->fifo_use == true) {
										SET_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_THST, 1);
										SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1);
										//SET_REG_FIELD(SMBFIF_CTS(bus), SMBFIF_CTS_CLR_FIFO, 1);
										SET_REG_FIELD(SMBFIF_CTS(bus), SMBFIF_CTS_RXF_TXE, 1);
										if (data > SMBUS_FIFO_SIZE)
											SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, SMBUS_FIFO_SIZE);
										else {
											if (data == 0)
												data = 1;

											/* clear the status bits                                           */
											SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR, (u8)data);
											SET_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_LAST_PEC, 1);
										}
									}
								} else {
									if (bus->fifo_use == false) {
										(void)SMB_ReadByte(bus, &data);
										bus->read_data_buf[bus->read_index++] = data;
									} else {
										SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1);
										SMB_MasterFifoRead(bus);
									}
								}

							} else {
								if (bus->fifo_use == true) {   // FIFO in used.
									if ((bus->read_size == block_zero_bytes) && (bus->read_block_use == true)) {
#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
										/* Enable "End of Busy" interrupt                                      */
										SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE, 1);
#endif
										SMB_Stop(bus);

										SMB_ReadFromFifo(bus, READ_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR));

#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
										bus->operation_state = SMB_STOP_PENDING;
										bus->stop_indication = SMB_MASTER_BLOCK_BYTES_ERR_IND;
#else
										/* Reset state for new transaction                                     */
										bus->operation_state = SMB_IDLE;

										/* Notify upper layer of transaction completion                        */
										SMB_callback(bus, SMB_MASTER_BLOCK_BYTES_ERR_IND, bus->read_index);
#endif
									} else
										SMB_MasterFifoRead(bus);
								} else {
									(void)SMB_ReadByte(bus, &data);
									bus->read_data_buf[bus->read_index++] = data;
								}
							}
						} else {
							/* last byte is about to be read - end of transaction.                             */
							/* Stop should be set before reading last byte.                                    */
							u8 data;
							SMB_STATE_IND_T ind = SMB_MASTER_DONE_IND;

#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
							/* Enable "End of Busy" interrupt.                                                 */
							SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_EOBINTE, 1);
#endif
							SMB_Stop(bus);

							(void)SMB_ReadByte(bus, &data);

							if ((bus->read_size == block_zero_bytes) && (bus->read_block_use == true))
								ind = SMB_MASTER_BLOCK_BYTES_ERR_IND;
							else {
								bus->read_data_buf[bus->read_index++] = data;
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
								if ((bus->PEC_use == true) && (REG_READ(SMBPEC(bus)) != 0))
#else
									if ((bus->PEC_use == true) && (bus->crc_data != 0))
#endif
										ind = SMB_MASTER_PEC_ERR_IND;

							}

#ifdef SMB_CAPABILITY_END_OF_BUSY_SUPPORT
							bus->operation_state = SMB_STOP_PENDING;
							bus->stop_indication = ind;
#else
							/* Reset state for new transaction                                                 */
							bus->operation_state = SMB_IDLE;

							/* Notify upper layer of transaction completion                                    */
							SMB_callback(bus, ind, bus->read_index);
#endif
						} /* last read byte */
					} /* read operation */
					// Edward 2014/6/17 masked.
				} /* Master mode: if (bus->operation_state == SMB_MASTER_START) */
			// End of Edward 2014/6/17 masked.
		} // End of master operation: SDA status is set - transmit or receive.
#if !defined SMB_MASTER_ONLY
else
#endif
#endif  /* !SMB_SLAVE_ONLY */

#if !defined SMB_MASTER_ONLY
			/* SDA status is set - transmit or receive: Handle slave mode                                      */
			if (bus->master_or_slave == SMB_SLAVE) {
				/* Perform slave read. No need to distinguish between last byte and the rest of the bytes.     */
				if ((bus->operation == SMB_READ_OPER)) {
					if (bus->fifo_use == false) {
						u8 data;

						(void)SMB_ReadByte(bus, &data);
						if (bus->read_index < bus->read_size) {
							/* Keep read data                                                                  */
							bus->read_data_buf[bus->read_index++] = data;
							if ((bus->read_index == 1) && bus->read_size == SMB_BYTES_BLOCK_PROT)
								/* First byte indicates length in block protocol                               */
								bus->read_size = data;
						}
					}
					// FIFO is used
					else {
						if (READ_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_THST)) {
							SMB_ReadFromFifo(bus, READ_REG_FIELD(SMBRXF_CTL(bus), SMBRXF_CTL_RX_THR));

							/* clear the status bits                                                           */
							SET_REG_FIELD(SMBRXF_STS(bus), SMBRXF_STS_RX_THST, 1);
						}
					}
				}
				/* Perform slave write.                                                                        */
				else {
					/* More bytes to write                                                                     */
					if ((bus->operation == SMB_WRITE_OPER) &&
					    (bus->write_index < bus->write_size)) {
						if (bus->fifo_use == false) {
							if ((bus->write_index + 1 == bus->write_size) &&
							    bus->PEC_use == true) {   // Send PEC byte
#if defined SMB_CAPABILITY_HW_PEC_SUPPORT
								REG_WRITE(SMBSDA(bus), REG_READ(SMBPEC(bus)));
#else
								REG_WRITE(SMBSDA(bus), bus->crc_data);
#endif
							} else if (bus->write_index < bus->write_size)
								SMB_WriteByte(bus, bus->write_data_buf[bus->write_index]);
							bus->write_index++;
						}
						// FIFO is used
						else {
							u16 wcount;
							wcount =  (bus->write_size - bus->write_index);
							if (wcount >= SMBUS_FIFO_SIZE)
								wcount = SMBUS_FIFO_SIZE;

							REG_WRITE(SMBTXF_CTL(bus), (u8)wcount);
							SMB_WriteToFifo(bus, wcount);

							/* clear the status bits                                                           */
							SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1);
						}
					}

					/* If all bytes were written, ignore further master read requests.                         */
					else {
#if !defined(SMB_WRAP_AROUND_BUFFER)
						ASSERT(false);
#endif
						if (bus->fifo_use == false) {
							/* Clear SDA Status bit                                                            */
							if (bus->write_index != 0)
								/* Was writing                                                                 */
								SMB_WriteByte(bus, 0xFF);
							else {
								u8 data;
								/* Was reading                                                                 */
								(void)SMB_ReadByte(bus, &data);
							}
						}
						/* write\read redundant bytes with FIFO (if there are any bytes to write)              */
						else {
							/* Set threshold size                                                              */
							REG_WRITE(SMBTXF_CTL(bus), (u8)SMBUS_FIFO_SIZE);

							SMB_WriteToFifo(bus, SMBUS_FIFO_SIZE);

							/* Clear the status bits                                                           */
							SET_REG_FIELD(SMBTXF_STS(bus), SMBTXF_STS_TX_THST, 1);
						}
						/* Notify upper layer of transaction completion                                        */
						SMB_callback(bus, SMB_NO_DATA_IND, bus->read_index);
					} // All bytes sent/received
				}
			} // slave mode
#endif  /* !SMB_MASTER_ONLY */
	} //SDAST
}

static void SMB_Reset(nuvoton_i2c_bus_t *bus)
{
	/* Save smbctl1 relevant bits. It is being cleared when the module is disabled */
	u8 smbctl1 = REG_READ(SMBCTL1(bus)) & (MASK_FIELD(SMBCTL1_GCMEN) |
					       MASK_FIELD(SMBCTL1_INTEN) |
					       MASK_FIELD(SMBCTL1_NMINTE));

	/* Disable the SMB module */
	SET_REG_FIELD(SMBCTL2(bus), SMBCTL2_ENABLE, DISABLE);

	/* Enable the SMB module */
	(void)SMB_Enable(bus);

	/* Restore smbctl1 status */
	REG_WRITE(SMBCTL1(bus),  smbctl1);

	/* Reset driver status */
	bus->operation_state = SMB_IDLE;
}


#if !defined SMB_SLAVE_ONLY
static void SMB_MasterAbort(nuvoton_i2c_bus_t *bus)
{
	SMB_AbortData(bus);
	SMB_Reset(bus);
}

#ifdef TBD
static void SMB_Recovery(nuvoton_i2c_bus_t *bus)
{

	/* Disable interrupt   */
	SMB_InterruptEnable(bus, false);

	/* Check If the SDA line is active (low) */
	if (READ_REG_FIELD(SMBCST(bus), SMBCST_TSDA) == 0) {
		u8   iter = 9;   // Allow one byte to be sent by the Slave
		u16  timeout;
		bool done = false;

		/* Repeat the following sequence until SDA becomes inactive (high) */
		do {
			/* Issue a single SCL cycle */
			REG_WRITE(SMBCST(bus), MASK_FIELD(SMBCST_TGSCL));
			timeout = ABORT_TIMEOUT;
			while (READ_REG_FIELD(SMBCST(bus), SMBCST_TGSCL) && (--timeout != 0))
				;
			/* If SDA line is inactive (high), stop */
			if (READ_REG_FIELD(SMBCST(bus), SMBCST_TSDA) == 1)
				done = true;
		} while ((done == false) && (--iter != 0));

		/* If SDA line is released (high) */
		if (done) {
			/* Clear BB (BUS BUSY) bit */
			REG_WRITE(SMBCST(bus), MASK_FIELD(SMBCST_BB));

			/* Generate a START condition, to synchronize Master and Slave */
			SMB_Start(bus);

			/* Wait until START condition is sent, or timeout */
			timeout = ABORT_TIMEOUT;
			while (!READ_REG_FIELD(SMBST(bus), SMBST_MASTER) && (--timeout != 0))
				;

			/* If START condition was sent */
			if (timeout > 0) {
				/* Send an address byte */
				SMB_WriteByte(bus, bus->dest_addr);

				/* Generate a STOP condition */
				SMB_Stop(bus);
			}
		}
	}

	/* Enable interrupt  */
	SMB_InterruptEnable(bus, true);
}
#endif // TBD
#endif  /* !SMB_SLAVE_ONLY */

static void SMB_InterruptEnable(nuvoton_i2c_bus_t *bus, bool enable)
{
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_INTEN, enable);
}

#if !defined SMB_MASTER_ONLY && defined SMB_RECOVERY_SUPPORT
static void SMB_SlaveAbort(nuvoton_i2c_bus_t *bus)
{
	volatile u8 temp;

	/* Disable interrupt. */
	SMB_InterruptEnable(bus, false);

	/* Dummy read to clear interface. */
	temp = REG_READ(SMBSDA(bus));

	/* Clear NMATCH and BER bits by writing 1s to them. */
	SET_REG_FIELD(SMBST(bus), SMBST_BER, true);
	SET_REG_FIELD(SMBST(bus), SMBST_NMATCH, true);

#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter = 0;
#endif

	/* Reset driver status */
	bus->operation_state = SMB_IDLE;

	/* Disable SMB Module */
	SET_REG_FIELD(SMBCTL2(bus), SMBCTL2_ENABLE, DISABLE);

	/* Delay 100 us */
	udelay(10); // TBD must be out of interrupt

	/* Enable SMB Module */
	(void)SMB_Enable(bus);

	/* Enable interrupt. */
	SMB_InterruptEnable(bus, true);

	//lint -e{550} suppress PC-Lint warning on Symbol 'temp' not accessed
}
#endif //!defined SMB_MASTER_ONLY && defined SMB_RECOVERY_SUPPORT

#if defined (SMB_CAPABILITY_WAKEUP_SUPPORT)
static void SMB_SetStallAfterStartIdle(nuvoton_i2c_bus_t *bus, bool enable)
{
	SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_IDL_START, enable);
}
#endif //SMB_CAPABILITY_WAKEUP_SUPPORT

#if !defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
static const u8 crc8_table[256] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

static u8 SMB_CalculateCRC8(u8 crc_data, u8 data)
{
	u8 tmp = crc_data ^ data;

	crc_data = crc8_table[tmp];

	return crc_data;
}
#endif // #if !defined (SMB_CAPABILITY_HW_PEC_SUPPORT)

static void SMB_CalcPEC(nuvoton_i2c_bus_t *bus, u8 data)
{
#if !defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
	if (bus->PEC_use)
		bus->crc_data = SMB_CalculateCRC8(bus->crc_data, data);
#endif
}


#ifdef SMB_STALL_TIMEOUT_SUPPORT
static void SMB_ConfigStallThreshold(nuvoton_i2c_bus_t *bus, u8 threshold)
{
	bus->stall_threshold = threshold;
}

static void SMB_StallHandler(nuvoton_i2c_bus_t *bus)
{
	if ((bus->operation_state == SMB_IDLE) ||
	    (bus->operation_state == SMB_DISABLE) ||
	    (bus->master_or_slave == SMB_SLAVE))
		; // ignore this bus
	else {
		/* increase timeout counter                                                                    */
		bus->stall_counter++;

		/* time expired, execute recovery */
		if ((bus->stall_counter) >= bus->stall_threshold) {
			SMB_MasterAbort(bus);
			SMB_callback(bus, SMB_BUS_ERR_IND, SMB_GetIndex(bus));
			return;
		}
	}
}
#endif

#ifdef TBD
static void SMB_ReEnableModule(nuvoton_i2c_bus_t *bus)
{
	/* Enable SMB interrupt and New Address Match interrupt source */
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_NMINTE, ENABLE);
	SET_REG_FIELD(SMBCTL1(bus), SMBCTL1_INTEN,  ENABLE);
}

static bool SMB_InterruptIsPending(void)
{
	nuvoton_i2c_bus_t *bus;
	bool         InterruptIsPending = false;

	for (bus = 0; bus < SMB_NUM_OF_MODULES; bus++)
		InterruptIsPending |= INTERRUPT_PENDING(SMB_INTERRUPT_PROVIDER,
	SMB_INTERRUPT(bus));

	return InterruptIsPending;
}
#endif

#ifdef SMB_CAPABILITY_FORCE_SCL_SDA
static void SMB_WriteSCL(nuvoton_i2c_bus_t *bus, SMB_LEVEL_T level)
{
	unsigned long bank_flags;

	/* Select Bank 0 to access SMBCTL4 */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

	/* Set SCL_LVL, SDA_LVL bits as Read/Write (R/W) */
	SET_REG_FIELD(SMBCTL4(bus), SMBCTL4_LVL_WE, 1);

	/* Set level */
	SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_SCL_LVL, level);

	/* Set SCL_LVL, SDA_LVL bits as Read Only (RO) */
	SET_REG_FIELD(SMBCTL4(bus), SMBCTL4_LVL_WE, 0);

	/* Return to Bank 1 */
	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);
}

static void SMB_WriteSDA(nuvoton_i2c_bus_t *bus, SMB_LEVEL_T level)
{
	unsigned long bank_flags;

	/* Select Bank 0 to access SMBCTL4 */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

	/* Set SCL_LVL, SDA_LVL bits as Read/Write (R/W) */
	SET_REG_FIELD(SMBCTL4(bus), SMBCTL4_LVL_WE, 1);

	/* Set level */
	SET_REG_FIELD(SMBCTL3(bus), SMBCTL3_SDA_LVL, level);

	/* Set SCL_LVL, SDA_LVL bits as Read Only (RO) */
	SET_REG_FIELD(SMBCTL4(bus), SMBCTL4_LVL_WE, 0);

	/* Return to Bank 1 */
	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);
}
#endif // SMB_CAPABILITY_FORCE_SCL_SDA

#ifdef CONFIG_NPCM750_I2C_DEBUG_PRINT
static void SMB_PrintRegs(nuvoton_i2c_bus_t *bus)
{
	unsigned int i;

	HAL_PRINT("/*--------------*/\n");
	HAL_PRINT("/*     SMB      */\n");
	HAL_PRINT("/*--------------*/\n\n");

	SMB_PrintModuleRegs(bus);

	//lint -e{774, 506} suppress PC-Lint warning on ''if' always evaluates to true'
	if (SMB_FIFO(bus)) {
		HAL_PRINT("/*--------------*/\n");
		HAL_PRINT("/*     SMBF     */\n");
		HAL_PRINT("/*--------------*/\n\n");

		SMBF_PrintModuleRegs(bus);
	}
}

static void SMB_PrintModuleRegs(nuvoton_i2c_bus_t *bus)
{
	HAL_PRINT("SMB%d:\n", bus->module__num);
	HAL_PRINT("------\n");
	HAL_PRINT("SMB%dSDA             = 0x%02X\n", bus->module__num,
		  REG_READ(SMBSDA(bus)));
	HAL_PRINT("SMB%dST              = 0x%02X\n", bus->module__num,
		  REG_READ(SMBST(bus)));
	HAL_PRINT("SMB%dCST             = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCST(bus)));
	HAL_PRINT("SMB%dCTL1            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL1(bus)));
	HAL_PRINT("SMB%dADDR1           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR1(bus)));
	HAL_PRINT("SMB%dCTL2            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL2(bus)));
	HAL_PRINT("SMB%dADDR2           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR2(bus)));
	HAL_PRINT("SMB%dCTL3            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL3(bus)));
#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)
	HAL_PRINT("SMB%dT_OUT           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBT_OUT(bus)));
#endif
	HAL_PRINT("SMB%dADDR3           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR3(bus)));
	HAL_PRINT("SMB%dADDR7           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR7(bus)));
	HAL_PRINT("SMB%dADDR4           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR4(bus)));
	HAL_PRINT("SMB%dADDR8           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR8(bus)));
	HAL_PRINT("SMB%dADDR5           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR5(bus)));
	HAL_PRINT("SMB%dADDR9           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR9(bus)));
	HAL_PRINT("SMB%dADDR6           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR6(bus)));
	HAL_PRINT("SMB%dADDR10          = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR10(bus)));
	HAL_PRINT("SMB%dCST2            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCST2(bus)));
	HAL_PRINT("SMB%dCST3            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCST3(bus)));
	HAL_PRINT("SMB%dCTL4            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL4(bus)));
	HAL_PRINT("SMB%dCTL5            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL5(bus)));
	HAL_PRINT("SMB%dSCLLT           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBSCLLT(bus)));
	HAL_PRINT("SMB%dSCLHT           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBSCLHT(bus)));
	HAL_PRINT("SMB%dVER             = 0x%02X\n", bus->module__num,
		  REG_READ(SMB_VER(bus)));

	HAL_PRINT("\n");
}

static void SMBF_PrintModuleRegs(nuvoton_i2c_bus_t *bus)
{

	/* Common Registers                                                                                    */
	HAL_PRINT("SMB%1X:\n", bus->module__num);
	HAL_PRINT("------\n");
	HAL_PRINT("SMB%1XSDA             = 0x%02X\n", bus->module__num,
		  REG_READ(SMBSDA(bus)));
	HAL_PRINT("SMB%1XST              = 0x%02X\n", bus->module__num,
		  REG_READ(SMBST(bus)));
	HAL_PRINT("SMB%1XCST             = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCST(bus)));
	HAL_PRINT("SMB%1XCTL1            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL1(bus)));
	HAL_PRINT("SMB%1XADDR1           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR1(bus)));
	HAL_PRINT("SMB%1XCTL2            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL2(bus)));
	HAL_PRINT("SMB%1XADDR2           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR2(bus)));
	HAL_PRINT("SMB%1XCTL3            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL3(bus)));
#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)
	HAL_PRINT("SMB%1XT_OUT           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBT_OUT(bus)));
#endif


	/* Bank 0 Registers                                                                                    */
	spin_lock_irqsave(&bus->bank_lock, bank_flags);
	SMB_SelectBank(bus, SMB_BANK_0);

	HAL_PRINT("SMB%1XADDR3           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR3(bus)));
	HAL_PRINT("SMB%1XADDR7           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR7(bus)));
	HAL_PRINT("SMB%1XADDR4           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR4(bus)));
	HAL_PRINT("SMB%1XADDR8           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR8(bus)));
	HAL_PRINT("SMB%1XADDR5           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR5(bus)));
	HAL_PRINT("SMB%1XADDR9           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR9(bus)));
	HAL_PRINT("SMB%1XADDR6           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR6(bus)));
	HAL_PRINT("SMB%1XADDR10          = 0x%02X\n", bus->module__num,
		  REG_READ(SMBADDR10(bus)));
	HAL_PRINT("SMB%1XCST2            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCST2(bus)));
	HAL_PRINT("SMB%1XCST3            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCST3(bus)));
	HAL_PRINT("SMB%1XCTL4            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL4(bus)));
	HAL_PRINT("SMB%1XCTL5            = 0x%02X\n", bus->module__num,
		  REG_READ(SMBCTL5(bus)));
	HAL_PRINT("SMB%1XSCLLT           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBSCLLT(bus)));
	HAL_PRINT("SMB%1XFIF_CTL         = 0x%02X\n", bus->module__num,
		  REG_READ(SMBFIF_CTL(bus)));
	HAL_PRINT("SMB%1XSCLHT           = 0x%02X\n", bus->module__num,
		  REG_READ(SMBSCLHT(bus)));
	HAL_PRINT("SMB%1X_VER            = 0x%02X\n", bus->module__num,
		  REG_READ(SMB_VER(bus)));


	/* Bank 1 Registers                                                                                    */

	SMB_SelectBank(bus, SMB_BANK_1);
	spin_unlock_irqrestore(&bus->bank_lock, bank_flags);

	HAL_PRINT("SMB%1XFIF_CTS         = 0x%02X\n", bus->module__num,
		  REG_READ(SMBFIF_CTS(bus)));
	HAL_PRINT("SMB%1XTXF_CTL         = 0x%02X\n", bus->module__num,
		  REG_READ(SMBTXF_CTL(bus)));
#if defined (SMB_CAPABILITY_HW_PEC_SUPPORT)
	HAL_PRINT("SMB%1XPEC             = 0x%02X\n", bus->module__num,
		  REG_READ(SMBPEC(bus)));
#endif
	HAL_PRINT("SMB%1XTXF_STS         = 0x%02X\n", bus->module__num,
		  REG_READ(SMBTXF_STS(bus)));
	HAL_PRINT("SMB%1XRXF_STS         = 0x%02X\n", bus->module__num,
		  REG_READ(SMBRXF_STS(bus)));
	HAL_PRINT("SMB%1XRXF_CTL         = 0x%02X\n", bus->module__num,
		  REG_READ(SMBRXF_CTL(bus)));

	HAL_PRINT("\n");
}

static void SMB_PrintVersion(void)
{
	HAL_PRINT("SMB         = %s\n", I2C_VERSION);
}
#endif //CONFIG_NPCM750_I2C_DEBUG_PRINT

static void SMB_callback(nuvoton_i2c_bus_t *bus, SMB_STATE_IND_T op_status, u16 info)
{
	switch (op_status) {
#if !defined SMB_MASTER_ONLY
		extern u8  read_data_buf[PAGE_SIZE];
		extern u16 read_size;
		extern u8  write_data_buf[32];
		extern u16 write_size;
	case SMB_SLAVE_RCV_IND:
		// Slave got an address match with direction bit clear so it should receive data
		//     the interrupt must call SMB_StartSlaveReceive()
		// info: the enum SMB_ADDR_T address match
		SMB_StartSlaveReceive(bus, read_size, read_data_buf);
		break;
	case SMB_SLAVE_XMIT_IND:
		// Slave got an address match with direction bit set so it should transmit data
		//     the interrupt must call SMB_StartSlaveTransmit()
		// info: the enum SMB_ADDR_T address match
		SMB_StartSlaveTransmit(bus, write_size, write_data_buf);
		break;
	case SMB_SLAVE_DONE_IND:
		// Slave done transmitting or receiving
		// info:
		//     on receive: number of actual bytes received
		//     on transmit: number of actual bytes transmitted,
		//                  when PEC is used 'info' should be (nwrite+1) which means that 'nwrite' bytes
		//                     were sent + the PEC byte
		//                     'nwrite' is the second parameter SMB_StartSlaveTransmit()
		break;
#endif // !defined SMB_MASTER_ONLY
	case SMB_MASTER_DONE_IND:
		// Master transaction finished and all transmit bytes were sent
		// info: number of bytes actually received after the Master receive operation
		//       (if Master didn't issue receive it should be 0)
	case SMB_NO_DATA_IND:
		// Notify that not all data was received on Master or Slave
		// info:
		//     on receive: number of actual bytes received
		//                 when PEC is used even if 'info' is the expected number of bytes,
		//                     it means that PEC error occured.
		{
			struct i2c_msg *msgs = bus->msgs;
			int msgs_num = bus->msgs_num;

			if (msgs[0].flags & I2C_M_RD)
				msgs[0].len = info;
			else if (msgs_num == 2 && msgs[1].flags & I2C_M_RD)
				msgs[1].len = info;

			bus->cmd_err = 0;
			complete(&bus->cmd_complete);
		}
		break;
	case SMB_NACK_IND:
		// MASTER transmit got a NAK before transmitting all bytes
		// info: number of transmitted bytes
		bus->cmd_err = -EAGAIN;
		complete(&bus->cmd_complete);
		break;
	case SMB_BUS_ERR_IND:
		// Bus error occured
		// info: has no meaning
		bus->cmd_err = -EIO;
		complete(&bus->cmd_complete);
		break;
	case SMB_WAKE_UP_IND:
		// SMBus wake up occured
		// info: has no meaning
		break;
	default:
		break;
	}
}


static int __nuvoton_i2c_init(struct nuvoton_i2c_bus *bus,
			      struct platform_device *pdev)
{
	u32 clk_freq;
	int ret;

	/* Initialize the internal data structures */
	bus->operation_state = SMB_DISABLE;
	bus->master_or_slave = SMB_SLAVE;
#ifdef SMB_STALL_TIMEOUT_SUPPORT
	bus->stall_counter   = 0;
	bus->stall_threshold = DEFAULT_STALL_COUNT;
#endif


	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &clk_freq);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		clk_freq = 100000;
	}
	I2C_DEBUG("clk_freq = %d\n", clk_freq);
	ret = SMB_InitModule(bus, SMB_MASTER, clk_freq / 1000);
	if (ret == false) {
		dev_err(&pdev->dev,
			"SMB_InitModule() failed\n");
		return -1;
	}
#if defined (SMB_CAPABILITY_TIMEOUT_SUPPORT)
	SMB_EnableTimeout(bus, true);
#endif //SMB_CAPABILITY_TIMEOUT_SUPPORT

#ifdef CONFIG_I2C_SLAVE
	/* If slave has already been registered, re-enable it. */
	if (bus->slave)
		__nuvoton_i2c_reg_slave(bus, bus->slave->addr);
#endif /* CONFIG_I2C_SLAVE */

	return 0;
}


static irqreturn_t nuvoton_i2c_bus_irq(int irq, void *dev_id)
{
	struct nuvoton_i2c_bus *bus = dev_id;

#ifdef SMB_SW_BYPASS_HW_ISSUE_SMB_STOP
	_npcm7xx_get_time_stamp(bus->interrupt_time_stamp);
#endif
	SMB_InterruptHandler(bus);

#ifdef CONFIG_I2C_SLAVE
	if (nuvoton_i2c_slave_irq(bus)) {
		dev_dbg(bus->dev, "irq handled by slave.\n");
		return IRQ_HANDLED;
	}
#endif /* CONFIG_I2C_SLAVE */

	return IRQ_HANDLED;
}


static int nuvoton_i2c_master_xfer(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num)
{
	struct nuvoton_i2c_bus *bus = adap->algo_data;
	struct i2c_msg *msg0, *msg1;
	unsigned long time_left, lock_flags;
	u16 nwrite, nread;
	u8 *write_data, *read_data;
	u8 slave_addr;
	int ret = 0;

	//spin_lock_irqsave(&bus->lock, flags);
	bus->cmd_err = 0;

	if (num > 2 || num < 1) {
		pr_err(" num = %d > 2.\n", num);
		return -1;
	}

	msg0 = &msgs[0];
	slave_addr = msg0->addr;
	if (msg0->flags & I2C_M_RD) { // read
		if (num == 2) {
			pr_err(" num = 2 but first msg is read instead of "
			       "write.\n");
			return -1;
		}
		nwrite = 0;
		write_data = NULL;
		if (msg0->flags & I2C_M_RECV_LEN)
			nread = SMB_BYTES_BLOCK_PROT;
		else
			nread = msg0->len;

		read_data = msg0->buf;
	} else { // write
		nwrite = msg0->len;
		write_data = msg0->buf;
		nread = 0;
		read_data = NULL;
		if (num == 2) {
			msg1 = &msgs[1];
			if (slave_addr != msg1->addr) {
				pr_err(" slave_addr == %02x but msg1->addr == "
				       "%02x\n", slave_addr, msg1->addr);
				return -1;
			}
			if ((msg1->flags & I2C_M_RD) == 0) {
				pr_err(" num = 2 but both msg are write.\n");
				return -1;
			}
			if (msg1->flags & I2C_M_RECV_LEN)
				nread = SMB_BYTES_BLOCK_PROT;
			else
				nread = msg1->len;

			read_data = msg1->buf;
		}
	}

	bus->msgs = msgs;
	bus->msgs_num = num;

	if (nwrite == 0 && nread == 0)
		nwrite = nread = SMB_BYTES_QUICK_PROT;

	reinit_completion(&bus->cmd_complete);
	SMB_StartMasterTransaction(bus, slave_addr, nwrite, nread, write_data,
				   read_data, 0);
	//spin_unlock_irqrestore(&bus->lock, flags);

	time_left = wait_for_completion_timeout(&bus->cmd_complete,
						bus->adap.timeout);

	if (time_left == 0){
        SMB_MasterAbort(bus);
		ret = -ETIMEDOUT;
	}
	else
		ret = bus->cmd_err;

	spin_lock_irqsave(&bus->lock, lock_flags);
#ifdef CONFIG_NPCM750_I2C_DEBUG
	if (bus->msgs[0].flags & I2C_M_RD)
		nread = bus->msgs[0].len;
	else if (bus->msgs_num == 2 && bus->msgs[1].flags & I2C_M_RD)
		nread = bus->msgs[1].len;
	if (nread && nread != SMB_BYTES_QUICK_PROT) {
		int i;
		char str[32 * 3 + 4];
		char *s = str;

		for (i = 0; (i < nread && i < 32); i++)
			s += sprintf(s, "%02x ", read_data[i]);

		printk("read_data  = %s\n", str);
	}
#endif


	bus->msgs = NULL;
	bus->msgs_num = 0;
	spin_unlock_irqrestore(&bus->lock, lock_flags);

	/* If nothing went wrong, return number of messages transferred. */
	if (ret >= 0)
		return num;
	else
		return ret;
}

static u32 nuvoton_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

#ifdef CONFIG_I2C_SLAVE
static void __nuvoton_i2c_reg_slave(struct nuvoton_i2c_bus *bus, u16 slave_addr)
{
	u32 addr_reg_val, func_ctrl_reg_val;

	/* Set slave addr. */
	addr_reg_val = readl(bus->base + ASPEED_I2C_DEV_ADDR_REG);
	addr_reg_val &= ~ASPEED_I2CD_DEV_ADDR_MASK;
	addr_reg_val |= slave_addr & ASPEED_I2CD_DEV_ADDR_MASK;
	writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);

	/* Turn on slave mode. */
	func_ctrl_reg_val = readl(bus->base + ASPEED_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val |= ASPEED_I2CD_SLAVE_EN;
	writel(func_ctrl_reg_val, bus->base + ASPEED_I2C_FUN_CTRL_REG);
}

static int nuvoton_i2c_reg_slave(struct i2c_client *client)
{
	struct nuvoton_i2c_bus *bus;
	unsigned long lock_flags;

	bus = client->adapter->algo_data;
	spin_lock_irqsave(&bus->lock, lock_flags);
	if (bus->slave) {
		spin_unlock_irqrestore(&bus->lock, lock_flags);
		return -EINVAL;
	}

	__nuvoton_i2c_reg_slave(bus, client->addr);

	bus->slave = client;
	bus->slave_state = ASPEED_I2C_SLAVE_STOP;
	spin_unlock_irqrestore(&bus->lock, lock_flags);

	return 0;
}

static int nuvoton_i2c_unreg_slave(struct i2c_client *client)
{
	struct nuvoton_i2c_bus *bus = client->adapter->algo_data;
	u32 func_ctrl_reg_val;
	unsigned long lock_flags;

	spin_lock_irqsave(&bus->lock, lock_flags);

	if (!bus->slave) {
		spin_unlock_irqrestore(&bus->lock, lock_flags);
		return -EINVAL;
	}

	/* Turn off slave mode. */
	func_ctrl_reg_val = readl(bus->base + ASPEED_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val &= ~ASPEED_I2CD_SLAVE_EN;
	writel(func_ctrl_reg_val, bus->base + ASPEED_I2C_FUN_CTRL_REG);

	bus->slave = NULL;
	spin_unlock_irqrestore(&bus->lock, lock_flags);

	return 0;
}
#endif /* CONFIG_I2C_SLAVE */

static const struct i2c_algorithm nuvoton_i2c_algo = {
	.master_xfer	= nuvoton_i2c_master_xfer,
	.functionality	= nuvoton_i2c_functionality,
#ifdef CONFIG_I2C_SLAVE
	.reg_slave	= nuvoton_i2c_reg_slave,
	.unreg_slave	= nuvoton_i2c_unreg_slave,
#endif /* CONFIG_I2C_SLAVE */
};

static int nuvoton_i2c_probe_bus(struct platform_device *pdev)
{
	struct nuvoton_i2c_bus *bus;
	struct resource *res;
	struct clk *i2c_clk;
	int ret;
	int module__num;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

#ifdef CONFIG_OF
	module__num = of_alias_get_id(pdev->dev.of_node, "i2c");
	bus->module__num = module__num;

	I2C_DEBUG("module__num = %d\n", module__num);

	i2c_clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(i2c_clk)) {
		pr_err(" I2C probe failed: can't read clk.\n");
		return  -EPROBE_DEFER; // this error will cause the probing to run again after clk is ready.
	}
	//clk_prepare_enable(otp_clk);
	bus->apb_clk = clk_get_rate(i2c_clk);
	I2C_DEBUG("I2C APB clock is %d\n", bus->apb_clk);
#endif //  CONFIG_OF

	if (gcr_regmap == NULL) {
		gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
		if (IS_ERR(gcr_regmap)) {
			pr_err("%s: failed to find nuvoton,"
			"npcm750-gcr\n", __func__);
			return IS_ERR(gcr_regmap);
		}
		regmap_write(gcr_regmap, I2CSEGCTL_OFFSET, I2CSEGCTL_VAL);
		printk("I2C%d: gcr mapped\n", bus->module__num);
	}

	if (clk_regmap == NULL) {
		clk_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-clk");
		if (IS_ERR(clk_regmap)) {
			pr_err("%s: failed to find nuvoton,"
			"npcm750-clk\n", __func__);
			return IS_ERR(clk_regmap);
		}
		printk("I2C%d: clk mapped\n", bus->module__num);
	}


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bus->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bus->base))
		return PTR_ERR(bus->base);

	I2C_DEBUG("base = %p\n", bus->base);

	/* Initialize the I2C adapter */
	spin_lock_init(&bus->lock);
	spin_lock_init(&bus->bank_lock);
	init_completion(&bus->cmd_complete);
	bus->adap.owner = THIS_MODULE;
	bus->adap.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	bus->adap.retries = 0;
	bus->adap.timeout = 50 * HZ / 1000;
	bus->adap.algo = &nuvoton_i2c_algo;
	bus->adap.algo_data = bus;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = pdev->dev.of_node;
	snprintf(bus->adap.name, sizeof(bus->adap.name), "Nuvoton i2c");

	bus->dev = &pdev->dev;

	ret = __nuvoton_i2c_init(bus, pdev);
	if (ret < 0)
		return ret;

	bus->irq = platform_get_irq(pdev, 0);
	if (bus->irq < 0) {
		pr_err("I2C platform_get_irq error.");
		return -ENODEV;
	}
	//bus->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	I2C_DEBUG("irq = %d\n", bus->irq);

	ret = request_irq(bus->irq, nuvoton_i2c_bus_irq, 0,
			  dev_name(&pdev->dev), (void *)bus);
	if (ret)
		return ret;

	ret = i2c_add_adapter(&bus->adap);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, bus);

	dev_info(bus->dev, "i2c bus %d registered, irq %d\n",
		 bus->adap.nr, bus->irq);

	return 0;
}

static int nuvoton_i2c_remove_bus(struct platform_device *pdev)
{
	struct nuvoton_i2c_bus *bus = platform_get_drvdata(pdev);
	unsigned long lock_flags;

	spin_lock_irqsave(&bus->lock, lock_flags);

	/* Disable everything. */
	SMB_Disable(bus);

	spin_unlock_irqrestore(&bus->lock, lock_flags);

	i2c_del_adapter(&bus->adap);

	return 0;
}

static const struct of_device_id nuvoton_i2c_bus_of_table[] = {
	{ .compatible = "nuvoton,npcm750-i2c-bus", },
	{ },
};
MODULE_DEVICE_TABLE(of, nuvoton_i2c_bus_of_table);

static struct platform_driver nuvoton_i2c_bus_driver = {
	.probe		= nuvoton_i2c_probe_bus,
	.remove		= nuvoton_i2c_remove_bus,
	.driver		= {
		.name		= "nuvoton-i2c-bus",
		.of_match_table	= nuvoton_i2c_bus_of_table,
	},
};
module_platform_driver(nuvoton_i2c_bus_driver);

MODULE_AUTHOR("Avi Fishman <avi.fishman@gmail.com>");
MODULE_DESCRIPTION("Nuvoton I2C Bus Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(I2C_VERSION);


