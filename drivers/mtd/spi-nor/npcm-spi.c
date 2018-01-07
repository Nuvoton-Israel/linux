/*
 * Copyright (c) 2014-2017 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/vmalloc.h>

#include <asm/sizes.h>
#include <mtd/mtd-abi.h>

typedef enum {
	FIU_MODULE_0 = 0,
	FIU_MODULE_3 = 1,
	FIU_MODULE_X = 2,
	FIU_MAX_MODULE_NUM = 3
} FIU_MODULE_T;

typedef struct bit_field {
	u8 offset;
	u8 size;
} bit_field_t;

/*
 *  Flash Interface Unit (FIU) Registers
 */

/*
 *   FIU Direct Read Configuration Register
 */
#define FIU_DRD_CFG(n)		(fiu_base[n] + 0x00)
static const bit_field_t FIU_DRD_CFG_R_BURST =   { 24, 2 };
static const bit_field_t FIU_DRD_CFG_ADDSIZ =   { 16, 2 };
static const bit_field_t FIU_DRD_CFG_DBW =   { 12, 2 };
static const bit_field_t FIU_DRD_CFG_ACCTYPE =   { 8, 2 };
static const bit_field_t FIU_DRD_CFG_RDCMD =   { 0, 8 };

/*
 * FIU Direct Write Configuration Register
 */
#define FIU_DWR_CFG(n)		(fiu_base[n] + 0x04)
static const bit_field_t FIU_DWR_CFG_LCK =   { 31, 1 };
static const bit_field_t FIU_DWR_CFG_W_BURST =   { 24, 2 };
static const bit_field_t FIU_DWR_CFG_ADDSIZ =   { 16, 2 };
static const bit_field_t FIU_DWR_CFG_ABPCK =   { 10, 2 };
static const bit_field_t FIU_DWR_CFG_DBPCK =   { 8, 2 };
static const bit_field_t FIU_DWR_CFG_WRCMD =   { 0, 8 };

/*
 *   FIU UMA Configuration Register
 */
#define FIU_UMA_CFG(n)		(fiu_base[n] + 0x08)
static const bit_field_t  FIU_UMA_CFG_LCK          =     { 31, 1 };
static const bit_field_t  FIU_UMA_CFG_CMMLCK       =     { 30, 1 };
static const bit_field_t  FIU_UMA_CFG_RDATSIZ      =     { 24, 5 };
static const bit_field_t  FIU_UMA_CFG_DBSIZ        =     { 21, 3 };
static const bit_field_t  FIU_UMA_CFG_WDATSIZ      =     { 16, 5 };
static const bit_field_t  FIU_UMA_CFG_ADDSIZ       =     { 11, 3 };
static const bit_field_t  FIU_UMA_CFG_CMDSIZ       =     { 10, 1 };
static const bit_field_t  FIU_UMA_CFG_RDBPCK       =     { 8, 2 };
static const bit_field_t  FIU_UMA_CFG_DBPCK        =     { 6, 2 };
static const bit_field_t  FIU_UMA_CFG_WDBPCK       =     { 4, 2 };
static const bit_field_t  FIU_UMA_CFG_ADBPCK       =     { 2, 2 };
static const bit_field_t  FIU_UMA_CFG_CMBPCK       =     { 0, 2 };

/*
 * FIU UMA Control and Status Register
 */
#define  FIU_UMA_CTS(n)		(fiu_base[n] + 0x0C)
static const bit_field_t  FIU_UMA_CTS_RDYIE           =  { 25, 1};
static const bit_field_t  FIU_UMA_CTS_RDYST           =  { 24, 1};
static const bit_field_t  FIU_UMA_CTS_SW_CS           =  { 16, 1};
static const bit_field_t  FIU_UMA_CTS_DEV_NUM         =  { 8, 2 };
static const bit_field_t  FIU_UMA_CTS_EXEC_DONE       =  { 0, 1 };

/*
 * FIU UMA Command Register
 */
#define FIU_UMA_CMD(n)		(fiu_base[n] + 0x10)
static const bit_field_t  FIU_UMA_CMD_DUM3           =   { 24, 8};
static const bit_field_t  FIU_UMA_CMD_DUM2           =   { 16, 8};
static const bit_field_t  FIU_UMA_CMD_DUM1           =   { 8, 8 };
static const bit_field_t  FIU_UMA_CMD_CMD            =   { 0, 8 };

/*
 *   FIU UMA Address Register
 */
#define FIU_UMA_ADDR(n)		(fiu_base[n] + 0x14)
static const bit_field_t  FIU_UMA_ADDR_UMA_ADDR          = { 0, 32};
static const bit_field_t  FIU_UMA_ADDR_AB3               = { 24, 8};
static const bit_field_t  FIU_UMA_ADDR_AB2               = { 16, 8};
static const bit_field_t  FIU_UMA_ADDR_AB1               = { 8, 8 };
static const bit_field_t  FIU_UMA_ADDR_AB0               = { 0, 8 };

/*
 * FIU UMA Write Data Bytes 0-3 Register
 */
#define FIU_UMA_DW0(n)		(fiu_base[n] + 0x20)
static const bit_field_t  FIU_UMA_DW0_WB3               = { 24, 8};
static const bit_field_t  FIU_UMA_DW0_WB2               = { 16, 8};
static const bit_field_t  FIU_UMA_DW0_WB1               = { 8, 8 };
static const bit_field_t  FIU_UMA_DW0_WB0               = { 0, 8 };

/*
 * FIU UMA Write Data Bytes 4-7 Register
 */
#define FIU_UMA_DW1(n)		(fiu_base[n] + 0x24)
static const bit_field_t  FIU_UMA_DW1_WB7              = { 24, 8};
static const bit_field_t  FIU_UMA_DW1_WB6              = { 16, 8};
static const bit_field_t  FIU_UMA_DW1_WB5              = { 8, 8 };
static const bit_field_t  FIU_UMA_DW1_WB4              = { 0, 8 };

/*
 * FIU UMA Write Data Bytes 8-11 Register
 */
#define FIU_UMA_DW2(n)		(fiu_base[n] + 0x28)
static const bit_field_t  FIU_UMA_DW2_WB11              = { 24, 8};
static const bit_field_t  FIU_UMA_DW2_WB10              = { 16, 8};
static const bit_field_t  FIU_UMA_DW2_WB9               = { 8, 8 };
static const bit_field_t  FIU_UMA_DW2_WB8               = { 0, 8 };

/*
 * FIU UMA Write Data Bytes 12-15 Register
 */
#define FIU_UMA_DW3(n)		(fiu_base[n] + 0x2C)
static const bit_field_t  FIU_UMA_DW3_WB15              = { 24, 8};
static const bit_field_t  FIU_UMA_DW3_WB14              = { 16, 8};
static const bit_field_t  FIU_UMA_DW3_WB13              = { 8, 8 };
static const bit_field_t  FIU_UMA_DW3_WB12              = { 0, 8 };

/*
 * FIU UMA Read Data Bytes 0-3 Register
 */
#define FIU_UMA_DR0(n)		(fiu_base[n] + 0x30)
static const bit_field_t  FIU_UMA_DR0_RB3               = { 24, 8};
static const bit_field_t  FIU_UMA_DR0_RB2               = { 16, 8};
static const bit_field_t  FIU_UMA_DR0_RB1               = { 8, 8 };
static const bit_field_t  FIU_UMA_DR0_RB0               = { 0, 8 };

/*
 * FIU UMA Read Data Bytes 4-7 Register
 */
#define FIU_UMA_DR1(n)		(fiu_base[n] + 0x34)
static const bit_field_t  FIU_UMA_DR1_RB15              = { 24, 8};
static const bit_field_t  FIU_UMA_DR1_RB14              = { 16, 8};
static const bit_field_t  FIU_UMA_DR1_RB13              = { 8, 8 };
static const bit_field_t  FIU_UMA_DR1_RB12              = { 0, 8 };

/*
 * FIU UMA Read Data Bytes 8-11 Register
 */
#define  FIU_UMA_DR2(n)		(fiu_base[n] + 0x38)
static const bit_field_t  FIU_UMA_DR2_RB15              = { 24, 8};
static const bit_field_t  FIU_UMA_DR2_RB14              = { 16, 8};
static const bit_field_t  FIU_UMA_DR2_RB13              = { 8, 8 };
static const bit_field_t  FIU_UMA_DR2_RB12              = { 0, 8 };

/*
 * FIU UMA Read Data Bytes 12-15 Register
 */
#define FIU_UMA_DR3(n)		(fiu_base[n] + 0x3C)
static const bit_field_t  FIU_UMA_DR3_RB15             =  { 24, 8};
static const bit_field_t  FIU_UMA_DR3_RB14             =  { 16, 8};
static const bit_field_t  FIU_UMA_DR3_RB13             =  { 8, 8 };
static const bit_field_t  FIU_UMA_DR3_RB12             =  { 0, 8 };

/*
 * FIU Protection Configuration Register
 */
#define FIU_PRT_CFG(n)		(fiu_base[n] + 0x18)
static const bit_field_t  FIU_PRT_CFG_LCK              =  { 31, 1};
static const bit_field_t  FIU_PRT_CFG_PEN              =  { 30, 1};
static const bit_field_t  FIU_PRT_CFG_DEVSIZ           =  { 8, 3 };
static const bit_field_t  FIU_PRT_CFG_OCALWD           =  { 4, 1 };
static const bit_field_t  FIU_PRT_CFG_PRTASIZ          =  { 0, 2 };

/*
 * FIU Protection Command Register
 */
#define FIU_PRT_CMD0(n)		(fiu_base[n] + 0x40)
#define FIU_PRT_CMD1(n)		(fiu_base[n] + 0x44)
#define FIU_PRT_CMD2(n)		(fiu_base[n] + 0x48)
#define FIU_PRT_CMD3(n)		(fiu_base[n] + 0x4C)
#define FIU_PRT_CMD4(n)		(fiu_base[n] + 0x50)
#define FIU_PRT_CMD5(n)		(fiu_base[n] + 0x54)
#define FIU_PRT_CMD6(n)		(fiu_base[n] + 0x58)
#define FIU_PRT_CMD7(n)		(fiu_base[n] + 0x5C)
#define FIU_PRT_CMD8(n)		(fiu_base[n] + 0x60)
#define FIU_PRT_CMD9(n)		(fiu_base[n] + 0x64)
static const bit_field_t  FIU_PRT_CMD9_ADBPCKB         =  { 29, 2};
static const bit_field_t  FIU_PRT_CMD9_CMBPCKB         =  { 27, 2};
static const bit_field_t  FIU_PRT_CMD9_ADDSZB          =  { 26, 1};
static const bit_field_t  FIU_PRT_CMD9_FRBDCB          =  { 24, 2};
static const bit_field_t  FIU_PRT_CMD9_CMDB            =  { 16, 8};
static const bit_field_t  FIU_PRT_CMD9_ADBPCKA         =  { 13, 2};
static const bit_field_t  FIU_PRT_CMD9_CMBPCKA         =  { 11, 2};
static const bit_field_t  FIU_PRT_CMD9_ADDSZA          =  { 10, 1};
static const bit_field_t  FIU_PRT_CMD9_FRBDCA          =  { 8, 2 };
static const bit_field_t  FIU_PRT_CMD9_CMDA            =  { 0, 8 };

/*
 * FIU Status Polling Configuration Register
 */
#define FIU_STPL_CFG(n)		(fiu_base[n] + 0x1C)
static const bit_field_t  FIU_STPL_CFG_LCK             =  { 31, 1 };
static const bit_field_t  FIU_STPL_CFG_BUST            =  { 30, 1 };
static const bit_field_t  FIU_STPL_CFG_RDYIE           =  { 29, 1 };
static const bit_field_t  FIU_STPL_CFG_RDY             =  { 28, 1 };
static const bit_field_t  FIU_STPL_CFG_SPDWR           =  { 27, 1 };
static const bit_field_t  FIU_STPL_CFG_POLLPER         =  { 16, 11};
static const bit_field_t  FIU_STPL_CFG_ENPOL           =  { 12, 1 };
static const bit_field_t  FIU_STPL_CFG_BUSYPOL         =  { 11, 1 };
static const bit_field_t  FIU_STPL_CFG_BUSYBS          =  { 8, 3  };
static const bit_field_t  FIU_STPL_CFG_CMD             =  { 0, 8  };

/*
 * FIU Configuration Register
 */
#define FIU_CFG(n)		(fiu_base[n] + 0x78)
static const bit_field_t  FIU_CFG_SPI_CS_INACT         =  { 1, 3};
static const bit_field_t  FIU_CFG_INCRSING             =  { 0, 1};

/*
 * FIU Version Register (FIU_VER) updated
 */
#define FIU_VER(n)		(fiu_base[n] + 0x7C)
static const bit_field_t  FIU_VER_FIU_VER               =  { 0, 8};

/*
 * Defines
 */
#define FIU_CAPABILITY_QUAD_READ
#define FIU_CAPABILITY_CHIP_SELECT

#define WIN_LIMIT_4K_SHIFT  12
#define BITS_7_0            0xFF
#define BITS_15_8           0xFF00
#define BITS_23_16          0xFF0000


/*
 * Typedef Definitions
 */
typedef enum _spi_w_burst_t {
	FIU_W_BURST_ONE_BYTE        = 0,
	FIU_W_BURST_FOUR_BYTE       = 2,
	FIU_W_BURST_SIXTEEN_BYTE    = 3
} SPI_w_burst_t;

typedef enum _spi_r_burst_t {
	FIU_R_BURST_ONE_BYTE        = 0,
	FIU_R_BURST_FOUR_BYTE       = 2,
	FIU_R_BURST_SIXTEEN_BYTE    = 3
} SPI_r_burst_t;

typedef enum _spi_w_protect_int_t {
	SPI_W_PROTECT_INT_DISABLE = 0,
	SPI_W_PROTECT_INT_ENABLE  = 1
} SPI_w_protect_int_t;

typedef enum _spi_incorect_access_int_t {
	SPI_INCORECT_ACCESS_INT_DISABLE   = 0,
	SPI_INCORECT_ACCESS_INT_ENABLE    = 1
} SPI_incorect_access_int_t;

/*
 * FIU Read Mode
 */
typedef enum _spi_read_mode_t {
	FIU_NORMAL_READ             = 0,
	FIU_FAST_READ               = 1,
	FIU_FAST_READ_DUAL_OUTPUT   = 2,
	FIU_FAST_READ_DUAL_IO       = 3,
	FIU_FAST_READ_QUAD_IO       = 4,
	FIU_FAST_READ_SPI_X         = 5,
	FIU_READ_MODE_NUM
} SPI_read_mode_t;

/*
 * FIU UMA data size
 */
typedef enum _spi_uma_data_size_t {
	FIU_UMA_DATA_SIZE_0         = 0,
	FIU_UMA_DATA_SIZE_1         = 1,
	FIU_UMA_DATA_SIZE_2         = 2,
	FIU_UMA_DATA_SIZE_3         = 3,
	FIU_UMA_DATA_SIZE_4         = 4,
	FIU_UMA_DATA_SIZE_5         = 5,
	FIU_UMA_DATA_SIZE_6         = 6,
	FIU_UMA_DATA_SIZE_7         = 7,
	FIU_UMA_DATA_SIZE_8         = 8,
	FIU_UMA_DATA_SIZE_9         = 9,
	FIU_UMA_DATA_SIZE_10         = 10,
	FIU_UMA_DATA_SIZE_11         = 11,
	FIU_UMA_DATA_SIZE_12         = 12,
	FIU_UMA_DATA_SIZE_13         = 13,
	FIU_UMA_DATA_SIZE_14         = 14,
	FIU_UMA_DATA_SIZE_15         = 15,
	FIU_UMA_DATA_SIZE_16         = 16
} SPI_uma_data_size_t;

/*
 * FIU Field value enumeration
 */
typedef enum _spi_drd_cfg_addsiz_t {
	FIU_DRD_CFG_ADDSIZE_24BIT = 0,
	FIU_DRD_CFG_ADDSIZE_32BIT = 1
}  SPI_drd_cfg_addsiz_t;

typedef enum _spi_trans_status_t {
	FIU_TRANS_STATUS_DONE        = 0,
	FIU_TRANS_STATUS_IN_PROG     = 1
} SPI_trans_status_t;

/*
 * SPI commands
 */
#define NPCM_SPI_RD_STATUS_3_REG_CMD		0x15
#define NPCM_SPI_EN_RST_CMD			0x66
#define NPCM_SPI_RST_DEVICE_CMD			0x99
#define NPCM_SPI_WR_EXT_ADDR_REG_CMD		0xC5

#ifdef REG_READ
#undef REG_READ
#endif
static inline u32 REG_READ(unsigned int __iomem *mem)
{
	return ioread32(mem);
}

#ifdef REG_WRITE
#undef REG_WRITE
#endif
static inline void REG_WRITE(unsigned int __iomem *mem, u32 val)
{
	iowrite32(val, mem);
}

#ifdef SET_REG_FIELD
#undef SET_REG_FIELD
#endif
/*
 * Set field of a register / variable according to the field offset and size
 */
static inline void SET_REG_FIELD(unsigned int __iomem *mem,
				 bit_field_t bit_field, u32 val)
{
	u32 tmp = ioread32(mem);

	tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset);
	tmp |= val << bit_field.offset;
	iowrite32(tmp, mem);
}

#ifdef SET_VAR_FIELD
#undef SET_VAR_FIELD
#endif
#define SET_VAR_FIELD(var, bit_field, value) {				\
	typeof(var) tmp = var;						\
	tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset);	\
	tmp |= value << bit_field.offset;				\
	var = tmp;							\
}

#ifdef READ_REG_FIELD
#undef READ_REG_FIELD
#endif
/*
 * Get field of a register / variable according to the field offset and size
 */
static inline u8 READ_REG_FIELD(unsigned int __iomem *mem,
				bit_field_t bit_field)
{
	u8 tmp = ioread32(mem);

	tmp = tmp >> bit_field.offset;
	tmp &= (1 << bit_field.size) - 1;
	return tmp;
}

#ifdef READ_VAR_FIELD
#undef READ_VAR_FIELD
#endif
#define READ_VAR_FIELD(var, bit_field) ({	\
	typeof(var) tmp = var;			\
	tmp = tmp >> bit_field.offset;		\
	tmp &= (1 << bit_field.size) - 1;		\
	tmp;					\
})

#ifdef MASK_FIELD
#undef MASK_FIELD
#endif
/*
 * Build a mask of a register / variable field
 */
#define MASK_FIELD(bit_field) (((1 << bit_field.size) - 1) << bit_field.offset)

#ifdef BUILD_FIELD_VAL
#undef BUILD_FIELD_VAL
#endif
/*
 * Expand the value of the given field into its correct position
 */
#define BUILD_FIELD_VAL(bit_field, value)  \
	((((1 << bit_field.size) - 1) & (value)) << bit_field.offset)


#ifdef SET_REG_MASK
#undef SET_REG_MASK
#endif
/*
 * Set field of a register / variable according to the field offset and size
 */
static inline void SET_REG_MASK(unsigned int __iomem *mem, u32 val)
{
	iowrite32(ioread32(mem) | val, mem);
}

#define DRIVER_NAME "npcm7xx_spinor"

#define SIZE_16MB 0x1000000
#define MAX_READY_WAIT_COUNT	1000000

#define WRITE_TRANSFER_16_BYTES

#ifdef WRITE_TRANSFER_16_BYTES
#define CHUNK_SIZE  16
#endif

#ifdef WRITE_TRANSFER_17_BYTES
#define CHUNK_SIZE  17
#endif

struct npcm7xx_spi_bus;

struct npcm7xx_chip {
	u32 fiu_num;
	u32 clkrate;
	u32 chipselect;
	struct spi_nor	nor;
	struct npcm7xx_spi_bus *host;
};

#define NPCM7XX_MAX_CHIP_NUM		4

struct npcm7xx_spi_bus {
	struct device *dev;
	struct mutex lock;

	void __iomem *regbase;
	struct resource *res_mem;
	u32 MaxChipAddMap;
	resource_size_t iosize;
	struct clk *clk;
	u32 fiu_num;
	bool Direct_Read;

	struct npcm7xx_chip *chip[NPCM7XX_MAX_CHIP_NUM];
};

void __iomem *fiu_base[FIU_MAX_MODULE_NUM];

/* #define NPCMX50_MTD_SPINOR_DEBUG */
#ifdef NPCMX50_MTD_SPINOR_DEBUG
	#define DEBUG_FLASH(format, args...)    printk(format, ##args)
	#define DUMP_MSG(format, args...)       dump_msg(format, ## args)
#else
	#define DEBUG_FLASH(format, args...)
	#define DUMP_MSG(format, args...)
#endif


/*
 * MTD static functions:
 */
#ifdef NPCMX50_MTD_SPINOR_DEBUG
static void dump_msg(const char *label, const unsigned char *buf,
		     unsigned int length);
#endif
//static void npcm7xx_spi_flash_unlock_protection(struct spi_nor *nor);

/*
 * FIU Functions
 */
static int npcm7xx_fiu_uma_read(struct spi_nor *nor, u8 transaction_code,
				u32 address, bool is_address_size, u8 *data,
				u32 data_size)
{
	u32 data_reg[4];
	u32 uma_cfg = 0x0;
	int ret = 0;
	u32 address_size = 0;

	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_spi_bus *host = chip->host;

	switch (chip->chipselect) {
	case 0:
	case 1:
	case 2:
	case 3:
		SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num),
			  FIU_UMA_CTS_DEV_NUM, (u32)chip->chipselect);
		break;
	default:
		return -ENODEV;
	}

	SET_REG_FIELD(FIU_UMA_CMD(host->fiu_num), FIU_UMA_CMD_CMD,
		      transaction_code);
	SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_CMDSIZ, 1);

	if (is_address_size)
		address_size = nor->addr_width;

	SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_ADDSIZ, address_size);

	REG_WRITE(FIU_UMA_ADDR(host->fiu_num), address);
	SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_RDATSIZ, data_size);
	SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_WDATSIZ, 0);

	REG_WRITE(FIU_UMA_CFG(host->fiu_num), uma_cfg);
	SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE, 1);

	/*
	 * wait for indication that transaction has terminated
	 */
	while (READ_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE)
	       == FIU_TRANS_STATUS_IN_PROG){
	}

	if (data_size >= FIU_UMA_DATA_SIZE_1)
		data_reg[0] = REG_READ(FIU_UMA_DR0(host->fiu_num));
	if (data_size >= FIU_UMA_DATA_SIZE_5)
		data_reg[1] = REG_READ(FIU_UMA_DR1(host->fiu_num));
	if (data_size >= FIU_UMA_DATA_SIZE_9)
		data_reg[2] = REG_READ(FIU_UMA_DR2(host->fiu_num));
	if (data_size >= FIU_UMA_DATA_SIZE_13)
		data_reg[3] = REG_READ(FIU_UMA_DR3(host->fiu_num));

	memcpy(data, data_reg, data_size);

	return ret;
}


static int npcm7xx_fiu_uma_write(struct spi_nor *nor, u8 transaction_code,
				 u32 address, bool is_address_size, u8 *data,
				 u32 data_size)
{
	u32 data_reg[4] = {0};
	u32 uma_reg   = 0x0;
	int ret = 0;
	u32 address_size = 0;

	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_spi_bus *host = chip->host;

	switch (chip->chipselect) {
	case 0:
	case 1:
	case 2:
	case 3:
		SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num),
			  FIU_UMA_CTS_DEV_NUM, (u32)chip->chipselect);
		break;
	default:
		return -ENODEV;
	}

	SET_REG_FIELD(FIU_UMA_CMD(host->fiu_num),
		      FIU_UMA_CMD_CMD, transaction_code);
	SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_CMDSIZ, 1);

	if (is_address_size)
		address_size = nor->addr_width;

	SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_ADDSIZ, address_size);
	REG_WRITE(FIU_UMA_ADDR(host->fiu_num), address);

	memcpy(data_reg, data, data_size);

	if (data_size >= FIU_UMA_DATA_SIZE_1)
		REG_WRITE(FIU_UMA_DW0(host->fiu_num), data_reg[0]);
	if (data_size >= FIU_UMA_DATA_SIZE_5)
		REG_WRITE(FIU_UMA_DW1(host->fiu_num), data_reg[1]);
	if (data_size >= FIU_UMA_DATA_SIZE_9)
		REG_WRITE(FIU_UMA_DW2(host->fiu_num), data_reg[2]);
	if (data_size >= FIU_UMA_DATA_SIZE_13)
		REG_WRITE(FIU_UMA_DW3(host->fiu_num), data_reg[3]);

	SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_WDATSIZ, data_size);
	SET_VAR_FIELD(uma_reg, FIU_UMA_CFG_RDATSIZ, 0);

	REG_WRITE(FIU_UMA_CFG(host->fiu_num), uma_reg);
	SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE, 1);

	/*
	 * wait for indication that transaction has terminated
	 */
	while (READ_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_EXEC_DONE)
	       == FIU_TRANS_STATUS_IN_PROG) {
	}

	return ret;
}


static int npcm7xx_fiu_manualwrite(struct spi_nor *nor, u8 transaction_code,
				   u32 address, u8 *data, u32 data_size)
{
	u8   uma_cfg  = 0x0;
	u32  num_data_chunks;
	u32  remain_data;
	u32  idx = 0;

	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_spi_bus *host = chip->host;

	SET_VAR_FIELD(uma_cfg, FIU_UMA_CFG_WDATSIZ, 16);

	num_data_chunks  = data_size / CHUNK_SIZE;
	remain_data  = data_size % CHUNK_SIZE;

	SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_DEV_NUM,
		      (u32)chip->chipselect);
	SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_SW_CS, 0);

	npcm7xx_fiu_uma_write(nor, transaction_code, address, true, NULL, 0);
/*
 * Starting the data writing loop in multiples of 8
 */
	for (idx = 0; idx < num_data_chunks; ++idx) {
		npcm7xx_fiu_uma_write(nor, data[0], (u32)NULL, false, &data[1],
				      CHUNK_SIZE-1);
		data += CHUNK_SIZE;
	}

/*
 * Handling chunk remains
 */
	if (remain_data > 0)
		npcm7xx_fiu_uma_write(nor, data[0], (u32)NULL, false, &data[1],
				      remain_data-1);

	SET_REG_FIELD(FIU_UMA_CTS(host->fiu_num), FIU_UMA_CTS_SW_CS, 1);

	return 0;
}

/*
 * SPI Functions
 */
static void npcm7xx_spi_flash_high_addr_wr(struct spi_nor *nor, u8 HighAddr)
{
	npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
	npcm7xx_fiu_uma_write(nor, NPCM_SPI_WR_EXT_ADDR_REG_CMD, 0, false,
			      &HighAddr, sizeof(u8));
}

static void npcm7xx_spi_flash_common_getstatus(struct spi_nor *nor, u8 *status)
{
	npcm7xx_fiu_uma_read(nor, SPINOR_OP_RDSR, 0, 0, status, 1);
}

static void npcm7xx_spi_flash_common_waittillready(struct spi_nor *nor)
{
	u8 busy = 1;

	do {
		npcm7xx_spi_flash_common_getstatus(nor, &busy);
		/* Keep only "busy" bit 0 */
		busy &= 0x01;
	} while (busy);
}

static void npcm7xx_spi_flash_common_write(struct spi_nor *nor, u32 destAddr,
				     u8 *data, u32 size)
{
	if ((destAddr >> 24) && (nor->addr_width == 3)) {
		npcm7xx_spi_flash_high_addr_wr(nor, destAddr >> 24);
		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
		npcm7xx_fiu_manualwrite(nor, SPINOR_OP_PP,
					(destAddr & 0xFFFFFF), data, size);
		npcm7xx_spi_flash_common_waittillready(nor);
		npcm7xx_spi_flash_high_addr_wr(nor, 0);
	} else {
		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
		npcm7xx_fiu_manualwrite(nor, SPINOR_OP_PP, destAddr, data,
					size);
		npcm7xx_spi_flash_common_waittillready(nor);
	}
}

static void npcm7xx_spi_flash_unlock_protection(struct spi_nor *nor)
{
	u8 status_reg_val = 0;

	npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL, 0);
	npcm7xx_fiu_uma_write(nor, SPINOR_OP_WRSR, 0, false,
			      &status_reg_val, sizeof(u8));
	npcm7xx_spi_flash_common_waittillready(nor);
}

static ssize_t npcm7xx_spi_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *write_buf)
{
	u32 local_addr = (u32) to;
	u32 cnt = (u32) len;
	u32 actual_size     = 0;
	struct mtd_info *mtd;

	mtd = &nor->mtd;

	DEBUG_FLASH("mtd_spinor: %s %s 0x%08x, len %zd\n", __func__,
		    "to", (u32)to, len);

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > mtd->size)
		return -EINVAL;

	if (cnt != 0) {
		while (cnt) {
			actual_size = ((((local_addr)/mtd->writesize) + 1)
				       *mtd->writesize) - (local_addr);
			if (actual_size > cnt)
				actual_size = cnt;

			npcm7xx_spi_flash_common_write(nor, local_addr,
						       (u_char *)write_buf,
						       actual_size);

			write_buf += actual_size;
			local_addr += actual_size;
			cnt -= actual_size;
		}
	}

	return (len - cnt);
}

static ssize_t npcm7xx_spi_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *read_buf)
{
	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_spi_bus *host = chip->host;
	void __iomem *flash_region_mapped_ptr = NULL;
	struct mtd_info *mtd;
	int i, readlen, currlen;
	u32 addr;
	u8 *buf_ptr;
	u32 retlen = 0;

	mtd = &nor->mtd;

	DEBUG_FLASH("mtd_spinor: %s %s 0x%08x, len %zd\n", __func__, "from",
		(u32)from, len);

	if (!len)
		return 0;

	if (from + len > (u32)mtd->size)
		return -EINVAL;

	DEBUG_FLASH("mtd_spinor: %s , mtd->size 0x%08x %p\n", __func__,
		    (u32) mtd->size, read_buf);


	if (host->Direct_Read) {
		if (!request_mem_region((host->res_mem->start +
					 (host->MaxChipAddMap *
					 chip->chipselect)) + from,
					 len, mtd->name))
			return -EBUSY;

		flash_region_mapped_ptr = (u32 *)ioremap_nocache(
		    (host->res_mem->start +
		     (host->MaxChipAddMap * chip->chipselect)) + from, len);

		if (!flash_region_mapped_ptr) {
			pr_err(": Failed to ioremap window!\n");
			release_mem_region((host->res_mem->start +
					    (host->MaxChipAddMap *
					     chip->chipselect)) + from, len);
			return -ENOMEM;
		}

		if (mtd->size > SIZE_16MB) {
			if (nor->addr_width == 3)
				SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),
					      FIU_DRD_CFG_RDCMD,
					      SPINOR_OP_READ_1_2_2_4B);

			else
				SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),
					      FIU_DRD_CFG_RDCMD,
					      SPINOR_OP_WRSR);
			SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),
				      FIU_DRD_CFG_ADDSIZ, 0x1);
		} else {
			SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),
				      FIU_DRD_CFG_RDCMD, SPINOR_OP_READ_1_2_2);
			SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num),
				      FIU_DRD_CFG_ADDSIZ, 0x0);
		}

		memcpy(read_buf, flash_region_mapped_ptr, len);
		wmb();

		npcm7xx_spi_flash_high_addr_wr(nor, 0);

		if (flash_region_mapped_ptr)
			iounmap((void __iomem *)flash_region_mapped_ptr);

		release_mem_region((host->res_mem->start +
				    (host->MaxChipAddMap * chip->chipselect))
				    + from, len);

		retlen = len;
	} else {
		/* NOTE:  OPCODE_FAST_READ (if available) is faster... */
		i = 0;
		currlen = (int) len;

		do {
			addr = ((u32) from + i);
			if (currlen < 4)
				readlen = currlen;
			else
				readlen = 4;

			DEBUG_FLASH("mtd_spinor: ori_addr=0x%x, addr=0x%x\n",
				    ((u32) from + i), addr);

			buf_ptr = read_buf + i;

			if ((addr >> 24) && (nor->addr_width == 3))
				npcm7xx_spi_flash_high_addr_wr(nor, addr >> 24);

			npcm7xx_fiu_uma_read(nor, SPINOR_OP_READ,
					     (addr&0xFFFFFF), true, buf_ptr,
					     readlen);

			if ((addr >> 24) && (nor->addr_width == 3))
				npcm7xx_spi_flash_high_addr_wr(nor, 0);

			DEBUG_FLASH("mtd_spinor: buf_ptr=0x%x buf_val=0x%x "
				    "i=%d readlen =%d\n", (u32)buf_ptr,
				    *((u32 *)buf_ptr), i, readlen);

			i += readlen;
			currlen -= 4;
		} while (currlen > 0);

		retlen = i;
	}

	DUMP_MSG("MTD_READ", read_buf, i);
	return retlen;
}

static int npcm7xx_spi_erase(struct spi_nor *nor, loff_t offs)
{
	u32 addr = (u32)offs;
	struct mtd_info *mtd;
	mtd = &nor->mtd;

	if ((addr >> 24) && (nor->addr_width == 3)) {
		npcm7xx_spi_flash_high_addr_wr(nor, addr >> 24);

		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL,
				      0);
		if (mtd->erasesize == 4096)
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_BE_4K,
					      (addr & 0xFFFFFF), true, NULL, 0);
		else
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_SE,
					      (addr & 0xFFFFFF), true, NULL, 0);
		npcm7xx_spi_flash_common_waittillready(nor);
		npcm7xx_spi_flash_high_addr_wr(nor, 0);
	} else {
		npcm7xx_fiu_uma_write(nor, SPINOR_OP_WREN, 0, false, NULL,
				      0);
		if (mtd->erasesize == 4096)
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_BE_4K, addr,
					      true, NULL, 0);
		else
			npcm7xx_fiu_uma_write(nor, SPINOR_OP_SE, addr,
					      true, NULL, 0);
		npcm7xx_spi_flash_common_waittillready(nor);
	}

	return 0;
}

static int npcm7xx_spi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	return npcm7xx_fiu_uma_read(nor, opcode, 0, 0, buf, len);
}

static int npcm7xx_spi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				 int len)
{
	return npcm7xx_fiu_uma_write(nor, opcode, (u32)NULL, false, buf, len);
}

static int npcm7xx_spi_nor_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_spi_bus *host = chip->host;

	mutex_lock(&host->lock);

	return 0;
}

static void npcm7xx_spi_nor_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct npcm7xx_chip *chip = nor->priv;
	struct npcm7xx_spi_bus *host = chip->host;

	mutex_unlock(&host->lock);
}

#ifdef NPCMX50_MTD_SPINOR_DEBUG
static void dump_msg(const char *label, const unsigned char *buf,
		     unsigned int length)
{
	unsigned int	start, num, i;
	char		line[52], *p;

	if (length > 32)
		length = 32;

	DEBUG_FLASH("MTD_SPI: %s, length %u:\n", label, length);

	start = 0;
	while (length > 0) {
		num = min(length, 16u);
		p = line;
		for (i = 0; i < num; ++i) {
			if (i == 8)
				*p++ = ' ';
			sprintf(p, " %02x", buf[i]);
			p += 3;
		}
		*p = 0;
		pr_info("%6x: %s\n", start, line);
		buf += num;
		start += num;
		length -= num;
	}
}
#endif

/*
 * Get spi flash device information and register it as a mtd device.
 */
static int npcm7xx_spi_nor_register(struct device_node *np,
				struct npcm7xx_spi_bus *host)
{
	struct device *dev = host->dev;
	struct spi_nor *nor;
	struct npcm7xx_chip *chip;
	struct mtd_info *mtd;
	u32 chipselect;
	int ret;
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_READ_1_1_2 |
			SNOR_HWCAPS_READ_1_2_2 |
			SNOR_HWCAPS_READ_2_2_2 |
			SNOR_HWCAPS_READ_1_1_4 |
			SNOR_HWCAPS_READ_1_4_4 |
			SNOR_HWCAPS_READ_4_4_4 |
			SNOR_HWCAPS_PP |
			SNOR_HWCAPS_PP_1_1_4 |
			SNOR_HWCAPS_PP_1_4_4 |
			SNOR_HWCAPS_PP_4_4_4,
	};

	/* This driver does not support NAND or NOR flash devices. */
	if (!of_device_is_compatible(np, "jedec,spi-nor")) {
		dev_err(dev, "The device is no compatible to jedec,spi-nor\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(np, "reg", &chipselect);
	if (ret) {
		dev_err(dev, "There's no reg property for %s\n", np->full_name);
		return ret;
	}

	if (chipselect >= NPCM7XX_MAX_CHIP_NUM) {
		dev_warn(dev, "Flash device number exceeds the maximum "
			 "chipselect number\n");
		return -ENOMEM;
	}

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->host = host;
	chip->chipselect = chipselect;

	nor = &chip->nor;
	mtd = &nor->mtd;

	nor->dev = dev;
	nor->priv = chip;

	spi_nor_set_flash_node(nor, np);

/*	ret = of_property_read_u32(np, "spi-max-frequency",
 *		&chip->clkrate);
 *	if (ret) {
 *	    dev_err(dev, "There's no spi-max-frequency property for %s\n",
 *		np->full_name);
 *	    return ret;
 *	}
 */
	nor->prepare = npcm7xx_spi_nor_prep;
	nor->unprepare = npcm7xx_spi_nor_unprep;

	nor->read_reg = npcm7xx_spi_read_reg;
	nor->write_reg = npcm7xx_spi_write_reg;
	nor->read = npcm7xx_spi_read;
	nor->write = npcm7xx_spi_write;
	nor->erase = npcm7xx_spi_erase;

	ret = spi_nor_scan(nor, NULL, &hwcaps);
	if (ret)
		return ret;

	if (mtd->size > SIZE_16MB) {
		/* If Flash size is over 16MB the spi_nor_scan sets
		   automatically the FLASH to work with 4 byte addressing.
		   Our driver handle Flash size over 16MB with 3 byte address.
		   Revert back to 3 byte address size cause issues so the
		   sequence below resets WINBOND and MACRONIX FLASH to work
		   again with 3 byte address (From Kernel 4.14 and above
		   the address width statically configured by the driver)*/
		   npcm7xx_spi_flash_common_waittillready(nor);
		   npcm7xx_fiu_uma_write(nor, NPCM_SPI_EN_RST_CMD, 0,
		   		      false, NULL, 0);
		   npcm7xx_fiu_uma_write(nor, NPCM_SPI_RST_DEVICE_CMD, 0,
		   		      false, NULL, 0);
		   npcm7xx_spi_flash_common_waittillready(nor);
		   nor->addr_width = 3;
	}

	npcm7xx_spi_flash_unlock_protection(nor);

	//mtd->name = np->name;
	ret = mtd_device_register(mtd, NULL, 0);
	if (ret)
		return ret;

	host->chip[chip->chipselect] = chip;
	return 0;
}

static void npcm7xx_spi_nor_unregister_all(struct npcm7xx_spi_bus *host)
{
	struct npcm7xx_chip *chip;
	int n;

	for (n = 0; n < NPCM7XX_MAX_CHIP_NUM; n++) {
		chip = host->chip[n];
		if (chip)
			mtd_device_unregister(&chip->nor.mtd);
	}
}

static int npcm7xx_spi_nor_register_all(struct npcm7xx_spi_bus *host)
{
	struct device *dev = host->dev;
	struct device_node *np;
	int ret;

	for_each_available_child_of_node(dev->of_node, np) {
		ret = npcm7xx_spi_nor_register(np, host);
		if (ret)
			goto fail;
	}

	return 0;

fail:
	npcm7xx_spi_nor_unregister_all(host);
	return ret;
}

static int npcm7xx_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct npcm7xx_spi_bus *host;
	static u32 fiu_num;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	platform_set_drvdata(pdev, host);
	host->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "control");
	host->regbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(host->regbase))
		return PTR_ERR(host->regbase);

	host->res_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "memory");
	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk))
		return PTR_ERR(host->clk);

	if (of_device_is_compatible(np, "nuvoton,npcm750-spi"))
		host->MaxChipAddMap = 0x8000000;
	else {
		ret = of_property_read_u32(dev->of_node, "chip-max-address-map",
					   &host->MaxChipAddMap);
		if (ret) {
			pr_info("There's no chip-max-address-map property "
				     "for %s adjust 128Mb\n",
				dev->of_node->full_name);
			host->MaxChipAddMap = 0x8000000;
		}
	}

	mutex_init(&host->lock);
	//clk_prepare_enable(host->clk);
	fiu_base[fiu_num] = host->regbase;
	host->fiu_num = fiu_num;

	if (host->res_mem) {
		/* set access to Dual I/O */
		SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num), FIU_DRD_CFG_ACCTYPE,
			      1);
		/* set dummy byte to 1 */
		SET_REG_FIELD(FIU_DRD_CFG(host->fiu_num), FIU_DRD_CFG_DBW, 1);
		host->Direct_Read = true;
	} else
		host->Direct_Read = false;

	ret = npcm7xx_spi_nor_register_all(host);
	if (ret) {
		mutex_destroy(&host->lock);
		//clk_disable_unprepare(host->clk);
		pr_info("npcm7xx_spi_nor_register_all failed\n");
	}

	fiu_num++;

	DEBUG_FLASH("mtd_spinor: %s() date=%s time=%s\n\n", __func,  __date__,
		     __time__);

	return ret;
}

static int npcm7xx_spi_remove(struct platform_device *pdev)
{
	struct npcm7xx_spi_bus *host = platform_get_drvdata(pdev);

	npcm7xx_spi_nor_unregister_all(host);
	mutex_destroy(&host->lock);
	clk_disable_unprepare(host->clk);
	return 0;
}

static const struct of_device_id npcm7xx_spi_dt_ids[] = {
	{ .compatible = "nuvoton,npcm750-spi" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, npcm7xx_spi_dt_ids);

static struct platform_driver npcm7xx_spi_driver = {
	.driver = {
		.name	= "npcm7xx-spi",
		.bus	= &platform_bus_type,
		.of_match_table = npcm7xx_spi_dt_ids,
	},
	.probe      = npcm7xx_spi_probe,
	.remove	    = npcm7xx_spi_remove,
};
module_platform_driver(npcm7xx_spi_driver);

MODULE_DESCRIPTION("Nuvoton SPI Controller Driver");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_LICENSE("GPL v2");
