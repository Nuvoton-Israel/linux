/*
 * Cryptographic API.
 *
 * Support for Nuvoton AES HW acceleration.
 *
 * Copyright (c) 2016 Nuvoton Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Based on Padlock AES.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <linux/ioport.h>
#include <linux/spinlock_types.h>
#include <linux/types.h>
#include <linux/percpu.h>
#include <linux/smp.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <asm/byteorder.h>
#include <asm/processor.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/ctr.h>


#include <asm/io.h>

//#include <mach/map.h>
//#include <mach/hal.h>
//#include <defs.h>
#include "npcm750_aes.h"



#ifdef SET_REG_FIELD
#undef SET_REG_FIELD
#endif
/*---------------------------------------------------------------------------*/
/* Set field of a register / variable according to the field offset and size */
/*---------------------------------------------------------------------------*/
static inline void SET_REG_FIELD(void __iomem *mem,bit_field_t bit_field, u32 val)
{
	u32 tmp = ioread32(mem);
	tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset);
	tmp |= val << bit_field.offset;
	iowrite32(tmp, mem);
}

#ifdef SET_VAR_FIELD
#undef SET_VAR_FIELD
#endif
// bit_field should be of bit_field_t type
#define SET_VAR_FIELD(var, bit_field, value) {                 \
    typeof(var) tmp = var;                 		       \
    tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset); \
    tmp |= value << bit_field.offset; 			       \
    var = tmp;                                                 \
}

#ifdef READ_REG_FIELD
#undef READ_REG_FIELD
#endif
/*---------------------------------------------------------------------------*/
/* Get field of a register / variable according to the field offset and size */
/*---------------------------------------------------------------------------*/
static inline u32 READ_REG_FIELD(void __iomem *mem, bit_field_t bit_field)
{
	u32 tmp = ioread32(mem);
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
/*----------------------------------------------*/
/* Build a mask of a register / variable field  */
/*----------------------------------------------*/
// bit_field should be of bit_field_t type
#define MASK_FIELD(bit_field) \
    (((1 << bit_field.size) - 1) << bit_field.offset) /* mask the field size */

#ifdef BUILD_FIELD_VAL
#undef BUILD_FIELD_VAL
#endif
/*-----------------------------------------------------------------*/
/* Expand the value of the given field into its correct position   */
/*-----------------------------------------------------------------*/
// bit_field should be of bit_field_t type
#define BUILD_FIELD_VAL(bit_field, value)  \
    ((((1 << bit_field.size) - 1) & (value)) << bit_field.offset)


#ifdef SET_REG_MASK
#undef SET_REG_MASK
#endif
/*---------------------------------------------------------------------------*/
/* Set field of a register / variable according to the field offset and size */
/*---------------------------------------------------------------------------*/
static inline void SET_REG_MASK(void __iomem *mem, u32 val)
{
	iowrite32(ioread32(mem) | val, mem);
}

/*---------------------------------------------------------------------------------------------------------*/
/* AES module local macro definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define AES_IS_BUSY()               READ_REG_FIELD(AES_BUSY, AES_BUSY_AES_BUSY)
#define AES_SWITCH_TO_DATA_MODE()   SET_REG_FIELD(AES_BUSY, AES_BUSY_AES_BUSY, 1)
#define AES_SWITCH_TO_CONFIG()      SET_REG_FIELD(AES_BUSY, AES_BUSY_AES_BUSY, 0)
#define AES_DOUT_FIFO_IS_EMPTY()    READ_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_EMPTY)
#define AES_DOUT_FIFO_IS_FULL()     READ_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_FULL)
#define AES_DIN_FIFO_IS_FULL()      READ_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_FULL)
#define AES_DIN_FIFO_IS_EMPTY()     READ_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_EMPTY)
#define AES_LOAD_KEY()              SET_REG_FIELD(AES_SK, AES_SK_AES_SK, 1)
#define AES_SOFTWARE_RESET()        SET_REG_FIELD(AES_SW_RESET, AES_SW_RESET_AES_SW_RESET, 1)

#define SECOND_TIMEOUT 0x300000
#define CRYPTO_TIMEOUT(CHECK,TIMEOUT)					\
{									\
    u32 aes_timeout = 0;						\
    while (CHECK && aes_timeout < TIMEOUT)      			\
    {   								\
	aes_timeout++;  						\
    }   								\
    if (aes_timeout == TIMEOUT)       					\
    {   								\
	pr_err("[%s]: Timeout expired",__func__);       		\
	return;     							\
    }   								\
}

/*---------------------------------------------------------------------------------------------------------*/
/* AES module local functions declaration                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
static int  AES_Config(NPCMX50_AES_OP_T op, NPCMX50_AES_MODE_T mode, NPCMX50_AES_KEY_SIZE_T keySize); 
static void AES_LoadCTR         (u32 *ctr);
static void AES_LoadIV          (u32 *iv, int iv_size);

static void AES_LoadKeyExternal (u32 *key, NPCMX50_AES_KEY_SIZE_T size);
static void AES_LoadKeyByIndex  (u8 index, NPCMX50_AES_KEY_SIZE_T size);
static void AES_LoadKey         (u32 *key, NPCMX50_AES_KEY_SIZE_T size, u8 index);
static void AES_WriteBlock      (u32 *dataIn);
static void AES_ReadBlock       (u32 *dataOut);
static void AES_CryptData       (u32 size, u32 *dataIn, u32 *dataOut);

#ifdef CONFIG_CRYPTO_CCM
static void AES_ReadIV          (u32 *iv);
static void AES_ReadPrevIV      (u32 *prevIv);
static void AES_FeedMessage     (u32  size, u32 *dataIn, u32 *dataOut); // cbc-mac only
#endif

static void AES_PrintRegs       (void);


/* AES MODULE registers */
static void __iomem *aes_base;
static struct platform_device *aes_dev;

static DEFINE_MUTEX(npcmx50_aes_lock);


/*---------------------------------------------------------------------------------------------------------*/
/* Dummy key: call AES with this key to indicate user wants to use OTP keys                                */
/*---------------------------------------------------------------------------------------------------------*/
static unsigned char dummy[AES_KEYSIZE_256 - 1] = { 
  /* first number is the key num */ 

  /*key_num*/  0xd0, 0x78, 0x50,   0x7d, 0xc3, 0x3c, 0x9c,   0x66, 0x96, 0x69, 0xbe,   0x8a, 0x8d, 0xda, 0x51,
       0xf3,   0xdc, 0xe8, 0xd0,   0x20, 0x5a, 0xb0, 0xe6,   0x80,	0x2c, 0x3d, 0x9c,  0xfe, 0x68, 0x3a, 0x6d           
};

// missing first byte on dummy array is 0..3 is used to indicate the key number

#define DUMMY_KEY_NUM_0  0x00
#define DUMMY_KEY_NUM_1  0x01
#define DUMMY_KEY_NUM_2  0x02
#define DUMMY_KEY_NUM_3  0x03




// #define AES_DEBUG_MODULE
#ifdef AES_DEBUG_MODULE
#define aes_print(fmt,args...)       printk(fmt ,##args)
#else
#define aes_print(fmt,args...)      (void)0
#endif



static void aes_print_hex_dump(char *note, unsigned char *buf, unsigned int len)
{
#ifndef AES_DEBUG_MODULE
        return;
#else
		aes_print(KERN_CRIT "%s", note);
		print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
				16, 1,
				buf, len, false);
#endif				
}




/*---------------------------------------------------------------------------------------------------------*/
/* OTP IOCTLs  : use this to print currently loaded key. remove when no need for print anymore             */
/*---------------------------------------------------------------------------------------------------------*/
extern long npcm750_otp_ioctl(struct file *filp, unsigned cmd, unsigned long arg);

#define NPCM750_OTP_IOC_MAGIC       '2'             /* used for all IOCTLs to npcm750_otp.  AES Key handling */

#define IOCTL_SELECT_AES_KEY        _IO(NPCM750_OTP_IOC_MAGIC,   0)/* To Configure the Bus topology based on I2CTopology.config file*/
#define IOCTL_DISABLE_KEY_ACCESS    _IO(NPCM750_OTP_IOC_MAGIC,   1)/* To send request header and perform I2C operation */
#define IOCTL_GET_AES_KEY_NUM       _IO(NPCM750_OTP_IOC_MAGIC,   2)/* To Configure the Bus topology based on I2CTopology.config file*/



/*---------------------------------------------------------------------------------------------------------*/
/* AES module functions implementation                                                                     */
/*---------------------------------------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_Config                                                                             */
/*                                                                                                         */
/* Parameters:      operation - AES Operation (Encrypt/Decrypt) [input].                                   */
/*                  mode      - AES Mode of Operation (ECB, CBC, etc) [input].                             */
/*                  keySize   - AES Key Size (128/192/256) [input].                                        */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Configure the AES engine in terms of operation, mode and key size.                                 */
/*---------------------------------------------------------------------------------------------------------*/
static int AES_Config (
    NPCMX50_AES_OP_T       operation,
    NPCMX50_AES_MODE_T     mode,
    NPCMX50_AES_KEY_SIZE_T keySize
)
{
    u32 ctrl = ioread32(AES_CONTROL);
    u32 orgctrlval = ctrl; 

    /* Determine AES Operation - Encrypt / Decrypt */
    SET_VAR_FIELD(ctrl, AES_CONTROL_DEC_ENC, operation);

    /* Determine AES Mode of Operation - ECB / CBC / CTR / MAC */
    SET_VAR_FIELD(ctrl, AES_CONTROL_MODE_KEY0, mode);

    /* Determine AES Key Size - 128 / 192 / 256 */
    if ( keySize != AES_KEY_SIZE_USE_LAST)
    {
        SET_VAR_FIELD(ctrl, AES_CONTROL_NK_KEY0, keySize);
    }

    aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_Config ctrl=0x%x\n", ctrl);
    if (ctrl != orgctrlval) 
    {
        
        iowrite32(ctrl, AES_CONTROL); 
        
        /* Workaround to over come Errata #648 - 
           oWDEN bit in FUSTRAP register affects the AES module: when set, it forces NK_KEY0 field to 0
           during writing to AES_CONTROL register. This means that the key is limited to 128 bits. */
        if (ctrl != ioread32(AES_CONTROL)) 
        {
            
            u16 keyctrl; 
            u32 wrtimeout; 
            u32 read_ctrl; 
            int intwr; 
            
            keyctrl = (u16)((ctrl & 0x301D) | ((ctrl >> 16) & 0x8000)); 
            for (wrtimeout = 0; wrtimeout < 1000; wrtimeout++) 
            {
                /* Write configurable info in a single write operation */
                for (intwr=0;intwr<10;intwr++) 
                {
	                iowrite32(ctrl, AES_CONTROL); 
	                iowrite16(ctrl, AES_CONTROL_WORD_HIGH);
	                wmb(); 
                }
                
                read_ctrl = ioread32(AES_CONTROL); 
                if (ctrl != read_ctrl) 
                {
                    aes_print(KERN_NOTICE  "\nexpected data=0x%x Actual AES_CONTROL data 0x%x wrtimeout %d\n\n", ctrl, read_ctrl, wrtimeout);
                } else 
                    break;
            }
            
            if (wrtimeout == 1000) 
            {
                pr_err("\nTIMEOUT expected data=0x%x Actual AES_CONTROL data 0x%x\n\n", ctrl, read_ctrl);
                return -EAGAIN;
            }
        }
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_LoadCTR                                                                            */
/*                                                                                                         */
/* Parameters:      ctr - Counter for ciphering [input].                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Load the Counter used in CTR mode of operation.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_LoadCTR (u32 *ctr)
{
    u32 i;

    /* Counter is loaded in 32-bit chunks */
    for (i = 0; i < (AES_MAX_CTR_SIZE / sizeof(u32)); i++, ctr++)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_LoadCTR write=0x%x\n", __le32_to_cpu(*ctr));
        *((PTR32)((AES_CTR0) + i*sizeof(u32))) = __le32_to_cpu(*ctr);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_LoadIV                                                                             */
/*                                                                                                         */
/* Parameters:      iv - Initialization Vector for ciphering [input].                                      */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Load the Initialization Vector used in CBC mode of operation.                                      */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_LoadIV (u32 *iv, int iv_size)
{
    u32 i;

    /* Initialization Vector is loaded in 32-bit chunks */
    for (i = 0; i < (iv_size / sizeof(u32)); i++, iv++)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_LoadIV write=0x%x\n", __le32_to_cpu(*iv));
        *((PTR32)((AES_IV_0) + i*sizeof(u32))) = __le32_to_cpu(*iv);
    }
}

#if 0 // used for CONFIG_CRYPTO_CCM
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_ReadIV                                                                             */
/*                                                                                                         */
/* Parameters:      iv - Initialization Vector [output].                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Read the Initialization Vector used in CBC mode of operation.                                      */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_ReadIV (u32 *iv)
{
    u32 i;

    /* Initialization Vector is read in 32-bit chunks */
    for (i = 0; i < (AES_MAX_IV_SIZE / sizeof(u32)); i++, iv++)
    {
        *iv = __cpu_to_le32((MEMR32((AES_IV_0) + i*sizeof(u32)))) ;
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_ReadIV read=0x%x\n", (*iv));        
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_ReadPrevIV                                                                         */
/*                                                                                                         */
/* Parameters:      prevIv - Previous Initialization Vector [output].                                      */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Read the Previous Initialization Vector used in XCBC-MAC mode of                                   */
/*      operation.                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void AES_ReadPrevIV (u32 * prevIv)
{
    u32 i;

    /* Initialization Vector is loaded in 32-bit chunks */
    for (i = 0; i < (AES_MAX_IV_SIZE / sizeof(u32)); i++, prevIv++)
    {
        *prevIv = (MEMR32((AES_PREV_IV_0) +i*sizeof(u32)));
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_ReadPrevIV read=0x%x\n", (*prevIv));
    }
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_LoadKeyExternal                                                                    */
/*                                                                                                         */
/* Parameters:      key  - Raw key data [input].                                                           */
/*                  size - Byte length of the key [input].                                                 */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Load secret key from memory.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_LoadKeyExternal (
    u32        *key,
    NPCMX50_AES_KEY_SIZE_T size
)
{
    u32 i;

    aes_print_hex_dump("\t\t\t*npcmX50-AES: AES_LoadKeyExternal ", (void *)key, AES_KEY_BYTE_SIZE(size));

    AES_SWITCH_TO_CONFIG();

    /* Key is loaded in 32-bit chunks */
    for (i = 0; i < (AES_KEY_BYTE_SIZE(size) / sizeof(u32)); i++, key++)
    {
         *((PTR32)((AES_KEY_0) + 4*i)) = __le32_to_cpu(*key);
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_LoadKeyByIndex                                                                     */
/*                                                                                                         */
/* Parameters:      index - Index to OTP key (in 128-bit steps) [input].                                   */
/*                  size  - Byte length of the key [input].                                                */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Load secret key from OTP.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_LoadKeyByIndex (
    u8          index,
    NPCMX50_AES_KEY_SIZE_T size
)
{
    CRYPTO_TIMEOUT(AES_IS_BUSY(), SECOND_TIMEOUT) 

    aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_LoadKeyByIndex: index %d -> size %d\n", index, size);
        

    /* Upload key from OTP to AES engine through side-band port */
    npcm750_otp_ioctl(NULL, IOCTL_SELECT_AES_KEY, (unsigned long)index);

    /* key is loaded using the aes_cryptokey input port */
    AES_LOAD_KEY();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_LoadKey                                                                            */
/*                                                                                                         */
/* Parameters:      key   - Raw key data [input].                                                          */
/*                  size  - Byte length of the key [input].                                                */
/*                  index - Index to OTP key [input].                                                      */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Load the key.                                                                                      */
/*      When key is not specified, index will be used as the key index to be                               */
/*      retrieved from the OTP (One Time Programming) and loaded to the AES                                */
/*      engine using the aes_cryptokey input port.                                                         */
/*      This way, the key is kept hidden from the application.                                             */
/*      When key is specified, index is ignored.                                                           */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_LoadKey (
    u32        *key,
    NPCMX50_AES_KEY_SIZE_T size,
    u8          index
)
{
    if (key != NULL)
    {
        /* Load secret key from memory */
        AES_LoadKeyExternal(key, size);
        aes_print("\tAES ext key\n");
    }
    else
    {
        /* Load secret key from OTP */
        AES_LoadKeyByIndex(index, size);
        aes_print("\tAES HRK%d key\n", index);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_WriteBlock                                                                         */
/*                                                                                                         */
/* Parameters:      dataIn - Input data for ciphering [input].                                             */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Write data block dataIn to the FIFO buffer                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_WriteBlock (u32 *dataIn)
{
    u32 i;

    aes_print_hex_dump("\t=>", (void *)dataIn, AES_BLOCK_SIZE);

    /* Data is written in 32-bit chunks */
    for (i = 0; i < (AES_BLOCK_SIZE / sizeof(u32)); i++, dataIn++)
    {
        iowrite32(__le32_to_cpu(*dataIn),AES_FIFO_DATA);
    }			
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_ReadBlock                                                                          */
/*                                                                                                         */
/* Parameters:      dataOut - Output data for ciphering [output].                                          */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Read data block dataOut from the FIFO buffer                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_ReadBlock (u32 *dataOut)
{
    u32 i;

    u32 *pReadData = dataOut;

    /* Data is read in 32-bit chunks */
     for (i = 0; i < (AES_BLOCK_SIZE / sizeof(u32)); i++, dataOut++)
    {       
        *dataOut = __cpu_to_le32(ioread32(AES_FIFO_DATA)); 
    }

     
    aes_print_hex_dump("\t\t\t\t\t\t\t\t\t\t  <= ",(void *)pReadData, AES_BLOCK_SIZE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_CryptData                                                                          */
/*                                                                                                         */
/* Parameters:      size    - Byte length of input data [input].                                           */
/*                  dataIn  - Input data for ciphering [input].                                            */
/*                  dataOut - Output data for ciphering [output].                                          */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Encrypt/Decrypt a message, possibly made up of multiple blocks.                                    */
/*      Used in all AES modes of operations except CBC-MAC.                                                */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_CryptData (
    u32  size,
    u32 *dataIn,
    u32 *dataOut
)
{
    u32 totalBlocks;
    u32 blocksLeft;
    /* dataIn/dataOut is advanced in 32-bit chunks */
    u32 AesDataBlock = (AES_BLOCK_SIZE / sizeof(u32));


    aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_CryptData: size %d\n", size);
     

    /* Calculate the number of complete blocks */
    totalBlocks = blocksLeft = AES_COMPLETE_BLOCKS(size);

    /* Quit if there is no complete blocks */
    if (totalBlocks == 0)
    {
        return;
    }

    /* Write the first block */
    if (totalBlocks > 1)
    {
        AES_WriteBlock(dataIn);
        dataIn += AesDataBlock;
        blocksLeft--;
    }

    /* Write the second block */
    if (totalBlocks > 2)
    {
        CRYPTO_TIMEOUT(!AES_DIN_FIFO_IS_EMPTY(), SECOND_TIMEOUT) 
        AES_WriteBlock(dataIn);
        dataIn += AesDataBlock;
        blocksLeft--;
    }

    /* Write & read available blocks */
    while (blocksLeft > 0)
    {
        /* Wait till DOUT FIFO is not full */
        CRYPTO_TIMEOUT(AES_DIN_FIFO_IS_FULL(), SECOND_TIMEOUT)

        /* Write next block */
        AES_WriteBlock(dataIn);
        dataIn  += AesDataBlock;

        /* Wait till DOUT FIFO is not empty */
        CRYPTO_TIMEOUT(AES_DOUT_FIFO_IS_EMPTY(), SECOND_TIMEOUT)

        /* Read next block */
        AES_ReadBlock(dataOut);
        dataOut += AesDataBlock;

        blocksLeft--;
    }

    if (totalBlocks > 2)
    {
        CRYPTO_TIMEOUT(!AES_DOUT_FIFO_IS_FULL(), SECOND_TIMEOUT)
        /* Read next block */
        AES_ReadBlock(dataOut);
        dataOut += AesDataBlock;
        CRYPTO_TIMEOUT(!AES_DOUT_FIFO_IS_FULL(), SECOND_TIMEOUT)
        /* Read next block */
        AES_ReadBlock(dataOut);
        dataOut += AesDataBlock;
    }
    else if (totalBlocks > 1)
    {
        CRYPTO_TIMEOUT(!AES_DOUT_FIFO_IS_FULL(), SECOND_TIMEOUT)
        /* Read next block */
        AES_ReadBlock(dataOut);
        dataOut += AesDataBlock;
    }

}

#if 0 // used for  CONFIG_CRYPTO_CCM
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_FeedMessage                                                                        */
/*                                                                                                         */
/* Parameters:      size    - Byte length of input data [input].                                           */
/*                  dataIn  - Input data for ciphering [input].                                            */
/*                  dataOut - Output data for ciphering [output].                                          */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*      Encrypt/Decrypt a message, possibly made up of multiple blocks.                                    */
/*      CBC-MAC mode only, no need to read output (output is IV).                                          */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_FeedMessage (
    u32  size,
    u32 *dataIn,
    u32 *dataOut
)
{
    u32 totalBlocks;
    u32 blocksLeft;
    /* dataIn/dataOut is advanced in 32-bit chunks */
    u8 AesDataBlock = (AES_BLOCK_SIZE / sizeof(u32));

    aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES_FeedMessage: size %d\n", size);

    /* Calculate the number of complete blocks */
    totalBlocks = blocksLeft = AES_COMPLETE_BLOCKS(size);

    /* Quit if there is no complete blocks */
    if (totalBlocks == 0)
    {
        return;
    }

    /* Write the first block */
    if (totalBlocks > 1)
    {
        AES_WriteBlock(dataIn);
        dataIn += AesDataBlock;
        blocksLeft--;
    }

    /* Write the second block */
    if (totalBlocks > 2)
    {
        while (!AES_DIN_FIFO_IS_EMPTY());
        AES_WriteBlock(dataIn);
        dataIn += AesDataBlock;
        blocksLeft--;
    }

    /* Write & read available blocks */
    while (blocksLeft > 0)
    {
        /* Wait till DOUT FIFO is not full */
        while (AES_DIN_FIFO_IS_FULL());

        /* Write next block */
        AES_WriteBlock(dataIn);
        dataIn  += AesDataBlock;

        blocksLeft--;
    }

    while (AES_IS_BUSY());

    /* Read the last block */
    AES_ReadIV(dataOut);
    dataOut += AesDataBlock;
}
#endif // CONFIG_CRYPTO_CCM


#ifdef TEST_AES
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        AES_PrintRegs                                                                          */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine prints the module registers                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void AES_PrintRegs (void)
{

    aes_print(KERN_NOTICE  "\t\t\t*/*--------------*/\n");
    aes_print(KERN_NOTICE  "\t\t\t*/*     AES      */\n");
    aes_print(KERN_NOTICE  "\t\t\t*/*--------------*/\n\n");

    aes_print(KERN_NOTICE  "\t\t\t*AES_KEY_0              = 0x%08X at 0x%08X\n",   ioread32(AES_KEY_0)            , (u32)(AES_KEY_0)              );
    aes_print(KERN_NOTICE  "\t\t\t*AES_IV_0               = 0x%08X at 0x%08X\n",   ioread32(AES_IV_0)             , (u32)(AES_IV_0)               );
	aes_print(KERN_NOTICE  "\t\t\t*AES_CTR0               = 0x%08X at 0x%08X\n",   ioread32(AES_CTR0)             , (u32)(AES_CTR0)               );
	aes_print(KERN_NOTICE  "\t\t\t*AES_BUSY               = 0x%08X at 0x%08X\n",   ioread32(AES_BUSY)             , (u32)(AES_BUSY)               );
	aes_print(KERN_NOTICE  "\t\t\t*AES_SK                 = 0x%08X at 0x%08X\n",   ioread32(AES_SK)               , (u32)(AES_SK)                 );
	aes_print(KERN_NOTICE  "\t\t\t*AES_PREV_IV_0          = 0x%08X at 0x%08X\n",   ioread32(AES_PREV_IV_0)        , (u32)(AES_PREV_IV_0)          );
	aes_print(KERN_NOTICE  "\t\t\t*AES_DIN_DOUT           = 0x%08X at 0x%08X\n",   ioread32(AES_DIN_DOUT)         , (u32)(AES_DIN_DOUT)           );
	aes_print(KERN_NOTICE  "\t\t\t*AES_CONTROL            = 0x%08X at 0x%08X\n",   ioread32(AES_CONTROL)          , (u32)(AES_CONTROL)            );
	aes_print(KERN_NOTICE  "\t\t\t*AES_VERSION            = 0x%08X at 0x%08X\n",   ioread32(AES_VERSION)          , (u32)(AES_VERSION)            );
	aes_print(KERN_NOTICE  "\t\t\t*AES_HW_FLAGS           = 0x%08X at 0x%08X\n",   ioread32(AES_HW_FLAGS)         , (u32)(AES_HW_FLAGS)           );
	aes_print(KERN_NOTICE  "\t\t\t*AES_SW_RESET           = 0x%08X at 0x%08X\n",   ioread32(AES_SW_RESET)         , (u32)(AES_SW_RESET)           );
	aes_print(KERN_NOTICE  "\t\t\t*AES_DFA_ERROR_STATUS   = 0x%08X at 0x%08X\n",   ioread32(AES_DFA_ERROR_STATUS) , (u32)(AES_DFA_ERROR_STATUS)   );
	aes_print(KERN_NOTICE  "\t\t\t*AES_RBG_SEEDING_READY  = 0x%08X at 0x%08X\n",   ioread32(AES_RBG_SEEDING_READY), (u32)(AES_RBG_SEEDING_READY)  );
	aes_print(KERN_NOTICE  "\t\t\t*AES_FIFO_DATA          = 0x%08X at 0x%08X\n",   ioread32(AES_FIFO_DATA)        , (u32)(AES_FIFO_DATA)          );
	aes_print(KERN_NOTICE  "\t\t\t*AES_FIFO_STATUS        = 0x%08X at 0x%08X\n\n", ioread32(AES_FIFO_STATUS)      , (u32)(AES_FIFO_STATUS)        );
}
#else
static void AES_PrintRegs (void)
{
		return;
}
#endif




/********************** AF_ALG ********************************/



static inline struct npcmx50_aes_ctx *npcmx50_aes_ctx_common(void *ctx);
static inline struct npcmx50_aes_ctx *npcmx50_aes_ctx(struct crypto_tfm *tfm);
static inline struct npcmx50_aes_ctx *npcmx50_blk_aes_ctx(struct crypto_ablkcipher *tfm);
static int npcmx50_aes_set_key(struct crypto_ablkcipher *tfm, const u8 *in_key, unsigned int key_len);
static inline void npcmx50_reset_key(void);

#ifdef CONFIG_CRYPTO_ECB
static int npcmx50aes_ecb_encrypt(struct ablkcipher_request *req );
static int npcmx50aes_ecb_decrypt(struct ablkcipher_request *req );
#endif

#ifdef CONFIG_CRYPTO_CBC
static int npcmx50aes_cbc_encrypt(struct ablkcipher_request *req);
static int npcmx50aes_cbc_decrypt(struct ablkcipher_request *req);	
#endif

#ifdef CONFIG_CRYPTO_CTR
static int npcmx50aes_ctr_encrypt(struct ablkcipher_request *req);
static int npcmx50aes_ctr_decrypt(struct ablkcipher_request *req);
#endif

#if 0 // CONFIG_CRYPTO_CCM
static int npcmx50_ccm_aes_set_key(struct crypto_aead *req, const u8 *in_key, unsigned int key_len);
static int npcmx50aes_cbc_mac_encrypt(struct aead_request *req);
static int npcmx50aes_cbc_mac_decrypt(struct aead_request *req);
#endif
static int  npcmx50aes_cra_init(struct crypto_tfm *tfm);
static void npcmx50aes_cra_exit(struct crypto_tfm *tfm);
static void npcmx50aes_unregister_algs(void);
static int  npcmx50aes_register_algs(void);




/* Hold the key in context instead of inside the HW
 * This is to limit the area between spinlock.
 */
struct npcmx50_aes_ctx {
    u8             in_key[AES_MAX_KEY_SIZE];
    unsigned int   key_len; 
	int            useHRK;
	int            key_num; // HRK key num (0..3)
};





static struct crypto_alg aes_algs[] = {
#ifdef CONFIG_CRYPTO_ECB
    {
    	.cra_name		= "ecb(aes)",
    	.cra_driver_name	= "Nuvoton-ecb-aes",
    	.cra_priority		= 300,
    	.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
    	.cra_blocksize		= AES_BLOCK_SIZE,
    	.cra_ctxsize		= sizeof(struct npcmx50_aes_ctx),
    	.cra_alignmask		= 0,  // 0xf
    	.cra_type		= &crypto_ablkcipher_type,
    	.cra_module		= THIS_MODULE,
    	.cra_init		= npcmx50aes_cra_init,
    	.cra_exit		= npcmx50aes_cra_exit,
    	.cra_u.ablkcipher = {
    		.min_keysize	= AES_MIN_KEY_SIZE,
    		.max_keysize	= AES_MAX_KEY_SIZE,
    		.setkey		= npcmx50_aes_set_key,
    		.encrypt	= npcmx50aes_ecb_encrypt,
    		.decrypt	= npcmx50aes_ecb_decrypt,
    	}
    } ,
#endif

#ifdef CONFIG_CRYPTO_CBC
    {
    	.cra_name		= "cbc(aes)",
    	.cra_driver_name	= "Nuvoton-cbc-aes",
    	.cra_priority		= 300,
    	.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
    	.cra_blocksize		= AES_BLOCK_SIZE,
    	.cra_ctxsize		= sizeof(struct npcmx50_aes_ctx),
    	.cra_alignmask		= 0,
    	.cra_type		= &crypto_ablkcipher_type,
    	.cra_module		= THIS_MODULE,
    	.cra_init		= npcmx50aes_cra_init,
    	.cra_exit		= npcmx50aes_cra_exit,
    	.cra_u.ablkcipher = {
    		.min_keysize	= AES_MIN_KEY_SIZE,
    		.max_keysize	= AES_MAX_KEY_SIZE,
    		.ivsize		= AES_BLOCK_SIZE,
    		.setkey		= npcmx50_aes_set_key,
    		.encrypt	= npcmx50aes_cbc_encrypt,
    		.decrypt	= npcmx50aes_cbc_decrypt,
    	}
    } ,
#endif

#ifdef CONFIG_CRYPTO_CTR
    {
    	.cra_name		= "ctr(aes)",
    	.cra_driver_name	= "Nuvoton-ctr-aes",
    	.cra_priority		= 300,
    	.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
    	.cra_blocksize		= 16,
    	.cra_ctxsize		= sizeof(struct npcmx50_aes_ctx),
    	.cra_alignmask		= 0,
    	.cra_type		= &crypto_ablkcipher_type,
    	.cra_module		= THIS_MODULE,
    	.cra_init		= npcmx50aes_cra_init,
    	.cra_exit		= npcmx50aes_cra_exit,
    	.cra_u.ablkcipher = {
    		.min_keysize	= AES_MIN_KEY_SIZE,
    		.max_keysize	= AES_MAX_KEY_SIZE,
    		.ivsize		= AES_MAX_CTR_SIZE,
    		.setkey		= npcmx50_aes_set_key,
    		.encrypt	= npcmx50aes_ctr_encrypt,
    		.decrypt	= npcmx50aes_ctr_decrypt,
    	}
    }
#endif    
#if 0 // CONFIG_CRYPTO_CCM    
    ,{
        .cra_name       = "ccm(aes)",   // cbc-mac
        .cra_driver_name    = "Nuvoton-cbc-mac-aes-ccm",
        .cra_priority       = 300,
        .cra_flags      = CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC ,
        .cra_blocksize      = 16,
        .cra_ctxsize        = sizeof(struct npcmx50_aes_ctx),
        .cra_alignmask      = 0,
        .cra_type       = &crypto_aead_type,
        .cra_module     = THIS_MODULE,
        .cra_init       = npcmx50aes_cra_init,
        .cra_exit       = npcmx50aes_cra_exit,
        .cra_aead       = {
            .maxauthsize    = AES_MAX_KEY_SIZE,
            .ivsize         = AES_BLOCK_SIZE,
            .setkey         = npcmx50_ccm_aes_set_key,
            .encrypt        = npcmx50aes_cbc_mac_encrypt,
            .decrypt        = npcmx50aes_cbc_mac_decrypt,

        }
    }
#endif // CONFIG_CRYPTO_CCM	
};


static inline struct npcmx50_aes_ctx *npcmx50_aes_ctx_common(void *ctx)
{
	unsigned long addr = (unsigned long)ctx;
	unsigned long align = 4; 

	if (align <= crypto_tfm_ctx_alignment())
		align = 1;
		
	return (struct npcmx50_aes_ctx *)addr;
}

static inline struct npcmx50_aes_ctx *npcmx50_aes_ctx(struct crypto_tfm *tfm)
{
	return npcmx50_aes_ctx_common(crypto_tfm_ctx(tfm));
}

static inline struct npcmx50_aes_ctx *npcmx50_blk_aes_ctx(struct crypto_ablkcipher *tfm)
{
	return npcmx50_aes_ctx_common(crypto_ablkcipher_ctx(tfm));
}

#if 0 // used for CONFIG_CRYPTO_CCM
static inline struct npcmx50_aes_ctx *npcmx50_aead_aes_ctx(struct crypto_aead *tfm)
{
	return npcmx50_aes_ctx_common(crypto_aead_ctx(tfm));
}
#endif


static int npcmx50_aes_set_key_internal(struct npcmx50_aes_ctx *ctx, u32 *flags, const u8 *in_key, unsigned int key_len)
{
    aes_print("\t\t\t*npcmX50-AES: set key, key_len=%d bits, flags = 0x%08X\n", 8 * key_len, *flags); 
    
	if (key_len % 8) {
    	*flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
    	aes_print(KERN_ERR  "\t\t\t*npcmX50-AES: set key, bad key len\n");
    	return -EINVAL;
	}

    if ((key_len != 0) && (in_key != NULL))
    {
        memcpy(ctx->in_key, in_key, key_len);
        ctx->key_len = key_len;
		ctx->useHRK = 0; 
    }
    else
    {
        ctx->key_len = 0;
		ctx->useHRK = 1; 
    }
	
	// if key is equal to dummy key , and it's not a nul key:
	if(in_key != NULL) {
		ctx->useHRK = 1;
		/* Check for dummy */
        if (memcmp(&in_key[1], dummy, key_len - 1)) 
        {
            ctx->useHRK = 0;
        }
        
		// in HRK mode: select the key number according to the last byte on the dummy key.
		if(ctx->useHRK == 1)
		{
		    ctx->key_num = in_key[0];
		    aes_print("\nAES set key: use HRK%d\n", ctx->key_num);
		}
	}   

	return 0;
}


static int npcmx50_aes_set_key(struct crypto_ablkcipher *tfm, const u8 *in_key, unsigned int key_len)
{
	struct npcmx50_aes_ctx *ctx = npcmx50_blk_aes_ctx(tfm);
	u32 *flags = &tfm->base.crt_flags;

    return npcmx50_aes_set_key_internal(ctx, flags, in_key, key_len);
}

/* ====== Encryption/decryption routines ====== */
static inline void npcmx50_reset_key(void)
{
    aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: reset key\n");
    // reset and clear everything.
	AES_SOFTWARE_RESET();	
	SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
}



/*---------------------------------------------------------------------------------------------------------*/
/* ECB                                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef CONFIG_CRYPTO_ECB
static int npcmx50aes_ecb_encrypt(struct ablkcipher_request *req )
{
    struct ablkcipher_walk walk;
	struct crypto_tfm *tfm = req->base.tfm;
	struct npcmx50_aes_ctx *ctx = npcmx50_aes_ctx(tfm);
	
	unsigned int nbytes = req->nbytes;
	int err;

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb encrypt %d\n", nbytes);

	ablkcipher_walk_init(&walk, req->dst, req->src, nbytes);
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ablkcipher_walk_init dst=0x%x, src=0x%x, nbytes=%d\n", (u32)req->dst, (u32)req->src, nbytes);
	
	err = ablkcipher_walk_phys(req, &walk);	
	if (err)	
	{
	    pr_err("[%s]: ablkcipher_walk_phys() failed!",	__func__);
	    return err;
	}
	
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb encrypt 0\n");

	mutex_lock(&npcmx50_aes_lock);
	
	npcmx50_reset_key();

    /* Configure AES Engine */
    err = AES_Config(AES_OP_ENCRYPT, AES_MODE_ECB, NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len));
    if (err)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES set Configuration failed please try again\n"); 
        mutex_unlock(&npcmx50_aes_lock); 
        return err;
    }
    
 	// load from side band\external source:
    AES_LoadKey(((ctx->useHRK == 0)? (u32 *)ctx->in_key : NULL), NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len), ctx->key_num);

    
	/* Switch from configuration mode to data processing mode */
    AES_SWITCH_TO_DATA_MODE();

	while ((nbytes = walk.nbytes) != 0) {		
	    u32  *dest_paddr, *src_paddr;		
	    src_paddr =  phys_to_virt(page_to_phys(walk.src.page) +	walk.src.offset);		
	    dest_paddr = phys_to_virt(page_to_phys(walk.dst.page) + walk.dst.offset);		

	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
		aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb encrypt 1, nbytes=%d\n", nbytes);
		AES_CryptData((u32)nbytes & AES_BLOCK_MASK, (u32 *)src_paddr, (u32 *)dest_paddr);		
		
		nbytes &= AES_BLOCK_SIZE - 1;
		err = ablkcipher_walk_done(req, &walk, nbytes);
        if (err) 
            break;
	}

    if (!err) 
        ablkcipher_walk_complete(&walk); 
	mutex_unlock(&npcmx50_aes_lock);

	
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb encrypt done, err = %d\n", err);
	
	AES_PrintRegs();

	return err;

}

static int npcmx50aes_ecb_decrypt(struct ablkcipher_request *req )
{
    struct ablkcipher_walk walk;
	struct crypto_tfm *tfm = req->base.tfm;
	struct npcmx50_aes_ctx *ctx = npcmx50_aes_ctx(tfm);
	unsigned int nbytes = req->nbytes;
	int err;

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb decrypt %d\n", nbytes);

	
    
	ablkcipher_walk_init(&walk, req->dst, req->src, nbytes);
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ablkcipher_walk_init dst=0x%x, src=0x%x, nbytes=%d\n", (u32)req->dst, (u32)req->src, nbytes);
	err = ablkcipher_walk_phys(req, &walk);	
	if (err)	
	{
	    pr_err("[%s]: ablkcipher_walk_phys() failed!",	__func__);
	    return err;
	}

	mutex_lock(&npcmx50_aes_lock);

	npcmx50_reset_key();


	/* Configure AES Engine */
    err = AES_Config(AES_OP_DECRYPT, AES_MODE_ECB, NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len));
    if (err)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES set Configuration failed please try again\n"); 
        mutex_unlock(&npcmx50_aes_lock); 
        return err;
    }
    
 	// load from side band\external source:
    AES_LoadKey(((ctx->useHRK == 0)? (u32 *)ctx->in_key : NULL), NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len), ctx->key_num);
	    
	/* Switch from configuration mode to data processing mode */
    AES_SWITCH_TO_DATA_MODE();

	while ((nbytes = walk.nbytes) != 0) {		
	    u32  *dest_paddr, *src_paddr;		
	    src_paddr =  phys_to_virt(page_to_phys(walk.src.page) +	walk.src.offset);		
	    dest_paddr = phys_to_virt(page_to_phys(walk.dst.page) + walk.dst.offset);	

	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
	    aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb decrypt 1, nbytes=%d\n", nbytes);
		AES_CryptData((u32)nbytes & AES_BLOCK_MASK, (u32 *)src_paddr, (u32 *)dest_paddr);	
		nbytes &= AES_BLOCK_SIZE - 1;
		err = ablkcipher_walk_done(req, &walk, nbytes);
        if (err) 
            break;
	}
    if (!err) 
        ablkcipher_walk_complete(&walk); 

	mutex_unlock(&npcmx50_aes_lock);

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ecb decrypt done, err = %d\n", err);
	
	AES_PrintRegs();

	return err;

}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* CBC                                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef CONFIG_CRYPTO_CBC
static int npcmx50aes_cbc_encrypt(struct ablkcipher_request *req)
{
	
	struct ablkcipher_walk walk;
	struct crypto_tfm *tfm = req->base.tfm;
	struct npcmx50_aes_ctx *ctx = npcmx50_aes_ctx(tfm);
	unsigned int nbytes = req->nbytes;
	int err;
	

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: cbc encrypt %d\n", nbytes);
    
	ablkcipher_walk_init(&walk, req->dst, req->src, nbytes);
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ablkcipher_walk_init dst=0x%x, src=0x%x, nbytes=%d\n", (u32)req->dst, (u32)req->src, nbytes);
	err = ablkcipher_walk_phys(req, &walk);	
	if (err)	
	{
	    pr_err("[%s]: ablkcipher_walk_phys() failed!",	__func__);
	    return err;
	}
    mutex_lock(&npcmx50_aes_lock);
	
	npcmx50_reset_key();
	    
	/* Configure AES Engine */
    err = AES_Config(AES_OP_ENCRYPT, AES_MODE_CBC, NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len));
    if (err)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES set Configuration failed please try again\n"); 
        mutex_unlock(&npcmx50_aes_lock); 
        return err;
    }
    
 	// load from side band\external source:
    AES_LoadKey(((ctx->useHRK == 0)? (u32 *)ctx->in_key : NULL), NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len), ctx->key_num);


	/* Switch from configuration mode to data processing mode */
    AES_SWITCH_TO_DATA_MODE();

    AES_LoadIV((u32 *)walk.iv, AES_BLOCK_SIZE);
    
	while ((nbytes = walk.nbytes) != 0) {		
	    u32  *dest_paddr, *src_paddr;		
	    src_paddr =  phys_to_virt(page_to_phys(walk.src.page) +	walk.src.offset);		
	    dest_paddr = phys_to_virt(page_to_phys(walk.dst.page) + walk.dst.offset);		

	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
		AES_CryptData((u32)nbytes & AES_BLOCK_MASK, (u32 *)src_paddr, (u32 *)dest_paddr);	
		nbytes &= AES_BLOCK_SIZE - 1;
		err = ablkcipher_walk_done(req, &walk, nbytes);
        if (err) 
            break;
	}
    if (!err) 
        ablkcipher_walk_complete(&walk); 

	mutex_unlock(&npcmx50_aes_lock);

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: cbc encrypt done, err = %d\n", err);
	
	AES_PrintRegs();

	return err;
}

static int npcmx50aes_cbc_decrypt(struct ablkcipher_request *req)
{	
	struct ablkcipher_walk walk;
	struct crypto_tfm *tfm = req->base.tfm;
	struct npcmx50_aes_ctx *ctx = npcmx50_aes_ctx(tfm);
	unsigned int nbytes = req->nbytes;
	int err;
	
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: cbc decrypt %d\n", nbytes);
    
	ablkcipher_walk_init(&walk, req->dst, req->src, nbytes);
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ablkcipher_walk_init dst=0x%x, src=0x%x, nbytes=%d\n", (u32)req->dst, (u32)req->src, nbytes);
	err = ablkcipher_walk_phys(req, &walk);	
	if (err)	
	{
	    pr_err("[%s]: ablkcipher_walk_phys() failed!",	__func__);
	    return err;
	}
    mutex_lock(&npcmx50_aes_lock);
	
	npcmx50_reset_key();
	    
	/* Configure AES Engine */
    err = AES_Config(AES_OP_DECRYPT, AES_MODE_CBC, NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len)); 
    if (err)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES set Configuration failed please try again\n"); 
        mutex_unlock(&npcmx50_aes_lock); 
        return err;
    }
    
 	// load from side band\external source:
    AES_LoadKey(((ctx->useHRK == 0)? (u32 *)ctx->in_key : NULL), NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len), ctx->key_num);

    AES_LoadIV((u32 *)walk.iv, AES_BLOCK_SIZE);

	/* Switch from configuration mode to data processing mode */
    AES_SWITCH_TO_DATA_MODE();
    
	while ((nbytes = walk.nbytes) != 0) {		
	    u32  *dest_paddr, *src_paddr;		
	    src_paddr =  phys_to_virt(page_to_phys(walk.src.page) +	walk.src.offset);		
	    dest_paddr = phys_to_virt(page_to_phys(walk.dst.page) + walk.dst.offset);	

	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
		AES_CryptData((u32)nbytes & AES_BLOCK_MASK, (u32 *)src_paddr, (u32 *)dest_paddr);	
		nbytes &= AES_BLOCK_SIZE - 1;
		err = ablkcipher_walk_done(req, &walk, nbytes);
        if (err) 
            break;
	}
    if (!err) 
        ablkcipher_walk_complete(&walk); 

	mutex_unlock(&npcmx50_aes_lock);

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: cbc decrypt done, err = %d\n", err);
	
	AES_PrintRegs();

	return err;

}
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* CTR                                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef CONFIG_CRYPTO_CTR
static int npcmx50aes_ctr_encrypt(struct ablkcipher_request *req)
{
	
	struct ablkcipher_walk walk;
	struct crypto_tfm *tfm = req->base.tfm;
	struct npcmx50_aes_ctx *ctx = npcmx50_aes_ctx(tfm);
	unsigned int nbytes = req->nbytes;
	int err;

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ctr encrypt %d\n", nbytes);
    
	ablkcipher_walk_init(&walk, req->dst, req->src, nbytes);
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ablkcipher_walk_init dst=0x%x, src=0x%x, nbytes=%d\n", (u32)req->dst, (u32)req->src, nbytes);
	err = ablkcipher_walk_phys(req, &walk);	
	if (err)	
	{
	    pr_err("[%s]: ablkcipher_walk_phys() failed!",	__func__);
	    return err;
	}
    mutex_lock(&npcmx50_aes_lock);
	
	npcmx50_reset_key();
	    
	/* Configure AES Engine */
    err = AES_Config(AES_OP_ENCRYPT, AES_MODE_CTR, NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len)); 
    if (err)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES set Configuration failed please try again\n"); 
        mutex_unlock(&npcmx50_aes_lock); 
        return err;
    }
    
 	// load from side band\external source:
    AES_LoadKey(((ctx->useHRK == 0)? (u32 *)ctx->in_key : NULL), NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len), ctx->key_num);


	/* Switch from configuration mode to data processing mode */
    AES_SWITCH_TO_DATA_MODE();

    AES_LoadCTR((u32 *)walk.iv);
    
	while ((nbytes = walk.nbytes) != 0) {		
	    u32  *dest_paddr, *src_paddr;		
	    src_paddr =  phys_to_virt(page_to_phys(walk.src.page) +	walk.src.offset);		
	    dest_paddr = phys_to_virt(page_to_phys(walk.dst.page) + walk.dst.offset);		

	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
		AES_CryptData((u32)nbytes & AES_BLOCK_MASK, (u32 *)src_paddr, (u32 *)dest_paddr);			
		nbytes &= AES_BLOCK_SIZE - 1;
        err = ablkcipher_walk_done(req, &walk, nbytes); 
        if (err) 
            break;
	}
    if (!err) 
        ablkcipher_walk_complete(&walk); 

	mutex_unlock(&npcmx50_aes_lock);

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ctr encrypt done, err = %d\n", err);
	
	AES_PrintRegs();

	return err;
}

static int npcmx50aes_ctr_decrypt(struct ablkcipher_request *req)
{
	
	struct ablkcipher_walk walk;
	struct crypto_tfm *tfm = req->base.tfm;
	struct npcmx50_aes_ctx *ctx = npcmx50_aes_ctx(tfm);
	unsigned int nbytes = req->nbytes;
	int err;
	

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ctr decrypt %d\n", nbytes);
    
	ablkcipher_walk_init(&walk, req->dst, req->src, nbytes);
	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ablkcipher_walk_init dst=0x%x, src=0x%x, nbytes=%d\n", (u32)req->dst, (u32)req->src, nbytes);
	err = ablkcipher_walk_phys(req, &walk);	
	if (err)	
	{
	    pr_err("[%s]: ablkcipher_walk_phys() failed!",	__func__);
	    return err;
	}
    mutex_lock(&npcmx50_aes_lock);
	
	npcmx50_reset_key();
	    
	/* Configure AES Engine */
    err = AES_Config(AES_OP_DECRYPT, AES_MODE_CTR, NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len));
    if (err)
    {
        aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: AES set Configuration failed please try again\n"); 
        mutex_unlock(&npcmx50_aes_lock); 
        return err;
    }
    
 	// load from side band\external source:
    AES_LoadKey(((ctx->useHRK == 0)? (u32 *)ctx->in_key : NULL), NPCMX50_AES_KEY_SIZE_TO_ENUM(ctx->key_len), ctx->key_num);

    AES_LoadCTR((u32 *)walk.iv);
    
	/* Switch from configuration mode to data processing mode */
    AES_SWITCH_TO_DATA_MODE();
    
	while ((nbytes = walk.nbytes) != 0) {		
	    u32  *dest_paddr, *src_paddr;		
	    src_paddr =  phys_to_virt(page_to_phys(walk.src.page) +	walk.src.offset);		
	    dest_paddr = phys_to_virt(page_to_phys(walk.dst.page) + walk.dst.offset);	
	    
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DIN_FIFO_OVERFLOW, 1);
	    SET_REG_FIELD(AES_FIFO_STATUS, AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW, 1);
		AES_CryptData((u32)nbytes & AES_BLOCK_MASK, (u32 *)src_paddr, (u32 *)dest_paddr);	
		nbytes &= AES_BLOCK_SIZE - 1;
		err = ablkcipher_walk_done(req, &walk, nbytes);
        if (err) 
            break;
	}
    
    if (!err) 
        ablkcipher_walk_complete(&walk); 

	mutex_unlock(&npcmx50_aes_lock);

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: ctr decrypt done, err = %d\n", err);
	
	AES_PrintRegs();

	return err;

}
#endif



static int npcmx50aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct npcmx50_aes_ctx);

	return 0;
}

static void npcmx50aes_cra_exit(struct crypto_tfm *tfm)
{
}

static void npcmx50aes_unregister_algs(/* struct npcmx50aes_dev *dd*/ )
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aes_algs); i++)
		crypto_unregister_alg(&aes_algs[i]);
}

static int npcmx50aes_register_algs(/*struct npcmx50aes_dev *dd*/ )
{
	int err, i, j;

	aes_print(KERN_INFO "\t\t\t* AES register algo\n");

	for (i = 0; i < ARRAY_SIZE(aes_algs); i++) {
		err = crypto_register_alg(&aes_algs[i]);
		if (IS_ERR_VALUE(err))
			goto err_aes_algs;
	}
	

	return 0;

err_aes_algs:
    aes_print(KERN_INFO "\t\t\t* AES register algo fail\n");
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&aes_algs[j]);

	return err;
}



static int npcm750aes_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	aes_print(KERN_INFO "\t\t\t* AES probe start\n");
	/*
	 * A bit ugly, and it will never actually happen but there can
	 * be only one RNG and this catches any bork
	 */
	if (aes_dev)
		return -EBUSY;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		ret = -ENOENT;
		goto err_region;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		ret = -EBUSY;
		goto err_region;
	}

	dev_set_drvdata(&pdev->dev, res);
	aes_base = ioremap(res->start, resource_size(res));
	aes_print(KERN_INFO "\t\t\t* AES base is 0x%08X \n", (u32)aes_base);
	if (!aes_base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}
    
	ret = npcmx50aes_register_algs();
	if (IS_ERR_VALUE(ret)) {
		goto err_register;
	}
	aes_dev = pdev;

    printk(KERN_INFO "NPCMx50: AES module is ready\n");

    AES_PrintRegs();


	return 0;

err_register:
	iounmap(aes_base);
	aes_base = NULL;
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_region:
    printk(KERN_INFO "* NPCMx50: AES module load fail .  Error %d\n", ret);

	return ret;
}

static int __exit npcmx50aes_remove(struct platform_device *pdev)
{
	struct resource *res = dev_get_drvdata(&pdev->dev);

	npcmx50aes_unregister_algs();

	aes_print(KERN_NOTICE  "\t\t\t*npcmX50-AES: remove: stop using Nuvoton npcmx50 AES algorithm.\n");

	iounmap(aes_base);

	release_mem_region(res->start, resource_size(res));
	aes_base = NULL;
	return 0;
}


/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750_aes");

static const struct of_device_id aes_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-aes",  },
	{},
};
MODULE_DEVICE_TABLE(of, aes_dt_id);

static struct platform_driver npcmx50aes_driver = {
	.probe		= npcm750aes_probe,
	.remove		= npcmx50aes_remove,
	.shutdown	= NULL,  // npcmx50aes_shutdown,
	.suspend	= NULL,  // npcmx50aes_suspend,
	.resume		= NULL,  // npcmx50aes_resume,
	.driver		= {
		.name	= "npcm750_aes",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(aes_dt_id),
	}
};


module_platform_driver(npcmx50aes_driver);


MODULE_DESCRIPTION("Nuvoton Technologies AES HW acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Tali Perry - Nuvoton Technologies");
