/*
 * NPCMP750 BMC On-Chip OTP (FUSE) Memory Interface
 *
 * Copyright 2016 Nuvoton Technologies
 *
 * Licensed under the GPL-2 or later.
 *
 * based on blackfin OTP.
 *
 */

#ifdef CONFIG_NPCM750_OTP

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mtd/mtd-abi.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/of_irq.h>

//#include <mach/map.h>
//#include <mach/hal.h>
//#include <defs.h>
#include "npcm750_otp.h"

#ifdef REG_READ
#undef REG_READ
#endif
static inline u32 REG_READ(unsigned int __iomem *mem ) {
    return ioread32(mem);
}

#ifdef REG_WRITE
#undef REG_WRITE
#endif   
static inline void REG_WRITE(unsigned int __iomem *mem, u32 val ) {
    iowrite32(val, mem);
}

#ifdef SET_REG_FIELD
#undef SET_REG_FIELD
#endif  
/*---------------------------------------------------------------------------------------------------------*/
/* Set field of a register / variable according to the field offset and size                               */
/*---------------------------------------------------------------------------------------------------------*/
static inline void SET_REG_FIELD(unsigned int __iomem *mem, bit_field_t bit_field, u32 val) {
    u32 tmp = ioread32(mem);
    tmp &= ~(((1 << bit_field.size) - 1) << bit_field.offset); // mask the field size
    tmp |= val << bit_field.offset;  // or with the requested value
    iowrite32(tmp, mem);
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
/*---------------------------------------------------------------------------------------------------------*/
/* Get field of a register / variable according to the field offset and size                               */
/*---------------------------------------------------------------------------------------------------------*/
static inline u8 READ_REG_FIELD(unsigned int __iomem *mem, bit_field_t bit_field) {
    u8 tmp = ioread32(mem);
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
/*---------------------------------------------------------------------------------------------------------*/
/* Build a mask of a register / variable field                                                             */
/*---------------------------------------------------------------------------------------------------------*/
// bit_field should be of bit_field_t type
#define MASK_FIELD(bit_field) \
    (((1 << bit_field.size) - 1) << bit_field.offset) /* mask the field size */ 

#ifdef BUILD_FIELD_VAL
#undef BUILD_FIELD_VAL
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Expand the value of the given field into its correct position                                           */
/*---------------------------------------------------------------------------------------------------------*/
// bit_field should be of bit_field_t type
#define BUILD_FIELD_VAL(bit_field, value)  \
    ((((1 << bit_field.size) - 1) & (value)) << bit_field.offset)


#ifdef SET_REG_MASK
#undef SET_REG_MASK
#endif  
/*---------------------------------------------------------------------------------------------------------*/
/* Set field of a register / variable according to the field offset and size                               */
/*---------------------------------------------------------------------------------------------------------*/
static inline void SET_REG_MASK(unsigned int __iomem *mem, u32 val) {
    iowrite32(ioread32(mem) | val, mem);
}

#ifdef READ_VAR_BIT
#undef READ_VAR_BIT
#endif  
#define READ_VAR_BIT(var, nb)      (((var) >> (nb)) & 0x1)

#ifdef ASSERT
#undef ASSERT
#endif  
#define ASSERT(cond)

#define stamp(fmt, args...) pr_debug("%s:%i: " fmt "\n", __func__, __LINE__, ## args)

#define DRIVER_NAME "npcm750_otp"

static DEFINE_MUTEX(npcm750_otp_lock);


/* OTP MODULE registers */
static void __iomem *otp_base[2];
static struct platform_device *otp_dev;

static struct clk* otp_clk;


//#define CONFIG_npcm750_otp_EMULATE
#ifdef CONFIG_NPCM750_OTP_EMULATE
    u8 *fuse_array_mem = (u8 *)0x2000000;
    #define npcm750_otp_ARR_SIZE 1024
#endif

// #define OTP_DEBUG_MODULE 1
#ifdef OTP_DEBUG_MODULE
#define printk_dbg(x...)            printk(x)
#else
#define printk_dbg(x...)            (void)0
#endif
	

/*---------------------------------------------------------------------------------------------------------*/
/* Fuse module constant definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


// Read cycle initiation value:
#define READ_INIT                   0x02

// Program cycle initiation values (a sequence of two adjacent writes is required):
#define PROGRAM_ARM                 0x1
#define PROGRAM_INIT                0xBF79E5D0

// Value to clean FDATA contents:
#define FDATA_CLEAN_VALUE           0x01

// Default APB Clock Rate (in MHz):
#define DEFAULT_APB_RATE            0x30

#define MIN_PROGRAM_PULSES          4
#define MAX_PROGRAM_PULSES          20


/*---------------------------------------------------------------------------------------------------------*/
/* Fuse module local macro definitions                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
// #define STORAGE_ARRAY_READY(sa)   READ_REG_FIELD(FST(sa),     FST_RDY)
#define KEY_IS_VALID()            READ_REG_FIELD(FKEYIND,     FKEYIND_KVAL)
#define DISABLE_KEY_ACCESS()      SET_REG_FIELD(FCFG(NPCM750_KEY_SA), FCFG_FDIS, 1)      /* Lock OTP module access  */

static bool npcm750_otp_IsInitialized = false;


/*---------------------------------------------------------------------------------------------------------*/
/* Internal functions for this module                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

/* HAL functions */ 
static int            NPCM750_OTP_WaitForOTPReadyWithTimeout(NPCM750_OTP_STORAGE_ARRAY_T array, u32 timeout);
static int            NPCM750_OTP_Init                      (void);
static void           NPCM750_OTP_Read                      (NPCM750_OTP_STORAGE_ARRAY_T arr, u32 addr, u8 *data);
static int            NPCM750_OTP_ProgramBit                (NPCM750_OTP_STORAGE_ARRAY_T arr, u32 byteNum, u8 bitNum);
static int            NPCM750_OTP_ProgramByte               (NPCM750_OTP_STORAGE_ARRAY_T arr, u32 byteNum, u8 value);
static bool           NPCM750_OTP_BitIsProgrammed           (NPCM750_OTP_STORAGE_ARRAY_T arr, u32 byteNum, u8 bitNum);
#if (defined RSA_MODULE_TYPE) || (defined AES_MODULE_TYPE)
static void           NPCM750_OTP_UploadKey                 (AES_KEY_SIZE_T keySize, u8 keyIndex);
static int            NPCM750_OTP_ReadKey                   (NPCM750_OTP_KEY_TYPE_T  keyType, u8  keyIndex, u8 *output);
static int            NPCM750_OTP_SelectKey                 (u8 keyIndex);
#endif
static void           NPCM750_OTP_DisableKeyAccess          (void);
#ifdef ROM_CODE_ONLY  
static int            NPCM750_OTP_LockAccess                (NPCM750_OTP_STORAGE_ARRAY_T array, u8 lockForRead, u8 lockForWrite, bool lockRegister);
#endif
#ifdef _CODE_THAT_SHOULD_BE_IN_USER_   
static int            NPCM750_OTP_NibParEccDecode           (u8 *datain,    u8 *dataout,    u32  encoded_size);
static int            NPCM750_OTP_NibParEccEncode           (u8 *datain,    u8 *dataout,    u32  encoded_size);
static int            NPCM750_OTP_MajRulEccDecode           (u8 *datain,    u8 *dataout,    u32  encoded_size);
static int            NPCM750_OTP_MajRulEccEncode           (u8 *datain,    u8 *dataout,    u32  encoded_size);
static unsigned int           NPCM750_OTP_Fustrap_Get               (NPCM750_OTP_FUSTRAP_FIELDS_T oFuse); // TODO: is this needed for Linux? 
#endif // _CODE_THAT_SHOULD_BE_IN_USER_  
static void           NPCM750_PrintRegs                     (void);
static void           NPCM750_PrintModuleRegs               (NPCM750_OTP_STORAGE_ARRAY_T array);

/* Linux IF */
static ssize_t        npcm750_otp_read                      (struct file *file, char __user *buff, size_t count, loff_t *pos);
static ssize_t        npcm750_otp_write                     (struct file *filp, const char __user *buff, size_t count, loff_t *pos);
       long           npcm750_otp_ioctl                     (struct file *filp, unsigned cmd, unsigned long arg);
static int            npcm750_otp_probe                     (struct platform_device *pdev);
static int __exit     npcmx50_otp_remove                    (struct platform_device *pdev);
static int __init     npcm750_otp_init                      (void);
static void __exit    npcm750_otp_exit                      (void);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_WaitForOTPReadyWithTimeout                                                 */
/*                                                                                                         */
/* Parameters:      array - fuse array to wait for                                                         */
/* Returns:         int                                                                             */
/* Side effects:                                                                                           */
/* Description:     Initialize the Fuse HW module.                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_WaitForOTPReadyWithTimeout(NPCM750_OTP_STORAGE_ARRAY_T array, u32 timeout)
{
    volatile u32 time = timeout;

    /*-----------------------------------------------------------------------------------------------------*/
    /* check parameters validity                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    if (array > NPCM750_FUSE_SA)
    {
        return -EINVAL;
    }    

    while (--time > 1)
    {
        if (READ_REG_FIELD(FST(array), FST_RDY))
        {
            /* fuse is ready, clear the status. */
            SET_REG_FIELD(FST(array), FST_RDST, 1);

            return 0;
        }
    }
    /* try to clear the status in case it was set */
    SET_REG_FIELD(FST(array), FST_RDST, 1);

    return -EINVAL;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_Init                                                                       */
/*                                                                                                         */
/* Parameters:      APBclock - APB clock rate in MHz                                                       */
/* Returns:         int                                                                             */
/* Side effects:                                                                                           */
/* Description:     Initialize the Fuse HW module.                                                         */
/*---------------------------------------------------------------------------------------------------------*/
// TaliP: notice: this entire function is realy not needed. ROM code does it for us. Thanks, ROM. 
static int NPCM750_OTP_Init (void)
{

    // APBRT (APB Clock Rate). Informs the fuse array state machine on the APB clock rate in MHz. The
    // software must update this field before writing the OTP, and before APB4 actual clock rate change. The
    // state machine contains an internal copy of this field, sampled at the beginning of every read or program
    // operation. Software should not write this field with 0. The reset value of this field is 1Fh (31 MHz). The
    // accuracy of the setting should be 10%.
    // Note: The minimum APB allowed frequency for accessing the fuse arrays is 10 MHz.

    u8 APBclock = clk_get_rate(otp_clk)/1000000 + 1;

    /* Configure the Key Storage Array APB Clock Rate */
    SET_REG_FIELD(FCFG(NPCM750_KEY_SA), FCFG_APBRT, APBclock & 0x3F);

    /* Configure the Fuse Storage Array APB Clock Rate */
    SET_REG_FIELD(FCFG(NPCM750_FUSE_SA), FCFG_APBRT, APBclock & 0x3F);

    npcm750_otp_IsInitialized = true;

    return 0;


}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_Read                                                                       */
/*                                                                                                         */
/* Parameters:      arr  - Storage Array type [input].                                                     */
/*                  addr - Byte-address to read from [input].                                              */
/*                  data - Pointer to result [output].                                                     */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     Read 8-bit data from an OTP storage array.                                             */
/*---------------------------------------------------------------------------------------------------------*/
static void NPCM750_OTP_Read (NPCM750_OTP_STORAGE_ARRAY_T arr,
                u32               addr,
                u8               *data
)
{
    //if (!npcm750_otp_IsInitialized)
    //{
    //    NPCM750_OTP_Init();
    //}

#ifdef CONFIG_npcm750_otp_EMULATE
     *data = fuse_array_mem[(arr*npcm750_otp_ARR_SIZE)+addr];
#else
    /* Wait for the Fuse Box Idle */
    NPCM750_OTP_WaitForOTPReadyWithTimeout(arr, 0xDEADBEEF ); // TODO: decide proper timeout

    /* Configure the byte address in the fuse array for read operation */
    SET_REG_FIELD(FADDR(arr), FADDR_BYTEADDR, addr);

    /* Initiate a read cycle from the byte in the fuse array, pointed by FADDR */
    REG_WRITE(FCTL(arr),  READ_INIT);

    /* Wait for read operation completion */
    NPCM750_OTP_WaitForOTPReadyWithTimeout(arr, 0xDEADBEEF ); // TODO: decide proper timeout

    /* Read the result */
    *data = READ_REG_FIELD(FDATA(arr), FDATA_FDATA);

    /* Clean FDATA contents to prevent unauthorized software from reading sensitive information */
    SET_REG_FIELD(FDATA(arr), FDATA_FDATA, FDATA_CLEAN_VALUE);
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_BitIsProgrammed                                                                   */
/*                                                                                                         */
/* Parameters:      arr     - Storage Array type [input].                                                  */
/*                  byteNum - Byte offset in array [input].                                                */
/*                  bitNum  - Bit offset in byte [input].                                                  */
/* Returns:         Nonzero if bit is programmed, zero otherwise.                                          */
/* Side effects:                                                                                           */
/* Description:     Check if a bit is programmed in an OTP storage array.                                  */
/*---------------------------------------------------------------------------------------------------------*/
static bool NPCM750_OTP_BitIsProgrammed (
    NPCM750_OTP_STORAGE_ARRAY_T  arr,
    u32                byteNum,
    u8                 bitNum
)
{
    u8 data;
    
    if (!npcm750_otp_IsInitialized)
    {
        NPCM750_OTP_Init();
    }

    /* Read the entire byte you wish to program */
    NPCM750_OTP_Read(arr, byteNum, &data);

    /* Check whether the bit is already programmed */
    if (READ_VAR_BIT(data, bitNum))
    {
        return true;
    }
    return false;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_ProgramBit                                                                        */
/*                                                                                                         */
/* Parameters:      arr     - Storage Array type [input].                                                  */
/*                  byteNum - Byte offset in array [input].                                                */
/*                  bitNum  - Bit offset in byte [input].                                                  */
/* Returns:         int                                                                                   */
/* Side effects:                                                                                           */
/* Description:     Program (set to 1) a bit in an OTP storage array.                                      */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_ProgramBit (
    NPCM750_OTP_STORAGE_ARRAY_T arr,
    u32               byteNum,
    u8                bitNum
)
{
    int status = 0;
    
    if (!npcm750_otp_IsInitialized)
    {
        NPCM750_OTP_Init();
    }

    /* Wait for the Fuse Box Idle */
    NPCM750_OTP_WaitForOTPReadyWithTimeout(arr, 0xDEADBEEF ); // TODO: decide proper timeout

    /* Make sure the bit is not already programmed */
    if (! NPCM750_OTP_BitIsProgrammed(arr, byteNum, bitNum))
    {
        
#ifdef CONFIG_npcm750_otp_EMULATE
        fuse_array_mem[(arr* NPCM750_OTP_ARR_SIZE)+byteNum] |= 1<<bitNum;
#else
        u8 read_data;
        int count;
        
        /* Configure the bit address in the fuse array for program operation */
        SET_REG_FIELD(FADDR(arr), FADDR_BYTEADDR, byteNum);

        SET_REG_FIELD(FADDR(arr), FADDR_BITPOS, bitNum);

        // program up to MAX_PROGRAM_PULSES
        for (count=1; count<=MAX_PROGRAM_PULSES; count++)
        {
            /* Arm the program operation */
            REG_WRITE(FCTL(arr), PROGRAM_ARM);

        /* Initiate a program cycle to the bit in the fuse array, pointed by FADDR */
        REG_WRITE(FCTL(arr), PROGRAM_INIT);

        /* Wait for program operation completion */
        NPCM750_OTP_WaitForOTPReadyWithTimeout(arr, 0xDEADBEEF ); // TODO: decide proper timeout

            // after MIN_PROGRAM_PULSES start verifying the result
            if (count >= MIN_PROGRAM_PULSES)
            {
                /* Initiate a read cycle from the byte in the fuse array, pointed by FADDR */
                REG_WRITE(FCTL(arr),  READ_INIT);

                /* Wait for read operation completion */
                NPCM750_OTP_WaitForOTPReadyWithTimeout(arr, 0xDEADBEEF ); // TODO: decide proper timeout

                /* Read the result */
                read_data = READ_REG_FIELD(FDATA(arr), FDATA_FDATA);

                /* If the bit is set the sequence ended correctly */
                if (read_data & (1 << bitNum))
                    break;
            }
        }
        
        // check if programmking failed
        if (count > MAX_PROGRAM_PULSES)
        {
            status = -EINVAL;
        }

        /* Clean FDATA contents to prevent unauthorized software from reading sensitive information */
        SET_REG_FIELD(FDATA(arr), FDATA_FDATA, FDATA_CLEAN_VALUE);        
#endif
    }

    return status;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_ProgramByte                                                                       */
/*                                                                                                         */
/* Parameters:      arr     - Storage Array type [input].                                                  */
/*                  byteNum - Byte offset in array [input].                                                */
/*                  value   - Byte to program [input].                                                     */
/* Returns:         int                                                                                   */
/* Side effects:                                                                                           */
/* Description:     Program (set to 1) a given byte's relevant bits in an OTP                              */
/*                  storage array.                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_ProgramByte (
    NPCM750_OTP_STORAGE_ARRAY_T  arr,
    u32                byteNum,
    u8                 value
)
{
    unsigned int  i;

    u8 data;
    int status = 0;
    
    if (!npcm750_otp_IsInitialized)
    {
        NPCM750_OTP_Init();
    }
	
	if (byteNum > NPCM750_OTP_ARR_BYTE_SIZE)
		return -1;
	
	if(arr > NPCM750_FUSE_SA)
		return -1;

    /* Wait for the Fuse Box Idle */
    NPCM750_OTP_WaitForOTPReadyWithTimeout(arr, 0xDEADBEEF ); // TODO: decide proper timeout

    /* Read the entire byte you wish to program */
    NPCM750_OTP_Read(arr, byteNum, &data);

    /* In case all relevant bits are already programmed - nothing to do */
    if ((~data & value) == 0)
        return status;

    /* Program unprogrammed bits. */
    for (i = 0; i < 8; i++)
    {
        if (READ_VAR_BIT(value, i) == 1)
        {
            /* Program (set to 1) the relevant bit */
            int last_status = NPCM750_OTP_ProgramBit(arr, byteNum, (u8)i);
            if (last_status != 0)
            {
                status = last_status;
            }
        }
    }

    return status;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_SelectKey                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  keyIndex - AES key index in the key array (in 128-bit steps) [input].                  */
/* Returns:         0 on successful read completion, int_ERROR* otherwise.                                 */
/* Side effects:                                                                                           */
/* Description:     Select a key from the key storage array.                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_SelectKey (
    u8            keyIndex  )
{

    u32  fKeyInd = 0;
    volatile u32 time = 2000;
    
    /* check if access disabled to the first 2048 bits of the fuse array */
    if (READ_REG_FIELD(FCFG(NPCM750_KEY_SA), FCFG_FDIS) == 1)
    {
       return -EACCES;
    }

    if (!npcm750_otp_IsInitialized)
    {
        NPCM750_OTP_Init();
    }

    if (keyIndex >= 4 )
    {
        return -EINVAL;
    }

    printk_dbg(KERN_DEBUG "\tnpcm750_otp: select key = %d\n", keyIndex);

    /* Do not destroy ECCDIS bit */
    fKeyInd = REG_READ(FKEYIND);

    /* Configure the key size */
    SET_VAR_FIELD(fKeyInd, FKEYIND_KSIZE, FKEYIND_KSIZE_VALUE_256);

    /* Configure the key index (0 to 3) */
    SET_VAR_FIELD(fKeyInd, FKEYIND_KIND, keyIndex);

    REG_WRITE(FKEYIND, fKeyInd);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Wait for selection completetion                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    while (--time > 1)
    {
        if (READ_REG_FIELD(FKEYIND, FKEYIND_KVAL))
            return 0;

		udelay(1);
    }

    return -ETIMEDOUT;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_otp_DisableKeyAccess                                                           */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine disables read and write accees to key array                               */
/*---------------------------------------------------------------------------------------------------------*/
static void NPCM750_OTP_DisableKeyAccess ()
{
    if (!npcm750_otp_IsInitialized)
    {
        NPCM750_OTP_Init();
    }
    printk_dbg("\tnpcm750_otp: disable key\n");
    DISABLE_KEY_ACCESS();
}

// TaliP: will the Linux lock access to fuses? need ioctl for this?
#ifdef ROM_CODE_ONLY  
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_otp_LockAccess                                                                 */
/*                                                                                                         */
/* Parameters:      lockForRead: bitwise, which block to lock for reading                                  */
/* Parameters:      lockForWrite: bitwise, which block to lock for program                                 */
/* Returns:         int                                                                                    */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine lock the otp blocks                                                       */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_LockAccess (NPCM750_OTP_STORAGE_ARRAY_T array, u8 lockForRead, u8 lockForWrite, bool lockRegister)
{
    u32 FCFG_VAR = 0;
    
    if (!npcm750_otp_IsInitialized)
    {
        NPCM750_OTP_Init();
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* check parameters validity                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    if (array > NPCM750_FUSE_SA)
    {
        return HAL_ERROR_BAD_PARAM;
    }    

    /*-----------------------------------------------------------------------------------------------------*/
    /* Read reg for modify all fields apart APBRT                                                          */
    /*-----------------------------------------------------------------------------------------------------*/
    FCFG_VAR = REG_READ(FCFG(array));


    SET_VAR_FIELD(FCFG_VAR, FCFG_FRDLK,  lockForRead & 0x00FF);

    SET_VAR_FIELD(FCFG_VAR, FCFG_FPRGLK, lockForWrite & 0x00FF);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Lock any access to this register (until next POR)                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    if ( lockRegister == true)
    {
        SET_VAR_FIELD(FCFG_VAR, FCFG_FCFGLK, (lockForWrite | lockForRead) & 0x00FF);
    }

     /*----------------------------------------------------------------------------------------------------*/
     /* Lock the side band in case it's a key array, and read is locked                                    */
     /*----------------------------------------------------------------------------------------------------*/
    if ( array == NPCM750_KEY_SA)
    {
        /* Set FDIS bit if oKAP bit 7 is set, to disable the side-band key loading. */
        if ( (lockForRead & 0x80) > 0 )
        {
            SET_VAR_FIELD(FCFG_VAR, FCFG_FDIS, 1);  // 1: Access to the first 2048 bits of the fuse array is disabled.
        }
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Return the moified value                                                                            */
    /*-----------------------------------------------------------------------------------------------------*/
    REG_WRITE(FCFG(array), FCFG_VAR);

    return 0;
}
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Logical level functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

#ifdef _CODE_THAT_SHOULD_BE_IN_USER_  

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_otp_NibParEccDecode                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  datain -       pointer to encoded data buffer (buffer size should be 2 x dataout)      */
/*                  dataout -      pointer to decoded data buffer                                          */
/*                  encoded_size - size of encoded data (decoded data x 2)                                 */
/*                                                                                                         */
/* Returns:         int                                                                             */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Decodes the data according to nibble parity ECC scheme.                                */
/*                  Size specifies the encoded data size.                                                  */
/*                  Decodes whole bytes only                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_NibParEccDecode (
    u8  *datain,
    u8  *dataout,
    u32  encoded_size
)
{
    u32 i;
    u8 BER;
    u8 src_re_calc;
    u8 dst;
    u8 E0;
    u8 E1;
    u8 E2;
    u8 E3;
    u8 E4;
    u8 E5;
    u8 E6;
    u8 E7;
    int status = 0;

//Define the Bit Field macros in order to use the SET_VAR_FIELD macro:
#define BITF0   0, 1
#define BITF1   1, 1
#define BITF2   2, 1
#define BITF3   3, 1
#define BITF4   4, 1
#define BITF5   5, 1
#define BITF6   6, 1
#define BITF7   7, 1

#define LSNF    0, 4
#define MSNF    4, 4

    for (i = 0; i < encoded_size; i++)
    {
        E0 = READ_VAR_FIELD(datain[i], BITF0);
        E1 = READ_VAR_FIELD(datain[i], BITF1);
        E2 = READ_VAR_FIELD(datain[i], BITF2);
        E3 = READ_VAR_FIELD(datain[i], BITF3);
        E4 = READ_VAR_FIELD(datain[i], BITF4);
        E5 = READ_VAR_FIELD(datain[i], BITF5);
        E6 = READ_VAR_FIELD(datain[i], BITF6);
        E7 = READ_VAR_FIELD(datain[i], BITF7);

        if (i % 2)
        {//Decode higher nibble
            SET_VAR_FIELD(dataout[i/2], BITF4, ((E0 & (E1 ^ E4)) | (E0 & (E2 ^ E6)) | ((E1 ^ E4) & (E2 ^ E6))));
            SET_VAR_FIELD(dataout[i/2], BITF5, ((E1 & (E0 ^ E4)) | (E1 & (E3 ^ E7)) | ((E0 ^ E4) & (E3 ^ E7))));
            SET_VAR_FIELD(dataout[i/2], BITF6, ((E2 & (E0 ^ E6)) | (E2 & (E3 ^ E5)) | ((E0 ^ E6) & (E3 ^ E5))));
            SET_VAR_FIELD(dataout[i/2], BITF7, ((E3 & (E2 ^ E5)) | (E3 & (E1 ^ E7)) | ((E2 ^ E5) & (E1 ^ E7))));

            dst = MSN( dataout[i/2] );
        }
        else
        {//Decode lower nibble
            SET_VAR_FIELD(dataout[i/2], BITF0, ((E0 & (E1 ^ E4)) | (E0 & (E2 ^ E6)) | ((E1 ^ E4) & (E2 ^ E6))));
            SET_VAR_FIELD(dataout[i/2], BITF1, ((E1 & (E0 ^ E4)) | (E1 & (E3 ^ E7)) | ((E0 ^ E4) & (E3 ^ E7))));
            SET_VAR_FIELD(dataout[i/2], BITF2, ((E2 & (E0 ^ E6)) | (E2 & (E3 ^ E5)) | ((E0 ^ E6) & (E3 ^ E5))));
            SET_VAR_FIELD(dataout[i/2], BITF3, ((E3 & (E2 ^ E5)) | (E3 & (E1 ^ E7)) | ((E2 ^ E5) & (E1 ^ E7))));

            dst = LSN( dataout[i/2] );
        }


        /*-------------------------------------------------------------------------------------------------*/
        /* calculate the encoded value back from the decoded value and compare the original value for      */
        /* comparison                                                                                      */
        /*-------------------------------------------------------------------------------------------------*/
		/* Take decode byte*/
        src_re_calc = dst;

		/* calc its' parity */
        E0 = READ_VAR_FIELD(dst, BITF0);
        E1 = READ_VAR_FIELD(dst, BITF1);
        E2 = READ_VAR_FIELD(dst, BITF2);
        E3 = READ_VAR_FIELD(dst, BITF3);

        SET_VAR_FIELD(src_re_calc, BITF4, E0 ^ E1);
        SET_VAR_FIELD(src_re_calc, BITF5, E2 ^ E3);
        SET_VAR_FIELD(src_re_calc, BITF6, E0 ^ E2);
        SET_VAR_FIELD(src_re_calc, BITF7, E1 ^ E3);

        /*-----------------------------------------------------------------------------------------------------*/
        /* Check that only one bit is corrected per byte                                                       */
        /*-----------------------------------------------------------------------------------------------------*/
        BER = src_re_calc ^ datain[i];

        BER =    READ_VAR_FIELD(BER, BITF0)
               + READ_VAR_FIELD(BER, BITF1)
               + READ_VAR_FIELD(BER, BITF2)
               + READ_VAR_FIELD(BER, BITF3)
               + READ_VAR_FIELD(BER, BITF4)
               + READ_VAR_FIELD(BER, BITF5)
               + READ_VAR_FIELD(BER, BITF6)
               + READ_VAR_FIELD(BER, BITF7);

        /*-------------------------------------------------------------------------------------------------*/
        /* Bit Error Rate can be 0x00 (no change) or 0x01 0x02 0x04 0x08 -> one bit change only            */
        /*-------------------------------------------------------------------------------------------------*/
        if ( BER > 1 )
        {
            /*---------------------------------------------------------------------------------------------*/
            /* Use original nible :                                                                        */
            /*---------------------------------------------------------------------------------------------*/
            if (i % 2)
            { // copy lower nibble to higher nibble
                SET_VAR_FIELD(dataout[i/2], MSNF, LSN( datain[i] ));

            }
            else
            { // copy lower nibble to lower nibble
                SET_VAR_FIELD(dataout[i/2], LSNF, LSN( datain[i] ) );
            }

            status = HAL_ERROR_BAD_PARITY;
        }


    }

    return status;



}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_otp_NibParEccEncode                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  datain -       pointer to decoded data buffer (buffer size should be 2 x dataout)      */
/*                  dataout -      pointer to encoded data buffer                                          */
/*                  encoded_size - size of encoded data (decoded data x 2)                                 */
/*                                                                                                         */
/* Returns:         int                                                                             */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Decodes the data according to nibble parity ECC scheme.                                */
/*                  Size specifies the encoded data size.                                                  */
/*                  Decodes whole bytes only                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_NibParEccEncode (
    u8 *datain,
    u8 *dataout,
    u32 encoded_size
)
{
    u32 i;
    u8 E0;
    u8 E1;
    u8 E2;
    u8 E3;
    int status = 0;
    u32 decoded_size = encoded_size/2;

//Define the Bit Field macros in order to use the SET_VAR_FIELD macro:
#define BITF0   0, 1
#define BITF1   1, 1
#define BITF2   2, 1
#define BITF3   3, 1
#define BITF4   4, 1
#define BITF5   5, 1
#define BITF6   6, 1
#define BITF7   7, 1

#define LSNF    0, 4
#define MSNF    4, 4

    for (i = 0; i < decoded_size; i++)
    {
        dataout[i*2] = LSN(datain[i]);
        E0 = READ_VAR_FIELD(datain[i], BITF0);
        E1 = READ_VAR_FIELD(datain[i], BITF1);
        E2 = READ_VAR_FIELD(datain[i], BITF2);
        E3 = READ_VAR_FIELD(datain[i], BITF3);

        SET_VAR_FIELD(dataout[i*2], BITF4, E0 ^ E1);
        SET_VAR_FIELD(dataout[i*2], BITF5, E2 ^ E3);
        SET_VAR_FIELD(dataout[i*2], BITF6, E0 ^ E2);
        SET_VAR_FIELD(dataout[i*2], BITF7, E1 ^ E3);
        
        dataout[i*2+1] = MSN(datain[i]);
        E0 = READ_VAR_FIELD(datain[i], BITF4);
        E1 = READ_VAR_FIELD(datain[i], BITF5);
        E2 = READ_VAR_FIELD(datain[i], BITF6);
        E3 = READ_VAR_FIELD(datain[i], BITF7);

        SET_VAR_FIELD(dataout[i*2+1], BITF4, E0 ^ E1);
        SET_VAR_FIELD(dataout[i*2+1], BITF5, E2 ^ E3);
        SET_VAR_FIELD(dataout[i*2+1], BITF6, E0 ^ E2);
        SET_VAR_FIELD(dataout[i*2+1], BITF7, E1 ^ E3);
    }

    return status;
}




/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_otp_MajRulEccDecode                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  datain -       pointer to encoded data buffer (buffer size should be 3 x dataout)      */
/*                  dataout -      pointer to decoded data buffer                                          */
/*                  encoded_size - size of encoded data (decoded data x 3)                                 */
/*                                                                                                         */
/* Returns:         int                                                                             */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Decodes the data according to Major Rule ECC scheme.                                   */
/*                  Size specifies the encoded data size.                                                  */
/*                  Decodes whole bytes only                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_MajRulEccDecode (
    u8 *datain,
    u8 *dataout,
    u32 encoded_size
)
{
    unsigned int  byte;
    unsigned int  bit;
    u8 E1, E2, E3;
	u32 decoded_size;

    if (encoded_size % 3)
        // return DEFS_STATUS_INVALID_PARAMETER;
        encoded_size -= encoded_size % 3;
		
    decoded_size = encoded_size/3;

    for (byte = 0; byte < decoded_size; byte++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            E1 = READ_VAR_BIT(datain[decoded_size*0+byte], bit);
            E2 = READ_VAR_BIT(datain[decoded_size*1+byte], bit);
            E3 = READ_VAR_BIT(datain[decoded_size*2+byte], bit);
            if ((E1+E2+E3) >= 2)
            {
                SET_VAR_BIT(dataout[byte], bit);    //Majority is 1
            }
            else
            {
                 CLEAR_VAR_BIT(dataout[byte], bit); //Majority is 0
            }
        }//Inner for (bit)
    }//Outer for (byte)

    return 0;

}



/*---------------------------------------------------------------------------------------------------------*/
/* Function:        npcm750_otp_MajRulEccEncode                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  datain -       pointer to decoded data buffer (buffer size should be 3 x dataout)      */
/*                  dataout -      pointer to encoded data buffer                                          */
/*                  encoded_size - size of encoded data (decoded data x 3)                                 */
/*                                                                                                         */
/* Returns:         int                                                                             */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  Decodes the data according to Major Rule ECC scheme.                                   */
/*                  Size specifies the encoded data size.                                                  */
/*                  Decodes whole bytes only                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int NPCM750_OTP_MajRulEccEncode (
    u8 *datain,
    u8 *dataout,
    u32 encoded_size
)
{
    unsigned int  byte;
    unsigned int  bit;
    u8 bit_val;
	u32 decoded_size;

    if (encoded_size % 3)
        // return DEFS_STATUS_INVALID_PARAMETER;
        encoded_size -= encoded_size % 3;    
		
	decoded_size = encoded_size/3;

    for (byte = 0; byte < decoded_size; byte++)
    {
        for (bit = 0; bit < 8; bit++)
        {
            bit_val = READ_VAR_BIT(datain[byte], bit);

            if (bit_val == 1)
            {
                SET_VAR_BIT(dataout[decoded_size*0+byte], bit);
                SET_VAR_BIT(dataout[decoded_size*1+byte], bit);
                SET_VAR_BIT(dataout[decoded_size*2+byte], bit);

            }
            else
            {
                CLEAR_VAR_BIT(dataout[decoded_size*0+byte], bit);
                CLEAR_VAR_BIT(dataout[decoded_size*1+byte], bit);
                CLEAR_VAR_BIT(dataout[decoded_size*2+byte], bit);
            }
        } // Inner for (bit)
    }// Outer for (byte)

    return 0;

}
#endif // _CODE_THAT_SHOULD_BE_IN_USER_  


#ifdef CHECK_FUSE_IS_LOCKED_FOR_ACCESS
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_OTP_ProgramByteFustrap                                                                       */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  oFuse - fuse value to read                                                             */
/*                                                                                                         */
/* Returns:         retVal                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This is a getter for fustrap                                                           */
/*---------------------------------------------------------------------------------------------------------*/
static unsigned int           NPCM750_OTP_Fustrap_Get (NPCM750_OTP_FUSTRAP_FIELDS_T oFuse)
{
    unsigned int retVal = 0;
    switch (oFuse)
    {
        case NPCM750_OTP_FUSTRAP_DIS_FAST_BOOT:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_DIS_FAST_BOOT);
            break;

        case NPCM750_OTP_FUSTRAP_oWDEN:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oWDEN);
            break;

        case NPCM750_OTP_FUSTRAP_oHLTOF:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oHLTOF);
            break;

        case NPCM750_OTP_FUSTRAP_oAESKEYACCLK:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oAESKEYACCLK);
            break;

        case NPCM750_OTP_FUSTRAP_oJDIS:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oJDIS);
            break;

        case NPCM750_OTP_FUSTRAP_oSECBOOT:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oSECBOOT);
            break;

        case NPCM750_OTP_FUSTRAP_USEFUSTRAP:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_USEFUSTRAP);
            break;

        case NPCM750_OTP_FUSTRAP_oPKInvalid2_0:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oPKInvalid2_0);
            break;

        case NPCM750_OTP_FUSTRAP_oAltImgLoc:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oAltImgLoc);
            break;

        case NPCM750_OTP_FUSTRAP_Bit_28:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_BIT_28);
            break;

        case NPCM750_OTP_FUSTRAP_oSecBootDisable:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oSecBootDisable);
            break;

        case NPCM750_OTP_FUSTRAP_oCPU1STOP2:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oCPU1STOP2);
            break;

        case NPCM750_OTP_FUSTRAP_oCPU1STOP1:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oCPU1STOP1);
            break;

        case NPCM750_OTP_FUSTRAP_oHINDDIS:
            retVal = READ_REG_FIELD(FUSTRAP, FUSTRAP_oHINDDIS);
            break;
                

        default:
            ASSERT(false);
            break;
    }

    return retVal;
}
#endif //  CHECK_FUSE_IS_LOCKED_FOR_ACCESS	

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_PrintRegs                                                                         */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine prints the module registers                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void NPCM750_PrintRegs (void)
{
    unsigned int i;

    printk_dbg("/*--------------*/\n");
    printk_dbg("/*     FUSE     */\n");
    printk_dbg("/*--------------*/\n\n");

    for (i = 0; i <= NPCM750_FUSE_SA; i++)
    {
        NPCM750_PrintModuleRegs((NPCM750_OTP_STORAGE_ARRAY_T)i);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        NPCM750_PrintModuleRegs                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  array - The Storage Array type module to be printed.                                   */
/*                                                                                                         */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine prints the module instance registers                                      */
/*lint -e{715}      Supress 'array' not referenced                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void NPCM750_PrintModuleRegs (NPCM750_OTP_STORAGE_ARRAY_T array)
{
    ASSERT(array <= NPCM750_FUSE_SA);

    printk_dbg("FUSE%1X:\n", (array+1));
    printk_dbg("------\n");
    printk_dbg("FST%d                = 0x%08X\n", (array+1), REG_READ(FST(array)));
    printk_dbg("FADDR%d              = 0x%08X\n", (array+1), REG_READ(FADDR(array)));
    printk_dbg("FDATA%d              = 0x%08X\n", (array+1), REG_READ(FDATA(array)));
    printk_dbg("FCFG%d               = 0x%08X\n", (array+1), REG_READ(FCFG(array)));

    if (array == NPCM750_KEY_SA)
    {
        printk_dbg("FKEYIND             = 0x%08X\n", REG_READ(FKEYIND));
    }
    else
    {
        printk_dbg("FUSTRAP             = 0x%08X\n", REG_READ(FUSTRAP));
    }

    printk_dbg("\n");
}



/**
 *	npcm750_otp_seek - Seek on the OTP, size is fixed to 2KB. 
 */
 static loff_t npcm750_otp_seek(struct file *file, loff_t off, int whence)
{
	printk_dbg("\t otp: seek off=%lld, whence = %d\n", off, whence);
	if(off >= 2*NPCM750_OTP_ARR_BYTE_SIZE)
		return -EINVAL;
    return fixed_size_llseek(file, off, whence, 2*NPCM750_OTP_ARR_BYTE_SIZE);
}


/**
 *	npcm750_otp_read - Read OTP pages
 */
static ssize_t npcm750_otp_read(struct file *file, char __user *buff, size_t count, loff_t *pos)
{
	ssize_t                      bytes_read = 0;
	u8                           *read_buf;
	u32                          addr;
	NPCM750_OTP_STORAGE_ARRAY_T  array;
	
	if(*pos >= 2*NPCM750_OTP_ARR_BYTE_SIZE)
		return -EINVAL;

	array = NPCM750_KEY_SA;
	addr  = (u32)(*pos % NPCM750_OTP_ARR_BYTE_SIZE);
	
	if ( (*pos % (2*NPCM750_OTP_ARR_BYTE_SIZE))>= NPCM750_OTP_ARR_BYTE_SIZE)
	    array = NPCM750_FUSE_SA;
	
	printk_dbg("\n\t otp: read %d bytes from address 0x%x, array%d, *pos=%lld\n", count, addr, (int)array, *pos);

	if (((int)array*NPCM750_OTP_ARR_BYTE_SIZE + addr + count) > 2*NPCM750_OTP_ARR_BYTE_SIZE)
	    count = 2*NPCM750_OTP_ARR_BYTE_SIZE - addr - (int)array*NPCM750_OTP_ARR_BYTE_SIZE; // can only read up to 2KB

	mutex_lock(&npcm750_otp_lock);


	read_buf = kmalloc(count, GFP_KERNEL);
	if (!read_buf) {
		printk("\totp: kmalloc fail\n");
		return -ENOMEM;
	}
	

    while ( bytes_read < count)
    {
	    NPCM750_OTP_Read(array, addr, &read_buf[bytes_read]);
	    printk_dbg("\t otp: read  array%d addr %d 0x%02x \t(%d/%d), pos=%lld \n", array, addr, read_buf[bytes_read], bytes_read, count, *pos);
		bytes_read++;
		*pos += 1;
		addr++;
		
	    // check if we crossed to the next array:
	    if (addr >= NPCM750_OTP_ARR_BYTE_SIZE)
	    {
	       addr = 0;
	       if (array == NPCM750_KEY_SA) 
			    array = NPCM750_FUSE_SA;
			else  /* OTP is 2KB only */
				break;		
	    }
	}

	if (copy_to_user(buff, read_buf, bytes_read)) {
		printk("\t otp: copy_to_user failed\n");
		bytes_read = -EFAULT;
	}

	mutex_unlock(&npcm750_otp_lock);

	kfree(read_buf);

	return bytes_read;
}

#ifdef CONFIG_NPCM750_OTP_WRITE_ENABLE


/**
 *	npcm750_otp_write - write OTP pages
 *
 *	All writes must be in half page chunks (half page == 64 bits).
 */
static ssize_t npcm750_otp_write(struct file *filp, const char __user *buff, size_t count, loff_t *pos)
{

	ssize_t                      bytes_prog = 0;
	u8                           *prog_buf;
	u32                          addr;
	NPCM750_OTP_STORAGE_ARRAY_T  array;
	
	if(*pos >= 2*NPCM750_OTP_ARR_BYTE_SIZE)
		return -EINVAL;

	array = NPCM750_KEY_SA;
	addr  = (u32)(*pos % NPCM750_OTP_ARR_BYTE_SIZE);
	
	if ( (*pos % (2*NPCM750_OTP_ARR_BYTE_SIZE))>= NPCM750_OTP_ARR_BYTE_SIZE)
	    array = NPCM750_FUSE_SA;
	
	printk_dbg("\t otp: prog %d bytes from address 0x%x, array%d, pos=%lld\n", count, addr, (int)array, *pos);

	if (((int)array*NPCM750_OTP_ARR_BYTE_SIZE + addr + count) > 2*NPCM750_OTP_ARR_BYTE_SIZE)
	    count = 2*NPCM750_OTP_ARR_BYTE_SIZE - addr - (int)array*NPCM750_OTP_ARR_BYTE_SIZE; // can only read up to 2KB

	mutex_lock(&npcm750_otp_lock);

	prog_buf = kmalloc(count, GFP_KERNEL);
	if (!prog_buf) {
		printk("otp: kmalloc fail\n");
		return -ENOMEM;
	}
	
	
	if (copy_from_user(prog_buf, buff, count)) {
		printk("otp: copy_from_user failed\n");
		bytes_prog = -EFAULT;
	}

    while ( bytes_prog < count)
    {
	    NPCM750_OTP_ProgramByte(array, addr, prog_buf[bytes_prog]);
	    printk_dbg("\t otp: prog array%d, addr=%d 0x%02x  (%d/%d), pos=%lld, \n", array, addr, prog_buf[bytes_prog], bytes_prog, count, *pos );
		*pos += 1;
		addr++;
		bytes_prog++;
		
	    // check if we crossed to the next array:
	    if (addr >= NPCM750_OTP_ARR_BYTE_SIZE)
	    {
	       addr = 0;
	       if (array == NPCM750_KEY_SA) 
			    array = NPCM750_FUSE_SA;
			else  /* OTP is 2KB only */
				break;		
	    }
	}

	mutex_unlock(&npcm750_otp_lock);

	kfree(prog_buf);

	return bytes_prog;
	
}

#else
# define npcm750_otp_write NULL
#endif


long npcm750_otp_ioctl(struct file *filp, unsigned cmd, unsigned long arg)
{
	
	int ret = -ENOKEY;
#ifdef CHECK_AES_KEY_IS_FUSED_TO_NONE_ZERO_KEY	
	int ii  = 0;
	u8  data;
#endif

	printk_dbg("\tnpcm750_otp: ioctl %d, arg= %ld\n",cmd, arg);

	if ( arg > NPCM750_MAX_KEYS)
	{
	    pr_err("\tnpcm750_otp: ioctl %d, arg= %ld. arg invalid\n",cmd, arg);
        return -EFAULT;
	}
	
	switch (cmd) {
		case IOCTL_SELECT_AES_KEY: {

			// TODO: finalyze with Eugene key verification!
#ifdef CHECK_FUSE_IS_LOCKED_FOR_ACCESS			
		    if (NPCM750_OTP_Fustrap_Get(NPCM750_OTP_FUSTRAP_oAESKEYACCLK) == 0 )
		    {
                pr_err("\tERROR: npcm750_otp: keys are not locked, oAESKEYACCLK not fused\n");
                return -ENOKEY;
		    }
#endif	// CHECK_FUSE_IS_LOCKED_FOR_ACCESS	    
			if (mutex_lock_interruptible(&npcm750_otp_lock))
				return -ERESTARTSYS;

			/*--------------------------------------------------------------------*/
            /* 			check that the keys fused and locked before using         */
            /*--------------------------------------------------------------------*/
#ifdef CHECK_AES_KEY_IS_FUSED_TO_NONE_ZERO_KEY		
            /* Read the key and make sure it's not all zeros : */
            for ( ii = 0 ; ii < 64; ii++)
            {
                NPCM750_OTP_Read(NPCM750_KEY_SA, arg*64+ii, &data);
                if ( data != 0)
                {
                    break;
                }

            }
            if (ii == 64)
            {
                mutex_unlock(&npcm750_otp_lock);
                pr_err("\tERROR: npcm750_otp: ioctl %d, arg= %ld. Key is not fused. Key is all zeros.\n",cmd, arg);
                return -ENOKEY;
            }
#endif //  CHECK_AES_KEY_IS_FUSED_TO_NONE_ZERO_KEY	
			
			ret = NPCM750_OTP_SelectKey(arg);
			
			mutex_unlock(&npcm750_otp_lock);

			return ret;
		}

		case IOCTL_GET_AES_KEY_NUM: {
            return READ_REG_FIELD(FKEYIND, FKEYIND_KIND);
            break;
		}
		
		case IOCTL_DISABLE_KEY_ACCESS: {
		    if (mutex_lock_interruptible(&npcm750_otp_lock))
				return -ERESTARTSYS;
			
			NPCM750_OTP_DisableKeyAccess();

			mutex_unlock(&npcm750_otp_lock);

			return 0;			
		}
	}

    return -EINVAL;
}
EXPORT_SYMBOL(npcm750_otp_ioctl);

////////////////////// driver init /////////////////////////////////////

static const struct file_operations npcm750_otp_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = npcm750_otp_ioctl,
	.read           = npcm750_otp_read,
#if CONFIG_NPCM750_OTP_WRITE_ENABLE	
	.write          = npcm750_otp_write,
#endif	
	.llseek		    = npcm750_otp_seek,
	// .open           = npcm750_otp_open,
};

static struct miscdevice npcm750_otp_misc_device = {
	.minor    = NVRAM_MINOR,   //    MISC_DYNAMIC_MINOR,
	.name     = DRIVER_NAME,
	.fops     = &npcm750_otp_fops,
};

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750_otp");

static const struct of_device_id otp_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-otp",  },
	{},
};
MODULE_DEVICE_TABLE(of, otp_dt_id);

static struct platform_driver npcm750_otp_driver = {
	.probe		= npcm750_otp_probe,
	.remove		= npcmx50_otp_remove,
	.shutdown	= NULL,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(otp_dt_id),
	}
};


static int npcm750_otp_probe(struct platform_device *pdev)
{
	struct resource *res[2] = {NULL, NULL};
	int ret;
	int resource_count = 0;

	printk_dbg(KERN_INFO "\t\t\t* OTP probe start\n");
	/*
	 * A bit ugly, and it will never actually happen but there can
	 * be only one RNG and this catches any bork
	 */
	if (otp_dev)
	{
	    pr_err("\t\t\t* OTP probe failed: can't instantiate more then one device.\n");
		return -EBUSY;
	}

#ifdef CONFIG_OF
	otp_clk = devm_clk_get(&pdev->dev, NULL);
    
    if (IS_ERR(otp_clk))	
    {
        pr_err(" OTP probe failed: can't read clk.\n");			
        ret =  -EPROBE_DEFER; // this error will cause the probing to run again after clk is ready.
        goto err_ioremap;
    }

    //clk_prepare_enable(otp_clk);    

    printk_dbg("\tOTP clock is %ld\n" , clk_get_rate(otp_clk)); 
#endif //  CONFIG_OF


	for (resource_count = 0; resource_count < 2 ; resource_count++)
	{
		res[resource_count] = platform_get_resource(pdev, IORESOURCE_MEM, resource_count);
		if (!res[resource_count]) {
		    pr_err(" OTP probe failed: can't read resource %d.\n", resource_count);
			ret = -ENOENT;
			goto err_ioremap;
		}

		if (!request_mem_region(res[resource_count]->start, resource_size(res[resource_count]), pdev->name)) {
		    pr_err(" OTP probe failed: can't request_mem_region for resource %d.\n", resource_count);
			ret = -EBUSY;
			goto err_ioremap;
		}

		dev_set_drvdata(&pdev->dev, res[resource_count]);
		otp_base[resource_count] = ioremap(res[resource_count]->start, resource_size(res[resource_count]));
		printk(KERN_INFO "\t\t\t* OTP%d (FUZE) base is 0x%08X, size 0x%08X, phys 0x%08X \n",
		    resource_count, (u32)otp_base[resource_count], resource_size(res[resource_count]), res[resource_count]->start);
		if (!otp_base[resource_count]) {
		    pr_err(" OTP probe failed: can't read otp base address for resource %d.\n", resource_count);			
			ret = -ENOMEM;
			goto err_ioremap;
		}
	}
	
	otp_dev = pdev;
    
	NPCM750_PrintRegs();

	printk(KERN_INFO "NPCMx50: OTP module probe OK\n");

	return 0;

err_ioremap:
	if (otp_base[0] != NULL)
	{
		iounmap(otp_base[0]);
		otp_base[0] = NULL;
	}
	if (otp_base[1] != NULL)
	{
		iounmap(otp_base[1]);
		otp_base[1] = NULL;
	}
	otp_dev = NULL;
	// release resources already allocated.
    for (;resource_count > 0 ; resource_count--)
	{
	    release_mem_region(res[resource_count]->start, resource_size(res[resource_count]));
	}
//err_region:
    printk(KERN_INFO "NPCMx50: OTP module load fail .  Error %d\n", ret);

	return ret;
}

static int __exit npcmx50_otp_remove(struct platform_device *pdev)
{
	struct resource *res = dev_get_drvdata(&pdev->dev);

	printk(KERN_NOTICE  "\t\t\t*npcmX50-OTP: remove: stop using Nuvoton npcmx50 OTP.\n");

	if (otp_base[0] != NULL)
	{
		iounmap(otp_base[0]);
		otp_base[0] = NULL;
	}
	if (otp_base[1] != NULL)
	{
		iounmap(otp_base[1]);
		otp_base[1] = NULL;
	}
	otp_dev = NULL;

	release_mem_region(res->start, resource_size(res));
	return 0;
}



//static dev_t npcm750_otp_dev;             // (major,minor) value

/**
 *	npcm750_otp_init - Initialize module
 *
 *	Registers the device and notifier handler. Actual device
 *	initialization is handled by npcm750_otp_open().
 */
static int __init npcm750_otp_init(void)
{
	int ret;
	
	/*
	 * A bit ugly, and it will never actually happen but there can
	 * be only one FUSE and this catches any bork
	 */
	//if (otp_dev)
	//	return -EBUSY;
	
	ret = platform_driver_register(&npcm750_otp_driver);
	if (ret) {
		printk("otp: unable to register a platform driver\n");
		goto err_region;
	}	
	ret = misc_register(&npcm750_otp_misc_device);
	if (ret) {
		printk("otp: unable to register a misc device\n");
		goto err_region;
	}

    printk("NPCMx50: OTP (FUZE) module is ready\n");
	
	return 0;


err_region:
    printk(KERN_INFO "NPCMx50: OTP module load fail .  Error %d\n", ret);

	return ret;
}

/**
 *	npcm750_otp_exit - Deinitialize module
 *
 *	Unregisters the device and notifier handler. Actual device
 *	deinitialization is handled by npcm750_otp_close().
 */
static void __exit npcm750_otp_exit(void)
{
    otp_dev = NULL;
    otp_base[0] = 0;
    otp_base[1] = 0;
	misc_deregister(&npcm750_otp_misc_device);
	platform_driver_unregister(&npcm750_otp_driver);
}

module_init(npcm750_otp_init);
module_exit(npcm750_otp_exit);


MODULE_AUTHOR("Tali Perry <tali.perry@nuvoton.com>");
MODULE_DESCRIPTION("NPCM750 OTP Memory Interface");
MODULE_LICENSE("GPL");


#endif//  CONFIG_NPCM750_OTP

