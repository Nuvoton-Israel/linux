/*
 * Cryptographic API.
 *
 * Support for Nuvoton SHA HW acceleration.
 *
 * Copyright (c) 2016 Nuvoton Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Based on 
 */

#include <crypto/internal/hash.h>
#include <crypto/sha.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/mutex.h>



//#include <asm/cpu_device_id.h>

#include <asm/io.h>

//#include <mach/map.h>
//#include <mach/hal.h>
//#include <defs.h>
#include "npcm750_sha.h"



//#ifndef NO_LIBC
#include <linux/string.h>
//#endif

//#define SHA_DEBUG_MODULE

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
/* calculate minimum value                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define MIN(a, b)               ((a)<(b) ? (a) : (b))

#define DEFS_STATUS_RET_CHECK(func)                                 \
{                                                                   \
    int ret;                                                \
                                                                    \
    if ((ret = func) != 0x0)                                        \
    {                                                               \
       return ret;                                                  \
    }                                                               \
}

#define DEFS_STATUS_COND_CHECK(cond, err)                           \
{                                                                   \
    if (!(cond))                                                    \
    {                                                               \
        return err;                                                 \
    }                                                               \
}


#ifdef SHA_DEBUG_MODULE
#define sha_print(fmt,args...)      printk(fmt ,##args)
#else
#define sha_print(fmt,args...)     (void)0
#endif


static void sha_print_hex_dump(char *note, unsigned char *buf, unsigned int len)
{
#ifndef SHA_DEBUG_MODULE
        return;
#else
		sha_print(KERN_CRIT "%s", note);
		print_hex_dump(KERN_CONT, "\t\t\t\t\t", DUMP_PREFIX_OFFSET,
				16, 1,
				buf, len, false);
#endif				
}


static DEFINE_MUTEX(npcmx50_sha_lock);


/*---------------------------------------------------------------------------------------------------------*/
/* Busy Wait with Timeout                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define BUSY_WAIT_TIMEOUT(busy_cond, timeout)       \
{                                                   \
    u32 __time = timeout;                        \
                                                    \
    do                                              \
    {                                               \
        if (__time-- == 0)                          \
        {                                           \
            return -ETIMEDOUT;                      \
        }                                           \
    } while (busy_cond);                            \
}



/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                                  TYPES & DEFINITIONS                                    */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

#define SHA_BLOCK_LENGTH     (512/8)
#define SHA_2_HASH_LENGTH    (256/8)
#define SHA_1_HASH_LENGTH    (160/8)

#define NUVOTON_ALIGNMENT 4


/*---------------------------------------------------------------------------------------------------------*/
/* SHA type                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
  SHA_TYPE_SHA2 = 0,/*do not change - match SHA arch spec */
  SHA_TYPE_SHA1,
  SHA_TYPE_NUM
} SHA_TYPE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* SHA instance struct handler                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct SHA_HANDLE_T
{
  u32                 hv[SHA_2_HASH_LENGTH / sizeof(u32)];
  u32                 length0;
  u32                 length1;
  u32                 block[SHA_BLOCK_LENGTH / sizeof(u32)];
  SHA_TYPE_T          shaType;
  bool             active;
} SHA_HANDLE_T;


/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           INTERFACE FUNCTIONS                                           */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

#ifdef SHA_CALC_IN_A_SINGLE_COMMAND
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Calc                                                                               */
/*                                                                                                         */
/* Parameters:      shaType     - SHA module type                                                          */
/*                  inBuff      -    Pointer to a buffer containing the data to be hashed                  */
/*                  len         -    Length of the data to hash                                            */
/*                  hashDigest  -   Pointer to a buffer where the reseulting digest will be copied to      */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs complete SHA calculation in one step including SHA_Init routine  */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_Calc (SHA_TYPE_T shaType, const u8* inBuff, u32 len, u8* hashDigest);
#endif // SHA_CALC_IN_A_SINGLE_COMMAND

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Init                                                                               */
/*                                                                                                         */
/* Parameters:      handlePtr - SHA processing handle pointer                                              */
/* Returns:         0 on success or other int error code on error.                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine initialize the SHA module                                                 */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_Init (SHA_HANDLE_T* handlePtr);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Start                                                                              */
/*                                                                                                         */
/* Parameters:      handlePtr   - SHA processing handle pointer                                            */
/*                  shaType     - SHA module type                                                          */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error.                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine start a single SHA process                                                */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_Start (SHA_HANDLE_T* handlePtr, SHA_TYPE_T shaType);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Update                                                                             */
/*                                                                                                         */
/* Parameters:      handlePtr  -   SHA processing handle pointer                                           */
/*                  buffer -   Pointer to the data that will be added to the hash calculation              */
/*                  len -      Length of data to add to SHA calculation                                    */
/*                                                                                                         */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine adds data to previously started SHA calculation                           */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_Update (SHA_HANDLE_T* handlePtr, const u8* buffer, u32 len);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Finish                                                                             */
/*                                                                                                         */
/* Parameters:      handlePtr  -   SHA processing handle pointer                                           */
/*                  hashDigest -     Pointer to a buffer where the final digest will be copied to          */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine finish SHA calculation and get the resulting SHA digest                   */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Finish (SHA_HANDLE_T* handlePtr, u8* hashDigest);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Reset                                                                              */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine reset SHA module                                                          */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_Reset (void);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Power                                                                              */
/*                                                                                                         */
/* Parameters:      on - true enable the module, false disable the module                                  */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine set SHA module power on/off                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_Power (bool on);

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_PrintRegs                                                                          */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine prints the module registers                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void SHA_PrintRegs (void);

#ifdef SHA_SELF_TEST
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_SelfTest                                                                           */
/*                                                                                                         */
/* Parameters:      shaType - SHA module type                                                              */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs various tests on the SHA HW and SW                               */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_SelfTest (SHA_TYPE_T shaType);
#endif//#ifdef SHA_SELF_TEST





/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           TYPES & DEFINITIONS                                           */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

#define SHA_SECRUN_BUFF_SIZE            64
#define SHA_TIMEOUT                     100
#define SHA_DATA_LAST_BYTE              0x80
#define SHA_SET_TYPE(type)              SET_REG_FIELD(HASH_CFG, HASH_CFG_SHA1_SHA2, type)
#ifdef SHA_SELF_TEST
#define SHA2_NUM_OF_SELF_TESTS          3
#define SHA1_NUM_OF_SELF_TESTS          4
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           SHA_BUFF_POS                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  length - number of bytes written                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine compute the # of bytes currently in the sha block buffer                  */
/*---------------------------------------------------------------------------------------------------------*/
#define SHA_BUFF_POS(length)            (length & (SHA_BLOCK_LENGTH - 1))

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           SHA_BUFF_FREE                                                                          */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  length - number of bytes written                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine compute the # of free bytes in the sha block buffer                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SHA_BUFF_FREE(length)           (SHA_BLOCK_LENGTH - SHA_BUFF_POS(length))

/*---------------------------------------------------------------------------------------------------------*/
/* Macro:           SHA_RET_CHECK                                                                          */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  func    - function to check                                                            */
/*                                                                                                         */
/* Returns:                                                                                                */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This macro checks if give function returns int error, and returns the error            */
/*                  immediately after SHA disabling                                                        */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define  SHA_RET_CHECK(func)                                        \
{                                                                   \
    int status;                                                     \
                                                                    \
    if ((status = func) != 0)                                       \
    {                                                               \
       DEFS_STATUS_RET_CHECK(SHA_Power(false));                     \
            if (mutex_is_locked(&npcmx50_sha_lock))                 \
                mutex_unlock(&npcmx50_sha_lock);                    \
       return status;                                               \
    }                                                               \
}

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                       LOCAL FUNCTIONS DECLARATION                                       */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

static void                             SHA_FlushLocalBuffer_l (const u32* buff);
static int                              SHA_BusyWait_l(void);
static void                             SHA_GetShaDigest_l(u8* hashDigest, SHA_TYPE_T shaType);
static void                             SHA_SetShaDigest_l(const u32* hashDigest, SHA_TYPE_T shaType);
static void                             SHA_SetBlock_l(const u8* data,u32 len, u16 position, u32* block);
static void                             SHA_ClearBlock_l(u16 len, u16 position, u32* block);
static void                             SHA_SetLength32_l(const SHA_HANDLE_T* handlePtr, u32* block);



/* SHA MODULE registers */
static void __iomem *sha_base;

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           INTERFACE FUNCTIONS                                           */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

#ifdef SHA_CALC_IN_A_SINGLE_COMMAND
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Calc                                                                               */
/*                                                                                                         */
/* Parameters:      shaType - SHA module type                                                              */
/*                  inBuff -    Pointer to a buffer containing the data to be hashed                       */
/*                  len -    Length of the data to hash                                                    */
/*                  hashDigest -   Pointer to a buffer where the reseulting digest will be copied to       */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs complete SHA calculation in one step                             */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Calc (
    SHA_TYPE_T shaType,
    const u8* inBuff,
    u32 len,
    u8* hashDigest
)
{
    SHA_HANDLE_T handle;

    SHA_Init(&handle);
    SHA_Power(true);
    SHA_Reset();
    SHA_Start(&handle, shaType);
    SHA_RET_CHECK(SHA_Update(&handle, inBuff, len));
    SHA_RET_CHECK(SHA_Finish(&handle, hashDigest));
    SHA_Power(false);
    return 0;
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Init                                                                               */
/*                                                                                                         */
/* Parameters:      handlePtr - SHA processing handle pointer                                              */
/* Returns:         0 on success or other int error code on error.                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine initialize the SHA module                                                 */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Init (SHA_HANDLE_T* handlePtr)
{
    handlePtr->active = false;

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Start                                                                              */
/*                                                                                                         */
/* Parameters:      handlePtr   - SHA processing handle pointer                                            */
/*                  shaType     - SHA module type                                                          */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error.                                         */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine start a single SHA process                                                */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Start (
    SHA_HANDLE_T* handlePtr,
    SHA_TYPE_T shaType
)
{
    /*-----------------------------------------------------------------------------------------------------*/
    /* Initialize handle                                                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    handlePtr->length0 = 0;
    handlePtr->length1 = 0;
    handlePtr->shaType = shaType;
    handlePtr->active = true;

    mutex_lock(&npcmx50_sha_lock);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set SHA type                                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SET_TYPE(handlePtr->shaType);
	
    /*-----------------------------------------------------------------------------------------------------*/
    /* Reset SHA hardware                                                                                  */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_Reset();

    /*-----------------------------------------------------------------------------------------------------*/
    /* The handlePtr->hv is initialized with the correct IV as the SHA engine automaticly                  */
    /* fill the HASH_DIG_Hn registers according to SHA spec (following SHA_RST assertion)                  */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_GetShaDigest_l((u8*)handlePtr->hv, shaType);

    if (mutex_is_locked(&npcmx50_sha_lock)) 
		mutex_unlock(&npcmx50_sha_lock);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Init block with zeros                                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    memset(handlePtr->block, 0, sizeof(handlePtr->block));

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Update                                                                             */
/*                                                                                                         */
/* Parameters:      handlePtr  -   SHA processing handle pointer                                           */
/*                  buffer -   Pointer to the data that will be added to the hash calculation              */
/*                  len -      Length of data to add to SHA calculation                                    */
/*                                                                                                         */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine adds data to previously started SHA calculation                           */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Update (
    SHA_HANDLE_T* handlePtr,
    const u8* buffer,
    u32 len
)
{
    u32 localBuffer[SHA_SECRUN_BUFF_SIZE / sizeof(u32)];
    u32 bufferLen          = len;
    u16 pos                = 0;
    u8* blockPtr;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Error check                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    DEFS_STATUS_COND_CHECK(handlePtr->active == true, -(EPROTO));

    sha_print("\t\t\t\t* SHA_Update buffer 0x%08x, len %d\n", buffer, len);

    sha_print_hex_dump("\t\t\t\t\t*npcmX50-SHA: SHA_Update\n",(unsigned char *)buffer, len);

    mutex_lock(&npcmx50_sha_lock);
	

    /*-----------------------------------------------------------------------------------------------------*/
    /* Wait till SHA is not busy                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_RET_CHECK(SHA_BusyWait_l());

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set SHA type                                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SET_TYPE(handlePtr->shaType);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Write SHA latest digest into SHA module                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SetShaDigest_l(handlePtr->hv, handlePtr->shaType);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set number of unhashed bytes which remained from last update                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    pos = SHA_BUFF_POS(handlePtr->length0);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Copy unhashed bytes which remained from last update to secrun buffer                                */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SetBlock_l((u8*)handlePtr->block, pos, 0, localBuffer);

    while (len)
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* Wait for the hardware to be available (in case we are hashing)                                  */
        /*-------------------------------------------------------------------------------------------------*/
        SHA_RET_CHECK(SHA_BusyWait_l());

        /*-------------------------------------------------------------------------------------------------*/
        /* Move as much bytes  as we can into the secrun buffer                                            */
        /*-------------------------------------------------------------------------------------------------*/
        bufferLen = MIN(len, SHA_BUFF_FREE(handlePtr->length0));

        /*-------------------------------------------------------------------------------------------------*/
        /* Copy current given buffer to the secrun buffer                                                  */
        /*-------------------------------------------------------------------------------------------------*/
        SHA_SetBlock_l((u8*)buffer, bufferLen, pos, localBuffer);

        /*-------------------------------------------------------------------------------------------------*/
        /* Update size of hashed bytes                                                                     */
        /*-------------------------------------------------------------------------------------------------*/
        handlePtr->length0 += bufferLen;

        if ((handlePtr->length0) < bufferLen)
        {
            handlePtr->length1++;
        }

        /*-------------------------------------------------------------------------------------------------*/
        /* Update length of data left to digest                                                            */
        /*-------------------------------------------------------------------------------------------------*/
        len -= bufferLen;

        /*-------------------------------------------------------------------------------------------------*/
        /* Update given buffer pointer                                                                     */
        /*-------------------------------------------------------------------------------------------------*/
        buffer += bufferLen;

        /*-------------------------------------------------------------------------------------------------*/
        /* If secrun buffer is full                                                                        */
        /*-------------------------------------------------------------------------------------------------*/
        if (SHA_BUFF_POS(handlePtr->length0) == 0)
        {
            /*---------------------------------------------------------------------------------------------*/
            /* We just filled up the buffer perfectly, so let it hash                                      */
            /* (we'll unload the hash only when we are done with all hashing)                              */
            /*---------------------------------------------------------------------------------------------*/
            SHA_FlushLocalBuffer_l(localBuffer);

            pos = 0;
            bufferLen = 0;
        }
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Wait till SHA is not busy                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_RET_CHECK(SHA_BusyWait_l());

    /*-----------------------------------------------------------------------------------------------------*/
    /* Copy unhashed bytes from given buffer to handle block for next update/finish                        */
    /*-----------------------------------------------------------------------------------------------------*/
    blockPtr = (u8*)handlePtr->block;
    while (bufferLen)
    {
        blockPtr[--bufferLen+pos] = *(--buffer);
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Save SHA current digest                                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_GetShaDigest_l((u8*)handlePtr->hv, handlePtr->shaType);

    if (mutex_is_locked(&npcmx50_sha_lock)) 
		mutex_unlock(&npcmx50_sha_lock);
    

    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Finish                                                                             */
/*                                                                                                         */
/* Parameters:      handlePtr  -   SHA processing handle pointer                                           */
/*                  hashDigest -     Pointer to a buffer where the final digest will be copied to          */
/*                                                                                                         */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine finish SHA calculation and get the resulting SHA digest                   */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Finish (
    SHA_HANDLE_T* handlePtr,
    u8* hashDigest

)
{
    u32 localBuffer[SHA_SECRUN_BUFF_SIZE / sizeof(u32)];
    const u8 lastbyte = SHA_DATA_LAST_BYTE;
    u16 pos;

    /*-----------------------------------------------------------------------------------------------------*/
    /* Error check                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    DEFS_STATUS_COND_CHECK(handlePtr->active == true, -(EPROTO));
	

    mutex_lock(&npcmx50_sha_lock);


    /*-----------------------------------------------------------------------------------------------------*/
    /* Set SHA type                                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SET_TYPE(handlePtr->shaType);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Wait till SHA is not busy                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_RET_CHECK(SHA_BusyWait_l());

    /*-----------------------------------------------------------------------------------------------------*/
    /* Finish off the current buffer with the SHA spec'ed padding                                          */
    /*-----------------------------------------------------------------------------------------------------*/
    pos = SHA_BUFF_POS(handlePtr->length0);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Init SHA digest                                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SetShaDigest_l(handlePtr->hv, handlePtr->shaType);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Load data into secrun buffer                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SetBlock_l((u8*)handlePtr->block, pos, 0, localBuffer);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Set data last byte as in SHA algorithm spec                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SetBlock_l(&lastbyte, 1, pos++, localBuffer);

    /*-----------------------------------------------------------------------------------------------------*/
    /* If the remainder of data is longer then one block                                                   */
    /*-----------------------------------------------------------------------------------------------------*/
    if (pos > (SHA_BLOCK_LENGTH - 8))
    {
        /*-------------------------------------------------------------------------------------------------*/
        /* The message length will be in the next block                                                    */
        /* Pad the rest of the last block with 0's                                                         */
        /*-------------------------------------------------------------------------------------------------*/
        SHA_ClearBlock_l((SHA_BLOCK_LENGTH - pos), pos, localBuffer);

        /*-------------------------------------------------------------------------------------------------*/
        /* Hash the current block                                                                          */
        /*-------------------------------------------------------------------------------------------------*/
        SHA_FlushLocalBuffer_l(localBuffer);

        pos = 0;

        /*-------------------------------------------------------------------------------------------------*/
        /* Wait till SHA is not busy                                                                       */
        /*-------------------------------------------------------------------------------------------------*/
        SHA_RET_CHECK(SHA_BusyWait_l());
    }

    /*-----------------------------------------------------------------------------------------------------*/
    /* Pad the rest of the last block with 0's except for the last 8-3 bytes                               */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_ClearBlock_l((SHA_BLOCK_LENGTH-(8-3))-pos, pos, localBuffer);

    /*-----------------------------------------------------------------------------------------------------*/
    /* The last 8-3 bytes are set to the bit-length of the message in big-endian form                      */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_SetLength32_l(handlePtr, localBuffer);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Hash all that, and save the hash for the caller                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_FlushLocalBuffer_l(localBuffer);

    /*-----------------------------------------------------------------------------------------------------*/
    /* Wait till SHA is not busy                                                                           */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_RET_CHECK(SHA_BusyWait_l());

    /*-----------------------------------------------------------------------------------------------------*/
    /* Save SHA final digest into given buffer                                                             */
    /*-----------------------------------------------------------------------------------------------------*/
    SHA_GetShaDigest_l(hashDigest, handlePtr->shaType);

    sha_print_hex_dump("\t\t\t*npcmX50-SHA: SHA_Finish\n", hashDigest,
        (((handlePtr->shaType == SHA_TYPE_SHA2) ? SHA_2_HASH_LENGTH : SHA_1_HASH_LENGTH)));
 
    if (mutex_is_locked(&npcmx50_sha_lock)) 
		mutex_unlock(&npcmx50_sha_lock);
		
    /*-----------------------------------------------------------------------------------------------------*/
    /* Free handle                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    handlePtr->active = false;

    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Reset                                                                              */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine reset SHA module                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Reset (void)
{
    SET_REG_FIELD(HASH_CTR_STS, HASH_CTR_STS_SHA_RST, 0x01);

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_Power                                                                              */
/*                                                                                                         */
/* Parameters:      on - true enable the module, false disable the module                                  */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine set SHA module power on/off                                               */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_Power (bool on)
{
    SET_REG_FIELD(HASH_CTR_STS, HASH_CTR_STS_SHA_EN, on);

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_PrintRegs                                                                          */
/*                                                                                                         */
/* Parameters:      none                                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine prints the module registers                                               */
/*---------------------------------------------------------------------------------------------------------*/
void SHA_PrintRegs (void)
{
    unsigned int i;

    sha_print("/*--------------*/\n");
    sha_print("/*     SHA      */\n");
    sha_print("/*--------------*/\n\n");

    sha_print("HASH_CTR_STS        = 0x%02X\n", REG_READ(HASH_CTR_STS));
    sha_print("HASH_CFG            = 0x%02X\n", REG_READ(HASH_CFG));

    for (i = 0; i < HASH_DIG_H_NUM; i++)
    {
        sha_print("HASH_DIG_H%d         = 0x%08X\n", i, REG_READ(HASH_DIG_H(i)));
    }
	
	sha_print("VER         = 0x%08X\n", REG_READ(HASH_VER));

    sha_print("\n");
}

#ifdef SHA_SELF_TEST
/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_SelfTest                                                                           */
/*                                                                                                         */
/* Parameters:      shaType - SHA module type                                                              */
/* Returns:         0 on success or other int error code on error                                          */
/* Side effects:                                                                                           */
/* Description:                                                                                            */
/*                  This routine performs various tests on the SHA HW and SW                               */
/*---------------------------------------------------------------------------------------------------------*/
int SHA_SelfTest (SHA_TYPE_T shaType)
{
    SHA_HANDLE_T handle;
    u8 hashDigest[MAX(SHA_1_HASH_LENGTH, SHA_2_HASH_LENGTH)];
    u8 i;
    u16 j;

    /*-----------------------------------------------------------------------------------------------------*/
    /* SHA1 tests info                                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    static const u8 sha1SelfTestBuff[SHA1_NUM_OF_SELF_TESTS][94] =
    {
        {"abc"},
        {"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"},
        {"0123456789012345678901234567890123456789012345678901234567890123"},
        {0x30, 0x5c, 0x30, 0x2c, 0x02, 0x01, 0x00, 0x30, 0x09, 0x06, 0x05, 0x2b,
         0x0e, 0x03, 0x02, 0x1a, 0x05, 0x00, 0x30, 0x06, 0x06, 0x04, 0x67, 0x2a,
         0x01, 0x0c, 0x04, 0x14, 0xe1, 0xb6, 0x93, 0xfe, 0x33, 0x43, 0xc1, 0x20,
         0x5d, 0x4b, 0xaa, 0xb8, 0x63, 0xfb, 0xcf, 0x6c, 0x46, 0x1e, 0x88, 0x04,
         0x30, 0x2c, 0x02, 0x01, 0x00, 0x30, 0x09, 0x06, 0x05, 0x2b, 0x0e, 0x03,
         0x02, 0x1a, 0x05, 0x00, 0x30, 0x06, 0x06, 0x04, 0x67, 0x2a, 0x01, 0x0c,
         0x04, 0x14, 0x13, 0xc1, 0x0c, 0xfc, 0xc8, 0x92, 0xd7, 0xde, 0x07, 0x1c,
         0x40, 0xde, 0x4f, 0xcd, 0x07, 0x5b, 0x68, 0x20, 0x5a, 0x6c}
    };

    static const u8 sha1SelfTestBuffLen[SHA1_NUM_OF_SELF_TESTS] =
    {
        3, 56, 64, 94
    };
    static const u8 sha1SelfTestExpRes[SHA1_NUM_OF_SELF_TESTS][SHA_1_HASH_LENGTH] =
    {
        {0xA9, 0x99, 0x3E, 0x36,
         0x47, 0x06, 0x81, 0x6A,
         0xBA, 0x3E, 0x25, 0x71,
         0x78, 0x50, 0xC2, 0x6C,
         0x9C, 0xD0, 0xD8, 0x9D},
        {0x84, 0x98, 0x3E, 0x44,
         0x1C, 0x3B, 0xD2, 0x6E,
         0xBA, 0xAE, 0x4A, 0xA1,
         0xF9, 0x51, 0x29, 0xE5,
         0xE5, 0x46, 0x70, 0xF1},
        {0xCF, 0x08, 0x00, 0xF7,
         0x64, 0x4A, 0xCE, 0x3C,
         0xB4, 0xC3, 0xFA, 0x33,
         0x38, 0x8D, 0x3B, 0xA0,
         0xEA, 0x3C, 0x8B, 0x6E},
        {0xc9, 0x84, 0x45, 0xc8,
         0x64, 0x04, 0xb1, 0xe3,
         0x3c, 0x6b, 0x0a, 0x8c,
         0x8b, 0x80, 0x94, 0xfc,
         0xf3, 0xc9, 0x98, 0xab}
    };

    /*-----------------------------------------------------------------------------------------------------*/
    /* SHA2 tests info                                                                                     */
    /*-----------------------------------------------------------------------------------------------------*/
    static const u8 sha2SelfTestBuff[SHA2_NUM_OF_SELF_TESTS][100] =
    {
        { "abc" },
        { "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" },
        {'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
         'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'}
    };

    static const u8 sha2SelfTestBuffLen[SHA2_NUM_OF_SELF_TESTS] =
    {
        3, 56, 100
    };

    static const u8 sha2SelfTestExpRes[SHA2_NUM_OF_SELF_TESTS][SHA_2_HASH_LENGTH] =
    {
        /*
         * SHA-256 test vectors
         */
        { 0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA,
          0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
          0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C,
          0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD },
        { 0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8,
          0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39,
          0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
          0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1 },
        { 0xCD, 0xC7, 0x6E, 0x5C, 0x99, 0x14, 0xFB, 0x92,
          0x81, 0xA1, 0xC7, 0xE2, 0x84, 0xD7, 0x3E, 0x67,
          0xF1, 0x80, 0x9A, 0x48, 0xA4, 0x97, 0x20, 0x0E,
          0x04, 0x6D, 0x39, 0xCC, 0xC7, 0x11, 0x2C, 0xD0 }
    };

    /*-----------------------------------------------------------------------------------------------------*/
    /* SHA 1 TESTS                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    if (shaType == SHA_TYPE_SHA1)
    {
        for(i = 0; i < SHA1_NUM_OF_SELF_TESTS; i++)
        {
            if (i != 3)
            {
                SHA_RET_CHECK(SHA_Calc(SHA_TYPE_SHA1, sha1SelfTestBuff[i], sha1SelfTestBuffLen[i], hashDigest));
            }
            else
            {
                SHA_Power(true);
                SHA_Reset();
                SHA_RET_CHECK(SHA_Start(&handle, SHA_TYPE_SHA1));
                SHA_RET_CHECK(SHA_Update(&handle, sha1SelfTestBuff[i],73));
                SHA_RET_CHECK(SHA_Update(&handle, &(sha1SelfTestBuff[i][73]),sha1SelfTestBuffLen[i] - 73));
                SHA_RET_CHECK(SHA_Finish(&handle, hashDigest));
                SHA_Power(false);
            }
            if (memcmp(hashDigest, sha1SelfTestExpRes[i], SHA_1_HASH_LENGTH))
            {
                return DEFS_STATUS_FAIL;
            }
        }

    }
    /*-----------------------------------------------------------------------------------------------------*/
    /* SHA 2 TESTS                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    else
    {
        for( i = 0; i < SHA2_NUM_OF_SELF_TESTS; i++ )
        {
            SHA_Power(true);
            SHA_Reset();
            SHA_RET_CHECK(SHA_Start(&handle, SHA_TYPE_SHA2));
            if( i == 2 )
            {
                for( j = 0; j < 10000; j++ )//not working
                {
                    SHA_RET_CHECK(SHA_Update(&handle, sha2SelfTestBuff[i], sha2SelfTestBuffLen[i]));
                }
            }
            else
            {
                SHA_RET_CHECK(SHA_Update(&handle, sha2SelfTestBuff[i], sha2SelfTestBuffLen[i]));
            }

            SHA_RET_CHECK(SHA_Finish(&handle, hashDigest));
            SHA_Power(false);
            if(memcmp(hashDigest, sha2SelfTestExpRes[i], SHA_2_HASH_LENGTH))
            {
                return DEFS_STATUS_FAIL;
            }

            SHA_Calc(SHA_TYPE_SHA2, sha2SelfTestBuff[i], sha2SelfTestBuffLen[i], hashDigest);
            if(memcmp(hashDigest, sha2SelfTestExpRes[i], SHA_2_HASH_LENGTH))
            {
                return DEFS_STATUS_FAIL;
            }
        }
    }
    return 0;
}
#endif //SHA_SELF_TEST

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                             LOCAL FUNCTIONS                                             */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_FlushLocalBuffer_l                                                                 */
/*                                                                                                         */
/* Parameters:                                                                                             */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     This routine flush secrun buffer to SHA module                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void SHA_FlushLocalBuffer_l (const u32* buff)
{
    unsigned int i;

    for(i = 0; i < (SHA_BLOCK_LENGTH / sizeof(u32)); i++)
    {
        iowrite32(buff[i], HASH_DATA_IN);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_BusyWait_l                                                                         */
/*                                                                                                         */
/* Parameters:                                                                                             */
/* Returns:         0 if no error was found or DEFS_STATUS_ERROR otherwise                                 */
/* Side effects:                                                                                           */
/* Description:     This routine wait for SHA unit to no longer be busy                                    */
/*---------------------------------------------------------------------------------------------------------*/
static int SHA_BusyWait_l (void)
{
    /*-----------------------------------------------------------------------------------------------------*/
    /* While SHA module is busy                                                                            */
    /*-----------------------------------------------------------------------------------------------------*/
    BUSY_WAIT_TIMEOUT((READ_REG_FIELD(HASH_CTR_STS, HASH_CTR_STS_SHA_BUSY) == 0x01), SHA_TIMEOUT);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_GetShaDigest_l                                                                     */
/*                                                                                                         */
/* Parameters:      hashDigest - buffer for the hash output.                                               */
/*                  shaType - SHA module type                                                              */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     This routine copy the hash digest from the hardware and into given buffer( in ram)     */
/*---------------------------------------------------------------------------------------------------------*/
static void SHA_GetShaDigest_l(
    u8* hashDigest,
    SHA_TYPE_T shaType
)
{
  u16 j;
  u32* src =     (u32 *)(HASH_DIG_H(0));
  u32* dest =    (u32 *)(void*)hashDigest;
  u8 len = ((shaType == SHA_TYPE_SHA2) ? SHA_2_HASH_LENGTH : SHA_1_HASH_LENGTH) / sizeof(u32);

  /*-------------------------------------------------------------------------------------------------------*/
  /* Copy Bytes from SHA module to given buffer                                                            */
  /*-------------------------------------------------------------------------------------------------------*/
  for (j = 0; j < len; j++)
  {
    dest[j] = src[j];
  }
  //sha_print_hex_dump("\t\t\t*npcmX50-SHA: SHA_GetShaDigest_l\n",(unsigned char *)src, len*sizeof(u32));
    
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_SetShaDigest_l                                                                     */
/*                                                                                                         */
/* Parameters:      hashDigest - input buffer to set as hash digest(in SHA module).                        */
/*                  shaType - SHA module type                                                              */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     This routine set the hash digest in the hardware from a given buffer( in ram)          */
/*---------------------------------------------------------------------------------------------------------*/
static  void SHA_SetShaDigest_l(
    const u32* hashDigest,
    SHA_TYPE_T shaType
)
{
  u16 j;
  u32* dest = (u32 *)(HASH_DIG_H(0));
  u8 len = ((shaType == SHA_TYPE_SHA2) ? SHA_2_HASH_LENGTH : SHA_1_HASH_LENGTH) / sizeof(u32);

  /*-------------------------------------------------------------------------------------------------------*/
  /* Copy Bytes from given buffer to SHA module                                                            */
  /*-------------------------------------------------------------------------------------------------------*/
  for (j = 0; j < len; j++)
  {
    dest[j] = hashDigest[j];
  }
  //sha_print_hex_dump("\t\t\t*npcmX50-SHA: SHA_SetShaDigest_l\n",(unsigned char *)hashDigest, len*sizeof(u32));
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_SetBlock_l                                                                         */
/*                                                                                                         */
/* Parameters:      data        - data to copy                                                             */
/*                  len         -  size of data                                                            */
/*                  position    - byte offset into the block at which data should be placed                */
/*                  block       - block buffer                                                             */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     This routine load bytes into block buffer                                              */
/*---------------------------------------------------------------------------------------------------------*/
static void SHA_SetBlock_l(const u8* data,u32 len, u16 position, u32* block)
{
    u8 * dest = (u8*)block;
    memcpy(dest + position, data, len);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_SetBlock_l                                                                         */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  len -  size of data                                                                    */
/*                  position - byte offset into the block at which data should be placed                   */
/*                  block - block buffer                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     This routine load zero's into the block buffer                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void SHA_ClearBlock_l (
    u16 len,
    u16 position,
    u32* block
)
{
    u8 * dest = (u8*)block;
    memset(dest + position, 0, len);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:        SHA_SetLength32_l                                                                      */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*                  handlePtr  -   SHA processing handle pointer                                           */
/*                  block - block buffer                                                                   */
/* Returns:         none                                                                                   */
/* Side effects:                                                                                           */
/* Description:     This routine set the length of the hash's data                                         */
/*                  len is the 32-bit byte length of the message                                           */
/*lint -efunc(734,SHA_SetLength32_l) Supperess loss of percision lint warning                              */
/*---------------------------------------------------------------------------------------------------------*/
static void SHA_SetLength32_l (const SHA_HANDLE_T* handlePtr, u32* block)
{
  u16*       secrunBufferSwappedPtr = (u16*)(void*)(block);

  secrunBufferSwappedPtr[(SHA_BLOCK_LENGTH/sizeof(u16)) - 1] = (u16)
        ((handlePtr->length0 << 3) << 8) | ((u16) (handlePtr->length0 << 3) >> 8);
  secrunBufferSwappedPtr[(SHA_BLOCK_LENGTH/sizeof(u16)) - 2] = (u16)
        ((handlePtr->length0 >> (16-3)) >> 8) | ((u16) (handlePtr->length0 >> (16-3)) << 8);
  secrunBufferSwappedPtr[(SHA_BLOCK_LENGTH/sizeof(u16)) - 3] = (u16)
    ((handlePtr->length1 << 3) << 8) | ((u16) (handlePtr->length1 << 3) >> 8);
  secrunBufferSwappedPtr[(SHA_BLOCK_LENGTH/sizeof(u16)) - 4] = (u16)
    ((handlePtr->length1 >> (16-3)) >> 8) | ((u16) (handlePtr->length1 >> (16-3)) << 8);
}







/********************** AF_ALG ********************************/

static int npcm750_sha1_init(struct ahash_request *req);
static int npcm750_sha256_init(struct ahash_request *req);
static int npcm750_sha_update(struct ahash_request *req);
static int npcm750_sha_final(struct ahash_request *req);
static int npcm750_sha1_finup(struct ahash_request *req);
static int npcm750_sha256_finup(struct ahash_request *req);
static int npcm750_sha_cra_init(struct crypto_tfm *tfm);
static void npcmx50sha_unregister_algs(void);
static int npcmx50sha_register_algs(void);
static int npcm750sha_probe(struct platform_device *pdev);
static int __exit npcmx50sha_remove(struct platform_device *pdev);




struct npcm750_sha_desc {
	SHA_HANDLE_T        shaHandle;
};


static int npcm750_sha1_init(struct ahash_request *req)
{
    struct npcm750_sha_desc *dctx = ahash_request_ctx(req);

	sha_print("\t\t\t* SHA1 init\n");

    memset(dctx, 0, sizeof(struct npcm750_sha_desc));
	SHA_Init(&(dctx->shaHandle));
    SHA_Start(&(dctx->shaHandle), SHA_TYPE_SHA1);

	return 0;
}

static int npcm750_sha256_init(struct ahash_request *req)
{
    struct npcm750_sha_desc *dctx = ahash_request_ctx(req);

	sha_print("\t\t\t* SHA256 init\n");

    memset(dctx, 0, sizeof(struct npcm750_sha_desc));

	SHA_Init(&(dctx->shaHandle));
    SHA_Start(&(dctx->shaHandle), SHA_TYPE_SHA2);

	return 0;
}


static int npcm750_sha_update(struct ahash_request *req)
{
    int ret = 0;
    struct npcm750_sha_desc *dctx = ahash_request_ctx(req);
    struct crypto_hash_walk walk;
    unsigned int nbytes = req->nbytes;
    u8  *src_paddr;

    nbytes = crypto_ahash_walk_first(req, &walk);

	while(nbytes)
	{

	    sha_print("\n\t\t\t* SHA update\n");
	    src_paddr =  (u8 *)phys_to_virt(page_to_phys(walk.pg) +	walk.offset);
       
		ret = SHA_Update(&(dctx->shaHandle),  src_paddr, nbytes);
		nbytes = crypto_ahash_walk_done(&walk, ret);
	}
	
    sha_print("\t\t\t* SHA update done, returning %d\n", ret);
	return ret;
}



static int npcm750_sha_final(struct ahash_request *req)
{
    int ret = 0;
    struct npcm750_sha_desc *dctx = ahash_request_ctx(req);
    u8 *out = req->result;

    sha_print("\n\t\t\t* SHA final\n");
    
	ret = SHA_Finish(&(dctx->shaHandle), out);

    sha_print("\t\t\t* SHA finish done, returning %d\n", ret);
	return ret;
}





static int npcm750_sha1_finup(struct ahash_request *req)
{
    int ret = 0;
 
	sha_print("\t\t* SHA1 finup\n");

	npcm750_sha1_init(req);
	
	ret = npcm750_sha_update(req);
	if(ret != 0)
	{
	    printk("\t\t\t* SHA1 finup: update fail, ret = %d\n", ret);
        return ret;
	}
	ret = npcm750_sha_final(req);
	if(ret != 0)
	{
	    printk("\t\t\t* SHA1 finup: final fail, ret = %d\n", ret);
        return ret;
	}
	return ret;
}


static int npcm750_sha256_finup(struct ahash_request *req)
{
    int ret = 0;
 
	sha_print("\t\t* SHA256 finup\n");

	npcm750_sha256_init(req);
	
	ret = npcm750_sha_update(req);
	if(ret != 0)
	{
	    printk("\t\t\t* SHA256 finup: update fail, ret = %d\n", ret);
        return ret;
	}
	ret = npcm750_sha_final(req);
	if(ret != 0)
	{
	    printk("\t\t\t* SHA256 finup: final fail, ret = %d\n", ret);
        return ret;
	}
	return ret;
}

static int npcm750_sha_cra_init(struct crypto_tfm *tfm)
{	
    crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),	
        sizeof(struct npcm750_sha_desc) + NUVOTON_ALIGNMENT);	
    return 0;
}

static struct ahash_alg sha_algs[2] = {
{
	.init		= npcm750_sha1_init,
	.update		= npcm750_sha_update,
	.final		= npcm750_sha_final,
	.finup		= npcm750_sha1_finup,
	.digest		= npcm750_sha1_finup,
	
	.halg.digestsize	= SHA1_DIGEST_SIZE,
	.halg.statesize  	= sizeof(struct npcm750_sha_desc) + NUVOTON_ALIGNMENT,
	.halg.base	= {
		.cra_name		    = "sha1",
	    .cra_driver_name	= "nuvoton_sha",
		.cra_priority		= 300,
		.cra_flags		    = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA1_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct npcm750_sha_desc) + NUVOTON_ALIGNMENT,
		.cra_alignmask		= 0,
		.cra_module		    = THIS_MODULE,
		.cra_init           = npcm750_sha_cra_init
	}
},
{
	.init		= npcm750_sha256_init,
	.update		= npcm750_sha_update,  // same as sha 1. The req already have the type (set during init)
	.final		= npcm750_sha_final,
	.finup		= npcm750_sha256_finup,
	.digest		= npcm750_sha256_finup,
	
	.halg.digestsize	= SHA256_DIGEST_SIZE,
	.halg.statesize  	= sizeof(struct npcm750_sha_desc) + NUVOTON_ALIGNMENT,
	.halg.base	= {
		.cra_name		    = "sha256",
	    .cra_driver_name	= "nuvoton_sha",
		.cra_priority		= 300,
		.cra_flags		    = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA256_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct npcm750_sha_desc) + NUVOTON_ALIGNMENT,
		.cra_alignmask		= 0,
		.cra_module		    = THIS_MODULE,
		.cra_init           = npcm750_sha_cra_init
	}
}
};


static void npcmx50sha_unregister_algs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sha_algs); i++)
		crypto_unregister_ahash(&(sha_algs[i]));
		
	SHA_Power(false);	
}

static int npcmx50sha_register_algs(void)
{
	int err, i, j;

	sha_print("\t\t\t* SHA register algo\n");

	SHA_Power(true);
	
	SHA_Reset();

	for (i = 0; i < ARRAY_SIZE(sha_algs); i++) {
		err = crypto_register_ahash(&(sha_algs[i]));
		if (IS_ERR_VALUE(err))
			goto err_sha_algs;
	}
	

	return 0;

err_sha_algs:
    printk( "\t\t\t* SHA register algo%d fail\n", i);
	for (j = 0; j < i; j++)
		crypto_unregister_ahash(&(sha_algs[j]));

	SHA_Power(false);

	return err;
}



static int npcm750sha_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	sha_print("\t\t\t* SHA probe start\n");
	/*
	 * A bit ugly, and it will never actually happen but there can
	 * be only one SHA and this catches any bork
	 */
	if (sha_base!=0)
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
	sha_base = ioremap(res->start, resource_size(res));
	sha_print("\t\t\t* SHA base is 0x%08X, size is 0x%08X (phys 0x%08X).\n", (u32)sha_base, resource_size(res), res->start);
	if (!sha_base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = npcmx50sha_register_algs();
	if (IS_ERR_VALUE(ret)) {
		goto err_register;
	}

    printk("NPCMx50: SHA module is ready\n");


    SHA_PrintRegs();

	return 0;

err_register:
	iounmap(sha_base);
	sha_base = NULL;
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_region:
    printk( "\t\tNPCMx50: SHA module load fail .  Error %d\n", ret);

	return ret;
}

static int __exit npcmx50sha_remove(struct platform_device *pdev)
{
	struct resource *res = dev_get_drvdata(&pdev->dev);

	npcmx50sha_unregister_algs();

	printk(KERN_NOTICE  "NPCMx50-SHA: remove: stop using Nuvoton npcmx50 SHA module.\n");

	iounmap(sha_base);

	release_mem_region(res->start, resource_size(res));
	sha_base = NULL;
	return 0;
}


/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm750_sha");

static const struct of_device_id sha_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-sha",  },
	{},
};
MODULE_DEVICE_TABLE(of, sha_dt_id);

static struct platform_driver npcmx50sha_driver = {
	.probe		= npcm750sha_probe,
	.remove		= npcmx50sha_remove,
	.shutdown	= NULL,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "npcm750_sha",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(sha_dt_id),
	}
};



MODULE_ALIAS_CRYPTO("nuvoton-sha");

module_platform_driver(npcmx50sha_driver);

MODULE_DESCRIPTION("Nuvoton Technologies SHA HW acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Tali Perry - Nuvoton Technologies");

