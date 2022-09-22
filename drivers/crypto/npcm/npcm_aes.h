/*---------------------------------------------------------------------------------------------------------*/
/*  Nuvoton Technology Corporation Confidential                                                            */
/*                                                                                                         */
/*  Copyright (c) 2014-2016 by Nuvoton Technology Corporation                                              */
/*  All rights reserved                                                                                    */
/*                                                                                                         */
/*<<<------------------------------------------------------------------------------------------------------*/
/* File Contents:                                                                                          */
/*   npcm750_aes.h                                                                                         */
/*            This file contains Advanced Encryption Standard (AES) interface                              */
/* Project:                                                                                                */
/*            SWC HAL                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef NPCM_AES_H
#define NPCM_AES_H



#include <crypto/internal/hash.h>

#define REG32               volatile u32
#define PTR32               REG32 *

typedef struct bit_field {
	u32 offset;
	u32 size;
} bit_field_t;

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                                 DEFINES                                                 */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           Registers definition                                          */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/**************************************************************************************************************************/
/*   Key 0 Register (AES_KEY_0)                                                                                           */
/**************************************************************************************************************************/
#define  AES_KEY_0                      (aes_base + 0x400)

/**************************************************************************************************************************/
/*   Initialization Vector 0 Register (AES_IV_0)                                                                          */
/**************************************************************************************************************************/
#define  AES_IV_0                       (aes_base + 0x440)
static const bit_field_t   AES_IV_0_AES_IV_0     = {  0, 32 };

/**************************************************************************************************************************/
/*   Initial Counter Value Register (AES_CTR0)                                                                            */
/**************************************************************************************************************************/
#define  AES_CTR0                       (aes_base + 0x460)


//#define AES_CTR_ADDR            REG_ADDR(AES_CTR0)   /* 16 byte   */
//#define AES_CTR                 ((PTR32)(AES_CTR_ADDR))


/**************************************************************************************************************************/
/*   Busy Register (AES_BUSY)                                                                                             */
/**************************************************************************************************************************/
#define  AES_BUSY                       (aes_base + 0x470)
static const bit_field_t   AES_BUSY_AES_BUSY     = {  0, 1 };
#define  AES_BUSY_NOT_BUSY               0                  /* 0 On read:  AES not busy status.                                                                            */
#define  AES_BUSY_IS_BUSY                1                  /* 1 On read:  AES busy status.                                                                                */
#define  AES_BUSY_Configuration          0                  /* 0 On write: Configuration mode.                                                                             */
#define  AES_BUSY_Data                   1                  /* 1 On write: Data processing mode.                                                                           */

/**************************************************************************************************************************/
/*   Sample Key Register (AES_SK)                                                                                         */
/**************************************************************************************************************************/
#define  AES_SK                         (aes_base + 0x478)                                                                           
static const bit_field_t   AES_SK_AES_SK     = { 0, 1 };

/**************************************************************************************************************************/
/*   Previous IV 0 Value Register (AES_PREV_IV_0)                                                                         */
/**************************************************************************************************************************/
#define  AES_PREV_IV_0                  (aes_base + 0x490)
static const bit_field_t   AES_PREV_IV_0_AES_PREV_IV_0     = {  0, 32 };
/**************************************************************************************************************************/
/*   Direct Data Access for Register (AES_DIN_DOUT)                                                                       */
/**************************************************************************************************************************/
#define  AES_DIN_DOUT                   (aes_base + 0x4A0) 

/**************************************************************************************************************************/
/*   Control Register (AES_CONTROL)                                                                                       */
/**************************************************************************************************************************/
#define  AES_CONTROL                    (aes_base + 0x4C0)
#define  AES_CONTROL_WORD_HIGH          (aes_base + 0x4C2)
static const bit_field_t   AES_CONTROL_DIRECT_ACC_N_DIN_DOUT     = {  31, 1 };
static const bit_field_t   AES_CONTROL_NK_KEY0     = {  12, 2 };
static const bit_field_t   AES_CONTROL_MODE_KEY0     = {  2, 3 };
static const bit_field_t   AES_CONTROL_DEC_ENC     = {  0, 1 };
/**************************************************************************************************************************/
/*   Version Register (AES_VERSION)                                                                                       */
/**************************************************************************************************************************/
#define  AES_VERSION                    (aes_base + 0x4C4)                                                                                                    */
static const bit_field_t   AES_VERSION_MAJOR_VERSION_NUMBER     = {  12, 4 };
static const bit_field_t   AES_VERSION_MINOR_VERSION_NUMBER     = {  8, 4 };
static const bit_field_t   AES_VERSION_FIXES     = {  0, 8 };

/**************************************************************************************************************************/
/*   Hardware Flag Register (AES_HW_FLAGS)                                                                                */
/**************************************************************************************************************************/
#define  AES_HW_FLAGS                   (aes_base + 0x4C8)
static const bit_field_t   AES_HW_FLAGS_DFA_CNTRMSR_EXIST     = {  12, 1 };
static const bit_field_t   AES_HW_FLAGS_SECOND_REGS_SET_EXISTS     = {  11, 1 };
static const bit_field_t   AES_HW_FLAGS_TUNNELING_ENB     = {  10, 1 };
static const bit_field_t   AES_HW_FLAGS_AES_SUPPORT_PREV_IV     = {  9, 1 };
static const bit_field_t   AES_HW_FLAGS_USE_5_SBOXES     = {  8, 1 };
static const bit_field_t   AES_HW_FLAGS_USE_SBOX_TABLE     = {  5, 1 };
static const bit_field_t   AES_HW_FLAGS_ONLY_ENCRYPT     = {  4, 1 };
static const bit_field_t   AES_HW_FLAGS_CTR_EXIST     = {  3, 1 };
static const bit_field_t   AES_HW_FLAGS_DPA_CNTRMSR_EXIST     = {  2, 1 };
static const bit_field_t   AES_HW_FLAGS_AES_LARGE_RKEK     = {  1, 1 };
static const bit_field_t   AES_HW_FLAGS_SUPPORT_256_192_KEY     = {  0, 1 };
/**************************************************************************************************************************/
/*   Software Reset Register (AES_SW_RESET)                                                                               */
/**************************************************************************************************************************/
#define  AES_SW_RESET                   (aes_base + 0x4F4)       /* Location: AES_BA+4F4h */
static const bit_field_t   AES_SW_RESET_AES_SW_RESET     = {  0, 1 };
/**************************************************************************************************************************/
/*   DFA Error Status Register (AES_DFA_ERROR_STATUS)                                                                     */
/**************************************************************************************************************************/
#define  AES_DFA_ERROR_STATUS           (aes_base + 0x4F8)       /* Location: AES_BA+4F8h */
static const bit_field_t   AES_DFA_ERROR_STATUS_AES_DFA_ERROR_STATUS     = {  0, 1 };
/**************************************************************************************************************************/
/*   RBG Seeding Ready Register (AES_RBG_SEEDING_READY)                                                                   */
/**************************************************************************************************************************/
#define  AES_RBG_SEEDING_READY          (aes_base + 0x4FC)      /* Location: AES_BA+4FCh */
static const bit_field_t   AES_RBG_SEEDING_READY_AES_RBG_SEEDING_READY     = {  0, 1 };
/**************************************************************************************************************************/
/*   AES FIFO Data Register (AES_FIFO_DATA)                                                                               */
/**************************************************************************************************************************/
#define  AES_FIFO_DATA                  (aes_base + 0x500)       /* Location: AES_BA+500h */
static const bit_field_t   AES_FIFO_DATA_AES_FIFO_DATA     = {  0, 32 };
/**************************************************************************************************************************/
/*   AES FIFO Status Register (AES_FIFO_STATUS)                                                                           */
/**************************************************************************************************************************/
#define  AES_FIFO_STATUS                (aes_base + 0x600)       /* Location: AES_BA+600h */
static const bit_field_t   AES_FIFO_STATUS_DOUT_FIFO_UNDERFLOW     = {  5, 1 };
static const bit_field_t   AES_FIFO_STATUS_DIN_FIFO_OVERFLOW     = {  4, 1 };
static const bit_field_t   AES_FIFO_STATUS_DOUT_FIFO_EMPTY     = {  3, 1 };
static const bit_field_t   AES_FIFO_STATUS_DOUT_FIFO_FULL     = {  2, 1 };
static const bit_field_t   AES_FIFO_STATUS_DIN_FIFO_EMPTY     = {  1, 1 };
static const bit_field_t   AES_FIFO_STATUS_DIN_FIFO_FULL     = {  0, 1 };

/*---------------------------------------------------------------------------------------------------------*/
/* AES Mode                                                                                                */
/* Do no change order, hardware dependent                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCM_AES_MODE_T
{
    AES_MODE_ECB = 0,   /* Electronic Codebook          */
    AES_MODE_CBC = 1,   /* Cipher Block Chaining        */
    AES_MODE_CTR = 2,   /* Counter                      */
    AES_MODE_MAC = 3    /* Message Authentication Code  */
} NPCM_AES_MODE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* AES Key Size                                                                                            */
/* Do no change order, hardware dependent                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCM_AES_KEY_SIZE_T
{
    AES_KEY_SIZE_128  = 0 ,
    AES_KEY_SIZE_192  = 1 ,
    AES_KEY_SIZE_256  = 2 ,
    AES_KEY_SIZE_USE_LAST = 0xFF  // AES Key is preloaed by npcmx50_aes_set_key. 
} NPCM_AES_KEY_SIZE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* AES Operation                                                                                           */
/* Do not change the order, hardware dependent                                                             */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCM_AES_OP_T
{
    AES_OP_ENCRYPT = 0,
    AES_OP_DECRYPT = 1
} NPCM_AES_OP_T;


/*---------------------------------------------------------------------------------------------------------*/
/* AES module macro definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/* The bit length of supported keys for the AES algorithm */
#define AES_KEY_BIT_SIZE(size)      (128 + (NPCM_AES_KEY_SIZE_T)(size) * 64)

/* The byte length of supported keys for the AES algorithm */
#define AES_KEY_BYTE_SIZE(size)     (AES_KEY_BIT_SIZE(size) / 8)

/* The bit length of supported keys for the AES algorithm */
#define NPCM_AES_KEY_SIZE_TO_ENUM(size)  ((size==32) ? AES_KEY_SIZE_256 : ((size==24) ? AES_KEY_SIZE_192 : AES_KEY_SIZE_128))

/* # of bytes needed to represent a key */
// #define AES_MAX_KEY_SIZE            AES_KEY_BYTE_SIZE(AES_KEY_SIZE_256)

/* The byte length of a block for the AES algorithm (b = 128 bit) */
// #define AES_BLOCK_SIZE              AES_KEY_BYTE_SIZE(AES_KEY_SIZE_128)

/* # of bytes needed to represent an IV  */
#define AES_MAX_IV_SIZE             (AES_BLOCK_SIZE)

/* # of bytes needed to represent a counter  */
#define AES_MAX_CTR_SIZE            (AES_BLOCK_SIZE)

/* Calculate the number of blocks in the formatted message */
#define AES_COMPLETE_BLOCKS(size)   (((size) + (AES_BLOCK_SIZE - 1)) / AES_BLOCK_SIZE)

#define AES_BLOCK_MASK	(~(AES_BLOCK_SIZE-1))

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                           INTERFACE FUNCTIONS                                           */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/




#endif //NPCM_AES_H