/*
 * NPCMP750 BMC On-Chip OTP (FUSE) Memory Interface
 *
 * Copyright 2016 Nuvoton Technologies
 *
 * Licensed under the GPL-2 or later.
 *
 *
 */

#ifndef NPCM750_NPCM750_OTP_H
#define NPCM750_NPCM750_OTP_H

typedef struct bit_field {
    u8 offset;
    u8 size;
} bit_field_t;

/*---------------------------------------------------------------------------------------------------------*/
/* OTP IOCTLs                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCM750_OTP_IOC_MAGIC       '2'             /* used for all IOCTLs to npcm750_otp.  AES Key handling */

#define IOCTL_SELECT_AES_KEY        _IO(NPCM750_OTP_IOC_MAGIC,   0)/* To Configure the Bus topology based on I2CTopology.config file*/
#define IOCTL_DISABLE_KEY_ACCESS    _IO(NPCM750_OTP_IOC_MAGIC,   1)/* To send request header and perform I2C operation */
#define IOCTL_GET_AES_KEY_NUM       _IO(NPCM750_OTP_IOC_MAGIC,   2)/* To Configure the Bus topology based on I2CTopology.config file*/



/*---------------------------------------------------------------------------------------------------------*/
/* Fuse module definitions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define NPCM750_OTP_ARR_BYTE_SIZE          (1024)
#define NPCM750_KEYS_ARR_BYTE_SIZE         (128)

#define NPCM750_RSA_KEY_BYTE_SIZE          (256)

// 4 aes keys, 3 rsa keys:
#define NPCM750_MAX_KEYS                   (4)


/*---------------------------------------------------------------------------------------------------------*/
/* Fuse ECC type                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCM750_OTP_ECC_TYPE_tag
{
    NPCM750_OTP_ECC_MAJORITY = 0,
    NPCM750_OTP_ECC_NIBBLE_PARITY = 1,
    NPCM750_OTP_ECC_NONE = 2
}  NPCM750_OTP_ECC_TYPE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* Fuse key Type                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCM750_OTP_KEY_TYPE_tag
{
    NPCM750_OTP_KEY_AES = 0,
    NPCM750_OTP_KEY_RSA = 1
}  NPCM750_OTP_KEY_TYPE_T;



/*---------------------------------------------------------------------------------------------------------*/
/* Fuse module enumerations                                                                                */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Storage Array Type:                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    NPCM750_KEY_SA    = 0,
    NPCM750_FUSE_SA   = 1
} NPCM750_OTP_STORAGE_ARRAY_T;



/*---------------------------------------------------------------------------------------------------------*/
/* AES Key Size                                                                                            */
/* Do no change order, hardware dependent                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    AES_KEY_SIZE_128 = 0, /*  = 128 */
    AES_KEY_SIZE_192 = 1, /*  = 192 */
    AES_KEY_SIZE_256 = 2, /*  = 256 */
} AES_KEY_SIZE_T;



/*---------------------------------------------------------------------------------------------------------*/
/* FUSTRAP fields definition                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum NPCM750_OTP_FUSTRAP_FIELDS_T_tag
{
    NPCM750_OTP_FUSTRAP_DIS_FAST_BOOT = 29,                    // (Disable Fast Boot).
    NPCM750_OTP_FUSTRAP_Bit_28 = 28,                           // unknown register field !
    NPCM750_OTP_FUSTRAP_oWDEN = 27,                            // (Watchdog Enable).
    NPCM750_OTP_FUSTRAP_oHLTOF = 26,                           // (Halt on Failure). I
    NPCM750_OTP_FUSTRAP_oAESKEYACCLK = 25,                     // (AES Key Access Lock).
    NPCM750_OTP_FUSTRAP_oJDIS = 24,                            // (JTAG Disable).
    NPCM750_OTP_FUSTRAP_oSECBOOT = 23,                         // (Secure Boot).
    NPCM750_OTP_FUSTRAP_USEFUSTRAP = 22,                       //
    NPCM750_OTP_FUSTRAP_oPKInvalid2_0 = 19,                    //
    NPCM750_OTP_FUSTRAP_oAltImgLoc = 18,                       //  Alternate image location definition.

    /*-----------------------------------------------------------------------------------------------------*/
    /* Added on Z2                                                                                         */
    /*-----------------------------------------------------------------------------------------------------*/
    NPCM750_OTP_FUSTRAP_oHINDDIS         =      14,            // oHINDDIS: disable eSPI independent mode
    NPCM750_OTP_FUSTRAP_oSecBootDisable  =      15,            // {oSecBootDisable} - when set, disables capability enter Secure Mode. Used for Derivatives.
    NPCM750_OTP_FUSTRAP_oCPU1STOP2 =            16,            // {oCPU1STOP2} - when set, stops CPU core 1 clock.
    NPCM750_OTP_FUSTRAP_oCPU1STOP1 =            17             // {oCPU1STOP1} - when set, CPU core 1 stops and cannot be woken.


} NPCM750_OTP_FUSTRAP_FIELDS_T;





/**************************************************************************************************************************/
/*   Fuse Array Control Register (FCTL1,2)                                                                                */
/**************************************************************************************************************************/
#define  FCTL(n)                         (otp_base[n] + 0x14)       /* Location: BASE+14h */
static const bit_field_t  FCTL_FCTL                   = { 0 , 32 };          
/* 31-0 FCTL. A sequence of two adjacent writes to this register, first with a 
value of 0000_0001h and the second with   */

/**************************************************************************************************************************/
/*   Fuse Array Status Register (FST1,2)                                                                                  */
/**************************************************************************************************************************/
#define  FST(n)                          (otp_base[n] + 0x00)      /* Location: BASE+00h */
static const bit_field_t  FST_RIEN                   = { 2 , 1 };              /* 2 RIEN (Ready Interrupt Enable). Enables an interrupt when RDST bit is set. (Bit added in Poleg)                      */
static const bit_field_t  FST_RDST                   = { 1 , 1 };              /* 1 RDST (Ready Status). This bit is set by hardware when a read or program operation is competed and                   */
static const bit_field_t  FST_RDY                    = { 0 , 1 };              /* 0 RDY (Ready). If cleared to 0, indicates that the fuse array interface is busy processing a read or                  */

/**************************************************************************************************************************/
/*   Fuse Array Address Register (FADDR1,2)                                                                               */
/**************************************************************************************************************************/
#define  FADDR(n)                        (otp_base[n] + 0x04)       /* Location: BASE+04h */
static const bit_field_t  FADDR_BITPOS                = { 10 , 3 };            /* (n)-10 BITPOS (Bit Position). For write operations, designates the position of the bit (to be programmed) in the byte  */
static const bit_field_t  FADDR_BYTEADDR              = { 0 , 10 };            /* 9-0 BYTEADDR (Fuse Read Address). Designates the byte address in the fuse array for read and program                  */

/**************************************************************************************************************************/
/*   Fuse Array Data Register (FDATA1,2)                                                                                  */
/**************************************************************************************************************************/
#define  FDATA(n)                        (otp_base[n] + 0x08)       /* Location: BASE+08h */
static const bit_field_t  FDATA_FDATA                 = { 0 , 8 };              
/* 7-0 FDATA. On read, returns the data from the read operation. The register 
contents are valid only if RDY bit in      */

/**************************************************************************************************************************/
/*   Fuse Array Configuration Register (FCFG1,2)                                                                          */
/**************************************************************************************************************************/
#define  FCFG(n)                         (otp_base[n] + 0x0C)       /* Location: BASE+0Ch */
static const bit_field_t  FCFG_FDIS                   = { 31 , 1 };             /* 31 FDIS (Fuse Array Disable). This sticky bit disables access to the first 2048 bits of the fuse array, either        */
static const bit_field_t  FCFG_APBRT                  = { 24 , 6 };            /* 29-24 APBRT (APB Clock Rate). Informs the fuse array state machine on the APB clock rate in MHz. The                  */
static const bit_field_t  FCFG_FCFGLK                 = { 16 , 8 };            /* 23-16 FCFGLK (FCFG Lock). Bit FCFGLKn locks the corresponding FPRGLKn and FRDLKn bits. These bits                     */
static const bit_field_t  FCFG_FPRGLK                 = { 8 , 8  };            /* FPRGLK (Fuse Program Lock). Controls program access to the fuse array. FPRGLKn bit protects the                       */
static const bit_field_t  FCFG_FRDLK                  = { 0 , 8  };            /* FRDLK (Fuse Read Lock). Controls APB read access to the fuse array. Bit FRDLKn protects the nth                       */

/**************************************************************************************************************************/
/*   Fuse Key Index Register (FKEYIND)                                                                                    */
/**************************************************************************************************************************/
#define  FKEYIND                        (otp_base[0] + 0x10)      /* Location: BASE+10h */
static const bit_field_t  FKEYIND_KIND                  = { 18 , 2 };            /* 19-18 KIND (Key Index). Indicates the address of the key in the fuse array, in 5(n)-bit steps. (Changed in             */
static const bit_field_t  FKEYIND_FUSERR                = { 8 , 1  };             /* 8 FUSERR (Fuse Error). Indicates that the ECC decoding mechanism detected an error while reading                      */
static const bit_field_t  FKEYIND_KSIZE                 = { 4 , 3  };            /* 6-4 KSIZE (Key Size). Indicates the size of the cryptographic key to upload on the sideband key port.                 */
static const bit_field_t  FKEYIND_KVAL                  = { 0 , 1  };             /* 0 KVAL (Key Valid). Indicates whether the sideband key port contents are valid. This bit is cleared to 0              */

/**************************************************************************************************************************/
/*   Fuse Strap Register (FUSTRAP)                                                                                        */
/**************************************************************************************************************************/
#define  FUSTRAP                        (otp_base[1] + 0x10)       /* Location: BASE+10h */
static const bit_field_t  FUSTRAP_DIS_FAST_BOOT         = { 29 , 1 };             /* 29 DIS_FAST_BOOT (Disable Fast Boot). (Z2) Disables the option to jump to SPI flash before MDLR is written.           */
static const bit_field_t  FUSTRAP_BIT_28                = { 28 , 1 };   /* Apears on ROM flow. Name unknown ! */
static const bit_field_t  FUSTRAP_oWDEN                 = { 27 , 1 };             /* 27 oWDEN (Watchdog Enable). If set, tells the ROM Code to enable a 22 seconds watchdog before jumping to              */
static const bit_field_t  FUSTRAP_oHLTOF                = { 26 , 1 };             /* 26 oHLTOF (Halt on Failure). If set, tells the ROM Code to halt execution on a signature verification failure.        */
static const bit_field_t  FUSTRAP_oAESKEYACCLK          = { 25 , 1 };             /* 25 oAESKEYACCLK (AES Key Access Lock). If set, prevents any access to the first 2048 bits of the Key Storage          */
static const bit_field_t  FUSTRAP_oJDIS                 = { 24 , 1 };             /* 24 oJDIS (JTAG Disable). If set, locks (disables) the BMC CPU JTAG port. It can be reopened via JTAGDIS bit in        */
static const bit_field_t  FUSTRAP_oSECBOOT              = { 23 , 1 };             /* 23 oSECBOOT (Secure Boot). If set, indicates that the ROM code will perform an integrity check on power-up,           */
static const bit_field_t  FUSTRAP_USEFUSTRAP            = { 22 , 1 };             /* 22 USEFUSTRAP. When set, indicates the configuration in this register must be used instead of the configuration       */
static const bit_field_t  FUSTRAP_oPKInvalid2_0         = { 19 , 3 };             /* 21-19 oPKInvalid2-0. A 3 bit field that may invalidate a Public Key (oPK2-0) stored in OTP. Bit 21 is for oPK2, Bit 20*/
static const bit_field_t  FUSTRAP_oAltImgLoc            = { 18 , 1 };             /* 18 oAltImgLoc. Alternate image location definition.                                                                   */
static const bit_field_t  FUSTRAP_FUSTRAP_SFAB          = { 11 , 1 };             /* 11 FUSTRAP(n). System Flash Attached to BMC (SFAB).                                                                    */
static const bit_field_t  FUSTRAP_FUSTRAP3_1            = { 0 ,  3 };             /* 2-0 FUSTRAP3-1. CPU core clock and DDR4 memory frequency (CKFRQ). See Power-On Setting Register                       */

/*---------------------------------------------------------------------------------------------------------*/
/* Added on Z2:                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
static const bit_field_t  FUSTRAP_oHINDDIS             = { 14 , 1 };              /* oHINDDIS (Host Independence Disable). Disables initialization of a few registers in step 1 of the ROM code */
static const bit_field_t  FUSTRAP_oSecBootDisable      = { 15 , 1 };              /* {oSecBootDisable} - when set, disables capability enter Secure Mode. Used for Derivatives.*/
static const bit_field_t  FUSTRAP_oCPU1STOP2           = { 16 , 1 };              /* {oCPU1STOP2} - when set, stops CPU core 1 clock. */
static const bit_field_t  FUSTRAP_oCPU1STOP1           = { 17 , 1 };              /* {oCPU1STOP1} - when set, CPU core 1 stops and cannot be woken.*/




/*---------------------------------------------------------------------------------------------------------*/
/* FKEYIND field values                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define  FKEYIND_KSIZE_VALUE_128          0x4
#define  FKEYIND_KSIZE_VALUE_192          0x5
#define  FKEYIND_KSIZE_VALUE_256          0x6


#endif /* NPCM750_NPCM750_OTP_H */
