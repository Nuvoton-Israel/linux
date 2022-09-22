// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021-2022 Nuvoton Technology corporation.

#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha2.h>
#include <crypto/sha1.h>
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
#include <linux/highmem.h>
#include <asm/io.h>
#include <crypto/internal/hash.h>
#include <linux/string.h>

/* SHA Registers */
#define HASH_DATA_IN            (sha_base + 0x000)
#define HASH_CTR_STS            (sha_base + 0x004)
#define HASH_CFG                (sha_base + 0x008)
#define HASH_VER                (sha_base + 0x00C)
#define SHA512_DATA_IN          (sha_base + 0x010)
#define SHA512_CTR_STS          (sha_base + 0x014)
#define SHA512_CMD              (sha_base + 0x018)
#define SHA512_DATA_OUT         (sha_base + 0x01C)
#define HASH_DIG_H(i)           (sha_base + 0x020 + (4 * i))

/*---------------------------------------------------------------------------------------------------------*/
/* HASH_CTR_STS fields                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define HASH_CTR_STS_SHA_RST	BIT(2)
#define	HASH_CTR_STS_SHA_BUSY	BIT(1)
#define	HASH_CTR_STS_SHA_EN	BIT(0)

/*---------------------------------------------------------------------------------------------------------*/
/* HASH_CFG fields                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define	HASH_CFG_SHA1_SHA2	BIT(0)

/*---------------------------------------------------------------------------------------------------------*/
/* SHA512_CMD fields                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SHA512_CMD_SHA512_RD    BIT(0)
#define SHA512_CMD_SHA512_WR    BIT(1)
#define SHA512_CMD_SHA512_ROUND BIT(2)
#define SHA512_CMD_SHA512_384   BIT(3)
#define SHA512_CMD_SHA512_LOAD  BIT(4)

//#define SHA_DEBUG_MODULE
#ifdef SHA_DEBUG_MODULE
#define sha_print(fmt, args...)      printk(fmt, ##args)
#else
#define sha_print(fmt, args...)     (void)0
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

/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*                                                  TYPES & DEFINITIONS                                    */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
#define SHA256_BLOCK_LENGTH             (512/8)
#define SHA512_BLOCK_LENGTH             (1024/8)
#define SHA1_HASH_LENGTH                (160/8)
#define SHA256_HASH_LENGTH              (256/8)
#define SHA384_HASH_LENGTH              (384/8)
#define SHA512_HASH_LENGTH              (512/8)
#define SHA_HASH_LENGTH_MAX             SHA512_HASH_LENGTH
#define SHA_GET_BLOCK_LENGTH(shaType)   ((shaType == SHA_TYPE_SHA256 || shaType == SHA_TYPE_SHA1) ? SHA256_BLOCK_LENGTH : SHA512_BLOCK_LENGTH)
#define SHA_GET_HASH_LENGTH(shaType)	((shaType == SHA_TYPE_SHA256) ? SHA256_HASH_LENGTH :   \
				((shaType == SHA_TYPE_SHA1)   ? SHA1_HASH_LENGTH    :   \
				((shaType == SHA_TYPE_SHA384) ? SHA384_HASH_LENGTH  :   \
				SHA512_HASH_LENGTH)))
/* TYPES & DEFINITIONS */
#define NPCM_SHA_TIMEOUT			1000
#define SHA_DATA_LAST_BYTE			0x80
#define SHA256_MSG_PADDING_LEN_SIZE_BYTES	8
#define SHA512_MSG_PADDING_LEN_SIZE_BYTES	16
#define SHA_MSG_PADDING_LEN_SIZE_BYTES(shaType)   ((shaType == SHA_TYPE_SHA256 || shaType == SHA_TYPE_SHA1) ? SHA256_MSG_PADDING_LEN_SIZE_BYTES : SHA512_MSG_PADDING_LEN_SIZE_BYTES)

#define SHA_SET_TYPE(type)              SET_REG_FIELD(HASH_CFG, HASH_CFG_SHA1_SHA2, type)
#define SHA_BUFF_POS(shaType, length)	(length & (SHA_GET_BLOCK_LENGTH(shaType) - 1))
#define SHA_BUFF_FREE(shaType, length)	(SHA_GET_BLOCK_LENGTH(shaType) - SHA_BUFF_POS(shaType, length))

#define NUVOTON_ALIGNMENT	4
#define HASH_DIG_H_NUM		8
/*---------------------------------------------------------------------------------------------------------*/
/* SHA type                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum {
	SHA_TYPE_SHA256 = 0, /*do not change - match SHA arch spec */
	SHA_TYPE_SHA1,
	SHA_TYPE_SHA384,
	SHA_TYPE_SHA512,
	SHA_TYPE_NUM
} SHA_TYPE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* SHA instance struct handler                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
static void __iomem *sha_base;

typedef struct SHA_HANDLE_T {
	u32		hv[SHA512_BLOCK_LENGTH / sizeof(u32)];
	u32		length0;
	u32		length1;
	u32		block[SHA512_BLOCK_LENGTH / sizeof(u32)];
	SHA_TYPE_T	shaType;
	bool		active;
	bool		internalRound;
} SHA_HANDLE_T;

struct npcm_sha_desc {
	SHA_HANDLE_T shaHandle;
};

void SHA_type(SHA_HANDLE_T *handlePtr)
{
	u8 reg_type;

	reg_type = ioread8(HASH_CFG);
	if (handlePtr->shaType == SHA_TYPE_SHA1)
		iowrite8(reg_type | HASH_CFG_SHA1_SHA2, HASH_CFG);
	if (handlePtr->shaType == SHA_TYPE_SHA256)
		iowrite8(reg_type & ~HASH_CFG_SHA1_SHA2, HASH_CFG);
}

void SHA_Reset(SHA_TYPE_T shaType)
{
	u8 reg_reset;

	if (shaType == SHA_TYPE_SHA1 || shaType == SHA_TYPE_SHA256) {
		reg_reset = ioread8(HASH_CTR_STS);
		iowrite8(reg_reset | HASH_CTR_STS_SHA_RST, HASH_CTR_STS);
	} else {
		reg_reset = ioread8(SHA512_CTR_STS);
		iowrite8(reg_reset | HASH_CTR_STS_SHA_RST, SHA512_CTR_STS);
	}
}

void SHA_Power(bool sha_en, SHA_TYPE_T shaType)
{
	u8 reg_en;

	if (shaType == SHA_TYPE_SHA1 || shaType == SHA_TYPE_SHA256) {
		reg_en = ioread8(HASH_CTR_STS);
		if (sha_en)
			iowrite8(reg_en | HASH_CTR_STS_SHA_EN, HASH_CTR_STS);
		else
			iowrite8(reg_en & ~HASH_CTR_STS_SHA_EN, HASH_CTR_STS);
	} else {
		reg_en = ioread8(SHA512_CTR_STS);
		if (sha_en)
			iowrite8(reg_en | HASH_CTR_STS_SHA_EN, SHA512_CTR_STS);
		else
			iowrite8(reg_en & ~HASH_CTR_STS_SHA_EN, SHA512_CTR_STS);
	}
}

static void SHA_FlushLocalBuffer_l(SHA_HANDLE_T *handlePtr, const u32 *buff)
{
	unsigned int i;

	if (handlePtr->shaType == SHA_TYPE_SHA1 ||
	    handlePtr->shaType == SHA_TYPE_SHA256) {
		for (i = 0; i < (SHA256_BLOCK_LENGTH / sizeof(u32)); i++)
			iowrite32(buff[i], HASH_DATA_IN);
	} else {
		u8 sha512_cmd = 0;

		if (handlePtr->shaType == SHA_TYPE_SHA512)
			sha512_cmd = SHA512_CMD_SHA512_384;
		if (handlePtr->internalRound)
			sha512_cmd |= SHA512_CMD_SHA512_ROUND;
		sha512_cmd |= SHA512_CMD_SHA512_WR;


		iowrite32(sha512_cmd, SHA512_CMD);
		for (i = 0; i < (SHA256_BLOCK_LENGTH / sizeof(u16)); i++)
			iowrite32(__le32_to_cpu(buff[i]), SHA512_DATA_IN);
	}

	handlePtr->internalRound = true;
}

static int SHA_BusyWait_l(SHA_HANDLE_T *handlePtr)
{
	u8 ctr_reg;
	int sha_count = 0;

	if (handlePtr->shaType == SHA_TYPE_SHA1 ||
	    handlePtr->shaType == SHA_TYPE_SHA256) {
		do {
			ctr_reg = ioread8(HASH_CTR_STS);
			ctr_reg &= HASH_CTR_STS_SHA_BUSY;
			sha_count++;
		} while ((sha_count < NPCM_SHA_TIMEOUT) && (ctr_reg));
	} else {
		do {
			ctr_reg = ioread8(SHA512_CTR_STS);
			ctr_reg &= HASH_CTR_STS_SHA_BUSY;
			sha_count++;
		} while ((sha_count < NPCM_SHA_TIMEOUT) && (ctr_reg));
	}

	if (sha_count >= NPCM_SHA_TIMEOUT)
		return -ETIMEDOUT;

	return 0;
}

static void SHA_GetShaDigest_l(u8 *hashDigest, SHA_TYPE_T shaType,
			       bool isLastRead)
{
	int  i;
	u8   len;
	u32 *dest = (u32 *)(void *)hashDigest;

	if (shaType == SHA_TYPE_SHA256 || shaType == SHA_TYPE_SHA1) {
		/*-------------------------------------------------------------------------------------------------*/
		/* Copy Bytes from SHA module to given buffer                                                      */
		/*-------------------------------------------------------------------------------------------------*/
		len = SHA_GET_HASH_LENGTH(shaType) / sizeof(u32);

		/*-------------------------------------------------------------------------------------------------*/
		/* len is positive unsigned byte, therefor loop should end                                         */
		/*-------------------------------------------------------------------------------------------------*/
		for (i = 0; i < len; i++)
			dest[i] = ioread32(HASH_DIG_H(i));
	} else {
		u32 x;
		u8 sha512_cmd = 0;

		/*-------------------------------------------------------------------------------------------------*/
		/* Enable digest read                                                                              */
		/*-------------------------------------------------------------------------------------------------*/
		if (shaType == SHA_TYPE_SHA512)
			sha512_cmd = SHA512_CMD_SHA512_384;
		sha512_cmd |= SHA512_CMD_SHA512_RD;

		iowrite32(sha512_cmd, SHA512_CMD);

		/*-------------------------------------------------------------------------------------------------*/
		/* Copy Bytes from SHA module to given buffer                                                      */
		/*-------------------------------------------------------------------------------------------------*/
		len = (isLastRead ? SHA_GET_HASH_LENGTH(shaType) : SHA_HASH_LENGTH_MAX) / sizeof(u32);

		/*-------------------------------------------------------------------------------------------------*/
		/* len is positive unsigned byte, therefor loop should end                                         */
		/*-------------------------------------------------------------------------------------------------*/
		for (i = 0; i < len; i++) {
			x = ioread32(SHA512_DATA_OUT);
			dest[i] = __le32_to_cpu(x);
		}
	}
}

static void SHA_SetShaDigest_l(SHA_HANDLE_T *handlePtr)
{
	int    i;
	u8   len;

	if (handlePtr->shaType == SHA_TYPE_SHA1 ||
	    handlePtr->shaType == SHA_TYPE_SHA256) {
		/*-------------------------------------------------------------------------------------------------*/
		/* Copy Bytes from given buffer to SHA module                                                      */
		/*-------------------------------------------------------------------------------------------------*/
		len = SHA_GET_HASH_LENGTH(handlePtr->shaType) / sizeof(u32);

		/*-------------------------------------------------------------------------------------------------*/
		/* len is positive unsigned byte, therefor loop should end                                         */
		/*-------------------------------------------------------------------------------------------------*/
		for (i = 0; i < len; i++)
			iowrite32(handlePtr->hv[i], HASH_DIG_H(i));
	} else {
		u8 cmd = ioread8(SHA512_CMD);
		/*-------------------------------------------------------------------------------------------------*/
		/* Enable digest write                                                                             */
		/*-------------------------------------------------------------------------------------------------*/
		iowrite8(cmd | SHA512_CMD_SHA512_LOAD, SHA512_CMD);

		/*-------------------------------------------------------------------------------------------------*/
		/* Copy Bytes from given buffer to SHA module                                                      */
		/*-------------------------------------------------------------------------------------------------*/
		len = SHA_HASH_LENGTH_MAX / sizeof(u32);

		/*-------------------------------------------------------------------------------------------------*/
		/* len is positive unsigned byte, therefor loop should end                                         */
		/*-------------------------------------------------------------------------------------------------*/
		for (i = 0; i < len; i++)
			iowrite32(__le32_to_cpu(handlePtr->hv[i]), SHA512_DATA_IN);


		/*-------------------------------------------------------------------------------------------------*/
		/* Disable digest write                                                                            */
		/*-------------------------------------------------------------------------------------------------*/
		iowrite8(cmd & ~SHA512_CMD_SHA512_LOAD, SHA512_CMD);
	}
}

static void SHA_SetBlock_l(const u8 *data, u32 len, u16 position, u32 *block)
{
	u8 *dest = (u8 *)block;

	memcpy(dest + position, data, len);
}

static void SHA_ClearBlock_l(u16 len, u16 position, u32 *block)
{
	u8 *dest = (u8 *)block;

	memset(dest + position, 0, len);
}

static void SHA_SetLength32_l(const SHA_HANDLE_T *handlePtr, u32 *block)
{
	u16 *secrunBufferSwappedPtr = (u16 *)(void *)(block);
	int length = (SHA_GET_BLOCK_LENGTH(handlePtr->shaType) / sizeof(u16));

	secrunBufferSwappedPtr[length - 1] = (u16)((handlePtr->length0 << 3) << 8) | ((u16)(handlePtr->length0 << 3) >> 8);
	secrunBufferSwappedPtr[length - 2] = (u16)((handlePtr->length0 >> (16 - 3)) >> 8) | ((u16)(handlePtr->length0 >> (16 - 3)) << 8);
	secrunBufferSwappedPtr[length - 3] = (u16)((handlePtr->length1 << 3) << 8) | ((u16)(handlePtr->length1 << 3) >> 8);
	secrunBufferSwappedPtr[length - 4] = (u16)((handlePtr->length1 >> (16 - 3)) >> 8) | ((u16)(handlePtr->length1 >> (16 - 3)) << 8);
}

int SHA_Start(SHA_HANDLE_T *handlePtr, SHA_TYPE_T shaType)
{
	/*-----------------------------------------------------------------------------------------------------*/
	/* Initialize handle                                                                                   */
	/*-----------------------------------------------------------------------------------------------------*/
	handlePtr->length0          = 0;
	handlePtr->length1          = 0;
	handlePtr->shaType          = shaType;
	handlePtr->active           = true;
	handlePtr->internalRound    = false;

	/*-----------------------------------------------------------------------------------------------------*/
	/* Init block with zeros                                                                               */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_ClearBlock_l(SHA_GET_BLOCK_LENGTH(shaType), 0, handlePtr->block);

	return 0;
}

int SHA_Update(SHA_HANDLE_T *handlePtr, const u8 *buffer, u32 len)
{
	u8   *unhashedBuffer;
	u32  bytesToCopy;
	u32  blockLength;
	u16  unhashedBytes; /* number of unhashed bytes from last update*/
	bool enoughData;    /* TRUE if we have enough data for at least one SHA digest */

	if (!buffer)
		return 0;

	sha_print_hex_dump("\t\t\t\t\t*NPCM-SHA: SHA_Update\n", (unsigned char *)buffer, len);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Get block length and the number of unhashed bytes which remained from last update                   */
	/*-----------------------------------------------------------------------------------------------------*/
	blockLength   = SHA_GET_BLOCK_LENGTH(handlePtr->shaType);
	unhashedBytes = SHA_BUFF_POS(handlePtr->shaType, handlePtr->length0);

	/*-----------------------------------------------------------------------------------------------------*/
	/* calculate if there is enough data for at least one SHA digest                                       */
	/*-----------------------------------------------------------------------------------------------------*/
	enoughData = ((unhashedBytes + len) >= blockLength);

	if (enoughData) {
		/*-------------------------------------------------------------------------------------------------*/
		/* There is enough data for at least one SHA digest so we re-activate the SHA HW before we use it. */
		/* 1 - Set SHA type                                                                                */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_type(handlePtr);

		/*-------------------------------------------------------------------------------------------------*/
		/* 2 - Reset SHA hardware (an initial HASH value is automatically inserted by hardware)            */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_Reset(handlePtr->shaType);

		/*-------------------------------------------------------------------------------------------------*/
		/* 3 - Enable SHA module                                                                           */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_Power(true, handlePtr->shaType);

		if (handlePtr->internalRound) {
			/*---------------------------------------------------------------------------------------------*/
			/* Write SHA latest digest into SHA module only if module already hashed data                  */
			/*---------------------------------------------------------------------------------------------*/
			SHA_SetShaDigest_l(handlePtr);
		}
	}

	/*-----------------------------------------------------------------------------------------------------*/
	/* Update size of hashed bytes (and handle overflow)                                                   */
	/*-----------------------------------------------------------------------------------------------------*/
	handlePtr->length0 += len;
	if ((handlePtr->length0) < len)
		handlePtr->length1++;

	/*-----------------------------------------------------------------------------------------------------*/
	/* Handle unhashed bytes from previous update                                                          */
	/*-----------------------------------------------------------------------------------------------------*/
	if (unhashedBytes > 0) {
		/*-------------------------------------------------------------------------------------------------*/
		/* Copy data to the "unhashed bytes buffer" (handlePtr->block), moving as much bytes as possible   */
		/*-------------------------------------------------------------------------------------------------*/
		bytesToCopy = min(len, blockLength - unhashedBytes);
		unhashedBuffer = (u8 *)handlePtr->block;
		memcpy(&unhashedBuffer[unhashedBytes], buffer, bytesToCopy);

		if (enoughData) {
			/*---------------------------------------------------------------------------------------------*/
			/* We have enough data (more then a block size) ,so we can Hash the current block              */
			/*---------------------------------------------------------------------------------------------*/
			SHA_FlushLocalBuffer_l(handlePtr, (u32 *)unhashedBuffer);

			/*---------------------------------------------------------------------------------------------*/
			/* Wait till SHA is not busy                                                                   */
			/*---------------------------------------------------------------------------------------------*/
			SHA_BusyWait_l(handlePtr);
		}

		/*-------------------------------------------------------------------------------------------------*/
		/* Update buffer and length                                                                        */
		/*-------------------------------------------------------------------------------------------------*/
		buffer += bytesToCopy;
		len -= bytesToCopy;
	}

	/*-----------------------------------------------------------------------------------------------------*/
	/* Handle the rest of the buffer                                                                       */
	/*-----------------------------------------------------------------------------------------------------*/
	while (len >= blockLength) {
		/*-------------------------------------------------------------------------------------------------*/
		/* Hash the current block                                                                          */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_FlushLocalBuffer_l(handlePtr, (u32 *)(void *)buffer);

		/*-------------------------------------------------------------------------------------------------*/
		/* Update length of data left to digest                                                            */
		/*-------------------------------------------------------------------------------------------------*/
		len -= blockLength;

		/*-------------------------------------------------------------------------------------------------*/
		/* Update given buffer pointer                                                                     */
		/*-------------------------------------------------------------------------------------------------*/
		buffer += blockLength;

		/*-------------------------------------------------------------------------------------------------*/
		/* Wait till SHA is not busy                                                                       */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_BusyWait_l(handlePtr);
	}

	if (len) {
		/*-------------------------------------------------------------------------------------------------*/
		/* Copy lefover (unhashed) bytes from given buffer to handlePtr->block for next update/finish      */
		/*-------------------------------------------------------------------------------------------------*/
		unhashedBuffer = (u8 *)handlePtr->block;
		memcpy(unhashedBuffer, buffer, len);
	}

	if (enoughData) {
		/*-------------------------------------------------------------------------------------------------*/
		/* Save SHA current digest                                                                         */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_GetShaDigest_l((u8 *)handlePtr->hv, handlePtr->shaType, false);
	}

	/*-----------------------------------------------------------------------------------------------------*/
	/* Reset SHA hardware (the result may be a SSP)                                                        */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_Reset(handlePtr->shaType);

	return 0;
}

int SHA_Finish(SHA_HANDLE_T *handlePtr, u8 *hashDigest)
{
	const u8 lastbyte = SHA_DATA_LAST_BYTE;
	u16      pos;

	SHA_type(handlePtr);
	SHA_Reset(handlePtr->shaType);
	SHA_Power(true, handlePtr->shaType);

	if (handlePtr->internalRound)
		SHA_SetShaDigest_l(handlePtr);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Finish off the current buffer with the SHA spec'ed padding                                          */
	/*-----------------------------------------------------------------------------------------------------*/
	pos = SHA_BUFF_POS(handlePtr->shaType, handlePtr->length0);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Set data last byte as in SHA algorithm spec                                                         */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_SetBlock_l(&lastbyte, 1, pos++, handlePtr->block);

	/*-----------------------------------------------------------------------------------------------------*/
	/* If the remainder of data is longer then one block                                                   */
	/*-----------------------------------------------------------------------------------------------------*/
	if (pos > (SHA_GET_BLOCK_LENGTH(handlePtr->shaType) - SHA_MSG_PADDING_LEN_SIZE_BYTES(handlePtr->shaType))) {
		/*-------------------------------------------------------------------------------------------------*/
		/* The message length will be in the next block                                                    */
		/* Pad the rest of the last block with 0's                                                         */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_ClearBlock_l((SHA_GET_BLOCK_LENGTH(handlePtr->shaType) - pos), pos, handlePtr->block);

		/*-------------------------------------------------------------------------------------------------*/
		/* Hash the current block                                                                          */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_FlushLocalBuffer_l(handlePtr, handlePtr->block);

		/*-------------------------------------------------------------------------------------------------*/
		/* Wait till SHA is not busy                                                                       */
		/*-------------------------------------------------------------------------------------------------*/
		SHA_BusyWait_l(handlePtr);

		pos = 0;
	}

	/*-----------------------------------------------------------------------------------------------------*/
	/* Pad the rest of the last block with 0's except for the last 4 bytes                                 */
	/* Notice: (SHA160/SHA256)/(SHA384/512) supports message size bit length of 64b/128b , but driver      */
	/* implementation uses 32b length field and therefor implementation limit the message size to be up to */
	/* 32b                                                                                                 */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_ClearBlock_l((SHA_GET_BLOCK_LENGTH(handlePtr->shaType) - (8 - 3)) - pos, pos, handlePtr->block);

	/*-----------------------------------------------------------------------------------------------------*/
	/* The last 4 bytes are set to the bit-length of the message in big-endian form                        */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_SetLength32_l(handlePtr, handlePtr->block);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Hash all that, and save the hash for the caller                                                     */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_FlushLocalBuffer_l(handlePtr, handlePtr->block);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Wait till SHA is not busy                                                                           */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_BusyWait_l(handlePtr);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Save SHA final digest into given buffer                                                             */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_GetShaDigest_l(hashDigest, handlePtr->shaType, true);

	/*-----------------------------------------------------------------------------------------------------*/
	/* Reset SHA hardware (the result may be a SSP)                                                        */
	/*-----------------------------------------------------------------------------------------------------*/
	SHA_Reset(handlePtr->shaType);

	sha_print_hex_dump("\t\t\t*NPCM-SHA: SHA_Finish\n", hashDigest, SHA_GET_HASH_LENGTH(handlePtr->shaType));

	/*-----------------------------------------------------------------------------------------------------*/
	/* Free handle                                                                                         */
	/*-----------------------------------------------------------------------------------------------------*/
	handlePtr->active = false;

	return 0;
}

static int npcm_sha1_init(struct ahash_request *req)
{
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);

	sha_print("\t\t\t* SHA1 init\n");
	SHA_Start(&(dctx->shaHandle), SHA_TYPE_SHA1);

	return 0;
}

static int npcm_sha256_init(struct ahash_request *req)
{
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);

	sha_print("\t\t\t* SHA256 init\n");
	SHA_Start(&(dctx->shaHandle), SHA_TYPE_SHA256);

	return 0;
}

static int npcm_sha384_init(struct ahash_request *req)
{
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);

	sha_print("\t\t\t* SHA384 init\n");
	SHA_Start(&(dctx->shaHandle), SHA_TYPE_SHA384);

	return 0;
}

static int npcm_sha512_init(struct ahash_request *req)
{
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);

	sha_print("\t\t\t* SHA512 init\n");
	SHA_Start(&(dctx->shaHandle), SHA_TYPE_SHA512);

	return 0;
}

static int npcm_sha_export(struct ahash_request *req, void *out)
{
	const struct npcm_sha_desc *dctx = ahash_request_ctx(req);

	memcpy(out, dctx, sizeof(*dctx));
	return 0;
}

static int npcm_sha_import(struct ahash_request *req, const void *in)
{
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);

	memcpy(dctx, in, sizeof(*dctx));
	return 0;
}

static int npcm_hash_walk_next(struct crypto_hash_walk *walk)
{
	unsigned int alignmask = walk->alignmask;
	unsigned int offset = walk->offset;
	unsigned int nbytes = min(walk->entrylen,
				  ((unsigned int)(PAGE_SIZE)) - offset);

	walk->data = kmap_atomic(walk->pg);
	walk->data += offset;

	if (offset & alignmask) {
		unsigned int unaligned = alignmask + 1 - (offset & alignmask);

		if (nbytes > unaligned)
			nbytes = unaligned;
	}

	walk->entrylen -= nbytes;
	return nbytes;
}

static int npcm_hash_walk_new_entry(struct crypto_hash_walk *walk)
{
	struct scatterlist *sg;

	sg = walk->sg;
	walk->offset = sg->offset;
	walk->pg = sg_page(walk->sg) + (walk->offset >> PAGE_SHIFT);
	walk->offset = offset_in_page(walk->offset);
	walk->entrylen = sg->length;

	if (walk->entrylen > walk->total)
		walk->entrylen = walk->total;
	walk->total -= walk->entrylen;

	return npcm_hash_walk_next(walk);
}

static int npcm_crypto_ahash_walk_first(struct ahash_request *req,
					struct crypto_hash_walk *walk)
{
	walk->total = req->nbytes;

	if (!walk->total) {
		walk->entrylen = 0;
		return 0;
	}

	walk->alignmask = crypto_ahash_alignmask(crypto_ahash_reqtfm(req));
	walk->sg = req->src;
	walk->flags = req->base.flags & CRYPTO_TFM_REQ_MASK;
	walk->flags |= CRYPTO_ALG_ASYNC;

	BUILD_BUG_ON(CRYPTO_TFM_REQ_MASK & CRYPTO_ALG_ASYNC);

	return npcm_hash_walk_new_entry(walk);
}

static inline int npcm_crypto_ahash_walk_done(struct crypto_hash_walk *walk,
					      int err)
{
	return crypto_hash_walk_done(walk, err);
}

static int npcm_sha_update(struct ahash_request *req)
{
	int ret = 0;
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);
	struct crypto_hash_walk walk;
	unsigned int nbytes = req->nbytes;
	u8  *src_paddr;

	nbytes = npcm_crypto_ahash_walk_first(req, &walk);

	while (nbytes) {

		src_paddr = (u8 *)phys_to_virt(page_to_phys(walk.pg) + walk.offset);

		ret = SHA_Update(&(dctx->shaHandle),  src_paddr, nbytes);
		if (ret < 0)
			return ret;
		nbytes = npcm_crypto_ahash_walk_done(&walk, ret);
	}

	sha_print("\t\t\t* SHA update done, returning %d\n", ret);
	return ret;
}

static int npcm_sha_final(struct ahash_request *req)
{
	int ret = 0;
	struct npcm_sha_desc *dctx = ahash_request_ctx(req);
	u8 *out = req->result;

	sha_print("\n\t\t\t* SHA final\n");

	ret = SHA_Finish(&(dctx->shaHandle), out);

	sha_print("\t\t\t* SHA finish done, returning %d\n", ret);
	return ret;
}

static int npcm_sha1_finup(struct ahash_request *req)
{
	int ret = 0;

	sha_print("\t\t* SHA1 finup\n");

	npcm_sha1_init(req);

	ret = npcm_sha_update(req);
	if (ret != 0) {
		printk("\t\t\t* SHA1 finup: update fail, ret = %d\n", ret);
		return ret;
	}
	ret = npcm_sha_final(req);
	if (ret != 0) {
		printk("\t\t\t* SHA1 finup: final fail, ret = %d\n", ret);
		return ret;
	}
	return ret;
}

static int npcm_sha256_finup(struct ahash_request *req)
{
	int ret = 0;

	sha_print("\t\t* SHA256 finup\n");

	npcm_sha256_init(req);

	ret = npcm_sha_update(req);
	if (ret != 0) {
		printk("\t\t\t* SHA256 finup: update fail, ret = %d\n", ret);
		return ret;
	}
	ret = npcm_sha_final(req);
	if (ret != 0) {
		printk("\t\t\t* SHA256 finup: final fail, ret = %d\n", ret);
		return ret;
	}
	return ret;
}

static int npcm_sha384_finup(struct ahash_request *req)
{
	int ret = 0;

	sha_print("\t\t* SHA384 finup\n");

	npcm_sha384_init(req);

	ret = npcm_sha_update(req);
	if (ret != 0) {
		printk("\t\t\t* SHA384 finup: update fail, ret = %d\n", ret);
		return ret;
	}
	ret = npcm_sha_final(req);
	if (ret != 0) {
		printk("\t\t\t* SHA384 finup: final fail, ret = %d\n", ret);
		return ret;
	}
	return ret;
}

static int npcm_sha512_finup(struct ahash_request *req)
{
	int ret = 0;

	sha_print("\t\t* SHA512 finup\n");

	npcm_sha512_init(req);

	ret = npcm_sha_update(req);
	if (ret != 0) {
		printk("\t\t\t* SHA512 finup: update fail, ret = %d\n", ret);
		return ret;
	}
	ret = npcm_sha_final(req);
	if (ret != 0) {
		printk("\t\t\t* SHA512 finup: final fail, ret = %d\n", ret);
		return ret;
	}
	return ret;
}

static int npcm_sha_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT);

	return 0;
}

static struct ahash_alg sha_algs[4] = {
	{
		.init	= npcm_sha1_init,
		.update	= npcm_sha_update,
		.final	= npcm_sha_final,
		.finup	= npcm_sha1_finup,
		.digest	= npcm_sha1_finup,
		.export = npcm_sha_export,
		.import = npcm_sha_import,

		.halg.digestsize = SHA1_DIGEST_SIZE,
		.halg.statesize  = sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
		.halg.base	= {
			.cra_name		= "sha1",
			.cra_driver_name	= "nuvoton_sha",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA1_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
			.cra_alignmask		= 0,
			.cra_module		= THIS_MODULE,
			.cra_init		= npcm_sha_cra_init
		}
	},
	{
		.init	= npcm_sha256_init,
		.update	= npcm_sha_update,
		.final	= npcm_sha_final,
		.finup	= npcm_sha256_finup,
		.digest	= npcm_sha256_finup,
		.export = npcm_sha_export,
		.import = npcm_sha_import,

		.halg.digestsize = SHA256_DIGEST_SIZE,
		.halg.statesize  = sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
		.halg.base	= {
			.cra_name		= "sha256",
			.cra_driver_name	= "nuvoton_sha",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA256_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
			.cra_alignmask		= 0,
			.cra_module		= THIS_MODULE,
			.cra_init		= npcm_sha_cra_init
		}
	},
	{
		.init	= npcm_sha384_init,
		.update	= npcm_sha_update,
		.final	= npcm_sha_final,
		.finup	= npcm_sha384_finup,
		.digest	= npcm_sha384_finup,
		.export = npcm_sha_export,
		.import = npcm_sha_import,

		.halg.digestsize = SHA384_DIGEST_SIZE,
		.halg.statesize  = sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
		.halg.base	= {
			.cra_name		= "sha384",
			.cra_driver_name	= "nuvoton_sha",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA384_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
			.cra_alignmask		= 0,
			.cra_module		= THIS_MODULE,
			.cra_init		= npcm_sha_cra_init
		}
	},
	{
		.init	= npcm_sha512_init,
		.update	= npcm_sha_update,
		.final	= npcm_sha_final,
		.finup	= npcm_sha512_finup,
		.digest	= npcm_sha512_finup,
		.export = npcm_sha_export,
		.import = npcm_sha_import,

		.halg.digestsize = SHA512_DIGEST_SIZE,
		.halg.statesize  = sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
		.halg.base	= {
			.cra_name		= "sha512",
			.cra_driver_name	= "nuvoton_sha",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
			.cra_blocksize		= SHA512_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct npcm_sha_desc) + NUVOTON_ALIGNMENT,
			.cra_alignmask		= 0,
			.cra_module		= THIS_MODULE,
			.cra_init		= npcm_sha_cra_init
		}
	}
};

static int npcm_sha_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	int sha_alg_support;
	int ret, i, j;

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
	if (!sha_base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	sha_alg_support = 2; //support sha1 and sha256
	if (of_device_is_compatible(np, "nuvoton,npcm845-sha"))
		sha_alg_support = 4; //support sha1, sha256, sha384 and sha512


	for (i = 0; i < sha_alg_support; i++) {
		ret = crypto_register_ahash(&(sha_algs[i]));
		if (ret)
			goto err_sha_algs;
	}

	printk("NPCM: SHA module is ready\n");

	return 0;

err_sha_algs:
	printk("\t\t\t* SHA register algo%d fail\n", i);
	for (j = 0; j < i; j++)
		crypto_unregister_ahash(&(sha_algs[j]));
	iounmap(sha_base);
	sha_base = NULL;
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_region:
	printk("\t\tNPCM: SHA module load fail .  Error %d\n", ret);

	return ret;
}

static int __exit npcm_sha_remove(struct platform_device *pdev)
{
	struct resource *res = dev_get_drvdata(&pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	SHA_TYPE_T shaType = SHA_TYPE_SHA256;
	int sha_alg_support;
	int i;

	sha_alg_support = 2; //support sha1 and sha256
	if (of_device_is_compatible(np, "nuvoton,npcm845-sha")) {
		sha_alg_support = 4; //support sha1, sha256, sha384 and sha512
		shaType = SHA_TYPE_SHA512;
	}

	for (i = 0; i < sha_alg_support; i++)
		crypto_unregister_ahash(&(sha_algs[i]));

	SHA_Power(false, shaType);
	iounmap(sha_base);
	release_mem_region(res->start, resource_size(res));
	sha_base = NULL;

	printk(KERN_NOTICE  "NPCM-SHA: remove: stop using Nuvoton npcm SHA module.\n");

	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:npcm_sha");

static const struct of_device_id sha_dt_id[] = {
	{ .compatible = "nuvoton,npcm750-sha",  },
	{ .compatible = "nuvoton,npcm845-sha",  },
	{ },
};
MODULE_DEVICE_TABLE(of, sha_dt_id);

static struct platform_driver npcm_sha_driver = {
	.probe		= npcm_sha_probe,
	.remove		= npcm_sha_remove,
	.shutdown	= NULL,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "npcm_sha",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(sha_dt_id),
	}
};

MODULE_ALIAS_CRYPTO("nuvoton-sha");

module_platform_driver(npcm_sha_driver);
MODULE_DESCRIPTION("Nuvoton Technologies SHA HW acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Tali Perry - Nuvoton Technologies");


