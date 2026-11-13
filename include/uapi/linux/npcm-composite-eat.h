/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_NPCM_COMPOSITE_EAT_H
#define _UAPI_LINUX_NPCM_COMPOSITE_EAT_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define NPCM_COMPOSITE_EAT_ABI_VERSION 1
#define NPCM_COMPOSITE_EAT_IOCTL_MAGIC 0xE6

enum npcm_composite_eat_status {
	NPCM_COMPOSITE_EAT_STATUS_OK = 0,
	NPCM_COMPOSITE_EAT_STATUS_BAD_VERSION = 1,
	NPCM_COMPOSITE_EAT_STATUS_BAD_REQUEST = 2,
	NPCM_COMPOSITE_EAT_STATUS_TOO_MANY_RECORDS = 3,
	NPCM_COMPOSITE_EAT_STATUS_REQUEST_TOO_LARGE = 4,
	NPCM_COMPOSITE_EAT_STATUS_RESPONSE_TOO_SMALL = 5,
	NPCM_COMPOSITE_EAT_STATUS_ADDRESS_INVALID = 6,
	NPCM_COMPOSITE_EAT_STATUS_SIGN_FAILED = 7,
	NPCM_COMPOSITE_EAT_STATUS_BUSY = 8,
	NPCM_COMPOSITE_EAT_STATUS_INTERNAL = 9,
	NPCM_COMPOSITE_EAT_STATUS_UNSUPPORTED = 10,
};

/**
 * struct npcm_composite_eat_io - Generate a signed Composite EAT.
 * @abi_version: Must be %NPCM_COMPOSITE_EAT_ABI_VERSION.
 * @reserved: Must be zero.
 * @req_ptr: Userspace pointer to the encoded request.
 * @req_len: Request length in bytes.
 * @reserved_req: Must be zero.
 * @resp_ptr: Userspace pointer to the response buffer.
 * @resp_cap: Response buffer capacity in bytes.
 * @resp_len: Actual response length, or required length when @status is
 *            %NPCM_COMPOSITE_EAT_STATUS_RESPONSE_TOO_SMALL.
 * @status: Firmware status from &enum npcm_composite_eat_status. A valid
 *          nonzero firmware status is returned with an ioctl return value of
 *          zero so userspace can inspect @status and @resp_len.
 * @reserved_resp: Must be zero.
 */
struct npcm_composite_eat_io {
	__u32 abi_version;
	__u32 reserved;
	__aligned_u64 req_ptr;
	__u32 req_len;
	__u32 reserved_req;
	__aligned_u64 resp_ptr;
	__u32 resp_cap;
	__u32 resp_len;
	__u32 status;
	__u32 reserved_resp;
};

#define NPCM_COMPOSITE_EAT_GENERATE \
	_IOWR(NPCM_COMPOSITE_EAT_IOCTL_MAGIC, 1, struct npcm_composite_eat_io)

#endif /* _UAPI_LINUX_NPCM_COMPOSITE_EAT_H */
