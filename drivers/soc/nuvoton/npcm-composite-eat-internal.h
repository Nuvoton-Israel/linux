/* SPDX-License-Identifier: GPL-2.0 */
#ifndef NPCM_COMPOSITE_EAT_INTERNAL_H
#define NPCM_COMPOSITE_EAT_INTERNAL_H

#include <linux/errno.h>
#include <linux/types.h>
#include <uapi/linux/npcm-composite-eat.h>

#define BMC_DIRECT_COMPOSITE_EAT_SHM_BASE 0x06201000ULL
#define BMC_DIRECT_COMPOSITE_EAT_REQ_SIZE 0x1000
#define BMC_DIRECT_COMPOSITE_EAT_RESP_SIZE 0x4000
#define BMC_DIRECT_COMPOSITE_EAT_SHM_SIZE \
	(BMC_DIRECT_COMPOSITE_EAT_REQ_SIZE + BMC_DIRECT_COMPOSITE_EAT_RESP_SIZE)

enum npcm_composite_eat_state {
	NPCM_COMPOSITE_EAT_IDLE,
	NPCM_COMPOSITE_EAT_IN_FLIGHT,
	NPCM_COMPOSITE_EAT_QUARANTINED,
};

struct npcm_composite_eat_request_state {
	enum npcm_composite_eat_state state;
	u32 next_request_id;
	u32 active_request_id;
};

static inline bool
npcm_composite_eat_io_valid(const struct npcm_composite_eat_io *io)
{
	return io->abi_version == NPCM_COMPOSITE_EAT_ABI_VERSION &&
		!io->reserved && !io->reserved_req && !io->reserved_resp &&
		io->req_ptr && io->resp_ptr && io->req_len && io->resp_cap &&
		io->req_len <= BMC_DIRECT_COMPOSITE_EAT_REQ_SIZE &&
		io->resp_cap <= BMC_DIRECT_COMPOSITE_EAT_RESP_SIZE;
}

static inline bool npcm_composite_eat_resource_valid(u64 start, u64 size)
{
	return start == BMC_DIRECT_COMPOSITE_EAT_SHM_BASE &&
		size == BMC_DIRECT_COMPOSITE_EAT_SHM_SIZE &&
		start + size - 1 <= U32_MAX;
}

static inline int
npcm_composite_eat_response_valid(u32 status, u32 resp_len, u32 resp_cap)
{
	if (status > NPCM_COMPOSITE_EAT_STATUS_UNSUPPORTED)
		return -EPROTO;

	if (status == NPCM_COMPOSITE_EAT_STATUS_RESPONSE_TOO_SMALL)
		return resp_len > resp_cap ? 0 : -EPROTO;

	if (status != NPCM_COMPOSITE_EAT_STATUS_OK)
		return resp_len ? -EPROTO : 0;

	if (!resp_len)
		return -EPROTO;

	if (resp_len > resp_cap || resp_len > BMC_DIRECT_COMPOSITE_EAT_RESP_SIZE)
		return -EMSGSIZE;

	return 0;
}

static inline bool npcm_composite_eat_tx_requires_quarantine(int ret)
{
	return ret == -ETIME;
}

static inline int
npcm_composite_eat_state_begin(struct npcm_composite_eat_request_state *state,
			       u32 *request_id)
{
	if (state->state != NPCM_COMPOSITE_EAT_IDLE)
		return -EBUSY;

	state->state = NPCM_COMPOSITE_EAT_IN_FLIGHT;
	state->next_request_id++;
	if (!state->next_request_id)
		state->next_request_id++;
	state->active_request_id = state->next_request_id;
	*request_id = state->active_request_id;
	return 0;
}

static inline void
npcm_composite_eat_state_reset(struct npcm_composite_eat_request_state *state)
{
	state->state = NPCM_COMPOSITE_EAT_IDLE;
	state->active_request_id = 0;
}

static inline void
npcm_composite_eat_state_timeout(struct npcm_composite_eat_request_state *state)
{
	if (state->state == NPCM_COMPOSITE_EAT_IN_FLIGHT)
		state->state = NPCM_COMPOSITE_EAT_QUARANTINED;
}

static inline bool
npcm_composite_eat_state_response(struct npcm_composite_eat_request_state *state,
				  u32 response_id)
{
	if (response_id != state->active_request_id)
		return false;

	if (state->state == NPCM_COMPOSITE_EAT_QUARANTINED)
		npcm_composite_eat_state_reset(state);
	else if (state->state != NPCM_COMPOSITE_EAT_IN_FLIGHT)
		return false;

	return true;
}

static inline bool
npcm_composite_eat_state_accept(struct npcm_composite_eat_request_state *state,
				u32 response_id)
{
	if (state->state != NPCM_COMPOSITE_EAT_IN_FLIGHT ||
	    response_id != state->active_request_id) {
		state->state = NPCM_COMPOSITE_EAT_QUARANTINED;
		return false;
	}

	npcm_composite_eat_state_reset(state);
	return true;
}

#endif /* NPCM_COMPOSITE_EAT_INTERNAL_H */
