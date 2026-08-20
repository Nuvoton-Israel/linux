// SPDX-License-Identifier: GPL-2.0

#include <kunit/test.h>
#include <linux/module.h>
#include <linux/stddef.h>

#include "npcm-composite-eat-internal.h"

static void npcm_composite_eat_io_validation_test(struct kunit *test)
{
	struct npcm_composite_eat_io io = {
		.abi_version = NPCM_COMPOSITE_EAT_ABI_VERSION,
		.req_ptr = 1,
		.req_len = 1,
		.resp_ptr = 1,
		.resp_cap = 1,
	};

	KUNIT_EXPECT_TRUE(test, npcm_composite_eat_io_valid(&io));
	KUNIT_EXPECT_EQ(test, sizeof(io), 48UL);
	KUNIT_EXPECT_EQ(test, offsetof(struct npcm_composite_eat_io, req_ptr), 8UL);
	KUNIT_EXPECT_EQ(test, offsetof(struct npcm_composite_eat_io, resp_ptr), 24UL);
	KUNIT_EXPECT_EQ(test, offsetof(struct npcm_composite_eat_io, status), 40UL);
	KUNIT_EXPECT_EQ(test, _IOC_TYPE(NPCM_COMPOSITE_EAT_GENERATE), 0xE6U);
	KUNIT_EXPECT_EQ(test, _IOC_SIZE(NPCM_COMPOSITE_EAT_GENERATE), 48U);

	io.abi_version++;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.abi_version = NPCM_COMPOSITE_EAT_ABI_VERSION;
	io.reserved = 1;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.reserved = 0;
	io.reserved_req = 1;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.reserved_req = 0;
	io.reserved_resp = 1;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.reserved_resp = 0;
	io.req_ptr = 0;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.req_ptr = 1;
	io.resp_ptr = 0;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.resp_ptr = 1;
	io.req_len = 0;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.req_len = BMC_DIRECT_COMPOSITE_EAT_REQ_SIZE + 1;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.req_len = 1;
	io.resp_cap = 0;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
	io.resp_cap = BMC_DIRECT_COMPOSITE_EAT_RESP_SIZE + 1;
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_io_valid(&io));
}

static void npcm_composite_eat_resource_validation_test(struct kunit *test)
{
	u64 base = BMC_DIRECT_COMPOSITE_EAT_SHM_BASE;
	u64 size = BMC_DIRECT_COMPOSITE_EAT_SHM_SIZE;
	bool valid;

	valid = npcm_composite_eat_resource_valid(base, size);
	KUNIT_EXPECT_TRUE(test, valid);
	valid = npcm_composite_eat_resource_valid(base + 1, size);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = npcm_composite_eat_resource_valid(base, size - 1);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = npcm_composite_eat_resource_valid(base, size + 1);
	KUNIT_EXPECT_FALSE(test, valid);
}

static void npcm_composite_eat_response_validation_test(struct kunit *test)
{
	u32 resp_len;
	u32 status;
	int ret;

	status = NPCM_COMPOSITE_EAT_STATUS_OK;
	ret = npcm_composite_eat_response_valid(status, 1, 1);
	KUNIT_EXPECT_EQ(test, ret, 0);
	ret = npcm_composite_eat_response_valid(status, 0, 1);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);
	ret = npcm_composite_eat_response_valid(status, 2, 1);
	KUNIT_EXPECT_EQ(test, ret, -EMSGSIZE);
	status = NPCM_COMPOSITE_EAT_STATUS_RESPONSE_TOO_SMALL;
	ret = npcm_composite_eat_response_valid(status, 2, 1);
	KUNIT_EXPECT_EQ(test, ret, 0);
	ret = npcm_composite_eat_response_valid(status, 1, 1);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);
	resp_len = BMC_DIRECT_COMPOSITE_EAT_RESP_SIZE + 1;
	ret = npcm_composite_eat_response_valid(status, resp_len, 1);
	KUNIT_EXPECT_EQ(test, ret, 0);
	status = NPCM_COMPOSITE_EAT_STATUS_BAD_REQUEST;
	ret = npcm_composite_eat_response_valid(status, 1, 1);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);
	ret = npcm_composite_eat_response_valid(status, 0, 1);
	KUNIT_EXPECT_EQ(test, ret, 0);
	status = NPCM_COMPOSITE_EAT_STATUS_UNSUPPORTED + 1;
	ret = npcm_composite_eat_response_valid(status, 0, 1);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);
}

static void npcm_composite_eat_tx_result_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, npcm_composite_eat_tx_requires_quarantine(-ETIME));
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_tx_requires_quarantine(-ENOBUFS));
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_tx_requires_quarantine(0));
}

static void npcm_composite_eat_state_test(struct kunit *test)
{
	struct npcm_composite_eat_request_state state = {0};
	u32 request_id;

	KUNIT_ASSERT_EQ(test, npcm_composite_eat_state_begin(&state, &request_id), 0);
	KUNIT_EXPECT_EQ(test, request_id, 1U);
	KUNIT_EXPECT_EQ(test, state.state, NPCM_COMPOSITE_EAT_IN_FLIGHT);
	KUNIT_EXPECT_EQ(test, npcm_composite_eat_state_begin(&state, &request_id),
			-EBUSY);
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_state_response(&state, 2));
	KUNIT_EXPECT_TRUE(test, npcm_composite_eat_state_response(&state, 1));
	KUNIT_EXPECT_TRUE(test, npcm_composite_eat_state_accept(&state, 1));
	KUNIT_EXPECT_EQ(test, state.state, NPCM_COMPOSITE_EAT_IDLE);
}

static void npcm_composite_eat_timeout_test(struct kunit *test)
{
	struct npcm_composite_eat_request_state state = {0};
	u32 request_id;

	KUNIT_ASSERT_EQ(test, npcm_composite_eat_state_begin(&state, &request_id), 0);
	npcm_composite_eat_state_timeout(&state);
	KUNIT_EXPECT_EQ(test, state.state, NPCM_COMPOSITE_EAT_QUARANTINED);
	KUNIT_EXPECT_EQ(test, npcm_composite_eat_state_begin(&state, &request_id),
			-EBUSY);
	KUNIT_EXPECT_FALSE(test, npcm_composite_eat_state_response(&state, 2));
	KUNIT_EXPECT_EQ(test, state.state, NPCM_COMPOSITE_EAT_QUARANTINED);
	KUNIT_EXPECT_TRUE(test, npcm_composite_eat_state_response(&state, 1));
	KUNIT_EXPECT_EQ(test, state.state, NPCM_COMPOSITE_EAT_IDLE);
}

static void npcm_composite_eat_request_id_wrap_test(struct kunit *test)
{
	struct npcm_composite_eat_request_state state = {
		.next_request_id = U32_MAX,
	};
	u32 request_id;

	KUNIT_ASSERT_EQ(test, npcm_composite_eat_state_begin(&state, &request_id), 0);
	KUNIT_EXPECT_EQ(test, request_id, 1U);
}

static void npcm_composite_eat_state_reset_test(struct kunit *test)
{
	struct npcm_composite_eat_request_state state = {0};
	u32 request_id;

	KUNIT_ASSERT_EQ(test, npcm_composite_eat_state_begin(&state, &request_id), 0);
	npcm_composite_eat_state_timeout(&state);
	KUNIT_ASSERT_EQ(test, state.state, NPCM_COMPOSITE_EAT_QUARANTINED);
	npcm_composite_eat_state_reset(&state);
	KUNIT_EXPECT_EQ(test, state.state, NPCM_COMPOSITE_EAT_IDLE);
	KUNIT_EXPECT_EQ(test, state.active_request_id, 0U);
}

static struct kunit_case npcm_composite_eat_test_cases[] = {
	KUNIT_CASE(npcm_composite_eat_io_validation_test),
	KUNIT_CASE(npcm_composite_eat_resource_validation_test),
	KUNIT_CASE(npcm_composite_eat_response_validation_test),
	KUNIT_CASE(npcm_composite_eat_tx_result_test),
	KUNIT_CASE(npcm_composite_eat_state_test),
	KUNIT_CASE(npcm_composite_eat_timeout_test),
	KUNIT_CASE(npcm_composite_eat_request_id_wrap_test),
	KUNIT_CASE(npcm_composite_eat_state_reset_test),
	{}
};

static struct kunit_suite npcm_composite_eat_test_suite = {
	.name = "npcm-composite-eat",
	.test_cases = npcm_composite_eat_test_cases,
};

kunit_test_suite(npcm_composite_eat_test_suite);

MODULE_LICENSE("GPL");
