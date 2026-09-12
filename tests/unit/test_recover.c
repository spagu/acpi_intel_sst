/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side test for the DSP self-recovery policy in
 * src/sst_recover_policy.h (issue #51: the DSP refuses stream
 * allocations after a cold boot; the driver reinitializes it, but
 * never in a tight loop).
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sst_recover_policy.h"

#define HZ		1000
#define MIN_TICKS	(SST_RECOVER_MIN_SECS * HZ)

static int tests_run;
static int tests_failed;

#define CHECK(cond, msg) do {						\
	tests_run++;							\
	if (!(cond)) {							\
		tests_failed++;						\
		fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__,	\
		    __LINE__);						\
	} else {							\
		printf("ok   %s\n", (msg));				\
	}								\
} while (0)

static void
test_which_errors_recover(void)
{
	CHECK(sst_recover_wanted(3),
	    "DSP 'out of resources' (the issue #51 reply) wants recovery");
	CHECK(sst_recover_wanted(1) && sst_recover_wanted(7),
	    "every catpt reply status 1-7 wants recovery");
	CHECK(sst_recover_wanted(60),
	    "ETIMEDOUT (DSP did not answer) wants recovery");
	CHECK(!sst_recover_wanted(0), "success does not want recovery");
	CHECK(!sst_recover_wanted(12) && !sst_recover_wanted(22),
	    "host errors ENOMEM/EINVAL do not want recovery");
	CHECK(!sst_recover_wanted(-1), "negative codes do not want recovery");
}

static void
test_first_attempt_is_immediate(void)
{
	struct sst_recover_state rs = { 0, 0 };

	CHECK(sst_recover_allowed(&rs, 5, MIN_TICKS),
	    "the first recovery is allowed right away");
	CHECK(rs.count == 1 && rs.last_ticks == 5,
	    "an allowed attempt is recorded");
}

static void
test_rate_limit(void)
{
	struct sst_recover_state rs = { 0, 0 };

	sst_recover_allowed(&rs, 1000, MIN_TICKS);
	CHECK(!sst_recover_allowed(&rs, 1000 + MIN_TICKS - 1, MIN_TICKS),
	    "a second attempt just inside the window is refused");
	CHECK(rs.count == 1, "a refused attempt is not counted");
	CHECK(sst_recover_allowed(&rs, 1000 + MIN_TICKS, MIN_TICKS),
	    "an attempt exactly at the window edge is allowed");
	CHECK(rs.count == 2 && rs.last_ticks == 1000 + MIN_TICKS,
	    "the second attempt restarts the window");
}

static void
test_tick_wraparound(void)
{
	/* Last attempt 100 ticks before the signed counter wraps */
	struct sst_recover_state rs = { 1, INT32_MAX - 100 };

	CHECK(!sst_recover_allowed(&rs, INT32_MIN + 100, MIN_TICKS),
	    "200 ticks across the wrap are still inside the window");
	CHECK(sst_recover_allowed(&rs, INT32_MIN + MIN_TICKS, MIN_TICKS),
	    "a full window across the wrap is allowed (no signed overflow)");
}

int
main(void)
{
	test_which_errors_recover();
	test_first_attempt_is_immediate();
	test_rate_limit();
	test_tick_wraparound();

	printf("\n%d/%d checks passed\n", tests_run - tests_failed,
	    tests_run);

	return (tests_failed == 0 ? 0 : 1);
}
