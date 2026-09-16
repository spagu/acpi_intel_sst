/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side test for the tick rate limiter in src/sst_rate_limit.h.
 *
 * The driver throttled SET_VOLUME with `(ticks - last) >= interval`.
 * FreeBSD starts `ticks` just short of overflow, so that difference goes
 * large and negative within a minute of boot and the throttle closed for
 * good: the mixer moved and the DSP never heard about it. These cases
 * are the ones that mattered, wrap included.
 *
 * Copyright (c) 2026 Tradik Limited
 * All rights reserved.
 */

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

#include "sst_rate_limit.h"

#define INTERVAL	100		/* SST_VOL_RATE_TICKS in the driver */

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

int
main(void)
{
	int boot, wrapped;

	/* The ordinary case: too soon, then long enough */
	CHECK(!sst_rate_elapsed(1000, 950, INTERVAL),
	    "50 ticks after the last update is too soon");
	CHECK(sst_rate_elapsed(1050, 950, INTERVAL),
	    "exactly the interval passes");
	CHECK(sst_rate_elapsed(5000, 950, INTERVAL),
	    "well past the interval passes");
	CHECK(!sst_rate_elapsed(950, 950, INTERVAL),
	    "no time at all is too soon");

	/*
	 * Boot values. FreeBSD sets ticks near INT_MAX so it wraps early;
	 * the counter is large and positive for the first seconds.
	 */
	boot = INT_MAX - 1000;
	CHECK(!sst_rate_elapsed(boot + 10, boot, INTERVAL),
	    "throttling works before the wrap");
	CHECK(sst_rate_elapsed(boot + 200, boot, INTERVAL),
	    "the interval still passes before the wrap");

	/*
	 * Across the wrap: now is negative, last is a large positive left
	 * over from before it. Signed subtraction wraps with the counter,
	 * so the elapsed time is small and correct.
	 */
	wrapped = INT_MIN + 50;			/* 1098 ticks after boot */
	CHECK(!sst_rate_elapsed(wrapped, INT_MAX - 20, INTERVAL),
	    "71 ticks across the wrap is still too soon");
	CHECK(sst_rate_elapsed(INT_MIN + 200, INT_MAX - 20, INTERVAL),
	    "221 ticks across the wrap passes");

	/*
	 * The case that broke the mixer: `last` never set, or so stale the
	 * difference underflows. That is not "too soon" - it is forever
	 * ago, and it must pass.
	 */
	CHECK(sst_rate_elapsed(INT_MIN + 1000, 0, INTERVAL),
	    "a never-set timestamp does not lock the throttle");
	CHECK(sst_rate_elapsed(-2139594894, 0, INTERVAL),
	    "the difference seen in the trace passes");
	/*
	 * One tick past INT_MAX is INT_MIN in wrapping arithmetic, so this
	 * is "one tick later", not "forever ago" - and it must be refused.
	 * The check below stays clear of signed overflow by keeping the two
	 * values one apart.
	 */
	CHECK(!sst_rate_elapsed(INT_MIN, INT_MAX, INTERVAL),
	    "one tick across the wrap is too soon, not forever ago");

	printf("\n%d tests, %d failed\n", tests_run, tests_failed);
	return (tests_failed == 0 ? 0 : 1);
}
