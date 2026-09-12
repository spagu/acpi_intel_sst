/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side test for the failure-streak accounting in
 * src/sst_errstat.h (issue #49: the codec I2C paths printed every
 * failed attempt; the driver now prints once per streak).
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sst_errstat.h"

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
test_healthy_bus_is_silent(void)
{
	struct sst_errstat es = { 0, 0 };

	CHECK(sst_errstat_ok(&es) == 0,
	    "success on a healthy bus reports no ended streak");
	CHECK(es.total == 0 && es.streak == 0,
	    "success on a healthy bus leaves the counters at zero");
}

static void
test_only_first_failure_reports(void)
{
	struct sst_errstat es = { 0, 0 };
	int reported = 0, i;

	/* Jack detection polling a sleeping codec: 400 failures in a row */
	for (i = 0; i < 400; i++)
		if (sst_errstat_fail(&es))
			reported++;

	CHECK(reported == 1,
	    "400 consecutive failures ask the caller to print once");
	CHECK(es.total == 400, "cumulative total counts every failure");
	CHECK(es.streak == 400, "streak counts the consecutive failures");
}

static void
test_recovery_reports_streak_length(void)
{
	struct sst_errstat es = { 0, 0 };

	sst_errstat_fail(&es);
	sst_errstat_fail(&es);
	sst_errstat_fail(&es);
	CHECK(sst_errstat_ok(&es) == 3,
	    "first success after failures reports the streak length");
	CHECK(es.streak == 0 && es.total == 3,
	    "recovery clears the streak but keeps the total");
	CHECK(sst_errstat_ok(&es) == 0,
	    "the recovery is reported only once");
}

static void
test_new_streak_reports_again(void)
{
	struct sst_errstat es = { 0, 0 };

	sst_errstat_fail(&es);
	sst_errstat_ok(&es);
	CHECK(sst_errstat_fail(&es) == true,
	    "a failure after a recovery starts a new, reported streak");
	CHECK(sst_errstat_fail(&es) == false,
	    "the second failure of the new streak is quiet");
	CHECK(es.total == 3, "total spans both streaks");
}

static void
test_streak_saturates(void)
{
	struct sst_errstat es = { 0, UINT32_MAX };

	CHECK(sst_errstat_fail(&es) == false && es.streak == UINT32_MAX,
	    "a saturated streak neither wraps to zero nor re-reports");
}

int
main(void)
{
	test_healthy_bus_is_silent();
	test_only_first_failure_reports();
	test_recovery_reports_streak_length();
	test_new_streak_reports_again();
	test_streak_saturates();

	printf("\n%d/%d checks passed\n", tests_run - tests_failed,
	    tests_run);

	return (tests_failed == 0 ? 0 : 1);
}
