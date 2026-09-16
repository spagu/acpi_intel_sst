/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side test for the deferred trigger request state in
 * src/sst_trig_state.h (issue #48: channel_trigger must not sleep, so
 * the trigger body runs in a taskqueue and requests are coalesced).
 *
 * The header is shared with the driver, so this exercises the exact
 * code sst_pcm.c runs, driven by a small model of the worker loop.
 *
 * Copyright (c) 2026 Tradik Limited
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdio.h>

#include "sst_trig_state.h"

/* Mirrors sound(4)'s PCMTRIG_* values; only distinctness matters here */
#define T_START	1
#define T_STOP	2
#define T_ABORT	3

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

/*
 * Model of sst_pcm_trig_task(): consume requests until none is left,
 * recording the order in which the body would have run.
 */
static int
run_worker(struct sst_trig_state *ts, int *ran, int max)
{
	int go, n = 0;

	while (sst_trig_state_take(ts, &go)) {
		if (n < max)
			ran[n] = go;
		n++;
	}
	return (n);
}

static void
test_idle_worker_has_nothing(void)
{
	struct sst_trig_state ts = { 0, false };
	int go = -1;

	CHECK(sst_trig_state_take(&ts, &go) == false,
	    "fresh state has no pending request");
	CHECK(go == -1, "take() leaves *go untouched when nothing pends");
}

static void
test_single_request_runs_once(void)
{
	struct sst_trig_state ts = { 0, false };
	int ran[4];

	sst_trig_state_request(&ts, T_START);
	CHECK(run_worker(&ts, ran, 4) == 1 && ran[0] == T_START,
	    "one START request runs the body exactly once with START");
	CHECK(run_worker(&ts, ran, 4) == 0,
	    "a consumed request is not run again");
}

static void
test_start_then_stop_coalesces(void)
{
	struct sst_trig_state ts = { 0, false };
	int ran[4];

	/* App opens and closes before the worker gets scheduled */
	sst_trig_state_request(&ts, T_START);
	sst_trig_state_request(&ts, T_STOP);
	CHECK(run_worker(&ts, ran, 4) == 1 && ran[0] == T_STOP,
	    "START immediately followed by STOP runs only STOP "
	    "(last requested state wins)");
}

static void
test_request_during_body_is_not_lost(void)
{
	struct sst_trig_state ts = { 0, false };
	int go;

	sst_trig_state_request(&ts, T_START);
	CHECK(sst_trig_state_take(&ts, &go) && go == T_START,
	    "worker takes START and starts running the body");

	/* STOP arrives while the START body is still executing */
	sst_trig_state_request(&ts, T_ABORT);
	CHECK(sst_trig_state_take(&ts, &go) && go == T_ABORT,
	    "worker loop picks up the ABORT that arrived mid-body");
	CHECK(sst_trig_state_take(&ts, &go) == false,
	    "nothing is left after the second pass");
}

static void
test_repeated_same_request(void)
{
	struct sst_trig_state ts = { 0, false };
	int ran[4];

	sst_trig_state_request(&ts, T_STOP);
	sst_trig_state_request(&ts, T_STOP);
	sst_trig_state_request(&ts, T_STOP);
	CHECK(run_worker(&ts, ran, 4) == 1 && ran[0] == T_STOP,
	    "three identical STOP requests collapse into one body run");
}

int
main(void)
{
	test_idle_worker_has_nothing();
	test_single_request_runs_once();
	test_start_then_stop_coalesces();
	test_request_during_body_is_not_lost();
	test_repeated_same_request();

	printf("\n%d/%d checks passed\n", tests_run - tests_failed,
	    tests_run);

	return (tests_failed == 0 ? 0 : 1);
}
