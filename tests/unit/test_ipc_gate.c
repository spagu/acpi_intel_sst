/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side test for the IPC sender gate in src/sst_ipc_gate.h.
 *
 * The driver used to serialize senders with a mutex held across
 * cv_timedwait(), so a sender slept owning a mutex and the machine
 * panicked ("sleeping thread holds sst_ipc_send") as soon as the volume
 * ramp task and the trigger task sent at the same time.  The gate is
 * the replacement, and this drives the exact code the driver runs with
 * a model of two senders meeting.
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdio.h>

#include "sst_ipc_gate.h"

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
 * Model of sst_ipc_send()'s entry: try, and on failure park on the cv
 * and retry when woken.  Returns the number of times it had to wait,
 * or -1 for the timeout path, where the command is dropped.
 */
static int
enter(struct sst_ipc_gate *g, int wakeups_available, bool timeout_after)
{
	int waited = 0;

	while (!sst_ipc_gate_try_enter(g)) {
		sst_ipc_gate_begin_wait(g);
		/* cv_timedwait() sleeps here with ipc.lock dropped */
		if (wakeups_available-- <= 0 && timeout_after) {
			sst_ipc_gate_end_wait(g);
			return (-1);
		}
		sst_ipc_gate_end_wait(g);
		waited++;
	}
	return (waited);
}

int
main(void)
{
	struct sst_ipc_gate g;
	bool signal_owed;

	sst_ipc_gate_init(&g);
	CHECK(!g.busy && g.waiters == 0, "a fresh gate is free");

	/* One sender: straight through, and nobody to wake on the way out */
	CHECK(enter(&g, 0, false) == 0, "an idle gate admits the first sender");
	CHECK(g.busy, "the gate is marked busy while a command is in flight");
	CHECK(!sst_ipc_gate_leave(&g), "leaving with no waiters owes no signal");
	CHECK(!g.busy, "the gate is free again after leaving");

	/*
	 * Two senders, the case that panicked the driver: the trigger
	 * task holds the gate while the ramp task arrives.
	 */
	sst_ipc_gate_init(&g);
	CHECK(sst_ipc_gate_try_enter(&g), "trigger task takes the gate");
	CHECK(!sst_ipc_gate_try_enter(&g), "ramp task is refused, not admitted");

	sst_ipc_gate_begin_wait(&g);
	CHECK(g.waiters == 1, "the parked sender is counted");
	signal_owed = sst_ipc_gate_leave(&g);
	CHECK(signal_owed, "leaving with a waiter owes a signal");
	sst_ipc_gate_end_wait(&g);
	CHECK(sst_ipc_gate_try_enter(&g), "the woken sender then gets in");
	CHECK(!sst_ipc_gate_leave(&g), "and owes nothing on its way out");

	/*
	 * A woken sender must re-check: two waiters, one wakeup, and the
	 * one that loses the race parks again instead of proceeding.
	 */
	sst_ipc_gate_init(&g);
	CHECK(sst_ipc_gate_try_enter(&g), "first sender holds the gate");
	sst_ipc_gate_begin_wait(&g);
	sst_ipc_gate_begin_wait(&g);
	CHECK(g.waiters == 2, "both late senders are counted");
	CHECK(sst_ipc_gate_leave(&g), "a signal is owed with two waiters");
	sst_ipc_gate_end_wait(&g);
	CHECK(sst_ipc_gate_try_enter(&g), "the woken sender takes the gate");
	CHECK(!sst_ipc_gate_try_enter(&g), "the other one still finds it busy");
	sst_ipc_gate_end_wait(&g);
	CHECK(sst_ipc_gate_leave(&g) == false,
	    "no signal owed once the last waiter gave up");

	/* The timeout path leaves no phantom waiter behind */
	sst_ipc_gate_init(&g);
	CHECK(sst_ipc_gate_try_enter(&g), "gate taken before the timeout test");
	CHECK(enter(&g, 0, true) == -1, "a sender that waits too long gives up");
	CHECK(g.waiters == 0, "a timed-out sender is no longer counted");
	CHECK(!sst_ipc_gate_leave(&g),
	    "so the holder owes no signal to a sender that left");

	/* An underflowing count would wrap: unsigned, so guard it */
	sst_ipc_gate_init(&g);
	sst_ipc_gate_end_wait(&g);
	CHECK(g.waiters == 0, "ending a wait that never began stays at zero");

	printf("\n%d tests, %d failed\n", tests_run, tests_failed);
	return (tests_failed == 0 ? 0 : 1);
}
