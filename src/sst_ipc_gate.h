/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Serialization gate for IPC senders.
 *
 * Only one IPC transaction may be in flight at a time: sst_ipc_send()
 * writes the shared mailbox and the shared message context, then waits
 * for the DSP to reply.  That used to be enforced with a mutex held
 * across the whole send-wait-complete cycle, which is a bug: the wait
 * is a cv_timedwait(), so the sender sleeps while owning a mutex.  A
 * second sender blocking on that mutex makes priority propagation find
 * a sleeping owner and panic with
 *
 *     panic: sleeping thread holds sst_ipc_send
 *
 * (seen with the volume ramp task and the trigger task both sending).
 *
 * The gate replaces the mutex with a flag plus a condition variable,
 * both covered by ipc.lock, which the sender already drops while it
 * sleeps.  This header holds the decisions, without any kernel types,
 * so tests/unit/test_ipc_gate.c can drive the exact code the driver
 * runs.  Keep the two in sync: the test is the specification.
 *
 * Usage, all under ipc.lock:
 *
 *	while (!sst_ipc_gate_try_enter(&gate)) {
 *		sst_ipc_gate_begin_wait(&gate);
 *		error = cv_timedwait(&send_cv, &lock, timeout);
 *		sst_ipc_gate_end_wait(&gate);
 *		if (error == EWOULDBLOCK)
 *			return (EBUSY);
 *	}
 *	... transaction ...
 *	if (sst_ipc_gate_leave(&gate))
 *		cv_signal(&send_cv);
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_IPC_GATE_H_
#define _SST_IPC_GATE_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdbool.h>
#endif

struct sst_ipc_gate {
	bool		busy;		/* a transaction is in flight */
	unsigned int	waiters;	/* senders parked on the cv */
};

/*
 * Claim the gate.  Returns false when another sender holds it, in
 * which case the caller waits on the condition variable and retries:
 * a woken sender must re-check rather than assume it won the race.
 */
static inline bool
sst_ipc_gate_try_enter(struct sst_ipc_gate *g)
{
	if (g->busy)
		return (false);
	g->busy = true;
	return (true);
}

/* Registered before sleeping so the leaving sender knows to signal. */
static inline void
sst_ipc_gate_begin_wait(struct sst_ipc_gate *g)
{
	g->waiters++;
}

/* Counterpart of begin_wait, on both the woken and the timed-out path. */
static inline void
sst_ipc_gate_end_wait(struct sst_ipc_gate *g)
{
	if (g->waiters > 0)
		g->waiters--;
}

/*
 * Release the gate.  Returns true when someone is parked on the cv and
 * a wakeup is owed; signalling with no waiters is harmless but pointless,
 * and the count keeps that decision testable.
 */
static inline bool
sst_ipc_gate_leave(struct sst_ipc_gate *g)
{
	g->busy = false;
	return (g->waiters > 0);
}

/* Fresh gate: nothing in flight, nobody waiting. */
static inline void
sst_ipc_gate_init(struct sst_ipc_gate *g)
{
	g->busy = false;
	g->waiters = 0;
}

#endif /* _SST_IPC_GATE_H_ */
