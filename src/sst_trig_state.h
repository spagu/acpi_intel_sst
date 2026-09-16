/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Deferred trigger request state (pure C, no kernel dependencies).
 *
 * sound(4) calls channel_trigger with channel mutexes held, so the
 * IPC-heavy trigger body runs later in a taskqueue thread.  Between
 * the request and the worker picking it up, further requests may
 * arrive; only the most recent one matters ("last requested state
 * wins"), so the state holds exactly one pending request.
 *
 * This header is included by src/sst_pcm.c and by the host-side unit
 * tests in tests/unit, which is why it must not pull in kernel headers.
 *
 * Copyright (c) 2026 Tradik Limited
 * All rights reserved.
 */

#ifndef _SST_TRIG_STATE_H_
#define _SST_TRIG_STATE_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdbool.h>
#endif

struct sst_trig_state {
	int	req;		/* Last requested PCMTRIG_* value */
	bool	pending;	/* req not yet consumed by the worker */
};

/*
 * Record a trigger request.  A request that has not been consumed yet
 * is replaced: a START immediately followed by a STOP leaves only the
 * STOP for the worker.
 */
static inline void
sst_trig_state_request(struct sst_trig_state *ts, int go)
{
	ts->req = go;
	ts->pending = true;
}

/*
 * Consume the pending request, if any.  Returns true and stores the
 * request in *go when there was one; the worker loops on this until it
 * returns false so a request that arrived while the body was running
 * is not lost.
 */
static inline bool
sst_trig_state_take(struct sst_trig_state *ts, int *go)
{
	if (!ts->pending)
		return (false);
	*go = ts->req;
	ts->pending = false;
	return (true);
}

#endif /* _SST_TRIG_STATE_H_ */
