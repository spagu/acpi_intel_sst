/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DSP self-recovery policy (pure C, no kernel dependencies).
 *
 * When the DSP rejects a stream allocation the driver reinitializes
 * it (issue #51).  Reinitialization takes about a second and tears
 * down every stream, so it must not be retried in a tight loop when
 * the DSP is genuinely broken.  This header decides whether a new
 * attempt is due; it is included by src/sst_pcm.c and by the host-side
 * unit tests in tests/unit, which is why it must not pull in kernel
 * headers.
 *
 * Copyright (c) 2026 Tradik Limited
 * All rights reserved.
 */

#ifndef _SST_RECOVER_POLICY_H_
#define _SST_RECOVER_POLICY_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif

/* Minimum spacing between two recoveries, in seconds */
#define SST_RECOVER_MIN_SECS	30

struct sst_recover_state {
	uint32_t	count;		/* Recoveries attempted so far */
	int		last_ticks;	/* ticks at the last attempt */
};

/*
 * Errors that mean "the DSP answered, but refused" or "the DSP did not
 * answer at all".  Both are cured by a reinitialization, unlike host
 * side errors (ENOMEM, EINVAL, ...) which are not.  The reply codes
 * are the catpt IPC status values 1-7; 60 is ETIMEDOUT.
 */
static inline bool
sst_recover_wanted(int error)
{
	return ((error >= 1 && error <= 7) || error == 60);
}

/*
 * Decide whether a recovery may start now and, if so, record it.
 * The first attempt is always allowed; later ones need at least
 * min_ticks (SST_RECOVER_MIN_SECS * hz) since the previous one.
 * The tick counter is signed and wraps; the elapsed time is computed
 * in unsigned arithmetic so the wrap is harmless and never overflows.
 */
static inline bool
sst_recover_allowed(struct sst_recover_state *rs, int now_ticks,
    int min_ticks)
{
	uint32_t elapsed;

	elapsed = (uint32_t)now_ticks - (uint32_t)rs->last_ticks;
	if (rs->count > 0 && elapsed < (uint32_t)min_ticks)
		return (false);
	rs->count++;
	rs->last_ticks = now_ticks;
	return (true);
}

#endif /* _SST_RECOVER_POLICY_H_ */
