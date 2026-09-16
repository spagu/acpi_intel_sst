/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Failure-streak accounting for repetitive bus transactions (pure C,
 * no kernel dependencies).
 *
 * The codec is polled over I2C every 250 ms by jack detection.  When
 * the bus is unhealthy every poll fails, and printing each failure
 * floods the console (issue #49).  The streak counter lets the caller
 * print one line when a failure streak begins and one when it ends,
 * while a cumulative total stays available through sysctl.
 *
 * Included by src/sst_codec.c and by the host-side unit tests in
 * tests/unit, which is why it must not pull in kernel headers.
 *
 * Copyright (c) 2026 Tradik Limited
 * All rights reserved.
 */

#ifndef _SST_ERRSTAT_H_
#define _SST_ERRSTAT_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif

struct sst_errstat {
	uint32_t	total;		/* Cumulative failures */
	uint32_t	streak;		/* Consecutive failures, 0 = healthy */
};

/*
 * Account for one failed transaction.  Returns true when this failure
 * starts a new streak, i.e. the caller should report it.
 */
static inline bool
sst_errstat_fail(struct sst_errstat *es)
{
	es->total++;
	if (es->streak != UINT32_MAX)
		es->streak++;
	return (es->streak == 1);
}

/*
 * Account for one successful transaction.  Returns the length of the
 * failure streak that just ended (0 when there was none), so the
 * caller can report the recovery once.
 */
static inline uint32_t
sst_errstat_ok(struct sst_errstat *es)
{
	uint32_t ended;

	ended = es->streak;
	es->streak = 0;
	return (ended);
}

#endif /* _SST_ERRSTAT_H_ */
