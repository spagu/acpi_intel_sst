/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Rate limiting against the kernel tick counter.
 *
 * SET_VOLUME goes to the DSP over IPC, and the firmware handles IPC
 * synchronously: a slider dragged across its range can starve the DMA
 * engine and kill the stream.  So volume updates are throttled, and the
 * throttle has to survive what `ticks` actually does.
 *
 * FreeBSD starts `ticks` just short of overflow on purpose, to shake out
 * code that assumes it counts up from zero: it is large and positive at
 * boot and wraps to large and negative within the first minute.  The
 * obvious test,
 *
 *	(ticks - last) >= interval
 *
 * is then false for every wrapped value once `last` is stale or was
 * never set - a difference near -2.1e9 - and the throttle closes for
 * good.  That is what made the mixer look dead: the slider moved, the
 * driver stored the value, and no SET_VOLUME was ever sent again.
 *
 * A negative difference means "so long ago the counter wrapped", which
 * is the opposite of "too soon", so it must pass.
 *
 * Kept free of kernel types so tests/unit/test_rate_limit.c can drive
 * the same code the driver runs, wrap included.
 *
 * Copyright (c) 2026 Tradik Limited
 * All rights reserved.
 */

#ifndef _SST_RATE_LIMIT_H_
#define _SST_RATE_LIMIT_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdbool.h>
#endif

/*
 * True when `interval` ticks have passed since `last`, or when the
 * difference is negative because the counter wrapped past it.
 *
 * All three are plain ints, as `ticks` is, so the subtraction wraps the
 * same way the kernel's does.
 */
static inline bool
sst_rate_elapsed(int now, int last, int interval)
{
	int dt = now - last;

	return (dt < 0 || dt >= interval);
}

#endif /* _SST_RATE_LIMIT_H_ */
