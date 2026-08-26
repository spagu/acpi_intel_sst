/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Intel SST Jack Detection Driver
 * Target: Intel Haswell/Broadwell-U with Realtek ALC3263
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/callout.h>

#include <machine/bus.h>

#include "acpi_intel_sst.h"
#include "sst_jack.h"

/*
 * Check single jack state
 *
 * Presence comes from the codec pin-sense verb.  The previous
 * implementation read/wrote SHIM CSR bits 8-11 as if they were GPIOs;
 * those bits are DSP STALL and SSP power-down controls, so jack
 * activity stalled the DSP core.
 *
 * Must be called WITHOUT j->lock held: codec access is I2C and takes
 * the codec lock plus millisecond-scale DELAYs.
 */
static enum sst_jack_state
sst_jack_check_one(struct sst_softc *sc, struct sst_jack_info *jack)
{
	bool present;

	if (sst_codec_pin_present(sc, jack->nid, &present) != 0)
		return (jack->state);	/* Codec unavailable: keep state */

	/* Apply inversion if needed */
	if (jack->inverted)
		present = !present;

	return (present ? SST_JACK_INSERTED : SST_JACK_REMOVED);
}

/*
 * Update jack state with debouncing
 */
static bool
sst_jack_update_one(struct sst_jack_info *jack, enum sst_jack_state new_state)
{
	bool changed = false;

	if (new_state == jack->state) {
		/* State stable, reset debounce */
		jack->debounce_cnt = 0;
		jack->pending_state = jack->state;
	} else if (new_state == jack->pending_state) {
		/* Same pending state, increment debounce */
		jack->debounce_cnt++;
		if (jack->debounce_cnt >= SST_JACK_DEBOUNCE_COUNT) {
			/* State change confirmed */
			jack->state = new_state;
			jack->debounce_cnt = 0;
			changed = true;
		}
	} else {
		/* New pending state */
		jack->pending_state = new_state;
		jack->debounce_cnt = 1;
	}

	return (changed);
}

/*
 * Handle jack state change
 */
static void
sst_jack_handle_change(struct sst_softc *sc, struct sst_jack_info *jack)
{
	struct sst_jack *j = &sc->jack;
	sst_jack_callback_t cb;
	void *cbArg;
	enum sst_jack_state state;
	uint32_t type;
	const char *typeName;
	const char *stateName;

	/*
	 * Snapshot everything the routing/callback needs under the lock;
	 * the codec call below must run unlocked.
	 */
	mtx_lock(&j->lock);
	type = jack->type;
	state = jack->state;
	cb = j->callback;
	cbArg = j->callback_arg;

	switch (type) {
	case SST_JACK_HEADPHONE:
		typeName = "Headphone";
		if (state == SST_JACK_INSERTED)
			j->hp_insertions++;
		break;
	case SST_JACK_MICROPHONE:
		typeName = "Microphone";
		if (state == SST_JACK_INSERTED)
			j->mic_insertions++;
		break;
	default:
		typeName = "Unknown";
		break;
	}
	mtx_unlock(&j->lock);

	stateName = (state == SST_JACK_INSERTED) ? "inserted" : "removed";

	sst_dbg(sc, SST_DBG_LIFE, "Jack: %s %s\n", typeName, stateName);

	/* Handle audio routing through the codec output amplifiers */
	if (type == SST_JACK_HEADPHONE)
		sst_codec_set_hp_route(sc, state == SST_JACK_INSERTED);

	/* Call user callback */
	if (cb != NULL)
		cb(cbArg, type, state);
}

/*
 * Polling timer callback
 */
static void
sst_jack_poll_timer(void *arg)
{
	struct sst_softc *sc = arg;
	struct sst_jack *j = &sc->jack;

	/*
	 * Runs with j->lock already held (callout_init_mtx).  Do NOT
	 * lock again and do NOT touch the codec here: reading pin sense
	 * is an I2C transaction with millisecond DELAYs, which must not
	 * run in softclock context.  Defer to a taskqueue thread.
	 */
	mtx_assert(&j->lock, MA_OWNED);

	if (!j->enabled || !j->polling)
		return;

	j->poll_count++;
	taskqueue_enqueue(taskqueue_thread, &j->poll_task);

	callout_reset(&j->poll_callout,
	    hz * SST_JACK_POLL_INTERVAL_MS / 1000,
	    sst_jack_poll_timer, sc);
}

/*
 * Deferred poll worker - runs in a taskqueue thread, may block
 */
static void
sst_jack_poll_task(void *arg, int pending __unused)
{
	struct sst_softc *sc = arg;
	struct sst_jack *j = &sc->jack;
	enum sst_jack_state hp_state, mic_state;
	bool hp_changed, mic_changed;

	mtx_lock(&j->lock);
	if (!j->enabled || !j->polling) {
		mtx_unlock(&j->lock);
		return;
	}
	mtx_unlock(&j->lock);

	/* Codec reads happen without j->lock held */
	hp_state = sst_jack_check_one(sc, &j->headphone);
	mic_state = sst_jack_check_one(sc, &j->microphone);

	mtx_lock(&j->lock);
	hp_changed = sst_jack_update_one(&j->headphone, hp_state);
	mic_changed = sst_jack_update_one(&j->microphone, mic_state);
	mtx_unlock(&j->lock);

	if (hp_changed)
		sst_jack_handle_change(sc, &j->headphone);
	if (mic_changed)
		sst_jack_handle_change(sc, &j->microphone);
}

/*
 * Initialize jack detection subsystem
 */
int
sst_jack_init(struct sst_softc *sc)
{
	struct sst_jack *j = &sc->jack;

	memset(j, 0, sizeof(*j));
	j->sc = sc;

	mtx_init(&j->lock, "sst_jack", NULL, MTX_DEF);
	callout_init_mtx(&j->poll_callout, &j->lock, 0);
	TASK_INIT(&j->poll_task, 0, sst_jack_poll_task, sc);

	/* Configure headphone jack */
	j->headphone.type = SST_JACK_HEADPHONE;
	j->headphone.state = SST_JACK_REMOVED;
	j->headphone.nid = SST_JACK_NID_HP;
	j->headphone.inverted = false;	/* Pin sense: 1 = jack present */
	j->headphone.debounce_cnt = 0;

	/* Configure microphone jack */
	j->microphone.type = SST_JACK_MICROPHONE;
	j->microphone.state = SST_JACK_REMOVED;
	j->microphone.nid = SST_JACK_NID_MIC;
	j->microphone.inverted = false;
	j->microphone.debounce_cnt = 0;

	/* Default to polling method */
	j->method = SST_JACK_METHOD_POLL;
	j->polling = false;
	j->enabled = false;

	/* Statistics */
	j->hp_insertions = 0;
	j->mic_insertions = 0;
	j->poll_count = 0;

	j->initialized = true;

	sst_dbg(sc, SST_DBG_LIFE, "Jack detection initialized (polling mode)\n");

	return (0);
}

/*
 * Cleanup jack detection subsystem
 */
void
sst_jack_fini(struct sst_softc *sc)
{
	struct sst_jack *j = &sc->jack;

	if (!j->initialized)
		return;

	sst_jack_disable(sc);

	callout_drain(&j->poll_callout);
	taskqueue_drain(taskqueue_thread, &j->poll_task);
	mtx_destroy(&j->lock);

	j->initialized = false;
}

/*
 * Enable jack detection
 */
int
sst_jack_enable(struct sst_softc *sc)
{
	struct sst_jack *j = &sc->jack;

	if (!j->initialized)
		return (ENXIO);

	mtx_lock(&j->lock);

	if (j->enabled) {
		mtx_unlock(&j->lock);
		return (0);
	}

	j->enabled = true;

	/* Start polling if using poll method */
	if (j->method == SST_JACK_METHOD_POLL) {
		j->polling = true;
		callout_reset(&j->poll_callout,
		    hz * SST_JACK_POLL_INTERVAL_MS / 1000,
		    sst_jack_poll_timer, sc);
	}

	mtx_unlock(&j->lock);

	/* Do initial detection */
	sst_jack_poll(sc);

	sst_dbg(sc, SST_DBG_OPS, "Jack detection enabled\n");

	return (0);
}

/*
 * Disable jack detection
 */
void
sst_jack_disable(struct sst_softc *sc)
{
	struct sst_jack *j = &sc->jack;

	if (!j->initialized)
		return;

	mtx_lock(&j->lock);

	if (!j->enabled) {
		mtx_unlock(&j->lock);
		return;
	}

	j->enabled = false;
	j->polling = false;

	mtx_unlock(&j->lock);

	/* Stop polling timer, then wait for any in-flight worker */
	callout_drain(&j->poll_callout);
	taskqueue_drain(taskqueue_thread, &j->poll_task);

	sst_dbg(sc, SST_DBG_OPS, "Jack detection disabled\n");
}

/*
 * Get current jack state
 */
enum sst_jack_state
sst_jack_get_state(struct sst_softc *sc, uint32_t jackType)
{
	struct sst_jack *j = &sc->jack;
	enum sst_jack_state state = SST_JACK_REMOVED;

	mtx_lock(&j->lock);

	if (jackType & SST_JACK_HEADPHONE)
		state = j->headphone.state;
	else if (jackType & SST_JACK_MICROPHONE)
		state = j->microphone.state;

	mtx_unlock(&j->lock);

	return (state);
}

/*
 * Check if jack is inserted
 */
bool
sst_jack_is_inserted(struct sst_softc *sc, uint32_t jackType)
{
	return (sst_jack_get_state(sc, jackType) == SST_JACK_INSERTED);
}

/*
 * Register callback for jack events
 */
int
sst_jack_set_callback(struct sst_softc *sc, sst_jack_callback_t cb, void *arg)
{
	struct sst_jack *j = &sc->jack;

	mtx_lock(&j->lock);
	j->callback = cb;
	j->callback_arg = arg;
	mtx_unlock(&j->lock);

	return (0);
}

/*
 * Manual poll (for debugging or forced update)
 */
void
sst_jack_poll(struct sst_softc *sc)
{
	struct sst_jack *j = &sc->jack;
	enum sst_jack_state hp_state, mic_state;
	bool hp_changed = false, mic_changed = false;

	if (!j->initialized)
		return;

	/* Codec (I2C) reads must not run under the jack lock */
	hp_state = sst_jack_check_one(sc, &j->headphone);
	mic_state = sst_jack_check_one(sc, &j->microphone);

	mtx_lock(&j->lock);

	if (hp_state != j->headphone.state) {
		j->headphone.state = hp_state;
		hp_changed = true;
	}

	if (mic_state != j->microphone.state) {
		j->microphone.state = mic_state;
		mic_changed = true;
	}

	mtx_unlock(&j->lock);

	if (hp_changed)
		sst_jack_handle_change(sc, &j->headphone);
	if (mic_changed)
		sst_jack_handle_change(sc, &j->microphone);
}

/*
 * Interrupt handler for GPIO-based jack detection
 */
void
sst_jack_intr(struct sst_softc *sc)
{
	struct sst_jack *j = &sc->jack;

	if (!j->initialized || !j->enabled)
		return;

	if (j->method != SST_JACK_METHOD_IRQ)
		return;

	/* Trigger immediate poll on interrupt */
	sst_jack_poll(sc);
}

/*
 * Sysctl handlers
 */
static int
sst_jack_sysctl_hp_state(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int state;

	state = sst_jack_is_inserted(sc, SST_JACK_HEADPHONE) ? 1 : 0;
	return (sysctl_handle_int(oidp, &state, 0, req));
}

static int
sst_jack_sysctl_mic_state(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int state;

	state = sst_jack_is_inserted(sc, SST_JACK_MICROPHONE) ? 1 : 0;
	return (sysctl_handle_int(oidp, &state, 0, req));
}

static int
sst_jack_sysctl_enabled(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	struct sst_jack *j = &sc->jack;
	int enabled, error;

	enabled = j->enabled ? 1 : 0;
	error = sysctl_handle_int(oidp, &enabled, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (enabled)
		sst_jack_enable(sc);
	else
		sst_jack_disable(sc);

	return (0);
}

/*
 * Initialize sysctl interface
 */
int
sst_jack_sysctl_init(struct sst_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct sysctl_oid *jack_tree;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);

	jack_tree = SYSCTL_ADD_NODE(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "jack", CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "Jack detection");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(jack_tree), OID_AUTO,
	    "headphone", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sst_jack_sysctl_hp_state, "I",
	    "Headphone jack state (0=removed, 1=inserted)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(jack_tree), OID_AUTO,
	    "microphone", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, sst_jack_sysctl_mic_state, "I",
	    "Microphone jack state (0=removed, 1=inserted)");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(jack_tree), OID_AUTO,
	    "enabled", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_jack_sysctl_enabled, "I",
	    "Jack detection enabled");

	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(jack_tree), OID_AUTO,
	    "hp_insertions", CTLFLAG_RD, &sc->jack.hp_insertions, 0,
	    "Headphone insertion count");

	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(jack_tree), OID_AUTO,
	    "mic_insertions", CTLFLAG_RD, &sc->jack.mic_insertions, 0,
	    "Microphone insertion count");

	SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(jack_tree), OID_AUTO,
	    "poll_count", CTLFLAG_RD, &sc->jack.poll_count, 0,
	    "Poll count");

	return (0);
}
