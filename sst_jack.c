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
 * Read GPIO pin state
 */
static int
sst_jack_gpio_read(struct sst_softc *sc, int gpio)
{
	uint32_t val;

	/*
	 * GPIO access via codec registers
	 * For Realtek ALC3263, GPIO is accessed through HDA verbs
	 * In SST mode, we use IPC to communicate with DSP which
	 * controls the codec
	 */

	/* Read GPIO data register from SHIM area */
	val = sst_shim_read(sc, SST_SHIM_CSR);

	/* Extract GPIO bit */
	return ((val >> (gpio + 8)) & 1);
}

/*
 * Write GPIO pin state
 */
static void
sst_jack_gpio_write(struct sst_softc *sc, int gpio, int value)
{
	uint32_t mask, val;

	mask = 1U << (gpio + 8);
	val = value ? mask : 0;

	sst_shim_update_bits(sc, SST_SHIM_CSR, mask, val);
}

/*
 * Check single jack state
 */
static enum sst_jack_state
sst_jack_check_one(struct sst_softc *sc, struct sst_jack_info *jack)
{
	int gpio_val;
	enum sst_jack_state new_state;

	gpio_val = sst_jack_gpio_read(sc, jack->gpio);

	/* Apply inversion if needed */
	if (jack->inverted)
		gpio_val = !gpio_val;

	new_state = gpio_val ? SST_JACK_INSERTED : SST_JACK_REMOVED;

	return (new_state);
}

/*
 * Update jack state with debouncing
 */
static bool
sst_jack_update_one(struct sst_softc *sc, struct sst_jack_info *jack)
{
	enum sst_jack_state new_state;
	bool changed = false;

	new_state = sst_jack_check_one(sc, jack);

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
	const char *typeName;
	const char *stateName;

	/* Get type name */
	switch (jack->type) {
	case SST_JACK_HEADPHONE:
		typeName = "Headphone";
		if (jack->state == SST_JACK_INSERTED)
			j->hp_insertions++;
		break;
	case SST_JACK_MICROPHONE:
		typeName = "Microphone";
		if (jack->state == SST_JACK_INSERTED)
			j->mic_insertions++;
		break;
	default:
		typeName = "Unknown";
		break;
	}

	/* Get state name */
	stateName = (jack->state == SST_JACK_INSERTED) ? "inserted" : "removed";

	device_printf(sc->dev, "Jack: %s %s\n", typeName, stateName);

	/* Handle audio routing */
	if (jack->type == SST_JACK_HEADPHONE) {
		if (jack->state == SST_JACK_INSERTED) {
			/* Mute speakers, unmute headphones */
			sst_jack_gpio_write(sc, SST_GPIO_SPEAKER_MUTE, 1);
			sst_jack_gpio_write(sc, SST_GPIO_HP_MUTE, 0);
		} else {
			/* Unmute speakers, mute headphones */
			sst_jack_gpio_write(sc, SST_GPIO_HP_MUTE, 1);
			sst_jack_gpio_write(sc, SST_GPIO_SPEAKER_MUTE, 0);
		}
	}

	/* Call user callback */
	if (j->callback != NULL) {
		j->callback(j->callback_arg, jack->type, jack->state);
	}
}

/*
 * Polling timer callback
 */
static void
sst_jack_poll_timer(void *arg)
{
	struct sst_softc *sc = arg;
	struct sst_jack *j = &sc->jack;
	bool hp_changed, mic_changed;

	mtx_lock(&j->lock);

	if (!j->enabled || !j->polling) {
		mtx_unlock(&j->lock);
		return;
	}

	j->poll_count++;

	/* Check headphone jack */
	hp_changed = sst_jack_update_one(sc, &j->headphone);

	/* Check microphone jack */
	mic_changed = sst_jack_update_one(sc, &j->microphone);

	mtx_unlock(&j->lock);

	/* Handle changes outside lock */
	if (hp_changed)
		sst_jack_handle_change(sc, &j->headphone);
	if (mic_changed)
		sst_jack_handle_change(sc, &j->microphone);

	/* Reschedule timer */
	mtx_lock(&j->lock);
	if (j->enabled && j->polling) {
		callout_reset(&j->poll_callout,
		    hz * SST_JACK_POLL_INTERVAL_MS / 1000,
		    sst_jack_poll_timer, sc);
	}
	mtx_unlock(&j->lock);
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

	/* Configure headphone jack */
	j->headphone.type = SST_JACK_HEADPHONE;
	j->headphone.state = SST_JACK_REMOVED;
	j->headphone.gpio = SST_GPIO_HP_DETECT;
	j->headphone.inverted = true;	/* Active low on most platforms */
	j->headphone.debounce_cnt = 0;

	/* Configure microphone jack */
	j->microphone.type = SST_JACK_MICROPHONE;
	j->microphone.state = SST_JACK_REMOVED;
	j->microphone.gpio = SST_GPIO_MIC_DETECT;
	j->microphone.inverted = true;
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

	device_printf(sc->dev, "Jack detection initialized (polling mode)\n");

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

	device_printf(sc->dev, "Jack detection enabled\n");

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

	/* Stop polling timer */
	callout_drain(&j->poll_callout);

	device_printf(sc->dev, "Jack detection disabled\n");
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

	mtx_lock(&j->lock);

	/* Direct read without debouncing */
	hp_state = sst_jack_check_one(sc, &j->headphone);
	mic_state = sst_jack_check_one(sc, &j->microphone);

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
