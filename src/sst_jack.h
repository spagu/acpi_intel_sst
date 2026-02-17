/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Intel SST Jack Detection Driver
 * Target: Intel Haswell/Broadwell-U with Realtek ALC3263
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_JACK_H_
#define _SST_JACK_H_

#include <sys/types.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/callout.h>

/*
 * Jack Types
 */
#define SST_JACK_HEADPHONE	(1 << 0)	/* Headphone jack */
#define SST_JACK_MICROPHONE	(1 << 1)	/* Microphone jack */
#define SST_JACK_HEADSET	(SST_JACK_HEADPHONE | SST_JACK_MICROPHONE)
#define SST_JACK_LINEOUT	(1 << 2)	/* Line out */
#define SST_JACK_LINEIN		(1 << 3)	/* Line in */

/*
 * Jack States
 */
enum sst_jack_state {
	SST_JACK_REMOVED = 0,
	SST_JACK_INSERTED,
	SST_JACK_DETECTING
};

/*
 * GPIO Configuration for Realtek ALC3263
 * GPIO pins vary by platform - these are for Dell XPS 13 9343
 */
#define SST_GPIO_HP_DETECT	0	/* Headphone detect GPIO */
#define SST_GPIO_MIC_DETECT	1	/* Microphone detect GPIO */
#define SST_GPIO_HP_MUTE	2	/* Headphone mute GPIO */
#define SST_GPIO_SPEAKER_MUTE	3	/* Speaker mute GPIO */

/*
 * GPIO Register Offsets (Realtek HDA-style)
 */
#define SST_GPIO_DATA		0x00	/* GPIO Data */
#define SST_GPIO_DIR		0x01	/* GPIO Direction (0=in, 1=out) */
#define SST_GPIO_WAKE		0x02	/* GPIO Wake Enable */
#define SST_GPIO_UNSOL		0x03	/* GPIO Unsolicited Response */
#define SST_GPIO_STICKY		0x04	/* GPIO Sticky Mask */

/*
 * Jack Detection Methods
 */
enum sst_jack_method {
	SST_JACK_METHOD_POLL = 0,	/* Polling-based detection */
	SST_JACK_METHOD_IRQ,		/* Interrupt-based detection */
	SST_JACK_METHOD_CODEC		/* Codec-based (via IPC) */
};

/*
 * Polling Configuration
 */
#define SST_JACK_POLL_INTERVAL_MS	250	/* Poll every 250ms */
#define SST_JACK_DEBOUNCE_MS		100	/* Debounce time */
#define SST_JACK_DEBOUNCE_COUNT		3	/* Stable readings needed */

/*
 * Jack Event Callback
 */
typedef void (*sst_jack_callback_t)(void *arg, uint32_t jackType,
				    enum sst_jack_state state);

/*
 * Individual Jack State
 */
struct sst_jack_info {
	uint32_t		type;		/* Jack type (SST_JACK_*) */
	enum sst_jack_state	state;		/* Current state */
	int			gpio;		/* GPIO pin number */
	bool			inverted;	/* Inverted logic */
	int			debounce_cnt;	/* Debounce counter */
	enum sst_jack_state	pending_state;	/* Pending state */
};

/*
 * Jack Detection Context
 */
struct sst_jack {
	struct sst_softc	*sc;		/* Parent softc */
	struct mtx		lock;		/* Lock */

	/* Detection method */
	enum sst_jack_method	method;		/* Detection method */

	/* Jack states */
	struct sst_jack_info	headphone;	/* Headphone jack */
	struct sst_jack_info	microphone;	/* Microphone jack */

	/* Polling */
	struct callout		poll_callout;	/* Polling timer */
	bool			polling;	/* Polling active */

	/* Callbacks */
	sst_jack_callback_t	callback;	/* Event callback */
	void			*callback_arg;	/* Callback argument */

	/* GPIO base address (from codec) */
	bus_addr_t		gpio_base;

	/* Statistics */
	uint32_t		hp_insertions;	/* Headphone insertions */
	uint32_t		mic_insertions;	/* Microphone insertions */
	uint32_t		poll_count;	/* Total polls */

	/* State */
	bool			initialized;	/* Subsystem ready */
	bool			enabled;	/* Detection enabled */
};

/* Forward declaration */
struct sst_softc;

/*
 * Jack Detection API
 */
int	sst_jack_init(struct sst_softc *sc);
void	sst_jack_fini(struct sst_softc *sc);

/* Enable/disable detection */
int	sst_jack_enable(struct sst_softc *sc);
void	sst_jack_disable(struct sst_softc *sc);

/* Get current jack state */
enum sst_jack_state sst_jack_get_state(struct sst_softc *sc, uint32_t jackType);
bool	sst_jack_is_inserted(struct sst_softc *sc, uint32_t jackType);

/* Register callback for jack events */
int	sst_jack_set_callback(struct sst_softc *sc, sst_jack_callback_t cb,
			      void *arg);

/* Manual poll (for debugging) */
void	sst_jack_poll(struct sst_softc *sc);

/* Interrupt handler (called from main ISR if GPIO IRQ) */
void	sst_jack_intr(struct sst_softc *sc);

/* Sysctl interface */
int	sst_jack_sysctl_init(struct sst_softc *sc);

#endif /* _SST_JACK_H_ */
