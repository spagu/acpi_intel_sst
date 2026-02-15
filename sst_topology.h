/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Intel SST Topology - Audio Pipeline Configuration
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Topology defines the audio pipeline structure:
 * - Pipelines (playback, capture, system sounds)
 * - Widgets (DAI, mixer, volume controls)
 * - Routes (connections between widgets)
 * - Module configurations (DSP processing)
 */

#ifndef _SST_TOPOLOGY_H_
#define _SST_TOPOLOGY_H_

#include <sys/types.h>

/*
 * Maximum limits
 */
#define SST_TPLG_MAX_PIPELINES		8
#define SST_TPLG_MAX_WIDGETS		32
#define SST_TPLG_MAX_ROUTES		64
#define SST_TPLG_MAX_MODULES		16
#define SST_TPLG_NAME_LEN		32

/*
 * Widget Types
 * Based on ALSA/ASoC widget types
 */
enum sst_widget_type {
	SST_WIDGET_INPUT = 0,		/* Audio input (ADC) */
	SST_WIDGET_OUTPUT,		/* Audio output (DAC) */
	SST_WIDGET_MUX,			/* Multiplexer */
	SST_WIDGET_MIXER,		/* Mixer */
	SST_WIDGET_PGA,			/* Programmable Gain Amplifier */
	SST_WIDGET_DAI_IN,		/* DAI input (from codec) */
	SST_WIDGET_DAI_OUT,		/* DAI output (to codec) */
	SST_WIDGET_AIF_IN,		/* Audio Interface input */
	SST_WIDGET_AIF_OUT,		/* Audio Interface output */
	SST_WIDGET_BUFFER,		/* Ring buffer */
	SST_WIDGET_EFFECT,		/* DSP effect module */
	SST_WIDGET_SRC,			/* Sample Rate Converter */
	SST_WIDGET_MAX
};

/*
 * Pipeline Types
 */
enum sst_pipeline_type {
	SST_PIPE_PLAYBACK = 0,		/* Host -> DSP -> Codec */
	SST_PIPE_CAPTURE,		/* Codec -> DSP -> Host */
	SST_PIPE_SYSTEM,		/* System sounds */
	SST_PIPE_LOOPBACK,		/* Loopback for monitoring */
	SST_PIPE_MAX
};

/*
 * Pipeline State
 */
enum sst_pipeline_state {
	SST_PIPE_STATE_INVALID = 0,
	SST_PIPE_STATE_CREATED,
	SST_PIPE_STATE_RUNNING,
	SST_PIPE_STATE_PAUSED,
	SST_PIPE_STATE_ERROR
};

/*
 * DSP Module Types (for effect processing)
 */
enum sst_module_type {
	SST_MOD_BASE_FW = 0,		/* Base firmware */
	SST_MOD_PCM,			/* PCM copier */
	SST_MOD_GAIN,			/* Gain/volume */
	SST_MOD_MIXER,			/* Mixer */
	SST_MOD_SRC,			/* Sample rate converter */
	SST_MOD_EQ,			/* Equalizer */
	SST_MOD_DRC,			/* Dynamic range compressor */
	SST_MOD_HPF,			/* High-pass filter */
	SST_MOD_LIMITER,		/* Peak limiter */
	SST_MOD_MAX
};

/*
 * EQ Preset IDs
 *
 * Each preset is a single 2nd-order biquad defining one EQ profile.
 * The catpt DSP supports only one biquad stage per stream (SET_BIQUAD
 * IPC has no stage index), so presets combine HPF + tone shaping into
 * a single filter section.
 */
enum sst_eq_preset_id {
	SST_EQ_PRESET_FLAT = 0,	/* Bypass (passthrough) */
	SST_EQ_PRESET_STOCK_SPEAKER,	/* HPF 150Hz (stock speaker protection) */
	SST_EQ_PRESET_MOD_SPEAKER,	/* HPF 100Hz (warmer, less bass cut) */
	SST_EQ_PRESET_MAX
};

/*
 * Widget Definition
 */
struct sst_widget {
	char			name[SST_TPLG_NAME_LEN];
	enum sst_widget_type	type;
	uint32_t		id;		/* Widget ID */
	uint32_t		pipeline_id;	/* Parent pipeline */
	uint32_t		module_type;	/* DSP module type */
	uint32_t		instance_id;	/* Module instance */

	/* Widget state */
	bool			active;
	uint32_t		stream_id;	/* Associated stream */

	/* Widget parameters */
	uint32_t		channels;
	uint32_t		sample_rate;
	uint32_t		bit_depth;

	/* Volume control (for PGA/MIXER) */
	int32_t			volume;		/* -64dB to +20dB in 0.5dB steps */
	bool			mute;

	/* EQ preset (for EFFECT/HPF) */
	enum sst_eq_preset_id	eq_preset;	/* Active EQ preset */

	/* Limiter control (for EFFECT/LIMITER) */
	uint32_t		limiter_threshold; /* Preset index, 0=bypass */

	/* Linked list */
	struct sst_widget	*next;
};

/*
 * Route Definition (widget connection)
 */
struct sst_route {
	char			source[SST_TPLG_NAME_LEN];
	char			sink[SST_TPLG_NAME_LEN];
	char			control[SST_TPLG_NAME_LEN];  /* Optional mixer control */

	/* Resolved widget pointers */
	struct sst_widget	*src_widget;
	struct sst_widget	*dst_widget;

	/* Connection state */
	bool			connected;
};

/*
 * Pipeline Definition
 */
struct sst_pipeline {
	char			name[SST_TPLG_NAME_LEN];
	uint32_t		id;
	enum sst_pipeline_type	type;
	enum sst_pipeline_state	state;

	/* Pipeline parameters */
	uint32_t		priority;	/* Scheduling priority */
	uint32_t		period_size;	/* Period size in frames */
	uint32_t		period_count;	/* Number of periods */

	/* Associated stream */
	uint32_t		stream_id;
	bool			stream_allocated;

	/* Hardware binding */
	uint32_t		ssp_port;	/* SSP port number */
	uint32_t		dma_channel;	/* DMA channel */

	/* Widgets in this pipeline */
	struct sst_widget	*widgets;
	uint32_t		widget_count;

	/* Pipeline memory (DSP side) */
	uint32_t		mem_pages;	/* Memory pages needed */
	uint32_t		core_id;	/* DSP core assignment */
};

/*
 * Module Instance Configuration
 */
struct sst_module_config {
	enum sst_module_type	type;
	uint32_t		instance_id;
	uint32_t		pipeline_id;

	/* Module-specific parameters */
	union {
		struct {
			int32_t	gain;		/* Gain in 0.1dB */
			bool	mute;
		} gain;

		struct {
			uint32_t input_rate;
			uint32_t output_rate;
		} src;

		struct {
			uint32_t bands;
			int32_t	gains[10];	/* Per-band gain */
		} eq;
	} params;
};

/*
 * Topology Context
 */
struct sst_topology {
	/* Pipelines */
	struct sst_pipeline	pipelines[SST_TPLG_MAX_PIPELINES];
	uint32_t		pipeline_count;

	/* Widgets */
	struct sst_widget	widgets[SST_TPLG_MAX_WIDGETS];
	uint32_t		widget_count;

	/* Routes */
	struct sst_route	routes[SST_TPLG_MAX_ROUTES];
	uint32_t		route_count;

	/* Module instances */
	struct sst_module_config modules[SST_TPLG_MAX_MODULES];
	uint32_t		module_count;

	/* State */
	bool			loaded;
	bool			initialized;
};

/* Forward declaration */
struct sst_softc;

/*
 * Topology API
 */
int	sst_topology_init(struct sst_softc *sc);
void	sst_topology_fini(struct sst_softc *sc);

/* Topology loading */
int	sst_topology_load_default(struct sst_softc *sc);
int	sst_topology_load_file(struct sst_softc *sc, const char *path);

/* Pipeline management */
int	sst_topology_create_pipeline(struct sst_softc *sc, uint32_t pipe_id);
int	sst_topology_destroy_pipeline(struct sst_softc *sc, uint32_t pipe_id);
int	sst_topology_start_pipeline(struct sst_softc *sc, uint32_t pipe_id);
int	sst_topology_stop_pipeline(struct sst_softc *sc, uint32_t pipe_id);

/* Widget management */
struct sst_widget *sst_topology_find_widget(struct sst_softc *sc,
					    const char *name);
int	sst_topology_set_widget_volume(struct sst_softc *sc,
				       struct sst_widget *w, int32_t volume);
int	sst_topology_set_widget_eq_preset(struct sst_softc *sc,
					  struct sst_widget *w,
					  enum sst_eq_preset_id preset);
int	sst_topology_set_widget_limiter(struct sst_softc *sc,
					struct sst_widget *w,
					uint32_t threshold_idx);

/* Route management */
int	sst_topology_connect_route(struct sst_softc *sc, struct sst_route *r);
int	sst_topology_disconnect_route(struct sst_softc *sc, struct sst_route *r);

/* Get pipeline for PCM stream */
struct sst_pipeline *sst_topology_get_pipeline(struct sst_softc *sc,
					       int dir, int stream_num);

/* Sysctl interface */
int	sst_topology_sysctl_init(struct sst_softc *sc);

#endif /* _SST_TOPOLOGY_H_ */
