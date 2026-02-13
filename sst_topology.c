/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST Topology - Audio Pipeline Configuration
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <dev/sound/pcm/sound.h>

#include "acpi_intel_sst.h"
#include "sst_topology.h"
#include "sst_ipc.h"

/*
 * Default topology for Dell XPS 13 9343 / Broadwell-U
 *
 * Pipeline layout:
 *   Playback: Host -> PCM -> Gain -> SSP0 DAI OUT -> Codec
 *   Capture:  Codec -> SSP1 DAI IN -> Gain -> PCM -> Host
 */

/*
 * Create default playback pipeline
 */
static int
sst_topology_create_playback_pipe(struct sst_softc *sc)
{
	struct sst_topology *tplg = &sc->topology;
	struct sst_pipeline *pipe;
	struct sst_widget *w;
	uint32_t idx;

	if (tplg->pipeline_count >= SST_TPLG_MAX_PIPELINES)
		return (ENOMEM);

	idx = tplg->pipeline_count++;
	pipe = &tplg->pipelines[idx];

	memset(pipe, 0, sizeof(*pipe));
	strlcpy(pipe->name, "Playback", SST_TPLG_NAME_LEN);
	pipe->id = idx;
	pipe->type = SST_PIPE_PLAYBACK;
	pipe->state = SST_PIPE_STATE_CREATED;
	pipe->priority = 0;
	pipe->period_size = 1024;	/* 1024 frames */
	pipe->period_count = 4;
	pipe->ssp_port = 0;		/* SSP0 for playback */
	pipe->dma_channel = -1;
	pipe->core_id = 0;

	/* Create widgets for playback pipeline */

	/* Widget 1: Host PCM (input from host DMA) */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "pcm0p", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_AIF_IN;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Widget 2: Gain/Volume control */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "PGA1.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_PGA;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_GAIN;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->volume = 0;		/* 0dB default */
		w->mute = false;
		pipe->widget_count++;
	}

	/* Widget 3: SSP0 DAI output (to codec) */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "ssp0-out", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_DAI_OUT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Create routes for playback pipeline */
	if (tplg->route_count + 2 <= SST_TPLG_MAX_ROUTES) {
		struct sst_route *r;

		/* Route: pcm0p -> PGA1.0 */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, "pcm0p", SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "PGA1.0", SST_TPLG_NAME_LEN);
		r->connected = true;

		/* Route: PGA1.0 -> ssp0-out */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, "PGA1.0", SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "ssp0-out", SST_TPLG_NAME_LEN);
		r->connected = true;
	}

	device_printf(sc->dev, "Topology: Created playback pipeline (id=%u)\n",
		      pipe->id);

	return (0);
}

/*
 * Create default capture pipeline
 */
static int
sst_topology_create_capture_pipe(struct sst_softc *sc)
{
	struct sst_topology *tplg = &sc->topology;
	struct sst_pipeline *pipe;
	struct sst_widget *w;
	uint32_t idx;

	if (tplg->pipeline_count >= SST_TPLG_MAX_PIPELINES)
		return (ENOMEM);

	idx = tplg->pipeline_count++;
	pipe = &tplg->pipelines[idx];

	memset(pipe, 0, sizeof(*pipe));
	strlcpy(pipe->name, "Capture", SST_TPLG_NAME_LEN);
	pipe->id = idx;
	pipe->type = SST_PIPE_CAPTURE;
	pipe->state = SST_PIPE_STATE_CREATED;
	pipe->priority = 0;
	pipe->period_size = 1024;
	pipe->period_count = 4;
	pipe->ssp_port = 1;		/* SSP1 for capture */
	pipe->dma_channel = -1;
	pipe->core_id = 0;

	/* Create widgets for capture pipeline */

	/* Widget 1: SSP1 DAI input (from codec) */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "ssp1-in", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_DAI_IN;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Widget 2: Capture Gain control */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "PGA2.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_PGA;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_GAIN;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->volume = 0;
		w->mute = false;
		pipe->widget_count++;
	}

	/* Widget 3: Host PCM output (to host DMA) */
	if (tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "pcm0c", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_AIF_OUT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_PCM;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		pipe->widget_count++;
	}

	/* Create routes for capture pipeline */
	if (tplg->route_count + 2 <= SST_TPLG_MAX_ROUTES) {
		struct sst_route *r;

		/* Route: ssp1-in -> PGA2.0 */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, "ssp1-in", SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "PGA2.0", SST_TPLG_NAME_LEN);
		r->connected = true;

		/* Route: PGA2.0 -> pcm0c */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, "PGA2.0", SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "pcm0c", SST_TPLG_NAME_LEN);
		r->connected = true;
	}

	device_printf(sc->dev, "Topology: Created capture pipeline (id=%u)\n",
		      pipe->id);

	return (0);
}

/*
 * Initialize topology subsystem
 */
int
sst_topology_init(struct sst_softc *sc)
{
	memset(&sc->topology, 0, sizeof(sc->topology));
	sc->topology.loaded = false;
	sc->topology.initialized = true;

	device_printf(sc->dev, "Topology subsystem initialized\n");

	return (0);
}

/*
 * Cleanup topology subsystem
 */
void
sst_topology_fini(struct sst_softc *sc)
{
	uint32_t i;

	if (!sc->topology.initialized)
		return;

	/* Destroy all pipelines */
	for (i = 0; i < sc->topology.pipeline_count; i++) {
		if (sc->topology.pipelines[i].state == SST_PIPE_STATE_RUNNING)
			sst_topology_stop_pipeline(sc, i);
		sst_topology_destroy_pipeline(sc, i);
	}

	memset(&sc->topology, 0, sizeof(sc->topology));
}

/*
 * Load default topology for Broadwell-U
 */
int
sst_topology_load_default(struct sst_softc *sc)
{
	int error;

	if (sc->topology.loaded) {
		device_printf(sc->dev, "Topology already loaded\n");
		return (EBUSY);
	}

	device_printf(sc->dev, "Loading default topology...\n");

	/* Create playback pipeline */
	error = sst_topology_create_playback_pipe(sc);
	if (error) {
		device_printf(sc->dev, "Failed to create playback pipeline\n");
		return (error);
	}

	/* Create capture pipeline */
	error = sst_topology_create_capture_pipe(sc);
	if (error) {
		device_printf(sc->dev, "Failed to create capture pipeline\n");
		return (error);
	}

	/* Resolve widget references in routes */
	for (uint32_t i = 0; i < sc->topology.route_count; i++) {
		struct sst_route *r = &sc->topology.routes[i];
		r->src_widget = sst_topology_find_widget(sc, r->source);
		r->dst_widget = sst_topology_find_widget(sc, r->sink);
	}

	sc->topology.loaded = true;

	device_printf(sc->dev, "Topology loaded: %u pipelines, %u widgets, %u routes\n",
		      sc->topology.pipeline_count,
		      sc->topology.widget_count,
		      sc->topology.route_count);

	return (0);
}

/*
 * Load topology from file (future extension)
 */
int
sst_topology_load_file(struct sst_softc *sc, const char *path)
{
	/* TODO: Parse ALSA topology format (.tplg) */
	device_printf(sc->dev, "Topology file loading not yet implemented: %s\n",
		      path);
	return (ENOTSUP);
}

/*
 * Create a pipeline on the DSP
 */
int
sst_topology_create_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state != SST_PIPE_STATE_CREATED &&
	    pipe->state != SST_PIPE_STATE_INVALID) {
		return (EBUSY);
	}

	/* Pipeline is ready - actual DSP setup happens on stream allocation */
	pipe->state = SST_PIPE_STATE_CREATED;

	device_printf(sc->dev, "Topology: Pipeline '%s' created\n", pipe->name);

	return (0);
}

/*
 * Destroy a pipeline
 */
int
sst_topology_destroy_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state == SST_PIPE_STATE_RUNNING)
		sst_topology_stop_pipeline(sc, pipe_id);

	/* Free stream if allocated */
	if (pipe->stream_allocated && sc->fw.state == SST_FW_STATE_RUNNING) {
		sst_ipc_free_stream(sc, pipe->stream_id);
		pipe->stream_allocated = false;
	}

	pipe->state = SST_PIPE_STATE_INVALID;

	return (0);
}

/*
 * Start a pipeline
 */
int
sst_topology_start_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;
	struct sst_alloc_stream_req req;
	int error;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state == SST_PIPE_STATE_RUNNING)
		return (0);

	if (pipe->state != SST_PIPE_STATE_CREATED &&
	    pipe->state != SST_PIPE_STATE_PAUSED) {
		return (EINVAL);
	}

	/* Stream allocation is now handled by sst_pcm via chan_trigger */
	(void)req;

	/* Resume stream */
	if (pipe->stream_allocated) {
		error = sst_ipc_stream_resume(sc, pipe->stream_id);
		if (error) {
			device_printf(sc->dev,
			    "Topology: Failed to resume stream for '%s'\n",
			    pipe->name);
			return (error);
		}
	}

	pipe->state = SST_PIPE_STATE_RUNNING;

	device_printf(sc->dev, "Topology: Pipeline '%s' started (stream=%u)\n",
		      pipe->name, pipe->stream_id);

	return (0);
}

/*
 * Stop a pipeline
 */
int
sst_topology_stop_pipeline(struct sst_softc *sc, uint32_t pipe_id)
{
	struct sst_pipeline *pipe;

	if (pipe_id >= sc->topology.pipeline_count)
		return (EINVAL);

	pipe = &sc->topology.pipelines[pipe_id];

	if (pipe->state != SST_PIPE_STATE_RUNNING)
		return (0);

	/* Pause stream */
	if (pipe->stream_allocated && sc->fw.state == SST_FW_STATE_RUNNING) {
		sst_ipc_stream_pause(sc, pipe->stream_id);
	}

	pipe->state = SST_PIPE_STATE_PAUSED;

	device_printf(sc->dev, "Topology: Pipeline '%s' stopped\n", pipe->name);

	return (0);
}

/*
 * Find widget by name
 */
struct sst_widget *
sst_topology_find_widget(struct sst_softc *sc, const char *name)
{
	uint32_t i;

	for (i = 0; i < sc->topology.widget_count; i++) {
		if (strcmp(sc->topology.widgets[i].name, name) == 0)
			return (&sc->topology.widgets[i]);
	}

	return (NULL);
}

/*
 * Set widget volume
 */
int
sst_topology_set_widget_volume(struct sst_softc *sc, struct sst_widget *w,
			       int32_t volume)
{
	struct sst_stream_params params;

	if (w == NULL)
		return (EINVAL);

	if (w->type != SST_WIDGET_PGA && w->type != SST_WIDGET_MIXER)
		return (EINVAL);

	w->volume = volume;

	/* Update DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		memset(&params, 0, sizeof(params));
		params.stream_id = w->stream_id;
		/* Convert dB to linear (simplified) */
		params.volume_left = 100 + (volume * 100 / 64);
		params.volume_right = params.volume_left;
		params.mute = w->mute;

		return sst_ipc_stream_set_params(sc, &params);
	}

	return (0);
}

/*
 * Connect a route
 */
int
sst_topology_connect_route(struct sst_softc *sc, struct sst_route *r)
{
	if (r == NULL)
		return (EINVAL);

	r->connected = true;

	/* Actual DSP routing is done during pipeline setup */

	return (0);
}

/*
 * Disconnect a route
 */
int
sst_topology_disconnect_route(struct sst_softc *sc, struct sst_route *r)
{
	if (r == NULL)
		return (EINVAL);

	r->connected = false;

	return (0);
}

/*
 * Get pipeline for PCM stream
 */
struct sst_pipeline *
sst_topology_get_pipeline(struct sst_softc *sc, int dir, int stream_num)
{
	uint32_t i;
	enum sst_pipeline_type target_type;

	target_type = (dir == PCMDIR_PLAY) ? SST_PIPE_PLAYBACK : SST_PIPE_CAPTURE;

	for (i = 0; i < sc->topology.pipeline_count; i++) {
		if (sc->topology.pipelines[i].type == target_type)
			return (&sc->topology.pipelines[i]);
	}

	return (NULL);
}
