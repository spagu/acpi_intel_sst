/*-
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/sound/pcm/sound.h>

#include "acpi_intel_sst.h"
#include "sst_topology.h"
#include "sst_ipc.h"

/*
 * dB-to-Q1.31 lookup table for widget volume control.
 * 169 entries: index 0 = -64.0dB, index 128 = 0dB (unity), index 168 = +20dB.
 * Each step = 0.5dB.  Volume parameter is in 0.5dB steps (-128..+40).
 * Formula: Q1.31 = round(0x7FFFFFFF * 10^(dB/20))
 * Values above 0dBFS are capped at 0x7FFFFFFF (unity).
 */
static const uint32_t sst_vol_db_to_q131[169] = {
	0x0014ACDB, 0x0015E67A, 0x001732AE, 0x00189292,
	0x001A074F, 0x001B9222, 0x001D345B, 0x001EEF5B,
	0x0020C49C, 0x0022B5AA, 0x0024C42C, 0x0026F1E1,
	0x002940A2, 0x002BB263, 0x002E4939, 0x00310756,
	0x0033EF0C, 0x003702D4, 0x003A454A, 0x003DB932,
	0x00416179, 0x0045413B, 0x00495BC1, 0x004DB486,
	0x00524F3B, 0x00572FC8, 0x005C5A4F, 0x0061D334,
	0x00679F1C, 0x006DC2F0, 0x007443E8, 0x007B2787,
	0x008273A6, 0x008A2E77, 0x00925E89, 0x009B0ACE,
	0x00A43AA2, 0x00ADF5D1, 0x00B8449C, 0x00C32FC3,
	0x00CEC08A, 0x00DB00C0, 0x00E7FACC, 0x00F5B9B0,
	0x01044915, 0x0113B557, 0x01240B8C, 0x01355991,
	0x0147AE14, 0x015B18A5, 0x016FA9BB, 0x018572CB,
	0x019C8651, 0x01B4F7E3, 0x01CEDC3D, 0x01EA4958,
	0x0207567A, 0x02261C4A, 0x0246B4E4, 0x02693BF0,
	0x028DCEBC, 0x02B48C50, 0x02DD958A, 0x03090D3F,
	0x0337184E, 0x0367DDCC, 0x039B8719, 0x03D2400C,
	0x040C3714, 0x04499D60, 0x048AA70B, 0x04CF8B44,
	0x05188480, 0x0565D0AB, 0x05B7B15B, 0x060E6C0B,
	0x066A4A53, 0x06CB9A26, 0x0732AE18, 0x079FDD9F,
	0x08138562, 0x088E0783, 0x090FCBF7, 0x099940DB,
	0x0A2ADAD1, 0x0AC51567, 0x0B68737A, 0x0C157FA9,
	0x0CCCCCCD, 0x0D8EF66D, 0x0E5CA14C, 0x0F367BEE,
	0x101D3F2D, 0x1111AEDB, 0x12149A60, 0x1326DD70,
	0x144960C5, 0x157D1AE2, 0x16C310E3, 0x181C5762,
	0x198A1357, 0x1B0D7B1B, 0x1CA7D768, 0x1E5A8471,
	0x2026F30F, 0x220EA9F4, 0x241346F6, 0x26368073,
	0x287A26C4, 0x2AE025C3, 0x2D6A866F, 0x301B70A8,
	0x32F52CFF, 0x35FA26A9, 0x392CED8E, 0x3C90386F,
	0x4026E73C, 0x43F4057E, 0x47FACCF0, 0x4C3EA838,
	0x50C335D3, 0x558C4B22, 0x5A9DF7AB, 0x5FFC8890,
	0x65AC8C2E, 0x6BB2D603, 0x721482BF, 0x78D6FC9E,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
	0x7FFFFFFF,
};

/*
 * Convert widget dB (0.5dB steps: -128..+40) to Q1.31 linear gain.
 */
static uint32_t
sst_db_to_linear(int32_t volume_half_db)
{
	int idx;

	idx = volume_half_db + 128;	/* shift to 0-based: 0==-64dB, 128==0dB */
	if (idx < 0)
		idx = 0;
	if (idx > 168)
		idx = 168;
	return (sst_vol_db_to_q131[idx]);
}

/*
 * EQ preset biquad coefficients.
 *
 * Each preset is a single 2nd-order biquad (Q2.30 fixed-point) sent
 * to the catpt DSP via SET_BIQUAD IPC.  The DSP supports only one
 * biquad stage per stream, so each preset defines a complete filter.
 *
 * Coefficients: 2nd-order Butterworth HPF at 48kHz sample rate.
 * Formula:
 *   omega = 2*pi*fc/fs, alpha = sin(omega)/sqrt(2), a0 = 1+alpha
 *   b0 = (1+cos(omega))/2/a0,  b1 = -(1+cos(omega))/a0,  b2 = b0
 *   a1 = -2*cos(omega)/a0,     a2 = (1-alpha)/a0
 */
struct sst_eq_preset_entry {
	enum sst_eq_preset_id	id;
	const char		*name;
	int32_t			b0, b1, b2;	/* Numerator, Q2.30 */
	int32_t			a1, a2;		/* Denominator, Q2.30 */
	int			peak_gain_db;	/* max gain at any freq (dB) */
};

#define SST_EQ_NUM_PRESETS	3

static const struct sst_eq_preset_entry sst_eq_presets[SST_EQ_NUM_PRESETS] = {
	/* FLAT: bypass (b0=1.0, rest=0) */
	{ SST_EQ_PRESET_FLAT,
	  "flat",
	  1073741824, 0, 0, 0, 0, 0 },
	/* STOCK_SPEAKER: 2nd-order Butterworth HPF at 150Hz/48kHz */
	{ SST_EQ_PRESET_STOCK_SPEAKER,
	  "stock_speaker",
	  1058936991, -2117873982, 1058936991, -2117669842, 1044336297, 0 },
	/* MOD_SPEAKER: 2nd-order Butterworth HPF at 100Hz/48kHz */
	{ SST_EQ_PRESET_MOD_SPEAKER,
	  "mod_speaker",
	  1063849116, -2127698232, 1063849116, -2127607086, 1054047555, 0 },
};

/*
 * Precomputed peak limiter threshold presets at 48kHz.
 * Q2.30 linear amplitude (range [-2.0, +2.0), 1.0 = 0x40000000).
 *
 * Formula: Q2.30 = round(2^30 * 10^(dB/20))
 *
 * Derived from Q1.31 volume table (Q2.30 = Q1.31 >> 1):
 *   -24dB: Q1.31[80]  = 0x08138562 → Q2.30 = 0x0409C2B1
 *   -18dB: Q1.31[92]  = 0x101D3F2D → Q2.30 = 0x080E9F97
 *   -12dB: Q1.31[104] = 0x2026F30F → Q2.30 = 0x10137988
 *    -9dB: Q1.31[110] = 0x2D6A866F → Q2.30 = 0x16B54338
 *    -6dB: Q1.31[116] = 0x4026E73C → Q2.30 = 0x2013739E
 *    -3dB: Q1.31[122] = 0x5A9DF7AB → Q2.30 = 0x2D4EFBD6
 *    -1dB: Q1.31[126] = 0x721482BF → Q2.30 = 0x390A4160
 *     0dB: 2^30 = 0x40000000
 *
 * Attack: fixed 1ms (1000us) for all presets (fast transient catch).
 * Release: scaled per preset (50ms at -24dB to 200ms at 0dB).
 */
struct sst_limiter_preset {
	uint32_t	threshold_db;		/* Threshold in dBFS (0=bypass) */
	int32_t		threshold_linear;	/* Q2.30 linear amplitude */
	uint32_t	attack_us;		/* Attack time in microseconds */
	uint32_t	release_us;		/* Release time in microseconds */
};

#define SST_LIMITER_NUM_PRESETS	9

static const struct sst_limiter_preset sst_limiter_presets[SST_LIMITER_NUM_PRESETS] = {
	{  0, 0x7FFFFFFF,    0,      0 },	/* 0: bypass (no limiting) */
	{ 24, 0x0409C2B1, 1000,  50000 },	/* 1: -24 dBFS */
	{ 18, 0x080E9F97, 1000,  75000 },	/* 2: -18 dBFS */
	{ 12, 0x10137988, 1000, 100000 },	/* 3: -12 dBFS */
	{  9, 0x16B54338, 1000, 115000 },	/* 4:  -9 dBFS */
	{  6, 0x2013739E, 1000, 135000 },	/* 5:  -6 dBFS (default) */
	{  3, 0x2D4EFBD6, 1000, 160000 },	/* 6:  -3 dBFS */
	{  1, 0x390A4160, 1000, 180000 },	/* 7:  -1 dBFS */
	{  0, 0x40000000, 1000, 200000 },	/* 8:   0 dBFS (full scale) */
};

/*
 * Default topology for Dell XPS 13 9343 / Broadwell-U
 *
 * Pipeline layout:
 *   Playback: Host -> PCM -> HPF -> Gain -> Limiter -> SSP0 DAI OUT -> Codec
 *   Capture:  Codec -> SSP1 DAI IN -> Gain -> PCM -> Host
 */

/*
 * Create default playback pipeline.
 *
 * The pipeline is built dynamically based on probed DSP capabilities:
 *   Always:      pcm0p (AIF_IN), PGA1.0 (Gain), ssp0-out (DAI_OUT)
 *   If biquad:   HPF1.0 inserted between pcm0p and PGA1.0
 *   If limiter:  LIMITER1.0 inserted between PGA1.0 and ssp0-out
 *
 * Resulting topologies:
 *   Full:    pcm0p -> HPF1.0 -> PGA1.0 -> LIMITER1.0 -> ssp0-out
 *   No HPF:  pcm0p -> PGA1.0 -> LIMITER1.0 -> ssp0-out
 *   No Lim:  pcm0p -> HPF1.0 -> PGA1.0 -> ssp0-out
 *   Minimal: pcm0p -> PGA1.0 -> ssp0-out
 */
static int
sst_topology_create_playback_pipe(struct sst_softc *sc)
{
	struct sst_topology *tplg = &sc->topology;
	struct sst_pipeline *pipe;
	struct sst_widget *w;
	struct sst_route *r;
	uint32_t idx;
	const char *prev;
	uint32_t route_count;
	bool has_hpf, has_lim;

	if (tplg->pipeline_count >= SST_TPLG_MAX_PIPELINES)
		return (ENOMEM);

	has_hpf = sc->fw.has_biquad;
	has_lim = sc->fw.has_limiter;

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

	/* Widget: Host PCM (input from host DMA) - always present */
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

	/* Widget: High-pass filter (speaker protection) - if supported */
	if (has_hpf && tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "HPF1.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_EFFECT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_HPF;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->eq_preset = SST_EQ_PRESET_STOCK_SPEAKER;
		pipe->widget_count++;
	}

	/* Widget: Gain/Volume control - always present */
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

	/* Widget: Limiter (speaker protection) - if supported */
	if (has_lim && tplg->widget_count < SST_TPLG_MAX_WIDGETS) {
		w = &tplg->widgets[tplg->widget_count++];
		memset(w, 0, sizeof(*w));
		strlcpy(w->name, "LIMITER1.0", SST_TPLG_NAME_LEN);
		w->type = SST_WIDGET_EFFECT;
		w->id = tplg->widget_count - 1;
		w->pipeline_id = pipe->id;
		w->module_type = SST_MOD_LIMITER;
		w->channels = 2;
		w->sample_rate = 48000;
		w->bit_depth = 16;
		w->limiter_threshold = 5;	/* Default -6 dBFS */
		pipe->widget_count++;
	}

	/* Widget: SSP0 DAI output (to codec) - always present */
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

	/*
	 * Build routes dynamically using a prev_widget chain.
	 * Count needed routes: 2 (pcm0p->PGA, PGA->ssp0) + hpf + limiter
	 */
	route_count = 2 + (has_hpf ? 1 : 0) + (has_lim ? 1 : 0);
	if (tplg->route_count + route_count <= SST_TPLG_MAX_ROUTES) {
		prev = "pcm0p";

		if (has_hpf) {
			r = &tplg->routes[tplg->route_count++];
			strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
			strlcpy(r->sink, "HPF1.0", SST_TPLG_NAME_LEN);
			r->connected = true;
			prev = "HPF1.0";
		}

		/* prev -> PGA1.0 */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "PGA1.0", SST_TPLG_NAME_LEN);
		r->connected = true;
		prev = "PGA1.0";

		if (has_lim) {
			r = &tplg->routes[tplg->route_count++];
			strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
			strlcpy(r->sink, "LIMITER1.0", SST_TPLG_NAME_LEN);
			r->connected = true;
			prev = "LIMITER1.0";
		}

		/* prev -> ssp0-out */
		r = &tplg->routes[tplg->route_count++];
		strlcpy(r->source, prev, SST_TPLG_NAME_LEN);
		strlcpy(r->sink, "ssp0-out", SST_TPLG_NAME_LEN);
		r->connected = true;
	}

	device_printf(sc->dev,
	    "Topology: Created playback pipeline (id=%u) "
	    "widgets=%u [HPF=%s, Limiter=%s]\n",
	    pipe->id, pipe->widget_count,
	    has_hpf ? "yes" : "skipped",
	    has_lim ? "yes" : "skipped");

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

	/*
	 * Enforce headroom ceiling, accounting for EQ peak gain.
	 *
	 * The base ceiling is -SST_HEADROOM_DB (-3dBFS in half-dB steps).
	 * If the pipeline's EQ preset boosts any frequency by peak_gain_db,
	 * we must reduce the volume ceiling by the same amount so that
	 * EQ peak + volume never exceeds -SST_HEADROOM_DB dBFS.
	 */
	{
		int32_t ceiling;
		struct sst_widget *hpf;

		ceiling = -SST_HEADROOM_HALF_DB;
		hpf = sst_topology_find_widget(sc, "HPF1.0");
		if (hpf != NULL && hpf->pipeline_id == w->pipeline_id &&
		    hpf->eq_preset < SST_EQ_NUM_PRESETS) {
			int peak = sst_eq_presets[hpf->eq_preset].peak_gain_db;
			if (peak > 0)
				ceiling -= peak * 2; /* dB to half-dB steps */
		}
		if (volume > ceiling)
			volume = ceiling;
	}

	w->volume = volume;

	/* Update DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		uint32_t gain;

		memset(&params, 0, sizeof(params));
		params.stream_id = w->stream_id;
		gain = sst_db_to_linear(volume);
		params.volume_left = gain;
		params.volume_right = gain;
		params.mute = w->mute;

		return sst_ipc_stream_set_params(sc, &params);
	}

	return (0);
}

/*
 * Set widget EQ preset.
 * Looks up preset in the EQ table and sends biquad coefficients to DSP.
 */
int
sst_topology_set_widget_eq_preset(struct sst_softc *sc, struct sst_widget *w,
				  enum sst_eq_preset_id preset)
{
	const struct sst_eq_preset_entry *entry;

	if (w == NULL || w->module_type != SST_MOD_HPF)
		return (EINVAL);
	if (preset >= SST_EQ_NUM_PRESETS)
		preset = SST_EQ_PRESET_FLAT;

	entry = &sst_eq_presets[preset];
	w->eq_preset = preset;

	/*
	 * Gain budget enforcement: if the new preset boosts any
	 * frequency, pull the pipeline's PGA volume down so that
	 * EQ peak + volume stays within headroom.
	 */
	if (entry->peak_gain_db > 0) {
		struct sst_widget *pga;
		int32_t ceiling;

		ceiling = -(SST_HEADROOM_HALF_DB +
		    entry->peak_gain_db * 2);
		pga = sst_topology_find_widget(sc, "PGA1.0");
		if (pga != NULL && pga->pipeline_id == w->pipeline_id &&
		    pga->volume > ceiling)
			sst_topology_set_widget_volume(sc, pga, pga->volume);
	}

	/* Send biquad coefficients to DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		return sst_ipc_set_biquad(sc, w->stream_id,
		    entry->b0, entry->b1, entry->b2,
		    entry->a1, entry->a2);
	}

	return (0);
}

/*
 * Set widget limiter threshold.
 * Looks up threshold_idx in the preset table and sends parameters to DSP.
 */
int
sst_topology_set_widget_limiter(struct sst_softc *sc, struct sst_widget *w,
				uint32_t threshold_idx)
{
	const struct sst_limiter_preset *entry;

	if (w == NULL)
		return (EINVAL);

	if (w->module_type != SST_MOD_LIMITER)
		return (EINVAL);

	/* Clamp to valid preset range */
	if (threshold_idx >= SST_LIMITER_NUM_PRESETS)
		threshold_idx = SST_LIMITER_NUM_PRESETS - 1;
	entry = &sst_limiter_presets[threshold_idx];

	w->limiter_threshold = threshold_idx;

	/* Send limiter parameters to DSP if stream is active */
	if (w->stream_id != 0 && sc->fw.state == SST_FW_STATE_RUNNING) {
		return sst_ipc_set_limiter(sc, w->stream_id,
		    entry->threshold_linear, entry->attack_us,
		    entry->release_us);
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

/*
 * Sysctl handler for EQ preset switching.
 * Reads/writes sc->pcm.eq_preset and applies to the HPF widget.
 */
static int
sst_eq_preset_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct sst_softc *sc = arg1;
	int preset, error;

	preset = (int)sc->pcm.eq_preset;
	error = sysctl_handle_int(oidp, &preset, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (preset < 0 || preset >= SST_EQ_NUM_PRESETS)
		return (EINVAL);

	sc->pcm.eq_preset = (enum sst_eq_preset_id)preset;

	{
		struct sst_widget *hpf_w;
		hpf_w = sst_topology_find_widget(sc, "HPF1.0");
		if (hpf_w != NULL)
			sst_topology_set_widget_eq_preset(sc, hpf_w,
			    sc->pcm.eq_preset);
	}

	return (0);
}

/*
 * Register EQ preset sysctl node under device tree.
 * Creates dev.acpi_intel_sst.N.eq_preset (RW).
 */
int
sst_topology_sysctl_init(struct sst_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "eq_preset", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, sst_eq_preset_sysctl, "I",
	    "EQ preset (0=flat, 1=stock_speaker, 2=mod_speaker)");

	return (0);
}
