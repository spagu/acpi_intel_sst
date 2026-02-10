/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST PCM Driver - sound(4) Integration
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

#include "mixer_if.h"

#include "acpi_intel_sst.h"
#include "sst_pcm.h"

/*
 * Supported formats
 */
static uint32_t sst_fmtlist[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	SND_FORMAT(AFMT_S24_LE, 2, 0),
	SND_FORMAT(AFMT_S32_LE, 2, 0),
	0
};

/*
 * Supported rates
 */
static struct pcmchan_caps sst_caps = {
	8000,		/* Minimum rate */
	192000,		/* Maximum rate */
	sst_fmtlist,	/* Formats */
	0		/* Flags */
};

/*
 * Channel methods forward declarations
 */
static void *sst_chan_init(kobj_t obj, void *devinfo,
    struct snd_dbuf *b, struct pcm_channel *c, int dir);
static int sst_chan_free(kobj_t obj, void *data);
static int sst_chan_setformat(kobj_t obj, void *data, uint32_t format);
static uint32_t sst_chan_setspeed(kobj_t obj, void *data, uint32_t speed);
static uint32_t sst_chan_setblocksize(kobj_t obj, void *data, uint32_t blocksize);
static int sst_chan_trigger(kobj_t obj, void *data, int go);
static uint32_t sst_chan_getptr(kobj_t obj, void *data);
static struct pcmchan_caps *sst_chan_getcaps(kobj_t obj, void *data);
static void sst_pcm_dma_callback(void *arg);

/* DSP stream allocation helpers */
static int sst_pcm_alloc_dsp_stream(struct sst_softc *sc,
    struct sst_pcm_channel *ch);
static void sst_pcm_free_dsp_stream(struct sst_softc *sc,
    struct sst_pcm_channel *ch);

/*
 * Channel method definitions
 */
static kobj_method_t sst_chan_methods[] = {
	KOBJMETHOD(channel_init,	sst_chan_init),
	KOBJMETHOD(channel_free,	sst_chan_free),
	KOBJMETHOD(channel_setformat,	sst_chan_setformat),
	KOBJMETHOD(channel_setspeed,	sst_chan_setspeed),
	KOBJMETHOD(channel_setblocksize, sst_chan_setblocksize),
	KOBJMETHOD(channel_trigger,	sst_chan_trigger),
	KOBJMETHOD(channel_getptr,	sst_chan_getptr),
	KOBJMETHOD(channel_getcaps,	sst_chan_getcaps),
	KOBJMETHOD_END
};

CHANNEL_DECLARE(sst_chan);

/*
 * DMA callback for buffer allocation
 */
static void
sst_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct sst_pcm_channel *ch = arg;

	if (error || nseg != 1)
		return;

	ch->dma_addr = segs[0].ds_addr;
}

/*
 * Allocate DMA buffer for PCM channel
 */
static int
sst_pcm_alloc_buffer(struct sst_softc *sc, struct sst_pcm_channel *ch)
{
	int error;

	/* Create DMA tag */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent */
	    4,				/* Alignment */
	    0,				/* Boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* Low address */
	    BUS_SPACE_MAXADDR,		/* High address */
	    NULL, NULL,			/* Filter */
	    SST_PCM_BUFFER_SIZE,	/* Max size */
	    1,				/* Segments */
	    SST_PCM_BUFFER_SIZE,	/* Max segment size */
	    0,				/* Flags */
	    NULL, NULL,			/* Lock */
	    &ch->dma_tag);

	if (error) {
		device_printf(sc->dev, "Failed to create DMA tag: %d\n", error);
		return (error);
	}

	/* Allocate DMA buffer */
	error = bus_dmamem_alloc(ch->dma_tag, &ch->buf,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO, &ch->dma_map);
	if (error) {
		device_printf(sc->dev, "Failed to allocate DMA buffer: %d\n",
		    error);
		bus_dma_tag_destroy(ch->dma_tag);
		ch->dma_tag = NULL;
		return (error);
	}

	/* Map buffer */
	error = bus_dmamap_load(ch->dma_tag, ch->dma_map, ch->buf,
	    SST_PCM_BUFFER_SIZE, sst_dma_cb, ch, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->dev, "Failed to map DMA buffer: %d\n", error);
		bus_dmamem_free(ch->dma_tag, ch->buf, ch->dma_map);
		bus_dma_tag_destroy(ch->dma_tag);
		ch->dma_tag = NULL;
		ch->buf = NULL;
		return (error);
	}

	ch->buf_size = SST_PCM_BUFFER_SIZE;
	ch->blk_size = SST_PCM_BLOCK_SIZE;
	ch->blk_count = SST_PCM_BLOCK_COUNT;

	return (0);
}

/*
 * Free DMA buffer
 */
static void
sst_pcm_free_buffer(struct sst_pcm_channel *ch)
{
	if (ch->dma_map != NULL) {
		bus_dmamap_unload(ch->dma_tag, ch->dma_map);
	}

	if (ch->buf != NULL) {
		bus_dmamem_free(ch->dma_tag, ch->buf, ch->dma_map);
		ch->buf = NULL;
	}

	if (ch->dma_tag != NULL) {
		bus_dma_tag_destroy(ch->dma_tag);
		ch->dma_tag = NULL;
	}
}

/*
 * Find free channel slot (multi-stream support)
 */
static struct sst_pcm_channel *
sst_pcm_alloc_channel(struct sst_softc *sc, int dir)
{
	struct sst_pcm_channel *ch;
	uint32_t *alloc;
	uint32_t max;
	int i;

	if (dir == PCMDIR_PLAY) {
		alloc = &sc->pcm.play_alloc;
		max = SST_PCM_MAX_PLAY;
	} else {
		alloc = &sc->pcm.rec_alloc;
		max = SST_PCM_MAX_REC;
	}

	/* Find first free slot */
	for (i = 0; i < max; i++) {
		if ((*alloc & (1 << i)) == 0) {
			*alloc |= (1 << i);
			if (dir == PCMDIR_PLAY) {
				ch = &sc->pcm.play[i];
				sc->pcm.play_count++;
			} else {
				ch = &sc->pcm.rec[i];
				sc->pcm.rec_count++;
			}
			ch->index = i;
			ch->allocated = true;
			return (ch);
		}
	}

	return (NULL);	/* No free slots */
}

/*
 * Release channel slot
 */
static void
sst_pcm_release_channel(struct sst_softc *sc, struct sst_pcm_channel *ch)
{
	uint32_t *alloc;

	if (!ch->allocated)
		return;

	if (ch->dir == PCMDIR_PLAY) {
		alloc = &sc->pcm.play_alloc;
		if (sc->pcm.play_count > 0)
			sc->pcm.play_count--;
	} else {
		alloc = &sc->pcm.rec_alloc;
		if (sc->pcm.rec_count > 0)
			sc->pcm.rec_count--;
	}

	*alloc &= ~(1 << ch->index);
	ch->allocated = false;
}

/*
 * Channel init
 */
static void *
sst_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct sst_softc *sc = devinfo;
	struct sst_pcm_channel *ch;
	int error;

	/* Allocate channel slot from pool */
	ch = sst_pcm_alloc_channel(sc, dir);
	if (ch == NULL) {
		device_printf(sc->dev, "No free %s channel slots\n",
		    (dir == PCMDIR_PLAY) ? "playback" : "capture");
		return (NULL);
	}

	/* SSP port: 0 for playback, 1 for capture */
	ch->ssp_port = (dir == PCMDIR_PLAY) ? 0 : 1;
	ch->dir = dir;
	ch->state = SST_PCM_STATE_INIT;
	ch->ptr = 0;
	ch->pcm_ch = c;

	/* Allocate DMA buffer */
	error = sst_pcm_alloc_buffer(sc, ch);
	if (error) {
		sst_pcm_release_channel(sc, ch);
		return (NULL);
	}

	/* Allocate DMA channel */
	ch->dma_ch = sst_dma_alloc(sc);
	if (ch->dma_ch < 0) {
		device_printf(sc->dev, "Failed to allocate DMA channel\n");
		sst_pcm_free_buffer(ch);
		sst_pcm_release_channel(sc, ch);
		return (NULL);
	}

	/* Setup sound buffer */
	if (sndbuf_setup(b, ch->buf, ch->buf_size) != 0) {
		device_printf(sc->dev, "Failed to setup sound buffer\n");
		sst_dma_free(sc, ch->dma_ch);
		sst_pcm_free_buffer(ch);
		sst_pcm_release_channel(sc, ch);
		return (NULL);
	}

	/* Default format */
	ch->format = SND_FORMAT(AFMT_S16_LE, 2, 0);
	ch->speed = 48000;
	ch->channels = 2;

	/* DSP stream not yet allocated */
	ch->stream_id = 0;
	ch->stream_allocated = false;

	device_printf(sc->dev, "PCM: Allocated %s stream %d\n",
	    (dir == PCMDIR_PLAY) ? "playback" : "capture", ch->index);

	return (ch);
}

/*
 * Channel free
 */
static int
sst_chan_free(kobj_t obj, void *data)
{
	struct sst_pcm_channel *ch = data;
	struct sst_softc *sc;

	if (ch == NULL)
		return (0);

	sc = ch->sc;

	if (ch->state == SST_PCM_STATE_RUNNING)
		sst_ssp_stop(sc, ch->ssp_port);

	/* Free DSP stream if allocated */
	if (ch->stream_allocated)
		sst_pcm_free_dsp_stream(sc, ch);

	if (ch->dma_ch >= 0) {
		sst_dma_free(sc, ch->dma_ch);
		ch->dma_ch = -1;
	}

	sst_pcm_free_buffer(ch);

	/* Release channel slot back to pool */
	sst_pcm_release_channel(sc, ch);

	device_printf(sc->dev, "PCM: Released %s stream %d\n",
	    (ch->dir == PCMDIR_PLAY) ? "playback" : "capture", ch->index);

	return (0);
}

/*
 * Set format
 */
static int
sst_chan_setformat(kobj_t obj, void *data, uint32_t format)
{
	struct sst_pcm_channel *ch = data;

	ch->format = format;
	ch->channels = AFMT_CHANNEL(format);

	return (0);
}

/*
 * Set sample rate
 */
static uint32_t
sst_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{
	struct sst_pcm_channel *ch = data;

	/* Clamp to supported range */
	if (speed < 8000)
		speed = 8000;
	if (speed > 192000)
		speed = 192000;

	ch->speed = speed;

	return (speed);
}

/*
 * Set block size
 */
static uint32_t
sst_chan_setblocksize(kobj_t obj, void *data, uint32_t blocksize)
{
	struct sst_pcm_channel *ch = data;

	/* Align to 4 bytes */
	blocksize &= ~3;

	if (blocksize < 256)
		blocksize = 256;
	if (blocksize > SST_PCM_BLOCK_SIZE)
		blocksize = SST_PCM_BLOCK_SIZE;

	ch->blk_size = blocksize;
	ch->blk_count = ch->buf_size / blocksize;

	return (blocksize);
}

/*
 * Allocate DSP stream via IPC
 */
static int
sst_pcm_alloc_dsp_stream(struct sst_softc *sc, struct sst_pcm_channel *ch)
{
	struct sst_alloc_stream_req req;
	int error;

	if (ch->stream_allocated)
		return (0);

	memset(&req, 0, sizeof(req));

	/* Set stream type */
	req.stream_type = (ch->dir == PCMDIR_PLAY) ?
	    SST_STREAM_TYPE_RENDER : SST_STREAM_TYPE_CAPTURE;

	/* Set audio path (SSP port) */
	req.path_id = ch->ssp_port;

	/* Set audio format */
	req.format.sample_rate = ch->speed;
	req.format.bit_depth = AFMT_BIT(ch->format);
	req.format.channels = ch->channels;
	req.format.format_id = SST_FMT_PCM;
	req.format.interleaving = 1;	/* Interleaved */

	/* Channel map: stereo = 0x03 (front left + front right) */
	if (ch->channels == 2)
		req.format.channel_map = 0x03;
	else
		req.format.channel_map = (1 << ch->channels) - 1;

	/* Set DMA buffer info */
	req.ring_buf_addr = (uint32_t)ch->dma_addr;
	req.ring_buf_size = ch->buf_size;
	req.period_count = ch->blk_count;

	/* Allocate stream on DSP */
	error = sst_ipc_alloc_stream(sc, &req, &ch->stream_id);
	if (error) {
		device_printf(sc->dev, "PCM: DSP stream allocation failed\n");
		return (error);
	}

	ch->stream_allocated = true;
	device_printf(sc->dev, "PCM: Allocated DSP stream %u for %s\n",
		      ch->stream_id,
		      (ch->dir == PCMDIR_PLAY) ? "playback" : "capture");

	return (0);
}

/*
 * Free DSP stream via IPC
 */
static void
sst_pcm_free_dsp_stream(struct sst_softc *sc, struct sst_pcm_channel *ch)
{
	if (!ch->stream_allocated)
		return;

	sst_ipc_free_stream(sc, ch->stream_id);
	ch->stream_allocated = false;
	ch->stream_id = 0;
}

/*
 * Trigger (start/stop)
 */
static int
sst_chan_trigger(kobj_t obj, void *data, int go)
{
	struct sst_pcm_channel *ch = data;
	struct sst_softc *sc;
	struct sst_ssp_config ssp_cfg;
	struct sst_dma_config dma_cfg;
	int error;

	/* Get parent softc */
	sc = ch->sc;

	switch (go) {
	case PCMTRIG_START:
		if (ch->state == SST_PCM_STATE_RUNNING)
			return (0);

		/* Allocate DSP stream if firmware is running */
		if (sc->fw.state == SST_FW_STATE_RUNNING) {
			error = sst_pcm_alloc_dsp_stream(sc, ch);
			if (error) {
				device_printf(sc->dev,
				    "DSP stream alloc failed: %d\n", error);
				/* Continue without DSP - direct SSP mode */
			}
		}

		/* Configure SSP */
		ssp_cfg.sample_rate = ch->speed;
		ssp_cfg.sample_bits = AFMT_BIT(ch->format);
		ssp_cfg.channels = ch->channels;
		ssp_cfg.format = SST_FMT_I2S;
		ssp_cfg.mclk_rate = 0;	/* Auto */
		ssp_cfg.master = true;

		error = sst_ssp_configure(sc, ch->ssp_port, &ssp_cfg);
		if (error) {
			device_printf(sc->dev, "SSP configure failed: %d\n",
			    error);
			return (error);
		}

		/* Configure DMA */
		memset(&dma_cfg, 0, sizeof(dma_cfg));
		if (ch->dir == PCMDIR_PLAY) {
			dma_cfg.src = ch->dma_addr;
			dma_cfg.dst = SST_SSP0_OFFSET + SSP_SSDR;
			dma_cfg.direction = DMA_TT_M2P;
		} else {
			dma_cfg.src = SST_SSP1_OFFSET + SSP_SSDR;
			dma_cfg.dst = ch->dma_addr;
			dma_cfg.direction = DMA_TT_P2M;
		}
		dma_cfg.size = ch->buf_size;
		dma_cfg.src_width = DMA_WIDTH_32;
		dma_cfg.dst_width = DMA_WIDTH_32;
		dma_cfg.src_burst = DMA_MSIZE_4;
		dma_cfg.dst_burst = DMA_MSIZE_4;
		dma_cfg.circular = true;

		error = sst_dma_configure(sc, ch->dma_ch, &dma_cfg);
		if (error) {
			device_printf(sc->dev, "DMA configure failed: %d\n",
			    error);
			return (error);
		}

		/* Set DMA callback */
		sst_dma_set_callback(sc, ch->dma_ch, sst_pcm_dma_callback, ch);

		/* Resume DSP stream if allocated */
		if (ch->stream_allocated)
			sst_ipc_stream_resume(sc, ch->stream_id);

		/* Start DMA first, then SSP */
		ch->ptr = 0;
		sst_dma_start(sc, ch->dma_ch);
		sst_ssp_start(sc, ch->ssp_port);

		ch->state = SST_PCM_STATE_RUNNING;
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (ch->state != SST_PCM_STATE_RUNNING)
			return (0);

		/* Pause DSP stream */
		if (ch->stream_allocated)
			sst_ipc_stream_pause(sc, ch->stream_id);

		/* Stop SSP first, then DMA */
		sst_ssp_stop(sc, ch->ssp_port);
		sst_dma_stop(sc, ch->dma_ch);

		/* Free DSP stream on abort */
		if (go == PCMTRIG_ABORT && ch->stream_allocated)
			sst_pcm_free_dsp_stream(sc, ch);

		ch->state = SST_PCM_STATE_PREPARED;
		break;

	case PCMTRIG_EMLDMAWR:
	case PCMTRIG_EMLDMARD:
		/* Software DMA trigger - not used */
		break;
	}

	return (0);
}

/*
 * Get current buffer position
 */
static uint32_t
sst_chan_getptr(kobj_t obj, void *data)
{
	struct sst_pcm_channel *ch = data;
	struct sst_softc *sc;
	size_t pos;

	sc = ch->sc;

	if (ch->state != SST_PCM_STATE_RUNNING)
		return (0);

	/* Get position from DMA */
	pos = sst_dma_get_position(sc, ch->dma_ch);

	return (pos % ch->buf_size);
}

/*
 * Get channel capabilities
 */
static struct pcmchan_caps *
sst_chan_getcaps(kobj_t obj, void *data)
{
	return (&sst_caps);
}

/*
 * DMA completion callback
 */
static void
sst_pcm_dma_callback(void *arg)
{
	struct sst_pcm_channel *ch = arg;

	/* Update position */
	ch->ptr += ch->blk_size;
	if (ch->ptr >= ch->buf_size)
		ch->ptr = 0;

	/* Notify sound subsystem */
	chn_intr(ch->pcm_ch);
}

/*
 * Mixer methods
 */
static int
sst_mixer_init(struct snd_mixer *m)
{
	struct sst_softc *sc = mix_getdevinfo(m);

	/* Register mixer controls */
	mix_setdevs(m, SOUND_MASK_PCM | SOUND_MASK_VOLUME);

	/* Set initial volumes */
	sc->pcm.vol_left = 100;
	sc->pcm.vol_right = 100;
	sc->pcm.mute = 0;

	return (0);
}

static int
sst_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct sst_softc *sc = mix_getdevinfo(m);
	struct sst_mixer_params params;
	struct sst_stream_params sp;
	int i;

	switch (dev) {
	case SOUND_MIXER_VOLUME:
	case SOUND_MIXER_PCM:
		sc->pcm.vol_left = left;
		sc->pcm.vol_right = right;

		/* Update DSP master mixer if firmware running */
		if (sc->fw.state == SST_FW_STATE_RUNNING) {
			memset(&params, 0, sizeof(params));
			params.output_id = 0;	/* Main output */
			params.volume = (left + right) / 2;
			params.mute = sc->pcm.mute;
			sst_ipc_set_mixer(sc, &params);
		}

		/* Update volume on all active playback streams */
		for (i = 0; i < SST_PCM_MAX_PLAY; i++) {
			if (sc->pcm.play[i].stream_allocated) {
				memset(&sp, 0, sizeof(sp));
				sp.stream_id = sc->pcm.play[i].stream_id;
				sp.volume_left = left;
				sp.volume_right = right;
				sp.mute = sc->pcm.mute;
				sst_ipc_stream_set_params(sc, &sp);
			}
		}
		break;
	default:
		return (-1);
	}

	return (left | (right << 8));
}

static uint32_t
sst_mixer_setrecsrc(struct snd_mixer *m, uint32_t src)
{
	/* Only support microphone */
	return (SOUND_MASK_MIC);
}

static kobj_method_t sst_mixer_methods[] = {
	KOBJMETHOD(mixer_init,		sst_mixer_init),
	KOBJMETHOD(mixer_set,		sst_mixer_set),
	KOBJMETHOD(mixer_setrecsrc,	sst_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(sst_mixer);

/*
 * Initialize PCM subsystem
 */
int
sst_pcm_init(struct sst_softc *sc)
{
	int i;

	sc->pcm.sc = sc;
	sc->pcm.dev = NULL;
	sc->pcm.registered = false;

	/* Initialize stream counters and allocation bitmaps */
	sc->pcm.play_count = 0;
	sc->pcm.rec_count = 0;
	sc->pcm.play_alloc = 0;
	sc->pcm.rec_alloc = 0;

	/* Initialize all playback channel slots */
	for (i = 0; i < SST_PCM_MAX_PLAY; i++) {
		sc->pcm.play[i].sc = sc;
		sc->pcm.play[i].dir = PCMDIR_PLAY;
		sc->pcm.play[i].index = i;
		sc->pcm.play[i].allocated = false;
		sc->pcm.play[i].state = SST_PCM_STATE_INIT;
		sc->pcm.play[i].dma_ch = -1;
		sc->pcm.play[i].dma_tag = NULL;
		sc->pcm.play[i].stream_id = 0;
		sc->pcm.play[i].stream_allocated = false;
	}

	/* Initialize all capture channel slots */
	for (i = 0; i < SST_PCM_MAX_REC; i++) {
		sc->pcm.rec[i].sc = sc;
		sc->pcm.rec[i].dir = PCMDIR_REC;
		sc->pcm.rec[i].index = i;
		sc->pcm.rec[i].allocated = false;
		sc->pcm.rec[i].state = SST_PCM_STATE_INIT;
		sc->pcm.rec[i].dma_ch = -1;
		sc->pcm.rec[i].dma_tag = NULL;
		sc->pcm.rec[i].stream_id = 0;
		sc->pcm.rec[i].stream_allocated = false;
	}

	/* Default mixer values */
	sc->pcm.vol_left = 100;
	sc->pcm.vol_right = 100;
	sc->pcm.mute = 0;

	device_printf(sc->dev, "PCM subsystem initialized: %d play, %d rec streams\n",
	    SST_PCM_MAX_PLAY, SST_PCM_MAX_REC);

	return (0);
}

/*
 * Cleanup PCM subsystem
 */
void
sst_pcm_fini(struct sst_softc *sc)
{
	sst_pcm_unregister(sc);
}

/*
 * Register PCM device with sound(4)
 */
int
sst_pcm_register(struct sst_softc *sc)
{
	int error;
	int i;
	char status[SND_STATUSLEN];

	if (sc->pcm.registered)
		return (0);

	/* Create PCM device */
	sc->pcm.dev = device_add_child(sc->dev, "pcm", -1);
	if (sc->pcm.dev == NULL) {
		device_printf(sc->dev, "Failed to create PCM device\n");
		return (ENXIO);
	}

	/* Build status string */
	snprintf(status, sizeof(status),
	    "at mmio 0x%lx irq %lu",
	    rman_get_start(sc->mem_res),
	    rman_get_start(sc->irq_res));

	/* Register with sound subsystem */
	error = pcm_register(sc->pcm.dev, status);
	if (error) {
		device_printf(sc->dev, "Failed to register PCM: %d\n", error);
		device_delete_child(sc->dev, sc->pcm.dev);
		sc->pcm.dev = NULL;
		return (error);
	}

	/* Add multiple playback channels for multi-stream support */
	for (i = 0; i < SST_PCM_MAX_PLAY; i++) {
		pcm_addchan(sc->pcm.dev, PCMDIR_PLAY, &sst_chan_class, sc);
	}

	/* Add multiple capture channels */
	for (i = 0; i < SST_PCM_MAX_REC; i++) {
		pcm_addchan(sc->pcm.dev, PCMDIR_REC, &sst_chan_class, sc);
	}

	/* Register mixer */
	error = mixer_init(sc->pcm.dev, &sst_mixer_class, sc);
	if (error) {
		device_printf(sc->dev, "Failed to register mixer: %d\n", error);
	}

	sc->pcm.registered = true;

	device_printf(sc->dev, "PCM device registered: /dev/dsp (%d play, %d rec)\n",
	    SST_PCM_MAX_PLAY, SST_PCM_MAX_REC);

	return (0);
}

/*
 * Unregister PCM device
 */
void
sst_pcm_unregister(struct sst_softc *sc)
{
	if (!sc->pcm.registered)
		return;

	if (sc->pcm.dev != NULL) {
		pcm_unregister(sc->pcm.dev);
		sc->pcm.dev = NULL;
	}

	sc->pcm.registered = false;
}

/*
 * PCM interrupt handler (called from DMA completion)
 * Iterates over all active streams and updates their positions
 */
void
sst_pcm_intr(struct sst_softc *sc, int dir)
{
	struct sst_pcm_channel *ch;
	int i;

	if (dir == PCMDIR_PLAY) {
		for (i = 0; i < SST_PCM_MAX_PLAY; i++) {
			ch = &sc->pcm.play[i];
			if (ch->allocated && ch->state == SST_PCM_STATE_RUNNING) {
				ch->ptr += ch->blk_size;
				if (ch->ptr >= ch->buf_size)
					ch->ptr = 0;
			}
		}
	} else {
		for (i = 0; i < SST_PCM_MAX_REC; i++) {
			ch = &sc->pcm.rec[i];
			if (ch->allocated && ch->state == SST_PCM_STATE_RUNNING) {
				ch->ptr += ch->blk_size;
				if (ch->ptr >= ch->buf_size)
					ch->ptr = 0;
			}
		}
	}
}
