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
#include <dev/sound/pcm/ac97.h>

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
 * Channel init
 */
static void *
sst_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct sst_softc *sc = devinfo;
	struct sst_pcm_channel *ch;
	int error;

	if (dir == PCMDIR_PLAY) {
		ch = &sc->pcm.play;
		ch->ssp_port = 0;	/* SSP0 for playback */
	} else {
		ch = &sc->pcm.rec;
		ch->ssp_port = 1;	/* SSP1 for capture */
	}

	ch->dir = dir;
	ch->state = SST_PCM_STATE_INIT;
	ch->ptr = 0;

	/* Allocate DMA buffer */
	error = sst_pcm_alloc_buffer(sc, ch);
	if (error)
		return (NULL);

	/* Allocate DMA channel */
	ch->dma_ch = sst_dma_alloc(sc);
	if (ch->dma_ch < 0) {
		device_printf(sc->dev, "Failed to allocate DMA channel\n");
		sst_pcm_free_buffer(ch);
		return (NULL);
	}

	/* Setup sound buffer */
	if (sndbuf_setup(b, ch->buf, ch->buf_size) != 0) {
		device_printf(sc->dev, "Failed to setup sound buffer\n");
		sst_dma_free(sc, ch->dma_ch);
		sst_pcm_free_buffer(ch);
		return (NULL);
	}

	/* Default format */
	ch->format = SND_FORMAT(AFMT_S16_LE, 2, 0);
	ch->speed = 48000;
	ch->channels = 2;

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

	sc = ch->dir == PCMDIR_PLAY ?
	    container_of(&ch, struct sst_softc, pcm.play) :
	    container_of(&ch, struct sst_softc, pcm.rec);

	if (ch->state == SST_PCM_STATE_RUNNING)
		sst_ssp_stop(sc, ch->ssp_port);

	if (ch->dma_ch >= 0) {
		sst_dma_free(sc, ch->dma_ch);
		ch->dma_ch = -1;
	}

	sst_pcm_free_buffer(ch);

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
	sc = ch->dir == PCMDIR_PLAY ?
	    container_of(&ch, struct sst_softc, pcm.play) :
	    container_of(&ch, struct sst_softc, pcm.rec);

	switch (go) {
	case PCMTRIG_START:
		if (ch->state == SST_PCM_STATE_RUNNING)
			return (0);

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

		/* Stop SSP first, then DMA */
		sst_ssp_stop(sc, ch->ssp_port);
		sst_dma_stop(sc, ch->dma_ch);

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

	sc = ch->dir == PCMDIR_PLAY ?
	    container_of(&ch, struct sst_softc, pcm.play) :
	    container_of(&ch, struct sst_softc, pcm.rec);

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
	struct sst_softc *sc;

	sc = ch->dir == PCMDIR_PLAY ?
	    container_of(&ch, struct sst_softc, pcm.play) :
	    container_of(&ch, struct sst_softc, pcm.rec);

	/* Update position */
	ch->ptr += ch->blk_size;
	if (ch->ptr >= ch->buf_size)
		ch->ptr = 0;

	/* Notify sound subsystem */
	chn_intr(sc->pcm.dev);
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

	switch (dev) {
	case SOUND_MIXER_VOLUME:
	case SOUND_MIXER_PCM:
		sc->pcm.vol_left = left;
		sc->pcm.vol_right = right;
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
	sc->pcm.sc = sc;
	sc->pcm.dev = NULL;
	sc->pcm.registered = false;

	/* Initialize channel structures */
	sc->pcm.play.dir = PCMDIR_PLAY;
	sc->pcm.play.state = SST_PCM_STATE_INIT;
	sc->pcm.play.dma_ch = -1;
	sc->pcm.play.dma_tag = NULL;

	sc->pcm.rec.dir = PCMDIR_REC;
	sc->pcm.rec.state = SST_PCM_STATE_INIT;
	sc->pcm.rec.dma_ch = -1;
	sc->pcm.rec.dma_tag = NULL;

	/* Default mixer values */
	sc->pcm.vol_left = 100;
	sc->pcm.vol_right = 100;
	sc->pcm.mute = 0;

	device_printf(sc->dev, "PCM subsystem initialized\n");

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
	char status[SND_STATUSLEN];

	if (sc->pcm.registered)
		return (0);

	/* Create PCM device */
	sc->pcm.dev = device_add_child(sc->dev, "pcm", -1);
	if (sc->pcm.dev == NULL) {
		device_printf(sc->dev, "Failed to create PCM device\n");
		return (ENXIO);
	}

	/* Register with sound subsystem */
	error = pcm_register(sc->pcm.dev, sc, 1, 1);
	if (error) {
		device_printf(sc->dev, "Failed to register PCM: %d\n", error);
		device_delete_child(sc->dev, sc->pcm.dev);
		sc->pcm.dev = NULL;
		return (error);
	}

	/* Add channels */
	pcm_addchan(sc->pcm.dev, PCMDIR_PLAY, &sst_chan_class, sc);
	pcm_addchan(sc->pcm.dev, PCMDIR_REC, &sst_chan_class, sc);

	/* Register mixer */
	error = mixer_init(sc->pcm.dev, &sst_mixer_class, sc);
	if (error) {
		device_printf(sc->dev, "Failed to register mixer: %d\n", error);
	}

	/* Build status string */
	snprintf(status, sizeof(status),
	    "at mmio 0x%lx irq %lu on %s",
	    rman_get_start(sc->mem_res),
	    rman_get_start(sc->irq_res),
	    device_get_nameunit(device_get_parent(sc->dev)));

	pcm_setstatus(sc->pcm.dev, status);

	sc->pcm.registered = true;

	device_printf(sc->dev, "PCM device registered: /dev/dsp\n");

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
 */
void
sst_pcm_intr(struct sst_softc *sc, int dir)
{
	struct sst_pcm_channel *ch;

	ch = (dir == PCMDIR_PLAY) ? &sc->pcm.play : &sc->pcm.rec;

	if (ch->state != SST_PCM_STATE_RUNNING)
		return;

	/* Update position and notify */
	ch->ptr += ch->blk_size;
	if (ch->ptr >= ch->buf_size)
		ch->ptr = 0;
}
