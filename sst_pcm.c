/*-
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <sys/callout.h>

#include <machine/bus.h>

#include <dev/sound/pcm/sound.h>

#include "mixer_if.h"

#include "acpi_intel_sst.h"
#include "sst_pcm.h"
#include "sst_topology.h"

/*
 * DMA position polling interval in ticks.
 * The DW-DMAC inside the SST DSP routes its interrupts to the DSP core,
 * not to the host. We poll the DMA position register instead.
 * ~5ms = hz/200 for responsive audio at 48kHz/16bit/2ch with 1KB blocks.
 */
#define SST_PCM_POLL_TICKS	(hz / 200 > 0 ? hz / 200 : 1)

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

/* Volume percent to Q1.31 (defined below, needed by poll timer) */
static uint32_t sst_percent_to_q131(unsigned int pct);

/* DSP stream allocation helpers */
static int sst_pcm_alloc_dsp_stream(struct sst_softc *sc,
    struct sst_pcm_channel *ch);
static void sst_pcm_free_dsp_stream(struct sst_softc *sc,
    struct sst_pcm_channel *ch);

/* Page table helpers */
static int sst_pcm_alloc_pgtbl(struct sst_softc *sc,
    struct sst_pcm_channel *ch);
static void sst_pcm_free_pgtbl(struct sst_pcm_channel *ch);

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
 * DMA callback for page table allocation
 */
static void
sst_pgtbl_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *addr = arg;

	if (error || nseg != 1)
		return;
	*addr = segs[0].ds_addr;
}

/*
 * Allocate page table DMA buffer for DSP ring buffer
 *
 * The catpt firmware expects a page table (array of PFNs) that
 * describes the physical pages of the ring buffer. The DSP reads
 * this page table to set up DMA transfers.
 */
static int
sst_pcm_alloc_pgtbl(struct sst_softc *sc, struct sst_pcm_channel *ch)
{
	uint32_t num_pages, i;
	size_t pgtbl_size;
	int error;

	num_pages = (ch->buf_size + PAGE_SIZE - 1) / PAGE_SIZE;
	if (num_pages > SST_MAX_RING_PAGES)
		num_pages = SST_MAX_RING_PAGES;

	/* Allocate full page like Linux catpt - packed 20-bit PFNs need
	 * 2.5 bytes each, but DSP may read beyond num_pages entries */
	pgtbl_size = PAGE_SIZE;

	/* Create DMA tag for page table */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),
	    4, 0,			/* 4-byte alignment */
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    pgtbl_size, 1, pgtbl_size,
	    0, NULL, NULL,
	    &ch->pgtbl_tag);
	if (error) {
		device_printf(sc->dev, "PCM: pgtbl tag create failed: %d\n",
		    error);
		return (error);
	}

	/* Allocate page table buffer */
	error = bus_dmamem_alloc(ch->pgtbl_tag, (void **)&ch->pgtbl_buf,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO, &ch->pgtbl_map);
	if (error) {
		device_printf(sc->dev, "PCM: pgtbl alloc failed: %d\n", error);
		bus_dma_tag_destroy(ch->pgtbl_tag);
		ch->pgtbl_tag = NULL;
		return (error);
	}

	/* Map page table buffer */
	error = bus_dmamap_load(ch->pgtbl_tag, ch->pgtbl_map, ch->pgtbl_buf,
	    pgtbl_size, sst_pgtbl_cb, &ch->pgtbl_addr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->dev, "PCM: pgtbl map failed: %d\n", error);
		bus_dmamem_free(ch->pgtbl_tag, ch->pgtbl_buf, ch->pgtbl_map);
		bus_dma_tag_destroy(ch->pgtbl_tag);
		ch->pgtbl_tag = NULL;
		ch->pgtbl_buf = NULL;
		return (error);
	}

	/*
	 * Fill page table with packed 20-bit PFN entries.
	 *
	 * Linux catpt uses catpt_arrange_page_table() which packs
	 * two 20-bit PFNs into every 5 bytes (40 bits):
	 *   Even entry i: pfn stored at byte offset (i*5)/2, bits [19:0]
	 *   Odd entry i:  pfn stored at byte offset (i*5)/2, bits [23:4]
	 *
	 * This packing is critical - the DSP firmware expects this
	 * exact format. Using simple uint32_t PFNs causes the DSP
	 * to read from wrong physical addresses → audio distortion.
	 */
	{
		uint8_t *pt = (uint8_t *)ch->pgtbl_buf;

		memset(pt, 0, PAGE_SIZE);
		for (i = 0; i < num_pages; i++) {
			uint32_t pfn, offset;
			uint32_t *entry;

			pfn = (uint32_t)((ch->dma_addr + i * PAGE_SIZE)
			    >> PAGE_SHIFT);
			/* Byte offset: (i*4 + i) / 2 = (i*5) / 2 */
			offset = ((i << 2) + i) >> 1;
			entry = (uint32_t *)(pt + offset);

			if (i & 1)
				*entry |= (pfn << 4);
			else
				*entry |= pfn;
		}
	}

	/* Sync to device */
	bus_dmamap_sync(ch->pgtbl_tag, ch->pgtbl_map,
	    BUS_DMASYNC_PREWRITE);

	device_printf(sc->dev,
	    "PCM: pgtbl allocated: %u pages, phys=0x%lx, first_pfn=0x%x\n",
	    num_pages, (unsigned long)ch->pgtbl_addr,
	    (uint32_t)(ch->dma_addr >> PAGE_SHIFT));

	return (0);
}

/*
 * Free page table DMA buffer
 */
static void
sst_pcm_free_pgtbl(struct sst_pcm_channel *ch)
{
	if (ch->pgtbl_map != NULL) {
		bus_dmamap_unload(ch->pgtbl_tag, ch->pgtbl_map);
	}
	if (ch->pgtbl_buf != NULL) {
		bus_dmamem_free(ch->pgtbl_tag, ch->pgtbl_buf, ch->pgtbl_map);
		ch->pgtbl_buf = NULL;
		ch->pgtbl_map = NULL;
	}
	if (ch->pgtbl_tag != NULL) {
		bus_dma_tag_destroy(ch->pgtbl_tag);
		ch->pgtbl_tag = NULL;
	}
	ch->pgtbl_addr = 0;
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

	device_printf(sc->dev, "PCM: chan_init dir=%d b=%p c=%p\n",
	    dir, b, c);

	/* Allocate channel slot from pool */
	ch = sst_pcm_alloc_channel(sc, dir);
	if (ch == NULL) {
		device_printf(sc->dev, "No free %s channel slots\n",
		    (dir == PCMDIR_PLAY) ? "playback" : "capture");
		return (NULL);
	}

	device_printf(sc->dev, "PCM: chan slot %d allocated\n", ch->index);

	/* SSP port: 0 for playback, 1 for capture */
	ch->ssp_port = (dir == PCMDIR_PLAY) ? 0 : 1;
	ch->dir = dir;
	ch->state = SST_PCM_STATE_INIT;
	ch->ptr = 0;
	ch->pcm_ch = c;
	ch->sndbuf = b;

	/* Allocate DMA buffer */
	device_printf(sc->dev, "PCM: allocating DMA buffer\n");
	error = sst_pcm_alloc_buffer(sc, ch);
	if (error) {
		device_printf(sc->dev, "PCM: DMA buffer alloc failed: %d\n",
		    error);
		sst_pcm_release_channel(sc, ch);
		return (NULL);
	}

	device_printf(sc->dev, "PCM: DMA buf=%p size=%zu\n",
	    ch->buf, ch->buf_size);

	/* Allocate page table for DSP ring buffer */
	error = sst_pcm_alloc_pgtbl(sc, ch);
	if (error) {
		device_printf(sc->dev, "PCM: pgtbl alloc failed: %d\n", error);
		sst_pcm_free_buffer(ch);
		sst_pcm_release_channel(sc, ch);
		return (NULL);
	}

	/* DMA channel no longer needed - DSP manages DMA internally */
	ch->dma_ch = -1;

	/* Setup sound buffer */
	device_printf(sc->dev, "PCM: sndbuf_setup buf=%p size=%zu\n",
	    ch->buf, ch->buf_size);
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

	/* Initialize position polling callout */
	callout_init(&ch->poll_timer, 1);

	/* DSP stream not yet allocated */
	ch->stream_id = 0;
	ch->stream_allocated = false;
	ch->read_pos_regaddr = 0;

	device_printf(sc->dev, "PCM: Allocated %s channel %d\n",
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

	/* Stop and drain polling timer */
	callout_drain(&ch->poll_timer);

	/* Free DSP stream if allocated */
	if (ch->stream_allocated)
		sst_pcm_free_dsp_stream(sc, ch);

	/* Free page table */
	sst_pcm_free_pgtbl(ch);

	/* Free DMA buffer */
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
 * Allocate DSP stream via IPC (catpt protocol)
 *
 * The catpt firmware manages DMA and SSP internally.
 * We just provide the ring buffer page table and audio format.
 * The DSP returns position register addresses for polling.
 */
static int
sst_pcm_alloc_dsp_stream(struct sst_softc *sc, struct sst_pcm_channel *ch)
{
	struct sst_alloc_stream_req req;
	struct sst_alloc_stream_rsp rsp;
	struct sst_module_entry mod;
	uint32_t mod_id;
	int error;

	if (ch->stream_allocated)
		return (0);

	memset(&req, 0, sizeof(req));
	memset(&rsp, 0, sizeof(rsp));

	/*
	 * Path and stream type (uint32_t - matches catpt enum size).
	 *
	 * Use SYSTEM type for normal PCM playback, not RENDER.
	 * RENDER is for offload (compressed) streams that require
	 * explicit SET_WRITE_POS ping-pong.  SYSTEM streams let the
	 * DSP manage the ring buffer autonomously.
	 */
	if (ch->dir == PCMDIR_PLAY) {
		req.path_id = SST_PATH_SSP0_OUT;
		req.stream_type = SST_STREAM_TYPE_SYSTEM;
		mod_id = SST_MODID_PCM_SYSTEM;
	} else {
		req.path_id = SST_PATH_SSP0_IN;
		req.stream_type = SST_STREAM_TYPE_CAPTURE;
		mod_id = SST_MODID_PCM_CAPTURE;
	}
	req.format_id = SST_FMT_PCM;

	/* Audio format (catpt_audio_format layout - 24 bytes) */
	req.format.sample_rate = ch->speed;
	req.format.bit_depth = AFMT_BIT(ch->format);
	req.format.valid_bit_depth = req.format.bit_depth;
	req.format.num_channels = ch->channels;
	req.format.interleaving = SST_INTERLEAVING_PER_CHANNEL;

	if (ch->channels == 2) {
		req.format.channel_config = SST_CHAN_CONFIG_STEREO;
		/*
		 * Channel map: each nibble encodes a catpt_channel_index
		 * for that output slot.  Unused nibbles = 0xF (INVALID).
		 *
		 * Linux catpt_get_channel_map(STEREO):
		 *   GENMASK(31,8) | CATPT_CHANNEL_LEFT(0)
		 *                 | (CATPT_CHANNEL_RIGHT(2) << 4)
		 *   = 0xFFFFFF00 | 0x00 | 0x20 = 0xFFFFFF20
		 */
		req.format.channel_map = 0xFFFFFF20;
	} else {
		req.format.channel_config = SST_CHAN_CONFIG_MONO;
		/* Mono: LEFT only, rest = 0xF */
		req.format.channel_map = 0xFFFFFFF0;
	}

	/*
	 * Ring buffer configuration (catpt protocol).
	 *
	 * From Linux catpt pcm.c (catpt_dai_hw_params):
	 *   page_table_addr = physical addr of PFN array
	 *   num_pages = buffer_bytes >> PAGE_SHIFT
	 *   size = total ring buffer size in bytes
	 *   offset = 0
	 *   first_pfn = first entry from PFN table
	 */
	{
		uint32_t num_pages;

		num_pages = (ch->buf_size + PAGE_SIZE - 1) / PAGE_SIZE;
		if (num_pages > SST_MAX_RING_PAGES)
			num_pages = SST_MAX_RING_PAGES;

		req.ring.page_table_addr = (uint32_t)ch->pgtbl_addr;
		req.ring.num_pages = num_pages;
		req.ring.size = ch->buf_size;
		req.ring.offset = 0;
		/*
		 * first_pfn = PFN_DOWN(first page physical address).
		 * Computed directly from dma_addr, not from packed
		 * page table (which uses 20-bit packed format).
		 */
		req.ring.first_pfn = (uint32_t)(ch->dma_addr
		    >> PAGE_SHIFT);
	}

	/* Module entry */
	req.num_entries = 1;
	memset(&mod, 0, sizeof(mod));
	mod.module_id = mod_id;
	if (mod_id < SST_MAX_MODULES && sc->fw.mod[mod_id].present)
		mod.entry_point = sc->fw.mod[mod_id].entry_point;

	/*
	 * Persistent and scratch memory from firmware module info.
	 * Offsets are DSP absolute addresses (DRAM base = 0x400000).
	 */
	if (mod_id < SST_MAX_MODULES && sc->fw.mod[mod_id].present) {
		req.persistent_mem.offset = SST_DSP_DRAM_OFFSET +
		    sc->fw.mod[mod_id].persistent_offset;
		req.persistent_mem.size =
		    sc->fw.mod[mod_id].persistent_size;
		if (sc->fw.mod[mod_id].scratch_size > 0) {
			req.scratch_mem.offset = SST_DSP_DRAM_OFFSET +
			    sc->fw.mod[mod_id].scratch_offset;
			req.scratch_mem.size =
			    sc->fw.mod[mod_id].scratch_size;
		}
	}

	req.num_notifications = 0; /* Linux catpt sends 0 */

	device_printf(sc->dev,
	    "DSP stream alloc: type=%u path=%u fmt=%u mod=0x%x "
	    "rate=%u depth=%u ch=%u pgtbl=0x%x pages=%u "
	    "persist=0x%x/%u scratch=0x%x/%u\n",
	    req.stream_type, req.path_id, req.format_id, mod_id,
	    ch->speed, req.format.bit_depth, ch->channels,
	    req.ring.page_table_addr, req.ring.num_pages,
	    req.persistent_mem.offset, req.persistent_mem.size,
	    req.scratch_mem.offset, req.scratch_mem.size);

	/* Allocate stream on DSP */
	error = sst_ipc_alloc_stream(sc, &req, &mod, 1, &rsp);
	if (error) {
		device_printf(sc->dev,
		    "DSP stream alloc failed: %d (status=INVALID_PARAM?)\n",
		    error);
		return (error);
	}

	ch->stream_id = rsp.stream_hw_id;
	ch->stream_allocated = true;
	ch->read_pos_regaddr = rsp.read_pos_regaddr;

	device_printf(sc->dev,
	    "PCM: Allocated DSP stream %u for %s "
	    "(read_pos=0x%x pres_pos=0x%x)\n",
	    ch->stream_id,
	    (ch->dir == PCMDIR_PLAY) ? "playback" : "capture",
	    rsp.read_pos_regaddr, rsp.pres_pos_regaddr);

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

	/*
	 * Linux catpt hw_free sequence: RESET then FREE.
	 * The DSP requires the stream to be reset before freeing.
	 */
	sst_ipc_stream_reset(sc, ch->stream_id);
	sst_ipc_free_stream(sc, ch->stream_id);
	ch->stream_allocated = false;
	ch->stream_id = 0;
}

/*
 * DSP position polling callback
 *
 * The DSP firmware writes the current playback/capture position to
 * a register in DRAM (read_pos_regaddr, returned by ALLOC_STREAM).
 * We poll this register to detect block boundary crossings and
 * notify sound(4) via chn_intr().
 */
static void
sst_pcm_poll(void *arg)
{
	struct sst_pcm_channel *ch = arg;
	struct sst_softc *sc = ch->sc;
	uint32_t pos;

	if (ch->state != SST_PCM_STATE_RUNNING)
		return;

	/* Read DSP position register */
	if (ch->read_pos_regaddr != 0)
		pos = bus_read_4(sc->mem_res, ch->read_pos_regaddr);
	else
		pos = 0;

	/* Wrap within buffer */
	if (ch->buf_size > 0)
		pos = pos % ch->buf_size;

	/* Debug: show first 3 polls after each start */
	if (ch->dbg_polls < 3) {
		device_printf(sc->dev,
		    "poll[%d]: pos=%u last=%u (buf=%zu blk=%u)\n",
		    ch->dbg_polls, pos, ch->last_pos,
		    ch->buf_size, ch->blk_size);
		ch->dbg_polls++;
	}

	/* If position crossed a block boundary, notify sound(4) */
	if (ch->blk_size > 0 &&
	    pos / ch->blk_size != ch->last_pos / ch->blk_size) {
		ch->ptr = pos;
		chn_intr(ch->pcm_ch);
		ch->stall_count = 0;
	} else if (pos == ch->last_pos) {
		ch->stall_count++;
	}

	/*
	 * DSP stream stall recovery.
	 *
	 * The catpt DSP firmware can stall its DMA engine after
	 * processing many SET_VOLUME IPC messages.  The position
	 * register stops advancing but the DSP core stays alive
	 * (IPC still works).  Recover by re-issuing RESUME.
	 *
	 * 200 polls at ~5ms = 1 second of no movement.
	 */
	if (ch->stall_count == 200 && ch->stream_allocated) {
		device_printf(sc->dev,
		    "PCM: stream %u stalled at pos=%u, restarting\n",
		    ch->stream_id, pos);
		/*
		 * Full stream restart: FREE old stream, ALLOC new one.
		 *
		 * The catpt DSP's DMA engine can stall after many
		 * SET_VOLUME IPCs.  Neither RESUME nor RESET recovers
		 * it.  The only fix is to tear down and reallocate
		 * the entire stream, keeping the same ring buffer.
		 */
		sst_pcm_free_dsp_stream(sc, ch);

		if (sst_pcm_alloc_dsp_stream(sc, ch) == 0) {
			sst_ipc_stream_reset(sc, ch->stream_id);
			sst_ipc_stream_pause(sc, ch->stream_id);

			sst_shim_update_bits(sc, SST_SHIM_HMDC,
			    SST_HMDC_HDDA_ALL, SST_HMDC_HDDA_ALL);

			sst_ipc_stream_resume(sc, ch->stream_id);
			sst_ssp_start(sc, ch->ssp_port);
			sst_codec_pll_rearm(sc);

			/* Restore current volume */
			{
				struct sst_stream_params vsp;
				memset(&vsp, 0, sizeof(vsp));
				vsp.stream_id = ch->stream_id;
				vsp.volume_left =
				    sst_percent_to_q131(sc->pcm.vol_left);
				vsp.volume_right =
				    sst_percent_to_q131(sc->pcm.vol_right);
				sst_ipc_stream_set_params(sc, &vsp);
			}

			device_printf(sc->dev,
			    "PCM: stream recovered as %u\n",
			    ch->stream_id);
		} else {
			device_printf(sc->dev,
			    "PCM: stream recovery failed\n");
		}
		ch->stall_count = 0;
	}

	ch->last_pos = pos;

	/*
	 * Flush deferred volume update.
	 *
	 * If the mixer rate-limiter deferred a SET_VOLUME, send
	 * the final value now (from callout context, outside the
	 * rapid ioctl storm).  Only flush once per deferral.
	 */
	if (ch->dir == PCMDIR_PLAY && sc->pcm.vol_pending) {
		struct sst_stream_params vsp;
		int elapsed = ticks - sc->pcm.vol_ticks;

		if (elapsed >= hz / 2 && ch->stream_allocated) {
			memset(&vsp, 0, sizeof(vsp));
			vsp.stream_id = ch->stream_id;
			vsp.volume_left =
			    sst_percent_to_q131(sc->pcm.vol_left);
			vsp.volume_right =
			    sst_percent_to_q131(sc->pcm.vol_right);
			vsp.mute = sc->pcm.mute;
			sst_ipc_stream_set_params(sc, &vsp);
			sc->pcm.vol_ticks = ticks;
			sc->pcm.vol_pending = false;
		}
	}

	/* Reschedule */
	callout_reset(&ch->poll_timer, SST_PCM_POLL_TICKS,
	    sst_pcm_poll, ch);
}

/*
 * Trigger (start/stop)
 *
 * Uses the catpt IPC protocol for all stream management.
 * The DSP firmware manages DMA and SSP internally.
 * Flow:
 *   START: alloc_stream → set_device_formats → resume → set_write_pos
 *   STOP:  pause → free_stream
 */
static int
sst_chan_trigger(kobj_t obj, void *data, int go)
{
	struct sst_pcm_channel *ch = data;
	struct sst_softc *sc;
	int error;

	sc = ch->sc;

	switch (go) {
	case PCMTRIG_START:
		if (ch->state == SST_PCM_STATE_RUNNING)
			return (0);

		/*
		 * Skip capture stream allocation for now.
		 * The DSP firmware doesn't support simultaneous
		 * playback and capture on the same SSP port.
		 */
		if (ch->dir == PCMDIR_REC) {
			ch->state = SST_PCM_STATE_RUNNING;
			return (0);
		}

		/* DSP firmware must be running */
		if (sc->fw.state != SST_FW_STATE_RUNNING) {
			device_printf(sc->dev,
			    "PCM trigger: DSP firmware not running\n");
			return (ENXIO);
		}

		/*
		 * Step 1: Allocate DSP stream.
		 * SET_DEVICE_FORMATS was already sent once at init
		 * time (like Linux catpt_dai_pcm_new).
		 */
		error = sst_pcm_alloc_dsp_stream(sc, ch);
		if (error) {
			device_printf(sc->dev,
			    "DSP stream alloc failed: %d\n", error);
			return (error);
		}

		/*
		 * Step 2: Prepare stream (catpt prepare sequence).
		 * Linux catpt does RESET+PAUSE in prepare() before
		 * trigger START, putting stream into a known state.
		 */
		sst_ipc_stream_reset(sc, ch->stream_id);
		sst_ipc_stream_pause(sc, ch->stream_id);

		/*
		 * Step 3: Enable HMDC (Host Memory DMA Control).
		 *
		 * The DSP's DMA engine accesses host memory through
		 * the PCH IOSF fabric.  HMDC bits must be set to
		 * allow DMA reads from the host page table and audio
		 * buffer.  Without this, the DSP plays silence.
		 */
		sst_shim_update_bits(sc, SST_SHIM_HMDC,
		    SST_HMDC_HDDA_ALL, SST_HMDC_HDDA_ALL);

		/* Step 4: Resume (start) DSP stream */
		error = sst_ipc_stream_resume(sc, ch->stream_id);
		if (error) {
			device_printf(sc->dev,
			    "DSP stream resume failed: %d\n", error);
			sst_pcm_free_dsp_stream(sc, ch);
			return (error);
		}

		/*
		 * Step 5: Enable SSP (set SSE bit).
		 *
		 * The firmware programs SSP registers via SET_DEVICE_FORMATS
		 * but does NOT set SSE (SSP Enable).  We must do it explicitly
		 * as the catpt firmware leaves this to the host.
		 */
		sst_ssp_start(sc, ch->ssp_port);

		/*
		 * Step 6: Re-enable codec PLL now that BCLK is active.
		 *
		 * The SSP is now clocking I2S (BCLK + LRCLK).  The
		 * codec PLL needs BCLK to lock.  Re-poke PLL enable
		 * and DAC format so codec locks to active BCLK.
		 */
		sst_codec_pll_rearm(sc);

		/* Initialize position tracking */
		ch->ptr = 0;
		ch->last_pos = 0;
		ch->dbg_polls = 0;
		ch->state = SST_PCM_STATE_RUNNING;

		device_printf(sc->dev,
		    "PCM: %s started (stream=%u read_pos=0x%x)\n",
		    (ch->dir == PCMDIR_PLAY) ? "playback" : "capture",
		    ch->stream_id, ch->read_pos_regaddr);

		/* Start position polling timer */
		callout_reset(&ch->poll_timer, SST_PCM_POLL_TICKS,
		    sst_pcm_poll, ch);
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (ch->state != SST_PCM_STATE_RUNNING)
			return (0);

		/* Stop polling first */
		callout_stop(&ch->poll_timer);

		/* Disable SSP (clear SSE bit) */
		sst_ssp_stop(sc, ch->ssp_port);

		/* Pause DSP stream */
		if (ch->stream_allocated) {
			error = sst_ipc_stream_pause(sc, ch->stream_id);
			if (error)
				device_printf(sc->dev,
				    "DSP stream pause failed: %d\n", error);
		}

		/* Free DSP stream */
		if (ch->stream_allocated)
			sst_pcm_free_dsp_stream(sc, ch);

		ch->state = SST_PCM_STATE_PREPARED;

		device_printf(sc->dev, "PCM: %s stopped\n",
		    (ch->dir == PCMDIR_PLAY) ? "playback" : "capture");
		break;

	case PCMTRIG_EMLDMAWR:
		/*
		 * Emulated DMA write notification from sound(4).
		 * Not needed for SYSTEM type streams - the DSP reads
		 * the ring buffer autonomously via the page table.
		 */
		break;
	case PCMTRIG_EMLDMARD:
		break;
	}

	return (0);
}

/*
 * Get current buffer position
 *
 * The DSP writes the current read position to a DRAM register
 * (read_pos_regaddr) which we can read via MMIO.
 */
static uint32_t
sst_chan_getptr(kobj_t obj, void *data)
{
	struct sst_pcm_channel *ch = data;
	struct sst_softc *sc;
	uint32_t pos;

	sc = ch->sc;

	if (ch->state != SST_PCM_STATE_RUNNING)
		return (0);

	/* Read DSP position register */
	if (ch->read_pos_regaddr != 0) {
		pos = bus_read_4(sc->mem_res, ch->read_pos_regaddr);
		return (pos % ch->buf_size);
	}

	return (ch->ptr % ch->buf_size);
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
 * Mixer percent (0-100) to Q1.31 lookup table.
 * Logarithmic curve: 0% = silence, 100% = -1dBFS (headroom).
 * Maps percent to dB range -60dB..(-1dB), then to Q1.31.
 */
static const uint32_t sst_pct_to_q131[101] = {
	0x00000000, 0x00231237, 0x00258942, 0x00282CA9,
	0x002AFF88, 0x002E0536, 0x00314145, 0x0034B787,
	0x00386C15, 0x003C634D, 0x0040A1E2, 0x00452CD5,
	0x004A0985, 0x004F3DB2, 0x0054CF81, 0x005AC587,
	0x006126CF, 0x0067FAE3, 0x006F49D5, 0x00771C48,
	0x007F7B79, 0x0088714D, 0x0092085B, 0x009C4BF8,
	0x00A74843, 0x00B30A3A, 0x00BF9FBF, 0x00CD17B3,
	0x00DB81FE, 0x00EAEFAB, 0x00FB72F2, 0x010D1F59,
	0x012009C1, 0x01344883, 0x0149F38D, 0x01612478,
	0x0179F6AC, 0x0194877D, 0x01B0F650, 0x01CF64BC,
	0x01EFF6B8, 0x0212D2C0, 0x02382205, 0x0260109D,
	0x028ACDB8, 0x02B88BD5, 0x02E98101, 0x031DE717,
	0x0355FC01, 0x03920204, 0x03D2400C, 0x04170202,
	0x04609926, 0x04AF5C6D, 0x0503A8E7, 0x055DE233,
	0x05BE72EB, 0x0625CD2A, 0x06946B12, 0x070ACF56,
	0x078985DC, 0x0811245F, 0x08A24B1F, 0x093DA5A0,
	0x09E3EB74, 0x0A95E115, 0x0B5458CA, 0x0C2033A4,
	0x0CFA6284, 0x0DE3E73B, 0x0EDDD5B7, 0x0FE9554B,
	0x1107A20F, 0x123A0E4E, 0x1382041D, 0x14E10703,
	0x1658B5C4, 0x17EACC4D, 0x199925BA, 0x1B65BE91,
	0x1D52B712, 0x1F6255C1, 0x21970A11, 0x23F36F48,
	0x267A4F92, 0x292EA74F, 0x2C13A895, 0x2F2CBEFE,
	0x327D93AE, 0x360A11A4, 0x39D66A63, 0x3DE71AE0,
	0x4240F0CF, 0x46E91057, 0x4BE4FA1E, 0x513A91CE,
	0x56F02506, 0x5D0C72D3, 0x6396B3A2, 0x6A96A1CF,
	0x721482BF,
};

/*
 * Convert mixer percent (0-100) to Q1.31 linear gain.
 */
static uint32_t
sst_percent_to_q131(unsigned int pct)
{

	if (pct > 100)
		pct = 100;
	return (sst_pct_to_q131[pct]);
}

/*
 * HPF cutoff frequencies for BASS mixer control.
 * Index 0 = bypass, indices 1-8 = 80..300 Hz presets.
 * Mixer value 0 = off, 1-100 mapped across presets via:
 *   idx = 1 + (val - 1) * 8 / 100
 */
static const uint32_t sst_hpf_cutoffs[] = {
	0, 80, 100, 120, 150, 180, 200, 250, 300
};

/*
 * Mixer methods
 */
static int
sst_mixer_init(struct snd_mixer *m)
{
	struct sst_softc *sc = mix_getdevinfo(m);
	uint32_t devs;

	/* Register mixer controls based on DSP capabilities */
	devs = SOUND_MASK_PCM | SOUND_MASK_VOLUME;
	if (sc->fw.has_biquad)
		devs |= SOUND_MASK_BASS;
	if (sc->fw.has_limiter)
		devs |= SOUND_MASK_TREBLE;
	mix_setdevs(m, devs);

	/* Set initial volumes */
	sc->pcm.vol_left = 100;
	sc->pcm.vol_right = 100;
	sc->pcm.mute = 0;

	/* Default HPF: 150 Hz (mixer bass = 50) */
	sc->pcm.hpf_cutoff = 150;

	/* Default limiter: -6 dBFS (mixer treble = 60, preset index 5) */
	sc->pcm.limiter_threshold = 5;

	return (0);
}

static int
sst_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct sst_softc *sc = mix_getdevinfo(m);
	struct sst_stream_params sp;
	int i;

	switch (dev) {
	case SOUND_MIXER_VOLUME:
	case SOUND_MIXER_PCM:
		sc->pcm.vol_left = left;
		sc->pcm.vol_right = right;

		/*
		 * Rate-limit SET_VOLUME IPC to prevent flooding the DSP.
		 *
		 * Rapid mixer slider drags can generate dozens of volume
		 * changes per second.  Each sends 2 IPC messages (L+R).
		 * The DSP handles IPC synchronously, starving its DMA
		 * engine and killing the stream.
		 *
		 * Throttle to at most one update per 100ms.  Intermediate
		 * values are stored and sent by the poll timer as a
		 * deferred update.
		 */
		{
			int now = ticks;
			int elapsed = now - sc->pcm.vol_ticks;

			if (elapsed < hz / 2 && sc->pcm.vol_ticks != 0) {
				/* Too soon — defer to poll timer */
				sc->pcm.vol_pending = true;
				break;
			}
			sc->pcm.vol_ticks = now;
			sc->pcm.vol_pending = false;
		}

		/* Update volume on all active playback streams */
		for (i = 0; i < SST_PCM_MAX_PLAY; i++) {
			if (sc->pcm.play[i].stream_allocated) {
				memset(&sp, 0, sizeof(sp));
				sp.stream_id = sc->pcm.play[i].stream_id;
				sp.volume_left = sst_percent_to_q131(left);
				sp.volume_right = sst_percent_to_q131(right);
				sp.mute = sc->pcm.mute;
				sst_ipc_stream_set_params(sc, &sp);
			}
		}
		break;

	case SOUND_MIXER_BASS: {
		struct sst_widget *hpf_w;
		uint32_t cutoff;
		int idx;

		/*
		 * Map mixer value (0-100) to HPF cutoff preset.
		 * 0 = HPF off (bypass), 1-100 = 80..300 Hz.
		 * Only the left channel value is used (mono control).
		 */
		if (left == 0) {
			idx = 0;
		} else {
			idx = 1 + (left - 1) * 8 / 100;
			if (idx > 8)
				idx = 8;
		}
		cutoff = sst_hpf_cutoffs[idx];
		sc->pcm.hpf_cutoff = cutoff;

		/* Update HPF widget if topology is loaded */
		hpf_w = sst_topology_find_widget(sc, "HPF1.0");
		if (hpf_w != NULL)
			sst_topology_set_widget_hpf(sc, hpf_w, cutoff);

		break;
	}

	case SOUND_MIXER_TREBLE: {
		struct sst_widget *lim_w;
		int idx;

		/*
		 * Map mixer value (0-100) to limiter threshold preset.
		 * 0 = limiter off (bypass), 1-100 = -24dB..0dB.
		 * Only the left channel value is used (mono control).
		 */
		if (left == 0) {
			idx = 0;
		} else {
			idx = 1 + (left - 1) * 7 / 99;
			if (idx > 8)
				idx = 8;
		}
		sc->pcm.limiter_threshold = idx;

		/* Update limiter widget if topology is loaded */
		lim_w = sst_topology_find_widget(sc, "LIMITER1.0");
		if (lim_w != NULL)
			sst_topology_set_widget_limiter(sc, lim_w, idx);

		break;
	}

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
		sc->pcm.play[i].read_pos_regaddr = 0;
		sc->pcm.play[i].pgtbl_tag = NULL;
		sc->pcm.play[i].pgtbl_buf = NULL;
		sc->pcm.play[i].pgtbl_addr = 0;
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
		sc->pcm.rec[i].read_pos_regaddr = 0;
		sc->pcm.rec[i].pgtbl_tag = NULL;
		sc->pcm.rec[i].pgtbl_buf = NULL;
		sc->pcm.rec[i].pgtbl_addr = 0;
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

	/*
	 * Delete any remaining children (e.g. stale pcm devices left
	 * over from a previous module load that didn't get cleaned up).
	 */
	device_delete_children(sc->dev);
}

/*
 * Register PCM device with sound(4)
 *
 * Sound(4) requires device_get_softc() to return a snddev_info of
 * size PCM_SOFTC_SIZE.  Since our ACPI device uses sst_softc, we
 * create a child "pcm" device whose softc is owned by sound(4).
 * Our sst_softc is passed via device_set_ivars().
 */
int
sst_pcm_register(struct sst_softc *sc)
{
	device_printf(sc->dev, "PCM: Creating child pcm device\n");

	if (sc->pcm.pcm_dev != NULL || sc->pcm.registered) {
		device_printf(sc->dev, "PCM: Already registered\n");
		return (0);
	}

	/*
	 * Clean up any stale children from a previous module load.
	 * After kldunload + kldload, old child devices may still exist
	 * with ivars pointing to the old (freed) softc.
	 */
	device_delete_children(sc->dev);

	sc->pcm.pcm_dev = device_add_child(sc->dev, "pcm", DEVICE_UNIT_ANY);
	if (sc->pcm.pcm_dev == NULL) {
		device_printf(sc->dev, "PCM: Failed to add child device\n");
		return (ENXIO);
	}

	/* Pass our softc to the child via ivars */
	device_set_ivars(sc->pcm.pcm_dev, sc);
	device_printf(sc->dev,
	    "PCM: child=%p ivars set to sc=%p sc->dev=%p\n",
	    sc->pcm.pcm_dev, sc, sc->dev);

	/*
	 * bus_attach_children triggers child probe+attach synchronously.
	 * bus_generic_driver_added (our bus method) also calls it, so
	 * the child may already be attached by the time we return.
	 * Mark pcm_dev non-NULL above to prevent re-entry.
	 */
	bus_attach_children(sc->dev);
	return (0);
}

/*
 * Unregister PCM device
 */
void
sst_pcm_unregister(struct sst_softc *sc)
{
	if (!sc->pcm.registered && sc->pcm.pcm_dev == NULL)
		return;

	if (sc->pcm.pcm_dev != NULL) {
		if (sc->pcm.registered)
			pcm_unregister(sc->pcm.pcm_dev);
		/*
		 * Delete the child device so it doesn't survive a
		 * module reload with stale ivars pointing to freed
		 * memory (the old softc).
		 */
		device_delete_child(sc->dev, sc->pcm.pcm_dev);
		sc->pcm.pcm_dev = NULL;
	}

	sc->pcm.registered = false;
}

/* ================================================================
 * Child PCM Driver (probe/attach/detach)
 *
 * This driver attaches as a child of acpi_intel_sst.  Its softc is
 * PCM_SOFTC_SIZE (struct snddev_info) so that sound(4) can use it.
 * The parent's sst_softc is retrieved via device_get_ivars().
 * ================================================================ */

static int
sst_pcm_child_probe(device_t dev)
{
	device_set_desc(dev, "Intel SST Audio");
	return (BUS_PROBE_DEFAULT);
}

static int
sst_pcm_child_attach(device_t dev)
{
	struct sst_softc *sc;
	char status[SND_STATUSLEN];
	int error, i;

	sc = device_get_ivars(dev);
	if (sc == NULL || sc->dev == NULL) {
		device_printf(dev,
		    "PCM: invalid ivars sc=%p sc->dev=%p\n",
		    sc, sc ? sc->dev : NULL);
		return (ENXIO);
	}

	device_printf(dev, "PCM child attach: parent=%s sc=%p\n",
	    device_get_nameunit(sc->dev), sc);

	/* Build status string */
	if (sc->irq_res != NULL) {
		snprintf(status, sizeof(status),
		    "at mmio 0x%lx irq %lu",
		    rman_get_start(sc->mem_res),
		    rman_get_start(sc->irq_res));
	} else {
		snprintf(status, sizeof(status),
		    "at mmio 0x%lx (polled)",
		    rman_get_start(sc->mem_res));
	}

	/*
	 * sound(4) registration on the CHILD device:
	 * device_get_softc(dev) returns snddev_info (PCM_SOFTC_SIZE).
	 * Our sst_softc is passed as devinfo to pcm_init.
	 */
	pcm_init(dev, sc);

	/* Add playback channels */
	for (i = 0; i < SST_PCM_MAX_PLAY; i++)
		pcm_addchan(dev, PCMDIR_PLAY, &sst_chan_class, sc);

	/* Add capture channels */
	for (i = 0; i < SST_PCM_MAX_REC; i++)
		pcm_addchan(dev, PCMDIR_REC, &sst_chan_class, sc);

	error = pcm_register(dev, status);
	if (error) {
		device_printf(dev, "PCM: pcm_register failed: %d\n", error);
		return (error);
	}

	/* Register mixer */
	mixer_init(dev, &sst_mixer_class, sc);

	sc->pcm.registered = true;

	device_printf(dev, "PCM registered: %s (%d play, %d rec)\n",
	    status, SST_PCM_MAX_PLAY, SST_PCM_MAX_REC);

	return (0);
}

static int
sst_pcm_child_detach(device_t dev)
{
	struct sst_softc *sc;
	int error;

	sc = device_get_ivars(dev);

	/* Only unregister if registration actually succeeded */
	if (sc != NULL && sc->pcm.registered) {
		error = pcm_unregister(dev);
		if (error)
			return (error);
		sc->pcm.registered = false;
	}

	return (0);
}

static device_method_t sst_pcm_methods[] = {
	DEVMETHOD(device_probe,		sst_pcm_child_probe),
	DEVMETHOD(device_attach,	sst_pcm_child_attach),
	DEVMETHOD(device_detach,	sst_pcm_child_detach),
	DEVMETHOD_END
};

static driver_t sst_pcm_driver = {
	"pcm",
	sst_pcm_methods,
	PCM_SOFTC_SIZE,
};

DRIVER_MODULE(snd_intel_sst, acpi_intel_sst, sst_pcm_driver, NULL, NULL);

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
