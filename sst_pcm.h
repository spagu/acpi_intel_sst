/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Intel SST PCM Driver - sound(4) Integration
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_PCM_H_
#define _SST_PCM_H_

#include <sys/types.h>
#include <sys/callout.h>

/*
 * PCM Channel Configuration
 *
 * Multi-stream support: allows multiple simultaneous playback/capture streams
 * Each stream gets its own DMA buffer and DSP stream allocation
 */
#define SST_PCM_MAX_PLAY	4	/* Max simultaneous playback streams */
#define SST_PCM_MAX_REC		2	/* Max simultaneous capture streams */
#define SST_PCM_MAX_STREAMS	(SST_PCM_MAX_PLAY + SST_PCM_MAX_REC)

/*
 * Buffer Configuration
 */
#define SST_PCM_BUFFER_SIZE	(64 * 1024)	/* 64KB buffer */
#define SST_PCM_BLOCK_SIZE	(4 * 1024)	/* 4KB per block */
#define SST_PCM_BLOCK_COUNT	16		/* Number of blocks */

/*
 * Supported Formats
 */
#define SST_PCM_FMTLIST_END	0

/*
 * PCM Channel State
 */
enum sst_pcm_state {
	SST_PCM_STATE_INIT = 0,
	SST_PCM_STATE_PREPARED,
	SST_PCM_STATE_RUNNING,
	SST_PCM_STATE_PAUSED
};

/*
 * PCM Channel Info
 */
struct sst_pcm_channel {
	struct sst_softc	*sc;		/* Parent softc */
	int			dir;		/* PCMDIR_PLAY/PCMDIR_REC */
	int			index;		/* Stream index (0 to MAX-1) */
	bool			allocated;	/* Channel slot allocated */
	enum sst_pcm_state	state;		/* Channel state */

	/* Buffer */
	bus_dma_tag_t		dma_tag;	/* DMA tag */
	bus_dmamap_t		dma_map;	/* DMA map */
	bus_addr_t		dma_addr;	/* Physical address */
	void			*buf;		/* Virtual address */
	size_t			buf_size;	/* Buffer size */

	/* Position tracking */
	uint32_t		blk_size;	/* Block size */
	uint32_t		blk_count;	/* Number of blocks */
	volatile uint32_t	ptr;		/* Current position */

	/* Hardware resources */
	int			ssp_port;	/* SSP port (0 or 1) */
	int			dma_ch;		/* DMA channel (legacy) */
	uint32_t		stream_id;	/* DSP stream ID */
	bool			stream_allocated; /* Stream allocated flag */

	/* DSP position register (from stream alloc response) */
	uint32_t		read_pos_regaddr; /* BAR0 offset for position */

	/* Page table for DSP ring buffer (PFN array) */
	bus_dma_tag_t		pgtbl_tag;	/* DMA tag for page table */
	bus_dmamap_t		pgtbl_map;	/* DMA map for page table */
	uint32_t		*pgtbl_buf;	/* Virtual address */
	bus_addr_t		pgtbl_addr;	/* Physical address */

	/* PCM channel reference for chn_intr */
	struct pcm_channel	*pcm_ch;	/* sound(4) channel */
	struct snd_dbuf		*sndbuf;	/* sound(4) hardware buffer */

	/* Format */
	uint32_t		format;		/* AFMT_* */
	uint32_t		speed;		/* Sample rate */
	uint32_t		channels;	/* Channel count */

	/* Position polling (DMA interrupts don't reach host) */
	struct callout		poll_timer;	/* Polling callout */
	uint32_t		last_pos;	/* Last polled position */
	int			dbg_polls;	/* Debug poll counter */
};

/*
 * PCM Device Context
 *
 * Sound(4) requires that the device passed to pcm_init() has a softc
 * of size PCM_SOFTC_SIZE (struct snddev_info).  Since our ACPI device
 * has sst_softc, we create a child "pcm" device for sound(4) and keep
 * our own state here.
 */
struct sst_pcm {
	struct sst_softc	*sc;		/* Parent softc */
	device_t		pcm_dev;	/* Child "pcm" device */

	/* Multi-stream channels */
	struct sst_pcm_channel	play[SST_PCM_MAX_PLAY];	/* Playback streams */
	struct sst_pcm_channel	rec[SST_PCM_MAX_REC];	/* Capture streams */
	uint32_t		play_count;		/* Active play streams */
	uint32_t		rec_count;		/* Active rec streams */

	/* Stream allocation bitmaps */
	uint32_t		play_alloc;		/* Playback allocation bitmap */
	uint32_t		rec_alloc;		/* Capture allocation bitmap */

	/* Mixer (master) */
	int			vol_left;	/* Left volume (0-100) */
	int			vol_right;	/* Right volume (0-100) */
	int			mute;		/* Mute state */

	/* HPF control */
	uint32_t		hpf_cutoff;	/* HPF cutoff in Hz, 0=bypass */

	/* State */
	bool			registered;	/* PCM device registered */
};

/* Forward declaration */
struct sst_softc;

/*
 * PCM API
 */
int	sst_pcm_init(struct sst_softc *sc);
void	sst_pcm_fini(struct sst_softc *sc);
int	sst_pcm_register(struct sst_softc *sc);
void	sst_pcm_unregister(struct sst_softc *sc);

/* Called from DMA completion */
void	sst_pcm_intr(struct sst_softc *sc, int dir);

#endif /* _SST_PCM_H_ */
