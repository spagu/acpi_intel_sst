/*-
 * SPDX-License-Identifier: BSD-2-Clause
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

/*
 * PCM Channel Configuration
 */
#define SST_PCM_CHANNELS	2	/* Playback + Capture */
#define SST_PCM_PLAY		0	/* Playback channel index */
#define SST_PCM_REC		1	/* Capture channel index */

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
	int			dir;		/* PCMDIR_PLAY/PCMDIR_REC */
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
	int			dma_ch;		/* DMA channel */

	/* Format */
	uint32_t		format;		/* AFMT_* */
	uint32_t		speed;		/* Sample rate */
	uint32_t		channels;	/* Channel count */
};

/*
 * PCM Device Context
 */
struct sst_pcm {
	device_t		dev;		/* PCM device */
	struct sst_softc	*sc;		/* Parent softc */

	/* Channels */
	struct sst_pcm_channel	play;		/* Playback */
	struct sst_pcm_channel	rec;		/* Capture */

	/* Mixer */
	int			vol_left;	/* Left volume (0-100) */
	int			vol_right;	/* Right volume (0-100) */
	int			mute;		/* Mute state */

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
