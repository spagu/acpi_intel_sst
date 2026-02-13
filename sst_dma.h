/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST DMA Controller
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_DMA_H_
#define _SST_DMA_H_

#include <sys/types.h>

/*
 * DMA Controller Configuration
 * Note: SST_DMA_OFFSET and SST_DMA_SIZE are defined in sst_regs.h
 */
#define SST_DMA_CHANNELS	8		/* Number of DMA channels */

/*
 * DMA Channel Registers (offset from channel base)
 */
#define DMA_SAR			0x00	/* Source Address Register */
#define DMA_DAR			0x08	/* Destination Address Register */
#define DMA_LLP			0x10	/* Linked List Pointer */
#define DMA_CTL_LO		0x18	/* Control Register Low */
#define DMA_CTL_HI		0x1C	/* Control Register High */
#define DMA_CFG_LO		0x40	/* Configuration Register Low */
#define DMA_CFG_HI		0x44	/* Configuration Register High */

/*
 * DMA Global Registers
 */
#define DMA_CFG			0x398	/* DMA Configuration */
#define DMA_CHEN		0x3A0	/* Channel Enable */
#define DMA_MASK_TFR		0x310	/* Mask Transfer Interrupt */
#define DMA_MASK_BLOCK		0x318	/* Mask Block Interrupt */
#define DMA_MASK_ERR		0x330	/* Mask Error Interrupt */
#define DMA_CLEAR_TFR		0x338	/* Clear Transfer Interrupt */
#define DMA_CLEAR_BLOCK		0x340	/* Clear Block Interrupt */
#define DMA_CLEAR_ERR		0x358	/* Clear Error Interrupt */
#define DMA_STATUS_TFR		0x2E8	/* Status Transfer */
#define DMA_STATUS_BLOCK	0x2F0	/* Status Block */
#define DMA_STATUS_ERR		0x308	/* Status Error */

/*
 * DMA_CTL_LO bits
 */
#define DMA_CTL_INT_EN		(1 << 0)	/* Interrupt Enable */
#define DMA_CTL_DST_WIDTH_MASK	(0x07 << 1)	/* Dest Transfer Width */
#define DMA_CTL_DST_WIDTH(x)	((x) << 1)
#define DMA_CTL_SRC_WIDTH_MASK	(0x07 << 4)	/* Src Transfer Width */
#define DMA_CTL_SRC_WIDTH(x)	((x) << 4)
#define DMA_CTL_DINC_MASK	(0x03 << 7)	/* Dest Address Inc */
#define DMA_CTL_DINC(x)		((x) << 7)
#define DMA_CTL_SINC_MASK	(0x03 << 9)	/* Src Address Inc */
#define DMA_CTL_SINC(x)		((x) << 9)
#define DMA_CTL_DST_MSIZE_MASK	(0x07 << 11)	/* Dest Burst Size */
#define DMA_CTL_DST_MSIZE(x)	((x) << 11)
#define DMA_CTL_SRC_MSIZE_MASK	(0x07 << 14)	/* Src Burst Size */
#define DMA_CTL_SRC_MSIZE(x)	((x) << 14)
#define DMA_CTL_TT_FC_MASK	(0x07 << 20)	/* Transfer Type & Flow Ctrl */
#define DMA_CTL_TT_FC(x)	((x) << 20)
#define DMA_CTL_LLP_DST_EN	(1 << 27)	/* LLP Dest Enable */
#define DMA_CTL_LLP_SRC_EN	(1 << 28)	/* LLP Src Enable */

/*
 * DMA_CTL_HI bits
 */
#define DMA_CTL_BLOCK_TS_MASK	0xFFF		/* Block Transfer Size */
#define DMA_CTL_DONE		(1 << 12)	/* Done bit */

/*
 * DMA_CFG_LO bits
 */
#define DMA_CFG_CH_PRIOR_MASK	(0x07 << 5)	/* Channel Priority */
#define DMA_CFG_CH_PRIOR(x)	((x) << 5)
#define DMA_CFG_CH_SUSP		(1 << 8)	/* Channel Suspend */
#define DMA_CFG_FIFO_EMPTY	(1 << 9)	/* FIFO Empty */
#define DMA_CFG_HS_SEL_DST	(1 << 10)	/* Dest HW/SW Handshaking */
#define DMA_CFG_HS_SEL_SRC	(1 << 11)	/* Src HW/SW Handshaking */
#define DMA_CFG_DST_HS_POL	(1 << 18)	/* Dest HS Polarity */
#define DMA_CFG_SRC_HS_POL	(1 << 19)	/* Src HS Polarity */
#define DMA_CFG_RELOAD_DST	(1 << 30)	/* Auto Reload Dest */
#define DMA_CFG_RELOAD_SRC	(1 << 31)	/* Auto Reload Src */

/*
 * DMA_CFG_HI bits
 */
#define DMA_CFG_FCMODE		(1 << 0)	/* Flow Control Mode */
#define DMA_CFG_FIFO_MODE	(1 << 1)	/* FIFO Mode */
#define DMA_CFG_PROTCTL_MASK	(0x07 << 2)	/* Protection Control */
#define DMA_CFG_PROTCTL(x)	((x) << 2)
#define DMA_CFG_DST_PER_MASK	(0x0F << 11)	/* Dest HW HS Interface */
#define DMA_CFG_DST_PER(x)	((x) << 11)
#define DMA_CFG_SRC_PER_MASK	(0x0F << 7)	/* Src HW HS Interface */
#define DMA_CFG_SRC_PER(x)	((x) << 7)

/*
 * Transfer Width
 */
#define DMA_WIDTH_8		0
#define DMA_WIDTH_16		1
#define DMA_WIDTH_32		2
#define DMA_WIDTH_64		3

/*
 * Address Increment
 */
#define DMA_INC_INCREMENT	0
#define DMA_INC_DECREMENT	1
#define DMA_INC_NO_CHANGE	2

/*
 * Burst Size (msize)
 */
#define DMA_MSIZE_1		0
#define DMA_MSIZE_4		1
#define DMA_MSIZE_8		2
#define DMA_MSIZE_16		3
#define DMA_MSIZE_32		4
#define DMA_MSIZE_64		5

/*
 * Transfer Type / Flow Control
 */
#define DMA_TT_M2M		0	/* Memory to Memory */
#define DMA_TT_M2P		1	/* Memory to Peripheral */
#define DMA_TT_P2M		2	/* Peripheral to Memory */
#define DMA_TT_P2P		3	/* Peripheral to Peripheral */

/*
 * DMA Buffer Descriptor (for linked list)
 */
struct sst_dma_desc {
	uint32_t	sar;		/* Source address */
	uint32_t	sar_hi;
	uint32_t	dar;		/* Destination address */
	uint32_t	dar_hi;
	uint32_t	llp;		/* Next descriptor */
	uint32_t	llp_hi;
	uint32_t	ctl_lo;		/* Control low */
	uint32_t	ctl_hi;		/* Control high */
} __packed;

/*
 * DMA Channel State
 */
enum sst_dma_state {
	SST_DMA_STATE_IDLE = 0,
	SST_DMA_STATE_CONFIGURED,
	SST_DMA_STATE_RUNNING,
	SST_DMA_STATE_PAUSED
};

/*
 * Max linked-list descriptors per channel (for circular DMA)
 */
#define SST_DMA_MAX_LLI		256

/*
 * DMA Channel Configuration
 */
struct sst_dma_config {
	bus_addr_t	src;		/* Source address */
	bus_addr_t	dst;		/* Destination address */
	size_t		size;		/* Total transfer size (full buffer) */
	uint32_t	direction;	/* Transfer direction */
	uint32_t	src_width;	/* Source width */
	uint32_t	dst_width;	/* Destination width */
	uint32_t	src_burst;	/* Source burst size */
	uint32_t	dst_burst;	/* Destination burst size */
	bool		circular;	/* Circular buffer mode */
	size_t		blk_size;	/* Block size (for circular mode) */
	uint32_t	blk_count;	/* Block count (for circular mode) */
};

/*
 * DMA Channel Context
 */
struct sst_dma_channel {
	uint32_t		id;		/* Channel ID */
	enum sst_dma_state	state;		/* Channel state */
	struct sst_dma_config	config;		/* Configuration */
	void			(*callback)(void *);	/* Completion callback */
	void			*callback_arg;

	/* Linked-list descriptors for circular DMA */
	struct sst_dma_desc	*lli;		/* LLI array (virtual) */
	bus_addr_t		lli_addr;	/* LLI array (physical) */
	bus_dma_tag_t		lli_tag;	/* DMA tag for LLI */
	bus_dmamap_t		lli_map;	/* DMA map for LLI */
	uint32_t		lli_count;	/* Number of LLI entries */
};

/*
 * DMA Controller Context
 */
struct sst_dma {
	struct sst_dma_channel	ch[SST_DMA_CHANNELS];
	bool			initialized;
	uint32_t		active_mask;	/* Active channels bitmask */
};

/* Forward declaration */
struct sst_softc;

/*
 * DMA API
 */
int	sst_dma_init(struct sst_softc *sc);
void	sst_dma_fini(struct sst_softc *sc);

/* Channel control */
int	sst_dma_alloc(struct sst_softc *sc);
void	sst_dma_free(struct sst_softc *sc, int ch);
int	sst_dma_configure(struct sst_softc *sc, int ch,
			  struct sst_dma_config *config);
int	sst_dma_start(struct sst_softc *sc, int ch);
int	sst_dma_stop(struct sst_softc *sc, int ch);
int	sst_dma_pause(struct sst_softc *sc, int ch);
int	sst_dma_resume(struct sst_softc *sc, int ch);

/* Callback */
void	sst_dma_set_callback(struct sst_softc *sc, int ch,
			     void (*callback)(void *), void *arg);

/* Interrupt handler */
void	sst_dma_intr(struct sst_softc *sc);

/* Status */
bool	sst_dma_is_running(struct sst_softc *sc, int ch);
size_t	sst_dma_get_position(struct sst_softc *sc, int ch);

#endif /* _SST_DMA_H_ */
