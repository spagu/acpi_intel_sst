/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST SSP (Serial Synchronous Port) / I2S Controller
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_SSP_H_
#define _SST_SSP_H_

#include <sys/types.h>

/*
 * SSP Port Configuration
 */
#define SST_SSP_PORTS		2		/* Number of SSP ports */
#define SST_SSP0_OFFSET		0x00A00000	/* SSP0 base offset */
#define SST_SSP1_OFFSET		0x00A01000	/* SSP1 base offset */
#define SST_SSP_SIZE		0x1000		/* SSP register space */

/*
 * SSP Registers (offset from SSP base)
 */
#define SSP_SSCR0		0x00	/* SSP Control Register 0 */
#define SSP_SSCR1		0x04	/* SSP Control Register 1 */
#define SSP_SSSR		0x08	/* SSP Status Register */
#define SSP_SSITR		0x0C	/* SSP Interrupt Test Register */
#define SSP_SSDR		0x10	/* SSP Data Register */
#define SSP_SSTO		0x28	/* SSP Timeout Register */
#define SSP_SSPSP		0x2C	/* SSP Programmable Serial Protocol */
#define SSP_SSTSA		0x30	/* SSP TX Time Slot Active */
#define SSP_SSRSA		0x34	/* SSP RX Time Slot Active */
#define SSP_SSTSS		0x38	/* SSP TX Time Slot Status */
#define SSP_SSACD		0x3C	/* SSP Audio Clock Divider */
#define SSP_SSCR2		0x40	/* SSP Control Register 2 */
#define SSP_SFIFOTT		0x6C	/* SSP FIFO Trigger Threshold */
#define SSP_SFRM_TStamp		0x70	/* SSP Frame Timestamp */

/*
 * SSCR0 - Control Register 0
 */
#define SSCR0_DSS_MASK		(0x0F << 0)	/* Data Size Select */
#define SSCR0_DSS(x)		(((x) - 1) << 0)
#define SSCR0_FRF_MASK		(0x03 << 4)	/* Frame Format */
#define SSCR0_FRF_MOTOROLA	(0x00 << 4)	/* Motorola SPI */
#define SSCR0_FRF_TI		(0x01 << 4)	/* TI SSP */
#define SSCR0_FRF_NATIONAL	(0x02 << 4)	/* National Microwire */
#define SSCR0_FRF_PSP		(0x03 << 4)	/* Programmable Serial Protocol */
#define SSCR0_ECS		(1 << 6)	/* External Clock Select */
#define SSCR0_SSE		(1 << 7)	/* SSP Enable */
#define SSCR0_SCR_MASK		(0xFFF << 8)	/* Serial Clock Rate */
#define SSCR0_SCR(x)		((x) << 8)
#define SSCR0_EDSS		(1 << 20)	/* Extended Data Size Select */
#define SSCR0_NCS		(1 << 21)	/* Network Clock Select */
#define SSCR0_RIM		(1 << 22)	/* Receive FIFO Overrun Int Mask */
#define SSCR0_TIM		(1 << 23)	/* Transmit FIFO Underrun Int Mask */
#define SSCR0_FRDC_MASK		(0x07 << 24)	/* Frame Rate Divider Control */
#define SSCR0_FRDC(x)		(((x) - 1) << 24)
#define SSCR0_ACS		(1 << 30)	/* Audio Clock Select */
#define SSCR0_MOD		(1 << 31)	/* Mode (0=Normal, 1=Network) */

/*
 * SSCR1 - Control Register 1
 */
#define SSCR1_RIE		(1 << 0)	/* Receive FIFO Interrupt Enable */
#define SSCR1_TIE		(1 << 1)	/* Transmit FIFO Interrupt Enable */
#define SSCR1_LBM		(1 << 2)	/* Loopback Mode */
#define SSCR1_SPO		(1 << 3)	/* Serial Clock Polarity */
#define SSCR1_SPH		(1 << 4)	/* Serial Clock Phase */
#define SSCR1_MWDS		(1 << 5)	/* Microwire TX Data Size */
#define SSCR1_TFT_MASK		(0x0F << 6)	/* TX FIFO Threshold */
#define SSCR1_TFT(x)		((x) << 6)
#define SSCR1_RFT_MASK		(0x0F << 10)	/* RX FIFO Threshold */
#define SSCR1_RFT(x)		((x) << 10)
#define SSCR1_EFWR		(1 << 14)	/* Enable FIFO Write/Read */
#define SSCR1_STRF		(1 << 15)	/* Select FIFO for EFWR */
#define SSCR1_IFS		(1 << 16)	/* Invert Frame Signal */
#define SSCR1_PINTE		(1 << 18)	/* Peripheral Trailing Byte Int */
#define SSCR1_TINTE		(1 << 19)	/* Receiver Timeout Int Enable */
#define SSCR1_RSRE		(1 << 20)	/* RX DMA Service Request Enable */
#define SSCR1_TSRE		(1 << 21)	/* TX DMA Service Request Enable */
#define SSCR1_TRAIL		(1 << 22)	/* Trailing Byte */
#define SSCR1_RWOT		(1 << 23)	/* Receive Without Transmit */
#define SSCR1_SFRMDIR		(1 << 24)	/* SSP Frame Direction */
#define SSCR1_SCLKDIR		(1 << 25)	/* SSP Clock Direction */
#define SSCR1_ECRB		(1 << 26)	/* Enable Clock Request B */
#define SSCR1_ECRA		(1 << 27)	/* Enable Clock Request A */
#define SSCR1_SCFR		(1 << 28)	/* Slave Clock Free Running */
#define SSCR1_EBCEI		(1 << 29)	/* Enable Bit Count Error Int */
#define SSCR1_TTE		(1 << 30)	/* TX Tristate Enable */
#define SSCR1_TTELP		(1 << 31)	/* TX Tristate Enable on LP */

/*
 * SSSR - Status Register
 */
#define SSSR_TNF		(1 << 2)	/* TX FIFO Not Full */
#define SSSR_RNE		(1 << 3)	/* RX FIFO Not Empty */
#define SSSR_BSY		(1 << 4)	/* SSP Busy */
#define SSSR_TFS		(1 << 5)	/* TX FIFO Service Request */
#define SSSR_RFS		(1 << 6)	/* RX FIFO Service Request */
#define SSSR_ROR		(1 << 7)	/* RX FIFO Overrun */
#define SSSR_TFL_MASK		(0x0F << 8)	/* TX FIFO Level */
#define SSSR_RFL_MASK		(0x0F << 12)	/* RX FIFO Level */
#define SSSR_TUR		(1 << 21)	/* TX FIFO Underrun */
#define SSSR_BCE		(1 << 23)	/* Bit Count Error */
#define SSSR_CSS		(1 << 22)	/* Clock Sync Status */
#define SSSR_TINT		(1 << 19)	/* Receiver Timeout Interrupt */
#define SSSR_PINT		(1 << 18)	/* Peripheral Trailing Byte Int */

/*
 * SSPSP - Programmable Serial Protocol
 */
#define SSPSP_SCMODE_MASK	(0x03 << 0)	/* Serial Bit Rate Clock Mode */
#define SSPSP_SCMODE(x)		((x) << 0)
#define SSPSP_SFRMP		(1 << 2)	/* Serial Frame Polarity */
#define SSPSP_ETDS		(1 << 3)	/* End of Transfer Data State */
#define SSPSP_STRTDLY_MASK	(0x07 << 4)	/* Start Delay */
#define SSPSP_STRTDLY(x)	((x) << 4)
#define SSPSP_DMYSTRT_MASK	(0x03 << 7)	/* Dummy Start */
#define SSPSP_DMYSTRT(x)	((x) << 7)
#define SSPSP_SFRMDLY_MASK	(0x7F << 9)	/* Serial Frame Delay */
#define SSPSP_SFRMDLY(x)	((x) << 9)
#define SSPSP_SFRMWDTH_MASK	(0x3F << 16)	/* Serial Frame Width */
#define SSPSP_SFRMWDTH(x)	((x) << 16)
#define SSPSP_DMYSTOP_MASK	(0x07 << 23)	/* Dummy Stop */
#define SSPSP_DMYSTOP(x)	((x) << 23)
#define SSPSP_FSRT		(1 << 25)	/* Frame Sync Relative Timing */
#define SSPSP_EDMYSTRT_MASK	(0x07 << 26)	/* Extended Dummy Start */
#define SSPSP_EDMYSTRT(x)	((x) << 26)
#define SSPSP_EDMYSTOP_MASK	(0x07 << 29)	/* Extended Dummy Stop */
#define SSPSP_EDMYSTOP(x)	((x) << 29)

/*
 * SSACD - Audio Clock Divider
 */
#define SSACD_ACDS_MASK		(0x07 << 0)	/* Audio Clock Divider Select */
#define SSACD_ACDS(x)		((x) << 0)
#define SSACD_SCDB		(1 << 3)	/* SSP Clock Divider Bypass */
#define SSACD_ACPS_MASK		(0x07 << 4)	/* Audio Clock PLL Select */
#define SSACD_ACPS(x)		((x) << 4)
#define SSACD_SCDX8		(1 << 7)	/* SSP Clock Divider x8 */

/*
 * SFIFOTT - FIFO Trigger Threshold
 */
#define SFIFOTT_TX_MASK		(0xFFFF << 0)
#define SFIFOTT_TX(x)		((x) << 0)
#define SFIFOTT_RX_MASK		(0xFFFF << 16)
#define SFIFOTT_RX(x)		((x) << 16)

/*
 * Audio Formats
 */
#define SST_FMT_I2S		0	/* I2S format */
#define SST_FMT_LEFT_J		1	/* Left Justified */
#define SST_FMT_RIGHT_J		2	/* Right Justified */
#define SST_FMT_DSP_A		3	/* DSP Mode A */
#define SST_FMT_DSP_B		4	/* DSP Mode B */

/*
 * Clock Sources
 */
#define SST_CLK_AUDIO		0	/* Audio PLL */
#define SST_CLK_MCLK		1	/* MCLK input */
#define SST_CLK_XTAL		2	/* Crystal */

/*
 * Sample Rates
 */
#define SST_RATE_8000		8000
#define SST_RATE_16000		16000
#define SST_RATE_22050		22050
#define SST_RATE_32000		32000
#define SST_RATE_44100		44100
#define SST_RATE_48000		48000
#define SST_RATE_88200		88200
#define SST_RATE_96000		96000
#define SST_RATE_176400		176400
#define SST_RATE_192000		192000

/*
 * SSP Port State
 */
enum sst_ssp_state {
	SST_SSP_STATE_IDLE = 0,
	SST_SSP_STATE_CONFIGURED,
	SST_SSP_STATE_RUNNING,
	SST_SSP_STATE_PAUSED
};

/*
 * SSP Configuration
 */
struct sst_ssp_config {
	uint32_t	sample_rate;	/* Sample rate in Hz */
	uint32_t	sample_bits;	/* Bits per sample (16, 24, 32) */
	uint32_t	channels;	/* Number of channels (1, 2) */
	uint32_t	format;		/* Audio format (I2S, etc) */
	uint32_t	mclk_rate;	/* MCLK frequency */
	bool		master;		/* true = master, false = slave */
};

/*
 * SSP Port Context
 */
struct sst_ssp_port {
	uint32_t		id;		/* Port ID (0 or 1) */
	bus_addr_t		base;		/* Base address offset */
	enum sst_ssp_state	state;		/* Port state */
	struct sst_ssp_config	config;		/* Current configuration */

	/* Statistics */
	uint32_t		tx_count;
	uint32_t		rx_count;
	uint32_t		errors;
};

/*
 * SSP Controller Context
 */
struct sst_ssp {
	struct sst_ssp_port	port[SST_SSP_PORTS];
	bool			initialized;
};

/* Forward declaration */
struct sst_softc;

/*
 * SSP API
 */
int	sst_ssp_init(struct sst_softc *sc);
void	sst_ssp_fini(struct sst_softc *sc);

/* Port control */
int	sst_ssp_configure(struct sst_softc *sc, int port,
			  struct sst_ssp_config *config);
int	sst_ssp_start(struct sst_softc *sc, int port);
int	sst_ssp_stop(struct sst_softc *sc, int port);
int	sst_ssp_pause(struct sst_softc *sc, int port);
int	sst_ssp_resume(struct sst_softc *sc, int port);

/* Data transfer (for non-DMA mode) */
int	sst_ssp_write(struct sst_softc *sc, int port,
		      const void *data, size_t len);
int	sst_ssp_read(struct sst_softc *sc, int port,
		     void *data, size_t len);

/* Status */
bool	sst_ssp_is_running(struct sst_softc *sc, int port);
void	sst_ssp_dump_regs(struct sst_softc *sc, int port);

#endif /* _SST_SSP_H_ */
