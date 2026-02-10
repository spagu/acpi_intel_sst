/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST IPC (Inter-Processor Communication) Protocol
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_IPC_H_
#define _SST_IPC_H_

#include <sys/types.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>

/*
 * IPC Message Types (Global)
 * Based on Linux catpt driver messages.h
 */
#define SST_IPC_GLBL_REPLY		0x00
#define SST_IPC_GLBL_GET_FW_VERSION	0x01
#define SST_IPC_GLBL_ALLOCATE_STREAM	0x02
#define SST_IPC_GLBL_FREE_STREAM	0x03
#define SST_IPC_GLBL_STREAM_MESSAGE	0x04
#define SST_IPC_GLBL_REQUEST_DUMP	0x05
#define SST_IPC_GLBL_SET_DEVICE_FORMATS	0x06
#define SST_IPC_GLBL_SET_DX		0x07
#define SST_IPC_GLBL_ENTER_DX_STATE	0x08
#define SST_IPC_GLBL_GET_MIXER_PARAMS	0x09
#define SST_IPC_GLBL_SET_MIXER_PARAMS	0x0A
#define SST_IPC_GLBL_NOTIFICATION	0x0F

/*
 * Stream Message Types
 */
#define SST_IPC_STR_PAUSE		0x00
#define SST_IPC_STR_RESUME		0x01
#define SST_IPC_STR_GET_PARAMS		0x02
#define SST_IPC_STR_SET_PARAMS		0x03
#define SST_IPC_STR_GET_POSITION	0x04
#define SST_IPC_STR_RESET		0x05
#define SST_IPC_STR_MUTE		0x06
#define SST_IPC_STR_UNMUTE		0x07

/*
 * Notification Types (from DSP)
 */
#define SST_NOTIFY_POSITION_CHANGED	0x00
#define SST_NOTIFY_GLITCH		0x01
#define SST_NOTIFY_UNDERRUN		0x02
#define SST_NOTIFY_OVERRUN		0x03

/*
 * Stream Types
 */
#define SST_STREAM_TYPE_RENDER		0x00	/* Playback */
#define SST_STREAM_TYPE_CAPTURE		0x01	/* Recording */
#define SST_STREAM_TYPE_SYSTEM		0x02	/* System sounds */
#define SST_STREAM_TYPE_LOOPBACK	0x03	/* Loopback */

/*
 * Audio Format IDs (for DSP)
 */
#define SST_FMT_PCM			0x00
#define SST_FMT_MP3			0x01
#define SST_FMT_AAC			0x02
#define SST_FMT_WMA			0x03

/*
 * IPC Header Format
 *
 * |31    |30    |29:24 |23:20 |19:16 |15:8  |7:0   |
 * |BUSY  |DONE  |TYPE  |RSVD  |MSG   |SIZE  |RSVD  |
 */
#define SST_IPC_TYPE_SHIFT	24
#define SST_IPC_TYPE_MASK	(0x3F << SST_IPC_TYPE_SHIFT)
#define SST_IPC_MSG_SHIFT	16
#define SST_IPC_MSG_MASK	(0xF << SST_IPC_MSG_SHIFT)
#define SST_IPC_SIZE_SHIFT	8
#define SST_IPC_SIZE_MASK	(0xFF << SST_IPC_SIZE_SHIFT)

#define SST_IPC_HEADER(type, msg, size) \
	(((type) << SST_IPC_TYPE_SHIFT) | \
	 ((msg) << SST_IPC_MSG_SHIFT) | \
	 ((size) << SST_IPC_SIZE_SHIFT))

/*
 * IPC Reply Status
 */
#define SST_IPC_REPLY_SUCCESS		0x00
#define SST_IPC_REPLY_ERROR		0x01
#define SST_IPC_REPLY_BUSY		0x02
#define SST_IPC_REPLY_PENDING		0x03

/*
 * Firmware Version Structure
 */
struct sst_fw_version {
	uint8_t		build;		/* Build number */
	uint8_t		minor;		/* Minor version */
	uint8_t		major;		/* Major version */
	uint8_t		type;		/* Build type */
} __packed;

struct sst_fw_version_reply {
	struct sst_fw_version	version;
	uint32_t		reserved[3];
} __packed;

/*
 * Audio Format Structure
 * Used for stream allocation
 */
struct sst_audio_format {
	uint32_t	sample_rate;	/* Sample rate in Hz */
	uint32_t	bit_depth;	/* Bits per sample (16/24/32) */
	uint32_t	channels;	/* Number of channels */
	uint32_t	channel_map;	/* Channel layout bitmask */
	uint32_t	interleaving;	/* Interleaved/non-interleaved */
	uint32_t	format_id;	/* SST_FMT_* */
	uint32_t	reserved[2];
} __packed;

/*
 * Stream Allocation Request
 */
struct sst_alloc_stream_req {
	uint32_t		stream_type;	/* SST_STREAM_TYPE_* */
	uint32_t		path_id;	/* Audio path ID (SSP port) */
	struct sst_audio_format	format;		/* Audio format */
	uint32_t		ring_buf_addr;	/* DMA buffer physical address */
	uint32_t		ring_buf_size;	/* DMA buffer size */
	uint32_t		period_count;	/* Number of periods */
	uint32_t		reserved[4];
} __packed;

/*
 * Stream Allocation Response
 */
struct sst_alloc_stream_rsp {
	uint32_t	stream_id;	/* Assigned stream ID */
	uint32_t	status;		/* Status code */
	uint32_t	reserved[6];
} __packed;

/*
 * Stream Parameters (for SET_PARAMS/GET_PARAMS)
 */
struct sst_stream_params {
	uint32_t	stream_id;	/* Stream ID */
	uint32_t	volume_left;	/* Left channel volume (0-100) */
	uint32_t	volume_right;	/* Right channel volume (0-100) */
	uint32_t	mute;		/* Mute flag */
	uint32_t	reserved[4];
} __packed;

/*
 * Stream Position (for GET_POSITION)
 */
struct sst_stream_position {
	uint32_t	stream_id;	/* Stream ID */
	uint32_t	position;	/* Current position in bytes */
	uint32_t	timestamp_low;	/* Timestamp (low 32 bits) */
	uint32_t	timestamp_high;	/* Timestamp (high 32 bits) */
} __packed;

/*
 * Mixer Parameters (for SET_MIXER_PARAMS/GET_MIXER_PARAMS)
 */
struct sst_mixer_params {
	uint32_t	output_id;	/* Output path ID */
	uint32_t	volume;		/* Master volume (0-100) */
	uint32_t	mute;		/* Master mute */
	uint32_t	reserved[5];
} __packed;

/*
 * Device Power State (for SET_DX)
 */
struct sst_dx_state {
	uint32_t	state;		/* D0, D3 */
	uint32_t	reserved[3];
} __packed;

#define SST_DX_STATE_D0		0x00
#define SST_DX_STATE_D0I3	0x03
#define SST_DX_STATE_D3		0x04

/*
 * IPC State
 */
enum sst_ipc_state {
	SST_IPC_STATE_IDLE = 0,
	SST_IPC_STATE_PENDING,
	SST_IPC_STATE_DONE,
	SST_IPC_STATE_ERROR
};

/*
 * IPC Message Context
 */
struct sst_ipc_msg {
	uint32_t	header;		/* IPC header */
	void		*data;		/* Message data */
	size_t		size;		/* Data size */
	uint32_t	reply;		/* Reply header */
	int		status;		/* Status code */
};

/*
 * IPC Mailbox
 */
#define SST_MBOX_SIZE_IN	0x400	/* Host -> DSP: 1KB */
#define SST_MBOX_SIZE_OUT	0x400	/* DSP -> Host: 1KB */

/*
 * IPC Context
 */
struct sst_ipc {
	struct mtx		lock;		/* IPC lock */
	struct cv		wait_cv;	/* Wait condition */
	enum sst_ipc_state	state;		/* Current state */
	bool			ready;		/* DSP ready flag */

	/* Current message */
	struct sst_ipc_msg	msg;

	/* Mailbox addresses */
	bus_addr_t		mbox_in;	/* Host -> DSP */
	bus_addr_t		mbox_out;	/* DSP -> Host */

	/* Statistics */
	uint32_t		tx_count;
	uint32_t		rx_count;
	uint32_t		error_count;
};

/* Forward declaration */
struct sst_softc;

/*
 * IPC API
 */
int	sst_ipc_init(struct sst_softc *sc);
void	sst_ipc_fini(struct sst_softc *sc);

/* Message operations */
int	sst_ipc_send(struct sst_softc *sc, uint32_t header,
		     void *data, size_t size);
int	sst_ipc_recv(struct sst_softc *sc, uint32_t *header,
		     void *data, size_t *size);

/* DSP ready check */
int	sst_ipc_wait_ready(struct sst_softc *sc, int timeout_ms);

/* Interrupt handler (called from main ISR) */
void	sst_ipc_intr(struct sst_softc *sc);

/* Utility commands */
int	sst_ipc_get_fw_version(struct sst_softc *sc,
			       struct sst_fw_version *version);

/* Stream management */
int	sst_ipc_alloc_stream(struct sst_softc *sc,
			     struct sst_alloc_stream_req *req,
			     uint32_t *stream_id);
int	sst_ipc_free_stream(struct sst_softc *sc, uint32_t stream_id);

/* Stream control */
int	sst_ipc_stream_pause(struct sst_softc *sc, uint32_t stream_id);
int	sst_ipc_stream_resume(struct sst_softc *sc, uint32_t stream_id);
int	sst_ipc_stream_reset(struct sst_softc *sc, uint32_t stream_id);
int	sst_ipc_stream_set_params(struct sst_softc *sc,
				  struct sst_stream_params *params);
int	sst_ipc_stream_get_position(struct sst_softc *sc, uint32_t stream_id,
				    struct sst_stream_position *pos);

/* Mixer control */
int	sst_ipc_set_mixer(struct sst_softc *sc, struct sst_mixer_params *params);
int	sst_ipc_get_mixer(struct sst_softc *sc, struct sst_mixer_params *params);

/* Power management */
int	sst_ipc_set_dx(struct sst_softc *sc, uint32_t state);

#endif /* _SST_IPC_H_ */
