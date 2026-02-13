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
 * From Linux catpt driver: enum catpt_global_msg_type (messages.h)
 * These go into bits[28:24] of IPCX header.
 */
#define SST_IPC_GLBL_GET_FW_VERSION	0	/* CATPT_GLB_GET_FW_VERSION */
#define SST_IPC_GLBL_ALLOCATE_STREAM	3	/* CATPT_GLB_ALLOCATE_STREAM */
#define SST_IPC_GLBL_FREE_STREAM	4	/* CATPT_GLB_FREE_STREAM */
#define SST_IPC_GLBL_STREAM_MESSAGE	6	/* CATPT_GLB_STREAM_MESSAGE */
#define SST_IPC_GLBL_REQUEST_DUMP	7	/* CATPT_GLB_REQUEST_CORE_DUMP */
#define SST_IPC_GLBL_SET_DEVICE_FORMATS	10	/* CATPT_GLB_SET_DEVICE_FORMATS */
#define SST_IPC_GLBL_ENTER_DX_STATE	12	/* CATPT_GLB_ENTER_DX_STATE */
#define SST_IPC_GLBL_GET_MIXER_PARAMS	13	/* CATPT_GLB_GET_MIXER_STREAM_INFO */
#define SST_IPC_GLBL_SET_MIXER_PARAMS	13	/* Uses same type as GET (data in mailbox) */
#define SST_IPC_GLBL_NOTIFICATION	15	/* CATPT_GLB_NOTIFICATION */

/*
 * Stream Message Types (catpt_stream_msg_type)
 */
#define SST_IPC_STR_RESET		0	/* CATPT_STRM_RESET_STREAM */
#define SST_IPC_STR_PAUSE		1	/* CATPT_STRM_PAUSE_STREAM */
#define SST_IPC_STR_RESUME		2	/* CATPT_STRM_RESUME_STREAM */
#define SST_IPC_STR_STAGE_MESSAGE	3	/* CATPT_STRM_STAGE_MESSAGE */
#define SST_IPC_STR_NOTIFICATION	4	/* CATPT_STRM_NOTIFICATION */

/* Stage actions (sub-type of STAGE_MESSAGE) */
#define SST_IPC_STG_SET_VOLUME		1
#define SST_IPC_STG_SET_WRITE_POS	2
#define SST_IPC_STG_MUTE_LOOPBACK	3

/*
 * Notification Types (from DSP)
 */
#define SST_NOTIFY_POSITION_CHANGED	0
#define SST_NOTIFY_GLITCH		1

/*
 * Stream Types (catpt_stream_type)
 */
#define SST_STREAM_TYPE_RENDER		0	/* CATPT_STRM_TYPE_RENDER */
#define SST_STREAM_TYPE_SYSTEM		1	/* CATPT_STRM_TYPE_SYSTEM */
#define SST_STREAM_TYPE_CAPTURE		2	/* CATPT_STRM_TYPE_CAPTURE */
#define SST_STREAM_TYPE_LOOPBACK	3	/* CATPT_STRM_TYPE_LOOPBACK */

/* Path IDs (catpt_path_id) */
#define SST_PATH_SSP0_OUT		0
#define SST_PATH_SSP0_IN		1
#define SST_PATH_SSP1_OUT		2
#define SST_PATH_SSP1_IN		3

/* Channel configuration (catpt_channel_config) */
#define SST_CHAN_CONFIG_MONO		0
#define SST_CHAN_CONFIG_STEREO		1

/* Interleaving style */
#define SST_INTERLEAVING_PER_CHANNEL	0
#define SST_INTERLEAVING_PER_SAMPLE	1

/*
 * Audio Format IDs (catpt_format_id)
 */
#define SST_FMT_PCM			0x00
#define SST_FMT_MP3			0x01
#define SST_FMT_AAC			0x02
#define SST_FMT_WMA			0x03

/* Module IDs (catpt_module_id) */
#define SST_MODID_BASE_FW		0x0
#define SST_MODID_PCM_CAPTURE		0xA
#define SST_MODID_PCM_SYSTEM		0xB
#define SST_MODID_PCM_REFERENCE		0xC
#define SST_MODID_PCM			0xD

/*
 * IPC Header Format (catpt)
 *
 * From Linux catpt union catpt_global_msg (messages.h):
 * |31   |30   |29      |28:24           |23:5    |4:0    |
 * |BUSY |DONE |FW_READY|GLOBAL_MSG_TYPE |CONTEXT |STATUS |
 *
 * From Linux catpt union catpt_notify_msg (FW_READY):
 * |31   |30   |29      |28:0                            |
 * |BUSY |DONE |FW_READY|MAILBOX_ADDRESS (>> 3)          |
 *
 * The firmware stores (actual_address >> 3) in bits[28:0].
 * Driver recovers actual offset by shifting left by 3.
 */
#define SST_IPC_FW_READY	(1U << 29)	/* FW Ready bit in IPCD */
#define SST_IPC_MSG_TYPE_SHIFT	24
#define SST_IPC_MSG_TYPE_MASK	(0x1F << SST_IPC_MSG_TYPE_SHIFT)
#define SST_IPC_STATUS_MASK	0x1F		/* bits 0-4 */
#define SST_IPC_CONTEXT_SHIFT	5
#define SST_IPC_CONTEXT_MASK	(0x7FFFF << SST_IPC_CONTEXT_SHIFT)

/* FW_READY mailbox address extraction from IPCD */
#define SST_IPC_MBOX_ADDR_MASK	0x1FFFFFFFU	/* bits [28:0] */
#define SST_IPC_MBOX_ADDR_SHIFT	3		/* left-shift to get byte offset */
#define SST_IPC_MBOX_OFFSET(ipcd) \
	(((ipcd) & SST_IPC_MBOX_ADDR_MASK) << SST_IPC_MBOX_ADDR_SHIFT)

/*
 * catpt IPC header format (global messages)
 *
 * For Host->DSP (IPCX): set global_msg_type in bits[28:24], BUSY in bit 31.
 * Context/data goes in bits[23:5] if needed.
 */
/*
 * catpt stream message header format (bits differ from global):
 * |31:29 |28:24           |23:20           |19:16        |15:12       |4:0   |
 * |flags |GLOBAL_MSG_TYPE |STREAM_MSG_TYPE |STREAM_HW_ID |STAGE_ACTION|STATUS|
 */
#define SST_IPC_STR_MSG_TYPE_SHIFT	20
#define SST_IPC_STR_HW_ID_SHIFT	16
#define SST_IPC_STR_STAGE_SHIFT		12

#define SST_IPC_HEADER(type, ctx, ...) \
	(((uint32_t)(type) << SST_IPC_MSG_TYPE_SHIFT) | \
	 ((uint32_t)(ctx) << SST_IPC_CONTEXT_SHIFT))

/* Stream message header: type=STREAM_MESSAGE, sub=stream_msg_type */
#define SST_IPC_STREAM_HEADER(str_type, stream_id) \
	(((uint32_t)SST_IPC_GLBL_STREAM_MESSAGE << SST_IPC_MSG_TYPE_SHIFT) | \
	 ((uint32_t)(str_type) << SST_IPC_STR_MSG_TYPE_SHIFT) | \
	 ((uint32_t)(stream_id) << SST_IPC_STR_HW_ID_SHIFT))

/*
 * IPC Reply Status (catpt_reply_status)
 */
#define SST_IPC_REPLY_SUCCESS		0
#define SST_IPC_REPLY_ERROR_INVALID	1	/* Invalid parameter */
#define SST_IPC_REPLY_UNKNOWN_MSG	2
#define SST_IPC_REPLY_OUT_OF_RES	3
#define SST_IPC_REPLY_BUSY		4
#define SST_IPC_REPLY_PENDING		5
#define SST_IPC_REPLY_FAILURE		6
#define SST_IPC_REPLY_INVALID_REQ	7

/*
 * FW_READY Mailbox (from Linux catpt messages.h: struct catpt_fw_ready)
 * DSP writes this to the outbox (DRAM) on boot completion.
 * Contains inbox/outbox configuration for subsequent IPC.
 */
#define SST_FW_INFO_SIZE_MAX	100

struct sst_fw_ready {
	uint32_t	inbox_offset;	/* Host -> DSP mailbox offset (in DRAM) */
	uint32_t	outbox_offset;	/* DSP -> Host mailbox offset (in DRAM) */
	uint32_t	inbox_size;	/* Inbox size */
	uint32_t	outbox_size;	/* Outbox size */
	uint32_t	fw_info_size;	/* FW info string size */
	char		fw_info[SST_FW_INFO_SIZE_MAX]; /* FW info string */
} __packed;

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
 * Audio Format (catpt_audio_format) - 24 bytes
 */
struct sst_audio_format {
	uint32_t	sample_rate;	/* Sample rate in Hz */
	uint32_t	bit_depth;	/* Bits per sample (16/24/32) */
	uint32_t	channel_map;	/* Channel layout bitmask */
	uint32_t	channel_config;	/* SST_CHAN_CONFIG_* */
	uint32_t	interleaving;	/* SST_INTERLEAVING_* */
	uint8_t		num_channels;	/* Number of channels */
	uint8_t		valid_bit_depth;/* Valid bits in container */
	uint8_t		reserved[2];
} __packed;

/*
 * Ring buffer info (catpt_ring_info) - 20 bytes
 */
struct sst_ring_info {
	uint32_t	page_table_addr;/* Physical addr of PFN array */
	uint32_t	num_pages;	/* Number of pages */
	uint32_t	size;		/* Ring buffer size in bytes */
	uint32_t	offset;		/* Start offset */
	uint32_t	first_pfn;	/* First page frame number */
} __packed;

/*
 * Memory info (catpt_memory_info) - 8 bytes
 */
struct sst_mem_info {
	uint32_t	offset;		/* DRAM offset */
	uint32_t	size;		/* Size in bytes */
} __packed;

/*
 * Module entry for stream allocation - 8 bytes
 */
struct sst_module_entry {
	uint32_t	module_id;	/* SST_MODID_* */
	uint32_t	entry_point;	/* Module entry point */
} __packed;

/*
 * Stream Allocation Request (catpt_alloc_stream_input)
 * Note: module entries are spliced between num_entries and persistent_mem
 * when building the actual mailbox payload.
 *
 * Linux catpt uses 8-bit bitfields (enum:8) for path_id, stream_type,
 * format_id packed into a single 32-bit word.  Using uint32_t per field
 * shifts all subsequent fields by 8 bytes, causing INVALID_PARAM.
 */
struct sst_alloc_stream_req {
	uint8_t			path_id;	/* SST_PATH_* (1 byte) */
	uint8_t			stream_type;	/* SST_STREAM_TYPE_* (1 byte) */
	uint8_t			format_id;	/* SST_FMT_* (1 byte) */
	uint8_t			reserved;	/* padding (1 byte) */
	struct sst_audio_format	format;		/* Audio format (24 bytes) */
	struct sst_ring_info	ring;		/* Ring buffer info (20 bytes) */
	uint8_t			num_entries;	/* Number of module entries */
	struct sst_mem_info	persistent_mem;	/* Module persistent memory */
	struct sst_mem_info	scratch_mem;	/* Module scratch memory */
	uint32_t		num_notifications;
} __packed;

/* Max DMA pages for ring buffer page table */
#define SST_MAX_RING_PAGES	16

/*
 * Stream Allocation Response (catpt_stream_info) - 40 bytes
 */
#define SST_CHANNELS_MAX	4

struct sst_alloc_stream_rsp {
	uint32_t	stream_hw_id;		/* Assigned stream HW ID */
	uint32_t	reserved;
	uint32_t	read_pos_regaddr;	/* Read position register */
	uint32_t	pres_pos_regaddr;	/* Presentation position register */
	uint32_t	peak_meter_regaddr[SST_CHANNELS_MAX];
	uint32_t	volume_regaddr[SST_CHANNELS_MAX];
} __packed;

/*
 * SSP Device Format (for SET_DEVICE_FORMATS IPC)
 * From Linux catpt: struct catpt_ssp_device_format (messages.h)
 *
 * This configures the SSP port hardware, NOT audio format.
 * 15 bytes packed.
 */

/* SSP interface IDs */
#define SST_SSP_IFACE_0		0
#define SST_SSP_IFACE_1		1

/* MCLK frequencies */
#define SST_MCLK_OFF		0
#define SST_MCLK_FREQ_6_MHZ	1
#define SST_MCLK_FREQ_21_MHZ	2
#define SST_MCLK_FREQ_24_MHZ	3

/* SSP modes */
#define SST_SSP_MODE_I2S_CONSUMER	0
#define SST_SSP_MODE_I2S_PROVIDER	1
#define SST_SSP_MODE_TDM_PROVIDER	2

struct sst_device_format {
	uint32_t	iface;		/* SST_SSP_IFACE_* */
	uint32_t	mclk;		/* SST_MCLK_* */
	uint32_t	mode;		/* SST_SSP_MODE_* */
	uint16_t	clock_divider;	/* Clock divider */
	uint8_t		channels;	/* Number of channels */
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
#define SST_DX_STATE_D3		0x03	/* CATPT_DX_STATE_D3 */

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
#define SST_IPC_REPLY_MAX	256	/* Max reply data cached in ISR */

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

	/* Reply data cached by ISR (before DONE is cleared) */
	uint8_t			reply_data[SST_IPC_REPLY_MAX];
	size_t			reply_size;	/* Bytes captured */

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
			     struct sst_module_entry *modules,
			     int num_modules,
			     struct sst_alloc_stream_rsp *rsp);
int	sst_ipc_free_stream(struct sst_softc *sc, uint32_t stream_id);

/* Stream control */
int	sst_ipc_stream_pause(struct sst_softc *sc, uint32_t stream_id);
int	sst_ipc_stream_resume(struct sst_softc *sc, uint32_t stream_id);
int	sst_ipc_stream_reset(struct sst_softc *sc, uint32_t stream_id);
int	sst_ipc_stream_set_params(struct sst_softc *sc,
				  struct sst_stream_params *params);
int	sst_ipc_stream_get_position(struct sst_softc *sc, uint32_t stream_id,
				    struct sst_stream_position *pos);

/* Device format */
int	sst_ipc_set_device_formats(struct sst_softc *sc,
				   struct sst_device_format *devfmt);

/* Stream write position (for ring buffer management) */
int	sst_ipc_set_write_pos(struct sst_softc *sc, uint32_t stream_id,
			      uint32_t pos, bool end_of_buffer);

/* Mixer control */
int	sst_ipc_set_mixer(struct sst_softc *sc, struct sst_mixer_params *params);
int	sst_ipc_get_mixer(struct sst_softc *sc, struct sst_mixer_params *params);

/* Power management */
int	sst_ipc_set_dx(struct sst_softc *sc, uint32_t state);

#endif /* _SST_IPC_H_ */
