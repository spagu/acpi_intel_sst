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
 */
#define SST_IPC_GLBL_REPLY		0x00
#define SST_IPC_GLBL_GET_FW_VERSION	0x01
#define SST_IPC_GLBL_ALLOCATE_STREAM	0x02
#define SST_IPC_GLBL_FREE_STREAM	0x03
#define SST_IPC_GLBL_STREAM_MESSAGE	0x04

/*
 * Stream Message Types
 */
#define SST_IPC_STR_PAUSE		0x00
#define SST_IPC_STR_RESUME		0x01
#define SST_IPC_STR_GET_PARAMS		0x02
#define SST_IPC_STR_SET_PARAMS		0x03
#define SST_IPC_STR_GET_POSITION	0x04

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
 * IPC State
 */
enum sst_ipc_state {
	SST_IPC_IDLE = 0,
	SST_IPC_PENDING,
	SST_IPC_DONE,
	SST_IPC_ERROR
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

#endif /* _SST_IPC_H_ */
