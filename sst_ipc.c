/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST IPC Protocol Implementation
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>

#include <machine/bus.h>

#include "acpi_intel_sst.h"
#include "sst_regs.h"
#include "sst_ipc.h"

/*
 * Initialize IPC subsystem
 */
int
sst_ipc_init(struct sst_softc *sc)
{
	mtx_init(&sc->ipc.lock, "sst_ipc", NULL, MTX_DEF);
	cv_init(&sc->ipc.wait_cv, "sst_ipc_cv");

	sc->ipc.state = SST_IPC_STATE_IDLE;
	sc->ipc.ready = false;

	/* Initialize mailbox addresses (from host perspective) */
	sc->ipc.mbox_in = SST_MBOX_INBOX_OFFSET;	/* Host -> DSP */
	sc->ipc.mbox_out = SST_MBOX_OUTBOX_OFFSET;	/* DSP -> Host */

	/* Clear message context */
	memset(&sc->ipc.msg, 0, sizeof(sc->ipc.msg));

	/* Statistics */
	sc->ipc.tx_count = 0;
	sc->ipc.rx_count = 0;
	sc->ipc.error_count = 0;

	device_printf(sc->dev, "IPC initialized: mbox_in=0x%lx, mbox_out=0x%lx\n",
		      (unsigned long)sc->ipc.mbox_in,
		      (unsigned long)sc->ipc.mbox_out);

	return (0);
}

/*
 * Cleanup IPC subsystem
 */
void
sst_ipc_fini(struct sst_softc *sc)
{
	cv_destroy(&sc->ipc.wait_cv);
	mtx_destroy(&sc->ipc.lock);
}

/*
 * Send IPC message to DSP
 */
int
sst_ipc_send(struct sst_softc *sc, uint32_t header, void *data, size_t size)
{
	uint32_t ipcx;
	int error = 0;
	int timeout;

	if (size > SST_MBOX_SIZE_IN) {
		device_printf(sc->dev, "IPC message too large: %zu\n", size);
		return (EINVAL);
	}

	mtx_lock(&sc->ipc.lock);

	/* Check if DSP is busy */
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);
	if (ipcx & SST_IPC_BUSY) {
		device_printf(sc->dev, "IPC: DSP busy\n");
		sc->ipc.error_count++;
		error = EBUSY;
		goto done;
	}

	/* Write data to mailbox */
	if (data != NULL && size > 0) {
		bus_write_region_1(sc->mem_res, sc->ipc.mbox_in, data, size);
	}

	/* Setup message context */
	sc->ipc.msg.header = header;
	sc->ipc.msg.data = data;
	sc->ipc.msg.size = size;
	sc->ipc.msg.reply = 0;
	sc->ipc.msg.status = 0;
	sc->ipc.state = SST_IPC_STATE_PENDING;

	/* Send header with BUSY bit set */
	sst_shim_write(sc, SST_SHIM_IPCX, header | SST_IPC_BUSY);
	sc->ipc.tx_count++;

	/* Wait for reply (with timeout) */
	timeout = SST_IPC_TIMEOUT_MS * hz / 1000;
	while (sc->ipc.state == SST_IPC_STATE_PENDING && timeout > 0) {
		error = cv_timedwait(&sc->ipc.wait_cv, &sc->ipc.lock, timeout);
		if (error == EWOULDBLOCK) {
			device_printf(sc->dev, "IPC timeout\n");
			sc->ipc.state = SST_IPC_STATE_ERROR;
			sc->ipc.error_count++;
			error = ETIMEDOUT;
			goto done;
		}
		timeout -= hz / 10; /* Approximate remaining time */
	}

	if (sc->ipc.state == SST_IPC_STATE_ERROR) {
		error = EIO;
		goto done;
	}

	error = sc->ipc.msg.status;

done:
	sc->ipc.state = SST_IPC_STATE_IDLE;
	mtx_unlock(&sc->ipc.lock);

	return (error);
}

/*
 * Receive IPC reply from DSP
 */
int
sst_ipc_recv(struct sst_softc *sc, uint32_t *header, void *data, size_t *size)
{
	uint32_t ipcd;
	size_t recv_size;

	mtx_lock(&sc->ipc.lock);

	/* Read reply header */
	ipcd = sst_shim_read(sc, SST_SHIM_IPCD);

	if (header != NULL)
		*header = ipcd;

	/* Read data from mailbox */
	recv_size = (ipcd & SST_IPC_SIZE_MASK) >> SST_IPC_SIZE_SHIFT;
	if (recv_size > SST_MBOX_SIZE_OUT)
		recv_size = SST_MBOX_SIZE_OUT;

	if (data != NULL && size != NULL && *size > 0) {
		if (recv_size > *size)
			recv_size = *size;
		bus_read_region_1(sc->mem_res, sc->ipc.mbox_out, data,
				  recv_size);
		*size = recv_size;
	}

	sc->ipc.rx_count++;

	mtx_unlock(&sc->ipc.lock);

	return (0);
}

/*
 * Poll IPC registers for DSP ready (used when IRQ unavailable)
 *
 * IPC protocol:
 *   IPCX (Host->DSP): Host sets BUSY to send, DSP clears BUSY when done
 *   IPCD (DSP->Host): DSP sets BUSY to send notification/ready, Host acknowledges
 *
 * DSP signals ready by setting BUSY bit in IPCD (not DONE).
 */
static int
sst_ipc_poll_ready(struct sst_softc *sc)
{
	uint32_t ipcd, ipcx;

	/* Check for message from DSP (BUSY in IPCD = DSP sent message to Host) */
	ipcd = sst_shim_read(sc, SST_SHIM_IPCD);
	if (ipcd & SST_IPC_BUSY) {
		/* Acknowledge: clear BUSY, set DONE */
		sst_shim_write(sc, SST_SHIM_IPCD,
			       (ipcd & ~SST_IPC_BUSY) | SST_IPC_DONE);

		/* First message after boot is "ready" notification */
		if (!sc->ipc.ready) {
			sc->ipc.ready = true;
			device_printf(sc->dev,
			    "DSP signaled ready (polled): IPCD=0x%08x\n", ipcd);
		} else {
			device_printf(sc->dev,
			    "IPC: DSP notification (polled): 0x%08x\n", ipcd);
		}
		sc->ipc.msg.reply = ipcd;
		sc->ipc.state = SST_IPC_STATE_DONE;
		return (1);
	}

	/* Check for IPC reply completion (DONE in IPCD = DSP finished our request) */
	if (ipcd & SST_IPC_DONE) {
		/* Clear DONE bit */
		sst_shim_write(sc, SST_SHIM_IPCD, ipcd & ~SST_IPC_DONE);
		device_printf(sc->dev, "IPC: Reply done (polled): 0x%08x\n", ipcd);
		sc->ipc.msg.reply = ipcd;
		sc->ipc.state = SST_IPC_STATE_DONE;
		return (1);
	}

	/* Check for our outgoing message being processed (BUSY cleared in IPCX) */
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);
	if ((sc->ipc.state == SST_IPC_STATE_PENDING) && !(ipcx & SST_IPC_BUSY)) {
		device_printf(sc->dev, "IPC: Command accepted: IPCX=0x%08x\n", ipcx);
		return (1);
	}

	return (0);
}

/*
 * Wait for DSP ready signal
 */
int
sst_ipc_wait_ready(struct sst_softc *sc, int timeout_ms)
{
	int timeout;
	int error = 0;
	int poll_interval_ms = 100;
	int elapsed = 0;

	/* If no IRQ, use polling mode */
	if (sc->irq_res == NULL) {
		device_printf(sc->dev, "IPC: Using polling mode (no IRQ)\n");

		while (elapsed < timeout_ms) {
			/* Poll IPC registers */
			sst_ipc_poll_ready(sc);

			if (sc->ipc.ready)
				return (0);

			/* Also check SHIM CSR for DSP status */
			if (elapsed % 1000 == 0) {
				uint32_t csr = sst_shim_read(sc, SST_SHIM_CSR);
				uint32_t isr = sst_shim_read(sc, SST_SHIM_ISRX);
				uint32_t ipcd = sst_shim_read(sc, SST_SHIM_IPCD);
				device_printf(sc->dev,
				    "IPC poll: CSR=0x%08x ISR=0x%08x IPCD=0x%08x\n",
				    csr, isr, ipcd);
			}

			DELAY(poll_interval_ms * 1000);
			elapsed += poll_interval_ms;
		}

		return (ETIMEDOUT);
	}

	/* IRQ mode - use condition variable */
	timeout = timeout_ms * hz / 1000;

	mtx_lock(&sc->ipc.lock);

	while (!sc->ipc.ready && timeout > 0) {
		error = cv_timedwait(&sc->ipc.wait_cv, &sc->ipc.lock, hz / 10);
		if (error == EWOULDBLOCK) {
			timeout -= hz / 10;
			error = 0;
			continue;
		}
	}

	if (!sc->ipc.ready) {
		error = ETIMEDOUT;
	}

	mtx_unlock(&sc->ipc.lock);

	return (error);
}

/*
 * IPC interrupt handler
 * Called from main interrupt handler
 *
 * IPC protocol:
 *   IPCD (DSP->Host): DSP sets BUSY to send notification, Host clears BUSY + sets DONE
 *   IPCX (Host->DSP): DSP sets DONE when our command completes
 */
void
sst_ipc_intr(struct sst_softc *sc)
{
	uint32_t isr, ipcd;

	/* Read interrupt status */
	isr = sst_shim_read(sc, SST_SHIM_ISRX);
	ipcd = sst_shim_read(sc, SST_SHIM_IPCD);

	/* Check for notification from DSP (BUSY in IPCD) */
	if (ipcd & SST_IPC_BUSY) {
		/* Acknowledge: clear BUSY, set DONE */
		sst_shim_write(sc, SST_SHIM_IPCD,
			       (ipcd & ~SST_IPC_BUSY) | SST_IPC_DONE);

		mtx_lock(&sc->ipc.lock);

		/* First message after boot is "ready" notification */
		if (!sc->ipc.ready) {
			sc->ipc.ready = true;
			device_printf(sc->dev, "DSP signaled ready: IPCD=0x%08x\n",
				      ipcd);
		} else {
			device_printf(sc->dev, "IPC: DSP notification: 0x%08x\n",
				      ipcd);
		}

		/* Store reply and wake up waiter */
		sc->ipc.msg.reply = ipcd;
		cv_signal(&sc->ipc.wait_cv);

		mtx_unlock(&sc->ipc.lock);
	}

	/* Check for reply completion (DONE in IPCD - DSP finished our command) */
	if (ipcd & SST_IPC_DONE) {
		/* Clear DONE bit */
		sst_shim_write(sc, SST_SHIM_IPCD, ipcd & ~SST_IPC_DONE);

		mtx_lock(&sc->ipc.lock);

		/* Store reply and wake up sender */
		sc->ipc.msg.reply = ipcd;
		sc->ipc.state = SST_IPC_STATE_DONE;
		cv_signal(&sc->ipc.wait_cv);

		mtx_unlock(&sc->ipc.lock);
	}

	/* Clear interrupt status */
	sst_shim_write(sc, SST_SHIM_ISRX, isr);
}

/*
 * Get firmware version
 */
int
sst_ipc_get_fw_version(struct sst_softc *sc, struct sst_fw_version *version)
{
	struct sst_fw_version_reply reply;
	uint32_t header;
	size_t size;
	int error;

	header = SST_IPC_HEADER(SST_IPC_GLBL_GET_FW_VERSION, 0, 0);

	error = sst_ipc_send(sc, header, NULL, 0);
	if (error)
		return (error);

	size = sizeof(reply);
	error = sst_ipc_recv(sc, NULL, &reply, &size);
	if (error)
		return (error);

	if (version != NULL)
		*version = reply.version;

	device_printf(sc->dev, "Firmware version: %u.%u.%u (type=%u)\n",
		      reply.version.major, reply.version.minor,
		      reply.version.build, reply.version.type);

	return (0);
}

/*
 * Allocate a stream on the DSP
 */
int
sst_ipc_alloc_stream(struct sst_softc *sc, struct sst_alloc_stream_req *req,
		     uint32_t *stream_id)
{
	struct sst_alloc_stream_rsp rsp;
	uint32_t header;
	size_t size;
	int error;

	header = SST_IPC_HEADER(SST_IPC_GLBL_ALLOCATE_STREAM, 0,
				sizeof(*req));

	error = sst_ipc_send(sc, header, req, sizeof(*req));
	if (error) {
		device_printf(sc->dev, "IPC: Stream allocation send failed\n");
		return (error);
	}

	size = sizeof(rsp);
	error = sst_ipc_recv(sc, NULL, &rsp, &size);
	if (error) {
		device_printf(sc->dev, "IPC: Stream allocation recv failed\n");
		return (error);
	}

	if (rsp.status != SST_IPC_REPLY_SUCCESS) {
		device_printf(sc->dev, "IPC: Stream allocation failed: %u\n",
			      rsp.status);
		return (EIO);
	}

	if (stream_id != NULL)
		*stream_id = rsp.stream_id;

	device_printf(sc->dev, "IPC: Allocated stream %u\n", rsp.stream_id);

	return (0);
}

/*
 * Free a stream on the DSP
 */
int
sst_ipc_free_stream(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;
	int error;

	header = SST_IPC_HEADER(SST_IPC_GLBL_FREE_STREAM, stream_id, 0);

	error = sst_ipc_send(sc, header, NULL, 0);
	if (error) {
		device_printf(sc->dev, "IPC: Stream free failed: %d\n", error);
		return (error);
	}

	device_printf(sc->dev, "IPC: Freed stream %u\n", stream_id);

	return (0);
}

/*
 * Pause a stream
 */
int
sst_ipc_stream_pause(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;

	header = SST_IPC_HEADER(SST_IPC_GLBL_STREAM_MESSAGE,
				SST_IPC_STR_PAUSE, 0);
	header |= (stream_id << 4);

	return sst_ipc_send(sc, header, NULL, 0);
}

/*
 * Resume a stream
 */
int
sst_ipc_stream_resume(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;

	header = SST_IPC_HEADER(SST_IPC_GLBL_STREAM_MESSAGE,
				SST_IPC_STR_RESUME, 0);
	header |= (stream_id << 4);

	return sst_ipc_send(sc, header, NULL, 0);
}

/*
 * Reset a stream
 */
int
sst_ipc_stream_reset(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;

	header = SST_IPC_HEADER(SST_IPC_GLBL_STREAM_MESSAGE,
				SST_IPC_STR_RESET, 0);
	header |= (stream_id << 4);

	return sst_ipc_send(sc, header, NULL, 0);
}

/*
 * Set stream parameters (volume, mute)
 */
int
sst_ipc_stream_set_params(struct sst_softc *sc, struct sst_stream_params *params)
{
	uint32_t header;

	header = SST_IPC_HEADER(SST_IPC_GLBL_STREAM_MESSAGE,
				SST_IPC_STR_SET_PARAMS, sizeof(*params));
	header |= (params->stream_id << 4);

	return sst_ipc_send(sc, header, params, sizeof(*params));
}

/*
 * Get current stream position
 */
int
sst_ipc_stream_get_position(struct sst_softc *sc, uint32_t stream_id,
			    struct sst_stream_position *pos)
{
	uint32_t header;
	size_t size;
	int error;

	header = SST_IPC_HEADER(SST_IPC_GLBL_STREAM_MESSAGE,
				SST_IPC_STR_GET_POSITION, 0);
	header |= (stream_id << 4);

	error = sst_ipc_send(sc, header, NULL, 0);
	if (error)
		return (error);

	if (pos != NULL) {
		size = sizeof(*pos);
		error = sst_ipc_recv(sc, NULL, pos, &size);
	}

	return (error);
}

/*
 * Set mixer parameters
 */
int
sst_ipc_set_mixer(struct sst_softc *sc, struct sst_mixer_params *params)
{
	uint32_t header;

	header = SST_IPC_HEADER(SST_IPC_GLBL_SET_MIXER_PARAMS, 0,
				sizeof(*params));

	return sst_ipc_send(sc, header, params, sizeof(*params));
}

/*
 * Get mixer parameters
 */
int
sst_ipc_get_mixer(struct sst_softc *sc, struct sst_mixer_params *params)
{
	uint32_t header;
	size_t size;
	int error;

	header = SST_IPC_HEADER(SST_IPC_GLBL_GET_MIXER_PARAMS, 0, 0);

	error = sst_ipc_send(sc, header, NULL, 0);
	if (error)
		return (error);

	if (params != NULL) {
		size = sizeof(*params);
		error = sst_ipc_recv(sc, NULL, params, &size);
	}

	return (error);
}

/*
 * Set device power state (D0/D3)
 */
int
sst_ipc_set_dx(struct sst_softc *sc, uint32_t state)
{
	struct sst_dx_state dx;
	uint32_t header;

	memset(&dx, 0, sizeof(dx));
	dx.state = state;

	header = SST_IPC_HEADER(SST_IPC_GLBL_SET_DX, 0, sizeof(dx));

	device_printf(sc->dev, "IPC: Setting DX state to %u\n", state);

	return sst_ipc_send(sc, header, &dx, sizeof(dx));
}
