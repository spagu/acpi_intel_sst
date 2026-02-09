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

	sc->ipc.state = SST_IPC_IDLE;
	sc->ipc.ready = false;

	/* Initialize mailbox addresses */
	sc->ipc.mbox_in = SST_MBOX_OFFSET;
	sc->ipc.mbox_out = SST_MBOX_OFFSET + SST_MBOX_SIZE_IN;

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
	sc->ipc.state = SST_IPC_PENDING;

	/* Send header with BUSY bit set */
	sst_shim_write(sc, SST_SHIM_IPCX, header | SST_IPC_BUSY);
	sc->ipc.tx_count++;

	/* Wait for reply (with timeout) */
	timeout = SST_IPC_TIMEOUT_MS * hz / 1000;
	while (sc->ipc.state == SST_IPC_PENDING && timeout > 0) {
		error = cv_timedwait(&sc->ipc.wait_cv, &sc->ipc.lock, timeout);
		if (error == EWOULDBLOCK) {
			device_printf(sc->dev, "IPC timeout\n");
			sc->ipc.state = SST_IPC_ERROR;
			sc->ipc.error_count++;
			error = ETIMEDOUT;
			goto done;
		}
		timeout -= hz / 10; /* Approximate remaining time */
	}

	if (sc->ipc.state == SST_IPC_ERROR) {
		error = EIO;
		goto done;
	}

	error = sc->ipc.msg.status;

done:
	sc->ipc.state = SST_IPC_IDLE;
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
 * Wait for DSP ready signal
 */
int
sst_ipc_wait_ready(struct sst_softc *sc, int timeout_ms)
{
	int timeout;
	int error = 0;

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
 */
void
sst_ipc_intr(struct sst_softc *sc)
{
	uint32_t isr, ipcd, ipcx;

	/* Read interrupt status */
	isr = sst_shim_read(sc, SST_SHIM_ISRX);

	/* Check for IPC reply (DONE from DSP) */
	ipcd = sst_shim_read(sc, SST_SHIM_IPCD);
	if (ipcd & SST_IPC_DONE) {
		/* Clear DONE bit */
		sst_shim_write(sc, SST_SHIM_IPCD, ipcd & ~SST_IPC_DONE);

		mtx_lock(&sc->ipc.lock);

		/* First message after boot is "ready" */
		if (!sc->ipc.ready) {
			sc->ipc.ready = true;
			device_printf(sc->dev, "DSP signaled ready\n");
		}

		/* Store reply and wake up sender */
		sc->ipc.msg.reply = ipcd;
		sc->ipc.state = SST_IPC_DONE;
		cv_signal(&sc->ipc.wait_cv);

		mtx_unlock(&sc->ipc.lock);
	}

	/* Check for IPC message from DSP */
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);
	if (ipcx & SST_IPC_BUSY) {
		/* Clear BUSY bit to acknowledge */
		sst_shim_write(sc, SST_SHIM_IPCX, ipcx & ~SST_IPC_BUSY);

		/* Handle unsolicited messages from DSP */
		device_printf(sc->dev, "IPC: DSP message received: 0x%08x\n",
			      ipcx);
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
