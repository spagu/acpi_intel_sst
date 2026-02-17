/*-
 * SPDX-License-Identifier: BSD-3-Clause
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
	mtx_init(&sc->ipc.send_mtx, "sst_ipc_send", NULL, MTX_DEF);
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

	sst_dbg(sc, SST_DBG_LIFE, "IPC initialized: mbox_in=0x%lx, mbox_out=0x%lx\n",
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
	mtx_destroy(&sc->ipc.send_mtx);
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

	/*
	 * Serialize IPC senders.
	 *
	 * cv_timedwait() releases ipc.lock while sleeping, which
	 * allows a second sender (e.g. poll-timer stall recovery)
	 * to enter and overwrite the shared state/mailbox.  The
	 * send_mtx is held across the entire send-wait-complete
	 * cycle so only one IPC transaction is in flight at a time.
	 */
	mtx_lock(&sc->ipc.send_mtx);
	mtx_lock(&sc->ipc.lock);

	/* Check if DSP is busy */
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);
	if (ipcx & SST_IPC_BUSY) {
		device_printf(sc->dev, "IPC: DSP busy\n");
		sc->ipc.error_count++;
		error = EBUSY;
		goto done;
	}

	/*
	 * Write data to mailbox (DRAM/SRAM).
	 *
	 * CRITICAL: Must use 32-bit writes!  PCH SRAM on Broadwell
	 * does not support sub-DWORD MMIO writes - byte writes via
	 * bus_write_region_1() are silently dropped.
	 */
	if (data != NULL && size > 0) {
		uint32_t dwords = size / 4;
		uint32_t remainder = size % 4;
		uint32_t k;
		const uint8_t *src = data;

		for (k = 0; k < dwords; k++) {
			uint32_t val;
			memcpy(&val, src + (k * 4), 4);
			bus_write_4(sc->mem_res,
			    sc->ipc.mbox_in + (k * 4), val);
		}
		if (remainder > 0) {
			uint32_t val = 0;
			memcpy(&val, src + (dwords * 4), remainder);
			bus_write_4(sc->mem_res,
			    sc->ipc.mbox_in + (dwords * 4), val);
		}
	}

	/* Readback verify (first 3 dwords) */
	if (data != NULL && size >= 12) {
		uint32_t rb0, rb1, rb2;
		rb0 = bus_read_4(sc->mem_res, sc->ipc.mbox_in);
		rb1 = bus_read_4(sc->mem_res, sc->ipc.mbox_in + 4);
		rb2 = bus_read_4(sc->mem_res, sc->ipc.mbox_in + 8);
		sst_dbg(sc, SST_DBG_TRACE,
		    "IPC: mbox readback @0x%lx: %08x %08x %08x\n",
		    (unsigned long)sc->ipc.mbox_in, rb0, rb1, rb2);
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
	mtx_unlock(&sc->ipc.send_mtx);

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

	/*
	 * Copy reply data from ISR-cached buffer.
	 * The ISR copies outbox data before clearing DONE to avoid
	 * race with DSP overwriting the outbox.
	 */
	if (data != NULL && size != NULL && *size > 0) {
		recv_size = *size;
		if (recv_size > sc->ipc.reply_size)
			recv_size = sc->ipc.reply_size;

		memcpy(data, sc->ipc.reply_data, recv_size);
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
			sst_dbg(sc, SST_DBG_LIFE,
			    "DSP signaled ready (polled): IPCD=0x%08x\n", ipcd);
		} else {
			sst_dbg(sc, SST_DBG_TRACE,
			    "IPC: DSP notification (polled): 0x%08x\n", ipcd);
		}
		sc->ipc.msg.reply = ipcd;
		sc->ipc.state = SST_IPC_STATE_DONE;
		return (1);
	}

	/*
	 * Check for IPC reply completion (DONE in IPCD)
	 * On Broadwell/WPT, DSP may use DONE bit for ready notification.
	 */
	if (ipcd & SST_IPC_DONE) {
		/* Clear DONE bit */
		sst_shim_write(sc, SST_SHIM_IPCD, ipcd & ~SST_IPC_DONE);

		/*
		 * First reply after boot is the ready notification on WPT.
		 * DSP uses DONE (not BUSY) to signal FW_READY.
		 */
		if (!sc->ipc.ready) {
			sc->ipc.ready = true;
			sst_dbg(sc, SST_DBG_LIFE,
			    "DSP signaled ready via DONE (polled): IPCD=0x%08x\n", ipcd);
		} else {
			sst_dbg(sc, SST_DBG_TRACE, "IPC: Reply done (polled): 0x%08x\n", ipcd);
		}

		sc->ipc.msg.reply = ipcd;
		sc->ipc.state = SST_IPC_STATE_DONE;
		return (1);
	}

	/* Check for our outgoing message being processed (BUSY cleared in IPCX) */
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);
	if ((sc->ipc.state == SST_IPC_STATE_PENDING) && !(ipcx & SST_IPC_BUSY)) {
		sst_dbg(sc, SST_DBG_TRACE, "IPC: Command accepted: IPCX=0x%08x\n", ipcx);
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
		sst_dbg(sc, SST_DBG_TRACE, "IPC: Using polling mode (no IRQ)\n");

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
				sst_dbg(sc, SST_DBG_TRACE,
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
 * catpt IPC protocol (from Linux catpt ipc.c):
 *
 * Host->DSP (IPCX register):
 *   Host sets BUSY to send command.  DSP clears BUSY and sets DONE
 *   when it has processed the command.  Host reads reply from outbox,
 *   then clears DONE.
 *
 * DSP->Host (IPCD register):
 *   DSP sets BUSY to send notification.  Host processes notification,
 *   then clears BUSY and sets DONE to acknowledge.
 */
void
sst_ipc_intr(struct sst_softc *sc)
{
	uint32_t isr, ipcx, ipcd;

	/*
	 * catpt IPC interrupt handler (matches Linux catpt_irq_handler).
	 *
	 * ISRX (ISC in catpt) tells us WHAT fired:
	 *   bit 0 (IPCCD) = DSP set DONE in IPCX → reply to our command
	 *   bit 1 (IPCDB) = DSP set BUSY in IPCD → notification from DSP
	 *
	 * Protocol per channel:
	 *   1. Mask interrupt in IMRX (IMC)
	 *   2. Read register, extract data
	 *   3. Clear register bits
	 *   4. Unmask interrupt in IMRX
	 */
	isr = sst_shim_read(sc, SST_SHIM_ISRX);

	/* Ignore spurious interrupts (hardware not ready or powered down) */
	if (isr == 0 || isr == 0xFFFFFFFF)
		return;

	/*
	 * IPCCD (bit 0): Reply to our command.
	 * DSP cleared BUSY and set DONE in IPCX (IPCC).
	 */
	if (isr & SST_IMC_IPCCD) {
		/* Mask IPCCD interrupt */
		sst_shim_update_bits(sc, SST_SHIM_IMRX,
		    SST_IMC_IPCCD, SST_IMC_IPCCD);

		ipcx = sst_shim_read(sc, SST_SHIM_IPCX);

		/*
		 * Copy reply data from mbox_in BEFORE clearing DONE.
		 *
		 * The catpt firmware writes the IPC reply to the same
		 * mailbox region where the host wrote the command
		 * (mbox_in / "outbox"), overwriting the command data.
		 * The "inbox" region only holds the FW_READY notification.
		 */
		{
			uint32_t dwords, k;
			uint8_t *dst = sc->ipc.reply_data;

			sc->ipc.reply_size = SST_IPC_REPLY_MAX;
			dwords = SST_IPC_REPLY_MAX / 4;
			for (k = 0; k < dwords; k++) {
				uint32_t val = bus_read_4(sc->mem_res,
				    sc->ipc.mbox_in + (k * 4));
				memcpy(dst + (k * 4), &val, 4);
			}
		}

		mtx_lock(&sc->ipc.lock);

		/* Extract status from reply (bits 4:0) */
		sc->ipc.msg.reply = ipcx;
		sc->ipc.msg.status = ipcx & SST_IPC_STATUS_MASK;
		sc->ipc.state = SST_IPC_STATE_DONE;
		cv_signal(&sc->ipc.wait_cv);

		sst_dbg(sc, SST_DBG_TRACE,
		    "IPC reply: IPCX=0x%08x status=%d\n",
		    ipcx, sc->ipc.msg.status);

		mtx_unlock(&sc->ipc.lock);

		/* Clear DONE in IPCX, unmask IPCCD interrupt */
		sst_shim_update_bits(sc, SST_SHIM_IPCX,
		    SST_IPC_DONE, 0);
		sst_shim_update_bits(sc, SST_SHIM_IMRX,
		    SST_IMC_IPCCD, 0);
	}

	/*
	 * IPCDB (bit 1): Notification from DSP.
	 * DSP set BUSY in IPCD.
	 */
	if (isr & SST_IMC_IPCDB) {
		/* Mask IPCDB interrupt */
		sst_shim_update_bits(sc, SST_SHIM_IMRX,
		    SST_IMC_IPCDB, SST_IMC_IPCDB);

		ipcd = sst_shim_read(sc, SST_SHIM_IPCD);

		if (ipcd & SST_IPC_BUSY) {
			mtx_lock(&sc->ipc.lock);

			/* First notification after boot is FW_READY */
			if (!sc->ipc.ready) {
				sc->ipc.ready = true;
				sst_dbg(sc, SST_DBG_LIFE,
				    "IPC: DSP ready: IPCD=0x%08x\n",
				    ipcd);
			} else {
				/* DSP notification (position update etc.) */
				/* Silently ACK - no printf in interrupt path */
			}

			mtx_unlock(&sc->ipc.lock);

			/* ACK: clear BUSY, set DONE in IPCD */
			sst_shim_update_bits(sc, SST_SHIM_IPCD,
			    SST_IPC_BUSY | SST_IPC_DONE, SST_IPC_DONE);
		}

		/* Unmask IPCDB interrupt */
		sst_shim_update_bits(sc, SST_SHIM_IMRX,
		    SST_IMC_IPCDB, 0);
	}
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

	sst_dbg(sc, SST_DBG_OPS, "Firmware version: %u.%u.%u (type=%u)\n",
		reply.version.major, reply.version.minor,
		reply.version.build, reply.version.type);

	return (0);
}

/*
 * Allocate a stream on the DSP (catpt protocol).
 *
 * The catpt firmware expects module entries spliced into the payload
 * between num_entries and persistent_mem fields.
 */
int
sst_ipc_alloc_stream(struct sst_softc *sc, struct sst_alloc_stream_req *req,
		     struct sst_module_entry *modules, int num_modules,
		     struct sst_alloc_stream_rsp *rsp)
{
	uint8_t payload[256];
	uint32_t header;
	size_t off, arrsz, paysize, rspsize;
	int error;

	/*
	 * Build payload with module entries spliced in.
	 * Layout: [req fields up to persistent_mem] [modules] [persistent_mem..end]
	 */
	off = __offsetof(struct sst_alloc_stream_req, persistent_mem);
	arrsz = sizeof(struct sst_module_entry) * num_modules;
	paysize = sizeof(*req) + arrsz;

	if (paysize > sizeof(payload)) {
		device_printf(sc->dev, "IPC: alloc payload too large\n");
		return (ENOMEM);
	}

	/* Copy first part (up to persistent_mem) */
	memcpy(payload, req, off);
	/* Insert module entries */
	if (arrsz > 0)
		memcpy(payload + off, modules, arrsz);
	/* Copy remainder (persistent_mem, scratch_mem, num_notifications) */
	memcpy(payload + off + arrsz, (uint8_t *)req + off,
	    sizeof(*req) - off);

	header = SST_IPC_HEADER(SST_IPC_GLBL_ALLOCATE_STREAM, 0, 0);

	sst_dbg(sc, SST_DBG_OPS,
	    "IPC: alloc stream path=%u type=%u fmt=%u "
	    "paysize=%zu off=%zu arrsz=%zu\n",
	    req->path_id, req->stream_type, req->format_id,
	    paysize, off, arrsz);

	/* Debug: dump full payload in 16-byte lines */
	{
		size_t j;
		for (j = 0; j < paysize; j += 16) {
			size_t end = j + 16;
			size_t k;
			if (end > paysize)
				end = paysize;
			sst_dbg(sc, SST_DBG_TRACE, "IPC: [%02zu]", j);
			for (k = j; k < end; k++)
				printf(" %02x", payload[k]);
			printf("\n");
		}
	}

	error = sst_ipc_send(sc, header, payload, paysize);
	if (error) {
		device_printf(sc->dev, "IPC: Stream alloc send failed: %d\n",
		    error);
		return (error);
	}

	rspsize = sizeof(*rsp);
	error = sst_ipc_recv(sc, NULL, rsp, &rspsize);
	if (error) {
		device_printf(sc->dev, "IPC: Stream alloc recv failed: %d\n",
		    error);
		return (error);
	}

	/* Debug: dump first 48 bytes from both mailbox locations */
	{
		uint32_t d[12];
		int j;

		/* Read from mbox_out (inbox, where DSP should write reply) */
		for (j = 0; j < 12; j++)
			d[j] = bus_read_4(sc->mem_res,
			    sc->ipc.mbox_out + j * 4);
		sst_dbg(sc, SST_DBG_TRACE,
		    "IPC: mbox_out @0x%lx: %08x %08x %08x %08x "
		    "%08x %08x %08x %08x\n",
		    (unsigned long)sc->ipc.mbox_out,
		    d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

		/* Read from mbox_in (outbox, where we sent command) */
		for (j = 0; j < 12; j++)
			d[j] = bus_read_4(sc->mem_res,
			    sc->ipc.mbox_in + j * 4);
		sst_dbg(sc, SST_DBG_TRACE,
		    "IPC: mbox_in  @0x%lx: %08x %08x %08x %08x "
		    "%08x %08x %08x %08x\n",
		    (unsigned long)sc->ipc.mbox_in,
		    d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

		/* Also check the ISR cached reply */
		sst_dbg(sc, SST_DBG_TRACE,
		    "IPC: cached reply[0..7]: %02x %02x %02x %02x "
		    "%02x %02x %02x %02x\n",
		    sc->ipc.reply_data[0], sc->ipc.reply_data[1],
		    sc->ipc.reply_data[2], sc->ipc.reply_data[3],
		    sc->ipc.reply_data[4], sc->ipc.reply_data[5],
		    sc->ipc.reply_data[6], sc->ipc.reply_data[7]);
	}

	sst_dbg(sc, SST_DBG_OPS,
	    "IPC: Allocated stream hw_id=%u read_pos=0x%x pres_pos=0x%x\n",
	    rsp->stream_hw_id, rsp->read_pos_regaddr,
	    rsp->pres_pos_regaddr);

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

	sst_dbg(sc, SST_DBG_OPS, "IPC: Freed stream %u\n", stream_id);

	return (0);
}

/*
 * Pause a stream
 */
int
sst_ipc_stream_pause(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;

	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_PAUSE, stream_id);
	return sst_ipc_send(sc, header, NULL, 0);
}

/*
 * Resume a stream
 */
int
sst_ipc_stream_resume(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;

	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_RESUME, stream_id);
	return sst_ipc_send(sc, header, NULL, 0);
}

/*
 * Reset a stream
 */
int
sst_ipc_stream_reset(struct sst_softc *sc, uint32_t stream_id)
{
	uint32_t header;

	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_RESET, stream_id);
	return sst_ipc_send(sc, header, NULL, 0);
}

/*
 * Set stream volume via STAGE_MESSAGE/SET_VOLUME
 */
int
sst_ipc_stream_set_params(struct sst_softc *sc, struct sst_stream_params *params)
{
	uint32_t header;
	int error;
	struct {
		uint32_t channel;
		uint32_t target_volume;
		uint64_t curve_duration;
		uint32_t curve_type;
	} __packed vol;

	/* Set volume for left channel (Q1.31 passed directly) */
	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE,
	    params->stream_id);
	header |= (SST_IPC_STG_SET_VOLUME << SST_IPC_STR_STAGE_SHIFT);

	memset(&vol, 0, sizeof(vol));
	vol.channel = 0; /* left */
	vol.target_volume = params->volume_left;
	vol.curve_duration = 0;
	vol.curve_type = 0;

	error = sst_ipc_send(sc, header, &vol, sizeof(vol));
	if (error)
		return (error);

	/* Set volume for right channel */
	vol.channel = 1; /* right */
	vol.target_volume = params->volume_right;

	return sst_ipc_send(sc, header, &vol, sizeof(vol));
}

/*
 * Set biquad filter coefficients via STAGE_MESSAGE/SET_BIQUAD.
 * Sends Q2.30 coefficients for both left and right channels.
 */
int
sst_ipc_set_biquad(struct sst_softc *sc, uint32_t stream_id,
		   int32_t b0, int32_t b1, int32_t b2,
		   int32_t a1, int32_t a2)
{
	uint32_t header;

	/* Skip if firmware lacks biquad support */
	if (!sc->fw.has_biquad)
		return (0);
	int error;
	struct {
		uint32_t channel;
		int32_t  coeff_b0;
		int32_t  coeff_b1;
		int32_t  coeff_b2;
		int32_t  coeff_a1;
		int32_t  coeff_a2;
	} __packed biquad;

	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE, stream_id);
	header |= (SST_IPC_STG_SET_BIQUAD << SST_IPC_STR_STAGE_SHIFT);

	/* Left channel */
	memset(&biquad, 0, sizeof(biquad));
	biquad.channel = 0;
	biquad.coeff_b0 = b0;
	biquad.coeff_b1 = b1;
	biquad.coeff_b2 = b2;
	biquad.coeff_a1 = a1;
	biquad.coeff_a2 = a2;

	error = sst_ipc_send(sc, header, &biquad, sizeof(biquad));
	if (error)
		return (error);

	/* Right channel */
	biquad.channel = 1;

	return sst_ipc_send(sc, header, &biquad, sizeof(biquad));
}

/*
 * Set peak limiter parameters via STAGE_MESSAGE/SET_LIMITER.
 * Sends Q2.30 threshold and timing for both left and right channels.
 */
int
sst_ipc_set_limiter(struct sst_softc *sc, uint32_t stream_id,
		    int32_t threshold, uint32_t attack_us,
		    uint32_t release_us)
{
	uint32_t header;

	/* Skip if firmware lacks limiter support */
	if (!sc->fw.has_limiter)
		return (0);
	int error;
	struct {
		uint32_t channel;
		int32_t  threshold;
		uint32_t attack_us;
		uint32_t release_us;
	} __packed limiter;

	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE, stream_id);
	header |= (SST_IPC_STG_SET_LIMITER << SST_IPC_STR_STAGE_SHIFT);

	/* Left channel */
	memset(&limiter, 0, sizeof(limiter));
	limiter.channel = 0;
	limiter.threshold = threshold;
	limiter.attack_us = attack_us;
	limiter.release_us = release_us;

	error = sst_ipc_send(sc, header, &limiter, sizeof(limiter));
	if (error)
		return (error);

	/* Right channel */
	limiter.channel = 1;

	return sst_ipc_send(sc, header, &limiter, sizeof(limiter));
}

/*
 * Get current stream position (reads from DSP register)
 * In catpt, position is read directly from MMIO, not via IPC.
 */
int
sst_ipc_stream_get_position(struct sst_softc *sc, uint32_t stream_id,
			    struct sst_stream_position *pos)
{
	/* Position is read from the register returned by alloc_stream */
	if (pos != NULL)
		memset(pos, 0, sizeof(*pos));
	return (0);
}

/*
 * Set device formats (SSP hardware config) - catpt_ipc_set_device_format
 *
 * Configures the SSP port: interface, MCLK, mode, clock divider, channels.
 * This is NOT audio format - it's SSP hardware configuration.
 */
int
sst_ipc_set_device_formats(struct sst_softc *sc,
			   struct sst_device_format *devfmt)
{
	uint32_t header;

	header = SST_IPC_HEADER(SST_IPC_GLBL_SET_DEVICE_FORMATS, 0, 0);

	sst_dbg(sc, SST_DBG_OPS,
	    "IPC: set_device_formats iface=%u mclk=%u mode=%u "
	    "clkdiv=%u ch=%u (size=%zu)\n",
	    devfmt->iface, devfmt->mclk, devfmt->mode,
	    devfmt->clock_divider, devfmt->channels,
	    sizeof(*devfmt));

	return sst_ipc_send(sc, header, devfmt, sizeof(*devfmt));
}

/*
 * Set stream write position - catpt_ipc_set_write_pos
 *
 * Tells the DSP how far the host has written in the ring buffer.
 * The DSP reads from read_pos up to write_pos.
 *
 * Payload: struct { uint32_t position; bool end_of_buffer; bool low_latency; }
 * Header: STREAM_MESSAGE / STAGE_MESSAGE / SET_WRITE_POS
 */
int
sst_ipc_set_write_pos(struct sst_softc *sc, uint32_t stream_id,
		      uint32_t pos, bool end_of_buffer)
{
	uint32_t header;
	struct {
		uint32_t position;
		uint32_t end_of_buffer;
		uint32_t low_latency;
	} __packed payload;

	header = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE, stream_id);
	header |= (SST_IPC_STG_SET_WRITE_POS << SST_IPC_STR_STAGE_SHIFT);

	memset(&payload, 0, sizeof(payload));
	payload.position = pos;
	payload.end_of_buffer = end_of_buffer ? 1 : 0;
	payload.low_latency = 0;

	return sst_ipc_send(sc, header, &payload, sizeof(payload));
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

	header = SST_IPC_HEADER(SST_IPC_GLBL_ENTER_DX_STATE, 0, 0);

	sst_dbg(sc, SST_DBG_OPS, "IPC: Setting DX state to %u\n", state);

	return sst_ipc_send(sc, header, &dx, sizeof(dx));
}

/*
 * DMA load callback for probe buffer allocations.
 */
static void
sst_probe_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *addr = arg;

	if (error || nseg != 1)
		return;
	*addr = segs[0].ds_addr;
}

/*
 * Probe DSP stage capabilities.
 *
 * Allocate a temporary system stream and send test stage messages
 * (SET_VOLUME, SET_BIQUAD, SET_LIMITER) with safe/bypass parameters.
 * IPC success → capability present; IPC error → capability absent.
 *
 * Must be called after sst_fw_boot() and sst_fw_alloc_module_regions().
 */
int
sst_ipc_probe_stage_caps(struct sst_softc *sc)
{
	struct sst_alloc_stream_req req;
	struct sst_alloc_stream_rsp rsp;
	struct sst_module_entry mod;
	uint32_t mod_id, stream_id;
	void *ring_buf, *pgtbl_buf;
	bus_addr_t ring_phys, pgtbl_phys;
	bus_dma_tag_t ring_tag, pgtbl_tag;
	bus_dmamap_t ring_map, pgtbl_map;
	int error;

	sst_dbg(sc, SST_DBG_OPS, "Probing DSP stage capabilities...\n");

	/*
	 * Allocate a 1-page ring buffer and 1-page page table
	 * using bus_dma for proper DMA-safe memory.
	 */
	ring_tag = NULL;
	pgtbl_tag = NULL;
	ring_buf = NULL;
	pgtbl_buf = NULL;
	ring_phys = 0;
	pgtbl_phys = 0;

	/* Ring buffer DMA allocation */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
	    PAGE_SIZE, 0, BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR,
	    NULL, NULL, PAGE_SIZE, 1, PAGE_SIZE, 0, NULL, NULL, &ring_tag);
	if (error)
		goto fallback;

	error = bus_dmamem_alloc(ring_tag, &ring_buf,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO, &ring_map);
	if (error) {
		bus_dma_tag_destroy(ring_tag);
		goto fallback;
	}

	error = bus_dmamap_load(ring_tag, ring_map, ring_buf,
	    PAGE_SIZE, sst_probe_dma_cb, &ring_phys, BUS_DMA_NOWAIT);
	if (error) {
		bus_dmamem_free(ring_tag, ring_buf, ring_map);
		bus_dma_tag_destroy(ring_tag);
		goto fallback;
	}

	/* Page table DMA allocation */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
	    PAGE_SIZE, 0, BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR,
	    NULL, NULL, PAGE_SIZE, 1, PAGE_SIZE, 0, NULL, NULL, &pgtbl_tag);
	if (error) {
		bus_dmamap_unload(ring_tag, ring_map);
		bus_dmamem_free(ring_tag, ring_buf, ring_map);
		bus_dma_tag_destroy(ring_tag);
		goto fallback;
	}

	error = bus_dmamem_alloc(pgtbl_tag, &pgtbl_buf,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO, &pgtbl_map);
	if (error) {
		bus_dma_tag_destroy(pgtbl_tag);
		bus_dmamap_unload(ring_tag, ring_map);
		bus_dmamem_free(ring_tag, ring_buf, ring_map);
		bus_dma_tag_destroy(ring_tag);
		goto fallback;
	}

	error = bus_dmamap_load(pgtbl_tag, pgtbl_map, pgtbl_buf,
	    PAGE_SIZE, sst_probe_dma_cb, &pgtbl_phys, BUS_DMA_NOWAIT);
	if (error) {
		bus_dmamem_free(pgtbl_tag, pgtbl_buf, pgtbl_map);
		bus_dma_tag_destroy(pgtbl_tag);
		bus_dmamap_unload(ring_tag, ring_map);
		bus_dmamem_free(ring_tag, ring_buf, ring_map);
		bus_dma_tag_destroy(ring_tag);
		goto fallback;
	}

	/* Fill page table: single 20-bit PFN entry (even index = bits[19:0]) */
	{
		uint32_t pfn = (uint32_t)(ring_phys >> PAGE_SHIFT);
		uint32_t *pt = (uint32_t *)pgtbl_buf;
		pt[0] = pfn;	/* Even entry: PFN in low 20 bits */
	}
	bus_dmamap_sync(pgtbl_tag, pgtbl_map, BUS_DMASYNC_PREWRITE);

	/* Allocate temporary system stream */
	mod_id = SST_MODID_PCM_SYSTEM;

	memset(&req, 0, sizeof(req));
	req.path_id = SST_PATH_SSP0_OUT;
	req.stream_type = SST_STREAM_TYPE_SYSTEM;
	req.format_id = SST_FMT_PCM;
	req.format.sample_rate = 48000;
	req.format.bit_depth = 16;
	req.format.valid_bit_depth = 16;
	req.format.num_channels = 2;
	req.format.channel_config = SST_CHAN_CONFIG_STEREO;
	req.format.channel_map = 0xFFFFFF20;
	req.format.interleaving = SST_INTERLEAVING_PER_CHANNEL;

	req.ring.page_table_addr = (uint32_t)pgtbl_phys;
	req.ring.num_pages = 1;
	req.ring.size = PAGE_SIZE;
	req.ring.offset = 0;
	req.ring.first_pfn = (uint32_t)(ring_phys >> PAGE_SHIFT);

	req.num_entries = 1;
	memset(&mod, 0, sizeof(mod));
	mod.module_id = mod_id;
	if (mod_id < SST_MAX_MODULES && sc->fw.mod[mod_id].present)
		mod.entry_point = sc->fw.mod[mod_id].entry_point;

	if (mod_id < SST_MAX_MODULES && sc->fw.mod[mod_id].present) {
		req.persistent_mem.offset = SST_DSP_DRAM_OFFSET +
		    sc->fw.mod[mod_id].persistent_offset;
		req.persistent_mem.size = sc->fw.mod[mod_id].persistent_size;
		if (sc->fw.mod[mod_id].scratch_size > 0) {
			req.scratch_mem.offset = SST_DSP_DRAM_OFFSET +
			    sc->fw.mod[mod_id].scratch_offset;
			req.scratch_mem.size =
			    sc->fw.mod[mod_id].scratch_size;
		}
	}
	req.num_notifications = 0;

	memset(&rsp, 0, sizeof(rsp));
	error = sst_ipc_alloc_stream(sc, &req, &mod, 1, &rsp);
	if (error) {
		device_printf(sc->dev,
		    "Stage probe: stream alloc failed (%d), "
		    "assuming all caps present\n", error);
		goto cleanup;
	}

	stream_id = rsp.stream_hw_id;

	/* Probe SET_VOLUME (unity gain, always expected to work) */
	{
		uint32_t hdr;
		struct {
			uint32_t channel;
			uint32_t target_volume;
			uint64_t curve_duration;
			uint32_t curve_type;
		} __packed vol;

		hdr = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE,
		    stream_id);
		hdr |= (SST_IPC_STG_SET_VOLUME << SST_IPC_STR_STAGE_SHIFT);

		memset(&vol, 0, sizeof(vol));
		vol.channel = 0;
		vol.target_volume = 0x7FFFFFFF;	/* Unity gain */

		error = sst_ipc_send(sc, hdr, &vol, sizeof(vol));
		sc->fw.has_volume = (error == 0);
	}

	/* Probe SET_BIQUAD (bypass: b0=1.0 Q2.30, rest=0) */
	{
		uint32_t hdr;
		struct {
			uint32_t channel;
			int32_t  coeff_b0;
			int32_t  coeff_b1;
			int32_t  coeff_b2;
			int32_t  coeff_a1;
			int32_t  coeff_a2;
		} __packed bq;

		hdr = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE,
		    stream_id);
		hdr |= (SST_IPC_STG_SET_BIQUAD << SST_IPC_STR_STAGE_SHIFT);

		memset(&bq, 0, sizeof(bq));
		bq.channel = 0;
		bq.coeff_b0 = 0x40000000;	/* 1.0 in Q2.30 */

		error = sst_ipc_send(sc, hdr, &bq, sizeof(bq));
		sc->fw.has_biquad = (error == 0);
	}

	/* Probe SET_LIMITER (bypass: max threshold) */
	{
		uint32_t hdr;
		struct {
			uint32_t channel;
			int32_t  threshold;
			uint32_t attack_us;
			uint32_t release_us;
		} __packed lim;

		hdr = SST_IPC_STREAM_HEADER(SST_IPC_STR_STAGE_MESSAGE,
		    stream_id);
		hdr |= (SST_IPC_STG_SET_LIMITER << SST_IPC_STR_STAGE_SHIFT);

		memset(&lim, 0, sizeof(lim));
		lim.channel = 0;
		lim.threshold = 0x7FFFFFFF;	/* Max (bypass) */
		lim.attack_us = 0;
		lim.release_us = 0;

		error = sst_ipc_send(sc, hdr, &lim, sizeof(lim));
		sc->fw.has_limiter = (error == 0);
	}

	sc->fw.caps_probed = true;

	sst_dbg(sc, SST_DBG_OPS,
	    "DSP stage caps: volume=%d biquad=%d limiter=%d\n",
	    sc->fw.has_volume, sc->fw.has_biquad, sc->fw.has_limiter);

	/* Free temporary stream */
	sst_ipc_stream_reset(sc, stream_id);
	sst_ipc_free_stream(sc, stream_id);

cleanup:
	/* Free DMA allocations */
	bus_dmamap_unload(pgtbl_tag, pgtbl_map);
	bus_dmamem_free(pgtbl_tag, pgtbl_buf, pgtbl_map);
	bus_dma_tag_destroy(pgtbl_tag);
	bus_dmamap_unload(ring_tag, ring_map);
	bus_dmamem_free(ring_tag, ring_buf, ring_map);
	bus_dma_tag_destroy(ring_tag);

	return (0);

fallback:
	/* Cannot allocate DMA memory; assume all stages present */
	device_printf(sc->dev,
	    "Stage probe: DMA alloc failed, assuming all caps present\n");
	sc->fw.has_volume  = true;
	sc->fw.has_biquad  = true;
	sc->fw.has_limiter = true;
	sc->fw.caps_probed = false;
	return (0);
}
