/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side mirror of the firmware image bounds-checking predicates
 * used by src/sst_firmware.c.
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * IMPORTANT - KEEP IN SYNC
 * -------------------------
 * src/sst_firmware.c cannot be compiled on a non-FreeBSD host: it pulls
 * in <sys/bus.h>, <sys/firmware.h>, device_t, bus_space accessors, etc.
 * that only exist inside a FreeBSD kernel build.  Rather than stub out
 * that whole environment, this header re-states, verbatim, the small
 * arithmetic predicates that decide whether a firmware-supplied offset
 * or size is safe to use.  Each function below names the exact source
 * function and line range it mirrors as of the fix in commit 2551d9e
 * ("audit: fix critical/high kernel bugs").  If those predicates in
 * src/sst_firmware.c ever change, THIS FILE MUST BE UPDATED TO MATCH,
 * or these tests will silently stop verifying real driver behavior.
 *
 * Struct layouts (sst_module_header, sst_block_header) are copied from
 * src/sst_firmware.h purely so this test can build realistic __packed
 * byte buffers representing a firmware image; they carry no behavior.
 */

#ifndef _SST_FW_PREDICATES_H_
#define _SST_FW_PREDICATES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- struct layouts copied from src/sst_firmware.h --------------- */

struct sst_module_header {
	char		signature[4];
	uint32_t	mod_size;
	uint32_t	blocks;
	uint16_t	slot;
	uint16_t	module_id;
	uint32_t	entry_point;
	uint32_t	persistent_size;
	uint32_t	scratch_size;
} __attribute__((packed));

struct sst_block_header {
	uint32_t	ram_type;
	uint32_t	size;
	uint32_t	ram_offset;
	uint32_t	reserved;
} __attribute__((packed));

#define SST_MOD_SIGNATURE	"$SST"
#define SST_BLK_TYPE_IRAM	1
#define SST_BLK_TYPE_DRAM	2
#define SST_BLK_TYPE_INSTANCE	3

/* ---- constants copied from src/sst_regs.h / src/sst_ipc.h --------- */

#define SST_DRAM_OFFSET		0x000000
#define SST_DRAM_SIZE		0x0A0000	/* 640KB */
#define SST_IRAM_OFFSET		0x0A0000
#define SST_IRAM_SIZE		0x050000	/* 320KB */

#define SST_MBOX_SIZE_IN	0x400		/* Host -> DSP: 1KB */
#define SST_IPC_REPLY_MAX	256		/* Max cached reply bytes */

/*
 * =====================================================================
 * Predicate 1: sst_fw_load_block() region-fit check
 * src/sst_firmware.c, sst_fw_load_block(), ~lines 101-108:
 *
 *	if (blk->ram_offset > max_size ||
 *	    blk->size > max_size - blk->ram_offset) {
 *		return (EINVAL);
 *	}
 *
 * Returns true if the block fits [ram_offset, ram_offset+size) inside
 * a region of max_size bytes, false if it must be rejected.
 * =====================================================================
 */
static inline bool
pred_block_in_region(uint32_t ram_offset, uint32_t size, size_t max_size)
{
	if (ram_offset > max_size || size > max_size - ram_offset)
		return (false);
	return (true);
}

/*
 * Pre-fix formula, kept only to prove the bug it had:
 *	if (blk->ram_offset + blk->size > max_size)
 * Both operands are uint32_t, so the addition happens in 32-bit
 * arithmetic and can wrap around before ever being compared against
 * max_size (which may be a much wider size_t).
 */
static inline bool
pred_block_in_region_OLD_BUGGY(uint32_t ram_offset, uint32_t size,
    size_t max_size)
{
	uint32_t sum = ram_offset + size;	/* may wrap in 32 bits */

	return (!(sum > max_size));
}

/*
 * =====================================================================
 * Predicate 2: sst_fw_load_module() block-header-fits-module check
 * src/sst_firmware.c, sst_fw_load_module(), ~lines 241-246:
 *
 *	if (offset > total_size ||
 *	    sizeof(struct sst_block_header) > total_size - offset) {
 *		return (EINVAL);
 *	}
 * =====================================================================
 */
static inline bool
pred_block_hdr_fits(uint32_t offset, size_t total_size, size_t hdr_size)
{
	if (offset > total_size || hdr_size > total_size - offset)
		return (false);
	return (true);
}

/* Pre-fix formula: if (offset + sizeof(blk_hdr) > total_size) */
static inline bool
pred_block_hdr_fits_OLD_BUGGY(uint32_t offset, size_t total_size,
    uint32_t hdr_size)
{
	uint32_t sum = offset + hdr_size;	/* may wrap in 32 bits */

	return (!((size_t)sum > total_size));
}

/*
 * =====================================================================
 * Predicate 3: sst_fw_load_module() block-data-fits-module check
 * src/sst_firmware.c, sst_fw_load_module(), ~lines 251-258:
 *
 *	if (blk->size > total_size - offset) {
 *		return (EINVAL);
 *	}
 *
 * Precondition established by the caller before this runs: offset has
 * already been checked <= total_size (predicate 2, then advanced past
 * the block header without exceeding total_size).
 * =====================================================================
 */
static inline bool
pred_block_data_fits(uint32_t size, uint32_t offset, size_t total_size)
{
	return (!(size > total_size - offset));
}

/* Pre-fix formula: if (offset + blk->size > total_size) */
static inline bool
pred_block_data_fits_OLD_BUGGY(uint32_t size, uint32_t offset,
    size_t total_size)
{
	uint32_t sum = offset + size;	/* may wrap in 32 bits */

	return (!((size_t)sum > total_size));
}

/*
 * =====================================================================
 * Predicate 4: sst_fw_boot() FW_READY mailbox window check
 * src/sst_firmware.c, sst_fw_boot(), ~lines 899-904:
 *
 *	if (fw_ready.outbox_offset < SST_DRAM_SIZE &&
 *	    fw_ready.inbox_offset < SST_DRAM_SIZE &&
 *	    SST_MBOX_SIZE_IN <= SST_DRAM_SIZE - fw_ready.outbox_offset &&
 *	    SST_IPC_REPLY_MAX <= SST_DRAM_SIZE - fw_ready.inbox_offset &&
 *	    fw_ready.inbox_size > 0 && fw_ready.outbox_size > 0) {
 *		... accept offsets ...
 *	}
 *
 * pred_mbox_window_fits() below models one "offset < region &&
 * window <= region - offset" clause; sst_fw_boot() applies it twice
 * (once for outbox_offset/SST_MBOX_SIZE_IN, once for
 * inbox_offset/SST_IPC_REPLY_MAX).
 * =====================================================================
 */
static inline bool
pred_mbox_window_fits(uint32_t offset, uint32_t window_size,
    uint32_t region_size)
{
	if (offset >= region_size)
		return (false);
	return (window_size <= region_size - offset);
}

/*
 * Pre-fix formula only checked "offset < DRAM_SIZE" with no window-size
 * term at all, which is equivalent to window_size == 0 always fitting.
 * To demonstrate the class of bug a window check must avoid, this
 * models what a naive *added* window check would have looked like if
 * written the wrap-prone way: if (offset + window_size <= region_size)
 */
static inline bool
pred_mbox_window_fits_OLD_BUGGY(uint32_t offset, uint32_t window_size,
    uint32_t region_size)
{
	uint32_t sum = offset + window_size;	/* may wrap in 32 bits */

	return (sum <= region_size);
}

/*
 * =====================================================================
 * Test harness: mirrors the block loop inside sst_fw_load_module()
 * (src/sst_firmware.c ~lines 238-268) using ONLY the predicates above,
 * so end-to-end crafted firmware buffers can be exercised the same way
 * the real parser walks them.  It does not touch any hardware/MMIO
 * path (sst_fw_load_block()'s bus_write_4 loop is not reachable on a
 * Linux host and is intentionally not reimplemented here).
 * =====================================================================
 */
static inline int
test_walk_module_blocks(const uint8_t *data, size_t total_size,
    uint32_t nblocks)
{
	uint32_t offset;
	uint32_t i;

	offset = (uint32_t)sizeof(struct sst_module_header);
	for (i = 0; i < nblocks; i++) {
		const struct sst_block_header *blk;

		if (!pred_block_hdr_fits(offset, total_size,
		    sizeof(struct sst_block_header)))
			return (-1); /* EINVAL: "Block N header exceeds module" */

		blk = (const struct sst_block_header *)(data + offset);
		offset += (uint32_t)sizeof(struct sst_block_header);

		if (!pred_block_data_fits(blk->size, offset, total_size))
			return (-1); /* EINVAL: "Block N data exceeds module" */

		offset += blk->size;
	}
	return (0);
}

#endif /* _SST_FW_PREDICATES_H_ */
