/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Host-side regression test for the firmware image parser bounds
 * checks in src/sst_firmware.c (32-bit overflow in offset+size style
 * comparisons, fixed in commit 2551d9e).
 *
 * This test cannot link against src/sst_firmware.c directly (it needs
 * a FreeBSD kernel build environment: <sys/bus.h>, device_t, bus_space
 * accessors, firmware(9), ...).  Instead it exercises the exact bounds
 * predicates mirrored in sst_fw_predicates.h -- see that file's header
 * comment for the sync contract.
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sst_fw_predicates.h"

static int tests_run;
static int tests_failed;

#define CHECK(cond, msg) do {						\
	tests_run++;							\
	if (!(cond)) {							\
		tests_failed++;					\
		fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__,	\
		    __LINE__);						\
	} else {							\
		printf("ok   %s\n", (msg));				\
	}								\
} while (0)

/*
 * Build a one-block module image into buf:
 *   [sst_module_header][sst_block_header][block data...]
 * "blocks" in the header can be forced to a value different from the
 * number of block headers actually present, to test truncated images.
 */
static size_t
build_one_block_module(uint8_t *buf, size_t bufsz, uint32_t declared_blocks,
    uint32_t block_size_field, uint32_t block_data_len)
{
	struct sst_module_header mh;
	struct sst_block_header bh;
	size_t off = 0;

	memset(&mh, 0, sizeof(mh));
	memcpy(mh.signature, SST_MOD_SIGNATURE, 4);
	mh.blocks = declared_blocks;
	mh.mod_size = 0; /* not used by test_walk_module_blocks() */

	memset(&bh, 0, sizeof(bh));
	bh.ram_type = SST_BLK_TYPE_DRAM;
	bh.size = block_size_field;
	bh.ram_offset = 0;

	if (sizeof(mh) + sizeof(bh) + block_data_len > bufsz)
		abort(); /* test bug, not a driver bug */

	memcpy(buf + off, &mh, sizeof(mh));
	off += sizeof(mh);
	memcpy(buf + off, &bh, sizeof(bh));
	off += sizeof(bh);
	memset(buf + off, 0x42, block_data_len);
	off += block_data_len;

	return (off);
}

/* ---- (a) a fully valid image parses successfully ------------------ */
static void
test_valid_image(void)
{
	uint8_t buf[128];
	size_t total;

	total = build_one_block_module(buf, sizeof(buf), 1, 16, 16);

	CHECK(test_walk_module_blocks(buf, total, 1) == 0,
	    "(a) valid single-block module is accepted");
}

/* ---- (b) block size field wraps 32-bit offset+size arithmetic ----- */
static void
test_wrapping_block_size(void)
{
	uint8_t buf[128];
	size_t total;
	uint32_t offset_after_hdrs;

	/* Real buffer only has 16 bytes of block data ... */
	total = build_one_block_module(buf, sizeof(buf), 1, 0xFFFFFFF0u, 16);

	/* ... but the block header lies and claims ~4GB of data. */
	CHECK(test_walk_module_blocks(buf, total, 1) == -1,
	    "(b) block claiming 0xFFFFFFF0 bytes is rejected by parser");

	/*
	 * Prove this is genuinely a wraparound bug the old formula had:
	 * with these exact numbers, the pre-fix "offset + size > total"
	 * check wraps around and reports the block as fitting.
	 */
	offset_after_hdrs = (uint32_t)(sizeof(struct sst_module_header) +
	    sizeof(struct sst_block_header));
	CHECK(pred_block_data_fits_OLD_BUGGY(0xFFFFFFF0u, offset_after_hdrs,
	    total) == true,
	    "(b) OLD buggy formula wrongly ACCEPTS the wrapped size "
	    "(demonstrates the bug being regression-tested)");
	CHECK(pred_block_data_fits(0xFFFFFFF0u, offset_after_hdrs, total) ==
	    false,
	    "(b) NEW overflow-safe formula correctly REJECTS the same input");
}

/* ---- (c) block header itself runs past the end of the module ----- */
static void
test_block_header_past_end(void)
{
	uint8_t buf[128];
	size_t total;

	/* Buffer physically holds only 1 block, but header claims 2. */
	total = build_one_block_module(buf, sizeof(buf), 2, 16, 16);

	CHECK(test_walk_module_blocks(buf, total, 2) == -1,
	    "(c) declared block count exceeding available data is rejected");
}

/* ---- (d) ram_offset places the block past the RAM region ---------- */
static void
test_ram_offset_past_region(void)
{
	/* Straightforward out-of-range offset, no overflow needed. */
	CHECK(pred_block_in_region(SST_DRAM_SIZE + 0x10, 4, SST_DRAM_SIZE) ==
	    false,
	    "(d) ram_offset beyond DRAM size is rejected outright");

	/* offset is in-range but offset+size wraps in the old formula. */
	CHECK(pred_block_in_region_OLD_BUGGY(0x10, 0xFFFFFFF0u,
	    SST_DRAM_SIZE) == true,
	    "(d) OLD buggy formula wrongly ACCEPTS ram_offset=0x10 "
	    "size=0xFFFFFFF0 against a 640KB region");
	CHECK(pred_block_in_region(0x10, 0xFFFFFFF0u, SST_DRAM_SIZE) == false,
	    "(d) NEW overflow-safe formula correctly REJECTS the same input");

	/* A legitimate IRAM block still fits. */
	CHECK(pred_block_in_region(0x100, 0x200, SST_IRAM_SIZE) == true,
	    "(d) a real in-bounds IRAM block is still accepted");
}

/* ---- (e) FW_READY mailbox window would extend past DRAM ----------- */
static void
test_mbox_window_past_dram(void)
{
	uint32_t near_end;

	/* Legitimate window well inside DRAM. */
	CHECK(pred_mbox_window_fits(0x1000, SST_MBOX_SIZE_IN, SST_DRAM_SIZE)
	    == true,
	    "(e) a mailbox window comfortably inside DRAM is accepted");

	/* Offset within DRAM, but window runs past the end. */
	near_end = SST_DRAM_SIZE - 0x100; /* only 0x100 bytes left */
	CHECK(pred_mbox_window_fits(near_end, SST_MBOX_SIZE_IN,
	    SST_DRAM_SIZE) == false,
	    "(e) a mailbox window overrunning the end of DRAM is rejected");

	/* offset+window wraps 32 bits in the old-style naive formula. */
	CHECK(pred_mbox_window_fits_OLD_BUGGY(0xFFFFFFFFu, SST_MBOX_SIZE_IN,
	    SST_DRAM_SIZE) == true,
	    "(e) naive wrap-prone offset+window formula wrongly ACCEPTS "
	    "offset=0xFFFFFFFF");
	CHECK(pred_mbox_window_fits(0xFFFFFFFFu, SST_MBOX_SIZE_IN,
	    SST_DRAM_SIZE) == false,
	    "(e) NEW overflow-safe formula correctly REJECTS the same input");

	/* Same class of check, using the inbox/SST_IPC_REPLY_MAX pairing. */
	CHECK(pred_mbox_window_fits(SST_DRAM_SIZE - 4, SST_IPC_REPLY_MAX,
	    SST_DRAM_SIZE) == false,
	    "(e) inbox window (SST_IPC_REPLY_MAX) overrunning DRAM end "
	    "is rejected");
}

int
main(void)
{
	test_valid_image();
	test_wrapping_block_size();
	test_block_header_past_end();
	test_ram_offset_past_region();
	test_mbox_window_past_dram();

	printf("\n%d/%d checks passed\n", tests_run - tests_failed,
	    tests_run);

	return (tests_failed == 0 ? 0 : 1);
}
