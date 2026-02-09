/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST Firmware Loader
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_FIRMWARE_H_
#define _SST_FIRMWARE_H_

#include <sys/types.h>

/*
 * Firmware file paths
 */
#define SST_FW_PATH_BDW		"intel/IntcSST2.bin"
#define SST_FW_PATH_HSW		"intel/IntcSST2.bin"

/*
 * SST Firmware Header
 * Based on Linux sound/soc/intel/common/sst-firmware.c
 */
struct sst_fw_header {
	char		signature[4];	/* "SST\0" or "$SST" */
	uint32_t	file_size;	/* Total file size */
	uint32_t	modules;	/* Number of modules */
	uint32_t	file_format;	/* File format version */
	uint32_t	reserved[4];	/* Reserved */
} __packed;

#define SST_FW_SIGNATURE	"$SST"
#define SST_FW_SIGN_SIZE	4

/*
 * SST Module Header
 */
struct sst_module_header {
	char		signature[4];	/* "MOD\0" or "$MOD" */
	uint32_t	mod_size;	/* Module size including header */
	uint32_t	blocks;		/* Number of blocks */
	uint16_t	slot;		/* DSP slot index */
	uint16_t	type;		/* Module type */
	uint32_t	entry_point;	/* Entry point address */
} __packed;

#define SST_MOD_SIGNATURE	"$MOD"

/*
 * SST Block Header
 */
struct sst_block_header {
	uint32_t	type;		/* Block type (IRAM/DRAM) */
	uint32_t	size;		/* Block data size */
	uint32_t	ram_offset;	/* Offset in DSP RAM */
	uint32_t	reserved;	/* Reserved */
} __packed;

/*
 * Block Types
 */
#define SST_BLK_TYPE_IRAM	0
#define SST_BLK_TYPE_DRAM	1
#define SST_BLK_TYPE_SRAM	2

/*
 * Firmware State
 */
enum sst_fw_state {
	SST_FW_STATE_NONE = 0,
	SST_FW_STATE_LOADING,
	SST_FW_STATE_LOADED,
	SST_FW_STATE_RUNNING,
	SST_FW_STATE_ERROR
};

/*
 * Firmware Context
 */
struct sst_firmware {
	const struct firmware	*fw;		/* Firmware handle */
	const uint8_t		*data;		/* Firmware data */
	size_t			size;		/* Firmware size */
	uint32_t		entry_point;	/* DSP entry address */
	uint32_t		modules;	/* Number of modules */
	enum sst_fw_state	state;		/* Current state */
};

/* Forward declaration */
struct sst_softc;

/*
 * Firmware API
 */
int	sst_fw_init(struct sst_softc *sc);
void	sst_fw_fini(struct sst_softc *sc);
int	sst_fw_load(struct sst_softc *sc);
void	sst_fw_unload(struct sst_softc *sc);
int	sst_fw_boot(struct sst_softc *sc);

#endif /* _SST_FIRMWARE_H_ */
