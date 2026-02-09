/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST DSP Register Definitions
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#ifndef _SST_REGS_H_
#define _SST_REGS_H_

/*
 * Memory Regions (Broadwell-U)
 */
#define SST_DSP_BAR		0	/* PCI BAR for DSP */

/*
 * ADSP Memory Map (Haswell/Broadwell-U)
 * All offsets within BAR0 (LPE memory region)
 * Based on Linux catpt driver
 */
#define SST_IRAM_OFFSET		0x00000		/* Instruction RAM */
#define SST_IRAM_SIZE		0x14000		/* 80KB */
#define SST_DRAM_OFFSET		0x80000		/* Data RAM */
#define SST_DRAM_SIZE		0x28000		/* 160KB */
/*
 * SHIM registers at offset 0xC0000 within LPE memory (BAR0)
 * Note: Power gating must be disabled first via PCI config (VDRTCTL0)
 */
#define SST_SHIM_OFFSET		0xC0000		/* SHIM registers */

/*
 * PCI Extended Config registers (accessed via BAR1 / shim_res)
 * Used for power gating control
 * Based on Linux catpt driver register definitions
 */
#define SST_PCI_VDRTCTL0	0xA0	/* Power gating control */
#define SST_PCI_VDRTCTL2	0xA8	/* Clock gating control */

/* VDRTCTL0 bits - from Linux catpt driver */
#define SST_VDRTCTL0_DSRAMPGE_MASK	0xFF		/* Bits 0-7: SRAM Power Gate Enable */
#define SST_VDRTCTL0_D3SRAMPGD		(1 << 8)	/* Bit 8: D3 SRAM Power Gate Disable */
#define SST_VDRTCTL0_D3PGD		(1 << 16)	/* Bit 16: D3 Power Gate Disable */

/* VDRTCTL2 bits */
#define SST_VDRTCTL2_DCLCGE		(1 << 1)	/* Dynamic Clock Gating Enable */
#define SST_VDRTCTL2_DTCGE		(1 << 10)	/* Trunk Clock Gating Enable */
#define SST_VDRTCTL2_APLLSE_MASK	(1 << 31)	/* Audio PLL Shutdown Enable */
#define SST_SHIM_SIZE		0x1000		/* 4KB */
/*
 * Mailbox offsets (from DSP perspective)
 * OUTBOX: DSP writes, Host reads (0xE0000)
 * INBOX:  DSP reads, Host writes (0xE0400)
 */
#define SST_MBOX_OUTBOX_OFFSET	0xE0000		/* DSP -> Host */
#define SST_MBOX_INBOX_OFFSET	0xE0400		/* Host -> DSP */
#define SST_MBOX_SIZE		0x400		/* 1KB each */

/*
 * SHIM Registers (Control/Status)
 */
#define SST_SHIM_CSR		0x00	/* Control/Status Register */
#define SST_SHIM_PISR		0x08	/* Platform Interrupt Status */
#define SST_SHIM_PIMR		0x10	/* Platform Interrupt Mask */
#define SST_SHIM_ISRX		0x18	/* IPC Interrupt Status (RX) */
#define SST_SHIM_ISRD		0x20	/* IPC Interrupt Status (Done) */
#define SST_SHIM_IMRX		0x28	/* IPC Interrupt Mask (RX) */
#define SST_SHIM_IMRD		0x30	/* IPC Interrupt Mask (Done) */
#define SST_SHIM_IPCX		0x38	/* IPC Command (Host -> DSP) */
#define SST_SHIM_IPCD		0x40	/* IPC Data (DSP -> Host) */
#define SST_SHIM_ISRSC		0x48	/* IPC Interrupt Status (SCU) */
#define SST_SHIM_ISRLPESC	0x50	/* IPC Interrupt Status (LPESC) */
#define SST_SHIM_IMRSC		0x58	/* IPC Interrupt Mask (SCU) */
#define SST_SHIM_IMRLPESC	0x60	/* IPC Interrupt Mask (LPESC) */
#define SST_SHIM_IPCSC		0x68	/* IPC SCU */
#define SST_SHIM_IPCLPESC	0x70	/* IPC LPESC */
#define SST_SHIM_CLKCTL		0x78	/* Clock Control */
#define SST_SHIM_CSR2		0x80	/* Control/Status Register 2 */

/*
 * CSR Bits
 */
#define SST_CSR_RST		(1 << 1)	/* DSP Reset */
#define SST_CSR_STALL		(1 << 0)	/* DSP Stall */
#define SST_CSR_PWAITMODE	(1 << 2)	/* Power Wait Mode */
#define SST_CSR_DCS_MASK	(0x7 << 4)	/* DSP Clock Select Mask */
#define SST_CSR_DCS_SHIFT	4
#define SST_CSR_SBCS_MASK	(0x7 << 8)	/* SRAM Bank Clock Select */
#define SST_CSR_SBCS_SHIFT	8
#define SST_CSR_S0IOCS		(1 << 21)	/* S0 Idle Clock Select */
#define SST_CSR_S1IOCS		(1 << 23)	/* S1 Idle Clock Select */
#define SST_CSR_LPCS		(1 << 31)	/* Low Power Clock Select */

/*
 * IPC Bits
 */
#define SST_IPC_BUSY		(1U << 31)	/* IPC Busy */
#define SST_IPC_DONE		(1U << 30)	/* IPC Done */

/*
 * Platform Interrupt Status/Mask (PISR/PIMR)
 */
#define SST_PIMR_IPCX		(1 << 0)	/* IPC Interrupt from Host */
#define SST_PIMR_IPCD		(1 << 1)	/* IPC Interrupt to Host */

/*
 * Clock Control Register
 */
#define SST_CLKCTL_SMOS_MASK	(0x3 << 24)
#define SST_CLKCTL_SMOS_SHIFT	24
#define SST_CLKCTL_MASK		(0x3 << 16)
#define SST_CLKCTL_DCPLCG	(1 << 18)	/* Disable Clock Power Gating */

/*
 * CSR2 Bits
 */
#define SST_CSR2_SDFD_SSP0	(1 << 1)	/* SSP0 Disable */
#define SST_CSR2_SDFD_SSP1	(1 << 2)	/* SSP1 Disable */

/*
 * SSP (Serial Synchronous Port) Registers
 * Offset from SSP base
 */
#define SST_SSP_SSCR0		0x00	/* SSP Control Register 0 */
#define SST_SSP_SSCR1		0x04	/* SSP Control Register 1 */
#define SST_SSP_SSSR		0x08	/* SSP Status Register */
#define SST_SSP_SSITR		0x0C	/* SSP Interrupt Test */
#define SST_SSP_SSDR		0x10	/* SSP Data Register */
#define SST_SSP_SSTO		0x28	/* SSP Timeout */
#define SST_SSP_SSPSP		0x2C	/* SSP Programmable Serial Protocol */
#define SST_SSP_SSTSA		0x30	/* SSP TX Time Slot Active */
#define SST_SSP_SSRSA		0x34	/* SSP RX Time Slot Active */
#define SST_SSP_SSTSS		0x38	/* SSP TX Time Slot Status */
#define SST_SSP_SSACD		0x3C	/* SSP Audio Clock Divider */

/*
 * DMA Registers
 * DW-DMA controller at offset 0x98000 within BAR0
 */
#define SST_DMA_OFFSET		0x98000
#define SST_DMA_SIZE		0x1000

/*
 * Firmware Memory Block Types
 */
#define SST_MEM_IRAM		0x00	/* Instruction RAM */
#define SST_MEM_DRAM		0x01	/* Data RAM */
#define SST_MEM_SRAM		0x02	/* Scratch RAM */
#define SST_MEM_CACHE		0x03	/* Cache */

/*
 * Hardware Constants
 */
#define SST_MIN_MMIO_SIZE	0x1000
#define SST_INVALID_REG_VALUE	0xFFFFFFFF

/*
 * Timeouts
 */
#define SST_IPC_TIMEOUT_MS	5000	/* 5 seconds */
#define SST_BOOT_TIMEOUT_MS	5000	/* 5 seconds */
#define SST_RESET_DELAY_US	1000	/* 1ms */
#define SST_STALL_DELAY_US	100	/* 100us */

#endif /* _SST_REGS_H_ */
