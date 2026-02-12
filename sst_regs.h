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

/*
 * VDRTCTL0 bits - from Linux catpt driver
 * IMPORTANT: SRAM power gating bits control:
 *   - Set bits = ENABLE power gating = power OFF SRAM
 *   - Clear bits = DISABLE power gating = power ON SRAM
 * To enable SRAM, you must CLEAR the xSRAMPGE bits!
 *
 * Register layouts differ between LPT (Haswell) and WPT (Broadwell):
 */

/* WPT (Wildcat Point = Broadwell-U) VDRTCTL0 bit definitions */
#define SST_WPT_VDRTCTL0_D3PGD		(1 << 0)	/* Bit 0: D3 Power Gate Disable */
#define SST_WPT_VDRTCTL0_D3SRAMPGD	(1 << 1)	/* Bit 1: D3 SRAM Power Gate Disable */
#define SST_WPT_VDRTCTL0_ISRAMPGE_MASK	0xFFC		/* Bits 2-11: IRAM Power Gate Enable (10 blocks) */
#define SST_WPT_VDRTCTL0_ISRAMPGE_SHIFT	2
#define SST_WPT_VDRTCTL0_DSRAMPGE_MASK	0xFF000		/* Bits 12-19: DRAM Power Gate Enable (8 blocks) */
#define SST_WPT_VDRTCTL0_DSRAMPGE_SHIFT	12

/* LPT (Lynx Point = Haswell) VDRTCTL0 bit definitions */
#define SST_LPT_VDRTCTL0_APLLSE		(1 << 0)	/* Bit 0: Audio PLL Shutdown Enable */
#define SST_LPT_VDRTCTL0_D3PGD		(1 << 1)	/* Bit 1: D3 Power Gate Disable */
#define SST_LPT_VDRTCTL0_D3SRAMPGD	(1 << 2)	/* Bit 2: D3 SRAM Power Gate Disable */
#define SST_LPT_VDRTCTL0_ISRAMPGE_MASK	0x3C0		/* Bits 6-9: IRAM Power Gate Enable */
#define SST_LPT_VDRTCTL0_ISRAMPGE_SHIFT	6
#define SST_LPT_VDRTCTL0_DSRAMPGE_MASK	0xFF0000	/* Bits 16-23: DRAM Power Gate Enable */
#define SST_LPT_VDRTCTL0_DSRAMPGE_SHIFT	16

/* Legacy aliases (use WPT for Broadwell-U) */
#define SST_VDRTCTL0_DSRAMPGE_MASK	SST_WPT_VDRTCTL0_DSRAMPGE_MASK
#define SST_VDRTCTL0_D3SRAMPGD		SST_WPT_VDRTCTL0_D3SRAMPGD
#define SST_VDRTCTL0_D3PGD		SST_WPT_VDRTCTL0_D3PGD

/* VDRTCTL2 bits (same for both LPT and WPT) */
#define SST_VDRTCTL2_DCLCGE		(1 << 1)	/* Dynamic Clock Gating Enable */
#define SST_VDRTCTL2_DTCGE		(1 << 10)	/* Trunk Clock Gating Enable */
#define SST_VDRTCTL2_CGEALL		0xF7F		/* All Clock Gating Enable mask */
/* APLLSE location differs: LPT=VDRTCTL0 bit0, WPT=VDRTCTL2 bit31 */
#define SST_VDRTCTL2_APLLSE_MASK	(1U << 31)	/* Audio PLL Shutdown Enable (WPT only) */

/*
 * LPSS Private Registers (at BAR0 + 0x800 for Lynxpoint/Broadwell)
 * These control LPSS device power state
 */
#define SST_LPSS_PRIV_OFFSET		0x800		/* LPSS private register offset */
#define SST_LPSS_PRIV_RESETS		0x04		/* LPSS Resets */
#define SST_LPSS_PRIV_RESETS_FUNC	(1 << 0)	/* Function Reset */
#define SST_LPSS_PRIV_RESETS_IDMA	(1 << 2)	/* iDMA Reset */

/*
 * PMCS - PCI Power Management Control/Status
 * Located at PM capability offset + 4
 */
#define SST_PCI_PMCS			0x84		/* PM Control/Status (PM cap at 0x80) */
#define SST_PMCS_PS_MASK		0x03		/* Power State Mask */
#define SST_PMCS_PS_D0			0x00		/* D0 Power State */
#define SST_PMCS_PS_D3			0x03		/* D3 Power State */
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
#define SST_SHIM_LTRC		0xE0	/* Low Trunk Clock */
#define SST_SHIM_HMDC		0xE8	/* Host Memory DMA Control */

/*
 * Additional PCI Extended Config registers (via BAR1)
 * Based on Linux catpt driver registers.h
 */
#define SST_PCI_CS1		0x00	/* PCI Config Space 1 (mirrors VID/DID) */
#define SST_PCI_IMC		0xE4	/* Interrupt Mask Clear */
#define SST_PCI_IMD		0xEC	/* Interrupt Mask Set */
#define SST_PCI_IPCC		0xE0	/* IPC Clear */
#define SST_PCI_IPCD_REG	0xE8	/* IPC Set (note: conflicts with HMDC) */

/*
 * IMC/IMD bits (via BAR1)
 */
#define SST_IMC_IPCDB		(1 << 0)	/* IPC Doorbell */
#define SST_IMC_IPCCD		(1 << 1)	/* IPC Completion */

/*
 * CS1 bits (via BAR1)
 */
#define SST_CS1_SBCS_MASK	(0x3 << 2)	/* SSP/SRAM Bank Clock Select */
#define SST_CS1_SBCS_24MHZ	(0x2 << 2)	/* 24MHz clock */

/*
 * HMDC bits - Host Memory DMA Control
 */
#define SST_HMDC_HDDA_E0	(1 << 0)	/* Host DMA DMA Access Enable 0 */
#define SST_HMDC_HDDA_E1	(1 << 1)	/* Host DMA DMA Access Enable 1 */
#define SST_HMDC_HDDA_E2	(1 << 2)	/* Host DMA DMA Access Enable 2 */
#define SST_HMDC_HDDA_E3	(1 << 3)	/* Host DMA DMA Access Enable 3 */
#define SST_HMDC_HDDA_ALL	0x0F		/* All DMA access enabled */

/*
 * LTRC bits - Low Trunk Clock
 */
#define SST_LTRC_VAL		0x3003		/* Default LTRC value */

/*
 * Default register values from Linux catpt driver
 */
#define SST_IMD_DEFAULT		0x7FFF0003	/* Default IMD value */
#define SST_SSC0_DEFAULT	0x00000000	/* Default SSP Control 0 */
#define SST_SSC1_DEFAULT	0x00000000	/* Default SSP Control 1 */
#define SST_SSCFS_DEFAULT	0x00200000	/* Default SSCFS */
#define SST_SSCFS_38_4KHZ	0x00200000	/* 38.4 kHz reference */
#define SST_CLKCTL_DEFAULT	0x02000000	/* Default CLKCTL (SMOS=2) */

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

/*
 * PCH RCBA (Root Complex Base Address) Register Definitions
 * RCBA base is read from LPC bridge (00:1F.0) config offset 0xF0
 */
#define PCH_LPC_RCBA_REG	0xF0		/* LPC RCBA register */
#define PCH_RCBA_ENABLE		(1 << 0)	/* RCBA Enable bit */
#define PCH_RCBA_MASK		0xFFFFC000	/* RCBA address mask */
#define PCH_RCBA_SIZE		0x4000		/* 16KB RCBA region */

/* Function Disable (FD) register at RCBA + 0x3418
 * Bit positions verified from DSDT OperationRegion RCRB field definitions:
 *   Offset (0x3418), skip 1 bit, ADSD 1, SATD 1, SMBD 1, HDAD 1, ...
 */
#define PCH_RCBA_FD		0x3418		/* Function Disable */
#define PCH_FD_ADSD		(1 << 1)	/* Audio DSP Disable (bit 1) */
#define PCH_FD_SATD		(1 << 2)	/* SATA Disable (bit 2) */
#define PCH_FD_SMBD		(1 << 3)	/* SMBus Disable (bit 3) */
#define PCH_FD_HDAD		(1 << 4)	/* HD Audio Disable (bit 4) */

/* Function Disable 2 (FD2) register at RCBA + 0x3428 */
#define PCH_RCBA_FD2		0x3428

/*
 * IOBP (I/O Bridge Port) Sideband Interface
 * Used to access internal PCH device configuration registers
 * Source: coreboot src/southbridge/intel/lynxpoint/iobp.c
 */
#define PCH_IOBPIRI		0x2330		/* IOBP Index Register (target address) */
#define PCH_IOBPD		0x2334		/* IOBP Data Register */
#define PCH_IOBPS		0x2338		/* IOBP Status Register */
#define PCH_IOBPU		0x233A		/* IOBP Undocumented/Magic Register */

/* IOBP Status Register bits */
#define PCH_IOBPS_READY		(1 << 0)	/* Transaction ready/busy */
#define PCH_IOBPS_TX_MASK	(3 << 1)	/* Transaction status mask */
#define PCH_IOBPS_READ		0x0600		/* Read opcode */
#define PCH_IOBPS_WRITE		0x0700		/* Write opcode */
#define PCH_IOBPU_MAGIC		0xF000		/* Magic value for writes */

/*
 * ADSP IOBP Register Addresses
 * Source: coreboot src/soc/intel/broadwell/include/soc/adsp.h
 */
#define ADSP_IOBP_PCICFGCTL	0xd7000500	/* PCI Configuration Control */
#define ADSP_IOBP_PMCTL	0xd70001e0	/* Power Management Control */
#define ADSP_IOBP_VDLDAT1	0xd7000624	/* Voltage/Data Line Config 1 */
#define ADSP_IOBP_VDLDAT2	0xd7000628	/* Voltage/Data Line Config 2 */

/* PCICFGCTL bits */
#define ADSP_PCICFGCTL_PCICD	(1 << 0)	/* PCI Config Disable */
#define ADSP_PCICFGCTL_ACPIIE	(1 << 1)	/* ACPI Interrupt Enable */
#define ADSP_PCICFGCTL_SPCBAD	(1 << 7)	/* Sideband PCH BAR Disable */

/* ADSP IOBP default values (from coreboot) */
#define ADSP_VDLDAT1_VALUE	0x00040100
#define ADSP_PMCTL_VALUE	0x3f

/* PSF Snoop enable at RCBA + 0x3350 */
#define PCH_RCBA_PSF_SNOOP	0x3350
#define PCH_PSF_SNOOP_ADSP	(1 << 10)	/* ADSP snoop to System Agent */

/* LPC bridge PCI location */
#define PCH_LPC_BUS		0
#define PCH_LPC_DEV		0x1F
#define PCH_LPC_FUNC		0

/* GNVS (Global NVS Area) for reading BIOS configuration
 * Address from custom DSDT: OperationRegion (GNVS, SystemMemory, 0xDB7EF000)
 */
#define PCH_GNVS_BASE		0xDB7EF000
#define PCH_GNVS_SIZE		0x035B

/* HDA controller PCI location */
#define PCH_HDA_BUS		0
#define PCH_HDA_DEV		0x1B
#define PCH_HDA_FUNC		0
#define PCH_HDA_VID		0x8086
#define PCH_HDA_DID_WPT	0x9CA0		/* Wildcat Point-LP HD Audio */

/*
 * DesignWare I2C Controller Register Definitions
 * Used for I2C1 at 0xFE105000 (codec communication)
 */
#define DW_IC_CON		0x00	/* I2C Control */
#define DW_IC_TAR		0x04	/* Target Address */
#define DW_IC_SAR		0x08	/* Slave Address */
#define DW_IC_HS_MADDR		0x0C	/* HS Master Mode Code Address */
#define DW_IC_DATA_CMD		0x10	/* Data Buffer and Command */
#define DW_IC_SS_SCL_HCNT	0x14	/* Standard Speed SCL High Count */
#define DW_IC_SS_SCL_LCNT	0x18	/* Standard Speed SCL Low Count */
#define DW_IC_FS_SCL_HCNT	0x1C	/* Fast Speed SCL High Count */
#define DW_IC_FS_SCL_LCNT	0x20	/* Fast Speed SCL Low Count */
#define DW_IC_INTR_STAT		0x2C	/* Interrupt Status */
#define DW_IC_INTR_MASK		0x30	/* Interrupt Mask */
#define DW_IC_RAW_INTR_STAT	0x34	/* Raw Interrupt Status */
#define DW_IC_RX_TL		0x38	/* Receive FIFO Threshold */
#define DW_IC_TX_TL		0x3C	/* Transmit FIFO Threshold */
#define DW_IC_CLR_INTR		0x40	/* Clear Combined Interrupt */
#define DW_IC_CLR_TX_ABRT	0x54	/* Clear TX_ABRT Interrupt */
#define DW_IC_ENABLE		0x6C	/* I2C Enable */
#define DW_IC_STATUS		0x70	/* I2C Status */
#define DW_IC_TXFLR		0x74	/* Transmit FIFO Level */
#define DW_IC_RXFLR		0x78	/* Receive FIFO Level */
#define DW_IC_TX_ABRT_SOURCE	0x80	/* TX Abort Source */
#define DW_IC_ENABLE_STATUS	0x9C	/* I2C Enable Status */
#define DW_IC_COMP_PARAM_1	0xF4	/* Component Parameters */
#define DW_IC_COMP_TYPE		0xFC	/* Component Type */

/* IC_CON bits */
#define DW_IC_CON_MASTER	(1 << 0)	/* Master mode */
#define DW_IC_CON_SPEED_SS	(1 << 1)	/* Standard speed (100kHz) */
#define DW_IC_CON_SPEED_FS	(2 << 1)	/* Fast speed (400kHz) */
#define DW_IC_CON_SPEED_MASK	(3 << 1)
#define DW_IC_CON_SLAVE_DIS	(1 << 6)	/* Slave disable */
#define DW_IC_CON_RESTART_EN	(1 << 5)	/* Restart enable */

/* IC_DATA_CMD bits */
#define DW_IC_DATA_CMD_READ	(1 << 8)	/* Read command */
#define DW_IC_DATA_CMD_STOP	(1 << 9)	/* STOP after this byte */
#define DW_IC_DATA_CMD_RESTART	(1 << 10)	/* RESTART before this byte */

/* IC_STATUS bits */
#define DW_IC_STATUS_ACTIVITY	(1 << 0)	/* I2C activity */
#define DW_IC_STATUS_TFNF	(1 << 1)	/* TX FIFO not full */
#define DW_IC_STATUS_TFE	(1 << 2)	/* TX FIFO empty */
#define DW_IC_STATUS_RFNE	(1 << 3)	/* RX FIFO not empty */

/* IC_COMP_TYPE expected value */
#define DW_IC_COMP_TYPE_VALUE	0x44570140	/* DesignWare signature */

/* I2C0 Physical Address - RT286 codec confirmed on iicbus6/ig4iic0 */
#define SST_I2C0_BASE		0xFE103000
#define SST_I2C0_SIZE		0x1000
#define SST_I2C0_CODEC_ADDR	0x1C		/* RT286 I2C address on I2C0 */
#define SST_I2C0_PRIV_BASE	0xFE104000	/* I2C0 LPSS private config */

/* I2C1 Physical Address (touchpad DLL0665 at 0x2c, NOT codec) */
#define SST_I2C1_BASE		0xFE105000
#define SST_I2C1_SIZE		0x1000
#define SST_I2C1_CODEC_ADDR	0x2C

/* RT286 Codec I2C Verb Definitions */
#define RT286_VENDOR_ID		0x10EC0286
#define RT286_GET_PARAM		0xF0000		/* Get Parameter verb base */
#define RT286_SET_POWER		0x70500		/* Set Power State verb */
#define AC_NODE_ROOT		0x00		/* Root node */
#define AC_PAR_VENDOR_ID	0x00		/* Vendor ID parameter */

#endif /* _SST_REGS_H_ */
