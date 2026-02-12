#!/usr/bin/env python3
"""
Parse RWEverything memory dump files and analyze PCH/LPSS register contents.

RW format:
  Line 1: "Type:Memory   Address XXXXXXXXXXXXXXXX"
  Line 2: "Width:01"
  Lines 3+: "XX=YY XX=YY ..." where XX is hex offset, YY is hex byte value
"""

import sys
import os
from collections import OrderedDict

def parse_rw_file(filepath):
    """Parse an RWEverything .rw memory dump file. Returns (base_addr, byte_dict)."""
    data = OrderedDict()
    base_addr = 0
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        line = line.strip()
        if line.startswith("Type:"):
            # Extract base address from "Type:Memory   Address XXXXXXXXXXXXXXXX"
            # or "Type:PCI   Address XXXXXXXX"
            parts = line.split("Address")
            if len(parts) == 2:
                base_addr = int(parts[1].strip(), 16)
        elif line.startswith("Width:"):
            continue
        elif '=' in line:
            pairs = line.split()
            for pair in pairs:
                if '=' in pair:
                    offset_str, val_str = pair.split('=')
                    offset = int(offset_str, 16)
                    val = int(val_str, 16)
                    data[offset] = val
    
    return base_addr, data

def read_dword(data, offset):
    """Read a little-endian 32-bit DWORD from byte dictionary."""
    b0 = data.get(offset, 0xFF)
    b1 = data.get(offset + 1, 0xFF)
    b2 = data.get(offset + 2, 0xFF)
    b3 = data.get(offset + 3, 0xFF)
    return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)

def read_word(data, offset):
    """Read a little-endian 16-bit WORD from byte dictionary."""
    b0 = data.get(offset, 0xFF)
    b1 = data.get(offset + 1, 0xFF)
    return b0 | (b1 << 8)

def is_all_ff(data):
    return all(v == 0xFF for v in data.values())

def is_all_zero(data):
    return all(v == 0x00 for v in data.values())

def print_hex_dump(data, base_addr):
    if not data:
        return
    max_off = max(data.keys())
    for row_start in range(0, max_off + 1, 16):
        addr = base_addr + row_start
        hex_str = ""
        for i in range(16):
            val = data.get(row_start + i, None)
            if val is not None:
                hex_str += "%02X " % val
            else:
                hex_str += "?? "
            if i == 7:
                hex_str += " "
        ascii_str = ""
        for i in range(16):
            val = data.get(row_start + i, 0)
            if 0x20 <= val <= 0x7E:
                ascii_str += chr(val)
            else:
                ascii_str += "."
        print("  %08X: %s |%s|" % (addr, hex_str, ascii_str))

def analyze_rcba(base_addr, data):
    print("=" * 80)
    print("PCH RCBA Analysis - Base Address: 0x%08X" % base_addr)
    print("=" * 80)
    
    if is_all_ff(data):
        print("  *** ALL BYTES ARE 0xFF - Region is UNMAPPED/INACCESSIBLE ***")
        return
    
    non_zero = {off: val for off, val in data.items() if val != 0x00}
    non_ff = {off: val for off, val in data.items() if val != 0xFF}
    
    print("  Total bytes in dump: %d" % len(data))
    print("  Non-zero bytes: %d" % len(non_zero))
    print("  Non-0xFF bytes: %d" % len(non_ff))
    
    max_offset = max(data.keys()) if data else 0
    print("  Dump covers offsets 0x00 to 0x%02X (%d bytes)" % (max_offset, max_offset + 1))
    print("  NOTE: Full RCBA is 16KB (0x4000). This dump is only %d bytes." % (max_offset + 1))
    
    # Decode known low-offset RCBA registers
    print("\n--- RCBA Low-Offset Register Decode ---")
    
    # RCBA+0x0000: VCH - Virtual Channel capability header (on some PCH)
    dw0 = read_dword(data, 0x00)
    print("  [0x0000] = 0x%08X" % dw0)
    
    # RCBA+0x0050: GCS area or HPTC on Broadwell PCH-LP
    if 0x50 in data:
        dw50 = read_dword(data, 0x50)
        dw54 = read_dword(data, 0x54)
        if dw50 != 0 or dw54 != 0:
            print("\n  [0x0050] = 0x%08X  (Possible HPTC / Timer Config)" % dw50)
            print("  [0x0054] = 0x%08X" % dw54)
            print("    Bytes: %02X %02X %02X %02X %02X %02X %02X %02X" % (
                data.get(0x50,0), data.get(0x51,0), data.get(0x52,0), data.get(0x53,0),
                data.get(0x54,0), data.get(0x55,0), data.get(0x56,0), data.get(0x57,0)))
            # HPTC interpretation: bits [1:0] = Address Select for HPET
            # 00 = FED00000h, 01 = FED01000h, 10 = FED02000h, 11 = FED03000h
            # bit 7 = Address Enable
            hptc_ae = (dw50 >> 7) & 1
            hptc_as = dw50 & 0x3
            hpet_addrs = {0: "0xFED00000", 1: "0xFED01000", 2: "0xFED02000", 3: "0xFED03000"}
            print("    If this is HPTC:")
            print("      Address Enable: %d" % hptc_ae)
            print("      Address Select: %d -> %s" % (hptc_as, hpet_addrs.get(hptc_as, "?")))

    if 0x84 in data and data[0x84] != 0:
        dw84 = read_dword(data, 0x84)
        print("\n  [0x0084] = 0x%08X" % dw84)
    
    # Print all non-zero bytes
    print("\n--- All Non-Zero Byte Values in RCBA Dump ---")
    for off in sorted(non_zero.keys()):
        val = non_zero[off]
        abs_addr = base_addr + off
        print("  [0x%04X] (phys 0x%08X) = 0x%02X (%d)" % (off, abs_addr, val, val))
    
    # Print as DWORDs
    print("\n--- Non-Zero DWORD Values ---")
    printed = set()
    for off in sorted(non_zero.keys()):
        dw_base = off & ~3
        if dw_base not in printed:
            dw = read_dword(data, dw_base)
            if dw != 0:
                abs_addr = base_addr + dw_base
                print("  [0x%04X] (phys 0x%08X) = 0x%08X" % (dw_base, abs_addr, dw))
                printed.add(dw_base)
    
    print("\n--- Full Hex Dump ---")
    print_hex_dump(data, base_addr)
    
    print("\n--- Missing LPSS Registers (outside this 256-byte dump) ---")
    print("  The following RCBA offsets are critical for LPSS but NOT in this dump:")
    print("    RCBA+0x2030 (phys 0x%08X) - LPSS IOSF SB port access" % (base_addr + 0x2030))
    print("    RCBA+0x3400 (phys 0x%08X) - FD (Function Disable)" % (base_addr + 0x3400))
    print("    RCBA+0x3418 (phys 0x%08X) - FD2 (Function Disable 2)" % (base_addr + 0x3418))
    print("    RCBA+0x3420 (phys 0x%08X) - Possible LPSS/SCC function disable" % (base_addr + 0x3420))
    print("    RCBA+0x3F02 (phys 0x%08X) - BUC (Backed Up Control)" % (base_addr + 0x3F02))

def analyze_lpss_private(base_addr, data):
    print("=" * 80)
    print("LPSS Private Region Analysis - Base Address: 0x%08X" % base_addr)
    print("=" * 80)
    
    if is_all_ff(data):
        print("  *** ALL BYTES ARE 0xFF - Region is UNMAPPED/INACCESSIBLE ***")
        print()
        print("  DIAGNOSIS: The LPSS private MMIO at 0x%08X returns all 0xFF." % base_addr)
        print("  This means LPSS I/O decode is NOT enabled for this address range.")
        print("  Possible causes:")
        print("    1. LPSS devices disabled in PCH Function Disable register")
        print("    2. LPSS ACPI-mode BAR not configured / Memory Space not enabled")
        print("    3. BIOS did not enable LPSS ACPI mode / private register space")
        print("    4. The region base address differs on this specific platform")
        return
    
    non_ff = {off: val for off, val in data.items() if val != 0xFF}
    print("  Non-0xFF bytes: %d out of %d" % (len(non_ff), len(data)))
    if non_ff:
        print("\n  --- Non-0xFF values ---")
        for off in sorted(non_ff.keys()):
            print("  [0x%04X] = 0x%02X" % (off, non_ff[off]))

def analyze_i2c(base_addr, data, name="I2C"):
    print("=" * 80)
    print("%s Controller Analysis - Base Address: 0x%08X" % (name, base_addr))
    print("=" * 80)
    
    if is_all_ff(data):
        print("  *** ALL BYTES ARE 0xFF - Region is UNMAPPED/INACCESSIBLE ***")
        print("  DIAGNOSIS: %s register space returns all 0xFF." % name)
        print("  Device is either disabled, in D3 power state, or BAR is not mapped.")
        return
    
    non_zero = {off: val for off, val in data.items() if val != 0x00}
    non_ff = {off: val for off, val in data.items() if val != 0xFF}
    
    print("  Non-zero bytes: %d out of %d" % (len(non_zero), len(data)))
    print("  Non-0xFF bytes: %d out of %d" % (len(non_ff), len(data)))
    
    dw_i2c_regs = {
        0x00: ("IC_CON",           "I2C Control Register"),
        0x04: ("IC_TAR",           "I2C Target Address"),
        0x08: ("IC_SAR",           "I2C Slave Address"),
        0x0C: ("IC_HS_MADDR",      "I2C HS Master Code Address"),
        0x10: ("IC_DATA_CMD",      "I2C Rx/Tx Data Buffer and Command"),
        0x14: ("IC_SS_SCL_HCNT",   "Std Speed SCL High Count"),
        0x18: ("IC_SS_SCL_LCNT",   "Std Speed SCL Low Count"),
        0x1C: ("IC_FS_SCL_HCNT",   "Fast Speed SCL High Count"),
        0x20: ("IC_FS_SCL_LCNT",   "Fast Speed SCL Low Count"),
        0x24: ("IC_HS_SCL_HCNT",   "High Speed SCL High Count"),
        0x28: ("IC_HS_SCL_LCNT",   "High Speed SCL Low Count"),
        0x2C: ("IC_INTR_STAT",     "Interrupt Status"),
        0x30: ("IC_INTR_MASK",     "Interrupt Mask"),
        0x34: ("IC_RAW_INTR_STAT", "Raw Interrupt Status"),
        0x38: ("IC_RX_TL",         "Receive FIFO Threshold"),
        0x3C: ("IC_TX_TL",         "Transmit FIFO Threshold"),
        0x40: ("IC_CLR_INTR",      "Clear Combined Interrupt"),
        0x44: ("IC_CLR_RX_UNDER",  "Clear RX_UNDER"),
        0x48: ("IC_CLR_RX_OVER",   "Clear RX_OVER"),
        0x4C: ("IC_CLR_TX_OVER",   "Clear TX_OVER"),
        0x50: ("IC_CLR_RD_REQ",    "Clear RD_REQ"),
        0x54: ("IC_CLR_TX_ABRT",   "Clear TX_ABRT"),
        0x58: ("IC_CLR_RX_DONE",   "Clear RX_DONE"),
        0x5C: ("IC_CLR_ACTIVITY",  "Clear ACTIVITY"),
        0x60: ("IC_CLR_STOP_DET",  "Clear STOP_DET"),
        0x64: ("IC_CLR_START_DET", "Clear START_DET"),
        0x68: ("IC_CLR_GEN_CALL",  "Clear GEN_CALL"),
        0x6C: ("IC_ENABLE",        "I2C Enable"),
        0x70: ("IC_STATUS",        "I2C Status"),
        0x74: ("IC_TXFLR",         "TX FIFO Level"),
        0x78: ("IC_RXFLR",         "RX FIFO Level"),
        0x7C: ("IC_SDA_HOLD",      "SDA Hold Time"),
        0x80: ("IC_TX_ABRT_SRC",   "TX Abort Source"),
        0x84: ("IC_SLV_DATA_NACK", "Slave Data NACK"),
        0x88: ("IC_DMA_CR",        "DMA Control"),
        0x8C: ("IC_DMA_TDLR",      "DMA TX Data Level"),
        0x90: ("IC_DMA_RDLR",      "DMA RX Data Level"),
        0x94: ("IC_SDA_SETUP",     "SDA Setup Time"),
        0x98: ("IC_ACK_GEN_CALL",  "ACK General Call"),
        0x9C: ("IC_ENABLE_STATUS", "Enable Status"),
        0xA0: ("IC_FS_SPKLEN",     "FS Spike Suppression"),
        0xA4: ("IC_HS_SPKLEN",     "HS Spike Suppression"),
        0xA8: ("IC_CLR_RESTART_DET","Clear RESTART_DET"),
        0xF4: ("IC_COMP_PARAM_1",  "Component Parameter 1"),
        0xF8: ("IC_COMP_VERSION",  "Component Version"),
        0xFC: ("IC_COMP_TYPE",     "Component Type"),
    }
    
    print("\n--- DesignWare I2C Register Decode ---")
    for off, (regname, desc) in sorted(dw_i2c_regs.items()):
        if off + 3 <= max(data.keys()):
            dw = read_dword(data, off)
            marker = ""
            if dw == 0xFFFFFFFF:
                marker = " [UNMAPPED]"
            elif dw == 0:
                continue  # skip zero regs for brevity
            print("  [0x%02X] %-24s = 0x%08X  %s%s" % (off, regname, dw, desc, marker))
    
    # IC_CON detail
    if 0x00 in data:
        ic_con = read_dword(data, 0x00)
        if ic_con != 0 and ic_con != 0xFFFFFFFF:
            print("\n  --- IC_CON (0x%08X) Bit Decode ---" % ic_con)
            print("    Bit 0  MASTER_MODE:      %d  (%s)" % (ic_con & 1, 'Master' if ic_con & 1 else 'Slave'))
            speed = (ic_con >> 1) & 0x3
            speed_map = {0: "Std <=100kbps", 1: "Std <=100kbps", 2: "Fast <=400kbps", 3: "High <=3.4Mbps"}
            print("    Bit 2:1 SPEED:            %d  (%s)" % (speed, speed_map.get(speed, '?')))
            print("    Bit 3  10BIT_ADDR_SLAVE:  %d" % ((ic_con >> 3) & 1))
            print("    Bit 4  10BIT_ADDR_MASTER: %d" % ((ic_con >> 4) & 1))
            print("    Bit 5  RESTART_EN:        %d" % ((ic_con >> 5) & 1))
            print("    Bit 6  SLAVE_DISABLE:     %d" % ((ic_con >> 6) & 1))
    
    # IC_TAR detail
    if 0x04 in data:
        ic_tar = read_dword(data, 0x04)
        if ic_tar != 0 and ic_tar != 0xFFFFFFFF:
            addr_7bit = ic_tar & 0x3FF
            print("\n  --- IC_TAR (0x%08X) Decode ---" % ic_tar)
            print("    Target address (10-bit field): 0x%03X" % addr_7bit)
            print("    7-bit address interpretation:  0x%02X (%d)" % (addr_7bit & 0x7F, addr_7bit & 0x7F))
            print("    Bit 10 SPECIAL:    %d" % ((ic_tar >> 10) & 1))
            print("    Bit 11 GC_OR_START: %d" % ((ic_tar >> 11) & 1))
            # Known I2C addresses for audio codecs
            known_addrs = {
                0x10: "RT5677 (Realtek DSP codec)",
                0x1A: "WM8994/WM8998/CS42L42 (audio codec)",
                0x1B: "RT5645/RT5651/RT286 (Realtek codec)",
                0x1C: "RT286/RT298 (Realtek HD Audio codec)",
                0x1D: "MAX98090/MAX98357 (Maxim codec)",
                0x34: "RT5677 (alt addr)",
                0x2C: "RT286/RT298 (alt addr)",
                0x38: "NXP TFA9890 (smart amplifier)",
                0x40: "SI4713 (FM transmitter)",
                0x50: "EEPROM",
                0x55: "WM8731/WM8750 (Wolfson codec)",
            }
            a7 = addr_7bit & 0x7F
            if a7 in known_addrs:
                print("    *** Known device at 0x%02X: %s ***" % (a7, known_addrs[a7]))
    
    # IC_STATUS
    if 0x70 in data:
        ic_status = read_dword(data, 0x70)
        if ic_status != 0xFFFFFFFF:
            print("\n  --- IC_STATUS (0x%08X) Decode ---" % ic_status)
            print("    Bit 0 ACTIVITY:      %d  (%s)" % (ic_status & 1, 'Active' if ic_status & 1 else 'Idle'))
            print("    Bit 1 TFNF:          %d  (%s)" % ((ic_status >> 1) & 1, 'TX Not Full' if (ic_status >> 1) & 1 else 'TX Full'))
            print("    Bit 2 TFE:           %d  (%s)" % ((ic_status >> 2) & 1, 'TX Empty' if (ic_status >> 2) & 1 else 'TX Not Empty'))
            print("    Bit 3 RFNE:          %d  (%s)" % ((ic_status >> 3) & 1, 'RX Not Empty' if (ic_status >> 3) & 1 else 'RX Empty'))
            print("    Bit 4 RFF:           %d  (%s)" % ((ic_status >> 4) & 1, 'RX Full' if (ic_status >> 4) & 1 else 'RX Not Full'))
            print("    Bit 5 MST_ACTIVITY:  %d  (%s)" % ((ic_status >> 5) & 1, 'Master Active' if (ic_status >> 5) & 1 else 'Master Idle'))
            print("    Bit 6 SLV_ACTIVITY:  %d  (%s)" % ((ic_status >> 6) & 1, 'Slave Active' if (ic_status >> 6) & 1 else 'Slave Idle'))
    
    # IC_ENABLE
    if 0x6C in data:
        ic_en = read_dword(data, 0x6C)
        if ic_en != 0xFFFFFFFF:
            print("\n  --- IC_ENABLE (0x%08X) Decode ---" % ic_en)
            print("    Bit 0 ENABLE: %d  (%s)" % (ic_en & 1, 'Enabled' if ic_en & 1 else 'Disabled'))
    
    # IC_COMP_VERSION / IC_COMP_TYPE
    if 0xF8 in data:
        ver = read_dword(data, 0xF8)
        if ver != 0 and ver != 0xFFFFFFFF:
            print("\n  --- IC_COMP_VERSION = 0x%08X ---" % ver)
            print("    DesignWare version: %d.%02d" % ((ver >> 24) & 0xFF, (ver >> 16) & 0xFF))
    if 0xFC in data:
        ct = read_dword(data, 0xFC)
        if ct != 0 and ct != 0xFFFFFFFF:
            print("  --- IC_COMP_TYPE = 0x%08X ---" % ct)
            if ct == 0x44570140:
                print("    Confirmed: DesignWare I2C component (magic 0x44570140)")
            else:
                print("    Expected 0x44570140 for DesignWare I2C, got 0x%08X" % ct)
    
    # SCL timing
    if 0x14 in data:
        ss_hcnt = read_dword(data, 0x14)
        ss_lcnt = read_dword(data, 0x18)
        fs_hcnt = read_dword(data, 0x1C)
        fs_lcnt = read_dword(data, 0x20)
        if ss_hcnt != 0xFFFFFFFF and ss_hcnt != 0:
            print("\n  --- SCL Timing ---")
            print("    SS_SCL_HCNT = %d (0x%X)" % (ss_hcnt, ss_hcnt))
            print("    SS_SCL_LCNT = %d (0x%X)" % (ss_lcnt, ss_lcnt))
            print("    FS_SCL_HCNT = %d (0x%X)" % (fs_hcnt, fs_hcnt))
            print("    FS_SCL_LCNT = %d (0x%X)" % (fs_lcnt, fs_lcnt))
            for clk_mhz in [100, 120, 133]:
                if ss_hcnt > 0 and ss_lcnt > 0:
                    ss_freq = clk_mhz * 1000000.0 / (ss_hcnt + ss_lcnt)
                    print("    @%dMHz input: SS = %.1f kHz" % (clk_mhz, ss_freq / 1000))
                if fs_hcnt > 0 and fs_lcnt > 0:
                    fs_freq = clk_mhz * 1000000.0 / (fs_hcnt + fs_lcnt)
                    print("    @%dMHz input: FS = %.1f kHz" % (clk_mhz, fs_freq / 1000))
    
    # SDA hold
    if 0x7C in data:
        sda_hold = read_dword(data, 0x7C)
        if sda_hold != 0 and sda_hold != 0xFFFFFFFF:
            tx_hold = sda_hold & 0xFFFF
            rx_hold = (sda_hold >> 16) & 0xFF
            print("\n  --- SDA Hold ---")
            print("    TX Hold: %d clocks, RX Hold: %d clocks" % (tx_hold, rx_hold))
    
    # IC_COMP_PARAM_1
    if 0xF4 in data:
        param1 = read_dword(data, 0xF4)
        if param1 != 0 and param1 != 0xFFFFFFFF:
            print("\n  --- IC_COMP_PARAM_1 (0x%08X) ---" % param1)
            print("    APB data width:    %d" % ((param1 >> 0) & 0x3))
            print("    Max speed mode:    %d (1=Std, 2=Fast, 3=High)" % ((param1 >> 2) & 0x3))
            print("    HC count values:   %d" % ((param1 >> 4) & 1))
            print("    INTR_IO:           %d" % ((param1 >> 5) & 1))
            print("    HAS_DMA:           %d" % ((param1 >> 6) & 1))
            print("    ENCODED_PARAMS:    %d" % ((param1 >> 7) & 1))
            rx_depth = ((param1 >> 8) & 0xFF) + 1
            tx_depth = ((param1 >> 16) & 0xFF) + 1
            print("    RX FIFO depth:     %d" % rx_depth)
            print("    TX FIFO depth:     %d" % tx_depth)
    
    # Full hex dump
    print("\n--- Full Hex Dump ---")
    print_hex_dump(data, base_addr)

def analyze_pci_config_space(base_addr, data):
    print("=" * 80)
    print("PCI Config Space Analysis - Base Address: 0x%08X" % base_addr)
    print("=" * 80)
    
    if is_all_ff(data):
        print("  *** ALL BYTES ARE 0xFF - No device present ***")
        return
    
    vid = read_word(data, 0x00)
    did = read_word(data, 0x02)
    
    print("  Vendor ID: 0x%04X" % vid, end="")
    if vid == 0x8086:
        print(" (Intel Corporation)")
    else:
        print()
    
    broadwell_ids = {
        0x9C60: "LPSS DMA (Lynx Point-LP)",
        0x9C61: "LPSS I2C0 (Lynx Point-LP)",
        0x9C62: "LPSS I2C1 (Lynx Point-LP)",
        0x9C63: "LPSS SPI0 (Lynx Point-LP)",
        0x9C64: "LPSS SPI1 (Lynx Point-LP)",
        0x9C65: "LPSS UART0 (Lynx Point-LP)",
        0x9C66: "LPSS UART1 (Lynx Point-LP)",
        0x9CA0: "LPSS DMA (Wildcat Point-LP)",
        0x9CA1: "LPSS I2C0 (Wildcat Point-LP)",
        0x9CA2: "LPSS I2C1 (Wildcat Point-LP)",
        0x9CA5: "LPSS UART0 (Wildcat Point-LP)",
        0x9CA6: "LPSS UART1 (Wildcat Point-LP)",
        0x9CB6: "SST Audio DSP (Wildcat Point-LP)",
        0x9CB0: "Audio DSP (Wildcat Point-LP variant 0)",
        0x9CB1: "Audio DSP (Wildcat Point-LP variant 1)",
        0x9CB2: "Audio DSP (Wildcat Point-LP variant 2)",
        0x9CB3: "Audio DSP (Wildcat Point-LP variant 3)",
        0x9CB4: "Audio DSP (Wildcat Point-LP variant 4)",
        0x9CB5: "Audio DSP (Wildcat Point-LP variant 5)",
        0x9C36: "SMBus (Lynx Point-LP)",
        0x9CA3: "LPSS SPI0 (Wildcat Point-LP)",
        0x9CA4: "LPSS SPI1 (Wildcat Point-LP)",
    }
    
    print("  Device ID: 0x%04X" % did, end="")
    if did in broadwell_ids:
        print(" (%s)" % broadwell_ids[did])
    else:
        print()
    
    cmd = read_word(data, 0x04)
    sts = read_word(data, 0x06)
    print("  Command:   0x%04X" % cmd)
    print("    Bit 0 IO Space Enable:     %d" % (cmd & 1))
    print("    Bit 1 Memory Space Enable: %d" % ((cmd >> 1) & 1))
    print("    Bit 2 Bus Master Enable:   %d" % ((cmd >> 2) & 1))
    print("    Bit 10 INTx Disable:       %d" % ((cmd >> 10) & 1))
    print("  Status:    0x%04X" % sts)
    
    rev = data.get(0x08, 0)
    prog_if = data.get(0x09, 0)
    subclass = data.get(0x0A, 0)
    baseclass = data.get(0x0B, 0)
    print("  Revision:  0x%02X" % rev)
    print("  Class:     %02X%02X%02Xh" % (baseclass, subclass, prog_if), end="")
    class_names = {
        (0x04, 0x01): "Multimedia - Audio",
        (0x04, 0x03): "Multimedia - HDA",
        (0x04, 0x80): "Multimedia - Other",
        (0x0C, 0x80): "Serial Bus - Other",
        (0x11, 0x80): "Signal Processing - Other",
    }
    ck = (baseclass, subclass)
    if ck in class_names:
        print(" (%s)" % class_names[ck])
    else:
        print()
    
    # Header type
    htype = data.get(0x0E, 0)
    print("  Header Type: 0x%02X (%s)" % (htype, "Type 0 (Endpoint)" if (htype & 0x7F) == 0 else "Type %d" % (htype & 0x7F)))
    
    # BARs
    for bar_idx in range(6):
        bar_off = 0x10 + bar_idx * 4
        if bar_off + 3 <= max(data.keys()):
            bar = read_dword(data, bar_off)
            if bar != 0 and bar != 0xFFFFFFFF:
                if bar & 1:
                    bar_addr = bar & 0xFFFFFFFC
                    print("  BAR%d:     0x%08X (I/O, addr=0x%08X)" % (bar_idx, bar, bar_addr))
                else:
                    bar_addr = bar & 0xFFFFFFF0
                    bar_64 = (bar >> 1) & 0x3
                    prefetch = (bar >> 3) & 1
                    print("  BAR%d:     0x%08X (Memory, addr=0x%08X, %s, %s)" % (
                        bar_idx, bar, bar_addr,
                        '64-bit' if bar_64 == 2 else '32-bit',
                        'Prefetch' if prefetch else 'Non-Prefetch'))
    
    # Subsystem
    sub_vid = read_word(data, 0x2C)
    sub_did = read_word(data, 0x2E)
    if sub_vid != 0 and sub_vid != 0xFFFF:
        print("  SubVID:    0x%04X" % sub_vid)
        print("  SubDID:    0x%04X" % sub_did)
    
    cap_ptr = data.get(0x34, 0)
    if cap_ptr != 0 and cap_ptr != 0xFF:
        print("  Cap Ptr:   0x%02X" % cap_ptr)
        # Walk capability list
        ptr = cap_ptr
        visited = set()
        while ptr != 0 and ptr != 0xFF and ptr not in visited and ptr + 1 <= max(data.keys()):
            visited.add(ptr)
            cap_id = data.get(ptr, 0xFF)
            next_ptr = data.get(ptr + 1, 0)
            cap_names = {
                0x01: "Power Management",
                0x05: "MSI",
                0x10: "PCIe",
                0x11: "MSI-X",
                0x09: "Vendor Specific",
            }
            cname = cap_names.get(cap_id, "Unknown (0x%02X)" % cap_id)
            print("    Cap @ 0x%02X: ID=0x%02X (%s), Next=0x%02X" % (ptr, cap_id, cname, next_ptr))
            
            # Power Management decode
            if cap_id == 0x01 and ptr + 5 <= max(data.keys()):
                pmcsr = read_word(data, ptr + 4)
                ps = pmcsr & 0x3
                ps_map = {0: "D0 (Full On)", 1: "D1", 2: "D2", 3: "D3hot (Off)"}
                print("      PMCSR = 0x%04X, Power State = %s" % (pmcsr, ps_map.get(ps, "?")))
            
            ptr = next_ptr
    
    int_line = data.get(0x3C, 0)
    int_pin = data.get(0x3D, 0)
    if int_pin != 0:
        pin_map = {1: "INTA#", 2: "INTB#", 3: "INTC#", 4: "INTD#"}
        print("  INT Line:  %d" % int_line)
        print("  INT Pin:   %s" % pin_map.get(int_pin, "0x%02X" % int_pin))
    
    print("\n--- Full Hex Dump ---")
    print_hex_dump(data, base_addr)

def main():
    dump_dir = "/root/acpi_intel_sst/rwdumps"
    
    print("#" * 80)
    print("# RWEverything Memory Dump Analysis")
    print("# Broadwell-U / Wildcat Point-LP PCH")
    print("#" * 80)
    
    # ---- RCBA ----
    filepath = os.path.join(dump_dir, "M00000000FED1C000.rw")
    if os.path.exists(filepath):
        base_addr, data = parse_rw_file(filepath)
        print("\n\n" + "#" * 80)
        print("# FILE: M00000000FED1C000.rw")
        print("# PCH Root Complex Base Address (RCBA)")
        print("# Base: 0x%08X, Bytes: %d" % (base_addr, len(data)))
        print("#" * 80 + "\n")
        analyze_rcba(base_addr, data)
    
    # ---- LPSS Private ----
    filepath = os.path.join(dump_dir, "M00000000FE300000.rw")
    if os.path.exists(filepath):
        base_addr, data = parse_rw_file(filepath)
        print("\n\n" + "#" * 80)
        print("# FILE: M00000000FE300000.rw")
        print("# LPSS Private Register Region")
        print("# Base: 0x%08X, Bytes: %d" % (base_addr, len(data)))
        print("#" * 80 + "\n")
        analyze_lpss_private(base_addr, data)
    
    # ---- I2C0 ----
    filepath = os.path.join(dump_dir, "M00000000FE103000.rw")
    if os.path.exists(filepath):
        base_addr, data = parse_rw_file(filepath)
        print("\n\n" + "#" * 80)
        print("# FILE: M00000000FE103000.rw")
        print("# LPSS I2C0 Controller (DesignWare)")
        print("# Base: 0x%08X, Bytes: %d" % (base_addr, len(data)))
        print("#" * 80 + "\n")
        analyze_i2c(base_addr, data, "I2C0")
    
    # ---- I2C1 ----
    filepath = os.path.join(dump_dir, "M00000000FE105000.rw")
    if os.path.exists(filepath):
        base_addr, data = parse_rw_file(filepath)
        print("\n\n" + "#" * 80)
        print("# FILE: M00000000FE105000.rw")
        print("# LPSS I2C1 Controller (DesignWare)")
        print("# Base: 0x%08X, Bytes: %d" % (base_addr, len(data)))
        print("#" * 80 + "\n")
        analyze_i2c(base_addr, data, "I2C1")
    
    # ---- SST PCI-like space at 0xFE100000 ----
    filepath = os.path.join(dump_dir, "M00000000FE100000.rw")
    if os.path.exists(filepath):
        base_addr, data = parse_rw_file(filepath)
        print("\n\n" + "#" * 80)
        print("# FILE: M00000000FE100000.rw")
        print("# SST/LPSS Region at 0xFE100000 (PCI config shadow?)")
        print("# Base: 0x%08X, Bytes: %d" % (base_addr, len(data)))
        print("#" * 80 + "\n")
        vid = read_word(data, 0x00)
        if vid == 0x8086:
            analyze_pci_config_space(base_addr, data)
        elif not is_all_ff(data):
            print("  VID = 0x%04X (not Intel), analyzing generically..." % vid)
            # still try PCI decode
            analyze_pci_config_space(base_addr, data)
    
    # ---- FE000000 ----
    filepath = os.path.join(dump_dir, "M00000000FE000000.rw")
    if os.path.exists(filepath):
        base_addr, data = parse_rw_file(filepath)
        print("\n\n" + "#" * 80)
        print("# FILE: M00000000FE000000.rw")
        print("# Region at 0xFE000000")
        print("# Base: 0x%08X, Bytes: %d" % (base_addr, len(data)))
        print("#" * 80 + "\n")
        if is_all_ff(data):
            print("  *** ALL BYTES ARE 0xFF - Region is UNMAPPED/INACCESSIBLE ***")
        else:
            print("  Has live data. Non-zero: %d bytes" % sum(1 for v in data.values() if v != 0))
            print_hex_dump(data, base_addr)
    
    # ---- PCI Config dumps ----
    for pci_file in ["P001400.rw", "P001F06.rw"]:
        filepath = os.path.join(dump_dir, pci_file)
        if os.path.exists(filepath):
            base_addr, data = parse_rw_file(filepath)
            print("\n\n" + "#" * 80)
            print("# FILE: %s (PCI Config Space)" % pci_file)
            # Parse the PCI BDF from filename: PBBDDFF where BB=bus, DD=dev, FF=func
            # P001400 => Bus 00, Dev 14, Func 00
            # P001F06 => Bus 00, Dev 1F, Func 06
            bdf = pci_file.replace("P", "").replace(".rw", "")
            if len(bdf) == 6:
                bus = int(bdf[0:2], 16)
                dev = int(bdf[2:4], 16)
                func = int(bdf[4:6], 16)
                print("# PCI Bus %02X Dev %02X Func %02X (%02X:%02X.%X)" % (bus, dev, func, bus, dev, func))
            print("# Bytes: %d" % len(data))
            print("#" * 80 + "\n")
            if not is_all_ff(data):
                analyze_pci_config_space(base_addr, data)
            else:
                print("  *** ALL BYTES ARE 0xFF - No device present ***")
    
    # ======== GRAND SUMMARY ========
    print("\n\n" + "=" * 80)
    print("GRAND SUMMARY")
    print("=" * 80)
    
    summary_files = [
        ("M00000000FED1C000.rw", "RCBA",        "PCH Root Complex Base Address"),
        ("M00000000FE300000.rw", "LPSS_PRIV",   "LPSS Private Region"),
        ("M00000000FE103000.rw", "I2C0",        "LPSS I2C0"),
        ("M00000000FE105000.rw", "I2C1",        "LPSS I2C1"),
        ("M00000000FE100000.rw", "SST_PCI",     "SST PCI Config Shadow"),
        ("M00000000FE000000.rw", "FE000000",    "Region at 0xFE000000"),
    ]
    
    results = {}
    for fn, tag, desc in summary_files:
        fp = os.path.join(dump_dir, fn)
        if os.path.exists(fp):
            ba, d = parse_rw_file(fp)
            aff = is_all_ff(d)
            azero = is_all_zero(d)
            nz = sum(1 for v in d.values() if v != 0x00)
            nff = sum(1 for v in d.values() if v != 0xFF)
            if aff:
                status = "UNMAPPED (all 0xFF)"
            elif azero:
                status = "ZEROED (all 0x00)"
            else:
                status = "ACTIVE (%d non-zero, %d non-FF)" % (nz, nff)
            print("  %-12s 0x%08X: %s" % (tag, ba, status))
            results[tag] = (aff, azero, ba, d)
    
    print("\n--- Key Findings ---")
    
    # RCBA
    if "RCBA" in results:
        aff, azero, ba, d = results["RCBA"]
        if not aff:
            nz = sum(1 for v in d.values() if v != 0x00)
            print("\n  RCBA (0x%08X):" % ba)
            print("    Accessible with %d non-zero bytes." % nz)
            print("    Non-zero values found at offsets: %s" % 
                  ", ".join("0x%02X=0x%02X" % (o, v) for o, v in sorted(d.items()) if v != 0))
            print("    CRITICAL: Only 256 bytes captured. Full RCBA is 16KB.")
            print("    LPSS control registers at RCBA+0x2030, +0x3400, +0x3418 are NOT in this dump.")
    
    # LPSS Private
    if "LPSS_PRIV" in results:
        aff, _, ba, _ = results["LPSS_PRIV"]
        print("\n  LPSS Private (0x%08X):" % ba)
        if aff:
            print("    INACCESSIBLE - all 0xFF. LPSS ACPI mode is NOT active.")
            print("    The PCH is not decoding LPSS private registers at this address.")
    
    # I2C0 vs I2C1
    if "I2C0" in results:
        aff0, _, ba0, _ = results["I2C0"]
        print("\n  I2C0 (0x%08X):" % ba0)
        if aff0:
            print("    INACCESSIBLE - all 0xFF. Device disabled or not mapped.")
    
    if "I2C1" in results:
        aff1, _, ba1, d1 = results["I2C1"]
        print("\n  I2C1 (0x%08X):" % ba1)
        if not aff1:
            ic_con = read_dword(d1, 0x00)
            ic_tar = read_dword(d1, 0x04)
            ic_en = read_dword(d1, 0x6C)
            ic_status = read_dword(d1, 0x70)
            print("    LIVE AND ACTIVE!")
            print("    IC_CON=0x%08X IC_TAR=0x%08X IC_ENABLE=0x%08X IC_STATUS=0x%08X" % 
                  (ic_con, ic_tar, ic_en, ic_status))
            tar_addr = ic_tar & 0x7F
            print("    Target I2C address: 0x%02X" % tar_addr)
    
    # SST
    if "SST_PCI" in results:
        aff, _, ba, d = results["SST_PCI"]
        print("\n  SST PCI Config (0x%08X):" % ba)
        if not aff:
            vid = read_word(d, 0x00)
            did = read_word(d, 0x02)
            cmd = read_word(d, 0x04)
            print("    VID=0x%04X DID=0x%04X CMD=0x%04X" % (vid, did, cmd))
            print("    Memory Space: %s" % ("ENABLED" if (cmd >> 1) & 1 else "DISABLED"))
            bar0 = read_dword(d, 0x10)
            bar1 = read_dword(d, 0x14)
            print("    BAR0=0x%08X BAR1=0x%08X" % (bar0, bar1))
    
    print("\n--- Recommended Actions ---")
    print("  1. CRITICAL: Dump full 16KB RCBA: addresses 0xFED1C000 through 0xFED1FFFF")
    print("     In RWEverything: Memory tab, Address=FED1C000, dump 16384 bytes")
    print("     Key offsets to check:")
    print("       0xFED1F400 (RCBA+0x3400): FD register - function disable bits")
    print("       0xFED1F418 (RCBA+0x3418): FD2 register")
    print("       0xFED1E030 (RCBA+0x2030): LPSS IOSF sideband port")
    print("  2. If I2C1 is live, probe for audio codec at common addresses:")
    print("     0x1B (RT5645), 0x1C (RT286), 0x2C (RT286 alt)")
    print("  3. Check ACPI DSDT for _ADR values to find the real LPSS BAR addresses")
    print("  4. For SST: ensure PCI Command bit 1 is set and BAR0 points to valid MMIO")

if __name__ == "__main__":
    main()


def compare_i2c_dumps():
    """Compare the I2C1 main dump vs BAK dump to see state changes."""
    dump_dir = "/root/acpi_intel_sst/rwdumps"
    
    print("\n\n" + "=" * 80)
    print("I2C1 DUMP COMPARISON: Current vs BAK (previous snapshot)")
    print("=" * 80)
    
    _, data_cur = parse_rw_file(os.path.join(dump_dir, "M00000000FE105000.rw"))
    _, data_bak = parse_rw_file(os.path.join(dump_dir, "M00000000FE105000.rw.BAK"))
    
    dw_i2c_regs = {
        0x00: "IC_CON", 0x04: "IC_TAR", 0x08: "IC_SAR", 0x0C: "IC_HS_MADDR",
        0x10: "IC_DATA_CMD", 0x14: "IC_SS_SCL_HCNT", 0x18: "IC_SS_SCL_LCNT",
        0x1C: "IC_FS_SCL_HCNT", 0x20: "IC_FS_SCL_LCNT", 0x24: "IC_HS_SCL_HCNT",
        0x28: "IC_HS_SCL_LCNT", 0x2C: "IC_INTR_STAT", 0x30: "IC_INTR_MASK",
        0x34: "IC_RAW_INTR_STAT", 0x38: "IC_RX_TL", 0x3C: "IC_TX_TL",
        0x40: "IC_CLR_INTR", 0x6C: "IC_ENABLE", 0x70: "IC_STATUS",
        0x74: "IC_TXFLR", 0x78: "IC_RXFLR", 0x7C: "IC_SDA_HOLD",
        0x80: "IC_TX_ABRT_SRC", 0x88: "IC_DMA_CR", 0x8C: "IC_DMA_TDLR",
        0x94: "IC_SDA_SETUP", 0x98: "IC_ACK_GEN_CALL", 0x9C: "IC_ENABLE_STATUS",
        0xA0: "IC_FS_SPKLEN", 0xA4: "IC_HS_SPKLEN",
        0xF4: "IC_COMP_PARAM_1", 0xF8: "IC_COMP_VERSION", 0xFC: "IC_COMP_TYPE",
        0x5C: "IC_CLR_ACTIVITY",
    }
    
    print("\n  %-6s %-24s  %-12s  %-12s  %s" % ("Offset", "Register", "BAK (prev)", "Current", "Change"))
    print("  " + "-" * 74)
    
    for off in range(0, 256, 4):
        dw_bak = read_dword(data_bak, off)
        dw_cur = read_dword(data_cur, off)
        if dw_bak != dw_cur:
            regname = dw_i2c_regs.get(off, "???")
            print("  0x%02X   %-24s  0x%08X    0x%08X    CHANGED" % (off, regname, dw_bak, dw_cur))
    
    # Decode the BAK state
    bak_con = read_dword(data_bak, 0x00)
    bak_en = read_dword(data_bak, 0x6C)
    bak_status = read_dword(data_bak, 0x70)
    bak_intr_mask = read_dword(data_bak, 0x30)
    bak_raw_intr = read_dword(data_bak, 0x34)
    bak_en_status = read_dword(data_bak, 0x9C)
    bak_dma_cr = read_dword(data_bak, 0x88)
    
    print("\n  --- BAK Snapshot State (I2C was ACTIVE) ---")
    print("    IC_ENABLE    = 0x%08X  (%s)" % (bak_en, "ENABLED" if bak_en & 1 else "DISABLED"))
    print("    IC_STATUS    = 0x%08X" % bak_status)
    print("      ACTIVITY=%d TFNF=%d TFE=%d RFNE=%d RFF=%d MST_ACT=%d SLV_ACT=%d" % (
        bak_status & 1, (bak_status>>1)&1, (bak_status>>2)&1,
        (bak_status>>3)&1, (bak_status>>4)&1, (bak_status>>5)&1, (bak_status>>6)&1))
    print("    IC_INTR_MASK = 0x%08X" % bak_intr_mask)
    print("      Enabled interrupts: ", end="")
    intr_names = ["RX_UNDER","RX_OVER","RX_FULL","TX_OVER","TX_EMPTY",
                  "RD_REQ","TX_ABRT","RX_DONE","ACTIVITY","STOP_DET",
                  "START_DET","GEN_CALL","RESTART_DET"]
    enabled = []
    for i, name in enumerate(intr_names):
        if (bak_intr_mask >> i) & 1:
            enabled.append(name)
    print(", ".join(enabled) if enabled else "(none)")
    
    print("    IC_RAW_INTR  = 0x%08X" % bak_raw_intr)
    pending = []
    for i, name in enumerate(intr_names):
        if (bak_raw_intr >> i) & 1:
            pending.append(name)
    print("      Pending raw: %s" % (", ".join(pending) if pending else "(none)"))
    
    print("    IC_ENABLE_ST = 0x%08X (EN=%d)" % (bak_en_status, bak_en_status & 1))
    print("    IC_DMA_CR    = 0x%08X (TDMAE=%d RDMAE=%d)" % (bak_dma_cr, bak_dma_cr & 1, (bak_dma_cr >> 1) & 1))
    
    print("\n  --- Current Snapshot State (I2C was DISABLED) ---")
    cur_en = read_dword(data_cur, 0x6C)
    cur_status = read_dword(data_cur, 0x70)
    print("    IC_ENABLE    = 0x%08X  (%s)" % (cur_en, "ENABLED" if cur_en & 1 else "DISABLED"))
    print("    IC_STATUS    = 0x%08X" % cur_status)
    print("      ACTIVITY=%d TFNF=%d TFE=%d" % (cur_status & 1, (cur_status>>1)&1, (cur_status>>2)&1))
    
    print("\n  INTERPRETATION:")
    print("    The BAK dump captured I2C1 in an ACTIVE state (IC_ENABLE=1, DMA enabled).")
    print("    The current dump shows I2C1 DISABLED (IC_ENABLE=0) but registers retained.")
    print("    This means something previously used I2C1 (likely BIOS/firmware for codec init)")
    print("    and then disabled it. The target address 0x2C is preserved in IC_TAR.")
    print("    0x2C = I2C address typically used by RT286/RT298 Realtek HD Audio codec.")

if len(sys.argv) > 1 and sys.argv[1] == "--compare":
    compare_i2c_dumps()
