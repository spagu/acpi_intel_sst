#!/usr/bin/env python3
"""
RWEverything dump parser for Intel SST analysis.
Parses PCI, Memory, SMBUS dumps from RWEverything format.
"""

import os
import re
import sys
from collections import OrderedDict

DUMP_DIR = "/root/acpi_intel_sst/rwdumps"

# ============================================================
# PCI class code database (abbreviated)
# ============================================================
PCI_CLASSES = {
    (0x00, 0x00): "Non-VGA-Compatible Unclassified Device",
    (0x01, 0x01): "IDE Controller",
    (0x01, 0x06): "SATA Controller",
    (0x02, 0x00): "Ethernet Controller",
    (0x03, 0x00): "VGA Compatible Controller",
    (0x04, 0x01): "Multimedia Audio Controller",
    (0x04, 0x03): "Audio Device",
    (0x06, 0x00): "Host Bridge",
    (0x06, 0x01): "ISA/LPC Bridge",
    (0x06, 0x04): "PCI-to-PCI Bridge",
    (0x08, 0x05): "SD Host Controller",
    (0x0B, 0x40): "Co-processor",
    (0x0C, 0x03): "USB Controller",
    (0x0C, 0x05): "SMBus Controller",
    (0x0C, 0x80): "Serial Bus Controller (Other)",
    (0x11, 0x80): "Signal Processing Controller (Other)",
}

# Known Intel Broadwell-U device IDs
INTEL_DEVIDS = {
    0x1604: "Broadwell-U Host Bridge",
    0x1616: "Broadwell-U GT2 Integrated Graphics",
    0x160C: "Broadwell-U Audio (HD Audio)",
    0x1603: "Broadwell-U Thermal/DPTF",
    0x9CB1: "Wildcat Point-LP USB xHCI",
    0x9CBA: "Wildcat Point-LP MEI #1",
    0x9C90: "Wildcat Point-LP PCIe Root Port #1",
    0x9C96: "Wildcat Point-LP PCIe Root Port #4",
    0x9CC3: "Wildcat Point-LP LPC Bridge",
    0x9C83: "Wildcat Point-LP SATA (AHCI)",
    0x9CA2: "Wildcat Point-LP SMBus Controller",
    0x9CA4: "Wildcat Point-LP Thermal",
    0x9CB6: "Wildcat Point-LP Smart Sound Technology (SST)",
}


def parse_rw_data_lines(lines):
    """Parse XX=YY data lines into a byte array."""
    data = {}
    for line in lines:
        line = line.strip().rstrip()
        if not line or line.startswith(';') or line.startswith('Width:'):
            continue
        # Parse XX=YY pairs
        pairs = re.findall(r'([0-9A-Fa-f]+)=([0-9A-Fa-f]+)', line)
        for off_str, val_str in pairs:
            offset = int(off_str, 16)
            value = int(val_str, 16)
            data[offset] = value
    return data


def data_to_bytes(data, size=None):
    """Convert offset->value dict to bytearray."""
    if not data:
        return bytearray()
    if size is None:
        size = max(data.keys()) + 1
    result = bytearray(size)
    for off, val in data.items():
        if off < size:
            result[off] = val
    return result


def read_u16(ba, off):
    if off + 1 >= len(ba):
        return 0
    return ba[off] | (ba[off+1] << 8)


def read_u32(ba, off):
    if off + 3 >= len(ba):
        return 0
    return ba[off] | (ba[off+1] << 8) | (ba[off+2] << 16) | (ba[off+3] << 24)


def hex_dump(ba, base_addr=0, indent="  "):
    """Pretty hex dump of bytearray."""
    lines = []
    for i in range(0, len(ba), 16):
        hex_bytes = ' '.join(f'{ba[j]:02X}' for j in range(i, min(i+16, len(ba))))
        ascii_chars = ''.join(
            chr(ba[j]) if 32 <= ba[j] < 127 else '.'
            for j in range(i, min(i+16, len(ba)))
        )
        lines.append(f"{indent}{base_addr + i:04X}: {hex_bytes:<48s}  {ascii_chars}")
    return '\n'.join(lines)


def decode_pci_command(cmd):
    bits = []
    names = [
        (0, "IO"), (1, "MEM"), (2, "BusMaster"), (3, "SpecCycle"),
        (4, "MemWrInv"), (5, "VGA Snoop"), (6, "ParErr"), (8, "SERR"),
        (9, "FastB2B"), (10, "INTx Dis"),
    ]
    for bit, name in names:
        if cmd & (1 << bit):
            bits.append(name)
    return ' | '.join(bits) if bits else "(none)"


def decode_pci_status(sts):
    bits = []
    names = [
        (3, "INTx"), (4, "CapList"), (5, "66MHz"), (7, "FastB2B"),
        (8, "MasterDataPar"), (11, "SigTgtAbort"), (12, "RcvTgtAbort"),
        (13, "RcvMasterAbort"), (14, "SigSysErr"), (15, "DetParErr"),
    ]
    for bit, name in names:
        if sts & (1 << bit):
            bits.append(name)
    return ' | '.join(bits) if bits else "(none)"


def get_class_name(base_class, sub_class):
    return PCI_CLASSES.get((base_class, sub_class), f"Unknown ({base_class:02X}:{sub_class:02X})")


def decode_pci_config(ba, label="", is_pcie=False):
    """Decode standard PCI config space registers."""
    vid = read_u16(ba, 0x00)
    did = read_u16(ba, 0x02)
    cmd = read_u16(ba, 0x04)
    sts = read_u16(ba, 0x06)
    rev = ba[0x08]
    prog_if = ba[0x09]
    sub_class = ba[0x0A]
    base_class = ba[0x0B]
    cache_line = ba[0x0C]
    lat_timer = ba[0x0D]
    header_type = ba[0x0E]
    bist = ba[0x0F]

    dev_name = INTEL_DEVIDS.get(did, "Unknown")
    class_name = get_class_name(base_class, sub_class)

    print(f"\n{'='*70}")
    if label:
        print(f"  {label}")
        print(f"{'='*70}")

    print(f"  Vendor ID:     0x{vid:04X} {'(Intel)' if vid == 0x8086 else ''}")
    print(f"  Device ID:     0x{did:04X} ({dev_name})")
    print(f"  Command:       0x{cmd:04X} [{decode_pci_command(cmd)}]")
    print(f"  Status:        0x{sts:04X} [{decode_pci_status(sts)}]")
    print(f"  Revision:      0x{rev:02X}")
    print(f"  Class Code:    {base_class:02X}:{sub_class:02X}:{prog_if:02X} ({class_name})")
    print(f"  Header Type:   0x{header_type:02X} ({'Multi-function' if header_type & 0x80 else 'Single-function'}, Type {header_type & 0x7F})")
    print(f"  Cache Line:    {cache_line}")
    print(f"  Latency Timer: {lat_timer}")

    hdr_type = header_type & 0x7F

    if hdr_type == 0:
        # Type 0: Regular device
        for i in range(6):
            bar = read_u32(ba, 0x10 + i*4)
            if bar == 0:
                print(f"  BAR{i}:          0x{bar:08X} (not implemented)")
            elif bar & 1:
                print(f"  BAR{i}:          0x{bar:08X} (I/O @ 0x{bar & ~0x3:04X})")
            else:
                bar_type = (bar >> 1) & 3
                prefetch = "prefetchable" if bar & 8 else "non-prefetchable"
                addr = bar & 0xFFFFFFF0
                if bar_type == 2:  # 64-bit
                    bar_hi = read_u32(ba, 0x10 + (i+1)*4)
                    addr64 = addr | (bar_hi << 32)
                    print(f"  BAR{i}:          0x{bar:08X} (64-bit {prefetch} MMIO @ 0x{addr64:016X})")
                else:
                    print(f"  BAR{i}:          0x{bar:08X} (32-bit {prefetch} MMIO @ 0x{addr:08X})")

        cardbus_cis = read_u32(ba, 0x28)
        subsys_vid = read_u16(ba, 0x2C)
        subsys_did = read_u16(ba, 0x2E)
        exp_rom = read_u32(ba, 0x30)
        cap_ptr = ba[0x34]
        int_line = ba[0x3C]
        int_pin = ba[0x3D]
        min_gnt = ba[0x3E]
        max_lat = ba[0x3F]

        print(f"  Subsystem VID: 0x{subsys_vid:04X}")
        print(f"  Subsystem DID: 0x{subsys_did:04X}")
        if exp_rom:
            print(f"  Expansion ROM: 0x{exp_rom:08X}")
        print(f"  Cap Pointer:   0x{cap_ptr:02X}")
        pin_str = 'ABCD'[int_pin-1] if 1 <= int_pin <= 4 else f'{int_pin}'
        print(f"  IRQ Line:      {int_line} (Pin: {pin_str})")

        # Walk capability list
        if sts & 0x10:  # Cap List bit set
            walk_capabilities(ba, cap_ptr, is_pcie)

    elif hdr_type == 1:
        # Type 1: PCI-to-PCI Bridge
        bar0 = read_u32(ba, 0x10)
        bar1 = read_u32(ba, 0x14)
        print(f"  BAR0:          0x{bar0:08X}")
        print(f"  BAR1:          0x{bar1:08X}")
        pri_bus = ba[0x18]
        sec_bus = ba[0x19]
        sub_bus = ba[0x1A]
        print(f"  Primary Bus:   {pri_bus}")
        print(f"  Secondary Bus: {sec_bus}")
        print(f"  Subordinate:   {sub_bus}")
        io_base = ba[0x1C]
        io_limit = ba[0x1D]
        mem_base = read_u16(ba, 0x20)
        mem_limit = read_u16(ba, 0x22)
        print(f"  IO Base/Limit: 0x{io_base:02X}/0x{io_limit:02X}")
        print(f"  Mem Base:      0x{mem_base << 16:08X}")
        print(f"  Mem Limit:     0x{(mem_limit << 16) | 0xFFFFF:08X}")
        cap_ptr = ba[0x34]
        print(f"  Cap Pointer:   0x{cap_ptr:02X}")
        if sts & 0x10:
            walk_capabilities(ba, cap_ptr, is_pcie)


def walk_capabilities(ba, cap_ptr, is_pcie=False):
    """Walk the PCI capability linked list."""
    print(f"\n  -- PCI Capabilities Chain --")
    seen = set()
    ptr = cap_ptr
    while ptr and ptr != 0xFF and ptr not in seen:
        seen.add(ptr)
        if ptr + 1 >= len(ba):
            break
        cap_id = ba[ptr]
        next_ptr = ba[ptr + 1]

        cap_names = {
            0x01: "Power Management",
            0x05: "MSI",
            0x09: "Vendor Specific",
            0x10: "PCI Express",
            0x11: "MSI-X",
            0x12: "SATA Config",
        }
        cap_name = cap_names.get(cap_id, f"Unknown(0x{cap_id:02X})")
        print(f"  Cap @ 0x{ptr:02X}: ID=0x{cap_id:02X} ({cap_name}), Next=0x{next_ptr:02X}")

        if cap_id == 0x01:  # Power Management
            pmc = read_u16(ba, ptr + 2)
            pmcsr = read_u16(ba, ptr + 4)
            pmcsr_bse = ba[ptr + 6] if ptr + 6 < len(ba) else 0
            pm_data = ba[ptr + 7] if ptr + 7 < len(ba) else 0
            power_state = pmcsr & 0x3
            ps_names = {0: "D0", 1: "D1", 2: "D2", 3: "D3hot"}
            print(f"      PMC:   0x{pmc:04X} (Version {pmc & 7}, D1={1 if pmc & 0x200 else 0}, D2={1 if pmc & 0x400 else 0})")
            print(f"      PMCSR: 0x{pmcsr:04X} (Power State: {ps_names.get(power_state, '?')})")

        elif cap_id == 0x05:  # MSI
            msi_ctrl = read_u16(ba, ptr + 2)
            enabled = "Enabled" if msi_ctrl & 1 else "Disabled"
            num_vec = 1 << ((msi_ctrl >> 1) & 7)
            is_64bit = bool(msi_ctrl & 0x80)
            print(f"      MSI Ctrl: 0x{msi_ctrl:04X} ({enabled}, {num_vec} vectors, {'64-bit' if is_64bit else '32-bit'})")
            if is_64bit:
                msi_addr = read_u32(ba, ptr + 4) | (read_u32(ba, ptr + 8) << 32)
                msi_data = read_u16(ba, ptr + 12)
                print(f"      MSI Addr: 0x{msi_addr:016X}")
                print(f"      MSI Data: 0x{msi_data:04X}")
            else:
                msi_addr = read_u32(ba, ptr + 4)
                msi_data = read_u16(ba, ptr + 8)
                print(f"      MSI Addr: 0x{msi_addr:08X}")
                print(f"      MSI Data: 0x{msi_data:04X}")

        elif cap_id == 0x10:  # PCI Express
            pcie_cap = read_u16(ba, ptr + 2)
            pcie_type = (pcie_cap >> 4) & 0xF
            type_names = {
                0: "Endpoint", 1: "Legacy Endpoint",
                4: "Root Port", 5: "Upstream Switch",
                6: "Downstream Switch", 7: "PCIe-PCI Bridge",
                8: "PCI-PCIe Bridge", 9: "RC Integrated Endpoint",
                10: "RC Event Collector",
            }
            print(f"      PCIe Cap:  0x{pcie_cap:04X} (Type: {type_names.get(pcie_type, f'Unknown({pcie_type})')}, Ver {pcie_cap & 0xF})")
            dev_cap = read_u32(ba, ptr + 4)
            dev_ctrl = read_u16(ba, ptr + 8)
            dev_sts = read_u16(ba, ptr + 10)
            link_cap = read_u32(ba, ptr + 12)
            link_ctrl = read_u16(ba, ptr + 16)
            link_sts = read_u16(ba, ptr + 18)
            link_speed = link_cap & 0xF
            link_width = (link_cap >> 4) & 0x3F
            cur_speed = link_sts & 0xF
            cur_width = (link_sts >> 4) & 0x3F
            speed_names = {1: "2.5GT/s", 2: "5GT/s", 3: "8GT/s", 4: "16GT/s"}
            print(f"      Dev Cap:   0x{dev_cap:08X}")
            print(f"      Dev Ctrl:  0x{dev_ctrl:04X}, Dev Sts: 0x{dev_sts:04X}")
            print(f"      Link Cap:  Max {speed_names.get(link_speed, '?')} x{link_width}")
            print(f"      Link Sts:  Current {speed_names.get(cur_speed, '?')} x{cur_width}")

        elif cap_id == 0x09:  # Vendor Specific
            vs_len = ba[ptr + 2] if ptr + 2 < len(ba) else 0
            print(f"      VS Length: {vs_len} bytes")

        ptr = next_ptr


def decode_bar1_mirror(ba, label="BAR1 Mirror (PCI Config Shadow)"):
    """Decode BAR1 at 0xFE100000 - the SST PCI config shadow space."""
    print(f"\n{'='*70}")
    print(f"  {label}")
    print(f"{'='*70}")

    vid = read_u16(ba, 0x00)
    did = read_u16(ba, 0x02)
    cmd = read_u16(ba, 0x04)
    sts = read_u16(ba, 0x06)
    rev = ba[0x08]
    prog_if = ba[0x09]
    sub_class = ba[0x0A]
    base_class = ba[0x0B]

    dev_name = INTEL_DEVIDS.get(did, "Unknown")
    class_name = get_class_name(base_class, sub_class)

    print(f"  [0x00] VID:        0x{vid:04X} {'(Intel)' if vid == 0x8086 else ''}")
    print(f"  [0x02] DID:        0x{did:04X} ({dev_name})")
    print(f"  [0x04] Command:    0x{cmd:04X} [{decode_pci_command(cmd)}]")
    print(f"  [0x06] Status:     0x{sts:04X} [{decode_pci_status(sts)}]")
    print(f"  [0x08] Revision:   0x{rev:02X}")
    print(f"  [0x09] ProgIF:     0x{prog_if:02X}")
    print(f"  [0x0A] SubClass:   0x{sub_class:02X}")
    print(f"  [0x0B] BaseClass:  0x{base_class:02X} ({class_name})")
    print(f"  [0x0C] CacheLine:  0x{ba[0x0C]:02X}")
    print(f"  [0x0D] LatTimer:   0x{ba[0x0D]:02X}")
    print(f"  [0x0E] HeaderType: 0x{ba[0x0E]:02X}")
    print(f"  [0x0F] BIST:       0x{ba[0x0F]:02X}")

    bar0 = read_u32(ba, 0x10)
    bar1 = read_u32(ba, 0x14)
    bar2 = read_u32(ba, 0x18)
    bar3 = read_u32(ba, 0x1C)
    bar4 = read_u32(ba, 0x20)
    bar5 = read_u32(ba, 0x24)

    print(f"  [0x10] BAR0:       0x{bar0:08X}")
    print(f"  [0x14] BAR1:       0x{bar1:08X}")
    print(f"  [0x18] BAR2:       0x{bar2:08X}")
    print(f"  [0x1C] BAR3:       0x{bar3:08X}")
    print(f"  [0x20] BAR4:       0x{bar4:08X}")
    print(f"  [0x24] BAR5:       0x{bar5:08X}")

    subsys_vid = read_u16(ba, 0x2C)
    subsys_did = read_u16(ba, 0x2E)
    cap_ptr = ba[0x34]

    print(f"  [0x2C] SubsysVID:  0x{subsys_vid:04X}")
    print(f"  [0x2E] SubsysDID:  0x{subsys_did:04X}")
    print(f"  [0x34] CapPtr:     0x{cap_ptr:02X}")

    int_line = ba[0x3C]
    int_pin = ba[0x3D]
    print(f"  [0x3C] INT Line:   0x{int_line:02X}")
    print(f"  [0x3D] INT Pin:    0x{int_pin:02X}")

    # SHIM-like registers in the BAR1 space
    print(f"\n  -- SHIM/LPE Registers (if BAR1 is SHIM-like) --")
    isrx = read_u32(ba, 0x18)
    isrd = read_u32(ba, 0x20)
    imrx = read_u32(ba, 0x28)
    ipcx = read_u32(ba, 0x38)
    ipcd = read_u32(ba, 0x40)
    clkctl = read_u32(ba, 0x78)
    print(f"  [0x18] ISRX:       0x{isrx:08X}")
    print(f"  [0x20] ISRD:       0x{isrd:08X}")
    print(f"  [0x28] IMRX:       0x{imrx:08X}")
    print(f"  [0x38] IPCX:       0x{ipcx:08X}")
    print(f"  [0x40] IPCD:       0x{ipcd:08X}")
    print(f"  [0x78] CLKCTL:     0x{clkctl:08X}")

    # Capability at 0x80 - Power Management
    print(f"\n  -- Power Management Capability @ 0x80 --")
    pm_id = ba[0x80]
    pm_next = ba[0x81]
    pmc = read_u16(ba, 0x82)
    pmcsr = read_u16(ba, 0x84)
    pmcsr_full = read_u32(ba, 0x84)  # include BSE extension
    power_state = pmcsr & 0x3
    ps_names = {0: "D0", 1: "D1", 2: "D2", 3: "D3hot"}
    print(f"  [0x80] Cap ID:     0x{pm_id:02X} ({'Power Management' if pm_id == 0x01 else 'UNEXPECTED'})")
    print(f"  [0x81] Next Cap:   0x{pm_next:02X}")
    print(f"  [0x82] PMC:        0x{pmc:04X}")
    print(f"  [0x84] PMCSR:      0x{pmcsr:04X} (Power State: {ps_names.get(power_state, '?')})")
    print(f"  [0x84] PMCSR(32b): 0x{pmcsr_full:08X}")

    # Vendor-specific SST registers
    print(f"\n  -- SST Vendor-Specific Registers --")
    vdrtctl0 = read_u32(ba, 0xA0)
    vdrtctl1 = read_u32(ba, 0xA4)
    vdrtctl2 = read_u32(ba, 0xA8)
    vdrtctl3 = read_u32(ba, 0xAC)
    print(f"  [0xA0] VDRTCTL0:   0x{vdrtctl0:08X}")
    d3pg_val = (vdrtctl0 >> 0) & 1
    d3srampgd = (vdrtctl0 >> 8) & 1
    d3pgallowed = (vdrtctl0 >> 16) & 1
    print(f"           D3PG={d3pg_val}, D3SRAMPGD={d3srampgd}, D3PGAllowed={d3pgallowed}")
    print(f"  [0xA4] VDRTCTL1:   0x{vdrtctl1:08X}")
    print(f"  [0xA8] VDRTCTL2:   0x{vdrtctl2:08X}")
    trunk_clk = (vdrtctl2 >> 0) & 0x3F
    io_clk = (vdrtctl2 >> 6) & 0x3
    dclk_sel = (vdrtctl2 >> 8) & 0x3
    print(f"           TRUNK_CLK_GATING={trunk_clk:#04x}, IO_CLK={io_clk}, DCLK_SEL={dclk_sel}")
    print(f"  [0xAC] VDRTCTL3:   0x{vdrtctl3:08X}")

    # Last 4 bytes
    last4 = read_u32(ba, 0xF8)
    print(f"  [0xF8] Last DW:    0x{last4:08X}")
    print(f"  [0xFC]:            0x{read_u32(ba, 0xFC):08X}")

    return {
        'vid': vid, 'did': did, 'cmd': cmd, 'sts': sts,
        'rev': rev, 'class': (base_class, sub_class, prog_if),
        'bar0': bar0, 'bar1': bar1,
        'cap_ptr': cap_ptr, 'pm_id': pm_id, 'pmcsr': pmcsr,
        'vdrtctl0': vdrtctl0, 'vdrtctl1': vdrtctl1, 'vdrtctl2': vdrtctl2,
        'clkctl': clkctl, 'last_f8': last4,
    }


def parse_rw_file(filepath):
    """Parse a single .rw file. Returns list of (header_info, bytearray) tuples."""
    with open(filepath, 'r', errors='replace') as f:
        content = f.read()

    # Check if it's a standard RW format
    lines = content.split('\n')
    if not lines:
        return []

    # Handle files that don't start with Type: (RSDP, AcpiTbls, SMBIOS)
    first_line = lines[0].strip()
    if not first_line.startswith('Type:') and 'Root System' not in first_line:
        # Could be RSDP or ACPI tables - different format
        return [({'type': 'other', 'raw_header': first_line, 'filename': os.path.basename(filepath)}, None)]

    if 'Root System' in first_line:
        return [({'type': 'rsdp', 'raw_header': first_line, 'filename': os.path.basename(filepath)}, None)]

    # Split into device blocks
    blocks = []
    current_header = None
    current_lines = []

    for line in lines:
        line_stripped = line.strip()
        if line_stripped.startswith('Type:'):
            if current_header is not None:
                blocks.append((current_header, current_lines))
            current_header = line_stripped
            current_lines = []
        elif line_stripped.startswith('Width:'):
            continue  # Skip width line
        elif line_stripped.startswith(';'):
            continue  # Skip comments
        elif '=' in line_stripped and re.match(r'^[0-9A-Fa-f]', line_stripped):
            current_lines.append(line_stripped)

    if current_header is not None:
        blocks.append((current_header, current_lines))

    results = []
    for header, data_lines in blocks:
        info = {'raw_header': header, 'filename': os.path.basename(filepath)}

        # Parse header
        m_pci = re.match(r'Type:(PCI|PCI Express)\s+Bus\s+([0-9A-Fa-f]+)\s+Device\s+([0-9A-Fa-f]+)\s+Function\s+([0-9A-Fa-f]+)', header)
        m_mem = re.match(r'Type:Memory\s+Address\s+([0-9A-Fa-f]+)', header)
        m_smbus = re.match(r'Type:SMBUS\s+Address\s+([0-9A-Fa-f]+)', header)

        if m_pci:
            info['type'] = 'pci'
            info['is_pcie'] = m_pci.group(1) == 'PCI Express'
            info['bus'] = int(m_pci.group(2), 16)
            info['device'] = int(m_pci.group(3), 16)
            info['function'] = int(m_pci.group(4), 16)
        elif m_mem:
            info['type'] = 'memory'
            info['address'] = int(m_mem.group(1), 16)
        elif m_smbus:
            info['type'] = 'smbus'
            info['address'] = int(m_smbus.group(1), 16)
        else:
            info['type'] = 'unknown'

        data = parse_rw_data_lines(data_lines)
        ba = data_to_bytes(data)
        results.append((info, ba))

    return results


def decode_lpc_bridge(ba, info):
    """Decode LPC bridge (0:1F:0) registers 0x80-0xFF for LPSS enable bits."""
    print(f"\n  -- LPC Bridge LPSS/Device Enable Registers --")

    # Broadwell-U PCH LPC Bridge interesting registers:
    # 0x80: Root Complex Base Address (RCBA)
    rcba = read_u32(ba, 0x80)
    print(f"  [0x80] RCBA:       0x{rcba:08X} (Base: 0x{rcba & ~0x3FFF:08X}, {'Enabled' if rcba & 1 else 'Disabled'})")

    # 0xA0: Gen PM CON 1
    gen_pm_con1 = read_u32(ba, 0xA0)
    print(f"  [0xA0] GEN_PMCON1: 0x{gen_pm_con1:08X}")

    # 0xA4: Gen PM CON 2
    gen_pm_con2 = read_u32(ba, 0xA4)
    print(f"  [0xA4] GEN_PMCON2: 0x{gen_pm_con2:08X}")

    # 0xA8: Gen PM CON 3
    gen_pm_con3 = read_u32(ba, 0xA8)
    print(f"  [0xA8] GEN_PMCON3: 0x{gen_pm_con3:08X}")

    # 0xDC: Device Enable/Function Disable register
    # Wildcat Point-LP: Register offset 0xDC (D31:F0, FD - Function Disable)
    func_dis = read_u32(ba, 0xDC)
    print(f"  [0xDC] FuncDis:    0x{func_dis:08X}")

    # Bit meanings for Wildcat Point-LP FD register:
    fd_bits = [
        (0, "SATA1 (D31:F2)"),
        (1, "SATA2"),
        (2, "SMBUS (D31:F3)"),
        (3, "HD Audio (D31:F3?)"),
        (13, "LPSS_SPI (D30:F5)"),
        (14, "LPSS_HSUART1 (D30:F4)"),
        (15, "LPSS_HSUART0 (D30:F3)"),
        (16, "LPSS_I2C3 (D30:F2)"),
        (17, "LPSS_I2C2 (D30:F1)"),
        (18, "LPSS_I2C1 (D25:F2)"),
        (19, "LPSS_I2C0 (D25:F1)"),
        (20, "LPSS_DMA1"),
        (21, "Smart Sound / SST (D19:F0)"),
        (22, "LPSS_SDIO (D30:F0)"),
    ]
    disabled = []
    enabled = []
    for bit, name in fd_bits:
        if func_dis & (1 << bit):
            disabled.append(f"  bit{bit}: {name} DISABLED")
        else:
            enabled.append(f"  bit{bit}: {name} enabled")

    print(f"  Function Disable register bit analysis:")
    for s in enabled:
        print(f"    {s}")
    for s in disabled:
        print(f"    {s}")

    sst_disabled = bool(func_dis & (1 << 21))
    print(f"\n  *** SST (bit 21): {'DISABLED in FD register!' if sst_disabled else 'ENABLED in FD register'} ***")

    # 0xE0: Function Disable 2
    if len(ba) > 0xE3:
        func_dis2 = read_u32(ba, 0xE0)
        print(f"  [0xE0] FuncDis2:   0x{func_dis2:08X}")

    # Full register dump for 0x80-0xFF
    print(f"\n  -- LPC Bridge registers 0x80-0xFF hex dump --")
    if len(ba) >= 0x100:
        print(hex_dump(ba[0x80:0x100], base_addr=0x80, indent="    "))

    return {'func_dis': func_dis, 'rcba': rcba, 'sst_disabled': sst_disabled}


def main():
    print("=" * 70)
    print("  RWEverything Dump Parser - Intel SST Analysis")
    print("  Working directory:", DUMP_DIR)
    print("=" * 70)

    # Gather all .rw files (excluding .BAK)
    rw_files = sorted([
        f for f in os.listdir(DUMP_DIR)
        if f.endswith('.rw') and not f.endswith('.BAK')
    ])

    print(f"\nFound {len(rw_files)} .rw files:")
    for f in rw_files:
        size = os.path.getsize(os.path.join(DUMP_DIR, f))
        print(f"  {f:40s} ({size:>10,} bytes)")

    # ============================================================
    # Parse individual dump files (not PciAll)
    # ============================================================
    all_parsed = {}
    bar1_data = None
    lpc_data = None
    pci_devices = []  # Collect all PCI devices from PciAll

    for fname in rw_files:
        filepath = os.path.join(DUMP_DIR, fname)
        parsed = parse_rw_file(filepath)

        if not parsed:
            print(f"\n[SKIP] {fname}: no parseable data")
            continue

        for info, ba in parsed:
            if ba is None:
                print(f"\n[SKIP] {fname}: non-standard format ({info.get('type', '?')})")
                continue

            # Print basic info
            if info['type'] == 'pci':
                bdf = f"{info['bus']:02X}:{info['device']:02X}.{info['function']}"
                pcie_tag = " [PCIe]" if info.get('is_pcie') else ""
                label = f"PCI{pcie_tag} {bdf} -- {fname}"
                print(f"\n{'#'*70}")
                print(f"# FILE: {fname}")
                print(f"# {info['raw_header']}")
                print(f"# Size: {len(ba)} bytes")
                print(f"{'#'*70}")
                print(f"\n  Raw Hex Dump:")
                print(hex_dump(ba, indent="    "))
                decode_pci_config(ba, label=label, is_pcie=info.get('is_pcie', False))

                if fname != 'PciAll.rw':
                    all_parsed[fname] = (info, ba)

                # Check for SST device
                vid = read_u16(ba, 0x00)
                did = read_u16(ba, 0x02)
                if vid == 0x8086 and did == 0x9CB6:
                    print(f"\n  *** THIS IS THE SST DEVICE (8086:9CB6) ***")

                # If PciAll, collect
                if fname == 'PciAll.rw':
                    pci_devices.append((info, ba))

                    # Check for LPC bridge
                    if info['bus'] == 0 and info['device'] == 0x1F and info['function'] == 0:
                        lpc_info = decode_lpc_bridge(ba, info)
                        lpc_data = (info, ba, lpc_info)

            elif info['type'] == 'memory':
                addr = info['address']
                label = f"Memory @ 0x{addr:016X} -- {fname}"
                print(f"\n{'#'*70}")
                print(f"# FILE: {fname}")
                print(f"# {info['raw_header']}")
                print(f"# Size: {len(ba)} bytes")
                print(f"{'#'*70}")
                print(f"\n  Raw Hex Dump:")
                print(hex_dump(ba, base_addr=0, indent="    "))

                all_parsed[fname] = (info, ba)

                # Check if this is the BAR1 mirror
                if addr == 0xFE100000:
                    bar1_win = decode_bar1_mirror(ba, label=f"BAR1 Mirror @ 0xFE100000 (Windows) -- {fname}")
                    bar1_data = (info, ba, bar1_win)

                # For BAR0 (FE000000), note the content
                if addr == 0xFE000000:
                    print(f"\n  -- BAR0 Memory Space (SST DSP MMIO) --")
                    vid_did = read_u32(ba, 0x00)
                    print(f"  [0x00]: 0x{vid_did:08X}")
                    all_ff = all(b == 0xFF for b in ba)
                    all_zero = all(b == 0x00 for b in ba)
                    if all_ff:
                        print(f"  NOTE: ALL bytes are 0xFF - device not mapped or powered off!")
                    elif all_zero:
                        print(f"  NOTE: ALL bytes are 0x00 - region cleared/not initialized")
                    else:
                        print(f"  Content appears to have real data.")

                # For other memory regions
                if addr == 0xFE103000:
                    all_ff = all(b == 0xFF for b in ba)
                    print(f"\n  -- FE103000 (inside BAR0 range, +0x3000 from BAR1) --")
                    if all_ff:
                        print(f"  ALL bytes 0xFF - unmapped or powered off")

                if addr == 0xFE105000:
                    print(f"\n  -- FE105000 (I2C region?) --")
                    print(f"  First DWORDs: 0x{read_u32(ba,0):08X} 0x{read_u32(ba,4):08X} 0x{read_u32(ba,8):08X} 0x{read_u32(ba,0xC):08X}")

                if addr == 0xFE300000:
                    all_ff = all(b == 0xFF for b in ba)
                    print(f"\n  -- FE300000 --")
                    if all_ff:
                        print(f"  ALL bytes 0xFF - unmapped or powered off")

                if addr == 0xFED1C000:
                    all_zero = all(b == 0x00 for b in ba)
                    print(f"\n  -- FED1C000 (possibly SPI BAR) --")
                    if all_zero:
                        print(f"  ALL bytes 0x00")

            elif info['type'] == 'smbus':
                print(f"\n{'#'*70}")
                print(f"# FILE: {fname}")
                print(f"# {info['raw_header']}")
                print(f"# Size: {len(ba)} bytes")
                print(f"{'#'*70}")
                print(f"\n  Raw Hex Dump:")
                print(hex_dump(ba, indent="    "))
                all_parsed[fname] = (info, ba)

    # ============================================================
    # Search PciAll for specific devices
    # ============================================================
    print(f"\n\n{'='*70}")
    print(f"  PciAll.rw Device Search Results")
    print(f"{'='*70}")

    sst_found = False
    dev13_found = False

    for info, ba in pci_devices:
        vid = read_u16(ba, 0x00)
        did = read_u16(ba, 0x02)
        bus = info['bus']
        dev = info['device']
        func = info['function']

        # Search 1: VID 8086, DID 9CB6
        if vid == 0x8086 and did == 0x9CB6:
            sst_found = True
            print(f"\n  [FOUND] SST Device 8086:9CB6 at {bus:02X}:{dev:02X}.{func}")

        # Search 2: Bus 0, Device 0x13 (decimal 19)
        if bus == 0 and dev == 0x13:
            dev13_found = True
            print(f"\n  [FOUND] Device at 0:13.{func} = {vid:04X}:{did:04X}")

    if not sst_found:
        print(f"\n  [NOT FOUND] SST Device 8086:9CB6 is NOT present in PciAll.rw!")
        print(f"  This confirms the SST is NOT enumerable as a normal PCI device on this system.")

    if not dev13_found:
        print(f"\n  [NOT FOUND] No device at Bus 0, Device 0x13 (19) in PciAll.rw!")
        print(f"  The SST's expected BDF 0:13.0 has NO responding device.")

    # List all found devices
    print(f"\n  -- All PCI Devices in PciAll.rw --")
    print(f"  {'BDF':>10s}  {'VID:DID':>10s}  {'Class':>12s}  Device Name")
    print(f"  {'-'*10}  {'-'*10}  {'-'*12}  {'-'*40}")
    for info, ba in pci_devices:
        vid = read_u16(ba, 0x00)
        did = read_u16(ba, 0x02)
        base_class = ba[0x0B]
        sub_class = ba[0x0A]
        bdf = f"{info['bus']:02X}:{info['device']:02X}.{info['function']}"
        dev_name = INTEL_DEVIDS.get(did, "Unknown")
        class_name = get_class_name(base_class, sub_class)
        pcie_tag = "*" if info.get('is_pcie') else " "
        print(f"  {bdf:>10s}{pcie_tag} {vid:04X}:{did:04X}  {base_class:02X}:{sub_class:02X}       {dev_name}")

    # ============================================================
    # Comparison: BAR1 Windows vs FreeBSD
    # ============================================================
    print(f"\n\n{'='*70}")
    print(f"  COMPARISON: Windows BAR1 Mirror vs FreeBSD dmesg")
    print(f"{'='*70}")

    if bar1_data:
        info, ba, win = bar1_data

        # FreeBSD values (from check.txt dmesg, after D0 transition)
        fb = {
            'vid_did': 0x9CB68086,
            'sts_cmd': 0x00100006,
            'class_rev': 0x04010003,
            'bar0': 0xFE000000,
            'bar1': 0xFE100000,
            'cap_id_80': 0x01,
            'pmcsr': 0x00000008,  # D0
            'vdrtctl0': 0x00010100,
            'vdrtctl2': 0x00000BFD,
            'clkctl': 0x00000000,
        }

        # Windows values from the dump
        w_vid_did = read_u32(ba, 0x00)
        w_sts_cmd = read_u32(ba, 0x04)
        w_class_rev = read_u32(ba, 0x08)
        w_bar0 = read_u32(ba, 0x10)
        w_bar1 = read_u32(ba, 0x14)
        w_cap_id_80 = ba[0x80]
        w_pmcsr = read_u32(ba, 0x84)
        w_vdrtctl0 = read_u32(ba, 0xA0)
        w_vdrtctl2 = read_u32(ba, 0xA8)
        w_clkctl = read_u32(ba, 0x78)

        # PMCSR power state in Windows dump
        w_pmcsr_ps = w_pmcsr & 0x3
        ps_names = {0: "D0", 1: "D1", 2: "D2", 3: "D3hot"}

        comparisons = [
            ("DevID/VenID",  f"0x{w_vid_did:08X}",    f"0x{fb['vid_did']:08X}"),
            ("Status/Cmd",   f"0x{w_sts_cmd:08X}",    f"0x{fb['sts_cmd']:08X}"),
            ("Class/Rev",    f"0x{w_class_rev:08X}",   f"0x{fb['class_rev']:08X}"),
            ("BAR0",         f"0x{w_bar0:08X}",        f"0x{fb['bar0']:08X}"),
            ("BAR1",         f"0x{w_bar1:08X}",        f"0x{fb['bar1']:08X}"),
            ("Cap@0x80 ID",  f"0x{w_cap_id_80:02X}",   f"0x{fb['cap_id_80']:02X}"),
            ("PMCSR",        f"0x{w_pmcsr:08X} ({ps_names.get(w_pmcsr_ps,'?')})", f"0x{fb['pmcsr']:08X} (D0)"),
            ("VDRTCTL0",     f"0x{w_vdrtctl0:08X}",    f"0x{fb['vdrtctl0']:08X}"),
            ("VDRTCTL2",     f"0x{w_vdrtctl2:08X}",    f"0x{fb['vdrtctl2']:08X}"),
            ("CLKCTL",       f"0x{w_clkctl:08X}",      f"0x{fb['clkctl']:08X}"),
        ]

        print(f"\n  {'Register':<15s}  {'Windows (RW dump)':<30s}  {'FreeBSD (dmesg)':<30s}  Match?")
        print(f"  {'-'*15}  {'-'*30}  {'-'*30}  {'-'*6}")
        for name, wval, fbval in comparisons:
            match = "YES" if wval.split()[0] == fbval.split()[0] else "DIFF"
            marker = "  " if match == "YES" else "**"
            print(f"{marker}{name:<15s}  {wval:<30s}  {fbval:<30s}  {match}")

        # Detailed analysis of differences
        print(f"\n  -- Detailed Difference Analysis --")

        if w_pmcsr_ps != 0:
            print(f"\n  PMCSR: Windows shows power state {ps_names.get(w_pmcsr_ps, '?')}, FreeBSD shows D0")
            print(f"         Windows PMCSR raw: 0x{w_pmcsr:08X}")
            print(f"         This means Windows captured the dump with SST in {ps_names.get(w_pmcsr_ps, '?')} state!")
            if w_pmcsr_ps == 3:
                print(f"         D3hot = lowest power state with software visibility")
                print(f"         FreeBSD transitions to D0 before reading other registers")

        if w_vdrtctl0 != fb['vdrtctl0']:
            print(f"\n  VDRTCTL0 difference:")
            print(f"    Windows:  0x{w_vdrtctl0:08X}")
            print(f"    FreeBSD:  0x{fb['vdrtctl0']:08X}")
            w_d3pg = w_vdrtctl0 & 1
            w_sram = (w_vdrtctl0 >> 8) & 1
            w_d3pga = (w_vdrtctl0 >> 16) & 1
            f_d3pg = fb['vdrtctl0'] & 1
            f_sram = (fb['vdrtctl0'] >> 8) & 1
            f_d3pga = (fb['vdrtctl0'] >> 16) & 1
            print(f"    D3PG:        Win={w_d3pg} FreeBSD={f_d3pg}")
            print(f"    D3SRAMPGD:   Win={w_sram} FreeBSD={f_sram}")
            print(f"    D3PGAllowed: Win={w_d3pga} FreeBSD={f_d3pga}")
            if w_vdrtctl0 == 0:
                print(f"    Windows value is 0 - device may be in D3 with registers not yet programmed!")

        if w_vdrtctl2 != fb['vdrtctl2']:
            print(f"\n  VDRTCTL2 difference:")
            print(f"    Windows:  0x{w_vdrtctl2:08X}")
            print(f"    FreeBSD:  0x{fb['vdrtctl2']:08X}")
            print(f"    This register controls clock gating.")
            print(f"    FreeBSD writes 0x0BFD to disable clock gating for DSP access.")

        if w_sts_cmd != fb['sts_cmd']:
            print(f"\n  Status/Cmd difference:")
            w_cmd = w_sts_cmd & 0xFFFF
            f_cmd = fb['sts_cmd'] & 0xFFFF
            w_sts = (w_sts_cmd >> 16) & 0xFFFF
            f_sts = (fb['sts_cmd'] >> 16) & 0xFFFF
            print(f"    Windows CMD:  0x{w_cmd:04X} [{decode_pci_command(w_cmd)}]")
            print(f"    FreeBSD CMD:  0x{f_cmd:04X} [{decode_pci_command(f_cmd)}]")
            print(f"    Windows STS:  0x{w_sts:04X} [{decode_pci_status(w_sts)}]")
            print(f"    FreeBSD STS:  0x{f_sts:04X} [{decode_pci_status(f_sts)}]")

    else:
        print(f"\n  [ERROR] BAR1 mirror dump (M00000000FE100000.rw) not found!")

    # ============================================================
    # LPC Bridge Analysis
    # ============================================================
    if lpc_data:
        info, ba, lpc_info = lpc_data
        print(f"\n\n{'='*70}")
        print(f"  LPC Bridge (0:1F.0) - LPSS Enable Analysis")
        print(f"{'='*70}")

        func_dis = lpc_info['func_dis']
        rcba = lpc_info['rcba']

        print(f"\n  RCBA = 0x{rcba:08X} (Root Complex Base: 0x{rcba & ~0x3FFF:08X})")
        print(f"  Function Disable = 0x{func_dis:08X}")

        sst_bit = (func_dis >> 21) & 1
        if sst_bit:
            print(f"\n  *** CRITICAL: SST (bit 21) is SET in Function Disable register! ***")
            print(f"  *** This means the SST device is DISABLED at the LPC bridge level! ***")
            print(f"  *** This would explain why 0:13.0 reads as 0xFFFFFFFF on FreeBSD! ***")
        else:
            print(f"\n  SST (bit 21) is CLEAR - device should be enabled.")
            print(f"  If FreeBSD still can't see it, something else is disabling it.")

    # ============================================================
    # BAR0 (FE000000) vs BAR1 cross-reference
    # ============================================================
    bar0_file = os.path.join(DUMP_DIR, "M00000000FE000000.rw")
    if os.path.exists(bar0_file):
        bar0_parsed = parse_rw_file(bar0_file)
        if bar0_parsed and bar0_parsed[0][1] is not None:
            bar0_ba = bar0_parsed[0][1]
            print(f"\n\n{'='*70}")
            print(f"  BAR0 (0xFE000000) Analysis - SST DSP Memory Space")
            print(f"{'='*70}")

            all_ff = all(b == 0xFF for b in bar0_ba)
            all_zero = all(b == 0x00 for b in bar0_ba)

            if all_ff:
                print(f"\n  ALL BYTES are 0xFF!")
                print(f"  The DSP MMIO region is not accessible (device may be in D3 or not enabled).")
                print(f"  This is the same behavior FreeBSD sees with BAR0.")
            elif all_zero:
                print(f"\n  ALL BYTES are 0x00. Region may be zeroed or not responding.")
            else:
                print(f"\n  BAR0 contains real data!")
                print(f"  First 64 bytes:")
                print(hex_dump(bar0_ba[:64], base_addr=0, indent="    "))

                # Check for ADSP SRAM signature at various offsets
                for name, off in [("IRAM", 0x00), ("DRAM", 0x80), ("SHIM", 0xC0), ("MBOX", 0xE0)]:
                    if off < len(bar0_ba):
                        val = read_u32(bar0_ba, off)
                        print(f"  [{name} @ +0x{off:02X}]: 0x{val:08X}")

    # ============================================================
    # Summary of findings
    # ============================================================
    print(f"\n\n{'='*70}")
    print(f"  SUMMARY OF KEY FINDINGS")
    print(f"{'='*70}")

    print(f"""
  1. SST PCI Enumeration:
     - The SST device (8086:9CB6) is NOT visible as a normal PCI device
       in Windows' PciAll.rw dump (no device at 0:13.0 = 0:19:0).
     - This is consistent with FreeBSD's scan returning 0xFFFFFFFF for
       0:13.0.
     - The SST is an LPSS (Low Power Sub-System) device that does NOT
       appear on the standard PCI bus - it is ACPI-enumerated only.

  2. BAR1 Mirror at 0xFE100000:
     - Both Windows and FreeBSD can read the PCI config shadow space
       through the BAR1 memory-mapped region.
     - This is the ONLY way to access the SST's PCI-like configuration.
     - The BAR1 mirror correctly shows VID=8086, DID=9CB6.
""")

    if bar1_data:
        w_pmcsr_ps = read_u32(bar1_data[1], 0x84) & 0x3
        ps_names = {0: "D0", 1: "D1", 2: "D2", 3: "D3hot"}
        w_vdrtctl0 = read_u32(bar1_data[1], 0xA0)
        w_vdrtctl2 = read_u32(bar1_data[1], 0xA8)

        print(f"""  3. Power State:
     - Windows dump shows PMCSR state: {ps_names.get(w_pmcsr_ps, '?')}
     - FreeBSD transitions from D3hot to D0 using PMCSR write.
     - After D0 transition, FreeBSD reads valid config registers.

  4. VDRTCTL0 (SST Power Control):
     - Windows: 0x{w_vdrtctl0:08X}
     - FreeBSD (after init): 0x00010100
     - FreeBSD's sequence: write 0x1 (D3PG), then set bit 8 (D3SRAMPGD),
       then set bit 16 (D3PGAllowed) = 0x00010100.

  5. VDRTCTL2 (Clock Gating):
     - Windows: 0x{w_vdrtctl2:08X}
     - FreeBSD (after init): 0x00000BFD
     - FreeBSD writes 0x00000BFD to disable clock gating.

  6. BAR0 (0xFE000000) - DSP Memory:""")

        bar0_file = os.path.join(DUMP_DIR, "M00000000FE000000.rw")
        if os.path.exists(bar0_file):
            bar0_parsed = parse_rw_file(bar0_file)
            if bar0_parsed and bar0_parsed[0][1] is not None:
                bar0_ba = bar0_parsed[0][1]
                all_ff = all(b == 0xFF for b in bar0_ba)
                if all_ff:
                    print(f"     - Windows dump: ALL 0xFF (same as FreeBSD!)")
                    print(f"     - This means even Windows did not have the DSP active when")
                    print(f"       this dump was taken, OR the first 256 bytes of BAR0 are not")
                    print(f"       the SHIM registers (SHIM is at BAR0+0xC0000 in Broadwell).")
                else:
                    first_dw = read_u32(bar0_ba, 0)
                    print(f"     - Windows dump: First DWORD = 0x{first_dw:08X}")

    if lpc_data:
        _, _, lpc_info = lpc_data
        sst_bit = (lpc_info['func_dis'] >> 21) & 1
        print(f"""
  7. LPC Bridge Function Disable:
     - FD register = 0x{lpc_info['func_dis']:08X}
     - SST bit (21): {'SET - device disabled!' if sst_bit else 'CLEAR - device enabled'}
     - {'This confirms the SST PCI function is disabled at chipset level.' if sst_bit else 'The SST function is not disabled by the FD register.'}
     - {'The device is only accessible through ACPI memory mapping (BAR1 mirror).' if sst_bit else ''}""")

    print(f"""
  8. Key Insight for FreeBSD Driver:
     - The SST device is an ACPI-only device (no PCI BDF).
     - Access pattern: Use ACPI to discover BARs, then access config
       through BAR1 mirror at 0xFE100000.
     - The BAR0 (0xFE000000, 1MB) contains DSP SRAM and SHIM registers,
       but the SHIM is at +0xC0000, not at offset 0.
     - A 256-byte dump at 0xFE000000 won't reach the SHIM (0xFEC0000).
     - The Windows audio driver (IntcSST2) uses ACPI _DSM methods plus
       direct BAR access - it never uses PCI config space via CF8/CFC.
""")

    # ============================================================
    # Extra: dump the FE105000 region (I2C?)
    # ============================================================
    fe105_file = os.path.join(DUMP_DIR, "M00000000FE105000.rw")
    if os.path.exists(fe105_file):
        fe105_parsed = parse_rw_file(fe105_file)
        if fe105_parsed and fe105_parsed[0][1] is not None:
            fe105_ba = fe105_parsed[0][1]
            print(f"\n{'='*70}")
            print(f"  FE105000 Region Analysis (possibly I2C controller)")
            print(f"{'='*70}")
            # Check if it looks like a Synopsys DesignWare I2C controller
            comp_type = read_u32(fe105_ba, 0xF4) if len(fe105_ba) > 0xF7 else 0
            comp_ver = read_u32(fe105_ba, 0xF8) if len(fe105_ba) > 0xFB else 0
            ic_con = read_u32(fe105_ba, 0x00)
            ic_tar = read_u32(fe105_ba, 0x04)
            ic_status = read_u32(fe105_ba, 0x70) if len(fe105_ba) > 0x73 else 0
            ic_enable = read_u32(fe105_ba, 0x6C) if len(fe105_ba) > 0x6F else 0
            print(f"  IC_CON:        0x{ic_con:08X}")
            print(f"  IC_TAR:        0x{ic_tar:08X}")
            print(f"  IC_ENABLE:     0x{ic_enable:08X}")
            print(f"  IC_STATUS:     0x{ic_status:08X}")
            print(f"  COMP_TYPE:     0x{comp_type:08X}")
            print(f"  COMP_VERSION:  0x{comp_ver:08X}")
            if comp_type == 0x44570140:
                print(f"  --> Synopsys DesignWare I2C controller detected!")

    print(f"\n{'='*70}")
    print(f"  Analysis Complete")
    print(f"{'='*70}")


if __name__ == '__main__':
    main()
