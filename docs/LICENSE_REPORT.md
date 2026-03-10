# License Report — acpi_intel_sst

**Generated:** 2026-03-10
**Tool:** [Trivy](https://trivy.dev/) v0.69.3 (full license scan)
**Repository:** `git@github.com:spagu/acpi_intel_sst.git`
**Branch:** `main`
**Commit:** `24cadab`

---

## Summary

| License | Category | Severity | Files |
|---------|----------|----------|-------|
| BSD-3-Clause | Notice | LOW | 28 |
| BSD-2-Clause | Notice | LOW | 3 (Trivy partial detection — see note) |
| Intel Binary Firmware License | Proprietary | — | 2 (firmware binaries) |
| Newlib (multi-license) | Mixed | — | firmware embedded |

> **Note:** Trivy detected BSD-2-Clause in 3 files (`acpi_intel_sst.h`, `sst_codec.c`, `sst_codec.h`) with confidence ~0.978. Manual verification of SPDX headers confirms all these files declare `SPDX-License-Identifier: BSD-3-Clause`. The discrepancy arises because the full license text in these files contains only 2 clauses (missing the endorsement clause), even though the SPDX identifier states BSD-3-Clause.

---

## Trivy Scan Results (raw)

### Loose File Licenses

| File | License | Confidence | Severity | Category |
|------|---------|------------|----------|----------|
| `LICENSE` | BSD-3-Clause | 100% | LOW | notice |
| `CHANGELOG.md` | Copyright | 100% | UNKNOWN | unknown |
| `src/acpi_intel_sst.h` | BSD-2-Clause | 97.8% | LOW | notice |
| `src/sst_codec.c` | BSD-2-Clause | 97.8% | LOW | notice |
| `src/sst_codec.h` | BSD-2-Clause | 97.8% | LOW | notice |

---

## Manual Verification — Source Files

All source files in `src/` contain the following SPDX header:

```
SPDX-License-Identifier: BSD-3-Clause
Copyright (c) 2026 FreeBSD Foundation
```

### Implementation files (.c)

| File | SPDX | Copyright |
|------|------|-----------|
| `src/acpi_intel_sst.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_codec.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_dma.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_firmware.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_ipc.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_jack.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_pch.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_pcm.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_power.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_sram.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_ssp.c` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_topology.c` | BSD-3-Clause | 2026 FreeBSD Foundation |

### Header files (.h)

| File | SPDX | Copyright |
|------|------|-----------|
| `src/acpi_intel_sst.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_codec.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_dma.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_firmware.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_ipc.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_jack.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_pcm.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_regs.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_ssp.h` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `src/sst_topology.h` | BSD-3-Clause | 2026 FreeBSD Foundation |

### Generated headers (no license header)

| File | Status |
|------|--------|
| `src/acpi_if.h` | Generated (FreeBSD build) |
| `src/bus_if.h` | Generated (FreeBSD build) |
| `src/channel_if.h` | Generated (FreeBSD build) |
| `src/device_if.h` | Generated (FreeBSD build) |
| `src/mixer_if.h` | Generated (FreeBSD build) |
| `src/opt_acpi.h` | Generated (FreeBSD build) |
| `src/opt_global.h` | Generated (FreeBSD build) |
| `src/opt_snd.h` | Generated (FreeBSD build) |
| `src/pci_if.h` | Generated (FreeBSD build) |

### Scripts

| File | SPDX | Copyright |
|------|------|-----------|
| `scripts/sst_report.sh` | BSD-3-Clause | 2026 FreeBSD Foundation |
| `tests/test_module.sh` | BSD-3-Clause | 2026 FreeBSD Foundation |

---

## Firmware — Third-Party Licenses

Firmware binaries (`firmware/ubuntu/`, `firmware/windows/`) are covered by separate licenses:

### Intel Binary Firmware Release License

- **License files:** `firmware/ubuntu/LICENSE.IntcSST2`, `firmware/windows/LICENSE.IntcSST2`, `firmware/windows/LICENSE.IntcSST1`
- **Applies to:** `IntcSST2.bin`, `IntcSST1.bin`
- **Type:** Proprietary (binary-only redistribution)
- **Conditions:**
  - Redistribution permitted only in binary form (no modification)
  - No reverse engineering, decompilation, or disassembly
  - Must retain copyright notice

### Embedded Newlib Licenses (in firmware)

The firmware contains newlib components under the following licenses:
1. **Red Hat** — BSD License
2. **University of California, Berkeley** — BSD-3-Clause
3. **David M. Gay / AT&T / Lucent** — permissive (custom)
4. **Advanced Micro Devices** — permissive (custom)

---

## Compatibility Summary

| Component | License | Compatibility |
|-----------|---------|---------------|
| Source code (src/) | BSD-3-Clause | Permissive, GPL-compatible |
| Project (root LICENSE) | BSD-3-Clause | Permissive, GPL-compatible |
| Firmware binaries | Intel Proprietary | Binary-only, restricted redistribution |
| Firmware newlib | BSD / permissive mix | Permissive |

**Project license: BSD-3-Clause**
**Note:** Intel firmware binaries have redistribution restrictions — they cannot be modified or decompiled.
