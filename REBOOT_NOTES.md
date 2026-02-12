# SST Driver - Stan przed rebootem (2026-02-12 sesja 2)

## Trzy naprawione bugi w tej sesji

### Bug 1: SRAM Power Gate Inversion (KRYTYCZNY - root cause BAR0 dead)
- `sst_wpt_power_up()` linia ~278: `|=` zmienione na `&= ~` dla ISRAMPGE/DSRAMPGE
- To samo w IOBP probe linia ~1350
- Stary kod WYŁĄCZAŁ SRAM zamiast go włączać (logika odwrócona)
- Dowód: VDRTCTL0 initial=0x00000001 (SRAM ON) -> po buggy code 0x000FFFFF (SRAM OFF!)
- Po naprawie: VDRTCTL0 = 0x00000003 (D3PGD + D3SRAMPGD, SRAM PGE clear = SRAM ON)

### Bug 2: PCI BAR Relocation (nowy Phase 0 fixup w attach)
- Po HDA enable + PCI rescan, BARy ADSP były przenoszone z 0xFE000000 na 0xDF800000
- Dodano Phase 0: czyta PCI config, wykrywa mismatch, przywraca BARy do adresów ACPI
- Potwierdzone: BAR1 teraz czyta poprawnie (VDRTCTL0 != 0xFFFFFFFF)

### Bug 3: Brak hw.acpi.install_interface w loader.conf
- OSYS = 0x07d9 (Windows 7) -> DSDT nie inicjalizuje LPSS fabric
- Dodano `hw.acpi.install_interface="Windows 2012"` do /boot/loader.conf
- DSDT wymaga OSYS >= 0x07DC żeby routować transakcje do BAR0

## KLUCZOWE ODKRYCIE: RT286 codec na I2C0

- **INT343A na I2C0 (iicbus6, addr 0x1c) = RT286 audio codec** ("Intel Smart Sound Technology Audio Codec")
- **0x2c na I2C1 to touchpad Dell (DLL0665), NIE kodek!**
- Driver probe'ował **zły bus i zły adres** — trzeba celować w I2C0 / iicbus6 / 0x1c

## Dlaczego HDA nie działa (dead end)

- hdac1 at PCI 0:1B.0 - controller działa, CORB/RIRB OK
- Scanning HDA codecs -> ZERO codecs found
- RT286/ALC3263 jest na I2S bus (przez SST DSP), NIE na HDA link
- BIOS skonfigurował hardware na I2S mode
- Jedyna droga do audio = SST DSP path (BAR0 + firmware + I2S)

## Co powinno się stać po reboot

Po raz PIERWSZY będziemy mieli jednocześnie:
1. OSYS >= 0x07DC (LPSS fabric enabled by DSDT)
2. SRAM powered ON (PGE bits cleared correctly)
3. BAR addresses correct (fixup restores if needed)

Historia problemów - nigdy nie mieliśmy wszystkich trzech:
| Boot | OSYS | LPSS fabric | SRAM | BAR0 |
|------|------|-------------|------|------|
| Stary | 0x07d9 | DEAD | N/A | DEAD |
| Z custom DSDT | 0x07dd | ALIVE | OFF (bug!) | DEAD |
| Dzisiejszy fix | 0x07d9 | nie zainicjowany | ON (fixed) | DEAD |
| **Po reboot** | **0x07dd** | **ALIVE** | **ON** | **???** |

BAR0 ACCESSIBLE -> driver wejdzie w `dsp_init`:
- Allocate IRQ + interrupt handler
- IPC init (mailbox)
- Firmware load (IntcSST2.bin)
- DMA init (DesignWare DMA at BAR0+0x98000)
- SSP/I2S init (port 0 + port 1)
- PCM register with sound(4)
- Jack detection

## Pliki zmienione

- `/root/acpi_intel_sst/acpi_intel_sst.c` - SRAM fix + BAR fixup (Phase 0)
- `/boot/loader.conf` - dodano hw.acpi.install_interface="Windows 2012"
- `/boot/modules/acpi_intel_sst.ko` - zainstalowany naprawiony moduł

## Co sprawdzić po reboot

```bash
# 1. OSYS powinien być 0x07dd
dmesg | grep OSYS

# 2. BAR0 test powinien być ACCESSIBLE
dmesg | grep 'BAR0 test'

# 3. Pełny output drivera
dmesg | grep 'acpi_intel_sst'

# 4. Audio devices
cat /dev/sndstat

# 5. Jeśli BAR0 ACCESSIBLE ale firmware failed:
ls /boot/firmware/IntcSST2.bin
ls /usr/local/share/firmware/intel/IntcSST2.bin
```

## Następne kroki po reboot (jeśli BAR0 działa)

1. **Firmware**: potrzebny IntcSST2.bin (z linux-firmware lub Windows driver)
2. **DSP boot**: załadować firmware do IRAM/DRAM, wystartować DSP
3. **IPC**: komunikacja z DSP (GET_FW_VERSION, ALLOCATE_STREAM)
4. **I2S/SSP**: konfiguracja portów audio
5. **RT286 codec**: konfiguracja przez I2C (routing, gains, power)
6. **PCM**: rejestracja z sound(4) -> /dev/dsp, /dev/sndstat

## Git - niezacommitowane zmiany

```
acpi_intel_sst.c:
  - Phase 0: PCI BAR fixup (nowy blok w attach)
  - sst_wpt_power_up: &= ~ zamiast |= dla SRAM PGE
  - sst_iobp_probe: &= ~ zamiast |= dla SRAM PGE

/boot/loader.conf:
  - hw.acpi.install_interface="Windows 2012"
```
