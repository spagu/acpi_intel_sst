# Architecture & Process Diagrams

Mermaid diagrams for the Intel SST Audio Driver internals.

---

## Audio Pipeline Overview

```mermaid
graph TD
    A["Application (play, mpv, firefox...)"] --> B["/dev/dsp — sound(4)"]
    B --> C["acpi_intel_sst.ko"]
    C -->|"IPC (catpt)"| D["DSP Firmware (IntcSST2.bin)"]
    D -->|"DMA"| E["SSP0 — I2S 48kHz stereo"]
    E --> F["RT286 / ALC3263 codec (I2C0)"]
    F --> G["Speakers / Headphones"]

    style C fill:#AB2B28,color:#fff
    style D fill:#0071C5,color:#fff
    style F fill:#00A98F,color:#fff
```

---

## BAR0 Memory Map

```mermaid
block-beta
    columns 2
    block:ram["DSP Memory"]:2
        DRAM["DRAM 0x000000 — 0x0A0000 (640KB)\nFirmware data, mailbox, scratch"]
        IRAM["IRAM 0x0A0000 — 0x0F0000 (320KB)\nFirmware code"]
    end
    block:io["I/O Peripherals"]:2
        SHIM["SHIM 0x0FB000\nControl, IPC, clocks"]
        SSP0["SSP0 0x0FC000\nI2S port 0 (playback)"]
        SSP1["SSP1 0x0FD000\nI2S port 1 (capture)"]
        DMA0["DMA0 0x0FE000\nDW-DMAC ch0-7"]
    end

    style DRAM fill:#0071C5,color:#fff
    style IRAM fill:#0071C5,color:#fff
    style SHIM fill:#AB2B28,color:#fff
    style SSP0 fill:#00A98F,color:#fff
    style SSP1 fill:#00A98F,color:#fff
    style DMA0 fill:#8E24AA,color:#fff
```

---

## Driver Initialization Sequence

```mermaid
sequenceDiagram
    participant K as FreeBSD Kernel
    participant D as acpi_intel_sst
    participant FW as sst_firmware
    participant IPC as sst_ipc
    participant SSP as sst_ssp
    participant COD as sst_codec
    participant PCM as sst_pcm

    K->>D: acpi_intel_sst_probe()
    Note over D: Match INT3438 / INT33C8
    K->>D: acpi_intel_sst_attach()
    D->>D: Allocate BAR0 + IRQ
    D->>FW: sst_fw_init()
    D->>IPC: sst_ipc_init()
    Note over IPC: Mailbox in=0x000, out=0x400
    D->>SSP: sst_ssp_init()
    Note over SSP: SSP0 @0xFC000, SSP1 @0xFD000
    D->>D: sst_dma_init()
    D->>PCM: sst_pcm_init()
    D->>COD: sst_codec_init()
    D->>D: sst_topology_init()
    D->>D: sst_jack_init()
    D->>FW: sst_fw_load()
    FW->>FW: Parse $SST header
    FW->>FW: Write modules to IRAM/DRAM
    D->>FW: sst_fw_boot()
    FW-->>IPC: Wait FW_READY (250ms)
    IPC-->>FW: FW_READY + mailbox offsets
    D->>COD: sst_codec_enable_speaker()
    D->>PCM: sst_pcm_register()
    Note over D: State = RUNNING
```

---

## DSP Firmware Boot

```mermaid
sequenceDiagram
    participant H as Host (driver)
    participant S as SHIM registers
    participant DSP as DSP Core
    participant M as Mailbox (DRAM)

    H->>S: CSR |= STALL (bit 10)
    Note over DSP: DSP halted
    H->>H: Write firmware to IRAM/DRAM (32-bit MMIO)
    H->>H: Readback verify first/last words
    H->>S: Clear IPCX (= 0)
    H->>S: Clear ISRX doorbell bits
    H->>S: Unmask IPC interrupts (IMRX)
    H->>S: CSR &= ~STALL
    Note over DSP: DSP begins executing

    DSP->>M: Write FW_READY struct
    DSP->>S: IPCD = FW_READY | BUSY
    S-->>H: ISRX.IPCDB set
    H->>M: Read inbox/outbox offsets
    H->>S: ACK: clear BUSY, set DONE in IPCD
    Note over H: Update mailbox pointers
    H->>S: Re-enable DCLCGE (clock gating)
    Note over H: FW_STATE = RUNNING
```

---

## IPC Message Exchange

```mermaid
sequenceDiagram
    participant D as Driver
    participant X as IPCX register
    participant CD as IPCD register
    participant DSP as DSP Firmware
    participant M as Mailbox

    Note over D,DSP: Host → DSP command
    D->>M: Write payload to mbox_in
    D->>D: Readback verify (first 12 bytes)
    D->>X: Write header | BUSY (bit 31)
    D->>D: cv_timedwait (5s timeout)

    X-->>DSP: DSP reads command
    DSP->>DSP: Process command
    DSP->>M: Write reply to mbox_in
    DSP->>X: Clear BUSY, set DONE (bit 30)
    X-->>D: ISR: ISRX.IPCCD set

    D->>M: Copy reply from mbox_in
    D->>X: Clear DONE
    D->>D: cv_signal → wake caller
    Note over D: Extract status bits[4:0]
```

---

## PCM Playback Flow

```mermaid
sequenceDiagram
    participant App as Application
    participant SND as sound(4)
    participant PCM as sst_pcm
    participant IPC as sst_ipc
    participant DSP as DSP Firmware
    participant SSP as sst_ssp
    participant COD as sst_codec

    App->>SND: write(/dev/dsp, audio_data)
    SND->>PCM: sst_chan_trigger(START)

    PCM->>IPC: ALLOC_STREAM
    Note over IPC: type=SYSTEM, path=SSP0_OUT<br/>format=48kHz/16bit/2ch<br/>page table PFNs
    IPC->>DSP: IPC command
    DSP-->>IPC: stream_hw_id + pos_register

    PCM->>IPC: RESET stream
    PCM->>IPC: PAUSE stream

    PCM->>PCM: Enable HMDC (host DMA access)
    PCM->>IPC: RESUME stream
    Note over DSP: DSP reads ring buffer via DMA

    PCM->>SSP: sst_ssp_start()
    Note over SSP: SSCR0.SSE = 1<br/>BCLK + LRCLK active

    PCM->>COD: sst_codec_pll_rearm()
    Note over COD: PLL locks to BCLK

    loop Every 5ms
        PCM->>PCM: Poll position register
        PCM->>SND: chn_intr() on block boundary
        SND->>App: Unblock write()
    end

    App->>SND: close()
    SND->>PCM: sst_chan_trigger(STOP)
    PCM->>SSP: sst_ssp_stop()
    PCM->>IPC: PAUSE → RESET → FREE stream
```

---

## RT286 Codec Initialization

```mermaid
sequenceDiagram
    participant D as Driver
    participant I2C as I2C0 (0xFE103000)
    participant RT as RT286 (addr 0x1C)

    Note over D,I2C: I2C controller setup
    D->>I2C: Wake from D3 → D0 (PMCSR)
    D->>I2C: Disable controller
    D->>I2C: Master mode, 400kHz, 7-bit
    D->>I2C: Set target = 0x1C
    D->>I2C: Enable controller

    Note over D,RT: Codec identification
    D->>RT: Read vendor ID (NID 0x00)
    RT-->>D: 0x10EC0286

    Note over D,RT: Index register defaults
    D->>RT: Power control (0x01, 0x02, 0x03)
    D->>RT: I2S control (0x09, 0x0A)
    D->>RT: DC gain calibration (0x0D)
    D->>RT: PLL control (0x49, 0x63) — disabled
    D->>RT: Depop controls (0x67, 0x68, 0x69)
    D->>RT: Dell GPIO6 (speaker amp)

    Note over D,RT: Speaker enable
    D->>RT: AFG → D0 (PLL mode)
    D->>RT: I2S slave mode (0xD010)
    D->>RT: Enable PLL (0x2906)
    D->>RT: DAC0, Mixer, SPK pin → D0
    D->>RT: DAC format: 48kHz/16bit/2ch
    D->>RT: Unmute mixer, select front
    D->>RT: Unmute speaker amp (0dB)
    D->>RT: Enable EAPD
```

---

## SSP/I2S Configuration

```mermaid
graph LR
    subgraph SSP0["SSP0 — Playback Port"]
        SSCR0["SSCR0\nFRF=PSP, DSS=15\nSCR=divider, MOD=1"]
        SSCR1["SSCR1\nTFT=8, RFT=8\nTSRE+RSRE (DMA)\nSCLKDIR+SFRMDIR"]
        SSPSP["SSPSP\nSCMODE=0 (I2S)\nSFRMWDTH=16\nSTRTDLY=1"]
    end

    DSP["DSP DMA"] -->|"Audio data"| SSP0
    SSP0 -->|"BCLK + LRCLK + DATA"| CODEC["RT286 Codec"]

    style DSP fill:#0071C5,color:#fff
    style CODEC fill:#00A98F,color:#fff
```

```mermaid
sequenceDiagram
    participant D as Driver
    participant SSP as SSP0 Registers

    D->>SSP: Clear SSCR0.SSE (disable)
    D->>SSP: SSCR0 = PSP | DSS(15) | SCR | MOD | FRDC(2)
    D->>SSP: SSCR1 = TFT(8) | RFT(8) | TSRE | RSRE | TRAIL
    D->>SSP: SSPSP = SCMODE(0) | SFRMP | FSRT | SFRMWDTH(16) | STRTDLY(1)
    D->>SSP: SSTSA = time slot assignment
    Note over D,SSP: Port configured, waiting for start

    D->>SSP: Set SSCR0.SSE (enable)
    Note over SSP: BCLK + LRCLK active
```

---

## DMA & Page Table

```mermaid
graph TD
    subgraph Host["Host Memory"]
        BUF["PCM Ring Buffer (64KB)\nPhysical pages"]
        PT["Page Table\n20-bit PFNs packed\n2 per 5 bytes"]
    end

    subgraph DSP["DSP"]
        HMDC["SHIM.HMDC\nHost Memory DMA Control"]
        ENGINE["DSP DMA Engine"]
    end

    PT -->|"ALLOC_STREAM IPC"| ENGINE
    HMDC -->|"Enable HDDA bits"| ENGINE
    ENGINE -->|"Read pages"| BUF
    ENGINE -->|"Audio data"| SSP["SSP0 (I2S)"]

    style HMDC fill:#AB2B28,color:#fff
    style ENGINE fill:#0071C5,color:#fff
    style SSP fill:#00A98F,color:#fff
```

---

## Module Detach / Cleanup

```mermaid
sequenceDiagram
    participant K as Kernel
    participant D as acpi_intel_sst
    participant PCM as sst_pcm
    participant COD as sst_codec
    participant FW as sst_firmware

    K->>D: acpi_intel_sst_detach()
    D->>PCM: sst_pcm_detach()
    Note over PCM: Stop polling, free DMA buffers
    D->>D: sst_jack_detach()
    D->>COD: sst_codec_detach()
    Note over COD: Mute speaker, power down codec
    D->>D: sst_topology_detach()
    D->>D: sst_dma_detach()
    D->>D: sst_ssp_detach()
    D->>D: sst_ipc_detach()
    D->>FW: sst_fw_detach()
    Note over FW: Stall DSP, release firmware
    D->>D: Release IRQ + BAR0
    D->>D: Destroy mutex
```

---

## Jack Detection

```mermaid
graph TD
    A["RT286 Combo Jack"] -->|"GPIO / Unsolicited Response"| B["sst_jack_poll()"]
    B --> C{"Jack state?"}
    C -->|"Inserted"| D["Detect type\n(headphone / headset)"]
    C -->|"Removed"| E["Route to speakers"]
    D -->|"Headphone"| F["Route to HP out\nMute speakers"]
    D -->|"Headset"| G["Route to HP out\nEnable mic input"]

    style A fill:#00A98F,color:#fff
    style B fill:#AB2B28,color:#fff
```
