# Implementation Plan: Phase 2-3

## Overview

This document outlines the detailed implementation plan for completing Phase 2 (DSP Init refinement) and Phase 3 (Firmware & IPC) of the Intel SST driver for FreeBSD.

---

## Phase 2: DSP Initialization (Refinement)

### 2.1 Interrupt Handler

**Status:** IRQ allocated but no handler installed

**Tasks:**
```c
/* Add to sst_softc */
struct mtx          sc_mtx;         /* Mutex for register access */
volatile uint32_t   ipc_pending;    /* Pending IPC messages */
```

**Implementation:**
1. Add mutex initialization in `sst_acpi_attach()`
2. Implement `sst_intr()` handler
3. Setup interrupt with `bus_setup_intr()`

**Code location:** `acpi_intel_sst.c:200-220`

### 2.2 Power Management

**Status:** Basic D0/D3 switching exists

**Tasks:**
- Add suspend/resume handlers
- Implement proper power sequencing

**Implementation:**
```c
static int sst_acpi_suspend(device_t dev);
static int sst_acpi_resume(device_t dev);

/* Add to device_method_t */
DEVMETHOD(device_suspend, sst_acpi_suspend),
DEVMETHOD(device_resume,  sst_acpi_resume),
```

### 2.3 Register Access Helpers

**Tasks:**
- Thread-safe register read/write
- Bit manipulation helpers

**Implementation:**
```c
static inline uint32_t
sst_shim_read(struct sst_softc *sc, uint32_t reg)
{
    return bus_read_4(sc->mem_res, reg);
}

static inline void
sst_shim_write(struct sst_softc *sc, uint32_t reg, uint32_t val)
{
    bus_write_4(sc->mem_res, reg, val);
}

static inline void
sst_shim_update_bits(struct sst_softc *sc, uint32_t reg,
                     uint32_t mask, uint32_t val)
{
    uint32_t old, new;

    mtx_lock(&sc->sc_mtx);
    old = sst_shim_read(sc, reg);
    new = (old & ~mask) | (val & mask);
    sst_shim_write(sc, reg, new);
    mtx_unlock(&sc->sc_mtx);
}
```

---

## Phase 3: Firmware & IPC

### 3.1 Firmware Format (Haswell/Broadwell SST)

**Format:** Intel SST Binary (not SOF)

**Header structure (from Linux `sound/soc/intel/common/sst-firmware.c`):**

```c
/* SST Firmware Header */
struct sst_fw_header {
    char    signature[4];       /* "SST\0" */
    uint32_t file_size;         /* Total file size */
    uint32_t modules;           /* Number of modules */
    uint32_t file_format;       /* File format version */
    uint32_t reserved[4];
};

/* SST Module Header */
struct sst_module_header {
    char    signature[4];       /* "MOD\0" */
    uint32_t mod_size;          /* Module size including header */
    uint32_t blocks;            /* Number of blocks */
    uint16_t slot;              /* DSP slot index */
    uint16_t type;              /* Module type */
    uint32_t entry_point;       /* Entry point address */
};

/* SST Block Header */
struct sst_block_header {
    uint32_t type;              /* Block type (IRAM/DRAM/etc) */
    uint32_t size;              /* Block data size */
    uint32_t ram_offset;        /* Offset in DSP RAM */
    uint32_t reserved;
};
```

### 3.2 Memory Regions (Broadwell-U)

**DSP Memory Map:**
| Region | Offset | Size | Description |
|--------|--------|------|-------------|
| IRAM | 0x00000 | 64KB | Instruction RAM |
| DRAM | 0x10000 | 64KB | Data RAM |
| SHIM | 0x40000 | 4KB | Control registers |
| MBOX | 0x44000 | 4KB | Mailbox (IPC) |

**Add to header:**
```c
/* Memory regions (Broadwell-U) */
#define SST_IRAM_OFFSET     0x00000
#define SST_IRAM_SIZE       0x10000     /* 64KB */
#define SST_DRAM_OFFSET     0x10000
#define SST_DRAM_SIZE       0x10000     /* 64KB */
#define SST_SHIM_OFFSET     0x40000
#define SST_SHIM_SIZE       0x1000      /* 4KB */
#define SST_MBOX_OFFSET     0x44000
#define SST_MBOX_SIZE       0x1000      /* 4KB */
```

### 3.3 Firmware Loader

**File: `sst_firmware.c` (new)**

**Tasks:**
1. Load firmware from `/boot/firmware/intel-sst-bdw.bin`
2. Parse and validate header
3. Copy blocks to DSP memory
4. Store entry point

**Implementation:**
```c
#include <sys/firmware.h>

#define SST_FW_NAME "intel/sst-bdw"

struct sst_firmware {
    const struct firmware   *fw;        /* Firmware handle */
    uint32_t                entry_point; /* DSP entry address */
    bool                    loaded;
};

int sst_fw_load(struct sst_softc *sc);
void sst_fw_unload(struct sst_softc *sc);
static int sst_fw_parse(struct sst_softc *sc, const uint8_t *data, size_t len);
static int sst_fw_load_block(struct sst_softc *sc, struct sst_block_header *blk,
                             const uint8_t *data);
```

**Boot sequence:**
```c
int
sst_dsp_boot(struct sst_softc *sc)
{
    int error;

    /* 1. Reset DSP (already done) */
    sst_reset(sc);

    /* 2. Load firmware */
    error = sst_fw_load(sc);
    if (error)
        return error;

    /* 3. Clear reset, keep stall */
    sst_shim_update_bits(sc, SST_SHIM_CSR,
                         SST_CSR_RST, 0);
    DELAY(1000);

    /* 4. Clear stall -> DSP runs */
    sst_shim_update_bits(sc, SST_SHIM_CSR,
                         SST_CSR_STALL, 0);

    /* 5. Wait for firmware ready (via IPC) */
    error = sst_ipc_wait_ready(sc, 5000); /* 5sec timeout */

    return error;
}
```

### 3.4 IPC Protocol

**File: `sst_ipc.c` (new)**

**IPC Message Format (Haswell/Broadwell):**
```c
/* IPC Header (32-bit) */
#define SST_IPC_HEADER(type, size)  \
    (((type) << 24) | ((size) << 16))

/* IPC Types */
#define SST_IPC_GLBL_REPLY          0x00
#define SST_IPC_GLBL_GET_FW_VERSION 0x01
#define SST_IPC_GLBL_STREAM_MESSAGE 0x03
#define SST_IPC_GLBL_DEBUG_INFO     0x04

/* Stream Messages */
#define SST_IPC_STR_ALLOC           0x10
#define SST_IPC_STR_FREE            0x11
#define SST_IPC_STR_START           0x12
#define SST_IPC_STR_STOP            0x13
#define SST_IPC_STR_PAUSE           0x14
```

**IPC Software Context:**
```c
struct sst_ipc {
    struct mtx          lock;
    struct cv           wait_cv;        /* Condition variable */
    uint32_t            pending_msg;
    bool                ready;          /* FW ready flag */

    /* Mailbox */
    bus_addr_t          mbox_in;        /* Host -> DSP */
    bus_addr_t          mbox_out;       /* DSP -> Host */
};
```

**Core IPC Functions:**
```c
/* Initialize IPC subsystem */
int sst_ipc_init(struct sst_softc *sc);
void sst_ipc_fini(struct sst_softc *sc);

/* Send message to DSP */
int sst_ipc_send(struct sst_softc *sc, uint32_t header,
                 void *data, size_t len);

/* Receive message from DSP */
int sst_ipc_recv(struct sst_softc *sc, uint32_t *header,
                 void *data, size_t *len);

/* Wait for DSP ready */
int sst_ipc_wait_ready(struct sst_softc *sc, int timeout_ms);

/* Interrupt handler (called from sst_intr) */
void sst_ipc_intr(struct sst_softc *sc);
```

**IPC Send Implementation:**
```c
int
sst_ipc_send(struct sst_softc *sc, uint32_t header,
             void *data, size_t len)
{
    int error = 0;

    mtx_lock(&sc->ipc.lock);

    /* Check if DSP is busy */
    if (sst_shim_read(sc, SST_SHIM_IPCX) & SST_IPC_BUSY) {
        error = EBUSY;
        goto done;
    }

    /* Write data to mailbox */
    if (data && len > 0) {
        bus_write_region_1(sc->mem_res, SST_MBOX_OFFSET,
                          data, len);
    }

    /* Write header with BUSY bit */
    sst_shim_write(sc, SST_SHIM_IPCX, header | SST_IPC_BUSY);

    /* Wait for DONE (with timeout) */
    error = cv_timedwait(&sc->ipc.wait_cv, &sc->ipc.lock,
                         hz * 2); /* 2 second timeout */

done:
    mtx_unlock(&sc->ipc.lock);
    return error;
}
```

---

## File Structure After Phase 3

```
acpi_intel_sst/
├── acpi_intel_sst.c      # Main driver (attach/detach/intr)
├── acpi_intel_sst.h      # Public structures and defines
├── sst_firmware.c        # Firmware loader
├── sst_firmware.h        # Firmware structures
├── sst_ipc.c             # IPC protocol
├── sst_ipc.h             # IPC structures
├── sst_regs.h            # All register definitions
└── Makefile              # Updated with new files
```

**Updated Makefile:**
```makefile
KMOD=   acpi_intel_sst
SRCS=   acpi_intel_sst.c sst_firmware.c sst_ipc.c
SRCS+=  bus_if.h device_if.h acpi_if.h

.include <bsd.kmod.mk>
```

---

## Implementation Order

### Week 1: Phase 2 Completion
| Day | Task | Files |
|-----|------|-------|
| 1-2 | Add mutex, interrupt handler | acpi_intel_sst.c |
| 3 | Register access helpers | acpi_intel_sst.h |
| 4-5 | Suspend/resume handlers | acpi_intel_sst.c |

### Week 2: Firmware Loader
| Day | Task | Files |
|-----|------|-------|
| 1-2 | Define firmware structures | sst_firmware.h |
| 3-4 | Implement firmware parser | sst_firmware.c |
| 5 | Implement block loader | sst_firmware.c |

### Week 3: IPC Protocol
| Day | Task | Files |
|-----|------|-------|
| 1-2 | Define IPC structures | sst_ipc.h |
| 3-4 | Implement send/recv | sst_ipc.c |
| 5 | Interrupt-driven IPC | sst_ipc.c |

### Week 4: Integration & Testing
| Day | Task | Files |
|-----|------|-------|
| 1-2 | DSP boot sequence | acpi_intel_sst.c |
| 3-4 | Get firmware version test | All |
| 5 | Debug and documentation | docs/ |

---

## Firmware Acquisition

Intel SST firmware for Broadwell is proprietary. Options:

1. **Extract from Linux firmware package:**
   ```bash
   # On Linux system
   ls /lib/firmware/intel/fw_sst_0f28.bin
   # Copy to FreeBSD: /boot/firmware/intel/sst-bdw.bin
   ```

2. **Download from linux-firmware:**
   ```
   https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/tree/intel
   ```

3. **SOF alternative (open source):**
   - Sound Open Firmware has BSD-licensed firmware
   - Different format, requires SOF-specific loader
   - https://github.com/thesofproject/sof-bin

---

## Testing Checkpoints

### Phase 2 Complete
- [ ] Module loads without panic
- [ ] Interrupt handler registered
- [ ] Suspend/resume works (kldunload/kldload)
- [ ] No mutex warnings in dmesg

### Phase 3 Complete
- [ ] Firmware file found and loaded
- [ ] DSP exits reset state
- [ ] IPC "get version" returns valid response
- [ ] No timeouts or hangs

---

## References

- Linux kernel: `sound/soc/intel/haswell/`
- Linux kernel: `sound/soc/intel/common/sst-firmware.c`
- Linux kernel: `sound/soc/intel/common/sst-ipc.c`
- FreeBSD: `sys/dev/sound/pci/hda/` (for sound(4) patterns)
