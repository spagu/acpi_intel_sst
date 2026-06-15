# Raport o Błędach (Bug Report) - Intel SST Audio Driver (FreeBSD)

W tym raporcie przedstawiono trzy zidentyfikowane problemy w kodzie źródłowym sterownika `acpi_intel_sst`, które mogą prowadzić do błędów typu **kernel panic**, zawieszenia systemu przy wybudzaniu (resume), braku komunikacji z procesorem sygnałowym (DSP) lub uszkodzenia pamięci systemowej.

---

## 1. Kernel Panic podczas Suspend/Resume w trybie PCI (NULL Pointer Dereference)

### Opis błędu
Podczas dołączania urządzenia w trybie PCI (funkcja `sst_pci_attach` w pliku [acpi_intel_sst.c](file:///home/rabczukr/github.com/spagu/acpi_intel_sst/src/acpi_intel_sst.c#L1140)), wskaźnik do ACPI `sc->handle` jest ustawiany na `NULL` (linia 1151).
Gdy system przechodzi w stan uśpienia (suspend) lub wznawia działanie (resume), jądro wywołuje powiązane metody `device_suspend` i `device_resume`, które są zmapowane na `sst_acpi_suspend` i `sst_acpi_resume`.
Funkcje te wywołują `acpi_pwr_switch_consumer(sc->handle, ...)` bez uprzedniego sprawdzenia, czy `sc->handle` nie jest `NULL` (linie 1533 oraz 1549). Wywołanie tej funkcji ACPI z parametrem `NULL` powoduje natychmiastowe dereferencjonowanie wskaźnika NULL w subsystemie ACPI, co wywołuje błąd stronicowania w jądrze (page fault) i prowadzi do **kernel panic**.

Dodatkowo, podczas wznawiania działania w trybie PCI (`sst_acpi_resume`), sterownik nie wywołuje funkcji `sst_wpt_power_up(sc)`, która inicjuje zasilanie DSP i włącza pamięć SRAM (w przeciwieństwie do trybu ACPI, gdzie robi to funkcja `acpi_pwr_switch_consumer`). Przez to, nawet gdyby udało się uniknąć paniki, urządzenie PCI nie zostałoby prawidłowo wybudzone.

### Sugerowane rozwiązanie
Należy dodać sprawdzenie `sc->handle != NULL` przed wywołaniem funkcji `acpi_pwr_switch_consumer`. W trybie PCI (gdy handle jest równy NULL), zawieszenie powinno wywoływać `sst_wpt_power_down(sc)`, a wybudzenie powinno wywoływać `sst_wpt_power_up(sc)` (która dodatkowo odpytuje i przywraca stan rejestrów sterujących PCI/SRAM).

#### Proponowany Patch (Diff):
```diff
--- src/acpi_intel_sst.c
+++ src/acpi_intel_sst.c
@@ -1530,7 +1530,10 @@
 
 	/* 6. Reset DSP and power off */
 	sst_reset(sc);
-	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3);
+	if (sc->handle != NULL)
+		acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3);
+	else
+		sst_wpt_power_down(sc);
 	sc->state = SST_STATE_SUSPENDED;
 
 	sst_dbg(sc, SST_DBG_LIFE, "Suspended\n");
@@ -1546,8 +1549,12 @@
 	sst_dbg(sc, SST_DBG_LIFE, "Resuming...\n");
 
 	/* 1. Power to D0 */
-	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0);
-	DELAY(10000);
+	if (sc->handle != NULL) {
+		acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0);
+		DELAY(10000);
+	} else {
+		sst_wpt_power_up(sc);
+	}
 
 	/* 2. Init SHIM (mask interrupts, reset DSP) */
 	sst_init(sc);
```
*(Uwaga: Aby funkcja `sst_wpt_power_down` była dostępna w `acpi_intel_sst.c`, należy usunąć słowo kluczowe `static` z jej deklaracji w `sst_power.c` oraz dodać jej prototyp do nagłówka `acpi_intel_sst.h`)*.

---

## 2. Brak rejestracji obsługi przerwań w trybie PCI (Stall IPC)

### Opis błędu
W funkcji `sst_pci_attach` (plik [acpi_intel_sst.c](file:///home/rabczukr/github.com/spagu/acpi_intel_sst/src/acpi_intel_sst.c#L1140)) następuje alokacja zasobów przerwania `sc->irq_res` (linie 1235–1236). Jednak w przeciwieństwie do ścieżki ACPI (`sst_acpi_attach`), sterownik w ogóle nie wywołuje funkcji `bus_setup_intr`, aby zarejestrować funkcję obsługi przerwania (`sst_intr`).

Konsekwencje:
1. Przerwania sprzętowe nigdy nie trafiają do sterownika (`sc->irq_cookie` jest `NULL`).
2. Ponieważ `sc->irq_res != NULL`, podsystem IPC uważa, że urządzenie działa w trybie przerwaniowym (IRQ) i podczas wywołań blokujących czeka za pomocą zmiennej warunkowej `cv_timedwait` (plik [sst_ipc.c](file:///home/rabczukr/github.com/spagu/acpi_intel_sst/src/sst_ipc.c#L335)).
3. Ponieważ przerwania nie są obsługiwane, wątek nigdy nie otrzymuje sygnału o nadejściu odpowiedzi od DSP. Każde wywołanie IPC kończy się błędem przekroczenia limitu czasu (**IPC timeout** po 1 sekundzie). Cała komunikacja z procesorem sygnałowym zostaje sparaliżowana.

### Sugerowane rozwiązanie
Należy dodać brakującą rejestrację procedury przerwań `bus_setup_intr` do funkcji `sst_pci_attach` zaraz po alokacji przerwania.

#### Proponowany Patch (Diff):
```diff
--- src/acpi_intel_sst.c
+++ src/acpi_intel_sst.c
@@ -1240,6 +1240,12 @@
 	} else {
 		sst_dbg(sc, SST_DBG_LIFE, "IRQ assigned\n");
+		error = bus_setup_intr(dev, sc->irq_res,
+		    INTR_TYPE_AV | INTR_MPSAFE,
+		    NULL, sst_intr, sc, &sc->irq_cookie);
+		if (error) {
+			device_printf(dev, "Failed to setup IRQ: %d\n", error);
+			sc->irq_cookie = NULL;
+		}
 	}
```

---

## 3. Zahardkodowany fizyczny adres pamięci BAR0 w funkcji awaryjnej SRAM (`sst_enable_sram_direct`)

### Opis błędu
W pliku [sst_sram.c](file:///home/rabczukr/github.com/spagu/acpi_intel_sst/src/sst_sram.c) zdefiniowano stałą:
```c
#define SST_PCI_BAR0_PHYS	0xDF800000	/* PCI-allocated BAR0 physical address */
```
oraz użyto jej w funkcji `sst_enable_sram_direct` (oraz nieużywanej `sst_check_sram_immediate`):
```c
bar0_va = pmap_mapdev_attr(SST_PCI_BAR0_PHYS, 0x100000, VM_MEMATTR_UNCACHEABLE);
```
Fizyczne adresy BAR0 i innych rejestrów urządzeń PCI są przydzielane dynamicznie przez BIOS/ACPI i system operacyjny podczas rozruchu. Adres `0xDF800000` był poprawny tylko na konkretnym komputerze dewelopera (np. Dell XPS 13 9343), na którym BAR0 wylądował akurat w tym miejscu.
Na każdym innym komputerze wywołanie `sst_enable_sram_direct` (które następuje, gdy standardowe włączenie SRAM się nie powiedzie w `sst_pci_attach`) zmapuje całkowicie losowy obszar pamięci fizycznej. Próba odczytu lub zapisu pod te wskaźniki (`*sram_base = ...`, `*ctrl_reg = ...`) spowoduje:
1. Uszkodzenie pamięci systemowej (nadpisanie struktur innych urządzeń lub danych jądra).
2. Wygenerowanie błędu stronicowania (page fault) w trybie jądra, co natychmiast doprowadzi do **kernel panic**.

### Sugerowane rozwiązanie
Funkcja `sst_enable_sram_direct` powinna przyjmować strukturę `sc` (lub odpytywać `softc` za pomocą `device_get_softc(dev)`) i pobierać poprawny fizyczny adres dynamicznie za pomocą funkcji `rman_get_start(sc->mem_res)`.

#### Proponowany Patch (Diff):
```diff
--- src/sst_sram.c
+++ src/sst_sram.c
@@ -242,18 +242,28 @@
 int
 sst_enable_sram_direct(device_t dev)
 {
+	struct sst_softc *sc;
 	void *bar0_va;
 	volatile uint32_t *ctrl_reg;
 	volatile uint32_t *sram_base;
 	uint32_t ctrl, test_val;
+	vm_paddr_t phys_addr;
 	const uint32_t clear_val = 0x84800400;
 	const uint32_t set_val = 0x8480041f;
 
 	device_printf(dev, "=== SRAM Check & Enable ===\n");
 
+	sc = device_get_softc(dev);
+	if (sc == NULL || sc->mem_res == NULL) {
+		device_printf(dev, "  Failed to enable SRAM: BAR0 resource not allocated\n");
+		return (ENXIO);
+	}
+
+	phys_addr = rman_get_start(sc->mem_res);
+
 	/* Map with UNCACHED attribute - atomic 32-bit writes work correctly */
-	bar0_va = pmap_mapdev_attr(SST_PCI_BAR0_PHYS, 0x100000, VM_MEMATTR_UNCACHEABLE);
+	bar0_va = pmap_mapdev_attr(phys_addr, 0x100000, VM_MEMATTR_UNCACHEABLE);
 	if (bar0_va == NULL) {
-		device_printf(dev, "  Failed to map BAR0 at 0x%x\n", SST_PCI_BAR0_PHYS);
+		device_printf(dev, "  Failed to map BAR0 at 0x%lx\n", (unsigned long)phys_addr);
 		return (ENOMEM);
 	}
```
