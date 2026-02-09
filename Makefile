# Makefile for acpi_intel_sst kernel module
#
# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 FreeBSD Foundation
#
# Usage:
#   make          - Build the kernel module
#   make clean    - Clean build artifacts
#   make load     - Load the module (requires root)
#   make unload   - Unload the module (requires root)
#   make test     - Run test suite (requires root)
#   make install  - Install to /boot/modules
#   make help     - Show this help

KMOD=   acpi_intel_sst
SRCS=   acpi_intel_sst.c sst_firmware.c sst_ipc.c sst_ssp.c sst_dma.c
SRCS+=  device_if.h bus_if.h acpi_if.h
SRCS+=  opt_acpi.h

# Compiler flags
CFLAGS+= -Werror
CFLAGS+= -Wno-cast-qual

# Debug build (uncomment for verbose debugging)
# CFLAGS+= -DDEBUG -g

# Module installation directory
KMODDIR?= /boot/modules

.include <bsd.kmod.mk>

# Custom targets (load/unload provided by bsd.kmod.mk)
.PHONY: reload test install help

reload:
	-kldunload $(KMOD)
	kldload ./$(KMOD).ko

test:
	@echo "Running test suite..."
	@chmod +x tests/test_module.sh
	@tests/test_module.sh

install:
	@echo "Installing $(KMOD).ko to $(KMODDIR)..."
	install -m 555 $(KMOD).ko $(KMODDIR)/
	@echo "Done. Add 'acpi_intel_sst_load=\"YES\"' to /boot/loader.conf to load at boot."

help:
	@echo "Intel SST Audio Driver for FreeBSD"
	@echo ""
	@echo "Available targets:"
	@echo "  make          - Build the kernel module"
	@echo "  make clean    - Clean build artifacts"
	@echo "  make load     - Load the module (from bsd.kmod.mk)"
	@echo "  make unload   - Unload the module (from bsd.kmod.mk)"
	@echo "  make reload   - Unload and reload the module"
	@echo "  make test     - Run test suite (requires root)"
	@echo "  make install  - Install to $(KMODDIR)"
	@echo "  make help     - Show this help"
	@echo ""
	@echo "Debug build:"
	@echo "  Uncomment CFLAGS+= -DDEBUG -g in Makefile"
