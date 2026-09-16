# Top-level Makefile for acpi_intel_sst
#
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Tradik Limited
#
#   make            - build the driver (src/) and the firmware module
#                     (firmware/), in that order
#   make install    - install both modules into /boot/modules
#   make clean      - clean both
#   make unit-test  - run the host-side unit tests (any OS with a C
#                     compiler; no kernel sources needed)
#
# The driver alone can still be built with "make -C src".

SUBDIR=		src firmware
SUBDIR_PARALLEL=

.PHONY: unit-test

unit-test:
	${MAKE} -C tests/unit test
	${MAKE} -C tests/unit coverage

.include <bsd.subdir.mk>
