# SPDX-License-Identifier: BSD-3-Clause
#
# Host-side unit tests for src/sst_firmware.c bounds-checking predicates.
#
# These tests build and run with a plain host gcc/clang - they do NOT
# require a FreeBSD kernel build environment, since they exercise a
# mirrored copy of the arithmetic predicates (see sst_fw_predicates.h
# for why, and for the keep-in-sync contract with src/sst_firmware.c).
#
# Copyright (c) 2026 FreeBSD Foundation
# All rights reserved.

CC?=		cc
CFLAGS?=	-std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g -O1
LDFLAGS?=	-fsanitize=undefined

BIN=		test_fw_bounds
SRCS=		test_fw_bounds.c
HDRS=		sst_fw_predicates.h

.PHONY: test clean

test: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $(BIN) $(SRCS) $(LDFLAGS)

clean:
	rm -f $(BIN)
