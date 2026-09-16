# SPDX-License-Identifier: BSD-3-Clause
#
# Host-side unit tests.  They build and run with a plain host gcc/clang
# and do NOT need a FreeBSD kernel build environment:
#
#   test_fw_bounds   - firmware image parser bounds predicates, mirrored
#                      in sst_fw_predicates.h (see that file for the
#                      keep-in-sync contract with src/sst_firmware.c)
#   test_trig_state  - deferred trigger request coalescing, shared with
#                      the driver via src/sst_trig_state.h (issue #48)
#   test_errstat     - I2C failure-streak accounting, shared with the
#                      driver via src/sst_errstat.h (issue #49)
#   test_recover     - DSP self-recovery policy, shared with the driver
#                      via src/sst_recover_policy.h (issue #51)
#   test_ipc_gate    - IPC sender serialization, shared with the driver
#                      via src/sst_ipc_gate.h (panic: sleeping thread
#                      holds sst_ipc_send)
#
# "make coverage" reruns the tests with gcov and fails when any shared
# header falls below COVERAGE_MIN percent of lines executed.
#
# Copyright (c) 2026 Tradik Limited
# All rights reserved.

CC?=		cc
CFLAGS?=	-std=c11 -Wall -Wextra -Werror -fsanitize=undefined -g -O1
LDFLAGS?=	-fsanitize=undefined
CPPFLAGS+=	-I../../src

TESTS=		test_fw_bounds test_trig_state test_errstat test_recover \
		test_ipc_gate test_rate_limit
SHARED_HDRS=	sst_trig_state.h sst_errstat.h sst_recover_policy.h \
		sst_ipc_gate.h sst_rate_limit.h
COVERAGE_MIN?=	96

.PHONY: test coverage clean

test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

test_fw_bounds: test_fw_bounds.c sst_fw_predicates.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ test_fw_bounds.c $(LDFLAGS)

test_trig_state: test_trig_state.c ../../src/sst_trig_state.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ test_trig_state.c $(LDFLAGS)

test_errstat: test_errstat.c ../../src/sst_errstat.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ test_errstat.c $(LDFLAGS)

test_recover: test_recover.c ../../src/sst_recover_policy.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ test_recover.c $(LDFLAGS)

test_ipc_gate: test_ipc_gate.c ../../src/sst_ipc_gate.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ test_ipc_gate.c $(LDFLAGS)

test_rate_limit: test_rate_limit.c ../../src/sst_rate_limit.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ test_rate_limit.c $(LDFLAGS)

# Line coverage of the headers shared with the driver (gcc + gcov).
coverage:
	@rm -rf cov && mkdir -p cov
	@for t in test_trig_state test_errstat test_recover test_ipc_gate \
	    test_rate_limit; do \
		$(CC) $(CPPFLAGS) -std=c11 -Wall -Wextra -O0 --coverage \
		    -o cov/$$t $$t.c && (cd cov && ./$$t >/dev/null); \
	done
	@cd cov && for h in $(SHARED_HDRS); do \
		gcov -o . *.gcno 2>/dev/null | \
		    awk -v h=$$h -v min=$(COVERAGE_MIN) \
		    '$$0 ~ "File .*" h "'"'"'" { getline; \
		     split($$0, a, ":"); pct = a[2] + 0; \
		     printf "%-20s %5.1f%% lines\n", h, pct; \
		     if (pct < min) { \
		         printf "coverage below %d%%\n", min; exit 1 } }' \
		    || exit 1; \
	done

clean:
	rm -f $(TESTS) *.gcda *.gcno *.gcov
	rm -rf cov
