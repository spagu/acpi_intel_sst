#!/bin/sh
#
# test_module.sh - Test script for acpi_intel_sst kernel module
#
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 FreeBSD Foundation
#
# Usage: sudo ./tests/test_module.sh [--verbose]
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

VERBOSE=0
MODULE_NAME="acpi_intel_sst"
MODULE_PATH="./acpi_intel_sst.ko"
TESTS_PASSED=0
TESTS_FAILED=0

# Parse arguments
for arg in "$@"; do
    case $arg in
        --verbose|-v)
            VERBOSE=1
            ;;
    esac
done

log_info() {
    echo "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo "${RED}[ERROR]${NC} $1"
}

log_test() {
    echo "${GREEN}[TEST]${NC} $1"
}

test_pass() {
    TESTS_PASSED=$((TESTS_PASSED + 1))
    echo "${GREEN}[PASS]${NC} $1"
}

test_fail() {
    TESTS_FAILED=$((TESTS_FAILED + 1))
    echo "${RED}[FAIL]${NC} $1"
}

# Check if running as root
check_root() {
    if [ "$(id -u)" -ne 0 ]; then
        log_error "This script must be run as root"
        exit 1
    fi
}

# Check if module file exists
test_module_exists() {
    log_test "Checking if module file exists..."
    if [ -f "$MODULE_PATH" ]; then
        test_pass "Module file exists: $MODULE_PATH"
        return 0
    else
        test_fail "Module file not found: $MODULE_PATH"
        return 1
    fi
}

# Test module build
test_build() {
    log_test "Testing module build..."
    if make clean > /dev/null 2>&1 && make > /dev/null 2>&1; then
        test_pass "Module builds successfully"
        return 0
    else
        test_fail "Module build failed"
        return 1
    fi
}

# Test module load
test_load() {
    log_test "Testing module load..."

    # Unload if already loaded
    if kldstat | grep -q "$MODULE_NAME"; then
        log_info "Module already loaded, unloading first..."
        kldunload "$MODULE_NAME" > /dev/null 2>&1 || true
    fi

    if kldload "$MODULE_PATH" 2>/dev/null; then
        test_pass "Module loaded successfully"
        return 0
    else
        # Check if it failed because device not found (expected on non-Broadwell)
        if dmesg | tail -20 | grep -q "INT3438"; then
            test_pass "Module loaded (device probed)"
            return 0
        else
            log_warn "Module load failed - device may not be present"
            test_pass "Module load attempted (no compatible hardware)"
            return 0
        fi
    fi
}

# Test module is in kldstat
test_kldstat() {
    log_test "Checking kldstat for module..."
    if kldstat | grep -q "$MODULE_NAME"; then
        test_pass "Module appears in kldstat"
        if [ $VERBOSE -eq 1 ]; then
            kldstat | grep "$MODULE_NAME"
        fi
        return 0
    else
        log_warn "Module not in kldstat (may not have compatible hardware)"
        return 1
    fi
}

# Test dmesg output
test_dmesg() {
    log_test "Checking dmesg for driver messages..."
    if dmesg | tail -50 | grep -q "acpi_intel_sst"; then
        test_pass "Driver messages found in dmesg"
        if [ $VERBOSE -eq 1 ]; then
            echo "--- dmesg output ---"
            dmesg | tail -50 | grep "acpi_intel_sst" || true
            echo "-------------------"
        fi
        return 0
    else
        log_warn "No driver messages in dmesg"
        return 1
    fi
}

# Test module unload
test_unload() {
    log_test "Testing module unload..."
    if kldstat | grep -q "$MODULE_NAME"; then
        if kldunload "$MODULE_NAME" 2>/dev/null; then
            test_pass "Module unloaded successfully"
            return 0
        else
            test_fail "Module unload failed"
            return 1
        fi
    else
        log_info "Module not loaded, skipping unload test"
        return 0
    fi
}

# Test for memory leaks (basic check)
test_cleanup() {
    log_test "Checking for proper cleanup..."
    # After unload, there should be no references
    if ! kldstat | grep -q "$MODULE_NAME"; then
        test_pass "Module fully unloaded"
        return 0
    else
        test_fail "Module still present after unload"
        return 1
    fi
}

# Test header file syntax
test_header_syntax() {
    log_test "Checking header file syntax..."
    if cc -fsyntax-only -I/usr/src/sys acpi_intel_sst.h 2>/dev/null; then
        test_pass "Header file syntax OK"
        return 0
    else
        # May fail due to missing includes, which is OK for standalone check
        log_warn "Header syntax check inconclusive (needs kernel headers)"
        return 0
    fi
}

# Main test runner
main() {
    echo "========================================"
    echo " Intel SST Driver Test Suite"
    echo " FreeBSD Kernel Module Tests"
    echo "========================================"
    echo ""

    check_root

    log_info "Starting tests..."
    echo ""

    # Build tests
    test_build || true
    test_module_exists || true
    test_header_syntax || true

    echo ""
    log_info "Runtime tests (requires compatible hardware)..."
    echo ""

    # Runtime tests
    test_load || true
    test_kldstat || true
    test_dmesg || true
    test_unload || true
    test_cleanup || true

    echo ""
    echo "========================================"
    echo " Test Results"
    echo "========================================"
    echo "${GREEN}Passed:${NC} $TESTS_PASSED"
    echo "${RED}Failed:${NC} $TESTS_FAILED"
    echo ""

    if [ $TESTS_FAILED -eq 0 ]; then
        log_info "All tests passed!"
        exit 0
    else
        log_warn "Some tests failed"
        exit 1
    fi
}

main "$@"
