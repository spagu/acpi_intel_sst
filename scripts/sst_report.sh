#!/bin/sh
#
# sst_report.sh - Generate comprehensive debug report for Intel SST Audio Driver
#
# This script collects system information, driver logs, ACPI tables, and
# audio configuration to help diagnose issues with acpi_intel_sst.
#
# Usage: sudo ./sst_report.sh
# Output: /tmp/sst_report_<timestamp>.tar.gz
#

if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root."
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT_DIR="/tmp/sst_report_${TIMESTAMP}"
REPORT_FILE="${REPORT_DIR}.tar.gz"

mkdir -p "${REPORT_DIR}"
echo "Collecting debug information to ${REPORT_DIR}..."

# --- 1. System Information ---
echo "- Collecting system info..."
uname -a > "${REPORT_DIR}/uname.txt"
kenv > "${REPORT_DIR}/kenv.txt"
sysctl hw.model hw.machine hw.ncpu > "${REPORT_DIR}/hw_info.txt"
sysctl dev.acpi_intel_sst > "${REPORT_DIR}/sst_sysctl.txt" 2>&1

# --- 2. Loader Configuration ---
echo "- Collecting loader.conf..."
cat /boot/loader.conf > "${REPORT_DIR}/loader.conf.txt"
cat /boot/device.hints > "${REPORT_DIR}/device.hints.txt"

# --- 3. PCI & Devinfo ---
echo "- Collecting PCI and device info..."
pciconf -lv > "${REPORT_DIR}/pciconf.txt"
devinfo -v > "${REPORT_DIR}/devinfo.txt"

# --- 4. Audio Configuration ---
echo "- Collecting audio state..."
cat /dev/sndstat > "${REPORT_DIR}/sndstat.txt"
mixer > "${REPORT_DIR}/mixer.txt" 2>&1

# --- 5. Kernel Logs (dmesg) ---
echo "- Collecting kernel logs..."
dmesg > "${REPORT_DIR}/dmesg.txt"
# Filter specifically for sst driver messages
grep -i "sst" "${REPORT_DIR}/dmesg.txt" > "${REPORT_DIR}/dmesg_sst_only.txt"

# --- 6. ACPI Tables (Warning: this dumps binary AML) ---
echo "- Collecting ACPI tables (acpidump)..."
acpidump -dt > "${REPORT_DIR}/acpidump.txt" 2>&1

# --- 7. GPIO / Pin Config (if available) ---
echo "- Collecting GPIO config..."
gpioctl -l > "${REPORT_DIR}/gpioctl.txt" 2>&1

# --- 8. Loaded Kernel Modules ---
echo "- Collecting kldstat..."
kldstat > "${REPORT_DIR}/kldstat.txt"

# --- 9. Firmware Check ---
echo "- Checking firmware files..."
ls -l /boot/firmware/intel/ > "${REPORT_DIR}/firmware_list.txt" 2>&1
md5 /boot/firmware/intel/* > "${REPORT_DIR}/firmware_md5.txt" 2>&1

# --- 10. DSP Status (if sysctl available) ---
echo "- Snapshotting DSP telemetry..."
sysctl dev.acpi_intel_sst.0.telemetry > "${REPORT_DIR}/dsp_telemetry.txt" 2>&1

# --- Bundle it all up ---
echo "Compressing report..."
tar -czf "${REPORT_FILE}" -C /tmp "sst_report_${TIMESTAMP}"

# Cleanup
rm -rf "${REPORT_DIR}"

echo ""
echo "========================================================"
echo "Report generated successfully!"
echo "File: ${REPORT_FILE}"
echo "========================================================"
echo "Please attach this file to your GitHub issue:"
echo "https://github.com/spagu/acpi_intel_sst/issues"
echo "========================================================"
