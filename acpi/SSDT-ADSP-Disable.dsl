/*
 * SSDT Override to Disable Intel ADSP (SST) - Force HDA Mode
 * ===========================================================
 *
 * This SSDT overrides the _SB.PCI0.ADSP._STA method to always return Zero,
 * effectively disabling the Intel SST (Smart Sound Technology) DSP device.
 * This forces the Dell XPS 13 9343 to use HDA audio mode instead of I2S mode.
 *
 * INSTALLATION (FreeBSD):
 *   1. Compile: iasl SSDT-ADSP-Disable.dsl
 *   2. Copy:    cp SSDT-ADSP-Disable.aml /boot/acpi_dsdt.aml
 *   3. Add to /boot/loader.conf:
 *        acpi_dsdt_load="YES"
 *        acpi_dsdt_name="/boot/acpi_dsdt.aml"
 *   4. Cold reboot (not soft reboot!)
 *
 * NOTE: FreeBSD's acpi_dsdt_load is for DSDT, not SSDT. For SSDT overlay,
 *       you may need to use a modified DSDT instead. See DSDT_patched.dsl
 *       for a full DSDT patch if this SSDT approach doesn't work.
 *
 * Target: Dell XPS 13 9343 (BIOS A02+)
 * Date:   2026-02-10
 */

DefinitionBlock ("SSDT-ADSP-Disable.aml", "SSDT", 2, "DELL", "ADSPOFF", 0x00001000)
{
    /*
     * Reference the existing ADSP device in the DSDT
     */
    External (_SB_.PCI0.ADSP, DeviceObj)

    /*
     * Override the _STA method to disable ADSP
     * Original method checked OSYS >= 0x07DC (Windows 2012+) for I2S mode
     * By returning Zero, we force the system to use HDA audio instead
     */
    Scope (\_SB.PCI0.ADSP)
    {
        Method (_STA, 0, NotSerialized)
        {
            /*
             * Return Zero to disable ADSP device
             * This prevents I2S/SST mode and forces HDA mode
             */
            Return (Zero)
        }
    }
}
