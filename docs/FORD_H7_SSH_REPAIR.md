> Historical SSH repair workflow. BlueDragon 1.0 now includes this exact H7
> application in its normal installation path and includes the repaired Params
> library. The older ford-bluepilot branches still use the workflow below.
> The H7 image was subsequently flashed and its running signature verified on
> the owner's comma 3X; the renamed package still needs a fresh-install check.

# H7 application repair for the Ford bp-5.0 port

The original prebuilt package kept stock Panda applications even though its Ford
safety source had changed. The port requests safety parameter bit 4 and emits
auxiliary steering signals that the stock firmware rejects. Device logs showed
the Ford/5 configuration, blocked commands and unchanged bundled firmware hashes.

`tools/ford_bluepilot/install_h7_firmware.py --stage` stages the rebuilt H7
application after checking the live TRES hardware type, ignition/offroad state,
the existing image, source/protocol checksums, SHA-256 and signing certificate.
It saves the original image in `/data/ford-bluepilot-backups`. The normal pandad
startup performs the flash and checks its signature after a parked reboot. The
installer never opens Panda directly, changes its safety configuration, flashes
a bootloader or modifies the repaired parameter library.

The candidate is under `tools/ford_bluepilot/firmware`, outside the normal
firmware lookup path. This branch is a targeted SSH repair workflow, not a full
replacement installation release. Do not use the main branch's UI update as a
substitute: it still contains the original prebuilt parameter library.

Build: source `f95cbde6532c5f1acf22f1bd020aa2e4079aef9e`, GNU Arm Embedded GCC
13.2.1, target `panda/board/obj/panda_h7.bin.signed`. The source already contains
the bounded Ford BluePilot checks; this repair introduces no additional steering
authority or reset bypass. Exact build inputs and hashes are in `firmware/build.json`.

Native controller/safety validation: 216 tests ran, 196 passed, 20 inherited
skips. Added ALKA tests verify setting, main-switch and movement gating, limit
enforcement, and main-switch disengagement. Installer checks cover application
signature/format, corruption rejection, atomic staging, backup, repeat staging
and refusal to overwrite unknown firmware. These are host tests; hardware flash
and driving behavior remain to be verified on the device.

Expected application version: `DEV-f95cbde6-DEBUG`.
Expected firmware signature prefix: `9a428b52a93e508a`.
Keep the car parked while staging, flashing and checking startup logs.

Rollback, while parked with ignition off:

```bash
python3 /tmp/ford-h7-fix.py --restore-stock
```

Rollback also disables the BluePilot controller so stock controller and stock
safety mode are paired after reboot. Do not restore stock Panda firmware while
leaving the BluePilot controller selected. Keep the backup until validation ends.
