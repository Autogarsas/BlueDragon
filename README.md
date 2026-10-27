# BlueDragon 1.1-dev2.1 — Pacifica always-on lateral

**dragonpilot features with BluePilot Ford steering.**

BlueDragon is an independent integration of dragonpilot 0.11.1 and the
curvature-primary Ford steering from BluePilot bp-5.0. This version adds opt-in
Pacifica always-on lateral while preserving the existing Ford controller.

Version 1.1-dev2.1 fixes ALKA settings visibility when the car profile arrives
after UI startup or changes within the same brand. It refreshes the complete
car context before showing settings.

## Versions and installation

- **`bluedragon-1.0`** preserves the current Ford baseline, with the parameter
  library repair and matching H7 Panda application included in the package.
- **`bluedragon-dev`** includes Ford steering diagnostics and Pacifica ALKA,
  adapted from BluePilot bp-5.0 / sunnypilot MADS. ACC Main enables lateral,
  the physical LKAS button toggles it, and steering requires speed above
  **63 km/h**. A matching signed H7 application is included. No BP7 control
  changes are included.

See [Pacifica ALKA](docs/PACIFICA_ALKA.md) for behavior, validation, update and
rollback instructions. Enable **Settings → BlueDragon → Lateral → Always-on
Lane Keeping Assist (ALKA)** and reboot to apply the setting.

See [Stage 1 diagnostics](docs/FORD_DIAGNOSTICS_STAGE1.md) for validation and
baseline logging instructions. Development installer:

**https://raw.githubusercontent.com/Autogarsas/BlueDragon/bluedragon-dev/installer/bluedragon-dev**

The stable installer remains available on the `bluedragon-1.0` branch.

Hardware verification so far covers a comma 3X / TRES / H7. The owner uses a
retrofitted 2019 Fusion Hybrid with the Ford Edge MK2 profile. This is a report
about that particular setup, not compatibility confirmation for other retrofits
or hardware. Install while parked and check startup before use.

The packaged ARM settings library has been tested under ARM Python 3.12 through
QEMU and is unchanged. The new H7 application was built in GitHub Actions and
its RSA signature, image layout and source manifest verified. New-firmware
flashing and Pacifica ALKA still require device validation. No full-system
rebuild or road-test result for this new feature is claimed.

## What is preserved

- Ford predicted/actuator curvature blending and the current steering tuning.
- Ford Edge MK2 profile and alternate steering-angle handling.
- Existing dragonpilot longitudinal behavior, driving model, ALKA, and features.
- Normal Panda checks during reset and recovery; no three-second reset bypass.
- The repair for all 11 Ford settings in `common/params_pyx.so`.
- The matching H7 application in its normal `panda/board/obj` installation path.

This is not bp-7.0 angle-primary steering. There is no driving-model selector
or legacy WD40 model in this version.

## Credits and licenses

- **[dragonpilot](https://github.com/dragonpilot/dragonpilot/tree/0.11.1)**:
  the base project and features. Thanks to Rick Lan and the dragonpilot community.
- **[BluePilot](https://github.com/BluePilotDev/bluepilot/tree/bp-5.0)**:
  the Ford steering work adapted here. Thanks to BluePilotDev and contributors.
- **[sunnypilot](https://github.com/sunnypilot/sunnypilot)**:
  upstream contributions carried through BluePilot.
- **[comma.ai/openpilot](https://github.com/commaai/openpilot),
  [opendbc](https://github.com/commaai/opendbc), and
  [panda](https://github.com/commaai/panda)**: the underlying software.

See [CREDITS.md](CREDITS.md) for source versions and the retained license notices.
Original dragonpilot documentation is preserved in
[English](README_DRAGONPILOT_EN.md) and [Chinese](README_DRAGONPILOT_ZH.md).

This software is licensed under a custom license requiring permission for use.

This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed under a custom license requiring permission for use.
