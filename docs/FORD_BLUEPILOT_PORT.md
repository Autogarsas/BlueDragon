# BlueDragon 1.0 — preserved bp-5.0 Ford steering port

Target: dragonpilot 0.11.1, base 3e8edb7e3d6d8fb0393f0604ee3954ee328857f5.
Source: BluePilotDev/bluepilot bp-5.0, c6f8350f17c0622d7a075fe03ae760425e68fd2c.
Hardware checked: comma 3X / TRES / H7 on the owner's retrofitted Fusion using Edge MK2.
The owner reports the repaired baseline works well. The renamed installation
package still needs a fresh-install/device-UI check; see the root README.

## Scope

Ports the curvature-primary Ford lateral calculations: predicted/actuator curvature blending, rate tables, lane-change scaling, optional path-angle lane positioning, and effective custom settings. Adds the bp-5.0 Edge MK2 platform and firmware definitions, gear signal, and alternate steering-angle reference. The existing dragonpilot vehicle-model collector discovers the new platform; a second vehicle selector is not included.

The Ford Steering (BluePilot) section exposes defaults and tuning. Predicted-curvature blend is 0.4, high-speed lane-change factor 0.85, human-turn detection enabled, lane positioning and custom profile disabled. Blend sliders require the custom profile. Positioning controls require lane positioning. The disable switch requires a restart and pairs the stock controller with stock safety mode. This version enables the port by default on Ford.

The existing dragonpilot longitudinal controller, cruise buttons, and HUD remain in place. This is not bp-7.0 angle steering, and does not include unrelated BluePilot UI, portal, themes, or inactive tuning controls. No claim is made that all BluePilot driving behavior is reproduced: model and planner versions differ.

## Intentional differences

- bp-5.0's safety reset latch can clear steering violations for about three seconds. It is not imported. Human-turn detection or standstill emits an inactive zeroed command; normal rate and measured-curvature checks apply on recovery.
- Auxiliary path angle is bounded to +/-0.25 rad, matching the source safety bound rather than its looser host bound. Path offset on CAN remains zero.
- Missing, stale, malformed, or nonfinite model data disables the new lateral output. No cached-model fallback.
- Edge steering angle is invalid until a valid absolute reference arrives; no uninitialized offset is used.
- The lane-line positioning option is wired to the attribute actually read by the controller. The source naming mismatch made its UI setting ineffective. Positioning remains off by default.
- Values loaded from settings are finite and bounded. Dead diagnostic tuning options are omitted.

## Validation completed

Python 3.12, pycapnp 2.2.4, hypothesis 6.47.5, numpy, and native C libsafety on the development host:

    PYTHONPATH=opendbc_repo python -m unittest       opendbc.car.ford.tests.test_bluepilot       opendbc.safety.tests.test_ford_bluepilot       opendbc.safety.tests.test_ford -q

210 tests ran, OK, 20 skipped by inherited suites. An additional six existing Ford firmware tests passed. New tests cover default blending, missing model, invalid values, Edge reference acquisition, human-turn resets, lane positioning, path-angle bounds/history, and reset-bypass rejection.

Synthetic sinusoidal command sequences cover six speeds (5, 10, 16, 20, 25, 35 m/s), 400 frames each, on CAN and two CAN-FD safety configurations. They exercise host calculations and safety message encoding; they are not vehicle-dynamics simulation or recorded-drive replay. BP-specific rate tests use interior/exterior margins to avoid float64/float32 rounding differences in the stock two-point oracle. The unchanged stock safety suite also runs with BP disabled.

New Python files pass Ruff. Settings keys were regenerated. These checks do not establish safety of combined curvature, curvature-rate, and path-angle authority.

## Packaging and remaining validation

BlueDragon 1.0 preserves the existing driving code and defaults. It adds branding,
credits, an ARM CPython 3.12 parameter library containing all 168 declared keys,
and the matching signed H7 image in the normal installation path. Provenance and
checksums are recorded in `tools/ford_bluepilot/release.json`.

The targeted parameter repair ran successfully on the device. The packaged
cross-built equivalent passed isolated key/storage checks under ARM Python 3.12
via QEMU. The exact packaged H7 image was flashed on the device and its running
signature matched `9a428b52a93e508a`. The owner subsequently reported good Ford
behavior. Those checks do not constitute a clean full-system build, independent
vehicle-dynamics validation, or a fresh install of this renamed package.

Remaining work includes checking the renamed UI and fresh installation on the
device, representative recorded-route replay, and review of combined auxiliary
steering authority. Other vehicles, retrofits, and non-H7 hardware are unverified.
The bp-7.0 angle controller is not part of this version.

## Attribution

Adapted from BluePilot, sunnypilot, openpilot, and opendbc contributors. See
CREDITS.md, LICENSE, LICENSE.md, and LICENSE_BLUEPILOT.md; the included components
do not all share one license. Source commits above identify the original work.
