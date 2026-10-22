# BluePilot bp-5.0 Ford steering port — development draft

Target: dragonpilot 0.11.1, base 3e8edb7e3d6d8fb0393f0604ee3954ee328857f5.
Source: BluePilotDev/bluepilot bp-5.0, c6f8350f17c0622d7a075fe03ae760425e68fd2c.
Intended hardware: comma 3X. Not built or validated on that hardware.

## Scope

Ports the curvature-primary Ford lateral calculations: predicted/actuator curvature blending, rate tables, lane-change scaling, optional path-angle lane positioning, and effective custom settings. Adds the bp-5.0 Edge MK2 platform and firmware definitions, gear signal, and alternate steering-angle reference. The existing dragonpilot vehicle-model collector discovers the new platform; a second vehicle selector is not included.

The BluePilot Ford Steering section exposes defaults and tuning. Predicted-curvature blend is 0.4, high-speed lane-change factor 0.85, human-turn detection enabled, lane positioning and custom profile disabled. Blend sliders require the custom profile. Positioning controls require lane positioning. The disable switch requires a restart and pairs the stock controller with stock safety mode. This draft enables the port by default on Ford.

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

## Release blockers

This is a source-review draft, not an installation release. Before deployment:
1. Confirm the actual vehicle year, trim, market, harness, and reason for selecting the Edge profile. Selecting Edge is not proof of compatibility with a different Ford.
2. Complete a clean comma 3X build, including regenerated parameter bindings and Panda firmware, and verify the settings UI/vehicle selector on device.
3. Replay representative recordings from the user's working bp-5.0 setup; compare steering output, safety rejections, engagement transitions, sensor validity, and model timing. The change to reset behavior particularly needs validation.
4. Review combined auxiliary steering authority and applicable vehicle limits with an experienced Ford/Panda maintainer. Verify ALKA and all relevant disable/override paths.
5. Only after review and bench validation, plan supervised controlled-environment testing and rollback.

No device was flashed and no repository was published. Keep the working bp-5.0 installation unchanged while reviewing this draft.

## Attribution

Adapted from openpilot and BluePilot contributors under the repository MIT license; see LICENSE. Source commit above identifies the original implementation.
