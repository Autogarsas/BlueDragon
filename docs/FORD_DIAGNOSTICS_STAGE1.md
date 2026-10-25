# BlueDragon 1.1-dev1: steering diagnostics

This checkpoint adds observations to the BP5 curvature controller. Steering,
tuning, manual-turn detection, safety checks and firmware remain unchanged.
BP7 angle control and stall recovery are not included. `bluedragon-1.0` is retained.

At each 20 Hz steering update, diagnostic flags observe curvature deviation,
rate, CAN-FD acceleration and magnitude limits; path-angle rate/magnitude and
curvature-rate signal clipping are also observed. Reset reasons distinguish
inactive control, missing model, invalid model, standstill and human turns.
The immediate BP5 human-turn rule is unchanged.

After CAN publication, a separate observer emits at most one
`bluedragon_ford_steering` cloudlog event per second. It counts every unique sample
and reason entry, preserving brief resets. The latest snapshot includes frame,
time, speed, driver input, wheel angle, EPS faults and input/predicted/blended/
measured/output curvature when available, in host sign convention before CAN
inversion. Duplicate 100 Hz observations are ignored. Logging failures disable
the observer without raising into control. No new file writer, schema, parameter
or firmware build is required; the existing nonblocking cloudlog socket is used.
Device timing and logging performance still need hardware checks.

These flags describe host limits, not Panda rejection reasons or EPS acceptance.
The reader reports existing Panda TX-blocked counters and EPS faults separately.
`controlsAllowed=false` alone does not prove ALKA is blocked. The final incomplete
one-second window is not flushed on shutdown. Missing segments/events reduce
coverage; zero diagnostic windows means no new events found, not no limits.

## Installation and baseline

Development installer (selects `bluedragon-dev`):
https://raw.githubusercontent.com/Autogarsas/BlueDragon/bluedragon-dev/installer/bluedragon-dev

The stable installer remains unchanged and selects `bluedragon-1.0`.
`build_dev_installer.py` reproduces the equal-length padded branch parameter
change; executable instructions, repository URL and file size remain unchanged.
The installer has not been executed on hardware here.

Install while parked, verify startup and the displayed development branch, then
record a normal supervised baseline with existing settings. Do not enable
lateral maneuver mode. Afterwards, while parked, run over SSH:

```sh
cd /data/openpilot
python3 tools/ford_bluepilot/check_drive_logs.py --all-segments
```

This reads all available segments of the latest route; long routes take time.
Without `--all-segments`, first + last six segments remain the default. Prefer
rlog; qlog fallback may omit events. Share the summary before the next change.

## Validation

Eleven tests (nine existing Ford tests and two observer tests) cover controller
basics, deduplication, brief reset counts, nonfinite values, event rate and logging
failure isolation. The regression script compares 10,000 frames against a5aa0011,
including all packed CAN messages and actuator outputs, for Edge CAN/Mach-E
CAN-FD and default/lane-positioning settings. Cases include inactive control,
invalid/missing models, low speed, driver override and lane changes. A real Capnp
serialization roundtrip checks the log reader. Installer reproduction and corrupt
source rejection are tested. Repaired Params, H7 firmware and safety sources are
unchanged. No real-route replay, new hardware drive, fresh installation or device
timing measurement is claimed.

Run from the repository root with dependencies installed:

```sh
python3 -m unittest opendbc.car.ford.tests.test_bluepilot dragonpilot.selfdrive.car.tests.test_ford_diagnostics
python3 tools/ford_bluepilot/verify_stage1_equivalence.py
python3 tools/ford_bluepilot/verify_stage1_logs.py
```

The comparison needs a5aa0011 in local git history. Upstream credits and licenses
remain in CREDITS.md and LICENSE_BLUEPILOT.md. These are new diagnostics adapted
to the BP5 port, not imported BP7 control algorithms.
