# Legacy Ford LKA test mode

`ford-lka-option` adds an active, limited-authority LKA controller for the owner's
Fusion retrofit using the existing `FORD_EDGE_MK2` classic-CAN profile. LCA remains
the default. The MKZ and other Ford profiles are not enabled for active LKA.
This is an experimental controller for evaluating the interface and recording
interruptions. Software checks cannot establish the installed rack's reference
calibration, sign, actuation acceptance, or the cause of an observed dropout.

## Selection and ownership

- `dp_ford_legacy_lka` defaults OFF and is read once at startup. Change while
  parked and restart to switch interfaces. OFF restores the previously saved
  BluePilot and factory-ACC preferences without overwriting them.
- ON forces factory Ford ACC for gas/brake/gap/resume, clears BluePilot selection,
  and selects exactly Ford Panda parameter **24** (`STOCK_ACC=8 | LEGACY_LKA=16`).
- ALKA is excluded. Normal ACC engagement is required. Physical driver controls
  and explicit cancellation are retained; host acceleration and resume are blocked.
- Legacy active steering is sent on 0x3CA at 33 Hz, matching recorded OEM traffic.
  A strictly neutral 0x3D3 LCA heartbeat is sent at 20 Hz for compatibility. The
  Panda blocks both OEM steering streams from being forwarded onto the controlled
  bus and rejects every non-neutral host LCA command in LKA mode.
- Unsupported Ford profiles are dashcam-only/noOutput when this option is saved.
  The setting does not affect Pacifica parameters or safety behavior.

## Initial command authority and independent Panda checks

The initial test mode permits active commands only from **15 to 30 m/s
(54–108 km/h)**, with fresh rack status, valid CAN, normal cruise engagement,
no brake/gas input, no EPS fault, and no driver override.

The packet uses only the observed ordinary intervention codes 2 and 4 and an
inactive code of 0. LDW requests, driver-override requests and quick ramp are
never used. Signed reference is limited to **+/-128 raw counts** about 2048;
curvature is limited to **+/-300 raw counts**, or +/-0.0015 m^-1. After entry, both fields change by at most **two raw counts per 30 ms update**.
The host seeds from the SCCM wheel signal, independently decoded on Panda.
The first Panda envelope allows +/-12 reference counts around the fresh measured
wheel and +/-8 curvature counts around fresh yaw/speed for transport delay and
quantization, rather than an arbitrary host-seeded value. Nonfinite or out-of-authority input releases the command.

The historical Ford LKA encoder names the reference scale as 0.04297 steering-
wheel degrees/count; the newer Ford DBC names the same field as 0.05 mrad/count.
The host follows the historical wheel-angle conversion after applying the
current Ford CAN-curvature sign convention. The raw 128-count limit corresponds
to +/-5.50 historical equivalent degrees or +/-6.40 modern-DBC mrad. **This does
not claim either label has been calibrated on the car.** Both raw values and
host intent are logged for first-response validation; no guessed unit is hidden
behind an unlimited encoder. There is no selectable larger-authority mode.

Panda independently checks absolute raw limits, raw slew, a 2-degree historical-
equivalent wheel error, direction consistency, and a 0.001 m^-1 curvature error
from yaw/speed. It also requires all ten RX checks to have been seen with good
checksums/quality/counters and age <=150 ms. The host uses a separate older-DBC parser for the exact SCCM wheel-angle
field on 0x76; Panda decodes the bits independently. That angle is
cross-checked against the existing camera reference on 0x3A8 to within 0.5 degree.
Driver torque above 1 Nm, camera/SCCM angle mismatch, denial/unavailability, wrong bus,
wrong action, reserved fields, an active packet interval outside 20–100 ms, and
conflicting mode flags are rejected. The active mode exists only in debug H7
builds; a release build or invalid parameter pairing fails closed.

LKA unavailability while the test is active releases commands and **latches the
host controller for that engagement**. A temporary `latActive=false` cannot
clear it; an actual disengagement is required. The latch is also surfaced as
a temporary steering fault and a steering warning, rather than showing effective
steering as active after commands have stopped. There are no periodic off pulses,
forced EPS resets, automatic rearm loops, watchdog defeats or firmware changes
to conceal the interruption. A dropout may still come from bounds, eligibility,
driver input, sensor freshness, or the rack; the logged reason distinguishes
these conditions. A controller test does not guarantee a timer lockout occurs.

## Recording and diagnostics

Each emitted LKA decision is logged as `bluedragon_ford_legacy_lka`, including
monotonic command time, enabled/latActive intent, requested curvature, effective
action, raw reference/curvature, rack availability, status age and latch reason.
Logging is observational and cannot alter commands.

The passive collector now also retains 0x7E and 0x85 pinion frames, which the
first collector omitted. It listens to `can`, `sendcan`, `carState`, `carControl`,
`pandaStates` and only the relevant command events from `logMessage`. RX and host
TX are kept separate; host TX alone does not prove physical bus transmission or
EPS execution. The default is **200,000 raw frames** and **20,000 state/command
samples**; both kinds of truncation are explicit. The full-minute state budget
replaces the old 4,000-sample limit that was inadequate once driving services run.
The collector never opens Panda, sends CAN, writes parameters or flashes modules.

After installing the matched package and checking live mode while parked:

```bash
cd /data/openpilot
python3 tools/ford_legacy_lka/check_option.py
PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python3 tools/ford_legacy_lka/capture.py --vehicle fusion_2019 --label before --seconds 60 --output /data/fusion-bluedragon-lka-before.json
```

Use a different filename on repeat runs. Retain the JSON, including requested
commands before an interruption, raw status, feedback, Panda state/counters and
recovery afterward. If `truncated` or `states_truncated` is true, do not interpret
missing later events as a lockout. Long captures still need an explicit budget
or several separate recordings.

## Existing evidence

The 20-second parked `fusion-lka-before.json` has SHA-256
`19848b4b7f50a602840dfad3d8d89204aa3f6236bf0e7a95b91d20cd3c3efa94`.
It contains only neutral steering traffic, with forced offroad/noOutput and
no carState/carControl samples. It was not a failed active-mode startup.

The 60-second OEM `fusion-stock-lka-active.json` has SHA-256
`61a3dc03f282997e838de4d0ab0c7f37eaa92e2861b17f0ee4ffbb67627a94cc`.
It contains 91,845 received frames, 600 Panda samples, no errors/truncation,
and five nonzero LKA request periods: 6.031, 6.050, 4.770, 4.591 and 4.760 seconds.
Four use code 4 and one code 2. LCA mode is zero throughout. Before each availability
drop, the camera's request is already zero by 30–41 ms in receive timestamps.
There is no recorded continuing active request against unavailable rack status,
so these are not established timer lockouts. Availability alone and the longest
16.53-second available run do not prove continuous assistance.

Latest recorded UCDS identity from the shared handoff (1 November 2026): PSCM
`K2GC-14D003-AH` / `K2GC-14D007-CD`, hardware `KK1C-14F079-AA`; IPMA
`LM2T-14F397-AA` / `L1TT-14F397-BD`. The capture does not independently identify
the modules or prove installed flash bytes. No replacement VBF or no-lockout
patch is included in this branch.

## Package and verification

The device release must include both the signed H7 application and the matching
170-key ARM Python 3.12 settings library. `tools/ford_legacy_lka/verify_release.py`
checks firmware signature, image structure, ARM architecture, hashes and source
pairing. `check_option.py` separately reads saved/startup mode and live services,
CAN validity, Panda parameter and TX-blocked counter. A correct safety parameter
does not independently prove the firmware signature; use its startup log too.

The source build commit precedes the packaging commit; the release manifest names
that build and the exact artifact hashes. The previous `ford-stock-acc` branch
and the OFF/LCA selection remain available for returning to the prior interface.
Do not reuse an older library or firmware with new source files.

## Validation scope

The tests exercise the real host controller against native C Panda checks,
including both signs, rate and error bounds, driver override, unavailable/denied
status, stale/missing sensors, malformed/reserved fields, wrong bus, invalid mode
pairings, ALKA exclusion, inactive resets, exclusive steering, factory ACC/AEB
forwarding, no automatic resume, unchanged Pacifica behavior and LCA restoration.
Native ARM builds test every parameter key in isolated storage. These checks do
not replace on-vehicle direction/response validation and do not demonstrate a
PSCM timeout cause or its removal.
