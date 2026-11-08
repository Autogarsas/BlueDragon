# Fusion LKA reference port

This branch ports the steering path from `JDGarlock/openpilot`, branch
`ford-devel-master`, pinned at `5b5f7516d813ec08c887183f9707ecd180306292`.
It is a paired development implementation, not a claim of verified EPS response.

## Exact reference material

Unmodified source files and their Git blob / SHA-256 hashes are under
`reference/jdgarlock/`. The original MIT license is retained there.

- openpilot: `5b5f7516d813ec08c887183f9707ecd180306292`
- opendbc: `d3a6e5fbeac38ff0f9c418f202ee8f004de28d1d`
- Panda, retained as **non-executable reference text only**:
  `38b393f73cc4251158336309e30c4ef72a751307`

The pinned Panda's forced-allow changes are not imported.

## Steering path

1. Modern planner curvature is converted to desired steering angle using the
   existing modern angle-controller convention. This bridges the old
   `path_plan.angleSteers` input.
2. The donor PI algorithm uses its original proportional/integral/feedforward
   behavior (`kp=.01`, `ki=.005`, `kf=1/87`), including its original anti-windup.
3. The normalized PID output is multiplied by `87`, exactly as in the donor.
4. The donor DBC encodes that numeric reference with scale `0.05`, offset
   `-102.4` and action `2`. Curvature comes from the donor's vehicle-model
   calculation on the desired steering angle. The live encoder explicitly
   sets LDW request to `LDW_Idle` and intensity to `None`, producing first byte
   `0x40`. The original `0xe` alert nibble is retained only in offline donor
   fixtures, not in the live encoder.
5. LKA packets are emitted at **100 Hz**. SCCM `0x76` supplies both published
   steering angle and controller/native-safety measurement. The unrelated
   ParkAid absolute-angle origin is not compared with SCCM's relative origin.

The old `carstate.py` refers to `BrakeSnData_5`, which is absent from its pinned
DBC. The retained steering signal is resolved by its actual pinned message
name, `Steering_Wheel_Data_CG1`, CAN ID `0x76`, rather than copying that stale
name. Sensor sign and raw scaling are unchanged.

The original Fusion geometry/tuning is used in this mode (2.85 m wheelbase,
14.8 steering ratio and the donor's mass/stiffness values). Those source values
are parity fixtures, not a VIN-specific calibration of the owner's hybrid.
The Edge selector remains the existing retrofit fingerprint alias. Cached
learned angle offsets are discarded when switching steering-reference modes.

## Explicit safety adaptations

These differences from the historical fork are intentional and tested:

- Native Panda parameter **40** (`STOCK_ACC | FUSION_LKA_PORT`) is debug-only
  and distinct from the archived parameter-24 prototype. Wrong flag pairings
  allow no output.
- The initial validation envelope remains 54–108 km/h, reference +/-128 raw
  counts, curvature +/-0.0015/m, and bounded slew. The upstream platform
  steering-output limit is set to `6.4/87` so its own anti-windup honors the
  same test authority. The raw caps do not establish physical rack calibration.
- Unavailable/invalid/stale inputs produce the previously observed OEM neutral
  packet `0380080000000000`. The donor writes `0xf` into a three-bit action
  field, producing reserved action 7; that inactive encoding is not imported.
- Active warning request/intensity fields must be zero in both host encoding
  and native safety. The old `0x4e` packet is rejected. Steering payload bytes,
  PID, action, rate, authority and torque cutoffs are unchanged. This targets a
  specific encoding mismatch after a report of factory-style LDW vibration;
  it does not establish the vibration's cause or prove its removal on a rack.
- Native checks independently enforce actual cruise permission, pedal and
  torque override, SCCM freshness, EPS/status validity, rate, raw authority and
  tracking limits. LCA stays inactive and factory ACC retains gas/brake/resume.
- Interruptions retain a host stop latch until actual disengagement. There are
  no periodic off pulses, automatic latch clearing or timer-reset workarounds.
- A native TX rejection latches the host off on the next Panda update and
  restores neutral heartbeats. Invalid/missing Panda mode status also blocks
  activation. This prevents repeating the one-second rejected-command gap
  observed in the earlier capture.

## Verification and packaging

The tests compare the real upstream PI, latcontrol and encoder functions with
the port, using the pinned DBC. Separate compiled C safety tests receive the
host's encoded stream, including the recorded camera/SCCM offset scenario.
Stock Ford, BluePilot, factory ACC and Pacifica regressions remain in CI.

The warning-suppression candidate requires a new paired H7 artifact. Pending
source revisions cannot be installed. The release manifest pins the artifact
ZIP digest, signed firmware digest, RSA signature, native source checksums and
existing ARM settings library. The installer leaves Legacy LKA disabled and
does not automatically restore an experimental steering selection. The
existing settings library is unchanged; no new setting key is required.

The paired warning-suppression candidate contains `DEV-c785bdc5-DEBUG`,
signature prefix `5de6e8a8352bce37`, from source commit
`c785bdc5f93e77b870d6d91a57fb67d874dea6b0`, workflow run `42381862110`, artifact
`13577288390`. CI ran 578 tests: 534 passed, with 44 inherited skips. Vehicle
validation remains pending; package pairing is not a claim that the reported
vibration has been removed.

The modern controller publishes the port's native PID telemetry through
`pidState`, while retaining the Ford angle-control platform type. A wire-schema
regression checks both that path and the existing stock angle telemetry.

Offline packet equivalence and native acceptance do not prove physical steering
response or identify an EPS duration limit. Vehicle validation remains a
separate closed-course step after pairing and startup verification.

## 8 November host correction and recording audit

The host-only count-boundary update retains the packaged
`DEV-c785bdc5-DEBUG` H7 and signature prefix `5de6e8a8352bce37` unchanged.
No native firmware code, driver override threshold, curvature authority,
command slew, automatic rearm or timing limit changes are included.

The 60-second capture contained 960 active requests across nine bursts, six
Panda-rejected packets and a longest active request of about 2.869 seconds.
It does not establish a rack duration timeout. The raw capture is private;
the regression fixtures retain only the relevant numeric boundary inputs.

One of two tracking stops was a floating-point comparison error: a candidate
of -42 counts against SCCM -0.2 was evaluated as
`abs(-2.1 - -0.1999999999998181) > 1.9`. Tracking and target margins are now
expressed in integer counts (38 and 36 respectively), while slew stays at one
count per update. The corrected boundary packet is accepted by the unchanged
compiled native safety checks. The other recorded stop has no valid candidate
inside the slew/tracking intersection and remains a latched stop. These
numeric comparisons do not validate the physical interpretation of PID*87.

Host telemetry now retains candidate reference, feasible bounds, tracking
error, yaw, measured curvature, torque and the blocked-TX count. The recorder
adds required native RX `0x213` (`DesiredTorqBrk`) and Park signal `0x230`.
The earlier recorder omitted `0x213`, so exact native replay of its six
rejections is not possible without inventing an input. The offline audit
reports that missing evidence explicitly:

```sh
python3 -m tools.ford_legacy_lka.replay --input /data/capture.json
```

Capture metadata includes current-boot firmware startup evidence. Verification
uses the latest current-boot connection entry and fails on a later mismatch;
an old boot or a packaged signature alone cannot verify connected firmware.
Host-only updates may reuse the same firmware across host commits.

### Installation with ignition power

Ignition can remain ON to power the comma. Select Park, disengage cruise and
select offroad mode. The installer listens only: it requires fresh Ford Park,
zero ABS speed with valid checksum/quality and advancing counters, cruise
disengaged, and live Panda `noOutput / 0` with controls disallowed. It does
not inspect ignition state. The parked proof must remain fresh for one second
and is repeated before checkout and before reboot. Missing or stale data,
moving/non-Park conditions or active controls stop installation. Tracked local
edits are preserved by refusing them. Legacy LKA remains disabled after update.

Vehicle verification of this update is pending. A recording cannot predict
how the vehicle would respond after correcting a previously latched stop.

## Zero-assistance correction to the investigation

The user confirmed no actual steering assistance in the latest tests.
Packet transmission and availability transitions do not establish steering.
See [the zero-actuation continuation](FUSION_ZERO_ACTUATION.md) for the
instruction-backed receive/contribution trace and its limits. The accompanying
stock-TJA status correction does not claim to restore steering. Keep Legacy
LKA OFF pending the command/prerequisite investigation.
