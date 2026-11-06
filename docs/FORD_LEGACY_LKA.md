# Legacy Ford LKA firmware research

Target vehicles are the owner's **2019 Ford Fusion Hybrid** and **2016 Lincoln
MKZ**. Their rack and camera configurations must be identified independently.
Selecting a vehicle label in these tools is not a car fingerprint or a claim
that the vehicle is supported for live LKA control.

## Current stage

The `ford-legacy-lka` branch provides passive CAN capture, two explicit DBC
interpretations, bounded offline byte fixtures, and regression tests. It does
**not yet implement a live LKA steering mode or a device settings toggle**.
Existing LCA/BluePilot, factory ACC, parameters and bundled Panda firmware are
unchanged. Capturing data does not require changing the installed branch.

This first stage gathers the evidence needed to implement a separately selected
LKA controller and its paired Panda limits. Copying the old fork directly would
leave an unresolved units/sign ambiguity on these two particular racks.

## Protocol evidence

Historical source: [JDGarlock/openpilot, ford-devel-master](https://github.com/JDGarlock/openpilot/tree/5b5f7516d813ec08c887183f9707ecd180306292).
The original [command generator](https://github.com/JDGarlock/openpilot/blob/5b5f7516d813ec08c887183f9707ecd180306292/selfdrive/car/ford/fordcan.py)
uses `Lane_Keep_Assist_Control` and `Steer_Angle_Req`.

The two DBCs already present in BlueDragon assign different physical meanings to
the same raw fields of 0x3CA:

| Field | Historical `ford_fusion_2018_pt` | `ford_lincoln_base_pt` |
| --- | --- | --- |
| Reference, raw 12 bits | `raw * 0.04297 - 88.00445` degrees | `raw * 0.05 - 102.4` mrad |
| Curvature, raw 12 bits | `raw * 0.000005 - 0.01` | `raw * 0.000005 - 0.01024` |
| First byte | 4-bit action + 4-bit alert | 3-bit intervention + LDW request/intensity |

These are DBC interpretations, not a verified steering-wheel/road-wheel
conversion for either test car. The tool reports both views instead of silently
converting between them. At 0x76 the DBCs even name different messages, so raw
sensor frames are retained without assuming the newer parser is correct.

The [Ford wiki](https://github.com/commaai/openpilot/wiki/Ford#lane-keep-assist-lka)
describes LKA availability on 0x3CC and the original approximately 10-second
timeout. A longer observed availability interval alone does not demonstrate
command execution or successful firmware modification.

## Collect a baseline without installing a new branch

While parked, download only the two standalone analysis files:

```bash
mkdir -p /data/ford-lka-research
curl -fL https://raw.githubusercontent.com/Autogarsas/BlueDragon/ford-legacy-lka/tools/ford_legacy_lka/protocol.py -o /data/ford-lka-research/protocol.py
curl -fL https://raw.githubusercontent.com/Autogarsas/BlueDragon/ford-legacy-lka/tools/ford_legacy_lka/capture.py -o /data/ford-lka-research/capture.py
```

Turn ignition on while parked. For the Fusion:

```bash
cd /data/openpilot
PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python3 /data/ford-lka-research/capture.py --vehicle fusion_2019 --label before --seconds 20 --output /data/fusion-lka-before.json
```

For the MKZ, connect the comma to that car and use:

```bash
cd /data/openpilot
PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python3 /data/ford-lka-research/capture.py --vehicle mkz_2016 --label before --seconds 20 --output /data/mkz-lka-before.json
```

These first parked captures identify available messages and cached ECU firmware
identification. They cannot establish the moving vehicle's steering authority
or the timeout. The report still records raw CAN if carState/carControl are
absent, as can happen on an unrecognized MKZ. Do not force the Edge profile onto
the MKZ merely to make the collector work.

Each report includes the selected vehicle label, installed branch/commit,
CarParams provenance, cached ECU IDs, raw 0x3CA/0x3CC and candidate sensor frames,
host requests separately from received CAN, and available car/Panda state.
Streams and buses stay separate. Invalid frames, denial and status gaps break
availability runs. An existing report is never overwritten; choose another
filename for repeat captures. Captures last 1–300 seconds and retain at most
100,000 selected frames; truncation and read failures are explicit.

The collector only opens messaging subscribers and local output files. It does
not open a direct Panda connection, publish CAN, write parameters, make UDS
requests, alter cruise settings or flash either module. Other software already
running on the device may still control the car; this tool does not change it.

Recorded local logs can be inspected with `--log /path/to/rlog.zst` instead of
live capture. qlogs may omit CAN or downsample it; missing frames and time gaps
must not be interpreted as proof of lockout. Cached ECU identity may be stale
after moving the comma between vehicles and is not a flash checksum.

## Next stage: live LKA implementation

Use the two captures plus the actual PSCM strategy/calibration and camera setup
to select each vehicle's signal layout, sign and angle conversion. If cached
firmware IDs are absent or stale, read the current module identification through
the owner's diagnostic tool before selecting a control configuration.

Then add a startup-selected LKA controller and matching Panda safety mode,
including independent bounds/rate/error checks, driver takeover/disengagement,
freshness and permission checks, and exclusion of simultaneous LCA commands.
Keep the current LCA mode as default and retain factory longitudinal ownership
for the initial LKA tests. This stage will require matched H7/settings binaries;
the current firmware deliberately rejects active legacy LKA requests.

For later PSCM firmware comparisons, retain original/patched image hashes and
the same vehicle configuration, requested commands, speed, measured wheel/yaw
response, driver input and faults. Availability bits, host TX counts or a
timeout disappearing from the UI alone are insufficient evidence of success.

## Validation

Run the passive, DBC-conformance and unchanged Panda-rejection tests:

```bash
PYTHONPATH=.:opendbc_repo python3 -m unittest tools.ford_legacy_lka.test_research tools.ford_legacy_lka.test_dbc tools.ford_legacy_lka.test_passive_safety
```

The branch also runs the existing Ford/factory-ACC/BluePilot and Pacifica suites
and verifies the unchanged bundled release. Local validation ran 509 tests
(465 passed, 44 inherited skips), including 22 research tests and 288 combinations
of active offline fixtures that the existing Panda safety correctly rejects.
No firmware-lockout fix or on-device capture validation is claimed.
