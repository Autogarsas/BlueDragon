# Fusion zero-actuation continuation â€” 8 November 2026

The user confirmed no physical steering assistance during the latest tests.
Requests, transmit echoes, status changes and software-model outputs do not
establish steering execution. Keep Legacy LKA OFF while this is investigated.

## Verified baseline and new firmware trace

The analysis continues the saved 2 November checkpoint, using stock strategy
K2GC-14D003-AH (SHA-256 df8c01c381689e8e3777aaa20132db7ce15a943cadbc3a9cb6e738270319ba39)
and calibration K2GC-14D007-CD
(8dd02f7fc8dc77d2d00ea20348d78aef9b4949955a4da9bda5f4d6e1d2771ab2).
The matching files pass VBF CRC32 and decoded-block CRC16. These are stock
downloads matched to the November identification, not an installed-flash readback.

The receiver initialization at 54782..5479C and dispatch mapping at
550BE..550E2 link 0x3CA to three selectable buffers. The tables start at BCD1C,
BCDD8 and BCEF4, with configuration offsets 0/33/63/93 at BC8DC.
For the three configurations the 0x3CA object IDs are 13/39/62 and buffers
FEDF2ACC/FEDF2BD4/FEDF2D1C. The original getters at 67648, 67708 and 67736
read a three-bit action, twelve-bit curvature and twelve-bit reference.

Routine 65412 and the input adapter D26EE..D2786 reproduce action 2 and zero
LDW from a live-style 0x40 header. The adapter scales reference by 0.05 with
offset -102.4 and curvature by 0.000005 with offset -0.01024. DD860..DD87A
then divides the reference by 1000 and combines it with speed and a converted
configuration parameter before subtracting yaw feedback. This supports the
milliradian interpretation; it does not validate using SCCM wheel degrees as
the numerical reference or establish a replacement controller conversion.

Twenty-four packet cases across the three selectors passed the original
receive/conversion instructions. Six synthetic nominal handler/controller
cases showed both command signs under both action 2 and action 4. A complete
D21EE wrapper plus original channel submission also produced a nonzero
software command with neutral LCA input in the synthetic ready context.
Thus these cases do not support guessing a direction rule or blaming action 2
or zero LDW alone. They are not vehicle validation.

## Acceptance and downstream contribution are distinct

Submission feedback 3 and a retained command at GP-7600 are followed by
additional processing. Routine 5A490 creates per-channel gates at GP-72A0.
For LKA channel 0, absence of feature bit 19 at GP+271C or suppression bit 1
at GP-7B48 rejects the channel's contribution. The original calibration
initialization 5A154 must precede this processing: omitting it leaves zero
model bounds and gives a misleading zero output.

With initialized stock bounds, a synthetic accepted request of 14 Q10 counts
contributed 14 through GP-7628 when enabled, and zero when either gate was
asserted. This is another reason acceptance does not prove actuation. The
vehicle's first ineffective condition has not yet been identified.

The saved November 1 UCDS sessions also contain configuration DIDs DE00 and
DE01. Read descriptors at BDD10/BDD20 link them to callbacks 6EAF0/6EAF6,
which select response encoders 70CAC/71390. The original encoders reproduce
both saved 24-byte responses. DE01 byte 6 reports feature bit 19 enabled,
and byte 5 reports bit 21 enabled. Both LKA and LCA were enabled in that
recorded configuration. This does not identify the November 8 runtime
suppression state and is not a reason to change configuration.

DE00 bytes 2..3 supply the numeric parameter 2847 at GP+29DC. Re-executing
D209E and DD5C8 with this recorded parameter reproduced the reference/yaw term:

    reference * (speed_kph / 3.6) / 2847 - yaw_feedback

For a decoded reference of 1.0 at 72 km/h, the term before yaw subtraction is
0.007024939 in the float32 model. Interpreting 2847 as millimetres of wheelbase
and the reference as road-wheel milliradians is consistent with this equation;
that physical interpretation remains an inference. No degrees-to-milliradians
multiplier or replacement live controller is introduced.

## Companion traffic

All three traced receive tables contain 0x3D3 but omit 0x3D8 and 0x18A.
That supports no direct UI-message input through those receive configurations;
it does not exclude indirect effects through other ECUs.

The neutral LCA request input can coexist with a nonzero LKA software command
in the synthetic ready context. Full vehicle arbitration and its diagnostics
are not established by this test. The heartbeat is retained.

The TJA status correction preserves the stock camera Tja_D_Stat whenever
LEGACY_LKA is selected. It corrects inaccurate status reporting while the
actual LCA command is inactive. It is not claimed to fix zero steering.
The documented cloud-only correction was absent from the laptop and from the
remote commit graph, so this change reconstructs its specified behavior.

The integration regression compares the complete ACCDATA_3 packet for all
eight raw TJA codes, both host request states, both main-switch states and
both stock FCW states. It also checks the existing non-LKA active status.
Steering requests, H7 source/binary, settings library, driver override,
authority, slew, factory ACC ownership and interruption/rearm policy are
unchanged.

## Recorded-byte replay

Browser access was enabled and the November 8 filtered capture and both raw
route segments were downloaded and retained locally. The full archive hash is
5daf376668ffa6805cfba1983aae1c6ee86656242606dbfed5389e9304f997d7.
The filtered capture hash is
63e8aa1c699baa08ddb07ce445cf060de05dffc243ec3bd43476b698cf096d48.
Private recordings are not included in this repository.

All 2,324 active 0x3CA packets in the filtered capture exactly match the full
raw send stream. The original 65412 callback and D26EE adapter decode their
actual payloads into the expected action, reference and curvature. These
checks use declared synthetic valid/fresh receive flags and nested-lock state;
they do not reproduce the physical CAN peripheral or prove ECU receipt.

The PSCM's yaw input is 0x77 BrakeSnData_3, not the host's 0x91 yaw field.
Receive object 25 points to FEDF2ABC. Getter 6AA2A, callback 64A4C, range getter
D02E6 and adapter D26B6..D26EE lead to GP-B658. The decoded twelve-bit input is:

    yaw_rad_s = (raw * 0.03663 - 75) * pi / 180

The original callback is warmed using recorded prehistory from 282 seconds
after boot; payload/scaling assertions cover the 302..363 second target window.
That window contains 3,044 valid raw yaw samples. It therefore does not show
a simply absent yaw prerequisite. Comparing this input with the latest 0x91
sample is descriptive only; it does not establish which difference, if any,
caused zero assistance. EPS status in the capture remains normal with no
reported EPAS failure; that status is not a steering-execution acknowledgement.

Six independent recorded packet/speed/yaw snapshots were held for forty model
updates each, using recorded configuration parameter 2847 and the complete
D21EE wrapper, D279E export, 57F54 submission and D2446..D245A feedback path.
With synthetic readiness and other unmeasured internal state, submitted
commands were 18, -7, 27, -4, 0 and -21 Q10 counts; all had feedback 3.
This is a conditional instruction-model result, not a time-aligned full-ECU
or actuator replay. It does not justify changing physical units, authority,
action direction, timer behavior or native safety limits.

The two long episodes end in measured availability drops about 7.055 and
7.044 seconds after continuing requests. The user felt no steering throughout.
These are status transitions, not proven steering-duration limits.

## Validation and remaining gap

Commit f7b7da54dbca3a5df5cc362e13d095f9efa4cbfd passed workflow
[42496678579](https://github.com/Autogarsas/BlueDragon/actions/runs/42496678579):
595 tests run, 551 passed and 44 inherited skips. The run also verified the
reused signed H7 package and the 170-key ARM settings library. The package is
unchanged; the connected Panda signature remains independently unverified.
The TJA correction is published on the experimental branch, not confirmed
installed on the device. Its effect on PSCM arbitration is unproven.

The first failed or ineffective actuation condition in the vehicle is still
unidentified. Remaining candidates require evidence of live eligibility,
post-submission suppression, downstream aggregation and the actual actuator
reference. GP-7B48 bit 1 is one demonstrated software gate, not an observed
vehicle failure. Neutral 0x3D3 coexistence in the model and the absence of
direct 0x3D8/0x18A receive mappings do not close whole-vehicle arbitration.

Continue tracing the downstream gate writers and actuator aggregation against
existing firmware before proposing any actuation change. Establish observable
read-only diagnostics for the missing state where possible. No additional
moving test or installation is recommended by these findings. Keep Legacy LKA
OFF. Factory ACC, driver override, native checks, limits and rearm rules stay
as published; no modified PSCM VBF is produced.
