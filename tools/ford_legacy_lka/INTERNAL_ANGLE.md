# Internal-angle feedback mapping — 9 November 2026

The current Fusion port uses raw SCCM steering-wheel angle. The matching stock
PSCM exposes a different internal angle through DID 3302. This report establishes
a continuous CAN representation of that internal angle, as a prerequisite for
correct controller/reference work. It does not establish the cause of zero
actuation or change the live controller.

## Original instruction relationship

In K2GC-14D003-AH / K2GC-14D007-CD:

- 63462 stores a relative angle R at GP-78A2.
- 6346A..6347C stores A-R at GP-78A4 and marks its availability, where A is
  the internal angle at GP-78AA.
- 6719A..671B2 negates that offset, encoding R-A plus 32000.
- 6720E..67216 encodes R plus 32000.
- 6853C/6861E and the original byte-packers put these values in bytes 0..1
  and 6..7 of the alternate 0x85 payload.
- 67218..6724C computes a checksum over those four bytes and the counter.
- DID 3302 separately encodes A+7800. D0144 and D24EA convert A/10 into the
  application's angle, with signed Q5 quantization.

Thus, for available alternate-format fields:

    internal_angle_deg = (uint16_be(bytes[0:2]) - uint16_be(bytes[6:8])) / 10

The original fifteen-bit compensated-angle DBC field in bytes 2..3 is not used
for this reconstruction. The second term is an offset; subtract it from the
relative angle. Do not treat either term alone as the absolute internal angle.

The counter is the high nibble of byte 5; the checksum is byte 4:

    checksum = 255 - ((b0 + b1 + b6 + b7 + counter) mod 256)

The helper rejects unknown/unavailable 0xFFFE/0xFFFF angle fields, wrong length,
incompatible reserved-bit layout and checksum failures. This is an opt-in
interpretation for the traced firmware, not universal identification of 0x85.

## Validation

Fourteen original encoder/DID cases agree exactly for positive, negative and
zero internal angles. Fifty-six original encoder cases cover unknown offset,
a relevant diagnostic fault, unavailable mode and an uninitialized diagnostic
subsystem. These inputs exist only in model dictionaries; no firmware bytes or
vehicle state are edited.

The private November 8 replay has 6,101 native 0x85 packets: all checksums pass,
with 6,100 advancing counters and no duplicates or gaps. All angle fields are
available. Reconstruction therefore supplies a continuous angle from an input
already present in the recordings. Differences from SCCM vary; this does not
support a hard-coded correction.

Six local regression methods include original-machine fixtures, unavailable
fields with valid checksums, all single-bit corruptions, stale/non-native reads,
invalid events, backwards timestamps and counter wrap/skip reporting.

The companion study additionally checks 2,976 original LCA decoder cases across
all three receive selectors. All 1,220 camera LCA messages have request mode 0;
the recorded emergency-steering/object message also requests no intervention.
Six held companion contexts produce the same tested LKA output at the two
sampled update positions. Synthetic readiness, core orientation/profile and
remaining model inputs limit that comparison; it is not whole-vehicle proof.

## Remaining work toward steering

The broadcast reconstruction still needs comparison against DID 3302 in the
vehicle's own recordings. internal_angle.py can read recent local rlogs for
already-recorded paired data; no adapter setup or new diagnostic request is
needed:

    python3 tools/ford_legacy_lka/internal_angle.py --recent-logs 12

It uses native main-bus CAN timestamps, ignores repeated init/header timestamps,
does not pair across file boundaries, and reports both moving and held-window
differences. It does not turn agreement into a physical-actuation claim.

After that comparison, the controller's feedback basis and the reference-unit
conversion can be evaluated together with the unchanged native safety bounds.
Do not substitute a new angle or multiply PID output based only on a DBC label.

No live steering, native safety, H7 package, authority, driver override, factory
ACC ownership or rearm behavior is changed by this decoder. Legacy LKA remains
OFF. The latest user report confirms factory physical assistance with comma
disconnected; physical steering from this fork remains unestablished.