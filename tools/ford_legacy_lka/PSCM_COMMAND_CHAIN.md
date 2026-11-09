# PSCM command chain continuation — 8 November 2026

Physical steering remains unverified: the user felt no assistance during the
latest tests. A working legacy-LKA fork has not been established. The vehicle
and comma are unavailable for further inspection tonight. Factory LKA with
the comma disconnected is also unconfirmed from the user's recollection.

## What has been closed offline

| Stage | Evidence | Remaining limitation |
|---|---|---|
| Receive/decode | All 2,324 active recorded 0x3CA payloads decode through original callbacks/getters | Synthetic valid/fresh callback flags do not prove live ECU receipt |
| Reference geometry | Fifteen cases equate original LCA measured-angle curvature with the legacy reference/yaw calculation | Installed runtime variant and measured-angle basis need confirmation |
| Application submission | Original handler/controller exports mode and Q10 command to channel 0 | No live submission observation is in the existing recording |
| Shared manager | Original feedback and independent eligibility paths reproduced | Requested mode is distinct from acceptance |
| Contribution/assistance lookup | Original gate, aggregation and speed-dependent lookup executed | Held synthetic snapshots are not a motor/vehicle plant model |
| Physical steering | No execution established | Remains the required final result |

The geometry calculations are DC4F0..DC51A and DD838..DD888. In the
profile-0 model, the ratio parameter is 15.1015625 and the wheelbase parameter
is the recorded 2847. The LCA slice uses angle / ratio * 0.0175 *
1000 / wheelbase; the legacy reference term is reference *
speed_mps / wheelbase. This establishes their mathematical relationship.
It does not authorize multiplying the existing PI output or changing the
native wheel-tracking checks without confirming the live angle basis.

## Corrections and downstream results

The previous partial snapshots initialized the assistance curve at zero
core speed while supplying moving application speed. Their numerical commands
must not be interpreted as predicted real torque. The continuation initializes
the curve with the held moving speed, supplies a declared synthetic orientation
and filtered driver input, and follows 5A490 -> 582B6 -> 5E6E8..5E7F0.
Remaining synthetic core state and feedback are explicitly recorded locally.

For one recorded field/sensor snapshot held for 40/100 model updates,
both headers 0x40 and 0x43 produce LKA submissions 62/173 Q10 counts,
aggregate 62/173 and assistance-lookup outputs 0/5 raw units. The provisional
otherInhibit flag differs (1 versus 0); other tested LKA outputs match.
The exact feature identity of that separate flag remains unresolved.
This does not justify restoring warning intensity or claiming whole-vehicle
equivalence. Header 0x40 is retained.

At the moving-speed curve, some small contributions quantize to zero.
The lookup results are software units, not measured motor current or torque.
They do not establish the vehicle's first ineffective condition.

Suppression-word writes at 57B66/57B74 and 57BBA/57BC8 occur in the handler
called from OS code at 19388. The traced paths OR a context bit for inputs
0x0A/0x0B. The ordinary masks 2/6 also force status byte 0x48 through
66D80 (denied, unavailable, hands-off false), unlike the recorded drops.
This narrows that specific explanation; all runtime suppression is not ruled out.

## Actual rack DTC response found in the raw logs

The expanded replay retains 139,134 frames and 18,936 original state samples,
with the same 2,324 active requests. Native required-RX coverage is complete,
including 0x213. Exact microcontroller ordering is still unverified.

At 312.475..312.495 seconds after boot, native main-bus response 0x738 carries:

    First frame:  10 0b 59 02 cb c0 01 88
    Consecutive: 21 08 c2 3a 00 0a 00 00
    UDS payload: 59 02 cb c0 01 88 08 c2 3a 00 0a

The captured request is ReadDTCInformation / reportDTCByStatusMask, 19 02 8f.
It was already in the recording; this analysis sent no diagnostic request.

| DTC | Status | Interpretation at the recorded response |
|---|---|---|
| U0001:88 | 08 | Confirmed; TestFailed bit clear |
| U023A:00 | 0A | Confirmed and failed during that operation cycle; TestFailed bit clear |

Ford identifies U0001:88 as a high-speed CAN bus-off code in
[OEM diagnostic material](https://static.nhtsa.gov/odi/inv/2012/INRD-PE12017-56148P.pdf).
Ford's submitted diagnostic listing identifies U023A:00 as lost communication
with Image Processing Module A in
[this record](https://static.nhtsa.gov/odi/inv/2024/INRD-RQ24014-72027P1.pdf).
These sources support the code names, not an assertion that their other
vehicle-specific diagnostic procedures apply to this retrofit.
The status-bit distinction follows the
[AUTOSAR diagnostic definition](https://www.autosar.org/fileadmin/standards/R18-03_R1.4.0/AP/AUTOSAR_SWS_AdaptiveDiagnostics.pdf).

The strategy indexes the DTC-code table at 7D840, as seen at 6EF88..6EF96.
Event 54 maps to C00188. Events 59, 117 and 118 map to C23A00.
The receive descriptors associate those communication events with
0x3CA / callback 65412, 0x3D3 / 65E96 and 0x3D7 / 65DE8 respectively.
Period parameters are 30/50/50.

These are not the slot-15 diagnostic query indices 1/2/81/82 from the previous
eligibility trace. Do not equate the public confirmed-DTC bit with the internal
eligibility word's bit 3. The observed codes are useful history, not a proven
cause of zero assistance or a reason to bypass a diagnostic condition.
Preserve the records and snapshots when the vehicle is available.

## Read-only observability prepared in the fork

The passive collector now retains compensated yaw 0x77, steering companion
0x3D7, UI messages 0x18A/0x3D8 and diagnostic traffic 0x730/0x738.
It can passively assemble bounded DTC read responses and decode allowlisted
positive DID responses on the native main bus. It sends no requests, flow
control or steering commands and changes no Panda mode.

The following response mappings were executed against the original stock
read encoders. Values apply only after matching the installed firmware IDs.

| Read DID | Response meaning |
|---|---|
| 3302 or 3020 | GP-78AA + 7800, unsigned big-endian; internal angle = (raw-7800)/10 degrees. FFFF is unavailable. Capability gating differs. |
| EE42 | A mask of last submitted request modes; bit 4 means LKA channel 0's last submitted mode was 3. |
| EE02 | Signed core orientation byte; +1 or -1 are the initialized orientation values. |
| EE43 | Assistance-curve selector byte. |

**EE42 is not acceptance.** The original manager was exercised with return
values 3 and 0 while its stored submitted mode remained 3. Both cases produce
EE42 data 0010. Seeing this bit can establish application submission under
the matching firmware mapping, but cannot establish accepted contribution
or physical execution.

When the vehicle is available, the useful initial evidence is stationary,
read-only identification, current DTCs/snapshots, those DID responses and the
connected Panda's current-boot firmware identity. Compare the internal angle
with simultaneous SCCM data before any controller conversion. The present
0x85 compensated-angle DBC field is not a substitute: the alternate packet
format leaves it unused in the recording.

No additional moving test or actuation change follows from this report.
Keep Legacy LKA OFF. No working-steering deadline can be confirmed without
closing the vehicle-dependent stages.

## Validation

The local instruction trace passes fifteen geometry cases, eight suppression
setter cases, three status-encoding cases, twenty-four assistance lookup cases,
thirty diagnostic-encoder cases and the two held-header sequences.
The separate request-mode trace verifies the acceptance distinction.
Eleven focused passive-decoder/API checks pass locally. The full Linux/native
regressions run on the branch commit; consult its workflow result.

A local attempt to run the entire passive module was limited by this laptop's
missing native cereal import environment; the focused tests pass without
substituting that environment. No native safety, H7, steering authority,
driver override, ACC ownership or rearm behavior is changed.


## 9 November continuation

Factory physical assistance with comma disconnected is now user-confirmed.
See [INTERNAL_ANGLE.md](INTERNAL_ANGLE.md) for the original-encoder-backed
continuous internal-angle mapping and recorded companion replay. These close
parts of the feedback investigation without an adapter setup dependency.
The live controller and safety are unchanged; physical fork steering remains
unestablished.
