# Pacifica always-on lateral (development)

This adapts BluePilot bp-5.0 / sunnypilot MADS Chrysler switch and heartbeat
handling to dragonpilot's existing ALKA framework. It is opt-in through the
existing **Always-on Lane Keeping Assist (ALKA)** setting, followed by a full
device reboot. No parameter-library rebuild is required.

## Behavior

- Initial release: Chrysler Pacifica profiles only, including the 2017–18
  nonhybrid profile named `CHRYSLER_PACIFICA_2018`.
- ACC Main rising enables lateral; the physical LKAS button toggles lateral;
  ACC Main off disables lateral. The switch also disables lateral during ACC.
- ACC cancel and braking retain lateral when LKAS remains enabled. Neither
  brake release nor cruise engagement re-arms a switch the driver turned off.
- Steering requires speed **above 63 km/h**, valid CAN/model/calibration/driver
  monitoring inputs, drive/low gear, closed doors, latched belt, and no EPS fault
  or driver-monitoring lockout. Longitudinal engagement remains separate.
- The 63 km/h minimum applies to all Pacifica profiles while this feature is
  enabled, including older EPS firmware and manual selection without firmware
  detection. With ALKA off, existing firmware detection and limits are unchanged.
- Normal EPS control-bit cooldown, steering torque/rate/measurement limits,
  cruise-button restrictions, and relay checks remain in place.

## CAN and safety integration

CarParams sets Pacifica flag `2` and Chrysler safety parameter bit `4` only for
supported Pacifica profiles with `DPFlags.LatALKA`. Panda additionally requires
the existing alternative-experience ALKA flag (`1024`). RAM/CUSW/Jeep/Durango
do not receive the new flags or switch behavior.

The physical switch is `TRACTION_BUTTON` (0x330), bus 0, bit 53. ACC Main is
`DAS_3` (0x1F4), bus 0, bit 20. Both host and Panda track these independently.
In ALKA mode only, the camera's `LKAS_HEARTBIT` (0x2D9), bus 2, is replaced by
a 10 Hz host message on bus 0. Only byte 3 bit 0 (`LKAS_DISABLED`) changes;
auto high beam and all other bytes are preserved. Panda verifies preservation.

The low-rate switch and camera messages are checked for bus, length, reception,
and age (10 seconds). They have no DBC checksum/counter. The existing critical
RX checks remain active; ALKA additionally checks that every required steering
input was received validly within one second. Panda requires both raw wheel
speeds at least 247 counts (`0.071028 m/s` per count), conservatively above
63 km/h. Nonzero torque is rejected if any prerequisite is missing, even if
normal ACC is active. ALKA never sets `controls_allowed` for longitudinal control.

## Validation and packaging

Native tests cover physical switches, speed boundaries, startup, stale/bad CAN,
heartbeat preservation, EPS/vehicle eligibility, and unchanged non-Pacifica
parameters. Existing Chrysler and Ford safety/controller suites also run.
The Ford baseline equivalence tool compares 10,000 CAN/controller frames.

The feature source branch must not be installed until a matching signed H7
application and verified build manifest are packaged. The old Ford-only H7
application does not implement these Pacifica permissions. The GitHub workflow
builds an application artifact without flashing hardware or changing bootstubs.

No hardware/road validation of this new feature is claimed. Initial device
validation must verify the flashed application signature, detected profile,
63 km/h minimum, physical switch behavior, and absence of CAN/EPS faults.
The stable `bluedragon-1.0` branch remains the prior Ford baseline.
