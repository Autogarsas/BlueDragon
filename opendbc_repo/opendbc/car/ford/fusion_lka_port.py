"""Pinned Fusion LKA command path with a separate, bounded safety envelope.

Steering encoding and PID normalization follow JDGarlock/openpilot 5b5f7516.
The live encoder explicitly disables LDW requests and intensity after the
reported factory-style warning vibration; the donor's overlapping alert
nibble is retained only in the offline reference helper.
The upstream DBC calls the reference mrad; its controller feeds a normalized
PID output multiplied by 87. These numerical semantics are preserved, rather
than reinterpreting the reference as a new steering-wheel angle controller.
"""
from dataclasses import dataclass
import math

from opendbc.can import CANPacker
from opendbc.car.vehicle_model import VehicleModel

MAX_ANGLE = 87.
REFERENCE_SCALE = .05
MAX_REFERENCE_RAW = 128
MAX_CURVATURE_RAW = 300
REFERENCE_STEP = 1
CURVATURE_STEP = 1
MIN_SPEED = 15.
MAX_SPEED = 30.


def raw_reference(value):
  # Match CANPacker's unsigned DBC conversion, including half-count rounding.
  return int(math.floor((value + 102.4) / .05 + .5)) - 2048


def raw_curvature(value):
  return int(math.floor((value + .01024) / .000005 + .5)) - 2048


def reference_values(normalized_pid, desired_angle_deg, speed, vehicle_model):
  """Unbounded upstream active-command values, before the paired test envelope."""
  angle = min(max(normalized_pid * MAX_ANGLE, -MAX_ANGLE), MAX_ANGLE)
  curvature = vehicle_model.calc_curvature(math.radians(desired_angle_deg), speed, 0.)
  return {"Lkas_Action": 2, "Lkas_Alert": 0xe,
          "Lane_Curvature": min(max(curvature, -.03), .03), "Steer_Angle_Req": angle}


@dataclass(frozen=True)
class FusionLKACommand:
  action: int = 0
  reference_raw: int = 0
  curvature_raw: int = 0


def create_command(packer, bus, command):
  if command.action == 0:
    if command.reference_raw or command.curvature_raw:
      raise ValueError('Inactive Fusion LKA must be neutral')
    # Retain the already-observed OEM neutral action. The old fork writes 0xf
    # into a 3-bit field, which packs as reserved action 7; that is not imported.
    return 0x3CA, bytes.fromhex('0380080000000000'), bus
  if (command.action != 2 or abs(command.reference_raw) > MAX_REFERENCE_RAW or
      abs(command.curvature_raw) > MAX_CURVATURE_RAW):
    raise ValueError('Fusion LKA command outside the paired test envelope')
  return packer.make_can_msg('Lane_Keep_Assist_Control', bus,
                            {'Lkas_Action': 2,
                             'LdwActvStats_D_Req': 0, 'LdwActvIntns_D_Req': 0,
                             'LkaDrvOvrrd_D_Rq': 0, 'LaRampType_B_Req': 0,
                             'Lane_Curvature': command.curvature_raw * .000005,
                             'Steer_Angle_Req': command.reference_raw * .05})


class FusionLegacyLKAPort:
  def __init__(self, CP):
    self.vm = VehicleModel(CP)
    self.packer = CANPacker('ford_fusion_legacy_lka')
    self.reference_last = self.curvature_last = 0
    self.active_last = self.latched = False
    self.latch_reason = ''
    self.last_time = None
    self.last_blocked = None
    self.diagnostics = {'sequence': 0}

  def update(self, CC, CS, now_nanos):
    out = CS.out
    feedback = getattr(CS, 'legacy_lka_feedback', {})
    speed = float(out.vEgoRaw)
    wheel = float(feedback.get('wheel_angle_deg', math.nan))
    normalized = float(CC.actuators.torque)
    desired = float(CC.actuators.steeringAngleDeg)
    age = (now_nanos - min(feedback.get('time_nanos', -1), feedback.get('wheel_time_nanos', -1))) / 1e9
    interval = None if self.last_time is None else (now_nanos - self.last_time) / 1e9
    blocked = getattr(CS, 'legacy_lka_tx_blocked', None)
    rejected = self.active_last and blocked is not None and self.last_blocked is not None and blocked != self.last_blocked
    self.last_blocked = blocked
    reason = 'active'
    if not CC.enabled:
      self.latched = False
      self.latch_reason = ''
      reason = 'not_requested'
    elif self.latched:
      reason = 'latched_after_interruption'
    elif rejected:
      reason = 'panda_rejected'
    elif not getattr(CS, 'legacy_lka_panda_valid', False):
      reason = 'panda_pair_unavailable'
    elif not CC.latActive:
      reason = 'lateral_inactive'
    elif not all(math.isfinite(v) for v in (speed, wheel, normalized, desired, out.yawRate, out.steeringTorque)):
      reason = 'nonfinite_input'
    elif not out.canValid or out.vehicleSensorsInvalid:
      reason = 'invalid_vehicle_data'
    elif not 0 <= age <= .12:
      reason = 'stale_rack_status'
    elif feedback.get('availability') not in (2, 3) or feedback.get('denied', True):
      reason = 'rack_unavailable_or_denied'
    elif out.steerFaultTemporary or out.steerFaultPermanent:
      reason = 'rack_fault'
    elif out.steeringPressed or abs(out.steeringTorque) > .9:
      reason = 'driver_override'
    elif out.brakePressed or out.gasPressed or not out.cruiseState.enabled:
      reason = 'cruise_or_brake'
    elif not MIN_SPEED <= speed <= MAX_SPEED:
      reason = 'outside_test_speed_range'
    elif abs(out.yawRate / speed) > .0015:
      reason = 'outside_curvature_authority'
    elif interval is not None and not .005 <= interval <= .03:
      reason = 'command_interval'
    elif abs(wheel) > 5.5:
      reason = 'outside_reference_authority'

    command = FusionLKACommand()
    if reason == 'active':
      values = reference_values(normalized, desired, speed, self.vm)
      target_reference = min(max(raw_reference(values['Steer_Angle_Req']), -128), 128)
      # The numeric command basis follows the upstream encoder and SCCM input.
      # Both host and native Panda bound tracking error without a camera offset.
      lo = math.ceil((wheel - 1.8) / .05)
      hi = math.floor((wheel + 1.8) / .05)
      target_reference = min(max(target_reference, lo), hi)
      reference_anchor = self.reference_last if self.active_last else raw_reference(wheel)
      reference = int(min(max(target_reference, reference_anchor - 1), reference_anchor + 1))
      measured_curvature = out.yawRate / speed
      target_curvature = min(max(values['Lane_Curvature'], measured_curvature - .0009), measured_curvature + .0009)
      target_curvature = min(max(raw_curvature(target_curvature), -300), 300)
      curvature_anchor = self.curvature_last if self.active_last else raw_curvature(measured_curvature)
      curvature = int(min(max(target_curvature, curvature_anchor - 1), curvature_anchor + 1))
      if abs(reference) > 128 or abs(curvature) > 300 or abs(reference * .05 - wheel) > 1.9:
        reason = 'output_tracking_limit'
      else:
        command = FusionLKACommand(2, reference, curvature)

    if reason != 'active' and CC.enabled and self.active_last:
      self.latched = True
      self.latch_reason = reason
      if reason == 'cruise_or_brake':
        self.latch_reason = ('brake_pressed' if out.brakePressed else
                             'accelerator_pressed' if out.gasPressed else 'cruise_inactive')
    self.active_last = command.action != 0
    self.reference_last, self.curvature_last = command.reference_raw, command.curvature_raw
    self.last_time = now_nanos
    self.diagnostics = dict(sequence=self.diagnostics['sequence'] + 1, command_time_nanos=int(now_nanos),
                            enabled=bool(CC.enabled), lat_active=bool(CC.latActive), reason=reason,
                            action=command.action, reference_raw=2048 + command.reference_raw,
                            curvature_raw=2048 + command.curvature_raw, status_age_s=age,
                            availability=feedback.get('availability'), denied=feedback.get('denied'),
                            speed_mps=speed if math.isfinite(speed) else None,
                            steering_angle_deg=wheel if math.isfinite(wheel) else None,
                            requested_curvature=float(CC.actuators.curvature),
                            desired_angle_deg=desired if math.isfinite(desired) else None,
                            normalized_pid=normalized if math.isfinite(normalized) else None,
                            latched=self.latched, latch_reason=self.latch_reason,
                            reference_port='JDGarlock/5b5f7516', safety_param=40)
    return command
