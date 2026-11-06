"""Limited-authority legacy LKA test controller; no timeout/rearm workaround.

Reference conversion deliberately follows the historical Ford LKA wheel-angle
encoder (0.04297 degree per raw count). The newer DBC calls the same raw field
milliradians. Neither label is treated as vehicle-validated calibration: command
authority is bounded in raw counts under both interpretations and logged as raw
counts. Direction/sign and response require the initial vehicle validation.
"""
from dataclasses import dataclass
import math

from opendbc.car.vehicle_model import VehicleModel

REFERENCE_SCALE = 0.04297  # historical equivalent steering-wheel degrees/count
MAX_REFERENCE_RAW = 128  # +/-5.50 historical degrees / +/-6.40 modern mrad
MAX_CURVATURE_RAW = 300  # +/-0.0015 m^-1
MAX_REFERENCE_ERROR_DEG = 1.8  # Panda independently enforces 2.0 degrees
MAX_REFERENCE_STEP = 2
MAX_CURVATURE_STEP = 2
MIN_SPEED = 15.0
MAX_SPEED = 30.0


@dataclass(frozen=True)
class LKACommand:
  action: int = 0
  reference_raw: int = 0  # signed counts about 2048
  curvature_raw: int = 0  # signed counts about 2048


def clamp(value, low, high):
  return min(max(value, low), high)


class LegacyLKALateral:
  def __init__(self, CP):
    self.vm = VehicleModel(CP)
    self.reference_last = 0
    self.curvature_last = 0
    self.active_last = False
    self.direction_last = 4
    self.last_time = None
    self.latched = False
    self.latch_reason = ""
    self.diagnostics = {"sequence": 0}

  def update(self, CC, CS, now_nanos):
    out = CS.out
    feedback = getattr(CS, "legacy_lka_feedback", {})
    speed = float(out.vEgoRaw)
    measured = float(feedback.get("wheel_angle_deg", math.nan))
    requested = float(CC.actuators.curvature)
    reason = "active"
    numeric = (speed, measured, requested, float(out.yawRate), float(out.steeringTorque))
    age = (now_nanos - min(feedback.get("time_nanos", -1), feedback.get("wheel_time_nanos", -1))) / 1e9
    interval = None if self.last_time is None else (now_nanos - self.last_time) / 1e9
    available = feedback.get("availability") in (2, 3) and not feedback.get("denied", True)

    if not CC.enabled:
      self.latched = False
      self.latch_reason = ""
      reason = "not_requested"
    elif self.latched:
      reason = "latched_after_interruption"
    elif not CC.latActive:
      reason = "lateral_inactive"
    elif not all(math.isfinite(x) for x in numeric):
      reason = "nonfinite_input"
    elif not out.canValid or out.vehicleSensorsInvalid:
      reason = "invalid_vehicle_data"
    elif not 0 <= age <= 0.12:
      reason = "stale_rack_status"
    elif not available:
      reason = "rack_unavailable_or_denied"
    elif out.steerFaultTemporary or out.steerFaultPermanent:
      reason = "rack_fault"
    elif out.steeringPressed or abs(out.steeringTorque) > 0.9:
      reason = "driver_override"
    elif out.brakePressed or out.gasPressed or not out.cruiseState.enabled:
      reason = "cruise_or_brake"
    elif not MIN_SPEED <= speed <= MAX_SPEED:
      reason = "outside_test_speed_range"
    elif abs(out.yawRate / speed) > 0.0015:
      reason = "outside_curvature_authority"
    elif interval is not None and not 0.02 <= interval <= 0.1:
      reason = "command_interval"
    elif abs(measured) > MAX_REFERENCE_RAW * REFERENCE_SCALE:
      reason = "outside_reference_authority"

    command = LKACommand()
    if reason == "active":
      # Preserve the current Ford port's CAN-curvature sign convention.
      target_curvature = clamp(-requested, -0.0015, 0.0015)
      target_curvature = clamp(target_curvature, out.yawRate / speed - 0.0009,
                                out.yawRate / speed + 0.0009)
      target_degrees = math.degrees(self.vm.get_steer_from_curvature(target_curvature, speed, 0.0))
      target_degrees = clamp(target_degrees, measured - MAX_REFERENCE_ERROR_DEG,
                              measured + MAX_REFERENCE_ERROR_DEG)
      reference = round(clamp(target_degrees / REFERENCE_SCALE, -MAX_REFERENCE_RAW, MAX_REFERENCE_RAW))
      anchor = self.reference_last if self.active_last else round(measured / REFERENCE_SCALE)
      reference = int(clamp(reference, anchor - MAX_REFERENCE_STEP, anchor + MAX_REFERENCE_STEP))
      curvature = round(target_curvature / 0.000005)
      curvature_anchor = self.curvature_last if self.active_last else round(out.yawRate / speed / 0.000005)
      curvature = int(clamp(curvature, curvature_anchor - MAX_CURVATURE_STEP,
                             curvature_anchor + MAX_CURVATURE_STEP))
      # Direction is based on change from the current wheel position, with a
      # small deadband to avoid chattering. No periodic command-off pulses.
      error = reference * REFERENCE_SCALE - measured
      direction = 2 if error < -0.10 else 4 if error > 0.10 else self.direction_last
      if abs(error) > 1.9 or abs(reference) > MAX_REFERENCE_RAW or abs(curvature) > MAX_CURVATURE_RAW:
        reason = "output_tracking_limit"
        self.latched = bool(CC.enabled and self.active_last)
        if self.latched:
          self.latch_reason = reason
        self.reference_last = self.curvature_last = 0
      else:
        command = LKACommand(direction, reference, curvature)
        self.reference_last, self.curvature_last = reference, curvature
        self.direction_last = direction
    else:
      if CC.enabled and self.active_last:
        self.latched = True
        self.latch_reason = reason
        if reason == "cruise_or_brake":
          self.latch_reason = ("brake_pressed" if out.brakePressed else
                               "accelerator_pressed" if out.gasPressed else "cruise_inactive")
      self.reference_last = self.curvature_last = 0

    self.active_last = command.action != 0
    self.last_time = now_nanos
    self.diagnostics = dict(sequence=self.diagnostics["sequence"] + 1,
      command_time_nanos=int(now_nanos), lat_active=bool(CC.latActive), reason=reason,
      enabled=bool(CC.enabled),
      requested_curvature=requested if math.isfinite(requested) else None,
      action=command.action, reference_raw=2048 + command.reference_raw,
      curvature_raw=2048 + command.curvature_raw, status_age_s=age,
      availability=feedback.get("availability"), denied=feedback.get("denied"),
      speed_mps=speed if math.isfinite(speed) else None,
      steering_angle_deg=measured if math.isfinite(measured) else None,
      latched=self.latched, latch_reason=self.latch_reason)
    return command
