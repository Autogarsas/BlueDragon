"""Modern planner adapter for the pinned JDGarlock Fusion angle PID."""
import math

from cereal import log
from opendbc.car.ford.fusion_lka_pid import ReferencePIController
from opendbc.car.ford.fusion_lka_port import MAX_ANGLE, MAX_REFERENCE_RAW, REFERENCE_SCALE
from openpilot.selfdrive.controls.lib.latcontrol import LatControl


class LatControlFordFusionLKA(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    tuning = CP.lateralTuning.pid
    # Upstream get_steer_max was a platform setting. Bound that setting for
    # initial validation and use the same limit in the original anti-windup path.
    self.steer_max = MAX_REFERENCE_RAW * REFERENCE_SCALE / MAX_ANGLE
    self.pid = ReferencePIController((tuning.kpBP, tuning.kpV), (tuning.kiBP, tuning.kiV),
                                     k_f=tuning.kf, pos_limit=self.steer_max, sat_limit=CP.steerLimitTimer)
    self.angle_steers_des = 0.

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    # Modern openpilot supplies curvature, whereas the old path planner supplied
    # angleSteers. Use the existing angle-controller's planner/sign conversion.
    desired_angle = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    desired_angle += params.angleOffsetDeg
    return self.update_angle(active, CS, desired_angle, steer_limited_by_safety or curvature_limited)

  def update_angle(self, active, CS, desired_angle, steering_rate_limited=False):
    """The upstream PID path, also usable for independent reference fixtures."""
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)
    if CS.vEgo < .3 or not active:
      output_steer = 0.
      pid_log.active = False
      self.pid.reset()
    else:
      self.angle_steers_des = desired_angle
      self.pid.pos_limit, self.pid.neg_limit = self.steer_max, -self.steer_max
      check_saturation = CS.vEgo > 10 and not steering_rate_limited and not CS.steeringPressed
      output_steer = self.pid.update(self.angle_steers_des, CS.steeringAngleDeg,
                                     check_saturation=check_saturation, override=CS.steeringPressed,
                                     feedforward=self.angle_steers_des, speed=CS.vEgo, deadzone=0.)
      pid_log.active = True
      pid_log.p, pid_log.i, pid_log.f = float(self.pid.p), float(self.pid.i), float(self.pid.f)
      pid_log.output = float(output_steer)
      pid_log.saturated = bool(self.pid.saturated)
    pid_log.steeringAngleDesiredDeg = float(self.angle_steers_des)
    pid_log.angleError = float(self.angle_steers_des - CS.steeringAngleDeg)
    return float(output_steer), float(self.angle_steers_des), pid_log
