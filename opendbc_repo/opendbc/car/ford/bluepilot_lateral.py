"""Ford curvature-primary controller adapted from BluePilot bp-5.0.

Source: BluePilotDev/bluepilot c6f8350f17c0622d7a075fe03ae760425e68fd2c
Original openpilot/BluePilot contributors, MIT; see LICENSE and docs/FORD_BLUEPILOT_PORT.md.
Pure calculations: messaging and persistent settings live in the dragonpilot adapter.
"""

from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from numpy import clip, interp
from opendbc.car.common.pid import PIDController
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.lateral import AngleSteeringLimits, apply_std_steer_angle_limits

T_IDXS = [10.0 * (i / 32.0) ** 2 for i in range(33)]
BP_ANGLE_LIMITS = AngleSteeringLimits(0.02, ([5, 16, 25], [0.0025, 0.0012, 0.00008]), ([5, 16, 25], [0.0025, 0.0014, 0.00018]))


@dataclass(frozen=True)
class LateralCommand:
  active: bool = False
  curvature: float = 0.0
  curvature_rate: float = 0.0
  path_angle: float = 0.0
  path_offset: float = 0.0
  ramp_type: int = 0
  precision_type: int = 1


def apply_ford_curvature_limits(requested, last, measured, speed, CP, diagnostics=None):
  before_deviation = requested
  if speed > 9:
    requested = clip(requested, measured - CarControllerParams.CURVATURE_ERROR, measured + CarControllerParams.CURVATURE_ERROR)
  result = apply_std_steer_angle_limits(requested, last, speed, 0, True, BP_ANGLE_LIMITS)
  if diagnostics is not None:
    diagnostics["deviation_limited"] = bool(requested != before_deviation)
    diagnostics["curvature_rate_limited"] = bool(result != requested)
  before_accel = result
  if CP.flags & FordFlags.CANFD:
    limit = (3.0 - 9.81 * 0.06) / max(speed, 1) ** 2
    result = clip(result, -limit, limit)
  if diagnostics is not None:
    diagnostics["lateral_accel_limited"] = bool(result != before_accel)
  return float(result)


class BluePilotLateral:
  def __init__(self, CP):
    self.CP = CP
    self.diagnostics = {"sequence": 0}
    # Initialize control variables
    self.apply_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.fcw_alert_last = False  # previous status of collision alert
    self.send_ui_last = False  # previous state of ui elements
    self.send_bars_ts_last = 0  # previous state of ui elements
    self.send_bars_last = False  # previous state of ACC Gap elements
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.accel_pitch_compensated = 0.0
    self.steering_wheel_delta_adjusted = 0.0
    self.last_button_frame = 0  # Track last ICBM button press frame

    ################################## lateral control parameters ##############################################

    # Variables to initialize (these get updated every scan as part of the control code)
    self.precision_type = 1  # precise or comfort
    self.human_turn = False  # have we detected a human override in a turn
    self.post_reset_ramp_active = False  # track if we're ramping after a steering reset
    self.reset_steering_last = False  # track previous reset_steering state
    self.enable_lane_positioning = False  # Updated from UI: enable Advanced Lane Positioning
    self.enable_high_curvature_mode = False  # Updated from UI: enable High Curvature Mode
    self.custom_profile = 0  # updated from UI
    self.pc_blend_ratio = 0.5
    self.steer_warning = False  # warning for steering limits exceeded
    self.steer_warning_count = 0  # count how many cycles the warning has existed
    self.steering_limited = 0  # count how many cycles the steering was limited
    self.disable_BP_lat_UI = False  # updated from UI: disable BP lateral control
    self.anti_overshoot_curvature_last = 0.0  # initialize anti_overshoot_curvature_last

    # Curvature variables
    self.curvature_lookup_time = 0.42  # from lagd
    self.lane_change_factor_bp = [4.4, 40.23]  # what speed to adjust lane_change_factor
    self.lane_change_factor_low = 0.95  # lane_change_factor at 4.4 m/s
    self.lane_change_factor_high = 0.85  # updated from UI: lane_change_factor at 40.23 m/s
    self.pc_blend_ratio_low_C_CAN = 0.40  # %-Predicted Curvature
    self.pc_blend_ratio_high_C_CAN = 0.40  # %-Predicted Curvature
    self.pc_blend_ratio_low_C_CANFD = 0.40  # %-Predicted Curvature
    self.pc_blend_ratio_high_C_CANFD = 0.40  # %-Predicted Curvature
    self.pc_blend_ratio_low_C_UI = 0.40  # Updated from UI: %-Predicted Curvature
    self.pc_blend_ratio_high_C_UI = 0.40  # Updated from UI: %-Predicted Curvature
    self.pc_blend_ratio_bp = [0.0, 0.001]  # curvature breakpoints in 1/m
    self.large_curve_factor_low = 1.0  # factor to reduce curvature for small curves
    self.large_curve_factor_high = 0.80  # factor to reduce curvature for large curves
    self.large_curve_factor_bp = [0.001, 0.02]  # curvature breakpoints in 1/m
    self.large_curve_factor_v = [self.large_curve_factor_low, self.large_curve_factor_high]  #  determine factor to reduce cu

    # Curvature rate variables
    self.curvature_rate_delta_t = 0.3  # [s] used in denominator for curvature rate calculation
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t / 0.05)))  # 0.3 seconds at 20Hz
    self.curvature_rate_speed_bp = [0.0, 14.5, 15.5]  # speed breakpoints in m/s
    self.curvature_rate_speed_v = [1.0, 1.0, 0.0]  # corresponding k_p values
    self.curvature_rate_PC_bp = [0.0, 0.008, 0.01]  # curvature breakpoints in 1/m
    self.curvature_rate_PC_v = [0.0, 0.0, 1.0]  # corresponding k_p values

    # path offset variables
    self.custom_path_offset = 0.0  # updated from UI: applies a custom offset to help with in-lane positioning
    self.path_offset_lookup_time = 0.2  # in seconds (from bp-2.1)
    self.lane_width_tolerance_factor = 0.75
    self.min_laneline_confidence_bp = [0.6, 0.8]
    self.enable_lanefull_mode = True

    # path angle shared variables
    self.path_angle_filter_samples = 3  # number of samples to use for the moving average filter
    self.path_angle_deque = deque(maxlen=self.path_angle_filter_samples)  # deque to hold the samples
    self.path_angle_wheel_angle_conversion = np.pi / 180  # degrees to radians

    # path angle low curvature variables
    self.LC_PID_GAIN_CAN = 5.0
    self.LC_PID_GAIN_CANFD_SMALL_VEHICLE = 3.0
    self.LC_PID_GAIN_CANFD_LARGE_VEHICLE = 3.0
    self.LC_PID_GAIN_UI = 0.0  # gain for UI tuning
    self.LC_PID_GAIN = 0.0
    self.LC_PID_k_p = 0.25
    self.LC_PID_k_i = 0.05
    self.LC_PID_controller = PIDController(k_p=self.LC_PID_k_p, k_i=self.LC_PID_k_i, rate=20)
    self.LC_PID_speed_bp = [0.0, 9.0, 15.0]  # speed breakpoints in m/s
    self.LC_PID_speed_v = [0.0, 0.0, 1.0]  # corresponding k_p values
    self.LC_path_angle_ROC_bp = [5, 15, 25]  # speed breakpoints in m/s
    self.LC_path_angle_ROC_v = [0.003, 0.0015, 0.002]  # match panda limits
    self.LC_path_angle_reset_counter = 0
    self.LC_path_angle_reset_duration = 1.5  # in seconds

    # path angle high curvature variables
    self.HC_PID_gain_UI = 0.5  # gain for UI tuning
    self.HC_PID_k_p = 1.0
    self.HC_PID_k_i = 0.05
    self.HC_PID_controller = PIDController(k_p=self.HC_PID_k_p, k_i=self.HC_PID_k_i, rate=20)
    self.wheel_angle_lookup_time = 0.05
    self.HC_PID_curvature_bp = [0.0, 0.008, 0.01, 0.02]  # curvature breakpoints in 1/m
    self.HC_PID_curvature_v = [0.0, 0.0, 1.0, 1.0]  # corresponding k_p values
    self.HC_PID_speed_bp = [0.0, 20.00, 22.00, 25.00]  # what speeds to adjust path_angle_speed_factor over.
    self.HC_PID_speed_v = [1.0, 1.0, 0.0, 0.0]
    self.pswa_blend_ratio = 1.0

    # max absolute values for all four signals
    self.path_angle_max = 0.25  # from dbc files
    self.path_offset_max = 2.0  # too much path offset causes issues
    self.curvature_max = 0.0115  # 0.02 is max from dbc files, but more than 0.012 can cause windup in big curves
    self.curvature_rate_max = 0.001023  # from dbc files

    # values from previous frame
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate = 0  # initialize curvature_rate

    # Logging variables

    # Lane change transition tracking
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.lane_change_last = False  # Track previous lane change state
    self.pre_lane_change_values = {"path_angle": 0.0, "path_offset": 0.0, "desired_curvature_rate": 0.0}

    # Maximum allowed changes per frame
    self.max_path_angle_change = 0.00125
    self.max_path_offset_change = 0.00125
    self.max_curvature_rate_change = 0.0001

    self.configure({})

  def handle_post_lane_change_transition(self, path_angle, path_offset, desired_curvature_rate):
    """
    Manages smooth transition of control variables after lane change
    Returns: Tuple of (path_angle, path_offset, desired_curvature_rate)
    """
    # Detect lane change completion (transition from True to False)
    if self.lane_change_last and not self.lane_change:
      self.post_lane_change_active = True
      self.post_lane_change_timer = 0
      # Store current values as starting point
      self.pre_lane_change_values = {
        "path_angle": 0.0,  # Start from zero since we're coming out of lane change
        "path_offset": 0.0,
        "desired_curvature_rate": 0.0,
      }

    # Update previous lane change state
    self.lane_change_last = self.lane_change

    # If we're in post-lane change state
    if self.post_lane_change_active:
      self.post_lane_change_timer += 1

      # Apply smooth transition using rate limiting
      new_path_angle = clip(
        path_angle,
        self.pre_lane_change_values["path_angle"] - self.max_path_angle_change,
        self.pre_lane_change_values["path_angle"] + self.max_path_angle_change,
      )

      new_path_offset = clip(
        path_offset,
        self.pre_lane_change_values["path_offset"] - self.max_path_offset_change,
        self.pre_lane_change_values["path_offset"] + self.max_path_offset_change,
      )

      new_curvature_rate = clip(
        desired_curvature_rate,
        self.pre_lane_change_values["desired_curvature_rate"] - self.max_curvature_rate_change,
        self.pre_lane_change_values["desired_curvature_rate"] + self.max_curvature_rate_change,
      )

      # Update stored values
      self.pre_lane_change_values = {"path_angle": new_path_angle, "path_offset": new_path_offset, "desired_curvature_rate": new_curvature_rate}

      # Exit transition state after 40 frames
      if self.post_lane_change_timer >= 160:
        self.post_lane_change_active = False

      return (new_path_angle, new_path_offset, new_curvature_rate)

    return (path_angle, path_offset, desired_curvature_rate)

  def configure(self, settings):
    from opendbc.car.ford.bluepilot_settings import SETTINGS

    for key, spec in SETTINGS.items():
      value = settings.get(key, spec["default"])
      if not isinstance(value, (int, float)) or not math.isfinite(value):
        value = spec["default"]
      if spec["type"] == "bool":
        value = bool(value)
      else:
        value = float(clip(value, spec.get("min", value), spec.get("max", value)))
      setattr(self, spec["attr"], value)

  def reset(self):
    self.apply_curvature_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.curvature_rate_deque.clear()
    self.path_angle_deque.clear()
    self.LC_PID_controller.reset()
    self.HC_PID_controller.reset()
    self.lane_change_last = False
    self.post_lane_change_active = False
    return LateralCommand()

  def update(self, CC, CS, model):
    # Observation only: diagnostic values never feed command calculations.
    self.diagnostics = {"sequence": self.diagnostics["sequence"] + 1, "reason": "inactive",
                        "deviation_limited": False, "curvature_rate_limited": False,
                        "lateral_accel_limited": False, "curvature_magnitude_limited": False,
                        "path_angle_rate_limited": False, "path_angle_magnitude_limited": False,
                        "curvature_rate_magnitude_limited": False}
    if not CC.latActive or model is None:
      if CC.latActive:
        self.diagnostics["reason"] = "missing_model"
      return self.reset()
    self.diagnostics["reason"] = "invalid_model"
    # Model arrays must be present, full length and finite before issuing any command.
    try:
      arrays = [model.orientationRate.z, model.position.y, model.laneLines[1].y, model.laneLines[2].y]
      if any(len(a) != 33 or not np.isfinite(a).all() for a in arrays):
        return self.reset()
      if not np.isfinite([CC.actuators.curvature, CS.out.vEgoRaw, CS.out.yawRate, CS.out.steeringAngleDeg, *model.laneLineProbs]).all():
        return self.reset()
    except (AttributeError, IndexError, TypeError):
      return self.reset()
    actuators = CC.actuators
    self.precision_type = 1
    steeringPressed = CS.out.steeringPressed
    steeringAngleDeg_PV = CS.out.steeringAngleDeg

    # determine tuning profile
    if self.custom_profile == 1:  # custom tuning profile
      self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_UI
      self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_UI
      self.LC_PID_GAIN = self.LC_PID_GAIN_UI

    elif self.CP.flags & FordFlags.CANFD:
      self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_CANFD
      self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_CANFD
      if self.CP.carFingerprint == CAR.FORD_ESCAPE_MK4_5 or self.CP.carFingerprint == CAR.FORD_MUSTANG_MACH_E_MK1:
        self.LC_PID_gain = self.LC_PID_GAIN_CANFD_SMALL_VEHICLE
      else:
        self.LC_PID_gain = self.LC_PID_GAIN_CANFD_LARGE_VEHICLE
    else:
      self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_CAN
      self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_CAN
      self.LC_PID_gain = self.LC_PID_GAIN_CAN

    self.pc_blend_ratio_v = [self.pc_blend_ratio_low_C, self.pc_blend_ratio_high_C]  # %-Predicted Curvature

    # calculate current curvature and model desired curvature
    current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)  # use canbus data to calculate current_curvature
    desired_curvature = actuators.curvature  # get desired curvature from model

    # Full model arrays were validated above.
    curvatures = np.array(model.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
    predicted_curvature = interp(self.curvature_lookup_time, T_IDXS, curvatures)

    # calculate blend ratio
    self.pc_blend_ratio = interp(abs(desired_curvature), self.pc_blend_ratio_bp, self.pc_blend_ratio_v)

    # equate requested_curvature to a blend of desired and predicted_curvature and apply curvature limits
    requested_curvature = (predicted_curvature * self.pc_blend_ratio) + (desired_curvature * (1 - self.pc_blend_ratio))

    # determine if a lane change is active
    if model.meta.laneChangeState == 1 or model.meta.laneChangeState == 2 or model.meta.laneChangeState == 3:
      self.lane_change = True
    else:
      self.lane_change = False

    # determine lane_change_factor based on speed
    lane_change_factor = interp(CS.out.vEgoRaw, self.lane_change_factor_bp, [self.lane_change_factor_low, self.lane_change_factor_high])

    # if changing lanes, modify curvature to smooth out the lane change
    if self.lane_change and (model.meta.laneChangeDirection == 1):  # if we are changing lanes to the left
      if requested_curvature < 0:  # and the curvature is taking us to the left
        requested_curvature = requested_curvature * lane_change_factor  # reduce the curvature to smooth out the lane change
      else:
        requested_curvature = requested_curvature  # if we are moving back right to correct for over travel, do not reduce curvature

      self.precision_type = 0  # use comfort mode

    if self.lane_change and (model.meta.laneChangeDirection == 2):  # if we are changing lanes to the right
      if requested_curvature > 0:  # and the curvature is taking us to the right
        requested_curvature = requested_curvature * lane_change_factor  # reduce the curvature to smooth out the lane change
      else:
        requested_curvature = requested_curvature  # if we are moving back left to correct for over travel, do not reduce curvature

      self.precision_type = 0  # use comfort mode

    # Determine if a human is making a turn and trap the value
    # if a human turn is active, reset steering to prevent windup
    if steeringPressed and abs(steeringAngleDeg_PV) > 45:
      self.human_turn = True
    else:
      self.human_turn = False

    # Determine when to reset steering
    if ((self.human_turn) and self.enable_human_turn_detection) or (CS.out.vEgoRaw < 0.1):
      reset_steering = 1
    else:
      reset_steering = 0

    if reset_steering:
      self.diagnostics["reason"] = "standstill" if CS.out.vEgoRaw < 0.1 else "human_turn"
      return self.reset()
    self.diagnostics.update(reason="active", requested_curvature=float(requested_curvature),
                            predicted_curvature=float(predicted_curvature), measured_curvature=float(current_curvature),
                            blend_ratio=float(self.pc_blend_ratio), lane_change=bool(self.lane_change))
    apply_curvature = apply_ford_curvature_limits(requested_curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw, self.CP, self.diagnostics)

    # detect if steering was limited (lanes changes always trigger, but complete just fine)
    if (requested_curvature != apply_curvature) and (not steeringPressed) and (not self.lane_change):
      self.steering_limited = self.steering_limited + 1
    else:
      self.steering_limited = 0

    # if steering was limited for 10 scans turn on steer_warning if above 15mph
    if self.steering_limited > 10 and CS.out.vEgoRaw > 7:
      self.steer_warning = True

    # latch steer_warning and count cycles before clearing
    if self.steer_warning and not self.steering_limited:
      self.steer_warning_count = self.steer_warning_count + 1

    # clear steer_warning after 10 counts of no steering limited
    if self.steer_warning_count > 10:
      self.steer_warning = False
      self.steer_warning_count = 0

    # compute curvature rate
    self.curvature_rate_deque.append(predicted_curvature)
    if len(self.curvature_rate_deque) > 1:
      delta_t = (
        self.curvature_rate_delta_t if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen else (len(self.curvature_rate_deque) - 1) * 0.05
      )
      desired_curvature_rate = (self.curvature_rate_deque[-1] - self.curvature_rate_deque[0]) / delta_t / max(0.01, CS.out.vEgoRaw)
    else:
      desired_curvature_rate = 0.0

    # calculate curvature rate PC factor
    curvature_rate_PC_factor = interp(abs(predicted_curvature), self.curvature_rate_PC_bp, self.curvature_rate_PC_v)
    desired_curvature_rate = desired_curvature_rate * curvature_rate_PC_factor

    # calcualte curvature rate speed factor
    curvature_rate_speed_factor = interp(CS.out.vEgoRaw, self.curvature_rate_speed_bp, self.curvature_rate_speed_v)
    desired_curvature_rate = desired_curvature_rate * curvature_rate_speed_factor

    # determine large curve factor
    large_curve_factor = interp(abs(requested_curvature), self.large_curve_factor_bp, self.large_curve_factor_v)

    # apply large curve factor to desired_curvature_rate
    desired_curvature_rate = desired_curvature_rate * large_curve_factor

    # no large curve factor in lane changes
    if self.lane_change:
      large_curve_factor = 1.0

    # if we are in a lane change, set the desired_curvature_rate to 0
    if self.lane_change:
      desired_curvature_rate = 0.0

    # get path offset from model.position.y
    path_offset_position = interp(self.path_offset_lookup_time, T_IDXS, model.position.y)

    # now get path offset from lanelines
    path_offset_lanelines = (model.laneLines[1].y[0] + model.laneLines[2].y[0]) / 2

    # determinie laneline width tolerance scaling factor
    laneline_width = model.laneLines[2].y[0] + (-model.laneLines[1].y[0])  # laneLines[1] is a negative value because it is left of the vehicle.
    laneline_width_tolerance = interp(laneline_width, [3.75, 4.25], [0.81, 0.59])  # 3.7 is the width of standard US lane in meters

    # determine laneline confidence
    laneline_confidence = min(model.laneLineProbs[1], model.laneLineProbs[2], laneline_width_tolerance)
    if not self.enable_lanefull_mode:
      laneline_confidence = 0.0

    # determine laneline path offset scale
    laneline_path_offset_scale = interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0])

    # get the total path_offset combining model and lanelines
    path_offset = (path_offset_position * (1 - laneline_path_offset_scale) + (path_offset_lanelines * laneline_path_offset_scale)) + self.custom_path_offset

    # no path_offset during lane changes (it will fight you until it swaps to new lane if you don't set to zero)
    if self.lane_change:
      path_offset = 0

    # Scale the UI gain to the original bp-2.1 tuning units.
    path_offset_error = path_offset * (self.LC_PID_gain_UI / 100)

    # determine speed factor
    LC_PID_speed_factor = interp(CS.out.vEgoRaw, self.LC_PID_speed_bp, self.LC_PID_speed_v)

    # apply speed factor to path_offset_error
    path_offset_error_adj = path_offset_error * LC_PID_speed_factor

    # if not using lane positioning, zero out path_offset_error_adj
    if not self.enable_lane_positioning:
      path_offset_error_adj = 0.0

    # Use path_angle to help with centering vehicle in lane, Use PID controller to calculate path_angle
    path_angle_low_c = self.LC_PID_controller.update(path_offset_error_adj)

    # if not using lane positioning, zero out path_angle_low_c (should be zeroed out in the PID controller, but just in case)
    if not self.enable_lane_positioning:
      path_angle_low_c = 0.0

    # rate limit path_angle_low_c for comfort
    path_angle_roc = interp(abs(CS.out.vEgoRaw), self.LC_path_angle_ROC_bp, self.LC_path_angle_ROC_v)
    before_path_angle_rate = path_angle_low_c
    path_angle_low_c = clip(path_angle_low_c, self.path_angle_last - path_angle_roc, self.path_angle_last + path_angle_roc)

    self.diagnostics["path_angle_rate_limited"] = bool(path_angle_low_c != before_path_angle_rate)

    # if the driver is applying consistent pressure to the steering wheel, reset the path_angle_low_c PID controller
    if steeringPressed:
      self.LC_path_angle_reset_counter = self.LC_path_angle_reset_counter + 1
    else:
      self.LC_path_angle_reset_counter = 0
    if self.LC_path_angle_reset_counter > self.LC_path_angle_reset_duration * 20:  # 20 scans per second
      self.LC_PID_controller.reset()

    # path_angle_high_c is not used in the current implementation
    path_angle_high_c = 0.0

    # sum path_angle_low_c and path_angle_high_c
    path_angle = path_angle_low_c + path_angle_high_c

    # Apply post lane change transition logic
    path_angle, path_offset, desired_curvature_rate = self.handle_post_lane_change_transition(path_angle, path_offset, desired_curvature_rate)

    self.diagnostics.update(
      curvature_magnitude_limited=bool(abs(apply_curvature) > self.curvature_max),
      path_angle_magnitude_limited=bool(abs(path_angle) > self.path_angle_max),
      curvature_rate_magnitude_limited=bool(abs(desired_curvature_rate) > self.curvature_rate_max),
      post_lane_change_transition=bool(self.post_lane_change_active))
    # clip all values to max.
    apply_curvature = clip(apply_curvature, -self.curvature_max, self.curvature_max)
    desired_curvature_rate = clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max)
    path_offset = clip(path_offset, -self.path_offset_max, self.path_offset_max)
    path_angle = clip(path_angle, -self.path_angle_max, self.path_angle_max)

    # bp-5.0 uses path angle for centering and always sends zero path offset.
    path_offset = 0.0

    ramp_type = 2

    self.apply_curvature_last = float(apply_curvature)
    self.curvature_rate_last = float(desired_curvature_rate)
    self.path_angle_last = float(path_angle)
    self.path_offset_last = 0.0
    return LateralCommand(True, float(apply_curvature), float(desired_curvature_rate), float(path_angle), 0.0, int(ramp_type), int(self.precision_type))
