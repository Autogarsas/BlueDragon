"""BP steering bounds, no post-reset bypass, and existing Ford safety regression tests."""

import unittest
import numpy as np
from opendbc.car.structs import CarParams
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.safety.tests import test_ford as stock


class BluePilotSafetyMixin:
  MAX_CURVATURE = 0.012
  ANGLE_RATE_BP = [5.0, 16.0, 25.0]
  ANGLE_RATE_UP = [0.0026, 0.0013, 0.0001]
  ANGLE_RATE_DOWN = [0.0026, 0.0015, 0.0002]

  def setUp(self):
    super().setUp()
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, self.PARAM | FordSafetyFlags.BLUEPILOT)
    self.safety.init_tests()

  def test_curvature_rate_limits(self):
    # Exercise all three breakpoints and both signs. Stay two CAN units away
    # from float32 interpolation boundaries (the stock two-point oracle uses
    # float64 and gives different rounding for this three-point table).
    unit = 1 / self.DEG_TO_CAN
    for speed in np.arange(0, 40, 0.5):
      for sign in (-1, 1):
        up = np.interp(speed - 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP)
        down = np.interp(speed - 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN)
        for initial, target, allowed in (
          (unit, unit + up - 2 * unit, True),
          (unit, unit + up + 3 * unit, False),
          (0.003, 0.003 - down + 2 * unit, True),
          (0.003, 0.003 - down - 3 * unit, False),
        ):
          self.safety.set_controls_allowed(True)
          self._reset_curvature_measurement(sign * (initial + target) / 2, speed)
          self._set_prev_desired_angle(sign * initial)
          low, high = self.get_canfd_curvature_limits(speed)
          if self.STEER_MESSAGE == stock.MSG_LateralMotionControl2:
            if abs(target) > high:
              allowed = False
            elif low <= abs(target) <= high:
              continue  # covered by inherited acceleration-limit tests
          with self.subTest(speed=speed, sign=sign, initial=initial, target=target):
            self.assertEqual(allowed, self._tx(self._lat_ctl_msg(True, 0, 0, sign * target, 0)))

  def test_controller_commands_pass_safety(self):
    from types import SimpleNamespace
    from opendbc.car import structs
    from opendbc.car.ford.tests.test_bluepilot import params, model
    from opendbc.car.ford.bluepilot_lateral import BluePilotLateral
    from opendbc.car.ford.values import CAR

    car = CAR.FORD_MUSTANG_MACH_E_MK1 if self.PARAM & FordSafetyFlags.CANFD else CAR.FORD_EDGE_MK2
    for speed in (5.0, 10.0, 16.0, 20.0, 25.0, 35.0):
      self.setUp()
      self.safety.set_controls_allowed(True)
      controller = BluePilotLateral(params(car))
      cc = structs.CarControl(latActive=True)
      cs = SimpleNamespace(out=structs.CarState(vEgoRaw=speed, vEgo=speed))
      measured = 0.0
      for frame in range(400):
        desired = 0.001 * np.sin(frame / 40.0)
        cc.actuators.curvature = float(desired)
        cs.out.yawRate = -measured * speed
        self._reset_curvature_measurement(-measured, speed)
        cmd = controller.update(cc, cs, model(float(desired), speed))
        with self.subTest(speed=speed, frame=frame):
          self.assertTrue(self._tx(self._lat_ctl_msg(cmd.active, -cmd.path_offset, -cmd.path_angle, -cmd.curvature, -cmd.curvature_rate)))
        measured = cmd.curvature

  def test_steer_allowed(self):
    for controls in (False, True):
      for active in (False, True):
        for curvature, angle, offset, rate in [(0, 0, 0, 0), (0.001, 0, 0, 0), (0, 0.001, 0, 0), (0, 0, 0.01, 0), (0, 0, 0, 0.0001)]:
          self.setUp()
          self.safety.set_controls_allowed(controls)
          self._reset_curvature_measurement(0, 5.0)
          expected = offset == 0 and ((controls and active) or (not active and curvature == angle == rate == 0))
          self.assertEqual(expected, self._tx(self._lat_ctl_msg(active, offset, angle, curvature, rate)))

  def test_reset_cannot_bypass_curvature_or_path_limits(self):
    for frame in range(65):
      for controls in (True, False):
        self.setUp()
        self.safety.set_controls_allowed(True)
        self._reset_curvature_measurement(0, 20.0)
        self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))
        for _ in range(frame):
          self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))
        self.safety.set_controls_allowed(controls)
        self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))

  def test_path_angle_rate_and_absolute_limit(self):
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 5.0)
    for angle in np.arange(0.002, 0.251, 0.002):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, float(angle), 0, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.252, 0, 0)))

  def test_rejected_path_frame_does_not_seed_next(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.202, 0, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.002, 0, 0)))

  def test_reinit_clears_path_history(self):
    self.safety.set_controls_allowed(True)
    for angle in np.arange(0.002, 0.102, 0.002):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, float(angle), 0, 0)))
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, self.PARAM | FordSafetyFlags.BLUEPILOT)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.1, 0, 0)))


class TestFordBluePilotCAN(BluePilotSafetyMixin, stock.TestFordLongitudinalSafety):
  PARAM = 0


class TestFordBluePilotCANFDStock(BluePilotSafetyMixin, stock.TestFordCANFDStockSafety):
  PARAM = FordSafetyFlags.CANFD


class TestFordBluePilotCANFDLong(BluePilotSafetyMixin, stock.TestFordCANFDLongitudinalSafety):
  PARAM = FordSafetyFlags.CANFD | FordSafetyFlags.LONG_CONTROL


if __name__ == "__main__":
  unittest.main()
