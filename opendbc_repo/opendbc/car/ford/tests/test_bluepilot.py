import unittest
from types import SimpleNamespace as NS

from opendbc.car import gen_empty_fingerprint, structs, Bus
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags, FordSafetyFlags
from opendbc.car.ford.bluepilot_lateral import BluePilotLateral, LateralCommand
from opendbc.car.ford.bluepilot_settings import SETTINGS
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.values import DBC
from opendbc.can import CANPacker


def params(car=CAR.FORD_EDGE_MK2, enabled=True):
  return CarInterface.get_params(car, gen_empty_fingerprint(), [], False, False, dp_params=structs.DPFlags.FordBluePilot if enabled else 0, docs=True)


def model(curvature=0.001, speed=20.0, offset=0.0):
  return NS(
    orientation=NS(x=[0.0] * 33),
    orientationRate=NS(z=[curvature * speed] * 33),
    position=NS(y=[offset] * 33),
    laneLines=[NS(y=[y + offset] * 33) for y in (-5.0, -1.8, 1.8, 5.0)],
    laneLineProbs=[1.0] * 4,
    meta=NS(laneChangeState=0, laneChangeDirection=0),
  )


class TestBluePilotController(unittest.TestCase):
  def setUp(self):
    self.CP = params()
    self.lat = BluePilotLateral(self.CP)
    self.cc = structs.CarControl(latActive=True)
    self.cc.actuators.curvature = 0.001
    self.cs = NS(out=structs.CarState(vEgoRaw=20.0, vEgo=20.0))

  def test_startup_safety_pair(self):
    self.assertTrue(self.CP.flags & FordFlags.BLUEPILOT)
    self.assertTrue(self.CP.flags & FordFlags.ALT_STEER_ANGLE)
    self.assertTrue(self.CP.safetyConfigs[-1].safetyParam & FordSafetyFlags.BLUEPILOT)
    stock = params(enabled=False)
    self.assertFalse(stock.safetyConfigs[-1].safetyParam & FordSafetyFlags.BLUEPILOT)
    self.assertEqual(stock.openpilotLongitudinalControl, self.CP.openpilotLongitudinalControl)
    self.assertAlmostEqual(self.CP.steerActuatorDelay, 0.22, places=6)

  def test_default_blend(self):
    # bp-5.0 defaults: 40% predicted + 60% actuator curvature.
    md = model(0.002)
    for _ in range(80):
      result = self.lat.update(self.cc, self.cs, md)
    self.assertAlmostEqual(result.curvature, 0.0014, places=7)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.path_offset, 0.0)
    self.assertTrue(result.active)
    self.assertEqual(result.ramp_type, 2)

  def test_missing_invalid_and_disengaged(self):
    for md in (None, model(float("nan")), model(float("inf"))):
      self.assertEqual(self.lat.update(self.cc, self.cs, md), LateralCommand())
    md = model()
    md.position.y = []
    self.assertEqual(self.lat.update(self.cc, self.cs, md), LateralCommand())
    self.cc.latActive = False
    self.assertEqual(self.lat.update(self.cc, self.cs, model()), LateralCommand())

  def test_human_turn_is_inactive_and_recovers_with_limits(self):
    self.cs.out.steeringPressed = True
    self.cs.out.steeringAngleDeg = 60.0
    self.assertEqual(self.lat.update(self.cc, self.cs, model()), LateralCommand())
    self.cs.out.steeringPressed = False
    cmd = self.lat.update(self.cc, self.cs, model())
    self.assertTrue(cmd.active)
    self.assertLessEqual(abs(cmd.curvature), 0.0014)

  def test_standstill(self):
    self.cs.out.vEgoRaw = 0.0
    self.assertEqual(self.lat.update(self.cc, self.cs, model()), LateralCommand())

  def test_settings_are_finite_and_bounded(self):
    self.lat.configure({k: float("nan") for k in SETTINGS})
    self.assertEqual(self.lat.pc_blend_ratio_high_C_UI, 0.4)
    self.lat.configure({k: 1000 for k in SETTINGS})
    self.assertEqual(self.lat.pc_blend_ratio_high_C_UI, 1.0)
    self.assertEqual(self.lat.custom_path_offset, 0.5)

  def test_lane_positioning_and_transition(self):
    self.lat.configure({k: True for k, v in SETTINGS.items() if v["attr"] == "enable_lane_positioning"})
    md = model(offset=0.2)
    for _ in range(30):
      cmd = self.lat.update(self.cc, self.cs, md)
    self.assertGreater(cmd.path_angle, 0)
    self.assertEqual(cmd.path_offset, 0)
    md.meta.laneChangeState = 2
    md.meta.laneChangeDirection = 2
    self.assertEqual(self.lat.update(self.cc, self.cs, md).precision_type, 0)
    md.meta.laneChangeState = 0
    for _ in range(170):
      cmd = self.lat.update(self.cc, self.cs, md)
      self.assertLessEqual(abs(cmd.path_angle), 0.25)

  def test_no_model_emits_no_steering_request(self):
    cc = CarController(DBC[self.CP.carFingerprint], self.CP)
    cs = NS(out=self.cs.out, acc_tja_status_stock_values={"Tja_D_Stat": 0}, buttons_stock_values={}, lkas_status_stock_values={})
    # Choose a lateral tick that does not send HUD or longitudinal messages.
    cc.frame = 5
    cc.lkas_enabled_last = True
    cc.lead_distance_bars_last = self.cc.hudControl.leadDistanceBars
    _, msgs = cc.update(self.cc.as_reader(), cs, 0)
    self.assertEqual(len(msgs), 1)
    self.assertEqual(msgs[0][0], 0x3D3)
    self.assertEqual((msgs[0][1][4] >> 2) & 7, 0)

  def test_edge_reference_required_before_valid_angle(self):
    state = CarState(self.CP)
    packer = CANPacker("ford_lincoln_base_pt")
    parsers = state.get_can_parsers(self.CP)

    def update(reference):
      packets = [packer.make_can_msg("SteeringPinion_Data_Alt", 0, {"StePinRelInit_An_Sns": 10.0}), packer.make_can_msg("ParkAid_Data", 0, reference)]
      parsers[Bus.pt].update([(0, packets)])
      return state.update(parsers)

    out = update({"EPASExtAngleStatReq": 1, "ExtSteeringAngleReq2": 20.0})
    self.assertTrue(out.vehicleSensorsInvalid)
    out = update({"EPASExtAngleStatReq": 0, "ExtSteeringAngleReq2": 20.0, "ApaSys_D_Stat": 0})
    self.assertFalse(out.vehicleSensorsInvalid)
    self.assertAlmostEqual(out.steeringAngleDeg, 20.0)


if __name__ == "__main__":
  unittest.main()
