"""Exercise the real Ford interface/controller with factory and OP speed control."""
import math
import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANParser, CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.tests.test_bluepilot import model
from opendbc.car.ford.values import CAR, DBC, FordFlags, FordSafetyFlags


def params(candidate=CAR.FORD_EDGE_MK2, factory=True, bluepilot=True, alpha=False):
  flags = structs.DPFlags.FordStockLon if factory else 0
  if bluepilot:
    flags |= structs.DPFlags.FordBluePilot
  return CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, False, dp_params=flags, docs=True)


def car_state(cp):
  parser = CANParser(DBC[cp.carFingerprint][Bus.pt], [], 0)
  out = structs.CarState(vEgo=20.0, vEgoRaw=20.0)
  out.cruiseState.available = True
  out.cruiseState.enabled = True
  return NS(out=out,
            buttons_stock_values=dict(parser.vl["Steering_Data_FD1"]),
            acc_tja_status_stock_values=dict(parser.vl["ACCDATA_3"]),
            lkas_status_stock_values=dict(parser.vl["IPMA_Data"]))


class TestFordFactoryACC(unittest.TestCase):
  def test_factory_ownership_and_matching_safety_flags(self):
    for candidate in CAR:
      for bluepilot in (False, True):
        for alpha in (False, True):
          with self.subTest(candidate=candidate, bluepilot=bluepilot, alpha=alpha):
            cp = params(candidate, bluepilot=bluepilot, alpha=alpha)
            self.assertFalse(cp.openpilotLongitudinalControl)
            self.assertFalse(cp.alphaLongitudinalAvailable)
            self.assertFalse(cp.autoResumeSng)
            self.assertTrue(cp.pcmCruise)
            self.assertTrue(cp.flags & FordFlags.STOCK_ACC)
            safety = cp.safetyConfigs[-1].safetyParam
            self.assertTrue(safety & FordSafetyFlags.STOCK_ACC)
            self.assertFalse(safety & FordSafetyFlags.LONG_CONTROL)
            self.assertEqual(bluepilot, bool(safety & FordSafetyFlags.BLUEPILOT))

  def test_default_mode_is_unchanged(self):
    for candidate in CAR:
      for alpha in (False, True):
        cp = params(candidate, factory=False, alpha=alpha)
        self.assertFalse(cp.flags & FordFlags.STOCK_ACC)
        self.assertFalse(cp.safetyConfigs[-1].safetyParam & FordSafetyFlags.STOCK_ACC)
        self.assertEqual(alpha or not cp.radarUnavailable, cp.openpilotLongitudinalControl)

  def test_saved_ford_option_does_not_change_pacifica(self):
    from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
    from opendbc.car.chrysler.values import CAR as ChryslerCAR
    baseline = ChryslerInterface.get_params(ChryslerCAR.CHRYSLER_PACIFICA_2018,
      gen_empty_fingerprint(), [], False, False, dp_params=0, docs=True)
    with_ford_setting = ChryslerInterface.get_params(ChryslerCAR.CHRYSLER_PACIFICA_2018,
      gen_empty_fingerprint(), [], False, False, dp_params=structs.DPFlags.FordStockLon, docs=True)
    self.assertEqual(baseline.to_bytes(), with_ford_setting.to_bytes())

  def test_no_accel_or_automatic_resume_even_if_requested(self):
    for candidate in (CAR.FORD_EDGE_MK2, CAR.FORD_MUSTANG_MACH_E_MK1):
      cp = params(candidate)
      controller = CarController(DBC[cp.carFingerprint], cp)
      cs = car_state(cp)
      cc = structs.CarControl(enabled=True, latActive=True, longActive=True)
      cc.cruiseControl.resume = True
      for frame in range(300):
        cc.actuators.accel = 2.0 * math.sin(frame / 10)
        controller.bp_model = model()
        output, messages = controller.update(cc.as_reader(), cs, frame * 10_000_000)
        self.assertNotIn(0x186, [a for a, _, _ in messages])
        self.assertNotIn(0x83, [a for a, _, _ in messages])
        self.assertEqual(0, output.accel)
        self.assertEqual(0, output.gas)

  def test_explicit_cancel_is_preserved(self):
    cp = params()
    controller = CarController(DBC[cp.carFingerprint], cp)
    cc = structs.CarControl()
    cc.cruiseControl.cancel = True
    _, messages = controller.update(cc.as_reader(), car_state(cp), 0)
    self.assertEqual({0, 2}, {bus for addr, _, bus in messages if addr == 0x83})
    self.assertNotIn(0x186, [a for a, _, _ in messages])

  def test_steering_messages_match_existing_mode(self):
    # Same lateral inputs produce exactly the same CAN bytes in both speed modes.
    for candidate in (CAR.FORD_EDGE_MK2, CAR.FORD_MUSTANG_MACH_E_MK1):
      for bluepilot in (False, True):
        cps = [params(candidate, factory=factory, bluepilot=bluepilot) for factory in (False, True)]
        controllers = [CarController(DBC[cp.carFingerprint], cp) for cp in cps]
        states = [car_state(cp) for cp in cps]
        cc = structs.CarControl(enabled=True, latActive=True, longActive=True)
        for frame in range(1000):
          cc.actuators.curvature = 0.001 * math.sin(frame / 40)
          cc.actuators.accel = math.sin(frame / 15)
          lateral = []
          for controller, cs in zip(controllers, states):
            controller.bp_model = model(cc.actuators.curvature)
            _, messages = controller.update(cc.as_reader(), cs, frame * 10_000_000)
            lateral.append([(a, d, b) for a, d, b in messages if a in (0x3D3, 0x3D6, 0x3CA, 0x3D8)])
          self.assertEqual(lateral[0], lateral[1], (candidate, bluepilot, frame))

  def test_factory_acc_display_and_fcw_preserved(self):
    cp = params()
    cs = car_state(cp)
    stock = cs.acc_tja_status_stock_values
    stock.update(AccTGap_D_Dsply=4, AccFllwMde_B_Dsply=1, AccMsgTxt_D2_Rq=3,
                 AccStopStat_D_Dsply=2, AccStopMde_B_Dsply=1, FcwVisblWarn_B_Rq=0)
    cc = structs.CarControl(latActive=True)
    cc.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.fcw
    cc.hudControl.leadDistanceBars = 1
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    for stock_fcw in (0, 1):
      stock["FcwVisblWarn_B_Rq"] = stock_fcw
      controller = CarController(DBC[cp.carFingerprint], cp)
      _, messages = controller.update(cc.as_reader(), cs, 0)
      parser = CANParser(DBC[cp.carFingerprint][Bus.pt], [], 0)
      parser.update([(0, [m for m in messages if m[0] == 0x18A])])
      result = parser.vl["ACCDATA_3"]
      original = CANParser(DBC[cp.carFingerprint][Bus.pt], [], 0)
      original.update([(0, [packer.make_can_msg("ACCDATA_3", 0, stock)])])
      for name, value in original.vl["ACCDATA_3"].items():
        if name != "Tja_D_Stat":
          self.assertEqual(value, result[name], name)


if __name__ == "__main__":
  unittest.main()
