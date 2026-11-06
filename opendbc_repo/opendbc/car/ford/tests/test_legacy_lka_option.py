"""The research selector is reversible and cannot actuate an unverified protocol."""
import ast
from pathlib import Path
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.tests.test_factory_acc import car_state
from opendbc.car.ford.values import CAR, DBC, FordFlags


def params(candidate=CAR.FORD_EDGE_MK2, legacy=False, bluepilot=True, factory=True, alpha=False):
  flags = (structs.DPFlags.FordLegacyLKA if legacy else 0)
  flags |= structs.DPFlags.FordBluePilot if bluepilot else 0
  flags |= structs.DPFlags.FordStockLon if factory else 0
  return CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, False, flags, True)


class TestLegacyLKAOption(unittest.TestCase):
  def test_selected_research_mode_cannot_request_host_control(self):
    for candidate in CAR:
      for bluepilot in (False, True):
        for factory in (False, True):
          for alpha in (False, True):
            with self.subTest(candidate=candidate, bluepilot=bluepilot, factory=factory, alpha=alpha):
              cp = params(candidate, True, bluepilot, factory, alpha)
              self.assertTrue(cp.dashcamOnly)
              self.assertTrue(cp.flags & FordFlags.LEGACY_LKA)
              self.assertFalse(cp.flags & FordFlags.BLUEPILOT)
              self.assertTrue(cp.flags & FordFlags.STOCK_ACC)
              self.assertFalse(cp.openpilotLongitudinalControl)
              self.assertFalse(cp.alphaLongitudinalAvailable)
              self.assertFalse(cp.autoResumeSng)
              self.assertFalse(cp.steerAtStandstill)
              self.assertEqual([(structs.CarParams.SafetyModel.noOutput, 0)],
                               [(s.safetyModel, s.safetyParam) for s in cp.safetyConfigs])

  def test_direct_controller_call_cannot_bypass_research_gate(self):
    for candidate in (CAR.FORD_EDGE_MK2, CAR.FORD_MUSTANG_MACH_E_MK1):
      cp = params(candidate, legacy=True)
      controller = CarController(DBC[cp.carFingerprint], cp)
      cs = car_state(cp)
      cc = structs.CarControl(enabled=True, latActive=True, longActive=True)
      cc.actuators.curvature, cc.actuators.accel, cc.actuators.gas = 0.01, 2.0, 2.0
      cc.cruiseControl.cancel = cc.cruiseControl.resume = True
      self.assertIsNone(controller.bp_lateral)
      for tick in range(300):
        output, messages = controller.update(cc.as_reader(), cs, tick * 10_000_000)
        self.assertEqual([], messages)
        self.assertEqual((0.0, 0.0, 0.0), (output.curvature, output.accel, output.gas))

  def test_disabling_option_restores_original_params_exactly(self):
    for candidate in CAR:
      for bluepilot in (False, True):
        flags = structs.DPFlags.FordStockLon | (structs.DPFlags.FordBluePilot if bluepilot else 0)
        baseline = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, flags, True)
        params(candidate, legacy=True, bluepilot=bluepilot)
        restored = params(candidate, legacy=False, bluepilot=bluepilot)
        self.assertEqual(baseline.to_bytes(), restored.to_bytes())

  def test_ford_selection_does_not_change_pacifica(self):
    from opendbc.car.chrysler.interface import CarInterface as ChryslerInterface
    from opendbc.car.chrysler.values import CAR as ChryslerCAR
    baseline = ChryslerInterface.get_params(ChryslerCAR.CHRYSLER_PACIFICA_2018,
      gen_empty_fingerprint(), [], False, False, 0, True)
    selected = ChryslerInterface.get_params(ChryslerCAR.CHRYSLER_PACIFICA_2018,
      gen_empty_fingerprint(), [], False, False, structs.DPFlags.FordLegacyLKA, True)
    self.assertEqual(baseline.to_bytes(), selected.to_bytes())

  def test_registered_option_defaults_off_and_is_loaded_only_at_startup(self):
    # Read the registration source since the shipped aarch64 library cannot be
    # imported on an x86 development host. ARM CI tests actual storage semantics.
    root = Path(__file__).resolve().parents[5]
    self.assertIn('{"dp_ford_legacy_lka", {PERSISTENT, BOOL, "0"}}',
                  (root / "common/params_keys.h").read_text())
    source = (root / "selfdrive/car/card.py").read_text()
    tree = ast.parse(source)
    readers = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
               and isinstance(node.func, ast.Attribute) and node.func.attr == "get_bool"
               and node.args and isinstance(node.args[0], ast.Constant)
               and node.args[0].value == "dp_ford_legacy_lka"]
    self.assertEqual(1, len(readers))
    init = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "__init__")
    self.assertIn(readers[0], list(ast.walk(init)))


if __name__ == "__main__":
  unittest.main()
