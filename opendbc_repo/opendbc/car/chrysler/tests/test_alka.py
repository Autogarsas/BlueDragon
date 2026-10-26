import random
import unittest
from collections import defaultdict
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.chrysler.alka import PacificaALKA, pacifica_steering_allowed
from opendbc.car.chrysler.carcontroller import CarController
from opendbc.car.chrysler.carstate import CarState
from opendbc.car.chrysler.chryslercan import create_lkas_heartbeat
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, DBC, PACIFICA_ALKA_CARS, ChryslerFlags, ChryslerSafetyFlags


def params(candidate=CAR.CHRYSLER_PACIFICA_2018, enabled=True, fw=None):
  return CarInterface.get_params(candidate, defaultdict(dict), fw or [], False, False,
                                 structs.DPFlags.LatALKA if enabled else 0, False)


class TestPacificaALKA(unittest.TestCase):
  def test_all_pacifica_profiles_use_63_with_and_without_fw(self):
    fw = structs.CarParams.CarFw(ecu="eps", fwVersion=b"68410000")
    for candidate in PACIFICA_ALKA_CARS:
      for car_fw in ([], [fw]):
        cp = params(candidate, fw=car_fw)
        self.assertEqual(17.5, cp.minSteerSpeed)
        self.assertTrue(cp.flags & ChryslerFlags.PACIFICA_ALKA)
        self.assertTrue(cp.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED)
        self.assertEqual(ChryslerSafetyFlags.PACIFICA_ALKA, cp.safetyConfigs[0].safetyParam)
    self.assertAlmostEqual(3.8, params(enabled=False).minSteerSpeed)
    self.assertEqual(17.5, params(enabled=False, fw=[fw]).minSteerSpeed)

  def test_other_chrysler_platforms_unchanged(self):
    for candidate in (CAR.JEEP_GRAND_CHEROKEE, CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN, CAR.JEEP_CHEROKEE_5TH_GEN, CAR.DODGE_DURANGO):
      self.assertEqual(params(candidate, enabled=False).to_dict(), params(candidate).to_dict())

  def test_button_state(self):
    state = PacificaALKA()
    self.assertFalse(state.update(False, []))
    self.assertTrue(state.update(True, []))
    self.assertFalse(state.update(True, [1]))
    self.assertFalse(state.update(True, [1, 1, 0]))
    self.assertTrue(state.update(True, [1]))
    self.assertFalse(state.update(False, [0, 1]))
    self.assertFalse(state.update(False, [0]))
    self.assertTrue(state.update(True, []))
    self.assertTrue(state.update(True, [1, 0, 1, 0]))

  def test_host_eligibility(self):
    cs = structs.CarState(canValid=True, vEgo=20, gearShifter="drive")
    self.assertTrue(pacifica_steering_allowed(cs, True, 17.5))
    self.assertFalse(pacifica_steering_allowed(cs, False, 17.5))
    for field, value in (("canValid", False), ("vEgo", 17.5), ("standstill", True), ("gearShifter", "park"),
                         ("gearShifter", "neutral"), ("gearShifter", "reverse"), ("gearShifter", "unknown"),
                         ("doorOpen", True), ("seatbeltUnlatched", True), ("steerFaultTemporary", True), ("steerFaultPermanent", True)):
      bad = structs.CarState(**cs.to_dict())
      setattr(bad, field, value)
      self.assertFalse(pacifica_steering_allowed(bad, True, 17.5), (field, value))

  def test_heartbeat_changes_only_one_bit(self):
    packer = CANPacker(DBC[CAR.CHRYSLER_PACIFICA_2018][Bus.pt])
    parser = CANParser(DBC[CAR.CHRYSLER_PACIFICA_2018][Bus.pt], [("LKAS_HEARTBIT", 1)], 2)
    rng = random.Random(17)
    for i in range(1000):
      raw = rng.randbytes(5)
      parser.update([[(i + 1) * 10_000_000, [(0x2D9, raw, 2)]]])
      for enabled in (True, False):
        addr, data, bus = create_lkas_heartbeat(packer, parser.vl["LKAS_HEARTBIT"], enabled)
        expected = bytearray(raw)
        expected[3] = (expected[3] & 0xFE) | (not enabled)
        self.assertEqual((0x2D9, bytes(expected), 0), (addr, data, bus))

  def test_carstate_real_parsers_and_button_samples(self):
    cp = params()
    state = CarState(cp)
    parsers = state.get_can_parsers(cp)
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    # First pass registers the remaining lazy parser messages.
    state.update(parsers)
    for i, button in enumerate((0, 1, 1, 0, 1, 0)):
      for bus, parser in parsers.items():
        messages = []
        for addr in tuple(parser.addresses):
          values = {"ACC_AVAILABLE": 1} if addr == 0x1F4 else {"TOGGLE_LKAS": button} if addr == 0x330 else {}
          _, data, _ = packer.make_can_msg(addr, int(bus == Bus.cam) * 2, values)
          messages.append((addr, data, int(bus == Bus.cam) * 2))
        parser.update([[(i + 1) * 10_000_000, messages]])
      state.update(parsers)
      self.assertEqual((True, False, False, False, True, True)[i], state.lkas_on)
      self.assertIsNotNone(state.lkas_heartbit)

  def test_alka_disabled_does_not_add_parser_messages(self):
    cp = params(enabled=False)
    parsers = CarState.get_can_parsers(cp)
    self.assertEqual(set(), parsers[Bus.pt].addresses)
    self.assertEqual(set(), parsers[Bus.cam].addresses)
    state = CarState(cp)
    state.update(parsers)
    self.assertNotIn(0x330, parsers[Bus.pt].addresses)
    self.assertNotIn(0x2D9, parsers[Bus.cam].addresses)

  def test_controller_inactive_and_switch_off_zero_torque(self):
    cp = params()
    cc = structs.CarControl(latActive=True)
    cc.actuators.torque = 0.5
    cs = SimpleNamespace(out=structs.CarState(canValid=True, vEgo=20, gearShifter="drive"), lkas_on=True,
                         lkas_heartbit=None, lkas_car_model=-1, button_counter=0, auto_high_beam=0)
    controller = CarController(DBC[cp.carFingerprint], cp)
    for i in range(250):
      output, _ = controller.update(cc.as_reader(), cs, i * 10_000_000)
    self.assertGreater(output.torqueOutputCan, 0)
    for mode in ("switch", "below_speed", "invalid", "inactive", "fault"):
      with self.subTest(mode=mode):
        cs.lkas_on = mode != "switch"
        cs.out.vEgo = 17.5 if mode == "below_speed" else 20
        cs.out.canValid = mode != "invalid"
        cs.out.steerFaultTemporary = mode == "fault"
        cc.latActive = mode != "inactive"
        for _ in range(2):
          output, _ = controller.update(cc.as_reader(), cs, 0)
        self.assertEqual(0, output.torqueOutputCan)


if __name__ == "__main__":
  unittest.main()
