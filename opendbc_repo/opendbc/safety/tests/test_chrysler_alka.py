"""Pacifica ALKA permission is independent of longitudinal controls_allowed."""
import unittest

from opendbc.car import structs
from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py


class TestPacificaALKASafety(unittest.TestCase):
  # Used by the common cross-mode test registry; these tests exercise the
  # additional, stateful ALKA whitelist explicitly below.
  TX_MSGS = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.chrysler, int(ChryslerSafetyFlags.PACIFICA_ALKA))
    self.safety.init_tests()
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ALKA)
    self.packer = CANPackerSafety("chrysler_pacifica_2017_hybrid_generated")
    self.safety.set_timer(1000)

  def rx(self, name, values, bus=0):
    return self.safety.safety_rx_hook(self.packer.make_can_msg_safety(name, bus, values))

  def main(self, enabled=True, active=False):
    return self.rx("DAS_3", {"ACC_AVAILABLE": enabled, "ACC_ACTIVE": active})

  def button(self, pressed=False):
    return self.rx("TRACTION_BUTTON", {"TOGGLE_LKAS": pressed})

  def inputs(self, speed=20, main=True, active=False):
    self.rx("EPS_2", {"EPS_TORQUE_MOTOR": 0})
    self.rx("ESP_1", {"Brake_Pedal_State": 0})
    self.rx("ECM_5", {"Accelerator_Position": 0})
    self.rx("SPEED_1", {"SPEED_LEFT": speed, "SPEED_RIGHT": speed})
    self.main(main, active)

  def prime(self, speed=20, active=False):
    self.inputs(speed, active=active)
    self.button()
    self.rx("LKAS_HEARTBIT", {}, 2)

  def steer(self, torque=3, previous=0):
    self.safety.set_desired_torque_last(previous)
    self.safety.set_rt_torque_last(previous)
    self.safety.set_torque_meas(previous, previous)
    msg = self.packer.make_can_msg_safety("LKAS_COMMAND", 0, {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": 1})
    return self.safety.safety_tx_hook(msg)

  def test_requires_both_opt_in_flags(self):
    self.prime()
    self.assertTrue(self.steer())
    self.safety.set_alternative_experience(0)
    self.assertFalse(self.steer())
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.chrysler, 0)
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ALKA)
    self.prime()
    self.assertFalse(self.safety.get_alka_allowed())
    self.assertFalse(self.steer())

  def test_lateral_only_and_cruise_buttons(self):
    self.prime()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.steer())
    resume = self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, {"ACC_Resume": True})
    cancel = self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, {"ACC_Cancel": True})
    self.assertFalse(self.safety.safety_tx_hook(resume))
    self.assertTrue(self.safety.safety_tx_hook(cancel))

  def test_switch_toggles_on_rising_edges_only(self):
    self.prime()
    self.assertTrue(self.steer())
    self.button(True)
    self.assertFalse(self.steer())
    self.button(True)
    self.assertFalse(self.steer())
    self.button(False)
    self.button(True)
    self.assertTrue(self.steer())
    self.main(False)
    self.assertFalse(self.steer())
    self.main(True)
    self.assertTrue(self.steer())

  def test_lkas_off_blocks_even_while_acc_active(self):
    self.prime(active=True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.button(True)
    self.assertFalse(self.steer())
    self.assertTrue(self.steer(0))

  def test_cancel_and_pedals_do_not_rearm_lkas(self):
    self.prime(active=True)
    self.main(True, False)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.steer())
    self.rx("ESP_1", {"Brake_Pedal_State": 1})
    self.assertTrue(self.steer())
    self.button(True)
    self.rx("ESP_1", {"Brake_Pedal_State": 0})
    self.rx("ECM_5", {"Accelerator_Position": 1})
    self.main(True, True)
    self.assertFalse(self.steer())

  def test_63_kmh_limit(self):
    for speed, allowed in ((0, False), (3.8, False), (17.46875, False), (17.5, False), (17.53125, True), (25, True)):
      with self.subTest(speed=speed):
        self.setUp()
        self.prime(speed)
        self.assertEqual(allowed, self.steer())
    self.rx("SPEED_1", {"SPEED_LEFT": 20, "SPEED_RIGHT": 17.5})
    self.assertFalse(self.steer())

  def test_startup_requires_all_inputs(self):
    self.inputs()
    self.assertFalse(self.steer())
    self.button()
    self.assertFalse(self.steer())
    self.rx("LKAS_HEARTBIT", {}, 2)
    self.assertTrue(self.steer())

  def test_stale_critical_inputs(self):
    self.prime()
    self.safety.set_timer(1001001)
    self.assertFalse(self.steer())
    self.assertTrue(self.steer(0))

  def test_stale_auxiliary_inputs(self):
    for stale in ("button", "heartbeat"):
      with self.subTest(stale=stale):
        self.setUp()
        self.prime()
        self.safety.set_timer(10001001)
        self.inputs()
        if stale == "button":
          self.rx("LKAS_HEARTBIT", {}, 2)
        else:
          self.button()
        self.assertFalse(self.steer())

  def test_auxiliary_wrong_bus_or_length(self):
    self.inputs()
    for addr, bus, length in ((0x330, 2, 8), (0x330, 0, 7), (0x2D9, 0, 5), (0x2D9, 2, 4)):
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, bytes(length)))
    self.assertFalse(self.steer())

  def test_bad_checksum_blocks_steering(self):
    self.prime()
    msg = self.packer.make_can_msg_safety("EPS_2", 0, {"EPS_TORQUE_MOTOR": 0})
    msg[0].data[7] ^= 1
    self.assertFalse(self.safety.safety_rx_hook(msg))
    self.assertFalse(self.steer())

  def test_bad_counter_blocks_steering(self):
    self.prime()
    for _ in range(6):
      self.rx("EPS_2", {"EPS_TORQUE_MOTOR": 0, "COUNTER": 0})
    self.assertFalse(self.steer())

  def test_existing_torque_and_rate_limits(self):
    self.prime()
    self.assertTrue(self.steer(3))
    self.assertTrue(self.steer(-3))
    self.assertFalse(self.steer(4))
    self.assertFalse(self.steer(-4))
    self.assertTrue(self.steer(261, 261))
    self.assertFalse(self.steer(262, 261))
    self.assertFalse(self.steer(-262, -261))

  def test_heartbeat_preserves_other_functions(self):
    self.prime()
    raw = bytes.fromhex("abcdef9234")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x2D9, 2, raw))
    self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x2D9, 0, raw)))
    for byte, bit in ((0, 1), (1, 128), (2, 4), (3, 128), (4, 16)):
      changed = bytearray(raw)
      changed[byte] ^= bit
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x2D9, 0, changed)))
    self.button(True)
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x2D9, 0, raw)))
    disabled = bytearray(raw)
    disabled[3] |= 1
    self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x2D9, 0, disabled)))

  def test_camera_forwarding_only_replaced_in_alka_mode(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x2D9))
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.chrysler, 0)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x2D9))
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x2D9, 0, bytes(5))))

  def test_relay_fault_blocks_output(self):
    self.prime()
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self.steer())
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x2D9))

  def test_reinitialization_and_ram_do_not_inherit_permission(self):
    self.prime()
    for param in (4, 1, 2, 5, 6, 0):
      self.safety.set_safety_hooks(structs.CarParams.SafetyModel.chrysler, param)
      self.assertFalse(self.safety.get_lkas_on())
      self.assertEqual(param == 4, self.safety.get_alka_allowed())
      self.assertFalse(self.steer())


if __name__ == "__main__":
  unittest.main()
