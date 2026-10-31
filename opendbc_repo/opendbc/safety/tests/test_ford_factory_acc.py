"""Factory ACC forwards OEM control and never accepts host gas/brake/resume."""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.safety.tests import test_ford as stock
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_ford_bluepilot import BluePilotSafetyMixin


class TestFordFactoryACCSafety(stock.TestFordSafetyBase):
  PARAM = FordSafetyFlags.STOCK_ACC
  STEER_MESSAGE = stock.MSG_LateralMotionControl
  TX_MSGS = [
    [stock.MSG_Steering_Data_FD1, 0], [stock.MSG_Steering_Data_FD1, 2],
    [stock.MSG_ACCDATA_3, 0], [stock.MSG_Lane_Assist_Data1, 0],
    [stock.MSG_LateralMotionControl, 0], [stock.MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (stock.MSG_ACCDATA_3, stock.MSG_Lane_Assist_Data1,
                                stock.MSG_LateralMotionControl, stock.MSG_IPMA_Data)}
  FWD_BLACKLISTED_ADDRS = {2: list(RELAY_MALFUNCTION_ADDRS[0])}
  # Same as the existing classic-CAN test: the lateral-acceleration limiter is CAN FD only.
  test_max_lateral_acceleration = stock.TestFordLongitudinalSafety.test_max_lateral_acceleration

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, self.PARAM)
    self.safety.init_tests()

  def test_acc_buttons(self):
    # Normal cancel and TJA behavior is retained, but host resume is forbidden.
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      for bus in (0, 2):
        self.assertFalse(self._tx(self._acc_button_msg(stock.Buttons.RESUME, bus)))
      self.assertTrue(self._tx(self._acc_button_msg(stock.Buttons.TJA_TOGGLE, 2)))
    for engaged in (False, True):
      self._rx(self._pcm_status_msg(engaged))
      for bus in (0, 2):
        self.assertEqual(engaged, self._tx(self._acc_button_msg(stock.Buttons.CANCEL, bus)))

  def test_no_host_accel_frames_even_with_controls_or_alka(self):
    for alternative in (0, 1024):
      self.safety.set_alternative_experience(alternative)
      for allowed in (False, True):
        self.safety.set_controls_allowed(allowed)
        for bus in (0, 1, 2):
          for payload in (b"\0" * 8, b"\xff" * 8, bytes(range(8))):
            self.assertFalse(self._tx(libsafety_py.make_CANPacket(stock.MSG_ACCDATA, bus, payload)))

  def test_oem_acc_and_aeb_forward_without_relay_fault(self):
    # Forwarding only chooses the destination; it does not alter payload bytes.
    for addr in (stock.MSG_ACCDATA, 0x187):
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr))
      self.assertEqual(2, self.safety.safety_fwd_hook(0, addr))
      self.assertEqual(-1, self.safety.safety_fwd_hook(1, addr))
      self.assertTrue(self._rx(libsafety_py.make_CANPacket(addr, 0, b"\0" * 8)))
      self.assertFalse(self.safety.get_relay_malfunction())

  def test_reinitialization_restores_default_acc_blocking(self):
    self.assertEqual(0, self.safety.safety_fwd_hook(2, stock.MSG_ACCDATA))
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, 0)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, stock.MSG_ACCDATA))
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, self.PARAM)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, stock.MSG_ACCDATA))


class TestFordFactoryACCBluePilotSafety(BluePilotSafetyMixin, TestFordFactoryACCSafety):
  pass


class TestFordFactoryACCConflictingFlagsSafety(TestFordFactoryACCSafety):
  PARAM = FordSafetyFlags.STOCK_ACC | FordSafetyFlags.LONG_CONTROL


class TestFordFactoryACCCANFDSafety(TestFordFactoryACCSafety):
  PARAM = FordSafetyFlags.STOCK_ACC | FordSafetyFlags.CANFD
  STEER_MESSAGE = stock.MSG_LateralMotionControl2
  TX_MSGS = stock.TestFordCANFDStockSafety.TX_MSGS
  RELAY_MALFUNCTION_ADDRS = stock.TestFordCANFDStockSafety.RELAY_MALFUNCTION_ADDRS
  FWD_BLACKLISTED_ADDRS = stock.TestFordCANFDStockSafety.FWD_BLACKLISTED_ADDRS
  test_max_lateral_acceleration = stock.TestFordSafetyBase.test_max_lateral_acceleration

  def test_reinitialization_restores_default_acc_blocking(self):
    # CAN FD already has OEM ACC without LONG_CONTROL; test transition to OP long.
    self.assertEqual(0, self.safety.safety_fwd_hook(2, stock.MSG_ACCDATA))
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.CANFD | FordSafetyFlags.LONG_CONTROL)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, stock.MSG_ACCDATA))
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, self.PARAM)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, stock.MSG_ACCDATA))


if __name__ == "__main__":
  unittest.main()
