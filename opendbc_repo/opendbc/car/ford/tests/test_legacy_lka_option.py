"""Startup selection, exclusive interfaces and legacy LKA host bounds."""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.fordcan import CanBus, create_legacy_lka_msg, create_acc_ui_msg
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.legacy_lka import LegacyLKALateral, LKACommand, REFERENCE_SCALE
from opendbc.car.ford.tests.test_factory_acc import car_state
from opendbc.car.ford.values import CAR, DBC, FordFlags
from tools.ford_legacy_lka.protocol import decode_lka


def params(candidate=CAR.FORD_EDGE_MK2, legacy=False, bluepilot=True, factory=True, alpha=False, reference_port=False):
  flags = (structs.DPFlags.FordLegacyLKA if legacy else 0)
  flags |= structs.DPFlags.FordBluePilot if bluepilot else 0
  flags |= structs.DPFlags.FordStockLon if factory else 0
  cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, False, flags, True)
  # Retain explicit fixtures for the archived parameter-24 controller. The real
  # startup path selects the reference port; new tests exercise that path too.
  if legacy and not reference_port and not cp.dashcamOnly:
    cp.flags &= ~int(FordFlags.FUSION_LKA_PORT)
    cp.safetyConfigs[-1].safetyParam = 24
  return cp


def state(cp, now=1_000_000_000):
  cs = car_state(cp)
  cs.out.canValid = True
  cs.legacy_lka_panda_valid = True
  cs.legacy_lka_feedback = dict(availability=3, denied=False, time_nanos=now, wheel_time_nanos=now, wheel_angle_deg=0.0)
  return cs


class TestLegacyLKAOption(unittest.TestCase):
  def test_mode_pair_and_factory_ownership(self):
    for bluepilot in (False, True):
      for factory in (False, True):
        cp = params(legacy=True, bluepilot=bluepilot, factory=factory, alpha=True, reference_port=True)
        self.assertFalse(cp.dashcamOnly)
        self.assertFalse(cp.flags & FordFlags.BLUEPILOT)
        self.assertTrue(cp.flags & FordFlags.LEGACY_LKA)
        self.assertTrue(cp.flags & FordFlags.STOCK_ACC)
        self.assertFalse(cp.openpilotLongitudinalControl)
        self.assertFalse(cp.alphaLongitudinalAvailable)
        self.assertFalse(cp.autoResumeSng)
        self.assertFalse(cp.steerAtStandstill)
        self.assertEqual(15, cp.minSteerSpeed)
        self.assertTrue(cp.flags & FordFlags.FUSION_LKA_PORT)
        self.assertEqual([(structs.CarParams.SafetyModel.ford, 40)],
                         [(s.safetyModel, s.safetyParam) for s in cp.safetyConfigs])

  def test_legacy_lka_preserves_all_stock_tja_codes_and_the_complete_acc_ui_packet(self):
    cp = params(legacy=True, reference_port=True)
    cs = state(cp)
    packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
    hud = structs.CarControl().hudControl
    stock = cs.acc_tja_status_stock_values
    stock.update(AccTGap_D_Dsply=4, AccFllwMde_B_Dsply=1, AccMsgTxt_D2_Rq=3,
                 AccStopStat_D_Dsply=2, AccStopMde_B_Dsply=1,
                 HaDsply_No_Cs=29, HaDsply_No_Cnt=7)
    for raw_tja in range(8):
      for enabled in (False, True):
        for main_on in (False, True):
          for stock_fcw in (0, 1):
            stock.update(Tja_D_Stat=raw_tja, FcwVisblWarn_B_Rq=stock_fcw)
            expected = packer.make_can_msg("ACCDATA_3", 0, stock)
            actual = create_acc_ui_msg(packer, CanBus(cp), cp, main_on, enabled,
                                      False, False, False, hud, stock)
            self.assertEqual(expected, actual, (raw_tja, enabled, main_on, stock_fcw))

    # The existing non-LKA display behavior is deliberately still exercised.
    cp.flags &= ~int(FordFlags.LEGACY_LKA)
    for raw_tja in range(8):
      stock["Tja_D_Stat"] = raw_tja
      expected = packer.make_can_msg("ACCDATA_3", 0, dict(stock, Tja_D_Stat=2))
      actual = create_acc_ui_msg(packer, CanBus(cp), cp, True, True,
                                False, False, False, hud, stock)
      self.assertEqual(expected, actual, raw_tja)

  def test_unsupported_profiles_remain_passive(self):
    for candidate in CAR:
      if candidate == CAR.FORD_EDGE_MK2:
        continue
      cp = params(candidate, legacy=True)
      self.assertTrue(cp.dashcamOnly)
      self.assertEqual(structs.CarParams.SafetyModel.noOutput, cp.safetyConfigs[-1].safetyModel)
      controller = CarController(DBC[cp.carFingerprint], cp)
      cc = structs.CarControl(enabled=True, latActive=True, longActive=True)
      _, messages = controller.update(cc.as_reader(), state(cp), 1_000_000_000)
      self.assertEqual([], messages)

  def test_disabling_option_restores_original_params_exactly(self):
    for candidate in CAR:
      for bluepilot in (False, True):
        flags = structs.DPFlags.FordStockLon | (structs.DPFlags.FordBluePilot if bluepilot else 0)
        baseline = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, flags, True)
        params(candidate, legacy=True, bluepilot=bluepilot)
        self.assertEqual(baseline.to_bytes(), params(candidate, bluepilot=bluepilot).to_bytes())

  def test_ford_selection_does_not_change_pacifica(self):
    from opendbc.car.chrysler.interface import CarInterface as CI
    from opendbc.car.chrysler.values import CAR as CARS
    args = (CARS.CHRYSLER_PACIFICA_2018, gen_empty_fingerprint(), [], False, False)
    baseline = CI.get_params(*args, 0, True)
    self.assertEqual(baseline.to_bytes(), CI.get_params(*args, structs.DPFlags.FordLegacyLKA, True).to_bytes())

  def test_no_active_lca_no_host_acceleration_or_resume(self):
    cp = params(legacy=True)
    controller = CarController(DBC[cp.carFingerprint], cp)
    cc = structs.CarControl(enabled=True, latActive=True, longActive=True)
    cc.cruiseControl.resume = True
    cs = state(cp)
    lka = []
    for frame in range(900):
      now = 1_000_000_000 + frame * 10_000_000
      cs.legacy_lka_feedback['time_nanos'] = now
      cs.legacy_lka_feedback['wheel_time_nanos'] = now
      cc.actuators.curvature = 0.0006 * math.sin(frame / 150)
      output, messages = controller.update(cc.as_reader(), cs, now)
      for address, data, bus in messages:
        self.assertNotIn(address, (0x186, 0x3D6))
        if address == 0x3D3:
          self.assertEqual(0, (data[4] >> 2) & 7)
        if address == 0x3CA:
          lka.append(decode_lka(data))
      self.assertEqual(0, output.accel)
      self.assertEqual(0, output.gas)
    self.assertEqual(300, len(lka))
    self.assertTrue(any(x['ford_base_view']['request'] for x in lka))
    self.assertIsNone(controller.bp_lateral)

  def test_raw_encoder_matches_recorded_packet_and_canonical_inactive(self):
    # Literal values extracted from the owner's OEM capture, not the encoder.
    can = CanBus(params(legacy=True))
    self.assertEqual('8381a87d00000000', create_legacy_lka_msg(can, LKACommand(4, 125, 26))[1].hex())
    # The captured 231-count reference is intentionally outside initial authority.
    with self.assertRaises(ValueError):
      create_legacy_lka_msg(can, LKACommand(4, 231, 0))
    self.assertEqual('4380080000000000', create_legacy_lka_msg(can, LKACommand(2))[1].hex())
    self.assertEqual('8380080000000000', create_legacy_lka_msg(can, LKACommand(4))[1].hex())
    self.assertEqual('0380080000000000', create_legacy_lka_msg(can, LKACommand())[1].hex())
    for command in (LKACommand(1), LKACommand(2, 129), LKACommand(4, 0, 301), LKACommand(0, 1)):
      with self.assertRaises(ValueError):
        create_legacy_lka_msg(can, command)

  def test_interruption_is_latched_until_actual_disengagement(self):
    cp = params(legacy=True); controller = LegacyLKALateral(cp)
    cc = structs.CarControl(enabled=True, latActive=True)
    cs = state(cp)
    self.assertNotEqual(0, controller.update(cc, cs, 1_000_000_000).action)
    cs.legacy_lka_feedback.update(availability=1, time_nanos=1_030_000_000)
    self.assertEqual(0, controller.update(cc, cs, 1_030_000_000).action)
    self.assertTrue(controller.latched)
    cc.latActive = False
    controller.update(cc, cs, 1_060_000_000)
    self.assertTrue(controller.latched)
    cs.legacy_lka_feedback.update(availability=3, time_nanos=1_090_000_000)
    cc.latActive = True
    self.assertEqual(0, controller.update(cc, cs, 1_090_000_000).action)
    cc.enabled = cc.latActive = False
    controller.update(cc, cs, 1_120_000_000)
    self.assertFalse(controller.latched)
    cc.enabled = cc.latActive = True
    cs.legacy_lka_feedback['time_nanos'] = 1_150_000_000
    cs.legacy_lka_feedback['wheel_time_nanos'] = 1_150_000_000
    self.assertNotEqual(0, controller.update(cc, cs, 1_150_000_000).action)

  def test_first_packet_is_rate_limited_from_measured_angle(self):
    cp = params(legacy=True); cs = state(cp); cs.out.steeringAngleDeg = 3.0; cs.legacy_lka_feedback['wheel_angle_deg'] = 3.0
    cc = structs.CarControl(enabled=True, latActive=True)
    c = LegacyLKALateral(cp).update(cc, cs, 1_000_000_000)
    self.assertLessEqual(abs(c.reference_raw - round(3.0 / REFERENCE_SCALE)), 2)

  def test_real_sccm_parser_supplies_independent_angle_and_timestamp(self):
    from opendbc.can import CANPacker
    from opendbc.car import Bus
    from opendbc.car.ford.carstate import CarState
    cp = params(legacy=True)
    cs = CarState(cp)
    parsers = cs.get_can_parsers(cp)
    wheel = CANPacker('ford_fusion_2018_pt').make_can_msg(
      'Steering_Wheel_Data_CG1', 0, {'SteWhlRelInit_An_Sns': 3.4})
    status = CANPacker('ford_lincoln_base_pt').make_can_msg(
      'Lane_Assist_Data3_FD1', 0, {'LaActAvail_D_Actl': 3})
    parsers['legacy_wheel'].update([(1_000_000_000, [wheel])])
    parsers[Bus.pt].update([(1_000_000_000, [status])])
    cs.update(parsers)
    self.assertAlmostEqual(3.4, cs.legacy_lka_feedback['wheel_angle_deg'])
    self.assertEqual(1_000_000_000, cs.legacy_lka_feedback['wheel_time_nanos'])
    cs.legacy_lka_latched = True
    self.assertFalse(cs.update(parsers).steerFaultTemporary)
    cs.legacy_lka_latched = False
    self.assertFalse(cs.update(parsers).steerFaultTemporary)

  def test_controller_exposes_latch_for_driver_warning_until_disengaged(self):
    cp = params(legacy=True); controller = CarController(DBC[cp.carFingerprint], cp)
    cs = state(cp); cc = structs.CarControl(enabled=True, latActive=True)
    controller.update(cc.as_reader(), cs, 1_000_000_000)
    self.assertFalse(cs.legacy_lka_latched)
    controller.frame = 3
    cs.legacy_lka_feedback.update(availability=1, time_nanos=1_030_000_000, wheel_time_nanos=1_030_000_000)
    controller.update(cc.as_reader(), cs, 1_030_000_000)
    self.assertTrue(cs.legacy_lka_latched)
    controller.frame = 6; cc.enabled = cc.latActive = False
    controller.update(cc.as_reader(), cs, 1_060_000_000)
    self.assertFalse(cs.legacy_lka_latched)

  def test_invalid_stale_override_speed_and_fault_inputs(self):
    cp = params(legacy=True)
    for name, value in [('canValid', False), ('vehicleSensorsInvalid', True), ('steeringPressed', True),
                        ('steeringTorque', 1.1), ('brakePressed', True), ('gasPressed', True),
                        ('vEgoRaw', 14.9), ('vEgoRaw', 30.1), ('steeringAngleDeg', 6.0),
                        ('steerFaultTemporary', True), ('steerFaultPermanent', True),
                        ('vEgoRaw', float('nan')), ('yawRate', float('inf'))]:
      cs = state(cp); setattr(cs.out, name, value)
      if name == 'steeringAngleDeg':cs.legacy_lka_feedback['wheel_angle_deg'] = value
      cc = structs.CarControl(enabled=True, latActive=True)
      self.assertEqual(0, LegacyLKALateral(cp).update(cc, cs, 1_000_000_000).action, name)
    for feedback in ({}, dict(availability=1, denied=False, time_nanos=1_000_000_000),
                     dict(availability=3, denied=True, time_nanos=1_000_000_000),
                     dict(availability=3, denied=False, time_nanos=800_000_000)):
      cs = state(cp); cs.legacy_lka_feedback = feedback
      cc = structs.CarControl(enabled=True, latActive=True)
      self.assertEqual(0, LegacyLKALateral(cp).update(cc, cs, 1_000_000_000).action)

  def test_no_periodic_off_pulses_for_twenty_seconds(self):
    cp = params(legacy=True); controller = LegacyLKALateral(cp); cs = state(cp)
    cc = structs.CarControl(enabled=True, latActive=True)
    for tick in range(700):
      now = 1_000_000_000 + tick * 30_000_000
      cs.legacy_lka_feedback['time_nanos'] = now
      cs.legacy_lka_feedback['wheel_time_nanos'] = now
      self.assertNotEqual(0, controller.update(cc, cs, now).action)


if __name__ == '__main__':
  unittest.main()
