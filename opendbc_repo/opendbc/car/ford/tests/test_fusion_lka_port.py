"""Pinned upstream equivalence, modern integration, and bounded host behavior."""
import hashlib
import ast
import json
import math
from pathlib import Path
import random
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
from cereal import car, log
from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs, Bus
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.fusion_lka_pid import ReferencePIController
from opendbc.car.ford.fusion_lka_port import (FusionLegacyLKAPort, FusionLKACommand, create_command,
                                           reference_values, raw_reference, raw_curvature)
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.tests.test_legacy_lka_option import params, state
from opendbc.car.ford.values import CAR, DBC, FordFlags
from openpilot.selfdrive.controls.lib.latcontrol_ford_fusion_lka import LatControlFordFusionLKA

ROOT = Path(__file__).resolve().parents[5]
REF = ROOT / 'tools/ford_legacy_lka/reference/jdgarlock'


def reference_module(name, aliases):
  namespace = {}
  with patch.dict('sys.modules', aliases):
    exec(compile((REF / name).read_text(), str(REF / name), 'exec'), namespace)
  return NS(**namespace)


def upstream_pid():
  return reference_module('pid.py', {'common.numpy_fast': NS(clip=lambda x, lo, hi: min(max(x, lo), hi), interp=np.interp)}).PIController


def port_params():
  return params(legacy=True, reference_port=True)


class TestFusionReferenceEquivalence(unittest.TestCase):
  def test_pinned_sources_and_runtime_dbc_have_exact_original_hashes(self):
    manifest = json.loads((REF / 'provenance.json').read_text())
    for name, expected in manifest['files'].items():
      self.assertEqual(expected['sha256'], hashlib.sha256((REF / name).read_bytes()).hexdigest(), name)
    runtime = ROOT / 'opendbc_repo/opendbc/dbc/ford_fusion_legacy_lka.dbc'
    self.assertEqual((REF / 'ford_lincoln_base_pt.dbc').read_bytes(), runtime.read_bytes())

  def test_pi_output_and_internal_state_match_upstream_for_10000_updates(self):
    kwargs = dict(k_p=([0.], [.01]), k_i=([0.], [.005]), k_f=1/87., pos_limit=1., neg_limit=-1., sat_limit=.8)
    original, port = upstream_pid()(**kwargs), ReferencePIController(**kwargs)
    rng = random.Random(31019)
    for i in range(10000):
      if i % 937 == 0:
        original.reset(); port.reset()
      args = dict(setpoint=rng.uniform(-60, 60), measurement=rng.uniform(-60, 60),
                  speed=rng.uniform(0, 35), check_saturation=i % 3 != 0,
                  override=i % 17 == 0, feedforward=rng.uniform(-60, 60),
                  freeze_integrator=i % 23 == 0)
      self.assertAlmostEqual(original.update(**args), port.update(**args), places=12)
      for attr in ('p', 'i', 'f', 'sat_count', 'control', 'saturated'):
        self.assertEqual(getattr(original, attr), getattr(port, attr), (i, attr))

  def test_modern_angle_adapter_matches_upstream_latcontrol_for_4000_updates(self):
    old_pid = upstream_pid()
    old_log = NS(ControlsState=NS(LateralPIDState=NS(new_message=NS)))
    original_class = reference_module('latcontrol_pid.py', {
      'selfdrive.controls.lib.pid': NS(PIController=old_pid),
      'selfdrive.controls.lib.drive_helpers': NS(get_steer_max=lambda cp, speed: 6.4/87.),
      'cereal': NS(car=car, log=old_log),
    }).LatControlPID
    cp = port_params(); old, new = original_class(cp), LatControlFordFusionLKA(cp, None, .01)
    rng = random.Random(67)
    for i in range(4000):
      measured, desired = rng.uniform(-5, 5), rng.uniform(-5, 5)
      speed = rng.uniform(0, 30)
      pressed, limited, active = i % 37 == 0, i % 31 == 0, i % 103 != 0
      cs = NS(vEgo=speed, steeringAngle=measured, steeringRate=0., steeringAngleDeg=measured,
              steeringRateDeg=0., steeringPressed=pressed, steeringRateLimited=limited)
      a = old.update(active, cs, cp, NS(angleSteers=desired, angleOffset=0.))
      b = new.update_angle(active, cs, desired, limited)
      self.assertAlmostEqual(a[0], b[0], places=12)
      self.assertEqual(a[1], b[1])
      self.assertEqual(a[2].active, b[2].active)
      if active and speed >= .3:
        self.assertEqual(a[2].saturated, b[2].saturated)

  def test_active_packet_bytes_match_pinned_encoder_for_5000_inputs(self):
    old = reference_module('fordcan.py', {
      'common.numpy_fast': NS(clip=lambda x, lo, hi: min(max(x, lo), hi)),
      'selfdrive.car.ford.values': NS(MAX_ANGLE=87.),
    })
    reference_packer = CANPacker(str(REF / 'ford_lincoln_base_pt.dbc'))
    runtime_packer = CANPacker('ford_fusion_legacy_lka')
    # A model with both the old two-argument and modern optional-roll signature.
    vm = NS(calc_curvature=lambda angle, speed, roll=0.: angle / 45.)
    rng = random.Random(2019)
    for i in range(5000):
      pid, angle, speed = rng.uniform(-1, 1), rng.uniform(-90, 90), rng.uniform(0, 35)
      curvature = vm.calc_curvature(math.radians(angle), speed)
      expected = old.create_steer_command(reference_packer, pid, True, 3, 0., curvature, 2)
      values = reference_values(pid, angle, speed, vm)
      actual = runtime_packer.make_can_msg('Lane_Keep_Assist_Control', 0, values)
      self.assertEqual(expected, actual, i)

  def test_live_packets_clear_warning_fields_and_preserve_donor_steering_payload(self):
    packer = CANPacker('ford_fusion_legacy_lka')
    for reference in range(-128, 129):
      for curvature in (-300, -17, 0, 17, 300):
        command = FusionLKACommand(2, reference, curvature)
        donor = packer.make_can_msg('Lane_Keep_Assist_Control', 0,
                                   {'Lkas_Action': 2, 'Lkas_Alert': 14,
                                    'Lane_Curvature': curvature * .000005, 'Steer_Angle_Req': reference * .05})
        actual = create_command(packer, 0, command)
        self.assertEqual(0x40, actual[1][0])
        self.assertEqual(0, (actual[1][0] >> 2) & 7)  # LDW_Idle
        self.assertEqual(0, actual[1][0] & 3)  # intensity None
        self.assertEqual(donor[1][1:], actual[1][1:])
        self.assertEqual((donor[0], donor[2]), (actual[0], actual[2]))
    self.assertEqual('0380080000000000', create_command(packer, 0, FusionLKACommand())[1].hex())
    # The upstream 0xf does not fit its own 3-bit action field: it becomes 7.
    original_inactive = packer.make_can_msg('Lane_Keep_Assist_Control', 0, {'Lkas_Action': 15, 'Lkas_Alert': 14})
    self.assertEqual(7, original_inactive[1][0] >> 5)


class TestFusionPortHost(unittest.TestCase):
  def setUp(self):
    self.cp = port_params()
    self.cs = state(self.cp)
    self.cs.legacy_lka_tx_blocked = 0
    self.cc = structs.CarControl(enabled=True, latActive=True)
    self.port = FusionLegacyLKAPort(self.cp)
    self.now = 1_000_000_000

  def step(self):
    self.cs.legacy_lka_feedback.update(time_nanos=self.now, wheel_time_nanos=self.now)
    command = self.port.update(self.cc, self.cs, self.now)
    self.now += 10_000_000
    return command

  def test_startup_selects_port_pid_geometry_and_distinct_safety_pair(self):
    self.assertTrue(self.cp.flags & FordFlags.FUSION_LKA_PORT)
    self.assertEqual(40, self.cp.safetyConfigs[-1].safetyParam)
    self.assertAlmostEqual(2.85, self.cp.wheelbase, places=5)
    self.assertAlmostEqual(14.8, self.cp.steerRatio, places=5)
    self.assertAlmostEqual(.01, self.cp.lateralTuning.pid.kpV[0], places=6)
    self.assertAlmostEqual(1/87., self.cp.lateralTuning.pid.kf, places=7)

  def test_controlsd_publishes_port_pid_telemetry_with_the_correct_wire_schema(self):
    source = ast.parse((ROOT / 'selfdrive/controls/controlsd.py').read_text())
    controls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Controls')
    publish = next(n for n in controls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish')
    start = next(i for i, n in enumerate(publish.body) if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'lat_tuning' for t in n.targets))
    production = ast.Module(body=publish.body[start:start+2], type_ignores=[])
    for cp, schema, expected in [
      (self.cp, log.ControlsState.LateralPIDState, 'pidState'),
      (params(legacy=False), log.ControlsState.LateralAngleState, 'angleState'),
    ]:
      message = log.Event.new_message();message.init('controlsState')
      namespace = {'self': NS(CP=cp), 'cs': message.controlsState,
                   'lac_log': schema.new_message(active=True), 'car': car, 'FordFlags': FordFlags}
      exec(compile(ast.fix_missing_locations(production), '<production controlsd telemetry>', 'exec'), namespace)
      restored = log.Event.from_bytes(message.to_bytes())
      with restored as event:
        self.assertEqual(expected, event.controlsState.lateralControlState.which())
        self.assertTrue(getattr(event.controlsState.lateralControlState, expected).active)

  def test_lka_stream_is_100_hz_without_active_lca_or_acceleration(self):
    controller = CarController(DBC[self.cp.carFingerprint], self.cp)
    self.cc.actuators.torque = .005
    counts = {}
    for i in range(1000):
      now = 1_000_000_000 + i * 10_000_000
      self.cs.legacy_lka_feedback.update(time_nanos=now, wheel_time_nanos=now)
      output, messages = controller.update(self.cc.as_reader(), self.cs, now)
      for address, data, bus in messages:
        counts[address] = counts.get(address, 0) + 1
        self.assertNotIn(address, (0x186, 0x3D6))
        if address == 0x3D3:self.assertEqual(0, (data[4] >> 2) & 7)
        if address == 0x3CA:self.assertEqual(0x40, data[0])
      self.assertEqual(0, output.accel)
    self.assertEqual(1000, counts[0x3CA])

  def test_native_rejection_triggers_neutral_until_manual_disengagement(self):
    self.assertEqual(2, self.step().action)
    self.cs.legacy_lka_tx_blocked = 1
    self.assertEqual(0, self.step().action)
    self.assertEqual('panda_rejected', self.port.latch_reason)
    for _ in range(150):self.assertEqual(0, self.step().action)
    self.cc.enabled = False
    self.step()
    self.cc.enabled = True
    self.assertEqual(2, self.step().action)

  def test_driver_override_and_availability_do_not_automatically_rearm(self):
    self.step()
    self.cs.out.steeringTorque = 1.1
    self.assertEqual(0, self.step().action)
    self.cs.out.steeringTorque = 0.
    self.cs.legacy_lka_feedback['availability'] = 3
    self.assertEqual(0, self.step().action)
    self.assertTrue(self.port.latched)

  def test_pid_reference_scale_is_the_pinned_dbc_scale(self):
    self.cc.actuators.torque = 1. / 87.
    for _ in range(20):command = self.step()
    self.assertEqual(20, command.reference_raw)
    self.assertEqual('JDGarlock/5b5f7516', self.port.diagnostics['reference_port'])

  def test_sccm_source_is_used_for_published_steering_and_controller_feedback(self):
    cs = CarState(self.cp);parsers = cs.get_can_parsers(self.cp)
    wheel = CANPacker('ford_fusion_legacy_lka').make_can_msg('Steering_Wheel_Data_CG1', 0, {'SteWhlRelInit_An_Sns': -1.9})
    factory = CANPacker('ford_lincoln_base_pt')
    camera = factory.make_can_msg('ParkAid_Data', 0, {'ExtSteeringAngleReq2': -3.1})
    status = factory.make_can_msg('Lane_Assist_Data3_FD1', 0, {'LaActAvail_D_Actl': 3})
    parsers['legacy_wheel'].update([(1_000_000_000, [wheel])])
    parsers[Bus.pt].update([(1_000_000_000, [camera, status])])
    out = cs.update(parsers)
    self.assertAlmostEqual(-1.9, out.steeringAngleDeg)
    self.assertAlmostEqual(out.steeringAngleDeg, cs.legacy_lka_feedback['wheel_angle_deg'])
    self.assertFalse(out.steerFaultTemporary)

  def test_all_invalid_inputs_keep_neutral_heartbeats(self):
    cases = [('canValid', False), ('vehicleSensorsInvalid', True), ('gasPressed', True), ('brakePressed', True),
             ('steerFaultTemporary', True), ('steerFaultPermanent', True), ('steeringPressed', True),
             ('steeringTorque', .9375), ('vEgoRaw', 14.9), ('vEgoRaw', 30.1), ('yawRate', math.nan)]
    for field, value in cases:
      self.setUp();setattr(self.cs.out, field, value)
      command = self.step()
      self.assertEqual(0, command.action, field)
      self.assertEqual('0380080000000000', create_command(self.port.packer, 0, command)[1].hex())

  def test_missing_or_stale_panda_pair_does_not_start_active(self):
    self.cs.legacy_lka_panda_valid = False
    self.assertEqual(0, self.step().action)
    self.assertEqual('panda_pair_unavailable', self.port.diagnostics['reason'])

  def test_learned_angle_offset_is_not_reused_across_sensor_reference_change(self):
    source = ast.parse((ROOT / 'selfdrive/locationd/paramsd.py').read_text())
    function = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == 'retrieve_initial_vehicle_params')
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), function], type_ignores=[])
    namespace = {'car': car, 'log': log, 'np': np, 'FordFlags': FordFlags,
                 'cloudlog': NS(error=lambda *a: None, info=lambda *a: None)}
    exec(compile(ast.fix_missing_locations(module), '<production parameter cache>', 'exec'), namespace)
    for old_port, new_port in [(False, True), (True, False), (True, True), (False, False)]:
      old_cp = params(legacy=True, reference_port=old_port)
      cp = params(legacy=True, reference_port=new_port)
      message = log.Event.new_message();message.init('liveParameters')
      message.liveParameters.steerRatio = 14.8
      message.liveParameters.stiffnessFactor = 1.
      message.liveParameters.angleOffsetAverageDeg = 1.3
      data = {'LiveParametersV2': message.to_bytes(), 'CarParamsPrevRoute': old_cp.to_bytes()}
      removed = []
      params_store = NS(get=lambda key: data.get(key), remove=lambda key: removed.append(key))
      result = namespace['retrieve_initial_vehicle_params'](params_store, cp, False, False)
      self.assertAlmostEqual(0. if old_port != new_port else 1.3, result[2], places=5)
      self.assertEqual(old_port != new_port, 'LiveParametersV2' in removed)


if __name__ == '__main__':unittest.main()
