"""Exercise production latch, parser, alert and engagement code without CAN I/O."""
import ast
from collections import defaultdict
from enum import IntEnum, IntFlag
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest

from selfdrive.car.ford_legacy_lka import lateral_allowed, stop_title
from tools.ford_legacy_lka import capture
from tools.ford_legacy_lka.protocol import Session

ROOT = Path(__file__).resolve().parents[2]


def tree(path):
  return ast.parse((ROOT / path).read_text())


class Flags(IntFlag):
  CANFD = 1
  ALT_STEER_ANGLE = 2
  LEGACY_LKA = 16
  FUSION_LKA_PORT = 32


def controller_class():
  source = tree('opendbc_repo/opendbc/car/ford/legacy_lka.py')
  source.body = [n for n in source.body if not isinstance(n, ast.ImportFrom) or n.module != 'opendbc.car.vehicle_model']
  module = ModuleType('_ford_lka_latch_test')
  sys.modules[module.__name__] = module
  module.VehicleModel = lambda cp: NS(get_steer_from_curvature=lambda curvature, speed, roll: curvature * 45.)
  exec(compile(source, '<production legacy_lka>', 'exec'), module.__dict__)
  return module.LegacyLKALateral


def parser_state(failure=0, available=3, denied=False):
  source = tree('opendbc_repo/opendbc/car/ford/carstate.py')
  cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'CarState')
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
  env = {'FordFlags': Flags, 'Bus': NS(pt='pt', cam='cam'),
         'TransmissionType': NS(automatic='auto', manual='manual'),
         'GearShifter': NS(reverse='reverse', drive='drive'),
         'ButtonType': NS(gapAdjustCruise='gap', lkas='lkas'),
         'CV': NS(KPH_TO_MS=1/3.6, MPH_TO_MS=0.44704),
         'CarControllerParams': NS(STEER_DRIVER_ALLOWANCE=1.),
         'create_button_events': lambda *args: [], 'structs': NS(CarState=lambda: NS(cruiseState=NS()))}
  exec(compile(ast.Module(body=[update], type_ignores=[]), '<production CarState.update>', 'exec'), env)
  values = defaultdict(lambda: defaultdict(float))
  values['EPAS_INFO']['EPAS_Failure'] = failure
  values['Lane_Assist_Data3_FD1'].update(LaActAvail_D_Actl=available, LaActDeny_B_Actl=denied)
  parser = NS(vl=values, ts_nanos=defaultdict(lambda: defaultdict(lambda: 1)))
  state = NS(CP=NS(flags=Flags.ALT_STEER_ANGLE | Flags.LEGACY_LKA, transmissionType='manual',
                   openpilotLongitudinalControl=False, enableBsm=False),
             steering_angle_offset_deg=0., update_speed_kf=lambda v: (v, 0.),
             update_steering_pressed=lambda pressed, count: pressed,
             distance_button=0, lc_button=False, legacy_lka_latched=True)
  return env['update'](state, {'pt': parser, 'cam': parser, 'legacy_wheel': parser})


def alert_environment():
  source = tree('selfdrive/selfdrived/events.py')
  names = {'Priority', 'ET', 'Alert', 'NoEntryAlert', 'ford_lka_latched_alert', 'ford_lka_latched_no_entry'}
  selected = [n for n in source.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
  env = {'IntEnum': IntEnum, 'DT_CTRL': .01, 'stop_title': stop_title,
         'AlertStatus': NS(normal=0, userPrompt=1), 'AlertSize': NS(mid=2),
         'VisualAlert': NS(none=0, steerRequired=1), 'AudibleAlert': NS(prompt=1, refuse=2),
         'HARDWARE': NS(get_device_type=lambda: 'tici')}
  module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)] + selected,
                      type_ignores=[])
  exec(compile(ast.fix_missing_locations(module), '<production alerts>', 'exec'), env)
  events = next(n.value for n in source.body if isinstance(n, ast.AnnAssign) and
                isinstance(n.target, ast.Name) and n.target.id == 'EVENTS')
  entry = next(v for k, v in zip(events.keys, events.values) if isinstance(k, ast.Attribute) and k.attr == 'fordLegacyLkaLatched')
  env['EmptyAlert'] = object()
  policy = eval(compile(ast.Expression(entry), '<production latch event>', 'eval'), env)
  return env, policy


class TestHostLatch(unittest.TestCase):
  def setUp(self):
    self.controller = controller_class()(NS())
    self.now = 1000000000
    self.out = NS(vEgoRaw=20., yawRate=0., steeringTorque=0., canValid=True, vehicleSensorsInvalid=False,
                  steerFaultTemporary=False, steerFaultPermanent=False, steeringPressed=False,
                  brakePressed=False, gasPressed=False, cruiseState=NS(enabled=True))
    self.cs = NS(out=self.out, legacy_lka_feedback={})
    self.cc = NS(enabled=True, latActive=True, actuators=NS(curvature=.0002))

  def step(self):
    self.now += 30000000
    self.cs.legacy_lka_feedback.update(availability=3, denied=False, wheel_angle_deg=0.,
                                       time_nanos=self.now, wheel_time_nanos=self.now)
    return self.controller.update(self.cc, self.cs, self.now)

  def test_accelerator_stop_preserves_first_reason_after_release(self):
    self.assertNotEqual(0, self.step().action)
    self.out.gasPressed = True
    self.assertEqual(0, self.step().action)
    self.assertEqual('accelerator_pressed', self.controller.latch_reason)
    self.out.gasPressed = False
    for _ in range(100):
      self.assertEqual(0, self.step().action)
    self.assertTrue(self.controller.latched)
    self.assertEqual('accelerator_pressed', self.controller.diagnostics['latch_reason'])

  def test_manual_disengagement_is_still_required_to_clear_latch(self):
    self.step()
    self.out.steeringTorque = 1.0625
    self.assertEqual(0, self.step().action)
    self.assertEqual('driver_override', self.controller.latch_reason)
    self.out.steeringTorque = 0.
    self.cc.latActive = False
    self.assertEqual(0, self.step().action)
    self.assertTrue(self.controller.latched)
    self.cc.enabled = False
    self.step()
    self.assertFalse(self.controller.latched)
    self.assertEqual('', self.controller.latch_reason)
    self.cc.enabled = self.cc.latActive = True
    self.assertNotEqual(0, self.step().action)

  def test_brake_and_cruise_stops_have_distinct_reasons(self):
    for brake, cruise, expected in [(True, True, 'brake_pressed'), (False, False, 'cruise_inactive')]:
      self.setUp()
      self.step()
      self.out.brakePressed, self.out.cruiseState.enabled = brake, cruise
      self.assertEqual(0, self.step().action)
      self.assertEqual(expected, self.controller.latch_reason)

  def test_host_latch_does_not_fabricate_eps_fault(self):
    state = parser_state()
    self.assertFalse(state.steerFaultTemporary)
    self.assertFalse(state.steerFaultPermanent)

  def test_real_eps_failure_unavailability_and_denial_are_preserved(self):
    for arguments in ({'failure': 1}, {'available': 1}, {'denied': True}):
      self.assertTrue(parser_state(**arguments).steerFaultTemporary)
    self.assertTrue(parser_state(failure=2).steerFaultPermanent)

  def test_controlsd_lateral_gate_is_scoped_and_fails_closed(self):
    source = tree('selfdrive/controls/controlsd.py')
    branch = next(n for n in ast.walk(source) if isinstance(n, ast.If) and
                  'FordFlags.LEGACY_LKA' in ast.unparse(n.test))
    for brand, flags, valid, latched, initial, expected in [
      ('ford', 16, True, False, True, True), ('ford', 16, True, True, True, False),
      ('ford', 16, False, False, True, False), ('ford', 16, False, True, True, False),
      ('ford', 16, True, False, False, False), ('ford', 0, False, True, True, True),
      ('toyota', 16, False, True, True, True),
    ]:
      class SM(dict):
        def all_checks(self, services):
          return valid
      cc = NS(latActive=initial)
      env = {'self': NS(CP=NS(brand=brand, flags=flags), sm=SM(carStateExt=NS(legacyLkaLatched=latched))),
             'CC': cc, 'FordFlags': Flags, 'lateral_allowed': lateral_allowed}
      exec(compile(ast.Module(body=[branch], type_ignores=[]), '<production lateral gate>', 'exec'), env)
      self.assertEqual(expected, cc.latActive)

  def test_host_warning_is_distinct_and_has_lower_priority_than_real_faults(self):
    env, policy = alert_environment()
    self.assertEqual({'warning', 'noEntry', 'overrideLateral'}, set(policy))
    sm = {'carStateExt': NS(legacyLkaLatchReason='accelerator_pressed')}
    alert = policy['warning'](None, None, sm, True, 0, None)
    self.assertEqual('LKA stopped: accelerator', alert.alert_text_1)
    self.assertIn('Steer manually', alert.alert_text_2)
    self.assertIn('cancel cruise', alert.alert_text_2)
    no_entry = policy['noEntry'](None, None, sm, True, 0, None)
    self.assertLess(no_entry.priority, env['Priority'].LOW)
    self.assertLess(alert.priority, env['Priority'].MID)

  def test_state_machine_keeps_host_latch_as_lateral_override(self):
    env, policy = alert_environment()
    state = NS(disabled=0, preEnabled=1, enabled=2, softDisabling=3, overriding=4)
    namespace = {'log': NS(SelfdriveState=NS(OpenpilotState=state)), 'DT_CTRL': .01, 'ET': env['ET']}
    source = tree('selfdrive/selfdrived/state.py')
    source.body = [n for n in source.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    source.body.insert(0, ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0))
    exec(compile(ast.fix_missing_locations(source), '<production state machine>', 'exec'), namespace)
    class Events:
      def __init__(self, types):
        self.types = types
      def contains(self, kind):
        return kind in self.types
    machine = namespace['StateMachine']()
    machine.state = state.enabled
    for _ in range(400):
      self.assertEqual((True, True), machine.update(Events(policy)))
      self.assertEqual(state.overriding, machine.state)
    machine.update(Events({'userDisable'}))
    self.assertEqual(state.disabled, machine.state)
    machine.update(Events(set()))
    self.assertEqual(state.disabled, machine.state)
    machine.update(Events({'enable'}))
    self.assertEqual(state.enabled, machine.state)

  def test_recorder_retains_host_state_changes_and_specific_trigger(self):
    session = Session('fusion_2019', 'after')
    for t, latched in [(0, False), (1, True), (2, True)]:
      payload = NS(legacyLkaLatched=latched, legacyLkaLatchReason='accelerator_pressed' if latched else '')
      event = NS(which=lambda: 'carStateExt', logMonoTime=t, valid=True, carStateExt=payload)
      capture.add_event(session, event)
    self.assertEqual([False, True], [s['host_lka_latched'] for s in session.states])
    session.add_state({'kind': 'legacyLkaCommand', 'mono_ns': 3,
                       'command': {'latched': True, 'reason': 'cruise_or_brake', 'latch_reason': 'accelerator_pressed'}})
    self.assertEqual('accelerator_pressed', session.report()['host_latch_observations'][0]['observed_trigger_reason'])


class TestHostLatchNativeSchema(unittest.TestCase):
  def test_wire_fields_and_new_event_round_trip(self):
    try:
      from cereal import log
    except ImportError:
      self.skipTest('requires the native cereal/pycapnp runtime')
    extension = log.Event.new_message()
    extension.init('carStateExt')
    extension.carStateExt.legacyLkaLatched = True
    extension.carStateExt.legacyLkaLatchReason = 'driver_override'
    with log.Event.from_bytes(extension.to_bytes()) as received:
      self.assertTrue(received.carStateExt.legacyLkaLatched)
      self.assertEqual('driver_override', received.carStateExt.legacyLkaLatchReason)
    events = log.Event.new_message()
    event = events.init('onroadEvents', 1)[0]
    event.name = 'fordLegacyLkaLatched'
    event.noEntry, event.overrideLateral = True, True
    with log.Event.from_bytes(events.to_bytes()) as received:
      self.assertEqual('fordLegacyLkaLatched', str(received.onroadEvents[0].name))
      self.assertTrue(received.onroadEvents[0].overrideLateral)


if __name__ == '__main__':
  unittest.main()
