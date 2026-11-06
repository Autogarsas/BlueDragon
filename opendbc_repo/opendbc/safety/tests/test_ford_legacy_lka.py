"""Independent native safety boundaries for the paired legacy LKA mode."""
import math
import unittest

from opendbc.car import structs
from opendbc.car.ford.legacy_lka import LegacyLKALateral, REFERENCE_SCALE
from opendbc.car.ford.fordcan import create_legacy_lka_msg, CanBus, create_lat_ctl_msg
from opendbc.car.ford.tests.test_legacy_lka_option import params, state
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.test_ford import checksum
from opendbc.safety.tests.libsafety import libsafety_py


def raw_lka(action=4, reference=0, curvature=0, bus=0, extra=0, lower=3):
  reference += 2048; curvature += 2048
  payload = bytes(((action << 5) | lower, curvature >> 4,
                   ((curvature & 15) << 4) | (reference >> 8), reference & 255, extra, 0, 0, 0))
  return libsafety_py.make_CANPacket(0x3CA, bus, payload)


class TestFordLegacyLKASafety(unittest.TestCase):
  TX_MSGS = [[0x83, 0], [0x83, 2], [0x18A, 0], [0x3CA, 0], [0x3D3, 0], [0x3D8, 0]]
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.ford, 24)
    self.safety.init_tests()
    self.packer = CANPackerSafety('ford_lincoln_base_pt')
    self.old = CANPackerSafety('ford_fusion_2018_pt')
    self.count = 0
    self.now = 1_000_000

  def rx(self, name, values, fix=None):
    msg = self.packer.make_can_msg_safety(name, 0, values, fix_checksum=fix)
    self.assertTrue(self.safety.safety_rx_hook(msg), name)

  def inputs(self, speed=20, wheel=0, camera=None, torque=0, availability=3,
             denied=False, fault=0, enabled=True, brake=False, gas=0, curvature=0, omit=()):
    self.safety.set_timer(self.now)
    packets = [
      ('BrakeSysFeatures', {'Veh_V_ActlBrk': speed*3.6, 'VehVActlBrk_D_Qf': 3,
                           'VehVActlBrk_No_Cnt': self.count%16}, checksum),
      ('EngVehicleSpThrottle2', {'Veh_V_ActlEng': speed*3.6, 'VehVActlEng_D_Qf': 3,
                                'VehVActlEng_No_Cnt': self.count%16}, checksum),
      ('Yaw_Data_FD1', {'VehYaw_W_Actl': curvature*speed, 'VehYawWActl_D_Qf': 3,
                      'VehRollYaw_No_Cnt': self.count%256}, checksum),
      ('EngBrakeData', {'BpedDrvAppl_D_Actl': 2 if brake else 1, 'CcStat_D_Actl': 5 if enabled else 0}, None),
      ('EngVehicleSpThrottle', {'ApedPos_Pc_ActlArb': gas}, None),
      ('DesiredTorqBrk', {'VehStop_D_Stat': 0}, None),
      ('EPAS_INFO', {'SteeringColumnTorque': torque, 'EPAS_Failure': fault}, None),
      ('ParkAid_Data', {'ExtSteeringAngleReq2': wheel if camera is None else camera}, None),
      ('Lane_Assist_Data3_FD1', {'LaActAvail_D_Actl': availability, 'LaActDeny_B_Actl': denied}, None),
    ]
    for name, values, fix in packets:
      if name not in omit:self.rx(name, values, fix)
    if 'wheel' not in omit:
      msg = self.old.make_can_msg_safety('Steering_Wheel_Data_CG1', 0, {'SteWhlRelInit_An_Sns': wheel})
      self.assertTrue(self.safety.safety_rx_hook(msg))
    self.count += 1

  def tx(self, *args, **kwargs):
    return self.safety.safety_tx_hook(raw_lka(*args, **kwargs))

  def test_active_requires_all_fresh_inputs_and_permission(self):
    self.assertFalse(self.tx())
    self.inputs(enabled=False)
    self.assertFalse(self.tx())
    self.inputs(enabled=True)
    self.assertTrue(self.tx())
    self.safety.set_timer(self.now + 160_000)
    self.assertFalse(self.tx())
    self.assertTrue(self.tx(action=0))

  def test_every_required_sensor_is_checked(self):
    for sensor in ['wheel', 'EPAS_INFO', 'ParkAid_Data', 'Lane_Assist_Data3_FD1',
                   'BrakeSysFeatures', 'EngVehicleSpThrottle2', 'Yaw_Data_FD1',
                   'EngBrakeData', 'EngVehicleSpThrottle', 'DesiredTorqBrk']:
      self.setUp();self.inputs(omit=(sensor,))
      self.safety.set_controls_allowed(True)
      self.assertFalse(self.tx(), sensor)

  def test_driver_takeover_brake_gas_fault_and_status(self):
    for kwargs in [dict(torque=1.0625),dict(torque=-1.0625),dict(brake=True),dict(gas=1),
                   dict(fault=1),dict(fault=2),dict(availability=0),dict(availability=1),dict(denied=True),
                   dict(speed=14.9),dict(speed=30.1),dict(wheel=5.6),dict(camera=1.0)]:
      self.setUp();self.inputs(**kwargs)
      self.assertFalse(self.tx(), kwargs)
      self.assertTrue(self.tx(action=0), kwargs)

  def test_wrong_bus_wrong_action_override_and_ldw_rejected(self):
    self.inputs()
    for bus in (1,2):self.assertFalse(self.tx(bus=bus))
    for action in (1,3,5,6,7):self.assertFalse(self.tx(action=action))
    for extra in (1,32,64,128,255):self.assertFalse(self.tx(extra=extra))
    for lower in (0,1,2,4,7,15,31):self.assertFalse(self.tx(lower=lower))
    self.assertTrue(self.tx())

  def test_only_canonical_neutral_when_inactive(self):
    self.assertTrue(self.tx(action=0))
    self.assertFalse(self.tx(action=0,reference=1))
    self.assertFalse(self.tx(action=0,curvature=-1))

  def test_rate_limits_and_rejected_packets_do_not_seed_history(self):
    self.inputs()
    self.assertTrue(self.tx())
    self.now += 30_000;self.inputs()
    self.assertFalse(self.tx(reference=3))
    self.assertFalse(self.tx(reference=4))
    self.assertTrue(self.tx(reference=2,curvature=2))
    self.now += 30_000;self.inputs()
    self.assertFalse(self.tx(reference=5,curvature=2))
    self.assertFalse(self.tx(reference=2,curvature=5))
    self.assertTrue(self.tx(reference=4,curvature=4))

  def test_tx_frequency_is_bounded(self):
    self.inputs();self.assertTrue(self.tx())
    self.safety.set_timer(self.now+19_000);self.assertFalse(self.tx())
    self.safety.set_timer(self.now+30_000);self.assertTrue(self.tx())
    self.safety.set_timer(self.now+140_000);self.assertFalse(self.tx())

  def test_reset_rebases_from_measurement_not_last_host_packet(self):
    self.inputs()
    for _ in range(100):
      self.assertTrue(self.tx(action=0))
      self.assertFalse(self.tx(reference=128))
      self.assertFalse(self.tx(reference=13))
      self.assertFalse(self.tx(curvature=9))
      self.assertTrue(self.tx(reference=2))
      self.now+=30_000;self.inputs()

  def test_absolute_reference_and_error_bounds(self):
    for sign in (-1,1):
      self.setUp();action=4 if sign>0 else 2
      self.inputs(wheel=sign*5.5)
      self.assertFalse(self.tx(action=action,reference=sign*129))
      self.assertTrue(self.tx(action=action,reference=sign*128))
      self.setUp();self.inputs()
      for reference in range(2,48,2):
        self.assertTrue(self.tx(action=action,reference=sign*reference))
        self.now+=30_000;self.inputs()
      self.assertFalse(self.tx(action=action,reference=sign*48))

  def test_curvature_error_and_absolute_bounds(self):
    self.inputs(curvature=0)
    for raw in range(2,202,2):
      self.assertTrue(self.tx(curvature=raw))
      self.now+=30_000;self.inputs()
    self.assertFalse(self.tx(curvature=202))
    self.setUp();self.inputs(curvature=0.0015)
    self.assertTrue(self.tx(curvature=300))
    self.now+=30_000;self.inputs(curvature=0.0015)
    self.assertFalse(self.tx(curvature=301))

  def test_alka_does_not_grant_permission(self):
    self.inputs(enabled=False)
    self.safety.set_alternative_experience(1024)
    self.safety.set_lkas_on(True)
    self.assertFalse(self.safety.get_alka_allowed())
    self.assertFalse(self.tx())

  def test_conflicting_flags_fail_closed(self):
    for param in (16,17,18,20,25,26,28,31):
      self.safety.set_safety_hooks(structs.CarParams.SafetyModel.ford,param)
      self.safety.init_tests();self.safety.set_controls_allowed(True)
      self.assertFalse(self.tx())
      self.assertFalse(self.tx(action=0))

  def test_no_host_longitudinal_and_oem_acc_forwarding(self):
    self.inputs()
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x186,0,b'\0'*8)))
    for addr in (0x186,0x187):
      self.assertEqual(0,self.safety.safety_fwd_hook(2,addr))
    for addr in (0x3CA,0x3D3):
      self.assertEqual(-1,self.safety.safety_fwd_hook(2,addr))
    resume=self.packer.make_can_msg_safety('Steering_Data_FD1',0,{'CcAsllButtnResPress':1})
    self.assertFalse(self.safety.safety_tx_hook(resume))

  def test_active_lca_is_rejected_neutral_lca_is_allowed(self):
    self.inputs();cp=params(legacy=True);can=CanBus(cp)
    for active,curvature,expected in [(False,0,True),(True,0,False),(False,0.0001,False)]:
      addr,data,bus=create_lat_ctl_msg(self.packer,can,active,0,0,curvature,0)
      self.assertEqual(expected,self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr,bus,data)))

  def test_real_host_commands_pass_independent_native_checks(self):
    cp=params(legacy=True);cs=state(cp);cc=structs.CarControl(enabled=True,latActive=True)
    controller=LegacyLKALateral(cp);can=CanBus(cp)
    reference=curvature=0
    for tick in range(800):
      self.now=1_000_000+tick*30_000
      wheel=round(reference*REFERENCE_SCALE,1)
      self.inputs(wheel=wheel,curvature=curvature*0.000005)
      cs.out.steeringAngleDeg=wheel
      cs.out.yawRate=curvature*0.000005*20
      cs.legacy_lka_feedback['time_nanos']=self.now*1000
      cs.legacy_lka_feedback['wheel_time_nanos']=self.now*1000
      cs.legacy_lka_feedback['wheel_angle_deg']=wheel
      cc.actuators.curvature=0.0004*math.sin(tick/80)
      command=controller.update(cc,cs,self.now*1000)
      self.assertNotEqual(0,command.action)
      a,b,bus=create_legacy_lka_msg(can,command)
      self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a,bus,b)),(tick,command,wheel))
      reference,curvature=command.reference_raw,command.curvature_raw


if __name__=='__main__':unittest.main()
