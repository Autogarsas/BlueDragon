"""Independent native safety for the reference port; no permissive Panda port."""
import math
import unittest

from opendbc.car import structs
from opendbc.can import CANPacker
from opendbc.car.ford.fusion_lka_port import FusionLegacyLKAPort, FusionLKACommand, create_command
from opendbc.car.ford.tests.test_legacy_lka_option import params, state
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests import test_ford_legacy_lka as archived_legacy


class TestFordFusionLKAPortSafety(unittest.TestCase):
  TX_MSGS = [[0x83, 0], [0x83, 2], [0x18A, 0], [0x3CA, 0], [0x3D3, 0], [0x3D8, 0]]
  rx = archived_legacy.TestFordLegacyLKASafety.rx
  inputs = archived_legacy.TestFordLegacyLKASafety.inputs

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.ford, 40)
    self.safety.init_tests()
    self.packer = CANPackerSafety('ford_lincoln_base_pt')
    self.old = CANPackerSafety('ford_fusion_legacy_lka')
    self.encoder = CANPacker('ford_fusion_legacy_lka')
    self.now, self.count = 1_000_000, 0

  def tx(self, action=2, reference=0, curvature=0, bus=0):
    addr, data, _ = create_command(self.encoder, bus, FusionLKACommand(action, reference, curvature))
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def raw(self, action=2, reference=0, curvature=0, bus=0, extra=0):
    reference += 2048;curvature += 2048
    data=bytes(((action<<5)|(14 if action else 3),curvature>>4,
                ((curvature&15)<<4)|(reference>>8),reference&255,extra,0,0,0))
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x3CA,bus,data))

  def test_reference_action_and_neutral_only(self):
    self.inputs()
    for first in [0x23, 0x43, 0x83, 0x8e, 0xee, 0x5e, 0x4f]:
      packet = bytes([first]) + bytes.fromhex('80080000000000')
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x3CA, 0, packet)), hex(first))
    self.assertTrue(self.tx())
    self.assertTrue(self.tx(action=0))

  def test_camera_offset_does_not_compare_different_angle_origins(self):
    # Literal source values from the owner's rejected packet. The dedicated
    # port uses SCCM consistently and does not require ParkAid_Data at all.
    self.inputs(wheel=-1.9, camera=-3.1, omit=('ParkAid_Data',))
    self.assertTrue(self.tx(reference=-38))
    self.now += 10_000;self.inputs(wheel=-1.9, camera=40.)
    self.assertTrue(self.tx(reference=-38))

  def test_every_used_sensor_is_required_and_stale_inputs_stop_active(self):
    for sensor in ['wheel', 'EPAS_INFO', 'Lane_Assist_Data3_FD1', 'BrakeSysFeatures',
                   'EngVehicleSpThrottle2', 'Yaw_Data_FD1', 'EngBrakeData', 'EngVehicleSpThrottle', 'DesiredTorqBrk']:
      self.setUp();self.inputs(omit=(sensor,));self.safety.set_controls_allowed(True)
      self.assertFalse(self.tx(), sensor)
      self.assertTrue(self.tx(action=0))
    self.setUp();self.inputs();self.safety.set_timer(self.now+160_000)
    self.assertFalse(self.tx())

  def test_pedals_takeover_fault_status_and_speed_still_block_active(self):
    for args in [dict(enabled=False),dict(brake=True),dict(gas=1),dict(torque=1.0625),dict(torque=-1.0625),
                 dict(availability=0),dict(availability=1),dict(denied=True),dict(fault=1),dict(fault=2),
                 dict(speed=14.9),dict(speed=30.1),dict(wheel=5.6)]:
      self.setUp();self.inputs(**args)
      self.assertFalse(self.tx(), args)
      self.assertTrue(self.tx(action=0), args)

  def test_100hz_slew_timing_and_rejected_frames_do_not_seed_state(self):
    self.inputs();self.assertTrue(self.tx())
    self.safety.set_timer(self.now+4_999);self.assertFalse(self.tx())
    self.now += 10_000;self.inputs()
    self.assertFalse(self.tx(reference=2))
    self.assertTrue(self.tx(reference=1,curvature=1))
    self.now += 10_000;self.inputs()
    self.assertFalse(self.tx(reference=3,curvature=1))
    self.assertFalse(self.tx(reference=1,curvature=3))
    self.assertTrue(self.tx(reference=2,curvature=2))
    self.safety.set_timer(self.now+30_001);self.assertFalse(self.tx(reference=2,curvature=2))

  def test_invalid_flag_pairings_never_allow_even_neutral(self):
    for param in [32,33,34,36,41,42,44,48,56,63]:
      self.safety.set_safety_hooks(structs.CarParams.SafetyModel.ford,param)
      self.safety.init_tests();self.safety.set_controls_allowed(True)
      self.assertFalse(self.tx(), param)
      self.assertFalse(self.tx(action=0), param)

  def test_absolute_tracking_wrong_bus_and_override_limits_are_independent(self):
    for sign in [-1,1]:
      self.setUp();self.inputs(wheel=sign*5.5)
      self.assertTrue(self.raw(reference=sign*110))
      for reference in range(111,129):
        self.now+=10_000;self.inputs(wheel=sign*5.5)
        self.assertTrue(self.raw(reference=sign*reference))
      self.now+=10_000;self.inputs(wheel=sign*5.5)
      self.assertFalse(self.raw(reference=sign*129))
      self.setUp();self.inputs()
      for reference in range(0,41):
        self.assertTrue(self.raw(reference=sign*reference))
        self.now+=10_000;self.inputs()
      self.assertFalse(self.raw(reference=sign*41))
    self.setUp();self.inputs(curvature=.0015)
    self.assertFalse(self.raw(curvature=301))
    self.assertTrue(self.raw(curvature=300))
    self.assertFalse(self.raw(action=0,reference=1))
    self.assertFalse(self.raw(action=0,curvature=1))
    for bus in [1,2]:self.assertFalse(self.raw(bus=bus))
    for extra in [1,32,64,128]:self.assertFalse(self.raw(extra=extra))

  def test_no_longitudinal_resume_or_active_lca_authority(self):
    self.inputs()
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x186,0,bytes(8))))
    resume=self.packer.make_can_msg_safety('Steering_Data_FD1',0,{'CcAsllButtnResPress':1})
    self.assertFalse(self.safety.safety_tx_hook(resume))
    for address in [0x3CA,0x3D3]:self.assertEqual(-1,self.safety.safety_fwd_hook(2,address))
    self.assertEqual(0,self.safety.safety_fwd_hook(2,0x186))
    self.safety.set_alternative_experience(1024);self.safety.set_lkas_on(True)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self.tx())

  def test_real_host_stream_passes_native_for_5000_updates_without_off_pulses(self):
    cp=params(legacy=True,reference_port=True);cs=state(cp);cs.legacy_lka_tx_blocked=0
    cc=structs.CarControl(enabled=True,latActive=True);port=FusionLegacyLKAPort(cp)
    reference=curvature=0
    for i in range(5000):
      self.now=1_000_000+i*10_000
      wheel=round(reference*.05,1)
      self.inputs(wheel=wheel,camera=wheel-1.3,curvature=curvature*.000005)
      cs.out.steeringAngleDeg=wheel;cs.out.yawRate=curvature*.000005*20
      cs.legacy_lka_feedback.update(time_nanos=self.now*1000,wheel_time_nanos=self.now*1000,wheel_angle_deg=wheel)
      cc.actuators.torque=.008*math.sin(i/300)
      cc.actuators.steeringAngleDeg=.5*math.sin(i/300)
      command=port.update(cc,cs,self.now*1000)
      self.assertEqual(2,command.action,(i,port.diagnostics))
      a,b,bus=create_command(port.packer,0,command)
      self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(a,bus,b)),(i,command,wheel))
      reference,curvature=command.reference_raw,command.curvature_raw


if __name__ == '__main__':unittest.main()
