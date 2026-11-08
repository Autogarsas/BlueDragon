"""Parked ignition-on installation policy, exercised without vehicle I/O."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from cereal import log

from tools.ford_legacy_lka import install
from opendbc.can import CANPacker
from opendbc.safety.tests.test_ford import checksum


class TestParkedInstall(unittest.TestCase):
  def setUp(self):
    self.guard = install.ParkedInstallGuard()
    self.packer = CANPacker('ford_lincoln_base_pt')

  def feed(self, now, speed=0., gear=0, cruise=0, counter=0):
    values = [('BrakeSysFeatures', {'Veh_V_ActlBrk': speed, 'VehVActlBrk_D_Qf': 3,
                                  'VehVActlBrk_No_Cnt': counter}),
              ('TransGearData', {'GearLvrPos_D_Actl': gear}),
              ('EngBrakeData', {'CcStat_D_Actl': cruise})]
    for name, signals in values:
      packet = self.packer.make_can_msg(name, 0, signals)
      if name == 'BrakeSysFeatures':
        packet = checksum(packet)
      address, data, bus = packet
      self.guard.observe(address, data, now, bus=bus)

  def test_fresh_stationary_park_is_required_continuously(self):
    for i in range(12):
      self.feed(i * 100_000_000, counter=i)
      self.assertEqual(i >= 10, self.guard.ready(i * 100_000_000))
    self.assertFalse(self.guard.ready(2_000_000_000))

  def test_motion_drive_cruise_invalid_checksum_and_quality_are_refused(self):
    for values in [dict(speed=.01), dict(gear=3), dict(gear=1), dict(gear=15), dict(cruise=4), dict(cruise=5)]:
      with self.subTest(values=values), self.assertRaises(SystemExit):
        self.feed(0, **values)
    self.guard = install.ParkedInstallGuard()
    with self.assertRaises(SystemExit):
      self.guard.observe(0x415, bytes(8), 0)

  def test_missing_gear_invalid_events_echoes_and_stale_samples_never_prove_park(self):
    for i in range(20):
      self.guard.observe(0x230, bytes(8), i * 100_000_000, bus=128)
      self.assertFalse(self.guard.ready(i * 100_000_000))
    with self.assertRaises(SystemExit):
      self.guard.observe(0x230, bytes(8), 0, valid=False)
    with self.assertRaises(SystemExit):
      self.guard.observe(0x230, bytes(7), 0)

  def test_repeated_speed_counter_and_can_gap_do_not_prove_stable_park(self):
    for i in range(12):
      self.feed(i * 100_000_000)
      self.assertFalse(self.guard.ready(i * 100_000_000))
    self.feed(2_000_000_000, counter=1)
    self.assertFalse(self.guard.ready(2_000_000_000))

  def test_ignition_on_is_allowed_only_with_nooutput_and_no_control_permission(self):
    panda = NS(safetyModel='noOutput', safetyParam=0, controlsAllowed=False,
               ignitionLine=True, ignitionCan=True)
    self.assertTrue(install.panda_no_output([panda]))
    self.assertFalse(install.panda_no_output([]))
    for field, value in [('safetyModel', 'ford'), ('safetyParam', 40), ('controlsAllowed', True)]:
      with patch.object(panda, field, value):
        self.assertFalse(install.panda_no_output([panda]))
    message = log.Event.new_message()
    actual = message.init('pandaStates', 1)[0]
    actual.safetyModel, actual.safetyParam = 'noOutput', 0
    actual.ignitionLine, actual.ignitionCan = True, True
    actual.controlsAllowed = False
    self.assertTrue(install.panda_no_output(message.pandaStates))

  def test_full_installer_rechecks_park_before_checkout_and_reboot_and_leaves_lka_off(self):
    calls = []
    sha = '1' * 40
    def run(*args, **kwargs):
      calls.append(args)
      if args[:3] == ('git', 'remote', 'get-url'):
        return 'https://github.com/Autogarsas/BlueDragon.git'
      if args[:2] == ('git', 'rev-parse'):
        return sha
      return ''
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      with (patch.object(install, 'ROOT', root), patch.object(install.platform, 'machine', return_value='aarch64'),
            patch.object(install, 'check_parked_offroad') as check,
            patch.object(install, 'run', side_effect=run),
            patch.object(install.subprocess, 'run', return_value=NS(returncode=0)),
            patch.object(install, 'Path', side_effect=lambda name: root / Path(name).name),
            patch.object(sys, 'argv', ['install.py', '--commit', sha])):
        install.main()
      self.assertEqual(3, check.call_count)
      self.assertTrue(any('dp_ford_legacy_lka' in str(call) and 'False' in str(call) for call in calls))
      self.assertEqual(('sudo', 'reboot'), calls[-1])


if __name__ == '__main__':
  unittest.main()
