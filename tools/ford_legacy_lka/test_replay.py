"""Captured stop boundaries, evidence completeness and current-boot firmware."""
import json
from pathlib import Path
import tempfile
import unittest

from opendbc.car.ford.fusion_lka_port import reference_envelope
from tools.ford_legacy_lka import replay
from tools.ford_legacy_lka.check_firmware import firmware_startup_status
from tools.ford_legacy_lka.protocol import Session, FUSION_NATIVE_RX_IDS


class TestCapturedStopBoundaries(unittest.TestCase):
  def test_recorded_first_tracking_stop_has_no_safe_one_count_command(self):
    for pid, wheel, previous in [(.04991314560174942, .7000000000000455, 60)]:
      result = reference_envelope(pid * 87., wheel, previous, True)
      self.assertFalse(result['reference_feasible'])
      self.assertGreater(result['reference_feasible_lo_raw'], result['reference_feasible_hi_raw'])
      for candidate in range(previous - 1, previous + 2):
        self.assertGreater(abs(candidate * .05 - wheel), 1.9)

  def test_recorded_second_stop_is_a_float_boundary_not_a_larger_error(self):
    result = reference_envelope(-.0274135023355484 * 87., -.1999999999998181, -43, True)
    self.assertTrue(result['reference_feasible'])
    self.assertEqual(-42, result['candidate_reference_raw'])
    self.assertEqual(-38, result['numeric_tracking_error_raw'])
    # Both signs and the immediately adjacent unsafe count remain bounded.
    for wheel in [-.1999999999998181, .1999999999998181]:
      sign = -1 if wheel < 0 else 1
      safe = reference_envelope(sign * 6., wheel, sign * 43, True)
      unsafe = reference_envelope(sign * 6., wheel, sign * 44, True)
      self.assertTrue(safe['reference_feasible'])
      self.assertFalse(unsafe['reference_feasible'])

  def test_no_missing_required_native_input_can_be_hidden_by_echoes(self):
    session = Session('fusion_2019', 'after')
    for address in FUSION_NATIVE_RX_IDS:
      bus = 128 if address == 0x213 else 0
      session.add_frame('can', bus, address, bytes(8), 1, True)
    result = session.report()
    self.assertFalse(result['native_rx_complete'])
    self.assertEqual(['0x213'], result['native_rx_missing'])
    session.add_frame('can', 0, 0x213, bytes(8), 2, True)
    self.assertTrue(session.report()['native_rx_complete'])

  def test_replay_is_explicit_about_truncation_and_missing_native_proof(self):
    session = Session('fusion_2019', 'after')
    for address in FUSION_NATIVE_RX_IDS:
      session.add_frame('can', 0, address, bytes(8), 1, True)
    report = session.report()
    report['truncated'] = True
    audit = replay.analyze(report)
    self.assertFalse(audit['native_replay_inputs_complete'])
    self.assertFalse(audit['native_rejections_exactly_reproduced'])


class TestFirmwareStartupEvidence(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    self.logs = self.root / 'logs'
    self.logs.mkdir()
    release = self.root / 'tools/ford_legacy_lka/release.json'
    release.parent.mkdir(parents=True)
    release.write_text(json.dumps({'h7_manifest': 'h7.json', 'package_status': 'paired-development'}))
    (self.root / 'h7.json').write_text(json.dumps({'build_version': 'DEV-c785bdc5-DEBUG',
                                                'firmware_signature_prefix': '5de6e8a8352bce37'}))

  def write(self, created, version='DEV-c785bdc5-DEBUG', signature='5de6e8a8352bce37', key='msg$s'):
    record = {'created': created, key: f'Panda internal connected, version: {version}, signature {signature}, expected 5de6e8a8352bce37',
              'ctx': {'commit': 'previous-host-only-commit'}}
    with (self.logs / 'swaglog.0000000001').open('a') as output:
      output.write(json.dumps(record) + '\n')

  def status(self):
    return firmware_startup_status(self.root, self.logs, boot_time=100.)

  def test_old_boot_cannot_verify_current_firmware(self):
    self.write(99.)
    self.assertFalse(self.status()['verified'])

  def test_current_boot_with_reused_firmware_does_not_require_same_host_commit(self):
    for key in ['msg', 'msg$s']:
      self.write(101., key=key)
      self.assertTrue(self.status()['verified'])

  def test_latest_mismatch_overrides_older_matching_startup(self):
    self.write(101.)
    self.write(102., signature='0123456789abcdef')
    self.assertFalse(self.status()['verified'])
    self.assertEqual('latest_startup_mismatch', self.status()['reason'])

  def test_packaged_expectation_and_matching_signature_do_not_validate_wrong_version(self):
    self.write(101., version='DEV-wrong-DEBUG')
    self.assertFalse(self.status()['verified'])


if __name__ == '__main__':
  unittest.main()
