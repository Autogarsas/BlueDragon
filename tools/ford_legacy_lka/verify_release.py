"""Verify the paired LKA host/settings/H7 package without vehicle access."""
from pathlib import Path
import hashlib
import json
import struct
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from tools.chrysler_alka.verify_release import verify as verify_h7


def main():
  manifest=json.loads((ROOT/'tools/ford_legacy_lka/release.json').read_text())
  if manifest.get('package_status') == 'native-build-pending':
    raise RuntimeError('Fusion reference port is source-only: paired H7 build is not packaged. Do not install.')
  verify_h7(ROOT,json.loads((ROOT/manifest['h7_manifest']).read_text()))
  binary=(ROOT/'common/params_pyx.so').read_bytes()
  if hashlib.sha256(binary).hexdigest()!=manifest['params_sha256']:
    raise RuntimeError('Settings-library checksum mismatch')
  if binary[:6]!=b'\x7fELF\x02\x01' or struct.unpack_from('<H',binary,18)[0]!=183:
    raise RuntimeError('Expected little-endian 64-bit ARM settings library')
  for name,digest in manifest['source_checksums'].items():
    p=(ROOT/name).resolve()
    if not p.is_relative_to(ROOT) or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
      raise RuntimeError('Source/package mismatch: '+name)
  h7=json.loads((ROOT/manifest['h7_manifest']).read_text())
  print('Verified legacy LKA H7:',h7['build_version'],'signature prefix:',h7['firmware_signature_prefix'])
  print('Verified ARM settings library:',manifest['registered_keys'],'keys')
  print('Paired mode: Ford safety',manifest['safety_param'],'; factory ACC; LCA remains the default when the LKA setting is off.')
  print('Package verification does not establish vehicle response, reference calibration or a timer lockout.')


if __name__=='__main__':main()
