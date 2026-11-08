"""Read current-boot Panda startup evidence; no USB, resets or CAN writes."""
from pathlib import Path
import json
import math
import re
import time

ROOT = Path(__file__).resolve().parents[2]
PATTERN = re.compile(r'version: (\S+), signature ([0-9a-f]+), expected ([0-9a-f]+)')


def firmware_startup_status(root=ROOT, log_root=Path('/data/log'), boot_time=None):
  release = json.loads((root / 'tools/ford_legacy_lka/release.json').read_text())
  firmware = json.loads((root / release['h7_manifest']).read_text())
  status = dict(verified=False, packaged_version=firmware['build_version'],
                packaged_signature_prefix=firmware['firmware_signature_prefix'],
                evidence='current-boot startup log; not a direct device read')
  if release.get('package_status') == 'native-build-pending':
    return dict(status, reason='native_package_pending')
  if boot_time is None:
    boot_time = time.time() - time.monotonic()
  latest = None
  # A host-only release may intentionally reuse firmware. Require current-boot
  # evidence and an actual signature match, rather than a particular host SHA.
  for path in log_root.glob('swaglog.*'):
    try:
      if path.stat().st_mtime < boot_time:
        continue
      with path.open(errors='replace') as source:
        for line in source:
          if 'connected, version:' not in line:
            continue
          try:
            record = json.loads(line)
            created = float(record['created'])
          except (ValueError, KeyError, TypeError):
            continue
          message = record.get('msg$s', record.get('msg'))
          match = PATTERN.search(message) if isinstance(message, str) else None
          if match and math.isfinite(created) and created >= boot_time and (latest is None or created >= latest[0]):
            latest = (created, match.groups(), record.get('ctx', {}).get('commit'))
    except OSError:
      continue
  if latest is None:
    return dict(status, reason='no_current_boot_startup_entry')
  created, (actual, loaded, expected), commit = latest
  verified = (actual == status['packaged_version'] and
              loaded == expected == status['packaged_signature_prefix'])
  return dict(status, verified=verified, reason='matched' if verified else 'latest_startup_mismatch',
              actual_version=actual, actual_signature_prefix=loaded,
              startup_expected_signature_prefix=expected, startup_commit=commit, startup_created=created)


def main():
  status = firmware_startup_status()
  print(json.dumps(status, indent=2))
  if not status['verified']:
    print('Loaded firmware is unverified. Keep Legacy LKA off and send this output before another test.')
    return 1
  print('Matching firmware startup evidence found for the current boot.')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
