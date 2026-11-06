"""Install the matched LKA test branch from an offroad comma, then reboot.

Preserves tracked edits by refusing them, saves the previous commit once, and
uses an explicit no-track branch switch to avoid single-branch clone failures.
"""
import argparse
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

ROOT = Path('/data/openpilot')
BRANCH = 'ford-lka-option'


def run(*args, capture=False):
  result = subprocess.run(args, cwd=ROOT, check=True, text=True,
                          stdout=subprocess.PIPE if capture else None)
  return result.stdout.strip() if capture else None


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--check', action='store_true', help='Read installation preconditions without changing anything')
  parser.add_argument('--commit', required=True, help='Exact reviewed release commit (40 hexadecimal characters)')
  args = parser.parse_args()
  if re.fullmatch(r'[0-9a-f]{40}', args.commit) is None:
    raise SystemExit('Stop: --commit must be the exact release commit')
  if platform.machine() != 'aarch64' or not ROOT.is_dir():
    raise SystemExit('This installer is for the AGNOS comma at /data/openpilot')
  offroad = Path('/data/params/d/IsOffroad')
  if not offroad.exists() or offroad.read_bytes().strip() != b'1':
    raise SystemExit('Stop: turn ignition off and wait for the offroad screen first')
  sys.path.insert(0, str(ROOT))
  from cereal import messaging
  sm = messaging.SubMaster(['pandaStates'])
  deadline = time.monotonic()+3
  while time.monotonic() < deadline:sm.update(200)
  if not sm.alive['pandaStates'] or not len(sm['pandaStates']):
    raise SystemExit('Stop: no live Panda ignition status; nothing changed')
  if any(p.ignitionLine or p.ignitionCan for p in sm['pandaStates']):
    raise SystemExit('Stop: ignition is still ON; forced offroad is not sufficient for installation')
  dirty = run('git', 'status', '--porcelain', '--untracked-files=no', capture=True)
  if dirty:
    raise SystemExit('Stop: tracked files have changes; nothing was discarded:\n' + dirty)
  origin = run('git', 'remote', 'get-url', 'origin', capture=True)
  if origin not in ('https://github.com/Autogarsas/BlueDragon.git',
                    'https://github.com/Autogarsas/BlueDragon',
                    'git@github.com:Autogarsas/BlueDragon.git'):
    raise SystemExit('Stop: origin is not the expected BlueDragon repository')
  previous = run('git', 'rev-parse', 'HEAD', capture=True)
  print('Offroad and clean. Current commit:', previous, flush=True)
  if args.check:
    print('Read-only checks passed; no files, settings or firmware changed')
    return
  backup = Path('/data/bluedragon-before-legacy-lka.commit')
  if not backup.exists():
    with backup.open('x') as f:f.write(previous+'\n')
  run('git', 'remote', 'set-branches', '--add', 'origin', BRANCH)
  run('git', 'fetch', '--no-tags', 'origin',
      f'refs/heads/{BRANCH}:refs/remotes/origin/{BRANCH}')
  if run('git','rev-parse',f'origin/{BRANCH}',capture=True) != args.commit:
    raise SystemExit('Stop: branch differs from the reviewed release commit; nothing was checked out')
  exists = subprocess.run(['git','show-ref','--verify','--quiet',f'refs/heads/{BRANCH}'],cwd=ROOT).returncode == 0
  if exists:
    run('git', 'switch', BRANCH)
    run('git', 'merge', '--ff-only', f'origin/{BRANCH}')
  else:
    run('git', 'switch', '--no-track', '-c', BRANCH, f'origin/{BRANCH}')
  run('git', 'branch', f'--set-upstream-to=origin/{BRANCH}', BRANCH)
  run(sys.executable, 'tools/ford_legacy_lka/verify_release.py')
  # Exercise the updated wire schema and host-stop policy in a fresh process
  # while ignition is off, before restarting any vehicle-control services.
  run(sys.executable, '-c',
      "from cereal import log; "
      "from openpilot.selfdrive.selfdrived.events import EVENTS; "
      "from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD; "
      "from openpilot.selfdrive.controls.controlsd import Controls; "
      "from openpilot.selfdrive.car.card import Car; "
      "assert log.OnroadEvent.EventName.fordLegacyLkaLatched in EVENTS; "
      "print('Native host-stop schema and service imports verified')")
  run(sys.executable, '-m', 'unittest',
      'tools.ford_legacy_lka.test_research',
      'tools.ford_legacy_lka.test_diagnostics',
      'tools.ford_legacy_lka.test_host_latch')
  # A fresh process must load the newly installed settings library, even if
  # earlier status imports cached the previous native module in this process.
  run(sys.executable, '-c',
      "from openpilot.common.params import Params; Params().put_bool('dp_ford_legacy_lka', True, block=True)")
  print('Matched package verified. Legacy LKA selected; rebooting to apply.', flush=True)
  run('sudo', 'reboot')


if __name__ == '__main__':
  try:
    main()
  except subprocess.CalledProcessError as ex:
    raise SystemExit(f'Stopped at command {ex.cmd!r}, exit {ex.returncode}. Send this output; do not force the next step.')
