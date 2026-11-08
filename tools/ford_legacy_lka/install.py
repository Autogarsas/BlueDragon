"""Install from a parked, no-output, offroad comma, then reboot.

Preserves tracked edits by refusing them, saves the previous commit once, and
uses an explicit no-track branch switch to avoid single-branch clone failures.
"""
import argparse
from dataclasses import dataclass, field
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

ROOT = Path('/data/openpilot')
BRANCH = 'ford-fusion-lka-port'


@dataclass
class ParkedInstallGuard:
  """Receive-only Ford Park/stationary proof; ignition may remain ON."""
  samples: dict = field(default_factory=dict)
  stable_since: int | None = None
  speed_counter: int | None = None
  speed_updates: int = 0
  last_speed_advance: int | None = None

  def observe(self, address, data, mono_ns, valid=True, bus=0):
    if bus != 0 or address not in (0x415, 0x230, 0x165):
      return
    if not valid or len(data) != 8:
      raise SystemExit('Stop: invalid live parked-state CAN data; nothing may be installed')
    previous = self.samples.get(address)
    if previous is not None and mono_ns < previous:
      raise SystemExit('Stop: out-of-order parked-state CAN data')
    if previous is not None and mono_ns - previous > 300_000_000:
      self.stable_since = None
    if address == 0x415:
      quality, counter = data[2] >> 6, (data[2] >> 2) & 15
      checksum = (255 - (data[0] + data[1] + quality + counter)) & 255
      if quality != 3 or checksum != data[3]:
        raise SystemExit('Stop: invalid ABS speed quality/checksum')
      if data[0] or data[1]:
        raise SystemExit('Stop: vehicle is moving; select Park and stay stationary')
      if self.speed_counter != counter:
        self.speed_updates += 1
        self.last_speed_advance = mono_ns
      self.speed_counter = counter
    elif address == 0x230 and ((data[1] >> 1) & 15) != 0:
      raise SystemExit('Stop: transmission is not in Park')
    elif address == 0x165 and (data[1] & 7) in (4, 5):
      raise SystemExit('Stop: factory cruise is engaged')
    self.samples[address] = mono_ns

  def ready(self, now_ns):
    fresh = all(address in self.samples and 0 <= now_ns - self.samples[address] <= 300_000_000
                for address in (0x415, 0x230, 0x165))
    if not fresh:
      self.stable_since = None
      return False
    if self.stable_since is None:
      self.stable_since = now_ns
    return (now_ns - self.stable_since >= 1_000_000_000 and self.speed_updates >= 3
            and self.last_speed_advance is not None and now_ns - self.last_speed_advance <= 100_000_000)


def panda_no_output(pandas):
  return bool(pandas) and all(str(p.safetyModel) == 'noOutput' and p.safetyParam == 0
                              and not p.controlsAllowed for p in pandas)


def check_parked_offroad():
  from cereal import messaging
  root = Path('/data/params/d')
  sm = messaging.SubMaster(['pandaStates', 'carControl'])
  sock = messaging.sub_sock('can', timeout=0)
  guard = ParkedInstallGuard()
  deadline = time.monotonic() + 6
  while time.monotonic() < deadline:
    if not (root / 'IsOffroad').exists() or (root / 'IsOffroad').read_bytes().strip() != b'1':
      raise SystemExit('Stop: select offroad mode and keep the car parked')
    onroad = root / 'IsOnroad'
    if onroad.exists() and onroad.read_bytes().strip() == b'1':
      raise SystemExit('Stop: driving mode is still running')
    sm.update(50)
    if sm.updated['pandaStates'] and (not sm.valid['pandaStates'] or not panda_no_output(sm['pandaStates'])):
      raise SystemExit('Stop: wait for offroad Panda noOutput / 0 with controls disallowed')
    if sm.updated['carControl']:
      cc = sm['carControl']
      if cc.enabled or cc.latActive or cc.longActive:
        raise SystemExit('Stop: a driving-control service is still active')
    for _ in range(200):
      event = messaging.recv_one_or_none(sock)
      if event is None:
        break
      for frame in event.can:
        guard.observe(int(frame.address), bytes(frame.dat), int(event.logMonoTime), bool(event.valid), int(frame.src))
    live_panda = sm.all_checks(['pandaStates']) and panda_no_output(sm['pandaStates'])
    if live_panda and guard.ready(time.monotonic_ns()):
      print('Verified live Park, zero speed, cruise disengaged, offroad and Panda noOutput. Ignition may remain ON.', flush=True)
      return
  raise SystemExit('Stop: live Park / zero speed / cruise-off / Panda noOutput proof is missing or stale; stay offroad')


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
  sys.path.insert(0, str(ROOT))
  check_parked_offroad()
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
  check_parked_offroad()
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
      'tools.ford_legacy_lka.test_host_latch',
      'tools.ford_legacy_lka.test_install',
      'tools.ford_legacy_lka.test_replay',
      'opendbc.car.ford.tests.test_fusion_lka_port',
      'opendbc.safety.tests.test_ford_fusion_lka_port')
  # The vibration correction is not vehicle-validated. Installation must not
  # automatically re-enable the experimental steering path.
  run(sys.executable, '-c',
      "from openpilot.common.params import Params; Params().put_bool('dp_ford_legacy_lka', False, block=True)")
  check_parked_offroad()
  print('Matched candidate verified. Legacy LKA remains disabled; rebooting to apply.', flush=True)
  run('sudo', 'reboot')


if __name__ == '__main__':
  try:
    main()
  except subprocess.CalledProcessError as ex:
    raise SystemExit(f'Stopped at command {ex.cmd!r}, exit {ex.returncode}. Send this output; do not force the next step.')
