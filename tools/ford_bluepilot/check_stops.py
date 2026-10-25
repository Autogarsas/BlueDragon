#!/usr/bin/env python3
"""Read recorded stop/restart states. Does not alter controls, settings or logs."""
import argparse
import bz2
from collections import Counter, deque
import json
from pathlib import Path
import sys

ROOT = Path('/data/openpilot')
if ROOT.is_dir():
  sys.path.insert(0, str(ROOT))
from cereal import log
import zstandard


def read_events(path):
  raw = path.read_bytes()
  if path.suffix == '.zst':
    with zstandard.ZstdDecompressor().stream_reader(raw) as reader:
      raw = reader.read()
  elif path.suffix == '.bz2':
    raw = bz2.decompress(raw)
  yield from log.Event.read_multiple_bytes(raw)


def report(paths):
  latest, times = {}, {}
  episodes = deque(maxlen=8)
  current = None
  counts, invalid = Counter(), Counter()
  errors = []
  previous_segment = None

  def snapshot(t):
    row = {'t': round(t, 2)}
    for key, fields in latest.items():
      age = t-times[key]
      row[key+'_age'] = round(age, 2)
      if 0 <= age <= 1.0:
        row.update(fields)
    return row

  def close(partial):
    nonlocal current
    if current is not None:
      current['partial_end'] = partial
      episodes.append(current)
      current = None

  for path in paths:
    segment = int(path.parent.name.rsplit('--', 1)[1])
    if previous_segment is not None and segment != previous_segment+1:
      close(True)
      latest.clear()
      times.clear()
    previous_segment = segment
    try:
      for event in read_events(path):
        kind = event.which()
        if kind not in ('carControl', 'carState', 'modelV2', 'longitudinalPlan', 'radarState', 'selfdriveState'):
          continue
        counts[kind] += 1
        if not event.valid:
          invalid[kind] += 1
          continue
        t = event.logMonoTime*1e-9
        if kind == 'carControl':
          c = event.carControl
          fields = dict(long_active=bool(c.longActive), enabled=bool(c.enabled), resume=bool(c.cruiseControl.resume),
                        actuator_accel=round(c.actuators.accel, 3), long_state=str(c.actuators.longControlState))
        elif kind == 'carState':
          c = event.carState
          fields = dict(kmh=round(c.vEgo*3.6, 2), vehicle_stopped=bool(c.standstill),
                        ford_hold=bool(c.cruiseState.standstill), brake=bool(c.brakePressed), gas=bool(c.gasPressed),
                        acc_enabled=bool(c.cruiseState.enabled))
        elif kind == 'modelV2':
          c = event.modelV2
          fields = dict(model_stop=bool(c.action.shouldStop), model_accel=round(c.action.desiredAcceleration, 3))
        elif kind == 'longitudinalPlan':
          c = event.longitudinalPlan
          fields = dict(plan_stop=bool(c.shouldStop), plan_accel=round(c.aTarget, 3),
                        source=str(c.longitudinalPlanSource), allow_throttle=bool(c.allowThrottle))
        elif kind == 'radarState':
          c = event.radarState.leadOne
          fields = dict(lead_m=round(c.dRel, 2) if c.status else None,
                        lead_kmh=round(c.vLead*3.6, 2) if c.status else None)
        else:
          c = event.selfdriveState
          fields = dict(experimental=bool(c.experimentalMode))
        latest[kind] = fields
        times[kind] = t
        if kind != 'carState':
          continue
        row = snapshot(t)
        stopped = row['vehicle_stopped'] or row['kmh'] < 1.08
        engaged = row.get('long_active', False)
        if current is None and stopped and engaged:
          current = dict(start=t, duration_s=0., sampled_seconds=0, state_counts={}, first=row,
                         last=row, changes=[], partial_start=True)
          # We see stop entry, but not the full approach or the actual lamp colour.
          current['partial_start'] = counts['carState'] == 1
        if current is None:
          continue
        current['duration_s'] = round(t-current['start'], 1)
        signature = tuple(row.get(k) for k in ('model_stop', 'plan_stop', 'ford_hold', 'resume', 'long_active', 'brake', 'gas'))
        if signature != current.pop('_signature', None) and len(current['changes']) < 6:
          current['changes'].append(row)
        current['_signature'] = signature
        current['last'] = row
        if t-current.get('_sample_t', -1e20) >= 1.0:
          current['_sample_t'] = t
          current['sampled_seconds'] += 1
          state_counts = current['state_counts']
          for name, flag in (
            ('model_stop', row.get('model_stop') is True),
            ('plan_stop', row.get('plan_stop') is True),
            ('model_clear_plan_stop', row.get('model_stop') is False and row.get('plan_stop') is True),
            ('plan_clear_ford_hold', row.get('plan_stop') is False and row.get('ford_hold') is True),
            ('resume_requested', row.get('resume') is True),
            ('brake_pressed', row.get('brake') is True),
            ('missing_model', 'model_stop' not in row), ('missing_plan', 'plan_stop' not in row)):
            state_counts[name] = state_counts.get(name, 0)+int(flag)
        if not stopped or not engaged:
          close(not engaged)
    except Exception as exc:
      close(True)
      latest.clear()
      times.clear()
      errors.append(f'{path.parent.name}/{path.name}: {type(exc).__name__}: {str(exc)[:100]}')
  close(True)
  clean = [{k:v for k,v in episode.items() if not k.startswith('_')} for episode in episodes]
  return dict(samples=dict(counts), invalid=dict(invalid), last_stops=clean, read_errors=errors)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--route', required=True)
  parser.add_argument('--logs', type=Path, default=Path('/data/media/0/realdata'))
  parser.add_argument('--segments', type=int, default=30, help='Most recent segments; 0 reads all')
  args = parser.parse_args()
  if args.segments < 0:
    parser.error('--segments must be >= 0')
  paths = []
  for directory in args.logs.glob(args.route+'--*'):
    if not directory.is_dir() or not directory.name.rsplit('--', 1)[1].isdigit():
      continue
    for name in ('rlog.zst', 'rlog.bz2', 'rlog', 'qlog.zst', 'qlog.bz2', 'qlog'):
      p = directory/name
      if p.is_file() and p.stat().st_size:
        paths.append(p)
        break
  paths.sort(key=lambda p:int(p.parent.name.rsplit('--', 1)[1]))
  if args.segments:
    paths = paths[-args.segments:]
  if not paths:
    raise RuntimeError('No recorded logs found for this route')
  print('READ ONLY:', args.route, '| segments:', ','.join(p.parent.name.rsplit('--',1)[1] for p in paths), flush=True)
  print('Reading logs; long routes can take a few minutes.', flush=True)
  result = report(paths)
  print('Samples:', json.dumps(result['samples']), flush=True)
  print('Invalid:', json.dumps(result['invalid']), '| Read errors:', json.dumps(result['read_errors']), flush=True)
  keys = ('t', 'kmh', 'model_stop', 'plan_stop', 'ford_hold', 'resume', 'long_active', 'long_state',
          'model_accel', 'plan_accel', 'brake', 'gas', 'lead_m')
  for index, episode in enumerate(result['last_stops'], 1):
    print('STOP', index, '| duration', episode['duration_s'], 's | partial end', episode['partial_end'], flush=True)
    print('Counts (~1Hz):', json.dumps(episode['state_counts']), flush=True)
    rows = episode['changes'] + [episode['last']]
    for row in rows:
      print(json.dumps({k: row.get(k) for k in keys}), flush=True)
  if not result['last_stops']:
    print('No engaged stops found in these segments.', flush=True)
  print('No lamp colours or exact stop-line distance are logged here. Counts are ~1 Hz samples, not exact durations.')
  print('Model-clear/plan-stop is a clue, not proof that the lamp was green. Host resume does not prove Ford accepted it.')


if __name__ == '__main__':
  main()
