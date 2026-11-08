"""Offline LKA evidence audit; no vehicle, transport, Params or Panda API.

Missing inputs are not synthesized to make native replay appear complete.
A recording is not a vehicle model for hypothetical prolonged assistance.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from opendbc.car.ford.fusion_lka_port import reference_envelope
from tools.ford_legacy_lka.protocol import FUSION_NATIVE_RX_IDS


def analyze(report):
  frames = sorted(report['frames'], key=lambda row: row['mono_ns'])
  origin = frames[0]['mono_ns'] if frames else 0
  commands = sorted((row['command'] for row in report['states'] if row['kind'] == 'legacyLkaCommand'),
                    key=lambda row: row['command_time_nanos'])
  tx = [row for row in frames if row['stream'] == 'sendcan' and row['bus'] == 0 and row['address'] == '0x3ca']
  rejected = [row for row in frames if row['stream'] == 'can' and row['bus'] == 192 and row['address'] == '0x3ca']
  present = {int(row['address'], 16) for row in frames
             if row['stream'] == 'can' and row['bus'] == 0 and row['event_valid']}
  missing = sorted(FUSION_NATIVE_RX_IDS - present)
  episodes, current = [], None
  for row in tx:
    if bytes.fromhex(row['data'])[0] >> 5:
      if current is None:
        current = dict(start_s=(row['mono_ns'] - origin) / 1e9, packets=0, first_ns=row['mono_ns'])
      current['packets'] += 1
    elif current is not None:
      current['duration_s'] = (row['mono_ns'] - current.pop('first_ns')) / 1e9
      episodes.append(current)
      current = None
  if current is not None:
    current.pop('first_ns')
    current['end_not_observed'] = True
    episodes.append(current)
  stops, previous = [], None
  for row in commands:
    if row['latched'] and previous is not None and not previous['latched']:
      stop = dict(time_s=(row['command_time_nanos'] - origin) / 1e9,
                  reason=row['latch_reason'] or row['reason'], availability=row['availability'],
                  denied=row['denied'])
      if row['reason'] == 'output_tracking_limit' and previous['action'] != 0:
        stop['recorded_reference_replay'] = reference_envelope(
          row['normalized_pid'] * 87., row['steering_angle_deg'], previous['reference_raw'] - 2048, True)
        stop['physical_units_validated'] = False
      stops.append(stop)
    previous = row
  return dict(active_packets=sum(bool(bytes.fromhex(row['data'])[0] >> 5) for row in tx),
              rejected_packets=len(rejected), active_episodes=episodes, stops=stops,
              stop_counts=dict(Counter(row['reason'] for row in stops)),
              native_rx_missing=[hex(address) for address in missing],
              native_replay_inputs_complete=not missing and not report.get('truncated', False),
              native_rejections_exactly_reproduced=False,
              limitations=['CAN receipt timestamps do not identify native processing order.',
                           'No physical steering response or duration cutoff is proven.',
                           'Missing native RX inputs are never synthesized.'])


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', type=Path, required=True)
  parser.add_argument('--output', type=Path)
  args = parser.parse_args()
  raw = args.input.read_bytes()
  result = analyze(json.loads(raw))
  result['input_sha256'] = hashlib.sha256(raw).hexdigest()
  result['input_name'] = args.input.name
  output = json.dumps(result, indent=2, allow_nan=False) + '\n'
  if args.output:
    with args.output.open('x') as destination:
      destination.write(output)
  print(output, end='')


if __name__ == '__main__':
  main()
