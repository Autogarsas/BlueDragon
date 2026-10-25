"""Check recorded-log aggregation and deterministic installer configuration."""
import json
import tempfile
from pathlib import Path
from cereal import log
from dragonpilot.selfdrive.car.ford_diagnostics import FordSteeringDiagnostics
from tools.ford_bluepilot.check_drive_logs import report
from tools.ford_bluepilot.build_dev_installer import configure


def main():
  records = []
  def emit(name, **data):
    event = log.Event.new_message(logMonoTime=data['end_nanos'])
    event.logMessage = json.dumps({'msg': dict(event=name, **data)})
    records.append(event.to_bytes())
  observer = FordSteeringDiagnostics(emit, 'FORD_EDGE_MK2')
  for i in range(61):
    observer.observe(dict(sequence=i+1, command_time_nanos=i*50_000_000,
                          reason='human_turn' if i == 3 else 'active', output_curvature=.001,
                          curvature_rate_limited=i in (3, 6)))
  with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)/'rlog'
    path.write_bytes(b''.join(records))
    result = report([path])
    assert not result['read_errors'], result
    diag = result['ford_host_diagnostics']
    assert diag['windows'] == 3 and diag['samples'] == 61, diag
    assert diag['sample_counts']['human_turn'] == 1, diag
    assert diag['sample_counts']['curvature_rate_limited'] == 2, diag
  source = Path('installer/bluedragon-1.0').read_bytes()
  output = configure(source)
  assert output == Path('installer/bluedragon-dev').read_bytes()
  assert len(source) == len(output)
  assert b'https://github.com/Autogarsas/BlueDragon.git?' in output
  assert b'bluedragon-dev?' in output and b'bluedragon-1.0?' not in output
  try:
    configure(b'wrong')
  except ValueError:
    pass
  else:
    raise AssertionError('Corrupt installer accepted')
  print('Recorded Capnp log roundtrip, aggregation and installer checks passed')


if __name__ == '__main__':
  main()
