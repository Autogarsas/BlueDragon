import json
import unittest
from dragonpilot.selfdrive.car.ford_diagnostics import FordSteeringDiagnostics


class TestFordDiagnostics(unittest.TestCase):
  def test_transitions_deduplication_and_bounded_logging(self):
    events = []
    observer = FordSteeringDiagnostics(lambda name, **data: events.append(data), 'FORD_EDGE_MK2')
    for i in range(601):
      snapshot = dict(sequence=i+1, command_time_nanos=i*50_000_000, reason='human_turn' if i == 4 else 'active',
                      output_curvature=.001, curvature_rate_limited=i in (3, 5), speed_mps=float('nan'))
      observer.observe(snapshot)
      observer.observe(snapshot)
    self.assertEqual(len(events), 30)
    self.assertEqual(events[0]['samples'], 21)
    self.assertEqual(events[0]['counts'], {'active':20, 'human_turn':1, 'curvature_rate_limited':2})
    self.assertEqual(events[0]['transitions'], {'active':2, 'human_turn':1})
    self.assertIsNone(events[0]['latest']['speed_mps'])
    json.dumps(events, allow_nan=False)

  def test_logging_failure_is_isolated(self):
    calls = []
    def fail(*args, **kwargs):
      calls.append(1)
      raise RuntimeError('socket unavailable')
    observer = FordSteeringDiagnostics(fail, 'FORD_EDGE_MK2')
    for i in range(100):
      observer.observe(dict(sequence=i+1, command_time_nanos=i*50_000_000, reason='active', output_curvature=.001))
    self.assertTrue(observer.disabled)
    self.assertEqual(len(calls), 1)
