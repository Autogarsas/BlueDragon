"""Bounded observational Ford steering summaries; never read by control logic."""
from collections import Counter
import math

EVENT = "bluedragon_ford_steering"


class FordSteeringDiagnostics:
  def __init__(self, emit, fingerprint):
    self.emit = emit
    self.fingerprint = str(fingerprint)
    self.last_sequence = 0
    self.window_start = None
    self.last_reason = None
    self.counts = Counter()
    self.transitions = Counter()
    self.samples = 0
    self.max_output = 0.0
    self.disabled = False

  def observe(self, snapshot):
    # Existing nonblocking cloudlog socket; no direct file writer or control feedback.
    if self.disabled:
      return
    try:
      seq = snapshot.get("sequence", 0)
      if seq <= self.last_sequence or "command_time_nanos" not in snapshot:
        return
      self.last_sequence = seq
      now = snapshot["command_time_nanos"]
      if self.window_start is None or now < self.window_start:
        self.window_start = now
      self.samples += 1
      reason = snapshot["reason"]
      self.counts[reason] += 1
      if reason != self.last_reason:
        self.transitions[reason] += 1
        self.last_reason = reason
      for key, value in snapshot.items():
        if key.endswith("_limited") and value:
          self.counts[key] += 1
      self.max_output = max(self.max_output, abs(snapshot["output_curvature"]))
      if now - self.window_start >= 1_000_000_000:
        latest = {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in snapshot.items()}
        self.emit(EVENT, schema=1, fingerprint=self.fingerprint, start_nanos=self.window_start,
                  end_nanos=now, samples=self.samples, counts=dict(self.counts),
                  transitions=dict(self.transitions), max_abs_output_curvature=self.max_output, latest=latest)
        self.window_start = now
        self.samples = 0
        self.max_output = 0.0
        self.counts.clear()
        self.transitions.clear()
    except Exception:
      # Logging failure disables diagnostics, never steering.
      self.disabled = True
