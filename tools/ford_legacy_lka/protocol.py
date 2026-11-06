"""Inspect Ford LKA frames without deciding which layout a vehicle supports.

These two DBC interpretations are hypotheses, not interchangeable units or proof
that a PSCM accepted a request. No transport, Params writes or flashing APIs live
here. The preview encoder is for offline fixtures only.
"""
from collections import Counter, defaultdict


VEHICLES = {
  "fusion_2019": {"name": "Ford Fusion Hybrid 2019", "reported_setup": "LCA retrofit; Edge MK2 profile"},
  "mkz_2016": {"name": "Lincoln MKZ 2016", "reported_setup": "Original LKA; rack calibration unconfirmed"},
}

# Include both candidate steering/speed sources. An address alone cannot identify
# the layout: 0x76 has a different meaning in the two DBCs, for example.
WATCH_IDS = frozenset((0x76, 0x7E, 0x82, 0x83, 0x85, 0x8A, 0x91, 0x165, 0x202, 0x204,
                       0x217, 0x246, 0x3A8, 0x3CA, 0x3CC, 0x3D3, 0x3D6, 0x415))
LAYOUTS = ("fusion_2018", "ford_base")


def decode_lka(data: bytes) -> dict:
  if len(data) != 8:
    raise ValueError("LKA 0x3CA must contain exactly 8 bytes")
  reference = ((data[2] & 0xF) << 8) | data[3]
  curvature = (data[1] << 4) | (data[2] >> 4)
  return {
    "reference_raw": reference,
    "curvature_raw": curvature,
    "fusion_2018_view": {
      "action": data[0] >> 4,
      "alert": data[0] & 0xF,
      "steer_angle_req_dbc_deg": reference * 0.04297 - 88.00445,
      "curvature_dbc_per_m": curvature * 5e-6 - 0.01,
    },
    "ford_base_view": {
      "request": data[0] >> 5,
      "ldw_request": (data[0] >> 2) & 7,
      "ldw_intensity": data[0] & 3,
      "reference_dbc_mrad": reference * 0.05 - 102.4,
      "curvature_dbc_per_m": curvature * 5e-6 - 0.01024,
      "driver_override_request": (data[4] >> 5) & 3,
      "quick_ramp": bool(data[4] & 0x80),
    },
  }


def decode_status(data: bytes) -> dict:
  if len(data) != 8:
    raise ValueError("LKA status 0x3CC must contain exactly 8 bytes")
  available = (data[0] >> 4) & 3
  return {"availability": available, "available_candidate": available in (2, 3),
          "denied": bool(data[0] & 0x40), "hands_off": bool(data[0] & 0x80)}


def preview_lka(layout: str, reference_raw: int = 2048, curvature_raw: int | None = None,
                direction: str = "off") -> bytes:
  """Make a small offline fixture, never a live steering command.

  Use raw integers deliberately: converting a requested steering-wheel angle
  requires the actual rack's units, sign, steering ratio and firmware behavior.
  The tight raw window limits fixtures; it is NOT a Panda safety implementation.
  """
  if layout not in LAYOUTS or direction not in ("off", "left", "right"):
    raise ValueError("Explicit supported layout and direction required")
  if type(reference_raw) is not int or not 2032 <= reference_raw <= 2064:
    raise ValueError("Offline reference fixture must be within 16 raw counts of centre")
  centre = 2000 if layout == "fusion_2018" else 2048
  curvature_raw = centre if curvature_raw is None else curvature_raw
  if type(curvature_raw) is not int or not centre - 16 <= curvature_raw <= centre + 16:
    raise ValueError("Offline curvature fixture must be within 16 raw counts of centre")
  if layout == "fusion_2018":
    action = {"off": 15, "left": 4, "right": 8}[direction]
    first = (action << 4) | 15  # historical DBC: no alert
  else:
    request = {"off": 0, "left": 2, "right": 4}[direction]
    first = request << 5  # ordinary intervention, no LDW/override/quick-ramp
  return bytes((first, curvature_raw >> 4,
                ((curvature_raw & 15) << 4) | (reference_raw >> 8), reference_raw & 255, 0, 0, 0, 0))


class Session:
  """Bounded raw evidence, with streams and buses kept separate."""
  def __init__(self, vehicle: str, label: str, max_frames: int = 100000):
    if vehicle not in VEHICLES or label not in ("baseline", "before", "after"):
      raise ValueError("Explicit vehicle and baseline/before/after label required")
    if not 1 <= max_frames <= 200000:
      raise ValueError("Invalid capture frame limit")
    self.vehicle, self.label, self.max_frames = vehicle, label, max_frames
    self.frames, self.states, self.errors = [], [], []
    self.states_truncated = False
    self.counts = Counter()
    self.truncated = False
    self._last_selfdrive_snapshot = None
    self._last_selfdrive_time = None

  def add_frame(self, stream: str, bus: int, address: int, data: bytes, mono_ns: int, valid: bool = True):
    if (stream not in ("can", "sendcan") or type(bus) is not int or bus < 0 or
        type(mono_ns) is not int or mono_ns < 0):
      raise ValueError("Invalid frame envelope")
    if address not in WATCH_IDS:
      return
    data = bytes(data)
    self.counts[(stream, bus, address)] += 1
    if len(self.frames) >= self.max_frames:
      self.truncated = True
      return
    row = {"stream": stream, "bus": bus, "address": hex(address), "data": data.hex(),
           "mono_ns": mono_ns, "event_valid": bool(valid)}
    try:
      if address == 0x3CA:
        row["dbc_views"] = decode_lka(data)
      elif address == 0x3CC:
        row["status_view"] = decode_status(data)
    except ValueError as ex:
      row["decode_error"] = str(ex)
    self.frames.append(row)

  def add_state(self, state: dict):
    # Retain every engagement/alert transition plus a one-second heartbeat.
    # Saving unchanged selfdriveState at 100 Hz would exhaust the minute budget.
    if state.get("kind") == "selfdriveState":
      snapshot = {k: v for k, v in state.items() if k != "mono_ns"}
      t = state["mono_ns"]
      if (snapshot == self._last_selfdrive_snapshot and self._last_selfdrive_time is not None
          and 0 <= t - self._last_selfdrive_time < 1000000000):
        return
      self._last_selfdrive_snapshot, self._last_selfdrive_time = snapshot, t
    # Retain a full minute of carState/carControl at 100Hz plus Panda/command
    # diagnostics. Longer captures explicitly report state truncation.
    if len(self.states) < 20000:
      self.states.append(state)
    else:
      self.states_truncated = True

  def report(self, metadata: dict | None = None) -> dict:
    # These are host-controller observations, kept separate from EPS status.
    # An already-latched capture cannot reveal the missing original trigger.
    latches = []
    current_latch = None
    for state in sorted(self.states, key=lambda r: r["mono_ns"]):
      if state.get("kind") != "legacyLkaCommand":
        continue
      command = state["command"]
      if command.get("latched") and current_latch is None:
        reason = command.get("reason")
        current_latch = {"first_observed_mono_ns": state["mono_ns"],
                         "observed_trigger_reason": reason if reason != "latched_after_interruption" else None,
                         "cleared_mono_ns": None}
        latches.append(current_latch)
      elif command.get("latched") is False and current_latch is not None:
        current_latch["cleared_mono_ns"] = state["mono_ns"]
        current_latch = None
    groups = defaultdict(list)
    for row in self.frames:
      if row["stream"] == "can" and "status_view" in row:
        groups[row["bus"]].append(row)
    status = []
    for bus, rows in sorted(groups.items()):
      rows = sorted(rows, key=lambda r: r["mono_ns"])
      runs, transitions, gaps = [], [], 0
      start = previous = last_state = None
      for row in rows:
        t = row["mono_ns"] / 1e9
        state = row["status_view"]
        usable = row["event_valid"] and state["available_candidate"] and not state["denied"]
        gap = previous is not None and t - previous > 0.2
        if gap:
          gaps += 1
        if start is not None and (not usable or gap):
          runs.append(max(0, previous - start))
          start = None
        if usable and start is None:
          start = t
        signature = (state["availability"], state["denied"], row["event_valid"])
        if signature != last_state:
          transitions.append({"mono_ns": row["mono_ns"], "event_valid": row["event_valid"], **state})
          last_state = signature
        previous = t
      if start is not None:
        runs.append(max(0, previous - start))
      status.append({"bus": bus, "samples": len(rows), "gaps_over_200ms": gaps,
                     "longest_observed_available_run_s": round(max(runs, default=0), 3),
                     "transitions": transitions})
    return {
      "format": "bluedragon-ford-lka-evidence-v1", "vehicle_label": self.vehicle,
      "vehicle_label_is_not_fingerprinting": True, "reported_setup": VEHICLES[self.vehicle], "label": self.label,
      "capture_sends_can": False, "live_lka_mode_implemented": None,
      "collector_is_not_host_feature_detection": True,
      "layout_confirmed": False, "eps_acceptance_proven": False,
      "interpretation": "DBC views are hypotheses. Received/host TX frames and availability do not prove EPS execution or lockout removal.",
      "metadata": metadata or {}, "truncated": self.truncated, "errors": self.errors,
      "states_truncated": self.states_truncated,
      "counts": [{"stream": s, "bus": b, "address": hex(a), "count": n}
                 for (s, b, a), n in sorted(self.counts.items())],
      "status_observations": status, "states": self.states, "frames": self.frames,
      "host_latch_observations": latches,
    }

