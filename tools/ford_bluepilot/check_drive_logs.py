#!/usr/bin/env python3
"""Read the latest recorded route; print steering diagnostics without changing it."""
import argparse
import bz2
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path("/data/openpilot")
if ROOT.is_dir():
  sys.path.insert(0, str(ROOT))
from cereal import log
import zstandard


def events(path):
  raw = path.read_bytes()
  if path.suffix == ".zst":
    with zstandard.ZstdDecompressor().stream_reader(raw) as reader:
      raw = reader.read()
  elif path.suffix == ".bz2":
    raw = bz2.decompress(raw)
  yield from log.Event.read_multiple_bytes(raw)


def report(paths):
  counts = Counter()
  pandas = {}
  alerts = Counter()
  faults = Counter()
  first_t = last_t = None
  active = False
  speed = 0.0
  desired = 0.0
  first_block = []
  max_angle = max_torque = max_desired = max_output = 0.0
  model_last = None
  model_gaps = model_invalid = 0
  can_commands = Counter()
  cp = None
  problems = []
  for path in paths:
    try:
      for event in events(path):
        kind = event.which()
        t = event.logMonoTime * 1e-9
        first_t = t if first_t is None else min(first_t, t)
        last_t = t if last_t is None else max(last_t, t)
        counts[kind] += 1
        if kind == "carParams":
          c = event.carParams
          cp = {"brand": c.brand, "fingerprint": c.carFingerprint, "flags": c.flags,
                "safety": [{"model": str(s.safetyModel), "param": s.safetyParam} for s in c.safetyConfigs]}
        elif kind == "carControl":
          active = event.carControl.latActive
          desired = event.carControl.actuators.curvature
          counts["lat_active_samples"] += int(active)
          if active:
            max_desired = max(max_desired, abs(desired))
        elif kind == "carOutput":
          if active:
            max_output = max(max_output, abs(event.carOutput.actuatorsOutput.curvature))
        elif kind == "carState":
          c = event.carState
          speed = c.vEgo * 3.6
          max_angle = max(max_angle, abs(c.steeringAngleDeg))
          max_torque = max(max_torque, abs(c.steeringTorque))
          for name, value in (("temporary", c.steerFaultTemporary), ("permanent", c.steerFaultPermanent),
                              ("can_invalid", not c.canValid), ("driver_steering", c.steeringPressed)):
            faults[name] += int(value)
        elif kind == "pandaStates":
          for index, p in enumerate(event.pandaStates):
            entry = pandas.setdefault(index, {"modes": set(), "first_tx": p.safetyTxBlocked, "last_tx": p.safetyTxBlocked,
                                              "blocked_increases": 0, "counter_resets": 0, "max_rx_invalid": 0,
                                              "rx_check_invalid_samples": 0, "controls_allowed_samples": 0, "samples": 0})
            entry["modes"].add((str(p.safetyModel), p.safetyParam))
            delta = p.safetyTxBlocked - entry["last_tx"]
            if delta > 0:
              entry["blocked_increases"] += delta
              if len(first_block) < 8:
                first_block.append({"seconds": round(t - first_t, 1), "blocked": delta,
                                    "lat_active": active, "kmh": round(speed, 1), "desired_curvature": round(desired, 6)})
            elif delta < 0:
              entry["counter_resets"] += 1
            entry["last_tx"] = p.safetyTxBlocked
            entry["max_rx_invalid"] = max(entry["max_rx_invalid"], p.safetyRxInvalid)
            entry["rx_check_invalid_samples"] += int(p.safetyRxChecksInvalid)
            entry["controls_allowed_samples"] += int(p.controlsAllowed)
            entry["samples"] += 1
        elif kind == "selfdriveState":
          s = event.selfdriveState
          if s.alertType:
            alerts[s.alertType] += 1
        elif kind == "modelV2":
          if model_last is not None and active and 0.25 < t - model_last < 10:
            model_gaps += 1
          model_last = t
          model_invalid += int(not event.valid)
        elif kind == "sendcan":
          for c in event.sendcan:
            data = c.dat
            if c.address == 0x3D3 and len(data) == 8:
              can_commands["ford_can_samples"] += 1
              can_commands["ford_can_inactive"] += int(((data[4] >> 2) & 7) == 0)
              auxiliary = (((data[1] & 31) << 8) | data[2]) != 4096 or ((data[3] << 3) | (data[4] >> 5)) != 1000
              can_commands["ford_can_nonzero_auxiliary"] += int(auxiliary)
            elif c.address == 0x3D6 and len(data) == 8:
              can_commands["ford_canfd_samples"] += 1
              can_commands["ford_canfd_inactive"] += int(((data[0] >> 4) & 7) == 0)
    except Exception as exc:
      problems.append(f"{path.parent.name}/{path.name}: {type(exc).__name__}: {str(exc)[:120]}")
  for entry in pandas.values():
    entry["modes"] = sorted(entry["modes"])
  return {"vehicle": cp, "span_seconds": round(last_t - first_t, 1) if first_t is not None else 0,
          "samples": {k: counts[k] for k in ("carControl", "carState", "carOutput", "pandaStates", "modelV2", "lat_active_samples")},
          "pandas": pandas, "blocked_command_examples": first_block, "steering_fault_samples": dict(faults),
          "alerts": dict(alerts.most_common(8)), "max_abs_desired_curvature": round(max_desired, 6),
          "max_abs_host_output_curvature": round(max_output, 6), "max_abs_steering_angle_deg": round(max_angle, 1),
          "max_abs_driver_torque_native_units": round(max_torque, 2), "model_gaps_over_250ms_while_active": model_gaps,
          "invalid_model_samples": model_invalid, "sampled_sendcan": dict(can_commands), "read_errors": problems}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--logs", type=Path, default=Path("/data/media/0/realdata"))
  args = parser.parse_args()
  logs = []
  for directory in args.logs.iterdir():
    if not directory.is_dir() or "--" not in directory.name:
      continue
    for name in ("rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
      path = directory / name
      if path.is_file() and path.stat().st_size:
        logs.append(path)
        break
  if not logs:
    raise RuntimeError("No recorded drive logs found")
  newest = max(logs, key=lambda p: p.stat().st_mtime)
  route = newest.parent.name.rsplit("--", 1)[0]
  route_logs = sorted([p for p in logs if p.parent.name.rsplit("--", 1)[0] == route],
                      key=lambda p: int(p.parent.name.rsplit("--", 1)[1]))
  selected = list(dict.fromkeys([route_logs[0], *route_logs[-6:]]))
  print("READ-ONLY: latest route", route, flush=True)
  print("Segments:", ", ".join(p.parent.name.rsplit("--", 1)[1] + "/" + p.name for p in selected), flush=True)
  result = report(selected)
  print(json.dumps(result, indent=2), flush=True)
  for rel in ("panda/board/obj/panda_h7.bin.signed", "panda_tici/board/obj/panda.bin.signed",
              "panda_tici/board/obj/panda_h7.bin.signed"):
    path = ROOT / rel
    if path.is_file():
      print("BUNDLED_FIRMWARE_SHA256", rel, hashlib.sha256(path.read_bytes()).hexdigest())
  print("Host output does not prove the EPS accepted a command. Safety-param bits do not prove firmware support.")


if __name__ == "__main__":
  main()
