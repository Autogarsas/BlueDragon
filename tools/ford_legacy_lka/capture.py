"""Passively capture legacy Ford LKA evidence on a running BlueDragon device.

May also read local rlog/qlog files. No PubMaster, CAN transmit, Params write,
diagnostic request, direct Panda access or firmware flashing is performed.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import time

if __package__:
  from .protocol import Session, VEHICLES
else:
  from protocol import Session, VEHICLES


def metadata(root: Path) -> dict:
  result = {"captured_utc": datetime.now(timezone.utc).isoformat(),
            "identification_is_not_firmware_flash_verification": True}
  for name, args in (("git_commit", ["rev-parse", "HEAD"]), ("git_branch", ["branch", "--show-current"])):
    try:
      r = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, timeout=3)
      result[name] = r.stdout.strip() if r.returncode == 0 else "unavailable"
    except (OSError, subprocess.TimeoutExpired):
      result[name] = "unavailable"
  try:
    from cereal import car
    params = Path("/data/params/d")
    path = params / "CarParams"
    cached = not path.is_file()
    if cached:
      path = params / "CarParamsPersistent"
    with car.CarParams.from_bytes(path.read_bytes()) as cp:
      result["car_params"] = {"source": path.name, "cached": cached, "brand": cp.brand,
                              "profile": cp.carFingerprint, "flags": cp.flags,
                              "openpilot_longitudinal": cp.openpilotLongitudinalControl,
                              "safety": [{"model": str(s.safetyModel), "param": s.safetyParam} for s in cp.safetyConfigs],
                              "ecu_identification": [{"ecu": str(f.ecu), "address": hex(f.address),
                                "sub_address": f.subAddress, "version_hex": bytes(f.fwVersion).hex()} for f in cp.carFw]}
  except Exception as ex:
    result["car_params_error"] = f"{type(ex).__name__}: {ex}"
  return result


def add_event(session: Session, event):
  kind = event.which()
  if kind in ("can", "sendcan"):
    for f in getattr(event, kind):
      session.add_frame(kind, int(f.src), int(f.address), bytes(f.dat), int(event.logMonoTime), bool(event.valid))
  elif kind == "carState":
    c = event.carState
    session.add_state({"kind": kind, "mono_ns": int(event.logMonoTime), "valid": bool(event.valid),
                       "speed_mps": finite(c.vEgo), "steering_angle_deg": finite(c.steeringAngleDeg),
                       "driver_torque": finite(c.steeringTorque), "driver_steering": c.steeringPressed,
                       "can_valid": c.canValid, "temporary_fault": c.steerFaultTemporary,
                       "permanent_fault": c.steerFaultPermanent})
  elif kind == "carControl":
    c = event.carControl
    session.add_state({"kind": kind, "mono_ns": int(event.logMonoTime), "valid": bool(event.valid),
                       "enabled": c.enabled, "lat_active": c.latActive,
                       "desired_curvature": finite(c.actuators.curvature)})
  elif kind == "logMessage":
    try:
      record = json.loads(event.logMessage)
      data = record.get("msg", {})
      if isinstance(data, dict) and data.get("event") == "bluedragon_ford_legacy_lka":
        session.add_state({"kind": "legacyLkaCommand", "mono_ns": int(event.logMonoTime),
                           "valid": bool(event.valid), "command": data})
    except (ValueError, TypeError, AttributeError):
      pass
  elif kind == "pandaStates":
    session.add_state({"kind": kind, "mono_ns": int(event.logMonoTime), "valid": bool(event.valid),
                       "pandas": [{"ignition": p.ignitionLine or p.ignitionCan,
                         "safety_model": str(p.safetyModel), "safety_param": p.safetyParam,
                         "controls_allowed": p.controlsAllowed, "rx_checks_invalid": p.safetyRxChecksInvalid,
                         "tx_blocked": p.safetyTxBlocked, "faults": [str(f) for f in p.faults]} for p in event.pandaStates]})


def finite(value):
  """Keep damaged/unavailable numeric state readable in strict JSON."""
  value = float(value)
  return value if math.isfinite(value) else None


def read_logs(session: Session, paths: list[str]):
  from openpilot.tools.lib.logreader import LogReader
  for name in paths:
    path = Path(name)
    if not path.is_file():
      raise ValueError(f"Local log not found: {name}")
    try:
      for event in LogReader(str(path), only_union_types=True):
        add_event(session, event)
    except Exception as ex:
      session.errors.append(f"{path.name}: {type(ex).__name__}: {ex}")


def listen(session: Session, seconds: float):
  from cereal import messaging
  streams = ("can", "sendcan", "carState", "carControl", "pandaStates", "logMessage")
  sockets = {name: messaging.sub_sock(name, timeout=0) for name in streams}
  end = time.monotonic() + seconds
  try:
    while time.monotonic() < end:
      received = False
      for sock in sockets.values():
        # Bound draining so an active bus cannot starve other services/the deadline.
        for _ in range(100):
          if time.monotonic() >= end:
            break
          event = messaging.recv_one_or_none(sock)
          if event is None:
            break
          received = True
          add_event(session, event)
      if not received:
        time.sleep(0.005)
  finally:
    # msgq socket implementations release resources through their destructors;
    # not every installed version exposes a public close() method.
    sockets.clear()


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--vehicle", choices=VEHICLES, required=True)
  parser.add_argument("--label", choices=("baseline", "before", "after"), default="baseline")
  parser.add_argument("--seconds", type=float, default=20)
  parser.add_argument("--max-frames", type=int, default=200000,
                      help="Retained frame limit (1-200000); truncation is explicit")
  parser.add_argument("--log", nargs="+", help="Local recorded rlog/qlog paths instead of live listening")
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--root", type=Path, default=Path("/data/openpilot"))
  args = parser.parse_args(argv)
  if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 300:
    parser.error("--seconds must be finite and between 1 and 300")
  if not 1 <= args.max_frames <= 200000:
    parser.error("--max-frames must be between 1 and 200000")
  session = Session(args.vehicle, args.label, max_frames=args.max_frames)
  info = metadata(args.root)
  info["source"] = "recorded_logs" if args.log else "live_subscribers"
  info["requested_seconds"] = None if args.log else args.seconds
  info["max_frames"] = args.max_frames
  # Reserve the output first: never listen and then overwrite an existing report.
  with args.output.open("x") as out:
    try:
      if args.log:
        read_logs(session, args.log)
      else:
        print(f"Listening for {args.seconds:g}s; no CAN is transmitted by this tool.", flush=True)
        listen(session, args.seconds)
    except KeyboardInterrupt:
      session.errors.append("Capture interrupted; retained partial evidence")
    except Exception as ex:
      session.errors.append(f"Capture failed: {type(ex).__name__}: {ex}")
    report = session.report(info)
    json.dump(report, out, indent=2, allow_nan=False)
    out.write("\n")
  print(json.dumps({"output": str(args.output), "vehicle_label": args.vehicle,
                    "frames": len(session.frames), "states": len(session.states),
                    "truncated": session.truncated, "errors": session.errors,
                    "states_truncated": session.states_truncated,
                    "installed_branch": info.get("git_branch"),
                    "loaded_profile": info.get("car_params", {}).get("profile"),
                    "cached_identification": info.get("car_params", {}).get("cached"),
                    "rack_camera_identification": [f for f in info.get("car_params", {}).get("ecu_identification", [])
                                                   if f["ecu"] in ("eps", "fwdCamera")],
                    "frame_counts": report["counts"],
                    "status_observations": report["status_observations"],
                    "layout_confirmed": False, "eps_acceptance_proven": False}, indent=2))
  return 1 if session.errors or not session.frames else 0


if __name__ == "__main__":
  raise SystemExit(main())
