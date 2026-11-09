"""Read the K2GC-AH/CD internal steering angle from existing CAN/log data.

Original 63462..6347C and 67124..67264 encode the internal angle as a
relative angle minus an offset in bytes 6..7 of the alternate 0x85 frame.
This tool sends no CAN, opens no Panda transport and changes no parameters.
It is not a live steering controller or a claim that this fixes actuation.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import statistics

MAPPING = {"strategy": "K2GC-14D003-AH", "calibration": "K2GC-14D007-CD"}
MAX_PAIR_AGE_NS = 100_000_000


def decode_internal_angle(data):
    """Decode only the traced alternate format, with its original checksum."""
    data = bytes(data)
    if len(data) != 8:
        return None
    # The traced encoder uses the high bit of byte 2 for power request; the
    # other bits in bytes 2/3 and the low nibble of byte 5 are unused here.
    if data[2] & 0x7f or data[3] or data[5] & 0x0f:
        return None
    relative_raw = int.from_bytes(data[:2], "big")
    offset_raw = int.from_bytes(data[6:], "big")
    if relative_raw >= 0xfffe or offset_raw >= 0xfffe:
        return None
    counter = data[5] >> 4
    checksum = (255 - (data[0] + data[1] + data[6] + data[7] + counter)) & 255
    if checksum != data[4]:
        return None
    return {
        "internal_angle_deg": (relative_raw - offset_raw) / 10.,
        "relative_angle_deg": (relative_raw - 32000) / 10.,
        "relative_minus_internal_deg": (offset_raw - 32000) / 10.,
        "counter": counter,
        "power_request": bool(data[2] & 0x80),
    }


def decode_angle_did(data):
    data = bytes(data)
    if len(data) != 8 or data[:4] != bytes.fromhex("05623302"):
        return None
    raw = int.from_bytes(data[4:6], "big")
    return None if raw == 0xffff else (raw - 7800) / 10.


def error_stats(values):
    if not values:
        return {"count": 0}
    ordered = sorted(abs(v) for v in values)
    return {
        "count": len(values),
        "median_signed_deg": statistics.median(values),
        "min_signed_deg": min(values),
        "max_signed_deg": max(values),
        "p95_absolute_deg": ordered[min(len(ordered)-1, int(.95*len(ordered)))],
        "max_absolute_deg": max(ordered),
    }


def compare_frames(frames):
    """Compare native received frames only; never infer ECU receipt from echoes."""
    recent = deque()
    previous_counter = None
    previous_time = None
    errors, held_errors, examples = [], [], []
    counts = {"angle_frames": 0, "usable_angle_frames": 0, "did3302": 0,
              "unpaired_did3302": 0, "counter_duplicates": 0, "counter_skips": 0}
    for ns, bus, address, data, valid in frames:
        if bus != 0:
            continue
        if not valid:
            if address == 0x85:
                recent.clear()
                previous_counter = None
            continue
        if previous_time is not None and ns < previous_time:
            recent.clear()
            previous_counter = None
        previous_time = ns
        if address == 0x85:
            counts["angle_frames"] += 1
            angle = decode_internal_angle(data)
            if angle is None:
                recent.clear()
                previous_counter = None
                continue
            counts["usable_angle_frames"] += 1
            counter = angle["counter"]
            if previous_counter is not None:
                if counter == previous_counter:
                    counts["counter_duplicates"] += 1
                elif counter != (previous_counter + 1) % 16:
                    counts["counter_skips"] += 1
            previous_counter = counter
            recent.append((ns, angle["internal_angle_deg"]))
            while recent and ns - recent[0][0] > MAX_PAIR_AGE_NS:
                recent.popleft()
        elif address == 0x738:
            did = decode_angle_did(data)
            if did is None:
                continue
            counts["did3302"] += 1
            if not recent or not 0 <= ns - recent[-1][0] <= MAX_PAIR_AGE_NS:
                counts["unpaired_did3302"] += 1
                continue
            broadcast_ns, broadcast = recent[-1]
            delta = broadcast - did
            errors.append(delta)
            held = (len(recent) >= 3 and recent[-1][0] - recent[0][0] >= 50_000_000
                    and max(a for _, a in recent)-min(a for _, a in recent) <= .100001)
            if held:
                held_errors.append(delta)
            row = {"boot_seconds": ns/1e9, "did3302_deg": did,
                   "broadcast_internal_deg": broadcast,
                   "broadcast_minus_did_deg": delta,
                   "pair_age_ms": (ns-broadcast_ns)/1e6, "held_window": held}
            if len(examples) < 6:
                examples.append(row)
    return {"counts": counts, "all_pairs": error_stats(errors),
            "held_pairs": error_stats(held_errors), "first_examples": examples}


def frames_from_log(path):
    from openpilot.tools.lib.logreader import LogReader
    for event in LogReader(str(path), only_union_types=True):
        if event.which() != "can":
            continue
        for frame in event.can:
            if frame.src == 0 and frame.address in (0x85, 0x738):
                yield (int(event.logMonoTime), int(frame.src), int(frame.address),
                       bytes(frame.dat), bool(event.valid))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recent-logs", type=int, default=12,
                        help="Number of most recently modified local rlogs to inspect (1..120)")
    parser.add_argument("--log-root", type=Path, default=Path("/data/media/0/realdata"))
    args = parser.parse_args()
    if not 1 <= args.recent_logs <= 120:
        parser.error("--recent-logs must be 1..120")
    files = sorted((p for p in args.log_root.glob("*/rlog.*")
                    if p.name in ("rlog.zst", "rlog.bz2") and p.is_file()),
                   key=lambda p: p.stat().st_mtime, reverse=True)[:args.recent_logs]
    report = {"mapping_assumption": MAPPING, "source": "existing_local_recordings",
              "scanned_files": len(files), "native_angle_frames": 0, "files_without_did3302": 0,
              "sends_can": False, "changes_parameters": False,
              "physical_steering_proven": False, "files": []}
    # Reset pairing at each file boundary; init/header timestamps never pair data.
    for path in reversed(files):
        try:
            result = compare_frames(frames_from_log(path))
            result["file"] = str(path)
        except Exception as exc:
            result = {"file": str(path), "error": f"{type(exc).__name__}: {exc}"}
        if "counts" in result:
            report["native_angle_frames"] += result["counts"]["usable_angle_frames"]
            if result["counts"]["did3302"] == 0:
                report["files_without_did3302"] += 1
                continue
        report["files"].append(result)
    report["files_found"] = len(files)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if files else 1


if __name__ == "__main__":
    raise SystemExit(main())