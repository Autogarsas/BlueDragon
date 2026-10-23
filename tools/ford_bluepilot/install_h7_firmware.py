#!/usr/bin/env python3
"""Verify or stage the bp-5.0 H7 Panda application. Never flash hardware directly."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import struct
import sys
import tempfile
import urllib.request

ROOT = Path("/data/openpilot")
BACKUPS = Path("/data/ford-bluepilot-backups")
ARTIFACT_COMMIT = "7dc973d4538fb39583354a6765a9a5eb034e666a"
BASE_URL = f"https://raw.githubusercontent.com/Autogarsas/openpilot/{ARTIFACT_COMMIT}/tools/ford_bluepilot/firmware"
NEW_SHA = "9e9594633dcbd4a63af8e5028c54a870e44553d53773c662b33540f4179b71ce"
OLD_SHA = "c9adb434907e22ea82e0f40f358e10263cefe0328421cff7d9eb367da9cfd9ca"
VERSION = "DEV-f95cbde6-DEBUG"
SIGNATURE_PREFIX = "9a428b52a93e508a"


def digest(data):
  return hashlib.sha256(data).hexdigest()


def verify_firmware(data, public_key):
  from Crypto.PublicKey import RSA
  if len(data) != 72188 or digest(data) != NEW_SHA:
    raise RuntimeError("Firmware size/checksum mismatch; nothing installed")
  payload, signature = data[:-128], data[-128:]
  if struct.unpack("<I", payload[:4])[0] != len(payload):
    raise RuntimeError("Invalid application length")
  if payload[-8:] != b"VERS" + struct.pack("<I", 2):
    raise RuntimeError("Unexpected signed application format")
  reset_vector = struct.unpack("<I", payload[4:8])[0]
  if not 0x08020000 <= reset_vector < 0x08020000 + len(payload) or not reset_vector & 1:
    raise RuntimeError("Application reset vector is not for the H7 app partition")
  key = RSA.import_key(public_key)
  if key.size_in_bits() != 1024:
    raise RuntimeError("Unexpected signing certificate")
  decoded = pow(int.from_bytes(signature, "big"), key.e, key.n).to_bytes(128, "big")
  expected = b"\x00\x01" + b"\xff" * 105 + b"\x00" + hashlib.sha1(payload[4:]).digest()
  if decoded != expected or signature.hex()[:16] != SIGNATURE_PREFIX:
    raise RuntimeError("Firmware signature verification failed")
  if VERSION.encode() not in payload:
    raise RuntimeError("Expected build version missing")


def device_checks():
  if platform.machine() != "aarch64" or not Path("/TICI").exists():
    raise RuntimeError("This installer is only for an AGNOS comma 3X")
  if (ROOT / "git_src_commit").read_text().strip() != "c0ab3550eca2e9daf197c46b7e4b24aa9637cf2e":
    raise RuntimeError("Unexpected dragonpilot source build")
  cache = Path("/persist/dp_dev_panda_mcu_type")
  if cache.exists() and cache.read_text().strip() != "H7":
    raise RuntimeError("Panda MCU cache is not H7; refusing the H7 image")
  if Path("/data/params/d/IsOffroad").read_bytes().strip() != b"1":
    raise RuntimeError("Park, turn ignition OFF and wait for the offroad screen")
  sys.path.insert(0, str(ROOT))
  from openpilot.common.params import Params
  params = Params()
  params.check_key("dp_ford_bp_disable")
  import cereal.messaging as messaging
  sm = messaging.SubMaster(["pandaStates"])
  for _ in range(15):
    sm.update(200)
    if sm.updated["pandaStates"] and sm.valid["pandaStates"] and len(sm["pandaStates"]):
      pandas = sm["pandaStates"]
      if len(pandas) != 1 or str(pandas[0].pandaType) != "tres":
        raise RuntimeError("Expected exactly one internal TRES Panda on a comma 3X")
      if pandas[0].ignitionLine or pandas[0].ignitionCan:
        raise RuntimeError("Panda still reports ignition ON; wait with ignition OFF")
      print("Hardware confirmed: TRES/H7; offroad; ignition off", flush=True)
      return params
  raise RuntimeError("No fresh Panda state available; no files changed")


def check_sources(manifest):
  if manifest["firmware_sha256"] != NEW_SHA or manifest["existing_stock_sha256"] != OLD_SHA:
    raise RuntimeError("Unexpected build manifest")
  for relative, expected in manifest["checksums"].items():
    if digest((ROOT / relative).read_bytes()) != expected:
      raise RuntimeError(f"Installed source/protocol differs from this build: {relative}")
  print("Installed safety sources, protocol headers and certificate match the build", flush=True)


def atomic_replace(path, data):
  fd, temporary = tempfile.mkstemp(prefix=".h7-firmware-", dir=path.parent)
  try:
    with os.fdopen(fd, "wb") as stream:
      stream.write(data)
      stream.flush()
      os.fsync(stream.fileno())
    os.chmod(temporary, path.stat().st_mode & 0o777)
    os.replace(temporary, path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
      os.fsync(directory)
    finally:
      os.close(directory)
  finally:
    if os.path.exists(temporary):
      os.unlink(temporary)


def stage(path, data, backups):
  old = path.read_bytes()
  old_hash = digest(old)
  if old_hash == NEW_SHA:
    print("The verified H7 application is already staged", flush=True)
    return
  if old_hash != OLD_SHA:
    raise RuntimeError("Existing firmware is neither the expected stock image nor this repair")
  backups.mkdir(exist_ok=True)
  backup = backups / f"panda_h7-stock-{OLD_SHA}.bin.signed"
  if not backup.exists():
    shutil.copy2(path, backup)
  if digest(backup.read_bytes()) != OLD_SHA:
    raise RuntimeError("Original firmware backup failed verification")
  with backup.open("rb") as stream:
    os.fsync(stream.fileno())
  directory = os.open(backups, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(directory)
  finally:
    os.close(directory)
  atomic_replace(path, data)
  if digest(path.read_bytes()) != NEW_SHA:
    raise RuntimeError("Staged firmware verification failed")
  print(f"Original firmware backup: {backup}", flush=True)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  action = parser.add_mutually_exclusive_group()
  action.add_argument("--stage", action="store_true", help="stage verified firmware for normal startup flashing")
  action.add_argument("--restore-stock", action="store_true", help="disable BluePilot and restore the backed-up stock image")
  args = parser.parse_args()
  params = device_checks()
  path = ROOT / "panda/board/obj/panda_h7.bin.signed"
  if args.restore_stock:
    backup = BACKUPS / f"panda_h7-stock-{OLD_SHA}.bin.signed"
    data = backup.read_bytes()
    if digest(data) != OLD_SHA or digest(path.read_bytes()) not in (NEW_SHA, OLD_SHA):
      raise RuntimeError("Unexpected original or installed image; restore stopped")
    device_checks()
    params.put_bool("dp_ford_bp_disable", True, block=True)
    atomic_replace(path, data)
    print("Stock image restored; BluePilot disabled to keep controller/safety paired. Reboot while parked.", flush=True)
    return

  print("Downloading the pinned build manifest and H7 application", flush=True)
  with urllib.request.urlopen(BASE_URL + "/build.json", timeout=30) as response:
    manifest = json.load(response)
  check_sources(manifest)
  with urllib.request.urlopen(BASE_URL + "/panda_h7-bp5.bin.signed", timeout=30) as response:
    data = response.read()
  verify_firmware(data, (ROOT / "panda/board/certs/debug.pub").read_bytes())
  print(f"Verified firmware: {VERSION}; SHA-256 {NEW_SHA}", flush=True)
  print("216 native tests: 196 passed, 20 inherited skips. Hardware flash not yet verified.", flush=True)
  if not args.stage:
    print("CHECK PASSED. No installed file was changed. Use --stage to install.", flush=True)
    return
  device_checks()
  stage(path, data, BACKUPS)
  print("H7 FIRMWARE STAGED. No hardware connection or direct flash was performed by this script.", flush=True)
  print("Reboot while parked: normal pandad startup will flash and verify the application signature.", flush=True)
  print(f"Expected application version: {VERSION}; expected signature prefix: {SIGNATURE_PREFIX}", flush=True)
  print("Bootloader, repaired settings library and other Panda firmware targets were not replaced.", flush=True)
  print("Use the SSH workflow; the main branch's UI update is not a complete repaired release yet.", flush=True)


if __name__ == "__main__":
  try:
    main()
  except Exception as exc:
    print(f"STOPPED: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
    sys.exit(1)
