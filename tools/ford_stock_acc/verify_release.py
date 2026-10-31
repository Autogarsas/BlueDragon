"""Verify the matched Ford factory-ACC device package without changing it."""
import hashlib
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.chrysler_alka.verify_release import verify as verify_h7


def main():
  manifest = json.loads((ROOT / "tools/ford_stock_acc/release.json").read_text())
  h7 = json.loads((ROOT / manifest["h7_manifest"]).read_text())
  verify_h7(ROOT, h7)
  params = (ROOT / "common/params_pyx.so").read_bytes()
  if hashlib.sha256(params).hexdigest() != manifest["params_sha256"]:
    raise RuntimeError("Settings-library checksum mismatch")
  if params[:6] != b"\x7fELF\x02\x01" or struct.unpack_from("<H", params, 18)[0] != 183:
    raise RuntimeError("Settings library must be a little-endian 64-bit ARM ELF")
  for name, digest in manifest["source_checksums"].items():
    path = (ROOT / name).resolve()
    if not path.is_relative_to(ROOT) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
      raise RuntimeError(f"Matching source check failed: {name}")
  print(f'Verified H7: {h7["build_version"]}; signature prefix: {h7["firmware_signature_prefix"]}')
  print(f'Verified ARM Python 3.12 settings library: {manifest["registered_keys"]} keys')
  print("Package verified. Connected Panda firmware and vehicle behavior require device checks.")


if __name__ == "__main__":
  main()
