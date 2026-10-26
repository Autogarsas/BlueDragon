"""Read-only verification of the packaged Pacifica ALKA H7 application."""
import hashlib
import json
import struct
from pathlib import Path

from Crypto.PublicKey import RSA

ROOT = Path(__file__).resolve().parents[2]


def sha256(data):
  return hashlib.sha256(data).hexdigest()


def verify(root, manifest):
  firmware = (root / "panda/board/obj/panda_h7.bin.signed").read_bytes()
  if len(firmware) != manifest["firmware_bytes"] or sha256(firmware) != manifest["firmware_sha256"]:
    raise RuntimeError("Packaged H7 firmware size/checksum mismatch")
  if len(firmware) < 136:
    raise RuntimeError("Truncated H7 application")
  payload, signature = firmware[:-128], firmware[-128:]
  if struct.unpack("<I", payload[:4])[0] != len(payload) or payload[-8:] != b"VERS" + struct.pack("<I", 2):
    raise RuntimeError("Unexpected signed H7 application format")
  reset = struct.unpack("<I", payload[4:8])[0]
  if not (0x08020000 <= reset < 0x08020000 + len(payload) and reset & 1):
    raise RuntimeError("Reset vector is not in the H7 application partition")
  key = RSA.import_key((root / "panda/board/certs/debug.pub").read_bytes())
  if key.size_in_bits() != 1024:
    raise RuntimeError("Unexpected Panda certificate size")
  decoded = pow(int.from_bytes(signature, "big"), key.e, key.n).to_bytes(128, "big")
  expected = b"\x00\x01" + b"\xff" * 105 + b"\x00" + hashlib.sha1(payload[4:]).digest()
  if decoded != expected or signature.hex()[:16] != manifest["firmware_signature_prefix"]:
    raise RuntimeError("Panda application signature verification failed")
  if manifest["build_version"].encode() + b"\0" not in payload:
    raise RuntimeError("Expected firmware build version missing")
  for name, expected_hash in manifest["checksums"].items():
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()) or sha256(path.read_bytes()) != expected_hash:
      raise RuntimeError(f"Source/protocol mismatch: {name}")
  return firmware


def main():
  manifest = json.loads((ROOT / "tools/chrysler_alka/h7-build.json").read_text())
  verify(ROOT, manifest)
  print(f'Verified H7 application: {manifest["build_version"]}')
  print(f'Signature prefix: {manifest["firmware_signature_prefix"]}')
  print(f'Matching source/protocol files: {len(manifest["checksums"])}')
  print("Read-only check. This does not prove the connected Panda has flashed this application.")


if __name__ == "__main__":
  main()
