#!/usr/bin/env python3
"""Configure comma's padded fork-installer URL parameter; never execute it."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def configure(source: bytes, manifest: dict) -> bytes:
  if hashlib.sha256(source).hexdigest() != manifest["source_sha256"]:
    raise ValueError("Upstream installer checksum differs from the reviewed build")
  if source[:5] != b"\x7fELF\x02" or struct.unpack_from("<H", source, 18)[0] != 183:
    raise ValueError("Expected an aarch64 ELF64 installer")
  start = manifest["parameter_buffer_offset"]
  size = manifest["parameter_buffer_bytes"]
  original = source[start:start + size]
  old = b"https://github.com/Autogarsas/openpilot.git?"
  new = manifest["repository"].encode() + b"?"
  if not original.startswith(old) or original[len(old):].strip(b" ") or len(new) > size:
    raise ValueError("Unexpected padded repository parameter")
  if source[start + size] != 0 or source.count(manifest["branch"].encode() + b"?") != 1:
    raise ValueError("Unexpected installer branch or parameter boundary")
  output = source[:start] + new.ljust(size, b" ") + source[start + size:]
  if len(output) != len(source) or hashlib.sha256(output).hexdigest() != manifest["output_sha256"]:
    raise ValueError("Configured installer differs from the reviewed output")
  return output


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path, help="Downloaded upstream installer; see installer/build.json")
  parser.add_argument("output", type=Path)
  args = parser.parse_args()
  if args.source.resolve() == args.output.resolve() or args.output.exists():
    raise ValueError("Choose a new output file; existing files are never overwritten")
  root = Path(__file__).resolve().parents[2]
  manifest = json.loads((root / "installer/build.json").read_text())
  output = configure(args.source.read_bytes(), manifest)
  with args.output.open("xb") as stream:
    stream.write(output)
  args.output.chmod(0o755)
  print(f"Installer prepared: {manifest['repository']} @ {manifest['branch']}")


if __name__ == "__main__":
  main()
