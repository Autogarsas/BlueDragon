#!/usr/bin/env python3
"""Rebuild only params_pyx for the prebuilt dragonpilot Ford port.

Run while parked, with ignition off. No OS, Panda firmware, vehicle parameters,
or source files are changed. --install replaces the library after isolated tests.
"""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import sysconfig
import tempfile
import urllib.request

SOURCE_REF = "c0ab3550eca2e9daf197c46b7e4b24aa9637cf2e"
SOURCES = {
  "params.cc": "2947645ca30b57d23feeace0e88bead5709c1681dc21359fb1e5a225ca63b187",
  "util.cc": "4479ecf72465e8f453d8af78447f7715f02d9397c58a49048f2bbc87a96d6b8a",
  "swaglog.cc": "b61e45b8ad799606ba2a78e0b5ee9303a665bad99a67d50f3b7134792bf7ba7c",
}

VERIFY = r'''
import importlib.util, json, logging, os, pathlib, sys, tempfile, types
root, binary, stub_logger = sys.argv[1:]
sys.path.insert(0, root)
if stub_logger == "1":
  logger = types.ModuleType("openpilot.common.swaglog")
  logger.cloudlog = logging.getLogger("params-test")
  sys.modules[logger.__name__] = logger
spec = importlib.util.spec_from_file_location("openpilot.common.params_pyx", binary)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
with tempfile.TemporaryDirectory(prefix="params-verify-") as storage:
  p = module.Params(storage)
  header = pathlib.Path(root, "common/params_keys.h").read_text()
  import re
  declared = set(re.findall(r'\{"([^"\n]+)",\s*\{', header))
  actual = {k.decode() if isinstance(k, bytes) else k for k in p.all_keys()}
  assert declared == actual, (declared - actual, actual - declared)
  ford = sorted(k for k in actual if k.startswith("dp_ford_bp_"))
  assert len(ford) == 11, ford
  assert not p.get_bool("dp_ford_bp_disable")
  assert p.get_default_value("dp_ford_bp_disable") is False
  assert p.get_default_value("dp_ford_bp_enable_human_turn_detection") is True
  assert p.get_default_value("dp_ford_bp_lane_change_factor_high") == 0.85
  assert p.get_default_value("dp_ford_bp_pc_blend_ratio_low_c_u_i") == 0.4
  for key in ford:
    p.check_key(key)
    value = p.get_default_value(key)
    assert value is not None, key
    p.put(key, value, block=True)
    assert p.get(key) == value, key
  p.put_bool("dp_ford_bp_disable", True, block=True)
  assert p.get_bool("dp_ford_bp_disable")
  p.put("dp_ford_bp_lane_change_factor_high", 0.75, block=True)
  assert p.get("dp_ford_bp_lane_change_factor_high") == 0.75
  p.put_bool("IsMetric", True, block=True)
  assert p.get_bool("IsMetric")
  p.put("CarParams", b"isolated-test-only", block=True)
  assert p.get("CarParams") == b"isolated-test-only"
  p.clear_all(module.ParamKeyFlag.CLEAR_ON_MANAGER_START)
  assert p.get("CarParams") is None
  assert p.get_bool("dp_ford_bp_disable")
  try:
    p.get_bool("DefinitelyNotAValidParameter")
  except module.UnknownKeyName:
    pass
  else:
    raise AssertionError("unknown parameters were accepted")
  del p
print(json.dumps({"verified_keys": len(actual), "ford_keys": len(ford), "checks": "passed"}))
'''


def require_parked(root):
  if platform.machine() != "aarch64" or not Path("/TICI").exists():
    raise RuntimeError("Installation is only supported on an AGNOS comma device")
  if root != Path("/data/openpilot").resolve():
    raise RuntimeError("Unexpected install directory")
  # Read directly: the currently installed Params library is the broken one.
  if Path("/data/params/d/IsOffroad").read_bytes().strip() != b"1":
    raise RuntimeError("Turn the ignition OFF and wait for the offroad screen first")


def run(argv, **kwargs):
  subprocess.run([str(x) for x in argv], check=True, **kwargs)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--root", type=Path, default=Path("/data/openpilot"))
  parser.add_argument("--install", action="store_true")
  args = parser.parse_args()
  root = args.root.resolve()
  if (root / "git_src_commit").read_text().strip() != SOURCE_REF:
    raise RuntimeError("This repair is for the original dragonpilot 0.11.1 build only")
  if sys.version_info[:2] != (3, 12):
    raise RuntimeError("Use the device's Python 3.12 environment")
  for rel in ("common/params_pyx.cpp", "common/params_pyx.so", "common/params_keys.h"):
    if not (root / rel).is_file():
      raise RuntimeError(f"Missing required file: {rel}")
  if "dp_ford_bp_disable" not in (root / "common/params_keys.h").read_text():
    raise RuntimeError("The Ford port's parameter declarations are missing")
  if args.install:
    require_parked(root)

  compiler = shutil.which("clang++") or shutil.which("g++")
  if not compiler:
    raise RuntimeError("No C++ compiler found")
  deps = [importlib.import_module(n) for n in ("capnproto", "json11", "zeromq")]
  capnp, json11, zmq = deps
  for package, library in ((json11, "libjson11.a"), (zmq, "libzmq.a")):
    if not Path(package.LIB_DIR, library).is_file():
      raise RuntimeError(f"Missing existing build dependency: {library}")
  python_h = Path(sysconfig.get_paths()["include"], "Python.h")
  if not python_h.is_file():
    raise RuntimeError(f"Missing Python development headers: {python_h}")

  with tempfile.TemporaryDirectory(prefix="ford-params-build-") as temporary:
    work = Path(temporary)
    (work / "common").mkdir()
    generated = work / "cereal/gen/cpp"
    generated.mkdir(parents=True)
    for filename, digest in SOURCES.items():
      print(f"Fetching matching source: {filename}", flush=True)
      url = f"https://raw.githubusercontent.com/dragonpilot/dragonpilot/{SOURCE_REF}/common/{filename}"
      with urllib.request.urlopen(url, timeout=30) as response:
        data = response.read()
      if hashlib.sha256(data).hexdigest() != digest:
        raise RuntimeError(f"Source checksum mismatch: {filename}")
      (work / "common" / filename).write_bytes(data)

    print("Generating schema headers in the temporary build directory", flush=True)
    environment = os.environ.copy()
    environment["PATH"] = str(capnp.BIN_DIR) + os.pathsep + environment.get("PATH", "")
    run([Path(capnp.BIN_DIR, "capnp"), "compile", f"--src-prefix={root / 'cereal'}",
         f"-I{capnp.INCLUDE_DIR}", f"-o{Path(capnp.BIN_DIR, 'capnpc-c++')}:{generated}",
         *[root / "cereal" / f"{schema}.capnp" for schema in ("log", "car", "deprecated", "custom")]], env=environment)
    includes = [work, root, python_h.parent, *[Path(p.INCLUDE_DIR) for p in deps]]
    flags = ["-std=c++17", "-D__TICI__", "-fPIC", "-O2", "-pthread"]
    include_flags = [f"-I{directory}" for directory in includes]
    objects = []
    for source in [work / "common" / name for name in SOURCES] + [root / "common/params_pyx.cpp"]:
      target = work / f"{source.stem}.o"
      print(f"Compiling {source.name}", flush=True)
      run([compiler, *flags, *include_flags, "-c", source, "-o", target])
      objects.append(target)
    candidate = work / "params_pyx.so"
    print("Linking settings library", flush=True)
    run([compiler, "-shared", "-pthread", *objects, Path(json11.LIB_DIR, "libjson11.a"),
         Path(zmq.LIB_DIR, "libzmq.a"), "-o", candidate])
    print("Testing every declared key and isolated parameter storage", flush=True)
    run([sys.executable, "-c", VERIFY, root, candidate, "0" if args.install else "1"], cwd=root)
    if not args.install:
      print("BUILD AND TEST PASSED. No installed library was changed.", flush=True)
      return

    require_parked(root)
    installed = root / "common/params_pyx.so"
    old_digest = hashlib.sha256(installed.read_bytes()).hexdigest()
    backups = Path("/data/ford-bluepilot-backups")
    backups.mkdir(exist_ok=True)
    backup = backups / f"params_pyx-{old_digest}.so"
    if not backup.exists():
      shutil.copy2(installed, backup)
    if hashlib.sha256(backup.read_bytes()).hexdigest() != old_digest:
      raise RuntimeError("Backup verification failed")
    fd, staged = tempfile.mkstemp(prefix=".params-repaired-", suffix=".so", dir=installed.parent)
    try:
      with os.fdopen(fd, "wb") as stream:
        stream.write(candidate.read_bytes())
        stream.flush()
        os.fsync(stream.fileno())
      os.chmod(staged, installed.stat().st_mode & 0o777)
      require_parked(root)
      os.replace(staged, installed)
    finally:
      if os.path.exists(staged):
        os.unlink(staged)
    print(f"REPAIR INSTALLED. Original library saved to {backup}", flush=True)
    print("Reboot while parked to load the repaired library. No reboot was triggered.", flush=True)
    print("This fixes parameter registration only; Ford Panda firmware still needs a separate build audit.", flush=True)


if __name__ == "__main__":
  try:
    main()
  except Exception as exc:
    print(f"STOPPED: {exc}", file=sys.stderr, flush=True)
    sys.exit(1)
