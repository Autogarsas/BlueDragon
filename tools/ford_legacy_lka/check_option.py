"""Read the research selector and CarParams without writing or sending CAN."""
from pathlib import Path


def main():
  from cereal import car
  from opendbc.car.ford.values import FordFlags

  root = Path("/data/params/d")
  setting = root / "dp_ford_legacy_lka"
  selected = setting.exists() and setting.read_bytes().strip() == b"1"
  print("Saved legacy LKA research option:", selected)
  print("Active legacy LKA controller: NOT IMPLEMENTED; research mode cannot steer")
  path = root / "CarParams"
  if not path.exists():
    path = root / "CarParamsPersistent"
    print("Using cached CarParams; this is not a live firmware check")
  if not path.exists():
    print("No CarParams available")
    return
  with car.CarParams.from_bytes(path.read_bytes()) as cp:
    active_selection = cp.brand == "ford" and bool(cp.flags & FordFlags.LEGACY_LKA)
    print("Profile:", cp.brand, cp.carFingerprint)
    print("Startup selected legacy LKA research:", active_selection)
    print("Dashcam only:", cp.dashcamOnly, "Passive:", cp.passive)
    print("Openpilot speed control:", cp.openpilotLongitudinalControl)
    print("Safety:", [(str(s.safetyModel), s.safetyParam) for s in cp.safetyConfigs])
    if selected != active_selection and cp.brand == "ford":
      print("Saved selection differs from startup selection; restart while parked to apply")


if __name__ == "__main__":
  main()
