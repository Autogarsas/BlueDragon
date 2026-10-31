"""Read the installed setting, live CarParams and Panda state; never send CAN."""
from pathlib import Path
import time

from cereal import car, messaging


def main():
  root = Path("/data/params/d")
  key = root / "dp_ford_stock_lon"
  print("Saved Ford factory ACC:", key.exists() and key.read_bytes().strip() == b"1")
  current = root / "CarParams"
  if not current.exists():
    current = root / "CarParamsPersistent"
    print("CarParams is cached; turn ignition on while parked for a live check.")
  with car.CarParams.from_bytes(current.read_bytes()) as cp:
    print("Profile:", cp.brand, cp.carFingerprint)
    print("Openpilot speed control:", cp.openpilotLongitudinalControl)
    print("Paired safety modes:", [(str(s.safetyModel), s.safetyParam) for s in cp.safetyConfigs])
    paired = (cp.brand == "ford" and not cp.openpilotLongitudinalControl and
              any(str(s.safetyModel) == "ford" and s.safetyParam & 8 for s in cp.safetyConfigs))
    print("Factory ACC host/safety configuration paired:", paired)
  sm = messaging.SubMaster(["carState", "pandaStates", "carControl"])
  end = time.monotonic() + 3
  while time.monotonic() < end:
    sm.update(200)
  print("Services alive:", sm.alive)
  if sm.alive["carState"]:
    cs = sm["carState"]
    print("CAN valid:", cs.canValid, "ACC fault:", cs.accFaulted)
  if sm.alive["carControl"]:
    print("Openpilot longitudinal active:", sm["carControl"].longActive)
  for p in sm["pandaStates"]:
    print("Panda:", p.pandaType, "Safety:", p.safetyModel, "Parameter:", p.safetyParam,
          "RX checks invalid:", p.safetyRxChecksInvalid)
  print("Safety bits alone do not verify firmware. Check the latest Panda startup signature matches the packaged image.")


if __name__ == "__main__":
  main()
