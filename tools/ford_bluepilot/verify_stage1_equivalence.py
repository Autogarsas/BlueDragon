"""Compare packed CAN and actuator outputs against the preserved 1.0 baseline."""
from collections import defaultdict
import math
import subprocess
import sys
import types
from types import SimpleNamespace as NS
from opendbc.car import structs
from opendbc.car.ford.tests.test_bluepilot import params, model
from opendbc.car.ford.values import CAR, DBC
from opendbc.car.ford.carcontroller import CarController

BASE = 'a5aa0011f0a858bf06d9ca0b180fa56bd667bb08'


def baseline(path, name):
  module = types.ModuleType(name)
  sys.modules[name] = module
  source = subprocess.check_output(['git', 'show', f'{BASE}:{path}'], text=True)
  exec(compile(source, path, 'exec'), module.__dict__)
  return module


def main():
  old_lat = baseline('opendbc_repo/opendbc/car/ford/bluepilot_lateral.py', 'baseline_lat')
  old_controller = baseline('opendbc_repo/opendbc/car/ford/carcontroller.py', 'baseline_controller')
  old_controller.BluePilotLateral = old_lat.BluePilotLateral
  frames = 0
  from opendbc.car.ford.bluepilot_settings import SETTINGS
  for car in (CAR.FORD_EDGE_MK2, CAR.FORD_MUSTANG_MACH_E_MK1):
    cp = params(car)
    for custom in (False, True):
      a, b = old_controller.CarController(DBC[car], cp), CarController(DBC[car], cp)
      settings = {k: True for k, v in SETTINGS.items() if custom and v['attr'] == 'enable_lane_positioning'}
      a.bp_lateral.configure(settings)
      b.bp_lateral.configure(settings)
      cc = structs.CarControl(latActive=True)
      cs = NS(out=structs.CarState(vEgoRaw=20, vEgo=20), acc_tja_status_stock_values=defaultdict(int),
              buttons_stock_values=defaultdict(int), lkas_status_stock_values=defaultdict(int))
      for i in range(2500):
        cc.latActive = i % 317 > 5
        speed = (0., 5., 10., 20., 30.)[(i//100) % 5]
        cs.out.vEgoRaw = speed
        cs.out.vEgo = speed
        cs.out.yawRate = math.sin(i/80)*.02
        cs.out.steeringPressed = i % 197 < 20
        cs.out.steeringAngleDeg = 60 if i % 197 < 10 else 5
        cc.actuators.curvature = math.sin(i/30)*.02
        md = model(math.cos(i/90)*.015, max(speed, 1), offset=.25)
        md.meta.laneChangeState = 2 if i % 400 < 50 else 0
        md.meta.laneChangeDirection = 1 if i % 800 < 400 else 2
        if i % 211 == 0:
          md = None
        elif i % 211 == 1:
          md.position.y = []
        elif i % 211 == 2:
          md.orientationRate.z[0] = float('nan')
        a.bp_model = b.bp_model = md
        x, can_a = a.update(cc.as_reader(), cs, i*10_000_000)
        y, can_b = b.update(cc.as_reader(), cs, i*10_000_000)
        assert x.to_dict() == y.to_dict(), (car, i, 'actuators')
        assert can_a == can_b, (car, i, 'CAN')
        frames += 1
  print(f'{frames} frames: packed CAN and actuators identical to 1.0 (CAN/CAN-FD, default/lane positioning)')


if __name__ == '__main__':
  main()
