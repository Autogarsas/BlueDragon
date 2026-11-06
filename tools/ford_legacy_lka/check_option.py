"""Read saved/startup selections and live car/Panda status; never transmit CAN."""
from pathlib import Path
import time


def main():
  from cereal import car, messaging
  from opendbc.car.ford.values import FordFlags

  root = Path('/data/params/d')
  setting = root / 'dp_ford_legacy_lka'
  selected = setting.exists() and setting.read_bytes().strip() == b'1'
  print('Saved legacy LKA:', selected)
  path = root / 'CarParams'
  if not path.exists():
    path = root / 'CarParamsPersistent'
    print('Using cached CarParams; ignition on is required for a live check')
  if not path.exists():
    print('No CarParams available');return
  with car.CarParams.from_bytes(path.read_bytes()) as cp:
    legacy = cp.brand == 'ford' and bool(cp.flags & FordFlags.LEGACY_LKA)
    expected = [(str(s.safetyModel), s.safetyParam) for s in cp.safetyConfigs]
    print('Profile:', cp.brand, cp.carFingerprint)
    print('Startup steering mode:', 'Legacy LKA test' if legacy else 'LCA')
    print('Dashcam only:', cp.dashcamOnly, 'Passive:', cp.passive)
    print('Openpilot speed control:', cp.openpilotLongitudinalControl)
    print('Configured safety:', expected)
    if legacy:print('Test limits: 54-108 km/h; reference +/-128 raw; curvature +/-0.0015 m^-1')
    if cp.brand == 'ford' and selected != legacy:print('Restart while parked to apply the saved selection')
  sm = messaging.SubMaster(['carState', 'carControl', 'pandaStates', 'carStateExt'])
  end = time.monotonic() + 3
  while time.monotonic() < end:sm.update(200)
  print('Services alive:', sm.alive)
  if sm.seen['carState']:
    cs=sm['carState']
    print('CAN valid:',cs.canValid,'ACC fault:',cs.accFaulted,'Steering temporary fault:',cs.steerFaultTemporary)
  if sm.seen['carControl']:print('Openpilot longitudinal active:',sm['carControl'].longActive)
  if sm.seen['carStateExt']:
    ext = sm['carStateExt']
    print('Host LKA latched:', ext.legacyLkaLatched, 'Reason:', ext.legacyLkaLatchReason or 'none',
          'Status valid/live:', sm.all_checks(['carStateExt']))
  for p in sm['pandaStates']:
    print('Panda:',p.pandaType,'Safety:',p.safetyModel,'Parameter:',p.safetyParam,
          'RX checks invalid:',p.safetyRxChecksInvalid,'TX blocked counter:',p.safetyTxBlocked)
  print('A parked unavailable-steering status is possible. Firmware signature requires the startup log check.')


if __name__=='__main__':main()
