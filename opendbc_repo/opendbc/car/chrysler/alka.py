"""Pacifica ALKA button state, adapted from BluePilot 5.0 / sunnypilot MADS.

MADS contributors: Copyright (c) 2021-, Haibin Wen, sunnypilot, and contributors.
The original MADS implementation is MIT licensed; see CREDITS.md.
"""

from opendbc.car import structs


def pacifica_steering_allowed(CS, lkas_on: bool, min_speed: float) -> bool:
  return (lkas_on and CS.canValid and CS.vEgo > max(min_speed, 17.5) and not CS.standstill and
          CS.gearShifter in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.low) and
          not CS.doorOpen and not CS.seatbeltUnlatched and not CS.steerFaultTemporary and not CS.steerFaultPermanent)


class PacificaALKA:
  """ACC main rising enables steering; LKAS toggles it; ACC main off disables it.

  ACC cancel and brake release never re-arm a disabled LKAS switch. The host and
  Panda start disabled and derive permission from the same physical CAN inputs.
  """

  def __init__(self):
    self.main_prev = False
    self.button_prev = False
    self.requested = False

  def update(self, main_on: bool, button_samples: list[float]) -> bool:
    if main_on and not self.main_prev:
      self.requested = True
    if not main_on:
      self.requested = False
    for pressed in button_samples:
      if pressed and not self.button_prev and main_on:
        self.requested = not self.requested
      self.button_prev = bool(pressed)
    self.main_prev = main_on
    return self.requested
