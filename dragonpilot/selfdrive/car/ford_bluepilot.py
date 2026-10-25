"""Messaging and settings adapter for the bp-5.0 Ford lateral port."""
import cereal.messaging as messaging
from opendbc.car.ford.bluepilot_settings import SETTINGS


class FordBluePilotAdapter:
  def __init__(self, controller, params):
    self.controller = controller
    self.params = params
    self.sm = messaging.SubMaster(["modelV2"])
    self.settings_frame = 0
    from openpilot.common.swaglog import cloudlog
    from dragonpilot.selfdrive.car.ford_diagnostics import FordSteeringDiagnostics
    self.diagnostics = FordSteeringDiagnostics(cloudlog.event, controller.CP.carFingerprint)

  def update(self):
    self.sm.update(0)
    # No stale model fallback: the lateral module emits inactive, zeroed commands.
    self.controller.bp_model = self.sm["modelV2"] if self.sm.all_checks(["modelV2"]) else None
    if self.settings_frame % 100 == 0:
      values = {}
      for key, spec in SETTINGS.items():
        value = self.params.get(key)
        values[key] = spec["default"] if value is None else value
      self.controller.bp_lateral.configure(values)
    self.settings_frame += 1

  def log_diagnostics(self):
    self.diagnostics.observe(self.controller.bp_lateral.diagnostics)
