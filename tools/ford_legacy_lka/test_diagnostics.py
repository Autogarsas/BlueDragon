"""Offline checks for refusal diagnostics; no vehicle or cereal runtime needed."""
import json
from types import SimpleNamespace as NS
import unittest

from tools.ford_legacy_lka import capture
from tools.ford_legacy_lka.protocol import Session


def event(kind, payload, t=0):
  return NS(which=lambda: kind, logMonoTime=t, valid=True, **{kind: payload})


class TestEngagementDiagnostics(unittest.TestCase):
  def test_pedal_cruise_and_button_evidence_survives_collection(self):
    session = Session("fusion_2019", "after")
    state = NS(vEgo=19., steeringAngleDeg=0., steeringTorque=1.0625,
               steeringPressed=False, gasPressed=True, brakePressed=False,
               cruiseState=NS(enabled=True, available=True), vehicleSensorsInvalid=False,
               buttonEvents=[NS(type="setCruise", pressed=True)],
               canValid=True, steerFaultTemporary=True, steerFaultPermanent=False)
    capture.add_event(session, event("carState", state))
    row = session.states[0]
    self.assertTrue(row["gas_pressed"])
    self.assertFalse(row["brake_pressed"])
    self.assertTrue(row["cruise_enabled"])
    self.assertEqual([{"type": "setCruise", "pressed": True}], row["button_events"])
    self.assertEqual(1.0625, row["driver_torque"])
    json.dumps(session.report(), allow_nan=False)

  def test_no_entry_reason_and_screen_alert_are_retained(self):
    session = Session("fusion_2019", "after")
    names = ("enable", "noEntry", "warning", "userDisable", "softDisable", "immediateDisable",
             "preEnable", "permanent", "overrideLateral", "overrideLongitudinal")
    reason = NS(name="calibrationIncomplete", **{n: n == "noEntry" for n in names})
    capture.add_event(session, event("onroadEvents", [reason], 10))
    state = NS(state="disabled", enabled=False, active=False, engageable=False,
               alertType="calibrationIncomplete/noEntry", alertText1="Calibration in Progress",
               alertText2="Drive to calibrate")
    capture.add_event(session, event("selfdriveState", state, 11))
    self.assertEqual([{"name": "calibrationIncomplete", "types": ["noEntry"]}], session.states[0]["events"])
    self.assertEqual("Calibration in Progress", session.states[1]["alert_text_1"])
    self.assertFalse(session.states[1]["engageable"])

  def test_full_minute_preserves_transient_alert_without_state_truncation(self):
    session = Session("fusion_2019", "after")
    for i in range(6000):
      t = i * 10000000
      for kind in ("carState", "carControl"):
        session.add_state({"kind": kind, "mono_ns": t})
      if i % 3 == 0:
        session.add_state({"kind": "legacyLkaCommand", "mono_ns": t, "command": {"latched": False}})
      if i % 10 == 0:
        session.add_state({"kind": "pandaStates", "mono_ns": t})
      if i % 100 == 0:
        session.add_state({"kind": "onroadEvents", "mono_ns": t, "events": []})
      # This single-frame alert must survive heartbeat sampling.
      session.add_state({"kind": "selfdriveState", "mono_ns": t,
                         "alert_type": "steerUnavailable/noEntry" if i == 55 else ""})
    self.assertFalse(session.states_truncated)
    self.assertLess(len(session.states), 15000)
    self.assertTrue(any(r.get("alert_type") == "steerUnavailable/noEntry" for r in session.states))
    changes = [r for r in session.states if r["kind"] == "selfdriveState"]
    self.assertEqual([0, 550000000, 560000000], [r["mono_ns"] for r in changes[:3]])

  def test_original_interruption_reason_survives_latched_messages(self):
    session = Session("fusion_2019", "after")
    for t, reason, latched in [(1, "active", False), (2, "cruise_or_brake", True),
                               (3, "latched_after_interruption", True), (4, "not_requested", False),
                               (5, "active", False), (6, "driver_override", True)]:
      session.add_state({"kind": "legacyLkaCommand", "mono_ns": t,
                         "command": {"reason": reason, "latched": latched}})
    self.assertEqual([
      {"first_observed_mono_ns": 2, "observed_trigger_reason": "cruise_or_brake", "cleared_mono_ns": 4},
      {"first_observed_mono_ns": 6, "observed_trigger_reason": "driver_override", "cleared_mono_ns": None},
    ], session.report()["host_latch_observations"])

  def test_capture_started_latched_does_not_invent_a_trigger(self):
    session = Session("fusion_2019", "after")
    session.add_state({"kind": "legacyLkaCommand", "mono_ns": 5,
                       "command": {"reason": "latched_after_interruption", "latched": True}})
    self.assertIsNone(session.report()["host_latch_observations"][0]["observed_trigger_reason"])


if __name__ == "__main__":
  unittest.main()
