import ast
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from tools.ford_legacy_lka import capture, protocol
from tools.ford_legacy_lka.protocol import Session, decode_lka, decode_status, preview_lka


def event(kind, **data):
  return NS(which=lambda: kind, logMonoTime=1234567890, valid=True, **{kind: NS(**data)})


class TestFordLKAEvidence(unittest.TestCase):
  def test_historical_zero_fixture_has_two_distinct_interpretations(self):
    # Literal bytes and expected values are independent of preview_lka.
    d = decode_lka(bytes.fromhex("5f7d080000000000"))
    self.assertEqual(2048, d["reference_raw"])
    self.assertEqual(2000, d["curvature_raw"])
    old, new = d["fusion_2018_view"], d["ford_base_view"]
    self.assertEqual((5, 15), (old["action"], old["alert"]))
    self.assertAlmostEqual(-0.00189, old["steer_angle_req_dbc_deg"])
    self.assertAlmostEqual(0, old["curvature_dbc_per_m"])
    self.assertEqual((2, 7, 3), (new["request"], new["ldw_request"], new["ldw_intensity"]))
    self.assertAlmostEqual(0, new["reference_dbc_mrad"])
    self.assertAlmostEqual(-0.00024, new["curvature_dbc_per_m"])

  def test_ordinary_base_fixture_does_not_request_alert_or_driver_override(self):
    d = decode_lka(bytes.fromhex("4080081000000000"))
    self.assertEqual(2064, d["reference_raw"])
    self.assertEqual(2048, d["curvature_raw"])
    new = d["ford_base_view"]
    self.assertEqual(2, new["request"])
    self.assertAlmostEqual(0.8, new["reference_dbc_mrad"])
    self.assertEqual(0, new["ldw_request"])
    self.assertEqual(0, new["driver_override_request"])
    self.assertFalse(new["quick_ramp"])

  def test_malformed_frames_remain_evidence_without_a_decode(self):
    for size in (0, 1, 7, 9, 16):
      for address, decoder in ((0x3CA, decode_lka), (0x3CC, decode_status)):
        with self.assertRaises(ValueError):
          decoder(bytes(size))
        s = Session("fusion_2019", "baseline")
        s.add_frame("can", 2, address, bytes(size), 1000)
        self.assertIn("decode_error", s.frames[0])
        self.assertNotIn("status_view", s.frames[0])

  def test_preview_is_bounded_explicit_and_keeps_layout_centres_distinct(self):
    self.assertEqual("ff7d080000000000", preview_lka("fusion_2018").hex())
    self.assertEqual("0080080000000000", preview_lka("ford_base").hex())
    for layout in protocol.LAYOUTS:
      for direction in ("off", "left", "right"):
        for raw in (2032, 2048, 2064):
          self.assertEqual(raw, decode_lka(preview_lka(layout, raw, direction=direction))["reference_raw"])
      for raw in (None, True, 2048.0, float("nan"), -1, 2031, 2065, 4095):
        with self.assertRaises(ValueError):
          preview_lka(layout, reference_raw=raw)
      for raw in (-1, 0, 4095, True, 2048.0):
        with self.assertRaises(ValueError):
          preview_lka(layout, curvature_raw=raw)
    with self.assertRaises(ValueError):
      preview_lka("automatic")
    with self.assertRaises(ValueError):
      preview_lka("ford_base", direction="increased_intervention")

  def test_status_bits_do_not_confuse_denial_and_availability(self):
    d = decode_status(bytes.fromhex("f000000000000000"))
    self.assertEqual(3, d["availability"])
    self.assertTrue(d["denied"])
    self.assertTrue(d["hands_off"])
    self.assertTrue(d["available_candidate"])

  def test_buses_and_host_requests_are_never_merged_with_eps_status(self):
    s = Session("fusion_2019", "before")
    for i in range(4):
      s.add_frame("can", 0, 0x3CC, bytes.fromhex("3000000000000000"), i * 100000000)
      s.add_frame("can", 2, 0x3CC, bytes(8), i * 100000000)
      s.add_frame("sendcan", 0, 0x3CC, bytes.fromhex("3000000000000000"), i * 100000000)
    r = s.report()
    self.assertEqual(3, len(r["counts"]))
    self.assertEqual([0.3, 0], [v["longest_observed_available_run_s"] for v in r["status_observations"]])
    self.assertFalse(r["eps_acceptance_proven"])

  def test_gaps_and_denial_break_an_available_run(self):
    s = Session("mkz_2016", "before")
    # An out-of-order log must be sorted within this bus, without bridging a gap.
    for t, first in ((1000000000, 0x30), (100000000, 0x30), (0, 0x30),
                     (1100000000, 0x70), (1200000000, 0x30)):
      s.add_frame("can", 0, 0x3CC, bytes([first]) + bytes(7), t)
    r = s.report()["status_observations"][0]
    self.assertEqual(1, r["gaps_over_200ms"])
    self.assertEqual(0.1, r["longest_observed_available_run_s"])
    self.assertEqual([False, True, False], [v["denied"] for v in r["transitions"]])

  def test_more_than_ten_seconds_available_is_not_proof_of_execution(self):
    s = Session("mkz_2016", "after")
    for i in range(121):
      s.add_frame("can", 0, 0x3CC, bytes.fromhex("3000000000000000"), i * 100000000)
    r = s.report()
    self.assertEqual(12, r["status_observations"][0]["longest_observed_available_run_s"])
    self.assertFalse(r["layout_confirmed"])
    self.assertFalse(r["eps_acceptance_proven"])
    self.assertFalse(r["live_lka_mode_implemented"])

  def test_capture_is_bounded_and_reports_truncation(self):
    s = Session("mkz_2016", "baseline", max_frames=2)
    for i in range(6):
      s.add_frame("can", 0, 0x3CA, bytes(8), i)
    self.assertEqual(2, len(s.frames))
    self.assertEqual(6, s.report()["counts"][0]["count"])
    self.assertTrue(s.report()["truncated"])
    s.add_frame("can", 0, 0x123, bytes(8), 100)
    self.assertEqual(1, len(s.counts))

  def test_vehicle_choices_are_labels_not_compatibility_claims(self):
    for vehicle in ("fusion_2019", "mkz_2016"):
      r = Session(vehicle, "baseline").report()
      self.assertEqual(vehicle, r["vehicle_label"])
      self.assertTrue(r["vehicle_label_is_not_fingerprinting"])
      self.assertFalse(r["layout_confirmed"])
    with self.assertRaises(ValueError):
      Session("pacifica", "before")

  def test_nonfinite_values_are_recorded_as_json_null(self):
    s = Session("mkz_2016", "baseline")
    add = event("carControl", enabled=True, latActive=True, actuators=NS(curvature=float("nan")))
    capture.add_event(s, add)
    self.assertIsNone(s.states[0]["desired_curvature"])
    json.dumps(s.report(), allow_nan=False)

  def test_raw_rx_and_host_tx_are_distinct(self):
    s = Session("fusion_2019", "baseline")
    f = NS(src=128, address=0x3CA, dat=bytes(8))
    for kind in ("can", "sendcan"):
      capture.add_event(s, NS(which=lambda k=kind: k, logMonoTime=1, valid=True, **{kind: [f]}))
    self.assertEqual(["can", "sendcan"], [f["stream"] for f in s.frames])
    self.assertEqual([128, 128], [f["bus"] for f in s.frames])

  def test_invalid_status_cannot_extend_available_run(self):
    s = Session("fusion_2019", "baseline")
    for i, valid in enumerate((True, True, False, True)):
      s.add_frame("can", 0, 0x3CC, bytes.fromhex("3000000000000000"), i * 100000000, valid)
    r = s.report()["status_observations"][0]
    self.assertEqual(0.1, r["longest_observed_available_run_s"])
    self.assertEqual([True, False, True], [t["event_valid"] for t in r["transitions"]])

  def test_existing_report_is_preserved_before_listener_starts(self):
    with tempfile.TemporaryDirectory() as d:
      out = Path(d) / "report.json"
      out.write_text("existing evidence")
      with patch.object(capture, "metadata", return_value={}), patch.object(capture, "listen") as listen:
        with self.assertRaises(FileExistsError):
          capture.main(["--vehicle", "mkz_2016", "--output", str(out)])
        listen.assert_not_called()
      self.assertEqual("existing evidence", out.read_text())

  def test_failed_listener_writes_partial_report_and_returns_failure(self):
    with tempfile.TemporaryDirectory() as d:
      out = Path(d) / "report.json"
      with (patch.object(capture, "metadata", return_value={}),
            patch.object(capture, "listen", side_effect=RuntimeError("service missing")),
            contextlib.redirect_stdout(io.StringIO())):
        code = capture.main(["--vehicle", "fusion_2019", "--output", str(out)])
      self.assertEqual(1, code)
      r = json.loads(out.read_text())
      self.assertIn("service missing", r["errors"][0])
      self.assertFalse(r["capture_sends_can"])

  def test_passive_modules_have_no_vehicle_write_or_transport_api(self):
    for module in (capture, protocol):
      tree = ast.parse(Path(module.__file__).read_text())
      names = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
      self.assertFalse({"PubMaster", "send", "sendcan", "can_send", "put", "put_bool", "flash", "Panda"} & names)
      imports = [n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
      self.assertFalse(any(n.startswith(("panda", "usb", "uds")) for n in imports))

  def test_listener_only_subscribes_and_preserves_traffic_direction(self):
    created, served = [], set()
    fake = NS()
    def subscribe(name, **kwargs):
      self.assertEqual({"timeout": 0}, kwargs)
      created.append(name)
      return name
    def receive(sock):
      if sock in served:
        return None
      served.add(sock)
      if sock in ("can", "sendcan"):
        return NS(which=lambda: sock, logMonoTime=1, valid=True,
                  **{sock: [NS(src=2, address=0x3CA, dat=bytes(8))]})
      return None
    fake.sub_sock, fake.recv_one_or_none = subscribe, receive
    ticks = iter(i * 0.0001 for i in range(10000))
    s = Session("mkz_2016", "baseline")
    with (patch.dict("sys.modules", {"cereal.messaging": fake}),
          patch.object(capture.time, "monotonic", side_effect=lambda: next(ticks)),
          patch.object(capture.time, "sleep")):
      capture.listen(s, 0.01)
    self.assertEqual(["can", "sendcan", "carState", "carControl", "pandaStates", "logMessage",
                      "selfdriveState", "onroadEvents", "carStateExt"], created)
    self.assertEqual(["can", "sendcan"], [r["stream"] for r in s.frames])

  def test_real_capnp_receive_and_panda_state_events(self):
    from cereal import log
    s = Session("fusion_2019", "before")
    for kind in ("can", "sendcan", "carState", "carControl", "pandaStates"):
      m = log.Event.new_message()
      m.valid = True
      m.logMonoTime = 123
      if kind in ("can", "sendcan"):
        f = m.init(kind, 1)[0]
        f.src, f.address, f.dat = 2, 0x3CC, bytes.fromhex("3000000000000000")
      elif kind == "pandaStates":
        p = m.init(kind, 1)[0]
        p.ignitionCan = True
        p.safetyModel, p.safetyParam = "ford", 12
      else:
        m.init(kind)
      with log.Event.from_bytes(m.to_bytes()) as read:
        capture.add_event(s, read)
    self.assertEqual(2, len(s.frames))
    self.assertEqual(3, len(s.states))
    self.assertEqual("ford", s.states[-1]["pandas"][0]["safety_model"])
    self.assertTrue(s.states[-1]["pandas"][0]["ignition"])
    json.dumps(s.report(), allow_nan=False)

  def test_pinion_frames_and_full_minute_of_host_states_are_retained(self):
    s = Session("fusion_2019", "before")
    for addr in (0x7E, 0x85):
      s.add_frame("can", 0, addr, bytes(range(8)), 1)
    self.assertEqual(2, len(s.frames))
    for i in range(16000):
      s.add_state({"kind": "carState", "mono_ns": i})
    self.assertEqual(16000, len(s.states))
    self.assertFalse(s.states_truncated)
    for i in range(5000):
      s.add_state({"kind": "carControl", "mono_ns": i})
    self.assertTrue(s.report()["states_truncated"])

  def test_only_legacy_command_diagnostics_are_kept_from_logs(self):
    s = Session("fusion_2019", "before")
    for payload in ('invalid json', json.dumps({"msg": "unrelated"}),
                    json.dumps({"msg": {"event": "bluedragon_ford_legacy_lka", "reason": "rack_unavailable_or_denied"}})):
      capture.add_event(s, NS(which=lambda: "logMessage", logMonoTime=1000, valid=True, logMessage=payload))
    self.assertEqual(1, len(s.states))
    self.assertEqual("legacyLkaCommand", s.states[0]["kind"])


if __name__ == "__main__":
  unittest.main()

