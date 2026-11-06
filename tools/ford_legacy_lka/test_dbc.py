"""Check passive raw decoding against the two independently maintained DBCs."""
import unittest

from opendbc.can import CANPacker
from tools.ford_legacy_lka.protocol import decode_lka, decode_status


class TestDBCInterpretations(unittest.TestCase):
  def test_legacy_dbc_matches_raw_bit_locations_and_scaling(self):
    packer = CANPacker("ford_fusion_2018_pt")
    for angle in (-1.0, 0.0, 1.0):
      for curvature in (-0.0001, 0, 0.0001):
        _, data, _ = packer.make_can_msg("Lane_Keep_Assist_Control", 0, {
          "Steer_Angle_Req": angle, "Lane_Curvature": curvature, "Lkas_Action": 5, "Lkas_Alert": 15})
        view = decode_lka(data)["fusion_2018_view"]
        self.assertEqual(5, view["action"])
        self.assertEqual(15, view["alert"])
        self.assertAlmostEqual(angle, view["steer_angle_req_dbc_deg"], delta=0.022)
        self.assertAlmostEqual(curvature, view["curvature_dbc_per_m"], delta=2.6e-6)

  def test_base_dbc_matches_raw_bit_locations_and_scaling(self):
    packer = CANPacker("ford_lincoln_base_pt")
    for angle in (-1.0, 0.0, 1.0):
      for curvature in (-0.0001, 0, 0.0001):
        for direction in (0, 2, 4):
          _, data, _ = packer.make_can_msg("Lane_Assist_Data1", 0, {
            "LaRefAng_No_Req": angle, "LaCurvature_No_Calc": curvature,
            "LkaActvStats_D2_Req": direction, "LkaDrvOvrrd_D_Rq": 2, "LaRampType_B_Req": 1,
            "LdwActvStats_D_Req": 3, "LdwActvIntns_D_Req": 1})
          view = decode_lka(data)["ford_base_view"]
          self.assertEqual(direction, view["request"])
          self.assertEqual(2, view["driver_override_request"])
          self.assertTrue(view["quick_ramp"])
          self.assertEqual(3, view["ldw_request"])
          self.assertEqual(1, view["ldw_intensity"])
          self.assertAlmostEqual(angle, view["reference_dbc_mrad"], delta=0.026)
          self.assertAlmostEqual(curvature, view["curvature_dbc_per_m"], delta=2.6e-6)

  def test_status_matches_both_dbcs(self):
    for dbc, message in (("ford_fusion_2018_pt", "Lane_Keep_Assist_Status"),
                         ("ford_lincoln_base_pt", "Lane_Assist_Data3_FD1")):
      packer = CANPacker(dbc)
      for availability in range(4):
        for denied in (0, 1):
          _, data, _ = packer.make_can_msg(message, 0, {
            "LaActAvail_D_Actl": availability, "LaActDeny_B_Actl": denied, "LaHandsOff_B_Actl": 1})
          status = decode_status(data)
          self.assertEqual(availability, status["availability"])
          self.assertEqual(bool(denied), status["denied"])
          self.assertTrue(status["hands_off"])


if __name__ == "__main__":
  unittest.main()
