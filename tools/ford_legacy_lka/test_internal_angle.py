"""Independent original-encoder fixtures and invalid/stale-frame regressions."""
import unittest
from tools.ford_legacy_lka.internal_angle import decode_internal_angle, decode_angle_did, compare_frames

# Produced by the original K2GC encoder and checked against its DID 3302 encoder.
FIXTURES = [
    ("7cc2800030607d0e", -7.6, "056233021e2c0000"),
    ("7d70800084607d0b", 10.1, "056233021edd0000"),
    ("7d0a8000eb607d0a", 0., "056233021e780000"),
    ("7cff8000e1607d20", -3.3, "056233021e570000"),
    ("7d8680005c607d1d", 10.5, "056233021ee10000"),
    ("7cdb800005607d20", -6.9, "056233021e330000"),
    ("7d8c80005b607d18", 11.6, "056233021eec0000"),
]


class TestInternalAngle(unittest.TestCase):
    def test_original_encoder_and_did_fixtures_agree(self):
        for packet, expected, did in FIXTURES:
            with self.subTest(packet=packet):
                self.assertEqual(decode_internal_angle(bytes.fromhex(packet))["internal_angle_deg"], expected)
                self.assertEqual(decode_angle_did(bytes.fromhex(did)), expected)

    def test_single_bit_corruption_is_rejected_except_independent_power_bit(self):
        for packet, _, _ in FIXTURES:
            original=bytes.fromhex(packet)
            for position in range(64):
                data=bytearray(original);data[position//8] ^= 1 << (position%8)
                decoded=decode_internal_angle(data)
                if position == 23:  # original power bit, outside checksum
                    self.assertIsNotNone(decoded)
                    self.assertEqual(decoded["internal_angle_deg"], decode_internal_angle(original)["internal_angle_deg"])
                else:
                    self.assertIsNone(decoded, (packet,position))

    def test_unavailable_field_sentinels_are_rejected_with_valid_checksum(self):
        for field in (slice(0,2),slice(6,8)):
            for unavailable in (0xfffe,0xffff):
                data=bytearray.fromhex(FIXTURES[0][0]);data[field]=unavailable.to_bytes(2,"big")
                data[4]=(255-(data[0]+data[1]+data[6]+data[7]+(data[5]>>4)))&255
                self.assertIsNone(decode_internal_angle(data))

    def test_paired_readings_require_native_valid_fresh_data(self):
        packet,_,did=FIXTURES[1];packet=bytes.fromhex(packet);did=bytes.fromhex(did)
        good=compare_frames([(1_000_000_000,0,0x85,packet,True),(1_010_000_000,0,0x738,did,True)])
        self.assertEqual(good["all_pairs"]["count"],1)
        self.assertEqual(good["all_pairs"]["max_absolute_deg"],0.)
        for bus,valid,age in ((128,True,10_000_000),(2,True,10_000_000),(0,False,10_000_000),(0,True,100_000_001)):
            result=compare_frames([(1_000_000_000,bus,0x85,packet,valid),(1_000_000_000+age,0,0x738,did,True)])
            self.assertEqual(result["all_pairs"]["count"],0)

    def test_invalid_frame_and_backwards_time_break_pairing(self):
        packet=bytes.fromhex(FIXTURES[0][0]);did=bytes.fromhex(FIXTURES[0][2])
        for middle in ((1_005_000_000,0,0x85,bytes(8),True),
                       (1_005_000_000,0,0x85,packet,False),
                       (900_000_000,0,0x738,bytes(8),True)):
            result=compare_frames([(1_000_000_000,0,0x85,packet,True),middle,(1_010_000_000,0,0x738,did,True)])
            self.assertEqual(result["all_pairs"]["count"],0)

    def test_counter_wrap_and_duplicate_are_reported(self):
        data=bytearray.fromhex(FIXTURES[0][0]);frames=[]
        for ns,counter in ((1_000_000_000,15),(1_010_000_000,0),(1_020_000_000,0),(1_030_000_000,2)):
            data[5]=counter<<4
            data[4]=(255-(data[0]+data[1]+data[6]+data[7]+counter))&255
            frames.append((ns,0,0x85,bytes(data),True))
        result=compare_frames(frames)
        self.assertEqual(result["counts"]["counter_duplicates"],1)
        self.assertEqual(result["counts"]["counter_skips"],1)


if __name__ == "__main__":
    unittest.main()