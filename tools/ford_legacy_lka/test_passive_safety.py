"""The research branch must not accidentally authorize active LKA frames."""
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from tools.ford_legacy_lka.protocol import preview_lka


class TestPassiveLKASafety(unittest.TestCase):
  def test_existing_ford_modes_reject_both_active_preview_layouts(self):
    safety = libsafety_py.libsafety
    for param in (0, 1, 4, 5, 8, 12):
      for alternative in (0, 1024):
        for allowed in (False, True):
          for layout in ("fusion_2018", "ford_base"):
            for direction in ("left", "right"):
              for bus in (0, 1, 2):
                safety.set_safety_hooks(CarParams.SafetyModel.ford, param)
                safety.init_tests()
                safety.set_alternative_experience(alternative)
                safety.set_controls_allowed(allowed)
                packet = libsafety_py.make_CANPacket(0x3CA, bus, preview_lka(layout, direction=direction))
                with self.subTest(param=param, alka=alternative, allowed=allowed, layout=layout,
                                  direction=direction, bus=bus):
                  self.assertFalse(safety.safety_tx_hook(packet))


if __name__ == "__main__":
  unittest.main()
