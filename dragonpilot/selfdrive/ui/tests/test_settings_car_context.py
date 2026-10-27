"""Exercise the real settings lifecycle methods without graphics dependencies."""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
LAYOUT = ROOT / "dragonpilot/selfdrive/ui/layouts/settings/dragonpilot.py"
SETTING = ROOT / "dragonpilot/settings/min-feat.lat.alka-v2.py"


class ScrollerStub:
  def __init__(self, items=(), **kwargs):
    self.items = items

  def show_event(self):
    pass


class TestSettingsCarContext(unittest.TestCase):
  def setUp(self):
    source = ast.parse(LAYOUT.read_text())
    layout = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == "DragonpilotLayout")
    methods = [n for n in layout.body if isinstance(n, ast.FunctionDef) and
               n.name in ("_refresh_visibility", "_check_condition", "show_event")]
    # Compile the production methods unchanged; replace only graphics and I/O.
    cls = ast.ClassDef(name="LayoutUnderTest", bases=[], keywords=[], body=methods, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    self.ui = SimpleNamespace(CP=None, update_params=lambda: None)
    namespace = {"ui_state": self.ui, "Scroller": ScrollerStub, "LITE": False, "MICI": False}
    exec(compile(module, str(LAYOUT), "exec"), namespace)
    self.panel = namespace["LayoutUnderTest"]()
    self.panel._brand = ""
    self.panel._car_fingerprint = ""
    self.panel._openpilot_longitudinal_control = False
    self.panel._reset_dp_conf_btn = object()
    self.panel._scroller = ScrollerStub()
    self.panel._update_toggles = lambda: None
    self.builds = []
    settings = ast.parse(SETTING.read_text())
    item = next(n.value for n in settings.body if isinstance(n, ast.Assign) and n.targets[0].id == "ITEMS").elts[0]
    fields = {ast.literal_eval(k): v for k, v in zip(item.keys, item.values)}
    self.condition = ast.literal_eval(fields["condition"])
    self.brands = ast.literal_eval(fields["brands"])

    def load_settings():
      visible = self.panel._brand in self.brands and self.panel._check_condition(self.condition)
      self.builds.append((self.panel._brand, self.panel._car_fingerprint, visible))
      self.panel._toggles = {"dp_lat_alka": object()} if visible else {}

    self.panel._load_settings = load_settings
    load_settings()

  def car(self, fingerprint, brand="chrysler", longitudinal=False):
    return SimpleNamespace(brand=brand, carFingerprint=fingerprint, openpilotLongitudinalControl=longitudinal)

  def test_profile_arrives_after_panel_construction(self):
    self.ui.CP = self.car("CHRYSLER_PACIFICA_2018")
    self.panel.show_event()
    self.assertTrue(self.builds[-1][2])
    self.assertIn("dp_lat_alka", self.panel._toggles)

  def test_same_brand_profile_change_updates_visibility(self):
    self.ui.CP = self.car("JEEP_GRAND_CHEROKEE")
    self.panel.show_event()
    self.assertFalse(self.builds[-1][2])
    self.ui.CP = self.car("CHRYSLER_PACIFICA_2018")
    self.panel.show_event()
    self.assertTrue(self.builds[-1][2])
    self.ui.CP = self.car("RAM_1500_5TH_GEN")
    self.panel.show_event()
    self.assertFalse(self.builds[-1][2])

  def test_current_params_loaded_before_visibility_evaluation(self):
    self.ui.update_params = lambda: setattr(self.ui, "CP", self.car("CHRYSLER_PACIFICA_2018"))
    self.panel.show_event()
    self.assertTrue(self.builds[-1][2])

  def test_identical_context_does_not_rebuild(self):
    self.ui.CP = self.car("CHRYSLER_PACIFICA_2018")
    self.panel.show_event()
    count = len(self.builds)
    self.panel.show_event()
    self.assertEqual(count, len(self.builds))

  def test_existing_ford_visibility_is_preserved(self):
    self.ui.CP = self.car("FORD_EDGE_MK2", brand="ford")
    self.panel.show_event()
    self.assertTrue(self.builds[-1][2])

  def test_lost_context_clears_profile_and_toggle(self):
    self.ui.CP = self.car("CHRYSLER_PACIFICA_2018")
    self.panel.show_event()
    self.ui.CP = None
    self.panel.show_event()
    self.assertEqual("", self.panel._car_fingerprint)
    self.assertFalse(self.builds[-1][2])


if __name__ == "__main__":
  unittest.main()
