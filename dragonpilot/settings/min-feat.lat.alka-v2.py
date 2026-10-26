from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Lateral",
    "key": "dp_lat_alka",
    "type": "toggle_item",
    "title": lambda: tr("Always-on Lane Keeping Assist (ALKA)"),
    "description": lambda: tr("Enable lateral control even when ACC/cruise is disengaged, using ACC Main or LKAS button to toggle. Vehicle must be moving. Pacifica: ACC Main enables, LKAS toggles, steering only above 63 km/h. Restart the device after changing this setting."),
    "brands": ["toyota", "hyundai", "honda", "volkswagen", "subaru", "mazda", "nissan", "ford", "chrysler"],
    "condition": "brand != 'chrysler' or carFingerprint.startswith('CHRYSLER_PACIFICA_')",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
