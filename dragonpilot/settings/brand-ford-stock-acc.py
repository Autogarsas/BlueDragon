from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Longitudinal",
    "key": "dp_ford_stock_lon",
    "type": "toggle_item",
    "title": lambda: tr("Use Ford Factory ACC"),
    "description": lambda: tr("Ford's original ACC controls acceleration, braking and following distance. BlueDragon keeps steering. Resume from a stop using the car's normal controls. Change while parked and restart the comma to apply."),
    "brands": ["ford"],
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
