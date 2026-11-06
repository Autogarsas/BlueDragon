from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Ford Steering (BluePilot)",
    "key": "dp_ford_legacy_lka",
    "type": "toggle_item",
    "title": lambda: tr("Legacy LKA Research — Steering Disabled"),
    "description": lambda: tr("OFF keeps your current LCA steering mode. ON selects the LKA research path, with all BlueDragon control disabled and factory ACC retained. Active LKA is pending protocol validation and matching Panda limits. Change while parked and restart the comma to apply."),
    "brands": ["ford"],
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
