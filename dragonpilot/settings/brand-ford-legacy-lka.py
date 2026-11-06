from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Ford Steering (BluePilot)",
    "key": "dp_ford_legacy_lka",
    "type": "toggle_item",
    "title": lambda: tr("Use Legacy LKA — Experimental"),
    "description": lambda: tr("OFF keeps your current LCA steering mode. ON uses limited-authority legacy LKA on the Fusion retrofit/Edge MK2 profile, with factory ACC. Test speed range 54–108 km/h. Assistance may be interrupted by the rack; disengage before rearming. Change while parked and restart the comma to apply."),
    "brands": ["ford"],
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
]
