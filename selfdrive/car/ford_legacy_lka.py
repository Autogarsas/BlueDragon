"""Host stop-latch presentation; no actuator commands or rearm operations."""


def lateral_allowed(extension_valid: bool, latched: bool) -> bool:
  # Missing/stale telemetry cannot restore lateral activity.
  return extension_valid and not latched


def stop_title(reason: str) -> str:
  cause = {
    "accelerator_pressed": "accelerator",
    "brake_pressed": "brake",
    "cruise_inactive": "cruise stopped",
    "driver_override": "steering input",
    "cruise_or_brake": "pedal/cruise input",
    "rack_unavailable_or_denied": "EPS unavailable",
    "rack_fault": "EPS fault",
    "stale_rack_status": "stale data",
    "invalid_vehicle_data": "invalid data",
    "nonfinite_input": "invalid input",
    "outside_test_speed_range": "speed limit",
    "outside_curvature_authority": "curve limit",
    "outside_reference_authority": "angle limit",
    "output_tracking_limit": "tracking limit",
    "command_interval": "timing check",
    "lateral_inactive": "lateral stopped",
  }.get(reason)
  return f"LKA stopped: {cause}" if cause else "LKA stopped"
