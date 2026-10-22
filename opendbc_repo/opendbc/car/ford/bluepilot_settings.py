"""Effective bp-5.0 Ford steering defaults and tuning bounds."""

SETTINGS = {
  "dp_ford_bp_enable_human_turn_detection": {"default": True, "type": "bool", "attr": "enable_human_turn_detection"},
  "dp_ford_bp_lane_change_factor_high": {"default": 0.85, "type": "float", "min": 0.5, "max": 1.0, "attr": "lane_change_factor_high"},
  "dp_ford_bp_pc_blend_ratio_high_c_u_i": {"default": 0.4, "type": "float", "min": 0.0, "max": 1.0, "attr": "pc_blend_ratio_high_C_UI"},
  "dp_ford_bp_pc_blend_ratio_low_c_u_i": {"default": 0.4, "type": "float", "min": 0.0, "max": 1.0, "attr": "pc_blend_ratio_low_C_UI"},
  "dp_ford_bp_enable_lane_positioning": {"default": False, "type": "bool", "attr": "enable_lane_positioning"},
  "dp_ford_bp_custom_path_offset": {"default": 0.0, "type": "float", "min": -0.5, "max": 0.5, "attr": "custom_path_offset"},
  "dp_ford_bp_path_offset_lookup_time": {"default": 0.0, "type": "float", "min": 0.0, "max": 0.8, "attr": "path_offset_lookup_time"},
  "dp_ford_bp_enable_lane_full_mode": {"default": False, "type": "bool", "attr": "enable_lanefull_mode"},
  "dp_ford_bp_custom_profile": {"default": 0, "type": "int", "min": 0, "max": 1, "attr": "custom_profile"},
  "dp_ford_bp_l_c_p_i_d_gain_u_i": {"default": 3.0, "type": "float", "min": 0.0, "max": 5.0, "attr": "LC_PID_gain_UI"},
}
