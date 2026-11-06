#pragma once

// Dedicated, limited-authority legacy-LKA test path. Bounds are expressed in
// raw 0x3CA counts, not assumed to be degrees in the newer DBC. The historical
// 0.04297-degree/count equivalent is used ONLY for the independent wheel-error
// check. The absolute 128-count cap bounds both candidate interpretations.
static bool ford_legacy_lka = false;
static bool ford_lka_active_last = false;
static int ford_lka_reference_last = 0;
static int ford_lka_curvature_last = 0;
static int ford_lka_wheel_decideg = 0;
static int ford_lka_camera_decideg = 0;
static int ford_lka_torque_raw = 0;
static bool ford_lka_available = false;
static bool ford_lka_eps_ok = false;
static uint32_t ford_lka_last_tx_ts = 0U;

static void ford_lka_reset(void) {
  ford_lka_active_last = false;
  ford_lka_reference_last = 0;
  ford_lka_curvature_last = 0;
  ford_lka_wheel_decideg = 0;
  ford_lka_camera_decideg = 0;
  ford_lka_torque_raw = 0;
  ford_lka_available = false;
  ford_lka_eps_ok = false;
  ford_lka_last_tx_ts = 0U;
}

static void ford_lka_rx(const CANPacket_t *msg) {
  if (msg->addr == 0x76U) {
    // Older-platform SCCM steering-wheel angle; independent of host commands.
    ford_lka_wheel_decideg = (int)(((msg->data[0] & 0x7FU) << 8) | msg->data[1]) - 16000;
  } else if (msg->addr == 0x3A8U) {
    // Cross-check the camera reference used by the existing Edge retrofit.
    ford_lka_camera_decideg = (int)(((msg->data[2] & 0x7FU) << 8) | msg->data[3]) - 10000;
  } else if (msg->addr == 0x82U) {
    ford_lka_torque_raw = (int)msg->data[0] - 128;  // 1/16 Nm
    ford_lka_eps_ok = (msg->data[1] & 3U) == 0U;
  } else if (msg->addr == 0x3CCU) {
    const unsigned int availability = (msg->data[0] >> 4) & 3U;
    ford_lka_available = ((availability == 2U) || (availability == 3U)) && !GET_BIT(msg, 6U);
  } else {
  }
}

static bool ford_lka_inputs_fresh(uint32_t now) {
  bool valid = true;
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxStatus *status = &current_safety_config.rx_checks[i].status;
    valid = valid && status->msg_seen && !status->lagging && status->valid_checksum && status->valid_quality_flag &&
            (status->wrong_counters < MAX_WRONG_COUNTERS) &&
            (safety_get_ts_elapsed(now, status->last_timestamp) <= 150000U);
  }
  return valid;
}

static bool ford_lka_tx(const CANPacket_t *msg) {
  const unsigned int action = msg->data[0] >> 5;
  const int reference = (int)(((msg->data[2] & 15U) << 8) | msg->data[3]) - 2048;
  const int curvature = ((msg->data[1] << 4) | (msg->data[2] >> 4)) - 2048;
  const bool active = action != 0U;
  const uint32_t now = microsecond_timer_get();
  bool violation = (msg->data[0] & 31U) != 3U;  // no LDW request, ordinary intensity field
  violation |= (action != 0U) && (action != 2U) && (action != 4U);
  violation |= (msg->data[4] != 0U) || (msg->data[5] != 0U) || (msg->data[6] != 0U) || (msg->data[7] != 0U);
  violation |= safety_max_limit_check(reference, 128, -128);
  violation |= safety_max_limit_check(curvature, 300, -300);

  if (!active) {
    violation |= (reference != 0) || (curvature != 0);
  } else {
    const float speed = vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR;
    const int wheel_reference = ROUND(ford_lka_wheel_decideg / 0.4297F);
    const int previous_reference = ford_lka_active_last ? ford_lka_reference_last : wheel_reference;
    const int previous_curvature = ford_lka_active_last ? ford_lka_curvature_last : angle_meas.values[0] * 4;
    const int reference_decideg = ROUND(reference * 0.4297F);
    const int error = reference_decideg - ford_lka_wheel_decideg;
    violation |= !controls_allowed || !lat_control_allowed() || brake_pressed || gas_pressed;
    violation |= !ford_lka_available || !ford_lka_eps_ok || !ford_lka_inputs_fresh(now);
    violation |= (speed < 15.0F) || (speed > 30.0F);
    violation |= safety_max_limit_check(ford_lka_wheel_decideg, 55, -55);
    violation |= safety_max_limit_check(ford_lka_wheel_decideg - ford_lka_camera_decideg, 5, -5);
    violation |= safety_max_limit_check(ford_lka_torque_raw, 16, -16);
    violation |= safety_max_limit_check(error, 20, -20);
    // Allow a small direction deadband. A release/re-engage cannot seed a fake
    // reference: the first active packet is rate-limited from the measured wheel.
    violation |= ((action == 2U) && (error > 3)) || ((action == 4U) && (error < -3));
    violation |= safety_max_limit_check(reference, previous_reference + 2, previous_reference - 2);
    violation |= safety_max_limit_check(curvature, previous_curvature + 2, previous_curvature - 2);
    // 0x3CA curvature has four times as many counts/m^-1 as angle_meas here.
    violation |= safety_max_limit_check(curvature, (angle_meas.max * 4) + 200, (angle_meas.min * 4) - 200);
    if (ford_lka_active_last) {
      const uint32_t elapsed = safety_get_ts_elapsed(now, ford_lka_last_tx_ts);
      violation |= (elapsed < 20000U) || (elapsed > 100000U);
    }
  }
  // Never let a rejected packet alter the next packet's rate envelope.
  if (!violation) {
    ford_lka_active_last = active;
    ford_lka_reference_last = active ? reference : 0;
    ford_lka_curvature_last = active ? curvature : 0;
    ford_lka_last_tx_ts = now;
  }
  return !violation;
}
