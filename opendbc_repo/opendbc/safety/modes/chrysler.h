#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/chrysler_common.h"

// Chrysler Pacifica/Jeep addresses
#define CHRYSLER_EPS_2            0x220  // EPS driver input torque
#define CHRYSLER_ESP_1            0x140  // Brake pedal and vehicle speed
#define CHRYSLER_ESP_8            0x11C  // Brake pedal and vehicle speed
#define CHRYSLER_ECM_5            0x22F  // Throttle position sensor
#define CHRYSLER_DAS_3            0x1F4  // ACC engagement states from DASM
#define CHRYSLER_DAS_6            0x2A6  // LKAS HUD and auto headlight control from DASM
#define CHRYSLER_LKAS_COMMAND     0x292  // LKAS controls from DASM
#define CHRYSLER_CRUISE_BUTTONS   0x23B  // Cruise control buttons
#define CHRYSLER_LKAS_HEARTBIT    0x2D9
#define CHRYSLER_TRACTION_BUTTON  0x330

// RAM DT addresses
#define CHRYSLER_RAM_DT_EPS_2            0x31
#define CHRYSLER_RAM_DT_ESP_1            0x83
#define CHRYSLER_RAM_DT_ESP_8            0x79
#define CHRYSLER_RAM_DT_ECM_5            0x9D
#define CHRYSLER_RAM_DT_DAS_3            0x99
#define CHRYSLER_RAM_DT_DAS_6            0xFA
#define CHRYSLER_RAM_DT_LKAS_COMMAND     0xA6
#define CHRYSLER_RAM_DT_CRUISE_BUTTONS   0xB1

// RAM HD addresses
#define CHRYSLER_RAM_HD_EPS_2            0x220
#define CHRYSLER_RAM_HD_ESP_1            0x140
#define CHRYSLER_RAM_HD_ESP_8            0x11C
#define CHRYSLER_RAM_HD_ECM_5            0x22F
#define CHRYSLER_RAM_HD_DAS_3            0x1F4
#define CHRYSLER_RAM_HD_DAS_6            0x275
#define CHRYSLER_RAM_HD_LKAS_COMMAND     0x276
#define CHRYSLER_RAM_HD_CRUISE_BUTTONS   0x23A

typedef enum {
  CHRYSLER_RAM_DT,
  CHRYSLER_RAM_HD,
  CHRYSLER_PACIFICA,  // plus Jeep
} ChryslerPlatform;
static ChryslerPlatform chrysler_platform;
static bool chrysler_alka;
static bool chrysler_alka_button_prev;
static bool chrysler_alka_button_seen;
static bool chrysler_alka_heartbeat_seen;
static uint32_t chrysler_alka_button_ts;
static uint32_t chrysler_alka_heartbeat_ts;
static uint8_t chrysler_alka_heartbeat[5];
static bool chrysler_alka_above_min_speed;

#define CHRYSLER_ADDR(name) ((uint32_t)((chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_##name : \
                                        ((chrysler_platform == CHRYSLER_RAM_HD) ? CHRYSLER_RAM_HD_##name : CHRYSLER_##name)))


static uint8_t chrysler_get_counter(const CANPacket_t *msg) {
  return (uint8_t)(msg->data[6] >> 4);
}

static void chrysler_rx_hook(const CANPacket_t *msg) {
  // Measured EPS torque
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(EPS_2))) {
    int torque_meas_new = ((msg->data[4] & 0x7U) << 8) + msg->data[5] - 1024U;
    update_sample(&torque_meas, torque_meas_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  const unsigned int das_3_bus = (chrysler_platform == CHRYSLER_PACIFICA) ? 0U : 2U;
  if ((msg->bus == das_3_bus) && (msg->addr == CHRYSLER_ADDR(DAS_3))) {
    bool cruise_engaged = GET_BIT(msg, 21U);
    pcm_cruise_check(cruise_engaged);
    if (chrysler_alka) {
      const bool main_on = GET_BIT(msg, 20U);
      if (main_on && !acc_main_on) {
        lkas_on = true;
      }
      if (!main_on) {
        lkas_on = false;
      }
      acc_main_on = main_on;
    }
  }

  // TODO: use the same message for both
  // update vehicle moving
  if ((chrysler_platform != CHRYSLER_PACIFICA) && (msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(ESP_8))) {
    vehicle_moving = ((msg->data[4] << 8) + msg->data[5]) != 0U;
  }
  if ((chrysler_platform == CHRYSLER_PACIFICA) && (msg->bus == 0U) && (msg->addr == 514U)) {
    int speed_l = (msg->data[0] << 4) + (msg->data[1] >> 4);
    int speed_r = (msg->data[2] << 4) + (msg->data[3] >> 4);
    vehicle_moving = (speed_l != 0) || (speed_r != 0);
    // SPEED_1 is 0.071028 m/s per bit: 247 = 17.543916 m/s (>63 km/h).
    chrysler_alka_above_min_speed = (speed_l >= 247) && (speed_r >= 247);
  }

  // exit controls on rising edge of gas press
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(ECM_5))) {
    gas_pressed = msg->data[0U] != 0U;
  }

  // exit controls on rising edge of brake press
  if ((msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(ESP_1))) {
    brake_pressed = ((msg->data[0U] & 0xFU) >> 2U) == 1U;
  }
}

// The switch and camera heartbeat are only 1 Hz. Keep them out of RxCheck,
// which requires >=10 Hz in this dragonpilot base, and bound their age here.
// All steering-critical high-rate messages retain their checksum/counter checks.
static void chrysler_rx_ext(const CANPacket_t *msg) {
  if (chrysler_alka) {
    if ((msg->bus == 0U) && (msg->addr == CHRYSLER_TRACTION_BUTTON) && (GET_LEN(msg) == 8U)) {
      const bool pressed = GET_BIT(msg, 53U);
      if (pressed && !chrysler_alka_button_prev && acc_main_on) {
        lkas_on = !lkas_on;
      }
      chrysler_alka_button_prev = pressed;
      chrysler_alka_button_seen = true;
      chrysler_alka_button_ts = microsecond_timer_get();
    }
    if ((msg->bus == 2U) && (msg->addr == CHRYSLER_LKAS_HEARTBIT) && (GET_LEN(msg) == 5U)) {
      for (int i = 0; i < 5; i++) {
        chrysler_alka_heartbeat[i] = msg->data[i];
      }
      chrysler_alka_heartbeat_seen = true;
      chrysler_alka_heartbeat_ts = microsecond_timer_get();
    }
  }
}

static bool chrysler_alka_inputs_valid(void) {
  const uint32_t now = microsecond_timer_get();
  bool valid = chrysler_alka_button_seen && chrysler_alka_heartbeat_seen &&
               (safety_get_ts_elapsed(now, chrysler_alka_button_ts) <= 10000000U) &&
               (safety_get_ts_elapsed(now, chrysler_alka_heartbeat_ts) <= 10000000U);
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxStatus *status = &current_safety_config.rx_checks[i].status;
    valid = valid && status->msg_seen && !status->lagging && status->valid_checksum && status->valid_quality_flag &&
            (status->wrong_counters < MAX_WRONG_COUNTERS) && (safety_get_ts_elapsed(now, status->last_timestamp) <= 1000000U);
  }
  return valid;
}

static bool chrysler_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits CHRYSLER_STEERING_LIMITS = {
    .max_torque = 261,
    .max_rt_delta = 112,
    .max_rate_up = 3,
    .max_rate_down = 3,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  const TorqueSteeringLimits CHRYSLER_RAM_DT_STEERING_LIMITS = {
    .max_torque = 350,
    .max_rt_delta = 112,
    .max_rate_up = 6,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  const TorqueSteeringLimits CHRYSLER_RAM_HD_STEERING_LIMITS = {
    .max_torque = 361,
    .max_rt_delta = 182,
    .max_rate_up = 14,
    .max_rate_down = 14,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  // STEERING
  if (msg->addr == CHRYSLER_ADDR(LKAS_COMMAND)) {
    int start_byte = (chrysler_platform == CHRYSLER_PACIFICA) ? 0 : 1;
    int desired_torque = ((msg->data[start_byte] & 0x7U) << 8) | msg->data[start_byte + 1];
    desired_torque -= 1024;

    if (chrysler_alka && (desired_torque != 0)) {
      // ALKA is lateral-only: never turn controls_allowed on for this feature.
      // Gate normal ACC steering as well, so LKAS OFF remains authoritative.
      if (((alternative_experience & ALT_EXP_ALKA) == 0) || !lkas_on || !acc_main_on ||
          !chrysler_alka_above_min_speed || !chrysler_alka_inputs_valid()) {
        tx = false;
      }
    }

    const TorqueSteeringLimits limits = (chrysler_platform == CHRYSLER_PACIFICA) ? CHRYSLER_STEERING_LIMITS :
                                        (chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_STEERING_LIMITS : CHRYSLER_RAM_HD_STEERING_LIMITS;

    bool steer_req = (chrysler_platform == CHRYSLER_PACIFICA) ? GET_BIT(msg, 4U) : (msg->data[3] & 0x7U) == 2U;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
    }
  }

  if (chrysler_alka && (msg->addr == CHRYSLER_LKAS_HEARTBIT)) {
    bool preserved = chrysler_alka_heartbeat_seen;
    for (int i = 0; i < 5; i++) {
      const uint8_t mask = (i == 3) ? 0xFEU : 0xFFU;
      preserved = preserved && ((msg->data[i] & mask) == (chrysler_alka_heartbeat[i] & mask));
    }
    if (!preserved || (!GET_BIT(msg, 24U) && (!lkas_on || !chrysler_alka_inputs_valid()))) {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (msg->addr == CHRYSLER_ADDR(CRUISE_BUTTONS)) {
    const bool is_cancel = msg->data[0] == 1U;
    const bool is_resume = msg->data[0] == 0x10U;
    const bool allowed = is_cancel || (is_resume && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  return tx;
}

static safety_config chrysler_init(uint16_t param) {
  const uint32_t CHRYSLER_PARAM_RAM_DT = 1U;  // set for Ram DT platform

  static RxCheck chrysler_ram_dt_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_DT_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_rx_checks[] = {
    {.msg = {{CHRYSLER_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{514, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_DAS_3, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_TX_MSGS[] = {
    {CHRYSLER_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_LKAS_COMMAND, 0, 6, .check_relay = true},
    {CHRYSLER_DAS_6, 0, 8, .check_relay = true},
  };

  static const CanMsg CHRYSLER_ALKA_TX_MSGS[] = {
    {CHRYSLER_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_LKAS_COMMAND, 0, 6, .check_relay = true},
    {CHRYSLER_DAS_6, 0, 8, .check_relay = true},
    {CHRYSLER_LKAS_HEARTBIT, 0, 5, .check_relay = true},
  };

  static const CanMsg CHRYSLER_RAM_DT_TX_MSGS[] = {
    {CHRYSLER_RAM_DT_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {CHRYSLER_RAM_DT_LKAS_COMMAND, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_6, 0, 8, .check_relay = true},
  };

#ifdef ALLOW_DEBUG
  static RxCheck chrysler_ram_hd_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_HD_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_RAM_HD_TX_MSGS[] = {
    {CHRYSLER_RAM_HD_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {CHRYSLER_RAM_HD_LKAS_COMMAND, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_HD_DAS_6, 0, 8, .check_relay = true},
  };

  const uint32_t CHRYSLER_PARAM_RAM_HD = 2U;  // set for Ram HD platform
  bool enable_ram_hd = GET_FLAG(param, CHRYSLER_PARAM_RAM_HD);
#endif

  safety_config ret;

  bool enable_ram_dt = GET_FLAG(param, CHRYSLER_PARAM_RAM_DT);
  const uint32_t CHRYSLER_PARAM_PACIFICA_ALKA = 4U;
  chrysler_alka = false;
  chrysler_alka_button_prev = false;
  chrysler_alka_button_seen = false;
  chrysler_alka_heartbeat_seen = false;
  chrysler_alka_button_ts = 0U;
  chrysler_alka_heartbeat_ts = 0U;
  chrysler_alka_above_min_speed = false;

  if (enable_ram_dt) {
    chrysler_platform = CHRYSLER_RAM_DT;
    ret = BUILD_SAFETY_CFG(chrysler_ram_dt_rx_checks, CHRYSLER_RAM_DT_TX_MSGS);
#ifdef ALLOW_DEBUG
  } else if (enable_ram_hd) {
    chrysler_platform = CHRYSLER_RAM_HD;
    ret = BUILD_SAFETY_CFG(chrysler_ram_hd_rx_checks, CHRYSLER_RAM_HD_TX_MSGS);
#endif
  } else {
    chrysler_platform = CHRYSLER_PACIFICA;
    chrysler_alka = GET_FLAG(param, CHRYSLER_PARAM_PACIFICA_ALKA);
    alka_allowed = chrysler_alka;
    if (chrysler_alka) {
      ret = BUILD_SAFETY_CFG(chrysler_rx_checks, CHRYSLER_ALKA_TX_MSGS);
    } else {
      ret = BUILD_SAFETY_CFG(chrysler_rx_checks, CHRYSLER_TX_MSGS);
    }
  }
  return ret;
}

const safety_hooks chrysler_hooks = {
  .init = chrysler_init,
  .rx = chrysler_rx_hook,
  .rx_ext = chrysler_rx_ext,
  .tx = chrysler_tx_hook,
  .get_counter = chrysler_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};
